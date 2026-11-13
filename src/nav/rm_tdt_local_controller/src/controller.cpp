// T-DT 局部规划即控制器：
//   全局 A*（rm_tdt_planner）给路径 → 取前方 lookahead 处作局部目标
//   → 局部 KinodynamicAstar(4 m) 出带时间轨迹 → 按轨迹前馈 + 位姿反馈出速度
// 失败时回退：沿路径的纯跟踪（pure pursuit）。
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "rm_nav_interfaces/msg/stage_status.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kinodynamicAstar.hpp"
#include "rm_plan_msgs/msg/trajectory_plan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace rm_tdt_local_controller {

static double wrap(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
static double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

class TdtLocalController : public nav2_core::Controller {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    logger_ = node_->get_logger();
    auto d = [&](const std::string& k, auto def) { return node_->declare_parameter(name + "." + k, def); };
    lookahead_ = d("lookahead", 4.0);
    max_vel_ = d("max_vel", 1.5);
    max_acc_ = d("max_acc", 3.0);
    max_w_ = d("max_w", 3.0);
    k_theta_ = d("k_theta", 2.0);
    k_lat_ = d("k_lat", 1.0);
    max_nodes_ = d("max_nodes", 20000);
    sample_time_ = d("sample_time", 0.02);
    goal_tol_ = d("goal_tol", 0.15);
    min_local_goal_ = d("min_local_goal", 1.0);
    use_kastar_ = d("use_kastar", true);
    // ★★★ 2026-11-12（**回到 T-DT 原版结构**）：
    //   原版 `doc/Astar.md`「混合搜索」原文：
    //     "先使用 Astar 或 JPS 进行远距离搜索…再选取距离车辆例如 4 m 以外的
    //      路径点作为 Kastar 的局部规划目标"
    //   ⇒ **近场轨迹本来就该由 Kastar 生成**。我们之前只在"没有轨迹可跟"时才用
    //   Kastar（`!have_traj`），等于把它降级成兜底，与原版不符。
    //   `kastar_local: true` = 每周期都让 Kastar 在当前滑窗内解到"前方 lookahead 处"，
    //   其轨迹同时提供**近场几何与速度前馈**（原版就是这么用的）。
    //   ⚠ 已知代价（实测 2026-11-10）：Kastar 轨迹**末端速度恒为 0**（原版设计），
    //     靠"不断重规划、永远追前方 4 m 的停车点"来巡航 ⇒ 平均速度会明显低于
    //     MinSnap 巡航曲线（当时实测峰值 1.6~2.1、平均 ~0.6 m/s）。
    //     代码里已用"取未来 0.5 s 窗口内最大速度"来缓解。
    //   回退：kastar_local: false（= Kastar 仅在无轨迹时兜底，原行为）。
    use_kastar_local_ = d("kastar_local", true);
    plan_max_age_ = d("plan_max_age", 2.0);
    grid_window_ = d("grid_window", 16.0);
    // ★ 方向修正权重：0 = 完全按轨迹切线（最贴线）；越大越像纯跟踪（越抄近道）
    p_lat_blend_ = d("lat_blend", 0.35);
    // ★★ 2026-11-10：**朝向伺服增益**。0 = 不控制车头朝向（全向底盘的正常用法）
    //   之前两条路径都用 k_theta 把车头对准"路径切线/前视点"，导致 w 长期 ±3 饱和、
    //   车一边原地打转一边平移 ⇒ 螺旋轨迹 ⇒ 点云畸变 ⇒ 定位降级 → LOST。
    p_heading_gain_ = d("heading_gain", 0.0);
    // 速度查表前视（m）：避开每周期重规划的起步加速段
    p_v_look_ = d("v_lookahead", 1.5);        // s：前视时间常数
    p_la_min_ = d("lookahead_min", 1.0);      // m：前视下限（静止时也给 1 m，起步不呆）
    p_la_max_ = d("lookahead_max", 3.0);      // m：前视上限
    // 停止距离保护用的减速度（m/s²）：越大越晚开始减速
    p_stop_dec_ = d("stop_decel", 2.5);
    use_plan_track_ = d("use_plan_track", true);     // ★ 跟踪 T-DT 轨迹（用 vx/vy/时间）
    decel_ = d("track_decel", 1.0);                  // 末端减速加速度
    // ★ 2026-11-08：与 T-DT 一致 —— 默认用**整张图**做局部规划（不再裁窗口）
    use_full_map_ = d("use_full_map", true);
    zlio_odom_topic_ = d("zlio_odom_topic", std::string("/localization/odometry"));
    use_zlio_vel_ = d("use_zlio_vel", true);
    p_kastar_replan_dt_ = d("kastar_replan_dt", 0.5);
    p_kastar_dev_ = d("kastar_dev", 0.30);
    p_kastar_goal_move_ = d("kastar_goal_move", 0.50);
    pub_kastar_ = node_->create_publisher<nav_msgs::msg::Path>("/nav/l2/dbg/samples", 1);
    // ★★★ 2026-11-12：**K-star 滑窗边界可视化**。
    //   窗口由 buildLocalGrid(px,py) 按 `grid_window`（默认 6 m 半径 = 12x12 m）算出，
    //   之前**只用于搜索、完全不可见** ⇒ 无法判断"局部到底在多大范围里找"。
    //   用 Path（闭合成矩形）发布，RViz 用 Path 显示即可，不必新增 Marker 类型。
    pub_window_ = node_->create_publisher<nav_msgs::msg::Path>("/nav/l2/dbg/window", 1);
    // ★★★ 2026-11-12：**局部栅格可视化**。
    //   全局有 `/nav/l1/dbg/plan_grid`，而 K-star **实际搜索的那张滑窗栅格从来没发过**
    //   ⇒ "K-star 为什么走这条/为什么失败"只能靠猜。这里把它按 2 Hz 发成 OccupancyGrid。
    //   取值：gdata_ 内部 0=障碍、255=自由（buildLocalGrid 写入），这里转成
    //   OccupancyGrid 语义 100=障碍、0=自由。
    //   ★ 2026-11-12（晚）：**改发 PointCloud2**。原来发 OccupancyGrid，
    //     但 RViz 的 Map 显示只支持 map/costmap 两种配色，**没法指定颜色**，
    //     而且与实际障碍（黑）混在一起。改发"障碍格中心点云"后，
    //     RViz 里可用 FlatColor 指定任意颜色（现为青色），与全局图一眼可分。
    pub_lgrid_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/nav/l2/dbg/grid", 1);
    pub_st_l3_ = node_->create_publisher<rm_nav_interfaces::msg::StageStatus>("/nav/l3/status", 1);
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { map_ = m; map_ready_ = true;
          grid_dirty_ = true;
          map_frame_ = m->header.frame_id.empty() ? "map" : m->header.frame_id; });
    mask_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/plan_mask", rclcpp::QoS(1),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { mask_ = m; grid_dirty_ = true; });
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        zlio_odom_topic_, rclcpp::QoS(50),
        [this](nav_msgs::msg::Odometry::SharedPtr m) { zlio_odom_ = m; zlio_odom_stamp_ = node_->now(); });
    plan_sub_ = node_->create_subscription<rm_plan_msgs::msg::TrajectoryPlan>(
        "/trajectory_plan", rclcpp::QoS(1),
        [this](rm_plan_msgs::msg::TrajectoryPlan::SharedPtr m) { plan_ = m; });
    RCLCPP_INFO(logger_,
                "[tdt-local] configured: lookahead=%.1f vmax=%.2f amax=%.2f "
                "kastar=%d kastar_local=%d grid=%s window=%.1f",
                lookahead_, max_vel_, max_acc_, (int)use_kastar_,
                (int)use_kastar_local_, use_full_map_ ? "全图" : "滑窗", grid_window_);
  }
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setPlan(const nav_msgs::msg::Path& path) override { path_ = path; }
  void setSpeedLimit(const double& limit, const bool& percentage) override {
    max_vel_ = percentage ? std::max(0.1, 2.0 * limit / 100.0) : std::max(0.1, limit);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* /*goal_checker*/) override {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    cmd.header.frame_id = costmap_ros_->getBaseFrameID();
    // ★★★ 2026-11-13（**查"车为什么一直慢"用**）：每秒打一次收到的路径形态。
    //   实测线索：车 68% 的周期命令 <0.2 m/s，而控制器日志里
    //   `d_goal`（= 路径**末点**到车的距离）有 89% 的周期 <1 m，可真正目标平均在 4.69 m。
    //   两种可能都会造成这个现象，日志一眼可辨：
    //     A. nav2 把路径裁短了（末点就是前视点）⇒ 首点距车小、末点距车小
    //     B. 路径顺序是反的（back() 是离车最近的点）⇒ 首点距车大、末点距车≈0
    //   同时把"路径总长"打出来（用于判断"路径够不够长、值不值得加速"）。
    if (path_.poses.size() >= 2) {
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
          "[tdt-local] 收到路径: %zu 点 | 长 %.2f m | 首点距车 %.2f m | 末点距车 %.2f m | frame=%s",
          path_.poses.size(), [&] {
            double L = 0.0;
            for (size_t i = 1; i < path_.poses.size(); ++i) {
              const auto & a = path_.poses[i - 1].pose.position;
              const auto & b = path_.poses[i].pose.position;
              L += std::hypot(b.x - a.x, b.y - a.y);
            }
            return L;
          }(),
          [&] {
            const auto & a = path_.poses.front().pose.position;
            return std::hypot(a.x - pose.pose.position.x, a.y - pose.pose.position.y);
          }(),
          [&] {
            const auto & b = path_.poses.back().pose.position;
            return std::hypot(b.x - pose.pose.position.x, b.y - pose.pose.position.y);
          }(),
          path_.poses.front().header.frame_id.c_str());
    }
    if (path_.poses.size() < 2) {
      // ★ 2026-11-11：**这条 return 之前一直是静默的**——它就是"控制器一条指令都没发"
      //   的元凶之一，但日志里查不到任何痕迹。现在明确报出来。
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
          "[tdt-local] 提前返回：路径点只有 %zu 个（<2），无法跟踪 ⇒ 发零速", path_.poses.size());
      return cmd;
    }

    // 位姿差分测速（本栈 /odom twist 恒 0）
    const auto now = node_->now();
    const double px = pose.pose.position.x, py = pose.pose.position.y;
    const double yaw = tf2::getYaw(pose.pose.orientation);
    bool have_zlio = false;
    if (use_zlio_vel_ && zlio_odom_ && (now - zlio_odom_stamp_).seconds() < 0.5) {
      // zlio 的 twist 是**传感器系**速度 ⇒ 用当前航向转到世界系
      const auto& t = zlio_odom_->twist.twist.linear;
      if (std::isfinite(t.x) && std::isfinite(t.y) && std::hypot(t.x, t.y) > 1e-4) {
        vx_ = std::cos(yaw) * t.x - std::sin(yaw) * t.y;
        vy_ = std::sin(yaw) * t.x + std::cos(yaw) * t.y;
        have_zlio = true;
      }
    }
    if (!have_zlio && meas_valid_) {
      const double dt = (now - meas_time_).seconds();
      if (dt > 0.01 && dt < 0.5) {
        vx_ = 0.7 * (px - meas_x_) / dt + 0.3 * vx_;
        vy_ = 0.7 * (py - meas_y_) / dt + 0.3 * vy_;
      }
    }
    meas_x_ = px; meas_y_ = py; meas_time_ = now; meas_valid_ = true;

    // 投影到路径 → 前视点
    size_t best = 0; double bestd = 1e18;
    for (size_t i = 0; i < path_.poses.size(); ++i) {
      const auto& q = path_.poses[i].pose.position;
      const double d2 = (q.x - px) * (q.x - px) + (q.y - py) * (q.y - py);
      if (d2 < bestd) { bestd = d2; best = i; }
    }
    double s_acc = 0.0; size_t li = best;
    while (li + 1 < path_.poses.size()) {
      const auto& a = path_.poses[li].pose.position;
      const auto& b = path_.poses[li + 1].pose.position;
      s_acc += std::hypot(b.x - a.x, b.y - a.y);
      ++li;
      if (s_acc >= lookahead_) break;
    }
    geometry_msgs::msg::Point goal_pt = path_.poses[li].pose.position;
    // ★ 优先用 /trajectory_plan（全局的**时间轨迹 + 走廊**）：与 T-DT 单节点直传等价
    bool used_plan = false;
    if (plan_ && plan_->t.size() >= 2 && plan_->x.size() == plan_->t.size()) {
      const double age = (node_->now() - rclcpp::Time(plan_->header.stamp)).seconds();
      if (age >= -0.5 && age < plan_max_age_) {
        size_t pi = 0; double pd = 1e18;
        for (size_t i = 0; i < plan_->x.size(); ++i) {
          const double d = std::hypot(plan_->x[i] - px, plan_->y[i] - py);
          if (d < pd) { pd = d; pi = i; }
        }
        double s = 0.0; size_t gi = pi;
        while (gi + 1 < plan_->x.size()) {
          s += std::hypot(plan_->x[gi + 1] - plan_->x[gi], plan_->y[gi + 1] - plan_->y[gi]);
          ++gi;
          if (s >= lookahead_) break;
        }
        goal_pt.x = plan_->x[gi]; goal_pt.y = plan_->y[gi];
        // 走廊只用于**安全检查**：若目标落在退化/缺失走廊之外，不夹取（夹取会贴近障碍）
        used_plan = true;
      }
    }
    (void)used_plan;
    // ★★★ 2026-11-12（**实测**）：**把局部目标裁进 K-star 的搜索盒子**。
    //   为什么（日志证据，20:43 run）：
    //     `[tdt-local] kastar **失败**: 局部目标 8.86 m, 窗口 5.0 m, 样本 0`
    //   K-star 的搜索范围是**以车为中心、半径 grid_window_ 的盒子**（5.0 m ⇒ 10x10 m），
    //   而上面的 lookahead 选点逻辑是「沿路径累积到 >= lookahead(4 m) 的那个**拐点**」，
    //   **没有距离上限**：
    //     · MinSnap 成功时 /plan 每段 10 个插值点，间距 ~0.2 m ⇒ 目标 ≈ 4 m ✅
    //     · MinSnap 回退时 /plan = T-DT 简化后的路径，**只有 4~8 个点**
    //       （用 T-DT 自己的库实测：A* 915 点 → 简化 8 点，平均间距 6 m）
    //       ⇒ 下一个拐点可能在 8.86 m ⇒ **目标在盒子外** ⇒ K-star 必然返回「样本 0」
    //   ⇒ 这里把目标沿「机器人→目标」方向按比例收缩到 0.8*窗口（4.0 m）：
    //     · 方向不变（长线段本身就是路径，收缩后仍在路径上）
    //     · 只影响给 K-star 的**局部目标**，不改 /plan、不改 T-DT 算法与配置
    //   ⚠ 只对"目标远超窗口"这一类有确定性收益；3.9~4.4 m 的失败属于贴边/绕行/节点上限，
    //     不能用这一招解决（需要单独证据）。
    {
      const double k_cap = (grid_window_ > 0.0) ? 0.8 * grid_window_ : 1e9;
      const double dg = std::hypot(goal_pt.x - px, goal_pt.y - py);
      if (dg > k_cap && dg > 1e-6) {
        const double k = k_cap / dg;
        goal_pt.x = px + (goal_pt.x - px) * k;
        goal_pt.y = py + (goal_pt.y - py) * k;
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
            "[tdt-local] 局部目标裁剪: %.2f m → %.2f m（窗口 %.1f m 的 80%%）",
            dg, k_cap, grid_window_);
      }
    }
    const geometry_msgs::msg::Point& goal = goal_pt;
    const double dist_goal = std::hypot(goal.x - px, goal.y - py);

    bool have_traj = false;
    double v_world_x = 0.0, v_world_y = 0.0, w_cmd = 0.0;

    // ★★ 2026-11-08：**跟踪 T-DT 的轨迹**（把之前没用到的 vx/vy/时间用起来）。
    //   做法（标准"速度曲线 + 纯跟踪"组合）：
    //     ① 把 plan 建成 弧长表 s[i] 与 速度表 v[i]
    //     ② 车投影到 plan 上得到 s_now（单调，防止重规划时回跳）
    //     ③ 方向：前视点 s_now + La（La 随剩余长度收缩，短路径不会指到终点）
    //     ④ 速度：**用 MinSnap 优化出来的 v(s_now + Δ)**（末端速度=0 ⇒ 自带减速）
    //     ⑤ 航向：对准 "车→前视点" 的方向（闭环，天然纠横向误差）
    if (use_plan_track_ && plan_ && plan_->t.size() >= 2 && plan_->x.size() == plan_->t.size()) {
      const double age = (node_->now() - rclcpp::Time(plan_->header.stamp)).seconds();
      if (age >= -0.5 && age < plan_max_age_) {
        const size_t n = plan_->x.size();
        std::vector<double> sa(n, 0.0), va(n, 0.0);
        for (size_t i = 1; i < n; ++i)
          sa[i] = sa[i - 1] + std::hypot(plan_->x[i] - plan_->x[i - 1], plan_->y[i] - plan_->y[i - 1]);
        for (size_t i = 0; i < n; ++i)
          va[i] = std::hypot(plan_->vx.empty() ? 0.0 : plan_->vx[i],
                             plan_->vy.empty() ? 0.0 : plan_->vy[i]);
        const double s_total = sa.back();
        // ② 投影（单调：只允许前进，或小幅回退 0.3 m）
        size_t pi = 0; double pd = 1e18;
        for (size_t i = 0; i < n; ++i) {
          const double d = std::hypot(plan_->x[i] - px, plan_->y[i] - py);
          if (d < pd) { pd = d; pi = i; }
        }
        if (s_total > 1e-6) {
          if (pi < track_pi_ && sa[track_pi_] - sa[pi] > 0.3) pi = track_pi_;   // 防回跳
          track_pi_ = pi;
          const double s_now = sa[pi];
          // ③ 方向前视：La 随剩余长度收缩
          const double s_rem = std::max(0.0, s_total - s_now);
          // ★★★ 2026-11-13（**向 T-DT 对齐**）：**前视距离随速度自适应**。
          //   T-DT 官方控制器（`pb_omni_pid_pursuit_controller`）用的是
          //   `use_velocity_scaled_lookahead_dist: true` —— 前视随速度伸缩：
          //   慢的时候前视短（跟得紧、不切弯），快的时候前视长（稳）。
          //   我们原来是**固定 lookahead**，慢速时会"看着很远的地方" ⇒ 方向迟钝。
          //   现在：La = clamp(v_lookahead · **v_plan**(当前位置的规划速度),
          //                   lookahead_min, lookahead_max)，
          //   并继续用 `0.5·剩余长度` 封顶（短路径不会指到终点）。
          // ★★★ 2026-11-13（**改用它自己的规划速度，而不是实测速度**）：
          //   用实测速度会形成"速度→前视→方向→速度"的反馈环，而实测速度有噪声
          //   （实测见过 5 m/s 的里程计尖峰）⇒ 前视瞬间从 1 m 跳到 3 m ⇒ 车头甩一下。
          //   T-DT 官方控制器也是用**它自己想要的**速度来缩放前视，不是量出来的速度。
          //   做法：先插值出"**当前位置**的规划速度"（轨迹在这一点的速度），
          //        用它决定前视长度；参考点仍按 `0.5·剩余长度` 封顶。
          auto interp = [&](double s_target, double* ox, double* oy, double* ov) {
            size_t j = 0;
            while (j + 2 < n && sa[j + 1] < s_target) ++j;
            const double seg = std::max(1e-9, sa[j + 1] - sa[j]);
            const double u = std::min(1.0, std::max(0.0, (s_target - sa[j]) / seg));
            *ox = plan_->x[j] + u * (plan_->x[j + 1] - plan_->x[j]);
            *oy = plan_->y[j] + u * (plan_->y[j + 1] - plan_->y[j]);
            *ov = va[j] + u * (va[j + 1] - va[j]);
          };
          // 规划速度（当前位置处）⇒ 前视长度
          double v_plan_here = 0.0;
          {
            double tx = 0.0, ty = 0.0;
            interp(s_now, &tx, &ty, &v_plan_here);
          }
          const double La_dyn = std::min(p_la_max_, std::max(p_la_min_, p_v_look_ * v_plan_here));
          const double La = std::min(La_dyn, std::max(0.6, 0.5 * s_rem));
          double dx_pt, dy_pt, v_dir;
          interp(std::min(s_total, s_now + La), &dx_pt, &dy_pt, &v_dir);
          double ex = dx_pt - px, ey = dy_pt - py;
          const double el = std::hypot(ex, ey);
          if (el > 1e-3) {
            ex /= el; ey /= el;
            // ★★ 2026-11-10：**化简为纯追踪（pure pursuit）**——速度与方向取自**同一个点**。
            //   之前速度来自 s+V、方向来自切线、再混 60% 前视 ⇒ 三个源，
            //   一旦不一致就"速度对、方向错"（实测 cmd 方向与目标方位差 72°）。
            //   现在：唯一参考点 P = 路径上 s_now + La 处
            //        方向 = 指向 P ；速度 = P 处的 v（规划器的巡航速度）
            //   两者永远指向同一点，不可能撕裂。w 恒 0（全向底盘）。
            // ★★★ 2026-11-13（**实测定位到的限速原因**）：**参考点不能取到轨迹末端**。
            //   实测（`收到路径` 日志，403 条）：路径长 均 5.41 m、**36% 短于 2 m**，
            //   而 `lookahead: 4.0` ⇒ `s_now + La` 经常超过路径长度，
            //   参考点被钉在**末端**；而 T-DT 轨迹**末端速度≈0**（它按"到点停车"分配时间）
            //   ⇒ `v_at_ref≈0` ⇒ 命令速度被压到 0.2 m/s 以下
            //   （实测：命令 均 0.40 m/s、**62% 的周期 <0.2 m/s**，而 v_ref 均值 1.38）。
            //   修法：参考点最多取到"末端前 0.35 m"，且不倒退。
            const double s_end = std::max(0.0, s_total - 0.35);
            const double s_ref = std::max(s_now, std::min(s_end, s_now + La));
            double px_ref = px, py_ref = py, v_at_ref = 0.0;
            interp(s_ref, &px_ref, &py_ref, &v_at_ref);
            double dx_dir = px_ref - px, dy_dir = py_ref - py;
            const double dl = std::hypot(dx_dir, dy_dir);
            if (dl < 1e-3) { dx_dir = ex; dy_dir = ey; }
            else           { dx_dir /= dl; dy_dir /= dl; }
            // 速度：同一个 P 处的规划速度，末端按停止距离收（只在最后 1~2 m 生效）
            const double d_goal_now = std::hypot(plan_->x.back() - px, plan_->y.back() - py);
            double v_ref = std::min(v_at_ref, max_vel_);
            // ★★ 2026-11-11：**停车阈值必须小于 goal checker 的容差**。
            //   原来在 0.30 m 就停，而 PlanGoalChecker 的 xy_tol 是 0.25 m
            //   ⇒ 车稳稳停在"差 5 cm"的位置、速度恒 0、永远判未到达
            //   ⇒ 表现为"最后一点路径走不动"。收紧到 0.12 m。
            if (d_goal_now < 0.12) {
              v_ref = 0.0;                                   // 到点：停
            } else {
              const double v_stop = std::sqrt(2.0 * p_stop_dec_ * std::max(0.0, d_goal_now - 0.15));
              v_ref = std::min(v_ref, std::max(v_stop, 0.25));
            }
            v_world_x = v_ref * dx_dir;
            v_world_y = v_ref * dy_dir;
            // 残余的"反方向"保险（定位跳变时目标在身后）
            {
              const double gx = plan_->x.back() - px, gy = plan_->y.back() - py;
              const double gl = std::hypot(gx, gy);
              if (gl > 1e-3 && v_ref > 1e-3) {
                const double cd = (v_world_x * gx + v_world_y * gy) / (v_ref * gl);
                if (cd < -0.17) {
                  v_world_x = v_ref * gx / gl;
                  v_world_y = v_ref * gy / gl;
                  RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                      "[tdt-local] 方向与目标夹角 >100° ⇒ 改按目标方位走");
                }
              }
            }
            const double yaw_ref = std::atan2(ey, ex);
            dbg_yaw_ref_ = yaw_ref; dbg_s_ = s_now; dbg_v_ref_ = v_ref;
            w_cmd = clampd(p_heading_gain_ * wrap(yaw_ref - yaw), -max_w_, max_w_);
            have_traj = true;
            ++track_ok_;
            RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
                "[tdt-local] track: s=%.2f/%.2f v=%.2f La=%.2f 剩 %.2f m",
                s_now, s_total, v_ref, La, s_rem);
          }
        }
      }
    }

    const bool grid_ok = use_full_map_ ? (map_ready_ ? buildFullGrid() : buildGrid())
                                       : (map_ready_ ? buildLocalGrid(px, py) : buildGrid());
    // ★ kastar_local=true 时即使已有轨迹也走 Kastar（原版语义：Kastar 是局部规划器）
    if ((!have_traj || use_kastar_local_) && use_kastar_ &&
        dist_goal > std::max(goal_tol_, min_local_goal_) && grid_ok) {
      // 局部目标在 costmap 帧（与栅格同帧）
      geometry_msgs::msg::PoseStamped p_cf = pose, g_cf;
      g_cf.header = path_.poses[li].header;
      g_cf.pose.position = goal;
      const std::string cf = map_ready_ ? map_frame_ : costmap_ros_->getGlobalFrameID();
      if (!map_ready_) {
        try {
          if (pose.header.frame_id != cf) tf_->transform(pose, p_cf, cf, tf2::durationFromSec(0.1));
          if (g_cf.header.frame_id != cf) tf_->transform(g_cf, g_cf, cf, tf2::durationFromSec(0.1));
        } catch (const std::exception & e) {
          // ★ 2026-11-12：原来这里是**空 catch**，TF 失败完全静默。
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
              "[tdt-local] 位姿/目标转帧失败 ⇒ fallback：%s", e.what());
          return fallback(cmd, px, py, yaw, best, li);
        }
      }

      // ★ 局部目标可能落在障碍格（先验图/Mask）⇒ 沿路径回退到最近的可通行点
      {
        for (int tries = 0; tries < 20 && li > best; ++tries) {
          const double gxx = g_cf.pose.position.x, gyy = g_cf.pose.position.y;
          const int mx = static_cast<int>((gxx - gox_) / gres_);
          const int my = static_cast<int>((gyy - goy_) / gres_);
          const int idx = my * gw_ + mx;
          if (mx >= 0 && my >= 0 && mx < gw_ && my < gh_ && gdata_[idx] != 0) break;
          --li;
          g_cf.pose.position = path_.poses[li].pose.position;
        }
      }
      // ── 先判断**是否需要重解**（T-DT 语义：Kastar 是局部规划器，不是控制器）──
      const auto tnow_k = node_->now();
      bool k_need = k_cache_.empty() || (tnow_k - k_stamp_).seconds() > p_kastar_replan_dt_ ||
                    std::hypot(g_cf.pose.position.x - k_goal_x_,
                               g_cf.pose.position.y - k_goal_y_) > p_kastar_goal_move_;
      if (!k_need) {
        // 偏离缓存轨迹太远 ⇒ 重解
        double best_d2 = 1e18;
        for (const auto& q : k_cache_) {
          const double dx = q.x - p_cf.pose.position.x, dy = q.y - p_cf.pose.position.y;
          best_d2 = std::min(best_d2, dx * dx + dy * dy);
        }
        if (best_d2 > p_kastar_dev_ * p_kastar_dev_) k_need = true;
      }

      if (!k_need) {
        // ── 跟踪缓存：按时间推进，插值取 (x,y,vx,vy) ──
        const double tc = (tnow_k - k_stamp_).seconds();
        size_t j = 0;
        while (j + 2 < k_cache_.size() && k_cache_[j + 1].t < tc) ++j;
        const double t0 = k_cache_[j].t, t1 = k_cache_[j + 1].t;
        const double u = (t1 - t0 > 1e-9)
                             ? std::min(1.0, std::max(0.0, (tc - t0) / (t1 - t0)))
                             : 0.0;
        // ★★ 2026-11-10：Kastar 的轨迹**终点速度恒为 0**（T-DT doc/Astar.md），
        //   而我们的局部目标是前方 4~5 m 的点、不是真终点 ⇒ 它在 4 m 处"停车"。
        //   直接读当前时刻 v(t)：t≈起点 ⇒ 读到加速段的 0.05 m/s ⇒ 车越走越慢。
        //   改为：取未来 0.5 s 窗口内的**最大**速度（跳过加速段、也避开末段减速）。
        const double t_end = k_cache_.back().t;
        double vmax2 = 0.0, bestvx = k_cache_[j].vx, bestvy = k_cache_[j].vy;
        for (size_t q = j; q < k_cache_.size(); ++q) {
          if (k_cache_[q].t > tc + 0.5 && q > j) break;
          const double m2 = k_cache_[q].vx * k_cache_[q].vx + k_cache_[q].vy * k_cache_[q].vy;
          if (m2 > vmax2) { vmax2 = m2; bestvx = k_cache_[q].vx; bestvy = k_cache_[q].vy; }
        }
        (void)t_end;
        const auto& a = k_cache_[j];
        const auto& b = k_cache_[j + 1];
        // 方向仍用当前段插值（连续），大小用窗口最大值
        const double cur_vx = a.vx + u * (b.vx - a.vx);
        const double cur_vy = a.vy + u * (b.vy - a.vy);
        const double cur_n = std::hypot(cur_vx, cur_vy);
        const double best_n = std::sqrt(vmax2);
        if (cur_n > 1e-3 && best_n > 1e-3) {
          v_world_x = cur_vx / cur_n * best_n;
          v_world_y = cur_vy / cur_n * best_n;
        } else {
          v_world_x = bestvx; v_world_y = bestvy;
        }
        const double vn = std::hypot(v_world_x, v_world_y);
        if (vn > max_vel_ && vn > 1e-6) {
          v_world_x *= max_vel_ / vn;
          v_world_y *= max_vel_ / vn;
        }
        w_cmd = 0.0;
        have_traj = true;
        ++kastar_ok_;
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
            "[tdt-local] kastar 跟踪缓存: t=%.2f/%zu 点 前馈 v=%.2f（未重解）| 实测速度 v_meas=%.2f",
            tc, k_cache_.size(), std::hypot(v_world_x, v_world_y),
            std::hypot(vx_, vy_));
        // 发布缓存轨迹（可视化保持一致）
        if (pub_kastar_) {
          nav_msgs::msg::Path kp;
          kp.header.stamp = tnow_k;
          kp.header.frame_id = cf;
          for (const auto& q : k_cache_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = kp.header;
            ps.pose.position.x = q.x;
            ps.pose.position.y = q.y;
            ps.pose.position.z = 0.02;
            ps.pose.orientation.w = 1.0;
            kp.poses.push_back(ps);
          }
          pub_kastar_->publish(kp);
        }
      } else {

      KinodynamicAstar kastar(gw_, gh_, gres_, gox_, goy_);
      kastar.setMap(gw_, gh_, gdata_.data());
      kastar.config.maxNodes = max_nodes_;
      kastar.config.sampleTime = sample_time_;
      kastar.config.maxVel = max_vel_;
      kastar.config.maxAcc = max_acc_;
      Eigen::Vector2d s(p_cf.pose.position.x, p_cf.pose.position.y);
      Eigen::Vector2d g(g_cf.pose.position.x, g_cf.pose.position.y);
      const double yaw_cf = tf2::getYaw(p_cf.pose.orientation);
      // ★ vx_/vy_ 已经是世界（map）系速度（上面用 yaw 从传感器系转过来了）⇒ 直接用
      Eigen::Vector2d v0(vx_, vy_);
      auto res = kastar.search(s, v0, Eigen::Vector2d(0, 0), g);
      if (!res.success) {
        ++kastar_fail_;
        // ★★★ 2026-11-12：**原来失败只加计数、没有任何日志** ⇒ 车停在半路时
        //   无法判断"是 K-star 解不出来，还是别的环节"。这里把关键上下文打出来：
        //   局部目标多远、窗口多大、解出的样本数（0 = 搜索失败/超节点上限）
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
            "[tdt-local] kastar **失败**: 局部目标 %.2f m, 窗口 %.1f m, 样本 %zu"
            "（累计 fail=%d）⇒ 本周期退化为 fallback",
            dist_goal, grid_window_, res.trajectory.size(), kastar_fail_);
      }
      if (res.success && res.trajectory.size() > 2) {
        {
          nav_msgs::msg::Path kp;
          kp.header.stamp = node_->now();
          kp.header.frame_id = cf;
          kp.poses.reserve(res.trajectory.size());
          for (const auto& smp : res.trajectory) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = kp.header;
            ps.pose.position.x = smp.state.x();
            ps.pose.position.y = smp.state.y();
            ps.pose.position.z = 0.02;
            ps.pose.orientation.w = 1.0;
            kp.poses.push_back(ps);
          }
          if (pub_kastar_) pub_kastar_->publish(kp);
        }
        // 取离机器人最近的样本，用它后面第 2 个的速度作前馈
        size_t nearest = 0; double nd = 1e18;
        for (size_t i = 0; i < res.trajectory.size(); ++i) {
          const auto& st = res.trajectory[i].state;
          const double d2 = (st.x() - s.x()) * (st.x() - s.x()) + (st.y() - s.y()) * (st.y() - s.y());
          if (d2 < nd) { nd = d2; nearest = i; }
        }
        const size_t idx = std::min(res.trajectory.size() - 1, nearest + 2);
        const auto& st = res.trajectory[idx].state;
        const double yaw_ref = std::atan2(st.y() - s.y(), st.x() - s.x());
        // ★★ 2026-11-08（修双重旋转）：Kastar 的输出速度**已经是栅格帧=map 帧**，
        //   这里再乘一次 R(yaw_cf) 属于重复旋转；后面输出又乘 R(-yaw) 把它抵消成
        //   "把 map 系速度当车体系分量"，导致**车头转多少、实际速度方向就多偏多少**
        //   ⇒ 车头转到 ~104° 时实际往 208° 走（远离目标）。这里直接取原值。
        v_world_x = st(2);
        v_world_y = st(3);
        w_cmd = clampd(p_heading_gain_ * wrap(yaw_ref - yaw_cf), -max_w_, max_w_);
        have_traj = true;
        ++kastar_ok_;
        // ★ 存下**整条**轨迹（此前只取一点的速度，其余丢弃 ⇒ 每周期都要重解）
        k_cache_.clear();
        k_cache_.reserve(res.trajectory.size());
        for (const auto& smp : res.trajectory) {
          k_cache_.push_back({smp.state.x(), smp.state.y(),
                              smp.state(2), smp.state(3), smp.time});
        }
        k_stamp_ = tnow_k;
        k_goal_x_ = g_cf.pose.position.x;
        k_goal_y_ = g_cf.pose.position.y;
        // ★ 诊断：把 Kastar 轨迹本身的速度能力打出来，用来判断"慢"是
        //   ①它真的只解出这么慢（峰值低），还是 ②我们读的位置不对。
        {
          double peak = 0.0;
          for (const auto& smp : res.trajectory)
            peak = std::max(peak, std::hypot(smp.state(2), smp.state(3)));
          const double dur = res.trajectory.back().time;
          RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
              "[tdt-local] kastar ok: %zu 样本, 局部目标 %.1f m, 前馈 v=%.2f | 轨迹 峰值 %.2f 时长 %.2f s 平均 %.2f m/s, v0=%.2f",
              res.trajectory.size(), dist_goal, std::hypot(v_world_x, v_world_y),
              peak, dur, dur > 1e-6 ? dist_goal / dur : 0.0, std::hypot(v0.x(), v0.y()));
        }
      }
      }   // ← 结束"需要重解"分支
    }

    if (!have_traj) return fallback(cmd, px, py, yaw, best, li);

    // 车体系输出 + 横向误差反馈
    const double vx_b = std::cos(yaw) * v_world_x + std::sin(yaw) * v_world_y;
    const double vy_b = -std::sin(yaw) * v_world_x + std::cos(yaw) * v_world_y;
    if (pub_st_l3_) {
      rm_nav_interfaces::msg::StageStatus st;
      st.header.stamp = node_->now();
      st.stage = 3;
      st.state = have_traj ? 1 : 3;
      st.detail = have_traj ? (track_ok_ ? "track" : "kastar") : "no trajectory ⇒ fallback";
      // 注意：track_ok_ / kastar_ok_ 是**累计计数**，不是布尔。
      // 之前把它们塞进 track/src 里，监视器显示成 track=1.8e3，读不懂；
      // 现在明确分开：track_mode=0/1 表示当前是否在跟踪，ok/fail 是累计次数。
      st.keys.resize(3); st.values.resize(3);
      st.keys[0]="v"; st.keys[1]="track_mode"; st.keys[2]="kastar_ok";
      st.values[0]=std::hypot(v_world_x, v_world_y);
      st.values[1]=static_cast<double>(have_traj ? 1 : 0);
      st.values[2]=static_cast<double>(kastar_ok_);
      pub_st_l3_->publish(st);
    }
    cmd.twist.linear.x = clampd(vx_b, -max_vel_, max_vel_);
    cmd.twist.linear.y = clampd(vy_b, -max_vel_, max_vel_);
    cmd.twist.angular.z = w_cmd;
    // ★ 诊断：实际下发的车体指令 + 朝向/距离（每 2 s）
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
        "[tdt-local] cmd: vx=%+.2f vy=%+.2f w=%+.2f | yaw=%.2f yaw_ref=%.2f d_goal=%.2f s=%.2f v_ref=%.2f track=%d",
        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z,
        yaw, dbg_yaw_ref_, dist_goal, dbg_s_, dbg_v_ref_, (int)have_traj);
    return cmd;
  }

 private:
  // 回退：纯跟踪（沿路径速度 + 前视航向）
  geometry_msgs::msg::TwistStamped& fallback(geometry_msgs::msg::TwistStamped& cmd, double px, double py,
                                             double yaw, size_t best, size_t li) {
    ++fallback_;
    // ★ 2026-11-12：fallback 是**全速冲向路径点**（不是停车），但原来只在每 50 次时
    //   打一个统计 ⇒ 看到车猛冲却不知道为什么。这里明确说明触发原因。
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "[tdt-local] fallback: 无可用轨迹（track 与 kastar 都不可用）⇒ 朝路径点 #%zu "
        "全速 %.1f m/s（累计 fallback=%d）", li, max_vel_, fallback_);
    const auto& g = path_.poses[li].pose.position;
    const double yaw_ref = std::atan2(g.y - py, g.x - px);
    const double v = clampd(0.6 * max_vel_ + 0.4 * max_vel_, 0.0, max_vel_);
    cmd.twist.linear.x = v * std::cos(wrap(yaw_ref - yaw));
    cmd.twist.linear.y = v * std::sin(wrap(yaw_ref - yaw));
    cmd.twist.angular.z = clampd(k_theta_ * wrap(yaw_ref - yaw), -max_w_, max_w_);
    (void)best;
    if ((kastar_ok_ + kastar_fail_ + fallback_) % 50 == 0)
      RCLCPP_INFO(logger_, "[tdt-local] 统计: kastar_ok=%d kastar_fail=%d fallback=%d",
                  kastar_ok_, kastar_fail_, fallback_);
    return cmd;
  }

  // ★ 2026-11-08：**整张图**建 Kastar 栅格（与 T-DT 的整图一致）。
  //   原点固定 = (map 原点 - P*res)，不再随机器人移动 ⇒ 没有"每周期重锚"的抖动。
  //   仅在 /map 或 /plan_mask 更新后重建（dirty 标记）。
  bool buildFullGrid() {
    if (!map_ || map_->info.width == 0) return false;
    if (!grid_dirty_ && gw_ > 0 && gh_ > 0) return true;          // 缓存命中
    const double res = map_->info.resolution;
    const double ox = map_->info.origin.position.x, oy = map_->info.origin.position.y;
    const int W = static_cast<int>(map_->info.width), H = static_cast<int>(map_->info.height);
    const int P = 2;
    gw_ = W + 2 * P; gh_ = H + 2 * P;
    gres_ = static_cast<float>(res);
    gox_ = static_cast<float>(ox) - P * gres_;
    goy_ = static_cast<float>(oy) - P * gres_;
    gdata_.assign(static_cast<size_t>(gw_) * gh_, 0);             // 边框=障碍
    const bool mask_ok = mask_ && mask_->info.resolution == res &&
                         std::abs(mask_->info.origin.position.x - ox) < 1e-6 &&
                         std::abs(mask_->info.origin.position.y - oy) < 1e-6 &&
                         mask_->info.width == W && mask_->info.height == H;
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        const size_t si = static_cast<size_t>(y) * W + x;
        bool occ = (map_->data[si] >= 65);   // OccupancyGrid: 0=free, 100=occupied
        if (!occ && mask_ok) occ = mask_->data[si] > 50;
        gdata_[static_cast<size_t>(y + P) * gw_ + (x + P)] = occ ? 0 : 255;
      }
    grid_dirty_ = false;
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000,
        "[tdt-local] 整图栅格: %dx%d(+pad2) @ %.3f, origin(%.2f,%.2f)",
        gw_, gh_, gres_, gox_, goy_);
    return true;
  }

  // 以机器人为中心裁 window（m），叠加 /plan_mask，外扩 2 格边框
  bool buildLocalGrid(double cx, double cy) {
    if (!map_ || map_->info.width == 0) return false;
    const double res = map_->info.resolution;
    const double ox = map_->info.origin.position.x, oy = map_->info.origin.position.y;
    const int W = static_cast<int>(map_->info.width), H = static_cast<int>(map_->info.height);
    const int P = 2;
    int x0 = static_cast<int>((cx - grid_window_ - ox) / res);
    int y0 = static_cast<int>((cy - grid_window_ - oy) / res);
    int x1 = static_cast<int>((cx + grid_window_ - ox) / res);
    int y1 = static_cast<int>((cy + grid_window_ - oy) / res);
    x0 = std::max(0, x0); y0 = std::max(0, y0);
    x1 = std::min(W - 1, x1); y1 = std::min(H - 1, y1);
    if (x1 - x0 < 8 || y1 - y0 < 8) return false;
    gw_ = (x1 - x0 + 1) + 2 * P; gh_ = (y1 - y0 + 1) + 2 * P;
    gres_ = static_cast<float>(res);
    gox_ = static_cast<float>(ox + x0 * res) - P * static_cast<float>(res);
    goy_ = static_cast<float>(oy + y0 * res) - P * static_cast<float>(res);
    gdata_.assign(static_cast<size_t>(gw_) * gh_, 0);
    // Mask 是否同图同分辨率
    const bool mask_ok = mask_ && mask_->info.resolution == map_->info.resolution &&
                         std::abs(mask_->info.origin.position.x - ox) < 1e-6 &&
                         std::abs(mask_->info.origin.position.y - oy) < 1e-6 &&
                         mask_->info.width == map_->info.width && mask_->info.height == map_->info.height;
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) {
        const size_t si = static_cast<size_t>(y) * W + x;
        bool occ = (map_->data[si] >= 65);   // OccupancyGrid: 0=free, 100=occupied
        if (!occ && mask_ok) occ = mask_->data[si] > 50;   // ★ 动态障碍叠加
        gdata_[static_cast<size_t>(y - y0 + P) * gw_ + (x - x0 + P)] = occ ? 0 : 255;
      }

    // ★ 滑窗边界（2 Hz 限频；每周期都发会刷屏，可视化不需要 30 Hz）
    if (pub_window_) {
      const auto tnow_w = node_->now();
      if (!win_stamp_valid_ || (tnow_w - win_stamp_).seconds() > 0.5) {
        win_stamp_ = tnow_w; win_stamp_valid_ = true;
        const double wx0 = ox + x0 * res, wy0 = oy + y0 * res;
        const double wx1 = ox + x1 * res, wy1 = oy + y1 * res;
        nav_msgs::msg::Path wp;
        wp.header.stamp = tnow_w;
        wp.header.frame_id = map_frame_;
        const double cx[5] = {wx0, wx1, wx1, wx0, wx0};
        const double cy[5] = {wy0, wy0, wy1, wy1, wy0};
        for (int i = 0; i < 5; ++i) {
          geometry_msgs::msg::PoseStamped ps;
          ps.header = wp.header;
          ps.pose.position.x = cx[i];
          ps.pose.position.y = cy[i];
          ps.pose.orientation.w = 1.0;
          wp.poses.push_back(ps);
        }
        pub_window_->publish(wp);
      }
    }

    // ★ 局部栅格（K-star 实际搜索的那张图），2 Hz 限频
    if (pub_lgrid_) {
      const auto tnow_g = node_->now();
      if (!lgrid_stamp_valid_ || (tnow_g - lgrid_stamp_).seconds() > 0.5) {
        lgrid_stamp_ = tnow_g; lgrid_stamp_valid_ = true;
        sensor_msgs::msg::PointCloud2 pc;
        pc.header.stamp = tnow_g;
        pc.header.frame_id = map_frame_;
        pc.height = 1;
        pc.is_dense = true;
        pc.is_bigendian = false;
        sensor_msgs::msg::PointField fx, fy, fz;
        fx.name = "x"; fx.offset = 0;  fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
        fy.name = "y"; fy.offset = 4;  fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
        fz.name = "z"; fz.offset = 8;  fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
        pc.fields = {fx, fy, fz};
        pc.point_step = 12;
        // 只发**障碍格**（K-star 眼里的墙）；自由格不发 ⇒ 点数少、传输便宜
        std::vector<float> buf;
        buf.reserve(4096);
        for (int gy = 0; gy < gh_; ++gy) {
          for (int gx = 0; gx < gw_; ++gx) {
            if (gdata_[static_cast<size_t>(gy) * gw_ + gx] != 0) continue;
            buf.push_back(gox_ + (gx + 0.5f) * gres_);
            buf.push_back(goy_ + (gy + 0.5f) * gres_);
            buf.push_back(0.02f);
          }
        }
        pc.width = static_cast<uint32_t>(buf.size() / 3);
        pc.row_step = pc.point_step * pc.width;
        pc.data.resize(buf.size() * sizeof(float));
        std::memcpy(pc.data.data(), buf.data(), pc.data.size());
        pub_lgrid_->publish(pc);
      }
    }
    return true;
  }

  // 从先验图 /map 建 Kastar 栅格（map 帧，0=障碍/255=自由，pad 2 格）
  void buildGridFromMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& m) {
    if (!m || m->info.width == 0 || m->info.height == 0) return;
    const int P = 2;
    const int w = static_cast<int>(m->info.width), h = static_cast<int>(m->info.height);
    gw_ = w + 2 * P; gh_ = h + 2 * P;
    gres_ = static_cast<float>(m->info.resolution);
    gox_ = static_cast<float>(m->info.origin.position.x) - P * gres_;
    goy_ = static_cast<float>(m->info.origin.position.y) - P * gres_;
    map_frame_ = m->header.frame_id.empty() ? "map" : m->header.frame_id;
    gdata_.assign(static_cast<size_t>(gw_) * gh_, 0);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const int8_t v = m->data[static_cast<size_t>(y) * w + x];
        gdata_[static_cast<size_t>(y + P) * gw_ + (x + P)] = (v >= 65) ? 0 : 255;
      }
    map_ready_ = true;
  }

  // 从局部 costmap 建 Kastar 栅格（0=障碍，255=自由），外扩 2 格障碍边框
  bool buildGrid() {
    auto* cm = costmap_ros_->getCostmap();
    const int W = static_cast<int>(cm->getSizeInCellsX());
    const int H = static_cast<int>(cm->getSizeInCellsY());
    if (W < 4 || H < 4) return false;
    const int P = 2;
    gw_ = W + 2 * P; gh_ = H + 2 * P;
    gres_ = cm->getResolution();
    gox_ = cm->getOriginX() - P * gres_;
    goy_ = cm->getOriginY() - P * gres_;
    gdata_.assign(static_cast<size_t>(gw_) * gh_, 0);          // 边框=障碍
    const unsigned char* c = cm->getCharMap();
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        const unsigned char v = c[y * W + x];
        gdata_[static_cast<size_t>(y + P) * gw_ + (x + P)] =
            (v == nav2_costmap_2d::LETHAL_OBSTACLE || v == nav2_costmap_2d::NO_INFORMATION) ? 0 : 255;
      }
    return true;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("tdt_local")};
  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  nav_msgs::msg::Path path_;
  double lookahead_ = 4.0, max_vel_ = 1.5, max_acc_ = 3.0, max_w_ = 3.0, min_local_goal_ = 1.0;
  double k_theta_ = 2.0, k_lat_ = 1.0, sample_time_ = 0.02, goal_tol_ = 0.15;
  int max_nodes_ = 20000;
  bool use_kastar_ = true;
  double plan_max_age_ = 2.0;
  std::string zlio_odom_topic_ = "/localization/odometry";
  bool use_zlio_vel_ = true;
  nav_msgs::msg::Odometry::SharedPtr zlio_odom_;
  rclcpp::Time zlio_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rm_plan_msgs::msg::TrajectoryPlan>::SharedPtr plan_sub_;
  rm_plan_msgs::msg::TrajectoryPlan::SharedPtr plan_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_, mask_;
  double grid_window_ = 16.0;
  bool map_ready_ = false;
  double dbg_yaw_ref_ = 0.0, dbg_s_ = 0.0, dbg_v_ref_ = 0.0;
  bool use_plan_track_ = true;
  double p_lat_blend_ = 0.35;
  double p_v_look_ = 1.5, p_stop_dec_ = 2.5;
  double p_la_min_ = 1.0, p_la_max_ = 3.0;   // 自适应前视的夹取范围（m）
  double p_heading_gain_ = 0.0;
  double decel_ = 1.0;
  size_t track_pi_ = 0;
  int track_ok_ = 0;
  bool use_full_map_ = true;
  bool use_kastar_local_ = true;   // true=Kastar 主导近场（T-DT 原版结构）
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_window_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lgrid_;
  rclcpp::Time win_stamp_{0, 0, RCL_ROS_TIME};   // 注意：必须 RCL_ROS_TIME，否则相减会抛异常
  bool win_stamp_valid_ = false;
  rclcpp::Time lgrid_stamp_{0, 0, RCL_ROS_TIME};
  bool lgrid_stamp_valid_ = false;
  bool grid_dirty_ = true;
  std::string map_frame_ = "map";
  double vx_ = 0.0, vy_ = 0.0, meas_x_ = 0.0, meas_y_ = 0.0;
  rclcpp::Time meas_time_{0, 0, RCL_ROS_TIME};
  bool meas_valid_ = false;
  int kastar_ok_ = 0, kastar_fail_ = 0, fallback_ = 0;
  // ★ 2026-11-10：Kastar 轨迹发布器（此前控制器没有任何 Publisher，
  //   所以 RViz 里看不到"局部到底在追什么"，只能看到规划器的线）
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_kastar_;
  rclcpp::Publisher<rm_nav_interfaces::msg::StageStatus>::SharedPtr pub_st_l3_;
  // ★★ 2026-11-10：**Kastar 局部轨迹缓存**（回到 T-DT 的语义）
  //   T-DT 的 doc/Astar.md 把 Kastar 定义为"局部规划器"：解一次、得到一条轨迹，然后跟踪。
  //   我们之前把它当控制器用（每 33 ms 重解一次），而它是**离散采样搜索**
  //   （velResolution 0.5 / sampleTime 0.01）⇒ 同一个起终点两次搜索会给出
  //   "不同但都合法"的轨迹 ⇒ 速度方向来回切 ⇒ 车身持续震颤（实测橙线逐帧跳动）。
  //   现在：只在"缓存过期 / 车偏离缓存 / 局部目标移动"时重解，其余时间跟踪缓存。
  struct KSample { double x, y, vx, vy, t; };
  std::vector<KSample> k_cache_;
  rclcpp::Time k_stamp_{0, 0, RCL_ROS_TIME};
  double k_goal_x_ = 0.0, k_goal_y_ = 0.0;
  double p_kastar_replan_dt_ = 0.5;    // 缓存有效期（s）
  double p_kastar_dev_ = 0.30;         // 偏离缓存超过此值就重解（m）
  double p_kastar_goal_move_ = 0.50;   // 局部目标移动超过此值就重解（m）
  int gw_ = 0, gh_ = 0; float gres_ = 0.05f; float gox_ = 0.f, goy_ = 0.f;
  std::vector<unsigned char> gdata_;
};

}  // namespace rm_tdt_local_controller

PLUGINLIB_EXPORT_CLASS(rm_tdt_local_controller::TdtLocalController, nav2_core::Controller)
