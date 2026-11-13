// T-DT 完整规划器（nav2 GlobalPlanner 插件）
// 链路：二值图+Mask → YAstar(势场) → simplify → 梯形时间 → MinimumSnap(SFC+迭代) → TrajectoryPlan
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
// ★ 2026-11-09：ROGMap 进程内集成（LGPL-3.0-or-later，见 src/vendor/rog_map/NOTICE）
#include "rog_map/rog_map_core/config.hpp"
#include "rog_map/map_registry.hpp"
#include "rog_map/map_query_interface.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rm_nav_interfaces/msg/stage_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_plan_msgs/msg/trajectory_plan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "yastar.hpp"
#include "sfcSquare.hpp"
#include "minimumSnap.hpp"

namespace rm_tdt_planner {

class TdtPlanner : public nav2_core::GlobalPlanner {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
    node_ = parent.lock();
    name_ = name;
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    logger_ = node_->get_logger();
    // ★★★ 2026-11-14：**configure() 必须幂等**。
    //   nav2 lifecycle_manager（以及本仓 nav_lifecycle_guard）在 configure 失败后会重试；
    //   第二次进入本函数时 `declare_parameter` 抛
    //   "parameter 'GridBased.xxx' has already been declared" ⇒ planner 永远起不来。
    //   实测：max_escape 类型不匹配 → configure 失败 → guard 重试 → 撞 already declared。
    auto d = [&](const std::string& k, auto def) -> decltype(def) {
      const std::string full = name + "." + k;
      if (node_->has_parameter(full)) {
        return node_->get_parameter(full).get_value<decltype(def)>();
      }
      return node_->declare_parameter(full, def);
    };
    p_max_range_ = d("max_corridor_range", 2.5);
    p_shrink_ = d("corridor_shrink", 0.0);
    p_order_ = d("min_snap_order", 6);
    p_max_dx_ = d("max_dx", 3);
    p_tl_ = d("iter_time_limit", 10.0);
    p_collision_iter_ = d("collision_check_iter", 6);
    p_max_speed_ = d("max_speed", 3.0);
    p_max_acc_ = d("max_acc", 1.0);
    p_norm_time_ = d("norm_time", true);
    p_simplify_ = d("simplify_threshold", 0.1);
    p_init_vel_mode_ = static_cast<int>(d("init_vel_mode", 0.0));   // 0=自由 1=零 2=差分估计
    p_strict_collision_ = d("strict_collision", false);             // ★ 默认关（上游默认）
    p_retry_osqp_path_ = d("retry_osqp_path", false);               // ★ 默认关：OSQPPath 无走廊会穿墙
    p_dilate_cells_ = d("dilate_cells", 0);
    p_mask_enable_ = d("use_mask", true);
    p_use_map_file_ = d("use_map_file", true);
    p_map_yaml_ = d("map_yaml", std::string(""));
    p_map_frame_ = d("map_frame", std::string("map"));
    if (p_use_map_file_ && !p_map_yaml_.empty()) loadMapFile(p_map_yaml_);
    p_mask_duration_ = d("mask_duration", 1.0);
    // ★ 2026-11-09：进程内 ROGMap（use_rog_map:=true 时启用）
    //   它的投影层（UNKNOWN/FREE/PASSABLE/OCCUPIED）逐格查询后直接填 a_occ/sfc_map，
    //   与订阅 /plan_mask 二选一（mask_impl:=rogmap 时走这条）。
    p_use_rog_map_ = d("use_rog_map", false);
    // ★★ 2026-11-11：状态发布器**必须无条件创建**。
    //   之前它们写在 `if (p_use_rog_map_)` 里面 ⇒ 关掉 ROGMap 后
    //   L0/L1 连发布器都没有 ⇒ 监视器一直"无数据"、RViz 报 no publisher。
    pub_st_l0_ = node_->create_publisher<rm_nav_interfaces::msg::StageStatus>("/nav/l0/status", 1);
    pub_st_l1_ = node_->create_publisher<rm_nav_interfaces::msg::StageStatus>("/nav/l1/status", 1);
    if (p_use_rog_map_) {
      rog_map::Config rcfg;
      rcfg.loadFromRosNode(node_, "rog_map");
      rog_ = std::make_shared<rog_map::ROGMapROS>(node_, rcfg, tf_);
      RCLCPP_INFO(logger_, "[tdt] ROGMap 进程内已启动: cloud=%s res=%.2f",
                  rcfg.cloud_topic.c_str(), rcfg.resolution);
    }
    // ★★ 2026-11-09（步骤 1）：两级规划
    //   L1 骨架 = 只用静态图的 A*（缓存，低频重算）——保证全局方向稳定不抖；
    //   L2 局部 = 从当前位姿算到"骨架上 s + local_window 处"——实时吃掉动态障碍。
    //   关掉（two_level:=false）即退回原来的"一步算到真目标"行为。
    // ★ 2026-11-09：恢复闭环（FSM，落点=规划器）
    p_recovery_enable_ = d("recovery_enable", true);
    p_stuck_time_ = d("stuck_time", 2.0);
    p_stuck_eps_ = d("stuck_eps", 0.05);
    p_escape_dist_ = d("escape_dist", 1.5);
    // ★★ 2026-11-14：**逃逸行为可配**（"宁动不停"）
    //   max_escape <= 0             ⇒ 无限逃逸（不因尝试次数进 FAILED）
    //   escape_allow_enclosed=true  ⇒ 16 方向全 <escape_min_clear 时**不判 FAILED**，
    //                                 选最宽方向、距离缩到 best_d*0.5（小范围来回动）
    p_escape_allow_enclosed_ = d("escape_allow_enclosed", true);
    p_escape_min_clear_ = d("escape_min_clear", 0.35);
    p_escape_speed_ = d("escape_speed", 0.3);
    p_escape_clear_ = d("escape_clear", 0.5);
    p_max_escape_ = static_cast<int>(d("max_escape", 3.0));
    // ★ 尾段速度：tail_decel=0 ⇒ 全速（不减速）；>0 ⇒ 末端按该减速度减速
    p_tail_speed_ = d("tail_speed", 0.0);       // 0 ⇒ 用 min(max_speed,1.5)
    p_tail_decel_ = d("tail_decel", 0.0);       // 0 ⇒ 不减速（默认）
    // ★ 2026-11-10：启用 T-DT 的两个闲置能力
    //   use_items : 用 SolveInput::Item 逐点约束（起点速度钉住，中途点必经）
    //   log_sdf   : 用 YAstar::getSDF() 报告轨迹的最小净空
    p_use_items_ = d("use_items", true);
    p_log_sdf_ = d("log_sdf", true);
    p_two_level_ = d("two_level", true);
    p_local_window_ = d("local_window", 8.0);
    p_backbone_age_ = d("backbone_age", 5.0);
    p_mask_layers_ = d("mask_num_layers", 4);
    plan_pub_ = node_->create_publisher<rm_plan_msgs::msg::TrajectoryPlan>("/trajectory_plan", 1);
    // ★ 2026-11-09：把**规划器实际看到的那张栅格**发出来（a_occ），用于 RViz 诊断：
    //   它 = 静态图 ∪ ROGMap(OCCUPIED) ∪ mask，也就是 A*/走廊/门控 真正用的障碍集。
    grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/nav/l1/dbg/plan_grid", rclcpp::QoS(1));
    // ★ 2026-11-09：三段路径分别发布，RViz 里像 T-DT 那样同时看
    //   ① A* 原始栅格路径  ② DP 简化后  ③ MinSnap 平滑后
    pub_astar_ = node_->create_publisher<nav_msgs::msg::Path>("/nav/l1/dbg/astar_raw", rclcpp::QoS(1));
    pub_simpl_ = node_->create_publisher<nav_msgs::msg::Path>("/nav/l1/dbg/simplified", rclcpp::QoS(1));
    pub_smooth_ = node_->create_publisher<nav_msgs::msg::Path>("/nav/l1/dbg/smooth", rclcpp::QoS(1));
    // ★ 飞行走廊（SFC 方框）——RViz 里用蓝色半透明方块
    pub_corridor_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/nav/l1/dbg/corridor", rclcpp::QoS(1));
    // ★★★ 2026-11-12：**骨架(全局 L1) 之前完全没有可视化**。
    //   注意（**别误读**）：`/plan` 本身是**全局**的——L2 只优化前 8 m，
    //   之后由下方 951 行附近的"骨架尾段拼接"接到真目标。
    //   缺可视化的只是**骨架本身**：8 m 之外那一段是静态图主干，
    //   **没有经过动态障碍优化** ⇒ 调试时要能把它单独看出来。
    pub_backbone_ = node_->create_publisher<nav_msgs::msg::Path>("/nav/l1/dbg/backbone", rclcpp::QoS(1));
    pub_rog_grid_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/nav/l0/dbg/grid", rclcpp::QoS(1));
    pub_rog_obs_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/nav/l0/dbg/occupied", rclcpp::QoS(1));
    pub_rog_win_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/nav/l0/dbg/window", rclcpp::QoS(1));
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { buildFromOccupancyGrid(m); });
    mask_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/plan_mask", rclcpp::QoS(1),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { mask_ = m; });
    // ★★ 2026-11-09（致命 bug）：rclcpp::Time 默认构造 = RCL_SYSTEM_TIME，
    //   而 node_->now() 在本栈是 ROS/SIM 时间 ⇒ `now - member` 直接抛
    //   "can't subtract times with different time sources [1 != 2]"，
    //   整个 compute_path 被 nav2 判失败（日志里每条 goal 都 abort）。
    //   修法：全部用 node_->now() 初始化，之后只在同一时钟下更新。
    {
      const auto t0 = node_->now();
      last_move_time_ = t0;
      last_escape_end_ = t0;
      prev_pos_time_ = t0;
      first_fail_time_ = t0;
      escape_start_ = t0;
    }

    RCLCPP_INFO(logger_, "[tdt] configured: range=%.2f shrink=%.2f order=%d maxDx=%d TL=%.1f iter=%d v=%.2f a=%.2f norm=%d",
                p_max_range_, p_shrink_, p_order_, p_max_dx_, p_tl_, p_collision_iter_, p_max_speed_, p_max_acc_,
                (int)p_norm_time_);
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                 const geometry_msgs::msg::PoseStamped& goal) override {
    nav_msgs::msg::Path out;
    out.header = start.header;

    // ★★ 2026-11-09：**恢复 FSM 短路**（ESCAPE / FAILED 时不走正常规划）
    //   ESCAPE 期间直接重发同一条逃逸直线（控制器只跟 /plan），
    //   时间到或已脱离 ⇒ 作废骨架、回到 NORMAL（下一周期强制重算主干）。
    // ★★ 2026-11-10（重要）：**新目标要复位恢复状态**
    //   否则一旦进入 FAILED，之后每个目标都只返回单点路径 ⇒ nav2 一律 abort
    //   （现象就是"目标被拒绝"），且永远无法自愈。
    {
      const double gx_new = goal.pose.position.x, gy_new = goal.pose.position.y;
      if (std::hypot(gx_new - last_goal_x_, gy_new - last_goal_y_) > 0.5 ||
          last_goal_x_ < -1e8) {
        if (rec_state_ != REC_NORMAL)
          RCLCPP_WARN(logger_, "[tdt][恢复] 新目标 ⇒ 状态复位（原 %s）",
                      rec_state_ == REC_FAILED ? "FAILED" : "ESCAPE");
        rec_state_ = REC_NORMAL;
        escape_attempts_ = 0;
        fail_streak_ = 0;
        last_goal_x_ = gx_new; last_goal_y_ = gy_new;
        last_move_time_ = node_->now();
      }
    }
    if (p_recovery_enable_ && rec_state_ == REC_FAILED) {
      nav_msgs::msg::Path stop;
      stop.header = start.header;
      stop.poses.push_back(start);
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 3000,
          "[tdt][恢复] FAILED：原地停车等待（目标不可达或被困死）");
      // 15 s 后自动重试，避免永久卡死
      if ((node_->now() - last_escape_end_).seconds() > 15.0) {
        rec_state_ = REC_NORMAL;
        escape_attempts_ = 0;
        fail_streak_ = 0;
        RCLCPP_WARN(logger_, "[tdt][恢复] FAILED 超时 ⇒ 复位重试");
      }
      return stop;
    }
    // ★★ 2026-11-09：ESCAPE 不再走"直线"，改为**把逃逸点当目标，走完整的 A*+SFC+MinSnap**
    //   ⇒ 逃逸段也是平滑、带动力学约束、且过零容差门控的轨迹。
    //   这里只负责判定"逃逸是否该结束"，真正的规划在下面对目标做覆盖。
    if (p_recovery_enable_ && rec_state_ == REC_ESCAPE) {
      const double px = start.pose.position.x, py = start.pose.position.y;
      const double elapsed = (node_->now() - escape_start_).seconds();
      const double need = escape_target_dist_ / std::max(0.05, p_escape_speed_);
      double clr = 0.0; bool have_clr = false;
      if (auto q = rog_map::MapRegistry::get()) {
        double dist = 0.0; Eigen::Vector3d grad;
        if (q->evaluate(Eigen::Vector3d(px, py, 0.0), dist, grad)) { clr = dist; have_clr = true; }
      }
      const bool escaped_ok = have_clr && clr >= p_escape_clear_;
      if (elapsed >= need + 0.5 || escaped_ok) {
        backbone_valid_ = false;
        rec_state_ = REC_NORMAL;
        last_escape_end_ = node_->now();
        RCLCPP_WARN(logger_, "[tdt][恢复] 逃逸完成（%s）⇒ 作废骨架、重新规划",
                    escaped_ok ? "clearance 达标" : "走满距离");
      } else {
        RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 1000,
            "[tdt][恢复] ESCAPE 中: 方向 %.0f°, 已 %.1f/%.1f s, clearance %.2f m, 尝试 %d/%d",
            escape_heading_ * 180.0 / M_PI, elapsed, need, clr, escape_attempts_, p_max_escape_);
      }
    }

    // ★ costmap 的世界坐标在**它的 global_frame**（本栈是 odom_level），
    //   而 nav2 传来的 start/goal 在 map 帧 ⇒ 必须先转帧，否则 A* 在错误的坐标里搜索。
    // ★ 优先用**先验图（PGM）**做规划（与 T-DT 同源：静态图离线，动态走 Mask）；
    //   没有 map_yaml 时才退回 costmap 的 lethal 格。
    std::string cf;
    double sx = start.pose.position.x, sy = start.pose.position.y;
    double gx = goal.pose.position.x, gy = goal.pose.position.y;
    unsigned int W = 0, H = 0; double res = 0.05, ox = 0, oy = 0;
    std::vector<int8_t> a_occ;
    std::vector<uint8_t> a_occ_dyn_;   // 本帧 mask 命中格（供 mask 膨胀用）
    SfcSquare::Map sfc_map;
    int n_occ = 0;

    if (p_use_map_file_ && map_loaded_) {
      cf = p_map_frame_;
      W = static_cast<unsigned int>(mw_); H = static_cast<unsigned int>(mh_);
      res = mres_; ox = mox_; oy = moy_;
      a_occ = mocc_;
      sfc_map = SfcSquare::Map(static_cast<int>(H), static_cast<int>(W));
      for (unsigned int y = 0; y < H; ++y)
        for (unsigned int x = 0; x < W; ++x)
          sfc_map(static_cast<int>(y), static_cast<int>(x)) = msfc_[static_cast<size_t>(y) * W + x];
      for (int8_t v : a_occ) n_occ += (v != 0) ? 1 : 0;
    } else {
      // 起终点转到 costmap 全局帧（costmap 的世界坐标在该帧下）
      cf = costmap_ros_->getGlobalFrameID();
      geometry_msgs::msg::PoseStamped s_cf = start, g_cf = goal;
      try {
        if (start.header.frame_id != cf) tf_->transform(start, s_cf, cf, tf2::durationFromSec(0.2));
        if (goal.header.frame_id != cf) tf_->transform(goal, g_cf, cf, tf2::durationFromSec(0.2));
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "[tdt] 转帧失败(%s→%s): %s", start.header.frame_id.c_str(), cf.c_str(), e.what());
        return out;
      }
      sx = s_cf.pose.position.x; sy = s_cf.pose.position.y;
      gx = g_cf.pose.position.x; gy = g_cf.pose.position.y;

      auto* cm = costmap_ros_->getCostmap();
      W = cm->getSizeInCellsX(); H = cm->getSizeInCellsY();
      res = cm->getResolution(); ox = cm->getOriginX(); oy = cm->getOriginY();
      const unsigned char* costs = cm->getCharMap();
      if (W == 0 || H == 0) { RCLCPP_ERROR(logger_, "[tdt] costmap 为空"); return out; }
      a_occ.assign(static_cast<size_t>(W) * H, 0);
      sfc_map = SfcSquare::Map(static_cast<int>(H), static_cast<int>(W));
      for (unsigned int y = 0; y < H; ++y)
        for (unsigned int x = 0; x < W; ++x) {
          const unsigned char c = costs[y * W + x];
          const bool occ = (c == nav2_costmap_2d::LETHAL_OBSTACLE);
          a_occ[static_cast<size_t>(y) * W + x] = occ ? 1 : 0;
          sfc_map(static_cast<int>(y), static_cast<int>(x)) = occ ? 0 : 255;
          n_occ += occ ? 1 : 0;
        }
    }
    // ★★ 2026-11-08：**严格几何校验**（0 容差 Bresenham，直接查 a_occ）。
    //   为什么必须自己查：T-DT 的 `lineInObsticle` 在**非严格模式**下允许穿过 ≤2 个障碍格
    //   （官方文档原话），而我们的手画图墙只有 1~2 格厚 ⇒ 平滑轨迹与 DP 简化都可能**合法地穿墙**。
    auto seg_clear = [&](const Eigen::Vector2f& a, const Eigen::Vector2f& b,
                         bool skip_first = false, bool skip_last = false) -> bool {
      int x0 = static_cast<int>(std::floor(a.x() / res)), y0 = static_cast<int>(std::floor(a.y() / res));
      const int x1 = static_cast<int>(std::floor(b.x() / res)), y1 = static_cast<int>(std::floor(b.y() / res));
      const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
      const int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
      int err = dx - dy;
      for (int guard = 0; guard < 100000; ++guard) {
        if (x0 < 0 || y0 < 0 || x0 >= static_cast<int>(W) || y0 >= static_cast<int>(H)) return false;
        // ★ 起点/终点那一格豁免：A* 无条件把起点塞进路径，它可能落在 mask/车身格上
        const bool at_first = (guard == 0 && skip_first);
        const bool at_last = (x0 == x1 && y0 == y1 && skip_last);
        if (!at_first && !at_last && a_occ[static_cast<size_t>(y0) * W + x0]) return false;
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
      }
      return true;
    };
    auto path_clear = [&](const std::vector<Eigen::Vector2f>& pts) -> bool {
      const size_t n = pts.size();
      for (size_t i = 1; i < n; ++i)
        if (!seg_clear(pts[i - 1], pts[i], /*skip_first=*/i == 1, /*skip_last=*/i + 1 == n)) return false;
      return true;
    };
    // ★★ 2026-11-11：**"只看墙"的判定**——动态层（ROGMap/mask）有噪声，
    //   它可以让 A* 绕路（代价），但**绝不该否决规划**。
    //   现实后果：ROGMap 报 31~38% 占用时，所有 5 m 以上的目标都被判"A* 原始路径也穿墙"
    //   ⇒ 直接 return 空路径 ⇒ 车不动 ⇒ T1 恢复。墙是真障碍，必须拒；动态层是估计，不能拒。
    auto seg_clear_static = [&](const Eigen::Vector2f& a, const Eigen::Vector2f& b,
                                bool skip_first, bool skip_last) -> bool {
      if (a_occ_static_backup_.empty()) return true;
      int x0 = static_cast<int>(std::floor(a.x() / res)), y0 = static_cast<int>(std::floor(a.y() / res));
      const int x1 = static_cast<int>(std::floor(b.x() / res)), y1 = static_cast<int>(std::floor(b.y() / res));
      const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
      const int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
      int err = dx - dy;
      for (int guard = 0; guard < 100000; ++guard) {
        if (x0 < 0 || y0 < 0 || x0 >= static_cast<int>(W) || y0 >= static_cast<int>(H)) return false;
        const bool at_first = (guard == 0 && skip_first);
        const bool at_last = (x0 == x1 && y0 == y1 && skip_last);
        // ★★ 2026-11-11：**起点附近给一个半径豁免**。
        //   车一旦贴到墙上（实测 SDF 0.07 m），它就落在 dilate_cells=2 的膨胀区**内部**，
        //   路径最前面好几格都必然是"占用" ⇒ 被判穿墙 ⇒ 所有规划被拒 ⇒ 原地卡死。
        //   这不是穿墙，是"车已经在膨胀区里"。0.25 m 覆盖膨胀层 + 车体半径。
        const double dcx = (x0 + 0.5) * res - a.x();
        const double dcy = (y0 + 0.5) * res - a.y();
        const bool near_start = (dcx * dcx + dcy * dcy) < 0.25 * 0.25;
        if (!at_first && !at_last && !near_start &&
            a_occ_static_backup_[static_cast<size_t>(y0) * W + x0]) return false;
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
      }
      return true;
    };
    auto path_clear_static = [&](const std::vector<Eigen::Vector2f>& pts) -> bool {
      const size_t n = pts.size();
      for (size_t i = 1; i < n; ++i)
        if (!seg_clear_static(pts[i - 1], pts[i], i == 1, i + 1 == n)) return false;
      return true;
    };

    // ★★ 2026-11-09：**ROGMap 进程内查询**（替代 /plan_mask）
    //   MapRegistry 是进程内静态注册表，ROGMapROS 构造时 init() 会把自己塞进去。
    //   查询按**我们自己的网格**逐格做 ⇒ 不需要重采样；窗口外保持静态图原值。
    if (p_use_rog_map_) {
      auto q = rog_map::MapRegistry::get();
      if (q) {
        // 静态图备份已在建图时做过（本函数上方）；这里只补 sfc 的备份
        sfc_map_static_backup_ = sfc_map;
        int n_occ = 0, n_pass = 0, n_unknown = 0, n_unknown_inside = 0;
        rog_grid_snapshot_.assign(static_cast<size_t>(W) * H, -1);
        for (unsigned int y = 0; y < H; ++y)
          for (unsigned int x = 0; x < W; ++x) {
            const double wx = ox + (x + 0.5) * res;
            const double wy = oy + (y + 0.5) * res;
            unsigned int mx = 0, my = 0;
            if (!q->worldToMap(wx, wy, mx, my) || !q->isValid(mx, my)) { ++n_unknown; continue; }
            const uint8_t v = q->value(mx, my);
            const size_t i = static_cast<size_t>(y) * W + x;
            // ★★ 2026-11-10（关键修复）：ROGMap 的 value 层编码**不是** CellType 枚举
            //   （见 projection_layer.cpp::assignValue）：
            //     0   = FREE
            //     255 = UNKNOWN（unknown_as_occupied=false 时）
            //     254 = OCCUPIED          ← 之前写 v==3，永远匹配不到 ⇒ 一直 0 格
            //     其他 = PASSABLE 的代价（passable_cost）
            if (v == 254U) {
              rog_grid_snapshot_[i] = 100;
              a_occ[i] = 1;
              sfc_map(static_cast<int>(y), static_cast<int>(x)) = 0;
              ++n_occ;
            } else if (v == 255U) {
              rog_grid_snapshot_[i] = -1;      // 未知：可视化里不画、也不当障碍
              ++n_unknown; ++n_unknown_inside;
            } else {
              rog_grid_snapshot_[i] = 0;
              if (v != 0U) ++n_pass;           // PASSABLE（带代价）
            }
          }
        // ★★ 2026-11-10：**可信度门控**。ROGMap 的占用率若明显不合理（>40%），
        //   说明这一帧的判定已经坏了（实测出现过 84% ⇒ A* 全失败）。
        //   此时**整帧丢弃**：不写进 a_occ / sfc_map，只用静态图继续跑，
        //   并打印警告，避免"地图全障碍"把规划器整个搞死。
        const int inside = n_occ + n_pass + (n_unknown_inside);
        const double occ_ratio = inside > 0 ? static_cast<double>(n_occ) / inside : 0.0;
        // ★★ 2026-11-11：门限 0.40 **太死**。实测场地本身障碍就密（墙+场地元素
        //   在 12 m 窗口里能占到 30~45%），于是每一帧都被当成"坏帧"丢弃
        //   ⇒ 动态障碍永远进不了规划，A* 只能在静态图上跑，还老撞"穿墙"判定。
        //   放宽到 0.75：只在**明显病态**（几乎全占，>75%）时才丢弃。
        const bool rog_trust = occ_ratio <= 0.75;
        if (pub_st_l0_) {
          rm_nav_interfaces::msg::StageStatus st;
          st.header.stamp = node_->now();
          st.stage = 0;
          st.state = rog_trust ? (n_occ > 0 ? 1 : 2) : 3;
          st.detail = rog_trust ? (n_occ > 0 ? "局部图正常" : "局部图无占用（点云稀疏？）")
                                : "占用率不合理，本帧丢弃";
          st.keys.resize(3); st.values.resize(3);
          st.keys[0]="occupied"; st.keys[1]="inside"; st.keys[2]="occ_ratio";
          st.values[0]=static_cast<double>(n_occ);
          st.values[1]=static_cast<double>(inside);
          st.values[2]=occ_ratio;
          pub_st_l0_->publish(st);
        }
        if (!rog_trust) {
          RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 3000,
              "[tdt] ROGMap 占用率 %.0f%% 不合理（窗口内 %d 格）⇒ 本帧丢弃，仅用静态图",
              occ_ratio * 100.0, inside);
          a_occ = a_occ_static_backup_;
          sfc_map = sfc_map_static_backup_;
          std::fill(rog_grid_snapshot_.begin(), rog_grid_snapshot_.end(), -1);
          n_occ = 0;
        }
        RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 5000,
            "[tdt] ROGMap 查询: OCCUPIED %d 格, PASSABLE %d 格, 窗口外 %d 格 | 查询点(%.2f,%.2f) 地图窗口 origin=(%.2f,%.2f) size=%dx%d res=%.3f",
            n_occ, n_pass, n_unknown, sx, sy, q->originX(), q->originY(),
            q->sizeX(), q->sizeY(), q->resolution());
      } else {
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 5000,
            "[tdt] ROGMap 未注册（MapRegistry 为空），退回静态图");
      }
    }

    // ★★ 2026-11-09（步骤 1）：**两级规划**
    //   此刻 a_occ / sfc_map 还是**纯静态**，正是算"骨架"的时机。
    const float lox0 = static_cast<float>(ox), loy0 = static_cast<float>(oy);
    double gx_solve = gx, gy_solve = gy;
    bool use_two_level = false;
    // ★ ESCAPE：把目标换成逃逸点（离机器人 escape_dist），交给 A*+SFC+MinSnap 求解
    if (p_recovery_enable_ && rec_state_ == REC_ESCAPE) {
      gx_solve = static_cast<double>(sx) + std::cos(escape_heading_) * escape_target_dist_;
      gy_solve = static_cast<double>(sy) + std::sin(escape_heading_) * escape_target_dist_;
      use_two_level = false;
    }
    double s_head = 0.0;
    // ★ 逃逸时**不能**再走两级：否则下面的两级块会把 gx_solve 覆盖回
    //   "骨架上 8 m 处"，逃逸目标就丢了（日志里 ESCAPE 却仍打印"两级"就是这个 bug）
    if (p_two_level_ && !(p_recovery_enable_ && rec_state_ == REC_ESCAPE)) {
      const bool goal_moved =
          std::hypot(gx - backbone_goal_x_, gy - backbone_goal_y_) > 0.3;
      const double age = backbone_valid_ ? (node_->now() - backbone_stamp_).seconds() : 1e9;
      if (!backbone_valid_ || goal_moved || age > p_backbone_age_) {
        YAstar ba;
        ba.setMapping(static_cast<float>(res));
        ba.setOriginPos(0.f, 0.f);
        ba.setMap(static_cast<int>(W), static_cast<int>(H), static_cast<float>(res),
                  0.f, 0.f, a_occ);                        // ← 纯静态
        ba.setCostField(1.0f, [](float x) { return 0.5f / (0.1f + x); });
        ba.initCostMap();
        auto bp = ba.search(Eigen::Vector2f(static_cast<float>(sx) - lox0, static_cast<float>(sy) - loy0),
                            Eigen::Vector2f(static_cast<float>(gx) - lox0, static_cast<float>(gy) - loy0));
        backbone_.clear(); backbone_s_.clear();
        for (auto& q : bp) backbone_.push_back(Eigen::Vector2f(q.x() + lox0, q.y() + loy0));
        backbone_s_.assign(backbone_.size(), 0.0);
        for (size_t i = 1; i < backbone_.size(); ++i)
          backbone_s_[i] = backbone_s_[i - 1] + (backbone_[i] - backbone_[i - 1]).norm();
        backbone_valid_ = backbone_.size() >= 2;
        backbone_goal_x_ = gx; backbone_goal_y_ = gy;
        backbone_stamp_ = node_->now();
        if (backbone_valid_) {
          RCLCPP_INFO(logger_, "[tdt] 骨架重算: %zu 点, %.1f m (静态图)",
                      backbone_.size(), backbone_s_.back());
          // 注意：make_path 是后面（约 691 行）才定义的 lambda，此处不能调用
          // ⇒ 就地构造。backbone_ 里存的已经是**世界坐标**，无需再加偏移。
          nav_msgs::msg::Path bp_msg;
          bp_msg.header.stamp = node_->now();
          bp_msg.header.frame_id = p_map_frame_;
          bp_msg.poses.reserve(backbone_.size());
          for (const auto& q : backbone_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = bp_msg.header;
            ps.pose.position.x = static_cast<double>(q.x());
            ps.pose.position.y = static_cast<double>(q.y());
            ps.pose.orientation.w = 1.0;
            bp_msg.poses.push_back(ps);
          }
          pub_backbone_->publish(bp_msg);
        }
      }
      if (backbone_valid_) {
        double best = 1e18; size_t bi = 0;
        for (size_t i = 0; i < backbone_.size(); ++i) {
          const double d = std::hypot(backbone_[i].x() - sx, backbone_[i].y() - sy);
          if (d < best) { best = d; bi = i; }
        }
        const double s_now = backbone_s_[bi];
        const double s_total = backbone_s_.back();
        if (s_total - s_now > p_local_window_ + 0.5) {
          const double s_goal = s_now + p_local_window_;
          size_t gi = bi;
          while (gi + 1 < backbone_s_.size() && backbone_s_[gi + 1] < s_goal) ++gi;
          gx_solve = backbone_[gi].x(); gy_solve = backbone_[gi].y();
          s_head = backbone_s_[gi];
          use_two_level = true;
        }
      }
    }

    // ★★ 2026-11-09（步骤 0）：**当前 mask 一次性并入 a_occ 与 sfc_map**。
    //   说明：**真正的 mask 合并在下面"按世界坐标重采样"那段**（支持几何不同：pad2 后的规划图
    //   584x345 vs /map 580x341），这里不再做"元数据必须完全相等"的二次合并
    //   （2026-11-14 移除：它永远不成立，只会每 5 s 打一条假警告"动态障碍不并入"）。

    // ★ 发布规划栅格（限流 ~5 Hz，仅在有订阅者时构造）
    if (grid_pub_ && grid_pub_->get_subscription_count() > 0) {
      static rclcpp::Time last_grid_pub(0, 0, RCL_ROS_TIME);   // RCL_ROS_TIME 与 node_->now() 一致
      const auto tnow = node_->now();
      if (last_grid_pub.nanoseconds() == 0 ||
          (tnow - last_grid_pub).seconds() > 0.2) {
        last_grid_pub = tnow;
        nav_msgs::msg::OccupancyGrid g;
        g.header.stamp = tnow;
        g.header.frame_id = cf;
        g.info.resolution = static_cast<float>(res);
        g.info.width = W;
        g.info.height = H;
        g.info.origin.position.x = ox;
        g.info.origin.position.y = oy;
        g.info.origin.orientation.w = 1.0;
        g.data.resize(static_cast<size_t>(W) * H);
        for (size_t i = 0; i < g.data.size(); ++i) g.data[i] = a_occ[i] ? 100 : 0;
        grid_pub_->publish(g);
      }
    }

    // ★ ROGMap 语义栅格快照（-1 表示窗口外/未知）
    if (pub_rog_grid_ && pub_rog_grid_->get_subscription_count() > 0 && !rog_grid_snapshot_.empty()) {
      nav_msgs::msg::OccupancyGrid g;
      g.header.stamp = node_->now();
      g.header.frame_id = cf;
      g.info.resolution = static_cast<float>(res);
      g.info.width = W;
      g.info.height = H;
      g.info.origin.position.x = ox;
      g.info.origin.position.y = oy;
      g.info.origin.orientation.w = 1.0;
      g.data = rog_grid_snapshot_;
      pub_rog_grid_->publish(g);
    }

    // ★ 局部障碍（蓝点）+ 局部窗口（蓝框）：让"哪里是局部感知范围"一眼可见
    if (p_use_rog_map_ && !rog_grid_snapshot_.empty() &&
        (pub_rog_obs_->get_subscription_count() > 0 || pub_rog_win_->get_subscription_count() > 0)) {
      const auto stamp = node_->now();
      // 采集 ROGMap 占用格
      std::vector<std::array<float, 3>> pts;
      pts.reserve(4096);
      for (unsigned int y = 0; y < H; ++y)
        for (unsigned int x = 0; x < W; ++x)
          if (rog_grid_snapshot_[static_cast<size_t>(y) * W + x] > 50)
            pts.push_back({static_cast<float>(ox + (x + 0.5) * res),
                           static_cast<float>(oy + (y + 0.5) * res), 0.08f});
      if (pub_rog_obs_->get_subscription_count() > 0 && !pts.empty()) {
        visualization_msgs::msg::MarkerArray ma;
        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(del);
        int id = 0;
        for (const auto& q : pts) {
          visualization_msgs::msg::Marker m;
          m.header.stamp = stamp; m.header.frame_id = cf;
          m.ns = "rog_obs"; m.id = id++;
          m.type = visualization_msgs::msg::Marker::CUBE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = q[0]; m.pose.position.y = q[1]; m.pose.position.z = 0.10;
          m.pose.orientation.w = 1.0;
          m.scale.x = 0.06; m.scale.y = 0.06; m.scale.z = 0.20;
          // 青色：与走廊的蓝色区分开（走廊是"约束区域"，障碍是"实体"）
          m.color.r = 0.0f; m.color.g = 0.85f; m.color.b = 0.85f; m.color.a = 0.95f;
          m.lifetime = rclcpp::Duration::from_seconds(0.6);
          ma.markers.push_back(m);
        }
        pub_rog_obs_->publish(ma);
      }
      // 窗口边框：从 ROGMap 查询接口拿真实窗口范围
      if (pub_rog_win_->get_subscription_count() > 0) {
        if (auto q = rog_map::MapRegistry::get()) {
          const int sw = static_cast<int>(q->sizeX()), sh = static_cast<int>(q->sizeY());
          const double sr = q->resolution();
          if (sw > 0 && sh > 0 && sr > 0.0) {
            const double x0 = q->originX(), y0 = q->originY();
            const double x1 = x0 + sw * sr, y1 = y0 + sh * sr;
            visualization_msgs::msg::MarkerArray ma;
            visualization_msgs::msg::Marker m;
            m.header.stamp = stamp; m.header.frame_id = cf;
            m.ns = "rog_window"; m.id = 0;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.04;
            // 中性白灰：只表示"局部有效范围"，不参与颜色语义
            m.color.r = 0.85f; m.color.g = 0.85f; m.color.b = 0.85f; m.color.a = 0.8f;
            const double xs[5] = {x0, x1, x1, x0, x0};
            const double ys[5] = {y0, y0, y1, y1, y0};
            for (int k = 0; k < 5; ++k) {
              geometry_msgs::msg::Point pt;
              pt.x = xs[k]; pt.y = ys[k]; pt.z = 0.05;
              m.points.push_back(pt);
            }
            ma.markers.push_back(m);
            pub_rog_win_->publish(ma);
          }
        }
      }
    }

    if (n_occ == 0) RCLCPP_WARN(logger_, "[tdt] 二值图里没有任何障碍");

    // ② 走廊地图：边界安全（1 格），**不做机器人半径膨胀**
    SfcSquare sfc;
    sfc.setMap(sfc_map, static_cast<float>(res), Eigen::Vector2f(0.f, 0.f));  // ★ 局部坐标（与 A* 一致）
    if (p_dilate_cells_ > 0) sfc.setSaveDistance(static_cast<float>(p_dilate_cells_ * res), false);

    // ③ A*
    // ★★ 2026-11-08（根因修复）：**全程使用"地图局部坐标"= world − origin**。
    //   原因：T-DT 内部有三处不同的 origin 约定，只在 origin==(0,0) 时才一致：
    //     · `SfcSquare::valueAt / _getBound / _getBoundSquare2`： (p + origin)/res
    //     · `SfcSquare::isOutside`、`MinimumSnap::lineInObsticle`：(p − origin)/res
    //   上游 demo 的地图 origin 就是 (0,0)，所以这个矛盾永远不暴露。
    //   我们若传 −origin 去迁就 `+` 那一组，就会让 `isOutside`/`lineInObsticle` 全部判成"出界"
    //   ⇒ 每段都判碰撞、update 拿到 NaN 走廊、第 2 轮 QP 直接失败 ⇒ **snap 永远=0**。
    //   修法：让 origin=(0,0)，把起终点/路径都平移到局部系，输出再平移回世界系。
    YAstar astar;
    astar.setMapping(static_cast<float>(res));
    astar.setOriginPos(0.f, 0.f);
    astar.setMaskNumLayers(p_mask_layers_);
    // ★★ 2026-11-11：在**并入任何动态层之前**先存一份纯静态图。
    //   下面所有"穿墙"判定都用这一份 ⇒ 感知噪声永远不会否决规划。
    a_occ_static_backup_ = a_occ;
    astar.setMap(static_cast<int>(W), static_cast<int>(H), static_cast<float>(res),
                 0.f, 0.f, a_occ);
    // ★★★ 2026-11-11（关键修复）：**不再要求掩码网格与规划器网格逐位相同**。
    //   实测：terrain_layer 产 560x313 @ origin(-19.90,-8.89)，
    //         规划器 +pad2 后是 564x317 @ origin(-20.00,-8.99)
    //   ⇒ 旧的 `width==W && origin 相差<1e-6` 永远不成立
    //   ⇒ **掩码一帧都没进过规划器**（日志一直写"(静态图)"）。
    //   现在按**世界坐标重采样**：只要求分辨率一致，原点/尺寸随意。
    if (p_mask_enable_ && mask_ && mask_->info.resolution > 0.0 &&
        std::abs(mask_->info.resolution - res) < 1e-6 &&
        mask_->info.width > 0 && mask_->info.height > 0) {
      const double mox = mask_->info.origin.position.x;
      const double moy = mask_->info.origin.position.y;
      const int MW = static_cast<int>(mask_->info.width);
      const int MH = static_cast<int>(mask_->info.height);
      Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mm(H, W);
      int n_dyn = 0, n_hit = 0, n_self = 0;
      a_occ_dyn_.assign(static_cast<size_t>(W) * H, 0);   // 本帧 mask 命中的格（不含膨胀）   // n_self = 因"自车 0.35 m 内"被忽略的格
      for (unsigned int y = 0; y < H; ++y)
        for (unsigned int x = 0; x < W; ++x) {
          const double wx = ox + (x + 0.5) * res;
          const double wy = oy + (y + 0.5) * res;
          const int mx = static_cast<int>(std::floor((wx - mox) / res));
          const int my = static_cast<int>(std::floor((wy - moy) / res));
          if (mx < 0 || my < 0 || mx >= MW || my >= MH) {
            mm(y, x) = 255;                       // 掩码范围外 ⇒ 视作可行
            continue;
          }
          const int8_t v = mask_->data[static_cast<size_t>(my) * MW + mx];
          bool occ = (v > 50);                    // OccupancyGrid: >50 = 占用
          // ★★★ 2026-11-12（晚，**离线快照实证**）：**屏蔽自车周围 0.35 m 内的 mask 格**。
          //   证据链：
          //     ① 失败快照（`.tmp_diag/dump/snapfail_106.pgm`）+ 当次控制点，
          //        用 minisnap_points_probe 复现出 **snap=0 iter=6**；
          //     ② 同一组点只喂静态图 ⇒ **snap=1**；
          //     ③ 逐格比对：mask 里"静态图没有"的格中有 **20 格离折线 <0.15 m**，
          //        最近一格距**起点仅 0.04 m**；
          //     ④ 把起点 0.35 m 内的 mask 格清掉后重新求解 ⇒ **snap=1 iter=3**。
          //   ⇒ 结论：**mask 会把自车周边（自身点残留/衰减幽灵）标成障碍**，
          //     而这些格正好落在第一个控制点上 ⇒ 方形走廊建不起来 ⇒ MinSnap 不可行。
          //   自车不可能身处障碍中，所以这一圈的数据本身无意义 ⇒ 直接忽略。
          //   （与逃逸采样、path_clear_static 的"起点豁免"是同一类修正。）
          if (occ) {
            const double wxr = wx - sx, wyr = wy - sy;   // sx/sy = 本帧起点（局部坐标）
            if (wxr * wxr + wyr * wyr < 0.35 * 0.35) { occ = false; ++n_self; }
          }
          mm(y, x) = occ ? 0 : 255;
          if (occ) {
            const size_t i = static_cast<size_t>(y) * W + x;
              // ★★★ 2026-11-14（实测根因）：**只有"非静态"的 mask 格才进膨胀集合**。
              //   /plan_mask 与静态图几乎相同（36,705 vs 36,720，非静态 14~17 格）——STVL 把静态墙也标 lethal。
              //   原来对每个 mask 格都置 a_occ_dyn_ ⇒ 下面 1 格膨胀把墙又膨胀一遍 ⇒ 墙净空
              //   = dilate_cells(0.10) + mask 膨胀(0.05) + corridor_shrink(0.15) = 0.30 m；
              //   0.7 m 门只剩 0.10 m ⇒ 角落/门口 A* 无解（实测 aocc 多出的 19,484 格里 81% 离墙 ≤0.10 m）。
              const bool is_static = (a_occ[i] != 0);
              if (!is_static) { a_occ[i] = 1; a_occ_dyn_[i] = 1; ++n_dyn; }
            sfc_map(static_cast<int>(y), static_cast<int>(x)) = 0;
            ++n_hit;
          }
        }
      // ★★★ 2026-11-13（凌晨，**第二类失败的根因**）：**mask 格也要膨胀**。
      //   失败快照 snapfail_124 里，"mask 独有格"离折线只有 **0.03~0.05 m**
      //   （在 (15.15,2.20) 一带，离起点 6 m ⇒ 不是自车格，是真障碍）。
      //   为什么 A* 会贴着它们走：**静态障碍按 dilate_cells=2 膨胀，mask 格却是"裸加"**
      //   （下面这段只做 `a_occ[i]=1`，没有膨胀）⇒ A* 可以在动态障碍旁边 1 格穿过
      //   ⇒ 交给 SFC 时走廊已被两侧夹死 ⇒ MinSnap 不可行。
      //   修法：mask 也按 **1 格**（0.05 m）膨胀（动态障碍是瞬时的，不取满 dilate_cells，
      //   避免有人经过时把窄道完全堵死）。
      {
        const int mk = 1;
        std::vector<std::pair<int,int>> add;
        for (unsigned int y2 = 0; y2 < H; ++y2)
          for (unsigned int x2 = 0; x2 < W; ++x2)
            if (a_occ_dyn_[static_cast<size_t>(y2) * W + x2]) add.emplace_back(x2, y2);
        for (auto [cx2, cy2] : add) {
          for (int dy2 = -mk; dy2 <= mk; ++dy2)
            for (int dx2 = -mk; dx2 <= mk; ++dx2) {
              const int nx2 = cx2 + dx2, ny2 = cy2 + dy2;
              if (nx2 < 0 || ny2 < 0 || nx2 >= static_cast<int>(W) || ny2 >= static_cast<int>(H)) continue;
              const size_t ii = static_cast<size_t>(ny2) * W + nx2;
              if (!a_occ[ii]) { a_occ[ii] = 1; ++n_dyn; }
              sfc_map(static_cast<int>(ny2), static_cast<int>(nx2)) = 0;
            }
        }
      }
      // ★★★ 2026-11-13（**定位"多出 1.5 万格障碍"用**）：把**规划器实际搜索的 a_occ** 落盘。
      //   为什么：日志显示 mask 与静态图的**格数几乎相同**（36,720 vs 36,720），
      //   却报了"新增障碍 15,777 格"——两者不可能同时成立，说明坐标映射有问题
      //   （离线模拟：上下翻转 ⇒ 17,646 格；错 2 格 ⇒ 12,738 格；正确 ⇒ 68 格）。
      //   把 a_occ 存下来，和 mask 叠加就能一眼看出是**翻转**还是**平移**。
      //   周期 30 s，留最近 3 张；参数 dump_grid_every_s:=0 可关。
      {
        static rclcpp::Time last_g{0, 0, RCL_ROS_TIME};
        const auto tg = node_->now();
        if (last_g.nanoseconds() == 0 || (tg - last_g).seconds() > p_dump_grid_every_s_) {
          last_g = tg;
          // ★ 2026-11-13：改用 $HOME，仓库放在哪都能写（原来写死 /home/weicheng/...）
          const char* home = std::getenv("HOME");
          const std::string dir = std::string(home ? home : "/tmp") + "/.ros/rm_tdt_dump";
          const std::string stamp = std::to_string(static_cast<long>(tg.seconds()));
          const std::string pg = dir + "/aocc_" + stamp + ".pgm";
          std::ofstream f(pg, std::ios::binary);
          if (f) {
            f << "P5\n" << W << " " << H << "\n255\n";
            std::vector<unsigned char> buf(static_cast<size_t>(W) * H, 254);
            for (unsigned int yy = 0; yy < H; ++yy)
              for (unsigned int xx = 0; xx < W; ++xx)
                if (a_occ[static_cast<size_t>(yy) * W + xx]) buf[static_cast<size_t>(yy) * W + xx] = 0;
            f.write(reinterpret_cast<const char *>(buf.data()), static_cast<std::streamsize>(buf.size()));
          }
          std::ofstream t(dir + "/aocc_" + stamp + ".txt");
          if (t) {
            t << "W " << W << " H " << H << " res " << res
              << " ox " << ox << " oy " << oy << "\n"
              << "# 行 0 = 栅格 y=0（ox,oy 为栅格左下角世界坐标）\n";
          }
          RCLCPP_WARN(logger_, "[tdt] a_occ 已落盘: %s (%ux%u origin %.2f,%.2f)", pg.c_str(), W, H, ox, oy);
        }
      }
      astar.setMaskMapTimed(mm, "dyn", p_mask_duration_, 16, 0);
      RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 5000,
          "[tdt] 动态掩码并入: 掩码 %dx%d(origin %.2f,%.2f) → 规划 %dx%d；"
          "命中 %d 格，新增障碍 %d 格，自车周围豁免 %d 格",
          MW, MH, mox, moy, W, H, n_hit, n_dyn, n_self);
      if (pub_st_l0_) {
        rm_nav_interfaces::msg::StageStatus st;
        st.header.stamp = node_->now();
        st.stage = 0;
        st.state = (n_hit > 0) ? 1 : 2;              // 2 = 有输出但本帧无占用
        st.detail = (n_hit > 0) ? "局部障碍正常" : "掩码本帧无占用";
        st.keys.resize(3); st.values.resize(3);
        st.keys[0] = "mask_hit"; st.keys[1] = "dyn_new"; st.keys[2] = "mask_w";
        st.values[0] = static_cast<double>(n_hit);
        st.values[1] = static_cast<double>(n_dyn);
        st.values[2] = static_cast<double>(MW);
        pub_st_l0_->publish(st);
      }
    } else if (p_mask_enable_) {
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 5000,
          "[tdt] 掩码未并入：%s", mask_ ? "分辨率不一致" : "还没收到 /plan_mask");
    }

    astar.setCostField(1.0f, [](float x) { return 0.5f / (0.1f + x); });
    astar.initCostMap();

    // ★ 局部坐标 → nav_msgs::Path（加回 origin，frame = cf）
    auto make_path = [&](const std::vector<Eigen::Vector2f>& pts, double lox_, double loy_) {
      nav_msgs::msg::Path p;
      p.header.stamp = node_->now();
      p.header.frame_id = cf;
      p.poses.reserve(pts.size());
      for (const auto& q : pts) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = p.header;
        ps.pose.position.x = static_cast<double>(q.x()) + lox_;
        ps.pose.position.y = static_cast<double>(q.y()) + loy_;
        ps.pose.orientation.w = 1.0;
        p.poses.push_back(ps);
      }
      return p;
    };

    // 世界 → 局部（局部 = 世界 − origin）
    const float lox = static_cast<float>(ox), loy = static_cast<float>(oy);
    auto path = astar.search(Eigen::Vector2f(static_cast<float>(sx) - lox, static_cast<float>(sy) - loy),
                             Eigen::Vector2f(static_cast<float>(gx_solve) - lox, static_cast<float>(gy_solve) - loy));
    if (path.size() < 2 && use_two_level) {
      RCLCPP_WARN(logger_, "[tdt] 局部目标不可达 ⇒ 退回全局目标一次");
      use_two_level = false;
      path = astar.search(Eigen::Vector2f(static_cast<float>(sx) - lox, static_cast<float>(sy) - loy),
                          Eigen::Vector2f(static_cast<float>(gx) - lox, static_cast<float>(gy) - loy));
    }
    if (path.size() < 2) {
      // ★★★ 2026-11-14（B+C，实测依据）：**A* 失败不再返回空路径**。
      //   原后果（446 s 那次）：返回空 ⇒ nav2 `ActionServer Aborting handle` ⇒ BT 放弃整个目标、
      //   再也不调 planner ⇒ 控制器 2 s 后 plan 过期 ⇒ 发 0 ⇒ 车停 67 s。
      //   现在：记一次失败（C：A* 失败也计入 fail_streak，并**立刻**转 ESCAPE，不等 3 次），
      //   采样 16 个方向可用长度（自车 0.25 m 内豁免，与逃逸触发同一套规则），
      //   直接返回一条**到逃逸点的两点直线**（不依赖 A* 也能出路径）⇒ BT 认为规划成功、继续跑。
      ++fail_streak_;
      const double px0 = static_cast<double>(sx), py0 = static_cast<double>(sy);
      const double k_self_clear = 0.25;
      double best_d = 0.0, best_h = escape_heading_;
      for (int k = 0; k < 16; ++k) {
        const double h = k * M_PI / 8.0;
        double d = 0.0;
        for (; d < p_escape_dist_; d += 0.05) {
          const double wx = px0 + std::cos(h) * d, wy = py0 + std::sin(h) * d;
          const int cx = static_cast<int>((wx - ox) / res);
          const int cy = static_cast<int>((wy - oy) / res);
          if (cx < 0 || cy < 0 || cx >= static_cast<int>(W) || cy >= static_cast<int>(H)) break;
          if (d >= k_self_clear && a_occ[static_cast<size_t>(cy) * W + cx]) break;
        }
        if (d > best_d) { best_d = d; best_h = h; }
      }
      escape_heading_ = best_h;
      escape_target_dist_ = (best_d >= p_escape_min_clear_)
          ? p_escape_dist_ : std::max(0.15, std::min(p_escape_dist_, best_d * 0.5));
      escape_start_ = node_->now();
      ++escape_attempts_;
      rec_state_ = REC_ESCAPE;
      const std::string cap2 = (p_max_escape_ > 0) ? std::to_string(p_max_escape_) : std::string("inf");
      RCLCPP_WARN(logger_,
          "[tdt] A* 没找到路径 ⇒ 逃逸（不返回空路径）：方向 %.0f°, 可用 %.2f m, 目标 %.2f m, 失败 %d 次, 尝试 %d/%s",
          best_h * 180.0 / M_PI, best_d, escape_target_dist_, fail_streak_, escape_attempts_, cap2.c_str());
      nav_msgs::msg::Path esc;
      esc.header = start.header;
      const double travel = std::min(escape_target_dist_, std::max(0.30, best_d * 0.5));
      for (int i = 0; i <= 1; ++i) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = esc.header;
        ps.pose.position.x = px0 + (i == 0 ? 0.0 : std::cos(best_h) * travel);
        ps.pose.position.y = py0 + (i == 0 ? 0.0 : std::sin(best_h) * travel);
        ps.pose.orientation.w = 1.0;
        esc.poses.push_back(ps);
      }
      return esc;
    }
    pub_astar_->publish(make_path(path, ox, oy));

    // ④ 简化
    auto path_s = astar.simplifyPath(path, static_cast<float>(p_simplify_));
    path_s = astar.simplifyPathHypot(path_s);
    // ★★ 2026-11-08（修我自己的回归）：原来这里写的是 `<3 点就退回原始 A* 路径`，
    //   结果**把简化到 2 点的成功结果丢掉了** ⇒ 开阔地带的路径也变成 50+ 个 5 cm 控制点
    //   ⇒ 梯形时间被碎段放大 10 倍（日志里大量 `A* 55→简化 55` 就是这么来的）。
    //   正确做法：2 点（一段直线）是合法输入，交给 MinSnap 解单段；只有 <2 才退回。
    if (path_s.size() < 2) path_s = path;
    // ★ 严格校验简化路径：DP 的首末连线判据用了 T-DT 的 2 格容差，可能切过薄墙
    if (!path_clear_static(path_s)) {
      RCLCPP_WARN(logger_, "[tdt] 简化路径穿墙(静态墙) ⇒ 退回 A* 原始栅格路径(%zu 点)", path.size());
      // ★★ 2026-11-10：退回到 **栅格原始路径**后必须先脱密。
      //   0.05 m 间距的 134 点喂给 MinSnap ⇒ 生成 1331 点 / 37 s 的"病态轨迹"，
      //   控制器照着它跑 ⇒ T3 跟踪误差 ⇒ 逃逸失败 ⇒ A* 无解（实测整条链）。
      //   这里重新用更大的阈值简化，点距至少 ~0.3 m 才交给优化器。
      path_s = astar.simplifyPath(path, 0.30f);
      if (path_s.size() < 2) path_s = path;
      const bool raw_bad_static = !path_clear_static(path_s);
      if (raw_bad_static) {
        RCLCPP_ERROR(logger_,
            "[tdt] A* 原始路径穿**静态墙** ⇒ 拒绝本次规划（交给 nav2 重试/恢复）");
        return nav_msgs::msg::Path{};
      }
      if (!path_clear(path_s)) {
        // 只有动态层（ROGMap/mask）挡着 ⇒ 仍发布，让控制器去绕/减速；
        // 否决它会让"感知噪声"变成"车完全不动"，代价更大。
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
            "[tdt] 原始路径与**动态层**相交（非墙）⇒ 仍发布，交给控制器/重规划处理");
      }
    }

    pub_simpl_->publish(make_path(path_s, ox, oy));

    // ⑤ 时间分配
    auto times = MinimumSnap::trapezoidalTimeAllocation(path_s, static_cast<float>(p_max_speed_),
                                                        static_cast<float>(p_max_acc_));

    // ⑥ MinimumSnap
    MinimumSnap ms;
    ms.setOrder(p_order_);
    ms.setMaxDx(p_max_dx_);
    ms.setDt(0.1);
    // ★ 与 A* 同一套局部坐标（origin=0）⇒ SfcSquare / MinimumSnap 的三处约定全部一致
    ms.setMap(sfc_map, static_cast<float>(res), 0.f, 0.f);
    ms.setTL(static_cast<float>(p_tl_));
    // ★★ 2026-11-08（据 doc/Usage.md）：MinimumSnap 对象**默认非严格碰撞检测**，
    //   非严格模式下 `lineInObsticle` 允许线段穿过 ≤2 个障碍栅格
    //   （官方原话："这不是几何意义上的安全距离保证"）⇒ 轨迹可能"擦墙/穿一格"。
    //   开启严格模式：遇到任一障碍格立即判碰撞（配合我们的成功门控 + A* 回退）。
    ms.setStrictCollision(p_strict_collision_);   // 默认 false（上游默认）；严格会几乎全部失败
    MinimumSnap::SolveInput in;
    // ★ 2026-11-10：用 Item 接口（T-DT 提供但我们之前没用）
    //   · 起点：钉住"当前速度"，让轨迹从巡航状态接着走（不再每段从 0 起步）
    //   · 中途点：作为必经点（autoCorridor=true ⇒ 仍自动生成走廊）
    //   · 末点：**不钉速度**（两级规划里它不是真终点，不该停）
    {
      // 用"上次规划的起点"差分估计当前速度（不改状态，状态由下面 init_vel 块维护）
      Eigen::Vector2f v_now(0.f, 0.f);
      {
        const auto tnow = node_->now();
        if (last_plan_valid_) {
          const double dt = (tnow - last_plan_time_).seconds();
          if (dt > 0.05 && dt < 3.0) {
            v_now = Eigen::Vector2f(
                static_cast<float>((sx - last_plan_x_) / dt),
                static_cast<float>((sy - last_plan_y_) / dt));
            if (!std::isfinite(v_now.norm()) || v_now.norm() > 5.0f) v_now.setZero();
          }
        }
      }
      const float vn = v_now.norm();
      if (p_use_items_ && path_s.size() >= 2) {
        std::vector<MinimumSnap::SolveInput::Item> items(path_s.size());
        for (size_t i = 0; i < path_s.size(); ++i) {
          items[i].xy = path_s[i];
          items[i].useVxvy = false;
          items[i].autoCorridor = true;
        }
        if (std::isfinite(vn) && vn > 0.15f && vn < 5.0f) {
          items.front().vxvy = v_now;
          items.front().useVxvy = true;      // 起点速度约束（= setInitVel 的强化版）
        }
        in.setItems(items);
      } else {
        in.setPath(path_s);
      }
    }
    in.setCollisionCheckIter(p_collision_iter_);
    in.setMaxSpeed(static_cast<float>(p_max_speed_));
    in.setMaxAcc(static_cast<float>(p_max_acc_));
    in.setMaxCorridorRange(static_cast<float>(p_max_range_));
    in.setCorridorShrink(static_cast<float>(p_shrink_));
    in.setNormTime(p_norm_time_);
    in.setBackend(MinimumSnap::Backend::OSQPCorridor);
    // ★★ 2026-11-08（据官方 doc/Usage.md）：`setPath()` 的**起点速度默认自由**，
    //   只有显式 setInitVel 才会把它钉住。上游 main.cpp 传 {0,0} 是因为它从静止起飞；
    //   我们车在跑，每 1 s 重规划一次，传 0 等于**强迫每段轨迹从静止起步**
    //   ⇒ 轨迹首段被压慢、QP 也更容易不可行（实测长绕行 OSQP 未收敛）。
    //   init_vel_mode: 0=自由（默认，最接近文档语义）1=零（旧行为）2=按两次调用差分估计
    if (p_init_vel_mode_ == 1) {
      in.setInitVel({0.f, 0.f});
    } else if (p_init_vel_mode_ == 2) {
      const auto now = node_->now();
      if (last_plan_valid_) {
        const double dt = (now - last_plan_time_).seconds();
        if (dt > 0.05 && dt < 3.0) {
          const Eigen::Vector2f v((static_cast<float>(sx) - last_plan_x_) / static_cast<float>(dt),
                                  (static_cast<float>(sy) - last_plan_y_) / static_cast<float>(dt));
          const float n = v.norm();
          if (std::isfinite(n) && n < 3.f) in.setInitVel(n > 1e-3f ? v : Eigen::Vector2f(0.f, 0.f));
        }
      }
      last_plan_time_ = now; last_plan_x_ = static_cast<float>(sx); last_plan_y_ = static_cast<float>(sy);
      last_plan_valid_ = true;
    }   // mode 0: 不调用 ⇒ 起点速度自由
    if (!times.empty()) in.setTimeAllocated(times);

    MinimumSnap::SolveOutput res_solve;
    if (path_s.size() >= 2) {          // ★ 允许单段（2 点）
      res_solve = ms.solve(in);
    } else {
      RCLCPP_DEBUG(logger_, "[tdt] 路径仅 %zu 点 ⇒ 跳过 MinSnap（用 A* + 梯形时间）", path_s.size());
    }
    std::vector<Eigen::Vector2f> traj_pts;
    // ★ 与 T-DT 一致：MinSnap 只要产出路径就用（它有 success 标志，但 demo 看的是 path）
    // ★★ 2026-11-08：**只有 snap 成功才用 MinSnap 的轨迹**。
    //   失败时（OSQP 未收敛 / 走廊不可行 / 碰撞未消除）它返回的是**部分解**，
    //   这条轨迹既可能撞墙、也和 A* 的绕行不一致 ⇒ 会表现为"路径乱绕、跟 PGM 无关"。
    //   失败时回退到 A* 的简化路径（一定不撞墙），时间按弧长分配。
    // ★ 2026-11-10：用 YAstar::getSDF() 报告轨迹的最小净空（T-DT 提供但我们之前没用）
    //   sdf(y,x) = 到最近障碍的距离（米）。取轨迹上每点在 sdf 上的值，报最小值。
    if (p_log_sdf_ && res_solve.success && res_solve.path.size() >= 2) {
      const auto sdf = astar.getSDF();
      const int rows = static_cast<int>(sdf.rows()), cols = static_cast<int>(sdf.cols());
      double min_clr = 1e9;
      for (const auto& q : res_solve.path) {
        const int cx = static_cast<int>((q.x()) / res);   // 局部坐标（origin=0）
        const int cy = static_cast<int>((q.y()) / res);
        if (cx < 0 || cy < 0 || cx >= cols || cy >= rows) continue;
        const double d = static_cast<double>(sdf(cy, cx));
        if (std::isfinite(d) && d < min_clr) min_clr = d;
      }
      if (min_clr < 1e8) {
        RCLCPP_INFO(logger_, "[tdt] 轨迹净空(SDF): 最小 %.2f m（%s）", min_clr,
                    min_clr < 0.30 ? "偏紧" : (min_clr < 0.60 ? "正常" : "宽松"));
      }
    }

    // ★ 2026-11-10：**病态轨迹上限**。平滑器偶尔会产出 1000+ 点、几十秒的轨迹
    //   （起因是喂进 0.05 m 间距的栅格路径），这种轨迹控制器跟不住，
    //   会把 FSM 推进"跟踪误差 → 逃逸 → 无解"。宁可判失败让 nav2 重试。
    if (res_solve.success) {
      double traj_len = 0.0;
      for (size_t i = 1; i < res_solve.path.size(); ++i)
        traj_len += (res_solve.path[i] - res_solve.path[i - 1]).norm();
      const double dur = res_solve.time;      // 求解耗时不是时长 ⇒ 用时分配近似
      (void)dur;
      if (traj_len > 30.0 && p_two_level_) {
        RCLCPP_WARN(logger_, "[tdt] 平滑轨迹长度 %.1f m 不合理 ⇒ 拒绝（避免病态轨迹）", traj_len);
        return nav_msgs::msg::Path{};
      }
    }

    if (res_solve.success && res_solve.path.size() >= 2 && path_clear(res_solve.path)) {
      traj_pts = res_solve.path;
    } else {
      if (res_solve.success)
        RCLCPP_WARN(logger_, "[tdt] MinSnap 轨迹**穿墙** ⇒ 拒绝，回退 A* 简化路径(%zu 点)", path_s.size());
      else {
        // ★★★ 2026-11-12：**失败时把简化后的控制点打出来**。
        //   为什么（日志证据）：`MinimumSnap 失败(snap=0, iter=6, 1.4ms)` 是**快速失败**
        //   ⇒ 是 QP 不可行（不是迭代超时）。而 T-DT 的方形走廊"**并不保证走廊相接**"
        //   （doc/MinimumSnap.md:196）⇒ 控制点太稀（如 A* 215→简化 4，段长 2~3 m）时
        //   相邻控制点的方形走廊接不上 ⇒ 硬约束矛盾 ⇒ 不可行。
        //   这里把点打出来，才能用离线工具复现"到底是哪一段走廊接不上"。
        std::string pts;
        const size_t n_show = std::min<size_t>(path_s.size(), 8);
        for (size_t i = 0; i < n_show; ++i) {
          char buf[64];
          std::snprintf(buf, sizeof(buf), " (%.2f,%.2f)", path_s[i].x(), path_s[i].y());
          pts += buf;
        }
        RCLCPP_WARN(logger_,
            "[tdt] MinimumSnap 失败(snap=0, iter=%d, %.1fms) ⇒ 回退 A* 简化路径(%zu 点)"
            "；控制点%s%s；走廊 shrink=%.2f range=%.2f",
            res_solve.iter, res_solve.time * 1e3, path_s.size(), pts.c_str(),
            path_s.size() > n_show ? " …" : "", p_shrink_, p_max_range_);

        // ★★★ 2026-11-12（晚）：**失败现场快照**（限频 5 s）。
        //   为什么：离线 `minisnap_points_probe` 只喂静态图时**复现不出**这些失败
        //   （4 组真实点全部 snap=1），说明差异在 **mask 叠加后的栅格**。
        //   这里把「合并后的规划栅格（含 mask）」+「控制点」落盘，
        //   下次可直接用同一张图离线复现，不用再靠猜。
        {
          static rclcpp::Time last_dump{0, 0, RCL_ROS_TIME};
          const auto tnow_d = node_->now();
          if (last_dump.nanoseconds() == 0 || (tnow_d - last_dump).seconds() > 5.0) {
            last_dump = tnow_d;
            // ★ 2026-11-13：改用 $HOME，仓库放在哪都能写（原来写死 /home/weicheng/...）
          const char* home = std::getenv("HOME");
          const std::string dir = std::string(home ? home : "/tmp") + "/.ros/rm_tdt_dump";
            const std::string stamp = std::to_string(static_cast<long>(tnow_d.seconds()));
            const std::string pgm = dir + "/snapfail_" + stamp + ".pgm";
            const std::string txt = dir + "/snapfail_" + stamp + ".txt";
            // 栅格：a_occ 里 1=障碍 → PGM 0；否则 255（**注意 PGM 第 0 行是顶部，
            // 这里按规划器局部坐标直接写，读回时用探针的 --noflip**）
            std::ofstream f(pgm, std::ios::binary);
            if (f) {
              f << "P5\n" << W << " " << H << "\n255\n";
              std::vector<unsigned char> buf(static_cast<size_t>(W) * H, 255);
              for (unsigned int yy = 0; yy < H; ++yy)
                for (unsigned int xx = 0; xx < W; ++xx)
                  if (a_occ[static_cast<size_t>(yy) * W + xx]) buf[static_cast<size_t>(yy) * W + xx] = 0;
              f.write(reinterpret_cast<const char *>(buf.data()), static_cast<std::streamsize>(buf.size()));
            }
            std::ofstream t(txt);
            if (t) {
              t << "# points (local grid coords, origin=0) 供 minisnap_points_probe 使用\n";
              for (const auto & q : path_s) t << q.x() << "," << q.y() << " ";
              t << "\n# pgm=" << pgm << "  (用 --noflip 读)\n";
            }
            RCLCPP_WARN(logger_, "[tdt] 失败现场已存: %s / %s", pgm.c_str(), txt.c_str());
          }
        }
      }
      traj_pts = path_s;
    }
    // ★ 局部 → 世界（+origin）
    for (auto& p : traj_pts) { p.x() += lox; p.y() += loy; }
    pub_smooth_->publish(make_path(traj_pts, 0.0, 0.0));   // 已是世界坐标

    // ⑦ 轨迹时间：**MinSnap 的分段时长**（lineDecoder 每段 11 点、拼接时去掉每段末点 ⇒ 每段 10 点）
    double total_t = 0.0;
    for (double t : times) total_t += t;
    if (total_t <= 1e-3) total_t = 1.0;
    // ★ 优先用 MinSnap 返回的**最终**分段时长（本地补丁字段）
    const size_t segs_guess = (traj_pts.size() > 1) ? (traj_pts.size() - 1) / 10 : 0;
    const std::vector<double>& seg_times =
        (res_solve.times.size() == segs_guess) ? res_solve.times : times;
    const size_t segs = seg_times.size();
    const bool pts_match = (traj_pts.size() == 10 * segs + 1);
    std::vector<double> t_of(traj_pts.size(), 0.0);
    if (pts_match) {
      double acc = 0.0;
      for (size_t i = 0; i < traj_pts.size(); ++i) {
        const size_t seg = std::min(segs - 1, i / 10);
        const size_t k = i - seg * 10;
        t_of[i] = acc + seg_times[seg] * static_cast<double>(k) / 10.0;
        if (k == 9) acc += seg_times[seg];
      }
      total_t = acc + seg_times.back();
    } else {
      // 交替迭代插了中点 ⇒ 段数变了，退回弧长比例（并在日志里标出）
      std::vector<double> s(traj_pts.size(), 0.0);
      for (size_t i = 1; i < traj_pts.size(); ++i)
        s[i] = s[i - 1] + (traj_pts[i] - traj_pts[i - 1]).norm();
      const double s_total = std::max(1e-9, s.back());
      for (size_t i = 0; i < traj_pts.size(); ++i) t_of[i] = total_t * s[i] / s_total;
      RCLCPP_DEBUG(logger_, "[tdt] 段数不匹配（path=%zu, segs=%zu）⇒ 时间用弧长比例", traj_pts.size(), segs);
    }

    // ★★ 2026-11-09（步骤 1）：把骨架**尾段**接到局部轨迹后面。
    //   两个作用：
    //     ① nav2 的 PlanGoalChecker 取 `/plan` 的**末点**当目标，不接尾段会在 8 m 处误判到达；
    //     ② RViz/控制器能看到完整路线（控制器只跟踪前端 4 m，不受影响）。
    if (use_two_level && backbone_valid_ && !traj_pts.empty()) {
      double t_last = t_of.empty() ? 0.0 : t_of.back();
      Eigen::Vector2f last = traj_pts.back();
      size_t appended = 0;
      // ★ 尾段速度用**保守值**：若按 p_max_speed（可能 10 m/s）折算时间，
      //   控制器在接缝附近 `v(s+0.4)` 会突然拿到 10 m/s 的参考 ⇒ 速度跳变。
      // ★ 2026-11-10：尾段速度策略可配。
      //   tail_decel = 0  ⇒ **全程全速**（默认，比赛优先时间）
      //   tail_decel > 0  ⇒ 按 sqrt(2·a·剩余) 递减，末端减速
      // ★ 2026-11-10（用户要求）：**不再自己压速度**。
      //   速度由 T-DT 的时间分配决定；尾段是算法给的直线延伸，
      //   直接沿用规划器的 max_speed，而不是我们自己拍的 1.5。
      const double v_tail = (p_tail_speed_ > 0.0)
          ? p_tail_speed_ : static_cast<double>(p_max_speed_);
      const double s_end = backbone_s_.empty() ? s_head : backbone_s_.back();
      for (size_t i = 0; i < backbone_.size(); ++i) {
        if (backbone_s_[i] <= s_head + 0.05) continue;          // 局部目标之前的不接
        const Eigen::Vector2f& q = backbone_[i];
        if ((q - last).norm() < 1e-3) continue;
        const double d = (q - last).norm();
        double v_here = v_tail;
        if (p_tail_decel_ > 0.0) {
          const double s_remain = std::max(0.0, s_end - backbone_s_[i]);
          const double v_lim = std::sqrt(std::max(0.0, 2.0 * p_tail_decel_ * s_remain));
          v_here = std::max(0.20, std::min(v_tail, v_lim));
        }
        t_last += d / v_here;
        traj_pts.push_back(q); t_of.push_back(t_last);
        last = q; ++appended;
      }
      if (appended > 0)
        RCLCPP_DEBUG(logger_, "[tdt] 骨架尾段拼接 %zu 点（局部目标 s=%.1f m）", appended, s_head);
    }

    if (!t_of.empty()) total_t = t_of.back();   // ★ 拼接尾段后日志时长要跟着更新

    rm_plan_msgs::msg::TrajectoryPlan plan;
    plan.header = out.header;
    plan.header.stamp = node_->now();   // ★ 用发布时刻，便于消费端算 age
    plan.traj_source = res_solve.success ? 1 : 3;   // 1=minimum_snap, 3=A*+弧长时间(回退)
    plan.corridor_shrink_mm = static_cast<uint16_t>(std::lround(p_shrink_ * 1000.0));
    plan.grid_resolution_mm = static_cast<uint16_t>(std::lround(res * 1000.0));
    for (size_t i = 0; i < traj_pts.size(); ++i) {
      plan.t.push_back(t_of[i]);
      plan.x.push_back(traj_pts[i].x());
      plan.y.push_back(traj_pts[i].y());
      plan.vx.push_back(0.0);
      plan.vy.push_back(0.0);
    }
    for (size_t i = 0; i + 1 < traj_pts.size(); ++i) {
      const double dt = std::max(1e-6, plan.t[i + 1] - plan.t[i]);
      plan.vx[i] = (plan.x[i + 1] - plan.x[i]) / dt;
      plan.vy[i] = (plan.y[i + 1] - plan.y[i]) / dt;
    }
    if (traj_pts.size() > 1) { plan.vx.back() = plan.vx[plan.vx.size() - 2]; plan.vy.back() = plan.vy[plan.vy.size() - 2]; }

    // ⑧ 走廊（沿简化路径取，再映射到世界）
    // ★ 与 T-DT 一致：走廊用 **MinSnap 内部**算出的那份（SolveOutput.corridor）；
    //   没有（失败/未跑）时才自己调 getCorridor 兜底。
    if (!res_solve.corridor.empty()) {
      for (const auto& b : res_solve.corridor) {   // 局部 → 世界
        plan.corridor.push_back(b[0] + lox); plan.corridor.push_back(b[1] + loy);
        plan.corridor.push_back(b[2] + lox); plan.corridor.push_back(b[3] + loy);
      }
    } else {
      const auto& cor_src = (path_s.size() >= 3) ? path_s : path;
      auto cor = sfc.getCorridor(cor_src, static_cast<float>(p_max_range_), static_cast<float>(p_shrink_));
      for (const auto& b : cor.corridor) {          // 局部 → 世界
        plan.corridor.push_back(b[0] + lox); plan.corridor.push_back(b[1] + loy);
        plan.corridor.push_back(b[2] + lox); plan.corridor.push_back(b[3] + loy);
      }
    }
    // ★ 转回请求帧（map）：轨迹点逐点转；走廊按“中心转 + 保持半宽”近似
    if (!out.header.frame_id.empty() && out.header.frame_id != cf) {
      try {
        const auto T = tf_->lookupTransform(out.header.frame_id, cf, tf2::TimePointZero);
        auto apply = [&](double x, double y, double* ox, double* oy) {
          const double c = std::cos(T.transform.rotation.z), s2 = std::sin(T.transform.rotation.z);
          // 用四元数更稳，但这里只做平面旋转+平移（本栈两帧间只差 yaw）
          const auto& q = T.transform.rotation;
          const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
          const double cc = std::cos(yaw), ss = std::sin(yaw);
          *ox = cc * x - ss * y + T.transform.translation.x;
          *oy = ss * x + cc * y + T.transform.translation.y;
          (void)c; (void)s2;
        };
        for (size_t i = 0; i < plan.x.size(); ++i) {
          double nx, ny; apply(plan.x[i], plan.y[i], &nx, &ny); plan.x[i] = nx; plan.y[i] = ny;
          double nvx, nvy; apply(plan.x[i] + plan.vx[i] * 1e-3, plan.y[i] + plan.vy[i] * 1e-3, &nvx, &nvy);
          plan.vx[i] = (nvx - nx) / 1e-3; plan.vy[i] = (nvy - ny) / 1e-3;
        }
        for (size_t j = 0; j + 3 < plan.corridor.size(); j += 4) {
          const double cx = 0.5 * (plan.corridor[j] + plan.corridor[j + 2]);
          const double cy = 0.5 * (plan.corridor[j + 1] + plan.corridor[j + 3]);
          const double hx = 0.5 * (plan.corridor[j + 2] - plan.corridor[j]);
          const double hy = 0.5 * (plan.corridor[j + 3] - plan.corridor[j + 1]);
          double nx, ny; apply(cx, cy, &nx, &ny);
          plan.corridor[j] = nx - hx; plan.corridor[j + 1] = ny - hy;
          plan.corridor[j + 2] = nx + hx; plan.corridor[j + 3] = ny + hy;
        }
        plan.header.frame_id = out.header.frame_id;
      } catch (const std::exception& e) {
        RCLCPP_WARN(logger_, "[tdt] 转回请求帧失败（plan 留在 %s）: %s", cf.c_str(), e.what());
        plan.header.frame_id = cf;
      }
    } else {
      plan.header.frame_id = cf;
    }
    plan_pub_->publish(plan);

    // ★ 走廊可视化：plan.corridor 每 4 个数一个方框 [xmin,ymin,xmax,ymax]（已在请求帧）
    if (pub_corridor_ && !plan.corridor.empty() && plan.corridor.size() % 4 == 0) {
      visualization_msgs::msg::MarkerArray ma;
      visualization_msgs::msg::Marker del;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      ma.markers.push_back(del);
      int id = 0;
      for (size_t i = 0; i + 3 < plan.corridor.size(); i += 4) {
        const double x0 = plan.corridor[i],   y0 = plan.corridor[i + 1];
        const double x1 = plan.corridor[i + 2], y1 = plan.corridor[i + 3];
        visualization_msgs::msg::Marker m;
        m.header = out.header;
        m.ns = "corridor";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = 0.5 * (x0 + x1);
        m.pose.position.y = 0.5 * (y0 + y1);
        m.pose.position.z = 0.05;
        m.pose.orientation.w = 1.0;
        m.scale.x = std::max(0.02, std::abs(x1 - x0));
        m.scale.y = std::max(0.02, std::abs(y1 - y0));
        m.scale.z = 0.12;
        m.color.r = 0.15f; m.color.g = 0.45f; m.color.b = 1.0f; m.color.a = 0.25f;
        m.lifetime = rclcpp::Duration::from_seconds(0.6);
        ma.markers.push_back(m);
        // 边框（同色更亮，便于看清方框）
        visualization_msgs::msg::Marker e = m;
        e.id = id++;
        e.type = visualization_msgs::msg::Marker::LINE_STRIP;
        e.scale.x = 0.02; e.scale.y = 0.0; e.scale.z = 0.0;
        e.color.a = 0.9f;
        e.points.resize(5);
        const double zs[5] = {x0, x1, x1, x0, x0};
        const double zt[5] = {y0, y0, y1, y1, y0};
        for (int k = 0; k < 5; ++k) {
          e.points[k].x = zs[k]; e.points[k].y = zt[k]; e.points[k].z = 0.11;
        }
        ma.markers.push_back(e);
      }
      pub_corridor_->publish(ma);
    }

    // ⑨ 返回 Path 给 nav2
    for (const auto& p : traj_pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      ps.pose.position.x = p.x(); ps.pose.position.y = p.y(); ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      out.poses.push_back(ps);
    }
    // ★★ 2026-11-09：**恢复检测 T1/T2**（在拿到规划结果之后判断）
    if (p_recovery_enable_) {
      const double px = static_cast<double>(sx), py = static_cast<double>(sy);
      const auto now_t = node_->now();
      // T1：无进度（跟踪态下 2 s 位移 < 5 cm）
      last_plan_len_ = 0.0;
    for (size_t i = 1; i < traj_pts.size(); ++i)
      last_plan_len_ += (traj_pts[i] - traj_pts[i - 1]).norm();
    // ★ A3：记录计划**初始方向**（取离起点约 1 m 的那一点）
    last_plan_dir_ = Eigen::Vector2f(0.f, 0.f);
    {
      const Eigen::Vector2f s0(static_cast<float>(sx), static_cast<float>(sy));
      Eigen::Vector2f pick = traj_pts.empty() ? s0 : traj_pts.back();
      for (const auto& q : traj_pts) { if ((q - s0).norm() >= 1.0f) { pick = q; break; } }
      const Eigen::Vector2f d = pick - s0;
      if (d.norm() > 0.2f) last_plan_dir_ = d.normalized();
    }
    if (last_pos_.x() > -1e8f) {
        const double moved = std::hypot(px - last_pos_.x(), py - last_pos_.y());
        if (moved > p_stuck_eps_) {
          last_pos_ = Eigen::Vector2f(px, py); last_move_time_ = now_t;
          // 正常前进且距上次逃逸 >5 s ⇒ 复位尝试计数
          if (rec_state_ == REC_NORMAL && (now_t - last_escape_end_).seconds() > 5.0)
            escape_attempts_ = 0;
        }
      } else { last_pos_ = Eigen::Vector2f(px, py); last_move_time_ = now_t; }
      // ★ A2：排除「已到终点附近」「计划本身没有前进段」这两类误触发
      const double dist_goal_now = std::hypot(gx - px, gy - py);
      const bool t1 = (now_t - last_move_time_).seconds() > p_stuck_time_ &&
                      dist_goal_now > 1.5 &&
                      last_plan_len_ > 0.5;
      // T2：连续失败（5 s 内 >=3 次）
      if (!res_solve.success) {
        if (fail_streak_ == 0) first_fail_time_ = now_t;
        ++fail_streak_;
      } else {
        fail_streak_ = 0;
      }
      const bool t2 = (fail_streak_ >= 3) &&
                      (now_t - first_fail_time_).seconds() < 5.0;

      // ★ A3：**跟踪误差**——实际位移方向与计划初始方向夹角 > 60° 持续约 2 s
      if ((now_t - prev_pos_time_).seconds() >= 1.0 || prev_pos_.x() < -1e8f) {
        const double ddx = px - prev_pos_.x(), ddy = py - prev_pos_.y();
        const double disp = std::hypot(ddx, ddy);
        if (disp > 0.30 && last_plan_dir_.norm() > 0.5) {
          const double cosang = (ddx / disp) * last_plan_dir_.x() + (ddy / disp) * last_plan_dir_.y();
          if (cosang < 0.5) ++t3_count_;        // > 60°
          else t3_count_ = 0;
        } else {
          t3_count_ = 0;
        }
        prev_pos_ = Eigen::Vector2f(px, py);
        prev_pos_time_ = now_t;
      }
      const bool t3 = t3_count_ >= 4;          // 2 Hz ⇒ 约 2 s

      if (t1 || t2 || t3) {
        // 逃逸方向：在 a_occ 上采样 16 个方向，选"能走最远"的那个
        //（不用 ESDF，避免依赖 esdf_en；需要时也可换成 ROGMap 的梯度）
        double best_d = -1.0, best_h = 0.0;
        // ★★★ 2026-11-12（晚，**实测修正**）：**逃逸采样必须豁免自车所在区域**。
        //   原来循环从 `d = 0.0` 开始、第一格就是机器人中心格 ⇒ 只要那一格在 a_occ 里
        //   是障碍（贴墙 / 落在 dilate_cells=2 的膨胀区内 / mask 幽灵格），
        //   16 个方向全部在第 0 步 break ⇒ best_d = 0 < 0.35 ⇒
        //   `[tdt][恢复] 16 个方向都 <0.35 m ⇒ FAILED` ⇒ **原地停车**。
        //   实测：两次运行各出现 3~6 次，全部是这一条（不是真的四面被堵）。
        //   `path_clear_static` 早就为同一问题加了 0.25 m 起点豁免（见 planner.cpp:320），
        //   这里保持一致：**0.25 m 以内不看占用**（自车半径 0.228 + 半格）。
        //   注意：0.25 m 之外仍然正常判占用 ⇒ 真被围死时 best_d 停在 0.25 < 0.35，
        //   依然会判 FAILED（保留原有的"四面被堵"检测能力）。
        const double k_self_clear = 0.25;
        const int self_cx = static_cast<int>((px - ox) / res);
        const int self_cy = static_cast<int>((py - oy) / res);
        const bool self_cell_occ =
          self_cx >= 0 && self_cy >= 0 && self_cx < static_cast<int>(W) &&
          self_cy < static_cast<int>(H) && a_occ[static_cast<size_t>(self_cy) * W + self_cx];
        for (int k = 0; k < 16; ++k) {
          const double h = k * M_PI / 8.0;
          double d = 0.0;
          for (; d < p_escape_dist_; d += 0.05) {
            const double wx = px + std::cos(h) * d, wy = py + std::sin(h) * d;
            const int cx = static_cast<int>((wx - ox) / res);
            const int cy = static_cast<int>((wy - oy) / res);
            if (cx < 0 || cy < 0 || cx >= static_cast<int>(W) || cy >= static_cast<int>(H)) break;
            if (d >= k_self_clear && a_occ[static_cast<size_t>(cy) * W + cx]) break;
          }
          if (d > best_d) { best_d = d; best_h = h; }
        }
        if (self_cell_occ) {
          RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 3000,
              "[tdt][恢复] 自车所在格被判占用（贴墙/膨胀区内）⇒ 逃逸采样已豁免 %.2f m 内",
              k_self_clear);
        }
        // 失败过一次就转到垂直方向尝试（左右交替）
        if (escape_attempts_ % 2 == 1) best_h += M_PI / 2.0;
        else if (escape_attempts_ >= 2) best_h -= M_PI / 2.0;
          if (best_d < p_escape_min_clear_ && !p_escape_allow_enclosed_) {
            // 旧行为（需 escape_allow_enclosed:=false）：四面都堵 ⇒ 判失败
            rec_state_ = REC_FAILED;
            RCLCPP_ERROR(logger_, "[tdt][恢复] 16 个方向都 <%.2f m ⇒ FAILED（停车）", p_escape_min_clear_);
          } else {
            escape_heading_ = best_h;
            escape_start_ = now_t;
            // ★ 被围住 ⇒ 缩短逃逸距离（best_d/2，最小 0.15 m）⇒ 原地小范围来回动
            escape_target_dist_ = (best_d >= p_escape_min_clear_)
                ? p_escape_dist_
                : std::max(0.15, std::min(p_escape_dist_, best_d * 0.5));
            ++escape_attempts_;
            rec_state_ = REC_ESCAPE;
            fail_streak_ = 0;
            const std::string cap = (p_max_escape_ > 0) ? std::to_string(p_max_escape_) : std::string("inf");
            RCLCPP_WARN(logger_, "[tdt][恢复] 触发(%s%s%s) ⇒ 方向 %.0f°, 可用 %.2f m, 目标 %.2f m (尝试 %d/%s)",
                        t1 ? "T1无进度" : "", t2 ? "T2连续失败" : "", t3 ? "T3跟踪误差" : "",
                        best_h * 180.0 / M_PI, best_d, escape_target_dist_, escape_attempts_, cap.c_str());
          }
          if (p_max_escape_ > 0 && escape_attempts_ > p_max_escape_) {
            rec_state_ = REC_FAILED;
            RCLCPP_ERROR(logger_, "[tdt][恢复] 超过 %d 次尝试 ⇒ FAILED（停车）", p_max_escape_);
          }
      }
    }

    if (pub_st_l1_) {
      rm_nav_interfaces::msg::StageStatus st;
      st.header.stamp = node_->now();
      st.stage = 1;
      st.state = 1;
      st.detail = "plan ok";
      st.keys.resize(2); st.values.resize(2);
      st.keys[0]="points"; st.keys[1]="corridor";
      st.values[0]=static_cast<double>(traj_pts.size());
      st.values[1]=static_cast<double>(plan.corridor.size() / 4);
      pub_st_l1_->publish(st);
    }
    RCLCPP_INFO(logger_, "[tdt] plan: %zu 点, 走廊 %zu, 时长 %.2f s, A* %zu→简化 %zu, snap=%d(iter=%d, %.1fms)%s",
                plan.t.size(), plan.corridor.size() / 4, total_t, path.size(), path_s.size(),
                (int)res_solve.success, res_solve.iter, res_solve.time * 1e3,
                use_two_level ? " [两级:骨架+局部8m]" : "");
    return out;
  }

 private:
  // 从 map_server 的 /map（OccupancyGrid）建二值图 + pad P 格边框
  void buildFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& m) {
    if (!m || m->info.width == 0 || m->info.height == 0) return;
    const int P = 2;
    const int w = static_cast<int>(m->info.width), h = static_cast<int>(m->info.height);
    mw_ = w + 2 * P; mh_ = h + 2 * P;
    mres_ = static_cast<float>(m->info.resolution);
    mox_ = static_cast<float>(m->info.origin.position.x) - P * mres_;
    moy_ = static_cast<float>(m->info.origin.position.y) - P * mres_;
    mocc_.assign(static_cast<size_t>(mw_) * mh_, 1);
    msfc_.assign(static_cast<size_t>(mw_) * mh_, 0);
    int n = 0;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const int8_t v = m->data[static_cast<size_t>(y) * w + x];
        // ★★ 2026-11-08（修正回归）：OccupancyGrid 的语义是 **0=free / 100=occupied / -1=unknown**，
        //   和 PGM 像素正好相反！之前照抄 T-DT 的 `==0` 会把整张图判成障碍（实测障碍数 175280 = 全图）。
        const bool occ = (v >= 65);
        const size_t idx = static_cast<size_t>(y + P) * mw_ + (x + P);
        mocc_[idx] = occ ? 1 : 0;
        msfc_[idx] = occ ? 0 : 255;
        n += occ ? 1 : 0;
      }
    map_loaded_ = true;
    RCLCPP_INFO(logger_, "[tdt] /map 先验图: %dx%d(+pad2) @ %.3f m, origin (%.2f, %.2f), 障碍 %d 格",
                mw_, mh_, mres_, mox_, moy_, n);
  }

  // 读 map_server 的 yaml + PGM（P5），转二值 + pad P 格边框
  void loadMapFile(const std::string& yaml_path) {
    std::ifstream yf(yaml_path);
    if (!yf) { RCLCPP_ERROR(logger_, "[tdt] 打不开 %s", yaml_path.c_str()); return; }
    double res = 0.05, ox = 0, oy = 0; std::string image;
    std::string line;
    while (std::getline(yf, line)) {
      std::istringstream is(line); std::string k;
      if (!(is >> k)) continue;
      if (k.rfind("resolution:", 0) == 0) is >> res;
      else if (k.rfind("origin:", 0) == 0) {
        std::string rest = line.substr(line.find('['));
        for (char& c : rest) if (c == '[' || c == ']' || c == ',') c = ' ';
        std::istringstream os(rest); os >> ox >> oy;
      } else if (k.rfind("image:", 0) == 0) { is >> image; }
    }
    const std::string base = yaml_path.substr(0, yaml_path.find_last_of('/') + 1);
    std::ifstream img(base + image, std::ios::binary);
    if (!img) { RCLCPP_ERROR(logger_, "[tdt] 打不开 %s%s", base.c_str(), image.c_str()); return; }
    std::string magic; int w = 0, h = 0, mx = 0;
    img >> magic >> w >> h >> mx; img.get();
    if (magic != "P5" || w <= 0 || h <= 0) { RCLCPP_ERROR(logger_, "[tdt] 不是 P5 PGM"); return; }
    std::vector<unsigned char> px(static_cast<size_t>(w) * h);
    img.read(reinterpret_cast<char*>(px.data()), static_cast<std::streamsize>(px.size()));

    const int P = 2;                      // 边框（_getBoundSquare2 不查边界）
    mw_ = w + 2 * P; mh_ = h + 2 * P;
    mres_ = static_cast<float>(res);
    mox_ = static_cast<float>(ox) - P * mres_;
    moy_ = static_cast<float>(oy) - P * mres_;
    mocc_.assign(static_cast<size_t>(mw_) * mh_, 1);      // 边框=障碍
    msfc_.assign(static_cast<size_t>(mw_) * mh_, 0);
    int n_occ = 0;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        // map_server trinary：0=障碍(黑)，其余(205/254)=自由；未知(205)按 free 处理（与 nav2 一致）
        // ★ 2026-11-08：与 T-DT 逐位一致 —— `pixel == 0` 才算障碍（205 未知/254 自由都算自由）
        const bool occ = px[static_cast<size_t>(y) * w + x] == 0;
        // ★★★ 2026-11-13（**实测定位到的根因**）：**PGM 的行要翻转**。
        //   PGM 第 0 行是**图像顶部**，而栅格索引 y 表示**世界 y 向上**（y = oy + (y+0.5)·res）。
        //   原来直接 `idx = (y+P)` ⇒ **整张静态图上下镜像**。
        //   证据（实测，`.tmp_diag/dump/aocc_29.pgm` vs `mask_51.pgm`）：
        //     · 规划器实际搜索的 a_occ 有 56,197 格障碍，mask 只有 36,788；
        //     · 只有"上下翻转"假设下两者才对得上（IoU 0.653；不翻转 0.09）；
        //     · 翻转映射后 mask 落在 a_occ 上只差 **68 格**（这是真实差异）；
        //   后果：静态墙是镜像的，而 mask 按**正确的世界坐标**并入 ⇒
        //     同一堵墙出现两份（一份镜像、一份正确）⇒ 每帧"新增障碍 15,777 格"、
        //     通道被夹死 ⇒ A* 只能规划 0.22 m ⇒ **车走不出去**。
        //   （旧图 `RMUC2027_v2` 场地上下近似对称，所以这个 bug 一直没被发现。）
        const size_t idx = static_cast<size_t>((h - 1 - y) + P) * mw_ + (x + P);
        mocc_[idx] = occ ? 1 : 0;
        msfc_[idx] = occ ? 0 : 255;
        n_occ += occ ? 1 : 0;
      }
    map_loaded_ = true;
    RCLCPP_INFO(logger_, "[tdt] 先验图已载入: %dx%d(+pad2) @ %.3f m, origin (%.2f, %.2f), 障碍 %d 格",
                mw_, mh_, mres_, mox_, moy_, n_occ);
    // ★ 朝向自检：把几个**已知世界坐标**的占用打出来，运行日志里就能看出有没有镜像。
    //   判据（用 cad_v3_edit 这张图核过）：出生点 (0,0) 必须是 0（可走）；
    //   (0, 5.5) 一带和 (6.5, 6.5) 处有结构 ⇒ 应为 1。若 (0,0)=1 而别处=0，就是镜像。
    {
      auto occ_at = [&](double wx, double wy) -> int {
        const int x = static_cast<int>((wx - mox_) / mres_);
        const int y = static_cast<int>((wy - moy_) / mres_);
        if (x < 0 || y < 0 || x >= mw_ || y >= mh_) return -1;
        return mocc_[static_cast<size_t>(y) * mw_ + x];
      };
      //   ★ 用一对**镜像不对称**的点做判据（cad_v3_edit 实测真值 A=障碍/B=可走）：
      //     A(-18.42, 6.76) 应为 1；B(-18.42, -8.95) 应为 0。
      //     若日志反过来（A=0,B=1）⇒ **静态图上下镜像**。
      RCLCPP_INFO(logger_, "[tdt] 朝向自检: A(-18.42,6.76)=%d (应为1)  B(-18.42,-8.95)=%d (应为0)  (0,0)=%d",
                  occ_at(-18.42, 6.76), occ_at(-18.42, -8.95), occ_at(0.0, 0.0));
    }
  }

  int p_init_vel_mode_ = 0;
  double p_dump_grid_every_s_ = 30.0;   // a_occ 落盘周期（0=关闭）
  bool p_strict_collision_ = false;
  bool p_retry_osqp_path_ = false;
  bool last_plan_valid_ = false;
  // 两级规划的骨架（世界系）与缓存状态
  bool p_two_level_ = true;
  bool p_use_rog_map_ = false;
  // ---- 恢复 FSM ----
  bool p_recovery_enable_ = true;
  double p_stuck_time_ = 2.0, p_stuck_eps_ = 0.05;
  double p_escape_dist_ = 1.5, p_escape_speed_ = 0.3, p_escape_clear_ = 0.5;
  bool   p_escape_allow_enclosed_ = true;
  double p_escape_min_clear_ = 0.35;
  int p_max_escape_ = 3;
  double escape_target_dist_ = 1.5;   // 本次逃逸目标距离（被围住时缩短）
  enum RecState { REC_NORMAL, REC_ESCAPE, REC_FAILED } rec_state_ = REC_NORMAL;
  Eigen::Vector2f last_pos_{-1e9f, -1e9f};
  rclcpp::Time last_move_time_;
  rclcpp::Time escape_start_;
  rclcpp::Time last_escape_end_;
  int escape_attempts_ = 0;
  double escape_heading_ = 0.0;
  int fail_streak_ = 0;
  double last_plan_len_ = 0.0;
  double p_tail_speed_ = 0.0, p_tail_decel_ = 0.0;
  bool p_use_items_ = true, p_log_sdf_ = true;
  std::vector<signed char> a_occ_static_backup_;
  rclcpp::Publisher<rm_nav_interfaces::msg::StageStatus>::SharedPtr pub_st_l0_, pub_st_l1_;
  SfcSquare::Map sfc_map_static_backup_;
  double last_goal_x_ = -1e9, last_goal_y_ = -1e9;
  Eigen::Vector2f last_plan_dir_{0.f, 0.f};   // 上一版计划的初始方向（单位向量）
  Eigen::Vector2f prev_pos_{-1e9f, -1e9f};    // 约 1 s 前的位姿（算实际位移方向）
  rclcpp::Time prev_pos_time_;
  int t3_count_ = 0;                          // T3 连续计数（2 Hz 下 4 次 ≈ 2 s）
  rclcpp::Time first_fail_time_;
  std::shared_ptr<rog_map::ROGMapROS> rog_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_astar_, pub_simpl_, pub_smooth_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_corridor_;
  // ★ 2026-11-09：ROGMap 自己的可视化（/rog_map/layer_value）在我们这套里
  //   根本没有 publisher（ros2 topic info 显示 publisher count = 0），
  //   所以由规划器把**查询结果**发一份：100=OCCUPIED, 0=可走, -1=窗口外。
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_rog_grid_;
  // ★ 2026-11-10：把"局部"表达清楚
  //   · /tdt/rog_obstacles : ROGMap 判为障碍的格子（点云，RViz 里固定蓝色、小点）
  //   · /tdt/rog_window    : ROGMap 有效窗口的边框（蓝色矩形线）
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rog_obs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rog_win_;
  std::vector<int8_t> rog_grid_snapshot_;
  double p_local_window_ = 8.0;
  double p_backbone_age_ = 5.0;
  std::vector<Eigen::Vector2f> backbone_;
  std::vector<double> backbone_s_;
  bool backbone_valid_ = false;
  double backbone_goal_x_ = 1e9, backbone_goal_y_ = 1e9;
  rclcpp::Time backbone_stamp_;
  rclcpp::Time last_plan_time_;
  float last_plan_x_ = 0.f, last_plan_y_ = 0.f;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("tdt_planner")};
  rclcpp::Publisher<rm_plan_msgs::msg::TrajectoryPlan>::SharedPtr plan_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr mask_;
  double p_max_range_ = 2.5, p_shrink_ = 0.0, p_tl_ = 10.0, p_simplify_ = 0.1;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_backbone_;
  int p_order_ = 6, p_max_dx_ = 3, p_collision_iter_ = 6, p_dilate_cells_ = 1, p_mask_layers_ = 4;
  double p_max_speed_ = 3.0, p_max_acc_ = 1.0, p_mask_duration_ = 1.0;
  bool p_norm_time_ = true, p_mask_enable_ = true;
  bool p_use_map_file_ = true, map_loaded_ = false;
  std::string p_map_yaml_, p_map_frame_ = "map";
  int mw_ = 0, mh_ = 0; float mres_ = 0.05f, mox_ = 0.f, moy_ = 0.f;
  std::vector<int8_t> mocc_;          // 0=free, 1=obstacle（已含 pad 边框）
  std::vector<unsigned char> msfc_;   // 0=obstacle, 255=free
};

}  // namespace rm_tdt_planner

PLUGINLIB_EXPORT_CLASS(rm_tdt_planner::TdtPlanner, nav2_core::GlobalPlanner)
