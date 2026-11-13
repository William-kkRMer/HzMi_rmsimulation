// nav2_core::Controller 插件：走廊约束 MPC
//  - 参考：沿全局路径按弧长 + v_ref 推进（nav2 的 Path 没有时间参数化）
//  - 走廊：以参考点为中心、±corridor_half_width 的方形（后续可换成 SfcSquare/规划器走廊）
//  - 求解：mpc_core（OSQP，软约束）
//  - 输出：x/y 速度（车体系）+ 由航向误差算的角速度
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rm_plan_msgs/msg/trajectory_plan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/utils.h"

#include "mpc_core/mpc_core.hpp"

namespace rm_mpc_controller {

namespace {
double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

class MpcController : public nav2_core::Controller {
 public:
  MpcController() = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
    parent_ = parent;
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    auto node = parent_.lock();
    logger_ = node->get_logger();
    auto declare = [&](const std::string& k, auto def) {
      return node->declare_parameter(name + "." + k, def);
    };
    p_.dt = declare("dt", 0.05);
    p_.N = declare("horizon", 20);
    p_.vmax = declare("vmax", 2.0);
    p_.amax = declare("amax", 4.0);
    p_.w_pos = declare("w_pos", 80.0);
    p_.w_vel = declare("w_vel", 20.0);
    p_.w_u = declare("w_u", 4.0);
    p_.w_slack = declare("w_slack", 2000.0);
    v_ref_ = declare("v_ref", 1.0);
    half_w_ = declare("corridor_half_width", 0.35);
    k_theta_ = declare("k_theta", 1.5);
    wmax_ = declare("wmax", 6.0);

    use_plan_ = node->declare_parameter(name + ".use_plan", true);
    plan_max_age_ = node->declare_parameter(name + ".plan_max_age", 2.0);
    plan_sub_ = node->create_subscription<rm_plan_msgs::msg::TrajectoryPlan>(
        "/trajectory_plan", rclcpp::QoS(1),
        [this](rm_plan_msgs::msg::TrajectoryPlan::SharedPtr m) { plan_ = m; });
    dbg_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/mpc_debug", 10);
    RCLCPP_INFO(logger_,
                "[rm_mpc] configured: N=%d dt=%.3f vmax=%.2f amax=%.2f v_ref=%.2f half_w=%.2f",
                p_.N, p_.dt, p_.vmax, p_.amax, v_ref_, half_w_);
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  void setPlan(const nav_msgs::msg::Path& path) override {
    path_ = path;
    s_.clear();
    s_.reserve(path_.poses.size());
    double acc = 0.0;
    s_.push_back(0.0);
    for (size_t i = 1; i < path_.poses.size(); ++i) {
      const auto& a = path_.poses[i - 1].pose.position;
      const auto& b = path_.poses[i].pose.position;
      acc += std::hypot(b.x - a.x, b.y - a.y);
      s_.push_back(acc);
    }
    near_idx_ = 0;
    v_est_x_ = v_est_y_ = 0.0;
    RCLCPP_INFO(logger_, "[rm_mpc] setPlan: %zu 点, 长度 %.2f m, 首点=(%.2f, %.2f)",
                path_.poses.size(), acc,
                path_.poses.empty() ? 0.0 : path_.poses.front().pose.position.x,
                path_.poses.empty() ? 0.0 : path_.poses.front().pose.position.y);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* /*goal_checker*/) override {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    cmd.header.frame_id = costmap_ros_->getBaseFrameID();
    const bool has_path = (path_.poses.size() >= 2 && s_.size() == path_.poses.size());
    const bool has_plan = (plan_ && plan_->t.size() >= 2);
    if (!has_path && !has_plan) return cmd;
    // 没有 path 时用 plan 的第一个点兜底（下面的路径分支会跳过）
    if (!has_path && has_plan) {
      // 直接走 plan 分支：把 ref/box 用 plan 填满
    }

    // ---- 投影到路径：找最近点（窗口搜索，避免整条扫）----
    const double px = pose.pose.position.x, py = pose.pose.position.y;
    std::vector<rm_mpc::RefStage> ref(p_.N);
    std::vector<rm_mpc::Box> box(p_.N);
    size_t best_i = 0;
    double s_now = 0.0;
    if (has_path) {
    size_t i0 = near_idx_ > 20 ? near_idx_ - 20 : 0;
    size_t i1 = std::min(path_.poses.size() - 1, near_idx_ + 60);
    double best = 1e18;
    size_t best_i = near_idx_;
    for (size_t i = i0; i <= i1; ++i) {
      const auto& q = path_.poses[i].pose.position;
      const double d = (q.x - px) * (q.x - px) + (q.y - py) * (q.y - py);
      if (d < best) { best = d; best_i = i; }
    }
    near_idx_ = best_i;
    s_now = s_[best_i];

    // ---- 参考 + 走廊（路径来源）----
    for (int k = 0; k < p_.N; ++k) {
      const double s_k = std::min(s_now + v_ref_ * (k + 1) * p_.dt, s_.back());
      // 找 s_k 所在段
      size_t j = std::upper_bound(s_.begin(), s_.end(), s_k) - s_.begin();
      j = std::min(j, s_.size() - 1);
      const size_t j0 = j > 0 ? j - 1 : 0;
      const double seg = std::max(1e-9, s_[j] - s_[j0]);
      const double a = clampd((s_k - s_[j0]) / seg, 0.0, 1.0);
      const auto& p0 = path_.poses[j0].pose.position;
      const auto& p1 = path_.poses[j].pose.position;
      ref[k].x = p0.x + a * (p1.x - p0.x);
      ref[k].y = p0.y + a * (p1.y - p0.y);
      const double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
      ref[k].vx = v_ref_ * std::cos(yaw);
      ref[k].vy = v_ref_ * std::sin(yaw);
      box[k] = {ref[k].x - half_w_, ref[k].y - half_w_, ref[k].x + half_w_, ref[k].y + half_w_};
    }
    }  // end has_path

    // ---- 若收到规划器的 TrajectoryPlan，则优先消费它（时间参考 + SFC 走廊）----
    bool used_plan = false;
    if (use_plan_ && plan_ && plan_->t.size() >= 2 && plan_->x.size() == plan_->t.size() &&
        plan_->y.size() == plan_->t.size()) {
      const double age = (parent_.lock()->now() - rclcpp::Time(plan_->header.stamp)).seconds();
      if (age >= -0.5 && age < plan_max_age_) {
        // ★ 参考时间按**机器人投影到 plan 的位置**取（空间跟踪），
        //   而不是按 plan 的绝对时间：否则机器人一落后，参考就跑远 ⇒ QP 饱和。
        size_t kbest = 0; double dbest = 1e18;
        for (size_t i = 0; i < plan_->x.size(); ++i) {
          const double d = std::hypot(plan_->x[i] - px, plan_->y[i] - py);
          if (d < dbest) { dbest = d; kbest = i; }
        }
        const double t_now = plan_->t[kbest];
        const auto& TP = *plan_;
        const size_t nT = TP.t.size();
        auto interp = [&](double t, double* ox, double* oy, double* ovx, double* ovy) {
          const double tt = clampd(t, TP.t.front(), TP.t.back());
          size_t i = 1;
          while (i + 1 < nT && TP.t[i] < tt) ++i;
          const double t0 = TP.t[i - 1], t1 = TP.t[i];
          const double a = clampd((tt - t0) / std::max(1e-9, t1 - t0), 0.0, 1.0);
          *ox = TP.x[i - 1] + a * (TP.x[i] - TP.x[i - 1]);
          *oy = TP.y[i - 1] + a * (TP.y[i] - TP.y[i - 1]);
          *ovx = TP.vx[i - 1] + a * (TP.vx[i] - TP.vx[i - 1]);
          *ovy = TP.vy[i - 1] + a * (TP.vy[i] - TP.vy[i - 1]);
        };
        for (int k = 0; k < p_.N; ++k) {
          interp(std::max(0.0, t_now) + (k + 1) * p_.dt, &ref[k].x, &ref[k].y, &ref[k].vx, &ref[k].vy);
          // 走廊：取包住该参考点的 box；没有则取中心最近的
          rm_mpc::Box bb{ref[k].x - half_w_, ref[k].y - half_w_, ref[k].x + half_w_, ref[k].y + half_w_};
          double bestd = 1e18;
          const size_t nB = TP.corridor.size() / 4;
          for (size_t j = 0; j < nB; ++j) {
            const double x0 = TP.corridor[4*j], y0 = TP.corridor[4*j+1];
            const double x1 = TP.corridor[4*j+2], y1 = TP.corridor[4*j+3];
            if (x1 - x0 < 0.10 || y1 - y0 < 0.10) continue;          // 退化 box 跳过
            const double cx = 0.5 * (x0 + x1), cy = 0.5 * (y0 + y1);
            const double d = std::hypot(cx - ref[k].x, cy - ref[k].y);
            if (d < bestd) { bestd = d; bb = {x0, y0, x1, y1}; }
          }
          box[k] = bb;
        }
        used_plan = true;
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
            "[rm_mpc] 消费 TrajectoryPlan：%zu 点, 走廊 %zu, age=%.2fs, source=%u",
            nT, TP.corridor.size() / 4, t_now, (unsigned)TP.traj_source);
      }
    }
    if (!has_path && !used_plan) return cmd;
    if (!has_path) { for (int k = 0; k < p_.N; ++k) box[k] = box[k]; }

    // ---- 求解（**速度空间**，与 prox_mpc 的公式化一致：控制量就是速度）----
    const double yaw = yawFromQuat(pose.pose.orientation);
    rm_mpc::VelParams vp;
    vp.dt = p_.dt; vp.N = p_.N; vp.vmax = p_.vmax; vp.amax = p_.amax;
    vp.w_pos = p_.w_pos; vp.w_vel = p_.w_vel; vp.w_slack = p_.w_slack;
    vp.w_du = 2.0;
    const Eigen::Vector2d p0(px, py);
    // ★ 真实速度反馈：本栈 `/odom` 的 twist 恒为 0，改用**位姿差分**测速
    {
      const rclcpp::Time now = parent_.lock()->now();
      if (last_meas_valid_) {
        const double dtm = (now - last_meas_time_).seconds();
        if (dtm > 0.01 && dtm < 0.5) {
          const double mvx = (px - last_meas_x_) / dtm;
          const double mvy = (py - last_meas_y_) / dtm;
          v_est_x_ = 0.7 * mvx + 0.3 * v_est_x_;
          v_est_y_ = 0.7 * mvy + 0.3 * v_est_y_;
        }
      }
      last_meas_x_ = px; last_meas_y_ = py; last_meas_time_ = now; last_meas_valid_ = true;
    }
    {   // 输出饱和时也别让 Δv 约束把状态钉死：把估计限幅到 vmax
      const double n = std::hypot(v_est_x_, v_est_y_);
      if (n > p_.vmax) { v_est_x_ *= p_.vmax / n; v_est_y_ *= p_.vmax / n; }
    }
    const Eigen::Vector2d v_prev(v_est_x_, v_est_y_);   // 测得的真实速度（世界系）
    auto sol = rm_mpc::solveVelocity(vp, p0, v_prev, ref, box);
    if (!sol.ok) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "[rm_mpc] QP 失败: %s", sol.status);
      return cmd;
    }
    // 输出：v_0 世界系 → 车体系
    const double vx0_b = std::cos(yaw) * sol.vx0 + std::sin(yaw) * sol.vy0;
    const double vy0_b = -std::sin(yaw) * sol.vx0 + std::cos(yaw) * sol.vy0;
    cmd.twist.linear.x = clampd(vx0_b, -p_.vmax, p_.vmax);
    cmd.twist.linear.y = clampd(vy0_b, -p_.vmax, p_.vmax);
    v_est_x_ = sol.vx0; v_est_y_ = sol.vy0;   // 供下一周期做 Δv 约束

    // ★ 航向误差应对**路径切线**（前视点），而不是"最近点方向"——
    //   最近点常常就在车正下方 ⇒ atan2(0,0) 方向乱 ⇒ 车会画龙（实测直线效率只有 74%）。
    // ★ 前视改成**按弧长**（0.8 m）取，而不是固定 6 个点：
    //   路径点间距随规划器变化，固定点数会让前视距离时大时小 ⇒ 航向抖 ⇒ 车画龙。
    const double look = s_now + 0.8;
    size_t li = best_i;
    while (li + 1 < path_.poses.size() && s_[li] < look) ++li;
    const size_t lj = std::min(path_.poses.size() - 1, li + 1);
    const auto& qa = path_.poses[li].pose.position;
    const auto& qb = path_.poses[lj].pose.position;
    double path_yaw;
    if (std::hypot(qb.x - qa.x, qb.y - qa.y) > 1e-3) {
      path_yaw = std::atan2(qb.y - qa.y, qb.x - qa.x);
    } else {
      path_yaw = std::atan2(qa.y - py, qa.x - px);
    }
    const double e_theta = wrap(path_yaw - yaw);
    cmd.twist.angular.z = clampd(k_theta_ * e_theta, -wmax_, wmax_);
    publishDebug(v_prev.x(), v_prev.y(), sol.vx0, sol.vy0, ref, s_now, near_idx_,
                 sol.ms, sol.iters, sol.ok, cmd);
    return cmd;
  }

  void setSpeedLimit(const double& speed_limit, const bool& percentage) override {
    if (percentage) {
      v_ref_ = std::max(0.05, 2.0 * speed_limit / 100.0);
    } else {
      v_ref_ = std::max(0.05, speed_limit);
    }
  }

 private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("rm_mpc")};
  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  rm_mpc::Params p_;
  double v_ref_ = 1.0, half_w_ = 0.35, k_theta_ = 1.5, wmax_ = 6.0;
  nav_msgs::msg::Path path_;
  std::vector<double> s_;
  size_t near_idx_ = 0;
  double v_est_x_ = 0.0, v_est_y_ = 0.0;   // 内部速度估计（世界系）
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dbg_pub_;
  rclcpp::Subscription<rm_plan_msgs::msg::TrajectoryPlan>::SharedPtr plan_sub_;
  rm_plan_msgs::msg::TrajectoryPlan::SharedPtr plan_;
  bool use_plan_ = true;
  double last_meas_x_ = 0.0, last_meas_y_ = 0.0;
  rclcpp::Time last_meas_time_{0, 0, RCL_ROS_TIME};
  bool last_meas_valid_ = false;
  double plan_max_age_ = 2.0;

  void publishDebug(double vx_w, double vy_w, double v0x, double v0y,
                    const std::vector<rm_mpc::RefStage>& ref, double s_now,
                    size_t idx, double ms, int iters, bool ok,
                    const geometry_msgs::msg::TwistStamped& cmd) {
    if (!dbg_pub_) return;
    std_msgs::msg::Float64MultiArray m;
    m.data = {vx_w, vy_w, v0x, v0y,
              ref.empty() ? 0.0 : ref[0].x, ref.empty() ? 0.0 : ref[0].y,
              ref.empty() ? 0.0 : ref[0].vx, ref.empty() ? 0.0 : ref[0].vy,
              cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z,
              static_cast<double>(idx), s_now, ms, static_cast<double>(iters),
              ok ? 1.0 : 0.0};
    dbg_pub_->publish(m);
  }
};

}  // namespace rm_mpc_controller

PLUGINLIB_EXPORT_CLASS(rm_mpc_controller::MpcController, nav2_core::Controller)
