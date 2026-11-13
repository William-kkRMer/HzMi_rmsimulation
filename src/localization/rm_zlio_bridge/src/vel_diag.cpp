// 速度链路诊断：把「期望速度」和「实际执行速度」并排打在一行里。
//
// 为什么需要：接 BIT(minco_planner + minco_controller) 之后出现"车不动"，
// 但每个节点各自只知道自己那一段，无法一眼看出速度是在哪一段掉的。
//
// 本节点只订阅、不发布，每秒打印一行：
//   全局期望 = /opt_path 里 MINCO 轨迹自带的速度场（整条轨迹的均值/峰值 + 首点）
//   局部期望 = /cmd_vel_mpc（MPC 求解出的世界系速度，即它想执行的）
//   车身命令 = /cmd_vel_chassis（world_vel_to_body 转成机体系后真正发给底盘的）
//   底盘实测 = /localization/odometry 的线速度
//
// 判读：哪一段开始掉到 ~0，问题就在那一段之前。

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_nav_interfaces/msg/stage_status.hpp"
#include "ros_interfaces/msg/mpc_position_command.hpp"

namespace rm_zlio_bridge
{

class VelDiag : public rclcpp::Node
{
public:
  VelDiag() : Node("vel_diag")
  {
    opt_path_topic_ = declare_parameter<std::string>("opt_path_topic", "/opt_path");
    mpc_topic_ = declare_parameter<std::string>("mpc_topic", "/cmd_vel_mpc");
    // ★★★ 2026-11-12：**T-DT 栈的真实话题**。
    //   上面的默认值是 BIT/MINCO 时代的（`/opt_path` = MpcPositionCommand、
    //   `/cmd_vel_mpc` = MPC 世界系速度），T-DT 一个都不发 ⇒
    //   L1 永远 "无 /opt_path"、L2 永远 "无 /cmd_vel_mpc"，
    //   每 2 s 刷两条假 FAILED，极易误判"链路坏了"。
    //   T-DT 对应的是：
    //     全局轨迹  = /plan（nav_msgs/Path，规划器最终输出）
    //     控制器输出 = /cmd_vel（velocity_smoother 之后、进 relay 之前）
    // ★★★ 2026-11-12：**实际行驶轨迹**（此前 RViz 里没有任何"车真的走过哪"的显示，
    //   只能看到"要求走哪"）。把 odom 位置按 5 cm 间距累积成 Path 发布。
    trail_topic_ = declare_parameter<std::string>("trail_topic", "/nav/l3/dbg/trail");
    trail_max_pts_ = static_cast<int>(declare_parameter<int>("trail_max_pts", 4000));
    trail_pub_ = create_publisher<nav_msgs::msg::Path>(trail_topic_, rclcpp::QoS(rclcpp::KeepLast(1)));
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/plan");
    ctrl_topic_ = declare_parameter<std::string>("ctrl_topic", "/cmd_vel");
    chassis_topic_ = declare_parameter<std::string>("chassis_topic", "/cmd_vel_chassis");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization/odometry");
    period_ = declare_parameter<double>("period", 1.0);

    opt_sub_ = create_subscription<ros_interfaces::msg::MpcPositionCommand>(
      opt_path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg) {
        if (!msg || msg->cmds.empty()) return;
        double sum = 0.0, peak = 0.0;
        for (const auto & c : msg->cmds) {
          const double s = std::hypot(c.velocity.x, c.velocity.y);
          sum += s;
          peak = std::max(peak, s);
        }
        traj_n_ = msg->cmds.size();
        traj_avg_ = sum / static_cast<double>(msg->cmds.size());
        traj_peak_ = peak;
        traj_first_ = std::hypot(msg->cmds.front().velocity.x, msg->cmds.front().velocity.y);
        // 消息里没有 time_from_start：用「点数 × 名义步长」近似（0.05 s 是 BIT 的 t_step）
        traj_dur_ = static_cast<double>(msg->cmds.size()) * 0.05;
        traj_stamp_ = now();
        has_traj_ = true;
      });

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](nav_msgs::msg::Path::SharedPtr m) {
        if (m->poses.size() < 2) return;
        double len = 0.0;
        for (size_t i = 1; i < m->poses.size(); ++i) {
          const auto &a = m->poses[i - 1].pose.position;
          const auto &b = m->poses[i].pose.position;
          len += std::hypot(b.x - a.x, b.y - a.y);
        }
        // Path 无时间信息 ⇒ 没有真实速度；用 "剩余长度" 与点数作为健康度
        plan_len_ = len;
        plan_n_ = m->poses.size();
        plan_stamp_ = now();
        has_plan_ = true;
      });
    ctrl_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      ctrl_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](geometry_msgs::msg::Twist::SharedPtr m) {
        ctrl_v_ = std::hypot(m->linear.x, m->linear.y);
        has_ctrl_ = true;
      });
    mpc_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      mpc_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { mpc_ = *msg; has_mpc_ = true; });

    chassis_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      chassis_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { chassis_ = *msg; has_chassis_ = true; });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        actual_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        has_odom_ = true;
        // 轨迹累积：只在移动 > 5 cm 时加点，避免原地抖动把点刷爆
        const double ox = msg->pose.pose.position.x, oy = msg->pose.pose.position.y;
        if (trail_.empty() ||
            std::hypot(ox - trail_.back().first, oy - trail_.back().second) > 0.05) {
          trail_.emplace_back(ox, oy);
          trail_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
          if (static_cast<int>(trail_.size()) > trail_max_pts_) {
            trail_.erase(trail_.begin(), trail_.begin() + trail_.size() / 4);  // 掉头丢掉前 1/4
          }
        }
      });

    // ★ 分层状态（与 T-DT 时代同一套契约 /nav/l{0..3}/status）
    st_l1_pub_ = create_publisher<rm_nav_interfaces::msg::StageStatus>(
      "/nav/l1/status", rclcpp::QoS(rclcpp::KeepLast(1)));
    st_l2_pub_ = create_publisher<rm_nav_interfaces::msg::StageStatus>(
      "/nav/l2/status", rclcpp::QoS(rclcpp::KeepLast(1)));
    st_l3_pub_ = create_publisher<rm_nav_interfaces::msg::StageStatus>(
      "/nav/l3/status", rclcpp::QoS(rclcpp::KeepLast(1)));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.2, period_)),
      [this]() { print(); });

    RCLCPP_INFO(get_logger(),
      "[VEL] 速度链路诊断已启动：轨迹=%s MPC=%s 车身=%s 实测=%s 周期=%.1fs",
      opt_path_topic_.c_str(), mpc_topic_.c_str(), chassis_topic_.c_str(),
      odom_topic_.c_str(), period_);
  }

private:
  void print()
  {
    const double mpc_v = std::hypot(mpc_.linear.x, mpc_.linear.y);
    const double ch_v = std::hypot(chassis_.linear.x, chassis_.linear.y);
    const double traj_age = has_traj_ ? (now() - traj_stamp_).seconds() : -1.0;

    RCLCPP_INFO(get_logger(),
      "[VEL] 全局期望(轨迹 %zu 点, 均值%.2f 峰值%.2f 首点%.2f, 龄%.1fs) | "
      "局部期望(MPC %s) | MPC输出 %.2f m/s | 车身命令 %.2f m/s | 底盘实测 %.2f m/s",
      traj_n_, traj_avg_, traj_peak_, traj_first_, traj_age,
      has_mpc_ ? "有" : "无", mpc_v, ch_v, has_odom_ ? actual_ : -1.0);

    publishStatus(mpc_v, ch_v);
    publishTrail();
  }

  void publishTrail()
  {
    if (!trail_pub_ || trail_.empty()) return;
    nav_msgs::msg::Path p;
    p.header.stamp = now();
    p.header.frame_id = trail_frame_;
    p.poses.reserve(trail_.size());
    for (const auto & q : trail_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = p.header;
      ps.pose.position.x = q.first;
      ps.pose.position.y = q.second;
      ps.pose.orientation.w = 1.0;
      p.poses.push_back(ps);
    }
    trail_pub_->publish(p);
  }

  // 分层状态：state = 1(OK) / 2(DEGRADED) / 3(FAILED)
  void publishStatus(double mpc_v, double ch_v)
  {
    const double traj_age = has_traj_ ? (now() - traj_stamp_).seconds() : 999.0;

    // L1 全局：有没有轨迹、轨迹是否够快、是否过期
    {
      rm_nav_interfaces::msg::StageStatus m;
      m.stage = 1;
      const double plan_age = has_plan_ ? (now() - plan_stamp_).seconds() : 999.0;
      if (has_traj_) {
        // BIT/MINCO 档：能拿到带速度的轨迹，按原逻辑判
        m.state = traj_age > 2.0 ? 2 : (traj_peak_ < 0.2 ? 2 : 1);
        m.detail = traj_peak_ < 0.2 ? "轨迹峰值速度过低（时间权重/参数）" :
                   (traj_age > 2.0 ? "轨迹过期" : "正常");
        m.keys = {"pts", "dur_s", "v_avg", "v_peak", "age_s"};
        m.values = {static_cast<double>(traj_n_), traj_dur_, traj_avg_, traj_peak_, traj_age};
      } else {
        // T-DT 档：/plan 是几何 Path（**没有时间/速度**），只判"有没有、多长、新不新"
        m.state = !has_plan_ ? 3 : (plan_age > 2.0 ? 2 : (plan_len_ < 0.2 ? 2 : 1));
        m.detail = !has_plan_ ? "无 /plan" :
                   (plan_age > 2.0 ? "轨迹过期" :
                    (plan_len_ < 0.2 ? "轨迹过短" : "正常"));
        m.keys = {"pts", "len_m", "age_s"};
        m.values = {static_cast<double>(plan_n_), plan_len_, plan_age};
      }
      m.header.stamp = now();
      st_l1_pub_->publish(m);
    }
    // L2 局部：MPC 有没有把参考转成速度
    {
      rm_nav_interfaces::msg::StageStatus m;
      m.stage = 2;
      // T-DT 档没有 MPC；用 /cmd_vel（控制器经 velocity_smoother 的输出）代替
      const bool ok = (has_ctrl_ || has_mpc_) && (ctrl_v_ > 1e-3 || mpc_v > 1e-3);
      m.state = ok ? 1 : ((has_ctrl_ || has_mpc_) ? 2 : 3);
      m.detail = ok ? (has_ctrl_ ? "控制器有输出(/cmd_vel)" : "MPC 有输出") :
                 ((has_ctrl_ || has_mpc_) ? "输出≈0（静止/约束）" : "无 /cmd_vel");
      m.keys = {"ctrl_v", "plan_len"};
      m.values = {has_ctrl_ ? ctrl_v_ : mpc_v, plan_len_};
      m.header.stamp = now();
      st_l2_pub_->publish(m);
    }
    // L3 执行：车身命令 vs 实测
    {
      rm_nav_interfaces::msg::StageStatus m;
      m.stage = 3;
      const double ratio = ch_v > 1e-6 ? actual_ / ch_v : 0.0;
      m.state = !has_chassis_ ? 3 : (ch_v > 0.05 && ratio < 0.2 ? 2 : 1);
      m.detail = !has_chassis_ ? "无 /cmd_vel_chassis" :
                 (ch_v > 0.05 && ratio < 0.2 ? "命令大但实测小（执行机构/摩擦）" : "正常");
      m.keys = {"cmd_v", "actual_v", "ratio"};
      m.values = {ch_v, actual_, ratio};
      m.header.stamp = now();
      st_l3_pub_->publish(m);
    }
  }

  std::string opt_path_topic_, mpc_topic_, chassis_topic_, odom_topic_;
  std::string plan_topic_, ctrl_topic_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ctrl_sub_;
  double plan_len_{0.0}, ctrl_v_{0.0};
  size_t plan_n_{0};
  rclcpp::Time plan_stamp_{0, 0, RCL_ROS_TIME};
  bool has_plan_{false}, has_ctrl_{false};
  std::string trail_topic_;
  int trail_max_pts_{4000};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trail_pub_;
  std::vector<std::pair<double, double>> trail_;
  std::string trail_frame_{"map"};
  double period_{1.0};
  rclcpp::Subscription<ros_interfaces::msg::MpcPositionCommand>::SharedPtr opt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mpc_sub_, chassis_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rm_nav_interfaces::msg::StageStatus>::SharedPtr st_l1_pub_, st_l2_pub_, st_l3_pub_;

  geometry_msgs::msg::Twist mpc_{}, chassis_{};
  double actual_{0.0};
  double traj_avg_{0.0}, traj_peak_{0.0}, traj_first_{0.0}, traj_dur_{0.0};
  size_t traj_n_{0};
  rclcpp::Time traj_stamp_{0, 0, RCL_ROS_TIME};
  bool has_traj_{false}, has_mpc_{false}, has_chassis_{false}, has_odom_{false};
};

}  // namespace rm_zlio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_zlio_bridge::VelDiag>());
  rclcpp::shutdown();
  return 0;
}
