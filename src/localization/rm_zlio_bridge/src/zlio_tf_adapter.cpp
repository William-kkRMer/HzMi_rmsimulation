// zlio 位姿 → 标准 TF 适配器（补 zlio 那条悬空边）
//
// 背景：zlio 前端 world_frame 默认 `odom`、body_frame 默认 `imu`，
// 我们 URDF 里没有名为 `imu` 的 link ⇒ 它发的 `odom → imu` 是一条**悬空边**，
// 与 `base_link` 那棵树连不上 ⇒ nav2 报 "two or more unconnected trees"，
// 局部/全局代价图永远拿不到位姿，RViz 也显示不出地图。
//
// 试过直接改 zlio 的 body_frame，但后端会报
// "invalid ZLIO observation frame/time/point layout" ⇒ 世界点云断流。
// ⇒ 改用本适配器：**只读** /localization/odometry，用它的位姿自己发 `odom → base_link`。
//    zlio 保持原样，不碰它的内部校验。
//
// 帧约定：
//   map ──(static identity, 由 bringup 的 map_to_odom_static_tf 发)──→ odom
//   odom ──(本节点，来自 /localization/odometry 的位姿)──→ base_link
//   base_link ──(URDF)──→ imu_link / livox_frame ...
//   base_link ──(fake_vel_transform)──→ base_link_fake

#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"

namespace rm_zlio_bridge
{

class ZlioTfAdapter : public rclcpp::Node
{
public:
  ZlioTfAdapter() : Node("zlio_tf_adapter")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization/odometry");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    max_age_ = declare_parameter<double>("max_age", 0.5);
    publish_rate_ = declare_parameter<double>("publish_rate", 100.0);
    // 位姿 z 是否需要置零（我们只用平面导航，避免定位高度抖动带着车上下抖）
    flatten_z_ = declare_parameter<bool>("flatten_z", true);
    // 时间戳前瞻（秒）。见 tickImpl 里的详细说明：nav2 用 now 查询，
    // 必须让缓冲区里存在">= now"的数据，否则永远差一个发布周期。
    stamp_ahead_ = declare_parameter<double>("stamp_ahead", 0.04);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_ = *msg;
        last_time_ = now();
        has_msg_ = true;
        ++recv_count_;
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_)),
      [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
      "[zlio_tf] %s → %s 由 %s 驱动（%.0f Hz, 超时 %.1fs 停发, flatten_z=%s）",
      parent_frame_.c_str(), child_frame_.c_str(), odom_topic_.c_str(),
      publish_rate_, max_age_, flatten_z_ ? "on" : "off");
  }

private:
  void tick()
  {
    // ★★★ 2026-11-11（关键）：**整个回调包 try/catch**。
    //   实测现象：只发出 1 帧 TF 就再无输出（`已发 1 帧`），随后所有查询都报
    //   "extrapolation into the past/future" —— 因为 TF 缓冲里只剩那一个时间点。
    //   原因：rclcpp 的定时器回调里抛异常会把执行器卡死（节点还活着、TF 却停了）。
    //   TF 是整条导航链的地基，**绝不能因为一行日志或一次异常就静默停掉**。
    try {
      tickImpl();
    } catch (const std::exception & e) {
      // 异常不能吞掉不报，但也不能让它终止发布
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "[zlio_tf] tick 异常（已捕获，继续发）: %s", e.what());
    }

    // 心跳：无论是否发布都每 3 s 报一次，这样"停了"能立刻看出来
    heartbeat();
  }

  void tickImpl()
  {
    if (!has_msg_) {
      return;
    }
    const double age = (now() - last_time_).seconds();
    if (age > max_age_) {
      // 定位超时：**不发**旧 TF（发旧位姿比不发更危险——规划器会以为车还在原地）
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[zlio_tf] 位姿过期 %.2fs > %.2fs，暂停发布 TF", age, max_age_);
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    // ★★★ 2026-11-12（**实测根因**）：给时间戳加一点点**前瞻**。
    //
    //   现象（global_costmap 激活日志，恒定不变）：
    //       `Requested time 29.390000 but the latest data is at time 29.304985`
    //   其中 29.390 **正好等于 now**。nav2 的 `waitForTransform` 是拿
    //   `robot_pose.header.stamp = clock_->now()` 去查的（不是 TimePointZero），
    //   所以它要的永远是"此刻"，而最新一帧 TF 最年轻也只能是"上一次发布的时刻"
    //   ⇒ **结构性永远差 0~1 个发布周期**，与 transform_tolerance 无关
    //   （这也解释了为什么把容差 0.1 改到 0.5 毫无作用）。
    //
    //   tf2 允许"当前时刻之后"已有数据（往过去插值合法），因此把戳放到
    //   now + 0.04 s（≈ 2 个发布周期）后，任何 `now` 的查询都必然命中。
    //   0.04 s 的位姿误差在本车 3 m/s 下 = 12 cm，远小于控制/规划精度需求；
    //   需要时可调 small，设为 0 即恢复旧行为。
    tf.header.stamp = now() + rclcpp::Duration::from_seconds(stamp_ahead_);
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = child_frame_;
    tf.transform.translation.x = latest_.pose.pose.position.x;
    tf.transform.translation.y = latest_.pose.pose.position.y;
    tf.transform.translation.z = flatten_z_ ? 0.0 : latest_.pose.pose.position.z;
    tf.transform.rotation = latest_.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
    ++sent_count_;
  }

  // 心跳：把"发了多少帧 / 位姿多旧"定期打出来。**不做时间减法**（防止跨 clock_type 抛异常）。
  void heartbeat()
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[zlio_tf] 心跳：已发 %ld 帧（%.0f Hz 目标），has_msg=%d，count=%ld",
      static_cast<long>(sent_count_), publish_rate_, has_msg_ ? 1 : 0,
      static_cast<long>(recv_count_));
  }

  std::string odom_topic_, parent_frame_, child_frame_;
  double max_age_{0.5}, publish_rate_{50.0};
  double stamp_ahead_{0.04};   // 时间戳前瞻（秒），见 tickImpl 注释
  bool flatten_z_{true};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry latest_{};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  bool has_msg_{false};
  size_t sent_count_{0};
  size_t recv_count_{0};
};

}  // namespace rm_zlio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_zlio_bridge::ZlioTfAdapter>());
  rclcpp::shutdown();
  return 0;
}
