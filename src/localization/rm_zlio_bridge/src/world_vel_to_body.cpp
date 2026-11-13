// 世界系速度 → 机体系速度（替代 BIT 的 communication 节点里那段 world→chassis 转换）
//
// 为什么需要：
//   minco_controller（北京理工大学追梦战队 navi_minco_bit）的 MPC 输出是
//   **global_frame 系速度**（源码 minco_mpc_controller.cpp:778 注释
//   "输出 global_frame_ 系速度，此处为 camera_init"），并且它会把这个速度
//   直接当作 Twist 返回给 nav2、同时发 /cmd_vel_mpc。
//   而底盘（mecanum）要的是**机体系**速度。
//   BIT 用 communication 节点按 delta_yaw 转成底盘协议方向；我们没有串口，
//   所以保留同样的语义，只把结果发到 /cmd_vel_chassis。
//
// 语义：
//   v_body = R(-yaw) · v_world
//     vx_body =  vx_w·cos(yaw) + vy_w·sin(yaw)
//     vy_body = -vx_w·sin(yaw) + vy_w·cos(yaw)
//     wz 不变
//
// 安全：
//   - TF 查不到 / 超时 ⇒ 发零速，绝不猜
//   - 指令超过 max_age ⇒ 发零速（避免 nav2 挂了车还在跑）
//   - 三个原因分别计数，日志里能看到到底卡在哪一边

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // tf2::getYaw(geometry_msgs::Quaternion)
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rm_zlio_bridge
{

class WorldVelToBody : public rclcpp::Node
{
public:
  WorldVelToBody() : Node("world_vel_to_body")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel_mpc");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_chassis");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization/odometry");
    max_age_ = declare_parameter<double>("max_age", 0.5);
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    // 是否用 TF 取 yaw。关了就用 odom 消息里的 orientation（少一次 TF 查询）
    use_tf_ = declare_parameter<bool>("use_tf", true);
    // ★ BIT 的 minco_controller 硬编码订阅 /aft_mapped_to_init（他的 Point-LIO 话题）。
    //   我们顺手把 /localization/odometry 转发到该话题，省掉一个额外节点。
    //   传空字符串则关闭转发。
    relay_from_ = declare_parameter<std::string>("relay_odom_from", "/localization/odometry");
    relay_to_ = declare_parameter<std::string>("relay_odom_to", "/aft_mapped_to_init");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = *msg;
        has_odom_ = true;
        if (relay_pub_) {
          relay_pub_->publish(*msg);
        }
      });

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = now();
        has_cmd_ = true;
      });

    if (!relay_from_.empty() && !relay_to_.empty()) {
      relay_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        relay_to_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
      RCLCPP_INFO(get_logger(), "[odom 转发] %s → %s（minco_controller 硬编码订阅后者）",
                  relay_from_.c_str(), relay_to_.c_str());
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WorldVelToBody::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "[world→body] %s（世界系）→ %s（机体系）｜世界系=%s 机体系=%s odom=%s max_age=%.2fs tf=%s",
      input_topic_.c_str(), output_topic_.c_str(), world_frame_.c_str(),
      base_frame_.c_str(), odom_topic_.c_str(), max_age_, use_tf_ ? "on" : "off");
  }

private:
  bool lookupYaw(double & yaw)
  {
    if (!use_tf_) {
      if (!has_odom_) {
        return false;
      }
      const auto & o = last_odom_.pose.pose.orientation;
      tf2::Quaternion q(o.x, o.y, o.z, o.w);
      yaw = tf2::getYaw(q);
      return true;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(
        world_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.05));
      yaw = tf2::getYaw(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void tick()
  {
    geometry_msgs::msg::Twist out;  // 默认全零

    if (!has_cmd_) {
      ++n_no_cmd_;
      pubZero(out, "还没收到 MPC 指令");
      return;
    }
    const double age = (now() - last_cmd_time_).seconds();
    if (age > max_age_) {
      ++n_stale_;
      pubZero(out, "MPC 指令过期");
      return;
    }

    double yaw = 0.0;
    if (!lookupYaw(yaw)) {
      ++n_tf_fail_;
      pubZero(out, "拿不到 yaw");
      return;
    }

    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    out.linear.x = last_cmd_.linear.x * c + last_cmd_.linear.y * s;
    out.linear.y = -last_cmd_.linear.x * s + last_cmd_.linear.y * c;
    out.angular.z = last_cmd_.angular.z;
    cmd_pub_->publish(out);

    // 每秒一次心跳，便于确认节点在工作而不是"悄悄发零"
    if (++n_ok_ % static_cast<uint64_t>(std::max(1.0, publish_rate_)) == 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[world→body] yaw=%.3f v_world=(%.2f,%.2f) → v_body=(%.2f,%.2f) wz=%.2f | 零速计数: 无指令 %lu / 过期 %lu / 无yaw %lu",
        yaw, last_cmd_.linear.x, last_cmd_.linear.y, out.linear.x, out.linear.y,
        out.angular.z, static_cast<unsigned long>(n_no_cmd_),
        static_cast<unsigned long>(n_stale_), static_cast<unsigned long>(n_tf_fail_));
    }
  }

  void pubZero(geometry_msgs::msg::Twist & out, const char * why)
  {
    out = geometry_msgs::msg::Twist{};
    cmd_pub_->publish(out);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "[world→body] 发零速：%s", why);
  }

  std::string input_topic_, output_topic_, world_frame_, base_frame_, odom_topic_;
  std::string relay_from_, relay_to_;
  double max_age_{0.5}, publish_rate_{50.0};
  bool use_tf_{true};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr relay_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist last_cmd_;
  nav_msgs::msg::Odometry last_odom_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false}, has_odom_{false};
  uint64_t n_ok_{0}, n_no_cmd_{0}, n_stale_{0}, n_tf_fail_{0};
};

}  // namespace rm_zlio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_zlio_bridge::WorldVelToBody>());
  rclcpp::shutdown();
  return 0;
}
