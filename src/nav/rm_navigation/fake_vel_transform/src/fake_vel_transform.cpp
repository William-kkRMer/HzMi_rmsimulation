#include "fake_vel_transform/fake_vel_transform.hpp"

#include <tf2/utils.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace fake_vel_transform
{
const std::string CMD_VEL_TOPIC = "/cmd_vel";
const std::string AFTER_TF_CMD_VEL = "/cmd_vel_chassis";
const std::string TRAJECTORY_TOPIC = "/local_plan";
// ★★★ 2026-11-12（**实测改**）：20 → 50 Hz。
//   实测（tf2_monitor map base_link_fake）：
//     Requested 21.910 = now   /   latest 21.805 = now − 0.105
//   即"最新一帧比查询时刻旧约 0.105 s"，而 nav2 的激活检查查的是 **now**，
//   tf2 又不允许外插到未来 ⇒ **每次都判失败、且永不追上** ⇒ costmap 永不激活。
//   0.105 s 的来源：本节点 20 Hz（≤50 ms）+ 适配器转发的 odom 20 Hz（≤50 ms）。
//   本节点提到 50 Hz（≤20 ms）后，总滞后降到 ~0.07 s，给容差留出余量。
//   回退：改回 20。
const int TF_PUBLISH_FREQUENCY = 50;  // base_link to base_link_fake. Frequency in Hz.
// 时间戳前瞻（秒）：见 publishTransform() 里的实测说明。0.0 = 旧行为。
static constexpr double TF_STAMP_AHEAD_S = 0.04;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  // Declare and get the spin speed parameter
  this->declare_parameter<float>("spin_speed", 0.0);   // ★ 默认改 0（直通）：-6.0 会让任何转向指令被替换成 6 rad/s 的小陀螺
  this->get_parameter("spin_speed", spin_speed_);

  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Create Publisher and Subscriber
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_TOPIC, 1, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    AFTER_TF_CMD_VEL, rclcpp::QoS(rclcpp::KeepLast(1)));
  local_pose_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    TRAJECTORY_TOPIC, 1,
    std::bind(&FakeVelTransform::localPoseCallback, this, std::placeholders::_1));

  // ★ 心跳：15 s 一次，打印 TF 帧数 + /cmd_vel 收到与转发条数。
  //   用途：判断①`base_link→base_link_fake` 这条边是否还在发（TF 断了会让
  //   整个 nav2 报 "two or more unconnected trees"）；②底盘指令是否真的转发出去。
  hb_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(15000),
    [this]() {
      RCLCPP_INFO(this->get_logger(),
        "[fake_vel] 心跳：TF 已发 %zu 帧（%d Hz）| /cmd_vel 收 %zu 条 → 转发 %zu 条 | spin_speed=%.1f",
        tf_count_, TF_PUBLISH_FREQUENCY, cmd_count_, relay_count_,
        static_cast<double>(spin_speed_));
    });

  // Create a timer to publish the transform regularly
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / TF_PUBLISH_FREQUENCY),
    std::bind(&FakeVelTransform::publishTransform, this));
}

// Get the local pose from planner
void FakeVelTransform::localPoseCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  // ★ 2026-09-24：spin_speed_ == 0 的直通模式下不做云台解耦，current_angle_ 恒为 0
  //   （base_link→base_link_fake 的单位旋转），nav 看到的就是底盘真实朝向。
  if (spin_speed_ == 0.0) {
    current_angle_ = 0.0;
    return;
  }

  if (!msg || msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty or invalid PoseArray message");
    return;
  }

  // Choose the pose based on the size of the poses array
  size_t index = std::min(msg->poses.size() / 4, msg->poses.size() - 1);
  const geometry_msgs::msg::Pose & selected_pose = msg->poses[index].pose;

  // Update current angle based on the difference between teb_angle and base_link_angle_
  double teb_angle = tf2::getYaw(selected_pose.orientation);
  current_angle_ = teb_angle - base_link_angle_;
}

// Transform the velocity from base_link to base_link_fake
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  ++cmd_count_;
  // ★ 2026-09-24 修复：原来无论 spin_speed 取什么值，都把 nav 的角速度**替换**成 spin_speed_：
  //     aft_tf_vel.angular.z = (msg->angular.z != 0) ? spin_speed_ : 0;
  //   于是当 spin_speed:=0.0（用来关掉小陀螺）时，**任何转弯指令都被写成 0** ⇒ 底盘永不转
  //   ⇒ nav 永远对不准朝向 ⇒ progress_checker 报 'Failed to make progress'、车一步不动。
  //   实测证据：/cmd_vel angular.z=0.75 → /cmd_vel_chassis angular.z=0.0，/odom_ground_truth 全 0。
  //   修法：spin_speed_ == 0 时进入「无云台解耦」直通模式（此时 base_link_fake ≡ base_link）：
  //         角速度与线速度原样下发，且不做 TF 查询（顺带避免了 TF 缺失时整条指令丢失）。
  //         spin_speed_ != 0 时保持原有小陀螺/云台解耦语义完全不变。
  if (spin_speed_ == 0.0) {
    current_angle_ = 0.0;
    ++relay_count_;
    cmd_vel_chassis_pub_->publish(*msg);
    return;
  }

  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf2_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);

    double angle_diff = -current_angle_;

    geometry_msgs::msg::Twist aft_tf_vel;
    aft_tf_vel.angular.z = (msg->angular.z != 0) ? spin_speed_ : 0;
    aft_tf_vel.linear.x = msg->linear.x * cos(angle_diff) + msg->linear.y * sin(angle_diff);
    aft_tf_vel.linear.y = -msg->linear.x * sin(angle_diff) + msg->linear.y * cos(angle_diff);

    ++relay_count_;
    cmd_vel_chassis_pub_->publish(aft_tf_vel);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
  }
}

// Publish transform from base_link to base_link_fake
void FakeVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  // ★★★ 2026-11-12（**实测根因，别再改错地方**）：
  //   nav2 的 costmap 激活判定拿 `robot_pose.header.stamp = clock_->now()` 去查 TF，
  //   即**要求缓冲区里存在 >= now 的数据**；而任何发布者最年轻的一帧只能是
  //   "上一次发布的时刻" ⇒ 戳写成 now() 时，这个查询**结构性永远差一个周期**。
  //
  //   本轮本地实测（tools/scripts/nav/tf_lookup_probe.py，查询 odom→base_link_fake）：
  //     A 只有适配器加前瞻、本条不加        → 成功   0/100  = 0%
  //     B 两条边都加 40 ms、本条 20 Hz      → 成功  99/100
  //     C 两条边都加 40 ms、本条 50 Hz      → 成功 100/100 = 100%
  //   ⇒ **链上最新可用时刻 = 两条动态边里较旧的那条**，所以两条都必须加前瞻；
  //     只改适配器那一条**必然无效**（这正是我前几轮怎么改都没用的原因）。
  //
  //   前瞻 40 ms 对位姿的代价：3 m/s 下 12 cm，远小于规划/控制精度需求。
  //   回退：把 TF_STAMP_AHEAD_S 改成 0.0 即恢复旧行为。
  const double ahead = TF_STAMP_AHEAD_S;
  t.header.stamp = get_clock()->now() + rclcpp::Duration::from_seconds(ahead);
  t.header.frame_id = "base_link";
  t.child_frame_id = "base_link_fake";
  ++tf_count_;
  tf2::Quaternion q;
  q.setRPY(0, 0, current_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)