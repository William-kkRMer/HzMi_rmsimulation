// 对仿真 IMU 做低通（仿真里车体在轮子上以物理步频抖动：ay ±2.5 m/s²、gx ±0.61 rad/s）。
// 保留原始时间戳与 frame_id，只平滑测量值。
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuLpf : public rclcpp::Node {
 public:
  ImuLpf() : Node("imu_lpf") {
    in_topic_ = declare_parameter("input_topic", std::string("/livox/imu"));
    out_topic_ = declare_parameter("output_topic", std::string("/livox/imu_filtered"));
    alpha_ = declare_parameter("alpha", 0.15);
    pub_ = create_publisher<sensor_msgs::msg::Imu>(out_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        in_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr m) { onImu(m); });
    RCLCPP_INFO(get_logger(), "[imu_lpf] %s -> %s (alpha=%.2f)", in_topic_.c_str(), out_topic_.c_str(), alpha_);
  }

 private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr& m) {
    sensor_msgs::msg::Imu out = *m;
    if (!init_) {
      ax_ = m->linear_acceleration.x; ay_ = m->linear_acceleration.y; az_ = m->linear_acceleration.z;
      gx_ = m->angular_velocity.x;   gy_ = m->angular_velocity.y;   gz_ = m->angular_velocity.z;
      init_ = true;
    } else {
      auto lp = [&](double prev, double v) { return prev + alpha_ * (v - prev); };
      ax_ = lp(ax_, m->linear_acceleration.x); ay_ = lp(ay_, m->linear_acceleration.y); az_ = lp(az_, m->linear_acceleration.z);
      gx_ = lp(gx_, m->angular_velocity.x);   gy_ = lp(gy_, m->angular_velocity.y);   gz_ = lp(gz_, m->angular_velocity.z);
    }
    out.linear_acceleration.x = ax_; out.linear_acceleration.y = ay_; out.linear_acceleration.z = az_;
    out.angular_velocity.x = gx_;   out.angular_velocity.y = gy_;   out.angular_velocity.z = gz_;
    pub_->publish(out);
  }
  std::string in_topic_, out_topic_;
  double alpha_ = 0.15, ax_{0}, ay_{0}, az_{0}, gx_{0}, gy_{0}, gz_{0};
  bool init_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuLpf>());
  rclcpp::shutdown();
  return 0;
}
