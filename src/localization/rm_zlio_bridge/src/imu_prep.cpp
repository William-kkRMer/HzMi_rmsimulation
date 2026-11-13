// 仿真 IMU 预处理：① 启动 2 s 内用平均加速度估计重力方向，求出 R_level←sensor；
// ② 之后把 accel/gyro 旋到水平系（消除 1.5° 安装倾角带来的常数偏置 0.25 m/s²）；
// ③ 可选低通。时间戳与 frame_id 原样保留。
#include <algorithm>
#include <array>
#include <iterator>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace {
struct Vec3 { double x{0}, y{0}, z{0}; };
Vec3 cross(const Vec3& a, const Vec3& b) { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
double norm(const Vec3& v) { return std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z); }
Vec3 normalize(const Vec3& v) { const double n=norm(v); return n>1e-9 ? Vec3{v.x/n,v.y/n,v.z/n} : Vec3{0,0,1}; }
}  // namespace

class ImuPrep : public rclcpp::Node {
 public:
  ImuPrep() : Node("imu_prep") {
    in_ = declare_parameter("input_topic", std::string("/livox/imu"));
    out_ = declare_parameter("output_topic", std::string("/livox/imu_level"));
    alpha_ = declare_parameter("alpha", 1.0);        // 1.0 = 不开低通
    align_ = declare_parameter("align_gravity", true);
    accel_sigma_ = declare_parameter("accel_sigma", 0.02);   // m/s^2
    gyro_sigma_ = declare_parameter("gyro_sigma", 0.001);    // rad/s
    fill_gaps_ = declare_parameter("fill_gaps", true);
    nominal_period_ = declare_parameter("nominal_period", 0.01);  // 100 Hz
    calib_s_ = declare_parameter("calibration_seconds", 2.0);
    pub_ = create_publisher<sensor_msgs::msg::Imu>(out_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        in_, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::SharedPtr m) { onImu(m); });
    RCLCPP_INFO(get_logger(), "[imu_prep] %s -> %s align=%d alpha=%.2f",
                in_.c_str(), out_.c_str(), static_cast<int>(align_), alpha_);
  }

 private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr& m) {
    const Vec3 a{m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z};
    const Vec3 g{m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z};
    const double t = m->header.stamp.sec + m->header.stamp.nanosec * 1e-9;

    if (!ready_) {
      if (t0_ < 0) t0_ = t;
      if (t - t0_ < calib_s_) {  // 静止标定期：累计平均加速度
        acc_.x += a.x; acc_.y += a.y; acc_.z += a.z; ++n_;
        return;                   // 标定期间不发布，避免污染前端初始化
      }
      if (n_ > 0) {
        const Vec3 mean = normalize(acc_);
        // level 系：z 轴与比力方向对齐（静止时 = -gravity），x 取 mean × world_y
        if (align_) {
          const Vec3 zl = mean;
          const Vec3 ref{0.0, 1.0, 0.0};
          const Vec3 xl = normalize(cross(ref, zl));
          const Vec3 yl = cross(zl, xl);
          // R_level<-sensor：行 = level 基在 sensor 系的表达
          R_[0][0]=xl.x; R_[0][1]=xl.y; R_[0][2]=xl.z;
          R_[1][0]=yl.x; R_[1][1]=yl.y; R_[1][2]=yl.z;
          R_[2][0]=zl.x; R_[2][1]=zl.y; R_[2][2]=zl.z;
        }
        RCLCPP_INFO(get_logger(), "[imu_prep] 标定完成 samples=%d 平均比力=(%.3f %.3f %.3f) 倾角=%.2f°",
                    n_, mean.x, mean.y, mean.z,
                    std::acos(std::fabs(mean.z)) * 180.0 / M_PI);
      }
      ready_ = true;
    }

    Vec3 al = rotate(a), gl = rotate(g);
    if (alpha_ >= 0.999) {
      if (!init_) { lp_a_ = al; lp_g_ = gl; init_ = true; }
    } else {
      auto lp = [&](double prev, double v) { return prev + alpha_ * (v - prev); };
      if (!init_) { lp_a_ = al; lp_g_ = gl; init_ = true; }
      else {
        lp_a_.x = lp(lp_a_.x, al.x); lp_a_.y = lp(lp_a_.y, al.y); lp_a_.z = lp(lp_a_.z, al.z);
        lp_g_.x = lp(lp_g_.x, gl.x); lp_g_.y = lp(lp_g_.y, gl.y); lp_g_.z = lp(lp_g_.z, gl.z);
        al = lp_a_; gl = lp_g_;
      }
    }

    // ★★ 2026-11-08：**补 IMU 空隙**。实测录包：10% 的采样间隔 > 50 ms、最大 204 ms（RTF 抖动导致丢步）。
    //   前端把这种空隙当"采样中断"并重启预积分 ⇒ 滤波器被反复打断 ⇒ 位姿发散。
    //   这里在相邻两帧之间按标称周期线性插补（时间戳走消息自己的时基），让下游看到连续流。
    if (fill_gaps_ && last_out_t_ > 0 && t > last_out_t_) {
      const double gap = t - last_out_t_;
      if (gap > 1.5 * nominal_period_) {
        const int n = static_cast<int>(std::lround(gap / nominal_period_)) - 1;
        for (int k = 1; k <= std::min(n, 200); ++k) {
          const double frac = static_cast<double>(k) / (n + 1);
          auto lerp = [&](double a, double b) { return a + (b - a) * frac; };
          sensor_msgs::msg::Imu fill = *m;
          fill.linear_acceleration.x = lerp(last_a_.x, a.x);
          fill.linear_acceleration.y = lerp(last_a_.y, a.y);
          fill.linear_acceleration.z = lerp(last_a_.z, a.z);
          fill.angular_velocity.x = lerp(last_g_.x, g.x);
          fill.angular_velocity.y = lerp(last_g_.y, g.y);
          fill.angular_velocity.z = lerp(last_g_.z, g.z);
          const double ft = last_out_t_ + k * nominal_period_;
          fill.header.stamp.sec = static_cast<int32_t>(ft);
          fill.header.stamp.nanosec = static_cast<uint32_t>((ft - std::floor(ft)) * 1e9);
          if (!zero_cov_filled(fill)) {}
          pub_->publish(fill);
        }
      }
    }
    last_out_t_ = t; last_a_ = a; last_g_ = g;

    sensor_msgs::msg::Imu out = *m;
    // ★ 仿真 IMU 的协方差全是 0 ⇒ 任何用 R 求逆的滤波器都会奇异/发散（实测：静止时位姿跳到 ±60 m）。
    //   这里在协方差全零时补上物理上合理的对角值（可由参数覆盖）。
    const bool zero_cov = std::all_of(std::begin(out.linear_acceleration_covariance),
                                      std::end(out.linear_acceleration_covariance),
                                      [](double v) { return v == 0.0; });
    if (zero_cov) {
      const double av = accel_sigma_ * accel_sigma_, gv = gyro_sigma_ * gyro_sigma_;
      for (int i = 0; i < 9; ++i) {
        out.linear_acceleration_covariance[i] = (i % 4 == 0) ? av : 0.0;
        out.angular_velocity_covariance[i] = (i % 4 == 0) ? gv : 0.0;
      }
    }
    out.linear_acceleration.x = al.x; out.linear_acceleration.y = al.y; out.linear_acceleration.z = al.z;
    out.angular_velocity.x = gl.x;    out.angular_velocity.y = gl.y;    out.angular_velocity.z = gl.z;
    pub_->publish(out);
  }

  bool zero_cov_filled(sensor_msgs::msg::Imu& m) {
    const double av = accel_sigma_ * accel_sigma_, gv = gyro_sigma_ * gyro_sigma_;
    for (int i = 0; i < 9; ++i) {
      m.linear_acceleration_covariance[i] = (i % 4 == 0) ? av : 0.0;
      m.angular_velocity_covariance[i] = (i % 4 == 0) ? gv : 0.0;
    }
    return true;
  }

  Vec3 rotate(const Vec3& v) const {
    return {R_[0][0]*v.x + R_[0][1]*v.y + R_[0][2]*v.z,
            R_[1][0]*v.x + R_[1][1]*v.y + R_[1][2]*v.z,
            R_[2][0]*v.x + R_[2][1]*v.y + R_[2][2]*v.z};
  }

  std::string in_, out_;
  double alpha_{1.0}, calib_s_{2.0}, t0_{-1}, accel_sigma_{0.02}, gyro_sigma_{0.001};
  bool fill_gaps_{true};
  double nominal_period_{0.01}, last_out_t_{-1};
  Vec3 last_a_{}, last_g_{};
  std::uint32_t last_seq_{0};
  bool align_{true}, ready_{false}, init_{false};
  double R_[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
  Vec3 acc_{}, lp_a_{}, lp_g_{};
  int n_{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPrep>());
  rclcpp::shutdown();
  return 0;
}
