// Mask 生产者：patchwork 非地面点 → 2D OccupancyGrid（带 duration 由 planner 侧处理）
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Geometry>

class MaskBuilder : public rclcpp::Node {
 public:
  MaskBuilder() : Node("plan_mask_builder") {
    res_ = declare_parameter("resolution", 0.05);
    w_ = declare_parameter("width", 560);
    h_ = declare_parameter("height", 313);
    ox_ = declare_parameter("origin_x", -19.9);
    oy_ = declare_parameter("origin_y", -8.89);
    zmin_ = declare_parameter("z_min", 0.10);
    zmax_ = declare_parameter("z_max", 1.20);
    frame_ = declare_parameter("frame_id", std::string("map"));
    const auto in_topic = declare_parameter("input_topic", std::string("/segmentation/obstacle"));
    // ★ 可直接给地图 yaml：自动取 resolution/origin/image 尺寸，省掉四个手填参数
    const auto map_yaml = declare_parameter("map_yaml", std::string(""));
    if (!map_yaml.empty()) loadMapYaml(map_yaml);

    grid_.assign(w_ * h_, 0);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    use_tf_ = declare_parameter("use_tf", true);
    target_frame_ = declare_parameter("target_frame", frame_);
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/plan_mask", rclcpp::QoS(1));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        in_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { onCloud(m); });
    RCLCPP_INFO(get_logger(), "[plan_mask] %dx%d @ %.2f m, z∈[%.2f,%.2f], in=%s",
                w_, h_, res_, zmin_, zmax_, in_topic.c_str());
  }

 private:
  void loadMapYaml(const std::string& path) {
    std::ifstream f(path);
    if (!f) { RCLCPP_WARN(get_logger(), "[plan_mask] 读不到 %s，用参数", path.c_str()); return; }
    std::string line, image;
    while (std::getline(f, line)) {
      std::istringstream is(line);
      std::string k;
      if (!(is >> k)) continue;
      if (k.rfind("resolution:", 0) == 0) is >> res_;
      else if (k.rfind("origin:", 0) == 0) {
        // 兼容两种 yaml 风格：
        //   origin: [-19.9, -8.89, 0.0]        （内联，手写图）
        //   origin:\n- -19.9\n- -8.89\n- 0.0   （块格式，yaml.safe_dump 产物）
        if (line.find('[') != std::string::npos) {
          std::string rest = line.substr(line.find('['));
          for (char& c : rest) if (c == '[' || c == ']' || c == ',') c = ' ';
          std::istringstream os(rest); os >> ox_ >> oy_;
        } else {
          for (int i = 0; i < 3; ++i) {
            std::string l2;
            if (!std::getline(f, l2)) break;
            std::istringstream os(l2);
            double v = 0.0;
            if (os >> v) { if (i == 0) ox_ = v; else if (i == 1) oy_ = v; }
          }
        }
      } else if (k.rfind("image:", 0) == 0) { is >> image; }
    }
    // 从 PGM 头取宽高（本仓库的 2D 图都是 P5）
    const std::string base = path.substr(0, path.find_last_of('/') + 1);
    std::ifstream img(base + image, std::ios::binary);
    if (img) {
      std::string magic; int w = 0, h = 0, mx = 0;
      img >> magic >> w >> h >> mx;
      if (magic == "P5" && w > 0 && h > 0) { w_ = w; h_ = h; }
    }
    RCLCPP_INFO(get_logger(), "[plan_mask] 从 yaml 读取: %dx%d @ %.3f m, origin (%.2f, %.2f)",
                w_, h_, res_, ox_, oy_);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    std::fill(grid_.begin(), grid_.end(), 0);
    // ★ 点云必须先转到 map 帧（patchwork 输出在 livox_frame）
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    bool have_tf = true;
    if (use_tf_ && !target_frame_.empty() && msg->header.frame_id != target_frame_) {
      try {
        geometry_msgs::msg::TransformStamped tf =
            tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id,
                                        rclcpp::Time(msg->header.stamp), rclcpp::Duration::from_seconds(0.2));
        Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                             tf.transform.rotation.y, tf.transform.rotation.z);
        T = Eigen::Isometry3d::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y,
                                          tf.transform.translation.z);
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "[plan_mask] TF %s→%s 失败: %s", msg->header.frame_id.c_str(), target_frame_.c_str(), e.what());
        return;
      }
    }
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    int n = 0;
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      Eigen::Vector3d p(*ix, *iy, *iz);
      if (have_tf) p = T * p;
      if (!p.allFinite() || p.z() < zmin_ || p.z() > zmax_) continue;
      const int gx = static_cast<int>((p.x() - ox_) / res_);
      const int gy = static_cast<int>((p.y() - oy_) / res_);
      if (gx < 0 || gy < 0 || gx >= w_ || gy >= h_) continue;
      grid_[gy * w_ + gx] = 100;
      ++n;
    }
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = target_frame_.empty() ? frame_ : target_frame_;
    out.info.resolution = res_;
    out.info.width = w_;
    out.info.height = h_;
    out.info.origin.position.x = ox_;
    out.info.origin.position.y = oy_;
    out.info.origin.orientation.w = 1.0;
    out.data = grid_;
    pub_->publish(out);
    if (n) RCLCPP_DEBUG(get_logger(), "[plan_mask] %d 点", n);
  }

  double res_, ox_, oy_, zmin_, zmax_;
  int w_, h_;
  std::string frame_, target_frame_;
  bool use_tf_ = true;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<int8_t> grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaskBuilder>());
  rclcpp::shutdown();
  return 0;
}
