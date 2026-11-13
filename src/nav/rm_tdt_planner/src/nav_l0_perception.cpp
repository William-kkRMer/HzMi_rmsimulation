// L0 感知层（独立节点）
// 职责：点云 → ROGMap → 2D 占用栅格。**只做这一件事**。
// 输出契约：/nav/l0/grid (nav_msgs/OccupancyGrid)  调试：/nav/l0/dbg/*  状态：/nav/l0/status
// 这样 ROGMap 可以脱离 nav2 单独跑、单独调（此前它藏在 planner_server 里，无法单测）。
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rog_map/rog_map_core/config.hpp"
#include "rog_map/map_registry.hpp"
#include "rog_map/map_query_interface.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"
#include "rm_nav_interfaces/msg/stage_status.hpp"

using namespace std::chrono_literals;

class NavL0 : public rclcpp::Node {
 public:
  NavL0() : Node("nav_l0_perception") {
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_l_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    rog_map::Config cfg;
    cfg.loadFromRosNode(this, "rog_map");
    rog_ = std::make_shared<rog_map::ROGMapROS>(shared_from_this(), cfg, tf_);

    pub_grid_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/nav/l0/grid", 1);
    pub_obs_ = create_publisher<visualization_msgs::msg::MarkerArray>("/nav/l0/dbg/occupied", 1);
    pub_st_ = create_publisher<rm_nav_interfaces::msg::StageStatus>("/nav/l0/status", 1);
    // 允许闸门参数外置，便于单独调试（阈值过大即"不可信"）
    occ_max_ = declare_parameter("nav_l0.occ_max_ratio", 0.40);
    timer_ = create_wall_timer(200ms, std::bind(&NavL0::tick, this));
    RCLCPP_INFO(get_logger(), "[L0] 就绪: cloud=%s res=%.2f 窗口 %.1fx%.1f m",
                cfg.cloud_topic.c_str(), cfg.resolution, cfg.map_size_d.x(), cfg.map_size_d.y());
  }

 private:
  void tick() {
    auto q = rog_map::MapRegistry::get();
    if (!q) return;
    const double res = q->resolution();
    const int W = static_cast<int>(q->sizeX()), H = static_cast<int>(q->sizeY());
    if (W <= 2 || H <= 2 || res <= 0.0) return;
    unsigned int mx0 = 0, my0 = 0;
    const double ox = q->originX(), oy = q->originY();
    nav_msgs::msg::OccupancyGrid g;
    g.header.stamp = now();
    g.header.frame_id = "map";
    g.info.resolution = res;
    g.info.width = W; g.info.height = H;
    g.info.origin.position.x = ox;
    g.info.origin.position.y = oy;
    g.data.assign(static_cast<size_t>(W) * H, -1);
    int n_occ = 0, n_in = 0;
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
      const double wx = ox + (x + 0.5) * res, wy = oy + (y + 0.5) * res;
      if (!q->worldToMap(wx, wy, mx0, my0) || !q->isValid(mx0, my0)) continue;
      ++n_in;
      const uint8_t v = q->value(mx0, my0);
      if (v == 254U) { g.data[y * W + x] = 100; ++n_occ; }
      else if (v == 255U) g.data[y * W + x] = -1;
      else g.data[y * W + x] = 0;
    }
    pub_grid_->publish(g);
    const double ratio = n_in > 0 ? static_cast<double>(n_occ) / n_in : 0.0;
    if (pub_st_) {
      rm_nav_interfaces::msg::StageStatus st;
      st.header.stamp = now(); st.stage = 0;
      st.state = ratio > occ_max_ ? 3 : (n_occ > 0 ? 1 : 2);
      st.detail = ratio > occ_max_ ? "占用率不合理" : (n_occ > 0 ? "正常" : "无占用");
      st.keys.resize(3); st.values.resize(3);
      st.keys[0] = "occupied"; st.keys[1] = "inside"; st.keys[2] = "ratio";
      st.values[0] = n_occ; st.values[1] = n_in; st.values[2] = ratio;
      pub_st_->publish(st);
    }
  }
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_l_;
  std::shared_ptr<rog_map::ROGMapROS> rog_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obs_;
  rclcpp::Publisher<rm_nav_interfaces::msg::StageStatus>::SharedPtr pub_st_;
  rclcpp::TimerBase::SharedPtr timer_;
  double occ_max_ = 0.40;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavL0>());
  rclcpp::shutdown();
  return 0;
}
