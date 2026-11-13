// 3D 占据 + 逐柱语义投影（对标 BIT 的 ROGMap + ProjectionLayer，2.5D 输出）
//
// 为什么需要它（mask_builder 的问题）：
//   mask_builder 只做「z 区间一刀切 + 每帧清空」：
//     · 坡道上的点全落在带内 ⇒ 整条坡被打成障碍 ⇒ 车永远不敢上坡
//     · 悬空横杆下方的净空表达不了 ⇒ 能钻的地方被判成堵死
//     · 没有 raycast 空闲清空、没有衰减 ⇒ 噪点挡满一整个周期
//
// 本节点的处理链：
//   点云 → 3D 体素(0.10 m) hit/miss → 时间衰减
//        → 逐 XY 柱投影 → 语义分类(FREE/PASSABLE/PASS_UNDER/OCCUPIED)
//        → 0.05 m 栅格输出
//
// 输出：
//   /plan_mask      OccupancyGrid  只把 OCCUPIED 标 100（坡、悬空下方 = 可通行）
//   /terrain_class  OccupancyGrid  0=FREE 1=PASSABLE 2=OCCUPIED 3=PASS_UNDER（调试/可视化）
//
// 判据（量级参考 BIT 的 ProjectionLayer）：
//   · 柱内最低占据层离地 < ground_z(0.15) 且 连续跨度 <= surface_span(0.12) ⇒ PASSABLE（坡/缓台）
//   · 最低占据层 >= robot_height + pass_under_margin ⇒ PASS_UNDER（横杆下方可钻）
//   · 其余（墙、高台侧面、跨度大）⇒ OCCUPIED
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <cstdint>
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

namespace {

constexpr int8_t kClsFree = 0;
constexpr int8_t kClsPassable = 1;
constexpr int8_t kClsOccupied = 2;
constexpr int8_t kClsPassUnder = 3;

inline int8_t clamp8(int v, int lo, int hi) {
  return static_cast<int8_t>(v < lo ? lo : (v > hi ? hi : v));
}

}  // namespace

class TerrainLayer : public rclcpp::Node {
 public:
  TerrainLayer() : Node("terrain_layer") {
    // ---- 体素 ----
    vres_ = declare_parameter("voxel_resolution", 0.10);
    // ★ 与 BIT ProjectionLayer 同参数（scan_z_min_abs / scan_z_max_abs）
    z_min_ = declare_parameter("z_min", -0.20);
    z_max_ = declare_parameter("z_max", 1.50);
    hit_gain_ = declare_parameter("hit_gain", 4);
    miss_penalty_ = declare_parameter("miss_penalty", 1);
    decay_per_frame_ = declare_parameter("decay_per_frame", 1);
    occ_threshold_ = declare_parameter("occ_threshold", 1);
    max_logodd_ = declare_parameter("max_logodd", 30);
    min_logodd_ = declare_parameter("min_logodd", -8);

    // ---- 语义判据 ----
    // BIT: surface_height_delta_max 0.10 / wall_height_delta_min 0.20
    //      tunnel_height_delta_min 0.25 / tunnel_height_delta_max 0.40
    ground_z_ = declare_parameter("ground_z", -0.14);
    surface_span_ = declare_parameter("surface_height_delta_max", 0.10);
    wall_min_span_ = declare_parameter("wall_height_delta_min", 0.20);
    tunnel_min_ = declare_parameter("tunnel_height_delta_min", 0.25);
    tunnel_max_ = declare_parameter("tunnel_height_delta_max", 0.40);
    robot_height_ = declare_parameter("robot_height", 0.45);
    pass_under_margin_ = declare_parameter("pass_under_margin", 0.15);

    // ---- 自车过滤 / 射线 ----
    self_radius_ = declare_parameter("self_radius", 0.35);
    use_raycast_ = declare_parameter("use_raycast", true);
    ray_stride_ = declare_parameter("ray_stride", 8);
    ray_max_range_ = declare_parameter("ray_max_range", 12.0);
    ray_origin_z_ = declare_parameter("ray_origin_z", 0.20);

    // ---- 输入/输出 ----
    in_topic_ = declare_parameter("input_topic", std::string("/localization/lio_world_cloud"));
    frame_ = declare_parameter("frame_id", std::string("map"));
    robot_frame_ = declare_parameter("robot_frame", std::string("base_link"));
    process_hz_ = declare_parameter("process_hz", 5.0);

    // ---- 2D 输出网格（默认从 map yaml 取，保证与 /map 完全一致）----
    res_ = declare_parameter("resolution", 0.05);
    w_ = declare_parameter("width", 560);
    h_ = declare_parameter("height", 313);
    ox_ = declare_parameter("origin_x", -19.9);
    oy_ = declare_parameter("origin_y", -8.89);
    const auto map_yaml = declare_parameter("map_yaml", std::string(""));
    if (!map_yaml.empty()) loadMapYaml(map_yaml);

    // 体素网格覆盖整张 2D 图（场地 28×15.7 m × 2 m @0.10 ⇒ 约 110 万体素，内存 ~1 MB）
    vx_ = static_cast<int>(std::ceil(w_ * res_ / vres_));
    vy_ = static_cast<int>(std::ceil(h_ * res_ / vres_));
    vz_ = static_cast<int>(std::ceil((z_max_ - z_min_) / vres_));
    vox_.assign(static_cast<size_t>(vx_) * vy_ * vz_, 0);
    cls_.assign(static_cast<size_t>(vx_) * vy_, kClsFree);
    mask_.assign(static_cast<size_t>(w_) * h_, 0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_mask_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/plan_mask", rclcpp::QoS(1));
    pub_cls_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain_class", rclcpp::QoS(1));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        in_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { onCloud(m); });

    last_proc_ = now();
    RCLCPP_INFO(get_logger(),
                "[terrain] 3D 体素 %dx%dx%d @ %.2f m (z≤%.1f) → 2D %dx%d @ %.2f m; "
                "robot_h=%.2f pass_under_margin=%.2f surface_span=%.2f",
                vx_, vy_, vz_, vres_, z_max_, w_, h_, res_, robot_height_, pass_under_margin_,
                surface_span_);
  }

 private:
  void loadMapYaml(const std::string& path) {
    // 复用 mask_builder 的解析方式：只取 resolution / origin / PGM 尺寸
    std::ifstream f(path);
    if (!f) return;
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
    const std::string base = path.substr(0, path.find_last_of('/') + 1);
    std::ifstream img(base + image, std::ios::binary);
    if (img) {
      std::string magic; int w = 0, hh = 0, mx = 0;
      img >> magic >> w >> hh >> mx;
      if (magic == "P5" && w > 0 && hh > 0) { w_ = w; h_ = hh; }
    }
    RCLCPP_INFO(get_logger(), "[terrain] 从 yaml 读取: %dx%d @ %.3f m, origin (%.2f, %.2f)",
                w_, h_, res_, ox_, oy_);
  }

  inline size_t vidx(int ix, int iy, int iz) const {
    return (static_cast<size_t>(iz) * vy_ + iy) * vx_ + ix;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    const double period = 1.0 / std::max(0.1, process_hz_);
    if ((now() - last_proc_).seconds() < period) return;   // 限频
    last_proc_ = now();
    const auto t0 = std::chrono::steady_clock::now();

    // ---- ① 时间衰减：所有占据体素向 0 靠 ----
    if (decay_per_frame_ > 0) {
      for (auto& v : vox_)
        if (v > 0) v = static_cast<int8_t>(v > decay_per_frame_ ? v - decay_per_frame_ : 0);
    }

    // ---- ② 机器人位置（map 系）：用于自车过滤与射线起点 ----
    Eigen::Vector3d robot(0.0, 0.0, 0.0);
    bool have_robot = false;
    if (!robot_frame_.empty()) {
      // ★ 用**最新** TF，不用消息时间戳：时间戳查不到时会阻塞 50 ms（实测把单帧
      //   推进到 274 ms），而自车过滤/射线起点并不需要严格对齐到点云时刻。
      try {
        const auto tf = tf_buffer_->lookupTransform(frame_, robot_frame_, tf2::TimePointZero);
        robot = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, 0.0);
        have_robot = true;
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                            "[terrain] 查不到 %s→%s: %s", frame_.c_str(), robot_frame_.c_str(), e.what());
      }
    }
    const Eigen::Vector3d ray_origin(robot.x(), robot.y(), ray_origin_z_);

    // ---- ③ 点云转 frame_（若不同）+ 插入 hit / raycast miss ----
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    bool need_tf = (msg->header.frame_id != frame_);
    if (need_tf) {
      try {
        const auto tf = tf_buffer_->lookupTransform(frame_, msg->header.frame_id,
                                                    rclcpp::Time(msg->header.stamp),
                                                    rclcpp::Duration::from_seconds(0.005));
        Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                             tf.transform.rotation.y, tf.transform.rotation.z);
        T.linear() = q.toRotationMatrix();
        T.translation() = Eigen::Vector3d(tf.transform.translation.x,
                                          tf.transform.translation.y,
                                          tf.transform.translation.z);
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                            "[terrain] 点云转 %s 失败: %s", frame_.c_str(), e.what());
        return;
      }
    }

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    int n_pts = 0, n_hit = 0, n_self = 0, n_ray = 0;
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      Eigen::Vector3d p(*ix, *iy, *iz);
      if (!p.allFinite()) continue;
      if (need_tf) p = T * p;
      if (p.z() < z_min_ || p.z() > z_max_) continue;
      // 自车过滤（map 系半径）
      if (have_robot && std::hypot(p.x() - robot.x(), p.y() - robot.y()) < self_radius_) {
        ++n_self;
        continue;
      }
      const int vxi = static_cast<int>((p.x() - ox_) / vres_);
      const int vyi = static_cast<int>((p.y() - oy_) / vres_);
      const int vzi = static_cast<int>((p.z() - z_min_) / vres_);
      if (vxi < 0 || vyi < 0 || vzi < 0 || vxi >= vx_ || vyi >= vy_ || vzi >= vz_) continue;
      ++n_pts;
      ++n_hit;
      auto& cell = vox_[vidx(vxi, vyi, vzi)];
      cell = clamp8(cell + hit_gain_, min_logodd_, max_logodd_);

      // 射线清空：抽样，避免每点都走
      if (use_raycast_ && have_robot && (n_pts % std::max(1, ray_stride_) == 0)) {
        const Eigen::Vector3d d = p - ray_origin;
        const double len = d.norm();
        if (len > 0.2 && len < ray_max_range_) {
          const int steps = static_cast<int>(len / vres_);
          for (int s = 1; s < steps; ++s) {
            const Eigen::Vector3d q = ray_origin + d * (static_cast<double>(s) / steps) * 0.98;
            if (q.z() < z_min_ || q.z() > z_max_) continue;
            const int xi = static_cast<int>((q.x() - ox_) / vres_);
            const int yi = static_cast<int>((q.y() - oy_) / vres_);
            const int zi = static_cast<int>((q.z() - z_min_) / vres_);
            if (xi < 0 || yi < 0 || zi < 0 || xi >= vx_ || yi >= vy_ || zi >= vz_) continue;
            auto& m = vox_[vidx(xi, yi, zi)];
            m = clamp8(m - miss_penalty_, min_logodd_, max_logodd_);
            ++n_ray;
          }
        }
      }
    }

    // ---- ④ 逐柱投影 → 语义分类（0.10 m）----
    // BIT 判据：柱内占据高度差 delta = z_top - z_bottom
    //   delta <= surface_max(0.10)                     => PASSABLE（坡/缓台）
    //   净空 >= tunnel_min(0.25) 且 delta <= tunnel_max(0.40) => PASS_UNDER（可钻）
    //   其余（delta >= wall_min(0.20)）                => OCCUPIED（墙）
    const int z_need = static_cast<int>(std::ceil((robot_height_ + pass_under_margin_) / vres_));
    int n_occ = 0, n_pass = 0, n_under = 0;
    for (int y = 0; y < vy_; ++y) {
      for (int x = 0; x < vx_; ++x) {
        // 找最低占据层
        int z0 = -1;
        for (int z = 0; z < vz_; ++z) {
          if (vox_[vidx(x, y, z)] > occ_threshold_) { z0 = z; break; }
        }
        int8_t cls = kClsFree;
        if (z0 >= 0) {
          // 从 z0 往上找连续占据的顶端（允许 1 格空隙，避免栅格量化把坡面拆散）
          int z1 = z0;
          int gap = 0;
          for (int z = z0 + 1; z < vz_; ++z) {
            if (vox_[vidx(x, y, z)] > occ_threshold_) { z1 = z; gap = 0; }
            else if (++gap > 1) break;
          }
          const double z_bottom = z_min_ + z0 * vres_;
          const double height_delta = (z1 - z0 + 1) * vres_;
          const double clearance = z_bottom - ground_z_;
          if (z0 >= z_need) {
            // ★ 只有"下方确实被射线清空过"才允许判可钻。
            //   否则是**遮挡**造成的假象：远处墙只被扫到顶部 ⇒ 最低占据层很高
            //   ⇒ 会整片误判成"可钻"（首测 218/403 格就是这么来的）。
            // 遮挡保护：远处墙只扫到顶部时 z_bottom 很高 ⇒ 必须"下方被射线清空过"才算可钻
            int cleared = 0;
            for (int z = 0; z < z0; ++z)
              if (vox_[vidx(x, y, z)] < 0) ++cleared;
            const bool below_cleared = (cleared >= 2);   // 至少 2 层被射线清空，抑制遮挡
            if (clearance >= tunnel_min_ && height_delta <= tunnel_max_ && below_cleared)
              cls = kClsPassUnder;            // 头顶横杆/隧道：可钻
            else
              cls = kClsOccupied;             // 高度不够或没被清空过
          } else if (height_delta <= surface_span_) {
            cls = kClsPassable;               // 坡/缓台：可走
          } else {
            cls = kClsOccupied;               // 墙 / 高台侧面
          }
        }
        cls_[static_cast<size_t>(y) * vx_ + x] = cls;
        n_occ += (cls == kClsOccupied);
        n_pass += (cls == kClsPassable);
        n_under += (cls == kClsPassUnder);
      }
    }

    // ---- ⑤ 0.10 → 0.05 上采样，只把 OCCUPIED 变成障碍 ----
    std::fill(mask_.begin(), mask_.end(), 0);
    const double ratio = vres_ / res_;                       // 通常 = 2
    for (int y = 0; y < h_; ++y) {
      for (int x = 0; x < w_; ++x) {
        const int vx_i = static_cast<int>((x + 0.5) * ratio);
        const int vy_i = static_cast<int>((y + 0.5) * ratio);
        if (vx_i < 0 || vy_i < 0 || vx_i >= vx_ || vy_i >= vy_) continue;
        if (cls_[static_cast<size_t>(vy_i) * vx_ + vx_i] == kClsOccupied)
          mask_[static_cast<size_t>(y) * w_ + x] = 100;
      }
    }

    // ---- ⑥ 发布 ----
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = frame_;
    out.info.resolution = res_;
    out.info.width = w_;
    out.info.height = h_;
    out.info.origin.position.x = ox_;
    out.info.origin.position.y = oy_;
    out.info.origin.orientation.w = 1.0;
    out.data = mask_;
    pub_mask_->publish(out);

    nav_msgs::msg::OccupancyGrid cls_msg = out;
    cls_msg.data.resize(static_cast<size_t>(w_) * h_, 0);
    for (int y = 0; y < h_; ++y)
      for (int x = 0; x < w_; ++x) {
        const int vx_i = static_cast<int>((x + 0.5) * ratio);
        const int vy_i = static_cast<int>((y + 0.5) * ratio);
        if (vx_i >= 0 && vy_i >= 0 && vx_i < vx_ && vy_i < vy_)
          cls_msg.data[static_cast<size_t>(y) * w_ + x] =
              cls_[static_cast<size_t>(vy_i) * vx_ + vx_i];
      }
    pub_cls_->publish(cls_msg);

    const auto ms = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - t0).count() / 1000.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                        "[terrain] 点 %d(自车滤 %d) 占有 %d 坡 %d 可钻 %d | 耗时 %.1f ms",
                        n_pts, n_self, n_occ, n_pass, n_under, ms);
    (void)n_ray; (void)n_hit;
  }

  // 参数
  double vres_ = 0.10, z_min_ = -0.20, z_max_ = 1.50;
  int hit_gain_ = 4, miss_penalty_ = 1, decay_per_frame_ = 1, occ_threshold_ = 1;
  int max_logodd_ = 30, min_logodd_ = -8;
  double ground_z_ = -0.14, surface_span_ = 0.10, wall_min_span_ = 0.20;
  double tunnel_min_ = 0.25, tunnel_max_ = 0.40;
  double robot_height_ = 0.45, pass_under_margin_ = 0.15;
  double self_radius_ = 0.35, ray_max_range_ = 12.0, ray_origin_z_ = 0.20;
  int ray_stride_ = 8;
  bool use_raycast_ = true;
  double process_hz_ = 5.0;
  std::string in_topic_, frame_, robot_frame_;
  // 2D 网格
  double res_ = 0.05, ox_ = -19.9, oy_ = -8.89;
  int w_ = 560, h_ = 313;
  // 体素
  int vx_ = 0, vy_ = 0, vz_ = 0;
  std::vector<int8_t> vox_, cls_;
  std::vector<int8_t> mask_;
  rclcpp::Time last_proc_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mask_, pub_cls_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainLayer>());
  rclcpp::shutdown();
  return 0;
}
