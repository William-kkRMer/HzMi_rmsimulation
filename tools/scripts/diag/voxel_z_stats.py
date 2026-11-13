#!/usr/bin/env python3
"""voxel_z_stats.py —— 统计 STVL 发布的体素点云在 **z 上的分布**，用来验证"高度带"到底落在哪。

背景（2026-11-02）：
  · 局部 STVL 在 `odom_level` 帧（地面 z≈0）⇒ [0.0, 0.20] 就是"离地 0~20 cm"；
  · 全局 STVL 在 `map` 帧，而 map 的 z 原点在地面上方 0.138 m（静态边 T_odom←odom_level z=−0.138）
    ⇒ 同一个 [0.0, 0.20] 其实是"离地 0.138~0.338"。
  · 本实验把全局改成 min=-0.138 / max=0.062（= 离地 0~0.20）。这个脚本就是用来**看结果**的。

用法：
  # 1) 先确认话题名（STVL 的 publish_voxel_map: true 才会发）：
  ros2 topic list | grep -i voxel
  # 2) 订阅它（默认 1 Hz 打印一次）：
  python3 tools/scripts/diag/voxel_z_stats.py /global_costmap/voxel_grid

判读：
  · 打印的 frame_id 应是 `map`（全局）或 `odom_level`（局部）；
  · 改前（min 0.0 / max 0.20）：z 全 ≥ 0，典型 0.00~0.20 ⇒ 相对地面 0.138~0.338；
  · 改后（min -0.138 / max 0.062）：z 应变成 -0.138~0.062 ⇒ 相对地面 0~0.20；
  · 若改后**一个点都没有** ⇒ STVL 不接受负 z（栅格原点固定）⇒ 回退 min 0.0 / max 0.20。
"""
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# map 帧里"地面"的 z（见静态边 T_odom←odom_level z=−0.138）
GROUND_Z_IN_MAP = -0.138


class VoxelZStats(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('voxel_z_stats')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(PointCloud2, topic, self.on_cloud, qos)
        self.topic = topic
        self.last = None
        self.get_logger().info('订阅 %s（1 Hz 统计，Ctrl-C 结束）' % topic)

    def on_cloud(self, msg: PointCloud2) -> None:
        try:
            pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                                 skip_nans=True)
        except Exception as exc:  # 老版本没有 read_points_numpy
            self.get_logger().error('read_points_numpy 失败：%s' % exc)
            return
        if pts.size == 0:
            self.last = (msg.header.frame_id, None)
            return
        # 兼容两种返回：结构化数组(有字段名) 或 普通 (N,3) 数组
        if getattr(pts.dtype, 'names', None):
            z = np.asarray(pts['z'], dtype=np.float64)
        else:
            arr = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
            z = arr[:, 2]
        self.last = (msg.header.frame_id, z)

    def tick(self) -> None:
        if self.last is None:
            return
        frame, z = self.last
        if z is None:
            self.get_logger().warn('[%s] 空点云（frame=%s）' % (self.topic, frame))
            return
        pct = np.percentile(z, [1, 50, 99])
        n = z.size
        above = int((z > 0.062).sum())
        below = int((z < GROUND_Z_IN_MAP - 0.02).sum())
        rel = ('地面相对值 z-(-0.138)：%.3f / %.3f / %.3f m'
               % (pct[0] + 0.138, pct[1] + 0.138, pct[2] + 0.138)
               if frame == 'map' else '（frame 不是 map，无偏移）')
        self.get_logger().info(
            '[%s] frame=%s n=%d  z: min=%.3f p1=%.3f p50=%.3f p99=%.3f max=%.3f  %s  '
            '| 超上界(>0.062)=%d  低于地面-2cm=%d'
            % (self.topic, frame, n, float(z.min()), pct[0], pct[1], pct[2], float(z.max()),
               rel, above, below))


def main() -> int:
    topic = sys.argv[1] if len(sys.argv) > 1 else '/global_costmap/voxel_grid'
    rclpy.init()
    node = VoxelZStats(topic)
    node.create_timer(1.0, node.tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
