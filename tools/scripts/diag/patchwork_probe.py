#!/usr/bin/env python3
"""patchwork_probe.py —— 看 Patchwork++ 到底把"离地多高"的点判成了 nonground（障碍）。

为什么要它：Patchwork 只回答"是不是地面"，**高度筛选是 STVL 做的**。所以"门口被标成障碍"
只可能是两种：
  (a) patchwork 把**地面点**漏判成 nonground（th_dist 太紧 / 坡面 / 台阶边缘）⇒ 贴地点的
      离地高度 ≈ 0~5 cm 却出现在 obstacle 里；
  (b) 那里真有 5~18 cm 高的东西（门槛/矮台沿）⇒ patchwork 判 nonground 是对的，是 STVL 的
      上界（0.18 m）把它算进了代价图。

用法（仿真跑起来时）：
  python3 tools/scripts/diag/patchwork_probe.py                 # 默认 livox 离地 0.188 m
  python3 tools/scripts/diag/patchwork_probe.py --h 0.226       # 换模型时改离地高度
  python3 tools/scripts/diag/patchwork_probe.py --r 2.0         # 只看 2 m 内（默认 3 m）

输出（1 Hz）：obstacle / ground 的点数，以及 obstacle 的"离地高度"分档计数。
判读：
  · 0~5 cm 一档有很多点 ⇒ (a) 地面被漏判（调 th_dist 0.08→0.12 或开 RNR）
  · 5~18 cm 有点 ⇒ (b) 真的有矮物（看它在哪：`--r 1.0` 逐步缩小半径可定位方位角）
  · 若 obstacle 里 0~18 cm 一个都没有 ⇒ 门口根本没问题，别再怀疑 patchwork/STVL
"""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Probe(Node):
    def __init__(self, h, r):
        super().__init__('patchwork_probe')
        self.h = h
        self.r = r
        self.obs = None
        self.gnd = None
        # ★ patchwork 的两个输出是 **sensor-data QoS（BEST_EFFORT）**；订阅端必须 BEST_EFFORT，
        #   否则报 "offering incompatible QoS ... Last incompatible policy: RELIABILITY"（一条也收不到）。
        #   BEST_EFFORT 订阅对 RELIABLE 发布端同样兼容，所以这里一律用 BEST_EFFORT。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, '/segmentation/obstacle', self.cb_obs, qos)
        self.create_subscription(PointCloud2, '/segmentation/ground', self.cb_gnd, qos)
        self.create_timer(1.0, self.tick)

    @staticmethod
    def _xyz(msg):
        try:
            p = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:
            return None
        if p.size == 0:
            return np.zeros((0, 3))
        if getattr(p.dtype, 'names', None):
            return np.stack([p['x'], p['y'], p['z']], 1).astype(np.float64)
        return np.asarray(p, dtype=np.float64).reshape(-1, 3)

    def cb_obs(self, m): self.obs = (m.header.frame_id, self._xyz(m))
    def cb_gnd(self, m): self.gnd = (m.header.frame_id, self._xyz(m))

    def tick(self):
        if self.obs is None or self.gnd is None:
            self.get_logger().info('等 /segmentation/obstacle + /segmentation/ground ...')
            return
        frame, o = self.obs
        _, g = self.gnd
        if o is None or g is None:
            self.get_logger().warn('点云解析为空'); return
        rxy = np.hypot(o[:, 0], o[:, 1]) if len(o) else np.zeros(0)
        near = o[rxy <= self.r] if len(o) else o
        zg = np.hypot(g[:, 0], g[:, 1]) if len(g) else np.zeros(0)
        gnear = g[zg <= self.r] if len(g) else g
        hh = near[:, 2] + self.h  # 离地高度（livox 在 +h；chassis 无倾角）
        bins = [(0.0, 0.05), (0.05, 0.10), (0.10, 0.18), (0.18, 0.35), (0.35, 0.8), (0.8, 99)]
        parts = ['%.2f~%.2f m: %d' % (a, b, int(((hh >= a) & (hh < b)).sum())) for a, b in bins]
        self.get_logger().info(
            '[%s] r<=%.1fm  obstacle=%d (其中近场 %d) | ground=%d (近场 %d)\n'
            '   obstacle 离地高度分档: %s\n'
            '   obstacle 离地 min/p50/max = %.2f / %.2f / %.2f m'
            % (frame, self.r, len(o), len(near), len(g), len(gnear),
               ' | '.join(parts),
               float(hh.min()) if len(hh) else float('nan'),
               float(np.median(hh)) if len(hh) else float('nan'),
               float(hh.max()) if len(hh) else float('nan')))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--h', type=float, default=0.188, help='传感器离地高度 (m)')
    ap.add_argument('--r', type=float, default=3.0, help='只看这个半径内的点 (m)')
    a = ap.parse_args()
    rclpy.init()
    n = Probe(a.h, a.r)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
