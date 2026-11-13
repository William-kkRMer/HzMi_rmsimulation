#!/usr/bin/env python3
"""量一下 zlio 的世界系点云（只读，不改任何行为）。

回答三个问题：
  1. 频率、每帧点数 —— 够不够 ROGMap / MINCO 用
  2. 是否是稠密云（点数 + 覆盖面积 → 密度 pts/m²）
  3. 有没有逐点时间字段 —— 能不能做去畸变

用法：
  python3 tools/scripts/nav/measure_cloud.py                 # 默认量 10 s
  python3 tools/scripts/nav/measure_cloud.py --seconds 20
  python3 tools/scripts/nav/measure_cloud.py --cloud /cloud_registered
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

try:
    from sensor_msgs_py import point_cloud2 as pc2
    _HAS_PC2 = True
except Exception:
    _HAS_PC2 = False


class Measure(Node):
    def __init__(self, cloud_topic, odom_topic):
        super().__init__('measure_cloud')
        self.frames = []          # (n_pts, zmin, zmax, xmin, xmax, ymin, ymax, frame_id)
        self.fields = None
        self.odom_times = []
        self.cloud_stamps = []     # 消息头时间戳（可能是 sim time）
        self.odom_stamps = []
        self.zmin_seen = 1e9
        self.zmax_seen = -1e9
        self.xy_area = []
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 50)
        self.get_logger().info(f'量测中: cloud={cloud_topic} odom={odom_topic}')

    def on_odom(self, msg):
        self.odom_times.append(time.time())
        self.odom_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def on_cloud(self, msg):
        n = int(msg.width) * int(msg.height)
        if self.fields is None:
            self.fields = [f.name for f in msg.fields]
        zmin, zmax, xr, yr = None, None, None, None
        if _HAS_PC2 and msg.height * msg.width > 0:
            try:
                pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
                if pts.size:
                    z = pts[:, 2]
                    z = z[np.isfinite(z)]
                    if z.size:
                        zmin, zmax = float(z.min()), float(z.max())
                        self.zmin_seen = min(self.zmin_seen, zmin)
                        self.zmax_seen = max(self.zmax_seen, zmax)
                        x = pts[:, 0]; y = pts[:, 1]
                        m = np.isfinite(x) & np.isfinite(y)
                        if m.any():
                            xr = float(x[m].max() - x[m].min())
                            yr = float(y[m].max() - y[m].min())
            except Exception:
                pass
        self.frames.append((n, zmin, zmax, xr, yr, msg.header.frame_id))
        self.cloud_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if xr and yr:
            self.xy_area.append(xr * yr)

    def report(self, elapsed):
        if not self.frames:
            print('\n❌ 没收到任何点云。')
            print('   常见原因：')
            print('     1) 测量跑在 launch 之前（zlio 还没起来）——先起仿真，等 NORMAL 再量')
            print('     2) 论文本终端和 launch 的 RMW 不一致：两边都要 export RMW_IMPLEMENTATION=rmw_fastrtps_cpp')
            print('     3) 话题名不对')
            # 自动列出候选话题，省得猜
            try:
                print('\n   当前系统里带 cloud/lidar 的话题：')
                found = False
                for name, types in self.get_topic_names_and_types():
                    low = name.lower()
                    if 'cloud' in low or 'lidar' in low or 'scan' in low:
                        pubs = self.count_publishers(name)
                        print(f'     {name}  [{",".join(types)}]  publishers={pubs}')
                        found = True
                if not found:
                    print('     （一个都没有 ⇒ zlio 没起来，或 RMW 不通）')
            except Exception as e:
                print('     列话题失败:', e)
            return
        ns = np.array([f[0] for f in self.frames], dtype=float)
        hz = len(self.frames) / elapsed if elapsed > 0 else 0.0
        ohz = len(self.odom_times) / elapsed if elapsed > 0 else 0.0
        print('\n================ 点云量测 ================')
        print(f'  帧数            : {len(self.frames)}   时长 {elapsed:.1f} s')
        print(f'  频率(墙钟)      : {hz:.2f} Hz')
        if len(self.cloud_stamps) >= 2:
            span = self.cloud_stamps[-1] - self.cloud_stamps[0]
            if span > 1e-6:
                print(f'  频率(消息时间戳): {(len(self.cloud_stamps)-1)/span:.2f} Hz   '
                      f'← 仿真里这是 sim time（RTF<1 时墙钟会小很多）')
        if len(self.odom_stamps) >= 2:
            sp = self.odom_stamps[-1] - self.odom_stamps[0]
            if sp > 1e-6:
                print(f'  里程计(时间戳)  : {(len(self.odom_stamps)-1)/sp:.2f} Hz')
        print(f'  每帧点数        : 中位 {np.median(ns):.0f}  min {ns.min():.0f}  max {ns.max():.0f}')
        print(f'  累计点数        : {ns.sum():.0f}')
        print(f'  frame_id        : {self.frames[-1][5]}')
        print(f'  z 范围(全局)    : [{self.zmin_seen:+.2f}, {self.zmax_seen:+.2f}] m')
        if self.xy_area:
            a = np.median(self.xy_area)
            print(f'  覆盖面积(中位)  : {a:.1f} m²')
            if a > 1e-6:
                print(f'  密度            : {np.median(ns) / a:.1f} pts/m²')
        print(f'  里程计频率      : {ohz:.2f} Hz')
        print(f'  点云字段        : {self.fields}')
        tf_field = [f for f in (self.fields or []) if f in ('offset_time', 'time', 't', 'timestamp', 't_sec')]
        print(f'  逐点时间字段    : {tf_field if tf_field else "无（无法逐点去畸变）"}')
        print('==========================================')
        print('判读参考：')
        print('  · 频率 ≥ 5 Hz、每帧 ≥ 1000 点  ⇒ ROGMap/MINCO 可用')
        print('  · 密度 ≥ 200 pts/m²          ⇒ 投影层能分出薄面/墙（稀疏则薄面判据不稳）')
        print('  · 有点时间字段                ⇒ 可做逐点去畸变（BIT 的 full 云有）')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cloud', default='/localization/lio_world_cloud')
    ap.add_argument('--odom', default='/localization/odometry')
    ap.add_argument('--seconds', type=float, default=10.0)
    a = ap.parse_args()

    rclpy.init()
    node = Measure(a.cloud, a.odom)
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.report(time.time() - t0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
