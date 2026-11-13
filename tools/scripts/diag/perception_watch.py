#!/usr/bin/env python3
"""perception_watch.py —— 长时间挂着跑，把"感知到底在挡什么"打成一条数据流。

回答的问题（就是你问的"能不能一直跑拿一串数据"）：
  · 每个时刻，/segmentation/obstacle 里**离地 0~18 cm** 的点有多少、在哪（车头方向多不多）；
  · 车在哪（map 坐标）——这样事后能对上是"哪个门/哪段坡"出的问题；
  · 数据同时写 CSV，跑完直接把文件（或最后几十行）给我。

用法（仿真跑起来后，**另开一个终端**，一直挂着，不用管它）：
  python3 tools/scripts/diag/perception_watch.py                 # 2 Hz，打印 + 写 CSV
  python3 tools/scripts/diag/perception_watch.py --r 2.0         # 只统计 2 m 内
  python3 tools/scripts/diag/perception_watch.py --h 0.188       # 传感器离地高度（默认 0.188）
  python3 tools/scripts/diag/perception_watch.py --rate 5        # 5 Hz

退出：Ctrl-C。CSV 路径在启动时打印（默认 ~/.ros/perception_watch_<时间>.csv）。

判读（关键就一条）：
  · obs_0_18_front 持续 > 0 且车正在朝那个方向走 ⇒ **感知在车头前方 0~18 cm 高度有点**
    → 把 CSV 里那几行的时间戳告我，我按 map 坐标去查那块是"坡口/台阶沿"还是"地面残点"；
  · 全程 obs_0_18 ≈ 0 ⇒ patchwork 在 0~18 cm 一个点都没产，感知无罪，问题在膨胀/规划。
"""
import argparse
import csv
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


class Watch(Node):
    def __init__(self, h, r, rate, csv_path):
        super().__init__('perception_watch')
        self.h, self.r, self.rate = h, r, rate
        self.obs = None
        self.gnd = None
        self.tf = None          # (x, y, yaw) of base_link in the cloud frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.csv_path = csv_path
        new = not os.path.exists(csv_path)
        self.f = open(csv_path, 'a', newline='')
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(['t', 'x', 'y', 'yaw_deg', 'frame',
                             'obs_n', 'obs_r_n', 'obs_0_5', 'obs_5_18', 'obs_18_35',
                             'front_0_18', 'gnd_n', 'clearest_m',
                             'band_r03', 'band_r05', 'band_r10', 'band_r20',
                             'sec_F', 'sec_B', 'sec_L', 'sec_R',
                             'gnd_lt1', 'gnd_1_3', 'gnd_gt3'])
        self.f.flush()
        # ★ patchwork 的两个输出是 **sensor-data QoS（BEST_EFFORT）**；订阅端必须 BEST_EFFORT，
        #   否则报 "offering incompatible QoS ... Last incompatible policy: RELIABILITY"（一条也收不到）。
        #   BEST_EFFORT 订阅对 RELIABLE 发布端同样兼容，所以这里一律用 BEST_EFFORT。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, '/segmentation/obstacle', self.cb_obs, qos)
        self.create_subscription(PointCloud2, '/segmentation/ground', self.cb_gnd, qos)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('写 CSV: %s' % csv_path)

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

    def cb_gnd(self, m): self.gnd = self._xyz(m)

    def _pose(self):
        for parent in ('map', 'odom_level', 'odom'):
            try:
                tr = self.tf_buffer.lookup_transform(parent, 'base_link', Time(),
                                                     timeout=rclpy.duration.Duration(seconds=0.05))
                p = tr.transform.translation
                q = tr.transform.rotation
                yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                              1 - 2 * (q.y * q.y + q.z * q.z)))
                return p.x, p.y, yaw, parent
            except Exception:
                continue
        return float('nan'), float('nan'), float('nan'), '-'

    def tick(self):
        if self.obs is None:
            self.get_logger().info('等 /segmentation/obstacle ...'); return
        frame, o = self.obs
        if o is None or len(o) == 0:
            self.get_logger().info('[%s] obstacle 云为空（这一帧没有任何 nonground 点）' % frame); return
        # ★ 2026-11-02 修正：/segmentation/obstacle 的 frame 是 **map**（不是 livox_frame）！
        #   所以"距离"必须换算到**车体系**；"离地高度"= map z + 0.138（map 的地面在 z=-0.138）。
        #   之前把 map 坐标当"离车多远"算，得到的"<0.3m 有一千多个点"其实是**地图原点附近**，
        #   不是车周围 —— 那个"自击晕"结论作废。
        if frame != 'map':
            xf, yf, _yawf, _p2 = self._pose()
        else:
            xf, yf, _yawf, _p2 = self._pose()
        base = o.copy()
        if np.isfinite(xf) and np.isfinite(yf):
            c, s_ = np.cos(-math.radians(_yawf)), np.sin(-math.radians(_yawf))
            px = base[:, 0] - xf
            py = base[:, 1] - yf
            base[:, 0] = c * px - s_ * py
            base[:, 1] = s_ * px + c * py
        rxy = np.hypot(base[:, 0], base[:, 1])
        near = o[rxy <= self.r]
        # 高度基准按**云的真实帧**选：传感器帧 ⇒ z + 传感器离地高度；map ⇒ z + 0.138
        zoff = self.h if frame.endswith('livox_frame') or frame == 'base_link' else 0.138
        hh = near[:, 2] + zoff
        inband = (hh >= 0.0) & (hh < 0.18)
        # 车头方向 ±35°、1.2 m 内、0~18 cm 高的点
        ang = np.degrees(np.arctan2(base[rxy <= self.r, 1], base[rxy <= self.r, 0]))
        rr = rxy[rxy <= self.r]
        front = (np.abs(ang) <= 35.0) & (rr <= 1.2) & inband
        # 0~18cm 带内的**距离分档**与**方位扇区**（判"是不是绕着车一圈的自击晕"）
        rb = [(0.0, 0.3), (0.3, 0.5), (0.5, 1.0), (1.0, 2.0)]
        band_r = [int((inband & (rr >= a) & (rr < b)).sum()) for a, b in rb]
        f_ = (np.abs(ang) <= 45.0) & inband
        b_ = (np.abs(ang) >= 135.0) & inband
        l_ = (ang > 45.0) & (ang < 135.0) & inband
        r_ = (ang < -45.0) & (ang > -135.0) & inband
        x, y, yaw, parent = self._pose()
        # 最近的 0~18cm 点距离（"最干净"的方向有多远）
        clear = float(rxy[rxy <= self.r][inband].min()) if inband.any() else float(self.r)
        # ground 云的距离分布（判"地面模型只在近处生效"还是"远处地面也被认出"）
        if self.gnd is not None and len(self.gnd):
            gr = np.hypot(self.gnd[:, 0], self.gnd[:, 1])
            g_b = [int((gr < 1.0).sum()), int(((gr >= 1.0) & (gr < 3.0)).sum()),
                   int((gr >= 3.0).sum())]
        else:
            g_b = [0, 0, 0]
        row = [round(time.time(), 2), round(x, 2), round(y, 2), round(yaw, 1), frame,
               len(o), len(near),
               int(((hh >= 0.0) & (hh < 0.05)).sum()),
               int(((hh >= 0.05) & (hh < 0.18)).sum()),
               int(((hh >= 0.18) & (hh < 0.35)).sum()),
               int(front.sum()), (len(self.gnd) if self.gnd is not None else 0),
               round(clear, 2),
               band_r[0], band_r[1], band_r[2], band_r[3],
               int(f_.sum()), int(b_.sum()), int(l_.sum()), int(r_.sum()),
               g_b[0], g_b[1], g_b[2]]
        self.w.writerow(row); self.f.flush()
        self.get_logger().info(
            '[云帧=%s | 位姿帧=%s] pos=(%.2f,%.2f,%.0f°) obs=%d near=%.1fm'
            ' | 离地 0-5cm=%d 5-18cm=%d 18-35cm=%d'
            ' | 车头<1.2m=%d | 最近=%.2fm'
            '\n      0~18cm 距离档: <0.3m=%d 0.3-0.5=%d 0.5-1=%d 1-2=%d'
            '  扇区: 前=%d 后=%d 左=%d 右=%d'
            '\n      ground 距离档: <1m=%d 1-3m=%d >3m=%d  （当前 ground 总 %d / obstacle 总 %d = %.0f%% 是地面）'
            % (frame, parent, x, y, yaw, len(o), self.r, row[7], row[8], row[9], row[10], clear,
               band_r[0], band_r[1], band_r[2], band_r[3],
               row[17], row[18], row[19], row[20], g_b[0], g_b[1], g_b[2],
               (len(self.gnd) if self.gnd is not None else 0), len(o),
               100.0 * (len(self.gnd) if self.gnd is not None else 0)
               / max(1, (len(self.gnd) if self.gnd is not None else 0) + len(o))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--h', type=float, default=0.188)
    ap.add_argument('--r', type=float, default=3.0)
    ap.add_argument('--rate', type=float, default=2.0)
    a = ap.parse_args()
    out = os.path.expanduser('~/.ros/perception_watch_%s.csv'
                             % time.strftime('%H%M%S'))
    rclpy.init()
    n = Watch(a.h, a.r, a.rate, out)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.f.close(); n.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        print('CSV 已写: %s' % out)


if __name__ == '__main__':
    main()
