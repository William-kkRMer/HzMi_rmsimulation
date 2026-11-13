#!/usr/bin/env python3
"""odom_error.py —— 直接量 LIO 漂不漂：把 `/odom`（LIO）与 `/odom_ground_truth`（仿真真值）对差。

为什么这么量：仿真里 mecanum 插件同时发 `/odom_ground_truth`（Gazebo 真值），
所以"LIO 有没有漂"不需要猜——逐帧对差就行：
  · z 误差单调增长（尤其上坡之后）⇒ 典型 LIO 垂向漂移；
  · 一上坡就出现**跳变**（不是缓增）⇒ 事件性的（重定位/丢帧/时间戳）；
  · 误差与速度/CPU 相关 ⇒ 大概率是算力（丢 IMU/点云帧）。

用法：ros2 launch 跑起来后，另开终端
  python3 tools/scripts/diag/odom_error.py            # 1 Hz 打印
  python3 tools/scripts/diag/odom_error.py --rate 5
Ctrl-C 结束，会打印这段的最大误差与"最后 10 s 的漂移速率"。
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class OdomErr(Node):
    def __init__(self, rate):
        super().__init__('odom_error')
        self.lio = None
        self.gt = None
        self.hist = []          # (t, ex, ey, ez) 绝对差（含两帧原点之间的常量偏置）
        self.base = None        # 首帧 (lio, gt) —— 之后只看**位移差**（消除常量偏置）
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/odom', self.cb_lio, qos)
        self.create_subscription(Odometry, '/odom_ground_truth', self.cb_gt, qos)
        self.create_timer(1.0 / rate, self.tick)

    def cb_lio(self, m): self.lio = m
    def cb_gt(self, m): self.gt = m

    def tick(self):
        if self.lio is None or self.gt is None:
            self.get_logger().info('等 /odom 与 /odom_ground_truth ...'); return
        p = self.lio.pose.pose.position
        q = self.gt.pose.pose.position
        if self.base is None:
            # `/odom` 的 (0,0,0) 是 LIO 初始化点，`/odom_ground_truth` 是世界坐标
            # ⇒ 两者天生差一个**常量**偏置。只有"位移差"才是有意义的误差。
            self.base = ((p.x, p.y, p.z), (q.x, q.y, q.z))
        (lx, ly, lz), (gx, gy, gz) = self.base
        ex, ey, ez = (p.x - lx) - (q.x - gx), (p.y - ly) - (q.y - gy), (p.z - lz) - (q.z - gz)
        n = math.sqrt(ex * ex + ey * ey + ez * ez)
        t = time.time()
        self.hist.append((t, ex, ey, ez))
        flag = ''
        if len(self.hist) >= 2:
            pv = self.hist[-2]
            if math.dist((pv[1], pv[2], pv[3]), (ex, ey, ez)) > 0.10:
                flag = '  ← 跳变!'
        self.get_logger().info(
            '位移差 err=(%+.3f,%+.3f,%+.3f) |e|=%.3f m | GT=(%6.2f,%6.2f,%6.2f)%s'
            % (ex, ey, ez, n, q.x, q.y, q.z, flag))

    def summary(self):
        if len(self.hist) < 2:
            return
        e = [h[0] for h in self.hist]
        last = [h for h in self.hist if h[0] > e[-1] - 10.0]
        dz = self.hist[-1][3] - last[0][3]
        dxy = math.hypot(self.hist[-1][1] - last[0][1], self.hist[-1][2] - last[0][2])
        jumps = sum(1 for a, b in zip(self.hist, self.hist[1:])
                    if math.dist(a[1:], b[1:]) > 0.10)
        self.get_logger().info(
            '【汇总】样本 %d；最大 |ez|=%.3f m、最大 |e|=%.3f m；跳变(>10cm/帧) %d 次；'
            '最后 10 s: dz=%+.3f m dxy=%.3f m'
            % (len(self.hist), max(abs(h[3]) for h in self.hist),
               max(math.sqrt(h[1]**2 + h[2]**2 + h[3]**2) for h in self.hist),
               jumps, dz, dxy))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=1.0)
    a = ap.parse_args()
    rclpy.init()
    n = OdomErr(a.rate)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        n.summary()
    finally:
        try:
            n.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
