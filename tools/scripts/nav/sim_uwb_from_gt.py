#!/usr/bin/env python3
"""sim_uwb_from_gt.py —— 把仿真的**世界位姿真值**转成 UWB 观测，喂给 zlio 后端。

为什么要有它：
    仿真里没有 UWB。但 `rm_gz_planar_move` 插件会发 `/odom_ground_truth`
    （`nav_msgs/Odometry`，**Gazebo 世界真值位姿**），这在物理上就等价于
    "UWB 解算出的绝对位置"——只是没有噪声、也不掉线。

    打开 UWB 的作用：后端会把 UWB 位置当作**全局观测**做联合优化，
    从而压住纯激光里程计的漂移/跳变（也就是你看到的"定位跳一下"）。

用法：
    # 最简：20 Hz、2 cm 噪声
    python3 tools/scripts/nav/sim_uwb_from_gt.py

    # 更真实：10 Hz、5 cm 噪声、10% 丢包、发布在 uwb 系
    python3 tools/scripts/nav/sim_uwb_from_gt.py --rate 10 --noise 0.05 --drop 0.1 --frame uwb

    # 完全干净（用来先确认链路通不通）
    python3 tools/scripts/nav/sim_uwb_from_gt.py --noise 0

参数：
    --rate   Hz，默认 20
    --noise  XY 高斯噪声标准差（m），默认 0.02
    --drop   丢包概率（0~1），默认 0（模拟 UWB 偶尔丢帧）
    --frame  发布坐标系，默认 "map"（配 zero_pose=0 + estimate_zero_pose_online=false
             就是"UWB 直接量出 map 系位置"，最稳）
    --topic  默认 /uwb_pose

配套：`tools/configs/zlio_backend_sim_uwb.yaml` 已把 uwb 打开并与本脚本对齐。
"""
from __future__ import annotations

import argparse
import math
import random
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class SimUwb(Node):
    def __init__(self, a):
        super().__init__('sim_uwb_from_gt')
        self.a = a
        self.last = None
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(Odometry, a.input, self.cb, qos)
        self.pub = self.create_publisher(PoseStamped, a.topic, 10)
        self.timer = self.create_timer(1.0 / max(1.0, a.rate), self.tick)
        self.n_in = 0
        self.n_out = 0
        self.get_logger().info(
            f'[sim_uwb] {a.input} → {a.topic} | {a.rate:.0f} Hz | noise={a.noise:.3f} m '
            f'| drop={a.drop:.0%} | frame={a.frame}')

    def cb(self, msg: Odometry):
        self.last = msg
        self.n_in += 1

    def tick(self):
        if self.last is None:
            return
        if self.a.drop > 0.0 and random.random() < self.a.drop:
            return
        p = self.last.pose.pose.position
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.a.frame
        out.pose.position.x = p.x + (random.gauss(0.0, self.a.noise) if self.a.noise > 0 else 0.0)
        out.pose.position.y = p.y + (random.gauss(0.0, self.a.noise) if self.a.noise > 0 else 0.0)
        out.pose.position.z = p.z
        # 朝向对 UWB 无意义，给单位四元数（后端只用位置）
        out.pose.orientation.w = 1.0
        self.pub.publish(out)
        self.n_out += 1
        if self.n_out % 100 == 0:
            self.get_logger().info(
                f'[sim_uwb] 已发 {self.n_out} 帧（收 {self.n_in}）| 最新 ({p.x:.2f}, {p.y:.2f})')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', default='/odom_ground_truth')
    ap.add_argument('--topic', default='/uwb_pose')
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--noise', type=float, default=0.02)
    ap.add_argument('--drop', type=float, default=0.0)
    ap.add_argument('--frame', default='map')
    # ★ 2026-11-13：用 parse_known_args —— 由 launch 启动时命令行会被追加
    #   `--ros-args -r __node:=... --params-file ...`，严格的 parse_args 会直接报错退出（rc=2）。
    a, _unknown = ap.parse_known_args()
    rclpy.init()
    node = SimUwb(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
