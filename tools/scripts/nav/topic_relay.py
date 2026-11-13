#!/usr/bin/env python3
"""topic_relay.py —— 最小话题中继（替代 `topic_tools/relay`，本机没装那个包）。

为什么需要：BIT 的 MINCO 控制器把话题名写死在源码里 ——
  · 订阅 `/aft_mapped_to_init`（FAST-LIO 的里程计名），我们的是 `/localization/odometry`
  · 发布 `/cmd_vel_mpc`，底盘吃的是 `/cmd_vel`
`MODE=bit` 时用这个脚本把两条线接上（**不改 BIT 源码，也不改我们的 bringup**）。

用法：
    python3 tools/scripts/nav/topic_relay.py --type odom \
        --src /localization/odometry --dst /aft_mapped_to_init
    python3 tools/scripts/nav/topic_relay.py --type twist \
        --src /cmd_vel_mpc --dst /cmd_vel

支持的类型（够用就行，不追求通用）：
    odom   nav_msgs/msg/Odometry
    twist  geometry_msgs/msg/Twist
    pose   geometry_msgs/msg/PoseStamped
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

_TYPES = {'odom': Odometry, 'twist': Twist, 'pose': PoseStamped}


class Relay(Node):
    def __init__(self, a):
        super().__init__('topic_relay')
        cls = _TYPES[a.type]
        # 传感器/里程计常用 best_effort；发布侧给可靠，两边都能收到
        sub_qos = QoSProfile(depth=20)
        sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sub_qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(cls, a.dst, 20)
        self.sub = self.create_subscription(cls, a.src, self.cb, sub_qos)
        self.n = 0
        self.get_logger().info(f'[relay] {a.type}: {a.src} → {a.dst}')

    def cb(self, msg):
        self.pub.publish(msg)
        self.n += 1
        if self.n % 200 == 0:
            self.get_logger().info(f'[relay] 已转发 {self.n} 条')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True)
    ap.add_argument('--dst', required=True)
    ap.add_argument('--type', choices=list(_TYPES), default='odom')
    # ★ 2026-11-13：用 parse_known_args —— 由 launch 启动时命令行会被追加
    #   `--ros-args -r __node:=... --params-file ...`，严格的 parse_args 会直接报错退出（rc=2）。
    a, _unknown = ap.parse_known_args()
    rclpy.init()
    node = Relay(a)
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
