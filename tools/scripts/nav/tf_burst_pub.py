#!/usr/bin/env python3
"""以固定频率发布一条动态 TF（模拟 fake_vel_transform 那类'用小陀螺边'）。

用途：在链路上复现"两条动态边各自打戳"的情形，验证
     链上最新可用时刻 = 两条边中**较旧**的那个。
"""
import argparse

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class Pub(Node):
    def __init__(self, parent, child, rate, ahead):
        super().__init__("tf_burst_pub")
        self.br = TransformBroadcaster(self)
        self.parent, self.child = parent, child
        self.ahead = ahead
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        t = TransformStamped()
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.header.stamp = self.get_clock().now().to_msg()
        if self.ahead:
            sec, nsec = t.header.stamp.sec, t.header.stamp.nanosec
            total = sec * 1_000_000_000 + nsec + int(self.ahead * 1e9)
            t.header.stamp.sec = total // 1_000_000_000
            t.header.stamp.nanosec = total % 1_000_000_000
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--parent", default="base_link")
    ap.add_argument("--child", default="base_link_fake")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--ahead", type=float, default=0.0)
    args = ap.parse_args()
    rclpy.init()
    node = Pub(args.parent, args.child, args.rate, args.ahead)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
