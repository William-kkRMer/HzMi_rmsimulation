#!/usr/bin/env python3
"""量"按 now 查询 TF"的成功率。

背景：nav2 的 costmap 激活判定用的是
    robot_pose.header.stamp = clock_->now()
再 transform()，即**要求缓冲区里存在 >= now 的数据**。
而任何发布者最年轻的一帧也只能是"上一次发布的时刻"
⇒ 若发布戳 == now()，这个查询会**结构性地永远差一个周期**。

本探针复现该查询，统计成功/失败次数与失败时的差额，
用来验证 "给发布戳加前瞻" 是否真的解决问题。

用法：
    ros2 run ... 不行（未安装），直接：
    python3 tools/scripts/nav/tf_lookup_probe.py --seconds 8 --parent odom --child base_link
"""
import argparse
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Probe(Node):
    def __init__(self, parent, child, period):
        super().__init__("tf_lookup_probe")
        self.parent = parent
        self.child = child
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.ok = 0
        self.fail = 0
        self.gaps = []
        self.timer = self.create_timer(period, self.tick)

    def tick(self):
        target = Time.from_msg(self.get_clock().now().to_msg())
        try:
            self.buffer.lookup_transform(
                self.parent, self.child, target, Duration(seconds=0.0))
            self.ok += 1
        except Exception as exc:                      # noqa: BLE001
            self.fail += 1
            text = str(exc)
            # 从 tf2 的报错里抠出 "requested ... latest ..." 的差额
            key_req = "Requested time "
            key_lat = "the latest data is at time "
            if key_req in text and key_lat in text:
                try:
                    req = float(text.split(key_req)[1].split(" ")[0])
                    lat = float(text.split(key_lat)[1].split(",")[0])
                    self.gaps.append(req - lat)
                except (IndexError, ValueError):
                    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--period", type=float, default=0.05)
    ap.add_argument("--parent", default="odom")
    ap.add_argument("--child", default="base_link")
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.parent, args.child, args.period)
    import time as _time
    end = _time.monotonic() + args.seconds
    while rclpy.ok() and _time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    total = node.ok + node.fail
    print(f"[probe] {args.parent} -> {args.child}")
    print(f"[probe] 查询 {total} 次 | 成功 {node.ok} | 失败 {node.fail}"
          f" | 成功率 {100.0 * node.ok / max(total, 1):.1f}%")
    if node.gaps:
        gaps = sorted(node.gaps)
        n = len(gaps)
        print(f"[probe] 失败时 requested-latest 差额："
              f"min {gaps[0]:.4f} / 中位 {gaps[n // 2]:.4f} / max {gaps[-1]:.4f} s")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
