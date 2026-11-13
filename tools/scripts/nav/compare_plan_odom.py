#!/usr/bin/env python3
"""把 /plan 与实际位姿轨迹画在同一张图上，用于判断"车走的和规划的为什么不一样"。

用法：
    python3 tools/scripts/nav/compare_plan_odom.py --seconds 30 --out /tmp/cmp.png
"""
import argparse, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path, Odometry
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class Cmp(Node):
    def __init__(self, odom_topic, secs):
        super().__init__("plan_odom_compare")
        self.plan = None
        self.track = []
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Path, "/plan", self.on_plan, qos)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        self.t0 = self.get_clock().now()
        self.secs = secs

    def on_plan(self, msg):
        self.plan = msg

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.track.append((p.x, p.y))
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 > self.secs:
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--odom", default="/localization/odometry")
    ap.add_argument("--out", default="/tmp/plan_vs_odom.png")
    a = ap.parse_args()

    rclpy.init()
    n = Cmp(a.odom, a.seconds)
    end = n.get_clock().now().nanoseconds * 1e-9 + a.seconds
    try:
        while rclpy.ok() and n.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(n, timeout_sec=0.1)
    except (KeyboardInterrupt, SystemExit):
        pass

    fig, ax = plt.subplots(figsize=(9, 6))
    if n.plan is not None:
        px = [q.pose.position.x for q in n.plan.poses]
        py = [q.pose.position.y for q in n.plan.poses]
        ax.plot(px, py, "-", lw=2, color="tab:green", label=f"/plan ({len(px)} pts)")
        ax.plot(px[0], py[0], "o", color="tab:green", ms=6)
        ax.plot(px[-1], py[-1], "s", color="tab:green", ms=6)
    else:
        print("!! 没有收到 /plan")
    if n.track:
        tx = [p[0] for p in n.track]
        ty = [p[1] for p in n.track]
        ax.plot(tx, ty, "-", lw=2, color="tab:red", label=f"odom ({len(tx)} pts)")
        ax.plot(tx[0], ty[0], "o", color="tab:red", ms=6)
        ax.plot(tx[-1], ty[-1], "s", color="tab:red", ms=6)
        # 统计：实际轨迹到 plan 的平均/最大偏差
        if n.plan is not None:
            d = []
            for x, y in zip(tx, ty):
                d.append(min(math.hypot(x - a2, y - b2) for a2, b2 in zip(px, py)))
            if d:
                print(f"实际轨迹到 plan 的偏差: 平均 {sum(d)/len(d):.3f} m, 最大 {max(d):.3f} m")
    else:
        print("!! 没有收到位姿（检查 --odom 话题）")
    ax.set_aspect("equal"); ax.grid(True, alpha=.3); ax.legend()
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("green=/plan  red=actual odometry")
    fig.tight_layout(); fig.savefig(a.out, dpi=130)
    print("saved:", a.out)


if __name__ == "__main__":
    main()
