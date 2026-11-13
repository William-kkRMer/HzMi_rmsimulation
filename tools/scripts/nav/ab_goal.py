#!/usr/bin/env python3
"""一次"发目标 + 按仿真时间记进度"的受控测量。

用法（栈已经跑起来时）：
    python3 tools/scripts/nav/ab_goal.py --gx 8.0 --gy 2.7 --budget 90 --out /tmp/ab.json

输出：到达与否、仿真耗时、实际移动距离、终点误差、平均/最大速度、cmd_vel 抖动量。
所有时间都用 /clock（仿真时间），避免 RTF 波动影响对比。
"""
from __future__ import annotations
import argparse, json, math, sys, time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

class Ab(Node):
    def __init__(self, gx, gy, budget, wall_budget=200.0):
        super().__init__('ab_goal')
        self.gx, self.gy, self.budget = gx, gy, budget
        self.wall_budget = wall_budget
        self.t = None; self.pose = None
        self.cmds = []          # (t_sim, vx, vy, wz)
        self.track = []         # (t_sim, x, y)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_clock(self, m): self.t = m.clock.sec + m.clock.nanosec * 1e-9
    def on_odom(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y)
        if self.t is not None:
            self.track.append((self.t, p.x, p.y))
    def on_cmd(self, m):
        if self.t is not None:
            self.cmds.append((self.t, m.linear.x, m.linear.y, m.angular.z))

    def run(self):
        if not self.ac.wait_for_server(timeout_sec=30.0):
            print('✗ action server 未就绪'); return None
        while self.pose is None or self.t is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        t0 = self.t; p0 = self.pose
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'; g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = self.gx; g.pose.pose.position.y = self.gy
        g.pose.pose.orientation.w = 1.0
        send = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        gh = send.result()
        if gh is None or not gh.accepted:
            print('✗ 目标被拒'); return None
        res = gh.get_result_async()
        deadline = t0 + self.budget
        wall_end = time.time() + self.wall_budget
        status = None
        while rclpy.ok() and self.t is not None and self.t < deadline and time.time() < wall_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if res.done():
                try: status = res.result().status
                except Exception: status = 0
                break
        t1 = self.t; p1 = self.pose
        dist = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
        err = math.hypot(p1[0]-self.gx, p1[1]-self.gy)
        speeds = [math.hypot(c[1], c[2]) for c in self.cmds]
        jit = 0.0
        for a, b in zip(self.cmds, self.cmds[1:]):
            jit = max(jit, abs(b[1]-a[1]) + abs(b[2]-a[2]))
        return dict(reached=(status == 4), status=status,
                    sim_dt=(t1 - t0) if (t1 and t0) else None,
                    moved=dist, final_err=err,
                    mean_speed=(dist/(t1-t0)) if (t1 and t0 and t1 > t0) else None,
                    max_cmd=max(speeds) if speeds else 0.0, cmd_jitter=jit,
                    n_cmd=len(self.cmds), track_n=len(self.track),
                    start=list(p0), end=list(p1))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gx', type=float, default=8.0); ap.add_argument('--gy', type=float, default=2.7)
    ap.add_argument('--budget', type=float, default=90.0, help='仿真时间预算 s')
    ap.add_argument('--wall-budget', type=float, default=200.0, help='墙钟安全上限 s（RTF 低时兜底）')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rclpy.init()
    n = Ab(a.gx, a.gy, a.budget, a.wall_budget)
    try:
        r = n.run()
    finally:
        try:
            n.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    if r is None: return 1
    print(json.dumps(r, indent=2, ensure_ascii=False))
    if a.out: json.dump(r, open(a.out, 'w'), indent=2)
    return 0 if r['reached'] else 2

if __name__ == '__main__':
    sys.exit(main())
