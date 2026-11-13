#!/usr/bin/env python3
"""T-DT 链路评测：依次发多个目标，统计到点率 / 用时（墙钟 + 仿真）。"""
import argparse
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


DEFAULT_GOALS = [(2.0, 0.5), (1.0, 2.5), (-13.0, -4.5), (2.0, 2.5)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goals', default='', help='可覆盖：x,y;x,y;...')
    ap.add_argument('--timeout', type=float, default=90.0)
    a = ap.parse_args()
    goals = DEFAULT_GOALS
    if a.goals:
        goals = [tuple(map(float, p.split(','))) for p in a.goals.split(';')]

    rclpy.init()
    node = Node('tdt_eval')
    cli = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not cli.wait_for_server(timeout_sec=20.0):
        print('❌ navigate_to_pose 不可用')
        return 1
    sim = {'t': None}
    node.create_subscription(Clock, '/clock',
                             lambda m: sim.__setitem__('t', m.clock.sec + m.clock.nanosec * 1e-9), 10)
    rows = []
    for i, (x, y) in enumerate(goals):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = node.get_clock().now().to_msg()
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        g.pose.pose.orientation.w = 1.0
        # 读一次 sim 时间
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
            if sim['t'] is not None:
                break
        t0s, t0w = sim['t'], time.time()
        fut = cli.send_goal_async(g)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        h = fut.result()
        if not h or not h.accepted:
            rows.append({'goal': [x, y], 'status': 'REJECTED'})
            print(f'{i+1}. ({x},{y}) REJECTED')
            continue
        rf = h.get_result_async()
        while rclpy.ok() and not rf.done() and time.time() - t0w < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not rf.done():
            h.cancel_goal_async()
            rows.append({'goal': [x, y], 'status': 'TIMEOUT', 'wall': time.time() - t0w})
            print(f'{i+1}. ({x},{y}) TIMEOUT {time.time()-t0w:.0f}s')
            continue
        st = rf.result().status
        name = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(st, str(st))
        sim_dt = (sim['t'] - t0s) if (sim['t'] is not None and t0s is not None) else float('nan')
        rows.append({'goal': [x, y], 'status': name, 'wall': time.time() - t0w, 'sim': sim_dt})
        print(f'{i+1}. ({x},{y}) {name}  wall {time.time()-t0w:.1f}s  sim {sim_dt:.1f}s')

    ok = [r for r in rows if r['status'] == 'SUCCEEDED']
    print('\n=== 汇总 ===')
    print(f'到点 {len(ok)}/{len(rows)} = {100.0*len(ok)/max(1,len(rows)):.0f}%')
    if ok:
        print(f"平均用时 wall {np.mean([r['wall'] for r in ok]):.1f}s, sim {np.mean([r['sim'] for r in ok]):.1f}s")
    print(json.dumps(rows, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
