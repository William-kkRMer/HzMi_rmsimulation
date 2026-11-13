#!/usr/bin/env python3
"""发导航目标 + 检查该点在场地地图上的离障距离（clearance）。

用法:
  tools/scripts/nav/tdt_goal.py 2.0 0.5            # 只检查
  tools/scripts/nav/tdt_goal.py 2.0 0.5 --send     # 检查并发送
  tools/scripts/nav/tdt_goal.py --list             # 列出开阔点
"""
import argparse
import math
import os
import sys
import time

import numpy as np
from scipy import ndimage

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
# ★ 2026-11-08：默认用 **zlio 先验生成的地图**（与定位同源同帧），
#   旧的 RMUC2027_v2.pgm 是另一份图，和先验对不上。
YAML = os.environ.get('TDT_MAP_YAML',
                      os.path.join(_ROOT, 'src', 'rm_nav_bringup', 'map', 'RMUC2027_v2.yaml'))


def _yaml_params():
    import yaml as _y
    with open(YAML) as f:
        y = _y.safe_load(f)
    return (float(y['resolution']), float(y['origin'][0]), float(y['origin'][1]),
            os.path.join(os.path.dirname(YAML), y['image']))


def load_map():
    global RES, OX, OY
    RES, OX, OY, img_path = _yaml_params()
    with open(img_path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    # ★ 2026-11-08 修正：trinary 地图里只有 254 是空闲；205 是未知、0 是占据。
    #   旧写法 `>= 200` 把未知(205)也当成空闲 ⇒ clearance 偏大、点到"未知区"仍报开阔。
    free = img >= 250
    dt = ndimage.distance_transform_edt(free) * RES
    return free, dt, w, h


def clearance(free, dt, x, y):
    c = int((x - OX) / RES)
    # ★ 2026-11-08 修正：P5 的首行是**图像顶部**（y 最大），原来漏了这个翻转，
    #   导致"地图上下颠倒"：报 (2.0,2.5) clearance 3.02 m，实际那格是占据(0)。
    r = free.shape[0] - 1 - int((y - OY) / RES)
    if not (0 <= r < free.shape[0] and 0 <= c < free.shape[1]):
        return None
    return float(dt[r, c]) if free[r, c] else -1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', nargs='?', type=float)
    ap.add_argument('y', nargs='?', type=float)
    ap.add_argument('--send', action='store_true')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--timeout', type=float, default=120.0)
    a = ap.parse_args()

    free, dt, w, h = load_map()
    if a.list:
        pts = []
        for x in np.arange(-19, 8, 1.0):
            for y in np.arange(-8.5, 6.5, 1.0):
                c = clearance(free, dt, x, y)
                if c and c > 1.0:
                    pts.append((c, round(float(x), 1), round(float(y), 1)))
        pts.sort(reverse=True)
        print('开阔点（clearance > 1 m）：')
        for c, x, y in pts[:15]:
            print(f'  ({x:6.1f}, {y:5.1f})  clearance {c:.2f} m')
        return 0

    if a.x is None or a.y is None:
        ap.error('需要 x y，或 --list')
    c = clearance(free, dt, a.x, a.y)
    if c is None:
        print(f'❌ ({a.x}, {a.y}) 在地图范围外')
        return 1
    if c < 0:
        print(f'❌ ({a.x}, {a.y}) 落在障碍格')
        return 1
    print(f'✅ ({a.x}, {a.y}) clearance {c:.2f} m' + ('（偏窄，规划可能贴墙）' if c < 0.4 else ''))
    if not a.send:
        return 0

    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped
    rclpy.init()
    node = rclpy.create_node('tdt_goal')
    cli = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not cli.wait_for_server(timeout_sec=10.0):
        print('❌ navigate_to_pose 服务不可用')
        return 1
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = 'map'
    g.pose.header.stamp = node.get_clock().now().to_msg()
    g.pose.pose.position.x = a.x
    g.pose.pose.position.y = a.y
    g.pose.pose.orientation.w = 1.0
    print('发送目标…')
    fut = cli.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut)
    handle = fut.result()
    if not handle.accepted:
        print('❌ 目标被拒绝')
        return 1
    res_fut = handle.get_result_async()
    t0 = time.time()
    while rclpy.ok() and not res_fut.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > a.timeout:
            print(f'⏱ {a.timeout:.0f} s 未结束，取消')
            handle.cancel_goal_async()
            break
    if res_fut.done():
        st = res_fut.result().status
        names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        print(f'结果: {names.get(st, st)}  用时 {time.time()-t0:.1f} s')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
