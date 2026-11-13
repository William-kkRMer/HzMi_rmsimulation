#!/usr/bin/env python3
"""把当前 /plan（nav2 的 Path）和 /tdt/astar_path（A* 简化路径）画在 PGM 上，存成 PNG。

用法（导航跑着的时候）：
    python3 tools/scripts/nav/dump_plan_png.py --out /tmp/plan.png --seconds 6
参数：
    --map   yaml 路径（默认 RMUC2027_v2.yaml）
    --out   输出 png
    --seconds 收几秒话题
颜色：白色=A*简化路径，红色=/plan（MinSnap 或回退），黄点=起点，绿点=终点
"""
import argparse, os, sys
import numpy as np
import yaml
from PIL import Image, ImageDraw

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


def world_to_px(x, y, res, ox, oy, H):
    c = int((x - ox) / res)
    r = H - 1 - int((y - oy) / res)
    return c, r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.join(os.environ.get('HOME', ''), 'HzMi_rmsimulation',
                                                  'src/bringup/rm_nav_bringup/map/RMUC2027_v2.yaml'))
    ap.add_argument('--out', default='/tmp/plan.png')
    ap.add_argument('--seconds', type=float, default=6.0)
    a = ap.parse_args()

    with open(a.map) as f:
        meta = yaml.safe_load(f)
    base = os.path.dirname(a.map)
    img = Image.open(os.path.join(base, meta['image'])).convert('RGB')
    W, H = img.size
    res = meta['resolution']; ox, oy = meta['origin'][0], meta['origin'][1]

    rclpy.init()
    node = Node('plan_dump')
    got = {'plan': None, 'astar': None}
    node.create_subscription(Path, '/plan', lambda m: got.__setitem__('plan', m), 1)
    node.create_subscription(Path, '/tdt/astar_path', lambda m: got.__setitem__('astar', m), 1)
    import time
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()

    draw = ImageDraw.Draw(img)
    def poly(msg, color, width):
        pts = [world_to_px(p.pose.position.x, p.pose.position.y, res, ox, oy, H) for p in msg.poses]
        if len(pts) >= 2:
            draw.line(pts, fill=color, width=width)
        for p in (pts[:1] + pts[-1:]):
            draw.ellipse([p[0]-3, p[1]-3, p[0]+3, p[1]+3], fill=(255, 255, 0) if p is pts[0] else (0, 255, 0))
        return pts

    n_a = n_p = 0
    if got['astar'] is not None:
        n_a = len(poly(got['astar'], (255, 255, 255), 2))
    if got['plan'] is not None:
        n_p = len(poly(got['plan'], (255, 40, 40), 3))

    img.save(a.out)
    print(f"已保存 {a.out}: A*路径 {n_a} 点，/plan {n_p} 点 "
          f"（白=A*，红=plan）" + ("" if n_p and n_a else "  ⚠️ 有一个话题没收到"))


if __name__ == '__main__':
    main()
