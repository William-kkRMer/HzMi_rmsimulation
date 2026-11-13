#!/usr/bin/env python3
"""probe_cell.py —— 查"某个坐标处到底为什么被判成障碍"。

用途：你在栅格图/仿真里看到某块地应该是可走的，但生成的图把它标成障碍时，
       用这个工具把那一点的点云高度分布打出来，判断是
         (a) 判据错（地面估计被拉低 / 高度阈值太严），还是
         (b) 几何真有东西（台阶、陡坡、柱子）。

用法：
    python3 tools/scripts/mapping/probe_cell.py --pcd CAD.pcd --x 1.2 --y -0.4 [--r 0.15]
    python3 tools/scripts/mapping/probe_cell.py --pcd CAD.pcd --x 0 --y 0 --grid out.pgm

输出：
    · 半径 r 内点的 z 分位（p01/p05/p25/p50/p75/p95/max）
    · 该点 0.2 m 粗格内的 p05（= 工具用的"局部地面"）
    · 高度差 dz = zmax - ground、格内竖直跨度 span、邻域坡度
    · 若给了 --grid，还会打印该坐标在生成图里的取值（0=障碍/254=可走）
"""
from __future__ import annotations
import argparse
import re
import sys

import numpy as np


def read_pcd(path: str) -> np.ndarray:
    raw = open(path, 'rb').read()
    i = raw.index(b'DATA binary\n') + len(b'DATA binary\n')
    hdr = raw[:i].decode('latin1')
    n = int(re.search(r'POINTS (\d+)', hdr).group(1))
    return np.frombuffer(raw, dtype=np.float32, count=n * 3, offset=i).reshape(-1, 3)


def read_pgm(path: str):
    d = open(path, 'rb').read()
    parts = d.split(b'\n', 3)
    w, h = int(parts[1].split()[0]), int(parts[1].split()[1])
    img = np.frombuffer(parts[3], dtype=np.uint8, count=w * h).reshape(h, w)[::-1]
    return img, w, h


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--r', type=float, default=0.10, help='统计半径（默认 0.10 m）')
    ap.add_argument('--grid', default='', help='生成的 pgm（可选，用来打印该点的判定值）')
    ap.add_argument('--origin', nargs=2, type=float, default=None,
                    help='pgm 原点（不给则从 .yaml 同名文件读，或按 0,0）')
    ap.add_argument('--res', type=float, default=0.05)
    a = ap.parse_args()

    d = read_pcd(a.pcd)
    m = (np.abs(d[:, 0] - a.x) <= a.r) & (np.abs(d[:, 1] - a.y) <= a.r)
    z = d[m, 2]
    print(f'[{a.x:.2f},{a.y:.2f}] 半径 {a.r:.2f} m 内: {z.size} 点')
    if z.size == 0:
        print('  → 该处**无点**（图上应该判"未知/空"，不可能是障碍）')
    else:
        q = lambda p: np.percentile(z, p)
        print(f'  z 分位: p01={q(1):.2f} p05={q(5):.2f} p25={q(25):.2f} '
              f'p50={q(50):.2f} p75={q(75):.2f} p95={q(95):.2f} max={z.max():.2f}')
        # 工具用的"局部地面"：0.2 m 粗格内 p05
        mc = (np.abs(d[:, 0] - a.x) <= 0.10) & (np.abs(d[:, 1] - a.y) <= 0.10)
        g = np.percentile(d[mc, 2], 5) if mc.any() else np.nan
        dz = z.max() - g
        span = z.max() - z.min()
        print(f'  局部地面(0.2m 粗格 p05) = {g:.2f} m')
        print(f'  dz = zmax - ground = {dz:.2f} m   （工具阈值 0.15 ⇒ {"判障碍" if dz>0.15 else "判可走"}）')
        print(f'  格内竖直跨度 span = {span:.2f} m（大 ⇒ 该格里有竖直面/多层）')
        # 邻域坡度（用 0.25 m 外环的 zmax 差）
        ring = (np.abs(d[:, 0] - a.x) <= 0.30) & (np.abs(d[:, 1] - a.y) <= 0.30) & \
               ~((np.abs(d[:, 0] - a.x) <= 0.10) & (np.abs(d[:, 1] - a.y) <= 0.10))
        if ring.any():
            dz_n = z.max() - np.percentile(d[ring, 2], 5)
            import math
            print(f'  到 0.3 m 邻域的落差 = {dz_n:.2f} m ⇒ 等效坡度 {math.degrees(math.atan(dz_n/0.3)):.0f}°')

    if a.grid:
        img, W, H = read_pgm(a.grid)
        o = a.origin
        if o is None:
            import yaml
            try:
                yml = yaml.safe_load(open(a.grid.rsplit('.', 1)[0] + '.yaml'))
                o = yml['origin'][:2]
            except Exception:
                o = (0.0, 0.0)
        cx = int((a.x - o[0]) / a.res)
        cy = int((a.y - o[1]) / a.res)
        if 0 <= cx < W and 0 <= cy < H:
            v = img[cy, cx]
            print(f'  生成图在该点 = {v} （0=障碍, 254=可走）⇒ {"障碍" if v == 0 else "可走"}')
        else:
            print('  该点不在生成图范围内')
    return 0


if __name__ == '__main__':
    sys.exit(main())
