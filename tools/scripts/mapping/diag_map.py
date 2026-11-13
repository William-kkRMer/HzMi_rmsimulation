#!/usr/bin/env python3
"""diag_map.py —— 把"生成的栅格 + 点云地形"画成一张**分区着色图**，用眼睛就能定位问题格。

为什么需要：逐点给坐标排查太慢。这张图按"地形坡度 × 判定结果"分区，
一眼就能看出"哪片是可行驶斜坡却被判成障碍"。

着色规则（默认，可用 --slope-limit / --flat 调）：
    黑  = 坡度 > 30° 且判障碍        ← **真障碍**（墙、台阶沿、柱子）
    红  = 坡度 5~30° 且判障碍        ← **嫌疑格：可行驶斜坡却被判障碍**
    橙  = 坡度 < 5° 且判障碍         ← 平坦却被判障碍（矮结构？平台边？）
    浅绿= 坡度 5~30° 且判可走        ← 正常斜坡
    白  = 坡度 < 5° 且判可走         ← 正常平地
    灰  = 该处没有点云

用法：
    python3 tools/scripts/mapping/diag_map.py \
        --pcd out.pcd --grid cad.pgm --out diag.png
    # 再叠加基准图（仿真在用的那张）把"两边不一致"的地方标出来
    python3 tools/scripts/mapping/diag_map.py --pcd out.pcd --grid cad.pgm \
        --out diag.png --ref src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm
"""
from __future__ import annotations
import argparse
import re
import sys

import numpy as np
from PIL import Image


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
    return np.frombuffer(parts[3], dtype=np.uint8, count=w * h).reshape(h, w)[::-1], w, h


def origin_of(pgm_path: str):
    import yaml
    try:
        return yaml.safe_load(open(pgm_path.rsplit('.', 1)[0] + '.yaml'))['origin'][:2]
    except Exception:
        return [0.0, 0.0]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--grid', required=True, help='生成的 pgm')
    ap.add_argument('--out', required=True)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--origin', nargs=2, type=float, default=None)
    ap.add_argument('--slope-soft', type=float, default=5.0, help='低于此坡度算"平地"')
    ap.add_argument('--slope-limit', type=float, default=30.0, help='高于此坡度算"陡面"')
    ap.add_argument('--ref', default='', help='基准 pgm（可选，只做统计对比）')
    a = ap.parse_args()

    g, W, H = read_pgm(a.grid)
    o = a.origin or origin_of(a.grid)
    occ = (g == 0)

    d = read_pcd(a.pcd)
    ix = ((d[:, 0] - o[0]) / a.res).astype(int)
    iy = ((d[:, 1] - o[1]) / a.res).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < W) & (iy < H)
    ix, iy, z = ix[ok], iy[ok], d[ok, 2]
    zmax = np.full((H, W), -np.inf)
    np.maximum.at(zmax, (iy, ix), z)
    has = np.isfinite(zmax)
    surf = np.where(has, zmax, 0.0)
    grad = np.zeros((H, W))
    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        grad = np.fmax(grad, np.abs(surf - np.roll(np.roll(surf, dy, 0), dx, 1)) / a.res)
    slope = np.degrees(np.arctan(grad))

    flat = slope < a.slope_soft
    soft = (slope >= a.slope_soft) & (slope <= a.slope_limit)
    steep = slope > a.slope_limit

    img = np.full((H, W, 3), 255, np.uint8)
    img[~has] = [140, 140, 140]                       # 灰：无点
    img[has & ~occ] = [255, 255, 255]                 # 白：平地可走
    img[has & ~occ & soft] = [180, 230, 180]          # 浅绿：斜坡可走
    img[has & occ & flat] = [255, 165, 0]             # 橙：平坦却判障碍
    img[has & occ & soft] = [230, 30, 30]             # 红：斜坡却判障碍 ★
    img[has & occ & steep] = [0, 0, 0]                # 黑：陡面判障碍（正常）

    Image.fromarray(img[::-1]).save(a.out)
    n_orange = int((has & occ & flat).sum())
    n_red = int((has & occ & soft).sum())
    n_black = int((has & occ & steep).sum())
    print(f'  写出 {a.out}  ({W}x{H})')
    print(f'    黑(陡面障碍) {n_black} 格 | 红(斜坡却判障碍) {n_red} 格 | 橙(平坦却判障碍) {n_orange} 格')
    if a.ref:
        r, RW, RH = read_pgm(a.ref)
        ro = origin_of(a.ref)
        yy, xx = np.mgrid[0:RH, 0:RW]
        jx = (((ro[0] + xx * 0.05) - o[0]) / a.res).astype(int)
        jy = (((ro[1] + yy * 0.05) - o[1]) / a.res).astype(int)
        ok2 = (jx >= 0) & (jy >= 0) & (jx < W) & (jy < H)
        ref_occ = np.zeros((H, W), bool)
        ref_occ[jy[ok2], jx[ok2]] = (r == 0)[ok2]
        print(f'    对比基准: 我们 {int(occ.sum())} 格 | 基准 {int(ref_occ.sum())} 格 '
              f'| 一致 {int((occ & ref_occ).sum())} | 我们多判 {int((occ & ~ref_occ).sum())} '
              f'| 漏判 {int((~occ & ref_occ).sum())}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
