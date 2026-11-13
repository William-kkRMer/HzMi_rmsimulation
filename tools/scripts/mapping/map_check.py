#!/usr/bin/env python3
"""map_check.py —— 改完图先跑这个：3 秒告诉你这次改动有没有把地图弄坏。

检查三件事（都会打印结论，不用自己算）：
  1. 基本信息：尺寸 / 分辨率 / origin（origin 必须和仿真世界对齐）
  2. 关键点：出生点 (0,0) 等位置是否**可走**（车被埋进墙里最常见）
  3. 与仿真基准图的对齐度：把本图和 `<ref>` 都重采样到同一栅格，
     统计"基准有墙、本图 0.15 m 内没有"的比例（漏判率）。
     **漏判率 > 30% 基本就是 origin 又偏了**（实测偏 6 m 时是 84%）。

用法：
    python3 tools/scripts/mapping/map_check.py .tmp_diag/gridmap/cad_v3.pgm
    python3 tools/scripts/mapping/map_check.py 我的图.pgm --pts 0,0 -3,2 5,-1
    python3 tools/scripts/mapping/map_check.py 我的图.pgm --ref src/bringup/rm_nav_bringup/map/RMUC2027_v2_thick1.pgm
"""
from __future__ import annotations
import argparse
import sys

import numpy as np
from scipy import ndimage


def read_pgm(path: str):
    d = open(path, 'rb').read()
    parts = d.split(b'\n', 3)
    w, h = int(parts[1].split()[0]), int(parts[1].split()[1])
    return np.frombuffer(parts[3], dtype=np.uint8, count=w * h).reshape(h, w)[::-1], w, h


def read_yaml_origin(pgm_path: str):
    import os
    import yaml
    y = pgm_path.rsplit('.', 1)[0] + '.yaml'
    if os.path.exists(y):
        d = yaml.safe_load(open(y))
        return d.get('origin', [0, 0, 0])[:2], d.get('resolution', 0.05), y
    return [0.0, 0.0], 0.05, None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('pgm')
    ap.add_argument('--pts', nargs='*', default=['0,0'], help='要检查的世界坐标点，如 0,0 -3,2')
    ap.add_argument('--ref', default='src/bringup/rm_nav_bringup/map/RMUC2027_v2_thick1.pgm')
    ap.add_argument('--ref-origin', nargs=2, type=float, default=[-19.9, -8.89])
    a = ap.parse_args()

    img, W, H = read_pgm(a.pgm)
    o, res, ypath = read_yaml_origin(a.pgm)
    print(f'[map_check] {a.pgm}')
    print(f'  尺寸 {W}x{H} @ {res} m | origin=({o[0]:.2f},{o[1]:.2f}) | yaml={ypath or "缺失!"}')
    if ypath is None:
        print('  ⚠ 没有同名 yaml ⇒ map_server 无法加载（origin 也丢失）')
    occ = (img == 0)
    print(f'  障碍 {int(occ.sum())} 格 ({occ.sum()*res*res:.1f} m²) | 可走 {int((img == 254).sum())} 格')

    print('  关键点：')
    for p in a.pts:
        x, y = [float(t) for t in p.split(',')]
        cx, cy = int((x - o[0]) / res), int((y - o[1]) / res)
        if 0 <= cx < W and 0 <= cy < H:
            v = img[cy, cx]
            n = int(0.7 / res)
            win = img[max(0, cy-n):cy+n, max(0, cx-n):cx+n]
            frac = 100 * (win == 0).mean()
            print(f'    ({x:+.2f},{y:+.2f}) = {v} ({"障碍 ❌" if v == 0 else "可走 ✅"}) '
                  f'| ±0.7 m 内障碍 {frac:.0f}%')
        else:
            print(f'    ({x:+.2f},{y:+.2f}) 不在图内 ❌（origin 可能偏了）')

    # 与基准图比对（漏判率）
    try:
        ref, RW, RH = read_pgm(a.ref)
    except Exception:
        print(f'  （跳过基准比对：读不到 {a.ref}）')
        return 0
    yy, xx = np.mgrid[0:RH, 0:RW]
    jx = ((a.ref_origin[0] + xx * res - o[0]) / res).astype(int)
    jy = ((a.ref_origin[1] + yy * res - o[1]) / res).astype(int)
    ok = (jx >= 0) & (jy >= 0) & (jx < W) & (jy < H)
    m = np.full((RH, RW), 255, np.uint8)
    m[ok] = img[jy[ok], jx[ok]]
    ob, nb = (ref == 0), (m == 0)
    miss = (ob & ~ndimage.binary_dilation(nb, iterations=3)).sum() / max(1, ob.sum())
    extra = (nb & ~ndimage.binary_dilation(ob, iterations=3)).sum() / max(1, nb.sum())
    verdict = '✅ 对齐正常' if miss < 0.30 else '❌ 大概率 origin 偏了'
    print(f'  与基准图比对：基准墙漏判 {100*miss:.0f}% | 本图多判 {100*extra:.0f}%  ⇒ {verdict}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
