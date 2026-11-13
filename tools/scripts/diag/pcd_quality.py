#!/usr/bin/env python3
"""pcd_quality.py —— 3D 建图质量体检（不看图也能判断"糊没糊"）。

对一张累积点云（默认 src/bringup/rm_nav_bringup/PCD/RMUC2027.pcd）量四个硬指标：
  ① 地面厚度：每个 0.5 m 格内"地面带"（z<0.15）的 p5~p95 跨度 —— 好图 ≈0.02~0.05 m；
  ② 地面起伏：每格"最密 5 cm 层"的高度，全场的 p5~p95 —— 好图 ≈ 场地的真实倾斜量；
  ③ 双层比例：同一格内出现**两个相距 >0.10 m** 的 5 cm 峰（重影/漂移的典型指纹）；
  ④ 上方层占比：地面带之上 0.15~0.5 m 的点数 / 地面带点数（幽灵面/矮墙都会进这里）。

用法：
  python3 tools/scripts/diag/pcd_quality.py [pcd] [--cell 0.5] [--zmax 0.15]
判读：
  · 厚度 >0.15 m 或 双层比例 >20% ⇒ 建图**有 z 漂/重影**，先修定位或换带回环的后端；
  · 厚度 <0.05 m 且 双层 <5% ⇒ 图是干净的，问题在别处（稀疏/覆盖不全）。
"""
from __future__ import annotations
import argparse, sys
import numpy as np


def load_pcd(path):
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            ln = f.readline().decode('ascii', 'ignore').strip()
            if not ln or ln.startswith('#'):
                continue
            k, *v = ln.split()
            hdr[k] = v
            if k == 'DATA':
                break
        n = int(hdr['POINTS'][0])
        dt = np.dtype([(f_, '<f4') for f_ in hdr['FIELDS']])
        data = f.read(n * dt.itemsize)
    return np.frombuffer(data, dtype=dt, count=n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd', nargs='?', default='src/bringup/rm_nav_bringup/PCD/RMUC2027.pcd')
    ap.add_argument('--cell', type=float, default=0.5)
    ap.add_argument('--zmax', type=float, default=0.15)
    a = ap.parse_args()
    p = load_pcd(a.pcd)
    x, y, z = p['x'].astype(float), p['y'].astype(float), p['z'].astype(float)
    print('%s: %d 点；x[%.1f,%.1f] y[%.1f,%.1f] z[%.2f,%.2f]'
          % (a.pcd, len(p), x.min(), x.max(), y.min(), y.max(), z.min(), z.max()))

    c = a.cell
    ix = np.floor((x - x.min()) / c).astype(int)
    iy = np.floor((y - y.min()) / c).astype(int)
    nx = ix.max() + 1
    m = (z > -0.8) & (z < a.zmax)
    key = (iy[m] * nx + ix[m]).astype(np.int64)
    zz = z[m]
    o = np.argsort(key); key, zz = key[o], zz[o]
    uniq, start = np.unique(key, return_index=True)

    edges = np.arange(-0.80, a.zmax + 1e-6, 0.05)
    thick, modes, dbl, tot = [], [], 0, 0
    for kk, s in zip(uniq, start):
        seg = zz[s:s + np.count_nonzero(key == kk)]
        if len(seg) < 40:
            continue
        thick.append(np.percentile(seg, 95) - np.percentile(seg, 5))
        h, _ = np.histogram(seg, bins=edges)
        modes.append(edges[int(np.argmax(h))] + 0.025)
        if len(seg) >= 60:
            tot += 1
            on = np.nonzero(h > 0.25 * h.max())[0]
            if len(on) and (np.diff(edges[on]) > 0.10).any():
                dbl += 1
    thick = np.array(thick); modes = np.array(modes)
    print('① 地面厚度 (每 %.2f m 格, p5~p95): 中位 %.3f m  p90 %.3f m  (格子 %d)'
          % (c, np.median(thick), np.percentile(thick, 90), len(thick)))
    print('② 地面起伏 (最密 5cm 层, 全场 p5~p95): %.3f m' % (np.percentile(modes, 95) - np.percentile(modes, 5)))
    print('③ 双层比例 (两个 5cm 峰相隔 >0.10 m): %.1f%% (%d/%d)' % (100 * dbl / max(1, tot), dbl, tot))
    lo = ((z > -0.4) & (z < a.zmax)).sum(); hi = ((z >= a.zmax) & (z < 0.5)).sum()
    print('④ 上方层占比 (%.2f~0.5 m / 地面带): %.1f%% (%d/%d)'
          % (a.zmax, 100 * hi / max(1, lo), hi, lo))
    bad = (np.median(thick) > 0.15) or (100 * dbl / max(1, tot) > 20)
    print('\n结论：%s' % ('❌ 有 z 漂/重影（先修定位或换带回环的后端）' if bad else '✅ 地面基本干净'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
