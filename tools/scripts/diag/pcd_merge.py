#!/usr/bin/env python3
"""pcd_merge.py —— 合并多份 PCD + 可选体素降采样（给 LIO-SAM 的 CornerMap/SurfMap 用）。

为什么需要：LIO-SAM 的 `save_map` 存的是**两份**特征云
  `CornerMap.pcd`（边缘）+ `SurfMap.pcd`（平面）
而 GICP / SC / `pcd_to_nav2_map.py` 都想要**一份**完整点云 ⇒ 需要合并。

用法：
  python3 tools/scripts/diag/pcd_merge.py out.pcd in1.pcd in2.pcd [--voxel 0.05] [--keep-intensity]
输出：二进制 PCD（x,y,z[,intensity]），并打印合并前后点数、体素后点数、密度。
"""
from __future__ import annotations
import argparse
import numpy as np


def load(path):
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            ln = f.readline().decode('ascii', 'ignore').strip()
            if not ln or ln.startswith('#'):
                continue
            k, *v = ln.split(); hdr[k] = v
            if k == 'DATA':
                break
        n = int(hdr['POINTS'][0])
        names = hdr['FIELDS']
        if hdr['DATA'][0].lower() == 'ascii':
            a = np.array([l.split() for l in f.read().decode().strip().split('\n') if l.strip()], float)
            idx = {nm: i for i, nm in enumerate(names)}
            cols = [idx['x'], idx['y'], idx['z']] + ([idx['intensity']] if 'intensity' in idx else [])
            return a[:, cols], [names[i] for i in cols]
        dt = np.dtype([(f_, '<' + {'F': 'f', 'U': 'u', 'I': 'i'}[t] + s)
                       for f_, s, t in zip(names, hdr['SIZE'], hdr['TYPE'])])
        a = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        cols = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in a.dtype.names else [])
        return np.stack([a[c].astype(np.float64) for c in cols], 1), cols


def voxel_down(p, leaf):
    keys = np.floor(p[:, :3] / leaf).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    _, idx = np.unique(flat, return_index=True)   # 每体素保留第一个
    return p[idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('inputs', nargs='+')
    ap.add_argument('--voxel', type=float, default=0.0, help='体素 leaf（0=不降采样）')
    a = ap.parse_args()
    parts, ncols = [], 3
    for f in a.inputs:
        p, cols = load(f)
        ncols = max(ncols, p.shape[1])
        parts.append(p[:, :3] if not a.voxel else p)   # 降采样只用 xyz
        print('  + %-52s %8d 点  (%s)' % (f, len(p), ','.join(cols)))
    allp = np.concatenate(parts, 0)
    n0 = len(allp)
    if a.voxel > 0:
        allp = voxel_down(allp, a.voxel)
    xy = allp[:, :2]
    area = len(np.unique(np.floor(xy / 0.5).astype(np.int64).view([('x', '<i8'), ('y', '<i8')]))) * 0.25
    print('合并：%d → %d 点%s；覆盖 ≈ %.1f m² ⇒ **%.0f 点/m²**'
          % (n0, len(allp), ('（体素 %.3f m）' % a.voxel) if a.voxel > 0 else '',
             area, len(allp) / max(area, 1e-9)))
    rec = np.zeros(len(allp), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
    rec['x'], rec['y'], rec['z'] = allp[:, 0], allp[:, 1], allp[:, 2]
    hdr = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
           'TYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
           'POINTS %d\nDATA binary\n' % (len(rec), len(rec)))
    with open(a.out, 'wb') as f:
        f.write(hdr.encode('ascii')); f.write(rec.tobytes())
    print('写出：%s（%.1f MB）' % (a.out, (84 + len(rec) * 12) / 1e6))


if __name__ == '__main__':
    main()
