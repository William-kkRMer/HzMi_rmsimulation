#!/usr/bin/env python3
"""pcd2ply.py —— PCD → PLY 转换（**因为 Ubuntu 的 cloudcompare 包不带 qPCL 插件，读不了 .pcd**）。

MeshLab 能直接读 PCD，但 CloudCompare（apt 版）不行；转成 PLY 后两者都能开。

用法：
  python3 tools/scripts/diag/pcd2ply.py <in.pcd> [out.ply]
支持：DATA ascii / binary，字段至少含 x,y,z；有 intensity 就一起写。
输出：binary_little_endian PLY，字段 x,y,z(float) [,intensity(float)]。
"""
from __future__ import annotations
import sys, struct
import numpy as np


def load(path):
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            ln = f.readline().decode('ascii', 'ignore').strip()
            if not ln or ln.startswith('#'):
                continue
            parts = ln.split()
            hdr[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break
        n = int(hdr['POINTS'][0]); fmt = hdr['DATA'][0].lower()
        fields = hdr['FIELDS']; sizes = [int(s) for s in hdr['SIZE']]; types = hdr['TYPE']
        if fmt == 'ascii':
            raw = np.array([l.split() for l in f.read().decode().strip().split('\n') if l.strip()], float)
            idx = {f_: i for i, f_ in enumerate(fields)}
            cols = [idx['x'], idx['y'], idx['z']] + ([idx['intensity']] if 'intensity' in idx else [])
            return raw[:, cols], [fields[i] for i in cols], 'f8'
        dt = np.dtype([(f_, '<' + {'F': 'f', 'U': 'u', 'I': 'i'}[t] + str(s))
                       for f_, s, t in zip(fields, sizes, types)])
        a = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        names = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in a.dtype.names else [])
        return np.stack([a[k].astype(float) for k in names], 1), names, 'f4'


def main():
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src.rsplit('.', 1)[0] + '.ply'
    pts, names, _ = load(src)
    n = len(pts)
    has_i = 'intensity' in names
    props = ['x', 'y', 'z'] + (['intensity'] if has_i else [])
    hdr = ('ply\nformat binary_little_endian 1.0\n'
           'element vertex %d\n' % n +
           ''.join('property float %s\n' % p for p in props) +
           'end_header\n')
    rec = np.zeros(n, dtype=[(p, '<f4') for p in props])
    for i, p in enumerate(props):
        rec[p] = pts[:, i]
    with open(dst, 'wb') as f:
        f.write(hdr.encode('ascii'))
        f.write(rec.tobytes())
    print('%s → %s  (%d 点, 字段 %s)' % (src, dst, n, ','.join(props)))


if __name__ == '__main__':
    main()
