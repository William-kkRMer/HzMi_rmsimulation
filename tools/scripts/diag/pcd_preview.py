#!/usr/bin/env python3
"""pcd_preview.py —— 把 PCD 渲染成 3 张 PNG（俯视 / 侧视 x-z / 侧视 y-z），不用装 pcl_viewer。

为什么：本机没装 pcl-tools，RViz 只能看**在线话题**（要看文件得先 pcd_to_pointcloud 发出来）。
这个脚本只依赖 numpy + matplotlib/PIL，直接出图，双击就能看。

用法：
  python3 tools/scripts/diag/pcd_preview.py <in.pcd> [--out 目录] [--res 0.05]
输出：
  <out>/top.png     俯视（按 z 上色：低=暗、高=亮）
  <out>/xz.png      x-z 侧视（看地面糊没糊、有没有双层）
  <out>/yz.png      y-z 侧视
"""
from __future__ import annotations
import argparse, os
import numpy as np
from PIL import Image


def load_pcd(path):
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
        dt = np.dtype([(f_, '<f4') for f_ in hdr['FIELDS']])
        buf = f.read(n * dt.itemsize)
    return np.frombuffer(buf, dtype=dt, count=n), hdr.get('DATA', ['?'])[0]


def render(u, v, c, bounds, res, out, base):
    ur, vr = bounds
    W = max(2, int((ur[1] - ur[0]) / res) + 1)
    H = max(2, int((vr[1] - vr[0]) / res) + 1)
    img = np.zeros((H, W), np.float32)
    iu = ((u - ur[0]) / res).astype(int)
    iv = ((v - vr[0]) / res).astype(int)
    m = (iu >= 0) & (iu < W) & (iv >= 0) & (iv < H)
    np.maximum.at(img, (iv[m], iu[m]), c[m])          # 取最高点 ⇒ 有高度层次
    lo, hi = np.percentile(img[img > 0], [5, 99]) if (img > 0).any() else (0, 1)
    norm = np.clip((img - lo) / max(hi - lo, 1e-6), 0, 1)
    rgb = (norm ** 0.6 * 255).astype(np.uint8)
    Image.fromarray(np.flipud(rgb)).save(os.path.join(out, base + '.png'))
    return W, H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('--out', default=None)
    ap.add_argument('--res', type=float, default=0.05)
    a = ap.parse_args()
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.pcd)) or '.', 'preview')
    os.makedirs(out, exist_ok=True)
    p, fmt = load_pcd(a.pcd)
    x, y, z = p['x'].astype(float), p['y'].astype(float), p['z'].astype(float)
    print('%s: %d 点 (%s)  x[%.1f,%.1f] y[%.1f,%.1f] z[%.2f,%.2f]'
          % (a.pcd, len(p), fmt, x.min(), x.max(), y.min(), y.max(), z.min(), z.max()))
    pad = 0.5
    xr = (x.min() - pad, x.max() + pad); yr = (y.min() - pad, y.max() + pad)
    zr = (max(z.min(), -1.0) - pad, min(z.max(), 3.0) + pad)
    w1 = render(x, y, z, (xr, yr), a.res, out, 'top')
    w2 = render(x, z, z, (xr, zr), a.res, out, 'xz')
    w3 = render(y, z, z, (yr, zr), a.res, out, 'yz')
    print('已输出：%s/{top,xz,yz}.png（尺寸 %s / %s / %s）'
          % (out, w1, w2, w3))


if __name__ == '__main__':
    main()
