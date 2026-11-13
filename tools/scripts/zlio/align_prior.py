#!/usr/bin/env python3
"""把 prior PCD 配准到 RMUC2027_v2 的 map 帧：yaw 搜索 + FFT 平移 + IoU 验收。

用法: tools/scripts/zlio/align_prior.py <in.pcd> <out.pcd>
"""
import sys
import numpy as np
from scipy.signal import fftconvolve

PGM = 'src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm'
RES, OX, OY = 0.05, -19.9, -8.89


def load_pcd(p):
    with open(p, 'rb') as f:
        hdr = {}
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if not line:
                continue
            k = line.split()
            hdr[k[0]] = k[1:]
            if k[0] == 'DATA':
                break
        n = int(hdr['POINTS'][0]); fmt = hdr['DATA'][0]; fields = hdr['FIELDS']
        if fmt == 'binary':
            dt = np.dtype([(c, 'f4') for c in fields])
            a = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
            return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float32)
        return np.loadtxt(f, max_rows=n)[:, :3].astype(np.float32)


def main():
    src, dst = sys.argv[1], sys.argv[2]
    pts = load_pcd(src)
    m = (pts[:, 2] > 0.05) & (pts[:, 2] < 1.8)
    xy = pts[m][:, :2].astype(np.float64)

    with open(PGM, 'rb') as f:
        assert f.readline().strip() == b'P5'
        W, H = map(int, f.readline().split()); f.readline()
        img = np.frombuffer(f.read(W * H), dtype=np.uint8).reshape(H, W)
    map_occ = (img < 100)[::-1].astype(np.float32)
    n_map = map_occ.sum()

    def raster(xy2):
        ix = ((xy2[:, 0] - OX) / RES).astype(int)
        iy = ((xy2[:, 1] - OY) / RES).astype(int)
        ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        g = np.zeros((H, W), np.float32)
        g[iy[ok], ix[ok]] = 1.0
        return g, ok.sum()

    best = (0.0, 0, 0.0, 0.0)          # iou, yaw, tx, ty
    for yaw in range(0, 360, 10):
        th = np.deg2rad(yaw)
        R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        xy_r = xy @ R.T
        g, n_ok = raster(xy_r)
        if n_ok < 100:
            continue
        corr = fftconvolve(map_occ, g[::-1, ::-1], mode='full')
        iy, ix = np.unravel_index(np.argmax(corr), corr.shape)
        dy = (iy - (H - 1)) * RES
        dx = (ix - (W - 1)) * RES
        for sx, sy in ((dx, dy), (-dx, -dy)):
            xy_t = xy_r + np.array([sx, sy])
            gt, _ = raster(xy_t)
            inter = float((gt * map_occ).sum())
            union = float(((gt + map_occ) > 0).sum())
            iou = inter / max(union, 1.0)
            if iou > best[0]:
                best = (iou, yaw, sx, sy)
    iou, yaw, tx, ty = best
    print(f'best: yaw={yaw}° t=({tx:.2f},{ty:.2f}) IoU={iou:.3f}')

    # 局部细化平移（±0.3 m, 0.05 步）
    th = np.deg2rad(yaw)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    xy_r = xy @ R.T
    for ddx in np.arange(-0.3, 0.31, 0.05):
        for ddy in np.arange(-0.3, 0.31, 0.05):
            gt, _ = raster(xy_r + np.array([tx + ddx, ty + ddy]))
            inter = float((gt * map_occ).sum()); union = float(((gt + map_occ) > 0).sum())
            v = inter / max(union, 1.0)
            if v > iou:
                iou, tx, ty = v, tx + ddx, ty + ddy
    print(f'refined: t=({tx:.2f},{ty:.2f}) IoU={iou:.3f}')

    out = pts.copy()
    out[:, :2] = (R @ pts[:, :2].T).T + np.array([tx, ty])
    with open(dst, 'wb') as f:
        f.write(b'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
        f.write(f'WIDTH {len(out)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(out)}\nDATA binary\n'.encode())
        rec = np.zeros(len(out), np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('i', 'f4')]))
        rec['x'], rec['y'], rec['z'] = out[:, 0], out[:, 1], out[:, 2]
        f.write(rec.tobytes())
    inmap = (out[:, 0] > OX) & (out[:, 0] < OX + W * RES) & (out[:, 1] > OY) & (out[:, 1] < OY + H * RES)
    print(f'in-map {inmap.sum()}/{len(out)} = {100.0*inmap.sum()/len(out):.1f}%')
    print('written', dst)


if __name__ == '__main__':
    main()
