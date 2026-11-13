#!/usr/bin/env python3
"""mesh_to_pcd.py —— 把场地网格（STL / OBJ）**按面积均匀采样**成 PCD。

为什么不"取顶点"：STL/OBJ 的顶点只分布在曲面折线处（平面往往只有 3 个角点），
直接拿顶点当点云 ⇒ 平面区域是空的 ⇒ GICP 对齐会飘、法向估计失败。
本工具做的是"在有面的地方按面积放点"，密度才均匀。

为什么支持 OBJ：OBJ 更通用（CloudCompare / MeshLab / trimesh 都能读，还能带材质），
STL→OBJ 再采样是常见做法；但**关键不是中间格式，而是"是否对表面采样"**。

用法：
    python3 tools/scripts/diag/mesh_to_pcd.py RMUC2027.stl -o PCD/RMUC2027_cad.pcd
    python3 tools/scripts/diag/mesh_to_pcd.py field.obj  -o out.pcd --points 4000000 --voxel 0.03
    # 只保留可观测高度带（去掉模型内腔/地面以下）
    python3 tools/scripts/diag/mesh_to_pcd.py field.stl -o out.pcd --zmin -0.6 --zmax 2.5
    # 单位不确定时：--unit auto（默认）按 bbox 判断 mm/m，也可 --unit mm|m
"""
from __future__ import annotations

import argparse
import struct
import sys

import numpy as np


def load_stl(path: str) -> tuple[np.ndarray, bool]:
    """返回 (tri[n,3,3], is_binary)。支持 binary 与 ascii。"""
    with open(path, 'rb') as f:
        head = f.read(84)
        if len(head) < 84:
            raise SystemExit('✗ STL 太短')
        n = struct.unpack('<I', head[80:84])[0]
        expect = 84 + n * 50
        import os
        size = os.path.getsize(path)
        if expect == size:
            data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
            tri = data[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)
            return tri, True
    # ascii 兜底
    verts = []
    with open(path, 'r', errors='ignore') as f:
        for line in f:
            t = line.strip().split()
            if t and t[0] == 'vertex':
                verts.append([float(t[1]), float(t[2]), float(t[3])])
    v = np.asarray(verts, dtype=np.float64)
    if v.size % 9:
        raise SystemExit('✗ ascii STL 顶点数不是 3 的倍数')
    return v.reshape(-1, 3, 3), False


def load_obj(path: str) -> np.ndarray:
    """只取 v + f（f 支持 v/vt/vn 与负索引）。"""
    verts, faces = [], []
    with open(path, 'r', errors='ignore') as f:
        for line in f:
            t = line.split()
            if not t:
                continue
            if t[0] == 'v' and len(t) >= 4:
                verts.append([float(t[1]), float(t[2]), float(t[3])])
            elif t[0] == 'f' and len(t) >= 4:
                idx = []
                for tok in t[1:]:
                    s = tok.split('/')[0]
                    if s:
                        idx.append(int(s))
                if len(idx) >= 3:
                    # 三角化扇形（多边形面）
                    for k in range(1, len(idx) - 1):
                        faces.append([idx[0], idx[k], idx[k + 1]])
    if not verts or not faces:
        raise SystemExit('✗ OBJ 里没有 v/f（只有点没有面？那就别用它采样）')
    v = np.asarray(verts, dtype=np.float64)
    f = np.asarray(faces, dtype=np.int64)
    f = np.where(f < 0, f + len(v), f - 1)          # 负索引 → 0-based
    return v[f]


def sample_surface(tri: np.ndarray, n: int, rng: np.random.Generator) -> np.ndarray:
    """按三角形面积加权、重心坐标均匀采样。"""
    v0, v1, v2 = tri[:, 0], tri[:, 1], tri[:, 2]
    cross = np.cross(v1 - v0, v2 - v0)
    area = 0.5 * np.linalg.norm(cross, axis=1)
    tot = area.sum()
    if tot <= 1e-12:
        raise SystemExit('✗ 网格总面积为 0')
    p = area / tot
    pick = rng.choice(len(tri), size=n, p=p)
    u = rng.random(n)
    w = rng.random(n)
    flip = (u + w) > 1.0
    u[flip] = 1.0 - u[flip]
    w[flip] = 1.0 - w[flip]
    a, b, c = tri[pick, 0], tri[pick, 1], tri[pick, 2]
    return a + u[:, None] * (b - a) + w[:, None] * (c - a)


def drop_downward(tri: np.ndarray, nz_min: float) -> tuple[np.ndarray, int]:
    """丢掉"朝下"的三角形（法向 z 分量 < nz_min）。

    为什么需要：CAD 网格是**双面实体**（楼板有上下两个面、台子有底面）。
    采样后同一个 XY 格子里会同时出现"顶面"和"底面"的点，
    而 2D 栅格化用"局部地面 p05 + 顶面高度"判占用时，
    底面会把地面估计拉低 ⇒ **平坦地面/平台/坡道被整块判成障碍**
    （实测：RMUC2027 的 0.02 点云导出后 90% 的格被判障碍）。
    真实 LiDAR 看不到朝下的面，所以这里按法向过滤是**物理正确**的。

    返回 (保留的三角形, 丢弃个数)。
    """
    v0, v1, v2 = tri[:, 0], tri[:, 1], tri[:, 2]
    n = np.cross(v1 - v0, v2 - v0)
    ln = np.linalg.norm(n, axis=1)
    ln[ln < 1e-12] = 1e-12
    nz = n[:, 2] / ln
    keep = nz >= nz_min
    return tri[keep], int((~keep).sum())


def voxel_downsample(pts: np.ndarray, leaf: float) -> np.ndarray:
    if leaf <= 0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def write_pcd(path: str, pts: np.ndarray) -> None:
    n = len(pts)
    hdr = ('# .PCD v0.7 - Point Cloud Data file format\n'
           'VERSION 0.7\n'
           'FIELDS x y z\n'
           'SIZE 4 4 4\n'
           'TYPE F F F\n'
           'COUNT 1 1 1\n'
           f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
           f'POINTS {n}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(hdr.encode('ascii'))
        f.write(pts.astype('<f4').tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('mesh', help='.stl / .obj')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--points', type=int, default=3_000_000, help='采样点数（默认 300 万）')
    ap.add_argument('--voxel', type=float, default=0.03, help='体素降采样边长 m（0 = 不降采样）')
    ap.add_argument('--unit', default='auto', choices=['auto', 'mm', 'm'])
    ap.add_argument('--drop-downward', action='store_true',
                    help='丢掉朝下的三角形（双面 CAD 网格必用，否则地面/平台会被判成障碍）')
    ap.add_argument('--downward-nz', type=float, default=-0.2,
                    help='法向 z 阈值：nz < 该值视为"朝下"（默认 -0.2）')
    ap.add_argument('--zmin', type=float, default=None)
    ap.add_argument('--zmax', type=float, default=None)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    low = args.mesh.lower()
    if low.endswith('.obj'):
        tri = load_obj(args.mesh)
    elif low.endswith('.stl'):
        tri, _ = load_stl(args.mesh)
    else:
        raise SystemExit('✗ 只支持 .stl / .obj')

    bbox = tri.reshape(-1, 3)
    span = bbox.max(0) - bbox.min(0)
    scale = 1.0
    if args.unit == 'mm' or (args.unit == 'auto' and span.max() > 200.0):
        scale = 0.001
    tri = tri * scale
    print('网格: %d 三角形，bbox=%.2f x %.2f x %.2f m（单位缩放 x%g）'
          % (len(tri), span[0] * scale, span[1] * scale, span[2] * scale, scale))

    if args.drop_downward:
        tri, dropped = drop_downward(tri, args.downward_nz)
        print(f'[mesh->pcd] 丢弃朝下三角形 {dropped} / {dropped + len(tri)}（剩余 {len(tri)}）')
        if len(tri) == 0:
            raise SystemExit('✗ 全部三角形都被判成朝下，检查 --downward-nz 或网格法向是否一致')
    rng = np.random.default_rng(args.seed)
    pts = sample_surface(tri, args.points, rng)
    print('表面采样: %d 点' % len(pts))
    if args.zmin is not None or args.zmax is not None:
        z = pts[:, 2]
        keep = np.ones(len(pts), bool)
        if args.zmin is not None:
            keep &= z >= args.zmin
        if args.zmax is not None:
            keep &= z <= args.zmax
        pts = pts[keep]
        print('高度带过滤后: %d 点' % len(pts))
    pts = voxel_downsample(pts, args.voxel)
    print('体素 %.3f m 后: %d 点' % (args.voxel, len(pts)))
    write_pcd(args.out, pts)
    print('写出 %s（binary PCD）' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
