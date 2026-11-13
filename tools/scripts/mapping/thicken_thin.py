#!/usr/bin/env python3
"""thicken_thin.py —— **只把"薄"的障碍加粗**，粗的地方不动。

为什么需要：从 CAD 点云导出的图，墙往往只有 1~2 格（5~10 cm）厚。
而 T-DT 的平滑器在非严格碰撞模式下允许"切过 ≤2 格"——
**1~2 格厚的墙正好等于容差**，于是轨迹可以合法穿墙。
把墙加粗能解决这个问题；但**整张图统一膨胀**会把通道一起吃掉，
所以这里只膨胀"细"的部分。

判据（都是可调参数）：
    对一个障碍格，算它到最近**自由格**的距离 dt（单位：格）
        dt <= --thin-max  ⇒ 视为"薄结构"（1 格宽的线 dt=1，2 格宽 dt=2）
    只对"薄结构"做 --cells 次 4/8 邻域膨胀，再与原因求并集
    ⇒ 粗结构（≥3 格厚）完全不变

用法：
    python3 tools/scripts/mapping/thicken_thin.py \
        --in cad.pgm --out cad_thick.pgm [--cells 1] [--thin-max 1.5]

    # 输出前先看一眼（写 <out>.png 预览：红=新加粗的格）
    python3 tools/scripts/mapping/thicken_thin.py --in a.pgm --out b.pgm --preview
"""
from __future__ import annotations
import argparse
import os
import sys

import numpy as np
from PIL import Image
from scipy import ndimage


def read_pgm(path: str):
    d = open(path, 'rb').read()
    parts = d.split(b'\n', 3)
    w, h = int(parts[1].split()[0]), int(parts[1].split()[1])
    return np.frombuffer(parts[3], dtype=np.uint8, count=w * h).reshape(h, w)


def write_pgm(path: str, a: np.ndarray) -> None:
    h, w = a.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(a.tobytes())


def copy_yaml(src_pgm: str, dst_pgm: str) -> None:
    """把同名 yaml 复制过去（只改 image 字段），保证 origin/resolution 一致。"""
    src_yaml = src_pgm.rsplit('.', 1)[0] + '.yaml'
    dst_yaml = dst_pgm.rsplit('.', 1)[0] + '.yaml'
    if not os.path.exists(src_yaml):
        print(f'  ⚠ 没有 {src_yaml}，未生成 yaml（map_server 需要它）')
        return
    import yaml
    d = yaml.safe_load(open(src_yaml))
    d['image'] = os.path.basename(dst_pgm)
    # ★ 2026-11-13：**origin 用内联写法** `[x, y, z]`。
    #   yaml.safe_dump 默认写成块状（`origin:` 换行 `- x`），而仓库里几个小工具
    #   （map_brush 的旧解析、某些 shell 脚本）只认内联 ⇒ 会读崩。这里手工拼。
    o = d.get('origin', [0.0, 0.0, 0.0])
    lines = [f'image: {d["image"]}']
    for k in ('mode', 'resolution'):
        if k in d:
            lines.append(f'{k}: {d[k]}')
    lines.append(f'origin: [{float(o[0])}, {float(o[1])}, {float(o[2]) if len(o) > 2 else 0.0}]')
    for k in ('negate', 'occupied_thresh', 'free_thresh'):
        if k in d:
            lines.append(f'{k}: {d[k]}')
    with open(dst_yaml, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'  写出 {dst_yaml}')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='src', required=True)
    ap.add_argument('--out', dest='dst', required=True)
    ap.add_argument('--cells', type=int, default=1, help='加粗几格（默认 1）')
    ap.add_argument('--thin-max', type=float, default=1.5,
                    help='到自由格距离 ≤ 该值的障碍算"薄"（默认 1.5 ⇒ 1 格宽）')
    ap.add_argument('--thick-max', type=float, default=2.5,
                    help='也可把"≤ 该值"的都当薄（例如 2.5 会把 2 格宽也加粗）')
    ap.add_argument('--preview', action='store_true')
    a = ap.parse_args()

    g = read_pgm(a.src)
    occ = (g == 0)
    if not occ.any():
        print('  ✗ 输入里没有障碍格')
        return 1
    # 到最近自由格的距离（只在障碍内部有意义）
    dt = ndimage.distance_transform_edt(occ)
    thin = occ & (dt <= a.thin_max)
    if a.thick_max and a.thick_max > 0:
        # 允许把"指定厚度以内"的结构也一起加粗
        thin = occ & (dt <= a.thick_max)
    add = ndimage.binary_dilation(thin, iterations=a.cells) & ~occ
    out = g.copy()
    out[add] = 0
    write_pgm(a.dst, out)
    copy_yaml(a.src, a.dst)
    print(f'  薄结构(到自由格 ≤{a.thick_max if a.thick_max>0 else a.thin_max} 格) {int(thin.sum())} 格')
    print(f'  加粗 {a.cells} 格 ⇒ 新增障碍 {int(add.sum())} 格')
    print(f'  障碍总数 {int((g==0).sum())} → {int((out==0).sum())} 格')
    print(f'  写出 {a.dst}')
    if a.preview:
        h, w = g.shape
        rgb = np.full((h, w, 3), 255, np.uint8)
        rgb[out == 0] = [0, 0, 0]
        rgb[add] = [220, 30, 30]        # 红 = 这次新加粗的
        p = a.dst.rsplit('.', 1)[0] + '.png'
        Image.fromarray(rgb[::-1]).save(p)
        print(f'  预览 {p}（红=新加粗）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
