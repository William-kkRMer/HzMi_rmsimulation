#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_edit.py —— 地图 P 图工具（PGM 栅格图编辑器，纯命令行 + PNG 预览）

为什么要有它：
    `slam_toolbox` 出来的 PGM 总有几类瑕疵：反光孤点、墙上被反光打出的洞、
    限高杆/横幅被投影成墙、地图右上角一片"没扫到"的未知区。
    本工具用**一条命令一个动作**的方式把它们修掉，**不改 yaml**，原文件自动备份。

三个约定（很重要）：
  · 灰度：0 = 占用（障碍）、254 = 自由、205 = 未知。与 nav2 map_server `negate: 0` 一致。
  · **世界坐标**：`map` 系 = 出生点相对系；`origin` 与 `resolution` 从同名 yaml 读。
    所有 `--rect/--line` 都可以用 `--world` 给**世界坐标 (x,y)**，否则按**像素 (col,row)**。
  · 图像行 0 在**上**，而世界 y 向上 ⇒ 内部翻转，用户只关心世界坐标。

子命令（`map_edit.py <cmd> -h` 看详细）：
    stats      统计三类像素占比 + 孤立点比例
    preview    左边原图 / 右边结果，输出 PNG（并排对比）
    despeckle  去孤立噪点（8 邻域里占用 < N 的占用格 → free）
    fill       补洞（8 邻域里 free 少于 M 的 free 格 → occupied），补反光打穿的墙
    erase      抹掉一块矩形（像素或世界坐标）→ free
    add        加一块矩形 → occupied（画一段临时墙/条挡住不能走的地方）
    dilate     障碍膨胀 N 格（保守化，规划更远离墙）
    erode      障碍腐蚀 N 格（激进化，窄通道能过）
    normalize  把非 0/205/254 的中间灰度按阈值重归类（地图被重采样过会用到）
    crop       裁掉四周全 unknown 的空白（缩小图，加快规划）

典型用法：
    # 看一眼
    python3 tools/scripts/mapping/map_edit.py stats src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm
    # 去反光 + 补墙洞，出对比图
    python3 tools/scripts/mapping/map_edit.py despeckle <pgm> --min-neighbors 2 --dry-run
    python3 tools/scripts/mapping/map_edit.py fill <pgm> --min-occ 6 --dry-run
    # 抹掉限高杆那一条（世界坐标 x∈[2,6], y≈3.1）
    python3 tools/scripts/mapping/map_edit.py erase <pgm> --world --rect 2.0 3.0 6.0 3.2
    # 出对比图
    python3 tools/scripts/mapping/map_edit.py preview <pgm> --out /tmp/before_after.png
    # 改完必须重编译（nav2 读 install 里的副本）
    colcon build --symlink-install --packages-select rm_nav_bringup
"""
import argparse
import shutil
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

OCC_THR = 100      # < 100 ⇒ 占用
FREE_THR = 250     # > 250 ⇒ 自由
OCC, FREE, UNK = 0, 254, 205


# ---------------------------------------------------------------- 读写/坐标
def load_map(pgm: Path):
    im = Image.open(pgm)
    if im.mode not in ('L', 'P', 'I;16'):
        im = im.convert('L')
    arr = np.array(im.convert('L'))
    if arr.ndim != 2:
        raise SystemExit(f'❌ 不是单通道 PGM: {pgm}')
    meta = read_yaml(pgm)
    return arr, meta


def read_yaml(pgm: Path) -> dict:
    """从同名 .yaml 读 resolution / origin（没有就给默认）。"""
    y = pgm.with_suffix('.yaml')
    out = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.0]}
    if not y.is_file():
        return out
    txt = y.read_text(encoding='utf-8', errors='replace')
    for key in ('resolution', 'origin'):
        for line in txt.splitlines():
            if line.strip().startswith(key + ':'):
                val = line.split(':', 1)[1].split('#')[0].strip()
                if key == 'resolution':
                    out['resolution'] = float(val)
                else:
                    out['origin'] = [float(x) for x in val.strip('[]').split(',')]
    return out


def world_to_pix(meta: dict, x: float, y: float, h: int):
    """世界 (x,y) → 像素 (col,row)。注意行要翻转。"""
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    col = int((x - ox) / res)
    row = int(h - 1 - (y - oy) / res)
    return col, row


def pix_to_world(meta: dict, col: int, row: int, h: int):
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    return ox + col * res, oy + (h - 1 - row) * res


# ---------------------------------------------------------------- 基本运算
def neighbors(mask: np.ndarray) -> np.ndarray:
    k = np.ones((3, 3), np.uint8)
    k[1, 1] = 0
    pad = np.pad(mask.astype(np.uint8), 1)
    h, w = mask.shape
    tot = np.zeros((h, w), np.int16)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            tot += pad[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
    return tot


def counts(arr: np.ndarray):
    occ = arr < OCC_THR
    free = arr > FREE_THR
    unk = ~occ & ~free
    return occ, free, unk


def backup(p: Path) -> Path:
    b = p.with_suffix(p.suffix + time.strftime('.bak-%Y%m%d-%H%M%S'))
    shutil.copy2(p, b)
    return b


def save(p: Path, arr: np.ndarray):
    Image.fromarray(arr.astype(np.uint8), mode='L').save(p)


# ---------------------------------------------------------------- 子命令
def cmd_stats(args):
    arr, meta = load_map(Path(args.pgm))
    occ, free, unk = counts(arr)
    n_occ = neighbors(occ)
    iso = occ & (n_occ <= 1)
    print(f'{args.pgm}  {arr.shape[1]}x{arr.shape[0]}  res={meta["resolution"]}')
    print(f'  占用 {occ.sum():7d} ({100*occ.mean():5.1f}%)')
    print(f'  自由 {free.sum():7d} ({100*free.mean():5.1f}%)')
    print(f'  未知 {unk.sum():7d} ({100*unk.mean():5.1f}%)')
    print(f'  孤立占用点 {iso.sum():5d} ({100*iso.sum()/max(occ.sum(),1):5.1f}% of 占用)'
          f'   ← 反光/噪点指标，>5% 建议 despeckle')
    # 世界范围
    h = arr.shape[0]
    x0, y0 = pix_to_world(meta, 0, h - 1, h)
    x1, y1 = pix_to_world(meta, arr.shape[1] - 1, 0, h)
    print(f'  世界范围 x[{x0:.2f},{x1:.2f}] y[{y0:.2f},{y1:.2f}]')
    return 0


def _apply(pgm: Path, arr: np.ndarray, args, desc: str):
    if args.dry_run:
        print(f'（--dry-run）{desc}')
        return 0
    b = backup(pgm)
    save(pgm, arr)
    print(f'✅ {desc}')
    print(f'   已写回 {pgm}；备份 {b.name}')
    print('   ⚠️ 记得 colcon build --symlink-install --packages-select rm_nav_bringup')
    return 0


def cmd_despeckle(args):
    pgm = Path(args.pgm)
    arr, _ = load_map(pgm)
    occ, _, _ = counts(arr)
    n = neighbors(occ)
    dead = occ & (n < args.min_neighbors)
    print(f'孤立占用点 {dead.sum()} / {occ.sum()} ({100*dead.sum()/max(occ.sum(),1):.1f}%) → free')
    out = arr.copy()
    out[dead] = FREE
    return _apply(pgm, out, args, f'去孤立点 {dead.sum()} 个')


def cmd_fill(args):
    pgm = Path(args.pgm)
    arr, _ = load_map(pgm)
    occ, free, _ = counts(arr)
    n_free = neighbors(free)
    hole = free & (n_free < (8 - args.min_occ)) & (neighbors(occ) >= args.min_occ)
    print(f'待补的墙洞 {hole.sum()} 个 → occupied')
    out = arr.copy()
    out[hole] = OCC
    return _apply(pgm, out, args, f'补洞 {hole.sum()} 格')


def _rect_slice(meta, h, w, x0, y0, x1, y1, world):
    if world:
        c0, r0 = world_to_pix(meta, min(x0, x1), min(y0, y1), h)
        c1, r1 = world_to_pix(meta, max(x0, x1), max(y0, y1), h)
    else:
        c0, r0 = int(min(x0, x1)), int(min(y0, y1))
        c1, r1 = int(max(x0, x1)), int(max(y0, y1))
    r0, r1 = sorted((r0, r1))
    c0, c1 = sorted((c0, c1))
    return slice(max(0, r0), min(h, r1 + 1)), slice(max(0, c0), min(w, c1 + 1))


def cmd_erase(args):
    pgm = Path(args.pgm)
    arr, meta = load_map(pgm)
    h, w = arr.shape
    sl = _rect_slice(meta, h, w, *args.rect, args.world)
    out = arr.copy()
    before = int((out[sl] < OCC_THR).sum())
    out[sl] = FREE
    print(f'矩形内原占用 {before} 格 → free')
    return _apply(pgm, out, args, f'抹除区域 {args.rect}')


def cmd_add(args):
    pgm = Path(args.pgm)
    arr, meta = load_map(pgm)
    h, w = arr.shape
    sl = _rect_slice(meta, h, w, *args.rect, args.world)
    out = arr.copy()
    out[sl] = OCC
    print(f'矩形内 {out[sl].size} 格 → occupied')
    return _apply(pgm, out, args, f'添加障碍 {args.rect}')


def cmd_dilate(args):
    pgm = Path(args.pgm)
    arr, _ = load_map(pgm)
    occ, _, _ = counts(arr)
    m = occ.copy()
    for _ in range(args.n):
        pad = np.pad(m, 1)
        h, w = m.shape
        grown = m.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                grown |= pad[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
        m = grown
    out = arr.copy()
    out[m & ~occ] = OCC
    print(f'膨胀 {args.n} 格：新增占用 {int((m & ~occ).sum())} 格')
    return _apply(pgm, out, args, f'膨胀 {args.n} 格')


def cmd_erode(args):
    pgm = Path(args.pgm)
    arr, _ = load_map(pgm)
    occ, _, _ = counts(arr)
    m = occ.copy()
    h, w = m.shape
    for _ in range(args.n):
        pad = np.pad(m, 1)
        n = np.zeros_like(m, np.int16)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                n += pad[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
        m = m & (n == 9)      # 3x3 全是占用才保留
    out = arr.copy()
    out[occ & ~m] = FREE
    print(f'腐蚀 {args.n} 格：减少占用 {int((occ & ~m).sum())} 格')
    return _apply(pgm, out, args, f'腐蚀 {args.n} 格')


def cmd_normalize(args):
    pgm = Path(args.pgm)
    arr, _ = load_map(pgm)
    out = arr.copy()
    out[arr < args.thresh] = OCC
    out[arr >= OCC_THR] = FREE
    print(f'阈值 {args.thresh}：占用 {(out < OCC_THR).sum()}，自由 {(out > FREE_THR).sum()}')
    return _apply(pgm, out, args, '灰度归一化')


def cmd_crop(args):
    pgm = Path(args.pgm)
    arr, meta = load_map(pgm)
    occ, free, unk = counts(arr)
    known = ~unk
    if not known.any():
        print('❌ 全图 unknown，没法裁')
        return 1
    rows = np.where(known.any(axis=1))[0]
    cols = np.where(known.any(axis=0))[0]
    pad = max(1, int(round(args.padding / meta['resolution'])))
    r0 = max(0, rows[0] - pad); r1 = min(arr.shape[0] - 1, rows[-1] + pad)
    c0 = max(0, cols[0] - pad); c1 = min(arr.shape[1] - 1, cols[-1] + pad)
    out = arr[r0:r1 + 1, c0:c1 + 1]
    # 新 origin：原左下角 + 裁掉的列/行
    res = meta['resolution']
    newox = meta['origin'][0] + c0 * res
    newoy = meta['origin'][1] + (arr.shape[0] - 1 - r1) * res
    print(f'裁到 {out.shape[1]}x{out.shape[0]}（原 {arr.shape[1]}x{arr.shape[0]}），新 origin=({newox:.2f},{newoy:.2f})')
    if args.dry_run:
        print('（--dry-run，未写）')
        return 0
    b = backup(pgm)
    save(pgm, out)
    y = pgm.with_suffix('.yaml')
    if y.is_file():
        text = y.read_text(encoding='utf-8')
        lines = []
        for line in text.splitlines():
            if line.strip().startswith('origin:'):
                lines.append(f'origin: [{newox:.3f}, {newoy:.3f}, 0.0]')
            else:
                lines.append(line)
        y.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        print(f'   已更新 {y.name} 的 origin')
    print(f'✅ 裁切完成；备份 {b.name}')
    return 0


def cmd_preview(args):
    """输出 PNG 预览；`--grid` 时叠加**世界坐标网格**（找 --rect 坐标全靠它）。"""
    pgm = Path(args.pgm)
    im = Image.open(pgm).convert('L')
    meta = read_yaml(pgm)
    scale = args.scale
    w = max(1, int(im.width * scale)); h = max(1, int(im.height * scale))
    im = im.resize((w, h), Image.NEAREST).convert('RGB')

    if args.grid:
        from PIL import ImageDraw
        dr = ImageDraw.Draw(im)
        res, ox, oy = meta['resolution'], meta['origin'][0], meta['origin'][1]
        # 世界坐标 → 大图像素
        def to_px(x, y):
            return (x - ox) / res * scale, h - 1 - (y - oy) / res * scale
        # 网格：1 m 细线、5 m 粗线
        import math
        x0, y0 = ox, oy
        x1 = ox + im.width / scale * res
        y1 = oy + im.height / scale * res
        for gx in range(int(math.floor(x0)), int(math.ceil(x1)) + 1):
            px, _ = to_px(gx, 0)
            major = (gx % args.grid_major == 0)
            dr.line([(px, 0), (px, h)], fill=(255, 60, 60) if major else (255, 190, 190),
                    width=2 if major else 1)
            if major:
                dr.text((px + 2, 2), f'x={gx}', fill=(200, 0, 0))
        for gy in range(int(math.floor(y0)), int(math.ceil(y1)) + 1):
            _, py = to_px(0, gy)
            major = (gy % args.grid_major == 0)
            dr.line([(0, py), (w, py)], fill=(60, 60, 255) if major else (190, 190, 255),
                    width=2 if major else 1)
            if major:
                dr.text((2, py + 1), f'y={gy}', fill=(0, 0, 200))
        # 标出生点（map 原点）
        px, py = to_px(0, 0)
        r = 6
        dr.ellipse([px - r, py - r, px + r, py + r], outline=(0, 160, 0), width=3)
        dr.text((px + 9, py - 6), 'ORIGIN(spawn)', fill=(0, 140, 0))

    out_path = Path(args.out or (pgm.parent / (pgm.stem + ('_grid.png' if args.grid else '_preview.png'))))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    im.save(out_path)
    print(f'✅ 预览图 {out_path}（{w}x{h}'
          + ('，带世界坐标网格：红线 x、蓝线 y、绿圈=出生点' if args.grid else '') + '）')
    return 0


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='地图 P 图工具（PGM 编辑）',
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def add_common(p, needs_pgm=True):
        if needs_pgm:
            p.add_argument('pgm')
        p.add_argument('--dry-run', action='store_true', help='只统计，不写文件')

    p = sub.add_parser('stats', help='统计三类像素 + 孤立点比例'); add_common(p)
    p.set_defaults(func=cmd_stats)

    p = sub.add_parser('preview', help='输出 PNG 预览（--grid 叠世界坐标网格）'); add_common(p)
    p.add_argument('--out', default=None)
    p.add_argument('--grid', action='store_true', help='叠加世界坐标网格（找 --rect 坐标用）')
    p.add_argument('--grid-major', type=int, default=5, help='每几米画一条粗线并标数字（默认 5）')
    p.add_argument('--scale', type=float, default=2.0, help='放大倍数（默认 2）')
    p.set_defaults(func=cmd_preview)

    p = sub.add_parser('despeckle', help='去孤立噪点'); add_common(p)
    p.add_argument('--min-neighbors', type=int, default=2)
    p.set_defaults(func=cmd_despeckle)

    p = sub.add_parser('fill', help='补墙洞'); add_common(p)
    p.add_argument('--min-occ', type=int, default=6, help='8 邻域里至少几个占用才补（默认 6）')
    p.set_defaults(func=cmd_fill)

    for name, fn, hlp in (('erase', cmd_erase, '抹掉矩形 → free'),
                          ('add', cmd_add, '矩形 → occupied')):
        p = sub.add_parser(name, help=hlp); add_common(p)
        p.add_argument('--rect', nargs=4, type=float, required=True, metavar=('A', 'B', 'C', 'D'),
                       help='矩形 (x0 y0 x1 y1)')
        p.add_argument('--world', action='store_true', help='用世界坐标（否则像素 col/row）')
        p.set_defaults(func=fn)

    for name, fn, hlp in (('dilate', cmd_dilate, '障碍膨胀 N 格'),
                          ('erode', cmd_erode, '障碍腐蚀 N 格')):
        p = sub.add_parser(name, help=hlp); add_common(p)
        p.add_argument('--n', type=int, default=1)
        p.set_defaults(func=fn)

    p = sub.add_parser('normalize', help='灰度归一化'); add_common(p)
    p.add_argument('--thresh', type=int, default=OCC_THR)
    p.set_defaults(func=cmd_normalize)

    p = sub.add_parser('crop', help='裁掉四周空白'); add_common(p)
    p.add_argument('--padding', type=float, default=0.5, help='保留的边距（米）')
    p.set_defaults(func=cmd_crop)

    args = ap.parse_args()
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())
