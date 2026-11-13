#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_brush.py —— 鼠标 P 图（Tkinter 画板，改 PGM 栅格地图）

为什么要有它：`map_edit.py` 是命令行批处理，但在图上"看哪儿不对就点哪儿"更直观。
本工具打开一个窗口，**鼠标拖动就能涂黑/擦白/涂灰**，支持**撤销**、**保存（自动备份）**。

快速上手：
    python3 tools/scripts/mapping/map_brush.py src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm

    左键拖动  = 用当前画笔涂（默认：黑 = 障碍）
    右键拖动  = **总是擦成白**（自由）—— 临时擦错最常用
    滚轮      = 缩放（以鼠标位置为中心）
    中键拖动  = 平移
    按住 Ctrl = 画笔临时变成"擦白"

    1 = 黑（障碍）    2 = 白（自由）    3 = 灰（未知）
    b = 黑  w = 白  u = 灰          ← 等价于 1/2/3
    r = 切换"矩形模式"（拖一个矩形，松手再填）
    [ / ] = 画笔变小 / 变大          （单位：格）
    g = 显示/隐藏网格（1 m 细线、5 m 粗线）
    Ctrl+Z = 撤销（最多 50 步）      Ctrl+S = 保存（自动备份原文件）
    Ctrl+R = 重新载入（丢弃未保存改动）  q / Esc = 退出（有改动会提示保存）
    鼠标移动时窗口底部显示：**世界坐标 (x,y)** + 像素 + 该点灰度

保存后要重编译（nav2 读 install 里的副本）：
    colcon build --symlink-install --packages-select rm_nav_bringup
"""
import argparse
import shutil
import sys
import time
import tkinter as tk
from pathlib import Path

import numpy as np
from PIL import Image, ImageTk

OCC, FREE, UNK = 0, 254, 205
COLOR = {OCC: (0, 0, 0), FREE: (255, 255, 255), UNK: (128, 128, 128)}


def read_yaml(pgm: Path) -> dict:
    """读同名 yaml 的 resolution / origin。

    ★ 2026-11-13：原来只做逐行字符串解析，只认 `origin: [x, y, z]` 这一种写法；
      而 `yaml.safe_dump` 写出来的是**块状**（`origin:` 换行后 `- x`），
      于是 `float('')` 直接崩（实测踩到）。现在优先用 PyYAML，解析失败再回退到逐行。
    """
    y = pgm.with_suffix('.yaml')
    out = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.0]}
    if not y.is_file():
        return out
    text = y.read_text(encoding='utf-8', errors='replace')
    try:
        import yaml as _yaml
        d = _yaml.safe_load(text) or {}
        if 'resolution' in d:
            out['resolution'] = float(d['resolution'])
        if 'origin' in d:
            o = d['origin']
            out['origin'] = [float(v) for v in (o if isinstance(o, (list, tuple)) else [o])]
            while len(out['origin']) < 3:
                out['origin'].append(0.0)
        return out
    except Exception:
        pass
    for line in text.splitlines():
        s = line.strip()
        if s.startswith('resolution:'):
            try:
                out['resolution'] = float(s.split(':', 1)[1].split('#')[0])
            except ValueError:
                pass
        elif s.startswith('origin:'):
            tail = s.split(':', 1)[1].strip()
            if tail.startswith('[') and tail.endswith(']'):
                try:
                    out['origin'] = [float(v) for v in tail.strip('[]').split(',')]
                except ValueError:
                    pass
    return out


class MapBrush:
    def __init__(self, root: tk.Tk, pgm: Path, scale: float):
        self.root = root
        self.pgm = pgm
        self.meta = read_yaml(pgm)
        self.res = self.meta['resolution']
        self.ox, self.oy = self.meta['origin'][0], self.meta['origin'][1]

        im = Image.open(pgm).convert('L')
        self.arr = np.array(im).copy()            # 行 0 在**上**（图像坐标）
        self.h, self.w = self.arr.shape
        self.orig = self.arr.copy()

        self.zoom = scale
        self.off = [0.0, 0.0]                     # 画布偏移（像素）
        self.brush_color = OCC
        self.brush_r = 1                          # 半径（格）
        self.rect_mode = False
        self.show_grid = True
        self.undo: list[np.ndarray] = []
        self.dirty = False
        self.dragging = False
        self.panning = False
        self.rect_start = None

        self._build_ui()
        self.render()

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        self.root.title(f'map_brush — {self.pgm.name}  ({self.w}x{self.h} @ {self.res} m)')
        self.canvas = tk.Canvas(self.root, bg='#2b2b2b', highlightthickness=0,
                                width=min(1600, int(self.w * self.zoom)),
                                height=min(900, int(self.h * self.zoom)))
        self.canvas.pack(fill=tk.BOTH, expand=True)

        bar = tk.Frame(self.root)
        bar.pack(fill=tk.X)
        self.status = tk.Label(bar, anchor='w', font=('monospace', 10))
        self.status.pack(side=tk.LEFT, padx=6)
        self.tool_lbl = tk.Label(bar, anchor='e', font=('monospace', 10))
        self.tool_lbl.pack(side=tk.RIGHT, padx=6)
        self._update_tool_label()

        c = self.canvas
        c.bind('<Button-1>', self._left_down)
        c.bind('<B1-Motion>', self._paint_move)
        c.bind('<ButtonRelease-1>', self._left_up)
        c.bind('<Button-3>', self._right_down)
        c.bind('<B3-Motion>', self._right_move)
        c.bind('<ButtonRelease-3>', lambda e: setattr(self, 'dragging', False))
        c.bind('<Button-2>', self._pan_down)
        c.bind('<B2-Motion>', self._pan_move)
        c.bind('<ButtonRelease-2>', lambda e: setattr(self, 'panning', False))
        c.bind('<Motion>', self._hover)
        c.bind('<MouseWheel>', self._wheel)
        c.bind('<Button-4>', lambda e: self._zoom_at(e.x, e.y, 1.15))
        c.bind('<Button-5>', lambda e: self._zoom_at(e.x, e.y, 1 / 1.15))
        self.root.bind('<Key>', self._key)
        c.focus_set()

    def _update_tool_label(self):
        name = {OCC: '黑=障碍', FREE: '白=自由', UNK: '灰=未知'}[self.brush_color]
        mode = '矩形' if self.rect_mode else '画笔'
        self.tool_lbl.config(
            text=f'{mode} {name} 半径{self.brush_r}  '
                 f'网格{"开" if self.show_grid else "关"}  '
                 f'缩放{self.zoom:.2f}x  {"● 未保存" if self.dirty else ""}')

    # ---------------------------------------------------------------- 坐标
    def canvas_to_img(self, cx, cy):
        """画布坐标 → 图像数组 (row, col)。"""
        col = (cx - self.off[0]) / self.zoom
        row = (cy - self.off[1]) / self.zoom
        return int(round(row)), int(round(col))

    def img_to_world(self, row, col):
        x = self.ox + col * self.res
        y = self.oy + (self.h - 1 - row) * self.res
        return x, y

    # ---------------------------------------------------------------- 渲染
    def render(self):
        rgb = np.zeros((self.h, self.w, 3), np.uint8)
        for v, c in COLOR.items():
            rgb[self.arr == v] = c
        # 其它灰度（重采样过的图）：按阈值就近归类
        other = ~np.isin(self.arr, list(COLOR))
        rgb[other & (self.arr < 100)] = COLOR[OCC]
        rgb[other & (self.arr >= 100)] = COLOR[FREE]

        if self.show_grid:
            step = max(1, int(round(1.0 / self.res)))         # 1 m 细线
            major = max(step, int(round(5.0 / self.res)))     # 5 m 粗线
            rgb[:, ::step] = (np.array(rgb[:, ::step]) * 0.75 + np.array([255, 80, 80]) * 0.25).astype(np.uint8)
            rgb[:, ::major] = (np.array(rgb[:, ::major]) * 0.4 + np.array([255, 0, 0]) * 0.6).astype(np.uint8)
            rgb[::step, :] = (np.array(rgb[::step, :]) * 0.75 + np.array([80, 80, 255]) * 0.25).astype(np.uint8)
            rgb[::major, :] = (np.array(rgb[::major, :]) * 0.4 + np.array([0, 0, 255]) * 0.6).astype(np.uint8)

        im = Image.fromarray(rgb, 'RGB')
        if abs(self.zoom - 1.0) > 1e-6:
            im = im.resize((max(1, int(self.w * self.zoom)), max(1, int(self.h * self.zoom))),
                           Image.NEAREST)
        self.photo = ImageTk.PhotoImage(im)
        if getattr(self, '_img_id', None) is None:
            self._img_id = self.canvas.create_image(self.off[0], self.off[1], anchor='nw',
                                                    image=self.photo)
        else:
            self.canvas.itemconfig(self._img_id, image=self.photo)
            self.canvas.coords(self._img_id, self.off[0], self.off[1])
        self.canvas.config(scrollregion=(0, 0, self.w * self.zoom, self.h * self.zoom))
        self._update_tool_label()

    # ---------------------------------------------------------------- 编辑
    def _push_undo(self):
        self.undo.append(self.arr.copy())
        if len(self.undo) > 50:
            self.undo.pop(0)

    def _stamp(self, row, col, color):
        r = self.brush_r
        r0, r1 = max(0, row - r), min(self.h, row + r + 1)
        c0, c1 = max(0, col - r), min(self.w, col + r + 1)
        if r0 >= r1 or c0 >= c1:
            return False
        # 圆形笔刷
        if r > 0:
            ys, xs = np.mgrid[r0:r1, c0:c1]
            mask = (ys - row) ** 2 + (xs - col) ** 2 <= r * r
            self.arr[r0:r1, c0:c1][mask] = color
        else:
            self.arr[r0:r1, c0:c1] = color
        self.dirty = True
        return True

    # ---------------------------------------------------------------- 事件
    def _left_down(self, e):
        self.canvas.focus_set()
        row, col = self.canvas_to_img(e.x, e.y)
        ctrl = bool(e.state & 0x0004)
        if self.rect_mode:
            self.rect_start = (row, col)
            return
        self._push_undo()
        self.dragging = True
        self._stamp(row, col, FREE if ctrl else self.brush_color)
        self.render()

    def _paint_move(self, e):
        if not self.dragging:
            return
        row, col = self.canvas_to_img(e.x, e.y)
        ctrl = bool(e.state & 0x0004)
        self._stamp(row, col, FREE if ctrl else self.brush_color)
        self.render()

    def _left_up(self, e):
        if self.rect_mode and self.rect_start is not None:
            row, col = self.canvas_to_img(e.x, e.y)
            r0, r1 = sorted((self.rect_start[0], row))
            c0, c1 = sorted((self.rect_start[1], col))
            self._push_undo()
            self.arr[max(0, r0):max(0, r1 + 1), max(0, c0):max(0, c1 + 1)] = self.brush_color
            self.dirty = True
            self.rect_start = None
            self.render()
        self.dragging = False

    def _right_down(self, e):
        self.canvas.focus_set()
        self._push_undo()
        self.dragging = 'erase'
        row, col = self.canvas_to_img(e.x, e.y)
        self._stamp(row, col, FREE)
        self.render()

    def _right_move(self, e):
        if self.dragging != 'erase':
            return
        row, col = self.canvas_to_img(e.x, e.y)
        self._stamp(row, col, FREE)
        self.render()

    def _pan_down(self, e):
        self.panning = True
        self._pan_from = (e.x - self.off[0], e.y - self.off[1])

    def _pan_move(self, e):
        if not self.panning:
            return
        self.off = [e.x - self._pan_from[0], e.y - self._pan_from[1]]
        self.render()

    def _zoom_at(self, cx, cy, factor):
        row, col = self.canvas_to_img(cx, cy)
        self.zoom = max(0.2, min(12.0, self.zoom * factor))
        self.off = [cx - col * self.zoom, cy - row * self.zoom]
        self.render()

    def _wheel(self, e):
        self._zoom_at(e.x, e.y, 1.15 if e.delta > 0 else 1 / 1.15)

    def _hover(self, e):
        row, col = self.canvas_to_img(e.x, e.y)
        if 0 <= row < self.h and 0 <= col < self.w:
            x, y = self.img_to_world(row, col)
            v = int(self.arr[row, col])
            self.status.config(
                text=f'世界 ({x:+.2f}, {y:+.2f}) m   |   像素 col={col} row={row}   |   灰度 {v}')
        else:
            self.status.config(text=f'窗口外  (col={col}, row={row})')

    def _key(self, e):
        k = e.keysym.lower()
        ctrl = bool(e.state & 0x0004)
        if k == 'z' and ctrl:
            if self.undo:
                self.arr = self.undo.pop()
                self.dirty = True
                self.render()
            return
        if k == 's' and ctrl:
            self.save()
            return
        if k == 'r' and ctrl:
            self.arr = self.orig.copy()
            self.undo.clear()
            self.dirty = False
            self.render()
            return
        if k in ('q', 'escape'):
            self.quit()
            return
        if k in ('1', 'b'):
            self.brush_color = OCC
        elif k in ('2', 'w'):
            self.brush_color = FREE
        elif k in ('3', 'u'):
            self.brush_color = UNK
        elif k == 'r':
            self.rect_mode = not self.rect_mode
        elif k == 'bracketleft':
            self.brush_r = max(0, self.brush_r - 1)
        elif k == 'bracketright':
            self.brush_r = min(40, self.brush_r + 1)
        elif k == 'g':
            self.show_grid = not self.show_grid
        self.render()

    # ---------------------------------------------------------------- 保存/退出
    def save(self):
        bak = self.pgm.with_suffix(self.pgm.suffix + time.strftime('.bak-%Y%m%d-%H%M%S'))
        shutil.copy2(self.pgm, bak)
        Image.fromarray(self.arr.astype(np.uint8), mode='L').save(self.pgm)
        self.orig = self.arr.copy()
        self.dirty = False
        self._update_tool_label()
        self.status.config(text=f'✅ 已保存 {self.pgm.name}（备份 {bak.name}）'
                                f' —— 记得 colcon build --symlink-install --packages-select rm_nav_bringup')
        print(f'✅ 已保存 {self.pgm}；备份 {bak}')

    def quit(self):
        if self.dirty:
            from tkinter import messagebox
            if not messagebox.askyesno('未保存', '有未保存的改动，先保存吗？'):
                self.root.destroy()
                return
            self.save()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description='鼠标 P 图（PGM 栅格地图画板）')
    ap.add_argument('pgm')
    ap.add_argument('--zoom', type=float, default=2.0, help='初始缩放（默认 2）')
    args = ap.parse_args()

    p = Path(args.pgm)
    if not p.is_file():
        print(f'❌ 找不到 {p}', file=sys.stderr)
        return 1
    root = tk.Tk()
    app = MapBrush(root, p, args.zoom)
    root.protocol('WM_DELETE_WINDOW', app.quit)
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
