#!/usr/bin/env python3
"""本机截图（X11，纯 Python，无需 gnome-screenshot / import / scrot）。

用法：
    python3 tools/scripts/shot.py                      # 全屏 → .tmp_cache/shot.png
    python3 tools/scripts/shot.py --out /tmp/a.png
    python3 tools/scripts/shot.py --monitor 1          # 第二块屏
    python3 tools/scripts/shot.py --region 100,100,800,600   # x,y,w,h 局部
"""
import argparse, os
import mss
from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='.tmp_cache/shot.png')
    ap.add_argument('--monitor', type=int, default=1)
    ap.add_argument('--region', default='')
    ap.add_argument('--delay', type=float, default=0.0, help='延迟秒数（给窗口切换留时间）')
    a = ap.parse_args()
    if a.delay > 0:
        import time; time.sleep(a.delay)

    with mss.mss() as sct:
        if a.region:
            x, y, w, h = (int(v) for v in a.region.split(','))
            box = {'left': x, 'top': y, 'width': w, 'height': h}
        elif a.monitor < len(sct.monitors):
            box = sct.monitors[a.monitor]
        else:
            box = sct.monitors[0]
        raw = sct.grab(box)
        img = Image.frombytes('RGB', raw.size, raw.bgra, 'raw', 'BGRX')

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    img.save(a.out)
    print(f'已保存 {a.out}  {img.width}x{img.height}')


if __name__ == '__main__':
    main()
