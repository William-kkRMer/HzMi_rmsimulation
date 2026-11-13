#!/usr/bin/env python3
"""把地图里的黑色墙线加粗 N 格，生成新地图（不动原图）。

用法：
    python3 tools/make_thick_map.py --cells 1        # 加粗 1 格（5 cm）
    python3 tools/make_thick_map.py --cells 2        # 加粗 2 格
产物：src/bringup/rm_nav_bringup/map/RMUC2027_v2_thickN.pgm + .yaml，并生成对比预览 PNG
"""
import argparse, os, shutil
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import binary_dilation

MAP_DIR = 'src/bringup/rm_nav_bringup/map'
SRC_YAML = os.path.join(MAP_DIR, 'RMUC2027_v2.yaml')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cells', type=int, default=1, help='加粗格数（1 格 = 5 cm）')
    a = ap.parse_args()

    meta = yaml.safe_load(open(SRC_YAML))
    src_pgm = os.path.join(MAP_DIR, meta['image'])
    m = np.array(Image.open(src_pgm))
    H, W = m.shape
    res = meta['resolution']; ox, oy = meta['origin'][0], meta['origin'][1]

    wall = (m == 0)
    r = a.cells
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disk = (xx * xx + yy * yy) <= r * r + 0.5
    wall2 = binary_dilation(wall, disk) & ~wall          # 新增的墙
    out = m.copy(); out[wall | wall2] = 0

    tag = f'RMUC2027_v2_thick{r}'
    Image.fromarray(out).save(os.path.join(MAP_DIR, tag + '.pgm'))
    meta2 = dict(meta); meta2['image'] = tag + '.pgm'
    with open(os.path.join(MAP_DIR, tag + '.yaml'), 'w') as f:
        yaml.safe_dump(meta2, f, sort_keys=False)

    def cell(x, y):
        c = int((x - ox) / res); rr = H - 1 - int((y - oy) / res)
        return out[rr, c] if (0 <= rr < H and 0 <= c < W) else -1

    print(f'原障碍 {int(wall.sum())} 格 → 加粗 {r} 格后 {int((wall2).sum())} 新增，总 {int((out==0).sum())} 格')
    print(f'占用率: {wall.sum()/(H*W)*100:.2f}% → {(out==0).sum()/(H*W)*100:.2f}%')
    for name, p in (('出生点', (0.0, 0.02)), ('远端目标', (-15.20, -3.09)), ('场地中心', (-6.0, -1.0))):
        print(f'  {name} {p} → {"自由" if cell(*p) != 0 else "★变成障碍★"}')

    # 对比预览：原图灰、新墙黑、新增部分红
    prev = np.stack([m] * 3, axis=-1)
    prev[wall] = (0, 0, 0)
    prev[wall2] = (220, 30, 30)
    Image.fromarray(prev).save('.tmp_cache/' + tag + '_preview.png')
    print(f'预览: .tmp_cache/{tag}_preview.png')


if __name__ == '__main__':
    main()
