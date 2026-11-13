#!/usr/bin/env python3
"""把 xwd 原始截图转成 PNG（环境里没有 ImageMagick，PIL 不认 XWD）"""
import sys, struct, numpy as np
from PIL import Image

def convert(src, dst):
    d = open(src,'rb').read()
    (hdr_size, ver, fmt, depth, w, h, xoff, bo, bunit, bbit, bpad, bpp, bpl,
     vclass, rmask, gmask, bmask, bits_rgb, cmap_entries, ncolors, nw, nh, wx, wy, bw) = \
        struct.unpack('>25I', d[:100])
    if fmt != 2:
        raise SystemExit(f'只支持 ZPixmap(2)，实际 {fmt}')
    off = hdr_size + ncolors*12          # XWD 的 colormap entry 是 12 字节
    n = w*h
    raw = np.frombuffer(d[off:off+bpl*h], dtype=np.uint8).reshape(h, bpl)
    if bpp == 32 or bpl == w*4:      # bpl=w*4 时每像素实际占 4 字节（BGRx）
        px = raw[:, :w*4].copy().view(np.uint32)
    elif bpp == 16:
        px = raw[:, :w*2].copy().view(np.uint16).astype(np.uint32)
    else:  # 24bpp 打包
        px = np.zeros((h,w), np.uint32)
        for i in range(h):
            r = raw[i]
            px[i] = (r[0::3].astype(np.uint32) << 16) | (r[1::3].astype(np.uint32) << 8) | r[2::3]
    def shift(mask):
        m = mask
        s = 0
        while m and not (m & 1):
            m >>= 1; s += 1
        return s, (mask >> s)
    def chan(mask):
        s, m = shift(mask)
        return ((px & mask) >> s).astype(np.float32) * (255.0 / max(m,1))
    r = chan(rmask) if rmask else chan(0xff0000)
    g = chan(gmask) if gmask else chan(0x00ff00)
    b = chan(bmask) if bmask else chan(0x0000ff)
    img = np.stack([r,g,b], -1).clip(0,255).astype(np.uint8)
    Image.fromarray(img).save(dst)
    print(f'{dst}: {w}x{h} ({bpp}bpp, masks {rmask:06x}/{gmask:06x}/{bmask:06x})')

if __name__ == '__main__':
    convert(sys.argv[1], sys.argv[2])
