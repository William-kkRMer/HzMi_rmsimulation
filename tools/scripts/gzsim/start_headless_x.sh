#!/usr/bin/env bash
# =============================================================================
# start_headless_x.sh —— 给 gz 的传感器渲染提供一个**带 GLX 的虚拟 X**（软件渲染 llvmpipe）
#
# 为什么需要：gz-sim 的 Sensors 系统走 Ogre2 → GL3Plus → **GLX**，必须有 X 显示。
#   实测：`DISPLAY` 未设/失效时，即使 `LIBGL_ALWAYS_SOFTWARE=1` 也照报
#   `X Error of failed request: BadValue … Major opcode 152 (GLX)`；
#   而"只有 X、没有 GLX"（例如某些 Xvfb 默认配置）同样会失败。
#   所以这里显式开 `+extension GLX`，并把 Mesa 钉到 llvmpipe（无 GPU 也能出软渲染）。
#
# 用法：
#   tools/scripts/gzsim/start_headless_x.sh            # 起在 :99（已存在就复用）
#   eval "$(tools/scripts/gzsim/start_headless_x.sh --print-env)"   # 只打印 export，供 source
#   tools/scripts/gzsim/start_headless_x.sh --stop
#
# 前置：`sudo apt install -y xvfb`（mesa 的 llvmpipe 一般已随 libgl1-mesa-dri 装好）
# =============================================================================
set -uo pipefail
DISP="${GZ_XVFB_DISPLAY:-:99}"
GEOM="${GZ_XVFB_GEOM:-1280x800x24}"
LOCK="/tmp/.X${DISP#:}-lock"

have_xvfb() { command -v Xvfb >/dev/null 2>&1; }
is_up() { [ -e "$LOCK" ] && DISPLAY="$DISP" timeout 5 xdpyinfo >/dev/null 2>&1; }

case "${1:-}" in
  --stop)
    pkill -f "Xvfb $DISP" 2>/dev/null && echo "已停止 Xvfb $DISP" || echo "没有在跑的 Xvfb $DISP"; exit 0;;
  --print-env)
    echo "export DISPLAY=$DISP"
    echo "export LIBGL_ALWAYS_SOFTWARE=1"
    echo "export GALLIUM_DRIVER=llvmpipe"
    echo "export EGL_PLATFORM=x11"
    exit 0;;
esac

if ! have_xvfb; then
  cat >&2 <<'MSG'
✗ 没有 Xvfb。请先装（一次就够，1~2 MB）：
    sudo apt install -y xvfb
MSG
  exit 1
fi

if is_up; then
  echo "✓ Xvfb $DISP 已在运行（复用）"
else
  # -noreset：客户端断开也别退出（gz 会 fork 多次）；+extension GLX：Ogre2 要的就是它
  Xvfb "$DISP" -screen 0 "$GEOM" +extension GLX +render -noreset >/tmp/gz_xvfb.log 2>&1 &
  for _ in $(seq 1 30); do sleep 0.2; [ -e "$LOCK" ] && break; done
  sleep 0.3
  if is_up; then echo "✓ Xvfb $DISP 起来了（日志 /tmp/gz_xvfb.log）"; else
    echo "✗ Xvfb 没起来，看 /tmp/gz_xvfb.log" >&2; exit 1; fi
fi

export DISPLAY="$DISP" LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe EGL_PLATFORM=x11
if command -v glxinfo >/dev/null 2>&1; then
  echo "--- glxinfo -B ---"; glxinfo -B 2>&1 | grep -E "OpenGL renderer|OpenGL version|direct rendering" | head -3
fi
cat <<MSG

把下面两行加进要跑 gz 的那个 shell（或 eval 本脚本的 --print-env）：
    export DISPLAY=$DISP
    export LIBGL_ALWAYS_SOFTWARE=1
MSG
