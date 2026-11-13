#!/usr/bin/env bash
# map_restore.sh —— 地图版本查看 / 恢复（"原图"和"自动备份"一目了然）
#
# 用法：
#   tools/scripts/mapping/map_restore.sh list    [map_name]      # 看有哪些版本
#   tools/scripts/mapping/map_restore.sh original [map_name]     # 恢复 ORIGINAL（最原始那版）
#   tools/scripts/mapping/map_restore.sh latest  [map_name]      # 恢复"最新一份 .bak"
#   tools/scripts/mapping/map_restore.sh use <文件路径>           # 用指定的某一版覆盖
#
# map_name 默认 RMUC2027_v2。
# 恢复/覆盖后**必须重编译**：nav2 读的是 install/ 里的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
MAPDIR="${ROOT}/src/bringup/rm_nav_bringup/map"
NAME="${2:-RMUC2027_v2}"
CUR="${MAPDIR}/${NAME}.pgm"

md5_of() { md5sum "$1" 2>/dev/null | awk '{print $1}'; }
stamp_of() { date -r "$1" +'%Y-%m-%d %H:%M:%S' 2>/dev/null; }

show_target() {
  echo "当前地图 : ${CUR}"
  if [[ -f "${CUR}" ]]; then
    echo "           时间 $(stamp_of "${CUR}")   大小 $(stat -c%s "${CUR}")   md5 $(md5_of "${CUR}")"
  else
    echo "           （不存在）"
  fi
}

case "${1:-list}" in
  list)
    show_target
    echo
    echo "== 原始备份（ORIGINAL，不会被工具覆盖）=="
    if compgen -G "${MAPDIR}/ORIGINAL/${NAME}.ORIGINAL-*.pgm" >/dev/null; then
      for f in "${MAPDIR}/ORIGINAL/${NAME}".ORIGINAL-*.pgm; do
        echo "  $(basename "$f")"
        echo "      时间 $(stamp_of "$f")   大小 $(stat -c%s "$f")   md5 $(md5_of "$f")"
      done
    else
      echo "  （没有 ORIGINAL 备份）"
    fi
    echo
    echo "== 自动备份（每次 P 图保存生成）=="
    local_n=0
    if compgen -G "${CUR}.bak-*" >/dev/null; then
      while IFS= read -r f; do
        local_n=$((local_n + 1))
        echo "  $(basename "$f")   时间 $(stamp_of "$f")   md5 $(md5_of "$f")"
      done < <(ls -1t "${CUR}".bak-* 2>/dev/null)
    fi
    [[ ${local_n} -eq 0 ]] && echo "  （还没有 .bak 备份）"
    echo
    echo "⇒ 恢复原图 : tools/scripts/mapping/map_restore.sh original ${NAME}"
    echo "⇒ 恢复上一个版本 : tools/scripts/mapping/map_restore.sh latest ${NAME}"
    ;;

  original)
    src=$(ls -1t "${MAPDIR}/ORIGINAL/${NAME}".ORIGINAL-*.pgm 2>/dev/null | head -1 || true)
    [[ -z "${src}" ]] && { echo "❌ 没有 ORIGINAL 备份：${MAPDIR}/ORIGINAL/${NAME}.ORIGINAL-*.pgm"; exit 1; }
    show_target
    cp -v "${src}" "${CUR}"
    ys="${src%.pgm}.yaml"
    [[ -f "${ys}" ]] && cp -v "${ys}" "${MAPDIR}/${NAME}.yaml"
    echo "✅ 已恢复 ORIGINAL。别忘了：colcon build --symlink-install --packages-select rm_nav_bringup"
    ;;

  latest)
    src=$(ls -1t "${CUR}".bak-* 2>/dev/null | head -1 || true)
    [[ -z "${src}" ]] && { echo "❌ 没有 .bak 备份"; exit 1; }
    show_target
    cp -v "${src}" "${CUR}"
    echo "✅ 已恢复 $(basename "${src}")。别忘了：colcon build --symlink-install --packages-select rm_nav_bringup"
    ;;

  use)
    src="${2:-}"
    [[ -z "${src}" || ! -f "${src}" ]] && { echo "❌ 用法: map_restore.sh use <文件路径>"; exit 1; }
    NAME="$(basename "${src}" .pgm)"; CUR="${MAPDIR}/${NAME}.pgm"
    cp -v "${src}" "${CUR}"
    echo "✅ 已用 $(basename "${src}") 覆盖。别忘了重编译。"
    ;;

  *)
    echo "用法: $0 {list|original|latest|use <文件>} [map_name]"
    exit 1
    ;;
esac
