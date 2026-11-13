2026-11-03 整理：从 map/ 移出的旧资产（未删除，需要可移回）
  RMUC2027_v2.pgm.bak-*        两张 pgm 备份
  RMUC2027.{pgm,yaml,meta.yaml,data,posegraph}   旧版 RMUC2027（非 v2）的整套地图/会话
保留在 map/ 的：
  RMUC2027_v2.{pgm,yaml,meta.yaml,data,posegraph}  ← 当前在用（launch 默认 map_name:=RMUC2027_v2）
  RMUC2027_v2.pgm（11-01 修订版）、RMUC2027_gt.{pgm,yaml}
  ORIGINAL/（v2 的原始备份）

追加（同一批）：RMUC2026 的旧 2D/meta 也一并移出
  RMUC2026_spl.yaml / .pgm      稀疏(单线)版 2D 图（run_mapping_headless.sh 里只是**注释**里的示例路径）
  RMUC2026_v4.meta.yaml / _v4b.meta.yaml   旧 posegraph meta
说明：RMUC2026 在 PCD/ 下保留 RMUC2026.pcd + RMUC2026_v4b.meta.yaml（供 GICP/SC 用）；
      map/ 下目前没有 RMUC2026 的 2D 图（历史上它只在 PCD 侧有资产）。

第二批（RMUC2026 的 2D 变体）：
  RMUC2026_v4_spl / v5_spl / v6w_spl 三套（.pgm+.yaml）
保留：RMUC2026.pgm/.yaml（基础版）、RMUC2026_cloud.pgm/.yaml（点云投影版）、
      empty_map.*（空图，脚本用）、RMUC*（RMUC2024）、RMUL*（RMUL 系列，未在本次范围内）
