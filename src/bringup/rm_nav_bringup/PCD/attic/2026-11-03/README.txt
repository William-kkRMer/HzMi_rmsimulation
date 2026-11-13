2026-11-03 整理：从 PCD/ 移出的旧资产（未删除）
  RMUC2027.pcd.bak-1029old      旧备份
  RMUC2027_v2.pcd / _v2.meta.yaml / _v2_sc_db.npz   v2 那一版（launch 用的是 PCD/<world>.pcd ⇒ RMUC2027.pcd）
  RMUC2026_spl.pcd              单线/稀疏版
  RMUC2026_v4.meta.yaml         旧 meta（保留 v4b）
保留在 PCD/ 的（launch 实际引用名）：
  RMUC2027.pcd + RMUC2027_sc_db.npz     ← GICP 先验 / SC 库
  RMUC2027_gt.pcd(+meta)、RMUC2027_slam_poses.jsonl、RMUC2027.meta.yaml
  RMUC2026.pcd + RMUC2026_v4b.meta.yaml
