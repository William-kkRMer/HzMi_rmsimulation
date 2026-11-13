2026-11-03 按用户要求：**所有 .pcd 从 PCD/ 移出**（RMUC2026 / RMUC2027 的 3D 先验都判定为"失败品"：
RMUC2027.pcd 实测地面厚度 0.357 m、双层比例 83.5%，见 tools/scripts/diag/pcd_quality.py）。

⚠️ 移出后需要重建才能用 3D 先验/GICP：
  · localization:=gicp|icp|small_gicp 没有先验 PCD 可用（这些槽会读 PCD/<world>.pcd）
  · cloud_accumulator 的跨会话续建（PCD 先验）失去起点
  · 仍保留：PCD/*.meta.yaml（资产来源记录）、RMUC2027_sc_db.npz / RMUC2027_slam_poses.jsonl
    —— 但注意 SC 库与旧 PCD 是**同一跑**的，建议一起重建
重建方法：mode:=mapping + cloud_accumulator:=True（或 LIO-SAM，见 lio_sam/README_SIM.md）
