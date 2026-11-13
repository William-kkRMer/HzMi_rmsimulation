# LIO-SAM（ros2 分支）在本仓的接入说明（2026-11-03）

来源：`TixiaoShan/LIO-SAM` 的 **ros2 分支**（`08af3f32`，2025-02-14），通过
`https://gh-proxy.com/` 镜像取得（本机直连 GitHub 被 TLS 拦），已删掉 `config/doc/`（66 MB 演示 gif）。

## 为什么引入

- 本仓的 3D 先验（`PCD/<world>.pcd`）由 `cloud_accumulator` **纯 LIO 里程计累积、没有回环**，
  实测 `pcd_quality.py`：地面厚度中位 **0.357 m**、双层比例 **83.5%**；
- LIO-SAM 用 **GTSAM iSAM2 + 回环检测**，正是缺的那块；
- 依赖 `ros-humble-gtsam 4.2.0` 本机已装；`colcon build --packages-select lio_sam` 通过。

## 本仓新增的两个文件

| 文件 | 作用 |
|---|---|
| `config/params_sim.yaml` | 仿真专用参数（话题/帧/外参/回环/存 PCD） |
| `launch/sim.launch.py` | 单独起：`livox_to_lio_sam` 适配 + LIO-SAM 四个节点 |

## 适配层（关键）

LIO-SAM 的 `imageProjection.cpp` 把点云反序列化成 **Velodyne 格式**
（`x,y,z,intensity,ring,time`，`sensor:=livox` 也走这条路）。我们的雷达只发
`livox_ros_driver2/CustomMsg` ⇒ 用 `cloud_accumulator/livox_to_lio_sam` 转换：

```
/livox/lidar (CustomMsg)  →  /lio_sam/points (PointCloud2: x,y,z,intensity,ring,time)
```

- `time` = `offset_time × 1e-9`（秒，相对帧首）⇒ **只有仿真插件开了
  `LIVOX_INTRA_FRAME_TIME=1` 才非零**，否则 LIO-SAM 不去畸变（= 与改造前等价）。
- IMU：用 `/imu/data`（`complementary_filter` 输出，**带 orientation**，LIO-SAM 需要它做初始对齐）。

## 用法（第一阶段：只当 3D 建图后端，不接管 map→odom）

```bash
# 1) 正常起仿真（mapping 模式，LIO 用 small_point_lio）
export LIVOX_INTRA_FRAME_TIME=1
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2027 mode:=mapping lio:=small_point_lio ...

# 2) 另开终端，单独起 LIO-SAM（它自己跑一套 map/odom，仅用于产出 3D 点云与回环结果）
ros2 launch lio_sam sim.launch.py
```

⚠️ 两者都发 `map→odom` ⇒ **同一时间只能有一个**。要把 LIO-SAM 接入主栈，需要：
① 让 LIO-SAM 输出到独立 frame 前缀（例如 `ls_map/ls_odom`）；或
② 关掉 LIO-SAM 的 TF（`mapOptimization` 里 `publishTF`），只取它的 `cloud_registered`。

## 验收

```bash
python3 tools/scripts/diag/pcd_quality.py <LIO-SAM 存的 pcd>
# 目标：地面厚度 <0.10 m、双层比例 <20%（对比现状 0.357 m / 83.5%）
```

## 已知待办

1. `N_SCAN=4 / Horizon_SCAN=1500` 是针对 MID-360 的**首猜**，需按实际点云密度调；
2. `map→odom` 发布权冲突（见上）；
3. `savePCDDirectory` 里 LIO-SAM 会**删掉整个目录再重建**（上游行为），别指向重要目录。
