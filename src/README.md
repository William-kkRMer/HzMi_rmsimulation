# `src/` 地图（15 个目录 / 23 个包）

> **2026-11-13 整理**：清了 `__pycache__`（13 个）与空目录（**注意：`*/config` 是 CMake `install(DIRECTORY)` 的目标，已恢复**）。
> **目录没有重排**——原因见文末「为什么不按分类重排」。

## 一、按职责分组（当前物理位置）

### 1. 主线导航（本仓自己写的，天天改）

| 包 | 干什么 |
|---|---|
| **`rm_tdt_planner/`** | T-DT 完整规划器（A* + 简化 + 梯形时间 + MinimumSnap）作为 nav2 **planner 插件**；含 `plan_mask` 合并、逃逸采样、失败快照 |
| **`rm_tdt_local_controller/`** | T-DT 局部：全局 A* 给路径 → 局部 KinodynamicAstar(4 m) 出轨迹 → 前馈+反馈出速度 |
| **`rm_navigation/rm_navigation`** | nav2 参数集（base + planner/controller 槽位文件）、RViz 配置 |
| **`rm_navigation/rm_goal_checker`** | 自定义 GoalChecker：拿 `/plan` 末点当目标（绕开 nav2 把"未到点"判成到达）|
| **`rm_navigation/fake_vel_transform`** | `/cmd_vel` → `/cmd_vel_chassis` 的速度帧变换（无 minco 时用）|
| **`rm_nav_bringup/`** | **启动与资产**：`launch/`、`map/`、`PCD/`、`urdf/`、`rviz/`；三套模式入口 `bringup_mode.launch.py` |
| **`rm_nav_interfaces/`** | 分层状态消息（`StageStatus`，`/nav/l0..l3/status`）|
| **`rm_plan_msgs/`** | 规划器输出：时间参数化轨迹 + 安全走廊 |
| **`rm_utils/`** | 运行期胶水/诊断（存档守卫、会话播报、LIO 外参断言）|

### 2. 定位（zlio 为主，其余为历史备选）

| 包 | 状态 |
|---|---|
| **`rm_zlio_bridge/`** | **在用**：z-localization 位姿 → TF、速度 → 控制器；`vel_diag` 速度链诊断 |
| `rm_localization/small_point_lio`、`lio_tf_adapter` | 备选 LIO（`lio:=small_point_lio` 时才用）|
| `rm_localization/gicp_registration`、`icp_registration` | 备选重定位（`localization:=gicp`）|
| `rm_localization/slam_toolbox`、`cartographer*`、`lio_sam` | 历史建图方案（`mapper:=` 可切，现在用 zlio）|
| `rm_localization/FAST_LIO`、`point_lio` | **子模块**（`.gitmodules`，本地有改动）|
| `rm_localization/scan_context_relocalizer` | Scan Context 重定位（离线评估用）|

### 3. 感知

| 包 | 用途 |
|---|---|
| `rm_perception/patchwork_ground_segmentation` | **在用**：地面分割（`/segmentation/obstacle`）|
| `rm_perception/cloud_accumulator` | 点云累积（建图存档用）|
| `rm_perception/pointcloud_to_laserscan` | 点云 → 2D `/scan`（STVL 的 scan 源）|
| `rm_perception/rm_ground_traversability` | 坡度/台阶可通行性判据 |
| `rm_perception/linefit_ground_segementation_ros2`、`imu_complementary_filter` | 备选地面分割 / IMU 融合 |

### 4. 仿真 / 模型

| 包 | 用途 |
|---|---|
| `rm_simulation/hzmi_rm_simulation` | **场景与启动**（world、mesh、`bringup_sim.launch.py` 里的 gz 后端）|
| `rm_simulation/rm_gz_planar_move` | 平面运动插件（发 `/odom_ground_truth`，UWB 桥的数据源）|
| `rm_simulation/rm_livox_gz_sim`、`livox_laser_simulation_RO2` | Livox 仿真插件（MID360 扫描模式）|
| `rm_simulation/chassis_description`、`robot11_description` | 两个机器人槽位的 URDF/mesh |
| `rm_driver/livox_ros_driver2/src` | **子模块**：实车 Livox 驱动 |

### 5. 第三方 / 参考（vendor，能编但不在主链）

| 目录 | 说明 |
|---|---|
| **`vendor_navi_bit/`** | BIT 的 MINCO 插件对：`minco_planner`（nav2 全局规划）+ `minco_controller`（MPC）+ `ros_interfaces`（自定义轨迹消息）。`MODE=bit` 时启用 |
| **`rog_map/`** | ROGMap 库（ESDF/占据查询 + `MapRegistry`）。**给 minco_planner 进程内用**；`mask_impl:=rogmap` 档已退役，但它本身是依赖 |
| `rm_mpc_controller/` | 走廊约束 MPC（nav2 Controller 插件），`nav:=rm_mpc` 可选 |

## 二、目录嵌套的规则（不是乱）

| 形态 | 例子 | 原因 |
|---|---|---|
| 单包对单目录 | `rm_tdt_planner/` | 一目录一包，最简 |
| 目录下多包 | `rm_localization/`（11 个）、`rm_simulation/`（6 个）、`rm_perception/`（5 个）、`rm_navigation/`（3 个 + 2 子模块）| **按主题聚合**：定位/仿真/感知/导航各一族，colcon 递归查找，包名唯一即可 |
| 第三方 vendor | `vendor_navi_bit/`（3 包）、`rog_map/` | 上游代码，改得越少越好 |

**规则**：`colcon build` 按 **package.xml 的名字**找包，与目录层级无关 ⇒ 嵌套不影响构建。

## 三、为什么曾经没重排、后来怎么重排的（2026-11-13）

**重排前**：15 个目录平铺，按"绝对路径"被引用的文件数如下（改动面）：

| 包 | 被绝对路径引用的文件数 |
|---|---:|
| `rm_nav_bringup` | 117 |
| `rm_simulation` | 41 |
| `rm_localization` | 27 |
| `rm_perception` | 23 |
| `rm_navigation` | 20 |
| `rm_driver` | 6 |
| **合计** | **≈240 处 + .gitmodules 5 条** |

**最终决定做了**（2026-11-13，分支 `tidy/src-layout`）：
一次性重排 + **脚本化改写 1,338 处路径**（132 + 55 个文件）+ 全量重建验证。

**重排后结构**：

```
src/
├── nav/          rm_tdt_planner · rm_tdt_local_controller · rm_navigation · rm_nav_interfaces
│                 rm_plan_msgs · rm_mpc_controller · rm_utils
├── bringup/      rm_nav_bringup（launch / map / PCD / urdf / rviz）
├── perception/   rm_perception（patchwork · cloud_accumulator · pointcloud_to_laserscan …）
├── localization/ rm_localization（+3 子模块）· rm_zlio_bridge
├── simulation/   rm_simulation（world / mesh / gz 插件 / 两个机器人描述）
├── driver/       rm_driver（livox_ros_driver2 子模块）
└── vendor/       vendor_navi_bit（BIT 三包）· rog_map
```

**规矩**：`colcon` 按 `package.xml` 的**包名**找包，与目录层级无关 ⇒ 以后新增包放进对应组即可。

## 四、`attic/` 是**故意的**（不要删）

```
src/bringup/rm_nav_bringup/map/attic/   （26 MB，退役地图 + 说明）
src/bringup/rm_nav_bringup/PCD/attic/   （91 MB，退役点云底图 + 说明）
```

这两个目录里有 README 写明规矩：**"只搬不删"**（2026-10-07 建立）。当前默认资产不在里面，但恢复时需要。
**⇒ 不要当垃圾清掉。**

## 五、常用命令

```bash
# 列出所有包及其路径
find src -name package.xml -not -path "*/build/*" -exec dirname {} \; | sort

# 只编某几个包
colcon build --packages-select rm_tdt_planner rm_tdt_local_controller

# 看某个包装到哪了
ros2 pkg prefix <包名>
```
