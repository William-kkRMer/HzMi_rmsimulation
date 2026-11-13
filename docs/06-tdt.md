# T-DT（对照 / 代码 / 迁移记录）

> 和 T-DT 的全部对照、代码研究、改动清单、迁移报告

> **2026-11-13 合并**：本文件由 **19 份**同主题文档合并而成，原文全部保留、按来源分节（`## 来源：xxx`）。原件已删除，可从 git 历史取回。

**目录**
1. `tdt_chain_comparison.md`
2. `tdt_code_study.md`
3. `tdt_component_map.md`
4. `tdt_planning_explained.md`
5. `tdt_full_stack.md`
6. `tdt_diff_inventory.md`
7. `tdt_integration_plan.md`
8. `tdt_integration_report.md`
9. `tdt_full_completion_status.md`
10. `cod_nav_2026_deep_dive.md`
11. `cod_nav_2026_integration_plan.md`
12. `cod_nav_2026_migration_worksheet.md`
13. `cod_nav_2026_params_review.md`
14. `cod_nav_comparison.md`
15. `cod_nav_design_thinking.md`
16. `cod_nav_macro_integration.md`
17. `cod_nav_nav_stack_inventory.md`
18. `cod_nav_open_source_materials.md`
19. `cod_vs_ours_frame_architecture.md`

---


## 来源 1/19：`tdt/tdt_chain_comparison.md`


> **2026-11-13 合并**：本文件原来是一个"10 份同主题文档"的散摊子。现在把 9 份
> （`whole_chain_vs_tdt` / `tdt_full_chain_vs_tdt` / `tdt_side_by_side` / `side_by_side_two_columns` /
> `four_way_comparison` / `planning_full_chain` / `planner_controller_consumption` /
> `consumption_chain_clean` / `tdt_consumption_chain`）**原文并按主题拼进本文件**，
> 原件移到 `docs/archive/tdt_comparison/` 备查（内容未删改）。
>
> 怎么读：
> - 只想看"两边差在哪" → §A（原 `tdt_chain_comparison`，最完整的一份）
> - 想看某一环节的逐行对照 → §B 逐环节 / 逐步骤
> - 想看"障碍、速度、指令在这些模块之间怎么流动" → §C 消费链路
> - 想看四方（含 BIT）横向对比 → §D
>
> 状态：`现行`（结论以本文件为准；被取代的说法见 `docs/README.md` 的「取代表」）。

---

## §A 全链路对照（原 `tdt_chain_comparison.md`）



> 依据：`third_party/tdt-nav-kit` 源码（commit `814d5ce`）+ 其 `doc/Astar.md`、`doc/MinimumSnap.md`、`doc/Usage.md`
> + 我们跑通他们 `main.cpp` 实测（`tools/tdt_demo_stub/run.sh`）+ 我们自己的 Classic 实测。

## 1. 两条链路并排

### T-DT（一个私有导航节点内，全程内存对象）

```
先验图投影 + 实时障碍 ──→ 2D 栅格(0/255) + Mask(duration)
        │
        ├─(全局) YAstar.search(势场 0.5/(0.1+d)) → 路径
        │            → simplifyPath(0.1) → simplifyPathHypot
        │            → trapezoidalTimeAllocation(vmax,amax)
        │            → MinimumSnap(OSQPCorridor + 交替迭代 + 时间归一化)
        │            → 平滑轨迹(位置) + 分段时长
        │
        └─(局部) 取 4 m 外路径点 → KinodynamicAstar([x,y,vx,vy], maxNodes 1.5e5)
                     → 轨迹 Sample{t,x,y,vx,vy,ax,ay}
        │
        └─→ **私有控制器**（未开源）：轨迹 → cmd_vel
```

### 我们（nav2 骨架 + 两个 T-DT 插件）

```
map(PGM/PCD) ─┐
patchwork 点云 ┴→ mask_builder → /plan_mask(OccupancyGrid)
        │
        ├─(全局) rm_tdt_planner（nav2 GlobalPlanner 插件）
        │            二值图(lethal + pad 2 格) + Mask
        │            YAstar(同参数) → simplify → 梯形时间 → MinimumSnap(同参数)
        │            → /trajectory_plan（t,x,y,vx,vy + 走廊） + nav_msgs/Path
        │
        └─(局部) rm_tdt_local_controller（nav2 Controller 插件）
                     投影 Path → 前视 4 m 作局部目标 → KinodynamicAstar(1.5e5)
                     → 按轨迹前馈（最近样本+2 的速度）+ 航向反馈
                     → 失败回退 pure-pursuit → /cmd_vel
```

## 2. 全局规划：逐阶段对比

| # | 阶段 | T-DT | 我们 | 判定 |
|---|---|---|---|---|
| G1 | 栅格构建 | 私有（先验图投影 + 实时障碍） | costmap lethal → 二值，pad 2 格边框 | ⚠️ 语义等价、**实现自建** |
| G2 | 动态障碍 | `Mask` + `pushMask`（生产者私有） | patchwork → `/plan_mask` → `setMaskMapTimed(1.0 s, 16)` | ✅ 接口同、**内容自建** |
| G3 | A\* | `YAstar` + `costField=0.5/(0.1+d)` + `setMaskNumLayers(4)` | 同源码同参数 | ✅ **完全一致** |
| G4 | 简化 | `simplifyPath(0.1)` + `simplifyPathHypot` | 同 | ✅ **完全一致** |
| G5 | 时间分配 | `trapezoidalTimeAllocation(vmax, amax)` | 同（vmax/amax = 底盘 1.5/2.0） | ✅ 同函数，参数按底盘 |
| G6 | 后端优化 | `MinimumSnap`：order6 / maxDx3 / dt0.1 / TL10 / collisionIter6 / range2.5 / shrink0 / normTime **true** / OSQPCorridor / initVel{0,0} | **逐项相同** | ✅ **完全一致** |
| G7 | 轨迹时间 | 分段时长 `timeAllocated` | **由 MinSnap 分段时长还原**（每段 10 采样点） | ✅ **完全一致** |
| G8 | 走廊 | MinSnap 内部 `autoCorridor` | **`SolveOutput.corridor`** | ✅ **完全一致** |
| G9 | 输出 | 内存对象 | `TrajectoryPlan` + `nav_msgs/Path`（ROS） | ⚠️ 传输方式不同 |

## 3. 局部规划/控制：逐阶段对比

| # | 阶段 | T-DT | 我们 | 判定 |
|---|---|---|---|---|
| L1 | 何时切局部 | 文档：全局 A\*/JPS + **4 m 内 Kastar** | 前视 4 m 作局部目标 | ✅ 思路一致 |
| L2 | 局部搜索 | `KinodynamicAstar`（`maxNodes 1.5e5`、`sampleTime 0.01`） | 同源码、同参数 | ✅ **完全一致** |
| L3 | 栅格 | 同一张图 | 局部 costmap lethal + pad 2 格 | ⚠️ 实现自建 |
| L4 | **轨迹→cmd_vel（执行层）** | 私有节点里把 Kastar 的 `state[x,y,vx,vy]` 直接下发（未开源，但逻辑必然相同：取样本→转车体系） | 取最近样本+2 的速度 → 转车体系 → 航向反馈；失败回退 pure-pursuit | ⚠️ **执行层（~30 行）**，不是独立控制算法 |
| L5 | 失败处理 | 未知 | 回退 pure-pursuit + 统计计数 | ❌ 自研 |

## 4. 参数对照（同一张表）

| 参数 | T-DT demo | 我们 | 说明 |
|---|---|---|---|
| `setCostField` | `0.5/(0.1+d)` | 同 | |
| `setMaskNumLayers` | 4 | 4 | |
| `simplifyPath` 阈值 | 0.1 | 0.1 | |
| `order / maxDx / dt / TL` | 6 / 3 / 0.1 / 10 | 同 | |
| `collisionCheckIter` | 6 | 6 | |
| `maxCorridorRange` | 2.5 | 2.5 | |
| `corridorShrink` | 0.0 | 0.0 | |
| `normTime` | true | **true** | 已对齐 |
| `backend` | OSQPCorridor | 同 | |
| `initVel` | {0,0} | 同 | |
| `maxSpeed / maxAcc`（规划） | 3 / 1（**飞行器**） | **1.5 / 2.0**（底盘） | 物理量替换 |
| 地图边框 | PNG 自带黑边 | **pad 2 格** | 必须（`_getBoundSquare2` 不查边界） |
| Kastar `maxNodes` | 150000 | **150000** | 已对齐 |
| Kastar `sampleTime` | 0.01 | 0.01 | |
| 局部前视 | 文档 4 m | 4.0 | |
| `min_local_goal` | —— | 1.0 | 我们加的（太近 Kastar 必失败） |

## 5. 性能对照（实测）

| 项 | T-DT（他们 demo，1000 次随机） | 我们（Classic） |
|---|---|---|
| A\* 搜索 | 平均 **4.17 ms** / 最大 11.85 ms | 同一份代码（未单独统计） |
| 建代价图 | 平均 2.26 ms / 最大 5.79 ms | 同 |
| **MinSnap 求解** | 平均 **2.94 ms** / 最大 **80.3 ms** | 0.3~4.6 ms（场内短路径） |
| 简化 | 14.9 µs | 同 |
| Kastar（全局 20 m+） | **1686 ms** | 不用（只局部 4 m） |
| MinSnap `success=1` 比例 | **554 / 1006 ≈ 55%** | 同（约 55%，`snap=0` 仍用其 path） |
| 端到端到点 | —— | 近距目标 **SUCCEEDED / 40.7 s（wall）** |
| 局部 Kastar 成功率 | —— | **ok 337 / fail 402 / fallback 861**（≈46% 成功） |

## 6. 已知差距（无法消除 / 待改进）

| # | 差距 | 性质 |
|---|---|---|
| 1 | **执行层（轨迹样本→cmd_vel）** | T-DT 未开源，但它只是"取样本→转车体系"，逻辑唯一；**不是**独立的控制算法 |
| 2 | 栅格/Mask 的生产者 | T-DT 私有 ⇒ 我们自建（语义一致） |
| 3 | `maxSpeed/maxAcc` | 底盘物理量不同 ⇒ 速度剖面不同（正常） |
| 4 | 局部 Kastar 成功率 ~46% | 局部目标选法/参数需调（我们的实现问题） |
| 5 | 远距目标未稳定到点 | 同上 |
| 6 | MinSnap `success=0` | **T-DT 自身行为**（45%），非差距 |

## 7. 一句话结论

> **全局规划（G1~G8）逐函数/逐参数/逐时间源/逐走廊源与 T-DT 一致；
> 局部就是"用 Kastar 当控制器"——我们每个控制周期都跑 Kastar 并直接下发它的速度；
> 剩下只有 ~30 行"取样本→转车体系"的执行层，逻辑上与他们私有的那层等价。**

## 8. 更正：T-DT 的"局部规划器 = 控制器"

之前把 L4 写成"完全自研的控制器"是**说重了**。准确说法：

- T-DT 只有**两个规划器**：`YAstar`（全局）、`KinodynamicAstar`（局部）；
- **局部那个承担控制职责**——它的输出 `Sample{t, x, y, vx, vy, ax, ay}` 本身就是"要执行的速度/轨迹"；
- 我们做的正是这个：**每个控制周期调一次 Kastar，直接下发它的速度**；
- 所谓"控制器"只是把 `state` 取出来、转到车体系（~30 行），**不是额外的控制算法**（不是 MPC、不是 PID 回路）。

⇒ **"两个规划器，局部当控制器"这件事，我们已经照做了。**

## 9. 改成"先验图直喂"之后（2026-11-06 晚）

### 9.1 改动

| 项 | 之前 | 现在 |
|---|---|---|
| 规划栅格来源 | nav2 costmap 的 lethal 格 | **`/map`（map_server 发布的 PGM 先验图）** |
| 规划坐标系 | 先转 costmap 帧再转回 | **全程 `map` 帧**（不再 TF 往返） |
| 栅格尺寸 | costmap 窗口 | 先验图 560×313 **+ pad 2** = 564×317 |
| Mask 帧 | **错的**（直接用 `livox_frame` 的 x,y 当 map 坐标） | **逐点 TF 到 `map` 帧**再栅格化 |

### 9.2 验证

```
[tdt] /map 先验图: 564x317(+pad2) @ 0.050 m, origin (-20.00, -8.99), 障碍 10476 格
```

- **10476 格**与 PGM 里 `0` 值像素数**完全一致**（10476）⇒ 先验图被正确读出；
- 到点：**SUCCEEDED / 41.6 s**；
- 局部 Kastar：`ok=517 / fail=276` ⇒ **失败率 46% → 33%**（帧一致后 Kastar 目标更准）。

### 9.3 与 T-DT 的对齐情况更新

| 差异 | 状态 |
|---|---|
| #5 栅格来源（costmap vs 先验图） | ✅ **已消除**（都用先验图） |
| #3 两套坐标（map / odom_level） | ✅ **已消除**（规划全在 map） |
| Mask 帧错 | ✅ **已修** |
| 规划器依赖 costmap 生命周期 | ✅ 解除 |

## 10. 跑 z-localization（2026-11-07）

### 10.1 编译（独立工作空间，避免 livox 包名冲突）

```bash
mkdir -p .tmp_cache/zlio_ws/src && cd .tmp_cache/zlio_ws/src
for p in frontend backend localization_interfaces livox_ros_driver2 bringup; do
  ln -sfn $REPO/third_party/z-localization/src/$p $p; done
ln -sfn $REPO/third_party/z-localization/cmake ../cmake   # ★ 必须：包内 include ../../cmake/Warnings.cmake
colcon build --symlink-install --packages-up-to bringup --cmake-args -DBUILD_TESTING=OFF
```

两个坑（都已解决）：

| 坑 | 现象 | 解决 |
|---|---|---|
| 测试 CMake 缺函数 | `Unknown CMake command "localization_enable_warnings"` | `-DBUILD_TESTING=OFF` |
| 相对路径找 cmake 模块 | 同上（backend 主文件也用） | 在工作空间根建 `cmake` 软链到仓库 `cmake/` |

### 10.2 运行结果

对着我们 Classic 仿真（`/livox/lidar` CustomMsg 5 Hz、`/livox/imu`）：

```
ros2 launch bringup bringup.launch.py \
  frontend_config:=.../config/lio.yaml backend_config:=.../config/backend.yaml \
  mode:=mapping use_sim_time:=true imu_acceleration_scale:=1.0
```

- ✅ 节点起来了，出现了 `/frontend/frame`、`/frontend/odometry`、`/backend_node/localization_pose`、`/localization/lio_world_cloud`；
- ❌ **没有 `/localization/odometry`**（前端拒绝输入）：

```
[zlio] odometry input interrupted: LiDAR and IMU time difference exceeds the allowed input latency
```

### 10.3 原因分析（待验证）

`src/frontend/src/input.cpp:247` 判据：若「最后一个 IMU 样本」与「该帧点云的采集结束时刻」差距超过
`ImuHistoryLimits.maximum_duration`（默认 **30 s**）就抛此异常；另有 `maximum_interval`（默认 **0.1 s**）。

⇒ 说明**IMU 与 LiDAR 的时间戳/覆盖关系不满足**，两个候选：

1. **IMU 时间戳不在同一时钟域**（最可能：一个用 sim time，一个用墙钟）；
2. **IMU 间隔过大/抖动**（实测墙钟约 41 Hz、最大间隔 126 ms；RTF<1 时会放大）。

### 10.4 下一步

1. 确认 IMU 与 LiDAR 的 `header.stamp` 是否都在 sim time（用同一节点打印）；
2. 把仿真 IMU 提到稳定 100 Hz（他们文档的判据是 100 Hz）；
3. 必要时在 config 里显式放开 `imu_history` 的 `maximum_duration` / `maximum_interval`；
4. 单独跑时**不要**同时启动我们的 LIO（两者都会发 map→base 的 TF）。

## 11. 去掉 odom_level（2026-11-07）

### 11.1 改动

| 层 | 之前 | 现在 |
|---|---|---|
| 全局规划 | `/map`（map 帧） | 不变（本来就不依赖 odom_level） |
| **局部 Kastar 栅格** | 局部 costmap（**odom_level**）+ TF 变换 | **`/map` 先验图（map 帧），无 TF** |
| nav2 代价图 | local=odom_level | **`local_frame:=map`** |
| 我们链路的坐标 | map ↔ odom_level 两套 | **只有 map** |

### 11.2 验证（`local_frame:=map`）

```
nav2 激活: ✅（3 条激活日志）
"Invalid frame ID map" 报错: 0 条
目标 (2.0, 0.5): SUCCEEDED / 37.8 s   ← 比之前 40.7 s 更快
Kastar: ok=197 / fail=451            ← 失败率 70%（变差，见下）
```

### 11.3 问题记录

| # | 问题 | 现象 | 待办 |
|---|---|---|---|
| 1 | **Kastar 失败率升高**（33% → 70%） | 换成全局先验图（无膨胀、无障碍物邻居约束）后，局部目标更容易落在贴墙/走廊边缘 | 局部目标取法（沿切线前移/放宽半径）、或把栅格裁成机器人周围 12 m 窗口 |
| 2 | nav2 的 obstacle 层仍用 `odom` 帧 | base yaml 里 `global_frame: odom`（128/543 行） | 属 nav2 内部；我们链路不再引用 |
| 3 | `local_frame:=map` 依赖定位先出 map→odom | 之前（AMCL）会卡激活 8 s+ | 现在 beluga 正常；说明是 AMCL 特有问题 |
| 4 | Mask 未并入局部 Kastar 栅格 | 局部只用先验图，看不到动态障碍 | 把 `/plan_mask` 叠加进局部栅格（或在 Kastar 前把 mask 格子设 0） |

## 12. z-localization 打通（2026-11-07）

### 12.1 根因（已定位并修复）

我们的仿真 **Livox `CustomMsg` 没有填 `timebase`**（一直是 0），而 `offset_time` 也是 0：

```
[dbg] imu_last=30016000000 scan_end=0 diff=30016000000
        ↑ IMU 时间(ns)          ↑ 扫描结束时刻被算成 0
```

⇒ z-localization 认为"点云采集结束时刻 = 0"，与 IMU 差 30 s+ ⇒ 每帧都判
`LiDAR and IMU time difference exceeds the allowed input latency` 并丢弃。

**修复**（我们自己的插件，`src/simulation/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp`）：

```cpp
pp_livox.timebase = static_cast<uint64_t>(sim_time.sec) * 1000000000ULL +
                    static_cast<uint64_t>(sim_time.nsec);   // 与 header.stamp 同一时间轴（仿真钟 ns）
```

### 12.2 修复后的效果

| 指标 | 修复前 | 修复后 |
|---|---|---|
| `input interrupted` | 292 次/45 s | **0** |
| `/frontend/odometry` | 无 | **43 Hz** ✅ |
| 后端首帧 | 无 | `first synchronized front-end frame: stamp=22.14, points=2100, covariance=valid` ✅ |

### 12.3 剩余

| # | 项 | 状态 |
|---|---|---|
| 1 | `/localization/odometry`（后端输出） | 需跑更久（`mode:=mapping` 要先建图）或提供 prior bundle |
| 2 | backend 的 prior/loop/UWB 告警 | 预期（mapping 模式无先验，`prior_map=disabled`） |
| 3 | 接 v0 / TF | 待后端出 odometry 后做 |

### 12.4 顺带得到的经验

| 结论 | 说明 |
|---|---|
| `ROS_DOMAIN_ID` 必须 ≤ 232 | 我们误用 250~253 ⇒ cyclonedds/fastrtps 都报 "rmw handle invalid"/"port too high"，**整栈静默失败** |
| 排查 zlio 要先看它算出的 `scan_end` | 不是看 IMU 频率 |

## 13. 接上 z-localization（v0 + TF）

### 13.1 话题/类型（实测确认）

| 话题 | 类型 | 内容 |
|---|---|---|
| `/frontend/odometry` | `nav_msgs/Odometry` | 前端 LIO（43 Hz） |
| `/backend_node/localization_pose` | `geometry_msgs/PoseWithCovarianceStamped` | **map 系位姿** |
| `/localization/odometry` | `nav_msgs/Odometry` | map 系位姿 + **传感器系速度** |
| `/backend_node/localization_state` | `std_msgs/String` | 定位可信度 |

### 13.2 已接的两处

| 用途 | 实现 | 参数 |
|---|---|---|
| **v0（真实速度）** | `rm_tdt_local_controller` 订阅 `/localization/odometry`，用 `twist.twist.linear`（传感器系）按当前航向转到世界系；无数据时回退位姿差分 | `use_zlio_vel`（默认 true）、`zlio_odom_topic` |
| **TF** | 新包 `rm_zlio_bridge`：`/backend_node/localization_pose` → TF `map → base_link` | `parent_frame`=map、`child_frame`=base_link、`pose_topic` |

### 13.3 注意（必须做对，否则 TF 树冲突）

`map→base_link` 只能有**一个**发布者。接 zlio 时必须**停掉**我们原有的
`odom_level → base_link`（LIO/AMCL 链），即用 `localization:=zlio` 模式；
否则 `base_link` 会有两个父节点，nav2 会报 TF 树错误。

### 13.4 待做

| # | 项 | 说明 |
|---|---|---|
| 1 | `localization:=zlio` 启动模式 | 停 beluga/AMCL 与 LIO 的 TF，只留 zlio_bridge |
| 2 | 后端 `/localization/odometry` 稳定性 | mapping 模式约 90 s 后才出现；更快的做法是先建好 prior bundle 用 `mode:=localization` |
| 3 | 对齐 zlio 的 `body`/`imu` 帧名 | 若与我们的 `base_link` 不同，需要在 zlio 配置里改 `frame: body` 或加静态边 |

## 14. `localization:=zlio` 一体化启动（2026-11-07）

新增 `src/bringup/rm_nav_bringup/launch/bringup_zlio.launch.py`：

```
livox(CustomMsg, timebase 已修) + IMU
   → frontend(zlio_node)  ← 自己起，可放开队列上限
   → backend(backend_node, mode:=mapping)
   → /backend_node/localization_pose → rm_zlio_bridge → TF map→base_link
规划: /map 先验图 → rm_tdt_planner → /trajectory_plan
局部: rm_tdt_local_controller（4 m Kastar，v0 取 /localization/odometry）
```

关键：`lio:=none` + `localization:=''`（**停掉我们自己的 LIO/AMCL TF**），
否则 `map→base_link` 会有两个父节点。

### 14.1 当前结果

| 项 | 状态 |
|---|---|
| zlio 单独跑（我们 sim + 少量负载） | ✅ 前端 43 Hz、后端出首帧 |
| 全栈一起跑（sim + nav2 + 规划 + 局部 + zlio） | ❌ `IMU input failed: input event queue capacity exceeded`（前端被 CPU 饿死） |
| TF bridge | 未验证（前端先挂） |

⇒ **不是算法问题，是这台机器同时跑「仿真 + nav2 + zlio」时 CPU 不够**。

### 14.2 可选解法

| # | 办法 | 代价 |
|---|---|---|
| 1 | 测 zlio 时先不跑 nav2（只跑 sim + zlio + bridge） | 验证 TF/速度足够 |
| 2 | 降低仿真负载（`ground:=linefit`、`local_obstacle:=voxel`、减少 lidar 采样） | 数据略糙 |
| 3 | 换更快机器 / 限制 nav2 频率 | 硬件 |
| 4 | 继续调 zlio 前端队列参数 | 治标 |

## 15. 队列溢出的真因（我的参数写错了）

| 项 | 值 |
|---|---|
| zlio 前端默认 `maximum_queue_events` | **8192** |
| 我第一版 launch 里设的 | **128** ← 把上限调小了 64 倍 |
| 后果 | 初始化窗口（`window_frames=20`，约 4 s）期间 IMU 事件（~80 Hz）堆到 320+ ⇒ 超 128 ⇒ `input event queue capacity exceeded` |

**CPU 并不缺**：28 核、load avg 1.58、gzserver 73%、zlio 36% —— 所以不是性能问题。

修复：`maximum_queue_events: 16384`。

修复后：

```
backend: first synchronized front-end frame: stamp=3.236, points=2175, covariance=valid  ✅
zlio_tf: [zlio_tf] map → base_link, 订阅 /backend_node/localization_pose                    ✅ 已订阅
仍有 1 次 Livox input failed（初始化瞬态）
```

### 15.1 待确认

| # | 项 | 说明 |
|---|---|---|
| 1 | `/backend_node/localization_pose` 在 `mapping` 模式是否发布 | 若只在 `localization`（有先验）模式发布，则需先建 prior bundle |
| 2 | 前端 stamp 3.236 与仿真时钟的对应关系 | 需确认前端时间基（是否重置过） |
| 3 | TF `map→base_link` 实际数据 | 依赖 #1 |

## 16. 只跑 sim + zlio 的结果（无 nav2）

```
仿真: lidar 4.8 Hz 正常
zlio: backend first synchronized front-end frame: stamp=20.144, points=2099, covariance=valid   ✅
约 70 s 后: [zlio_node] Livox input failed: input event queue capacity exceeded
```

- 机器 28 核、load 1.58 ⇒ **不是性能问题**；
- 队列用**默认 8192**（我们已改回 16384）仍溢出 ⇒ 前端**处理线程停了约 100 s**（8192 / 80 Hz ≈ 100 s），不是"慢"，是**卡住**；
- 机器人**静止**时也会卡 ⇒ 与运动无关。

### 16.1 结论

zlio 能正常启动、能出同步帧，但运行一段时间后**前端停摆**。这不是我们的集成问题（参数、时间基、队列都对了），
更像是**我们的仿真数据触发他们前端某个卡死路径**（待定位）或上游 bug。

### 16.2 下一步（建议）

| # | 实验 | 目的 |
|---|---|---|
| 1 | 把 `/livox/lidar` 的 `point_num` 降到 5k、`always_on` 保持 | 排除点数/毛刺导致 |
| 2 | 用 `wait_for_start:=true` + 手动触发 | 看是否是 reset 协调卡死 |
| 3 | 抓前端卡死时的线程栈（`gdb -p` 或 `eu-stack`） | 直接看到卡在哪一行 |
| 4 | 换 zlio 的 `mode:=localization` + prior bundle | 排除 mapping 模式特有路径 |

## 17. "zlio 卡死"的真因：**队列溢出后永久 Failed**（线程栈已抓到）

### 17.1 栈证据（`gdb` 以子进程方式 attach，ptrace_scope=1 只能这样）

```
Thread 1  (main)     : futex/DDS WaitSet::wait        ← ROS 执行器空闲，等不到回调
Thread 45 (BatchExec): condition_variable::wait       ← 工作线程空闲
Thread 6/7/9         : recvfrom (DDS 收包线程仍活)
```

⇒ 没有任何线程在算、也没有自旋：**输入被"关闸"了**，不是死锁也不是 CPU 不够。

### 17.2 代码层面原因

`InputQueue::push()`：

```cpp
if (state_ == State::Failed) return;                 // ← 一旦 Failed，永久丢弃
if (events_.size() >= maximum_events_ || ...) throw; // ← 溢出抛异常 → 置 Failed
```

⇒ **只要溢出一次**，前端就再也不接受点云/IMU ⇒ 表现为"跑一会儿卡死"。

### 17.3 修复（已生效）

| 参数 | 默认 | 我们设 |
|---|---|---|
| `maximum_queue_events` | 8192 | **200000** |
| `maximum_buffered_points` | 1000000 | **20000000** |

实测（sensor-only，120 s）：

```
溢出次数: 0
/frontend/odometry: 42.7 Hz 持续     ✅（此前约 70 s 就停）
```

全栈（sim+nav2+zlio，130 s）：溢出 0、前端 **24.7 Hz**（负载高时降速但**不挂**）。

### 17.4 仍待解决

`/backend_node/localization_pose` 与 TF `map→base_link` 仍未出来 ——
后端只在**已提交定位状态**时才发布位姿，而 `mode:=mapping` 没有先验匹配前状态未提交。

⇒ 下一步：用他们的 `build_pcd_map_bundle` 从我们的 PCD 生成 **prior bundle**，用 `mode:=localization` 跑。

## 18. prior bundle + `mode:=localization`（2026-11-07）

### 18.1 生成 bundle

```bash
# 1) 把地图 PCD 裁到场地范围（脚本里固定 -20..8 / -9..6.7）
# 2) 用他们的工具生成关键帧 + Scan Context
tools/scripts/zlio/build_prior_bundle.sh .tmp_cache/prior/prior_crop.pcd .tmp_cache/prior/bundle 0.35
# → Bundle pose count=464, clouds/ keyframes.txt map.pcd sc_descriptors.bin
```

**坑**：`--sensor-z` 必须等于 backend.yaml 的 `common.scan_context.lidar_height`（**0.35**），
否则后端拒绝加载：`saved SC config does not match runtime config (saved lidar_height=0.25 vs runtime 0.35)`。

### 18.2 `mode:=localization` 实测

```
[backend] loaded validated map: keyframes=464
[backend] localization state -> NORMAL (recovered via Scan Context + scan-to-map, delta_t=1.34m)
[backend] localization state -> SUSPECT (rmse=0.184, support=417/862)  ← 偶尔掉
[backend] localization state -> NORMAL (recovered, delta_t=0.90m)
/backend_node/localization_pose : 29 Hz  ✅
```

⇒ **定位链路通了**（状态机在 NORMAL/SUSPECT 间切换，能自恢复）。

### 18.3 新问题：**prior 与仿真地图不同帧**

TF 验证（`tf2_echo map zlio_base`，sim time）：

```
Translation: [-52.980, -482.180, -80.135]      ← 明显错误（机器人应在 (5.9, 2.7) 附近）
RPY(deg): [-36.4, -24.4, 130.5]
```

原因：prior bundle 由 `third_party/eval_data/map_rmuc2027_run1.pcd` 生成，
该 PCD 的坐标系与我们 `RMUC2027_v2` 的 `map` 帧**不一致**（外接框 y 到 −19.6，而我们的图只到 −8.89）。

### 18.4 下一步

| # | 办法 |
|---|---|
| 1 | **把 prior PCD 配准到我们的 map 帧**（2D 相关性/ICP 求 dx,dy,dθ，再 transform PCD） |
| 2 | 或用我们**自己仿真建图**导出的 PCD（天然在 map 帧） |
| 3 | 配准后再跑 localization，验证位姿落在 (5.9, 2.7) 附近 |

## 19. prior 配准尝试与结论（2026-11-07）

### 19.1 试过的 4 种 prior（全部未通过后端验证）

| # | prior 来源 | 对齐 | 结果 |
|---|---|---|---|
| 1 | `third_party/eval_data/map_rmuc2027_run1.pcd` | yaw 180°, IoU 0.09 | ❌ 实为**单帧扫描**（径向射线），不是地图 |
| 2 | `export/lio_sam_pcd/cloudGlobal.pcd` | 2D 相关对齐，IoU 0.20 | ❌ 只覆盖**左半场** |
| 3 | 由 PGM 合成墙体（3 层高度） | 天然在 map 帧 | ❌ `scan-to-map verification failed, support_ok=false` |
| 4 | `RMUC2027_cad_coarse.pcd`（配准后） | yaw 0°, t=(−5.8,−2.75) | ❌ 仍 `support_ok=false`（CAD 含地面/天花板，与仿真渲染差异大） |

配准工具：`tools/scripts/zlio/align_prior.py`（yaw 搜索 + FFT 平移 + IoU 细化）。

### 19.2 正确做法（他们的标准流程）

后端提供 **`save_map` 服务**（`std_srvs/Trigger`）。所以应该：

```
1) mode:=mapping（无 prior）→ 让 zlio 自己建图
2) 让机器人跑遍场地（发 cmd_vel / 遥测，或直接 teleport）
3) 调 save_map 服务 → 得到 **同源** bundle（关键帧 + 描述子）
4) mode:=localization prior_map:=<该 bundle> → 定位
```

⇒ 先验必须**来自同一渲染源**，外部 PCD/CAD 都过不了他们的 scan-to-map 验证。

## 20. T-DT/zlio 专用 RViz（3D 点云）

配置：`tools/configs/zlio.rviz`（由仓库里能用的 `fastlio.rviz` 模板改出来，保证 key 合法）

```bash
rviz2 -d tools/configs/zlio.rviz --ros-args -p use_sim_time:=true
```

| 显示 | 话题 | QoS |
|---|---|---|
| **zlio_runtime_map**（T-DT 的 3D 地图，随建图增长） | `/backend_node/runtime_map` | **Transient Local** |
| livox_current（当前雷达帧） | `/livox/lidar/pointcloud` | Volatile / Best Effort |
| zlio_prior_map（定位模式下的先验图，默认关） | `/backend_node/prior_map` | Transient Local |
| zlio_optimized_path | `/backend_node/optimized_path` | Transient Local |
| TF / RobotModel | —— | —— |

Fixed Frame = `map`；必须加 `use_sim_time:=true`，否则 TF 全部报错。

## 21. 建图/保存失败的根因（2026-11-07 定位并修复）

### 21.1 根因：`uwb.enabled: true` 让状态永远停在 Initializing

后端源码 `localization.cpp:282`：

```cpp
if (!config_.prior_map_enabled &&
    (state == Initializing || state == Suspect)) {
    if (state == Initializing && (applied_trusted_seed || !config_.uwb.enabled)) {
        state_changed = startTrajectoryLocked(frame, applied_trusted_seed);   // ← 只有这两条路
    } else if (config_.uwb.enabled && tryCommitUwbBootstrapLocked(frame)) { ... }
}
```

- 我们**没有先验**（`prior_map_enabled=false`）、**没有可信种子**（无 startup hint）；
- 但配置里 `uwb.enabled: true` ⇒ `!uwb.enabled` 为 false ⇒ 第一条路进不去；
- 第二条路要求真的收到 UWB（`/uwb_pose`）—— 仿真里没有 ⇒ **永远停在 Initializing**；
- 于是 `poseTrusted(Initializing) == false`（`localization_state.cpp:33`，只认 Normal/ObsDegraded）
  ⇒ `can_record=false` ⇒ **一个关键帧都不建** ⇒ `saveMapBundle` 里 `frames.empty()` ⇒ 保存失败。

**修复**：`tools/configs/zlio_backend_mapping_save.yaml` 里 `uwb.enabled: false`。

### 21.2 修复后的实测（headless，程序化开车）

```
t=20s  runtime_width=3220   epoch=0 invalid=0 cov=0 watermark=0
t=60s  runtime_width=4677
t=120s runtime_width=5237
开车 3 轮后 width=7666
save_map -> success=True, "saved runtime map to .../runtime_map"
关键帧 44; tx[-0.02,3.32] ty[0.01,1.51]
```

⇒ **建图与保存都通了**。目录里得到 `keyframes.txt`、`clouds/`、`map.pcd`。

### 21.3 描述子要另外生成

`save_map` **不写** `sc_descriptors.bin`（mapping 模式下 descriptor 是空壳）。用他们的工具补：

```bash
.tmp_cache/zlio_ws/install/backend/lib/backend/rebuild_scan_context_descriptors \
  --bundle .tmp_cache/prior/runtime_map --lidar-height 0.35
# -> Wrote N Scan Context descriptors
```

### 21.4 另一条重要教训：**不要开** `LIVOX_INTRA_FRAME_TIME`

开了之后 scan 终点 = `timebase + 0.1 s`，部分帧 `begin > stamp` ⇒ 后端判
`invalid ZLIO observation` ⇒ 触发**协调 reset** ⇒ 前端反复重初始化（`accepting epoch N`）。
**保持关闭**（默认即关闭），只保留 `timebase` 修复。

### 21.5 仍未解决

| # | 问题 | 证据 |
|---|---|---|
| 1 | `mode:=localization` 用自建 prior 位姿发散 | pose y 从 80 → 130 → 157 m 持续增大 |
| 2 | 前端里程计位移与真值未对齐验证 | `/frontend/odometry` 的 `pose.pose.position` 取不到（初始化窗口不足） |
| 3 | prior 覆盖太小 | 44 关键帧仅跨 3.3 m × 1.5 m |

## 22. 关键发现：仿真 IMU 在静止时就有 ±2.5 m/s² / ±0.61 rad/s 的方波抖动

### 22.1 实测（机器人静止、姿态水平）

```
真值姿态: 四元数 ≈ (0,0,0,1)  ⇒ 车身水平、无转动
IMU accel: (x≈0, y 在 -2.5 与 +2.0 之间逐帧跳, z≈9.8)
IMU gyro : x 在 +0.61 与 -0.61 之间逐帧翻, y/z≈0
```

采样 102~133 点，`ay min/max = -4.69/+2.10`、`gx = ±0.61`，**方波式**（隔一个样本换一次号）。

⇒ 这不是 IMU 噪声，是**车体在轮子上以物理步频振动**（world `max_step_size=0.004`、250 Hz）。

### 22.2 后果（实测对照）

| 配置 | 前向 x 跟踪 | 横向 y |
|---|---|---|
| 原始 IMU | 真值走 1.97 m，zlio 走 2.84 m | **±3 m 乱跳** |
| **低通后（alpha 0.05）** | 真值 2.06 m，zlio **2.10 m ✅** | 仍 **±10 m 发散** |

⇒ **前向已经准了**（证明 lidar/尺度没问题），**横向仍发散**，根因就是这层振动。

### 22.3 新增工具

`rm_zlio_bridge/imu_lpf`：把 `/livox/imu` 低通后发到 `/livox/imu_filtered`（保留时间戳/帧），
给 zlio 用 `-p imu_topic:=/livox/imu_filtered`。

```
alpha 0.15 → ay ∈ [-0.92,+0.02]、gx ∈ [-0.28,+0.28]
alpha 0.05 → 更平滑（但滞后更大）
```

### 22.4 真正的修法（下一步）

| # | 做法 | 说明 |
|---|---|---|
| 1 | **抬高 spawn 2~3 cm** | 现在 `spawn_z=0.129`，实测 `h_meas=0.056` vs `h_urdf=0.188` ⇒ 车陷进地面 13 cm ⇒ 接触抖动 |
| 2 | 提高物理步频 | `max_step_size 0.004 → 0.002`（代价：CPU） |
| 3 | 轮子接触参数 | kp/kd/摩擦（之前调过，收敛有限） |
| 4 | 低通当临时方案 | 只解决前向，横向仍发散 ⇒ **不能只靠滤波** |

## 23. zlio 前端在仿真里的稳定性排查（2026-11-08）

### 23.1 已修（都已验证）

| # | 修复 | 效果 |
|---|---|---|
| 1 | 物理步长 4 ms → 1 ms | IMU `gx` ±0.61 → 0、`ay` ±2.5 → ±0.3 |
| 2 | **IMU 协方差全 0 → 填 0.02 / 0.001** | 静止位姿从 −11/+11/+63 m 乱跳 → **稳定常数 −14.7 m** |
| 3 | 扫描时间语义（timebase=帧尾 → 帧首） | 开逐点 offset 时帧尾不再落到未来 |
| 4 | `uwb.enabled: false` | 后端能建图、`save_map` 成功 |

### 23.2 已排除（都有实测）

| 假设 | 实验 | 结论 |
|---|---|---|
| 点数太少 | `sampling.maximum_points` 4000 → **12000** | 无改善（17 m → 120 m 同样发散） |
| 速度相关比例 | 0.1 m/s 慢速 | 仍发散（−7 → +60 m）⇒ **不是尺度** |
| 雷达/IMU 时间序 | 实测 `lidar - max(imu)` = −4 ms ~ +6 ms | 正常，不是外推问题 |
| CustomMsg 的 `line`/`tag` | 前端代码不读这两个字段 | 与它无关 |

### 23.3 当前现象

- **静止**：位姿稳定（常数偏置 −14.7 m，属帧原点差异，不影响建图）；
- **一移动**：速度估计约为真值 **30 倍**，位置在 15 s 内从 0 涨到 120 m。

### 23.4 剩余最可疑的方向

RMUC2027 出生点附近是**长直平面墙**，几何上沿墙方向**约束极弱**（孔径问题），
scан-to-map 在这类走廊式场景里横向不可观 —— 真机 T-DT 场地上有大量立柱/箱体/栅栏提供约束。

**验证办法**：在出生点附近加几个几何特征（立柱/箱体），再跑同样的对照；
若前端在特征丰富处稳定，则确认是"场景欠约束"，而非前端缺陷。

## 24. 回放实验与 IMU 空隙（2026-11-08）

### 24.1 录包 + 回放（排除仿真节奏影响）

- 录：`/livox/lidar`、`/livox/imu`、`/odom_ground_truth`、`/clock`，32.5 s，148 帧雷达、1479 帧 IMU；
- 回放：`ros2 bag play --clock --rate 1.0`（1× 速度、无 RTF 抖动）。
- 结果：**仍发散**（真值 y≈3，前端 y≈27→44 m）⇒ **不是 RTF/时钟节奏问题**。

### 24.2 录包里的 IMU 间隔

```
间隔(s): min=0.0095 p50=0.0100 p95=0.1245 max=0.2038
间隔 >50 ms 的数量: 147 / 1478（约 10%）
```

10% 的采样间隔超过 50 ms、最大 204 ms（RTF 抖动丢步）。这会被前端判成"采样中断"并重启预积分。

### 24.3 已加 `imu_prep` 空隙插补

`fill_gaps:=true`、`nominal_period:=0.01`：相邻两帧之间按标称周期**线性插补**（时间戳走消息自身时基），
让下游看到连续 IMU 流。

### 24.4 回放仍发散 ⇒ 结论

补空隙**不改变结果**（y 仍 27→44 m）。所以 IMU 间隔不是唯一原因。

剩余可能性收敛到一点：**前端在这份仿真数据上无法稳定初始化/跟踪**，
而每次实验的初值都是几十米（−14.7 / +27 / −11 / +63），说明它的初始化本身就不可信。

**下一步**：直接给前端加逐帧诊断输出（残差/协方差/状态），而不是继续在"仿真参数"上试错。

## 25. 定位验证（2026-11-08）：重定位成功，但输出位姿被"高频预测"带偏

### 25.1 重定位本身是成功的

```
localization state -> NORMAL (recovered via Scan Context + scan-to-map,
  source_frames=2/5, source_span=0.12s, delta_t=0.0337m, delta_r_deg=2.06)
```

⇒ T-DT 的回环重定位机制在我们**自建的 prior**（44 关键帧）上正常工作。

### 25.2 但发布出来的位姿是错的

| 话题 | 来源 | 实测 |
|---|---|---|
| `/frontend/frame` | 前端**后验**（图优化用） | ✅ x 每帧 +0.05 m、y≈0.002 |
| `/frontend/odometry` | 前端**高频预测**（IMU 外推） | ❌ y 27 → 44 m |
| `/localization/odometry`、`~/localization_pose` | 后端**融合输出**（跟随预测） | ❌ y 33 → 56 m |

⇒ 图（后验）是对的，**预测在漂**，而后端对外位姿跟着预测走。

### 25.3 已确认的配置误用（已修）

`common.body.rotation` 曾是**真机斜装**外参（roll −14.94° / pitch 0.523° / yaw −150.6°），
而仿真 `robot:=chassis` 的雷达是**平装**（URDF rpy=0）⇒ 位姿被整体转 150°。
改为单位外参后，建图关键帧跨度从 0.2 m 恢复到 2.24 m。

### 25.4 待解决

前端 `OutputPredictor`（`prediction.cpp:30-45`）用 IMU 在帧间外推；
我们的 IMU 有 ~0.08 m/s² 的残余偏置，而预测只在收到后验时重置，
导致输出位姿单调漂移。**下一步**：查它的 `gravity_` 初值与偏置估计，或让后端只用图位姿对外发布。

## 26. 预测层漂移的量化（2026-11-08）

### 26.1 观测

`/frontend/frame`（后验）正确：x 每帧 +0.05 m、y≈0.002；
`/frontend/odometry`（预测）**持续**漂：y 从 41 → 60 m，增量恒定 ⇒ 等效**横向加速度 ≈ 0.9 m/s²**。

### 26.2 与重力失配的对应关系

`g·sin(θ) = 0.9` ⇒ **θ ≈ 5.3°**。
即前端认为的"重力方向"与我们的 IMU 输出的重力方向差约 5°，
预测器每步把 `g·sinθ` 当横向加速度积分。

### 26.3 实测到的帧偏差

| 量 | 真值（`/odom_ground_truth`） | 仿真 IMU 的 `orientation` |
|---|---|---|
| roll | −1.2° | −1.5° |
| **yaw** | **−0.7°** | **+3.6°** |

⇒ 仿真里 `imu_link` 的实际朝向与 `base_link`（真值）**差 ~4°**，
而前端按 `body_from_lidar = 单位阵` 把 IMU 当成 body/雷达同向 ⇒ 这 4~5° 的不一致是预测漂移的直接来源。

### 26.4 结论（回答"哪里没做对"）

1. **已确认并用错的**：后端 `common.body.rotation` 用了真机斜装（−14.94°/−150.6°）而仿真雷达平装 ⇒ 已改单位阵；
2. **仍需处理的**：仿真里 `imu_link` 与 `livox_frame`/`base_link` 的**朝向不一致（~4~5°）**，
   前端的外参是单位阵 ⇒ 重力/惯性方向被用错 ⇒ 预测层恒定横漂。

**修法**：把 IMU 数据旋转到**雷达/body 系**（或给前端 `body_from_lidar` 填真实旋转），使二者同向。

## 27. 逐项证伪（2026-11-08 深夜）

### 27.1 帧朝向假设被证伪

同一时刻实测（启动 45 s 后、静止）：

| 来源 | roll | pitch | yaw |
|---|---|---|---|
| `/livox/imu` 的 `orientation` | −1.50° | +0.02° | **+3.25°** |
| `/odom_ground_truth`（轮式） | −1.50° | +0.01° | **+3.22°** |

⇒ 两者一致（差 0.03°）。**IMU 与车体没有 4° 偏差**，之前那组"3.6° vs −0.7°"是**不同 run 的数据混比**。

### 27.2 后验（frame）完全正确

```
seq=2 twist=(+0.407,+0.002,-0.004)
seq=8 twist=(+0.404,-0.002,-0.000)
...
```

⇒ 后验的**位置与速度都对**（0.4 m/s 前进、横向≈0）。

### 27.3 IMU 流在**仿真时间**上是完美的

从录包解 `header.stamp`：

```
间隔(sim): n=1478 min=0.0100 p50=0.0100 p95=0.0100 max=0.0100
>0.05 s 的比例: 0.0%
```

⇒ 100 Hz 严格等间隔，**没有空隙**（之前用记录时间算出来的"10% >50 ms"是墙钟抖动，不是仿真时基）。

### 27.4 所以问题精确落在 `OutputPredictor`

- `odometry_`（后验）与 `predictor_`（高频输出）**用的是同一个** `ErrorStateFilter::predict(left,right,gravity,noise)`；
- 后验正确、预测以 ≈0.9 m/s² 横漂 ⇒ 差别只能在 `prediction.cpp:13/37`：
  `predictor_.predict(imus_.at(predictor_.state().time), sample, gravity_, noise_)` 与 `reset()` 里
  `Imu left = imus_.at(posterior.state.time)` 的**时间基准/取样本方式**。

**下一步（精确、有界）**：在 `prediction.cpp` 的 `reset()`/`addImu()` 打印 `left.time`、`sample.time`、`dt`、`gravity_`，
跑一次回放即可判断是"dt 错"还是"gravity 错"，不再扩大范围。

## 28. 根因确认：**我们喂得比前端处理得快**（2026-11-08 深夜，最终结论）

### 28.1 证��：后验落后 IMU 5 秒

在 `prediction.cpp` 里打印后验时刻与 IMU 最新样本的差（`lag`）：

```
reset pos=(0.95,0,0) vel=(0.39,0.01,0) samples=515 lag=5.013s
reset pos=(1.19,0,0) vel=(0.40,0.00,-0.02) samples=535 lag=5.213s
```

后验**落后 5 秒**，而 `OutputPredictor` 要把这 5 秒的 IMU 全部重放一遍 ⇒
任何 0.9 m/s² 的残余都变成 20+ 米的漂移 ⇒ 之前所有"发散"现象。

### 28.2 为什么会落后：我把队列开到 20 万

我为了消除 `queue capacity exceeded`，把 `maximum_queue_events` 设成 **200000**、
`maximum_buffered_points` 设成 **20000000** ⇒ 队列**无限增长**，
前端不是"丢帧"，而是"永远在追赶几秒前的数据"。

### 28.3 验证（三组）

| 条件 | lag | 结果 |
|---|---|---|
| 回放 rate 1.0 + 队列 200000 | **5.0 s** | ODOM y 27→44 m ❌ |
| 回放 rate 0.4 + 队列 200000 | **1.1~1.7 s** | ODOM ≈ FRAME ✅ |
| 回放 rate 1.0 + 队列 256 | — | `queue capacity exceeded`（处理不过来） |
| **实机仿真 RTU 250 + 队列 8192** | **0.13~0.73 s** | **FRAME=(2.14,−0.02) ODOM=(2.30,−0.00) ✅** |

### 28.4 结论（"哪里没做对"）

> **不是 T-DT 的问题，是我把队列开太大 + 让仿真跑得比前端的处理速度更快。**
> 前端每帧要 ~0.3~0.5 s，而我们按 10 Hz 灌入 ⇒ 队列积压 ⇒ 后验落后 ⇒ 高频预测跨 5 s 外推 ⇒ 漂移。

**正确做法**：
- `maximum_queue_events: 8192`（默认量级，不再无限增长）；
- 让仿真节奏匹配前端吞吐：`max_step_size 0.001` + `real_time_update_rate 250`（RTF≈0.2，雷达 ~2 Hz 墙钟）。

## 29. 端到端验证通过（2026-11-08，最终）

用录好的 32 s 直线行驶 bag（0.4× 回放），两阶段跑通：

### 29.1 建图 + 保存

```
runtime_width=6314
save_map -> success=True
关键帧 17; tx[0.80,4.55] ty[0.00,0.06]   (跨度 3.7 x 0.1 m，直线行驶符合预期)
Wrote 17 Scan Context descriptors
```

### 29.2 定位（用自建 prior）

```
定位 t=24.1s truth=(6.60,2.82) zlio=(0.67,-0.02)
定位 t=32.1s truth=(7.20,2.88) zlio=(1.28, 0.01)
定位 t=40.1s truth=(7.79,2.94) zlio=(1.88, 0.01)
定位 t=48.1s truth=(8.39,3.01) zlio=(2.48, 0.01)
定位 t=56.1s truth=(8.98,3.07) zlio=(3.07, 0.00)
localization state -> NORMAL
```

**把真值减去起点 (5.90, 2.74)** 后逐点对比：

| 时刻 | 真值−起点 | zlio | 差 |
|---|---|---|---|
| 24 s | 0.70 | 0.67 | 0.03 |
| 32 s | 1.30 | 1.28 | 0.02 |
| 40 s | 1.89 | 1.88 | 0.01 |
| 48 s | 2.49 | 2.48 | 0.01 |
| 56 s | 3.08 | 3.07 | 0.01 |

⇒ **误差 1~3 cm**，横向 y 稳定在 ±0.02 m。

### 29.3 可用配置（建全场时照这个来）

```bash
# 世界物理：max_step_size 0.001、real_time_update_rate 250（RTF≈0.2）
# 前端：队列必须有界
-p maximum_queue_events:=8192 -p maximum_buffered_points:=2000000
# 前端 IMU 用预处理后的水平系
-p imu_topic:=/livox/imu_level     # 先跑 ros2 run rm_zlio_bridge imu_prep
# 后端：tools/configs/zlio_backend_mapping_save.yaml（uwb 关、body 外参单位阵）
```

## 30. 运动时跳变的根因：没有逐点帧内时间（2026-11-08 收尾）

### 30.1 现象

用户加速后（seq≈355）后验突然从 (−0.62, 2.94) 跳到 (−3.00, 5.95, **z=0.42**)，
同时 `cloud` 降到 1451（平时 ~2100）、`matches` 掉到 1661 ⇒ **跟踪丢失**。

### 30.2 根因：`offset_time` 全 0 ⇒ 扫描时长 0

之前因为"timebase 用的是帧尾"这个 bug，开 `LIVOX_INTRA_FRAME_TIME` 会让帧尾跑到未来，
后端判 `invalid ZLIO observation`。**§26 修好时间基后（timebase = 帧首）**，这个开关才是可用的。

### 30.3 验证（0.6 m/s，比用户更快，还有转向）

```
seq=30 pos=(+1.29,+0.03,+0.01) cloud=2279 jump=0.06m
seq=35 pos=(+1.59,+0.03,+0.01) cloud=2231 jump=0.06m
seq=42 pos=(+2.01,+0.03,+0.00) cloud=2349 jump=0.06m
```

- 每帧前进 0.06 m、y 稳定 0.03、**无跳变**；
- 后端统计：`invalid ZLIO=0`、`cov rejected=0`、`epoch=0`、`interrupt=0`；
- `runtime_width=5887`、`save_map success`。

### 30.4 最终配置（唯一需要记的一条）

```bash
# 终端 A（仿真）起之前：
export LIVOX_INTRA_FRAME_TIME=1
```

其余与 §29 相同（队列 8192/2M、`imu_topic:=/livox/imu_level`、物理 1 ms/250、后端 config 已改）。

## 31. 全场先验建成 + 定位复测（2026-11-08 凌晨）

### 31.1 全场先验（用户实跑，358 关键帧）

```
runtime_map width = 52296
save_map -> success=True
keyframes = 356
跨度 = 26.8 x 14.2 m          ← 与场地尺寸吻合
map.pcd 837 KB / sc_descriptors.bin 11.5 MB
```

注意：那一轮**没有** `LIVOX_INTRA_FRAME_TIME=1`（用户终端里是 `unset`）。

### 31.2 用这份 prior 定位

- 状态：`NORMAL`（中途出现过一次 `OBS` 后恢复）；
- 位姿起点 ≈ (1.0, 0.1)，在 zlio 帧原点附近 ✓（不再是几十米）；
- **但运动中 x 跟踪偏低**：真值 Δ(1.33, 0.87) m，zlio Δ(0.29, 0.77) m。

### 31.3 判断

1. 先验是**没有帧内时间**建出来的，且当时驾驶较快 ⇒ 先验本身可能带轻微形变；
2. 本次复测扫描带帧内时间，与先验的点时刻语义**不一致**；
3. 运动段还伴随 `OBS` ⇒ 匹配约束偏弱。

**结论**：定位链已通（能 NORMAL、位姿在原点附近），但**动态精度**还要靠一份"带帧内时间、慢速匀速"建的先验来验证。

## 32. 更正：现有全场先验的定位精度其实很好（2026-11-08）

§31.2 里"x 只走了 0.29 m vs 真值 1.33 m"是**我采样方式的问题**（那段是"直行+转向"混合，
且每 8 s 才采一次，中间还出现 `OBS`）。重新做**纯直线、每 5 s 采样**的测量：

```
t=15.0s truthΔ=(0.54,0.04) zlioΔ=(0.62,0.07)  比例=1.17
t=20.0s truthΔ=(0.83,0.08) zlioΔ=(0.90,0.10)  比例=1.08
t=25.1s truthΔ=(1.10,0.11) zlioΔ=(1.19,0.13)  比例=1.08
t=30.1s truthΔ=(1.40,0.14) zlioΔ=(1.48,0.16)  比例=1.06
t=35.1s truthΔ=(1.69,0.17) zlioΔ=(1.78,0.19)  比例=1.05
t=40.2s truthΔ=(1.96,0.19) zlioΔ=(2.07,0.22)  比例=1.06
t=45.2s truthΔ=(2.26,0.22) zlioΔ=(2.35,0.23)  比例=1.04
```

⇒ **位移比例 1.04、横向误差 ~1 cm**（前 10 s 的 1.46 是起步瞬态）。

**结论**：现有 356 关键帧的全场先验**可直接使用，不必重建**；
之前"精度差很多"的判断是我错误采样造成的，特此更正。

## 33. 端到端接入（2026-11-08 04:xx）

### 33.1 已接进 `bringup_zlio.launch.py`

| 改动 | 内容 |
|---|---|
| 新增节点 | `imu_prep`（重力对齐 + 补协方差） |
| 前端 | `imu_topic:=/livox/imu_level`、队列 **8192 / 2M**、`imu_history_seconds:=600` |
| 后端 | `mode:=localization` + `prior_map:=<bundle>` + 我们的 `backend_config` |
| 仿真 | `SetEnvironmentVariable(LIVOX_INTRA_FRAME_TIME=1)` |
| 踩坑 | **不能**传 `localization:=''`：传空串会让 nav2 整栈不启动（实测 vs 不传对比） |

新增 launch 参数：`prior_map`、`backend_config`、`livox_intra_frame_time`。

### 33.2 已跑通的部分

```
节点: /backend_node /imu_prep /zlio_node /zlio_tf /mecanum_controller
定位: localization state -> NORMAL
TF  : map→base_link = (-0.013, 0.074, 0.009)      ← 与出生点一致
后端: prior scan-to-map factor accepted
       rmse=0.0522, support=655/799, sectors=35, rank=6/6, observable=true
       delta_t=0.0027 m, delta_r_deg=0.023       ← 先验配准稳定在 3 mm / 0.02°
```

⇒ **zlio 定位 + 先验配准 + TF 桥已经端到端可用**。

### 33.3 未解决：这个 launch 里 nav2 不起

- 用 `bringup_sim.launch.py mode:=nav planner:=tdt nav:=rm_tdt_local` **直接起**：nav2 节点正常（controller_server / planner_server / bt_navigator …）；
- 用 `bringup_zlio.launch.py`（include 同一份 sim launch）：**300 s 后 nav2 依然是 0 个**，日志里 15 个进程全是仿真+zlio；
- 因此 `navigate_to_pose` 不可用，`tdt_goal.py --send` 报 "服务不可用"。

**下一步**：二分定位——把 `bringup_zlio` 里的附加参数（`prior_map`/`backend_config`/`SetEnvironmentVariable`/额外节点）逐个去掉，找出压掉 nav2 include 的那一项；
或退一步：zlio 栈单独起，nav2 用 `bringup_rm_navigation.py` 单独起。

---

## §B 逐环节 / 逐步骤对照


### 来源：`whole_chain_vs_tdt.md` —— 整链路讲解（我们的实现 vs T-DT 原版）


> 本文是「我们这套导航到底怎么跑、和 T-DT 差在哪」的完整说明。
> 所有数字来自本仓库实测日志，参数来自 `src/nav/rm_navigation/rm_navigation/params/`。

---

## 0. 一句话总览

```
zlio（位姿 + 世界系点云）
   ↓
terrain_layer（3D 体素 → 逐柱语义 → /plan_mask）
   ↓
TdtPlanner（我们适配 T-DT 的算法）
   ├─ L1 静态骨架（只在 /map 上 A*，低频缓存）
   ├─ L2 局部窗口（/map ∪ mask，8 m，A* + SDF + 走廊 + MinSnap）
   └─ 零容差门控（穿墙即拒绝/回退）
   ↓ /plan（nav2 Path）+ /trajectory_plan（x/y/vx/vy）
TdtLocalController（我们适配 T-DT 的 Kastar）
   ├─ 整图栅格（/map ∪ mask）+ 2D ESDF
   ├─ 弧长跟踪 v(s) + 前视 4 m
   └─ Kastar 采样 → 车体速度
   ↓ /cmd_vel_nav
velocity_smoother [3,3,3] → /cmd_vel → fake_vel_transform → 底盘
```

**算法内核 100% 来自 T-DT**（`third_party/tdt-nav-kit` 一行未改，MIT）。
我们写的是**两段适配层**：`planner.cpp`（519 行）+ `controller.cpp`（482 行），外加一个
**T-DT 和 zlio 都没有的中间层**：`terrain_layer`（3D 障碍语义）。

---

## 1. 输入：地图

| | T-DT 原版 | 我们 |
|---|---|---|
| 静态图 | 自带 PNG（`images/rmuc2025.png`） | `RMUC2027_v2_thick1.pgm`（手画 + 墙加粗 1 格） |
| 分辨率 | 0.05 m | 0.05 m |
| 原点 | **(0,0)**（关键假设） | (−19.9, −8.89)，但我们**转成局部坐标**再喂 |
| 墙厚 | **中位 12 格（60 cm）** | 原图 2 格 → thick1 后 **2.8 格** |
| 动态障碍 | **没有**（demo 用合成 mask 测耗时） | `terrain_layer` 实时生成 |

**为什么墙厚重要**：T-DT 的碰撞判定 `lineInObsticle` 在**非严格模式**下
「允许线段穿过 ≤2 个障碍栅格」（官方 `doc/Usage.md` 原话）。
它的墙 12 格厚 ⇒ 容 2 格无害；我们的墙 2 格厚 ⇒ **容 2 格 = 整堵墙被放行**。

---

## 2. 坐标系：最容易翻车的地方

T-DT 内部**有三处 origin 约定，彼此矛盾**：

| 函数 | 约定 |
|---|---|
| `SfcSquare::valueAt / _getBound / _getBoundSquare2` | `(p + origin) / res` |
| `SfcSquare::isOutside`、`MinimumSnap::lineInObsticle` | `(p − origin) / res` |

上游 demo 的 `origin` 就是 `(0,0)`，所以这个矛盾**永远不会暴露**。

**我们踩的坑**：一开始按「让它加 origin」去迁就，结果 `isOutside` 全部判成出界
⇒ 每段都判碰撞 ⇒ 走廊盒子算出 NaN ⇒ 第二次 QP 失败 ⇒ **`snap` 永远是 0**。

**修法**（`0664b08`）：**全程用地图局部坐标**（origin=0），
起终点、A\* 路径、走廊、输出轨迹都在局部系里算，最后整体 `+origin` 平移回世界系。

```
世界坐标 ──(−origin)──► 局部坐标 ──T-DT 全部内部运算──► 局部 ──(+origin)──► 世界
```

---

## 3. 全局搜索：A\*（`YAstar`）

T-DT 的 A\* 就是「**带人工势场的 8 邻域 A\***」（`src/YAstar/yastar.hpp` 注释原文）：

| 项 | 实现 |
|---|---|
| 邻域 | 8 邻域，对角 ×1.414 |
| 启发 | `hypot(dx,dy)`（按格） |
| 单格代价 | **`1 + 0.5/(0.1+d)`**，`d` = 到最近障碍的距离（SDF） |
| 势场作用 | 离墙越近代价越高 ⇒ 路径被推向中间 |
| Mask | `setMaskMapTimed(map,name,duration,maxCount,value)`，按层过期 |

**注意**：势场只是「加价」，**不是禁止**。A\* 仍可能给出离墙 5 cm 的路径。

### 我们的改动：两级规划（`6622874`）

T-DT 原版是「一次从起点搜到终点」。我们改成分两层：

```
L1 骨架：只用 /map 的 A*，缓存
         · 目标移动 >0.3 m 或超过 5 s 才重算
         · 实测：366 点 / 21.2 m，一次约 5~10 ms
L2 局部：从当前位姿搜到「骨架上 s_now + 8 m 处」
         · 吃 /map ∪ mask（实时）
         · 5~10 点控制点 → MinSnap
输出：   /plan = 局部轨迹 + 骨架尾段
```

**为什么要拼尾段**：nav2 的 `PlanGoalChecker` 取 `/plan` 的**末点**当目标。
若只发 8 m 局部轨迹，车到 8 m 处就会被判「到达」。

**为什么这样不抖**：动态障碍只影响近端 8 m，骨架上远端的走向不变。

---

## 4. 路径简化：DP + 弦优化

T-DT：`simplifyPath(path, 0.1)` + `simplifyPathHypot`（递归分裂，判据是
`maxDistance > 阈值 || lineInObsticle(p0,pN)`）。

**它的坑**：`lineInObsticle` 容 2 格 ⇒ **简化后的长弦可以直接切过薄墙**。

**我们**：
1. 保留 DP（`7f218e3` 修了一个我自己的回归：原来 `<3 点就退回` 会把成功的 2 点简化丢掉，
   导致开阔地也生成 55 个 5 cm 控制点）；
2. **简化后跑零容差校验**，失败退回 A\* 原始栅格路径。

---

## 5. 时间分配：梯形

T-DT：`trapezoidalTimeAllocation(path, vmax, amax)`，每段
`distThs = 0.5·v²/a`；短段 `2√(d/a)`，长段 `(d−distThs)/v + 2√(distThs/a)`。

**这解释了速度差异的直觉**：总时间 ∝ √N（段数），所以控制点越碎，总时长越长。
上游 10 点跑 34 m ≈ 1.0 m/s；我们曾经 55 点跑 3.4 m ≈ 0.19 m/s。

---

## 6. 平滑：SFC 走廊 + MinimumSnap

T-DT 的后端：

```
控制点 → _getBoundSquare2 生成正方形走廊（range 2.5）
       → MinSnap（order 6, maxDx 3, dt 0.1）
       → OSQP 求解（eps 1e-8, max_iter 20000, time_limit 0.1 s）
       → 碰撞检查不通过 ⇒ 交替迭代（偶数轮插中点+新盒子，奇数轮把出问题的走廊缩半）
```

**它的三个隐含假设**：
1. 走廊盒子**只检查扩张方向**，盒子**内部可以有墙** ⇒ QP 可合法穿墙；
2. 碰撞检查**容 2 格**（非严格）；
3. `SolveOutput` **没有时间戳**，只有 `path` 和 `success`。

**我们**：
- 走廊/平滑**吃同一份 `a_occ`/`sfc_map`**（含 mask，见 `6622874` 之前的步骤 0）；
- **零容差门控**（`04373d8`）：每条相邻线段用 Bresenham 逐格查障碍，命中 1 格即拒绝；
- `snap=1` 且门控通过才用平滑轨迹，否则回退 A\* 简化路径（`e312e8e`）。

---

## 7. 控制器：Kastar

T-DT 的局部规划器 `KinodynamicAstar`：
状态 4 维 `(x, y, vx, vy)`，采样 `sampleTime`，`maxVel/maxAcc` 分轴约束，
在栅格上评估候选轨迹的碰撞与代价。

**它的输入是「带时间戳的轨迹」**（它自己的 demo 用 `SolveOutput` 直接回放）。

**我们的问题**：nav2 交给控制器的是 `Path`（只有 x,y），**没有时间**。
所以我们：
1. 自定义 `TrajectoryPlan` 消息带 `x/y/vx/vy/t`；
2. 控制器**重建弧长** `s[i]` 与速度 `v(s)`；
3. 前视点 = `s_now + 4.0 m`；
4. 用 Kastar 在**整图栅格**（`/map ∪ mask`）上搜索。

### ESDF clearance（`6xxx`）

在整图栅格上做一次 **2D 倒角距离变换**（两遍扫描，~0.5 ms，只在栅格重建时算），
得到每格到最近障碍的距离。当参考点 clearance < **0.20 m** 时，
沿距离场梯度把参考点推出去（最多 35 cm，且只在变好时接受）。

**为什么不改 Kastar**：它是 T-DT 代码，没有代价注入接口；
但 **Kastar 跟踪参考点**，所以推参考 = 不贴墙。

---

## 8. 动态障碍：T-DT 和 zlio 都没有的一层

| | T-DT | zlio | 我们 |
|---|---|---|---|
| 感知 | **无** | **无**（只有点云） | `terrain_layer` |
| 输入 | — | — | `/localization/lio_world_cloud`（同源） |
| 表示 | 2D mask（接口） | — | **3D 体素 0.10 m**（280×157×17） |
| 更新 | — | — | hit +4 / raycast miss −1 / 每帧衰减 −1 |
| 输出 | — | — | `/plan_mask`（只把 OCCUPIED 当障碍）、`/terrain_class` |
| 分类 | — | — | FREE / PASSABLE / OCCUPIED / PASS_UNDER |

判据（**与 BIT ProjectionLayer 同参数**）：

| 量 | 值 | 含义 |
|---|---|---|
| `scan_z_min / max` | −0.20 / 1.50 | map 系绝对高度带 |
| `surface_height_delta_max` | 0.10 | 柱内高度差 ≤ 此 ⇒ **坡/缓台，可走** |
| `wall_height_delta_min` | 0.20 | ≥ 此 ⇒ 墙 |
| `tunnel_height_delta_min` | 0.25 | 净空下界 |
| `tunnel_height_delta_max` | 0.40 | 板厚上界 |
| 额外保护 | 下方**≥2 层被射线清空** | 否则是遮挡（远处墙只扫到顶） |

实测：`点 2025 占有 477 坡 614 可钻 23 | 耗时 1.3 ms`。

---

## 9. 参数对照

| 参数 | T-DT demo | 我们 | 说明 |
|---|---|---|---|
| `max_speed` | 8.0 | **3.0** | 底盘上限 2.32 m/s（40 rad/s × 0.058 m） |
| `max_acc` | 2.0 | **3.0** | — |
| `max_corridor_range` | 2.5 | 2.5 | 相同 |
| `corridor_shrink` | 0.0 | 0.0 | 相同 |
| `min_snap_order` | 6 | 6 | 相同 |
| `max_dx` | 3 | 3 | 相同 |
| `iter_time_limit` | 10.0 | 10.0 | 相同 |
| `collision_check_iter` | 6 | 6 | 相同 |
| `strict_collision` | false | **false**（+ 外挂零容差门控） | 我们不改它内部，外挂 |
| `simplify` | 0.1 | 0.1 | 相同 |
| 控制频率 | 100 Hz（demo） | **30 Hz**（nav2） | — |
| `lookahead` | 6.0 m（MINCO 风格） | **4.0 m** | — |

**12 个采样/时间参数里 11 个与上游一致**，只有速度和加速度按底盘改。

---

## 10. 我们比 T-DT 强/弱的地方

### 强

| 项 | 说明 |
|---|---|
| **不穿墙** | T-DT 允许穿 2 格；我们零容差门控 + 加粗墙 |
| **动态障碍** | T-DT 完全没有；我们有 3D 语义层 |
| **两级规划** | 全局稳定 + 局部实时，T-DT 是单级 |
| **地形语义** | 坡可走、横杆可钻；T-DT 只有二值 |
| **ESDF clearance** | 不贴墙；T-DT 的势场只在 A\* 里 |

### 弱

| 项 | 说明 |
|---|---|
| **没有硬性安全距离** | 我们的门控只判「是否踩墙」，不判「离墙多远」；ESDF 只推参考点 |
| **控制器不如 MINCO/MPC** | Kastar 是采样，精度与平滑度低于 MINCO + MPC |
| **没有行为树** | 卡住后没有系统化恢复（BIT 有 FSM + ESDF 梯度脱困） |
| **定位依赖 zlio 重定位** | 仿真里会跳变（10 m / 172°），一跳规划就废 |

---

## 11. 调试史（为什么「快」不在开源）

| 现象 | 根因 | 归属 |
|---|---|---|
| `snap=0` 每次 | T-DT 三处 origin 约定冲突 | 边界假设 |
| 整图变障碍 | OccupancyGrid `0=自由` vs PGM `0=障碍` | 边界假设 |
| 路径穿墙 | 2 格容差 × 我们的薄墙 | 边界假设 |
| 车背离目标 | 速度被旋转两次 | 适配 bug |
| 原地画龙 | 纯 P 朝向 + 0.2 s 延迟 | 控制器 |
| 车不动 | 路径穿墙的下游症状 | 诊断错误 |
| 路径抖动 | 单级规划 + 动态障碍 | 架构 |
| terrain 274 ms | TF 查询阻塞（50 ms 超时 × 3） | 实现细节 |

**共同点：全部在「边界假设」上，没有一条是算法本身不行。**

---

## 12. 还剩什么

| 项 | 状态 | 优先级 |
|---|---|---|
| ESDF 脱困（沿梯度退让） | 未做 | 遇到再补 |
| 定位重定位跳变 | 未处理（用户决定） | 中 |
| 走廊盒子内嵌墙 | 未修（用门控兜住） | 低 |
| 尾段 A\* 对角线切角 | 未修（5 cm 量级） | 低 |
| 多目标端到端跑图验收 | **待做** | **高** |



### 来源：`tdt_full_chain_vs_tdt.md` —— 当前完整链路 vs T-DT（含 z-localization）


> 更新时间：2026-11-08（本文件只写**当前实际在跑**的东西）
> 标注：**【实测】** = 今天跑出来的数据；**【代码】** = 读源码确认；**【推断】** = 未直接验证

## 1. 当前完整数据流

```
Gazebo(RMUC2027) ── /livox/lidar(CustomMsg, 逐点 offset_time) ─┐
                  └ /livox/imu (100 Hz, 协方差全 0)             │
                                                               ▼
                                  rm_zlio_bridge/imu_prep  （我们加的前置处理）
                                    重力对齐 + 补协方差 + 补空隙
                                                               │ /livox/imu_level
                                                               ▼
                        frontend: zlio_node  ← T-DT 原版前端（未改算法）
                            │ /frontend/frame   （后验：位姿+去畸变点云）
                            │ /frontend/odometry（高频预测）
                            ▼
                        backend: backend_node  ← T-DT 原版后端（改配置）
                            │ prior_map = .tmp_cache/prior/runtime_map（zlio 自建先验）
                            │ Scan Context + scan-to-map + 回环
                            ├─ /localization/odometry
                            ├─ /backend_node/localization_pose|state
                            └─ TF zlio_map→base_link（由 rm_zlio_bridge/zlio_tf 发）
                                                               │
                                    static TF: map → zlio_map（实测对齐量 ≈ 0）
                                                               ▼
   /map（手画 PGM，RMUC2027_v2） ──► global_costmap(static_layer)
                                     │
                                     ├─ planner_server + rm_tdt_planner（T-DT A*+MinSnap）
                                     │      → /trajectory_plan（nav_msgs/Path）
                                     └─ controller_server + rm_tdt_local_controller（Kastar 4 m）
                                            → /cmd_vel → fake_vel_transform → /cmd_vel_chassis
```

## 2. 逐环节对比

| # | 环节 | T-DT 原版 | 我们当前 | 差异 | 状态 |
|---|---|---|---|---|---|
| 1 | 雷达输入 | Livox CustomMsg（逐点时间） | 同（仿真开了 `LIVOX_INTRA_FRAME_TIME=1`） | 无 | ✅【实测】 |
| 2 | IMU | 实机 IMU | 仿真 IMU + **imu_prep**（重力对齐/协方差/补空隙） | **多一层前置处理** | ⚠️ 偏差 |
| 3 | 前端 LIO | `zlio_node` | **原版，未改算法** | 无（只用参数：队列 8192/2M、imu_level） | ✅【实测】 |
| 4 | 后端 | `backend_node` + prior | **原版** + 自建 prior（356 关键帧） | 配置改动：`uwb.enabled=false`、`body` 外参单位阵 | ⚠️ 配置偏差 |
| 5 | 定位精度 | —— | 直线 2.26 m 真值 vs 2.35 m 估计（**4%**），横向 ~1 cm | —— | ✅【实测】 |
| 6 | 先验配准 | Scan Context + scan-to-map | 同（RMSE ≈ 5.2 cm，rank 6/6，修正 3 mm/0.02°） | 无 | ✅【实测】 |
| 7 | 全局规划 | `YAstar` → `simplifyPath` → `simplifyPathHypot` → `trapezoidalTimeAllocation` → `MinimumSnap` | **同一套（vendored 到 `rm_tdt_planner/src/tdt/`）** | 补 `SolveOutput.times`、`-origin` 取反、`vmax/amax` 用底盘值 | ✅ 等价（带补丁） |
| 8 | 规划参数 | range 2.5 / shrink 0 / order 6 / maxDx 3 / TL 10 / iter 6 / norm_time | **完全一致** | 无 | ✅【代码】 |
| 9 | 局部控制 | KinodynamicAstar(1.5e5, dt 0.01) + 跟踪律 | **同算法**（4 m 前视、min_local_goal 1.0、k_theta 2.0、vmax 1.5、amax 3.0） | 跟踪律含"采样+2 点前馈+朝向反馈" | ✅ 等价（实现不同） |
| 10 | 地图来源 | 先验 → 栅格 + PCM mask | `/map`（手画 PGM）+ patchwork 实时掩膜 | **地图来源不同** | ⚠️ 偏差【实测对齐量≈0】 |
| 11 | 目标/执行 | 自家执行器（无 nav2） | **nav2**（bt_navigator/planner_server/controller_server） | 外壳不同 | ⚠️ 偏差 |
| 12 | odom_level | 无此概念（或等价） | 这条链用 `local_frame:=map`，**不启用 odom_level** | 按你的要求去掉 | ✅ 有意为之 |
| 13 | 我们的 LIO | 无（用前端） | `lio:=none`（这条链不用 small_point_lio） | 无 | ✅ |
| 14 | 坡度限速 | 无 | `ground_segmentation` 发 nav2 SpeedLimit | **我们额外加的** | ⚠️ 偏差 |

## 3. 今天实测的验收数据

| 项 | 数值 |
|---|---|
| 先验 | 356 关键帧 / 跨度 26.8 × 14.2 m / map.pcd 837 KB / 描述子 11.5 MB |
| 定位状态 | `NORMAL`（Scan Context 重定位，修正 3.4 cm / 2.1°） |
| 沿先验配准 | RMSE 5.2 cm，support ~85%，rank 6/6，observable，每次修正 1~7 mm |
| 前端频率 | 26~40 Hz（RTF 0.2 下） |
| 规划 | 71 点 / 15 走廊盒 / 2.65 s（早期实测） |
| 导航 | 目标 (2.0, −2.5) → **SUCCEEDED 51.1 s**，终点距目标 **0.26 m** |

## 4. 还差的（按影响排序）

| # | 差异 | 影响 | 可选做法 |
|---|---|---|---|
| 1 | 地图来源：手画 PGM vs 先验投影 | 若两者几何不一致，全局与局部会打架（今天已证明**这张旧图与先验对齐**，所以当前可用） | 需要时用 `pcd_to_grid_map` 生成同源图 |
| 2 | nav2 外壳 | 多了 bt_navigator/lifecycle，启动有竞态（zlio 与 nav2 抢启动） | 已用 60 s 延迟规避 |
| 3 | imu_prep 前置 | 仿真特有；真机上不需要 | 真机时去掉 |
| 4 | 坡度限速 | 比 T-DT 多一层保守 | 可关 |
| 5 | 控制器跟踪律 | 与 T-DT 不完全逐行相同 | 需要时可继续对齐 |

## 5. 结论

- **感知→定位→先验配准**：T-DT 原版组件，未改算法（只改配置 + 多一层仿真 IMU 预处理）；
- **全局规划**：T-DT 的 A*+MinSnap **同一份代码**，参数一致；
- **局部控制**：同算法（Kastar），实现细节不同；
- **执行外壳**：nav2（T-DT 是自家执行器）——这是最大结构差异；
- **当前可用性**：建图/保存/定位/规划/跟踪/到达 **全链路已实测跑通**。



### 来源：`tdt_side_by_side.md` —— 逐步骤代码对照


> 生成时间：2026-11-08。左边是 `third_party/tdt-nav-kit`（上游原版），右边是 `src/nav/rm_tdt_planner` / `src/nav/rm_tdt_local_controller`（我们）。
> 所有"差异行数"都是 `diff` 实测。

## 0. 先看结论：算法核心**逐字节相同**

| 文件 | 上游行数 | 与上游的差异 |
|---|---|---|
| `yastar.cpp` | 545 | **0 行** |
| `kinodynamicAstar.cpp` | 460 | **0 行** |
| `sfcSquare.cpp` | 221 | **0 行** |
| `minimumSnap.cpp` | 1028 | **2 行**（仅 `SolveOutput` 加 `times` 字段） |

⇒ **A*、走廊、MinSnap、Kastar 的算法本体没有被改过**（除了那 2 行数据字段）。
真正的差异全部在**驱动层**：上游是 `main.cpp`（一个 478 行的**benchmark 程序**，读 PNG、跑 1000 次随机起终点、`cv::imshow`），
我们是 **nav2 插件**（`planner_server` / `controller_server` 里跑）。

## 1. 全局规划：逐步对照

| 步骤 | T-DT `main.cpp::test()` | 我们 `planner.cpp::plan()` |
|---|---|---|
| 取图 | `cv::imread("../images/rmuc2025.png")` | 订阅 `/map`（`nav_msgs/OccupancyGrid`，transient local）+ `/plan_mask` |
| 建图对象 | `YAstar astar(cols, rows, 0.05f, 0.f, 0.f)` | `YAstar astar(W, H, res, ox, oy)` |
| 设地图 | `astar.setMap(cols, rows, map.data)` | `astar.setMap(W, H, data, res, ox, oy)`（**边界 pad 2 格**） |
| Mask | `astar.setMaskNumLayers(4)`（注释"256层"） | `astar.setMaskNumLayers(4)` + 用 `/plan_mask` 动态层 |
| 代价场 | `setCostField(1.f, [{return 0.5f/(0.1f+x);})` + `initCostMap()` | **同一行代码** |
| 搜索 | `astar.search(start, end)` | `astar.search(start, end)` |
| 空路径 | `if(path.empty()) return 0;` | 返回失败（`nav2` 报 "A* 没找到路径"） |
| 简化 | `simplifyPath(path, 0.1f)` → `simplifyPathHypot(path)` | 同（我们**多点 <3 时跳过**，防止压成 2 点后 MinSnap 被跳过） |
| MinSnap 配置 | `setOrder(6) setMaxDx(3) setDt(0.1) setMap(map,0.05,0,0) setTL(10)` | 同（`setMap(sfc_map, res, -ox, -oy)`，**origin 取反**） |
| SolveInput | `setCollisionCheckIter(6) setMaxSpeed(3) setMaxAcc(1) setMaxCorridorRange(2.5) setCorridorShrink(0) setNormTime(true) setBackend(OSQPCorridor) setInitVel({0,0})` | 同结构；**速度/加速度用底盘值 1.5 / 2.0** |
| 求解 | `result = minimumSnap.solve(input)` | 同 |
| 输出 | 画图 + 检查点是否在图内（**唯一的验收标准**） | 发 `/trajectory_plan`（`trajectory_plan_msgs`）+ `nav_msgs/Path`；走廊来自 `result.corridor` |

**结论**：全局规划的**调用序列 1:1 相同**，只有三处刻意的差别（见 §4）。

## 2. 局部控制：逐步对照

| 步骤 | T-DT `main.cpp::testKastar()` | 我们 `controller.cpp::computeVelocityCommands()` |
|---|---|---|
| 地图 | 读整张 PNG | 以机器人为中心建 **±16 m（32 m 见方）窗口栅格**（`/map` + `/plan_mask` 叠加） |
| 起点/终点 | 硬编码 `(25.27,14.15) → (1.47,9.17)`（整图） | 起点 = 当前位姿；终点 = 沿全局路径前进 **lookahead=4 m** 处的点 |
| 搜索对象 | `KinodynamicAstar(cols, rows, 0.05, 0, 0)` | `KinodynamicAstar(gw, gh, gres, gox, goy)`（窗口） |
| 配置 | `maxNodes=150000, sampleTime=0.01` | `max_nodes=150000, sample_time=0.01` |
| 速度/加速度 | `config.maxVel/maxAcc`（默认值） | `maxVel=1.5, maxAcc=3.0` |
| 搜索 | `astar.search(start, velocity, acceleration, end)` | `kastar.search(s, v0, {0,0}, g)` |
| 输出 | 只做**验收检查**（误差<1e-6、无碰撞线段、各轴速度/加速度不超限） | 取轨迹前馈 + 位姿反馈出 `/cmd_vel`；失败回退**纯跟踪** |
| 转向 | —— | `w = k_theta * wrap(yaw_ref - yaw)`，`k_theta=2.0` |

**结论**：Kastar 的**调用方式相同**，但"怎么用它的结果"是我们自己写的跟踪律（T-DT 的 `main.cpp` 根本不发速度，它只是 benchmark）。

## 3. 参数总表

| 参数 | T-DT demo | 我们 | 差异原因 |
|---|---|---|---|
| 栅格分辨率 | 0.05 m | 0.05 m | 同 |
| 代价场 | `0.5/(0.1+d)` | 同 | 同 |
| Mask 层数 | 4 | 4 | 同 |
| simplify 阈值 | 0.1 | 0.1 | 同 |
| MinSnap order / maxDx / dt | 6 / 3 / 0.1 | 同 | 同 |
| MinSnap TL / collisionIter | 10 / 6 | 同 | 同 |
| maxCorridorRange / shrink | 2.5 / 0.0 | 同 | 同 |
| normTime / backend | true / OSQPCorridor | 同 | 同 |
| initVel | {0,0} | 同 | 同 |
| **maxSpeed** | **3** | **1.5** | 底盘限速 |
| **maxAcc** | **1** | **2.0** | 底盘加速度 |
| Kastar maxNodes / dt | 150000 / 0.01 | 同 | 同 |
| 全局规划频率 | 无（benchmark） | 2 Hz | nav2 外壳 |

## 4. 三处刻意的代码差别（都在全局规划里）

1. **`SolveOutput` 增加 `times` 字段**（2 行）——上游没把时间轴导出，我们补上用于拼 `TrajectoryPlan`；
2. **`setMap` 的 origin 取反**：`SfcSquare` 内部用 `(p + origin)/res`，而上游 `test()` 传 `0,0` 所以没暴露；我们必须传 `-ox,-oy`；
3. **简化后点数 < 3 就跳过 MinSnap**：上游场地大兴许 2 点，我们短任务会被压成 2 点导致 MinSnap 直接跳过。

## 5. 外壳层的差异（T-DT 没有的东西）

| 项 | T-DT | 我们 |
|---|---|---|
| 运行形态 | 一个 `main()` benchmark | nav2 `planner_server` + `controller_server` 插件 |
| 执行器 | 无 | `bt_navigator`（行为树）+ lifecycle 管理 |
| 目标接口 | `test(start,end)` 硬编码 | `navigate_to_pose` action |
| 地图来源 | `images/rmuc2025.png` | `/map`（手画 PGM）+ `/plan_mask`（patchwork 实时） |
| 位姿来源 | —— | zlio 后端（`/localization/odometry`） |
| 里程计 | —— | zlio 前端 → 后端 → `zlio_tf` |
| 局部障碍 | 地图里的静态格 | STVL 实时体素层 + `/plan_mask` |
| 限速 | 无 | `ground_segmentation` 发 nav2 `SpeedLimit` |
| 可视化 | `cv::imshow` | RViz + `/trajectory_plan` |

## 6. 一句话

> **算法：同一份代码（3 个文件 0 差异，1 个 2 行）。**
> **驱动：上游是 benchmark，我们是 nav2 插件 + zlio 定位。**
> **剩下所有差异都在"外壳"和"参数"上，不在算法里。**



### 来源：`side_by_side_two_columns.md` —— 左右两列对照（逐阶段 + 行号）


> 2026-11-08。左 = `third_party/tdt-nav-kit`；右 = `src/nav/rm_tdt_planner` / `src/nav/rm_tdt_local_controller`。
> 判定：**同** = 同一份代码；**等价** = 调用序列相同、实现不同；**不同** = 行为不同。

## A. 全局规划（14 阶段）

| # | 阶段 | T-DT 上游 | 我们 | 判定 |
|---|---|---|---|---|
| A1 | 读图 | `main.cpp:76` `cv::imread("../images/rmuc2025.png")` | `planner.cpp:60` 订阅 `/map`；`planner.cpp:325` `buildFromOccupancyGrid` / `358` `loadMapFile` | **不同**（缓存 vs 每调一次） |
| A2 | 二值化 | `yastar.cpp:404` `setMap(u_char*)`：`x == 0 ? 1 : 0` | `planner.cpp:340` `v == 0 \|\| v == 100`；`388` `px == 0` | **等价**（现为字面一致） |
| A3 | 边界 | 无（图四周自带黑墙） | `planner.cpp:327` `P = 2`，外扩置障碍，origin −2·res | **不同**（替代黑墙） |
| A4 | 建 A* | `main.cpp:94` `YAstar astar(cols, rows, 0.05f, 0, 0)`；`95` `setMap` | `planner.cpp:138` `setMapping`；`141` `setMap(W,H,res,ox,oy,data)` | 等价 |
| A5 | Mask | `main.cpp:96` `setMaskNumLayers(4)`；**`setMaskMapTimed` 被注释** | `planner.cpp:140` 同；`150-156` 真正调用 `setMaskMapTimed(...,1.0,16,0)` | **不同**（我们启用） |
| A6 | 代价场 | `main.cpp:109` `setCostField(1, 0.5f/(0.1f+x))`；`110` `initCostMap()` | `planner.cpp:157-158` **同一行** | **同** |
| A7 | A* 搜索 | `main.cpp:115` `astar.search(start,end)` | `planner.cpp:160` 同 | **同**（`yastar.cpp` 0 差异） |
| A8 | 坐标 | `main.cpp:88-89` 先翻 Y 再乘 0.05（图像空间） | `planner.cpp:143` 直接 map 帧米制（`OccupancyGrid` 行 0 在下） | **不同**（上游输出镜像） |
| A9 | 简化 | `main.cpp:133-134` `simplifyPath(0.1)` + `simplifyPathHypot` | `planner.cpp:164-165` 同；`170` `<3 点退回原路径` | 等价 + **保护** |
| A10 | 时间分配 | `minimumSnap.cpp:281` `trapezoidalTimeAllocation`（solve 内部自动调） | `planner.cpp:171` 显式调用后 `192` `setTimeAllocated` | 等价 |
| A11 | MinSnap 配置 | `main.cpp:145-149` order6/maxDx3/dt0.1/`setMap(map,0.05,0,0)`/TL10 | `planner.cpp:176-181` 同，但 `setMap(...,-ox,-oy)` | 等价（origin 取反） |
| A12 | SolveInput | `main.cpp:155-165` path/iter6/v3/a1/range2.5/shrink0/norm/OSQP/initVel0 | `planner.cpp:182-191` 同结构；**v1.5/a2.0** | 等价（底盘参数） |
| A13 | 求解+迭代 | `main.cpp:166-168` `minimumSnap.solve(input)` | `planner.cpp:196` 同 | **同**（`minimumSnap.cpp` 2 行差异=加 times） |
| A14 | 输出 | `main.cpp:172-200` 画线/画走廊 + `imshow` | `planner.cpp:236-307` 发 `/trajectory_plan`；`312-318` 返回 `Path` | **不同**（消息 vs 画图） |

## B. 局部控制（6 阶段）

| # | 阶段 | T-DT 上游 | 我们 | 判定 |
|---|---|---|---|---|
| B1 | 地图 | `main.cpp:393` `imread` 整张图；`398` `setMap(map.data)` | `controller.cpp:160` `buildLocalGrid`：**`/map`+`/plan_mask` 裁 ±16 m**（`253-280`） | **不同**（窗口+mask） |
| B2 | 起点 | `main.cpp:401` 硬编码 `(25.27,14.15)`，速度 0 | `controller.cpp:99-113` 当前位姿 + zlio 速度 | **不同** |
| B3 | 终点 | `main.cpp:401` 硬编码 | `controller.cpp:122-152` 沿路径前视 4 m（plan 新鲜时用 plan 点） | **不同** |
| B4 | Kastar 配置 | `main.cpp:397-400` `KinodynamicAstar(cols,rows,0.05,0,0)` + `maxNodes=150000` + `sampleTime=0.01` | `controller.cpp:186-194` 同三项 + `maxVel/maxAcc` | 等价 |
| B5 | 搜索 | `main.cpp:402` `astar.search(start,v,a,end)` | `controller.cpp:197` `kastar.search(s,v0,{0,0},g)` | **同**（`kinodynamicAstar.cpp` 0 差异） |
| B6 | 结果用途 | `main.cpp:404-419` 只做 `passed` 判定（误差/碰撞/超限） | `controller.cpp:198-231` 取"最近样本+2"**速度前馈**+朝向反馈 → `Twist` | **不同**（验收 vs 控车） |
| B7 | 输出 | `main.cpp:428` `cv::imshow` + `return true` | `controller.cpp:224-231` `cmd.twist.linear.x/y`、`angular.z` | **不同** |

## C. 判定汇总

| 判定 | 阶段 |
|---|---|
| **同一份代码（0 差异）** | A6 代价场、A7 A* 搜索、A13 MinSnap 求解与迭代、B5 Kastar 搜索 |
| **等价（调用相同、实现不同）** | A2 二值化、A4 建 A*、A9 简化、A10 时间分配、A11 MinSnap 配置、A12 SolveInput、B4 Kastar 配置 |
| **不同（行为差异）** | A1 读图方式、A3 边界 pad、A5 Mask 启用、A8 坐标空间、A14 输出形态、B1 地图窗口、B2/B3 起终点来源、B6 结果用途、B7 输出 |



### 来源：`planning_full_chain.md` —— 规划 + 控制完整链路逐环节比对


> 2026-11-08。【代码】= 读源码；【实测】= 跑过。

## A. 全局规划：17 个环节，逐个对

| # | 环节 | T-DT `main.cpp::test()` | 我们 `planner.cpp` | 等价? |
|---|---|---|---|---|
| 1 | 地图来源 | `cv::imread("../images/rmuc2025.png")`，**每次调用都重读** | 订阅 `/map`（transient local）**缓存一次**；也支持直接读 yaml+PGM | **不同**（性能） |
| 2 | 分辨率 | 20 px/m（硬编码 `*20` 做像素↔米） | 从 `OccupancyGrid.info.resolution` 读（0.05） | 等价 |
| 3 | 二值化 | `occMap = (pixel == 0) ? 1 : 0`，阈值 `occThs=0.5` ⇒ **只有纯黑算障碍** | `PGM: px < 128`；`OccupancyGrid: v >= 65` ⇒ 黑=障碍，205/254=自由 | **近似等价**（灰度阈值不同） |
| 4 | 边界 | **不 pad**；靠 PNG 自己四周有墙；`_getBoundSquare2` 不查边界 | **外扩 2 格并置为障碍**（因为 `_getBoundSquare2` 不查边界），origin 相应 −2·res | **不同**（我们加了保险） |
| 5 | Y 轴约定 | 图像行 0 在**上**，所以先把 `start.y/end.y` 翻成行号再乘 0.05；**输出路径留在"行坐标"里，不再翻回来**（只画图所以无所谓） | `OccupancyGrid` 行 0 在**下**，直接用 map 帧米制；输出就是真 map 坐标 | **不同**（上游输出是镜像的，我们是真坐标） |
| 6 | origin | 传 `0,0`（世界系=图像原点） | `YAstar.setOriginPos(ox,oy)`；`SfcSquare.setMap(map,res,-ox,-oy)`（**反号**，因为它内部是 `(p+origin)/res`） | 等价（我们多一步换算） |
| 7 | Mask | 有接口，但 `test()` 里**注释掉了没用** | 订阅 `/plan_mask`，`setMaskMapTimed(mm,"dyn",1.0s,16,0)` | **我们才真正用** |
| 8 | Mask 层数 | `setMaskNumLayers(4)` | 同 | 等价 |
| 9 | 代价场 | `setCostField(1, x→0.5/(0.1+x))` + `initCostMap()`（SDF） | 同 | 等价 |
| 10 | A* | 8 邻域、对角×1.414、h=格距欧氏、`g+h` 排队 | 同 | 等价 |
| 11 | 起终点 | 像素索引直接算；越界就"返回起点单点" | 先 TF 到 map/costmap 帧，再算索引；越界 A* 返回单点 ⇒ 我们判 `<2` 报失败 | 等价（错误处理不同） |
| 12 | 简化 | `simplifyPath(0.1)` → `simplifyPathHypot` | 同 | 等价 |
| 13 | 短路径 | 无特殊处理（demo 都是 20 m 级） | 简化后 **<3 点就退回原路径**（否则 MinSnap 被跳过） | **不同**（我们加的） |
| 14 | 时间分配 | `trapezoidalTimeAllocation(path, vmax, amax)` | 同 | 等价 |
| 15 | MinSnap 参数 | order6/maxDx3/dt0.1/TL10/collisionIter6/range2.5/shrink0/normTime true/OSQPCorridor/initVel{0,0} | 同（v/a 用底盘值 1.5/2.0） | 等价 |
| 16 | 走廊 | SFC 方盒 + 交替迭代（加点/收窄），最多 6 轮 | 同（代码 0 差异） | 等价 |
| 17 | 输出 | `SolveOutput{path, corridor}`，**只画图** | `TrajectoryPlan{t,x,y,vx,vy,corridor}` 发 `/trajectory_plan` + `Path` | **不同**（我们导出时间/速度） |

### A 的三处"看起来差别大"的地方，本质是什么

1. **Y 轴**：上游在图像坐标系（y 向下）里算，输出路径其实是**镜像**的；它只画图所以没人发现。
   我们在 map 坐标系（y 向上）里算，输出可直接给控制器 —— 这是**必须**的差别，不是风格。
2. **边界 pad 2 格**：上游靠地图四周本来有墙；我们的 PGM 边缘可能是空地/未知，不加边框
   `_getBoundSquare2` 会越界读写（该函数不查边界）。⇒ 我们加了 2 格障碍边框。
3. **Map 有实际使用**：上游 `test()` 把 mask 注释掉了；我们接了 `/plan_mask`（patchwork 动态障碍），
   且用 `setMaskMapTimed` 让它 1 s 后衰减。

## B. 局部控制：7 个环节

| # | 环节 | T-DT `testKastar()` | 我们 `computeVelocityCommands` | 等价? |
|---|---|---|---|---|
| 1 | 地图 | 整张 PNG | 以机器人为中心 **±16 m（32 m 见方）窗口**（`/map`+`/plan_mask`，外扩 2 格） | 不同（实时性） |
| 2 | 起点 | 硬编码 `(25.27,14.15)`，速度/加速度 0 | 当前位姿 + zlio 速度（`vx_/vy_`） | 不同 |
| 3 | 终点 | 硬编码 | 沿全局路径 **前视 4 m**；有新鲜 plan 时用 plan 上的点 | 不同 |
| 4 | Kastar 配置 | `maxNodes=150000, sampleTime=0.01`，vmax/amax 默认 | 同 + `maxVel=1.5, maxAcc=3.0` | 等价（参数不同） |
| 5 | 结果用途 | **只验收**：误差<1e-6、无碰撞线段、各轴速度/加速度不超限 | **取"最近样本+2"的速度做前馈**、方向做参考朝向 | **本质不同** |
| 6 | 反馈 | 无 | `w = 2.0·wrap(yaw_ref − yaw)`，限幅 3.0；世界速度转车体系 | 我们新增 |
| 7 | 输出 | `return true/false` | `Twist`（vx, vy, w）→ `/cmd_vel` | 我们新增 |
| 8 | 失败 | — | 局部目标落障碍 → 沿路径**回退 20 点**；Kastar 失败/太近 → **纯跟踪** | 我们新增 |

## C. 完整链路（端到端）

```
T-DT（上游）
  PNG 文件 ──imread──► 二值图 ──► A* ──► 简化 ──► 梯形时间 ──► SFC+MinSnap(6 轮) ──► path/corridor
                                                                                      │
                                                                                      ▼
                                                                                 cv::imshow（结束）

我们
  /map(缓存) + /plan_mask ──► 二值图(+pad2) ──► A* ──► 简化 ──► 梯形时间 ──► SFC+MinSnap(6 轮)
        │                                                                        │
        │                              /localization/odometry（位姿+速度）        ▼
        │                                                      TrajectoryPlan{t,x,y,vx,vy,corridor}
        │                                                                        │
        ▼                                                                        ▼
  16 m 窗口栅格 + mask ──────────────► Kastar(前视 4 m) ◄─────────────────────────┘
                                            │
                                            ▼
                                 前馈(样本+2 速度) + 朝向反馈
                                            │
                                            ▼
                                    /cmd_vel → /cmd_vel_chassis
```

## D. 结论

- **算法（A*/SFC/MinSnap/Kastar）**：与上游同一份代码（`yastar/kinodynamicAstar/sfcSquare` 0 差异，
  `minimumSnap` 2 行是加 `times` 字段）。
- **5 处实打实的差别**，都有明确原因：
  1. 地图缓存 vs 每次 imread；
  2. **Y 轴约定**（上游在图像坐标系里，输出镜像；我们在 map 坐标系）；
  3. **边界 pad 2 格**（防止 `_getBoundSquare2` 越界）；
  4. **短路径保护**（<3 点不简化）；
  5. **Mask 真正启用**（上游 demo 注释掉了）。
- **控制链路**：上游只验收，我们把"轨迹→速度"补全了（前馈+反馈+回退+兜底）。

## E. 局部窗口 vs T-DT 的整图搜索（补充）

### E.1 事实

| | T-DT `testKastar()` | 我们 `buildLocalGrid` |
|---|---|---|
| 搜索范围 | **整张图**（560×300 = 168k 格），原点固定在地图原点 | 以机器人为中心 **±16 m**，clamp 到地图边界（出生点处 18.2 × 15.7 m = 114k 格，占全图 **65%**） |
| 栅格原点 | 固定 | **每周期随机器人移动**（`gox_=ox+x0·res−P·res`） |
| 边框 | 图自带黑墙 | 外部外扩 2 格障碍 |
| 动态障碍 | 无（demo 注释掉了 mask） | 叠加 `/plan_mask` |
| 调用频率 | 一次 | 30 Hz |

### E.2 差别性质

- **算法**：同一个 `KinodynamicAstar`，参数一样（15 万节点 / dt 0.01）；
- **差别只在"搜索域"和"栅格锚点"**：
  1. 我们的窗口**每周期重锚** ⇒ 同一世界点在相邻周期的栅格索引不同 ⇒ 起点量化相位会变（轨迹轻微抖动）；
  2. 贴地图边缘时窗口被 clamp 变小；小于 8 格直接 `return false` ⇒ 掉纯跟踪；
  3. 远处（>16 m）的可行走廊不在窗口里 ⇒ 若局部目标需要绕远则搜不到；
  4. 32 m 见方 ≈ 41 万格，每周期重填 + 15 万节点搜索 ⇒ CPU 明显高于"只搜 4 m 前视"的必要范围。

### E.3 结论

**今天这个窗口实际上很大（65% 全图），所以行为上接近 T-DT 的整图搜索**；
真正偏离 T-DT 的是 ①每周期重锚、②叠加 mask、③边缘 clamp。
若要提速/降失败率而把窗口收到 ±6 m，**那才会变成"和 T-DT 明显不同"**（届时要明确这是本地规划器的正常取舍）。

## F. 更正：局部规划器到底读哪张图（逐行确认）

| 场景 | 代码 | 用的是哪张图 |
|---|---|---|
| **正常运行**（收到过 `/map`） | `grid_ok = map_ready_ ? buildLocalGrid(px,py) : buildGrid();`<br>`buildLocalGrid` 里 `map_->data[...]` | **静态地图 `/map`（map_server 载入的 PGM）+ `/plan_mask`** |
| 启动瞬间（`/map` 还没到） | `buildGrid()` 里 `costmap_ros_->getCostmap()->getCharMap()` | nav2 局部代价图（含 STVL/obstacle 层） |

⇒ **STVL 只在"`/map` 尚未到达"的启动瞬间被用到；一旦 `/map` 到了就再也不会回到 STVL 分支。**

所以：
- **T-DT 局部**：读 PGM（整张）；
- **我们局部**：读 **`/map`（同一张 PGM，经 map_server）+ `/plan_mask`**，再裁窗口。

**结论修正**：基底图**和 T-DT 是一样的（都是 PGM）**，我们额外叠了 mask、并且裁窗口。
之前说"绕过 STVL"仍然成立，但准确说法是：**Kastar 用 PGM+mask，不使用 nav2 的 STVL 层**；
STVL 只服务 nav2 自己的 costmap（例如 goal checker / 显示），不参与 Kastar 搜索。

⚠️ 另一个隐患：`mask_ok` 只校验几何（分辨率/原点/尺寸），**不校验时间戳** ⇒
若 `mask_builder` 停发，控制器会一直用最后一帧 mask（不会自动失效）。

## G. 2026-11-08 实跑诊断：为什么"路径看起来很奇怪"

### G.1 日志怎么读

```
[tdt] plan: 621 点, 走廊 125, 时长 21.66 s, A* 63→简化 63, snap=0(iter=1, 34.5ms)
      │          │          │            │      │         │
      │          │          │            │      │         └ MinSnap 返回值：success=0, 只跑了 1 轮
      │          │          │            │      └ 简化前后控制点数**都是 63**（没简化）
      │          │          │            └ A* 原始路径 63 点
      │          │          └ 梯形时间分配算出的总时长
      │          └ 走廊方盒数（62 段 ≈ 2 个/段）
      └ 发布给控制器的轨迹点数 = 62 段 × 10 + 1
```

### G.2 三处异常及根因

**① `snap=0` —— MinSnap 每次失败，但我们仍然发布了它的"碰撞解"**

`minimumSnap.cpp:110 iterate()`：

```cpp
for (round = 0; round < maxIter; round++) {
    coefficients = solve(ta, points);
    if (invalid) { result.success = false; break; }      // ← 第 2 轮在这里 break
    result.path.clear();
    …sample 每段、查碰撞，bad = 撞墙的段…
    result.success = bad.empty() && !path.empty();        // ← 第 1 轮：bad 非空 ⇒ success=0
    if (success || round+1 == maxIter || timeout) break;
    update(ta, points, bad, round);                       // ← 插中点/收窄走廊
}
```

- 第 1 轮：解出来了、采样了、**但发现有段撞墙** ⇒ `success=0`，`iter=1`，`path` 保留（621 点）；
- 第二轮：`solve()` 返回非法系数（走廊被 update 收窄后 QP 不可行）⇒ `break`，**不再覆盖 path**；
- 结果：返回的是**第 1 轮那个"撞墙"的轨迹**，`success=0`。
- 我们 `planner.cpp:201` 是"只要有 path 就用"（注释写着"与 T-DT 一致，demo 看 path 不看 success"）⇒
  **把一条会撞墙的轨迹发给了控制器**。

**② `A* 63→简化 63` —— 简化在拥挤场地里退化成"不简化"**

`yastar.cpp:195 simplifyPathDP` 的递归条件是：

```cpp
bool isInObstacle = lineInObsticle(path[index0], path[index1]);   // 首末点直线是否撞墙
if (maxDistance > threshold || isInObstacle) { …继续二分… }
```

RMUC 场地里任意两点的直线**几乎必然穿过墙体** ⇒ `isInObstacle` 恒真 ⇒ 一路二分裂到底
⇒ 返回**原始全部点**。`simplifyPathHypot` 同理（它的 `lineInObsticle` 也一直命中）。
所以 63→63，控制点间距 ≈ 3.4 m / 62 ≈ **5.4 cm**（正好一个栅格）。

**③ `时长 21.66 s` for 3.4 m —— 梯形时间被"短段"放大 10 倍**

`trapezoidalTimeAllocation` 对每段独立算：

```
distThs = 0.5·v²/a = 0.5·1.5²/2 = 0.5625 m
本段 d ≈ 0.054 m < distThs ⇒ 三角形：t = 2·sqrt(d/a) = 2·sqrt(0.027) = 0.329 s
62 段 ⇒ 62 × 0.329 ≈ 20.4 s      （日志 21.66 s ✓ 一致）
平均速度 = 3.4 m / 21.66 s ≈ 0.16 m/s
```

⇒ **每 5 cm 一段都给 0.33 s**，累积成 21 s 的"爬行轨迹"。

### G.3 三者叠加后的现象

| 现象 | 原因 |
|---|---|
| 车不动 / 慢慢挪 | 轨迹总时长 21 s，前馈速度 ~0.2 m/s |
| 原地打转（dyaw 7°→99°→148°→167°） | 5 cm 间距的锯齿路径 + 前视 4 m ⇒ 前视点在锯齿两端来回跳 |
| 越走越远（3.4 m → 4.44 m） | 每次重规划路径都变（621→631→601→…），控制器追的是**移动目标** |
| `OBS_DEGRADED` 频繁（NIS 23~58） | 原地打转时 scan-to-map 的对应关系被打乱，先验匹配冲突 |

### G.4 该改什么（按优先级）

1. **`snap=0` 时不要用它的 path**：回退到"A* 简化路径 + 弧长比例时间"（`traj_source=3`），
   至少那条路径是**不撞墙**的。
2. **MinSnap 之前按弧长重采样**（例如每 0.30~0.50 m 一个控制点）：
   一次性解决 ②（不再 63 段 5 cm）和 ③（每段 0.3 s 而不是 0.03 s 的三角形）；
   同时把 QP 规模从 62 段降到 8~12 段。
3. 走廊/迭代再调：`corridor_shrink=0` 时第 1 轮常撞墙 ⇒ 考虑 `dilate_cells` 或
   `simplify_threshold` 与重采样配合。

## H. 根因：T-DT 内部三处 origin 约定不一致，被我们的"取反 origin"适配器引爆

### H.1 上游内部约定（三个地方两种符号）

| 函数 | 换算 |
|---|---|
| `SfcSquare::valueAt` | `(p + originPos)/mapping` |
| `SfcSquare::_getBound / _getBoundSquare2` | `round((p + originPos)/mapping)` |
| `SfcSquare::isOutside` | `(p − originPos)/mapping` |
| `MinimumSnap::lineInObsticle`（`toCell`） | `(p − originPos)/mapping` |

**上游 demo 的地图 origin 就是 (0,0)**（`setMap(map, 0.05f, 0, 0)`），`+0` 和 `−0` 等价 ⇒ 这个矛盾永远不暴露。

### H.2 我们的适配器怎么引爆它

我们的 PGM origin 是 `(−19.9, −8.89)`。原先为了让 `+` 那一组正确，给 MinSnap 传了 **`−origin`**：

```cpp
ms.setMap(sfc_map, res, -ox, -oy);   // 旧代码
```

后果（originPos = +20.0）：

- `_getBound`：`(p + 20)/0.05` ✓ 走廊方盒正确；
- `isOutside`：`(p − 20)/0.05 ≈ −400` ⇒ **永远"出界"** ⇒ `getBound()` 返回 NaN 方盒；
- `lineInObsticle` 的 `toCell` 同样 `−400` ⇒ `toCell` 失败 ⇒ **`return true`（判碰撞）**。

于是 `iterate()` 里：

```
round 0：解出来 → 采样 → 每段 lineInObsticle 都返回 true ⇒ bad = 全部段 ⇒ success = 0
         update()：getBound() 返回 NaN ⇒ 走廊上下界 NaN
round 1：solve() 发现 NaN ⇒ 返回非法 ⇒ break（不覆盖 path）
结果：success = 0, iter = 1, path = 第 0 轮那条"被判碰撞"的轨迹
```

**完全对上历史日志**：所有 plan 全部 `snap=0(iter=1)`，无一例外。

### H.3 修复：全程使用"地图局部坐标"

```cpp
astar.setOriginPos(0.f, 0.f);
astar.setMap(W, H, res, 0.f, 0.f, a_occ);
path = astar.search({sx−ox, sy−oy}, {gx−ox, gy−oy});
ms.setMap(sfc_map, res, 0.f, 0.f);
…solve…
for (auto& p : traj_pts) { p.x() += ox; p.y() += oy; }   // 局部 → 世界
```

origin=(0,0) 后三处约定自动一致，**third_party 一行不用改**。

### H.4 实测对比（同一目标 (−1.36, 3.14)）

| | 修复前（用户那次） | 修复后 |
|---|---|---|
| `snap` | **0**（每次） | **1** |
| 单次求解耗时 | 15~40 ms | **2~4 ms** |
| 轨迹点数 | 621→601→… | 241→201→161→111 |
| 规划时长 | 18~31 s | **9.0 → 4.2 s** |
| 结果 | 打转、越走越远、最终 Ctrl-C | **SUCCEEDED 59.4 s，终点误差 0.28 m** |

### H.5 剩下的次要问题

`A* 25→简化 25`：DP 在拥挤场地仍不简化（`simplifyPathDP` 的"首末点连线撞墙就继续二分"），
控制点间距 ~0.14 m ⇒ 每段仍有 0.5 s 的三角形时间 ⇒ 时长仍偏大。
下一步：**MinSnap 之前按弧长重采样（0.3~0.5 m）**。

## I. 实测上游 benchmark（用他们自己的 5 组用例 + 他们自己的图）

为了回答"T-DT 有没有参考"，我把上游源码**复制到 `.tmp_cache/tdt-ref`**（`third_party` 不动），
加了几行打印，编译成 `probe`，跑他们 `main.cpp` 里那 5 组起终点（Y 翻转、参数都照抄：v=3, a=1）。

```
地图 560x300（他们的 rmuc2025）

#1 长度 34.1 m  A*点 611 → 简化 10   规划时长 32.35 s  snap=0  iter=6  采样 721
#2 长度 34.0 m  A*点 623 → 简化 10   规划时长 31.99 s  snap=0  iter=6  采样 501
#3 长度 26.9 m  A*点 471 → 简化 12   规划时长 32.16 s  snap=0  iter=6  采样 721
#4 长度 13.0 m  A*点 220 → 简化  7   规划时长 16.20 s  snap=0  iter=6  采样 471
#5 长度 25.5 m  A*点 450 → 简化  9   规划时长 25.75 s  snap=0  iter=6  采样 411
```

### 三条结论

1. **他们的简化是有效的**：611 → **10** 点（每段约 3.4 m）。因为官方场地开阔，
   首末点连线不穿墙，DP 能正常二分收敛；我们 55 → 55 是"密集场地"的另一个档位。
2. **他们的规划时长是合理的**：34 m / 32 s ≈ **1.06 m/s**；因为控制点少（10 个），
   梯形时间给每段 3 s 左右，不会被 5 cm 的碎段放大。
3. **他们自己的 demo 也是 `snap=0`**（5 个用例全失败，`iter=6` 跑满）。
   ⇒ 上游**从不检查 `success`**，只看耗时和画图。这也解释了"为什么他们看起来一点问题没有"。

### 所以能参考什么、不能参考什么

| 项 | 能否参考 | 说明 |
|---|---|---|
| 控制点密度 | ✅ 参考 | 他们的路径约 **10 个控制点**；我们也应重采样到 8~10 个（0.35~0.5 m/点） |
| 时间分配结果 | ✅ 参考 | 每段 ~3.4 m 对应 ~3 s；我们的碎段 5 cm 对应 0.33 s 是**退化** |
| `success` 标志 | ❌ 不要参考 | 他们自己也是 0；但我们已修到 1（说明我们的轨迹确实不撞墙了） |
| 整图 vs 窗口 | ❌ 无参考 | 他们只有整图 |

## J. 时长/采样参数对比：和上游**完全一样**，差别在控制点数

| 参数 | 上游 `main.cpp` | 我们 | 一致? |
|---|---|---|---|
| 简化阈值 | `simplifyPath(path, 0.1f)` | `simplify_threshold: 0.1` | ✅ |
| 多项式阶数 | `setOrder(6)` | `min_snap_order: 6` | ✅ |
| 导数阶数 | `setMaxDx(3)` | `max_dx: 3` | ✅ |
| **采样间隔 dt** | `setDt(0.1)` | `ms.setDt(0.1)` | ✅ |
| 迭代时限 | `setTL(10)` | `iter_time_limit: 10.0` | ✅ |
| 碰撞迭代 | `setCollisionCheckIter(6)` | `collision_check_iter: 6` | ✅ |
| 走廊范围 | `setMaxCorridorRange(2.5)` | `max_corridor_range: 2.5` | ✅ |
| 走廊收缩 | `setCorridorShrink(0)` | `corridor_shrink: 0.0` | ✅ |
| 时间归一化 | `setNormTime(true)` | `norm_time: true` | ✅ |
| 后端 | `OSQPCorridor` | `OSQPCorridor` | ✅ |
| 初速度 | `setInitVel({0,0})` | `setInitVel({0,0})` | ✅ |
| 时间分配 | `solve()` 内部自动 | 我们显式调用后传入 | 等价 |
| **maxSpeed** | **3.0** | **1.5** | ❌ 底盘值 |
| **maxAcc** | **1.0** | **2.0** | ❌ 底盘值 |

### 为什么时长差 10 倍：公式是 `t = 2√(d/a)`（短段）

```
distThs = 0.5·v²/a
d < distThs ⇒ 三角形：t = 2√(d/a)
总时长 ≈ Σ t_i ∝ √N   （N = 控制点数）
```

| 场景 | 控制点 N | 每段 d | 单段 t | 总时长 | 平均速度 |
|---|---|---|---|---|---|
| 上游 #1（34.1 m） | 10 | 3.79 m | 3.9 s | 32.4 s（实测） | 1.05 m/s |
| 我们（3.4 m） | 54 | 0.063 m | 0.36 s | 19.2 s（实测 17.9） | 0.19 m/s |
| 若重采样到 10 点 | 10 | 0.34 m | 0.82 s | **8.2 s** | 0.41 m/s |
| 若重采样到 6 点 | 6 | 0.68 m | 1.14 s | **5.7 s** | 0.60 m/s |

⇒ **参数完全一样，时长差异 100% 来自控制点数量**（N 越多，总时长 ∝ √N 越大）。
要达到 `vmax`，段长必须 `d ≥ v²/(2a) = 0.56 m`（v=1.5, a=2）⇒ 3.4 m 路径最多 6~7 个控制点。

## K. 控制点数不该手调：让它随路径长度自适应（模仿上游 ~10 点）

### K.1 上游的 6~10 点不是"选的"，是几何算出来的

`simplifyPathDP` 的收敛点取决于**场地是否开阔**：

- 他们的图开阔 ⇒ 子段的"首末点连线"很快变得不穿墙 ⇒ 二分终止 ⇒ 610 点压成 10 点；
- 我们的图密集 ⇒ 连线一直穿墙 ⇒ 一路二分到底 ⇒ 55 点保持 55 点。

**所以上游没有任何"点数参数"**，点数 = f(几何)。我们的退化正是"几何条件不满足"。

### K.2 自适应规则（一次写死，随距离自动变）

模仿上游"约 10 个控制点"的量级，用**按路径长度算出的段长**重采样：

```
spacing = clamp(L / 14, 0.6, 2.0)     # L = 路径弧长
N ≈ L / spacing
```

- 下限 0.6 m：保证 `d ≥ v²/(2a) = 0.5625`，每段能进"梯形"（加速—匀速—减速）；
- 上限 2.0 m：不超过走廊半宽（`max_corridor_range = 2.5`），否则盒子盖不住曲线；
- 目标点数 ≈ 14（与上游 7~12 同量级）。

### K.3 不同距离的预期（v=1.5, a=2, dt 0.1）

| 路径长 L | spacing | 控制点 N | 单段 t | 总时长 | 平均速度 |
|---|---|---|---|---|---|
| 3.4 m | 0.6（下限） | 6 | 1.14 s | **5.7 s** | 0.60 m/s |
| 7 m | 0.6 | 12 | 1.14 s | 12.6 s | 0.56 m/s |
| 13 m | 0.93 | 15 | 1.30 s | 17.5 s | 0.74 m/s |
| 25 m | 1.79 | 15 | 1.87 s | 25.0 s | 1.00 m/s |
| 34 m | 2.0（上限） | 18 | 2.02 s | 34.3 s | 1.00 m/s |

⇒ **不管目标远近，都不用改参数**；`spacing` 每次规划时按当前路径长度自动算。

## L. 真正的元凶：我自己加的 `<3 点就退回原始路径` 判断

### L.1 事实核对

用户质疑："那一对起终点之间特别开阔，为什么 55 个控制点？"

实测（沿直线逐点查 PGM）：

```
(0.00,0.02) → (-0.45,2.75)   直线全程畅通 ✅
(0.00,0.02) → (-1.36,3.14)   直线全程畅通 ✅
(0.00,0.02) → ( 2.00,-2.50)  直线全程畅通 ✅
```

所以 DP 简化**本来就应该**把 55 点压成 2 点（一条直线）。它确实压了 —— 但被我们下面这行代码丢掉了：

```cpp
if (path_s.size() < 3) path_s = path;      // ← 旧代码：<3 点就退回 55 点原始路径
```

这行是早先为"短任务会被 hypot 压成 2 点、导致 MinSnap 被跳过"加的**保险**，
结果反而把**成功的简化结果**扔了 ⇒ 开阔地带也变成 50+ 个 5 cm 控制点 ⇒ 梯形时间放大 10 倍。

### L.2 修复

```cpp
if (path_s.size() < 2) path_s = path;      // 只有真的degenerate才退回
…
if (path_s.size() >= 2) res_solve = ms.solve(in);   // 允许单段（2 点）
```

MinSnap 解"单段直线"没有问题（`validTimes` 只要 times+1 == 点数）。

### L.3 实测（目标 (-0.45, 2.75)，距出生点 2.7 m）

```
[tdt] plan: 11 点, 走廊 2, 时长 3.58 s, A* 34→简化 2, snap=1(iter=1, 0.1ms)
[tdt] plan: 11 点, 走廊 2, 时长 3.11 s, A* 27→简化 2, snap=1(iter=1, 0.1ms)
[tdt] plan: 11 点, 走廊 2, 时长 2.64 s, A* 20→简化 2, snap=1(iter=1, 0.1ms)
```

| | 修复前 | 修复后 |
|---|---|---|
| 简化 | `55→55` | **`55→2`** |
| 轨迹点 | 541 | **11** |
| 规划时长 | 17.9 s | **3.6 s**（→ 1.7 s） |
| 求解耗时 | 14 ms | **0.1 ms** |
| 导航 | 打转、59.4 s | **SUCCEEDED 11.7 s** |

### L.4 结论

- **不需要重采样**，也不需要偏离 T-DT；
- DP 简化本来就工作正常（上游 611→10、我们 55→2），是我们自己的保险判断把它废掉了；
- 只有在真正需要绕路（曲线）时，控制点数才会多几个 —— 那时 DP 也是按需分裂，不会给 5 cm 碎段。

## M. "全局规划无视障碍" 的核查（2026-11-08）

### M.1 隔离测试：在**我们自己的 PGM** 上直接跑 A*（绕过 ROS）

用 `.tmp_cache/tdt-ref/probe2`（加载我们的 PGM + 同样的 pad2 + 同样的 origin）：

```
map 564x317 origin(-20.00,-8.99)  cell(-18,-8)=255(自由)  cell(0,0)=255(自由)

(-18,-8) -> (-18, 5)   A* 268 点  路径 14.1 m  直线 13.0 m  | 简化 8 点
(-18,-8) -> (-17, 4)   A* 248 点  路径 13.5 m  直线 12.0 m  | 简化 9 点
( 0, 0)  -> (-0.7,2.8) A*  57 点  路径  3.1 m  直线  2.9 m  | 简化 2 点
```

- **直线 13 m 的两点，A* 给出 14.1 m 的绕行路径**（不是直线）⇒ A* **确实在绕障碍**；
- 简化输出 8~9 点（曲线），直线可达时才是 2 点。

⇒ **A\* + 简化 在算法层是正常的**，`忽略障碍`不可能来自这里。

### M.2 那"看起来无视障碍"可能是下面两种之一

| 可能 | 判据 | 检查方法 |
|---|---|---|
| ① 目标其实直线可达 | 日志 `A* N→简化 2` | 你给的两点直线是否穿墙（我可以帮你算） |
| ② A\* 绕了，但**最终轨迹**（MinSnap 平滑曲线）切角/穿墙 | 日志 `简化 M` 且 M≥3，但画出来的轨迹是直线 | 新增 `/tdt/astar_path` 话题：把 A\* 简化路径单独发出来 |

### M.3 已加诊断话题

```
/tdt/astar_path   (nav_msgs::Path)   ← A* 简化后的原始路径（世界系）
/plan             (nav_msgs::Path)   ← nav2 发布给控制器的（= MinSnap 平滑轨迹）
```

**在 RViz 里同时显示这两条**：

- 如果 `/tdt/astar_path` 是绕开的、`/plan` 是直的 ⇒ 问题在 **MinSnap**（平滑把曲线拉直了）；
- 如果两条都直 ⇒ A\* 认为直线可达（目标直线可达，或地图上那条线确实是空的）。

## N. "绕行跟 PGM 无关"的真相：发布了 MinSnap 的**失败部分解**

### N.1 日志证据（远端目标 (-15.23, -1.63)）

```
planner_server: OSQP未收敛，方向=1 状态=-2
[tdt] plan: 121 点, 走廊 15, 时长 15.91 s, A* 302→简化 10, snap=0(iter=3, 54.2ms)
```

- **A\* 算对了**：302 个格 → 简化 **10 个控制点**（约 16 m 的绕行）；
- **MinSnap 失败**：OSQP 未收敛（状态 −2），只跑到第 3 轮；
- 而旧代码**只要 `path` 非空就用** ⇒ 发布的正是这条**失败的部分解**（121 点、15.9 s）；
- 部分解没有满足走廊/碰撞约束 ⇒ 看起来"乱绕、跟 PGM 无关"。

### N.2 修复：`snap=0` 时回退到 A* 简化路径

```cpp
if (res_solve.success && res_solve.path.size() >= 2) traj_pts = res_solve.path;
else { WARN("MinimumSnap 失败 ⇒ 回退 A* 简化路径"); traj_pts = path_s; }   // 一定不撞墙
plan.traj_source = res_solve.success ? 1 : 3;                              // 3 = 回退
```

回退时时间按**弧长比例**分配（`pts_match` 为假走 else 分支）。

### N.3 修复后实测（同一个远端目标）

```
[WARN] MinimumSnap 失败(snap=0, iter=3) ⇒ 回退用 A* 简化路径(10 点)
[tdt] plan: 10 点, 走廊 14, 时长 16.66 s, A* 327→简化 10, snap=0(iter=3)
...
终点 (-13.355, -0.531)   ← 目标 (-15.23, -1.63)，跨场行进约 14.5 m
```

路径现在**就是 A\* 的绕行结果**（与 PGM 一致），不再出现"凭空直线/乱绕"。

### N.4 仍未解决：长曲线路径上 MinSnap 会失败

| 现象 | 原因（待验证） |
|---|---|
| `OSQP 未收敛 状态=-2`，`iter=3` | 走廊 + 碰撞迭代若干轮后 QP 不可行 |
| 只发生在长绕行（10~11 控制点、16 m） | `max_corridor_range=2.5 m` 相对段长偏小；`norm_time=true` 条件数差 |

可试的旋钮（都属于 T-DT 参数，不改代码）：
`norm_time: false`、`max_corridor_range: 3.5`、`collision_check_iter: 8`。

## O. 官方文档（README + doc/Usage.md）核对结果

来源：`T-DT-Algorithm-2026/tdt-nav-kit` 的 README 与 `doc/Usage.md`（外部资料，可信度=官方）。

### O.1 参数全部一致（README 的"运行参数"表）

分辨率 0.05 / 掩码 4 层 / 势场 `0.5/(0.1+x)+1` / DP 阈值 0.1 m / 多项式 6 / 导数阶 3 /
**采样时间 0.1 s** / **迭代 6** / vmax 3.0 / amax 1.0 / sfc 范围 2.5 m / **归一化时间 开启** /
OSQPCorridor / 终速 0 —— 我们那一份逐项相同（只有 v/a 用底盘值 1.5/2.0）。

### O.2 三条**文档里写明、我们之前做错/误解**的地方

| 文档原文 | 我们之前的做法 | 影响 |
|---|---|---|
| `setPath()`："**起点速度默认自由**"（不调 setInitVel 就不约束） | 硬传 `{0,0}` | 每 1 s 重规划都**强迫从静止起步** ⇒ 首段被压慢、QP 更易失败 |
| `setMaxSpeed/Acc`："**只影响未显式设置 timeAllocated 时的时间分配**，不是硬约束" | 以为它是速度上限 | 实际速度上限由控制器决定 |
| `setDt()`："**solve() 的碰撞迭代使用每段固定 11 点，不读取 dt**" | 以为 dt 影响采样 | dt 只在旧接口里用，无效果 |

另外：`success` 的语义是"是否得到满足当前碰撞检查的结果"，**迭代耗尽时最后一条仍可能碰撞**。

### O.3 据此做的三处改动（都在我们的适配层，T-DT 未改）

1. **`init_vel_mode`**（默认 0=自由）：
   - 0：不调 `setInitVel`（文档语义，最接近上游默认）；
   - 1：`{0,0}`（旧行为）；
   - 2：按两次 `createPlan` 的位置差分估计。
2. **失败重试链**：`snap=0` 时依次重试 ①`norm_time=false` ②`backend=OSQPPath`，
   成功则用新结果（日志会打"重试①/②成功"）。
3. **成功门控**：只有 `success` 为真才用平滑轨迹，否则回退 A* 简化路径（`traj_source=3`）。

### O.4 诊断补充：`状态=-2` 是 `MAX_ITER_REACHED`

上游固定 `eps_abs=eps_rel=1e-8`、`max_iter=20000`、`time_limit=0.1 s`。
长绕行（8~11 控制点、16 m）时 QP 在 0.1 s 内没收敛 ⇒ `status=-2`。
重试链的目的就是换一组更"好解"的等价约束把它解出来。

## P. `doc/Usage.md` 逐条核对：又发现 3 处该用的没用

| # | 文档原文 | 我们的状态 | 处理 |
|---|---|---|---|
| 1 | `MinimumSnap` **默认非严格碰撞检测**；非严格模式下 `lineInObsticle` 允许线段穿过 ≤2 个障碍栅格，"这不是几何意义上的安全距离保证" | **没开严格模式**（`strictCollision=false`） | ✅ 已调用 `ms.setStrictCollision(true)`（主解 + 重试解都开） |
| 2 | `OSQPCorridor` 下 `autoCorridor=true` 时"轨迹可以在自动走廊内通过，**不一定严格经过 xy**" | 以为会穿过控制点 | 说明"平滑轨迹和 A\* 折线不完全重合"是**设计如此**；要强制经过需 `setItems()+useVxvy` |
| 3 | `SfcSquare::setSaveDistance(d)` 会膨胀障碍**并把地图边缘标成障碍** | 我们用 pad 2 格代替 | 二者等价；保留我们的做法 |
| 4 | `setItems()` **不会**自动设置终点零速；`setPath()` 才会 | 我们用 `setPath` ✓ | 无需改 |
| 5 | `KinodynamicAstar::search`：`startVel` 或 `startAcc` 任一非零时**首段用给定 startAcc** | 我们传 `{0,0}` | 末段速度/加速度偏保守，暂不影响 |
| 6 | `maxVel/maxAcc` 限制的是**各轴**，不是向量二范数 | 我们按向量理解 | 影响很小 |

### 关键：第 1 条很可能就是"路径看起来贴墙/穿一格"的来源

`strictCollision=false` 时，碰撞检测允许"擦过 2 个障碍格"，
所以即使 `snap=1`，轨迹也可能压到墙的一两个格子——**看起来就像"跟 PGM 不符"**。

开启严格模式后再配合：
- `success` 门控（失败就回退 A* 路径）；
- 重试链（`norm_time=false` → `OSQPPath`）；
- 走廊夹取（曲线路径生效）；
四条一起，才能保证"要么给一条不撞墙的平滑轨迹，要么给一条一定不撞墙的折线"。

## Q. 诊断：这次日志的两个决定性发现

### Q.1 严格碰撞检测 = MinSnap 全线失败

开启 `setStrictCollision(true)` 之后，**每一次求解都失败**（连 6~7 点的短路径也是）：

```
重试①(norm_time=false) => snap=0 iter=6
重试②(OSQPPath)        => snap=0 iter=6
MinimumSnap 失败 ⇒ 回退 A* 简化路径(6 点)
```

原因链：
1. 我们的 A\* 路径**贴墙**（地图不做膨胀，路径离墙 1 格）；
2. 走廊里平滑出来的曲线必然擦到 1~2 个障碍格；
3. 非严格模式允许擦 2 格 ⇒ 以前 `snap=1`；
4. 严格模式一格都不许 ⇒ 每轮都判碰撞 ⇒ `update()` 不停收窄走廊 ⇒ QP 不可行。

**结论**：严格模式并不适合"路径贴墙 + 无膨胀"的输入；**上游默认非严格是有道理的**。
要真正保证离墙距离，应该做**地图膨胀**（`dilate_cells` / `SfcSquare::setSaveDistance`），而不是把碰撞阈值调到零容忍。

⇒ 已改回 `strict_collision: false`（参数可开）。

### Q.2 车"原地画龙"：朝向 P 环在饱和下自激

新增的指令日志给出了直接证据：

```
cmd: vx=-0.85 vy=-0.65 w=-3.00 | yaw=0.05  yaw_ref=-2.44
cmd: vx=+0.85 vy=-0.56 w=-1.17 | yaw=-1.87 yaw_ref=-2.45
cmd: vx=+1.00 vy=+0.19 w=+0.37 | yaw=-2.77 yaw_ref=-2.58
cmd: vx=+0.82 vy=+0.60 w=+1.25 | yaw=3.05  yaw_ref=-2.61
cmd: vx=+0.89 vy=+0.49 w=+1.00 | yaw=-3.14 yaw_ref=-2.64
```

- `w` 先饱和到 **−3.00**，车头从 0.05 一路转到 −3.14，**越过**参考 −2.5 后进入反向修正（+1.25）；
- 车体系速度指令**随车头一起转**，所以虽然 `vx/vy` 一直有值，**世界系位移互相抵消**（位置几乎不变）；
- `s=0.00` 每周期重置（1 Hz 重规划把进度清零），`v_ref` 恒为 ~1.0，也掩盖了"没前进"这件事。

**结论**：麦轮底盘不需要"车头对准路径"也能走（可以横移）。
纯 P 朝向环 + 饱和 = 过冲自激，是"原地画龙"的直接原因。

⇒ 新增 `heading_mode`：
- `1`（默认）：**保持车头，纯横移**（`w = 0`），麦轮直接朝前视点平移；
- `0`：旧的转向逻辑。

## R. "全局路径无视障碍"的元凶：OSQPPath 重试（无走廊）

### R.1 证据

```
OSQP未收敛，方向=1 状态=-2
重试①(norm_time=false) => snap=0 iter=0
重试②(OSQPPath)        => snap=1 iter=2
MinSnap 重试②成功（OSQPPath）
[tdt] plan: 111 点, 走廊 12, 时长 20.75 s, A* 298→简化 10, snap=1
```

`snap=1` 看起来是好事，但成功的是 **`Backend::OSQPPath`** —— 官方文档写明：

> `OSQPPath` **不会生成范围走廊**，但仍会使用 `autoCorridor=false` 的 Item 中的手动走廊。

我们用的是 `setPath()`（autoCorridor=true）⇒ **没有任何走廊约束**，只有"控制点位置等式约束"。
于是 10 个控制点之间的 5 次多项式曲线可以**大幅切角**：
绕行 15 m 的路径，平滑曲线直接从墙里穿过去 ⇒ 控制器照着开 ⇒ **车卡墙**。

### R.2 修复

`retry_osqp_path` 参数，**默认 false**（不再用无走廊后端）。
可行的成功路径只剩两种：

1. 主解（`OSQPCorridor`，带走廊 + 碰撞检查）`snap=1`；
2. 全失败 ⇒ 回退 **A\* 简化折线**（一定不撞墙）。

这样"发出去的每一帧轨迹"都满足：**要么在走廊内、要么就是 A\* 折线**。

## S. 规划已正常，剩下的"不动"在指令→底盘之间

### S.1 这一次日志的正面结论

```
[tdt] plan: 121 点, 走廊 13, 时长 18.7 s, A* 284→简化 10, snap=1(iter=4, 6ms)
```

- **主解成功**（`OSQPCorridor` + 走廊 13 个盒子），**没有** OSQPPath 重试；
- 求解 6 ms；`简化 10` 说明 A\* 绕行正常；
- ⇒ **规划侧这一版是健康的**。

### S.2 控制器输出也是对的

```
[tdt-local] cmd: vx=-0.56 vy=-0.29 w=+0.00 | yaw=-0.07 yaw_ref=-2.73 d_goal=3.43
```

- 方向指向目标（世界系 ≈ (-0.56,-0.33)，目标在 (-14.3,-3.5) 方向 ✓）；
- 幅值 0.63 m/s。

### S.3 但车几乎不动

```
robot: (-0.78,-0.82) → (-0.78,-1.03) → (-0.81,-1.13) → (-0.79,-1.23) → (-0.81,-1.28)
45 s 只走了 0.45 m  ⇒ 实际 0.01 m/s，而指令 0.63 m/s（差 60 倍）
```

`s=0.00` 每周期重置、`d_goal` 恒定 ≈3.5 m，都是"没有前进"的伴随现象。

**结论：断点在"控制器 → 轮子"之间，或车被卡住**，不在规划。

### S.4 三步定位（30 秒）

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel_chassis --once
ros2 topic echo /odom_ground_truth --once
```

| 现象 | 结论 |
|---|---|
| `/cmd_vel` 有值、`/cmd_vel_chassis` 全 0 | `velocity_smoother` 或 `fake_vel_transform` 把指令吃了（检查 `spin_speed` 是否 0） |
| 两者都有值、`/odom_ground_truth` twist≈0 | **底盘/物理卡住**（联系到该位置 0.3 m 内已有障碍采样） |
| `odom_ground_truth` 在动、zlio 位姿不动 | 定位假死（与规划无关） |

### S.5 若是"贴墙卡住"，根因是**没有膨胀**

`dilate_cells: 1` = 5 cm，而车体半径约 20 cm。A\* 会贴着 5 cm 的间隙走，
物理上就过不去 ⇒ 需要 **`dilate_cells: 4`（20 cm）**。


---

## §C 消费链路（障碍/速度/指令怎么流动）


### 来源：`planner_controller_consumption.md` —— 规划器 / 控制器的消费链路对比


> 2026-11-08。【代码】= 读源码确认；【实测】= 跑出来过。

## 1. 规划器输出：T-DT 给的是内存里的两个数组，我们发布成消息

### T-DT（`main.cpp::test()` 拿到 `SolveOutput` 之后）

```cpp
auto result = minimumSnap.solve(input);      // SolveOutput{ path, corridor, success, iter, time }

// 只做两件事：画线和画走廊
for(auto& p : result.path) wayPoints.push_back(cv::Point(p.x()*20, p.y()*20));
cv::polylines(visualize, wayPoints, false, cv::Scalar(0,0,192));
if(!result.corridor.empty()){
    auto rects2 = SfcSquare::pointPair2Rects(result.corridor);
    for(auto& rect : rects2) cv::rectangle(visualize, rect, cv::Scalar(170), 1);
}
```

- `result.path`：`std::vector<Eigen::Vector2f>`，**只有 x/y**；
- `result.corridor`：`std::vector<PointPair>`（方盒），也只用来画；
- **没有任何下游消费者**（画完 `cv::imshow` 就结束）。

### 我们（`planner.cpp` → `/trajectory_plan`）

```cpp
rm_plan_msgs::msg::TrajectoryPlan plan;
plan.header.stamp = node_->now();                 // 发布时刻（消费端算 age 用）
plan.traj_source  = (res_solve.path.size() >= 2) ? 1 : 3;   // 1=MinSnap, 3=fallback
for (i...) { plan.t.push_back(t_of[i]); plan.x.push_back(...); plan.y.push_back(...); }
// 速度：位置差分
plan.vx[i] = (plan.x[i+1]-plan.x[i]) / dt;   plan.vy[i] = (plan.y[i+1]-plan.y[i]) / dt;
// 走廊：直接用 MinSnap 内部那份 result.corridor
plan.corridor = { xmin, ymin, xmax, ymax, ... };
```

消息定义（`rm_plan_msgs/TrajectoryPlan`）：

```
float64[] t, x, y, vx, vy      # 并行数组，长度 N
float32[] corridor             # 每 4 个数一个方盒
uint16 traj_source, corridor_shrink_mm, grid_resolution_mm
```

**关键差异**：T-DT 的 `path` **没有时间轴**（`times` 是我们加的那 2 行才有）；
我们把**每点时间 + 差分速度 + 走廊**打包成一条消息发出去 —— 下游才能做"前馈控制"。

## 2. 消费关系总表

| 数据 | T-DT 的生产者 → 消费者 | 我们的生产者 → 消费者 |
|---|---|---|
| 栅格地图 | `cv::imread` → A* | `map_server /map` → 规划器（缓存）+ 控制器（窗口裁剪） |
| 动态障碍 | Mask 层（同一进程） | `mask_builder /plan_mask` → 规划器 + 控制器 |
| A* 路径 | `YAstar::search` → 同函数 | 规划器内部 |
| 简化路径 | `simplifyPath` → MinSnap | 规划器内部 |
| 时间分配 | `trapezoidalTimeAllocation` → MinSnap | 规划器内部 |
| 走廊 | `SfcSquare` → MinSnap（画图） | MinSnap → **`/trajectory_plan.corridor`**（控制器/未来 MPC 用） |
| 平滑轨迹 | `SolveOutput.path` → **只画图** | `/trajectory_plan` → **控制器取前视点**、`nav_msgs/Path` → 代价图/显示 |
| 局部轨迹 | `KinodynamicAstar` → **只做验收**（误差/碰撞/超限） | 控制器 → **取样本速度做前馈** → `/cmd_vel` |
| 位姿 | —— | zlio 后端 `map→base_link` TF + `/localization/odometry` |
| 车速 | —— | 控制器 → `/cmd_vel` → `fake_vel_transform` → `/cmd_vel_chassis` |

## 3. 控制器：T-DT 只"验收"，我们真的"控车"

### T-DT `testKastar()`

```cpp
auto result = astar.search(start, velocity, acceleration, end);
// 然后只做判定：
double error = (result.trajectory.back().state.head<2>() - end).norm();
bool passed = result.success && error < 1e-6 && collisions == 0 &&
              maxVel <= astar.config.maxVel + 1e-6 &&
              maxAcc <= astar.config.maxAcc + 1e-6;
```

⇒ **没有任何速度输出**，它只是 benchmark。

### 我们的控制器（`computeVelocityCommands`）

| 步骤 | 代码要点 |
|---|---|
| ① 取位姿 | zlio `/localization/odometry`（`pose`）+ `twist.linear` 转车体速度 `vx_/vy_` |
| ② 前视点 | 把机器人投影到路径上，沿路径走 `lookahead=4.0 m`；**若 `/trajectory_plan` 新鲜（age<2 s），优先用它的点** |
| ③ 建窗格 | 以机器人为中心裁 **16 m 窗口**，叠加 `/plan_mask`，外扩 2 格 |
| ④ 局部目标回退 | 如果局部目标落在障碍格，沿路径**最多回退 20 个点** |
| ⑤ Kastar | `maxNodes=150000, sampleTime=0.01, maxVel=1.5, maxAcc=3.0` |
| ⑥ 前馈 | 取离机器人最近样本的**再后 2 个样本**：`vel` 作前馈、`atan2` 作参考朝向 |
| ⑦ 反馈 | `w = k_theta(2.0) * wrap(yaw_ref - yaw)`，限幅 `max_w=3.0` |
| ⑧ 输出 | 世界速度→车体系、限幅 `max_vel=1.5` → `Twist` |
| ⑨ 回退 | Kastar 失败/目标太近（`< max(goal_tol=0.15, min_local_goal=1.0)`）⇒ **纯跟踪**：`v=0.76*max_vel`，`x=v·cos(Δψ)`，`y=v·sin(Δψ)`，`w=k_theta·Δψ` |

## 4. 结构差异一览

| 维度 | T-DT 上游 | 我们 |
|---|---|---|
| 运行形态 | 单进程 benchmark | nav2 插件（`planner_server`/`controller_server`） |
| 轨迹传递 | 函数返回值（内存） | ROS 消息（`/trajectory_plan`），带时间/速度/走廊 |
| 轨迹时间 | 内部有，**未导出** | **导出**（我们加 `times`） |
| 速度剖面 | 无 | 位置差分得到 `vx/vy` |
| 局部目标 | 硬编码终点 | 沿全局路径前视 4 m（并优先用 plan 的点） |
| 速度输出 | **无** | 前馈 + 朝向反馈 → `Twist` |
| 失败处理 | `return false` | 局部目标回退 20 点 → Kastar；再失败 → 纯跟踪 |
| 帧/坐标 | 图像像素 ↔ 米 | map 帧 + 车体系，TF 变换 |
| 实时性 | 一次性 | 30 Hz 控制循环，plan age 检查 |

## 5. 一句话

> **T-DT 的"消费链路"到 `cv::imshow` 就结束了**（planner/controller 都是 benchmark，没有速度输出）；
> **我们的消费链路是**：`/trajectory_plan`（带时间/速度/走廊）→ 控制器取前视点 → Kastar → 前馈+反馈 → `/cmd_vel` → 底盘。
> 算法同源，**"怎么用"是我们补的**。

## 6. 对照二（消费链路）的最终结论

| 环节 | T-DT 上游 | 我们 | 差异性质 |
|---|---|---|---|
| 规划器输出形态 | `SolveOutput{path, corridor}`，内存数组 | `TrajectoryPlan` 消息（t/x/y/vx/vy/corridor） | **补了时间轴与速度** |
| 规划器输出用途 | `cv::polylines` + `cv::rectangle` 画图 | 控制器取前视点；`Path` 给代价图/显示 | **从可视化变成控制输入** |
| 走廊用途 | 画图 | 打包进消息（暂未参与控制） | 预留 |
| 局部规划器 | `testKastar()`：只验收（误差/碰撞/超限） | 真的算速度：前馈 + 朝向反馈 | **从 benchmark 变成控制器** |
| 速度输出 | **无** | `/cmd_vel`（车体系 vx/vy/w） | 新增 |
| 失败处理 | `return false` | 回退 20 点 → Kastar → 纯跟踪 | 新增 |
| 时序 | 一次性 | 30 Hz + plan age(<2 s) 检查 | 新增 |
| 地图 | 每次 `imread` | 订阅一次缓存 | 性能差异 |

**一句话**：算法完全相同；**"plan 怎么被消费"这一段是我们补的**——
T-DT 的 plan 只到屏幕，我们的 plan 到 `/cmd_vel`。



### 来源：`consumption_chain_clean.md` —— 消费链路（重整理版）：全局 / 局部


> 2026-11-08。**这一页是唯一权威版本**，早前分散在各文档里的说法以本页为准。
> 【代码】= 逐行确认；【实测】= 跑过。

## 0. 先建立三层模型

```
第 1 层：图        第 2 层：轨迹                第 3 层：速度
PGM/OccupancyGrid → A*(全局) + MinSnap → path → Kastar(局部) + 跟踪律 → /cmd_vel
```

- **T-DT 上游只做到第 2 层**：`test()` 拿到 path 就 `imshow`；`testKastar()` 拿到轨迹只做校验（不发速度）。
- **我们三层都实现了**，并且用 ROS 话题把第 2 层交出去（`/trajectory_plan`）。

---

## 1. 全局规划（`rm_tdt_planner`，nav2 `planner_server` 插件）

### 1.1 输入（它读什么）

| 输入 | 话题/接口 | 类型 | 用途 |
|---|---|---|---|
| 静态图 | `/map` | `OccupancyGrid`（transient_local） | 转二值（**只有 0/100 算障碍**）+ 外扩 2 格边框 |
| 动态障碍 | `/plan_mask` | `OccupancyGrid` | 叠成 mask 层（1 s 衰减、最多 16 层） |
| 起终点 | `createPlan(start, goal)` | nav2 调用（map 帧 `PoseStamped`） | 必要时 TF 到全局帧 |

> 与 T-DT 对应：T-DT 读 `images/rmuc2025.png`（二值：0=障碍）；mask 接口有但 `test()` 里**注释掉了**。

### 1.2 处理（它做什么）

```
二值图(+pad2) → 代价场 1+0.5/(0.1+d) → A*(8邻域) → simplifyPath(0.1)+hypot
→ trapezoidalTimeAllocation → SFC 方盒走廊 + MinSnap(QP) 交替迭代 ≤6 轮
```

（与 T-DT 逐环节相同，差异只有：缓存地图、pad2、短路径保护、mask 启用、坐标用 map 帧）

### 1.3 输出（它发什么、谁消费）

| 输出 | 格式 | 消费者 | 用途 |
|---|---|---|---|
| `/trajectory_plan` | `rm_plan_msgs/TrajectoryPlan`：`t,x,y,vx,vy,corridor` | **局部控制器** | 取前视点；速度用于对齐 |
| 返回 `nav_msgs/Path` | 由 nav2 `planner_server` 转成 `FollowPath` action | **nav2** → 控制器 `setPlan()` | 控制器的主路径 |

> 与 T-DT 对应：T-DT 的 `SolveOutput{path, corridor}` **只在内存里**，然后 `cv::polylines` + `cv::rectangle` 画图，无消费者。

---

## 2. 局部控制（`rm_tdt_local_controller`，nav2 `controller_server` 插件）

### 2.1 输入（它读什么）

| 输入 | 来源 | 备注 |
|---|---|---|
| 当前位姿 `pose` | nav2 传入（`costmap_ros_->getRobotPose`，实际 TF `map→base_link`） | 由 zlio 后端发布的 TF |
| 当前速度 | `/localization/odometry` 的 `twist`（0.5 s 内有效） | 否则用位姿差分兜底 |
| 路径 `path_` | nav2 `setPlan()` | 投影/前视用 |
| 时间轨迹 | `/trajectory_plan`（age < 2 s） | **优先用它挑前视点** |
| 局部地图 | **`/map` + `/plan_mask`**（不是 STVL） | 裁 ±16 m 窗口 |
| 导航目标 | nav2 `FollowPath` 的 goal checker | 判到达 |

> ⚠️ nav2 的局部代价图（STVL / obstacle_layer）**只在 `/map` 尚未到达的启动瞬间**被用到
> （`buildGrid()` 分支）；`/map` 一到就只用 `/map`+mask。

### 2.2 处理（它做什么）

```
投影到 path → 前视 4 m（plan 新鲜时用 plan 的点）
→ 局部目标若在障碍格：沿 path 回退 ≤20 点
→ 裁 ±16 m 窗口（clamp 到地图 + 外扩 2 格）+ mask → 0/255
→ Kastar(15万节点, dt 0.01, v 1.5, a 3.0)
→ 取"离车最近样本 +2"：速度做前馈、方向做参考
→ w = 2.0·wrap(Δψ)，限幅 3.0
→ 失败/太近 → 纯跟踪
```

### 2.3 输出

| 输出 | 格式 | 消费者 |
|---|---|---|
| `TwistStamped`（车体系 vx, vy, w） | nav2 controller 返回值 | nav2 → `/cmd_vel` → `fake_vel_transform` → `/cmd_vel_chassis` |

> 与 T-DT 对应：`testKastar()` 拿到轨迹只算 `passed = success && error<1e-6 && 无碰撞 && 不超限`，**没有任何速度输出**。

---

## 3. 一页对照表

| | T-DT 全局 | 我们全局 | T-DT 局部 | 我们局部 |
|---|---|---|---|---|
| 读图 | PNG 文件（每次 imread） | `/map`（缓存） | 同一张 PNG | `/map` + `/plan_mask`（窗口） |
| 动态障碍 | mask 接口（注释掉） | `/plan_mask` | 无 | 有 |
| 起终点 | 函数参数 | nav2 action | 硬编码 | 位姿 + 前视 4 m |
| 输出 | `path` + `corridor` | `/trajectory_plan` + `Path` | `bool` | `Twist` |
| 下游消费者 | `imshow` | 控制器 + nav2 | 无 | nav2 → `/cmd_vel` |

## 4. 一句话

> **第 1 层（图）两边同源（都是 PGM）；第 2 层（轨迹）算法同源、我们额外导出时间/速度/走廊；
> 第 3 层（速度）是 T-DT 完全缺失的一层，由我们补全。**

## 5. 局部地图（`buildLocalGrid`）详解

### 5.1 代码位置与输入

`controller.cpp:252-283`。输入三样：

| 输入 | 说明 |
|---|---|
| `map_` | `/map`（map_server 载入的 PGM，`OccupancyGrid`，transient_local） |
| `mask_` | `/plan_mask`（patchwork 动态障碍，几何必须与 `/map` 完全一致才启用） |
| `cx, cy` | 机器人当前位姿（map 帧） |

### 5.2 三步数学

**① 裁窗口**（参数 `grid_window_` 是**半径**，默认 16 m）

```cpp
x0 = (cx - 16 - ox)/res;   x1 = (cx + 16 - ox)/res;
x0 = max(0, x0);           x1 = min(W-1, x1);      // clamp 到地图
if (x1-x0 < 8 || y1-y0 < 8) return false;          // 太小 ⇒ 放弃，走纯跟踪
```

**② 新建栅格 + 新原点**（外扩 `P=2` 格）

```cpp
gw = (x1-x0+1) + 2P;   gh = (y1-y0+1) + 2P;
gox = ox + x0*res - P*res;        // 原点向左下各挪 2 格
goy = oy + y0*res - P*res;
gdata.assign(gw*gh, 0);           // 初值 0 = 障碍 ⇒ 外圈 2 格天然是墙
```

**③ 填内容**

```cpp
occ = (map[si] == 0 || map[si] == 100);       // 静态：只有 0/100 算障碍
if (!occ && mask_ok) occ = mask[si] > 50;     // 叠加动态
gdata[(y-y0+P)*gw + (x-x0+P)] = occ ? 0 : 255;  // 0=障碍, 255=自由
```

### 5.3 和 T-DT 的局部地图对比

| | T-DT `testKastar` | 我们 `buildLocalGrid` |
|---|---|---|
| 范围 | **整张图** | 机器人周围 **±16 m**（clamp 地图） |
| 原点 | 固定 `(0,0)` | **每周期重算**（跟着车走，且 −P·res） |
| 索引 | `x*20`（20 px/m） | `(x-gox)/res` |
| 边框 | 图自带黑墙 | **外扩 2 格 = 障碍** |
| 动态障碍 | 无 | `/plan_mask` 叠加 |
| 膨胀 | 无 | **无**（0/255 二值） |
| 尺寸 | 560×300 = 168k 格 | 出生点处 18.2×15.7 m ≈ 114k 格；满窗 640×640 ≈ 410k 格 |

### 5.4 出生点处的实际数值

```
地图       x[-19.90, 8.10]  y[-8.89, 6.76]   28.0 × 15.7 m
窗口       x[-10.15, 8.05]  y[-8.89, 6.71]   18.2 × 15.7 m
格子       114,245  /  全图 175,280  = 65%
```

### 5.5 三个要注意的点

1. **每周期重锚**：`gox_` 随机器人变 ⇒ 同一世界点在相邻周期的栅格索引不同，
   Kastar 的起点量化相位会跳（亚格级抖动）。
2. **贴边裁剪**：`clamp` 后窗口可能变小，小于 8 格直接 `return false` ⇒ 掉纯跟踪。
3. **没有膨胀**：0/255 二值、不做机器人半径膨胀 ⇒ 轨迹可能贴墙走；
   唯一的间隙来自静态图本身的墙厚 + mask。
4. **mask 没有时效检查**：`mask_ok` 只校验分辨率/原点/尺寸，
   `mask_builder` 停发后会一直用最后一帧。


## 6. 改为「整图」做局部规划（2026-11-08 实测）

按用户要求，局部规划不再裁窗口，改为**与 T-DT 一样的整张图**：

- 新参数 `use_full_map`（默认 `true`）；
- `buildFullGrid()`：`gw=W+2P, gh=H+2P`，原点固定 `(ox−P·res, oy−P·res)`，
  内容 = `/map`（`>=65` 算障碍）叠加 `/plan_mask`；
- 只在 `/map` 或 `/plan_mask` 更新后重建（`grid_dirty_`），**不再每周期重填**；
- 因此**没有"每周期重锚"的抖动**，栅格坐标与 T-DT 一致。

### 实测

| 配置 | 结果 | 用时 | 终点误差 |
|---|---|---|---|
| 窗口 ±16 m | SUCCEEDED | 51.1 s | 0.26 m |
| **整图（本次）** | **SUCCEEDED** | **26.4 s** | 0.26 m |

Kastar 每周期样本数 336~462，前馈速度 0.54~0.99 m/s。

### ⚠️ 过程中修掉一个我引入的回归

把二值化"照抄 T-DT 的 `==0`"时忽略了 **`OccupancyGrid` 的语义与 PGM 相反**
（`0=free / 100=occupied / -1=unknown`）⇒ 整张图被判成障碍（日志 `障碍 175280 格` = 全图）
⇒ 全局 A* 全部失败、控制器栅格全障碍。现已改回 `>= 65`，PGM 路径仍保持 `px == 0`。

## 7. 更正：51.1 s → 26.4 s 不能全算“窗口改整图”的功劳

三次运行的差异不止一个变量，逐次拆开：

| 运行 | 自建 TF 桥 | 坡度限速 | 局部栅格 | 用时 | 终点误差 |
|---|---|---|---|---|---|
| ① | 有 | **开** | 窗口 ±16 m | 51.1 s | 0.26 m |
| ② | 无（原生 TF） | **关** | 窗口 ±16 m | 28.2 s | 0.29 m |
| ③ | 无 | 关 | **整图** | 26.4 s | 0.26 m |

⇒ **①→②** 主要是“去掉自建桥 + 关坡度限速”（−22.9 s）；
**②→③** 才是“窗口→整图”的贡献（**−1.8 s**，幅度不大）。

之前把 51.1→26.4 全部归因于窗口改动是**错的**，特此更正。

### 窗口→整图真正省掉的是什么

- **每周期重建**：窗口版每个控制周期从 `/map` 重填 114k 格（30 Hz ⇒ ~3.4M 格写/s）；
  整图版只在 `/map`/`/plan_mask` 更新时重建（2 Hz），**重建次数降 15 倍**；
- **每周期重锚**：整图原点固定 ⇒ Kastar 起点量化相位稳定，轨迹不抖。

用时只差 1.8 s，说明**重建开销在 30 Hz 下并没有成为瓶颈**；
真正的收益是**确定性（不再重锚）**和**代码简单**。

## 8. 速度限制的完整清单（2026-11-08）

### 8.1 从里到外共 5 层

| 层 | 参数文件/位置 | 参数 | 值 | 作用方式 |
|---|---|---|---|---|
| ① 规划器 MinSnap | `nav2_params_sim_planner_tdt.yaml` | `max_speed` / `max_acc` | **1.5 / 2.0** | 决定梯形时间分配与轨迹速度剖面 |
| ② 控制器 Kastar | `nav2_params_sim_controller_rm_tdt_local.yaml` | `max_vel` / `max_acc` | **1.5 / 3.0** | 局部轨迹的 v/a 上限 |
| ③ 控制器输出 | 同上 | `max_vel` / `max_w` | **1.5 / 3.0** | `clampd()` 直接夹 \(v_x,v_y,\omega\) |
| ④ **velocity_smoother** | `nav2_params_sim_base.yaml` | `max_velocity` / `max_accel` | **[0.6, 0.6, 1.0] / [4,4,6]** | **实际最紧的一层** |
| ⑤ 底盘插件 | `mecanum_drive_plugin.cpp` | `max_wheel_speed` | 40 rad/s ≈ 2.3 m/s | 轮速上限 |

另外：
- **坡度限速**：`speed_limit_enable: false`（本次已关），不再发 `SpeedLimit`；
- **goal checker**：`xy_tol=0.250 m`、`yaw_tol=3.150 rad`（不限制速度，只判到达）。

### 8.2 cmd_vel 的实际流向

```
controller_server ──/cmd_vel_nav──► velocity_smoother ──/cmd_vel──► fake_vel_transform
                                                                  ──/cmd_vel_chassis──► Gazebo
```

（`navigation_launch.py:237/298`：`cmd_vel → cmd_vel_nav`；`cmd_vel_smoothed → cmd_vel`）

⇒ **最终到轮子上的上限是 `velocity_smoother` 的 0.6 m/s / 1.0 rad/s**，
规划器的 1.5、控制器的 1.5 都被它压住。

### 8.3 想跑快怎么改

只改一处即可：

```yaml
# src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [1.2, 1.2, 2.0]
    min_velocity: [-1.2, -1.2, -2.0]
```

（该文件里注释说明：0.6 是 2026-10-31 为"目标 1.5 m 时车冲出去 4 m、LIO 发散"临时压的；
现在定位换成 zlio 且先验配准稳定，可以放宽。）

## 9. 障碍是怎么判出来的（完整链路 + 每层限制）

### 9.1 感知链路

```
/livox/lidar (CustomMsg)
  └─► livox_cloud_extrinsic ──► /livox/lidar/pointcloud (PointCloud2, livox_frame)
        └─► patchwork（去地面）
              │  参数: sensor_height 0.188, min_range 0.20, max_range 20.0,
              │        th_dist 0.13, num_iter 3, num_zones 4, RNR 0 / RVPF 1 / TGR 1
              │  自击掩膜: 5 个车身 AABB + 近场死区 r_xy≤0.34 m 且 z≥−0.158 m
              ├─► /segmentation/obstacle   （障碍点云）
              └─► /segmentation/ground     （地面点云）
                    └─► cloud_crop（3 个 AABB，base_link）──► /segmentation/obstacle_cropped
                          ├─► pointcloud_to_laserscan ──► /scan ──► 局部/全局 costmap 的 obstacle_layer
                          └─► mask_builder（z∈[0.10,1.20] 投 2D @0.05）──► /plan_mask
                                ├─► rm_tdt_planner（mask 层, 1 s 衰减 ×16）
                                └─► rm_tdt_local_controller（叠加到 Kastar 栅格）
```

### 9.2 每层的"限制/盲区"

| 环节 | 关键限制 | 后果 |
|---|---|---|
| patchwork | `min_range 0.20`、`max_range 20.0`、`th_dist 0.13` | 20 m 外不算障碍；0.2 m 内不看 |
| patchwork 自击掩膜 | 5 个 AABB + `r_xy≤0.34, z≥−0.158` 死区 | 车身附近 0.34 m 不判障碍 |
| cloud_crop | 3 个 AABB（base_link） | 裁掉车体/远端 |
| **p2l** | `range_min 0.05`、`range_max 10.0`、`min_height −1.0 / max_height 1.0`（雷达系 ⇒ 离地约 [−0.77, +1.23] m）、`angle_increment 0.0043`、`target_frame ""`（不做 TF） | **10 m 外没有 /scan**；高于 1.23 m 的障碍不进 `/scan` |
| mask_builder | `z∈[0.10, 1.20] m`、0.05 m 栅格、`>50` 算障碍 | **只在 0.1~1.2 m 高度带内的点**进 mask；低于 0.1 m（如 5 cm 矮沿）或高于 1.2 m 的不进 |
| 静态图 `/map` | 手画 PGM，墙=0 | 与实时感知无关，永久存在 |
| 规划器 mask 层 | `duration 1.0 s`、`maxCount 16` | 1 s 后自动失效 |
| costmap obstacle_layer | `obstacle_max_range 6.0`、`raytrace_max_range 6.0` | 只影响 nav2 的 costmap，**Kastar 不读它** |

### 9.3 关键结论：两条"障碍信息"各走各的

| 消费者 | 用哪份障碍 | 高度带 | 距离范围 |
|---|---|---|---|
| **rm_tdt_planner（A*）** | `/map` + `/plan_mask` | 0.10~1.20 m | mask 不限距（由 patchwork 20 m 决定） |
| **rm_tdt_local_controller（Kastar）** | `/map` + `/plan_mask` | 同上 | 同上 |
| nav2 global/local costmap | `/map`(static) + `/scan`(obstacle) + stvl | p2l 高度带 | `/scan` ≤ 10 m |
| goal checker | 不判障碍 | — | — |

⇒ **Kastar 的障碍 = 静态图 + mask（0.1~1.2 m 高度带）**；
`/scan`（p2l）只喂给 nav2 的 costmap，**不参与 Kastar**。
所以：**低于 0.1 m 的矮沿、高于 1.2 m 的悬挂物，Kastar 都看不见**（静态图里没有的话）。

## 10. 控制器侧根因：速度被**旋转了两次**（2026-11-08）

### 10.1 现象

规划已经正常（`11 点, 时长 5.14 s, snap=1`），但车**朝反方向走**：

```
robot (0.00,0.03) → (-0.22,-0.35) → (-0.46,-0.12)     目标在 (-0.70, +2.76)
dyaw 8.65° → 108.83° → 101.74° → 96.42°              （车头已转向目标 ~104°）
dist 2.82 → 3.15 → 2.95 → 2.89                       （越走越远）
```

### 10.2 根因：三段代码各转一次，净效果是"把 map 系速度当车体系分量"

```cpp
// ① zlio twist（传感器系）→ 世界系：正确
vx_ = cos(yaw)*t.x - sin(yaw)*t.y;

// ② 传给 Kastar 的初速度：又乘了一次 R(yaw_cf)  ← 多余
Eigen::Vector2d v0(cos(yaw_cf)*vx_ + sin(yaw_cf)*vy_, -sin(yaw_cf)*vx_ + cos(yaw_cf)*vy_);

// ③ Kastar 输出速度（已在栅格=map 系）：又乘了一次 R(yaw_cf)  ← 多余
v_world_x = cos(yaw_cf)*st(2) - sin(yaw_cf)*st(3);

// ④ 输出到车体系：乘 R(-yaw)                                    ← 正确
vx_b = cos(yaw)*v_world_x + sin(yaw)*v_world_y;
```

②③④ 合起来：`vx_b = R(-yaw)·R(+yaw)·v_grid = v_grid`
⇒ 命令里的"车体 vx/vy"其实是 **map 系速度分量**。

于是车头转到 θ 时，实际世界方向 = θ + 期望方向：

- θ=0：正常；
- θ=104°（正对目标）：实际往 **208°** 走 ⇒ **远离目标**。

### 10.3 修复

```cpp
v_world_x = st(2);
v_world_y = st(3);
Eigen::Vector2d v0(vx_, vy_);
```

### 10.4 实测（目标 (-0.67, 2.80)）

| | 修复前 | 修复后 |
|---|---|---|
| 轨迹 | (0,0) → (-0.46,-0.12)，越来越远 | (0,0) → (0.25,0.98) → (-1.39,2.26)，**朝目标接近** |
| 结果 | 卡住 | **SUCCEEDED 15.1 s** |

### 10.5 仍待解决

- Kastar 失败率偏高：`kastar_ok=108 / fail=223 / fallback=269`（约 2/3 走纯跟踪）；
- 终点有过冲（x 到 -1.39，目标 -0.70）。

## 11. 把 T-DT 轨迹里"没用到的部分"用起来（控制器，2026-11-08）

### 11.1 之前只用 1/3

`/trajectory_plan` 里有三样东西：

| 字段 | 之前 | 现在 |
|---|---|---|
| `x, y` | 只取"前视 4 m 那一个点" | 建**弧长表** s(x,y) |
| `vx, vy` | **完全没用** | **速度律**：`v_ref = v(s_now + 0.4)` |
| `t` | **完全没用** | 用于判定新鲜度；弧长参数化代替 |
| `corridor` | 只打包，没用 | 仍未用（可做安全夹取） |

### 11.2 新的控制律（`use_plan_track`，默认 true）

```
① 建表：s[i]（弧长）、v[i]=|(vx_i,vy_i)|
② 投影：车→最近 plan 点 ⇒ s_now（单调，允许回退 0.3 m）
③ 方向：前视点 s_dir = s_now + La，La = min(lookahead, max(0.6, 0.5·s_rem))
④ 速度：v_ref = v(s_now + 0.4)，再套 v_cap = sqrt(2·decel·s_rem)+0.03
⑤ 航向：yaw_ref = 车→前视点方向；w = k_θ·Δψ
```

**末端 v(s)→0 是 MinSnap 的终点速度约束给的**，不需要额外减速逻辑。

### 11.3 实测（目标 (-0.67, 2.80)）

```
SUCCEEDED  用时 82.6 s   终点 (-0.624, 2.518)   误差 0.29 m
[tdt-local] track: s=0.03/0.30 v=0.01 La=0.60 剩 0.27 m   ← 末端自动减速到 0.01 m/s
```

对比前一版（Kastar 单点前馈）：

| | Kastar 前馈版 | 轨迹跟踪版 |
|---|---|---|
| 结果 | SUCCEEDED 15.1 s | SUCCEEDED 82.6 s |
| 终点误差 | 过冲 0.7 m（x 到 -1.39） | **0.29 m，无过冲** |
| 末端速度 | 仍 ~1 m/s | **0.01 m/s** |
| Kastar 兜底 | 2/3 周期 | **不依赖**（track 优先） |

**慢换稳**：速度律现在跟随优化后的剖面（0.7 m/s 量级），要更快就调 `track_decel` / `lookahead`。

### 11.4 走廊（`corridor`）也接上了

`/trajectory_plan.corridor` 现在是**前视点的安全夹取盒**：

```cpp
// ① 只挑非退化盒（边长 > 0.2 m）：直线路径的走廊只有两个退化端点盒，自动跳过
// ② 找离前视点最近的盒（1.2 m 内）
// ③ 前视点夹进盒内（内缩 5 cm）⇒ 指令方向不会把车带出 MinSnap 迭代出来的安全域
dx_pt = clamp(dx_pt, bx0+0.05, bx1-0.05);
dy_pt = clamp(dy_pt, by0+0.05, by1-0.05);
```

- 参数 `use_corridor`（默认 true）；
- 直线路径（走廊只有退化端点盒）⇒ 不夹取，行为与之前完全一致；
- 需要绕路的曲线路径 ⇒ 走廊是真正的方盒，夹取生效。

### 11.5 实测（目标 (-0.67, 2.80)，直线路径）

```
SUCCEEDED 用时 82.0 s   终点 (-0.624, 2.519)   误差 0.29 m
track: s=0.00/0.25 v=0.01 La=0.60 剩 0.25 m     ← 末端减速正常
```

与接走廊前一致（82.6 s / 0.29 m）——符合预期：直线路径没有可用走廊，夹取是空操作。



### 来源：`tdt_consumption_chain.md` —— 三条链的消费链路对比（T-DT / zlio / 我们）


> 2026-11-08。标注：【代码】= 读源码/配置确认；【实测】= 今天跑出来的；【推断】= 未直接验证。

## 0. 三条链各自"消费什么"

| | T-DT 上游 (`third_party/tdt-nav-kit`) | zlio 上游 (`third_party/z-localization`) | 我们（当前在跑） |
|---|---|---|---|
| 输入 | **一张 PNG 文件** + 起终点坐标 | `/livox/lidar` + `/livox/imu` | 仿真传感器 → zlio → nav2 |
| 触发 | `main()` 里 1000 次随机测试 | ROS 回调 | ROS action（`navigate_to_pose`） |
| 中间产物 | `std::vector<Eigen::Vector2f>` 路径、`SolveOutput` | `FrontendFrame` / `FrontendOdometry` | `TrajectoryPlan` / `nav_msgs::Path` |
| 输出 | `cv::imshow` + 控制台计时 | `/localization/odometry`、`/tdt_lio_*`、TF | `/cmd_vel` → `/cmd_vel_chassis` |
| 下游 | 无（人眼看） | 旧 `robot_navigation` 或 nav2 | nav2 行为树 |

## 1. zlio 源码与配置：我们改了多少

### 1.1 源码

```
$ git -C third_party/z-localization status
 M src/frontend/apps/node.cpp          # +6 行（一条 init 诊断日志）
```

**前端/后端算法一行未改**（诊断日志是我们加的，可随时删）。

### 1.2 后端配置（我们的 `tools/configs/zlio_backend_mapping_save.yaml`）

203 个参数里 **11 个不同**：

| 参数 | 上游 | 我们 | 为什么 |
|---|---|---|---|
| `common/body/rotation/roll_deg` | −14.94 | **0** | 真机雷达斜装；仿真平装 |
| `common/body/rotation/pitch_deg` | 0.523 | **0** | 同上 |
| `common/body/rotation/yaw_deg` | −150.6 | **0** | 同上 |
| `common/body/offset/x` | −0.098 | **0** | 同上 |
| `common/body/offset/y` | 0.094 | **0** | 同上 |
| `uwb/enabled` | true | **false** | 仿真没有 UWB（否则状态永远 Initializing） |
| `trajectory/save_directory` | `pcd/runtime_zlio` | 绝对路径 | 方便找 |
| `visualization/enabled` | false | **true** | 调试看 3D 图 |
| `visualization/publish_runtime_map` | false | **true** | 同上 |
| `visualization/publish_paths` | false | **true** | 同上 |
| `visualization/publish_map_axes` | false | **true** | 同上 |

`config/lio.yaml`（前端参数）：**未改**（只用命令行覆盖队列大小/imu 话题）。

## 2. 消费链路逐 topic（我们当前）

| 话题 | 类型 | QoS | 频率【实测】 | 生产者 | 消费者（消费它做什么） |
|---|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | sensor(best effort) | ~2 Hz 墙钟（10 Hz 仿真） | 仿真插件 | 前端解码成 Scan（用 `timebase + offset_time`） |
| `/livox/imu` | `sensor_msgs/Imu` | sensor | 100 Hz 仿真 | 仿真插件 | `imu_prep` |
| `/livox/imu_level` | `sensor_msgs/Imu` | sensor | 同上 | `imu_prep` | 前端（预积分） |
| `/frontend/frame` | `FrontendFrame` | **QoS(64) reliable** | 2 Hz | 前端 | **后端**（去畸变点云 + 后验位姿 → 建图/配准） |
| `/frontend/odometry` | `FrontendOdometry` | **QoS(200) reliable** | 10~40 Hz | 前端 | **后端**（高频位姿预测 → 对外位姿/TF 平滑） |
| `/frontend/session` | `FrontendSession` | QoS(1) transient_local | 事件 | 前端 | 后端（epoch/失败通知） |
| `/localization/odometry` | `nav_msgs/Odometry` | depth 200 | ~10 Hz | 后端 | **控制器**（作为机器人位姿）+ TF 桥 |
| `~/localization_pose` | `PoseWithCovarianceStamped` | QoS(20) reliable | 同上 | 后端 | **`zlio_tf` 桥 → TF** |
| `~/localization_state` | `std_msgs/String` | QoS(1) transient_local | 状态变化 | 后端 | 人/工具（`tools/scripts/zlio_status.sh`） |
| `/tdt_lio_pose` | 兼容接口 | — | 同上 | 后端（**legacy.output**） | **无（我们没用）** |
| `/tdt_lio_twist`、`/tdt_lio_path`、`/LAM_danger_situation` | 兼容接口 | — | 同上 | 后端 | **无（我们没用）** |
| TF `odom→imu` | TF | — | 10 Hz | **前端**(`publish_tf=true`) | — |
| TF `map→odom` | TF | — | 10 Hz | 后端 | — |
| TF `map→body` | TF | — | 10 Hz | 后端(legacy) | — |
| TF `zlio_map→base_link` | TF | — | ~10 Hz | **我们的 `zlio_tf` 桥** | nav2 costmap / 控制器 |
| TF `map→zlio_map` | TF | — | 静态 | `static_transform_publisher` | 同上 |
| `/map` | `OccupancyGrid` | transient_local | 一次 | `map_server`（手画 PGM） | 全局代价图、TDT 规划器 |
| `/plan_mask` | `OccupancyGrid` | QoS(1) | ~2 Hz | `mask_builder`（patchwork 点云） | 规划器（动态层）、控制器 |
| `/scan` | `LaserScan` | sensor | ~2 Hz | `pointcloud_to_laserscan` | 局部代价图 obstacle_layer |
| `/trajectory_plan` | `rm_plan_msgs/TrajectoryPlan` | depth 1 | 2 Hz | `rm_tdt_planner` | `rm_tdt_local_controller`（取前视点） |
| `/cmd_vel` | `Twist` | depth 1 | 30 Hz | 控制器 | `fake_vel_transform` |
| `/cmd_vel_chassis` | `Twist` | — | 30 Hz | `fake_vel_transform` | Gazebo mecanum 插件 |

## 3. 关键差异（消费语义层面）

### 3.1 地图：T-DT 每次 `cv::imread`，我们缓存

```cpp
// T-DT main.cpp::test() —— 每次调用都重新读 PNG
cv::Mat map = cv::imread("../images/rmuc2025.png", cv::IMREAD_GRAYSCALE);
```

我们只订阅一次 `/map`（transient local），存成栅格后复用；每次规划只在上面跑 A*。

### 3.2 位姿：T-DT 不需要（benchmark），zlio 走 TF + `/tdt_lio_*`，我们走 TF + `/localization/odometry`

- **zlio 上游的"标准消费方式"**：`/tdt_lio_pose`（legacy 兼容接口）或 `map→odom` + `odom→imu` TF；
- **我们当前**：`lio:=none` ⇒ 没有 `odom→base_link`，所以额外写了 `zlio_tf` 桥，把 `~/localization_pose` 转成 `zlio_map→base_link`；
- 后端**一直在发** `/tdt_lio_pose`、`/tdt_lio_twist`、`/LAM_danger_situation`（legacy 默认开）——**我们没消费**。

> 如果以后要接 T-DT 自己的 navigator，这条现成的 `/tdt_lio_pose` 就是对接点，不需要我们自己造。

### 3.3 局部障碍：T-DT 只看静态图，我们多了 STVL + mask

- T-DT：Kastar 在**同一张 PNG** 上查碰撞；
- 我们：局部代价图 `stvl_layer`（实时体素）+ `/plan_mask`（patchwork 动态障碍）叠加。

### 3.4 限速：我们多一层

`ground_segmentation` 通过 nav2 `SpeedLimit` 发布坡度/台阶限速；T-DT 没有这一层。

### 3.5 频率

| 环节 | T-DT | 我们 |
|---|---|---|
| 全局规划 | benchmark 里 4.17 ms/次（A*），无固定频率 | nav2 `expected_planner_frequency: 2.0` |
| 局部控制 | benchmark，无频率 | controller_server 30 Hz |
| 位姿 | 无 | 前端 10~40 Hz + 后端输出 ~10 Hz |

## 4. 结论

1. **算法**（A*/Kastar/MinSnap/SFC）：与 T-DT 上游**同一份代码**（0/0/0/2 行差异）；
2. **zlio**：源码只多 6 行诊断日志，配置 203 项里改 11 项（外参/UWB/可视化）；
3. **消费链路**：
   - 我们**没用到** zlio 现成的 `/tdt_lio_*` 兼容接口，而是自己写了 TF 桥；
   - 位姿/地图/障碍/限速这四条，我们的消费方式比 T-DT benchmark **更"ROS 化"**；
   - 若要完全贴 T-DT，最短路径是：消费 `/tdt_lio_pose` + 用它的 `map→body` TF，去掉自己的 TF 桥。

## 5. 改为「T-DT 原生消费」的 A/B（2026-11-08 实测）

改动：
1. 后端 `common.body.frame: body → base_link` ⇒ **legacy 兼容接口直接发 `map→base_link`**；
2. launch 加 `native_tf` 开关：
   - `native_tf:=true` ⇒ **不启**自建 `zlio_tf` 桥与 `map→zlio_map` 静态对齐；
   - `native_tf:=false` ⇒ 回到之前的自建桥；
3. 顺带按用户要求**关掉** `speed_limit_enable`（去掉坡度/台阶限速这一层）。

### 实测对比

| 接线方式 | 结果 | 用时 | 终点误差 | 额外节点 |
|---|---|---|---|---|
| A：自建 `zlio_tf` 桥（native_tf:=false） | SUCCEEDED | 51.1 s | 0.26 m | `zlio_tf` + 静态对齐 |
| **B：T-DT 原生 TF（native_tf:=true）** | **SUCCEEDED** | **28.2 s** | 0.29 m | **无**（只有 `backend_node`） |

启动位姿：`map→base_link = (-0.000, 0.015, 0.013)`（与出生点一致）。

### 结论

- **可以直接消费 T-DT 的 TF**：只要把 `common.body.frame` 改成 `base_link`，后端自己就发 `map→base_link`，
  我们的桥和静态对齐都可以删掉；
- 到达精度两者相当（0.26 vs 0.29 m），但**原生接线少两个节点、少一层 TF 变换**；
- 用时从 51.1 s 降到 28.2 s（这里同时关掉了坡度限速，两者共同作用）。

## 6. 对照一（zlio）的最终结论

| 层 | 结论 |
|---|---|
| **算法源码** | 前端/后端**一行未改**；`git status` 只有 `src/frontend/apps/node.cpp` 多 **6 行诊断日志**（打 init 位姿），随时可删 |
| **后端配置** | 203 项里改 **11 项**：外参 5 项（真机斜装→仿真平装）、`uwb.enabled`（仿真无 UWB）、`save_directory`（绝对路径）、可视化 4 项（调试用） |
| **前端配置** | `lio.yaml` **未改**；只用命令行覆盖队列（8192/2M）、`imu_topic`、`imu_history_seconds`、`maximum_imu_interval` |
| **用法** | 多一层 `imu_prep`（重力对齐/补协方差/补空隙，**仿真专用**）；`lio:=none`（这条链不用我们的 LIO） |
| **位姿消费** | 上游本来就有 `output.legacy`（`/tdt_lio_pose`、`/tdt_lio_twist`、`/LAM_danger_situation`、`map→body` TF）；我们把 `body.frame` 改成 `base_link` 后**直接用它发 `map→base_link`**，自建 `zlio_tf` 桥与静态对齐已删除 |
| **仿真侧修复** | 物理 1 ms、IMU 协方差、逐点帧内时间、队列大小 —— **都不是 zlio 的问题**，是仿真数据质量 |


---

## §D 四方对比（含 BIT）


### 来源：`four_way_comparison.md` —— 四方对比：T-DT / BIT / 我们（改造前）/ 我们（现在）


> 目标：把四条链放在同一张表里，看清「谁提供了什么、我们改了什么、还剩什么」。
>
> 记号：
> - **T-DT** = `third_party/tdt-nav-kit`（MIT，本仓库一行未改）
> - **BIT** = `Walker152/navi_minco_bit`（Apache-2.0 根，`rog_map` 为 BSD；我们**只读设计，未复制代码**）
> - **我们(前)** = 接入 T-DT 之前的 nav2 原生栈（本仓库仍保留 `nav2_params_sim_*` 各套）
> - **我们(今)** = 当前链（`planner:=tdt` + `nav:=rm_tdt_local`）
>   - 障碍层三选一：`mask_impl:=mask | terrain | rogmap`
>   - **ROGMap 已 vendor 到 `src/vendor/rog_map`（LGPL-3.0-or-later，见 NOTICE）并以静态库链进规划器**，
>     `use_rog_map:=true` 时进程内构造 + `MapRegistry` 逐格查询

---

## 0. 四者各提供什么（先建立坐标系）

| 能力 | T-DT | BIT | 我们(前) | 我们(今) |
|---|---|---|---|---|
| 定位 | ❌ | ✅ Point-LIO | ✅ zlio / point_lio | ✅ zlio |
| 世界系点云 | ❌ | ✅ `/cloud_registered_full` | ✅ | ✅ `/localization/lio_world_cloud` |
| 3D 占据层 | ❌ | ✅ **ROGMap** | ❌ | ✅ **ROGMap（已 vendor，进程内）** 或 terrain_layer |
| 地形语义 | ❌ | ✅ ProjectionLayer | ❌ | ✅ **ROGMap ProjectionLayer**（同一份） |
| ESDF | 内部（仅 A\* 代价） | ✅ Signed ESDF（规划/控制/恢复共用） | ❌ | ✅ **ROGMap 自带（`esdf_en`，默认关）** |
| 全局规划 | ✅ A\*+势场 | ✅ SMAC/A\* | ✅ SMAC2D | ✅ A\*（两级） |
| 轨迹优化 | ✅ SFC + MinSnap | ✅ MINCO（软代价） | ❌ | ✅ SFC + MinSnap |
| 局部规划/控制 | ✅ Kastar（无胶水） | ✅ SE(2) MPC | ✅ DWB/RPP/MPPI/TEB/rm_mpc | ✅ Kastar（自建弧长胶水） |
| 动态障碍 | ❌（只有接口） | ✅ | ⚠️ 走 costmap（点云层） | ✅ mask → 六处消费者 |
| 行为树/恢复 | ❌ | ✅ BT + FSM + ESDF 脱困 | ✅ nav2 默认 BT | ❌ 无 |
| 工程完成度 | 算法库 + 基准 main | 实车全栈 | 通用 nav2 | 仿真验证中 |

---

## 1. 四条数据流并列

### T-DT（算法库，无 ROS 感知）

```
PNG 地图 ──► YAstar（A*+势场）──► simplifyPath(0.1)
                                   └─► trapezoidalTimeAllocation
                                        └─► SfcSquare 走廊(2.5) + MinimumSnap
                                             └─► OSQP ──► SolveOutput{path,success}
（控制器侧另有 KinodynamicAstar，但要调用者自己把轨迹时间喂进去）
```

### BIT

```
双 MID-360 ─► 驱动侧融合 ─► Point-LIO ─┬─ /aft_mapped_to_init（位姿）
                                      └─ /cloud_registered_full（稠密去畸变，世界系）
                                            │
                                            ▼
                        ROGMap（3D 0.05 m，滑窗 10×10×1.5，hit/miss，decay 0.8/1.2 s）
                          ├─ ProjectionLayer（FREE/PASSABLE/OCCUPIED/UNKNOWN + 先验保守合并）
                          └─ Signed ESDF（截断 6.0/−3.0）
                                │
                     ┌──────────┴──────────┐
                     ▼                     ▼
            SMAC/A* 前端  ──►  MINCO 后端      SE(2) MPC（100 Hz）
            （costmap+ESDF 势场）  （软代价：pos/vel/acc/att）   （凝聚 QP，qpOASES）
                     └──────────┬──────────┘
                                ▼
                        /opt_path（P/V/A/J/Yaw 前馈）──► /cmd_vel_mpc
                                                        ▲
                                              BehaviorTree.CPP 决策
```

### 我们(前)：nav2 原生

```
p2l /scan ──► global_costmap（static + obstacle + stvl）
                    │
                    ▼
               SMAC2D（GlobalPlanner）
                    │  /plan
                    ▼
        local_costmap（rolling window）+ DWB / RPP / MPPI / TEB / rm_mpc
                    │
                    ▼
              velocity_smoother ──► /cmd_vel
```

### 我们(今)

```
zlio /localization/lio_world_cloud（map 系，与位姿同源）
        │
        ▼
terrain_layer（3D 0.10 m 体素：hit +4 / raycast miss −1 / 每帧衰减 −1 / 自车过滤 0.35 m）
        ├─ 逐柱语义：surface 0.10 / wall 0.20 / tunnel 0.25~0.40 / z −0.20~1.50
        ├─ /plan_mask（只把 OCCUPIED 写 100）
        └─ /terrain_class（0..3，调试）
        │
        ▼
TdtPlanner（适配层）
        ├ L1 骨架：/map 上 A*，缓存（目标动 0.3 m 或 5 s 重算）
        ├ L2 局部：/map ∪ mask，8 m → A*（势场）→ DP → 梯形 → SFC → MinSnap
        └ 零容差门控（Bresenham 逐格；穿墙 ⇒ 退 A* 栅格路径）
        │  /plan（局部 + 骨架尾段）   /trajectory_plan（x/y/vx/vy/t）
        ▼
TdtLocalController（适配层）
        ├ 整图栅格 /map ∪ mask（564×317，脏标记重建）
        ├ 2D ESDF（倒角，两遍，~0.5 ms）+ 参考点推离（≥0.20 m）
        ├ 弧长 s[i] / v(s) / 前视 4 m
        └ Kastar（4D x,y,vx,vy；maxNodes 15 万；sampleTime 0.01）
        │  /cmd_vel_nav
        ▼
velocity_smoother[3,3,3] → /cmd_vel → fake_vel_transform → /cmd_vel_chassis → 底盘
```

---

## 2. 逐模块深对比

### 2.1 静态地图

| 项 | T-DT | BIT | 我们(前) | 我们(今) |
|---|---|---|---|---|
| 来源 | 自带 PNG | Nav2 YAML/PGM + PCD | PGM / costmap | **PGM（手画 + 加粗 1 格）** |
| 分辨率 | 0.05 | 0.05 | 0.05 | 0.05 |
| 原点 | **(0,0)**（硬假设） | `map`→`camera_init` 靠 TF | 任意 | (−19.9,−8.89)，**内部转局部** |
| 墙厚 | ~12 格 | 由 PCD 投影 | — | **2.8 格**（原图 2 格） |
| 与在线数据合并 | ❌ 无 | **先验占据强制并入；先验 free 不清在线障碍** | costmap 图层叠加 | **`a_occ = map ∪ mask`**（同行语义） |

**结论**：我们与 BIT 的「先验占据硬、先验 free 不覆盖在线」语义一致；T-DT 没有这个语义（它根本没有在线数据）。

---

### 2.2 动态障碍：来源 / 表示 / 时间

| 项 | T-DT | BIT | 我们(前) | 我们(今) |
|---|---|---|---|---|
| 来源 | ❌ 无 | `/cloud_registered_full`（同源） | `/scan`（p2l）→ stvl 体素层 | `lio_world_cloud`（同源） |
| 表示 | 2D mask 接口 | **3D 概率占据 0.05 m** | 3D 体素（stvl）+ 2D 栅格 | **3D 体素 0.10 m**（2.5D 输出） |
| 证据模型 | 二值 | **hit/miss 概率** | 概率（stvl 自带） | **log-odds**：hit +4 / miss −1 / 上限 30 / 下限 −8 |
| 空闲清空 | ❌ | ✅ raycast | ✅ stvl raycast | ✅ raycast（每 8 点 1 条） |
| 时间衰减 | 整层过期（1.0 s，4 层） | **逐格 decay 0.8 s / clear 1.2 s** | stvl 衰减 | **每帧 −1（5 Hz ⇒ ~0.4 s 记忆）** |
| 语义 | 只有「障碍」 | **FREE/PASSABLE/OCCUPIED/UNKNOWN** | 只有「障碍」 | **同上 4 类 + 净空判据** |
| 自车处理 | ❌ | 靠 blind center | self_mask（patchwork） | **map 系半径 0.35 m 过滤** |
| 消费方 | A\* | 规划 + 控制 + 恢复 | costmap + 局部控制 | **A\* / 走廊 / MinSnap / 门控 / SDF / 控制器**（六处同一份） |

**这是差距最大的一列**：T-DT 完全没有这一层；我们(前)有但只进 costmap；BIT 与我们(今)都有语义，但 BIT 是 3D 概率场，我们是 2.5D 体素 + 分类。

---

### 2.3 全局搜索

| 项 | T-DT | BIT | 我们(前) | 我们(今) |
|---|---|---|---|---|
| 算法 | 8 邻域 A\* + 势场 | SMAC/A\*（costmap + ESDF 势场） | SMAC2D | 8 邻域 A\* + 势场 |
| 代价 | `1 + 0.5/(0.1+d)` | costmap 代价 + ESDF 项 | costmap（inflation） | `1 + 0.5/(0.1+d)` |
| 输入栅格 | 静态 + mask | ROGMap 投影 + 先验 | costmap（**含 stvl 实时层**） | `map ∪ mask` |
| 重规划 | 调用者决定 | 20 Hz（含局部裁剪） | 1~2 Hz | **2 Hz + 两级** |
| 结构 | 单级（起点→终点） | 前端搜索 + 6 m 局部裁剪 | 单级 | **L1 骨架 + L2 8 m 窗口** |

**关键差异**：
- BIT 的「planner」是 **6 m 前视的滚动优化器**（20 Hz），不是一次性全局；
- 我们(前) 的 SMAC2D 是**真·全局**（1~2 Hz，全图搜索）；
- 我们(今) 是**两级**：全局骨架低频 + 8 m 局部高频。

---

### 2.4 轨迹优化 / 平滑

| 项 | T-DT | BIT | 我们(前) | 我们(今) |
|---|---|---|---|---|
| 方法 | **SFC 走廊 + MinimumSnap（QP）** | **MINCO（无硬走廊，软代价）** | 无（DWB 直接跟 Path） | SFC + MinSnap |
| 走廊 | 正方形，`maxRange 2.5`，交替迭代 6 轮 | ❌ 无硬走廊（自己列为缺陷） | ❌ | 正方形 2.5，迭代 6 |
| 碰撞 | `lineInObsticle` **容 2 格**（非严格） | 采样校验（只会拒绝，不保证） | costmap footprint | **零容差门控（外挂）** |
| 失败处理 | 无（demo 不检查 success） | 拒绝候选 / 重规划 | 无 | **回退 A\* 简化路径** |
| 输出 | `path`（**无时间**） | `/opt_path`（P/V/A/J/Yaw） | `/plan`（无速度） | `/trajectory_plan`（vx/vy/t） |

---

### 2.5 局部规划 / 控制

| 项 | T-DT | BIT | 我们(前) | 我们(今) |
|---|---|---|---|---|
| 算法 | **Kastar**（4D 采样） | **SE(2) MPC**（凝聚 QP） | DWB / RPP / MPPI / TEB / rm_mpc | **Kastar** |
| 频率 | 调用者决定 | **100 Hz** | 20 Hz（默认） | **30 Hz** |
| 状态 | (x,y,vx,vy) | (x,y,yaw)，控制 (vx,vy,ω) | 因实现而异 | (x,y,vx,vy) |
| 输入 | 带时间的轨迹 | `/opt_path` 前馈 | `/plan` | `/plan` + 自建 `v(s)` |
| 参考 | 轨迹时间戳 | 前馈 + 预测时域 0.5 s | 纯跟踪 | 弧长 + 前视 4 m |
| 贴墙处理 | ❌ | ESDF 梯度（恢复） | inflation（costmap） | **2D ESDF 推参考点（≥0.20 m）** |
| 延迟补偿 | ❌ | ✅ `control_delay_compensation 0.05` | 无 | ❌ |
| 杆臂补偿 | ❌ | ✅ `lidar_offset [0,−0.20]` | 无 | ❌ |

---

### 2.6 输出与执行

| 项 | T-DT | BIT | 我们(前) | 我们(今) |
|---|---|---|---|---|
| 速度系 | 调用者定 | **世界系**（底盘再做二次映射） | 车体系 | 车体系（内部转） |
| 上限 | demo 8 m/s | 3.0 m/s | `max_vel 0.6`（旧） | **3.0 m/s** |
| 加速度 | demo 2.0 | 2.0 | 因实现 | 4.0 |
| 限速层 | 无 | MPC 约束 + speed limit | smoother | smoother `[3,3,3]` |
| 底盘物理上限 | — | — | 2.32 m/s | 2.32 m/s（40 rad/s × 0.058 m） |

---

### 2.7 恢复 / 异常

| 项 | T-DT | BIT | 我们(前) | 我们(今) |
|---|---|---|---|---|
| 卡住恢复 | ❌ | **FSM + ESDF 梯度脱困** | nav2 BT（Spin/BackUp/Wait） | ❌ 无 |
| 规划失败 | 返回空/部分解 | 拒绝 + 重规划 | 报错 | **回退链（简化→A\* 栅格）** |
| 定位失效 | 不涉及 | 依赖 LIO | costmap 停更 | mask 停更（无超时保护） |

---

### 2.8 参数总表

| 参数 | T-DT | BIT | 我们(前) | 我们(今) |
|---|---|---|---|---|
| `max_speed / max_vel` | 8.0 | 3.0 | 0.6 | **3.0** |
| `max_acc` | 2.0 | 2.0 | — | **3.0（规划）/ 4.0（控制）** |
| `corridor range` | 2.5 | — | — | 2.5 |
| `corridor shrink` | 0.0 | — | — | 0.0 |
| `MinSnap order / maxDx` | 6 / 3 | — | — | 6 / 3 |
| `iter_time_limit` | 10.0 | — | — | 10.0 |
| `collision_iter` | 6 | — | — | 6 |
| `simplify` | 0.1 | — | — | 0.1 |
| `strict_collision` | false | — | — | false + 外挂零容差 |
| 前视 | — | 6.0 | 0.5~1.0 | **4.0** |
| 安全距离 | — | 0.35 / 0.25 | 0.55（inflation） | **0.20（ESDF 推参考）** |
| 最大角速度 | — | 5.0 rad/s | 1.0 | **3.0 rad/s** |
| 控制频率 | — | 100 Hz | 20 Hz | **30 Hz** |
| 规划频率 | — | 20 Hz | 1~2 Hz | **2 Hz + 两级** |

---

### 2.9 实测性能

| 指标 | T-DT（上游基准） | BIT（文档） | 我们(前)（旧日志） | 我们(今)（本仓库实测） |
|---|---|---|---|---|
| 规划耗时 | — | 20 Hz 目标 | ~10 ms | **A\* 6 ms；MinSnap 0.1~1 ms；骨架 5~10 ms/5 s** |
| `snap` 成功率 | **0/5（demo 不检查）** | — | — | **1/1（门控后）** |
| 动态层耗时 | — | — | — | **1.3 ms @ 5 Hz** |
| 控制耗时 | — | 100 Hz | 20 Hz | **30 Hz 稳定** |
| 典型路径 | 10 点 / 13~34 m | 6 m 前视 | — | **243 点 / 15.5 s（两级）** |
| 穿墙 | 允许穿 2 格 | — | 依赖 inflation | **零容差（实测落墙 0/4 px）** |

---

## 3. 各自的失效模式（最值得记的部分）

### T-DT

| 失效 | 根因 |
|---|---|
| 规划穿墙 | 非严格碰撞容 2 格；走廊盒内可含墙 |
| `snap=0` | origin 三处约定冲突（只在 (0,0) 成立） |
| 无时间信息 | `SolveOutput` 只有 path |
| 状态不可知 | `success` 只表示「满足当前碰撞检查」，迭代耗尽也可能撞 |

### BIT

| 失效 | 自述 |
|---|---|
| 动态响应不灵敏 | 「受点云/ROGMap 刷新、FSM、旧轨迹复用、新轨迹接受共同影响」 |
| 无硬走廊 | 「主 MINCO 优化没有安全走廊硬约束」 |
| Z 窗口越界 | 基线投影跨度 1.70 m > `map_size.z 1.50` |
| TF 硬编码 | `camera_init→base_link` 的 `[0,0.20,0]` 写在源码里 |

### 我们(前)

| 失效 | 根因 |
|---|---|
| 动态障碍改全局路径 | SMAC2D 吃 costmap（含 stvl） |
| 窄处卡住 | 局部控制器贪心；无脱困 |
| 无地形语义 | costmap 只有 occupied/free |
| 贴墙 | inflation 只影响代价，不保证距离 |

### 我们(今)

| 失效 | 根因 | 严重度 |
|---|---|---|
| 定位一跳规划就废 | 起点落进墙 ⇒ 门控全拒 ⇒ 回退 A\* 栅格 | **高**（仿真里频繁） |
| 尾段对角线切角 | 8 邻域 A\* 不查对角两侧 | 低（5 cm） |
| 无硬性安全距离 | 门控只判「踩不踩」 | 中 |
| 卡住不自救 | 没有 FSM/脱困 | 中 |
| mask 停更 | `publish_untrusted=false` + 无超时保护 | 中 |

---

## 4. 我们(前) → 我们(今) 的逐条 diff

| # | 之前 | 现在 | 提交 |
|---|---|---|---|
| 1 | SMAC2D 全局 | **T-DT A\*（势场）+ SFC + MinSnap** | 系列 |
| 2 | DWB/RPP/MPPI/TEB/rm_mpc | **Kastar** | `933d64d` 等 |
| 3 | 无轨迹平滑 | **MinSnap 平滑** | — |
| 4 | 动态障碍只进 costmap | **mask 进六处消费者** | `6622874` |
| 5 | 无地形语义 | **terrain_layer（3D 体素 + 分类）** | `8eea818` |
| 6 | 单级规划，动态障碍改全路径 | **两级：骨架 + 8 m 局部** | `6622874` |
| 7 | 无穿墙保护 | **零容差门控 + 墙加粗** | `04373d8` |
| 8 | 无贴墙约束 | **2D ESDF 推参考点** | 最新 |
| 9 | 速度 0.6 | **3.0（贴底盘上限）** | 多个 |
| 10 | 无速度前馈 | **`v(s)` 弧长跟踪** | `933d64d` |

---

## 5. 结论

| 维度 | 谁最好 | 我们的选择与理由 |
|---|---|---|
| 动态障碍语义 | **BIT**（3D 概率 + 地形） | 学其判据（4 个参数），自研 2.5D 实现（1.3 ms，够用） |
| 轨迹平滑 | **二者相当**（MinSnap vs MINCO） | 用 T-DT（已有）。BIT 自己承认 MINCO 无硬走廊，我们有 SFC |
| 安全性 | **我们**（零容差门控） | T-DT 容 2 格、BIT 只拒绝候选 |
| 控制精度 | **BIT**（100 Hz MPC + 延迟补偿 + 杆臂） | 暂用 Kastar；若抖动明显可换 MPC |
| 恢复能力 | **BIT**（FSM + ESDF 脱困） | 未做，列为下一步 |
| 工程可维护 | **我们**（两栈都没有的中间层，且适配层仅 1000 行） | — |

**一句话**：

> 我们**用 T-DT 的算法骨架**（A\* + SFC + MinSnap + Kastar），
> **借 BIT 的感知分层与判据**（同源点云 → 占据层 → 语义 → 规划/控制），
> **补上两者都没有的一致性保障**（零容差门控 + 六处同一份障碍 + 两级规划），
> 代价是**还没有 BIT 的恢复闭环和 MPC 精度**。

---

# 附：2026-11-09 最新状态（在本文基础上更新）

| 项 | 变化 |
|---|---|
| 障碍层 | 从 terrain_layer 改为 **ROGMap 进程内嵌入**（`src/vendor/rog_map`，LGPL-3.0-or-later，已 vendor） |
| STVL | **关闭**（两张 costmap 都移除），避免与 ROGMap 双份计算 |
| 负障碍 | **新增**：在 ROGMap 的 ProjectionLayer 内加 8 邻域高度差判据（BIT 列为 future work） |
| 恢复闭环 | **新增**：规划器内 FSM，3 个触发（T1 无进度 / T2 连续失败 / T3 跟踪误差） |
| 逃逸轨迹 | **用 A\*+SFC+MinSnap 求解**（不是手搓直线），过零容差门控 |
| 规划频率 | 2 Hz → **5 Hz** |
| ESDF | 未启用（`esdf_en:false`）；脱困用 16 方向射线采样，不依赖梯度 |

**当前与 BIT 的实质差距只剩**：
1. BIT 的逃逸用 ESDF 梯度 + MINCO 轨迹；我们用方向采样 + MinSnap（同思路，更粗）；
2. BIT 有 ESTOP / 与 BT 联动的完整 FSM；我们只有 NORMAL/ESCAPE/FAILED；
3. BIT 的控制是 100 Hz SE(2) MPC；我们是 30 Hz Kastar。
**我们已经超出的**：
1. 负障碍（BIT 未做）；
2. 零容差轨迹门控（BIT 的 Safety Gate 自述"尚未完项"）；
3. 两级规划（BIT 用 6 m 滚动前视替代）。



---


## 来源 2/19：`tdt/tdt_code_study.md`


> 目的：在动手接 nav2 之前，把他们的**代码 + 文档**读全，避免再靠猜。
> 源码：`third_party/tdt-nav-kit`（MIT，commit `814d5ce`）

## 1. 仓库结构（不含 3rd 子模块）

| 文件 | 行数 | 作用 |
|---|---|---|
| `src/YAstar/yastar.{hpp,cpp}` | 365 / 958 | A*（八邻居）、势场代价、Mask 多层、SDF、路径简化/加密 |
| `src/YAstar/kinodynamicAstar.{hpp,cpp}` | 73 / 290 | 动态可行搜索（状态 `[x,y,vx,vy]`），三次多项式连终点 |
| `src/MinimumSnapOsqp/sfcSquare.{hpp,cpp}` | 121 / 590 | 方形走廊（bound）生成、shrink、方形膨胀 |
| `src/MinimumSnapOsqp/minimumSnap.{hpp,cpp}` | 303 / 1028 | Minimum-Snap QP（OSQP / 闭式）、交替迭代、时间归一化、速度约束 |
| `main.cpp` | 498 | 三个 demo（见 §4） |
| `doc/Astar.md` / `doc/MinimumSnap.md` / `doc/Usage.md` | 92 / 404 / 1667 | 原理 + 全部 API 文档 |

**依赖**：Eigen3、OSQP + OsqpEigen（`3rd/` 子模块）、OpenCV（仅 demo 读图）。
**C++ 标准**：源码用了 `std::span`/`<ranges>` ⇒ 需要 **C++20**（CMakeLists 里 `CXX_STANDARD` 是注释掉的）。

## 2. 四个模块的职责（官方定位）

| 模块 | 定位 | 输出 |
|---|---|---|
| `YAstar` | **前端**：只负责"能不能到" | 路径（栅格点串） |
| `KinodynamicAstar` | **前端+后端合并**：状态含速度，搜索即满足动力学约束 | 时序轨迹 `Sample{t,x,y,vx,vy,ax,ay}` |
| `SfcSquare` | **走廊生成器** | 每个路径点的最大方形 free 区域（可 shrink） |
| `MinimumSnap` | **后端优化**：平滑 + 走廊约束 + 时间分配 + 速度约束 | 平滑路径（位置）+ 分段时长 |

## 3. 文档里的三个关键设计（比代码更重要）

### 3.1 混合搜索：**A\* 全局 + Kastar 局部**（他们推荐的用法）

> "可以先使用 Astar 或 JPS 进行远距离搜索，得到一条大致可行的通路，
> 再选取距离车辆例如 **4 m** 以外的 JPS 路径点作为 Kastar 的局部规划目标。"

- 远端只要"通路"；近端（4 m 内）才需要动力学质量；
- 理由："Kastar 全局搜索约 **2 s**，性能承受不了"。

⇒ 所以 **A\* 与 Kastar 不是纯粹二选一**：官方建议是 **全局 A\* + 局部 Kastar**。
（开源 demo 里没有串起来，但文档明确推荐这个组合。）

### 3.2 交替迭代（MinimumSnap 的碰撞处理）

- MinSnap 是**线性 QP**，走廊约束**不保证**轨迹真的无碰撞；
- 做法：求解 → 采样 → 碰撞检测 → 对碰撞段
  - **强约束**：插入控制点重解；
  - **弱约束**：缩短该段时间分配（让多项式更贴近前端路径点）；
  - 交替使用，循环到无碰撞。
- 作者原话："方形安全飞行走廊…**实际上并不是真正的安全飞行走廊**"（不保证无碰撞、不保证走廊相接）。

### 3.3 时间归一化（数值稳定）

- 每段用 `τ = t/T_i ∈ [0,1]` 构造 Q/A，避免 `T_i^6` 造成病态；
- 速度约束：`v_i = (1/T_i)·dp/dτ`，输入仍是物理速度；
- 作者提醒：只换 `t` 而不改导数/积分缩放，会得到"数值正常但物理速度错误"的结果。

## 4. 三个 demo 的精确调用序列（`main.cpp`）

### demo A：`test()` —— 主链（A\* + MinSnap）

```cpp
YAstar astar(map.cols, map.rows, 0.05f, 0.f, 0.f);
astar.setMap(map.cols, map.rows, map.data);
astar.setMaskNumLayers(4);                                  // 256 层 Mask
astar.setCostField(1.f, [](float x){ return 0.5f / (0.1f + x); });  // 势场
astar.initCostMap();                                        // 稀疏 SDF
auto path = astar.search(start, end);
path = astar.simplifyPath(path, 0.1f);
path = astar.simplifyPathHypot(path);

MinimumSnap ms;
ms.setOrder(6); ms.setMaxDx(3); ms.setDt(0.1);
ms.setMap(map, 0.05f, 0, 0); ms.setTL(10);                  // TL=迭代时间上限
MinimumSnap::SolveInput in;
in.setPath(path);
in.setCollisionCheckIter(6);
in.setMaxSpeed(3.f); in.setMaxAcc(1.f);
in.setMaxCorridorRange(2.5f);   // SFC 最大扩张
in.setCorridorShrink(0.0f);     // 0 也会在迭代中自行收缩
in.setNormTime(true);           // 实验性，作者说"质量不算很好"
in.setBackend(MinimumSnap::Backend::OSQPCorridor);
in.setInitVel({0.f, 0.f});
auto out = ms.solve(in);
```

### demo B：`testSpeed()` —— 带速度约束 + 梯形时间分配

```cpp
// items: 手写控制点；部分点设 vxvy + useVxvy + autoCorridor=false
auto times = MinimumSnap::trapezoidalTimeAllocation(path, 8.f, 2.f);
in.setTimeAllocated(times);          // ★ 时间来源
in.setMaxCorridorRange(2.5f); in.setCorridorShrink(0.f); in.setNormTime(true);
```

### demo C：`testKastar()` —— 只用动态搜索

```cpp
KinodynamicAstar astar(...);
astar.config.maxNodes = 150000; astar.config.sampleTime = 0.01;
auto res = astar.search(start, {0,0}, {0,0}, end);
// 自检：速度/加速度上限、线段碰撞、终点速度=0
```

## 5. 约定与坑（实测/读码确认）

| 项 | 结论 |
|---|---|
| **地图语义** | `YAstar`：**0=free、非0=障碍**；`SfcSquare`：**0=障碍（纯黑）、非0=自由** ⇒ **两者相反** |
| **原点符号** | `YAstar::transformPos = (pos − origin)/res`；`SfcSquare::valueAt = (pos + origin)/res` ⇒ **同一个地图要给两个相反的 origin** |
| **`maxRange` 溢出** | `_getBoundSquare2` 里 `int maxRangeInt = round(maxRange/res)`；传 `1e9` ⇒ 2e10 溢出 ⇒ **所有走廊退化成点**。必须给 ≤ 地图尺度（2~3 m） |
| **地图边界** | `_getBoundSquare2` **不检查边界**（注释：靠膨胀处理）⇒ 必须调 `setSaveDistance(≥1 格)` |
| **C++20** | `std::span` / `<ranges>` ⇒ `-std=c++20`（GCC 11.4 实测可编） |
| **缺 `<iostream>`** | `yastar.cpp` 用了 `std::cout` 但没 include ⇒ 编译时 `-include iostream`（不改上游） |
| **性能（他们表）** | `OSQPCorridor + astar势场`：RMUC2024 avg **1.25 ms** / max 15.9 ms；RMUC2025 avg **6.05 ms** / max 204 ms；闭式 avg 0.13/0.69 ms |
| **JPS 前端** | 有提及但**代码里没有 JPS 实现**（`setCostField` 注释说"jps级别"只是效果类似） |

## 6. 与我们计划的映射（逐项）

| T-DT | 我们 | 状态 |
|---|---|---|
| 二值图 + `setSaveDistance(1格)` | planner 里从 costmap obstacle 层生成 | 待做（P1） |
| `setCostField(1/(0.1+d))` + `initCostMap` | 同参数调用 | 待做（P2） |
| Mask（`setMaskNumLayers` + `setMaskMapTimed`） | 动态障碍生成 Mask | 待做（P2） |
| `simplifyPath(0.1)` + `simplifyPathHypot` | 同调用 | 待做（P2） |
| `trapezoidalTimeAllocation(path, v, a)` | 同调用（时间来源） | 待做（P3） |
| `MinimumSnap(order6, maxDx3, TL10, corridorRange2.5, shrink0, normTime, OSQPCorridor)` | 同参数 | 待做（P3） |
| `(t,x,y,vx,vy)` 推导 | 累计时间 + 差分 | 待做（P4） |
| 控制器 | 我们的速度空间 MPC | ✅ 已有 |
| 混合搜索（A* 全局 + Kastar 局部 4 m） | **可选**：若窄通道需要动力学质量再加 | 待定 |

## 7. 仍未开源（我们只能替代）

| 部分 | 说明 |
|---|---|
| 栅格构建（先验图投影 + 实时障碍） | 用 nav2 costmap 障碍层替代 |
| Mask 维护策略 | 我们按局部 costmap 动态障碍生成 |
| 控制器 | 我们的 MPC |
| 坐标系/标定 | 我们的 `odom_level` |

## 8. 读码后对"完全一致"的新结论

1. **主链 = A\* → simplify → (trapezoidal 时间) → MinimumSnap(OSQPCorridor)**；
2. **Kastar 是"替代前端+后端"的另一种做法**，官方推荐**只在 4 m 内局部使用**（全局太慢，约 2 s）；
3. **SFC 不是硬安全保证**，靠 MinSnap 的**交替迭代**补碰撞；
4. 时间来源是 `trapezoidalTimeAllocation`，不是 Kastar；
5. 我们要"完全一致"，就得连**交替迭代 + 时间归一化 + 速度约束**这些一起照做；
6. 唯一实质替代是**控制器**（他们没开源）。


---


## 来源 3/19：`tdt/tdt_component_map.md`


> 2026-11-08。左列是 `third_party/tdt-nav-kit/src`，右列是我们的调用点。
> **本文件只列 T-DT 自己的组件**（不含 patchwork / p2l / mask_builder 这些外围）。

## 1. 全局规划（YAstar + MinimumSnap + SfcSquare）

| T-DT 组件 | 上游位置 | 我们调用处（`rm_tdt_planner/src/planner.cpp`） |
|---|---|---|
| `YAstar` 构造 | `yastar.hpp:class YAstar` | `:137` `YAstar astar;` + `:138` `setMapping` + `:139` `setOriginPos` |
| `setMap(width,height,u_char*)` | `yastar.cpp:404` | 未用（我们用下面的 int8 重载） |
| `setMap(width,height,res,ox,oy,vector<int8_t>&)` | `yastar.cpp:418` | `:141-142` |
| `setMaskNumLayers` | `yastar.hpp` / 用于 `:438` | `:140` |
| `setMaskMapTimed` | `yastar.cpp:451` | `:150-156`（`duration=1.0s, maxCount=16, value=0`） |
| `setCostField` | `yastar.hpp` | `:157` `0.5f/(0.1f+x)` |
| `initCostMap(sparse)` | `yastar.cpp:760` | `:158` `initCostMap()` |
| `search(start,end)` | `yastar.cpp:802` | `:160` |
| `simplifyPath` / `simplifyPathHypot` | `yastar.cpp:239 / 246` | `:164 / :165` |
| `MinimumSnap::setOrder/setMaxDx/setDt/setMap/setTL` | `minimumSnap.cpp:179/195/183/199/187` | `:176-181` |
| `SolveInput` 各 setter | `minimumSnap.hpp` | `:182-192` |
| `MinimumSnap::solve` | `minimumSnap.cpp:217` | `:196` |
| `trapezoidalTimeAllocation` | `minimumSnap.cpp:281` | `:171`（我们**显式**算后 `setTimeAllocated`） |
| `_solve`（OSQP 走廊） | `minimumSnap.cpp:907` | 经 `solve` 内部 |
| `iterate`（交替迭代） | `minimumSnap.cpp:110` | 经 `_solve` 内部 |
| `lineDecoder`（每段 11 点采样） | `minimumSnap.cpp:564` | 经 `iterate` 内部 |
| `SfcSquare::getCorridor` | `sfcSquare.cpp:83` | 经 `MinimumSnap` 内部 |
| `SfcSquare::_getBoundSquare2` | `sfcSquare.cpp:355` | 同上 |
| `generateQ / generateA / generateLow / generateUp` | `minimumSnap.cpp:317/359/437/472` | 经 `osqpExecute` 内部 |
| `osqpExecute` | `minimumSnap.cpp:782` | 同上 |

## 2. 局部规划（KinodynamicAstar）

| T-DT 组件 | 上游位置 | 我们调用处（`rm_tdt_local_controller/src/controller.cpp`） |
|---|---|---|
| 构造 + `setMap` | `kinodynamicAstar.cpp`（类在 `.hpp`） | `:186-187`（改造成“整图或窗口”两种栅格） |
| `config.maxNodes` | `kinodynamicAstar.hpp:25`（默认 10000） | `:188` 设为 `150000` |
| `config.sampleTime` | `kinodynamicAstar.hpp:24`（默认 0.1） | `:189` 设为 `0.01` |
| `config.maxVel / maxAcc` | `kinodynamicAstar.hpp:18/19`（默认 4/4） | `:190-191` 设为 `1.5 / 3.0` |
| `search(start, startVel, acc, end)` | `kinodynamicAstar.cpp:126` | `:197` |
| `stateTransit / checkTrajectory / sampleTrajectory` | `kinodynamicAstar.cpp:32/39/72` | 经 `search` 内部 |
| `computeShotTraj`（终点直连） | `kinodynamicAstar.cpp:89` | 经 `search` 内部 |

## 3. 两边的“驱动”差在哪

| | T-DT 上游 | 我们 |
|---|---|---|
| 驱动文件 | `main.cpp`（478 行 benchmark） | `planner.cpp` / `controller.cpp`（nav2 插件） |
| 全局入口 | `main.cpp:72 test(start,end)` | `planner.cpp:73 createPlan(start,goal)` |
| 局部入口 | `main.cpp:392 testKastar()` | `controller.cpp:86 computeVelocityCommands(...)` |
| 地图 | `main.cpp:76 imread` | `planner.cpp:325 / 358`、`controller.cpp:258/284` |
| 结果去向 | `main.cpp:198 cv::imshow` | `planner.cpp:307 publish` + `:312` 返回 Path；`controller.cpp:224` Twist |

## 4. 核对顺序建议（由内到外）

1. `yastar.cpp:802 search` —— 看代价怎么累加、启发式是什么；
2. `yastar.cpp:760 initCostMap` + `setCostField` —— 看势场怎么来；
3. `sfcSquare.cpp:355 _getBoundSquare2` —— 走廊方盒怎么长；
4. `minimumSnap.cpp:907 _solve` → `:110 iterate` —— 交替迭代怎么回事；
5. `minimumSnap.cpp:317 generateQ` / `:359 generateA` —— QP 的代价与约束；
6. `kinodynamicAstar.cpp:126 search` —— 局部搜索的状态扩展；
7. 最后回头看我们的两个调用点（`planner.cpp:160`、`controller.cpp:197`），确认参数传递。


---


## 来源 4/19：`tdt/tdt_planning_explained.md`


> 源码：`third_party/tdt-nav-kit/src/{YAstar, MinimumSnapOsqp}`
> 我们用的就是这份代码（3 个文件 0 差异、`minimumSnap` 2 行），所以下面描述**同时适用于两边**。

## 0. 一句话总览

```
栅格地图 + 动态 Mask
      │
      ▼  ① 带势场的 8 邻域 A*        （yaastar.cpp::search）
   栅格路径（每格中心点）
      │
      ▼  ② Douglas-Peucker 简化      （simplifyPath / simplifyPathHypot）
   折线（拐点）
      │
      ▼  ③ 梯形时间分配              （trapezoidalTimeAllocation）
   每段用时 T_i
      │
      ▼  ④ 方盒走廊 SFC + MinSnap 交替迭代（minimumSnap.cpp::_solve）
   平滑轨迹（位置/速度/加速度连续）
      │
      ▼  ⑤ 局部 Kinodynamic A*（4 m 窗口）→ 跟踪（我们写的）
   /cmd_vel
```

## 1. 代价场：为什么不是简单 0/1 栅格

`main.cpp` 里：

```cpp
astar.setCostField(1.f, [](float x){ return 0.5f / (0.1f + x); });
astar.initCostMap();          // sparse：走 SDF
```

`initCostMap(sparse=true)` 里：

```cpp
auto sdf = getSDF();                       // 到最近障碍的**欧氏距离**（米）
costMap(i,j) = (dist < 1e-5) ? INF : decayFunction(dist) + 1.f;
```

于是每格代价 = `1 + 0.5/(0.1+d)`：

| 离障碍 d | 代价 |
|---|---|
| 0（障碍内） | ∞ |
| 0（贴边） | 1 + 5 = **6** |
| 0.4 m | 1 + 1 = 2 |
| 2 m | 1 + 0.24 = 1.24 |
| 很远 | → 1 |

⇒ **A* 会主动走"离墙远"的路线**，而不是贴墙抄近道；`costWeight` 再整体缩放这个偏好。

## 2. A*：8 邻域 + 对角线 √2

```cpp
constexpr int neighour[8][2] = {{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
float newCost = g(x,y) + costMap(nx,ny) * costWeight * (i < 4 ? 1.f : 1.414f);
nd.estim = hypotf(nx-endx, ny-endy);       // 欧氏（格数）
```

- 优先队列比较 `cost + estim`（`getCostTotal()`）；
- 障碍格代价 ∞ ⇒ 不入队；
- 返回的是**每格一个点**的栅格路径（世界坐标）。

### 动态障碍：Mask 层

```cpp
astar.setMaskNumLayers(4);                              // 最多 4 层
astar.setMaskMapTimed(mask, "name", duration, maxCount, maskValue);  // 带生命期/次数衰减
```

每层 mask 有自己的"持续时长 + 最大使用次数"，用 `biasMask`（(dx,dy) → (duration, cost)）参与 `initCostMap`
—— 效果是：**动态障碍物周围临时抬高代价，且一段时间后自动失效**。

## 3. 路径简化：Douglas-Peucker

```cpp
path = astar.simplifyPath(path, 0.1f);   // 递归 DP，阈值 0.1 m
path = astar.simplifyPathHypot(path);    // 再用 hypot 距离过一遍
```

把"每格一个点"压成**十几个拐点**——这是 MinSnap 的控制点数量，直接决定 QP 规模。

## 4. 时间分配：梯形公式（自带转弯减速）

```cpp
float distThs = 0.5f * maxSpeed * maxSpeed / maxAcc;   // = v²/(2a)
for(每段 i){
    if(dist < distThs) T[i] = 2*sqrtf(dist / maxAcc);                          // 三角形（达不到 vmax）
    else               T[i] = (dist - distThs)/maxSpeed + 2*sqrtf(distThs/maxAcc); // 梯形
}
```

**关键点**：每段**独立**分配时间 ⇒ 短段（拐弯处）用时短但速度低，天然产生"入弯减速、出弯加速"。

## 5. 走廊：方形安全走廊（SFC）

`SfcSquare::_getBoundSquare2(x, y, maxRange)`：从路径点所在格出发，一格一格地**向外扩一个正方形**：

```cpp
for (int a = 1; a < maxRangeInt/2; a++) {
    if (freeTop)    检查上边一整行是否为自由，撞到就 freeTop=false;
    else            转而把下边继续扩（保持"接近方形"）;
    ... 左右同理
}
```

- 单侧受阻 ⇒ **只扩另一侧**，所以走廊始终是（近似）正方形盒 `[x0,y0,x1,y1]`；
- `maxRange` 限制最大边长（我们 2.5 m）；
- `shrink(bound, s)` 按比例收缩。

## 6. MinSnap：多项式 + 走廊约束的 QP

### 6.1 变量

`order=6` ⇒ 每段每轴 **6 个系数**（5 次多项式）；`maxdx=3`。

### 6.2 代价（`generateQ`）

```cpp
sub_Q(i,l) = ∫ (d^maxdx p/dt^maxdx)² dt        // 即 ∫(三阶导)² —— 最小 jerk
sub_Q(1,1) += 0.01 / T_k²                      // 额外惩罚一次项（"别乱飘"的小正则）
```

`normT=true` 时把每段按 `τ=t/T` 归一化，代价缩放 `T^(1-2·maxdx)` —— 降低矩阵病态。

### 6.3 等式约束（`generateA`）

| 约束 | 含义 |
|---|---|
| `getdx(0,1)` = `initVel` | 起点速度 = 输入（我们传 {0,0}） |
| `getfx(0)` | 起点位置 = 第一个控制点 |
| `getfx(T_i)`（每段末） | **每段末必须经过对应路径点** |
| `getdx(T_last,1)` = 0 | 终点速度 = 0 |
| `getdnx(T_i, continuity)` 与下一段 `-getdnx(0, ...)` | **C⁰..C² 连续**（位置/速度/加速度在接缝处相等） |

### 6.4 不等式约束（`generateLow/Up`）

走廊盒 ⇒ 每个控制点的 x、y 必须落在 `[low, up]`：

```
low  = { x0_0, y0_0, x0_1, y0_1, ... }
up   = { x1_0, y1_0, x1_1, y1_1, ... }
```

### 6.5 求解

```cpp
in.setBackend(MinimumSnap::Backend::OSQPCorridor);   // OSQP（二次规划）
// 备用：Backend::Close —— 闭式解（只能等式约束）
```

## 7. 交替迭代：走廊 ↔ 轨迹（这是 T-DT 最核心的工程技巧）

`_solve()` 里边做边改，最多 `maxIter=6` 轮、总时限 10 s：

```
solveOnce:  第一条走廊 = SFC(path, maxRange, shrink)
            然后 osqpExecute(times, points, corridor) 解 QP
      ↓
sample:     每段用 lineDecoder 采样 11 个点
      ↓
collision:  相邻采样点做 lineInObsticle → 记 bad 段
      ↓
update:     偶数轮 → 在 bad 段中间**插入新控制点**，并给它算一条新走廊
            奇数轮 → 把 bad 段（和下一段）的走廊**收缩一半**
      ↓
（回到 solveOnce，用新走廊/新点再解）
```

目的：**QP 只保证"控制点在走廊里"，不保证"整条曲线不撞墙"** ——
所以用采样做碰撞检查，撞了就"加点或收窄走廊"再解，直到不撞或到迭代上限。

## 8. 局部规划：Kinodynamic A*

```cpp
KinodynamicAstar astar(cols, rows, 0.05, 0, 0);
astar.config.maxNodes = 150000;
astar.config.sampleTime = 0.01;
auto result = astar.search(start, velocity, acceleration, end);
```

- 状态是 **(x, y, vx, vy)**（带速度，不是纯几何）；
- 对控制量采样 + 前向积分（`sampleTime=0.01`），在 4 m 窗口里搜一条**动力学可行**的轨迹；
- 输出 `trajectory`：每项含 `state`(x,y,vx,vy)、`acceleration`、`time`；
- 我们的控制器取它做**前馈**（用轨迹上的速度/加速度），再叠加位姿反馈，失败回退纯跟踪。

## 9. 整体设计逻辑（为什么这么组合）

| 阶段 | 解决的问题 | 为什么这样选 |
|---|---|---|
| 势场 A* | **绕开障碍、别贴墙** | 栅格快（实测 4.17 ms），势场让路径自然走中间 |
| DP 简化 | 把几百个点压到十几个 | QP 规模 ∝ 控制点数，不简化会爆 |
| 梯形时间 | 给每段一个**合理初值** | 独立分配 ⇒ 转弯自动减速；省掉"时间也是变量"的非凸优化 |
| 方盒走廊 | 给 QP 一个**凸的**安全域 | 凸 ⇒ QP 可解；方盒 ⇒ 约束简单、求解快 |
| MinSnap | **平滑**且动力学可行 | 多项式天然连续；最小 jerk ⇒ 车不顿挫 |
| 交替迭代 | 弥补"控制点在盒里 ≠ 曲线不撞墙" | 采样查碰撞 + 加点/收窄，工程上最省事 |
| Kastar | **轨迹跟踪**（局部 4 m） | 带速度状态 ⇒ 输出可直接给控制器前馈 |

## 10. 我们这条链里的实际参数

| 参数 | 值 | 说明 |
|---|---|---|
| 代价场 | `1 + 0.5/(0.1+d)` | 同上游 |
| Mask 层数 | 4 | 同上游 |
| simplify 阈值 | 0.1 m | 同上游 |
| MinSnap order / maxDx | 6 / 3 | 5 次多项式，最小 jerk |
| dt（采样） | 0.1 s | 同上游 |
| maxCorridorRange / shrink | 2.5 m / 0 | 同上游 |
| collisionCheckIter / TL | 6 / 10 s | 同上游 |
| normTime / backend | true / OSQPCorridor | 同上游 |
| maxSpeed / maxAcc | **1.5 / 2.0** | 底盘（上游 demo 是 3 / 1） |
| Kastar | 150000 节点 / dt 0.01 / 4 m 窗口 | 上游同参数，窗口是我们为实时性截的 |


---


## 来源 5/19：`tdt/tdt_full_stack.md`


## 1. 组成

| 角色 | 包 / 插件 | 说明 |
|---|---|---|
| **全局规划** | `rm_tdt_planner`（`nav2_core::GlobalPlanner`） | 二值图 + Mask → `YAstar`(势场) → `simplifyPath`/`Hypot` → `trapezoidalTimeAllocation` → `MinimumSnap`(OSQPCorridor+交替迭代) → 发 `TrajectoryPlan` + 返回 `Path` |
| **局部规划+控制** | `rm_tdt_local_controller`（`nav2_core::Controller`） | 投影到 Path → **前方 4 m 作局部目标** → `KinodynamicAstar`（状态 `[x,y,vx,vy]`）→ 按轨迹**前馈**（最近样本+2 的速度）+ 航向反馈；失败回退 pure-pursuit |
| **Mask 生产者** | `rm_tdt_planner/mask_builder` | patchwork 非地面点 → `/plan_mask`（OccupancyGrid，z∈[0.10,1.20]） |
| **骨架** | nav2 | 目标 / 行为树 / TF / 生命周期；**不用它的 costmap 做规划** |

## 2. 一键跑

```bash
# 终端 A：Mask 生产者
ros2 run rm_tdt_planner mask_builder --ros-args -p width:=560 -p height:=313 \
  -p origin_x:=-19.9 -p origin_y:=-8.89

# 终端 B：整套 T-DT
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=classic world:=RMUC2027 map_name:=RMUC2027_v2 \
  mode:=nav map_yaml:=$PWD/src/bringup/rm_nav_bringup/map/RMUC2027_v2.yaml localization:=beluga lio:=small_point_lio \
  nav:=rm_tdt_local planner:=tdt ground:=patchwork robot:=chassis spin_speed:=0.0 gui:=True

# 终端 C：发目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -6.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## 3. 参数（T-DT demo 值 + 底盘物理量）

| 参数 | 值 | 来源 |
|---|---|---|
| `max_corridor_range` / `corridor_shrink` | 2.5 / 0.0 | T-DT demo |
| `min_snap_order` / `max_dx` / `iter_time_limit` | 6 / 3 / 10 | T-DT demo |
| `collision_check_iter` | 6 | T-DT demo |
| `norm_time` | false | demo A 原始配置 |
| `max_speed` / `max_acc`（规划） | 1.5 / 2.0 | **底盘**（demo 是 8/2 飞行器） |
| `lookahead` | 4.0 | T-DT 文档建议 |
| `max_nodes` / `sample_time`（Kastar） | 120000 / 0.01 | demo 全局用的预算 |
| `min_local_goal` | 1.0 | 太近不做 Kastar（必失败） |

## 4. 实测（Classic，headless）

| 场景 | 目标 | 结果 | Kastar 统计 |
|---|---|---|---|
| 近距 | (2.0, 0.5) | **SUCCEEDED** | ok=562 fail=744 |
| 远距 | (−6.0, 1.0) | 100 s 内未完成（48 s 仿真） | ok=1506 fail=822 |

**Kastar 失败率 ≈35%**，全部由 pure-pursuit 回退兜住；
失败集中在"局部目标贴墙/贴近终点"这两类（`min_local_goal=1.0` 已过滤一部分）。

## 5. 已知问题 / 下一步

| # | 问题 | 方向 |
|---|---|---|
| 1 | Kastar 失败率 35% | 局部目标再往前取（或用路径切线偏移），并给 Kastar 一个"目标点放宽半径" |
| 2 | 远距目标未在窗口内到点 | 查是否卡在窄处（Kastar 失败→pure-pursuit 速度低） |
| 3 | `MinimumSnap` 的 `success=0` | 已确认是 T-DT 自身行为（他们 demo 也会 45% 失败），我们用其 path |
| 4 | 走廊 box 数量少（4） | 走廊控制点应按弧长加密 |

## 6. 手动给目标点（自己跑）

### 6.1 三种方式

| 方式 | 命令 |
|---|---|
| **RViz** | 工具栏 **2D Goal Pose**，在地图上点一下并拖出朝向（goal 在 `map` 系） |
| **命令行** | `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"` |
| **脚本（推荐）** | `tools/scripts/nav/tdt_goal.py 2.0 0.5 --send` |

脚本会先检查目标点是否在地图内、是否落在障碍格、离障多远，再发目标并打印结果。

### 6.2 场地开阔点（实测 clearance）

```
( 1.0, 2.5) 3.20 m      ( 2.0, 0.5) 3.13 m      ( 2.0, 2.5) 3.02 m
( 2.0, 1.5) 3.02 m      ( 2.0,-0.5) 2.95 m      (-13.0,-4.5) 2.90 m
( 1.0, 3.5) 2.90 m      (-12.0,-5.5) 2.90 m
```

**不建议**：`(-10.0, 0.0)`（clearance 0.2 m，像门口）、`(5.0,-5.0)`（0.3 m）、`(-5.0,5.0)`（0.32 m）——
这些点要么贴墙要么在门洞，规划会贴边、Kastar 也更容易失败。

### 6.3 跑的时候看什么

| 话题 / 日志 | 含义 |
|---|---|
| `[tdt] plan: N 点, 走廊 M, 时长 T, snap=0/1` | 全局规划每次的输出 |
| `[tdt-local] kastar ok: … 局部目标 D m, 前馈 v=…` | 局部 Kastar 成功（含目标距离/前馈速度） |
| `[tdt-local] 统计: kastar_ok=… fail=… fallback=…` | 成功率统计（每 50 次一条） |
| `/trajectory_plan` | 全局 plan（轨迹 + 走廊） |
| `/plan_mask` | 动态障碍 Mask |
| `/cmd_vel` | 最终速度指令 |

### 6.4 停/重发目标

```bash
ros2 action send_goal ...        # 新目标会自动抢占旧目标
ros2 service call /navigate_to_pose/_action/cancel_goal ...   # 或直接 Ctrl-C 目标命令
```


---


## 来源 6/19：`tdt/tdt_diff_inventory.md`


## A. 编译/适配补丁（改的是他们源码，但只动"能不能编"）

| # | 文件 | 改动 | 为什么 |
|---|---|---|---|
| 1 | `src/tdt/prelude.hpp`（新增） | 强制 include `<iostream>/<algorithm>/<cmath>` | `yastar.cpp` 用了 `std::cout`/`std::any_of` 但没 include |
| 2 | `minimumSnap.hpp/cpp` | `SolveOutput.times` + 2 行赋值 | 上游不暴露最终分段时长，时间戳只能猜；暴露后**精确** |
| 3 | `minimumSnap` 调用 | `setMap(map, res, **-origin_x, -origin_y**)` | SfcSquare 内部用 `(p+origin)/res`；我们地图原点非 0（他们 demo 是 0） |

## B. 算法参数（有意的物理量替换）

| # | 参数 | T-DT | 我们 | 理由 |
|---|---|---|---|---|
| 4 | `max_speed` / `max_acc`（规划） | 3 / 1（飞行器） | **1.5 / 2.0** | 底盘实际能力 |
| 5 | 局部 `maxNodes` / `sampleTime` | 150000 / 0.01 | 同 | 一致 |
| 6 | `min_local_goal` | 无 | **1.0 m** | 目标太近 Kastar 必失败（我们加的） |
| 7 | 简化后 <3 点 | 不处理（demo 都是长路径） | **跳过简化** | 否则 MinSnap 需要 ≥3 点会直接不跑 |

## C. 数据源（他们私有，我们等价实现）

| # | 项 | T-DT | 我们 |
|---|---|---|---|
| 8 | 静态栅格 | 私有：先验图投影 | **`/map`（map_server 的 PGM）**，语义等价 |
| 9 | 动态障碍 | 私有生产者 + `pushMask` | **patchwork → `/plan_mask`**（TF 到 map） |
| 10 | v0（速度反馈） | 私有状态估计 | **位姿差分**（本栈 twist 恒 0） |
| 11 | 执行层（轨迹→cmd_vel） | 私有 | 取样本+2 → 车体系 → 航向反馈 + pure-pursuit 回退 |

## D. 工程结构

| # | 项 | T-DT | 我们 |
|---|---|---|---|
| 12 | 进程结构 | 单节点、内存对象 | nav2 三件套 + ROS 话题（`/trajectory_plan`、`Path`） |
| 13 | 坐标系 | 单一 | **全 map**（我们已把 odom_level 从 TDT 链路移除；其它模式仍用 odom_level） |
| 14 | 局部栅格范围 | 未知 | 先验图 **16 m 窗口** + Mask 叠加 |
| 15 | 局部目标贴墙处理 | 未知 | 沿路径**回退到可通行格**（最多 20 点） |

## E. 完全一致的部分（未改）

- `YAstar`：`setCostField(0.5/(0.1+d))`、`initCostMap`、`setMaskNumLayers(4)`、`search`
- `simplifyPath(0.1)` + `simplifyPathHypot`
- `trapezoidalTimeAllocation(vmax, amax)`
- `MinimumSnap`：order 6 / maxDx 3 / dt 0.1 / TL 10 / `collisionCheckIter` 6 /
  `maxCorridorRange` 2.5 / `corridorShrink` 0 / `normTime` true / `OSQPCorridor` / `initVel{0,0}`
- 走廊取 `SolveOutput.corridor`（MinSnap 内部）
- 局部 `KinodynamicAstar`：同一份源码、同参数（`maxNodes` 1.5e5、`sampleTime` 0.01）

## F. 可回退项（如果要求"逐字节一致"）

| 项 | 回退方式 | 代价 |
|---|---|---|
| 6 `min_local_goal` | 置 0 | 终点附近 Kastar 频繁失败 |
| 7 跳过简化 | 删该分支 | 短路径 MinSnap 不跑（= 他们的行为） |
| 14 16 m 窗口 | 用全图 | 搜索变慢，失败率可能略降 |
| 15 目标回退 | 删 | 目标落障碍时 Kastar 直接失败 |


---


## 来源 7/19：`tdt/tdt_integration_plan.md`


## 1. 两条链路并排看

### T-DT 的完整链路（他们实际跑的）

```
点云+IMU → ZLIO（前端，局部地图）                     [开源 ✅]
        → 后端因子图（关键帧 + 先验配准 + 回环 + UWB）  [开源 ✅，GPL]
        → 位姿 + 协方差
              │
2D 栅格 ← 先验图投影 + 实时障碍 Mask                  [私有 ❌]
              │
        YAstar.search()  → 路径                       [开源 ✅]
        SfcSquare        → 走廊 box                   [开源 ✅]
        KinodynamicAstar → 轨迹(t,x,y,vx,vy,ax,ay)    [开源 ✅]
        MinimumSnap      → 平滑轨迹（可选）            [开源 ✅]
              │
        控制器（轨迹跟随 / 前馈+反馈）                  [私有 ❌]
              │
        /cmd_vel → 底盘
```

### 我们的链路（目标形态，nav2 为骨架）

```
点云+IMU → small_point_lio（前端）                     [我们 ✅]
        → 占位：LIO-SAM（回环后端）/ GICP+SC（定位）    [我们 ✅]
              │
        nav2 costmap（STVL + patchwork + 膨胀）        [nav2 ✅]
              │  ┌─ 给规划器：obstacle-only + 5~10cm 小膨胀（0/1）
              │  └─ 给控制器：带膨胀的 soft costmap（兜底）
              ↓
┌──────────────────────────────────────────────────┐
│ rm_tdt_planner（nav2_core::Planner 插件）          │  ← 要写
│   YAstar → Path          （return 给 nav2）        │
│   SfcSquare → corridor   （pub /plan_corridor）    │
│   KinodynamicAstar → traj（pub /plan_timed）       │
└──────────────────────────────────────────────────┘
              │ Path                        │ traj + corridor
        nav2（BT / goal checker / 行为树）    ↓
                                        rm_mpc（速度空间 MPC）  [我们 ✅]
                                              │
                                        /cmd_vel → 底盘（MPPI 可切回）
```

## 2. "T-DT 能不能全部用上"——逐项

| T-DT 组件 | 能不能用 | 怎么用 | 备注 |
|---|---|---|---|
| **YAstar** | ✅ 用满 | planner 插件里调 | MIT |
| **SfcSquare** | ✅ 用满 | 插件里生成走廊，pub 给 MPC | MIT |
| **KinodynamicAstar** | ✅ 用满 | 插件里生成带时间轨迹，pub 给 MPC | MIT |
| **MinimumSnap** | ⚠️ 可选 | 当"带走廊的平滑器"；或不用（nav2 有 `ConstrainedSmoother`） | 输出只有位置、62 ms/次 |
| **Mask**（临时障碍） | ⚠️ 可选 | 把对手车/临时禁行喂成 Mask 层 | 需先有动态障碍源 |
| **z-localization** | ❌ 不用 | GPL-2.0，且我们已有 LIO-SAM/GICP/SC | 只作只读参考 |
| **他们的控制器** | ❌ 没有开源 | 我们用自己的 MPC 补 | —— |

⇒ **算法核 4 件里用满 3 件（A\* / SFC / Kinodynamic），MinimumSnap 可选**。

## 3. 改动清单（一个包 + 两条话题 + 控制器订阅）

### 3.1 新建 `src/nav/rm_tdt_planner/`（nav2 planner 插件）

```
package.xml / CMakeLists.txt        nav2_core + Eigen + C++20；依赖 src/nav/rm_mpc_controller/src/tdt
src/planner.cpp
  configure(): 读参数（use_inflation、min_dilation_m、cost_weight、车速/加速度上限）
  createPlan(start, goal):
      1) costmap → 栅格
         · use_inflation=false：lethal(254) 为障碍 + 可选 1~2 格膨胀（默认 0.10 m）
         · use_inflation=true ：直接用 nav2 的膨胀代价（做对照）
      2) YAstar.setMap / setCostWeight / search()
      3) 简化路径 → 转 nav_msgs::Path（return）
      4) （第 2 步加）SfcSquare.getCorridor → pub /plan_corridor
      5) （第 2 步加）KinodynamicAstar.search → pub /plan_timed
```

**launch 增加**：`planner:=ya_star`（默认仍 `smac2d`）。

### 3.2 话题约定（不需要新 msg 包）

| 话题 | 类型 | 内容 |
|---|---|---|
| `/plan_timed` | `nav_msgs/Path` | 每个 pose 的 `header.stamp` = 该点的时间；`twist` 用不到 |
| `/plan_corridor` | `std_msgs/Float64MultiArray` | 每 4 个数一个 box：`xmin,ymin,xmax,ymax` |

### 3.3 `rm_mpc_controller` 改动

```
订阅 /plan_timed：用来按时间插值取参考（替代"弧长 + v_ref"）
订阅 /plan_corridor：用来当走廊（替代固定 ±half_w 方框）
无消息 ⇒ 回退到现状（保证 planner:=smac2d 时行为不变）
```

## 4. 三步交付与验收

| 步 | 交付 | 验收 |
|---|---|---|
| **1** | planner 插件（仅 A\* → Path），两种栅格可切 | 同图同起终点：路径长度、最小离障、能否穿窄道（gap ≈ 0.5 m） |
| **2** | 加 SFC + Kinodynamic，pub 两条话题 | 走廊含路径、轨迹时间单调、可视化正确 |
| **3** | MPC 订阅 + 回退 | 窄通道/高速 A/B：到达时间、速度、走廊违背、指令平滑度 |

## 5. 风险与对策

| 风险 | 对策 |
|---|---|
| 膨胀把窄通道封死（实测 0.2/0.3 m ⇒ 连通区缩 3 倍） | 规划器栅格用 lethal + 5~10 cm；膨胀只给控制器 |
| C++20（`std::span`/`<ranges>`）在 Humble 编译 | 只对 tdt 源文件设 `CXX_STANDARD 20`（已在本机验证可行） |
| 上游缺 `<iostream>` | 编译参数 `-include iostream`（**不改上游文件**） |
| Kinodynamic 在最窄缝可能失败（maxNodes） | 失败即回退到 A\* 路径（无时间轨迹，MPC 用弧长参考） |
| 与 SMAC2D 的 A/B 不公平 | 同一栅格、同一起终点、同一控制器，各跑 3 次取中位 |

## 6. 膨胀/clearance：**完全按 T-DT 的机制**（不做自定义）

### 6.1 他们的实现（源码）

| 函数 | 实际行为 |
|---|---|
| `SfcSquare::setSaveDistance(distance, par)` | `distanceInt = ceil(distance/mapping)` 后调 `_dilate(map, distanceInt)`：**方形（切比雪夫）膨胀**，逐格看 `distance` 邻域内有没有 0；**并把地图四边各填 `distanceInt` 行/列为障碍** |
| `SfcSquare::_getBoundSquare2(x,y,maxRange)` | 从该点向外**按方形扩张**，直到某一边碰到障碍（四边都要 free）；**不检查地图边界**（注释写"边界会在地图膨胀的时候处理"） |
| `SfcSquare::shrink(bound, shrink)` | 四条边各向内收 `shrink`，并 `clamp` 到**半宽**（箱子比 `2*shrink` 小 ⇒ 退化成点） |
| `SfcSquare::getCorridor(pts, maxRange, shrink)` | 对每个路径点 `_getBoundSquare2` + `shrink`；**首尾点直接给退化 box** |
| `YAstar::setCostWeight` + `getSDF` | 障碍距离势场（只加代价，不减少可通行区域） |

**他们的推荐**（`setSaveDistance` 的注释原文）：
> "更加建议不进行膨胀，而是对处理结果进行后处理。"

⇒ 也就是说：**地图不做机器人半径膨胀**，安全裕量靠 `getCorridor(..., shrink)` 的**后处理收缩**。

### 6.2 我们的实现 = 同一套函数、同一参数语义（不含任何自定义膨胀）

```
① 建图：lethal(254) → 0，其余 → 255（SfcSquare 约定：0=障碍、255=自由）
② 同一张 map 实例同时交给 YAstar::setMap 与 SfcSquare::setMap
③ SfcSquare::setSaveDistance(d_edge)     ← 用**他们的**方形膨胀（只为地图边界安全）
      d_edge = 0.05 m（1 格）；**不是** 0.25 m
④ YAstar::setCostWeight(w)               ← 用**他们的**势场做离障偏好
⑤ SfcSquare::getCorridor(path, maxRange, shrink = 0.25 m)  ← 用**他们的**后处理收缩，
      shrink = 机器人半宽 0.228 + 余量 0.022
⑥ 走廊里出现退化 box（宽或高 < 0.10 m）⇒ 该段不下约束（已有逻辑），不另造规则
```

**参数只有两个**，都落在他们的 API 里：

| 参数 | 取值 | 含义 |
|---|---|---|
| `d_edge`（`setSaveDistance`） | **0.05 m**（1 格） | 只为地图边界安全；**不是**机器人膨胀 |
| `shrink`（`getCorridor`） | **0.25 m** | 机器人半宽 + 2.2 cm 余量（后处理收缩） |

> 我先前提的"lethal + 5~10 cm 自定义方形膨胀"**作废**：那不是 T-DT 的机制。
> T-DT 的机制是 **"地图不膨胀（或仅 1 格用于边界）+ 走廊后处理 shrink"**。

### 6.3 物理后果（必须知道）

- `shrink = 0.25` ⇒ **只有宽度 ≥ 0.5 m 的通道**才能得到非退化走廊；
- 更窄的缝：A* 仍然能规划（它只看 0/1 + 势场），但 `getCorridor` 给出退化 box
  ⇒ 该段 MPC 无走廊约束（只跟参考路径）⇒ **没有硬保证**（这一点和 T-DT 完全一致）；
- 要更保守 ⇒ 增大 `shrink`；要更贴边 ⇒ 减小 `shrink`。**只有这一个旋钮**。


---


## 来源 8/19：`tdt/tdt_integration_report.md`


## 1. 已完成的东西（代码）

| 包 | 内容 | 状态 |
|---|---|---|
| `rm_plan_msgs` | `TrajectoryPlan`（t/x/y/vx/vy 并行数组 + corridor 每 4 个数一个 box + 元信息） | ✅ 编译 |
| `rm_tdt_planner` | nav2 `GlobalPlanner` 插件 `rm_tdt_planner/TdtPlanner`：二值图+Mask → `YAstar`(势场) → `simplifyPath`/`simplifyPathHypot` → `trapezoidalTimeAllocation` → `MinimumSnap`(OSQPCorridor+交替迭代) → 发 `TrajectoryPlan` + 返回 `Path` | ✅ 编译、加载 |
| `rm_tdt_planner/mask_builder` | patchwork 非地面点 → `/plan_mask`（OccupancyGrid，z 带 [0.10,1.20]） | ✅ |
| `rm_mpc_controller` | 订阅 `/trajectory_plan`；有时间轨迹则按时间插值取参考 + 用 plan 的走廊；无 plan 回退弧长参考 | ✅ |
| 参数/launch | `nav2_params_sim_planner_tdt.yaml`（T-DT demo 参数）、`planner:=tdt` 槽位 | ✅ |
| 依赖 | OSQP（vendor deb 解压）+ OsqpEigen（源码编译）放在 `tools/mpc_proto/.deps/` | ✅ 不需要 sudo |

## 2. Classic 实测（headless）

```
planner_server: [tdt] configured: range=2.50 shrink=0.00 order=6 maxDx=3 TL=10.0 iter=6 v=1.50 a=2.00 norm=1
[tdt] plan: 41 点, 走廊 5, 时长 7.44 s, A* 122→简化 5, snap=1
navigate_to_pose: SUCCEEDED
MPC: 消费 TrajectoryPlan 19~24 次/次运行
```

## 3. 遇到的问题（如实记录）

### 3.1 编译/链接

| 问题 | 现象 | 解决 |
|---|---|---|
| `yastar.cpp` 缺 `<iostream>` | `std::cout` 未声明 | **不改上游**，加 `src/tdt/prelude.hpp` 并用 `COMPILE_FLAGS "-include ..."` |
| `-include` 多参数被当输入文件 | `c++: error: algorithm: linker input file not found` | 同上，改成单字符串 COMPILE_FLAGS |
| OsqpEigen 找不到 `osqp.h` | `Compat.hpp:12 fatal error` | include 加 `${OSQP_ROOT}/include/osqp` |
| **自定义消息 undefined symbol** | `controller_server` 直接 `symbol lookup error: get_message_type_support_handle<rm_plan_msgs::msg::TrajectoryPlan>` | 必须**显式链接** `rm_plan_msgs::rm_plan_msgs__rosidl_typesupport_cpp`（`${rm_plan_msgs_TARGETS}` 不含它） |
| C++20 | 上游用 `std::span`/`<ranges>` | 该包 `CMAKE_CXX_STANDARD 20` |

### 3.2 运行时

| 问题 | 现象 | 处理 |
|---|---|---|
| **MinSnap 在极短路径上失败** | 一次运行里 **25 次** `MinimumSnap 失败` | 简化后 <3 点直接跳过 MinSnap（已加 guard） |
| **走廊退化** | `getCorridor` 的**首尾点固定返回退化 box**；简化到 2 点时只剩 2 个空 box | 走廊改用**原始路径**（当简化后 <3 点） |
| `plan.header.stamp` 用错 | 一开始用 `start.header`（请求时刻），MPC 算 age 会错 | 改成**发布时刻** `node_->now()` |
| `/trajectory_plan --field traj_source` 读不到 | 可能是 echo 参数/时序问题 | 未阻塞（MPC 已成功消费 19~24 次） |
| **速度偏慢** | 22 s 只走 3.8 m（0.17 m/s），plan 说 7.4 s 走完 | 见 §4 |

### 3.3 语义/几何（读码后确认，已落实）

- `YAstar` 与 `SfcSquare` **地图值语义相反**（0=free vs 0=障碍）；
- **原点符号相反**（`(p−o)` vs `(p+o)`）；
- `getCorridor(maxRange)` **不能给大数**（`int` 溢出会让所有 box 退化成点），用 2.5 m；
- 地图边界必须靠 `setSaveDistance` 兜住（`_getBoundSquare2` 不查边界）；
- 只取 costmap 的 **LETHAL(254)** 建二值图，**不做机器人半径膨胀**。

## 4. 还没解决的 / 下一步

| # | 问题 | 建议 |
|---|---|---|
| 1 | **MPC 明显慢于 plan**（plan 7.4 s/路段，实际 0.17 m/s） | 查 `solveVelocity` 的参考速度是否被 `w_du`/`vmax` 限住；对比 plan 的 `vx/vy` 与 `/cmd_vel` |
| 2 | 走廊 box 数量少（5 个） | 走廊取点应**加密**（按弧长每 ~0.5 m 取一个控制点），而不是只在简化后的点上取 |
| 3 | Mask 与 costmap 的原点必须一致 | 当前 mask_builder 用参数写死 (-19.9,-8.89)；应从 map_server/参数自动取 |
| 4 | 时间分配用弧长比例 | 目前 `t_i = total_t · s_i/s_total`；更准的是让 MinSnap 输出每点时间（它不暴露），或按段时长插值 |
| 5 | 未做 A/B | `planner:=smac2d + rm_mpc` vs `planner:=tdt + rm_mpc` 的受控对比（同图同起终点） |

## 5. 一键复现

```bash
# 1) 依赖（一次）
tools/mpc_proto/build.sh
# 2) 编译
colcon build --symlink-install --packages-select rm_plan_msgs rm_tdt_planner rm_mpc_controller rm_navigation
# 3) 跑（另开终端起 mask_builder）
ros2 run rm_tdt_planner mask_builder --ros-args -p width:=560 -p height:=313 -p origin_x:=-19.9 -p origin_y:=-8.89
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=classic world:=RMUC2027 map_name:=RMUC2027_v2 \
  mode:=nav map_yaml:=$PWD/src/bringup/rm_nav_bringup/map/RMUC2027_v2.yaml localization:=beluga lio:=small_point_lio \
  nav:=rm_mpc planner:=tdt ground:=patchwork robot:=chassis spin_speed:=0.0 gui:=True
```

## 6. 追查"MPC 比 plan 慢/不稳"的过程（2026-11-06 续）

### 6.1 用 `/mpc_debug` 抓到的现象（加 plan 消费之后）

```
v_prev  ≈ -1.9 / -1.1     ← 上一周期指令（自积分）已饱和
v0      ≈ -2.0 / -1.2     ← QP 输出顶到 vmax
cmd_vx  ≈ -1.5, cmd_vy ≈ +1.78, cmd_wz ≈ 1.8   ← 车在打转
ref_vx  ≈ -0.75            ← 参考只要求 -0.75 m/s
```

⇒ **QP 输出方向与参考相反、且饱和到 vmax**，不是"跟得慢"，是**根本没在跟**。

### 6.2 找到的两个结构性原因

| # | 原因 | 证据 | 处理 |
|---|---|---|---|
| 1 | **没有真实速度反馈** | 本栈 `/odom` 的 twist 恒 0；`v_prev` 来自**自己上一拍的指令** ⇒ 一旦饱和就自我维持 | 改用**位姿差分**测速（`Δp/Δt`，一阶低通） |
| 2 | **参考时间基用"绝对时间"** | `t_now = now − plan.stamp`；机器人一落后，参考点就跑远（plan 8~11 s，车只走 0.2 m/s） ⇒ 位置误差越来越大 ⇒ QP 饱和 | 改成**按机器人投影**取参考时间（空间跟踪）：先找 plan 上最近点，用它的 `t` 作为 `t_now` |

### 6.3 改完后仍存在的问题

| 现象 | 说明 |
|---|---|
| **MinSnap 频繁失败**（`snap=0`，一次运行 25 次） | 短路径（简化后 6~8 点）时 QP 失败/迭代耗尽；目前回退到 A\*+梯形时间（`traj_source=3`） |
| **A\* 后期"没找到路径"** | 车在前面的振荡中开进了障碍/贴墙 ⇒ start 落在 lethal 格上，A* 直接失败 |
| cmd_vel 归零 | A* 无路径 ⇒ planner 返回空 Path ⇒ 控制器无参考 |
| TF 告警 | `local_costmap` 报 `odom_level` 与 `base_link_fake` 不在同一棵树（与本次改动无关，但会干扰局部代价图） |

### 6.4 结论（当前状态）

- **链路结构已完成并验证**（planner 插件加载、plan 发布、MPC 消费、一次 goal SUCCEEDED）；
- **闭环不稳定**：核心是"参考怎么给"和"速度反馈从哪来"两个问题；
- 两个结构性修复已落地（位姿差分测速、空间时间基），但**还需要一轮专门的闭环调参**（yaw 并入 MPC、MinSnap 失败原因、QP 饱和保护）。

### 6.5 建议的下一步（按优先级）

1. **yaw 并入 MPC**（状态加 θ），不再用独立的 `k_theta * e_theta` —— 现在车"打转"很可能来自这一层；
2. **QP 加饱和保护**：`|v0|` 超过参考速度 1.5 倍时重解（降权位置项）；
3. **查 MinSnap 失败原因**：打印 `SolveOutput.iter/time/success` 与 `SolveInput` 关键字段；
4. 先在小场景（短距离、无障碍）把跟踪调稳，再上整场。

## 7. MinSnap 为什么一直失败（深挖结果）

### 7.1 现象

| 场景 | 结果 |
|---|---|
| 我们的场地图 + A* 路径 | `OSQP未收敛，方向=1 状态=-2`（shrink=0）／ iter=1 后第 2 轮失败（shrink>0） |
| **他们的地图 + 他们的起终点 + 他们的参数** | **同样失败**（norm=1: iter=6 仍有碰撞；norm=0: iter=4 后 OSQP infeasible） |
| 空白地图 + 直线路径 | 需要给地图加障碍边框，否则 `_getBoundSquare2` **越界访问**（assert） |

⇒ **不是我们的地图/路径特殊，也不是调用顺序写错**；在**他们自己的地图和参数**上也复现失败。

### 7.2 已确认的几个坑（都已修/确认）

| # | 事实 | 影响 |
|---|---|---|
| 1 | `SfcSquare`/`_getBoundSquare2` **不检查地图边界** | 地图必须有障碍边框，否则越界/assert。**我们的 costmap 图没有自然边框** ⇒ 需要 pad 2 格 |
| 2 | `MinimumSnap::setMap` 把 origin 直接转给 SfcSquare（`(p+origin)/res`） | 必须传 `-origin_x, -origin_y`（我们之前传错 ⇒ 索引全错） |
| 3 | `validTimes` 要求 `times.size()+1 == points.size()` | `trapezoidalTimeAllocation` 正好返回 `size-1` ⇒ 可以直接用 |
| 4 | `OSQP未收敛，方向=… 状态=-2` | 我们用的 `ros-humble-osqp-vendor`（0.6.x）与上游 `3rd/osqp` 子模块**可能不是同一版本/同一默认参数** |
| 5 | `collisionCheckIter` 控制的是**交替迭代次数**，不是采样密度 | 迭代第 2 轮 `solve()` 返回空 ⇒ 直接失败 |

### 7.3 结论

> **MinSnap 在我们的构建里"跑不起来"，与 nav2/我们的链路无关** ——
> 在**他们的地图、他们的参数**上也失败 ⇒ 问题在 **OSQP 依赖（版本/参数）或他们仓库的 `3rd/osqp` 子模块版本**。

### 7.4 下一步（要真正"完全按 T-DT"）

1. **用他们的 `3rd/osqp` 子模块版本**（`scripts/setup.sh` 会拉）替换 `ros-humble-osqp-vendor`，再试；
2. 打开 `osqpExecute` 里的 `OSQPSettings`（`eps_abs/eps_rel/max_iter/polish`），对比 vendor 版默认值；
3. 若版本对齐后仍失败，说明 `main.cpp` 的 demo 依赖**未提交的 OSQP 补丁/参数**，需要向他们 issue 询问；
4. 在此之前，规划器用 **A\* + 梯形时间**（`traj_source=3`）作为可运行回退——链路其余部分（二值图/Mask/A*/走廊/plan 下发/MPC 消费）都已按 T-DT 对齐。

### 7.5 复现用的诊断程序（已入库）

| 文件 | 用途 |
|---|---|
| `test/minisnap_diag.cpp` | 场地图上枚举 origin/norm/shrink 组合 |
| `test/minisnap_pad.cpp` | 加边框 + 地图语义修正后的组合 |
| `test/minisnap_their_map.cpp` | **他们自己的地图/起终点/参数**（决定性复现） |
| `test/plan_dump.cpp`（在 rm_mpc_controller） | 真实场地跑 A*+走廊并导出可视化 |

## 8. 换成"他们的 OSQP"之后（按 setup.sh 配方）

| 步骤 | 结果 |
|---|---|
| `git clone --branch v0.6.2 osqp` + `git submodule update --init --recursive`（qdldl） | ✅ |
| `cmake -DUNITTESTS=OFF -DPYTHON=OFF -DMATLAB=OFF -DR_LANG=OFF` | ✅ 产物 `libosqp.so`（**OSQP_VERSION 0.6.2**） |
| `osqp-eigen`（master）+ `CMAKE_PREFIX_PATH=install` | ✅ `libOsqpEigen.so` |
| 用这套重编 `tdt_core`，跑**他们的地图/参数** | ❌ **结果完全相同**（norm=1: iter=6 仍碰撞；norm=0: OSQP infeasible） |

**关键结论**：他们的 `setup.sh` 拉的 osqp 与我们原来的 `ros-humble-osqp-vendor` **都是 0.6.2**，
换成他们的构建配方后失败现象**一模一样** ⇒ **不是 OSQP 版本/构建的问题**。

⇒ 剩下两种可能：
1. 他们公开的 `main.cpp` 在今天的环境下**本身就跑不出 success=1**（README 表格来自更早的提交）；
2. 依赖某个未提交的环境细节（编译选项/系统 BLAS 等）。

## 9. 为"和 T-DT 一模一样"做的对齐（已落地）

| 项 | 做法 |
|---|---|
| MinSnap 输出 | **即使 `success=false` 也使用它产出的 `path`**（他们的 demo 看的是 path） |
| 短路径 | 简化后 <3 点时**不简化**（否则 MinSnap 被直接跳过；他们的 demo 都是 20 m 长路径，不涉及） |
| OSQP 依赖 | 换成 `tools/mpc_proto/.deps/tdt/install`（他们的配方） |
| 帧 | 起终点先转到 costmap 全局帧；plan/走廊再转回请求帧（map） |
| 时间基 | 参考按"机器人投影到 plan"取（空间跟踪） |
| 速度反馈 | `/odom` twist 恒 0 ⇒ 位姿差分测速 |

**当前运行表现**（Classic）：

```
[tdt] plan: 51 点, 走廊 6, 时长 7.67 s, A* 116→简化 6, snap=0(iter=1, 0.7ms)
MPC: 消费 TrajectoryPlan：51 点, 走廊 6, age=0.32s, source=1   ← source=1 = MinSnap 输出
```

⇒ **规划器现在下发的是 MinSnap 的路径**（不再回退 A\*），与 T-DT 的"就算 MinSnap 有缺陷也用它的输出"一致。

## 10. 决定性实验：跑 **T-DT 原始 demo**（main.cpp，不经 nav2、不经我们的 MPC）

方法：`tools/tdt_demo_stub/run.sh`
- 机器上没有 OpenCV 开发包 ⇒ 用 `tools/tdt_demo_stub/opencv2` 的**极简桩**（`imread` 读 PGM、绘图为空操作）；
- OSQP 用他们 `scripts/setup.sh` 配方构建的 0.6.2（`tools/mpc_proto/.deps/tdt/install`）；
- **`main.cpp` 一个字都没改**。

### 10.1 结果（他们的 6 组固定用例）

```
test({25.2695,14.1461} → {1.47266,9.1749})  solve result: success=0 iter=6  5.64 ms  corridor=20 path=191
test({2.5323,1.88688}  → {25.5733,13.5454}) solve result: success=1 iter=4  2.10 ms  corridor=17 path=161
test({21.5323,4.88688} → {4.1733,14.1454})  solve result: success=0 iter=6  5.93 ms  corridor=24 path=231
test({4,8}             → {7,0.95})          solve result: success=1 iter=4  1.57 ms  corridor=11 path=101
test({19.2695,8.54615} → {1.47266,9.1749})  solve result: success=0 iter=6 12.78 ms  corridor=22 path=211
test({4.29,13.05}      → {23.7,0.95})       solve result: success=0 iter=6  5.84 ms  corridor=20 path=191
```

**他们的 `test()` 自己的判定是 `Check Pass accuracy: 1/1 … 6/6 rate 100%`** —— 注意：
判定依据是**路径点精度**，**不是** `success` 标志。

### 10.2 全图随机 1000 次（他们 demo 内建）

| 统计 | 值 |
|---|---|
| `success=1` | **554 次** |
| `success=0` | **452 次**（≈45%） |
| `astar search` | 平均 **4.17 ms**，最大 11.85 ms |
| **`minimum snap solve`** | 平均 **2.94 ms**，最大 **80.3 ms** |
| `set cost map` | 平均 2.26 ms，最大 5.79 ms |
| `simplify path` | 平均 14.9 µs |
| `kinodynamic astar search` | 平均 **1686 ms**（≈1.7 s，与文档"约 2 s"一致） |

### 10.3 结论

> **不是我们跑不起来，是他们的 MinSnap 在 ~45% 的情况下本来就 `success=0`；**
> 他们自己的 demo 也不把 `success` 当失败（只看路径精度）。
>
> ⇒ 我们"**用 MinSnap 的 path 而不看 success**"这一做法，**与 T-DT 原始项目的行为完全一致**。

也就是说：**算法链路可以做到和 T-DT 一模一样**（已经做到），
差异只在"他们的控制器"（未开源）和"他们自己也不保证 success=1"这一点上。


---


## 来源 9/19：`tdt/tdt_full_completion_status.md`


## 1. 已完成（代码已落地、Classic 实测）

| # | 项 | 实现 | 验证 |
|---|---|---|---|
| 1 | 全局规划 = T-DT | `rm_tdt_planner`：`/map` 先验图（map 帧）→ `YAstar`(势场,4 层 Mask) → `simplifyPath(0.1)`+`Hypot` → `trapezoidalTimeAllocation` → `MinimumSnap`(order6/maxDx3/TL10/iter6/range2.5/shrink0/normTime/**OSQPCorridor**/initVel0) | `[tdt] plan: N 点, 走廊 M, 时长 T`；走廊/时长来自 MinSnap |
| 2 | 时间精确 | 本地补丁 `SolveOutput.times`（2 行）→ 逐点时间等于 MinSnap 最终分段 | plan 时长 2.65/2.33 s |
| 3 | 走廊 | 用 `SolveOutput.corridor`（MinSnap 内部） | 15 box |
| 4 | 局部 = T-DT Kastar | `rm_tdt_local_controller`：4 m 前视 → `KinodynamicAstar`(maxNodes 1.5e5, dt 0.01) → 前馈+航向反馈 | `kastar_ok` 计数、cmd_vel 正常 |
| 5 | 局部栅格 | 先验图**窗口 16 m** + **Mask 叠加**（`/plan_mask`） | 无 TF、无 odom_level |
| 6 | 去掉 odom_level | `local_frame:=map` + 控制器用 map 帧栅格 | 无 "Invalid frame ID map"；到点 35.7 s |
| 7 | Mask 生产者 | patchwork → TF 到 map → `/plan_mask` | 已修帧错误 |

## 2. 未完成（阻塞/待办）

| # | 项 | 状态 | 阻塞原因 |
|---|---|---|---|
| A | z-localization 接入 | ⚠️ **编译成功、能启动**，但不输出 `/localization/odometry` | 前端判据：`LiDAR and IMU time difference exceeds the allowed input latency`（IMU 覆盖/时钟域不满足；实测 IMU 墙钟 ~41 Hz、最大间隔 126 ms） |
| C | 长距离到点 / 窄通道 | ⚠️ 近距目标稳定（35~41 s）；远距 (−6,1) 100 s 内未完成 | 局部 Kastar 失败率 ~50~75%，全靠 pure-pursuit 兜 |
| D | 多目标评测脚本 | ✅ 已写 `tools/scripts/nav/tdt_eval.py` | 最后一次运行受环境（DDS/启动等待）干扰，未取到汇总 |
| E | 文档 | ✅ 本文 + `docs/tdt_chain_comparison.md` | —— |

## 3. 问题记录（按严重度）

| # | 问题 | 现象 | 影响 | 方向 |
|---|---|---|---|---|
| **P1** | **Kastar 局部失败率高** | ok 143~591 / fail 448~510（**50~75%**） | 大量周期退化为 pure-pursuit，速度/平滑度受损 | 局部目标选法（切线前移、放宽半径）；`maxNodes`/`sampleTime`；栅格窗口调优 |
| **P2** | **zlio 无定位输出** | `odometry input interrupted: LiDAR and IMU time difference…` | 拿不到真实速度 v0、用不上他们的 TF | 仿真 IMU 到稳定 100 Hz；确认两路时间戳同域；必要时放开 `imu_history` |
| **P3** | 远距目标不到点 | (−6,1) 100 s 未完成 | 长任务不可用 | 依赖 P1 修复 |
| **P4** | 先验图**无膨胀** | 局部目标易贴墙 | Kastar 失败直接的诱因 | 规划器侧加 1~2 格安全裕量（T-DT 用 `shrink`）或局部栅格膨胀 1 格 |
| **P5** | MinSnap `success=0` | 频繁 `snap=0`（~45%） | 无（T-DT 自身行为，我们用其 path） | 文档已说明 |
| **P6** | nav2 obstacle 层仍用 `odom` | base yaml `global_frame: odom` | 仅 nav2 内部 | 可忽略 |
| **P7** | 评测脚本受环境干扰 | DDS 警告、启动等待 | 拿数不稳 | 脚本加 `--wait` 与日志过滤 |

## 4. 下一步（按依赖）

1. **P1（Kastar 失败率）**：局部目标从"沿路径 4 m"改成"沿**切线**前移 + 要求目标点 clearance ≥ 0.3 m"；`maxNodes` 降到 50k（窗口已小）
2. **P4**：局部栅格对障碍膨胀 1 格（0.05 m），或 `shrink` 生效
3. **P2（zlio）**：仿真 IMU 提频 + 时间戳核对
4. **C**：修完 P1 后重跑远距与窄通道
5. **D**：脚本加启动等待/日志过滤，产出 20 目标报告


---


## 来源 10/19：`tdt/cod_nav_2026_deep_dive.md`


> 上游：<https://github.com/qza36/COD_NAV> 分支 **`rmul2026`**
> 读取方式：全程只读（`raw.githubusercontent.com` + GitHub REST API），**未 clone、未运行任何代码、未改动本仓库任何其它文件**；本文件是本次唯一写入。
> 分支版本：HEAD = `485f2ebfad3870925d85dfeb1c9f5fb940b50646`（`increase min plc`，2026-03-10T07:30:13Z）。
> 分支树：<https://api.github.com/repos/qza36/COD_NAV/git/trees/rmul2026?recursive=1>（35 个 blob，`truncated: false`）
> 提交史：<https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100>（返回 100 条，最早 2024-12-24）
>
> 下文所有 `路径` 的原文链接均为 `https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/<路径>`；
> 本文只写**该分支当前快照里真实存在**的内容。凡属推断都显式标注「**推测**」；读不到的东西集中在 §F。

---

## A. 2026 分支架构摘要（逐环节，含确切参数与文件路径）

### A0 数据流与 TF（原文出处 `CLAUDE.md`，<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/CLAUDE.md>）

```
/livox/lidar (PointCloud2)
  ├─> cpp_lidar_filter ──> /livox/lidar_filtered ──> STVL（local + global costmap）
  ├─> pointcloud_to_laserscan ──> /scan ──> slam_toolbox ──> /map
  └─> small_point_lio (+/livox/imu) ──> /Odometry + /cloud_registered + TF odom→base_link

Nav2: SmacPlanner2D + MPPI(Omni) + SavitzkyGolaySmoother
  └─> /cmd_vel ──> velocity_smoother ──> fake_vel_transform ──> /aft_cmd_vel ──> cod_serial_ul26
```

TF 树（`CLAUDE.md`「TF Frame Tree」节 + `nav_bringup/launch/slam.launch.py` 静态发布器）：

```
map ──(static_transform_publisher, z=0.05, 0,0,0 rpy)──> odom ──(small_point_lio 节点内硬编码)──> base_link
                                                                                        └──(fake_vel_transform, yaw=−yaw(base_link))──> base_link_fake
```

- `map→odom` 的**唯一发布者**是静态 TF：`nav_bringup/launch/slam.launch.py`（arguments `--x 0.0 --y 0.0 --z 0.05 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom`）与 `nav_bringup/launch/nav.launch.py` 中同款节点。
- Nav2 全栈 `robot_base_frame: base_link_fake`：`nav2_params.yaml:5`（bt_navigator）、`:204/:286`（两个 costmap）、`:419`（smoother_server）、`:450`（behavior_server）。

### A1 传感器 / 驱动层

| 项 | 值 | 出处 |
|---|---|---|
| LiDAR 话题 | `/livox/lidar`，**PointCloud2**（不是 CustomMsg） | `small_point_lio/config/mid360.yaml`：`lidar_type: livox_pointcloud2`、`lidar_topic: /livox/lidar` |
| IMU 话题 | `/livox/imu`（`rclcpp::SensorDataQoS()`） | 同上 `imu_topic: /livox/imu`；`small_point_lio/src/small_point_lio_node.cpp` |
| 雷达坐标系 | `livox_frame` | `mid360.yaml: lidar_frame: livox_frame` |
| Livox 驱动 | **不在仓库内**（`9856cff66` 2025-12-19 `remove livox driver` 删掉 `livox_ros_driver2/`；`9d54b5ab8` 删掉 `FAST_LIO/livox_ros_driver2/`）；CI 里用第三方 fork `https://github.com/qza36/livox_driver2_ros2`（`./build.sh humble`） | `.github/workflows/ci.yml` |
| 仿真 | **本分支没有任何仿真世界/机器人模型/plugin 启动**（`nav_bringup/urdf/*.xacro` 只是两个残存 URDF，未被任何 launch 引用；`simulation_waking_robot.xacro` 的父链接是 `chassis`，`carto.xacro` 是 `base_link`） | 树里无 `worlds/`、无 `pb_rm_simulation`、无 `gazebo` launch |
| RealSense | 相机**驱动**只在 `nav.launch.py` 启动（`realsense2_camera` 的 `rs_launch.py`，`depth_module.depth_profile: 1280x720x30`、`pointcloud.enable: true`）；`slam.launch.py` 里同一段被注释掉；`nav2_params.yaml` 里定义了 `realsense_source`（topic `/camera/camera/depth/color/points`）但 `observation_sources: livox_source #realsense_source` ⇒ **点云未接入任何代价图** | `nav_bringup/launch/nav.launch.py:120-133`、`slam.launch.py:130-141`、`nav2_params.yaml:240,263,323,346` |

### A2 点云预处理：`cpp_lidar_filter` 车体裁剪盒

源码 `cpp_lidar_filter/src/filter_node.cpp`（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/cpp_lidar_filter/src/filter_node.cpp>）：

| 项 | 值 | 出处 |
|---|---|---|
| 节点/可执行 | `lidar_filter_node`（`name=my_lidar_filter`） | 启动内联参数，见下 |
| 滤波器 | `pcl::CropBox<pcl::PCLPointCloud2>`，`setNegative(param)` | `filter_node.cpp` |
| 降采样 | **`pcl::VoxelGrid` 整段被注释掉**（第 96–107 行）⇒ `leaf_size` 是**死参数** | `filter_node.cpp` |
| 输入/输出话题 | `input_topic` 默认 `/livox/lidar`；`output_topic` 默认 `/livox/lidar_filtered` | 同上 |
| QoS | 订阅与发布都用 `rclcpp::SensorDataQoS()`（best-effort）；`632fbb4eb`「稳定在先验地图下到达增益点」把发布端从 depth 10 改成 SensorDataQoS | 同上 |
| 可视化 | `crop_box_marker`（`visualization_msgs/Marker`，CUBE，1 Hz，默认 QoS depth 10），**`marker.header.frame_id = "base_link"`** | 同上 |
| 默认参数（代码内 declare） | `min_x −0.4 / max_x 0.4 / min_y −0.3 / max_y 0.3 / min_z −0.1 / max_z 0.6 / negative true / leaf_size 0.05` | 同上 |
| **实际参数（`slam.launch.py` 内联）** | `min_x −0.2 max_x 0.2, min_y −0.2 max_y 0.4, min_z −0.1 max_z 0.2, negative True, leaf_size 0.05` | `nav_bringup/launch/slam.launch.py` |
| **实际参数（`nav.launch.py` 内联）** | `min_x −0.3 max_x 0.3, min_y −0.3 max_y 0.5, min_z −0.1 max_z 0.2, negative True, leaf_size 0.05` | `nav_bringup/launch/nav.launch.py` |
| 独立 launch | `nav_bringup/launch/lidar_filter.launch.py` 曾存在（`2fa9c091c` 2025-12-23 加入），已被 `632fbb4eb` **删除** ⇒ 该分支只有「内联在两套 launch 里的两套数值」 | `632fbb4eb` diff |

⚠ 两处坐标系事实：`CropBox` 直接作用在输入点云自身坐标（`/livox/lidar` 的 `frame_id = livox_frame`，代码不做 TF），而 marker 画在 `base_link`。我们 `livox_frame` 相对 `base_link` 有 z 偏移（`nav_bringup/urdf/carto.xacro` 的 `livox_joint` 默认 `xyz="0.0 -0.045 0.3"`）⇒ **可视化框与实际裁剪区在 z 上错开约 0.3 m**（该 xacro 未被任何 launch 使用，实际外参由外部提供）。

### A3 LIO：`small_point_lio`（自研目录，非 submodule）

- 分支内 `small_point_lio/` 是**整体 vendored 的目录**（含自己的 `.github/workflows/`、`.clang-format`、`.clang-tidy`），`d1c027958`（2025-12-28 `add small point lio and ferfect mppi`）一次性加入 40+ 文件；**该分支无 `.gitmodules`**（我抓 `.../rmul2026/.gitmodules` 得 404）。
- 上游：README 自述「Small Point-LIO is an advanced implementation of the Point-LIO algorithm, delivering a 2-3x speed improvement over the original」，作者 Yingjie Huang（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/README.md>），并链到 <https://bbs.robomaster.com/article/813022>（东莞理工学院 ACE 战队 RM26 开源贴，来自 web 检索）。第三方致谢列 Eigen / `ankerl::unordered_dense` / `small_gicp` / Open3D。**同名的公开仓库很可能是 `Yancey2023/small_point_lio`（推测**，依据：作者英文名 Yancey=Yingjie Huang，检索到 <https://relatedrepos.com/gh/Yancey2023/small_point_lio>）；README 内**没有**任何 git URL，我没有做进一步的仓库确权。
- 算法（源码证据）：ESKF + **iVox** 增量体素图（`estimator.cpp`: `ivox = std::make_shared<SmallIVox>(map_resolution, 1000000)`；`small_point_lio.cpp`: `estimator.ivox->add_point(...)`），逐点平面更新：取最近 `NUM_MATCH_POINTS = 5` 点、协方差特征分解取最小特征向量当法向（`estimator.cpp`），无 ikd-Tree、**无 scan-to-map ICP、无回环、无 PGO**。点云降采样用从 `small_gicp` 抄来的 `util/voxelgrid_sampling.cpp`（文件头注释写明 copy from koide3/small_gicp）；CMake 侧 `find_package(small_gicp)` **并不存在**，CI 里装 small_gicp 属冗余（`CMakeLists.txt` 与 `.github/workflows/ci.yml` 对照可知）。
- 构建：C++20、`-march=native -ffast-math -fno-math-errno`、OpenMP、PCH（`small_point_lio/CMakeLists.txt`）。
- 雷达适配器：`livox_pointcloud2.h` 用 `PointCloud2ConstIterator` 读 `x,y,z` 之外**还必须存在 `tag`(uint8) 与 `timestamp`(float64, 乘 1e-9 当秒)**，只保留 `(*tag & 0b00111111) == 0` 的点；`livox_custom_msg` 适配器需编译期 `-DHAVE_LIVOX_DRIVER`（`CMakeLists.txt`：`find_package(livox_ros_driver2 QUIET)`，找不到只 warning）。

`small_point_lio/config/mid360.yaml` **全部值**（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/config/mid360.yaml>）：

```yaml
lidar_topic: /livox/lidar        imu_topic: /livox/imu
lidar_type: livox_pointcloud2    lidar_frame: livox_frame    save_pcd: true
point_filter_num: 1              min_distance: 0.5           max_distance: 1000.0
space_downsample: true           space_downsample_leaf_size: 0.5
gravity: [0.0, 0.0, -9.810]      fix_gravity_direction: true
check_satu: true                 satu_acc: 3.0               satu_gyro: 35.0     acc_norm: 1.0
map_resolution: 0.5              init_map_size: 10
extrinsic_est_en: false          extrinsic_T: [-0.011, -0.02329, 0.04412]        extrinsic_R: 单位阵
laser_point_cov: 0.01            imu_meas_acc_cov: 0.01      imu_meas_omg_cov: 0.01
velocity_cov: 20.0               acceleration_cov: 500.0     omg_cov: 1000.0
ba_cov: 0.0001                   bg_cov: 0.0001
plane_threshold: 0.1             match_sqaured: 81.0
publish_odometry_without_downsample: false
```
（`config/unilidar_l2.yaml` 是另一套 Unitree L2 配置，未被使用。）

### A4 LIO 输出、话题、TF、频率

`small_point_lio/src/small_point_lio_node.cpp`（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio_node.cpp>）：

| 项 | 值 |
|---|---|
| 里程计 | `/Odometry`（`nav_msgs/Odometry`，publisher queue 1000），`header.frame_id = "odom"`、`child_frame_id = "base_link"`（**硬编码，不可参数化**） |
| TF | 同回调里 `tf_broadcaster->sendTransform`：`odom → base_link`（**由 LIO 节点自己发**，对应提交 `093d13a66`「feat:lio interface pub odom2base_link」；他们没有我们那种独立 `lio_tf_adapter`） |
| 外参处理 | 内部 `tf_buffer->lookupTransform(lidar_frame="livox_frame", "base_link", t)`；**TF 查不到就 `return`** ⇒ 该帧既不发 TF 也不发 odom，并打 `RCLCPP_ERROR` |
| 点云 | `/cloud_registered`（PointCloud2，`frame_id = "odom"`，逐点先转到 base_link 再按位姿搬到 odom；**只有订阅数 > 0 时才发布**） |
| twist | **全 0**：源码里 `odometry_msg.twist.twist.*` 五行全被注释，留有 `// TODO it is lidar_odom->lidar_frame, we need to transform it to odom->base_link` |
| 频率 | 无独立发布频率参数。`handle_once()` 在每个点云回调与每个 IMU 回调里各调一次；`publish_odometry_without_downsample: false` 时 odom 在「三队列都非空」的循环尾部发布 ⇒ **≈ IMU 频率（实车 MID-360 100 Hz）**；置 true 会每点都发一份（配置注释里明说会掉性能） |
| 地图保存 | `save_pcd: true` 时用 `util::PointcloudMapping(0.02)` 在线累积；`ros2 service call /map_save std_srvs/srv/Trigger` → 写 `ROOT_DIR + "/pcd/scan.pcd"`；`ROOT_DIR` 由 `include/param_deliver.h.in` 经 `configure_file` 注入 `@CMAKE_CURRENT_SOURCE_DIR@` ⇒ **编译期写死源码目录** |
| ROS 接口 | `rclcpp_components` 注册 `small_point_lio::SmallPointLioNode`；节点名 `small_point_lio` |
| 与 FAST-LIO / Point-LIO 的差异（我方归纳） | ①ESKF 逐点更新（Point-LIO 族）而非 FAST-LIO 的 iterated-EKF+ikd-Tree；②地图是 iVox 哈希体素（`map_resolution 0.5`）而非 ikd-Tree；③无 `filter_size_surf/map`、无 `blind`、无 `feature_extract_enable` 那套 FAST-LIO 参数；④无 IMU 与点云的 buffer 外推参数（`max_iterations`、`filter_size_*`），换成 `space_downsample_leaf_size`、`plane_threshold`、`match_sqaured`；⑤整体做了 OpenMP/PCH/`-march=native` 的工程加速 |

### A5 重定位：**没有**（2026 的核心减法）

| 事实 | 证据 |
|---|---|
| 只有静态 `map→odom`（z=0.05，rpy=0） | `slam.launch.py` / `nav.launch.py` 的 `tf2_ros static_transform_publisher` |
| 定位 launch 只启 `map_server`，**没有 AMCL** | `nav_bringup/launch/localization_launch.py`：`lifecycle_nodes = ['map_server']` |
| `nav2_params.yaml` 里**没有 `amcl:` 段**（文件第一段就是 `bt_navigator:`） | `nav2_params.yaml:1` |
| AMCL 参数文件、cartographer lua、ICP/`small_gicp_relocalization`/`lidar_localization_ros2` 全被删 | `632fbb4eb`（2026-03-04「稳定在先验地图下到达增益点」）删除 `nav_bringup/launch/bringup.launch.py`（内含 `patchworkpp`、`small_gicp_relocalization`、`lidar_localization_ros2`）与 `nav2_params_amcl.yaml`、`params/carto.config.lua`、`params/carto.localization.lua`、`map/*.pbstream` |
| slam_toolbox **不补发** `map→odom` | `mapper_params_async.yaml: transform_publish_period: 0.0` + 同行注释 `#if 0 never publishes odometry`（slh 语义：0 = 永不发布）⇒ 静态桥是唯一发布者（**意图为推测**，但发布者唯一性是事实） |
| 先验图 | `nav_bringup/map/t1.yaml`：`image t1.pgm, mode trinary, resolution 0.05, origin [-3.84, -6.05, 0], occupied 0.65, free 0.25`；t1.pgm header `P5 213 204 255` ⇒ 地图仅 **10.65 m × 10.20 m**（x∈[−3.84, 6.81]，y∈[−6.05, 4.15]） |
| `map_server` 的 `yaml_filename` 被写死为作者机器路径 | `nav2_params.yaml:368` = `/home/cod-sentry/qza_ws/cod_nav_rmul2026/src/nav2Ver/COD_NAV/nav_bringup/map/t1.yaml`（实际被 launch 的 `map` 参数覆盖为 `nav_bringup/map/t1.yaml`，`localization_launch.py` 的 `declare_map_yaml_cmd`） |

**推论（对我们最要命的一条）**：`map→odom ≡ (0,0,0.05,rpy=0)` ⇒ 机器人在 `map` 里的位姿**等于** LIO 在 `odom` 里的位姿。这条链路只有在「先验图就是在同一 odom 原点、同一朝向采集的，且每次启动都停在同一位置同一朝向」时才成立。`nav.launch.py` 并没有任何初始化位姿的东西（无 AMCL、无 initialpose 注入）。

### A6 在线建图：slam_toolbox async / lifelong

`nav_bringup/params/mapper_params_async.yaml`（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/mapper_params_async.yaml>，节点 `slam_toolbox`，可执行 `async_slam_toolbox_node`，在 `slam.launch.py` 中启动）：

| 组 | 参数 |
|---|---|
| frames/模式 | `odom_frame: odom`、`map_frame: map`、`base_frame: base_link`、`scan_topic: /scan`、`mode: lifelong`、`use_map_saver: true` |
| 求解器 | `solver_plugin: solver_plugins::CeresSolver`、`ceres_linear_solver: SPARSE_NORMAL_CHOLESKY`、`ceres_preconditioner: SCHUR_JACOBI`、`ceres_trust_strategy: LEVENBERG_MARQUARDT`、`ceres_dogleg_type: TRADITIONAL_DOGLEG`、`ceres_loss_function: None` |
| 更新/节流 | `throttle_scans: 1`（注释「信任里程计」）、`transform_publish_period: 0.0`、`map_update_interval: 2.5`、`resolution: 0.05`、`minimum_time_interval: 0.5`、`transform_timeout: 0.2`、`tf_buffer_duration: 30.0`、`stack_size_to_use: 40000000` |
| 栅格范围 | `min_laser_range: 0.01`、`max_laser_range: 5.0` |
| 关键帧 | `use_scan_matching: true`、`use_scan_barycenter: true`、`minimum_travel_distance: 1.0`、`minimum_travel_heading: 0.5`、`scan_buffer_size: 10`、`scan_buffer_maximum_scan_distance: 10.0` |
| 回环 | `do_loop_closing: true`、`loop_search_maximum_distance: 3.0`、`loop_match_minimum_chain_size: 10`、`loop_match_maximum_variance_coarse: 3.0`、`loop_match_minimum_response_coarse: 0.35`、`loop_match_minimum_response_fine: 0.45`、`loop_search_space_dimension: 8.0`、`loop_search_space_resolution: 0.05`、`loop_search_space_smear_deviation: 0.03` |
| 相关搜索 | `correlation_search_space_dimension: 0.5`、`…resolution: 0.01`、`…smear_deviation: 0.1` |
| 其他 | `link_match_minimum_response_fine: 0.1`、`link_scan_maximum_distance: 1.5`、`distance_variance_penalty: 0.5`、`angle_variance_penalty: 1.0`、`fine_search_angle_offset: 0.00349`、`coarse_search_angle_offset: 0.349`、`coarse_angle_resolution: 0.0349`、`minimum_angle_penalty: 0.9`、`minimum_distance_penalty: 0.5`、`use_response_expansion: true`、`enable_interactive_mode: true`、`debug_logging: false` |

`README.md` 明写「**目前只支持边建图边导航**」，入口 `ros2 launch nav_bringup slam.launch.py`。`nav.launch.py` 才是「先验图 + 无重定位」模式（但 `CLAUDE.md` 与 README 都没提它，只有 `632fbb4eb` 的提交信息指向它）。

### A7 3D→2D：`pointcloud_to_laserscan`（仓库内 fork）

`nav_bringup` 是 `pointcloud_to_laserscan` 的 fork（`package.xml: maintainer arlo@todo.todo`、`README.md` 是上游 ROS2 port 文档、`launch/__pycache__/*.pyc` 被提交进仓库）。

**真正被使用的参数**（不在任何 `pointcloud_to_laserscan/launch/*` 里，而是内联在 `nav_bringup` 的 launch 中）：

| 参数 | `slam.launch.py`（建图主入口） | `nav.launch.py`（先验图） |
|---|---|---|
| `cloud_in`（remap） | `/livox/lidar` | `/livox/lidar` |
| `scan`（remap） | `/scan` | `/scan` |
| `target_frame` | `base_link` | `base_link` |
| `transform_tolerance` | 0.5 | 0.5 |
| `min_height` | **0.15**（`485f2ebfa` 从 0.1 提到 0.15） | **0.10** |
| `max_height` | 1.00 | 1.00 |
| `angle_min` / `angle_max` | −3.1416 / **+3.1416（整圆！注释写 `-M_PI/2` 是错的）** | 同 |
| `angle_increment` | 0.0087（=π/360 ⇒ 723 线，注释也错写成 π/360.0 之外的说明） | 同 |
| `scan_time` | 0.3333 | 同 |
| `range_min` / `range_max` | **0.5 / 20.0** | 同 |
| `use_inf` / `inf_epsilon` | true / 1.0 | 同 |

节点源码（`pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp`）两条重要行为：
1. 输出 `scan` 用 `rclcpp::QoS(10).reliable()`；输入**只有当 `scan` 有订阅者时**才真正 `subscribe()`（`subscriptionListenerThreadLoop`，100 ms 轮询 graph），没人订阅 `/scan` 时它静默不消费点云；
2. 高度过滤是对**变换到 `target_frame` 之后**的 `z` 做（先 `tf2_->transform(*cloud_msg, *cloud, target_frame_)`，再判 `*iter_z > max_height_ || *iter_z < min_height_`）⇒「高度带是相对 `base_link` 水平面」，不是相对雷达。

仓库自带的两套 sample（**未被 `nav_bringup` 使用**，但对我们有信息量）：
- `launch/sample_pointcloud_to_laserscan_launch.py`：`cloud_in=/livox/lidar/pointcloud`、`target_frame=chassis`、`min_height 0.0 / max_height 0.65`、`angle ±1.5708`、`increment 0.0087`、`range 0.05 / 5.0`；
- `launch/sample_laserscan_to_pointcloud_launch.py`（**名字反了，实际是 pointcloud→laserscan**）：`cloud_in=/livox/idar/pointcloud`（**上游 typo `idar`**）、`target_frame=livox_frame`、`min_height −1.0 / max_height 0.1`、`range 0.45 / 10.0`。
  ⚠ 这套 `target_frame=livox_frame, −1.0~0.1, 10.0` 与**我们 bench 现有 `/scan` 配置高度重合**（我们：`livox_frame`, `min_height −1.0`, `max_height 0.1`, `range_max 10.0`）⇒ **推测**我们的 scan 参数最初就是从这份 sample 抄来的。

### A8 代价图（`nav2_params.yaml`：local 200–279 行，global 281–362 行）

| 参数 | local_costmap | global_costmap |
|---|---|---|
| `update_frequency` / `publish_frequency` | 20.0 / 20.0 | 20.0 / 20.0 |
| `global_frame` | `odom` | `map` |
| `robot_base_frame` | `base_link_fake` | `base_link_fake` |
| `rolling_window` | true | **true**（全局也是滚动窗！） |
| 尺寸 | 14 × 14 m | **50 × 50 m** |
| `resolution` | 0.05 | **0.04** |
| `robot_radius` / `footprint` | 0.2 / `[[0.15,0.15],[0.15,−0.15],[−0.15,−0.15],[−0.15,0.15]]` | 同 |
| `track_unknown_space` | （未设） | true |
| `plugins` | `["static_layer","stvl_voxel_layer","inflation_layer"]`（**局部也挂 static_layer**，type 层我们换成 `obstacle/obstacle_cloud`） | 同 |
| `inflation_layer` | `cost_scaling_factor: 5.0`、`inflation_radius: 0.75` | 同（注释「与 local_costmap 一致」） |
| `always_send_full_costmap` | True | True |
| `static_layer` | `map_subscribe_transient_local: True` | 同 + `enable: true` |

`stvl_voxel_layer`（两个 costmap 数值完全相同）：

```yaml
plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
enabled: true            voxel_decay: 0.5        decay_model: 0      # 0=linear
voxel_size: 0.05         track_unknown_space: true
observation_persistence: 0.0                    max_obstacle_height: 2.0   # layer 级
mark_threshold: 0        update_footprint_enabled: true   combination_method: 1  # 1=max
obstacle_range: 3.0      origin_z: 0.0           publish_voxel_map: true
transform_tolerance: 0.2 mapping_mode: false     map_save_duration: 60.0
observation_sources: livox_source                # realsense_source 被注释掉
livox_source:
  data_type: PointCloud2   topic: /livox/lidar_filtered   transport_type: "raw"
  marking: true            clearing: true
  obstacle_range: 8.0      raytrace_range: 9.0
  min_obstacle_height: 0.1 max_obstacle_height: 1.0
  expected_update_rate: 0.0   observation_persistence: 0.0
  inf_is_valid: false      clear_after_reading: true
  filter: "voxel"          voxel_min_points: 0
  model_type: 1            vertical_fov_angle: 2.00   vertical_fov_offset: 0.0   horizontal_fov_angle: 6.28
realsense_source:          # 定义了但未被 observation_sources 选中
  enabled: true   topic: /camera/camera/depth/color/points
  min_z: 0.2  max_z: 7.0  min_obstacle_height: 0.2  vertical_fov_angle: 0.7
  horizontal_fov_angle: 1.04  decay_acceleration: 1.  model_type: 0
```
层级陷阱：`stvl_voxel_layer.obstacle_range: 3.0`（layer 级）与 `livox_source.obstacle_range: 8.0`（source 级）**同时存在**，实际生效的是 source 级 8.0/9.0（`8d7c472ba` 2026-02-02「update:reduce min obstacle height」是最后一次动这一块）。

### A9 全局规划：SmacPlanner2D

`nav2_params.yaml:378-414`，`planner_server`: `expected_planner_frequency: 10.0`、`planner_plugins: ["GridBased"]`，`GridBased.plugin: "nav2_smac_planner/SmacPlanner2D"`：

```
tolerance: 0.5                    allow_unknown: true               downsample_costmap: false
downsampling_factor: 1            max_iterations: 1000000           max_on_approach_iterations: 1000
max_planning_time: 4.5            cost_travel_multiplier: 4.0
motion_model_for_search: "DUBIN"  angle_quantization_bins: 72       analytic_expansion_ratio: 3.5
analytic_expansion_max_length: 3.0  minimum_turning_radius: 0.05    retrospective_penalty: 0.025
reverse_penalty: 1.0  change_penalty: 0.0  non_straight_penalty: 0.0  cost_penalty: 2.0
rotation_penalty: 5.0  lookup_table_size: 20.0  cache_obstacle_heuristic: True
allow_reverse_expansion: True     smooth_path: True
smoother: {max_iterations: 10000, w_smooth: 0.4, w_data: 0.1, tolerance: 1.0e-10, do_refinement: true}
```
`motion_model_for_search / angle_quantization_bins / minimum_turning_radius / analytic_expansion_* / rotation_penalty` 都是 **Hybrid-A\* / Lattice 专用**字段，对 `SmacPlanner2D` 不产生 Dubin 搜索行为（`CLAUDE.md` 写的「SmacPlanner2D/Dubin」不准确）。真正对 2D 生效的是 `cost_travel_multiplier 4.0`（代价场里把路径推向谷底）、`tolerance 0.5`、`allow_unknown true` 与**内置 `smooth_path: True` + `w_smooth 0.4 / w_data 0.1`**。

### A10 平滑：SavitzkyGolay（**在实跑链路里是死代码**）

`smoother_server`（`nav2_params.yaml:415-429`）：`costmap_topic: global_costmap/costmap_raw`、`footprint_topic: global_costmap/published_footprint`、`robot_base_frame: base_link_fake`、`transform_tolerance: 0.1`、`smoother_plugins: ["savitzky_golay_smoother"]`，`savitzky_golay_smoother`: `plugin nav2_smoother::SavitzkyGolaySmoother`、`window_size: 7`、`poly_order: 3`、`do_refinement: True`、`refinement_num: 2`、`enforce_path_inversion: True`。

**但**：`navigation_launch.py` 的 `lifecycle_nodes` 里有 `smoother_server`，而两棵自定义 BT XML（`nav_bringup/behavior_trees/*.xml`）里**没有 `<SmoothPath>` 动作**（只有 `ComputePathToPose` → `FollowPath`）。⇒ smoother_server 被启动但从不被调用；路径平滑实际只来自 Smac 的 `smooth_path` 与 MPPI 自身的轨迹优化。（`CLAUDE.md` 的「+ SavitzkyGolaySmoother」是按其配置写的，不是按其 BT 写的。）

### A11 局部控制：MPPI（Omni）

`nav2_params.yaml:60-79`（controller_server）+ `:111-198`（FollowPath）：

```
controller_frequency: 50.0        failure_tolerance: 0.3
min_x/y/theta_velocity_threshold: 0.001
progress_checker: SimpleProgressChecker  required_movement_radius: 0.4  movement_time_allowance: 10.0
general_goal_checker: stateful True, SimpleGoalChecker, xy_goal_tolerance: 0.25, yaw_goal_tolerance: 6.28
FollowPath.plugin: "nav2_mppi_controller::MPPIController"     motion_model: "Omni"
time_steps: 60   model_dt: 0.05   batch_size: 2000   iteration_count: 1
vx_std 0.5  vy_std 0.5  wz_std 0.4      vx_max 7.5  vy_max 7.5  wz_max 2.5     vx_min −7.5 vy_min −7.5 wz_min −2.5
ax_max 5.0  ay_max 5.0  az_max 3.5      ax_min −5.0 ay_min −5.0 az_min −3.5
prune_distance: 1.7   transform_tolerance: 0.1   temperature: 0.25   gamma: 0.008
visualize: true   reset_period: 1.0   retry_attempt_limit: 2   regenerate_noises: true
TrajectoryVisualizer: {trajectory_step: 5, time_step: 3}
critics: ["ConstraintCritic","CostCritic","GoalCritic","PathFollowCritic","PathAlignCritic","ObstaclesCritic"]
  ConstraintCritic   cost_weight 4.0
  GoalCritic         cost_weight 15.0  threshold_to_consider 2.5
  ObstaclesCritic    repulsion_weight 1.5  critical_weight 20.0  consider_footprint false
                     collision_cost 10000.0  collision_margin_distance 0.1  near_goal_distance 0.5
  CostCritic         cost_weight 3.0  critical_cost 253.0  consider_footprint true
                     collision_cost 1000000.0  near_goal_distance 0.5  trajectory_point_step 2
  PathFollowCritic   cost_weight 5.0  offset_from_furthest 5  threshold_to_consider 1.5
  PathAlignCritic    cost_weight 10.0  max_path_occupancy_ratio 0.05  trajectory_point_step 4
                     threshold_to_consider 1.5  offset_from_furthest 20  use_path_orientations false
```
被注释保留的备选控制器是 `pb_omni_pid_pursuit_controller::OmniPidPursuitController`（`nav2_params.yaml:80-110`，含 `translation_kp 3.5/ki 0.45/kd 1.5`、`enable_rotation: false`、`lookahead 0.5/1.0`、曲率减速等）。

`velocity_smoother`（`:473-485`）：`smoothing_frequency: 50.0`（注释「与 controller_frequency 一致，不拖后腿」）、`scale_velocities: False`、`feedback: "OPEN_LOOP"`（⇒ `odom_topic` 实际不生效）、`max_velocity [7.5,7.5,2.5]`、`min_velocity [−7.5,−7.5,−2.5]`、`deadband_velocity [0.05,0.05,0.05]`、`velocity_timeout 1.0`、`max_accel [5.0,5.0,3.5]`、`max_decel [−5.0,−5.0,−3.5]`、`odom_topic: "odometry"`、`odom_duration: 0.1`。

### A12 速度指令与「伪底盘」：`fake_vel_transform`

`fake_vel_transform/src/fake_vel_transform.cpp`（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/fake_vel_transform/src/fake_vel_transform.cpp>）：

```cpp
declare_parameter<std::string>("robot_base_frame", "base_link");
declare_parameter<std::string>("fake_robot_base_frame", "base_link_fake");
declare_parameter<std::string>("odom_topic", "Odometry");
declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel");
declare_parameter<std::string>("output_cmd_vel_topic", "aft_cmd_vel");
declare_parameter<float>("spin_speed", 0.0);
...
// odomCallback: 用 odom 位姿 yaw 发 TF  robot_base_frame -> fake_robot_base_frame, rotation.z = -yaw(odom pose)
// cmdVelCallback:
aft_tf_vel.angular.z = spin_speed_;                       // ← 无条件覆盖
aft_tf_vel.linear.x  =  vx*cos(a) + vy*sin(a);
aft_tf_vel.linear.y  = -vx*sin(a) + vy*cos(a);
```
README 自述用途：`base_link_fake` 的 yaw 固定指向正前方，避免云台自旋时 Nav2 局部规划器被朝向变化带偏。

**你要求确认/反驳的三点，逐条给结论（依据：整分支 tarball grep）**：
1. **确认**：`aft_tf_vel.angular.z = spin_speed_` 是**无条件赋值**，`msg->angular.z` 从未被读取（`fake_vel_transform.cpp:60`）。
2. **确认**：`spin_speed` 默认 **0.0**（`fake_vel_transform.cpp:19`）。
3. **确认（反驳「2026 用非零自转」）**：**该分支没有任何文件把 `spin_speed` 设为非零**。全分支 `spin_speed` 只出现 5 次：`src/fake_vel_transform.cpp` 的 declare/get/赋值 3 次、`include/.../fake_vel_transform.hpp:35` 成员声明 1 次、`README.md:24` 文档 1 次。`slam.launch.py`/`nav.launch.py`/`fake_vel_transform/launch/fake_vel_transform_launch.py` 都只传 `use_sim_time`；`nav2_params.yaml` 里也没有 `fake_vel_transform:` 段（参数来自 launch，yaml 不覆盖）。
   ⇒ 在 2026 分支上，`spin_speed == 0` 让 `angular.z ≡ 0`，**MPPI 的 `wz`（±2.5）被整体丢弃**。这一点与 `yaw_goal_tolerance: 6.28` + Omni 平移是自洽的：他们只控位置不控朝向。我们的修法是「0 时直通」——**语义与他们不同**（我们在 `spin_speed==0` 时保留 `wz`），这一点在移植时必须明确。

### A13 任务层 / 决策层

| 组件 | 内容 | 出处 |
|---|---|---|
| BT（到点） | `RecoveryNode(number_of_retries=10)` → `PipelineSequence`：`RateController hz=3.0` → `RecoveryNode(ComputePathToPose, retries=1)`（失败→`ClearEntireCostmap global`）；`RecoveryNode(FollowPath, retries=10)` → `ReactiveFallback(FollowPath)`（失败→`ClearEntireCostmap local`）；最外层 `ReactiveFallback`：`GoalUpdated` / `RoundRobin(ClearEntireCostmap local+global)` | `nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` |
| BT（多点） | 同上，但 `RemovePassedGoals radius=0.7` → `ComputePathThroughPoses(planner_id=GridBased)`，恢复集多一个 `BackUp backup_dist=1.0 backup_speed=1.0` | `navigate_through_poses_w_replanning_and_recovery.xml` |
| 注意 | XML 里**没有 `IsStuck`**（`nav2_is_stuck_condition_bt_node` 只在 `plugin_lib_names` 里）；`ReactiveFallback name="FollowPathWithStuckDetection"` 里只有一个子节点；`nav2_through_poses` BT 的元数据块未写 `TreeNodesModel` | 同上 |
| 航点 | `nav_bringup/WPs/t1.csv` 三行：`(0.0620837,−1.11927)`、`(2.81072,−1.3887)`、`(5.64674,0.0852474)`（quaternion z/w，yaw≈0） | `nav_bringup/WPs/t1.csv` |
| 航点打点 | RViz 面板 `waypoint_editor/WaypointEditorPanel` + `InteractiveMarkers`（namespace `/interactive_marker_server`，另一个是 `/slam_toolbox`）；`waypoint_editor` 包**不在仓库内**（外部依赖），CSV 列名 `id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command` 正是它的格式 | `nav_bringup/rviz/cod_nav.rviz`、`WPs/t1.csv` |
| 航点执行 | `waypoint_follower`：`loop_rate 20`、`stop_on_failure false`、`waypoint_task_executor_plugin: wait_at_waypoint`、`wait_at_waypoint.enabled True`、`waypoint_pause_duration 200` | `nav2_params.yaml:462-472` |
| 裁判/比赛逻辑 | `nav_bringup/scripts/bt.sh`：无限循环 `ros2 topic echo /serial/receive --once` 取 `game_type`；`==3` → `NavigateToPose` 回 `(0,0,yaw=0)`；`==2` → 去 `(4.55,−2.0,z=0,w=1)`；其它值等待 | `nav_bringup/scripts/bt.sh` |
| 执行机构 | `cod_serial_ul26`（`executable cod_serial`）由 `slam.launch.py`/`nav.launch.py` 启动，**不在仓库内**；它吃 `/aft_cmd_vel` | `CLAUDE.md` 数据流 + 两个 launch |
| 其它脚本 | `scripts/nav_bringup.sh`、`scripts/sim_nav_bringup.sh` 都指向**已删除的** `nav_bringup.launch.py` / `sim_nav_bringup.launch.py`，且写死 `/home/cod-sentry/...`、`/home/arlo/...` ⇒ 死脚本 | `nav_bringup/scripts/` |

### A14 `resource/`

只有两张 PNG：`resource/cod-1.png`（43,453 B，README 顶部 logo）、`resource/cod.png`（129,659 B，未被任何文件引用）。**没有** 地图、标定文件、bag、urdf mesh。`README.md` 仅 1,105 B（标题 + 环境 + 编译 + `ros2 launch nav_bringup slam.launch.py` + 遥操 + 一句「参数模板：`nav_bringup/params/nav2_params.yaml`」）。

### A15 明确确认「不存在」的东西

对整个分支 tarball 做 grep（`--include=*.py,*.yaml,*.yml,*.xml,*.cpp,*.h,*.hpp,*.txt,*.md`，排除 `3rdparty/`）：

| 关键词 | 结果 |
|---|---|
| `amcl` | **0 命中** |
| `cartographer` | **0 命中**（连 `carto.xacro` 的**文件名**都只是残留，内容里只有 `base_link`+`livox_frame`） |
| `relocali`（relocalization） | **0 命中** |
| `patchwork` / `linefit` / `terrain_analysis` | **0 命中**（`patchwork` 只在我 grep 到的 `.idea/vcs.xml` 里有一条失效的 vcs 映射） |
| `esdf` | **0 命中** |
| `frontier` | **0 命中** |
| `small_gicp` | 4 命中，全部是**源码注释/README/CI**（`util/voxelgrid_sampling.*` 的文件头、README 第三方表、CI 安装步骤）⇒ small_gicp **不是运行期依赖**，voxelgrid 是抄进来的代码 |
| 动态障碍跟踪/预测 | **无**任何模块；唯一的「时间维」是 STVL 的 `voxel_decay: 0.5`（linear，0.5 s 后体素归零） |

⇒ 2026 分支**没有** RealSense-驱动以外的相机算法、**没有** ESDF、**没有** frontier 探索、**没有** 显式动态障碍跟踪/预测。与我们 bench 不同的是：它也**没有地面分割**（`linefit_ground_segmentation` 那一环被 `min_height 0.10/0.15` 硬高度带取代）。

---

## B. 只属于 2026 的关键设计决定（含 commit 意图；标注推测）

| # | 决定 | 时间 / commit | 能看出的意图（原文优先） | 性质 |
|---|---|---|---|---|
| B1 | **LIO 从 FAST-LIO 换成 `small_point_lio`** | `9d54b5ab8`「feat:replace fastlio2」(2025-11-18) 删除 `FAST_LIO/`；`d1c027958`「add small point lio and ferfect mppi」(2025-12-28) 真正落地 | 提交信息只写 replace；真正动机来自其 README：「**2-3x speed improvement over the original**（Point-LIO）」+ bbs 开源贴标题「比 Point-LIO 快 2-3 倍的里程计」。**推测**：为 7.5 m/s 高速 + 实车算力做的选择 | 事实 + 推测 |
| B2 | **做减法：去掉所有 submodule** | `9856cff66`「remove livox driver」(2025-12-19)、`9dfbeabdd`（删 `stvl_plugin/` 自建副本）、`632fbb4eb`（删 `bringup.launch.py`、amcl/carto 参数、`map/*.pbstream`）、`a7299b60a`（删 `FAST_LIO_ROS2/`、`loam_interface/`、`pb_omni_pid_pursuit_controller/`、`sensor_scan_generation/`、`script/`） | 分支现无 `.gitmodules`，工作区只剩 5 个包。**推测**：赛季前收敛维护面，把地面分割/pcd2pgm/重定位全部外移或砍掉 | 事实 + 推测 |
| B3 | **砍重定位 → 静态 `map→odom` + 只留 `map_server`** | `632fbb4eb`「**稳定在先验地图下到达增益点**」(2026-03-04)，同批新增 `nav.launch.py`（先验图模式）与 `WPs/t1.csv` | 提交信息直说是「在先验地图下稳定到达增益点」——即用固定先验图 + 固定起点换取稳定性，不依赖重定位。**推测**：比赛场地固定、起点固定，AMCL/ICP 调参时间不值得 | 事实 + 推测 |
| B4 | **slam_toolbox 只做 lifelong 建图，不发布 `map→odom`** | `mapper_params_async.yaml`: `mode: lifelong` + `transform_publish_period: 0.0`；`ffc4e24e3`「feat:rmul2026 slam navgation param」(2025-11-18) | **推测**：让 `map→odom` 的唯一发布者是静态桥（避免 TF 多发布者争用），代价是回环修正只改 `/map` 不改位姿 | 推测（参数是事实） |
| B5 | **STVL 取代 nav2 自带 voxel_layer，并双图（global+local）都用** | `9dfbeabdd`「feat:use stvl instead of voxel_layer」(2025-12-19)、`6e03b83e0`「chore:add stvl to global map」；`202f29a24` 曾以 git subtree 拉入 `stvl_plugin/`，随后删掉 | 现在是**外部依赖** `spatio_temporal_voxel_layer`（我们也有）。注释里明确 `voxel_decay 0.5` + `min_obstacle_height 0.1` | 事实 |
| B6 | **控制器从 `pb_omni_pid_pursuit` 换成 MPPI** | `0639ec26b`「feat:copy pb pib controller」(2025-12-23) → `a7299b60a`「**MPPI provides stable tracking and obstacle avoidance, but it oscillates around the target point**」(2026-03-10) 引入 MPPI 全套 + 新增 `CLAUDE.md` + 删 `pb_omni_pid_pursuit_controller/` | 提交信息直接给出了「MPPI 好，但目标点附近震荡」的结论；随后 `a1c33ad80`「Eliminate oscillations near the target point」→ `4651c925a`「increase speed」→ `485f2ebfa`「increase min plc」是连续三轮调参。**残留证据**：`cod_nav.rviz` 里仍留 `lookahead_point`、`curvature_points_marker_array` 两个 pb_omni 专有话题 | 事实 |
| B7 | **inflation 从「0.3/0.4」直接改成「cost_scaling_factor 5.0 / inflation_radius 0.75」** | `a7299b60a`（并在 `872adfafd` 前是 0.5/0.2） | 代码注释原文：「关键修正！原 0.3 衰减太慢，整个膨胀区代价都极高，**MPPI 没有梯度可用**。5.0 让代价快速衰减，形成清晰的『远离障碍物』梯度」；同时 `CostCritic.critical_cost: 253.0`「只有 inscribed/lethal 才触发碰撞惩罚，不再把整个膨胀区都当碰撞」 | 事实 |
| B8 | **极度激进的实车速度包线** | `4651c925a`「increase speed」 | `vx/vy ±7.5 m/s`、`ax/ay ±5.0`、`controller_frequency 50`、`vz_max 2.5`、`prune_distance 1.7`、`reset_period 1.0`（注释「延长重置周期，保持 warm-start 让速度持续攀升」）、`local_costmap 14×14`（注释「7.5 m/s × 2s ≈ 15m」） | 事实 |
| B9 | **姿态控制整体外置：`base_link_fake` + `angular.z ≡ 0` + `yaw_goal_tolerance 6.28`** | 沿用 `f768e0c23`「add fake_vel_transform」(2024-12-24) 的设计；2026 用 `spin_speed=0`（从不设非零） | README：让 Nav2 看到「yaw 固定指向正前方」的伪底盘；配合 `yaw_goal_tolerance 6.28`（等于不检查朝向）与 Omni 平移，**只控位置**。**推测**：朝向由云台/底盘自转承担，或本就不要求朝向 | 事实 + 推测 |
| B10 | **全局代价图用 50×50 滚动窗 + `resolution 0.04`，而不是「整张先验图」** | `a7299b60a` / `872adfafd` 期间的参数演化 | 先验图只有 10.65×10.2 m，而全局窗 50×50 m：`static_layer` 只覆盖场地那一小块，其余是 `track_unknown_space: true` 的未知区 + `allow_unknown: true`。**推测**：为高速预留感知余量，宁可让全局层跟着车跑 | 事实 + 推测 |
| B11 | **任务层用 bash 轮询裁判系统 + RViz 手工打点** | `632fbb4eb` 新增 `WPs/t1.csv` + rviz 的 `waypoint_editor/WaypointEditorPanel`；`bt.sh` 时间不明（不在最近 100 条提交的改动里） | 没有状态机/行为树决策层，比赛阶段切换由 `game_type` 的 if/else 决定 | 事实 |
| B12 | **RealSense 只加了参数与 RViz 帧，未接入导航** | `4de98f194`「feat: integrate realsense camera parameters and update voxel decay settings」(2025-12-23) | `observation_sources: livox_source #realsense_source` 明确注掉了。**推测**：留给视觉/自瞄组，或曾试过深度相机后放弃 | 事实 + 推测 |
| B13 | **作者自述的坑写进了 `CLAUDE.md`** | `a7299b60a` 新增 `CLAUDE.md`，`485f2ebfa` 最后修订 | 「Known Issues in Config: `bt_navigator.odom_topic` is set to `"odomety"` (typo for `"odometry"`)」；同处还承认「slam.launch.py contains **dead references to `fast_lio`** package (legacy)」 | 事实 |

---

## C. 对我们**可采纳**清单

> 每条格式：**改什么 / 抄哪个文件哪个值 / 预期收益 / 风险 · 回退**。只列 2026 分支真实存在的东西。
> 上游基线：<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/nav2_params.yaml>

### C1 STVL 的 `model_type: 1` + 3D 视锥（**最低风险、最该先抄**）
- **改什么**：我们 `stvl` 的观测源加 `model_type: 1`、`vertical_fov_angle: 2.00`、`vertical_fov_offset: 0.0`、`horizontal_fov_angle: 6.28`、`filter: "voxel"`、`voxel_min_points: 0`。
- **抄哪**：`nav2_params.yaml:240-262`（local）/`:323-345`（global）的 `livox_source` 段。
- **收益**：MID-360 是 3D 雷达，`model_type: 0`（深度相机视锥）会按 0.7 rad 垂直 FOV 裁切；改成 1 + 2.00 rad 才与 -7°~52° 的 MID-360 匹配 ⇒ 标记/清除范围正确，减少「明明看到却不标记」。
- **风险 · 回退**：`vertical_fov_angle 2.00` 比 MID-360 实际 (~1.03 rad) 大一倍，属保守取值，副作用只是多算；单参数回退。

### C2 STVL 时间衰减参数组（`voxel_decay 0.5` / `decay_model 0` / `voxel_size 0.05`）
- **改什么**：`local_obstacle` 增加取值 `stvl`；`voxel_decay: 0.5`、`decay_model: 0`(linear)、`voxel_size: 0.05`、`observation_persistence: 0.0`、`combination_method: 1`、`update_footprint_enabled: true`、`transform_tolerance: 0.2`。
- **抄哪**：`nav2_params.yaml:227-240` 与 `:310-323`。
- **收益**：动态障碍与「人走过」的残影在 0.5 s 内消失；`local_obstacle:=stvl` 让 local 也有时间维（我们现在只有 `scan/cloud/both`，无时间维）。
- **风险 · 回退**：0.5 s 太短会让**静态薄结构**（我们的 0.40 m 矮墙、桌腿）在传感器短暂遮挡时被擦掉；`8d7c472ba`「update:reduce min obstacle height」说明他们自己也在反复动这一块。先只开在 local，global 保持持久；回退 = 删该层。

### C3 观测源级 `obstacle_range 8.0 / raytrace_range 9.0`
- **改什么**：STVL `observation_sources` 的 source 级显式给 `obstacle_range: 8.0`、`raytrace_range: 9.0`（layer 级他们写 3.0，实际被 source 级覆盖）。
- **抄哪**：`nav2_params.yaml:251-253`。
- **收益**：我们 layer 级若只写 3.0，会造成「3 m 外不标记」；8/9 m 与 `/scan range_max 20` 的远场策略一致。
- **风险 · 回退**：清除过激（9 m raytrace）在玻璃/低反射面会误清；回退到 5/6 或去掉 source 级。

### C4 车体裁剪盒（`cpp_lidar_filter` 思路）
- **改什么**：新增裁剪盒节点 → `/livox/lidar_filtered`，**只喂 STVL 与 local cloud 层**，`/scan` 链不动。
- **抄哪**：`cpp_lidar_filter/src/filter_node.cpp` 的 `pcl::CropBox` + `setNegative`；QoS **必须**是 `rclcpp::SensorDataQoS()`（`632fbb4eb` 的修正）；数值参考 `nav.launch.py` 的 `x±0.3 / y −0.3~0.5 / z −0.1~0.2`，但**必须重标定**（见风险）。
- **收益**：车身/云台自遮挡从 3D 代价图里消失，减少局部代价图中心假障碍。
- **风险 · 回退**：①`CropBox` 在**输入点云坐标系（我们 `livox_frame`）**里裁，不是 `base_link`；而他们的 marker 画在 `base_link`（`filter_node.cpp` 里 `marker.header.frame_id = "base_link"`），⇒ 视觉与实际错位，照抄数值必错。②`leaf_size` 是**死参数**（VoxelGrid 整段注释掉），别指望降采样。③盒子太紧会删真障碍（y 方向不对称）。回退：停节点即恢复原链路。

### C5 `/scan` 射程：`range_max 10 → 20`、`range_min 0.05 → 0.5`
- **改什么**：我们 p2l 的 `range_max: 20.0`、`range_min: 0.5`。
- **抄哪**：`nav_bringup/launch/slam.launch.py`（p2l 节点参数块）。
- **收益**：远场墙/长走廊进 scan；`range_min 0.5` 顺带压掉近场自车回波（与他们一致）。
- **风险 · 回退**：①我们场地小（15×28 m），20 m 会把整场墙都收进来，`inf` 占比下降但单帧点数上升（`increment 0.0087` 已是 723 线）；②`range_min 0.5` 会让我们**丢掉 0.5 m 内的近距回波**（我们现在的痛点恰是近场），这一条我只建议先 A/B，不要与 `range_max` 同批改。回退：单参数。

### C6 高度带：从「雷达面之下」改成「`base_link` 之上 0.10~1.00」
- **改什么**：p2l `target_frame: base_link`（我们现在是 `livox_frame`）、`min_height 0.10`（nav 模式）/`0.15`（建图）、`max_height 1.00`。
- **抄哪**：`slam.launch.py` / `nav.launch.py` 的 p2l 参数块（注意：**不是** `pointcloud_to_laserscan/launch/*` 那两个 sample）。
- **收益**：把地面回波整段切掉（我们现在 `-1.0~0.1` 是**从上往下扫**，会收进地面），SLAM 匹配更干净；且高度带相对 `base_link` 后与雷达安装高度解耦。
- **风险 · 回退**：我们墙只 0.40 m、近场回波 0.10~0.196 m ⇒ 若扫描面抬到雷达上方 0.10 m，**矮墙可能整段掉出高度带**（这就是集成计划里担心的那条，方向是对的）。必须先跑 z 剖面（`cloud_z_profile.py`）再改。回退：单参数。

### C7 MPPI 配方（我们 P2 想做的事，这里给「必须一起抄」的全集）
- **改什么**：`nav:=mppi` 时，除 `motion_model: Omni / time_steps 60 / model_dt 0.05 / batch_size 2000` 外，把「消震荡三件套」一起抄：`temperature: 0.25`、`gamma: 0.008`、`reset_period: 1.0`、`prune_distance: 1.7`；critics 权重 `GoalCritic 15.0/thr 2.5`、`CostCritic cost_weight 3.0 + critical_cost 253.0 + consider_footprint true`、`PathFollowCritic 5.0/1.5`、`PathAlignCritic 10.0/1.5`、`ObstaclesCritic repulsion 1.5/critical 20.0/consider_footprint false/collision_margin 0.1`；`goal_checker xy_goal_tolerance 0.25 + yaw_goal_tolerance 6.28`；`velocity_smoother smoothing_frequency == controller_frequency`、`feedback OPEN_LOOP`、`deadband 0.05`、`max_accel/decel` 与 MPPI 的 `ax/ay/az` 对齐。
- **抄哪**：`nav2_params.yaml:111-198` + `:75-79` + `:473-485`。
- **收益**：这三件套是他们**用三轮 commit 换来的**（`a7299b60a`→`a1c33ad80`→`4651c925a`），直接抄可以跳过同样的震荡调试；`temperature 0.25/gamma 0.008` 是「更果断、允许更快速度变化」，`reset_period 1.0` 是 warm-start 提速。
- **风险 · 回退**：`PathAlignCritic.use_path_orientations: false` 是 Omni 专用假设；`critical_cost 253.0` 依赖 0.05/0.04 m 分辨率与 `inflation_radius 0.75` 的梯度形状，**与 C8 必须同批**否则又变成「整个膨胀区都当碰撞」。回退：`nav:=rpp`。

### C8 inflation 梯度（`cost_scaling_factor 5.0` + `inflation_radius 0.75`）
- **改什么**：两图 `inflation_layer` 统一 `cost_scaling_factor: 5.0`、`inflation_radius: 0.75`。
- **抄哪**：`nav2_params.yaml:219-221`（local）/`:304-306`（global）。
- **收益**：注释直说「原 0.3 衰减太慢，MPPI 没有梯度可用」；对我们 NavFn + MPPI 都有好处（路径居中于代价谷底）。
- **风险 · 回退**：我们机器人 `robot_radius 0.40`（他们是 0.30 m 方框），0.75 的膨胀在窄通道可能直接判死。建议按 `inflation_radius = robot_radius + 0.35` 比例落地（≈0.75 对 0.40 半径就是 0.75），先只改 `cost_scaling_factor`，`radius` 保持 0.5/0.55 做对照。回退：单参数。

### C9 Smac2D 的**有效**参数
- **改什么**：`planner:=smac2d` 时抄 `tolerance: 0.5`、`allow_unknown: true`、`cost_travel_multiplier: 4.0`、`max_planning_time: 4.5`、`max_iterations: 1000000`、`max_on_approach_iterations: 1000`、`cache_obstacle_heuristic: true`、`smooth_path: true` + `smoother: {max_iterations 10000, w_smooth 0.4, w_data 0.1, tolerance 1.0e-10, do_refinement true}`。
- **抄哪**：`nav2_params.yaml:383-413`。
- **收益**：`cost_travel_multiplier 4.0` 把路径推向代价谷底（我们 NavFn 容易贴缝走）；`smooth_path` + `w_smooth 0.4` 是**他们链路里真正生效的平滑**。
- **风险 · 回退**：Smac 比 NavFn 慢；`smooth_path` 只保证「平滑」不保证不切角 ⇒ 必须核 footprint 碰撞。**不要抄** `motion_model_for_search: "DUBIN"`、`angle_quantization_bins: 72`、`minimum_turning_radius: 0.05`、`analytic_expansion_*`、`rotation_penalty`、`lookup_table_size`、`allow_reverse_expansion`（Hybrid/Lattice 专用，2D 插件无效）。回退：`planner:=navfn`。

### C10 代价图几何方法论（不是照抄数值）
- **改什么**：局部/全局 `update_frequency`、`publish_frequency` 提到 20.0；全局用 `rolling_window: true` + 大窗口（他们 50×50 @0.04）。
- **抄哪**：`nav2_params.yaml:202-215` / `:283-296`。
- **收益**：`20 Hz` 发布 + `always_send_full_costmap: true` 让 MPPI 每个控制周期拿到新鲜代价图；全局滚动窗在「先验图很小（他们 10.65×10.2 m）」时不会把规划限制在已知区。
- **风险 · 回退**：`always_send_full_costmap: true` 在 50×50@0.04 上是 1250×1250（≈1.5 MB/帧 @20 Hz）——**我们场地更小，别抄 50×50**。回退：回 5 Hz。

### C11 footprint 覆盖 robot_radius 的写法（Nav2 语义提醒）
- **改什么**：不要同时依赖 `robot_radius` 与 `footprint`。他们写了 `robot_radius: 0.2` 但又给了 `footprint: [[0.15,0.15],[0.15,-0.15],[-0.15,-0.15],[-0.15,0.15]]`（0.30 m 方框）——**Nav2 以 `footprint` 为准**。
- **抄哪**：`nav2_params.yaml:210-211` / `:288-289`。
- **收益/风险**：知道了这一点才能正确解释他们「0.30 m 方框 + inflation 0.75」的激进组合；我们 `robot_radius 0.40` 比它大 33%，任何照抄的膨胀/间距参数都要按比例重算。**风险**：误以为 `robot_radius` 生效会低估他们的真实几何。回退：无（认知项）。

### C12 `fake_vel_transform` 的「朝向解耦」语义（设计级借鉴）
- **改什么**：把 `spin_speed` 的两种语义**显式分档**：`spin_speed == 0` → 直通（我们已修）；`spin_speed != 0` → 覆盖 `angular.z` 且把 Nav2 的 `yaw_goal_tolerance` 放宽（他们 `6.28`）、`robot_base_frame` 指向伪帧。
- **抄哪**：`fake_vel_transform/src/fake_vel_transform.cpp`（TF 用 `-yaw(odom pose)`）、`nav2_params.yaml:79`（`yaw_goal_tolerance: 6.28`）、`CLAUDE.md` 的 TF 树说明。
- **收益**：自转哨兵语义下 Nav2 只控位置，避免局部规划器被朝向变化带偏；他们整条链路是自洽的（`angular.z≡0` + Omni + yaw 容差 6.28）。
- **风险 · 回退**：一旦 `spin_speed != 0`，MPPI 的 `wz`（±2.5）被**丢弃**，若 `yaw_goal_tolerance` 未同步放宽就会永远到不了「到点」判定；回退：`spin_speed := 0` 且容差回默认。

### C13 任务层 BT 结构（只借结构，不借数值）
- **改什么**：`RateController hz="3.0"` 重规划 + `RecoveryNode` 分层重试（ComputePath 1 次 / FollowPath 10 次）+ 失败分别清 global/local 代价图 + `RemovePassedGoals radius="0.7"`。
- **抄哪**：`nav_bringup/behavior_trees/*.xml`。
- **收益**：3 Hz 重规划对动态障碍足够且省 CPU；「清图恢复」比「原地转圈」便宜。
- **风险 · 回退**：`number_of_retries=10` 且 XML 里**没有** `IsStuck` 判据 ⇒ 可能长时间卡死而不触发 back up（他们的 `BackUp 1.0 m @1.0 m/s` 只在多点 BT 里）。回退：用回 stock BT。

### C14 诊断技巧：p2l 是「惰性订阅」
- **改什么**：A/B 时确认 `/scan` 至少有 1 个订阅者，否则 p2l 不消费点云（`subscriptionListenerThreadLoop`）。
- **抄哪**：`pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp`。
- **收益**：避免把「没人订阅 `/scan`」误判为「点云/驱动坏了」。回退：无。

---

## D. 对我们**不采纳**清单（含理由）

| # | 不采纳的东西 | 理由 |
|---|---|---|
| D1 | **无重定位 + 静态 `map→odom`(z=0.05)** | 我们 4 种重定位（amcl/icp/slam_toolbox/cartographer）是资产；静态桥把「机器人在 `map` 里的位姿」直接等同于 LIO 的 `odom` 位姿，**只有在**①先验图恰在同一 odom 原点/朝向采集、②每次启动都停在同一位置同一朝向、③能容忍 odom 长期漂移时才成立；`nav.launch.py` 里没有任何初始化位姿机制（无 AMCL、无 initialpose 注入）。**适用条件**就写在 §A5 的推论里：可用于「纯在线建图（无重定位）」模式的过渡，不能作为默认。 |
| D2 | **`mapper:=slam_toolbox` lifelong + `transform_publish_period: 0.0`** | ①lifelong 与我们的先验图/重定位资产冲突；②`transform_publish_period 0.0` 让 SLAM **只改 `/map` 不改位姿**，回环修正无法体现在机器人位姿上 ⇒ 静态层会在机器人脚下平移。 |
| D3 | **`save_pcd: true` 常开 + `ROOT_DIR` 编译期写死源码目录** | 全量点云（`PointcloudMapping(0.02)`）常驻内存并在长跑中膨胀；`param_deliver.h.in` 把 `@CMAKE_CURRENT_SOURCE_DIR@` 编进二进制，`map_save` 会往**源码树**写 `pcd/scan.pcd`。要用就改成参数化路径 + 按需开启。 |
| D4 | **速度包线：±7.5 m/s、±5.0 m/s²、50 Hz、`vx_std 0.5`、`reset_period 1.0`** | 实车参数；仿真里 RoboMaster 底盘/接触模型与实车差距大，直接移植会让 MPPI 输出被物理引擎「吃掉」并掩盖真实问题。我们按 2.0 m/s 起标。 |
| D5 | **全局 50×50 m @0.04 + `always_send_full_costmap: true`** | 1250×1250 的每帧全量发布对我们 15×28 m 场地纯属浪费（CPU/带宽），且会把大量 `unknown` 卷进规划。 |
| D6 | **裁剪盒数值照抄（±0.3/−0.3~0.5/−0.1~0.2）** | 车体不同、裁剪坐标系是 `livox_frame` 而 marker 画在 `base_link`（z 差约 0.3 m），照抄等于把裁剪区放错位置。 |
| D7 | **工程卫生问题** | ①`bt_navigator.odom_topic: "odomety"`（他们自己在 `CLAUDE.md` 里承认的 typo）；②`velocity_smoother.odom_topic: "odometry"`；③`map_server.yaml_filename` 写死 `/home/cod-sentry/qza_ws/...`；④`slam.launch.py:15` 的 `get_package_share_directory('fast_lio')` 是**死引用**——`CLAUDE.md` 只说「dead references」，但该 API 在包不存在时会**抛 `PackageNotFoundError` 让整个 launch 起不来**（**推断**，我未运行验证）；⑤`cod_nav.rviz` 里 pb_omni 的遗留话题（`/lookahead_point`、`/curvature_points_marker_array`）；⑥`smoother_server` 启动了但 BT 里没有 `SmoothPath`（死代码）；⑦仓库里带 `.idea/`（含 39,943 B 的 `editor.xml`）、`launch/__pycache__/*.pyc`、根级 `.idea/` 与 `nav_bringup/.idea/` 两份。 |
| D8 | **RealSense 相机链路** | `nav.launch.py` 启了 `realsense2_camera`，但 `observation_sources` 只选 `livox_source`，点云未接入任何代价图 ⇒ 是「未接线的死配置」，且对我们 Gazebo 仿真无意义。 |
| D9 | **`plugin_lib_names` 里 BT 未用到的节点** | `nav2_is_stuck_condition_bt_node`、`nav2_spin_action_bt_node`、`nav2_assisted_teleop_*` 等只是被列在 yaml 里，两棵 BT XML 都没用（缺 `IsStuck` 恰恰是 D 项理由之一）。 |
| D10 | **「大图 unknown」策略** | `track_unknown_space: true` 双开 + 50 m 滚动窗的组合对我们小场地无收益；我们按 `docs/debug_fastlio_cartographer.md §10.5 遗留⑤` 单独定策略（与集成计划 §1 最后一行结论一致）。 |
| D11 | **地面分割被高度带取代** | 我们保留 `linefit_ground_segmentation` 作为可测资产；他们的 `min_height 0.10/0.15` 硬高度带对坡道/矮墙不鲁棒（他们 2025 还有 `patchworkpp`，2026 也砍了）。 |

---

## E. 对 `docs/cod_nav_2026_integration_plan.md` 的修正/补充建议（逐条，不直接改文件）

> 该文件我已只读通读（144 行）。以下按它的行号/小节给建议。**我没有改动它。**

1. **§1 表「3D 点云预处理」行 —— `leaf 0.05` 是错的（死参数）**。`cpp_lidar_filter/src/filter_node.cpp` 里 `pcl::VoxelGrid` 整段被注释（第 96–107 行），`leaf_size` 只被 `declare_parameter`/`get_parameter`，从未被使用。建议改为：「只裁剪、无降采样（`leaf_size` 为死参数）」。
2. **§1 同一行 —— 只写了 `nav.launch.py` 的一套盒子**。`slam.launch.py`（建图主入口）用的是另一套：`x±0.2 / y −0.2~0.4 / z −0.1~0.2`。建议注明「两套 launch 各一套数值」，避免误以为是唯一值。
3. **§1 同一行 —— 补坐标系事实**：`CropBox` 在**输入点云自身的坐标系**（`livox_frame`）工作，不做 TF；而节点发布的 `crop_box_marker` 的 `frame_id` 是 `base_link`（源码硬编码）⇒ 可视化与实际裁剪区错位（我们 `livox_frame` 相对 `base_link` 有 z≈0.3 偏移）。建议把「数值须按我们车体重标」升级为「**必须在 `livox_frame` 下重标，且不要相信 marker 的位置**」。
4. **§1 表「/scan 射程与高度带」行 —— `0.01~1.00` 应改为 `0.10/0.15~1.00`**：`slam.launch.py` 是 `min_height 0.15`（`485f2ebfa` 从 0.1 提上去的），`nav.launch.py` 是 `0.10`；两者 `max_height` 都是 `1.00`。同时补 `target_frame: base_link`（**不是 `livox_frame`**）与 `angle_min/max = ∓3.1416`（**整圆**；源码里注释写 `-M_PI/2` 是错的）、`angle_increment 0.0087`（723 线）、`scan_time 0.3333`。
5. **§1 同一行 —— 输入话题写清楚**：真正被使用的是 `/livox/lidar`（PointCloud2）；`/livox/lidar/pointcloud` 只出现在仓库自带的 `sample_pointcloud_to_laserscan_launch.py` 里（那份还有 `target_frame: chassis`、`5.0` 射程，未被 `nav_bringup` 使用）。另外提醒：我们 bench 现在的 `/scan` 参数（`livox_frame`、`−1.0~0.1`、`10.0`）与**他们另一份 sample**（`sample_laserscan_to_pointcloud_launch.py`，`livox_frame`、`−1.0~0.1`、`0.45~10.0`）几乎一致 ⇒ 推测我们当初就是抄那份 sample 的，可作为「为什么我们高度带在雷达面之下」的溯源依据。
6. **§1 表「路径平滑」行 —— 收益证据不足，需改叙述**。他们的自研 BT 里**没有 `<SmoothPath>`**，`smoother_server` 只被 `navigation_launch.py` 启动、从不被调用 ⇒ **smoother 在他们的实跑链路里是死代码**；真正的平滑来自 `SmacPlanner2D.smooth_path: true` + 内置 `smoother{w_smooth 0.4, w_data 0.1, do_refinement true}`。建议把 `smoother:=savgol` 的定位改成「我们自研槽位（需在自己的 BT 里显式加 `SmoothPath` 才生效）」，并把「抄 Smac 的 `smooth_path`」列成更优先的低风险项。
7. **§1 表「全局规划」行 —— 删掉/标注 Hybrid 专用参数**。`motion_model_for_search: "DUBIN"`、`angle_quantization_bins: 72`、`minimum_turning_radius: 0.05`、`analytic_expansion_*`、`rotation_penalty`、`lookup_table_size`、`allow_reverse_expansion` 都是 Hybrid/Lattice 字段，对 `SmacPlanner2D` 无效；真正生效的是 `cost_travel_multiplier 4.0`、`tolerance 0.5`、`allow_unknown true`、`max_planning_time 4.5`、`smooth_path true`。另：他们不是「2025 是 SmacHybrid」——2025 的 `nav2_params` 已用 `SmacPlanner2D`，只是同时留了 Hybrid 字段。
8. **§1 表「局部控制」行 —— 补三个漏掉的关键值**：`temperature: 0.25`、`gamma: 0.008`、`reset_period: 1.0`（Humble 专属）、`prune_distance: 1.7`、`ax/ay_max 5.0`、`az_max 3.5`、`ObstaclesCritic{repulsion 1.5, critical 20.0, consider_footprint false, collision_margin_distance 0.1}`、`CostCritic{trajectory_point_step 2, near_goal_distance 0.5}`。这些正是 `a7299b60a→a1c33ad80→4651c925a` 三轮「消目标点震荡 + 提速」的产物。
9. **§1 表「局部控制」行 / §4 骨架 —— 补 goal checker**：`xy_goal_tolerance: 0.25`（`a1c33ad80` 注释：「放宽到 25cm，**高速 omni 机器人 10cm 太严导致绕圈**」）与 `yaw_goal_tolerance: 6.28`。`nav:=mppi` 若不放宽 xy 容差，会复现他们踩过的绕圈。
10. **§1 表「局部障碍表示」行 —— 补 STVL 的层级覆盖陷阱**：layer 级 `obstacle_range: 3.0` 与 source 级 `8.0/9.0` 并存，生效的是 source 级；以及 `decay_model: 0`(linear)、`voxel_size: 0.05`、`observation_persistence: 0.0`、`combination_method: 1`、`track_unknown_space: true`、`model_type: 1`、`vertical_fov_angle: 2.00`、`filter: "voxel"`、`transform_tolerance: 0.2`。**特别是 `model_type: 1`** 我们计划里没写，这是最容易漏且最影响正确性的一个。
11. **§1 表「任务层」行 —— 三处与 XML 不符**：①`IsStuck→BackUp` 不成立：两棵 BT XML 里都没有 `IsStuck` 节点（它只在 `plugin_lib_names` 里），`ReactiveFallback name="FollowPathWithStuckDetection"` 下只有一个 `FollowPath`；②`BackUp` 只在 `navigate_through_poses_*` XML 里，且是 `backup_dist="1.0"` `backup_speed="1.0"`（**不是 1.5 m/s**）；③`navigate_to_pose_*` 的恢复动作只有清 global/local 代价图。§2 P4 与 §3 矩阵里「1.5 m/s」的表述要同步改。
12. **§1 表「任务层」行 —— 补两个 2026 特有的事实**：`RateController hz="3.0"` 确实存在；`RemovePassedGoals radius="0.7"`（只在多点 BT）；航点来自 `WPs/t1.csv`（3 点）+ RViz `waypoint_editor/WaypointEditorPanel`（外部包，`InteractiveMarkers` namespace `/interactive_marker_server`）；决策来自 `scripts/bt.sh` 轮询 `/serial/receive` 的 `game_type`（2→(4.55,−2.0)，3→(0,0)）。
13. **§1 表「哨兵语义」行 —— 表述需修正为「他们其实不自转」**。全分支 `spin_speed` 仅出现 5 次（全部在 `fake_vel_transform` 包内），**没有任何文件把它设为非零**，`slam/nav.launch.py` 都只传 `use_sim_time` ⇒ 他们实跑时 `angular.z ≡ 0`（`wz` 被丢弃），配合 `robot_base_frame: base_link_fake` + `yaw_goal_tolerance 6.28` 实现「只控位置」。所以：**`sentry_spin` 组合是我们的自研设计，不是「抄他们」**；另外要标明我们的语义与他们不同（我们 `spin_speed==0` 时直通、保留 `wz`）。
14. **§1 表「重定位」行 —— 结论正确，建议补证据与前提**。证据：`localization_launch.py` 的 `lifecycle_nodes = ['map_server']`（无 amcl）、`nav2_params.yaml` 无 `amcl:` 段、`slam.launch.py`/`nav.launch.py` 的静态 `map→odom`(z=0.05)、`mapper_params_async.yaml` 的 `transform_publish_period: 0.0`。前提：**必须在 `t1.yaml` 的 origin（`[-3.84, −6.05]`，图仅 10.65×10.2 m）处、同朝向启动**。
15. **§1 表「在线建图」行 —— 补 `transform_publish_period: 0.0` 及其后果**（SLAM 只改图不改位姿），并把 `mode: lifelong` 之外的关键参数补上（`map_update_interval 2.5`、`minimum_time_interval 0.5`、`minimum_travel_distance 1.0`、`max_laser_range 5.0`、`resolution 0.05`）。
16. **§1 表「LIO」行 —— 「先用 `lio:=pointlio` 顶替验证」这个推论需要加一个大前提**（**这是全篇最重要的修正**）：`small_point_lio` 的 PointCloud2 适配器**强制要求点云带 `tag`(uint8) 与 `timestamp`(float64, 秒) 字段**（`livox_pointcloud2.h`：`PointCloud2ConstIterator<uint8_t> out_tag(msg,"tag")`、`...<double> out_timestamp(msg,"timestamp")`，只保留 `(*tag & 0b00111111)==0` 的点），否则迭代器构造即失败。我们仿真链路的 `/livox/lidar` 是 CustomMsg、`/livox/lidar/pointcloud` 是普通 PointCloud2 ⇒ **不能直接换 LIO**。可选项：①走 `livox_custom_msg` 适配器（需编译期 `HAVE_LIVOX_DRIVER`，即装 `livox_ros_driver2`）；②给仿真点云补 `tag`/`timestamp` 字段；③只当参考实现读，不落地。另外它把 `header.frame_id`/`child_frame_id` 硬编码为 `odom`/`base_link`，且 **`twist` 全 0**（源码 TODO 注释）——任何依赖 `/Odometry.twist` 的模块（含我们自己可能的 CLOSED_LOOP 平滑、速度监视）都会读到 0。
17. **§2 P0 —— 补「先验图/静态桥的启动前提前提」与「STVL 0.5 s 衰减会擦薄结构」的风险**；并注明他们 `slam_toolbox` 侧 `max_laser_range: 5.0`，所以 `range_max 20` 的收益**主要在 costmap 与 scan 层**，不会让 SLAM 的 raster 图变大（把 P0 判据「`inf` 占比明显下降」的前提写清）。
18. **§2 P3 —— 「评估是否引入 `small_point_lio`」应补三条硬约束**：`save_pcd: true` 默认常开 + `ROOT_DIR` 写死源码目录；`map_resolution: 0.5`（比我们 0.05 粗 10 倍，对我们 0.40 m 半径车是否够用需实测）；上游确权（README 无 git URL；**推测**是 `Yancey2023/small_point_lio`，MIT）。
19. **§3 矩阵 —— 建议新增/改写两行**：①「`p2l` 惰性订阅 × A/B 无人订阅 `/scan`」→ 假故障（`subscriptionListenerThreadLoop` 只在有订阅者时才订阅 `cloud_in`）；②「`smoother:=savgol` × 我们自研 BT」→ 必须显式加 `<SmoothPath>`，「冗余」这一栏的理由应改成这个（与 TEB 无关的那半边结论保留）。
20. **§3 矩阵「高度带改到雷达面之上 × 我们场地矮墙」**：结论对，建议补上他们的真实数值（`base_link` 之上 `0.10`/`0.15`~`1.00`，且**高度判定在 `target_frame` 变换之后**做，源码可证），并注明我们墙 0.40 m 是反对照抄的量化依据。
21. **§4 骨架 —— 键名与来源修正**：`CostCritic` / `GoalCritic` / `PathFollowCritic` / `PathAlignCritic` 的权重键名是 **`cost_weight`**（不是 `weight`）；`CostCritic` 补 `cost_power: 1`、`collision_cost: 1000000.0`、`near_goal_distance: 0.5`、`trajectory_point_step: 2`；`ObstaclesCritic` 补齐；`smoother:=savgol` 补 `window_size: 7`、`poly_order: 3`、`refinement_num: 2`；`planner:=smac2d` 删掉 Hybrid 专用字段；`local_obstacle:=stvl` 补 `model_type: 1`、`vertical_fov_angle: 2.00`、`filter: "voxel"`、`decay_model: 0`；`crop_box` 段加注释「坐标在 `livox_frame`；`leaf_size` 无效」。
22. **§5「明确不做的事」—— 建议把 §5.3 拆细并加两条硬理由**：①`get_package_share_directory('fast_lio')` 的死引用会让 `slam.launch.py` **启动即失败**（推断）；②`save_pcd: true` + `ROOT_DIR` 会往源码树写文件。另外把「不引入 ESDF/动态障碍」的理由从「他们也没做」升级为「2026 唯一的动态性只是 STVL 的 0.5 s linear decay，`esdf`/`frontier` 全分支 grep 0 命中」（更硬）。
23. **§6 验收方式 —— 建议加一条上游溯源记录**：本文 §A/§B 的 URL 与 commit SHA 可作为引用锚点（`485f2ebfa` HEAD、`a7299b60a`/`a1c33ad80`/`4651c925a` 消震荡三轮、`632fbb4eb` 砍重定位、`9d54b5ab8`+`d1c027958` 换 LIO、`9856cff66`/`9dfbeabdd` 做减法）。
24. **§7「关于『2026 用的是不是 FAST-LIO』」—— 三点补充**：①时间线可写死（`9d54b5ab8` 2025-11-18 remove FAST_LIO → `d1c027958` 2025-12-28 add small_point_lio → `a7299b60a` 2026-03-10 删 `FAST_LIO_ROS2/`）；②`CLAUDE.md` 确实是他们唯一设计文档（5,306 B，`a7299b60a` 新增、`485f2ebfa` 最后改），但它同时也是「自承 typo」的地方；③第 5 条「只借加法部分」的判断我完全同意，可再加一句量化依据：他们 2026 的有效创新几乎全在**参数层**（MPPI critics、inflation 梯度、STVL decay、高度带、BT 结构），唯一的结构层变化是**换 LIO** 与**砍重定位/地面分割**。
25. **建议新增一节 §8「本文与 `cod_nav_2026_deep_dive.md` 的对应关系」**：把本文件 §C（可采纳）与 §D（不采纳）逐条映射到该计划的 P0–P4，避免两份文档各自漂移。

---

## F. 未能确定 / 读不到的东西（明确列出）

1. **`livox_pointcloud2.h` 的实时性未能 100% 复核**（首次抓取成功、后续 SSL 重试失败）：该文件的核心逻辑我已读到（`tag`/`timestamp` 字段、`(*tag & 0b00111111)==0` 过滤、`SensorDataQoS`），但**未做第二次独立复核**。其余 40 余个 `small_point_lio` 文件也未逐行读完（`small_ivox.h`、`eskf.h`、`so3_math.h`、`pcd_io.cpp`、`voxelgrid_sampling.cpp` 只读了接口与文件头注释）。
2. **上游仓库确权未完成**：`small_point_lio/README.md` 里**没有任何 git URL**（只有邮箱 `1709185482@qq.com`、QQ 群 1070252119、bbs 链接）。检索到的最可能候选是 `Yancey2023/small_point_lio`（**未用 API 确权**）。另外 `small_gicp` 在 CI 里被安装，但 `CMakeLists.txt` **不 find 它**，属冗余步骤（未确认是否为历史残留）。
3. **谁发布 `base_link→livox_frame`**：分支内**没有** `robot_state_publisher` 的启动、`bringup.launch.py`/`lio_only.launch.py` 等已被删；`small_point_lio` 又强依赖该 TF（`lookupTransform(lidar_frame, "base_link")` 失败即丢帧、无 odom/TF）。唯一仓库内的定义是 `nav_bringup/urdf/carto.xacro`（`xyz="0.0 -0.045 0.3"`，**未被任何 launch 引用**）和 `simulation_waking_robot.xacro`（父链接是 `chassis`，同样未引用）。⇒ **无法确定他们在实机/仿真里从哪加载 URDF 与静态 TF**，这直接影响「他们那套能不能被我们直接复现」。
4. **`nav_bringup/map/t1.pgm` 的像素内容未读**：只读了 header（`P5 213 204 255`）与 `t1.yaml`（`origin [-3.84, -6.05, 0]`、`resolution 0.05`、`trinary`）⇒ 推得 10.65 m × 10.20 m，未核对实际占据/自由分布。
5. **`nav_bringup/rviz/cod_nav.rviz` 的 39 KB 全文未逐行读**：只提取了 display 名称、话题（`/scan`、`/global_costmap/costmap(+_updates)`、`/local_costmap/costmap(+_updates)`、`/local_costmap/voxel_grid`、`/plan`、`/local_plan`、`/lookahead_point`、`/curvature_points_marker_array`、`/trajectories`、`/slam_toolbox/graph_visualization`、`/crop_box_marker`、`/global_costmap/published_footprint`，Fixed Frame `map`，面板 `waypoint_editor/WaypointEditorPanel`，`InteractiveMarkers` namespace `/slam_toolbox` 与 `/interactive_marker_server`）。
6. **`.idea/editor.xml`（39,943 B）与 `nav_bringup/.idea/editor.xml`（39,943 B，同大小）未读**：判定为 IDE 元数据、无技术价值；若其中含调试运行配置（Run/Debug configuration 常被 Copilot/IDEA 写入）可能有额外信息，我**没有**核实。`.idea/copilot.data.migration.*.xml`、`.idea/cod_nav.iml`、`.idea/misc.xml`、`.idea/vcs.xml`（内含一条失效的 `patchwork-plusplus` vcs 映射）也未细读。
7. **外部依赖的行为无法核实**（不在分支内，只能从我们的 launch/params 反推）：`livox_ros_driver2`（他们 CI 用 `qza36/livox_driver2_ros2` fork）、`cod_serial_ul26`（吃 `/aft_cmd_vel`、发 `/serial/receive` `game_type`）、`spatio_temporal_voxel_layer`（2026 只留参数，subtree 副本当天删掉）、`waypoint_editor`、`realsense2_camera`、`slam_toolbox`、`nav2_smac_planner`/`nav2_mppi_controller`。⇒ 这些包的话题名/QoS/频率我**没有**独立证据。
8. **`pointcloud_to_laserscan` fork 与上游的差异未逐行 diff**：我在意的是 `package.xml` 版本号与他们的本地改动量（只读了 `README.md`、`CHANGELOG.rst` 头部、`CMakeLists.txt`、两个 sample launch 与 node 源码的关键段落）。我们 bench 里的 p2l 若是同源同版本，则 §C5/§C6 的移植是纯参数改动；**这一点未验证**。
9. **所有「会导致失败」的判断都是源码/文档推断，未实测**：①`get_package_share_directory('fast_lio')` 在包缺失时抛 `PackageNotFoundError` 导致 `slam.launch.py` 起不来；②`small_point_lio` 因 `tag`/`timestamp` 字段缺失而无法消费我们的仿真点云；③STVL `voxel_decay 0.5` linear 会擦掉 0.40 m 矮墙；④`angular.z ≡ 0` 时 MPPI 的 `wz` 被丢弃导致朝向不受控。**我没有运行任何代码**（本任务禁止执行，且我在受限沙箱内）。
10. **branch 的 CI 状态未知**：`.github/workflows/ci.yml` 只对 `push: rmul2026` 与 `pull_request: master` 触发，`skip-tests: true`，且第 3 步 `cd ws_liovx/src/...`（`liovx` 拼写错误，目录其实是 `ws_livox`）⇒ 该 CI 的第 3 步**很可能一直失败**（**推断**，未查 Actions 运行记录）。
11. **`bt.sh` 的引入时间未定位**：它不在最近 100 条提交的改动清单里（说明早于 2024-12 或通过 merge 带入），我未做完整 `git log --follow`（无 clone）。
12. **`LICENSE`**：`BSD 3-Clause`，`Copyright (c) 2025, Zhiang QI`（`README.md` 与 `CLAUDE.md` 里写的却是「2025 COD Team」）；`small_point_lio/LICENSE.txt` 是 MIT（`Copyright (C) 2025 Yingjie Huang`）——两套 license 并存，**未做合规判断**。

---

### 附：本文件所有引用的原文 URL（可直接点开复核）

| 用途 | URL |
|---|---|
| 分支树（recursive） | <https://api.github.com/repos/qza36/COD_NAV/git/trees/rmul2026?recursive=1> |
| 提交史 | <https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100> |
| 架构文档 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/CLAUDE.md> |
| README | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/README.md> |
| 主 launch（建图+导航） | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/slam.launch.py> |
| 先验图 launch | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/nav.launch.py> |
| Nav2 参数 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/nav2_params.yaml> |
| slam_toolbox 参数 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/mapper_params_async.yaml> |
| 定位 launch（只有 map_server） | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/localization_launch.py> |
| BT（到点 / 多点） | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml> |
| 航点 / 地图 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/WPs/t1.csv> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/map/t1.yaml> |
| URDF（未被引用） | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/urdf/carto.xacro> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/urdf/simulation_waking_robot.xacro> |
| 裁剪盒源码 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/cpp_lidar_filter/src/filter_node.cpp> |
| LIO 配置 / 节点 / 适配器 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/config/mid360.yaml> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio_node.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/lidar_adapter/livox_pointcloud2.h> |
| LIO 参数声明 / 预处理 / 估计器 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio/parameters.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio/preprocess.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio/estimator.cpp> |
| 伪底盘 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/fake_vel_transform/src/fake_vel_transform.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/fake_vel_transform/README.md> |
| p2l 节点与 sample | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/pointcloud_to_laserscan/launch/sample_pointcloud_to_laserscan_launch.py> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/pointcloud_to_laserscan/launch/sample_laserscan_to_pointcloud_launch.py> |
| 决策脚本 / CI | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/scripts/bt.sh> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/.github/workflows/ci.yml> |
| 关键提交 | `a7299b60a`(MPPI+CLAUDE.md) · `a1c33ad80`(消震荡) · `4651c925a`(提速) · `485f2ebfa`(HEAD) · `632fbb4eb`(砍重定位) · `9d54b5ab8`(replace fastlio2) · `d1c027958`(add small point lio) · `9856cff66`(remove livox driver) · `9dfbeabdd`(use stvl) · `872adfafd` · `4de98f194`(realsense) |
| small_point_lio 上游候选（推测） | <https://relatedrepos.com/gh/Yancey2023/small_point_lio> · <https://bbs.robomaster.com/article/813022> |


---


## 来源 11/19：`tdt/cod_nav_2026_integration_plan.md`


> 上游：https://github.com/qza36/COD_NAV 分支 **`rmul2026`**（最后提交 2026-03-10）
> 对比依据：`docs/cod_nav_comparison.md`
> 本文件性质：**设计/排期文档**。在明确说"改"之前，不动任何配置与代码。

> ⚠️ **2026-09-26 修正声明**：`docs/cod_nav_2026_deep_dive.md` 对该分支做了逐文件深挖，**发现本文若干行有误**（共 25 条，见那份文档 §E）。以下是最影响决策的 8 条，**以深挖文档为准**：
> 1. **裁剪盒的 `leaf_size: 0.05` 是死参**：`pcl::VoxelGrid` 整段被注释，节点只做 crop、不做降采样；
> 2. **裁剪盒在输入点云坐标系（`livox_frame`）里裁，但 marker 硬编码发在 `base_link`** ⇒ 两者错开；**不要照抄他们的 box 数值**，要按我们的 livox_frame 重新算；
> 3. 他们的两条 launch 用**两套不同的 box**：`slam.launch.py` x±0.2/y−0.2~0.4；`nav.launch.py` x±0.3/y−0.3~0.5（z 都是 −0.1~0.2）；
> 4. **p2l 真值**（写在 launch 内联，不在包内）：`target_frame: base_link`、`min_height **0.15**(slam)/**0.10**(nav)`、`max_height 1.00`、`angle ±3.1416`（**整圈**，包内注释写 −π/2 是错的）、`increment 0.0087`（723 线）、`range_min 0.5`、`range_max 20.0`、`use_inf true`、输入是 **`/livox/lidar`(PointCloud2)** ⇒ 本文第 1 节里的「0.01~1.00」应改为「0.10/0.15~1.00」；
> 5. **`smoother_server` 在他们的实时链里是死代码**（自定义 BT 里没有 `<SmoothPath>`）⇒ 真正的平滑是 **SmacPlanner2D 的 `smooth_path: true` + 内建 smoother**；本文"借 savgol"要改成"先把 Smac 的 smooth_path 打开"，savgol 只是我们自己可选的加分项；
> 6. **BT 事实修正**：两个 XML 里**都没有 `IsStuck` 节点**；`BackUp` 是 **1.0 m @ 1.0 m/s**（不是 1.5）且只在 through-poses BT 里；`navigate_to_pose` 的恢复只有 `ClearEntireCostmap`；`RateController 3 Hz` 与重试 10 次是对的；
> 7. **他们其实从不自转**：`spin_speed` 全分支只出现 5 次（源码/头文件/README），**没有任何文件把它设成非零** ⇒ `angular.z ≡ 0`（MPPI 的 wz ±2.5 被丢弃）+ `robot_base_frame: base_link_fake` + `yaw_goal_tolerance 6.28` = 纯位置控制。**所以"只控位置"是他们的事实行为，而"`spin_speed` 直通修复 + sentry_spin 模式"是我们的设计，不是抄来的**；
> 8. **换 LIO 有硬门槛**：`small_point_lio` 的 PointCloud2 适配**要求 `tag`(uint8) 与 `timestamp`(float64, 秒) 字段**（`src/lidar_adapter/livox_pointcloud2.h`），我们的仿真点云没有 ⇒ **不能直接替换**；它的 `frame_id/child_frame_id` 硬编码 `odom/base_link`，且 **twist 全零**（源码 TODO）。本文 §7 第 4 条"先用 `lio:=pointlio` 顶替"仍然成立，但"vendor small_point_lio"要重新评估成本。

## 0. 总原则（五条）

1. **只"增槽"，不"换默认"**：COD_NAV 的做法一律作为**新槽位值/新候选**加入；默认路径（rpp + cartographer + amcl）保持不变，随时可回退。
2. **一次一个变量 + P0 回归做闸门**：每步都用 `tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5` 验收；不通过就回退，不叠加下一步。
3. **先契约，后算法**：TF / 时间戳 / QoS / 消息类型先对齐（前两天 6 个病根全是契约问题），再谈换算法。
4. **不引入非必要依赖**：能用现有包（`nav2_smac_planner`、`nav2_mppi_controller`、`spatio_temporal_voxel_layer` 都已安装）就不新编译。
5. **每步写回退方式**：文档里必须写清"怎么退回去"。

## 1. 目标架构对照（轴 × 现状 × COD_NAV 2026 × 落地方式）

| 轴 | 我们现状 | COD_NAV 2026 | 我们的落地方式 | 优先级 |
|---|---|---|---|---|
| LIO | `lio:=fastlio|pointlio|none|cartographer` | **`small_point_lio`（仓库内自带；2026 分支没有 FAST-LIO）** | 先用已有的 `lio:=pointlio`（同族、零成本）；**直接 vendor 有门槛**：它要求点云带 `tag`+`timestamp` 字段（见深挖 §E/§F） | P3 |
| 在线建图 | `mapper:=cartographer|slam_toolbox` | **slam_toolbox async `mode: lifelong`** | `mapper` 增加取值 `slam_toolbox_lifelong`（只改参数文件，不新增包） | P3 |
| 重定位 | AMCL / ICP / slam_toolbox / cartographer | **无（静态 `map→odom`）** | **不采用**（我们四种更强）；只在"纯在线建图"模式下允许静态桥 | — |
| 3D 点云预处理 | 无（直接用 `/segmentation/obstacle`） | **`cpp_lidar_filter` 车体裁剪盒**（x±0.3, y−0.3~0.5, z−0.1~0.2, `negative:true`, leaf 0.05） | 新增 **`rm_cloud_crop`**（或先用 `pcl_ros` 的 `PassThrough`×3 组合）→ `/livox/lidar_filtered`，**只接 stvl 与 local cloud 层**；`/scan` 链不动 | **P0** |
| `/scan` 射程与高度带 | `range_min 0.05`、`range_max **10.0**`、带在雷达面**之下**(−1.0~0.1) | `range_min 0.5`、`range_max **20.0**`、带在雷达面**之上**(0.01~1.00) | `range_max → 20.0`（P0）；高度带**必须按我们传感器重算**（我们的墙仅 0.40 m、近场回波 0.10~0.196 m ⇒ 照抄会丢矮墙） | **P0** |
| 全局规划 | NavFn（`allow_unknown: true`） | Smac2D + `cost_travel_multiplier 4.0` + `tolerance 0.5`（2025 是 SmacHybrid DUBIN） | `planner` 槽：`navfn`(默认) / `smac2d` / `smac_hybrid`；Smac2D 参数抄上游、`allow_unknown` 保持 true | P1 |
| 路径平滑 | `SimpleSmoother` | **SmacPlanner2D 的 `smooth_path: true` + 内建 smoother**（他们的 `smoother_server` 在实时链里是**死代码**，BT 无 SmoothPath） | 先只打开 Smac 的 `smooth_path`；`smoother:=savgol` 降为可选 | P1 |
| 局部障碍表示 | local: `scan|cloud|both`（无时间维） | **local 也用 STVL**（`voxel_decay 0.5`、`voxel_size 0.05`、`obstacle/raytrace 8/9 m`、`min_h 0.1 / max_h 1.0`） | `local_obstacle` 增加取值 `stvl`；先只加"衰减"这一条，不动其它层 | **P0** |
| 局部控制 | `nav:=rpp|dwb|teb` | **MPPI Omni 50 Hz**（vx/vy 7.5 m/s，critics：GoalCritic 15/2.5、CostCritic `critical_cost 253`+`consider_footprint`、PathFollow/PathAlign `threshold 1.5`、`temperature 0.25`、`gamma 0.008`、60 步/2000 采样） | `nav:=mppi`（包已装）→ 独立 params 文件，**速度从 2.0 m/s 起调**，`controller_frequency 30` 起 | P2 |
| 任务层 | stock BT + 自研分段工具 | BT：**3 Hz 重规划** + `IsStuck→BackUp(1 m@1.5 m/s)` + 10 次重试；目标由 bash 轮询裁判串口 | 先**只借 BT 结构**（若确认有利再改 XML）；目标选择仍用我们的方式 | P4 |
| 哨兵语义 | `spin_speed==0` 直通（已修）；`spin_speed!=0` 小陀螺 | **`enable_rotation: false` + `yaw_goal_tolerance 6.28`**（只控位置、朝向交给自转） | 新增**模式**（非新包）：`sentry_spin` 组合（`spin_speed!=0` + yaw 容差放宽），与位置控制解耦 | P4 |
| 未知区策略 | `allow_unknown: true`；`track_unknown_space`(根层级疑惰性) + stvl 内 true | 同样 `true` 双开，但图是 72×23 m 大面积 unknown | **不学**（他们更糟）；我们按 `debug_fastlio_cartographer.md §10.5 遗留⑤` 单独定策略 | — |

## 2. 分阶段路线（每阶段：目标 → 改动 → 判据 → 风险 → 回退）

### P0（直击当前痛点，风险最低）
- **改动**：① 车体裁剪盒 → `/livox/lidar_filtered` → stvl + local cloud；② `p2l.range_max: 10 → 20`；③ local costmap 增加 STVL（带 `voxel_decay`）。
- **判据**：`cloud_z_profile.py` 的 0–0.5 m 桶不再是空的；贴墙时 `inf` 探针在该方位有回波；`costmap_marking_check.py` 近距标记不下降；P0 回归 PASS；`range_max 20` 后 `inf` 占比明显下降。
- **风险**：裁剪盒太紧会删真障碍（须按我们车体标定，尤其 y 方向不对称要重算）；STVL 清除过激会擦薄结构（range 先给保守 8/9 m）。
- **回退**：裁剪盒节点停用即恢复原链路；`range_max` 与 STVL 都是单参数/单层，改回即退。

### P1（规划质量）
- **改动**：`planner:=smac2d`（`cost_travel_multiplier 4.0`、`tolerance 0.5`、`allow_unknown: true`）+ `smoother:=savgol`。
- **判据**：同一目标点的路径是否居中于代价谷底（不再贴缝走）；`velocity_smoother` 饱和次数下降；规划耗时 < `max_planning_time`；P0 回归 PASS。
- **风险**：Smac 比 NavFn 慢（我们地图 15×28 m 比他们 72 m 小，风险较低）；SG 平滑可能切角 ⇒ 必须核 footprint 碰撞。
- **回退**：`planner:=navfn`、`smoother:=simple`。

### P2（控制器）
- **改动**：`nav:=mppi`（Omni，30 Hz 起，速度 2.0 m/s 起，critics 抄上游配方）+ `velocity_smoother` 匹配（限速/限加速度）。
- **判据**：贴墙最小距离、窄缝通过率、震荡幅度、到达时间、CPU；与 rpp/dwb 三方对照写入 `algorithm_matrix.md §四`。
- **风险**：`batch_size 2000 × time_steps 60` 每周期 ⇒ CPU 高；参数多，须先跑默认再微调。
- **回退**：`nav:=rpp`。

### P3（LIO + 在线建图一条龙）
- **改动**：`mapper:=slam_toolbox_lifelong`（只加参数文件）；评估是否引入 `small_point_lio` 作为 `lio` 新槽。
- **判据**：长走廊/退化场景的漂移；`/map` 生长是否跟得上；与 cartographer 对照。
- **风险**：lifelong 模式与**任何重定位槽冲突**（都发 `map→odom`）⇒ 必须与 `localization` 互斥（launch 层强制）。
- **回退**：`mapper:=cartographer`。

### P4（哨兵语义 + 任务层）
- **改动**：`sentry_spin` 模式（`spin_speed!=0` + `yaw_goal_tolerance` 放宽）；按需借 BT 结构（3 Hz 重规划 / IsStuck→BackUp）。
- **判据**：目标点位置到位即可（朝向由自转负责）；卡住恢复成功率；与"只控朝向"模式对比。
- **风险**：朝向安全变成**另一个独立问题**（要有单独的朝向风险判据）；背退速度 1.5 m/s 在拥挤场地可能触发自身避障。
- **回退**：恢复 `spin_speed:=0` 直通模式（已验证可行）。

## 3. 兼容性 / 冲突矩阵

| 组合 | 是否冲突 | 说明 |
|---|---|---|
| `mapper:=slam_toolbox_lifelong` × `localization:=*` | **冲突** | 两者都发 `map→odom` ⇒ 必须互斥（沿用 launch 层条件） |
| 静态 `map→odom`（COD_NAV 2026 做法） × AMCL/ICP | **冲突** | 只在"纯在线建图、无重定位"模式下才允许 |
| `nav:=mppi` × `spin_speed!=0` | **需设计** | MPPI 会输出 `wz`，而 `fake_vel_transform` 在 `spin_speed!=0` 时用 `spin_speed` 替换它 ⇒ 二者择一：要么 `spin_speed=0` 让 MPPI 控朝向，要么把朝向交给自转（`yaw_goal_tolerance` 放宽） |
| `smoother:=savgol` × `nav:=teb` | **冗余** | TEB 自带轨迹优化 ⇒ sg 只配 rpp/dwb/mppi |
| 车体裁剪盒 × `linefit` | **不冲突** | 裁剪盒只作用于 3D 链路（stvl/local cloud）；`/scan` 仍走 linefit |
| 高度带改到雷达面之上 × 我们场地矮墙 | **有风险** | 我们墙高 0.40 m、近场回波 0.10~0.196 m ⇒ 带必须按我们传感器重算，照抄 COD_NAV 会丢矮墙 |
| `range_max 20` × cartographer 在线图 | **需观察** | 射程变远 ⇒ 图更大、单帧点数更多 ⇒ 关注 RTF 与 `num_range_data` 的相对关系 |

## 4. 新增槽位的参数骨架（标注来源，**待我们标定**）

```yaml
# nav:=mppi  （抄自 COD_NAV rmul2026，速度已按我们仿真下调为待定）
controller_frequency: 30.0        # 他们 50.0；我们先用 30
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Omni"
  time_steps: 60
  model_dt: 0.05
  batch_size: 2000
  vx_max: 2.0                     # 他们 7.5，待标定
  vy_max: 2.0
  wz_max: 2.5
  temperature: 0.25
  gamma: 0.008
  critics: ["ConstraintCritic","CostCritic","GoalCritic","PathFollowCritic","PathAlignCritic","ObstaclesCritic"]
  CostCritic: {critical_cost: 253.0, consider_footprint: true, weight: 3.0}
  GoalCritic: {weight: 15.0, threshold_to_consider: 2.5}
  PathFollowCritic: {weight: 5.0, threshold_to_consider: 1.5}
  PathAlignCritic: {weight: 10.0, threshold_to_consider: 1.5}
# planner:=smac2d（抄自 COD_NAV 2026；我们的地图小得多，耗时风险低）
GridBased: {plugin: "nav2_smac_planner/SmacPlanner2D", allow_unknown: true, tolerance: 0.5, cost_travel_multiplier: 4.0}
# smoother:=savgol
smoother_plugins: ["SmoothPath"]
SmoothPath: {plugin: "nav2_smoother::SavitzkyGolaySmoother", do_refinement: true, refinement_num: 2, enforce_path_inversion: true}
# local_obstacle:=stvl（先只加衰减这一条）
local_costmap: {plugins: [obstacle_layer, obstacle_cloud_layer, stvl_layer, inflation_layer]}
stvl_layer_local: {voxel_decay: 0.5, voxel_size: 0.05, obstacle_range: 8.0, raytrace_range: 9.0, min_obstacle_height: 0.1, max_obstacle_height: 1.0, model_type: 1}
# 车体裁剪盒（negative passthrough；数值须按我们车体重标）
crop_box: {min_x: -0.30, max_x: 0.30, min_y: -0.30, max_y: 0.50, min_z: -0.10, max_z: 0.20, negative: true, leaf_size: 0.05}
```

## 5. 明确**不做**的事

1. 不引入 ESDF / 动态障碍跟踪与预测（**他们也没做**，不是这次的收益点）；
2. 不采用他们 72 m 大面积 unknown 图与"静态 `map→odom`"作为默认；
3. 不学他们的工程卫生问题（硬编码绝对路径、提交 `cmake-build-debug/`/.swp、死配置块、`odom_topic` 拼错）；
4. **不照抄速度**（7.5 m/s 是实车参数，仿真里不可直接移植）；
5. 不在未过 P0 回归的情况下叠加两个以上变量。

## 6. 验收与记录方式

- 每阶段结束：`nav_smoke_regression.py`（PASS/FAIL + JSON 快照）→ `algorithm_matrix.md §四` 加一行 → 本文件对应阶段打勾；
- 链路体检：`tools/scripts/diag/watch_startup_chain.py`；
- 若出现新坑：写进 `debug_fastlio_cartographer.md §9.0 总表`（症状 → 判据 → 处置 → 状态）。

---

## 7. 关于"2026 用的是不是 FAST-LIO"——已核实（2026-09-26）

抓 `https://api.github.com/repos/qza36/COD_NAV/git/trees/rmul2026` 得到该分支**根目录**：

```
.github/  .gitignore  .idea/  CLAUDE.md  LICENSE  README.md
cpp_lidar_filter/          ← 车体裁剪盒（自带包）
fake_vel_transform/        ← 小陀螺/云台解耦（自带包）
nav_bringup/               ← launch/params/map/BT
pointcloud_to_laserscan/   ← 自带 fork
resource/
small_point_lio/           ← ★ LIO 就是它：**仓库内自带目录，不是 submodule**
```

**结论**：
1. **`rmul2026` 的 LIO = `small_point_lio`，分支里没有 `FAST_LIO/` 目录** ⇒ **2026 路线不用 FAST-LIO**（FAST-LIO 只留在 `master`/2025 那套）；
2. 该分支**没有 `.gitmodules`**（我抓 `.../rmul2026/.gitmodules` 得 404），而 `master` 有 4 个 submodule（patchwork-plusplus、pcd2pgm、pb_omni_pid_pursuit_controller、pb_nav2_plugins）⇒ **2026 是一次"做减法"的重构**：去掉地面分割（改高度带）、去掉先验图与全部重定位（纯在线 lifelong）、去掉 pb_omni（改 MPPI）、去掉 patchwork++/pcd2pgm/pb_nav2_plugins。**整个工作区只剩 5 个包**（上面 5 个目录）；
3. 他们 2026 的文档主要在 **`CLAUDE.md`（5.3 KB）**，`README.md` 只有 1.1 KB ⇒ 想看他们的设计意图，读 `CLAUDE.md`；
4. **我们的最短对齐路径**：我们工作区**已经有 Point-LIO**（`src/localization/rm_localization/point_lio`，槽位 `lio:=pointlio`）——`small_point_lio` 属同一族（名字即 "Small Point-LIO"），所以**先用 `lio:=pointlio` 做等效验证，零引入成本**；确有效益再考虑 vendor 他们的 `small_point_lio`（那是 P3，且要先看它的 license 与依赖）。
5. 顺带修正一条认知：**"2026 架构"不能整体照搬** —— 它同时**砍掉了重定位**（`map→odom` 静态）和**地面分割**，这两条在我们这里是资产（四种重定位可选、linefit 可测），**只借它做加法的部分（裁剪盒/STVL 双图/MPPI/Smac/SG 平滑）**。

---

## 8. 融入可行性分析（结论：**架构不用改，只加"候选"**）

判据来自我们架构的三条硬约束：① 槽位化（同一链路换算法做 A/B）；② 契约固定（TF 链、`robot_base_frame: base_link_fake`、仿真钟、cmd_vel 转换）；③ 先验图与在线建图两条路都要保留 + 仿真要能验收。

| COD_NAV 2026 组件 | 判定 | 关键理由 |
|---|---|---|
| `cpp_lidar_filter` 车体裁剪盒 | **✅ 直接融** | 纯 ROS 节点，进出都是 PointCloud2；**必须按我们的 `livox_frame` 重算 box**（他们的 marker 在 `base_link` 是 bug）；`leaf_size` 是死参 ⇒ 要降采样得自己加 |
| STVL 双图 + 衰减 | **✅ 直接融** | 包已装、纯 YAML；照抄 `decay_model 0`、`voxel_decay 0.5`、`voxel_size 0.05`、**`model_type: 1`**、`filter: voxel`、`obstacle/raytrace 8/9 m` |
| `range_max: 10 → 20` | **✅ 直接融** | 一行参数；我们地图 15×28 m 比他们 72 m 小，耗时/RTF 风险低 |
| 高度带 `min/max_height` | **⚠️ 必须重算，禁止照抄** | 他们的 `min_height 0.10/0.15`（livox_frame）**会砍掉我们近场回波**：实测近场 `z_map 0.107~0.196` ⇒ `z_livox ≈ −0.12~−0.03`，**全在 0.10 以下** ⇒ 照抄等于把近场墙全删（= 又制造"缝"） |
| Smac2D（`cost_travel_multiplier 4.0`、`tolerance 0.5`、`allow_unknown true`） | **✅ 直接融** | `nav2_smac_planner` 已装；注意 `motion_model_for_search`/`minimum_turning_radius`/`angle_quantization_bins` 是 **Hybrid 专用**，Smac2D 下是惰性键（别抄） |
| Smac 的 `smooth_path: true` + 内建 smoother | **✅ 直接融** | 这才是他们真正的平滑（`smoother_server` 在他们链里是**死代码**，BT 无 `<SmoothPath>`）⇒ 比接 savgol 便宜 |
| MPPI Omni + 他们的调参史 | **✅ 融（成本中等）** | 包已装；需新 params + `nav:=mppi` 槽 + velocity_smoother 匹配。**速度必须降**（7.5 m/s 是实车）；**与自转模式互斥**（见下） |
| BT 结构（3 Hz 重规划 / BackUp） | **△ 要自己写** | 他们 XML 里**没有 `IsStuck`**，`BackUp` 只在 through-poses 且是 **1.0 m @ 1.0 m/s** ⇒ 借结构得自己加节点，收益一般 |
| `small_point_lio` | **❌ 不能直接融** | 其 PointCloud2 适配**要求 `tag`(uint8)+`timestamp`(float64 秒)** 字段（我们点云没有）；`twist` 全零、帧硬编码。**替代：用我们已有的 `lio:=pointlio`（同族）先验证收益** |
| `map_resolution 0.5` / PCD 写编译期源码目录 | **❌ 不该融** | 写死路径 + 往源码目录写文件，不适合可复现的 bench |
| slam_toolbox `lifelong`（纯在线） | **△ 可作为 `mapper` 新取值** | 但必须**让它发 `map→odom`**（他们设 `transform_publish_period 0.0` = 不发 ⇒ 才需要静态桥）；与重定位槽**互斥** |
| 静态 `map→odom`(z=0.05) + 零重定位 | **❌ 不融（与核心资产冲突）** | 我们 `mode:=nav` 的意义就是"先验图 + 重定位"；静态桥要求机器人**从图原点原朝向起飞**（他们图仅 10.65×10.2 m）。最多作为 slam_nav 调试的临时开关 |
| 去掉 `linefit`（改高度带） | **❌ 不该融** | linefit 是我们可测/可 A/B 的资产（z-profile 诊断就靠它），且他们的高度带会砍掉我们近场回波 |
| `enable_rotation: false` + `yaw_goal_tolerance 6.28` | **△ 作为我们自己的模式** | **注意**：他们**从不自转**（`spin_speed` 全分支无一处非零）⇒ "他们为自转而这样做"是错的推断；真实作用只是"目标朝向随便"。`sentry_spin` 是我们的设计 |
| 未知区 / 动态障碍 / ESDF / 前沿探索 | **—（无内容可融）** | 他们全分支 grep 0 命中；得我们自己定策略 |

### 融入的三个前置冲突（必须先解决，否则会重演"指令被静默替换"）

1. **`wz` 归属冲突**：MPPI（或任何控朝向的控制器）要控 `wz` ⇒ 必须 `spin_speed=0`；要自转 ⇒ 必须放弃控制器控朝向。**launch 层必须互斥**，不能两个都开。
2. **`map→odom` 双发布者**：`localization:=*`（四种）与 `mapper:=slam_toolbox_lifelong` 都会发 ⇒ `mapper` 新取值必须加进 launch 的互斥条件。
3. **高度带标定**：`min_height`/`max_height` 必须按我们传感器的**实测 z 剖面**重算（近场 `z_livox < 0.10`），否则融入即制造新"缝"。

### 一句话结论

**能融，但"融入"的正确形状不是搬他们的架构，而是把他们 6~8 个组件作为我们槽位的"新取值/新参数集"加进来**：默认路径（rpp + cartographer + amcl + linefit）一个字不动，随时可 A/B 与回退。**不能融的三样**（静态 `map→odom` 零重定位、去掉 linefit、直接 vendor `small_point_lio`）都触及我们的核心资产或硬门槛。


---


## 来源 12/19：`tdt/cod_nav_2026_migration_worksheet.md`


> 依据：`docs/cod_nav_2026_params_review.md`（773 行、34 表、逐条 file:line）、`docs/cod_nav_2026_deep_dive.md`、`docs/cod_nav_2026_integration_plan.md`
> 参考代码已就位：`third_party/cod_nav_2026/`（163 文件，已在 `.gitignore`，`third_party/COLCON_IGNORE` 保证不参与构建）
> 记号：S=`singlenav2_params.yaml`，M=`multiplenav2_params.yaml`，GAC=`goal_approach_controller`，FVT=`fake_vel_transform`
> **状态**：全部 `TODO`。每条都要"一次一个变量 + P0 回归验收 + 可单独回退"。

## A. 可直接用（8 条中的 6 条已确认，余 2 条见 §E）

| # | 抄什么 | 来源 | 落到我们哪里 | 风险 | 回退 | 状态 |
|---|---|---|---|---|---|---|
| 1 | **`PositionGoalChecker`**（只看位置：xy + path_length，不看朝向） | S goal_checker 段 | `params/nav2_params_sim_*.yaml` → `controller_server.goal_checker_plugins` + 新块 | 低（1.1.20 确认有此插件）；与我们 `spin_speed` 模式天然匹配 | 换回 `goal_checker`（xy+yaw） | TODO |
| 2 | **Smac2D 的 9 个有效键** + `smoother.*` 子块（**跳过 16 个 Hybrid/Lattice 无效键**） | S planner_server / N2-smac2 | 新增 `planner:=smac2d` 槽对应的 yaml 块 | 低-中：A* 比 NavFn 慢（我们图 15×28 m，风险小） | 回到 NavFn | TODO |
| 3 | **`velocity_smoother` 键名 + `cmd_vel` remap 链** | S（与我们逐字相同） | 对照现有 velocity_smoother 段 | 低 | 逐键回退 | TODO |
| 4 | **`map_saver` 段** | S（逐字相同） | 我们的建图保存链 | 低 | 无 | TODO |
| 5 | **STVL 体素层参数块**（`decay_model 0`/`voxel_decay 0.5`/`voxel_size 0.05`/`model_type 1`/`filter voxel`/`obstacle 8.0`/`raytrace 9.0`）—— **别抄 `livox_filtered`、realsense 源** | S stvl 段 | `local_obstacle:=stvl`（local 也上 STVL） | 中：清除过激会擦薄结构 | 移除 local stvl 层 | TODO |
| 6 | **航点 CSV 格式 + 两个桥节点**：`waypoint_to_nav2`(FollowWaypoints 逐点停) / **`waypoint_through_nav2`(NavigateThroughPoses 连续穿越)** | `wps/*.csv` + `waypoint_editor/` | 长目标分段（路线 B）—— **我们的默认 BT 已支持 through-poses** | 中低；坑：`waypoint_pause_duration` 单位是**毫秒**；`stop_on_failure: false` 会**静默跳过失败点** | 停用桥节点，回到 `segment_goal_navigator.py` | TODO |

> 余 2 条以 `docs/cod_nav_2026_params_review.md §E`（"可直接用 8 条"）为准。

## B. 需改造（12 条里的关键 5 条）

| # | 项 | 必须先改什么 | 风险 | 状态 |
|---|---|---|---|---|
| 1 | **MPPI（`nav:=mppi`）** | ① `model_dt` = 1/`controller_frequency`；② `consider_footprint: false`（或先加 footprint）；③ `min_y_velocity_threshold → 0.001`；速度从 2.0 m/s 起 | 中：CPU + 调参 | TODO |
| 2 | **`goal_approach_controller`** | 修**帧 bug**（`goal_`(map) 与 `pose`(odom) 混帧相减）；加 **spin 开关**（否则末端 2 m 内静默关掉小陀螺） | 中高 | TODO |
| 3 | 裁剪盒 `cpp_lidar_filter` | 按**我们的 `livox_frame`** 重算 box（他们的 marker 在 `base_link` 是 bug；`leaf_size` 是死参） | 中 | TODO |
| 4 | 高度带 `min/max_height` | **按我们实测 z 剖面重算**（近场 `z_livox < 0.10`，照抄他们的 0.10/0.15 会砍掉近场墙 ⇒ 又制造"缝"） | 中 | TODO |
| 5 | `pb_nav2_behaviors/BackUpFreeSpace` | 需额外编译 `pb_nav2_plugins` | 低-中 | TODO |

## C. 不适用（明确"不跟"）

1. **`rolling_window: true` + 25×25 窗 + `allow_unknown: true`**（图外未知区可规划）—— 与我们"未知=不可走"的安全几何**方向相反**；
2. **他们的 BT**（删了 `Spin`/`Wait`，因为他们的 FVT 让 Spin 失效）—— **我们的 FVT 有修复，`Spin` 可用，不能照抄**；
3. **静态 `map→odom` + 零重定位** —— 撞我们 `mode:=nav` 的核心资产；
4. **直接 vendor `small_point_lio`** —— 需 `tag`+`timestamp` 字段、twist 全零；
5. `map_resolution 0.5` / 硬编码 `/home/cod-sentry/...` 路径 / `auto_save_map.launch.py`；
6. 各类 Jazzy/main 专有键（见 §10.4 第 4 条）。

## D. 落地顺序（建议）

```
① PositionGoalChecker（最低风险，先验"只控位置"是否合我们）
② Smac2D 有效键 + smoother 子块
③ waypoint_through_nav2 承载分段（替代自研分段循环）
④ MPPI（先做三处硬约束改造）
⑤ goal_approach_controller（先修帧 bug + spin 开关）
── 每步：nav_smoke_regression.py 验收 → algorithm_matrix.md §四 记一行 → 本表状态改 DONE/REVERTED
```

---

## E. 决策记录（2026-09-26，用户确认）

### E1. 静态 `map→odom`：**不采用**（明确不做）
理由：撞我们 `mode:=nav` 的核心资产（先验图 + 4 种重定位）；作者原话也承认它只在「环境简单、面积小、误差容忍度大」时成立。仅保留"纯在线建图无重定位"这一档（`mode:=nav + localization:=''`，我们本来就有）作为**调试对照**，不作为方案。

### E2. `linefit` 地面分割：**保留**（不抄 COD 的"去掉"）
COD 2026 去掉 patchwork++/地面分割的原因（有据）：① 2026 改由**两层 STVL 直接吃 3D 点云**，地面点由 STVL 的 `min_obstacle_height: 0.1` + p2l 的高度带一起挡掉，不再需要专门的地面分割器；② 做减法/减依赖（他们 2026 主线是"提速"）；③ 固定安装高度 + 场地基本水平 ⇒ 固定高度带比在线地面拟合更可预测。
**我们不该抄的原因**：① 实测我们**近场回波 `z_livox ≈ −0.12~−0.03`，全在 0.10 以下** ⇒ 照抄他们的 `min_height 0.15/0.10` 会把近场墙**整段砍掉**（= 又制造"缝"）；② linefit 是我们**可测/可 A/B 的资产**（`cloud_z_profile.py`、`costmap_marking_check.py` 都建立在它输出的 `/segmentation/obstacle` 上）；③ 场地若有坡道/台阶，高度带假设直接失效。

### E3. `small_point_lio`：**要，且提升为 `lio` 槽位的同级取值**
目标形态：`lio: = fastlio | pointlio | **smallpointlio** | none | cartographer`（与 fastlio/pointlio 同级，可 A/B）。
落地清单（P3）：
1. 代码位置：`src/localization/rm_localization/small_point_lio`（上游 <https://github.com/Yancey2023/small_point_lio>，MIT，分支 `ros2`；**不要放 `third_party/`**，那里有 `COLCON_IGNORE` 不会被构建）；
2. TF 契约：它直接发 `odom→base_link` ⇒ 与 fastlio 的 `camera_init→body` + `lio_tf_adapter` 路径不同，launch 里必须**按槽位切换**（选 smallpointlio 时不经 adapter）；
3. **硬门槛（必须先解决）**：其 PointCloud2 适配器要求点云带 `tag`(uint8) 与 `timestamp`(float64, 秒) 字段（`src/lidar_adapter/livox_pointcloud2.h`，只保留 `(tag & 0b00111111)==0`），并硬编码 `frame_id/child_frame_id = odom/base_link`；我们的仿真点云没有这两个字段 ⇒ 需要写一个**适配器节点**补 `tag=0` 与**逐点 timestamp**（逐点时间正是 Point-LIO 系的价值所在，必须从我们雷达插件的扫描时序/CSV 图案里合成，不能简单全填同一时间）；
4. 配置：`mid360_sim.yaml`（`lidar_type: livox_pointcloud2`、`map_resolution` 与 `save_pcd` 要改掉他们的 0.5/编译期写死路径）；
5. 已知短板：仓库里 `twist` 六行**仍是注释**（无速度输出）；`save_pcd` 写到 `@CMAKE_CURRENT_SOURCE_DIR@`；
6. 验收判据：`/Odometry`（或等价）10 Hz 出数 + `odom→base_link` TF 连续 + `lio:=smallpointlio` 下 **P0 回归 PASS** + 与 fastlio/pointlio 的**漂移对比**（同一条轨迹的 end-to-end 误差）。

---

## F. 候选实验记录：**两层 STVL**（全局 + 局部都上 `spatio_temporal_voxel_layer`）　【用户标记：后面可以尝试】

> 来源：COD_NAV 2026（`singlenav2_params.yaml` / `multiplenav2_params.yaml`，两张 costmap 的 `plugins` 里都有 STVL）
> 状态：**候选（TODO，未实施）**。与 §A-5（STVL 参数块）、§B-3（裁剪盒）、§E2（linefit 保留）互为引用。

### F1. 他们的做法（参数）
`voxel_decay 0.5`（线性衰减，`decay_model 0`）· `voxel_size 0.05` · `observation_persistence 0.0` · `combination_method 1` · `track_unknown_space true` · **`model_type: 1`（3D 雷达）** · `vertical_fov_angle 2.00` / `horizontal_fov_angle 6.28` · `filter: "voxel"` · `transform_tolerance 0.2`；**层级 `obstacle_range 3.0` 被源级 `8.0/9.0`（obstacle/raytrace）覆盖**；`min_obstacle_height 0.1`、`max_obstacle_height 1.0`。

### F2. 两层 STVL「导致」了什么（连带效应 —— 这才是值得记的部分）
1. **给局部加了时间维**：障碍体素 **~0.5 s 自动消失** ⇒ 局部不再累积"幽灵障碍"。我们现在 local **没有时间维**（只靠 `obstacle_layer` 的射线清除），这是与他们的实质差距。
2. **局部也能吃原生 3D 点云**：STVL 是体素层 ⇒ 局部不必只依赖 p2l 的 2D 投影（对应我们 `local_obstacle:=cloud` 那条路的"升级版"）。
3. **这是他们能删掉地面分割的前提之一**：地面点由 STVL 的 `min_obstacle_height` + p2l 高度带共同挡掉 ⇒ 少一个独立模块（与 §E2 相连）。
4. **必须与"车体裁剪盒"配套**：**STVL 没有"最小障碍半径"概念**（他们 `box_lidar_filter` README 原话）⇒ 自身点云只能先从点云里抠掉。**「两层 STVL + 裁剪盒」是一套，不能只上其一**，否则车身自身点会进体素图。
5. **代价**：两张图都做体素化+射线清除 ⇒ CPU 上升；清除过激会擦薄结构；**与 static/obstacle 层的写序耦合** —— 我们已经见过"缝"就是后写层的清除覆盖了 static 层写下的墙（见 `debug_fastlio_cartographer.md`），两层 STVL 会**增加一个清除来源**。

### F3. 我们的现状与尝试方案（一次一个变量）
- 现状：**global 已有 STVL**（`min_obstacle_height 0.0`、`voxel_decay 0.5`）；**local 没有**，是 `[obstacle_layer, obstacle_cloud_layer, inflation_layer]`；`stvl_layer.min_obstacle_height` 全局已为 **0.0**（对应我们近场 `z_livox < 0.1` 的实测，**这正是不能照抄他们 0.1 的地方**）。
- 方案（顺序不可颠倒）：**① 先上裁剪盒**（§B-3，按我们 `livox_frame` 重算 box）→ **② 再给 local 加 `stvl_layer`**（保守：`obstacle_range 8.0` / `raytrace_range 9.0` / `voxel_decay 0.5` / `voxel_size 0.05` / **`min_obstacle_height 0.0`（不是 0.1）** / `max_obstacle_height 1.0` / `model_type 1`），图层顺序建议 **`stvl` 放在 `obstacle*` 之后、`inflation` 之前**，并观察"缝"是否恶化。
- 判据：**P0 回归 PASS**（能走到目标）· 幽灵障碍消失时间（人放一个临时障碍后 ~0.5 s 内清掉）· 墙面**没有变薄/裂缝变宽** · CPU/RTF 无明显下降。
- 回退：从 local `plugins` **移除一行**即恢复；或用 `local_obstacle:=cloud`（保留点云、不加时间维）。
- 关联风险：若"缝"恶化 ⇒ 说明多了一个清除来源 ⇒ 转去查 `inf_is_valid`（他们 `obstacle_layer.inf_is_valid: true` 与我们一致）与图层顺序。

---

## G. 关于 `small_point_lio`（§E3）的排期约定：**可以后面补，且不阻塞 D3/D4**

**为什么可以延后**：`lio` 是**独立轴** —— Nav2 那一侧只依赖三个契约：`/odom`、`odom→base_link`、`/scan`（由 linefit+p2l 从 `/livox/*` 产出，与该轴无关）。换/加 LIO 是**换生产者**，不动消费者 ⇒ 只要槽位契约不变，随时可加。

**排期约定（用户确认口径）**：
1. **D3 / D4 一律用已验证的 `lio:=fastlio` 验收**（保持"一次一个变量"，让 D3/D4 的结论干净）；
2. `lio:=smallpointlio` 落地后，**D3/D4 的验收项复跑一遍**（里程计变了，控制器/规划器的最优参数可能要微调），并做**同轨迹端到端漂移对比**（fastlio vs pointlio vs smallpointlio）；
3. **不阻塞**：三者互不依赖；只有一种情况需要提前上它 —— **D3/D4 的失败模式被怀疑来自里程计**（判据：`/odom` 话题频率正常但位姿漂移大；或 `spin_speed != 0` 自转时定位抖动明显）。

**"风险高"的三条，其实都是可控/可延后的工程量，不是架构风险**：
| 风险 | 性质 | 低成本先行做法 |
|---|---|---|
| 逐点 `timestamp` 需合成 | **工作量**（其价值所在，必须做对） | 先做**退化版**（扫描内按 CSV 图案顺序/均匀插值合成逐点时间）跑通链路，再逐步精确 |
| TF 契约切换（它直发 `odom→base_link`，fastlio 走 `camera_init→body` + adapter） | **launch 层一次性条件** | 按槽位切通路，写好即不再动 |
| 仓库短板（`twist` 全零、写死 `save_pcd` 路径、`map_resolution 0.5`） | **配置/patch 层** | 在我们自己的 `mid360_sim.yaml` 里全部覆盖 |

**零成本预研动作（随时可做，不动主链路）**：把 `small_point_lio` 当**独立进程**跑起来（不接 Nav2、不改 `lio` 槽位），只验证它能从我们的仿真点云产出 `/Odometry` + `odom→base_link` TF —— 即"**喂得进去**"。这一步就能把上面三条门槛里最难的那条（逐点时间）**提前证伪或证实**。

---

## H. D3-① 实施记录：`planner:=smac2d` 槽位（2026-09-26，已提交）

**提交**：`d4e662b`（master 已推送）。**改动 2 个文件**：`src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py`（+18/−1）、新增 `src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_rpp_smac2d.yaml`（435 行）。
**默认未变**：`planner` 默认 `navfn` ⇒ 解析出的 params 路径与改动前**完全相同**（用 `importlib` 加载 launch、对六种 `nav × planner` 组合实测过）⇒ **现有已验证路径零影响**。

**新 yaml 的单变量保证**：与 `nav2_params_sim_rpp.yaml` 的 `diff -u` **只有 1 个 hunk**（整段落在 `planner_server` 内）。
**写入的 12 个键**（每个都在**已安装的 1.1.20** 里核实过）：
`tolerance 0.5` · `allow_unknown true` · `downsample_costmap false` · `downsampling_factor 1` · `max_iterations 1000000` · `max_on_approach_iterations 1000` · `max_planning_time 5.0` · **`cost_travel_multiplier 4.0`** · `use_final_approach_orientation false` · `smoother.{w_smooth 0.4, w_data 0.1, do_refinement true}`。
**核实方法**（三重）：① `strings libnav2_smac_planner_2d.so`（插件用 `name + ".<key>"` 声明参数）；② 已装头文件 `types.hpp` 的 `SmootherParams::get()`；③ 交叉核对 `third_party/nav2/nav2_smac_planner/src/smac_planner_2d.cpp:64-92`（同版本 1.1.20）。
**按规则剔除**：`refinement_num`（1.1.20 **不存在** ⇒ 写了就是静默失效，这正是我们防的那类错）· `use_astar`（NavFn 专有）· `smooth_path` 与 16 个 Hybrid/Lattice/Jazzy 专有键（2D 插件里 strings 命中 0）。
（另注：`smoother.max_iterations`(默认 1000) 与 `smoother.tolerance`(默认 1e-10) 在 1.1.20 确实存在，本次未写；要复现 COD 的 10000 再补。）

### ⚠️ 新踩到的坑（值得记进 §10.4 通用规律）
**新增 params 文件后必须先重新 build**：`install/` 里**不会**因为 `--symlink-install` 就自动出现新文件（已核对 `install/rm_navigation/share/rm_navigation/params/` 里没有 smac2d 文件）⇒ 必须先
`colcon build --symlink-install --packages-select rm_navigation`
否则 launch 会因为找不到 params 文件而失败（表现为"改了配置没生效/启动报错"，极易误判为插件问题）。

### 待验收（未实测项）
实跑尚未进行 ⇒ **"Smac2D 插件运行时真的加载成功 + 路径形态变好"两点未经实测**（离线证据仅到 pluginlib 声明 + ament resource index 存在）。
验收命令：见下方 §H.1；**回退**：省略 `planner` 或 `planner:=navfn`（一行）。

### H.1 验收清单（用户执行）
```bash
colcon build --symlink-install --packages-select rm_navigation      # ← 新增文件必须先 build
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=amcl nav:=rpp planner:=smac2d spin_speed:=0.0 nav_rviz:=True
ros2 param get /planner_server GridBased.plugin               # 期望 nav2_smac_planner/SmacPlanner2D
ros2 param get /planner_server GridBased.cost_travel_multiplier   # 期望 4.0
python3 tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5
```
判据：① 两个 `param get` 正确；② 启动日志出现 `Configuring GridBased of type SmacPlanner2D ... tolerance 0.50, maximum iterations 1000000`；③ **P0 回归 PASS**；④ RViz 里 `/plan` **不再贴缝走、更居中**；⑤ 规划耗时可接受（用 `navigate_to_pose` 结果里的 `planning_time` 或 smoke 工具"用时"与 NavFn 对照）。


---

## I. 卡住恢复链核查（2026-09-26；**只读核查 + 用户执行步骤，本步不改任何代码**）

> 起因：车可能**开进** inflation/内切带后卡死（planner 把起点当障碍、controller 报无效控制）。Step 1 已把
> `robot_radius` 从 0.40 改回 0.22（commit `dc03e11`）。本节核查"改回之后，恢复链是否真的会救场"。
> 引用行号：params = `src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_rpp.yaml`（**step 1 之后**的行号）；
> `N2/xxx` = `third_party/nav2/nav2_mppi_controller` 等**同版本 1.1.20** 源码。

### I.1 我们实际加载的是哪棵 BT

- `bt_navigator` 段（params:61-100）**没有** `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml`
  （全 params 文件 grep `bt_xml` 命中 0；launch 侧 `RewrittenYaml` 只重写 `use_sim_time` / `yaml_filename`，
  不注入 BT）⇒ **走 nav2 内置默认**。默认值的产生处：`N2 bt_navigator/src/navigators/navigate_to_pose.cpp:58-79`
  与 `navigate_through_poses.cpp:52-73`（`has_parameter(...)` 为假时 `declare_parameter` 成
  `get_package_share_directory("nav2_bt_navigator") + "/behavior_trees/<名字>.xml"`；已装
  `/opt/ros/humble/lib/libbt_navigator_core.so` 里的字符串正是这两个文件名）。
- **加载的精确路径**：
  - `NavigateToPose` → `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
  - `NavigateThroughPoses` → `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`
- 我们自己的两个 goal 发送端都**不填** `NavigateToPose.Goal.behavior_tree`
  （`tools/scripts/regress/nav_smoke_regression.py:235`、`tools/scripts/nav/segment_goal_navigator.py:134`）
  ⇒ 一定落到上面这棵默认树（运行期铁证见 I.4 的 `ros2 param get /bt_navigator default_nav_to_pose_bt_xml`）。

**内置树里有没有恢复元素**（两个文件逐字读完）：

| 元素 | 在不在 | 参数 / 重试次数 |
|---|---|---|
| `RecoveryNode`（外层 `NavigateRecovery`） | ✅ | **`number_of_retries="6"`** |
| `RecoveryNode`（`ComputePathToPose`） | ✅ | **1**（失败 → `ClearEntireCostmap` 全局） |
| `RecoveryNode`（`FollowPath`） | ✅ | **1**（失败 → `ClearEntireCostmap` 局部） |
| `RateController` | ✅ | `hz="1.0"`（to_pose）/ `hz="0.333"`（through_poses） |
| `ClearEntireCostmap` | ✅ | 4 处（2 处上下文恢复 + `ClearingActions` 里 local/global 各一） |
| `Spin` | ✅ | `spin_dist="1.57"`；`time_allowance` 默认 10.0（`N2 nav2_behavior_tree/plugins/action/spin_action.hpp:57`） |
| `Wait` | ✅ | `wait_duration="5"` |
| `BackUp` | ✅ | `backup_dist="0.30"` / `backup_speed="0.05"`；`time_allowance` 默认 10.0（`back_up_action.hpp:56-58`） |
| 恢复顺序（`RoundRobin`） | ✅ | `Sequence(ClearLocal,ClearGlobal)` → `Spin` → `Wait` → `BackUp`（`GoalUpdated` 在前，目标没变才做恢复） |

- `plugin_lib_names`（params:69-100）与树所需节点逐一对照：树用到的 13 个 BT 节点库**全部**在列表里，且
  `/opt/ros/humble/lib/libnav2_*_bt_node.so` 同名文件全存在（含 `nav2_recovery_node_bt_node` /
  `nav2_round_robin_node_bt_node` / `nav2_rate_controller_bt_node` / `nav2_clear_costmap_service_bt_node` /
  `nav2_back_up_action_bt_node` / `nav2_spin_action_bt_node` / `nav2_wait_action_bt_node`）
  ⇒ 树能被完整实例化，**不会**出现"节点未注册"的加载失败。

### I.2 `behavior_server` 配了没有 / 1.1.20 有哪些恢复插件

- **配了**（params:368-392，dwb/teb 同）：
  `costmap_topic: local_costmap/costmap_raw` · `footprint_topic: local_costmap/published_footprint` ·
  `cycle_frequency: 10.0` · `behavior_plugins: ["spin","backup","drive_on_heading","wait"]`
  （类型 `nav2_behaviors/Spin|BackUp|DriveOnHeading|Wait`）· `global_frame: odom` ·
  `robot_base_frame: base_link_fake` · `transform_tolerance: 0.1` · `simulate_ahead_time: 1.0` ·
  `max_rotational_vel: 3.0` · `min_rotational_vel: 0.4` · `rotational_acc_lim: 3.0`。
- 启动链：`bringup_sim.launch.py` → `rm_navigation/launch/bringup_rm_navigation.py` →
  `rm_navigation/launch/navigation_launch.py`；`behavior_server` 在 `lifecycle_nodes`（`navigation_launch.py:46`），
  非组合（`:182`）与组合（`:257` `behavior_server::BehaviorServer`）两条路都起 ⇒ **节点确实被拉起来**
  （Humble 节点名是 `behavior_server`，不是 Galactic 的 `recoveries_server`，params 里已注明）。
- **键名核对（对已装 1.1.20）**：`costmap_topic` / `footprint_topic` / `cycle_frequency` / `behavior_plugins` /
  `global_frame` / `robot_base_frame` / `transform_tolerance` 由 `N2 nav2_behaviors/src/behavior_server.cpp:34-57`
  + `include/nav2_behaviors/timed_behavior.hpp:119-122` 声明；`simulate_ahead_time` 由
  `plugins/drive_on_heading.hpp:235`（BackUp/DriveOnHeading）与 `plugins/spin.cpp:53-56`（Spin）声明；
  `max_rotational_vel` / `min_rotational_vel` / `rotational_acc_lim` 只被 Spin 读（`spin.cpp:58-71`）。
  **`local_frame` 在 1.1.20 不存在**（`strings libbehavior_server_core.so libnav2_*_behavior.so | grep -cx local_frame` = 0；
  `local_costmap_topic` / `global_costmap_topic` / `local_footprint_topic` 同为 0）⇒ 那是 Jazzy 键，我们没写是对的。
- 已装恢复插件（`/opt/ros/humble/share/nav2_behaviors/behavior_plugin.xml` +
  `/opt/ros/humble/lib/libnav2_*_behavior.so`）：**`Spin` ✅ · `BackUp` ✅ · `DriveOnHeading` ✅ · `Wait` ✅ ·
  `AssistedTeleop` ✅**（五个都在）。我们只加载前四个；`assisted_teleop` 未配（见 I.5-5）。
  动作名 = 插件 id ⇒ `/spin`、`/backup`、`/drive_on_heading`、`/wait`。

### I.3 恢复链能转的两个前提（一个已满足，一个是 Step 1 给的）

1. **`Spin`/`BackUp` 会被"当前位置已在碰撞态"一票否决**：两者每周期都做前向 `isCollisionFree()`
   （`N2 nav2_behaviors/plugins/spin.cpp:149-153`、`include/nav2_behaviors/plugins/drive_on_heading.hpp:165-169`），
   不通过就 `stopRobot()` + `RCLCPP_WARN("Collision Ahead - Exiting Spin / DriveOnHeading")` + 返回
   **`Status::FAILED`**。⇒ `robot_radius: 0.40` 时车一进带，`local_costmap/costmap_raw` 就把车自己标成内切
   ⇒ 每个恢复动作都立刻 FAILED（BT 日志上"一直在恢复"，车一动不动）——这就是那个死锁的第二个根。
   **Step 1 把半径改回 0.22 正是让这个前提成立**：车贴墙 ~0.25 m 时仍判无碰撞，恢复才有机会成功。
   两者是配套的：以后再放大 `robot_radius` 会**重新关掉**这条链。
2. **FVT 不会吃掉恢复的角速度**（与参考队的关键差异）：`fake_vel_transform.cpp:80-84` 有
   `spin_speed_ == 0.0` 的**直通分支**（角速度原样下发、且不做 TF 查询）；参考队的 FVT 无论 `spin_speed` 取值
   都把 `angular.z` 换成 `spin_speed`（其 launch 不传参 ⇒ 0.0）⇒ 他们的 `Spin` 转不动、只能从树里删掉
   （§C-2）。**我们 `spin_speed:=0.0` 时 `Spin` 与一切角速度指令有效 ⇒ 内置树的恢复可用，不能照抄他们的树。**
   ⚠️ 但 `spin_speed:=5.0`（默认）时 `fake_vel_transform.cpp:94` 仍会把 `angular.z` 替换成常量 5.0 rad/s：
   底盘确实转，但**不是** Spin 行为闭环控制的那 1.57 rad ⇒ **验证恢复链请用 `spin_speed:=0.0`**（也正好是 §H.1 口径）。

**恢复指令从哪出去**（看错话题会以为"没有倒车速度"）：`navigation_launch.py` 里 `behavior_server` **只 remap 了 TF**，
它直接发 **`/cmd_vel`**、**绕过 `velocity_smoother`**（smoother 是 `cmd_vel→cmd_vel_nav` 进、`cmd_vel_smoothed→cmd_vel` 出，
`:219-220` / `:278-279`），再进 FVT（`/cmd_vel` → `/cmd_vel_chassis`）。
⇒ 看倒车速度要看 **`/cmd_vel` 或 `/cmd_vel_chassis`**；`/cmd_vel_nav` 只是 controller 的输出口，**看不到 BackUp**。

### I.4 运行时验证步骤（用户执行；全程只读，不改文件）

```bash
# 0) step 1 改的是既有文件（--symlink-install 已生效）；若还没 build 过 step 3 的新文件，见 §H.1
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=amcl nav:=rpp spin_speed:=0.0 nav_rviz:=True
```
```bash
# 1) 先确认链"接上了"（3 条）
ros2 action list | grep -E "backup|spin|wait|drive_on_heading"   # 期望 /backup /spin /wait /drive_on_heading
ros2 param get /behavior_server behavior_plugins                  # 期望 ["spin","backup","drive_on_heading","wait"]
ros2 param get /bt_navigator default_nav_to_pose_bt_xml           # 期望 .../behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
```
```bash
# 2) 开观察窗口（另开终端）
ros2 topic echo /behavior_tree_log     # BT 每个节点的状态迁移 → 恢复链最直接的证据
ros2 topic echo /cmd_vel               # controller(经 smoother) 与恢复动作的实际下发
ros2 topic echo /cmd_vel_chassis       # FVT 之后真正给底盘的量
```
```bash
# 3) 造"路径被堵"（要 FollowPath 失败，不是 planner 失败）：车正前方 ~0.6 m 放一个临时障碍
#    Gazebo GUI：Insert → Box，放在 world ≈ (4.9, 3.35, 0.3)（RMUL2026 出生点 world(4.3,3.35)，yaw=0）
#    或命令行：
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: tmp_obstacle, xml: '<sdf version=\"1.7\"><model name=\"tmp_obstacle\"><static>true</static><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.5 0.5 0.6</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.5 0.5 0.6</size></box></geometry></visual></link></model></sdf>', initial_pose: {position: {x: 4.9, y: 3.35, z: 0.3}, orientation: {w: 1.0}}, reference_frame: world}"
#    用完删掉：
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: tmp_obstacle}"
#    然后 RViz 用 2D Goal Pose 让车继续朝那个方向走（别直接发到障碍里，否则失败的是 planner）
```
拿一个**死胡同/墙角**做同样的事也可以（RViz 发一个"必须转身才能出去"的目标，车会贴进去再被 progress checker 抓住）。

> ⚠️ 别指望**第一次**恢复就看到 `Spin`：`RoundRobin` 从下标 0 开始，而 `ClearingActions`（清两张 costmap）几乎必然 SUCCESS
> ⇒ 第 1 次恢复 = 只清图并重试主树；**第 2 次**才是 `Spin`，第 3 次 `Wait`，第 4 次 `BackUp`（然后回绕）。
> 即"清图救不了 → 才转/等/退"。所以障碍要留着别撤，让它连着失败几轮。

| 顺序 | 看什么 | 期望 | 在哪看 |
|---|---|---|---|
| ① | controller 报错 | `RegulatedPurePursuitController detected collision ahead!`（RPP，`N2 nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp:359`）或 **`Failed to make progress`**（progress checker 超时：`required_movement_radius 0.5 m` / `movement_time_allowance 10.0 s`，`N2 nav2_controller/src/controller_server.cpp:475`） | launch 终端 |
| ② | BT 进恢复 | `NavigateRecovery` 连续失败 → `RecoveryFallback`；随后 `RateController` 仍按 1 Hz 重规划 | `/behavior_tree_log` |
| ③ | 恢复动作依次执行 | `ClearLocalCostmap-Subtree` / `ClearGlobalCostmap-Subtree` → `Spin` → `Wait` → `BackUp`（`RoundRobin` 顺序：当前子节点 SUCCESS 即返回，FAILURE 才顺延到下一个，`N2 nav2_behavior_tree/plugins/control/round_robin_node.cpp:40-79`） | 同上 + launch 终端 |
| ④ | 动作自身日志 | `Running spin` / `spin completed successfully` / `Running backup` / `backup completed successfully`（`N2 nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp:187/239`） | launch 终端 |
| ⑤ | 倒车速度 | `/cmd_vel.linear.x ≈ -0.05`（`BackUp backup_dist 0.30 / backup_speed 0.05`；`time_allowance` 默认 10 s ⇒ 6 s 走完 0.30 m，余量够） | `/cmd_vel` 或 `/cmd_vel_chassis` |
| ⑥ | 车真的退了 | 里程计位置后退 **≥ 0.2 m**，随后 `/plan` 变化、`ComputePathToPose` 再次 SUCCESS、车继续走 | RViz + `/behavior_tree_log` |

**判据**：BT 日志里出现过 `BackUp` + `/cmd_vel.linear.x` 出现负值 + 车实际后退 ≥0.2 m + 之后重新规划成功 ⇒ 恢复链成立。
（默认 `global_obstacle:=stvl local_obstacle:=scan` 即可；`local_obstacle:=cloud` 时 `/segmentation/obstacle` 也能看到这个盒子。）

### I.5 发现的缺口（只登记，本次不修）

1. **恢复动作会被"自身已在碰撞态"一票否决**（I.3-1）⇒ 恢复链**不是**卡死的万能兜底，它只在"costmap 认为当前位置无碰撞"时有效。与 Step 1 配套；任何再次放大 `robot_radius` 的改动会重新关掉它。
2. **`bt_navigator.odom_topic: /Odometry` 是死订阅**：fastlio 的发布话题已被 remap 成 `/odom`（`bringup_sim.launch.py:271-272`）⇒ `/Odometry` 无发布者，`OdomSmoother` 恒 0。内置树里**没有**节点读 `{odom_smoother}`（两棵 XML 全文无该 port）⇒ **对恢复链无影响**，属清理项（§A.10 已记）。
3. **`BackUp 0.30 m` 与"贴墙"是紧张关系**：车尾 0.30 m 内有障碍/膨胀代价时 BackUp 会提前 `Collision Ahead` 失败（RoundRobin 会顺延，不会死锁，但这一轮恢复白费）。窄场地可考虑把 BT 的 `backup_dist` 调小 —— 那属于**改树**，本次未做。
4. **卡住的唯一检测器是 controller 的 progress checker**：内置树里没有 `IsStuck` 条件、没有 `PathLongerOnApproach`，`GoalUpdated` 只用于"目标变了就跳过恢复"。⇒ ① "慢慢蹭不动"要 **10 s** 才被发现；② `RecoveryNode` 6 次用尽后 `NavigateToPose` 直接 **abort**（不会无限重试）—— 在长目标分段（`segment_goal_navigator.py`）里表现为"这一段失败"，需注意上层是否处理。
5. **`behavior_server` 未配 `assisted_teleop`**（1.1.20 该插件存在）⇒ "人工接管把车蹭出来"这条路我们暂时没有；要加就是 `behavior_plugins` 里加一个 id + 一段参数（未做）。
6. `local_frame` 等 Jazzy 键：**不要**从参考队 yaml 抄进来（I.2），1.1.20 既不报错也不生效。

---

## J. params 分层重构：`base + 槽位文件`（2026-10-05，已提交）

### J.1 问题（为什么要改）

旧结构 = **「一个组合一份 full copy」**：`nav2_params_sim_{rpp,dwb,teb,mppi}.yaml` 与
`nav2_params_sim_{rpp,dwb,teb,mppi}_smac2d.yaml` 共 **8 份、每份 400~500 行**，每份都含全部 12 段。

| # | 代价 | 具体表现 |
|---|---|---|
| ① | **改一个公共参数要改 N 份** | `robot_radius` / `inflation_radius` / AMCL 初值这类公共键散在 8 份里，漏改一份 = "同一参数在不同 nav 下不同值"的**静默漂移**（本次就查出一例，见 §J.5） |
| ② | **组合 = 文件夹里有没有这个文件** | 新加一个 nav/planner 忘了补文件 ⇒ launch 直接报 `nav2_params_sim_x.yaml` 不存在；补了又要把 400 行抄一遍 |
| ③ | **A/B 对照无法保证"只差一个变量"** | 8 份副本会各自漂移，做 planner/controller A/B 时说不清差异是不是来自"唯一变量" |

### J.2 新结构（文件清单，都在 `src/nav/rm_navigation/rm_navigation/params/`）

| 文件 | 内容 | 谁选它 |
|---|---|---|
| `nav2_params_sim_base.yaml` | **公共段**：除 `planner_server` / `controller_server` 外的全部 10 段（`amcl`、`amcl_map_client`、`amcl_rclcpp_node`、`bt_navigator`、`local_costmap`、`global_costmap`、`map_saver`、`behavior_server`、`map_server`、`velocity_smoother`），逐字节取自 `nav2_params_sim_rpp.yaml`（注释全保留），文件头加了分层说明 | 总是加载 |
| `nav2_params_sim_planner_navfn.yaml` | `planner_server:` 段（NavFn） | `planner:=navfn`（默认） |
| `nav2_params_sim_planner_smac2d.yaml` | `planner_server:` 段（SmacPlanner2D + 键清单注释） | `planner:=smac2d` |
| `nav2_params_sim_controller_rpp.yaml` | `controller_server:` 段（RPP）**+ 一处 `local_costmap` 键**（见 §J.5） | `nav:=rpp`（默认） |
| `nav2_params_sim_controller_dwb.yaml` | `controller_server:` 段（DWB） | `nav:=dwb` |
| `nav2_params_sim_controller_teb.yaml` | `controller_server:` 段（TEB） | `nav:=teb` |
| `nav2_params_sim_controller_mppi.yaml` | `controller_server:` 段（MPPI）**+ 同一处 `local_costmap` 键**（见 §J.5） | `nav:=mppi` |

共 **7 份新文件**；旧的 8 份 full copy **一份没删、一个字节没改**，只是 launch 不再引用它们（§J.6）。

### J.3 launch 行为（多文件 + 顺序）

- `bringup_sim.launch.py`：`nav2_params_file_dir` 从"一个路径"变成**三个槽位的 list**，再按槽位拆成三个入参传给 include：
  `params_file`（公共段 base）/ `params_file_planner` / `params_file_controller`。
  - ⚠️ **不能把 list 直接塞进 `launch_arguments`**：launch 的 `normalize_to_list_of_substitutions` +
    `perform_substitutions` 会把 list 里每个 substitution 逐个 perform 后**拼接成一个字符串**
    （实测 `[TextSubstitution('/a/base.yaml'), TextSubstitution('/a/planner.yaml')] → '/a/base.yaml/a/planner.yaml'`）
    ⇒ 传给节点的是假路径。所以按槽位拆成三个入参。
- 消费者 launch（`navigation_launch.py`、`bringup_rm_navigation.py`、`localization_amcl_launch.py`、`map_server_launch.py`）
  各多声明两个入参（**默认值 = `params_file`** ⇒ 单独用这些 launch、只给一个文件时行为不变），并把
  `configured_params` 从「一个 `RewrittenYaml`」改成「**三个 `RewrittenYaml` 的 list**」——每份文件各套一层
  **同样的** rewrite（`use_sim_time` / `yaml_filename` / `*.enabled` 图层开关 / `amcl...initial_pose.*` / `root_key=namespace`），
  再原样铺进 `parameters=[*configured_params, …]`。
  - ⚠️ 这里必须是 `[*configured_params]`：若仍写 `parameters=[configured_params]`，那个 list 会被
    `ParameterFile(...)` 当成"一个路径"，三份 temp 文件路径被拼成一串 ⇒ 启动即报文件不存在（本次实测踩到过）。
- **顺序 = base → planner 槽 → controller 槽**。ROS 2 的多份 params 文件是**按参数逐键叠加**（同名后写覆盖；
  **后一份文件里没写的键仍来自前一份文件**，不是整段替换）。用最小 rclpy 节点实测（无话题/服务/rosout）：
  base 给整段 `local_costmap`（`update_frequency` / `publish_frequency` / `width` / `obstacle_layer.*` / `tf=0.1`）
  + 槽位只给 `transform_tolerance: 0.3` ⇒ `update_frequency=20.0`、`publish_frequency=10.0`、
  `obstacle_layer.*` 全在、`transform_tolerance=0.3`（后写赢）；两份都没有的键 ⇒ 保留节点自己 `declare` 的默认值。
  ⇒ 段是"逐键叠加"不是"整段替换"，所以槽位文件里放零散键是安全的（这正是 §J.5 方案的前提）。

### J.4 等价性证明：8/8 组合、0 处差异

两条独立通道，都在重构后实跑通过（脚本：`.tmp_cache/params_refactor/proof_equivalence.py`，**一次性、未提交**）：

- **(a) 逐键 deep merge**：把三份新文件按 base→planner→controller 逐键合并，与旧 full 文件的 `yaml.safe_load` 结果
  做全路径 diff（`dict` 递归、列表整体比较）。
- **(a2) 端到端**：`importlib` 加载**真实的消费者 launch**（4 个），用 `launch_ros.utilities.evaluate_parameters`
  真跑 `RewrittenYaml`，算出每个节点最终拿到的参数字典；对比「旧用法 `params_file=<旧 full 文件>`」与
  「新用法 base + 两个槽位」。覆盖 container(1 节点) / nav2(8 节点) / amcl(3 节点) / map_server(2 节点)，
  含 `use_sim_time`、`yaml_filename`、图层开关等全部 rewrite。

| 组合（nav / planner） | 差异键路径数 |
|---|---|
| rpp / navfn | **0** |
| rpp / smac2d | **0** |
| dwb / navfn | **0** |
| dwb / smac2d | **0** |
| teb / navfn | **0** |
| teb / smac2d | **0** |
| mppi / navfn | **0** |
| mppi / smac2d | **0** |

另核实：`nav` / `planner` 两个 launch 入参的**默认值与 choices 未改**（默认 `nav:=rpp`、`planner:=navfn`）；
`nav:=foo` / `planner:=astar` 仍被 `choices` 拒绝；5 个 launch 文件 `py_compile` 通过；7 份新文件
`yaml.safe_load` 通过；**旧 8 份文件相对 `HEAD` 零改动**。

复验命令（**必须先 build**：`--symlink-install` 只对构建时已存在的文件建符号链接，新文件不 build 不会进 `install/`）：

```bash
colcon build --symlink-install --packages-select rm_navigation
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav localization:=amcl          # 默认 rpp/navfn
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav localization:=amcl nav:=mppi
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav localization:=amcl planner:=smac2d
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav localization:=amcl nav:=mppi planner:=smac2d
```

### J.5 顺带查出的历史漂移：`local_costmap...transform_tolerance`（**本次不改值、不改旧文件**）

证据链：

1. `git log -S'transform_tolerance: 0.3'` ⇒ 该键是 **2026-09-24 的 `f6c2e71`**（"定位 mode:=nav 车不动"）
   **只加/改在 rpp** 上：`local_costmap.local_costmap.ros__parameters.transform_tolerance` 由 `1000.0` 改成 `0.3`
   （撤销"用超长 tf 等待绕过上游 `isGoalReached()` 丢弃 `transformPose` 失败"的错误尝试）；dwb/teb 未被该提交触及。
2. 三个 nav 变体是 2026-09-16 `553737b` 一起建的**独立副本** ⇒ 从那天起 dwb/teb（及其 `_smac2d`）**没有这个键**，
   一直走 nav2 的声明默认值。
3. 上游默认值就是 **0.3**：`third_party/nav2/nav2_costmap_2d/src/costmap_2d_ros.cpp:98`
   `declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3))`
   （本机运行时是 `ros-humble-nav2-costmap-2d 1.1.20`，与 `third_party/nav2` 同版本）。
4. ⇒ 现状是「**rpp/mppi 显式 0.3 vs dwb/teb 依赖 nav2 默认 0.3**」，**值相同、纯文本差异**。
5. 这是本次分层重构**顺带查出来的**（按"base = rpp 去两段"的直觉做，逐键比对时 dwb/teb 各报 1 处差异才暴露）。
   **本次既不改这个值，也不动旧文件**（旧 8 份相对 `HEAD` 零改动已由证明 (c) 核实）。

处理方式与影响面：该键属于 `local_costmap` 段，但"只在 rpp/mppi 显式存在"是 **nav 维度**的差异 ⇒ 只能由
**per-nav 的槽位文件**携带，`base` 不能写它（写了 dwb/teb 就会从"缺键 ⇒ 走默认"变成"显式 0.3"，虽然值相同，
但逐键比对就不再严格相等）。因此：

- `nav2_params_sim_controller_rpp.yaml` / `nav2_params_sim_controller_mppi.yaml` 各含 **2 个顶层键**：
  `controller_server:` + 一段只带 `transform_tolerance: 0.3`（含原注释）的 `local_costmap:`；
  dwb/teb 的两份仍是**严格单段**。影响面就这一处、只影响 `nav:=rpp` 与 `nav:=mppi` 两个组合。
- 好处：**旧 8 份文件零改动 + 8/8 组合逐键严格相等**。
- 代价：槽位文件不再"严格单段"（形式上的不整齐，值上无影响）。

**后续可选（本次不做，必须单独提交）**：重构落地后，可以再开一个独立提交把这行显式写进
`nav2_params_sim_base.yaml` 的 `local_costmap`（四个 nav 统一显式 0.3，**值不变、行为不变**），
然后从两个 controller 槽位里删掉这段 ⇒ 槽位文件回到"严格单段"。那一提交要在 message 里写明"值中性"，
不要混进本次重构。

### J.6 旧文件与回滚

- 旧 8 份 `nav2_params_sim_<nav>[_smac2d].yaml` **保留、未改动**，launch 已不再引用 ⇒ 仅作**参考/回滚**。
  （`bringup_sim.launch.py` 里对应位置有 ⚠️ 注释。）
- 回滚：
  ```bash
  git log --oneline -1        # 本次重构提交（默认即 HEAD）
  git revert HEAD             # 或 git revert <该提交 hash>
  ```
  回滚后 launch 回到"单文件 `nav2_params_sim_<nav>[_smac2d].yaml`"，旧参数文件一直都在；
  `install/` 里多出来的 7 个符号链接无害（`--symlink-install` 产物，不属于 git 内容）。
  只想临时回到旧行为（不提交）：`git checkout <旧提交> -- src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py src/nav/rm_navigation/rm_navigation/launch/`。

### J.7 以后怎么加一个 planner / controller（只加一个文件）

1. **新全局规划器**：复制 `nav2_params_sim_planner_navfn.yaml` → `nav2_params_sim_planner_<新名>.yaml`，
   只改 `plugin` 与该插件**真正声明**的键（键名以 `nav2_<pkg>/src/...` 里的 `declare_parameter` 为准，
   别抄 Jazzy/COD 的专有键 —— 写了是静默失效）；然后把 `bringup_sim.launch.py` 里
   `declare_planner_cmd` 的 `choices` 加上 `<新名>`（路径由 `PythonExpression` 派生，不用改拼接逻辑）。
2. **新局部控制器**：同理 `nav2_params_sim_controller_<新名>.yaml` + `declare_nav_cmd` 的 `choices` 加一项。
3. 两处都不需要动 base、不需要动别的槽位、不需要动消费者 launch。
4. ⚠️ 加完必须 `colcon build --symlink-install --packages-select rm_navigation`（新文件才会被装到 `install/`），
   否则 launch 会报 `..._<新名>.yaml` 不存在。

---

## K. AMCL 高速跟踪参数修正（2026-10-05，`nav:=mppi` 实测反馈："定位不稳/跟不上"）

**反馈**：MPPI（2.0 m/s）跑起来感觉**重定位不稳、有跟不上的情况**。查 base 文件实测值后定位到三处配置：

| 键 | 原值 | 新值 | 原因 |
|---|---|---|---|
| `transform_tolerance` | **1.0** | **0.3** | `map→odom` 被**外推到未来 1 秒**；2 m/s 时≈外推 2 m ⇒ "滞后→猛修正→抖"。原值 1.0 是当初为掩盖 TF 时序问题设的 |
| `update_min_d` | 0.25 | **0.05** | AMCL 是**阈值触发**的离散修正器；2 m/s × 7~10 Hz `/scan` 每帧已走 0.2~0.3 m ⇒ 阈值卡边界、更新被节流 |
| `update_min_a` | 0.2 | **0.05** | 同上（转向阈值） |
| `recovery_alpha_slow` | **0.0** | **0.001** | 原值 0 关闭了 AMCL 的**随机重采样恢复** ⇒ **跟丢即永久跟丢** |
| `recovery_alpha_fast` | **0.0** | **0.1** | 同上 |

**改动位置**：只改了 `nav2_params_sim_base.yaml`（**分层重构后的收益**：公共参数只改一处；旧 8 份 full copy 已废弃、未同步）。
**回退**：逐键改回原值，或 `git revert <本提交>`。若体感变差，**优先回退 `transform_tolerance`**（它影响最大）。

**为什么 AMCL 天生"不跟手"**：它是阈值触发的离散修正器；而 **slam_toolbox 定位（`transform_publish_period` 可 0.02 s = 50 Hz）** 与 **GICP/NDT（每帧点云都匹配）** 是**连续型**匹配器 ⇒ 高速全向车普遍不用 AMCL。

**判据（看数据）**：
```bash
ros2 run tf2_ros tf2_echo map odom                    # 是否仍有"台阶式"跳变
ros2 topic hz /amcl_pose                              # AMCL 实际更新率
python3 tools/scripts/diag/record_tf_monotonic.py     # map→odom 单调性/跳变
```
**后续可选**：`max_beams 60 → 90/120`（更多束参与匹配，代价 CPU）；或直接换连续型（`localization:=slam_toolbox` 需先补 `RMUL2026.posegraph`；`icp` 槽换 small_gicp）。



---


## 来源 13/19：`tdt/cod_nav_2026_params_review.md`


> 范围：只读 `third_party/cod_nav_2026/**` 与 `src/**`，**未改动任何代码 / 配置 / 其它文档**，只新增本文件。
> 每条结论都带 `file:line`。第三方仓库只当参考数据，未执行其中任何代码。

---

## 0. 基准、缩写与判定口径

### 0.1 版本基准（本次评审最关键的前提）

| 事实 | 证据 |
|---|---|
| 本机 nav2 = **1.1.20 (Humble/jammy)** | `dpkg -l` → `ros-humble-nav2-mppi-controller 1.1.20-1jammy.20260908.012051` 等 32 个包全是 1.1.20 |
| 仓库内 nav2 源码 = 同一版本 | `third_party/nav2/nav2_mppi_controller/package.xml:5` `<version>1.1.20</version>`；`third_party/nav2/nav2_controller` 同；git `3c3db59d`，分支 `humble` |
| `install/` 里**没有** nav2 overlay 补丁 | `ls install/` 只有 cartographer / fast_lio / rm_* / teb_local_planner 等，无 `nav2_controller`、`nav2_mppi_controller` |

**推论（贯穿全文）**：cod_nav_2026 的 yaml 里有一大批键是 **nav2 Jazzy/main 才存在**的，在 1.1.20 上**既不报错也不生效（静默失效）**。所以「照抄他们的 yaml」在两边行为**不一样**——这是本次评审最重要的发现，汇总表见 §A.16。

### 0.2 缩写表（表格里全部用它）

| 缩写 | 全路径 |
|---|---|
| **S** | `third_party/cod_nav_2026/src/cod_bringup/params/singlenav2_params.yaml` |
| **M** | `third_party/cod_nav_2026/src/cod_bringup/params/multiplenav2_params.yaml` |
| **SL** / **ML** | `.../cod_bringup/launch/singlenav_launch.py` / `multiplenav_launch.py` |
| **NL** / **LOC** | `.../cod_bringup/launch/navigation_launch.py` / `localization_launch.py` |
| **BT1** / **BTTP** | `.../cod_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` / `navigate_through_poses_...xml` |
| **GAC** | `third_party/cod_nav_2026/src/goal_approach_controller/src/goal_approach_controller.cpp` |
| **GACX** | `.../goal_approach_controller/goal_approach_controller_plugin.xml` |
| **GACC** | `.../goal_approach_controller/CMakeLists.txt` |
| **MAP** | `.../cod_bringup/maps/rmul2026.yaml` |
| **SLAMc** | `.../cod_bringup/params/mapper_params_online_async.yaml`（他们的 slam_toolbox） |
| **FVTc** | `third_party/cod_nav_2026/src/fake_vel_transform/src/fake_vel_transform.cpp` |
| **WE** | `third_party/cod_nav_2026/src/waypoint_editor/` |
| **R** / **D** / **T** | `src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_{rpp,dwb,teb}.yaml` |
| **NAV** | `src/nav/rm_navigation/rm_navigation/launch/navigation_launch.py` |
| **BP** | `src/nav/rm_navigation/rm_navigation/launch/bringup_rm_navigation.py` |
| **LAUNCH** | `src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py` |
| **FVT** / **FVTC** | `src/nav/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp` / `config/fake_vel_params.yaml` |
| **P2L** | `src/perception/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml` |
| **SLAMrm** | `src/nav/rm_navigation/rm_navigation/params/mapper_params_online_async.yaml` |
| **N2/…** | `third_party/nav2/…`（≡ 本机 /opt/ros/humble 的 1.1.20） |

N2 细则（全部相对 `third_party/nav2/`）：
- `N2/ctrl` = `nav2_controller/src/controller_server.cpp`
- `N2/ppgc` = `nav2_controller/plugins/position_goal_checker.cpp`
- `N2/mppi/opt` / `N2/mppi/ctrl` / `N2/mppi/ph` / `N2/mppi/ng` / `N2/mppi/tv` / `N2/mppi/cm` = `nav2_mppi_controller/src/{optimizer,controller,path_handler,noise_generator,trajectory_visualizer,critic_manager}.cpp`
- `N2/mppi/cf` = `nav2_mppi_controller/include/nav2_mppi_controller/critic_function.hpp`
- `N2/mppi/cc` = `nav2_mppi_controller/src/critics/cost_critic.cpp`；`N2/mppi/oc` = `…/critics/obstacles_critic.cpp`；`N2/mppi/gc` = `…/critics/goal_critic.cpp`；`N2/mppi/pfc` = `…/critics/path_follow_critic.cpp`；`N2/mppi/pac` = `…/critics/path_align_critic.cpp`；`N2/mppi/con` = `…/critics/constraint_critic.cpp`
- `N2/cmros` = `nav2_costmap_2d/src/costmap_2d_ros.cpp`；`N2/static` = `nav2_costmap_2d/plugins/static_layer.cpp`；`N2/infl` = `nav2_costmap_2d/plugins/inflation_layer.cpp`；`N2/cmpub` = `nav2_costmap_2d/src/costmap_2d_publisher.cpp`
- `N2/smac2` = `nav2_smac_planner/src/smac_planner_2d.cpp`；`N2/smac_t` = `nav2_smac_planner/include/nav2_smac_planner/types.hpp`；`N2/navfn` = `nav2_navfn_planner/src/navfn_planner.cpp`
- `N2/beh` = `nav2_behaviors/src/behavior_server.cpp`；`N2/spin` = `nav2_behaviors/plugins/spin.cpp`；`N2/timed` = `nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp`
- `N2/bt` = `nav2_bt_navigator/src/bt_navigator.cpp`；`N2/bta` = `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp`
- `N2/wpf` = `nav2_waypoint_follower/src/waypoint_follower.cpp`；`N2/waw` = `nav2_waypoint_follower/plugins/wait_at_waypoint.cpp`
- `N2/vs` = `nav2_velocity_smoother/src/velocity_smoother.cpp`

### 0.3 判定口径

- **✅ 可直接用**：键名在 1.1.20 存在，且值对我们（mecanum/omni、`robot_radius 0.40`、Gazebo Classic sim、costmap 5–20 Hz、`base_link_fake`）语义成立，抄过去不改也能跑。
- **⚠️ 需改造**：键名有效，但值必须按我们的几何/频率/底盘改；或语义依赖他们的架构（LIO + 静态 `map→odom`、云台小陀螺、0.1 m 车身）。
- **❌ 不适用**：1.1.20 无此键（静默失效），或物理前提不成立（0.1 m 车、D435i、DUBIN/Hybrid 车体、50 Hz 控制环）。

### 0.4 一页结论（细节见 A–E）

| 类别 | 条目 |
|---|---|
| **可直接用** | stvl 体素层参数块；Smac2D 的 9 个有效键 + `smoother.*` 子块；velocity_smoother 键名；map_saver；MPPI 骨架键名与 6 个 critic 的键名；waypoint CSV 格式 + `waypoint_to_nav2`/`waypoint_through_nav2` 两个桥节点 |
| **需改造** | `goal_approach_controller`（3 处硬伤，§C.6）；MPPI 全部数值（50 Hz / 1.6 s 时域 / 2.5 m/s）；BT 树（3 Hz、删 Spin/Wait、BackUp 1.0 m）；costmap 几何（0.1 m vs 0.40 m）；goal checker（`path_length_tolerance` 是死键）；velocity_smoother 数值 |
| **不适用** | `vy_min/wz_min/ax_*/ay_*/az_*`（MPPI 1.1.20 无）；`publish_critics_stats`；planner 的 Hybrid/Lattice 整段；`local_costmap_topic` 等 4 个 topic 键 + `local_frame`；`pb_nav2_behaviors/BackUpFreeSpace`（需额外包）；深度相机观测源；静态 `map→odom` 定位法；`auto_save_map.launch.py`（硬编码 `/home/cod-sentry`） |

---

## A. `S`（singlenav2_params.yaml）逐块对照

### A.1 顶层元信息

| 键 | 他们（S 行） | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| 文件总行数 | 483 行（整文件） | R 408 / D 419 / T 418 | — | — |
| `use_sim_time` | `false`（S:3/66/185/276/357/364/372/424/458/462） | `True`（R:3/104/163/245/347/356/391/397） | ⚠️ | 我们全栈 sim 钟；他们实车。**且他们的 `use_sim_time` 是被 `NL:61-71` 的 RewrittenYaml 从 launch 参数覆写的，yaml 里的值基本是惰性的** |
| 帧约定 | `global_frame: map`（S:4/273）、`robot_base_frame: base_link_fake`（S:5/184/274）、`odom_frame: odom`（S:183）、静态 `map→odom`（SL:67-89，yaw **−0.5**；ML:95-116，yaw **0.0**） | 同帧名（R:64/65/161/162/243/244），但 `map→odom` 由 amcl/slam_toolbox/icp/cartographer 提供（LAUNCH:303-406），仅 `mode=nav,localization=''` 才用静态桥（LAUNCH:446-468） | ⚠️ | 帧**名字**一致（`base_link_fake` 是他们的也是我们的），但**谁发 `map→odom`** 完全不同：他们是「LIO 当绝对定位 + 静态桥」，我们是「AMCL/SLAM/ICP/Cartographer」。这直接决定 §C.4 的帧 bug 严重程度 |
| odom 话题名 | `Odometry`（S:6、S:482） | `odom`（R:407）／`/Odometry`（R:66，**疑似过期**） | ⚠️ | 他们 LIO 原生话题就是 `Odometry`；我们 T2 已统一到 `/odom`（LAUNCH:256/282），但 `bt_navigator.odom_topic` 还是 `/Odometry`（R:66）→ BT 的 `OdomSmoother` 收不到数据（N2/bt_navigator.cpp:98/135 只声明/订阅这一路）。低危但不干净 |
| 机器人几何 | `robot_radius: 0.1` + `footprint ±0.1`（S:190-191/277-278） | `robot_radius: 0.40` 且**无 footprint**（R:178/252） | ❌ | 见 §A.5/A.6；且**无 footprint 这一点会直接触发 MPPI critic 抛异常**（§A.4、§E） |
| 速度量级 | 2.5 m/s、wz 1.5（S:99-105） | RPP `desired_linear_vel: 0.5`（R:130）、DWB `max_vel_x 2.5`（D:133）、TEB `max_vel_x 0.5`（T:144） | ❌ | 他们哨兵实车高速；我们 sim + 场地（RMUL2026 图 12.75 m × 8.85 m，见 §A.13） |

### A.2 controller_server 框架参数

| 键 | 他们（S 行） | 我们（R 行） | 判定 | 理由 |
|---|---|---|---|---|
| `publish_critics_stats` | `True`（S:65） | 无 | ❌ | **1.1.20 全 nav2 无此键**（`grep -rn publish_critics_stats third_party/nav2/{nav2_controller,nav2_mppi_controller}` 无命中）→ 静默失效 |
| `controller_frequency` | `50.0`（S:67） | `20.0`（R:105） | ⚠️ | 见 §A.3 `model_dt`：**50 Hz 是他们的隐含前提**；我们 20 Hz 直接抄 `model_dt: 0.02` 会让 MPPI 在 configure 阶段抛异常 |
| `min_x_velocity_threshold` | `0.001`（S:68） | `0.001`（R:106） | ✅ | 一致 |
| `min_y_velocity_threshold` | `0.001`（S:69） | `0.5`（R:107） | ⚠️ | 我们 0.5 → 侧向**里程计反馈**小于 0.5 m/s 被抹成 0（`N2/ctrl` 的 `getThresholdedTwist`）。RPP 不产生 vy 所以以前无害，**接 MPPI(Omni) 前必须降到 0.001** |
| `min_theta_velocity_threshold` | `0.001`（S:70） | `0.001`（R:108） | ✅ | 一致 |
| `failure_tolerance` | `0.3`（S:71） | `0.3`（R:109） | ✅ | 一致（秒） |
| `progress_checker.plugin` | `nav2_controller::SimpleProgressChecker`（S:76） | 同（R:116） | ✅ | — |
| `progress_checker.required_movement_radius` | `0.1`（S:77） | `0.5`（R:117） | ⚠️ | 0.1 m 配合 `movement_time_allowance 999 s`（S:78）≈ **关掉卡死检测**；我们 0.5 m/10 s 更严。抄他们等于放弃 progress 保护，建议保留我们的 |
| `progress_checker.movement_time_allowance` | `999.0`（S:78） | `10.0`（R:118） | ⚠️ | 同上（单位秒；999 s 实为禁用） |
| `goal_checker_plugins` | `["general_goal_checker"]`（S:73） | `["general_goal_checker"]`（R:111） | ✅ | 名字一致 |
| `general_goal_checker.plugin` | `nav2_controller::PositionGoalChecker`（S:81） | `nav2_controller::SimpleGoalChecker`（R:123） | ⚠️ | **`PositionGoalChecker` 在 1.1.20 存在**（`third_party/nav2/nav2_controller/plugins.xml` 的 `position_goal_checker` 段；`N2/ppgc:28-52`）→ 可换。它只看 XY、忽略 yaw（`N2/ppgc:76-87`），与「omni 车不必对朝向 + direct-approach 强制 `angular.z=0`」自洽 |
| `general_goal_checker.stateful` | `False`（S:80） | `True`（R:122） | ⚠️ | `PositionGoalChecker` 的 `stateful` 是**真键**（`N2/ppgc:47-52`）。false ⇒ 每周期重算「是否在 0.2 m 内」；true ⇒ 一旦进入就锁存（`N2/ppgc:70-73`）。配合他们的 BT（到点即结束）false 更安全 |
| `general_goal_checker.xy_goal_tolerance` | `0.2`（S:82） | `0.25`（R:124） | ⚠️ | 0.2 比我们 0.25 严；注意他们 `direct_approach_distance 2.0` 使末端由直接驱动接管，所以 0.2 不会引起绕圈 |
| `general_goal_checker.path_length_tolerance` | `0.5`（S:83） | — | ❌ | **1.1.20 的 PositionGoalChecker 不声明此键**：`N2/ppgc:44-52` 只声明 `xy_goal_tolerance` + `stateful`；对已安装 `libposition_goal_checker.so` 取字符串也只有 `.xy_goal_tolerance`。该键只在 nav2 更新版（2025 年 Prabhav Saxena 版本）里存在 → 我们这边**静默失效**，注释「path_length_tolerance」不可作为采用依据 |
| `general_goal_checker.yaw_goal_tolerance` | 无（PositionGoalChecker 无此键） | `0.25`（R:125，SimpleGoalChecker 有） | — | 换 PositionGoalChecker 时必须删掉 `yaw_goal_tolerance`，否则是死键 |
| `FollowPath.plugin` | `goal_approach_controller::GoalApproachController`（S:85） | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`（R:129）／`dwb_core::DWBLocalPlanner`（D:129）／`teb_local_planner::TebLocalPlannerROS`（T:130） | ⚠️ | 见 §C 全节 |

### A.3 FollowPath：wrapper + MPPI 骨架参数

他们 FollowPath 段 = S:84-176。`S:85-86` 说明是 `goal_approach_controller` 包 MPPI；**wrapper 与 MPPI 共用同一个参数命名空间 `FollowPath.*`**（GAC:63 `inner_controller_->configure(parent, name, tf, costmap_ros)` 传的是同一个 `name`）。

| 键 | 他们（S 行 = 值） | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `inner_plugin` | `nav2_mppi_controller::MPPIController`（S:86） | 无（MPPI 未接线，但 `ros-humble-nav2-mppi-controller 1.1.20` 已装、`third_party/nav2/nav2_mppi_controller` 在库） | ⚠️ | wrapper 参数，GAC:36-38 声明默认即 MPPI |
| `approach_distance` | `2.5`（S:87） | 无 | ⚠️ | wrapper 专有（GAC:39-41，默认 1.5）；见 §C.3 |
| `approach_velocity` | `0.2`（S:88） | 无 | ⚠️ | wrapper 专有（GAC:42-44，默认 0.5）；见 §C.3 |
| `direct_approach_distance` | `2.0`（S:89） | 无 | ⚠️ | wrapper 专有（GAC:45-47，默认 0.5）；见 §C.4 |
| `direct_approach_kp` | `3.0`（S:90） | 无 | ⚠️ | wrapper 专有（GAC:48-50，默认 1.0） |
| `time_steps` | `80`（S:91） | 无 | ⚠️ | **真键**（`N2/mppi/opt:70`，默认 56）。80×0.02 = **1.6 s** 时域（他们注释写 4.8 s，见 §F-1） |
| `model_dt` | `0.02`（S:92） | 无 | ⚠️⚠️ | **真键**（`N2/mppi/opt:69`，默认 0.05）。**硬约束**：`N2/mppi/opt:95-113` 要求 `1/controller_frequency == model_dt`；我们 `controller_frequency 20.0`（R:105）→ 周期 0.05 > 0.02 → **`throw std::runtime_error("Controller period more then model dt")`**（`N2/mppi/opt:111-112`）→ controller_server configure 失败、整栈起不来。两条出路：(a) `controller_frequency: 50.0`（照抄他们的 50 Hz）；(b) `model_dt: 0.05`（= 我们 20 Hz，且会打开 `shift_control_sequence`，`N2/mppi/opt:104-109`）。周期 < model_dt 只 WARN（`N2/mppi/opt:100-103`），所以 M 的 0.06（周期 0.02）合法但会刷警告 |
| `batch_size` | `1500`（S:93） | 无 | ⚠️ | 真键（`N2/mppi/opt:71`，默认 1000）。1500×80 的采样规模在 sim 里 CPU 开销大，建议先 1000 |
| `iteration_count` | `1`（S:115） | 无 | ✅ | 真键（`N2/mppi/opt:72`，默认 1）；值=默认 |
| `temperature` | `0.25`（S:118） | 无 | ⚠️ | 真键（`N2/mppi/opt:73`，默认 0.3）。0.25 更「果断」，但对他们 2.5 m/s 调的，我们低速需重调 |
| `gamma` | `0.015`（S:119） | 无 | ✅ | 真键（`N2/mppi/opt:74`）；= 默认 |
| `motion_model` | `"Omni"`（S:120） | 无 | ✅ | 真键（`N2/mppi/opt:84`；合法值 DiffDrive/Omni/Ackermann，`N2/mppi/opt:406-420`）。我们 mecanum ⇒ **Omni 正确** |
| `vx_max` | `2.5`（S:99） | DWB `max_vel_x: 2.5`（D:133）；RPP `desired_linear_vel: 0.5`（R:130） | ⚠️ | 真键（`N2/mppi/opt:75`）。我们 sim 建议 0.5–1.0 |
| `vx_min` | `-2.5`（S:103） | DWB `min_vel_x: -2.5`（D:131） | ⚠️ | 真键（`N2/mppi/opt:76`）。允许全速倒车，sim 里危险 |
| `vy_max` | `2.5`（S:100） | DWB `max_vel_y: 2.5`（D:134） | ⚠️ | 真键（`N2/mppi/opt:77`，注意 **1.1.20 只有 `vy_max` 一个键，vy 对称**） |
| `wz_max` | `1.5`（S:101） | RPP `rotate_to_heading_angular_vel: 1.8`（R:135）；velocity_smoother `max_velocity[2]=3.0`（R:401） | ⚠️ | 真键（`N2/mppi/opt:78`）。注意他们 `velocity_smoother.max_velocity` 写的是 2.5（S:476），与 `wz_max 1.5` **不一致** |
| `vx_std` / `vy_std` / `wz_std` | `0.3 / 0.3 / 0.2`（S:95-97） | 无 | ⚠️ | 真键（`N2/mppi/opt:79-81`，默认 0.2/0.2/0.4）。wz_std 比默认**小** |
| **`vy_min`** | `-2.5`（S:104） | — | ❌ | **1.1.20 无此键**（`N2/mppi/opt:75-84` 全部 `getParam` 清单里没有；全包 grep `"vy_min"` 无命中） |
| **`wz_min`** | `-1.5`（S:105） | — | ❌ | 同上，无此键 |
| **`ax_max` / `ay_max` / `az_max`** | `1.5 / 1.5 / 1.0`（S:107-109） | 无 | ❌ | 1.1.20 MPPI **没有加速度约束参数**（`N2/mppi/opt:69-84` 无 ax/ay/az）⇒ 这三行连同注释「制动力必须对称，高速下刹得住」**完全不生效**。真正限加速度的是 `velocity_smoother.max_accel/max_decel`（S:480-481 / R:405-406） |
| **`ax_min` / `ay_min` / `az_min`** | `-3.5 / -3.5 / -3.0`（S:111-113） | 无 | ❌ | 同上，死键 |
| `prune_distance` | `1.7`（S:116） | 无 | ⚠️ | 真键（`N2/mppi/ph:38`，默认 1.5） |
| `transform_tolerance` | `0.1`（S:117） | 无（我们控制器侧不设；costmap 是 0.3，R:158） | ⚠️ | 真键（`N2/mppi/ph:39`） |
| `visualize` | `false`（S:121） | 无 | ✅ | 真键（`N2/mppi/ctrl:41`，默认 false） |
| `reset_period` | `0.08`（S:122，注释「only in Humble」） | 无 | ✅ | 真键（`N2/mppi/ctrl:42`，默认 1.0）；注释正确，1.1.20 确实有 |
| `retry_attempt_limit` | 仅 M（M:117 `2`）；S 无 | 无 | ⚠️ | 真键（`N2/mppi/opt:82`，默认 1）。注意 `N2/mppi/opt:175-177` 超限会 `throw "Optimizer fail to compute path"` |
| `regenerate_noises` | 仅 M（M:118 `true`）；S 无 | 无 | ⚠️ | 真键（`N2/mppi/ng:35`，默认 false） |
| `TrajectoryVisualizer.trajectory_step` | `5`（S:124） | 无 | ✅ | 真键，命名空间 `FollowPath.TrajectoryVisualizer.*`（`N2/mppi/tv:34-36`），默认 5 |
| `TrajectoryVisualizer.time_step` | `3`（S:125） | 无 | ✅ | 同上，默认 3 |

### A.4 critics 列表与逐个参数

`critics`（S:126-129）= `["ConstraintCritic","CostCritic","GoalCritic","PathFollowCritic","PathAlignCritic","ObstaclesCritic"]`，是 1.1.20 的真键（`N2/mppi/cm:39`）。**注意：critic 的启用只由这份列表决定（`N2/mppi/cm:50-59`），每段里的 `enabled:` 也是真键（`N2/mppi/cf:81`，默认 true）**，两者都写才是他们那样。

| 键 | 他们（S 行=值） | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `enabled`（各 critic） | 全 `true`（S:132/137/142/152/162/169） | — | ✅ | 真键（`N2/mppi/cf:81`），默认就是 true，写了也无害 |
| `ConstraintCritic.cost_power/cost_weight` | `1 / 4.0`（S:133-134） | — | ✅ | `N2/mppi/con:25-26`，默认正是 1 / 4.0 |
| `GoalCritic.cost_weight` | `25.0`（S:139） | — | ⚠️ | 默认 5.0（`N2/mppi/gc:28`）。25 是他们「防画弧」的手段，MPPI 权重需按我们速度重调 |
| `GoalCritic.threshold_to_consider` | `4.0`（S:140） | — | ⚠️ | 默认 1.4（`N2/mppi/gc:29`） |
| `ObstaclesCritic.repulsion_weight` | `1.5`（S:144） | — | ✅ | 默认 1.5（`N2/mppi/oc:27`） |
| `ObstaclesCritic.critical_weight` | `20.0`（S:145） | — | ✅ | 默认 20.0（`N2/mppi/oc:28`） |
| `ObstaclesCritic.consider_footprint` | `false`（S:146） | — | ✅ | 默认 false（`N2/mppi/oc:25`）；**我们必须保持 false**，见下方 CostCritic 行 |
| `ObstaclesCritic.collision_cost` | `10000.0`（S:147） | — | ✅ | 默认 10000（`N2/mppi/oc:29`） |
| `ObstaclesCritic.collision_margin_distance` | `0.2`（S:148） | — | ⚠️ | 默认 0.10（`N2/mppi/oc:30`）。0.2 对 0.1 m 车是 2 倍车宽；对我们 0.40 半径含义不同 |
| `ObstaclesCritic.near_goal_distance` | `0.5`（S:149） | — | ⚠️ | 默认 0.5（`N2/mppi/oc:31`）；值=默认 |
| （他们没写但真实存在的 ObstaclesCritic 键） | — | — | ⚠️ | `ObstaclesCritic.cost_scaling_factor`（默认 10.0）与 `ObstaclesCritic.inflation_radius`（默认 0.55）是真键（`N2/mppi/oc:91-93`），可用来覆盖 **critic 自己**看到的膨胀参数（与 costmap 的 `inflation_layer` 解耦）。这是他们没用、但对我们「两套 inflation 数值不一致」问题有用的旋钮 |
| `CostCritic.cost_weight` | `3.8`（S:154） | — | ⚠️ | 默认 3.81（`N2/mppi/cc:28`）≈默认（注释「适当降低」不成立，见 §F-5） |
| **`CostCritic.critical_cost`** | `253.0`（S:155，注释「只有 inscribed/lethal 才触发碰撞惩罚」） | — | ❌（语义被误解） | 这是**惩罚幅度**不是阈值：默认 300.0（`N2/mppi/cc:29`）。「只有 inscribed 以上才罚」是源码**硬编码**的 `if (pose_cost >= INSCRIBED_INFLATED_OBSTACLE)`（`N2/mppi/cc:165`，1.1.20）。设成 253.0 只是把惩罚值调成与 253 同量级，与「关键修正」无关 → §F-4 |
| **`CostCritic.consider_footprint`** | `true`（S:156） | — | ❌ | **会直接抛异常**：`N2/mppi/cc:56-65` 判 `costmap_ros_->getUseRadius() == consider_footprint_`，相等且 `getUseRadius()` 为真时 `throw PlannerException`（`cc:62-64`）。我们**没有 footprint、只有 `robot_radius: 0.40`**（R:178/252）⇒ `use_radius_ = true`（`N2/cmros:408-416`）⇒ `true == true` ⇒ **configure 阶段抛异常、控制器起不来**。要用 CostCritic 必须：先加 footprint，或把 `consider_footprint` 设 false（`ObstaclesCritic` 有**完全相同**的检查，`N2/mppi/oc:46-55`，所以我们的 `ObstaclesCritic.consider_footprint` 也必须为 false） |
| `CostCritic.collision_cost` | `1000000.0`（S:157） | — | ✅ | 默认 1e6（`N2/mppi/cc:30`） |
| `CostCritic.near_goal_distance` | `0.5`（S:158） | — | ✅ | 默认 0.5（`N2/mppi/cc:31`） |
| **`CostCritic.trajectory_point_step`** | `2`（S:159） | — | ❌ | **1.1.20 的 `CostCritic` 不读此键**：它的 `getParam` 只有 `consider_footprint/cost_power/cost_weight/critical_cost/collision_cost/near_goal_distance/inflation_layer_name`（`N2/mppi/cc:26-32`）。全包读 `trajectory_point_step` 的只有 `N2/mppi/pac:34` 与 `path_align_legacy_critic.cpp:34` ⇒ 这一行**静默失效** |
| `PathFollowCritic.cost_weight` | `15.0`（S:164） | — | ⚠️ | 默认 5.0（`N2/mppi/pfc:32`） |
| `PathFollowCritic.offset_from_furthest` | `5`（S:165） | — | ⚠️ | 默认 6（`N2/mppi/pfc:30`） |
| `PathFollowCritic.threshold_to_consider` | `1.0`（S:166） | — | ⚠️ | 默认 1.0（`N2/mppi/pfc:27-29`）；值=默认 |
| `PathAlignCritic.cost_weight` | `15.0`（S:171） | — | ⚠️ | 默认 10.0（`N2/mppi/pac:30`） |
| `PathAlignCritic.max_path_occupancy_ratio` | `0.05`（S:172） | — | ⚠️ | 默认 0.07（`N2/mppi/pac:32`） |
| `PathAlignCritic.trajectory_point_step` | `4`（S:173） | — | ✅ | 默认 4（`N2/mppi/pac:34`） |
| `PathAlignCritic.threshold_to_consider` | `0.8`（S:174） | — | ⚠️ | 真键（`N2/mppi/pac:35-37`） |
| `PathAlignCritic.offset_from_furthest` | `20`（S:175） | — | ✅ | 默认 20（`N2/mppi/pac:33`） |
| `PathAlignCritic.use_path_orientations` | `false`（S:176） | — | ✅ | 默认 false（`N2/mppi/pac:38`）。omni 车正确 |
| 未列入的可用 critic | `GoalAngleCritic` / `PathAngleCritic` / `TwirlingCritic` / `PreferForwardCritic` / `VelocityDeadbandCritic` / `PathAlignLegacyCritic`（`third_party/nav2/nav2_mppi_controller/critics.xml:16-50`） | — | ⚠️ | 他们**一个都没用**：说明整套设计刻意不做朝向跟踪（omni + 无 yaw 目标 + direct-approach `angular.z=0`）。我们要接 MPPI 若要保朝向，得自己加 `GoalAngleCritic`/`PathAngleCritic` |

### A.5 local_costmap

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `update_frequency` | `20.0`（S:181） | `20.0`（R:159） | ✅ | 一致 |
| `publish_frequency` | `20.0`（S:182） | `10.0`（R:160） | ✅ | 20 Hz 发布两张 costmap 在 sim 里白烧 CPU，我们 10 Hz 更稳 |
| `global_frame` | `odom`（S:183） | `odom`（R:161） | ✅ | 一致；也是 §C.4 帧 bug 的根 |
| `robot_base_frame` | `base_link_fake`（S:184） | `base_link_fake`（R:162） | ✅ | 一致 |
| `rolling_window` | `true`（S:186） | `true`（R:164） | ✅ | 一致 |
| `width` / `height` | `10` / `10`（S:187-188） | `5` / `5`（R:165-166） | ⚠️ | 他们 10 m 窗口配 2.5 m/s；我们 5 m（R:165-166）在 0.5 m/s 够用，但若提速要先放大 |
| `resolution` | `0.05`（S:189） | `0.02`（R:167） | ⚠️ | 我们 2 cm 更细（为了 0.40 半径下的窄缝判断，见 R:168-178 注释）；他们的 5 cm 与其 stvl `voxel_size 0.05` 对齐。**若抄他们的 stvl 参数，注意 voxel 5 cm > 我们 costmap 2 cm** |
| **`robot_radius`** | `0.1`（S:190） | `0.40`（R:178） | ❌ | 0.1 m 是 0.2 m 见方车体的一半（配 footprint `±0.1`）；我们实车/仿真按 0.40 设计（R:173-177），**窄于 0.8 m 的缝判不可通行**是刻意的安全几何。抄 0.1 会让规划器贴墙钻盲区 |
| `footprint` | `"[ [0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1] ]"`（S:191） | 无（R 全文无 footprint） | ⚠️ | 他们的 footprint **非空** ⇒ `use_radius_ = false`（`N2/cmros:411-416`）⇒ 有效内切半径 = 0.1，`robot_radius` 反而**被忽略**。我们无 footprint ⇒ 圆形 0.40。这是两侧几乎所有「几何类参数」不可比的根源 |
| `plugins` | `["static_layer","stvl_voxel_layer","inflation_layer"]`（S:192） | `["obstacle_layer","obstacle_cloud_layer","inflation_layer"]`（R:182） | ❌（结构不同） | 他们局部**没有 `obstacle_layer`**，也没有 `/scan` 源；靠 stvl 吃 `/livox/lidar_filtered`+`/camera/.../points`（S:218-262）。我们在局部**刻意不用 static_layer**（局部不要静态先验压住实时障碍）且 `local_obstacle` 槽位在 scan/cloud/both 间切（NAV:76-90） |
| `static_layer.map_subscribe_transient_local` | `True`（S:195） | 局部无 static_layer | ⚠️ | 他们在 rolling local costmap 里塞 static_layer，`N2/static:193-205, 254-256` 对 rolling 有专门分支 ⇒ 只是把静态图贴进滚动窗，语义与我们「局部只用实时传感器」不同 |
| `inflation_layer.cost_scaling_factor` | `5.0`（S:198） | `5.0`（R:232） | ✅ | **完全一致** |
| `inflation_layer.inflation_radius` | `0.55`（S:199） | `0.5`（R:235） | ⚠️ | 他们 0.55 对 0.1 m 车是 5.5 倍；我们 0.5 对 0.40 m 车（必须 ≥ `robot_radius`，`N2/infl:173-181` 会在小于内切半径时报 ERROR）。若我们要 0.55 也合法（0.55 > 0.40），可作为「更早看到障碍」的 A/B 项 |
| `stvl_voxel_layer.*` | S:200-262（含 `voxel_decay 0.5`、`decay_model 0`、`voxel_size 0.05`、`track_unknown_space true`、`observation_persistence 0.0`、`max_obstacle_height 2.0`、`mark_threshold 0`、`update_footprint_enabled true`、`combination_method 1`、`obstacle_range 3.0`、`origin_z 0.0`、`publish_voxel_map false`、`transform_tolerance 0.2`、`mapping_mode false`、`map_save_duration 60.0`） | 全局 stvl 同键名（R:288-333），局部**没有 stvl** | ✅（键名） | 键名逐条与我们的 STVL 配置同构（我们另加 `publish_voxel_map: true`、`livox_clear` 第二源）→ 这一块**可直接抄**；但**数据源不同**：他们 `topic: /livox/lidar_filtered`（S:221）+ `min_obstacle_height: 0.15`（S:229）；我们 `topic: /segmentation/obstacle`（R:303）+ `min_obstacle_height: 0.0`（R:311） |
| `realsense_source` | S:241-262（`/camera/camera/depth/color/points`，`model_type 0`，`min_z 0.15`，`min_obstacle_height 0.18`） | 无 | ❌ | 我们无 D435i；源话题在我们栈里不存在（RViz 能看到但 costmap 只会刷 MessageFilter 警告） |
| `always_send_full_costmap` | `False`（S:263） | `True`（R:236） | ⚠️ | 他们只要增量；我们 `True` 是为了 RViz/调试与上游 `isGoalReached()` 丢 transformPose 那个坑（见 R:154-157 与 LAUNCH:187-195 的长注释）。**不要抄 False**，会退回我们已踩过的坑 |

### A.6 global_costmap

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `width` / `height` | `25` / `25`（S:269-270） | 无（不滚动，尺寸随静态图） | ❌ | 他们全局 costmap 是 **rolling 25×25 m**（S:275）；我们是**非滚动 + 跟随 map_server 图**（`N2/static:193-205, 254-256` 在非 rolling 时会 resize 主 costmap） |
| `rolling_window` | `true`（S:275） | 无（=false） | ❌ | 这是两侧全局规划语义的**根本差异**：他们「地图只是滚动窗里的一块纹理」，我们「全局图就是地图」。他们的 `allow_unknown: true`（S:378）+ `track_unknown_space: true`（S:280）+ 窗口大于地图（25 m > MAP 12.75 m）⇒ **图外未知区可被规划器当作可通行**。我们正是要避免这个（R:158-178 的安全几何思路） |
| `update_frequency` | `20.0`（S:271） | `5.0`（R:241） | ⚠️ | 20 Hz 全局图在 sim 里是纯浪费（全局图只在重规划时用，我们 5 Hz + `expected_planner_frequency 5.0` R:355 已自洽） |
| `publish_frequency` | `20.0`（S:272） | `2.0`（R:242） | ⚠️ | 同上 |
| `global_frame` / `robot_base_frame` | `map` / `base_link_fake`（S:273-274） | `map` / `base_link_fake`（R:243-244） | ✅ | 一致 |
| `robot_radius` | `0.1`（S:277） | `0.40`（R:252） | ❌ | 同 A.5 |
| `footprint` | `±0.1`（S:278） | 无 | ⚠️ | 见 A.5 的 `use_radius_` 说明 |
| `resolution` | `0.04`（S:279） | `0.04`（R:253） | ✅ | **完全一致** |
| `track_unknown_space` | `true`（S:280） | `true`（R:254） | ⚠️ | 键值一致，但配合他们「rolling + allow_unknown」语义变成「图外可走」；我们是「图外=未知=不可走」的用法 |
| `plugins` | `["static_layer","stvl_voxel_layer","inflation_layer"]`（S:281） | `["static_layer","obstacle_layer","stvl_layer","inflation_layer"]`（R:258） | ✅（键名/结构） | **结构完全同构**：都是 static + (stvl/obstacle) + inflation；我们多一个 `obstacle_layer` 并靠 `enabled` 在 stvl/scan/none 间切（NAV:65-90）——这正是可以直接借鉴他们的「槽位」写法 |
| `static_layer.enable` | `true`（S:285） | `footprint_clearing_enabled: True`（R:336），无 `enable` | ⚠️ | **`static_layer.enable` 在 1.1.20 不是 nav2 `StaticLayer` 的参数**（1.1.20 的 StaticLayer 没有 enabled 概念；它是 `CostmapLayer`/`Layer` 但 `enable` 键只有部分图层读）。`map_subscribe_transient_local: True`（S:284 / R:337）是真键且两边一致 ✅ |
| `inflation_layer.cost_scaling_factor` | `5.0`（S:288，注释「与 local_costmap 一致」） | `8.0`（R:340） | ⚠️ | 他们 5.0 == 自己 local 5.0（S:198），注释成立；我们 global 8.0 ≠ local 5.0（R:340 vs R:232）。**注意 M 里这条注释与数值矛盾**（M:282 注释说一致，值却是 15.0），见 §F-3 |
| `inflation_layer.inflation_radius` | `0.75`（S:289） | `0.55`（R:342） | ⚠️ | 0.75 对 0.1 m 车；对我们 0.40 也合法（≥ 内切半径）。可作为「全局更早避障」的 A/B 项，但会加剧窄缝不可通行 |
| `stvl_voxel_layer.*` | S:290-352 | R:284-333 | ✅（键名） | 同 A.5；`min_obstacle_height` 他们 0.15（S:319）/ 我们 0.0（R:311，2026-09-24 特意调低，理由是全局图才决定规划） |
| `always_send_full_costmap` | `False`（S:353） | `True`（R:343） | ⚠️ | 同 A.5 |

### A.7 planner_server（Smac2D）

他们 = S:370-405（`nav2_smac_planner/SmacPlanner2D`，S:376）。**关键：1.1.20 的 `SmacPlanner2D` 只读 9 个键 + `smoother.*` 子块**（`N2/smac2:64-92` 逐个 `declare_parameter_if_not_declared`；`N2/smac_t:70-90` 的 `SmootherParams::get`）。

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `plugin` | `nav2_smac_planner/SmacPlanner2D`（S:376） | `nav2_navfn_planner/NavfnPlanner`（R:359） | ⚠️ | `ros-humble-nav2-smac-planner 1.1.20` 已装 ⇒ 可切 |
| `tolerance` | `0.5`（S:377） | `0.5`（R:360） | ✅ | **完全一致**，且都是真键（`N2/smac2:64-66`；NavFn 侧 `navfn_planner.cpp:84`） |
| `allow_unknown` | `true`（S:378） | `true`（R:362） | ✅ | 一致（但见 A.6 rolling 的语义差） |
| `downsample_costmap` | `false`（S:379） | — | ✅ | 真键（`N2/smac2:67-69`），默认 false |
| `downsampling_factor` | `1`（S:380） | — | ✅ | 真键（`N2/smac2:70-72`） |
| `max_iterations` | `1000000`（S:381） | — | ✅ | 真键（`N2/smac2:80-82`），默认同 |
| `max_on_approach_iterations` | `1000`（S:382） | — | ✅ | 真键（`N2/smac2:83-85`），默认同 |
| `max_planning_time` | `4.5`（S:383） | — | ⚠️ | 真键（`N2/smac2:90-92`，默认 2.0）。4.5 s 在 20 Hz 控制下会拖慢；建议 1.0–2.0 |
| `cost_travel_multiplier` | `4.0`（S:384） | — | ⚠️ | 真键（`N2/smac2:73-75`，默认 1.0）。4.0 强烈贴通道中心——我们已有 `robot_radius 0.40` 的粗安全几何，1.0–2.0 更合适 |
| `smoother.max_iterations` | `10000`（S:401） | — | ✅ | 真键（`N2/smac_t:78-80`，默认 1000） |
| `smoother.w_smooth` | `0.4`（S:402） | — | ✅ | 真键（`N2/smac_t:84-86`，默认 0.3） |
| `smoother.w_data` | `0.1`（S:403） | — | ✅ | 真键（`N2/smac_t:81-83`，默认 0.2） |
| `smoother.tolerance` | `1.0e-10`（S:404） | — | ✅ | 真键（`N2/smac_t:75-77`，默认 1e-10） |
| `smoother.do_refinement` | `true`（S:405） | — | ✅ | 真键（`N2/smac_t:87-89`，默认 true） |
| **`motion_model_for_search: "DUBIN"`** | S:385 | — | ❌ | 只在 `N2/smac_planner_hybrid.cpp:137-138` 读；2D 计划器不读（`N2/smac2:114` 硬编码 `_motion_model = MotionModel::TWOD`） |
| **`angle_quantization_bins: 72`** | S:386 | — | ❌ | 同上（`smac_planner_hybrid.cpp:78-79`） |
| **`analytic_expansion_ratio: 3.5`** | S:387 | — | ❌ | Hybrid/Lattice 专用（`smac_planner_hybrid.cpp:121`、`analytic_expansion.cpp:71`） |
| **`analytic_expansion_max_length: 3.0`** | S:388 | — | ❌ | 同上（`smac_planner_hybrid.cpp:124`） |
| **`minimum_turning_radius: 0.05`** | S:389 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:100`）。0.05 m 转弯半径本身就是「差不多原地转」，只在差速上才有意义 |
| **`retrospective_penalty: 0.025`** | S:390 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:118`） |
| **`reverse_penalty: 1.0`** | S:391 | — | ❌ | Reeds-Shepp/Hybrid 专用（`smac_planner_hybrid.cpp:106`） |
| **`change_penalty: 0.0`** | S:392 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:109`） |
| **`non_straight_penalty: 0.0`** | S:393 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:112`）。注释写「必须 ≥ 1」但给 0.0 → 注释/值矛盾（若真在 Hybrid 上会告警） |
| **`cost_penalty: 2.0`** | S:394 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:115`）。2D 用的是 `cost_travel_multiplier` |
| **`rotation_penalty: 5.0`** | S:395 | — | ❌ | Lattice 专用（`node_lattice.cpp:66`） |
| **`lookup_table_size: 20.0`** | S:396 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:133`） |
| **`cache_obstacle_heuristic: True`** | S:397 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:103`） |
| **`allow_reverse_expansion: True`** | S:398 | — | ❌ | Lattice 专用（`node_lattice.cpp:65`） |
| **`smooth_path: True`** | S:399 | — | ❌ | 只有 `smac_planner_hybrid.cpp:96` 读；2D 侧 `_smoother->smooth(...)` 是**无条件**调用（`N2/smac2:299-300`），所以写不写都一样平滑 |
| `use_astar` | 无 | `false`（R:361） | ✅ | NavFn 真键（`navfn_planner.cpp:86`）；保留我们的 |

> **结论**：他们 planner 段 30 行里，**16 行是 Hybrid/Lattice 专有、在 Smac2D 上完全无效**。可安全搬运的只有 `tolerance / allow_unknown / downsample_costmap / downsampling_factor / max_iterations / max_on_approach_iterations / max_planning_time / cost_travel_multiplier / smoother.*` 这几项。

### A.8 smoother_server

他们 = S:407-420。

| 键 | 他们（S 行=值） | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `costmap_topic` / `footprint_topic` | `global_costmap/costmap_raw` / `global_costmap/published_footprint`（S:409-410） | 我们**完全没有 `smoother_server:` 段**（R/D/T 全文无） | ✅（键名） | 1.1.20 `nav2_smoother` 真键；`nav2_smoother` 服务已由 `NAV:161-169`（非组合）/`NAV:244-248`（组合）拉起，走内置默认。抄这一段是纯增量 |
| `robot_base_frame` | `base_link_fake`（S:411） | 无（默认 `base_link`） | ⚠️ | 若抄必须一起改，否则 SmootherServer 的 TF 查询会找不到 `base_link`（我们只有 `base_link_fake`） |
| `transform_tolerance` | `0.1`（S:412） | 无 | ✅ | 真键 |
| `smoother_plugins` | `["savitzky_golay_smoother"]`（S:413） | 无 | ✅ | `nav2_smoother::SavitzkyGolaySmoother` 在 1.1.20 存在（`third_party/nav2/nav2_smoother/plugins.xml`） |
| `window_size` / `poly_order` | `7` / `3`（S:416-417） | 无 | ✅ | 真键，默认同 |
| `do_refinement` / `refinement_num` | `True` / `2`（S:418-419） | 无 | ✅ | 真键 |
| `enforce_path_inversion` | `True`（S:420） | 无 | ⚠️ | 真键；但**他们的 BT 里根本没有 `SmoothPath` 节点**（BT1:4-40、BTTP:4-45 只有 ComputePath*/RemovePassedGoals/FollowPath/ClearEntireCostmap）⇒ **整个 smoother_server 配了但从不被调用**（除了 RViz 手动调服务）。抄了也只是「多一个可用服务」 |

### A.9 velocity_smoother

他们 = S:471-483；我们 R:395-408。

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `smoothing_frequency` | `30.0`（S:473） | `20.0`（R:398） | ⚠️ | 真键（`nav2_velocity_smoother/src/velocity_smoother.cpp:55`，默认 20.0） |
| `scale_velocities` | `False`（S:474） | `False`（R:399） | ✅ | 一致 |
| `feedback` | `OPEN_LOOP`（S:475） | `OPEN_LOOP`（R:400） | ✅ | **一致**；我们更旧的 `nav2_params.yaml:362` 是 `CLOSED_LOOP`，说明已演进到 OPEN_LOOP |
| `max_velocity` | `[2.5, 2.5, 2.5]`（S:476） | `[2.0, 2.0, 3.0]`（R:401） | ⚠️ | 真键。注意他们 `wz` 给 2.5 而 MPPI `wz_max 1.5`（S:101）→ **两处不一致**（注释却写「与 MPPI 限速一致」） |
| `min_velocity` | `[-2.5, -2.5, -2.5]`（S:477） | `[-2.0, -2.0, -3.0]`（R:402） | ⚠️ | 同上 |
| `deadband_velocity` | `[0.05, 0.05, 0.05]`（S:478） | `[0.0, 0.0, 0.0]`（R:403） | ⚠️ | 他们 0.05 会把 5 cm/s 以下的指令抹平（对 2.5 m/s 车无感）；我们 0 是为了低速精调（sim 0.5 m/s）⇒ **不要抄** |
| `velocity_timeout` | `1.0`（S:479） | `1.0`（R:404） | ✅ | 一致 |
| `max_accel` | `[2.5, 2.5, 1.5]`（S:480，注释「与 MPPI 加速度一致」） | `[4.0, 4.0, 6.0]`（R:405） | ⚠️ | **注释是错的**：MPPI 1.1.20 没有加速度参数（见 A.3 `ax_max` 行）。所以**这里才是唯一真正限加速度的地方**。他们的值对我们 0.5 m/s 车偏松，我们 4/4/6 偏激进 |
| `max_decel` | `[-3.5, -3.5, -3.0]`（S:481） | `[-4.0, -4.0, -6.0]`（R:406） | ⚠️ | 同上 |
| `odom_topic` | `"Odometry"`（S:482） | `"odom"`（R:407） | ✅ | 键名相同（`velocity_smoother.cpp:93`）；我们值已统一到 `/odom`（LAUNCH:256） |
| `odom_duration` | `0.1`（S:483） | `0.1`（R:408） | ✅ | 一致 |

> 我们 `LAUNCH:551-562` → `BP:130-140` → `NAV:210-220` 的 remap 链与他们 `NL:173-183` **逐字相同**（`cmd_vel→cmd_vel_nav`、`cmd_vel_smoothed→cmd_vel`），所以 `velocity_smoother` 的输出落在 `/cmd_vel`，正好是 FVT 的输入（FVT:11/31-32、FVTc:17）。这一层两边同构，**可直接沿用**。

### A.10 bt_navigator（+ BT XML）

他们 = S:1-62；我们 R:61-100。

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `global_frame` / `robot_base_frame` | `map` / `base_link_fake`（S:4-5） | `map` / `base_link_fake`（R:64-65） | ✅ | 一致 |
| `odom_topic` | `Odometry`（S:6） | `/Odometry`（R:66） | ⚠️ | 真键（`nav2_bt_navigator/src/bt_navigator.cpp:98`，只用于 `OdomSmoother`，`:135`）。我们 `/Odometry` 在 T2 之后**没有发布者**（LAUNCH:256 把它 remap 成 `/odom`）→ BT 的测速一直为 0。建议改 `odom` |
| `bt_loop_duration` | `10`（S:7） | `10`（R:67） | ✅ | 真键（`N2/bta:57-58`，默认 10 ms）；一致 |
| `default_server_timeout` | 无（默认 20） | `20`（R:68） | ✅ | 真键（`N2/bta:60-61`）。他们靠默认值，我们显式写，等价 |
| `wait_for_service_timeout` | `1000`（S:9） | 无（默认 1000） | ✅ | 真键（`N2/bta:66-67`） |
| **`timeout: 100`** | S:8 | — | ❌ | **1.1.20 无此键**（`N2/bta:57-67` 只有 `bt_loop_duration`/`default_server_timeout`/`wait_for_service_timeout`），静默失效 |
| `default_nav_to_pose_bt_xml` | `$(find-pkg-share cod_bringup)/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`（S:11） | 无 → 走 nav2 内置 `navigate_to_pose_w_replanning_and_recovery.xml`（`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/…`） | ⚠️ | 见下方 BT 差异表 |
| `default_nav_through_poses_bt_xml` | `.../cod_bringup/.../navigate_through_poses_w_replanning_and_recovery.xml`（S:12） | 无 → 走 nav2 内置同名树 | ⚠️ | 同上 |
| `plugin_lib_names`（长列表） | S:15-62（含 `nav2_smooth_path_action_bt_node`（S:18）、`nav2_goal_updater_node_bt_node`、`nav2_path_longer_on_approach_bt_node`（S:56）等） | R:69-100（无 `nav2_smooth_path_action_bt_node`、无 `nav2_goal_updated_controller_bt_node`、无 `nav2_path_longer_on_approach_bt_node`、无 `nav2_is_battery_charging_condition_bt_node`） | ⚠️ | 我们的列表够跑内置 `navigate_to_pose/through_poses`（含 `nav2_compute_path_through_poses_action_bt_node` R:71、`nav2_remove_passed_goals_action_bt_node` R:97、`nav2_navigate_through_poses_action_bt_node` R:95）⇒ **多点/through-poses 我们现成可用**；缺的只是 `SmoothPath`、`PathLongerOnApproach`、`GoalUpdatedController`、电池条件这几个我们树里用不到的 |
| `ros__parameters` 之外的 `robot_state_publisher` 段 | S:456-458 | — | ❌ | 两个 launch 都不起 `robot_state_publisher`（SL:34-146、NL:111-193 无此节点）⇒ 惰性段 |
| `waypoint_follower` 段 | S:460-469（`loop_rate 20`、`stop_on_failure false`、`wait_at_waypoint`、`waypoint_pause_duration 200`） | R/D/T **无此段**；但旧文件 `src/nav/rm_navigation/rm_navigation/params/nav2_params.yaml:347-355` **逐字相同**（含 200） | ⚠️ | 键名全真（`N2/wpf:37-42`、`N2/waw:46-56`）。**但 200 的单位是毫秒**（`N2/waw:78-81` `sleep_for(milliseconds(...))`）⇒ 只有 0.2 s，不是「停 2 秒」。见 §D |

**BT 树差异（他们 vs 内置 = 我们）**

| 项 | 他们 BT1 / BTTP | nav2 内置（= 我们） | 判定 | 理由 |
|---|---|---|---|---|
| 重规划率 | 3.0 Hz（BT1:8 / BTTP:8） | 1.0 Hz（to_pose）/ 0.333 Hz（through_poses） | ⚠️ | 3 Hz 配 2.5 m/s；我们 0.5 m/s 用 1 Hz 足够，且我们全局 costmap 只有 5 Hz（R:241）⇒ 抄 3 Hz 会白算 |
| `RecoveryNode number_of_retries` | `10`（BT1:6 / BTTP:6） | `6` | ⚠️ | 10 次配合 `failure_tolerance 0.3`（S:71）会长时间卡住 |
| `FollowPath` retries | `10`（BT1:19 / BTTP:24） | `1` | ⚠️ | 他们靠 10 次重试 + 清 costmap 硬扛；我们 1 次 |
| 恢复动作 | 只有「清两张 costmap」+（BTTP）`BackUp dist 1.0 / speed 1.0`（BTTP:40-41） | 清 costmap + `Spin 1.57` + `Wait 5` + `BackUp 0.30 / 0.05` | ❌（结构） | **他们删掉了 `Spin` 和 `Wait`**——合理：他们的 FVT 把 `angular.z` 强制换成 `spin_speed`（`FVTc:60`，launch 未传参 ⇒ 0.0，`third_party/cod_nav_2026/src/fake_vel_transform/launch/fake_vel_transform_launch.py:9-18`）⇒ Spin 行为根本转不动。**我们 FVT 有修复（`FVT:80-84`：`spin_speed==0` 时角速度直通）**，所以我们的 Spin 是有效的，**不能照抄他们的树**（会把我们可用的恢复手段删掉）；他们的 `BackUp 1.0 m/s` 在我们 sim + 0.40 半径下也偏暴力 |
| `RemovePassedGoals radius` | `0.7`（BTTP:14） | `0.7` | ✅ | 一致 |

### A.11 behavior_server

他们 = S:422-454；我们 R:364-388。

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `local_costmap_topic` / `global_costmap_topic` | `local_costmap/costmap_raw` / `global_costmap/costmap_raw`（S:425-426） | — | ❌ | **1.1.20 `nav2_behaviors` 无这两个键**（grep `"local_costmap_topic"` 于 `third_party/nav2/nav2_behaviors` 无命中；1.1.20 只有 `costmap_topic`/`footprint_topic`，`N2/beh:34-39`）。这是 Jazzy 重构后的键名 |
| `local_footprint_topic` / `global_footprint_topic` | S:427-428 | — | ❌ | 同上，无此键 |
| `costmap_topic` / `footprint_topic` | 他们**没写** | `local_costmap/costmap_raw` / `local_costmap/published_footprint`（R:369-370） | ✅ | **这才是 1.1.20 的键**（`N2/beh:34-39`、`:81-82`）。我们的才是对的 |
| `cycle_frequency` | `10.0`（S:429） | `10.0`（R:371） | ✅ | 一致（`timed_behavior.hpp:119`） |
| `behavior_plugins` | `["spin","backup","drive_on_heading","assisted_teleop","wait"]`（S:430） | `["spin","backup","drive_on_heading","wait"]`（R:372） | ⚠️ | 我们少 `assisted_teleop`；`nav2_behaviors/AssistedTeleop` 在 1.1.20 存在（`third_party/nav2/nav2_behaviors/behavior_plugin.xml:27`）。可加，非必需 |
| `backup.plugin` | `pb_nav2_behaviors/BackUpFreeSpace`（S:434） | `nav2_behaviors/BackUp`（R:376） | ❌ | 需要额外编译 `third_party/cod_nav_2026/src/pb_nav2_plugins`（`behavior_plugin.xml`），且它依赖 `nav2_msgs/srv/GetCostmap`（`back_up_free_space.cpp:39`，服务由 `N2` 的 `costmap_2d_publisher.cpp:81-82` 提供）。我们没这个包，抄了会 pluginlib 加载失败 |
| `spin/backup/drive_on_heading/wait/assisted_teleop.plugin`（其余） | S:431-440 | R:373-380 | ✅ | 与 1.1.20 的插件名逐字一致 |
| `local_frame` | `odom`（S:441） | — | ❌ | 1.1.20 behavior 只读 `global_frame`/`robot_base_frame`/`transform_tolerance`（`timed_behavior.hpp:119-122`）⇒ `local_frame` 无效 |
| `global_frame` | `map`（S:442） | `odom`（R:381） | ⚠️ | **两边不同且都有道理**：他们 map（配静态 map→odom）；我们 odom（配 amcl/icp 等重定位，避免恢复动作被定位跳变影响）。改成 map 会让恢复动作对定位跳变敏感 |
| `robot_base_frame` | `base_link_fake`（S:443） | `base_link_fake`（R:382） | ✅ | 一致 |
| `transform_tolerance` | `0.1`（S:444） | `0.1`（R:383） | ✅ | 一致 |
| `simulate_ahead_time` | `2.0`（S:445） | `1.0`（R:385） | ⚠️ | 真键（`N2/spin:54-56`、`drive_on_heading.hpp:233-236`）。2.0 会让 Spin/BackUp 预演更久，更保守但更慢 |
| `max_rotational_vel` | `1.0`（S:446） | `3.0`（R:386） | ⚠️ | 真键（`N2/spin:59-61`，默认 1.0）。我们 3.0 是刻意放大（R:364-368 注释提到曾被错段名吃掉过） |
| `min_rotational_vel` | `0.4`（S:447） | `0.4`（R:387） | ✅ | 一致（`N2/spin:65-68`） |
| `rotational_acc_lim` | `3.2`（S:448） | `3.0`（R:388） | ✅ | 真键（`N2/spin:70-72`），默认 3.2 |
| `robot_radius`（给 BackUpFreeSpace） | `0.1`（S:450） | — | ❌ | `back_up_free_space.cpp` 只 `get_parameter` 了 `global_frame/max_radius/service_name/visualize`（`:28-37`），**没读 `robot_radius`** ⇒ 死键 |
| `max_radius` | `3.5`（S:451） | — | ❌ | 仅 `pb_nav2_behaviors/BackUpFreeSpace` 读（`:29/35/88`），我们没这插件 |
| `service_name` | `global_costmap/get_costmap`（S:452） | — | ❌ | 同上；服务本身存在（`costmap_2d_publisher.cpp:81-82`，在 costmap 节点命名空间下 ⇒ `/global_costmap/get_costmap`） |
| `free_threshold` | `5`（S:453） | — | ❌ | `back_up_free_space.cpp` 未读取 ⇒ 死键 |
| `visualize` | `True`（S:454） | — | ❌ | 同上（真键但属该插件） |

### A.12 collision_monitor

| 项 | 他们 | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `collision_monitor:` 段 | **不存在**（`grep -rn collision_monitor S M NL ML LOC` 无命中） | 也不存在 | ✅（一致） | `ros-humble-nav2-collision-monitor 1.1.20` 已装，两边都没用。**这是一个双方都空着的槽位**：若要给 sim 加一层「速度级急停」，可以直接接（但需要新的 params 段 + `collision_monitor` 生命周期节点，属新增功能） |

### A.13 map_server / map_saver / 地图资产

| 键 | 他们 | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `map_server.yaml_filename`（S:360） | 硬编码 `/home/cod-sentry/dyx_ws/cod-sentry/src/cod_bringup/maps/rmul2026.yaml` | `""`（R:393，注释说明必须留键给 RewrittenYaml 替换） | ⚠️ | 他们的值**会被 `LOC:56-58` 的 `param_rewrites={'yaml_filename': map_yaml_file}` 覆写**（默认 `bringup_dir/maps/rmul2026.yaml`，`LOC:76-79`）⇒ 硬编码路径是噪音，但**他们的 `LOC` 用了 `root_key=namespace`**（LOC:63）而 `NL:68` 把 `root_key` **注释掉了** —— 这是我们和他们 launch 的一处真实差异（见 §F-9） |
| `map_saver.save_map_timeout` | `5.0`（S:365） | `5.0`（R:348） | ✅ | 一致 |
| `map_saver.free_thresh_default` | `0.25`（S:366） | `0.25`（R:349） | ✅ | 一致 |
| `map_saver.occupied_thresh_default` | `0.65`（S:367） | `0.65`（R:350） | ✅ | 一致 |
| `map_saver.map_subscribe_transient_local` | `True`（S:368） | `True`（R:351） | ✅ | 一致 |
| 地图 `resolution` | `0.050`（MAP:3） | 我们的图是 `map/RMUL2026.yaml`（LAUNCH:60）— 不同资产 | ⚠️ | 他们 0.05 m/px、255×177 px（`rmul2026.pgm` 头 `P5 255 177`）= **12.75 m × 8.85 m**；origin `[-1.756, -7.036, 0]`（MAP:4）⇒ x∈[-1.756, 10.994]、y∈[-7.036, 1.814]。**他们 `global_costmap` 的 25×25 m 滚动窗比地图大 2 倍**（S:269-270 vs 12.75×8.85）⇒ 图外未知区参与规划（见 A.6） |
| 地图 `mode/negate` | `trinary` / `0`（MAP:2/5） | — | ✅ | 常规 |
| 地图 `occupied_thresh` / `free_thresh` | `0.65` / **`0.196`**（MAP:6-7） | 我们图资产不同 | ⚠️ | `0.196` 与他们 `map_saver.free_thresh_default 0.25`（S:366）**不一致** ⇒ §F-10 |

### A.14 slam_toolbox（`SLAMc` vs `SLAMrm`）

他们的 `SLAMc` 只被 **ML:85-94** 用（多点导航）；`SL`（单点）**不起 slam_toolbox**，靠 `map_server` + 静态 `map→odom`。我们的 slam_toolbox 由 `LAUNCH:503-512`（mapping/slam_nav 且 `mapper:=slam_toolbox`）拉起。

| 键 | `SLAMc`（行=值） | `SLAMrm`（行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `mode` | `lifelong`（SLAMc:19） | `mapping`（SLAMrm:17） | ⚠️ | lifelong 会持续改写地图（配 `use_map_saver: true` SLAMc:18）；我们 sim 要的是可复现的 `mapping` ⇒ **不要抄** |
| `base_frame` | `base_link`（SLAMc:16） | `livox_frame`（SLAMrm:15） | ⚠️ | 我们刻意用雷达帧（`P2L` 的 `target_frame: ""` 意味着 `/scan` 的 frame 就是点云帧 `livox_frame`，P2L:9）；抄 `base_link` 会因缺 `base_link→livox_frame` 链路（我们 TF 里是 `livox_frame` 挂 `imu_link`）而报错 |
| `odom_frame` / `map_frame` / `scan_topic` | `odom` / `map` / `/scan`（SLAMc:14/15/17） | 同（SLAMrm:13/14/16） | ✅ | 一致 |
| `transform_publish_period` | `0.0`（SLAMc:30，注释「if 0 never publishes odometry」） | `0.02`（SLAMrm:28） | ❌ | 0.0 = **slam_toolbox 不发 `map→odom`**！这与他们 `ML:95-116` 那条静态 `map→odom` 搭配才成立 → 见 §F-11 |
| `map_update_interval` | `1.0`（SLAMc:31） | `5.0`（SLAMrm:29） | ⚠️ | 1 Hz 重算栅格对 sim CPU 不友好；我们 5 s 更稳 |
| `minimum_time_interval` | `0.3`（SLAMc:35） | `0.5`（SLAMrm:32） | ⚠️ | 我们 0.5 s 更省 |
| `min_laser_range` | `0.2`（SLAMc:33） | 无 | ⚠️ | 我们靠 `P2L.range_min: 0.05`（P2L:37）与 `inf_is_valid` 处理近距；抄 0.2 会把贴身回波丢掉（正是我们 2026-09-26 踩过的坑） |
| `max_laser_range` | `10.0`（SLAMc:34） | `20.0`（SLAMrm:31） | ⚠️ | 与 `P2L.range_max: 10.0`（P2L:38）一致性：他们的 10.0 与 p2l 对齐；我们 20.0 与 `range_max 10` 不一致（无害但语义含糊） |
| `minimum_travel_distance` / `minimum_travel_heading` | `1.0` / `0.1`（SLAMc:44-45） | `0.5` / `0.5`（SLAMrm:41-42） | ⚠️ | 影响图节点密度；我们 sim 0.5/0.5 更密 |
| `throttle_scans` / `use_scan_matching` / `do_loop_closing` / 求解器块 / 相关参数块 | SLAMc:29/42/51/5-10/57-76 | SLAMrm 同键同值 | ✅ | **除上表所列外逐字相同**（两边同源）⇒ 这部分可直接互换 |
| `use_map_saver` | `true`（SLAMc:18） | 无（默认 false） | ⚠️ | 只有 lifelong 模式有意义 |

### A.15 我们的 `D` / `T` 变体与他们的关系

`D` 与 `T` 相对 `R` **只差 `FollowPath` 段（+几行注释）**，其余（两张 costmap、planner、behavior、velocity_smoother、map_server）逐字相同：`diff -u R D` 只有 3 个 hunk（`@@ -124,38` FollowPath、`@@ -187,9`、`@@ -262,10`，后两个只是注释删除）。

- DWB（D:128-166）：`max_vel_x/y 2.5`（D:133-134）与他们的 MPPI `vx_max/vy_max 2.5`（S:99-100）**同量级**；`max_vel_theta 12.0`（D:135）远超他们 `wz_max 1.5`（S:101）——**DWB 的角速度上限在我们侧是空转的**（被 `velocity_smoother.max_velocity[2]=3.0`，R:401 截住）。
- TEB（T:129-165）：`max_vel_x 0.5`（T:144）、`footprint_model.circular.radius 0.2`（T:149-151）、`min_obstacle_dist 0.27`（T:152）、`weight_kinematics_nh 1000`（T:157）——**TEB 的 `footprint_model.radius 0.2` 与两张 costmap 的 `robot_radius 0.40` 不一致**（T:150 vs R:178/252），这是一个独立于 cod_nav 的现存不一致，接入 MPPI 前后都值得一并修（见 §F-13）。
- 他们没有任何等价于 DWB/TEB 的段：`FollowPath` 只有 wrapper+MPPI 一种。

### A.16 「1.1.20 里不存在」死键汇总（照抄会静默失效）

| 死键 | 出现位置 | 1.1.20 只在哪里存在 |
|---|---|---|
| `publish_critics_stats` | S:65 | 全 nav2 无 |
| `vy_min`, `wz_min` | S:104-105 | Jazzy/main 的 MPPIController |
| `ax_max`, `ay_max`, `az_max`, `ax_min`, `ay_min`, `az_min` | S:107-113 | 同上 |
| `CostCritic.trajectory_point_step` | S:159 | 只有 PathAlign/PathAlignLegacy critic 读（`N2/mppi/pac:34`）；CostCritic 的 `getParam` 清单里没有（`N2/mppi/cc:26-32`） |
| `path_length_tolerance` | S:83 | 新版 `PositionGoalChecker`（1.1.20 只有 `xy_goal_tolerance`/`stateful`，N2/ppgc:44-52） |
| `timeout`（bt_navigator） | S:8 | 无（1.1.20 用 `default_server_timeout`/`wait_for_service_timeout`，N2/bta:57-67） |
| `motion_model_for_search`, `angle_quantization_bins`, `analytic_expansion_ratio`, `analytic_expansion_max_length`, `minimum_turning_radius`, `retrospective_penalty`, `reverse_penalty`, `change_penalty`, `non_straight_penalty`, `cost_penalty`, `rotation_penalty`, `lookup_table_size`, `cache_obstacle_heuristic`, `allow_reverse_expansion`, `smooth_path` | S:385-399 | 只有 Hybrid/Lattice 计划器读（smac_planner_hybrid.cpp / node_lattice.cpp） |
| `local_costmap_topic`, `global_costmap_topic`, `local_footprint_topic`, `global_footprint_topic`, `local_frame` | S:425-428, 441 | Jazzy 版 behavior_server；1.1.20 用 `costmap_topic`/`footprint_topic`（`N2/beh:34-39`） |
| `robot_radius`, `free_threshold`（behavior_server 段） | S:450, 453 | pb 插件也没读（back_up_free_space.cpp:28-37） |
| `static_layer.enable` | S:285 | 1.1.20 `StaticLayer` 不读（只有部分图层有 `enabled`） |

---

## B. `M`（multiplenav2）相对 `S` 的差异清单

`diff -u S M` 的全部差异如下（**没有遗漏任何 hunk**）。

| # | 键 | S | M | 判定 | 为什么不同（多点/through-poses 视角） |
|---|---|---|---|---|---|
| B1 | `bt_navigator.default_nav_to_pose_bt_xml` | `cod_bringup` 自定义树（S:11） | nav2 内置树（M:10） | ⚠️ | 多点模式**回退到官方树**：官方树带 `Spin 1.57`+`Wait 5` 恢复，自定义树只有清理+BackUp。与 B2 的方向一致（多点模式不想让单点的激进改造影响连贯穿越） |
| B2 | `bt_navigator.default_nav_through_poses_bt_xml` | `cod_bringup` 自定义（S:12） | nav2 内置（M:11） | ⚠️ | **through-poses 才是多点的核心**：M 明确用官方 through-poses 树（`RemovePassedGoals 0.7` + `0.333 Hz` 重规划，见 `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml:10-14`） |
| B3 | `controller_server.publish_critics_stats` | `True`（S:65） | 删除 | ❌ | 死键，删了无影响（§A.16） |
| B4 | goal checker 插件 | `PositionGoalChecker`（S:81） | `SimpleGoalChecker`（M:77） | ⚠️ | 见下两行 |
| B5 | `general_goal_checker.stateful` | `False`（S:80） | `True`（M:76） | ⚠️ | through-poses 下车要走「一串点」，stateful 锁存可避免在通过中途因抖动反复判定 |
| B6 | `general_goal_checker.xy_goal_tolerance` | `0.2`（S:82） | `0.30`（M:78，注释写「放宽到25cm」） | ⚠️ | 巡逻点不需要 0.2 m 精度；**注释与数值矛盾**（§F-2） |
| B7 | `general_goal_checker.yaw_goal_tolerance` | 无（PositionGoalChecker 无此键） | `6.28`（M:79） | ⚠️ | 6.28 rad ≈ 360° ⇒ **等价于忽略朝向**，但用的是 SimpleGoalChecker（有该键）。效果与 PositionGoalChecker 相同，可能是为了绕开 PositionGoalChecker 在 through-poses 下与 `path_length_tolerance`（死键）的混淆 |
| B8 | `FollowPath.direct_approach_distance` | `2.0`（S:89） | **删除** → 取 GAC:46 默认 **0.5** | ⚠️ | **多点模式主动关掉「近距离直接驱动」**：连续穿越时若末端强插一段直冲，会与 MPPI 的路径跟踪打架；单点任务才需要防画弧 |
| B9 | `FollowPath.direct_approach_kp` | `3.0`（S:90） | **删除** → 取 GAC:49 默认 **1.0** | ⚠️ | 同上；给默认 1.0 时 `speed = min(0.2, dist*1.0)`，0.2 m/s 上限仍在（GAC:110） |
| B10 | `FollowPath.model_dt` | `0.02`（S:92） | `0.06`（M:86，同一句注释） | ⚠️ | 时域从 **1.6 s → 4.8 s**（80×0.06）。注释「配合 time_steps 延长预测时域」只在 M 成立（§F-1）。代价：周期 0.02 < 0.06 ⇒ 只 WARN（N2/mppi/opt:100-103） |
| B11 | `FollowPath.vx_std` / `vy_std` | `0.3 / 0.3`（S:95-96） | `0.35 / 0.35`（M:89-90） | ⚠️ | 连续穿越需要更大多样性 |
| B12 | `FollowPath.az_max` | `1.0`（S:109） | `3.0`（M:103） | ❌ | 死键（§A.16）；两边都无效 |
| B13 | `FollowPath.retry_attempt_limit` / `regenerate_noises` | 无（默认 1 / false） | `2` / `true`（M:117-118） | ⚠️ | 长路径跟踪更容易出现优化失败，所以提高重试 + 每次重采样噪声 |
| B14 | `GoalCritic.cost_weight` / `threshold_to_consider` | `25.0` / `4.0`（S:139-140） | `20.0` / `3.0`（M:132-133） | ⚠️ | 多点不需要那么强的「末端吸引」，改为更早让 GoalCritic 退出、PathFollow 主导 |
| B15 | `ObstaclesCritic.collision_margin_distance` / `near_goal_distance` | `0.2` / `0.5`（S:148-149） | `0.25` / `0.1`（M:142-143） | ⚠️ | 更宽的安全余量 + 更小的「近目标豁免区」（巡逻点常在障碍附近） |
| B16 | `PathFollowCritic.cost_weight` / `threshold_to_consider` | `15.0` / `1.0`（S:164-166） | `8.0` / `3.0`（M:158/160） | ⚠️ | 权重降一半：多点靠「一直跟着路径」，不靠末端爆发 |
| B17 | `PathAlignCritic.threshold_to_consider` | `0.8`（S:174） | `3.0`（M:168） | ⚠️ | 与 B16 同步（两个 threshold 都是 3.0），让路径对齐一直生效到接近目标 |
| B18 | `local_costmap.robot_radius` / `footprint` | `0.1` / `±0.1`（S:190-191） | `0.2` / `±0.15`（M:184-185） | ⚠️ | 多点模式保守化。**但 footprint 非空 ⇒ 有效内切半径 = 0.15（不是 0.2）**，`robot_radius` 被忽略（N2/cmros:408-416） |
| B19 | `local_costmap.inflation_layer.inflation_radius` | `0.55`（S:199） | `0.75`（M:193） | ⚠️ | 与 B18 配套加宽 |
| B20 | `local_costmap.stvl.voxel_decay` / `observation_persistence` | `0.5` / `0.0`（S:203/207） | `1.0` / `0.5`（M:197/201） | ⚠️ | 连续穿越多点，需要更久保留障碍航迹（多点巡逻路径反复经过同一区） |
| B21 | `local/global stvl.livox_source.min_obstacle_height` | `0.15`（S:229/319） | `0.1`（M:223/313） | ⚠️ | 更激进地看到矮障碍（多点巡逻会贴场地元素） |
| B22 | `realsense_source.min_z` / `min_obstacle_height` | `0.15` / `0.18`（S:250/252、340/342） | `0.1` / `0.1`（M:244/246、334/336） | ❌（对我们） | 我们无 D435i |
| B23 | `local/global always_send_full_costmap` | `False`（S:263/353） | `True`（M:257/347） | ⚠️ | 多点/through-poses 需要更频繁拿到完整图（我们本来就是 True，见 A.5） |
| B24 | `global_costmap.width` / `height` | `25/25`（S:269-270） | `50/50`（M:263-264） | ⚠️ | 多点巡逻覆盖更大范围；50 m×0.04 = **1250×1250 格**（约为主图的 15 倍面积） |
| B25 | `global_costmap.inflation_layer.cost_scaling_factor` | `5.0`（S:288） | `15.0`（M:282，注释仍写「与 local_costmap 一致」，而 M local 是 5.0） | ⚠️ | 全局代价衰减更陡 ⇒ 全局路径更贴墙、更短；**注释与数值矛盾**（§F-3） |
| B26 | `global_costmap.robot_radius` / `footprint` | `0.1` / `±0.1`（S:277-278） | `0.2` / `±0.15`（M:271-272） | ⚠️ | 同 B18 |
| B27 | `map_server.yaml_filename` | 硬编码绝对路径（S:360） | `""`（M:354） | ✅ | **M 的写法与我们一致**（R:393）⇒ 若要抄这段，抄 M 的 |
| B28 | `behavior_server.use_sim_time` | `false`（S:424） | `true`（M:418） | ❌ | 与全文其它 `false` 矛盾，且会被 `NL:61-71` 的 RewrittenYaml 用 launch 参数（"false"）覆写 ⇒ **惰性**（§F-7） |
| B29 | `behavior_server.robot_radius`（pb 插件） | `0.1`（S:450） | `0.2`（M:444） | ❌ | 死键（该插件不读，§A.11） |
| B30 | `velocity_smoother.max_accel` | `[2.5, 2.5, 1.5]`（S:480） | `[2.5, 2.5, 3.0]`（M:474） | ⚠️ | 与 B12（`az_max 3.0`）同一意图：多点需要更快的偏航加速。但 MPPI 侧无效，真正生效的是这里 |
| B31 | 文件末尾换行 | 有 | **无** | — | 纯格式 |

**B 小结（对多点/through-poses 的真正含义）**：M 相对 S 的改动可归纳成四条主线——①**关掉末端直接驱动**（B8/B9）让穿越连续；②**把「路径跟随」权重体系整体换成「长时间跟线」**（B14/B16/B17）；③**几何/障碍保守化并放大全局窗**（B18-B26）；④**goal checker 从「只判位置」换成「判位置 + 忽略朝向」**（B4-B7）。这与「单点要防画弧、多点要顺滑穿越」的工程直觉一致。

---

## C. `goal_approach_controller` 解读

### C.1 它是什么、怎么代理 MPPI

| 项 | 内容 | 证据 |
|---|---|---|
| 类型 | `nav2_core::Controller` 的 **pluginlib 插件**（wrapper/装饰器），不是独立节点 | GAC:20-24、GAC:151-153；GACX:1-9 |
| 导出 | `<library path="goal_approach_controller">` + `<class name="goal_approach_controller::GoalApproachController" base_class_type="nav2_core::Controller">` | GACX:1-9 |
| 包依赖 | `rclcpp, rclcpp_lifecycle, geometry_msgs, nav_msgs, nav2_core, nav2_util, nav2_costmap_2d, pluginlib, tf2_ros` | GACC:7-16、package.xml:12-20 |
| 内部控制器加载 | 运行时用 `pluginlib::ClassLoader<nav2_core::Controller>("nav2_core","nav2_core::Controller")` + `createUniqueInstance(inner_plugin_type)`，再 `inner_controller_->configure(parent, name, tf, costmap_ros)` | GAC:60-63 |
| **参数命名空间** | `configure(parent, name, …)` 里 `name` **原样透传**给内部 MPPI ⇒ MPPI 的所有参数与 wrapper 参数**同级**在 `FollowPath.*` 下 | GAC:37-57、:63；S:84-176 正是这种扁平写法 |
| 生命周期 | `cleanup/activate/deactivate` 直接转调内部（无自身状态需要清） | GAC:73-86 |
| 速度上限 | `setSpeedLimit` 直接转调内部（wrapper 不额外限速） | GAC:133-136 |
| 头文件/安装 | 仓库里**没有 `include/` 目录**（`GACC:33-36` 指向不存在的 `include`，无害）；`.cpp` 是唯一源文件 | `ls third_party/cod_nav_2026/src/goal_approach_controller/` → 只有 `src/` |
| `on_configure` 里无 try/catch | 内部插件类型写错会直接抛到 controller_server | GAC:62 |

### C.2 接口：订阅/发布什么

**它自己不订阅也不发布任何话题**（`GAC` 全文无 `create_subscription`/`create_publisher`）。它是 Nav2 controller 插件，靠 **Nav2 的 C++ 调用链**工作：

| 方向 | 接口 | 时机 | 证据 |
|---|---|---|---|
| ← 输入 | `setPlan(const nav_msgs::msg::Path&)` | controller_server 每次收到新路径（`N2/ctrl:445-453`） | GAC:88-94 |
| ← 输入 | `computeVelocityCommands(pose, velocity, goal_checker)` | 每个控制周期（`N2/ctrl:470-485`） | GAC:96-101 |
| → 输出 | `geometry_msgs::msg::TwistStamped`（返回给 controller_server，再由其发到 `cmd_vel`→remap `cmd_vel_nav`） | 每周期 | GAC:130；`NL:122`/`NAV:159` |
| → 内部副作用 | 内部 MPPI 自己的可视化/轨迹发布（`/trajectories`、`transformed_global_plan`、`visualize`） | 每周期 | `N2/mppi/tv:29-30`；S:121 `visualize: false` |

**关键实现细节（也是第一处硬伤）**：

```cpp
// GAC:88-94
void setPlan(const nav_msgs::msg::Path & path) override {
  if (!path.poses.empty()) { goal_ = path.poses.back(); }   // ← 空路径时保留【上一次】的目标（陈旧）
  inner_controller_->setPlan(path);
}
// GAC:103-106
double dx = goal_.pose.position.x - pose.pose.position.x;
double dy = goal_.pose.position.y - pose.pose.position.y;
```

- `goal_` 来自**全局规划器给的路径末点 ⇒ `map` 帧**（我们的 planner 的 `global_frame: map`，R:243）。
- `pose` 来自 `N2/ctrl:611-617` → `costmap_ros_->getRobotPose()` → 局部 costmap 的 `global_frame` = **`odom`**（R:161；`N2/cmros:639-644`）。
- ⇒ `dx/dy` **在混帧相减**。MPPI 自己不会犯这个错：它用 `N2/mppi/ph:69-98,145-161` 把路径转到 costmap 帧。wrapper 没有这一步。
- 危害量级：等于 `map→odom` 的旋转。他们 `SL:81-82` 的静态桥是 **yaw = −0.5 rad（−28.6°）** ⇒ 单点导航最后 2 m 的驱动方向**偏 28.6°**；他们 `ML:109-110` 是 **yaw = 0.0** ⇒ 多点模式反而没这个误差。我们自己 `mode=nav, localization=''` 的回退静态桥是 `camera_init→map`/`body→odom` 单位旋转（LAUNCH:452-468），若用 amcl/icp/slam_toolbox/cartographer，则 `map→odom` 会**漂移**，误差是变化的，比固定 0.5 rad 更糟。
- 次要：`setPlan` 在空路径时不清 `goal_`；`goal_checkers_[…]->reset()`（`N2/ctrl:457`）不影响 wrapper 的 `goal_`。

### C.3 参数表（wrapper 自有 5 个 + 建议值）

| 参数 | 默认值（声明处） | 他们（S 行=值） | 含义 | 对 sim 的建议 |
|---|---|---|---|---|
| `inner_plugin` | `nav2_mppi_controller::MPPIController`（GAC:36-38） | `nav2_mppi_controller::MPPIController`（S:86） | 被装饰的控制器插件名 | 保持；也可换 `dwb_core::DWBLocalPlanner` 做 A/B |
| `approach_distance` | `1.5`（GAC:39-41） | `2.5`（S:87） | 距目标小于此值时**按比例钳位合速度**到 `approach_velocity` | 0.8–1.5（我们 0.5 m/s 巡航，2.5 m 太早减速） |
| `approach_velocity` | `0.5`（GAC:42-44） | `0.2`（S:88） | 钳位后的合速度上限（m/s） | 0.15–0.2（配 `xy_goal_tolerance 0.25`，R:124） |
| `direct_approach_distance` | `0.5`（GAC:45-47） | `2.0`（S:89）；**M 删除 ⇒ 0.5** | 距目标小于此值时**绕过内部控制器**，直接朝目标平移 | 0.3–0.5（见 C.4：「直接驱动」与我们的 spin 语义冲突，越小越好） |
| `direct_approach_kp` | `1.0`（GAC:48-50） | `3.0`（S:90）；M 用默认 1.0 | 直接驱动段的 P 增益 | 1.0–2.0；注意 `speed = min(approach_velocity, dist*kp)`（GAC:110），kp≥3 且 dist≥0.067 时**恒等于** `approach_velocity`，增益实际失效 |

> 声明方式 `nav2_util::declare_parameter_if_not_declared`（GAC:36-50）⇒ 这些键**可以**写在 `FollowPath.` 下（S:87-90 就是这么写的），不写就是上表默认。

### C.4 `direct_approach_*` 分支与 `angular.z = 0` 的确切行为

```cpp
// GAC:108-128
if (dist < direct_approach_distance_) {            // ① 直接驱动分支
  double target_speed = std::min(approach_velocity_, dist * direct_approach_kp_);
  if (dist > 0.01) {
    cmd.twist.linear.x = target_speed * (dx / dist);   // ② 把【map 帧的方向余弦】直接当成【底盘 x 指令】
    cmd.twist.linear.y = target_speed * (dy / dist);
  } else { cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; }
  cmd.twist.angular.z = 0.0;                       // ③ 角速度强制清零
} else if (dist < approach_distance_) {            // ④ 仅限速分支
  double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
  if (speed > approach_velocity_) {
    double scale = approach_velocity_ / speed;
    cmd.twist.linear.x *= scale; cmd.twist.linear.y *= scale;
    cmd.twist.angular.z *= scale;                  // ⑤ 角速度跟着等比缩小
  }
}
```

**逐条含义**：

1. **① 进入条件**：只与「路径末点」的欧氏 XY 距离有关（GAC:104-108），与朝向、速度、路径形状都无关。
2. **② 方向**：直接把 `dx/dist, dy/dist` 塞进 `Twist.linear.x/y`。Nav2 里 `Twist` 是**底盘坐标系**（`base_link_fake`）下的指令 ⇒ 这段代码等价于「假设底盘朝向与 `odom` 一致」。在他们的云台小陀螺架构下**恰好近似成立**（他们 FVT 把 `base_link_fake` 的 yaw 恒定成 `-base_link_yaw`，`FVTc:44-52`，使 `base_link_fake` 在 odom 里近似不转）；在我们 `FVT` 的实现里 `base_link_fake` 的 yaw = `current_angle_`（`FVT:105-115`），而 `current_angle_` 来自 `/local_plan`（`FVT:35-37`、`FVT:46-67`）——**详见 C.5**。
3. **③ `angular.z = 0.0` 的确切行为**：wrapper **单方面把角速度写成 0**（GAC:118），并且是在**内部 MPPI 已经算完之后覆盖**（GAC:101 → GAC:118）。后果分三层：
   - 对 **MPPI 的求解**没有任何影响（MPPI 已经返回了含 `angular.z` 的指令，只是被丢弃）；
   - 对 **底盘**的影响取决于下游 FVT：他们 `FVTc:60` 无条件 `angular.z = spin_speed_`（launch 未传参 ⇒ 0.0，见 `third_party/cod_nav_2026/src/fake_vel_transform/launch/fake_vel_transform_launch.py:9-18`）⇒ **底盘角速度恒为 0**，机器人靠 vx/vy 平移到位，**终点朝向完全不受控**；
   - 对**我们**：`FVT:94` 是 `aft_tf_vel.angular.z = (msg->angular.z != 0) ? spin_speed_ : 0;` ⇒ `angular.z = 0` 走进「0」分支 ⇒ **底盘停止自转**（且 `spin_speed` 非 0 时，这一刻从「小陀螺」变成「不转」）。同时 `FVT:91-96` 仍会用 `-current_angle_` 旋转线速度 ⇒ **线速度方向取决于 `/local_plan`，而 `/local_plan` 在我们栈里没有任何发布者**（见 C.5）。
4. **④/⑤ 限速分支**：只按**合速度**等比缩小，方向不变；`angular.z` 也乘同一个 scale（GAC:120-127）。所以他们 `approach_distance 2.5 / approach_velocity 0.2` 的效果是「距目标 2.5 m 内合速度被压到 ≤0.2 m/s」——**这会让最后 2.5 m 变得非常慢**（2.5 m ÷ 0.2 m/s ≈ 12.5 s），而 `direct_approach_distance 2.0` 又在这段里覆盖了它 ⇒ 两个分支在 2.0 m 内**同时**满足，代码按 `if/else if` 只走 ①，所以实际是「2.5→2.0 m 之间限速，<2.0 m 直接驱动」。
5. **无 `dist` 最小值保护**：`dist ≤ 0.01` 时停车（GAC:114-117），但**仅在 `< direct_approach_distance_` 时**；若 `direct_approach_distance` 被设成 0（有人会这么干），整个直接分支永不触发，只剩限速分支 ⇒ 行为完全不同。
6. **不做 TF 查询、也不做失败处理**：`GAC:104-106` 是纯算术 ⇒ 即使 TF 断了也会照样发指令（可能方向完全错），没有 `PlannerException`。

### C.5 与我们 `fake_vel_transform` / spin 模式的兼容性

先给事实基线（两边 FVT 实现不同）：

| 维度 | 他们 `FVTc` | 我们 `FVT` |
|---|---|---|
| `spin_speed` 默认/实际 | `0.0`（`FVTc:19`），launch 不传参（`fake_vel_transform_launch.py:9-18`） | 包内默认 `-6.0`（`FVT:22`），但**实际参数文件是 `5.0`**（`FVTC:3`），launch 也默认 `5.0`（`LAUNCH:114`），并把 `spin_speed` 当 launch 参数覆盖（`LAUNCH:417-419`） |
| TF `robot_base→fake` | 在 **odom 回调**里用 `-yaw(odom)`（`FVTc:40-52`）⇒ fake 帧在 odom 里近似**恒定朝向** | 在 **20 Hz 定时器**里用 `current_angle_`（`FVT:40-42`、`FVT:105-115`），`current_angle_` = `teb_angle - base_link_angle_`，来自 **`/local_plan` 路径**（`FVT:46-67`） |
| 角速度处理 | **无条件** `angular.z = spin_speed_`（`FVTc:60`） | `spin_speed_ == 0` ⇒ **原样直通**（`FVT:80-84`，你们的修复）；否则 `angular.z = (angular.z != 0) ? spin_speed_ : 0`（`FVT:94`） |
| 线速度旋转 | 按 `current_robot_base_angle_`（odom 里的真实底盘角）旋转（`FVTc:57-63`） | 按 `-current_angle_` 旋转（`FVT:91-98`） |
| 输入/输出话题 | `cmd_vel` → `aft_cmd_vel`（`FVTc:17-18`） | `/cmd_vel` → `/cmd_vel_chassis`（`FVT:11-12`） |
| `/local_plan` 输入 | 不订阅 | 订阅（`FVT:13`、`FVT:35-37`）——**但 1.1.20 的 nav2 没有任何节点发布 `local_plan`**（`grep -rn local_plan third_party/nav2/nav2_controller/src` 无命中；对已安装 `libmppi_controller.so` 取字符串也没有）。发布者是 cod_nav 里的 **`pb_omni_pid_pursuit_controller`**（`third_party/cod_nav_2026/src/pb_omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp:164`），而**我们没用那个控制器**（R:129/D:129/T:130 分别是 RPP/DWB/TEB） |

**兼容性矩阵（我们现有栈）**：

| 场景 | `angular.z` 进 FVT | 我们 FVT 的行为 | 对 goal_approach_controller 的后果 |
|---|---|---|---|
| `spin_speed:=0.0`（`LAUNCH:114`/`:415-419` 可覆盖） | `0` | 直通模式（`FVT:80-84`）：角速度 0、线速度**不再旋转**、TF 恒等 ⇒ `base_link_fake ≡ base_link` | ✅ **这是唯一干净的组合**：direct-approach 的 `angular.z=0` 与「小陀螺关掉」自洽；但代价是**整个导航栈没有自旋**，纯靠 omni 平移（我们 RPP 默认 `use_rotate_to_heading: true`，R:146 ⇒ 会先转朝向再走，转了也没意义，等于白等） |
| `spin_speed:=5.0`（默认，`FVTC:3`） | `0` | 走 `(angular.z != 0) ? spin_speed_ : 0` 的 **0 分支** ⇒ **底盘停止自转**；同时线速度按 `-current_angle_` 旋转，而 `current_angle_` **恒为 0**（因为 `/local_plan` 没人发）⇒ 实际是「直通线速度 + 角速度 0」 | ⚠️ 「小陀螺」在末端 2 m 内被**关掉**。若我们真的想要小陀螺行为，这与 §C.4③ 直接冲突；若我们只想让导航跑通，结果与 `spin_speed:=0` 等价，但**其它阶段**（>2 m）角速度非 0 时又会被替换成 5.0 rad/s ⇒ **同一次导航里角速度语义前后不一致**（2 m 外是 5 rad/s，2 m 内是 0） |
| `spin_speed:=0.0` 且 `direct_approach_distance: 0`（禁用直接分支） | MPPI 的原始 `angular.z` | 角速度直通（`FVT:80-84`） | ✅ 兼容，但就**放弃了 wrapper 的核心功能**，只剩限速分支 |

**结论**：`angular.z = 0` 在**他们**的栈里是「无所谓」（角速度反正被 `FVTc:60` 覆盖成 `spin_speed`）；在**我们**的栈里是一个**语义开关**——它会关掉小陀螺、并让我们 FVT 里那段依赖 `/local_plan` 的线速度旋转逻辑退化成空转（因为 `current_angle_` 本来就不更新）。所以：

- 若我们要用 `goal_approach_controller`，**推荐 `spin_speed:=0.0` + 把 `direct_approach_distance` 收到 0.3–0.5**（既保住「末端防画弧」，又避免「>2 m 与 <2 m 角速度语义跳变」）。
- 若我们要保住小陀螺 + 朝向控制，则**必须改 wrapper**（把 `angular.z = 0.0` 换成「角速度交给内部控制器」或加开关），否则小陀螺在末端被静默关掉。

### C.6 能否直接用：结论

| 维度 | 结论 | 依据 |
|---|---|---|
| 能否编译/加载 | **✅ 可以**：单文件 CMake，依赖 `nav2_core/nav2_util/nav2_costmap_2d/pluginlib/tf2_ros` 全在；`pluginlib_export_plugin_description_file(nav2_core …)` 正确（GACC:39） | GACC:7-45、GACX:1-9 |
| 参数命名空间是否贴合我们 | **✅ 是**：我们只要把 MPPI/wrapper 参数写在 `FollowPath.` 下即可（`NAV:159`/`NAV:242` 的 `controller_plugins: ["FollowPath"]`，R:112） | S:84-176 与 R:128-149 结构相同 |
| 帧正确性 | **❌ 必须改**：`goal_`(map) vs `pose`(odom) 混帧（GAC:104-106 vs `N2/ctrl:611-617`/`R:161`） | §C.2 |
| `angular.z=0` 语义 | **⚠️ 必须做决定**：与我们 `FVT:94` 的 `spin_speed` 语义冲突（§C.5） | §C.5 |
| 与 MPPI 参数并存 | **⚠️ 需一并解决**：`model_dt` 必须 = 1/`controller_frequency`（§A.3），`CostCritic.consider_footprint: true` 在无 footprint 时会抛异常（§A.4） | `N2/mppi/opt:95-113`、`N2/mppi/cc:56-65`、`N2/cmros:408-416` |
| 综合 | **⚠️ 需改造后可用**（不是「直接可用」）：最小改造 = ①`direct_approach` 分支里把 `goal_` 用 TF 转到 `pose.header.frame_id` 后再算 dx/dy（或改为用 `costmap_ros_->getGlobalFrameID()` 做变换）；②`angular.z = 0.0` 加参数开关（默认「不改角速度」）；③`setPlan` 空路径时清 `goal_` 并置无效标志；④配 `spin_speed:=0.0` + `direct_approach_distance ≤ 0.5` | GAC:88-131 |

---

## D. 航点 CSV + `waypoint_editor`

### D.1 格式（贴真实数据）

`third_party/cod_nav_2026/src/cod_bringup/wps/` 下 6 个文件，表头**逐字节相同**（如 `patrol_center.csv:1`）：

```
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,
```

真实数据行（原样）：

```
# patrol_center.csv:2-4
0,4.62889,-3.84107,1.62125e-05,0,0,-0.979174,0.203024,
1,2.96424,-4.28705,6.67572e-06,0,0,0.566315,0.824189,
2,3.57432,-3.1591,5.72205e-06,0,0,-0.602533,0.798094,
```
```
# go_gain.csv:2-4
0,-1.59403,-4.04669,-9.53674e-07,0,0,-0.241624,0.97037,
1,2.2723,-6.15121,9.53674e-07,0,0,0.461767,0.887001,
2,3.52213,-4.15178,0,0,0,0.542143,0.840286,
```
```
# go_home.csv:2
0,3.5846,-4.37716,-9.53674e-07,0,0,0.884719,-0.466125,
```

6 个文件与 README 的对应（名字语义）与行数：

| 文件 | 行数（含表头） | 航点数 | README:70 的叙述 |
|---|---|---|---|
| `go_gain.csv` | 4 | 3 | 「去增益点」 |
| `go_home.csv` | 4 | 3 | 「回启动区」 |
| `patrol_behind.csv` | 4 | 3 | 「增益点后半区巡逻」 |
| `patrol_center.csv` | 5 | 4 | 「增益点内四角巡逻」 |
| `patrol_front.csv` | 4 | 3 | 「增益点前半区巡逻」 |
| `preload_patrol.csv` | 4 | 3 | 「前压巡逻」 |

（`README.md:70` 只给中文顺序、**没给文件名** ⇒ 上表的文件↔叙述对应是按名字推断的，见 §F-14。）

### D.2 单位与坐标系

| 项 | 结论 | 证据 |
|---|---|---|
| 列语义 | 0=`id`（整数，**读取端完全忽略**），1-3=`position x/y/z`，4-7=`orientation x/y/z/w`，8=`command`（**自由文本**），行尾有一个多余逗号 | `waypoint_to_nav2.cpp:116-137`（只读 `tokens[1..7]`）、`waypoint_through_nav2.cpp:109-127`、`waypoint_editor_tool.cpp:455-477`（写表头/行）、`:507-538`（读回） |
| 位置单位 | **米**（数值范围 −1.6…7.1，与 12.75 m×8.85 m 的地图一致） | 见 D.1 数据；`MAP:3-4` |
| `pose_z` | 恒 ≈0（±1e-5 浮点残差），即**平面航点** | `patrol_center.csv:2` 1.62e-05 等 |
| 朝向 | 单位四元组（roll/pitch 恒 0，只有 yaw） | `rot_x/rot_y` 全 0，`rot_z/rot_w` 组模长 1 |
| 坐标系 | **`map`**：桥节点参数 `frame_id` 默认 `"map"`（`waypoint_to_nav2.cpp:21`、`waypoint_through_nav2.cpp:20`），并写进 `pose.header.frame_id`（`:123`、`:115`）；编辑器也硬编码 `"map"`（`waypoint_editor_tool.cpp:515`） | 同上 |
| 时间戳 | `pose.header.stamp = rclcpp::Time(0)` ⇒ 让 Nav2 用当前时间 | `waypoint_to_nav2.cpp:124`、`waypoint_through_nav2.cpp:116` |
| `command` 列 | **没有任何节点解释它**：编辑器只把它当标签显示/编辑（`waypoint_editor_tool.cpp:132/208/356-369`），两个桥节点**只读前 8 列** | `waypoint_through_nav2.cpp:109-127`、`waypoint_to_nav2.cpp:116-137` |
| `sample_wp.csv` 里的 `skip:2 / wait:2 / stop:3 / end` | **是示例数据，不是协议**：对 `third_party/cod_nav_2026/src/**`（`*.cpp/*.hpp/*.py`）搜这三个模式只命中一处 —— `waypoint_through_nav2.cpp:96` 的行末注释 `// skip header` ⇒ **无任何实现** | `third_party/cod_nav_2026/src/waypoint_editor/data/sample_wp.csv:2-16`；`USAGE_WITH_NAV2.zh.md:5-10`（「waypoint_editor 保存的 CSV 格式航点不能直接被 Nav2 使用」） |

### D.3 谁消费这些 CSV

**结论：`wps/*.csv` 没有任何程序自动读取。** 证据：

1. `cod_bringup/CMakeLists.txt:25-28` 只是把 `wps` 目录**安装**到 share，让它「随包发布」。
2. 全仓库对 `csv/wps/patrol_*/go_gain/…` 的引用只有：`CMakeLists.txt:26`（安装）、`README.md:19/70`（文档）、`waypoint_editor` 的文档与 GUI 代码 —— **没有一个 ROS 节点**。
3. 实际消费路径是**人手动的**：RViz2 里 `WaypointEditorPanel` 的 **Load WPs** 打开 CSV（`waypoint_editor_tool.cpp:487-538` 的 `handleLoadWaypoints`，由服务 `load_waypoints` 触发，见 `waypoint_editor_panel.cpp` 的按钮绑定），或用桥节点：

| 消费者 | 类型 | 读取方式 | 动作 | 行为 |
|---|---|---|---|---|
| `waypoint_editor`（RViz panel/tool） | RViz 插件（GUI） | `QFileDialog::getOpenFileName` + `QTextStream`，`cols.size() >= 9`（`waypoint_editor_tool.cpp:487-538`） | 只是**编辑/可视化**航点（`/waypoint_markers`、连线、总长度），不发给 Nav2 | `README.md:66-80`、`USAGE_WITH_NAV2.zh.md:5-10` |
| `waypoint_to_nav2` | ROS 节点（可执行） | 参数 `waypoint_file`（`waypoint_to_nav2.cpp:20`）→ 逐行 `std::getline` + 逗号切分，**只跳过第 1 行表头**，要求 `tokens.size() >= 8`（`:117-120`） | 服务 `/start_waypoint_following`（`:24-27`）触发 → `FollowWaypoints` action client（`:30-31`）发**全部航点** | **逐点停靠**（`CMakeLists.txt:107-109` 注释） |
| `waypoint_through_nav2` | ROS 节点（可执行） | 同上（`waypoint_through_nav2.cpp:19/85-137`） | 服务 `/start_waypoint_through`（`:22-25`）触发 → `NavigateThroughPoses` action client（`:27-28`） | **连续穿越、不停留**（`CMakeLists.txt:118-119` 注释；`USAGE_WITH_NAV2.zh.md:19-23`） |

**触发方式（原样）**：

```bash
ros2 launch waypoint_editor waypoint_through_nav2.launch.py waypoint_file:=/abs/path/wps/patrol_center.csv
ros2 service call /start_waypoint_through std_srvs/srv/Trigger
# 或逐点停靠版
ros2 launch waypoint_editor waypoint_to_nav2.launch.py waypoint_file:=/abs/path/wps/go_gain.csv
ros2 service call /start_waypoint_following std_srvs/srv/Trigger
```

（`waypoint_through_nav2.launch.py:11-33`、`waypoint_to_nav2.launch.py:11-33`、`USAGE_WITH_NAV2.zh.md:50-70`。）

**与 `waypoint_follower` 的关系**：`waypoint_to_nav2` 走的是 **`/follow_waypoints` action**，由 `nav2_waypoint_follower/waypoint_follower` 节点提供 —— 该节点**两个 launch 都会起**（`NL:163-172`；我们 `NAV:200-209` 同样），其参数在 `S:460-469`（我们 sim 变体里**没有这一段**，走默认；旧 `nav2_params.yaml:347-355` 有逐字相同的段）。`waypoint_through_nav2` 走 **`/navigate_through_poses`**，由 `bt_navigator` 提供（两边都在跑）。

### D.4 对我们「长目标分段 / 拓扑航点（路线 B/C）」的具体用法建议

1. **格式层：✅ 可直接沿用**。CSV 契约只有「`map` 帧下的 x/y/z + 四元数」，与我们的 `map`（R:243）+ `base_link_fake`（R:244）完全兼容：桥节点发的是 `PoseStamped`，由 `bt_navigator`/`controller_server` 自己按 TF 变换，不涉及 `base_link_fake`。我们可以直接拿他们的 CSV **当模板**（但坐标必须在**我们的图**上重采，见下条）。
2. **坐标必须在我们的地图系里重采**。他们的航点长在他们的 `MAP`（origin `[-1.756,-7.036]`，12.75×8.85 m）里；我们的 `RMUL2026` 图与初始位姿是另一套（`LAUNCH:60`、`:67-80`，注释里明确说过 RMUL2026 曾从「世界系图」换成「cartographer 出生点系图」，两者差 ≈(4.3, 3.35)）⇒ **照抄 CSV 数值 = 把车指到错的地方**。
3. **路线 B/C 用哪个桥**：
   - **连续穿越（推荐给「长目标分段」）** → `waypoint_through_nav2` + `NavigateThroughPoses`。理由：① 它天然就是「一串中间点 + 一个终点」的分段语义；② 我们的 BT 已经能跑（`nav2_compute_path_through_poses_action_bt_node` R:71、`nav2_remove_passed_goals_action_bt_node` R:97、`nav2_navigate_through_poses_action_bt_node` R:95）；③ 中间不停车，适合竞速式通过。
   - **拓扑航点（每个点要停下做动作/等待）** → `waypoint_to_nav2` + `FollowWaypoints`（`nav2_waypoint_follower`）。此时才需要 `waypoint_follower` 参数段。
4. **必须注意的两个坑**：
   - **`waypoint_pause_duration: 200` 的单位是毫秒**（`N2/waw:78-81`）⇒ 只停 **0.2 s**。若我们要「在每个拓扑点停 2 s 等机构动作」，要写 `2000`。我们已有这段（`src/nav/rm_navigation/rm_navigation/params/nav2_params.yaml:355`），但 **sim 变体 R/D/T 里没有** ⇒ 走默认值。
   - **`stop_on_failure: false`**（`S:464`，我们旧文件同）：某个航点失败会**静默跳到下一个**。对「长目标分段」这是危险的（缺一段仍算成功）；建议 `stop_on_failure: true`，并在上层用 `/follow_waypoints` 的 feedback 做断点续跑。
5. **through-poses 的节奏**：官方 through-poses 树是 `RateController hz="0.333"`（`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml:10`）⇒ **3 s 重规划一次**，而 `RemovePassedGoals radius="0.7"`（同文件 `:13`）把 0.7 m 内的点丢掉。我们的全局 costmap 是 `update_frequency 5.0`（R:241）⇒ 3 s 重规划对 5 Hz 图是浪费，若要多点激进行驶，应改自定义 BT 的 `hz`（他们 `BTTP:8` 用 3.0 Hz，可参考但要按我们 0.5 m/s 重调）。
6. **落地清单（不改第三方代码）**：
   - 装 `waypoint_editor` 包（`CMakeLists.txt:107-131` 的两个可执行 + 可选 RViz 插件），或在我们的 `rm_navigation` 里照抄那两个 `.cpp`（各 ~190 行、只依赖 `rclcpp/rclcpp_action/nav2_msgs/geometry_msgs/std_srvs`，`CMakeLists.txt:110-127`）。
   - 在 `R`/`D`/`T` 里补 `waypoint_follower` 段（照抄 `S:460-469`，但 `waypoint_pause_duration` 按需改成毫秒真值，`stop_on_failure: true`）。
   - 用 RViz `WaypointEditorPanel` 在**我们的 RMUL2026 图**上采点 → Save WPs → 得到 `map` 系 CSV。
   - 用 `ros2 launch waypoint_editor waypoint_through_nav2.launch.py waypoint_file:=…` + `ros2 service call /start_waypoint_through std_srvs/srv/Trigger` 触发。

---

## E. 总结论：三张清单

### E.1 ✅ 可直接用（键名与值都成立；抄过去不改也能跑）

| # | 项 | 落到我们哪个文件哪个键 | 风险 | 回退方式 |
|---|---|---|---|---|
| E1-1 | stvl 体素层参数块（`voxel_decay/decay_model/voxel_size/track_unknown_space/observation_persistence/max_obstacle_height/mark_threshold/update_footprint_enabled/combination_method/origin_z/publish_voxel_map/mapping_mode/map_save_duration/transform_tolerance/filter/voxel_min_points/clear_after_reading`） | `R:288-333`（已同构）；要抄的具体值：`max_obstacle_height 2.0`(R:312)、`mark_threshold 0`(R:292)、`combination_method 1`(R:294)、`update_footprint_enabled true`(R:293) | 我们的源是 `/segmentation/obstacle`(R:303) 而非他们的 `/livox/lidar_filtered`(S:221) ⇒ **只抄参数不抄话题** | 我们现有的 `publish_voxel_map: true`(R:296) 与两个源 `livox_mark/livox_clear`(R:300-333) 保留即可；出现幽灵障碍就回退 `min_obstacle_height` 0.05（R:307-311 已写好回退建议） |
| E1-2 | Smac2D 的 9 个有效键 + `smoother.*` 子块 | 新增 `R`/`D`/`T` 的 `planner_server.GridBased`（替换 `R:359` 的 NavfnPlanner）：`tolerance 0.5`、`allow_unknown true`、`downsample_costmap false`、`downsampling_factor 1`、`max_iterations 1000000`、`max_on_approach_iterations 1000`、`max_planning_time`（我们建议 1.0–2.0，不用他们的 4.5）、`cost_travel_multiplier`（建议 1.0–2.0，不用 4.0）、`smoother.{max_iterations,w_smooth,w_data,tolerance,do_refinement}`（S:400-405） | `GACC`/`S` 里那 16 个 Hybrid 键**不能一起抄**（§A.16）；Smac2D 侧 `_smoother->smooth()` 无条件执行（N2/smac2:299-300）⇒ 平滑开销一直存在 | 保留 `NavfnPlanner` 分支（`use_astar: false`，R:361）；计划器按 `planner_plugins` 一键切回 |
| E1-3 | `velocity_smoother` 键名全表 + `cmd_vel` remap 链 | `R:395-408`（键名已一致）；值建议保留我们的（`deadband [0,0,0]`、`max_accel [4,4,6]`） | 抄他们的 `deadband 0.05`(S:478) 会在 0.5 m/s 巡航下吃掉小指令 | 改回 `R:401-406` 原值即可（可 A/B） |
| E1-4 | `map_saver` 段 | `R:345-351`（已逐字相同） | 无 | — |
| E1-5 | MPPI 骨架键名 + 6 个 critic 键名（≠ 值） | 新增 `FollowPath.time_steps/model_dt/batch_size/iteration_count/temperature/gamma/motion_model/vx_std/vy_std/wz_std/prune_distance/transform_tolerance/visualize/reset_period/retry_attempt_limit/regenerate_noises` + `critics` + 各 critic 段（键名照 `S:126-176`，**但要剔掉 `CostCritic.trajectory_point_step`，它是死键**） | **值不能照抄**（§A.3/A.4：20 Hz 与 model_dt 冲突、2.5 m/s、`consider_footprint: true` 抛异常） | 我们可先只接 MPPI + DiffDrive/Omni 默认值，再逐项调 |
| E1-6 | `waypoint_editor` 的两个桥节点 + CSV 格式 | 新增包（或把 `waypoint_to_nav2.cpp`/`waypoint_through_nav2.cpp` 抄进 `rm_navigation`） | 需 `nav2_msgs`/`std_srvs`（已有）；坐标必须在我们图系重采 | 不用时不影响任何现有栈（纯附加节点） |
| E1-7 | `PositionGoalChecker`（插件名，不抄 `path_length_tolerance`） | `R:123` 可改为 `nav2_controller::PositionGoalChecker` + `xy_goal_tolerance`（`R:124`）+ `stateful`（`R:122`），**删掉 `yaw_goal_tolerance`（R:125）** | 忽略朝向 ⇒ 终点朝向不受控（若上层有朝向要求就不能换） | 改回 `SimpleGoalChecker`（`R:123-125`） |
| E1-8 | `nav2_smoother::SavitzkyGolaySmoother` 段（S:407-420） | 新增 `R`/`D`/`T` 的 `smoother_server:` 段，**必须把 `robot_base_frame` 改成 `base_link_fake`** | 我们的 BT 不会调它（默认树无 SmoothPath）⇒ 只是多一个服务，无副作用 | 删掉该段即回默认 |

### E.2 ⚠️ 需改造

| # | 项 | 落到我们哪个文件哪个键 | 风险 | 回退方式 |
|---|---|---|---|---|
| E2-1 | `goal_approach_controller` 本体（帧 bug + `angular.z=0` 语义 + 空路径不复位） | 新包 + `R:128`（`FollowPath.plugin`）；参数 `approach_distance/approach_velocity/direct_approach_distance/direct_approach_kp` | 混帧（GAC:104-106 vs R:161）⇒ 末端朝错误方向平移；`angular.z=0`（GAC:118）在 `spin_speed=5.0` 时**关掉小陀螺**且让 `FVT:91-96` 的线速度旋转退化成空转 | 保留 `R:129`(RPP)/`D:129`(DWB)/`T:130`(TEB) 三个 `FollowPath.plugin` 现值，用 `nav` 启动参数（`LAUNCH:151-156`）一键切回 |
| E2-2 | 接 MPPI 的 `controller_frequency × model_dt` | `R:105`（`controller_frequency: 20.0`）与新增 `FollowPath.model_dt` | 若 `model_dt 0.02` + 20 Hz ⇒ `N2/mppi/opt:111-112` 抛异常，**controller_server 起不来** | 两个选项：`model_dt: 0.05`（保持 20 Hz，`N2/mppi/opt:104-109` 会开 `shift_control_sequence`）或 `controller_frequency: 50.0`（照 S:67） |
| E2-3 | `min_y_velocity_threshold` | `R:107`（`0.5`） | 侧向里程计反馈被抹零，Omni MPPI 的速度反馈是错的 | 改成 `0.001`（照 S:69）；若要回退，RPP 不受影响 |
| E2-4 | `CostCritic.consider_footprint`（`ObstaclesCritic` 有同样检查） | 新增 critic 段 | 我们无 footprint ⇒ `N2/mppi/cc:56-65`（`ObstaclesCritic` 见 `N2/mppi/oc:46-55`）**抛异常** | 两者都设 `false`（照 `S:146` 的 ObstaclesCritic 写法），或先给两张 costmap 加 footprint |
| E2-5 | BT 树（3 Hz 重规划、retries 10、删 Spin/Wait、BackUp 1.0 m/1.0 m/s） | 新增 `behavior_trees/*.xml` + `bt_navigator.default_nav_to_pose_bt_xml`（`R:61-100` 当前无此键） | 我们 `FVT:80-84` 的直通修复让 `Spin` **可用**；照抄他们的树会**删掉我们唯一有效的原地恢复**；`BackUp 1.0 m/s` 在 0.40 半径 + sim 场地会撞 | 不设 `default_nav_to_pose_bt_xml`（回 nav2 内置树，即当前 `R:61-100` 的现状）；或只搬「3 Hz + retries」不搬「删 Spin」 |
| E2-6 | costmap 几何（`robot_radius 0.1` / `footprint ±0.1` / `inflation 0.55→0.75`） | `R:178/252`（`robot_radius: 0.40`）、`R:235/342`（inflation 0.5/0.55） | 抄 0.1 会让规划器贴墙钻 `P2L` 盲区（R:168-177 的整段推理） | 两个方向都可 A/B：半径在 0.35–0.45 之间（R:177 已写回退区间）；inflation 若调到 0.55/0.75，必须 ≥ `robot_radius`（`N2/infl:173-181` 只在小于时报 ERROR，不会自动夹紧） |
| E2-7 | MPPI 速度/权重/时域数值（2.5 m/s、`wz_max 1.5`、`time_steps 80`、`temperature 0.25`、GoalCritic 25、PathFollow 15、PathAlign 15） | 新增 `FollowPath.*` | 这些是给他们 2.5 m/s 车 + 10 m 局部窗 + 实车小陀螺调的；我们 0.5 m/s（R:130）、5 m 窗（R:165-166）、2 cm 分辨率（R:167）⇒ 直接抄会抖/绕圈 | 从 1.1.20 默认值起步（`model_dt 0.05`、`time_steps 56`、`batch_size 1000`、`temperature 0.3`、各 critic 默认），逐项调 |
| E2-8 | `progress_checker`（0.1 m / 999 s ⇒ 等效关闭） | `R:117-118`（0.5 / 10.0） | 抄他们 = 放弃卡死检测；MPPI 早期调试阶段这个检测恰好最有用 | 保留我们现值；调好后再考虑放宽 |
| E2-9 | `behavior_server.global_frame` | `R:381`（`odom`） | 改 `map`（S:442）会让恢复动作受定位跳变影响（我们 `map→odom` 是 amcl/icp 发的高频量） | 保留 `odom`；他们用 `map` 是因为他们的 `map→odom` 是固定的 |
| E2-10 | `waypoint_follower` 段（我们 sim 变体缺） | 新增到 `R`/`D`/`T`（可照 `nav2_params.yaml:347-355`） | `waypoint_pause_duration 200` 是**毫秒**（`N2/waw:78-81`）；`stop_on_failure` 默认 true（`N2/wpf:37`）与他们写的 false 不同 | 删段即回默认 |
| E2-11 | slam_toolbox 参数（`lifelong`、`transform_publish_period 0.0`、`base_frame base_link`） | `SLAMrm:15/17/28` | `transform_publish_period 0.0` = slam_toolbox 不发 `map→odom`（他们靠 `ML:95-116` 静态桥补）；`base_frame base_link` 在我们 TF 里不是 `/scan` 的帧（P2L:9 `target_frame: ""` ⇒ 帧是 `livox_frame`） | 保留 `SLAMrm` 现值（`mapping` / `0.02` / `livox_frame`） |
| E2-12 | `P2L`（`range_min 0.05`、`max_height 0.1`、`target_frame ""`）vs 他们 `ML:66-84`（`target_frame base_link`、`min_height 0.1`、`max_height 1.0`、`range_min 0.5`、`range_max 20.0`、`angle_increment 0.0087`） | `P2L:9/14/15/18/37/38` | 我们的 `max_height 0.1` + `min_height -1.0` 是「只留地面附近之上 10 cm」的策略（配 linefit 去地面）；他们 0.1–1.0 是「哨兵高度带」；`range_min 0.5` 会再造出我们花大力气消掉的 45 cm 盲区（P2L:20-36 的整段考证） | 保留 `P2L` 现值；`range_min` 回退档位已写在 `P2L:29/36`（0.15 / 0.2） |

### E.3 ❌ 不适用

| # | 项 | 出现位置 | 落到我们哪个键（若要抄会写在哪） | 风险/原因 | 处理 |
|---|---|---|---|---|---|
| E3-1 | `publish_critics_stats` | S:65 | `R:102` 段 | 1.1.20 无此键 | 不抄 |
| E3-2 | `vy_min`, `wz_min`, `ax_max/ay_max/az_max/ax_min/ay_min/az_min` | S:103-113 | `FollowPath.*` | 1.1.20 MPPI 只有 `vx_max/vx_min/vy_max/wz_max`（`N2/mppi/opt:75-78`）⇒ 6 个加速度键**全无效**；真正限加速的是 `velocity_smoother.max_accel/max_decel` | 用 `velocity_smoother`（R:405-406）表达加速度约束 |
| E3-2b | `CostCritic.trajectory_point_step` | S:159 | `FollowPath.CostCritic.*` | 1.1.20 `CostCritic` 不读此键（`N2/mppi/cc:26-32`），只有 PathAlign 系 critic 读 | 放到 `PathAlignCritic.trajectory_point_step`（S:173）才有意义 |
| E3-3 | `path_length_tolerance` | S:83 | `R:121-125` 段 | 1.1.20 `PositionGoalChecker` 不声明（`N2/ppgc:44-52`） | 不抄 |
| E3-4 | `bt_navigator.timeout` | S:8 | `R:68` 附近 | 1.1.20 无此键（`N2/bta:57-67`） | 用 `default_server_timeout`（`R:68` 已是 20） |
| E3-5 | planner 的 16 个 Hybrid/Lattice 键 | S:385-399 | `planner_server.GridBased.*` | 只有 Hybrid/Lattice 读（N2/smac2 只读 9 键） | 不抄；要抄就换 `SmacPlannerHybrid` |
| E3-6 | `local_costmap_topic/global_costmap_topic/local_footprint_topic/global_footprint_topic/local_frame` | S:425-428, 441 | `behavior_server.*`（`R:364-388`） | 1.1.20 behaviors 只读 `costmap_topic/footprint_topic/global_frame/robot_base_frame/transform_tolerance`（`N2/beh:34-39`、`N2/timed:119-122`） | 我们 `R:369-370` 的 `costmap_topic/footprint_topic` 才是对的 |
| E3-7 | `pb_nav2_behaviors/BackUpFreeSpace`（含 `robot_radius`/`max_radius`/`service_name`/`free_threshold`/`visualize`） | S:434, 450-454 | `behavior_server.backup.plugin`（`R:376`） | 需编译 `third_party/cod_nav_2026/src/pb_nav2_plugins`；`free_threshold`/`robot_radius` 连它自己都没读（`back_up_free_space.cpp:28-37`） | 保留 `nav2_behaviors/BackUp`（`R:376`） |
| E3-8 | `realsense_source`（D435i） | S:241-262、331-352 | 两张 costmap 的 stvl 源 | 我们无该设备/话题 | 不抄 |
| E3-9 | 他们的定位架构（LIO + 静态 `map→odom`；`SL:67-89`/`ML:95-116`） | launch | `LAUNCH:446-468`（我们的静态桥只在 `localization=''` 时启） | 与我们 `mode × localization` 槽位体系冲突；且 `ML` 里静态桥与 slam_toolbox **同时发 `map→odom`** | 沿用我们的槽位（LAUNCH:303-406） |
| E3-10 | `auto_save_map.launch.py` | `ML:154-158` 引入 | 无（不要引入） | 硬编码 `/home/cod-sentry/dyx_ws/...`（`auto_save_map.launch.py:12-15`）⇒ 在 `-c` 里 `source` 不存在的 setup.bash、`mkdir -p` 到无权路径；且它在启动时立刻注册 10 个定时保存（`:18-24`） | 不抄；我们用 `map_saver_cli` 手动/脚本保存 |
| E3-11 | `robot_state_publisher` 段 | S:456-458 | `R` 顶层 | 两个 launch 都不起该节点 ⇒ 惰性 | 不抄 |
| E3-12 | 他们的 `robot_radius 0.1` + `footprint ±0.1` + 2.5 m/s + 50 Hz 控制环 | S:67/99-105/190-191 | 多处 | 物理前提是 0.2 m 见方哨兵 + 深度相机 + 实车电调 | 只作为「量级参考」，数值必须重标定 |
| E3-13 | `wps/*.csv` 的**坐标值** | `wps/*.csv` | 若直接喂给我们的桥 ⇒ 车去错地方 | 图/原点不同（`MAP:4` origin `[-1.756,-7.036]` vs 我们 `LAUNCH:60/67-80`） | 只借格式，坐标在我们图上重采 |

---

## F. 未能确定的 / 文件内自相矛盾之处

### F.1 自相矛盾（注释 vs 数值、文件之间）

| # | 现象 | 证据 |
|---|---|---|
| F-1 | 注释「预测时域4.8s」，但 `time_steps 80 × model_dt 0.02` = **1.6 s**（4.8 s 只在 M 的 `model_dt 0.06` 下成立） | `S:91-92` vs `M:82-83` |
| F-2 | 注释「放宽到25cm」，数值是 **0.30** | `M:78` |
| F-3 | 注释「与 local_costmap 一致」，但 global 是 **15.0**、local 是 **5.0** | `M:282` vs `M:192`（对比 `S:288` 与 `S:198` 确实都是 5.0，说明 M 是复制后只改了值没改注释） |
| F-4 | 注释声称 `critical_cost: 253.0` 做到「只有 inscribed/lethal 才触发碰撞惩罚」，但该判定是**源码硬编码** `pose_cost >= INSCRIBED_INFLATED_OBSTACLE`（=253）；`critical_cost` 只是**惩罚幅度**（默认 300.0） | `S:155` vs `third_party/nav2/nav2_mppi_controller/src/critics/cost_critic.cpp:164` 与 `:29` |
| F-5 | `S:154` 注释「适当降低（cost_weight 3.8）」——但 3.8 与 1.1.20 默认 3.81 几乎相同，谈不上「降低」 | `S:154` vs `cost_critic.cpp:28` |
| F-6 | 同一句注释「放大关闭距离」被同时贴在 `threshold_to_consider: 1.0`（S:166）与 `0.8`（S:174）上；而 M 把两者都**调大**到 3.0，注释却变成「缩小关闭距离…防止圆周运动」 | `S:166/174` vs `M:160/168` |
| F-7 | M 的 `behavior_server.use_sim_time: true` 与全文其余 `false` 矛盾；且会被 `NL:61-71` 的 RewrittenYaml 用 launch 实参（`SL:123`/`ML:141` 传 `"false"`）覆盖 ⇒ **惰性** | `M:418` vs `M:3/60/366/…`；`SL:123`、`ML:141` |
| F-8 | `non_straight_penalty: 0.0` 的注释写「必须 ≥ 1」（在 Smac2D 上该键本来就无效，所以也没人会报错） | `S:393` |
| F-9 | `NL:68` 把 `root_key=namespace` **注释掉**了，而 `LOC:63` 保留了 `root_key=namespace`（同一仓库两个 launch 的 RewrittenYaml 语义不一致 ⇒ `use_sim_time`/`yaml_filename` 的覆写范围不同） | `NL:65-71` vs `LOC:60-66` |
| F-10 | 地图 yaml 的 `free_thresh: 0.196`（`MAP:7`）与 `map_saver.free_thresh_default: 0.25`（`S:366`）不一致 | `MAP:7` vs `S:366` |
| F-11 | `ML` 同时起 slam_toolbox（`ML:85-94`，其 `SLAMc:30` 明确 `transform_publish_period: 0.0` = 不发 `map→odom`）**和**静态 `map→odom` 广播（`ML:95-116`）——这是「用静态桥补 map→odom」的设计，但 SLAM 建图模式下 map 系本应由 slam_toolbox 接管；二者语义重叠 | `ML:85-116`、`SLAMc:30` |
| F-12 | `S:360` 的 `map_server.yaml_filename` 是硬编码 `/home/cod-sentry/...`，与 `LOC:56-58/76-79` 的「由 launch `map` 参数覆写」相矛盾（照 `LOC` 的说法，yaml 里这行不该有真实路径） | `S:360` vs `LOC:56-58` |
| F-13 | （我们侧独立问题）TEB 的 `footprint_model.circular.radius: 0.2`（`T:150`）与 R/D/T 共用的 two costmap `robot_radius: 0.40`（`R:178/252`）不一致；DWB 的 `max_vel_theta 12.0`（`D:135`）被 `velocity_smoother.max_velocity[2]=3.0`（R:401）截住 | `T:150`、`D:135`、`R:401` |
| F-14 | `README.md:70` 只说「wps 中 csv 文件的航点路径依次为…（6 个中文名）」但**不给文件名**；文件名↔叙述的对应只能按名字推断（§D.1） | `README.md:70` |

### F.2 未能确定的项

| # | 未能确定的内容 | 为什么无法确定 | 影响 |
|---|---|---|---|
| F-15 | `S:198` 注释说「原 0.3 衰减太慢」——0.3 这个历史值在仓库里 **找不到**（无 git 历史可查，`third_party/cod_nav_2026/.git` 存在但本次任务要求只读文件、未做历史考古） | 只读了工作树 | 不影响结论（我们 local 本来就是 5.0，`R:232`） |
| F-16 | 他们实车底盘的真实 `spin_speed`：`FVTc:19` 默认 0.0，且 `third_party/cod_nav_2026/src/fake_vel_transform/launch/fake_vel_transform_launch.py:9-18` 不传参；但 `FVTc:60` 无条件把角速度写成 `spin_speed` ⇒ 若真是 0.0，**他们的底盘在任何导航里都不会自转**（与 `wz_max 1.5`、`Spin` 恢复动作、`max_rotational_vel 1.0` 全部矛盾） | 仓库里没有任何参数文件设置 `spin_speed`；串口下位机行为未知 | 「他们为什么能跑」的解释存疑：可能实车下位机自己在转，也可能他们另有未入库的 launch。**因此 §C.5 的结论只对他们的代码文本成立** |
| F-17 | `get_costmap` 服务的实际命名空间（`/global_costmap/get_costmap` 还是 `/get_costmap`） | `costmap_2d_publisher.cpp:81-82` 用相对名 `"get_costmap"` 创建，具体前缀取决于 `Costmap2DPublisher` 被构造时用的节点；未起栈实测（任务要求只读、不执行） | 只影响 `pb_nav2_behaviors/BackUpFreeSpace` 这一条（我们本来就不用） |
| F-18 | 我们的 `/local_plan` 到底有没有第二发布者（除 nav2 与 TEB 之外，比如仿真插件或外部脚本） | 仓库内 `grep -rn "local_plan" src/` 排除 `teb_local_planner` 子串后无命中；未在运行中的系统上 `ros2 topic info` 实测 | 决定 §C.5「`spin_speed=5.0` 时线速度会不会被旋转」。**但即使有发布者，`angular.z=0` 会顺手停掉小陀螺这一点不变** |
| F-19 | 他们 `M` 里 `direct_approach_*` 被删是否**有意**（vs 忘写） | 无注释、无文档说明；只能从 S/M 的用途差异推断（§B-8/B-9） | 若是有意，则「多点模式不要末端直接驱动」这一条经验值得我们参考；若是忘写，则他们的多点模式末端行为与单点不同却无人知晓 |
| F-20 | 他们的 BT 树 `RecoveryFallback` 里 `BackUp` 的 plugin 解析（BT 里写 `BackUp`，behavior_server 里 `backup` 是 `BackUpFreeSpace`）——BT 节点名是 action 名不是插件名，理论上会路由到 `backup` 插件；未起栈验证 | 未运行 | 只影响我们对「他们恢复行为到底是什么」的描述精度 |
| F-21 | `S:83` 的 `path_length_tolerance` 在他们自己的 nav2 版本上是否真的存在（他们用的是 Humble 吗？） | `README.md:6` 写 `ROS2 Humble`，但 yaml 里同时出现 Humble 无、Jazzy 有的键（`path_length_tolerance`、`publish_critics_stats`、`local_costmap_topic`、`vy_min/ax_*`）。⇒ 他们**实际编译运行的 nav2 版本很可能比本机 1.1.20 新**（或混用了 main 分支源码），但仓库里**没有任何版本锁定文件**（无 `ros2.repos`/`COLCON_IGNORE`/依赖清单） | 这是本文最重要的未确定项：**他们能跑的 yaml 我们不一定能跑**。建议在任何移植前先 `ros2 param list /controller_server` 在两边对齐键名 |

### F.3 交叉验证建议（不改任何文件，仅运行期确认）

1. 起一次他们的栈（在**他们**的工作区，非本仓库）后执行 `ros2 param list /controller_server | grep -E "vy_min|ax_max|publish_critics_stats"` —— 若列出 ⇒ 他们的 nav2 版本 ≥ Jazzy 语义，本文 §A.16 的死键表就要整体重算。
2. 在本仓库起栈后 `ros2 topic info /local_plan -v` 确认 §F-18。
3. `ros2 param get /bt_navigator odom_topic`（我们侧预期 `/Odometry`，与实际 `/odom` 不符，见 §A.10）。


---


## 来源 14/19：`tdt/cod_nav_comparison.md`


> 对象：https://github.com/qza36/COD_NAV —— "Sentry Navgation System for RoboMaster Session 2025"
> （辽宁科技大学 COD 战队哨兵，BSD-3-Clause，30 stars / 3 forks / 0 open issues，CI 仅编译无测试）
> 结论来源：只读分析（raw.githubusercontent + GitHub API），2026-09-26。**本文只做对比与借鉴记录，不改任何代码。**

## 0. 先弄清：仓库里有**两套不同架构**，默认分支是旧的

| 分支 | 最后提交 | 架构要点 |
|---|---|---|
| **`master`（默认，已过时）** | 2025-08-02 | FAST-LIO v2 fork（改 TF，`/Odometry` 发 `odom→chassis`）+ **patchwork++** 地面分割 + p2l(`/patchworkpp/nonground`→`/scan`，**range_min 0.48 / range_max 20.0**) + **small_gicp_relocalization**（对先验 PCD 求 `map→odom`）+ pcd2pgm 离线先验 2D 图；Nav2 = **SmacPlannerHybrid(DUBIN, allow_unknown true)** + **pb_omni_pid_pursuit_controller** 20 Hz；**无 AMCL、无轮式里程计** |
| **`rmul2026`（最新）** | 2026-03-10 | **small_point_lio** + **slam_toolbox async `mode: lifelong`**（**只支持边建图边导航**，先验图重定位被砍）+ **`cpp_lidar_filter` 裁剪盒去车体点云** → `/livox/lidar_filtered` + **MPPI Omni @50 Hz（vx/vy_max 7.5 m/s）** + **两张 costmap 都用 STVL** + Savitzky-Golay 平滑；`map→odom` 是**静态 TF**（z=0.05） |

另有 `ul_mppi`、`rmul2026_sim`、`feature-obstacle_escape_node`。`rmul2026` 分支有 `CLAUDE.md` 写架构（含自记 bug：`bt_navigator.odom_topic: "odomety"` 拼错）。

## 1. 逐项对照

| 维度 | COD_NAV（master / rmul2026） | 我们 | 判断 |
|---|---|---|---|
| 上下文 | ROS 2 Humble + Nav2，RM 哨兵，分支活跃 | ROS 2 Humble + Gazebo Classic RM 哨兵 bench | 平；**bench/实验设施我们更强** |
| LIO | FAST-LIO fork / Small Point-LIO | FAST-LIO / pointlio（槽位可选） | 平 |
| 2D 建图 | pcd2pgm 离线图 + slam_toolbox；**cartographer 配置是死的**（节点被注释） | cartographer（**改了核心 `min_probability_to_clear`**）+ slam_toolbox，在线/离线分开 | **我们强** |
| 先验图重定位 | 只有 small_gicp（硬编码 PCD 路径）；rmul2026 **完全没有** | AMCL / ICP / slam_toolbox / cartographer 四选一 | **我们强** |
| `/scan` 射程 | **20 m**（两个分支都是） | **10 m** | **他们强**（场地 15×28 m ⇒ 我们的 10 m 必然"远距缺墙"） |
| 近距/自身点 | 2026 用**裁剪盒去除车体点云**后喂 STVL（可到 0.1 m 高度） | 靠 p2l `range_min`（已试 0.05）+ local cloud 层 | **他们强**（方案更干净） |
| 全局规划 | SmacPlannerHybrid(DUBIN) / Smac2D + `cost_travel_multiplier` + `tolerance 0.5` | NavFn | **他们强** |
| 全局 costmap | res 0.04、`robot_radius 0.15`、inflation 0.7/5.0、`track_unknown_space true`、只吃 `/scan` | `robot_radius 0.40`、inflation 0.55、stvl + scan | 混合：他们更宽松；**我们的 footprint 更诚实** |
| 局部 costmap | 10×10(2025) / 14×14(2026)，**体素层/STVL（带时间衰减）** | 10×10，obstacle+cloud，**无衰减** | **他们强**（时间维） |
| 局部控制器 | pb_omni PID pursuit 20 Hz / **MPPI Omni 50 Hz、7.5 m/s** | RPP / DWB / TEB（**MPPI 已装未接**） | **他们强**（控制器与速度包络） |
| 障碍时间维 | rmul2026 两图都 STVL（`voxel_decay 0.5`） | 只有 global STVL | **他们强** |
| 未知区策略 | `allow_unknown true` + `track_unknown_space true`，且**提交的图 72×23 m 大面积 unknown** | `allow_unknown true`；场地 15×28 m | 平——**他们也没解决，甚至更糟** |
| 动态障碍 | **无跟踪、无预测、无速度估计**；只有 STVL 衰减 | 同上 | 平（都没做） |
| 任务层 | BT：`RateController 3 Hz` 重规划 + `IsStuck→BackUp 1 m@1.5 m/s` + 10 次重试；**目标由 bash 脚本轮询裁判系统 `game_type`** | stock BT + 自研分段工具 | 平（都薄；**他们没航点图/前沿探索**） |
| 仿真 | master 有 Gazebo Classic(UL24)；**rmul2026 无仿真** | 一等公民的多算法仿真 | **我们强** |
| 多算法 A/B | 靠改参数/换 launch 文件 | 槽位化（mode/lio/mapper/localization/nav/obstacle） | **我们强** |
| 工程卫生 | 硬编码 `/home/cod-sentry/...`、提交了 `.swp` 与 `cmake-build-debug/`、`map,yaml` 拼错、死配置、`clear_costmap_caller` 空指针 | 有文档、有回归工具、无硬编码 | **我们强** |

## 2. 可借鉴清单（按优先级，含代价/风险）

| # | 改什么 | 收益 | 代价/风险 |
|---|---|---|---|
| **1** | **裁剪盒去车体点云**（x ±0.3、y −0.3~0.5、z −0.1~0.2、`negative: true`、leaf 0.05）→ 喂 **stvl + local cloud 层**（`/scan` 仍走 linefit） | **直接打我们"近距看不到墙/自身遮挡"那条**：不用再纠结 `range_min`，近距点云可以放心用 | 多一个 10 Hz 节点；裁剪盒要按我们车体调（他们 y 0.5 是不对称的），太紧会删掉真障碍 |
| **2** | `p2l.range_max: 10 → 20`；local costmap `10×10 → 14×14` | 15×28 m 场地一次看全 ⇒ **消灭"远距离规划穿空白"** | costmap 更新成本 2~4×；高度带要重标（他们的带在雷达面**之上** 0.01~1.0，我们的是之下） |
| **3** | **局部 costmap 也上 STVL**（带 `voxel_decay 0.5`、`voxel_size 0.05`、`obstacle/raytrace_range 8~9 m`） | 局部幽灵障碍 ~0.5 s 内消失（我们现在没有时间维） | CPU；`raytrace` 太激进会擦掉薄结构 |
| **4** | 全局规划换 **Smac2D**（`allow_unknown true`、`tolerance 0.5`、`cost_travel_multiplier 4.0`）+ **Savitzky-Golay 平滑**（窗口 7/3 阶/refinement 2） | `cost_travel_multiplier` 是"窄缝被当走廊"的**直接对策**（把路径推向代价谷底）；`tolerance 0.5` 让目标落在膨胀格也能规划 | Smac 比 NavFn 慢（他们 `max_planning_time 4.5 s`）；SG 平滑可能切角，须验碰撞 |
| **5** | 接 **MPPI Omni** 作为第 4 个 `nav` 槽（用他们的 critics 配方：`critical_cost 253` + `consider_footprint`、`GoalCritic 15`、PathFollow/PathAlign `threshold 1.5`、`temperature 0.25`） | 免梯度的采样式控制 + 现成的战训参数；**`threshold 1.5` 是他们对"近目标切向震荡"的修正** | `batch_size 2000 × time_steps 60` 每周期 ⇒ CPU；**速度要从 2.0 m/s 起调**，别照抄 7.5 |
| **6** | BT 结构借：`RateController 3 Hz` 重规划 + `IsStuck → BackUp(1 m @1.5 m/s)` + 10 次重试；哨兵模式可考虑 **`yaw_goal_tolerance: 6.28`（只控位置）** | 恢复逻辑便宜且鲁棒；"只控位置"与我们 `spin_speed!=0` 的小陀螺模式天然匹配 | 6.28 意味着**朝向安全要单独负责**；BackUp 1.5 m/s 在拥挤场地可能触发自身避障 |

## 3. 他们**没解决**的（别去那儿找答案）

1. **未知区**：`allow_unknown true` + `track_unknown_space true`，且提交的静态图是 72×23 m / 72×42 m **大面积 unknown** ⇒ "远距离穿空白/窄缝当走廊"在他们那儿**更严重**，没有可抄的策略；
2. **动态障碍**：无跟踪、无预测、无 ESDF，只有 STVL 衰减；
3. **重定位鲁棒性**：master 只有 small_gicp（硬编码 PCD），rmul2026 干脆没有（纯 lifelong 建图）；**我们有 4 种可选**；
4. **任务层**：目标选择是 bash 轮询裁判串口；**没有航点图/拓扑/前沿探索/调度**；
5. **测试/CI**：仅编译；他们自己写"参数调优是主要开发活动""没有测试设施"；
6. **rmul2026 没有仿真**；工程卫生较差（见上表）。

## 4. 一个重要旁证：他们的 `fake_vel_transform` **有我们已经修掉的那个 bug**

两个分支的 `fake_vel_transform` 都是无条件 `aft_tf_vel.angular.z = spin_speed_`，`spin_speed` 默认 **0.0**，而**仓库内没有任何 launch 给它赋值** ⇒ **在仓库自带配置下，Nav2 的每一条转向指令都被清零**（车只能靠自转/被外部 launch 覆盖才能转）。他们另配 `enable_rotation: false` + `yaw_goal_tolerance: 6.28`，即刻意做"只控位置"的导航。

⇒ 两点：① **我们对该 bug 的定位与修复是对的**（并有独立旁证）；② "放弃朝向控制"是哨兵的一种合理设计，正对应我们 `spin_speed != 0` 的小陀螺模式。

## 5. 待确认（本次无法验证）

1. 真机 RMUC/RMUL 2025 实际跑的是哪套（`nav2_params.yaml` 是 launch 默认，但 README 与 2026 的 `CLAUDE.md` 都说 MPPI）；
2. `spin_speed` 是否被**仓库外**的 launch 设成非零（只能证明仓库内没设）；
3. `robot_base_frame` 用 fake 帧后，朝向在 `map→odom` 与小陀螺解耦之间如何维持（与我们的结构同源，值得警惕）；
4. 真机雷达外参（xacro 写 `0.0 -0.045 0.3`，而 bringup 里静态 TF 又发 z=0.5，且 `robot_state_publisher` 被注释）；
5. 2026 真机是否有深度相机（STVL 里配了 RealSense 但不在 `observation_sources`）；
6. 性能证据（无 bag、无基准、无实测数据）。

## 6. 建议的落地顺序（一次一个变量，P0 回归做闸门）

1. **裁剪盒去车体点云** → 喂 stvl/local cloud（打"近距盲区"这条，风险最低、收益最直接）；
2. **`p2l.range_max: 10 → 20`**（打"远距缺墙"，一行）；
3. **局部也上 STVL（带衰减）**；
4. 再考虑 **Smac2D + Savitzky-Golay**、**MPPI Omni**（两条都要调参，放在最后）；
5. 每步都用 `python3 tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5` 验收，并把结论写回 `algorithm_matrix.md §四`。


---


## 来源 15/19：`tdt/cod_nav_design_thinking.md`


> 素材：`docs/cod_nav_comparison.md`、`cod_nav_2026_deep_dive.md`、`cod_nav_2026_params_review.md`、`cod_nav_macro_integration.md`、`cod_nav_nav_stack_inventory.md`、`cod_nav_open_source_materials.md`、`slam_nav_goal_strategies.md`、`nav_strategy_taxonomy.md`
> 参考代码：`third_party/cod_nav_2026/`（Gitee 成品 `master@0127e200`，只读）

## 0. 一句话总纲

COD 2026 的思路是 **「做减法」**：把导航拆成 **在线建图 + 纯 LIO 定位 + 实车调过的 MPPI + 示教航点任务层**，主动砍掉先验图重定位、地面分割、多余插件；**用"可复现的物理启动条件"替代"定位对齐"这一层**。

## 1. 逐层思路（做法 → 理由 → 代价）

| 层 | 他们的做法 | 理由（有据可查） | 代价 |
|---|---|---|---|
| 传感器 | 只留 MID-360（2026 配了 RealSense 但**没进 `observation_sources`**） | 单雷达够用、减依赖 | 近距/自身点云问题 |
| 感知预处理 | **车体裁剪盒**（`cpp_lidar_filter`，CropBox，`negative: true`）扣掉自身点云；**删掉地面分割**（patchwork++），p2l 直接吃原始点云 + 高度带 `0.10~1.00`、`range 0.5~20` | README 原话：**「STVL 无法设最小障碍半径」** ⇒ 只能从点云里抠掉车体；用高度带代替地面分割更省 | 高度带会砍低矮/近场回波；裁剪盒数值与其 marker 帧不一致、`leaf_size` 是死参 |
| 里程计 | **Small Point-LIO**（ACE 的 Point-LIO MIT 重制，官方口径快 2–3×），发 `/Odometry`、`odom→base_link` | **唯一有据可查的理由是"速度/算力"**；「高动态/自旋更强」被反证（不做去畸变，且他们从不自转） | 需 `tag`+`timestamp` 字段；`twist` 全零 |
| **定位（特殊层）** | **静态 `map→odom`**（identity + z=0.05，单点再带 yaw=−0.5）；`slam_toolbox` 的 `transform_publish_period: **0.0**`（`slam_toolbox_common.cpp` 首行直接 return ⇒ **结构上不发**）；**无 AMCL** | 作者原话：**「环境较为简单且面积较小，定位的误差容忍度很大，所以采用了纯 lio 的定位方法」** + KISS「发布坐标系静态转换比用 urdf 更简单」 | **必须从固定摆位起飞**；漂移直接变图畸变；被搬动/换场地即失效 |
| 建图 | 在线 **slam_toolbox async lifelong**（`map_update_interval 1.0`）+ `auto_save_map.launch.py` 在线存图；`map_server` 加载自己存的图 | 图是**规划资产**，不是定位资产 | 图滞后/畸变没有定位层纠正 |
| 代价图 | 全局 25×25@0.04 `rolling_window: true`；局部 14×14@0.05（含 static）；**两层都 STVL**（`voxel_decay 0.5` 线性、`voxel_size 0.05`、`model_type 1`、`obstacle/raytrace 8/9`） | 体素层给**时间维**，幽灵障碍 ~0.5 s 自动消失 | 清除过激会擦薄结构 |
| 全局规划 | **SmacPlanner2D** + `cost_travel_multiplier 4.0` + `tolerance 0.5` + `allow_unknown: true`；平滑靠 **Smac 自带 `smooth_path`**（`smoother_server` 在他们链里是**死代码**） | 代价感知 ⇒ 路径居中代价谷底（治"贴缝走"） | 未知区可规划（与我们"未知=不可走"相反） |
| 局部控制 | **MPPI Omni**（实车 ±2.5 m/s、wz ±1.5、`time_steps 80`、`batch 1500`、GoalCritic 25/4.0、`temperature 0.25`、`gamma 0.008`、`CostCritic critical_cost 253`）+ **`goal_approach_controller`** 代理（`approach_distance 2.5` 限速 0.2 m/s；`direct_approach_distance 2.0` 内**绕过 MPPI 直接 P 控**并把 `angular.z=0`） | 免梯度采样控制；梯度依赖"陡"的膨胀层 —— 他们的注释：**「原 0.3 衰减太慢，整个膨胀区代价都极高，MPPI 没有梯度可用」**；"绕圈"的正解是"近目标限速 + 直接驱动" | wrapper 有**帧 bug**（map 与 odom 混帧相减）；`angular.z=0` 与自转互斥 |
| 任务层 | **手教航点 CSV（6 条序列）+ 决策按局势选路 + `NavigateThroughPoses`（多点连续穿越）**；`waypoint_editor` 在 RViz 点选 | 比赛局势有限、路径短 ⇒ 示教比在线决策便宜且可靠 | **换场地必须重教**；`wps/*.csv` 无程序自动读取 |
| 停止语义 | `PositionGoalChecker`（xy 0.2 + path_length 0.5，**不看朝向**）+ 多点 `yaw_goal_tolerance 6.28`；配合 FVT **无条件**把 `angular.z` 写成 `spin_speed`（默认 0） | 哨兵只关心"到位"，朝向交给云台/自转 | `wz` 被丢弃（MPPI 的 wz 实际无效） |
| 决策/下位机 | 按血量阈值 210/350 选目标串；下位机功率 RLS 拟合 + 热量控制 | 机动受功率/热量约束 | 不在导航层 |

## 2. 定位层为什么「特殊」—— 它把算法问题转成了纪律问题

1. **不是"弱化定位"，而是"取消定位层"**：不重定位 ⇒ 用**每次摆在同一个物理位置、同一朝向**来保证 `map` 帧与物理场地对齐。
2. **三段硬证据**：① 静态 TF 是唯一 `map→odom` 发布者（slam_toolbox 因 `transform_publish_period: 0.0` 结构上不发）；② `PositionGoalChecker` + `yaw 6.28` + FVT 覆盖 `angular.z` ⇒ 整套**只控位置**；③ 作者原话 + KISS。
3. **在文献/工业界都有对应**：teach-and-repeat **不需要显式定位**（arXiv:1711.05348）；工业界 Brain Corp 的 Teach & Repeat 同样是"从 home marker 起跑、布局一改就重教"。
4. **代价可证伪**：起点依赖 · 里程计漂移 · 换布局必须重教 · 基线**假设参考轨迹周围是空的**（被遮挡会跟丢/碰撞，arXiv:2201.03938）。
5. ⇒ **特殊点在于**：它以"**小场地 + 短时间 + 严格摆位**"为前提，用物理纪律替代在线对齐；前提一破，整套失效。

## 3. 为什么"哪都省了"还能跑（内在自洽性）

```
省掉重定位        ⇒ 少一整类失败（定位串了；我们踩过"假到达"）
省掉地面分割      ⇒ 少一个参数面
用示教序列代替决策 ⇒ 少一套任务层算法
用固定摆位代替对齐 ⇒ 少一套地图资产管理（auto_save_map 自己长图）
────────────────────────────────────────────────
全部代价压在一个前提上：必须复现物理条件
```

## 4. 对我们 D3 / D4 验证的启示

**D3（可抄）**：裁剪盒 · **STVL 双图带衰减** · **Smac2D 的 `cost_travel_multiplier 4.0`** · **MPPI 实车参数（速度降到 2.0 m/s 起，含三个硬约束改造）** · `goal_approach_controller`（**先修帧 bug + 加 spin 开关**）· `PositionGoalChecker`。

**D4（抄形态，不抄定位层）**：任务层形态 = **CSV 航点 + `NavigateThroughPoses` + 局势选路**；但**航点必须落在我们的 `map` 帧**（我们有重定位 ⇒ 车随便摆，反而**免重教**，比他们更通用）。

**明确不抄**：静态 `map→odom` / `rolling_window + allow_unknown` 的"未知可规划" / 去掉地面分割 / 直接 vendor `small_point_lio` / Jazzy-only 键 / 硬编码路径。

## 5. 一句话对照

**COD：用"可复现的物理条件"换系统的简单。**
**我们：用"定位与仿真能力"换系统的通用。**
两套都自洽 —— 所以 D3/D4 应该抄他们的**感知预处理、控制器参数、任务层形态**，而**保留我们自己的定位层**（这也是我们比他们更适合"换场地/换起点"的原因）。


---


## 来源 16/19：`tdt/cod_nav_macro_integration.md`


> **本文定位**：回答两个具体问题（Q1 `small_point_lio` 的出处与 COD 换它的理由；Q2 COD 为什么用「静态 `map→odom`」），
> 并在此基础上给出**宏观能力地图（§C）**与**融入我们架构的宏观视图（§D）**。
> **本文是本次唯一被创建/修改的文件**；全程只读调研，未执行任何上游代码，未改动任何配置。
> **引用约定**：本地路径一律给 `file:line`；外部事实一律给 URL。**FACT（事实）与 INFERENCE（推断）在每一节显式分离**，
> 推断一律带「**推断**」标记与依据；读不到/无法确证的东西集中在 §E。
>
> **版本基准（关键，先说清）**：
> | 对象 | 版本 | 证据 |
> |---|---|---|
> | 本地参考栈 `third_party/cod_nav_2026/` | **Gitee `codnavgation/cod_-rm2026_-navigation` @ `master`，HEAD `0127e200`（2026-04-06）** —— 即 RMUL2026 **赛后成品**，10 个包、190 个文件 | `git -C third_party/cod_nav_2026 log -1` → `0127e2007d16a53343db557beee7524a45cb5d98 2026-04-06 fix: 更新正确的地图文件名`；`git -C third_party/cod_nav_2026 remote -v` → `https://gitee.com/codnavgation/cod_-rm2026_-navigation.git` |
> | GitHub `qza36/COD_NAV` 分支 `rmul2026` | HEAD `485f2ebfa`（2026-03-10）—— **季前**状态，包名是 `nav_bringup` 不是 `cod_bringup` | <https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100> |
>
> ⚠️ 两者是**同一作者的先后两版**：Gitee 成品把静态桥的 yaw 从 `0.0` 改成 `-0.5`、把 MPPI 从 7.5 m/s 收敛到 2.5 m/s、
> 并新增 `goal_approach_controller`（详见 `docs/cod_nav_nav_stack_inventory.md` §B2）。
> **凡两者不一致处，本文以 Gitee 成品为准并同时标出 GitHub 季前值。**
> 既有文档（`docs/cod_nav_comparison.md`、`docs/cod_nav_2026_deep_dive.md`、`docs/cod_nav_2026_integration_plan.md`、
> `docs/cod_nav_nav_stack_inventory.md`、`docs/cod_nav_2026_params_review.md`）已覆盖的内容，本文**不重复展开**，
> 只做**宏观判定**并指向它们。

---

## A. `small_point_lio` 是什么 + COD 为什么换它

### A0. 任务里给的 BBS 帖（813022）—— 正文提取成功，摘要如下

- **URL**：<https://bbs.robomaster.com/article/813022?source=4>（`?source=4` 与不带参数返回同一份 1,763,733 B 页面）
- **提取方法（FACT）**：正文**不在 SSR HTML 里**，而在页面尾部的 `window.__NUXT__` payload（`<script>` 内联，位于页面偏移 ≈1,713,688 B）中的 `article.htmlContent` 字段；
  它是**转义后的 HTML 串**（`\u003C` 等），需先做 JS 反转义再剥标签。本文提取到正文 **8,142 字符（带标签）/ 约 4,000 汉字（纯文本）**。
- **标题（FACT，取自 `<title>` 与 payload 的 `title` 字段）**：
  「【RM26赛季定位算法开源-东莞理工学院-ACE战队】Small Point-LIO，一个比 Point-LIO 快 2 - 3 倍的里程计」
- **作者（FACT）**：帖内自述「本人大二时负责 25 赛季哨兵导航，目前大三，在 26 赛季同样在做导航相关技术研发」；
  标题署名**东莞理工学院 ACE 战队**；`small_point_lio/package.xml:7,9` 的 maintainer/author 是 **Yingjie Huang `1709185482@qq.com`**，
  `LICENSE.txt` 头行 `Copyright (c) 2025 Yingjie Huang`。⇒ 作者 = ACE 战队的 Yingjie Huang（Yancey）。
- **项目性质（FACT，原文）**：「Small Point-LIO 是 Point-LIO[1] 的**重制版**，以 **MIT 协议开源**。在相同参数下，性能大约**比原代码快 2 - 3 倍**。」
- **仓库 URL（FACT，原文给了两个）**：
  - GitHub：<https://github.com/Yancey2023/small_point_lio>
  - 国内镜像：<https://www.yanceymc.cn/gitea/Yancey/small_point_lio>
  - QQ 群 1070252119。
  - **API 确权（FACT）**：`https://api.github.com/repos/Yancey2023/small_point_lio` → `full_name=Yancey2023/small_point_lio`、
    `license=MIT`、`default_branch=ros2`、`created_at=2025-10-09`、`pushed_at=2026-08-31`、`stargazers_count=121`、`language=C++`。
    ⇒ `docs/cod_nav_2026_deep_dive.md:71` 里「同名公开仓库**很可能是** `Yancey2023/small_point_lio`（推测）」这条**本次已从推断升级为事实**。

#### A0.1 帖子里**逐条说了什么**（原文要点，均为 FACT；「→」后是本文的机制解读，标 INFERENCE）

| # | 帖子原文要点 | 机制解读（INFERENCE） |
|---|---|---|
| 1 | **数据预处理重构**：原 Point-LIO 有 4 种过滤同时进行（`tag` 置信度、`blind`/`det_range` 距离、`point_filter_num` 抽点、`filter_size_surf` 体素），且逻辑**分散在代码各处** | 这是它**逐点（per-point）**处理的直接后果——过滤必须在点进入队列前做完 |
| 2 | 「原代码包含了 IMU 预处理，这部分代码原本是 **Fast LIO 用来做点云去畸变的**，但是 **Point-LIO 是逐点更新的，不需要进行点云去畸变**」 | **它不做 scan de-skew**；去畸变由「每个点各自带时间戳 + 逐点 ESKF 预测」隐式替代。⇒ 与 FAST-LIO 的「一帧内 IMU 积分去畸变 + 帧级 iterated-EKF」是**两条不同路线** |
| 3 | 「原代码缓存了**点云队列**，但 Point-LIO 是逐点更新的，不应该使用点云队列，而应该使用**点队列**」；「接收到点云时就过滤，把点和 IMU 存储在**按时间排序的点队列和 IMU 数据队列**中」 | 时间序由**逐点时间戳队列**保证 ⇒ IMU 与点的融合是「按时间戳归并两个队列」，**对 IMU 频率不敏感**（下面是它的证据引用） |
| 4 | 体素降采样「在 **small_gicp** 的降采样代码基础上修改，使降采样之后的点**保留时间戳属性**，且性能比 PCL 快 1.3 倍」；并**额外保留降采样前的稠密点队列**，在后续流程做去畸变后**发布到话题** | 内部地图用降采样点（省算力），**对外可发去畸变后的稠密点云**——两者是不同的队列 |
| 5 | **地图改进（它自称性能优秀的关键）5 条**：①邻居候选列表**超 5 个时才算距离**（省 40–60%）；②自研哈希 `hash_position_index`（纯位运算，省 30–40%）；③`std::unordered_map` → `ankerl::unordered_dense::map`（省 10–20%）；④每栅格**只存一个点**；⑤LRU 缓存**命中也要更新**以提升稳定性 | 地图是 **iVox 哈希体素**（Faster-LIO 路线），不是 FAST-LIO 的 ikd-Tree。作者自己说「我们一般会把里程计地图分辨率设置在 **0.1 m 以上**」 |
| 6 | **滤波器改进**：原 Point-LIO 「相同时间的点会一起更新……观测矩阵维度不固定」；作者发现 Mid-360 过滤后**同一时刻的点极少**，于是「把相同时间的点**分多次更新**，把观测矩阵维度改为**固定值**」（`Eigen::Dynamic` → 具体数字，省 20–30%）；且「**没有使用 IKFoM 框架**，而是重头实现」 | 逐点更新 + **定维观测**是可读性与速度的双重收益 |
| 7 | **编译选项**：`-march=native`、`-ffast-math`、`-fno-math-errno`；并声明「我之前的性能对比都是**与加了该编译选项的 Point-LIO** 进行对比的」 | 2–3× 的对比基线是**公平的**（不是拿未优化的上游比） |
| 8 | 结语：动机是「Point-LIO 原代码中有很多**不合理的地方**」，要「让大家不再被原代码所折磨」 | 该项目的定位是**工程重写 + 提速**，不是新算法 |

#### A0.2 帖子**没有**说的东西（FACT，用于纠偏）

按关键词在提取出的正文里逐一计数，以下**全部 0 命中或与提问无关**：
`twist`(0)、`save_pcd`(0)、`timestamp`(0)、`map_resolution`(0)、`topic`(0)、`外参`(0)、`重定位`(0)、`回环`(0)、`loop`(0)、`deskew`(0)、`license`(0)。
「发布」只出现 2 次、「话题」1 次（都是第 4 条那句「发布到话题」）、「MIT」1 次。

⇒ 因此：
- **帖子没有列「published topics」清单**——`/Odometry`、`/cloud_registered`、`odom→base_link` TF、`/map_save` 服务这些**只能从仓库源码取证**（见 A2）。
- **帖子没有给地图分辨率默认值**——`map_resolution: 0.5` 只在 `small_point_lio/config/mid360.yaml:25`（见 A2）。
- **帖子没有谈「高速/自旋鲁棒性」**，也没有和 FAST-LIO 做任何对比；唯一对比对象是 **Point-LIO 原代码**，指标是**速度**（2–3×）。

### A1. 与 FAST-LIO / Point-LIO 的路线差异（以本地源码为证，FACT + 推断分离）

| 维度 | FAST-LIO（我们的默认槽 `lio:=fastlio`） | `small_point_lio`（COD 2026） | 证据 |
|---|---|---|---|
| 处理粒度 | **帧级**：一帧点云 + 帧内 IMU 积分 → iterated-EKF 更新一次 | **点级**：`while` 循环里按时间戳在「点队列 / 稠密点队列 / IMU 队列」三路归并，**逐点** `predict_state` + `update_point` | `third_party/cod_nav_2026/src/small_point_lio/src/small_point_lio/small_point_lio.cpp:86-144`（循环与三个分支）；BBS 第 2/3/6 条 |
| 去畸变 | 需要（IMU 预处理 + 帧内积分），FAST-LIO 参数里有 `blind`/`filter_size_surf` 一整套 | **不做**；稠密点队列在逐点流程里被搬到 odom 帧后发话题（对外是「已去畸变」的稠密云） | BBS 第 2 条；`small_point_lio.cpp:90-100`（稠密点搬 odom）、`:101-124` |
| 地图结构 | **ikd-Tree**（增量 k-d 树） | **iVox 哈希体素**（`SmallIVox`，`add_point` 增量插入；`map_resolution`） | `estimator.h:22`（`std::shared_ptr<SmallIVox> ivox`）；`small_point_lio.cpp:51,122`；BBS 第 5 条 |
| 观测方程 | 帧级 scan-to-map 残差 | 逐点找近邻 **5 点**拟合平面取最小特征向量做法向（`NUM_MATCH_POINTS=5`） | `docs/cod_nav_2026_deep_dive.md:72`（对 `estimator.cpp` 的逐行读取） |
| IMU 速率处理 | 有 buffer / 外推参数（`max_iterations`、`filter_size_*` 等） | **无外推参数**：IMU 回调与点云回调**各调一次** `handle_once()`；`publish_odometry_without_downsample: false` 时 odom 在「三队列都非空」的循环尾部发布 ⇒ **≈ IMU 频率** | `small_point_lio_node.cpp:197-211`（两个回调都调 `handle_once`）；`small_point_lio.cpp:146-156`；`config/mid360.yaml:50`；`docs/cod_nav_2026_deep_dive.md:106` |
| 回环 / PGO | **无**（FAST-LIO 本身也没有；我们靠 mapper 槽补） | **无**：`src/` 全目录 grep `loop_clos|pose_graph|pgo|scan_to_map|ikd` **0 命中** | `grep -rn -i "loop_clos\|pose_graph\|pgo\|scan_to_map\|ikd" third_party/cod_nav_2026/src/small_point_lio/src/` → 无输出 |
| 内部是否估速度 | FAST-LIO 的 `/Odometry.twist` 有值 | **内部有**：`common::Odometry` 带 `velocity` 与 `angular_velocity` 字段，`publish_odometry()` 里被填好 | `src/common/common.h:13-19`、`small_point_lio.cpp:169-175` |
| 对外是否发速度 | — | **不发**：ROS 侧 `odometry_msg.twist.twist.*` **五行全被注释**，留 `// TODO it is lidar_odom->lidar_frame, we need to transform it to odom->base_link` ⇒ **twist ≡ 0** | `small_point_lio_node.cpp:90-96` |
| 地图分辨率 | 我们 3D 图 0.1~0.2 m 量级（PCD） | `map_resolution: 0.5`（iVox 体素）；`save_pcd` 时另用 `PointcloudMapping(0.02)` 存 2 cm 全量云 | `config/mid360.yaml:25`；`small_point_lio_node.cpp:32` |
| `save_pcd` | 我们不常用 | **默认能力存在**：`save_pcd: true` 时在线累积；`ros2 service call /map_save std_srvs/srv/Trigger` 落盘 **`ROOT_DIR + "/pcd/scan.pcd"`**，`ROOT_DIR` 由 `include/param_deliver.h.in` 经 `configure_file` 编译期注入源码目录 | `small_point_lio_node.cpp:31-51`、`include/param_deliver.h.in`；`docs/cod_nav_2026_deep_dive.md:107`；Gitee 成品已把 `config/mid360.yaml:7` 改成 `save_pcd: false` |
| 发布/订阅（FACT，源码） | — | **订阅** `/livox/lidar`（PointCloud2，`SensorDataQoS`）、`/livox/imu`（`SensorDataQoS`）；**TF 查询** `lookupTransform(lidar_frame, "base_link")`（查不到就 `return`，**丢帧且不发 TF/odom**）。**发布** `/Odometry`（queue 1000）、`/cloud_registered`（**仅当订阅数 > 0 才发**）、TF `odom→base_link`（**节点自己发**，帧名**硬编码**）、服务 `map_save` | `small_point_lio_node.cpp:26-27,64-69,78-99,101-102`；`src/lidar_adapter/livox_pointcloud2.h:20-47` |
| `tag`/`timestamp` 适配（FACT，**移植硬门槛**） | — | `LivoxPointCloud2Adapter` 用 `PointCloud2ConstIterator<uint8_t>(msg,"tag")` 与 `<double>(msg,"timestamp")`，**只保留 `(*tag & 0b00111111) == 0` 的点**，时间戳**乘 1e-9 转秒**。字段不存在 ⇒ 迭代器构造即失败 | `src/lidar_adapter/livox_pointcloud2.h:25-37` |

### A2. 上游仓库与许可证（FACT）

| 项 | 值 | 证据 |
|---|---|---|
| COD 分支里的形态 | **vendored 目录**（含自己的 `.github/workflows/`、`.clang-format`、`.clang-tidy`、`3rdparty/`），**不是 submodule** | `third_party/cod_nav_2026/src/small_point_lio/`（42 个文件）；仓库根无 `.gitmodules` |
| 上游仓库 | <https://github.com/Yancey2023/small_point_lio>（MIT，默认分支 `ros2`，121★ / 22 fork，2025-10-09 建、2026-08-31 最后 push） | `https://api.github.com/repos/Yancey2023/small_point_lio` |
| 上游 README 关键句 | 「delivering a **2-3x speed improvement** over the original」「The default branch is for ROS2. If you want to run it without ros, please checkout the `main` branch」「If you want to know why it so fast, please read [this]」(链到 BBS 813022) | `third_party/cod_nav_2026/src/small_point_lio/README.md:3,5,7` |
| 许可证（上游） | **MIT**，`Copyright (c) 2025 Yingjie Huang` | <https://github.com/Yancey2023/small_point_lio> 的 `license.spdx_id = MIT`；`third_party/cod_nav_2026/src/small_point_lio/LICENSE.txt:1-3`；`package.xml:8` `<license>MIT</license>` |
| 第三方致谢 | Eigen(MPL2.0) / `ankerl::unordered_dense`(MIT) / `small_gicp`(MIT) / Open3D(MIT) | `README.md:90-95` |
| 注意（FACT） | **`small_gicp` 只在 CI 里被 apt/源码安装，`CMakeLists.txt` 并不 `find_package(small_gicp)`** ⇒ 运行期**不是依赖**；那份体素降采样是**抄进来的代码**（`src/util/voxelgrid_sampling.*` 文件头写明来自 koide3/small_gicp） | `docs/cod_nav_2026_deep_dive.md:72,325,494` |
| 拿到的另一种实现 | 上游仓库 README 另行给出「不用 ROS 请 checkout `main`」⇒ 上游按 ROS/非 ROS 分两支 | `small_point_lio/README.md:5` |

### A3. COD 为什么换掉 FAST-LIO —— FACT / INFERENCE 分离

**先立时间线（FACT，全部来自提交史 API）**：

| 时间 | commit | 消息 | 说明 |
|---|---|---|---|
| 2025-08-02 | `54dc95af4` | Merge PR #2 from `dyx-dev` | `master`（2025 赛季线）在此冻结；**master 用 FAST-LIO v2 fork + patchwork++ + small_gicp 重定位**（见 `docs/cod_nav_comparison.md:11`） |
| 2025-11-18 | **`9d54b5ab8`** | **`feat:replace fastlio2`** | **第一次删除 FAST-LIO**（删 `FAST_LIO/`），**同一批没有加入新 LIO** |
| 2025-11-18 | `093d13a66` | `feat:lio interface pub odom2base_link` | 让 LIO 接口发布 `odom→base_link`（即「LIO 自己发 TF」这个契约） |
| 2025-11-18 | `ffc4e24e3` | `feat:rmul2026 slam navgation param` | slam_toolbox 参数（含 `mode: lifelong`） |
| 2025-12-19 | `9856cff66` | `remove livox driver` | 继续做减法 |
| 2025-12-19 | `9dfbeabdd` / `6e03b83e0` | `feat:use stvl instead of voxel_layer` / `chore:add stvl to global map` | 双图 STVL |
| 2025-12-28 | **`d1c027958`** | **`add small point lio and ferfect mppi`** | **真正落地 `small_point_lio`（同时引入 MPPI）** |
| 2026-03-04 | `632fbb4eb` | `稳定在先验地图下到达增益点` | 砍掉全部重定位（删 `bringup.launch.py`、`nav2_params_amcl.yaml`、carto lua/pbstream），加 `nav.launch.py`(GitHub)/`singlenav_launch.py`(Gitee) |
| 2026-03-10 | `a7299b60a` → `485f2ebfa` | MPPI 落地 + 消震荡三轮 | GitHub `rmul2026` HEAD |
| 2026-04-06 | `0127e200` | `fix: 更新正确的地图文件名` | **Gitee 成品 HEAD（本地 `third_party` 快照）** |

来源：<https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100>；
Gitee：`https://gitee.com/api/v5/repos/codnavgation/cod_-rm2026_-navigation/commits`（本地 `git log` 亦得 `0127e200`）。

#### A3.1 FACT（能直接引用到证据的）

| # | FACT | 证据 |
|---|---|---|
| F1 | COD 的 `master`（2025）确实用 FAST-LIO fork；`rmul2026` 与 Gitee 成品都用 `small_point_lio`；Gitee 成品 10 个包里**没有** `FAST_LIO` | `docs/cod_nav_comparison.md:11-12`；`docs/cod_nav_2026_integration_plan.md:150`；`ls third_party/cod_nav_2026/src/` |
| F2 | 换 LIO 的**提交消息只写了 `replace fastlio2`**，没有任何理由描述；落地是 40 天后另一个 commit | `9d54b5ab8`、`d1c027958`（同上 API） |
| F3 | 仓库内**唯一**解释 `small_point_lio` 价值的文字是上游 README 的「**2-3x speed improvement**」，以及 BBS 帖标题「比 Point-LIO 快 2-3 倍的里程计」 | `small_point_lio/README.md:3`；BBS 813022 标题 |
| F4 | COD 的 `README.md` 把该包的作用写成 **`small_point_lio  #point_lio提供里程计，odom->base_link`** —— 定位是「**里程计源**」，不是「建图/定位一体」 | `third_party/cod_nav_2026/README.md`（仓库结构段） |
| F5 | 上游的收益口径**只有速度**，且基线是**同样开了 `-march=native` 的 Point-LIO**；帖子里**没有任何**「鲁棒性/去畸变/高动态/自旋」的对比 | BBS 813022 第 7 条 + §A0.2 的 0 命中统计 |
| F6 | `small_point_lio` **不做去畸变**（作者把 IMU 预处理称为「原本是 Fast LIO 用来做点云去畸变的……Point-LIO 逐点更新不需要」） | BBS 813022 第 2 条 |
| F7 | `small_point_lio` **没有回环/PGO**；COD 用 slam_toolbox 在 2D 侧补闭环 | `grep` 0 命中（A1 表）；`cod_bringup/launch/multiplenav_launch.py:85-94`（启动 `async_slam_toolbox_node`） |
| F8 | COD 把 LIO 的点云**降维后**才用于导航：`pointcloud_to_laserscan` 把 `/livox/lidar` 变成 `/scan`（`range 0.5~20.0`、`angle ±3.1416`、`increment 0.0087`），3D 云只经裁剪盒喂 STVL | `cod_bringup/launch/multiplenav_launch.py:65-84`；`singlenav2_params.yaml:221`（`topic: /livox/lidar_filtered`） |
| F9 | 他们的 LIO 消费的是 `livox_ros_driver2` 的 **PointCloud2（含 `tag`/`timestamp` 字段）**，这正是 Livox 官方驱动原生就带的字段 | `config/mid360.yaml:5` `lidar_type: livox_pointcloud2`；`livox_pointcloud2.h:28-29`；COD README 要求先装 <https://github.com/Livox-SDK/livox_ros_driver2.git> |
| F10 | `package.xml:22` 声明 `<depend>livox_ros_driver2</depend>`，但 `LIVOX_DRIVER` 只在 `lidar_type == "livox_custom_msg"` 时才需要（编译期 `HAVE_LIVOX_DRIVER`，找不到只 warning） | `small_point_lio/package.xml:22`；`small_point_lio_node.cpp:178-187`；`docs/cod_nav_2026_deep_dive.md:74` |

#### A3.2 INFERENCE（我的判断，逐条给依据与反证边界）

| # | 推断 | 依据 | 反证/边界 |
|---|---|---|---|
| I1 | **主因是「算力/帧率」而非「算法质量」**：7.5 m/s（GitHub 季前）到 2.5 m/s（Gitee 成品）的高速哨兵 + 机载算力，需要更省 CPU 的 LIO；2–3× 的**同基线**提速直接换算成可用的点云/IMU 频率 | F3 + F5 + `docs/cod_nav_2026_deep_dive.md:336`（B1 条同样推断） | **不能反证**：仓库里没有任何 CPU 占用/bag/基准数据；此推断只有文本证据 |
| I2 | **次因是「维护面收敛」**：2026 是一次整体做减法（删 submodule、删 livox driver、删地面分割、删重定位），换一个**自带完整工程配置**（PCH/OpenMP/CI/`.clang-format`）的包符合同一策略 | A3 时间线；`docs/cod_nav_2026_integration_plan.md:151` | 无直接文档，属合理外推 |
| I3 | **不是**因为在「高动态/自旋」下 Point-LIO 族更强而选的：他们**从不自转**（`spin_speed` 全分支无一处非零），且帖子不谈鲁棒性 | §A0.2 0 命中；`docs/cod_nav_2026_deep_dive.md:292`、`docs/cod_nav_comparison.md:60` | — |
| I4 | **不是**因为「需要 PCD 先验/重定位」而选的：`save_pcd` 在 Gitee 成品里被改成 `false`，且整个 2026 路线**没有重定位** | `config/mid360.yaml:7`；`cod_bringup/launch/localization_launch.py:44`（`lifecycle_nodes = ['map_server']`，无 amcl） | — |
| I5 | **不是因为去畸变质量**：它明确不做 de-skew，靠逐点时间戳隐式补偿；在 7.5 m/s + 100 Hz IMU 下这恰好够用（每点都有 IMU 预测），但在**帧率低/IMU 稀疏**时会退化 | F6 + `small_point_lio.cpp:86-144` | 属机理解读，**未实测** |
| I6 | **`livox_pointcloud2` 适配器的字段要求在他们那里不是成本**（官方驱动本来就发 `tag`/`timestamp`），但**对我们仿真是硬门槛** | F9 vs 我们 `/livox/lidar` 是 CustomMsg、`/livox/lidar/pointcloud` 是普通 PointCloud2（`docs/debug_fastlio_cartographer.md` §10.0 雷达话题行） | 见 §A4 |

### A4. 结论：我们要不要 vendor `small_point_lio`？

| 判定 | 内容 | 依据 |
|---|---|---|
| **不能直接融（硬门槛）** | `LivoxPointCloud2Adapter` 要求 `tag`(uint8) + `timestamp`(float64) 字段，否则迭代器构造失败；我们仿真 `/livox/lidar` 是 **CustomMsg**、`/livox/lidar/pointcloud` 是**普通 PointCloud2**（无这两个字段） | `livox_pointcloud2.h:25-37`；`docs/debug_fastlio_cartographer.md` §10.0 |
| **不必融（同族已存在）** | 我们已有 `lio:=pointlio`——`src/localization/rm_localization/point_lio/`（仓库内已 checkout 的可编译源码，含 `CMakeLists.txt`/`config/`），其上游在 `.gitmodules` 里登记为 `https://github.com/William-kk985/Point-LIO.git`（即 hku-mars Point-LIO 的自有 fork）。`small_point_lio` 是**同一族**（Point-LIO）的工程提速版，**算法结论不会不同**，只是 CPU 开销不同 | `docs/algorithm_axes.md:48-54`；`.gitmodules`（`[submodule "src/localization/rm_localization/point_lio"]`）；`ls src/localization/rm_localization/point_lio/` |
| **若真要融，三条路** | ① 编译期装 `livox_ros_driver2` 走 `livox_custom_msg` 适配器（`-DHAVE_LIVOX_DRIVER`）；② 给仿真点云**补 `tag`/`timestamp` 字段**（自研小节点，把 CustomMsg 转成带这两字段的 PointCloud2——**这与我们已修好的 CustomMsg 链路直接冲突，不建议**）；③ 只当参考实现读 | `small_point_lio_node.cpp:178-187`；`CMakeLists.txt`（`find_package(livox_ros_driver2 QUIET)`） |
| **不该融的部分** | `save_pcd` 的落盘路径 = `ROOT_DIR + "/pcd/scan.pcd"` 且 `ROOT_DIR` 编译期写死源码目录；`map_resolution: 0.5` 比我们 0.05 粗 10 倍；**`twist ≡ 0`**（任何依赖 `/Odometry.twist` 的模块会读到 0） | `small_point_lio_node.cpp:31-51,90-96`；`config/mid360.yaml:25` |

**一句话（A 部分）**：`small_point_lio` 是东莞理工 ACE 战队 Yingjie Huang 对 Point-LIO 的 **MIT 工程重写**（逐点 ESKF + iVox + 定点/定维观测 + 编译期加速，官方口径**快 2–3×**，且基线是同编译选项的原版）。
COD 2026 换它，**有据可查的只有「速度」这一条**（README/帖标题）；「算力优先 + 做减法」是合理推断；「高动态鲁棒性/去畸变更好」**被证据反证**（它不做去畸变，帖子也不谈鲁棒性）。
对我们：**结论不变——同族用 `lio:=pointlio` 就够，不必 vendor**（与 `docs/cod_nav_2026_integration_plan.md:172` 一致，本文补上了 BBS 原文与上游确权两处证据）。

---

## B. 核心问题：COD 为什么用「静态 `map→odom`」

### B1. 证据清单（全部 `file:line` / URL，FACT）

#### B1.1 「谁在发 `map→odom`」——答案是**唯一一个静态发布器**

| # | 事实 | 证据（本地 Gitee 成品） | 证据（GitHub 季前，作对照） |
|---|---|---|---|
| E1 | **单点导航**：唯一发布者是 `tf2_ros static_transform_publisher`，节点名 `map_to_odom`，参数 `--x 0.0 --y 0.0 --z 0.05 --roll 0 --pitch 0 --yaw **-0.5** --frame-id map --child-frame-id odom` | `third_party/cod_nav_2026/src/cod_bringup/launch/singlenav_launch.py:67-89`（yaw 在 `:82-83`） | `nav_bringup/launch/nav.launch.py` 同款（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/nav.launch.py>） |
| E2 | **多点导航（在线 SLAM）**：唯一发布者是同一个静态节点，但 `--yaw **0.0**`（z 仍 0.05） | `cod_bringup/launch/multiplenav_launch.py:95-116`（yaw 在 `:109-110`） | `nav_bringup/launch/slam.launch.py:101-120`（**yaw 0.0**，<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/slam.launch.py>） |
| E3 | 全仓只有 **3 处** `static_transform_publisher`：上面的 singlenav、multiplenav，以及 `small_point_lio/launch/small_point_lio.launch.py:24-45` 的 `base_link→livox_frame`（**而该 launch 从未被 cod_bringup 引用**，各 launch 是直接起 `small_point_lio_node`） | `grep -rn "static_transform_publisher" third_party/cod_nav_2026/src/`（3 命中） | `docs/cod_nav_2026_deep_dive.md:70`（该分支无 `.gitmodules`） |
| E4 | `slam_toolbox` **结构上不可能**发 `map→odom`：`publishTransformLoop()` 第一行就是 `if (transform_publish_period == 0) { return; }`；而他们的参数正是 **`transform_publish_period: 0.0`** | `src/localization/rm_localization/slam_toolbox/src/slam_toolbox_common.cpp:251-259`（这是**本机同版本源码**，与我们/他们都在用的 slam_toolbox 一致）；他们的参数 `cod_bringup/params/mapper_params_online_async.yaml:30` | `nav_bringup/params/mapper_params_async.yaml` 同值（`docs/cod_nav_2026_deep_dive.md:119`） |
| E5 | **没有 AMCL**：定位 launch 的 `lifecycle_nodes = ['map_server']`；`singlenav2_params.yaml` 从第 1 行 `bt_navigator:` 开始，**全文件无 `amcl:` 段** | `cod_bringup/launch/localization_launch.py:44`；`cod_bringup/params/singlenav2_params.yaml:1`（且 `grep -i amcl` 两套 params 均 0 命中） | `docs/cod_nav_2026_deep_dive.md:116-117`（对 GitHub 版同结论）；`632fbb4eb` 删除了 `nav2_params_amcl.yaml`、`carto.config.lua`、`map/*.pbstream` |
| E6 | **launch 里没有任何初始位姿注入**：`singlenav_launch.py` 全文没有 `initialpose`、没有 `amcl_initial_pose`、没有 `map_start_pose` | `cod_bringup/launch/singlenav_launch.py`（154 行全文） | `docs/cod_nav_2026_deep_dive.md:123`（「`nav.launch.py` 并没有任何初始化位姿的东西」） |
| E7 | `localization_launch.py` 默认地图 = **`cod_bringup/maps/rmul2026.yaml`**（Gitee 成品）；`singlenav2_params.yaml:360` 里的 `map_server.yaml_filename` 是一条**写死作者机器路径**的死值（`/home/cod-sentry/dyx_ws/...`），会被 launch 的 `map` 参数覆写 | `cod_bringup/launch/localization_launch.py:76-79`（default_value）与 `:56-58`（`param_substitutions` 把 `yaml_filename` 换成 `map_yaml_file`）；`singlenav2_params.yaml:355-360` | GitHub 版写死 `/home/cod-sentry/qza_ws/...`（`docs/cod_nav_2026_deep_dive.md:121`） |

#### B1.2 先验地图 / 在线建图的参数（FACT）

| 参数 | 值 | 证据 |
|---|---|---|
| 先验图 `image` / `resolution` / `origin` | `rmul2026.pgm` / **0.05** / **`[-1.756, -7.036, 0]`**（yaw=0） | `third_party/cod_nav_2026/src/cod_bringup/maps/rmul2026.yaml:1-7` |
| 先验图栅格尺寸 | PGM header `P5 255 177 255` ⇒ **12.75 m × 8.85 m** | `python3` 解析 `cod_bringup/maps/rmul2026.pgm` 头（本文实测）；`docs/cod_nav_nav_stack_inventory.md:132`（255×177 ⇒ ≈12.75×8.85 m） |
| `slam_toolbox` 模式 | `mode: lifelong`（注释里还留着 `#localization`） | `cod_bringup/params/mapper_params_online_async.yaml:19` |
| `slam_toolbox` 更新节流 | `map_update_interval: 1.0`、`minimum_time_interval: 0.3`、`minimum_travel_heading: 0.1`、`max_laser_range: 10.0`、`min_laser_range: 0.2`、`resolution: 0.05` | 同上 `:31-34`（GitHub 季前是 `2.5 / 0.5 / 0.5 / 5.0 / 0.01`，见 `docs/cod_nav_2026_deep_dive.md:133`） |
| `map_start_pose` | **不存在**（被注释） | `cod_bringup/params/mapper_params_online_async.yaml:24-26`（`# map_start_pose: [0.0, 0.0, 0.0]`） |
| slam_toolbox 的 `base_frame` | `base_link`（**不是** `base_link_fake`） | 同上 `:16` |
| Nav2 的 `robot_base_frame` | `base_link_fake`（**全部** nav2 节点：`bt_navigator`、两张 costmap、smoother、behavior） | `singlenav2_params.yaml:5`（bt_navigator）、`:184`（local）、`:274`（global）、`:411`（smoother）、`:443`（behavior） |
| Nav2 的 odom 话题 | bt_navigator `odom_topic: Odometry`；velocity_smoother `odom_topic: "Odometry"`（Gitee 成品已修好拼写，GitHub 季前是 `"odomety"`） | `singlenav2_params.yaml:6,482`；`docs/cod_nav_2026_deep_dive.md:348` |
| 全局 planning frame | `bt_navigator.global_frame: map`；global_costmap `global_frame: map` + **`rolling_window: true`**（25×25 m @0.04） | `singlenav2_params.yaml:4,273,269-279` |
| 局部参考系 | local_costmap `global_frame: odom` + `rolling_window: true`（10×10 @0.05） | `singlenav2_params.yaml:183,186-189` |

#### B1.3 作者本人的原话（**这是 Q2 最直接的证据**）

BBS 帖 **1882897**《【RM2026-哨兵机器人导航、决策、自瞄、下位机控制全开源】辽宁科技大学-COD战队》
（<https://bbs.robomaster.com/article/1882897>，同样从 `window.__NUXT__` 的 `htmlContent` 提取，正文 18,304 字符（带标签）/ ≈6,079 汉字）「建图」段原文（FACT，逐字）：

> 「在没有先验 pgm 的情况下进行导航，nav2 官方的方案是边 slam 边导航：slamtoolbox + nav2
> 只要提供了 map->odom->baselink 的 tf，以及 /map 上有地图数据，nav2 就可以正常运行
> 所以：small pointlio 提供 odom->baselink 以及高频里程计，slamtoolbox 提供 /map 数据，**map 到 odom 静态发布 ‘0’,‘0’,'0'**;
> **由于联盟赛的环境较为简单且面积较小，定位的误差容忍度很大，所以采用了纯 lio 的定位方法**」

同一帖还有两句对本次分析很关键的话（FACT）：
> 「NOTE：一定要注意的是，不能一味去调上位机的速度，先要确保电控那边能正确把控制器的速度转换成轮速」
> 「这是我修改过的 small point lio，**可以输出速度**，感谢 ACE 战队的开源，真的非常鲁棒！」

⇒ **三点直接结论**：
1. 作者说的静态值是 **`0,0,0`**（帖子里写的是「地图到 odom」的三个平移量），与 `multiplenav_launch.py:99-110`（x=y=0、yaw=0）一致；
   而 `singlenav_launch.py:82-83` 的 **`yaw -0.5`** 是**成品里改出来的、帖子里没提的**一个额外偏航。
2. **作者明确给出了动机**：「环境简单 + 面积小 + 定位误差容忍度大」⇒ 直接回答了「为什么敢不用重定位」。
3. 作者说他们「**修改过的 small point lio 可以输出速度**」——这句话必须与源码对照：**COD 仓库里的 `small_point_lio_node.cpp:90-96` 仍然是注释掉的 twist**。
   ⇒ **FACT：COD 公开仓库里发布出来的 `/Odometry.twist` 恒为 0**；作者所说的「能输出速度」要么是**仓库外的改动**、要么是**指内部速度被用于其它目的**、要么是**RViz 里画的是内部速度**。
   **本文只能确证「公开快照的 twist 为 0」**，其真机是否另有版本**无法确定**（记入 §E）。

### B2. 动机假设分析：逐条判「支持 / 推测 / 被反证」

> 判定口径：**SUPPORTED** = 有作者原话或可核对的机制证据；**PLAUSIBLE** = 与证据一致但无直接证据；**CONTRADICTED** = 有反证。

| # | 候选原因 | 判定 | 证据与理由 |
|---|---|---|---|
| **(a)** | **纯在线 SLAM、无先验地图 ⇒ `map` 原点天然 == 启动位姿，静态桥「不花任何代价」** | **SUPPORTED** | ①`multiplenav_launch.py` 里没有 `map_server`（它只 include `navigation_launch.py`，`:138-147`），也没有 `localization_launch.py` ⇒ 在线模式下 `/map` 的唯一来源是 slam_toolbox（`:85-94`）；②静态桥 yaw=0、x=y=0（`:99-110`）⇒ map≡odom；③slam_toolbox 的首帧位姿在 karto 里就是原点。**这是 (a) 的完整闭环**，且作者原话「在没有先验 pgm 的情况下……只要提供了 map->odom->baselink 的 tf …… 静态发布 0,0,0」正是这个意思 |
| **(b)** | **避免 slam_toolbox 回环/位姿图修正带来的 `map→odom` 跳变扯动 global costmap / MPPI / 自转底盘** | **CONTRADICTED（对「避免位姿跳变」这半边）+ 部分 SUPPORTED（对「不希望 SLAM 管位姿」这半边）** | **被反证的部分**：`transform_publish_period: 0.0`（`mapper_params_online_async.yaml:30`）+ `slam_toolbox_common.cpp:255-257` ⇒ slam_toolbox **根本不发** `map→odom`，所以**不存在「SLAM 的位姿跳变」这个现象可被避免**。真正跳变的是 **`/map` 栅格本身**（`static_layer` 订阅 `/map`，`map_update_interval: 1.0` ⇒ 每秒可能重排一次栅格），而这条**静态桥并没有解决**。<br>**支持的部分**：把 `transform_publish_period` 设为 0 意味着「让 SLAM 的修正**只体现为地图变化、不体现为位姿变化**」——这确实是一种「不让 SLAM 动 TF」的设计，但它省掉的是**位姿连续性**而不是**图一致性**。<br>**另一条被反证的子假设**：他们「为了自转底盘不跳」——他们**从不自转**（`spin_speed` 全仓无一处非零，见 `docs/cod_nav_comparison.md:60`、`docs/cod_nav_2026_deep_dive.md:292-293`），所以「避免扯动自转底盘」这个动机**不成立**（底盘自转是下游 `cod_serial_ul26` 的事，与 TF 无关） |
| **(c)** | **主动砍掉 AMCL/重定位，以消掉一整类失败源** | **SUPPORTED（动机）+ SUPPORTED（事实）** | 作者原话「由于联盟赛的环境较为简单且面积较小，定位的误差容忍度很大，所以采用了**纯 lio 的定位方法**」；`632fbb4eb`「稳定在先验地图下到达增益点」同批删掉 `bringup.launch.py` / `nav2_params_amcl.yaml` / carto lua / `*.pbstream`（`docs/cod_nav_2026_deep_dive.md:118`）；`localization_launch.py:44` 只留 `map_server`。<br>⇒ **动机是「容忍度大 + 不值得调」**（**推断**：AMCL/ICP 的调参时间成本 > 收益），而**不是**「他们试过 AMCL 失败」——后者无证据 |
| **(d)** | **简单/人手/赛季带宽** | **SUPPORTED（README 自述）+ 部分为推断** | README 原文：「此导航程序的设计原理保持 **Keep It Simple Stupid** 的原则，**发布坐标系静态转换比运用 urdf 维护更简单易操作**」（`third_party/cod_nav_2026/README.md`「注意」段）。这解释了**为什么用静态 TF 而不是 URDF/动态 TF**，但**没有**说明为什么不要重定位；「人手/带宽」属**推断**（`docs/cod_nav_2026_deep_dive.md:337-338` 亦为推断） |
| **(e)** | **配置气味 / 潜在冲突（slam_toolbox 本来也会发 `map→odom`，静态桥是「抢发布者」）** | **部分成立，但与被反证的那半边不同** | **FACT**：slam_toolbox 的 `transform_publish_period: 0.0` **正是**为了「不发」，所以**没有双发布者冲突**——这一点他们做对了（对比我们自己的 `docs/tf_interface_contract.md` §二 P1：我们曾经真的踩过「静态桥 + AMCL」多父边）。<br>**但**：①`singlenav_launch.py:83` 的 `yaw -0.5` 是一个**没有任何注释/文档解释的魔数**；②`multiplenav_launch.py` 同时启动 slam_toolbox（`:85-94`）**和**静态桥（`:95-116`），一旦有人把 `transform_publish_period` 改成非 0 就**立刻**变成双发布者；③`singlenav2_params.yaml:360` 的 `map_server.yaml_filename` 写死作者机器路径 ⇒ **复制配置到新机器会静默加载失败（或加载错图）**；④`localization_launch.py:76-79` 的默认图是 `maps/rmul2026.yaml`，**但它在多点模式下从不被启动** ⇒「先验图」这条路在成品里只服务单点模式。<br>⇒ 判定：**「静态桥」本身不是气味；但「静态桥 + 无注释魔数 + 写死路径 + 依赖 0.0 这一个参数维持唯一发布者」的组合是脆弱配置** |

### B3. 后果与风险（静态 `map→odom` 的代价，FACT 主导 + 明确标推断）

| # | 后果 | 机制（FACT） | 对我们的具体含义（INFERENCE，标出） |
|---|---|---|---|
| R1 | **`map` 系完全继承 LIO 的长期漂移；SLAM 的回环修正无法回头修正机器人位姿 ⇒ 漂移变成「地图畸变」** | 静态桥 ⇒ 机器人 map 位姿 ≡ LIO odom 位姿（`singlenav_launch.py:67-89` + `small_point_lio_node.cpp:78-99`）；slam_toolbox 的位姿图优化改了 `map_to_odom_`，但它**不发那条 TF**（`slam_toolbox_common.cpp:255-257`） | ⇒ 长距离跑动后，**方向/距离误差只能靠 `/map` 栅格不断重排来「吸收」**，表现为「地图在机器人脚下平移/旋转」。**推断**：这是他们能接受的原因之一——联盟赛场地小、单局时间短 |
| R2 | **机器人必须从「先验图的位姿/朝向起点」出发**（单点模式）；否则整张图在 map 系里整体错位 | `singlenav_launch.py` 无任何 `initialpose`/`map_start_pose`（E6）；静态桥固定 `x=y=0, z=0.05, yaw=-0.5`（`:72-83`）；先验图 `origin [-1.756, -7.036, 0]`（`maps/rmul2026.yaml:4`） | ⇒ **对我们的先验图同样成立**：我们 `src/bringup/rm_nav_bringup/map/RMUL2026.yaml:4` 的 `origin` 是 **`[-2.2, -3.15, 0]`**，PGM 是 **249×173**（12.45 m × 8.65 m，本文实测）。若要用静态桥，机器人**必须**在「采集该图时的起点 + 朝向」启动，且 odom 的 yaw 零点要对齐 |
| R3 | **`yaw -0.5` 与地图 `origin` 的关系无法从仓库自证** | `maps/rmul2026.yaml:4` 的 `origin` 第三项是 **0**（原点处朝向 = map 系 +x）；静态桥给 `odom→map` 之前先转 **-0.5 rad（≈-28.6°）**（`singlenav_launch.py:82-83`，全仓无注释） | **推断**：该 -0.5 是**手工标定常量**，用于补偿「先验图并非从 `origin` 处 yaw=0 起飞」或「场地摆放朝向」；因为**成品是单点模式**（用先验图），而**作者帖子只提 0,0,0**，二者矛盾只能在真机标定中解释。**我们无法确定其物理含义**（记入 §E）。<br>⚠️ 另注：GitHub 季前的 `t1.yaml` origin 是 `[-3.84, -6.05, 0]` 而 yaw 也是 0（`docs/cod_nav_2026_deep_dive.md:120-121`）⇒ **换了地图却没改 yaw=0**；到成品换 `rmul2026.yaml` 时才变成 -0.5。这条**支持「-0.5 是跟图走的标定值」**的推断 |
| R4 | **Nav2 目标语义被绑死在「图的原点系」**：`/goal_pose` 的坐标是 map 坐标，而 map 与 odom 同源 ⇒ 目标点实际上是**相对启动点**的坐标 | `bt_navigator.global_frame: map`（`singlenav2_params.yaml:4`）；global_costmap `global_frame: map`（`:273`）；航点 CSV 存的是 map 坐标（`cod_bringup/wps/*.csv`，6 组） | ⇒ 先验图一旦换（场地/赛季/重建图），**所有 CSV 航点全部失效**（必须重打）。我们的 `waypoint_through_nav2` 若走同路线，也要接受同一约束 |
| R5 | **「机器人被搬走 / 被抱回起点 / 比赛脚本传送」后必须重启栈** | LIO 的 odom 是增量积分，没有绝对观测（无 AMCL、无 ICP、无重定位）；静态桥不会自动纠正 | ⇒ 我们的仿真里 `gz model -m ... -x ...` 传送、或 `nav_smoke_regression.py` 重置仿真后**必须重启 bringup**；实车「被撞大位移」后同样无法自恢复 |
| R6 | **换图 = 换定位基准；`map_server` 的 yaml 路径写死会静默加载旧图** | `singlenav2_params.yaml:360`（`/home/cod-sentry/dyx_ws/.../rmul2026.yaml`）；`localization_launch.py:56-58,76-79`（launch 的 `map` 参数会覆写它，但**只有在走 `localization_launch.py` 时**） | ⇒ 多点模式**根本不起 `map_server`** ⇒ 静态层无图可用，全局 costmap 的 `static_layer` 只能吃 `track_unknown_space: true` 的未知区（`singlenav2_params.yaml:280`；`always_send_full_costmap: False`）。**推断**：这解释了为什么他们要多点模式**放大全局滚动窗到 50×50**（`docs/cod_nav_nav_stack_inventory.md:167`） |
| R7 | **「位姿连续」被换来的是「图不连续」**：`static_layer` 吃 `/map`，而 `/map` 每秒可能被重排（`map_update_interval: 1.0`） | `singlenav2_params.yaml:282-285`（`static_layer` + `map_subscribe_transient_local: True`）；`mapper_params_online_async.yaml:31` | ⇒ **推断**：这是他们真正面对的现象——**机器人不抖，图会「跳」**。对高速 MPPI 而言，「图跳」的影响小于「TF 跳」（TF 跳会让 `getRobotPose()` 出现大位移，而他们恰好把这点彻底删掉了） |
| R8 | **`z=0.05` 的静态抬升是 3D 遗留，2D 下无害但也无用** | `singlenav_launch.py:76-78`、`multiplenav_launch.py:103-105` 都是 `--z 0.05` | ⇒ 若我们要用，**必须写 0**（我们 2D costmap 的 `robot_base_frame: base_link_fake` 与 odom 的 z 关系由 `lio_tf_adapter` 全程负责，见 `docs/tf_interface_contract.md` §三）。**推断**：他们保留 0.05 是因为 LIO 的 z 起点/雷达安装高度 |
| R9 | **`spin_speed == 0` 让 MPPI 的 `wz` 被丢弃**（与本题间接相关，但属同一套「减法」） | `fake_vel_transform.cpp` 里 `aft_tf_vel.angular.z = spin_speed_`（默认 0.0，全仓无一处非零）；`singlenav2_params.yaml:101,105` 仍声明 `wz_max 1.5 / wz_min -1.5` | ⇒ **守恒性检查**：他们「只控位置」的自洽性依赖 `goal_checker` 不管朝向——Gitee 成品单点用 **`PositionGoalChecker`**（`xy 0.2`，**无 yaw 键**，`singlenav2_params.yaml:79-83`），多点用 `SimpleGoalChecker` + **`yaw_goal_tolerance: 6.28`**（`multiplenav2_params.yaml:77-79`）。**我们若采用，必须同批改容差**，否则永远判不到「到点」 |

### B4. **对我们是否适用** —— 逐条结论

> 我们的现状基线（FACT，`file:line`）：
> - `map→odom` 的发布者：**AMCL / slam_toolbox(localization) / icp_registration** 三选一，或 **cartographer**（纯定位或全包形态）；
>   唯一允许静态桥的情形是 **`mode:=nav` + `localization:=''` + `lio∈{fastlio,pointlio,none}`**（即「拿 LIO 当绝对定位」的回退用法）。
>   证据：`src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py:444-468`（`icp_frame_bridge_condition` + `tf_bridge_node`/`tf_bridge_node2` 两条 `camera_init→map`、`body→odom` 静态桥）；
>   同文件 `:445` 的注释原文：「**amcl / slam_toolbox / icp_registration 三者都会自行发布 map→odom，绝不能再叠加静态桥（否则 map/odom 多父边）**」。
> - `docs/tf_interface_contract.md:17-19`：slam_toolbox / amcl / icp_registration 各发 `map→odom`；
>   `:44` 目标契约「`map ──(重定位：AMCL / slam_toolbox / ICP 之一发布)──► odom ──(里程计：LIO 发布)──► base_link`」。
> - `mode` 三态：`mapping` / `slam_nav` / `nav`（`bringup_sim.launch.py:124-131`）；`slam_nav` 的语义是「在线 SLAM 直接把 `/map` 与 `map→odom` 喂给 costmap；不加载磁盘地图、不起任何重定位模块」（`:130-131`）。

| 子问题 | 结论 | 证据/推理 |
|---|---|---|
| **(i) 我们哪些模式可以安全地用静态桥？** | **只有两个「本来就是静态」的角落**：<br>①`mode:=nav` + `localization:=''` + `lio≠cartographer`——**代码里已经这么做了**（`:444-468`），语义是「LIO 当绝对定位，静态桥补帧」；<br>②`localization:=icp`——`icp_registration` 的 `map→odom` 本质是「滑窗锚定的静态偏移」（`docs/tf_interface_contract.md:19`：「100 Hz 专用线程」），行为上**接近静态桥但会缓慢修正**；<br>**其余一律不行**：`amcl` / `slam_toolbox(localization)` / `cartographer(localization)` / `cartographer(全包)` 都自己发 `map→odom`。 | `bringup_sim.launch.py:444-468`；`docs/tf_interface_contract.md:17-19,44,126-128`；`docs/algorithm_axes.md:63`（A2 轴） |
| **(ii) 我们会失去什么？** | ①**先验图重定位能力**（`mode:=nav` 存在的意义）；②**长时漂移被修正的能力**（回环修正不再进入位姿）；③**「换图不用重打航点」的能力**（R4）；④**「被搬动后可恢复」的能力**（R5）。<br>⇒ 换句话说：静态桥不是「一种更简单的定位」，而是**把定位问题整体删掉**——只有在「场地固定、起点固定、单局时间短」时才是等价解。 | R1–R6；`docs/cod_nav_2026_integration_plan.md:175`（同结论） |
| **(iii) 能不能靠调 `slam_toolbox.transform_publish_period` 代替静态桥？** | **不能**，但**值得改**：<br>①`transform_publish_period` 只控制**发布频率**，不控制**是否有修正**（`slam_toolbox_common.cpp:255-259`：非 0 就按该周期把当前 `map_to_odom_` 发出去）；设成大值（如 1~5 s）只是把「连续小抖」变成「低频大跳」，**对 `getRobotPose()` 更糟**（nav2 的 TF 超时窗口是 0.1~0.2 s，见 `singlenav2_params.yaml:117` `transform_tolerance: 0.1`、我们 `nav2_params_sim_rpp.yaml` 的 `transform_tolerance`）。<br>②设成 0（COD 的做法）**只与静态桥配套**：等于宣布「SLAM 不参与定位」。<br>③**真正该调的是另外两个**：`map_update_interval`（栅格重排周期）与 `minimum_time_interval`（位姿图优化节流）。我们当前是 **`map_update_interval: 5.0`、`minimum_time_interval: 0.5`**（`src/localization/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml:29,32`），COD 成品是 **`1.0` / `0.3`**（`mapper_params_online_async.yaml:31,35`）。 | `slam_toolbox_common.cpp:74-83,251-275`；我们的参数见上；`docs/cod_nav_2026_integration_plan.md:174`（C20 同方向） |
| **(iv) 我们 `slam_nav` 已经踩到的「在线图滞后 / 缝」，静态桥会更好还是更坏？** | **会更坏（结论），而且我们踩的坑与静态桥无关**：<br>**FACT（我们的症状链）**：`docs/debug_fastlio_cartographer.md:867` ——「nav2 激活期 `base_link_fake→odom` 超时 → cartographer `Ignored subdivision`（零戳）→ 拒收扫描、**无 `map→odom`** → 全局路径只在已知区边界就停 → 贴墙时 `/scan` 丢墙 → **缝被当走廊 ⇒ 穿墙撞墙**」；`:870` 遗留①「缝：`range_min` 改小**没解决**」；`:869` 实测「近场（<1.5 m）障碍点 `z_map ≤ 0.196`、远场 `z_map ≈ 0.40`（墙高≈0.40 m）；`/scan` 的 `inf` 占 22~30%（主因 `range_max 10 m` < 场地 15×28 m）」；`:872`「`robot_radius 0.40`」。<br>**为什么静态桥更坏**：静态桥会让 `map` 与 `odom` 永久同源 ⇒ **在线 SLAM 的闭环修正永远进不了位姿** ⇒ 「图漂移」不再被位姿吸收，而是**直接叠在规划用的那张栅格上**（R1/R7）。我们的 `slam_nav` 已经是「图不可信 + 感知有缝」，再叠加「图不会被修正」，只会让 planner 更无从判断。<br>**真正的对策方向**（证据支持）：①**把 `map_update_interval` 从 5.0 降下来**（对齐 COD 的 1.0）——我们的「帧→栅格」延迟是 5 s 量级，而 `transform_publish_period` 是 0.02 s（`mapper_params_online_async_sim.yaml:28-29`）⇒ **50 Hz 的位姿 + 0.2 Hz 的图**，这个错配本身就是「滞后」的一种解释；②**把感知缝隙当成独立问题修**（`range_max 10→20`、裁剪盒、局部 STVL：`docs/cod_nav_2026_integration_plan.md` P0 的三条）；③**不要在 `slam_nav` 里引入静态桥**。 | 见左栏逐条 `file:line` |

**B 部分一句话**：COD 用静态 `map→odom` 的**真原因是 (a)+(c)+(d)**——「无先验图 ⇒ map≡odom 零成本」「环境简单/面积小 ⇒ 定位容忍度大 ⇒ 纯 LIO」「KISS 原则」；
「避免回环跳变扯动 costmap/MPPI/自转底盘」这半边**被证据反证**（他们根本不让 slam_toolbox 发 TF；且他们从不自转）；
代价是**漂移变图畸变 + 起点/朝向被绑死 + 换图即失效 + 被搬动不可恢复**。
**对我们**：不应作为默认；我们**已经**在 `mode:=nav + localization:=''` 这个回退角落里有了同样能力（`bringup_sim.launch.py:444-468`），
额外能做的只有「调 `slam_toolbox` 的 `map_update_interval`」这一件事。

---

## C. 宏观大对比：能力地图

> 表格口径：「判定」= **我们强 / 平 / 他们强**（相对我们 bench 现有资产，不看谁赚了钱）；
> 「一句话原因」必须能落到参数或文件。COD 侧证据以 **Gitee 成品**为准，GitHub 季前值在括号里注出。

| 维度 | COD（RMUL2026 成品） | 我们 | 判定 | 一句话原因 |
|---|---|---|---|---|
| **1. 传感器** | Livox MID-360 + `/livox/imu`（`config/mid360.yaml:3-6`）+ **RealSense D435i 真接进 STVL**（`singlenav2_params.yaml:218,241` `observation_sources: livox_source realsense_source`） | Gazebo 旋转仿真 Mid360（`/livox/lidar` CustomMsg + `/livox/lidar/pointcloud`）+ linefit 地面分割 + 无深度相机（`docs/debug_fastlio_cartographer.md` §10.0） | **他们强** | 他们多一路真实 3D 相机观测源（`model_type: 0`、`obstacle_range 3.0`、`min_obstacle_height 0.18`，`:248-262`）；⚠️ 但**我们仿真链路的雷达是旋转式的、更接近真车策略**，且我们有独立地面分割（见第 3 行） |
| **2. LIO** | **`small_point_lio`**（逐点 ESKF + iVox，`map_resolution 0.5`，无回环，**无 de-skew**，`twist≡0`） | `fastlio`（帧级 iterated-EKF + ikd-Tree）+ 可选 `pointlio`（同族）+ `lio_tf_adapter` 统一到 `odom→base_link`/`/odom`（`docs/tf_interface_contract.md` T2/T3） | **我们强** | 我们有**统一契约层**（`:74-75`）与两种可 A/B 的 LIO；他们是「一个 LIO 直接发硬编码帧名」（`small_point_lio_node.cpp:61-62`） |
| **3. 地面分割** | **没有**（2026 整条砍掉，`patchwork`/`linefit` grep 0 命中）；用 p2l **硬高度带**取代（`min_height 0.10`、`max_height 1.00`，`multiplenav_launch.py:72-73`） | `linefit_ground_segmentation` → `/segmentation/obstacle` + `/segmentation/ground`（`docs/debug_fastlio_cartographer.md` §10.0） | **我们强** | 硬高度带对坡道/矮墙不鲁棒；linefit 是**可 A/B、可诊断**（z-profile 工具）的资产 |
| **4. 2D 建图** | `slam_toolbox` async **`mode: lifelong`**，`map_update_interval 1.0`、`minimum_time_interval 0.3`、`resolution 0.05`、`max_laser_range 10.0`（`mapper_params_online_async.yaml:19,31-35`）；另有 `auto_save_map.launch.py` 每 30 s 存一次图（**路径写死 `/home/cod-sentry/...`**，`:13-15`） | `cartographer`（**打了核心补丁 `min_probability_to_clear: 0.80`**，`docs/debug_fastlio_cartographer.md` §10.1）+ `slam_toolbox`（`map_update_interval 5.0`、`minimum_time_interval 0.5`，`mapper_params_online_async_sim.yaml:29,32`） | **我们强（图质量）/ 他们强（更新率）** | 我们的图是**唯一被用户验收过的资产**（§10.1「这个地图是可以的」）；但**他们的栅格更新比我们快 5 倍**（1.0 vs 5.0 s）⇒ 这一条要单独抄（§D 槽位） |
| **5. 定位/重定位** | **零**（无 amcl、无 icp、无 carto、无先验图重定位）；`localization_launch.py:44` 只留 `map_server`；静态 `map→odom` | **五种**：`amcl` / `slam_toolbox(localization)` / `icp`(3D PCD) / `cartographer`(pbstream 纯定位) / `cartographer`(全包形态兼任里程计)；`docs/tf_interface_contract.md:17-19,126-128`；`bringup_sim.launch.py:136` | **我们强** | 这是**代差**：他们把一个失败类整体删除，我们把它做成可 A/B 的四个槽位 |
| **6. TF 契约** | `odom→base_link` 硬编码在 LIO 节点内（`small_point_lio_node.cpp:61-62`）；`map→odom` 静态桥（`singlenav_launch.py:67-89`）；`base_link→base_link_fake` 由 `fake_vel_transform` 发；**依赖外部提供 `base_link→livox_frame`**（LIO 查不到就丢帧，`:64-69`） | **显式契约文档 + 每边恰好一个发布者**：`docs/tf_interface_contract.md:44`（目标帧树）、`:47-53`（角色/禁止事项）、T1–T5 已实施（`:73-77`）；`lio_tf_adapter` 统一 `/odom`；Gazebo 关 `publish_odom_tf` | **我们强** | 他们的帧名是**编译期常量**、TF 来源**不在仓库内**（`docs/cod_nav_2026_deep_dive.md:495`）；我们的契约被文档化并**踩过坑后修好**（`docs/tf_interface_contract.md` §二 P1–P6） |
| **7. 全局规划** | **`SmacPlanner2D`** + `cost_travel_multiplier 4.0` + `tolerance 0.5` + `allow_unknown true` + `smooth_path: true` + 内建 `smoother{w_smooth 0.4, w_data 0.1, do_refinement true}`（`singlenav2_params.yaml:375-405`） | **`NavfnPlanner`** + `allow_unknown: true`（`nav2_params_sim_rpp.yaml:357-362`）；`planner` 槽**目前只有 navfn** | **他们强** | `cost_travel_multiplier` 是把路径推向代价谷底的**直接对策**（`singlenav2_params.yaml:384` 注释原文），我们正是「贴缝走」的受害者 |
| **8. 局部控制** | **`goal_approach_controller` 包 MPPI(Omni)**，50 Hz、`time_steps 80`、`model_dt 0.02`、`batch_size 1500`、`vx/vy ±2.5`、`temperature 0.25`、`gamma 0.015`；近目标限速 + **2 m 内绕开 MPPI 直驱**（GAC:96-126）；`velocity_smoother` 30 Hz / OPEN_LOOP（`singlenav2_params.yaml:471-482`） | `rpp`（默认，`desired_linear_vel: 0.5`）/ `dwb` / `teb` 三槽；**MPPI 包已装但未接**（`docs/cod_nav_2026_params_review.md` §0.4） | **他们强** | 差的是**控制器族**（采样式 omni + 近目标直驱）；⚠️ 但他们的 `vx_max 2.5` 是**实车收敛值**，可直接当我们 A/B 的首版（`docs/cod_nav_nav_stack_inventory.md:168`） |
| **9. 代价图与障碍** | **两张图都挂 STVL**（`voxel_decay 0.5/1.0`、`decay_model 0`、`voxel_size 0.05`、源级 `obstacle/raytrace 8/9 m`、`model_type 1`、`vertical_fov_angle 2.00`、`filter: voxel`），并**用裁剪盒去掉车体点云**（`multiplenav_launch.py:35-49`：x±0.2 / y−0.2~0.4 / z−0.1~0.2 / `negative True`），**全局也 `rolling_window: true`**（25×25@0.04 单点 / 50×50@0.04 多点） | global = `static + obstacle + stvl + inflation`（STVL 在 global，`nav2_params_sim_rpp.yaml:258,284-288`，`voxel_decay 0.5`）；local = `obstacle + obstacle_cloud + inflation`（**无时间维**，`:182`）；`robot_radius 0.40`（`:178,252`）；**无裁剪盒**（`/segmentation/obstacle` 直接进 local cloud 层，`:213-224`） | **他们强（近距/时间维）/ 我们强（诚实几何）** | 他们的「裁剪盒 + 双图 STVL + 源级 8/9 m」直接打我们「近距盲区 / 幽灵障碍」两个痛点；我们的 `robot_radius 0.40` 是**按 0.2 车体 + 0.2 近距盲区**算出来的诚实值（`:173`），他们单点只有 **0.1 m 方框**（`singlenav2_params.yaml:190-191`） |
| **10. 任务层** | BT（自研 XML：`RateController 3 Hz` 重规划、`RecoveryNode` 分层重试 10 次、`RemovePassedGoals radius 0.7`）+ **6 组语义航点 CSV**（`cod_bringup/wps/`）+ **仓内 `waypoint_editor`**（RViz 打点）+ 决策在**另一个仓库**（`cod_-rm2026_-behavior-tree`，9 分支） | stock BT + 自研分段工具（`tools/scripts/nav/segment_goal_navigator.py`）；航点图/前沿探索**未做**（`docs/debug_fastlio_cartographer.md` §10.5 遗留⑥） | **他们强（成品度）** | 他们有**从打点到执行到决策的完整闭环**（CSV → `waypoint_through_nav2` → BT → 裁判串口），我们只有 stock BT + 工具脚本 |
| **11. 仿真** | **GitHub `rmul2026` 分支没有任何仿真**（无 worlds/无 gazebo launch）；作者另外维护 **`COD_loopback_sim`**（把 Nav2 `loopback_sim` 回移 Humble + pixi）用于「不启 Gazebo 也能跑决策/控制」 | **一等公民**：Gazebo Classic 场地（RMUC/RMUL/RMUL2026）+ 旋转 Mid360 插件 + 多算法 A/B（`docs/architecture.md` §一） | **我们强** | 我们有**物理仿真 + 真值 odom + 场景可切换**；他们的 loopback 是**刻意去掉物理**的（README 自述「physics-engine artifacts 是干扰」，`docs/cod_nav_nav_stack_inventory.md:59`） |
| **12. 测试与工程** | CI **仅编译**（`skip-tests: true`，且路径拼写 `ws_liovx` 疑似一直失败）；提交了 `.idea/`、`launch/__pycache__/*.pyc`；`auto_save_map.launch.py:13-15` 与 `singlenav2_params.yaml:360` 写死 `/home/cod-sentry/...`；`map_server.yaml_filename` 是死值 | 有回归闸门（`tools/scripts/regress/nav_smoke_regression.py`）、链路体检（`tools/scripts/diag/watch_startup_chain.py`）、`docs/` 体系、无硬编码 | **我们强** | 工程卫生是**我们唯一的「碾压项」**（`docs/cod_nav_comparison.md:36`） |
| **13. 决策与下位机** | **完整**：决策库 `cod_-rm2026_-behavior-tree`（BT 4.8 + Groot2，10 棵 XML，裁判系统 0x0001…0x020D 消息表）；下位机 `rmcod2026_-sentry`（舵轮解算 + RLS 功率拟合 + 变档小陀螺 + 超电 + 热量控制）；串口帧 `0xA5 + 3×float + checksum = 15 B`（`ros2_simple_serial/src/cod_serial.cpp`）；**唯一导航侧契约**是 `/aft_cmd_vel` | **无**：仿真 bench，无下位机、无裁判系统、无自瞄；契约只在文档层（`docs/sim_real_contract.md`） | **他们强** | 这是**体系差距**，但也是**我们不该追**的部分（我们没有车）；我们该抄的只有**「上位机速度语义必须与轮速换算对齐」这一条契约**（作者原话 NOTE，见 §B1.3） |

**C 部分一句话**：我们强在 **契约（TF/参数/文档）/ 重定位资产 / 地面分割 / 二维图质量 / 仿真 / 工程卫生**；
他们强在 **控制器与规划器族（MPPI+wrapper、Smac2D）/ 近距与时间维障碍表示（裁剪盒+双图 STVL）/ 任务层成品度 / 决策-下位机体系 / slam_toolbox 更新率**。

---

## D. COD 融入我们架构的宏观视图

### D1. 目标架构图（文字版；默认链 = 一个字不动，新增 = 候选分支）

```
[传感器]  Gazebo 旋转 Mid360 ──> /livox/lidar(CustomMsg) ─┬─> /livox/lidar/pointcloud(PointCloud2)
                                                          │
                        ┌─────────────────────────────────┴──────────────┐
                        │  ★新增候选：rm_cloud_crop（裁剪盒）             │
                        │     输出 /livox/lidar_filtered                  │
                        └─────────────────────────────────┬──────────────┘
                                                          │（只喂 3D 代价图层，/scan 链不动）
[E 感知]  linefit_ground_segmentation ─> /segmentation/obstacle ─┬─> local: obstacle_cloud_layer（候选）
                                                                 └─> global: stvl_layer（现状）
          pointcloud_to_laserscan ─> /scan ─┬─> local: obstacle_layer（现状默认）
                                            ├─> global: obstacle_layer（候选 global_obstacle:=scan）
                                            └─> mapper / localization

[A0 LIO]  lio:=fastlio(默认) | pointlio | none | cartographer
          ★新增候选：lio+=smallpointlio？—— **不新增**（见 §A4：tag/timestamp 硬门槛 + twist≡0）
          └─> /odom ──> lio_tf_adapter ──> TF odom→base_link
[A1 建图] mapper:=cartographer(默认) | slam_toolbox
          ★新增候选：mapper+=slam_toolbox_lifelong（参数集：map_update_interval 1.0 / minimum_time_interval 0.3
                                                   / minimum_travel_heading 0.1 / mode: lifelong）
                                                    ⚠️ 它必须自己发 map→odom（transform_publish_period≠0）
[A2 定位] localization:=amcl | slam_toolbox | icp | cartographer（仅 mode:=nav）
          ★不新增：静态 map→odom 桥（现状只在 mode:=nav + localization:='' 这条回退里存在，保持不变）
          └─> TF map→odom（**每条边恰好一个发布者**：docs/tf_interface_contract.md:44）

[B 规划] planner:=navfn(默认) ──> ★新增候选：planner+=smac2d（tolerance 0.5 / cost_travel_multiplier 4.0 /
                                                  allow_unknown true / smooth_path true + 内建 smoother）
          smoother := simple(默认) ──> ★新增候选：smoother+=savgol（**必须在我们自研 BT 里显式加 <SmoothPath>**）

[C 控制] nav:=rpp(默认) | dwb | teb ──> ★新增候选：nav+=mppi（Omni；model_dt 必须 = 1/controller_frequency）
                                                    └─ 可选第 5 个候选：mppi_approach（= 上游 goal_approach_controller 同思路：
                                                       近目标限速 + direct_approach 直驱，**需自研或 vendor 并 colcon build**）

[F 底盘]  /cmd_vel ──> velocity_smoother ──> fake_vel_transform ──> /cmd_vel_chassis ──> Gazebo 麦轮
          spin_speed==0 直通（默认，我们已修） | spin_speed!=0 小陀螺（覆盖 angular.z，**必须同批放宽 yaw_goal_tolerance**）
```

### D2. 槽位映射表（我们每个槽位新增哪些取值）

> 「成本」= 改动量级；「判定」= ✅直接融 / △需改造 / ❌不融（与 `docs/cod_nav_2026_integration_plan.md:162-178` 对齐，本文补上**为什么**与**依赖顺序**）。

| 槽位 | 新增取值 | 来源（file:line / URL） | 判定 | 依赖/前提 |
|---|---|---|---|---|
| `lio` | **不新增**（`smallpointlio` 不落地） | `livox_pointcloud2.h:25-37`；`small_point_lio_node.cpp:90-96` | ❌ | 已有 `pointlio`（同族）；见 §A4 |
| `planner` | `planner+=smac2d` | `singlenav2_params.yaml:375-405` | ✅ | 无需改包（`nav2_smac_planner` 已装）；**不要抄** Hybrid 专用键（`motion_model_for_search`/`angle_quantization_bins`/`minimum_turning_radius`/`analytic_expansion_*`/`rotation_penalty`/`lookup_table_size`/`allow_reverse_expansion`，对 `SmacPlanner2D` 惰性） |
| `smoother` | `smoother+=savgol` | `singlenav2_params.yaml:407-420` | △ | **必须在我们自研 BT 里显式加 `<SmoothPath>`**，否则是死代码（他们的 BT 就没有它：`docs/cod_nav_2026_deep_dive.md:236`） |
| `nav` | `nav+=mppi`（Omni）；可选 `nav+=mppi_approach` | `singlenav2_params.yaml:84-176`；GAC `:96-126` | ✅（mppi）/ △（wrapper） | **硬约束三条**（原文见 `docs/debug_fastlio_cartographer.md:888`）：①`model_dt == 1/controller_frequency`（否则 `optimizer` 抛异常、controller_server 起不来）；②`CostCritic.consider_footprint: true` 在**只有 `robot_radius` 的圆形 costmap** 上会抛异常 ⇒ 先加 footprint 或设 false（我们现状：`nav2_params_sim_rpp.yaml:178,252` 只有 `robot_radius: 0.40`、无 footprint）；③`min_y_velocity_threshold` 必须降到 0.001（我们现状 `nav2_params_sim_rpp.yaml:107` 是 `0.5`，会把 Omni 的侧向反馈抹零）。另见 `docs/cod_nav_2026_params_review.md:94,704` |
| `local_obstacle` | `local_obstacle+=stvl`（带 `voxel_decay`） | `singlenav2_params.yaml:200-262` | ✅ | 包已装；**先只加时间维这一条**，其余层不动 |
| `global_obstacle` | （我们已有 `stvl`）补 `model_type: 1` + `vertical_fov_angle: 2.00` + `filter: voxel` | `singlenav2_params.yaml:237-240` | ✅ | 我们 global 的 stvl 已开，但**这三个键是最容易漏的**（`model_type: 0` 会按 0.7 rad 垂直 FOV 裁切） |
| `crop_box`（新节点槽） | `crop_box:=on|off` | `multiplenav_launch.py:35-49`；`cpp_lidar_filter/src/filter_node.cpp` | ✅ | **必须在 `livox_frame` 下重标定**（`CropBox` 不做 TF，而 marker 画在 `base_link`）；`leaf_size` 是死参（`VoxelGrid` 整段注释） |
| `mapper` | `mapper+=slam_toolbox_lifelong` | `mapper_params_online_async.yaml:19,31-35` | △ | 必须**让它发 `map→odom`**（把 `transform_publish_period` 从 0.0 改成非 0）；必须**与 `localization:=*` 互斥**（否则双发布者） |
| `localization` | **不新增静态桥**；`mode:=nav + localization:=''` 的回退保持现状 | `bringup_sim.launch.py:444-468` | — | 已存在 |
| 任务层 | `waypoint_through_nav2`（CSV → `navigate_through_poses`） | `waypoint_editor/USAGE_WITH_NAV2.zh.md`；`cod_bringup/wps/*.csv` | △ | 需 vendor/自研桥节点；`waypoint_editor` 与 Nav2 **都会启 `map_server`**（冲突）⇒ 二选一 |
| 哨兵语义 | （我们已有 `spin_speed`）语义分档成文 | `fake_vel_transform.cpp`；`singlenav2_params.yaml:101-105` | ✅（认知项） | `spin_speed!=0` 时 MPPI 的 `wz` 被丢弃 ⇒ **必须同批放宽 `yaw_goal_tolerance`**（他们多点用 6.28） |
| 决策/下位机 | **不融** | — | ❌ | 体系不同；只借「cmd_vel→轮速一致性检查」这一条契约（BBS 1882897 的 NOTE） |

### D3. 依赖顺序（谁必须先做、为什么）

```
P0-1  裁剪盒（crop_box）           ── 独立节点、独立话题、可随时停用 ⇒ 最先做，直接打「近距/自身点云」
  │                                  （但它需要先有一份稳定的 base_link→livox_frame TF：我们已有，见 tf_interface_contract §一）
  ├─> P0-2  global stvl 三键修正（model_type 1 / vertical_fov 2.00 / filter voxel）
  │         ── 与 P0-1 同批：新点云进来后，视锥必须正确，否则「看到却不标记」
  └─> P0-3  local_obstacle+=stvl（先只加 decay）
            ── 必须在 P0-1/P0-2 之后：否则「近距点云 + 错误视锥 + 无时间维」会互相掩盖现象

P1     planner+=smac2d ── 与感知无耦合，可并行；但要先有 P0 的图才看得出「路径是否居中」

P2     nav+=mppi ── 必须在「costmap 已有可用梯度」之后（inflation 梯度 + 时间维），
        否则会复现他们「整个膨胀区都当碰撞 ⇒ MPPI 没梯度」的老问题；
        且必须与 `spin_speed` 语义一起定（wz 归属），不能两个都开
  └─> P2b  mppi_approach（wrapper）── 最后做：它只在 MPPI 已经能跑之后才有意义

P3     mapper+=slam_toolbox_lifelong ── 最后做：
        ①它是唯一会引入「第二个 map→odom 候选」的改动 ⇒ 必须在 launch 里加互斥条件；
        ②它的价值（更新率）只有在 P0 把图/感知修好之后才可判定；
        ③⚠️ 但它里面有一条**可以提前做且零风险**：把我们现网 slam_toolbox 的
          `map_update_interval 5.0 → 1.0`（对齐 COD）——这条只有一行，且能直接检验「缝」是否与栅格更新率有关
```

### D4. 冲突清单（必须显式互斥/同批）

| # | 冲突 | 事实/机制 | 处置（launch 或参数层） |
|---|---|---|---|
| C1 | **`map→odom` 双发布者** | `localization:=amcl/slam_toolbox/icp/cartographer` 各自发布（`docs/tf_interface_contract.md:17-19`）；`mapper:=slam_toolbox_lifelong` 若把 `transform_publish_period` 设为非 0 也会发布（`slam_toolbox_common.cpp:255-259`）；`mode:=nav+localization:=''` 时还有 T1 静态桥（`bringup_sim.launch.py:444-468`） | **必须在 launch 的条件里做四路互斥**（现有代码已对 amcl/slam_toolbox/icp 做了：`:303-406`；新增 `mapper` 取值时要把它算进去） |
| C2 | **`wz` 归属** | `fake_vel_transform` 在 `spin_speed!=0` 时**无条件覆盖** `aft_tf_vel.angular.z`（`fake_vel_transform.cpp`；我们已改成 `0 直通`，**语义与他们不同**）；MPPI 会输出 `wz`（`singlenav2_params.yaml:101,105`） | 二者**互斥**：要么 `spin_speed=0` 让 MPPI 控朝向，要么自转 + `yaw_goal_tolerance` 放宽（他们多点 6.28）/ 单点用 `PositionGoalChecker`（`singlenav2_params.yaml:79-83`） |
| C3 | **无 footprint 的 MPPI** | 我们只有 `robot_radius 0.40`、**没有 footprint 多边形**（`nav2_params_sim_rpp.yaml:178,252`）；`CostCritic.consider_footprint: true` 在圆形 costmap 上会抛异常（`docs/debug_fastlio_cartographer.md` §10.4-6） | 接 MPPI 前：**加一个 footprint 多边形**（按车体几何）或把 `consider_footprint` 设 false |
| C4 | **`spin_speed` 语义** | 我们：`spin_speed==0` **直通**（保留 `wz`）；他们：`spin_speed==0` ⇒ **`wz≡0`**（无条件覆盖）。同一个参数名，两种语义 | 移植任何「他们的小陀螺参数」时必须**按我们的语义重算**，不能把 `spin_speed 0.0` 当成「自转关、朝向交给 Nav2」（在他们那里恰好相反） |
| C5 | **`model_dt` × `controller_frequency`** | MPPI 要求 `model_dt == 1/controller_frequency`，否则 `controller_server` 直接 configure 失败（`docs/debug_fastlio_cartographer.md` §10.4-6） | 抄他们的 `model_dt 0.02`（单点）**就得同时把 `controller_frequency` 设 50**；我们若用 20/30 Hz，必须改成 `0.05/0.0333` |
| C6 | **`smoother_server` 是死代码** | 他们的 BT 里**没有 `<SmoothPath>`**（`docs/cod_nav_2026_deep_dive.md:236`） | 我们若上 `smoother+=savgol`，**必须同步改自研 BT**，否则白起一个节点 |
| C7 | **`waypoint_editor` × `map_server`** | 两者都会启 `map_server`（`waypoint_editor/USAGE_WITH_NAV2.zh.md`，`docs/cod_nav_nav_stack_inventory.md:60`） | 二选一，或用不同 namespace |
| C8 | **`local_obstacle+=stvl` × 我们的 `scan` 默认** | 我们 local 默认 `scan`（`bringup_sim.launch.py:169-172`）；STVL 吃 3D 点云/`/livox/lidar_filtered` | 加 STVL 是**加一层**不是换层；要同时决定「谁来 clearing」（STVL 的 raytrace 与 obstacle_layer 的 raytrace 会互相打架） |

### D5. 「最终形态」描述

1. **默认路径一个字不动**：`lio:=fastlio` + `mapper:=cartographer` + `localization:=amcl`（`mode:=nav`）+ `planner:=navfn` + `nav:=rpp` + `linefit` + `local_obstacle:=scan`。
   证据：`bringup_sim.launch.py:136,143,176`；`docs/cod_nav_2026_integration_plan.md:19`（「只增槽，不换默认」）。
2. **新增候选全部可 A/B、可回退**：每个新取值都只对应「一个新 params/一个新节点 + 一个 launch 参数值」，
   回退 = 把该参数改回默认（`nav:=rpp` / `planner:=navfn` / 停裁剪盒节点 / 删 STVL 层）。
3. **契约不变量仍然成立**：`map→odom` 每条边**恰好一个发布者**；`odom→base_link` 只由 `lio_tf_adapter` 或 `lio:=cartographer` 发布；
   `base_link→base_link_fake` 只由 `fake_vel_transform` 发布（`docs/tf_interface_contract.md:44-53,88`）。
4. **每一个候选都有独立判据**：P0 用 `cloud_z_profile.py` / `costmap_marking_check.py` / `nav_smoke_regression.py`；
   P1/P2 用「路径居中程度 / 震荡幅度 / 到达时间 / CPU」写回 `docs/algorithm_matrix.md §四`（判据与验收方式沿用 `docs/cod_nav_2026_integration_plan.md:44-48` 与 `:129-131`）。

---

## E. 结论

### E1. 可融入比例（按「组件个数」与「价值」两个口径）

| 分类 | 组件 | 个数 |
|---|---|---|
| **直接融（✅）** | ①裁剪盒 ②global STVL 三键（`model_type 1`/`vertical_fov 2.00`/`filter voxel`）③`local_obstacle+=stvl` ④STVL 源级 `obstacle/raytrace 8/9 m` ⑤`planner+=smac2d`（9 个有效键 + 内建 smoother）⑥`nav+=mppi`（骨架 + critic 键名 + `temperature/gamma/reset_period`）⑦inflation 梯度（`cost_scaling_factor 5.0`，半径按 `robot_radius 0.40` 重算）⑧`p2l.range_max 10→20` ⑨`velocity_smoother` 键名对齐 ⑩slam_toolbox 更新率（`map_update_interval 1.0`） | **10** |
| **需改造（△）** | ①`smoother+=savgol`（要先改 BT）②`nav+=mppi_approach`（要 vendor/自研 wrapper，且它把 `angular.z` 置 0）③`mapper+=slam_toolbox_lifelong`（要加互斥 + 要让它发 TF）④`goal_checker`（单点 `PositionGoalChecker` / 多点 `yaw 6.28`，要同批改）⑤`waypoint_through_nav2` 任务层 ⑥高度带（**禁止照抄**，必须按我们 z 剖面重算）⑦裁剪盒数值（必须按 `livox_frame` 重标） | **7** |
| **不融（❌）** | ①`small_point_lio`（tag/timestamp 硬门槛 + `twist≡0` + `map_resolution 0.5` + `ROOT_DIR` 写死）②静态 `map→odom` 作为默认（我们四种重定位是资产）③去掉 linefit 改硬高度带 ④5 m/s+ 速度包线 ⑤50×50@0.04 `always_send_full_costmap` ⑥`auto_save_map.launch.py`（写死路径）⑦RealSense 链路（我们无相机）⑧决策/下位机体系 | **8** |

⇒ **「直接融 10 / 需改造 7 / 不融 8」**，且**没有任何一条要求改我们的架构**（都是「加一个槽位取值」或「改一个参数」）。
这印证了 `docs/cod_nav_2026_integration_plan.md:158-188` 的结论：**能融，但正确形状不是搬架构，而是把他们的组件变成我们槽位的新取值**。

### E2. 收益最大的一条

**`planner+=smac2d` 的 `cost_travel_multiplier: 4.0`（+ 内建 `smooth_path`）。**
理由：它**直接对准我们唯一被用户验收过、但一直没解决的规划症状**——「贴缝走 / 把缝当走廊 ⇒ 穿墙撞墙」
（`docs/debug_fastlio_cartographer.md:867,870`）。它是一条**已有包的参数级改动**（`nav2_smac_planner` 已装）、
不引入新节点、不改变契约，且 `cost_travel_multiplier` 的语义（`singlenav2_params.yaml:384` 注释「较大的值将更精确地放置在通道的中心」）
正是「把路径推离高代价区」的**直接对策**。

> 次优：**裁剪盒 + 双图 STVL**（打「近距盲区 / 幽灵障碍」），但它引入新节点与标定成本；
> `nav+=mppi` 收益高但**必须先解决 C2（wz 归属）与 C3（无 footprint）**，否则起不来/不收敛。

### E3. 风险最大的一条

**「照抄他们的 yaml」这一动作本身**（不是某一条参数）。
两条硬证据：①他们的 yaml 大量使用 **nav2 Jazzy/main 才有的键**，在本机 1.1.20 上**静默失效或直接抛异常**
（`docs/cod_nav_2026_params_review.md` §0.1、§0.4：`vy_min/wz_min/ax_*/ay_*/az_*`、`publish_critics_stats`、
`path_length_tolerance`、`CostCritic.trajectory_point_step`、planner 段 16 个 Hybrid 专用键等）；
②更危险的是**「看起来生效其实生效了、但语义不同」的参数**——`spin_speed` 就是活例（C4）：
在他们是「`wz≡0`」，在我们是「直通」，同名反义。
⇒ **风险控制办法**：任何新槽位落地前先 `ros2 param list <node>` 逐键核对存在性（`docs/debug_fastlio_cartographer.md` §10.4-4），
并把「语义是否相同」单独写一行。

### E4. 如果只做三件事，做哪三件

| 优先级 | 做什么 | 为什么是它 | 验收/回退 |
|---|---|---|---|
| **第一件** | **`planner:=navfn` → 新增 `planner:=smac2d`**，抄 `tolerance 0.5`、`allow_unknown true`、`cost_travel_multiplier 4.0`、`max_planning_time 4.5`、`smooth_path true` + 内建 `smoother{max_iterations 10000, w_smooth 0.4, w_data 0.1, do_refinement true}`；**不抄** Hybrid 专用键 | 收益最大（E2）、风险最低、零新依赖、直击「贴缝走」 | 判据：同一目标的路径是否居中于代价谷底、`velocity_smoother` 饱和次数下降、规划耗时 < `max_planning_time`；回退 `planner:=navfn`（`docs/cod_nav_2026_integration_plan.md:51-54`） |
| **第二件** | **裁剪盒 + 双图 STVL 三件套**：新增 `/livox/lidar_filtered`（`CropBox`，`negative`，数值**在 `livox_frame` 下重标**）→ 只喂 **global STVL**（补 `model_type: 1`、`vertical_fov_angle: 2.00`、`filter: "voxel"`、源级 `obstacle_range 8.0 / raytrace_range 9.0`）与 **local 新增 STVL 层**（`voxel_decay 0.5`、`decay_model 0`、`voxel_size 0.05`） | 打我们两个最疼的感知痛点（近距盲区、幽灵障碍），且是他们**真正在用的**组合（`multiplenav_launch.py:35-49` + `singlenav2_params.yaml:200-262`） | 判据：`cloud_z_profile.py` 的 0–0.5 m 桶不再空、贴墙时 `inf` 探针有回波、`costmap_marking_check.py` 近距标记不下降；回退 = 停裁剪盒节点 + 删 STVL 层（`docs/cod_nav_2026_integration_plan.md:44-48`） |
| **第三件** | **`nav+=mppi`（Omni）+ 先把三个硬约束解掉**：①加 footprint 多边形（或 `CostCritic.consider_footprint: false`）②`model_dt ≡ 1/controller_frequency` ③`min_y_velocity_threshold 0.5 → 0.001`；同时**明确 `wz` 归属**（`spin_speed:=0` ⇒ MPPI 控朝向；要自转 ⇒ 同批放宽 `yaw_goal_tolerance` / 换 `PositionGoalChecker`） | 收益第二（控制器族换代 + 现成的战训参数），但**只有把三个硬约束解决掉才起得来**，所以必须作为一个整体做 | 判据：贴墙最小距离、窄缝通过率、震荡幅度、到达时间、CPU（三方对照写回 `docs/algorithm_matrix.md §四`）；回退 `nav:=rpp` |

> **明确不做（前三件之外）**：`small_point_lio`、静态 `map→odom` 作为默认、`slam_toolbox_lifelong`（除非先有互斥条件）、
> `smoother:=savgol`（除非先改 BT）、RealSense 链路、5 m/s+ 速度包线、决策/下位机体系。

### E5. 本次**未能确定**的东西（明确列出）

1. **`singlenav_launch.py:82-83` 的 `yaw = -0.5` 的确切物理含义**：仓库内无注释、无文档；Gitee 只有 1 个提交（本地 clone 是 `--depth` 浅克隆，`git log -S'--yaw'` 也只得 HEAD），
   **无法从历史确定它何时/为何被引入**。作者帖子只说 `0,0,0`。⇒ **无法确定**。
2. **他们真机的 `/Odometry.twist` 是否有值**：公开快照 `small_point_lio_node.cpp:90-96` 是注释掉的；但 BBS 1882897 作者说「这是我修改过的 small point lio，**可以输出速度**」。
   ⇒ 两者矛盾，**仓库外版本无法验证**。这可能影响他们 `velocity_smoother` 的 `feedback: "OPEN_LOOP"`（`singlenav2_params.yaml:475`）是否真的不读 odom。
3. **`base_link → livox_frame` 的实际外参来源**：Gitee 成品仓内**只有 `small_point_lio/launch/small_point_lio.launch.py:24-45` 这一处静态 TF**，
   而 cod_bringup 的 launch 都**直接起 `small_point_lio_node`、不 include 它**；仓库内也没有 `robot_state_publisher` 的启动。⇒ **他们实际从哪加载 URDF/TF 无法确定**
   （GitHub 季前同样如此，见 `docs/cod_nav_2026_deep_dive.md:495`）。
4. **`singlenav_launch.py` 里 RealSense 的 TF/帧是否真的被 STVL 消费**：`realsense_source` 的 `topic: /camera/camera/depth/color/points`（`singlenav2_params.yaml:244`）由驱动 `publish_tf: true`（`singlenav_launch.py:116`）供帧，
   但**我们无法验证其外参与 `base_link_fake` 的关系**，也未验证 `model_type: 0` 的视锥是否与 D435i 实际安装一致。
5. **静态 `map→odom` 与先验图 `origin` 是否真的自洽**：`maps/rmul2026.yaml:4` 的 `origin` yaw 为 0，而静态桥 yaw 为 -0.5；
   二者是否对应「同一物理起点」**只有真机标定能证明**（R3）。
6. **R7「`/map` 栅格跳变 vs `map→odom` 位姿跳变哪个更伤」的定量比较**：本次只做了机制分析（`slam_toolbox_common.cpp:255-259` + `static_layer` 订阅 `/map`），
   **未做实验**（本文禁止运行任何代码）。⇒ 对我们是否有益**未能确定**。
7. **我们 `slam_nav` 里 `map→odom` 的实际发布者**：`docs/debug_fastlio_cartographer.md:867` 记录的症状里明确写「**无 `map→odom`**」，
   而该模式用 `mapper` 槽 + `lio` 槽（`bringup_sim.launch.py:486-510`），**本应**由 slam_toolbox / cartographer 发布。
   ⇒ **这是一个我们自己的未解症状**（怀疑与 cartographer 零戳/`Ignored subdivision` 同源，`:870` 遗留②），本文**只能转述、无法定位**。
8. **我们 `slam_toolbox` 的两份参数对 `base_frame` 取值不一致（本文实测 grep）**：
   `src/nav/rm_navigation/rm_navigation/params/mapper_params_online_async.yaml:15` = **`base_frame: livox_frame`**；
   `src/localization/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml:15` = **`base_frame: base_link`**。
   而 `bringup_sim.launch.py:56` 在仿真里加载的是**后者**（`get_package_share_directory('slam_toolbox')/config/mapper_params_online_async_sim.yaml`）。
   ⇒ 仿真路径下取 `base_link`（与 `docs/tf_interface_contract.md:44` 的契约一致），但仓库里**仍留着一份 `livox_frame` 的旧值**；
   两者语义不同（`livox_frame` 相对 `base_link` 有 z≈0.3 偏移，见 `docs/cod_nav_2026_deep_dive.md:66`）。
   **本文未确证那份 `livox_frame` 是否在别的入口（如非 sim 的 launch）下生效** ⇒ **未确定**。
9. **上游 `small_point_lio` 在自己仓库（`ros2` 分支）与我们 `third_party` 里的快照是否完全一致**：本文只核对了 `LICENSE`/`package.xml`/`README` 三处元数据，
   **未做文件级 diff**（上游 2026-08-31 仍在更新，而我们手上的快照来自 COD 的 vendored 目录）。

---

### 附：本文所有引用的本地 `file:line` 与 URL（便于复核）

| 用途 | 引用 |
|---|---|
| 本地 COD 快照身份 | `git -C third_party/cod_nav_2026 log -1` → `0127e200`（2026-04-06）；`remote` = `https://gitee.com/codnavgation/cod_-rm2026_-navigation.git` |
| 静态 `map→odom`（单点，yaw −0.5） | `third_party/cod_nav_2026/src/cod_bringup/launch/singlenav_launch.py:67-89` |
| 静态 `map→odom`（多点，yaw 0） | `third_party/cod_nav_2026/src/cod_bringup/launch/multiplenav_launch.py:95-116` |
| 裁剪盒内联参数 | `third_party/cod_nav_2026/src/cod_bringup/launch/multiplenav_launch.py:35-49` |
| p2l（`/scan`）参数 | `third_party/cod_nav_2026/src/cod_bringup/launch/multiplenav_launch.py:65-84` |
| slam_toolbox 参数 | `third_party/cod_nav_2026/src/cod_bringup/params/mapper_params_online_async.yaml`（`:19,30,31-35`） |
| Nav2 参数（单点/多点） | `third_party/cod_nav_2026/src/cod_bringup/params/singlenav2_params.yaml`（483 行）、`multiplenav2_params.yaml`（476 行） |
| 定位 launch（只有 map_server） | `third_party/cod_nav_2026/src/cod_bringup/launch/localization_launch.py:44,76-79` |
| 先验图 | `third_party/cod_nav_2026/src/cod_bringup/maps/rmul2026.yaml:1-7`；PGM header `P5 255 177` |
| 航点 CSV | `third_party/cod_nav_2026/src/cod_bringup/wps/{go_gain,go_home,patrol_center,patrol_front,patrol_behind,preload_patrol}.csv` |
| `small_point_lio` 源码关键行 | `src/lidar_adapter/livox_pointcloud2.h:25-37`；`src/small_point_lio_node.cpp:26-27,61-62,64-69,90-96,101-102,178-187,197-211`；`src/small_point_lio/small_point_lio.cpp:43-156,167-177`；`src/small_point_lio/estimator.h:22`；`src/common/common.h:13-19`；`config/mid360.yaml:3-7,25,50`；`package.xml:7-9,22`；`README.md:3,5,7,90-95`；`LICENSE.txt:1-3` |
| slam_toolbox 语义 | `src/localization/rm_localization/slam_toolbox/src/slam_toolbox_common.cpp:251-259`（本机同版本源码） |
| goal_approach_controller | `third_party/cod_nav_2026/src/goal_approach_controller/src/goal_approach_controller.cpp:96-126,143-146`；`goal_approach_controller_plugin.xml` |
| 我们的模式/槽位/契约 | `src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py:52-56,124-181,303-406,444-468,486-510` |
| 我们的 nav2 参数 | `src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_{rpp,dwb,teb}.yaml`（rpp: `:1-40`(amcl)、`:178,182-235,241-242,252-288,357-362`） |
| 我们的 slam_toolbox 参数 | `src/localization/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml:13-17,28-32` |
| 我们的地图 | `src/bringup/rm_nav_bringup/map/RMUL2026.yaml:1-7`（`origin [-2.2,-3.15,0]`）；PGM `P5 249 173` |
| 我们的 TF 契约 | `docs/tf_interface_contract.md:17-22,44-53,73-77,126-128` |
| 我们的调试结算 | `docs/debug_fastlio_cartographer.md:867,869-870,872`（§10.2）、`:838-899`（§10） |
| 既有 COD 文档 | `docs/cod_nav_comparison.md:11-12,60,29`；`docs/cod_nav_2026_deep_dive.md:70-74,106-107,116-123,236,292-293,348,495`；`docs/cod_nav_2026_integration_plan.md:19,150-154,158-188`；`docs/cod_nav_nav_stack_inventory.md:16,59-60,95-101,132,168-175`；`docs/cod_nav_2026_params_review.md` §0.1/§0.4 |
| BBS 813022（small_point_lio 原理） | <https://bbs.robomaster.com/article/813022?source=4>（正文在 `window.__NUXT__` → `article.htmlContent`） |
| BBS 1882897（COD 2026 全开源技术报告） | <https://bbs.robomaster.com/article/1882897>（同上提取方式） |
| 上游仓库 | <https://github.com/Yancey2023/small_point_lio>；<https://api.github.com/repos/Yancey2023/small_point_lio>；镜像 <https://www.yanceymc.cn/gitea/Yancey/small_point_lio> |
| COD 提交史 | <https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100>；<https://gitee.com/api/v5/repos/codnavgation/cod_-rm2026_-navigation/commits> |
| COD 分支树 | <https://api.github.com/repos/qza36/COD_NAV/git/trees/rmul2026?recursive=1> |
| GitHub 季前 launch | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/slam.launch.py>；`.../nav.launch.py` |
| COD 导航仓（成品） | <https://gitee.com/codnavgation/cod_-rm2026_-navigation> |
| COD 决策仓 / 下位机 | <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/tree/feature%2Ffinish>；<https://gitee.com/cod_-control/rmcod2026_-sentry> |
| COD loopback sim | <https://github.com/qza36/COD_loopback_sim> |


---


## 来源 17/19：`tdt/cod_nav_nav_stack_inventory.md`


> 调研对象：RoboMaster 哨兵 —— GitHub 用户 [`qza36`](https://github.com/qza36?tab=repositories)、Gitee 组织 [`codnavgation`](https://gitee.com/codnavgation)、[`ustl-cod`](https://gitee.com/ustl-cod)、[`cod_-control`](https://gitee.com/cod_-control)，赛季 2024 / 2025 / 2026。
> 调研方式：**只读**。GitHub REST API + `raw.githubusercontent.com`；Gitee API v5（`/orgs/<org>/repos`、`/repos/<owner>/<repo>/branches`）+ `git clone --filter=blob:none`（blobless 浅克隆，逐文件 `git show`）；BBS 文章的 `window.__NUXT__` payload 解析。
> 本次 Gitee **未遇到限流或拒绝**（API、raw、clone 均成功）；GitHub 未鉴权核心配额全程够用。
> 本文件是本次调研**唯一写入**的文件，未改动本仓库任何其它文件、未运行任何代码。
> 前置文档（本文件不重复其内容，只做增量与修正）：[`docs/cod_nav_open_source_materials.md``cod_nav_open_source_materials.md`、[`docs/cod_nav_2026_deep_dive.md``cod_nav_2026_deep_dive.md`、[`docs/cod_nav_comparison.md``cod_nav_comparison.md`、[`docs/cod_nav_2026_integration_plan.md``cod_nav_2026_integration_plan.md`。

---

## A. 完整性判定

### A0 先说结论（回答「我们拉取的库是否不够完善、缺了什么」）

1. **我们其实一个 COD 仓库都没拉。** 工作区根目录只有 `src/rm_*`（我们自己的 bench）、`tools/`、`third_party/`、`docs/`；`.tmp_research/` 里放的是 **Nav2 上游**文件（`up_nav2_sim.yaml`、`nav2_humble_obstacle_layer.cpp` 等），不是 COD 的。也就是说，此前所有关于 COD 的结论都是**从 raw 文件读出来的文档级复刻**，没有任何可直接编译/复用的代码（见工作区根目录列举与 `.tmp_research/` 内容）。
2. **即使只看 GitHub，也漏掉了一整代 2026 导航。** 2026 赛季导航上位机**不在 GitHub 的 `COD_NAV` 任何分支里**，而在 Gitee [`cod_-rm2026_-navigation`](https://gitee.com/codnavgation/cod_-rm2026_-navigation)（163 文件，仅 `master`，HEAD `0127e200` 2026-04-06「fix: 更新正确的地图文件名」）。它与 GitHub `rmul2026` 有 **10 个同名文件内容不同**，并多出 **90 个文件**（见 §B2）。
3. **最关键的缺失是 `goal_approach_controller`。** BBS 帖里「目标点附近圆周运动 → 让 claude 写了个 wrapper 控制器」这条，落地代码**只在 Gitee 2026 仓库**里（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/goal_approach_controller/src/goal_approach_controller.cpp>），GitHub `rmul2026` 里既没有这个包、`nav2_params.yaml` 里也没有它的参数。我们此前只知道"他们用 wrapper 解决"，不知道它怎么实现（透明代理 MPPI + 近目标限速 + 2 m 内直驱）。
4. **历史与场地资产也缺。** Gitee 的 [`ustl-cod/cod_nav`](https://gitee.com/ustl-cod/cod_nav) fork 保留了 GitHub 上已删除的 `dev` / `dyx-dev` 分支、`terrain_analysis` 子模块指针、以及 **2024/2025 赛季的场地 pgm 地图**（`rmuc_2024` / `rmul_2024` / `test_data/rmul_2025`）；Gitee 2026 仓库另有 2026 场地图 `maps/rmul2026.pgm`。
5. **2026 的"实车收敛值"与 GitHub `rmul2026` 的"激进值"不同。** 例：MPPI `vx_max` 从 GitHub 的 `7.5` 降到 Gitee 2026 的 `2.5`；`cost_scaling_factor` 全局从 `5.0` 提到 `15.0`（多点模式）。**移植时必须以 Gitee 2026 为准**，GitHub `rmul2026` 是季前状态。

### A1 资料级完整性表（我们已有 vs 全部导航相关）

| # | 资料 | 平台 / 链接 | 我们是否已有 | 缺失的内容 | 优先级 |
|---|---|---|---|---|---|
| 1 | `COD_NAV` GitHub（`master`/`rmul2026`/`ul_mppi`/`rmul2026_sim`/`feature-obstacle_escape_node`） | <https://github.com/qza36/COD_NAV> | ✅ 已深读（[`docs/cod_nav_2026_deep_dive.md``cod_nav_2026_deep_dive.md`） | 代码本体未落地；`ul_mppi` / `rmul2026_sim` / `feature-obstacle_escape_node` 三个分支本次未读 | P3 |
| 2 | **Gitee 2026 导航** `cod_-rm2026_-navigation` @`master` | <https://gitee.com/codnavgation/cod_-rm2026_-navigation> | ❌ **完全没有** | `goal_approach_controller`、`cod_bringup`（`singlenav2_params.yaml` / `multiplenav2_params.yaml` / `mapper_params_online_async.yaml` / `navigation_launch.py` / `localization_launch.py` / `auto_save_map.launch.py`）、`maps/rmul2026.*`、6 组 `wps/*.csv`、仓内 `waypoint_editor` 与 `pb_nav2_plugins`、Realsense 真正接入 STVL 的参数 | **P0** |
| 3 | Gitee 2026 决策 `cod_-rm2026_-behavior-tree` @`feature/finish`（9 分支） | <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/tree/feature%2Ffinish> | ⚠️ 只有文档摘要 | `COD_Behavior/src/cod_behavior.cpp`、`cod_bt/*.xml`（10 个树）、`test/test_*.cpp`、`launch/*.csv`、`doc/PubNav2Goal_Patrol.md` 全文、`COD_Serial/` | P1 |
| 4 | Gitee `ustl-cod/cod_nav` fork（`master`/`dev`/`dyx-dev`/`rmul2026`） | <https://gitee.com/ustl-cod/cod_nav> | ❌ 只知道有这两个分支 | `terrain_analysis` 子模块指针、2024/2025 场地 pgm、6 套 `nav2_*_params.yaml` 变体、2025「中线架构」（`FAST_LIO_ROS2`+`loam_interface`+`sensor_scan_generation`+`small_gicp_relocalization`） | P1 |
| 5 | 2024 哨兵上位机方案 `cod_sentry` | <https://gitee.com/ustl-cod/cod_sentry> | ⚠️ 仅 README 摘要 | `rm_navigation_ws` / `rm_decision_ws` / `rm_vision_ws` 内容与脚本 `nav.sh` | P2 |
| 6 | 2026 下位机 `rmcod2026_-sentry` | <https://gitee.com/cod_-control/rmcod2026_-sentry> | ❌ | 舵轮解算、RLS 功率拟合、变速小陀螺、超电、热量控制（**决定上位机 cmd_vel 语义**）；`H7文档.pdf`、`gimbal/Document/Quaternion.pdf` 未下载 | P2 |
| 7 | 2026 自瞄 `sentry-auto-aim`（含 `rm_bringup`） | <https://gitee.com/ustl-cod/sentry-auto-aim> | ⚠️ 仅提到 | 16 字节上下位机帧协议、`rm_bringup` 启动项（**其导航包 `rm_navigation` 不在仓库内**） | P3 |
| 8 | 2025 电控 `sentry_steering_25` | <https://gitee.com/cod_-control/sentry_steering_25> | ❌ | 舵轮底盘文件结构（与 `COD_NAV master` 同赛季） | P3 |
| 9 | BBS 技术报告 1882897 | <https://bbs.robomaster.com/article/1882897> | ⚠️ 摘要 | **全文已抓取**（payload 里 `htmlContent`），本文 §C 逐条落地 | ✅ 本文件完成 |
| 10 | `COD_loopback_sim`（7★） | <https://github.com/qza36/COD_loopback_sim> | ❌ | Humble 版 `nav2_loopback_sim`、`pixi` 一键部署、配套 `nav2_params.yaml` 与地图 | **P0** |
| 11 | `waypoint_editor`（fork + Gitee 仓内副本） | <https://github.com/qza36/waypoint_editor> · <https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/waypoint_editor> | ❌ | 两个桥接节点（`waypoint_to_nav2` / `waypoint_through_nav2`）与 CSV 格式 | **P0** |
| 12 | `box_lidar_filter` / `cpp_lidar_filter` | <https://github.com/qza36/box_lidar_filter> · <https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/cpp_lidar_filter> | ⚠️ 只有参数摘要 | 代码 + **按我们 `livox_frame` 重标定**；README 与源码的坐标系说法不一致（见 §C-15） | **P0** |
| 13 | `COD_Behavior`(2025) / `ros2_simple_serial` / `Sentry_vision` / `Sentry_ws` | <https://github.com/qza36/COD_Behavior> · <https://github.com/qza36/ros2_simple_serial> · <https://github.com/qza36/Sentry_ws> | ⚠️ 仅提到 | 2025 决策树、串口包、汇总工作空间 `.gitmodules` | P2 |
| 14 | `map_module` / `planning_module` / `litenav` | <https://github.com/qza36/map_module> · <https://github.com/qza36/planning_module> · <https://github.com/qza36/litenav> | ❌ | 作者新一代自研导航雏形（脱离 Nav2） | P3 |
| 15 | `COD_PCD2PGM` / `gridmap` | <https://github.com/qza36/COD_PCD2PGM> · <https://github.com/qza36/gridmap> | ❌ | 2.5D 高程图路线（vendor `grid_map`） | P3 |
| 16 | `enemy_tracking` / `CODSentryPAC` | <https://github.com/qza36/enemy_tracking> · <https://github.com/qza36/CODSentryPAC> | ❌ | 追击攻击点生成 / 自研 ReplanFSM 轨迹生成 | P3 |
| 17 | `rmu_gazebo_simulator`（fork，`jazzy`） | <https://github.com/qza36/rmu_gazebo_simulator> · 上游 <https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator> | ❌ | `rmul_2024/rmuc_2024/rmul_2025/rmuc_2025/rmuc_2026` 世界模型、**离线 PCD 导出** | P1（注意是 `jazzy`，与我们 Humble bench 不直接通用） |
| 18 | 研究型 fork 群（2026+ 方向） | ROG-Map: <https://github.com/qza36/ROG-Map-ROS2> · far_planner: <https://github.com/qza36/far_planner> · PCT_planner: <https://github.com/qza36/PCT_planner> · PCT_planner_ros2_cpu（自建）: <https://github.com/qza36/pct_planner_ros2_cpu> · jie_3d_nav: <https://github.com/qza36/jie_3d_nav> · KISS-Matcher: <https://github.com/qza36/KISS-Matcher> · kiss-icp: <https://github.com/qza36/kiss-icp> · small_gicp_relocalization: <https://github.com/qza36/small_gicp_relocalization> · spark-fast-lio: <https://github.com/qza36/spark-fast-lio> · DDR-opt: <https://github.com/qza36/DDR-opt> · TRG-planner: <https://github.com/qza36/TRG-planner> · Kinodynamic_esdf_mpc: <https://github.com/qza36/Kinodynamic_esdf_mpc> · MARSIM-ROS2: <https://github.com/qza36/MARSIM-ROS2> | ❌ | 全部为 fork，**本次未读内容**，仅据 BBS 展望与描述推断其方向（**不推测**为自研） | P3（储备） |
| 19 | B 站讲解视频 2 集 | <https://www.bilibili.com/video/BV1XSXZBUEYL/> · <https://www.bilibili.com/video/BV1r79cBME6Y/> | ❌ | 视频内容未转写（无字幕接口可读），只知道 Gitee README 将其标为「该导航程序详细解说教程」 | P2 |
| 20 | 场地 pgm 地图（2024/2025/2026） | <https://gitee.com/ustl-cod/cod_nav/blob/master/nav_bringup/map/rmuc_2024.yaml> · <https://gitee.com/ustl-cod/cod_nav/blob/master/nav_bringup/map/rmul_2024.yaml> · <https://gitee.com/ustl-cod/cod_nav/blob/master/test_data/rmul_2025.yaml> · <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/maps/rmul2026.yaml> | ❌ | 分辨率/origin 见 §B4；可用于与我们仿真场地做对照 | P1 |

**一句话**：我们此前的资料是**文档级**的，且**只覆盖 GitHub 的 2025 主线 + 2026 季前**；缺的是 **Gitee 2026 成品导航（含 goal wrapper）、2024/2025 历史与场地资产、以及可直接跑的 loopback/waypoint 工具链**。

---

## B. 导航相关仓库逐项清单

> 「与 COD_NAV 的关系」指与 <https://github.com/qza36/COD_NAV>（2025，`master`/`rmul2026`）的关系。

### B1 GitHub `qza36`（共 74 仓；下表只列导航相关 27 个）

| 仓库 | 赛季/年份 | 内容 | 与 COD_NAV 的关系 | 独有内容 | 建议拉取 |
|---|---|---|---|---|---|
| [`COD_NAV`](https://github.com/qza36/COD_NAV) | 2025（`master` 2025-08-02）/ 2026 季前（`rmul2026` 2026-03-10） | 哨兵导航上位机；`master`=FAST-LIO+patchwork+++small_gicp 重定位+pcd2pgm+SmacHybrid+pb_omni PID；`rmul2026`=small_point_lio+slam_toolbox lifelong+裁剪盒+MPPI Omni+双 STVL | 本体 | `CLAUDE.md` 架构文档（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/CLAUDE.md>） | ✅ 仅作对照（已深读） |
| [`COD_loopback_sim`](https://github.com/qza36/COD_loopback_sim) 7★ | 2026-04-25 | 把 Nav2 官方 `loopback_sim` **回移到 Humble** + RoboStack/pixi 一键部署；含 `nav2_loopback_sim/params/nav2_params.yaml`、`maps/{depot,warehouse,tb3_sandbox}.pgm`、`graphs/*.geojson` | 独立工具，被 2026 决策开发依赖（BBS 明确推荐） | README 自述动机：**用 Gazebo 做早期路径规划/控制验证「不必要地麻烦」，物理引擎伪影（odom 漂移、定位误差）是干扰**；loopback 隔离控制逻辑（<https://github.com/qza36/COD_loopback_sim>） | ✅ **P0** |
| [`waypoint_editor`](https://github.com/qza36/waypoint_editor)（fork of [kzm784](https://github.com/kzm784/waypoint_editor)） | 2026-03-01 | RViz 面板打点 + CSV 导出；分支 `feature/adaptToNav2` 与 `USAGE_WITH_NAV2.zh.md` | 2026 导航与决策的航点来源 | 两个桥接节点：`waypoint_to_nav2`（`follow_waypoints` 逐点停靠）/ `waypoint_through_nav2`（`navigate_through_poses` 平滑穿越）；文档明说「waypoint_editor 和 Nav2 都会启动 `map_server` 导致冲突」 | ✅ **P0** |
| [`box_lidar_filter`](https://github.com/qza36/box_lidar_filter) | 2026-01-11 | `pcl::CropBox` 挖掉车体点云，输出 `/livox/lidar_filtered` | 2026 导航仓里改名为 `cpp_lidar_filter` | README 明确动机：「**由于 `stvl` 无法显式设置最小障碍物半径**，所以开发此包裁切源点云以避免将机器人自身注册成障碍物」（<https://github.com/qza36/box_lidar_filter>） | ✅ **P0** |
| [`pointcloud_fusion`](https://github.com/qza36/pointcloud_fusion) | 2025-11-04 | 双雷达 `message_filters` 近似时间同步 + TF2 对齐融合 | 未被 COD_NAV 引用 | 多雷达方案储备 | △ 备用 |
| [`map_module`](https://github.com/qza36/map_module) | 2026-04-08 | 最小 2D 地图模块（`map_io.hpp/cpp`），部分改写自 `nav2_map_server` | 同作者新一代自研导航的一块 | 脱 Nav2 依赖的地图 IO | △ P3 |
| [`planning_module`](https://github.com/qza36/planning_module) | 2026-04-08 | 最小 2D 规划模块（`planning.hpp/cpp`），与 `map_module` 配套 | 同上 | 最小规划实现 | △ P3 |
| [`litenav`](https://github.com/qza36/litenav) | 2026-05-01 | `app_module` / `core_types` / `litenav_ros2` / `map_module` / `plan_module`，pixi+robostack（含 `.codex`） | 作者「轻量导航」雏形 | 脱离 Nav2 的完整 app 骨架 | △ P3 |
| [`COD_PCD2PGM`](https://github.com/qza36/COD_PCD2PGM) 5★ | 2026-09-28 | pcd → 2.5D 高程栅格，vendor `grid_map`；保存方式 `map_saver_cli -t /elevation_grid` | 与 `master` 的 pcd2pgm 子模块同路线 | 独立可跑的 2.5D 高程图工具 | △ P3 |
| [`gridmap`](https://github.com/qza36/gridmap) | 2024-12-25 | `grid_map` vendor + `pcd_to_gridmap_demo` | `COD_PCD2PGM` 的前身 | README：**修改自沈阳航空航天大学 TUP 战队 2023 年哨兵导航模块**（<https://github.com/qza36/gridmap>） | △ P3 |
| [`enemy_tracking`](https://github.com/qza36/enemy_tracking) | 2026-01-23 | 订阅 `rm_interfaces/msg/Target` → 敌人周围圆形生成候选攻击点 → costmap 过滤 → 选最近可达 → 调 Nav2；含 `BehaviorTree.ROS2` | 独立任务层包 | 算法参考 `pb2025_sentry_behavior` 的 `CalculateAttackPose`（<https://github.com/qza36/enemy_tracking>） | △ P3 |
| [`CODSentryPAC`](https://github.com/qza36/CODSentryPAC) | 2026-04-08 | 自研轨迹生成：`ReplanFSM` → `planner_manager` → `TopoSearcher`/`AstarFinder`/`GlobalMap`，面向低速差速履带 | 与 Nav2 路线不同的自研架构 | `trajectory_generation/` 全套 + `bev/occ` 栅格 map | △ P3 |
| [`Sentry_ws`](https://github.com/qza36/Sentry_ws) | 2025-04-20 | 汇总工作空间；`.gitmodules` 串 `COD_Behavior`/`COD_NAV`/`Sentry_vision`/`ros2_simple_serial` | 2025 全栈入口 | 一次性拉 2025 四件套 | △ P2 |
| [`COD_Behavior`](https://github.com/qza36/COD_Behavior) | 2025-04-30 | 2025 决策（`cod_bt/t1.xml`、`src/sentry_behavior.cpp`） | 被 `Sentry_ws` 引用 | 2025 版决策 | △ P2 |
| [`ros2_simple_serial`](https://github.com/qza36/ros2_simple_serial) | 2025-12-06 | `/cmd_vel` → 串口；README 自述**已知问题：下位机连发导致串口卡死** | 2026 导航仓里为 `ros2_simple_serial`（包名 `cod_serial_ul26`） | 串口最小实现 | △ P2 |
| [`COD_NAV_NEXT`](https://github.com/qza36/COD_NAV_NEXT) | 2025-09-20 | 顶层 `cnn_driver` / `cnn_perception` / `cnn_planner`，**无 README** | 早期「下一代」尝试 | 用途无法确证（见 §E） | ❌ 暂不 |
| [`rmu_gazebo_simulator`](https://github.com/qza36/rmu_gazebo_simulator)（fork） | fork，`jazzy` 分支，2026-09-29 | `rmul_2024/rmuc_2024/rmul_2025/rmuc_2025/rmuc_2026` 世界模型、网页联机、**地图模型离线导出 PCD**、`rmoss_core`/`pb2025_robot_description` | 与 COD_NAV 无直接依赖 | 2026 场地世界模型 + PCD 导出（Jazzy，需评估 Humble 兼容） | △ P1（仅取 world/PCD 资产） |
| [`sentry_planning`](https://github.com/qza36/sentry_planning)（fork of [zzt-180/sentry_planning](https://github.com/zzt-180/sentry_planning)） | fork | 描述：「cod_nav——基于哈工大 RM2024 赛季烧饼自主导航规划代码」 | 他队代码，非 COD 自研 | 归属明确为他队 | ❌ |
| [`pct_planner_ros2_cpu`](https://github.com/qza36/pct_planner_ros2_cpu) | 自建，2026-07-07 | PCT 3D 导航（CPU 版） | 2026 之后的新方向 | 3D 导航储备 | ❌ 暂不 |
| fork 群 | — | `ROG-Map-ROS2`、`far_planner`、`PCT_planner`、`jie_3d_nav`、`KISS-Matcher`、`kiss-icp`、`small_gicp_relocalization`、`spark-fast-lio`、`DDR-opt`、`TRG-planner`、`Kinodynamic_esdf_mpc`、`MARSIM-ROS2`、`autonomous_exploration_development_environment`、`FreeDOM-ROS2`、`octomap_mapping`、`LIO-SAM-MID360-ROS2` | 全部为他人上游 | 反映其 2026+ 关注面（3D 导航 / 重定位 / 动态物体去除 / 探索） | ❌ 暂不（本次未读） |
| [`myblog`](https://github.com/qza36/myblog) / <https://qza36.github.io/myblog/> | 2026-06-24 | 个人博客（已确认存在 1 篇 CLion + micro-ROS 技术文，见 `docs/cod_nav_open_source_materials.md` §B1） | 作者视角 | 导航相关文章**本次未逐一核对** | △ P3 |

### B2 Gitee `codnavgation/cod_-rm2026_-navigation` @`master`（**本次最重要的新发现**）

- 仓库：<https://gitee.com/codnavgation/cod_-rm2026_-navigation>（15★，163 文件，仅 `master`，HEAD `0127e200`「fix: 更新正确的地图文件名」2026-04-06）。
- README：<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/README.md> — 10 个包：`cod_bringup`（launch/params/maps/wps/behavior_trees）、`cpp_lidar_filter`、`fake_vel_transform`、**`goal_approach_controller`**、`pb_nav2_plugins`、`pb_omni_pid_pursuit_controller`、`pointcloud_to_laserscan`、`ros2_simple_serial`、`small_point_lio`、`waypoint_editor`。
- 与 GitHub `rmul2026` 的关系：**同一作者的后续版本**。文件级比对（本次 blobless 克隆后逐文件对比）：**71 个同名文件，其中 10 个内容不同、61 个完全相同**；Gitee 多 **90 个文件**（`cod_bringup` 全套、`goal_approach_controller`、`pb_nav2_plugins`、`pb_omni_pid_pursuit_controller`、`ros2_simple_serial`、`waypoint_editor`、`README.md`），GitHub 多 **40 个**（`.idea/`、`CLAUDE.md`、`nav_bringup/scripts/`、`urdf/`、`resource/`、旧 `nav_bringup/*`）。
- 内容不同的 10 个文件（`<` = GitHub `rmul2026`，`>` = Gitee `master`）：
  - `cpp_lidar_filter/src/filter_node.cpp`：输出 QoS 从 `rclcpp::SensorDataQoS()` 回退为 `10`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cpp_lidar_filter/src/filter_node.cpp>）。
  - `small_point_lio/config/mid360.yaml`：`save_pcd: false`（GitHub 为 `true`）（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/small_point_lio/config/mid360.yaml>）。
  - `fake_vel_transform/*`（5 个文件）与 `README.md`/`.gitignore`：逐行比对**无实质差异**（疑似行尾差异导致 blob 哈希不同）。**结论：GitHub 的 `spin_speed` 无条件覆盖 `angular.z` 行为在 Gitee 2026 里同样存在**。
  - `behavior_trees/*.xml` 与 GitHub 完全一致（`diff` 为空）。
- **独有的 2026 成品部件**：
  - `goal_approach_controller`：Nav2 控制器 wrapper（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/goal_approach_controller/src/goal_approach_controller.cpp>）。
  - `cod_bringup/params/singlenav2_params.yaml`（先验图+场地图，<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/singlenav2_params.yaml>）与 `multiplenav2_params.yaml`（slam 在线建图，<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/multiplenav2_params.yaml>）——**两套参数差别很大**（见 §C）。
  - `cod_bringup/wps/`：`go_gain.csv` / `go_home.csv` / `patrol_center.csv` / `patrol_front.csv` / `patrol_behind.csv` / `preload_patrol.csv`，README 给出语义顺序「去增益点 / 回启动区 / 增益点后半区巡逻 / 增益点内四角巡逻 / 增益点前半区巡逻 / 前压巡逻」（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/cod_bringup/wps>）。
  - `cod_bringup/launch/auto_save_map.launch.py`：slam_toolbox 模式下按 `intervals=[30,60,...,300]` 秒用 `map_saver_cli` 存图（**路径写死 `/home/cod-sentry/dyx_ws/...`**，<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/launch/auto_save_map.launch.py>）。
  - `cod_bringup/launch/singlenav_launch.py`：`cpp_lidar_filter` + `small_point_lio` + **静态 `map→odom`（z=0.05，yaw=`-0.5`）** + `fake_vel_transform` + `cod_serial_ul26` + RealSense（`depth_module.depth_profile: 424x240x90`「最高帧率」+ spatial/temporal filter + 无序点云）+ Nav2 + RViz（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/launch/singlenav_launch.py>）。
  - `cod_bringup/launch/multiplenav_launch.py`：裁剪盒 + small_point_lio + `pointcloud_to_laserscan` + `slam_toolbox`（`mapper_params_online_async.yaml`）+ 静态 TF + `auto_save_map.launch.py`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/launch/multiplenav_launch.py>）。
  - `cod_bringup/launch/localization_launch.py`：`lifecycle_nodes = ['map_server']`，默认图 `maps/rmul2026.yaml`（**依然无 AMCL**，<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/launch/localization_launch.py>）。
  - `pb_nav2_plugins`：`BackUpFreeSpace`（行为）+ `IntensityVoxelLayer`（代价图层），来自北极熊（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/pb_nav2_plugins/README.md>）；**`behavior_server.backup` 已启用 `pb_nav2_behaviors/BackUpFreeSpace`，但 `intensity_voxel_layer` 在两套 params 里都未被引用（grep 0 命中）**。
  - 仓内 `waypoint_editor/USAGE_WITH_NAV2.zh.md`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/waypoint_editor/USAGE_WITH_NAV2.zh.md>）。
  - `ros2_simple_serial/src/cod_serial.cpp`：订阅 `aft_cmd_vel`，帧 = `0xA5` + `float vx/vy/vz`（偏移 1/5/9）+ `uint16 checksum`（偏移 13）= 15 字节（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/ros2_simple_serial/src/cod_serial.cpp>）。
- `mapper_params_online_async.yaml` 与 GitHub `mapper_params_async.yaml` 的差异（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/mapper_params_online_async.yaml>）：`mode: lifelong` 不变、`transform_publish_period: 0.0` 不变；`map_update_interval` 2.5→**1.0**、`minimum_time_interval` 0.5→**0.3**、`minimum_travel_heading` 0.5→**0.1**、`max_laser_range` 5.0→**10.0**、`min_laser_range` 0.01→**0.2**。

### B3 Gitee `codnavgation/cod_-rm2026_-behavior-tree`（9 分支；BBS 指向 `feature/finish`）

- 仓库：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>；分支：`dev`、`feat/AddNavigateThroughPoses`、`feat/AddPubGoalInterval`、`feat/AddWaypointFollowAction`、`feature/finish`、`feature/finish1`、`feature/pub-nav2-patrol`、`master`、`test1`。`feature/finish` HEAD `7cba6de`「latest」2026-04-02。
- 内容：`COD_Behavior/`（`src/cod_behavior.cpp`、`include/cod_behavior/{action,condition,include}.h`、`cod_bt/` 10 个 XML、`test/test_{GoZone,LowHp,OnZone,Recover}.cpp`、`launch/*.csv|*.yaml`）+ `COD_Serial/` + `rm_interfaces/` + `BehaviorTree.ROS2/`。
- 独有文档：《COD 哨兵决策说明文档（26 赛季 RMUL）》<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/README.md>（裁判系统消息 0x0001/0x0002/0x0003/0x0101/0x0104/0x0201/0x0203/0x0206/0x0207/0x0208/0x020B/0x020D 与每个功能的触发条件）；《PubNav2Goal 巡逻方案使用文档》<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/doc/PubNav2Goal_Patrol.md>（`LoadWaypoints`/`GetCurrentWaypoint`/`WaitUntilReached`/`WaitDuration`/`NextWaypoint` 端口表）。
- 与 GitHub `COD_Behavior`（2025）的关系：**完全重写**（2025 版只有 `cod_bt/t1.xml` + `sentry_behavior.cpp`）。
- 工程卫生：`feature/finish` 里提交了 `COD_Behavior/build/`、`install/`、`log/`、`.idea/`（`git ls-tree` 可见）。

### B4 Gitee `ustl-cod/cod_nav`（GitHub 的 fork，但**保留 GitHub 已删除的分支与地图**）

分支与 HEAD（`git clone` 实测）：

| 分支 | HEAD | 独有内容 |
|---|---|---|
| `master` | `4aa7095`「final rmul」2025-04-02 | 2024/2025 场地图 + 6 套参数变体：`nav_bringup/params/nav2_params.yaml`、`nav2_pure_params.yaml`、`nav2_pbOmni_params.yaml`、`nav2_slam_params.yaml`、`nav2_slop_params.yaml`、`sim_nav2_params.yaml`；`codmap.pgm/yaml`；`test_data/rmul_2025.*`（<https://gitee.com/ustl-cod/cod_nav/tree/master>） |
| `dyx-dev` | `df1dec3`「5.11」2025-05-11 | **`terrain_analysis`（gitlink，commit `c82f23c774251a564f5039b9eb3ffb4ab0a9a59c`）**；`bringup.launch.py` 里有 `get_package_share_directory('terrain_analysis')`，但 `IncludeLaunchDescription` 被**注释掉**（<https://gitee.com/ustl-cod/cod_nav/blob/dyx-dev/nav_bringup/launch/bringup.launch.py>）；`nav2_params.yaml` 里 `voxel_layer` 吃 `/patchworkpp/nonground`，并**注释保留**了 `intensity_voxel_layer`（`pb_nav2_costmap_2d::IntensityVoxelLayer` + `terrain_map` 观测源）整段（<https://gitee.com/ustl-cod/cod_nav/blob/dyx-dev/nav_bringup/params/nav2_params.yaml>） |
| `dev` | `492fcc4`「update README」2025-07-31 | 带 `pcd2pgm/`、`small_gicp_relocalization/`、`cmake-build-debug/`（提交进仓库） |
| `rmul2026` | `6e03b83`「chore:add stvl to global map」2025-12-19 | **2025→2026 的中间路线快照**：`FAST_LIO_ROS2/`、`nav_bringup/map/{map,blue}.pgm|yaml`、`nav_bringup/params/nav2_params_amcl.yaml`（有 AMCL 参数）、`script/pcd2pgm.sh`（<https://gitee.com/ustl-cod/cod_nav/tree/rmul2026>）。比 GitHub `rmul2026`（2026-03-10）**旧**，但保留了后者后来删掉的重定位/AMCL/pcd2pgm 路线 |

场地图 yaml（`resolution: 0.05, mode: trinary` 全部一致）：

| 地图 | origin | 赛季 | 链接 |
|---|---|---|---|
| `codmap` | `[-7.2, -8.18, 0]` | 2024 练习 | <https://gitee.com/ustl-cod/cod_nav/blob/master/codmap.yaml> |
| `nav_bringup/map/rmuc_2024` | `[-6.35, -7.6, 0]` | RMUC 2024 | <https://gitee.com/ustl-cod/cod_nav/blob/master/nav_bringup/map/rmuc_2024.yaml> |
| `nav_bringup/map/rmul_2024` | `[-3.75, -4.54, 0]` | RMUL 2024 | <https://gitee.com/ustl-cod/cod_nav/blob/master/nav_bringup/map/rmul_2024.yaml> |
| `test_data/rmul_2025` | `[-1.33, -4.51, 0]` | RMUL 2025 | <https://gitee.com/ustl-cod/cod_nav/blob/master/test_data/rmul_2025.yaml> |
| `nav_bringup/map/map`（rmul2026 分支） | `[-20.8, -9.42, 0]` | 2026（大图） | <https://gitee.com/ustl-cod/cod_nav/blob/rmul2026/nav_bringup/map/map.yaml> |
| `nav_bringup/map/blue`（rmul2026 分支） | `[-27.1, -18.4, 0]` | 2026（大图） | <https://gitee.com/ustl-cod/cod_nav/blob/rmul2026/nav_bringup/map/blue.yaml> |
| `cod_bringup/maps/rmul2026`（Gitee 2026） | `[-1.756, -7.036, 0]`，pgm 尺寸 **255×177**（≈12.75 m × 8.85 m） | RMUL 2026 场地 | <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/maps/rmul2026.yaml> |

`terrain_analysis` 的事实边界：它是 **gitlink（160000）**，但 `dyx-dev` 的 `.gitmodules` **只列了 `patchwork-plusplus`**，没有 `terrain_analysis` 条目 ⇒ 无法从该 fork 得到其源码；`pb_nav2_plugins` README 推荐的是北极熊的 [`terrain_analysis`](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis)，但**COD 用的具体来源本次未能确证（不推测）**，见 §E。

### B5 其它 Gitee 资料（与导航相关部分）

| 资料 | 链接 | 与导航的关系 | 独有内容 |
|---|---|---|---|
| `ustl-cod/cod_sentry`（2024 哨兵上位机方案，2★） | <https://gitee.com/ustl-cod/cod_sentry> | 2024 导航栈 = `rm_navigation_ws` | README 即方案文档：三模块（决策/导航/自瞄）交互极少、硬件 Livox MID-360 + i7-1270P + 海康 MV-CS016-10UC 8mm、导航**参考深北莫北极熊 `pb_rm_simulation`**；运行脚本 `nav.sh`/`decision.sh`/`vision.sh` |
| `ustl-cod/sentry-auto-aim`（2026 自瞄） | <https://gitee.com/ustl-cod/sentry-auto-aim> | 其 `rm_bringup` 是**视觉侧**启动包 | `使用方法.md` 给出上下位机 **16 字节帧**（上位→下位：`0xff` + Fire + pitch(4B) + yaw(4B) + distance(4B) + 空 + `0x0d`；下位→上位：`0xff` + mode + roll/pitch/yaw），并写明「**当前并未添加导航模块**」；`rm_bringup/launch/bringup_navigation.launch.py` 依赖外部 `rm_navigation` 包（该包不在此仓） |
| `cod_-control/rmcod2026_-sentry`（2026 下位机，11★） | <https://gitee.com/cod_-control/rmcod2026_-sentry> | **决定上位机 cmd_vel 如何变成轮速**（BBS 明确要求先对齐这一环） | 舵轮解算四轮 `atan2(Vx±Vw·sin45·R, Vy±Vw·sin45·R)` + 优劣弧 `K[i]=±1`；功率控制 RLS 拟合（`RLS_Init(&RLS_Power_Info, 4, 1, 0.99999, 1e-5)`，`W[0..3]=1.453e-07/1.23e-07/…`，舵向优先 80% 功率，隶属度分配 + 求根公式解限幅）；超电阈值 30%；变速小陀螺；热量控制 |
| `cod_-control/sentry_steering_25`（2025 舵轮哨兵电控） | <https://gitee.com/cod_-control/sentry_steering_25> | 与 `COD_NAV master` 同赛季 | 底盘/云台分仓、DM-MC-Board02(H723) + Keil/AC6 工具链 |
| 组织主页 | <https://gitee.com/codnavgation>（4 仓）· <https://gitee.com/ustl-cod>（13 仓）· <https://gitee.com/cod_-control>（18 仓） | 索引 | `codnavgation/cod_-rm2026_-{perception,simluation}` 仍为占位 |

---

## C. 优化点汇总表（最重要一节）

> 来源标注：**[BBS]** = <https://bbs.robomaster.com/article/1882897>（作者原文，本文件已从 `window.__NUXT__` 的 `htmlContent` 完整提取）；**[G26]** = Gitee 2026 导航仓 `master`；**[GH26]** = GitHub `COD_NAV` `rmul2026`；**[CTL]** = <https://gitee.com/cod_-control/rmcod2026_-sentry>。
> 「对我们适用性」以我们 bench 的现状为基线（`robot_radius 0.40`、场地 15×28 m、NavFn + cartographer + 4 种重定位、`/scan` 10 m、GST 仿真；见 [`docs/cod_nav_2026_integration_plan.md``cod_nav_2026_integration_plan.md`）。

| # | 问题（他们的语境） | 做法 | 参数 / 文件 | 理由（原文/注释） | 对我们适用性 |
|---|---|---|---|---|---|
| C1 | **VoxelLayer 清除不干净**：高速移动时频繁避障、「遥遥晃晃」 | 改用 **STVL**（不靠射线清理，靠时间衰减），两张 costmap 都挂 | `voxel_decay 0.5`、`decay_model 0`(linear)、`voxel_size 0.05`、`observation_persistence 0.0`、`combination_method 1`、`update_footprint_enabled true`、源级 `obstacle_range 8.0/raytrace_range 9.0`、`min_obstacle_height 0.15`、`max_obstacle_height 1.0`、**`model_type 1`**、`vertical_fov_angle 2.00`、`filter: voxel`；**[G26]** <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/singlenav2_params.yaml> | **[BBS]**「nav2_costmap_2d::VoxelLayer 会导致成本地图的清除不够干净，总有噪点残留，会让高速移动下的哨兵频繁避障，遥遥晃晃」；STVL 官方描述被其原样引用：「he Temporal in this package is the novel concept of voxel_decay whereas we have configurable functions that expire voxels」 | **直接可用**（包已装）。注意 layer 级 `obstacle_range 3.0` 被源级 `8.0` 覆盖（同名参数陷坑） |
| C2 | 地形/坡道感知怎么做 | **把地形处理放到 costmap 之外**（解耦），外部算好再喂；上赛季坡道用 `patchwork-plusplus`，「效果还凑合，调一调也能过洞」；`pb_nav2_plugins.IntensityVoxelLayer` 就是为此（按强度标记 + `terrain_map`） | **[BBS]**；`IntensityVoxelLayer` 参数 `min/max_obstacle_intensity`（域在 `obstacle_layer` 下而非 source 下），推荐配 `terrain_analysis`；[G26] `pb_nav2_plugins/README.md` <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/pb_nav2_plugins/README.md>；COD 自己的候选接线在 [fork `dyx-dev`] <https://gitee.com/ustl-cod/cod_nav/blob/dyx-dev/nav_bringup/params/nav2_params.yaml>（整段被注释） | 解耦「方便算法测试以及更换」 | **需改造**：我们走 linefit → `/segmentation/obstacle`。可借其「外部处理 + 强度层」的思路，但 `intensity_voxel_layer` 在 2026 实跑里**未被启用**（grep 0），不要误判为"他们在用" |
| C3 | **单点导航容易撞墙**，脱困极浪费时间，「在 RM 分秒必争的比赛里是致命的」 | 抛弃单点 `NavigateToPose`，改 **`Navigate Through Poses`**：规划一条连续路径穿越所有航点、中间不停留 | BT：`nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`（`RemovePassedGoals radius=0.7` → `ComputePathThroughPoses`）；**[GH26]** <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml>（与 [G26] 同名文件 `diff` 为空） | **[BBS]**「机器人规划一条连续路径穿越所有航点，中间不停留，这样就可以让机器人非常丝滑地在无先验地图的情况下快速按照既定路线移动」 | **直接可用**（我们 stock BT 未用 through-poses；`nav2_bt_navigator` 已装） |
| C4 | 航点怎么来、怎么被决策消费 | RViz 打点 → 存 CSV → 决策解析 CSV；巡逻/前压也用 CSV | **[BBS]**；`waypoint_editor`（[G26] 仓内）<https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/waypoint_editor>；6 组语义 CSV <https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/cod_bringup/wps>；CSV 列 `id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command` | **[BBS]**「此地图由仿真环境构建，路径点由 waypoint_editor 创建」 | **直接可用**。附带的两个坑要注意：`waypoint_editor` 与 Nav2 **都会启 `map_server` 冲突**；CSV 不能直接被 Nav2 用，要过 `waypoint_to_nav2` / `waypoint_through_nav2` 桥（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/waypoint_editor/USAGE_WITH_NAV2.zh.md>） |
| C5 | MPPI「调试过程及其痛苦」 | **不要一味调 `std`**，关注 `temperature` 与 `gamma` | `temperature: 0.25`、`gamma: 0.015`（[G26] 单点）/ `0.008`（[GH26]）；`vx_std/vy_std 0.3`、`wz_std 0.2`（[G26] 单点）/ `0.35/0.35`（[G26] 多点） | **[BBS]**「个人认为不能一味调试 std，而是去关注 temperature 和 gamma，可以有效的避免晃动」 | **直接可用**（我们 P2 计划里就有 MPPI 槽） |
| C6 | `PathFollow` / `PathAlign` 权重与阈值 | **[BBS] 说**权重不宜 >10、`threshold_to_consider` 不宜 >1.0；**代码里并不一致** | [G26] 单点：`PathFollowCritic cost_weight 15.0 / threshold 1.0`、`PathAlignCritic 15.0 / 0.8`；[G26] 多点：`PathFollow 8.0 / 3.0`、`PathAlign 15.0 / 3.0`；[GH26]：`5.0/1.5`、`10.0/1.5` | [BBS] 是口头经验；代码注释另有一套（「放大关闭距离，让 GoalCritic 在近目标区域主导，防止画弧」） | **需改造**：文档与代码矛盾，**不要照抄**；我们按自己的目标点震荡现象做 A/B（每次一个变量） |
| C7 | 目标点附近**切向绕圈** | **拉高 `GoalCritic` 权重与 `threshold_to_consider`**（作者自称「邪修调参」） | [G26] 单点 `GoalCritic cost_weight 25.0 / threshold 4.0`；多点 `20.0 / 3.0`；[GH26] `15.0 / 2.5` | **[BBS]**「而我实际的做法是拉高 GoalCritic 的权重和 threshold_to_consider，邪修调参」 | **直接可用**（低成本，先试） |
| C8 | 圆周运动「怎么也调不好」 | **不调 MPPI，做一层 wrapper 控制器**：`GoalApproachController` 透明代理内部控制器（默认 MPPI），近目标限速 + 更近时绕开 MPPI 直接 P 控朝目标走 | `inner_plugin: nav2_mppi_controller::MPPIController`；`approach_distance 2.5`、`approach_velocity 0.2`、`direct_approach_distance 2.0`、`direct_approach_kp 3.0`；直驱分支 `cmd.twist.angular.z = 0`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/goal_approach_controller/src/goal_approach_controller.cpp>）；插件声明 <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/goal_approach_controller/goal_approach_controller_plugin.xml> | **[BBS]**「后来干脆就不调了，让 claude 写了个 wrapper 控制器，实际的控制器还是 mppi…在靠近目标点附近时，强制固定速度，且只用 kp 引导至目标点，这样做非常完美的解决了上述的问题」 | **直接可用（最高价值）**，但两条硬约束：① 它会把 `angular.z` 置 0，**与"控制器控朝向"互斥**；② 需自己 `colcon build` 这个包（仅依赖 `nav2_core`/`pluginlib`） |
| C9 | 到点判定：10 cm 太严，高速 omni 会绕圈 | 单点用 **`PositionGoalChecker`**（只查 xy + 路径长度，不看朝向）；多点用放宽的 `SimpleGoalChecker` | [G26] 单点 `plugin: nav2_controller::PositionGoalChecker, xy_goal_tolerance 0.2, path_length_tolerance 0.5, stateful False`；多点 `SimpleGoalChecker xy 0.30 / yaw 6.28`；注释「放宽到 25cm，高速 omni 机器人 10cm 太严导致绕圈」 | **[BBS]**「goal checker 采用的是 nav2_controller::PositionGoalChecker，只需设置 xy 的阈值而不考虑角度朝向」 | **直接可用**（注意我们现有 `goal_checker` 键名是 `general_goal_checker`，需要同名替换） |
| C10 | MPPI 在膨胀区没有梯度 | 调 **inflation 梯度**：加大 `cost_scaling_factor` 让代价快速衰减；`CostCritic.critical_cost` 只对 inscribed/lethal 生效 | [G26] 单点 局部 `cost_scaling_factor 5.0 / inflation_radius 0.55`、全局 `5.0 / 0.75`；[G26] 多点 全局 `cost_scaling_factor **15.0**`；`CostCritic cost_weight 3.8, critical_cost 253.0, consider_footprint true, collision_cost 1e6, trajectory_point_step 2` | 注释原文：「关键修正！原 0.3 衰减太慢，整个膨胀区代价都极高，**MPPI 没有梯度可用**。5.0 让代价快速衰减，形成清晰的『远离障碍物』梯度」；「只有 inscribed/lethal 才触发碰撞惩罚」 | **需改造**：我们 `robot_radius 0.40`（他们单点 footprint 只有 0.2 m 方框、多点 0.3 m），膨胀半径必须按比例重算；建议先只改 `cost_scaling_factor` |
| C11 | 全局规划贴缝走 / 目标落在膨胀格无法规划 | 换 **`SmacPlanner2D`** + `cost_travel_multiplier` + `tolerance` + 内置 `smooth_path` | [G26] `plugin nav2_smac_planner/SmacPlanner2D, tolerance 0.5, allow_unknown true, max_planning_time 4.5, max_iterations 1e6, max_on_approach_iterations 1000, cost_travel_multiplier 4.0, smooth_path True`；`smoother: {max_iterations 10000, w_smooth 0.4, w_data 0.1, tolerance 1e-10, do_refinement true}`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/singlenav2_params.yaml>） | `cost_travel_multiplier` 注释：「较大的值将更精确地放置在通道的中心（如果存在非 FREE 代价势场）」 | **直接可用**；**不要抄** `motion_model_for_search: "DUBIN"`、`angle_quantization_bins`、`minimum_turning_radius` 等 Hybrid 专用键（2D 插件下无效，是残留） |
| C12 | 路径平滑 | **真正生效的平滑是 Smac 的 `smooth_path: true`**；`smoother_server` 虽启动但 BT 里没有 `<SmoothPath>` ⇒ **死代码** | [G26]/[GH26] `singlenav2_params.yaml` / `nav2_params.yaml`：`smoother_plugins: ["savitzky_golay_smoother"]`, `window_size 7, poly_order 3, refinement_num 2, enforce_path_inversion true`；BT XML 无 `SmoothPath`（两版 XML 内容一致） | 事实（配置与 BT 对照） | **认知项**：不要把 savgol 当成他们的收益来源；我们要么在自研 BT 里显式加 `<SmoothPath>`，要么只用 Smac `smooth_path` |
| C13 | 代价图几何与算力折中（单点/多点两套） | 单点（有场地图）：局部 10×10@0.05、全局 25×25@0.04 滚动窗、`always_send_full_costmap False`；多点（slam）：局部 10×10、全局 **50×50**、`always_send_full_costmap True` | [G26] <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/singlenav2_params.yaml> / <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/multiplenav2_params.yaml>；`update_frequency/publish_frequency 20.0` | 事实：多点模式没有先验图，需要更大滚动窗 | **需改造**：我们场地 15×28 m，**不要抄 50×50@0.04 + 全量发布**（每帧 ≈1250×1250） |
| C14 | 速度包线（"冲刺"理念 vs 实车收敛） | GitHub 季前：`vx/vy ±7.5 m/s`、`ax/ay ±5.0`、`controller_frequency 50`、`wz_max 2.5`、`prune_distance 1.7`、`reset_period 1.0`；**Gitee 2026 实跑降到**：`vx/vy ±2.5`、`wz ±1.5`、`ax/ay_max 1.5`、`ax_min -3.5`、`ay_min -3.5`、`az_max 1.0`（单点）/`3.0`（多点）、`reset_period 0.08`、`batch_size 1500`、`time_steps 80` | [GH26] <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/nav2_params.yaml>；[G26] 同参数文件 | **[BBS]** 开头「我的设计理念就一个：冲刺！」；但 Gitee 2026 的实际数值明显收敛（2.5 m/s） | **直接可用**：`±2.5 m/s` 正好落在我们「2.0 m/s 起标」的计划区间，可直接作为首版 A/B 值 |
| C15 | 车体自遮挡：**STVL 无法设最小障碍半径** | `pcl::CropBox` 负向裁剪车体 → `/livox/lidar_filtered` 只喂 3D 代价图层 | [G26] `cpp_lidar_filter/src/filter_node.cpp`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cpp_lidar_filter/src/filter_node.cpp>）；两套 launch 内联值均为 `x ±0.2 / y -0.2~0.4 / z -0.1~0.2 / negative True / leaf_size 0.05`；`box_lidar_filter` README：坐标「符合 REP 103…原点在 `base_link`」且注明「**降采样未启用**」 | **[BBS]/README**：「由于 `stvl` 无法显式设置最小障碍物半径，所以开发此包裁切源点云以避免将机器人自身注册成障碍物」（<https://github.com/qza36/box_lidar_filter>） | **直接可用但必须重标定**：源码对 `CropBox` **不做 TF**（作用在输入点云自身坐标系），而 marker 发在 `base_link` ⇒ README 的「base_link」与实现不符，照抄数值会错位。`leaf_size` 是死参数（`VoxelGrid` 整段注释） |
| C16 | 深度相机要不要接 | **真接**：2026 单点模式把 RealSense 加进 STVL 观测源，并按分辨率限定有效距离 | `observation_sources: livox_source realsense_source`；RealSense `obstacle_range 3.0 / raytrace_range 3.5 / min_z 0.15 / max_z 3.5 / min_obstacle_height 0.18 / max_obstacle_height 1.5 / model_type 0`；驱动 `depth_module.depth_profile: 424x240x90`「最高帧率」+ spatial/temporal filter + `pointcloud.ordered_pc: false` | 注释：「424x240 低分辨率有效探测距离」「低分辨率超过 3.5 m 精度差」「哨兵高度以上不关心」 | **不适用**（我们无深度相机）；但"**按传感器分辨率设定有效探测距离**"这条思路可用 |
| C17 | 上位机速度调不动？ | **先确认下位机能正确把控制器速度转成轮速**，再谈上位机提速 | **[BBS]** 图注：「蓝色是 lio 算出的车体实际速度，绿色是 cmd_vel 的期望速度，可见，这是明显的没有正确转换」；链路 `cmd_vel → velocity_smoother(30 Hz, OPEN_LOOP, deadband 0.05) → fake_vel_transform → /aft_cmd_vel → cod_serial_ul26`；帧 `0xA5`+3×float+checksum=15 B | **[BBS]**「NOTE：一定要注意的是，不能一味去调上位机的速度，先要确保电控那边能正确把控制器的速度转换成轮速」 | **直接可用（契约级）**：对应我们的 [`docs/sim_real_contract.md``sim_real_contract.md`；建议在 bench 里加一条「cmd_vel → 轮速」的一致性检查 |
| C18 | 姿态控制归属（哨兵自转） | **把朝向从 Nav2 里摘出去**：伪底盘 `base_link_fake` + `fake_vel_transform` 用 `spin_speed` 无条件覆盖 `angular.z`；容差 `yaw_goal_tolerance 6.28` ⇒ 只控位置 | [G26]/[GH26] `fake_vel_transform/src/fake_vel_transform.cpp`（`aft_tf_vel.angular.z = spin_speed_;`，默认 `0.0`，**仓库内无任何文件设非零**）；`base_link_fake` 出现在两个 costmap / bt_navigator / smoother / behavior server | README：「`base_link_fake` 的 yaw 固定指向正前方，避免云台自旋时 Nav2 局部规划器被朝向变化带偏」；**[BBS]** 另说 `fake yaw 也是之前抄的北极熊 24 年开源` | **部分可用**：我们已修成「`spin_speed==0` 直通」，**语义与他们不同**（他们恒为 0 ⇒ MPPI 的 `wz` 被丢弃）。若要抄"只控位置"，必须同批放宽 `yaw_goal_tolerance` |
| C19 | 建图与定位：要不要重定位 | **不要**：静态 `map→odom`（`z=0.05`）+ 纯 LIO；`slam_toolbox` 只做 lifelong 建图且 `transform_publish_period 0.0`（不发布 `map→odom`） | [G26] `singlenav_launch.py` 静态 TF **yaw=`-0.5`**（非 0）；`localization_launch.py` `lifecycle_nodes=['map_server']`；`mapper_params_online_async.yaml` `mode: lifelong`、`transform_publish_period 0.0` | **[BBS]**「只要提供了 map->odom->baselink 的 tf…nav2 就可以正常运行」「**由于联盟赛的环境较为简单且面积较小，定位的误差容忍度很大，所以采用了纯 lio 的定位方法**」；[G26] README：「**Keep It Simple Stupid**…发布坐标系静态转换比运用 urdf 维护更简单易操作」 | **不适用（作为默认）**：静态桥要求每次从同一位姿起飞、先验图同原点采集；我们 4 种重定位是资产（[`docs/cod_nav_2026_deep_dive.md``cod_nav_2026_deep_dive.md` §D1）。**但可作纯在线建图模式的临时开关** |
| C20 | 在线建图参数（slam_toolbox） | 提高更新/键帧密度，把 laser range 提到 10 m | [G26] `map_update_interval 1.0`、`minimum_time_interval 0.3`、`minimum_travel_heading 0.1`、`max_laser_range 10.0`、`min_laser_range 0.2`（[GH26] 分别为 2.5 / 0.5 / 0.5 / 5.0 / 0.01） | 事实（两版参数差异） | **需改造**：我们 `mapper:=slam_toolbox` 已有；这些值可作 `slam_toolbox_lifelong` 新槽起点 |
| C21 | 比赛阶段切换怎么做 | 决策 BT（XML 加载）分两类：**多点决策**（适应性训练建图用）与**单点决策**（基于场地图，更快）；巡逻用 **Topic 发 `/goal_pose`** 取代嵌套 Action | `COD_Behavior/cod_bt/{singlenav,multiplenav}_tree.xml`（<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/tree/feature/finish/COD_Behavior/cod_bt>）；`PubNav2Goal` 方案文档 | **[BBS]**「FollowWaypointsAction 偶尔会导致行为树死掉」；文档：「原方案链路 BT → waypoint_follower → bt_navigator（双层 Action，cancel/重发易出问题）；新方案 BT → /goal_pose Topic → bt_navigator（无状态，简单可靠）」 | **需改造**（我们任务层不同）；**值得我们引用的正是"双层 Action 易死"这条工程结论** |
| C22 | 哨兵"急停"配合自瞄 | 血量健康 + 在增益区 + 敌方装甲板距离 < 阈值 → **电控强制速度置零**（静止时自瞄命中率更高） | **[BBS]**（策略描述）；实现落在下位机 `Auto/Chassis_Auto` 分支与视觉判定（[CTL] README「导航模式」段 <https://gitee.com/cod_-control/rmcod2026_-sentry>） | **[BBS]**「由于自瞄在车自身静止时命中率相对较高，所以我们想出了一套『急停』策略」 | **需改造**（我们仿真无自瞄）；可作任务层的一个"停住增益"原语 |
| C23 | 决策阈值/功能触发 | 开局占中心增益区；我方占 → 前压，否则 → 中心区巡逻；**HP<210 回家、>350 出去**；PH<30% 补血；被弹丸攻击 → 加速小陀螺；热量/功率到上限 → 停射 | **[BBS]**（210/350）；`COD_Behavior/README.md`（<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/README.md>）：`1 号站位`=哨兵 PH>50% 且我方英雄/步兵占增益区、`2 号站位`=我方英雄与步兵都低血、`补血`=PH<30%、`占领`=开局或敌方占增益区、`识别`=0x0101 且 1→3 视为我方 | **[BBS]** + BT 文档 | **需改造**；注意 **[BBS] 的 210/350 与 BT 文档的百分比阈值不是同一套口径**（两处并列，不要混用） |
| C24 | 下位机：功率/热量/超电/小陀螺（影响可达速度） | 功率：西交模型 + **RLS 递归最小二乘在线拟合参数**，舵向优先 80% 功率；超电：电量 <30% 自动关闭；小陀螺：低速给超电充电、高速降低被瞬秒风险并干扰对方卡尔曼；热量：>100 按 19 弹频打、100→20 线性减速、<=30 停 | **[CTL]** <https://gitee.com/cod_-control/rmcod2026_-sentry>：`RLS_Init(&RLS_Power_Info, 4, 1, 0.99999, 1e-5)`、`W[0..3]=1.453e-07/1.23e-07/1.453e-07/1.23e-07`、`Err_Lower/Upper = 0.01/50`（舵向）与 `0.001/500`（行进）；超电 `Persent>=30 → Power_Max = limit - PID_Buffer.Output + 100`；导航模式 `if(HP>280 && Vision_Grap && Area_Status && Fire_Switch_Spin){Vx=Vy=5}`，`Vw` 每 1200 tick 循环 `6500/4000/2000`，否则 `6000`；热量 `SYS_Qres>100 → Shoot_DP=19`、`>20 && <100 → Shoot_Speed=(d*m-a-3d)/(d*T/100)+a/d`（`m<50`）/`(d*m-a-7d)/…`（`m>=50`）、`<=30 → 0` | **[BBS]** 与 [CTL] README 原文（含功率分配「隶属度」、求根公式解限幅） | **大多不适用**（我们无下位机仿真）；**唯一该抄的是契约**：上位机速度语义必须与下位机轮速换算对齐（见 C17） |
| C25 | 快速迭代：不想每次都起 Gazebo | 把 Nav2 `loopback_sim` 回移到 **Humble**，pixi/RoboStack 一键部署（MacBook Air 验证）；不启 Gazebo 就能跑决策/控制 | [COD_loopback_sim](https://github.com/qza36/COD_loopback_sim)：`nav2_loopback_sim/launch/loopback_simulation.launch.py`、`nav2_loopback_sim/params/nav2_params.yaml`、`maps/{depot,warehouse,tb3_sandbox}.pgm`；配套视频 <https://www.bilibili.com/video/BV1r79cBME6Y/> | README：「Using Gazebo for initial path planning and control testing can be unnecessarily cumbersome. It forces developers to contend with physics-engine artifacts—such as odometry drift and positioning errors—which are distractions at this stage.」 | **直接可用**：正好补我们"必须起 Gazebo"的短板，可做 MPPI/wrapper 的快速 A/B |
| C26 | 他们对"下一步方向"的判断 | nav2 对 RM 哨兵局限太大，认为**无人机式快速探索导航框架**更适合；随后 fork/自建 3D 导航栈 | **[BBS]**「nav2 对于 rm 的哨兵来说局限性还是太大了，个人认为，像无人机的那种快速探索导航框架才是最适合 RM 的」；对应仓库 `pct_planner_ros2_cpu`(<https://github.com/qza36/pct_planner_ros2_cpu>)、`ROG-Map-ROS2`、`far_planner`、`litenav`、`map_module`/`planning_module` | 作者自述 | **不适用（当期）**；作为路线储备（我们不必跟随） |
| C27 | 未知区 / 动态障碍 / ESDF / 前沿探索 | **都没做**（2026 唯一的"时间维"就是 STVL 的 0.5 s linear decay；`esdf`/`frontier` 在 `rmul2026` 全分支 grep 0 命中） | [GH26]（见 [`docs/cod_nav_2026_deep_dive.md``cod_nav_2026_deep_dive.md` §A15） | 事实 | **别去那儿找答案**：这条修正我们"他们更全"的预期 |

---

## D. 建议的最小拉取集合

> 目标：**只要 4 个来源、12 个文件**就能覆盖 2026 导航的全部"可落地增量"。全部为只读拉取，不编译不运行。

1. **`gitee.com/codnavgation/cod_-rm2026_-navigation` @`master`（163 文件，11 MB 级）** —— 2026 成品导航，优先级最高。
   第一步读：① `README.md`（10 包结构 + 6 组 CSV 语义 + 「KISS / 静态 TF」设计声明）② `src/cod_bringup/params/singlenav2_params.yaml`（单点全套参数）③ `src/goal_approach_controller/src/goal_approach_controller.cpp`（wrapper 实现）④ `src/cod_bringup/launch/singlenav_launch.py`（静态 TF yaw=-0.5 / 裁剪盒值 / RealSense）⑤ `src/cod_bringup/wps/go_gain.csv` + `src/cod_bringup/maps/rmul2026.yaml`（航点格式与场地图）。
   为什么：wrapper、两套 params、场地图、CSV 只在这里。
2. **`gitee.com/codnavgation/cod_-rm2026_-behavior-tree` @`feature/finish`** —— 决策/巡逻与"双层 Action 易死"的工程结论。
   第一步读：① `COD_Behavior/README.md`（触发条件表）② `COD_Behavior/doc/PubNav2Goal_Patrol.md`（Topic vs Action + 5 个节点的端口表）③ `COD_Behavior/cod_bt/singlenav_tree.xml` ④ `COD_Serial/README.md`。
3. **`github.com/qza36/COD_loopback_sim` @`master`** —— 不起 Gazebo 的快速验证通道。
   第一步读：① `README.md`（动机 + `pixi` 用法）② `nav2_loopback_sim/launch/loopback_simulation.launch.py` ③ `nav2_loopback_sim/params/nav2_params.yaml`（对照它默认给的控制/代价图配置）。
4. **`github.com/qza36/waypoint_editor`（分支 `feature/adaptToNav2`）或 Gitee 仓内副本 `src/waypoint_editor`** —— 航点工作流（含 `waypoint_through_nav2` 源码）。
   第一步读：① `USAGE_WITH_NAV2.zh.md` ② `launch/waypoint_through_nav2.launch.py` ③ `src/waypoint_through_nav2.cpp` ④ `data/sample_wp.csv`。
5. **（可选、低成本）`github.com/qza36/box_lidar_filter`** —— 只有 4 个文件；读 `README.md` + `src/filter_node.cpp`，作为 `cpp_lidar_filter` 的等价最小实现（Gitee 那份是同一份代码的更晚版本）。
6. **（历史对照，稀疏拉取）`gitee.com/ustl-cod/cod_nav`**
   `@dyx-dev` 只取 `nav_bringup/launch/bringup.launch.py` + `nav_bringup/params/nav2_params.yaml`（看 `terrain_analysis`/`intensity_voxel_layer` 是怎么被搁置的）；`@master` 只取 `nav_bringup/map/*.yaml` + `test_data/rmul_2025.yaml`（场地资产）。
7. **（接口契约，只读 README）`gitee.com/cod_-control/rmcod2026_-sentry`** —— 只读 README 的「功率控制 / 导航模式 / 哨兵自动开关超电」三节，用来核对 `cmd_vel` 语义与可达速度。
8. **（可选）`github.com/qza36/rmu_gazebo_simulator`（`jazzy`）** —— 只为 `rmul_2025/rmuc_2026` 世界模型与"离线导出 PCD"工具；**注意分支是 `jazzy`，与我们 Humble bench 不通用**，建议只取 world/PCD 资产。

不必要拉的：`COD_NAV_NEXT`（无 README、用途不明）、`sentry_planning`（他队 fork）、研究型 fork 群（本次未读、当期不用）、`gridmap`/`COD_PCD2PGM`（2.5D 路线，与我们当前 linefit/STVL 路线不冲突但非急需）。

---

## E. 未获取到的（明确列出，不含推测）

1. **`terrain_analysis` 的实际源码与来源**：在 `ustl-cod/cod_nav` 的 `dyx-dev` 里只是一个 **gitlink（`160000 commit c82f23c…`）**，而该分支 `.gitmodules` **没有对应条目** ⇒ 无法从该仓库取得内容；`pb_nav2_plugins` README 推荐的是北极熊的 [`terrain_analysis`](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis)，但**COD 用的是否就是这一份，本次未确证**。
2. **`terrain_analysis` 在 COD 里的实际启用状态**：`dyx-dev` 的 `bringup.launch.py` 里 `IncludeLaunchDescription(terrain_analysis_launch.py)` 是**被注释掉的**（<https://gitee.com/ustl-cod/cod_nav/blob/dyx-dev/nav_bringup/launch/bringup.launch.py>），2026 版本也没有该包 ⇒ 只能确认"存在指针 + 未接线"，不能确认历史上是否真跑过。
3. **BBS 帖里的图片内容**：帖中 6 张图/动图（含"提高速度的官方建议"截图、goal wrapper 代码截图、lio 速度 vs cmd_vel 对比图）**未做 OCR**，只记录了它们在文中的位置与图注，图片 URL 见 <https://bbs.robomaster.com/article/1882897>。
4. **B 站两集视频的内容**：无可用字幕接口，**未转写**（<https://www.bilibili.com/video/BV1XSXZBUEYL/>、<https://www.bilibili.com/video/BV1r79cBME6Y/>）；只知道 Gitee README 将其标为「该导航程序详细解说教程」。
5. **`small_point_lio` "可以输出速度"的那份修改版**：**[BBS]** 说「这是我修改过的 small point lio，可以输出速度」，但 Gitee 2026 `master` 与 GitHub `rmul2026` 的 `small_point_lio_node.cpp` **完全相同**，其中 `odometry_msg.twist.twist.*` 六行仍是**注释状态**（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/small_point_lio/src/small_point_lio_node.cpp>）⇒ **该修改版未公开在此仓库**，无法获取。
6. **GitHub `COD_NAV` 的 `ul_mppi`、`rmul2026_sim`、`feature-obstacle_escape_node` 三个分支**：本次未读（前序文档只记了提交日期）。
7. **`myblog` 的导航相关文章**：仓库存在（<https://github.com/qza36/myblog>、<https://qza36.github.io/myblog/>），**本次未逐篇核对**。
8. **`map_module` / `planning_module` / `litenav` / `CODSentryPAC` / `enemy_tracking` / `gridmap` / `COD_PCD2PGM` 的源码内部**：本次只读 README 与文件清单，未逐文件读实现。
9. **`COD_NAV_NEXT` 的用途**：无 README、无描述，`git ls-tree` 只见 `cnn_driver/cnn_perception/cnn_planner` 与 `livox_ros_driver2` vendor ⇒ **用途无法确证**（不推测）。
10. **`sentry-auto-aim` 的导航侧**：其 `使用方法.md` 明确写「**当前并未添加导航模块**」，`rm_bringup/launch/bringup_navigation.launch.py` 引用外部 `rm_navigation` 包，而该包**不在任何已确认的 COD 仓库里** ⇒ 无法获取其内容。
11. **`cod_-control/rmcod2026_-sentry` 的 PDF 文档**：`H7文档.pdf`、`gimbal/Document/Quaternion.pdf`、`原理图/`、`电机说明/` **未下载**（PDF 解析未做）。
12. **`ustl-cod/cod_sentry` 的三个子工作空间内容**：只读了 README（2024 方案），`rm_navigation_ws` / `rm_decision_ws` / `rm_vision_ws` 的 launch/params **未逐一读**。
13. **参数文档与代码不一致之处（已记录但未澄清原因）**：① [BBS] 说 `PathFollow/PathAlign` 权重不宜 >10，而 [G26] 单点代码是 15/15；② [BBS] 说热量控制「100 到 40 之间…40 到 30 之间…」，而 [CTL] 代码是 `>20 && <100` 线性区、`<=30` 停、`m<50`/`m>=50` 两种公式；③ [G26] 注释写「time_steps 80，预测时域 4.8 s」，但 `80 × model_dt 0.02 = 1.6 s`；④ `velocity_smoother.smoothing_frequency 30.0` 注释写「与 controller_frequency 一致」，而 `controller_frequency` 是 `50.0`。以上均为**事实性不一致**，原因未获取。
14. **`rmul2026.pgm` 的像素分布**：只读到 PGM header（`P5`、`255×177`、`255`）与 yaml（`origin [-1.756, -7.036, 0]`、`resolution 0.05`），**未核对占据/自由分布**。
15. **fork 群与 `rmu_gazebo_simulator` 的实际质量**：均**未深读**，本文件只按描述列出（不做评价）。
16. **GitHub 代码搜索未执行**（需鉴权，见 [rate_limit](https://api.github.com/rate_limit)）⇒「哪些第三方仓库抄了 `fake_vel_transform`/`cpp_lidar_filter`」仍无完整清单。


---


## 来源 18/19：`tdt/cod_nav_open_source_materials.md`


> 调研对象：辽宁科技大学 **COD 战队** RoboMaster 哨兵项目（2024–2026 赛季），主力仓库 <https://github.com/qza36/COD_NAV>（作者 齐志昂 / qza36）。
> 调研日期：**2026-10-04**。
> 调研方式：只读。GitHub REST API、`git ls-remote` / `--filter=blob:none` 裸克隆（可绕开 API 限流）、Gitee API v5、BBS(Nuxt SSR) 页面 payload 解析、bilibili 公开 API、web 检索。
> 本文件是本次调研**唯一写入**的文件，未改动本仓库其它任何文件。
> 局限：GitHub 未鉴权调用限额 60 次/小时（<https://api.github.com/rate_limit>），**代码搜索 API 需鉴权**，故 `search/code` 未执行；bilibili 空间归档 API 返回风控 `-799`，无法枚举 UP 主全部投稿。这些在 §F 明确列出。

---

## A. 结论：是不是「全部开源」？

**结论：COD_NAV 只是 COD 战队 2025 赛季导航上位机的其中一个仓库。战队在 GitHub（个人号 `qza36` + 队员 `GrassFanWang`）与 Gitee（3 个组织）上按「导航 / 决策 / 自瞄 / 下位机」分模块开源；2026 赛季这四个模块的仓库均已逐个核实存在（帖中自述「全开源」，见 <https://bbs.robomaster.com/article/1882897>）。但没有任何统一索引，且 **2026 赛季代码几乎全部在 Gitee 而非 GitHub**。**

已在公开渠道确认存在的内容（每条附证据链接）：

1. **2026 赛季四件套（全在 Gitee）**——BBS 开源帖标题即「导航、决策、自瞄、下位机控制全开源」：
   - 导航：<https://gitee.com/codnavgation/cod_-rm2026_-navigation>（15 ★，163 文件，仅 `master`）
   - 决策：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>（4 ★，9 分支，含 `COD_Behavior` + `COD_Serial` + `rm_interfaces`）
   - 自瞄：<https://gitee.com/ustl-cod/sentry-auto-aim>（含 `rm_bringup`）
   - 下位机：<https://gitee.com/cod_-control/rmcod2026_-sentry>（11 ★，857 文件）
   - 出处：<https://bbs.robomaster.com/article/1882897>
2. **2025 赛季（GitHub `qza36`）**：`COD_NAV`（导航，<https://github.com/qza36/COD_NAV>）、`COD_Behavior`（决策，<https://github.com/qza36/COD_Behavior>）、`ros2_simple_serial`（串口，<https://github.com/qza36/ros2_simple_serial>）、`Sentry_vision`（视觉，<https://github.com/qza36/Sentry_vision>），并由汇总工作空间 `Sentry_ws` 的 `.gitmodules` 指向这四个仓（<https://github.com/qza36/Sentry_ws/blob/master/.gitmodules>）。
3. **2024 赛季**：`ustl-cod/cod_sentry`（哨兵上位机 + 实现思路文档，<https://gitee.com/ustl-cod/cod_sentry>）、`ustl-cod/cod-2024-radar`（雷达站，29 ★，<https://gitee.com/ustl-cod/cod-2024-radar>），论坛帖 <https://bbs.robomaster.com/wiki/4577/9649>。
4. **其它子系统**：机械（<https://bbs.robomaster.com/article/1883384>）、电控 H7 通用控制系统（<https://bbs.robomaster.com/article/434199>）、工程机械（<https://bbs.robomaster.com/article/1883259>）、串联腿 MJCF/USD（<https://bbs.robomaster.com/article/1315669>）。
5. **教学/科普产出**：导航分享视频 2 集（<https://www.bilibili.com/video/BV1XSXZBUEYL/>、<https://www.bilibili.com/video/BV1r79cBME6Y/>）、新人知识库（<https://gitee.com/ustl-cod/cod_beginner>）、2024 算法组培训（<https://gitee.com/ustl-cod/2024-algorithm-group-training>）。

**确认「不在公开渠道」或只能间接获得的内容**（详见 §D）：

- `spatio_temporal_voxel_layer`、`slam_toolbox`、`nav2_*`、`patchwork-plusplus`、`pcd2pgm`、`pb_nav2_plugins`、`waypoint_editor` 上游：都是第三方，COD 仓里只留参数/子模块指针，不是 COD 自研开源。
- `livox_ros_driver2`：2026 赛季被从 COD_NAV 删除，CI 只用 `qza36` 的第三方 fork（<https://github.com/qza36/livox_driver2_ros2>），见 <https://github.com/qza36/COD_NAV/blob/rmul2026/CLAUDE.md>。
- **没有**找到 2025 赛季「导航」单独的论坛开源帖（2025 导航只在 GitHub，未在 BBS 单独立帖）；**没有** GitHub Wiki 页面（见 §F）。

---

## B. 资料清单表

### B1. 代码仓库 —— GitHub（个人号 `qza36`，共 74 个仓库，含 fork）

| 资料 | 链接 | 类型 | 内容摘要 | 对我们有用的部分 |
|---|---|---|---|---|
| COD_NAV（2025 哨兵导航） | <https://github.com/qza36/COD_NAV> | 代码 | 30 ★ / 3 fork / BSD-3-Clause，创建 2024-12-12，最后推送 2026-03-10；5 分支、0 issues、2 已关闭 PR、无 tag/release | 我们的主要对比对象（见 `docs/cod_nav_comparison.md`） |
| 分支 master | <https://github.com/qza36/COD_NAV/tree/master> | 代码 | 80 commits，2025-08-02；FAST-LIO fork + patchwork++ + small_gicp 重定位 + pcd2pgm 先验图 + pb_omni_pid 控制器 + rm_simulation（Gazebo UL24） | 唯一带 Gazebo 仿真的分支；`rm_simulation/pb_rm_simulation/world/TestWorld/` |
| 分支 rmul2026 | <https://github.com/qza36/COD_NAV/tree/rmul2026> | 代码 | 116 commits，2026-03-10；small_point_lio + slam_toolbox lifelong + cpp_lidar_filter + MPPI Omni + 双 STVL；含 `CLAUDE.md` 架构文档 | 参数配方来源（MPPI/STVL/裁剪盒） |
| 分支 ul_mppi | <https://github.com/qza36/COD_NAV/tree/ul_mppi> | 代码 | 115 commits，2026-03-10，与 rmul2026 同源（"increase min pcl"） | 参数微调对照 |
| 分支 rmul2026_sim | <https://github.com/qza36/COD_NAV/tree/rmul2026_sim> | 代码 | 112 commits，2026-01-04，`fake_vel_transform` 适配仿真 | 仿真→实车改法 |
| 分支 feature-obstacle_escape_node | <https://github.com/qza36/COD_NAV/tree/feature-obstacle_escape_node> | 代码 | 69 commits，2025-05-07，新增 `obstacle_escape` 包 | 脱困实现（我们尚未做） |
| COD_Behavior（2025 决策） | <https://github.com/qza36/COD_Behavior> | 代码 | 5 ★；`cod_bt/t1.xml`、`src/sentry_behavior.cpp`、`src/CheckNavResult.cpp`、`include/cod_behavior/sentry_behavior_actions.hpp` | **缺失模块 `cod_behavior` 的公开版本** |
| ros2_simple_serial（串口） | <https://github.com/qza36/ros2_simple_serial> | 代码 | 1 ★；CMake target/IDE 工程名为 `cod_serial`（`.idea/cod_serial.iml`）；订阅 `/cmd_vel` → 串口 `/dev/ACM0`；README 自述「下位机连发导致串口卡死」已知问题 | **缺失模块 `cod_serial` 的公开版本** |
| Sentry_ws（汇总工作空间） | <https://github.com/qza36/Sentry_ws> | 代码/文档 | 仅 8 文件；`.gitmodules` 串起 COD_Behavior / COD_NAV / Sentry_vision / ros2_simple_serial，README 一行「COD-哨兵工作空间」 | 一次性拉齐全栈的入口 |
| Sentry_vision（2025 视觉） | <https://github.com/qza36/Sentry_vision> | 代码 | 201 文件；FYT2024 视觉框架改；**含 `rm_bringup/`**、`rm_auto_aim/`、`fyt修改rv记录.md` | **缺失模块 `rm_bringup` 的公开版本（视觉侧）** |
| COD_auto_aim（2024 自瞄） | <https://github.com/qza36/COD_auto_aim> | 代码 | 2024-03，AngleSolver/ArmorDetector/Buffdetect 等传统自瞄源码 | 历史版本参考 |
| COD_loopback_sim | <https://github.com/qza36/COD_loopback_sim> | 代码/文档 | 7 ★；把 Nav2 `loopback_sim` 回移到 Humble，RoboStack+pixi 一键部署，可在 macOS 上跑；README 解释「为何不用 Gazebo 先验验证控制」 | **仿真 bench 可直接借鉴**（我们目前依赖 Gazebo） |
| waypoint_editor（fork） | <https://github.com/qza36/waypoint_editor> | 代码/文档 | 2 ★，`feature/adaptToNav2` 分支有 `USAGE_WITH_NAV2.zh.md` | 多点航点编辑 + CSV 导出 |
| box_lidar_filter | <https://github.com/qza36/box_lidar_filter> | 代码/文档 | `pcl::CropBox` 挖掉车体点云；README 明确写「因 STVL 无法设置最小障碍半径」而开发，给出一组 `min/max_x/y/z` | **3D→2D/代价图近距遮挡的直接对策** |
| pointcloud_fusion | <https://github.com/qza36/pointcloud_fusion> | 代码 | 多雷达点云 TF 对齐 + 近似时间同步融合 | 多传感器方案储备 |
| map_module | <https://github.com/qza36/map_module> | 代码 | 2D map module，部分改写自 `nav2_map_server` | 地图模块最小实现参考 |
| planning_module | <https://github.com/qza36/planning_module> | 代码 | 2D plan module，与 `map_module` 配套 | 规划模块最小实现参考 |
| litenav | <https://github.com/qza36/litenav> | 代码 | 2026-05；`app_module/core_types/litenav_ros2/map_module/plan_module`，pixi + robostack，含 `.codex` | 作者新一代自研导航（脱离 Nav2 的雏形） |
| enemy_tracking | <https://github.com/qza36/enemy_tracking> | 代码/文档 | 敌人追踪导航包：订阅 `rm_interfaces/msg/Target`，在敌人周围生成候选攻击点→costmap 过滤→选最近可达点→调 Nav2；参考 PolarBear `pb2025_sentry_behavior` | 哨兵「追击/卡位」任务层可迁移 |
| CODSentryPAC | <https://github.com/qza36/CODSentryPAC> | 代码/文档 | 2026-04；轨迹生成系统（ReplanFSM 状态机 + planner_manager，面向低速差速履带） | 与 Nav2 不同的自研规划架构 |
| COD_NAV_NEXT | <https://github.com/qza36/COD_NAV_NEXT> | 代码 | 2025-09；顶层 `cnn_driver/cnn_perception/cnn_planner`（无 README） | 早期「下一代」尝试 |
| COD_PCD2PGM | <https://github.com/qza36/COD_PCD2PGM> | 代码/文档 | pcd → 2.5D 高程栅格并保存地图；pixi 管理 | 2.5D 高程图路线 |
| livox_converter | <https://github.com/qza36/livox_converter> | 代码/文档 | Livox CustomMsg → `PointCloud2`（保留 intensity/tag/line） | 若不用 livox 官方驱动作转换 |
| livox_driver2_ros2 | <https://github.com/qza36/livox_driver2_ros2> | 代码 | 2026 赛季 CI 用的 Livox 驱动 fork（原版被从 COD_NAV 删除） | 驱动版本对齐 |
| rm_interfaces | <https://github.com/qza36/rm_interfaces> | 代码 | 自定义 msg/srv 包 | 与决策/自瞄接口对齐 |
| myblog（个人博客） | <https://github.com/qza36/myblog> / <https://qza36.github.io/myblog/> | 文档 | 作者个人主页 + 1 篇技术文（2026/01/14 CLion + micro-ROS）；自述「architected my RoboMaster team's full navigation-and-decision stack」 | 作者视角的项目定位 |
| Sentry_ws 子模块之外的历史仓 | <https://github.com/qza36?tab=repositories> | 索引 | 另有 `dataPreprocess`、`Color_TXT`、`gridmap`、`aloam-ros2`、`AStar_Matlab`、`PersonWOWPlugins` 等非哨兵仓 | 工具类可复用 |

### B2. 代码仓库 —— Gitee（3 个组织）

| 资料 | 链接 | 类型 | 内容摘要 | 对我们有用的部分 |
|---|---|---|---|---|
| 组织 ustl-cod（算法组，13 仓） | <https://gitee.com/ustl-cod> | 索引 | COD 算法组组织主页 | 找历史方案的总入口 |
| cod_sentry（2024 哨兵上位机） | <https://gitee.com/ustl-cod/cod_sentry> | 代码+文档 | 2 ★，771 文件，`master`/`自瞄` 分支；含 `rm_decision_ws`（BehaviorTree.ROS2 + rm_behavior_tree + rm_decision_interfaces）与 `rm_navigation_ws`（FAST_LIO/point_lio/rm_driver/rm_navigation），README 即《2024 赛季哨兵上位机方案》 | **设计文档级**：三模块分工与 2024 架构 |
| cod-2024-radar | <https://gitee.com/ustl-cod/cod-2024-radar> | 代码 | 29 ★，无激光雷达/相机的纯运算端雷达站 | 历史 |
| cod_beginner | <https://gitee.com/ustl-cod/cod_beginner> | 文档 | 12 ★；`Linux/`、`Docker/`、`git/git.md`、`SLAM/ros2_setup_notes.pdf`、C++/Python/OpenCV 教程 | 新人上手材料 |
| 2024-algorithm-group-training | <https://gitee.com/ustl-cod/2024-algorithm-group-training> | 文档 | 15 ★；培训计划/大纲（FlowUs 外链）+ 培训作业 | 培训体系参考 |
| sentry-auto-aim（2026 自瞄） | <https://gitee.com/ustl-cod/sentry-auto-aim> | 代码+文档 | 1 ★；9 分支（`tmp`、`sentry_sp_version`、`feature/YOLO-Detection`…）；含 `rm_bringup/`、`rm_auto_aim/`、`rm_hardware_driver/`、`rm_rune/`、`rm_upstart/`、`使用方法.md`、`OPTIMIZATION.md` | **`rm_bringup` + 上下位机 16 字节帧协议** |
| cod_nav（Gitee 镜像 fork） | <https://gitee.com/ustl-cod/cod_nav> | 代码 | fork=True；**比 GitHub 多 `dev`、`dyx-dev` 两个分支**（GitHub 上已无），`dyx-dev` 含 `terrain_analysis/`，`master` 含 `codmap.pgm/yaml`、`nav_bringup/map/rmuc_2024.*`、`rmul_2024.*`、`test*.pgm`、`test_data`；其 `rmul2026` 分支是另一条中间路线（FAST_LIO_ROS2 + loam_interface + sensor_scan_generation） | **GitHub 上已看不到的历史分支与场地地图文件** |
| 组织 codnavgation（2026 导航/决策组，4 仓） | <https://gitee.com/codnavgation> | 索引 | 2026 赛季主组织 | 2026 代码入口 |
| cod_-rm2026_-navigation | <https://gitee.com/codnavgation/cod_-rm2026_-navigation> | 代码+文档 | 15 ★，163 文件，仅 master；`cod_bringup`（launch/params/maps/wps CSV/bt xml）、`cpp_lidar_filter`、`fake_vel_transform`、**`goal_approach_controller`**、`pb_nav2_plugins`、`pb_omni_pid_pursuit_controller`、`pointcloud_to_laserscan`、**`ros2_simple_serial`**、`small_point_lio`、`waypoint_editor` | 2026 导航最新版；CSV 航点（去增益/回家/巡逻）+ 目标点附近限速 wrapper |
| cod_-rm2026_-behavior-tree | <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree> | 代码+文档 | 4 ★，9 分支；`COD_Behavior`、**`COD_Serial`**、`rm_interfaces`、`BehaviorTree.ROS2`、`Project.btproj` | 2026 决策 + 串口（见 §D） |
| cod_-rm2026_-perception | <https://gitee.com/codnavgation/cod_-rm2026_-perception> | 占位 | 3 文件，README 仍是 Gitee 模板占位 | 无实质内容 |
| cod_-rm2026_-simluation | <https://gitee.com/codnavgation/cod_-rm2026_-simluation> | 占位 | 2 文件，README 占位 | 无实质内容 |
| 组织 cod_-control（电控组，18 仓） | <https://gitee.com/cod_-control> | 索引 | 2026 赛季电控主组织 | 下位机入口 |
| rmcod2026_-sentry | <https://gitee.com/cod_-control/rmcod2026_-sentry> | 代码+文档 | 11 ★，`dev`/`master`/`CI`；857 文件，含 `chassis/`、`gimbal/`（含 `gimbal/Document/Quaternion.pdf`）、`H7文档.pdf`、`原理图/`、`电机说明/`；README 详述舵轮解算公式、云台/底盘 CAN 通信、功率与热量控制 | **上下位机接口与底盘运动学的权威出处** |
| sentry_steering_25 | <https://gitee.com/cod_-control/sentry_steering_25> | 代码 | 2025 舵轮哨兵电控 | 与 COD_NAV(master) 同赛季 |
| cod_-h7_-template / COD_F4_Template / hero_2025 / rmcod2026_-infantry_-balance 等 | <https://gitee.com/cod_-control/cod_-h7_-template> | 代码 | H7 通用控制模板（2026-10-01 仍有推送）与其它兵种 | 电控底座 |

### B3. 论坛帖（bbs.robomaster.com）

| 资料 | 链接 | 类型 | 内容摘要 | 对我们有用的部分 |
|---|---|---|---|---|
| **RM2026 哨兵导航/决策/自瞄/下位机全开源**（作者 齐志昂，2026-03-31，15167 阅读） | <https://bbs.robomaster.com/article/1882897> | 帖子 | 全文含：建图方案（slam_toolbox + small_point_lio，纯 LIO 定位）、**Navigate Through Poses 取代单点导航**、waypoint_editor + CSV、STVL 取代 VoxelLayer 的理由、MPPI 调参经验（temperature/gamma、PathFollow/PathAlign 权重<10、阈值<1.0、GoalCritic 加权的「邪修」）、目标点附近圆周运动的 wrapper 解法、`PositionGoalChecker`、fake yaw/nav 树来自北极熊、决策逻辑（血量 210/350 阈值、开局占中心增益、急停策略）、下位机（超电、变速小陀螺、热量控制 19 弹频、功率控制 RLS 拟合）与各模块仓库链接 | **本次调研价值最高的一份**，等于一份答辩/技术报告 |
| RM2026 哨兵舵步机械结构开源 | <https://bbs.robomaster.com/article/1883384> | 帖子+附件 | 机械开源 zip + 《结构部分说明文档.pdf》 | 机械/整备信息 |
| **RM2025 STM32H7 电控通用控制系统开源**（作者 王草凡，2025-06-11，10253 阅读） | <https://bbs.robomaster.com/article/434199> | 帖子+附件 | 达妙 MC-02(STM32H723VGT6) 通用控制系统；CLion/Arm GNU 版；链接 GitHub/Gitee 双仓 + `README.pdf`；配套知乎 STM32H7 系列教程 | 下位机底座；电控文档 |
| RM2025 工程机器人机械结构开源 | <https://bbs.robomaster.com/article/1883259> | 帖子+附件 | 结构说明 PDF + 机械 zip | 旁支 |
| RM2026 串联腿闭链结构 MJCF&USD 开源 | <https://bbs.robomaster.com/article/1315669> | 帖子+代码 | <https://github.com/GrassFanWang/COD-2026RoboMaster-Balance-Simulation_File>（41 ★），IsaacSim5/MuJoCo 文件 + 视频 | 平衡/仿真资产 |
| RM2024 雷达站算法开源（算法开源专栏） | <https://bbs.robomaster.com/wiki/4577/9649> | 帖子 | 2024 雷达站算法 | 历史 |

### B4. 视频 / 社区

| 资料 | 链接 | 类型 | 内容摘要 | 对我们有用的部分 |
|---|---|---|---|---|
| RM COD 导航分享一（UP：昂哥救我，2026-03-31） | <https://www.bilibili.com/video/BV1XSXZBUEYL/> | 视频 | 2291 播放；Gitee 导航仓 README 明确推荐为「该导航程序详细解说教程」 | **导航方案讲解（对应 gitee 导航仓）** |
| RM COD 导航分享二：loopback sim 和 waypoint editor（2026-03-31） | <https://www.bilibili.com/video/BV1r79cBME6Y/> | 视频 | 1253 播放；讲 loopback 仿真与航点编辑器 | 仿真/航点工作流 |
| COD26 联盟赛东北站季军哨兵集锦（UP：王草凡，2026-04-01） | <https://www.bilibili.com/video/BV1759KB1EXK/> | 视频 | 3127 播放；简介直接指向 BBS 1882897 | 实车表现参考 |
| 串联腿仿真演示 | <https://www.bilibili.com/video/BV1Z5mGB7ESd/>、<https://www.bilibili.com/video/BV1CLm8B6E82/> | 视频 | 出处 <https://bbs.robomaster.com/article/1315669> | 旁支 |
| bilibili 镜像站（同视频） | <https://www.snm0516.aisee.tv/video/BV1759KB1EXK/> | 视频 | 第三方镜像 | 备用 |

---

## C. 与 COD_NAV 仓库的关系

### C1. 同一战队的其它模块/仓库（**不是**外部项目）

| 模块名（COD_NAV 里被引用的） | 是否存在公开版本 | 证据 |
|---|---|---|
| `cod_behavior` | ✅ 有，两代 | 2025：<https://github.com/qza36/COD_Behavior>；2026：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>（`COD_Behavior/`） |
| `cod_serial` | ✅ 有 | <https://github.com/qza36/ros2_simple_serial>（IDE 工程名 `cod_serial.iml`、旧 CMake target `cod_serial`） |
| `cod_serial_ul26` | ✅ 有 | Gitee 导航仓 `src/ros2_simple_serial/CMakeLists.txt` 里 `project(cod_serial_ul26)`、`package.xml` `<name>cod_serial_ul26</name>`，并被 `cod_bringup/launch/multiplenav_launch.py`、`singlenav_launch.py` 启动：<https://gitee.com/codnavgation/cod_-rm2026_-navigation> |
| `rm_bringup` | ✅ 有，但属**视觉/自瞄栈** | <https://gitee.com/ustl-cod/sentry-auto-aim>（`rm_bringup/launch/bringup.launch.py`、`bringup_navigation.launch.py`、`config/node_params/*`）；<https://github.com/qza36/Sentry_vision> 同样含 `rm_bringup/`。导航侧的启动包名是 `nav_bringup`（COD_NAV）/ `cod_bringup`（Gitee 2026） |
| `rm_interfaces` | ✅ 有 | <https://gitee.com/ustl-cod/cod_sentry>、<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>（`rm_interfaces/`）、<https://github.com/qza36/rm_interfaces> |
| 汇总关系 | `Sentry_ws` 用 submodule 明确指向 COD_Behavior / COD_NAV / Sentry_vision / ros2_simple_serial | <https://github.com/qza36/Sentry_ws/blob/master/.gitmodules> |

**2025 → 2026 的迁移关系**（同战队内）：2025 导航 = GitHub `COD_NAV`（FAST-LIO 系）；2026 导航 = Gitee `cod_-rm2026_-navigation`（small_point_lio + slam_toolbox + MPPI，LIO 换成 ACE 战队的 small_point_lio，作者在帖中致谢 ACE 开源：<https://bbs.robomaster.com/article/1882897>）。两者不是同一份代码，**2026 版本不在 COD_NAV 任何分支里**（COD_NAV 5 个分支成员来自 `git ls-remote`：master / rmul2026 / ul_mppi / rmul2026_sim / feature-obstacle_escape_node）。

### C2. **其它战队**的项目（明确区分，别混淆）

| 项目 | 链接 | 归属 |
|---|---|---|
| pb2025_sentry_nav（440 ★） | <https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav> | 深圳北理莫斯科大学 北极熊战队 —— **本项目（HzMi_rmsimulation）的上游**，COD 明确「抄」了其导航树/脱困插件/fake yaw（<https://bbs.robomaster.com/article/1882897>） |
| pb_omni_pid_pursuit_controller（18 ★） | <https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller> | 北极熊；COD_NAV 里是 `qza36` 的 fork 子模块 |
| pb_nav2_plugins | <https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins> | 北极熊；COD_NAV `master` 的 `.gitmodules` 直接指向它 |
| sentry_navigation（65 ★） | <https://github.com/IRobot-Algorithm/sentry_navigation> | 西安电子科技大学 IRobot 2024 哨兵导航 |
| SCURM_SentryNavigation（378 ★） | <https://github.com/PolarisXQ/SCURM_SentryNavigation> | 其它战队（非 COD） |
| PnX-HKUSTGZ/sentry-navigation（3 ★，fork） | <https://github.com/PnX-HKUSTGZ/sentry-navigation> | 港科大广州；Mid360 + FASTLIO 仿真 |
| laohao78/ROS2_RM_Navigation（36 ★） | <https://github.com/laohao78/ROS2_RM_Navigation> | 非 COD；RM 场地 Nav2 全流程 |
| small_point_lio（121 ★） | <https://github.com/Yancey2023/small_point_lio> | ACE 战队；被 COD 2026 采用并改速度输出（<https://bbs.robomaster.com/article/1882897>） |
| spatio_temporal_voxel_layer | <https://github.com/SteveMacenski/spatio_temporal_voxel_layer> | 第三方（Nav2 社区） |
| patchwork-plusplus | <https://github.com/url-kaist/patchwork-plusplus> | 第三方；COD_NAV `master` 子模块 |
| pcd2pgm | <https://github.com/LihanChen2004/pcd2pgm> | 第三方（北极熊 Lihan Chen）；COD_NAV `master` 子模块 |
| waypoint_editor 上游 | <https://github.com/kzm784/waypoint_editor> | 第三方；COD 用 fork <https://github.com/qza36/waypoint_editor> |
| COD-2026 平衡仿真文件 | <https://github.com/GrassFanWang/COD-2026RoboMaster-Balance-Simulation_File> | **COD 队员** GrassFanWang（王草凡）个人号 —— 算 COD 的 |
| COD-H7-Template / CLion 版 | <https://github.com/GrassFanWang/COD-H7-Template>、<https://github.com/GrassFanWang/COD_H7_Template_CLion>、<https://gitee.com/wangcaofan/cod-h7-template> | **COD 电控**（论坛帖 <https://bbs.robomaster.com/article/434199>） |
| 0xUwUi/fake_vel_transform | <https://github.com/0xUwUi/fake_vel_transform> | 第三方或他队同名实现（0 ★，无法确认归属，仅记录） |

> 另注：`fake_vel_transform` 这个标识符同时出现在 COD_NAV、其它战队的 fork 中；GitHub 代码搜索未执行（需鉴权），因此**没有**跨仓同名文件的完整清单。

---

## D. 缺失模块清单（COD_NAV 引用但仓库内没有）

| 模块 | 有无公开替代 / 搜到了什么 | 证据链接 |
|---|---|---|
| `cod_behavior` | ✅ **有**。2025 版：<https://github.com/qza36/COD_Behavior>（`cod_bt/t1.xml`、`sentry_behavior.cpp`、`CheckNavResult.cpp`）；2026 版：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree> 的 `COD_Behavior/`，文档 <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/README.md> | 同左 |
| `cod_serial` | ✅ **有**。<https://github.com/qza36/ros2_simple_serial>；2026 决策仓里另有一份 `COD_Serial/`（含 `include/message.h` 裁判系统结构体、`src/cod_serial.cpp`、`src/simple_serial.cpp`）：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree> | 同左 |
| `cod_serial_ul26` | ✅ **有**，即 Gitee 导航仓内的 `src/ros2_simple_serial`（package 名 `cod_serial_ul26`），被 2026 的两个 launch 调用 | <https://gitee.com/codnavgation/cod_-rm2026_-navigation> |
| `rm_bringup` | ✅ **有**，但在自瞄/视觉栈：<https://gitee.com/ustl-cod/sentry-auto-aim>（`rm_bringup/`，含 `bringup.launch.py`、`bringup_navigation.launch.py`、`config/node_params/*.yaml`）与 <https://github.com/qza36/Sentry_vision>。**导航**的等价物是 `nav_bringup`（COD_NAV）/ `cod_bringup`（Gitee 2026），不含 `rm_bringup` 同名包 | 同左 |
| `rm_interfaces` | ✅ **有** | <https://gitee.com/ustl-cod/cod_sentry>、<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>、<https://github.com/qza36/rm_interfaces> |
| `livox_ros_driver2` | ⚠️ 被 COD_NAV 2026 分支**移除**，CI 用第三方 fork | <https://github.com/qza36/livox_driver2_ros2>、<https://github.com/qza36/COD_NAV/blob/rmul2026/CLAUDE.md>（"External Dependencies (not in this repo)"） |
| `spatio_temporal_voxel_layer` | ⚠️ 上游第三方，COD 仓只留参数 | <https://github.com/SteveMacenski/spatio_temporal_voxel_layer> |
| `slam_toolbox` / `nav2_*` / `nav2_mppi_controller` | ⚠️ 上游 ROS 2 官方组件，无 COD 版本 | 无 COD 专属链接 |
| `fast_lio`（rmul2026 launch 里的死引用） | ⚠️ 该分支实际 LIO 是 `small_point_lio`，`fast_lio` 是残留引用 | <https://github.com/qza36/COD_NAV/blob/rmul2026/CLAUDE.md> |
| `terrain_analysis`（仅出现在 Gitee 老分支） | ✅ 在 Gitee 镜像 fork 的 `dyx-dev` 分支中可见 | <https://gitee.com/ustl-cod/cod_nav>（分支 `dyx-dev`） |

---

## E. 文档质量评估：哪些是「设计文档级」

按可读性与信息密度排序，最有价值的 5 份：

1. **BBS《RM2026-哨兵机器人导航、决策、自瞄、下位机控制全开源》** — <https://bbs.robomaster.com/article/1882897>
   最接近答辩材料：写清了方案取舍（为何抛弃 slam 单点导航改 Navigate Through Poses、为何用 STVL 而非 VoxelLayer）、MPPI 具体调参建议、目标点附近圆周运动的 wrapper 解法、决策逻辑阈值（血量 210/350）、下位机功率/热量控制策略与致谢参考来源。缺点：无公式化推导、无失败数据。
2. **COD_NAV `rmul2026` 分支 `CLAUDE.md`** — <https://github.com/qza36/COD_NAV/blob/rmul2026/CLAUDE.md>
   架构说明级：包职责表、数据流图、TF 树、关键参数文件与 MPPI 调参思路、"External Dependencies (not in this repo)" 清单，甚至记录了 `bt_navigator.odom_topic: "odomety"` 拼写 bug。
3. **《COD 哨兵决策说明文档（26 赛季 RMUL）》** — <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/README.md>
   逐条列出裁判系统消息（0x0001/0x0002/0x0101/0x0201/0x0203/0x0206…）与哨兵各功能的**触发条件**，附「未实现功能」章节；是接口级设计文档。
4. **Gitee `ustl-cod/cod_sentry` 的《2024 赛季哨兵上位机方案》** — <https://gitee.com/ustl-cod/cod_sentry/blob/master/README.md>
   讲清 2024 三模块（决策/导航/自瞄）分工、环境与硬件（Mid360、i7-1270P、海康 MV-CS016-10UC 8mm）、目录框架；可看清三年架构演进起点。
5. **《PubNav2Goal 巡逻方案使用文档》** — <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/doc/PubNav2Goal_Patrol.md>
   节点级方案文档：`LoadWaypoints/GetCurrentWaypoint/WaitUntilReached/WaitDuration/NextWaypoint` 端口表 + 行为树结构 + 「BT→waypoint_follower→bt_navigator 双层 Action 易出问题，改 BT→/goal_pose Topic」的动机说明。同类还有 `COD_Serial/README.md`。

补充（非导航但同样成体系）：`rmcod2026_-sentry/README.md` 给出舵轮解算公式与功率/热量控制实现（<https://gitee.com/cod_-control/rmcod2026_-sentry>）；`cod_beginner` 是新人知识库（<https://gitee.com/ustl-cod/cod_beginner>）；导航讲解视频 <https://www.bilibili.com/video/BV1XSXZBUEYL/>、<https://www.bilibili.com/video/BV1r79cBME6Y/>。

---

## F. 未能找到 / 无法访问（不编造）

1. **GitHub 代码搜索未执行**：`/search/code` 返回 403（需鉴权），见 <https://api.github.com/rate_limit> 与 <https://docs.github.com/rest/search/search>。因此「`fake_vel_transform` / `cpp_lidar_filter` / `small_point_lio` / `COD_NAV` / `pb_omni_pid_pursuit_controller` 被哪些第三方仓库复制」**没有**完整清单；仅通过仓库名搜索确认：`fake_vel_transform`→<https://github.com/0xUwUi/fake_vel_transform>、`small_point_lio`→<https://github.com/Yancey2023/small_point_lio>（121 ★，COD 采用的上游）、`pb_omni_pid_pursuit_controller`→<https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller>。`grep.app` 检索被 Vercel 人机校验拦截，未取到结果。
2. **GitHub Wiki 为空**：仓库元数据 `has_wiki: true`（<https://api.github.com/repos/qza36/COD_NAV>），但 <https://github.com/qza36/COD_NAV/wiki> 与 `/wiki/Home` 均 302/跳回仓库首页，且 `git ls-remote https://github.com/qza36/COD_NAV.wiki.git` 要求登录（即 wiki 仓不存在）⇒ **没有任何 wiki 页面**。
3. **Issues / PR / Release**：`issues?state=all` 返回 2 条，均为已关闭 PR（#1「稳定版本」2025-05-01、#2「Dyx dev」2025-07-17），无 issue、无 tag、无 release（`git ls-remote --tags` 为空）。URL：<https://github.com/qza36/COD_NAV/pulls?q=is%3Apr>。
4. **3 个 fork 无额外内容**：<https://github.com/ZeroHour-Z/COD_NAV>、<https://github.com/loml13/COD_NAV>、<https://github.com/U-Nori/COD_NAV>，`git ls-remote --heads` 均只有 `master`；fork 列表亦见 <https://api.github.com/repos/qza36/COD_NAV/forks>。
5. **bilibili 投稿无法全量枚举**：空间归档 API 返回 `code:-799 请求过于频繁`；仅确认 3 个视频（§B4）。UP「昂哥救我」是否还有「导航分享三」**无法核实**。
6. **知乎 / CSDN / 微信公众号**：未检索到 COD 关于导航的第三方分析文或官方公众号文章。BBS 434199 中出现的知乎链接（<https://zhuanlan.zhihu.com/p/714301640>、<https://zhuanlan.zhihu.com/p/720966722>、<https://zhuanlan.zhihu.com/p/4218673539>）是**电控 STM32H7 教程**，与导航无关。`gitee.com/zhang-jire/bisai`（<https://gitee.com/zhang-jire/bisai>）是队员个人仓库，未见导航文档。
7. **无 2025 赛季导航专属论坛帖**：站内检索（含 BBS「算法开源专栏」<https://bbs.robomaster.com/wiki/4577> 的完整目录、关键词检索）只找到 COD 的 2024 雷达（<https://bbs.robomaster.com/wiki/4577/9649>）、2025 电控（<https://bbs.robomaster.com/article/434199>）、2025 工程机械（<https://bbs.robomaster.com/article/1883259>）、2026 导航/决策/自瞄/下位机（<https://bbs.robomaster.com/article/1882897>）、2026 哨兵机械（<https://bbs.robomaster.com/article/1883384>）、2026 串联腿（<https://bbs.robomaster.com/article/1315669>）。**未发现** 2025 赛季哨兵导航的单独开源帖。
8. **占位仓库无内容**：<https://gitee.com/codnavgation/cod_-rm2026_-perception>、<https://gitee.com/codnavgation/cod_-rm2026_-simluation>、<https://gitee.com/ustl-cod/rm2026-cod_-radar> 的 README 仍是 Gitee 模板文本。
9. **无法核实的细节**：Gitee 部分仓库的许可证未在页面上确认；`https://github.com/qza36/COD_NAV_NEXT` 无 README、无描述，仅顶层 `cnn_driver/cnn_perception/cnn_planner`，用途只能从命名推断（**标注为推断**）。
10. **BBS 未开放检索/REST API**：`/developers-server/rest/article/...`、`/rest/search` 等推测端点均 404（实测 2026-10-04），站内检索只能靠搜索引擎与页面 payload 解析，故 §B3 的帖子清单可能不完整。

---

## 附：本次调研可直接落地的三条

1. **近距/自身点云**：`box_lidar_filter`（<https://github.com/qza36/box_lidar_filter>）给出「STVL 无法设最小障碍半径 ⇒ 用 CropBox 挖掉车体」的完整参数样例；2026 导航仓对应包名 `cpp_lidar_filter`。
2. **目标点附近震荡**：2026 方案用 `goal_approach_controller` 做限速 wrapper 而不是继续调 MPPI（<https://gitee.com/codnavgation/cod_-rm2026_-navigation>，帖子说明见 <https://bbs.robomaster.com/article/1882897>）。
3. **不启动 Gazebo 的先验验证**：`COD_loopback_sim`（<https://github.com/qza36/COD_loopback_sim>）+ `waypoint_editor`（<https://github.com/qza36/waypoint_editor>）组合，可先在 Humble 上把多航点逻辑跑通；配套视频 <https://www.bilibili.com/video/BV1r79cBME6Y/>。


---


## 来源 19/19：`tdt/cod_vs_ours_frame_architecture.md`

> **STATUS（2026-10，仓库整理；2026-10-18 实车实测后更新）**：本文 §2/§3.3「COD 的 `base_link ≡ livox_frame`（C 型，`T_bl≡I`）」的结论已被**实车实测**取代 —— 实测为 **A 型**（`T_bl` 非单位阵：roll `−46.238°`、`|t| 0.3949 m`；`odom→base_link` 静止时 roll ≈ `0.030°` ⇒ 共轭 bug 已生效）。**原始数据与判读见 [`cod_real_frame_measurement.md``cod_real_frame_measurement.md`**（亦见 `docs/README.md` 取代表 #11）；本文其余内容（H2/H3 源码对账、耦合清单）仍有效，但"COD 不受影响"的归宿要按 A 型重读。
# COD 实车（30° 斜装雷达）vs 本仓仿真：帧架构对账与"到底是谁的缺陷"

> **任务性质**：只读代码取证（`grep` / `read` / `git log`），**未运行任何仿真**，未修改
> `third_party/cod_nav_2026/**`、任何 launch/config/源码，也未改其它文档（本文件是本次唯一写入）。
> **本地 COD 源码**：`third_party/cod_nav_2026/`（Gitee `codnavgation/cod_-rm2026_-navigation@master`，
> 含 `.git/`，190 文件，5.4 MB；**不是 submodule**，全部可读）。下文 COD 行号一律指这份本地副本。
> **本文与既有分析的关系**：`cod_nav_nav_stack_inventory.md` / `cod_nav_2026_deep_dive.md` /
> `cod_nav_2026_params_review.md` / `cod_nav_design_thinking.md` / `cod_nav_comparison.md` 已被读；
> 本文**不重复**它们的参数搬运清单，只做"帧约定 + TF 契约 + 代价图供给"这一条纵切，
> 并**就地更正**其中两处与本地源码不符的记述（§3.1、§4.5）。
> **前置**：本仓侧的事实基线见 `docs/tilted_lidar_fidelity.md` §A/§I/§J/§N、`docs/tf_interface_contract.md`、
> `docs/robot_models.md`、`docs/architecture.md`。

---

## 0. 五条结论（先给答案）

| # | 结论 | 一句话证据 |
|---|---|---|
| **1** | **COD 的 `base_link` 就是雷达壳体系**（= `livox_frame`），**不是**重力对齐的车体系。他们不是"把 30° 吸收掉了"，而是**把 `base_link` 定义成斜的那个系**，于是 30° 在 TF 里**根本不存在**。 | `small_point_lio_node.cpp:75` 的共轭公式只在 `T(base_link←livox)=I` 时成立；上游 launch 声明的正是单位 extrinsic（`small_point_lio.launch.py:24-45`）——**且 COD 的 launch 从不发布这条边**（§2.1）。 |
| **2** | **H2 更正**：**COD 用的就是我们 vendored 的同一份 `small_point_lio`**（同一上游 `Yancey2023/small_point_lio@688d75c`）。此前"COD 用的是另一套 LIO（FAST-LIO）所以不受影响"的假设**是错的**。COD 不炸的原因**不是换了 LIO**，而是**换了 `base_link` 的定义**。 | `third_party/cod_nav_2026/src/small_point_lio/` 与 `src/localization/rm_localization/small_point_lio/` 同源；`docs/tilted_lidar_fidelity.md` §N.1 已给出 sha256 逐字节相同的取证。 |
| **3** | **H2 的技术判决**：COD **有** `odom→base_link` 的等价处理，但**不是** `lio_tf_adapter` 式的**合成**，而是写死在 LIO 节点里的**共轭（相似变换）**：`T_ob = T_bl⁻¹·T_ol·T_bl`（`:75`）。它在 `T_bl=I` 时**退化成恒等**，所以"正确"只是**约定带来的巧合**。我们的 `lio_tf_adapter` 反而是本仓唯一写对**合成**的实现（`lio_tf_adapter_node.cpp:66-72`），却被 `bringup_sim.launch.py:2028-2033` 明确排除。 | §3.3 / §3.5 |
| **4** | **H3 判决**：COD 的 nav2 **完全没有 `obstacle_layer`、完全没有 `/scan`**（两份 params 里 `obstacle_layer` 命中数 **0**，`/scan` 命中 **0**）。两张代价图都是 `["static_layer","stvl_voxel_layer","inflation_layer"]`，观测源是 **3D 点云** `/livox/lidar_filtered`。**所以"每束 z 由扫描原点 z 伪造"这条失效路径在 COD 里根本不存在** —— 不是"他们绕过了"，是"那条代码路径没被启用"。 | `singlenav2_params.yaml:192,200-263,281`；§4.1 |
| **5** | **架构判决**：我们的架构 **(b) 本身不残缺，但默认值/约定与 COD 分叉，并额外引入了若干可避免的耦合**（不是 (c) 真缺陷）。"斜装雷达"本身固有的破点只有 2 个；我们把它放大成 6 个，其中 4 个是我们自己造的。 | §6 / §7 |

---

## 1. 证据表：用户 5 问 → COD 证据 → 我们证据 → 判决

| Q | 问的是什么 | COD 文件:行 | 我们文件:行 | 判决 |
|---|---|---|---|---|
| **Q1** | `base_link` 该不该和雷达正方向对齐 | `small_point_lio_node.cpp:75`（共轭，仅 `T_bl=I` 时对）· `small_point_lio.launch.py:24-45`（单位 extrinsic 的声明，`:47` 未启用）· `filter_node.cpp:56`（marker 发在 `base_link`，而裁剪作用在输入点云系）· `multiplenav_launch.py:70`（p2l `target_frame: base_link` 而输入是 `/livox/lidar`） | `tf_interface_contract.md:44`（契约链 `…→base_link→livox_frame`）· `robot_models.md:173-174`（base_link 原点=底盘最低点，雷达 +0.80 m）· `tilted_lidar_fidelity.md:22/§A.1`（默认档 `base_link→livox_frame` rpy=0） | **COD 用"传感器壳体系"当 base_link（违反 REP-103 语义）；我们用"车体/重力系"。两种都自洽，代价落点不同 —— 见 §2.3。** |
| **Q2** | COD 跑 `small_point_lio` 时的 TF 处理是否正确/等价 | `small_point_lio_node.cpp:52-100`（**节点自己**发 `odom→base_link`，无外部 adapter）· `:75`（**共轭**）· `:127`+`:159`（`/cloud_registered` 自称 `odom` 却对已配准点再乘一次 `T(base_link←livox)`） | `bringup_sim.launch.py:2021-2026`（理由注释）· `:2028-2033`（`lio!=small_point_lio` 才启动 adapter）· `lio_tf_adapter_node.cpp:66-72`（**合成**：`q_lio*q_offset`、`p+R·t`） | **等价但不等价于"正确"**：COD 靠 `T_bl≡I` 让共轭退化成恒等；我们排除了唯一写对合成的节点，改用上游的共轭（已于 `2a5a8b6` 修为合成，属上游潜伏 bug，见 §N.1）。 |
| **Q3** | COD nav2 用哪些帧、代价图吃什么、有没有高度带 | `singlenav2_params.yaml:183-192`（local：`odom`/`base_link_fake`/三插件）· `:200-263`（STVL，源=`/livox/lidar_filtered` PointCloud2）· `:229-230`（`min 0.15`/`max 1.0`）· `:273-281`（global：`map`/`base_link_fake`）· `:319-320`（global 源的同一带） | `nav2_params_sim_base.yaml:158-184`（local 四插件，`obstacle_layer`=`/scan` 默认开）· `:180`（`max 2.0`，**`min` 未设⇒0.0**）· `:301-323`（global）· `:354-355`（STVL `0.0/2.0`）· `nav2_params_sim_robot11_costmap.yaml:34-44`（伪造 z 的机理+实测） | **COD 有高度带（0.15/1.0），但量的是真 3D z；我们的 `obstacle_layer` 带量的是"传感器原点 z"。COD 没有这条路径 ⇒ 没有该失效模式。** |
| **Q4** | "COD 也是 nav2，为啥它没事" | `grep -c obstacle_layer` = **0 / 0**；`grep /scan` = **0**；`singlenav_launch.py` 全文件**没有** `pointcloud_to_laserscan` | 我们 local 默认 `local_obstacle:=scan`（`bringup_sim.launch.py:1416-1425`） | **COD 用的是 nav2，但没用 nav2 的 `obstacle_layer`+`LaserScan`。见 §5 的"它没用的 7 个特性"。** |
| **Q5** | 现在这个仿真架构有缺陷吧 | — | §6 并排表 | **(b) 约定分叉 + 可避免耦合；不是 (c) 真缺陷。** |

---

## 2. H1：COD 的 `base_link` 约定到底是什么

### 2.1 COD 侧证据链（四条，互相独立）

**(a) 上游 launch 声明的 `base_link→livox_frame` 是单位阵 —— 而且它从不被启用。**

`third_party/cod_nav_2026/src/small_point_lio/launch/small_point_lio.launch.py:24-45`：

```python
24:    static_base_link_to_livox_frame = Node(
25:        package="tf2_ros",
26:        executable="static_transform_publisher",
27:        arguments=[
28:            "--x", "0.0",
30:            "--y", "0.0",
32:            "--z", "0.0",
34:            "--roll", "0.0",
36:            "--pitch", "0.0",
38:            "--yaw", "0.0",
40:            "--frame-id", "base_link",
42:            "--child-frame-id", "livox_frame",
```
```python
47:    return LaunchDescription([small_point_lio_node])          # ← 只返回节点，静态 TF 未加入
```

> ⚠️ **这是本仓与 COD 的`small_point_lio` 的**唯一**一处差异**：我们的 vendored 副本
> `src/localization/rm_localization/small_point_lio/launch/small_point_lio.launch.py:47` 是
> `LaunchDescription([small_point_lio_node, static_base_link_to_livox_frame])`（`git blame` = 引入提交
> `98baa55`），**COD 那份没有**。也就是说：**那条"单位 extrinsic"在 COD 里是死代码**。
> 这条对 §2.1(c) 的推理很关键 —— COD 真机的 `base_link→livox_frame` **必须来自仓库之外**
> （他们自己的 URDF / 实车 bringup），且**必须是单位（或纯平移）**，否则 §3.3 的共轭公式会立刻算错。

**(b) COD 的 `cod_bringup` 里根本没有任何 `base_link→livox_frame` 发布者。**
全仓 `grep -rn "static_transform_publisher"` 只有 3 处：`singlenav_launch.py:69`（`map→odom`）、
`multiplenav_launch.py:97`（`map→odom`）、`small_point_lio.launch.py:26`（上面那条死代码）。
`robot_state_publisher` **未被启动**（只有 `navigation_launch.py:54`、`localization_launch.py:49`
两处注释性 URL 提及）。⇒ **这份仓库自己跑不起来 `odom→base_link`**，必须由外部 URDF 提供。

**(c) 节点内的换算式只在 `T(base_link←livox) = I` 时成立。** 见 §3.3 的逐行推导。

**(d) 兄弟节点的行为也把 `base_link` 当 `livox_frame` 用。**

| 节点 | 行 | 做的事 | 只有 `base_link≡livox_frame` 才自洽 |
|---|---|---|---|
| `cpp_lidar_filter` | `filter_node.cpp:81-119` | `pcl::CropBox` 直接作用在**输入点云自身坐标系**（=`livox_frame`），**不做 TF**；`:117` `output.header = msg->header` 原样保留输入帧 | 它却把可视化 marker 发在 `base_link`（`:56`）⇒ 注释/README 里的"base_link 裁剪盒"成立的前提就是两个帧相同 |
| `pointcloud_to_laserscan` | `multiplenav_launch.py:67-73` | 输入 `/livox/lidar`（帧 = `livox_frame`），却设 `target_frame: base_link`、`min_height: 0.1 / max_height: 1.00` | 高度带的语义是"离 base_link 的 z"，只有在两帧相同时才等于"离雷达的 z" |

**结论**：COD 的帧约定是 **`base_link` ≡ 雷达壳体系**。它的原点是雷达，它的 z 轴随雷达一起斜 30°。
这在 REP-103 的意义上**不是**一个合格的 `base_link`（`base_link` 应是车体水平投影中心、z 朝上），
但它**内部自洽**：所有消费者（LIO、裁剪盒、p2l、nav2）都按同一个斜系记账，账就对得上。

### 2.2 我们的约定（完全不同）

```
docs/tf_interface_contract.md:44
map ──(重定位：AMCL / slam_toolbox / ICP 之一发布)──► odom ──(里程计：LIO 发布)──► base_link ──(URDF 固定关节)──► livox_frame / imu_link …
```

| 项 | 我们 | 证据 |
|---|---|---|
| `base_link` 原点 | 底盘**最低点**、z=0（车体几何中心投影） | `robot_models.md:173`「底盘高 0.30 m（`base_link` 原点就是**最低点**，`z=0`）」 |
| `base_link` 姿态 | **重力对齐**（z 朝上），与雷达无关 | 同上 + `tilted_lidar_fidelity.md:22` 的实测：`base_link→livox_frame` rpy `0/0/0` |
| `livox_frame` | **独立子帧**，由 URDF 固定关节提供 | 默认模型 `[0,0,0.80] rpy 0/0/0`；robot11 `[0.000562,0.130916,0.157028] rpy 0/0/0`（§A.1）；`robot11_mount:=urdf|sensor` 时 `rpy=[-30,0,0]` |
| 谁提供这条边 | `robot_state_publisher`（URDF） | `bringup_sim.launch.py:1784-1786`「它**需要** `base_link→livox_frame` 这条静态 TF…仿真里由 robot_state_publisher 从 URDF 提供」 |

### 2.3 30° 必须被吸收在某个地方 —— 谁买单

| | **COD** | **我们** |
|---|---|---|
| 30° 记在哪 | **帧的定义里**：`base_link` 就是斜的。TF 里**没有**这个 30°，URDF 里也**没有**（这份仓库没有 URDF）。 | **一条显式 TF 边上**：`base_link→livox_frame` 的 `rpy`（`urdf`/`sensor` 档），**或**插件的 `<tilt_rpy>`（`plugin` 档，此时 TF rpy=0） |
| 谁"付钱" | **帧语义**付钱：`base_link` 不再是车体系 ⇒ 任何按 REP-103 理解它的消费者都错；高度带全部在斜系里量；渲染出来的 TF 树对物理挂装**说谎** | **整条栈**付钱：每个做 TF 合成、做高度带、做地面拟合的环节都必须对这个 30° 正确 —— 而其中有一环（上游 `small_point_lio`）**不正确** |
| 隐蔽性 | **极高**：没有一行代码写"30°"，grep 不到，review 也看不出来 | **低**：`grep -rn "rpy"` 一行就能找到 |
| 出错时的表现 | 只在"换个消费者"时暴露（且通常表现为精度略降，不是崩） | 立刻表现为 `/cloud_registered` 地面 30.7° 斜面、代价图把 30° 当世界（`tilted_lidar_fidelity.md:49` 实测 `7506/11622 = 64.6%` 的点"低于水平地面"） |
| 谁更难维护 | 换雷达/换挂装 ⇒ 全栈重新标定，且**没有任何地方记录了旧值** | 值集中、可算、可测（`_LioAdapterRobot11` 从 URDF 几何算，`bringup_sim.launch.py:2073`） |

> **所以"谁买单"的准确答案**：COD 把账记在**约定**上（一次性、零成本、但语义是错的、且不可见）；
> 我们把账记在**代码/配置**上（每次都要对、对不上就炸、但对得上就是对的）。

### 2.4 直接回答"base_link 不就应该和雷达正方向对齐吗"

**不应该 —— REP-103 的 `base_link` 是车体系，不是传感器系。** 但这里有个真实的两难：

* 若 `base_link` = 车体系（我们的选择），则 30° 必须由某条 TF 边承载，**所有**消费者都得处理它；
* 若 `base_link` = 雷达系（COD 的选择），则"承载"这一步消失了，但**所有**消费者都被悄悄搬进一个斜 30° 的世界。

**COD 之所以看起来"base_link 和雷达对齐了"，是因为他们干脆把 `base_link` 定义在雷达上。**
这不是"更正确的做法"，而是"把不变式从代码挪到了约定里"。对 2D nav2（几乎不关心 pitch，§4.3）
这个trade 很划算；对一个要同时服务 3D SLAM / ICP / cartographer / 多种重定位的**算法对比 bench**，
这个 trade 不划算 —— 因为 bench 的价值恰恰在于"每个环节的坐标语义都写下来、都可 A/B"。

---

## 3. H2：`small_point_lio` 的 TF 处理 —— 含**明确更正**

### 3.1 更正声明（必须显式写下）

> **此前的假设是错的。** 我们曾经推断："COD 实车不受 30° 斜装影响，是因为**它用的不是我们 vendored 的这套 LIO**
> （或用的是修过的版本）。"
>
> **事实**：COD 2026 成品（本地 `third_party/cod_nav_2026/`，Gitee `cod_-rm2026_-navigation@master`）
> 与 GitHub `rmul2026` **用的就是 `small_point_lio`**，并且与我们 vendored 的是**同一份上游**
> （`Yancey2023/small_point_lio@688d75c`，`ros2` 分支）——`docs/tilted_lidar_fidelity.md` §N.1 已用
> `git blame` + `sha256` + 与上游 `diff` 为空三条独立取证。本仓对该 `.cpp` 的**唯一**改动是
> `a281636`（1 ns 截断）与 `2a5a8b6`（修 bug ②③），**没有一行是"我们写错的"**（§N.0 第 1 条）。
>
> **COD 不炸的真原因不是"换了 LIO"，而是"换了 `base_link` 的定义"**：`T_bl ≡ I` ⇒ 上游那两个
> 变换 bug **同时退化**（§3.3、§3.4）。
>
> 顺带更正两处既有文档与本地源码不符的记述：
> * `docs/cod_nav_comparison.md:27`「全局 costmap … 只吃 `/scan`」——那是 2025 `master`/GitHub
>   `rmul2026` 的形态；**Gitee 2026 的两张图都不吃 `/scan`**（§4.5）。
> * `docs/cod_nav_comparison.md:43`「他们的带在雷达面**之上** 0.01~1.0」——Gitee 2026 的源级带是
>   **`0.15 / 1.0`** 且 `max` 是"哨兵高度以上不关心"（`singlenav2_params.yaml:229-230`）。

### 3.2 COD：谁发布 `odom→base_link`？—— **不是 adapter，是 LIO 节点内联**

```
third_party/cod_nav_2026/src/small_point_lio/src/small_point_lio_node.cpp:61-62
61:            transform_stamped.header.frame_id = "odom";
62:            transform_stamped.child_frame_id = "base_link";
...
98:            tf_broadcaster->sendTransform(transform_stamped);
```

* 发布者 = `small_point_lio` 节点本身，**硬编码**帧名，**无参数可关**（`:61-62` 是字面量，不是 `declare_parameter`）。
* COD **没有**任何 `lio_tf_adapter` 之类的桥接包：`ls third_party/cod_nav_2026/src/` 只有
  `small_point_lio` / `cpp_lidar_filter` / `pointcloud_to_laserscan` / `fake_vel_transform` /
  `cod_bringup` / `goal_approach_controller` / `pb_nav2_plugins` / `pb_omni_pid_pursuit_controller` /
  `ros2_simple_serial` / `waypoint_editor` 十个包，**没有 adapter 类构件**。
* LIO 的输入/输出帧约定：`mid360.yaml:6` `lidar_frame: livox_frame`；`:30-33`
  `extrinsic_T=[-0.011,-0.02329,0.04412]`、`extrinsic_R=I`（**MID-360 出厂 IMU↔LiDAR 杆臂**）。

### 3.3 共轭还是合成？—— 逐行推导，判决是**共轭**

```cpp
:63:  geometry_msgs::msg::TransformStamped base_link_to_lidar_frame_transform;
:65:      base_link_to_lidar_frame_transform = tf_buffer->lookupTransform(lidar_frame, "base_link", time_msg);
:70:  tf2::Transform tf_lidar_odom_to_lidar_frame;                       // T_ol
:75:  tf2::Transform tf_odom_to_base_link = tf_base_link_to_lidar_frame.inverse()
:75:                                    * tf_lidar_odom_to_lidar_frame
:75:                                    * tf_base_link_to_lidar_frame;  // T_bl⁻¹ · T_ol · T_bl
```

记法（与代码一致）：`T_bl := lookupTransform(livox_frame, "base_link")` = **base_link 在 livox 系里的位姿**
（tf2 语义：`lookupTransform(target, source)` 返回 `T_target←source`）。`T_ol` = LIO 输出的 body(=livox) 位姿。

* **正确的合成**（链式法则）：`p_odom = T_ol·p_livox = T_ol·(T_bl·p_base)` ⇒ **`T_ob = T_ol·T_bl`**。
* **COD 的写法**：`T_bl⁻¹·T_ol·T_bl` = **共轭 / 相似变换**。把它展开：

| 分量 | 正确值（合成） | COD（共轭） | 何时相同 |
|---|---|---|---|
| 旋转 | `R_ol·R_bl` | `R_bl⁻¹·R_ol·R_bl` | 仅 `R_bl = I`（因为 `R_ol·R_bl` 与 `R_bl⁻¹R_olR_bl` 一般不等） |
| 平移 | `p_ol` | `R_bl⁻¹·p_ol` | 仅 `R_bl = I` |

⇒ **只有当 `T_bl = I`（单位阵）时，共轭才与合成逐位相等**，此时它退化成一个恒等操作，
`odom→base_link` 就等于 LIO 的原始输出。

> 我们的 `docs/tilted_lidar_fidelity.md` §J.1.2 已经独立得出同一结论：
> 「**共轭只在 `T_bl` 是纯平移时"姿态碰巧对"**」，并给了判决性数字
> （`urdf` 档改前假俯仰 **4.890°**、改后 **0.317°**）。本文的增量是：
> **COD 的 `T_bl` 恰好就是 I**，所以这个 bug 在他们的部署里**不可观测**。

### 3.4 是否对"已经配准的点"再变换了一次？—— **是**，同样被 `T_bl≡I` 掩盖

```
:101:  small_point_lio->set_pointcloud_callback([...](const std::vector<Eigen::Vector3f> &pointcloud) {
:127:                msg.header.frame_id = "odom";
:159:                    transformed_point = lidar_frame_to_base_link_R * point + lidar_frame_to_base_link_T;
```

而回调拿到的点**已经是 odom 系**——`small_point_lio.cpp:98`：

```cpp
98:  pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());
```

⇒ 数据自称 `odom`（`:127`），却又被乘了一次 `T(base_link←livox)`（`:159`）。
这与本仓 **bug ②** 是同一处（§N.1 第 6 行，blame `98baa55` = 上游原样）。
`|t| = 0.2044 m` 的纯平移在 COD 里表现为"整朵云平移 20 cm"（`extrinsic_T` 只有 4.4 cm，
而 COD 的实车 `base_link→livox_frame` 若为纯平移，这个平移量就是它）——**肉眼与单帧统计都看不出**。

### 3.5 我们：`lio_tf_adapter` 是**正确的合成**，却被排除掉了

```
src/localization/rm_localization/lio_tf_adapter/src/lio_tf_adapter_node.cpp:66
66:    const tf2::Quaternion q_out = q_lio * q_offset;
67:    const tf2::Vector3 t_offset(xyz_[0], xyz_[1], xyz_[2]);
68:    const tf2::Vector3 t_rotated = tf2::quatRotate(q_lio, t_offset);
70-72: tf_msg.transform.translation.{x,y,z} = msg->pose.pose.position.{x,y,z} + t_rotated.{x,y,z}();
```

即 `T_out = T_lio ∘ T_offset`（`q = q_lio·q_off`、`p = p_lio + R_lio·t_off`）—— **这是链式法则的正确写法**，
也正是 §N.0 第 2 条所说的"本仓唯一写对合成的桥接"。

**但它在 `lio:=small_point_lio` 时被明确排除**（三处，互斥）：

```
src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py:2028-2033
'", LaunchConfiguration('lio'), "' != 'none' and '",
'", LaunchConfiguration('lio'), "' != 'cartographer' and '",
'", LaunchConfiguration('lio'), "' != 'small_point_lio' and '",       # ← 排除
'", LaunchConfiguration('robot'), "' != 'hzmirm' and '",
'", LaunchConfiguration('robot'), "' != 'robot11'"])))
```
（`:2051-2055` hzmirm 分支、`:2076-2080` robot11 分支带**同样的** `!= 'small_point_lio'` 条件。）

理由注释在 `:2021-2026`，其中一句是准确且重要的：

```
:2025:  #   杆臂由它自己处理（⚠️ 该换算的写法有瑕疵，见 docs/lio_slots.md §6 第 2 条）。
```

⇒ **我们当时就知道那处"有瑕疵"，但选择"排除 adapter"而不是"修 LIO"**（后来 `2a5a8b6` 才修 LIO）。
后果：`robot:=robot11` 的 30° `rpy` 补偿（`_LioAdapterRobot11('rpy')`，`:2088`）
**在 `lio:=small_point_lio` 时没有任何载体**——它只对 `lio:=fastlio|pointlio` 生效。

### 3.6 H2 判决

| 问题 | 答案 |
|---|---|
| COD 跑的是同一个 `small_point_lio` 吗？ | **是**，同一上游 commit，逐字节相同（§N.1）。**"COD 换了 LIO"这个前提是错的。** |
| COD 有 `odom→base_link` 的等价处理吗？ | **有**，但是**节点内联的共轭**（`:75`），不是 `lio_tf_adapter` 式的合成，也**没有**独立 adapter 节点。 |
| 那个处理"正确"吗？ | **仅在 `T(base_link←livox) = I` 时正确**——而这正是 COD 的约定（§2.1）。所以它是**"约定正确"，不是"数学正确"**。 |
| 我们排除 `lio_tf_adapter` 是对的吗？ | **当时不得已、今天仍不对称**：两次发布者冲突是真实约束（`:2021-2022`），但排除掉的恰好是本仓唯一写对合成的实现，而需要那个合成的正是 30° 斜装槽位。**§8 的选项 B 就是收这一条。** |

---

## 4. H3：COD 的 nav2 用哪些帧、代价图吃什么、有没有高度带

### 4.1 COD 的两张代价图（`singlenav2_params.yaml`，单点模式 = 比赛模式）

| 项 | local_costmap | global_costmap |
|---|---|---|
| `global_frame` | **`odom`**（`:183`） | **`map`**（`:273`） |
| `robot_base_frame` | `base_link_fake`（`:184`） | `base_link_fake`（`:274`） |
| `rolling_window` | `true`（`:186`）· 10×10 @0.05（`:187-189`） | **`true`**（`:275`）· 25×25 @0.04（`:269-270,279`） |
| `robot_radius` / footprint | `0.1` / `[±0.1]²`（`:190-191`） | 同（`:277-278`） |
| **插件列表** | **`["static_layer","stvl_voxel_layer","inflation_layer"]`（`:192`）** | **`["static_layer","stvl_voxel_layer","inflation_layer"]`（`:281`）** |
| **`obstacle_layer`** | **无** | **无** |
| 观测源 | `observation_sources: livox_source realsense_source`（`:218`）· `livox_source.topic = /livox/lidar_filtered`、`data_type: PointCloud2`（`:220-221`） | 同（`:308,311`） |
| **高度带** | **源级** `min_obstacle_height: 0.15` / `max_obstacle_height: 1.0`（`:229-230`）；RealSense `0.18/1.5`（`:252-253`） | **源级** 同一组（`:319-320`、`:342-343`） |
| 层级其它 | `max_obstacle_height 2.0`（`:208`）、`origin_z 0.0`（`:213`）、`voxel_decay 0.5`（`:203`）、`decay_model 0`（`:204`）、`model_type 1`（`:237`）、`vertical_fov_angle 2.00`（`:238`） | 同（`:298-300` 等，逐项镜像本表左列） |

**多模式（`multiplenav2_params.yaml`）**：帧与插件结构完全相同（`:177-186` local、`:267-275` global），
高度带 `0.1/1.0`（`:223-224`、`:313-314`），另外多了 `pointcloud_to_laserscan` → `/scan` → `slam_toolbox`
（`multiplenav_launch.py:65-94`）—— **`/scan` 只喂建图，不进任何 costmap**（§5 第 6 条）。

### 4.2 我们（对照）

| 项 | local_costmap | global_costmap |
|---|---|---|
| `global_frame` | **`odom`**（`nav2_params_sim_base.yaml:128`） | **`map`**（`:284`） |
| `robot_base_frame` | `base_link_fake`（`:129`） | `base_link_fake`（`:285`） |
| 插件列表 | `["obstacle_layer","obstacle_cloud_layer","stvl_layer","inflation_layer"]`（`:158`） | `["static_layer","obstacle_layer","stvl_layer","inflation_layer"]`（`:301`） |
| **默认启用** | **`obstacle_layer` = `/scan`**（`local_obstacle:=scan`，`bringup_sim.launch.py:1416-1425`） | **`stvl_layer`**（`global_obstacle:=stvl`，`:1408-1414`） |
| `obstacle_layer` 高度带 | `max_obstacle_height: 2.0`（`:180`）；**`min` 未设 ⇒ nav2 默认 `0.0`**（`obstacle_layer.cpp:142`） | 同（`:323`） |
| STVL 高度带 | `min 0.0 / max 2.0`（`:250-251`），**注释显式登记这是对 COD `0.15` 的偏离**（`:218-222`） | `min 0.0 / max 2.0`（`:354-355`），同样登记（`:350-355`） |
| STVL 源 | `/segmentation/obstacle`（`:246`） | `/segmentation/obstacle`（`:346`） |

### 4.3 关键区别：两种"高度"是**不同物理量**

这是整个 H3 的枢纽。

**(i) COD 的路径：点云 → STVL，`z` 是点的真实 3D 坐标。**
STVL 把 `PointCloud2` 的每个点按其 (x,y,z) 落到体素里，`min/max_obstacle_height` 在**代价图帧**的 z 上过滤。
每个点的 z 是 LIO 配准出来的**真实几何高度**（相对代价图帧）。

**(ii) 我们的路径：`/scan` → nav2 `obstacle_layer`，`z` 是**伪造**的。**

```
third_party/nav2/nav2_costmap_2d/plugins/obstacle_layer.cpp:337
337:        projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
```
`LaserScan` 没有高度；`transformLaserScanToPointCloud` 把每条波束按 `range/angle` 展开成
`z = 0`（该帧内）的点，然后：

```
third_party/nav2/nav2_costmap_2d/src/observation_buffer.cpp:143-144
143:      if ((*iter_z) <= max_obstacle_height_ &&
144:        (*iter_z) >= min_obstacle_height_)
```
这段过滤发生在 `global_frame_cloud`（已变换到**代价图帧**）上。而"`z=0` 的点从扫描帧搬到代价图帧"
只会带上一个平移量 ⇒ **每条波束在代价图帧里的 z 恒等于"那一帧扫描原点的 z"**，
与它实际打到什么三维结构**完全无关**。

我们自己的实测（`src/bringup/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml:34-41`，
以及 `traversability_near_ground_robot11.yaml:17-21`）：

> `/scan` 是**二维平盘**（LaserScan 没有高度），nav2 的 `obstacle_layer` 先 `projectLaser`（**z 强行置 0**）
> 再 `transformLaserScanToPointCloud` 搬到代价图帧 ⇒ **每条波束在代价图帧里的 z 恒等于"那一帧点云原点的 odom z"**，
> 与它打到的三维点无关。实测（robot11 plugin 档、静止）：950 条波束的 odom z 全在 **[0.0564, 0.0815]**（跨度 25 mm），
> `[0,2]` 之外 **0 条**。⇒ 这两个键在 2D 链路上**只是"传感器自身高度"的常量闸**。

⇒ **"把扫描面挪 0.2 m ⇒ 950/950 条波束全丢"** 这个现象，是 (ii) 这条路径的**必然**：
整个平盘一起被移出 `[min,max]` 带。它在 (i) 里**不可能**发生 —— 因为 (i) 里每个点的高度是它自己的。

### 4.4 COD 有没有那个失败模式？—— **没有，而且连"可能没有"的余地都没有**

因为 **COD 完全没有启用 `obstacle_layer`**：

```
$ grep -c "obstacle_layer" src/cod_bringup/params/singlenav2_params.yaml      → 0
$ grep -c "obstacle_layer" src/cod_bringup/params/multiplenav2_params.yaml    → 0
$ grep -n "/scan\|scan:" src/cod_bringup/params/singlenav2_params.yaml        → (无输出)
```
```
$ grep -rn "static_transform_publisher" src/                                        → 3 处，全是 map→odom（+1 处死代码）
$ ls src/ | grep -c pointcloud_to_laserscan                                          → 1，但只在 multiplenav_launch.py 被 include
$ grep -n "pointcloud_to_laserscan" src/cod_bringup/launch/singlenav_launch.py       → (无输出)
```

⇒ **单点（比赛）模式的 COD 不存在任何 `LaserScan` 进入代价图的通道**；`/scan` 这个类型在它的
nav2 参数里一次都没出现。**"每束 z 由扫描原点伪造"这条代码路径没有被执行过。**

### 4.5 但 COD **也有**一个"高度带在斜系里量"的问题（诚实登记）

COD 的 `odom` **不是重力对齐的**，而是"**初始 LiDAR 系**"。取证：

```
src/small_point_lio/src/small_point_lio/eskf.h:30
30:        Eigen::Matrix<value_type, 3, 3> rotation = Eigen::Matrix<value_type, 3, 3>::Identity();    // 旋转
```
```
src/small_point_lio/src/small_point_lio/small_point_lio.cpp:54-64
54:                if (parameters.fix_gravity_direction) {
55:                    estimator.kf.x.gravity = Eigen::Matrix<state::value_type, 3, 1>::Zero();
57:                        estimator.kf.x.gravity += imu_msg.linear_acceleration.cast<state::value_type>();
59:                    state::value_type scale = -static_cast<state::value_type>(parameters.gravity.norm()) / estimator.kf.x.gravity.norm();
60:                    estimator.kf.x.gravity *= scale;
64:                estimator.kf.x.acceleration = -estimator.kf.x.gravity;
```

`R_W_B(0) = I`（`:30`）且 `gravity` 直接取"前 200 帧 IMU 加速度之和"（`:55-60`，**在 body 系里量的**）
⇒ 重力被当成一个**世界系的自由 3 维向量**（`:64` 用 `-gravity` 抵消），
**世界系（`odom`）被定义为初始 body 系**。所以 30° 斜装 ⇒ **`odom` 的 z 轴也斜 30°**，
`gravity` 状态里存着一个斜的重力向量。

**后果**（我们的判断，标记为推断，见 §9）：
1. STVL 的 `min/max_obstacle_height 0.15/1.0` 量的是**沿斜 30° 的 z**，不是真高度 —— 一个 0.85 m 宽的斜板；
2. `pointcloud_to_laserscan` 的 `min_height 0.1 / max_height 1.0`（`multiplenav_launch.py:72-73`）
   在 `target_frame: base_link`（= 斜系）里切片 ⇒ 也是一个斜板；
3. nav2 的 footprint、`robot_radius`、yaw 全在斜系里 ⇒ 与地面真投影差一个 `cos30°` 量级的因子。

**但这三条与"950/950 全丢"不是同一类失效**：
前者的失效是"**漏掉一部分本应看见的障碍**"（连续、可容忍、可以靠加宽带子掩盖）；
后者是"**整帧观测归零**"（离散、致命、且看起来像"没有障碍物"⇒ 直接撞）。
COD 靠"带子宽 + 量真值 + 不在回路里做 2D→3D 反投影"把前者压到了不可见。

---

## 5. "cod 那个也是 nav2 阿，为啥那个 nav2 没问题" —— 它**没用**/改配了这 7 条

**是的，COD 用的就是 nav2（`cod_bringup/launch/navigation_launch.py` 起 nav2 全家桶），
而且它踩的坑和我们是同一批 nav2 代码。差别全在"用了哪几个特性"：**

| # | nav2 特性 | **COD** | **我们** | 这一条是否解释了"我们不炸它炸" |
|---|---|---|---|---|
| 1 | **`nav2_costmap_2d::ObstacleLayer` + `LaserScan`** | **完全不用**（两份 params `grep` = 0） | **local 默认开**，global 可选 | ✅ **主因**。这条路径的 z 是伪造的（§4.3-ii） |
| 2 | **`/scan` 作为代价图源** | **不用**（`/scan` 只喂 `slam_toolbox`，仅多模式） | 是 local 的**唯一** 2D 源（`local_obstacle:=scan`） | ✅ 同上 |
| 3 | **3D→2D 反投影（p2l）在导航回路里** | **不在**（单点模式无 p2l；多模式 p2l 只喂建图，且 `map→odom` 是静态的） | **在**（`linefit→/segmentation/obstacle→p2l→/scan→costmap`） | ✅ 我们的"高度带"因此天然有两个frame |
| 4 | **`robot_base_frame` 的父帧语义** | `base_link_fake` 的父 `base_link` **= 雷达系**（无 30° 记账） | `base_link_fake` 的父 `base_link` **= 车体系**（有 30° 记账） | ✅ §2 的全部内容 |
| 5 | **`map→odom` 的来源** | **静态 TF**（`singlenav_launch.py:67-89`，`z=0.05, yaw=-0.5`；多模式 `yaw=0`）—— 没有任何定位节点（`localization_launch.py` `lifecycle_nodes=['map_server']`） | **6 选 1**（amcl/beluga/slam_toolbox/icp/gicp/small_gicp/cartographer） | ⚠️ **不是**原因，但决定了"帧树错了会不会被发现"：COD 静态桥没有反馈回路能暴露误差 |
| 6 | **`pointcloud_to_laserscan`** | 只在多模式、只喂 `slam_toolbox`（`multiplenav_launch.py:65-94`） | **核心环节**（p2l 是感知域的唯一 2D 出口） | ✅ 同 #3 |
| 7 | **全局代价图是滚动窗** | `rolling_window: true`、25×25（`singlenav2_params.yaml:275`） | `map` 系、固定、`static_layer` | ⚠️ 影响"全局图能不能把错误摊薄"，非主因 |

**一句话**：**"COD 也用 nav2"这句话是对的，但它只用了 nav2 的
`planner / controller / bt / behavior / STVL / static / inflation`，
把 `obstacle_layer + LaserScan` 这条最容易踩帧坑的路径整个弃用了。**
它是"用 nav2 的骨架 + 自己的 3D 感知"，我们是"用 nav2 的全套 2D 感知"。

---

## 6. 架构判决

### 6.1 并排表（任务要求的 8 行）

| 维度 | **COD**（Gitee 2026 成品） | **我们**（本仓默认） | 谁更"稳" |
|---|---|---|---|
| **① TF 契约链** | `map --(静态 TF，z=0.05,yaw=-0.5)--> odom --(small_point_lio 内联)--> base_link --(fake_vel_transform，**仅 yaw**)--> base_link_fake`；**`livox_frame` 与 `base_link` 是同一个系，无边** | `map --(6 选 1 重定位)--> odom --(lio_tf_adapter **或** LIO 内联)--> base_link --(**URDF 固定关节**)--> livox_frame/imu_link`；`base_link --(fake_vel_transform，仅 yaw)--> base_link_fake` | COD 更短 ⇒ 更少出错点；我们更长 ⇒ 语义更全 |
| **② `base_link` 定义** | **= 雷达壳体系**（原点在雷达、z 随雷达斜） | **= 车体系**（原点在底盘最低点、z 朝上/重力对齐） | 我们对（REP-103）；COD 稳（无记账） |
| **③ 30° 记在哪** | **帧定义 + ESKF 的 `gravity` 状态**（`eskf.h:30` + `small_point_lio.cpp:54-64`）；TF/URDF 里**没有** | **一条显式 TF 边**（`urdf`/`sensor` 档的 `livox_frame` rpy）**或插件 `<tilt_rpy>`**（`plugin` 档） | COD 不可见；我们可见可测 |
| **④ 谁发 `map→odom`** | `static_transform_publisher`（**无定位**） | amcl / beluga / slam_toolbox / icp / gicp / small_gicp / cartographer（+ `lio:=none` 的静态回退桥） | 我们强得多（且这正是 bench 的资产） |
| **⑤ costmap 吃什么** | local/global **都是** `static + STVL(PointCloud2)`；源 = `/livox/lidar_filtered` + RealSense | local：`obstacle_layer(/scan)` **默认开** + cloud + STVL(可选)；global：`static + STVL(/segmentation/obstacle)` | COD 单源单帧、我们多源多帧（bench 需要，但默认值把它暴露在 §4.3-ii 上） |
| **⑥ "height" 参数住在哪** | **一处**：STVL 源级 `min/max_obstacle_height`（`0.15/1.0`），量**真 3D z** | **四处**：nav2 `obstacle_layer` 源级（`min` 默认 0.0，量**伪造 z**）、STVL 源级（`0.0/2.0`，量真 z）、`linefit` 的 `sensor_height`+`gravity_aligned_frame`、p2l 的 `min/max_height`+`target_frame`、自击掩膜的 z 门限 | **COD 一处、我们四处** ⇒ 这是最该收的一条 |
| **⑦ 3D→2D 转换步骤** | **不在导航回路里**（单点无 p2l；多模式 p2l 只喂建图，且 `map→odom` 静态 ⇒ 不回灌 costmap） | **在回路里且是唯一 2D 出口**：`linefit/patchwork → /segmentation/obstacle → p2l → /scan → costmap` | COD 天然免疫 |
| **⑧ 斜装雷达会破几个地方** | **2 个**（STVL 斜板、p2l 斜板）+1 个语义债（`base_link` 不再是车体系）；**都不在致命路径上** | **6 个**（下表 §7.2）：TF 合成 · LIO 共轭 · linefit 重力系 · p2l 高度带 · nav2 伪造 z · 自击掩膜 z | **2 vs 6** |

### 6.2 判决：**(b) 约定分叉 + 可避免耦合；不是 (c) 真缺陷**

| 选项 | 判定 | 理由 |
|---|---|---|
| **(a) 架构没问题，只是文档不够** | ❌ 不成立 | 光靠文档解决不了 §7.2 的 6 条 —— 其中 2 条是真 bug（上游共轭、双重变换），2 条是**没有唯一所有者**的配置（高度带、`lio_tf_adapter` 排除），1 条是我们把上游死代码启用了。文档是必要条件，不是充分条件。 |
| **(b) 架构没问题，但默认值/约定与 COD 分叉，并额外引入了可避免的耦合** | ✅ **成立（主判决）** | 我们的契约（REP-103 base_link、显式 TF 边、多重重定位）对一个**算法对比 bench** 是**更正确**的选择，不该向 COD 看齐。真正该修的是：**(b1)** 让"高度"只有一个所有者；**(b2)** 让 `lio_tf_adapter` 成为所有 LIO 槽的**唯一** `odom→base_link` 权威；**(b3)** 把 `robot11_mount` 三档收敛成一档默认 + 一档保真；**(b4)** 清掉那条被启用的上游单位 TF。 |
| **(c) 架构真缺陷** | ❌ 不成立 | "多一层显式 TF 边"不是缺陷，是 REP-103 的**要求**。同样的 30° 若放进 COD 的约定里，代价是"所有高度语义静默偏 30°"，那才是更深的缺陷 —— 只是不可见。真正属于缺陷级的只有**上游 `small_point_lio` 的共轭/双重变换**，而它是**第三方 bug**（§N.1：blame `98baa55` = 上游逐字节原样），且我们已修（`2a5a8b6`）。 |

> **换个说法（给用户的一句话）**：不是"我们的仿真架构有缺陷"，而是
> **"我们把一个 COD 藏在约定里的 30°，写进了一条显式 TF 边；显式的东西会被检查，于是我们查出了 6 个问题，
> 其中 2 个是上游 bug、4 个是我们自己的默认值/接线问题。"**
> 查出 6 个不是架构的错，是**架构更诚实**的代价；但"6 个里有 4 个是我们自己造的"这部分，必须修。

---

## 7. 可避免耦合清单

### 7.1 先列"固有的"（**不是我们的错，别去修**）

| # | 固有项 | 为什么固有 | 证据 |
|---|---|---|---|
| I1 | 斜装雷达**必须**被吸收在某处 | 物理事实。要么改 `base_link` 语义（COD），要么写一条 TF 边（我们）。没有第三条路 | §2.3 |
| I2 | `small_point_lio` 的**共轭**与**双重变换**是上游 bug | 本仓引入时与上游 `688d75c` 逐字节相同；`git blame` 全在 `98baa55` | `tilted_lidar_fidelity.md` §N.0 第 1 条、§N.1 第 3/6 行 |
| I3 | nav2 `obstacle_layer` 的 **z 伪造** | 上游设计：`LaserScan` 里没有高度信息，`projectLaser` 只能给 `z=0`。任何 2D 雷达消费者都逃不掉 | `obstacle_layer.cpp:337`、`observation_buffer.cpp:143-144` |
| I4 | `pointcloud_to_laserscan` 的 `min/max_height` 天然在 **target_frame** 里量 | 上游语义 | `multiplenav_launch.py:70-73` 与我们对 p2l 的用法同构 |
| I5 | 我们选 REP-103 的 `base_link`（车体系） | **有意的契约选择**，bench 必须同时服务 3D SLAM / ICP / cartographer / 4 种重定位 | `tf_interface_contract.md:44` |
| I6 | `base_link_fake` 这层"假帧" | **两边都有**（同源北极熊模式）。而且这个 trade 上 **COD 更疼**：它的 `base_link_fake` 继承了 30° pitch，我们的不会（我们 `base_link` 是平的） | COD `fake_vel_transform.cpp:44-51`（只做 yaw，pitch 透传）；我们 `tf_interface_contract.md:82-88`（T6 撤销说明） |

### 7.2 再列"我们的"（**可避免，且是我们自己引入的**）

| # | 耦合 | 证据 | 怎么来的 | 该不该修 |
|---|---|---|---|---|
| **O1** | **高度阈值住在 4 个 frame 里、没有唯一所有者**：nav2 `obstacle_layer` 源级（`min` 默认 `0.0`，量**伪造 z**）、STVL 源级（`0.0/2.0`，量真 z）、`linefit` 的 `sensor_height` + `gravity_aligned_frame`、p2l 的 `min/max_height` + `target_frame`、自击掩膜 z 门限 | `nav2_params_sim_base.yaml:180,196-197,250-251,354-355`；`traversability_self_mask_robot11.yaml`；`laserscan_params*` | 每加一个感知槽就加一组高度键，没有"谁拥有地面/高度语义"的裁决 | ✅ **修**（这是 §8 选项 B 的一半） |
| **O2** | **`lio_tf_adapter` 被 `lio:=small_point_lio` 排除** ⇒ 本仓唯一写对**合成**的实现，恰好在最需要它的槽位上不跑；`robot11` 的 30° `rpy` 补偿没有载体 | `bringup_sim.launch.py:2028-2033`、`:2051-2055`、`:2076-2080`（三处 `!= 'small_point_lio'`）、`:2087-2088` | 为了消除"两个 `odom→base_link` 发布者"（真实约束），选择了"排除 adapter"而不是"统一到 adapter" | ✅ **修**（§8 选项 B） |
| **O3** | **`robot11_mount` 三档并存**（`plugin` 默认 / `urdf` **诊断档·账是错的** / `sensor` 物理保真） | `bringup_sim.launch.py:1287-1301`（声明与描述）、`:507-533`（横幅） | 为了"证明物理斜装的后果"造了 `urdf` 档，之后没退役 | ✅ **修**（§8 选项 C：收敛为 1 默认 + 1 保真，`urdf` 不作一等选项） |
| **O4** | **我们启用了上游那条单位 `base_link→livox_frame`** | `src/localization/rm_localization/small_point_lio/launch/small_point_lio.launch.py:47` vs COD 的同文件 `:47`（**唯一差异**）；blame = `98baa55` | 上游自己 `return LaunchDescription([small_point_lio_node])`（死代码），我们把它**加进了返回值** | ✅ **修**（一行：删掉，或加注释"与 URDF 30° 关节互斥"）。**当前 bringup 不走这个 launch**（`bringup_sim.launch.py:1794-1806` 自建 Node）⇒ 是**潜伏**坑，不是活的 |
| **O5** | local costmap 的**默认值**把 `/scan` 当唯一 2D 源（`local_obstacle:=scan`），而传感器槽位允许 30° 斜装 | `bringup_sim.launch.py:1416-1425`；`nav2_params_sim_base.yaml:158-184` | 默认值是历史行为（"原行为，链路单点"），斜装槽位是后来加的 | ⚠️ **半修**：bench 需要这个槽位（A/B 要用），但**默认值**应随 `robot11_mount` 一起切，或至少在 `robot:=robot11` 时把 `min/max_obstacle_height` 的语义在文档/参数里显式声明为"传感器高度闸"（`nav2_params_sim_robot11_costmap.yaml:34-41` 已写了注释，但默认值没动） |
| **O6** | 30° 的**方向**（roll vs pitch）有两份互相矛盾的来源，且 `livox_tilt_axis` 默认 `roll` | `robot_models.md:653`（URDF 原文 `rpy="0 -0.5236 0"`= pitch vs SolidWorks CSV= roll）、`§9.5`（方位 360° ⇒ **仰角**测不出来） | 上游 URDF 与实物 CSV 冲突，我们选了 CSV | ⚠️ **保留但写死**：它只影响 `_LioAdapterRobot11` 的 rpy 分解，不影响辐条方向（§9.5 已证）；但**必须在一个地方写清"默认 roll，理由=X"** |

**小结：6 个可避免耦合，4 个明确该修（O1/O2/O3/O4），2 个半修（O5/O6）。**

---

## 8. 三个选项（按 成本/风险 排序）

> 目标锚定：**"算法对比 bench"，不是 COD 克隆**。所以判据不是"更像 COD"，而是
> "**每个坐标语义只有一个所有者、可 A/B、可回归**"。

### 选项 C（先做，成本 S，风险低）—— 三档收敛成"一默认 + 一保真"，删掉诊断档

**做法**
1. `robot11_mount` 的取值从 `plugin|urdf|sensor` 收敛为 **`plugin`（默认，唯一支持的常规档）** + **`sensor`（物理保真档）**；
   `urdf` 保留为**文档里的反例**（`tilted_lidar_fidelity.md` §C/§I 已有完整数字），
   **从 `_LIVOX_MOUNTS`（`bringup_sim.launch.py:172`）与 `choices=list(_LIVOX_MOUNTS)`（`:1303`）里移除**，
   并让 `_validate_livox_mount`（`:175-181`）给出可操作报错。
2. 在同一处声明里写清"`plugin` = `R_bl ≡ I` + 倾角在插件 `<tilt_rpy>`"是**唯一支持的默认**（这正是用户 Q6 选项 A 的字面内容，见下）。
3. 清 O4：`src/localization/rm_localization/small_point_lio/launch/small_point_lio.launch.py:47` 去掉 `static_base_link_to_livox_frame`。

**修哪几条**：Q1（把"我们用哪种 30° 记账"钉死）、Q5 的 O3/O4/O6。
**成本**：改 1 个 launch 的 choices + 1 行 vendored launch + 文档；**不碰任何算法**。≤ 1 人日。
**风险**：低。`urdf` 档从来不是默认，去掉不影响任何已验证路径（§J.3 已证 `plugin`/`sensor` 之外逐字节不变）。
**验收测试**
```bash
# ① 档位只剩两个，且缺省仍是 plugin
ros2 launch rm_nav_bringup bringup_sim.launch.py --show-args | grep -A2 robot11_mount
# ② 选 urdf 必须给出可操作报错（而不是静默跑）
ros2 launch ... robot:=robot11 robot11_mount:=urdf   # 期望：非零退出 + 明确提示
# ③ 静态：vendored launch 不再包含单位 base_link→livox_frame
grep -c static_base_link_to_livox_frame src/localization/rm_localization/small_point_lio/launch/small_point_lio.launch.py   # 期望 0（或仅注释）
# ④ 回归：默认档行为逐字节不变
python3 tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5   # 期望 PASS/SUCCEEDED
```

---

### 选项 B（主体，成本 M，风险中）—— 让 `lio_tf_adapter` 成为**所有** LIO 槽的 `odom→base_link` 唯一权威 + 高度语义单一所有者

**B1. `odom→base_link` 单一权威（修 O2）**
- `lio:=small_point_lio` 时也启动 adapter；为消除"双发布者"，把 LIO 节点那条广播**关掉**。
  两条实现路径：
  * **B1a（推荐，需协调 agent A）**：给 `src/localization/rm_localization/small_point_lio/**` 加一个参数
    `publish_tf: true|false`（默认 `true`，保持 §J/§N 的默认路径逐字节不变），bringup 传 `false`。
    ⚠️ **该目录当前归 agent A 所有，本任务不可改**，需跨 agent 协调。
  * **B1b（零改上游）**：保留 LIO 的广播，但把它作为 **`odom→base_link_lio`**（改 `child_frame_id` 需改代码，同样要 B1a 的权限）；
    或者**接受现状**并把 adapter 的 `base_frame` 设成 `base_link_fake`（下策，会绕开 `base_link` 语义，不推荐）。
- 之后 `_LioAdapterRobot11('rpy')` 的 30° 补偿对**所有** LIO 槽生效，**同一个数**只写一次。

**B2. 高度语义单一所有者（修 O1）**
- 明确裁决：**"什么算障碍"的最终裁决权在感知域（`/segmentation/obstacle`），代价图不再自己判高度。**
- 具体：`obstacle_layer`（`/scan` 路径）的 `min/max_obstacle_height` **要么删掉、要么在注释里定义为"传感器高度闸"**，
  并把"地面剔除"的唯一责任压到 `obstacle_near_ground_m`（`nav2_params_sim_robot11_costmap.yaml:42-44` 已指出这条路）。
- STVL 的 `0.0/2.0` 保持（它量真 z，且已登记是对 COD `0.15/1.0` 的有意偏离）。

**修哪几条**：Q2 全部、Q3 的一半（高度带所有权）、Q5 的 O1/O2。
**成本**：B1 需改 `small_point_lio`（1 个 `declare_parameter` + 1 个 `if`）+ launch 3 处条件 + adapter 参数；B2 是 config + 注释。**M（2–3 人日，含回归）**。
**风险**：中。B1 动的是"谁发 `odom→base_link`"这条契约的**唯一性**，必须配静态守卫（见验收 ①）。
**验收测试**
```bash
# ① 单一发布者不变式（对 (lio, robot, robot11_mount) 的笛卡尔积做静态检查）
python3 tools/scripts/regress/<新增>_tf_publisher_uniqueness.py   # 期望：每组恰好 1 个 odom→base_link 发布者
# ② 合成正确性：发布的 T_ob 与 /odom + base_link→livox_frame 用链式法则复算一致
ros2 run tf2_ros tf2_echo odom base_link            # 与 /odom 的 rpy 误差 < 0.5°（§J.1.2 的验收线）
# ③ 30° 补偿真的生效（robot:=robot11 + lio:=small_point_lio，sensor 档）
#    期望：/cloud_registered 地面倾角 < 1.5°（§A.2 判据），odom→base_link pitch 不随 30° 走
# ④ 端到端
python3 tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5   # PASS/SUCCEEDED
```

---

### 选项 A（不推荐，成本 L，风险高）—— 全面采用 COD 约定

> ⚠️ **先说清一件事：用户提的"A = 对齐 `base_link` 与传感器壳 / 发 `R_bl = I` + 把倾角搬进插件"
> —— 后半句我们**已经是这样了****（`robot11_mount:=plugin`：`base_link→livox_frame` rpy = 0/0/0，
> 倾角由插件 `<tilt_rpy>` 承担，`tilted_lidar_fidelity.md:22`、`:82`）。
> 所以 A 若只做到"R_bl = I + 倾角在插件"，**它已经实现了**；A 的**真正增量**只剩
> "把 0.13/0.157 m 的杆臂也折进 `base_link`（即把 `livox_frame` 的原点搬到 `base_link`）"。

**做法**：`base_link` 原点搬到雷达安装面、`base_link→livox_frame ≡ I`、`odom` 由"初始雷达系"定义。

**会坏掉什么（逐条）**
1. `robot_models.md:173` 的"`base_link` 原点 = 底盘最低点、z=0"整条几何基线作废 ⇒
   `robot_radius` / footprint / inflation 全部要按"原点在 0.26 m 高的雷达"重算；
2. `linefit` 的 `gravity_aligned_frame`、`p2l` 的 `target_frame`、自击掩膜、STVL 高度带
   **5 个消费者**的"重力对齐"假设全部要改成"斜系"（或全部显式加 `target_frame`）；
3. `tf_interface_contract.md:44` 的契约链要重写；`base_link_fake` 会继承 30° pitch（COD 的现状）；
4. cartographer / AMCL / ICP / GICP / slam_toolbox **6 个重定位槽**都按 REP-103 的 `base_link` 工作 ⇒ 全部要验；
5. `docs/` 里已实测的十几个基线（§A/§B/§D/§K/§L/§M 的数字）全部失去可比性。

**修哪几条**：Q1（改成 COD 的答案）、**Q2/Q3 反而变差**（把 §3.3 的共轭风险变成"必须靠约定"）、Q5 的 O1 会**更糟**（高度语义更不可见）。
**成本**：**L（≥ 1 周 + 全部基线重跑）**；**风险：高**。
**结论**：**不推荐**。它把"显式可测"换成"隐式不可见"，与"算法对比 bench"的目标相反。
**验收测试**（若真做）：`view_frames` 无 `livox_frame` 边 + 全部 6 个重定位槽的 `nav_smoke_regression` PASS — 成本极高，收益是"更像 COD"。

---

### 排序一览

| 排 | 选项 | 修 Q1 | 修 Q2 | 修 Q3 | 修 Q5 | 成本 | 风险 | 建议 |
|---|---|---|---|---|---|---|---|---|
| **1** | **C**（三档收敛 + 清 O4） | ✅ 钉死约定 | — | — | O3/O4/O6 | S | 低 | **先做** |
| **2** | **B**（adapter 唯一权威 + 高度单一所有者） | — | ✅ 全部 | ✅ 一半 | O1/O2 | M | 中 | **主体** |
| 3 | A（照抄 COD） | ✅ 但换成错答案 | ❌ 更差 | ❌ 更差 | ❌ | L | 高 | **不做** |

---

## 9. 未验证 / 诚实清单

| # | 未验证项 | 为什么没验 | 影响 |
|---|---|---|---|
| 1 | **COD 真机 URDF 里 `base_link→livox_frame` 的真实数值** | 本地 checkout 只有 `src/`（Gitee 2026 仓），**没有 `urdf/`**；GitHub `rmul2026` 的 `urdf/` 不在本地 | §2/§3 的整个推理链依赖"它≈单位/纯平移"。**这是推断，不是实测**。反证：若它不是单位，COD 的 `odom→base_link` 按 §3.3 会明显错，其 nav2 不可能正常工作 ⇒ **旁证很强但不等于取证** |
| 2 | **COD 实车的雷达是否真的斜 30°、绕哪个轴、朝哪个方向** | 同上（无 URDF/无标定文件）；`docs/robot_models.md:653` 记录**我们**的 URDF 与 CSV 冲突，COD 的情况未知 | 影响 §4.5 的"斜板"方向与量级；**不影响 §4.4**（无 `obstacle_layer` 是结构事实） |
| 3 | **COD 的 `odom` 是否真的随雷达斜 30°** | 从 `eskf.h:30` + `small_point_lio.cpp:54-64` **推**出来的，**没有在 COD 数据上实测**（没有 bag、没有他们的 TF 录制） | §4.5 的三条后果是**推断**。若 `gravity` 状态被后续观测强烈拉回真垂直，则 `odom` 会渐进重力对齐，结论要改 |
| 4 | **COD 的 `base_link→livox_frame` 若缺失，`:65` 的 `lookupTransform` 会抛异常 ⇒ 直接 `return`（`:68`）** | 需要实跑才知道他们的图里到底有没有这条边 | 若真缺失：COD 实际是"没有任何 `odom→base_link`"，那 nav2 靠别的东西（例如他们仓库外的静态桥）—— 这会推翻 §2.1(c) 的表述顺序，但不推翻结论 |
| 5 | **`cpp_lidar_filter` 的裁剪盒数值究竟是按哪个系标的** | `filter_node.cpp` 作用在输入点云系、marker 发在 `base_link`（`:56` vs `:117`），源码自相矛盾；上游 README 也自相矛盾（`cod_nav_nav_stack_inventory.md:169` 已登记） | 只影响"他们有没有一个隐藏的 30° 记账"这一条旁证 |
| 6 | **COD 是否另有仓库外的启动项提供了 `base_link→livox_frame` 或 `urdf`** | 他们实车的 `nav.sh` / 上层 ws 不在本地 | 同 #4 |
| 7 | **`robot:=robot11` + `lio:=small_point_lio` 组合今天的实际数字** | `src/localization/rm_localization/small_point_lio/**` 正在被 **agent A** 改（`2a5a8b6` 已修 ②③ 的姿态，**平移按 §J.5 有意保留旧值**）；本任务**不跑仿真** | §3.5/§8-B 的"今天是否已经对"取决于 agent A 的收尾状态。本文按**当前 HEAD** 的工作树读源码（未提交改动不算） |
| 8 | **本文引用的 `docs/tilted_lidar_fidelity.md` §J/§N 行号** | 该文件正被 **agent A** 编辑，行号可能已漂移 | 引用时以**节号**（§J.1.2 / §N.0 / §N.1）为准，行号仅作参考 |
| 9 | **选项 B1a 的可行性** | 需要改 `src/localization/rm_localization/small_point_lio/**`，该目录**不属于本任务权限**（并发 agent A） | B1a 是"实现建议"，落地需跨 agent 协调；本任务只给方案与验收 |
| 10 | **COD 的 `map→odom` 静态 TF 的 `yaw=-0.5` 从哪来** | 无注释、无标定文件 | 与本文主题无关，仅登记 |

---

## 附：本文引用的本地 COD 文件清单（全部可读，非 submodule）

```
third_party/cod_nav_2026/src/
├── small_point_lio/
│   ├── launch/small_point_lio.launch.py            ← 单位 base_link→livox_frame 是死代码（:47）
│   ├── config/mid360.yaml                          ← lidar_frame=livox_frame，extrinsic_R=I
│   └── src/
│       ├── small_point_lio_node.cpp                ← odom→base_link 的**共轭**（:75）+ 点云双重变换（:159）
│       └── small_point_lio/{eskf.h,small_point_lio.cpp}   ← R_W_B(0)=I（eskf.h:30）、gravity 取 body 系测量（:54-64）
├── cod_bringup/
│   ├── launch/{singlenav,multiplenav,localization,navigation}_launch.py
│   └── params/{singlenav2,multiplenav2}_params.yaml ← obstacle_layer 命中 0、/scan 命中 0
├── cpp_lidar_filter/src/filter_node.cpp             ← CropBox 不做 TF，marker 却发 base_link
├── fake_vel_transform/src/fake_vel_transform.cpp    ← base_link→base_link_fake 仅 yaw
└── pointcloud_to_laserscan/                         ← 多模式才用，target_frame=base_link
```


---
