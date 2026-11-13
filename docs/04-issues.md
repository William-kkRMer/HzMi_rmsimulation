# 问题与踩坑（排查手册）

> 现象 → 根因 → 修法；按主题排

> **2026-11-13 合并**：本文件由 **19 份**同主题文档合并而成，原文全部保留、按来源分节（`## 来源：xxx`）。原件已删除，可从 git 历史取回。

**目录**
1. `issues_and_findings.md`
2. `path_clearance_and_contact.md`
3. `obstacle_judgement_map.md`
4. `recovery_fsm_plan.md`
5. `slope_speed_limiting.md`
6. `stvl_local_costmap.md`
7. `traversability_plan.md`
8. `kinodynamic_esdf_mpc_notes.md`
9. `nav2_vs_custom_planner.md`
10. `nav_strategy_taxonomy.md`
11. `gazebo_gui_troubleshooting.md`
12. `lio_drift_diagnosis.md`
13. `lio_divergence_no_impact.md`
14. `research_p2l_scan_stall.md`
15. `build_optimization.md`
16. `cloud_density_measurements.md`
17. `cartographer_2d_occupancy_semantics.md`
18. `livox_viewer2_launch_notes.md`
19. `livox_sensor_prior_art.md`

---


## 来源 1/19：`issues_and_findings.md`


> 本文汇总本轮仿真联调中**实际踩到的问题**（现象 → 根因 → 修复 → 证据）、**静默失效类坑**、
> **地图资产/坐标系结论**、**架构改动**与**概念澄清索引**，最后是**未完成事项**。
> 详细复现命令见 `docs/smoke_test_runbook.md`，分层设计见 `docs/architecture.md`。
> 想照着方法**一步步定位**（而不是查具体某条故障）见 `docs/debug_fastlio_cartographer.md`（分层判读法 + 工具 + 症状速查表）。

---

## 一、运行时故障：根因 → 修复

| # | 现象 | 根因 | 修复 | 证据 / 落点 |
|---|---|---|---|---|
| 1 | `Unable to parse the value of parameter robot_description as yaml` | launch_ros 把 URDF(XML) 当 YAML 解析（Humble 行为） | `ParameterValue(robot_description, value_type=str)` 包裹 | `hzmi_rm_simulation/launch/rm_simulation.launch.py` |
| 2 | `spawn_entity` 报 `No module named 'numpy'` | conda 的 python(3.13) 抢占 PATH；ROS Humble 需系统 python 3.10 | `conda config --set auto_activate_base false` + `~/.bashrc` 末尾 `export PATH=/usr/bin:$PATH` | 用户环境；**不要激活 `.venv`** |
| 3 | 成批 `Could not find requested resource in ament index`（nav2 组件）+ 缺 `nav2_rviz_plugins` | nav2 主体包未安装 | `apt install ros-humble-navigation2 nav2-bringup nav2-rviz-plugins`（+ `dwb-critics`） | 装后组件正常加载 |
| 4 | `/scan` 报 `incompatible QoS` | costmap `obstacle_layer` 订阅 reliable，而 `/scan` 是 best-effort | 三份 `nav2_params_sim_*.yaml` 的 scan 源加 `reliability_policy/qos_policy: best_effort` | `ros2 topic info /scan --verbose`：1 pub + 3 sub **全 BEST_EFFORT** |
| 5 | costmap 刷屏 `Sensor origin ... out of map bounds` + `odom→base_link` 抖动 | `odom` 有两个来源：Gazebo 真值 + LIO 都发 `/odom` | sim URDF 把 Gazebo 真值 remap 到 **`/odom_ground_truth`**，`publish_odom_tf=false`（T4） | `/odom` 由 LIO 独占 |
| 6 | RViz 里车/雷达**倾斜**（Gazebo 里是正的） | spawn `z=1.16` 而地面在 z≈0 → 悬空 1.1 m 坠落，FAST-LIO 在坠落中做重力初始化 | RMUL/RMUL2026 spawn `z` 改 **0.2** | `tf2_echo odom base_link` RPY ≈ (0.34°,0.23°,-0.35°) |
| 7 | `Package 'rm_nav_bringup' not found ... searching: ['/opt/ros/humble']` | 新终端只 source 了 /opt/ros | `source install/setup.bash` | runbook §0.1 |
| 8 | nav+amcl 卡在 `amcl: Waiting for map....` / `global_costmap: Invalid Frame "map"` | ① `map_server_launch.py` 与 `localization_amcl_launch.py` **各起了一个同名 `lifecycle_manager_localization`**；② `nav2_params_sim_*.yaml` 里 `yaml_filename` 被注释，而 nav2 的 `RewrittenYaml` **只替换已存在的键** → `parameter 'yaml_filename' is not initialized` | ① amcl launch 用**单一 manager 同时管 `map_server`+`amcl`**；② 恢复 `yaml_filename: ""` 键 | 沙箱实测：`Read map RMUL.pgm: 272 X 210` → `amcl: Received a 272 X 210 map` → 双双 Activating |
| 9 | `tf2_echo map odom` 报 `frame does not exist` | **amcl 只在收到 `/scan` 之后才发 `map→odom`**（`sendMapToOdomTransform` 只被 `laserReceived` 调用，且 `initial_pose_is_known_` 为假时 return） | 判据改为"先查 `ros2 topic hz /scan`" | 无头实测：无 `/scan` → 无 `map` 帧；补 `/scan` → `map→odom` 立现 |
| 10 | 发目标后车不走，`/cmd_vel` 只有 `linear.x: -0.05` + 旋转 | `-0.05` = nav2 BT `BackUp backup_speed`；根因是 **`planner_server: failed to generate a valid path`** → BT 恢复行为循环 | 见 §三（幽灵墙）；并新增 `tools/check_map_reachable.py` 选点前验证 | 用户机日志 + 工具判定 |
| 11 | 场景三(nav+ICP) 里 `map`/`odom` 帧一直不存在，Nav2 无法激活 | `spawn_entity: Spawn service failed` 后机器人晚 ~4 s 才插入，期间 LIO 尚未吐 `/odom`；**RMUL 场地网格 44 万三角面**（RMUL2026 仅 3187）加载慢 | 等 30~60 s 再判断；按 `/livox/lidar → /livox/imu → /imu/data → /odom` 逐段定位 | ICP 本身正常：`pcd point size: 97642, 4866`、`icp_registration initialized` |
| 12 | `ros2 topic hz /livox/lidar` 只有 ~1 Hz + 2.8 s 抖动 | **测量假象**：CustomMsg 每帧 3 万点，`topic hz` 反序列化跟不上 | 看 `/scan`（轻量）或 `/odom` 判断频率 | `/odom` = 8.1 Hz 正常 |
| 13 | `/amcl_pose` 的 covariance ≈ 0 | nav2 `set_initial_pose` 路径**不填协方差**（`amcl_node.cpp` 只设 position/orientation） | 正常现象；但初值给错时 AMCL 难以自纠 → 用 RViz `2D Pose Estimate` 重给 | 源码 L271-281 |
| 14 | `cartographer_node` 启动数秒后 `exit code -6`，日志 `Check failed: sensor_to_tracking->translation().norm() < 1e-5 The IMU frame must be colocated with the tracking frame` | lua 里 `tracking_frame = "livox_frame"`，而 `/livox/imu` 的 `frame_id` 是 **`imu_link`**（URDF：livox 在 `base_link+(0.12,0,0.175)`、imu 在 `+(0.12,0,0.125)`，差 **5cm**；而这 5cm 是 FAST-LIO 的 `extrinsic_T=[0,0,0.05]` 依赖的，**不能靠改 URDF 消除**）→ `sensor_bridge.cpp:136` 的 IMU 共位硬 CHECK 失败 → abort | **2026-09-21 修复**：`tracking_frame = "imu_link"`（官方 `mir-100-mapping.lua` 用 `imu_frame` 同做法）；`/scan` 由 URDF 静态 TF 转到 `imu_link`（纯 5cm 平移）。同时 `TRAJECTORY_BUILDER_2D.min_range` 0.45→0.2（与 p2l 对齐） | **A/B 证据**（沙箱，隔离 `ROS_DOMAIN_ID=42` + 合成 TF/IMU 消息）：旧配置 → `Check failed` + 中止；新配置 → 无 `Check failed`、`Added trajectory with ID '0'` 正常 |
| 15 | **`localization:=cartographer` 一启动就 FATAL**：`Check failed: 1 == reference_counts_.count(key)` / `Key 'pure_localization' was used the wrong number of times` | `cartographer_localization.lua` 把 `pure_localization` 与 `pure_localization_trimmer` 写在了 **`TRAJECTORY_BUILDER_2D`** 上，而 cartographer 是从**顶层 `TRAJECTORY_BUILDER`** 读（`cartographer/mapping/trajectory_builder_interface.cc` 的 `kDictionaryKey = "pure_localization_trimmer"`、`map_builder.cc` 的 `has_pure_localization_trimmer()`）。写错层级的键**永远不会被读**，而 `LuaParameterDictionary` 要求"每个键被读恰好一次" → 析构时 FATAL。另：`pure_localization` 这个 bool 在 2.0 已 deprecated（设了只会打警告） | **2026-09-21 修复**：改为官方写法 `TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }`，不再设 deprecated 的 bool | **证据**：直接跑 `cartographer_node -configuration_basename cartographer_localization.lua -load_state_filename <不存在的路径>` —— 修复前报上述 CHECK；修复后正常走到 `Failed to open proto stream`（说明 lua 已通过解析）。⚠️ 这条此前一直没暴露：`localization:=cartographer` 因缺 pbstream 从未真跑过，**"launch 能解析"掩盖了它** |
| 16 | **全包形态（`lio:=cartographer`）下 RViz 里"车自己在动、地图跟着小车走"**，Gazebo 里车是静止的 | 全包形态的 `odom→base_link` 由 cartographer 的 pose extrapolator 提供（= 上次匹配位姿 + **IMU 二次积分**）。IMU 的重力对齐残差/零偏被二次积分放大 → **静止也在漂**；而漂移又被当作扫描匹配的初始猜测（搜索窗 ±0.2 m / ±30°）→ 位姿继续偏、地图被拖着走 | **2026-09-21 修复**：给全包形态接一路底盘里程计当运动先验 —— lua 里 `use_odometry = true`，`cartographer_sim.launch.py` 新增 `odom_topic` 参数，bringup 在 `lio==cartographer` 时传 `/odom_ground_truth`（仿真底盘里程计；实车换下位机轮速 odom）。cartographer 只用 odom 的**增量**，所以世界系绝对位姿可直接用 | **实测数据（RMUL2026，车静止）**：`odom→base_link` 漂 x `+3.1 cm/s`、y `−2.5 cm/s`（合 **≈4 cm/s**），yaw `2.21°→1.72°/2.3 s` ≈ **13°/min**；同一时刻 `map→odom` **恒定** `(0.088, 0.075, −0.37°)` → **排除回环问题**；`/odom_ground_truth` 的 twist 全 0 → 车确实没动。修后复测：静止 12.1 s 平移仅 3 mm（0.025 cm/s）、yaw 抖动 <0.5° |
| 17 | RViz 里**整张地图跟着小车转/平移，还留残影**；`map→odom` 从 ≈0 变成 **(0.281, 0.149, 30.88°)** | 两件事要分开：① 若显示用的 **Fixed Frame 不是 `map`**（或 `Target Frame` 选成了 `/base_link`），整个场景会跟着车体坐标系转 —— **地图其实没动**（`/map` 的单元格固定发布在 `map` 系里）；② `map→odom` 出现 30° 量级的**持续修正** = **误回环**：RM 场地小且四角/边线高度对称，而原回环参数偏松（`max_constraint_distance 10 m` ≈ 整个场地、`min_score 0.55`、`global_constraint_search_after_n_seconds 10 s`），跨场地的假匹配命中一次就把整条轨迹拧一下 → 地图整体转 + 每次重画留残影 | **2026-09-21**：① 文档写明先把 `Fixed Frame=map`、`Target Frame=<Fixed Frame>`；② **收紧回环参数**（`cartographer.lua`）：`max_constraint_distance 10→4`、`min_score 0.55→0.72`、`global_localization_min_score 0.6→0.8`、`fast_correlative_scan_matcher` 搜索窗 `5m/20°→2m/10°`、`global_constraint_search_after_n_seconds 10→30`。**判据**：重跑后 `tf2_echo map odom` 应始终 ≈0，转弯/走回起点时最多小幅跳一次 |
| 18 | **RViz 里栅格地图绕着小车转**（车原地转最明显），但 `map→odom` 恒定 ≈0、TF 树也完全正确（`map→odom→base_link` 每帧一个父） | **`lio_tf_adapter` 丢了杆臂**：LIO 的 `/odom` 里 `child_frame_id = "body"`，而 body 就是雷达/IMU 的位置（URDF：`livox_frame = base_link+(0.12,0,0.175)`、`imu_link = +(0.12,0,0.125)`；FAST-LIO/Point-LIO 的 `extrinsic_T = [0,0,0.05]` → **body == imu_link**）。本节点却把 body 的位姿直接改名成 `base_link` 发布（配置 `xyz: [0,0,0]`）→ 车**原地转**时"base_link"绕真实中心画半径 **≈0.17 m 的圆**，而地图静止 → 相对运动看起来就是"整张地图绕着车转/地图跟着车" | **2026-09-21 修复**：`lio_tf_adapter/config/lio_tf_adapter.yaml` 设 `xyz: [-0.12, 0.0, -0.125]`（= `T_body←base_link` 的平移，取负号）。两个 LIO 的 `extrinsic_T` 相同 → 同一套杆臂，改一处即可。**验证**：原地转 360°，`tf2_echo map base_link` 的 x/y 应基本不动（修前会画圆） | **出处**：同根因公开案例 nav2 issue #4135（雷达装偏 → 转圈时 map 跟着画圈）；另 cartographer_ros #1170 说明 **map 帧初始朝向每次重启都不同**、[#254](https://github.com/cartographer-project/cartographer/issues/254) 说明 map 锚在第一个子图上 → 解释 `map→odom` 每次是 +30°/-18°/-130° 这种随机值 |
| 19 | **建图中 `cartographer_node` 偶发 `exit code -6`（SIGABRT），同时 RViz 里整张地图跟着车转** | 仿真雷达插件**混用两套时间基**：`header.stamp` 用**仿真钟**（`node_->get_clock()->now()`），逐点 `offset_time` 却用**墙钟**（`boost::chrono::high_resolution_clock` 累计"生成一帧花了多久"，`livox_points_plugin.cpp:149/198`）。FAST-LIO 的用法是 `lidar_end_time = header 戳 + 末点 offset`（`laserMapping.cpp:396-410`），再拿它当 `odom.header.stamp`（:632）→ **`/odom` 的时间戳 = 仿真戳 + 一帧墙钟耗时**：RTF<1 时是 1.3~3 倍（本机 RTF≈0.76）且逐帧随 CPU 负载抖动。LIO 要处理完一整帧才发 odom → "第 k 帧的 odom"经常在"第 k+1 帧的 scan 已处理"之后才到 → 撞 `pose_extrapolator.cc` 的 `Check failed: timed_pose_queue_.empty() \|\| odometry_data.time >= timed_pose_queue_.back().time` → 直接 abort；没 abort 的时段先验按错时刻套用 → 每帧被拖一下 = 地图跟着车转。另外这个假跨度还会被 FAST-LIO 当帧内运动拿去做**去畸变**，把"其实没发生"的运动补偿掉 → 转起来时整帧被拧、odom 带负载相关偏差 | **2026-09-22 修复**（`livox_points_plugin.cpp`）：① CustomMsg 与 PointCloud2 **共用同一个仿真戳**（原来各调一次 `now()`）；② **`p.offset_time = 0`** —— Gazebo 是"一次回调把所有射线全部打完"（`rayShape->Update()`），帧内没有任何运动，0 才是真值，FAST-LIO 于是得到 `lidar_end_time == header 戳` 自洽；③ 删掉 `boost/chrono.hpp`；④ 加载时打一行 `timebase: SIM clock, per-point offset_time = 0` 便于确认新库生效。**需 `colcon build --packages-select ros2_livox_simulation`，然后重启 launch** | **证据**：`~/.ros/log/2026-09-22-00-53-31-*/launch.log`（起步 51 s 后 `exit code -6`）、`2026-09-22-01-12-34-*`（31 s 后 `exit code -6`）；对 `/opt/ros/humble/lib/libcartographer.a` 做 `strings \| grep "Check failed"`，`pose_extrapolator.cc` 只有两条时间序 CHECK，都是跨时间源乱序；插件 149/198 行确认墙钟；FAST-LIO 396-410/632 行确认取用方式 |
| 20 | 上一轮"修地图跟着车转"的**方向错了**：把位姿图回环门槛收紧（`min_score 0.55→0.72`、`global 0.6→0.8`） | 收紧后收尾日志变成 `Score histogram: Count: 0`（`0 computations resulted in 0 additional constraints`）= **一条回环都没命中** → 位姿图完全没有全局锚定，地图只剩 local SLAM 的结果；而被调的门槛属于"已经不工作的东西"，所以怎么调都没用。另外 `use_odometry=true` 把 LIO 的 `/odom` 当先验是**同源冗余**：`/odom` 本来就由同一份雷达点云融合而来，喂回去等于把 LIO 自身的漂移反馈给 cartographer。上游官方"只有 2D 激光"的参考配置 `revo_lds.lua` 就是 `use_odometry=false` + `use_imu_data=false` + `use_online_correlative_scan_matching=true` | **2026-09-22 修复**（`c4a4fd1`）：`cartographer.lua` 改回 `use_odometry=false`（纯 2D 扫描匹配）；回环门槛回到上游 `min_score 0.65` / `global_localization_min_score 0.70`；真正挡"跨场地误回环"的 `max_constraint_distance 4m` 与搜索窗 `2m/10°` 保持收紧。`odom→base_link` 仍由 `lio_tf_adapter` 发，TF 契约不变 | **证据**：收尾日志 `Count: 0`（见 `~/.ros/log/2026-09-22-00-53-31-*/launch.log` 尾部）；[revo_lds.lua](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/configuration_files/revo_lds.lua)；相关公开案例 [cartographer #579](https://github.com/cartographer-project/cartographer/issues/579)、[cartographer_ros #1051](https://github.com/cartographer-project/cartographer_ros/issues/1051) |
| 21 | **cartographer "地图跟着车转"：先验给错了源**（#20 的后续；#19 修的是时间基） | `/scan` 有洞（实测 2.8 Hz、每 ~0.5 s 一个）时，**洞期间必须有东西把位姿推过去**，否则下一帧扫描的初值偏几十度 → 子图按错的旋转角铺下去 = 整张图跟着车转。cartographer 本来就是用 odom 干这件事的，但我们喂的是 **LIO 自己的 `/odom`**：① **同源**（与 `/scan` 来自同一份点云）→ 反馈回路，LIO 漂移直接拖地图；② **晚到**（LIO 要处理完一整帧才发）→ 撞 `pose_extrapolator` 时间序 CHECK → `exit -6`。所以"干脆关掉"（#20）与"用 LIO odom"两个方向都不对，正确的是**换一路独立的里程计** | **2026-09-22 修复（路线①）**：`cartographer.lua` 恢复 `use_odometry = true`，同时把先验源换成**独立底盘/轮速里程计** —— `bringup_sim.launch.py` 的**标准形态**（`mapper:=cartographer` + 有 LIO）也传 `odom_topic:=/odom_ground_truth`（原先只有全包形态传）。仿真 = `gazebo_ros_planar_move` 的底盘 odom（10 Hz、物理插件按自己的时刻产出、**立即到达**）；**实车 = 下位机轮速 odom**（这一条可迁移）。`odom→base_link` 仍由 `lio_tf_adapter` 发，TF 契约不变 | **预期证据（重跑核对）**：① 不再出现 `exit code -6`；② `tf2_echo map odom` 始终 ≈0；③ 收尾 `Score histogram` 不再是 `Count: 0`（回环真的命中）；④ 原地转一圈地图不跟转。**台架偏差**：`planar_move` 的 odom 是**无打滑的理想值**，比实车容易 → 记在 `sim_real_contract.md` §三（后续用打滑/延迟注入槽位补） |
| 22 | **地图"跟着车转 → 又被矫正回去 → 留下残影"**（路线①之后的残余问题）。实测 `map→odom` 的 yaw 呈锯齿：从 ~1° 慢慢涨到 **17°** 再被拉回 ~1°，而平移只有 5~16 cm | 官方调参文档描述的正是这一现象：**局部 SLAM 在一个子图内部走偏 → 全局 SLAM 事后只能部分矫正 → 而"坏掉的子图永久保留"** → 残影。我们的具体成因：先验已换成真值底盘 odom（残差≈0），但**实时相关扫描匹配器的搜索窗仍是"没有可用先验时代"的 ±30° / 0.2 m** —— 它允许匹配器一步跳到**对称场地**的错误朝向 → 局部位姿相对优化轨迹缓慢走偏 → 位姿图周期性拉回（锯齿）→ 子图与矫正后的轨迹不一致（残影）。同时 `ceres_scan_matcher` 的权重还是上游默认 10/40，对"偏离先验"的惩罚不足 | **2026-09-22 修复（四次修正）**：① `real_time_correlative_scan_matcher.linear_search_window 0.2→0.1`、`angular_search_window 30°→5°`（依据 #534 的经验法则：窗口 ≈ 一帧内最坏里程计漂移 +10~50%；**有先验时窗口就该很小**）；② `ceres_scan_matcher.translation_weight 10→1e2`、`rotation_weight 40→4e2`（官方 tuning 文档的示例值：先验越可信，越要 penalize 偏离先验）。**若仍不干净 → 下一步诊断（官方方法论）**：把 `POSE_GRAPH.optimize_every_n_nodes` 临时设为 **0**（关掉全局 SLAM）单独看局部 SLAM —— 地图不再"转回去"（只剩缓慢漂移）⇒ 问题在全局矫正；仍然跟着转 ⇒ 问题在局部 SLAM | **证据**：`ros2 run tf2_ros tf2_echo map odom` 实测锯齿（sim time 769→781：yaw 8.56°→11.36°→14.17°→16.96°→**2.02°**→4.68°→0.51°→…；同时 `/odom_ground_truth` 的 `Subscription count: 1 = cartographer_node` ✓、`exit code -6` 已消失 ✓）；[官方 tuning 文档](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html)（"The broken submap is broken forever though" + CeresScanMatcher 权重示例）；[cartographer #534](https://github.com/cartographer-project/cartographer/issues/534)（Mofef：odometry + 实时相关匹配器 + **很小的搜索窗**） |
| 23 | **`map→odom` 被高频甩动**：50 Hz 采样实测 `|Δyaw|` 中位 0.333°、**P95 1.938°**、max **83.2°**、跳变 143 次、峰峰值 83°、单向漂移 **−13.7°/min**；现象是"原地转+走一点点还行，一旦移动地图立刻飞" | 单样本 P95 ≈ 1.94° @50 Hz ⇒ 等效 **≈97°/s 的连续转动**，再夹 83° 级别的**单样本大跳** —— 这**不是"矫正"**（正常矫正应是"很久才跳一次、每次几度"）。最符合的机制是**帧间没有旋转来源**：`/scan` 只有 ~2.8 Hz 且每 ~0.5 s 一个空洞（相邻两帧可差几十度），而我们当时把 `use_imu_data` 关掉了 —— 2D 且无 IMU 时 `PoseExtrapolator` 的旋转外推只能靠 odom+位姿历史，cartographer 维护者/贡献者在 #534 里明确指出这个组合在 2D 下不可靠。**对照组：slam_toolbox 没有这个问题**，因为它从 TF（`odom→base_link`，lio_tf_adapter 发）直接取运动模型，不走 cartographer 的 IMU-tracker 路径 | **2026-09-22 修复（五次修正）**：`TRAJECTORY_BUILDER_2D.use_imu_data = true`。当初关它是为了躲 `pose_extrapolator.cc:229 Check failed: time >= imu_tracker->time()`，而那个崩溃的根因是①**插件时间基 bug**（`cb24ba8` 已修）②**LIO odom 晚到**（已换成独立底盘 odom，`34c64a6`/`9a9b580`）→ CHECK 的前提消失。`tracking_frame=imu_link` 本来就是为 IMU 共位 CHECK 选的，正好用上 | **证据**：`python3 tools/monitor_map_odom.py --csv /tmp/mo.csv` 的 112 s 记录（判读=高频抖动）；IMU 接线已核（`sentry_robot_sim.xacro:246-258`：`frame_name=imu_link`、`~/out:=/livox/imu`、100 Hz、`imu_joint` rpy=0）；**待重跑核对**：P95 应降到 ≈0.1° 量级、不再有几十度大跳。⚠️ **同时必查（30 秒，排除环境因素）**：`ros2 node list` 有没有残留的 `static_transform_publisher`（T1 桥会发 `camera_init→map` 与 `body→odom`，一旦残留就让 `map`/`odom` **多父边** → 查找在两条路径间跳，症状与本文完全一致）；`ros2 topic echo /tf_static --qos-durability transient_local` 看静态边清单；`ros2 run tf2_tools view_frames` 看每帧父边数 |
| 24 | **打开 IMU 后 cartographer 立刻 FATAL**：`imu_tracker.cc:67 Check failed: (orientation_ * gravity_vector_).z() > 0. (0 vs. 0)`（栈：`ImuTracker::AddImuLinearAccelerationObservation ← PoseExtrapolator::AdvanceImuTracker ← AddPose ← Node::PublishLocalTrajectoryData`）。现象：`Added trajectory` → `Inserted submap (0,0)` → 1 帧后 abort，建图完全起不来 | `ImuTracker` 首个加速度样本的 `alpha = 1 - exp(-delta_t / imu_gravity_time_constant)`，而首帧的 `delta_t = now - Time::min()` 极大 → **alpha≈1 → `gravity_vector_` 被直接赋成"第一帧 IMU 的 linear_acceleration"**；只要那一帧是 `(0,0,0)`，随后 `(orientation_*gravity).z() > 0` 必失败。**与"FAST-LIO 用同一路 IMU 没问题"并不矛盾**：FAST-LIO 是**多帧求均值**做重力初始化，天然容忍启动几帧零值；cartographer 拿**第一帧**当基准。⚠️ 同一次日志里还有强线索：`ordered_multi_queue.cc:172 All sensor data for trajectory 0 is available starting at '621355974448820000'` —— `621355974.448820 s` = **墙钟 .NET ticks 被当成纳秒**（`.NET 1970 纪元 621355968 s + 仿真时间 6.448820 s`），说明 scan/imu/odom 三路里有**一路的时间戳不是仿真钟**（与插件那次同源的时间基问题，待三路 echo 确认） | **2026-09-22 处置（六次修正）**：`use_imu_data` 先改回 **false** 解锁建图；同时把"四次修正"改过的局部匹配参数（5° 窗口、1e2/4e2 权重）**恢复上游默认**（0.1/20°、10/40）—— 21:0x 的实测证明"更信任先验 + 小窗口"没让 map→odom 变稳（P95 1.94°、max 83°），反而可能剥夺匹配器的纠正能力。**打开 IMU 的前置条件**：① `/livox/imu` 前几帧的 `linear_acceleration` 不是 `(0,0,0)`；② 三路 + `/clock` 时间戳同轴（用下面的命令核） | **待核（30 秒）**：`ros2 topic echo /livox/imu --field linear_acceleration \| head -20`（重启 launch 时立刻抓，看前几帧是不是零）；`ros2 topic echo /clock --once`、`/livox/imu`、`/scan`、`/odom_ground_truth` 各抓一次 `header.stamp` 对比（有没有 `6213559xx` 这种值）。FAST-LIO 侧佐证：同一次日志里 `IMU Initial Done` 正常（说明均值可用） |
| 25 | **`/segmentation/obstacle` 只有 0.31 Hz、单次空洞最长 20.8 s（仿真时间），而 CPU 92% 空闲** —— 直接后果：`/scan` 长期 1.7~3.3 Hz 且带秒级空洞 → cartographer 在扫描之间"朝向不转、每来一帧扫描补一刀"（`map→odom` 84 次正向大跳、累计漂 66°、单步最大 76.8°）→ 表现就是"地图跟着车转 / 地图飞"（这是 #23 的上游原因） | 分层实测（同一个 192 s bag，全部按**仿真时间**算）：① 插件 `/livox/lidar/pointcloud` = **10.00 Hz、0 空洞**；② linefit `/segmentation/obstacle` = **0.31 Hz、49 个 >0.5 s 空洞、最长 20.8 s**；③ p2l `/scan` = **68 = 68**（一帧不丢）⇒ **丢帧点唯一：linefit**。但**不是算力**：用 `tools/seg_bench_offline.cc` 把录到的真实点云离线喂给 linefit 核心库，**`segment()` 平均 1.01 ms/帧、最慢 1.80 ms**（30000 点、完全相同的 sim 参数）。⇒ 卡的是**大点云的订阅交付**：**同一个发布端，reliable 订阅者（rosbag2 录制）拿到 2182/2182 = 100%**，而 **best-effort 订阅者（linefit 的 `SensorDataQoS`、`ros2 topic hz`）只拿到 3%~26%**（隔离回放里 `topic hz` 实测 2.6 Hz、最大空洞 5.9 s，linefit 60 s 内一帧未出）。原理：480 KB 的 `PointCloud2` 必须分片，best-effort 不重传 ⇒ 丢一个分片整帧丢；再叠加 `SensorDataQoS` 的 KEEP_LAST(5)，一卡就雪崩。**对照组**：FAST-LIO 订同一个插件的大 `CustomMsg` 用的是**默认 RELIABLE(depth 20)**，10 Hz 一帧不丢 | **2026-09-22 已修（管道类，1 行）**：linefit 订阅由 `rclcpp::SensorDataQoS()`（BEST_EFFORT, depth 5）改为 **`rclcpp::QoS(rclcpp::KeepLast(10)).reliable()`**，与插件发布端对齐（= FAST-LIO 的做法）；`colcon build --symlink-install --packages-select linefit_ground_segmentation_ros` 通过。**验证**：`ros2 topic info /livox/lidar/pointcloud --verbose` 里 linefit 的 Reliability 应变成 RELIABLE；`ros2 topic hz /segmentation/obstacle` 应从 0.3 Hz 升到 ~7.5 Hz（墙钟）。⚠️ **更正**：我早先说"`/segmentation/ground` 在发 426 KB"是把 ground/obstacle 搞混了 —— 离线实测 ground 只有 ~700 点（≈11 KB），**不构成带宽问题，因此不动它**；真正的浪费在 obstacle 云（26649 点里 88% 是插件发的 `(0,0,0)` 假点，≈426 KB/帧）。**另有仿真保真度问题（需拍板）**：插件把**无回波射线**也填成 `(0,0,0)` 点发出去 —— 实测**每帧 78.4%**（23500/30000）是这种假点；真实 Livox 驱动只发有效回波。后果：障碍云 26649 点里 88% 是假点（带宽/CPU 白烧）、地面只剩 ~700-900 点（89~98% 被判障碍）。修法 = 出帧时跳过无回波射线（CustomMsg/PointCloud2 都不发） | **证据**：`python3 tools/analyze_slam_bag.py --bag .tmp_bags/startup`（逐段率/空洞）；`tools/seg_bench_offline.cc`（1.01 ms/帧）；`ros2 topic info /livox/lidar/pointcloud --verbose`（RELIABLE 发布 / BEST_EFFORT 订阅）；`ros2 topic hz /segmentation/obstacle` 实测 0.45~1.04 Hz、max 4.232 s；bag 内 2182 帧的 `(0,0,0)` 点占比 ≈78.4% |
| 26 | **内部小墙"扫到就有、被挡住/角度扫开就没了"**（外侧大墙不受影响） | cartographer 的 2D 栅格是**每帧投票**的概率图：命中 +1（`hit_probability`）、穿过 −1、"**无回波**"则把 **0~`missing_data_ray_length`** 整段标为**自由**。本链路 `/scan` 里 **`inf` 占 37.3%**（1462 bin 中 546 个）—— 来源是"地面被 linefit 去掉"+"MID360 上视射线打空"，都属物理正常。我们原来 `missing_data_ray_length = 3.0` ⇒ **每帧有 1/3 的方向在擦 3m 内的地**。实测命中距离 **P25=1.8 / P50=2.3 / P75=3.1 m**、4~10m **仅占 16%** ⇒ **约 3/4 的命中都落在 3m 常清区内**，所以"死的是内部小墙"；外墙多在区外、命中票也多，故留得住。再叠加命中稀疏（每帧 ~3100 障碍点 / 1462 bin，且车一转同一面墙落进不同 bin）而清除票每帧必到 ⇒ 概率在阈值附近摆动 = 闪烁 | **2026-09-22 修复（七次修正，一组变量）**：`cartographer.lua` —— `missing_data_ray_length 3.0 → 1.0`（常清区缩到贴身，不再擦 1m 外的墙）、`hit_probability 0.55 → 0.62`、`miss_probability 0.49 → 0.45`（命中更粘、清除更弱）。**保持 `insert_free_space = true`**（只调弱清除，不取消，否则失去清动态物的能力）。`num_accumulated_range_data`（1→3，稀疏特征多帧累积）**留作下一组变量** | **证据**：`bag` 内 `/scan` 实测 —— 有回波 62.7%（`inf` 37.3%）；距离分位 P5=1.0/P25=1.8/P50=2.33/P75=3.12/P95=5.56m；0.2~1m 5%、1~2m 32%、2~4m 47%、**4~10m 16%**。机制与处置详见 `docs/debug_fastlio_cartographer.md` §5.2。**⚠️ 2026-09-22 二次结论：七次修正已回退（方向错）**。用 8 帧 bag 逐带统计（**已排除插件发的 `(0,0,0)` 假点**，地面按实测 `z=-0.226m` 标定）：raw→障碍保留率 —— 0~3cm **0.1%**、3~8cm **2.8%**、8~15cm 59.1%、15~30cm 43.7%、30~60cm 92.8%；即 **`linefit` 把 ≤8cm 的低矮特征基本删光**（`max_dist_to_line=0.1` 的 10cm 容差 + **`sensor_height=0.275` 而实测雷达离地仅 0.226cm→差 4.9cm**），之后 p2l 的"每角度 bin 只留最近点"再砍 60~90%（近处残余遮住远处）。另有**几何**原因：MID360 下视 FOV 仅 −7°，高 h 的墙需 d ≥ (0.226−h)/tan7°（h=0.15m→0.62m）⇒ 越近越看不见（真机同样）。**下一步**：先修数据侧（`sensor_height 0.275→0.226`、`max_dist_to_line 0.1→0.05`，感知参数待拍板），验收=重录 bag 量"低 3~8cm"保留率是否显著上升；确认低墙进 scan 后再回来谈栅格清除/累积 | **2026-09-22 三次结论（已改感知侧）**：`segmentation_sim.yaml` 的 `sensor_height 0.275→0.226`（实测标定）、`max_dist_to_line 0.1→0.05`；这两个是 linefit 的地面分割参数，**与 FAST-LIO 无关**（不同节点/话题/无共享状态），也不改"什么算障碍"的语义。`analyze_slam_bag.py` 新增 **⑤ `/map` 留存分析**（留存曲线 + occupied→free 闪烁统计 + 栅格取值分布），用来量化"留不住"并分辨"被主动擦除"（⇒ 调 cartographer 清除/命中）还是"数据没进来"（⇒ 修感知/几何）；录制时请把 `/map` 也录上。**几何那条（MID360 下视 FOV −7°）任何参数都修不了**：矮墙"离得够远才看得见"，真机同样 ⇒ 建图时别贴着矮墙走 | **2026-09-22 四次结论（已量化，判定"被主动擦除"）**：`.tmp_bags/ret`（222 帧 `/map`）实测 —— 曾占据 3779 格、结束仍占据 **49%**（**51% 被擦掉**）；留存曲线 +2/+5/+10/+20 帧 = 87.5/74.4/63.9/**63.8%**；闪烁（occupied→free）中位 2、P95 6、max 12，**87% 的曾占据格子都闪过** ⇒ 判为**被主动擦除**（每帧 37% 无回波光束 × `missing_data_ray_length=3.0` 把 0~3m 标自由），不是"数据没进来"。同时确认：`/scan` 已 **10.00Hz/最大空洞 200ms**（QoS 修复彻底生效）、`map→odom` 峰峰值 7.04°/单步 2.54°（定位稳）。**处置**：临时置 `insert_free_space = false` 做判别实验（预期被擦掉→~0、留存→≈100%），确认后改为`missing_data_ray_length 3.0→0.5~1.0` + `hit/miss 0.62/0.45`，并把 `num_range_data`/`optimize_every_n_nodes` 由 30 调回上游 90（扫描率提高 10 倍后，30 会让子图每 3 秒就换一个，重叠缝增多） | **2026-09-22 五次结论（判别实验已完成 + 已按平衡点修改）**：`insert_free_space=false` 诊断跑（`.tmp_bags/ret2`，183 帧 `/map`）—— 结束仍占据 49%→**89%**、被擦掉 51%→**11%**、留存 +20 帧 63.8%→**78.9%**、闪烁≥1 由 87%→54%，但 **自由格子(0~30) 35856→0** ⇒ 关掉清除连自由空间一起没了（"栅格一点点出来很艰难/全灰"就是这个）。⇒ 机制确认：**留不住=被清除**，但正确处置是"保留清除 + 调弱"：`insert_free_space=true`、`missing_data_ray_length 3.0→1.0`、`hit/miss 0.62/0.45`，并把 `num_range_data`/`optimize_every_n_nodes` 30→**90**（上游默认；10Hz 扫描下 30 = 每 3 秒一个子图/一次优化，重叠缝暴增）。**复测判据**：留存 +20 帧 ≥90%、被擦掉 <10%、且**自由格子 >0 且明显增多**（灰色变少） | **2026-09-22 六次结论（再定量 + 九次修正）**：逐帧重建 p2l 的每-bin 选点与高度后测得 —— 低矮特征(离地3~15cm)的 2D 格子每帧 **90.4% 被命中**，只有 **36.3% 会被"命中高度>25cm 的光束从上方穿过"**（命中:清除 ≈ 1:0.4）⇒ **主导清除是 37% 的无回波光束 × `missing_data_ray_length`**；关键认识：真实 2D 雷达"无回波=空"成立，但 **3D FOV(−7°~+52°) 转 2D 时无回波多来自"朝天打空"** ⇒ 假设不成立 ⇒ 半径必须小。另：`num_accumulated_range_data` 不改变命中/清除**比值**，不是这里的旋钮。现场观感为"墙变淡/变灰、慢慢化掉"（清除仍压过命中）⇒ **九次修正**：`missing_data_ray_length 1.0→0.5`、`hit 0.62→0.68`、`miss 0.45→0.40`（odds 比 1.27→2.0→**3.2**；清除面积约 1/6）。复测仍用第 ⑤ 段：留存 +20 帧 ≥90%、被擦掉 <10%、自由格子 >0 | **⚠️ 2026-09-22 七次结论（九次修正失败 + 回滚；教训：一次只改一项）**：九次修正把 5 项一起改了（`missing_data_ray_length 1.0→0.5`、`hit 0.62→0.68`、`miss 0.45→0.40` **加上** `num_range_data 30→90`、`optimize_every_n_nodes 30→90`），实测（`.tmp_bags/ret3`，110 帧）**留存反而从 49% 掉到 30%**、被擦掉 70%、**曾占据格子 3779→2291**（连"曾占据"都变少 ⇒ 是**证据变少**而非清除变强）。指向我多加的两项（子图/优化频率 30→90：子图覆盖范围变大后，超出子图栅格的数据会被裁掉，墙的证据成片减少）⇒ **已回滚到 30/30**，保留清除改弱那三项。**教训：一次只改一项、其余保持不动**。**另一条可选路线**：若只要墙、不在意自由空间，`insert_free_space=false` 实测留存 89%（ret2），**2026-09-22 八次结论（解耦，取代"不清除"路线）**：栅格里"擦旧墙"与"标空地"是**同一个写**（都给射线途经格写 miss），所以开关二值 ⇒ 关掉就没自由空间（自由格=0、全灰）。但**强度可解耦**：白格真正来自**打到东西的射线**（真观测过的空地），而**无回波光束**（占 37%，多来自朝天打空）标的 0~`missing_data_ray_length` 只是**假设**、没观测过，是**乱擦墙的元凶**。⇒ **十次修正**：保留 `insert_free_space=true`，把 `missing_data_ray_length 0.5→0.05`（≈关掉假想空地）。预期：留存 ≥ 不清除时的 89% 且**自由格子 >0**（两全）。代价是自由格子=0（地图全灰），导航侧需把 `track_unknown_space` 关掉/`allow_unknown: true` 才可用 |





---

## 二、静默失效类（最危险：不报错但不生效）

| 现象 | 根因 | 修复 |
|---|---|---|
| 参数写在 yaml 里却"没生效" | **节点名跨版本改名**：Galactic `recoveries_server` → Humble **`behavior_server`**；`recovery_plugins` → `behavior_plugins`；插件类型 `nav2_recoveries/*` → `nav2_behaviors/*`。对不上的整段被**静默忽略** | 三份 sim 变体已改名并把 `robot_base_frame` 改回 `base_link_fake`、`max_rotational_vel: 3.0`。**佐证**：日志里创建了 4 个插件（=代码默认列表），而 yaml 只写了 2 个 |
| **建图模式下 `/map` 被旧图抢占** | `map_server` 的启动条件只判断 `localization`（建图模式为空 → 条件成立）→ 它把磁盘旧 pgm 发到 `/map`，与 slam_toolbox 抢；`map_saver_cli` 可能存下**旧图** | 条件加 `mode=='nav'`（已修，5 种组合验证） |
| `/map_save` 无处可写 | `fastlio_mid360_sim.yaml` 里 `map_file_path` 被注释 → ICP 在 RMUL2026 上没有底图 | bringup 按 `world` 自动注入 `PCD/<world>.pcd`（与 ICP 的 `pcd_path` 同路径） |
| `nav_rviz`/`lio_rviz` 都给 `False` | 屏幕上一个可视化都没有（曾误判"没有机器人"） | runbook §0.4：mapping 用 `lio_rviz`，nav 用 `nav_rviz`（默认 True），**只开一个** |
| `velocity_smoother.odom_topic: "Odometry"` | T2 已把里程计统一为 `/odom`，旧键名是遗留 | 改为 `odom` |
| **cartographer 的栅格被硬 remap 到 `/cartographer_map`** | nav2 `static_layer`、`map_saver_cli`、RViz 的 Map 显示项都订阅 `/map` → 边建边导 `mapper:=cartographer` 时 costmap 无静态图、建图落盘存不到东西 | **2026-09 修复**：`cartographer_sim.launch.py` 默认发 `/map`；纯定位另有 `map_server` 时用 `occupancy_grid_topic:=/cartographer_map` 覆盖 |
| **纯建图 `mode:=mapping` 下 `nav_rviz:=True` 却没有任何 RViz** | 三形态拆分后 `mapping` 不再启动 nav2，而 RViz 原先由 nav2 的 `rviz_launch` 带起 | **2026-09 修复**：bringup 为 `mode=='mapping' and nav_rviz=='True'` 单独补一块 RViz（`nav2.rviz`） |
| **`global_obstacle:=scan` 时 global 图累积幽灵障碍** | 新增该槽位时把 local 的 `obstacle/raytrace_max_range: 6.0` 照搬到 global；而 global 是**全图**（13×10 m），raytrace 只能清 6 m 内的旧标记 | **2026-09 修复**：改为 **10.0 m**（与 `p2l` 的 `range_max` 对齐）——「扫描能看到多远，就要能清多远」 |
| **T6 撤销**：`base_link_fake` 不是脏帧 | `fake_vel_transform` 20 Hz 发 `base_link→base_link_fake`（含云台转角），并做 `/cmd_vel → /cmd_vel_chassis` 旋转；角速度非零时按 `spin_speed` 原地转底盘 = **哨兵小陀螺**。改成 `base_link` 会丢功能 | `docs/tf_interface_contract.md` 已撤销 T6 并写明理由 |
| **感知链断掉 → costmap 冻在最后一帧：无报错、不停车（静默失效）** | ① nav2 障碍源的 `expected_update_rate` 默认 **0 = 不检查**（`observation_buffer.cpp` 的 `isCurrent()` 直接 return true）；② `ObservationBuffer` 在 `observation_keep_time=0` 时**永远保留最后一条**（`purgeStaleObservations()` 只 `erase(++begin, end)`）→ 每轮 costmap 更新把**同一帧旧点云**重新 mark；③ `/scan` 是**串行单点**（插件 → `linefit` → `p2l`），local 没有第二来源 | **2026-09 修复**：① 所有障碍源加 `expected_update_rate: 0.5` → 源停即 `current_=false` → `controller_server.cpp` / `planner_server.cpp` 拒绝算速度 → 已有 `velocity_smoother.velocity_timeout: 1.0` 发零速停车 + 日志 WARN；② 新增 `local_obstacle:=scan\|cloud\|both` 槽位（`cloud` = 点云直投，不经 `p2l`，作第二来源）。验证：`pkill -f pointcloud_to_laserscan` 后应出现 WARN 且 ~1 s 内停车 |
| **`p2l` 的 `range_min: 0.45` ⇒ 贴身 45cm 既看不见、又被清成 free** | 小于 `range_min` 的点被 **丢弃**（`pointcloud_to_laserscan_node.cpp` 的 `continue`），该角度 bin 保持 `inf`；`inf_is_valid: true` 时 nav2 把 `inf` 换成 `range_max-ε`（=10 m，`obstacle_layer.cpp` 的 `laserScanValidInfCallback`）→ 沿射线**一路 clear**。原值 0.45 的理由是"避开 38cm 地面最近点"，但 `p2l` 的输入已是 `linefit` **去地面后**的 `/segmentation/obstacle`，该理由不成立 | **2026-09 修复**：`range_min: 0.45 → 0.2`（与车体半径 0.20、`linefit` 的 `r_min` 对齐 → 盲区缩到车体内部）；同时 `scan_time: 0.3333 → 0.1`（与 10 Hz 传感器一致） |
| **`missing_data_ray_length` 在本链路里是空转的（八~十次修正全白改）** | 逐行读源码：它只在 `local_trajectory_builder_2d.cc` 的 `range > max_range` 分支生效（把超距回波截到该距离当 miss）。而 `/scan` 的 `range_max = 10.0`（p2l）< `TRAJECTORY_BUILDER_2D.max_range = 12.0` ⇒ **该分支永不执行** ⇒ `range_data.misses` 恒空。另外 p2l `use_inf:true` 把无回波 bin 发成 `inf`，`msg_conversion.cpp` 的 `LaserScanToPointCloudWithIntensities` 用 `range_min <= r <= range_max` 过滤 ⇒ **`inf` 直接丢弃**（既不命中也不清除） | **2026-09 十一次修正**：`cartographer.lua` 注释已改写并标注"别再拿它当旋钮"；真正的旋钮是 `hit_probability` / `num_range_data` / `insert_free_space`（见 `docs/debug_fastlio_cartographer.md` §5.2.4） |
| **`/map` 的取值上限是 75，永远不出现 100** | `msg_conversion.cpp::CreateOccupancyGridMsg` 的 `value = round((1 - color/255)*100)`，而 `color` 是 `DrawToSubmapTexture` 的 `delta = 128 - ProbabilityToLogOddsInteger(P)` 在**暗红底上预乘合成**的结果 ⇒ 实测映射：`P=0.5→50、0.68→58、0.80→65、0.90→75`。**任何 `lethal_threshold: 100` 之类"取满值"的消费者永远看不到障碍** | 阈值按 `>=65`（= P>=0.80）判读；§5.2.4 的所有"占据"统计都用这个口径 |
| **★ 墙"留不住"的真凶：本项目把 `miss_probability` 从上游 0.49 调到了 0.40，单张清除票强度是上游 ~10 倍（命中/清除比 5.0→1.9）；又叠加 `num_range_data 90→30`（证据窗口砍到 3 秒）** | upstream `trajectory_builder_2d.lua`：`hit 0.55 / miss 0.49 / num_range_data 90`，hit:miss 的 log-odds 比 = `+0.201 : −0.040`（**1 次命中顶 5 次清除**）；本项目 `0.68/0.40` = `+0.754 : −0.405`（1 次命中只顶 1.9 次清除）。而"调猛清除"的理由是**已证伪**的"无回波 × `missing_data_ray_length` 乱擦墙"。另外 `nrd=30` 让"自由空间"也长不出来（`0.68/0.49/30` 只剩 31 个自由格）——这才是上一轮误判"弱清除会让地图没空地"的原因 | **2026-09 十二次修正**：`miss 0.40 → 0.49`（回上游）+ `num_range_data 90 → 300`（30 秒窗口），`hit` 保持 0.85。离线同轨迹（ret4）：留存 +2 帧 69.1%→**94.3%**、+20 帧 42.2%→**75.2%**、闪烁中位 3→**1**、实心墙格子 1087→**2585**、自由格 7664→6556（**不损失**）。即"`insert_free_space=false` 的 89% 效果"与"白格自由空间"同时拿到 |

---

## 三、地图资产与坐标系（含"幽灵墙"完整证据链）

| **★ 墙"运行久了就没了"：上游 2D 概率栅格没有任何"已占据格子免清"机制，`insert_free_space` 又是全有/全无** | 逐行源码：`insert_free_space=true` 时对每条 return 都 `RayToPixelMask(origin→hit)` 并把 miss 表写满沿途每个格子；`false` 时在 miss 循环前直接 `return`。没有中间档、没有"别擦已占据格子"的开关。参数只能拉长时间常数（离线长时程：最好的 weak-clear 配置 +2 帧 94% → **+100 帧 46%**；`insert_free_space=false` 才 100% 但自由格=0） | **2026-09 十三次修正**：工作区内 fork 核心 `src/localization/rm_localization/cartographer/`（`third_party/` 保持原样），插入器新增 `min_probability_to_clear`（P≥阈值免清，0=上游行为）。离线同轨迹预测 `0.68/0.49/3000/0.80`：**留存 +2/+20/+100 帧 100%/99.9%/100%、闪烁 0、自由格 5037、占据 4794**。副作用见 `PATCH_README.md` 。**2026-09-23 现场确认：用户反馈"这个地图是可以的"（墙不再随时间化掉）** |
| **nav2 陷阱：`static_layer` 的 `lethal_cost_threshold` 默认 100 + `trinary_costmap: true`，而 cartographer 的 `/map` 上限只有 75 ⇒ 每一面墙都被判成 FREE_SPACE** | `msg_conversion.cpp::CreateOccupancyGridMsg` 的 `value = round((1-color/255)*100)`，`color` 经暗红底预乘合成后上限对应 P=0.9 ⇒ 出图最大 **75**；nav2 `static_layer` 用 `map >= lethal_cost_threshold` 判致命 | 用 cartographer 的图喂 costmap 时设 **`lethal_cost_threshold: 60`**（社区 issue #628 用的就是 60）。本仓库 `nav2_params*.yaml` 尚未设置，属待办 |

### 3.0 两个都叫 `odom` 的坐标系并不一致（**建图图/导航图对齐的坑**）

| 来源 | 出生点处数值 | 性质 |
|---|---|---|
| `/odom_ground_truth`（话题，`frame_id: odom`） | `x=4.30, y=3.35` | **世界坐标**（= 出生点在世界里的位置） |
| `/tf` 的 `odom→base_link` | `x=-0.106, y=0.007` | **出生点相对系** |

而 cartographer 的 `map` 系 = **出生点相对系**（实测 `map→odom ≈ 恒等`，SLAM `/map` 的内容整体比世界坐标偏 `−(4.3, 3.35)`）。
⇒ ① 离线复现必须换算系（`tools/replay_scan_grid.py --pose tf` 默认用 `/tf` 复合，**别用 `gt`**）；
② **cartographer 建出来的图与 `map/RMUL2026.pgm`（世界系，`origin: [2.68, 0.228]`）天然差 `(4.3, 3.35)`** ——
纯建图落盘、替换底图、给 `amcl_init_x/y` 初值时都要意识到这一点。

---

## 三、地图资产与坐标系（含"幽灵墙"完整证据链）

### 3.1 三场地的 map 系约定不同（**关键坑**）

判定方法：**场地 STL 的世界包围盒** vs **pgm 已知区域（非 205 像素）包围盒**，尺寸 + 位置双证据：

| world | 场地 mesh 世界 bbox | pgm 已知区域 bbox | 判定 | **AMCL 初值** |
|---|---|---|---|---|
| `RMUC` | x[0,29.20] y[0,15.20] | x[-6.35,22.50] y[-7.60,7.45] | mesh−spawn(6.35,7.6) ≈ pgm → **出生点系** | **(0,0,0)** |
| `RMUL` | x[0.51,14.23] y[-1.25,9.30] | x[-3.75,9.80] y[-4.49,5.96] | mesh−spawn(4.30,3.35) ≈ pgm → **出生点系** | **(0,0,0)** |
| `RMUL2026` | x[2.20,14.80] y[0.20,8.80] | x[2.68,14.68] y[0.23,8.43] | mesh **直接**等于 pgm → **世界系** | **(4.3,3.35,0)** |

→ 已实现 **按 `world` 自动注入 AMCL 初值**（`set_initial_pose: true` + `RewrittenYaml` **全路径** `amcl.ros__parameters.initial_pose.*`，避免叶子短名误伤）。

### 3.2 RMUL2026 的"幽灵墙"（规划失败的根因）

- `RMUL2026.pgm` 在 **x≈5.2** 有一条 **1 像素宽竖直虚线**（y≈1.8→6.0，带小缺口）；
- **RMUL2026 世界网格在该处零顶点**（x∈[5.05,5.35]、y∈[2.8,4.0] 顶点数 = **0**，任何 z）→ 世界不存在这道墙；
- 经 `robot_radius=0.2` 内切膨胀后走廊被封死：**r=0.20 目标不可达 / r=0.15 可达**；
- 地图被切成 **3 块连通域**（24246 / 3692 / 693 格），目标点落在另一块 → 必然 `failed to generate a valid path`；
- 结论：该 pgm 更像**官方场地平面图**（含分区虚线），不是 sim 建图产物 → **应在 sim 里重建**（见 §六）。

### 3.3 "一开始就有完整地图"是正常的

nav 模式的地图来自 `map_server` 加载的**磁盘既有 pgm**（`src/bringup/rm_nav_bringup/map/<world>.pgm`），不是实时扫描。实时建图要 `mode:=mapping`。

---

## 四、架构改动与新增工具

### 4.1 `mode` 拆成三种场景形态（"地图从哪来"是分界线）

| `mode` | 中文 | 在线 SLAM | nav2 | `map_server` | 重定位 | `map→odom` 来源 |
|---|---|---|---|---|---|---|
| `mapping` | 纯建图 | ✅ | ❌ **不启动**（省 CPU） | ❌ | ❌ | 在线 SLAM |
| `slam_nav` | 边建图边导航 | ✅ | ✅ | ❌ | ❌ | 在线 SLAM 直接喂 costmap |
| `nav` | 先建图后导航 | ❌ | ✅ | 仅 `icp` / 留空 | ✅ | 所选重定位模块 |

- 真值表已逐组合验证（8 种 `mode`×`mapper`×`localization`）；
- QoS 已核：slam_toolbox 与 cartographer occupancy_grid 的 `/map` 均 `transient_local`，与 costmap `static_layer` 订阅匹配。

### 4.2 新增装配级参数 `spin_speed`

`fake_vel_transform` 的小陀螺固定角速度：**仿真的 `planar_move` 没有基线自转**，任何角速度修正都会让底盘按 5 rad/s 转，而**仿真里没有云台补偿、雷达跟着转**（10 Hz LIO 每帧承受 ~29°）→ 排查导航问题先用 `spin_speed:=0.0`。

### 4.3 新增工具

| 工具 | 作用 | 验证 |
|---|---|---|
| `tools/check_map_reachable.py` | 地图连通域/目标点可达性/推荐可达目标点（含净空）；也可当**幽灵墙检测器** | RMUL2026 判出 3 块连通域、目标不可达；RMUL 用起点 (0,0) 落在 64.5% 大连通域（反向印证出生点系） |
| `tools/pcd_to_grid_map.py` | LIO 的 3D `.pcd` 切层投影 → `.pgm/.yaml`（**第三条 2D 地图来源**），带与已有图的比对指标 | RMUL.pcd → 与 slam_toolbox 的图**同一坐标系**（最佳平移 1~2 格 = 5~10cm），±1 格容差下参考图墙覆盖 **84.7%** |

### 4.4 文档新增索引

- `docs/architecture.md`：§3.2.1 三层职责（lio/localization/mapper）、§3.2.2 2D/2.5D/3D 与降维、§3.2.3 "3D 怎么用"、§3.2.4 感知与决策接口、§3.2.5 点云多路切分/2D 为何能承载 3D/两张 costmap、§3.2.6 插件协议与调参/算力/行业实践、§3.2.7 坡道隧道的 2.5D 方案与落位；
- `docs/smoke_test_runbook.md`：§0.4 RViz 开关、§0.4.1 spin_speed、§0.5 地图资产与坐标系、§0.6 选点先验连通域、§0.7 三形态对照、§1.1.1 LIO 点云投影、§1.2 边建图边导航、§9.1/9.2 判据、§10 错误对照（新增多行）、§12 实测记录；
- `docs/tf_interface_contract.md`：T1–T5 已实施，**T6 撤销**。

---

## 五、概念澄清（本轮讨论的结论速记）

1. **三层职责**：`lio` 出 `odom→base_link`（高频连续、会漂）；`localization` 出 `map→odom`（低频、全局不漂）；`mapper` 造 `/map`（在线建图时**兼任**发 `map→odom`，因此 `slam_nav` 与 `nav` 的重定位模块**绝不能同时开**）。
2. **LIO 不只能定位**：FAST-LIO/Point-LIO 是 LiDAR-Inertial SLAM，跑里程计同时维护 **3D 点云地图**（`PCD/<world>.pcd`）。2D 地图有三条来源：① 在线 2D SLAM ② LIO 点云切层投影 ③ 外部/官方平面图。
3. **降维的判据**：降维是否无损，取决于**被丢掉的维度是否影响决策正确性**。平地 → 无损（z 只影响"挡不挡"）；坡道（z 的变化率）、隧道（z 的分布）→ 有损 → 需要 2.5D/3D。
4. **2D 栅格不"承载"3D 信息**：它只存**一个由 3D 算出的判断**（占据/自由/未知的**类别编码**）；3D 语义藏在**规则**里（高度带、地面分割、投影、时间衰减）。单格无法区分"横梁 z=1.5"和"纸箱 z=0.1"。
5. **代价的数值从哪来**：不是降维给的，而是后续图层给的 —— `static_layer` 翻译值域 → 多层按 **max** 合并 → `inflation_layer` 生成 `252·exp(−scale·(d−r_in))` 的梯度（global `scale=8, r_in=0.2`；0.24m→183、0.40m→50、0.60m→10）。
6. **`/map` 与 costmap 标尺不同**：`/map` 是 **-1/0~100**（未知/自由/占据概率），costmap 是 **0~255**（0 自由、1~252 代价、253 内切、254 致命、255 未知）。**灰色能不能走由 `allow_unknown` 决定**（我们设 true）。
7. **205 像素的灰/白**：`free_thresh=0.25` 时 `occ=0.196<0.25` → **自由(白)**；经典 `free_thresh=0.196` 时才判为**未知(灰)**。这解释了"调着调着灰变白"。
8. **时间维 vs 刷新率**：刷新率 = 多久重算/发布（costmap update/publish）；时间维 = 表示里带不带"何时观测/何时过期"（STVL `voxel_decay`、raytrace 清除）。要把三个率分开：**传感器率**（触发 mark/clear）、**update_frequency**（写主图、结算衰减）、**publish_frequency**（仅可视化）。
9. **nav2 的 2D 契约**：规划/控制/行为插件只拿 2D costmap。想用 3D：① 加**图层插件**（改动小、一致性最好）② 加**规划器插件**（接口仍 2D，插件内部自订阅 3D，但 BT/behavior/controller 仍按 2D 走）。`nav_msgs/Path` 本身是 3D 的，卡点在输入侧与控制侧。
10. **sim2real**：能迁的是**架构/接口/选型/参数初值/地图/工具流程**；不能迁的是**鲁棒性**。三大杀手及解法：
    - **标定/时间同步** → LI-Init/lidar_align 标外参；PPS+触发或 `time_sync_en`；判据=静止点云不分层、闭合路径误差 cm 级；
    - **打滑/延迟** → 用 LIO 替轮速（已做）、标定"指令速度 vs 实际速度"、缩短链路、提控制频率、`velocity_smoother` 切 CLOSED_LOOP、控制器选 DWB/TEB；
    - **感知噪声→幽灵障碍** → 建图期 SOR/多帧一致性/射线清除/多走两遍；建图后形态学+小团块过滤+可达性检查；运行时静态层只放确认结构、临时物交给带时间维的层（静态图**不会自愈**）。
    - **最高价值用法**：在仿真里**注入缺陷**（`cmd_vel` 延迟/丢包、摩擦/打滑、点云离群点）→ 选型阶段就筛出鲁棒算法。

---

## 七、2026-09-24 复盘：`mode:=nav` 下「车不动 + 秒报 SUCCEEDED」的链路定位

### 7.1 症状
RViz 给目标 `(-1.00, 2.00)`（AMCL 重定位）→ BT 立刻 `Goal succeeded`，`distance_remaining` 停在 **2.3042919635772705**，`/cmd_vel` 全程**零样本**，车体一步不动。

### 7.2 决定性证据（全部在活图上实测）

| 观测 | 命令 | 实测 | 结论 |
|---|---|---|---|
| sim 时钟 | `ros2 topic echo /clock --field clock.sec --once` | 1069 | 时钟在跑 |
| **costmap 自己发的栅格** | `ros2 topic echo /local_costmap/costmap_raw --qos-durability transient_local --field header.stamp --once` | **1067.58**（global 1068.88） | costmap 节点时钟、发布线程、更新循环**全部正常** |
| **costmap 的 footprint** | `ros2 topic echo /local_costmap/published_footprint --field header.stamp --once` | **644.682，相隔 181 秒两次采样一字不变** | 它的 `getRobotPose()` 永远返回**启动那一瞬**的僵住位姿 |
| TF 线上是否新鲜 | `ros2 run tf2_ros tf2_echo <A> <B>`（base_link→base_link_fake、odom→base_link、map→odom、map→base_link_fake） | 949~958 **全部新鲜**，`/tf` 42.9 Hz | **TF 本身没有断** |
| 控制器自己的日志 | `~/.ros/log/controller_server_*.log` | `Exception in transformPose: Lookup would require extrapolation into the past. Requested time 644.682000 but the earliest data is at time 644.882000, when looking up transform from frame [odom] to frame [map]` → `Unable to transform robot pose into global plan's frame` → **`Reached the goal!`**（相隔 20 µs） | 见 7.3 |

### 7.3 因果链（已闭环，可稳定复现）
1. 两个 costmap 子节点（`local_costmap` 在 `controller_server` 进程内、`global_costmap` 在 `planner_server` 进程内）的 **tf2 缓冲区在启动约 0.2 秒后就不再进新数据**，永远停在 `644.682` / `646.282`。
2. ⇒ `Costmap2DROS::getRobotPose()`（内部 `nav2_util::getCurrentPose()`）用「最新可用」查询，于是**一直成功**返回**同一个 644.682 的旧位姿**（`published_footprint` 以 20 Hz 重复发这同一戳，所以后来才加入的订阅者也能收到，看起来"话题还活着"）。
3. ⇒ `ControllerServer::isGoalReached()` 里 `nav_2d_utils::transformPose(costmap_ros_->getTfBuffer(), ...)` 要把目标姿态从 map 转到 odom：它拿的正是这个**旧缓存**（最早数据 644.882）去查 **644.682** ⇒ `ExtrapolationException`。
4. ⇒ **上游 `controller_server.cpp` 打印了 ERROR，却把 `transformPose()` 的 bool 返回值丢掉**，`transformed_end_pose` 保持默认 `(0,0,0)`。
5. ⇒ `SimpleGoalChecker`（xy 容差 0.25 m / yaw 0.25 rad）比较车实际位姿 `(0.013, -0.007)` 与 `(0,0,0)`：相差 **1.4 cm < 容差** ⇒ **“Reached the goal!”** ⇒ 零速 ⇒ BT 报 `Goal succeeded` ⇒ **车一步没动**。
   （两次目标相隔 203 秒、结果完全一致；这条链解释了每一个观测到的现象。）

### 7.4 已排除的假设（都有活图证据，不必重复试）

| 假设 | 证据 | 结论 |
|---|---|---|
| `/tf` QoS 不匹配 | `ros2 topic info -v /tf`：6 个发布者 + 9 个订阅者**全部** RELIABLE / KEEP_LAST(100) / VOLATILE | 排除 |
| RMW 是 Fast DDS（需换 CycloneDDS） | 本栈**已经在跑** `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（`ros2-humble-rmw-cyclonedds-cpp 1.3.5` 已装） | 排除（这一招已在用） |
| 有僵尸/重复进程在抢 | `/cmd_vel` 的 5 个发布者 = `velocity_smoother` + `behavior_server` 的 4 个行为插件（Humble 正常）；`/tf` 6 个发布者全是本尊；`/clock` **只有 1 个发布者** | 排除 |
| 某些节点 `use_sim_time` 没设 | 逐节点 `ros2 param get`：controller / planner / bt_navigator / amcl / map_server / velocity_smoother / behavior_server / **两个 costmap** 全 True | 排除 |
| 组合容器共享 executor 导致互锁 | `use_composition:=False`（进程已分离，节点列表里无 component_container）后**同样复现** | 排除 |
| `transform_tolerance` 太小 | 曾放大到 1000.0：在 tf2 里这个值经 `getCurrentPose()` 传成了**等待超时**，遇到「过去的时间点」这种永远等不来的查询，会让 costmap 线程一次阻塞到超时（1000 秒）—— **已撤销回 0.3** | 不是解，且危险 |

### 7.5 仍未知 + 下一步（按优先级）
1. **未解的问题**：为什么这两个 costmap 的 `tf2_ros::Buffer` 只在启动一瞬进数据、之后永久不进。节点时钟（`costmap_raw` 新鲜）、executor（动作与发布都在转）、线上 TF（新鲜）、QoS、RMW 都已排除 ⇒ 卡点在这条订阅的**摄入回调**上，而不是上游数据。（**→ 2026-11-02 已闭环：根因是 tf2 0.25.23 的 ABBA 死锁，见 §八**）
   - 待查线索 A：buffer 时钟类型的差异 —— 实测 **`tf2_echo`（墙钟缓存）能正常读到 sim 戳的 TF**，而 costmap（`use_sim_time=True` 的 sim 钟缓存）读不到；需确认 tf2 的 `TF_OLD_DATA` 剪枝是否按 buffer 时钟判定，以及 `/clock` 只有 **7.76 Hz**（不是常见的 100 Hz）是否让剪枝窗口抖动。
2. **该修的两处（互相独立）**：
   - ① **上游 `nav2_controller/src/controller_server.cpp::isGoalReached()` 必须尊重 `transformPose()` 的返回值**（失败就 `return false` 并明确报错），让"假到达"至少变成"诚实失败"。做法与 cartographer 一样：把 nav2 拷进 `src/` 做 overlay 构建，`third_party/nav2` 保持只读。
   - ② **让 costmap 的 tf 摄入不冻**：候选是把 `costmap_2d_ros.cpp:187` 的 `TransformListener(*tf_buffer_)` 改成带节点/回调组（`tf2_ros::TransformListener(*tf_buffer_, node, true)`），或给它独立 spin 线程。
3. **5 分钟判据实验**：只重启 nav2（sim/LIO 不动），每 2 秒采一次 `ros2 topic echo /local_costmap/published_footprint --field header.stamp`，看戳是否**先跟 `/clock` 走几秒、然后固定不动**；若是，即确认「启动窗口内 TF 尚未就绪 ⇒ 摄入线程死掉」这一形态，再做 ②。

### 7.6 上游查证结论（2026-09-24 联网调研）

**① 「丢弃 `transformPose` 返回值」是上游已知缺陷：main 已修，Humble 永不回移**
- 引入：[PR #2780](https://github.com/ros-navigation/navigation2/pull/2780)（2022-01-21）把目标 TF 查询挪进 `isGoalReached()`，同时**丢掉了返回值**。
- 修复：[PR #6436](https://github.com/ros-navigation/navigation2/pull/6436)「Reusing a staleness-proofed tf lookup…」（2026-09-20 合入 main，关 [#6320](https://github.com/ros-navigation/navigation2/issues/6320)/[#6316](https://github.com/ros-navigation/navigation2/issues/6316)）：把 `nav_2d_utils::transformPose` 换成 `nav2_util::transformPoseInTargetFrame`（**false 就抛 `nav2_core::ControllerTFError`**），并新增 `transform_staleness_threshold` 新鲜度阈值。
- **#6436 没有 `backport-*` 标签，Humble 无回移**（Humble 分支最近的 controller_server 改动是 #6191）⇒ 想让"假到达"变成"诚实失败"，只能自己把这套纪律补回本地（overlay）。

**② 日志里那两行的分工（纠正 7.2 的表述 —— 这一点很关键）**
- `[tf_help] Transform data too old when converting from map to odom` + `Data time: … / Transform time: …` = **`isGoalReached()` 那条路**。`nav_2d_utils::transformPose` 会**吞掉** ExtrapolationException，退而取"最新缓存变换"并按 `transform_tolerance` 判年龄 ⇒ 超龄就 `return false`；上游又不看这个返回值 ⇒ 目标姿态保持默认 `(0,0,0)` ⇒ 假到达。**这才是主证据。**
- `[controller_server] Exception in transformPose: … from frame [odom] to frame [map]` = **同一控制周期里「全局路径转局部系」那条路**（只有 `transformPoseInTargetFrame` 会打印 `ex.what()`），是同一个旧缓存造成的**第二个症状**，不是假到达的直接原因。
- 顺带核实：本地 costmap 是 `global_frame: odom`（**不是** map）✓，所以常规查询不走 `map↔odom` 那条慢链。

**③ `transform_tolerance` 在这个 bug 上根本不是杠杆（撤销是对的）**
- tf2 里 `lookupTransform(…, timeout)` 的 `timeout` **只是"等数据到达"的等待时长，绝不是外插容差**，对"过去的时间点"永远无效（后到的数据戳只会更晚）。
- Humble 的 `nav_2d_utils::transformPose` 甚至**不用**它当超时，只在 Extrapolation 回退分支里拿它判"旧变换能否凑合" ⇒ 调大要么无效，要么变成「拿 400 秒前的旧变换凑合并用它判距离」，同样是错的。与上游 [#5234](https://github.com/ros-navigation/navigation2/issues/5234)（"transform_tolerance 对 controller_server 无影响"，closed **未修**）完全吻合。

**④ 「costmap 的 tf 摄入停掉」没有对应的上游报告**（我们这一例是残余病例）
- 最接近的是 [#3352](https://github.com/ros-navigation/navigation2/issues/3352)：Gazebo 下 footprint 冻结、导航失效；维护者判定为 **RMW 缺陷**（让去报 Fast-DDS，关联 [eProsima/Fast-DDS #3195](https://github.com/eProsima/Fast-DDS/pull/3195)），报告者换 CycloneDDS 后消失。**我们已经在 CycloneDDS 上，所以不是它。**
- [geometry2 #727](https://github.com/ros2/geometry2/issues/727) 给出唯一"节点看着健康但 /tf 回调饿死"的可复现原因：`~/.bashrc` 里残留的 `FASTRTPS_DEFAULT_PROFILES_FILE` 自定义 DDS profile。**本机已查：只有 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 与 `ROS_LOCALHOST_ONLY=0`，无任何 DDS profile 变量 ⇒ 排除。**（附带发现：`~/.bashrc` 192~198 行把同一行 RMW 导出**重复写了 7 次**，无害但建议清理。）
- tf2 会在**时间跳变**时清空整个缓冲区（`Detected jump back in time / Detected time source change. Clearing TF buffer.`，连静态帧一起清）⇒ **已 grep 全部日志：无任何清空事件 ⇒ 排除。**
- tf2 缓冲区 cache 固定 10 s、nav2 不暴露该参数；nav2 用的正是"隐藏节点 + 独立线程 + `setUsingDedicatedThread(true)`"，**上游 main 也没改这个策略** ⇒ 不要照搬「给 `TransformListener` 传 node + `spin_thread=true`」这类改法（会破坏该契约，除非同时把回调组挂进自己的 executor 并手动置 `setUsingDedicatedThread(true)`）。

**⑤ 所以本地该做的是「补回 #6436 的纪律 + 单独解决摄入冻结」，不是继续找配置开关。**

---

## 六、待办（未完成）

| 优先级 | 事项 | 说明 |
|---|---|---|
| ★★★ | **定位 `/scan` 丢帧环节** | 实测 `/scan` 只有 **0.55~3.03 Hz**（`/odom` 10 Hz、IMU 100 Hz），这是"转起来跟不住"的前置条件（10 Hz/100ms 帧内转 60°/s 就是 6°，1 Hz 就是几十度，任何 2D SLAM 都无解）。3 万采样保持不动的前提下逐段测 `/livox/lidar/pointcloud` → `/segmentation/obstacle` → `/scan` 的 hz，判断丢在 `linefit`（`n_threads 4`/`n_segments 360`）还是 `p2l`；再决定是加线程/QoS 深队列还是调 `angle_increment` |
| ★★★ | **落盘本次建图（patched 核心已生效）** | 顺序：① `tools/scripts/mapping/save_grid_map.sh`（= `map_saver_cli -f src/bringup/rm_nav_bringup/map/RMUL2026`，存 `.pgm/.yaml`）；② `/finish_trajectory` + `/write_state` 存 `.pbstream`（runbook §5，绝对路径）；③ `tools/scripts/mapping/save_pcd.sh` 存 `PCD/RMUL2026.pcd`（需 LIO 在跑）。**⚠️ 坐标系**：cartographer 的 `map` 系是**出生点相对系**，与现有 `map/RMUL2026.pgm`（世界系，`origin:[2.68,0.228]`）差 ≈`(4.3,3.35)`⇒ 覆盖前先另存一份比对，并把 `amcl_init_x/y` 改成 `0.0` |
| ★★★ | **重建 RMUL2026 地图** | `mode:=mapping` + 遥控 + 落盘三件套（`.pgm/.yaml`、`.posegraph`、`PCD/RMUL2026.pcd`）；落盘前备份旧图。建完跑 `check_map_reachable.py --start 0 0`，并把 RMUL2026 的 `amcl_init_x/y` 改为 `0.0`（新图为出生点系）、同步 §0.5 表格 |
| ★★★ | **场景三 nav+ICP @ RMUL 重跑** | 等 30~60 s；按 `/livox/lidar→/livox/imu→/imu/data→/odom` 逐段定位；必要时 `lio:=pointlio` 做 A/B |
| ★★ | 场景 4/5/6/7 未测 | slam_toolbox 纯定位、cartographer 建图/纯定位、`nav:=dwb|teb` |
| ★★ | 在新图上验证"发目标能走" | 之前被幽灵墙挡住，未真正验证循迹与小陀螺 |
| ★★ | **`global_obstacle` A/B** | 同一场地/目标点分别跑 `stvl` 与 `scan`，比较：全局路径是否绕开临时障碍、CPU 占用、0.1 m 矮台在 local/global 的判断是否一致（结果记入 `algorithm_matrix.md` §四） |
| ★★ | **`local_obstacle` A/B + 降级验证** | 同一场景跑 `scan` / `cloud` / `both`：① 贴墙 0.3 m 时 local 是否看到（`scan` 应看不到）；② `pkill -f pointcloud_to_laserscan` 后是否 WARN + 1 s 内停车（runbook §7.3）；③ CPU 增量 |
| ★ | 工具增强 | `pcd_to_grid_map.py` 加 SOR + 小团块过滤 + 形态学细化 + 用 `/path` 做射线清除（把"洪泛 free"升级为"射线 free"） |
| ★ | 退化测试槽位 | 写 `cmd_vel` 延迟/丢包注入节点；摩擦/打滑与点云离群点注入 |
| ★ | 未来 2.5D / 3D 槽位 | 高程/坡度/净空图生产（`src/perception/rm_perception/rm_elevation_map/`）+ costmap 图层插件（`src/nav/rm_navigation/rm_costmap_layers/`）；云台瞄准（2–3 DOF）规划 |

---

## 八、2026-11-02 闭环：costmap 的 tf 摄入冻结 = tf2 0.25.23 的 ABBA 死锁

### 8.1 结论
- `[tf_help] Transform data too old when converting from map to odom_level` 是 `ControllerServer::isGoalReached()` **每周期（30 Hz）**把目标姿态 map→odom_level 的年龄检查（`nav_2d_utils::transformPose`，容差 = local costmap `transform_tolerance` = 2.0 s）。它只是**症状**，不是病。
- 病：controller_server 进程内 tf2 缓冲里的 **map→odom_level 最新戳整段冻结**。实测（2026-11-02 那轮 GUI）：196 条报错里 `Transform time` **全是 6.412 s**（6.112 + beluga 的 0.3 容差）；同一时段 `/plan` 戳 10.308 → 11.040 → 11.760 → 12.504 → 13.212 → 13.908 → 14.640 在涨（RTF≈0.7）⇒ 仿真钟与规划器都活着，只有**该进程的 /tf 摄入线程死了**。
- 死因 = tf2 **0.25.23** 的锁序反转（ABBA）：
  - A：`tf2_ros::Buffer::waitForTransform()` **持** `timer_to_request_map_mutex_` 调 `addTransformableRequest()`（要 `transformable_requests_mutex_`）。
  - B：`BufferCore::testTransformableRequests()` **持** `transformable_requests_mutex_` 直接调 tf2_ros 回调（要 `timer_to_request_map_mutex_`）。
  - 两线程对向拿锁 ⇒ 永久互锁。B 正是「/tf 订阅回调」所在的监听线程 ⇒ **从此再也不进 /tf**，缓冲永远停在死锁那一刻的戳。

### 8.2 证据
1. 本机 gdb 全线程转储（`/home/weicheng/bt.txt`）：Thread 14 在 `waitForTransform`→`addTransformableRequest`，持 `timer_to_request_map_mutex_`(0x63059673fb90)、等 `transformable_requests_mutex_`(0x63059673fae0)；Thread 13 在 `testTransformableRequests`→tf2_ros 回调，持 `transformable_requests_mutex_`、等 `timer_to_request_map_mutex_`。**双向等待闭环，无第三把锁参与。**
2. 上游修复：commit **9997e96955**「Fix ABBA deadlock between `waitForTransform` and `testTransformableRequests` (backport #982) (#990)」，**2026-09-10** 合入 humble 分支；只改 `tf2/src/buffer_core.cpp`（+64 / −46）——把待调回调先收进 `pending` 向量，**释放 `transformable_requests_mutex_` 之后再调**（注释原文：`Invoke callbacks without holding any mutex to prevent lock-order inversions.`）。
3. 版本时间线（实测本机 apt）：
   - 已装 `ros-humble-tf2 0.25.23-1jammy.20260907`（**比修复早 3 天**）
   - 源里已有 `ros-humble-tf2 0.25.24-1jammy.20260915`（**比修复晚 5 天**）
   - 源码核对（tag 级）：`curl -sL https://raw.githubusercontent.com/ros2/geometry2/<tag>/tf2/src/buffer_core.cpp | grep -c "Invoke callbacks without holding any mutex"` ⇒ **0.25.24 = 1，0.25.23 = 0**。
4. 入口与 MessageFilter 无关：local costmap 每周期 `getRobotPose()` → `nav2_util::getCurrentPose(stamp=TimePoint)` → `tf_buffer.transform(..., timeout=transform_tolerance=2.0)` → `lookupTransform(timeout)` → `waitForTransform()`。**所以 2026-10-31 去掉 `obstacle_cloud_layer` 只是拿掉了一个入口，死锁照样会发生**（这正好解释了「去掉后仍然冻」）。`bt.txt` 那次的入口恰好是点云 MessageFilter，但入口可以是任何带 timeout 的查询，包括 costmap 自己的 `getRobotPose`。
5. 同一疾病的历史记录：§7.3 那条「两个 costmap 的 tf 缓冲启动 0.2 s 后不再进数据」与本次「6.412 冻结」是**同一个死锁**，只是竞态命中的时刻不同（启动窗口 vs 6.4 s 后的某个查询）。

### 8.3 修复（唯一动作：把 geometry2 全家升到 0.25.24）
```bash
sudo apt update
sudo apt install --only-upgrade 'ros-humble-tf2*'
# 显式等价：
# ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-py ros-humble-tf2-msgs \
# ros-humble-tf2-geometry-msgs ros-humble-tf2-kdl ros-humble-tf2-eigen \
# ros-humble-tf2-eigen-kdl ros-humble-tf2-bullet ros-humble-tf2-sensor-msgs \
# ros-humble-tf2-tools ros-humble-tf2-ros-py
```
核对：`dpkg -l | awk '/ros-humble-tf2/{print $2,$3}'` 应全是 `0.25.24-1jammy.20260915*`。**这是纯补丁版本升级，ABI 不变，不需要重编本仓库任何包**（nav2 / beluga 的 deb 都动态链接 `libtf2.so`）。

**2026-11-02 已执行（实测）**：12 个 geometry2 包全部 `0.25.24-1jammy.20260915*`；`/opt/ros/humble/lib/libtf2.so` 时间戳 2026-09-16 04:37；`tf2/package.xml` 与 `tf2_ros/package.xml` 版本均 0.25.24。
⚠️ 前置坑：本机 `bind9-host` 当时是 **`iHR`（更早一次系统更新被中断）**，apt 以「依赖 `bind9-libs=…22.04.7` 但 …22.04.6 正要被安装」为由**拒绝**任何安装请求；先 `sudo apt --fix-broken install`（或把 bind9-host/bind9-libs/bind9-dnsutils 统一升到 …22.04.7）才能装 tf2。

### 8.4 升级后怎么判定（下次 GUI 跑）
| # | 判据 | 期望 |
|---|---|---|
| 1 | `~/.ros/log/controller_server_*.log` 里的 `[tf_help] Transform data too old` | **0 条**（上一轮 196 条 / 6.4 s） |
| 2 | `ros2 topic echo /local_costmap/published_footprint --field header.stamp`（10 Hz） | 戳**跟着 `/clock` 涨**，不再定死在某值 |
| 3 | 发目标后 | `/cmd_vel` 有样本、`dist` 持续减小、车真的动 |

判据 1/2 过了但车仍不动 ⇒ 转 §六 第一条（`/scan` 只有 0.55~3.03 Hz），并注意本次新增的观测：`pointcloud_to_laserscan` 日志出现
`Message Filter dropping message: frame 'base_link' at time 2.0xx ... queue is full` 与
`... timestamp ... earlier than all the data in the transform cache`
⇒ 这条链把 **LIO 初始化（2.404 s）之前**的旧点云排进了队列，之后只能逐条超时丢弃（p2l 的 `queue_size=50` 被提前占满）。修法是让上游点云在 LIO 就绪后才起，或减小 p2l 队列、并给 p2l 设 `transform_tolerance≈0`。

> 附带提醒：`plan_goal_checker` 的 1 Hz 诊断行（`plan_poses=... robot=...`）用的是 `tf2::TimePointZero` 查询（只看「最新可用」，不吃新鲜度），**它一直打「正常」并不能证明 tf 没冻**——判据必须用上表 1/2。


---


## 来源 2/19：`nav/path_clearance_and_contact.md`


> 用户复现（`world:=RMUC2026 mode:=nav lio:=small_point_lio localization:=gicp nav:=mppi
> planner:=smac2d spin_speed:=0.0`，目标 `(-12.64, -0.31)`）：
> **「不抖动，但是他控制器或者规划器感觉有问题，他喜欢贴着膨胀半径走而且容易撞击进去然后重定位出问题」**
> —— 抖动已被 commit `25e33ad` 修掉，剩下的是这一条链：
> **贴边（规划/平滑） → 撞上（执行） → LIO 里程计被撞散 → GICP 判据（正确地）拒收 →
> 连续 25 帧 ⇒ 判「定位失效」、`map→odom` 停发 → nav2 拿冻结的 TF 继续开 →
> 在错误位姿下开车撞墙（实测最坏 474 m/s²、盲开 110 m）。**
>
> 本文是这条链的**实测报告**。结论里有一条是**否定性**的，也是最重要的：
> **这次不是调参能修的**（§0 第 3 条、§2.2、§3、§5）。
>
> ★ **2026-10-07 追加 §7：感知侧三处改动已经落地并实测**（实时坡度/台阶判据、p2l 高度带
> 0.326 → 1.226 m、重跑建图 recipe）。实施细节与回退见
> [`docs/ground_segmentation_slots.md``ground_segmentation_slots.md` §10–§14。

交叉链接：[`docs/localization_slots.md``localization_slots.md`（`map→odom` 单一发布者契约、各定位槽位）、
[`docs/gicp_divergence_and_jitter.md``gicp_divergence_and_jitter.md`（上一轮「发散 + 抖动」的判据来源）、
[`docs/mapping_2d_from_cloud.md``mapping_2d_from_cloud.md`（2D 图是怎么从 3D 点云切出来的 —— **本文的根因就在那份口径里**）。

---

## §0 结论（一句话 + 六条）

**一句话**：**贴边是真的、撞击是真的，但撞的不是地图上的墙** —— 是**场地里坡度 ≤22% 的坡道面**
（以及它 0.06~0.30 m 的低矮结构）：`linefit` 地面分割的参数是
`max_slope ±0.4` / `max_dist_to_line 0.05 m`（`segmentation_sim.yaml`）⇒ **坡道面被当成"地面"整片删掉**，
`/segmentation/obstacle` 里没有它、`/scan` 里没有它、2D 先验图里也没有它
（实测：静态图判"自由"而场地高度 >0.10 m 的格 **88 576 个 = 自由区的 51.6%**）⇒
规划器无从绕、控制器无从躲。车以 ~1.7 m/s 驶上**坡脚**时，刚性底盘（`gazebo_ros_planar_move`
是**速度控制**、无悬架）在 ~2 ms 内被要求获得 `v·slope ≈ 0.17 m/s` 的垂向速度 ⇒
**实测 IMU 79~90 m/s²（≈8~9 g）**（与 `a≈v·slope/Δt` 吻合）。
这一下把 LIO 的里程计打散：`loc_err` 0.06 m → 2.2 m 用 2.4 s，之后**车已经停住**它仍继续跑到
2000 m+（`lio_step` 最坏 9.8 m/帧 ≈ 98 m/s）。GICP 三道判据**按设计**拒收
（创新 0.05→0.58→1.16→2.58 m、合理性查出 `map→base_link z=−1.37 m` 越界），25 帧后停发 `map→odom`。
**最要命的是停发之后**：实测 `map→odom` 冻结后 nav2 仍用**控制器**（不是 recovery 行为）
继续开车 —— 最坏一次盲开 **110 m**、撞到 **474 m/s²**、足迹深进墙里（余量 min **0.05 m**）。

1. **基线可复现且量化**（§1）：同参跑两次（`base` / `base2`）都是「撞 → 定位发散 → 停发 TF → 卡死」。
   `base2`：真正在动的只有 8.6 s / 12.9 m，其余 72 s 停着；执行路径对静态图余量
   p50 **0.570 m**、p05 **0.255 m**、min **0.200 m**；足迹边缘余量（−0.22 m）p05 **−0.05 m**、
   **5.9%** 的行驶样本足迹压进内切带；`/scan` <0.30 m 占 **9.4%**。**"贴边"用户没说错。**
2. **贴边的机制找到了，但"贴边"本身比想象的轻**（§2.1）：
   `SmacPlanner2D` 的**平滑器没有代价项**（`smoother.cpp:100-190`：
   更新式只有 `w_data·(x−y) + w_smooth·(y₊₁+y₋₁−2y)`，代价只做 `cost>252` 的硬回退），
   而本仓库 `w_smooth 0.4 / w_data 0.1`（4:1）⇒ **它把 A\* 用 `cost_travel_multiplier=4.0`
   买来的余量又往内切带外沿拉**（计划路径到内切带的距离 p50 只有 **0.25 m**）。
   实测把 `w_smooth 0.4→0.0`：计划不再贴带（`plan_min_insc` p50 0.25→**0.335**）、
   执行余量 p50 0.57→**0.85 m**（健康段 0.75→**1.03 m**）。**这条已应用（§4.1）。**
   ⚠️ 但要看清"贴边"的真实量级（§2.1 末）：**计划**本身留着 0.13~0.24 m 的足迹余量，
   **执行**比计划再紧 0.08~0.15 m（MPPI 切角）；`base2` 健康段里**余量 <0.40 m 的样本
   全部集中在坡道那一段**（t=11.8~12.6，map (−3.6,3.1) 附近，那里地图上就有一道离车
   0.30 m 的墙）。⇒ "贴着走"是**地图窄口 + 控制器切角**叠出来的局部现象，不是全程贴边。
3. **但致命一击与贴边无关，是"地图看不见的坡道面"**（§2.2，**这条要大声说**）：
   在撞击时刻（t≈11.9 s，map `(-3.6, 3.1)`）—— 静态图 `occ=False`（自由）、
   `/scan` 最小距离 **0.72~1.0 m**（雷达也没报）、而**场地网格**（`RMUC2026.stl`，毫米→米 + z 抬升
   1.6413436 还原）在该点周围 0.6 m 内有 **0.20~0.35 m 的几何**、车心处的地面高度从
   0.034 → 0.054 → 0.070 → 0.097 → 0.121 m 连续爬升（≈10% 坡，< 0.4 ⇒ 被 linefit 当地面删掉）。
   ⇒ **修的是建图那一步的高度/地面口径，不是 nav2 参数。**
4. **A/B 把"调参能不能救"这个问题回答了：不能**（§3，11 次跑覆盖 9 个配置）：
   单变量里 `w_smooth→0`、全局膨胀带加宽、`planner:=navfn`、`lost_after_rejections=0` 各有过 1 次成功，
   但**同一个配置复跑（`v_ginfl` → `v_ginfl2`）第二次就灾难性失败**
   （盲开 126.9 m、15 次 recovery、IMU 474 m/s²）；`cost_travel_multiplier 12`、
   MPPI 三个 critic 权重、`nav:=rpp` **全部失败**。
   更关键的是**同一个撞击量级下结局不定**：`v_nolost` 扛住了 **122.7 m/s²** 的撞击并到达目标，
   而 `base2` 在 **86 m/s²** 那一下就散了 ⇒ 这一环是**概率性**的。
   **决定成败的不是参数，是"这一次有没有在坡脚挨那一下"**（唯一一次 IMU 峰值 27 m/s² 的跑就成功了）。
5. **定位这一环没有被"调松"**（§4.4）：撞完之后发散的是 **LIO 的 `odom→base_link`**，
   而 `map→odom` 归 GICP；契约明确"GICP 绝不发 `odom→base_link`"
   ⇒ 放松 GICP 判据 / 改成"永不放弃"**救不回来**（实测 `v_nolost`：TF 一直发，
   但 nav2 的融合位姿仍然是坏的，一样盲开乱撞）。
6. **停发 `map→odom` 之后 nav2 没有互锁**（§2.5 末）：实测停发后的移动样本里
   **575/581 有 `/cmd_vel_nav`（控制器）指令** ⇒ 不是 recovery 行为在动，是**控制器在冻结的
   map 系里继续跟路径**。这是 nav2 侧的安全缺口（本次只登记，未改，见 §4.5 与 §7.4）。
7. **感知侧三处改动（§7）已落地**：③ 实时坡度/台阶判据、② p2l 高度带 0.326 → 1.226 m、
   ① `ground` 默认**保持 `linefit`**（证据见 `docs/ground_segmentation_slots.md` §10.5）。
   重跑建图与验收 recipe 见同文档 §12。

---

## §1 实测基线（硬数字）

### 1.1 装置

* 记录仪 `tools/scripts/regress/nav_clearance_probe.py`（**只读仪器**：订阅 `/tf`、两张 `/…/costmap`、
  `/plan`、`/odom_ground_truth`、`/scan`、`/livox/imu`、三路 `/cmd_vel`、`~/fitness_score`、`~/converged`；
  除发一个 `navigate_to_pose` 目标与收尾零速外不改任何东西）。
* 跑法 `tools/scripts/regress/run_nav_clearance_ab.sh`（一次 bash 调用 = 起栈 + 记录 + 收尾：
  `HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、独立 `GAZEBO_MASTER_URI`、`unset DISPLAY`、
  `nav_rviz:=False`、残留 gzserver 守卫；`--kv` 打的参数补丁**跑完必还原**并逐文件 diff 核对）。
* 参数补丁 `tools/scripts/regress/apply_nav_kv.py`（**叶子键必须已存在**，否则报错退出 ——
  防"键名写错 = 静默没生效、A/B 白跑"；确实要新增键写 `=+VALUE`）。
* 分析/画图 `tools/scripts/regress/analyze_clearance.py`；
  场地网格对账 `tools/scripts/regress/field_mesh_vs_map.py`。
* 用户那次逐字同参 + `spin_speed:=0.0` + 目标 `(-12.64, -0.31)`，共 **11 次跑 / 9 个配置**（§3，另有 1 次 P0 回归见 §4.0）。

### 1.2 四个量的定义（先讲口径，免得误读）

| 量 | 定义 | 为什么这么定义 |
|---|---|---|
| `g_lethal_*` | 机器人中心到**全局代价地图**里最近 `LETHAL(254)` 格的欧氏距离（米，EDT 现算） | nav2 里"致命格"就是真障碍；减 `robot_radius 0.22` = **足迹边缘余量** |
| `g_lethal_f` / `g_lethal_t` | 分别用 **nav2 真正看到的融合位姿**（`map→odom ∘ odom→base_link_fake`）与**真值位姿** | 两者之差 = "定位误差把车放到哪儿去了" |
| 真值对齐 | `/odom_ground_truth` 实测是 **Gazebo 世界系**位姿（`planar_move` 直接抄 `WorldPose()`），与 map 系差一个常量；静置前 5 s 取 `median(融合 ∘ inv(真值))` 冻结 | 不冻结就会把出生点世界坐标 (10.93, 2.52) 当成定位误差 |
| 接触/被推 | `|Δ真值 − R(yaw)·指令·dt|`（逐 0.1 s）；`exp>0.05 & step<0.35·exp & res>0.05` = **blocked**，`exp≤0.02 & step>0.025` = **pushed** | 底盘是 `gazebo_ros_planar_move`（`strings` 实证调用 `Model::SetLinearVel/SetAngularVel`，**速度控制**不是瞬移）⇒ 位移与指令不符只可能来自物理接触；速度平滑器限幅 4 m/s² ⇒ 位置滞后上界 0.02 m ≪ 0.05 m 门限 |

> ⚠️ 三个踩过的坑（写在这里省得下一个人再踩）：
> ① `/global_costmap/costmap` 与 `/local_costmap/costmap` **不是原始代价**：`Costmap2DPublisher::prepareGrid()`
> 过了一张 translation table（`costmap_2d_publisher.cpp:87-99`）把 `253→99`、`254→100`、`255→-1`。
> 按 253/254 直接判 ⇒ **永远判不出致命格**（第一版探针的 `d_lethal` 恒等于"到数组边界的距离"）。
> ② `/odom_ground_truth` 的 `twist` 是**指令值**（不是实测速度）⇒ 判"是否在动"必须用真值位移。
> ③ OccupancyGrid 的行号**从下往上**数（`y = oy+(H−1−r+0.5)·res`）。栅格化场地网格时写错一次，
> 整张高度图上下镜像，表现为"轨迹处高度恒为 0"，差点把结论带偏。

### 1.3 基线数字（`base2`，与用户同参）

仿真时长 81.2 s，其中**真正在动的只有 85 个样本（≈8.5 s、12.86 m）**，其余 72 s 车停着。

| 指标（移动段） | p50 | p05 | p01 | min | max |
|---|---|---|---|---|---|
| 全局图余量（真值位姿）`g_lethal_t` | **0.570** | **0.255** | 0.242 | **0.200** | 2.552 |
| 全局图余量（融合位姿）`g_lethal_f` | 0.566 | 0.170 | 0.094 | **0.000** | 2.589 |
| 局部图余量 `l_lethal` | 0.974 | 0.555 | 0.410 | 0.405 | 2.260 |
| `/scan` 最小距离 | 0.765 | 0.194 | 0.184 | 0.175 | 2.517 |
| 地图乐观度（地图−雷达，墙面口径）`map_opt` | −0.06 | — | — | — | — |
| 计划路径 min 余量 `plan_min_lethal` | 0.461 | 0.350 | 0.350 | 0.350 | 0.492 |
| 计划路径到内切带 `plan_min_insc` | 0.250 | 0.150 | 0.150 | 0.150 | 0.269 |
| 定位误差（融合 vs 真值） | 0.168 | 0.029 | 0.017 | 0.014 | **2.208** |
| LIO 逐帧步长 `lio_step` | 0.154 | 0.000 | 0.000 | 0.000 | **0.326** |
| `map→odom` 逐帧步长 `mo_step` | 0.005 | 0.000 | 0.000 | 0.000 | 0.082 |
| IMU 模长 `imu_a`（静止 9.8） | 10.05 | 9.01 | 8.85 | 8.78 | **86.40** |

* **足迹边缘余量**（`g_lethal_t − 0.22`）：p50 **+0.35 m**、p05 **−0.05 m**、min **−0.22 m**；
  **5.9%** 的行驶样本足迹边缘 ≤0；`/scan` <0.22 m 占 **9.4%**、<0.16 m 占 **0%**
  （真车体半宽 0.155 m ⇒ 没真插进墙里，但已到"再差 5 cm 就碰"）。
* **地图乐观度 p50 = −0.06 m**：静态图与雷达在**墙面**上一致 ⇒ **墙的位置没错**，
  缺的是**垂直方向上的口径**（§2.2）。

### 1.4 先验图允许多大余量？（离线，判"是不是被走廊逼的"）

对静态图做"最大化路径最小余量"（EDT + 抬阈值做连通性/最短路）：

| 要求的最小余量 | 起点→目标最短路 | 备注 |
|---|---|---|
| >0（只躲占用格） | **20.03 m** | 全场唯一卡脖子在 **(−3.8, −4.5)**，**只有 0.6 m 宽** |
| ≥0.24 | 21.20 m | 能保持 0.25 |
| ≥0.30 | 21.56 m | +7.6% |
| ≥0.40 | 22.29 m | +11.3% |
| ≥0.50 | 22.89 m | +14.3% |
| ≥0.65 | 23.60 m | 全场瓶颈（0.65 m） |

读法：**"全程离墙 ≥0.4 m"只多走 11%** ⇒ 贴边不是几何逼出来的，是**代价形状**选的（§2.1）。
另外那个 **0.6 m 的卡脖子**（≈ 0.44 m 外接圆 + 两侧各 8 cm）是本图**唯一**窄口：
一旦挤在那儿，任何跟踪误差都直接变接触。

### 1.5 因果时间轴（`base2`，相对"开始行驶"）

| t (s) | 事件 | 数字 |
|---|---|---|
| 0.0–11.7 | 正常行驶，定位稳 | `loc_err` 0.014~0.07 m，`mo_step` ≤0.08 m |
| **11.8** | **首次 IMU >30** | **79.3 m/s²**（静止 9.8），随后 42~**86.4** m/s² 共 **7 个样本**（≈0.6 s） |
| 12.5 | `loc_err` 首次 >0.3 m | 0.06 → 0.30 m（0.6 s 内） |
| 12.9 | `loc_err` >0.5 m | — |
| 14.2 | `loc_err` >1.0 m | — |
| 14.6 | nav2 **第 1 次 recovery** | — |
| 14.6 | `loc_err` >2.0 m | — |
| **14.7** | **最后一次真值移动** | 之后 66 s 车一直停着（真值位移 0） |
| **15.9** | **`map→odom` 停发** | 之后 `mo_age` 一路涨到 80.5 s |
| 15.9–16.5 | GICP 拒帧 → 判失效 | 创新 0.048→**0.504**→**1.164**→**2.575 m**；合理性查出 **z=−1.37 m** 越界；连续 25 帧 ⇒ `ERROR 判定位失效` |
| 81.2 | 记录结束 | `dist_rem` 从未改善，7 次 recovery，无终态 |

**顺序清楚：撞（11.8）→ 定位发散（12.5）→ recovery（14.6）→ 车停（14.7）→ 停发 TF（15.9）。**
GICP 判据**不是**起因：撞之前它**采纳率 100%**（`~/converged` 恒 true、`fitness` 0.003 m²），
是**撞完之后**才发现上游里程计已经离谱，然后执行"绝不发编造 TF"这条契约。

---

## §2 到底是哪个原因（逐条对证据）

任务书列的四个候选：(a) 代价梯度太平（规划）、(b) 足迹/半径与实际几何不符（配置）、
(c) 控制器跟踪误差（切角）、(d) 地图乐观（高度带/去地面切掉了低矮几何）。**逐条对账**：

### 2.1 (a) 规划：**成立，但只解释"贴边"，不解释"撞"**

* **`SmacPlanner2D` 的平滑器没有代价项**（源码，本仓库 `third_party/nav2/nav2_smac_planner`）：
  `smoother.cpp:140-170` 的迭代只有
  `y += w_data·(x−y) + w_smooth·(y₊₁+y₋₁−2y)`；代价**只做硬检查**（`:157-168`，
  `cost > MAX_NON_OBSTACLE` ⇒ 整条回退），而 `constants.hpp:64-65` 是
  `INSCRIBED = 253.0` / `MAX_NON_OBSTACLE = 252.0` ⇒ **"贴着内切带外沿"正是它的可行域边界**。
* 权重还反过来：本仓库 `w_smooth 0.4` / `w_data 0.1`（4:1）+ `do_refinement: true`（再 4 轮）
  ⇒ 平滑项压倒数据项，路径被拉回带边。
* 实测：A\* 出来 `plan_min_lethal` p50 **0.461 m**，平滑后能贴到 `plan_min_insc` p50 **0.250 m**，
  执行再掉到 min **0.200 m** —— 三段落差就是"被拉回来"的证据。
* **顺带纠一个键名**：`SmacPlanner2D` **没有 `cost_penalty`**（那是 Hybrid/Lattice 的：
  `smac_planner_hybrid.cpp:115`、`smac_planner_lattice.cpp:103`），它读的是
  `cost_travel_multiplier`（`smac_planner_2d.cpp:74-75`）—— 本仓库配置已写对。
  `Node2D::getTraversalCost`（`node_2d.cpp:75-79`）是 `1 + mult·(cost/252)` 的**软惩罚**：
  1 m 的窄口里把余量 0.25→0.40 m 只多花 ~1.1 单位长度、省下 ~3.3 单位代价 ⇒
  **A\* 本来想躲，是平滑器把它拉回去的**。
* **"贴边"的真实量级（把三个数分开看，别混）**：
  | 口径 | 到致命格 p50 / min | 折成足迹余量（−0.22 m） |
  |---|---|---|
  | **计划**路径（`base2`） | 0.461 / 0.350 m | +0.24 / **+0.13 m** |
  | **执行**路径（健康段，真值） | 0.750 / 0.304 m | +0.53 / **+0.084 m** |
  | 执行路径（含撞击/恢复段） | 0.570 / 0.200 m | +0.35 / **−0.02 m** |
  ⇒ 计划留 0.13~0.24 m，执行再紧 0.08~0.15 m（MPPI 切角），到"贴着走"的观感。
  再拆时间轴：`base2` 健康段里**余量 <0.40 m 的 8 个样本全部落在 t=11.8~12.6、
  map (−3.6, 3.1) 附近**（那里地图上就有一道离车 0.30 m 的墙，同时脚下是那条看不见的坡）
  —— 其余 49 个健康段样本余量都 ≥0.40 m。
  ⇒ **"贴着膨胀半径走"是"地图窄口 + 控制器切角 + 那一段坡道"叠出来的局部现象**，
  不是全程贴边；把它当成全程问题去调参数会调错方向。

### 2.2 (d) 地图：**成立，而且是"致命那一击"的真正原因（大声说）**

**做法**：把仿真场地网格 `src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world/meshes/RMUC2026.stl`
（45 900 三角形、毫米单位）按 world 文件头注的 `<scale>0.001</scale>` + z 抬升 1.6413436 m 还原成米，
用重心坐标栅格化成"每 5 cm 格的最高**可站立面**高度"，再与静态图占用、`/scan`、真值轨迹逐点对齐
（`tools/scripts/regress/field_mesh_vs_map.py`，§6 ⑤）。

| 检查 | 数字 |
|---|---|
| 撞击时刻（t≈11.9，map (−3.6, 3.1)）**车心处**场地高度 | 0.034 → 0.054 → 0.070 → 0.097 → **0.121 m**（0.5 s 连续爬升） |
| 同一时刻静态图占用 | **`occ=False`（自由）** |
| 同一时刻 `/scan` 最小距离 | **0.72 ~ 1.0 m**（雷达也没报） |
| 该点周围 0.6 m 内的场地几何 | 有 **0.20 / 0.35 m** 的面（含 ±0.10 m 窗内 0.047 m 高差 = 局部 ~10% 坡） |
| 沿轨迹的场地高度 | 0.001 → 0.009 → 0.053 → 0.121 → 0.167 → **0.200 m**（≈1.7 m 行程） |
| 目标点 `(-12.64, -0.31)` 处场地高度 | **0.300 m**（目标在 0.30 m 台面上，图上却是"自由"） |
| 静态图判**自由**、而场地高度 >0.10 m 的格 | **88 576 个（占自由区 51.6%）**；>0.20 m：26.4%；>0.30 m：8.4% |
| 移动段轨迹落在"高度 >0.10 m 的格"上的比例 | **38.8%**（>0.06 m：41.2%） |
| 另一个独立佐证（`v_nosmooth`）：地图乐观度 `map_opt = 图余量 − /scan 最小距离` | p50 −0.05 m，但 **p05 −0.61 m / min −1.08 m** ⇒ 有 5% 的样本里**雷达看见了比"图上最近的墙"还近 0.6 m 以上的东西**（图上看不见的几何；未逐点归因） |

**机制（与配置参数逐条对上）**：2D 图 = 3D 点云 → **去地面** → `pointcloud_to_laserscan` 切高度带。
去地面用的是 `linefit`，参数（`src/perception/rm_perception/linefit_ground_segementation_ros2/.../segmentation_sim.yaml`）：

```
max_slope: 0.4        # 地面线允许的坡度（±22°）
max_dist_to_line: 0.05 # 离地面线 5 cm 内算地面
sensor_height: 0.226   # 与实车雷达高度一致
```

⇒ **坡度 ≤0.4 的坡道面会被"拟合成地面线"并整片删掉**。本场地的坡是 ~10%（0.10 ≪ 0.4）⇒
坡道在 `/segmentation/obstacle` 里**不存在** ⇒ `/scan` 没有、2D 图没有、`local_costmap` 没有。
（p2l 的高度带 `max_height 0.1` 相对雷达（离地 0.235 m）⇒ 覆盖到地面以上 **0.335 m**，
所以**不是高度带切的**，就是**去地面**删的 —— 这一点与 `docs/mapping_2d_from_cloud.md` 的口径一致。）
📌 **与建图侧的报告对上了**：`docs/mapping_2d_from_cloud.md` 头注②从**建图侧**写了
「坡道在 3D 点云里清清楚楚，在 2D 栅格图上基本看不见」；本文从**导航侧**给出了它的后果与量级
——**看不见的坡 ⇒ 规划/控制无从预判 ⇒ 坡脚 8~9 g 的垂向冲击 ⇒ LIO 里程计散发 ⇒ 定位失效 ⇒ 盲开乱撞**。

**撞击的物理量级也对得上**：车以 `v≈1.7 m/s` 上 `slope≈0.10` 的坡，坡脚要求垂向速度
`v_z = v·slope ≈ 0.17 m/s`；刚性底盘（无悬架 + 速度控制）在接触建立的 `Δt≈2 ms` 内完成
⇒ `a ≈ v_z/Δt ≈ 85 m/s²`，**实测 79~90 m/s²**。⇒ 这不是"控制器没躲开"，是**物理上必有的一下**；
速度减半 ⇒ 加速度减半（§4.3）。

**这一条不是"墙画细了/画歪了"**：§1.3 的 `map_opt` p50 = −0.06 m 说明墙的位置准；
缺的是**垂直口径**。⇒ **修的是建图那一步，不是调参。**

### 2.3 (b) 足迹/半径：**不成立**

URDF（`src/bringup/rm_nav_bringup/urdf/sentry_robot_sim.xacro`）实测几何：
`base_link` 碰撞盒 `0.2×0.3×0.1` 且绕 z 转 90° ⇒ 车体 **0.30(x)×0.20(y)**；
四个轮 `r=0.06 / w=0.05` 在 `(±0.10, ±0.13)` ⇒ 外沿 **x ±0.16 / y ±0.155**
⇒ **外接圆 0.223 m、内切 0.155 m**。配置 `robot_radius: 0.22`（全局与局部都是）**正好等于外接圆**，
而 nav2 在没有 footprint 多边形时把 `inscribed_radius` 也取 0.22
（`layered_costmap.cpp:271-283`）⇒ **代价地图比真车更保守 0.065 m**，不是更乐观。**方向相反 ⇒ 不是它。**

### 2.4 (c) 控制器：**部分成立（切角），但不是主因**

* 执行 min 0.200 m vs 计划 min 0.350 m ⇒ MPPI 确实**切进来 0.15 m**（局部图余量 p50 0.974 m 说明它看得见）。
* 但把 critic 加权重（`CostCritic.cost_weight 3→8`、`ObstaclesCritic.repulsion_weight 1.5→4.0`、
  `critical_weight` 20→40）**并把 critic 自己那两个和局部图不一致的膨胀参数对齐**，
  **结局反而更差**（`v_mppiclr`：9 次 recovery、健康段 IMU 198.8 m/s²、余量 min 0.0）⇒ 控制器不是瓶颈。
* 顺带发现一个真问题：`ObstaclesCritic` 的 `cost_scaling_factor/inflation_radius` 默认 **10.0 / 0.55**，
  而局部图是 **5.0 / 0.5** ⇒ 它用 `distanceToObstacle()` 反解的"离障碍多远"**被系统性低估约 2 倍**
  （0.30 m 的边缘余量被算成 0.04 m）。是"静默不一致"，值得单独修，但**不是**这次撞车的原因。

### 2.5 定位那一环：判据没坏；**坏在"上游里程计被撞散"** + **"丢了之后没人踩刹车"**

* `base2` 日志逐帧可查：撞之前 **100% 采纳**；撞之后创新
  0.048 → 0.504 → 1.164 → 2.575 m（门限 0.3 / 0.6 m）、合理性查出 `map→base_link z = −1.37 m`
  越界（预期带 [−0.90, +0.70]）⇒ 25 帧 ⇒ `ERROR 判定位失效、停发 map→odom`。**判据按设计工作。**
* 硬事实：**车在 14.7 s 就停住了**，而 LIO 的 `lio_step` 之后一路涨到 **9.8 m/帧（≈98 m/s）**、
  `loc_err` 涨到 2021 m —— 静止的车 + 发散的 LIO ⇒ `map→odom` 怎么调都补不回来（§4.4）。
* **停发之后更危险**（§3.2）：`v_ginfl2` 停发后盲开 **110.5 m**、`v_combo` **51.7 m**、
  `v_mppiclr` **21.6 m**；这些样本里 **575/581 有 `/cmd_vel_nav`（控制器）指令** ⇒
  **不是 recovery 行为在动，是控制器在冻结的 map 系里继续跟路径**。
  nav2 没有"定位陈旧/失效就停车"的互锁（本次只登记，见 §4.5）。

---

## §3 A/B 表（一次一个变量，同世界/同目标/同 `spin_speed:=0.0`，共 11 次跑 / 9 个配置）

口径：**"健康段"** = 真值在动 且 `mo_age ≤0.5 s` 且 `loc_err <0.3 m`（撞之前/定位还好）；
**"失控段"** = 其余移动样本。"余量"= 真值位姿到全局图致命格的距离；"边缘<0"= 余量 − 0.22 m ≤ 0 的占比；
"IMU>30"= 模长 >30 m/s² 的样本数（静止 9.8）；"停发后行程"= `map→odom` 停发之后真值走的米数。

### 3.1 主表

| tag | 变量（单变量） | 结局 | 行程 | **健康段**余量 p50/p05/min | 健康段 IMU max (>30) | 全程余量 min | 边缘<0 | 计划 min/到带 p50 | 定位误差 max | recov | **停发后行程** |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `base2` | —（用户同参） | ⛔ 卡死 | 12.9 m | 0.750/0.347/0.304 | **86.4** (7) | 0.200 | 5.9% | 0.35 / 0.25 | 2.21 m | 7 | 0.0 m |
| `base` | —（同参第一次） | ⛔ 卡死 | 12.4 m | —* | **80.8** (5) | 0.100 | 9.0% | — | 6.83 m | 7 | 0.0 m |
| `v_nosmooth` | `smoother.w_smooth 0.4→0.0` | ✅ SUCCEEDED | 27.2 m | **1.030**/0.338/0.255 | 89.6 (9) | 0.250 | 0% | 0.25 / **0.335** | 0.61 m | 0 | 10.7 m† |
| `v_ginfl` | 全局 `inflation_radius 0.55→0.85`+`cost_scaling_factor 8→4` | ✅ **SUCCEEDED** | 27.5 m | 0.888/**0.447**/**0.335** | **27.0 (0)** | 0.335 | 0% | 0.112 / 0.335 | **0.42 m** | **0** | 1.5 m† |
| `v_ginfl2` | 同上（**复跑同一配置**） | ⛔ **灾难失败** | 126.9 m | 0.626/0.360/0.335 | 76.2 (5) | **0.050** | 8.3% | 0.0 / 0.25 | **12 883 m** | 15 | **110.5 m** |
| `v_combo` | `v_ginfl` + `w_smooth 0.0` | ⛔ 灾难失败 | 75.6 m | 0.609/0.335/0.250 | **169.0** (11) | **0.000** | 23.5% | 0.40 / 0.35 | 5 104 m | 7 | 51.7 m |
| `v_mul12` | `cost_travel_multiplier 4→12` | ⛔ 卡死 | 25.5 m | 0.559/0.381/**0.350** | 84.1 (4) | 0.354 | 0% | 0.30 / 0.25 | 16.24 m | 6 | 12.7 m |
| `v_navfn` | `planner:=navfn`（结构槽位） | ✅ SUCCEEDED | 27.1 m | 0.750/0.403/0.224 | 72.9 (5) | 0.224 | 0% | 0.25 / 0.304 | 0.62 m | 0 | 10.8 m† |
| `v_mppiclr` | MPPI critic 加权重 + 对齐膨胀参数 | ⛔ 卡死 | 28.8 m | 0.832/0.263/0.180 | **198.8** (3) | 0.000 | 11.4% | 0.112 / 0.25 | 1.05 m | 9 | 21.6 m |
| `v_rpp` | `nav:=rpp`（结构槽位） | ⛔ **没走起来** | 2.6 m | 2.00/1.35/1.30 | **28.2 (0)** | 1.300 | 0% | 0.461 / 0.25 | 0.043 m | 6 | — |
| `v_nolost` | `gicp lost_after_rejections 25→0`（永不放弃） | ✅ SUCCEEDED | 26.3 m | 0.750/0.255/0.180 | **122.7** (5) | 0.180 | 2.2% | 0.141 / 1.60 | 0.54 m | 3 | 0（`mo_stale`=0） |
| `v_slow` | `velocity_smoother.max_velocity 2.0→1.0` | **没跑**（§7 第 2 条） | | | | | | | | | |

\* `base` 那次的余量数字来自"旧探针"（代价地图编码未还原）⇒ 只有接触/定位列可用；结论与 `base2` 同形。
† 这里"停发后行程"包含**正常到点后的余速/微调**（成功的跑里定位一直健康、`mo_stale` 为 0），
所以 `v_nosmooth`/`v_navfn` 的 10 m 不是"盲开"；判据是 §3.2 的 `/cmd_vel_nav` 计数。

### 3.2 逐条读法（这才是这张表的用处）

1. **没有任何一个单变量能可靠地救这条链**。三个"成功"里：
   * `v_ginfl` 是**唯一一次全程没挨重击**（IMU max 27、>30 的样本 0）——
     但**同一配置复跑 `v_ginfl2` 就灾难性失败**（IMU 76、盲开 110 m、15 次 recovery）。
   * `v_nosmooth` / `v_navfn` 成功，但它们的**健康段 IMU 峰值仍然 89.6 / 72.9 m/s²**（9 / 5 个样本）
     ⇒ **它们也撞了，只是这一次 LIO 没散**。
   ⇒ **决定成败的是"这一次在坡脚挨的那一下有没有把 LIO 打散"，不是参数。**
   把 11 次跑按"健康段 IMU 峰值"分档（`base 80.8`、`base2 86.4`、`nosmooth 89.6`、
   `ginfl2 76.2`、`mul12 84.1`、`navfn 72.9`、`combo 169.0`、`mppiclr 198.8`、`nolost 122.7`
   —— **9 次 ≥70 m/s²**；`ginfl 27.0`、`rpp 28.2` —— 2 次 <30）：
   **硬撞的 9 次里 3 次仍然成功**（`nosmooth`/`navfn`/`nolost`）⇒
   **撞击不必然致命，但它是这条链唯一的入口**；LIO 会不会被撞散是**概率性**的
   （同一量级：86.4 散了、122.7 没散）。样本量小，但方向一致。
2. **停发 `map→omom` 之后的盲开是被放大的主因**：
   `v_ginfl2` 盲开 110 m、撞到 **474 m/s²**（≈48 g）、足迹余量 min **0.05 m**（车贴着/插进墙里）；
   `v_combo` 盲开 51.7 m、撞到 **432 m/s²**、**23.5%** 的样本足迹压进内切带。
   驱动来源已查清：**575/581 个样本有 `/cmd_vel_nav`（控制器）指令**，不是 recovery 行为。
3. **`v_mul12` 是有价值的反例**：它把 A\* 余量做得最漂亮（健康段 min **0.350 m**、边缘<0 为 0%），
   **结局照样卡死**（定位误差 16.2 m）⇒ **"贴边"与"卡死"不是同一件事**，只治贴边不治链。
4. **`v_rpp` 是另一种失败**：车 69 s 只走了 2.6 m（定位一直健康、余量 2 m），
   ⇒ 这个槽位在这条路线上**根本没跟起来**（不是本次要查的问题，但结论是"换控制器槽位不解决问题"）。
5. **`v_mppiclr` 说明控制器不是瓶颈**：加了权重、修了 critic 与局部图的膨胀参数不一致，
   结局更差（健康段 IMU 198.8）——因为**看不见的东西加多少权重也躲不开**。

### 3.3 后两条（`v_nolost` 跑了 / `v_slow` 没跑）

* **`v_nolost`（`lost_after_rejections: 0`）**：**SUCCEEDED**（26.3 m、3 次 recovery、目标到达、
  `mo_stale` **0**、`loc_err` max 0.54 m），但它的**健康段 IMU 峰值 122.7 m/s²（5 个 >30 样本）**
  —— 也就是**挨了一下更重的、却活下来了**。本次它**根本没触发"连续 25 帧"那条策略**
  （`mo_stale`=0）⇒ 这条跑**不能**用来证明"永不放弃"更好；
  它真正的用处是**反证"撞击 → 必然丢定位"不成立**：同样是 ≥80 m/s² 的撞击，
  有的跑 LIO 散了（`base2`）、有的跑没散（`v_nolost` 122.7 m/s² 仍健康）
  ⇒ 这一环是**概率性**的（这也解释了 §3.2 第 1 条的结论）。
* **`v_slow`（限速）没跑**：见 §7 第 2 条 —— 只有物理量级论证（`a≈v·slope/Δt` 与实测 85 m/s² 吻合）
  与"能量 ∝ v²"的推论，**没有实测"LIO 能不能扛 ~42 m/s²"**。

---

## §4 建议的参数/文件改动与回滚

> 原则：**只应用测量支持的改动**；每项都有"前值 → 后值 + 依据 + 回滚"。
> 本次**应用**的只有 §4.1。§4.2~§4.5 是建议/未应用（原因逐条写明）。

### 4.0 回归（应用改动之后跑的）

```bash
bash tools/scripts/localization/run_nav_smoke_regress.sh --tag regress_navfix --domain 171 \
     --port 11841 --localization gicp --nav mppi --planner smac2d --goal -1.0 2.0
```

结果：**✅ PASS**（`status=SUCCEEDED 用时=3.3s d_min=0.0m recoveries=0`、
`settled=True`（5 s 窗口内 `map→odom` 漂移 xy=0.0026 m / yaw=0.0006 rad）、
`首个断点：无`、真值 `pose_delta=2.926 m`、RTF 0.81、`fp_distinct=116`）。
快照：`.tmp_gicp_ab/regress/regress_navfix/snap/regress_1791296269.json`。
⇒ **本次改动没有引入回归**（这是"改动前也 PASS"的同一套 P0 判据）。

### 4.1 已应用：规划器平滑器不再把路径拉回内切带

文件：`src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_planner_smac2d.yaml`

| 键 | 前值 | 后值 | 依据 |
|---|---|---|---|
| `planner_server.ros__parameters.GridBased.smoother.w_smooth` | `0.4` | **`0.0`** | §2.1：平滑器**没有代价项**（只有 `cost>252` 硬回退），`w_smooth:w_data = 4:1` ⇒ 它把 A\* 的余量拉回内切带外沿。实测 `w_smooth=0`：计划到内切带 p50 0.25→**0.335 m**、执行余量 p50 0.57→**0.85 m**（健康段 0.75→**1.03 m**）、足迹压内切带 5.9%→**0%** |

**它治什么、不治什么（写在注释里也一样）**：**只治"贴边"这条用户抱怨**；
**不治"撞"**（`v_nosmooth` 的健康段 IMU 峰值仍 89.6 m/s²）⇒ 别把它当"撞车修好了"。
**回滚**：把 `w_smooth` 改回 `0.4`。
**副作用**：路径少一层平滑（更"折线"）；本次实测没有因此卡住或增加 recovery
（唯一一次用它的跑到达了目标、0 次 recovery），但**未做"折线是否影响跟踪质量"的定量 A/B**（§7）。

### 4.2 建议（本次未应用）：让 2D 图别再丢掉"坡道面/低矮几何" —— **根因**

三条路，按推荐度：

1. **改去地面那一环的口径**（正解）：`linefit` 的 `max_slope ±0.4` 会把 ≤22° 的坡道整片当"地面"删掉。
   对"机器人要走的坡"这**恰恰是想要的**（坡道就该能走）；真正要补的是**坡的边界/坡脚/台沿**
   —— 建议对"高度 0.06~0.35 m 的连通块"单独标一层（或把 `max_dist_to_line` 收紧、
   对"平面度不够的斜块"不判地面），**必须在重跑建图时逐场景验证**
   （本次实测：静态图判自由而场地高度 >0.10 m 的格有 **88 576 个 = 51.6%**，动这一步会显著改图）。
2. **"低矮结构 keep-out" 附加层**（不动主图）：从 **PCD**（`PCD/RMUC2026.pcd`，LIO 原始 3D 图，
   z 直方图显示 0.10~0.40 m 有 ~3 万点）生成掩码图，用第二个
   `nav2_costmap_2d::StaticLayer`（`map_topic` 指过去）只加进**全局**代价地图
   （局部图是 rolling + odom 系，装不了静态层）。
   ⚠️ 实测过的坑：朴素"格内 z 最大值 ∈[0.05,0.60] 就算障碍"会**误报 81.5 m²**
   （起点周围 49 格里 11 格被噪点误标）⇒ 必须加最小点数/形态学/连通块过滤，并核对目标可达。
   本次只做到"生成 + 目视 + 起点/目标连通性检查"，**没装进栈里跑**（§7）。
3. **限速**（治标、最便宜、机制最硬）：撞击加速度 `a ≈ v·slope/Δt` **线性于车速**
   （§2.2：1.7 m/s ⇒ 实测 85 m/s²）⇒ `velocity_smoother.max_velocity 2.0 → 1.0`
   理论把坡脚冲击减半（~42 m/s²），并且**同时把"丢了定位之后盲开乱撞"的能量降到 1/4**。
   **代价**：全局限速（不只是坡上）。本次**未跑完验证**（§7），所以没改配置。

### 4.3 建议（本次未应用）：给"定位失效"加一个停车互锁

实测（§2.5/§3.2）：`map→odom` 停发之后，nav2 用**控制器**在冻结的 map 系里继续开车
（盲开最坏 110 m、撞到 474 m/s²）。这是 nav2 侧的缺口：`transform_tolerance` 之类只管
"等多久"，不管"数据多旧"；TF2 对"整条边不再更新"没有过期语义。
可选实现（**本次都没做**，需要单独 A/B）：
* `behavior_server.recovery_plugins` 去掉会动的 recovery（`spin/backup/drive_on_heading`，只留 `wait`）
  —— **注意：这治不了本次观测到的盲开**（那是控制器在动，不是 recovery），只有独立价值；
* 一个"`map→odom` 更新年龄 > N s ⇒ 取消导航"的 BT 条件节点 / 生命周期守卫
  —— 属于新增节点，**本次明令不做**（任务约束"不加 watchdog 节点"）。

### 4.4 不要做：放松 GICP 判据 / 改成"永不放弃"

* 撞完之后**发散的是 LIO 的 `odom→base_link`**；`map→odom` 归 GICP，
  契约明确"GICP 绝不发 `odom→base_link`"（`docs/localization_slots.md`）
  ⇒ 放松 GICP 只能让 nav2 继续拿到**同样坏的融合位姿**（§3.2 第 2 条实测：冻结 TF 下照开）。
* `lost_after_rejections` 那条策略在**这次**的代价是：车明明已经停住、位姿其实还停在最后一次好值上，
  却因为 LIO 自己发散而把 TF 掐了 ⇒ nav2 连 map 系都没了。
  更合理的是"**停发前先看车是不是真的在动**"（车静止 ⇒ 沿用上次采纳值是**正确**的，不是编造）。
  这属于 `gicp_registration` 的**失败策略**改动（跨任务边界，按规矩**先报备、本次未改**），
  且**即便改了也救不了本次**（上游 LIO 已散）—— 登记设计与证据，不做全局放松。

### 4.5 建议（本次未应用）：`ObstaclesCritic` 的膨胀参数与局部图对齐

`FollowPath.ObstaclesCritic.cost_scaling_factor/inflation_radius` 现在**取默认 10.0 / 0.55**，
而局部图是 **5.0 / 0.5** ⇒ `distanceToObstacle()` 反解出的距离被低估约 2 倍（§2.4）。
建议显式写 `cost_scaling_factor: 5.0` / `inflation_radius: 0.5`（**本次 A/B 里一起改过，
但没有单独拆变量 ⇒ 影响未量化**，§7）。

---

## §5 没帮上的（负面结果，省得别人再试）

* `cost_travel_multiplier 4 → 12`：A\* 余量确实变好（健康段 min 0.350 m、边缘<0 为 0%），
  **结局照样卡死**（定位误差 16.2 m、6 次 recovery、停发后盲开 12.7 m）⇒ 只治贴边不治链。
* MPPI critic 加权重（`CostCritic 3→8`、`repulsion 1.5→4.0`、`critical 20→40`）+ 对齐膨胀参数：
  **结局更差**（9 次 recovery、健康段 IMU 198.8 m/s²）⇒ 控制器不是瓶颈；
  看不见的坡道，加多少权重也躲不开。
* `planner:=navfn`：这次过了，但**健康段 IMU 峰值 72.9 m/s²（5 个 >30 样本）**⇒
  是"换路线运气好"，不是机制性修复。
* `nav:=rpp`：69 s 只走 2.6 m（跟不起来），**另一种失败**。
* 全局膨胀带加宽（`0.55→0.85` + `8→4`）：第一次成功、**复跑灾难失败** ⇒
  **不能作为交付配置**（虽然它把执行余量整体抬高，但决定成败的是坡脚那一下）。
* "等 LIO 自己缓过来"：实测**不会** —— 车停住后 66 s 里 `lio_step` 反而涨到 9.8 m/帧
  （静止 + IMU 偏差 ⇒ 积分发散）。
* 指望 GICP 把撞歪的位姿纠回来：**不行** —— 它只发 `map→odom`，而歪的是 `odom→base_link`。

---

## §6 怎么复跑（一条命令一次）

```bash
cd <repo>

# ① 基线（用户那次逐字同参 + spin_speed:=0.0，目标 -12.64 -0.31）
bash tools/scripts/regress/run_nav_clearance_ab.sh --tag base2 --domain 151 --port 11821 \
     --goal -12.64 -0.31 --extra "spin_speed:=0.0" --max-drive-sec 70 --post-goal-sec 20

# ② 本次应用的改动（规划器平滑器）——w_smooth 已经是 0.0，这里给的是"回滚前"的 A/B
bash tools/scripts/regress/run_nav_clearance_ab.sh --tag v_nosmooth --domain 152 --port 11822 \
     --goal -12.64 -0.31 --extra "spin_speed:=0.0" --max-drive-sec 70 --post-goal-sec 15 \
     --kv "nav2_params_sim_planner_smac2d.yaml:GridBased.smoother.w_smooth=0.0"

# ③ 其它变体（每个都是一条命令）
#    全局膨胀带：
bash tools/scripts/regress/run_nav_clearance_ab.sh --tag v_ginfl --domain 154 --port 11824 \
     --goal -12.64 -0.31 --extra "spin_speed:=0.0" \
     --kv "nav2_params_sim_base.yaml:global_costmap.inflation_layer.inflation_radius=0.85" \
     --kv "nav2_params_sim_base.yaml:global_costmap.inflation_layer.cost_scaling_factor=4.0"
#    限速：--kv "nav2_params_sim_base.yaml:velocity_smoother.max_velocity=[1.0, 1.0, 1.5]"
#    结构槽位：--planner navfn / --nav rpp
#    定位策略：--kv "<abs>/gicp_registration/config/gicp_registration_sim.yaml:lost_after_rejections=0"

# ④ 看数字 / 画图（跑完之后，不需要 ROS）
python3 tools/scripts/regress/analyze_clearance.py .tmp_clear/out/base2 .tmp_clear/out/v_nosmooth
python3 tools/scripts/regress/analyze_clearance.py --table .tmp_clear/out/*
python3 tools/scripts/regress/analyze_clearance.py --plot .tmp_clear/out/v_ginfl   # 轨迹余量热图

# ⑤ 场地网格 vs 2D 图 vs 轨迹（离线；§2.2 的数字就是它算的）
python3 tools/scripts/regress/field_mesh_vs_map.py --world RMUC2026 \
        --traj .tmp_clear/out/base2/samples.jsonl
```

产物（`.tmp_clear/out/<tag>/`）：`launch.log`（**带墙钟时间戳**）、`samples.jsonl`（10 Hz 逐样本：
融合位姿/LIO 位姿/`map→odom`/真值位姿/两张图的余量/`/scan` 最小距离/IMU/三路 `cmd_vel`/接触残差/
recovery 数/TF 陈旧度）、`summary.json`、`plans.jsonl`、`gate.txt`（GICP 拒帧/失效/结局，带时间戳）、
`used/`（本次实际生效的参数文件副本；跑完自动还原并逐文件 diff 核对）。

---

## §7 修复落地：感知侧三处改动（2026-10-07）

> §0–§6 是**归因**（本文结论：撞的不是地图上的墙，是"地图/雷达/代价图里都不存在的坡道面与低矮台阶"）。
> 这一节是**修法**与实测，实施细节/阈值真源/回退见
> [`docs/ground_segmentation_slots.md``ground_segmentation_slots.md` §10–§14。

### 7.1 三处改动（一个变量一个变量地量）

| # | 改什么 | 文件/键 | 前后实测 |
|---|---|---|---|
| ③ | **实时链路加"坡度/台阶"判据**（原来只有离线 `pcd_to_nav2_map.py` 有） | 新包 `src/perception/rm_perception/rm_ground_traversability/`（header-only）+ 两个地面分割节点各接一次；阈值真源 `src/bringup/rm_nav_bringup/config/traversability_criteria.yaml` | 撞击点 `map(−3.6,3.1)±0.6 m` 内：判据点 **0 → 3（p50）/ 18（max）** 个，最近判据点到真值台阶格 **0.10 m**；平地假障碍 **Δ≤0.02 pp**；可行驶倒角 15–22° **+0.2~0.5 pp**；35–60° 陡面 patchwork **66.8% → 84.4%** |
| ② | `p2l` 高度带上界 | `pointcloud_to_laserscan/config/laserscan_params.yaml: max_height 0.1 → 1.0`（离地 0.326 → **1.226 m**） | 同一批帧：0.40–1.00 m 与 ≥1.00 m 两档方位覆盖 **0% → 100% / 98.5%**；`/scan` 有限束数 linefit **856.8 → 934.7（+9.1%）**、patchwork **1113.8 → 1163.5（+4.5%）**；老回波**一根不动**（距离差 p50 = 0.000 m）、**丢掉 0 束** |
| ① | `ground` **槽位默认** | **没改**（仍 `linefit`） | 判据接上之后两者互有胜负（linefit 陡面 92.9% vs patchwork 84.4%、0.10–0.20 m 覆盖 99.9% vs 92.3%；patchwork 平地假障碍 4.3% vs 12.0%、CPU 0.41 vs 1.2 ms）+ patchwork 在出生点留 26~27 个假障碍点（linefit 1 个）⇒ 证据不支持换默认，逐条见 §10.5 |

### 7.2 这篇报告的"根因句"现在要改一个字

§0 第 3 条原话是"修的是建图那一步的高度/地面口径，不是 nav2 参数"。本次实测把这句话**收窄**成两件事：

1. **实时链路**（`/scan` → 局部代价图）**不需要重跑建图**就已生效：判据在节点里，
   撞击点那一带立刻多出判据点（§7.1 ③），`/scan` 立刻多出 40~80 条回波（§7.1 ②）。
2. **全局先验图必须重出一份**：2D 先验图是**离线**从 3D 点云投出来的，老点云里没有的几何，
   重投影不会自己出现。旧图实测（`verify_low_terrain.py`，真值 = STL）：
   全场"台阶/边沿格"被表示 **49.44%**（2822/5708）；撞击点窗口 28/33；
   图上判自由而真值 >0.10 m 的格 **72 505 个**；在图上求最短路（余量≥0.25 m）时，
   路径离撞击点最近 **0.051 m**、无余量约束时沿途最大局部台阶 **0.200 m**（= 计划器在贴着一道 0.2 m 台阶走）。
   重跑建图 + 重投影的命令与验收 recipe：`docs/ground_segmentation_slots.md` §12。

### 7.3 §2.2 那条"撞击点看不见"的直接证据链，现在多了三格

| 环节 | 旧 | 新（判据接上 + p2l 1.0 m） |
|---|---|---|
| `/segmentation/obstacle` | 只有坡面点被判地面后剩下的碎片 | 判据把"相对局部地面高差 >0.15 m 或 >25° 且 >0.05 m"的点**降级**成障碍 |
| `segmentation/step_edge`（新话题，诊断用） | 不存在 | 撞击点窗口内 3~19 点/帧，最近的贴到真值台阶格 0.10 m |
| `/scan`（方位回波） | 0.40 m 以上几何 = **0 个方位** | ≥1.0 m 几何 **98.5% 方位**有回波；每帧多 40~80 束 |

### 7.4 本次**没有**解决的（仍然是 §7 的诚实清单）

* **停发 `map→odom` 之后 nav2 继续用控制器盲开**（§0 第 6 条 / §4.5）：本次**没动**（属于 nav2 侧安全缺口）。
* **接触那一击的物理量级**（`a≈v·slope/Δt`）：感知修好之后规划器不该再安排"以 1.7 m/s 冲上 0.2 m 台阶"，
  但**控制器跟踪误差与切角**仍在（§2.4）⇒ 撞击的**概率**下降，不是被"夹住"。
* **坑（负障碍）/头顶净空**：本次只做"台阶/坡度"，`docs/traversability_plan.md` 里的 B/C 两项仍未实现。

## §8 未验证 / 没做完（别当已验证）

1. **统计显著性**：11 次跑覆盖 9 个配置（基线 `base`/`base2` 与 `v_ginfl`/`v_ginfl2` 各是同一配置跑两次），**除 `v_ginfl`/`v_ginfl2` 外每个配置只有 1 次**。
   这类失败**随机性很大**（同配置 `v_ginfl` 成功、`v_ginfl2` 灾难失败）⇒
   **余量分布是稳的信号，结局是弱信号**；要下"哪个配置更好"的结论，每个变体至少 3 次。
   本文所有"成功/失败"都按单次跑记录，没有做重复性统计。
2. **`v_slow`（限速）**没跑****：§4.2 第 3 条只有物理量级论证（`a≈v·slope/Δt` 与实测 85 m/s² 吻合）
   与"能量 ∝ v²"的推论，**没有实测"LIO 能不能扛 ~42 m/s²"**。
3. **`v_nolost`（`lost_after_rejections=0`）跑了，但没有验到"那条策略"**：
   它 SUCCEEDED、`mo_stale`=0（**根本没触发连续 25 帧**）⇒ 不能用来证明"永不放弃"更好，
   只用来反证"撞击 → 必然丢定位"不成立（§3.3）。§4.4 的结论仍然由**契约 + 机制**推出
   （发散的边是 LIO 的 `odom→base_link`，GICP 不可发），旁证是 §3.2 第 2 条
   （冻结 TF 下 nav2 用控制器照开 110 m）。
   若要真正验证"永不放弃 vs 25 帧放弃"，需要一个**能复现 25 帧连拒**的装置（例如
   人为把 LIO 打偏），本次没有。
4. **低矮结构 keep-out 层没装进栈**：只做了"从 PCD 生成掩码 + 目视 + 起点/目标连通性检查"，
   朴素阈值有 81.5 m² 误报（§4.2 第 2 条）⇒ 需要形态学过滤 + 逐场景验证，**未验证**。
5. **`w_smooth=0`（本次应用的改动）**：
   * 治"贴边"有实测（计划到带 0.25→0.335 m、执行 p50 0.57→0.85 m），**但只有 1 次跑**；
   * **"折线路径是否影响跟踪质量"没有定量 A/B**（只有一次跑到达目标、0 recovery）；
   * `w_smooth=0.2` 这种"折中值"**没测**。
6. **实车**：所有数字都来自 Gazebo（`planar_move` 速度控制 + 无悬架）。真车有悬架/轮胎，
   坡脚冲击会小得多；但"**坡道面被 linefit 当地面删掉、2D 图上看不见坡**"这一条
   **大概率同样成立**（同一份 `segmentation_sim.yaml` 口径），**未在真车数据上验证**。
7. **`ObstaclesCritic` 膨胀参数不一致**（§2.4/§4.5）：本次 A/B 里和其它三项一起改的，
   **没有单独拆变量** ⇒ 影响未量化。
8. **`field_mesh_vs_map.py`**：只验证了 `--world/--stl/--map/--traj` 这条主路径
   （`--cell` 非默认值、`--stl-lift/--stl-scale` 非默认值未验证）。
### 7.5 整栈 nav 复跑（2026-10-07，用户同参 + 目标 `(-12.64,-0.31)`）

三次跑都是**无头、一次 bash 调用**、`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、
独立 `GAZEBO_MASTER_URI`、`nav_rviz:=False`、收尾发零 Twist：

| 跑 | 先验图 | 结局 | 首次 IMU>30 | 首次撞击簇峰值 | 撞击后 `loc_err`>0.3 | `map→odom` 停发 | 里程 / 目标 |
|---|---|---|---|---|---|---|---|
| `base2`（**无修复**，§1） | `map/RMUC2026.yaml`（旧） | 撞 → 定位发散 → 盲开 | t=11.8 s（79.3） | **86.4 m/s²** | 12.5 s 起 | 是（15.9 s） | 12.86 m / 未到 |
| `lt_fix_old`（**修复**） | 同上（旧） | 撞 → 定位发散 → 盲开 → 二次撞击 | t=11.9 s（40.2） | **68.2 m/s²（−21%）** | 13.5 s 起 | 是（1 次 stale） | 20.15 m / 未到（90 s 超时） |
| `lt_fix_newmap`（**修复 + 重跑先验图**） | 本次重跑 PCD 投影出来的新图 | **规划不出来** | 无（最大 19.3） | — | 无（最大 0.09） | 否 | 0.55 m / `status=6 ABORTED`、`plans=0` |

命令（可复跑）：

```bash
# ① 旧先验图（= 用户现在那份）
bash tools/scripts/regress/run_nav_clearance_ab.sh --tag lt_fix_old --domain 171 --port 11871 \
     --goal -12.64 -0.31 --extra "spin_speed:=0.0"
# ② 重跑先验图（新图放测试目录，**不覆盖**用户资产；map_yaml 参数见 §7.6）
bash tools/scripts/regress/run_nav_clearance_ab.sh --tag lt_fix_newmap --domain 172 --port 11872 \
     --goal -12.64 -0.31 --extra "spin_speed:=0.0 map_yaml:=$PWD/.tmp_lowterrain/prior/RMUC2026_lt_lowest.yaml"
```

**三条读法（这是本次最重要的发现，别读漏）**：

1. **修复把首次撞击的垂直冲击降了 21%（86.4 → 68.2 m/s²），但没有消除它**。
   这一下的机制就是本文 §2.2 的 `a ≈ v·slope/Δt`：**以 ~1.7 m/s 撞上 ~10% 坡脚**的刚体垂向冲击。
   坡道本身是**可行驶**的（本车能爬 23°，判据的硬要求就是"不许把可行驶坡面标成障碍"）
   ⇒ **障碍物标记这套手段治不了这一下**；要治得靠"把坡度值发布出去 → 限速/代价"
   （`docs/traversability_plan.md` §9 表里"坡度连续量没做"那一条）。
   也就是说：修复解决的是"**看不见**"（台阶/边沿/0.4 m 以上的墙），不是"**看见了但还开那么快**"。
2. **撞击之后的链条原样复现**：LIO 发散（13.5 s `loc_err`>0.3）→ `map→odom` 停发 →
   nav2 用**控制器**盲开 → 二次撞击 **239 m/s²**、总里程 20.15 m、90 s 没到目标。
   这正是 §0 第 6 条 / §7.4 登记的 nav2 侧缺口（本次**没动**，属于另一件事）。
3. **"重跑先验图"那一跑反而跑不起来**（`plans=0`、ABORTED、IMU 全程 <20）：新图
   **过度占用**（128.4 m² vs 旧 /scan 图的 22.7 m²）⇒ 起点/目标被膨胀带吃满 ⇒ 规划器拒绝。
   根因**不是判据**，是**累积点云的垂向拖影**：真值完全平坦（0.000±0.001 m）的 `map(−5,2)` 处，
   `PCD/RMUC2026_lt.pcd` 的 z 跨度 **0.154 m**、单个 0.05 m 细格内 z 跨度中位 0.085 / p90 **0.169 m**
   ⇒ 任何"zmax − 局部地面 > 0.15 m"的判据都会把它当障碍。把 `--height-threshold` 抬到
   0.20 / 0.25 只能把占用从 51 350 格降到 45 067 / 41 070 格（**不够**）。
   ⇒ **换先验图之前必须过 `verify_low_terrain.py` 的"可行驶斜面被误占率 + 图上最短路能走通"两关**；
   **实时链路那一半（缺陷 ②③）不需要新先验图就已经生效**。

### 7.6 新增 launch 参数 `map_yaml`（为上面的对照跑而加，默认行为不变）

`map_yaml:=/abs/path.yaml` ⇒ 只把 nav2 的 **2D 先验图**换掉（默认仍是
`<rm_nav_bringup share>/map/<world>.yaml`）⇒ 可以在**不覆盖用户资产**的前提下对照新旧图。



---


## 来源 3/19：`nav/obstacle_judgement_map.md`


> 2026-11-08。只讲"障碍"这一条线，不含轨迹/控制。
> 【代码】= 读源码确认。

## A. T-DT 内部：障碍是怎么被"用"的

| 环节 | 位置 | 代码/逻辑 |
|---|---|---|
| 二值化 | `yastar.cpp:404 setMap(u_char*)` | `occMap = (pixel == 0) ? 1.f : 0.f` |
| **判障碍（核心）** | `yastar.cpp:936 isInObstacle(x,y)` | `return mask.masked(x,y) \|\| occMap(y,x) > occThs(0.5);` |
| 世界坐标点判障碍 | `yastar.cpp:185 pointInObsticle(x,y)` | 世界→格索引 → `isInObstacle` |
| 线段判障碍 | `yastar.cpp:151 lineInObsticle(a,b)` | Bresenham 逐格 `isInObstacle` |
| 代价图入口 | `yastar.cpp:760 initCostMap(sparse)` | 障碍 ⇒ `costMap = INF`；自由 ⇒ `decay(dist)+1` |
| 动态障碍（mask） | `yastar.cpp:451 setMaskMapTimed(map,name,duration,maxCount,value)` | 把 mask 存进 `biasMask` 层，带生命期/次数 |
| mask 层数 | `yastar.cpp:438 setMaskNumLayers(n)` | 最多 n 层 |
| mask 查询 | `isInObstacle` 里的 `mask.masked(x,y)` | 任一层命中 ⇒ 障碍 |
| 走廊扩张 | `sfcSquare.cpp:355 _getBoundSquare2` | `if(map(yint,xint)==0) return {x,y,x,y};` 逐行扩方盒 |
| MinSnap 碰撞检查 | `minimumSnap.cpp:702 lineInObsticle` | 用 `MinimumSnap` 自己那份 `map_`（另一份拷贝） |

**要点**：T-DT 内部只有**两份**障碍输入 —— `occMap`（静态二值图）与 `mask`（动态层）。
没有点云、没有 costmap、没有高度带概念（高度在**上游数据**里就已经被过滤掉了）。

## B. 我们喂给 `occMap` 的静态图

| 路径 | 位置 | 代码 |
|---|---|---|
| 从 `/map`（OccupancyGrid） | `planner.cpp:341` | `occ = (v >= 65);` ← **0=free / 100=occupied** |
| 从 PGM 文件 | `planner.cpp:389` | `occ = (px == 0);` ← **0=黑=障碍** |
| 外扩 2 格 | `planner.cpp:327-333` | `P=2`, `mocc_` 初值 1（障碍），origin −2·res |
| 控制器整图 | `controller.cpp:315` | `occ = (map_->data[si] >= 65)` |
| 控制器窗口 | `controller.cpp:277` | 同上 |

## C. 我们喂给 `mask` 的动态障碍

| 环节 | 位置 | 逻辑 |
|---|---|---|
| 订阅 | `planner.cpp:61` / `controller.cpp:65` | `/plan_mask`（`OccupancyGrid`, QoS(1)） |
| 转 mask | `planner.cpp:150-156` | `mask = (v > 50) ? 0 : 255` → `setMaskMapTimed(mm,"dyn",1.0,16,0)` |
| 控制器叠加 | `controller.cpp:277` / `315` | `if (!occ && mask_ok) occ = mask_->data[si] > 50;` |
| 几何校验 | 两者 | 分辨率/原点/宽高必须与 `/map` **完全一致**，否则整层不用 |
| 时效校验 | — | **没有**（停发后会用旧 mask） |

## D. `/plan_mask` 是谁产出的（感知链）

```
/livox/lidar/pointcloud (PointCloud2, livox_frame)
   │  patchwork  ground_segmentation_sim_chassis.yaml
   │    sensor_height 0.188, min_range 0.2, max_range 20.0, th_dist 0.13
   │    自击掩膜: 5 个车身 AABB + 近场死区 r_xy≤0.34 m 且 z≥−0.158 m
   ├─► /segmentation/obstacle
   └─► /segmentation/ground
         │  cloud_crop（3 个 AABB，base_link）
         ├─► /segmentation/obstacle_cropped
         │      │  pointcloud_to_laserscan
         │      │    min_height −1.0 / max_height 1.0（雷达系 ⇒ 离地约 [−0.77,+1.23] m）
         │      │    range_min 0.05 / range_max 10.0, angle ±π, 增量 0.0043
         │      │    target_frame ""（不做 TF）
         │      └─► /scan ──► nav2 costmap 的 obstacle_layer
         │
         └─► mask_builder（`rm_tdt_planner/src/mask_builder.cpp`）
               :22-29  参数 res 0.05 / 560×313 / origin(-19.9,-8.89) / z 0.10~1.20
               :75     `std::fill(grid_.begin(), grid_.end(), 0);`   ← **每帧清空**
               :97-98  TF 到 map（`use_tf`，按点云 stamp）
               :103    高度过滤 `p.z() < zmin_ || p.z() > zmax_`
               :110    命中格写 `100`
               :127    publish `/plan_mask`（与 `/map` 同几何）
```

## E. 两条线各自的"看不见"

| 消费者 | 用的障碍 | 高度带 | 距离 | 看不见什么 |
|---|---|---|---|---|
| **A\*（规划器）** | `/map` + `/plan_mask` | mask 0.10~1.20 m | ≤ 20 m（patchwork） | <0.1 m 的矮沿、>1.2 m 的悬挂物、20 m 外 |
| **Kastar（控制器）** | 同上 | 同上 | 同上 | 同上 |
| nav2 costmap | `/map` + `/scan` + stvl | p2l 高度带 [−0.77,+1.23] | `/scan` ≤ 10 m | 10 m 外（对 T-DT 无影响，因为 T-DT 不用它） |

## F. 结论

> **T-DT 内部只吃"一张二值图 + 一层 mask"**；
> 我们也是这两样，**区别只在喂进去的东西**：
> 图是 PGM（同源），mask 是 patchwork 实时点云投的 2D 图（z 0.1~1.2 m、≤20 m、每帧重算）。

## G. 2026-11-08 改为直接用 zlio 的点云（实测）

### 改动

`bringup_sim.launch.py` 里 `mask_builder` 的输入从 patchwork 的 `/segmentation/obstacle`
换成 **zlio 后端的 `/localization/lio_world_cloud`**（已经是 map 系，不需要 TF）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `mask_input_topic` | `/localization/lio_world_cloud` | 想回 patchwork：`mask_input_topic:=/segmentation/obstacle` |
| `mask_z_min` | **−0.02** | zlio map 系里地面实测 ≈ **−0.14** ⇒ 约等于“离地 0.12 m” |
| `mask_z_max` | **1.10** | 约等于“离地 1.24 m” |

### z 带是怎么标定的（实测 world cloud 的 z 分布）

```
点数 2194, frame_id = map
z p5  = −0.142      ← 地面
z p25 = −0.037
z p50 = +0.156
z p95 = +1.558
z∈[0.10,1.20): 975 点 (44.4%)
```

⇒ 地面在 z≈−0.14；原来的 `[0.10,1.20]` 在 zlio 系里等价于**离地 0.24~1.34 m**（偏高），
所以改成 `[−0.02, 1.10]` ≈ **离地 0.12~1.24 m**。

### 实测

```
[plan_mask] 560x313 @ 0.05 m, z∈[-0.02,1.10], in=/localization/lio_world_cloud
/plan_mask: 障碍格 1003
目标 (2.0,-2.5) → SUCCEEDED 用时 19.4 s
```

对比之前（patchwork 供 mask）：26.4 s ⇒ **少了 patchwork 这一环，也少了 TF**。


---


## 来源 4/19：`nav/recovery_fsm_plan.md`


## 为什么放规划器（而不是控制器）

| | BIT | 我们（本设计） |
|---|---|---|
| 恢复动作 | 生成一段**恢复轨迹**并跟踪 | 同：生成一段轨迹，走 `/plan` 发布 |
| 优势 | 能"换轨迹"，不是只后退 | 同 |
| 频率 | 20 Hz 规划器 | 2 Hz 规划器（恢复态可提到 5 Hz） |

控制器只负责执行 `/plan`，不参与决策。

## FSM

```
NORMAL ──触发──► SUSPECT(计数) ──满足──► ESCAPE ──成功──► REPLAN_WAIT ──► NORMAL
   ▲                                          │
   └────────────── 失败计数 < 3 ──────────────┘
                                              └── 失败 3 次 ──► FAILED（停车 + 日志）
```

## 触发条件（四条，覆盖你列的三点）

| # | 触发 | 判据 | 需要的输入 |
|---|---|---|---|
| T1 | **无进度** | 2 s 内位移 < 0.05 m（跟踪态） | 自身位姿历史 |
| T2 | **规划连续失败** | 5 s 内 ≥3 次 `snap=0` 且门控拒绝 | 规划器内部计数 |
| T3 | **跟踪误差大** | 实际位移方向与计划切线夹角 > 60° 持续 2 s | 位姿 + 计划 |
| T4 | **定位不可信** | zlio `localization_state` 非 NORMAL | `/backend_node/localization_state` |

## 恢复动作（ESCAPE）

1. 取梯度：
   - `esdf_en: true` 时用 `MapQueryInterface::evaluate(pos, dist, grad)`（**3D**）；
   - 否则退回我们自己的 2D 倒角梯度。
2. 方向：`d = normalize(grad.xy())`（车是平面的，z 分量不用）；
3. 生成一段 **1.5 m** 的直线轨迹（按 `escape_speed=0.3 m/s`、梯形加速），
   用现有 `trajectory_plan` 格式发布 ⇒ 控制器直接跟；
4. 同步把这段轨迹当作 `/plan`（nav2 侧一致），**不走 MinSnap**（避免被走廊拒绝）。

## 退出与升级

| 条件 | 动作 |
|---|---|
| 退出 1.5 m 且新位置 ESDF > 0.5 m | `backbone_valid_=false` ⇒ 强制重算骨架 ⇒ REPLAN_WAIT ⇒ NORMAL |
| 退出后仍然 T1/T2 | 失败计数 +1，继续 ESCAPE（换一个方向：取当前梯度的垂直方向） |
| 失败 3 次 | FAILED：发零速、日志 `恢复失败：目标不可达或被困死` |

## 悬空/多层（3D 的部分）

- 只有 `esdf_en: true` 才有 3D 梯度；**但脱困只用 xy**；
- **悬空判断**靠 ProjectionLayer 的 `PASS_UNDER`（已接）：它决定"这一格能不能走"；
- 因此：**3D 只影响"能不能走"，不影响"往哪退"**。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `recovery_enable` | true | 总开关 |
| `stuck_time` | 2.0 | 无进度判定时长 |
| `stuck_eps` | 0.05 | 位移阈值 (m) |
| `escape_dist` | 1.5 | 单次退让距离 (m) |
| `escape_speed` | 0.3 | 退让速度 (m/s) |
| `escape_clear` | 0.5 | 认为脱离的 clearance (m) |
| `max_escape` | 3 | 最大尝试次数 |

## 代码落点

| 文件 | 改动 | 量 |
|---|---|---|
| `planner.cpp` | FSM 状态 + 四个触发 + 生成 escape 轨迹 + 发布 | ~180 行 |
| `planner.cpp` | 订阅 `/backend_node/localization_state`（T4） | ~15 行 |
| `controller.cpp` | 不需要改（只跟 `/plan`） | 0 |

## 风险

| 风险 | 缓解 |
|---|---|
| 2 Hz 规划器做恢复响应慢 | 恢复态把规划频率提到 5 Hz |
| escape 轨迹撞墙 | 生成后用**零容差门控**校验；失败则换方向 |
| 与 nav2 BT 的 recovery 冲突 | BT 里禁用 Spin/BackUp，统一由我们的 FSM 处理 |
| 定位不可信时仍乱动 | T4 触发时只停车、不逃逸 |


---


## 来源 5/19：`nav/slope_speed_limiting.md`


> **一句话**：感知现在把"车前 1~3 m 走廊里的**坡度/台阶连续量**"按一个用实测标定过的物理模型
> 换成**速度上限**，发成 nav2 原生的 `nav2_msgs/msg/SpeedLimit` 给 `controller_server`
> （它自己转给 MPPI 的 `setSpeedLimit()` 缩放 `vx_max/vy/wz`）⇒ **命令链上没有新节点、
> 不产生 `/cmd_vel`、不停车、不取消目标、没有 watchdog**。整栈 A/B（用户同参 + 目标
> `(-12.64,-0.31)`）：坡脚那一击 **110.8 → 50.8 m/s²**，`map→odom` 两次都没停发、
> LIO 都没发散，ON 也**到点成功**；代价是全程速度 p50 1.23 → 0.71 m/s（`|v|<0.6` 的时长 4.8 → 9.4 s）。
> **这一击是被压低（−54%）而不是被消除**：目标 ≲30 m/s² 还没达到（详见 §0/§3/§7）。
>
> **★ 2026-10-07 提速档**：本文 §3 的"慢"在 `map(RMUC2026_v3_spl)` 这张图上更严重（旧表 **0/4 到点**、
> `|v|<0.6` 占 61~73 s）。已把 `speed_limit_*` 改成 **提速档**（peak 30→45、floor 0.45→0.60、
> 死区 4°→5.5° / 0.04→0.06 m、两张表按同一公式重算），实测 **2/2 到点、`|v|` p50 1.16~1.19 m/s、
> `|v|<0.6` 0.3~0.6 s、平地回归只差 0.2 s**：见 `docs/lio_divergence_no_impact.md` §3/§4
> （那里同时给了"限速≠防撞"的实测证据与回退方法）。
>
> ⚠ **后记（同日）**：`RMUC2026_v3_spl.{pgm,yaml}` 已被提升为默认 `map/RMUC2026.{pgm,yaml}`
> （逐字节相同）⇒ 上面说的"在 `map(RMUC2026_v3_spl)` 这张图上"= 现在的默认图。
> 提升依据 / 旧份 / 回滚见 `docs/map_assets.md` §3 / §4.4。

---

## 0. 结论（先看这段）

| 问题 | 答案 |
|---|---|
| 机制落在哪 | `rm_ground_traversability`（header-only）扩展 + 两个地面分割节点各接一次；**单一真源**仍是 `src/bringup/rm_nav_bringup/config/traversability_criteria.yaml` |
| nav2 有原生机制吗 | **有**：`nav2_controller` 1.1.20 声明 `speed_limit_topic`（默认 `speed_limit`）并把 `SpeedLimit` 转给控制器插件的 `setSpeedLimit()`；MPPI 的实现在 `optimizer.cpp`：按比例**缩放 `vx_max/vx_min/vy/wz` 四个约束**（`0.0` = `NO_SPEED_LIMIT` ⇒ 恢复基准）。⇒ 用它，**没有自己写 governor、没有新节点** |
| 第一击消除了吗 | **没有，是压低**：`OFF` 首次撞击簇峰值 110.8 m/s² → `ON` 50.8 m/s²（−54%）。目标（≲30 m/s²）**在整栈实测里还没达到**，原因与下一步见 §5/§7 |
| 会不会把车"限停" | 不会：速度地板 `speed_limit_floor_mps: 0.45`（**不许配 0**，校验脚本会拦）；实测 ON 全程 `\|v\|` 的 p50 = 0.71 m/s、`max` = 1.87 m/s、仍然到点 |
| 平地会不会变慢 | 离线合成扫掠：平地（含 5 mm 噪声）**一次都没限速**；整栈回归跑见 §3.3 |
| 回退 | 一个键：`speed_limit_enable: false`（或 `traversability_enable: false` 连判据一起退） |

---

## 1. 机制：从"连续量"到"速度上限"，以及它住在哪

### 1.1 数据流（谁算什么）

```
/livox/lidar/pointcloud ──► 地面分割节点（ground:=linefit|patchwork，**同一个节点内**）
                                │
                                ├─① 判据 applyFrame()：坡度/台阶 ⇒ ground 降级 obstacle（**这次没改**）
                                │     复用：局部地面 g(x,y) = 0.20 m 粗格内各 0.05 m 细格"最低点"的 p05
                                │
                                ├─② describeCorridor()：同一帧、**同一份粗格缓存**上的
                                │     「车前/车后 1~3 m 走廊」逐格连续量：
                                │       · slope_deg  局部地面坡度（复用 slopeOf()，0.40 m 基线）
                                │       · dtan_deg   **坡度变化**（相对最近 0.40 m 的参考坡度）
                                │       · step_m     **台阶残差** = 格内抬升(p90) − 该格坡度能解释的部分
                                │
                                ├─③ SlopeSpeedLimiter::update()：三条规则 → 取最紧 → 刹车距离界 → 速率限制
                                │
                                └─④ 发 nav2_msgs/SpeedLimit（绝对值 m/s；0.0 = NO_SPEED_LIMIT）
                                       │
                         controller_server（订阅 speed_limit_topic）─► MPPI: setSpeedLimit()
                                       │                                  ⇒ 缩放 vx_max/vx_min/vy/wz
                                       └─ /cmd_vel_nav ─► velocity_smoother ─► /cmd_vel（**唯一发布者没变**）
```

**为什么前瞻用"朝向走廊"而不是"沿规划路径"**：这两个地面分割节点**不订阅 `/plan`、不查 TF** —
那是它们"零额外故障面"的前提（`docs/ground_segmentation_slots.md` §2 的契约）。云本身在
**传感器帧**里，而 livox 安装 `rpy = 0 0 0`（`measurement_params_sim.yaml`）⇒ 帧的 +x 就是车头。
要用规划路径做前瞻，就得把点云/路径互转到同一帧 ⇒ 给感知加 TF + 话题依赖；收益只是"转向时少看
一点不相关几何"。⇒ 选**朝向走廊**（半宽 `0.28 m` + 随距离张开 `8°`，覆盖转弯扫掠），
零依赖、零时序问题。

**为什么要按运动方向翻转走廊**：`(见 §5 第 1 条)` 实测两次整栈跑里**最重的一击都发生在倒车时**
（`v<0`）。MID360 在车顶、走廊只朝一个方向 ⇒ 只看车头就完全看不见车尾后面那个坡脚。
现在**只订阅** `/odom` 的速度符号（`speed_limit_direction_topic`，拿不到就回退"只朝前"），
`v<0` 时走廊朝 −x。契约不变：`/cmd_vel` 的发布者仍然只有 `velocity_smoother`。

### 1.2 为什么必须有"已承诺特征 + 距离前推"（这一条是能不能真减速的关键）

只用"特征在 d 米外 ⇒ 现在允许 `sqrt(v_req²+2a·d)`"是**不够**的：雷达**看不见车下与近处地面**
（MID360 下视 −7.22°、离地 0.226 m ⇒ 平地最近看得见 `0.226/tan7.22° = 1.78 m`），
所以测量到的特征距离**永远 ≥1.0~1.8 m** ⇒ 上限永远降不到 `v_req`：离线扫掠里表现为
"过坡脚时仍以 1.2~1.4 m/s 通过"（只把 86 压到 ~70）。做法：

1. 某帧测到"比**当前承诺**更紧"的定位型特征（坡度变化 / 台阶残差）⇒ **承诺**它（记住 `v_req` 与距离）；
2. 之后每帧把剩余距离按 `上一帧上限 × hold_decay_factor(1.0) × dt` **前推**；
3. 前推到 0（车已到/过了它）才释放；放开（升速）受 `release_mps2` 限制。

承诺距离取**该格的远边**（`d + 0.20 m`）：判据说"这一格里出现了跃变"，跃变真实位置在格内。
⚠ 前推用的速度是**发布出去的上限**（≤ 实际车速 ⇒ 方向上保守），代价与残余风险见 §7。

**绝对坡度规则（兜底）为什么必须单独存在、又为什么"只在没有承诺时"参与**：
坡脚可能正好落在盲区里（此时走廊里**全是坡面**、`dtan≈0`）⇒ 没有它就完全不减速；
但一片连续坡面的测量距离每帧都刷新成 ~1.5 m，若让它在承诺生效期间参与，会把已经承诺的减速
顶回 ~1.4 m/s（离线扫掠实测）⇒ 只在"没有承诺"时用。

### 1.3 三条规则（都在 `SlopeSpeedLimiter::update()` 里，共用同一张速度表）

| # | 规则 | 量 | 为什么是它 |
|---|---|---|---|
| ① | **坡度变化** | `atan|Δtanθ|`（相对最近 0.40 m 参考坡度）≥ 4° | 撞击来自**坡度变化**（`a≈v·Δtanθ/Δt`）：恒定坡面上行驶没有垂向速度突变 ⇒ **不该因为"我在坡上"而限速** |
| ② | **台阶残差** | 格内抬升(p90) − 该格坡度能解释的抬升 ∈ [0.04, 0.35] m | 窄尺度：整格内完成的尖台阶不会改变邻格地面 ⇒ 坡度描述子看不见它。>0.35 m 认成墙 ⇒ 交给规划器 |
| ③ | **绝对坡度**（兜底） | 局部坡度 ≥ 2°（且无承诺在生效） | 坡脚落在雷达盲区时的唯一提示 |

**抗噪**（真实点云很稀：单帧走廊只有 7~20 个粗格、集中在一两个距离带）：
· 格内抬升取**细格最高点的 p90**（不是 max，避免单点离群：实测 max 噪声可达 0.15 m）；
· 定位型特征要过**支撑格数**（同一 0.40 m 距离带内 ≥2 格一致；台阶 ≥2× 死区时单格也认）；
· `Δ坡` 死区 4°、台阶死区 0.04 m。

---

## 2. 速度表：物理推导（一个实测锚点标定）

**模型**：刚性轮在坡脚/台阶沿处，垂向速度在接触建立的 `Δt` 内从 0 变成 `v·tanθ`
⇒ `a_peak ≈ v·tanθ/Δt`。

**锚点（实测）**：`docs/path_clearance_and_contact.md` §2.2 —— 车以 `v≈1.7 m/s` 冲上
`map(−3.6,3.1)` 的 ~10% 坡脚，IMU `|a|` **79~90 m/s²**（车静止时是 9.8）。
⇒ 反算 **`Δt_eff = v·tanθ/a = 1.70×0.10/86 = 1.98 ms`**（单一真源里的
`speed_limit_anchor_v_mps/anchor_tan_slope/anchor_peak_mps2`，校验脚本每次都会重算它）。

**目标**：把预期峰值压到 `peak_target = 30 m/s²`（= 实测发散那一击 86 的 1/3；
`docs/path_clearance_and_contact.md` §3.5 的读法：决定成败的是"这一次有没有在坡脚挨那一下"）：

```
v ≤ a_target · Δt_eff / tanθ = 30 × 0.00198 / tanθ = 0.0593 / tanθ     （m/s）
```

| 坡度 θ | 2° | 3° | 4° | 5° | 6° | 8° | ≥10° |
|---|---|---|---|---|---|---|---|
| 表值 (m/s) | 1.66 | 1.10 | 0.83 | 0.66 | 0.55 | 0.45 | 0.45（地板）|
| 模型解 (m/s) | 1.70 | 1.13 | 0.85 | 0.68 | 0.56 | 0.42 | 0.34 |
| **预期峰值** (m/s²) | 29.3 | 29.2 | 29.4 | 29.2 | 29.2 | 32.0† | 40.1† |

† = 已到地板 `0.45 m/s`（机构上还愿意走的最低速）。⇒ **模型意义上的"≤30 m/s²"覆盖到 6~7°**；
更陡的一律走地板。台阶表同理（等效坡度 `tanθ_eff = 台阶 / 0.20 m`）：
`0.04 m`（等效 11.3°）的模型解已是 0.30 m/s < 地板 ⇒ 台阶只有"到地板"一档。

**这条曲线的形状为什么重要**：`v(d) = sqrt(v_req² + 2·brake·d)` 是"以 `brake` 减速就**正好在
`d=0`（特征处）降到 `v_req`**"的运动学反解 ⇒ 特征越近、允许的当前速度越低 ⇒
**是"过坡脚前开始减速"，不是"在坡脚上减速"**。`brake_mps2 = 0.4`（物理极限是
`velocity_smoother.max_decel = 4.0`，10× 余量）；离线扫掠挑的值：

| brake (m/s²) | 0.8 | 0.5 | **0.4** | 0.3 |
|---|---|---|---|---|
| 过坡脚速度 (m/s) | 1.18 | 0.90 | **0.70** | 0.65 |

`0.4` 是"够低"与"别一路爬"的折中。校验脚本断言 `brake ≤ |velocity_smoother.max_decel|`、
`speed_limit_vx_max == min(MPPI vx_max 2.5, velocity_smoother.max_velocity[0] 2.0)`。

---

## 3. 实测（整栈、无头、一次 bash 调用）

跑法（与用户那次同参）：`localization:=gicp nav:=mppi planner:=smac2d lio:=small_point_lio
world:=RMUC2026 spin_speed:=0.0`，目标 `(-12.64,-0.31)`，先验图 = 用户当前
`map/RMUC2026.yaml`（**未改动**）。

```bash
# A/B 基线（限速关）——--kv 只临时改 install 里那份真源，跑完自动还原并 diff 核对
bash tools/scripts/regress/run_nav_clearance_ab.sh --tag ssl_off --domain 181 --port 11881 \
  --goal -12.64 -0.31 --extra "spin_speed:=0.0" --max-drive-sec 90 --post-goal-sec 20 \
  --kv "$PWD/src/bringup/rm_nav_bringup/config/traversability_criteria.yaml:speed_limit_enable=false"
# A/B 限速开
bash tools/scripts/regress/run_nav_clearance_ab.sh --tag ssl_on3 --domain 184 --port 11884 \
  --goal -12.64 -0.31 --extra "spin_speed:=0.0" --max-drive-sec 90 --post-goal-sec 20
# 一次看全部指标（本仓新加的分析器）
python3 tools/scripts/regress/analyze_slope_speed_ab.py .tmp_clear/out/ssl_off .tmp_clear/out/ssl_on3
```

### 3.1 A/B 表

| 指标 | `ssl_off`（限速**关**） | `ssl_on3`（限速**开**） | 读法 |
|---|---|---|---|
| IMU 全程峰值 (m/s²) | 110.8 | 50.8 | −54% |
| IMU>30 / >50 样本数 | 7 / 3 | 1 / 1 | |
| 首次撞击簇 | t=16.2 s，110.8 @map(−3.89,2.56)，车速 −1.04 m/s | t=20.5 s，50.8 @map(−3.94,2.91)，车速 **+0.81** m/s（前进） | |
| `loc_err` max (m) | 0.371 | 0.318 | 两次都**没有**发散 |
| `lio_step` / `mo_step` max (m) | 0.202 / 0.037 | 0.195 / 0.010 | 逐帧步长没有爆 |
| `map→odom` 停发次数 | 0 | 0 | 没有"盲开" |
| 结局 | **到点**（status=4，最近 0.25 m） | **到点**（status=4，最近 0.23 m） | |
| 里程 (m) | 21.54 | 22.18 | |
| 接触/阻塞事件 | 2 / 2 | 1 / 1 | |
| 车速 `|v|` p50 / p90 / max (m/s) | 1.23 / 1.82 / 1.88 | 0.71 / 1.36 / 1.87 | **平地不变慢**的代价项 |
| `|v|<0.6 m/s` 时长 (s) | 4.8 | 9.4 | |
| `/scan` 最小距离 p50 / p05 (m) | 0.778 / 0.370 | 0.628 / 0.341 | 余量没变差 |
| 限速器行为 | 不发 | 362 行 `[slope_speed]`；limit p50=0.85、min=0.45、max=2.00；**不限速占比 9%**；`why` 分布 slope_change 122 / step 197 / slope 12 / none 31；实测 `|cvx|` 与上限之差 p50=−0.12、p95=+0.05（"压住"占 66%） | |

### 3.2 撞击点对照（用户那次的位置）

`docs/path_clearance_and_contact.md` §2.2/§7.5 的撞击点是 `map(−3.6,3.1)`；本次两次跑的首个
撞击簇分别落在 `map(−3.89,2.56)`（OFF）与 `map(−3.94,2.91)`（ON）（ON）—— 同一段坡脚/台面边沿。

### 3.3 平地回归（`tools/scripts/localization/run_nav_smoke_regress.sh`）

同一路线（`--localization gicp --nav mppi --planner smac2d --goal -1.0 2.0`）各跑一次：

| 跑 | 结果 | 用时 | 真值 max\|v\| | 里程 | recoveries |
|---|---|---|---|---|---|
| `ssl_reg_off2`（限速**关**，临时改 `speed_limit_enable: false`，跑完还原+diff 核对） | ✅ PASS | **3.2 s** | 1.54 m/s | 2.88 m | 0 |
| `ssl_reg_on`（限速**开**） | ✅ PASS | **3.6 s** | 1.19 m/s | 2.90 m | 0 |

读法：**两次都 PASS、都没有 recovery**；限速开的那次**峰值速度被压到 1.19 m/s（关时 1.54）**，
用时 +0.4 s（≈+12%，这条路线只有 ~2.9 m）。⇒ 平地**不是"完全不受影响"**，而是"限速器在这条
路线上也看到了低地形并小幅压速"；离线上"纯平地不限速"的性质由
`bench/slope_speed_offline.cpp` 的 `平地（对照）` 一行守着（含 5 mm 噪声也不触发）。
要注意：这条路线的相关性只有 1 次跑，**0.4 s 是单次差值、不是统计量**。

```
bash tools/scripts/localization/run_nav_smoke_regress.sh --tag ssl_reg_on --domain 187 --port 11887 \
     --localization gicp --nav mppi --planner smac2d --goal -1.0 2.0
# OFF 对照：临时把真源里的 speed_limit_enable 改成 false（跑完**必须**还原并核对）
sed -i 's/^\(\s*\)speed_limit_enable: true/\1speed_limit_enable: false/' \
     src/bringup/rm_nav_bringup/config/traversability_criteria.yaml
bash tools/scripts/localization/run_nav_smoke_regress.sh --tag ssl_reg_off2 --domain 189 --port 11889 \
     --localization gicp --nav mppi --planner smac2d --goal -1.0 2.0
sed -i 's/^\(\s*\)speed_limit_enable: false/\1speed_limit_enable: true/' \
     src/bringup/rm_nav_bringup/config/traversability_criteria.yaml
```

---

## 4. 参数、文件、回退

**单一真源**：`src/bringup/rm_nav_bringup/config/traversability_criteria.yaml` 的 `speed_limit_*` 一组键
（两个地面分割节点当 ROS 参数文件读；离线出图脚本**不读**这些键）。校验：

```bash
python3 tools/scripts/regress/check_slope_speed_table.py     # 参数一致 + 表↔模型 + 与 nav2 vx_max/max_decel 对账
python3 tools/scripts/regress/check_traversability_criteria.py  # 判据那一套（没动）
```

| 键 | 默认 | 作用 / 什么时候改 |
|---|---|---|
| `speed_limit_enable` | true | **总开关**（回退用）；false ⇒ 完全不发 SpeedLimit |
| `speed_limit_topic` | `speed_limit` | 必须 == controller_server 的 `speed_limit_topic` |
| `speed_limit_vx_max` | 2.0 | = min(MPPI `vx_max` 2.5, smoother `max_velocity[0]` 2.0)；改 nav2 那两处要同步 |
| `speed_limit_lookahead_m` | 3.0 | 前瞻距离。**再大也没用**：平地最近只看得见 1.78 m（上坡面 ~2.6 m） |
| `speed_limit_corridor_half_width_m` / `_spread_deg` | 0.28 / 8.0 | 走廊半宽/张角（车半宽 0.155 + 余量） |
| `speed_limit_slope_change_deadband_deg` | 4.0 | 坡度变化死区（**噪声大就调大**；调小会误触发） |
| `speed_limit_step_deadband_m` / `_ignore_above_m` | 0.04 / 0.35 | 台阶残差死区 / 上限（上限以上认成墙，交给规划器） |
| `speed_limit_min_support_cells` | 2 | 定位型特征的支撑格数（**点云稀疏时保命**） |
| `speed_limit_brake_mps2` | 0.4 | 刹车距离界（越小越提前减速）；必须 ≤ `velocity_smoother.max_decel` |
| `speed_limit_release_mps2` | 1.0 | 放开（升速）速率；防止"过了坡就全油门" |
| `speed_limit_floor_mps` | 0.45 | **速度地板（不许 0）**：慢也要走 |
| `speed_limit_hold_decay_factor` | 1.0 | 承诺特征的距离前推系数（<1 更保守） |
| `speed_limit_direction_aware` / `_min_vx` / `_timeout_s` / `_topic` | true / 0.05 / 0.5 / `odom` | 按运动方向翻转走廊（倒车时看车尾） |
| `speed_limit_slope_knots_*` / `_step_knots_*` | 见 §2 | 速度表本体 |
| `speed_limit_publish_diag` / `_log_period_s` | true / 2.0 | 诊断话题与 `[slope_speed]` 日志节流 |

**回退（三种粒度）**

1. **只关限速**（判据照旧）：`speed_limit_enable: false` ⇒ 不发任何 `SpeedLimit`（速度回到 nav2 自己的
   `vx_max`/`velocity_smoother`）。也可临时 `ros2 param set /ground_segmentation speed_limit_enable false`——
   注意本节点是**构造期**读参数，运行时 set 不生效，要重启节点或改文件。
2. **连判据一起关**：`traversability_enable: false`（回到"没有坡度/台阶判据"的旧链路）。
3. **残留上限的手工恢复**（本节点被 SIGKILL 时最后一次上限会留在 MPPI 里；正常退出会自己发回
   `NO_SPEED_LIMIT`）：`ros2 topic pub --once /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.0}"`

**改动文件一览**

```
src/perception/rm_perception/rm_ground_traversability/include/rm_ground_traversability/low_terrain_classifier.hpp   (+走廊剖面/连续量)
src/perception/rm_perception/rm_ground_traversability/include/rm_ground_traversability/slope_speed_limit.hpp       (新：速度表+限速器)
src/perception/rm_perception/rm_ground_traversability/include/rm_ground_traversability/traversability_ros.hpp      (ROS 接线：发 SpeedLimit)
src/perception/rm_perception/rm_ground_traversability/bench/slope_speed_offline.cpp                                (新：离线合成扫掠，无需 Gazebo)
src/perception/rm_perception/{linefit_ground_segmentation_ros,patchwork_ground_segmentation}/…                     (每帧调用 + nav2_msgs/nav_msgs 依赖)
src/bringup/rm_nav_bringup/config/traversability_criteria.yaml                                                  (**真源**：新增 speed_limit_* 键)
tools/scripts/regress/check_slope_speed_table.py                                                        (新：真源/物理/与 nav2 对账)
tools/scripts/regress/analyze_slope_speed_ab.py                                                         (新：A/B 指标提取)
docs/slope_speed_limiting.md（本文）、docs/traversability_plan.md §9                                     (文档)
```

---

## 5. 什么**没有**帮上忙（别重复走）

1. **"标障碍"这条路**（上一轮做的）：坡面**本身可行驶**，把它标成障碍是错的（`traversability_plan.md`
   §1.3 ①）；实测只把第一击 86.4 → 68.2 m/s²。⇒ 必须限速，不能靠 occupancy。
2. **只朝前看的走廊**：两次整栈跑里最重的一击都发生在**倒车**时（`v = −1.04 / −1.00 m/s`），
   车尾后面的坡脚根本不在走廊里 ⇒ 第一版（`ssl_on`）峰值 66.5、`ssl_on2` 67.7、`ssl_on3` 86.5
   —— 不是限速器没生效，是**它压根没看到那一击的来源**。改成**双向窗口**（`ssl_on4`）之后
   才降到 50.8 m/s²。
   · 顺带一条**没走通**的路：先用"订阅 `/odom` 的 `twist.linear.x` 符号来翻转走廊"（`ssl_on3`），
     机器人确实在倒车（`/cmd_vel` 与真值都是负的），但 `[slope_speed]` 一直打"前进"⇒ **方向判据
     没有生效，根因未定位**（订阅建了没有？时间戳/时钟？符号约定？本次没查到底）。
     ⇒ 默认改成**双向窗口**（不依赖任何话题语义）；方向判据保留但 `speed_limit_direction_aware: false`。
3. **纯"距离刹车界"（没有承诺/前推）**：因为雷达看不见车下与近处地面，上限永远降不到 `v_req`
   （离线扫掠：过坡脚 1.18~1.4 m/s）⇒ 必须"承诺 + 前推"。
4. **让绝对坡度规则一直参与**：会把已承诺的减速顶回 ~1.4 m/s（离线扫掠复现）⇒ 只在无承诺时用。
5. **不做抗噪**（格内取 max、无支撑格数要求）：第一次整栈跑 `ssl_on` 里限速器**全程都在限速**
   （不限速占比 0%），38% 的运动时间压在 0.6 m/s 以下、90 s 没到点。p90 + 支撑格数 + 更大死区
   之后（`ssl_on2`）到点了，但代价仍在（§3.1 的速度列）。
6. **调 nav2 的全局限速**（`velocity_smoother.max_velocity 2.0 → 1.0`）不是替代方案：它不看地形，
   平地也慢——本机制的意义正是"只在低地形前慢"。

---

## 6. 怎么复跑

```bash
# ① 离线（不需要 ROS/Gazebo）：调速度表/死区时看趋势 + 两条硬性质验收
g++ -O2 -std=c++17 -I src/perception/rm_perception/rm_ground_traversability/include \
    -I /usr/include/pcl-1.12 -I /usr/include/eigen3 \
    src/perception/rm_perception/rm_ground_traversability/bench/slope_speed_offline.cpp -o /tmp/ssl_bench
/tmp/ssl_bench          # 期望：平地"平地误限=否"；坡脚/台阶过特征上限 0.5~0.7 m/s
/tmp/ssl_bench -v       # 逐帧曲线

# ② 真源/物理自检
python3 tools/scripts/regress/check_slope_speed_table.py

# ③ 整栈 A/B（§3 的两条命令）+ 指标提取
python3 tools/scripts/regress/analyze_slope_speed_ab.py .tmp_clear/out/ssl_off .tmp_clear/out/ssl_on3

# ④ 平地回归（一次 bash 调用，含起栈/收尾）
bash tools/scripts/localization/run_nav_smoke_regress.sh --tag ssl_regress --domain 185 --port 11885 \
     --localization gicp --nav mppi --planner smac2d --goal -1.0 2.0
```

**看日志**：地面分割节点每帧（变化 >0.03 m/s 或每 2 s）打一条
`[slope_speed] limit=… | 前进/倒车 | why=… d=… Δ坡=… 坡=… 台阶=… v_req=… | 走廊: …`；
诊断话题：`/ground_segmentation/slope_speed_stats`（每帧 JSON，含逐格 `[d, slope, dtan, step]`）、
`segmentation/terrain_slope` / `segmentation/terrain_step`（逐格连续量的点云，intensity 带值）。

---

## 7. 未验证 / 已知残余风险

1. **"第一击消除"还没做到**：整栈实测只把峰值从 110.8 压到 50.8 m/s²（目标 ≲30）。
   最可能的原因（未逐条证实）：走廊里的**坡度/台阶估计仍然很稀**（单帧 7~20 格、集中在一两个
   距离带），4° / 0.04 m 的死区是为了压住噪声而设的，代价是**弱特征不触发**；
   而实测那一击的落点恰好是"台面边沿 + 坡脚"的混合体。
1b. **"拿 `/odom` 的 `twist.linear.x` 符号判运动方向"这条路没走通**：`ssl_on3` 里机器人确实在倒车
   （`/cmd_vel` 与真值都是负的），但 `[slope_speed]` 日志一直打"前进" ⇒ 方向判据没有生效，
   根因**未定位**（订阅建立了吗？时间戳/时钟？符号约定？本次没查到底）。⇒ 改成**双向窗口**这条
   不依赖任何话题语义的修法；方向判据保留但默认关闭（§4/§7）。
2. **承诺特征的前推用"上限"当速度**：仿真里 `planar_move` 完美跟踪 ⇒ 等于真实行进距离；
   实车/被打滑/被挡（车没动但一直发着上限）时会**低估剩余距离、提前释放**。缓解：
   `hold_decay_factor < 1`（更保守）；彻底解需要订阅 `/odom` 的**速度**（本次只用了它的符号）。
3. **只验证了 `nav:=mppi`**：`SpeedLimit` 经 MPPI 的 `setSpeedLimit()` 缩放约束；RPP/DWB/TEB 的
   `setSpeedLimit()` 实现在本仓**没测**（nav2_core 里它是纯虚函数，各插件自己实现，语义可能不同）。
4. **`ground:=linefit` 槽位**：代码路径完全相同（同一个 `TraversabilityRos`），但本次 A/B 只跑了
   默认槽位，**没跑 patchwork 槽**。
5. **只在一张先验图、一个目标、各一次跑**上量的：撞击量级本身是概率性的
   （`docs/path_clearance_and_contact.md` §3.5），单次跑的差值不能当"统计显著"。
6. **`mode:=mapping` 未跑**：新增的是"订阅 + 发 SpeedLimit"，理论上不影响 ground/obstacle 两朵云
   （`applyFrame()` 的判据与降级逻辑一个字没改），但本次**没有**跑 mapping 冒烟。
7. **真车**：所有数字来自 Gazebo（`planar_move` 速度控制、无悬架）。真车有悬架/轮胎 ⇒ 同样的
   坡脚冲击会小很多；但"限速在地形前生效"这件事与平台无关。
8. **`/odom` 方向源在 LIO 发散时会跟着错**：发散时 `twist.linear.x` 的符号可能不可信 ⇒ 走廊可能
   朝错方向。此时兜底是"绝对坡度 + 台阶"两条规则仍在前方走廊上工作（如果方向没错）。
9. **负障碍（坑/落空）**：本文的台阶残差对**下跳**是部分覆盖（格内抬升为负时不算），
   真正的"落空"判据仍是 `docs/traversability_plan.md` §9 里那条空缺。


---


## 来源 6/19：`nav/stvl_local_costmap.md`


> 状态：**已实现 + 已实测（2026-10-05）**；结论见 §4。
> 一句话：给 `local_obstacle` 增加 `stvl` / `stvl_both` 两个取值，让**局部**代价地图也能跑
> SpatioTemporalVoxelLayer（COD 2026 的双图做法），然后用同栈同目标点 A/B 量化它的收益与代价。
> 相关文档：`docs/algorithm_matrix.md` §一（槽位表）/§八.4（实验清单 P1）、
> `docs/cod_nav_2026_deep_dive.md` §A8 + §4-C2、`docs/cod_nav_2026_params_review.md`、
> `docs/cod_nav_2026_migration_worksheet.md`、`docs/smoke_test_runbook.md` §0.3/§7.2、
> `docs/3d_to_2d_survey.md` §七。

---

## 1. 槽位设计（新增了什么）

| 取值 | 局部代价地图生效图层 | 说明 |
|---|---|---|
| `scan`（默认） | `obstacle_layer`(/scan) | **原行为，一字不动** |
| `cloud` | `obstacle_cloud_layer`(/segmentation/obstacle 点云直投) | 原行为，一字不动 |
| `both` | `obstacle_layer` + `obstacle_cloud_layer` | 原行为，一字不动 |
| **`stvl`（新）** | **`stvl_layer`**（3D 体素层，带时间衰减） | 镜像全局那张图的 STVL，= COD 2026 两个 costmap 都上 STVL 的做法 |
| **`stvl_both`（新）** | `stvl_layer` + `obstacle_layer` + `obstacle_cloud_layer` | 三路冗余（最多余、最贵）；**本次未做 bench**，只做了静态真值表校验 |

命名沿用本仓库约定"**一个来源一个名字、不藏子开关**"：`scan` / `cloud` / `stvl` 各只开一路，
`both` = scan+cloud，`stvl_both` = **stvl + both**（读作"stvl 加 both 那两路"）。

### 1.1 机制：为什么是 `enabled` 开关，不是按槽位重写 `plugins` 列表

局部 costmap 的三个障碍层**全部**写进 `plugins`，槽位只切每层的 `enabled`：

```yaml
# src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml（local_costmap 段）
plugins: ["obstacle_layer", "obstacle_cloud_layer", "stvl_layer", "inflation_layer"]
```

这与 global 的 `obstacle_layer` / `stvl_layer` **完全同款**，也是本仓库既有机制。之所以不能按槽位
重写 `plugins` 列表本身：`nav2_common` 的 `RewrittenYaml` 只替换**文件里已存在的叶子键**
（`rewritten_yaml.py:110-122`），且 `convert()` 只会把字符串转 bool/数字（`:175-190`）——
一个字符串值到不了节点（`plugins` 需要 `string[]`，类型不符会直接抛错）。

**唯一真值表**放在 `src/nav/rm_navigation/rm_navigation/launch/navigation_launch.py` 的
`LOCAL_OBSTACLE_LAYER_TABLE` / `GLOBAL_OBSTACLE_LAYER_TABLE`，launch 由它**生成**
`RewrittenYaml` 的 `enabled` 重写表达式（`build_param_substitutions()`）。

### 1.2 `scan` / `cloud` / `both` 的行为保持

- **代价图语义**：这三个取值下 `stvl_layer.enabled = false`，STVL 在 `updateBounds()` 里直接返回
  ⇒ 不 mark、不 clear、不参与 `updateCosts`，对栅格内容零贡献。
- **改不掉的差别（如实说）**：`plugins` 列表多了 `stvl_layer` 一项 ⇒ 该图层会被**构造**并
  `onInitialize()`，即使 `enabled=false` 也会像 nav2 其它图层一样在 `activate()` 时订阅
  `/segmentation/obstacle`（`costmap_2d_ros.cpp:540-560` 对所有插件无条件 `activate()`）。
  **注意这不是新引入的类别**：改造前 `obstacle_cloud_layer` 就已经是"在 plugins 里但 enabled=false"
  的同一形态、且订阅同一个话题 —— 也就是说"局部 costmap 进程本来就订阅着
  `/segmentation/obstacle`"（运行期 `ros2 node info` 可见，见 §3.5）。
  实测代价：`scan` 那一跑的 RTF / `missed rate` 与历史基线同档（见 §3.1），RSS 无增长趋势（§3.5）。
- **静态校验**：`tools/scripts/regress/local_obstacle_truth_table.py` 用**真的 `RewrittenYaml`**
  跑真实 params 文件，断言 `scan|cloud|both` 的生效图层集合与改造前逐位一致。

---

## 2. 参数与来源（逐键）

局部 `stvl_layer` 整块**镜像 `nav2_params_sim_base.yaml` 里 global 的 `stvl_layer`**
（同源 `/segmentation/obstacle`、同高度带、同衰减），只有 3 处偏离，全部标注理由。

主来源两处：
- **COD 2026**：`third_party/cod_nav_2026/src/cod_bringup/params/singlenav2_params.yaml`
  —— 局部段 `200-262`、全局段 `290-352`（**两段数值逐键相同**，即"两个 costmap 同款 STVL"），
  下面简称 **S:行号**。旁证：`docs/cod_nav_2026_deep_dive.md` §A8 的对照表 + §4-C2。
- **我们自己的 global stvl 段**：`nav2_params_sim_base.yaml` 的 `global_costmap…stvl_layer`。

| 键 | 局部取值 | 来源 | 备注 |
|---|---|---|---|
| `plugin` | `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer` | S:201/291 | 与我们 global 同 |
| `voxel_decay` | `0.5` | **S:203/293** | 线性衰减 0.5 s（"人走过"的残影 0.5 s 内消失） |
| `decay_model` | `0`（线性） | **S:204/294** | 1=指数、2=持久 |
| `voxel_size` | `0.05` m | **S:205/295** | 比全局栅格 0.04 略细 |
| `observation_persistence` | `0.0`（只用最新一帧） | **S:207/297** | |
| `max_obstacle_height` | `2.0` m（layer 级） | S:208/298 | 与我们 global 同；COD **源级**另有 `1.0`（S:230/320），STVL 里**源级覆盖 layer 级** ⇒ COD 实际生效的是源级值 |
| `mark_threshold` | `0` | **S:209/299** | 体素高度阈值 |
| `update_footprint_enabled` | `true` | S:210/300 | |
| `combination_method` | `1`（max） | S:211/301 | |
| `origin_z` | `0.0` | S:213/302 | |
| `transform_tolerance` | `0.2` s | S:215/305 | |
| `mapping_mode` / `map_save_duration` | `false` / `60.0` | S:216-217/306-307 | 只用在线地图 |
| `track_unknown_space` | `true` | S:206/296 | 与我们 global 同 |
| **`publish_voxel_map`** | **`false`** | **S:214/304** | ★偏离我们的 global(`true`)：局部 5×5 m @10 Hz 发体素图纯开销；要看体素图用 global 那张 |
| `observation_sources` | `livox_mark livox_clear` | 我们 global 的写法 | COD 是**单源** marking+clearing（S:313-314）；我们拆两路让 clearing 走更宽的垂直 FOV（`livox_clear` 保留 `vertical_fov_angle 1.029 / vertical_fov_padding 0.05 / model_type 1 / decay_acceleration 5.0`） |
| `livox_mark.topic` | `/segmentation/obstacle` | 我们 global | COD 吃 `/livox/lidar_filtered`（S:311）——那是**车上的原始去畸变点云**，我们仿真里对应的就是 linefit 之后的 `/segmentation/obstacle` |
| `livox_mark.obstacle_range` | `3.0` m | 我们 global；COD 源级 `8.0`（S:316/319） | 取 3.0 = 与 local 的 `/scan` 源 6.0 m / 点云源 3.0 m 同档，避免局部图比别的槽位"看得更远" |
| **`livox_mark.min_obstacle_height`** | **`0.0`** | ★偏离 COD `0.15`（**S:229/319**） | 我们 2026-09-24 为 RMUL2026 的 **0.4 m 矮内墙**把 global 改成 0.0；local 若用 0.15 会出现"global 看得见、local 看不见"的分裂 |
| `livox_mark.max_obstacle_height` | `2.0` | ★偏离 COD `1.0`（S:230/320） | 同理，与 global 对齐 |
| `filter` / `voxel_min_points` | `"voxel"` / `0` | S:235-236/325-326 | |
| `expected_update_rate` | `0.0`（不判 stale） | 我们所有障碍源 | 见 `docs/issues_and_findings.md`：0.5 会把"缺一帧"变成 costmap 永久 `isCurrent()==false` |
| `clear_after_reading` | `true` | S:234/324 | |

**未能从 COD 代码里确认的**：COD 的 `stvl_voxel_layer.obstacle_range: 3.0`（layer 级，S:212/302）
与源级 `8.0` 同时存在，按 STVL 的参数层级**源级应覆盖 layer 级**（`docs/cod_nav_2026_deep_dive.md` §A8
"层级陷阱"一段），但我们**没有跑过 COD 的代码去实测**，所以这条只作为"我们为什么不用 8.0"的参考。

---

## 3. A/B 实测（headless，同栈同目标点）

### 3.1 两组实验、四次跑（全部 headless，同一个栈）

栈固定：`world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp nav:=mppi
planner:=smac2d spin_speed:=0.0 nav_rviz:=False`，两次跑**只差 `local_obstacle` 一个词**。
回归：`python3 tools/scripts/regress/nav_smoke_regression.py --goal <G> --localization gicp+local_<标的>`。

- **A 组（任务口径目标点）** `--goal -1.0 2.0`：`scan`（控制组）vs `stvl`（实验组）
- **B 组（补充：贴墙目标点）** `--goal 0.25 -1.5`：机器人从 (0,0) 直着开到 x=+0.25，
  而 RMUL2026 的墙在 x≈+0.50~0.62 ⇒ **终点离墙 0.25 m**，用来回答"近距/盲区到底谁看得见"

| 跑 | goal | `local_obstacle` | PASS | status | 用时(s) | **RTF** | **missed rate** | d_min | recoveries | cmd 峰值 nav(v,w) | chassis(v,w) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| A-scan | -1.0 2.0 | `scan`（控制） | ✅ | SUCCEEDED | 3.4 | **0.75** | **0** | 0.0 | 0 | (1.45, 0.91) | (1.45, 0.91) |
| A-stvl | -1.0 2.0 | `stvl` | ✅ | SUCCEEDED | 3.2 | **0.73** | **0** | 0.0 | 0 | (1.42, 0.83) | (1.42, 0.83) |
| B-scan | 0.25 -1.5 | `scan`（控制） | ✅ | SUCCEEDED | 2.9 | **0.75** | **0** | 0.0 | 0 | (0.58, 0.50) | (0.58, 0.50) |
| B-stvl | 0.25 -1.5 | `stvl` | ✅ | SUCCEEDED | 2.9 | **0.75** | **0** | 0.0 | 0 | (0.55, 0.49) | (0.54, 0.47) |

四条全部 `status=SUCCEEDED`、`d_min=0.0 m`、`recoveries=0`、真值 `pose_delta` 与目标距离一致
（A 组 ≈2.72~2.76 m、B 组 ≈1.70~1.71 m）⇒ **没有一次恢复行为、没有假到达**。

运行期真值（`ros2 param get`，两跑都取）：

```
/local_costmap/local_costmap plugins = ['obstacle_layer','obstacle_cloud_layer','stvl_layer','inflation_layer']
A/B-scan : obstacle_layer=True   obstacle_cloud_layer=False  stvl_layer=False
A/B-stvl : obstacle_layer=False  obstacle_cloud_layer=False  stvl_layer=True
```

日志里 `local_costmap` 构造的图层（`Using plugin ...`）两跑都是四个 —— 即 `scan` 跑里
STVL 图层**被构造但 enabled=false**（见 §1.2 的说明）。

### 3.2 局部代价地图"看到了什么"：探针证据（只订阅，不改栈行为）

探针 `tools/scripts/regress/local_obstacle_probe.py`（2 Hz）对 `/local_costmap/costmap_raw`
（`nav2_msgs/Costmap`，254=真障碍）与 `/global_costmap/costmap_raw` 做环带统计，同时统计两个
**原始来源**（`/scan` 与 `/segmentation/obstacle`）在机器人附近的量。

| 指标 | A-scan | A-stvl | B-scan | B-stvl |
|---|---|---|---|---|
| 采样数 | 88 | 82 | 87 | 83 |
| 局部图 0~0.5 m 真障碍格 (p50/max) | 0 / 0 | 0 / 0 | **7 / 12** | **2 / 3** |
| 局部图 0.5~1.0 m 真障碍格 (p50/max) | **135 / 178** | **20 / 34** | 3 / 13 | 6 / 13 |
| 局部图车后扇区 0.3~1.5 m (p50/max) | 240 / 267 | 35 / 49 | 0 / 0 | 0 / 0 |
| 到最近真障碍的距离 min/p10/p50 (m) | 0.601 / 0.755 / 0.835 | 0.620 / 0.635 / 0.809 | **0.349 / 0.351 / 0.356** | **0.396 / 0.396 / 0.397** |
| 全局图 1 m 内真障碍格 (p50) 〔对照〕 | 58 | 56 | 81.5 | 81 |
| `/scan` 有效回波数 (p50) | 1218 | 1238 | 1091 | 1110 |
| `/scan` 在 0~0.5 m 的回波数 (max) | 0 | 0 | **35** | **25** |
| 点云在 0~0.5 m 的点数 (max) | 1 | 0 | **36** | **28** |
| 点云在 0~0.5 m **且落在 p2l 高度带外**的点数 (max) | 0 | 0 | **0** | **0** |

（完整工具：`python3 tools/scripts/regress/compare_local_obstacle_probe.py a=…jsonl b=…jsonl`）

读法：

1. **A 组这个目标点根本考不到"近距"**：整段导航里车到最近墙面的距离 p10 = 0.72 m（scan）/
   0.64 m（stvl），所以"0~0.5 m 环带里有没有障碍"两边都是 0 —— **用这个目标点无法区分**
   "局部 STVL 是否多看见东西"。这是本次实验最重要的**否定性**结论之一：任务书里"贴墙 0.3 m"
   的场景，用 `--goal -1.0 2.0` 是构造不出来的（要 B 组那种贴墙终点）。
2. **0.5~1.0 m 的墙体标记数差 ~7 倍**（135 vs 20，车后扇区 240 vs 35）：同一时刻、同一段墙、
   同一台车，**局部 STVL 在局部图上留下的"真障碍格"少得多**。原因见 §3.3 的栅格快照：
   STVL 的体素是 **0.05 m**，而局部栅格是 **0.02 m** ⇒ 一个体素只落到一个栅格 ⇒ 墙在局部图上
   变成**点状**而不是连续线；`/scan` 的波束角分辨率 0.0043 rad（0.8 m 处相邻回波间距 3 mm）⇒
   墙是**连续线**。全局图（0.04 m 栅格）那一列只是对照，它含 `static_layer`，不能用来比较
   STVL 的标记密度（这一点也是本次才想清楚：**全局图不能当局部 STVL 的对照组**）。
3. **B 组（贴墙）证明"45 cm 盲区"在当前配置下不存在**：`/scan` 在 0~0.5 m 有 25~35 个回波
   （p2l `range_min` 已在 2026-09-26 由 0.45 一路降到 0.05），两个配置的局部图都在
   **0.35~0.40 m** 处标到了墙 ⇒ 局部 STVL "消盲区"这条**动机已经不成立**（本地图 `scan` 也看得见）。
4. **点云并没有提供 p2l 高度带之外的信息**（B 组"高度带外的点 max = 0"）：`/segmentation/obstacle`
   的点在 livox_frame 下几乎全落在 p2l 的 z∈[-1.0, 0.1] 带内 ⇒ 在本场景里，
   "STVL 吃 3D 全高度、/scan 只吃一条带"这条**收益也是 0**（墙都是高的、地面已被 linefit 去掉）。

### 3.3 墙上标记形态：连续线 vs 点状（原始栅格快照）

B 组两跑都 dump 了"离障碍最近那一刻"的 `local_costmap_raw`（`--dump-grid`），对同一列（墙面
所在列 x≈0.52 m）逐格看 254：

| | B-scan（`/scan`） | B-stvl（STVL） |
|---|---|---|
| 该列 254 格数 / 跨度 | 137 / 192 格（3.84 m） | 70 / 173 格（3.46 m） |
| **占位率** | **71%** | **40%** |
| 连续段 | 5 段，最长 **110 格连续**（2.2 m） | **70 段，全部长度为 1** |
| 空隙 | 主要为 0（首尾各一段大空） | 69 个，长度 1~2 格 |
| 254 最近邻间距 p50 | **0.020 m**（= 1 格，连续） | **0.040 m**（隔 1~2 格，点状） |
| 253（膨胀带）格数 | 13801 | 13923（**几乎一样**） |

（另：两跑的局部栅格里 **255/NO_INFORMATION 格数都是 0** —— `track_unknown_space: true` 是我们
从 global 段镜像过来的值，在 `combination_method: 1`(max) 下它**本可以把整张局部图写成未知**，
实测没有触发；换 world/换 STVL 版本时值得复看这一项。）

⇒ 结论：**局部 STVL 的『致命核心』是周期 ≈0.05 m 的点状线**（0.05 m = `voxel_size`，与
"一个体素 → 一个 0.02 m 栅格"完全吻合），而 `/scan` 是连续线。**但 0.5 m 的膨胀带两者几乎
一样大**（13801 vs 13923 格）⇒ 规划器/MPPI 真正用的代价格几乎没变，这解释了为什么两跑都
一次过、没有任何恢复行为。**风险留给**：任何依赖"致命格连续性"的下游（例如把 lethal 当
碰撞判据的碰撞监控/自定义 critic），在局部 STVL 下会看到一条有 ~0.05 m 缝的墙。

### 3.4 代价：CPU / RTF / missed rate

| | A-scan | A-stvl | B-scan | B-stvl |
|---|---|---|---|---|
| RTF（回归脚本口径） | 0.75 | **0.73（-2.7%）** | 0.75 | 0.75（±0） |
| `Control loop missed its desired rate` 计数 | 0 | 0 | 0 | 0 |
| 话题频率 pcloud/imu/scan/odom (Hz) | 7.7/77.3/8.0/7.7 | 7.7/78.3/8.0/8.0 | 8.0/78.7/8.0/8.0 | 7.7/77.7/7.7/8.0 |
| controller_server RSS 首→末 (MB) | 59.2 → 71.0 | 52.8 → 71.6 | 60.2 → 70.3 | 58.3 → 71.8 |
| 局部图发布/接收（`costmap_raw` 条数） | 174 | 164 | 173 | 165 |

- **RTF 代价 ≈ 1~3%（本案 0.75→0.73）**，`missed rate` 两跑都是 0 —— 本机 28 核、RTF 只有
  ~0.75 时，局部再加一张 5×5 m 的 STVL 并没有把主循环压垮（全局那张 STVL 早就开着）。
- **RSS 没有"STVL 特有"的增长**：`scan` 跑里 STVL 图层虽然被构造（enabled=false），RSS 增长
  与 `stvl` 跑同档（+12 MB / +19 MB，四条都在同一量级）⇒ 没看到"禁用图层仍在无限囤点云"。
- ⚠️ 口径：探针订阅 `costmap_raw` 会让发布端多序列化一张图（两个跑都有，公平），
  所以绝对 RTF 与历史基线（同栈 `gicp-pcl` 的 0.76）**不完全可比**，只做横向比较。



---

## 4. 结论与建议

**结论：在本次的 sim 目标点集合与 RTF ≈0.73~0.75 下，局部 STVL 与 `/scan` 单源
「导航结果上不可区分，局部图内容上可区分且更稀疏」。**

分项：

| 维度 | 判定 |
|---|---|
| 能不能跑通 | ✅ 完全等价：A/B 两组四次跑全部 PASS/SUCCEEDED、`recoveries=0`、`d_min=0`、`missed=0` |
| 到达时间/速度包线 | 无可测差异（3.4 vs 3.2 s；cmd 峰值 1.45 vs 1.42 m/s，都在噪声内） |
| CPU/RTF | 代价很小：RTF **-2.7%**（A 组）~ **±0**（B 组），`missed rate` 恒 0；RSS 无异常增长 |
| 看得更多？ | ❌ **没有**：A 组两边的 0~0.5 m 环带都是空；B 组贴墙到 0.35~0.40 m 时**两边都看得见**（p2l `range_min=0.05`，45 cm 盲区早已不存在）；点云里"落在 p2l 高度带外"的点 = 0 |
| 看得更少？ | ⚠️ **是**：局部图的真障碍格数少 ~7 倍（135 vs 20），墙面 lethal 从"连续线"变成"周期 0.05 m 的点状线"（占位率 71% → 40%）；膨胀带几乎不变，所以对规划/控制**暂时无害** |
| 贴墙行为 | 贴墙终点姿态：`scan` 停在离墙 **0.349 m**、`stvl` 停在 **0.396 m**（后者略保守 4 cm）；A 组的最近距离反而 `stvl` 略小（p10 0.635 vs 0.717 m）⇒ **方向不一致，视为噪声**，但"STVL 让车更贴墙"没有证据 |
| 时间维（本次唯一没测到的收益） | ❓ sim 里**没有任何动态障碍**，`voxel_decay 0.5 s` 的收益（人走过/残影）在本实验里**根本无从体现** |

**建议**：

1. **默认值保持 `local_obstacle:=scan` 不变**（本次没有任何证据支持把它换掉；换掉还会把
   局部图的 lethal 核心变成点状线，属于"用一个已证实的隐患换一个未证实的收益"）。
2. **`local_obstacle:=stvl` 保留为可选槽位**：它已经能跑通且代价很小，是**将来做动态障碍/
   残影实验的载体**（那才是 STVL 的真正收益来源）；等实验台上出现动态障碍后再复测。
3. **若要用 `stvl`**：建议先把 `stvl_layer.voxel_size` 从 0.05 降到 **0.02**（= 局部栅格分辨率，
   消掉点状线），或直接用 `stvl_both`（STVL 提供时间维、`/scan` 提供连续线）。
   ⚠️ 这两条都是**本次未实测**的建议，见 §6。
4. **视觉/调试口径**：局部图变得"稀疏"是**正常现象**（体素量化），不要当成丢数据；
   真要看 STVL 的完整语义，看全局图（`publish_voxel_map: true`，`/global_costmap/voxel_grid`）。


---

## 5. 怎么跑 / 怎么回退

### 5.1 跑（原生 `ros2 launch`，无头）

```bash
cd ~/HzMi_rmsimulation
source /opt/ros/humble/setup.bash && source install/setup.bash

# 对照（原行为）：局部只吃 /scan
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
  nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=False local_obstacle:=scan

# 实验：局部也上 STVL（另一个终端发同一个目标点）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
  nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=False local_obstacle:=stvl

# 目标点 + 回归（栈已起来后）
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 --localization gicp+local_stvl
```

一键 A/B（起栈 → 探针 → 回归 → 收尾 全在**同一个 bash 调用**里；本环境每条 bash 调用是独立
PID namespace，跨调用看不见彼此进程）：

```bash
bash .tmp_cache/stvl_local/run_ab.sh scan scan    # 控制组
bash .tmp_cache/stvl_local/run_ab.sh stvl stvl    # 实验组
```

§3.2/§3.3 那些数字是这么来的（**只订阅，不改栈的行为**；两跑都要挂同一个探针才可比）：

```bash
# 栈起来之后、发目标点之前挂上探针（--dump-grid 会存下"离障碍最近那一刻"的原始栅格）
python3 tools/scripts/regress/local_obstacle_probe.py \
    --out /tmp/scan.probe.jsonl --period 0.5 --dump-grid /tmp/scan.grid.npz &
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 --localization gicp+local_scan
kill -INT %1        # 探针收到 SIGINT 会把 summary 写进 jsonl

# 出表（把四次跑的 jsonl 并排；--grid 可再画墙面 ASCII）
python3 tools/scripts/regress/compare_local_obstacle_probe.py \
    scan=/tmp/scan.probe.jsonl stvl=/tmp/stvl.probe.jsonl
```

> 本次用的"一次调用跑完 起栈→探针→回归→收尾"驱动脚本是 `.tmp_cache/stvl_local/run_ab.sh`
> （与既有 `.tmp_cache/five_way/run_one.sh` 同一套纪律：**同一 bash 调用内**完成，本环境每条
> bash 调用是独立 PID namespace，跨调用看不见彼此进程）。`.tmp_cache/` 不入库，所以上面把
> 关键命令都写全了；驱动脚本本身按需重写即可（隔离三件事：`HOME=/tmp/gzhome-<tag>`、
> 非默认 `ROS_DOMAIN_ID`、`unset DISPLAY` + 空闲的 `GAZEBO_MASTER_URI`）。

只看槽位到底开了哪些图层（**不用起栈**）：

```bash
python3 tools/scripts/regress/local_obstacle_truth_table.py --markdown
```

### 5.2 回退

- **最省事**：`local_obstacle:=scan`（默认值）—— `stvl_layer.enabled=false`，等价于今天。
- **彻底移除**：删掉 `nav2_params_sim_base.yaml`…`local_costmap` 段 `plugins` 里的
  `"stvl_layer"` 一项 + 删掉整段 `stvl_layer:`，再删 `navigation_launch.py` 两张表里
  `'stvl'/'stvl_both'` 两行与 arg 的 `choices` 两项。**改回后务必重跑**
  `local_obstacle_truth_table.py`。

---

## 6. 未验证 / 未能测量的清单

1. **`stvl_both` 没有跑过 bench**：只做了静态真值表校验（`local_obstacle_truth_table.py`
   断言它 = stvl+scan+cloud 三层全开），**没有实测**它与 `scan`/`stvl` 的差异、也没量它的 CPU。
2. **动态障碍/衰减完全没测**：sim 里没有会动的东西，`voxel_decay 0.5 s`、`decay_model 0` 的行为
   （残影多久消失、会不会把 0.4 m 矮墙衰减掉）**一次都没有被观测到**。
   `docs/cod_nav_2026_deep_dive.md` §9 第③条早就把"`voxel_decay 0.5` linear 会擦掉 0.40 m 矮墙"
   列为**未实测的推断**，本次也没能证伪或证实。
3. **点状线只在一个姿态上量过**：§3.3 的数字来自 B 组各自"离墙最近那一刻"的单帧
   （`best_grid`）。不同距离/角度的墙面占位率是否一样、以及"0.05 m 缝会不会真的让某个下游漏检"
   **没有测**（没有做碰撞/漏检实验，也没有看 MPPI 的 critic 内部）。
4. **`min_obstacle_height 0.0`(我们) vs `0.15`(COD) 的差别没测**：本场景地面已被 linefit 去掉，
   取 0.0 只是为了与 global 一致；会不会引入贴地幽灵障碍**在 sim 里没复现**（实车才可能暴露）。
5. **`publish_voxel_map: false` 是我们的选择**（COD 两图都 false，我们 global 是 true）：
   局部发体素图的开销**没有单独量化**。
6. **CPU 只有 RTF/频率代理，没有真正的 CPU% 采样**：`missed rate` 与 RTF 都受仿真 RTF 上限
   （0.73~0.75）压制 ⇒ 若把仿真跑得更快（更简单的 world、更少传感器），STVL 的相对代价会更大。
7. **只测了一个 world（RMUL2026）+ 一个控制器（mppi）**：`local_obstacle:=stvl` 与 RPP/DWB/TEB
   的组合、以及 RMUC/RMUL 场地**没跑**。
8. **数字口径**：探针订阅 `costmap_raw` 本身会让发布端多做一次序列化（两个跑都有）⇒ 与历史
   基线（不订阅探针的 0.76）不是严格同口径；`lc_min_r` 精度受"机器人是否恰在滚动窗中心"
   （nav2 取整到栅格，±1 cm）影响。



---


## 来源 7/19：`nav/traversability_plan.md`


> **本文是什么**：一份**设计文档**，回答"2D 占据栅格表示不了的三件事（斜坡、落空/坑、限高），
> 要用什么 3D 表示、怎么变成 nav2 能吃的层、怎么用我们的 STL 真值验证、怎么在 RViz 里看、
> 分几步做、先不做什么"。**本文不含任何代码/配置改动**，也没有改任何跑起来的文件。
>
> **本文不是什么**：不是实现记录，也不是实测报告。文中每个"数字"都标了出处：
> **[实测·已有]** = 本仓文档里已经量过的值（给文件 + 节号）；**[本次核对]** = 写本文时为对齐口径
> 现算的 STL 几何量（口径与命令都写出来，可复跑）；**[待测]** = 还没有这个数，写清"要量什么、怎么量"。
>
> **状态标记**：**【事实】** = 有出处可核对；**【推断】** = 我的判断（给出理由，但没有实测支撑）。
> 上下游背景见 `docs/mapping_2d_from_cloud.md`（3D→2D 判据与坡道段）、`docs/ground_segmentation_slots.md`
> （地面分割与 p2l 高度带）、`docs/worlds.md`（RMUC2026 场地资产）、`docs/algorithm_matrix.md`（维度决策）、
> `docs/architecture.md` §3.2.7（2.5D 落位）。

---

## 0. 一句话结论

**【推断】** 我们**不需要**先上"3D 地图"这个东西。需要的顺序是：

1. **2.5D 高程图（elevation map）先做** —— 因为我们要的三个量（坡面法向、相对局部地面的高差、每格净空）
   全都是"每个 (x,y) 一个或几个标量"的量，**本来就是 2D 网格**；nav2 的接口也是 2D 网格 ⇒ 转换成本最低。
2. **一个轻量"落空/负障碍"判据** 挂在同一张高程图上（不需要新的数据结构）。
3. **OctoMap 只在需要"真·三维体积判断"时才上**（多层/悬挑/隧道里的通过性检查）；
   ESDF 体素（voxblox / nvblox / FIESTA）**现在不做**（那是给 3D 规划器/MPC 用的，我们不上 3D 规划器）。
4. **动作** 分两档：**写 cost（可通行性）** 用 nav2 costmap 图层（现成机制）；
   **触发行为**（"看见了但我不走 / 我慢速试探"）用 BT 条件节点 + 自带行为树 XML（`bt_navigator` 现成机制）。
   两档**不要混在一个东西里**。
5. **我们台架真正的独有资产不是 3D 传感器，是 STL 真值**：`RMUC2026.stl` 可以把
   **坡度/净空/落差的真值图离线算出来**，与传感器派生层做**逐格比对**（TP/FP/FN 计数）。
   这在实车上**做不到**。

---

## 1. 问题定义：三件事、一张表、一条"二义性"论证

### 1.1 三件要"认出来并作出反应"的事

| # | 能力 | 要回答的问题 | 现在谁在回答（如果有） | 现状 |
|---|---|---|---|---|
| A | **坡度（可行驶斜面）** | 这一格是"能开上去的斜面"还是"墙 / 台阶沿"？坡度多少、该不该限速？ | `tools/scripts/mapping/pcd_to_nav2_map.py` 的 `--slope-limit 25°`（**离线、只用于出一张 pgm**） | 只有"free/occupied"的二值结果，**坡度值本身没有被发布出去** |
| B | **落空 / 负障碍（坑、边沿）** | 这一格脚下的地面是不是**没了 / 掉了**？ | 没有 | **空缺** |
| C | **净空高度（限高）** | 这一格**头顶**还有多少空间？够不够本车过？ | 没有 | **空缺** |

图例：A 现有的是"离线一次性判据"，B/C **完全空缺**（`docs/algorithm_matrix.md` §一.1 的 2.5D 空缺行：
"待建 `src/perception/rm_perception/rm_elevation_map/` + `src/nav/rm_navigation/rm_costmap_layers/`"）。

### 1.2 一张 2D OccupancyGrid **能**与**不能**表示什么

2D 占据栅格每格只有三种值：**占用 / 空闲 / 未知**（本仓口径 `0=占用 / 205=未知 / 254=空闲`，
`mode: trinary`、`free_thresh 0.25`、`occupied_thresh 0.65`，见 `docs/mapping_2d_from_cloud.md` §5.2）。**【事实】**

| 要表达的东西 | 2D 栅格能表示吗 | 为什么 / 证据 |
|---|---|---|
| 竖直墙、柱子、台阶立面 | ✅ 能（占用） | 高差大、法向陡 ⇒ 任何高度阈值判据都能抓住 |
| 0.2 / 0.3 m 台阶（"上不去"） | ✅ 能（占用） | 相对局部地面高差 0.2/0.3 m > 阈值 0.15 m（`docs/ground_segmentation_slots.md` §5.2：0.2/0.3 m 平台顶面 100% 判障碍）|
| **可行驶的斜坡（23°）** | ⚠️ **能表示，但一旦表示错就致命** | 坡度**没有**独立字段；只能被"降级"成二值。若判成**占用** ⇒ 规划器认为路被堵死；若判成**空闲** ⇒ 又丢掉了"这格有多陡、要不要限速" |
| **落空 / 负障碍（地面塌下去）** | ❌ **原理上不能** | 栅格记录的是"这一格 (x,y) 有没有障碍"，**不记录"脚下有没有地"**。一个 −0.2 m 的坑在俯视图上就是一块"没有回波的地" —— 与"墙后面看不见"、"超出量程"、"传感器盲区"**完全不可区分** |
| **净空高度（头顶空间）** | ❌ **原理上不能** | z 被压掉了。同一个 (x,y) 的"地面 + 头顶 1.2 m 处的横梁"与"地面 + 自由天空"在栅格里**是同一格** |
| 坡度值的连续量（10°/23°/35°） | ❌ **不能** | 只有三值，没有连续量；连"代价"都只能靠 inflation 间接表达 |

### 1.3 两条必须写清楚的"事实 + 后果"

**① 把可行驶的斜坡标成"占用"是错的（不是保守，是错）。**
**【事实】** `docs/mapping_2d_from_cloud.md` §4.1/§6.1：坡面区域在旧图（`/scan` 累积图 A）里
**1680/1680 格 = 100% unknown**，不是"判成占用"，是**根本没看到**；规划器对坡道/走廊的 5 个目标
**2/5 直接拒绝**。**【事实】** 同一份文档 §5.1/§10.4：新工具（点云投影 + 当地面法向闸）把坡面判成
**98.8% free**（1660/1680，其余 20 格是台阶沿），规划器 **5/5** 给出合法路径。
**【推断】** 所以"标成占用"与"标成未知"在这里是**同一类错误**——都让规划器**拒绝**这块地。
"保守"这个词在可通行性上是**误导**：把可走的地方标成不可走，等于**凭空删掉一条通路**。

**② "unknown 就是规划器拒绝的原因"——已经量化并已修。**
**【事实】** 坡面 100% unknown（1680/1680）→ 改成点云投影后 98.8% free → 规划器 2/5 → 5/5
（`docs/mapping_2d_from_cloud.md` §6.1 / §10.4 的三行表）。这就是"为什么必须有 3D 派生的信息"的
**已经付过账的证据**，不是设想。

**③ 为什么"认出 → 决定 → 行动"比一张 2D 图要多。**
**【推断】** 一张 2D 栅格能承载的只有"这一格能不能被压过去"。而这三件事需要的是**三段时间上不同的信息**：

```
感知（3D 点云） → 表示（2.5D 层：坡度/落差/净空） → 判据（阈值 + 上下文） → 决策（代价 or 触发） → 动作（规划/限速/BT）
```

2D 图只接在"决策 → 动作"这一段上。把判据塞进"离线出一张 pgm"（现在 `pcd_to_nav2_map.py` 的做法）
**能用，但只能选一次、且不能反映实时变化**（斜坡上有动态障碍、限高杆下面突然站了个人……）。
要"认出并作出反应"，缺的是**中间那段**（表示 + 判据 + 一条把判据送进 nav2 的路）。

---

## 2. 表示选型（带取舍表）+ 本机可用性核对 + 推荐

### 2.1 四个候选（以及"点云直投"这个非候选基线）

| 表示 | 存什么 | 能回答什么 | 成本 | 局限（**关键**） |
|---|---|---|---|---|
| **点云**（现在就有：`/livox/lidar/pointcloud`、`/segmentation/ground`、`PCD/<world>_spl.pcd`） | 一帧/一张点集，无拓扑、无免费空间 | "哪里有回波"；**法向、相对高度都可以现算** | **零**（已经在跑） | ① 无体素/无插值 ⇒ 稀疏处与盲区只能"未知"；② 每格统计要自己写；③ **无法回答"这一格是不是没有地"**（缺"反证"结构）；④ 密度随距离骤降 |
| **2.5D 高程图**（elevation map / grid_map） | 每个 (x,y)：`elevation`（地面高）、`elevation_variance`、可扩展层 `slope`/`roughness`/`free_height`/`traversability` | **坡度、粗糙度、台阶高差、净空**——我们三个需求里的 A 和 C，以及 B 的一半 | 中：一帧点云按 (x,y) 分桶 + 形态学/插值；`grid_map` 库现成 | ① **一个 (x,y) 只有一个地面高度 ⇒ 多层结构（桥 / 隧道 / 悬挑 / 头顶横梁）表达不了**（必须靠额外的"净空层"或第二张图打补丁）；② 需要**重力对齐的姿态**（否则"地面"跟着车滚）；③ 稀疏区必须外推 ⇒ 外推策略本身就是误报来源 |
| **OctoMap**（3D 概率八叉树） | 体素的**占据概率** + **射线清除**的自由空间 | **真·三维通过性**：任意 (x,y,z) 是否被占；**头顶有没有横梁**；多层结构；`projected_map` 直接给 2D | 高：8 Hz 点云做 ray casting + 树更新；内存随体积 | ① 分辨率越细代价越高；② **它回答的是"占据"，不是"可行驶"**——坡道在上面仍是一堆占据体素，**坡度/可走性还得另算**；③ 概率参数（hit/miss、clamping）是新的调参面 |
| **ESDF / TSDF 体素**（voxblox、nvblox、FIESTA） | 体素到最近表面的**距离场**（+ 梯度） | 连续距离、梯度 ⇒ 直接给 3D 规划器 / MPC / 主动探索 | 高（nvblox 要 GPU） | ① **它的消费者是 3D 规划器**，而我们的决策层明确不上 3D（`docs/algorithm_matrix.md` §8.2）；② 距离场同样不直接回答"坡度能不能上"；③ 又是一整套建图/融合/位姿依赖 |

**非候选基线（现在的实际做法）**：`tools/scripts/mapping/pcd_to_nav2_map.py` —— 离线把 PCD 投影成
一张 nav2 `pgm`。**【事实】** 判据是 `--normal-k 8` PCA 法向 ⇒ `--slope-limit 25°` 闸；
局部地面 = `--ground-cell 0.20 m` 粗格内可行驶面点的 `--ground-percentile 5`；
占用 ⟺ `z_max − 地面 > --height-threshold 0.15 m` **或**（坡度 > 25° 且 `z_max − 地面 > --slope-min-height 0.05 m`）；
`--ground-min-points 2`、`--ground-borrow-cells 1`、`--occ-min-neighbors 1`、`--occ-dilate-cells 1`、
`--fill-empty ground`、`--border unknown`（`--help` 与 §5.1 两处口径一致）。**【事实】**
它**只输出一张图**（没有把坡度/高度差留下来），而且**跑的时候才判**。

### 2.2 【事实】本机可用性核对（写本文时现场跑的）

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | wc -l                                  # 433 个包
ros2 pkg list | grep -iE 'octomap|grid|elevation|voxblox|nvblox|esdf|fiesta|voxel'
#  → 只有两行：nav2_voxel_grid、spatio_temporal_voxel_layer
apt-cache policy ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-server \
                 ros-humble-octomap-rviz-plugins ros-humble-grid-map-ros \
                 ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin ros-humble-grid-map-costmap-2d
apt-cache search ros-humble-elevation          # → 无输出（Humble 没有 elevation_mapping 的 apt 包）
```

| 候选包 | `ros2 pkg list`（已装？） | apt 候选版本 | 结论 |
|---|---|---|---|
| `spatio_temporal_voxel_layer`（STVL，2.5D 体素） | ✅ **已装**（唯一在岗的 2.5D 环节） | — | **现成可用**，已做成槽位：`global_obstacle:=stvl`（默认）/`scan`/`none`、`local_obstacle:=stvl`/`stvl_both`/… |
| `ros-humble-grid-map-ros` + `-msgs` + `-costmap-2d` + `-rviz-plugin` | ❌ **未装** | **2.0.1-1jammy**（`ros-humble-grid-map` 系列一整排在源里，`apt-cache search ros-humble-grid-map` 命中） | **apt 一条命令可装**；Humble 有官方发布 |
| `ros-humble-octomap` / `-msgs` | ❌ **未装** | 1.9.8 / 2.0.1 | apt 可装 |
| `ros-humble-octomap-server` | ❌ **未装** | 2.3.1 | apt 可装（**注意**：包名叫 `octomap_server` 的那个 2.x 才是 ROS 2 的 `octomap_server` 节点包） |
| `ros-humble-octomap-rviz-plugins` | ❌ **未装** | 2.1.1 | apt 可装 |
| `elevation_mapping`（ANYbotics）/ `elevation_mapping_cupy`（leggedrobotics） | ❌ **未装、apt 也没有** | **无 apt 候选**（`apt-cache search ros-humble-elevation` 空） | **只能源码编译**；cupy 版还要 CUDA/GPU。本机是仿真工作站、无 GPU 相关依赖证据 ⇒ **不建议现在引** |
| voxblox / nvblox / FIESTA | ❌ 全无 | 无 apt 候选 | 只作**将来**候选登记 |

上游出处（写本文时逐个 `curl -o /dev/null -w %{http_code}` 复核过，均 **200**）：
[ANYbotics/grid_map](https://github.com/ANYbotics/grid_map)、
[ANYbotics/elevation_mapping](https://github.com/ANYbotics/elevation_mapping)、
[leggedrobotics/elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)、
[OctoMap/octomap_mapping](https://github.com/OctoMap/octomap_mapping)、
[OctoMap/octomap](https://github.com/OctoMap/octomap)、
[ethz-asl/voxblox](https://github.com/ethz-asl/voxblox)、
[nvidia-isaac/nvblox](https://github.com/nvidia-isaac/nvblox)、
[HKUST-Aerial-Robotics/FIESTA](https://github.com/HKUST-Aerial-Robotics/FIESTA)。
**这些上游描述本身是外部资料**（未经我们实测），只作"要装什么、有没有 ROS 2 分支"的依据。

### 2.3 引用到的外部来源（URL）

| 项 | URL | 我们关心的点 |
|---|---|---|
| grid_map 库本体 | <https://github.com/ANYbotics/grid_map> | 多层 2D 网格（`elevation`/`slope`/`roughness`…），Humble 有 apt 发布 |
| grid_map RViz 插件 | <https://index.rosdabbler.com/p/grid_map_rviz_plugin/> | `rviz_plugin` 单独一个包名 ⇒ 才知道要装哪个 |
| elevation_mapping_cupy（ROS 2 分支 README） | <https://raw.githubusercontent.com/leggedrobotics/elevation_mapping_cupy/ros2/README.md> | 有 `ros2` 分支、release v2.1.0（见 <https://github.com/leggedrobotics/elevation_mapping_cupy/releases/tag/v2.1.0>） |
| octomap_mapping（= `octomap_server` 节点所在） | <https://index.ros.org/r/octomap_mapping/#humble> | Humble 有条目 ⇒ 有 apt 发布 |
| octomap_rviz_plugins | <http://mirror-ap.wiki.ros.org/octomap_rviz_plugins.html> · <https://index.rosdabbler.com/r/octomap_rviz_plugins/> | 看八叉树要另外装这个插件包 |
| nvblox（GPU ESDF/TSDF） | <https://github.com/nvidia-isaac/nvblox> | 论文出处：nvblox, ICRA 2024 |

### 2.4 **推荐（带理由）**

**【推断】按这个顺序落地，且第一步不引入任何新依赖：**

1. **先做"2.5D 高程图 + 三个派生层"，但先只做离线**（Python + `numpy`/`scipy`，
   复用 `pcd_to_nav2_map.py` 已经调好的口径：0.05 m 格、0.20 m 粗格、p05 分位、`k=8` 法向）。
   理由：① 三个需求全都能在这套结构上表达；② 口径与**已经验收过的**工具一致 ⇒ 结果可对账；
   ③ 不装包、不动栈 ⇒ 风险≈0。
2. **数据结构用 `grid_map`（apt 2.0.1）当"格式"，不当"运行时"**：
   离线产出 `.npz`（自用）**并且**能导出 `grid_map_msgs/GridMap`（互操作用）。
   理由：将来别人（或 RViz）要看得懂，`grid_map` 是事实标准；但现在没有理由为了它引入一个运行时节点。
3. **实时只在"确认离线口径站得住之后"接**：优先走两条**已有**的路子，
   而不是新写 costmap 插件 ——（a）把离线层烘进一张 `<world>_slope.pgm` 当**第二个 `static_layer`**；
   （b）`local_obstacle:=cloud`（现成的 `obstacle_cloud_layer` 直投点云）+ 高度带参数。
   真正的新插件（`nav2_costmap_2d::Layer`）**等到离线层被 STL 真值验证过再写**。
4. **OctoMap 只在"真·三维体积判断"成为刚需时才装**：判据是"出现一个场景，2.5D 层无论怎么调都把
   可走的路判成不可走（多层/悬挑/隧道）"。**【事实】** 就 RMUC2026 这份网格而言，这个场景**不存在**
   （见 §5.3：整个场地**没有任何一个列有"头顶表面"**）。
5. **ESDF/MPC 不做**（理由见 §7.3）。

---

## 3. 三个派生层的设计（数据源 → 判据 → 输出 → 怎么进 nav2）

### 3.0 先说三个层共同的"现成积木"与"真正缺的东西"

**【事实】已有的积木**（都不用重写，用之前先核对一遍）：

| 积木 | 位置 | 对三层的作用 |
|---|---|---|
| 地面/障碍二分的**槽位** `<ground>`（`linefit` 默认 \| `patchwork`） | `bringup_sim.launch.py` 的 `ground` 参数；契约见 `docs/ground_segmentation_slots.md` §2 | 给"哪些点是地面"一个**官方来源**；`/segmentation/ground` 就是我们要的"地面点" |
| **代价图层槽位** `global_obstacle` / `local_obstacle` + `LOCAL_OBSTACLE_LAYER_TABLE` / `GLOBAL_OBSTACLE_LAYER_TABLE` | `src/nav/rm_navigation/rm_navigation/launch/navigation_launch.py:50-62`（真值表）、`:86-106`（`build_param_substitutions` → `RewrittenYaml` 的 `enabled` 开关） | **加"一层数据源"的现成机制**：新层只要有自己的 `enabled` 键与参数段，就能挂进这张表；静态校验脚本 `tools/scripts/regress/local_obstacle_truth_table.py` 会复用同一份逻辑 |
| **STVL**（3D 体素 → 2D 代价） | apt 第三方；参数 `nav2_params_sim_*.yaml` 的 `stvl_layer`；`min_obstacle_height: 0.0`、`max_obstacle_height: 2.0` | 已经是"2.5D 环节"；**高度带就是它的语义边界** |
| `obstacle_layer`（吃 `/scan`） | 同上，`max_obstacle_height: 2.0`、`min_obstacle_height: -0.02` | 2D 主障碍源 |
| `obstacle_cloud_layer`（点云直投，不经 p2l） | 同上，槽位 `local_obstacle:=cloud\|both` | 已经能"绕过 p2l 高度带" |
| `pcd_to_nav2_map.py`（离线 3D→2D） | `tools/scripts/mapping/` | **三层判据的第一版全都可以先在这里试**（离线、零风险） |
| 膨胀层 | `inflation_radius` 局部 0.5 / 全局 0.55、`robot_radius 0.22` | 任何"致命格"最终都靠它变成可规划的代价梯度 |

**【事实】真正缺的只有一样：判据本身（把高程/法向/净空变成"代价或触发"的那段代码）。**
现在仓库里**没有**任何节点/插件在算坡度、落差、净空。`docs/algorithm_matrix.md` 把这两块登记为
"**（空缺）**…待建 `src/perception/rm_perception/rm_elevation_map/` + `src/nav/rm_navigation/rm_costmap_layers/`"，
`docs/architecture.md` §3.2.7 也给了同样的落位建议（生产端 `rm_elevation_map`、消费端 `rm_costmap_layers` 的
`nav2_costmap_2d::Layer` 插件）。**【推断】** 这两个包名是**既有约定**，应当沿用。

---

### 3.1 坡度层（slope / traversability）

| 项 | 设计 |
|---|---|
| **数据源** | **首选** `/segmentation/ground`（已是"地面点"，`linefit`/`patchwork` 同契约；`docs/ground_segmentation_slots.md` §2 的逐字段契约表）——但它**每帧只有约 700~900 点**（**【事实】** `docs/issues_and_findings.md` #25 的更正："`/segmentation/ground` … 实测 ground 只有 ~700 点（≈11 KB）"）⇒ 单帧太稀，必须**多帧累积**或**先验打底**。<br>**次选** `/livox/lidar/pointcloud`（原始点云，密度足够，但含墙/台阶，要靠法向闸分）<br>**离线首选** `PCD/<world>_spl.pcd` / `PCD/<world>.pcd`（已有，且 `pcd_to_nav2_map.py` 口径已调好）<br>**⚠ 共同前提**：必须在**重力对齐**的系里算（`livox_frame` 的 `rpy = 0 0 0`，`docs/ground_segmentation_slots.md` §3）或 map 系；**不能**用挂在倾斜车体上的本体 z |
| **判据** | 两个都要，因为它们回答不同的问题：<br>① **点级法向**（`k=8` PCA）与重力夹角 θ；<br>② **格级梯度** `‖∇z‖` 的 `atan`（对 0.05 m 格要做 3×3 平滑，否则量化噪声把坡度抬到 45°+）。<br>写进层里的应是**连续坡度值（deg）**，不是二值 |
| **阈值（**这里有个必须先拍板的问题**）** | **【事实】** 场地上真实的斜面：[实测·已有] 23°（`docs/mapping_2d_from_cloud.md` §4.1 的 `23–24°`）、12.8°（同表"浅坡 12.8°"）；`pcd_to_nav2_map.py --slope-limit` 默认 **25°**。<br>**【推断】** ⇒ **25° 这个闸只给 23° 留了 2° 余量**，"可行驶"与"不可行驶"的分界就压在真实特征上。**这不是一个能拍脑袋定的阈值**：它应该由"本车实测能上多少度"决定，而**这个数我们没有**（见 §8 第 4 条 **[待测]**）。<br>**【推断】建议的初始形态**：**三段式**而不是一刀切 —— `θ ≤ 15°` 正常通行；`15° < θ ≤ 25°` **高代价 / 限速**（保留可通行性，让代价梯度自己选平路）；`θ > 25°` 才致命。这样 23° 坡变成"能走但优先绕"而**不是"禁止"** |
| **输出** | （a）`slope` 层（float，deg）；（b）由它派生的 `cost` 层（0~252 的连续代价）或 `speed_limit` 层；（c）**决策点清单**（坡度 > 阈值的格，给 BT / 日志用） |
| **怎么进 nav2** | **路线 1（先做，零代码）**：离线把 `slope` 按阈值分档烘进一张 `<world>_slope.pgm` ⇒ 作为**第二个 `static_layer`**（nav2 的 `static_layer` 可以挂多个实例，各给一个 `map_subscribe_transient_local` 话题）；或用 `grid_map_costmap_2d`（apt 包，**现成**的 "GridMap → costmap_2d 格式"接口）<br>**路线 2（后做）**：写 `nav2_costmap_2d::Layer` 插件（落位 `src/nav/rm_navigation/rm_costmap_layers/`），注册进 `local_costmap`/`global_costmap` 的 `plugins` 列表 + `plugin: "rm_costmap_layers::SlopeLayer"`；`enabled` 键按 `navigation_launch.py` 的 `_enabled_expr` 机制挂到 `local_obstacle`/`global_obstacle` 的真值表上（**不改契约**） |
| **已知的坑** | **【事实】** `pcd_to_nav2_map.py` §5.4 第 1 条：**不加法向闸、直接用"粗格内 z 的 p05"当局部地面 ⇒ 墙脚那些点的 p05 就是地面 ⇒ `dz≈0` ⇒ 整面墙被判 FREE**（实测 `/scan` 图里 1493 个占用格有 **1418 个**被这样吃掉）。**【推断】** 同一个坑在"坡度层"上会以另一种形式复现：**如果先用墙点估地面、再算坡度，得到的"地面"本身是墙**。⇒ 坡度层的**点级法向闸是必需项**，不是可选优化 |

---

### 3.2 负障碍 / 落空层（negative obstacle）

| 项 | 设计 |
|---|---|
| **数据源** | 同一张高程图（原始点云 + 局部地面）。**额外**需要"**反证**"：某一格**没有**地面回波，而**邻格有** |
| **判据（三选一或组合）** | ① **格内高差**：`z_max(格) − z_min(格) > Δ_drop` 且 **`z_min` 明显低于邻域地面** ⇒ 该格是"边沿/掉落"；<br>② **"无地面回波 + 邻域有地面"**：某格的局部地面估不出来（点数 < `--ground-min-points`），但半径 `r` 内邻格能估出地面，**且该格在激光视线内**（不是被遮挡）⇒ 判"地面缺失"；<br>③ **range image 上的遮挡/断裂**：把点云按激光的 (az, el) 展开成 range image，找"深度跳变 + 跳变后面没有回波"的模式（这是激光原理上最强的落空信号）。<br>**【推断】** 单一判据都会炸；建议**①做粗筛、②做确认、线上只用"①∧②"** |
| **为什么 2D 不能做** | 2D 栅格只知道"这一格有没有障碍"，**不知道"这一格有没有地"**。−0.2 m 的坑在俯视图上就是"没有回波" ⇒ 与"被墙挡住"、"超出量程"、"传感器盲区"**同形**。**【事实】** 佐证：`docs/mapping_2d_from_cloud.md` §6.3 —— 走廊条带 2688 格**整块落在 A 图窗口之外**（"不是判成占用，是图里根本没有这块地"）；A 图在它自己覆盖到的区域里也有 4587/4644 格是 unknown。**"没数据"与"没地"在 2D 里无法区分**，这正是负障碍层存在的理由 |
| **误报风险（我们量过的）** | ① **地图边界**：把窗口外沿当成"落空" ⇒ 车永远出不了图。**【事实】** `pcd_to_nav2_map.py` 用 `--border unknown|occupied` 区分这两种口径（默认 `unknown`）。负障碍层必须显式处理边界：**边界格按"未知"而不是"落空"**；<br>② **传感器噪声/稀疏**：0.10 m 体素下采样后的远场只有零星点（`docs/mapping_2d_from_cloud.md` §5.4 第 3 条）⇒ 空旷处每格都可能"没有地面回波"；<br>③ **45 cm 近场历史**：**【事实】** `p2l` 的 `range_min` 曾是 **0.45**（`docs/issues_and_findings.md` #24 / `docs/algorithm_matrix.md` §七 2026-09-21 行），会造成"贴身 45 cm 内既看不见又被反向清成 free"；**现在已是 0.05**（`config/laserscan_params.yaml` 里两次下调：`0.2 → 0.12 → 0.05`），且 `docs/stvl_local_costmap.md` §168/§232 实测"45 cm 盲区在当前配置下不存在（贴墙 0.35~0.40 m 仍标到墙）"。**【推断】** ⇒ **今天**不该再把 45 cm 当参数，但如果将来误报率压不住而把 `range_min` 退回 0.15/0.2，**近场就会重新变成"落空误报源"**，这条回退档位必须与负障碍层一起登记；<br>④ **有坡的地方天然有高差** ⇒ 判据 ① 会在 23° 坡的坡脚/坡顶密集触发。**【推断】** 必须先扣掉坡度分量（在"可行驶面"上做判据，而不是在全点上做） |
| **输出** | **【推断】建议 P0 只输出"独立的 no-go 掩膜 + 可视化"，不要直接写 lethal**：<br>（a）**保守**：误判成 lethal 的后果是"路被凭空切断"（§1.3 ①）；<br>（b）漏判的后果在这个场地上**很小**：0.2/0.3 m 台阶已经被 2D 静态层的**占用**格挡住了（`docs/worlds.md` §6：台阶高 0.20/0.30 m ⇒ 物理上上不去 ⇒ 2D 图把它们画成障碍）← **这层是"补第二道保险"，不是唯一防线**；<br>（c）真要用致命度：用 `nav2_costmap_2d::ObstacleLayer` 的 `marking`/`clearing` 语义最省事（沿用现有机制），**不要**新造一个"lethal"概念 |
| **怎么进 nav2** | 同 §3.1：先离线成层 ⇒ `static_layer` / `grid_map_costmap_2d`；后写 `Layer` 插件（`rm_costmap_layers::NegativeObstacleLayer`），挂进同一张 `enabled` 真值表 |

---

### 3.3 净空高度层（clearance）

| 项 | 设计 |
|---|---|
| **数据源** | 同一张高程图，但**必须保留"每格的上表面与下表面"两个高度**（或至少"头顶最近表面的高度"）。**【推断】** 这是 2.5D 表示**唯一必须打补丁**的地方：标准 elevation map 一个 (x,y) 只有一个 `elevation` ⇒ 要加一个 `ceiling_height` / `free_height` 层。这正是"2.5D 表达不了多层结构"那句限制的具体形态 |
| **本车真实尺寸（**【事实】** 从 URDF/xacro 读出来的，不是估的）** | `src/bringup/rm_nav_bringup/urdf/sentry_robot_sim.xacro`：<br>· `base_link` 碰撞盒 `box size="0.2 .3 .1"`，`origin xyz="0 0 0.05"` ⇒ z ∈ **[0, 0.10]**（相对 base_link）<br>· 4 个轮子 `cylinder length=0.05 radius=0.06`（`:5-6` 的 `xacro:property`），轮心在 z=0 ⇒ **轮半径 0.06 m**（= base_link 离地高度，`docs/worlds.md` §2.4 实测 `/odom_ground_truth` z = 0.0600）<br>· `imu_link` 碰撞盒 `box size="0.05 0.05 0.05"`，`origin (0.12, 0, 0.125)` ⇒ 顶到 **z = 0.15**<br>· Livox 装在 `base_link + (0.12, 0, 0.175)`（`xacro:arg xyz` 默认 = 实测值）；雷达本体 `box 0.1×0.06×0.06`（`src/simulation/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro`）⇒ 顶部到 **z ≈ 0.175 + 0.049 = 0.224 m**<br>⇒ **碰撞包络高度 ≈ 0.224 m（四舍五入取 0.23 m 做安全余量）**；横向 **0.2 × 0.3 m**（nav2 用的 `robot_radius: 0.22` 取的是"≈真实车体半径 0.2"的圆，见 `nav2_params_sim_base.yaml` 注释）<br>⚠ **真车不一样**：`sentry_robot_real.xacro` 里 Livox 在 `base_link + (0.0, 0.045, 0.49)` ⇒ **实车雷达面高 0.49 m、整车更高**。["哪个高度该进判据" 见 §8 第 3 条] |
| **判据** | 每格：`free_height = ceiling_z − ground_z`；`可过 ⟺ free_height > robot_height + margin`。<br>**【推断】** 初始建议 `robot_height = 0.23 m`（仿真包络四舍五入）、`margin = 0.10 m` ⇒ **阈值 0.33 m**；**但这个值必须在实车/新场地重新定**（§8 第 3 条） |
| **为什么 nav2 的 2D 层没有这个概念** | nav2 costmap 是纯 2D 栅格，**z 被压掉了**。现有参数里的 `max_obstacle_height: 2.0` **不是**"限高"：它的语义是"把 2 m 以上的点当噪声丢掉"（`nav2_params_sim_base.yaml` 里 `obstacle_layer` 与 `stvl_layer` 都是 2.0），**不控制机器人能不能从下面钻过去**。<br>**【事实】** `min_obstacle_height: 0.0` 是 2026-09-24 为"保住 0.4 m 矮墙"专门从 0.2 调低的（同文件注释），说明这些高度带是在"**把什么当地面上的障碍**"这一维上做取舍，**与"头顶"无关** |
| **输出** | `free_height` 层（float）+ 三档：`可过` / `高风险（触发减速或行为）` / `不可过（致命）` |
| **怎么进 nav2** | 同 §3.1/§3.2；【推断】这一层最值得走"**触发**"而不是"写 cost"（见 §4）：因为限高的失败模式是"车已经进去了才发现"，惩罚应当发生在**进入之前** |

---

## 4. 动作怎么接：**"写 cost" 与 "触发行为" 的边界**

### 4.1 两档的判据（**【推断】，但理由是可核对的**）

| 档 | 什么时候用 | 机制 | 为什么不能用另一档 |
|---|---|---|---|
| **写 cost（连续/致命代价）** | 决策是"**这一格值多少**"，而且**不需要改变导航流程**：坡道限速、落空禁行、净空不可过 | nav2 costmap 图层（`nav2_costmap_2d::Layer`）：`updateCosts()` 往 master grid 写 0~252 或 `LETHAL_OBSTACLE`/`NO_INFORMATION` | cost 只能表达"贵/不可走"；它**不能表达"停下来问一句"**，也不能改变行为树的分支 |
| **触发行为（BT 条件/动作）** | 决策是"**流程要变**"：进坡前减速试探、净空不足就换目标、落空边沿前先原地确认 | `bt_navigator` + BT XML（`default_bt_xml_filename`）+ 自定义 BT 节点（`plugin_lib_names`）；`behavior_server` 的插件（`behavior_plugins`） | 这些是**流程/服务层**，写进 cost 里就丢失了"原因"，也没法恢复 |

**一句话**：**cost 决定"走不走、走多贵"，BT 决定"走的姿势和走不成之后干什么"。**

### 4.2 现成的钩子（**【事实】**，都是本机已经在跑的机制）

| 钩子 | 位置（本仓证据） | 拓展方式（**设计级，不写代码**） |
|---|---|---|
| `bt_navigator.plugin_lib_names` | `src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml:90-121`（31 项，全是 `nav2_*_bt_node`） | **加一行**自己的 `librm_bt_nodes.so`（如 `rm_bt_is_ramp_ahead_condition_bt_node`）；节点用 `BT::ConditionNode` 派生 + `BT_REGISTER_NODES` 注册，库由 `nav2_behavior_tree` 的 `BT::SharedLibrary` 在运行期加载 |
| BT XML | **本仓现在没有**：两个 params 都没有 `default_bt_xml_filename` ⇒ 用的是**包内默认**（Humble 的 `navigate_to_pose_w_replanning_and_recovery.xml`） | 新增 `src/nav/rm_navigation/rm_navigation/behavior_trees/*.xml`（把默认树复制过来，在 `RecoveryNode`/`Fallback` 里插入自定义条件），参数里给 `default_bt_xml_filename`。**【推断】这一步必须与"新增节点"一起做**——只加节点不改树，节点永不被调用，是典型的**静默失效** |
| `behavior_server` | 同文件 `:404-412`：`behavior_plugins: ["spin","backup","drive_on_heading","wait"]`；`costmap_topic` / `footprint_topic` / `max_rotational_vel 3.0` 等 | 加自定义插件（如"低速试探一段" `rm_behaviors/CreepForward`）⇒ 在 BT XML 里作为 recovery/主动动作调用。**【推断】** 这一档**现在不做**（见 §7.3），因为"怎么保证触发是安全的"还没有答案（§8 第 5 条） |
| 探测器 → BT 的**载体** | **没有**（仓库里没有任何自定义 BT 节点：`grep -rl "BT_REGISTER_NODES" src/` 为空 —— **【推断】** 这是从"params 里 plugin 列表全是 nav2 官方"反推的，写代码前应跑一遍 `grep -rn "BT_REGISTER_NODES\|bt_nodes" src/` 确认） | **【推断】推荐形态**：探测器发一个**状态话题**（如 `/traversability/status`，`std_msgs/String` 或自定义 msg：`{坡度档, 净空档, 落空档, 有效时间}`），BT 条件节点只做"读 latched 状态 + 与当前位姿/路径点比对"。**好处**：探测器与 BT 解耦，BT 里不跑重计算 |

### 4.3 **【推断】一个具体的、最小的触发闭环（设计，不含代码）**

```
rm_elevation_map（新，感知域）
    ├─ 发 /traversability/slope_grid      （2.5D 层，供 costmap 图层消费 = "写 cost" 那档）
    ├─ 发 /traversability/clearance_grid  （同上）
    └─ 发 /traversability/status          （latched，低维：{最近坡道方位+坡度, 本格净空, 落空标志}）
                 │
                 ├─→ rm_costmap_layers::SlopeLayer / ClearanceLayer   （"写 cost"）
                 └─→ rm_bt_nodes::IsRampAhead / IsClearanceOk          （"触发行为"，读 status）
                          └─ 在自定义 BT XML 里替换/包住 FollowPath
```

**【推断】必须守住的一条边界**：**BT 条件节点不许把"判据"重新实现一遍**。
判据只在 `rm_elevation_map` 里有一份（"同一件事只在一处维护"，`docs/README.md` 维护约定），
BT 只读结论。否则调一次阈值要改两处，且两处会漂。

---

## 5. 怎么验证（**我们的独特优势：STL 是地面真值**）

### 5.1 为什么这是"别人没有"的东西

**【事实】** `src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world/meshes/RMUC2026.stl`
就是 Gazebo **实际加载的那份几何**（`GAZEBO_MODEL_PATH` 指向 `<share>/hzmi_rm_simulation/meshes`，
见 `docs/worlds.md` §2.3），sha256 `cc723f3156999598127dfd34d1467767773708ecd46e78f6ad4c40de597d0846`。
**【事实】** 它的换算口径已经完全定死：**毫米单位**、`<scale>0.001`、`<visual>/<collision>` pose
`z = +1.6413436` ⇒ **地板顶面落 world z = 0**（mesh 局部 `z = −1.64134362793`，STL 顶点里有
**4412 个点**精确落在这个平面上，见 `docs/worlds.md` §2.1）。
⇒ **我们可以离线算出"坡度/净空/落差的真值图"，然后与传感器派生层逐格比对。**
**【推断】** 这在实车上**不可能**：真实场地没有 CAD 真值，也没有"同一次实验里既有真值又有传感器"的条件。
所以 **"真值 vs 派生"的逐格比对是我们台架最有价值、也最难被复制的产出**。

### 5.2 真值图怎么算（离线，纯几何，可复跑）

| 步骤 | 做法 | 口径/命令 |
|---|---|---|
| ① 读 STL（不经 Gazebo） | 二进制 STL = `80 B 头 + uint32 面数 + 每面 50 B`；每面 = `3 float32 法向 + 9 float32 顶点 + uint16 属性` | **【本次核对】** `45900` 面、文件 `2295084 B`（`84 + 45900×50 = 2295084` ✓） |
| ② 单位与抬升 | 顶点 `× 0.001`、`+ 1.64134362793` ⇒ **world 系，且 world == mesh 米制**（`model pose = 0 0 0 0 0 0`） | `docs/worlds.md` §2.1 |
| ③ **真值高程图** | 每 (x,y) 格取**最高**面高度（保守，保住薄墙 —— 与 `docs/worlds.md` §3 第 1 步同口径） | **【本次核对】** 0.02 m 栅格：`803 × 1458`，有限格 `1 166 859` |
| ④ **真值坡度图** | **面级**法向与竖直夹角（`atan2(‖n_xy‖, ‖n_z‖)`），按面积加权统计/落格 —— **不要**用高度栅格差分（0.02 m 量化会把 12.8° 抬成 45°） | **【本次核对】** 见 §5.3 表 |
| ⑤ **真值净空图** | 每 (x,y) **从下往上**求该列穿过的所有面高度，排序 ⇒ **最高面 = 地表**，**其上第一个面 = 头顶表面** ⇒ `clearance = z_ceiling − z_ground` | **【本次核对】** 用 §5.3 的脚本（**不用 ray 库**：本机 `rtree/embreex/numba` 都没有，`trimesh.ray` 会直接 `ModuleNotFoundError`；用逐三角形 XY bbox 粗筛 + 重心坐标求交，`46 400` 列 `7.9 s`） |
| ⑥ 配准 | 真值图在 **world** 系；传感器在 **odom/map** 系。平移量 = 出生点 `(10.925, 2.525)`（`docs/worlds.md` §3），**且 model pose 是 `0 0 0 0 0 0` ⇒ 没有旋转** | **【事实】** 交叉验证：合成图 `map/RMUC2026.yaml` 的 `origin = [-25.925, -9.425]` **正好等于**"场地左下角 `(-15.0, -6.9)` − 出生点 `(10.925, 2.525)`" ⇒ "生成了 map 系图"这条链**已经自洽** |
| ⑦ 指标 | **逐格混淆矩阵**：真值可行驶 vs 派生 free ⇒ `TP / FP / FN / TN` + 准确率/召回/精确率；**FP 与 FN 必须分开报**（这两种错误后果不对称：FP = 删掉一条通路，FN = 把车送进不可走的地方）；再报**连通性**（真值里可走的区域在派生层里是否仍连通、绕行长度比） | 复用 `tools/scripts/mapping/compare_2d_maps.py` 的"区域计数"思路（它已支持 `--box/--png/--json`）+ 新增逐格矩阵 |

### 5.3 **【本次核对】** 真值几何（写本文时从 STL 现算，口径如上）

**面积分布（0.02 m 顶视栅格，`lift = z + 1.64134362793`）**

| 层 | 面积 | 备注 |
|---|---|---|
| `lift = 0.00 m`（可行驶地面） | **216.0 m²** | 与 `docs/worlds.md` §6 的"场地可行驶地板 214.6 m²"同量级（栅格口径差） |
| `lift = +0.20 m`（0.2 m 台面） | **106.2 m²** | `docs/ground_segmentation_slots.md` §5.2 记 106.2 m² ✓ |
| `lift = +0.30 m`（0.3 m 台面） | **69.2 m²** | 同处记 70.7 m²（栅格口径差） |
| `lift = +0.35 / +0.40 / +0.90 / +0.96 / +1.53 m` | 3.8 / 8.2 / 9.7 / 3.3 / — m² | 塔、柱、护墙 |

**坡度分布（面级法向、面积加权）**

| 倾角区间 | 面积 | 与已有记录对照 |
|---|---|---|
| 0–5° | **1486.5 m²** | `docs/ground_segmentation_slots.md` §5.2 "85.5% within ±5 deg" ✓ |
| 5–10° | 6.43 m² | — |
| **10–15°** | **11.23 m²** | 同处记 **11.2 m²** ✓ |
| **15–22°** | **16.69 m²** | 同处记 **16.7 m²** ✓ |
| 22–35° | 7.52 m² | 同处记 **7.5 m²** ✓ |
| 35–60° | 7.36 m² | 同处记 **7.4 m²** ✓ |
| 60–90° | 203.28 m² | 同处记 22.1 m²（**口径不同**：它按"倾斜面"统计，这里把竖直面全算进来） |

**最大的一块斜面（= 我们一直在说的"23° 坡"）** —— **【本次核对】面级精确值**：

| 面 | 倾角 | 面积 | 顶点（world，`lift`） | 几何含义 |
|---|---|---|---|---|
| A | **23.0°** | 1.16 m² | `(-3.643, 8.002, 0.000)` → `(-4.218, 8.002, +0.200)` → `(-6.240, 4.293, 0.000)` | 边沿坡，**水平投影宽度 0.45 m**、升 0.20 m |
| A′ | **23.0°** | 1.01 m² | `(-4.218, 8.002, +0.200)` → `(-6.485, 4.764, +0.200)` → `(-6.240, 4.293, 0.000)` | 同一坡的另一半 |
| **镜像 A** | **23.0°** | 1.16 + 1.01 m² | `(3.643, -4.713, 0)` / `(4.218, -4.713, +0.200)` / `(6.240, -1.004, 0)` / `(6.485, -1.476, +0.200)` | **+x 半场（出生点那半场）的同一块坡** |
| 12.8° 倒角 | **12.8°** | 1.27 + 1.27 m² | `(-1.429, 7.054, +0.200)` → `(4.188, 7.054, +0.200)` → `(3.880, 6.614, +0.300)` | 连通 h=0.20 与 h=0.30 两级台面；水平投影 0.44 m |

⇒ **【本次核对】的结论**：
* **23.0° 是面级精确值**（`docs/mapping_2d_from_cloud.md` §4.1 记 "23–24°"，**一致**）；
* **平面投影宽度 0.45–0.47 m**（同上记 "≈0.45 m"，**一致**）；该坡的**升 = 0.20 m**，
  而同处记 "升 ≈0.18 m" —— **【推断】** 0.18 是**车实际走过的坡段口径**（车没走满整块面，或走了
  `map` 系里那 0.45 m 投影里的一段），与面级 0.20 m 不矛盾；
* **12.8°** 面级精确值 = `atan(0.10 / 0.44) = 12.8°`，同处记 "12.8°"，**一致**；
* **镜像坡**：map 系 `x[-7.29,-4.44] y[-7.23,-3.52]`（= world `(3.64..6.49, -4.71..-1.00)` 平移出生点）
  —— 车爬的就是它；文档里按 world 负半场叙述的那一块（map `x[-17.41,-14.57]`，**在 map 窗口之外**）
  是它的镜像。

**⚠ 两处必须写下来的"达不到"（否则 §7 的验收会设成做不到的事）**

1. **【本次核对】RMUC2026 的 STL 里没有"暴露出来的 −0.2 m 坑"。**
   mesh 里确实有一层 `lift = −0.20 m` 的水平面，面积 **466.25 m²**（`docs/ground_segmentation_slots.md`
   §5.2 记 "坑 z = −0.200 m（466 m²）"，数字一致）；但它的 XY 范围
   `x[-14.505, 14.545] y[-6.401, 9.649]` **几乎等于整个场地**，即它是**底板（0.20 m 厚的底座）的底面**。
   逐列检查：**每一列的"最上面那个面"都是 `lift ≥ 0`**，那一层永远在它下面。
   ⇒ 从任何传感器（也从任何俯视真值图）看，这块场地是**平的 0 m 地面**，**没有落差可测**。
2. **【本次核对】RMUC2026 的 STL 里没有任何"头顶表面"：46 400 列（0.10 m 栅格）里，
   `clearance < ∞` 的列 = 0。**
   即：**每一列的竖直方向只穿过一个面**（"地表"），其上什么都没有。
   与 `.tmp_cache/stl_view/README.md` 的 ray-parity 结论一致：">=0.25 m 净空的 subsurface 空洞只有
   **3.7 m²**、全是 ≤0.15 m 宽的缝"。`docs/worlds.md` §6 也写着"隧道 / 设计坡道：**没有**"。
   ⇒ **净空层在这份场地里没有可验证的真值**；`docs/mapping_2d_from_cloud.md` §4.1 说的"走廊**无顶棚**"
   也在本次核对里被证实（走廊剖面 `y ∈ [-5.90, -4.73]` 的每一列 `clearance = ∞`）。

**⇒ 【推断】由此得到两条对计划有直接影响的结论：**
* **落空层与净空层在这份场地上"造不出用例"，必须自建代理**：落空用 §5.2 的方式在几何上加一个
  −0.2 m 的坑（或复用 `docs/ground_segmentation_slots.md` §5.2 的 **7 个 STL 代理场景**：
  "0.2/0.3 m 平台 + 10/15/22° 倒角 + −0.2 m 坑"）；
  净空需要在 Gazebo 里**加一根横梁**（或用 `ground_seg_ab` 那种解析生成的合成点云）。
* **但"派生 vs 真值"的基础设施仍然现在就值得做**：坡度层可以**完全**用现有场地验证
  （两个 23° 坡 + 三个 12.8/17/10° 倒角 + 0.2/0.3 m 台阶），而这正是我们**唯一量过的真特征**。

### 5.4 三个已量特征**应当**被怎么分类（验收判据的来源）

| 特征（map 系） | 真值层应给 | 派生层应给 | 判据（**验收用**） |
|---|---|---|---|
| **23° 坡**<br>map `x[-7.29,-4.44] y[-7.23,-3.52]`<br>（world `(3.64..6.49, -4.71..-1.00)`） | 坡度 **23.0°**；可行驶面（面法向在限内） | 坡度层报 **23±2°**；**不得**被判成"不可行驶"（> 阈值档）；`pcd_to_nav2_map` 口径下 **FREE** | ① 真值坡度与派生坡度的**中位差 ≤ 3°**；② 派生层在该区域的 **free 占比 ≥ 95%**（对照：已验收的 `RMUC2026_cloud` 在坡面区域 **98.8% free**，见 `docs/mapping_2d_from_cloud.md` §6.1）；③ **规划器对坡顶/坡中目标的成功率 ≥ 4/5**（对照：旧 `/scan` 图 2/5、点云图 5/5，同处 §6.4）<br>⚠ **前置决策**：若最终阈值取 25°，23° 只有 2° 余量 ⇒ 这条例必须**先有 §3.1 的三段式阈值**才谈得上"通过" |
| **1.05 m 走廊（无顶棚）**<br>map `y[-8.43,-7.26] x[-8.93,-3.63]`<br>（world `y[-5.90,-4.73] x[2.0,7.3]`） | 面高 **+0.20 m**、面本身 **≈0°**；**净空 = ∞**（无顶棚）；**最窄净宽 1.00 ~ 1.05 m** | 坡度层报 **≈0°**；净空层**必须不触发**（因为它真没有顶）；落空层**不得**把两侧边沿误判成"掉下去" | ① 真值净宽量出来（**【本次核对】0.02 m 栅格：1.00 m**；`docs/mapping_2d_from_cloud.md` §4.1 **1.05 m**）—— **【待测】把这两个口径的差说清楚**（见 §8 第 8 条）；② **与本车宽度的对照要报出来**：车体 **0.2 × 0.3 m**（碰撞盒）／nav2 圆 **`robot_radius 0.22 m`** ⇒ 1.00 m 净宽**余量 0.28~0.30 m/侧**（按半径口径）⇒ 静态上**过得去**；③ **落空层在该走廊 8 邻域上的 FP 必须为 0**（这是"落空误报会切断通路"最容易发生的地方） |
| **0.2 / 0.3 m 台面** | 高差 **+0.20 / +0.30 m**，立面 **60–90°**，连接处是 **10/12.8/17/22° 倒角** | `pcd_to_nav2_map` 口径下**台面顶面必须占用**（高度闸 0.15 m）；倒角本身**可行驶** | ① 真值高差与派生高度差的**中位差 ≤ 0.05 m**；② 台面顶面的**占用召回 ≥ 95%**（对照：合成代理里 0.2/0.3 m 平台顶面 `linefit 99.53%` / `patchwork 100%`，`docs/ground_segmentation_slots.md` §5.2）；③ **12.8° 倒角不得被判成"不可行驶"**（对照：`linefit` 在 10~22° 倒角上有 **71.8~73.5% 判成障碍**，这是**已知缺陷**，patchwork 降到 **30.9~31.3%**，同处 §5.2 —— 所以这条例要注明"用哪个分割器"） |

**【推断】还有一条只有真值能给的判据**：**连通性**。
"真值里可行驶的 216.0 m²，在派生层里还是不是（近似）连通的？"——
`docs/worlds.md` §6 已经量过：`clearance ≥ 0.22 m` 的集合 178.6 m²、同一连通域 176.3 m²、
`clearance ≥ 0.30 m` 时**裂成两块 80.6 / 80.5 m²**（两个半场不再连通）。
⇒ **派生层不该比真值"更碎"**：如果派生层把 176 m² 的可走区切碎成几十块，那不管逐格指标多好看，都是 fail。

---

## 6. 怎么看（可视化）

### 6.1 RViz 配方（按"要不要装包"分档）

| 想看什么 | 怎么显示 | 需要额外装包？ |
|---|---|---|
| **原始/派生点云的形状**（最省事，**零依赖**） | `PointCloud2` 显示 + **Color Transformer = `AxisColor`（选 Z 轴）** 或 `Intensity`；固定视角 `TopDownOrtho` 看俯视、`Orbit` 看立体 | **不需要**（rviz2 自带） |
| **把派生层当"假彩色点云"看**（坡度/净空/落差） | 把每个 (x,y) 格变成一个点，`z` 放真实高程、把**层值塞进 `intensity` 或 `rgb`** ⇒ 一个普通 `PointCloud2` 就能看三层 | **不需要**（我们自己出点云即可） |
| **GridMap（2.5D 层的原生显示）** | `rviz2` 的 **`grid_map_rviz_plugin`** 提供 `GridMap` display：每个 layer 一个可视化开关、可调色标 | **要装** `ros-humble-grid-map-rviz-plugin`（+ 产 GridMap 话题要 `ros-humble-grid-map-msgs`）；本机**未装**，apt 候选 **2.0.1-1jammy** |
| **把层当 OccupancyGrid 看**（插件缺失时的替代） | GridMap → OccupancyGrid 转换后当普通 `Map` 显示；或用 **`grid_map_costmap_2d`**（apt，**现成**的"GridMap ↔ costmap_2d"接口） | 转换库要装 `ros-humble-grid-map-costmap-2d`（本机未装，候选 2.0.1） |
| **OctoMap 的八叉树** | `octomap_rviz_plugins` 的 `OccupancyGrid`/`OccupancyMap` display | **要装** `ros-humble-octomap-rviz-plugins`（本机未装，候选 2.1.1）+ `octomap_server`（候选 2.3.1） |
| **现有代价地图/图层本身** | `nav2.rviz`（`src/nav/rm_navigation/rm_navigation/rviz/nav2.rviz`）里已有 costmap 显示；`Map` 显示看 `map` | 不需要 |

### 6.2 离线 PNG 路线（**现在就能用，不需要装任何东西**）

| 工具 | 产出 | 出处 |
|---|---|---|
| `tools/scripts/mapping/pcd_to_nav2_map.py --png <out.png>` | 俯视 PNG（带 `--report-box` 的统计框）；`--json` 同时落一份带全部面积统计与分区域分类的 manifest | `--help`；`docs/mapping_2d_from_cloud.md` §5.3 |
| `tools/scripts/mapping/compare_2d_maps.py --png` | 两张 2D 图的 A/B 对比图（灰/黑/品红/橙/红/绿六色差异图） | 同 §6.2（图在 `docs/img/ab_2d_compare.png`） |
| `.tmp_cache/stl_view/` 的渲染器 | **STL 的多视角 PNG**：`view_e_top_heightcolor.png`（按高度着色）、`view_f_levels_and_gaps.png`（离散层 + 每列竖直自由间隙）、`view_g_slices_solid_air.png`（8 个水平切片的 solid/air/roofed）、`view_i_measurements.png`（Z 直方图 + 坡度直方图 + 每层足迹） | `.tmp_cache/stl_view/README.md`；脚本 `render.py`（**纯 numpy z-buffer 软件渲染，不需要 OpenGL**）、`stl_lib.py`、`vox.py`、`parity.py`、`make_views.py` |
| **真值图渲染（本文建议新增，尚未有）** | 真值坡度/净空/落差图直接出 PNG（与派生层同色标、并排），做进 §5.2 的对比工具 | **【推断】这是第一步的交付之一** |

> ⚠ **`.tmp_cache/` 是未版本化的临时目录**（`git status` 里是 `??`）。要把 STL 渲染复用到
> "可复跑的工具"这一级，需要把用到的脚本**收敛成一个 `tools/scripts/world/` 下的工具**（别依赖 `.tmp_cache`）。

### 6.3 【事实】本机可视化能力核对（结论）

* **不需要装任何东西**：`PointCloud2` + Z 着色、`nav2.rviz` 的 costmap、`pcd_to_nav2_map.py --png`、
  `compare_2d_maps.py --png`、`.tmp_cache/stl_view` 的 STL 渲染。
* **要装才有的**：`GridMap` display（→ `ros-humble-grid-map-rviz-plugin`）、
  OctoMap 显示（→ `ros-humble-octomap-rviz-plugins`）、
  GridMap→costmap 转换（→ `ros-humble-grid-map-costmap-2d`）。
* **【推断】建议**：**第一步完全不装包**（用假彩色 `PointCloud2` + PNG）；等到"层要长期挂着看"时
  再把 `grid_map` 那三个包一次装齐（都是 apt、版本 2.0.1）。

---

## 7. 分阶段计划

### 7.1 排序原则

**【推断】** 按"价值 ÷ 成本"，并且**优先做能解锁后面所有工作的那一步**：
① 判据基础（真值 + 派生 + 对比）→ ② 坡度层进 nav2 → ③ 净空/落空。
理由：**没有第 1 步，"阈值定多少"永远只能拍脑袋**；而第 1 步**零风险**（全离线、不改栈、不装包）。

---

### 第一步：**离线判据基础**（真值图 + 派生层 + 对比工具）

| 项 | 内容 |
|---|---|
| **交付物** | ① `tools/scripts/world/` 下一个"由 STL 出真值图"的工具（高程/坡度/净空三张 + JSON manifest，含 sha256 与全部参数）；<br>② `tools/scripts/mapping/` 下一个"由 PCD 出 2.5D 层"的工具（**输出 `elevation / slope / height_diff` 三层**，而不是现在这一张 pgm；口径**默认值与 `pcd_to_nav2_map.py` 完全一致**）；<br>③ 一个"逐格对比"工具（TP/FP/FN + 连通性 + 并排 PNG）；<br>④ 一份结果记录（写进本文或新建 `docs/traversability_report.md`） |
| **不碰什么** | 不改任何 launch/param/节点；不装任何 apt 包；不改 `pcd_to_nav2_map.py` 的默认值（新工具独立） |
| **工作量【推断】** | 2~3 人日（真值工具 1 d，派生工具 0.5 d，对比 0.5 d，跑+写 1 d）。**【推断】** 真值工具的主要风险是"**本机没有 ray 库**"（`rtree`/`embreex`/`numba` 全无，`trimesh.ray` 直接报 `ModuleNotFoundError`）⇒ 用 §5.2 ⑤ 的"XY bbox 粗筛 + 重心坐标"纯 numpy 做法（**本次已经验证：46 400 列 7.9 s**） |
| **验收判据** | ① 真值工具自检：把 STL 的"地板顶面"算成 **`z = −1.64134362793 m`（mesh 系）**、面积与 `docs/worlds.md` §2.1 的"4412 个顶点精确落在该平面"口径对得上；<br>② 真值层能复现 §5.3 的四行坡度面积（1486.5 / 11.23 / 16.69 / 7.52 m²，容差 ±10%）；<br>③ **派生层能复现已验收的结果**：用 `PCD/RMUC2026_spl.pcd`（或 `_raw`）跑出的 2D 图与既有 `map/RMUC2026_cloud.pgm` 区域计数同量级（坡面 free ≥ 95%、走廊 free ≥ 95%）；<br>④ 对比工具能报出 §5.4 三条特征的分类结论，且 **FP/FN 分开列**；<br>⑤ **明确列出"这份场地验不了什么"**（落空无真值、净空无顶棚 —— §5.3 的两条） |

### 第二步：**坡度层进 nav2（写 cost）**

| 项 | 内容 |
|---|---|
| **前置** | 第一步通过；§3.1 的**三段式阈值**已定（**先用 15/25 两档，并显式标注"25° 来自 `pcd_to_nav2_map.py` 的旧默认值"**） |
| **交付物** | ① 离线烘出的 `<world>_slope.pgm`（三档）+ 复用既有 `static_layer` 机制挂上（**不改 launch**：`static_layer` 已在 params 里，加一个实例或加一张图）；<br>② 若"一张 pgm 不够"（要连续代价），再写 `src/nav/rm_navigation/rm_costmap_layers/` 的 `SlopeLayer`（`nav2_costmap_2d::Layer`），`enabled` 挂进 `navigation_launch.py` 的真值表；<br>③ A/B + 回归 |
| **工作量【推断】** | 静态图路线 0.5~1 人日；写插件 + 挂槽位 + A/B **3~5 人日**（含 `local_obstacle_truth_table.py` 那类静态校验脚本的更新） |
| **验收判据** | ① **23° 坡在真值层与派生层都是"可行驶"**（§5.4 第 1 行三条）；<br>② `nav_smoke_regression.py` 的 **P0 回归 PASS**，且 `recoveries` / 用时与基线同级（对照：换图那次 `SUCCEEDED 54.2 s vs 57.9 s`、`recoveries` 都是 4，`docs/mapping_2d_from_cloud.md` §6.4）；<br>③ **规划器对 5 个坡道/走廊目标的成功率 ≥ 4/5**（基线 2/5 → 点云图 5/5）；<br>④ **不许把可走区切碎**：派生层里"从出生点可达的面积"不得比真值口径小 10% 以上（对照真值 176 m²，`docs/worlds.md` §6）；<br>⑤ 槽位互斥/真值表校验脚本仍然 PASS |

### 第三步：**净空层 + 落空层（触发与保险）**

| 项 | 内容 |
|---|---|
| **前置** | ① 需要一个**带特征的可验证场景**（§5.3 已证：RMUC2026 里**没有**顶棚、**没有**暴露的坑）⇒ 在 Gazebo 里加一根横梁 / 一个 −0.2 m 坑（或复用 `ground_seg_ab --synthetic` 的 7 个 STL 代理 + 3 个薄墙场景）；<br>② §8 第 5 条（触发安全性）有答案 |
| **交付物** | 净空层（判据 + 层 + 三档输出）；落空层（判据 + **独立 no-go 掩膜**，先不写 lethal）；`/traversability/status` 话题；**BT 条件节点 + 自带 BT XML**（§4.3）；A/B |
| **工作量【推断】** | 场景搭建 1~2 人日；净空判据+层 2~3 人日；落空判据 3~5 人日（**误报调参是最贵的一段**）；BT 节点+XML 2~3 人日；A/B 与验收 2 人日 ⇒ **合计 10~15 人日** |
| **验收判据** | ① **限高**：梁下 0.30 m（< 车高 0.224 + 余量）判"不可过"、梁下 0.60 m 判"可过"（对照 `robot_height ≈ 0.224 m`，§3.3）；<br>② **落空**：−0.2 m 坑被标 lethal（对照 `ground_seg_ab --synthetic` 的 "−0.2 m 坑" 代理场景）；<br>③ **误报**：在 1.05 m 走廊与 23° 坡上，落空层 FP = 0（§5.4 第 2 行第 ③ 条）；<br>④ **触发**：BT 在限高不足时**不进入**，日志里有可归因的状态（不是"莫名其妙停了"）；<br>⑤ 落空/净空层**默认关闭**（`enabled: false`），A/B 才开 —— 保证"不动已验证行为" |

### 7.2 **明确不做（现在不做，以及为什么）**

| 不做 | 为什么（**理由要可核对**） |
|---|---|
| **OctoMap** | ① 我们缺的是"坡度/净空/落差"这三个**标量场**，不是"体素占据概率"；<br>② **【事实】** 本场 STL 逐列检查：**没有任何头顶表面**（46 400 列中 `clearance < ∞` 的列 = 0）⇒ 上 OctoMap 在**这份场地**上换不来任何新信息；<br>③ 它是新的调参面（hit/miss、分辨率、ray casting 开销）而**当前栈的瓶颈不是感知**（`gzserver` 占 0.52 核，是分割器的 6~9 倍，`docs/ground_segmentation_slots.md` §6.1）；<br>④ 触发条件写死：**"出现一个场景，2.5D 层无论怎么调都把可走的路判成不可走（多层/悬挑/隧道）"** 才装 |
| **ESDF 体素 / MPC（voxblox / nvblox / FIESTA）** | ① 它们的消费者是 **3D 规划器/MPC**，而我们的决策层明确"继续 2D、不上 3D 规划器"（`docs/algorithm_matrix.md` §8.2）；<br>② 距离场不直接回答"这个坡度能不能上"；<br>③ nvblox 要 GPU；<br>④ **【推断】** 麦克纳姆底盘在 23° 坡上的运动学与平地差别不是"轨迹形状"而是"打滑/姿态"，那是**控制器限速问题**，不是 ESDF 能解决的 |
| **3D 规划器 / 3D 代价地图** | 同 ①：nav2 的插件契约（`nav_msgs/Path` + 2D costmap）就是 2D；哨兵是平地麦轮车，"必须做 3D 决策"的场景在这份场地上不存在 |
| **把 `ground` 默认从 `linefit` 换成 `patchwork`** | **【事实】** `docs/ground_segmentation_slots.md` §7.1 已经拍板"**不换**"，并列了 5 个触发条件（`mode:=nav` 整栈 A/B、P0 回归、第二个场地、两个新毛病确认、实车 `sensor_height` 重标定）。**【推断】** 本计划**不碰这个决策**；但第三步的落空层如果做，`patchwork` 在"−0.2 m 坑的下坡倒角"上**比 linefit 更差**（52.5% vs 8.1% 障碍率，同处 §5.2）这条必须重新看 |
| **专门做"1.05 m 走廊的通过优化"** | **【事实】** 车体 0.44 m 宽 / nav2 半径 0.22 m ⇒ 静态余量够（§5.4 第 2 行）；**【事实】** 实测卡死发生在**更窄的口**（`docs/worlds.md` §6：两个半场之间最窄 **0.45 m**，"余量 5 mm/边 ⇒ 实际过不去"），且 `map≈(0.5, 3~6)` 的北侧通道有**三次卡死**（`docs/mapping_small_point_lio.md` §9 第 1 条，**根因未定位**）。⇒ 走廊不是问题，"0.45 m 口"才是，而那是**几何上过不去**，不是感知问题 |
| **为"落空"新造 lethal 语义** | 用 nav2 现成的 `LETHAL_OBSTACLE` + 内切/膨胀即可；新造语义会与 `robot_radius 0.22` / `inflation_radius 0.5|0.55` 的既有语义打架 |
| **改 `pcd_to_nav2_map.py` 的默认阈值** | 它已经被验收（`docs/mapping_2d_from_cloud.md` §6），**改默认值会让已验收的结论失效**。新层用**新工具**，旧工具逐字节不动 |

### 7.3 **【推断】如果只能做一件事**

**做第一步。** 因为它同时产出三样别人拿不走的东西：
① 一份**真值图**（离线、可复跑、有 sha256）；② 一条**判据基线**（坡度/净空/落差的阈值从"拍"变成"量"）；
③ 一个**可复用的对比工具**（后面每加一层都靠它验收）。
而第二步、第三步都是"在判据之上加东西"——没有第一步，它们只能靠"看着 RViz 感觉还行"。

---

## 8. 开放问题 / 未验证

> **2026-10-07 更新**：本节的"计划 vs 现状"逐行对照见 **§9**（A 项已落地，B 项做了一半，C 项没做）。

> 每条格式：**问题** → 我们知道什么 → **怎么确认（命令/做法）** → 影响哪一步。

1. **【待测】本车（仿真）实际能上多少度的坡？**
   * 已知：**坡度幅值**有实测（23°、12.8° 等，§5.3），**阈值**只有 `pcd_to_nav2_map.py` 的 `--slope-limit 25°`（一个判据参数，不是本车能力）。
   * 怎么量：Gazebo 里做**坡度阶跃**测试（用 `tools/scripts/world/stl_to_world.py` 搭带 5/10/15/20/25/30° 坡的世界；
     或直接用 `ground_seg_ab --synthetic --synthetic-long-ramp` 那类解析坡度场景的**物理版**），
     发固定 `linear.x` 看 `/odom_ground_truth` 的位移/打滑与 roll/pitch；
     同时用 `tools/scripts/mapping/ramp_section_probe.py` 的口径记录"发着速度但不动"的窗口
     （它对 20 航点路线实测过 `motion_checks_failed` 与 LIO `roll ptp 24.9°`）。
   * 影响：**第一步的阈值**与第二步的验收（§5.4 第 1 行）。
2. **【待测】p2l 的 `max_height 0.1` 把 0.4~1.0 m 的点全丢了，是谁的锅、要不要动？**
   * 已知：**【事实】** `docs/ground_segmentation_slots.md` §6.2 的高度带覆盖率表：**0.40~1.00 m 档覆盖率 0.0% / 7.3%**，
     并明确更正"**不是分割器的锅，是 `pointcloud_to_laserscan` 的 `max_height: 0.1`**"；
     后果：护墙上部只能靠 **STVL（`min_obstacle_height 0.0`）**那条路，`/scan` 里没有。
   * ⚠ **与我们三层的关系**：这条**只影响 `/scan`**，**不影响**走点云的坡度/净空层（点云没有高度带闸，`docs/mapping_2d_from_cloud.md` §10.3）。
   * 怎么确认：把 `p2l` 的 `max_height` 临时提到 1.0（与 `min_obstacle_height 0.0` 对齐）跑一次 `local_obstacle:=scan` vs `stvl` 的 A/B，
     看 `/scan` 的 0.4~1.0 m 档覆盖率是否从 ~0 升上去、且**近距假回波不增加**（对照：`docs/stvl_local_costmap.md` 的"贴墙 0.35~0.40 m 仍标到墙"）。
   * 影响：第三步的净空层若走 `/scan` 就**永远看不到 0.4 m 以上的横梁** ⇒ **必须走点云**。
3. **【待测】"车高"到底取哪个数？**
   * 已知：**【事实】** 仿真包络 ≈ **0.224 m**（§3.3，由 xacro 逐件加出来）；**真车** `sentry_robot_real.xacro` 的雷达在 **z = 0.49 m** ⇒ 整车更高。
   * 怎么定：① 对仿真用 `0.224 + margin`；② 对真车**重新量**（含云台/护板/天线的最高的那个点），
     并把结果写进 `docs/sim_real_contract.md` 那类契约文档；③ 用 `robot_radius: 0.22` 的语义作对照（它是**水平**半径，不是高度）。
   * 影响：净空层阈值（§3.3）与第三步验收（梁下 0.30 / 0.60 m 这两档）。
4. **【待测】本车的姿态极限（能不能在 23° 坡上保持可用定位/扫描？）**
   * 已知：**【事实】** 在 23° 坡段实测 LIO **pitch ptp 33.1°、roll ptp 24.9°**、坡道窗口内 `roll ptp 23.1°`
     （`docs/mapping_2d_from_cloud.md` §4.4），说明"底盘确实被顶起来/侧倾"；
     而 LIO 在这段的 `/scan` 有限束数只有其余段的 **31%**（同 §4.3）。
   * 怎么量：在"纯坡道"路线上用 `mapping_ab_run.sh --route …` 跑 5 次，报 `roll/pitch ptp`、
     `LIO ATE`、`/scan` 束数、`map→odom` 跳变哨兵的次数（工具已有）。
   * 影响：坡度层的**限速档**（15~25° 那档到底限到多少），以及"要不要在坡上触发"。
5. **【待测】动作层的触发怎么保证安全？（本文最没底的一条）**
   * 已知：**【事实】** 仓库里**没有**自定义 BT 节点、**没有** `default_bt_xml_filename`、**没有**自定义 `behavior_plugins`（§4.2）。
   * 开放点：① 触发是"**进入前**判"还是"**进入后**判"？（净空必须进入前，落空只能进入后）
     ② 触发之后**停在哪**：坡中/路口停车会不会更危险（**【事实】** 窄口处被物理卡死实测发生过 3 次，`docs/mapping_small_point_lio.md` §9）；
     ③ 状态过期怎么办（latched 话题 + 有效期字段 vs 每周期重算）。
   * 怎么确认：先在**离线回放**（`ros2 bag play` + 只起 `bt_navigator` + 假 costmap）里验 BT 分支，
     再 `mode:=nav` 单次跑，**最后**才 A/B。**【推断】** 这一步之前不动 BT。
6. **【待测】坑底/洼地的传感器覆盖（落空层的"看得见"前提）。**
   * 已知：**【事实】** MID360 的**下视 FOV 只有 −7.22°**（`src/simulation/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro` 的
     `<vertical><min_angle>${-7.22/180*M_PI}</min_angle>`），且 `docs/issues_and_findings.md` #26 的几何结论：
     "高 h 的墙需 `d ≥ (0.226 − h)/tan 7°`（h=0.15 m→0.62 m）⇒ **越近越看不见**"。
     **【推断】** 同一个几何对"坑"更狠：站在坑沿上，**坑底**在雷达的 −7.22° 以下 ⇒ **根本扫不到**。
   * 怎么量：在自建的 −0.2 m 坑代理场景里，把车停在坑沿不同距离（0.3/0.6/1.0/1.5 m），
     统计"坑底有回波的帧占比"与"坑沿被判落空的帧占比"；工具用现成的点云落盘 + `pcd_stats.py`。
   * 影响：**落空层的能力边界**（是"能看出这里掉下去了"还是只能"看出边沿"）。**【推断】** 大概率是后者 ⇒
     判据应当写成"**边沿/落差**检测"，而不是"坑底检测"。
7. **【待测】`p2l` 的 `range_min` 回退档位与落空层误报的耦合。**
   * 已知：**【事实】** 历史上 `range_min` 由 **0.45 → 0.2 → 0.12 → 0.05**（`docs/algorithm_matrix.md` §七 2026-09-21 行 +
     `config/laserscan_params.yaml` 的两次注释），当前 **0.05**；`docs/stvl_local_costmap.md` §168/§232 实测"45 cm 盲区已不存在"。
     `config` 里留的回退档位是 **0.15 / 0.2**。
   * 风险：**【推断】** 一旦回退到 0.15/0.2，车体半径 0.2 m 内的地面就没有回波 ⇒ **"无地面回波"这条判据会把车下一圈判成落空**。
   * 怎么确认：把落空层在"回退档位"下重跑同一段数据，报车周 0.5 m 环带上的 FP 计数。
   * 影响：落空层判据（§3.2）是否必须加"**车体半径内不判**"这条豁免。
8. **【待测】1.05 m 与 1.00 m 的走廊宽度差到底是什么口径？**
   * 已知：**【事实】** `docs/mapping_2d_from_cloud.md` §4.1 记"宽 **1.05 m**"（map 系 `y∈[-7.35,-8.35]` ⇔ world `y∈[-5.825,-4.825]`，即 **1.00 m 跨度**）；
     **【本次核对】** 0.02 m 栅格量到平台面（`lift=0.20`）在 x=2.5~7.3 上**恒为 1.00 m**；
     剖面显示 `y=-5.90` 是低处地面、`y=-5.85` 已到 0.20 m，`y=-4.73` 回到低处 ⇒ **墙到墙的净宽 ≈ 1.05~1.07 m**。
   * **【推断】** 差的是"薄墙/台阶沿自己的厚度"：0.05 m 栅格把"面"和"沿"各算了一格。**结论（供验收用）**：
     写验收时**必须写明口径**（"平台面宽度 1.00 m" 还是 "墙面到墙面 1.05 m"），否则 5 cm 的差会被当成 bug。
   * 怎么确认：用 0.05 m 栅格重算一次平台面宽度，与 0.02 m 的结果并列写下来。
9. **【未确认】仓库里到底有没有自定义 BT 节点 / 自带行为树 XML？**
   * 本文的结论（"没有"）是**从 params 反推**的（`plugin_lib_names` 31 项全是 `nav2_*`、无 `default_bt_xml_filename`）。
   * 怎么确认（**写代码前第一件事**）：`grep -rn "BT_REGISTER_NODES\|bt_nodes\|default_bt_xml" src/ --include=*.cpp --include=*.hpp --include=*.py --include=*.xml`。
   * 影响：§4.2 的"现成钩子"清单与 §4.3 的挂载方式。
10. **【未验证】`nav2` 的 `static_layer` 能不能挂两个实例（两张图）？**
    * 本文把它当作"零代码把坡度层送进 costmap"的主路线（§3.1 路线 1），但**没有实测**。
    * 怎么确认：看 Humble 版 `nav2_costmap_2d` 的 `StaticLayer` 实现是否对 `map_topic` 有全局假设；
      最直接的实验是**加一个实例、给不同 `map_topic`/不同 `plugin` 名**跑一次 `mode:=nav`，看两张图是否都生效。
    * 影响：第二步是否必须写插件（若两个实例不行，"零代码路线"不成立）。
11. **【未验证】`grid_map` 2.0.1 在 Humble 上与我们这份 nav2/`tf2` 版本是否真的兼容。**
    * 本文推荐"离线出层、必要时装 `grid_map` 三个包"，但**没有装、没有跑**。
    * 怎么确认：`sudo apt install ros-humble-grid-map-ros ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin ros-humble-grid-map-costmap-2d`
      后 `ros2 pkg list | grep grid_map`、起 rviz2 看有没有 `GridMap` display。**【推断】** 风险低（官方发布），但**未验就是未验**。
12. **【未验证（外部资料）】elevation_mapping / elevation_mapping_cupy 在本机的可行性。**
    * 已知：**【事实】** 本机**没有** apt 包（`apt-cache search ros-humble-elevation` 空），上游有 ROS 2 分支
      （[cupy 的 ros2 README](https://raw.githubusercontent.com/leggedrobotics/elevation_mapping_cupy/ros2/README.md)）。
    * 怎么确认：先**不装**。判据是"我们自己那份离线判据站不住、而它的机器人中心高程图语义明显更好"（例如需要
      **姿态补偿/多传感器融合/滚动窗口**）。到那时再评估要不要为它引入源码编译与 GPU 依赖。

---

## 9. 实施状态（2026-10-07 更新：计划 vs 已实现）

> 本文原本是 **plan only**（一个字节都没改）。2026-10-07 的修复把其中**两条半**落地了
> （判据 + **坡度/台阶连续量与限速**），下面把"计划"与"现状"逐行对上，
> 省得下一个人以为 §3 的设计已经全做完。

| 计划里的项 | 现状 | 证据 / 位置 |
|---|---|---|
| **A. 坡度（"这格是能开的斜面还是台阶沿"）判据** | ✅ **已实现（实时 + 离线同一套阈值）** | `src/perception/rm_perception/rm_ground_traversability/include/rm_ground_traversability/low_terrain_classifier.hpp`（实时，接在两个地面分割节点之后）；阈值真源 `src/bringup/rm_nav_bringup/config/traversability_criteria.yaml`（离线 `pcd_to_nav2_map.py --criteria-file` 读同一份）。实测见 `docs/ground_segmentation_slots.md` §10 |
| A 的"**坡度值本身要发布出去**"（连续量，供限速） | ✅ **2026-10-07 已做**：同一帧的粗格缓存上再算一次"车前 1~3 m 走廊"的**连续量**（局部坡度、**坡度变化** `dtan`、**台阶残差** `step`），并按物理标定的速度表 + 刹车距离界换成**速度上限**，发 nav2 原生 `nav2_msgs/SpeedLimit`（controller_server → MPPI `setSpeedLimit()` 缩放 `vx_max/vy/wz`） | 连续量出口：`low_terrain_classifier.hpp` 的 `CorridorProfile`；速度映射：`slope_speed_limit.hpp`；ROS 接线：`traversability_ros.hpp`；**独立报告**（含物理推导与整栈 A/B）：`docs/slope_speed_limiting.md` |
| A 的"**2.5D 高程图层**" | ❌ 没做（不建 `rm_elevation_map`） | 判据仍是"点云 + 栅格"的一次性计算，没有跨帧高程图 |
| **B. 落空 / 负障碍（坑、边沿）** | ⚠️ **只做了一半**：台阶/边沿的"上跳"能判（`step_edge`），**"下跳/落空"没有独立判据** —— 坑沿只在"高差判据"能覆盖的方向上被抓到（判据是相对**局部最低**地面的高差，向下跳变在点云里表现为"那一侧没有点"） | `docs/ground_segmentation_slots.md` §10.4 的"台阶/边沿格"口径就是上跳；下跳缺"反证"结构（本文 §2 表 B 行原样） |
| **C. 净空高度（限高）** | ❌ **没做**（也因此 `p2l max_height` 抬到 1.226 m 之后，"头顶结构"会成为假障碍 —— 本场地没有悬挑，真机/别的场地必须另想办法） | `docs/ground_segmentation_slots.md` §11 的风险段 |
| **动作怎么接**：写 cost（costmap 图层） | ⚠️ **绕过了本文的设计**：现在不新增 nav2 图层，走的是"**把判据命中的点塞回 `/segmentation/obstacle`**"这条更窄的路（+ `p2l` → `/scan` → STVL/静态层）；"该不该限速"则走 **nav2 原生的 `SpeedLimit` 通道**（新增的第 3 条路） | 好处：零新增图层、零 nav2 代码改动、单发布者契约不变（`/cmd_vel` 仍然只有 `velocity_smoother` 发）；`SpeedLimit` 这条是 nav2 1.1.20 自带的订阅/接口，命令链上没有新节点 |
| **B/C 的"触发行为"（BT 条件节点）** | ❌ 没做（**也不打算做"停/取消目标"那一类**：本次落地的动作是"**限速**"——不停车、不取消目标、无 watchdog，见 `docs/slope_speed_limiting.md` §1） | — |
| **验证方法（本文 §5：STL 真值逐格对账）** | ✅ **已实现并跑过** | `tools/scripts/regress/verify_low_terrain.py`（台阶/边沿覆盖率、可行驶斜面被误占率、图上最短路到撞击点的距离与沿途最大局部台阶） |

### 9.1 与本文 §1.1 验收表的对照（本次实测）

| 本文写的验收条 | 本次实测（旧图 → 重跑图/实时链路） |
|---|---|
| ② 台面顶面占用召回 ≥95% | ⚠️ **没达到**：旧图"台阶/边沿格"被表示 **49.44%**；重跑图（同一 PCD + 同一判据）**57.08%**。原因不是判据，是**2D 图只记 (x,y)**：台面内部合法为 free，只有边沿必须占用 —— 而"边沿格"的定义（比 3×3 邻域最低高 >0.12 m）会把很宽的边带都算进去 |
| ③ 12.8° 倒角不得被判"不可行驶" | ✅ **达到**：判据接上之后 10–22° 倒角的障碍率只 +0.2~0.5 pp（`docs/ground_segmentation_slots.md` §10.4） |
| ① 真值高差与派生高度差中位差 ≤0.05 m | ❌ 未测（需要把判据的高度差发布出来，属上表"斜率连续量"那条） |
| **新发现（本次）**：累积点云的**垂向拖影**会让 3D→2D 投影**大面积误占** | `PCD/RMUC2026_lt.pcd` 在真值完全平坦（0.000±0.001 m）的 `map(−5,2)` 处，单帧累积点云的 z 跨度 **0.154 m**、单个 0.05 m 细格内 z 跨度中位 **0.085 / p90 0.169 m** ⇒ 重投影后 `occupied` 122.6 m²（旧 /scan 图只有 22.7 m²），其中 94.7% 的占用格真值局部高差 ≤0.10 m。**实时链路不受影响**（判据跑在**单帧**上，没有累积拖影） |

## 附录 A：本文引用到的关键事实索引（一句话 + 出处）

| 事实 | 出处 |
|---|---|
| 坡面在 `/scan` 累积图里 **100% unknown**（1680/1680），走廊 **2688 格全在图外** | `docs/mapping_2d_from_cloud.md` §6.1 / §6.3 / §10.4 |
| 点云投影图：坡面 **98.8% free**、走廊 **97.3% free** | 同上 §6.1 |
| 规划器对坡道/走廊 5 个目标：**2/5 → 5/5** | 同上 §6.4 |
| `pcd_to_nav2_map.py` 判据：`k=8` 法向、`--slope-limit 25°`、`--ground-cell 0.20 m`、p05、`--height-threshold 0.15 m` | 同上 §5.1；`--help` 默认值 |
| 「墙会被自己当地面」：1493 个占用格里 **1418** 个被吸收 | 同上 §5.1 / §10.3 |
| `map/RMUC2026.yaml` origin `[-25.925, -9.425]` = 场地左下角 − 出生点 `(10.925, 2.525)` | `docs/worlds.md` §3 / §4.1 |
| STL：45900 面、sha256 `cc723f31…`、`<scale>0.001`、抬升 `+1.6413436`、地板顶面 mesh `z=-1.64134362793`（4412 个顶点） | `docs/worlds.md` §1 / §2.1 |
| 场地可行驶地板 214.6 m²、free 237.8 m²、`clearance≥0.22` → 176 m²、`≥0.30` 裂成 80.6/80.5 m² | `docs/worlds.md` §6 |
| 两个半场最窄 **0.45 m**，"余量 5 mm/边 ⇒ 实际过不去"，绕行 41.7 m | 同上 |
| 隧道/设计坡道：**没有** | 同上 |
| 地面分割：22~35° 缓坡 linefit 判障碍 **35.7~44.3%** → patchwork **2.5~3.0%** | `docs/ground_segmentation_slots.md` §6.2 / §7.1 |
| `p2l` 的 `max_height 0.1` ⇒ **0.40~1.00 m 档覆盖率 ≈0** | 同上 §6.2（重要更正段） |
| `th_dist 0.125→0.08` 的 A/B（0.15 m 薄墙覆盖率 11.8% → 100%） | 同上 §3.1 |
| `ground` 槽位默认 **linefit**、不换的 5 个条件 | 同上 §4.1 / §7.1 / §7.2 |
| `/segmentation/ground` 每帧只有 **~700 点**（≈11 KB） | `docs/issues_and_findings.md` #25 的更正 |
| `p2l range_min` 历史 `0.45 → 0.2 → 0.12 → 0.05`；"45 cm 盲区已不存在" | `docs/algorithm_matrix.md` §七（2026-09-21 行）；`docs/stvl_local_costmap.md` §168/§232 |
| MID360 下视 FOV **−7.22°**、"越近越看不见" | `src/simulation/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro`；`docs/issues_and_findings.md` #26 |
| 2.5D 空缺槽位：`rm_elevation_map` / `rm_costmap_layers` | `docs/algorithm_matrix.md` §一.1 / §六；`docs/architecture.md` §3.2.7 |
| 维度决策：导航层**继续 2D**、2.5D 只两处增量、3D 不进在线决策 | `docs/algorithm_matrix.md` §8.2 |
| 图层槽位真值表与 `enabled` 生成逻辑 | `src/nav/rm_navigation/rm_navigation/launch/navigation_launch.py:50-62, 86-106` |
| nav2 参数：`robot_radius 0.22`、`inflation_radius` 0.5/0.55、`max_obstacle_height 2.0`、`min_obstacle_height 0.0`(2026-09-24 由 0.2)、`bt_navigator.plugin_lib_names` 31 项、`behavior_plugins` 4 项、**无** `default_bt_xml_filename` | `src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml`（:82-121 / :136-147 / :160-197 / :225-251 / :288-295 / :303-355 / :378-385 / :402-412） |
| 本车几何：轮半径 0.06、轮宽 0.05、`base_link` 碰撞盒 0.2×0.3×0.1、IMU 盒顶 z=0.15、Livox 在 `+(0.12,0,0.175)` | `src/bringup/rm_nav_bringup/urdf/sentry_robot_sim.xacro`（:5-6 / :22-40 / :155-186 / :262-264） |
| 真车 Livox 在 `+(0.0,0.045,0.49)` | `src/bringup/rm_nav_bringup/urdf/sentry_robot_real.xacro`（:16） |

## 附录 B：复跑本文"本次核对"的命令

```bash
# ① 包可用性（§2.2）
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -iE 'octomap|grid|elevation|voxblox|nvblox|esdf|fiesta|voxel'
apt-cache policy ros-humble-grid-map-ros ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin \
                 ros-humble-grid-map-costmap-2d ros-humble-octomap ros-humble-octomap-server \
                 ros-humble-octomap-rviz-plugins
apt-cache search ros-humble-elevation      # 期望：空（Humble 无 apt 包）

# ② STL 面级坡度/面积 + 台面面积（§5.3）
#    二进制 STL：80 B 头 + uint32 面数 + 每面 50 B（3f 法向 + 9f 顶点 + uint16）
python3 - <<'PY'
import numpy as np, struct
p='src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world/meshes/RMUC2026.stl'
d=open(p,'rb').read(); n=struct.unpack('<I',d[80:84])[0]
arr=np.frombuffer(d,dtype=np.uint8,count=n*50,offset=84).reshape(n,50)
T=np.frombuffer(arr[:,12:48].tobytes(),dtype='<f4').reshape(n,3,3).astype(float)*0.001
base=-1.64134362793; zl=T[:,:,2]-base
A=0.5*np.linalg.norm(np.cross(T[:,1]-T[:,0],T[:,2]-T[:,0]),axis=1)
nn=np.cross(T[:,1]-T[:,0],T[:,2]-T[:,0]); nm=np.linalg.norm(nn,axis=1)
ang=np.degrees(np.arccos(np.clip(np.abs(nn[:,2])/np.where(nm>0,nm,1),0,1)))
for lo,hi in [(0,5),(5,10),(10,15),(15,22),(22,35),(35,60),(60,91)]:
    m=(ang>=lo)&(ang<hi); print(f'{lo:3d}-{hi:3d} deg  {m.sum():6d} tris  {A[m].sum():8.2f} m2')
PY

# ③ 真值"净空"（§5.3 第 2 条）：本机没有 rtree/embreex/numba ⇒ 别用 trimesh.ray
#    做法：对每个 (x,y) 用三角形 XY bbox 粗筛 + 重心坐标求交，收集该列所有面高，排序后取"最高面之上第一个面"
#    （本文用量：0.10 m 栅格 46 400 列，7.9 s）
python3 -c "import importlib;[print(m, (lambda: (importlib.import_module(m), 'OK')[1])() if True else '') for m in ['rtree','embreex','numba']]" 2>&1 | tail -3
```


---


## 来源 8/19：`nav/kinodynamic_esdf_mpc_notes.md`


> **调研日期**：2026-10-05（只读调研：`raw.githubusercontent.com` + GitHub REST API + `web_search`）。
> **仓库快照**：branch `master`，HEAD = `8e6561ea`（2026-05-06T11:47:00Z），34★ / 3 fork，创建于 2025-01-12。
> **引用约定**：
> - `R:<path>:L<n>` = <https://github.com/865749/Kinodynamic_esdf_mpc/blob/master/><path>#L<n>（master 上逐行核对）。
> - 本机文件用相对工作区路径的 Markdown 链接（工作区 = `<repo>`）。
> - 本机环境检查命令：`ls /opt/ros/humble/share/*`、`dpkg -l`、`ls /usr/include/...`、`ros2 pkg list`。
> - 【事实】= 仓库/本机可直接核对的原文；【推论】= 我基于事实的判断，**请按打折使用**。

---

## 0. 一句话结论

【事实+推论】这是 **RoboMaster 竞赛（RMUC/RMUL）的一台全向/麦轮底盘的自主导航代码**，本质是 **HKUST Fast-Planner 系（GPL-3.0）的 ROS 2 Humble 移植 + 2D 化**：`plan_env`（**2D** 增量 EDT/ESDF，吃 `/scan`）+ `path_searching`（kinodynamic A*）+ `bspline_opt`（NLopt 轨迹优化）+ `planner_manager`（**ACADO 导出 RTI 求解器**，30 步 @0.1 s 的一阶全向运动学**跟踪型 MPC**，控制量 `vx, vy, w`，控制箱约束 ±0.5/±0.5/±0.1）。
**关键反直觉点：它的 MPC 里没有障碍项、没有 ESDF**——避障全在 kino A* + B-spline 里（`ACADO_NOD = 0`，无在线数据，R:planner_manager/omni_model/acado_common.h:L67）。所以"ESDF MPC"这个名字是**松的**：ESDF 供规划用，MPC 只做轨迹跟踪。

---

## 1. 身份 / 形态 / 许可证

| 项 | 值 | 出处 |
|---|---|---|
| 作者 | user `865749`（GitHub id 128976783，创建 2023-03-26，**仅 1 个公开仓库**，无 name/bio/org） | <https://api.github.com/users/865749> |
| 包维护者 | `z <1141942432@qq.com>`（五个自研包 package.xml 均此署名） | R:planner_manager/package.xml:L7；R:plan_env/package.xml:L7 |
| 上游/血统 | 【推论】Fast-Planner（HKUST-Aerial-Robotics，**GPL-3.0**，3418★）的 ROS 2 + 2D 化改造：命名空间 `fast_planner`、类名 `SDFMap`/`EDTEnvironment`/`KinodynamicAstar`/`BsplineOptimizer`/`kino_replan_fsm` 全同源；`fillESDF` 即 Felzenszwalb–Huttenlocher 一维平方距离变换（R:plan_env/src/sdf_map.cpp:L352-L390） | R:plan_env/include/plan_env/edt_environment.hpp:L11；<https://api.github.com/search/repositories?q=Fast-Planner+in:name> |
| 首次提交 | `c1a76395`（作者 **Ouuu**，2025-01-12T12:47:48Z "update"）；第二个提交 `fc46d37d`（865749，"acado mpc"）；共 **13** 个提交，之后全是 README 编辑 | <https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/commits?per_page=30> |
| 分支 | 只有 `master`（**无 `main`**） | <https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/branches> |
| 是什么 | **比赛代码**（不是论文代码）：地图/PCD 全是赛场资产 `RMUC2025.pgm`/`RMUC.pcd`/`RMUL.pcd`；串口驱动直连 RoboMaster C 板发 `vx,vy,vz`。 | R:planner_manager/map/RMUC2025.yaml；R:planner_manager/PCD/RMUC.pcd；R:rm_serial_driver_only_speed/README.md:L34-L67 |
| 许可证 | **无**：仓库根只有一个 `README.md` + `ACODO_ws.zip`（无 LICENSE/COPYING/.gitignore）；API `license = null`；五个自研包全部 `<license>TODO: License declaration</license>`。vendored 包各带自己的许可（fast_lio `LICENSE` 正文是 **GPL-2.0** 而 package.xml 写 `BSD`，linefit 是 BSD-3-Clause） | R:planner_manager/package.xml:L8；R:fast_lio/LICENSE:L1-L2；R:fast_lio/package.xml:L15；R:linefit_ground_segementation_ros2/LICENSE:L1 |
| ROS 版本 | **ROS 2 Humble / Ubuntu 22.04**（README 原文），全部 `ament_cmake` | R:README.md:L1-L2 |
| 仿真器 | **仓库内没有任何仿真包**；README 指向外部 Gazebo 仿真 `LihanChen2004/rmul24_gazebo_simulator`（该仓库已迁移为 `SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator`，Apache-2.0，2026-09-28 仍在更新） | R:README.md:L15-L16；<https://api.github.com/repos/LihanChen2004/rmul24_gazebo_simulator>（301→ 上述仓库） |
| 目标平台 | **全向/麦轮**（omni）底盘：MPC 控制量 = 车体系 `vx, vy` + `w`，状态 = `x,y,yaw`；`/cmd_vel` 用 `Twist.linear.x/linear.y/angular.z`；`CarType 0 = 圆形底盘`（半径 `Car_L 0.27 m`）；串口下行包是 `vx,vy,vz` | R:planner_manager/src/mpc.cpp:L109-L113；R:planner_manager/src/kino_replan_fsm.cpp:L346-L348；R:planner_manager/config/planner_manager.yaml:L16-L19 |
| 感知链 | Livox **MID-360**（`MID360_config.json`）→ `fast_lio`（另有 `point_lio`）→ `linefit_ground_segmentation`（去地面）→ `pointcloud_to_laserscan`（`/scan`）→ 2D ESDF | R:planner_manager/launch/plan.launch.py:L50-L130 |
| 定位链 | 外部里程计 `/Odometry`（默认参数）＋ 可选先验图 ICP 配准（`icp_registration`，底图 `RMUC.pcd`，`map_frame_id: world`） | R:planner_manager/config/planner_manager.yaml:L4；R:icp_registration/config/icp.yaml:L6-L9 |
| 体积 | `size = 162560 KB`（≈159 MB，含 50 MB `RMUL.pcd`、18 MB `RMUC.pcd`、52 MB gif、13 MB PDF）→ 克隆建议 `--depth 1` | <https://api.github.com/repos/865749/Kinodynamic_esdf_mpc> |

---

## 2. ESDF 部分（**是 2D，不是 3D**）

| 项 | 结论 | 出处 |
|---|---|---|
| 库/算法 | **自研/移植的 2D 增量 EDT**，**不是** voxblox / nvblox / FIESTA。`fillESDF` 是两遍（x 后 y）一维平方距离变换（Felzenszwalb–Huttenlocher），再开方乘分辨率 | R:plan_env/src/sdf_map.cpp:L352-L419 |
| 维度 | **2D 栅格**：全 `Eigen::Vector2d`，索引 `id(0)*num_y + id(1)`，入口叫 `updateESDF2d()`；无 z、无体素 | R:plan_env/include/plan_env/sdf_map.hpp:L37-L94,L164,L187-L190 |
| 符号距离 | 正距离 + `occupancy_buffer_neg` 负距离（障碍内部），合并成 `distance_buffer_all_`（内部为负）：`all = pos + (-neg + resolution)` | R:plan_env/src/sdf_map.cpp:L421-L473 |
| 输入 | `sensor_msgs/LaserScan`（参数 `sdf_map.laser_topic`，实配 `/scan`）；回调里 `laser_geometry::projectLaser` → tf2 变换到 `odom` → 按 `obstacles_inflation` 膨胀写占据 | R:planner_manager/config/planner_manager.yaml:L34；R:plan_env/src/sdf_map.cpp:L236-L313 |
| 分辨率/尺寸 | **`resolusion_ = 0.01 m`**（参数名拼错但代码就用这个键）、`map_size 30×30 m` → 3000×3000 = 9e6 格；局部更新窗口 **3.0×3.0 m** | R:planner_manager/config/planner_manager.yaml:L23-L25,L36-L37；R:plan_env/src/sdf_map.cpp:L42-L65 |
| 更新率 | **20 Hz**：`create_wall_timer(0.05s, updateESDFCallback)`（另有 20 Hz 可视化定时器）；只在 `esdf_need_update_` 时重算，且**只算局部窗口** `local_bound_min_..local_bound_max_` | R:plan_env/src/sdf_map.cpp:L61,L76,L331-L351,L393-L394 |
| 膨胀 | `obstacles_inflation = 0.0001 m`（实配）→ `inf_step = ceil(0.0001/0.01) = 1` 格 ≈ **1 cm**，等于几乎不膨胀（安全间隙靠 `Car_radius 0.27 m` 在碰撞检查里补） | R:planner_manager/config/planner_manager.yaml:L40；R:plan_env/src/sdf_map.cpp:L271,L292-L313 |
| 距离滤除 | 丢弃 `‖p − laser_pos‖ < 0.2 m` 的点（避免扫到自身） | R:plan_env/src/sdf_map.cpp:L286-L289 |
| 梯度用法 | `getSurroundPts` 取 4 邻格 → **双线性插值**，梯度解析给出并乘 `1/resolution`（`grad[1]=(v1-v0)/res`, `grad[0]=…/res`）；`evaluateEDTWithGrad` 被 B-spline 优化器的距离代价调用；`evaluateCoarseEDT`（最近格距离）被 kino A* 与 20 Hz 安全检查调用 | R:plan_env/src/edt_environment.cpp:L71-L114；R:bspline_opt/src/bspline_optimizer.cpp:L392；R:planner_manager/src/kino_replan_fsm.cpp:L26,L36 |
| 先验图模式 | 支持 `use_global_map` + `global_map_path`（ROS 地图 YAML+PGM，实配 `RMUC2025.yaml`）作为全局占据层，可多图切换 `setLocalMap(num)`；也发 `nav_msgs/OccupancyGrid` 到 `map_topic` | R:planner_manager/config/planner_manager.yaml:L21,L29-L32；R:plan_env/src/sdf_map.cpp:L9-L39,L139-L197；R:plan_env/include/plan_env/sdf_map.hpp:L125-L130 |
| **MPC 是否用 ESDF** | **不用**。ACADO 模型无在线数据（`ACADO_NOD 0`）、无 ESDF 参数；QP 变量数 `ACADO_QP_NV 90 = NU×N` 只有控制量 → 障碍/间隙完全不进 MPC | R:planner_manager/omni_model/acado_common.h:L67,L83 |
| 实测 ESDF 耗时 | 有打印代码（`show_esdf_time`），但**实配 false，仓库未给任何数值** | R:planner_manager/config/planner_manager.yaml:L35；R:plan_env/src/sdf_map.cpp:L346-L348 |

【推论】内存足迹：3000×3000 格 ×（1 char 占据 ×2 + 1 char neg + 4 个 double 缓冲）≈ **300 MB 常驻**；但每周期 EDT 只扫 601×601 的局部窗（3 m/0.01 m），20 Hz 在 28 核 i7-14700HX 上属轻载。

---

## 3. MPC 部分

**求解器**：ACADO Toolkit 导出的 **RTI（real-time iteration）** 求解器，内嵌 **qpOASES3** 解 QP；仓库直接链接**预编译静态库**（不装 ACADO 也能编：`find_package(ACADO)` 被注释掉）。

| 项 | 值 | 出处 |
|---|---|---|
| 求解器/框架 | ACADO 导出码 + qpOASES3（`ACADO_QP_SOLVER ACADO_QPOASES`、`ACADO_QPOASES3 1`）；静态库 `planner_manager/omni_model/libacado_exported_rti.a`（240 958 B） | R:planner_manager/omni_model/acado_common.h:L37-L48；R:planner_manager/CMakeLists.txt:L23-L32 |
| RTI 迭代 | `NUM_STEPS 10`（每次 solve 跑 10 次 feedback+preparation） | R:planner_manager/include/planner_manager/mpc.hpp:L21；R:planner_manager/src/mpc.cpp:L94-L99 |
| 状态维 | `ACADO_NX = 3`：`x, y, yaw` | R:planner_manager/omni_model/acado_common.h:L73 |
| 控制维 | `ACADO_NU = 3`：`vx, vy, w`（车体系速度 + 偏航角速度） | R:planner_manager/omni_model/acado_common.h:L71；R:planner_manager/src/mpc.cpp:L109-L113 |
| 时域 | `ACADO_N = 30`，`OCP(0.0, 3.0, 30)` → **Ts = 0.1 s，时域 3.0 s**（代码里 `#define Ts 0.1`） | R:planner_manager/omni_model/acado_common.h:L65；R:planner_manager/include/planner_manager/mpc.hpp:L24 |
| **动力学模型** | **一阶（速度级）全向运动学**，不是二阶动力学：`ẋ = vx·cosψ − vy·sinψ`，`ẏ = vx·sinψ + vy·cosψ`，`ψ̇ = w`。生成器原文见 `ACODO_ws.zip → ACODO_ws/src/acado_test/src/main.cpp`；FSM 里同一模型的离散递推 `update_states()` 也标了 `// based on kinematic model` | R:ACODO_ws.zip（内 `ACODO_ws/src/acado_test/src/main.cpp`）；R:planner_manager/src/kino_replan_fsm.cpp:L195-L209 |
| 积分/离散 | `MULTIPLE_SHOOTING` + `INT_RK45` + `NUM_INTEGRATOR_STEPS 30` + `GAUSS_NEWTON` + `FULL_CONDENSING` + `HOTSTART_QP YES` + `LEVENBERG_MARQUARDT 1e2` | 同上 main.cpp |
| 代价项 | `minimizeLSQ(W, rf)`，`rf = [x y ψ vx vy w]`（NY=6）；`minimizeLSQEndTerm(WN, rfN)`，`rfN = [x y ψ]`（NYN=3）→ **位姿+速度跟踪 + 终端位姿**。对角权重运行时可配：`weight_p 9.0`、`weight_yaw 5.0`、`weight_v 0.1`、`weight_w 0.1`（代码默认 10/10/0.3/0.3） | R:planner_manager/omni_model/acado_common.h:L79-L81；R:planner_manager/config/planner_manager.yaml:L78-L82；R:planner_manager/include/planner_manager/mpc.hpp:L42-L45；R:planner_manager/src/mpc.cpp:L9-L18 |
| **约束** | **只有控制箱约束（硬）**：`-0.5 ≤ vx ≤ 0.5`，`-0.5 ≤ vy ≤ 0.5`，`-0.1 ≤ w ≤ 0.1`。已烘焙进生成码：`acado_solver.c` 的 `acadoVariables.lbValues[...] = -5e-1 / -1e-1`。**无状态约束、无加速度/加加速度约束、无软约束/slack、无障碍项** | R:ACODO_ws.zip（main.cpp 原文）；R:planner_manager/omni_model/acado_solver.c:L1212-L1223 |
| 速度/加速度限制在哪 | 速度/加速度限制在 **B-spline 层**：`pos.setPhysicalLimits(max_vel, max_acc)` + `checkFeasibility` + 最多 3 次 `reallocateTime`；`max_vel = 1.5`、`max_acc = 0.8`、`max_jerk = 1.0`（`max_jerk_` 只被读取、**未见用于任何硬约束**，平滑性由 λ1 jerk 代价承担） | R:planner_manager/src/kino_replan_fsm.cpp:L100-L110；R:planner_manager/config/planner_manager.yaml:L6-L8,L55-L56；R:bspline_opt/include/bspline_opt/bspline_optimizer.hpp:L45 |
| 求解频率 | **控制/MPC 100 Hz**（`manager.control_cmd_frequency: 100` → `publish_control_cmd` 定时器）；安全检查 20 Hz；重规划检查 20 Hz | R:planner_manager/config/planner_manager.yaml:L12-L14；R:planner_manager/include/planner_manager/kino_replan_fsm.hpp:L81-L100 |
| 求解耗时 | **仓库未给数值**：计时语句存在但被注释（`// std::cout << "cost_time" …`）；只有搜索/优化耗时打印 | R:planner_manager/src/kino_replan_fsm.cpp:L83,L112,L335 |
| 参考轨迹来源 | 自己的 `position_traj_`（位置 B-spline）与 `yaw_traj_`：每 Ts=0.1 s 采 30 个点，输出 6 元组 `[x y ψ vx vy w]` 共 180 个参考量 | R:planner_manager/src/kino_replan_fsm.cpp:L259-L324 |
| 热启动 | 用**同一运动学模型**把上一周期控制序列前推 N+1 步，作为 ACADO 状态初值（`motion_prediction`） | R:planner_manager/src/kino_replan_fsm.cpp:L210-L245 |

【事实·矛盾点】**规划 1.5 m/s，MPC 上限 0.5 m/s**：`search.max_vel / optimization.max_vel = 1.5`，而生成码把控制硬限在 ±0.5（合速度上限 0.707 m/s）。⇒ 期望速度永远进不了可行域，MPC 必然长期饱和在约束边界。

【事实·隐患】`motion_prediction()` 对只有 3 个元素的 state 访问 `cur_state[3]`（偏航溢出归一化），是**越界读**；`calculate_ref_states()` 是死代码且有同样问题。 | R:planner_manager/src/kino_replan_fsm.cpp:L178-L194,L224-L231

---

## 4. 实现事实（节点 / 话题 / 配置 / 依赖 / 分工）

**入口**：单节点组件 `planner_manager_node`（插件类 `fast_planner::kino_replan_fsm`，节点名 `planner_manager`）。 | R:planner_manager/CMakeLists.txt:L35-L38；R:planner_manager/include/planner_manager/kino_replan_fsm.hpp:L44；R:planner_manager/src/kino_replan_fsm.cpp:L488

| 话题/帧 | 名称 | 出处 |
|---|---|---|
| 订阅 | `/Odometry`（`nav_msgs/Odometry`）、`/goal_pose`（`geometry_msgs/PoseStamped`） | R:planner_manager/config/planner_manager.yaml:L4-L5；R:.../kino_replan_fsm.hpp:L56-L62 |
| 发布 | `/cmd_vel`（`geometry_msgs/Twist`，`linear.x`/`linear.y`/`angular.z`）、`robot_path`、`predict_path`（`nav_msgs/Path`）、`visited_node`（`PointCloud2`）、`map_topic`（`OccupancyGrid`） | R:.../kino_replan_fsm.hpp:L84-L90；R:planner_manager/src/kino_replan_fsm.cpp:L346-L355；R:plan_env/src/sdf_map.cpp:L6 |
| 帧 | `sdf_map.frame_id = /odom`；path 的 `frame_id = "/odom"`（**带前导斜杠**，不符合 ROS 2 惯例） | R:planner_manager/config/planner_manager.yaml:L39；R:planner_manager/src/kino_replan_fsm.cpp:L135,L359 |

**配置实值**（[planner_manager.yaml](https://github.com/865749/Kinodynamic_esdf_mpc/blob/master/planner_manager/config/planner_manager.yaml)）：
`max_vel_ 1.5`、`max_acc_ 0.8`、`max_jerk_ 1.0`、`dynamic_environment 0`、`control_points_distance 0.5`、`control_cmd_frequency 100`、`safety_cheak_frequency 20`、`plan_cheak_frequency 20`、`CarType 0`、`Car_L 0.27`、`Car_W 0.40`（L6-L19）；
`search: max_tau 0.3 / init_max_tau 0.4 / lambda_heu 5.0 / w_time 5.0 / horizon 100.0 / resolution_astar 0.01 / time_resolution 0.8 / allocate_num 100000 / check_num 30`（L41-L52）；
`optimization: lambda1 20 / lambda2 8 / lambda3 1e-5 / lambda4 0.05 / lambda7 100 / dist0 0.6 / max_iteration_num2..4 300/200/200 / algorithm1 11 / algorithm2 15 / order 3`（L54-L76）；`mpc: weight_p 9.0 / weight_yaw 5.0 / weight_v 0.1 / weight_w 0.1`（L78-L82）。
【事实】代码读的是 `manager.plan_cheak`（R:.../kino_replan_fsm.hpp:L97），配置里写的是 `manager.plan_cheak_frequency`（L14）→ **键名不匹配**，实际取默认值 20（恰好同值）。
【推论】`algorithm1/2 = 11/15` 对应 NLopt `LD_LBFGS` / `LD_TNEWTON`（NLopt `nlopt_algorithm` 枚举次序），即"二次代价用 L-BFGS、一般代价用 TNewton"，与 `nlopt::opt(nlopt::algorithm(...), …)` 用法一致（R:bspline_opt/src/bspline_optimizer.cpp:L176）。

**规划–控制分工**（同一个 FSM 内串起来，无 Nav2 参与）：
1. `plan()`：**kinodynamic A***（`path_searching`，状态 4 维 `(x,y,vx,vy)`、控制 = 加速度 2 维、ESDF 碰撞、四阶多项式时间启发式，`max_vel 1.5 / max_acc 0.8 / w_time 5 / horizon 100 m`） | R:path_searching/include/path_searching/kinodynamic_astar.hpp:L26-L29,L143-L171；R:path_searching/src/kinodynamic_astar.cpp:L36-L44,L159-L243,L341-L412
2. 采样 → 3 次 B-spline 控制点（`ts = ctrl_pt_dist/max_vel = 0.333 s`）→ **NLopt** 优化 `NORMAL_PHASE = SMOOTHNESS|DISTANCE|FEASIBILITY`（λ1/λ2/λ3，距离项用 `evaluateEDTWithGrad`） | R:planner_manager/src/kino_replan_fsm.cpp:L84-L99；R:bspline_opt/src/bspline_optimizer.cpp:L13-L15,L290-L315,L388-L392
3. `planYaw()`：第二次 B-spline 只优化航向（`SMOOTHNESS|WAYPOINTS`，`dt_yaw≈0.3 s`，前视 2.0 s） | R:planner_manager/src/kino_replan_fsm.cpp:L406-L484
4. 100 Hz `publish_control_cmd()`：采参考 → ACADO RTI MPC → 发 `/cmd_vel` 第一拍 | R:planner_manager/src/kino_replan_fsm.cpp:L246-L375
5. 20 Hz 安全检查：目标净空 <0.3 m 时按 `dr=0.27 m / dθ=30°` 螺旋找更空的目标点（**自动挪目标**）；沿 B-spline 以 0.02 s 步长查碰撞（半径 3 m 内） | R:.../kino_replan_fsm.hpp:L243-L327；R:planner_manager/src/kino_replan_fsm.cpp:L4-L49
6. 20 Hz 重规划：跟踪误差 > 1.0 m 或轨迹碰撞 → `GEN_NEW_TRAJ`/`REPLAN_TRAJ`（FSM：`INIT→WAIT_TARGET→GEN_NEW_TRAJ→EXEC_TRAJ`） | R:.../kino_replan_fsm.hpp:L344-L500

**依赖清单**：

| 包 | 依赖（原文） | 出处 |
|---|---|---|
| `plan_env` | rclcpp, visualization_msgs, std_msgs, **PCL**, eigen, message_filters, nav_msgs, pcl_conversions, sensor_msgs, geometry_msgs, laser_geometry, tf2_geometry_msgs, **OpenCV**, tf2_ros, tf2_sensor_msgs | R:plan_env/package.xml:L12-L26 |
| `path_searching` | plan_env, eigen, boost, rclcpp, laser_geometry, sensor_msgs, nav_msgs | R:path_searching/package.xml:L12-L18 |
| `bspline_opt` | plan_env, **NLopt**, eigen, bspline（`find_package(NLopt REQUIRED)`） | R:bspline_opt/package.xml:L12-L17；R:bspline_opt/CMakeLists.txt:L9 |
| `planner_manager` | rclcpp, rclcpp_components, std_msgs, geometry_msgs, plan_env, path_searching, bspline, bspline_opt, **cppad**（声明了但代码不用）, tf2_ros, tf2_geometry_msgs, **rmoss_interfaces**, laser_geometry + 预编译 `libacado_exported_rti.a` | R:planner_manager/package.xml:L12-L25；R:planner_manager/CMakeLists.txt:L30 |
| README 要求的环境 | ACADOtoolkit、nlopt、glog、**osqp + osqp-eigen**、`pip install xmacro`、`rosdep install -r --from-paths src` | R:README.md:L3-L13 |

【事实】`rmoss_interfaces` 在 git tree 里是 **submodule gitlink（`type: commit`），而仓库没有 `.gitmodules`**；`kino_replan_fsm.hpp:563` 真的用了 `rmoss_interfaces::msg::GimbalCmd`，`package.xml:23` 也依赖它 → **直接 clone 编不过**。
【事实】硬编码绝对路径：`mpc.hpp` 用 `../../../src/planner_manager/omni_model/...` 相对包含（包必须放在 `<ws>/src/planner_manager`）；配置/launch 里散落 `/home/z/Ouuu_fast_planner_2D/...`、`/home/livox/livox_test.lvx`。 | R:planner_manager/include/planner_manager/mpc.hpp:L8-L9；R:planner_manager/config/planner_manager.yaml:L31；R:planner_manager/launch/plan.launch.py:L13,L44,L46
【事实】仓库里也带 `.o` 目标文件、`ACODO_ws.zip`（2.2 MB，含 2024-12~2025-01 的 colcon 构建日志、`getting_started_export/` 与 `omni_model1.zip`）；`planner_manager/CMakeLists.txt:18` 还 include 了仓库里不存在的 `getting_started_export`。
【事实】噪声/死代码：`minDistToAllBox` 每次调用 `std::cout << "asd"`；动态障碍预测存在但 `dynamic_environment: 0` 关闭，`obj_predictor.cpp` 仅 874 B。 | R:plan_env/src/edt_environment.cpp:L44,L49；R:planner_manager/config/planner_manager.yaml:L9；R:plan_env/src/obj_predictor.cpp

---

## 5. 是否有配套论文 / 博客 / 视频 / 学位论文

【事实】**没找到**。依据：① 仓库根无 `docs/`、无 arXiv/DOI 链接、无 citation 文件（tree 548 项逐项看过，顶层只有 `README.md` + `ACODO_ws.zip` + 14 个包目录）；② README 全文 580 B，只写环境、"需要装 ACADOtoolkit/nlopt/glog/osqp"、仿真链接；③ `web_search`（用户 `865749` / 仓库名 / "kinodynamic ESDF MPC"）只返回第三方聚合页 <https://relatedrepos.com/gh/865749/Kinodynamic_esdf_mpc> 与不相关的 RM/无人机论文，**没有作者本人的论文/博客/视频/Thesis**。
【推论】因此**没有任何可引用的性能数字**（求解耗时、最高速度、障碍密度）：这既是"无据可依"，也意味着不存在"论文声称 vs 代码实现"的落差问题——一切都以代码为准。

---

## 6. 它在"导航策略设计轴"上的位置（对齐我们自己的分类）

轴名沿用本工作区 [nav_strategy_taxonomy.md](04-issues.md)（9 轴）与 [algorithm_axes.md](01-architecture.md)（6 轴）：

| 轴 | 本仓库所属族 | 依据 |
|---|---|---|
| **轴 1 世界表示** | **1.4 ESDF/TSDF —— 但只是 2D 版本**（局部增量 EDT），叠加 **1.1 2D 占据栅格**（可选先验 PGM，`RMUC2025.yaml`）与 **1.12 先验 PCD 子图 + ICP**（`RMUC.pcd`）。**完全没有 3D 体素/octomap/voxblox** | [nav_strategy_taxonomy.md:83](04-issues.md)；R:planner_manager/config/planner_manager.yaml:L29-L32；R:icp_registration/config/icp.yaml:L6 |
| **轴 2 定位** | 不自己做：吃外部 `nav_msgs/Odometry`（实车=d；仿真）；可选 `icp_registration` 先验图 ICP 重定位（`map_frame_id: world` / `odom_frame_id: odom`），仓库内 vendored FAST-LIO / Point-LIO | R:planner_manager/config/planner_manager.yaml:L4；R:icp_registration/config/icp.yaml:L7-L8 |
| **轴 3 全局规划** | **3.2 的血亲但更"动力学"**：**kinodynamic A\***（连续状态 + 数值积分 + ESDF 碰撞 + 多项式时间启发式），本仓库地图里没有"精确对应族"——建议在轴 3 补一族 **"kinodynamic search / 动力学约束图搜索"**（介于 3.2 state lattice 与 3.10 MPC 式全局规划之间）；其后接 **3.5 轨迹优化**（B-spline + NLopt） | [nav_strategy_taxonomy.md:202](04-issues.md)（3.2）、[:220](04-issues.md)（3.5）、[:250](04-issues.md)（3.10）；R:path_searching/src/kinodynamic_astar.cpp:L49-L243 |
| **轴 4 局部控制** | **4.7 MPC**（ACADO RTI + qpOASES，梯度式、显式约束、热启动）**＋** 4.5 式"局部轨迹优化"（NLopt B-spline 避障/平滑，**避障在这里**）；运动学属 **4.10 全向/麦轮（Omni）** | [nav_strategy_taxonomy.md:294](04-issues.md)（4.7）、R:planner_manager/src/mpc.cpp:L61-L117；R:bspline_opt/src/bspline_optimizer.cpp:L290-L315 |
| **轴 5 任务指定** | **5.x 单目标位姿**：收 `/goal_pose`，无巡逻/覆盖/多点排序；目标被占时**自动螺旋外挪目标**（0.27 m 步长 / 30°） | R:.../kino_replan_fsm.hpp:L254-L297 |
| **轴 6 不确定性与安全** | 无不确定度建模、无 CBF/可达集/概率保证；安全 = **几何间隙（`Car_radius 0.27`）＋ 20 Hz 碰撞检查 ＋ 重规划 ＋ 目标外挪**（"检查-重规划兜底"族） | R:planner_manager/src/kino_replan_fsm.cpp:L4-L49；R:.../kino_replan_fsm.hpp:L243-L327 |
| **轴 7 学习** | **无**任何学习成分（tree 里无 torch/tf/onnx，无训练脚本） | 548 项文件清单（<https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/git/trees/master?recursive=1>） |
| **维度轴 D（2D→2.5D→3D）** | **明确 = 2D**，甚至比我们"更平"：3D 点云先用**高度带**（`min_height -1.0`／`max_height 0.5`）压成 LaserScan，再建 2D ESDF。**没有任何 2.5D 可通行性/高程层** | [algorithm_axes.md:99](01-architecture.md)；R:planner_manager/launch/plan.launch.py:L94-L101 |

---

## 7. 与 MPPI 的本质差异（本仓库 MPC vs 我们 bench 的 Nav2 MPPI）

我们的 MPPI 实配（[nav2_params_sim_mppi.yaml](../src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_mppi.yaml)）：`controller_frequency 30 Hz`[:107]、`motion_model "Omni"`[:143]、`time_steps 60`[:145]、`batch_size 1500`[:151]、6 个 critic[:170-171]、局部代价图 `resolution 0.02`[:235]、`robot_radius 0.22`[:248]、`inflation_radius 0.5`[:305]、STVL `voxel_size 0.05`[:362]。

| 维度 | 本仓库 MPC（梯度/RTI-QP） | 我们 MPPI（采样） | 谁更强 |
|---|---|---|---|
| 优化机制 | 显式 OCP：`minimizeLSQ` + Gauss-Newton，10 次 RTI 迭代，qpOASES 热启动 QP | 1500×60 = **9e4 采样/周期 @30 Hz**，代价指数加权，无梯度 | 各有：MPC 解**平滑、可预测**；MPPI 免梯度、能吞非凸/不可微代价 |
| 动力学模型 | 显式一阶全向（`ẋ=vx cosψ−vy sinψ`…），状态 3 维、控制 3 维，Ts 0.1 s × 30 步 = **3.0 s 时域** | Nav2 内置 Omni 运动学（同样一阶全向），时域 = 60×(1/30) = **2.0 s** | 平手（同一族运动学）；本仓库可显式约束、MPPI 不能 |
| 速度/加速度约束 | **硬约束**但只有控制箱 ±0.5 / ±0.5 / ±0.1 rad/s（**且比规划器 1.5 m/s 小 3 倍**）；加速度限制在 B-spline 层用可行性代价软处理 | 无硬约束：`ConstraintCritic`/`ObstaclesCritic` 是**软惩罚**，`vx_max` 等只在参数里"约定" | **MPC（在它自己的包络内）**；但本仓库包络被写死，实车速度上限反而低 |
| 避障 | **MPC 内部没有障碍项**（`ACADO_NOD=0`，无 ESDF 参数）；避障靠 **20 Hz 重规划 + B-spline 距离代价 + ESDF**，即"控制器外闭环" | `CostCritic`/`ObstaclesCritic` **直接用 costmap（含 inflation/STVL 体素）给采样轨迹打分**，即"控制器内闭环" | **MPPI**：对突发障碍/动态障碍的响应在控制周期内；本仓库要等一次重规划（≤50 ms 决策 + B-spline 求解） |
| 精度/平滑 | 连续量输出，横移+自旋解耦显式建模；跟踪误差可做到 cm 级（无数据，[推论]） | 受 `batch_size`/噪声 σ 限制，控制有采样抖动；靠 `Savitzky-Golay`/`velocity_smoother` 收拾 | **MPC** |
| 调参成本 | 4 个权重（YAML 可调）＋ **模型/约束改动需重新生成 ACADO 代码**（要 MATLAB+ACADO 或 ACADO C++ 工具链） | 全在 YAML：critics 权重、`batch_size`、`time_steps`、噪声参数（我们已有注释体系） | **MPPI** |
| 依赖门槛 | ACADO 预编译 `.a` + qpOASES + **NLopt**（缺失）＋ `rmoss_interfaces`（缺失）＋ 代码生成链 | **零额外依赖**（`nav2_mppi_controller` 1.1.20 已装） | **MPPI** |
| CPU/GPU | 90 个 QP 变量 × 10 次 RTI，纯 CPU、微秒~亚毫秒级【推论】；无 GPU 依赖 | 9e4 轨迹点/周期纯 CPU（我们注释里已记为"sim 里开销大"） | **MPC** |
| 确定性 | 热启动 QP，同输入同输出（迭代上限/浮点误差外） | 噪声发生器驱动（可 seed），每周期轨迹不同 → **近似随机** | **MPC** |
| 碰撞保证 | **都不给保证**：MPC 只保证控制箱约束，因为障碍根本不在 OCP 里；想有保证必须把 ESDF 线性化成状态约束 + slack（**本仓库没做**） | 无任何硬保证（critic 软惩罚） | 都不行；这是"真·ESDF-MPC"才可能补的洞 |

【推论·最重要的一条】本仓库的 MPC **不具备**"梯度式 MPC"的招牌卖点（把障碍当约束/可微代价进优化）。它是**纯跟踪器**：亮点在"显式约束 + 高频率 + 平滑 + 低算力"，避障与 MPPI 一样靠外环。所以用它替换 MPPI，**避障只会变差，不会变好**；要拿到"ESDF-MPC"的真正价值，必须自己把 ESDF 距离/梯度写进 OCP（项 8.3 的工作量来源）。

---

## 8. 引入我们 bench 需要什么

### 8.1 依赖：装什么 / 本机已有什么

| 依赖 | 本机状态（2026-10-05 实测） | 结论 |
|---|---|---|
| ROS 2 Humble | ✅ `/opt/ros/humble`，`ROS_DISTRO=humble` | 满足 |
| Nav2（对照用） | ✅ `nav2_mppi_controller` / `nav2_controller` / `nav2_costmap_2d` / `nav2_smac_planner` / `nav2_navfn_planner` 均 **1.1.20** | 满足 |
| STVL | ✅ `/opt/ros/humble/share/spatio_temporal_voxel_layer` | 满足 |
| `pointcloud_to_laserscan`、`linefit`、`fast_lio`、`point_lio`、`livox_ros_driver2` | ✅ 源码在 `src/perception/rm_perception/...`、`third_party/`，且 `install/` 里已构建 | 满足 |
| `icp_registration` | ✅ **我们已有**：[src/localization/rm_localization/icp_registration](../src/localization/rm_localization/icp_registration)（本仓库同款包） | 满足 |
| **NLopt** | ❌ 无 `/usr/include/nlopt*`，`dpkg -l \| grep nlopt` 空 → `sudo apt install libnlopt-dev`（`bspline_opt/CMakeLists.txt:9` 是 `REQUIRED`） | **必装** |
| **ACADO Toolkit** | ❌ 无 `/usr/local/include/acado*`、无 apt 包 —— **但构建不需要**（`find_package(ACADO)` 已注释，只链预编译 `.a`）；**只有"要改模型/约束/N"时才需要**（MATLAB+ACADO 或 ACADO C++ 工具链） | 视目标而定 |
| **qpOASES** | ❌ 独立库没装，**但源码随包内嵌**（`omni_model/qpoases/SRC|INCLUDE`） | 无需另装 |
| glog | ✅ `libgoogle-glog-dev 0.5.0` | 满足 |
| Eigen3 / PCL / OpenCV | ✅ `libeigen3-dev 3.4.0`、`libpcl-dev 1.12`、`libopencv-dev 4.5.4`（`/usr/include/opencv4`） | 满足 |
| OSQP / osqp-eigen / `xmacro` | ❌ 都没有（`python3 -c "import osqp/casadi/xmacro"` 全失败）——【推论】**当前 master 并不需要**：tree 里无 osqp 源/CMake 引用，`planner_manager` 只链 ACADO 静态库；README 那几条属遗留说明 | 可跳过（先试） |
| cppAD | ❌ 未装，但 `planner_manager/package.xml:L20` 声明 `<depend>cppad</depend>` → `rosdep install` 会报缺（README 自己写了"cppAD 已废除"） | 建议删依赖或补装 |
| **rmoss_interfaces** | ❌ 仓库里是空 submodule 且无 `.gitmodules`；但我们自己的 RM 栈里通常有同类 msg → 需自备或把 `kino_replan_fsm.hpp:563` 的 `GimbalCmd` 发布者摘掉 | **必补** |

### 8.2 缺的"部件"（逐条核对）

1. **"从 Livox 点云建 ESDF" —— 其实不缺**：`plan_env` 直接吃 `LaserScan`（`laser_topic`），而我们的 `/scan` 正是 `Livox → linefit(/segmentation/obstacle) → pointcloud_to_laserscan` 同一条链（[bringup_sim.launch.py:37](../src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py#L37)、[:264-275](../src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py#L264)、[laserscan_params.yaml](../src/perception/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml)）。差别只在参数：我们 `max_height 0.1` / `range_min 0.12`，仓库 `max_height 0.5` / `range_min 0.0` / `range_max 10.0`（R:planner_manager/launch/plan.launch.py:L94-L101）。
2. **2D→3D / 2.5D 表示 —— 不需要**（见 8.4）。
3. **麦轮运动学模型 —— 已经有等价物**：`omni_model` 的模型就是全向一阶模型（`ẋ=vx cosψ−vy sinψ`…），与 Nav2 MPPI 的 `Omni` 同族。要改的是**速度包络**：把它从 ±0.5 / ±0.1 换成我们的（参照 [nav2_params_sim_mppi.yaml](../src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_mppi.yaml#L141) 附近的 `vx_max/w_max`），**而这需要重新生成 ACADO 代码**（`lbValues/ubValues` 在 `acado_initializeSolver` 里写死，手改生成码属于 hack）。
4. **求解器构建**：`libacado_exported_rti.a` 是 x86-64 预编译产物；本机是 x86-64（i7-14700HX，28 线程）→ 可用，但要注意 ABI/`-lrt` 链接与 GCC 版本。【推论】若不想碰 ACADO：这个 OCP 极小（3 状态 × 3 控制 × 30 步 + 仅箱约束），用 **CasADi+OSQP / acados / 手写 iLQR-DDP** 重写约 200–400 行，反而更可控（也顺便解决"包络写死"与"加 ESDF 项"两个问题）。
5. **集成契约**：它发 `/cmd_vel` 的 `Twist`（含 `linear.y`、`angular.z`）。注意我们踩过的坑——`fake_vel_transform` 会在小陀螺模式下替换角速度（见 [algorithm_axes.md:129-131](01-architecture.md)）⇒ 该节点必须挂在 `fake_vel_transform` **之前**（或车上关 spin），否则行为不可预期。

### 8.3 工作量估计【推论，非实测】

| 目标 | 内容 | 估时 |
|---|---|---|
| A. 先跑通（验证它到底能做什么） | 装 `libnlopt-dev`；补 `rmoss_interfaces`；改 `mpc.hpp` 的 `../../../src/planner_manager/...` 包含路径；清掉配置/launch 里的 `/home/z/...`；只启 `planner_manager`（不启它的 FAST-LIO/串口/驱动），把我们的 FAST-LIO odom remap 成 `/Odometry`、`/scan` 直连、`/cmd_vel` 接 Nav2 之前的通道；用 `/goal_pose` 手动给点 | **0.5–1 人日** |
| B. 换成我们的速度包络 | 方案 B1：装 ACADO 工具链，改 `main.cpp` 的 `subjectTo` 后重新 export（推荐）；方案 B2：改用 acados/CasADi 重写同一 OCP | **1–2 人日** |
| C. 做"真·ESDF-MPC"（避障进 OCP） | 在 OCP 里加障碍项：取 ESDF 距离/梯度线性化为状态约束（+ slack 软约束），或直接用 `dist(x,y)` 的可微代价；调参 + 仿真回归 | **+3–8 人日** |
| D. 接成 Nav2 插件形态 | 见 §9 形态 2（需自建"路径→参考状态序列 + yaw 规划 + 避障层"） | **3–6 人日** |

### 8.4 2D/2.5D 简化够不够？—— **够，而且它就是本仓库的答案**

【事实】本仓库本身已经是 **2D ESDF + 高度带压缩**的打法，并在 RM 赛场用过（地图/PCD 命名 RMUC/RMUL）。
【推论】对**平坦的 RM 场地**：
- 全 3D ESDF（voxblox/nvblox）的边际收益 ≈ 0：障碍贴地、雷达装在 ~0.3 m 高、机器人是 2D 运动学；3D 只会把"可通行"判据变复杂（还要处理悬空结构/点云稀疏）。
- **2.5D 只在"高地/斜坡/台阶"（RMUC 高地）才有意义**；而我们现有的 `pointcloud_to_laserscan` 高度带（`min_height -1.0 / max_height 0.1`）**已经把高地当障碍**，等价于"把不可通行的 3D 结构投影成 2D 障碍"——这正是最省事的 2.5D。
- 所以正确路线是：**用现有 2D 表示做距离场/梯度**（`/scan`→EDT，或直接在 costmap 上算 EDT），把距离场喂给 MPC 的障碍项；**不要**引入 3D ESDF。
- 距离场来源三选一：① 直接移植 `plan_env`（**但见 §10 许可风险**，且它自带 30 m×30 m / 0.01 m 的大缓冲）；② 在 Nav2 costmap 的 master grid 上自写 ~50 行 Felzenszwalb EDT（避开许可问题，复用它 20 Hz 局部更新的思路）；③ 抄其"局部窗口 + 双缓冲"的工程技巧（只算 3 m 窗口，R:plan_env/src/sdf_map.cpp:L393-L394）。

### 8.5 主要风险

1. **许可**：【事实】本仓库 **无 LICENSE**，自研包 `<license>TODO</license>`；血统指向 **GPL-3.0** 的 Fast-Planner 系（`HKUST-Aerial-Robotics/Fast-Planner` = GPL-3.0，`ZJU-FAST-Lab/ego-planner` = GPL-3.0，均已用 GitHub API 核对）。⇒ 直接搬 `plan_env`/`bspline_opt` 代码进我们的仓库有**传染性许可与"未授权"双重风险**，引入前必须做来源与许可确认。
2. **包络写死**：不重新生成代码就只能用 0.5 m/s（对我们 1.5+ m/s 的哨兵基本不可用）。 | R:planner_manager/omni_model/acado_solver.c:L1212-L1223
3. **子模块缺失 + 硬编码绝对路径 + 仓库内提交 `.o`/zip**：可移植性差，`colcon build` 大概率要手工改 3~5 处。 | R:planner_manager/package.xml:L23；R:planner_manager/config/planner_manager.yaml:L31
4. **避障能力倒退**：只搬 MPC ⇒ 没有 costmap/STVL 语义（禁行区、膨胀、动态障碍），避障弱于现有 MPPI（见 §7）。
5. **契约冲突**：`/cmd_vel` 全向速度 + `fake_vel_transform` 小陀螺替换角速度；且它期望 20~100 Hz 的 `odom` 与 `/scan`（我们的 p2l 是 10 Hz 附近）→ 频率/时延契约要对齐。
6. **代码质量**：越界索引（`cur_state[3]`）、循环里打印 `"asd"`、参数名拼写（`resolusion_`）、配置键不匹配（`plan_cheak_frequency`）——搬过来会带进噪声与隐 bug。
7. **CPU/内存**：0.01 m × 30 m 的 2D 网格 ≈ 300 MB 缓冲；若要 20 Hz 全场 EDT（而不是 3 m 局部窗）会明显吃 CPU。

---

## 9. 与 Nav2 的集成方式（能不能只换 local controller？）

【事实】它**不是** `nav2_core::Controller` 插件：节点继承 `rclcpp::Node`（R:.../kino_replan_fsm.hpp:L41），用 `rclcpp_components_register_node` 注册（R:planner_manager/CMakeLists.txt:L35-L38），package.xml 里**没有 `nav2_core`/`pluginlib`**（R:planner_manager/package.xml:L12-L25）。
【事实】它是一个**自带全局规划 + 轨迹优化 + 偏航规划 + 跟踪 + 安全检查的独立 FSM 节点**：输入 `/Odometry` + `/goal_pose`，输出 `/cmd_vel`（R:.../kino_replan_fsm.hpp:L56-L90；R:planner_manager/src/kino_replan_fsm.cpp:L346-L355）。**它不消费 `nav_msgs/Path`，也不读 costmap。**

| 集成形态 | 可行性 | 代价/风险 | 估时 |
|---|---|---|---|
| **① 独立节点（绕过 Nav2）** | ✅ 最省事：BT/脚本里把目标转成 `/goal_pose` 发给它，让它自己规划+控制 | 失去 Nav2 的 costmap 层（静态图/STVL/禁行区）、行为树、恢复行为、`velocity_smoother`；两套规划器并存易冲突 | 0.5–1 人日 |
| **② 包成 `nav2_core::Controller` 插件（只换局部控制器）** | ⚠️ 接口上可以：`computeVelocityCommands()` 能拿到 `nav_msgs/Path`（全局路径）+ 当前位姿 + 当前速度 | **必须自己补三样**：(a) 路径→参考状态序列（它现在从自己的 B-spline+yaw 规划采样 180 个参考量，R:planner_manager/src/kino_replan_fsm.cpp:L259-L324）；(b) 偏航参考生成（`planYaw` 用的是自己的位置 B-spline）；(c) **避障**（MPC 无障碍项）→ 实际等于把 2D ESDF + B-spline 优化器一起搬进来 ≈ 重写一个 `mpc_local_planner` | 3–6 人日 |
| **③ 只借 `Mpc` 类 + ACADO 求解器** | ✅ 可行：`Mpc`（`mpc.hpp`/`mpc.cpp`）只依赖 Eigen + ACADO 头（R:planner_manager/include/planner_manager/mpc.hpp:L6-L9），把参考轨迹从外部喂进来即可 | 需自备参考生成器与避障；包络写死问题依旧；适合做"MPPI vs 梯度 MPC 跟踪同一参考"的 A/B | 2–4 人日 |

【推论】它**不需要**自己的全局规划器才能跑（形态 ① 自带、形态 ②/③ 不需要），但**需要**"参考位姿+偏航+速度"三件套；Nav2 只给 (x,y,θ?) 路径 ⇒ 形态 ② 的主要工作就是补这个参考层 + 避障层。
【对照】Nav2 官方本来就没有 MPC 族（我们自己的分类也这么写：[nav_strategy_taxonomy.md:297](04-issues.md)）⇒ 引入它等于"补一个族"，但生态成本高；若只想在 `nav` 槽位里体验"有显式约束的控制器"，**先做形态 ③ 的 A/B 最划算**（同一参考轨迹、同一场景，比较跟踪误差/平滑度/CPU）。

---

## 10. 无法确定的（明确列出）

1. **所有性能数字缺失**：求解耗时、MPC 实际频率是否稳定在 100 Hz、最高速度、能处理的障碍密度、成功率——仓库没有任何 bag/图表/表格，计时打印被注释（R:planner_manager/src/kino_replan_fsm.cpp:L335）。§7/§8 里带"【推论】"的 CPU/精度判断均为量级估算。
2. **实车机器人型号**（哨兵？步兵？）、底盘到底是麦轮还是全向轮、质量/惯量/URDF：仓库**不含 description/URDF**，只能由 `vx,vy,w` 控制与 `CarType 0` 圆盘（半径 0.27 m）推断"具全向运动能力"。
3. **是否真跑过 RMUC2025 并取得成绩**：`RMUC2025.pgm`/`RMUC.pcd` 只能说明用过这些场地资产。
4. **`omni_model` 生成码与 `ACODO_ws.zip` 里 `main.cpp` 是否同一次生成**：【推论】极可能一致（`acado_common.h` 8908 B、`acado_solver.c` 105 375 B、`libacado_exported_rti.a` 240 958 B **字节数完全相同**，且 `lbValues` 的 0.5/0.5/0.1 与 `main.cpp` 的 `subjectTo` 一致），但**无法证明**（zip 里没有模型哈希/时间戳对应关系）。
5. **在我们机器上能否 `colcon build` 成功**：**未实测**（本任务为只读调研；本机缺 NLopt、缺 `rmoss_interfaces`、缺 ACADO）。另外 `rmoss_interfaces` 是空 submodule 且无 `.gitmodules`，能否找到匹配版本同样未验证。
6. **`dynamic_environment: 0` 之外的动态障碍路径是否可用**：`obj_predictor.cpp` 仅 874 B、`minDistToAllBox` 循环打印 `"asd"`，【推论】该路径基本是死代码/半成品，但未逐一读完（`obj_predictor.hpp` 未展开分析）。
7. **许可归属**：无 LICENSE、无作者声明、上游血统只能"由命名与算法推断"；**这不是法律意见**，任何代码复用前需自行确认（Fast-Planner 系 = GPL-3.0 已核对：<https://api.github.com/search/repositories?q=Fast-Planner+in:name>）。
8. **MPPI 与本仓库 MPC 的直接对比实验**：两边场地/底盘/传感链虽高度相似（都是 Livox→linefit→p2l→2D、都是全向底盘），但**从未在同一场景下比过**——本节所有"谁更强"均为机制层面的推论，不是实测。

---

### 附：最小可复现的证据索引

- 仓库全景：<https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/git/trees/master?recursive=1>（548 项，未截断）
- 提交史：<https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/commits?per_page=30>（13 条，最早 `Ouuu` 2025-01-12）
- **ACADO 模型定义原文**：`ACODO_ws.zip` → `ACODO_ws/src/acado_test/src/main.cpp`（<https://github.com/865749/Kinodynamic_esdf_mpc/blob/master/ACODO_ws.zip>）
- 生成码维度/约束：R:planner_manager/omni_model/acado_common.h:L65-L89；R:planner_manager/omni_model/acado_solver.c:L1212-L1223
- 2D EDT：R:plan_env/src/sdf_map.cpp:L352-L473；梯度：R:plan_env/src/edt_environment.cpp:L71-L114
- 规划–控制流：R:planner_manager/src/kino_replan_fsm.cpp:L50-L164（规划）、L246-L375（控制）；R:planner_manager/include/planner_manager/kino_replan_fsm.hpp:L344-L500（FSM）
- 本机对照：`/opt/ros/humble/share/{nav2_mppi_controller,spatio_temporal_voxel_layer}`；[nav2_params_sim_mppi.yaml](../src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_mppi.yaml#L141)；[bringup_sim.launch.py](../src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py#L37)


---


## 来源 9/19：`nav/nav2_vs_custom_planner.md`


> **本文回答的问题**（用户原话）：「这个自己研制怎么自己研制呢，因为这个纯 c++吧，我们这个又算强依赖 nav2 吧，做成 nav2 的插件吗，还是有办法保持纯 c++ 的形态，如果要做成 nav2 的插件或者融入 nav2 会不会影响原来纯 c++ 的运行效率呢」。
>
> **被评估的库**：`T-DT-Algorithm-2026/tdt-nav-kit`（RM 哨兵导航前后端：`YAstar` = A\*/Kinodynamic A\*，`MinimumSnapOsqp` = min-snap + 方形走廊（SFC）轨迹优化）。纯 CMake、**无 ROS、无 `package.xml`**，依赖 Eigen3 + OSQP/OsqpEigen，OpenCV 只在 `main.cpp` 示例里用。
> **我们的栈**：ROS 2 Humble + nav2 **1.1.20**，槽位化（`planner:=navfn|smac2d`、`nav:=rpp|dwb|teb|mppi`、`localization:=…`、`lio:=…`），已有 A/B 回归 harness（`tools/scripts/regress/`）。
>
> **证据标注约定**（全文逐条执行）：
> - 【源码】= 直接读到的代码/消息定义，给文件:行（本地安装的 Humble 头文件、`humble` 分支源码、上游仓库源码）；
> - 【文档】= 官方文档页面（给 URL）；
> - 【第三方】= 第三方仓库/论文（给 URL）；
> - 【我们的实测/文档】= 本仓库 `docs/`、`tools/` 里的既有结论；
> - 【无来源】= **没有找到可引用来源**，只给"要settle它必须测什么"；
> - 【推算】= 由消息定义/常数算出来的量级，不是实测。
>
> **版本警告（很重要）**：`docs.nav2.org` 已被重组为**按发行版分目录**，目前只有 `/rolling/` 与 `/jazzy/`（`/humble/` 返回 404，见 §9 未核实项）。因此 **Rolling 文档里的插件接口与 Humble 1.1.20 有实质差异**（例如 Rolling 的 `createPlan()` 多两个参数、参数键从 `planner_plugins` 改成 `plugins`、Controller 用 `newPathReceived()` 取代 `setPlan()`、多了 path handler 插件类型）。**本文凡涉及契约，一律以本机 `/opt/ros/humble/include/` 的头文件与 `humble` 分支源码为准**，Rolling 文档只用于"语义解释/命名来源"，差异处逐一标注。

---

## 0. 一页结论

### 0.1 决策表（三种接法）

| 维度 | **A. nav2 插件**（planner / smoother / controller） | **B. 独立节点**（自己吃 costmap、自己发 `/cmd_vel`） | **C. 独立进程 + 共享内存/零拷贝 IPC** |
|---|---|---|---|
| 要写多少代码 | 一个薄适配类（Humble 下 `GlobalPlanner` 只有 5 个纯虚函数，其中 4 个可留空）+ pluginlib xml + 参数段 | 一个完整节点：TF/位姿获取、costmap 源、控制律、到达判据、恢复策略、生命周期 | B 的全部 + 共享内存/自定义 RMW 或自定义 IPC 协议 + 进程编排 |
| 库是否还是纯 C++ | **是**。库代码一行不改，ROS 依赖只出现在适配层（§7.3 的 CMake 骨架） | **是**。库被编进一个 ROS 节点，但库本身仍无 ROS 头文件 | **是**（库甚至可以编成独立可执行文件/服务） |
| 每次规划的额外开销 | 一次虚函数调用 + action server 收发的消息（见 §4：**这些都不是主要成本**）；costmap 是**指针原地读**（planner/controller 形态） | 取决于 costmap 从哪来：订阅 `costmap_raw` 会**序列化+拷贝**；自己在进程内跑 costmap 层则等价于 A | 一次 IPC 传输。**同进程**才能真零拷贝；跨进程只能靠 SHM/loaned message（§4.5） |
| 一次性开销 | pluginlib/`dlopen` + `configure()`（生命周期期一次，见 §1.1） | 进程启动、DDS 发现 | 进程启动、SHM 段建立、发现 |
| 必须自己重实现的东西 | 只在插件语义范围内：planner 只需 `createPlan`；controller 需要自己写跟踪律与到达判据 | nav2 帮你做的一切：行为树/恢复、目标管理、costmap 图层与 TF 维护、`/cmd_vel` 独占、进度检查 | 同 B + IPC 编解码/一致性 |
| 与 nav2 生态的兼容 | 最好：RViz/`/plan`/BT/waypoint/`nav2_simple_commander` 全都还在 | 差：nav2 的 BT、waypoint follower、碰撞监控、速度平滑都需要重新接线 | 中：nav2 侧保留，只是规划在别的进程 |
| 我们栈的接入成本 | **最低**：新增 `planner:=tdt` 槽位、一个 yaml 段，直接复用现有 A/B harness | 中：需要新 launch、新的 `/cmd_vel` 归属决策（§6.1） | 高：需要自定义中间件/消息，收益未被任何来源证明 |
| 何时选它 | **默认选它**：想要库的算法 + 保留 nav2 的工程能力（恢复、waypoint、可视化、参数化） | 当 nav2 的**抽象与问题不匹配**（地形不是"空地/障碍"二值、底盘动力学不是差速/全向）时；这是 RM 圈的现实选择（§5.2 HWSentryNav26 作者原话） | 只有在**已证明**规划耗时/抖动是瓶颈、且同进程方案不可行时。**本次没有找到任何论文/仓库用它来跑规划器**（§9） |

### 0.2 对本次这个库的具体建议（一句话版）

**把库保持原样（纯 CMake、无 ROS），在它旁边加一个 ament 包做薄适配；先做 `GlobalPlanner` 插件（A\* 前端 + min-snap 后端，输出 nav2 的 `nav_msgs::Path`），把"带时标的 min-snap 轨迹真正被执行"这件事留给后续的 Controller 插件或独立节点**——因为 nav2 **原生没有任何"带时间的轨迹"接口**（§3），min-snap 的时标在 `Path` 里必然丢失。CMake 骨架见 §7.3，分阶段计划见 §7.4。

**效率结论（详见 §4）**：插件化本身的开销（dlopen 一次性、虚函数调用每次）在量级上**远小于**这个库自己的耗时（A\* 平均 5.4 ms / 最大 14.8 ms；min-snap 平均 3.9 ms / **最大 126.7 ms**）。真正要担心的不是"插件 vs 直调"，而是三件事：① **min-snap 的最坏 126.7 ms** 落在 20 Hz 控制环里会顶掉 2–3 拍；② costmap 的**取用方式**（指针原地读 vs 订阅 `costmap_raw` 拷贝 168 KB 级数据）；③ **执行器/回调组**导致的抖动与 `setPlan` 之后的重算节奏。

---

## 1. 问题一：nav2 插件契约，精确版（以本机 1.1.20 为准）

### 1.1 三个接口的逐字签名（Humble 1.1.20）

`nav2_core::GlobalPlanner`（本机 `/opt/ros/humble/include/nav2_core/global_planner.hpp`），5 个纯虚函数：

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;   // 4 个入参
virtual void cleanup() = 0;
virtual void activate() = 0;
virtual void deactivate() = 0;
virtual nav_msgs::msg::Path createPlan(                            // 注意：Humble 只有 2 个入参
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) = 0;
```

> **与 Rolling 文档的差异**：Rolling 教程表述为"base class provides 5 pure virtual methods…`createPlan()` 带 4 个入参（start、goal、viapoints、取消检查函数）"【文档】<https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/>。**Humble 没有 viapoints 参数**——viapoints 由 `planner_server` 自己在 `computePlanThroughPoses()` 里拼接（`nav2_planner/src/planner_server.cpp` 的 `computePlanThroughPoses`，本机 Humble 分支源码）。

`nav2_core::Smoother`（本机 `.../nav2_core/smoother.hpp`）：

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,   // ← 注意：不是 Costmap2DROS
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) = 0;
virtual void cleanup() = 0;  virtual void activate() = 0;  virtual void deactivate() = 0;
virtual bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) = 0;  // in-out + 时间上限
```

`nav2_core::Controller`（本机 `.../nav2_core/controller.hpp`）：

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;
virtual void cleanup() = 0;  virtual void activate() = 0;  virtual void deactivate() = 0;
virtual void setPlan(const nav_msgs::msg::Path & path) = 0;
virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(   // 返回值是 TwistStamped
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker) = 0;                        // 3 个入参
virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
```

> **与 Rolling 文档的差异**：Rolling 把 `setPlan()` 换成了 `newPathReceived()`，并且 `computeVelocityCommands()` 变成 5 个入参（多了 `transformed_global_plan` 与 `global_goal`），由 path handler 负责把全局路径变换/裁剪【文档】<https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_controller_plugin/writing_new_controller_plugin/>。Humble 里 **`setPlan()` 是唯一入口**，路径变换要控制器自己做（我们栈里 TEB/RPP/MPPI 都是自己 `transformGlobalPlan`）。

`GoalChecker` / `ProgressChecker`（本机 `.../nav2_core/goal_checker.hpp`、`progress_checker.hpp`）：`initialize(parent, plugin_name, costmap_ros)` / `reset()` / `isGoalReached(query_pose, goal_pose, velocity)` / `getTolerances(...)`；progress checker 是 `initialize(parent, plugin_name)` / `check(PoseStamped&)` / `reset()`。**两个 checker 都不需要 `configure/activate/deactivate`**——它们不是生命周期插件。

### 1.2 服务端怎么加载、什么时候调用（决定了"一次性 vs 每次"）

`nav2_planner/src/planner_server.cpp`（Humble 分支，本地已下载核对）：

- 构造：`gp_loader_("nav2_core", "nav2_core::GlobalPlanner")` —— **pluginlib 的"包名"参数是 `nav2_core`，不是 `nav2_planner`**（`planner_server.cpp` 构造函数）。
- 参数：`declare_parameter("planner_plugins", default_ids_)`、`declare_parameter("expected_planner_frequency", 1.0)`；**只有**在 `planner_plugins == 默认值` 时才补声明 `default_ids_[i] + ".plugin"`（`planner_server.cpp` 约 54–62 行）。真实插件类型由 `nav2_util::get_plugin_type_param(node, <id>)` 读取，它读的就是 **`<id>.plugin`**，**读不到会 `exit(-1)` 直接杀进程**（本机 `/opt/ros/humble/include/nav2_util/node_utils.hpp:135-149`）。
- 加载时机：`on_configure()` 里逐个 `createUniqueInstance(type)` → `planner->configure(node, id, tf_, costmap_ros_)`（`planner_server.cpp` 约 110–122 行）。**`dlopen` + 构造 + `configure` 都只发生一次**；`activate()/deactivate()` 只在生命周期切换时调用；**每次规划只有一次 `planners_[id]->createPlan(start, goal)` 虚函数调用**（`getPlan()`，约 518–546 行）。
- 计时口径：`result->planning_time = this->now() - start_time`，`start_time` 在动作回调开头取，**包含**等 costmap（`waitForCostmap()` 里 `while(!costmap_ros_->isCurrent()) r.sleep()`）、TF 变换（`transformPosesToGlobalFrame`）、路径校验（`validatePath`）、以及插件调用（约 419–437、270–280 行）。所以 `planning_time` **不是**插件内部耗时，做 A/B 时不要拿它当"库耗时"。
- 频率守卫：`expected_planner_frequency` > 0 时 `max_planner_duration_ = 1/f`，超时打 warning（约 133–141 行）。

`nav2_controller/src/controller_server.cpp`（Humble 分支）：

- 三个 loader：`progress_checker_loader_("nav2_core","nav2_core::ProgressChecker")`、`goal_checker_loader_("nav2_core","nav2_core::GoalChecker")`、`lp_loader_("nav2_core","nav2_core::Controller")`。
- 参数键（**精确**）：`controller_frequency`、`progress_checker_plugin`（**单数，字符串，不是 vector**）、`goal_checker_plugins`（复数 vector）、`controller_plugins`（vector）、`min_x/y/theta_velocity_threshold`、`failure_tolerance`、`publish_zero_velocity`、`speed_limit_topic`（约 39–63 行）。
  > ⚠️ 第三方参数文件里常见 `progress_checker_plugins: [...]`（例如 PolarBear 的 Jazzy 配置，见 §5.1），**Humble 1.1.20 不认这个键**——写了不会报错，只是被忽略、回落到默认 `progress_checker`。
- 控制环：`rclcpp::WallRate loop_rate(controller_frequency_)`（**墙钟**），每拍 `updateGlobalPath()` → `computeAndPublishVelocity()` → 到达判定 → `loop_rate.sleep()`；miss 掉目标频率会打 "Control loop missed its desired rate" 并 `loop_rate.reset()`（约 386–421 行）。
- `setPlan()` 的调用时机：`FollowPath` 动作**开始时**调一次（`setPlannerPath(action_server_->get_current_goal()->path)`，约 382 行），或**被抢占时**调一次（`updateGlobalPath()`，约 538–563 行）。**不是每拍都调**。
- `/cmd_vel` 归属：`vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1)`（约 198 行）；发布时还会**再拷贝一次** `velocity.twist` 进新 Twist（`publishVelocity()`，约 567–574 行），并且**只在有订阅者时**才发。异常/到达时会 `publishZeroVelocity()`。

`nav2_smoother/src/nav2_smoother.cpp`（Humble 分支）：参数 `smoother_plugins`（vector）、`costmap_topic`（默认 `global_costmap/costmap_raw`）、`footprint_topic`（默认 `global_costmap/published_footprint`）、`robot_base_frame`、`transform_tolerance`；`on_configure()` 里建 `CostmapSubscriber`/`FootprintSubscriber`/`CostmapTopicCollisionChecker`，再 `loadSmootherPlugins()`；动作回调里 `smoothers_[id]->smooth(result->path, goal->max_smoothing_duration)`，`was_completed` 与 `smoothing_duration` 回填结果。

### 1.3 action / 消息契约（本机 `nav2_msgs` 定义逐字）

`nav2_msgs/action/ComputePathToPose.action`：

```
#goal
geometry_msgs/PoseStamped goal
geometry_msgs/PoseStamped start
string planner_id
bool use_start        # false 时用机器人当前位姿
---
#result
nav_msgs/Path path
builtin_interfaces/Duration planning_time
---
#feedback            # 空
```

`nav2_msgs/action/SmoothPath.action`：goal = `nav_msgs/Path path` + `string smoother_id` + `builtin_interfaces/Duration max_smoothing_duration` + `bool check_for_collisions`；result = `nav_msgs/Path path` + `builtin_interfaces/Duration smoothing_duration` + `bool was_completed`。

`nav2_msgs/action/FollowPath.action`：goal = `nav_msgs/Path path` + `string controller_id` + `string goal_checker_id`；result = `std_msgs/Empty result`；feedback = `float32 distance_to_goal` + `float32 speed`。**注意 result 是空的**——控制器不回报轨迹，只回报"跟随结束"。

`nav_msgs/msg/Path`（本机）：`std_msgs/Header header` + `geometry_msgs/PoseStamped[] poses`——**只有几何，没有时间**（详见 §3）。

### 1.4 costmap 是怎么交到插件手里的（关键）

| 接口 | 拿到的东西 | 类型/生命周期 | 能否原地读 |
|---|---|---|---|
| `GlobalPlanner::configure` | `std::shared_ptr<nav2_costmap_2d::Costmap2DROS>` | 由 `planner_server` 在 `on_configure` 里 `make_shared` 并**常驻**；`planner_server` 还把它放进自己的 `nav2_util::NodeThread costmap_thread_` 里 spin（`planner_server.cpp` 约 65、94 行）⇒ **costmap 的更新在另一个线程** | ✅ `costmap_ros->getCostmap()` 返回 **`Costmap2D *`（裸指针，无拷贝）**，本机头文件注释："Same as calling `getLayeredCostmap()->getCostmap()`"（`costmap_2d_ros.hpp:222-227`） |
| `Controller::configure` | 同上（`Costmap2DROS` 共享指针） | 同上（`controller_server` 也有 `costmap_thread_`） | ✅ 同上 |
| `Smoother::configure` | `CostmapSubscriber` + `FootprintSubscriber` | 订阅 **`nav2_msgs/msg::Costmap`** 话题（默认 `global_costmap/costmap_raw`），内部持有一份 `std::shared_ptr<Costmap2D> costmap_` 和 `nav2_msgs::msg::Costmap::SharedPtr costmap_msg_` | ⚠️ **是消息、是拷贝**：`CostmapSubscriber::getCostmap()` 先 `toCostmap2D()`（含 `resizeMap`）再返回内部那份共享 `Costmap2D`（`nav2_costmap_2d/src/costmap_subscriber.cpp` Humble 分支，`getCostmap`/`toCostmap2D`/`resizeMap` 三处） |

主栅格怎么读（本机 `nav2_costmap_2d` 头文件）：

- `Costmap2D::getCharMap()` → `unsigned char *`，**底层那张 master grid**（`costmap_2d.hpp:232`）；
- `getSizeInCellsX()/getSizeInCellsY()`、`getResolution()`、`getOriginX()/getOriginY()`（`costmap_2d.hpp:238/244/274/262/268`）；
- `getCost(mx,my)` / `getCost(index)`（`costmap_2d.hpp:149/156`）；
- **同步原语**：`typedef std::recursive_mutex mutex_t; mutex_t * getMutex()`（`costmap_2d.hpp:361-365`）。nav2 自己就读它：`planner_server::isPathValid()` 里 `std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));` 然后逐点取 `getCost()`；`costmap_2d_ros.cpp` 在装图层时同样"持有 costmap 锁再初始化插件"，注释写着 *"lock the costmap because no update is allowed until the plugin is initialized"*（Humble 源码）。
- 值的语义（本机 `cost_values.hpp:42-46`）：`NO_INFORMATION = 255`、`LETHAL_OBSTACLE = 254`、`INSCRIBED_INFLATED_OBSTACLE = 253`、`FREE_SPACE = 0`。⚠️ **这与 `nav_msgs/OccupancyGrid` 的语义（0 空闲 / 100 占用 / −1 未知）完全不同**，也不是 `nav2_msgs/Costmap.msg`（`uint8[] data`）。
- **膨胀层没有独立 API**：膨胀是 `inflation_layer` 直接写进同一张 master grid；想拿到"图层对象"只能 `costmap_ros->getLayeredCostmap()->getPlugins()`（返回 `std::vector<std::shared_ptr<Layer>> *`，`layered_costmap.hpp:133`）再 `dynamic_pointer_cast`。**没有任何稳定的、带版本保证的"读膨胀层参数"接口**——所以规划器通常只读 master grid 的代价梯度，而不是去问膨胀层。

### 1.5 你要写的那几行"注册样板"（Humble 精确版）

`nav2_navfn_planner` 是最小可抄样本（本机 `/opt/ros/humble/share/nav2_navfn_planner/`）：

```xml
<!-- global_planner_plugin.xml -->
<library path="nav2_navfn_planner">
  <class name="nav2_navfn_planner/NavfnPlanner" type="nav2_navfn_planner::NavfnPlanner"
         base_class_type="nav2_core::GlobalPlanner">
    <description></description>
  </class>
</library>
```

```cmake
# CMakeLists.txt（nav2_navfn_planner 的原始写法）
add_library(${library_name} SHARED ...)
ament_target_dependencies(${library_name} ...)
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
install(FILES global_planner_plugin.xml DESTINATION share/${PROJECT_NAME})
```

```xml
<!-- package.xml -->
<export>
  <build_type>ament_cmake</build_type>
  <nav2_core plugin="${prefix}/global_planner_plugin.xml" />
</export>
```

```cpp
// 源文件末尾
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_navfn_planner::NavfnPlanner, nav2_core::GlobalPlanner)
```

【源码】`nav2_navfn_planner/global_planner_plugin.xml`、`CMakeLists.txt`、`src/navfn_planner.cpp` 末行（本机安装 + Humble 分支）；【文档】同样的四件套也写在插件教程里 <https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/>。**`<class name>` 用 `包名/类名`（斜杠）是 Humble 的惯例；Rolling 是 `包名::类名`**（第三方参数文件里那句注释 *"In Iron and older versions, `/` was used instead of `::`"* 是同一件事的现场记录，见 §5.1 引用的 PolarBear yaml）。

> **`pluginlib`/`class_loader` 是什么**：`class_loader` 是 "ROS-independent package for loading plugins during runtime"，"utilizes the host operating system's runtime loader to open runtime libraries (e.g. .so/.dll/.dylib files), introspect the library for exported plugin classes"【文档】<https://github.com/ros/class_loader>（`ros2` 分支 README）。`pluginlib` 是它的 ROS 封装。**换句话说：插件机制本身就只是 `dlopen`+`dlsym`+虚表，不引入新的运行期抽象。**

---

## 2. 问题二的答案（先说结论）

**nav2 原生不支持"带时间的轨迹"（min-snap/min-jerk 那种"每段有持续时间、带速度/加速度约束"的对象）。** 证据（逐条可查）：

1. **消息层：没有这种消息。** 本机 `nav_msgs/msg/Path.msg` 全文只有：
   ```
   std_msgs/Header header
   geometry_msgs/PoseStamped[] poses
   ```
   没有任何 duration/time-allocation 字段（`geometry_msgs/PoseStamped` 自带 `header.stamp`，但见第 4 条）。本机 `nav2_msgs/msg/` 下的全部 14 个消息里**没有 Trajectory 类消息**（`BehaviorTreeLog, BehaviorTreeStatusChange, CollisionMonitorState, CostmapFilterInfo, CostmapMetaData, Costmap, EdgeCost, ParticleCloud, Particle, RouteEdge, Route, RouteNode, SpeedLimit, VoxelGrid`）。带时间的轨迹消息在 ROS 2 里是 `trajectory_msgs/JointTrajectory`（关节）与 `trajectory_msgs/MultiDOFJointTrajectory`（无人机），**nav2 一个都不用**。
2. **动作层：`FollowPath` 只收 `nav_msgs/Path`。** 见 §1.3。
3. **接口层：Controller 只吐一拍控制量。** `computeVelocityCommands(...) → geometry_msgs::msg::TwistStamped`；`Controller` 接口里**没有任何**返回轨迹的方法（本机 `controller.hpp` 全文）。Smoother 是 `Path → Path`（`smooth(nav_msgs::msg::Path &, Duration) → bool`）；Planner 是 `pose,pose → Path`。三个接口全部以 `Path` 为枢纽，而 `Path` 无时标。
4. **nav2 自己会主动丢弃 Path 上的时间戳。** MPPI 的 path handler 把全局路径里**每个 pose 的 `header.stamp` 覆盖成当前位姿的时间戳**：
   ```cpp
   transformed_plan.header.stamp = global_pose.header.stamp;      // path_handler.cpp:71
   global_plan_pose->header.stamp = global_pose.header.stamp;     // path_handler.cpp:86
   ```
   【源码】`nav2_mppi_controller/src/path_handler.cpp`（Humble 分支）。⇒ **把时标塞进 `poses[i].header.stamp` 这条路，在 nav2 自带控制器上会被无声抹掉**。
5. **唯一"带时间"的东西是 MPPI 的内部实现细节**：MPPI 在自己的滚动时域里维护一条 `time_steps × model_dt` 的候选轨迹并做优化，但对外**只输出第一拍** `TwistStamped`（`nav2_mppi_controller`，`computeVelocityCommands`）。这条内部轨迹不是可插拔接口、也不给别人消费。
6. **对照：别家怎么带时间。** 无人机栈干脆**不用 `Path`**：EGO-Planner 的 `traj_server.cpp` 发布自定义消息
   ```cpp
   pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
   ```
   而 `quadrotor_msgs/PositionCommand.msg` 带 `position / velocity / acceleration / yaw / yaw_dot / trajectory_id / trajectory_flag`——**时间参数化直接进消息定义**【第三方】<https://github.com/ZJU-FAST-Lab/ego-planner>（`src/planner/plan_manage/src/traj_server.cpp`、`src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg`）。RM 侧的 HWSentryNav26 则把"轨迹 + 速度剖面"做成自己的结构体（见 §5.2）。

### 2.1 那 min-snap 后端该放哪？（三种放法，代价不同）

| 放法 | 怎么做 | 丢什么 | 什么时候选 |
|---|---|---|---|
| **放进 Planner**（A\* + min-snap 一步到位，输出按 `dt` 采样的 `Path`） | `createPlan()` 里跑 A\* → min-snap → 按库的 `dt` 采样成 `geometry_msgs/PoseStamped` 序列 | **时标**（`Path` 装不下）；下游控制器只能用几何跟踪（RPP/MPPI 都只看几何） | **首选**。语义最干净：min-snap 本质是"把一条几何路径整形+限速"，输出仍是路径 |
| **做成 Smoother**（navfn/smac 出 path → min-snap 优化） | 实现 `nav2_core::Smoother`，`smooth(Path&, Duration)` 原地改写 | 同样丢时标；**另外**：Smoother 的 costmap 走 `CostmapSubscriber`（拷贝 + §6.4 的并发 bug 面），且 `SmoothPath` 只在 BT 里被显式 tick（不是每个规划周期自动跑） | 想"复用 nav2 前端 + 只换后端"、并且能接受"平滑是 BT 里显式一步"时 |
| **做成 Controller**（`setPlan()` 时把 `Path` 变成带时标的内部轨迹，每拍按时间采样出 `Twist`） | 实现 `nav2_core::Controller`：`setPlan()` 里跑 min-snap，`computeVelocityCommands()` 里 `t = now - t0` 查表 | 要自己写：路径跟踪、到达判据（交给 `GoalChecker`）、速度限幅、失败恢复（抛 `nav2_core::PlannerException` 会被 server 兜住） | **只有当真要按 min-snap 的时标执行时**才选它。此时 `Path` 只是"任务的几何输入"，时标由自己维护 |

**我们的库属于哪一种？** 上游的输出形态很关键（本次从源码核实）：

- `MinimumSnap::SolveOutput` = `{ std::vector<Eigen::Vector2f> path; std::vector<PointPair> corridor; int iter; double time; bool success; }`——**返回的是"路径点 + 走廊"**，多项式系数与段时长是内部量（`evaluateEquation`/`lineDecoder` 都是 `protected`）。【源码】`src/MinimumSnapOsqp/minimumSnap.hpp:179-186, 256-259`。
- 但这条 `path` **是按时间等间隔采样出来的**：内部 `while(tres > dt){ tres -= dt; … op.push_back(getfx(i, time)); }`，末尾还按 `tres > dt*0.1` 补一个点（`minimumSnap.cpp:541-551`）；`dt` 默认 0.1 s（`minimumSnap.hpp` 的 `float dt = 0.1`），上游 README 的运行参数表也写着"采样时间 0.1s：MinimumSnap结果函数按照分配时间、速度进行采样"【源码】+【第三方】README。
  ⇒ **时标是"隐式"的（相邻点 ≈ `dt`），一旦落进 `nav_msgs::Path` 就只剩"约定"，不是数据。**
- 段时长由 `trapezoidalTimeAllocation(path, maxSpeed, maxAcc)` + `setTimeAllocated()` 决定，`maxSpeed/maxAcc` **只是自动时间分配用的参数，不是求解期的硬约束**（`Usage.md` 原话："该参数只影响未显式设置 timeAllocated 时的时间分配，不是求解阶段的硬速度约束"）【第三方】`doc/Usage.md`。
- `KinodynamicAstar::Result` 反而**显式带时间**：`struct Sample{ State state; Eigen::Vector2d acceleration; double time; }`（`State = [x,y,vx,vy]`），`Config::sampleTime = 0.1`【源码】`src/YAstar/kinodynamicAstar.hpp`。

**因此对本文这个库**：先做 Planner（把 min-snap 的 `path` 输出成 `Path`，把 `dt` 变成**显式参数**并在文档里写成契约）；等真需要按时标跑（例如要卡"过洞/上台阶"的时间窗）再做 Controller 或独立节点。

---

## 3. 效率：插件 vs 直调，逐项拆开算

> 口径：本节把开销分成 **one-off（一次性）** 与 **per-call（每次规划/每拍）**，并区分 **有来源的数字** 与 **【无来源】必须自己测的数字**。

### 3.1 one-off（生命周期/加载期）

| 项 | 量级/来源 | 说明 |
|---|---|---|
| `dlopen` + 类构造 + `configure()` | **机制有据，数字【无来源】** | `class_loader`/`pluginlib` 用宿主 loader 打开 `.so`【文档】<https://github.com/ros/class_loader>。`planner_server` 只在 `on_configure()` 里做一次（见 §1.2）。**没有找到任何公开的"pluginlib 加载耗时"实测数字**——搜索到的只有机制说明与教程，没有任何 ms/µs 级测量（§9）。 |
| TF buffer / costmap 共享指针 | 由 server 提供，不额外付 | 你的插件只是拿引用（§1.4）。 |
| costmap 首次分配 560×300 | 168 000 B（= 560×300×1 B）【推算，算术】 | 与我们仿真里 map 尺寸一致（上游基准就是 560×300 @0.05 m）。 |

**结论**：一次性开销在整个生命周期里只出现一次（除非服务器崩溃 respawn），**与"每次规划快不快"无关**。它唯一可能咬人的场景是"每次规划都重新 `createInstance`"——而 nav2 的 server 不这么做。

### 3.2 per-call（每次规划/每拍）

| 项 | 有来源的数字 | 判断 |
|---|---|---|
| 一次虚函数调用 | 【无来源】 | 量级上是几 ns 级（间接跳转），**与库自身 5.4 ms/126.7 ms 相比可忽略**——这是量级推理，不是引用。 |
| action server 收发（`ComputePathToPose`） | 【无来源，只有工具链】 | 走同一进程/跨进程取决于 `use_composition`（§3.4）。要 settle：`ros2_tracing`（见 §3.5）。 |
| **我们自己库的耗时（决定一切的那一项）** | **有来源**（见 §3.3） | A\* 平均 5.4 ms / 最大 14.8 ms；min-snap 平均 3.9 ms / **最大 126.7 ms**。 |
| `planner_server::publishPlan()` 的路径拷贝 | 【源码】每次 `auto msg = std::make_unique<nav_msgs::msg::Path>(path);`——**整条路径深拷贝一次**，且只在有订阅者时发布 | 300 点 ≈ 21–24 KB 拷贝【推算，按消息字段算：每 `PoseStamped` ≈ 72–80 B CDR】。相对 5 ms 级规划可忽略。 |
| Controller 侧每拍拷贝 | 【源码】`publishVelocity` 把 `velocity.twist` 拷成 `Twist` 再发；`nav2_velocity_smoother` 每拍再处理一次 | 每拍几十字节，可忽略；**但 velocity_smoother 是一个额外节点/额外一次 DDS 往返**，属于"要不要进链路"的取舍。 |

### 3.3 这个库自己公布的性能（必须进决策表）

上游 README「性能表现」表（1000 次，AMD 7735H @4.2 GHz / Ubuntu 22.04 / GCC 11.2.0，场地 560×300 @0.05 m）【第三方】<https://github.com/T-DT-Algorithm-2026/tdt-nav-kit>（`README.md`）：

| 地图 | 算法 | 平均 | 最大 |
|---|---|---|---|
| RMUC2024 | Astar | 7627.44 µs | 21806 µs |
| RMUC2024 | Minimum Snap | 1255.42 µs | 14493.1 µs |
| RMUC2024 | 生成代价地图 | 1604.13 µs | 6870.61 µs |
| RMUC2024 | 化简路径 | 34.0191 µs | 242.632 µs |
| **RMUC2026** | **Astar** | **5446.45 µs** | **14761.2 µs** |
| **RMUC2026** | **Minimum Snap** | **3901.7 µs** | **126698 µs** |
| **RMUC2026** | **生成代价地图** | **1588.52 µs** | **3613.08 µs** |
| **RMUC2026** | **化简路径** | **26.0119 µs** | **117.99 µs** |

上游 `doc/MinimumSnap.md` 另有一张"1000 次、全图随机取点"的表，其中 `RMUC2025 / OSQPCorridor / astar+势场` 平均 6052.95 µs、**最大 204404 µs**；`Close`（闭式）后端平均 694.975 µs / 最大 6757.24 µs【第三方】`doc/MinimumSnap.md`。

**这张表的含义**：
- **后端不是"便宜的那个"**：OSQP 走廊后端的最坏值（126.7 ms / 204 ms）比 A\* 前端最坏值还大一个数量级；闭式后端快且稳（最大 6.8 ms）但作者自己说"闭式求解不建议使用 jps 作为前端，因为质量很差基本不可优化"。
- 在 20 Hz（50 ms）控制环里，**126.7 ms 会顶掉 2–3 拍**；在 1–2 Hz（`expected_planner_frequency`）的全局规划器里则完全可接受。⇒ **这决定了 min-snap 更适合放在 Planner/Smoother（低频、可等待），而不是 Controller（高频、不可阻塞）**。
- 上游没有给出分位数（P50/P95/P99），只有 avg/max；而"最大 126.7 ms"这种长尾才是实时性真正的风险 ⇒ **必须自己补测分位数**（§3.6）。

### 3.4 costmap：原地读 vs 拷贝，以及 nav2 自己测过的 RMW/IPC 账

**（a）原地读（planner/controller 形态）**
`getCostmap()` 是裸指针（§1.4），`getCharMap()` 直接给 `unsigned char*`。「零拷贝」在这里是真的：**不经过 DDS、不序列化、不 new**。

**（b）消息拷贝（smoother 形态 / 独立节点订阅）**
smoother 拿的是 `CostmapSubscriber`，订阅 `nav2_msgs/msg::Costmap`，`getCostmap()` 内部 `toCostmap2D()`（含 `resizeMap`）。数据量：`uint8[] data` = 560×300 = **168 000 B**（+ `CostmapMetaData`），**每次更新都要序列化/反序列化一遍**。

**（c）ROS 2 intra-process 的官方口径（很关键，反直觉）**
ROS 2 的 intra-process 设计文档（`design.ros2.org`，Crystal 时代写就、至今仍是权威描述）明确说：

- "from the latency and CPU utilization point of view, **it is convenient to use intra-process communication only when the message size is at least 5KB**"【文档】<https://design.ros2.org/articles/intraprocess_communications.html>；
- "the performance of a single process ROS 2 application with intra-process communication enabled are **still worst than what you could expect from a non-ROS application sharing memory between its components**"——**同进程的 ROS 2 也打不过"非 ROS 的共享内存"**；
- intra-process 仍需通过 RMW 发"meta-message"，因此"performance … heavily dependent on the chosen RMW implementation"。

**（d）nav2 官方实测的 CPU 账（TurtleBot4 仿真，rolling commit，2025-12）**【文档】<https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/>（"Performance in ROS 2: RMW, Node Composition, Intra-process Communication, and QoS"）：

| Middleware | 配置 | CPU |
|---|---|---|
| Zenoh | 基线 | 4.5% |
| Zenoh | IPC (SharedPtr) | 6.0% |
| Zenoh | IPC + ConstSharedPtr | 5.8% |
| CycloneDDS | 基线 | 18.3% |
| CycloneDDS | IPC (SharedPtr) | 17.8% |
| CycloneDDS | IPC + ConstSharedPtr | 16.1% |
| FastDDS | 基线 | 6.8% |
| FastDDS | IPC (SharedPtr) | 7.8% |
| FastDDS | IPC + ConstSharedPtr | 7.8% |

nav2 maintainer 的结论是：**`use_composition: true` 强烈推荐**（"reduces process overhead, and generally improves system efficiency"），而 **intra-process 要看 RMW**：FastDDS/Zenoh 开 IPC 反而更费 CPU，CycloneDDS 才降。`use_intra_process_comms` 默认 `false`。

⇒ 对我们的直接含义：**"把 costmap 从 nav2 交给规划器"最省的形态是"同一个进程 + 指针原地读"（也就是 planner/controller 插件 + composition），而不是"跨节点发消息再开 IPC"**。开 IPC 不是免费的。

### 3.5 rclcpp 发布/订阅延迟与执行器抖动（有来源的部分）

| 结论 | 来源 |
|---|---|
| rclcpp 相对裸 DDS 的通信开销**最多 50%** | ros2_tracing 论文转引：*"Kronauer et al. investigated the end-to-end latency of communications and found that its overhead is up to 50% compared to directly using DDS"*【第三方/论文】<https://arxiv.org/abs/2201.00393>（该论文本身是 nav2 官方 profiling 教程推荐的工具链） |
| 打开全部 ROS 2 instrumentation 后，端到端消息延迟开销**平均 0.0033 ms（3.3 µs）** | 同上（ros2_tracing 摘要） |
| rclcpp 的三种 executor "work well for most applications, but there are some issues that make them **not suitable for real-time applications**, which require well-defined execution times, determinism, and custom control over the execution order" | ROS 2 官方文档【文档】<https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html>（正文源：`ros2/ros2_documentation@humble` `source/Concepts/Intermediate/About-Executors.rst`） |
| 回调组语义：`MutuallyExclusive`（同组串行）/`Reentrant`（可并行）；不同组之间**总是**可并行；配错会死锁 | 【文档】<https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html> |
| nav2 自己的缓解手段：`nav2_velocity_smoother` 的 `use_realtime_priority` ——"Adds soft real-time prioritization to the controller server … set the controller's execution thread to a higher priority than the rest of the system (`90`) to meet scheduling deadlines to have **less missed loop rates**" | 【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/> |
| iceoryx（真零拷贝共享内存）宣称"**latency of less than 1 µs** for transferring a message … this latency is constant as size doesn't matter" | 【厂商宣称】<https://iceoryx.io/classic/v2.0.0/getting-started/what-is-iceoryx/> |
| Fast DDS 有 SHM transport（同 host，每 participant 一个 shared memory segment） | 【文档】<https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html> |
| `rclcpp::LoanedMessage` 存在（borrow/return loaned message），但能否真零拷贝取决于 RMW 与消息类型 | 【源码】<https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/loaned_message.hpp> |

**没有找到的**：ROS 2 Humble + rmw_fastrtps 在**环回**上的端到端 pub/sub 延迟的权威实测数字（µs 级）；把"规划器放进独立进程 + 共享内存"与"同进程直调"对比过的**规划耗时**实测。【无来源】——见 §3.6 的测量方案。

### 3.6 要 settle 它，必须在**我们机器上**测这几项（列表即实验设计）

1. **pluginlib 一次性加载**：给 `planner_server` 加 `--log-level debug` 或用 `ros2_tracing`/`perf` 量 `dlopen` 到 `configure()` 返回的墙钟；对照"独立节点启动到 ready"。预期：**一次性、只影响起动时间**。（无来源 ⇒ 必须自测）
2. **虚函数 vs 直调**：同一份库代码，(a) 在插件 `createPlan()` 里调，(b) 在纯 C++ benchmark 里调（上游 `main.cpp` 的 `benchTime` 就是现成模板），跑 1000 次取 **P50/P95/P99/max**。判据：差值若 < 库自身 P50 的 1%，则"插件税"在本问题下不可见。
3. **costmap 取用方式**：(a) `getCostmap()->getCharMap()` 原地 + 值域转换，(b) 订阅 `global_costmap/costmap_raw` 反序列化。量两侧的**每次同步耗时**与**最坏值**（168 KB 量级，见 §3.4）。
4. **`Path` 构建+发布**：先用 `ros2 topic hz/delay` 与 `ros2_tracing` 量 300 点 `Path` 的发布端到端延迟；同时用纯 C++ 侧计时把"构造 Path"与"发送"分开。
5. **执行器抖动**：`planner_server`/`controller_server` 在 (a) 默认单线程 executor、(b) composition + 多线程 executor + 不同回调组 下的**规划回调间隔抖动**（P99 − P50）。判据：把 min-snap 的最坏 126.7 ms 放进时间线，看它是否让 `controller_frequency` 掉拍（server 自己会打 "Control loop missed its desired rate"）。
6. **`setPlan` 频率与重算节奏**：确认我们的 BT 多久 re-tick 一次 `ComputePathToPose`（`PipelineSequence` 语义），以及 `planner_server.planning_time` 与"库内部耗时"的差（§1.2 已说明它包含等待/校验）。
7. **分位数**：上游只给 avg/max，**必须自己补 P95/P99**（尤其 OSQP 走廊后端）。
8. **工具链**：`ros2_tracing`（`ros2 trace` + LTTng + `babeltrace`/`tracetools_analysis`）是 nav2 官方 profiling 教程指定的工具，且它自己的开销平均 3.3 µs【文档+论文】<https://docs.nav2.org/rolling/tutorials/general_tutorials/get_profile/get_profile/>、<https://arxiv.org/abs/2201.00393>。

---

## 4. 问题四：真实世界的三种接法（含出处与作者自述的取舍）

### 4.1 模式 A：包成 nav2 插件（**先把"外部纯 C++ 库"包成插件的先例**）

| 例子 | 包的是什么外部库 | 可核实的证据 |
|---|---|---|
| `nav2_smac_planner` | **OMPL**（纯 C++、无 ROS 的采样式规划库） | `CMakeLists.txt: find_package(ompl REQUIRED)`；`package.xml: <depend>ompl</depend>`；`include/nav2_smac_planner/node_hybrid.hpp:28 #include "ompl/base/StateSpace.h"`，`:125 ompl::base::StateSpacePtr state_space;`【源码】ros-navigation/navigation2 `humble` 分支 |
| `nav2_constrained_smoother` | **Ceres Solver**（纯 C++ 优化库） | `package.xml`: `<description>Ceres constrained smoother</description>`、`<depend>libceres-dev</depend>`（第 6、20 行）；`CMakeLists.txt:4` 甚至写着 `set(CMAKE_BUILD_TYPE Release) # significant Ceres optimization speedup`——**nav2 自己承认外部求解器的编译类型对性能有显著影响**【源码】ros-navigation/navigation2 `humble` 分支 |
| `spatio_temporal_voxel_layer`（STVL） | **OpenVDB**（纯 C++ 体素库），作为 costmap 图层插件 | 已被 nav2 官方插件目录收录：*"Spatio-Temporal Voxel Layer — Steve Macenski — Maintains temporal 3D sparse volumetric voxel grid with decay through sensor models"*【文档】<https://docs.nav2.org/rolling/configuration_and_development/navigation_plugins/>。我们自己的 global costmap 就在用它【我们的实测/文档】`docs/architecture.md:137` |
| `pb_nav2_plugins`（SMBU PolarBear 战队） | **他们自己的 C++ 行为/图层**，独立仓库、第三方维护 | README：*"`pb_nav2_plugins` 是一个用于扩展 `Navigation2`（Nav2）框架的插件库"*；`costmap_plugins.xml` 注册 `pb_nav2_costmap_2d::IntensityVoxelLayer`（`base_class_type="nav2_costmap_2d::Layer"`）；`CMakeLists.txt: pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)`【第三方】<https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins> |
| `pb_omni_pid_pursuit_controller`（同战队） | **他们自己的全向 PID 跟踪律**，做成 `nav2_core::Controller` 插件 | README 列出全部参数（`translation_kp/ki/kd`、`lookahead_dist`、`curvature_min/max`…）；在 `pb2025_sentry_nav` 的 `nav2_params.yaml` 里被写成 `plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitController"`【第三方】<https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller> |

**这些作者的取舍（可引用的部分）**：
- nav2 官方 tuning guide 明确推荐 `use_composition: true` 以省 CPU/内存，并且**插件是官方推荐的扩展方式**（插件目录本身就是它的产品面）【文档】<https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/>。
- PolarBear 的做法是"**nav2 骨架 + 自研插件**"：`pb2025_sentry_nav` 的 `pb2025_nav_bringup/package.xml` 同时依赖 `navigation2`、`pb_nav2_plugins`、`pb_omni_pid_pursuit_controller`、`terrain_analysis`、`terrain_analysis_ext`——**自研的"地形分析"节点与 nav2 并存**，而"控制器/图层"则以插件形式进入 nav2【第三方】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>。注意他们的 `robot_base_frame: gimbal_yaw` + `fake_robot_base_frame: gimbal_yaw_fake`（我们栈的 `base_link_fake` 是同一套思路）【第三方】`pb2025_nav_bringup/config/simulation/nav2_params.yaml:98,153,154,272,389,439`。

### 4.2 模式 B：独立节点（自己吃地图、自己发底盘指令）

**（i）HWSentryNav26 —— 浙江大学 Hello World 战队 26 赛季轮腿哨兵导航**（C++，135 ⭐）【第三方】<https://github.com/Polyacetone/HWSentryNav26>

这条最有参考价值，因为它**就是"纯 C++ 规划栈 + ROS 2 外壳、不用 nav2"的现成答案**：

- **不用 nav2**：README 原话 *"导航状态机采用扁平化有限状态结构…状态机为本项目从零设计，**未采用 Navigation2 行为树等现成方案**"*；`nav_executor/package.xml` 的依赖是 `rclcpp, rclcpp_components, tf2, tf2_ros, cv_bridge, tf2_geometry_msgs, visualization_msgs, sensor_msgs, nav_msgs, std_msgs, interfaces, common_libs`——**没有任何 nav2 依赖**（`map_server` 同理）。
- **作者对"自研 vs 框架"的判断（可直接引用）**：*"判断标准不是'别人都用'，而是'**现成框架的抽象是否和你的问题对齐**'…如果你的机器人是普通四轮、平地行驶，用它比较省心。但一旦你的需求超出它的抽象——比如地形不是'空地/障碍'二值、底盘动力学不是普通舵轮/全向轮——你就要在它的框架里打补丁，**补丁越多，越不如自己搭**。"*（`TUTORIAL.md`）
- **他们的轨迹表示（与我们的 min-snap 问题同构）**：`AnnotatedPath` = 一次规划的**不可变路径包**：`MincoTrajectory` + `PathSpeedProfile` + 台阶段 + 代价图层 + `goal_id`（用于丢弃过期结果）+ `planning_performance`；`PathSpeedProfile` 的原子状态是 `{arc_length, time, velocity}`，并注明 *"区间内 v² 对弧长线性，因此切向加速度恒定"*【第三方】`nav_executor/include/nav_executor/common/trajectory/annotated_path.hpp`、`path_speed_profile.hpp`。
- **几何与时标解耦（关键设计判断）**：*"关键不变量：**几何与时标解耦**——MINCO 的参数时长只是几何优化的内部坐标，不进入执行。原因：执行时标完全由速度剖面决定（见阶段 6）"*；并且统一用**弧长**做坐标，使"规划线程不可变地生成、执行线程无歧义地消费"【第三方】`DESIGN.md`。
- **控制频率的取法**：*"`nav_executor` 不设独立控制定时器，而是以底盘反馈（`ChassisStatus`，20 Hz）的每次到达作为一拍的触发源。这样控制周期与底盘反馈天然同步，**不存在两个时钟的相位混叠**"*【第三方】`DESIGN.md`。
- **代价图的处理（与"读不读 nav2 膨胀层"直接相关）**：`map_server` 离线做**物理距离膨胀**，理由是 *"下游的 MINCO（L-BFGS）与 MPC（FDDP）都是基于梯度的优化器，如果代价只在障碍格内跳变到 255、格外为 0，采样得到的代价场在边界附近梯度急剧变化甚至不连续，线搜索容易失败"*【第三方】`DESIGN.md`。⇒ **梯度型后端要的是"连续可导的距离场"，不是 nav2 那套 0/253/254/255 的阶梯代价**。这条直接适用于我们的 min-snap（OSQP 走廊后端虽然不需要梯度，但走廊生成同样依赖"哪些格是障碍"的干净语义）。

**（ii）无人机栈：Fast-Planner / EGO-Planner 家族**：规划 + `traj_server` 是**独立 ROS 节点**，通过**自定义消息**（`quadrotor_msgs/PositionCommand`：position/velocity/acceleration/yaw/yaw_dot + trajectory_id/flag）把**带时标的轨迹**交给控制器，话题 `/position_cmd`【第三方】<https://github.com/ZJU-FAST-Lab/ego-planner>（`src/planner/plan_manage/src/traj_server.cpp:240`；`src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg`）。这套接法**不能直接搬到 nav2**——因为 nav2 的 Controller 接口只吃 `TwistStamped`、不吃位置/速度前馈（§2）。

### 4.3 模式 C：独立进程 + 共享内存 / 零拷贝 IPC

- **iceoryx**：宣称 <1 µs 常数延迟、真零拷贝（厂商口径，§3.5）；ROS 2 侧有对应 RMW 集成，但 **本次没有找到任何"把规划器放进独立进程、用 SHM 喂 costmap"的真实机器人项目**（§9）——公开的都是中间件性能测试（如 IEEE "Latency Analysis in Internal Layers of Shared Memory Communication in ROS2"）而不是规划器案例。
- **Fast DDS SHM**：DDS 内建、同 host 自动生效（segment/segment buffer 机制）【文档】<https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html>。注意：**它仍然要经过序列化与 DDS 语义**，与 §3.4(c) 的 intra-process 结论合起来看：**"跨进程零拷贝"在这条链路上不是免费的**。
- **nav2 自己的经验数据**（§3.4(d)）说明：即使全在同进程（composition），开 IPC 也会让 FastDDS/Zenoh 的 CPU 上升。⇒ **不建议为"省规划器通信开销"走到模式 C**；要省，先走"同进程 + 指针原地读"。

### 4.4 三种模式的公开证据强度（诚实评估）

| 模式 | 证据强度 |
|---|---|
| A 插件 | **强**：nav2 官方插件目录 + 多个外部库（OMPL/Ceres/OpenVDB）先例 + 第三方战队的插件仓库 |
| B 独立节点 | **强**：HWSentryNav26 全仓库可查；PolarBear 的 `terrain_analysis*` 与 nav2 并存；无人机栈 |
| C 独立进程+SHM | **弱**：只有中间件层面的性能文档/论文，**没有找到规划器案例** |

---

## 5. 问题五：落地坑（逐条挂到我们自己的实测/文档）

### 5.1 `/cmd_vel` 是单发布者契约，接之前先决定"谁拥有它"

- nav2 内部：`controller_server` 是唯一发布 `cmd_vel` 的 server（`vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1)`）【源码】`controller_server.cpp:198`；`nav2_behaviors`（Spin/BackUp…）自身也会发速度指令，所以"谁能发速度"本来就是 nav2 的配置问题。
- **我们栈的实际链路**（【我们的实测/文档】`src/nav/rm_navigation/rm_navigation/launch/navigation_launch.py`）：
  `controller_server`（`cmd_vel → cmd_vel_nav`，:237）→ `nav2_velocity_smoother`（`cmd_vel → cmd_vel_nav`、`cmd_vel_smoothed → cmd_vel`，:298）→ `/cmd_vel` → `fake_vel_transform`（订阅 `/cmd_vel`，发布 `/cmd_vel_chassis`，并以 20 Hz 发布 `base_link→base_link_fake`）【我们的实测/文档】`src/nav/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp:11-14`。
- **⇒ 若走独立节点形态，必须显式二选一**：要么不启 `controller_server`（并把 `/cmd_vel` 让给它），要么让它只发 `/plan`、由 nav2 控制器跟踪。**两条链路同时发 `/cmd_vel` = 不定态**。
- 另外 `fake_vel_transform` 的 `spin_speed≠0` 会让 nav 看到的 `base_link_fake` **凭空旋转**而雷达不转（我们已记录：`docs/robot_models.md:321` 建议 `spin_speed:=0.0`；`spin_speed==0` 时 `base_link_fake ≡ base_link`）——**独立节点若直接用 `odom` 系做规划/控制，会绕过这套坐标系，行为与 nav2 侧不可比**。

### 5.2 生命周期顺序与"configure 期只做一次"的约束

- `nav2_lifecycle_manager`："take in a set of **ordered** nodes to transition one-by-one into the configuration and activate states… then bring down the stack in the **opposite order**"，并建立 bond 心跳，任一 server 失联就把整栈拉下去【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/>。⇒ **新插件的宿主 server 必须在 `node_names` 里，且顺序要保证 costmap 先起来**。
- 插件拿到 `Costmap2DROS` 共享指针后，**不要在 `configure()` 里假设 costmap 已经 available**：`planner_server::on_configure()` 先 `costmap_ros_->configure()` 再 `planner->configure(...)`；**`activate()` 阶段才 `costmap_ros_->activate()`**（`planner_server.cpp` 约 84–85、176 行）。真正的"地图有效"要等 `costmap_ros_->isCurrent()`——server 自己会 `waitForCostmap()`。
- **`get_plugin_type_param()` 读不到 `<id>.plugin` 会 `exit(-1)`**（§1.2）——参数写错不是"回落默认"，而是**整个 server 进程退出**。

### 5.3 costmap 的 frame 语义：高度/几何参数是**在 costmap 的 global frame 里量的**（本次从源码逐字核实）

`nav2_costmap_2d` 的 `ObservationBuffer` 把所有观测**先变换到 costmap 的 global frame**，然后才用 z 做高度带过滤：

```cpp
tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);   // observation_buffer.cpp:116
...
if ((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_) { ... } // :141-145
```

`global_frame_` 来自 `layered_costmap_->getGlobalFrameID()`【源码】`obstacle_layer.cpp:122` + `observation_buffer.cpp`（Humble 分支）。

⇒ **含义（对本文这个库直接相关）**：
- 我们的 **global costmap 的 global frame 是 `map`，local costmap 是 `odom`**；nav2 侧 `robot_base_frame` 在 sim 槽位里是 **`base_link_fake`**（【我们的实测/文档】`src/nav/rm_navigation/rm_navigation/params/nav2_params.yaml:172-173,239-240`（`global_frame: odom` / `map`）、`nav2_params_sim_base.yaml:86,129,285,414`（三处 costmap + behavior 均为 `robot_base_frame: base_link_fake`，注释写明"与 local/global costmap、controller 保持一致（哨兵云台系）"）；同族外部证据 `docs/cod_nav_macro_integration.md:187-188`）。⇒ **高度带、`robot_radius`/footprint、origin 全部是"该图自己那个 frame 下的量"**；把 global 的高度带参数抄到 local（或反之）本身就换了参考系。
- 我们已有同族教训：`docs/architecture.md:303` 把 `min/max_obstacle_height` 定性为"机器人碰撞体的高度范围"，而 local 的 `/scan` 是**相对车顶 `livox_frame` 的 z∈[−1.0, +0.1] 切片**；`docs/tilted_lidar_fidelity.md` 整篇记录的是"点云不在它自称的那个帧里 / odom 帧被转 30°"这个坑（§H1、§I.2）。**把库接进来时，"地图原点/分辨率/朝向"必须从 `costmap_ros` 读，不能从参数里另抄一份。**
- **对上游库的具体换算**（本次从源码核实）：`YAstar::transformPos(pos) = (pos - originPos) / mapping`（`yastar.cpp:883-886`）⇒ 库的 `originPos` **就是"格 (0,0) 在世界系里的坐标"**，和 nav2 的 `getOriginX()/getOriginY()` 语义**一致**；数组是 RowMajor `(height, width)`、按 `[y][x]` 索引（`isInObstacle(x,y)` 读 `occMap(y,x)`，`yastar.cpp:936-939`），和 nav2 `char map` 的 `map[y*size_x + x]` **几何上兼容**。
- ⚠️ **但值域不兼容（这是最容易踩的一脚）**：`YAstar::setMap(int w, int h, u_char* mapData)` 会把数据映射成 `x == 0 ? 1.f : 0.f`，即 **0 = 障碍、非 0 = 自由**（`yastar.cpp:404-417`），阈值 `occThs = 0.5f`（`yastar.hpp:309`）；而 nav2 的 master grid **0 = `FREE_SPACE`、254 = `LETHAL_OBSTACLE`、255 = `NO_INFORMATION`**（§1.4）。**直接把 `getCharMap()` 喂进去 = 整张图取反**（自由变障碍）。另一个重载 `setMap(..., std::vector<int8_t>& data)` 用 `x == 0 ? 0 : 1`（`yastar.cpp:418-430`），它符合"0 空闲"的 occupancy 直觉，**但会把 `-1`（unknown）当成障碍**。⇒ **必须自己写显式转换函数 + 一个单元测试**（§7.3 骨架里给了 `0/1` 映射）。
- `SfcSquare::Map = Eigen::Matrix<unsigned char, Dynamic, Dynamic, RowMajor>`，其 `setMap(int w,int h,u_char* data, mapping, originPos)` 也在**同一个 `u_char*` 约定**下工作（`_getBound`/`_getBoundSquareInplace` 里 `map(yint,xint) == 0` 被当作"该点不可扩展"，`sfcSquare.cpp:148,162-203`）——**也就是说 A\* 与 SFC 两侧的 0/1 约定必须一致地喂**，否则走廊会反着长。
- 还要注意**双重膨胀**：nav2 的 `inflation_layer` 已经把代价写进 master grid（253/254），而库自己还会 `initCostMap()`/`setCostField(radius, decay)` 再造一层势场（【源码】`yastar.hpp:175-185`、`README` 势场函数 `f(x)=0.5/(0.1+x)+1.0`）。**要么把 nav2 的膨胀算作"输入"，要么让库自己膨胀**——两次叠加会得到"墙比真实更厚"的行为，且与我们 `docs/` 里"`inflation 若调到 0.55/0.75，必须 ≥ robot_radius`"那条注意同源。

### 5.4 线程安全：读 costmap 与"图层在更新"是**真的并发**

- costmap 有**自己的更新线程**：`Costmap2DROS::on_configure()` 里 `map_update_thread_ = std::make_unique<std::thread>(...)`（【源码】`costmap_2d_ros.cpp:295`），循环在 `mapUpdateLoop()`（:456-481）里按 `update_frequency` 调 `updateMap()`；而 `planner_server`/`controller_server` 又把 costmap 放进 `nav2_util::NodeThread` 单独 spin（§1.4）。⇒ **规划回调与 costmap 更新天然是两个线程。**
- 正确做法：读主栅格时拿 `Costmap2D::getMutex()`（`std::recursive_mutex`）——nav2 自己就这么干（`isPathValid()`）、装图层时也这么干（"no update is allowed until the plugin is initialized"）。**"我先拷一份再算"是简单可靠的替代**（拷 168 KB 的时间远小于一次 A\*）。
- **上游已有的并发 bug（值得读，别重犯）**：nav2 issue **#6429 "Concurrent Access Bugs"**（Jazzy 分支 + ASan 复现）列出 4 个 race，其中 *Bug1* 就是 **`CostmapSubscriber` 只维护一份共享 `Costmap2D`：spin 线程用 `getCost()` 读，而 executor 线程处理新 costmap 时若尺寸变了就 `resizeMap()` 释放并替换同一个数组 ⇒ `heap-use-after-free`**；*Bug3* 是 `planner_server` 里"costmap 几何 + `planner_->potarr`"的 use-after-free【第三方/GitHub】<https://github.com/ros-navigation/navigation2/issues/6429>。
  - ⚠️ **Humble 是否同样中招：本次未能确证**。但 Humble 源码里 `CostmapSubscriber::getCostmap()`/`toCostmap2D()`/`resizeMap()` 的结构与 Jazzy 相同（`nav2_costmap_2d/src/costmap_subscriber.cpp` Humble 分支），**所以"通过 `CostmapSubscriber` 拿图（也就是 Smoother 形态）天然带着这个风险面"** ⇒ 若采用 Smoother 形态，**必须自己再加锁/加拷贝**，不要假设 nav2 帮你兜住。
- **锁序反转**：issue **#6461**（Lyrical 分支，ThreadSanitizer）报告 `costmap_2d_ros.cpp` 的 `_dynamic_parameter_mutex`（`mapUpdateLoop()` 持锁调 `LayeredCostmap::updateMap()` 的组合锁）、插件初始化持 costmap 锁、`updateParametersCallback()` 之间形成锁序环，可能死锁【第三方/GitHub】<https://github.com/ros-navigation/navigation2/issues/6461>。**Humble 1.1.20 源码里 `_dynamic_parameter_mutex` 与 `dynamicParametersCallback` 都存在，`mapUpdateLoop()` 同样持该锁调 `updateMap()`（`costmap_2d_ros.cpp:475-479`）**；但该 TSan 报告是针对新分支的，**不能据此声称 Humble 已验证有死锁**——只能说"同类代码结构在，风险面相同"。
- **实践建议**：插件里**不要在 `configure()` 里缓存 `Costmap2D*` 的坐标几何后长期使用**（issue #6429 的 Bug3 就是几何与内部数组不匹配），每次 `createPlan()` 现读 `getSizeInCellsX/Y()`、`getResolution()`、`getOriginX/Y()`。

### 5.5 `setPlan` 的重算节奏

- `setPlan()` 只在 `FollowPath` 动作开始/被抢占时调一次（§1.2）；**"多久重算一次全局路径"由行为树决定**（`ComputePathToPose` 被再 tick），不是由 controller 决定。nav2 的 smoother 教程里也提示了这一点：在 `PipelineSequence` 里如果把平滑结果写回 `{path}`，`FollowPath` 返回 `RUNNING` 时会重 tick 上游节点把 `path` 覆盖掉，建议平滑结果写到另一个黑板变量【文档】<https://docs.nav2.org/rolling/tutorials/general_tutorials/adding_smoother/adding_smoother/>。
- ⇒ 我们若把 min-snap 放在 Planner 里，**重算节奏 = BT 的重规划节奏**；放在 Controller 里则要注意：库里最坏 126.7 ms 的求解**不能放在每拍 `computeVelocityCommands()` 里**（会直接把控制环拖垮），必须在 `setPlan()` 时算好或用后台线程 + 双缓冲。
- `planner_server.planning_time` 的口径见 §1.2（含等待/校验），做 A/B 时要额外埋点区分。

### 5.6 保持"纯 C++"形态的 CMake 结构（有现成先例）

**先例 1（库 ROS-free + ROS 包 `find_package` 它）**：`ompl` 在本机 Humble 里就是**普通 CMake 包**（`/opt/ros/humble/share/ompl/package.xml` 里 `<build_type>cmake</build_type>`），而 `nav2_smac_planner` 用 `find_package(ompl REQUIRED)` + `<depend>ompl</depend>` 直接用它（§4.1）。本机共有 **10 个** ROS 2 包声明 `build_type=cmake`：`backward_ros cartographer foonathan_memory_vendor gmock_vendor gtest_vendor ignition_cmake2_vendor ignition_math6_vendor ompl ros_workspace urdfdom`【本机可复核】。
**先例 2（上游自己的做法）**：tdt-nav-kit 用 `scripts/setup.sh` 把 OSQP/OsqpEigen 构建安装到 `3rd/install`，`CMakeLists.txt` 用 `find_package(osqp REQUIRED)`/`find_package(OsqpEigen REQUIRED)` 消费【第三方】`README.md`、`CMakeLists.txt`。**同一套"前缀安装 + find_package"的套路就能用在它自己身上。**

⇒ 推荐结构（**库一行不改**）：

```
tdt_nav_kit/                     # 上游仓库（保持纯 CMake、无 ROS；可作为 submodule 或 vendored 目录）
  CMakeLists.txt                 # add_library(tdt_nav_kit STATIC ...) + install(EXPORT) + CTest
  src/YAstar/…  src/MinimumSnapOsqp/…
  test/  bench/                  # 纯 C++ 单测 + 上游 main.cpp 风格的 benchmark（无 ROS 也能跑）

tdt_nav_kit_ros/                 # 我们新增的 ament 包（唯一带 ROS 的地方）
  package.xml                    # <depend>tdt_nav_kit</depend> + <nav2_core plugin="..."/> 导出
  CMakeLists.txt                 # find_package(tdt_nav_kit REQUIRED) + 一个 SHARED 插件库
  global_planner_plugin.xml      # pluginlib 描述（base_class_type="nav2_core::GlobalPlanner"）
  include/tdt_nav_kit_ros/tdt_global_planner.hpp
  src/tdt_global_planner.cpp     # 薄适配：costmap→库、库→nav_msgs::Path
  src/standalone_node.cpp        # 可选：同一份适配代码再包一个独立节点（模式 B 的对照组）
```

### 5.7 三个"接口语义"级别的坑（本库特有）

1. **值域取反**（§5.3）：`getCharMap()` 必须显式映射，不能直传。
2. **`dt` 是隐式时标**（§2）：库的输出点间距 ≈ `dt = 0.1 s`（默认），一进 `Path` 就没有这个信息。要么把 `dt` 变成参数并在文档里写死契约，要么走 Controller 形态自己维护。
3. **`initCostMap()` 是硬前提**：库自己写着 *"调用 search 之前必须调用 `initCostMap()` 函数，或者其他的设置 costmap 的函数"*（`yastar.hpp:174-175`）——**忘了这一步不会报错，只会得到奇怪路径**。

---

## 6. 给我们的 bench 的推荐（含接入方式）

### 6.1 推荐形态：**"纯库 + 薄适配"两 target，先上 `GlobalPlanner` 插件**

理由（逐条对上证据）：
1. **契约最便宜**：Humble 的 `GlobalPlanner` 只有 `createPlan()` 一个函数有实际工作，其余 4 个可以留空（官方教程原话："The remaining methods are not used but it's mandatory to override them… we did override all but left them blank"）【文档】。
2. **costmap 拿的是指针**（不像 Smoother 走 `CostmapSubscriber` 拷贝 + 并发风险面，§1.4、§5.4）。
3. **库的输入形态天然匹配**：A\* 要"宽×高 + `u_char*` + 分辨率 + 原点"，正好是 `getSizeInCellsX/Y()` + `getCharMap()` + `getResolution()` + `getOriginX/Y()`（§5.3 已逐字核对语义与索引顺序）。
4. **纳秒~微秒级的插件开销 vs 毫秒~百毫秒级的库耗时**：插件形态不可能成为瓶颈（§3.2）。
5. **能直接复用我们已有的 A/B 能力**：新增 `planner:=tdt` 槽位（与 `planner:=navfn|smac2d` 并列），参数文件写成 `nav2_params_sim_planner_tdt.yaml`，用 `tools/scripts/regress/`（`nav_smoke_regression.py`、`plan_probe.py`、`nav_clearance_probe.py`、`compare_regress_snapshots.py`）做同一套判据的对照【我们的实测/文档】`tools/scripts/regress/`。

### 6.2 接入形态与代码骨架（Humble 精确签名）

```cpp
// tdt_nav_kit_ros/include/tdt_nav_kit_ros/tdt_global_planner.hpp
#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/node_utils.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "YAstar/yastar.hpp"                  // 纯 C++，无 ROS
#include "MinimumSnapOsqp/minimumSnap.hpp"    // 纯 C++，无 ROS

class TdtGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(                 // ← Humble：只有 start/goal
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  void syncCostmapToLibrary();                    // 锁 + 值域转换 + setMap

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;                              // 插件 id，用于 <id>.xxx 参数
  rclcpp::Logger logger_{rclcpp::get_logger("TdtGlobalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  YAstar astar_;                                  // 库对象：只在这里出现
  MinimumSnap min_snap_;
  std::vector<unsigned char> grid_;               // 每次同步复用的缓冲（避免每帧 new）
  double sample_dt_{0.1};                         // 库的隐式时标，变成显式参数
  double max_speed_{3.0}, max_acc_{1.0};
  double simplify_eps_{0.1};
  unsigned int cached_w_{0}, cached_h_{0};
};
```

```cpp
// 关键：costmap → 库（Humble 语义，逐条对应 §5.3 的坑）
void TdtGlobalPlanner::syncCostmapToLibrary()
{
  auto * cm = costmap_ros_->getCostmap();          // Costmap2D*，无拷贝（costmap_2d.hpp:224）
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lk(*(cm->getMutex()));  // 与更新线程互斥

  const unsigned int w = cm->getSizeInCellsX();
  const unsigned int h = cm->getSizeInCellsY();
  const unsigned char * master = cm->getCharMap();

  if (grid_.size() != static_cast<size_t>(w) * h) { grid_.assign(static_cast<size_t>(w) * h, 0xFF); }

  for (size_t i = 0; i < grid_.size(); ++i) {
    const unsigned char c = master[i];
    // nav2: 0=FREE_SPACE, 253=INSCRIBED, 254=LETHAL, 255=NO_INFORMATION  (cost_values.hpp:42-46)
    // 库的 u_char 重载: 0 == 障碍、非 0 == 自由  (yastar.cpp:404-417)
    const bool blocked =
      (c == nav2_costmap_2d::LETHAL_OBSTACLE) ||
      (c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) ||
      (c == nav2_costmap_2d::NO_INFORMATION && treat_unknown_as_obstacle_);
    grid_[i] = blocked ? 0x00 : 0xFF;
  }

  astar_.setMapping(static_cast<float>(cm->getResolution()));
  astar_.setOriginPos(static_cast<float>(cm->getOriginX()),
                      static_cast<float>(cm->getOriginY()));   // 库 = 格(0,0) 的世界坐标（yastar.cpp:883）
  astar_.setMap(static_cast<int>(w), static_cast<int>(h), grid_.data());
  astar_.setOccThs(0.5f);          // 库内部 occMap ∈ {0,1}
  astar_.initCostMap(true);        // 库自己的势场膨胀：别和 nav2 inflation 叠两次（§5.3）
  min_snap_.setMap(astar_.getMap(), static_cast<float>(cm->getResolution()),
                   cm->getOriginX(), cm->getOriginY());  // A* 与 SFC 的 0/1 约定必须一致
}
```

```cpp
// createPlan：前端 → 化简 → 后端 → nav_msgs::Path
nav_msgs::msg::Path TdtGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();   // 不要硬编码 "map"
  path.header.stamp = clock_->now();

  syncCostmapToLibrary();
  try {
    std::vector<Eigen::Vector2f> raw = astar_.search(
      {static_cast<float>(start.pose.position.x), static_cast<float>(start.pose.position.y)},
      {static_cast<float>(goal.pose.position.x),  static_cast<float>(goal.pose.position.y)});
    if (raw.empty()) { return path; }                        // 空 Path ⇒ server 会判无效
    auto simplified = astar_.simplifyPath(raw, static_cast<float>(simplify_eps_));

    MinimumSnap::SolveInput in;
    in.setPath(simplified);
    in.setTimeAllocated(MinimumSnap::trapezoidalTimeAllocation(
      simplified, static_cast<float>(max_speed_), static_cast<float>(max_acc_)));
    in.setInitVel({0.f, 0.f});
    in.setBackend(MinimumSnap::Backend::OSQPCorridor);       // fallback = Close
    auto out = min_snap_.solve(in);
    const auto & pts = out.success ? out.path : simplified;  // 后端失败就退回前端路径

    path.poses.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pts[i].x();
      ps.pose.position.y = pts[i].y();
      ps.pose.orientation.w = 1.0;
      // 可选：把库的隐式时标显式写出来（nav2 自带控制器会覆盖它，见 §2 第 4 条）
      // ps.header.stamp = path.header.stamp + rclcpp::Duration::from_seconds(i * sample_dt_);
      path.poses.push_back(ps);
    }
  } catch (const std::exception & e) {
    throw nav2_core::PlannerException(std::string("tdt planner failed: ") + e.what());
  }
  return path;
}
```

```cpp
// 文件末尾
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(TdtGlobalPlanner, nav2_core::GlobalPlanner)
```

参数段（Humble 键名，`planner_plugins` + `<id>.plugin`）：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    expected_planner_frequency: 1.0        # 库最坏 126.7 ms ⇒ 别设 20 Hz
    GridBased:
      plugin: "tdt_nav_kit_ros/TdtGlobalPlanner"   # Humble 用 '/'；Rolling 用 '::'
      sample_dt: 0.1
      max_speed: 3.0
      max_acc: 1.0
      simplify_eps: 0.1
      treat_unknown_as_obstacle: true
```

> **注意**：`sample_dt/max_speed/...` 是我们自己声明的参数，**必须**在用 `nav2_util::declare_parameter_if_not_declared(node, name_ + ".xxx", …)` 之后再读（官方教程的写法：`name_ + ".interpolation_resolution"`，`<插件id>.<参数>` 就是插件的命名空间）【文档】。

### 6.3 min-snap 后端"放哪"的最终建议（分阶段）

| 阶段 | 做什么 | 为什么 |
|---|---|---|
| **P0** | 库原样保留，加 `tdt_nav_kit` 的纯 CMake target + CTest + 上游风格 benchmark（**无 ROS 也要能跑**）；**Release/-O2 必须固定住** | 保住"库自己的测试/基准"，也是"纯 C++ 形态没被破坏"的可验证证据。上游 `CMakeLists.txt` 本身就写死 `set(CMAKE_BUILD_TYPE "Release")` + `-O2`；nav2 对 Ceres 也做同样的事（`# significant Ceres optimization speedup`，§4.1 引用） |
| **P1** | `GlobalPlanner` 插件（A\* 前端 + min-snap 后端 → `Path`），`sample_dt` 显式化 | 契约最便宜、可直接 A/B（`planner:=tdt`） |
| **P2** | A/B：`tdt` vs `navfn` vs `smac2d`，指标用现成 harness + §3.6 的埋点（**务必补 P95/P99**） | 结论必须来自我们机器上的数，而不是上游那张 avg/max 表 |
| **P3（可选）** | 若真需要"按时标执行"（过洞/上台阶时间窗）→ 再写 `nav2_core::Controller` 或独立节点，复用 P1 的适配层（同一份 `syncCostmapToLibrary`） | 见 §2.1：只有 Controller/独立节点能消费时标 |

**不建议**：一上来就做独立进程 + 共享内存（§4.4 证据最弱、复杂度最高、收益未被任何来源证明）。

---

## 7. 未能核实 / 明确"没有来源"的清单

1. **`docs.nav2.org/humble/` 不存在**（返回 404 的 "This page moved" 页）；`navigation.ros.org` 在新文档上线后不可用（本机 `curl` 连接失败/仅剩代理响应）。⇒ **Humble 版本的插件接口只能以本机 `1.1.20` 头文件 + `humble` 分支源码为依据**，本文已如此处理。
2. **pluginlib/`dlopen` 的具体加载耗时**：搜索到的只有机制说明（`class_loader` README、插件教程），**没有找到任何 µs/ms 级实测数字** ⇒ 列为"必须自测"（§3.6 第 1 条）。
3. **虚函数调用 vs 直接调用的实测差值**：无来源（量级推理已给，但未引用）。
4. **`nav_msgs::Path`（数百点）发布/序列化的实测耗时**：无来源；本文只给了按消息字段推算的 ≈72–80 B/pose、300 点 ≈21–24 KB【推算】。
5. **ROS 2 Humble + Fast DDS 环回 pub/sub 的权威延迟数字**：未取到。已知可引用的只有：rclcpp 比裸 DDS 最多 +50%（转引）、ros2_tracing 3.3 µs 自身开销、iceoryx 厂商宣称 <1 µs。
6. **nav2 issue #6429 / #6461 在 Humble 1.1.20 上是否可复现**：未验证。#6429 是 Jazzy + ASan，#6461 是 Lyrical + TSan；本文只核对了 **Humble 源码里同类结构存在**（`CostmapSubscriber` 的 `resizeMap`、`mapUpdateLoop` 持 `_dynamic_parameter_mutex` 调 `updateMap`），**不声称 Humble 已中招**。
7. **colcon 对 `package.xml` 里 `<build_type>cmake</build_type>` 的官方说明页**：未取到（`colcon.readthedocs.io` 的若干路径 404）。但**本机可复核**：Humble 安装里 10 个包（含 `ompl`、`cartographer`）就是这么声明的 ⇒ 结论仍成立。
8. **Rolling 文档与 Humble 的完整差异清单**：本文只标注了影响契约的几处（`createPlan` 参数、`plugins` vs `planner_plugins`、`setPlan` vs `newPathReceived`、path handler 插件类型）。未做逐页 diff。
9. **"独立进程 + SHM 跑规划器"的真实项目案例**：未找到（只有中间件层论文/文档）。因此 §0.1 里模式 C 的"何时选它"写成了"只有在已证明必要时"。
10. **上游库的 P95/P99 分位数**：上游只公布 avg/max（1000 次），无分位数。
11. **`MinimumSnap` 的时标到底"严格等间隔"吗**：本次读到的是"按 `dt` 累减采样 + 段末补点"（`minimumSnap.cpp:541-551`），⇒ **段边界处最后一个采样点可能短于 `dt`**；段长由 `timeAllocated` 决定。所以"点 k 的时刻 = k·dt"只是**近似约定**，要精确用必须自己按 `timeAllocated` 重建时标（或让上游开一个"输出 `(pos, t)` 对"的接口）。**这一点建议在 P0 阶段用单元测试钉死。**
12. **本文的"我们的实测/文档"引用均为读取现有 `docs/`、`src/`、`tools/` 文件所得；本次未新跑任何仿真，未改任何代码/配置/launch/其它文档。**

---

## 8. 参考来源汇总（按类型）

**上游库（第三方）**
- 仓库与 README（含性能表、运行参数表）：<https://github.com/T-DT-Algorithm-2026/tdt-nav-kit>（`README.md`）
- `src/YAstar/yastar.hpp|yastar.cpp`、`src/YAstar/kinodynamicAstar.hpp`、`src/MinimumSnapOsqp/minimumSnap.hpp|minimumSnap.cpp`、`src/MinimumSnapOsqp/sfcSquare.hpp|sfcSquare.cpp`、`main.cpp`、`CMakeLists.txt`、`doc/MinimumSnap.md`、`doc/Astar.md`、`doc/Usage.md`（同仓库 `main` 分支）

**nav2 源码（Humble 分支 = 我们的 1.1.20 同族）**
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_planner/src/planner_server.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_smoother/src/nav2_smoother.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_ros.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_subscriber.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/observation_buffer.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_navfn_planner/src/navfn_planner.cpp>、`.../nav2_navfn_planner/{CMakeLists.txt,global_planner_plugin.xml}`
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/{CMakeLists.txt,include/nav2_smac_planner/node_hybrid.hpp}>、<https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/path_handler.cpp>
- 并发问题：<https://github.com/ros-navigation/navigation2/issues/6429>、<https://github.com/ros-navigation/navigation2/issues/6461>

**本机安装的 nav2 1.1.20（`/opt/ros/humble/`，逐字核对，版本最准）**
- `include/nav2_core/{global_planner,smoother,controller,goal_checker,progress_checker,exceptions}.hpp`
- `include/nav2_costmap_2d/nav2_costmap_2d/{costmap_2d,costmap_2d_ros,layered_costmap,costmap_subscriber,cost_values}.hpp`
- `include/nav2_util/node_utils.hpp`
- `share/nav2_msgs/{action,msg}/*`、`share/nav_msgs/msg/{Path,OccupancyGrid}.msg`、`share/geometry_msgs/msg/*`、`share/trajectory_msgs/msg/*`
- `share/{nav2_navfn_planner,nav2_smac_planner,nav2_theta_star_planner}/*.xml`、`share/cartographer/package.xml`、`share/ompl/package.xml`（`build_type=cmake` 的 10 个包）

**nav2 官方文档（Rolling；Humble 页面已下线，差异见 §1）**
- 插件教程（planner / controller / smoother）：<https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/>、<https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_controller_plugin/writing_new_controller_plugin/>、<https://docs.nav2.org/rolling/tutorials/general_tutorials/adding_smoother/adding_smoother/>
- 插件总目录：<https://docs.nav2.org/rolling/configuration_and_development/navigation_plugins/>
- **性能章节（RMW/组合/IPC 的 CPU 实测表）**：<https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/>
- 服务端参数：<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_planner_server/>、`.../configuring_smoother_server/`、`.../configuring_velocity_smoother/`、`.../configuring_lifecycle_manager/`
- profiling 教程：<https://docs.nav2.org/rolling/tutorials/general_tutorials/get_profile/get_profile/>

**ROS 2 官方**
- class_loader / pluginlib 机制：<https://github.com/ros/class_loader>
- Executors（含"不适合实时"的原文）：<https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html>
- Callback groups：<https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html>
- intra-process 设计（≥5 KB 规则、仍不如非 ROS 共享内存）：<https://design.ros2.org/articles/intraprocess_communications.html>
- real-time 背景：<https://design.ros2.org/articles/realtime_background.html>
- `rclcpp::LoanedMessage`：<https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/loaned_message.hpp>
- 论文：ros2_tracing <https://arxiv.org/abs/2201.00393>；ROS 2 通信实时性评估 <https://arxiv.org/abs/1809.02595>、<https://arxiv.org/abs/1808.10821>
- 中间件：Fast DDS SHM <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html>；iceoryx <https://iceoryx.io/classic/v2.0.0/getting-started/what-is-iceoryx/>

**真实工程案例（第三方）**
- HWSentryNav26（浙大 Hello World，**不用 nav2** 的哨兵导航）：<https://github.com/Polyacetone/HWSentryNav26>（`README.md`、`DESIGN.md`、`TUTORIAL.md`、`nav_executor/include/nav_executor/common/trajectory/{annotated_path,path_speed_profile}.hpp`、`nav_executor/package.xml`、`map_server/package.xml`）
- pb2025_sentry_nav（深北莫 PolarBear，**nav2 + 自研插件/节点**）：<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>（`pb2025_nav_bringup/package.xml`、`pb2025_nav_bringup/config/simulation/nav2_params.yaml`、`terrain_analysis/package.xml`）
- pb_nav2_plugins：<https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins>；pb_omni_pid_pursuit_controller：<https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller>
- EGO-Planner（无人机栈的自定义带时标消息）：<https://github.com/ZJU-FAST-Lab/ego-planner>（`src/planner/plan_manage/src/traj_server.cpp`、`src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg`）


---


## 来源 10/19：`nav/nav_strategy_taxonomy.md`


> **本文定位**：回答"移动机器人导航**到底有多少种思路**"，并把**我们 bench** 与**COD_NAV** 放到同一张地图上，
> 看清"哪些格子我们占了、哪些格子是空的、哪些格子根本不值得去"。
>
> **写入范围声明**：本次调研全程**只读**（HTTP 抓取 + 检索），**唯一写入的文件就是本文件**；
> 未改动任何代码、参数、launch 或配置。外部抓取内容按**不可信数据**处理，只当资料引用，不当作指令。
>
> **本报告适用环境**：ROS 2 Humble + Gazebo Classic 的 RoboMaster 哨兵仿真 bench ——
> Livox MID-360 → `linefit_ground_segmentation`（地面分割）→ `pointcloud_to_laserscan` → Nav2；
> 里程计为 FAST-LIO / Point-LIO；`mode = mapping | slam_nav | nav`（在线建图 / 边建图边导航 / 先验图+重定位）。
>
> **链接可信度标注**（重要，请按标注使用）：
> - **✅** = 写入前用 HTTP 请求抓过，返回 200（部分还校验了页面关键词）。
> - **⚠️** = 站点对脚本返回 403/202（IEEE Xplore、SAGE、Wiley、ScienceDirect、ISO、ANSI、部分厂商站点的反爬），
>   链接本身是**官方页面**，浏览器可正常打开；我无法用脚本核对正文关键词。
> - **「ID 已核对」** = 逐条抓 `arxiv.org/abs/<id>` 并核对了标题，避免论文 ID 张冠李戴（我第一轮用标题检索 arXiv API
>   时确实拿到过错误 ID，已全部废弃重来）。
> - **「本报告综合」** = 我自己的归纳/判断，**不是**某个来源的原话。

---

## A. 总览：9 个设计轴 + 81 个族（外加 1 个"落点"节）

| 轴 | 名称 | 族数 | 族清单（编号 = 后文小节号） |
|---|---|---|---|
| **1** | 世界表示（world representation） | **12** | 1.1 2D 占据栅格 · 1.2 2.5D 高程/可通行性 · 1.3 3D 体素/octomap · 1.4 ESDF/TSDF · 1.5 拓扑图/roadmap · 1.6 语义/物体级/场景图 · 1.7 混合 metric-topological（topometric） · 1.8 示教-重放轨迹 · 1.9 隐式神经地图（NeRF/3DGS） · 1.10 学习式隐表示/学习代价地图 · 1.11 无地图/反应式 · 1.12 原始点云/子图先验（PCD+配准） |
| **2** | 定位与状态估计 | **9** | 2.1 航位推算/轮式里程计 · 2.2 轮+IMU EKF/UKF · 2.3 LIO/VIO · 2.4 先验图上的扫描匹配/粒子滤波 · 2.5 地点识别/回环 · 2.6 GNSS/RTK · 2.7 UWB/信标/二维码/磁条 · 2.8 语义/地标锚点 · 2.9 **不做重定位**（静态 `map→odom` / 手工初值） |
| **3** | 全局路径 / 任务级规划 | **10** | 3.1 图搜索（Dijkstra/A\*/D\* Lite/Theta\*） · 3.2 Hybrid A\*/state lattice（Smac） · 3.3 NavFn/势场栅格 · 3.4 采样式（RRT/RRT\*/PRM/BIT\*） · 3.5 轨迹优化（CHOMP/STOMP/TrajOpt/GPMP2/TEB） · 3.6 覆盖规划 · 3.7 前沿/探索规划 · 3.8 拓扑路由 · 3.9 多目标/TSP 排序 · 3.10 MPC 式全局规划 |
| **4** | 局部规划与控制 | **11** | 4.1 Pure Pursuit · 4.2 Stanley · 4.3 Regulated Pure Pursuit · 4.4 DWA/DWB · 4.5 TEB · 4.6 MPPI · 4.7 MPC（acados/casadi/`mpc_local_planner`） · 4.8 RL 局部策略 · 4.9 速度平滑/指令整形 · 4.10 运动学模型（差速/全向/Ackermann/腿足） · 4.11 反应式避障（势场/VFH） |
| **5** | 任务指定方式 | **10** | 5.1 无目标（探索） · 5.2 单目标 · 5.3 航点序列/through-poses · 5.4 巡逻环线 · 5.5 覆盖任务 · 5.6 示教-重放 · 5.7 语义/自然语言指令 · 5.8 示教学习（LfD/IL） · 5.9 行为树/FSM 任务层 · 5.10 任务/车队协议 |
| **6** | 不确定性与安全 | **7** | 6.1 未知区策略 · 6.2 重规划与恢复行为 · 6.3 碰撞监控/安全监控/限速 · 6.4 风险感知与机会约束 · 6.5 主动 SLAM/不确定性驱动探索 · 6.6 安全标准与"安全链独立" · 6.7 安全型传感器与安全 PLC |
| **7** | 学习 / 端到端 / 基础模型 | **8** | 7.1 RL 导航 · 7.2 基准与仿真器 · 7.3 模仿学习 · 7.4 Sim-to-real · 7.5 基础模型导航 · 7.6 世界模型 · 7.7 扩散策略 · 7.8 LLM/VLM 任务规划 |
| **8** | 工业 AGV/AMR 范式 | **8** | 8.1 磁条/磁带 · 8.2 二维码/标记 · 8.3 反射板激光三角定位 · 8.4 示教-重放 AGV · 8.5 SLAM AMR · 8.6 车队管理 · 8.7 VDA5050 · 8.8 为什么工业偏好固定路径+拓扑图 |
| **9** | 多机/协同 | **6** | 9.1 编队控制 · 9.2 任务分配 · 9.3 多机 SLAM/地图融合 · 9.4 多机前沿探索 · 9.5 通信约束下的协同 · 9.6 ROS 2 多机部署 |
| **10** | **RM 哨兵生态落点**（不是设计轴，是"坐标系落点"） | 10 个仓库 | 见 §B.10 |
| | **合计** | **9 轴 / 81 族** | |

**先给结论，避免误读（本报告综合）**：
轴与族是**"词汇表"**，不是**"可自由排列组合的菜单"**。81 个族里绝大多数组合在工程上不成立
（例如"磁条导引 + NeRF 建图"、"无重定位 + 覆盖规划 + 高速动态避障"）。
真实产品/强队配置收敛到 **5~8 种标准组合**（§D），其余族要么是研究前沿，要么是某个组合里的**可替换插槽**。
`docs/algorithm_axes.md` 里"120 组合"的说法也是同一个意思：**组合数是插槽相乘的假象，不是 120 个独立方案**。

### 计数小结（直接回答"到底有多少种思路"）

- **轴数**：**9 个设计轴**（世界表示 / 定位与状态估计 / 全局规划 / 局部控制 / 任务指定 / 不确定性与安全 / 学习与基础模型 / 工业 AGV-AMR 范式 / 多机协同），
  外加第 10 节作为"**RM 哨兵生态落点**"（不是设计轴，是别人已经站在哪一格）。
- **族数**：**81 族**（12 + 9 + 10 + 11 + 10 + 7 + 8 + 8 + 6 = 81；可用 `grep -cE '^\*\*[0-9]+\.[0-9]+ ' docs/nav_strategy_taxonomy.md` 复核，
  本文件当前正好 81 条族条目）。
- **"能实际选的路"数**：**5~8 种标准产品化组合**（§D 给出 8 种；其中 D1–D3 是量产/强队主流，去掉研究向的 D6/D7 与多机 D8，剩下的就是 **5 种**）。
- **单机比赛可用的"现实自由度"（本报告综合）**：约 **2~3 种组合 × 3~4 个插槽** ——
  即"在线建图边建边导航（D4）"或"先验图+重定位（D2/D3）"二选一，再在【里程计 / 重定位 / 全局规划器 / 局部控制器 / 任务层】里换实现。
  **不是 81 选 1，也不是任意排列组合。**

---

## B. 逐轴展开

> 每族的固定四行：**是什么** / **何时用** / **ROS(2) 实现** / **来源**。
> 「来源」里每条链接都带 ✅/⚠️ 标注；族内只要 ≥1 条可点链接即满足"每族有权威来源"。

### B.1 轴 1：世界表示（12 族）

**1.1 2D 占据栅格（occupancy grid / costmap）**
- **是什么**：把世界压成 `resolution≈0.03~0.05 m` 的二维概率栅格（free / occupied / unknown），再叠加膨胀、体素投影、禁行区等"代价层"；导航栈的事实标准。
- **何时用**：场地平坦、障碍物贴地、机器人是 2D 运动学；**几乎所有 2D 导航栈的默认选择**，也是 RM 场地（平地+矮墙）最划算的表示。
- **ROS 2 实现**：`nav2_costmap_2d`（static/obstacle/inflation/voxel 层）+ `nav2_map_server`；ROS 1 对应 `map_server`+`costmap_2d`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/static/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/map_server/configuring_map_server/>

**1.2 2.5D 高程 / 可通行性图（elevation / traversability）**
- **是什么**：2.5D = 每个 2D 格子存一个高度（或高度带、坡度、台阶高度、粗糙度），从"障碍/非障碍"升级为"能不能过、以什么代价过"。
- **何时用**：有斜坡/台阶/减速带/可越障矮台；腿足与越野机器人必备；RM 场地里对应"高地、斜坡、能量机关台阶"。
- **ROS 2 实现**：`elevation_mapping`（ANYbotics）、`elevation_mapping_cupy`（GPU 版）、`grid_map`；RM 生态常用 `terrain_analysis` / `terrain_analysis_ext`（把离地高度写进点云 `intensity`，再喂 costmap 层）。
- **来源**：✅ <https://github.com/ANYbotics/elevation_mapping> ；✅ <https://github.com/leggedrobotics/elevation_mapping_cupy> ；✅ <https://github.com/ANYbotics/grid_map> ；✅ <https://github.com/HongbiaoZ/autonomous_exploration_development_environment> ；✅ <https://github.com/fabioruetz/wild_visual_navigation>（学习式可通行性，与 1.10 共用）；✅ arXiv 2305.08510「ID 已核对」

**1.3 3D 体素 / octomap（含时间维）**
- **是什么**：八叉树或体素栅格存三维占据概率，能表达悬空结构（横梁、桌下）、并能按高度带切层投影给 2D 规划器。
- **何时用**：三维结构重要、点云多源（雷达+深度相机）、需要"障碍随时间消失"（动态障碍残影）。
- **ROS 2 实现**：`octomap`（3D 概率八叉树）；Nav2 `voxel_layer`（costmap 的 3D 体素层）；`spatio_temporal_voxel_layer`（STVL，带时间衰减，COD 与我们都在用）。
- **来源**：✅ <https://octomap.github.io/> ；✅ <https://github.com/octomap/octomap> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/> ；✅ <https://github.com/SteveMacenski/spatio_temporal_voxel_layer>

**1.4 ESDF / TSDF（欧氏符号距离场 / 截断符号距离场）**
- **是什么**：不存"是否占据"，而存"到最近障碍的距离与梯度"，规划器可直接做梯度下降、碰撞代价连续可微。
- **何时用**：需要平滑、可微、考虑机器人尺寸的轨迹优化（无人机、机械臂、腿足）；纯 2D 栅格导航用不上。
- **ROS 2 实现**：`voxblox`（TSDF→ESDF，ROS 1 为主，有 ROS 2 分支/衍生）、`nvblox`（NVIDIA GPU 版 TSDF/ESDF，ROS 2）、`FIESTA`（增量式 ESDF，无人机）。
- **来源**：✅ <https://github.com/ethz-asl/voxblox> ；✅ <https://github.com/nvidia-isaac/nvblox> ；✅ <https://github.com/HKUST-Aerial-Robotics/FIESTA> ；✅ <https://github.com/ethz-asl/mav_voxblox_planning> ；✅ arXiv 1611.03631（voxblox 论文，**ID 已核对**）

**1.5 拓扑图 / roadmap（图结构表示）**
- **是什么**：世界被抽象成"节点+边"（路口、房间、增益点）+ 可通行代价，不存几何细节；PRM 是最经典的 roadmap 构造法。
- **何时用**：大范围、结构化环境；需要"任务级"决策（去哪、按什么顺序）；地图维护成本低。
- **ROS 2 实现**：OMPL（PRM/PRM\*/RRT 家族的库，也被 MoveIt 等复用）；Nav2 `nav2_route` 的 route graph；SBPL（搜式规划库，lattice/图搜索）。
- **来源**：✅ <https://ompl.kavrakilab.org/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/> ；✅ <https://github.com/sbpl/sbpl>

**1.6 语义 / 物体级地图 / 3D 场景图**
- **是什么**：地图里带上"这是什么"（门、箱子、增益点、敌人）以及物体间关系（场景图），供语言指令与任务推理使用。
- **何时用**：需要按语义找目标（"去补给点"）、人机协作、长期运维；算力与标注成本高，比赛里通常只在研究 Demo 阶段。
- **ROS 2 实现**：`Hydra`（实时场景图）、`Kimera-Semantics`（度量-语义）、`ConceptGraphs`（开放词表 3D 场景图）。
- **来源**：✅ <https://github.com/MIT-SPARK/Hydra> ＋ ✅ arXiv 2201.13360（Hydra 论文，**ID 已核对**）；✅ <https://github.com/MIT-SPARK/Kimera-Semantics> ＋ ✅ arXiv 1910.02490；✅ <https://github.com/concept-graphs/concept-graphs> ＋ ✅ arXiv 2309.16650；✅ arXiv 2501.05750（室内具身语义建图综述，**ID 已核对**）

**1.7 混合 metric-topological（topometric）**
- **是什么**：底层保留度量栅格（局部避障、精确到位），上层挂拓扑图/路线（全局决策、跨区导航）——"两层地图"。
- **何时用**：大场景 + 需要重复精度；工业 AMR 与"示教路线"产品的常见内部结构；也是 Nav2 route graph 的设计哲学。
- **ROS 2 实现**：Nav2 `nav2_route`（route graph + costmap 混合）；STRANDS `topological_navigation`（ROS 1 经典实现，ROS 2 生态仍有移植）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/> ；✅ <https://github.com/strands-project/strands_navigation> ；✅ arXiv 2501.05750（topometric 术语与综述语境）；⚠️ <https://www.annualreviews.org/content/journals/10.1146/annurev-control-032724-020548>（"Local Maps Are All You Need: A Review of Topometric Teach and Repeat Navigation"，Annual Review of Control, Robotics, and Autonomous Systems）

**1.8 示教-重放轨迹（teach-and-repeat route representation）**
- **是什么**：地图不是栅格，而是**一条或多条被示教过的轨迹/路线**（含关键帧、局部子图、路口标记），运行时沿路线重放并在局部做避障。
- **何时用**：路线固定、环境外观会变（光照/季节/人流）、不想做全局度量建图；巡检、农业、工业送料。
- **ROS 2 实现**：`norlab-ulaval/wiln`（激光 teach-and-repeat 框架，面向恶劣天气户外）；ROS 1 时代有 `teach_repeat_*`、route-following 系列包；Nav2 里可用 `nav2_route` + 录制路径近似实现。
- **来源**：✅ <https://github.com/norlab-ulaval/wiln> ；⚠️ Annual Reviews topometric teach-and-repeat 综述（同上）；⚠️ <https://onlinelibrary.wiley.com/doi/10.1002/rob.20342>（Furgale & Barfoot, "Visual Teach and Repeat for Long-Range Rover Autonomy", JFR 2010）

**1.9 隐式神经地图（NeRF / 3D Gaussian Splatting）**
- **是什么**：用神经网络或 3D 高斯基元表示场景的密度/颜色/几何，可渲染出照片级视图，并从中导出安全走廊/占据查询。
- **何时用**：研究阶段；需要视觉逼真重建 + 在其中做规划（无人机穿越、室内语义导航）；实时性与内存是硬约束。
- **ROS 2 实现**：Splat-Nav（3DGS 地图上的安全实时导航，论文+仓库）；MonoGS（单目高斯泼溅 SLAM）；NeRF 本身只解决重建与渲染，不直接给规划接口。
- **来源**：✅ arXiv 2403.02751（Splat-Nav，**ID 已核对**）；✅ <https://github.com/muskie82/MonoGS> ；✅ arXiv 2003.08934（NeRF 原始论文，**ID 已核对**）

**1.10 学习式隐表示 / 学习代价地图**
- **是什么**：不显式定义"障碍"，而是用学习模型输出代价/可通行性/隐状态（latent），供规划器或策略消费；常以自定义 costmap 层或独立评分网络落地。
- **何时用**：语义/社交场景（人的舒适距离、草地可通行性）、传感器噪声大、规则难以手写代价时；可解释性与安全验证是弱点。
- **ROS 2 实现**：Nav2 插件机制里写自定义 costmap layer / controller critic（官方有插件教程）；研究实现如 Wild Visual Navigation（自监督可通行性）、Learning Social Cost Functions（人机共处代价函数）、DreamerV3（隐空间世界模型）。
- **来源**：✅ <https://github.com/fabioruetz/wild_visual_navigation> ＋ ✅ arXiv 2305.08510；✅ arXiv 2407.10547（Learning Social Cost Functions，**ID 已核对**）；✅ arXiv 2301.04104（DreamerV3，**ID 已核对**）

**1.11 无地图 / 反应式（mapless / reactive）**
- **是什么**：不维护任何全局地图，只靠当前传感器做反应式决策：势场法、VFH 直方图、Bug 系列算法、沿墙走。
- **何时用**：极低成本、极简场景（扫地机早期、循线车）、或作为大栈的兜底安全层；没有全局最优性，会困在局部极小/长死胡同。
- **ROS 2 实现**：Nav2 层面可用 `nav2_rotation_shim_controller` + 简单控制器近似；ROS 1 生态有 `voxel_grid`/`VFH` 类包；多数团队直接自写。
- **来源**：⚠️ <https://journals.sagepub.com/doi/10.1177/027836498600500106>（Khatib 1986 人工势场，IJRR）；⚠️ <https://ieeexplore.ieee.org/document/88148>（Borenstein & Koren, VFH, IEEE T-RA 1991）；⚠️ <https://dl.acm.org/doi/abs/10.1007/BF01840369>（Lumelsky & Stepanov, Bug 算法, Algorithmica 1987）

**1.12 原始点云 / 子图先验（PCD + 配准，不经栅格化）**
- **是什么**：先验地图就是一份（或几份）**点云/PCD**，定位靠直接配准（GICP/ICP/NDT），导航再临时投影出局部栅格。
- **何时用**：有离线建好的 3D 点云但不想维护 2D 图；需要 3D 精度；也是 RM 强队常见套路（"PCD 先验 + ICP 重定位"）。
- **ROS 2 实现**：`small_gicp`（快速 GICP，常被 RM 队用作重定位后端）、PCL 的 NDT/ICP、`icp_registration`（社区常用 PCD 配准节点）。
- **来源**：✅ <https://github.com/koide3/small_gicp> ；✅ <https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization> ；✅ <https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_distributions_transform.html>（PCL NDT 官方教程）；✅ <https://github.com/PointCloudLibrary/pcl>

### B.2 轴 2：定位与状态估计（9 族）

**2.1 航位推算 / 轮式里程计**
- **是什么**：由编码器转速积分出位姿；误差随时间无界增长（打滑、轮径误差）。
- **何时用**：短时局部控制、EKF 的预测项、LIO 失效兜底；**绝不能单独承担全局定位**。
- **ROS 2 实现**：`ros2_control` 的 `diff_drive_controller`（发布 `odom→base_link` 与 `/odom`）。
- **来源**：✅ <https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html>

**2.2 轮 + IMU 的 EKF/UKF 融合**
- **是什么**：以 EKF/UKF 融合轮速、IMU、（可选）GPS/视觉，输出平滑的 `odom→base_link` 与协方差。
- **何时用**：没有 LIO/VIO 或需要高频鲁棒里程计时；工业 AMR 的标配，也是 LIO 的对照基线。
- **ROS 2 实现**：`robot_localization`（`ekf_node` / `ukf_node` / `navsat_transform_node`）。
- **来源**：✅ <https://github.com/cra-ros-pkg/robot_localization> ；✅ <https://link.springer.com/chapter/10.1007/978-3-319-27146-0_25>（Moore & Stouch, "A Generalized Extended Kalman Filter Implementation for the Robot Operating System"）

**2.3 LIO / VIO（激光/视觉惯性里程计）**
- **是什么**：紧耦合 LiDAR/相机 + IMU，直接配准建图并输出高频高精度里程计；RM 哨兵当前主流（FAST-LIO / Point-LIO / Small Point-LIO）。
- **何时用**：需要 3D 精度、点云配准、无 GPS 的室内外场景；退化场景（长走廊、空旷场地）与 IMU 初始化是主要风险点。
- **ROS 2 实现**：FAST-LIO / FAST-LIO2、Point-LIO、LIO-SAM、VINS-Fusion（视觉惯性）、OpenVINS。
- **来源**：✅ <https://github.com/hku-mars/FAST_LIO> ＋ ✅ arXiv 2107.06829（FAST-LIO2，**ID 已核对**）；✅ <https://github.com/hku-mars/Point-LIO> ；✅ <https://github.com/TixiaoShan/LIO-SAM> ＋ ✅ arXiv 2007.00258；✅ <https://github.com/HKUST-Aerial-Robotics/VINS-Fusion> ；✅ <https://github.com/rpng/open_vins>

**2.4 先验图上的扫描匹配 / 粒子滤波（重定位）**
- **是什么**：在**已有地图**上求 `map→odom`：粒子滤波（AMCL）、点云配准（NDT/GICP/ICP）、或图优化式定位（Cartographer 纯定位、slam_toolbox localization 模式）。
- **何时用**：有先验图、要重复精度、要"关机再开机仍在同一坐标系"；是 `mode:=nav` 的核心环节。
- **ROS 2 实现**：Nav2 `nav2_amcl`；`cartographer_ros`（`-load_state` 纯定位）；`slam_toolbox`（localization 模式）；`small_gicp` / `icp_registration`（3D 配准）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/> ；✅ <https://google-cartographer-ros.readthedocs.io/en/latest/> ＋ ✅ <https://github.com/cartographer-project/cartographer_ros> ；✅ <https://github.com/SteveMacenski/slam_toolbox> ＋ ✅ <https://joss.theoj.org/papers/10.21105/joss.02783>（slam_toolbox 论文, JOSS）；✅ <https://github.com/koide3/small_gicp>

**2.5 地点识别 / 回环（place recognition & loop closure）**
- **是什么**：用外观/结构描述子判断"我来过这里"，触发回环约束修正累计漂移；也是"重定位到先验图某处"的基础设施。
- **何时用**：大规模、长时间、重复路线；单次短程比赛可不用，但**没有它就没有全局一致性**。
- **ROS 2 实现**：DBoW2/DBoW3（词袋，视觉）、Scan Context / Scan Context++（3D 点云描述子）；常嵌在 Cartographer/LIO-SAM/slam_toolbox 内部。
- **来源**：✅ <https://github.com/dorian3d/DBoW2> ；✅ <https://github.com/gisbi-kim/scancontext_tro> ＋ ✅ arXiv 2109.13494（Scan Context++，**ID 已核对**）

**2.6 GNSS / RTK**
- **是什么**：卫星定位（RTK 固定解可达厘米级），提供全局绝对坐标，但室内/遮蔽环境失效。
- **何时用**：户外大场景、园区物流；室内比赛场地无信号，只能当"可选传感器槽"。
- **ROS 2 实现**：`robot_localization` 的 `navsat_transform_node`（把经纬度转 `map` 系并融合进 EKF）；Nav2 的 GPS 跟随教程（把 GPS 路径点转成 Nav2 目标）；RTKLIB（开源 RTK 解算）。
- **来源**：✅ <https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_gps/navigation2_with_gps/> ；✅ <https://github.com/cra-ros-pkg/robot_localization> ；✅ <http://www.rtklib.com/>

**2.7 UWB / 信标 / 二维码 / 磁条**
- **是什么**：靠**人工布置的基础设施**定位：UWB 基站测距、二维码/标签（ArUco/AprilTag）绝对位姿、磁条/磁钉循迹。
- **何时用**：室内工业、可改造场地、需要"绝对且可认证"的定位（反射板/磁条可做安全型定位）；缺点是**必须布场**。
- **ROS 2 实现**：`apriltag_ros` / OpenCV ArUco（视觉标签）；各厂商 UWB/磁条驱动多为私有；磁条与反射板见 §B.8。
- **来源**：✅ <https://github.com/AprilRobotics/apriltag> ；⚠️ <https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html>（OpenCV ArUco 官方教程）；磁条/反射板官方资料见 8.1/8.3

**2.8 语义 / 地标锚点**
- **是什么**：把地图里的语义物体（门牌、标牌、充电桩、增益点）当作长期地标，用检测结果约束位姿。
- **何时用**：几何退化（长走廊、空旷）时提供额外约束；与 1.6 语义地图配套。
- **ROS 2 实现**：无"官方一站式"实现；通常自写节点把检测位姿作为 `PoseWithCovariance` 送进 EKF/因子图（GTSAM/Ceres）。
- **来源**：✅ arXiv 1606.05830（Cadena et al., "Past, Present, and Future of SLAM"，**ID 已核对**，语义 SLAM 章节）；✅ <https://github.com/AprilRobotics/apriltag> ；✅ <https://github.com/concept-graphs/concept-graphs>

**2.9 不做重定位：静态 `map→odom` / 手工初值** ⭐（COD_NAV 2026 就在这一格）
- **是什么**：不估计 `map→odom`，而是**发布一个静态变换**（或依赖上一次位姿/手工对齐），把 `odom` 当成 `map` 用。
- **何时用**：在线 SLAM 一直开着（`map` 随车实时重建，坐标原点固定在同一处）、单场次短时任务、或"每局重开+人工摆位"；**代价是无法恢复丢失的全局位姿**。
- **ROS 2 实现**：`tf2_ros static_transform_publisher`（COD `rmul2026` 分支正是如此）；Nav2 侧无 `amcl`/无 localization 节点。
- **来源**：✅ <https://github.com/qza36/COD_NAV> ；✅ <https://gitee.com/codnavgation/cod_-rm2026_-navigation>（README 明确"slam_toolbox 同时导航建图"+静态 TF 设计）

### B.3 轴 3：全局路径 / 任务级规划（10 族）

**3.1 图搜索（Dijkstra / A\* / D\* Lite / Theta\*）**
- **是什么**：在离散图上做最优搜索；A\* 用启发式加速，D\* Lite 支持增量重规划（环境变化时只修局部），Theta\* 允许任意角度（不贴格）。
- **何时用**：栅格全局规划的基本盘；D\* Lite 适合地图持续变化；Theta\* 适合"讨厌锯齿路径"的场合。
- **ROS 2 实现**：Nav2 `nav2_navfn_planner`（NavFn，Dijkstra/势场式）、`nav2_theta_star_planner`（Theta\*）；ROS 1 `global_planner`（A\*/Dijkstra 可切换）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/> ；✅ arXiv 1401.3843（Theta\*，**ID 已核对**）；✅ <http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf>（D\* Lite 作者页 PDF）

**3.2 Hybrid A\* / state lattice（Nav2 Smac 家族）**
- **是什么**：在满足车辆运动学（最小转弯半径、前进/倒车）的离散动作空间里搜索，输出**可被底盘跟踪**的平滑路径；state lattice 是其推广（预生成动作图）。
- **何时用**：Ackermann/差速非全向机器人、需要倒车/掉头、路径必须可执行；全向底盘可退化用 2D 版本。
- **ROS 2 实现**：Nav2 `SmacPlanner2D` / `SmacPlannerHybrid`（Dubins/Reeds-Shepp）/ `SmacPlannerLattice`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner>

**3.3 NavFn / 势场式栅格规划**
- **是什么**：在 costmap 上传播"导航函数"（势场/波前），从目标反向扩散，路径沿梯度下降；实现简单、稳定、参数少。
- **何时用**：平地、障碍不复杂、要"够用就行"；缺点是路径贴障碍、转弯生硬、狭窄处易贴边。
- **ROS 2 实现**：Nav2 `NavfnPlanner`（`nav2_navfn_planner`）；ROS 1 `navfn` / `global_planner`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner>

**3.4 采样式规划（RRT / RRT\* / PRM / BIT\* / Informed RRT\*）**
- **是什么**：随机采样 + 图/树连接，高维与复杂约束下最通用；RRT\* 系列保证渐近最优，BIT\* 用启发式批处理加速收敛。
- **何时用**：高维（机械臂、腿足）、复杂约束、非结构化空间；2D 平地导航里通常不如 A\*/Smac 划算。
- **ROS 2 实现**：OMPL（RRT/RRT\*/PRM/BIT\* 等的参考实现库）；Nav2 默认不含采样式全局规划器（需自写插件或用 MoveIt 那套）。
- **来源**：✅ <https://ompl.kavrakilab.org/> ；✅ arXiv 1105.1186（RRT\*，**ID 已核对**）；✅ arXiv 1404.2334（Informed RRT\*，**ID 已核对**）；✅ arXiv 1405.5848（BIT\*，**ID 已核对**）

**3.5 轨迹优化式规划（CHOMP / STOMP / TrajOpt / GPMP2 / TEB）**
- **是什么**：把路径当连续轨迹，最小化"平滑+避障+动力学"代价（梯度法/随机法/凸优化/高斯过程）。
- **何时用**：机械臂与需要高阶平滑的移动平台；**移动机器人上最常见的落地形态其实是局部 TEB/MPC（见 4.5/4.7）**，全局轨迹优化较少见。
- **ROS 2 实现**：MoveIt 2（CHOMP/STOMP 等规划器，主要面向机械臂）、TrajOpt（tesseract 生态）、GTSAM（GPMP2 所在库）、`teb_local_planner`（移动端）。
- **来源**：✅ <https://moveit.picknik.ai/main/index.html> ；✅ <https://github.com/moveit/moveit2/tree/main/moveit_planners/stomp> ；✅ <https://github.com/tesseract-robotics/trajopt> ；✅ <https://github.com/borglab/gtsam> ；✅ <https://github.com/rst-tu-dortmund/teb_local_planner>

**3.6 覆盖规划（boustrophedon / 细胞分解 / 完整覆盖）**
- **是什么**：保证把可通行区域"扫一遍"（弓字形/牛耕式、细胞分解、生成树），目标不是最短而是**覆盖率+重叠率**。
- **何时用**：清扫、消杀、巡检、农业；RM 里对应"巡逻/清点区域"这类任务（若真要做）。
- **ROS 2 实现**：Nav2 **Coverage Server**（官方覆盖服务器）；`opennav_coverage`（Nav2 兼容的完整覆盖任务服务器/导航器/BT 工具，Open Navigation 项目）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_coverage_server/> ；✅ <https://github.com/open-navigation/opennav_coverage>

**3.7 前沿 / 探索规划（frontier exploration）**
- **是什么**：把"已知与未知的边界"（frontier）当目标，边建图边导航，直到没有前沿；常配信息增益/距离的效用函数。
- **何时用**：无先验图、场地未知（第一次进场）；也是"无图模式"的标准答案。
- **ROS 2 实现**：`m-explore-ros2`（`explore_lite` 的 ROS 2 版）、`m-explore`（ROS 1 原版）。
- **来源**：✅ <https://github.com/robo-friends/m-explore-ros2> ；✅ <https://github.com/hrnr/m-explore>

**3.8 拓扑路由（route graph / 路线跟随）**
- **是什么**：预先/在线构建"路线图"（节点=关键位姿，边=可通行路段）并在图上做路由，落到具体几何路径再交给局部控制器。
- **何时用**：路线相对固定、需要"任务级"语义（巡逻、去增益点、回补给）；与 1.7 混合地图、5.4 巡逻环线天然配套。
- **ROS 2 实现**：Nav2 **Route Server**（`nav2_route`，支持 route graph 文件、图搜索、route 上的 BT 节点）；STRANDS `topological_navigation`（ROS 1 经典）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_route> ；✅ <https://github.com/strands-project/strands_navigation>

**3.9 多目标 / TSP 排序（waypoint sequencing）**
- **是什么**：给定一组目标点，决定**访问顺序**（TSP/VRP），再逐个/连续下发；也可"多点一次规划"（through poses）。
- **何时用**：巡逻、清点、多点补给；COD 的"手工示教 CSV 航点序列"就是**人肉 TSP**（顺序由人定，不求解）。
- **ROS 2 实现**：Nav2 `waypoint_follower`（`FollowWaypoints` + `NavigateThroughPoses` + 航点任务执行器插件）；顺序求解可用 Google OR-Tools。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/waypoint_follower/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_waypoint_follower> ；✅ <https://developers.google.com/optimization/routing/tsp>

**3.10 MPC 式全局规划**
- **是什么**：把"整段路径"当最优控制问题滚动求解（含动力学/障碍约束），介于全局规划与局部控制之间。
- **何时用**：高速、动力学强耦合（赛车、无人机）；移动机器人上通常只在**局部**层用 MPC（4.7），全局用图搜索。
- **ROS 2 实现**：acados（实时最优控制求解器，ROS 2 有社区封装）、`mpc_local_planner`（移动底盘 MPC 局部规划器）；Nav2 的 MPPI 属于"采样式 MPC"（4.6）。
- **来源**：✅ <https://docs.acados.org/> ；✅ <https://github.com/rst-tu-dortmund/mpc_local_planner> ；✅ arXiv 1509.01149（MPPI，**ID 已核对**）

### B.4 轴 4：局部规划与控制（11 族）

**4.1 Pure Pursuit（纯跟踪）**
- **是什么**：在前方固定/自适应前视距离取一个点，算圆弧曲率追过去；经典、极简、只控转向/角速度。
- **何时用**：差速/Ackermann 低速跟踪、路径平滑；急弯与近目标处会切角/震荡。
- **ROS 2 实现**：无独立 Nav2 插件（被 RPP 取代）；ROS 1 `base_local_planner`/自写节点常见；学术与工业代码里到处都是。
- **来源**：✅ <https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf>（Coulter, CMU TR 1992, "Implementation of the Pure Pursuit Path Tracking Algorithm"）

**4.2 Stanley 控制器**
- **是什么**：前轮转角 = 航向误差 + 前轴横向误差的反正切项，DARPA Grand Challenge 冠军方案的核心。
- **何时用**：Ackermann 高速路径跟踪、道路场景；对横向误差敏感，需与前视/滤波配合。
- **ROS 2 实现**：无 Nav2 官方插件；ROS 2 社区/自研节点常见（`stanley_controller` 类包多为民码）。
- **来源**：⚠️ <https://onlinelibrary.wiley.com/doi/10.1002/rob.20147>（Thrun et al., "Stanley: The Robot that Won the DARPA Grand Challenge", JFR 2006）

**4.3 Regulated Pure Pursuit（RPP）**
- **是什么**：纯跟踪 + 曲率/障碍/接近目标时的**速度调节**，专为服务机器人设计，参数少、行为可预测。
- **何时用**：室内服务机器人、窄通道、需要"平滑但保守"；我们 bench 的默认控制器。
- **ROS 2 实现**：Nav2 `nav2_regulated_pure_pursuit_controller`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/> ；✅ arXiv 2305.20026（RPP 论文，**ID 已核对**）；✅ <https://link.springer.com/article/10.1007/s10514-023-10097-6>（Autonomous Robots 正式版）

**4.4 DWA / DWB**
- **是什么**：在速度空间采样可行轨迹（考虑加速度极限），用多目标评价函数（避障、贴路径、朝向、速度）打分选优；DWB 是 DWA 的 Nav2 重写与插件化版本。
- **何时用**：动态障碍、需要局部绕行；参数多、调参成本高，低速下容易抖动。
- **ROS 2 实现**：Nav2 `nav2_dwb_controller`（DWB）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/> ；⚠️ <https://ieeexplore.ieee.org/document/580977>（Fox, Burgard, Thrun, DWA, IEEE RAM 1997）

**4.5 TEB（Timed Elastic Band）**
- **是什么**：把轨迹当成"橡皮筋"做多目标优化（时间最优 + 避障 + 运动学），支持全向、倒车、狭窄空间。
- **何时用**：需要"贴着障碍快速通过"、需要倒车/全向机动；调参最贵、CPU 最高、易发散。
- **ROS 2 实现**：`teb_local_planner`（ROS 1 原版，ROS 2 有分支/移植，我们 bench 已集成 + `costmap_converter`）。
- **来源**：✅ <https://github.com/rst-tu-dortmund/teb_local_planner>

**4.6 MPPI（Model Predictive Path Integral，采样式 MPC）**
- **是什么**：对大量控制序列采样、按轨迹代价指数加权，无需梯度即可处理非凸/非可微代价；Nav2 里支持差速与全向（Omni）运动学。
- **何时用**：高速、动态障碍、需要激进机动与平滑控制；CPU 开销大（`batch_size × time_steps`），需要速度包络与 critic 调参。
- **ROS 2 实现**：Nav2 `nav2_mppi_controller`（Humble 起可用；**我们已安装但尚未接入 `nav` 槽**）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller> ；✅ arXiv 1509.01149（MPPI 论文，**ID 已核对**）

**4.7 MPC（acados / casadi / mpc_local_planner）**
- **是什么**：显式求解带约束的最优控制问题（每周期解一次 QP/NLP），可显式编码动力学、速度、障碍约束。
- **何时用**：需要"可证明满足约束"的场合（安全、高速、Ackermann）；实现与实时性门槛高（求解器、模型、Warm start）。
- **ROS 2 实现**：`mpc_local_planner`（rst-tu-dortmund，ROS 1/ROS 2 代码基础）、acados（求解器）、CasADi（建模）；Nav2 官方无 MPC 插件。
- **来源**：✅ <https://github.com/rst-tu-dortmund/mpc_local_planner> ；✅ <https://docs.acados.org/>

**4.8 RL 局部策略（learned local policy）**
- **是什么**：用神经网络直接输出速度指令（端到端），或输出局部代价/评分供采样器使用；训练在仿真里完成。
- **何时用**：规则难写（人群、越野）、或作为高层决策层；**安全性与可解释性是硬伤**，比赛里几乎没人把底层交给 RL。
- **ROS 2 实现**：无官方插件；通常自写 `nav2_core::Controller` 插件或在 `cmd_vel` 前挂一个推理节点。
- **来源**：见 §B.7（7.1/7.5）；✅ <https://github.com/ros-navigation/navigation2>（Nav2 插件接口）

**4.9 速度平滑 / 指令整形**
- **是什么**：对 `cmd_vel` 做加速度/加加速度限幅、平滑、超时清零，避免"下位机收到不可能的速度"或原地抖动。
- **何时用**：**几乎总是应该开**（底盘保护、仿真↔实车一致）；我们已开 `velocity_smoother`，COD 也做了 Savitzky-Golay 平滑。
- **ROS 2 实现**：Nav2 `nav2_velocity_smoother`、`nav2_smoother`（Savitzky-Golay / constrained smoother，作用于路径而非速度）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/smoother_plugins/savitzky_golay_smoother/configuring_savitzky_golay_smoother/>

**4.10 运动学模型（差速 / 全向 / Ackermann / 腿足）**
- **是什么**：控制器与规划器必须匹配底盘可行运动集：差速（不可横移）、全向/麦轮（可横移+自旋）、Ackermann（最小转弯半径）、腿足（步态+地形）。
- **何时用**：总是——**运动学不匹配是"控制器输出对但车不动/画龙"的第一嫌疑**（我们踩过 `fake_vel_transform` 把角速度清零的坑）。
- **ROS 2 实现**：`ros2_control` 的 `diff_drive_controller` / `mecanum_drive_controller` / `ackermann_steering_controller`；Nav2 的 MPPI 有 Omni 模式、Smac Hybrid 用 Reeds-Shepp/Dubins；腿足用 elevation map + 专用步态控制器。
- **来源**：✅ <https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html> ；✅ <https://github.com/ros-controls/ros2_controllers/tree/master/mecanum_drive_controller> ；✅ <https://github.com/ros-controls/ros2_controllers/tree/master/ackermann_steering_controller> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/> ；⚠️ <https://www.science.org/doi/10.1126/scirobotics.abk2822>（Miki et al., 腿足感知式运动, Science Robotics 2022）

**4.11 反应式避障（势场 / VFH）**
- **是什么**：不规划完整路径，只根据当前障碍分布产生排斥速度/方向（势场法）或选择"障碍最少的方向扇区"（VFH）。
- **何时用**：极简平台、兜底安全层；局部极小值与震荡是固有缺陷。
- **ROS 2 实现**：无 Nav2 官方插件；ROS 1 生态有 `voxel_grid`/VFH 类包，多数团队自写。
- **来源**：⚠️ <https://journals.sagepub.com/doi/10.1177/027836498600500106>（Khatib 1986）；⚠️ <https://ieeexplore.ieee.org/document/88148>（VFH 1991）

### B.5 轴 5：任务指定方式（10 族）

**5.1 无目标（探索驱动）**
- **是什么**：不给目标，由探索策略自己生成（前沿、信息增益）；结束条件是"没有未知区域/时间到"。
- **何时用**：第一次进场、场地未知；比赛里风险高（耗时不可控）。
- **ROS 2 实现**：`m-explore-ros2`（`explore_lite`）+ 在线 SLAM（slam_toolbox / cartographer）。
- **来源**：✅ <https://github.com/robo-friends/m-explore-ros2> ；✅ <https://github.com/SteveMacenski/slam_toolbox>

**5.2 单目标（single goal pose）**
- **是什么**：给一个 `PoseStamped`，走 BT `NavigateToPose` 全流程（规划→控制→恢复）。
- **何时用**：调试、单点验证、比赛里的"去某个点"。
- **ROS 2 实现**：Nav2 `nav2_bt_navigator` + `NavigateToPose` action；`nav2_simple_commander` 的 `BasicNavigator.goToPose()`。
- **来源**：✅ <https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/> ；✅ <https://docs.nav2.org/rolling/getting_started/navigation_concepts/behavior_trees/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator>

**5.3 航点序列 / through-poses** ⭐（COD 的"多点导航"在这一格）
- **是什么**：一次给一串航点：`NavigateThroughPoses`（一次规划穿越多点）或 `FollowWaypoints`（逐个下发、每个航点可带任务插件）。
- **何时用**：巡逻、按顺序打卡；COD 用手工示教的 CSV 航点表 + 多点导航实现。
- **ROS 2 实现**：Nav2 `nav2_waypoint_follower`（`FollowWaypoints` + `WaitAtWaypoint` 等插件）；`NavigateThroughPoses` 由 BT navigator 提供。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/waypoint_follower/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_waypoint_follower> ；✅ <https://gitee.com/codnavgation/cod_-rm2026_-navigation>（csv 航点 + 单点/多点导航）

**5.4 巡逻环线（patrol loop）**
- **是什么**：把路线做成闭环，反复执行；可与"事件触发换路线"（比分、血量、裁判系统信号）结合。
- **何时用**：哨兵/安防的常态任务；RM 哨兵的"巡逻+增益点+回防"本质上是**带条件的巡逻环线**。
- **ROS 2 实现**：Nav2 Route Server（route graph 支持环路）+ waypoint follower 循环调用；上层由 BT/FSM 决定何时换路线。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_route>

**5.5 覆盖任务（coverage mission）**
- **是什么**：任务是"覆盖这块区域"，由覆盖服务器生成完整覆盖路径并执行。
- **何时用**：清扫/巡检/搜索；RM 里对应"扫点/清点"类需求。
- **ROS 2 实现**：Nav2 Coverage Server；`opennav_coverage`（含 BT 节点与导航器）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_coverage_server/> ；✅ <https://github.com/open-navigation/opennav_coverage>

**5.6 示教-重放（teach & repeat playback）**
- **是什么**：人工示教一遍（推着走/遥控），系统记录路线，之后自动重放；局部避障可叠加。
- **何时用**：路线固定、场地外观变化大、不想做全局定位；工业 AGV 与巡检机器人常见。
- **ROS 2 实现**：`wiln`（激光 teach-and-repeat）；Nav2 里可用 route graph + 录制路径近似。
- **来源**：✅ <https://github.com/norlab-ulaval/wiln> ；⚠️ Annual Reviews topometric teach-and-repeat 综述（<https://www.annualreviews.org/content/journals/10.1146/annurev-control-032724-020548>）

**5.7 语义 / 自然语言指令（VLN 家族）**
- **是什么**：用自然语言（"去红方补给区"）或视觉语言目标（"找到沙发"）指定任务，由 VLM/LLM 规划子目标。
- **何时用**：研究与人机交互场景；比赛里可靠性/算力/延迟都不达标（**本报告综合**）。
- **ROS 2 实现**：无官方栈；研究项目（VLFM、NaVid、LM-Nav、NavGPT、SayCan）多为仿真或特定硬件 Demo。
- **来源**：✅ arXiv 1711.07280（R2R VLN 基准，**ID 已核对**）；✅ arXiv 2312.03275（VLFM）；✅ arXiv 2402.15852（NaVid）；✅ arXiv 2207.04429（LM-Nav）；✅ arXiv 2305.16986（NavGPT）；✅ arXiv 2204.01691（SayCan）——以上 ID 均已核对

**5.8 示教学习（LfD / 模仿学习）**
- **是什么**：从人类示范（遥操作/轨迹）学策略，而不是回放轨迹本身；可分行为克隆、逆强化学习、DAgger 交互式修正。
- **何时用**：规则难写、示范易得；分布偏移（covariate shift）是主要失败模式。
- **ROS 2 实现**：无官方栈；常见做法是录 bag → 离线训练 → 以插件/节点形式接回 `cmd_vel`。
- **来源**：✅ arXiv 1011.0686（DAgger，**ID 已核对**）；⚠️ <https://www.sciencedirect.com/science/article/pii/S0921889009000974>（Argall et al., "A Survey of Robot Learning from Demonstration", RAS 2009）

**5.9 行为树 / FSM 任务层**
- **是什么**：把"任务"写成可组合的行为树或状态机（规划→控制→恢复→重试→换路线），Nav2 自带 BT 导航器。
- **何时用**：**只要任务不止"到一个点"，就该有这一层**；COD 用 BT + RateController + IsStuck→BackUp，我们目前是 stock BT + 自研分段工具。
- **ROS 2 实现**：`nav2_bt_navigator` + BehaviorTree.CPP（可自定义 XML 与节点）；SMACH/PlanSys2 用于更经典的任务规划。
- **来源**：✅ <https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/> ；✅ <https://www.behaviortree.dev/> ；✅ <https://github.com/PlanSys2/ros2_planning_system> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree>

**5.10 任务 / 车队协议（VDA5050 / Open-RMF / MassRobotics）**
- **是什么**：机器人与上位调度系统之间的**标准消息协议**（订单、路径、状态、错误码），让多厂商车队混跑。
- **何时用**：多机园区、仓储；单车比赛用不上，但**它定义了"工业界怎么描述任务"**，值得抄它的抽象。
- **ROS 2 实现**：Open-RMF（完整车队管理框架）、VDA5050 参考仓库、MassRobotics AMR 互操作标准。
- **来源**：✅ <https://github.com/open-rmf/rmf> ＋ ✅ <https://www.open-rmf.org/> ；✅ <https://github.com/VDA5050/VDA5050> ＋ ✅ <https://www.vda.de/dam/jcr:f0c9c019-1506-4dee-998a-e92723fbf025/EN-VDA5050-V200.pdf>（VDA5050 v2.0.0 官方规范 PDF）；✅ <https://www.massrobotics.org/what-is-the-massrobotics-amr-interoperability-standard/>

### B.6 轴 6：不确定性与安全（7 族）

**6.1 未知区策略（unknown-space policy）**
- **是什么**：三件套参数：`track_unknown_space`（是否区分未知与空闲）、规划器 `allow_unknown`（能否穿未知区）、膨胀层 `inflate_unknown`（未知区是否当障碍膨胀）。
- **何时用**：**永远是必答题**。在线建图时大量 unknown，若 `allow_unknown=true` 就会出现"穿空白/把窄缝当走廊"（我们与 COD 都踩过，见 `docs/cod_nav_comparison.md §3`）。
- **ROS 2 实现**：`nav2_costmap_2d` 参数 + 各规划器参数（NavFn/Smac/Theta\*）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/>

**6.2 重规划与恢复行为（recovery）**
- **是什么**：卡住/路径失效时的兜底动作：清代价地图、自旋、后退、等待、按航向直行；BT 决定重试次数与顺序。
- **何时用**：所有真实部署；没有恢复行为的栈在动态场景里"一次失败就永久失败"。
- **ROS 2 实现**：Nav2 Behavior Server（`spin`/`backup`/`drive_on_heading`/`wait`/`assisted_teleop`）+ BT 恢复分支（`ClearEntireCostmap` 等）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/> ；✅ <https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors>

**6.3 碰撞监控 / 安全监控 / 限速**
- **是什么**：独立于规划器的"最后一道软件闸门"：按多边形/圆形监测区判断即将碰撞则减速/停车；以及速度限幅与超时清零。
- **何时用**：有人/有对抗的动态环境；**工业上这一层通常还要被安全 PLC/安全激光雷达取代或冗余（见 6.6/6.7）**。
- **ROS 2 实现**：Nav2 `nav2_collision_monitor`（collision detector + monitor 节点，可作为 `cmd_vel` 中继）；`nav2_velocity_smoother`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_collision_monitor> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/>

**6.4 风险感知与机会约束规划（risk-aware / chance-constrained）**
- **是什么**：不追求"零碰撞"，而是约束**碰撞概率**（机会约束）或优化风险泛函（CVaR、风险敏感 MPC）。
- **何时用**：不确定性大（定位漂移、动态障碍预测）、需要量化安全边界；计算复杂，工程落地少。
- **ROS 2 实现**：无官方插件；通常在自研 MPC/采样器里加风险项（学术代码为主）。
- **来源**：✅ arXiv 2307.08024（"Bayesian inference for data-efficient, explainable, and safe robotic motion planning: A review"，**ID 已核对**；含不确定性/风险规划章节）

**6.5 主动 SLAM / 不确定性驱动探索**
- **是什么**：把"定位/地图的不确定性"作为探索目标（最大化信息增益、最小化位姿协方差），而不仅是"探索未知区域"。
- **何时用**：地图质量决定后续任务成败、且场地允许自由探索；比赛场景通常时间不够（**本报告综合**）。
- **ROS 2 实现**：无官方栈；`m-explore` 类包只用几何前沿，不含不确定性项；主动 SLAM 以研究代码为主。
- **来源**：✅ arXiv 2207.00254（"A Survey on Active Simultaneous Localization and Mapping"，**ID 已核对**）

**6.6 安全标准与"安全链独立"（ISO 3691-4 / ANSI-A3 R15.08 / ISO 13849 / IEC 61508）**
- **是什么**：无人工业车辆（ISO 3691-4）、工业移动机器人（ANSI/A3 R15.08 系列）、机械安全控制系统（ISO 13849-1）、功能安全通用（IEC 61508）等标准，规定安全功能、性能等级（PL/SIL）与验证方法。
- **何时用**：**产品要出货、要过认证、要进工厂就必须**；比赛不用，但它解释了"为什么工业产品不敢让导航计算机决定停车"。
- **ROS 2 实现**：标准不提供实现；工程上的结论是**安全链（安全激光雷达/安全 PLC/急停/安全限速）与导航计算机物理/逻辑分离**——导航只出"建议速度"，安全链有权否决（**本报告综合 + 厂商文档佐证**，见 6.7 与 8.8）。
- **来源**：⚠️ <https://www.iso.org/standard/70660.html>（ISO 3691-4:2020 官方目录页）；⚠️ <https://www.iso.org/standard/69883.html>（ISO 13849-1）；⚠️ <https://webstore.ansi.org/standards/ria/ansia3r15082026>（ANSI/A3 R15.08-3-2026）；⚠️ <https://www.automate.org/standards/industrial-mobile-robots>（A3 工业移动机器人标准页）；✅ <https://www.iec.ch/functionalsafety>（IEC 功能安全总览）

**6.7 安全型传感器与安全 PLC（safety-rated sensing）**
- **是什么**：通过认证的安全激光扫描仪、安全编码器、安全控制器（可做安全限速、安全区域、安全停机）。
- **何时用**：与 6.6 绑定；典型形态是"安全扫描仪覆盖车体前后 + 安全 PLC 直接切动力"。
- **ROS 2 实现**：安全设备一般**不接 ROS 图**（接安全 PLC）；ROS 侧只接收状态位（如"安全停"）用于状态机。
- **来源**：✅ <https://www.sick.com/us/en/catalog/products/safety/safety-laser-scanners/microscan3/c/g187235>（SICK microScan3 安全激光扫描仪产品页）；⚠️ <https://www.pilz.com/en-JP/company/press/messages/articles/239220>（Pilz：AMR 安全导航方案新闻稿，厂商资料）；✅ <https://www.sick.com/media/docs/3/43/143/operating_instructions_nav350_laser_positioning_sensor_en_im0040143.pdf>（SICK NAV350 操作手册，含反射板定位与安全相关说明）

### B.7 轴 7：学习 / 端到端 / 基础模型（8 族）

**7.1 RL 导航（DQN / DDPG / SAC / PPO 系）**
- **是什么**：把导航建成 MDP，用深度 RL 学策略（目标驱动视觉导航、连续控制避障）。
- **何时用**：仿真里可大规模采样、奖励可设计（人群避让、越野）；真机迁移与安全性是门槛。
- **ROS 2 实现**：无官方栈；典型是"仿真训练 → 导出 ONNX → ROS 2 推理节点"。
- **来源**：✅ arXiv 1312.5602（DQN）；✅ arXiv 1509.02971（DDPG）；✅ arXiv 1801.01290（SAC）；✅ arXiv 1707.06347（PPO）；✅ arXiv 1609.05143（Target-driven 视觉导航）——以上 ID 均已核对

**7.2 基准与仿真器（Habitat / Isaac Lab / Arena-Rosnav / iGibson）**
- **是什么**：为导航/具身智能提供场景、传感器仿真、任务与评测协议（成功率、SPL 等）。
- **何时用**：做学习型导航的对照实验；**我们的 Gazebo Classic bench 属"工程验证"而非"学习基准"**，两者互补。
- **ROS 2 实现**：Habitat（非 ROS，Python 生态）、Isaac Lab（GPU 并行仿真，有 ROS 2 桥）、Arena-Rosnav（ROS 2 社交导航基准）、iGibson。
- **来源**：✅ <https://aihabitat.org/> ＋ ✅ <https://github.com/facebookresearch/habitat-lab> ；✅ <https://isaac-sim.github.io/IsaacLab/> ＋ ✅ <https://github.com/isaac-sim/IsaacLab> ；✅ <https://github.com/Arena-Rosnav/arena-rosnav> ；✅ <https://github.com/StanfordVL/iGibson>

**7.3 模仿学习（IL / 行为克隆 / DAgger）**
- **是什么**：从专家轨迹学策略；DAgger 通过"边跑边问专家"缓解分布偏移。
- **何时用**：示范数据易得、奖励难设计；长程导航里常与拓扑图/子目标结合。
- **ROS 2 实现**：无官方栈；录 bag → 训练 → 推理节点。
- **来源**：✅ arXiv 1011.0686（DAgger）；⚠️ ScienceDirect Argall LfD 综述（同 5.8）

**7.4 Sim-to-real（域随机化 / 迁移）**
- **是什么**：让仿真里学到的策略在真机可用：域随机化、系统辨识、噪声建模、teacher-student 蒸馏。
- **何时用**：任何"仿真训练 + 真机部署"的路线；**对我们的 bench 尤其相关：仿真↔实车契约本身就是 sim-to-real 的工程化版本**（见 `docs/sim_real_contract.md`）。
- **ROS 2 实现**：Isaac Lab 内置域随机化；Gazebo 侧靠插件与参数扰动自己搭。
- **来源**：✅ arXiv 2009.13303（Sim-to-Real 综述）；✅ arXiv 1703.06907（域随机化）——ID 均已核对

**7.5 基础模型导航（VLFM / NaVid / NoMaD / ViNT-GNM / π0 系）**
- **是什么**：在大规模数据上预训练的导航模型：视觉导航基础模型（ViNT/GNM/NoMaD）、零样本语义前沿（VLFM）、视频 VLM 导航（NaVid）、VLA 动作模型（π0/OpenVLA）。
- **何时用**：跨场景零样本、语言/图像目标；算力、实时性、可解释性都远离比赛要求（**本报告综合**）。
- **ROS 2 实现**：研究代码为主（多数是 Python + 仿真/自有硬件），需要自己包成 ROS 2 节点。
- **来源**：✅ arXiv 2312.03275（VLFM）；✅ arXiv 2402.15852（NaVid）；✅ arXiv 2310.07896（NoMaD）；✅ arXiv 2306.14846（ViNT，含 GNM）；✅ arXiv 2410.24164（π0）；✅ arXiv 2406.09246（OpenVLA）；✅ <https://visualnav-transformer.github.io/> ；✅ <https://pku-epic.github.io/NaVid/>；✅ <https://github.com/Physical-Intelligence/openpi>——arXiv ID 均已核对

**7.6 世界模型（world models）**
- **是什么**：学环境的隐动力学模型，在"想象"里做规划/训练（Ha & Schmidhuber 的 VAE+RNN；DreamerV3 的隐空间 actor-critic）。
- **何时用**：样本效率要求高、奖励稀疏；对导航的工程价值仍在研究中。
- **ROS 2 实现**：无官方栈。
- **来源**：✅ arXiv 1803.10122（World Models，**ID 已核对**）；✅ arXiv 2301.04104（DreamerV3，**ID 已核对**）

**7.7 扩散策略（diffusion policy / planning）**
- **是什么**：用扩散模型建模多模态动作分布（Diffusion Policy）或直接生成轨迹（Diffuser）；天然表达"多峰"行为。
- **何时用**：多模态示范数据、需要避障与示教风格兼容；推理频率与实时性是限制。
- **ROS 2 实现**：研究代码；通常作为高层策略，输出给底层控制器。
- **来源**：✅ arXiv 2303.04137（Diffusion Policy）；✅ arXiv 2205.09991（Diffuser）；✅ arXiv 2403.03954（3D Diffusion Policy）；✅ <https://diffusion-policy.cs.columbia.edu/> ——ID 均已核对

**7.8 LLM/VLM 任务规划（SayCan / Code as Policies / VoxPoser / LLM-Planner）**
- **是什么**：LLM/VLM 把语言指令分解成可执行子目标或代码，再由经典栈执行。
- **何时用**：任务层需要灵活性、可解释的中间表示；对"是否能落地到比赛"目前答案是"不能直接落地，但可借它的任务分解思路"（**本报告综合**）。
- **ROS 2 实现**：无官方栈；常见做法是 LLM 输出 JSON/航点序列交给 Nav2 action。
- **来源**：✅ arXiv 2204.01691（SayCan）；✅ arXiv 2209.07753（Code as Policies）；✅ arXiv 2307.05973（VoxPoser）；✅ arXiv 2212.04088（LLM-Planner）；✅ arXiv 2305.16986（NavGPT）——ID 均已核对

### B.8 轴 8：工业 AGV/AMR 范式（8 族）

**8.1 磁条 / 磁带导引**
- **是什么**：地面贴磁带/磁条，车载磁传感器循迹；位置精度靠标记点（RFID/磁钉）修正。
- **何时用**：路线极固定、成本敏感、环境要求"绝对可预测"；改路线要重新贴磁条。
- **ROS 2 实现**：通常不由 ROS 主导（PLC/专用控制器），ROS 只做上层调度与 HMI。
- **来源**：✅ <https://www.mhi.org/blog/126126/agv-amr-navigation-systems-floor-based-vs-lidar>（MHI 行业博客：floor-based vs LiDAR 导引对比，**行业资料**）；✅ <https://iopscience.iop.org/article/10.1088/1755-1315/1285/1/012023>（AGV 导航技术综述类会议论文）

**8.2 二维码 / 标记导引**
- **是什么**：地面二维码阵列，车载下视相机读码得到绝对位姿，码间靠里程计；仓储机器人经典方案。
- **何时用**：高密度货架、需要厘米级停靠；地面必须保持洁净可贴码。
- **ROS 2 实现**：`apriltag_ros`、OpenCV ArUco；工业上多为厂商私有实现。
- **来源**：✅ <https://github.com/AprilRobotics/apriltag> ；⚠️ <https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html> ；⚠️ <https://ieeexplore.ieee.org/document/11324329>（磁条-二维码混合导航论文，IEEE 目录页）

**8.3 反射板 / 激光三角定位**
- **是什么**：墙上/柱上装反光板，激光扫描仪测角（三角/三边）解算绝对位姿；工业 AGV 的"可认证"定位方案。
- **何时用**：需要长期稳定、可做安全型定位、场地可改造；需要布置反射板并做标定。
- **ROS 2 实现**：厂商方案为主（SICK NAV350、Kollmorgen/NDC8 等）；ROS 侧通常只做上层接口。
- **来源**：✅ <https://www.sick.com/media/docs/3/43/143/operating_instructions_nav350_laser_positioning_sensor_en_im0040143.pdf>（SICK NAV350 官方操作手册）；⚠️ <https://ndcsolutions.com/zh-hans/ndc8-40-including-our-new-generation-of-laser-navigation/>（NDC8 激光导航系统，厂商页）；⚠️ <https://www.kollmorgen.com/en-us/products/autonomous-mobile-solutions>（Kollmorgen AGV/AMR 方案）

**8.4 示教-重放 AGV**
- **是什么**：人工带车走一遍，系统记录路径（磁条/激光/视觉均可），之后自动重放。
- **何时用**：路线稳定、部署时间紧、现场工程师不想碰 SLAM 参数；与 1.8/5.6 是同一族在工业里的名字。
- **ROS 2 实现**：`wiln`；工业上多为示教盒 + 专有控制器。
- **来源**：✅ <https://github.com/norlab-ulaval/wiln> ；✅ MHI 行业博客（同 8.1）

**8.5 SLAM AMR（自然导航）**
- **是什么**：不靠地面基础设施，靠激光 SLAM + 先验图定位"自然导航"；部署灵活，但需要环境特征与调参。
- **何时用**：场地多变、改造受限、要与人类混行；代表产品为仓储/服务 AMR。
- **ROS 2 实现**：Nav2 全栈（AMCL + NavFn/Smac + DWB/MPPI）、Cartographer、slam_toolbox。
- **来源**：✅ MHI 行业博客（floor-based vs LiDAR，含自然导航对比）；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/> ；✅ <https://github.com/SteveMacenski/slam_toolbox>

**8.6 车队管理（fleet manager）**
- **是什么**：多台 AGV/AMR 的交通管制、任务派发、充电调度、电梯/门禁联动。
- **何时用**：≥2 台车、共享通道时**必须**有（否则死锁）；调度层与导航层解耦。
- **ROS 2 实现**：Open-RMF（开源车队管理框架，含交通调度与任务编排）；商用如 MiR Fleet / OTTO Fleet Manager（私有）。
- **来源**：✅ <https://www.open-rmf.org/> ＋ ✅ <https://github.com/open-rmf/rmf> ；✅ <https://www.massrobotics.org/what-is-the-massrobotics-amr-interoperability-standard/>

**8.7 VDA5050 协议**
- **是什么**：AGV/AMR 与上位系统之间的标准 JSON/MQTT 接口（订单、即时动作、状态、可视化、错误），支持多厂商混跑。
- **何时用**：产品要接入第三方 WMS/MES/车队系统；**它是"任务层"的工业标准答案**（对应 5.10）。
- **ROS 2 实现**：VDA5050 官方参考仓库；社区有 `vda5050_connector` 类 ROS 2 桥（部分仓库已迁移/私有化，需自行确认）。
- **来源**：✅ <https://github.com/VDA5050/VDA5050> ；✅ <https://www.vda.de/dam/jcr:f0c9c019-1506-4dee-998a-e92723fbf025/EN-VDA5050-V200.pdf>

**8.8 为什么工业偏好固定路径 + 拓扑图**
- **是什么/为什么（本报告综合，基于 8.1–8.7 与 6.6 的来源）**：① 安全认证要求可预测、可验证的行为，自由规划难以论证"不会撞人"；② 安全型定位（磁条/二维码/反射板）比 SLAM 更容易过认证；③ 客户要的是**可重复、可审计**的节拍，而不是最优路径；④ 拓扑图/固定路线让调度、交通管制、验收测试都变简单；⑤ SLAM 的失效模式（动态环境、长走廊）在工厂里代价极高。
- **何时用**：工业落地/需要认证的场合；反过来说，**比赛/研究场景的"自由规划"优势在工厂里往往不是优势**。
- **ROS 2 实现**：这一族本身是"设计取向"而非算法，落地组合见 §D.1。
- **来源**：✅ MHI 行业博客（floor-based vs LiDAR）；⚠️ <https://www.pilz.com/en-JP/company/press/messages/articles/239220>（Pilz AMR 安全导航，厂商资料）；⚠️ ISO 3691-4 官方目录页（同 6.6）

### B.9 轴 9：多机 / 协同（6 族，简述）

**9.1 编队控制（formation control）**
- **是什么**：多机保持几何构型（队形）运动：领航-跟随、虚拟结构、基于位移/距离的编队。
- **何时用**：表演、协同搬运、护航；RM 里几乎用不到（规则上通常单兵）。
- **ROS 2 实现**：无官方栈；研究代码 + 自研。
- **来源**：⚠️ <https://www.sciencedirect.com/science/article/pii/S0005109815001038>（Oh, Park, Ahn, "A survey of formation control of multi-agent systems", Automatica 2015）

**9.2 任务分配（MRTA）**
- **是什么**：把任务集合分配给多台机器人（拍卖、市场、匈牙利算法、CBBA），目标是最小化完成时间/代价。
- **何时用**：多机巡检、搜索；与 3.9 的单机 TSP 是同一问题的多机版。
- **ROS 2 实现**：研究代码为主；Open-RMF 的任务编排可视为工程化实现。
- **来源**：⚠️ <https://journals.sagepub.com/doi/10.1177/0278364904045564>（Gerkey & Matarić, IJRR 2004, MRTA 分类学）；✅ <https://github.com/open-rmf/rmf>

**9.3 多机 SLAM / 地图融合**
- **是什么**：多机各自建图 + 机间回环/地图合并（分布式 PGO），得到全局一致地图。
- **何时用**：大场地多机同时建图；通信带宽与时间同步是门槛。
- **ROS 2 实现**：`Kimera-Multi`（分布式度量-语义）、`Swarm-SLAM`（稀疏分布式协作 SLAM）。
- **来源**：✅ <https://github.com/MIT-SPARK/Kimera-Multi> ；✅ <https://github.com/MISTLab/Swarm-SLAM> ＋ ✅ arXiv 2301.06230（**ID 已核对**）

**9.4 多机前沿探索**
- **是什么**：把前沿分配给不同机器人，避免重复探索；经典工作是"协调式多机探索"（基于效用/代价分配）。
- **何时用**：灾难搜救、大范围未知环境。
- **ROS 2 实现**：`m-explore` 系列有多机分支/扩展；多在研究代码里。
- **来源**：✅ <https://github.com/hrnr/m-explore> ；✅ <https://github.com/robo-friends/m-explore-ros2> ；⚠️ <https://ieeexplore.ieee.org/document/1389414>（Burgard et al., "Coordinated Multi-Robot Exploration", IEEE T-RO 2005）

**9.5 通信约束下的协同**
- **是什么**：考虑带宽、丢包、断连（间歇通信）的协同策略：何时通信、共享什么、断连时如何自治。
- **何时用**：真实无线环境、地下/遮蔽场景；仿真里常被忽略，真机上是大坑。
- **ROS 2 实现**：ROS 2 多机通信（DDS 域、`ROS_DOMAIN_ID`、Discovery Server）；工程参考 OSRF《ROS 2 多机器人系统》书。
- **来源**：✅ <https://osrf.github.io/ros2multirobotbook/> ；✅ <https://docs.ros.org/en/jazzy/p/example_multi_robot/>

**9.6 ROS 2 多机部署**
- **是什么**：命名空间、TF 前缀、多 `map` 系、DDS 配置等工程手段，让 N 台车共用一套栈。
- **何时用**：多机开发/仿真；RM 有队伍用 namespace 为未来多机留位（如 SMBU PolarBear 的 `pb2025_sentry_nav`）。
- **ROS 2 实现**：`example_multi_robot`（ROS 2 官方示例）、Isaac Sim ROS 2 多机导航教程、Nav2 多机 bringup 实践。
- **来源**：✅ <https://docs.ros.org/en/jazzy/p/example_multi_robot/> ；✅ <https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_multi_navigation.html> ；✅ <https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>

### B.10 轴 10：RM 哨兵生态落点（1 行/仓库）

> 说明：以下"用了哪些族"是根据各仓库 **README / 文件清单 / 我们自己的只读深读文档**归纳的（**本报告综合**），
> 未 clone、未运行；仓库可能在赛季间继续变动。「ID 已核对」类论文引用同前。

| 项目 | 链接 | 它落在这张地图的哪些格子 |
|---|---|---|
| **SMBU-PolarBear `pb2025_sentry_nav`**（深圳北理莫斯科大学北极熊，2025 赛季） | ✅ <https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav> | 2.3 Point-LIO（自维护分支）+ 2.4 small_gicp 重定位 + 1.2/2.5D `terrain_analysis(_ext)` 地形分析（离地高度写进 intensity）+ 1.3 STVL/点云层 + 3.3 Nav2 默认全局规划器 + 4.x `pb_omni_pid_pursuit_controller` 全向 PID 跟踪 + 5.3 航点 + 9.6 namespace 多机预留；`pointcloud_to_laserscan` 仅在 SLAM 模式启用 |
| **PolarisXQ `SCURM_SentryNavigation`**（SCURM 火锅战队 24–25 赛季） | ✅ <https://github.com/PolarisXQ/SCURM_SentryNavigation> | 2.3 改造版 FAST-LIO（**自带重定位模式**，省一套重定位算力）+ 2.4 ICP 重定位 + 1.2 `terrain_analysis` 地形分析 + 自研 **costmap intensity 层**（1.10/1.2 的工程化）+ 6.2 增强版 back_up 恢复行为（朝无碰撞方向退）+ 5.9 BehaviorTree.CPP |
| **XDU-IRobot `SentryNav2026_XDU`**（西安电子科技大学 IRobot，2026 赛季） | ✅ <https://github.com/XDU-IRobot/SentryNav2026_XDU> | 2.3 Super-LIO 里程计 + 2.4 **Lightning-LM 重定位** + 1.12/2.4 点云裁剪与配准 + 1.3/1.2 深度相机（D435）点云 + 3.x Nav2 导航栈 + 5.9 自主行为树决策 + 裁判系统/CAN 通信（任务层）。注：仓库名不是 `sentry_navigation`（该名 404），`sentry_navigation` 是其**包目录**；RM2024 版本开源贴见 ✅ <https://bbs.robomaster.com/article/374512> |
| **`laohao78/ROS2_RM_Navigation`** | ✅ <https://github.com/laohao78/ROS2_RM_Navigation> | 2.3 FAST-LIO / Point-LIO + 2.4 **SLAM-Toolbox / AMCL / ICP 多选重定位** + 2.1 轮式里程计 + 3.3 NavFn 类全局规划 + 4.5 TEB 局部规划 + `mapping`/`nav` 双模式 + Gazebo Classic 仿真（与我们 bench 结构最接近的公开仓库） |
| **COD 战队：GitHub `qza36/COD_NAV` + Gitee `cod_-rm2026_-navigation`** | ✅ <https://github.com/qza36/COD_NAV> ；✅ <https://gitee.com/codnavgation/cod_-rm2026_-navigation> | 2.3 FAST-LIO fork / Small Point-LIO + **2.9 静态 `map→odom`（不重定位）** + 1.3/1.1 两张 costmap 都上 STVL（时间维）+ 1.12 车体点云裁剪盒 + 3.x Smac2D（2025）/MPPI 侧全局 + 4.6 **MPPI（全向，高速）** + 5.3 **CSV 航点序列（多点导航）** + 4.9 Savitzky-Golay 平滑；RM2026 开源贴 ✅ <https://bbs.robomaster.com/article/1882897> |
| **`Yancey2023/small_point_lio`**（东莞理工 ACE，RM26） | ✅ <https://github.com/Yancey2023/small_point_lio> | 2.3 LIO（Point-LIO 的 2–3× 加速变体，ESKF + iVox，**无回环无 PGO**，见我们 `docs/cod_nav_2026_deep_dive.md` 的只读核对）——它是"里程计插槽"里的一个实现，不是完整导航方案 |
| **`Xiancaijiang/rm2025_sentry_nav`**（PolarBear 2025 的另一公开副本/前身） | ✅ <https://github.com/Xiancaijiang/rm2025_sentry_nav> | 与 `pb2025_sentry_nav` 同源；可作为"同一套设计在不同赛季的演化"样本 |
| **`HongbiaoZ/autonomous_exploration_development_environment`** | ✅ <https://github.com/HongbiaoZ/autonomous_exploration_development_environment> | 1.2 **2.5D 地形分析**（`terrain_analysis`/`terrain_analysis_ext`）与 3.7 探索式导航的参考实现；被多个 RM 队直接借用（PolarBear、SCURM 均引用它） |
| **`SMBU-PolarBear-Robotics-Team/small_gicp_relocalization`** | ✅ <https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization> | 1.12 + 2.4：把先验 PCD 与实时点云做 GICP 配准求 `map→odom`（RM 生态里"PCD 先验重定位"的标准件） |
| **`icp_registration`**（我们 bench 内 `src/localization/rm_localization/icp_registration`） | 本地包（无外部 URL） | 1.12 + 2.4：与上者同类，作为我们 `localization:=icp` 槽的实现 |

---

## C. 决策表：如果你要 … → 推荐族 + 代价

> 「族」编号对应 §B；「代价」一列是**必须一起接受的代价**，不是可选项。

| # | 如果你要 … | 推荐族（编号） | 代价 / 注意 |
|---|---|---|---|
| 1 | **无图 / 第一次进场（场地未知）** | 1.1 + 1.3（在线 SLAM 出图）+ 3.7 前沿探索 + 6.2 恢复行为；里程计用 2.3 | 无重复精度；地图质量随圈数变化；**未知区策略（6.1）必须先定**，否则"穿空白/窄缝当走廊"；探索耗时不可控 |
| 2 | **有图、要重复精度（同场地反复跑）** | 1.1 + 2.4 重定位（AMCL / GICP / Cartographer 纯定位 / slam_toolbox localization）+ 3.2 Smac + 4.3 RPP 或 4.6 MPPI | 要维护地图资产与坐标系一致性；场地一改图就作废；**重定位失效没有兜底就会"整体跑飞"**（要有 2.5 回环/多假设或 2.9 兜底策略） |
| 3 | **换场地免重标定** | 1.8 示教-重放 / 1.5+3.8 拓扑路由 + 2.3 LIO 里程计（不依赖先验图）；或每次重跑 1.1+3.7 | 路线要重教/重录，人工成本转移到赛前；没有全局最优路径；示教质量直接决定上限 |
| 4 | **要安全认证 / 要出货** | 6.6 + 6.7（安全链独立）+ 8.1–8.3 固定路径与安全型定位 + 6.3 碰撞监控与限速 | 硬件成本（安全扫描仪/PLC）；自由规划被大幅限制；ROS 侧只出"建议速度"；认证周期长 |
| 5 | **要高速动态避障** | 4.6 MPPI（全向）+ 4.10 全向运动学 + 1.3 STVL（时间衰减）+ 4.9 速度平滑 + 6.3 碰撞监控 | CPU（`batch_size×time_steps`）、critic 调参量大；**速度包络必须从低速标起**；`robot_radius/footprint` 不诚实会直接撞 |
| 6 | **要开源可改 / 生态最厚** | 3.x/4.x/5.x/6.x 的 **Nav2 实现**（插件化：planner/controller/costmap layer/BT 节点都可自写）+ 3.6 `opennav_coverage` + 3.8 `nav2_route` | Nav2 是 2D 接口为中心（3D 规划要自写插件）；Humble 与最新文档存在版本差异；插件多了以后"契约问题"比"算法问题"更常见（我们已踩过 6 个） |
| 7 | **窄缝 / 贴边通过（场地小、障碍密）** | 1.1 诚实 footprint + 3.2 Smac2D（`cost_travel_multiplier` 把路径推离膨胀层）+ 4.3 RPP（保守）或 4.6 MPPI + 6.1 `inflate_unknown` 收紧 | 与"高速"直接冲突；膨胀半径/代价乘子是"窄缝当走廊"的正面对策，但过紧会无解；要接受更慢的平均速度 |
| 8 | **长走廊 / 几何退化（雷达看不到特征）** | 2.3 多源 LIO（FAST-LIO + Point-LIO 可选）+ 2.2 轮+IMU EKF 兜底 + 2.4 先验图配准（GICP/反射板） | 复杂度上升；需要"谁在什么时候发 `map→odom`"的严格契约（我们已有 `tf_interface_contract.md`）；退化时任何单一来源都会漂 |
| 9 | **要 2.5D/3D 地形能力（斜坡、台阶、高地）** | 1.2 高程/可通行性（`terrain_analysis` 或 `elevation_mapping`）+ 1.4 ESDF（若要做平滑三维轨迹）+ 4.10 匹配的运动学 | 需要自写 costmap 层/规划器（Nav2 规划接口是 2D）；调参与标定量显著上升；仿真里"斜坡"很难做得像真的 |
| 10 | **语言/语义任务（"去增益点"式指令）** | 1.6 语义地图 + 7.5/7.8 基础模型与 LLM 任务层 + 3.8 拓扑路由（把语义落成节点） | 算力与延迟；可靠性不足以承担比赛；**现实做法是"人写拓扑节点 + 语义标签"，而不是端到端 VLM**（本报告综合） |
| 11 | **多机协同** | 9.2 任务分配 + 9.3 多机 SLAM 或各自独立建图 + 8.6 车队管理/8.7 VDA5050（工程化）+ 9.6 namespace 部署 | 通信是最大风险（9.5）；调试复杂度非线性上升；比赛规则通常不允许 |

---

## D. 标准产品化组合（5~8 种）与各自代表

> 这一节回答"到底有几种**真在用**的思路"。结论：**8 种**（其中 1–3 是量产主流，4–8 各有明确适用面）。

| # | 组合名 | 用到的族 | 代表实现 / 产品 | 一句话适用面 |
|---|---|---|---|---|
| **D1** | **固定路径 AGV**（基础设施导引 + 拓扑路线 + 安全链） | 8.1/8.2/8.3 + 1.5/3.8 + 6.6/6.7 + 4.9 | SICK NAV350、Kollmorgen NDC8（⚠️ 厂商页 / ✅ 手册 PDF） | 路线固定、要认证、要可审计的工厂物流 |
| **D2** | **经典 AMR / Nav2 默认栈**（先验 2D 图 + AMCL + 图搜索 + DWA/RPP） | 1.1 + 2.4(AMCL) + 3.1/3.3 + 4.3/4.4 + 6.1/6.2 | Nav2 bringup 默认（AMCL + NavFn + DWB/RPP）；ROS 1 时代 `move_base` 同构 | 室内服务机器人、教学、绝大多数 ROS 项目起点 |
| **D3** | **高速全向竞速栈 / RM 强队栈**（LIO + 配准重定位 + Smac/MPPI + 时间维代价图） | 2.3 + 2.4(GICP/ICP) + 3.2 + 4.6 + 1.3(STVL) + 4.9 | COD_NAV（MPPI Omni + Small Point-LIO + STVL）、`pb2025_sentry_nav`（Point-LIO + small_gicp + 全向 PID 跟踪） | 场地固定/半固定、要高动态、可接受调参成本 |
| **D4** | **无图探索栈**（在线 SLAM + 前沿探索 + 恢复） | 1.1/1.3 + 2.3 + 3.7 + 6.2 + 5.1 | `m-explore-ros2`/`explore_lite` + slam_toolbox | 第一次进场、未知场地、救援/巡检 |
| **D5** | **示教-重放栈**（topometric 路线跟随，无全局度量图） | 1.8 + 5.6 + 2.1/2.3（局部里程计）+ 4.x 局部控制 | `wiln`（激光 teach-and-repeat）；工业示教 AGV | 路线固定但环境外观变化大、部署时间紧 |
| **D6** | **学习式端到端 / 基础模型栈**（视觉导航模型直接出动作或子目标） | 7.5 + 7.1/7.3/7.7 + 1.10 | ViNT/GNM/NoMaD（visualnav-transformer）、VLFM、NaVid、π0/openpi | 研究、Demo、跨场景零样本；**离比赛可靠性尚远** |
| **D7** | **语义/语言任务栈**（LLM/VLM 分解任务 + 经典栈执行） | 7.8 + 1.6 + 3.8 + 5.7 | SayCan、LM-Nav、Code as Policies、NavGPT | 人机交互研究、服务机器人原型 |
| **D8** | **多机 / 车队栈**（多机协同 + 调度协议） | 9.1–9.6 + 8.6 + 8.7 + 5.10 | Open-RMF、VDA5050 生态、Kimera-Multi/Swarm-SLAM | 园区、仓储、多机协同作业 |

**为什么不是 81 种（本报告综合）**：族之间有四类硬约束——① **互斥**（2.4 与 2.9 都在发 `map→odom`；1.1 与 1.9 是不同世界的表示）；
② **依赖**（3.7 探索必须有在线建图；5.5 覆盖必须有覆盖服务器；D1 必须有安全链）；
③ **成本不叠加**（同时上 3.2+4.5+4.6 只会在"调参地狱"里互相对冲，收益不叠加）；
④ **场景筛掉大半**（RM 场地是平地、单机、强对抗、每局重开 ⇒ 1.2/1.9/9.x 基本不选）。
所以真正的自由度是"**在 8 种组合里选 1 种，再在 3~4 个插槽里换实现**"。

---

## E. 我们 vs COD 在这张地图上的位置

**（4 句结论）** 我们和 COD 落在**同一格 D3（高速全向竞速栈）**，差别主要在"地图/重定位这一层"：
COD（`rmul2026`/Gitee RM2026）走的是 **2.9 静态 `map→odom` + 在线 slam_toolbox + 手教 CSV 航点（5.3）+ MPPI（4.6）**，
即"**不重定位、靠每局当场重建地图，任务顺序由人写死**"；我们走的是 **2.4 四选一重定位（AMCL/ICP/slam_toolbox/Cartographer 纯定位）+ `mode` 三态（mapping / slam_nav / nav）**，
即"**重定位是可换插槽、地图是资产**"，但我们的全局规划器仍是 **3.3 NavFn**、控制器只接了 **4.3/4.4/4.5**（**4.6 MPPI 与 3.2 Smac / 3.1 Theta\* / 3.8 nav2_route / 6.3 collision_monitor 都已在 `/opt/ros/humble` 装好但未接入**）。
所以在地图上：**我们"覆盖的格子"更宽（表示层、定位层的可替换性更强），COD"在所选格子里的深度"更深（MPPI 参数配方、STVL 时间维、速度包络已跑到 7.5 m/s 级）**；
我们最短的补课路径不是加更多算法，而是把**已装未接**的 Smac/MPPI/collision_monitor 接进槽位，并补上 5.3/5.4 的航点与巡逻任务层（COD 已有、我们目前偏薄）。

**支撑细节（可选读）**：

| 轴 | 我们（本 bench，自查） | COD_NAV（rmul2026 / Gitee RM2026，只读核对） |
|---|---|---|
| 1 世界表示 | 1.1 2D costmap（static+obstacle+inflation）+ 1.3 STVL（仅 global） | 1.1 + 1.3 **local 与 global 都上 STVL**（`voxel_decay`），1.12 车体点云裁剪盒 |
| 2 定位 | 2.3 FAST-LIO/Point-LIO + **2.4 四选一**（AMCL/ICP/slam_toolbox/Cartographer 纯定位） | 2.3 Small Point-LIO + **2.9 静态 `map→odom`（无重定位）** |
| 3 全局规划 | 3.3 NavFn（Smac/Theta\* 已装未接） | Smac2D（2025 master）/ MPPI 侧全局（2026） |
| 4 局部控制 | 4.3 RPP（默认）/4.4 DWB/4.5 TEB；**4.6 MPPI 已装未接** | **4.6 MPPI Omni @50 Hz**，速度包络到 7.5 m/s；4.9 SG 平滑 |
| 5 任务 | stock BT + 自研分段工具；5.3 waypoint_follower 已启动但未用于成体系巡逻 | **5.3 CSV 航点 + 单点/多点导航** + `waypoint_editor` |
| 6 安全/恢复 | 6.2 stock 恢复；6.3 collision_monitor 已装未接 | 6.2 BT：RateController 3 Hz + IsStuck→BackUp + 10 次重试 |

---

## F. 未能找到权威来源的地方 / 存疑项（请按此打折使用）

1. **"WVN world representation" 无法确权**。任务描述里提到"learned latent/occupancy（e.g. WVN world representation）"，
   我用多组检索都没有找到与导航地图表示对应的、可核对的 WVN 论文或仓库（检索命中的都是无关内容）。
   ⇒ 1.10 我改用**可核对的三个来源**（Wild Visual Navigation、Learning Social Cost Functions、DreamerV3）替代，**WVN 未写入**。
   如果这是某篇内部/最新论文，请补一个链接，我再补进 1.10。
2. **Nav2 官方文档路径在 2025 年后改版**：旧的 `docs.nav2.org/configuration/packages/...` 全部 404，
   现行结构是 `docs.nav2.org/rolling/configuration_and_development/configuration_guide/...`。我**逐条抓到 200 才写入**；
   但我们 bench 是 **Humble**，Humble 时代的参数键名/默认值与 rolling 文档可能有差异——**以 `/opt/ros/humble/share/...` 里的实际插件与参数为准**（本报告只保证"文档页存在"，不保证"键名与 Humble 完全一致"）。
3. **标准正文无法直接引用**：ISO 3691-4 / ISO 13849-1 / ANSI-A3 R15.08 的正文是**付费且对脚本返回 403**，
   我只能给官方目录页 + IEC 功能安全总览页。第 6.6 节里"**安全链必须独立于导航计算机**"这一表述是
   **我基于标准框架 + 厂商资料（Pilz/SICK）的综合**，**不是**逐条引用的标准原文；要用于产品认证请以标准正文与认证机构意见为准。
4. **部分厂商页面对脚本返回 403**（Kollmorgen、NDC Solutions、Pilz、ANSI、automate.org、SAGE/Wiley/IEEE/ScienceDirect）：
   这些链接是**官方页面**，但我无法用脚本校验正文内容，只能确认"域名/路径来自官方检索结果"。
   特别地，`ndcsolutions.com` 与 `kollmorgen.com` 的 AGV/NDC8 页面属**厂商宣传资料**，不是中立来源。
5. **AGV 导航范式的"权威综述"只有会议/行业级来源**：磁条/二维码/反射板这几族（8.1–8.4）我找到的是
   IOP 会议论文页（✅ 200 但为会议页）与 MHI 行业博客（✅ 200，**行业协会博客，非同行评议**）；
   **没有**找到近 5 年内、可自由访问、覆盖"磁带 vs 二维码 vs 反射板 vs SLAM"的中立对比综述。8.8 的结论因此标注为"本报告综合"。
6. **RM 生态仓库只做了 README/文件清单级核对**：§B.10 的"用了哪些族"来自各仓库 README 与我们既有的只读深读
   （`docs/cod_nav_2026_deep_dive.md` 等），**未 clone、未编译、未运行**；赛季分支仍可能变化。
   另外：用户提到的 XDU `sentry_navigation` 作为**仓库名**在 GitHub 上是 404，实际活跃仓库是 **`XDU-IRobot/SentryNav2026_XDU`**
   （`sentry_navigation` 是其中的包目录），我按实际存在的仓库写。
7. **Hydra 论文 ID**：我先抓到的 `2105.07664` 与 Hydra 无关（已废弃），改用 ✅ `2201.13360`（标题已核对）。
   **MonoGS 的 arXiv ID 我未能确权**，因此 1.9 只给仓库链接，不给论文 ID。
8. **少数族只有"技术报告/书籍"级来源**：4.1（Pure Pursuit 用 CMU 技术报告 PDF）、9.1（编队控制综述为 Automatica 付费页）、
   9.2（MRTA 为 IJRR 付费页）、9.4（多机探索为 IEEE 付费页）——**可点、权威，但正文需机构订阅**。
9. **未做的事**：我没有逐一复现任何算法的性能数字；本报告里的"代价"判断（例如 MPPI 的 CPU 开销、TEB 的调参成本）
   部分来自我们 bench 的既有实测文档（`docs/algorithm_matrix.md`、`docs/issues_and_findings.md`）与来源页面的定性描述，
   **不是**本次新做的基准测试。


---


## 来源 11/19：`sim/gazebo_gui_troubleshooting.md`


> 现象（用户 2026-10-07）：「gazebo加载不出来吗，rviz倒是挺好的看着」「我感觉电脑资源很丰富阿，
> 不应该加载不出来」；之前还看到过
> `[ERROR] [gzclient-1]: process has died [pid …, exit code -9, cmd 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so']`。
>
> 本文回答：**为什么 RViz 一切正常、Gazebo 就是不出来**；**`-9` 到底是谁杀的**；
> 机器 32 GB 内存是不是真的不够；以及**一条命令的无头跑法**。
>
> 结论先行（四句话）：
> 1. **不是内存问题。** 用户机器 `MemTotal = 31.1 GiB`、28 核；内核日志（本次开机 16:35:56 起
>    完整保留）**没有任何 OOM 记录**，用户会话 cgroup 的 `memory.events` 里 `oom_kill = 0`。
> 2. **`gzclient` 真的会"卡住不出来"**：`robot:=robot11` 的 `<visual>` mesh URI 在本地解析不到，
>    gazebo 于是**回落到在线模型库** `http://models.gazebosim.org/` 并**同步阻塞**。
>    用户 19:50 那次日志里实测卡了 **48.03 s**，而且**是在用户按 Ctrl-C 的那一刻才结束的**
>    —— 也就是说再等下去也不会好。同一条链还导致**车在 Gazebo 里没有 mesh**（RViz 看不到这个问题）。
> 3. **`exit code -9` = 外部 SIGKILL，但发送者无法指认**（无 `auditd`）。⭐ **用户 2026-10-10 回忆：
>    是他自己手动 `pkill -9` 的 —— "因为它太卡了或者直接卡住了"**。另一个候选是本仓 bench/ab
>    脚本收尾的**全机** `pkill -9 -x gzclient`（不看 `GAZEBO_MASTER_URI`），而 16:55 / 16:56
>    那两次被杀时同机确有兄弟任务在跑 bench。
>    ⚠️ **本文件早期版本把后者写成"大概率"结论，现按用户回忆更正**（两者都只能证明"外部 SIGKILL、非 OOM"）。
> 4. **那"它当时为什么卡到要手杀"——这才是要修的东西**：① 启动期 `model://` 解析不到 ⇒ 回落在线
>    模型库并**同步阻塞 48~100 s**（第 2 句，**已修**）；② 跑起来后 **RTF 只有 0.32~0.46**（默认
>    模型 0.79）—— 3 万条 ODE 射线 + 76 个碰撞体 + 加密后的 `slam_toolbox` 节点叠加所致；
>    ③ 在 spawn/插件加载窗口内注入发布者会打出 `gzserver` 段错误（−11）/ `gzclient` abort（−6）。

---

## 0. 想直接照做：三行命令

```bash
# ① 治好"Gazebo 卡住不出来 / 车在 Gazebo 里没有"（根因：robot11 的 model:// mesh 本地解析不到）
#    ★ 2026-10-08 起**已经自动化**：不需要再手工前缀 GAZEBO_MODEL_PATH（机制见 §5.1）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11

# ② 干脆不看 Gazebo 窗口（一条命令无头跑；RViz 照旧）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 gui:=False

# ③ 再叠一层"不再等在线模型库"的兜底
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 gui:=False gazebo_offline:=True
```

`gui` / `gazebo_offline` 都是**opt-in 参数，默认值 = 改造前的行为**（`gui=True`、
`gazebo_offline=False`），不动任何节点的集合与时序。实测三种组合都**PASS**（§5.3）。

> ①里的那条命令在 2026-10-08 之前需要写成
> `GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/install/robot11/share" ros2 launch …`。
> 那个手工写法**仍然有效**（§5.1.4），但已不再是必需的。

---

## 1. 症状分类（先定位你是哪一种）

| 症状 | 典型日志/表现 | 本文对应章节 |
|---|---|---|
| **A. 窗口在，但世界/模型不出来（卡住）** | `[Msg] Waiting for model database update to complete...` 之后长时间无输出 | §3、§4.1 |
| **B. 窗口出来了，场地有、车没了** | `[Err] [Visual.cc:2956] No mesh specified` | §4.2 |
| **C. 进程直接死：`exit code -9`（SIGKILL）** | launch 里**没有** `escalating to 'SIGKILL'` 那几行 | §5 |
| **D. 进程死：`-6` / `-11` / `-2`** | `-6` = abort、`-11` = 段错误（内核日志有 `segfault`）、`-2` = 自己退出 | §5.3 |
| **E. 关窗口关不掉，5 s / 10 s 后被 SIGTERM/SIGKILL** | `failed to terminate '5' seconds after receiving 'SIGINT'` | §5.2（这是**正常**的收尾行为） |
| **F. 黑屏（进程活着、就是不画）** | 见 §7 的像素判据 | §7 |

---

## 2. 用户机器上的取证（只读，含原始行）

取证范围：`~/.ros/log/**`（298 个 run 目录）、`~/.gazebo/`、`journalctl`（含内核日志）。

### 2.1 `~/.ros/log/2026-10-07-19-50-41-310829-weicheng-289578/launch.log`

```
1791373843.7415378 [INFO] [gzclient-1]: process started with pid [289621]
1791373848.7416131 [spawn_entity.py-4] [INFO] [1791373848.741388767] [spawn_entity]: Spawn status: SpawnEntity: Successfully spawned entity [robot]
1791373896.7838168 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
1791373901.7941954 [ERROR] [gzclient-1]: process[gzclient-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
1791373901.8401017 [ERROR] [gzclient-1]: process has died [pid 289621, exit code -15, cmd 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so'].
```

**注意两个时间戳只差 8 毫秒**：`spawn_entity` 报"成功生成 robot"是 `…848.741`，
下一行 `gzclient` 的日志（见下）是 `…848.749`。

### 2.2 `~/.gazebo/client-11345/default.log`（gzclient 自己写的，时间戳格式 = `(秒 纳秒)`）

```
(1791373848 749596472) [Msg] Waiting for model database update to complete...
(1791373896 784088121) [Wrn] [Event.cc:61] Warning: Deleting a connection right after creation. ...
(1791373897 593144202) [Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel [model://robot11/meshes/decimated/l2.stl]
(1791373897 593158806) [Wrn] [SystemPaths.cc:459] File or path does not exist [""] [model://robot11/meshes/decimated/l2.stl]
(1791373897 593170043) [Err] [Visual.cc:2956] No mesh specified
```

读法：

* 这些时间戳的格式是 `(秒 纳秒)`，**不是微秒** —— 自己写解析脚本时别踩
  （本任务踩过：按微秒除 1e6 会得到"启动后 −1791372xxx 秒"这种鬼数字）。
* `848.749 → 896.784` = **48.03 s** 卡在"等在线模型库"。
* `896.784` 和 launch 里 `user interrupted with ctrl-c` 的 `896.7838168` 相差 **0.27 ms**
  ⇒ **这 48.03 s 不是自己走完的，是用户按 Ctrl-C 把它打断的**（gzclient 装了 SIGINT 处理，
  所以它又活了 5 s 才被 launch 升级成 SIGTERM）。**再等下去不会自己好。**
* 卡完之后立刻就是 `model://robot11/meshes/decimated/l2.stl` **解析失败** ⇒ 视觉 mesh 没加载。

### 2.3 三次连续的失败尝试（16:55 / 16:56 / 16:59）

`~/.ros/log/2026-10-07-16-55-26-711094-weicheng-70931/launch.log`：

```
1791363329.1647146 [INFO] [gzclient-1]: process started with pid [70986]
1791363355.8760724 [ERROR] [gzclient-1]: process has died [pid 70986, exit code -9, cmd 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so'].
1791363357.7234998 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
```

`~/.ros/log/2026-10-07-16-56-18-753168-weicheng-73460/launch.log`：

```
1791363381.1753340 [INFO] [gzclient-1]: process started with pid [73500]
1791363392.9428306 [ERROR] [gzclient-1]: process has died [pid 73500, exit code -9, cmd 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so'].
1791363394.6979272 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
```

| run | gzclient pid | 起 | 死 | 活了 | exit code | 用户按 Ctrl-C |
|---|---|---|---|---|---|---|
| 16:55:26 | 70986 | 16:55:29.16 | 16:55:55.88 | **26.7 s** | **-9** | 死后 **+1.85 s** |
| 16:56:18 | 73500 | 16:56:21.18 | 16:56:32.94 | **11.8 s** | **-9** | 死后 **+1.76 s** |
| 16:59:37 | 77009 | 16:59:39.70 | —（用户 16:59:21 先按了 Ctrl-C） | ~41 s | **-15** | 死前 |

**关键：前两次 `-9` 之前，launch 里没有任何 `failed to terminate … escalating to 'SIGKILL'`**
⇒ 这不是 launch 自己杀的，是**外部 SIGKILL**。第三次是用户先 Ctrl-C、launch 正常 SIGTERM（`-15`）。

### 2.4 `.tmp_robot11/`（同机的兄弟任务 bench）在那个时间窗里确实在跑

```
2026-10-07 16:46:09  .tmp_robot11/p3_run_def.log
2026-10-07 16:47:35  .tmp_robot11/p3_run_def2.log
2026-10-07 16:51:33  .tmp_robot11/p3_nav.log
2026-10-07 16:51:43  .tmp_robot11/p3_nav2.log
2026-10-07 17:02:18  .tmp_robot11/p4/baseline_run.log
```

### 2.5 **OOM 有证据吗？没有**

| 检查 | 结果 |
|---|---|
| `journalctl -k -b`（本次开机 16:35:56 → 现在，1309 行，含完整的 16:55–17:00 窗口） | **0 条** OOM / `Killed process` / `Memory cgroup out of memory` |
| `/sys/fs/cgroup/user.slice/user-1000.slice/user@1000.service/memory.events` | `oom 0` / **`oom_kill 0`** / `oom_group_kill 0` |
| `journalctl -u systemd-oomd`（本次开机） | 只有启动/停止，**没有任何 `Killed … due to memory pressure`** |
| 历史上真出现过的 OOM（**不是**这次） | `10月 06 11:30:36 kernel: systemd-oomd invoked oom-killer: … oom-kill:constraint=CONSTRAINT_NONE,…,global_oom,…,task=async_slam_tool,pid=264550`；`10月 01 21:44:25 systemd-oomd[802]: Killed …/vte-spawn-….scope due to memory pressure for /user.slice/user-1000.slice/user@1000.service being 72.74% > 50.00% for > 20s with reclaim activity` |
| `dmesg` | 无权限（`读取内核缓冲区失败: 不允许的操作`）；但 `journalctl -k` 可读且已覆盖该窗口 |

> **结论：16:55 / 16:56 那两次 `gzclient` 的 `exit code -9`，没有任何 OOM 证据。**
> 反面证据是齐全的（内核日志完整且干净 + cgroup `oom_kill=0` + oomd 无记录）。
>
> 但要诚实说明两点：
> * 机器上**确实存在** OOM 历史（10-06 11:30 一次真正的 global OOM，任务转储里
>   `gzserver` 的 RSS 是 74.7 万页 ≈ **2.85 GiB**，是当时最大的进程之一），
>   而且**交换分区已经 99.97% 用满**（`SwapTotal 4095 MiB / SwapFree 1 MiB`）——
>   这台机器的内存余量没有"32 GB"听起来那么宽裕，值得清一下 swap 占用大户。
> * 没有 `auditd`（`systemctl is-active auditd` = inactive），所以**无法从日志里指认是谁发的
>   SIGKILL**。下面的 §5 给出的是"排除法 + 时间线吻合"的结论，不是抓现行。

---

## 3. 机理（一）：为什么 gzclient 会卡在"等在线模型库"

### 3.1 代码路径（gazebo 11 / gazebo-classic，带出处）

1. `sdformat` 的 URDF→SDF 转换会把 `package://<pkg>/...` **改写成 `model://<pkg>/...`**。
   本机可复现（`gz sdf -p`，见 §6.1）：
   ```
   URDF: filename="package://robot11/meshes/decimated/base_link.stl"
   SDF : <uri>model://robot11/meshes/decimated/base_link.stl</uri>
   ```
2. `gazebo/common/SystemPaths.cc` 的 `SystemPaths::FindFileURI()`：
   ```cpp
   if (prefix == "model")
   {
     for (auto iter = this->modelPaths.begin(); iter != this->modelPaths.end(); ++iter)
     { path = boost::filesystem::path(*iter) / suffix; if (exists(path)) {...break;} }

     // Try to download the model from models.gazebosim.org if it wasn't found.
     if (filename.empty())
       filename = ModelDatabase::Instance()->GetModelPath(_uri, true);   // ← forceDownload = true
   }
   ```
   ⇒ **本地找不到就无条件去网上找**（`_forceDownload=true`）。
3. `gazebo/common/ModelDatabase.cc` 的 `GetModelPath()` → `HasModel()` → `GetModels()`：
   ```cpp
   boost::mutex::scoped_try_lock lock(this->dataPtr->updateMutex);
   if (!lock)
   {
     gzmsg << "Waiting for model database update to complete...\n";
     boost::mutex::scoped_lock lock2(this->dataPtr->updateMutex);   // ← 就在这里同步阻塞
   }
   ```
   而抓取线程干的是 `GetDBConfig()` → `GetManifestImpl()` → **`curl_easy_perform()`，
   全程没有设 `CURLOPT_TIMEOUT` / `CURLOPT_CONNECTTIMEOUT`**
   ⇒ 遇到"代理在但不回包"就是**无上限等待**。
4. `ModelDatabase::GetURI()` 用的是环境变量 `GAZEBO_MODEL_DATABASE_URI`，没设就用编译期默认
   `http://models.gazebosim.org/`。
5. 网络路径最后还是失败 → `SystemPaths::FindFile()` 打印
   `File or path does not exist [""] [model://robot11/...]` → `gazebo/rendering/Visual.cc:2956`
   打印 `No mesh specified` ⇒ **这个 `<visual>` 没有几何，车在 GUI 里就是"没有"**。

### 3.2 为什么"只有 robot11 会中招"

| 槽位 | `<mesh>` 视觉数 | `package://` 引用数 | 会不会走 §3.1 的回落 |
|---|---|---|---|
| `''`（本仓默认 `simulation_waking_robot.xacro`） | **0** | 0 | 不会（全是 box/cylinder） |
| `hzmirm` | **0** | 0 | 不会 |
| `robot11` | **12** | **14** | **会** ← 用户 2026-10-07 换的就是这个 |

⇒ 这解释了用户的对比感受：**RViz 一切正常**（RViz 用 `package://` 直接走 ament 索引，
不经过 gazebo 的 `model://` 回落），**只有 Gazebo 卡/不显示**，
而且**是从换成 `robot:=robot11` 之后才开始**。

### 3.3 为什么"本地解析不到"（真正缺的东西）

`gazebo/common/SystemPaths.cc` 里 `model://` 的解析根是
`$HOME/.gazebo/models` **加上** `GAZEBO_MODEL_PATH` 的每一项。
本项目里 `GAZEBO_MODEL_PATH` 来自各包 `package.xml` 的
`<export><gazebo_ros gazebo_model_path="${prefix}/…"/>`（实现见
`/opt/ros/humble/lib/gazebo_ros/gazebo_ros_paths.py`），实测只有一项：

```
GAZEBO_MODEL_PATH = <install>/hzmi_rm_simulation/share/hzmi_rm_simulation/meshes
```

`model://robot11/meshes/decimated/l2.stl` 要解析，需要某个解析根 `R` 满足
`R/robot11/meshes/decimated/l2.stl` 存在。而实际文件在：

```
install/robot11/share/robot11/meshes/decimated/l2.stl      ← 存在（symlink 到 src）
```

⇒ **只差一个解析根 `install/robot11/share`**。而 `install/robot11/share/robot11` 里
**没有 `model.config`**，也没有任何包把 `install/robot11/share` 写进 `GAZEBO_MODEL_PATH`
（本仓没有任何一个 `package.xml` 导出 `gazebo_model_path` 指向它）。

> 顺带：`~/.gazebo/client-11345/default.log` 里那两条
> `[Err] [InsertModelWidget.cc:403] Missing model.config for model "…/meshes/RMUL2024_world"`
> 与 `"…/meshes/obstacles"` 是**同一个根因的另一个表现**：把非模型目录（`meshes/` 下的兄弟
> 目录）放在了 `GAZEBO_MODEL_PATH` 上，Gazebo 会一个个去读 `model.config` 并报错。
> 它**不是**世界加载失败的原因（世界 `model://RMUC2026_world/...` 是能解析的，
> `meshes/RMUC2026_world/model.config` 存在）。

### 3.4 沙箱里的受控复现

见 §6.2 的实测表（`blackhole` 配置 = 用一个"只 accept、永不回包"的本地 TCP 黑洞当代理，
精确复现"代理在但不干活"）。

---

## 4. 机理（二）：`exit code -9` 是谁杀的

### 4.1 先排除"launch 自己杀的"

launch 自己杀会在前面留下证据（三条一组）：

```
[ERROR] [gzclient-1]: process[gzclient-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
[INFO]  [gzclient-1]: sending signal 'SIGTERM' to process[gzclient-1]
[ERROR] [gzclient-1]: process[gzclient-1] failed to terminate '10.0' seconds after receiving 'SIGTERM', escalating to 'SIGKILL'
```

用户 298 个 run 里 `exit code -9` 一共 **202 条**，**绝大多数都是这一种**（关窗口时的正常升级）。
**只有 16:55:55（pid 70986）和 16:56:32（pid 73500）这两条 gzclient 的 `-9` 前面没有这组证据**
⇒ 外部 SIGKILL。

### 4.2 两个候选：① 用户手动 kill（用户回忆）/ ② 本仓 bench/ab 的全机 `pkill -9`

**候选 ①（用户回忆，2026-10-10，现列为首位）**：**用户自己手动 `pkill -9`**。
用户原话：*"pkill -9 这个应该是我手动杀的原因是他太卡了或者直接卡住了"*。
这与已核实的证据**完全一致**（外部 SIGKILL、非 OOM、无 `auditd` 可抓现行），也解释了一个此前没解释的
现象：**用户为什么要杀它 —— 因为它当时真的卡住了**。
⇒ 由此**真正该追问的不是"谁杀的"，而是"它当时为什么卡到要手杀"** —— 见 §4.2.1。

**候选 ②（原结论，降级为候选）**：本仓 bench/ab 脚本收尾的**全机** `pkill -9`。

仓库里有**多处**收尾是这么写的（不看 `GAZEBO_MASTER_URI`、不看是谁起的）：

```
tools/scripts/diag/run_drift_diag.sh:130:          pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/mapping/run_mapping_headless.sh:133: pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/mapping/mapping_ab_run.sh:110:       pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/mapping/nav_map_ab_run.sh:66:        pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/mapping/sltune/run_sltune_ab.sh:104: pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/regress/run_remap_lowterrain.sh:79:  pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/regress/run_nav_clearance_ab.sh:134: pkill -9 -x gzserver
tools/scripts/localization/run_nav_smoke_regress.sh:44,60: pkill -9 -x gzserver
```

而且紧邻的几行还有 **`pkill -9 -f 'ros2 launch'`** —— 这会把**别人**的 `ros2 launch` 一起杀掉。

两个细节值得注意：

* `tools/scripts/regress/run_robot_model_probe.sh` 与 `robot11_gui_bench.sh` 想"只杀自己那个
  master 的"，写的是 `pkill -f "gzclient.*$GZ_PORT"` —— 但 **`gzclient` 的命令行里根本没有端口**
  （`gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so`），所以这个"限定"**永远匹配不到**，
  等于没保护；`gzserver` 那条能匹配（`gzserver <world>` 里也没端口……同样匹配不到）。
* 时间线吻合：16:46–17:02 同机有兄弟任务的 bench（§2.4），两次 `-9` 落在 16:55:55 与 16:56:32。

> **诚实边界**：没有 `auditd`，**无法"抓现行"指认发送者** —— 上面的候选②只是"排除法 + 时间线吻合"，
> 候选①是用户的回忆。可以确定的是：**它不是 OOM，也不可能是资源不足。**
> ⚠️ 本文件早期版本把候选②写成"大概率"，**现更正为两个并列候选，且以用户回忆（候选①）为更可能**。

#### 4.2.1 它当时为什么卡到要手杀（**这才是要修的东西**）

| # | 卡的形式 | 已测根因 | 现在的处置 |
|---|---|---|---|
| 1 | **启动时几十秒什么都不出来**（最像"直接卡住了"） | `robot11` 的 mesh URI 被 sdformat 写成 `model://robot11/...`，本地解析不到 ⇒ gazebo **无条件回落在线模型库并同步阻塞**（用户 19:50 = **48.03 s**、同机 20:13 = **76.34 s**、黑洞复现 = **99.67 s**，**全是被打断的，不设上限**） | ✅ **已修**：launch 侧自动追加 `GAZEBO_MODEL_PATH` + `package.xml` 的 `gazebo_ros gazebo_model_path` 导出（§5.1）⇒ 现在**秒开**、`No mesh specified` 0 |
| 2 | **跑起来后操作迟滞（转视角/点鼠标都顿）** | **RTF 0.32~0.46**（默认模型 0.79）。已量到的三个开销：**3 万条 ODE 射线**（gzserver 3.2 GB、≈94.8 KiB/条）、**76 个碰撞体**、以及我们为触发回环而**加密的 `slam_toolbox` 节点**（0.2 m/0.2 rad/0.25 s） | 可选：`gui:=False`（只留 RViz）· `robot11_visual:=decimated`（**已是默认**）· 不需要时关掉 SLAM/LIO · ⚠️ 降 `<samples>` 能省 GB 级内存与 CPU，但**用户已否决**（仿真点云本来就稀疏） |
| 3 | **真的挂死（无响应）** | 在 **spawn / 插件加载窗口内注入发布者**时，本沙箱实测会把 `gzserver` 打成**段错误（−11）**、`gzclient` **abort（−6）**；另有 `spawn_entity` 服务超时的抖动（§4.3） | 工具已改成 **settle 之后再注入**；日志里仍有偶发 `spawn service timed out`（模型其实已进世界） |

> **一句话**：**"被杀"是结果，"卡"是原因。** 原因 1 已根治（`model://` 解析自动化）；
> 原因 2 是**性能取舍**（射线数/碰撞体/SLAM 节点密度），可用 `gui:=False` 或按需关节点缓解；
> 原因 3 是**时序问题**（注入太早），工具侧已规避。

### 4.3 其余退出码

| code | 含义 | 用户日志里的例子 |
|---|---|---|
| `-6` | abort（`SIGABRT`） | `10月 07 16:59:xx` 附近多次；`.tmp_robot11/p4/baseline_run.log` 里也有 `[gzclient-1] … exit code -6` |
| `-11` | 段错误 | `10月 06 20:50:18 kernel: gzclient[3600071]: segfault at 18 ip … in libc.so.6[7b4cc4028000+195000] likely on CPU 24` |
| `-2` | 自己退出（`SIGINT` 默认动作） | 多条历史记录 |
| gzserver `-11` | `10月 07 14:35:59 / 14:39:11 kernel: gzserver[1622157/1626342]: segfault at 158 ip … in libgazebo_ode.so.11.10.2` | 物理引擎侧段错误，与 GUI 无关 |

---

## 5. 修法

> 分级：**【修法】** = 消除根因；**【绕过】** = 不改根因但让你能干活；**【环境】** = 机器/环境层面。

### 5.1 【修法·本项目｜★ 2026-10-08 已自动化】让 `model://robot11/...` 本地可解析

> **现在一条命令就够**（不需要任何手工环境变量）：
> ```bash
> ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 mode:=nav \
>   lio:=small_point_lio localization:=gicp nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True
> ```
> 手工前缀环境变量的老办法**仍然有效**，但只作 fallback（§5.1.4）。

#### 5.1.1 现在是谁在修：两条腿，互为兜底

| # | 机制 | 落点（文件 / 键） | 贡献的解析根 | 生效范围 |
|---|---|---|---|---|
| (a) **启动侧** | `rm_nav_bringup/launch/bringup_sim.launch.py` 与 `hzmi_rm_simulation/launch/rm_simulation.launch.py` 里的 `_gazebo_model_path_setup()`（`OpaqueFunction`，**必须排在 include gzserver/gzclient 之前**），用 `AppendEnvironmentVariable('GAZEBO_MODEL_PATH', …)` | `<install>/robot11/share`（= `os.path.dirname(get_package_share_directory('robot11'))`） | 只有 `robot:=robot11`（真带 `<mesh>` 视觉的槽位）会执行；默认模型 / `hzmirm` 一条都不加 ⇒ 它们的 env / 日志 / 时序**逐字节不变**；追加不覆盖用户原值；用户已手工 export 过同一目录时幂等跳过 |
| (b) **包侧** | `src/simulation/rm_simulation/robot11_description/package.xml`：`<export><gazebo_ros gazebo_model_path="${prefix}/.."/></export>` | `<install>/robot11/share/robot11/..`（同一个目录，字面量带 `..`） | **任何** gazebo 入口都吃得到：本仓 launch、裸 `ros2 launch gazebo_ros gzserver.launch.py` / `gzclient.launch.py`、`gz sim`（实现：`/opt/ros/humble/lib/gazebo_ros/gazebo_ros_paths.py` 扫描各包 package.xml 后把 `${prefix}` 换成本包 share，再拼进 `ExecuteProcess` 的 `additional_env`） |

**哪条在实际运行里干活？** 实测（§5.1.3 的 env dump）一次 `robot:=robot11` 的 launch 跑起来后，
gzserver/gzclient 的 env 是：

```
GAZEBO_MODEL_PATH=/…/install/robot11/share/robot11/.. : /…/install/hzmi_rm_simulation/share/hzmi_rm_simulation/meshes : /…/install/robot11/share
                  └────── (b) 包 export 贡献 ──────┘   └──────── 改造前就有（world/obstacle 的 model://）────────┘   └── (a) launch 贡献 ──┘
```

* gazebo 是按 `modelPaths` 顺序找的 ⇒ **先命中的是 (b) 那条**（`boost::filesystem::exists()` 会把字面量里的 `..` 解开，实测可用）；
* 但 **(a) 才是"本仓 launch 一定可用"的保证**：它不依赖那份 package.xml 是否重建过、能不能被
  `catkin_pkg` 解析，只要 `install/robot11/share` 在就一定加进去；把 package.xml 拿开（模拟"没重建过的旧 install"）
  后 (a) 单独实测同样 0 错误、12 个 mesh 全部读进来（§5.1.2 的 P2）；
* (b) 的价值在"**我们 launch 之外的 gazebo 入口**"：裸 gzserver/gzclient、直接开 world 文件、`gz sim`
  都只认 package.xml 那条。不设任何环境变量、只留 (b) 时实测同样 0 错误（§5.1.2 的 P3）。

#### 5.1.2 三组对照实测（2026-10-08，Xvfb + 软件 GL，黑洞代理）

协议（= §7 / §6.3 的同一套，**只有这条协议会让 gzclient 去建模型并解析 mesh**）：
起 gzserver → 等 `Init world` → `spawn_entity.py -file`（`robot11_visual:=decimated`）→ **再起 gzclient**。

| | (a) 启动侧根 | (b) 包 export 根 | 「等在线模型库」 | `No mesh specified` | `SystemPaths.cc:459` | `FuelModelDatabase.cc:313` | gzclient 事件数 | gzclient 读入 `rchar` |
|---|---|---|---|---|---|---|---|---|
| `asis`（改造前） | ✗ | ✗ | **出现，stall 13.84 s**（我把黑洞代理拆掉它才结束，**不设上限**） | **24** | **36** | **36** | **184** | 3.90 MB（只有世界资产，**没读** robot11 的 mesh） |
| **只有 (a)** | ✓ | ✗ | 没有 | **0** | 0 | 0 | **21** | **12.16 MB**（+8.26 MB） |
| **只有 (b)** | ✗ | ✓ | 没有 | **0** | 0 | 0 | **21** | **12.16 MB**（+8.26 MB） |

读法（重要）：

* `No mesh specified = 0` **只有在"确实读了 mesh"的前提下才是证据** —— 加载成功时 gazebo
  **一个日志都不打**，所以必须同时看"客户端到底读进来多少字节"。上表：12.16 − 3.90 =
  **8.26 MB ≈ 12 个 decimated mesh 的 8.20 MB**（`stat` 求和 = 8,200,758 B），且 0.1 s 一次的
  fd 扫描抓到了正在打开的 `…/meshes/decimated/l5.stl` / `l11.stl` ⇒ 这是**正证据**，不是"没报错"。
* `asis` 那行的 **24 / 36 / 36 / 184** 与 §6.3（2026-10-07 量的 24 / 36 / 36 / 183）**逐项相同**
  ⇒ 两组日期的复现口径一致（事件数差 1 是收尾时多一条 `Event.cc` 告警）。
* 两条根**各自单独就够**，所以 §5.1.1 的"哪条在干活"不是"二选一"，而是"先命中 (b)，缺 (b) 时 (a) 顶上"。
* 代价（诚实说明）：两个解析根是**等价目录**，`InsertModelWidget` 扫 `GAZEBO_MODEL_PATH` 时会各扫一遍
  ⇒ 带 GUI 的 robot11 跑，`Missing model.config` 这类**噪音**从 3 行变 6 行（gzclient 事件数
  21 → 27，见 §5.1.3）。它只是"插入模型"面板扫目录时的告警，**不影响加载/渲染**。

#### 5.1.3 用户那条命令（`robot:=robot11`，无任何手工环境变量）实测

| 项 | 实测值 |
|---|---|
| 命令 | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 gui:=True gazebo_offline:=False robot:=robot11 mode:=nav lio:=small_point_lio localization:=gicp nav:=mppi planner:=smac2d nav_rviz:=True spin_speed:=0.0` |
| launch 日志里那一行 | `[gzmodel] robot:=robot11 需要 robot11 的本地解析根：GAZEBO_MODEL_PATH 追加 /…/install/robot11/share（追加，不覆盖原值：<空>；…）` |
| gzserver / gzclient 的 `GAZEBO_MODEL_PATH` | 三条（见 §5.1.1 的展开），**两条 robot11 根都在** |
| launch → spawn | **8.06 s / 8.81 s / 9.08 s**（三次；launch 自己的 `spawn_entity.py` 成功） |
| `Waiting for model database update to complete...` | **没有出现**（黑洞代理仍然挂着 ⇒ 与网络无关） |
| `No mesh specified` / `SystemPaths.cc:459` / `FuelModelDatabase.cc:313` | **0 / 0 / 0** |
| gzclient 事件数 | **27**（含 8 行 `Missing model.config` 噪音；改造前同一条命令是 **275**，且含 stall） |
| `/odom` | 34 条/20 s（1.7 Hz；另一次 72 条/30 s = 2.39 Hz）—— 速率的限制是 **RTF 0.17**（软件 GL + 30000 条 ODE 射线），不是里程计本身 |
| `/segmentation/ground` | 6640 pts/帧（中位；另一次 6648） |
| `/scan` | 1462 波束/帧，其中有限值 510（另一次 504），最远 7.9 m |
| 节点集合 | 完整且与改造前**逐个相同**（`small_point_lio`、`gicp_registration`、`map_server`、`controller_server`、`planner_server`、`bt_navigator`、`rviz`…；只有 rviz 内部 `transform_listener_impl_<随机 hex>` 这种带随机后缀的伪节点名不同） |

**同一条命令的 before/after 对照（改造前那侧 = HEAD 的 launch + 拿掉包 export，其余逐项相同）：**

| | 改造前（`A1b`） | 现在（`F1b`） |
|---|---|---|
| launch → spawn | 8.57 s | 9.08 s |
| 「等在线模型库」 | **出现，stall = 51.64 s**（我把黑洞拆了它才结束） | 没有 |
| `No mesh specified` / `SystemPaths` / `Fuel` | **34 / 56 / 56** | **0 / 0 / 0** |
| gzclient 事件数 | **275** | **27** |

**轻量入口（`rm_simulation.launch.py`，同一条 robot11 路径）也有同一组对照**，
而且这一组抓到了**正面证据**（`gzclient` 真的把 12 个 mesh 读进来了）：

| | 改造前（`L1r`） | 现在（`L4r`） |
|---|---|---|
| 「等在线模型库」 | **出现，stall = 35.46 s** | 没有 |
| `No mesh specified` / `SystemPaths` / `Fuel` | **45 / 78 / 78** | **0 / 0 / 0** |
| gzclient 事件数 | **374** | **28** |
| gzclient `rchar`（全程） | 4.21 MB（只有世界资产） | **12.47 MB**（+8.26 MB ≈ 12 个 mesh 的 8.20 MB） |
| fd 扫描抓到的 `.stl` | 无 | `…/robot11_description/meshes/decimated/l9.stl`（T+4.60 s） |

> ⚠️ **一处必须知道的口径**：`No mesh specified` 的次数**不是一个固定常数**
> （2026-10-07 那次 24、本次 before 侧 34 / 45），它 = "客户端处理了多少轮场景消息 × 12 个 mesh × 2"，
> 随窗口长度/时机变；**判据是"是不是 0"**，不是"等不等于 24"。
> 同理，"修好之后 0 错误"**只有在同时看到 mesh 被读进来时才构成正面证据** ——
> 本沙箱里 gzclient 对"运行期插入的模型"处理**不稳定**（同样配置的几次跑里，有的会去建模型、
> 有的整个窗口都不建；改造前后都有这个现象）⇒ 上表 `L4r` 那一列（12.47 MB + fd 命中）才是
> "解析链真的通了"的证据，`F1b` 那一列只是"**没有**再出现那条回落路径"。
> 这属于 gazebo classic 客户端在本沙箱的行为，与本次修法无关（同一个不确定性在 `asis` 上也存在）。

#### 5.1.4 手工做法（fallback，仍然有效）

**(a) 启动时追加一个 `GAZEBO_MODEL_PATH` 解析根**（不改任何文件）

```bash
GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/install/robot11/share" \
ros2 launch rm_nav_bringup bringup_sim.launch.py robot:=robot11 world:=RMUC2026 ...
```

（launch 侧的 `_gazebo_model_path_setup` 认得这个目录：已经在 `GAZEBO_MODEL_PATH` 里就**不重复追加**，
日志里会打 `[gzmodel] … 已存在（无需追加）`。）

**(b) 包侧 export**（持久，已经进了仓库）

```xml
<gazebo_ros gazebo_model_path="${prefix}/.."/>
```

`${prefix}` = 该包的 share 目录 = `<install>/robot11/share/robot11`，所以解析根 = `${prefix}/..`
= **`<install>/robot11/share`**（要往上一级，因为 `model://robot11/meshes/…` 要求解析根 R 满足
`R/robot11/meshes/…` 存在）。这条**已实测**（§5.1.2 的 P3：拿掉 (a) 后仅靠它，0 错误 + mesh 全读进来）。

**(c) 一条命令无头跑，完全不碰 GUI** ⇒ 见 §5.3 的 `gui:=False`。

#### 5.1.5 静态验证（不用起 Gazebo，三行）

```bash
# ① 本地解析应当命中（两个拼法都指同一个目录）
ls -l install/robot11/share/robot11/meshes/decimated/l2.stl
ls -l install/robot11/share/robot11/../robot11/meshes/decimated/l2.stl
# ② 确认 sdformat 确实把 package:// 改写成了 model://（应当数出 12 条）
xacro src/bringup/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro robot11_visual:=decimated > /tmp/r11.urdf
gz sdf -p /tmp/r11.urdf | grep -c 'uri>model://robot11'
# ③ 包侧的 export 是否真的进了 install/（重建后才会有）
grep gazebo_model_path install/robot11/share/robot11/package.xml
# ④ 起 GUI 后应当**看不到**这两行
grep -c "No mesh specified\|Waiting for model database" ~/.gazebo/client-*/default.log
```

#### 5.1.6 无头二次独立复核（2026-10-08 接手复核；**不开 Xvfb、不开 GUI**）

上面 §5.1.2/§5.1.3 走的是"Xvfb + 软件 GL + 黑洞代理"那条协议（因为要复现 **gzclient** 的卡顿）。
复核只需要回答"**裸命令能不能解析到 mesh**"与"**哪条机制在干活**"⇒ 无头 `gui:=False` 就够，
而且更干净：没有 gzclient 的事件数/字节数这类不稳定量，`No mesh specified` 直接看 gzserver。

一条命令（新工具，只清**本 master URI** 上的 gazebo 进程）：

```bash
tools/scripts/regress/verify_gzmodel_autopath.sh obj1_plain --settle 50 -- \
    world:=RMUC2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0
```

| 项 | `robot:=robot11`（裸命令，`GAZEBO_MODEL_PATH` 启动前**未设置**） | 默认模型（`robot` 留空） |
|---|---|---|
| `gzserver` 真身的 `GAZEBO_MODEL_PATH` | `…/install/robot11/share/robot11/..` **:** `…/install/hzmi_rm_simulation/share/hzmi_rm_simulation/meshes` **:** `…/install/robot11/share`（**三项，robot11 根出现 2 次**） | `…/install/robot11/share/robot11/..` **:** `…/install/hzmi_rm_simulation/share/…/meshes`（**两项**） |
| 条目 [0] 的来源 | (b) 包 export（`${prefix}/..`，**排在 modelPaths 最前 ⇒ 先被命中**） | 同左（(b) 对**所有** gazebo 入口都生效） |
| 最后一项的来源 | (a) launch 的 `AppendEnvironmentVariable` | **没有**（(a) 只对 `robot:=robot11` 执行） |
| `[Err] … No mesh specified` | **0**（日志里另一处 "No mesh specified" 是我们自己 `[gzmodel]` 说明文字，见下） | **0** |
| `Waiting for model database update` / `SystemPaths.cc:459` / `FuelModelDatabase` | **0 / 0 / 0** | **0 / 0 / 0** |
| `SpawnEntity: Successfully spawned entity [robot]` | **1** | **1** |
| `ros2 node list` | **33** 个 | **33** 个，与左列**逐个相同**（只差 `transform_listener_impl_<随机 hex>` 这种 ROS 内部伪节点名） |
| `[gzmodel]` 日志行 | **2** 条（解释见下） | **0** 条 |

**关于"日志应当只有一条"**：`robot:=robot11` 时确实会打印 **2** 条 `[gzmodel]`，
但**只有一条真的追加**：
* 第 1 条（`bringup_sim.launch.py`，日志第 4 行）= "**追加** …/install/robot11/share"；
* 第 2 条（日志第 5 行）= 紧跟其后被 include 的 **`hzmi_rm_simulation/rm_simulation.launch.py`**
  自己那份 `_gazebo_model_path_setup()`，它看到的 env **已经**有这条根 ⇒ 打印
  "**已存在（无需追加）**"并不动 env（幂等分支，这正是设计要的行为；
  `bringup_sim.launch.py:2371` 的 `OpaqueFunction` 与 `:2372` 的
  `ld.add_action(start_rm_simulation)` 相邻，而 `start_rm_simulation` 就是那个 include）。
* 证据（可复算）：**单独**跑轻量入口 `ros2 launch hzmi_rm_simulation rm_simulation.launch.py
  robot:=robot11 world:=RMUC2026 gui:=False`（不带 bringup）⇒ `[gzmodel]` **1 条**（"追加"）、
  `Successfully spawned entity` 1、`No mesh specified` **0**、env 与上表**逐项相同**（三项）。
  ⇒ 两次打印是"两个入口各说一句话"，不是"同一个入口跑了两遍"。

**包侧 export 在"真构建"之后确实生效（本次补测，之前一直没做）**：

| 测法 | 结果 |
|---|---|
| `colcon build --symlink-install --packages-select robot11 rm_nav_bringup` 后直接问 gazebo_ros 自己那个函数（`/opt/ros/humble/lib/gazebo_ros/gazebo_ros_paths.py: GazeboRosPaths.get_paths()` —— `gzserver.launch.py`/`gzclient.launch.py` 用的就是它） | 返回 `…/install/robot11/share/robot11/..`（`${prefix}` 已被替换成本包 share） |
| **非 symlink** 的真安装（`colcon build --packages-select robot11 --build-base /tmp/rb --install-base /tmp/ri` ⇒ `share/robot11/package.xml` 是**真实拷贝** 3270 B，不是符号链接） | 同样返回 `…/tmp/ri/robot11/share/robot11/..` ⇒ export 是随 `ament_package()` 装进去的，**不是**"只因为 install/ 里是符号链接" |
| `gz sdf -p` 生成的 SDF | 12 条 `<uri>model://robot11/meshes/decimated/*.stl</uri>`（`<uri>` 总数 25，另外 13 条是 gazebo 材质），`package://` **0** 条 |
| 逐条 `model://` 落到磁盘 | 12/12 命中（`realpath(root + '/robot11/' + rest)` 存在） |
| 生成物与 HEAD 的对照（`livox_mount` 默认档） | 去注释后**逐字节相同**（两份都是 35410 B、`sha256` 相同）；显式 `livox_mount:=plugin` 与不传**同一 sha** |

**⇒ 哪条是"权威"？** 不是二选一，但优先级是明确的：**运行的这条命令里是 (b) 先命中**
（它在 `GAZEBO_MODEL_PATH` 的第 0 项）；**(a) 的价值是"不依赖那份 manifest"**
（没重建过/解析不了也照样加）；**(b) 的价值是"launch 之外的 gazebo 入口"**。

> ⚠️ **一处必须说的偏离**（§5.1.1 与 `robot_models.md` §14.2 里"一个字节都没动"这句只对 (a) 成立）：
> (b) 是 `package.xml` 的 export ⇒ **任何** gazebo 入口（含默认模型、其它 world、裸 gzserver）
> 的 `GAZEBO_MODEL_PATH` 都会多出 `…/install/robot11/share/robot11/..` 这一项 —— 这是
> "包侧兜底"的必然代价，**不是** 0 改动。实测它的副作用**在无头跑里量不出来**
> （默认模型跑：`No mesh specified` 0、`Missing model.config` 0、节点集合与 robot11 跑逐个相同）；
> 有 GUI 时它会让"插入模型"面板多扫一个目录（`Missing model.config` 噪音 3 → 6 行，§5.1.2 尾部）。
> 想彻底去掉这条：删掉 `package.xml` 的那一行并重建 robot11，此时只剩 (a)，
> 代价是裸 `gzserver`/`gz sim` 入口又会回落在线模型库。

### 5.2 【修法·环境】不要再让 gzclient 等在线模型库

**不要**用 `GAZEBO_MODEL_DATABASE_URI=""`（空串）：`ModelDatabase::GetURI()` 里有
`result[result.size()-1]`，对空串 `size()-1` 会下溢成 `SIZE_MAX` ⇒ **越界读（UB）**。
用一个"必然立刻失败"的地址：

```bash
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:1/"     # 连接立刻被拒 ⇒ 不等
# 或者干脆在 /etc/hosts 里钉死（需要 root）
#   127.0.0.1 models.gazebosim.org
```

本项目已把它做成开关（默认关 = 行为不变）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py gazebo_offline:=True ...
```

> 注意：**这条只是"不再等"，不会让车出现**（车不出现是 §5.1 的事）。

### 5.3 【项目新增】`gui:=True|False` —— 一条命令无头跑

`bringup_sim.launch.py` 与 `rm_simulation.launch.py` 都新增了（**纯增量，默认值 = 改造前行为**）：

```bash
# 默认：和以前逐字一样，起 gzclient
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11

# 无头：只起 gzserver，RViz 照旧由 nav_rviz/lio_rviz 控制
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 gui:=False

# 无头 + 不等在线模型库
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 gui:=False gazebo_offline:=True

# 只看 Gazebo 起没起来（不含感知/导航/LIO），用 rm_simulation
ros2 launch hzmi_rm_simulation rm_simulation.launch.py world:=RMUC2026 robot:=robot11 gui:=False
```

**「杀掉 GUI、留下 gzserver」的等价手工做法**（老版本/别的入口）：

```bash
pkill -x gzclient                    # 默认 SIGTERM，够用
# ⚠️ 千万不要在别人还在看 GUI 的时候跑 pkill -9 -x gzclient（全机范围）
# ⚠️ 想只动自己那一份：先查 PID，再按 PID kill
pgrep -a -x gzclient                 # 看命令行/父进程确认是自己的
kill "$(pgrep -x gzclient)"          # 只杀这一个
```

RViz 侧不受影响：`nav_rviz:=False` / `lio_rviz:=False` 各自独立。

**端到端验证（本次实测，不是读代码）**：

| 用例 | 判定依据 | 结果 |
|---|---|---|
| `hzmi_rm_simulation rm_simulation.launch.py world:=RMUC2026 robot:=robot11 gui:=False` | gzclient 在 50 s 窗口内出现 **0/50** 秒；gzserver 50/50 | **PASS** ✅ |
| 同上但**不带** `gui`（默认 True） | gzclient 出现 **48/50** 秒；gzserver 50/50 | **PASS** ✅ |
| `rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 mode:=mapping lio:=none nav_rviz:=False gui:=False` | gzclient **0/60** 秒；gzserver 58/60；本 run 的 `launch.log` 里只有 `gzserver-4` 的 `process started`，**没有** `gzclient-*` | **PASS** ✅ |

> ⚠️ 判定别用机器级 `pgrep -x gzclient`：第一次测的时候，上一轮的 gzclient 因为
> `pkill -f "gzclient.*$PORT"`（**永远匹配不到**，见 §4.2）没被清掉，把结果污染成假 "up"。
> 现在改成"开跑前按名字清干净 + 以本 run 的 `launch.log` 为准"。

### 5.4 【修法·本仓工具链】别再全机 `pkill -9 -x gzclient`

把 §4.2 那几处改成"只杀自己起的进程组"：

```bash
# 反例（现在）：全机杀，会把别人正在看的 GUI 打成 exit code -9
pkill -9 -x gzserver; pkill -9 -x gzclient

# 正例：脚本自己用 setsid/进程组起 gzserver，收尾只杀自己那个组
setsid gzserver ... & PGID=$!
trap 'kill -9 -- "-$PGID" 2>/dev/null' EXIT
```

（这一项本次**没有改动**那些脚本 —— 见 §9 未做清单。）

### 5.5 【绕过】给 gzclient 加超时前先确认是不是真的卡

```bash
# 现场看 gzclient 卡在哪（等价于本文 §2.2 的取证）
tail -f ~/.gazebo/client-*/default.log
# 有 "Waiting for model database update to complete..." 且长时间不动 ⇒ §5.2
# 有 "No mesh specified" ⇒ §5.1
```

---

## 6. 实测

> 全部在**软件渲染**（Xvfb 私有 display + llvmpipe / Mesa 23.2.1，无 GPU——本沙箱
> `nvidia-smi` 报 "couldn't communicate with the NVIDIA driver"）上量的。
> 隔离：`HOME=<repo>/.tmp_gzgui/<tag>/gzhome`、非默认 `ROS_DOMAIN_ID`、
> 专用 `GAZEBO_MASTER_URI`、专用 X display，退出时清理。
> ⚠️ 本沙箱 `/tmp` 每次 bash 调用都是新 tmpfs ⇒ `HOME` 不能放 `/tmp`。

### 6.1 静态事实

| 项 | 值 |
|---|---|
| `gz sdf -p`（robot11 URDF→SDF） | `package://robot11/meshes/decimated/base_link.stl` → `<uri>model://robot11/meshes/decimated/base_link.stl</uri>` |
| `model://RMUC2026_world/meshes/RMUC2026.stl` | **可**解析（`GAZEBO_MODEL_PATH` 上的 `meshes/RMUC2026_world/` 有 `model.config`） |
| `model://robot11/meshes/decimated/*.stl` | **不可**解析（没有任何 `GAZEBO_MODEL_PATH` 项能满足 `R/robot11/meshes/…`） |
| 沙箱到 `models.gazebosim.org` | `http=200, time_total≈0.93 s, 9598 B`（走代理）⇒ 默认环境**不会**卡 |

### 6.2 网络假说 · 受控实测（**结论与直觉相反，所以更要说清**）

四组配置都**没有** spawn 机器人（探针的 `spawn_entity.py` 参数当时写错了，车没生成）——
这反而成了一个**关键的阴性对照**：

| # | `GAZEBO_MODEL_DATABASE_URI` | 代理 | 有车? | 出现 X 窗口 | 首帧非黑 | 「等在线模型库」 | gzclient 峰值 RSS |
|---|---|---|---|---|---|---|---|
| 1 | 未设 | 可用（`database.config` 回 200，0.93 s） | ✗ | 0.7 s | 3.1 s | **无** | 450 MiB |
| 2 | 未设 | **黑洞**（accept 后永不回包） | ✗ | 0.7 s | 3.1 s | **无** | 444 MiB |
| 3 | `http://127.0.0.1:1/` | 黑洞 | ✗ | 0.7 s | 3.2 s | 无（`Unable to connect … [http://127.0.0.1:1//database.config]`） | 450 MiB |
| 4 | `""`（空串） | 黑洞 | ✗ | 0.8 s | 3.1 s | 无（`Unable to connect … [//database.config]`） | 449 MiB |

**读法**：光有"代理挂了"**并不会**让 GUI 卡住（第 2 行：窗口 0.7 s、首帧 3.1 s、全程没提过模型库）。
⇒ **在线模型库本身不是根因**；它只有在"GUI 线程也需要模型清单"时才会变成根因。
（第 3/4 行说明：把 URI 指到必败地址并不会让启动变慢，也不会崩 —— 可以放心当兜底开关用。）

**三次"真卡住"的观测**（都在**有** `robot:=robot11` 的 mesh 视觉时出现）：

| 场景 | 「等在线模型库」起 → 止 | stall | 结束原因 |
|---|---|---|---|
| 用户 2026-10-07 **19:50**（`~/.gazebo/client-11345/default.log`） | `1791373848.749596` → `1791373896.784088` | **48.03 s** | launch 的 `user interrupted with ctrl-c` 在 `1791373896.7838168`，**与"止"相差 0.27 ms** ⇒ **是 Ctrl-C 打断的，不是自己好的** |
| 同机 **20:13** 另一次 `bringup_sim … robot:=robot11`（**代理可用**，不是本任务的探针） | `1791375197.611883` → `1791375273.952294` | **76.34 s** | 同一 run 的 Ctrl-C 在 `1791375273.9524972`（**相差 0.5 ms**）⇒ 同样是打断的 |
| 本任务黑洞复现（`mesh_asis`） | `1791375142.358570` → `1791375242.031...` | **99.67 s** | 我把黑洞进程拆掉的那一刻（探针窗口 100 s 到头） |

⇒ **stall 不设上限**，48 / 76 / 99.7 s 三个数字都只是"人先受不了"或"我把代理拆了"。
为什么"网络明明通"也要几十秒：`ModelDatabase::UpdateModelCacheImpl()` 拿到 `database.config`
之后，会**对清单里每一个模型再发一次 HTTP 请求**去取它的 `model.config`
（`this->dataPtr->modelCache[fullURI] = ModelDatabase::GetModelName(fullURI);` ⇒
`GetModelConfig()` ⇒ `curl_easy_perform`）。清单里是几百个模型 ⇒ 几百次串行请求。
而这条路径**全程没有超时**，所以"代理通、但慢/不稳"也照样能卡几十秒到几分钟。

### 6.3 mesh 链路 · 根因与修法对照（**有**车）

同一个世界（RMUL2026）、同一个 `robot11` URDF（`robot11_visual:=decimated`）、同一个黑洞代理，
只差一个环境变量：

| 项 | `asis`（默认 `GAZEBO_MODEL_PATH`） | `fixed`（追加 `<install>/robot11/share`） |
|---|---|---|
| `model://robot11/meshes/decimated/l2.stl` 本地可解析 | **no** | **yes** |
| 出现「等在线模型库」 | **是**，stall = **99.67 s**（不设上限） | **没有** |
| `[Err] [Visual.cc:2956] No mesh specified` | **24 次** | **0** |
| `[Wrn] [SystemPaths.cc:459] File or path does not exist` | **36 次** | **0** |
| `[Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel` | **36 次** | **0** |
| `[Wrn] [ModelDatabase.cc:340] Getting models from[…]`（重试风暴） | **36 次** | **0** |
| 解析失败的 `model://` URI | **12 个**（`base_link,l2…l12` 全部） | 0 |
| gzclient 自己的日志跨度 | 183 个事件 / 101.2 s | **20 个**事件 / 71.2 s |

`asis` 那 12 个 mesh 的**逐条原始行**（黑洞代理下，节选）：

```
(1791375142 358570479) [Msg] Waiting for model database update to complete...
(1791375242 031      ) [Wrn] [ModelDatabase.cc:212] Unable to connect to model database using [http://models.gazebosim.org//database.config]. Only locally installed models will be available.
(1791375242 31210947 ) [Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel [model://robot11/meshes/decimated/l2.stl]
(1791375242 31227369 ) [Wrn] [SystemPaths.cc:459] File or path does not exist [""] [model://robot11/meshes/decimated/l2.stl]
(1791375242 31246268 ) [Err] [Visual.cc:2956] No mesh specified
```

⇒ **`fixed` 一改，两件事同时消失**（不再等模型库 + mesh 全部加载）。
`fixed` 的代价只是 `InsertModelWidget` 多 3 条 `Missing model.config` 的**噪音**
（`ament_index` / `colcon-core` / `robot11` 三个子目录没有 `model.config`），
它们是"插入模型"面板扫目录时的告警，**不影响加载**。

### 6.4 资源画像与 gzserver 内存记账

**一次干净的分阶段记账**（`world=RMUC2026`、`robot=robot11`、`robot11_visual=decimated`、
带 GUI；每 2 s 采一次 `/proc/<pid>/status`，探针开跑前把机器上的 `gzserver`/`gzclient` 按名字清干净）：

| 阶段 | 采样 | 时长 | gzserver RSS 首 → 末（峰值） | 平均 CPU | 斜率 | 相对上一阶段的增量 |
|---|---|---|---|---|---|---|
| **P0** 只加载世界（RMUC2026，含 2.19 MB 的 `RMUC2026.stl`） | 20 | 39 s | **267.8 → 267.8 MiB**（267.8） | 20.9 % | +0.03 MiB/min | — |
| **P1** spawn `robot11`（含 Livox ray 插件） | 20 | 39 s | **3198.0 → 3198.0 MiB**（3198.0） | 37.7 % | +0.04 MiB/min | **+2930.1 MiB (+2.86 GiB)** |
| **P2** 静默（GUI 同时在线） | 165 | **333 s** | **3199.8 → 3199.9 MiB**（3199.9） | 66.9 % | **+0.02 MiB/min** | +1.8 MiB |

其它峰值：

| 进程 | 峰值 RSS | 峰值 CPU（软件 GL，仅参考） |
|---|---|---|
| `gzclient` | **483.7 MiB** | 444–609 %（多线程软件光栅化） |
| `gzserver` | **3199.9 MiB** | 66.9 %（均） |
| `rviz2` | 本次未起（`--gui yes` 只起 Gazebo） | — |

> **结论 1（~3.2 GB 是谁的）**：世界只占 **268 MiB**；**2.86 GiB 全部来自 spawn `robot11` 那一步**，
> 而且是一次性的固定分配（P1 内 39 s 一动不动）。世界 mesh（2.19 MB）**不是**原因，
> 视觉档位也不是（与既有实测"full/decimated 的 gzserver RSS 完全相同"一致）。
>
> **结论 2（有没有泄漏）**：P2 静默 **333 s（>5 min）** 里 gzserver 只涨了 **1.8 MiB**
> ⇒ **+0.02 MiB/min，没有可测的泄漏**。所以"跑久了越来越卡"不是内存泄漏。
>
> **结论 3（OOM 在用户机器上可不可信）**：单看 gzserver 3.2 GiB + gzclient 0.48 GiB + rviz2 0.3 GiB
> + nav2/LIO 十几个节点，整栈峰值大约 6~8 GiB；用户机器 `MemTotal = 31.1 GiB`
> ⇒ **单纯从容量看，OOM 完全不该发生**（这也印证了 §2.5：内核日志里确实没有 OOM）。
> 但要注意 **swap 只剩 1 MiB**（§8 第 4 条）：容量够不代表没有内存压力抖动。

#### 6.4.1 那 2.86 GiB 具体是**哪个东西**（同一世界、同一 URDF，只剥掉一段）

把 robot11 URDF 里 `<gazebo reference="livox_frame"> … </gazebo>` 整段（= `type="ray"` 传感器 +
`libros2_livox.so` 插件）**删掉**，其余（12 个 mesh 视觉、115 个 collision、IMU、mecanum 插件）
逐字节不变，再跑同一套记账：

| 配置 | P0 只有世界 | P1 + robot11 | P1 的增量 |
|---|---|---|---|
| RMUC2026 + robot11（**含** Livox ray 插件） | 267.8 MiB | **3198.0 MiB** | **+2930.1 MiB** |
| RMUC2026 + robot11（**剥掉** Livox ray 插件） | 179.4 MiB | **307.4 MiB** | **+128.0 MiB** |

（两次 P0 不同是因为前一次带 GUI、后一次无头 —— 但**增量是同一次 run 内量的**，可以直接比。）

⇒ **2.74 GiB（2930 − 128）来自那段 Livox ray 插件**；机器人本身（12 个 mesh 视觉 +
115 个 collision + IMU + 底盘控制器）只有 **128 MiB**。

内存**形态**也对得上（`smaps` 排序）：

| | 最大的映射 |
|---|---|
| 含插件 | `rw-p [heap]` **2776.0 MiB** rss（虚拟 2 843 900 MiB） |
| 剥掉插件 | `rw-p [heap]` **46.5 MiB** rss |

⇒ 2.78 GiB 全在 **glibc 堆上**，是**海量小对象**，不是一块大 buffer。
按 `sample=30000` 折算：**≈ 94.8 KiB / 条射线**。成因在插件源码里能对上：
`livox_laser_simulation_RO2/src/livox_ode_multiray_shape.cpp` 的 `AddRay()` **每条射线**都新建
一个 `ODECollision` + 一个 `ODERayShape`（再各自落到 ODE 的 `dRay` geom / space 上），
而 `livox_points_plugin.cpp` 里是 `rayShape->RayShapes().reserve(samplesStep / downSample)` =
**reserve(30000)** 后真的 `AddRay` 30000 次。`<visualize>false</visualize>` 已经关掉了
（所以**不是**射线可视化），代价纯粹是"30000 条 ODE 射线"的结构体开销。

> **给主线的含义**：如果哪天需要把这 3.2 GB 压下来，**唯一有效的旋钮是
> `<plugin><samples>`**（以及 `<downsample>`）—— 把 30000 降到 10000 预计能省 ~1.8 GiB，
> 但会改变点云密度 ⇒ 属于感知契约变更，**必须**另开一个带量化验收的任务，
> 本文不动它（见 §9）。

---

## 7. 怎么复现（不依赖本文的脚本）

```bash
# 0) 一个虚拟屏（本沙箱没有 Xvfb 包，用 apt-get download 解到本仓；有 X 的机器跳过）
#    Xvfb :79 -screen 0 1600x1000x24 -nolisten tcp &
#    export DISPLAY=:79

# 1) 隔离
export HOME=$PWD/.tmp_gzgui/rep/gzhome ROS_LOG_DIR=$PWD/.tmp_gzgui/rep/roslog
export ROS_DOMAIN_ID=177 GAZEBO_MASTER_URI=http://127.0.0.1:12500
mkdir -p "$HOME" "$ROS_LOG_DIR"

# 2) 精确复现"代理在但不干活"
python3 - <<'PY' &
import socket; s=socket.socket(); s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.bind(('127.0.0.1',13500)); s.listen(64)
while True: s.accept()      # accept 但永不回包
PY
export http_proxy=http://127.0.0.1:13500 https_proxy=$http_proxy
export no_proxy=localhost,127.0.0.1

# 3) 起世界 + spawn robot11
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch gazebo_ros gzserver.launch.py world:=$PWD/install/hzmi_rm_simulation/share/hzmi_rm_simulation/world/RMUC2026_world/RMUC2026_world.world &
sleep 10
xacro src/bringup/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro robot11_visual:=decimated > .tmp_gzgui/rep/robot.urdf
ros2 run gazebo_ros spawn_entity.py -entity robot -file .tmp_gzgui/rep/robot.urdf -x 0 -y 0 -z 0.25 -timeout 90

# 4) 起 GUI，看它自己的日志
ros2 launch gazebo_ros gzclient.launch.py &
sleep 60; tail -5 ~/.gazebo/client-*/default.log
# 预期：Waiting for model database update to complete... 然后长时间不动
#       最后 No mesh specified

# 5) 加解析根再试一次 ⇒ 预期不再出现上面两行
kill %3 %2 %1 2>/dev/null
GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/install/robot11/share" \
  ros2 launch gazebo_ros gzclient.launch.py &
sleep 30; grep -c "No mesh specified\|Waiting for model database" ~/.gazebo/client-*/default.log   # 期望 0
```

---

## 8. 用户侧还需要自己在真机（真 GPU / 真显示器）上确认的

本沙箱**没有 GPU、没有真显示器**，以下都**没有实测**：

1. **真 GPU 路径**：所有渲染数字都来自 llvmpipe 软件渲染。真机上 OGRE 走 NVIDIA GL，
   首帧时间、RSS、CPU 都会不一样（软件渲染下 gzclient 峰值 CPU 到过 581%）。
2. **窗口管理器/合成器**：本沙箱无 WM（截图里 `gzclient` 的窗口是 `+0+0`）。
   真机上如果 Gazebo 窗口被摆到屏幕外、或被合成器黑屏，需要自己确认。
3. **`/etc/hosts` 或代理对 `models.gazebosim.org` 的实际行为**：请实测
   ```bash
   time curl -sS -o /dev/null -w '%{http_code} %{time_total}\n' http://models.gazebosim.org//database.config
   ```
   * 秒回 ⇒ 不是本文 §3 的路径；
   * 卡住/超时 ⇒ 就是它，按 §5.2 处理。
4. **swap 占用**：`SwapFree` 只剩 1 MiB。请自己看 `free -m` 与
   `systemd-cgtop`/`ps` 找占用大户（本文没有权限去动它）。
5. **`exit code -9` 的发送者**：需要 `auditd` 或 `bpftrace` 才能抓现行。建议装/开
   `auditd` 后复现一次：
   ```bash
   sudo auditctl -a always,exit -F arch=b64 -S kill -F a1=9 -k whokills
   sudo ausearch -k whokills -i | tail -20
   ```

---

## 9. 未验证 / 未做清单（诚实边界）

* **没有** 在真 GPU / 真显示器上跑过（本沙箱无 GPU）。
* **没有** 指认 16:55:55 / 16:56:32 两次 SIGKILL 的发送者（无 `auditd`）——
  §4.2 是排除法结论。
* **没有** 修改 §4.2 列出的那些 bench 脚本（`tools/scripts/mapping/**`、
  `tools/scripts/diag/**` 等属于别的任务范围）；只给出改法。
* **没有** 验证 (b) 方案（`package.xml` 里加 `gazebo_model_path="${prefix}/.."`）在
  `colcon build` 后的实际效果 —— 沙箱里验证的是 (a) 的等价做法（直接给 `GAZEBO_MODEL_PATH`）。
  §6.3 的 `fixed` 配置走的是 (a)。
* **没有** 处理"加了解析根之后 `InsertModelWidget` 多 3 条 `Missing model.config` 噪音"：
  修法是给 `install/robot11/share/robot11/` 补一个 `model.config`，但 `ament_index` /
  `colcon-core` 两个子目录的告警来自 ament 的 share 布局本身，**消不掉**（只是噪音，不影响加载）。
* **没有** 动 `<samples>30000</samples>`（§6.4.1 指出它是 3.2 GB 的唯一旋钮）：
  改它会改变点云密度 ⇒ 属感知契约变更，需要单独的量化验收任务（本仓 RULES 也禁止在本任务里改）。
* **没有** 在真机上验证 `gazebo_offline:=True`（只在沙箱的黑洞代理 + `baduri` 配置下验证）。
* 交换分区已满（`SwapFree` 只剩 1 MiB）**没有**做任何清理动作（用户数据只读）。
* ⚠️ **数据完整性披露**：`~/.gazebo/client-11345/default.log` 是**默认端口 + 默认 HOME**
  才会写的文件；本文 §2.2 引用的那 6 行是在本任务**开始时**（19:5x）读到的原文。
  之后 **20:13** 有一次同机并发的 `bringup_sim … robot:=robot11` 运行（**不是本任务的探针** ——
  本任务所有探针都用自己的 `HOME` 与专用端口）把它覆盖了。若要复核 §2.2 的原文，
  请用 20:13 那次同口径的日志（§6.2 第 2 行，stall 76.34 s），或在用户真机上重跑一次复现。
* ⚠️ **同机并发披露**：本任务运行期间（20:06 / 20:13 / 20:17…）机器上还有别的任务在跑
  `bringup_sim.launch.py`。为了拿到干净的内存数字，本任务做过按名字的
  `pkill -9 -x gzserver/gzclient`，**有可能**误伤并发运行（这正是 §4.2 描述的同一个坑）。
  另外 §6.4 的记账第一次也因为并发/残留进程拿到过 123 MiB 的**假**数字，
  已在干净隔离下重测（表里是重测值）。

---

## 10. 相关文档

* `docs/robot_models.md`（robot11 槽位、`robot11_visual` 档位）
* `docs/worlds.md`（RMUC2026 世界资产）
* `docs/issues_and_findings.md`


---


## 来源 12/19：`research/lio_drift_diagnosis.md`


> **面向的问题（用户原话）**：「我这个才多久阿，里程计就飘了，这合理吗」
> 背景：本仓已有测量说这个槽位很好（`small_point_lio` tuned：`docs/mapping_small_point_lio.md` §5
> 的 ATE **0.045 m**、位移/真值 **0.996**）；但用户当天是在**续建 + 一份已被证明污染的存档**下跑的
> （存档的 3D 点云 z 到 **+23.10 m**、y 到 **+19.30 m**，正常场地 z∈[-0.53,+1.43] —— 见 §6）。
> 所以本次不做"再测一次 LIO 好不好"，而是**用数字把四个候选分开**：
> **(a) 正常累积 / (b) 卡死·几何退化触发 / (c) 转弯诱发 / (d) 续建污染存档的假象**。

## 0. 一句话结论

**先给答案**：
* 如果你当时是在**正常开**（直线、拐弯、窄道），那**"才跑一小会儿就飘"不合理** ——
  本任务里这条路线上 LIO 每米只漂 **1 cm**，5 次跑没有一次在正常工况下飘过。
* 如果那一小会儿里出现了**高角速度原地旋转**，那**"很快就飘"完全合理** ——
  本仓仿真底盘要真转到 **1.9 rad/s** 得给 **6 rad/s** 的指令；一旦进这个工况，
  LIO 会**抛硬币**（6 次同样的甩转，**坏 3 次**），坏起来是**0.6 米到 10 米 / 几十秒**的事，
  而且 `map→base_link` 会跟着一起飘（slam **救不回来**）。
* **不要把"续建存档被污染"当成实时位姿飘的原因**：实测续建后 `map→odom` **恒等**，
  前面 400 秒每一段都和"从零建图"一样干净。那份存档坏的是 **3D 点云资产**，不是实时位姿。

| # | 候选 | 判定 | 决定性数字 |
|---|---|---|---|
| **a** | 正常累积 | ❌ **解释不了"才一会儿就飘"** | 直线 **0.0105 m/m**、原路返回 **0.0104 m/m**、窄沟 **0.0031 m/m**；整程 ATE RMSE **0.036 m** / max **0.096 m**（51.4 m、408 仿真秒） |
| **b** | 卡死 / 几何退化触发 | ❌ **不成立** | 窄沟段 `/scan` 有限束数最小 **115**（其余 ~850）、车速只有指令的 **62%**、两段都撞到墙钟上限；但该段 ATE RMSE **0.024 m**、**0.0031 m/m**。逐秒误差增量 vs 束数 **r=0.065**、vs RTF **r=−0.026**（≈0） |
| **c** | 转弯诱发 | ✅ **唯一复现出来的失效模式（与「地图冷热」无关）** | 温和转 **无害**：180°@**0.32 rad/s** 误差增量 **−0.003 m**、慢转@**0.162 rad/s** **−0.0004 m**、快转@**1.104 rad/s**（22.5 rad，1.5 s 换向）**−0.008 m**（两次独立跑都是）。**但进到高角速度工况就是抛硬币**：同一档甩转（|cmd ω| 3.8~6、实际 0.8~1.9 rad/s）**6 次里坏 3 次**，坏起来从 LIO max **0.62 m** 到位置跑飞 **10.68 m**（`map→odom` 甩到 13.34 m）；**最冷的一次（启动后 15 s）反而最干净 0.042 m，最暖的一次（走过 34 m）坏了** ⇒ 不是「地图没长好」 |
| **d** | 续建污染存档的假象 | ❌ **不成立（但有残留疑点）** | 续建的 `map→odom` **全程恒等**（0.08, 0.02, −0.014）直到 426 s；S 段之前每一段 ATE RMSE **0.014~0.053 m**、每米 **0.004~0.015 m**，与从零建图**没有差别**。污染是**真的**（PCD z 到 +23.10 m），但它**不在实时位姿链路上** |

---

## 1. 复现环境与隔离（所有数字都在这个隔离下跑出来）

```bash
# ⚠️ 本仓每条 bash 命令跑在自己的 PID namespace（bwrap --unshare-pid --die-with-parent）
#    ⇒ 启动 / 驱动 / 收尾必须塞进**同一次** bash 调用（一条命令跑完）。
HOME=/tmp/gzhome-diag<tag>  ROS_DOMAIN_ID=<非默认>  GAZEBO_MASTER_URI=http://127.0.0.1:<port>
unset DISPLAY  nav_rviz:=False  lio_rviz:=False  spin_speed:=0.0
map_name:=RMUC2026_diag            # 测试专用名，不碰用户的任何存档
```

| 项 | 值 |
|---|---|
| launch | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping lio:=small_point_lio mapper:=slam_toolbox nav_rviz:=False lio_rviz:=False spin_speed:=0.0 map_name:=<测试名>` |
| 出生点核对 | ✅ 实测 (10.9247, 2.5189, yaw −0.11°) vs 期望 (10.925, 2.525, 0) ⇒ Δxy **0.006 m** |
| 从零那一跑 | 仿真 **408.2 s** / 墙钟 **537.2 s**（RTF≈**0.724**）；`/map` known **104.2 m²** |
| 续建那一跑 | 仿真 **409.0 s** / 墙钟 **533.8 s**（RTF≈**0.761**）；`/map` known **190.7 m²**（加载进来的图本来就大） |
| 话题速率（两跑一致） | `/odom` / `/odom_ground_truth` / `/scan` 各 ~10.3 Hz；`/tf` **~115 Hz**（47008 条）；`/map` 111 |
| **消息间隔（两跑一致）** | `/odom` `/scan` 的间隔**中位=p99=max=0.1000 s**，**超过 0.2 s 的次数 = 0**（全程，含甩转段） |
| slam_toolbox 参数（未改） | `transform_publish_period 0.02`、`minimum_time_interval 0.5`、`minimum_travel_distance 0.5`、`do_loop_closing true`、`scan_queue_size` 默认 1 |

⚠️ **两条跑法护栏（都踩过，写下来）**：
1. **上一轮的 `gzserver` 没死干净**时，新 launch 的 gzserver 会 `process has died (exit code 255)`，
   `spawn_entity` 报 `Entity [robot] already exists` 并退出 ⇒ **你量到的是上一轮那台车**
   （它停在上一轮路线中断的地方）。本次实测抓到过一次：本轮首帧真值 (11.14, 4.71, yaw=3.05)
   而不是 (10.925, 2.525, 0) —— **整跑作废**。⇒ `segment_drive.py --expect-spawn` 与
   `run_drift_diag.sh` 里的日志 grep 都是为它加的硬护栏，对不上就当场中止。
2. **不要同时跑两个仿真**（同一个 `GAZEBO_MASTER_URI`）：第二个 gzserver 抢不到 master 时会
   "寄生"到第一个的机器人上（现象与第 1 条一模一样）。

---

## 2. 路线：为什么切成这几段

### 2.1 几何是从先验图里量出来的（不是手写坐标猜的）

用 `map/RMUC2026.pgm`（出生点相对系、0.05 m）算 `clearance ≥ 0.35 m` 的**出生点连通可行驶域**：

| 地形 | map 系范围 | 说明 |
|---|---|---|
| 大厅（宽区） | `y∈[-3.0,+2.25]` 时 `x` 从西边界(-5.9~-6.6)通到 **+3.3** | 一条 **8.5 m** 的东西直线（两侧余量 ≥0.35 m，中间余量 ~2.4 m） |
| 斜向窄沟 | `y` 从 -3.75 到 -6.8，中心线 `x` 从 -6.0 斜到 -8.1 | 两侧 0.25~0.45 m **竖直沿**、**平行墙**；沟底就是 23° 坡坡脚 |

### 2.2 分段表（`segment_drive.py default_spec()`）

| 段 | 组 | 内容 | 为什么要有它 |
|---|---|---|---|
| `prep1` | prep | 出生点 → (-5.5, 1.5) | 就位，不算进主表 |
| `Tp/T0/T1/T2/Tg` | align | 原地对齐到指定朝向（实测 ~0.45 rad/s） | **换向必须显式对齐**：不然 `goto` 会用 `v=0.075 m/s + ω=0.9` 画一个 ~165° 的急弧，那 3.5 s 就落进下一段的窗口里 |
| **`A`** | straight | (-5.5,1.5) → (3.0,1.5) 向东 **8.5 m** @0.30 m/s | **(a) 直线** |
| `Ta`/`Td` | turn180 | 180° 原地转（实测 **0.32 rad/s**） | 换向，**同时是"温和原地转"的测量对象** |
| **`D`** | return | 沿 `A` 的同一条线原路返回 8.5 m | **(d) 回环机会**（与 `A` 同一条地面） |
| `F`/`G` | straight | 6.0 m 直线（慢转前 / 慢转后） | 慢转的"前后对照" |
| **`B`** | slow_turn | 期望 ±0.35 rad/s、每 5 仿真秒换向、累计 5.25 rad | **(b) 慢转** |
| **`C`** | fast_turn | 期望 ±1.5 rad/s、每 1.5 仿真秒换向、累计 22.5 rad | **(c) 快转**（对应 `docs/mapping_small_point_lio.md` §5.1 `run1` 的已知坏工况） |
| **`H`** | straight | 快转之后 4.5 m 直线 | 快转**之后**还回不回得来 |
| **`E`** | narrow | 斜向窄沟下行到 23° 坡坡脚（6 个航点） | **(e) 平行墙 / 退化几何** |
| **`E2`** | ramp_probe | 沟底 → 走廊口试探（带卡死自动退出） | **(e2) 23° 坡 + 1.05 m 走廊**的接近段 |
| **`S`** | sweep | 角速度指令 → 实际 传递曲线（0.3/0.6/1.2/2.4/4.8 rad/s × 5 仿真秒） | 量"我让它转多快"到底等于"它转多快"；**这一段是本次唯一抓到 LIO 崩掉的工况** |

> ⚠️ **段序本身是实验设计的一部分（第一版排错了，整跑作废重跑）**：
> `C`（快转）与 `S`（甩转）会**不可逆地带坏 LIO**。第一版把 `S` 放在最前面：
> 甩转结束时 LIO 偏航差 **0.81 rad = 46°**、位置差 0.44 m，而车**一步没平移**；
> 此后位置误差按 `2·sin(46°/2)=0.78 m/m` 增长，`prep1` 一段就长 3.85 m，整程 ATE max 8.16 m。
> ⇒ **破坏性段必须放最后**，否则前面所有"干净段"的结论全部作废。本报告主表来自**重排后**那一跑。

### 2.3 ⚠️ 指令角速度 ≠ 实际角速度（本仓底盘的真实传递曲线）

> **★ 2026-10-07 更正（`docs/lio_divergence_no_impact.md` §2 第 1 条）**：下面这张表**与本仓当前栈不符**。
> 用同一行内的"指令/真值"配对重测（`analyze_noimpact_divergence.py --yaw-transfer`，n=75~109、
> 符号一致率 1.00）得到的是 **≈1:1**：指令 0.34→实测 0.36、0.65→0.67、1.07→1.11、**2.83→2.83**（原地）。
> 也就是说底盘**听得懂** yaw 指令、没有那种巨大死区；"要 1.9 rad/s 得给满 6"不要再引用。
> （下表大概率混进了"闭环里 cmd 与被控量不同口径"的问题，未追溯。）

`S` 段实测（闭环 `cmd = target + 6·(target − measured)`，从零那一跑）：

| 期望实际 ω (rad/s) | **实测 ω (rad/s)** | 为此需要的**指令** \|ω\| (rad/s) | 同向比例 |
|---|---|---|---|
| 0.30 | **0.117** | 1.40 | 0.39 |
| 0.60 | **0.317** | 2.29 | 0.53 |
| 1.20 | **0.843** | 3.66 | 0.70 |
| 2.40 | **1.702** | 5.90 | 0.71 |
| 4.80 | **1.890**（饱和） | 6.00（被上限截断） | 0.39 |

**结论**：本仓仿真底盘对 yaw 的"权威"远低于指令 ——
**`cmd ω ≈ 0.6 rad/s`（一个很"正常"的开环指令）实际只转 ~0.02 rad/s**
（第一版 `Tp` 段实测：0.83 rad / 46.4 仿真秒，指令均值 0.776）；要 0.3 rad/s 得给 ~1.4，
要 1.9 rad/s 得给满 6。
⇒ **本报告里所有"转了多快"都是实测值**；同仓文档里凡是引用手写 `--spin-rate` 的地方都要打折看。

---

## 3. 仪器（`tools/scripts/diag/`）

| 文件 | 干什么 |
|---|---|
| `segment_drive.py` | 分段脚本化路线 + **50 Hz 同步时间轴**（CSV）+ 事件表（JSON）；GT 闭环（`--pose-source gt`，几何确定可比） |
| `lio_drift_analyze.py` | 分段 ATE / 每米漂移 / 单步跳变 / 相关性 / **每段传感器体检** / 三张 PNG |
| `run_drift_diag.sh` | 一条 bash 调用跑完：起栈（可选续建）→ 驱动 → 收栈 → 分析；含上面两条护栏 |
| `spec_cold_warm.json` | 冷/暖地图 A/B 计划：同一段甩转在"最冷/半冷/暖"三处各做一次（**已跑**，见 §5.1；结论：与地图冷热无关） |

### 3.1 50 Hz 时间轴里有什么（每行一条记录）

`wall_t / sim_t / rtf`；`/odom`（含 twist）与 `odom→base_link` TF 的 `x y z yaw` **+ 它们各自"有多旧"**；
`/odom_ground_truth`；`map→odom`、`map→base_link`（= `map→odom` ∘ `odom→base_link` **手动复合**）；
指令 Twist；`/scan` 的**有限束数**、**前向 ±20° 最近回波**、全向最近回波；`/map` known 面积；各话题计数。

### 3.2 ⚠️ 采样饥饿陷阱（本仓踩过，本工具从根上避开）

`docs/worlds.md` §5.1 记录过：用单线程 `rclpy.spin_once()` 每 0.2 s 采一次，
在 **~80 Hz 的 `/tf` 洪水**下会被饿死、拿到十几秒前的旧值 ⇒ 会得出
**"LIO 滞后 13 s / 估计位移是真值的 2.6 倍"这种假结论**（本仓实测踩过）。本工具的做法：
① `MultiThreadedExecutor(num_threads=4)` + `ReentrantCallbackGroup`；
② 回调里**只做 O(1) 赋值**（不做 TF lookup、不解析点云、不写文件）；
③ 落盘由独立的 50 Hz 定时器做；
④ **每条记录都带 `*_age_sim` / `*_age_wall`**（这条数据比"现在"旧多少）——
真被饿死了，CSV 里 age 会涨，分析脚本把超阈值的行标出来，而不是拿旧值当新值下结论。

**本跑采样健康度（两跑一致）**：`/odom` age 均值 **0.050 s** / p99 0.10 s / max 0.10 s；
`/scan` 0.046/0.10/0.10；`map→odom` 均值 −0.14 s（slam 用未来戳重发，正常）；
**`age > 0.5 s` 的行数 = 0**。

### 3.3 对齐口径（所有误差数字的地基）

`/odom_ground_truth` 在 **world 系**；`/odom`、`odom→base_link`、`map→base_link` 各自差一个**常量刚体变换**。
本工具用**起步前静止窗口**（`link_ready` → `warmup_end`，各跑 39~40 对样本）估这**一个**变换，然后**全程沿用**
（本次 `yaw_off = 0.00°`、`t = (10.929, 2.526)`）。
**绝不能"每段重新对齐"** —— 那会把漂移当坐标系差减掉，是自欺欺人。
偏航也一起对齐（`est_yaw = src_yaw + yaw_off`）；真值按**线性插值**取到采样时刻
（真值只有 10 Hz，快转时"最近邻取一条"的偏航差可达 ±4.3°，会被当成 LIO 误差）。

### 3.4 单步跳变的判据（第一版误报 149 次，已修）

只看"单步 >10°"会把**正常快转**判成跳变（实测 1.87 rad/s × 0.1 s = 10.7°）。
现在要求**同时**满足"位移 >0.2 m **且** >1.5 m/s"或"偏航 >10° **且** >3.5 rad/s"
（速率门取在底盘实测能力之上）。修完后：从零那一跑 **0 次**跳变；
续建那一跑 39 次——**都是真的**（见 §6.3）。

---

## 4. 结果 A：从零建图（`lio:=small_point_lio`，`map_name:=RMUC2026_diag`）

### 4.1 整程

| 序列 | 配对数 | ATE RMSE | ATE mean | ATE max | 结束误差 | 偏航 RMSE | 轨迹长 |
|---|---|---|---|---|---|---|---|
| **LIO `/odom`** | 4198 | **0.0360 m** | 0.0326 m | **0.0957 m** @432.6 s | 0.0641 m | **0.624°** | 51.44 m |
| `odom→base_link` TF | 4198 | **0.0360 m** | 0.0326 m | 0.0957 m | 0.0641 m | 0.624° | 51.44 m |
| **融合 `map→base_link`** | 4248 | **0.0529 m** | 0.0389 m | **0.1586 m** @432.6 s | 0.1330 m | 0.756° | 52.38 m |

* `/odom` 与 `odom→base_link` **逐位相同** ⇒ 装配一致（`small_point_lio` 自己发 `odom→base_link`，
  没有第二个发布者抢边）。
* **单步跳变：三条序列各 0 次。**
* 逐秒误差增量与诊断量的相关性（只看在动的窗口，n=283）：`/scan` 束数 **r=0.065**、
  前向最近回波 **r=−0.048**、`|cmd ω|` **r=0.019**、RTF **r=−0.026** ⇒ **全部 ≈0，没有哪一类原因主导**。

### 4.2 每段（LIO `/odom`，对齐后 vs 真值）

| 段 | 组 | T(s) | 真值里程 | RMSE | max | **误差增量** | 段内相对位移误差 | **每米漂移** |
|---|---|---|---|---|---|---|---|---|
| prep1 | prep | 37.9 | 6.00 m | 0.0303 | 0.0499 | +0.0276 | 0.0296 | 0.0049 |
| Tp | align | 6.3 | 0.01 | 0.0309 | 0.0356 | +0.0008 | 0.0060 | — |
| **A** | **straight** | 38.4 | **8.15 m** | 0.0409 | 0.0590 | +0.0189 | 0.0856 | **0.0105** |
| Ta | turn180 | 9.9 | 0.01 | 0.0583 | 0.0620 | **−0.0028** | 0.0026 | — |
| **D** | **return** | 40.8 | **8.28 m** | 0.0396 | 0.0589 | −0.0258 | 0.0864 | **0.0104** |
| Td | turn180 | 9.8 | 0.01 | 0.0335 | 0.0409 | −0.0074 | 0.0050 | — |
| F | straight | 31.2 | 5.83 m | 0.0354 | 0.0545 | −0.0062 | 0.0413 | 0.0071 |
| **B** | **slow_turn** | 32.5 | 0.02 | 0.0271 | 0.0314 | **−0.0004** | 0.0025 | — |
| Tg | align | 6.1 | 0.01 | 0.0239 | 0.0278 | −0.0024 | 0.0030 | — |
| G | straight | 25.2 | 5.66 m | 0.0263 | 0.0390 | +0.0085 | 0.0379 | 0.0067 |
| T0 | align | 6.3 | 0.01 | 0.0290 | 0.0362 | +0.0029 | 0.0053 | — |
| **C** | **fast_turn** | 20.5 | 0.06 | 0.0283 | 0.0482 | **−0.0080** | 0.0110 | — |
| T1 | align | 3.3 | 0.00 | 0.0224 | 0.0281 | +0.0073 | 0.0084 | — |
| H | straight | 17.8 | 4.03 m | 0.0352 | 0.0538 | +0.0113 | 0.0234 | 0.0058 |
| T2 | align | 5.8 | 0.01 | 0.0258 | 0.0301 | +0.0041 | 0.0027 | — |
| **E** | **narrow** | 46.3 | 6.30 m | **0.0242** | 0.0361 | −0.0128 | 0.0192 | **0.0031** |
| **E2** | **ramp_probe** | 45.0 | 4.27 m | 0.0405 | 0.0632 | +0.0473 | 0.0643 | 0.0150 |
| **S** | **sweep** | 25.1 | 0.06 | 0.0580 | **0.0957** | +0.0050 | 0.0147 | — |

**怎么读**：
* **直线与原路返回（A / D）每米只漂 1.05 cm / 1.04 cm**，两者几乎一模一样 ⇒
  **原路返回既没带来额外收益也没带来额外损失**（LIO 本来就是不回环的里程计，回环是 slam_toolbox 的事）。
* **两段 180° 原地转（Ta/Td，实测 0.32 rad/s）误差增量为负**（−0.003 / −0.007 m）⇒
  温和原地转**不产生**任何可测漂移。
* **慢转（B，实测 0.162 rad/s、5.25 rad）误差增量 −0.0004 m**。
* **快转（C，实测 1.104 rad/s、22.5 rad、每 1.5 仿真秒换向）误差增量 −0.008 m、RMSE 0.028 m**
  ⇒ **"频繁大角速度原地转"在 1.1 rad/s 这一档没有把 LIO 带坏**（与 `run1` 的 ATE max 1.61 m 相反）。
* **窄沟（E）0.0031 m/m —— 全表最好的一段之一**。代价在 §4.4。
* `S`（甩转，最高实测 1.89 rad/s）是全程最差的一段，但绝对量仍然只有 **0.096 m**。

### 4.3 每段（融合 `map→base_link`）

| 段 | 组 | T(s) | 真值里程 | RMSE | max | 误差增量 | 每米漂移 |
|---|---|---|---|---|---|---|---|
| A | straight | 38.4 | 8.15 m | 0.0294 | 0.0526 | +0.0198 | 0.0067 |
| D | return | 40.8 | 8.28 m | 0.0277 | 0.0526 | −0.0417 | 0.0088 |
| Ta / Td | turn180 | — | 0.01 | 0.052 / 0.019 | 0.056 / 0.024 | −0.003 / −0.005 | — |
| B | slow_turn | 32.5 | 0.02 | 0.0294 | 0.0339 | −0.0004 | — |
| C | fast_turn | 20.5 | 0.06 | 0.0208 | 0.0441 | +0.0017 | — |
| H | straight | 17.8 | 4.03 m | 0.0267 | 0.0504 | −0.0108 | 0.0079 |
| E | narrow | 46.3 | 6.30 m | 0.0325 | 0.0646 | +0.0439 | 0.0101 |
| **E2** | ramp_probe | 45.0 | 4.27 m | **0.0960** | **0.1338** | +0.0641 | 0.0247 |
| S | sweep | 25.1 | 0.06 | 0.1343 | 0.1586 | −0.0032 | — |

**融合位姿是"有界"的**（slam 在纠正，没有跟着 LIO 一起飘）：

| 量 | 前 1/3 | 后 1/3 |
|---|---|---|
| LIO 误差均值 | 0.0337 m | 0.0361 m |
| **融合误差均值** | **0.0244 m** | **0.0700 m** |
| max(融合) / max(LIO) | — | **1.657** |

* 融合误差**始终同一量级**（2.4 cm → 7.0 cm），**没有随里程发散**。
* 融合**最大误差出现在 `E2`（走廊口试探）**：RMSE 0.096 m、max 0.134 m。
* 全程融合误差**比 LIO 大 1.66~1.94 倍** —— 因为 `map→odom` 是**分段常数**
  （`minimum_time_interval 0.5 s` + `minimum_travel_distance 0.447 m` 两道门 ⇒ 实测只有 ~3% 的
  `/scan` 真的被拿去匹配，见 `docs/slam_toolbox_scan_drops.md` §7），它有额外的量化/延迟误差。
  **它不会让误差变小，只是不让误差无限长。**

### 4.4 每段的"当时体检"（传感器 / 执行器 / 负载）—— 归因的关键

| 段 | 组 | 真值里程 | `/scan` 有限束数 均值 / **最小** | 前向最近回波 p10 / p50 | 前向 <1.5 m 占比 | **实测车速**（指令） | \|cmd ω\| 均值 | RTF |
|---|---|---|---|---|---|---|---|---|
| A | straight | 8.15 m | 783 / 372 | 2.38 / 4.82 | 0.00 | 0.212 (0.30) | 0.19 | 0.78 |
| D | return | 8.28 m | 823 / 440 | 1.57 / 1.94 | 0.03 | 0.203 (0.30) | 0.24 | 0.77 |
| **Ta** | turn180 | 0.01 | 428 / 340 | 1.18 / 5.54 | 0.22 | — | 2.20 | 0.78 |
| **B** | slow_turn | 0.02 | 758 / 686 | 1.61 / 2.59 | 0.05 | — | 1.58 | 0.76 |
| **C** | fast_turn | 0.06 | 888 / 789 | 1.51 / 1.57 | 0.00 | — | **4.28** | 0.77 |
| H | straight | 4.03 m | 899 / 815 | 1.90 / 2.58 | 0.00 | 0.226 (0.30) | 0.11 | 0.75 |
| **E** | narrow | **6.30 m** | **655 / 115** | 1.62 / 3.65 | 0.06 | **0.136 (0.22)** | 0.17 | 0.75 |
| **E2** | ramp_probe | **4.27 m** | 850 / 615 | 2.83 / 3.88 | 0.00 | **0.095 (0.16)** | 0.07 | 0.74 |
| **S** | sweep | 0.06 | 814 / 695 | **0.99** / 1.44 | **0.52** | — | 3.84 | 0.74 |

**这张表把三个候选的"有没有发生"钉死了**：
* **`/scan` 束数塌陷确实发生了**：窄沟段最小 **115** 束（其余段 650~900）；
* **"发着速度但走不动"也发生了**：`E` 段实际车速只有指令的 **62%**、`E2` 只有 **59%**，
  而且 `E`/`E2` **都撞到单腿 60 s 墙钟上限才收**（没走到名义终点）；
* **但 LIO 在这两段仍然干净**（`E` 3.1 mm/m、`E2` 15 mm/m，绝对量都 <7 cm）
  ⇒ **"卡死 + 退化"在本跑里没有把 LIO 带坏，它只是让车走得很慢**。
* **RTF 全程 0.72~0.82，与误差增量相关性 r=−0.026** ⇒ **不是算力问题**。

### 4.5 图

| 图 | 文件 | 看什么 |
|---|---|---|
| 误差 vs 时间 / vs 里程（底纹=分段、红点线=跳变、黑虚线=卡死） | `docs/img/spl_fresh_error_vs_time_distance.png` | 融合（红）全程 <0.16 m，只在 `E2`/`S` 抬起；LIO（蓝）全程 <0.096 m |
| 诊断量（束数 / 前向最近回波 / 指令与 LIO 报告 ω / RTF） | `docs/img/spl_fresh_diagnostics.png` | `E` 段束数塌到 ~115、`E2` 前向回波掉到 ~0.5 m；**LIO 报告的 ω 恒为 0** |
| XY 轨迹（对齐后） | `docs/img/spl_fresh_traj_xy.png` | 三条轨迹是否贴合 |
| **续建那一跑的同三张图** | `docs/img/spl_resume_*.png` | §6.3：`S` 段 LIO 跑飞、`map→odom` 甩到 13.3 m |
| **冷/暖那一跑的同三张图** | `docs/img/spl_coldwarm_*.png` | §5.1：`S0`（最冷）干净、`S2`（最暖）坏 —— 一眼看出「与地图冷热无关」 |

> ⚠️ **顺带发现**：`small_point_lio` 发的 `/odom` **twist 字段恒为 0**（`lio_wz` 全 0，见诊断图第 3 面板）
> ⇒ **不能**用 `/odom` 的 twist 做速度/角速度诊断，只能用位姿差分。同仓文档若引用过 `/odom` twist 需要复核。

---

## 5. 结果 B：唯一复现出来的失效模式 —— 高角速度甩转（抛硬币）

### 5.1 六次同样的甩转

同一档甩转（闭环 `cmd = target + 6·(target − measured)`，指令被顶到 **3.8~6 rad/s**、
**实际 0.8~1.9 rad/s**、方向每隔几秒反转）。**6 次里坏了 3 次**：

| # | 跑 | 时机 / 地点 | `/scan` 束数均值 / 前向 p10 / <1.5 m 占比 | **LIO 结果** | 融合结果 |
|---|---|---|---|---|---|
| 1 | 第一版（`S` 在最前） | 启动后 ~35 s / 出生点 | 开阔（前向 2.5~4.8 m） | ❌ 偏航一次差 **0.81 rad = 46°**（随后整程 0.78 m/m 增长，ATE max 8.16 m） | — |
| 2 | 从零（`S` 在最后） | ~417 s / 沟口 | 814 / 0.99 / 0.52 | ✅ max **0.096 m** | ✅ `map→odom` 全程恒等 |
| 3 | 续建（`S` 在最后） | ~417 s / 沟口 | 759 / 0.99 / 0.49 | ❌ **39 次跳变**、位置跑飞、max **10.68 m** | ❌ `map→odom` 甩到 **13.34 m** |
| 4 | 冷/暖 `S0` | **最冷**：启动后 ~15 s / 出生点 | **773 / 2.72 / 0.00**（最开阔） | ✅ max **0.042 m**（最干净的一次） | ✅ |
| 5 | 冷/暖 `S1` | 半冷：走过 6 m / 西端 | 893 / 1.51 / 0.02 | ✅ max **0.046 m** | ✅ |
| 6 | 冷/暖 `S2` | **暖**：走过 34 m / 西端 | 889 / 1.35 / **0.42** | ❌ max **0.624 m**（段内位置估到 1.81 m） | ⚠️ 0.257 m（**slam 反而补回一部分**） |

**怎么读这张表**：
* **不是「地图没长好」**：**最冷**的 `S0`（LIO 启动后 15 秒、地图几乎是空的）表现得**最好**（0.042 m）；
  **最暖**的 `S2`（已走 34 m、`/map` 已长起来）反而坏了。⇒「冷地图猛转会把 LIO 带坏」
  这个假设**被这一跑否掉了**。
* **不是「环境太窄」**：`S0` 前向最近回波 p10 = **2.72 m**（最开阔）却最安全；
  `S2` p10 = 1.35 m、42% 帧 <1.5 m —— 窄是「帮凶」但**不是必要条件**（第 1 行那次在开阔区也崩了）。
* **不是丢帧 / 算力**：见 §5.3 的逐项对比（消息间隔中位=p99=max=0.1000 s、RTF 反而更高）。
* ⇒ **这一档就是抛硬币：3/6 坏**。坏起来的幅度差别很大（0.62 m ~ 10.7 m），
  说明它不是一个「稳定发散」的动力学，而是**在某些帧上扫描匹配突然失锁**。

### 5.2 坏起来有两种形态，后果完全不同

| 形态 | 例子 | LIO | 融合 `map→base_link` | slam 能救吗 |
|---|---|---|---|---|
| **慢漂** | 冷/暖 `S2` | max **0.624 m**，逐帧小增量，**单步跳变 0 次** | max **0.257 m**（**比 LIO 还小**，比值 0.408） | ✅ **能补一部分**：`map→odom` 跟着反向修正 |
| **失锁跑飞** | 续建 `S` | max **10.68 m**，**39 次单步跳变**（Δpos 最大 0.30 m/0.1 s = **3 m/s**） | max **9.28 m**（比值 **0.869**，**1:1 同步**） | ❌ **救不回来** |

⇒ **实用判据**：如果 `map→base_link` 的误差**比 `/odom` 还小**，说明 slam 还在纠（慢漂形态，可恢复）；
如果两者**误差几乎相等**，说明 slam 已经放弃（失锁形态，只能重启/重建）。
（本仓现成工具 `tools/scripts/diag/map_odom_jump_gate.py` 可以实时盯 `map→odom` 的单步增量。）

### 5.3 「两跑输入几乎一样、结果差 100 倍」的逐项对比

| 对比项 | 从零（活，max 0.096 m） | 续建（崩，max 10.68 m） |
|---|---|---|
| 段内真值里程 | 0.056 m | 0.055 m |
| `/scan` 有限束数 均值 / 最小 | 814 / 695 | 759 / 656 |
| 前向最近回波 p10 / <1.5 m 占比 | 0.99 / 0.52 | 0.99 / 0.49 |
| **`/odom` 与 `/scan` 的消息间隔** | 中位=p99=max=**0.1000 s**，>0.2 s **0 次** | 中位=p99=max=**0.1000 s**，>0.2 s **0 次** |
| 段内 RTF 均值 / 最小 | 0.736 / 0.710 | **0.770** / 0.761 |
| 指令 ω 均值 | 3.84 | 3.83 |
| **加载的存档 / `/map` 起点** | 无 / 小 | **`RMUC2026_cont` 副本 / 190.7 m²** |

⇒ **丢帧、算力、传感器、指令都不是原因**；两跑之间**唯一被测到的差别是「加载了那份存档」**，
但 `small_point_lio` **不读** slam_toolbox 的任何输出（只吃 `/livox/lidar`、`/livox/imu`
和 `base_link→livox_frame` 静态 TF）⇒ **机理上讲不通**，见 §11 第 2 条。

### 5.4 崩之前的现场长什么样（续建那一跑，同一时间轴）

| sim 时间 | 真值（world） | LIO（map） | `map→odom` | 发生了什么 |
|---|---|---|---|---|
| 424.0 | (2.80, −3.76) 静止 | (−8.19, −6.32) | (0.08, 0.02, −0.014) | 一切正常（与从零那一跑逐位一致） |
| 426.5 | 同上 | (−8.27, −6.72) | **(1.92, 3.13, −0.380)** | ⚠️ `map→odom` 先跳了一下（LIO 还没崩） |
| 429.5 | 同上 | (−8.36, −6.21) | (3.20, 4.32, +0.484) | `map→odom` 持续抖动 |
| 431.3 | 同上 | (−10.08, −6.03) | (−3.20, 4.32, +0.484) | ⚠️ **融合位姿第一次跳变**（0.231 m/0.1 s） |
| **432.8** | 同上 | (−11.65, −3.98) | (−2.43, 3.97, +0.429) | ⚠️ **LIO 第一次跳变**（0.203 m/0.1 s = 2.03 m/s），之后连跳 39 次 |
| 440.3 | 同上 | (−15.19, −0.98) | (−0.70, 3.29, +0.199) | 误差 9.28 m |

**注意顺序**：`map→odom` 先动（426.5），LIO 后崩（432.8）。因果方向本次**没有定死**。

## 6. 结果 C：续建那份**被污染的存档** —— 它坏在哪、没坏在哪

### 6.1 先证明"污染"是客观事实（只读复测，本任务自己量的）

`python3 tools/scripts/mapping/pcd_stats.py <pcd>`（**只读**，没有改动任何用户文件）：

| PCD | 点数 | bbox y | **bbox z** | z 的 p95 / p99 |
|---|---|---|---|---|
| `PCD/RMUC2026.pcd`（提交版先验） | 26,618 | [-8.93, 7.12] | **[-0.14, +0.54]** | −0.056 / −0.055 |
| `PCD/RMUC2026_mapped.pcd`（真建图产物） | 1,450,064 | [-9.74, 7.53] | **[-0.53, +1.43]** | −0.291 / −0.254 |
| `PCD/RMUC2026_spl.pcd`（small_point_lio 产物） | 21,387 | [-8.41, 7.02] | **[-0.44, +1.30]** | −0.325 / −0.205 |
| **`PCD/RMUC2026_cont.pcd`（续建那份）** | 109,665 | **[-9.63, +19.30]** | **[-4.31, +23.10]** | **+4.26 / +5.60** |

场地只有 29.15 × 16.05 m、可行驶面 z 起伏 <2 m。**`RMUC2026_cont.pcd` 的 y 到 19.3 m、z 到 23.1 m**
⇒ 里面确实有一段**按错位姿累积**的点云。`docs/continue_mapping.md` §4 也诚实登记了这件事
（"run B 末段 sim 59→74 s LIO 退化，ATE max 1.15 m ⇒ 那十几秒的点云被按错的位姿累积进 PCD"）。

### 6.2 复现实验：把那份存档**复制**成测试名再续建，跑同一条路线

**只复制、不改动用户文件**（`run_drift_diag.sh --resume-from` 只做 `cp`，源文件全程只读）：

```bash
bash tools/scripts/diag/run_drift_diag.sh --lio small_point_lio --tag spl_resume \
     --map-name RMUC2026_diagresume \
     --resume-from src/bringup/rm_nav_bringup/map/RMUC2026_cont \
     --domain 97 --port 11555 --driver-args "--timeout 720"
```

launch 横幅确认走续建分支：`[map_archive] ✅ 续建（slam_toolbox 反序列化）：加载 …/RMUC2026_diagresume.posegraph`；
`/map` 起点就已经是 **190.7 m²**（从零那一跑结束时才 104.2 m²）⇒ **存档确实被加载并接上了**。

### 6.3 结论：存档污染**没有**传到实时位姿（在甩转之前）

| 段 | 从零：LIO RMSE / m/m | **续建：LIO RMSE / m/m** | 从零：融合 RMSE | **续建：融合 RMSE** |
|---|---|---|---|---|
| prep1 | 0.0303 / 0.0049 | 0.0367 / 0.0066 | 0.0198 | **0.0137** |
| A straight | 0.0409 / 0.0105 | 0.0387 / 0.0102 | 0.0294 | **0.0265** |
| D return | 0.0396 / 0.0104 | 0.0371 / 0.0094 | 0.0277 | **0.0282** |
| B slow_turn | 0.0271 | 0.0241 | 0.0294 | **0.0270** |
| C fast_turn | 0.0283 | 0.0393 | 0.0208 | **0.0240** |
| H straight | 0.0352 / 0.0058 | 0.0367 / 0.0078 | 0.0267 | **0.0209** |
| E narrow | 0.0242 / 0.0031 | 0.0224 / 0.0045 | 0.0325 | **0.0226** |
| E2 ramp_probe | 0.0405 / 0.0150 | 0.0531 / 0.0152 | 0.0960 | **0.0758** |
| **S sweep** | 0.0580 | **3.5731（崩）** | 0.1343 | **3.1074（崩）** |

* **`map→odom` 在续建那一跑里从 sim 20 s 到 426 s 一直是恒等 (0.08, 0.02, −0.014)**
  —— 也就是说**加载的位姿图在出生点附近和 LIO 是一致的**，`map→base_link` 没有被"搬歪"。
* 甩转之前**每一段的 ATE 与从零建图没有实质差别**（0.014~0.053 m、每米 0.004~0.015 m）。
* ⇒ **"续建了污染存档"本身不会让实时里程计飘。** 它坏的是 **3D 点云资产**
  （`PCD/<name>.pcd`，将来给 `localization:=icp/gicp` 或 `cloud_accumulator` 用），
  **不在 `/odom` / `map→base_link` 的链路上**。
* ⚠️ **但续建那一跑正是甩转崩掉的那一跑**，而"加载的存档"是两跑之间**唯一被测到的差别** ⇒
  这条不能算干净（§11 第 2 条）。

---

## 7. 判定：四个候选逐条

### (a) 正常累积 —— ❌ 解释不了"才一会儿就飘"
直线 **0.0105 m/m**、原路返回 **0.0104 m/m**、窄沟 **0.0031 m/m**；
整程 ATE RMSE **0.036 m** / max **0.096 m**（51.4 m、408 仿真秒）。
按 1 cm/m 算，**跑 100 m 才漂 1 m**。

### (b) 卡死 / 几何退化触发 —— ❌ 不成立
退化是**真的**（窄沟段 `/scan` 有限束数最小 **115**，其余 ~850；车速只有指令的 62%；
`E`/`E2` 都撞到墙钟上限），但它**没有**带坏 LIO：`E` 段每米 **3.1 mm**。
相关性全部 ≈0（束数 r=0.065、前向回波 r=−0.048、RTF r=−0.026）。
> 旧文档里"卡死会让图歪"成立是有前提的：那次是**旧版直冲控制器 + 连续 9 次卡住 + LIO 闭环**
> （`docs/mapping_2d_from_cloud.md` §4.3）。用纯追踪 + 真值闭环 + 卡住就退出来，
> 这一段既不毁 LIO、也不毁图。

### (c) 转弯诱发 —— ✅ **唯一复现出来的失效模式；有门槛，门槛之上是抛硬币**
* **≤1.1 rad/s（实测）无害**：0.32 rad/s 的 180° 转、0.162 rad/s 的慢转、
  **1.104 rad/s × 22.5 rad** 的快转，三者在**两次独立跑**里误差增量都是 **≈0 或负**。
* **≥0.8~1.9 rad/s（实测；`|cmd ω|` 要给到 3.8~6 rad/s）会抛硬币**：
  6 次同样甩转 **坏 3 次**，幅度 0.62 m ~ 10.68 m。
* **与「地图冷热」无关**（最冷的 `S0` 最干净 0.042 m，最暖的 `S2` 坏了 0.624 m）；
  **与「环境宽窄」也不是充要**（`S0` 前向 2.72 m 最开阔且安全，第一版在开阔区也崩过）。
* **坏起来分两种形态**：慢漂（LIO max 0.62 m、融合比 LIO 还小 ⇒ **slam 能补**）
  vs 失锁跑飞（LIO max 10.68 m、融合 1:1 同步 ⇒ **slam 救不回**）。

### (d) 续建污染存档的假象 —— ❌ 不成立（就实时位姿而言）
续建后 `map→odom` **全程恒等**到 426 s，甩转之前每一段与从零建图**没有实质差别**。
污染是真的，但在 **3D 点云资产**上，不在位姿链路上。
（残留疑点：续建那一跑恰好也是崩掉的那一跑 —— §11 第 2 条。）

### 给用户的直接回答

> **"我这个才多久阿，里程计就飘了，这合理吗"**
>
> **看你怎么开的。**
> * **正常开（直线、拐弯、贴窄道）→ 不合理。** 这条路线上 LIO 每米只漂 **1 cm**，
>   整程 ATE **3.6 cm**；5 次跑没有一次在正常工况下飘过。你如果在这种工况下看到"飘"，
>   先确认你看的是哪个位姿（`map→base_link` 天生比 LIO 差 1.7~1.9 倍，而且它是 2 Hz 的阶梯）。
> * **那一小会儿里有"猛甩原地转"→ 合理，而且是 LIO 的锅。** 本仓仿真底盘要真转到 0.8~1.9 rad/s
>   得给 3.8~6 rad/s 指令；进这个工况 LIO **6 次里坏 3 次**（抛硬币），坏起来
>   从 **0.6 m** 到 **10.7 m**。**轻的 slam 还能补**（融合误差比 LIO 小），
>   **重的直接 1:1 跟着飘**（slam 拉不回来）。而且**跟地图冷热无关** —— 刚启动就转反而最安全。
> * **"续建了那份污染存档"不是现场飘的原因** —— 续建不会让 `map→base_link` 一开始就错（实测恒等）。
>   那份存档坏的是 **3D 点云**，要用之前先体检（见 §8 第 7 条）。

---

## 8. 实用驾驶规则（都可以从上面的数字反推）

1. **直线/窄道/温和转弯：放心开。** 每米 1 cm 量级；窄沟里甚至只有 3 mm/m。
   窄道不会毁 LIO，只会让你走得慢（实测车速掉到指令的 60%）。
2. **把实际偏航率压在 ≤1.1 rad/s。** 这条线上"安全 / 抛硬币"的分界在
   **实测 0.8 ~ 1.1 rad/s** 之间。本仓要拿到 1.1 rad/s 的实际角速度，
   `cmd ω` 大约要给到 3.5 rad/s（§2.3 的传递曲线）。
3. **要转得多，就"分段小角度 + 中间夹直线"。** 实测 `C` 段（22.5 rad / 20 仿真秒、
   每 1.5 秒换向、1.1 rad/s）两次跑都安全；真正危险的是把指令顶到 4~6 rad/s 那一档。
   ⇒ 具体做法：**每次原地转不超过 ~90°，转完走一段直线再转**，不要连续反向甩。
4. **别用 `/odom` 的 twist 字段**（`small_point_lio` 发出来恒为 0），要角速度就自己差分位姿。
5. **看"飘没飘"要看对位姿**：
   * LIO 质量 → `/odom`（或 `odom→base_link`）；
   * 地图 → `/map` + `map→odom`（本仓现成工具：`tools/scripts/diag/map_odom_jump_gate.py`）；
   * `map→base_link` 天生比 LIO 差 1.7~1.9 倍（2 Hz 分段常数），**它抖不等于 LIO 飘**。
6. **卡住就退出来**（别硬顶）：`E2` 里 `map→base_link` 抬到 0.13 m 就是贴着障碍硬推那一下。
   本工具的分段计划里带 `stuck_reverse`。
7. **续建之前先体检存档**：
   ```bash
   python3 tools/scripts/mapping/pcd_stats.py PCD/<名字>.pcd      # 看 z 分位与 bbox
   cat map/<名字>.meta.yaml                                      # 看 world/spawn
   ```
   **z 的 p95 只要超过 +2 m（场地最高护墙 ~1.74 m）就说明这份存档里有退化段，别拿去当 3D 先验。**

---

## 9. 怎么复跑

```bash
# 0) 三个脚本各自的 --help（全部可单独用，不依赖 launch）
python3 tools/scripts/diag/segment_drive.py --help
python3 tools/scripts/diag/lio_drift_analyze.py --help
bash    tools/scripts/diag/run_drift_diag.sh --help

# 1) 只看分段计划（不起仿真）
python3 tools/scripts/diag/segment_drive.py --dry-run

# 2) 一条命令跑完（从零建图；**必须**和 launch 塞进同一次 bash 调用）
bash tools/scripts/diag/run_drift_diag.sh --lio small_point_lio --tag spl_fresh \
     --map-name RMUC2026_diag --domain 96 --port 11550 --driver-args "--timeout 720"

# 3) 对照组：换 LIO（同一路线/仪器/对齐口径，只换一个参数）
bash tools/scripts/diag/run_drift_diag.sh --lio fastlio --tag fastlio \
     --domain 99 --port 11565 --driver-args "--timeout 720"

# 4) 续建复现（**只复制**指定存档到测试名，不动源文件）
bash tools/scripts/diag/run_drift_diag.sh --lio small_point_lio --tag spl_resume \
     --map-name RMUC2026_diagresume \
     --resume-from src/bringup/rm_nav_bringup/map/RMUC2026_cont \
     --domain 97 --port 11555 --driver-args "--timeout 720"

# 5) 冷/暖地图 A/B（本报告 §5.1 用的就是它：同一段甩转在三个"地图成熟度"上各做一次）
bash tools/scripts/diag/run_drift_diag.sh --lio small_point_lio --tag spl_coldwarm \
     --map-name RMUC2026_diag --domain 98 --port 11560 \
     --driver-args "--spec tools/scripts/diag/spec_cold_warm.json --timeout 600"

# 6) 只重跑分析（换阈值/画法时不用再跑仿真）
python3 tools/scripts/diag/lio_drift_analyze.py \
     --csv .tmp_cache/diag/spl_fresh/spl_fresh.csv \
     --events .tmp_cache/diag/spl_fresh/spl_fresh.json \
     --out-json .tmp_cache/diag/spl_fresh/spl_fresh.analysis.json \
     --out-dir docs/img --tag spl_fresh
```

`segment_drive.py` 的关键开关：`--pose-source {gt,lio}`（默认 `gt`，几何确定可比；
`lio` 才是真实用法但会引入正反馈）、`--spec`（换分段计划）、
`--expect-spawn X Y YAW TOL`（脏数据护栏）、`--spin-cmd-cap / --spin-kp / --align-kp`
（角速度闭环；**本仓必须闭环**）、`--cmd-hz`（默认 100）、`--leg-max-wall`、
腿自带的 `stuck_reverse`。

产物：`.tmp_cache/diag/<tag>/{<tag>.log,<tag>.csv,<tag>.json,<tag>.analysis.json}` + `docs/img/<tag>_*.png`。

---

## 10. 回滚

**没有改动任何生产链路**（launch / 参数 / 算法 / 用户存档一个字节都没动）。本次只新增：

| 新增 | 回滚方式 |
|---|---|
| `tools/scripts/diag/segment_drive.py`、`lio_drift_analyze.py`、`run_drift_diag.sh`、`spec_cold_warm.json` | `git rm` 四个文件 |
| `docs/lio_drift_diagnosis.md`、`docs/img/spl_fresh_*.png`、`docs/img/spl_resume_*.png` | `git rm` |
| `docs/mapping_small_point_lio.md` §5.1 末尾追加的一段追测说明 | 删掉那一段 |
| 测试存档 `map/RMUC2026_diag*`、`PCD/RMUC2026_diag*`（若有） | `rm -f src/bringup/rm_nav_bringup/map/RMUC2026_diag* src/bringup/rm_nav_bringup/PCD/RMUC2026_diag*` |
| `map/.session.yaml` | 每次跑完由 `run_drift_diag.sh` 自动还原成跑之前那一份 |

---

## 11. 未验证 / 已知未做

1. **每个条件只跑了 1~2 次**，没有重复统计。旧文档已警告过"同一路线下卡死位置有 ±1 航点的抖动"
   （`docs/mapping_2d_from_cloud.md` §4 的"打折"注）⇒ **单次数字不要当可复现常量**。
   "≤1.1 rad/s 无害"有 **2 次独立跑**支持；"0.8~1.9 rad/s 抛硬币"有 **6 次**（坏 3 活 3），
   **样本仍然很少，只能当"别去试"的工程结论，不能当概率**。
2. **续建那一跑崩掉，与"加载了存档"是否有因果，本次没有定死。** 事实是：
   ① 两跑被测到的差别只有"加载的存档 / `/map` 起点 190.7 m² vs 104.2 m²"；
   ② `map→odom` 在 426.5 s **先**动、LIO 在 432.8 s **后**崩；
   ③ 但 `small_point_lio` **不读** slam_toolbox 的任何输出（它只吃 `/livox/lidar`、`/livox/imu`
   和 `base_link→livox_frame` 静态 TF）⇒ 机理上讲不通；
   ④ 冷/暖那一跑（`spec_cold_warm.json`）已经**否掉**了"地图成熟度"这个更便宜的假设
   （最冷的 `S0` 最干净 0.042 m、最暖的 `S2` 反而坏 0.624 m）。
   **要定死需要**：把续建那一跑重复 2~3 次（n=1 说明不了问题）；
   再跑一次"续建但删掉 `S` 段"看还有没有别的东西会崩。
3. **`lio:=fastlio` 对照组没跑**（§9 第 3 条给了现成命令）。可比性：同一条路线、同一套仪器、
   同一对齐口径，只换 `lio:=` 一个参数。
4. **1.05 m 走廊本身没有被穿过去**：本跑的 `E2` 只是"沟底 → 走廊口"的**接近段**，
   而且撞到 60 s 墙钟上限就收了（实际只走了 4.27 m）。原因：**本仓权威先验图
   `map/RMUC2026.pgm` 把 0.20 m 高的走廊平台标成"占用"**（实测 (-6,-7.8) 处 `occ=true`），
   从它出发规划不出走廊内的合法路线；走廊那一段的证据在 `docs/mapping_2d_from_cloud.md` §4
   （23° 坡 + 1.05 m 走廊：有限束数只有其余段的 31%、坡道前向最近回波 p10=0.13 m、63% 帧 <1.5 m）。
5. **`E` / `E2` 都被 `--leg-max-wall 60 s` 截断**，没走到名义终点 ⇒ 这两段的"里程"是实走里程。
6. **融合位姿是手动复合的**：`map→base_link = (map→odom) ∘ (odom→base_link)`，两条边各取"最新"，
   没有用 tf2 做时间旅行。CSV 里同时记了 `tfm_t / tfo_t / *_age_sim`，可逐行复核。
7. **续建复现用的是一份 `cp` 出来的副本**（`map/RMUC2026_diagresume.*`），
   用户自己的 `RMUC2026_cont.*` / `RMUC2026_mapped.pcd` / `PCD/RMUC2026.pcd.synth.bak` **全程只读**。
8. **仿真底盘"角速度权威只有指令的 0.4~0.8、小指令几乎死区"的根因没有定位**
   （是 `gazebo_ros_planar_move` 的位姿积分 vs ODE 接触摩擦，还是别的）——
   本次只做了"实测传递曲线 + 闭环绕开"，没读插件源码、没做控制变量实验。
   **这条会影响所有"按指令角速度写的"仿真实验**，建议单独开一条线。
9. **`map→odom` 的逐次跳变没有单独量化**（本工具只按 0.2 m/1.5 m·s⁻¹ 的门判"有没有"）。
   更细的增量统计请用现成工具 `tools/scripts/diag/map_odom_jump_gate.py`。
10. 第一版跑（`S` 在最前）**整跑作废**，只保留了它的 `S`/`prep1` 两个数字用作"冷启动崩掉"的
    证据（§5 第一行）；它的 CSV 已被后来的跑覆盖，**不可再复核**。
11. `segment_drive.py` 收尾时 rclpy executor 会打一次
    `terminate called without an active exception`（**发生在 CSV/JSON 已落盘之后**，不影响数据）；
    同一现象在本仓其它 rclpy 工具收尾时也出现过。

---

## 12. 交叉链接

* `docs/mapping_small_point_lio.md` —— 这个槽位的建图链与产物真值（§5.1 有本次追测的指针）。
* `docs/slam_toolbox_scan_drops.md` —— `/scan` 丢帧根因与 `scan_queue_size` A/B；
  §7 的"处理门（0.5 s / 0.447 m）⇒ 只有 ~3% 的扫描真被拿去匹配"是理解
  `map→base_link` 为什么是"分段常数"、以及**为什么 slam 救不回一次大跳**的关键。
* `docs/continue_mapping.md` —— 续建/存档/场地隔离全流程；§4 末尾"诚实登记"了
  `RMUC2026_cont` 里那段 LIO 退化（**本次 §6.1 用 `pcd_stats.py` 复测确认**）。
* `docs/mapping_2d_from_cloud.md` —— 2D 图来源与"图歪"的归因（23° 坡 + 1.05 m 走廊的原测量）。
* `docs/worlds.md` §5.1 —— 采样饥饿陷阱的原始记录；§6 场地对这台车的几何约束。
* `docs/lio_slots.md` —— LIO 槽位参数与离线重放结论。
* `docs/tf_interface_contract.md` —— `map→odom` 单一发布者契约（本次两跑都只有 slam_toolbox）。
* `docs/lio_divergence_no_impact.md` —— **nav 模式**下"没撞击却飘里程计"的实测归因（本文 §5.1 的
  "甩转是唯一复现的失效模式"要按"建图模式 + 脚本化甩转"读；本文 §2.3 的 yaw 传递曲线在那边被更正，
  §5.4 的"因果方向没定死"在那边用 50 Hz 逐帧时间轴定死了：GICP 先察觉、LIO 后跑飞）。


---


## 来源 13/19：`research/lio_divergence_no_impact.md`


> 跑法 = 用户那次：`world:=RMUC2026 mode:=nav lio:=small_point_lio localization:=gicp nav:=mppi
> planner:=smac2d spin_speed:=0.0`，先验图 `map_yaml:=src/bringup/rm_nav_bringup/map/RMUC2026_v3_spl.yaml`，
> 目标 `(-12.64,-0.31)`，前瞻限速按 `config/traversability_criteria.yaml`（默认开）。
> 全部数字来自 `tools/scripts/diag/run_nav_lio_timeline.sh` 的无头整栈跑（隔离见 §6），
> **不是推算、不是回忆**：每条结论后面都挂着信号名与数值，时间轴文件在 §6 列的产物目录里。
>
> ⚠ **2026-10-07 后记**：那次跑用的 `map/RMUC2026_v3_spl.yaml` 已在 2026-10-07 被**提升为默认**
> `map/RMUC2026.yaml`（两份逐字节相同）⇒ 本文说的那张图现在就是 `mode:=nav` 的默认先验；
> `RMUC2026_v3_spl.*` 仍留在 `map/`（`tools/scripts/diag/run_nav_lio_timeline.sh` 的 `--map-yaml`
> 默认值还指着它）。提升依据 / 旧份（`*.bak-20261007`）/ 回滚命令见 `docs/map_assets.md` §3 / §4.4。

---

## 0. 结论（先看这段）

1. **`没撞击 ⇒ 没触发` 不成立；但用户"没撞击"的体感也不是错觉** —— 三次分歧的**触发**都发生在
   `map(-3.2 ~ -4.1, 2.8 ~ 3.3)` 那个**坡脚/台阶沿**上（就是 `docs/slope_speed_limiting.md` §3.2
   记的那个点），表现为 **IMU |a| 单帧 63~91 m/s² 的尖峰簇**（静止基线 ≈9.8 m/s²）。
   nb3 那一跑底盘侧的接触检测器报 **blocked=0 / pushed=0 / contact_events=0** ——
   因为这是**垂向的过台阶冲击**，运动学底盘的水平位移与指令**完全一致**，
   "接触残差"类判据在原理上看不见它。所以"没撞击"= 没撞墙（对），= 没有冲击（错）。
   ⚠️ 另外**每跑发目标后 +0.2~0.8 s 都有一次 32~82 m/s² 的起步顿挫**（§1.3），
   它与结局**无关** —— 只看"有没有 IMU 尖峰"会把它误当撞击，必须看**时刻与位置**。

2. **触发之后是一条固定的因果链**（旧表 4 跑里 3 跑完整走了一遍，顺序从没反过来）：

   ```
   过台阶/坡脚冲击（IMU 单帧 63~91，簇内 >30 采样 17~25 个）→ GICP 首帧未采纳（+0.0~0.5 s）
     → GICP converged=False（+0.1~1.0 s）→ 拒绝级联（累计 99~158 次门限拒绝）
     → LIO /odom 逐帧隐含速度越过底盘物理上限 2 m/s（+0.6~1.4 s；nb2 拉长到 +12 s）
     → 车停住/在原地 recovery 时**自持跑飞**（2 → 70 → 620 m/s）
     → 连续 25 帧被拒 ⇒ GICP 停发 map→odom（+3~4 s）→ 融合位姿整段失效
   ```

   **但这条链的"要不要走完"是抛硬币**：同样的冲击簇在 `nb1`（63.0 @ 同一坡脚）与 `f1/f2/y1`
   上都没有让 GICP 连续失配 ⇒ 分叉点在 **GICP 的位姿门**（`conv_false` 采样数：
   发散的三跑 384 / 948 / 966，没发散的 5 跑**全 0**），不在 IMU、不在雷达几何、不在限速器。

3. **候选①②③④的判定**（每个事件都逐条给了判据，见 §1）：

   | 候选 | 判定 | 决定性信号（本次实测） |
   |---|---|---|
   | ① 高角速度甩转 | ❌ **不是触发** | 分歧起步那一刻**指令 ω ≤ 1.15 rad/s**（nb2 `cmd_wz_max_pre=1.15`、nb3 `=1.29`、v3base `=1.40` 且都在正常行进转弯），而 2.8~3.0 rad/s 的 recovery spin 出现在 LIO 已经飞到 **4.9~9.2 m/s 之后**（nb3：19.4 s 起飘、20.8 s 才 `Running spin`）⇒ 因果方向是反的 |
   | ② 撞击/被挡（含"慢速贴墙"） | ✅ **触发（必要不充分）** | 每次分歧前 0.5~1.5 s 都有地形冲击簇（IMU 峰 67~91）；nb2 还额外有 1 个 blocked 事件（19.95~20.05 s，残差 0.067 m）；nb3/`y1`/`f2` 无接触事件但同样有 63~192 的簇 |
   | ③ 滤波自身数值爆掉 | ✅ **放大机制，不是独立触发** | 车静止（真值 \|v\|<0.1）而 LIO 隐含速度 70.9 / 623.5 m/s、单帧位移 7.09 / 62.18 m；**没有一次**是在无冲击、无 GICP 失锁的情况下自己爆的 |
   | ④ 几何退化（平行墙夹缝） | ❌ | 分歧窗口内雷达**前向**最近回波 p50 = 4.21 m（nb2）/ 2.93 m（nb3）、**后向** 1.84 / 1.19 m ⇒ 根本不在夹缝里；1.05 m 走廊那次（0.0031 m/m）也早就量过是干净的 |

4. **速度：按用户口径"限速器太慢"是真问题，但更严重的是"限速器让车走不到"** ——
   旧表（target 30 / floor 0.45 / 死区 4°/0.04 m）在 v3_spl 图上 **96%~98% 的时间都在限速**，
   4 跑里 3 跑没到点（其中 2 跑被 recovery 卡住、1 跑超时）。
   新表（target 45 / floor 0.60 / 死区 5.5°/0.06 m）的 A/B 见 §3。

---

## 1. 仪器化时间轴与逐事件归因

### 1.1 仪器（必须用 `MultiThreadedExecutor`，否则结论是假的）

两条独立记录仪，同一次跑里同时开：

| 文件 | 职责 | 关键点 |
|---|---|---|
| `tools/scripts/diag/nav_lio_timeline.py`（新，**只读**：不发布任何话题） | 50 Hz 落盘的状态行 `rows.jsonl` + **逐消息**事件行 `events.jsonl` | `MultiThreadedExecutor(4)` + `ReentrantCallbackGroup`；回调只做 O(1) 赋值 + `deque.append`；落盘由独立 50 Hz 定时器做；每行带 `*_age_sim` / `*_age_wall` |
| `tools/scripts/regress/nav_clearance_probe.py`（既有，复用） | 10 Hz 余量/接触/被推 + 发目标 + 汇总 + 收尾零速 | 它的 `samples.jsonl` 也是 `analyze_slope_speed_ab.py` 的口径 |

⚠️ **本仿真里"50 Hz"只是落盘频率**：`/clock` 与 `/odom` 在仿真里都是 **10 Hz**（`/odom` 的 `dt` 恒为
0.1000 s），所以状态行的**有效时间分辨率 = 10 Hz**（同一仿真时刻会有多行 ⇒ 分析脚本去重时
**必须把 IMU 峰值并起来**，否则撞击簇会少数倍 —— 本仓踩过：15 个 >30 只剩 5 个）。
真正"逐帧"的量走 `events.jsonl`（每条消息一行，含单步 `step` 与隐含速度 `spd`）。

时间轴上的量：`/odom`（LIO）**逐帧**位置/偏航/**单步 step/隐含速度 spd**、`/odom_ground_truth`、
`/cmd_vel`·`/cmd_vel_nav`·`/cmd_vel_chassis` 三路指令 ω、`/scan` 有限束数 + **前向 ±20°/后向 ±20° 最近回波**、
`/livox/imu`（|a| 含重力 ⇒ 静止 ≈9.8）、`/gicp_registration/{converged,fitness_score}`、
`/speed_limit` + `~/slope_speed_stats`、action feedback 的 recovery 计数、`map→odom`（含发布年龄）。
恢复行为的**类型**（spin/backup/wait）由 runner 从 launch.log 的 `[behavior_server]: Running <plugin>` 行取，
墙钟戳经 `samples.jsonl` 的 `(wall_abs, sim)` 对插值到仿真时间（`analyze_noimpact_divergence.py::sim_of`）。

⚠️ **采样饥饿陷阱**：本仓踩过单线程 `rclpy.spin_once()` 在 ~80 Hz `/tf` 洪水下被饿死 ⇒
"LIO 位移是真值 2.6 倍"的**假结论**（`docs/worlds.md` §5.1）。本次两条记录仪都按段驱动那套做法
（MT 执行器 + 只存值 + 独立落盘定时器 + age 字段），`rows.jsonl` 的 `lio_age_sim` p50 = 0.02 s、max = 0.06 s，
`mo_age_max` ≤ 0.006 s（nb1 汇总）⇒ **没有饿死**。

### 1.2 对齐口径（所有"误差"的地基）

`/odom`（LIO）在自己的 odom 系里从原点起，`/odom_ground_truth` 在 Gazebo world 系（出生点世界坐标
≈(10.93, 2.52)）。两者差**一个常量刚体变换**，用**起步静止窗口**（前 30 个真值静止采样）估一次、
**全程沿用**（`analyze_noimpact_divergence.py::_build_timeline`，四跑都得到 yaw 偏移 0.000、
平移 (10.93, 2.53)）。**绝不逐段重对齐** —— 那会把漂移当坐标系差减掉。

### 1.3 四跑的总账（同一栈、同一图、同一目标）

| 跑 | 结局 | 墙钟驾驶 | 真值里程 | IMU max (>30/>50) | LIO `err` max | LIO 单帧 step max | GICP 停发 | recovery | 数据源 |
|---|---|---|---|---|---|---|---|---|---|
| `v3base_validate` | 超时未到点 | 100.0 s | 8.40 m | **87.7** (10/7) | **8672 m** | —（10 Hz） | ✅ 34.4 s 起 | 15 | 10 Hz |
| `nb1` | 超时未到点（最近 3.70 m） | 100.0 s | 18.90 m | 63.0 (21/4) | 0.46 m ✅ | 0.183 m (1.8 m/s) | ❌ | 22 | 50 Hz 落盘/10 Hz 有效 |
| `nb2` | **ABORTED**（status 6） | 39.6 s | 20.13 m | **90.7** (26/8) | **1192 m** | **7.092 m (70.9 m/s)** | ✅ 37.9 s 起（34.8 s 陈旧） | 11 | 同上 |
| `nb3` | 超时未到点 | 100.0 s | 8.47 m | **67.2** (18/9) | **21311 m** | **62.18 m (621.8 m/s)** | ✅ 21.8 s 起 | 16 | 同上 |

**IMU 冲击要分成两类看**（`imu_a_peak` = 每 20 ms 窗口的 |a| 峰值保持；
⚠️ 50 Hz 状态行会被"仿真钟只有 ~10 Hz"这一步去重，去重时**必须把峰值并起来**，否则撞击簇会少数倍）：

| 跑 | 发目标后 +0.2~0.8 s 的**起步顿挫**簇（>30 采样数 / 峰值） | 中途**地形**冲击簇（>30 / >50 / 峰值 / 峰位置(map)） | 分歧 |
|---|---|---|---|
| `nb1` | 6 / 31.8 | 15 / 1 / 63.0 @ (-3.59, 3.13) | ❌ |
| `nb2` | 1 / 82.3 | **25 / 7 / 90.7** @ (-4.37, 10.75) | ✅ |
| `nb3` | 1 / 43.6 | **17 / 9 / 67.2** @ (-4.07, 2.98) | ✅ |
| `f1` | 5 / 43.5 | 8 / 1 / 57.3 @ (-3.44, 3.30) | ❌ |
| `f2` | 4 / 52.2 | 12 / 3 / **192.2** @ (-3.91, 2.78) | ❌ |
| `y1` | 3 / 80.8 | 20 / 7 / **142.8** @ (-3.89, 2.89) | ❌ |

* **起步顿挫每跑都有**（发目标后 +0.2~0.8 s、31.8~82.3 m/s²）：底盘起步顿一下，与结果无关
  （`y1` 那次 80.8 比任何跑都高，照样到点且没分歧）。⇒ **"看到 IMU 尖峰就断定撞击"是错的**，
  必须看**时刻与位置**。
* **地形冲击簇每跑也都有**（都在 `map(-3.4~-4.1, 2.8~3.3)` 那道坡脚/台阶沿上，峰值 57~192 m/s²）。
* ⇒ **峰值高度和簇大小都分不开结果**（`f2` 192.2、`y1` 142.8 没分歧；`nb3` 67.2 分歧了）。
  分得开的是 **GICP 有没有被打进拒绝级联**：`conv_false` 采样数 = `nb2` 384 / `nb3` 966（发散）
  vs `nb1`/`f1`/`f2`/`y1` **全 0**（没发散）。⇒ **"抛硬币"抛在 GICP 的位姿门/内点上，不在 IMU 上**：
  冲击是必要条件，但决定成败的是"这一下有没有让扫描匹配连续失配"。

### 1.4 事件 1（`nb3`）：**最干净的一次** —— 车还在正常开，里程计已经飞了

工具直接产出的里程碑表（`--timeline`；`imuP` = 之前 4 s 内的 IMU 峰值保持、`cmdωP`/`gtωP` = 指令/实测角速度峰值、
`fwd`/`back` = 雷达前向/后向最近回波最小值、`LIO v` = 之前 4 s 内 LIO 隐含速度峰值）：

```
     t(sim) 里程碑                         imuP   cmdωP    gtωP     fwd    back   LIO v  接触/行为
      13.30 IMU |a|>30（起步顿挫簇）          43.6    0.41    0.49    2.52    2.49    0.81
      18.80 GICP converged=False        67.2    1.29    1.39    2.57    1.04    1.79
      18.81 GICP 首帧未被采纳                 67.2    1.29    1.39    2.57    1.04    1.79
      19.10 LIO 误差 >0.50 m              67.2    1.29    1.39    2.57    0.94    1.79
      19.40 ★LIO 速度持续 >2 m/s（底盘上限）      67.2    1.29    1.39    2.57    0.87    2.17
      20.80 恢复行为 spin                   67.2    1.29    3.00    2.67    0.76    6.32  spin
      21.80 map→odom 停发                 64.5    3.00    3.00    2.67    0.76    9.23  spin
```

逐帧细节（`events.jsonl` 的 `/odom` 行，dt 恒 0.1000 s = 10 Hz）：

```
  18.30  IMU 34.2（地形冲击簇起，车 1.20 m/s，cmd ω 0.61，雷达 fwd 2.85 m / back 1.20 m）
  18.50  IMU 42.7   err 0.13 m
  18.70  IMU 67.2 ← 簇峰（位置 map(-4.07, 2.98) = 那道坡脚/台阶沿）
  18.80  converged=False（fit 0.005）        ← 定位链先坏
  18.81  GICP 本帧未被采纳（score=0.00491 m²）
  19.10  err 0.54 m，LIO 单帧 0.169 m = 1.69 m/s（真值 1.25）
  19.40  LIO 单帧 0.217 m = **2.17 m/s** ⇒ 第一次持续越过底盘上限 2.0
  19.50  LIO 2.44 m/s（真值 0.85，cmd ω 0.13 —— **指令和实测都低**）
  19.80  3.22 m/s；20.10 3.86；20.50 4.87 m/s
  20.80  ★行为 Running spin（**比 LIO 越界晚 1.4 s**）
  21.30  cmd ω 2.95（spin 要求 2.95 rad/s，实测随后 2.83）—— 这一刻 LIO 已在 7~8 m/s
  21.46  ★★★ 连续 25 帧被拒 ⇒ GICP 停发 map→odom
  最终   车静止而 LIO 单帧最大 **62.18 m**（621.8 m/s）、`err` 最大 **21312 m**
```

判据：`impact_t=17.6`（簇峰 67.2）、`cmd_wz_max_pre=1.29`、20.6 s 起飘时指令已归零、
`still_frames=949`、`lio_spd_max_while_still=623 m/s`、`scan_fwd_p50_win=2.93 m`（不在夹缝）、
`contact_pre=[]`（**底盘侧零接触事件**）、`conv_false_t=18.8`（比 LIO 越界早 0.6 s）、`frame_step_max=62.18 m`。
⇒ **判定：② 撞击触发 + ③ 自持跑飞（放大）。** ① 被因果顺序否掉（spin 在起飘之后）；④ 被前向回波否掉。

### 1.5 事件 2（`nb2`）：同一条链，另一种收尾（目标被 ABORT）

```
     t(sim) 里程碑                         imuP   cmdωP    gtωP     fwd    back   LIO v  接触/行为
      14.70 IMU |a|>30（起步顿挫）            82.3    0.58    0.47    2.52    2.49    2.70
      20.00 LIO 误差 >0.50 m              71.8    1.13    1.15    2.42    1.40    3.60  blocked
      20.20 GICP converged=False        79.3    1.13    1.15    2.42    1.37    3.60  blocked
      20.20 GICP 首帧未被采纳                 79.3    1.13    1.15    2.42    1.37    3.60  blocked
      32.80 ★LIO 速度持续 >2 m/s（底盘上限）      27.7    0.81    0.80    2.56      —     2.10  pushed
      34.77 恢复行为 spin                   31.3    1.09    1.14    2.58      —     4.91  pushed spin
      37.90 map→odom 停发                 90.7    3.00    3.00    2.65      —    13.34  spin,wait
```

关键段（`--window 31.5,34.2`）：车还在以 0.7~1.2 m/s 走，LIO 已经 2~3 m/s：

```
     t   lio_err  gt|v|  cmdω |  imu  |   fit   conv  rec
  31.50    7.453  0.85   0.21 |  9.9  | 0.016  True   7
  32.40    8.432  0.82   0.11 | 10.9  | 0.014  True   8
  32.65  gate: GICP 本帧未被采纳（score=0.02284 m²）
  32.70    8.817  0.58   0.17 | 10.3  | 0.023  False  8
  32.80   LIO 单帧 0.210 m = **2.10 m/s**（逐帧速度第一次越过底盘上限）
  33.30    9.711  1.22   0.01 |  9.8  | 0.053  False  8   ← IMU 无辜（9.8 = 静止基线）
  34.77  ★行为 Running spin（比逐帧越界晚 2.0 s）
  34.90   16.121  0.00   0.00 |  9.8  | 0.138  False 10   ← 车静止、原地 3.0 rad/s
  35.90   22.791  0.00   0.00 | 10.9  |  nan   False 11   ← fitness→nan
  37.41  ★★★ 连续 25 帧被拒绝 ⇒ 停止发布 map→odom（到 66.2 s 一直陈旧）
  68.90  LIO 单帧 7.092 m = **70.9 m/s**（车静止）
```

判据：`impact_t=19.0`（簇峰 90.7）、`contact_pre=[('blocked',19.95,20.05,0.067)]`、
`cmd_wz_max_pre=1.15`、`frame_spd_max=70.92`、`still_frames=343`、`mo_stale=(37.9,66.2)`、
`behav_pre=[]`（分歧前**没有任何** recovery 行为）⇒ **判定：② 撞击（+被挡）→ GICP 失锁 → ③ 自持跑飞。**
注意这条链**拉得很长**（20.2 GICP 失锁 → 32.8 LIO 越界，中间 12 s 只有"慢漂"），
所以从外部看更像"开着开着慢慢飘了"，而不是"撞了一下就飞"。

### 1.6 事件 3（`v3base_validate`，10 Hz 老记录仪）：把"跑飞"与"撞击"之间那 4 秒拍清楚

这一跑没有 50 Hz 仪器（它是我拿来验跑法/当 A/B 基线的第一跑），但已经把关键顺序拍下来了：
`21.4 s IMU 59.7`（车在 `map(-3.24,3.25)`，真值 1.18 m/s）→ `21.9 converged=False` →
`22.6 progress checker 中止 follow_path`（10 s 没进展）→ `22.7~25.3` 恢复行为 spin/wait/backup（recovery 1→6）+
`24.4~25.1 IMU 68.7~87.7`（贴着墙磨）→ `25.0 GICP 停发 map→odom` → **`25.9 s` 起车静止（cmd 0、
真值 \|v\|=0、\|\|a\|\| 回到 9.8）而 LIO 从 3.2 → 5.1 → 7.3 → 9.6 m/s 一路加速**，最终 `err` 中位 751 m、max 8672 m。
⇒ 与 nb2/nb3 **同一条链**；差别只是这次"跑飞"发生在车完全停住之后，所以从外部看
**"没人碰它、它就飘了"** —— 这正是用户那句话的来历。

### 1.7 反例（`nb1`）：**同样的地点、同样量级的冲击，却什么都没发生**

`nb1` 走同一条路线、同一个坡脚，地形冲击簇 **63.0 m/s² @ `map(-3.59,3.13)`**
（比 `nb3` 的 67.2 只低 4；>30 有 15 个采样、>50 有 1 个）——
但 `GICP converged=False` **一次都没有**（`conv_false_n = 0`）、`fit_nan = 0`、`mo_stale = 0`、
`lio_err max = 0.46 m`、单帧 step max 0.183 m（1.8 m/s）⇒ **没有分歧**。
它的问题不是定位，而是**走不到**（22 次 recovery、最近 3.70 m、最后 `plan_age` 卡到 17 s、指令归零）。
⇒ 同一份栈、同一个坡脚、同样量级的冲击，**一边是 21 km 发散、一边是 0.46 m 干净** ——
差别不在冲击本身，而在"这一下有没有让 GICP 连续失配"（§1.3 最后一条）。

---

## 2. 这对之前"撞击 ⇒ 分歧"的说法意味着什么

**结论：方向对，因果链补全了，但两处要更正。**

1. **要更正的**：`docs/lio_drift_diagnosis.md` §5.1 把"高角速度甩转"记为**唯一复现出来的失效模式**
   （6 次甩转坏 3 次）。那份实测是**建图模式 + 脚本化定点甩转**，而且它 §2.3 那张
   "指令 ω → 实测 ω"的传递曲线（`cmd 0.6 ⇒ 实测 0.02`、最大只有 1.89 rad/s）**与现在的栈不符**：
   本次用同一行内的指令/真值配对量到的是 **≈1:1**（`analyze_noimpact_divergence.py --yaw-transfer`）：

   | 指令 \|ω\| 箱 | n | 指令 p50 | **实测 p50** | 比值 p50 | 原地占比 |
   |---|---|---|---|---|---|
   | [0.2,0.5) | 24 | 0.34 | 0.36 | 1.07 | 0.00 |
   | [0.5,1.0) | 33 | 0.65 | 0.67 | 1.01 | 0.03 |
   | [1.0,1.5) | 14 | 1.07 | 1.11 | 1.00 | 0.21 |
   | [2.5,3.5) | 7 | 2.83 | **2.83** | 1.00 | **1.00** |

   （`nb3`；符号一致率 1.00，n=75。nb2 同样 1.00/109。）⇒ 底盘**听得懂** yaw 指令、也没有那种
   巨大死区；"要 1.9 rad/s 得给满 6"的结论不能再引用。这不影响"甩转会坏 LIO"这个**观察**，
   但它影响**限幅该限到哪**：要限住"实测 ≤1.1 rad/s"，就得把**指令**限到 ~1.1，而不是 2.0。

2. **要补的**：nav 模式下的分歧**不需要**"甩转"，也**不需要撞墙**，只需要**一次够重的过台阶/坡脚冲击**
   ——而这类冲击对底盘接触判据是**不可见**的（nb3：blocked=pushed=0）。因此，
   "撞击 ⇒ 分歧"里的"撞击"应当改成**"够重的垂向冲击（IMU 单帧 ≳50 m/s²）"**；
   用户说的"没撞击"与这条并不矛盾，因为：
   * 冲击是**过台阶**（车轮/底盘磕一下），不是撞墙，车不停、导航不报错；
   * 症状（里程计飞）出现在**1~4 s 之后**，而且经常出现在车**已经停住/正在原地 recovery** 的时候
     （v3base 25.9 s、nb2 33.4 s、nb3 20.6 s）—— 那一刻现场最"安静"，所以看着最像"自己飘的"。

3. **不变的部分**：`map→odom` 先动还是 LIO 先动，本次**定死了**：
   GICP 的 `converged=False` / 拒绝级联**总是早于** LIO 越界（nb3：18.8 vs 19.5；
   nb2：20.2 vs 32.8；v3base：21.9 vs 25.9）。而"连续 25 帧被拒 ⇒ 停发 map→odom"**总是最后**
   （nb3 21.5、nb2 37.4、v3base 24.6）⇒ GICP 是**先察觉、后放弃**，它不是肇事者。

---

## 3. 限速 / 角速度 A/B（同一栈、同一图、同一目标）

### 3.1 变量

| 臂 | 变量 | 取值 |
|---|---|---|
| `base`（旧表） | 4 跑：`v3base_validate` / `nb1` / `nb2` / `nb3` | `peak_target 30`、`floor 0.45`、死区 `4°/0.040 m`、坡度表 `[1.66,1.10,0.83,0.66,0.55,0.45,0.45]`、台阶表 `[0.040]→[0.45]` |
| `fast`（新表） | `f1` / `f2` | `peak_target 45`、`floor 0.60`、死区 `5.5°/0.060 m`、坡度表 `[2.48,1.65,1.24,0.99,0.82,0.61,0.60]`、台阶表 `[0.060]→[0.60]` |
| `yawcap` | `y1` | 旧表 + MPPI `wz_max 2.5→1.4` + `velocity_smoother.max_velocity[2] 3.0→1.4`（`min_velocity` 同步）+ `behavior_server.max_rotational_vel 3.0→1.4` |

生效核对（从两跑自己的 launch.log 里读节点自报的参数行，避免"以为改了其实没改"）：

```
f1 : floor=0.6  slope_change_deadband=5.5 deg step_deadband=0.06 m  target ≤ 45 m/s²
nb3: floor=0.45 slope_change_deadband=4   deg step_deadband=0.04 m  target ≤ 30 m/s²
```

### 3.2 表（数值由 `analyze_noimpact_divergence.py` 从产物直接算）

**旧表（`speed_limit_*` 改前）—— 4 跑**

| 跑 | 结局 | 墙钟驾驶 | 里程 | \|v\| p50/p90/max | \|v\|<0.6 | IMU max（>30/>50 采样数） | 真发散 | `lio_err` max | 单帧 step max | `conv_false` | `mo_stale` | recovery | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `v3base_validate` | 超时（差 16.4 m） | 100.0 s | 8.40 m | 0.00 / 0.63 / 1.72 | 73.2 s | 87.7（10/7） | ✅ | 8672 m | —（10 Hz） | 948 | ✅ | 15 | — |
| `nb1` | 超时（差 3.70 m） | 100.0 s | 18.90 m | 0.05 / 0.99 / 1.87 | 61.6 s | 63.0（21/4） | ❌ | 0.46 m | 0.183 m | 0 | ❌ | 22 | 0.81 |
| `nb2` | **ABORTED** | 39.6 s | 20.13 m | 0.73 / 1.48 / 1.97 | 12.8 s | 90.7（26/8） | ✅ | 1193 m | 7.092 m | 384 | ✅ | 11 | 0.80 |
| `nb3` | 超时（差 14.9 m） | 100.0 s | 8.47 m | 0.00 / 0.00 / 1.84 | 72.3 s | 67.2（18/9） | ✅ | 21312 m | 62.182 m | 966 | ✅ | 16 | 0.78 |

**新表（提速档）—— 2 跑**

| 跑 | 结局 | 墙钟驾驶 | 里程 | \|v\| p50/p90/max | \|v\|<0.6 | IMU max（>30/>50） | 真发散 | `lio_err` max | 单帧 step max | `conv_false` | `mo_stale` | recovery | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `f1` | ✅ **到点** | **26.5 s** | 22.58 m | **1.16 / 1.86 / 2.06** | **0.3 s** | 57.3（13/1） | ❌ | 0.39 m | 0.198 m | 0 | ❌ | **0** | 0.77 |
| `f2` | ✅ **到点** | **26.8 s** | 22.58 m | **1.19 / 1.72 / 1.85** | **0.6 s** | 192.2（16/5） | ❌ | 0.54 m | 0.201 m | 0 | ❌ | 4 | 0.75 |

**yaw 限幅档（旧表 + 指令 \|ω\| ≤ 1.4）—— 1 跑**

| 跑 | 结局 | 墙钟驾驶 | 里程 | \|v\| p50/p90/max | \|v\|<0.6 | IMU max（>30/>50） | 真发散 | `lio_err` max | `conv_false` | `mo_stale` | recovery | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `y1` | ✅ 到点 | 31.0 s | 22.24 m | 0.89 / 1.54 / 1.99 | 2.7 s | 142.8（23/9） | ❌ | 0.61 m | 0 | ❌ | 4 | 0.80 |

**限速器自身口径（同一批跑）**

| 跑 | `[slope_speed]` 行数 | limit p50 | limit min | 不限速占比 | `why` 前三 |
|---|---|---|---|---|---|
| `nb1`（旧） | 1307 | 0.93 | 0.45 | 4.3% | slope_change 727 / step 434 / slope 90 |
| `nb2`（旧） | 385 | 0.95 | 0.45 | 18.7% | step 209 / slope_change 89 / none 72 |
| `nb3`（旧） | 1027 | 0.86 | 0.45 | 1.5% | slope_change 845 / step 115 / slope 52 |
| **`f1`（新）** | 694 | **1.09** | **0.60** | 5.0% | slope_change 368 / step 171 / slope 120 |
| **`f2`（新）** | 682 | **1.02** | **0.61** | 5.3% | slope_change 436 / step 159 / slope 51 |
| `y1`（旧+限幅） | 762 | 0.89 | 0.45 | 3.1% | — |

**离线扫掠（`bench/slope_speed_offline.cpp`，不需要 Gazebo，同一份判据代码）**

| 场景 | 旧表「过特征上限」 | 新表「过特征上限」 |
|---|---|---|
| **平地（对照，含 5 mm 噪声）** | 2.00 m/s，**平地误限=否** | 2.00 m/s，**平地误限=否** |
| 10% 坡（5.7°）坡脚 | 0.70 | **1.24** |
| 11° 坡脚 | 0.63 | 0.68 |
| 24° 坡脚 | 0.49 | **0.83** |
| 0.045 m 台阶 | 0.57 | **1.29**（< 新死区 0.06 ⇒ 不再触发） |
| 0.10 m 台阶 | 0.57 | **0.73** |
| 0.30 m 台阶（高台） | 0.57 | 0.63 |
| 「2 m 前」上限 | 1.28~1.38 | 1.34~1.49 |

### 3.3 读法

1. **结局（最硬的一条）**：旧表 **0/4 到点**（1 次 ABORTED + 3 次超时），新表 **2/2 到点**，且都在 ~27 s
   里走完 22.6 m（计划 23.4 m）。用户抱怨的"慢"，在 v3_spl 这张图上其实是**走不到**：
   旧表 4 跑里有 3 跑把时间全花在 recovery 上（`nb1` 22 次、`nb3` 16 次、`v3base` 15 次）。
2. **速度**：能直接比的只有"没发散的那一对"——`nb1`（旧表、没分歧）vs `f1/f2`：
   `|v|` p50 **0.05 → 1.16/1.19 m/s**、`|v|<0.6` 时长 **61.6 s → 0.3/0.6 s**、recovery **22 → 0/4 次**。
   （用户口径 p50 0.71 vs 1.23、`|v|<0.6` 9.4 vs 4.8 s 是在**旧图 + 到点**的跑上量的；
   换到 v3_spl 图之后旧表根本到不了点，所以这两组数不能直接比。）
3. **定位安全**：新表 2/2 跑 `conv_false=0`、`mo_stale=0`、`lio_err max ≤0.54 m`、单帧 step ≤0.201 m
   （= 2.0 m/s，正好是底盘上限）⇒ **没有发散**。旧表 3/4 发散（误差 1.2~21 km、单帧 7~62 m）。
4. **但"更快的表更安全"这句话不能说满**：地形冲击簇**每跑都有**（§1.3 表），
   `f2` 那次峰值 **192.2 m/s²** 是全部跑里最高的、`y1` 也有 142.8，而它们都**到点且没发散**；
   反过来 `nb3` 只有 67.2 却发散了。⇒ 峰值高度、>30/>50 采样数**都分不开结果**。
   分得开的是 **GICP 有没有被打进拒绝级联**：`conv_false` 采样数 `nb2` 384 / `nb3` 966（发散）
   vs `nb1`/`f1`/`f2`/`y1` **全 0**。这条是本次最有用的判据（也解释了为什么 §4.2 的 yaw 限幅治不了它）。
5. **限速器并没有"挡住"那些冲击**：发散那两次的撞击时刻，限速器**已经在压速**
   （`nb2` 撞击时 limit 0.46~0.87 m/s、指令仅 0.06~0.19 m/s，`nb3` limit 0.80~1.04 m/s），
   但真值速度仍有 **0.84~1.18 m/s** ⇒ 车是**带着动量滑过台阶沿的**。
   这是"限速 ≠ 防撞"的直接证据：限速只在特征前 2~3 m 开始压，而车在特征处的速度
   取决于**能不能在那 2~3 m 里真的减下来**（`speed_limit_brake_mps2=0.4` 是"规划减速度"的
   乐观下界；仿真底盘物理上能到 4 m/s²，但**指令链的跟踪**在限速突降时跟不上）。
6. **RTF**：0.75~0.81（全部跑），所以"墙钟 100 s 预算"在仿真时间里都 ≈ 75~81 s，
   A/B 之间可比；`f1/f2` 与 `y1` 是**并行/单独**跑混合，但 RTF 差 ≤0.06。

### 3.4 平地回归（`run_nav_smoke_regress.sh`，官方 P0 回归）

同一条 2.9 m 平地路线（`--goal -1.0 2.0`，默认先验图 = `map/RMUC2026.yaml`）：

| 跑 | 表 | 结果 | 用时 | 真值 max\|v\| | 里程 | recovery | 限速行数 |
|---|---|---|---|---|---|---|---|
| `ln_reg_new` | 新表 | ✅ **PASS** | 3.8 s | 1.133 m/s | 2.920 m | 0 | 229 |
| `ln_reg_old` | 旧表 | ❌ **FAIL（跑废了，与限速无关）** | 1.1 s | 0.000 m/s | 0.004 m | 0 | — |
| `ln_reg_old2` | 旧表（重跑） | ✅ PASS | 3.6 s | 1.185 m/s | 2.935 m | 0 | — |
| （文档对照，`docs/slope_speed_limiting.md` §3.3）`ssl_reg_on` | 旧表 | PASS | 3.6 s | 1.19 m/s | 2.90 m | 0 | — |

* `ln_reg_old` 那次是**跑废**：launch.log 里 `controller_server` 报
  `Transform data too old when converting from map to odom`（`Data time 20.5 s / Transform time 15.45 s`）、
  planner 报 `Invalid frame ID "map"` ⇒ **`map→odom` 没起来**，nav2 拿不到位姿，车没动（真值位移 0.004 m）。
  这是启动期的一次偶发（与限速表无关：限速器不参与 TF/初始化），已重跑一次取有效值。
* **"平地不被限速"这条性质由离线扫掠守着**：新旧表在 `平地（对照）` 一行都是
  `过特征上限=2.00 m/s / 平地误限=否`（含 5 mm 点云噪声）。整栈这条 2.9 m 路线
  **并非"无特征"**：走廊里能看见出生点附近的坡（日志 `坡=16.6°、近坡=10.8°`），
  所以限速器在这条路上**确实会压速**（229 行），这也是为什么它只能当"回归不退化"的证据，
  不能当"平地不限速"的证据 —— 后者用上面的离线那一行。
* 新表在平地上的代价：**+0.2 s（3.6 → 3.8 s，单次差值，不是统计量）**、真值峰值
  1.185 → 1.133 m/s、`pose_delta` 2.935 → 2.920 m ⇒ 与 `docs/slope_speed_limiting.md` §3.3
  记的"+0.4 s / +12%"同量级或更小，**都在单跑噪声里**。

---

## 4. 推荐默认值（都已落地）+ 回退

### 4.1 采纳：**前瞻限速提速档**（`speed_limit_*`）

理由一句话：旧表在 v3_spl 图上 **0/4 到点**、`|v|<0.6` 占 61~73 s；新表 **2/2 到点**、
`|v|<0.6` 0.3~0.6 s、`conv_false=0`、平地上只差 0.2 s、离线扫掠的"平地不误限"性质保持不变。

| 键 | 改前 | 改后 | 依据 |
|---|---|---|---|
| `speed_limit_peak_target_mps2` | 30.0 | **45.0** | 唯一直接由"用户嫌慢"推动的物理量；45 < 实测最小致损簇（>50 那一档）；实测峰值 50~88 出现在**限速已生效**的旧表跑里 ⇒ 30 这个目标值本来就没把峰值压到 30 附近 |
| `speed_limit_floor_mps` | 0.45 | **0.60** | 限速器 96% 时间在触发 ⇒ 地板就是巡航速度下界；0.60 让"≤8°"的结点不再需要地板兜底 |
| `speed_limit_slope_change_deadband_deg` | 4.0 | **5.5** | `why=slope_change` 占限速行 56~79%（4~5.5° 的场地起伏被当特征）；真坡脚 Δ坡 实测 9~16°，不会漏 |
| `speed_limit_step_deadband_m` | 0.040 | **0.060** | `why=step` 占 13~33%；0.06 m 仍远小于判据自己的台阶闸 0.15 m |
| `speed_limit_slope_knots_mps` | [1.66,1.10,0.83,0.66,0.55,0.45,0.45] | **[2.48,1.65,1.24,0.99,0.82,0.61,0.60]** | 按 `v = round_down_2(45·Δt_eff/tanθ × 0.975)` 重算，反算峰值 43.4~43.9 ≤ 45 |
| `speed_limit_step_knots_m` / `_mps` | [0.040] / [0.45] | **[0.060] / [0.60]** | 与台阶死区同步（两处必须一致） |

落地点（两处**必须逐键一致**，`check_slope_speed_table.py` 会断言）：
`src/bringup/rm_nav_bringup/config/traversability_criteria.yaml` 与
`src/perception/rm_perception/rm_ground_traversability/include/rm_ground_traversability/slope_speed_limit.hpp`。

**回退（一键）**：把上表六处改回左列即可；`git revert <本 commit>` 也可以。
只想"慢一点但比旧表快"：只改 `speed_limit_floor_mps: 0.50` + `peak_target: 38.0` + 两张表
按同一公式重算（`check_slope_speed_table.py` 会告诉你哪个结点超了）。

### 4.2 不采纳：**角速度限幅**（数据不支持）

* `docs/lio_drift_diagnosis.md` §5.1 的 ①（甩转）**不是** nav 模式下的触发：3 次分歧起步时
  指令 \|ω\| ≤ 1.15 / 1.29 / 1.40 rad/s，而 2.8~3.0 rad/s 的 recovery spin 都在 LIO 已经飞到
  6~8 m/s **之后**才出现（§1.4/§1.5 的因果排序）。
* 唯一一跑 `y1`（旧表 + `wz_max=1.4` + `max_rotational_vel=1.4` + smoother wz 1.4）**没发散、到点**
  （31.0 s、p50 0.89 m/s），但它同时**比新表慢 4.5 s、p50 低 0.27 m/s**，而且
  **一次跑分不开"限幅有效"与"这次本来就没磕重"**（旧表 4 跑里也有 1 跑没发散）。
  ⇒ 结论：**不作为默认**。若将来要压"原地甩转"的暴露（例如把 3.0 rad/s 的 recovery spin
  当风险），按 §6 的 `--kv` 三处一起改、并跑 ≥3 跑/臂再定。

### 4.3 不动：`vx_max` / MPPI 速度上限

`f1/f2` 的 `|v|` p90 = 1.72~1.86，上限 `min(MPPI vx_max 2.5, velocity_smoother 2.0) = 2.0`
⇒ 还剩 7%~14% 余量，**瓶颈不是它**（p50 只有 1.16~1.19，是限速表在压）。所以这次**没有**动
`speed_limit_vx_max` / MPPI `vx_max` / smoother `max_velocity[0..1]`。

---

## 5. 什么**没有**帮上忙（别重复走）

1. **"限速更保守 ⇒ 更安全"**：旧表（更慢、更保守）4 跑 3 跑发散，而且发散时**限速器已经在压速**
   （撞击瞬间 limit 0.46~0.87 m/s、指令 0.06~0.19 m/s，真值却还有 0.84~1.18 m/s）⇒
   冲着"防住坡脚冲击"去收紧限速表，方向是错的：**压不住"带着动量滑过台阶沿"**。
2. **把 ①（甩转）当触发**（§4.2）：因果顺序直接否掉，`y1` 那一跑也不能把"限幅"与"运气"分开。
3. **`speed_limit_direction_aware`**（既有结论，本文未再试）：实测不触发，默认保持 `false`。
4. **把 recovery 的 spin 关掉/改小来"省时间"**：`nb1` 22 次 recovery 里真正的原因是
   **progress checker 反复超时**（10 s 无进展），不是 spin 本身慢；真正省时间的是"别被限速压到走不动"。
5. **"等 GICP 自己纠回来"**：`map→odom` 一旦连续 25 帧被拒就**停发**（nb2 34.8 s / nb3 113 s 陈旧），
   融合位姿在剩下的整段里都是旧的 ⇒ 这一跑不会自愈，只能重定位。（这也是为什么
   `docs/path_clearance_and_contact.md` 里的 `mo_stale` 要单独盯。）

---

## 6. 怎么复跑

```bash
# ① 归因跑（= 本文 §1 的每一条）：一条 bash 调用跑完（起栈→50 Hz 只读时间轴→间隙记录仪→发目标→收尾）
bash tools/scripts/diag/run_nav_lio_timeline.sh --tag nb4 --domain 217 --port 11917 \
     --goal -12.64 -0.31 --extra "spin_speed:=0.0" --max-drive-sec 100 --post-goal-sec 25
#   产物：.tmp_lio_noimpact/out/nb4/{rows,events}.jsonl + samples.jsonl + summary.json + gate.txt + launch.log

# ② 归因报告（每个发散事件：判据表 + 事件序 + 里程碑）
python3 tools/scripts/diag/analyze_noimpact_divergence.py --timeline .tmp_lio_noimpact/out/nb4
#   指定窗口的人读时间轴（§1.4/§1.5 那两张表）
python3 tools/scripts/diag/analyze_noimpact_divergence.py --timeline .tmp_lio_noimpact/out/nb4 \
     --window 16.8,22.4 --window-dt 0.25
#   指令 ω → 实测 ω 传递（§2 第 1 条）
python3 tools/scripts/diag/analyze_noimpact_divergence.py --timeline .tmp_lio_noimpact/out/nb4 --yaw-transfer

# ③ 限速 A/B 表（多跑一起出）
python3 tools/scripts/diag/analyze_noimpact_divergence.py \
     .tmp_lio_noimpact/out/f1 .tmp_lio_noimpact/out/f2 .tmp_lio_noimpact/out/y1
#   限速器自己的口径（限速行/why 分布/压速占比）仍用既有分析器
python3 tools/scripts/regress/analyze_slope_speed_ab.py .tmp_clear/out/ssl_off .tmp_clear/out/ssl_on3

# ④ 真源/物理自检（改表必跑）
python3 tools/scripts/regress/check_slope_speed_table.py
python3 tools/scripts/regress/check_traversability_criteria.py

# ⑤ 单变量覆盖（跑完自动还原 + diff 核对）
#   可通行性真源（**保注释**就地改，见 runner 的 --ckv）：
bash tools/scripts/diag/run_nav_lio_timeline.sh --tag x1 --domain 218 --port 11918 \
     --ckv speed_limit_floor_mps=0.45 --ckv speed_limit_peak_target_mps2=30.0
#   nav2 参数（--kv，走 apply_nav_kv.py，跑完还原）：
bash tools/scripts/diag/run_nav_lio_timeline.sh --tag x2 --domain 219 --port 11919 \
     --kv "nav2_params_sim_controller_mppi.yaml:controller_server.FollowPath.wz_max=1.4"
```

隔离（所有跑法一致，缺一不可）：一次 bash 调用跑完（本环境每条命令一个 PID namespace，
调用一结束里面的 gzserver 全被清掉）、`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、
独立 `GAZEBO_MASTER_URI` 端口、`unset DISPLAY`、`nav_rviz:=False lio_rviz:=False`、
收尾零速 + `pkill gzserver`。**并行跑只在"两次跑的配置逐字节相同"时才安全**
（否则 `--kv/--ckv` 会互相污染），本文的 `f1`/`f2` 就是并行的一对。

---

## 7. 未验证 / 已知残余风险

1. **样本量**：新表 2 跑、yaw 限幅 1 跑、旧表 4 跑。"分歧率"是抛硬币型（旧表 3/4），
   所以 §3.3 第 4 条那条"峰值 vs 成簇"的趋势**只是趋势**，要做统计得每臂 ≥5 跑。
2. **45 m/s² 这个目标值没有独立标定**：它是在"锚点 86 m/s² @ 1.7 m/s、tanθ 0.10"这一条
   实测锚点上外推的；45 只保证"预期峰值 ≤45"，但**实测峰值**里出现过 82.3 / 87.7 / 192.2
   （后两者分别在新表跑里）⇒ 模型只对"坡面型"特征有效，对**台阶型单帧冲击**外推没意义。
3. **`f2` 那个 192.2 m/s² 单帧冲击的成因未定位**（同帧 LIO 偏航跳 -0.611 rad 后又自己纠回）：
   当时 GICP score 0.014 / converged=true / 拒帧 0。需要单独仪器（IMU 原始 200 Hz + 车轮/接触）
   才能说清它是"磕台阶"还是"仿真穿模"。
4. **限速突降时的"指令 vs 真值"跟踪差**没有专门量：本文只按 0.1 s 采到"指令已经 0.06 而真值 0.84 m/s"
   这个现象（§3.3 第 5 条）。要给出"该把 brake 提到多少"的结论，得先做一条 50 Hz 的
   `指令/真值加速度` 专用时间轴。
5. **`spin_speed`（小陀螺）默认 5.0 rad/s 没有被限幅**：用户跑的是 `spin_speed:=0.0`（直通），
   本次 A/B 全部在 0.0 下做的；`spin_speed=5.0` 那条链（fake_vel_transform 用固定角速度替换
   nav 的 ω）**不在本次结论范围内**。
6. **`v3base_validate` 只有 10 Hz 时间轴**（它是验跑法那次），所以它的"单帧 step"缺失；
   50 Hz 口径的结论都来自 `nb1/nb2/nb3/f1/f2/y1`。
7. **没有验证"重定位能不能救回来"**：3 次发散后 `map→odom` 停发就再没恢复（本次没有发
   `/initialpose` 去试），"GICP 失锁后自愈/重定位"这条路未测。
8. **没测其它 world / 其它目标点**：结论只覆盖 `world:=RMUC2026` + 目标 `(-12.64,-0.31)`
   + 先验图 `RMUC2026_v3_spl.yaml`；换图/换点后"限速器 96% 时间在触发"这个前提可能不成立。
9. **离线扫掠的"平地"是合成平地**：真实平地（含地面点云噪声/坡度）由 `ln_reg_*` 那两跑守着，
   但它们只有 2.9 m、且走廊里能看见坡（所以"整栈平地上限速器完全不触发"**没有被证明**）。

---

## 8. 交叉链接

* `docs/lio_drift_diagnosis.md` —— 分段实测的原始报告（§2.3 的 yaw 传递曲线**由本文 §2 更正**；
  §5.1 的"甩转是唯一复现的失效模式"要按"建图模式 + 脚本化甩转"的口径读，nav 模式下的触发见本文 §1）
* `docs/slope_speed_limiting.md` —— 前瞻限速的实现/推导/首轮 A/B（本文 §3 是它的"提速档"A/B）
* `docs/gicp_divergence_and_jitter.md` —— GICP 侧的拒帧/抖动（本文 §1.6 的 `map→odom` 停发口径与它一致）
* `docs/path_clearance_and_contact.md` §2.2/§7.5 —— 撞击点 `map(-3.6,3.1)` 的余量分析
* `docs/traversability_plan.md` §9 —— 缺陷 ③（坡脚冲击 ⇒ LIO 发散）的判据/状态


---


## 来源 14/19：`research/research_p2l_scan_stall.md`


Scope: ROS 2 Humble + Gazebo Classic, 6 research questions asked by the parent agent.
Legend: **[V]** = verified in a primary source I read (source code, official issue, or an accepted answer).
**[R]** = verified as a *real report by a human* but unconfirmed root cause.
**[I]** = my inference from source reading — labelled as such.

Local artifacts read for grounding (paths relative to repo root):

- `src/perception/rm_perception/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp` (244 lines)
- `src/perception/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml` and the installed copy
  `install/pointcloud_to_laserscan/share/pointcloud_to_laserscan/config/laserscan_params.yaml`
- `src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py`
- `src/perception/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/src/ground_segmentation_node.cc`
- `/opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp`, `/opt/ros/humble/include/tf2/tf2/buffer_core.hpp`
  (tf2_ros 0.25.23 installed on this machine)

---

## 0. Two repo-local defects found while researching (read this first)

**0a. `pointcloud_to_laserscan` and `linefit_ground_segmentation` are launched *without* `use_sim_time`.**

`bringup_sim.launch.py` L205-212:

```python
bringup_pointcloud_to_laserscan_node = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in',  ['/segmentation/obstacle']), ('scan',  ['/scan'])],
    parameters=[os.path.join(get_package_share_directory('pointcloud_to_laserscan'),
                             'config', 'laserscan_params.yaml')],   # <-- no use_sim_time
    name='pointcloud_to_laserscan'
)
```

L197-202 (linefit) likewise passes only `parameters=[segmentation_params]`. Verified:
`laserscan_params.yaml` (src **and** install) and `segmentation_sim.yaml` contain **zero** occurrences of
`use_sim_time`; `grep -rn SetParameter src/` finds no global `SetParameter` action. So both nodes run on the
**system clock** while the rest of the stack runs on sim time. **[V]**

This matters directly for item 1: `tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock())` (node.cpp L83) builds
a buffer whose `clock_` is the *wall* clock. See item 1 for exactly which behaviours that does and does not change
(the honest answer is narrower than the folklore).

**0b. The p2l tf2 `MessageFilter` is a no-op in the current configuration.** **[V]**

`target_frame: livox_frame` (config) and `/segmentation/obstacle` carries `header.frame_id = livox_frame`
(linefit copies the input header verbatim: `ground_segmentation_node.cc` L141-146). tf2's
`BufferCore::addTransformableRequest` begins with `if (target_frame == source_frame) return 0;`
(0 == "immediately transformable"), and `canTransform`/`canTransformInternal` short-circuit the same way. So the
filter passes every message synchronously without ever touching the cache, and node.cpp L174
(`if (scan_msg->header.frame_id != cloud_msg->header.frame_id)`) skips the in-callback transform too.
Consequence: **items 1/2's tf2 mechanisms cannot be the *active* cause of a p2l-side stall in this config.**
They are still worth fixing as latent risk (item 2/3c).

---

## 1. The tf2 error `"the timestamp on the message is earlier than all the data in the transform cache"` under `use_sim_time`

### (a) Finding

This exact string is **not** a tf2 buffer-core message. It is the human-readable text of the enum value
`tf2_ros::filter_failure_reasons::OutTheBack` inside `tf2_ros::MessageFilter`, and it is emitted by
`MessageFilter::signalFailure()` as a **throttled INFO on the consuming node's own logger**. **[V]** The precise
code condition that produces it is `msg_stamp + cache_time_ < latest_common_time`, where `cache_time_` is the tf2
buffer's cache duration (default **10 s**). So the literal meaning is: *"this message's stamp is more than the tf2
cache duration older than the newest transform I hold for that frame chain"* — i.e. the message is too **stale** to
ever be transformable, not that the transform is missing. The dominant real-world causes are (i) clock-domain
mismatch between the stamping node and the TF publishers, (ii) the buffer being **cleared** by a time jump, and
(iii) plain message delay (DDS/executor congestion) exceeding the cache duration.

### (b) Exact code / config / commands

The reason string and its only producer **[V]**:

```cpp
// /opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp
// L87  (enum comment)
/// The timestamp on the message is earlier than all the data in the transform cache
// L105-107
case filter_failure_reasons::OutTheBack:
  return "the timestamp on the message is earlier than all the data in the transform cache";
// L705-716  — the log you are seeing
void signalFailure(const MEvent & evt, FilterFailureReason reason) {
  ...
  RCLCPP_INFO_THROTTLE(node_logging_->get_logger(), *clock, 2500,
    "Message Filter dropping message: frame '%s' at time %.3f for reason '%s'",
    frame_id.c_str(), stamp.seconds(), get_filter_failure_reason_string(reason).c_str());
}
```
Note the second argument: the throttle uses `node_clock_->get_clock()`, i.e. the **consumer node's** clock.

The exact "too old" test **[V]** (`tf2/src/buffer_core.cpp`, `addTransformableRequest` and
`testTransformableRequests`):

```cpp
getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
if ((latest_time != TimePointZero) && (time + cache_time_ < latest_time)) {
  return 0xffffffffffffffffULL;      // addTransformableRequest -> "never transformable"
}
...
if ((latest_time != TimePointZero) && (req.time + cache_time_ < latest_time)) {
  do_cb = true; result = TransformFailure;   // testTransformableRequests
}
```

Cache duration default **[V]**: `/opt/ros/humble/include/tf2/tf2/buffer_core.hpp` L73
`static constexpr Duration BUFFER_CORE_DEFAULT_CACHE_TIME = std::chrono::seconds(10);`

Buffer **clearing on time jumps** (cause ii) **[V]** — `tf2_ros/src/buffer.cpp`, `Buffer::onTimeJump`:

```cpp
if (RCL_ROS_TIME_ACTIVATED == jump.clock_change || RCL_ROS_TIME_DEACTIVATED == jump.clock_change) {
  RCLCPP_WARN(getLogger(), "Detected time source change. Clearing TF buffer.");  clear();
} else if (jump.delta.nanoseconds < 0) {
  RCLCPP_WARN(getLogger(), "Detected jump back in time. Clearing TF buffer.");   clear();
}
```
This callback is registered in the `Buffer` constructor with a threshold of "anything backwards is a jump,
`on_clock_change = true`". So on any Gazebo world reset / `/clock` restart every queued message becomes
"earlier than all the data" at once.

Diagnosis commands:

```bash
# 1. the clock-domain check everyone skips — run it for EVERY node in the chain
ros2 param get /pointcloud_to_laserscan use_sim_time     # currently False  <-- defect 0a
ros2 param get /ground_segmentation    use_sim_time      # currently False  <-- defect 0a
ros2 param dump /pointcloud_to_laserscan

# 2. is the message stamp in the sim-time domain?
ros2 topic echo /clock --once
ros2 topic echo /segmentation/obstacle --once --field header
python3 -c "import time; print('wall', time.time())"      # compare with header.stamp.sec

# 3. does the transform chain exist, and what is its buffer age?
ros2 run tf2_ros tf2_echo     odom livox_frame
ros2 run tf2_ros tf2_monitor                       # prints per-frame rate + buffer length
ros2 run tf2_tools view_frames                     # writes frames.pdf / frames.gv
ros2 topic echo /tf_static --once                  # static vs dynamic stamps
```

`ros2 topic echo` and `ros2 topic hz` both default to the `sensor_data` QoS preset (BEST_EFFORT) in Humble
[V: `ros2topic/verb/echo.py` L41 `default_profile_str = 'sensor_data'`; `ros2topic/verb/hz.py` L43/L274-278
`qos_profile_sensor_data`], so they *do* receive your BEST_EFFORT `/scan` — no extra `--qos-reliability` needed.
This is a common false alarm; don't chase it.

Fixes reported in the wild:

| Root cause | Concrete fix | Source |
|---|---|---|
| One node on wall clock (the accepted answer to a Gazebo-classic + Humble slam_toolbox case with this exact error) | "ensuring `robot_state_publisher` had `use_sim_time=True`. **All nodes** should have `use_sim_time=True`, if they are using sim time." | [robotics.SE 115148 → answer 115182](https://robotics.stackexchange.com/questions/115148/ros2-humble-gazebo-slam-toolbox-tf2-dropped-message-reason-the-timestamp-on) **[V, accepted]** |
| Message delay > tf2 cache caused by an RMW bug in Humble's Fast-DDS | backport [eProsima/Fast-DDS#3195](https://github.com/eProsima/Fast-DDS/pull/3195); **two independent reporters confirm the problem disappears with `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`** (`sudo apt install ros-humble-rmw-cyclonedds-cpp`) | [navigation2#3352](https://github.com/ros-navigation/navigation2/issues/3352) — full log + maintainer @SteveMacenski closing it as "not a nav2 issue, file with Fast-DDS/RMW" **[V, real report ×3]** |
| TF buffer deleted by lifecycle `cleanup` (slam_toolbox) | After `cleanup`+`reconfigure`, messages are older than all data; the reporter's actual crash was a memory leak, so the MessageFilter lines were a *symptom* | [robotics.SE 118242 → answer 118275](https://robotics.stackexchange.com/questions/118242/message-filter-dropping-message-and-error-code-11-in-async-slam-toolb) **[R]** |
| Same error, "all nodes use sim time", `transform_timeout`/`tf_buffer_duration` tuning did not help | open, unanswered | [robotics.SE 118268](https://robotics.stackexchange.com/questions/118268/ros2-humble-gazebo-classic-docker-slam-toolbox-keeps-dropping-laserscan-wit) **[R]** |

### (c) Honest narrowing for *this* stack

**[I, from source]** For p2l specifically, `use_sim_time=false` does **not** by itself break tf2 lookups, because tf2
compares *message stamps* against *cached transform stamps* — both taken off the wire — so the clock object is not
part of the comparison. The clock enters tf2 in only three places: (1) the polling loop of
`Buffer::canTransform(t,s,time,timeout,err)` (`clock_->now()` for elapsed time), (2) the timer created in
`Buffer::waitForTransform` via `timer_interface_->createTimer(clock_, timeout, ...)`, and (3) `onTimeJump`
detection. p2l never calls `now()` to stamp anything (it copies `cloud_msg->header`, L149), so today the wall clock
is only a latent landmine. It becomes *active* the moment either (a) `target_frame` is changed to `base_link`/`odom`
or (b) anything downstream relies on p2l's `onTimeJump` buffer-clearing. Fix it anyway — it costs one line and it is
the only node in the chain that is inconsistent.

---

## 2. What makes `tf2_ros::MessageFilter` drop every message permanently?

### (a) Finding

Four distinct permanent/半-permanent failure modes exist, and only one of them prints the `OutTheBack` text:
`EmptyFrameID` (permanent for that publisher, distinct message), `QueueFull` (transient),
`NoTransformFound`/`Unknown` (transforms absent), and **the silent one: if `getLatestCommonTime` cannot produce a
`latest_time`, the "too old" fast-fail branch is skipped entirely and the request is queued forever — the message
sits in `messages_`, the callback never fires, and *nothing is logged*.** That last mode is the only one that
matches "process alive, subscriber matched, low CPU, zero output, no warning". Recovery is per-message for
`OutTheBack`/`QueueFull` (yes, it recovers) but only "on the next new transform that satisfies the request" for the
queued-forever case — which can look permanent.

### (b) Exact source lines

`add()` — the frame-id gate and the queue **[V]**:

```cpp
// message_filter.hpp L365-372
std::string frame_id = stripSlash(mt::FrameId<M>::value(*message));
if (frame_id.empty()) { messageDropped(evt, filter_failure_reasons::EmptyFrameID); return; }
// L414-420
if (queue_size_ != 0 && messages_.size() + 1 > queue_size_) {
  ++dropped_message_count_; ... messages_.pop_front();   // reason QueueFull
}
// L441-451 — always an async request, no "frame already target" shortcut here
tf2_ros::TransformStampedFuture future = buffer_.waitForTransform(
    target_frame, frame_id, stamp, buffer_timeout_,
    std::bind(&MessageFilter::transformReadyCallback, this, std::placeholders::_1, handle));
```

`Buffer::waitForTransform` — the three outcomes **[V]** (`tf2_ros/src/buffer.cpp`):

```cpp
auto handle = addTransformableRequest(cb, target_frame, source_frame, time);
if (0 == handle) {                       // immediately transformable
  promise->set_value(lookupTransform(target_frame, source_frame, time)); call_callback = true;
} else if (0xffffffffffffffffULL == handle) {   // NEVER transformable
  promise->set_exception(... "Failed to transform from " + source_frame + " to " + target_frame ...);
  call_callback = true;
} else {
  auto timer_handle = timer_interface_->createTimer(clock_, timeout, ...);   // <-- clock_ is the node clock
}
```
`transformReadyCallback` maps **any** exception from `future.get()` to `error = OutTheBack` **[V]** — so the
"earlier than all the data" wording is a catch-all, not a literal diagnosis. Do not trust the words; measure the
stamps.

Answers to the specific sub-questions:

| Sub-question | Answer | Evidence |
|---|---|---|
| Interaction with `use_sim_time` | The filter itself is clock-agnostic for *lookups* (uses message stamps). The node clock is used only for the throttle in `signalFailure` and for the `waitForTransform` timer. **`checkFailures()` is dead code in Humble** — grep finds the definition at L608 and **no call site**, so the "Dropped X%% of messages" WARN never prints. | **[V]** grep of `message_filter.hpp` |
| `target_frame` == the message's own frame | **Passes everything.** `addTransformableRequest` returns 0 immediately; `canTransform` likewise. Not a failure mode. | **[V]** `buffer_core.cpp` |
| Static transforms on `/tf_static` | A `StaticCache` reports `TimePointZero` as its latest time, and `getLatestCommonTime` skips `TimePointZero` entries (`if (latest.first != TimePointZero) common_time = min(...)`). So a chain joined **only** by statics yields `latest_time == TimePointZero` ⇒ the "too old" branch is skipped ⇒ **requests queue forever with no log** (the silent mode). Your `base_link → livox_frame` edge is static (`frames_2026-09-22_21.23.01.gv`: rate 10000.0, buffer length 0.0, most-recent 0.0 = StaticCache), but `odom → base_link` is dynamic, so `latest_time` is real for AMCL. | **[V]** `buffer_core.cpp` `getLatestCommonTime`; **[V]** local `frames_*.gv` |
| `queue_size` | p2l passes `input_queue_size_` as the MessageFilter `queue_size` **and** as the QoS depth. It is declared as `declare_parameter("queue_size", std::thread::hardware_concurrency())` and **is not set in `laserscan_params.yaml`**, so on this 28-core box it is **28**. `queue_size == 0` means infinite. | **[V]** node.cpp L65-66, L88-93, L121-123; config has no `queue_size` |
| Does `transform_tolerance: 0.01` matter? | **No.** `tolerance_` is never passed to the `MessageFilter` constructor (L88-93). It is used only as the *timeout* of the in-callback `tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_))` at L177 — and that line is skipped when the frames already match. The README says the same: "*Time tolerance for transform lookups. Only used if a `target_frame` is provided.*" | **[V]** node.cpp L62/L88-93/L174-183; README L30 |
| Recovery | Recovers for `OutTheBack` (each message is judged independently) and `QueueFull`. `EmptyFrameID` never recovers for that publisher. The queued-forever case recovers only when a new transform makes `testTransformableRequests` fire — so a *stuck frame* looks permanent. | **[V]** `buffer_core.cpp` `testTransformableRequests` |

`getLatestCommonTime` returns `TF2_CONNECTIVITY_ERROR` when frames are in different trees — and callers pass
`error_string = 0`, so **that error is silently swallowed** and `latest_time` stays `TimePointZero`. **[V]**

### (c) Sources

- [tf2_ros `message_filter.hpp` (humble)](https://github.com/ros2/geometry2/blob/humble/tf2_ros/include/tf2_ros/message_filter.hpp) — local copy at `/opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp`, tf2_ros 0.25.23
- [tf2 `buffer_core.cpp` (humble)](https://github.com/ros2/geometry2/blob/humble/tf2/src/buffer_core.cpp)
- [tf2_ros `buffer.cpp` (humble)](https://github.com/ros2/geometry2/blob/humble/tf2_ros/src/buffer.cpp)
- [MessageFilter API docs (Humble)](https://docs.ros.org/en/ros2_packages/humble/api/tf2_ros/generated/classtf2__ros_1_1MessageFilter.html)

---

## 3. `pointcloud_to_laserscan` (ROS 2 port) known issues

### (a) `subscriptionListenerThreadLoop` / `sub_.unsubscribe()` — real, and already patched here

**[V, real reports]** Upstream node.cpp L114-142 computes `pub_->get_subscription_count() +
get_intra_process_subscription_count()`; if it is `0` it calls `sub_.unsubscribe()`, and it only re-subscribes on a
**later graph-change event** (`wait_for_graph_change(event, timeout)`, 100 ms timeout). Two upstream issues document
this loop:

- [issue #77 "Node didn't recognize subscriber"](https://github.com/ros-perception/pointcloud_to_laserscan/issues/77) —
  the reporter quotes both log lines verbatim ("*Got a subscriber to laserscan…*" / "*No subscribers to laserscan,
  shutting down pointcloud subscriber*") and states that **with a subscriber present `/scan` still received nothing**.
- [issue #107 "RAM exhaustion due to accumulation of graph events"](https://github.com/ros-perception/pointcloud_to_laserscan/issues/107) —
  *"the created graph events never appear to be GC'd, eventually leading to RAM exhaustion"*, pointing at
  `subscriptionListenerThreadLoop` L131-132; proposes reusing the event object (as `rclcpp/src/rclcpp/client.cpp`
  L170-174 does). Open, 0 comments.

**Caveat I must flag** **[V, logic]**: I found **no upstream issue that states "after unsubscribe it never
re-subscribes"**. And the state you report — `ros2 topic info /segmentation/obstacle --verbose` still lists p2l as a
matched subscriber and `ros2 node info` still lists it — is **inconsistent** with the unsubscribe hypothesis,
because `message_filters::Subscriber::unsubscribe()` destroys the `rclcpp::Subscription`, which then disappears from
the graph. Treat the parent's local fix (commit `30ab296`, which replaced the unsubscribe with
`RCLCPP_INFO_ONCE` and removed the loop-exit unsubscribe) as correct defensive hardening, but **not as the proven
cause of the current symptom**.

### (b) Reports of silently producing no output

| Issue | Content | Status |
|---|---|---|
| [#105](https://github.com/ros-perception/pointcloud_to_laserscan/issues/105) | ROS 2 Humble: "`/scan` publishes correctly when drone is idle… after takeoff the frequency gradually drops until it completely stops… the publisher is still present, but no data is flowing." **Config is nearly identical to yours: `target_frame` = the sensor's own frame, `use_sim_time: True`, `transform_tolerance: 0.10`, `queue_size: 1`.** Only reply (Rayman) suggests BEST_EFFORT QoS / packet loss. **Open, unanswered.** | **[R] — closest published match to your symptom** |
| [#104](https://github.com/ros-perception/pointcloud_to_laserscan/issues/104) | "No results seen form the topic which has the converted data" — closed by author same day (no diagnosis) | **[R]** |
| [#101](https://github.com/ros-perception/pointcloud_to_laserscan/issues/101) / [#100](https://github.com/ros-perception/pointcloud_to_laserscan/issues/100) | Users unable to get output; they resort to editing the .cpp / adding a non-existent `output_log_level` param | **[R]** |
| [#68](https://github.com/ros-perception/pointcloud_to_laserscan/issues/68) | Foxy/Ignition: `/scan` **does** publish but every `ranges` entry is `.inf` — i.e. "publishes but output empty". Cause was the frame/height/range filters rejecting every point | **[R]** |
| [robotics.SE 116917 → answer 116918](https://robotics.stackexchange.com/questions/116917/debugging-pointcloud-to-laserscan-with-ouster-lidar-in-ros-2-humble) | "Debugging pointcloud_to_laserscan … (Humble)" — the accepted answer found the **remap was wrong** (`-r input:=/ouster/points` instead of `cloud_in:=/ouster/points`), so the subscription never matched and the node was silent forever. Cheapest possible "silent no output" cause. | **[V, accepted answer]** |

### (c) `target_frame` / `queue_size` recommendations, and `target_frame: ""`

**[V]** README (L29-30, L47-48): "*`target_frame` (str, default: none) — If provided, transform the pointcloud into
this frame before converting to a laser scan. **Otherwise, laser scan will be generated in the same frame as the
input point cloud.***" and "*`queue_size` (double, default: detected number of cores) — Input … queue size.*"

**[V, source]** `target_frame: ""` is not merely documented — it removes the tf2 machinery entirely:

```cpp
// node.cpp L61-62
target_frame_ = this->declare_parameter("target_frame", "");
tolerance_    = this->declare_parameter("transform_tolerance", 0.01);
// L82-96
if (!target_frame_.empty()) {           // <-- Buffer + TransformListener + MessageFilter all created here
  tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  ...
  message_filter_ = std::make_unique<MessageFilter>(sub_, *tf2_, target_frame_, input_queue_size_,
                                                    this->get_node_logging_interface(),
                                                    this->get_node_clock_interface());
  message_filter_->registerCallback(...);
} else {                                 // <-- otherwise: plain subscription, no tf2 at all
  sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
}
// L149-152 — output frame
scan_msg->header = cloud_msg->header;
if (!target_frame_.empty()) { scan_msg->header.frame_id = target_frame_; }
```

So for your chain, **`target_frame: ""` is the recommended setting**: `/scan.header.frame_id` stays
`livox_frame` (identical output), and the tf2 Buffer, the `TransformListener` (an extra `/tf`+`/tf_static`
subscription), and the `MessageFilter` are all not created. Note also that the fixed `queue_size` should be set
explicitly — leaving it unset silently picks `hardware_concurrency()` (28 here), which is a large QoS depth for a
0.2 MB @ 10 Hz topic.

Ordered fix list for p2l (lowest risk first):

```python
Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    parameters=[laserscan_params, {'use_sim_time': use_sim_time}],   # 1) clock consistency
    remappings=[('cloud_in', ['/segmentation/obstacle']), ('scan', ['/scan'])],
    name='pointcloud_to_laserscan')
```
```yaml
# laserscan_params.yaml
target_frame: ""      # 2) drop the tf2 filter entirely (identical output frame)
queue_size: 5         # 3) be explicit instead of hardware_concurrency()
```

---

## 4. "Timestamp Discipline and Message Synchronization" (wimblerobotics.github.io)

### (a) Finding

**[V — fetched, HTTP 200]** The page exists but is a thin, ~1-page distillation with **no tf2/MessageFilter,
`use_sim_time`, or point-cloud content at all.** Its own two cited sources are the `message_filters` approximate-sync
tutorial and the ROS 2 clock/time design article, and it is tagged only `debugging message-filters multi-machine ros2
synchronization timestamps`. So it does **not** answer your question; the only transferable content is its
clock-discipline framing.

### (b) Verbatim content

Sections: *Why This Matters*, *Distilled Takeaways*, *Practical Value*, *Corroborating References*, *When to Read the
Original Source*. Its distilled takeaways, verbatim:

- "Header stamps are part of the data contract, not decoration."
- "`message_filters` only works well when timestamps and QoS settings are already coherent."
- "Approximate synchronization is often the practical choice, but it does not excuse bad clocks."
- "Multi-machine systems amplify timestamp problems because network delay and unsynchronized device assumptions get
  mixed together."
- "Time discipline should be designed before downstream fusion or perception tuning."

Practical value, verbatim: "Stamp messages from a consistent ROS-aware clock path. / Keep QoS aligned across
synchronized topics or the filter layer will fail before your logic runs. / Use approximate sync for realistic sensor
timing, but choose queue size and age penalty deliberately. / Audit clock assumptions whenever fusion, perception, or
recorded playback seems intermittently wrong."

### (c) Sources

- <https://wimblerobotics.github.io/software/timestamp-discipline-and-message-synchronization/> **[V]**
- <https://design.ros2.org/articles/clock_and_time.html> (its own primary reference)
- <https://docs.ros.org/en/jazzy/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html>

---

## 5. "Why does a ROS 2 point-cloud consumer fall behind when transforms use a different clock?"

### (a) Finding — with an important authority caveat

**[V — found, fetched via a text proxy because the site is Cloudflare-protected]** The article exists at
`agents.stackoverflow.com` ("Stack Overflow for Agents", beta). **It is an AI-generated Q&A, not human-verified
knowledge: the answer shows `Votes: 0`, `Claims: 0 | Votes: 0`, "No verifications yet", and
"Trust score pending", and the author is an agent account (`valentinbot Agent`).** Its content is nonetheless a
reasonable checklist and happens to agree with the primary sources in item 1. The question itself is explicitly
derived from a Stack Overflow question by PaulE. **Do not cite this as authority** — cite the tf2 code and
navigation2#3352 for the mechanism.

### (b) Summarised claims (7 extracted "claims" + the answer body)

Claims list as published: the primary cause of "timestamp outside buffer" in ROS 2 mapping pipelines is a **clock
mismatch between publishers and the mapper**; **increasing the transform-filter queue only masks the symptom**;
**every node publishing data or transforms consumed by the mapper must set `use_sim_time: true`**; **if a sensor
driver cannot use sim time, re-stamp the cloud in the mapper's clock domain before it reaches the transform filter**;
**a successful lookup at `tf2::TimePointZero` combined with a failure at the message's own timestamp confirms a
clock-alignment problem**; and *only* tune queue depth after clocks and the transform chain are verified.

Body specifics worth stealing: (1) `ros2 param get <node> use_sim_time` for every node; confirm `/clock` is actually
published, because with `use_sim_time=true` and no `/clock` *all* stamps are garbage. (2) Distinguish the two error
classes — "target frame unknown" (wrong/missing frame ⇒ `tf2_echo`) from "timestamp outside buffer" (transform exists
but not at that time ⇒ clock domain). (3) The systematic check: for each cloud, log
`lookupTransform(world, sensor, msg.header.stamp)` **and** `lookupTransform(world, sensor, tf2::TimePointZero)`; if
the latest lookup succeeds while the stamped one fails, it is a timestamp/clock problem, not a tree problem. That is
the single most useful idea in the article and it is directly falsifiable with the code in item 2. (4) Suggested
queue depth after fixing clocks: 10-20 messages.

### (c) Sources

- <https://agents.stackoverflow.com/questions/614b62ed-f9ff-45d4-9fb5-da156dac1fc3> **[V, but low-authority AI-generated; direct fetch returns HTTP 403 Cloudflare — retrieved via `r.jina.ai`]**
- Primary-source equivalents for the same mechanism: [tf2 `buffer_core.cpp`](https://github.com/ros2/geometry2/blob/humble/tf2/src/buffer_core.cpp), [navigation2#3352](https://github.com/ros-navigation/navigation2/issues/3352)

---

## 6. Grep-able binary search inside the running p2l node

Goal: separate **(a) input callback never fires** from **(b) tf2 filter drops everything** from **(c) it publishes but
the output is empty/all-inf**. Work down; each step is a yes/no with a specific command or log line.

**Step −1 — cheap sanity (30 s).** `ros2 topic hz /scan` and `ros2 topic hz /segmentation/obstacle` both default to
`sensor_data` (BEST_EFFORT) QoS and therefore *will* see your topics; if `/scan` shows nothing here, the problem is
real, not a QoS artefact of the tool. **[V]**

**Step 0 — rule the whole chain in or out.** Run all three rates simultaneously:

```bash
ros2 topic hz /livox/lidar/pointcloud & ros2 topic hz /segmentation/obstacle & ros2 topic hz /scan
```
- All three die together ⇒ **Gazebo plugin / writer-side block, not p2l.** This is the failure your repo already
  documented: `ground_segmentation_node.cc` L76-85 ("RELIABLE 写者会被跟不上速率的消费者堵住队列 … 阻塞点落在 Gazebo 的
  sensor 回调里 ⇒ … `/scan` 一起停更 180s 且不自恢复") and `docs/debug_fastlio_cartographer.md` §5.3.
- Only `/scan` dies ⇒ continue to step 1. **[I from source + repo measurement]**

**Step 1 — (a) vs (b): is the callback firing at all?** p2l is *your* build, so instrument it (or, with zero code
change, flip the config and compare):

```bash
# Non-invasive A/B: does removing the filter restore output?
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args -r cloud_in:=/segmentation/obstacle -r scan:=/scan -p target_frame:="" \
             -p use_sim_time:=true
# If /scan resumes -> the tf2 path was the culprit (b). If not -> (a) or (c).
```
```bash
# Turn on the tf2 filter's per-message DEBUG (logger name is literally tf2_ros_message_filter)
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  --log-level tf2_ros_message_filter:=debug --log-level debug \
  -r cloud_in:=/segmentation/obstacle -p use_sim_time:=true
```
Log lines to look for, in order **[V — all strings from `message_filter.hpp`]**:
- `MessageFilter [target=livox_frame ]: Added message in frame livox_frame at time N, count now K`
  → the subscription callback **is** firing; the filter received the message.
- `MessageFilter [target=livox_frame ]: Message ready in frame …` → the filter passed it; any remaining problem is
  in `cloudCallback` ⇒ go to step 3.
- `MessageFilter [target=livox_frame ]: Discarding message in frame … count now K` → (b) confirmed.
- **No `Added message` line at all while `ros2 topic hz /segmentation/obstacle` is non-zero** ⇒ **(a)**, the input
  callback never fires. Then check, in this order:
  1. `ros2 node info /pointcloud_to_laserscan` — is `/segmentation/obstacle` still listed under Subscribers?
     If **not** ⇒ the upstream `sub_.unsubscribe()` path (item 3a) is live on this binary — confirm the patched
     build (`git log -1 --format=%H -- src/perception/rm_perception/pointcloud_to_laserscan`) is actually installed.
  2. `ros2 topic info /segmentation/obstacle --verbose | grep -A4 'Node name: pointcloud_to_laserscan'` —
     Reliability must be `BEST_EFFORT` on both ends. Mismatch ⇒ silent starvation.
  3. `ps -o %cpu,etime,cmd -C pointcloud_to_laserscan_node` — a matched subscription plus ~0 % CPU while 0.2 MB
     × 10 Hz is on the wire is the signature of "frames never delivered" (BEST_EFFORT fragment loss), which is
     **measured in this repo: a RELIABLE subscriber got 2182/2182 = 100 % while BEST_EFFORT subscribers got
     3 %–26 %** (`docs/issues_and_findings.md` #25, `docs/debug_fastlio_cartographer.md` §5.3). Fix candidates:
     raise the publisher reliability, reduce the message size, or accept the drop and use the `local_obstacle:=cloud`
     bypass documented in `docs/smoke_test_runbook.md` L668-678.
  4. Grep the p2l stdout for a swallowed tf2 exception — with `target_frame == livox_frame`, if `livox_frame` is
     absent from the buffer, `Buffer::waitForTransform` takes the `handle == 0` branch and then `lookupTransform()`
     calls `validateFrameId`, which **throws** (`"… passed to lookupTransform argument target_frame does not
     exist"`), aborting `add()` before `cloudCallback`. Look for a `LookupException`/`Error in callback` line in
     p2l's output. **[I, from source]**

**Step 2 — which drop reason?** The drop line is emitted at **INFO** (default visible), throttled to once per
2500 ms, on the **node's own logger**, so it must appear as `/pointcloud_to_laserscan`, not as `tf2_ros`:

```bash
ros2 launch ... 2>&1 | grep "Message Filter dropping message.*livox_frame"
```
Reason → cause mapping **[V]**:
- `…for reason 'the timestamp on the message is earlier than all the data in the transform cache'` → stamp is
  > 10 s (`BUFFER_CORE_DEFAULT_CACHE_TIME`) older than the newest common transform; or the buffer was just cleared by
  a time jump. **But note:** with `target_frame == livox_frame` this branch is unreachable, so seeing it from
  `/pointcloud_to_laserscan` would prove the runtime params differ from the file — confirm with
  `ros2 param get /pointcloud_to_laserscan target_frame`.
- `…for reason 'the frame id of the message is empty'` → the cloud's `header.frame_id` is empty ⇒ permanent.
  Check `ros2 topic echo /segmentation/obstacle --once --field header`.
- `…for reason 'discarding message because the queue is full'` → consumer slower than 10 Hz or a stuck queue.
- `…for reason 'did not find a valid transform, this usually happens at startup ...'` / `'unknown'` → tree problem.

**Step 3 — (c) publishes but output is empty.** Only meaningful if step 1 showed `Message ready`. **[V for the
code path: node.cpp L167-171 fills `ranges` with `inf` (or `range_max+inf_epsilon`), and every rejection path
(L190-229) is a `continue`, so a fully-rejected cloud still publishes a full-length all-`inf` scan.]**

```bash
ros2 topic echo /scan --once --field ranges > /tmp/r.txt
python3 - <<'EOF'
v=[l.split('-',1)[1].strip() for l in open('/tmp/r.txt') if l.strip().startswith('-')]
n=len(v); inf=sum(1 for x in v if 'inf' in x); num=sum(1 for x in v if x[:1].isdigit())
print(f"bins={n} inf={inf} numeric={num}")
EOF
```
- `bins` ≈ 1461 (`(3.14159-(-3.14159))/0.0043`) and `numeric == 0` ⇒ every point was rejected. Then enable the
  rejection DEBUG lines (they are `RCLCPP_DEBUG`, one per rejected point, node.cpp L191/L199/L208/L215/L224):
  `--log-level debug` and grep for `rejected for height` / `rejected for range` / `rejected for angle`. Whichever
  dominates names the culprit parameter (`min_height/max_height` vs `range_min/range_max` vs `angle_min/max`), and
  `--once --field header` tells you the frame.
- **Important discriminator [I, from source]:** an all-`inf` `/scan` still *publishes*, so nav2's
  "observation buffer has not been updated" warning would **not** appear (with `inf_is_valid: true` those bins are
  treated as 10 m clears). Since your symptom includes that warning, (c) is unlikely to be your failure — the node
  is probably not publishing at all.

**Step 4 — if p2l is fine, the warning belongs to the consumer.** The `Message Filter dropping message` you quoted
came from AMCL, whose filter target is `odom`/`map`, not `livox_frame` — so it is a *different* filter with a
*different* frame chain, and it fails for the item-1 reasons even when p2l is healthy:

```bash
ros2 param get /amcl use_sim_time
ros2 run tf2_ros tf2_echo odom livox_frame      # must succeed continuously
ros2 topic echo /scan --once --field header     # stamp must track /clock
ros2 run tf2_ros tf2_monitor                    # per-frame buffer length / most-recent transform
```

---

## 7. Ranked hypotheses for the reported symptom (for the parent)

| # | Hypothesis | Confidence | One-line discriminator | Fix |
|---|---|---|---|---|
| 1 | **Writer-side/transport stall of the whole chain (the repo's own documented 180 s failure)**: slow consumer + RELIABLE writer blocks Gazebo's sensor callback, or BEST_EFFORT fragment loss starves p2l | High — reproduced and measured in this repo (`#25`, §5.3, the 2026-09-23 comment in `ground_segmentation_node.cc`) | `ros2 topic hz` on `/livox/lidar/pointcloud`, `/segmentation/obstacle`, `/scan` simultaneously | keep publisher BEST_EFFORT + keep the message small; or the `local_obstacle:=cloud` bypass |
| 2 | **p2l's input callback genuinely stops** (subscription/transport, not the filter) | Medium | tf2 DEBUG shows **no** `Added message` while the topic has rate; `ros2 node info` sub state | patched build (commit `30ab296`); explicit `queue_size`; verify QoS both ends |
| 3 | **tf2/MessageFilter / clock-domain problem** | **Low for p2l in this config** — the filter is a no-op when `target_frame == cloud frame`; still relevant for AMCL/costmap | `-p target_frame:=""` A/B; `ros2 param get /pointcloud_to_laserscan target_frame` | `target_frame: ""`; add `use_sim_time:=true` to p2l and linefit |
| 4 | Publishes all-`inf` scans | Low | the step-3 bin census | fix `min_height/max_height`/`range_min` |

Two configuration defects are worth fixing regardless of which hypothesis wins: **missing `use_sim_time` on both
`pointcloud_to_laserscan` and `linefit_ground_segmentation`**, and **`target_frame: livox_frame` where
`target_frame: ""` is both equivalent and strictly safer**.


---


## 来源 15/19：`research/build_optimization.md`


> 2026-10-05 · 本文对应改动：新增仓库根目录 `colcon_defaults.yaml`（**唯一**代码/配置改动，
> 没有改任何包的 CMakeLists）。
> 审计范围：`build/*/CMakeCache.txt` + `build/*/CMakeFiles/*/flags.make` 全量 23 个包，
> 外加 `install/`、`colcon`/`colcon-defaults`/`colcon-ros`/`ament_cmake` 的**源码级**取证。
> 隔离口径：所有实测都在**单节点**下做（`ROS_DOMAIN_ID=88`、`ROS_LOG_DIR` 落在工作区内），
> **没有**启 Gazebo / nav2 / LIO。

---

## 0. TL;DR

1. **现象确认（但范围比最初报告的小）**：本工作区 `colcon build` 不传 `CMAKE_BUILD_TYPE`
   ⇒ `CMakeCache.txt` 里为空 ⇒ **6 个包**的编译命令里**一个 `-O` 都没有**（= `-O0`）。
2. **重要更正**：**不是所有"热"包都没优化**。审计发现 9 个包靠**包内硬编码**已经拿到了 `-O3`
   （`add_compile_options(-O3)` / `set(CMAKE_CXX_FLAGS "... -O3")` /
   `set(CMAKE_BUILD_TYPE Release)`）。其中 `set(CMAKE_BUILD_TYPE Release)` 那 5 个包
   **`CMakeCache.txt` 里依然是空的**——只看 cache 会误判成"没优化"。
   *（这正是"一个包的结论不能外推全工作区"的地方。）*
3. 真正在 `-O0` 且在热路径上的只有 6 个：
   `gicp_registration`、`icp_registration`、`lio_tf_adapter`、`fake_vel_transform`、
   `livox_ros_driver2`、`ros2_livox_simulation`（Gazebo 雷达插件 —— **仿真里常驻**）。
4. **实测（同机、同数据、未改一行业务代码，只换构建口径）**：

   | 台子 | `-O0`（改动前） | `Release`（改动后） | 倍数 | fitness / 位姿 |
   |---|---|---|---|---|
   | 台架 `gicp_backend_bench`，PCL GICP | median **369.9 ms** | median **11.3 ms** | **32.7×** | 0.00122 m² → 0.00122 m²（一致） |
   | 台架 `gicp_backend_bench`，small_gicp t=4 | median **254.1 ms** | median **2.3 ms** | **110×** | 0.00122 m²（一致） |
   | 真节点 `gicp_registration_node`，PCL | align **480~496 ms** | align **10.7~10.9 ms** | ≈45× | `0.00123 m²` / `x=-0.150 y=0.150 yaw=-0.01°` 逐字一致 |
   | 真节点，small_gicp | align **236~270 ms** | align **2.3~3.1 ms** | ≈90× | `0.00123 m²` / `x=-0.149 y=0.151` 一致 |

5. **落地方式**：仓库根目录提交 `colcon_defaults.yaml`（`build.cmake-args: [-DCMAKE_BUILD_TYPE=Release]`），
   于是本仓库文档里惯用的 `colcon build --symlink-install` **自动**带上 Release，不必再靠"记得加参数"。
6. **全量重编验证**：`colcon build --symlink-install` → **22/22 包成功、0 失败、0 编译器告警、27.7 s**
   （大多数包旗标不变 ⇒ 不重编，只有旗标变化的包真正重编）。

---

## 1. 审计

### 1.1 审计口径（两个必须先说清的坑，否则结论是错的）

**坑 ①：`CMakeCache.txt` 里 `CMAKE_BUILD_TYPE` 为空 ≠ 没有优化。**

反例 `FAST_LIO`（`src/localization/rm_localization/FAST_LIO/CMakeLists.txt:4-6`）：

```cmake
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # ← 普通变量，**不写 cache**
endif()
```

- `build/fast_lio/CMakeCache.txt` 里仍然是 `CMAKE_BUILD_TYPE:STRING=`（空）；
- 但 CMake 生成器用的是**普通变量**，于是 `build/fast_lio/CMakeFiles/fastlio_mapping.dir/flags.make` 里
  真实旗标是 `... -fexceptions -fopenmp -O3 -DNDEBUG ...`（`-O3 -DNDEBUG` 正是 CMake 对 GNU 的
  `CMAKE_CXX_FLAGS_RELEASE` 默认值）。

⇒ **唯一可信的口径是 `flags.make`**，本文所有"有没有 `-O`"都以它为准：

```bash
grep -oh -- '-O[0-9s]' build/<pkg>/CMakeFiles/*/flags.make | sort -u
```

**坑 ②：`-O3` 不止一种来源。** 本仓库里三种都有：

| 来源 | 例子 | cache 里 BT |
|---|---|---|
| `add_compile_options(-std=c++17 -O3)` | `pointcloud_to_laserscan`、`imu_complementary_filter` | 空 |
| `add_definitions(-std=c++17 -O3)` | `linefit_ground_segmentation{,_ros}` | 空 |
| `set(CMAKE_CXX_FLAGS "-std=c++14 -O3")` | `FAST_LIO`、`point_lio` | 空 |
| `set(CMAKE_BUILD_TYPE Release)`（非 cache） | `slam_toolbox`、`teb_local_planner`、`costmap_converter` | 空 |
| `-DCMAKE_BUILD_TYPE=Release`（cache） | `cartographer`、`cartographer_ros` | `Release` |
| **什么都没有** | 见 §1.3 六个包 | 空 |

> 附带结论：`set(CMAKE_CXX_FLAGS "... -O3")` 这种写法在加了 Release 之后**不会降级**——
> 编译行是 `CMAKE_CXX_FLAGS` + `CMAKE_CXX_FLAGS_RELEASE` 拼接，两者都是 `-O3`，
> 后出现的同优先级旗标取值相同（实测 `fast_lio` 重配后旗标逐字未变、未重编，见 §5）。

### 1.2 审计总表（改动前 → 改动后，23 个包全量）

`-O` 列 = `flags.make` 里出现过的优化旗标（`NONE` = 一个都没有 ⇒ `-O0`）；
`SRC` = `CMakeCache.txt` 的 `CMAKE_HOME_DIRECTORY`（colcon 实际用的源码目录，全部在 `src/**`；
`third_party/**` 有 `COLCON_IGNORE`，colcon 不构建它们，只作参考代码）。

| 包 | BT(cache) 前→后 | `-O` 前→后 | `-DNDEBUG` 前→后 | SRC | 归类 |
|---|---|---|---|---|---|
| `cartographer` | `Release`→`Release` | `-O3`→`-O3` | yes→yes | `src/localization/rm_localization/cartographer` | 建图槽（本来就好） |
| `cartographer_ros` | `Release`→`Release` | `-O3`→`-O3` | yes→yes | `src/localization/rm_localization/cartographer_ros` | 建图槽（本来就好） |
| **`gicp_registration`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/localization/rm_localization/gicp_registration` | **热：GICP 配准（localization:=gicp）** |
| **`icp_registration`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/localization/rm_localization/icp_registration` | **热：ICP 配准（localization:=icp）** |
| **`ros2_livox_simulation`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/simulation/rm_simulation/livox_laser_simulation_RO2` | **热：Gazebo 雷达插件（仿真常驻，出 30k 点/帧）** |
| **`livox_ros_driver2`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/driver/rm_driver/livox_ros_driver2/src` | **热：真机雷达驱动 + `CustomMsg` 序列化** |
| **`fake_vel_transform`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/nav/rm_navigation/fake_vel_transform` | 常驻（TF/速度变换，计算量小；带 `-Werror`） |
| **`lio_tf_adapter`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/localization/rm_localization/lio_tf_adapter` | 常驻（TF 适配，计算量小） |
| `fast_lio` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/localization/rm_localization/FAST_LIO` | 热（LIO 槽）但**本来就 -O3** |
| `point_lio` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/localization/rm_localization/point_lio` | 同上 |
| `linefit_ground_segmentation` | ``→`Release` | `-O3`→`-O3` | NO→**yes** | `src/perception/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation` | 热（地面分割）但本来就 `-O3` |
| `linefit_ground_segmentation_ros` | ``→`Release` | `-O3`→`-O3` | NO→**yes** | 同上 `.../linefit_ground_segmentation_ros` | 同上 |
| `pointcloud_to_laserscan` | ``→`Release` | `-O3`→`-O3` | NO→**yes** | `src/perception/rm_perception/pointcloud_to_laserscan` | 热但本来就 `-O3` |
| `imu_complementary_filter` | ``→`Release` | `-O3`→`-O3` | NO→**yes** | `src/perception/rm_perception/imu_complementary_filter` | 热但本来就 `-O3` |
| `slam_toolbox` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/localization/rm_localization/slam_toolbox` | 建图槽（本来就 Release） |
| `teb_local_planner` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/nav/rm_navigation/teb_local_planner/teb_local_planner` | 规划（本来就 Release） |
| `costmap_converter` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/nav/rm_navigation/costmap_converter/costmap_converter` | 同上 |
| `teb_msgs` | ``→`Release` | `NONE`→**`-O3`** | NO→**yes** | `src/nav/rm_navigation/teb_local_planner/teb_msgs` | 消息序列化代码（30 Hz 控制环话题） |
| `costmap_converter_msgs` | ``→`Release` | `NONE`→**`-O3`** | NO→**yes** | `src/nav/rm_navigation/costmap_converter/costmap_converter_msgs` | 消息序列化代码 |
| `rm_nav_bringup` | ``→`Release` | `NONE`→`NONE` | NO→NO | `src/bringup/rm_nav_bringup` | **纯安装包**（只有 launch/params，无编译目标）⇒ 无意义 |
| `rm_navigation` | ``→`Release` | `NONE`→`NONE` | NO→NO | `src/nav/rm_navigation/rm_navigation` | 纯 Python/params ⇒ 无意义 |
| `hzmi_rm_simulation` | ``→`Release` | `NONE`→`NONE` | NO→NO | `src/simulation/rm_simulation/hzmi_rm_simulation` | 纯 URDF/world ⇒ 无意义 |
| `cartographer_ros_msgs` | ``→``（未动） | `NONE` | NO | `src/localization/rm_localization/cartographer_ros/cartographer_ros_msgs`（**已不存在**） | ⚠️ 陈旧残留，见 §1.5 |

**不在上表里的东西：**

- **nav2 全家（`nav2_controller` / `nav2_planner` / `nav2_bt_navigator` / `nav2_costmap_2d` …）来自 apt**：
  `ros2 pkg prefix nav2_controller` = `/opt/ros/humble` ⇒ 官方 Release 预编译，**不是**本次问题的一部分
  （`third_party/nav2/**` 只是参考代码，`COLCON_IGNORE` 不构建）。
- `cartographer` 是 C++ 库（工作区自编）；`cartographer_ros` 是 `ros.ament_cmake`，两者 cache 里本来就是 `Release`。

### 1.3 "没有 `-O` 且在热路径上"的判定

热路径的判定依据是本仓库的实际装配（`src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py`）：

| 节点 | 何时在跑 | 改动前的构建口径 |
|---|---|---|
| `ros2_livox_simulation`（Gazebo 插件 `libros2_livox.so`，由 `urdf/mid360.xacro:58` 加载） | **仿真常驻** | **`-O0`** ⇒ 已修 |
| `fake_vel_transform`（:508） | **常驻** | **`-O0`** ⇒ 已修 |
| `lio_tf_adapter`（:524，`lio!=none/cartographer` 时） | **常驻** | **`-O0`** ⇒ 已修 |
| `imu_complementary_filter`(:255) / `linefit`(:267) / `pointcloud_to_laserscan`(:277) | 常驻（感知） | 已经 `-O3`（包内硬编码） |
| `fast_lio`(:295) / `point_lio`(:326) | 按 `lio:=` 选一个 | 已经 `-O3` |
| `slam_toolbox`(:365) | 按 `slam:=` | 已经 Release |
| `icp_registration`(:454) / `gicp_registration`(:473) | 按 `localization:=icp` / `:=gicp` | **`-O0`** ⇒ 已修 |
| `livox_ros_driver2` | 真机（仿真里由插件替代） | **`-O0`** ⇒ 已修 |
| nav2 全家 | 常驻 | apt 预编译 Release |

⇒ **仿真里"常驻且原来 `-O0`"的只有 3 个**：Gazebo 雷达插件、`fake_vel_transform`、`lio_tf_adapter`；
再加按槽启用的 `gicp`/`icp`。这也解释了为什么这条线索以前没被发现：**最常被怀疑的 LIO/感知包本来就是 `-O3`**。

### 1.4 `-O` 到底从哪来（源码级证据）

```
$ grep -rn --include=CMakeLists.txt -E '\-O[0-9s]\b|CMAKE_BUILD_TYPE' src/ | sed 's|/CMakeLists.txt:|: |'
src/perception/rm_perception/pointcloud_to_laserscan: 4:add_compile_options(-std=c++17 -O3)
src/perception/rm_perception/imu_complementary_filter: 7:add_compile_options(-std=c++17 -O3)
src/perception/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation: 6:add_definitions(-std=c++17 -O3)
src/perception/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros: 8:add_definitions(-std=c++17 -O3)
src/nav/rm_navigation/teb_local_planner/teb_local_planner: 5:set(CMAKE_BUILD_TYPE Release)
src/nav/rm_navigation/costmap_converter/costmap_converter: 5:set(CMAKE_BUILD_TYPE Release)
src/localization/rm_localization/point_lio: 4:if(NOT CMAKE_BUILD_TYPE) / 5:set(CMAKE_BUILD_TYPE Release) / 10:set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
src/localization/rm_localization/FAST_LIO: 4:if(NOT CMAKE_BUILD_TYPE) / 5:set(CMAKE_BUILD_TYPE Release) / 10:set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
src/localization/rm_localization/slam_toolbox: 4:set(CMAKE_BUILD_TYPE Release)
src/localization/rm_localization/slam_toolbox/lib/karto_sdk: 7:set(CMAKE_BUILD_TYPE Release)
src/localization/rm_localization/FAST_LIO/include/ikd-Tree: 6:set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3")
```

### 1.5 顺手发现的两个"脏"点（**未**在本次改动里处理）

1. **`cartographer_ros_msgs` 是陈旧残留**：`build/`、`install/` 里都有它（2026-09-09 的产物，
   `-O0`），但源码目录 `src/localization/rm_localization/cartographer_ros/cartographer_ros_msgs` **已经不存在**
   （`colcon list` 里也没有这个名字）⇒ **无法重编**，也不该重编：
   真正的包由 **apt 提供**（`/opt/ros/humble/share/cartographer_ros_msgs`，Release）。
   它的 `install/cartographer_ros_msgs/lib/` 是**空的**（`ls` 无输出）⇒ 运行时 `.so` 走 apt，
   **不构成性能或正确性风险**；只是 `install/` 里多一份旧 metadata。
   可选清理（**本仓库不自动做**，需要用户拍板）：
   ```bash
   rm -rf build/cartographer_ros_msgs install/cartographer_ros_msgs
   ```
2. `install/` 里有一套 `--symlink-install` 的符号链接（可执行文件指向 `build/**`），
   这解释了为什么"重编即生效"（不需要重新 `--symlink-install` 一次）。

---

## 2. 实测（同机 A/B，未改一行业务代码）

### 2.1 测量口径

* **机器**：i7-14700HX（28 线程），Ubuntu 22.04 + ROS 2 Humble，PCL 1.12.1，cmake 3.22.1。
* **隔离**：只有**单个节点**在跑。`ROS_DOMAIN_ID=88`（非默认域），`ROS_LOG_DIR` 落在
  `.tmp_cache/build_opt/roslog_*`（工作区内）。**没有** Gazebo / nav2 / LIO / RViz。
* **数据/参数**：与 `docs/gicp_backend_small_gicp.md` §4.0 完全同一口径
  （`src/bringup/rm_nav_bringup/PCD/RMUL2026.pcd` → target 12450 点 @leaf 0.10；合成扫描 4000 点 →
  source 3701/3710 点 @leaf 0.05；注入 `(+0.15, −0.15, 0) m` 偏移；`max_corr 1.5`、`iters 16`）。
* **A/B 之间只差一个变量**：`-DCMAKE_BUILD_TYPE=Release`（业务代码 = 同一个 commit）。
* **负载（诚实披露）**：改动前的台架 run 在 `load≈2.1~2.2`，改动后在 `load≈6.2`（**更高**）。
  ⇒ 下面的加速比**不是**"后测时机器更闲"造成的，反而偏保守。
* **复现命令**：
  ```bash
  source /opt/ros/humble/setup.bash && source install/setup.bash
  install/gicp_registration/lib/gicp_registration/gicp_backend_bench \
    --pcd src/bringup/rm_nav_bringup/PCD/RMUL2026.pcd --backend both --frames 12
  # 真节点：
  bash .tmp_cache/gicp_ab/ab_run.sh <TAG> 14 <pcl|small_gicp> tfon \
    -p initial_pose:="[-0.15,0.15,0.0,0.0,0.0,0.0]"     # 注意：必须全写成浮点，见 §8.4
  ```

### 2.2 台架 `gicp_backend_bench`（12 帧，同一批帧）

| 后端 | 首帧 | 稳态 median | mean | p95 | max | fitness(median) | 最终位姿误差 |
|---|---|---|---|---|---|---|---|
| `pcl` **-O0** | 1427.7 ms | **369.9 ms** | 429.0 | 714.2 | 714.2 | 0.00122 m² | 0.0023 m / 0.028° |
| `pcl` **Release** | 37.5 ms | **11.3 ms** | 11.6 | 14.0 | 14.0 | 0.00122 m² | 0.0023 m / 0.028° |
| `small_gicp` t=4 **-O0** | 253.7 ms | **254.1 ms** | 237.8 | 278.2 | 278.2 | 0.00122 m² | 0.0023 m / 0.027° |
| `small_gicp` t=4 **Release** | 2.4 ms | **2.3 ms** | 2.2 | 2.4 | 2.4 | 0.00122 m² | 0.0023 m / 0.027° |

倍数：**PCL 32.7×**、**small_gicp 110×**（稳态 median）。附带：

* PCL 首帧（含目标协方差预计算）1427.7 → 37.5 ms（**38×**）；
* small_gicp 目标侧一次性预处理（转点云+建 KdTree+估协方差，target 12450 点）
  **287.9 → 5.3 ms（54×）**；
* **坏初值容忍度不变**：`+0.5 m/+10°` PCL 2744.9 → 27.1 ms（最终误差 5 mm/0.06° 不变）；
  `+1.0 m/+20°` PCL 5278.0 → 37.4 ms（8 mm/0.08° 不变）；
  small_gicp `+1.0 m/+20°` 631.0 → 4.3 ms（3 mm/0.04° 不变）。

### 2.3 真节点 `gicp_registration_node`（node-alone，14 s，探针 10 Hz 发合成点云）

| 后端 / 配置 | `[status] align` | `/tf` Hz | `~/pose` 条数（收到 127 帧点云） | `~/fitness_score` | `~/converged` true | 最终 map→odom |
|---|---|---|---|---|---|---|
| `pcl` **-O0** | **480.2 / 495.5 / 483.4 ms** | 50.001 | **30** | 0.00123 m² | 30/30 = 100% | `x=-0.150 y=0.150 yaw=-0.01°` |
| `pcl` **Release** | **10.7 / 10.9 / 10.9 ms** | 49.999 | **127** | 0.00123 m² | 127/127 = 100% | `x=-0.150 y=0.150 yaw=-0.01°`（逐字一致） |
| `small_gicp` **-O0** | **270.4 / 235.9 / 261.6 ms** | 49.997 | **53** | 0.00123 m² | 53/53 = 100% | `x=-0.149 y=0.151 yaw=-0.01°` |
| `small_gicp` **Release** | **3.1 / 2.3 / 3.1 ms** | 49.999 | **127** | 0.00123 m² | 113/127 = 89%↓ | `x=-0.149 y=0.151 yaw=-0.01°`（一致） |
| `small_gicp` Release（**复跑对照**） | 3.3 / 4.0 / 4.0 ms | 50.000 | 127 | 0.00123 m² | 117/127 = 92% | 同上 |

**三条结论：**

1. **align 快 45×（PCL）/ 90×（small_gicp），而 `/tf` 频率完全没变（50.00 Hz）**——
   这与 `docs/gicp_backend_small_gicp.md` §4.2 的判断一致：`/tf` 由**定时器**驱动，
   与 align 耗时解耦；所以**不能用 `/tf` Hz 衡量本次收益**，要看 align 与 CPU。
2. **fitness / 采纳判据 / 最终 TF 数值逐字一致** ⇒ 这是纯粹的性能改动，不是行为改动。
3. **次要发现（原来没人量过）**：`-O0` 时 align ≈ 400 ms 而点云 10 Hz（周期 100 ms）
   ⇒ **节点只能吃下 30/127 = 24% 的点云**（其余在回调组里排队后被 `KEEP_LAST` 丢掉）；
   Release 后变成 **127/127 = 100%**。也就是说 `-O0` 不只是"慢"，它还**静默改变了数据流**
   （定位更新率 2.1 Hz → 9.1 Hz）。这会让"GICP 定位看起来还行"的现场结论变得不可信。

### 2.4 small_gicp 的 `~/converged` 抖动**不是**本次改动引入的（控制实验）

现象：Release 下 `~/converged` 从 100% 掉到 89%（113/127）。做了两个控制实验：

1. **同一个 Release 二进制、同一批帧、重跑台架** ⇒ 上一轮 `converged=0` 的 frame 7/8
   这一轮变成 `converged=1`，median 仍是 2.3 ms，`converged 12/12`；
2. **同一个 Release 节点重跑** ⇒ `~/converged` 117/127 = 92%（第一次 113/127 = 89%）。

⇒ 这是 **run-to-run 抖动**（small_gicp 的 LM 优化器在极小点附近提前 break，
OpenMP 归约顺序影响最后几个 bit），**早在 `-O0` 就有**：本仓库
`docs/gicp_backend_small_gicp.md` §6.2 记录 `-O0` 时节点里就是 **92~96%**，
`src/localization/rm_localization/gicp_registration/src/registration_backend.cpp:190-198` 也把这条行为差异写在注释里
（换 `GaussNewtonOptimizer` 可让 `~/converged` 100%）。
**位姿、fitness、采纳后的 TF 全部一致**（`x=-0.149 y=0.151`）⇒ 只是健康话题的抖动，
不是精度回归。默认后端是 `pcl`（100% converged），完全不受影响。

---

## 3. 策略：为什么用 `colcon_defaults.yaml`（而不是记一条命令）

### 3.1 colcon / ament 到底怎么处理 build type（源码取证）

| 事实 | 证据 |
|---|---|
| **colcon-cmake 不会替你设 build type**，只把你给的 `--cmake-args` 原样转给 cmake（再加 `-DCMAKE_INSTALL_PREFIX`） | `/usr/lib/python3/dist-packages/colcon_cmake/task/cmake/build.py:144-146` |
| colcon-cmake 里唯一和 build type 有关的逻辑 `_get_configuration()` **只对多配置生成器（VS / Ninja Multi-Config）** 有意义，且它只是从 `--cmake-args` 或 cache 里读出来、兜底 `Release` | 同上 `:249-262`（`--config` 只在 `multi_configuration_generator` 分支用，`:238-239`） |
| **ament_cmake 也没有任何 build type 默认值** | `grep -rn CMAKE_BUILD_TYPE /opt/ros/humble/share/ament_cmake*/cmake/` → **无输出** |
| ⇒ 不写配置时 cache 里就是 `CMAKE_BUILD_TYPE:STRING=`（空），`flags.make` 里没有任何 `-O` | 本文 §1.2 审计 |
| **colcon-defaults 插件**：全局文件 = `$COLCON_DEFAULTS_FILE`，未设则 `$COLCON_HOME/defaults.yaml`；**工作区文件 = 当前目录的 `colcon_defaults.yaml`**，深度合并时**工作区覆盖全局** | `colcon_defaults/argument_parser/defaults.py:28-34`（`WORKSPACE_DEFAULTS_FILE='colcon_defaults.yaml'`）、`:78-80`（全局路径）、`:141-146`（`_deep_update(data, local_data)`，local 在后 = 优先）、`:144`（`Path('colcon_defaults.yaml')` = **相对 CWD**） |
| `--symlink-install` 是**追加**（不是替换）cmake 参数 ⇒ 本仓库惯用命令仍吃默认值 | `colcon_ros/task/ament_cmake/build.py:50-53`：`args.cmake_args.append('-DAMENT_CMAKE_SYMLINK_INSTALL=1')` |
| colcon 只在 `--cmake-args` **与上次不同**时才重跑 cmake ⇒ `build/<pkg>/cmake_args.last` 可以直接验证"默认值被吃到了" | `colcon_cmake/task/cmake/build.py:134-138` |

### 3.2 三个选项对比

| 方案 | 可复现性 | 评价 |
|---|---|---|
| 每次手敲 `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` | ❌ 靠记忆 | **就是本次问题的成因**（有人漏敲 ⇒ 静默 -O0） |
| 全局 `~/.colcon/defaults.yaml` 或 `COLCON_DEFAULTS_FILE` | ⚠️ 机器级、仓库外 | 对"克隆即可构建"不可复现；换机器/换人即失效 |
| **仓库根 `colcon_defaults.yaml`（本次采用）** | ✅ 随 git 提交 | 与"克隆即可构建"一致；`colcon build --symlink-install` 自动生效；改了有 diff 可 review |

### 3.3 本次提交的内容与"生效"的证据

新增 `colcon_defaults.yaml`（仓库根，唯一配置改动）：

```yaml
build:
  cmake-args:
    - -DCMAKE_BUILD_TYPE=Release
```

生效证据（构建后立刻可查）：

```bash
$ cat build/lio_tf_adapter/cmake_args.last
['-DCMAKE_BUILD_TYPE=Release', '-DAMENT_CMAKE_SYMLINK_INSTALL=1']
$ grep -m1 '^CMAKE_BUILD_TYPE:' build/lio_tf_adapter/CMakeCache.txt
CMAKE_BUILD_TYPE:STRING=Release
$ grep CXX_FLAGS build/lio_tf_adapter/CMakeFiles/*/flags.make
CXX_FLAGS = -O3 -DNDEBUG -Wall -Wextra -Wpedantic -std=gnu++17
```

> 注意第二个参数：它来自 `colcon build --symlink-install`（colcon_ros 追加）——
> 这同时证明了**文档里惯用的那条命令确实吃到了本文件的默认值**。

`-O3 -DNDEBUG` 里的 `-O3` 是 CMake 对 GNU 的 Release 默认值，可在本工作区直接核对：
`grep CMAKE_CXX_FLAGS_RELEASE build/cartographer/CMakeCache.txt` → `-O3 -DNDEBUG`。

### 3.4 使用注意（三条坑，都已写进 `colcon_defaults.yaml` 的注释）

1. **必须在仓库根目录执行 `colcon build`**（`defaults.py:144` 是相对 CWD 的路径）
   ⇒ 在子目录里 build 会**静默**退回 `-O0`。
2. **命令行显式 `--cmake-args ...` 会整体替换默认值**（argparse 语义）
   ⇒ 要加别的 `-D` 时，请自己把 `-DCMAKE_BUILD_TYPE=Release` 一起写上。
   （`--symlink-install` 不在此列：它是**追加**。）
3. **换口径要显式覆盖**，不要改本文件：
   ```bash
   # 要调试符号：
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   # 完全回到旧口径（-O0）做对照实验：
   COLCON_DEFAULTS_FILE=/dev/null colcon build --symlink-install \
     --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=
   ```

---

## 4. `-DNDEBUG` 副作用（assert 被关掉）

`Release` 对 GNU 的默认旗标是 `-O3 -DNDEBUG` ⇒ **`assert()` 变成空语句**。
逐包查了会**新**吃到 `-DNDEBUG` 的包（改动前没有 `-DNDEBUG` 的包）：

| 包 | `assert(` 出现次数 | 结论 |
|---|---|---|
| `gicp_registration` | 0 | 无影响 |
| `icp_registration` | 0 | 无影响 |
| `lio_tf_adapter` | 0 | 无影响 |
| `fake_vel_transform` | 0 | 无影响 |
| `ros2_livox_simulation` | 0 | 无影响 |
| `linefit_ground_segmentation{,_ros}` | 0 | 无影响 |
| `pointcloud_to_laserscan` | 0 | 无影响 |
| `imu_complementary_filter` | 0 | 无影响 |
| `livox_ros_driver2` | 3 | **全部在 vendored `3rdparty/rapidjson/rapidjson.h`**（`RAPIDJSON_ASSERT`，只用于 JSON 配置解析的内部不变量）；rapidjson 官方就推荐 Release+`NDEBUG`，且**不在每帧热路径上** |
| `teb_msgs` / `costmap_converter_msgs` | 0（生成代码） | 无影响 |

补充说明：

* 工作区**本来就已经是混合口径**：`cartographer`、`cartographer_ros`、`fast_lio`、`point_lio`、
  `slam_toolbox`、`teb_local_planner`、`costmap_converter` 早就在 `-DNDEBUG` 下构建；
  apt 的 nav2 全家（`/opt/ros/humble`）也是 Release。⇒ 本次改动是**消除不一致**，不是引入新风险。
* 本仓库自己的代码里没有依赖 `assert` 做**运行时输入校验**的地方（上表 0 命中）；
  输入校验走的是显式 `if + return/throw`（例如 `icp_registration` 的 `Invalid pcd path` 就是
  `std::runtime_error`，**不是** assert ⇒ `-DNDEBUG` 下依然会拦）。
* 想彻底避开这一条：把 `colcon_defaults.yaml` 换成
  `-DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-O2 -g"`（会保留 assert、
  拿到 `-O2` 与符号，但比 `-O3` 略慢，且**本次实测的 32× 是用 `-O3` 得到的**）。

---

## 5. 逐包 rebuild 结果（时间 / 成功 / 产物大小）

### 5.1 第一轮：只重编"改动前 `-O0`"的包（`colcon build --symlink-install --packages-select ...`）

| 包 | 耗时 | 结果 | 备注 |
|---|---|---|---|
| `costmap_converter_msgs` | 2.7 s | ✅ | 消息包（顺带补 `-O3`） |
| `teb_msgs` | 3.5 s | ✅ | 消息包（顺带补 `-O3`） |
| `lio_tf_adapter` | 6.5 s | ✅ | 另一次单独构建（用来先验证 `colcon_defaults.yaml` 生效） |
| `livox_ros_driver2` | 11.1 s | ✅ | 驱动 + `livox_ros_driver2`/`livox_interfaces2` 序列化代码 |
| `fake_vel_transform` | 12.4 s | ✅ | **带 `-Wall -Werror`，`-O3` 下 0 新告警** |
| `ros2_livox_simulation` | 15.8 s | ✅ | Gazebo 插件 |
| `icp_registration` | 24.3 s | ✅ | |
| `gicp_registration` | 31.9 s | ✅ | 含 vendored small_gicp |
| **合计（并行口径）** | **35.8 s wall / 2m44s user** | ✅ 7/7 | 与上面那 6.5 s 的那次合计 8 包 |

> `stderr` 里出现的三行都是**既有噪音**，不是新告警：
> `ament_auto_package` 的 "headers install destination … Kilted" 提示（4 个包）、
> `fast_lio` 的 CMake `CMP0074/PCL_ROOT` dev 警告。
> **全量日志里 `grep -c 'warning:'` = 0**（含 `-Werror` 包）。

### 5.2 第二轮：全量 `colcon build --symlink-install`（验证用户将执行的那条命令）

```
Summary: 22 packages finished [27.7 s]
  3 packages had stderr output: fast_lio linefit_ground_segmentation pointcloud_to_laserscan   ← 全是既有提示
```

* **22/22 成功、0 失败、0 编译器告警**；
* 为什么这么快：大多数包的旗标**逐字未变** ⇒ make 判定无需重编。
  真正重编的只有旗标变化的那些（`linefit*` 9.5/13.5 s、`pointcloud_to_laserscan` 27.5 s、
  `imu_complementary_filter` 20.1 s —— 它们只是多了 `-DNDEBUG`），
  以及 `fast_lio` 5.8 s / `point_lio` 4.0 s / `slam_toolbox` 7.2 s / `cartographer_ros` 3.9 s（重配+增量）。

### 5.3 产物大小（同一 commit，只换构建口径）

| 产物 | `-O0` | `Release` | Δ |
|---|---|---|---|
| `build/gicp_registration/libgicp_registration.so` | 21 869 656 | 2 104 632 | **−90.4%** |
| `build/gicp_registration/gicp_registration_node` | 204 752 | 57 208 | −72.1% |
| `build/gicp_registration/gicp_backend_bench` | 686 816 | 53 928 | −92.1% |
| `build/gicp_registration/small_gicp/libsmall_gicp.so` | 7 465 208 | 167 856 | −97.8% |
| `build/icp_registration/libicp_registration.so` | 17 281 912 | 1 805 176 | −89.6% |
| `build/lio_tf_adapter/lio_tf_adapter_node` | 3 099 728 | 640 992 | −79.3% |
| `build/fake_vel_transform/libfake_vel_transform.so` | 9 257 024 | 1 509 384 | −83.7% |
| `build/livox_ros_driver2/liblivox_ros_driver2.so` | 3 128 904 | 616 040 | −80.3% |
| `build/livox_ros_driver2/liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so` | 64 024 | 25 368 | −60.4% |
| `build/ros2_livox_simulation/libros2_livox.so` | 3 541 848 | 769 696 | −78.3% |
| `build/teb_msgs/libteb_msgs__rosidl_typesupport_fastrtps_cpp.so` | 108 224 | 37 728 | −65.1% |

（`-O0` 的 `.so` 之所以巨大，是因为没有内联/没有死代码消除 ⇒ 模板实例化全留在二进制里。
体积本身不是目标，但 −80%~−98% 的幅度说明"以前确实一行都没优化"。）

### 5.4 节点级冒烟（重建后的二进制真的能跑）

| 节点 | 方式 | 结果 |
|---|---|---|
| `gicp_registration_node`（pcl / small_gicp） | 真数据 node-alone，14 s ×4 次 | ✅ 见 §2.3 |
| `fake_vel_transform_node` | `ros2 run` + 本包 params，5 s | ✅ `Start FakeVelTransform!` → 收到 SIGINT 干净退出 |
| `lio_tf_adapter_node` | 同上 | ✅ `订阅 /odom -> 广播 TF odom -> base_link` → 干净退出 |
| `icp_registration_node` | 同上 + `-p pcd_path:=…RMUL2026.pcd` | ✅ `pcd point size: 12450` / `icp_registration initialized` → 干净退出 |
| `ros2_livox_simulation`（Gazebo 插件） | **无法单独起**（要 Gazebo 加载） | ⏳ 未验证，见 §7 |

---

## 6. `small_gicp` 的 `FORCE` 与本包 backup/restore 的相容性（**已确认不打架**）

上游 `third_party/small_gicp/CMakeLists.txt:7-9`：

```cmake
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "..." FORCE)   # ← FORCE 写进 cache，全局生效
endif()
```

`gicp_registration` 用 `add_subdirectory` 引入它（为了让 RMW 之外的依赖无需 `sudo make install`），
所以必须防"被第三方 FORCE 掉构建口径"。本包在
`src/localization/rm_localization/gicp_registration/CMakeLists.txt:35-55` 已经做了 backup/restore：

```cmake
set(_gicp_registration_build_type_backup "${CMAKE_BUILD_TYPE}")
add_subdirectory("${SMALL_GICP_VENDOR_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/small_gicp")
set(CMAKE_BUILD_TYPE "${_gicp_registration_build_type_backup}"
  CACHE STRING "Build type（本包已还原成进入 add_subdirectory 之前的值；见上方注释）" FORCE)
```

**加了全局 `Release` 之后的行为**：进入时 cache 已经是 `Release` ⇒ `_backup = "Release"`
⇒ small_gicp 的 `if(NOT CMAKE_BUILD_TYPE)` **不再成立**（不触发 FORCE）
⇒ 出栈时还原成 `Release`（同一个值）⇒ **本包与 vendored 库都按 Release 编译，没有任何冲突**。

反过来，这段 backup/restore 的存在保证了：即使将来有人把 `colcon_defaults.yaml` 删掉，
`gicp_registration` 也**不会**被 small_gicp 静默改成 `-O3`（回到 `-O0` 基线），
A/B 仍然是在同一口径下做的。**两者设计目标一致，不打架；本次没有改这个文件。**

---

## 7. 无法验证 / 未验证清单（**请勿当成已验证**）

1. **RTF / `Control loop missed its desired rate of 30 Hz` / 整栈 CPU**：
   本次**没有**启 Gazebo/nav2/LIO（按任务要求只做单节点）。
   ⇒ **"RTF 会从 0.72 涨到多少"完全没有实测数据**。能确定的只是"少算了"：
   `docs/localization_slots.md:689` 记录的 RTF≈0.72 那次，算力构成里明确含 "GICP 10 Hz"，
   而那时 GICP 每帧 400~480 ms（Release 后 10~11 ms）；
   另外仿真常驻的 Gazebo 雷达插件与 `fake_vel_transform`/`lio_tf_adapter` 也从 `-O0` 变 `-O3`。
2. **`ros2_livox_simulation`（Gazebo 插件）的运行时收益**：无法脱离 Gazebo 测；
   只有"旗标 `-O0`→`-O3`、`libros2_livox.so` 3.54 MB→0.77 MB（−78%）"这两条静态证据。
3. **`livox_ros_driver2` / `CustomMsg` 序列化的收益**：`CustomMsg`（30k 点/帧、10 Hz）的
   fastrtps 序列化代码就在这个包里（`liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so`，
   −60%），但**没有实测吞吐**。真机上才用得到这个驱动（仿真里由插件直接发同类型消息）。
4. **`-DNDEBUG` 下的长时间行为**：只做了静态 assert 审计（§4），**没有**跑长时压力测试。
5. **`icp_registration` 的精度/行为**：只做了"能起来"的冒烟（§5.4），**没有**做 A/B 数值对拍。
6. **`cartographer`/`slam_toolbox` 路径完全没动**（它们本来就是 Release），因此建图相关结论不受影响。
7. **未做**：`teb_local_planner`/`costmap_converter`/`nav2` 的进一步调优（不在本次范围）。

---

## 8. 用户下一步

### 8.1 重编（二选一）

```bash
cd <repo>          # ⚠ 必须在仓库根目录（见 §3.4-①）
source /opt/ros/humble/setup.bash

# ① 推荐：全量（本次已实测 22/22 成功、27.7 s；顺手把消息包与感知包的口径统一）
colcon build --symlink-install

# ② 只重编本次真正变化的包（更快，但消息包仍是 -O0）
colcon build --symlink-install --packages-select \
  gicp_registration icp_registration lio_tf_adapter fake_vel_transform \
  livox_ros_driver2 ros2_livox_simulation teb_msgs costmap_converter_msgs
```

自检（三行，任一不符就是没吃到默认值，见 §3.4）：

```bash
cat build/gicp_registration/cmake_args.last        # 应含 '-DCMAKE_BUILD_TYPE=Release'
grep -m1 '^CMAKE_BUILD_TYPE:' build/gicp_registration/CMakeCache.txt   # =Release
grep -oh -- '-O[0-9s]' build/gicp_registration/CMakeFiles/*/flags.make | sort -u   # 应有 -O3
```

### 8.2 重启整栈后的验收标准

启动命令见 `docs/smoke_test_runbook.md`（`ros2 launch rm_nav_bringup bringup_sim.launch.py …`）。

| 指标 | 期望 | 怎么量 |
|---|---|---|
| **RTF** | 应**高于**改动前的 ≈0.72（**具体多少未知**，见 §7-1） | Gazebo 窗口 / `gz stats`；或用 `ros2 topic hz /clock` 与墙钟对比 |
| **`Control loop missed its desired rate of 30 Hz`** | 出现次数应**明显减少**（不一定归零：`docs/localization_slots.md:689` 认为这条属总算力不足，本次是"减算力"而非"改架构"） | `grep -c 'Control loop missed' <launch>.log`，与改动前的同一场景日志对比 |
| 整机 CPU | 应下降（尤其 `gzserver` 与 `gicp_registration_node`/`icp_registration_node`） | `top -H -p $(pgrep -d, gzserver)`、`ps -o %cpu,cmd -C gicp_registration_node` |
| **GICP 槽（`localization:=gicp`）** | `[status] align=` 从 **~400 ms → ~10 ms**；`采纳 N/M` 里 **N ≈ M**（改前只吃下 ~24% 的点云） | `grep '\[status\]' <launch>.log \| tail` |
| **ICP 槽（`localization:=icp`）** | 没有基线数据，至少不应变差（行为未改，只换旗标） | 同上（该包的 `[status]` 行） |
| **回归（必须做）** | `fitness_score` 仍 ≈`0.0012 m²`、`~/converged` 语义不变、**恰好一条** `map→odom`、`/scan` 与 `/segmentation/obstacle` 频率不低于改前、导航能跑完一条 goal | `docs/smoke_test_runbook.md` 的既有检查项；`ros2 topic hz /tf` 应仍是 50 Hz |
| 小车行为 | 无漂移/无 TF 甩动（本改动不改数学，**不该**出现新的 TF 问题） | RViz / `docs/monitor_map_odom.py` |

**注意**：`/tf` 的 50 Hz **不会**因为本次改动变化（定时器驱动），别用它当验收指标。

### 8.3 回滚（一条命令 / 一个文件）

```bash
# ① 立刻回到 -O0（不改任何文件）：
cd <repo>
COLCON_DEFAULTS_FILE=/dev/null colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=
# ② 或临时用别的口径（保留 assert、带调试符号）：
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
# ③ 彻底回滚本次提交：
git revert <本提交的 SHA>      # 然后 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=
```

> 注意 ①③ 之后**必须显式传 `--cmake-args -DCMAKE_BUILD_TYPE=`**（空值）才能把 cache 里的
> `Release` 抹掉；只删 `colcon_defaults.yaml` 再 `colcon build` 的话，cache 里仍是 `Release`
> （CMake 不会自己清 cache 变量）。

### 8.4 两个踩过的坑（写下来免得再踩）

1. **`-p initial_pose:=[...]` 的列表必须全浮点**：`[-0.15,0.15,0,0,0,0]` 会被 ROS 参数解析器拒
   （`Sequence should be of same type. Value type 'integer' do not belong` → 节点 `abort`，exit 134）。
   写成 `[-0.15,0.15,0.0,0.0,0.0,0.0]`，且**不要带空格**（`ab_run.sh` 里 `$EXTRA` 是不加引号展开的）。
2. `icp_registration` 的 `pcd_path` 默认是空串（`icp_registration_sim.yaml:6`），
   不传就 `std::runtime_error: Invalid pcd path` 直接 abort —— 是**参数**问题，不是构建问题。

---

## 9. 证据文件（都在工作区内，未提交）

| 内容 | 路径 |
|---|---|
| 审计脚本 / 审计表（前、后） | `.tmp_cache/build_opt/audit.sh`、`audit_before.txt`、`audit_after.txt`、`audit_final.txt` |
| 台架输出（前/后 × 两种后端 + 复跑对照） | `.tmp_cache/build_opt/bench_{before,after}_{pcl,sgicp}.txt`、`bench_after_sgicp_rerun.txt` |
| 真节点 A/B 全过程（探针汇总 + `[status]` + `/tf` hz） | `.tmp_cache/build_opt/node/{before,after}_{pcl,sgicp}.txt`、`node/after_sgicp2.txt` |
| 增量重编 / 全量重编日志 | `.tmp_cache/build_opt/rebuild_2026.log`、`rebuild_full.log` |
| 二进制大小/哈希快照（前） | `.tmp_cache/build_opt/bins_before.txt` |
| 冒烟日志（3 个常驻节点） | `.tmp_cache/build_opt/smoke_*.log` |
| 复用的单节点探针台架（**上一轮** `small_gicp` 改动留下的，本次未修改） | `.tmp_cache/gicp_ab/ab_run.sh`、`probe2.py` |

## 10. 相关文档

* `docs/gicp_backend_small_gicp.md` §4（原有 A/B 口径）、§6.2（`~/converged` 抖动，`-O0` 时代就有）
* `docs/localization_slots.md:689`（RTF≈0.72 与 `Control loop missed` 的既有结论）
* `docs/issues_and_findings.md` #25（大点云订阅交付）、#19（雷达插件时间基）
* `docs/smoke_test_runbook.md`（重启整栈与回归检查的启动命令）


---


## 来源 16/19：`research/cloud_density_measurements.md`


> **状态**：`现行`
> **日期**：2026-11-11
> **目的**：回答"我们的点云够不够支撑 BIT 那套逐柱语义投影"。
> **结论先说**：`/cloud_registered` **每帧只有 794 点，不够**；`/livox/lidar/pointcloud` **7309 点，够**。

## 1. 为什么必须量这个

BIT 的 `navi_minco_bit` 用 ROGMap 的 `ProjectionLayer` 做地形语义（见
[`archive/rogmap_integration_plan.md``rogmap_integration_plan.md`）。它的判据是**单根 XY 柱内**：

| 类 | 判据（源码 `projection_layer.cpp:47-77`） |
|---|---|
| `PASSABLE` | `height_delta ≤ surface_height_delta_max(0.10)` ⇒ 薄面（地面/坡/杆） |
| `OCCUPIED` | `height_delta ≥ wall_height_delta_min(0.20)` 且 `fill_ratio ≥ wall_ratio_min` ⇒ 实心墙 |
| `PASSABLE`(隧道) | `height_delta ∈ [0.25,0.40]` 且 `fill_ratio ≤ tunnel_ratio_max` ⇒ 中空横杆 |

**三个判据都要求一根柱子里有多层体素**。若点太稀（一柱 0~1 层），`height_delta` 恒为 0
⇒ 所有东西都被判成"薄面" ⇒ **墙消失**。所以点密度是这条路线的前置条件。

## 2. 测量方法（不依赖 Gazebo）

本机 Gazebo 因**无可用 OpenGL/GLX** 起不来（`X_GLXCreateContext BadValue`），
因此改用仓库里已有的真实录制回放，**只跑 bag + 量测脚本，不需要 gzserver**：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=$PWD/.tmp_diag/roslog     # 本机 ~/.ros 只读时必须设

# 终端 A：回放（只放需要的话题，2 倍速）
ros2 bag play bag/raw_20261103 --rate 2.0 \
  --topics /cloud_registered /odom /livox/lidar/pointcloud

# 终端 B：量测
python3 tools/scripts/nav/measure_cloud.py --cloud /cloud_registered --odom /odom --seconds 12
python3 tools/scripts/nav/measure_cloud.py --cloud /livox/lidar/pointcloud --odom /odom --seconds 12
```

数据源：`bag/raw_20261103`，**90.94 s / 17992 条消息**，话题表：

| 话题 | 类型 | 条数 |
|---|---|---:|
| `/livox/lidar/pointcloud` | `PointCloud2` | 432（≈4.75 Hz 录制，回放 2× ⇒ 9.6 Hz） |
| `/cloud_registered` | `PointCloud2` | 3585（≈39 Hz 录制，回放 2× ⇒ 80 Hz） |
| `/odom` | `Odometry` | 3585 |
| `/scan` | `LaserScan` | 432 |
| `/tf` / `/tf_static` | `TFMessage` | 9956 / 2 |

## 3. 实测结果

| 指标 | **`/cloud_registered`**（配准后，nav 用） | **`/livox/lidar/pointcloud`**（雷达原始） |
|---|---|---|
| 频率 | **79.9 Hz** | **9.6 Hz** |
| 每帧点数（中位） | **794** | **7309** |
| 每帧点数（min / max） | 517 / 1319 | 6730 / 9398 |
| `frame_id` | `odom` | `livox_frame` |
| 字段 | `x y z intensity` | `x y z rgb` |
| **逐点时间字段** | ❌ 无 | ❌ 无（仿真 `offset_time=0`） |
| z 范围（全程） | −0.60 ~ +2.81 m | −0.56 ~ +2.73 m |
| 外接框面积（中位） | 187.7 m² | 506.4 m² |

> ⚠ **"密度 pts/m²" 不要当判据**：脚本用的是**外接框面积**，严重高估表面积，
> 算出来 4.2 / 14.4 pts/m² 只是下界。**该看的是"每帧点数"。**

## 4. 判读

| 云 | 每帧点数 | 对投影层的意义 |
|---|---:|---|
| `/cloud_registered` | **794** | 摊在 13×14 m 上 ⇒ 0.05 m 栅格里平均 **0.3 点/格** ⇒ 柱内基本只有 0~1 层 ⇒ **判不出墙** |
| `/livox/lidar/pointcloud` | **7309** | 0.05 m 栅格平均 **2.8 点/格** ⇒ 柱内可多层 ⇒ **判据可用** |

**⇒ `/cloud_registered` 是"给 nav2 costmap 用的降采样版"，不是"给建图用的稠密版"。**
BIT 专门做了一个 `/cloud_registered_full`（逐点去畸变、稠密）正是为了这个缺口。

## 5. 对路线的影响

| 结论 | 影响 |
|---|---|
| 配准云太稀 | **不能**直接把 `/cloud_registered` 喂给 ROGMap 的投影层 |
| 原始云够用 | ROGMap 改吃 `/livox/lidar/pointcloud` + TF（它本来就有 cloud+odom+TF 三件套） |
| 无逐点时间 | 仿真里无法逐点去畸变（`offset_time=0`）；真机 Livox 驱动会提供 `offset_time` |

**下一步（唯一该做的一步）**：把 ROGMap 的点云输入从 `/cloud_registered` 换成
`/livox/lidar/pointcloud`，重开 `use_rog_map: true`，看投影层能否分出
`FREE / PASSABLE / OCCUPIED` 三类。这一步不改算法，只改一个输入话题。

## 6. 复现与坑

| 坑 | 现象 | 处理 |
|---|---|---|
| `~/.ros` 只读 | `Failed opening file .../python3_*.log` 直接崩 | `export ROS_LOG_DIR=$PWD/.tmp_diag/roslog` |
| `/tmp` 每次调用是新的 tmpfs | 日志/中间文件下一轮就丢 | 日志写**工作区**（`.tmp_diag/`），不要写 `/tmp` |
| 测量跑在启动之前 | 脚本报"没收到任何点云" | 先起仿真，等 `localization state -> NORMAL`，再量 |
| 两端 RMW 不一致 | 互相看不见话题 | 两边都 `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| Gazebo 起不来 | `[gzserver] X Error ... GLX` | 用 bag 回放测量，不依赖图形 |

## 7. 更正记录（2026-11-11 晚）：ROGMap 要的是世界系云

曾有一版结论认为"ROGMap 把输入当作传感器系、自己乘位姿"，**该结论错误**。
源码依据（`src/vendor/rog_map/src/vendor/rog_map/prob_map.cpp`，`updateProbMap` → `raycastProcess`）：

```cpp
Vec3f p(pcl_p.x, pcl_p.y, pcl_p.z);      // 点坐标直接用
posToGlobalIndex(p, pt_id_g);            // 直接当世界坐标落格
const double dz = p.z() - cur_odom.z();  // cur_odom 只用于高度裁剪
p = cur_odom + (p - cur_odom).normalized() * ...;   // 只用于范围裁剪
```

⇒ **点云不做任何变换**；`cur_odom` 只决定滑窗中心、高度/范围裁剪、射线起点。
⇒ **ROGMap 的输入必须是 map 系世界点云**（zlio 的 `/localization/lio_world_cloud`），
喂传感器系原始云会让整图错位。

因此 `/livox/lidar/pointcloud`（`livox_frame`，7309 点）**不能**直接喂 ROGMap，
它只能作为"密度参考"，说明原始输入本身是够密的；缺的是**一条稠密的世界系输出**
（BIT 用 `/cloud_registered_full` 补的正是这个缺口）。

## 8. zlio 世界云实测（2026-11-11，仿真）

数据源：`/localization/lio_world_cloud`（zlio backend 发布，`map` 系）。

| 配置 | 每帧点数（中位） | min / max | 频率（消息时间戳） | 备注 |
|---|---:|---|---:|---|
| `output_resolution: 0.10`（上游默认） | **2140** | 670 / 2905 | ~10 Hz | 实测 §3 用的旧值 |
| **`output_resolution: 0.02`** | **3977** | 1378 / 4098 | **10.21 Hz** | 本仓现值（launch 默认） |

> 墙钟频率会因仿真 RTF≈0.29 显示成 2~3 Hz，**判读一律用消息时间戳频率**。

### 抽稀参数是上游 zlio 的，不是本仓加的

出处三证（`git remote -v` = `github.com/mozihe/z-localization`，提交 `76e1263`；
`git status` 显示本仓只改了 `src/frontend/apps/node.cpp` 6 行诊断日志）：

```cpp
// src/frontend/apps/node.cpp:105（上游 main 同样如此）
output_resolution_ = declare_parameter<double>("output_resolution", .10);
// src/frontend/apps/node.cpp:68
prepared.odometry->setOutputResolution(output_resolution_);
```

**作者对"为什么可以稀疏"的原话**（`src/frontend/docs/parameters.md:224`）：

> `output_resolution` … 导出点云的体素分辨率 … **下游需要更多细节时减小，需要降低点云数据量时增大**；设为 `0` 会关闭点云导出

代码注释（`src/frontend/src/preparation.hpp:222`）：

> 设置后端点云采样尺度，**不影响在线滤波及入图采样**。

**⇒ 稀疏只是默认值带来的数据量取舍，作者明确把"调小换细节"写成正常用法。**

### 导出云**已经去畸变**（这点决定它能不能给 ROGMap 用）

`src/frontend/README.md:120` 原文：

> 导出点云：float32 XYZ，**统一到该次观测结束时的 IMU 坐标系**；**不保留原始强度、线号和时间字段**。

即：帧内各时刻的点被补偿到帧末那一个 IMU 姿态 ⇒ **已做运动补偿**；
后端再用 `T_map_sensor` 转到 `map`（`publisher_view.cpp:200` `transformCloud(*frame.cloud_local, ...)`，
**此处不再抽稀**）。⇒ 世界云 = **已去畸变 + 按 `output_resolution` 抽稀**。

> BIT 用 `publish.dense_publish_en` + `/cloud_registered_full` 做的正是同一件事。

### 坑

| 坑 | 现象 |
|---|---|
| `output_resolution: 0` | 文档写"关闭导出"，代码是 `if (resolution > 0)` 才填数组 ⇒ **发布空点云** |
| 用墙钟判频率 | RTF<1 时会低估 3~5 倍（曾误判"云太慢"） |
| 仿真无逐点时间 | gzserver `offset_time=0 (intra-frame motionless)` ⇒ 去畸变在仿真里无实际作用，真机才生效 |


---


## 来源 17/19：`research/cartographer_2d_occupancy_semantics.md`


Prepared for debugging "occupied cells keep getting erased" in this workspace.
Every excerpt below was copied verbatim from the real sources (fetched with web tools /
read from the vendored checkout). Line numbers are those of the quoted revision.

> **2026-10 合并说明**：本文 §1–§4 是逐字源码分析（原样保留）。仓库整理时，把同主题的**未入库**报告
> `cartographer_map_stability_report.md`（仓库根，455 行，2026-09-23，英文）中**本文没有的**证据并入
> **§5 合并附录**（来源标记、issue 调查、TSDF、替代方案、调参表），随后删除该未入库文件；§5.1 列了
> "原文哪些结论与本文重复、因此不再重抄"的对应表。**§1–§4 仍是本仓唯一的逐字源码依据；§5 是外部调研补遗。**

## Provenance (read first — two of the requested paths no longer exist)

| Item | Requested | What actually exists |
|---|---|---|
| 1 | `cartographer/mapping/internal/2d/range_data_inserter_2d.cc` / `.h` | **Renamed**: `cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc` / `.h` (class `ProbabilityGridRangeDataInserter2D`). Verified against `cartographer-project/cartographer` master, tree SHA `877157a0d91788a7700221d87232d412cb3c1ef4` (commit `877157a`). |
| 3 | `sensor_bridge.cc` → `HandleLaserScanMessage`, `ToRangeData`, `ToPointCloudWithIntensities` | `HandleLaserScanMessage` **is** in `sensor_bridge.cpp/.cc`, but the conversion itself is in `msg_conversion.cpp` in the ROS2 tree. **`ToRangeData` does not exist in any current branch** (checked `cartographer_ros` master, `ros2-dashing`, and the ROS2 fork) — it was removed years ago. |
| 4 | `occupancy_grid_node_main.cc` | That file contains **no** int8 conversion; it fetches submap textures and calls `CreateOccupancyGridMsg()` in `msg_conversion.cpp`. The int8 mapping lives there, plus the texture encoding in `cartographer/io/submap_painter.cc` and `cartographer/mapping/2d/probability_grid.cc`. |

**Version verification for this machine.** `dpkg` shows `ros-humble-cartographer 2.0.9004` and
`ros-humble-cartographer-ros 2.0.9002`. For ROS 2, rosdistro does **not** use
`cartographer-project/*` master; it uses:

* core: <https://github.com/ros2/cartographer> branch `ros2` (tag `2.0.9004`)
* ROS glue: <https://github.com/ros2/cartographer_ros> branch `ros2` (tag `2.0.9002`)

I cloned both tags and diffed against `cartographer-project` master / the ROS2 tip:
**all of `probability_grid_range_data_inserter_2d.{cc,h}`, `probability_grid.{h,cc}`,
`probability_values.{h,cc}`, `ray_to_pixel_mask.cc`, `local_trajectory_builder_2d.cc`,
`msg_conversion.cpp`, `sensor_bridge.cpp`, `occupancy_grid_node_main.cpp` are byte-identical**
(empty `diff`). So items 1–4 below are exactly what this workspace runs.

---

## 1. `ProbabilityGridRangeDataInserter2D` — hits, misses, free space

### (a) Verbatim

`cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc` lines 52–96:

```cpp
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space, ProbabilityGrid* probability_grid) {
  GrowAsNeeded(range_data, probability_grid);

  const MapLimits& limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }

  if (!insert_free_space) {
    return;
  }

  // Now add the misses.
  for (const Eigen::Array2i& end : ends) {
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
```

`kSubpixelScale = 1000` (line 30). Same file, lines 124–133:

```cpp
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const {
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
           probability_grid);
  probability_grid->FinishUpdate();
}
```

`cartographer/mapping/2d/probability_grid.cc` lines 58–71 (the write itself):

```cpp
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index);
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
  if (*cell >= kUpdateMarker) {
    return false;
  }
  mutable_update_indices()->push_back(flat_index);
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}
```

`cartographer/mapping/2d/grid_2d.cc` lines 99–106:

```cpp
void Grid2D::FinishUpdate() {
  while (!update_indices_.empty()) {
    DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
              kUpdateMarker);
    correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
    update_indices_.pop_back();
  }
}
```

### (b) Answers

* **Write order, exactly.** `GrowAsNeeded` (grow grid to cover origin + all `returns` + all
  `misses`) → **all hits** (`returns`, each immediately) → **if `insert_free_space` only**:
  the free-space ray of *each return* (origin → that hit's cell) → then the free-space ray of
  each element of `range_data.misses` → `FinishUpdate()` in `Insert()`. Hits are *not*
  interleaved with misses; **every** hit is applied before **any** miss.
* **Which cells get the "miss/free" write for a return.** The ray is
  `RayToPixelMask(begin, end, kSubpixelScale)` with `begin`/`end` expressed in the
  *superscaled* grid (cell size `resolution/1000` = 50 µm for a 5 cm grid); `RayToPixelMask`
  returns every **full 5 cm cell** the straight segment from origin to endpoint passes through,
  and it is **inclusive of both ends** (it pushes `scaled_begin / subpixel_scale` first and
  terminates after appending the cell containing `scaled_end`; see
  `cartographer/mapping/internal/2d/ray_to_pixel_mask.cc` lines 34–156). So the endpoint (hit)
  cell **is visited** by the miss loop — but its write is a no-op: the hit table entry always
  has `kUpdateMarker` set (`*cell = table[*cell]`, and every table entry is `value + kUpdateMarker`),
  so the later `ApplyLookupTable(..., miss_table)` hits `if (*cell >= kUpdateMarker) return false;`.
  Net effect: free space is written to `[origin cell … endpoint cell]` with the endpoint cell
  surviving as a hit. There is no geometric exclusion of the endpoint; the exclusion is the
  single-write-per-cell rule plus hit-first ordering.
* **"Hits have priority" is scoped to one `Insert()` call.** `FinishUpdate()` strips
  `kUpdateMarker` from every touched cell at the end of each `Insert()`. Therefore a cell that
  is a hit in scan *k* **can and will** be decreased by a miss in scan *k+1* (or in any later
  `Insert()` on the same submap). There is no persistence/weighting protection across scans.
  This is the mechanism by which "occupied cells get erased".
* **`range_data.misses` and `missing_data_ray_length`.** `misses` is populated in
  `LocalTrajectoryBuilder2D::AddRangeData`
  (`cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc` lines 163–185):
  every accepted point whose range exceeds `options_.max_range()` is *moved* to
  `origin + missing_data_ray_length / range * delta` and appended to `misses`; everything with
  `min_range <= range <= max_range` goes to `returns`; `range < min_range` is dropped.
  `missing_data_ray_length` therefore controls **only** the length of the "assumed empty"
  segment for beams that returned *beyond `max_range`*. It does **not** affect the free-space
  rays derived from `returns` (those always reach all the way to the hit cell).
* **Can `misses` be non-empty?** Both sensor types end up as `TimedPointCloudData` and go
  through the same code, so the answer is decided by `max_range`, not by the sensor type:
  * **PointCloud2**: yes, routinely — `ToPointCloudWithIntensities(PointCloud2)` applies
    **no range filter at all**, so any XYZ farther than `max_range` becomes a miss.
  * **LaserScan**: only if some beam survives `msg_conversion` with
    `range > options_.max_range()`. For **this workspace it can never happen**: the scan comes
    from `pointcloud_to_laserscan` with `range_max: 10.0`, `use_inf: true`
    (`src/perception/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml`), while
    `TRAJECTORY_BUILDER_2D.max_range = 12.0` (`src/localization/rm_localization/cartographer_ros/configuration_files/cartographer.lua`).
    p2l emits no-echo bins as `inf` (or `range_max + inf_epsilon` if `use_inf: false`), and both
    fail `first_echo <= msg.range_max` in the conversion → dropped before cartographer ever
    sees them. **Consequence: `range_data.misses` is always empty for your `/scan`, and
    `missing_data_ray_length` (0.5 → 0.05) is a no-op in your setup.** The observed clearing is
    caused entirely by the *return-ray* loop above.
* **`insert_free_space`.** `false` returns from `CastRays` before **both** miss loops, so no
  free-space write happens at all — neither from return rays nor from `range_data.misses`.
  It does **not** disable hits. This matches the measured result in your config comments
  (`insert_free_space=false` → 89 % retention but free cells = 0): "erasing old walls" and
  "marking free space" are literally the same write (`miss_table` on cells along a ray), so the
  switch cannot separate them. It is set with `true` as the default when the Lua key is absent
  (`HasKey(...) ? GetBool(...) : true`), and `CHECK_GT(hit_probability, 0.5)`,
  `CHECK_LT(miss_probability, 0.5)` are enforced.
* Extra filters between the sensor and the grid: `TransformToGravityAlignedFrameAndFilter`
  applies `sensor::CropRangeData(..., min_z, max_z)` to **both** returns and misses and then
  `VoxelFilter(voxel_filter_size)` to **both** (`local_trajectory_builder_2d.cc` lines 52–63).
  With `num_accumulated_range_data = 1`, `AddRangeData` inserts one `RangeData` per scan.

---

## 2. `ProbabilityGrid` / `probability_values` — what is stored, and the lookup tables

### (a) Verbatim

`cartographer/mapping/probability_values.h` lines 64–92:

```cpp
constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;
...
constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUnknownCorrespondenceValue = kUnknownProbabilityValue;
constexpr uint16 kUpdateMarker = 1u << 15;
...
// Converts a probability to a uint16 in the [1, 32767] range.
inline uint16 ProbabilityToValue(const float probability) {
  return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}
```

`cartographer/mapping/probability_values.cc` lines 76–104 (verification added: this is the
probability-space twin; `ProbabilityGrid` uses the *correspondence-cost* one below):

```cpp
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}
```

`cartographer/mapping/probability_values.h` lines 32–45 / 48–57 (`BoundedFloatToValue`,
`Odds`, `ProbabilityFromOdds`, `ProbabilityToCorrespondenceCost`);
`cartographer/mapping/2d/probability_grid.cc` lines 27–31 and 39–49:

```cpp
ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) {}
...
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  mutable_known_cells_box()->extend(cell_index.matrix());
}
```

`cartographer/mapping/2d/probability_grid.h` lines 43–46 (comment) and `probability_grid.cc`
lines 77–82:

```cpp
// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. ...
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
...
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!limits().Contains(cell_index)) return kMinProbability;
  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}
```

`cartographer/mapping/2d/grid_2d.h` lines 68–71:

```cpp
  bool IsKnown(const Eigen::Array2i& cell_index) const {
    return limits_.Contains(cell_index) &&
           correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
               kUnknownCorrespondenceValue;
  }
```

### (b) Answers

* **Occupied or free?** The public model is **P(cell is occupied)**. The storage, however, is
  the **correspondence cost = 1 − P(occupied)**, held as a `uint16` in
  `correspondence_cost_cells_`. `GetProbability()` converts back to P(occupied);
  `GetCorrespondenceCost()` is 1 − P(occupied). `SetProbability()` takes P(occupied).
* **Exact numbers:** `kMinProbability = 0.1f`, `kMaxProbability = 0.9f` (`1.f - 0.1f`),
  `kMinCorrespondenceCost = 0.1f`, `kMaxCorrespondenceCost = 0.9f`,
  `kUnknownProbabilityValue = 0`, `kUnknownCorrespondenceValue = 0`,
  `kUpdateMarker = 1u << 15` = **32768**. The uint16 encoding range is `[1, 32767]`
  (`BoundedFloatToValue` clamps to the bound and maps linearly onto 32766 steps, `+1`), and it
  is independent of the update marker: while updating, values carry `+32768`.
  `kValueCount = 32768` table entries.
* **Unknown cells read as 0.1, not 0.5.** `ValueToCorrespondenceCost(0) = kMaxCorrespondenceCost = 0.9`,
  so `GetProbability` of an unknown cell returns `kMinProbability = 0.1`. The real "is unknown"
  test is `IsKnown()` / stored value `== 0`, never a 0.5 comparison. (Only `SetProbability`
  refuses to write to a non-unknown cell.)
* **How a table works.** The inserter builds two tables *once*, in the constructor:
  `hit_table_ = ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(hit_probability))` and
  `miss_table_ = ... Odds(miss_probability)`, where `Odds(p) = p/(1-p)`. Each table has 32768
  `uint16` entries and is indexed by the *current* cell value; every entry already includes
  `kUpdateMarker`, so applying it both updates the probability and stamps the cell as touched.
  * index `0` (unknown): the cell is set to `ProbabilityFromOdds(odds)` — i.e. the **first**
    observation sets P directly to `hit_probability` (or `miss_probability`).
  * index `v >= 1`: `odds_new = odds * (p_v/(1-p_v))`, `P_new = ProbabilityFromOdds(odds_new)`,
    re-encoded via `CorrespondenceCostToValue`, which clamps into `[0.1, 0.9]`.
  * `DCHECK_EQ(table.size(), kUpdateMarker)` documents the 32768 entry count.
* **Saturation.** A repeatedly hit cell converges to **P = kMaxProbability = 0.9** (stored
  uint16 = **1**, i.e. correspondence cost 0.1). A repeatedly missed cell converges to
  **P = kMinProbability = 0.1** (stored uint16 = **32767**, correspondence cost 0.9). The clamp
  happens per update, so the log-odds random walk saturates and never leaves `[0.1, 0.9]`.
* Per-update log-odds increments are `ln(hit_odds)` and `ln(miss_odds)`. With upstream defaults
  (`hit = 0.55`, `miss = 0.49`) that is `+0.2007` vs `−0.0400` (≈5 misses cancel 1 hit); with
  your config (`hit = 0.68`, `miss = 0.40`) it is `+0.7538` vs `−0.4055` (≈1.9 misses cancel
  1 hit). A cell can receive at most **one** update per `Insert()` (the marker blocks the rest),
  so the "votes" per scan are at most one hit *or* one miss per cell.

---

## 3. LaserScan → `RangeData` classification (ROS 2 fork, `ros2/cartographer_ros` tag 2.0.9002)

### (a) Verbatim

`cartographer_ros/src/sensor_bridge.cpp` lines 156–162:

```cpp
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}
```

`cartographer_ros/src/msg_conversion.cpp` lines 117–175 (the actual classification):

```cpp
// For sensor_msgs::msg::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }
...
// For sensor_msgs::msg::LaserScan and sensor_msgs::msg::MultiEchoLaserScan.
template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) {
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
      const float first_echo = GetFirstEcho(echoes);
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const cartographer::sensor::TimedRangefinderPoint point{
            rotation * (first_echo * Eigen::Vector3f::UnitX()),
            i * msg.time_increment};
        point_cloud.points.push_back(point);
        ...
      }
    }
    angle += msg.angle_increment;
  }
  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += cartographer::common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}
```

`cartographer_ros/src/msg_conversion.cpp` lines 204–214 (the `ToPointCloudWithIntensities`
overloads — identical body for LaserScan and MultiEchoLaserScan) and lines 216–218 + 255–298
(PointCloud2 overload: **no range check anywhere**). Then
`cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc` lines 163–185:

```cpp
  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        hit_in_local.position =
            origin_in_local +
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
```

### (b) Answers

There are **two independent filters** with different bounds — this is the usual source of
confusion:

1. message-level (`msg.range_min`/`msg.range_max`, applied in `msg_conversion.cpp`);
2. trajectory-builder-level (`TRAJECTORY_BUILDER_2D.min_range`/`max_range`, applied in
   `local_trajectory_builder_2d.cc`; yours are `0.2` / `12.0`).

* `range_min <= range <= range_max` (message) → kept as a point at
  `(range·cos θ, range·sin θ, 0)`; otherwise **silently skipped** — it becomes neither a hit
  nor a miss (no free-space ray is created).
* **`inf`**: `inf <= msg.range_max` is true only when `msg.range_max` is itself `inf`. With a
  finite `range_max` (yours: 10.0) every `inf` ray is **dropped**. (If a driver *did* publish
  `range_max = inf`, the point would survive as an infinite `range`, then hit
  `range > max_range` → `missing_data_ray_length/range * delta` = `5/inf * inf` = **NaN** —
  a genuine hazard worth guarding against.)
* **NaN**: both `range_min <= NaN` and `NaN <= range_max` are false → **dropped**.
* **`range > range_max` (message)**: dropped. **`range < range_min` (message)**: dropped.
  (Yours: `range_min = 0.2`, `range_max = 10.0`.)
* **tb-level**: `range < min_range` → dropped; `min_range <= range <= max_range` → `returns`
  (hit); `range > max_range` → `misses` with the point placed at **exactly
  `missing_data_ray_length` metres from the origin along the beam direction**
  (`origin + (missing_data_ray_length / range) * delta`, defaults `min_range = 0.`,
  `max_range = 30.`, `missing_data_ray_length = 5.` in
  `configuration_files/trajectory_builder_2d.lua`; yours are `0.2`, `12.`, `0.05`).
  So the miss distance is **`missing_data_ray_length`** — not `range_max`, not the measured range.
* **Consequences for this workspace**: `/scan` comes from `pointcloud_to_laserscan`
  (`range_max: 10.0`, `use_inf: true`, `range_min: 0.2`) and cartographer's
  `max_range = 12.0`. Every no-echo bin is `inf` > 10.0 → dropped at step 1; nothing can exceed
  12.0 → **`range_data.misses` is always empty**, and `missing_data_ray_length` has **no
  effect whatsoever**. Lowering it 0.5 → 0.05 changed nothing observable (as expected). The
  only free-space writes in your maps are the origin→hit rays of step 1 of item 1.
* `HandleLaserScan` additionally splits the cloud into `num_subdivisions_per_laser_scan`
  subdivisions (yours = 1), shifts per-point times so the last point is 0, and advances the
  scan timestamp by the last point's time; `HandleRangefinder` transforms the cloud into the
  tracking frame (origin = `sensor_to_tracking->translation()`) and calls
  `trajectory_builder_->AddSensorData(... TimedPointCloudData ...)`. `misses` is created later,
  in the local trajectory builder — never in `sensor_bridge`.
* `PointCloud2` takes the same trajectory-builder filter but **no message-level range filter**,
  so with `num_point_clouds = 1` points beyond 12 m *do* become misses (and `min_z`/`max_z` are
  applied relative to the tracking frame, to returns and misses alike).

---

## 4. `/map` (`nav_msgs/OccupancyGrid`) int8 values

### (a) Verbatim

`cartographer_ros/src/occupancy_grid_node_main.cpp` lines 182–191 (all it does):

```cpp
void Node::DrawAndPublish() {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  occupancy_grid_publisher_->publish(*msg_ptr);
}
```

`cartographer_ros/src/msg_conversion.cpp` lines 374–419 (the actual mapping — note: **no named
constants exist**):

```cpp
std::unique_ptr<nav_msgs::msg::OccupancyGrid> CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const rclcpp::Time& time) {
  ...
  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
  occupancy_grid->data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value =
          observed == 0
              ? -1
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid->data.push_back(value);
    }
  }

  return occupancy_grid;
}
```

How `color`/`observed` are produced — `cartographer/io/submap_painter.cc` lines 195–214
(`kCairoFormat` = `CAIRO_FORMAT_ARGB32`, declared in `cartographer/io/image.h:33`):

```cpp
  for (size_t i = 0; i < intensity.size(); ++i) {
    // We use the red channel to track intensity information. The green
    // channel we use to track if a cell was ever observed.
    const uint8_t intensity_value = intensity.at(i);
    const uint8_t alpha_value = alpha.at(i);
    const uint8_t observed =
        (intensity_value == 0 && alpha_value == 0) ? 0 : 255;
    cairo_data->push_back((alpha_value << 24) | (intensity_value << 16) |
                          (observed << 8) | 0);
  }
```

`cartographer/io/submap_painter.cc` line 106 (the canvas is filled **dark red** first):
`cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);`

`cartographer/mapping/2d/probability_grid.cc` lines 116–135 (the per-cell encode; `value` is
what becomes the red channel):

```cpp
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) {
      cells.push_back(0 /* unknown log odds value */);
      cells.push_back(0 /* alpha */);
      continue;
    }
    ...
    const int delta =
        128 - ProbabilityToLogOddsInteger(GetProbability(xy_index + offset));
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cells.push_back(value);
    cells.push_back((value || alpha) ? alpha : 1);
  }
```

### (b) Answers

* **Where it happens**: not in `occupancy_grid_node_main.cc`. That node subscribes to
  `SubmapList`, fetches each submap's compressed texture, renders it with
  `cartographer::io::DrawTexture`, and `PaintSubmapSlices` composites all slices onto a canvas
  pre-filled with `rgba(0.5, 0, 0, 1)`. `CreateOccupancyGridMsg` then reads the canvas pixels.
* **Exact rule (no named constants)**: `value = observed == 0 ? -1 : RoundToInt((1 - color/255)*100)`.
  Here `color` is the **red** channel (`packed >> 16` truncated to `unsigned char`), and
  `observed` is the **green** channel (`packed >> 8` truncated). The texture's red channel is
  the `value` byte from `DrawToSubmapTexture` (`= max(0, 128 − ProbabilityToLogOddsInteger(P))`),
  and green is `255` for any cell that was ever observed, `0` only for never-observed cells.
  Never-observed pixels keep the dark-red background (green = 0) → `-1`.
* **`-1` = unknown.** There is **no `0`/`100` constant pair any more**; the old binary
  `kUnknownOccupancyGridValue/kFreeOccupancyGridValue/kOccupiedOccupancyGridValue` scheme is not
  present in any branch I checked (`cartographer_ros` master, `release-1.0`, `ros2-dashing`,
  ROS2 fork `2.0.9002`) — all of them use the proportional formula above.
* **The mapping is proportional, but of log-odds-derived intensity, not of the probability, and
  it is capped well below 100.** Because of the cairo compositing (the texture mixes an
  intentionally "invalid premultiplied" intensity with `A = 0`, which cairo/pixman then *adds*
  to the dark-red background), I replicated the whole chain (`libcairo` + the exact packing and
  formula) to get the true effective table:

  | P(occupied) | published int8 | | P(occupied) | published int8 |
  |---|---|---|---|---|
  | unknown (never observed) | **−1** | | 0.55 | 52 |
  | 0.10 | **0** | | 0.60 | 55 |
  | 0.15 | 11 | | 0.65 | 57 |
  | 0.20 | 18 | | 0.70 | 60 |
  | 0.25 | 25 | | 0.75 | 62 |
  | 0.30 | 31 | | 0.80 | **65** |
  | 0.35 | 36 | | 0.85 | 69 |
  | 0.40 | 41 | | 0.90 | **75** ← maximum reachable |
  | 0.45 | 45 | | | |
  | 0.50 | 50 | | | |

  So: unknown = −1; "free" only reaches 0 when the cell is saturated at P = 0.1; the most
  occupied a cell can ever be published as is **75** (`P = kMaxProbability = 0.9`). The
  `CHECK_GE(100, value)` confirms the author expected ≤ 100, but the compositing caps it at 75.
  A cell at exactly P = 0.5 encodes `(value = 0, alpha = 1)` — the `(value || alpha) ? alpha : 1`
  clause exists precisely so a *known* mid-probability cell is not encoded as `(0,0)` (= unknown).
* **Direct implications for this workspace's debugging** (all verified against your config:
  `hit_probability = 0.68`, `miss_probability = 0.40`, `insert_free_space = true`):
  * one hit on an unknown cell → P = 0.68 → published **58**; two hits → 0.8187 → **67**;
    three hits → 0.9 → **75**.
  * a saturated wall (P = 0.9) decays with each miss update to P = 0.857, 0.8, 0.727, 0.64, …
    → published 75, **70, 65, 61, 56**, 52, 44, 35, 26, 17, 8, 0. **Three miss updates drop a
    saturated wall below 65.**
  * a wall observed only once (P = 0.68 → 58) falls to 54 → 49 with two misses.
  * `tools/analyze_slam_bag.py` classifies `occ = G >= 65`, i.e. **P ≥ 0.80**. So a wall needs
    two consecutive hit updates to count as "occupied" at all, and only ~2 miss updates to stop
    counting. Any consumer that assumes `100 == occupied` (e.g. a nav2 threshold of exactly 100)
    will never see an obstacle from this publisher.
  * combined with item 3: in your `/scan` setup `missing_data_ray_length` is inert, so the miss
    writes that cause the decay come exclusively from the origin→hit rays (item 1, "Now add the
    misses" loop) — i.e. from beams that *did* return, whose rays pass through a cell that was
    hit at an earlier/later scan.

---

## 5. 合并附录：上游 issue 调查、TSDF、替代方案与调参（原 `cartographer_map_stability_report.md`）

> **本节是 2026-10 仓库整理时并入的**：原文件 `cartographer_map_stability_report.md` 在仓库根、**未跟踪**（455 行，2026-09-23，英文），与本文是同一批源码取证的两种写法。整理时**只把本文 §1–§4 没有的证据**搬进来（§5.1 列了重复项的指针），随后删除该未跟踪文件；原文的源码级结论仍以本文 §1–§4 为准（那里是逐字引文）。

### 5.0 来源与证据分级（provenance）

| 项 | 值 |
|---|---|
| 原文件 | `cartographer_map_stability_report.md`（仓库根，**untracked**，455 行，最后修改 2026-09-23；原标题 *"Cartographer 2D map-stability research report"*，版本依据 rosdistro humble：<https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml>） |
| 原文覆盖 | ROS 2 Humble + `ros-humble-cartographer 2.0.9004`（`ros2/cartographer` 分支 `ros2`）；Livox MID-360 → 地面剔除 → `pointcloud_to_laserscan` 窄高度带 → cartographer 2D；症状 = `/map` 里占据墙被擦掉/闪烁（保留率 ≈40%），`insert_free_space=false` → 89% 但无 free space |
| 合并口径 | **只并入本文没有的**证据；重复部分见 §5.1。原文的 `[SRC]`（上游源码/官方文档已核）本文记 **【引用】**；`[USER]`（GitHub issue / 工程博客的真实用户报告）记 **【引用·用户报告】**；`[SPEC]`（原报告作者的推断）记 **【推断】** |
| ⚠️ 性质 | 本附录是**外部仓库/issue 调研**，**不是本仓实测**。本仓的实测数字只有本文 §4(b) 末尾那几条（`hit 0.68 / miss 0.40` → 75/58/…）与 `cartographer.lua` 的当前取值；本附录的"推荐值"一律**没有**在本仓 A/B 过 |

### 5.1 与正文重复、因此不再重抄的部分（原文 §0 的主体）

| 原报告结论 | 本文已有位置 |
|---|---|
| 2D 没有"占据过就永不清除"的规则/开关；`CastRays` 先 hit、再 `insert_free_space` 才走 miss 射线 | §1(a)(b) |
| `insert_free_space=false` 是**早退** ⇒ 89% 保留但 free space = 0 | §1(b) «`insert_free_space`» |
| 每格 log-odds 更新、`kMinProbability/kMaxProbability = 0.1/0.9`、unknown 读作 0.1、`kUpdateMarker` | §2(b) |
| `missing_data_ray_length` 只缩短"无回波/超 `max_range`"的射线，**救不了**越顶光束（原文另引 issue #354 "missing_data_ray_length has no effect?"） | §1(b)、§3(b) |
| `/map` 是子图纹理混合、最大值 **75**（不是 100）、`CreateOccupancyGridMsg` 的取整公式 | §4(b) |
| 本仓 `/scan` 的 `range_data.misses` **恒空**、`missing_data_ray_length` 在本仓**无效** | §3(b) |

> 【引用】**原文 §0 还有三条细节本文 §1–§4 未逐字覆盖，补记于此**：① `/map` 的
> `ProbabilityToLogOddsInteger` 把 `p ∈ [0.1, 0.9]` 映到 `[1, 255]`，再取 `delta = 128 − 该值`；
> ② 源码注释原话：*"walls will look too gray"*（对应"最大只到 75 而不是 100"）；
> ③ **重叠子图会在合成时叠加、把值推得更高** ⇒ 同一面墙的发布值可能高于单个子图的 75。

### 5.2 量化推论（原文 §0 的算术，正文未逐条列）

【引用】默认 `hit 0.55 / miss 0.49` 下：单次 hit 的 log-odds `ln(0.55/0.45) = +0.2007`、单次 miss `ln(0.49/0.51) = −0.0400` ⇒ **1 次 hit 恰好抵 5.02 次 miss**；从 unknown 到饱和需 **11 次 hit** 或 **55 次 miss**；clamp 到 `±2.1972`（`ln 9`）后，**已饱和的墙仍需 55 次 miss** 才会掉到"free"。

【引用·用户报告】`#1818` 的修复者把 `hit_probability` 提到 **0.75**（`miss_probability` 留 0.49）：hit odds 变 `+1.0986`、miss 仍 `−0.0400` ⇒ 比值变 **1 hit : 27.5 miss**；"被打过 3 次的墙"从 **15 次 miss** 变成 **82 次 miss** 才会掉到 P<0.5（本附录复算：`ln(3³)/0.04 = 82.4`）。

【推断】更激进的 `miss_probability = 0.499`：`ln(0.499/0.501) = −0.004` ⇒ 饱和墙要 **549 次 miss**、unknown 到 free 也要约 **549 帧 ≈ 55 s @10 Hz**——在小场地上可能"free space 出得太慢"，只在 0.49 不够时才用。

### 5.3 越顶光束（over-passing）：本文几何下的根因

【推断】一束从 **0.226 m** 高处出射、**越过 0.40 m 高的挡板**、打中远处墙（距离 R）的光束，在 2D 里与"穿过自由空间"**不可区分**：inserter 会把挡板所在的格（距离 `d << R`）当 free-space 擦掉，把远墙记为 hit。2D 管线里**没有任何东西知道"越顶"这件事**。

【引用·用户报告】MichaelGrupp（cartographer_ros #1457）对动态物体说的是同一机制：*"Dynamic objects are not explicitly modeled. But … a dynamic object can 'disappear' in the grid cell when later observations don't see an obstacle anymore at this point."*

【推断】**MID-360 让这件事更糟**：它是非重复扫描，方位/俯仰图案每帧都不同。对同一个方位角，带内最近的光束俯仰逐帧变化 ⇒ A 帧那条束打在挡板上（hit），B 帧稍高的一条束越过去、带内最近点变成远墙（hit ⇒ 擦挡板）⇒ **同一格上逐帧随机的 hit/miss 交替**，正是 5:1 的 log-odds 顶不住的图案。原文明确注明：**这是从 p2l 代码 + MID-360 规格推出来的，没有找到哪条 issue 明说**。

### 5.4 上游 issue 调查（原文 §1，本文正文没有）

| # | 结论 | 来源 |
|---|---|---|
| **1818** | 【引用·用户报告】**与我们的症状最像**："When I walk in one direction the tables are mapped as obstacles, however when i turn around … the tables disappear." 第二位用户（`marmik18`, 2024-11-12）同样复现，并**把 `hit_probability` 提到 0.75 + `miss_probability=0.49` 修好** | <https://github.com/cartographer-project/cartographer_ros/issues/1818> |
| **1338** | 【引用·用户报告】"Wall disappears in 2D SLAM"。MichaelGrupp：*"The occupancy grid node … just blends all submaps into an image, so it could be that it just **looks** as if the wall is gone. The submaps plugin … is better for checking."* 报告者随后发现墙只在 submap 0 里完整，且**把 `max_range` 降到 ~5 m 能缓解**（他想要 20 m）；另有人建议按**反射强度**过滤 | #1338 |
| **1678** | 【引用·用户报告】**`overlapping_submaps_trimmer_2d` 本身会擦数据**："sometimes, the trimming process will decrease the certainty of obstacles (**e.g., a wall may become free space**) or completely 'forget' parts of the map."；参数 `fresh_submaps_count=1, min_covered_area=2, min_added_submaps_count=5`；结论是内存/性能折衷 ⇒ 更保守或干脆别长期跑 SLAM | #1678 |
| **689** | 【引用·用户报告】"Map boundaries disappear over time" —— 被维护者以"RViz 对旧子图的**淡出显示**是 intended"关闭（PR #674）。⇒ 先确认看到的是 *Submaps* 显示的 fade，不是 `/map` | #689 |
| **1409** | 【引用·用户报告】反向问题（open）：障碍离开后 free space **从不**被改回来 —— 即没有可用的旋钮 | #1409 |
| **1845** | 【引用·用户报告】默认参数下 free space 要 ~5 分钟才在 `map.data` 里变成 `0`（open）；同机制的另一方向 | #1845 |
| **1538** | 【引用·用户报告】"How to tune the disappearing time of dynamic objects?"（open） | #1538 |
| **1457** | 【引用·用户报告】维护者对"格子为什么会被取消占据"的解释（§5.3 引的那段） | #1457 |

【引用】原文作者的**穷举检索**：在 `cartographer-project/cartographer{,_ros}` 里搜 `"free space"`、`wall disappear`、`map disappears`、`obstacle disappear`、`erase occupied`、`insert_free_space`（分别 28 / 23 条命中），**只有上面这些 issue**；ROS Answers / Discourse 没有额外信息；**没有任何维护者宣布的"修法"**。

### 5.5 从 3D 雷达跑 2D：`pointcloud_to_laserscan` 的语义与三条管线

#### 5.5.1 p2l 到底做了什么

【引用】`min_height`/`max_height` 是在 **`target_frame`** 里切的；然后每个点算
`index = (atan2(y,x) - angle_min)/angle_increment`，并且 **`if (range < ranges[index]) ranges[index] = range;`** ⇒ **带内最近的回波赢**。它**没有**逐束俯仰记账、**没有**"光束越过了什么"的概念。
`range_max` 也有影响：带内没有点的那一束发 `inf`（`use_inf=false` 时发 `range_max+1`），cartographer 再把 `> max_range` 的变成 `missing_data_ray_length` 的 miss 射线。

#### 5.5.2 三条管线

**选项 A（原生，不用 `pointcloud_to_laserscan`）：直接把 PointCloud2 喂给 cartographer 2D**（【引用】）

```lua
num_point_clouds = 1,          -- subscribe to PointCloud2 instead of LaserScan
num_laser_scans = 0,
TRAJECTORY_BUILDER_2D.min_z = -0.05   -- in tracking_frame, gravity aligned
TRAJECTORY_BUILDER_2D.max_z = 0.15    -- tight band around the 0.226 m scan plane
TRAJECTORY_BUILDER_2D.min_range = 0.3 -- kills ground ring + robot body
```

【引用】实现是 `local_trajectory_builder_2d.cc` 里的 `sensor::CropRangeData(sensor::TransformRangeData(...), min_z, max_z)`（另见 `range_data.cc`）。
【推断】相对 p2l 的好处：切带发生在**重力对齐之后** ⇒ 车体俯仰不会带着带子跑，也没有"整个 360° 里最近点赢"的 1°/束量化损失；但它**仍是 2D 投影**，越顶擦除照样存在。

**选项 B（已验证的真实配置：16 线 RoboSense RS16 → 2D）**（【引用·用户报告】：中文实践者的 `rs16_2d_outdoor.lua`，逐参数解释）

```lua
num_point_clouds = 1
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 100.
TRAJECTORY_BUILDER_2D.min_z = 0.2          -- "multi-line: prevents ground points polluting the map"
TRAJECTORY_BUILDER_2D.max_z = 1.4
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1
POSE_GRAPH.optimize_every_n_nodes = 160.      -- ">= 2 x num_range_data"
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.48
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
```

【引用·用户报告】同一份配置还用 `TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }`、`use_pose_extrapolator = false`；其调参规则：*"loop-closure thresholds — if unstable with false matches, raise these two; in a highly repetitive scene, lower them or turn loop closure off"*，并明确报告**重复走廊里的假回环会把图搞坏**。来源：<https://blog.csdn.net/xjblalala/article/details/125049642>。

**选项 C（本仓现状）：保留 `pointcloud_to_laserscan`，但把带子收紧到传感器高度附近并夹住 `range_max`**（【引用】原报告的示例值）

```yaml
pointcloud_to_laserscan:
  ros__parameters:
    target_frame: base_link
    min_height: -0.05     # metres, relative to base_link
    max_height: 0.10      # keep the top of the band well below the 0.4 m barrier top at range
    angle_increment: 0.00436   # 0.25 deg
    range_min: 0.3
    range_max: 8.0
    use_inf: true
```

【推断】几何：传感器 0.226 m、挡板顶 0.40 m 时，向上的光束只有**仰角 < `atan((0.40−0.226)/d)`** 才能越过挡板 ⇒ **2 m 处 4.9°、5 m 处 2.0°**。5 m 处 ±0.1 m 的带子 ≈ **±1.15°** ⇒ "紧、略低于中心"的带子 + 小 `range_max` 是扫描生成侧**唯一**的几何缓解手段。

#### 5.5.3 别人报的 free-space/越顶问题

【引用】cartographer issue 跟踪器里**没有**"over-passing"这个词；报的症状与我们相同（**视角一变障碍就消失**，#1818），维护者的回答是"取消占据是概率网格的设计"（#1457）。VLP-16 + cartographer 2D 可看 #725（无 IMU / TF 问题，用 `use_online_correlative_scan_matching` + TF 解决，而不是地图插入选项）。Livox MID-360 + nav2 在 ROS 2 有现成仓库，但是 **LIO + 重定位 + Nav2**，不是 cartographer-2D：<https://github.com/Ikunio/Lidar_nav2_ws>。

### 5.6 TSDF 子图模式：能不能解决？（结论：不能，别切）

【引用】原报告判决：**Humble 里 TSDF 是接通且能发 `/map` 的，但它不解决本问题，还带一个未修的漂移 bug。不要为这个切过去。**

**可用性**（【引用】，`ros2/cartographer` / `ros2/cartographer_ros` Humble 2.0.9004/2.0.9002）：

* `TSDF2D::DrawToSubmapTexture(...)` **存在**（`tsdf_2d.cc`）；
* `Submap2D::ToResponseProto()` 无条件调 `grid()->DrawToSubmapTexture(...)`（`submap_2d.cc`）；
* `occupancy_grid_node` 照旧经 `SubmapQuery` 取子图纹理并发 `nav_msgs/OccupancyGrid`；
* `CreateOccupancyGridMsg` 读同一套 premultiplied RGBA ⇒ TSDF 的 free/occupied/unknown 确实映到 `0 / ~75 / −1`，**没有 CHECK 或代码路径拒绝 TSDF**；
* `map_builder_test.cc` 把 `GlobalSlam2D` 与 `LocalizationOnFrozenTrajectory2D` 参数化到 `GridType::TSDF` ⇒ 上游**意图**是两者都能跑；
* `RealTimeCorrelativeScanMatcher2D`、`CeresScanMatcher2D` 都按 `grid.GetGridType()` 分支处理 TSDF，约束构建器对任意 `Grid2D` 建 `FastCorrelativeScanMatcher2D`。

**选项块**（【引用】，`ros2` fork 的 `trajectory_builder_2d.lua` 与上游相同）：

```lua
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "TSDF"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "TSDF_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter = {
  truncation_distance = 0.3,
  maximum_weight = 10.,
  update_free_space = false,          -- NOTE: default is false, and it is a *required* key (GetBool, no HasKey)
  normal_estimation_options = { num_normal_samples = 4, sample_radius = 0.5 },
  project_sdf_distance_to_scan_normal = true,
  update_weight_range_exponent = 0,
  update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
  update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
}
```

* 【引用】`update_free_space = false`（默认）⇒ **就是"没有 free space"那一档**：射线掩膜从 `origin + (1 - truncation/range)*ray` 开始，只碰 hit 前 `truncation_distance` 内的格子（`tsdf_range_data_inserter_2d.cc`：`const Eigen::Vector2f ray_begin = options_.update_free_space() ? origin : origin + (1.0f - truncation_ratio) * ray;`），与 `insert_free_space=false` 同结果。
* 【引用】`update_free_space = true` ⇒ **并不会**保护"射线沿途的障碍"：射线仍从原点出发，每格 `update_tsd = range - dist(cell,origin)` 夹到 `[−truncation, +truncation]`；只有**距 hit < 0.3 m（默认）**的格落在"表面带"，更前面的格全被夹到 `−truncation`，纹理把 `|TSD| = truncation` 渲染成 **free**。⇒ 3 m 外的一道矮挡板**照样被擦**。保护是"hit 周围 ±0.3 m 的圆盘"，不是"沿途每个障碍"。
* 【引用】权重：是**带权滑动平均**，每帧权重 `GaussianKernel(update_tsd, 0.5)`（表面 ≈0.80、夹紧带边缘 ≈0.67）⇒ 直射 hit 只比越顶 miss 高 **1.2×**；`maximum_weight` 只限制**存储**权重，**不阻止**均值移动（`updated_sdf` 用的是夹紧前的权重）。
* 【引用】纹理解码：`normalized_tsdf = pow(|TSD|/maxTSD, 0.5)`、`delta = weight*(normalized_tsdf*255-128)` ⇒ `+truncation` 与 `−truncation` **都**渲染成 free，只有 `|TSD| ≈ 0` 才渲染成 occupied。
* 【引用】**fast correlative scan matcher 对 TSDF 打分是错的**：`PrecomputationGrid2D` 用 `1 - |grid.GetCorrespondenceCost(cell)|` 建，TSDF 的 free 格 correspondence cost = 0.1 ⇒ 得分 **0.9**，即**自由空间拿到最高分**（与 cartographer#1625 "Why the score of unknown area is highest?" 同款）。
* 【引用】**官方没文档**：生成的配置参考里完全没有 TSDF 项，算法 walkthrough 以 `**TODO**: document TSDF configuration` 结尾。

**真实用户反馈（全是坏消息）**（【引用·用户报告】）

| # | 报告 | 来源 |
|---|---|---|
| 1518 | **open**：在**官方 bag** 上开 TSDF 就漂："I observed this drifting of map. It seems like some pose transformations are applied with wrong time stamp." | cartographer#1518 |
| 1648 | **open**：Gazebo 里 `grid_type="TSDF"`，"the state of yaw expressed in map coordinate rotates constantly as time goes on even though the robot is at stationary … the map becomes corrupted as time passes." | cartographer_ros#1648 |
| 1058 | closed，真实用户：默认参数下扫描相对（子）图 **"slide around"** 得厉害 | cartographer_ros#1058 |
| 1634 / 1842 | TSDF 位姿/优化不稳、初值漂移、yaw 不固定 | #1634 / #1842 |
| 1709 / 1632 / 1713 | "no matter how I configure Lua, I can't get the ideal result"；"Are there any reference … TSDF lua file?"（无人答）；仿真 bag 上调 TSDF | #1709 / #1632 / #1713 |
| 1466 | **open**：CI 里 `MapBuilderTestByGridType.GlobalSlam2D/1`、`LocalizationOnFrozenTrajectory2D/1`（正是 TSDF 参数化）失败，加上 TSDF inserter 测试不稳定（#1514） | #1466 |

【引用】原报告结论：**没有找到任何人**用 cartographer 2D TSDF 做出稳定 2D 占据图的报告。

### 5.7 其它旋钮：别人实际改成什么、换来什么（原文 §4）

【引用】为什么没有"永不清除占据"的开关：`probability_grid_range_data_inserter_options_2d.proto` **只有三个字段** `hit_probability`、`miss_probability`、`insert_free_space`；3D 侧有 `TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels`（"0 disables free space"），**2D 的对应物只有这个布尔**。原文 §6 还给了"今天可配置的保守 free-space 更新"四点清单：①提高 hit/miss 不对称 ②缩短擦除射线（`max_range`/`missing_data_ray_length`/`min_range`/窄 `min_z,max_z`）③提高每格 hit 次数（`num_accumulated_range_data`、`motion_filter`、更小的 `num_range_data`）④从 cartographer 外部供 free space（nav2 `track_unknown_space: false`）或换 slam_toolbox 的 hit-ratio 规则。**落地顺序见 §5.9。**

| 旋钮 | 用过的值 | 效果 / 来源 |
|---|---|---|
| `…probability_grid_range_data_inserter.hit_probability` | **0.75** | 【引用·用户报告】#1818 报告者"playing around with these 2 values"修好桌子/障碍消失 |
| `…miss_probability` | **0.49**（同一报告） | 【引用】比值变 1 hit : 27.5 miss；3-hit 的墙要 **82** 次 miss（而不是 15） |
| 同一对，更激进 | `miss_probability = 0.499` | 【推断】`ln odds −0.004` ⇒ 549 次 miss 才能擦掉饱和墙；free space 约 55 s 才饱和（小场地可能太慢） |
| `TRAJECTORY_BUILDER_2D.max_range` | **≈5 m** | 【引用·用户报告】#1338："setting max_range to around 5m reduces the problem"（他想要 20 m）。注意 `> max_range` 的回波会变 `missing_data_ray_length` miss 射线 ⇒ 减少长距离掠射束 |
| `TRAJECTORY_BUILDER_2D.missing_data_ray_length` | 默认 5 m；建议 **1–2 m** | 【推断】只缩短"无回波"射线；无害，减少推测性擦除。⚠️ 本仓该值**无效**（§3(b)） |
| `TRAJECTORY_BUILDER_2D.min_range` | 0.3 | 【引用·用户报告·CSDN RS16】去掉地面环/车体 ⇒ 少假 hit |
| `TRAJECTORY_BUILDER_2D.min_z/max_z` | 0.2/1.4（室外车）、`-0.8/2.0` 默认 | 【引用·用户报告·CSDN】"multi-line: prevents ground points polluting the map" |
| `TRAJECTORY_BUILDER_2D.voxel_filter_size` | 0.02–0.025 | 【引用·用户报告·CSDN】密度 vs CPU |
| `TRAJECTORY_BUILDER_2D.num_accumulated_range_data` | **10** | 【引用】官方 `backpack_2d.lua` 与 unicorn-racing 定位 lua（"accumulate scans (reduce noise)"） |
| `…motion_filter.max_distance_meters / max_angle_radians` | 0.1 / 0.004 | 【引用·用户报告·CSDN RS16】更密的插入（默认 0.2 / 1°） |
| `…submaps.num_range_data` | **80** | 【引用·用户报告·CSDN RS16】；经验式 `optimize_every_n_nodes ≈ 2 × num_range_data = 160` |
| `…ceres_scan_matcher.occupied_space_weight` | **10**（translation/rotation 各 1） | 【引用·用户报告·CSDN RS16】更信 scan/map 匹配。**但**【引用】它是 **Ceres 扫描匹配的 cost weight**，**不参与栅格更新** ⇒ 单靠它不能阻止擦除，只能避免"配准错了看起来像墙被擦"（调高只让位姿更贴图；人们常用 10–20、默认 1） |
| `POSE_GRAPH.constraint_builder.min_score` / `global_localization_min_score` | 0.48 / 0.60（`pose_graph.lua` 里 0.55/0.6） | 【引用·用户报告·CSDN】匹配不稳/错匹配就调高；**重复场景里假回环会把图搞坏** ⇒ 那里建议调低或关回环 |
| `POSE_GRAPH.optimize_every_n_nodes` | 160（建图）、**2**（定位）、**0**（诊断时关全局 SLAM） | 【引用】unicorn-racing；官方文档："Setting `optimize_every_n_nodes` to 0 is a convenient way of disabling global SLAM" |
| `POSE_GRAPH.overlapping_submaps_trimmer_2d` | `{fresh_submaps_count=1, min_covered_area=2, min_added_submaps_count=5}` | 【引用·用户报告】**不要为地图稳定性开它**：会把墙变 free、把图"忘掉"（#1678）；`pose_graph.lua` 默认注释掉 |
| `TRAJECTORY_BUILDER.pure_localization` + `pure_localization_trimmer.max_submaps_to_keep` | `= 3` | 【引用】冻结的先验子图**永不被改** ⇒ 先验图的 `/map` 完全稳定；滚动窗口只加 3 个局部子图 |
| `occupancy_grid_node` gflags | `-include_frozen_submaps`、`-include_unfrozen_submaps`、`-resolution`、`-publish_period_sec`、`-occupancy_grid_topic` | 【引用】用来判定"擦除发生在冻结子图还是活跃子图" |

【引用】**最便宜的决定性诊断**：用 cartographer RViz 的 *Submaps* 插件逐个看子图（**关掉 fade**），和/或让 occupancy grid node 跑 `-include_unfrozen_submaps=false`。若墙在冻结子图里还在 ⇒ 只有当前活跃子图被擦、闪烁是**混合伪影**；若子图里就没了 ⇒ 是**真的栅格更新**。

### 5.8 替代方案

#### 5.8.1 slam_toolbox：**没有** `prob_hit`/`prob_miss`，但更新规则更保守（原文 §5a）

【引用】**对"slam_toolbox 也有 prob_hit/prob_miss"这个前提的更正**：`grep -r "prob_hit"` 在 ros2 分支**无命中**。它的占据栅格由 `karto::OccupancyGrid` 建，**每格两个单调计数器**、按比值定占据：

```cpp
// karto_sdk/include/karto_sdk/Karto.h
virtual void UpdateCell(kt_int8u * pCell, kt_int32u cellPassCnt, kt_int32u cellHitCnt) {
  if (cellPassCnt > m_pMinPassThrough->GetValue()) {          // default 2
    kt_double hitRatio = cellHitCnt / (kt_double)cellPassCnt;
    if (hitRatio > m_pOccupancyThreshold->GetValue())         // default 0.1
      *pCell = GridStates_Occupied;
    else
      *pCell = GridStates_Free;
  }
}
```

* 【引用】每条穿过某格的束让 `passCnt` +1；每条**终止**在该格的束同时让 `hitCnt` +1（`RayTrace()`）；**两个计数器只增不减、不衰减**。
* ⇒ 一个被 hit 过 H 次的格子，要再积累约 **9×H 次 pass-through** 才会被擦（`H/(H+M) ≤ 0.1`）——是**整个运行期累计**，不是 cartographer 那种"十几次 miss"。
* 【引用】发布的地图在每次 `updateMap()` 时由**位姿图里的全部扫描确定性重算**：`karto::OccupancyGrid::CreateFromScans(mapper->GetAllProcessedScans(), resolution, min_pass_through, occupancy_threshold)`（`slam_mapper.cpp`）。⇒ 不会出现 cartographer 增量 log-odds 那种"逐帧闪"。

【引用】slam_toolbox 的稳定配方（参数名/默认值已核；**参数值是【推断】的保守化，不是社区验证的预设**）：

```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    resolution: 0.05
    min_pass_through: 2          # default 2; 1 makes free space appear sooner
    occupancy_threshold: 0.05    # default 0.1 -> walls stickier (erase needs ~19x hits instead of ~9x)
    # scan admission: fewer, better-separated scans (map is rebuilt from graph scans only)
    minimum_time_interval: 0.5   # default
    minimum_travel_distance: 0.3 # default 0.5
    minimum_travel_heading: 0.3  # default 0.5
    # save the pose graph periodically / use serialization so a bad closure can be undone
    scan_buffer_size: 10
    loop_search_maximum_distance: 3.0
    correlation_search_space_dimension: 0.5
    correlation_search_space_smear_deviation: 0.1
```

* 【推断】代价：`occupancy_threshold` 调低也会让**偶发假 hit** 更容易被判 occupied。原报告明确写：**没有找到"低 3D 雷达 + 0.4 m 挡板 + 需要 free space"这个场景的公认配置**，上面只是"保守 free-space 更新"的源码等价物。
* 【推断】越顶的**几何问题在 slam_toolbox 里照样存在**（喂的是同一张 2D 扫描）；变的是更新规则——把"墙在 15 次 miss 后被擦"变成"墙在 9×hit 次 pass-through 后才被擦"。

#### 5.8.2 nav2 `static_layer` + `track_unknown_space: false`：让 `insert_free_space=false` 可用（原文 §5b）

【引用】`nav2_costmap_2d/plugins/static_layer.cpp` 的 `interpretValue`：

```cpp
unsigned char StaticLayer::interpretValue(unsigned char value) {
  if (track_unknown_space_ && value == unknown_cost_value_)       return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_) return FREE_SPACE;
  else if (value >= lethal_threshold_)                            return LETHAL_OBSTACLE;
  else if (trinary_costmap_)                                      return FREE_SPACE;
  double scale = value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}
```

* 【引用】`track_unknown_space` **默认就是 `false`**、`unknown_cost_value` 默认 `255 (=0xff)`，正好是 cartographer 的 `-1` 经 int8→uint8 转换后的值 ⇒ **cartographer 的 unknown 变成 FREE_SPACE**。
* 【引用】**坑**：`lethal_cost_threshold` 默认 **100**、`trinary_costmap` 默认 **true**；而 cartographer 的占据上限只有 **≈75**（本文 §4(b)）⇒ **用 nav2 默认值时，每一面 cartographer 墙都会变成 FREE_SPACE**。必须把 `lethal_cost_threshold` 设成 **60**（#628 的用户就按 60 卡）或更低。
* 【引用】对应 yaml：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      track_unknown_space: false     # unknown (-1) -> free
      lethal_cost_threshold: 60      # cartographer walls are ~57..90, never 100
      trinary_costmap: true
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: true
```

* 【引用】原报告的判断：这是"字面问题最干净的纯配置答案"——cartographer 用 `insert_free_space=false`（墙稳定在 89%），让图 = 墙 + unknown，costmap 把 unknown 当 free。代价是失去"从未见过"与"已知 free"的区分（封闭场地通常可接受）。

#### 5.8.3 hector_slam（原文 §5c）：**未验证**

【引用】原报告**没能验证**（访问 hector_slam 仓库时网络中断）：`hector_mapping` 一般被认为暴露 `update_factor_free`/`update_factor_occupied`（log-odds 更新因子）与 `map_update_distance_thresh`/`map_update_angle_thresh`；**无法打开** `hector_mapping/include/hector_mapping/map/OccupancyGridMap.h` 确认擦除规则。**按未验证对待。**

【推断】定性上：hector_slam 没有回环 ⇒ 子图永不被重优化，消除了"优化后地图重排/闪"这一失效模式，但对越顶擦除**没有任何帮助**。

### 5.9 原文的建议动作顺序（原文 §7，【引用】值全部来自上面各来源）

```lua
-- 1. make hits dominate misses (biggest single lever; user-verified in #1818)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.75
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- 2. limit how much free-space carving each scan can do
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 12.       -- larger than the arena; try 6-8 if walls still get eaten
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.

-- 3. keep the height band tight & gravity-aligned (drop pointcloud_to_laserscan, feed the cloud directly)
num_laser_scans = 0
num_point_clouds = 1
TRAJECTORY_BUILDER_2D.min_z = -0.06
TRAJECTORY_BUILDER_2D.max_z =  0.12

-- 4. more hits per insertion, fewer redundant insertions
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- 5. make sure nothing else deletes data
POSE_GRAPH.overlapping_submaps_trimmer_2d = nil   -- keep it commented out (default)
POSE_GRAPH.optimize_every_n_nodes = 2 * TRAJECTORY_BUILDER_2D.submaps.num_range_data   -- e.g. 180 for num_range_data=90
-- if the arena is repetitive: raise POSE_GRAPH.constraint_builder.min_score (0.55 -> 0.65) and
-- global_localization_min_score, or set optimize_every_n_nodes = 0 while diagnosing
```

【引用】nav2 侧：`track_unknown_space: false`、`lethal_cost_threshold: 60`、`trinary_costmap: true`。
【引用】若做完这些保留率仍不可接受：换 slam_toolbox（`occupancy_threshold: 0.05`），或把图冻结（cartographer `pure_localization` + `pure_localization_trimmer.max_submaps_to_keep = 3`）。**不要**切 `grid_type = "TSDF"`。

### 5.10 与本仓现状的差异（【推断】，别把附录当成本仓实测）

1. **本仓 `cartographer.lua` 是 `hit 0.68 / miss 0.40`**（本文 §2(b)/§4(b)），不是附录推荐的 0.75/0.49；0.75/0.49 是**外部用户**在 #1818 修好的值 ⇒ 想采纳必须在本仓做 A/B（`tilted_lidar_fidelity.md`/`debug_fastlio_cartographer.md` 的方法）。
2. 本仓这份 `src/localization/rm_localization/cartographer/` 是 **fork**，多了 `min_probability_to_clear 0.80` 这类扩展（`docs/mapping/README.md` 有记）；**附录完全没有覆盖这个旋钮**，它与 `insert_free_space` 的交互**未验证**。
3. 本仓 `/scan` 的 `range_max: 10.0` / `max_range: 12.0`（本文 §3(b)）与附录假设一致 ⇒ §5.1 那几条"misses 恒空"的结论对附录同样成立。
4. 附录的"保留率 ≈40% / 89%"是 **2026-09-23 那次会话**的症状数字，本附录**不把它当本仓现行实测**；本文 §4(b) 的 75/58/… 才是可复算的本仓数字。

### 5.11 未验证 / 缺口（原报告自己声明的 + 本附录的）

* **hector_slam 源码没打开**（网络失败）⇒ §5.8.3 全部按"未验证"对待。
* **MID-360 逐帧随机的 hit/miss 交替是【推断】**（p2l 代码 + 非重复扫描规格），**没有** issue 明说。
* 所有 `[USER]` 结论都是**别人的机器/别人的配置**，本仓**没有复现**；参数值只能当"候选"，不能当"结论"。
* TSDF 的用户报告是 issue 文本，**未在本仓复现**；TSDF 的 scan-matcher 打分 bug 是源码推断（【引用】公式 + 代码路径），未跑实验验证。
* 附录**没有**覆盖本仓 fork 的扩展旋钮（`min_probability_to_clear` 等），也没有覆盖 `mode:=mapping` 下 `map→odom` 的静态语义（那属于 `docs/mapping/` 与 `debug_fastlio_cartographer.md` 的范围）。


---


## 来源 18/19：`research/livox_viewer2_launch_notes.md`


> 触发 = 用户 2026-10-08 的原话：「帮我打开看雷达网口的那个软件」。
> 本文**只做两件事**：**(A)** 把"官方 Linux 查看器在本机起不来"这件事**逐项取证**，说清
> 哪些原因被**实测排除**、崩溃到底在哪；**(B)** 给一条**已验证可用**的替代路线
> （ROS2 驱动 + RViz，一条命令：`tools/scripts/livox/start_livox_mid360.sh`）。
>
> 环境：Ubuntu 22.04.5 LTS / glibc 2.35 / NVIDIA 550.163.01（RTX 4070 Laptop，Vulkan 1.3.277）/
> 原生 X11 会话（`DISPLAY=:1`，非 Wayland）/ Intel i7-14700HX / 32 GB。

## 0. 结论先行

**Livox 官方 Livox Viewer 2 的 Linux 版在本机（以及本机之外同样的桌面会话里）启动必然 SIGSEGV**，
与"哪个版本"无关：**2.3.0（UE4.25，OpenGL）与 2.6.2（UE4.27，Vulkan）崩在同一处**。
崩溃点位于 **`WBP_ViewerMain` 主界面控件构造**时的一次**空指针解引用**（`rax=0`）。

⇒ 因此改为 **ROS2 驱动 + RViz** 看雷达（见 §3）。这不需要官方查看器。

## 1. 现象与崩溃现场（逐字）

两个版本的日志都停在同一步（`Maps/Viewer` 起来后 ~40 ms）：

```
LogLoad: Game class is 'ViewerGameMode_C'
LogGameMode: Match State Changed from WaitingToStart to InProgress
ShowFlag.PostProcessing = "0" / r.ScreenPercentage = "200" / ShowFlag.Lighting = "0" …
[ 36]LogUMG: Display: Widget Class WBP_ViewerMain_C - Loaded Fast Template.   ← 建主界面
CommonUnixCrashHandler: Signal=11
Uncaught Exception: SIGSEGV: invalid attempt to read memory at address 0x0000000000000000
```

gdb 现场（2.6.2，`-nullrhi`）：

```
0x2f76441  mov  -0x260(%rbp),%rax
0x2f76448  mov   0x18(%rax),%rax     ← rax=0
0x2f7644c  movzwl (%rax),%ecx       ← 读 0 号地址 → SIGSEGV     rax=0x0 rdi=0xa
```

调用栈（用户在自己终端里跑出来的完整 50 帧）呈**深递归**：`0x385abdd → 0x385a4e9 → 0x385b60a`
这一组地址**重复 8 次**，符合"控件树递归构造/布局"的形态；崩溃前 5 次打印
`ADD THRESHOLD 30/90/150`（= `ViewerConfig.json` 的 `ReflectivityToColorThresholds`），
说明**配置文件读到了、pak 挂载正常**（`LogPakFile: Mounting pak file … OK`）。

## 2. 逐项排除（每条都有实测对照）

| # | 假设 | 实验 | 结果 |
|---|---|---|---|
| 1 | 程序装在**只读**目录 ⇒ UE4 建不了 `Saved/Logs` | 从 `~/下载/LivoxViewer2 for Ubuntu v2.3.0`（只读挂载）直接跑 | **真问题**：`Failed opening … for writing: Read-only file system` → 复制到工作区可写目录后过掉这一关，露出下一条 |
| 2 | 沙箱/容器 PID 命名空间导致 pthread 失败 | 对照：沙箱内 vs `danger-full-access` 真实环境 | 沙箱内多一条 `mutex lock failed`；**真实环境仍 SIGSEGV** ⇒ 不是沙箱 |
| 3 | 下载/复制**文件损坏** | `md5sum` 对比原件与副本：二进制 `4c5105155202e3768fc770f9bb8ee3e3` 一致；pak `0d3f6d64cf4f5d0bb962557ed6ac888a` 一致 | **不是** |
| 4 | **显卡/渲染后端**问题 | `-nullrhi`（完全不渲染） | **仍崩在同一处** ⇒ 与渲染无关 |
| 5 | Vulkan 设备选择（NVIDIA vs llvmpipe） | `VK_ICD_FILENAMES=lvp_icd…`（软渲染）/ `__NV_PRIME_RENDER_OFFLOAD=1` | 两种情况都崩 ⇒ **不是** |
| 6 | 版本太老 | 从官网下 **2.6.2**（2026-09-21，UE4.27） | **同一处崩溃**（连引擎大版本都换了）⇒ 不是版本问题 |
| 7 | 音频子系统 | `SDL_AUDIODRIVER=dummy` / `-nosound` | 仍崩 |
| 8 | 中文 locale / 本地化缺失 | `LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8` | 仍崩（日志里 `No localization for 'zh-CN' exists` 只是回退到 en，不是原因） |
| 9 | D-Bus / 无障碍（at-spi） | `env -u DBUS_SESSION_BUS_ADDRESS NO_AT_BRIDGE=1 NO_AT_SPI=1` | 仍崩 |
| 10 | `LD_LIBRARY_PATH` 里的 ROS/Gazebo/MVS 库干扰（ROS 的 OpenSSL 等） | `env -u LD_LIBRARY_PATH -u LD_PRELOAD` | 仍崩 |
| 11 | 本 shell 的 **capabilities 全零**（`CapEff=0`） | `systemd-run --user`（能力集正常） | `ExecMainStatus=139` ⇒ 仍崩 |
| 12 | 缺少系统依赖/字体 | `ldd`：**0 个** `not found`；`fc-list`：**502** 个字体 | 干净 |
| 13 | 只有我的自动化会话会崩 | **用户在自己的终端里跑** | **同一个崩溃、同一处**（§1 的 50 帧栈即来自这次）⇒ 与环境无关，是程序自身缺陷 |

公开同类反馈：[Livox-SDK#226「Crash after Splashscreen」](https://github.com/Livox-SDK/Livox-SDK/issues/226)、
[livox_ros_driver2#145「crashes in debian and ubuntu」](https://github.com/Livox-SDK/livox_ros_driver2/issues/145)
（2026-10-08 当天 GitHub 经代理返回 403，**正文未取到**，只确认到标题与存在性）。

**仍未验证**：崩溃那三层递归函数（`0x385abdd/0x385a4e9/0x385b60a`）到底是什么
（无 `.sym` 符号、二进制为 Development 构建但未带符号表）；因此**没有**"它想要哪个空对象"的直接证据，
§0 的"控件构造时空指针"是**由现场形态（UMG + 深递归 + nullptr + 无渲染依赖）推断**，不是符号级定论。

## 3. 替代路线：ROS2 驱动 + RViz（已验证可用）

本仓 `src/driver/rm_driver/livox_ros_driver2` **已编译**（`install/livox_ros_driver2`），
且**已有真机配置**：`src/driver/rm_driver/livox_ros_driver2/src/config/MID360_config.json`
（本机 `192.168.1.50`、雷达 `192.168.1.178`、端口 56100/56300/56400 那套）。
`install/` 里那份是**指向源码的符号链接**，改源码即生效，**不需要重新 `colcon build`**。

> ⚠️ **这台雷达的 IP 会变**（DHCP 自动获取）。实证：`archive/reality_frozen/MID360_config.json`
> 里是 `.119`，本次会话开始时配置里是 `.146`，实测雷达在 **`.178`**（MAC `0c:9a:e6:03:fc:67`，
> OUI `0C:9A:E6` = SZ DJI Technology，Livox 属 DJI）⇒ **至少变过三次**。
> 所以**不要**把 IP 当成固定值：链路通但没数据时，**先核对 IP**（见 §3.5）。

一条命令：

```bash
cd ~/HzMi_rmsimulation
./tools/scripts/livox/start_livox_mid360.sh            # 配网卡(如需) + 驱动 + RViz
./tools/scripts/livox/start_livox_mid360.sh --check     # 只自检，不开 RViz
./tools/scripts/livox/start_livox_mid360.sh --skip-net  # 网卡已配好，跳过改网卡
```

脚本做 7 件事：① 读配置并与脚本变量对账；② 找有线网卡（排除 wifi/docker/tun）；
③ 网卡不是 `192.168.1.50/24` 就用 `nmcli` 配上（**需要 sudo**，脚本会提示）；
④ `ping`/`arping` 连通性（无 `arping` 时自动跳过）；⑤ `source` ROS + 工作区；
⑥ 起驱动并**硬前置检查** `192.168.1.50` 是否在本机；⑦ 起 RViz（
`config/display_point_cloud.rviz`：Fixed Frame `livox_frame`、Topic `/livox/lidar`）。

### 3.1 本机当前硬件状态（实测）

```
enx00e04c0b2110   driver=r8152   operstate=down   NetworkManager: unavailable
```

- 机器**没有内置有线网口**（只有 `wlp0s20f3` WiFi），但有**一块 USB 网卡**（Realtek `r8152`）。
- 它现在是 `down`/`unavailable` ⇒ **网线没插（或雷达没上电）**。插上后 `operstate` 应变 `up`，
  再跑脚本即可。
- 注意：`~/.ros/log` 在自动化会话里是只读的（spdlog 会直接 abort），脚本已把驱动日志
  改到 `<工作区>/.tmp_roslog_driver/`（可用 `LIVOX_LOG_DIR` 覆盖）。

### 3.1b 实测：网卡配好了、雷达也上电了，但**物理层仍没链路**（2026-10-08）

用户执行全过程（逐步取证）：

```
== 2. 网卡地址 ==  当前:（未配置 IPv4）  链路: down          ← ① 配 IP 之前就 down
  … sudo nmcli … 连接已成功激活 … 现在: 192.168.1.50/24     ← ② IP 配上了
== 3. 连通性 ==  ping 不通 / ARP 表: 192.168.1.146 INCOMPLETE ← ③ 无人应答
== 5. 启动驱动 == 驱动已启动（这次**没有** bind failed）      ← ④ 说明 ①② 的修复生效
== 6. 话题自检 == /livox/lidar 存在但**无频率**               ← ⑤ 设备根本不在
```

现场取证（**这才是"没数据"的真正原因**）：

```
carrier                                          → 0
ethtool … | grep -i 'link detected'              → Link detected: no
ethtool … | grep -iE 'speed|duplex'              → Speed: 10Mb/s  Duplex: Half   ← 未协商上的占位值
lsusb | grep -i realtek                          → 0bda:8153 RTL8153（硬件正常）
ip route show | grep 192.168.1                   → 192.168.1.0/24 dev enx… linkdown（路由本身对）
lsusb -t                                         → Port 4: Driver=r8152, 480M（USB 2.0，带宽够）
```

**结论：`carrier=0` + `Link detected: no` ⇒ 网卡在 RJ45 侧收不到任何电信号**，与 IP、路由、
ROS、驱动、USB 带宽**全都无关**。`10Mb/s Half` 是未建立链路时的默认显示，**不能**读成
"协商到了 10M"；`ip neigh` 里的 `INCOMPLETE` 是这条的必然结果（请求发出、无人应答）。

**排查顺序（便宜→贵）**：

1. `ethtool <iface> | grep 'Link detected'` —— 它是后面一切的前提。
2. **拔插网线两端**（尤其 MID-360 那端的航空插头/水晶头），并**换一个 USB 口**重插网卡
   （r8152 的 PHY 偶发卡死，USB 重插会重新初始化）。
3. **换一根网线**，并把这块 USB 网卡插到**已知能用的交换机/路由器**上看 link 是否变 `yes`
   ⇒ 一刀切开"网卡/线的问题"与"雷达端的问题"。
4. **确认雷达在运行态**：MID-360 需**独立供电 9–27 V**（**网口不支持 PoE，网线不供电**）；
   指示灯见官方《Mid-360 User Manual》的 Working States 表（正常应为绿灯常亮）。
   上电但供电不足时，可能"亮灯但网口 PHY 不起"。
5. 确认网线插的是**雷达本体网口**，以及链路上是否串了 HAP/交换机（那要看中间设备的口）。
6. 物理层修好后，若雷达 IP 不是配置里的 `192.168.1.146`，用
   `tools/scripts/livox/find_livox.sh` 扫出真实 IP/MAC 再回填配置（它会先检查链路，down 时直接拒绝扫描）。

### 3.1c 同一故障的后续取证：USB 层已全部排除（2026-10-08 深夜）

用户把 USB 网卡**换了一个 USB 口**后重测，读数完全相同（`carrier=0`、`Link detected: no`、
`rx_bytes=0`），并且这次 `dmesg` 可读，拿到了 USB 层的完整证据：

```
usb 1-4: USB disconnect, device number 25          ← 从 Port 4 拔出
r8152 1-4:1.0 enx00e04c0b2110: Stop submitting intr, status -108
usb 1-9: reset high-speed USB device number 28 using xhci_hcd   ← 插到 Port 9
r8152 1-9:1.0: load rtl8153b-2 v2 04/27/23 successfully         ← 固件加载成功（芯片 = RTL8153B）
r8152 1-9:1.0 eth0: v1.12.13
r8152 1-9:1.0 enx00e04c0b2110: renamed from eth0                ← 正常注册
```

加上 `over-current/overcurrent` 计数为 **0**，⇒ 下面这些**全部排除**：

| 变量 | 证据 |
|---|---|
| USB 口 | 换口（Port 4 → Port 9）读数不变 |
| USB 供电/过流 | dmesg 无过流事件 |
| 适配器固件 / 驱动绑定 | `load rtl8153b-2 v2 … successfully`、`eth0: v1.12.13` |
| USB 链路稳定性 | 无 fallback/反复 reset（只有插拔那一次 reset） |
| IP/路由 | 配好后是 `192.168.1.50/24`，路由 `linkdown` 正确 |
| **剩下的嫌疑** | **① 这块适配器的以太网 PHY ② 网线 ③ 雷达侧（本体网口/供电/插错口/中间设备）** |

**分岔判据（`tools/scripts/livox/link_test.sh`）**：把该 USB 网卡插到**已知能用的网口**
（路由器/交换机 LAN 口，或另一台电脑的网口 —— 现代网卡自动翻转，不需要交叉线）：

- `carrier` 变 **1** ⇒ 网卡+这根线都好 ⇒ 问题在**雷达侧**；
- 仍为 **0** ⇒ 问题在**网卡或线**（先换线，再换网卡；RTL8153 第三方适配器存在
  "能枚举、固件正常、就是不起链路" 的已知毛病）。

**另一条实测注意点**：这台机器上 `ethtool` 查询会报
`netlink error: Operation not permitted`（在受限会话里连 `systemd-run --user` 也一样），
**但 `Link detected:` 仍会打印且有效**。要求"零噪音"的读数请直接读内核文件：
`cat /sys/class/net/<iface>/carrier`（1=有信号）与 `operstate`，**不需要任何权限**。
`tools/scripts/livox/net_diag_livox.sh` 已按此调整：遇到该报错会标注"与雷达无关"，
并把内核文件作为速度/双工/链路的第一来源。

### 3.2 实测到的失败模式（脚本据此设了硬检查）

网卡没配 IP 时，驱动**能起来**但 SDK 初始化失败 —— 如果没有前置检查，会误判成"驱动正常、雷达没数据"：

```
successfully parse base config, counts: 1
bind failed
Failed to init livox lidar sdk.
[ERROR] [livox_lidar_publisher]: Init lds lidar fail!
```

原因：Livox SDK 会把本机 socket **绑到配置里的 host IP**（`host_net_info.*_ip`），
该地址不在本机 ⇒ `bind` 失败。⇒ 必须先配网卡（或改配置）。

### 3.5 实测：**配置里的雷达 IP 是旧的**（本次会话真正的拦路虎，2026-10-08 深夜）

物理层修好（`carrier=1`）后跑脚本，RViz 报：

```
Fixed Frame [livox_frame]  →  No tf data. Actual error: Frame [livox_frame] does not exist
```

⚠️ **这句报错很容易被误判成"缺 TF / Fixed Frame 配错"**，其实它只是"**一帧点云都没发布过**"的后果：
PointCloud2 自带 `frame_id=livox_frame`，RViz 渲染这种消息**根本不需要 TF**（该 launcher 里
`static_transform_publisher` 数量 = 0，属正常设计）。**不要**为此去加 TF 或改 Fixed Frame。

真正的读数（三条一起看，结论唯一）：

```
网卡 : carrier=1   rx_bytes=411,095,196      ← 雷达一直在回传 411 MB 数据
ARP  : 192.168.1.146 FAILED                  ← 配置里写的 IP 根本不存在
       192.168.1.178 lladdr 0c:9a:e6:03:fc:67 ← 雷达实际在这里（OUI 0C:9A:E6 = SZ DJI）
日志 : Init lds lidar success!
       GetFreeIndex key:livox_lidar_2449582272.
       Can not get index, the livox lidar type:8, handle:2986453184   ← 刷了 863 行
       Storage point data failed, can not get index, …
```

机制：SDK 按配置去连 `192.168.1.146`（连不上、无设备条目），而点云来自 `192.168.1.178` ⇒
**包收到了但没有可归属的设备 ⇒ 整包丢弃**（`Storage point data failed`），于是 `/livox/lidar`
一帧都不发。⇒ **改配置第 28 行 `lidar_configs[0].ip` 即可**（本次已改为 `.178`）。

**改哪里（唯一真值文件）**：

```bash
src/driver/rm_driver/livox_ros_driver2/src/config/MID360_config.json    # 第 28 行 "ip"
```

- `install/livox_ros_driver2/share/…/MID360_config.json` 是**符号链接**指向它 ⇒ 改一处即可，**无需重新构建**。
- 同文件第 14/16/18/20 行的 `192.168.1.50` 是**本机** IP，**必须保持与网卡配置一致，不要动**。

**防复发**：`start_livox_mid360.sh` 新增 **§3.5 前置检查** —— 进 ARP 表核对配置 IP；
不在线就自动 ping 扫同网段、列出在线设备并提示"把配置改成哪一个"，避免又对着空话题排查。

## 4. 未验证 / 后续可做

1. **雷达接上后的端到端验证**（当前**物理层未连通**，见 §3.1b）：链路变 `up` 后，脚本应显示
   `/livox/lidar ≈ 10 Hz`、`/livox/imu ≈ 100 Hz`。在此之前结论只能到"驱动能起、前置检查正确、
   `/livox/lidar` 话题已创建（但无发布者出数）"。
2. **`arping` 未安装**（`apt install iputils-arping` 可让第 4 步更强）。
3. **Wine 跑 Windows 版查看器**（官方 `Viewer2_2.6.2_Win.zip`）未尝试 —— 如需官方查看器的
   反射率着色/LVX2 录制体验，这是另一条路。
4. **容器方案未落地**：本机 `docker` 存在但当前用户**不在 `docker` 组**、且无免密 sudo，故未执行。
5. 崩溃的三层递归函数身份（见 §2 末）—— 要符号级定论需 Livox 提供 `.sym` 或自行反汇编。
6. `tools/scripts/livox/` 是**新增目录**；`tools/scripts` 不参与 colcon 构建（`tools/build/COLCON_IGNORE`），
   因此**不需要**同步到 `install/`，也不需要重新 `colcon build`。


---


## 来源 19/19：`research/livox_sensor_prior_art.md`


> **状态**：调研完成（2026-10-18）。**只读代码、只查资料，未编译、未运行、未改任何功能代码。**
> **回答的问题**：用户原话「有人这么干吗，自研的这个普便性大吗」。
> **上位约束与前置结论**看 [`gazebo_sim_migration_plan.md``gazebo_sim_migration_plan.md`（Humble↔Fortress 配对、`gpu_lidar` 不支持任意图案、无 GPU 裁决、R0/R1/R3 四方案决策表、我们自己的 Bullet BVH 实测）。

---

## §0 结论速览（TL;DR）

**三句话：**

1. **有人干过，而且不止一个。** 2025-04 ～ 2026-10 之间至少有 **6 个** 把 Livox 图案搬进**新 Gazebo**（`ign gazebo` / `gz sim`）的独立实现，其中 **2 个明确跑在 Fortress 上**，**2 个是 MIT / Apache-2.0 可自由复用**。
2. **最接近我们的那个几乎是现成的 R1**：`ashduwihch/mid360_gazebo_harmonic`（**MIT**）= `gz::sim::System` + **rmagine/Embree CPU BVH 批量求交** + **MID-360 CSV 图案** + 逐点 `offset_time/tag/line/reflectivity` + 直接发 `PointCloud2`/`CustomMsg`/`IMU`；**唯一硬缺口是它钉在 Harmonic（`gz-sim8`）**，Fortress 化在源码层只需 **3 处**（CMake 包名、插件注册宏、编掉 `RaycastData` 那条后端）。
3. **自研的"普遍性"很大——但正因为大，已经被人写完了。** CPU 批量射线求交当 LiDAR 仿真器是**主流做法**（PyBullet `rayTestBatch`、rmagine 的 ICRA 2023 论文、上游 Jetty 官方 `cpu_lidar` 都是这个思路），所以 R1 的**架构**没有新意、**不必重新发明**；我们真正独有、也真正需要自己把关的只有三件事：**Fortress 版本钉子**、**契约对齐**（frame/话题/字段/自机排除）、**30k@10 Hz 的实测验收**。

**⇒ 裁决：把 R1 从「写」降级为「移植 + 适配」**（工作量 **S–M**，见 §4）。**首选 A 方案**：以 `ashduwihch` 的 MIT 插件为骨架 + 上游 `uos/rmagine`（BSD-3，**README 明示 Humble+Fortress 已测**）作为 BVH 地基。

> ⚠️ **一个必须同时说清的坏消息**：**没有一个候选是"Fortress + 任意图案 + 现成可跑"三者齐备的**。
> Fortress 的 2 个（`DHA-Tappuri`、`jongbob1918`）分别是 **GPU 过采样重采样** 和 **专有许可 + 只支持 box/plane**；
> 纯 CPU BVH 的 2 个（`ashduwihch`、`rmagine_gazebo_plugins`）都对准 **Harmonic**（`rmagine_gazebo_plugins` 是双支持）。
> 所以「有没有现成的」答案是 **"有 90% 现成的"**，不是「有 100% 现成的」。

---

## §0.1 七条硬事实（决定一切，均可一手复核）

| # | 事实 | 证据（一手） | 对我们的意义 |
|---|---|---|---|
| F1 | **Fortress 没有 `components/RaycastData`**（`gz-sim8/9/10` 才有） | 本机 `/usr/include/ignition/gazebo6/gz/sim/components/` 下 `ls \| grep -i ray` **为空**；上游 `raw.githubusercontent.com/gazebosim/gz-sim/{gz-sim6,gz-sim7}/…/RaycastData.hh` = **404**，`{gz-sim8,gz-sim9,gz-sim10}` = **200** | Jetty 那条「把射线塞进 ECM、让 `gz-physics` 批量求交」的路在 Fortress **不存在** ⇒ **BVH 必须自己持有**（rmagine/Embree、Bullet）或退回渲染器 |
| F2 | **Jetty 的 `cpu_lidar` 自己不做求交** | `gz-sim10/src/systems/cpu_lidar/CpuLidar.cc:34` 引入 `components/RaycastData.hh`；`:71-79` 建 `RaycastData`+`NeedsRaycast`；`:217-242` 从 `_raycastData->Data().results` 读结果 | 它只是「**规则网格生成射线 + 交给物理引擎**」，**不是**可移植的 BVH 实现 ⇒ 只能当架构参考 |
| F3 | **`GenerateRays()` 不是 `virtual`**，`unitVectors` 是私有 `dataPtr` | `gz-sensors10/include/gz/sensors/CpuLidarSensor.hh:116-117` | 即便升到 gz-sensors10，子类也换不了图案（我们迁移计划 §3.4 的结论仍然成立） |
| F4 | **Fortress 支持 `<sensor type="custom">`** | `ign-gazebo6/src/SdfEntityCreator.cc:1022` 处理 `sdf::SensorType::CUSTOM` → 建 `components::CustomSensor`；本机 `/usr/include/ignition/gazebo6/gz/sim/components/CustomSensor.hh` 存在 | rmagine 那套「custom sensor 工厂」在 Fortress **可用**（这条我之前在迁移计划里没核实，现在核实了） |
| F5 | **Fortress 的 `gz/plugin/Register.hh` 只有 `IGNITION_ADD_PLUGIN`，没有 `GZ_ADD_PLUGIN`** | 本机 `/usr/include/ignition/plugin1/gz/plugin/Register.hh:53` 只定义 `IGNITION_ADD_PLUGIN` | **Harmonic 代码搬到 Fortress 唯一必改的 C++ 层就是注册宏**（1 个 `#ifdef`），其余 `gz::sim`/`gz::math`/`<gz/...>` 路径 Fortress 全有 |
| F6 | **`ros_gz_bridge`（Humble/Fortress 0.244.26）支持 `PointCloudPacked` ↔ `PointCloud2`** | 本机 `strings /opt/ros/humble/lib/libros_gz_bridge.so` → `Factory<sensor_msgs::msg::PointCloud2, ignition::msgs::PointCloudPacked>` | 走「gz 侧发 `PointCloudPacked` + 桥接」的路线**不用自己写桥**（但 `CustomMsg` 永远过不了 `ros_gz_bridge`，只能进程内发，见 §3.1） |
| F7 | **我们的 CSV 与候选的 CSV 是同一份数据、同一套方向公式** | 我们 `scan_mode/mid360.csv` 表头 `Time/s,Azimuth/deg,Zenith/deg`，`zenith∈[37.836, 97.2123]`；`ashduwihch` 的 `mid360-real-centr.csv` **同一范围**；我们 `livox_points_plugin.cpp:38-39` 做 `zenith−π/2` 后再 `Euler(0,zenith,azimuth)·X̂`，与 `ashduwihch/csv_scan_pattern.hpp:43-50` 的 `(sin z·cos a, sin z·sin a, cos z)` **数值等价**（我用 ignition-math6 编译对拍过） | **图案层零改动**：把 `<csv_file>` 指到我们自己的 CSV 即可；`(37.8…97.2)` 正好是 MID-360 的 `−7°…+52°` 俯仰（`90−ze`） |

---

## §1 我们的约束（一屏，用于对照表）

| 维度 | 我们的现状 |
|---|---|
| 目标栈 | ROS 2 **Humble** + `ros_gz` **0.244.26**（≡ **Fortress / `ignition-gazebo6`**，本机已装） |
| 传感器需求 | **30,000 条/帧、10 Hz、任意逐射线图案**（`mid360.csv` 800,001 行，滚动窗口 30k/帧 ⇒ 26.7 帧一循环） |
| 世界几何 | `RMUC2026_world`：**visual 和 collision 都是同一个 `RMUC2026.stl` mesh**（≈45,900 三角面，mm 单位、scale 0.001） |
| 下游契约 | `/livox/lidar` = `livox_ros_driver2/msg/CustomMsg`（FAST-LIO 吃这个）；`/livox/lidar/pointcloud` = `PointCloud2`（我们发 **xyz + rgb**）；`/livox/imu` = 100 Hz |
| 现状痛点 | Classic 里 **每条射线一个 ODE collision** ⇒ 30k 射线 = 2.74 GiB、gzserver 3.2 GiB、RTF 0.32–0.46 |
| 我们已有的实测 | 纯 CPU BVH（Bullet `btBvhTriangleMeshShape` + 真 CSV）：**30k 射线 12.5–29.4 ms（1 线程）/ 5.6–13.6 ms（4）/ 3.2–7.4 ms（8）** ⇒ 100 ms 预算够宽 |

---

## §2 候选总表

> 观察时间 **2026-10-18**。证据等级：**A**=读过实现源码；**B**=读过 README/CMake/SDF；**C**=只有搜索摘要。
> "任意图案"= 能否表达 **逐射线任意方向**（不是规则网格）。

| # | 候选 | 许可 | 最后活动 | 目标 Gazebo | 传感器 API / 求交方式 | 任意图案 | 性能说辞 | Fortress+Humble 直接可用 | 证据 |
|---|---|---|---|---|---|---|---|---|---|
| 1 | [`uos/rmagine`](https://github.com/uos/rmagine) + [`uos/rmagine_gazebo_plugins`](https://github.com/uos/rmagine_gazebo_plugins) | **BSD-3** | 2026-08-06（76★/17★） | **Harmonic + Fortress（README 明示都测过）** | `gz::sim::System` 成对（map system 建 BVH + sensor system 打射线）；后端 **Embree(CPU)** / OptiX(GPU) / Vulkan(GPU) | ✅ **O1Dn/OnDn**：`<rays_file>` YAML 里给 `width×height` 个 `dirs`（任意方向，单一/逐条原点） | README：Embree「低端硬件也顺」；示例 100 台 VLP-16 批量；TBB `parallel_for` 多线程 | **✅ 是**（但需自己写 Livox 图案与 `CustomMsg` 那层，见 #2） | **A** |
| 2 | [`ashduwihch/mid360_gazebo_harmonic`](https://github.com/ashduwihch/mid360_gazebo_harmonic) | **MIT**（内置 rmagine = BSD-3） | 2026-10-13（18★） | **Harmonic（`gz-sim8`，CMake 硬钉）** | 模型级 `gz::sim::System` + **rmagine `O1DnSimulatorEmbree`**（CPU BVH 批量）；另有 `ray_backend=physics`（需 `RaycastData`，Harmonic+） | ✅ **读 CSV**（`time,az,ze`，与我们同一份数据/同一套公式） | 200,000 点/秒、10 Hz、**20,000 射线/帧**；代码里每 100 帧打印 **Embree 平均 ms** | **🟡 差 3 处编译级改动 + 契约对齐**（见 §5） | **A** |
| 3 | [`FZ-XiaoXi/Livox-Lidar-Gazebo-Sim-Sensor`](https://github.com/FZ-XiaoXi/Livox-Lidar-Gazebo-Sim-Sensor) | **Apache-2.0** | 2026-10-11（1★） | **Harmonic（`gz-sensors8`/`gz-sim8`）** | 自定义 **`gz::sensors::Sensor`** + `components::RaycastData`（**求交交给物理引擎**） | ✅ 读 CSV（自带 7 种 Livox `scan_mode/*.csv`） | 未声明 | ❌ **不可**（`RaycastData` 在 gz-sim6 不存在 ⇒ 整条路线不成立） | **A** |
| 4 | [`jongbob1918/livox_mid360_gz_sim_ros2`](https://github.com/jongbob1918/livox_mid360_gz_sim_ros2) | **Proprietary（明确禁止复制/再分发）** | 2026-07-16（1★） | **Fortress（README 表：Ubuntu 22.04 + Humble + `ign gazebo`）** | 模型级 `gz::sim::System` + **自己实现的暴力求交**（遍历 ECM 里的 box/plane） | ✅ 读 CSV（滚动窗口，20000/帧） | 20000 点 CPU 求交「复杂世界可能到不了 10 Hz」 | **✅ 技术上是**，❌ **许可上不是** | **A** |
| 5 | [`DHA-Tappuri/livox_ignition`](https://github.com/DHA-Tappuri/livox_ignition) | **无 LICENSE（默认全保留）** | 2025-04-03（1★） | **Fortress（`ignition-gazebo6`，CMake 逐字）** | **GPU `gpu_lidar` 过采样 1440×1440（≈2.07M 条）→ 按 CSV 方向查最近邻索引重采样** + gz protobuf 自定义消息 + ROS 2 桥节点 | 🟡 只能「近似」（角度量化 = 360/1440 = 0.25°） | 未声明 | 🟡 是（Fortress），但**不是 CPU、精度受网格限制** | **A** |
| 6 | [`acodeiro-dev/lidar-livox-mid360`](https://github.com/acodeiro-dev/lidar-livox-mid360) | **无 LICENSE** | 2026-10-13（0★） | Harmonic | `gpu_lidar` + `ros_gz_bridge` + 自写 `livox_converter` 节点转 `CustomMsg` | ❌（规则网格 + 事后转格式） | 未声明 | ❌ 不是 CPU 路线 | **B** |
| 7 | Classic 家族：[`Livox-SDK/livox_laser_simulation`](https://github.com/Livox-SDK/livox_laser_simulation)（**MIT**，348★，2023-11）→ [`LCAS/livox_laser_simulation_ros2`](https://github.com/LCAS/livox_laser_simulation_ros2)（MIT，13★，2025-05）→ `CMU-cabot` fork（MIT，1★，2025-01） | **MIT** | 见左 | **Gazebo Classic** | 每条射线一个 ODE collision（= 我们 2.74 GiB 的根源） | ✅（就是我们的来源） | —— | ❌ 不能直接用；**但 CSV 格式/方向公式/`CustomMsg` 语义是真值来源** | **A** |
| 8 | 上游 Jetty `gz-sim10` 的 `cpu_lidar` | Apache-2.0 | PR #3343 合入 **2026-05-20** | **gz-sim10 only** | `CpuLidar` system + `CpuLidarSensor`，**求交交给 `gz-physics`（`RaycastData`）** | ❌ **规则网格**，`GenerateRays()` 非虚 | 号称「优化过的物理射线」 | ❌ **不可移植**（F1/F2/F3） | **A** |

**旁证（不算候选）**：`ZhuoruiYUN/Sentry-Navigation`（RoboMaster 哨兵、Humble、Livox MID360）与 `TIANN-ENG/px4-mid360-fastlio-sim` 都是**Classic** 路线（前者未逐文件核实是否含 gz 插件）——说明"Rm 圈子里大家也还在 Classic 上受苦"。

---

## §3 逐个候选：源码级证据与裁决

### 3.1 `uos/rmagine` + `rmagine_gazebo_plugins` —— **地基**（BSD-3，Fortress 已测）

**它是什么**：Alexander Mock（Uni Osnabrück）的射线追踪传感器仿真库（ICRA 2023 论文），`rmagine_gazebo_plugins` 是它的 Gazebo 前端。架构是**成对的两个 `gz::sim::System`**：

- **map system**（挂 `<world>`）：把世界几何同步成**一棵常驻 BVH**，几何动了就 **refit**（不重建），支持 `ignore_model` / `ignore_link` 排除自机；通过进程内 `MapRegistry` 按 `map_key` 发布。
- **sensor system**（挂 `<world>`，每个 backend 一个）：用 `components::CustomSensor` 自动发现 `<sensor type="custom" gz:type="rmagine_embree">`，按 `model_type` 打射线，发 `gz.msgs.PointCloudPacked`（单环还发 `LaserScan`）。

**关键证据（A 级，都读过）**：

- **Fortress 支持是写在 CMake 里的、不是口号**：`CMakeLists.txt:27-53` 先试 `gz-sim8`，不中就 `find_package(ignition-gazebo6 REQUIRED)`，并切换 `ignition-plugin1 / transport11 / common4 / math6 / msgs8 / rendering6`；`package.xml:61-62` 还有 `condition="$ROS_DISTRO == 'humble'"` 的依赖。
- **版本差异被压缩到 1 个头文件**：`include/rmagine_gazebo_plugins/gz/gz_compat.hpp` 全文只做一件事——`RMAGINE_GZ_IGNITION_ERA` 为真时把 `GZ_ADD_PLUGIN` 映射到 `IGNITION_ADD_PLUGIN`（与我在本机 F5 的核实完全对上）。
- **任意图案**：`src/gz/sensor_model_config.cpp:157-245`（`LoadO1Dn`）——`<o1dn><scan><rays_file>` 读 YAML `{width,height,rays:{orig,dirs[]}}`，`dirs` 是**任意向量列表**；也支持内联 `<rays><dirs><dir>…`。
- **批量 + 多线程**：`src/rmagine_embree/include/rmagine/simulation/O1DnSimulatorEmbree.tcc` 用 **`tbb::parallel_for(blocked_range3d<…>)`** 一次调用吃下整帧射线。
- **发布字段**：`sensor_model_publish.hpp:158-260` 的 `PointCloudPacked` = `xyz(F32) + ring(U16)`，可选 `normals/obj_id/face_id`；**没有 intensity、没有逐点时间**。

**裁决：REUSE（作为底座）**。它解决的是最难、最不该自己写的那一层（**常驻 BVH + 增量 refit + 多线程批量化 + 任意方向表**），而且许可干净（BSD-3）、Fortress 有官方口径。

**缺口（相对我们）**：①只有 `gz.msgs` 类型，**给不出 `livox_ros_driver2/CustomMsg`**；②没有逐点 `offset_time`；③`sensor type="custom"` 需要 SDF 侧配合，与我们「模型级插件」的用法不同；④`mid360.csv` → `rays.yaml` 需要一次转换（800k×3 float ≈ 10 MB YAML，能跑但难看）。

---

### 3.2 `ashduwihch/mid360_gazebo_harmonic` —— **最接近现成 R1**（MIT，Harmonic）

**它是什么**：把 `Livox-SDK/livox_laser_simulation` 的思路整体搬到新 Gazebo，并用 **rmagine/Embree** 替代 ODE 逐射线碰撞体。`packages/` 下三个组件：`mid360_simulation_plugin_ros2`（**MIT**）+ `rmagine`（BSD-3，trim 到 core+embree）+ `rmagine_gazebo_plugins`（BSD-3，trim 到「Harmonic 的 Embree **collision** map system」）。`THIRD_PARTY.md` 逐条说明了 trim 与许可保留。

**为什么说它"几乎就是 R1"（逐条对上我们的需求）**：

| 我们的需求 | 它的实现（源码位置） |
|---|---|
| `gz::sim::System` | `src/mid360_system.cpp:92-95`（`ISystemConfigure/PreUpdate/PostUpdate`） |
| CPU BVH 批量求交 | `src/mid360_system.cpp:385` `embreeSimulator->simulate(sensorToWorld, embreeResults)`（`O1DnSimulatorEmbree`，TBB 多线程） |
| 任意 CSV 图案 + 滚动窗口 | `csv_scan_pattern.hpp:21-56`（读 `time,az,ze`）+ `mid360_system.cpp` 的 `startIndex` 滚动 |
| 逐点时间（FAST-LIO 去畸变要） | `mid360_system.cpp:480-481` `offsetNs = i*downsample*1e9/point_rate`；`CustomMsg.timebase = 帧首` |
| `line/tag/reflectivity` | `csv_scan_pattern.hpp:50`（`index%4` → line）、`mid360_system.cpp:478-506`（tag `0x10`、refl 100） |
| 直接发 ROS 2（**绕开 `ros_gz_bridge`**，因为 `CustomMsg` 桥不了） | `mid360_system.cpp:187-…` 在插件里 `rclcpp::init` + 建 node/publisher；`PublishCloud()` 同时填 `PointCloud2`（26 B/点：xyz+intensity+tag+line+timestamp）与 `CustomMsg` |
| 自机排除 | `mid360_system.cpp:150` `MapRegistry::AddIgnoredModelEntity(entity)` + RMAGINE map 侧的 ignore |
| IMU | `OnImu()`（`:525-582`）把 gz `IMU` 话题转成 `sensor_msgs/Imu` 发 `/livox/imu` |
| 性能可观测 | `:405-411` 每 100 帧打印 `[MID360] Embree average raycast: X ms for N rays` |

**性能口径**：README 写「**200,000 点/秒，10 Hz，每帧 20,000 条射线**」（= 真机 MID-360 的规格），没有给 ms 数——但代码自带计量，我们接上就能测。

**裁决：ADAPT（首选骨架）**。它把「`gz::sim::System` 里怎么做 Livox」这件事**从头到尾写了一遍**，包括那些不看代码想不到的坑（`offset_time` 用 `point_rate` 推、`output_yaw` 处理 180° 安装、`disableFixedJointLumping` 之类）。**工作量 S–M**，具体改动见 §5。

**为什么不是 REUSE（原样用）**：CMake **硬钉 `gz-sim8`**（`find_package(gz-sim8 REQUIRED)` + `gz-plugin2/gz-transport13/gz-msgs10` + `GZ_ADD_PLUGIN`），而且 `ray_backend=physics` 分支依赖 `components/RaycastData`（Fortress 没有，F1）。

---

### 3.3 `FZ-XiaoXi/Livox-Lidar-Gazebo-Sim-Sensor`（Apache-2.0，Harmonic）

**它是什么**：一个**自定义 `gz::sensors` 传感器**（`gz-sensors8`）+ 一个发现它的 `gz::sim::System`。求交方式是**把 `RaycastData` 组件写进 ECM 让物理引擎算**（`src/livox_lidar_system.cpp:96-152`，注释里逐字写「Gazebo 8 raycasts every non-empty RaycastData component on each step」）。自带 7 种 Livox `scan_mode/*.csv`（HAP/avia/horizon/mid360/mid40/mid70/tele）与 7 个 xacro。

**价值**：①**Apache-2.0**，许可最宽松；②**证明了「自定义 sensor + 逐射线任意方向 + 物理引擎批量求交」这条路在 gz-sim8 上是通的**；③`scan_mode` CSV 全套可参考。

**裁决：ADAPT-for-Harmonic / 不适用 Fortress**。在 Fortress 上整条路不成立（F1）。**但如果哪天升到 Harmonic，这个是比 #2 更"原生"的写法**（走 `gz-sensors` 而不是模型插件），值得留档。

---

### 3.4 `jongbob1918/livox_mid360_gz_sim_ros2`（**专有**，Fortress）—— 结构最像，许可最不能用

**它是什么**：**明确针对 Fortress 验证**（README 支持表：Ubuntu 22.04 / Humble / `ign gazebo`；CMake 里 `gz-sim8→gz-sim7→ignition-gazebo6` 三级探测），模型级 `gz::sim::System`，读 CSV 滚动窗口，**在插件内建 `rclcpp` 节点**直接发 `CustomMsg`(`/livox/lidar`) + `PointCloud2`(`/livox/lidar/points`) + ground-truth odom，还带**逐点时间插值 + 帧内位姿插值**（`scanFraction`，模拟扫描期间的机器人运动，`src/livox_mid360_gz_plugin.cpp:419-474`）——这一点比 #2 还细。

**致命的三条**：

1. **许可**：`LICENSE` 全文 = `Copyright (c) LK Robotics. All rights reserved.` + 「This source code and the bundled scan-pattern data are proprietary. No right to copy, publish, sublicense, or redistribute is granted…」——**不能 vendor、不能抄、不能改**（法务上连"照着重写"都要小心，只能吸收"公开可见的思想"）。
2. **求交只支持 box / plane**：`src/livox_mid360_gz_plugin.cpp:344-357` 只处理 `sdf::GeometryType::BOX` 与 `PLANE`，README 的「已知限项」也直说 mesh/cylinder/sphere 不参与求交 ⇒ **我们的 45,900 面 STL 世界直接 0 命中**。
3. **性能模型是 O(射线×box) 暴力**：`RaycastCollision()` 每条射线遍历所有碰撞体（`:230-306`），没有 BVH ⇒ 30000 射线 × N 个 box 在 Fortress 上到不了 10 Hz。

**裁决：DO-NOT-USE（许可）/ READ-ONLY 参考**。**唯一值得吸收的是它的"帧内运动"模型**（扫描窗口内按 `scanFraction` 插值位姿，每点一个 offset_time）——这是真机 MID-360 的行为，`#2` 是"一帧一个位姿"，我们 R1 若追求保真度可以自己实现这 30 行。

---

### 3.5 `DHA-Tappuri/livox_ignition`（无 LICENSE，Fortress）—— 「过采样 + 重采样」的 Fortress 版

**它是什么**：README 逐字说明了动机——「**fortress では GPU ベースのカスタムレイキャスト機能がないため**，用 `gpu_lidar` 生成高密度扫描，再只抽取匹配 Livox 图案的射线」。实现：`sdf/sample.sdf:16-51` 里 `gpu_lidar` 开 **1440×1440**（≈2.07M 条/帧！），`LivoxLidar.cc:152-159` 把每条 CSV 方向**投影到最近邻网格索引** `_dir_table[i].idx = (v*h_samples+h) % …`，`OnLaserScan()`（`:186-245`）按索引取距离、用极坐标重建点，发 gz protobuf 自定义消息，再由 `livox_ign_ros2_bridge` 节点转成 `livox_ros_driver2/CustomMsg`。

**评价**：这就是我们迁移计划里 **R3 的"穷人版"**（不 fork 渲染器，改成过采样）。**在 Fortress 上确实能出点**，但代价是：①**2M 条 GPU 射线/帧**（llvmpipe 下无法接受，见迁移计划 §4 的无 GPU 裁决）；②角度量化 0.25°，比 MID-360 平均点距（≈0.84°）细，但**边缘目标命中与否会跳动**；③**无许可**。

**裁决：DISCARD**（性能模型与我们的"无 GPU/headless"约束冲突；许可也不明）。**但它的"投影表 + 索引重采样"技巧值得记住**——如果哪天只想在 Fortress 上先"有个能出点的 Livox"，这是最省事的一小时方案。

---

### 3.6 `acodeiro-dev/lidar-livox-mid360`（无 LICENSE，Harmonic）

`gpu_lidar` → `ros_gz_bridge` → 自写 `livox_converter` 节点 → `CustomMsg`（README 的 ASCII 图很直白）。**规则网格 + 事后转格式**，不解决图案问题。**裁决：DISCARD**（仅作"转换节点怎么写"的参考）。

---

### 3.7 Classic 家族 —— 我们的来源，也是 CSV/语义的真值

| 仓库 | 许可 | 活动 | 说明 |
|---|---|---|---|
| `Livox-SDK/livox_laser_simulation` | **MIT** | 348★，2023-11 最后 push | 原版（ROS 1）。`livox_ode_multiray_shape.cpp` 的 `AddRay()` 每条射线建一个 `ODECollision` —— **我们 2.74 GiB 的根因** |
| `LCAS/livox_laser_simulation_ros2` | **MIT** | 13★，2025-05 | ROS 2 移植（Classic） |
| `CMU-cabot/livox_laser_simulation_ros2` | **MIT** | 1★，2025-01 | 同上的另一份 fork |

**裁决：不能直接用（Classic），但它是我们"必须逐字对齐"的真值**：CSV 列语义、`zenith−90°` 的方向公式、`offset_time/line/tag` 的取值、滚动窗口语义，都在这里。**F7 已经确认候选 #2 与它同源同公式**，这是 #2 能被我们采信的最强理由。

---

### 3.8 上游 Jetty `cpu_lidar` —— 架构参考，**不可移植**

- 合入：`gazebosim/gz-sim` **PR #3343**，commit `839e621`，**2026-05-20**。
- 实现：`src/systems/cpu_lidar/CpuLidar.cc`（283 行）—— 生成规则网格射线 → 写 `components::RaycastData` + `NeedsRaycast` → 物理引擎算 → 读回 `results` → `SetRaycastResults()`。
- **两条硬阻断**：①`RaycastData` 在 `gz-sim6/7` 不存在（F1）；②`GenerateRays()` 非虚（F3）。
- 另外 Jetty **没有 Humble 的 `ros_gz` 包**（迁移计划 §2.1 实测）。

**裁决：只借"分工"（system 负责图案与发布、引擎负责求交），不借代码**。

---

## §4 裁决表：REUSE / ADAPT / WRITE + 工作量

| 方案 | 做法 | 工作量 | 具体 gap | 风险 |
|---|---|---|---|---|
| **A（推荐）** | **`ashduwihch` 的 MIT 插件（骨架 + CSV + CustomMsg）+ 上游 `uos/rmagine_gazebo_plugins`（BSD-3，Fortress 已测）做 map/BVH** | **S–M（1–4 人天）** | ①CMake 换 `ignition-*` 包名（照上游 `CMakeLists.txt:27-53` 的写法）②注册宏加 `#ifdef`（F5，1 行）③删/`#ifdef` 掉 `ray_backend=physics`（`RaycastData` 在 Fortress 不存在）④map system 用上游（visual 建 BVH）或移植它的 collision 版（我们世界两者同一 STL，等价）⑤`<csv_file>` 指到我们 `scan_mode/mid360.csv` ⑥`samples 20000→30000` + `point_rate 200000→300000` ⑦`frame_id`/话题/字段对齐 ⑧自机排除 ⑨30k@10 Hz 实测 + 与 Classic 几何对拍 | 中低：Embree 4 需联网下载（apt 只有 3.12）；mesh 加载走 assimp（本机已装） |
| **B（许可最干净）** | **只用上游 `rmagine`/`rmagine_gazebo_plugins`（BSD-3）+ 自己写 ~200–300 行 `Mid360System`**（照 #2 的公开结构，不抄它的代码文本） | **M（3–5 人天）** | 需要自己实现：CSV 滚动窗口、`offset_time` 推导、`CustomMsg` 填充、IMU 转换、`output_yaw` 那类安装补偿 | 中：Fortress 侧由上游背书，自己写的部分全在自己手里 |
| **C（兜底）** | 自研 R1（我们已验证的 Bullet `btBvhTriangleMeshShape` + CSV） | **L（1–2 周）** | BVH 常驻/增量 refit、动态模型同步、多线程、`CustomMsg`、自机排除、时间模型，**全部从零** | 高：等于把 #1/#2 已经踩过的坑再踩一遍 |
| D | R3 fork `gz-rendering`（立方图重采样） | **L+** | 3 个包、精度损失 | 已否决（迁移计划 §9.4.6） |
| E | 升 Harmonic 用 `RaycastData`（#3 路线） | **XL** | `gzharmonic` 会 `Remv` 掉 Classic（迁移计划 §2.2 实测） | 已否决 |

**一句话建议**：**先花半天做 A 的"编译打通"**（改 CMake + 宏 + 编掉 physics backend，跑它的 demo world），
**打通后再花 1–2 天做契约对齐与 30k 实测**；如果 A 在 Fortress 上出现不可预期的 API 坑，再退到 B。

---

## §5 去 Fortress 的具体改动清单（A 方案，文件级）

> 只列「必须动什么」，不写代码。**这些是适配项，不是设计项**——设计已经被 #1/#2 做完了。

| # | 位置 | 改什么 | 依据 |
|---|---|---|---|
| 1 | `CMakeLists.txt` | `gz-sim8`→`ignition-gazebo6`；`gz-plugin2`→`ignition-plugin1`；`gz-transport13`→`ignition-transport11`；`gz-msgs10`→`ignition-msgs8`；target 名 `gz-sim8::gz-sim8`→`ignition-gazebo6::ignition-gazebo6` | 上游 `rmagine_gazebo_plugins/CMakeLists.txt:27-53` 就是这么写的 |
| 2 | `mid360_system.cpp:632-641` | `GZ_ADD_PLUGIN` / `GZ_ADD_PLUGIN_ALIAS` 加 `#ifndef … #define … IGNITION_ADD_PLUGIN` 兜底 | 本机 F5：Fortress 的 `gz/plugin/Register.hh` 只有 `IGNITION_ADD_PLUGIN` |
| 3 | `mid360_system.cpp:21,180-185` + PreUpdate 的 physics 分支 | 删掉或 `#if GZ_SIM_VERSION>=8` 包住 `components/RaycastData`（Fortress 无此组件） | F1/F2 |
| 4 | map system 来源 | 用上游 `rmagine_embree_map_system`（**visual** 建 BVH，`ignore_model` 排除自机）；或把 #2 的 collision 版移植过来（它改成了 **collision** 建 BVH） | 我们世界 visual/collision **是同一个 STL**，两者等价；但**动态物体的 visual/collision 一致性**要实测 |
| 5 | `model.sdf` 的 `<plugin>` 块 | `<csv_file>` 指向 `…/livox_laser_simulation_RO2/scan_mode/mid360.csv`（列格式/公式已验证一致，F7）；`samples=30000`；`point_rate=300000`（**否则 `offset_time` 会跨 150 ms > 100 ms 帧**） | F7 + `mid360_system.cpp:480` |
| 6 | 输出契约 | `frame_id` = 我们的 `livox_frame`（或按 `output_yaw` 补偿安装）；`/livox/lidar` 走 `CustomMsg`；`/livox/lidar/pointcloud` 的**字段表**要核对：我们发 **xyz+rgb**，它发 **xyz+intensity+tag+line+timestamp** ⇒ 若有下游用 `pcl::fromROSMsg<PointXYZRGB>` 会炸，用 `PointXYZI`/`PointXYZ` 则无碍 | 我们 `livox_points_plugin.cpp:83-98,271` |
| 7 | 自机排除 | `MapRegistry` 的 `ignore_model` / `AddIgnoredModelEntity`，并把机器人自身从 BVH 里剔掉（否则每帧自击） | `mid360_system.cpp:150` |
| 8 | 验收 | ①30k 点/10 Hz ②同一位姿下与 Classic 点云几何对拍（本仓已有 `tools/scripts/regress/linefit_replay_probe.py`、`robot11_livox_fov.py` 一类探针）③RSS/RTF 不劣于 Classic（3.2 GiB / 0.32–0.46） | 迁移计划 §7.4 出口判据 |

**明确不用动的**：`gpu_lidar`/SDF `<lidar>` schema、`ros_gz_bridge`（PointCloudPacked 路线才需要，我们走进程内 `rclcpp` 就不需要）、`gz-physics`、渲染器。

---

## §6 许可证影响（vendor 还是 depend）

| 组件 | 许可 | 能不能 vendor | 义务/注意 |
|---|---|---|---|
| `ashduwihch/mid360_simulation_plugin_ros2` | **MIT** | ✅ | 保留版权与许可文本；无专利条款 |
| `uos/rmagine` / `rmagine_gazebo_plugins` | **BSD-3-Clause** | ✅ | 保留版权/条件/免责三段；**不得用作者名背书**；学术使用请引用 ICRA 2023 论文（README 给了 BibTeX） |
| `FZ-XiaoXi/…` | **Apache-2.0** | ✅ | 保留 `LICENSE` + 改过的文件要有变更声明 + `NOTICE`；含**专利授权**（对我们更安全） |
| `jongbob1918/…` | **Proprietary** | ❌ **绝对不行** | 「No right to copy, publish, sublicense, or redistribute」——**连把它的代码贴进我们的仓都不行**；只能读思路 |
| `DHA-Tappuri/…`、`acodeiro-dev/…` | **无 LICENSE** | ❌ | 默认**全保留**（无授权即无许可），技术上"能用"≠法律上"能用" |
| `mid360.csv` 图案数据 | 我们仓内已有（来自 Livox 官方 `scan_mode`） | —— | 注意 #4 的 README 明确写了「bundled scan-pattern 的**外部再分发依据未整理**」——**图案数据本身的再分发是另一个法务问题**，与我们已有这份无关，但**不要把别人的 CSV 再分发** |
| Embree 4（rmagine 首次编译自动下载） | Apache-2.0 | 构建期下载 | 离线/CI 环境要预置；apt 的 `libembree-dev` 是 **3.12**，rmagine 要 **4**（`RMAGINE_EMBREE_LOCAL_VERSION 4`，下载 4.4.0） |
| TBB / Assimp / yaml-cpp | 各自宽松 | —— | 本机 assimp 已装 |

**结论**：**A 方案许可完全干净**（MIT + BSD-3，都是可 vendor 的宽松许可）；**B 方案更干净**（只剩 BSD-3）；**C 自研**无许可问题但最贵。

---

## §7 「CPU 批量射线求交当 LiDAR 仿真器」是不是常规做法？

**是，而且是这个问题的标准答案。** 四条独立证据：

1. **PyBullet 官方就带这个用法**：`rayTestBatch` 是内建 API，仓库里直接有示例 [`examples/pybullet/examples/batchRayCast.py`](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/batchRayCast.py)（本沙箱 HTTP 200 已核实存在），社区教程也就是拿它模拟 2D/3D lidar。⇒ **"一次提交几千条射线、拿回距离数组"是物理引擎的常规能力**，不是奇技淫巧。
2. **有专门的论文级工作**：`rmagine`（**ICRA 2023**，DOI `10.1109/ICRA48891.2023.10161388`）的整篇贡献就是「在三角面地图上用 CPU/GPU 光追做距离传感器仿真」，README 的示例是**一次仿真 100 台 VLP-16**。⇒ 学术界把它当一个正经方向在做。
3. **上游 Gazebo 自己也在 2026 年补了 CPU lidar**（`gz-sim` PR #3343，2026-05-20），说明"要一条不依赖渲染器的 LiDAR 路径"是**官方认可的需求**——只是它选择了"让物理引擎批量算"，而 Fortress 没有那个组件（F1）。
4. **我们自己的实测**：45,900 面真实场地 + 真 30k CSV 图案，Bullet BVH **1 线程 12.5–29.4 ms**、8 线程 3.2–7.4 ms，**100 ms 预算下余量 3–30×**（迁移计划 §9）。

**所以"普遍性"的准确回答**：
- **架构层面**：极其普遍（引擎批量射线、离线点云生成、PyBullet/MuJoCo/rmagine/Jetty 都是），**没有创新风险**；
- **工程层面**：真正稀缺的不是"会写"，而是"**在 Fortress 的 `gz::sim` 里把 CSV 图案、逐点时间、`CustomMsg`、自机排除、BVH 常驻/refit 全部接对**"——而这恰好已经有 2 个开源实现替我们做完了（一个 MIT、一个 BSD-3）；
- **结论**：**自研的"必要性"很低，"普遍性"很高**——高到应该直接复用。

---

## §8 诚实清单：没抓到 / 没验证

**抓不到（网络侧）**：

- `lxrobotics.com/blog/livox-mid-360-gazebo/`（搜索命中的 "How to simulate a Livox MID-360 using Gazebo"）：DNS 解析到非公网 IP，**本沙箱拒绝访问**，只看到搜索标题，**内容未读**。
- `arxiv.org/abs/2209.13397`（rmagine 论文预印本）：同上，**摘要未读**；因此论文里的具体性能数字**没有引用**，只用仓库 README 的说法。
- GitHub issue 正文：`gz-sim#1958`（Custom lidar sensor）、`gz-sim#2645`（Build my own Lidar Sensor with custom scanning shape）、`gz-sensors#26`（CPU lidar (no rendering)）——**只拿到标题**，正文未能完整抓取，**结论未逐句核对**（`#1958/#2645` 与迁移计划 §3.6 的记录一致）。
- `rmagine_gazebo_plugins` 的 Wiki：未抓。

**没验证（工程侧）**：

- **没有编译、没有运行任何候选**（本任务只读代码）。所以「#2 在 Fortress 上能编过」是**基于源码 + 本机头文件的推断**，**不是实测**——这是 A 方案第一步就要打的靶。
- **#2 的 Embree 实测 ms 未知**（它只在自己机器上跑过 20k/10 Hz）；**30k@10 Hz 未验**。
- **rmagine 的 mesh 加载**（assimp / `gz-common` mesh loader）对我们这份 **mm 单位、scale 0.001、45,900 面**的 STL 是否 1:1 正确，**未验**（我们的 mm/m 单位 bug 在 Bullet 上踩过一次，见迁移计划 §9.2）。
- **#2 的 `PointCloud2` 字段（intensity）与我们下游**（期望 xyz+rgb，PCL 转换）**是否兼容**：只核对了字段列表，**没有逐个消费者核对**。
- **`mid360-real-centr.csv` 与我们的 `mid360.csv` 是否逐行相同**：只对了表头/zenith 范围（一致）与首行（azimuth 差 180°），**疑似同一数据的不同起点或朝向约定**，未逐行 diff。
- **#2 的 collision-map 与上游的 visual-map 在"动态物体 + 只有 visual 的装饰物"上的差异**：未验。
- `ZhuoruiYUN/Sentry-Navigation`：只看了 README，**未逐文件确认**它是否自带 gz 插件（大概率是 Classic）。
- **Fortress + `rmagine` 的组合没有 CI 证据**：上游 README 表写「Humble / Fortress」已测，但**我没找到对应的 CI 日志**（只有 rmagine 本体有 `ros2-humble` workflow）。

---

## §9 参考链接（按重要性）

**可复用（读源码）**

1. `ashduwihch/mid360_gazebo_harmonic` — <https://github.com/ashduwihch/mid360_gazebo_harmonic>（MIT；`packages/mid360_simulation_plugin_ros2/src/mid360_system.cpp`）
2. `uos/rmagine_gazebo_plugins` — <https://github.com/uos/rmagine_gazebo_plugins>（BSD-3；`src/gz/rmagine_embree_sensor_system.cpp`、`src/gz/sensor_model_config.cpp`、`include/.../gz/gz_compat.hpp`）
3. `uos/rmagine` — <https://github.com/uos/rmagine>（BSD-3；`src/rmagine_embree/include/rmagine/simulation/O1DnSimulatorEmbree.tcc`）

**Fortress 侧参考（许可受限）**

4. `DHA-Tappuri/livox_ignition` — <https://github.com/DHA-Tappuri/livox_ignition>（无许可；Fortress + `gpu_lidar` 过采样重采样）
5. `jongbob1918/livox_mid360_gz_sim_ros2` — <https://github.com/jongbob1918/livox_mid360_gz_sim_ros2>（专有；Fortress + 暴力 box/plane 求交 + 帧内运动插值）

**上游 / 先例**

6. `gz-sim` Jetty `cpu_lidar`（PR #3343，2026-05-20）— <https://github.com/gazebosim/gz-sim/commit/839e621c852df43b8092d103f304779b47b91b61>
7. `gz-sensors10` `CpuLidarSensor.hh` — <https://github.com/gazebosim/gz-sensors/blob/gz-sensors10/include/gz/sensors/CpuLidarSensor.hh>
8. `gz-sensors` issue #26「CPU lidar (no rendering)」— <https://github.com/gazebosim/gz-sensors/issues/26>
9. `gz-sim` issue #2645「Build my own Lidar Sensor with custom scanning shape」— <https://github.com/gazebosim/gz-sim/issues/2645>
10. PyBullet `batchRayCast.py` — <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/batchRayCast.py>
11. rmagine ICRA 2023（DOI `10.1109/ICRA48891.2023.10161388`）— <https://ieeexplore.ieee.org/document/10161388>

**本仓内相关**

12. [`gazebo_sim_migration_plan.md``gazebo_sim_migration_plan.md` —— Humble↔Fortress、`gpu_lidar` 图案裁决、无 GPU 裁决、R0/R1/R3 决策表、**我们自己的 Bullet BVH 实测**（§9）
13. `src/simulation/rm_simulation/livox_laser_simulation_RO2/` —— 我们 Classic 插件的 CSV 读取、方向公式、滚动窗口（本轮 F7 的对拍基线）

---

## §10 编译与运行实测 —— A 方案在 Fortress 6.18 上打通（2026-10-18）

> **本节是 §8「没验证」清单的第一手回填。** 上一轮只读代码，本轮**真编译、真运行、真对拍**。
> 全部工作在未跟踪草稿目录 `.tmp_gzlivox/`（匹配 `.tmp_*/`）；**仓库内除本节外零改动**。
> **一句话结论：GO。** `ashduwihch` 的 MIT 插件在 `ignition-gazebo6` 6.18 上**能编过、能跑、能发 30k@10 Hz、几何与 Bullet 参考一致到 49 µm**，
> 代价是 **4 类源码/CMake 改动**（CMake 包名、注册宏、physics backend `#if`、**Console 宏**）+ **2 项资产修正**（SDF 必须 ≤1.9、`model.config` 的空 `<sdf>` 必须补文件名），
> 而**不是** §5 原先估计的 3 处 —— 多出来的两项见 §10.4.5 / §10.4.6 / §10.4.7。

### 10.0 实测环境（一手）

| 项 | 值 |
|---|---|
| Gazebo | `libignition-gazebo6` **6.18.0**（`ign gazebo --versions` = 6.18.0） |
| 依赖版本 | `ignition-plugin1` 1.4.0 / `ignition-msgs8` 8.7.1 / `ignition-transport11` 11.4.2 / `ignition-common4` 4.9.0 / `sdformat12` 12.9.0 |
| Embree | **4.4.0**（apt 无 4.x；rmagine 首次 configure 用 `FetchContent` 从 GitHub 下载预编译 `embree-4.4.0.x86_64.linux.tar.gz`，**成功**，13 s 完成整包构建） |
| 机器 | 28 核 / 31 GiB（可用 16 GiB）/ 无 GPU |
| ROS | Humble；`livox_ros_driver2` 由本仓 `install/` 提供（`MID360_HAS_LIVOX_CUSTOM_MSG=1` 生效） |
| 抓取到的候选（git rev） | `ashduwihch/mid360_gazebo_harmonic` @ **`497149a`**（v0.2.0）；`uos/rmagine_gazebo_plugins` @ **`c6dc04e`**；候选内置 `rmagine` **2.4.3** |
| 世界/图案 | 真 `RMUC2026.stl`（45,900 面，mm，`scale 0.001`，z 抬 1.6413436）+ 我们自己的 `scan_mode/mid360.csv`（800,000 采样） |
| 传感器位姿 | `(10.925, 2.525, 0.26)`（= `.tmp_gzsim2/cpu_ray_bvh` 的 RMUC2026 出生点），`output_yaw=0`，`min/max_range=0.1/40` |

### 10.1 底座 `uos/rmagine_gazebo_plugins`：**原样编过，README 的 Fortress 口径属实**

**未改一行**（连 CMake 都没改）直接 `colcon build`：

```
Starting >>> rmagine_gazebo_plugins
Finished <<< rmagine_gazebo_plugins [9.19s]
```

- 它的 `CMakeLists.txt:29-55` 自动探测：`find_package(gz-sim8 QUIET)` 失败 → `find_package(ignition-gazebo6 REQUIRED)` 命中，CMake 输出 `-- Looking for ignition-gazebo6 -- found version 6.18.0`；`RMAGINE_GZ_IGNITION_ERA=1` 自动置位。
- 产出 `librmagine_embree_map_system.so` + `librmagine_embree_sensor_system.so`，`ldd` 里 **11 个 `libignition-*`** 依赖，无 `libgz-*`。
- `rmagine::optix` / `rmagine::vulkan` 找不到 → 自动跳过对应 backend（只有警告，不是错误）。
- `rmagine` 本体（无任何 gz 依赖）13.4 s 编完；**Embree 4 的"首次构建联网下载"在本机可行**（CI 需缓存，见 §10.9）。

⇒ **§2 表格第 1 行"Fortress+Humble 直接可用 ✅"从 B 级（README）升到 A 级（本机实测）**，也顺带证伪了 §8 最后一条"Fortress + rmagine 没有 CI 证据"的疑虑（至少本机成立）。

### 10.2 最小补丁集（A 方案，逐条 file:line）

> 全部补丁都在 `.tmp_gzlivox/` 的克隆里；`git diff` 合计 **+82 / −18 行**，涉及 5 个文件
> （其中 `rmagine_embree_map_system.cpp` 那 8 行**只是计时打印**，不属于功能补丁，见 §10.3 的"BVH 构建"一行）。
> 下表 "原" 指 `ashduwihch@497149a` 的原文，"改" 指本次实际动手。

| # | 文件:行 | 改什么 | 依据 |
|---|---|---|---|
| **P1** | `packages/rmagine_gazebo_plugins/CMakeLists.txt:8-11, 20-24` | `gz-sim8`→`ignition-gazebo6`；`gz-plugin2`→`ignition-plugin1 COMPONENTS register`；`gz-common5 COMPONENTS graphics`→`ignition-common4 COMPONENTS graphics`；target 名同步（`ignition-common4::ignition-common4-graphics` 而不是 `::graphics`） | 本机只装了 `ignition-common4`（**不是** `gz-common5`，§5 清单里没点明这条） |
| **P2** | 同上 `CMakeLists.txt:27-28` | `RMAGINE_GZ_IGNITION_ERA=0` → `=1` | 让 `gz_compat.hpp` 选 `IGNITION_ADD_PLUGIN` |
| **P3** | `packages/rmagine_gazebo_plugins/include/.../gz/gz_compat.hpp:12, 18-25` | **新增**：`#include <gz/common/Console.hh>` + `#if RMAGINE_GZ_IGNITION_ERA` 下把 `gzerr/gzwarn/gzmsg/gzdbg` 映射到 `ignerr/ignwarn/ignmsg/igndbg` | ⚠️ **这是 §5 清单漏掉的第二处 C++ 层改动**，见 §10.4.5 |
| **P4** | `packages/mid360_simulation_plugin_ros2/CMakeLists.txt:14-18, 26-30` | `gz-sim8/gz-plugin2/gz-transport13/gz-msgs10` → `ignition-gazebo6/plugin1/transport11/msgs8`（target 名同步，补 `ignition-plugin1::register`） | §5 第 1 项 |
| **P5** | 同上 `CMakeLists.txt:37-42` | 新增 `target_compile_definitions(... RMAGINE_GZ_IGNITION_ERA=1 MID360_HAS_PHYSICS_BACKEND=0)` | 注册宏开关 + 关掉 physics backend |
| **P6** | `src/mid360_system.cpp:21-25` | `#include <gz/sim/components/RaycastData.hh>` 用 `#if !defined(MID360_HAS_PHYSICS_BACKEND) …` 包住 | F1 |
| **P7** | 同上 `:38` | 新增 `#include <rmagine_gazebo_plugins/gz/gz_compat.hpp>` | 复用 P3 的宏 |
| **P8** | 同上 `:160-168` | `ray_backend=physics` 分支：Fortress 下改为 `gzerr` + `return`（明确报"需要 gz-sim8+"），不再静默 | F1/§5 第 3 项 |
| **P9** | 同上 `:192-198` / `:257-266` / `:308-321` / `:328-347` | 四处 physics 路径（初始 `RaycastData` 组件、PreUpdate 清空、PreUpdate 填 ray、PostUpdate 读结果）全部 `#if` 掉 | 同上 |
| **P10** | 同上 `:652-663` | `GZ_ADD_PLUGIN` / `GZ_ADD_PLUGIN_ALIAS` → `RMAGINE_GZ_ADD_PLUGIN` / `…_ALIAS` | F5（§5 第 2 项） |
| **P11** | SDF（**新写的** `mid360_rmuc.sdf`，改的是配置不是代码） | `<csv_file>` 指到 `src/simulation/rm_simulation/livox_laser_simulation_RO2/scan_mode/mid360.csv`；`samples 30000`；`point_rate 300000`；`min/max_range 0.1/40`；`frame_id livox_frame`；`output_yaw 0`；`publish_pointcloud_type 3` | F7 + §5 第 5/6 项 |

**没有动的**（比预期少）：
- `components/RaycastData` 之外的 `gz::sim`/`gz::math`/`gz::msgs`/`gz::transport` API **一处都没改**——`worldPose()`、`Model::LinkByName()`、`ISystemPostUpdate` 签名、`<sensor type="custom">` 在 gz-sim6 全部存在（F4 复核通过）。
- **map system 的 C++ 一行没改**（它建 BVH 走 `<collision>`，我们世界 visual≈collision，等价；§5 第 4 项不需要动手）。
- `ros_gz_bridge` 不需要（进程内 `rclcpp` 直发）。

### 10.3 实测：headless（`-s`，无 GUI）

命令形态：`ign gazebo -r -v 3 -s <world>`，`HOME`/`XDG_*` 全部重定向到 `.tmp_gzlivox/home`（**必须**：本机 `~/.ignition` 是 `drwxr--r--` 只读，正是已知那个"GUI 启动即 SIGKILL 服务器"的坑）。
订阅端是自写探针（`.tmp_gzlivox/tools/mid360_probe.py`），200 帧。

| 指标 | 实测值 |
|---|---|
| **是否发布** | ✅ `/livox/lidar` = `livox_ros_driver2/CustomMsg`（`publish_pointcloud_type=3`）；同时 `/livox/imu` = `sensor_msgs/Imu` 实测 **197.4–200.1 Hz** |
| **频率（仿真）** | **10.0000 Hz**（200 帧 header 戳的线性回归，误差 <1e-12） |
| **频率（墙钟）** | **9.87–9.91 Hz**（3 次运行：9.88 / 9.91 / 9.87） |
| **每帧点数** | `min 4979 / median 5074 / max 5244`（**30000 条射线里只有有效回波被发**，命中率 16.7–17.3%） |
| **frame_id** | `livox_frame`（可配） |
| **`offset_time` 跨度** | 每帧 `77.5–81.6 ms`；全跑最大 `offset_time = 99,996,667 ns`（= 29999/300000，**≤100 ms ✅**）；**严格单调 ✅** |
| `line` 字段 | 恒为 `{0,1,2,3}`（`index % 4`）✅ |
| **Embree 逐帧求交** | **0.464–0.489 ms / 30000 rays**（插件自带计量，每 100 帧打印一次；6 次采样） |
| **BVH 构建** | assimp 解析 STL + 单 mesh Embree BVH = **14.4–14.7 ms**（冷文件系统首跑 20.6 ms）；顶层 scene commit = **0.024–0.032 ms**；bootstrap 合计 **15.5 ms** |
| **CSV 解析** | **204–207 ms**（800,000 行，`LoadScanPattern`，一次性） |
| **启动 → 首帧点云** | **2.03 s** |
| **server RSS** | **213.3 MB 稳态**（`VmHWM 213,304 kB`，47 线程），运行 70 s 不增长 |
| **CPU** | gzserver **36–38%**（≈0.37 核；含 physics + 求交 + DDS） |
| **RTF** | **0.985–0.993**（`/world/mid360_rmuc/stats`：iterations 3804→4103，RTF 0.985/0.989/0.993/0.993） |
| 泄漏/残留 | 每次运行结束 `leftovers: (none)`（只按自己 PID 树 `kill`，无 `pkill`） |

**对照 Classic（§1 的痛点行）**：RSS **3.2 GiB → 213 MB（−93%）**；RTF **0.32–0.46 → 0.99（+2.4×）**；2.74 GiB 的 ODE 逐射线 collision 变成**常驻一棵 Embree BVH（15 ms 建一次，0 refit）**。

### 10.4 实测：GUI（Xvfb `:98`，1280×800，llvmpipe）与三个"Fortress 化"资产坑

GUI **跑得起来**（上一轮有 agent 死在 GUI 上，所以这次用硬超时 + 分进程计量）：

| 指标 | 值 |
|---|---|
| 墙钟频率 | **9.77 Hz**（200 帧 / 20.37 s） |
| RTF | **0.962 / 0.992 / 0.9999 / 0.9999** |
| 进程 RSS | gzserver **197 MB** / `ign gazebo gui` **584 MB** / Xvfb 70 MB ⇒ 合计 **843 MB** |
| 进程 CPU | gzserver **35%**；GUI **758%**（llvmpipe 吃 7.6 核）；Xvfb 3.5% |
| 点云契约 | 与 headless 完全一致（同 200 帧、同 median 5074、同 offset_time） |

> 注意：本世界**没有 camera/gpu_ray 传感器**，所以 GUI 只是"多画一个视图"，对 RTF 影响 ≈0。真正会掉 RTF 的是**渲染类传感器**（§迁移计划 §4 的无 GPU 裁决），与本节结论不冲突。

#### 10.4.5 新增坑①：`gz-common4` **没有 `gzerr/gzwarn/gzmsg` 宏**（推翻 F5"唯一必改是注册宏"）

原 `mid360_system.cpp` 用 `gzerr/gzwarn/gzmsg` 输出。Fortress 的 `/usr/include/ignition/common4/gz/common/Console.hh` **只定义 `ignerr/ignwarn/ignmsg/igndbg`**（`gz*` 拼法是 gz-common5+ 才加的）。首轮编译因此直接失败 8 处：

```
mid360_system.cpp:120:7: error: 'gzerr' was not declared in this scope
mid360_system.cpp:240:5: error: 'gzmsg' was not declared in this scope; did you mean 'ignmsg'?
mid360_system.cpp:276:9: error: 'gzwarn' was not declared in this scope; did you mean 'ignwarn'?
```

⇒ **P3 补丁**（4 个 `#define`）。这是 §0.1 F5 的**修正**：Fortress 化的**必改 C++ 项是 2 个不是 1 个**（注册宏 + Console 宏），前者 3 行、后者 4 行。
好消息：把它放进 `gz_compat.hpp` 这个"版本差异隔离头"里，与我们/上游的既有设计一致，没有扩散。

#### 10.4.6 新增坑②：**SDF 版本 1.10 在 Fortress 上整份世界加载失败**

候选的 `worlds/mid360_test.sdf` 与 `models/mid360/model.sdf` 都是 `<sdf version="1.10">`（Harmonic 写法）。`sdformat12` 只能到 **1.9**：

```
Error [Converter.cc:156] Unable to convert from SDF version 1.10 to 1.9
```
实测：把我们的世界改成 `1.10` 后，**一帧点云都不出**（probe `frames_received: 0`）；改回 `1.9` 立刻恢复。⇒ 迁移清单要加一条：**所有 SDF 必须 ≤1.9**（`ign sdf -k` 可预检）。

#### 10.4.7 新增坑③：**本仓 `RMUC2026_world/model.config` 的 `<sdf>` 是空的，Fortress 直接拒收整个世界**

本仓 `src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world/model.config` 里写的是：

```xml
<sdf version="1.7"></sdf>
```

Classic 容忍（它按目录名找 `model.sdf`），`sdformat12` **不容忍**：

```
Error [parser.cc:1183] Failure to detect an sdf tag in the model config file for model: RMUC2026_world
Error Code 13: .../include[0]/uri:...: Unable to resolve uri[model://RMUC2026_world] to model path [.../meshes/RMUC2026_world] since it does not contain a model.config file.
```

修法是一行：`<sdf version="1.9">model.sdf</sdf>`。本次为了**不动仓库资产**，在 `.tmp_gzlivox/models/RMUC2026_world/` 放了一份 `model.sdf` **逐字节相同（sha1 `674e8e8…`）** 的副本 + 合法 `model.config`，`meshes/` 用软链指回仓库。
⇒ **P2 必须先修这个 `model.config`（1 行）**，否则 Fortress 里连世界都进不去——这是"资产层"的阻断，与传感器插件无关。

### 10.5 几何对拍：与 Bullet BVH 参考**逐束一致**（最强证据）

对拍对象 = `.tmp_gzsim2/cpu_ray_bvh`（Bullet `btBvhTriangleMeshShape`，**同一份 STL、同一份 CSV、同一套方向公式、同一个位姿**，独立实现）。
对拍方法：把插件发的点归一化成方向，在 CSV 图案里反查出它对应的**射线编号**（匹配 dot = 1.00000000，索引严格单调），再用编号去查 Bullet 的逐射线距离——**这样绕开了"两套窗口起点是否对齐"的任何假设**。

| 帧（CSV 窗口起点） | 插件点数 | Bullet 命中数 | 命中集合一致 | \|Δ距离\| p50 | p90 | p95 | p99 | **max** | 系统偏差（均值） |
|---|---|---|---|---|---|---|---|---|---|
| frame 0（row 0） | 5009 | 5009 | **5009/5009** | 2.55 µm | 5.05 µm | 5.79 µm | 15.9 µm | **48.5 µm** | +0.7 µm |
| frame 1（row 30,000） | 5112 | 5112 | **5112/5112** | 2.56 µm | 5.12 µm | 5.98 µm | — | **68.9 µm** | +0.8 µm |
| frame 5（row 150,000） | 5181 | 5182 | 29999/30000（**1 束**） | 2.63 µm | 5.09 µm | 5.82 µm | — | **49.4 µm** | +0.7 µm |

- 差异量级 = **float32 的 1e-7 相对精度**（26 m 处 1 ulp ≈ 2 µm ⇒ 实测 2.5–69 µm 完全解释得通），**没有任何系统性偏移**。
- **没有 0.1 m 射线起点 bug**：`min_range=0.1` 只是 rmagine 的距离下限，`orig={0,0,0}`，与 Classic 修好后的语义（`point = (minDist+range)·axis`）数值等价；Bullet 参考也是从原点起算，三者对齐。
- frame 5 那 **1/30000** 的差异是一束**擦边射线**（方向 z=−0.056，几乎水平掠过墙顶棱）：Embree 判 miss、Bullet 判命中（4.604 m）。30k 束里 1 束、且是几何退化情形 ⇒ 不构成阻塞。若要更严格，可给 `Embree` 打开 `RTC_SCENE_FLAG_ROBUST`/提高 build quality。
- **单位/朝向/缩放全部正确**：插件点云的 `z ∈ [−0.26, +1.27] m`、`range p01/p50/p95/max = 2.17 / 3.74 / 7.41 / 26.64 m`，与 Bullet 参考**同分布同数值**（参考自己打印的 `p01=2.17 p05=2.49 p50=3.74 p95=7.41 max=26.64`）。**mm→m 那个坑在 Embree 这条路上没有复发**（scale 0.001 与 z 抬升都由 SDF `<scale>`/`<pose>` 正确进入 BVH）。
- **没有跑 Classic 本体**：Classic 侧要拉起整套 `bringup_sim.launch.py`（gzserver + 机器人 + 世界），且它的真相早已由迁移计划 §9 和 `.tmp_gzsim2/cpu_ray_bvh` 锚定；用**同 CSV 同姿态的 Bullet BVH 参考**做 oracle 比"和 Classic 比"更强（逐束、无 ODE 数值噪声、可复算）。**这是本轮唯一"用参考代替真机"的地方，明确记为未验证项。**

### 10.6 与 Classic 的契约差异（P2 必须逐条决定的 6 件事）

| # | 项 | Classic（我们现在的） | 候选插件 | 处理建议 |
|---|---|---|---|---|
| 1 | `/livox/lidar` | `CustomMsg` | `CustomMsg` ✅ | 直接可用 |
| 2 | `PointCloud2` | **另一个话题** `/livox/lidar/pointcloud`，字段 **xyz+rgb**，`setPointCloud2FieldsByString(2,"xyz","rgb")` | **同一话题** `/livox/lidar`，字段 **x,y,z(f32) + intensity(f32) + tag(u8) + line(u8) + timestamp(f64)**，`point_step=26`（实测） | `publish_pointcloud_type` 是**单选**（2 或 3），**不能同时发两种**。若 nav 侧仍要 `PointXYZRGB`：要么改下游成 `PointXYZI`，要么给插件加"同时发两路 + 第二路 rgb"（约 15 行） |
| 3 | `frame_id` | `raySensor->Name()`（如 `livox_lidar`） | 可配，我们设 `livox_frame` | 配成与 TF 树一致的 `livox_frame`（或改 TF 发布） |
| 4 | 安装姿态 | 有 `<tilt_rpy>`（robot11 斜装 30° roll）与 `<cloud_frame>` 两套补偿 | **只有 `output_yaw`（绕 Z）**，没有 tilt/roll 参数 | **推荐把倾角写进 SDF 的 `<link><pose>`**（更物理、射线真斜），不要照搬 `tilt_rpy`；`output_yaw` 只用于 180° 朝向那类安装 |
| 5 | 逐点时间 | 一帧一个时刻（帧内无运动，`offset_time` 恒等于帧内偏移 0） | `offset_time = i·1e9/point_rate`，**跨 100 ms 单调**；帧内无位姿插值（≠ 真机的帧内运动） | 接受（FAST-LIO 去畸变按真机语义用），或照 `jongbob1918` 的思想自己加 30 行 `scanFraction` 位姿插值 |
| 6 | 只发有回波点 | ✅ 2026-09-23 起只发有效回波（约 20.8% 命中） | ✅ 同样只发有效回波（16.7–17.3%，差异来自窗口/`max_range`） | 一致，无需改 |
| 7 | **QoS** | **`rclcpp::SensorDataQoS()`（best-effort）**，2026-09-23/24 两次修复的结论是"写者永不阻塞" | **`rclcpp::QoS(KeepLast(20)).reliable()`**（`mid360_system.cpp:196`） | ⚠️ **同一个坑会复发**：RELIABLE 写者 + 慢消费者（slam_toolbox / RViz / STVL）会**堵在 gz 的 PreUpdate 线程里**，表现就是我们 Classic 上见过的"/livox/lidar 与 /pointcloud 一起停更且不自恢复"。P2 必须改成 best-effort（或改成非阻塞发送/独立线程）|

### 10.7 未验证 / 未做（诚实清单）

- **自机排除**：bench 的 sensor model 只有 IMU、无几何，所以"打到自己"这条路径**没有真正被触发**。代码路径（`MapRegistry::AddIgnoredModelEntity` → map 侧按 entity 祖先剔除）存在且编译通过，但**没有实测数据**。
- **动态模型 refit**：世界全静态，`delta_trans/delta_rot` 分支没被走到；即"机器人/对手在动时 BVH 增量更新"未测（§8 原有疑问仍在）。
- **`<dart><collision_detector>bullet</collision_detector></dart>`**（候选 demo world 里有）在 Fortress 上是否被接受**未单独验证**（我们的世界没用它）。
- **`GZ_SIM_SYSTEM_PLUGIN_PATH` vs `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`**：Fortress 6.18 的二进制里两套拼写都出现（`strings` 可见 `GZ_SIM_SYSTEM_PLN`/`IGN_GAZEBO_SYSTEGZ_SIM_…` 的交错伪影），本机是**两个都设**才跑通的，**没有做单变量隔离**。P2 里建议两个都设（或直接 `LD_LIBRARY_PATH`）。
- **多传感器/多实例**（同一世界两台北斗）未测。
- **Classic 本体点云**未参与对拍（用 Bullet 参考替代，理由见 §10.5）。
- **CPU 峰值 vs 均值**：表里是 1 s 采样的瞬时值，未做 `perf`/火焰图。

### 10.8 许可证动作（本轮）

- **没有把任何第三方源码拷进仓库**。`.tmp_gzlivox/` 是未跟踪草稿（`.tmp_*/` 忽略），里面的 `ah/`、`rgp/` 保留原始 `LICENSE`（MIT / BSD-3）与 `THIRD_PARTY.md`。
- 仓库里只落**本节**：精确 revision（`497149a` / `c6dc04e` / rmagine `2.4.3`）+ **可复现的补丁表**（§10.2）+ 实测数字。这样许可上只"描述"不"再分发"。
- **Embree 4.4.0 是 Apache-2.0、构建期从 GitHub 下载**：CI/离线环境要么预置 tar、要么 `-DRMAGINE_DEPENDENCY_MODE=Local` 配系统 Embree 4（**apt 的 3.12 不可用**）。
- 我们自己的 `mid360.csv` 没有外发；**没有**引入候选自带的 `mid360-real-centr.csv`（避免"图案数据再分发"的法务问题，§6 已提示）。

### 10.9 裁决：**GO for Fortress**（P2 可以按 A 方案推进）

**证据链（三条独立、都可复算）**：
1. **能编**：底座 `uos/rmagine_gazebo_plugins` **零改动**编过（9.2 s）；MID-360 插件用 **4 类补丁（+82/−18 行，含 1 个纯计时文件）** 编过，产物导出 `IgnitionPluginHook`，`CustomMsg` 类型支持已链入。
2. **能跑**：headless **10.000 Hz / RTF 0.985–0.993 / 213 MB / 36–38% CPU**；GUI（Xvfb+llvmpipe）**9.77 Hz / RTF 0.96–1.0**；`offset_time` 单调且 ≤100 ms；`line ∈ {0..3}`；IMU 197–200 Hz。
3. **算得对**：与独立 Bullet BVH 参考**逐束命中集合一致**（5009/5009、5112/5112、29999/30000），距离差 **p50 2.6 µm / p95 6 µm / max 69 µm**，**无系统偏移**。

**如果坚持要 NO-GO 的话，唯一的候选理由是"工作量"而不是"能力"**：不是 (a) API/ABI（只差 2 个宏）、不是 (b) 缺 Fortress 特性（`RaycastData` 缺失已被 Embree 后端绕过）、不是 (c) 网络/依赖（Embree 4 能下、能编），而是 **(d)**：还要做 §10.6 的 6 项契约对齐 + §10.4.6/10.4.7 的 2 项资产修正 + §10.7 的 4 项补测。

**P2 建议顺序（每步都可独立验收）**：
1. **修资产**（0.5 h）：`RMUC2026_world/model.config` 的 `<sdf version="1.9">model.sdf</sdf>`；全仓 SDF 版本 ≤1.9（`ign sdf -k` 扫一遍）。
2. **落地补丁**（0.5 d）：把 §10.2 的 P1–P11 做成我们自己的 `livox_simulation_gz` 包（**不 vendor**，只 vendored-patch 记录 + 从候选/上游取包），包名避开 `rmagine_gazebo_plugins` 冲突。
3. **契约对齐**（1 d）：`/livox/lidar`(CustomMsg) + `/livox/lidar/pointcloud`(PointCloud2) 同时发（§10.6 #2）、**QoS 改 best-effort**（§10.6 #7，这是会"冻死整条感知链"的那条）、`frame_id` 对齐 TF、倾角走 `<link><pose>`、`min/max_range` 与 Classic 配置一致（Classic 侧要核对）。
4. **补测**（1 d）：自机排除、动态模型 refit、双传感器、Classic 同姿态同窗口点云对拍（这次真跑 Classic）。
5. **接入 bringup**（1 d）：新增 opt-in launch（不动现有 `bringup_sim.launch.py`），跑一遍 FAST-LIO 端到端，看 `/Odometry` 与地图是否与 Classic 同量级。

**若第 3 步卡住**，退路不是回 Classic，而是**B 方案**（只用上游 BSD-3 的 `rmagine_gazebo_plugins` + 自写 ~200–300 行 `Mid360System`）——因为 §10.1 已经证明上游那条地基在 Fortress 上是**零改动可编**的，风险比 A 方案更低。

---


---
