# 操作手册（跑起来 / 建图 / 验收 / 显示）

> 按步骤执行的命令与验收标准

> **2026-11-13 合并**：本文件由 **6 份**同主题文档合并而成，原文全部保留、按来源分节（`## 来源：xxx`）。原件已删除，可从 git 历史取回。

**目录**
1. `smoke_test_runbook.md`
2. `machine_hygiene_runbook.md`
3. `rviz_guide.md`
4. `gz_acceptance_checklist.md`
5. `runbook_2027_mapping.md`
6. `README.md`

---
7. **2026-11 新增操作面**（三套模式 / 地图流程 / 日志分析 / UWB / BIT 前提）

---

## 来源 1/6：`smoke_test_runbook.md`


> 用途：用**原生 `ros2 launch` 指令**逐场景验证仿真链路（含 T1–T5 TF/话题契约改造后的效果）。
> 配套文档：`docs/algorithm_matrix.md`（**算法现状唯一真值来源**：槽位/资产/实测状态）、
> `docs/architecture.md`（目录架构与概念 §3.2.x）、`docs/issues_and_findings.md`（坑与根因汇总）、
> `docs/3d_to_2d_survey.md`（3D→2D 各家实现对照）、`docs/tf_interface_contract.md`（TF 契约）、
> `docs/params_ownership_checklist.md`（参数归属）。
> 说明：本文不使用任何封装脚本，全部为可直接复制的原生命令。

---

## 0. 前置

### 0.0 依赖前提（首次务必确认）

**① 系统 python 必须是 3.10（不能用 conda）**
ROS Humble 的 `rclpy` 等 C 扩展是 **cpython-310** 编译的；conda base 若是 3.13 等版本，**无法通过 pip 补齐**（`import rclpy` 直接失败），并且 `spawn_entity.py` 会因 conda python 缺 numpy 而崩溃 → 机器人不生成。

**本项目统一采用"全跟系统 python"方案**，推荐做法（一次配置，永久生效）：
```bash
# 方式一（推荐）：关闭 conda base 自动激活 —— 新终端直接就是系统 python
conda config --set auto_activate_base false      # 重开终端生效；需要时再 conda activate base
# 方式二：每个 ROS 终端手动退出
conda deactivate
# 方式三：不退出 conda，只让系统 python 优先
export PATH=/usr/bin:$PATH
```
**保底做法**：若新终端里 `which python3` 仍是 `~/miniconda3/bin/python3`，在 `~/.bashrc` **末尾**追加一行即可（conda 的 init 块会把 PATH 插到前面，末尾覆盖最稳）：
```bash
echo 'export PATH=/usr/bin:$PATH' >> ~/.bashrc && source ~/.bashrc
```
（当前终端若已残留 conda PATH：`conda deactivate; export PATH=/usr/bin:$PATH; hash -r`）
验证（必须全部满足）：
```bash
which python3                                   # 必须是 /usr/bin/python3
python3 -c "import rclpy, numpy; print('OK', numpy.__version__)"   # 必须是 3.10 的 numpy 1.x
grep -n "vision\|hzmirmvision" ~/.bashrc        # 若有旧的 vision 工作区 source 行，建议删掉（会污染 AMENT_PREFIX_PATH）
```

**② Nav2 主体包必须安装**（否则组件与 RViz 面板全部加载失败）
```bash
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins
ls /opt/ros/humble/share | grep -c "^nav2"      # 期望 20+（只装基础件时只有 7）
```
另外 teb 编译依赖 `ros-humble-dwb-critics`（若尚未安装：`sudo apt install -y ros-humble-dwb-critics`）。

**③ 不要激活仓库里的 `.venv`**
`.venv/` 是工具链（MCP 等）用的隔离 Python 环境（`include-system-site-packages = false`），**与 ROS 无关**：激活后 `which python3` 会指向 `.venv/bin/python3`，`import numpy / rclpy` 全部失败。
```bash
deactivate                    # 若提示符是 (.venv)，先退出
which python3                 # 必须是 /usr/bin/python3，而不是 .../.venv/bin/python3
```

### 0.1 环境（每个新终端都要做）
```bash
cd ~/HzMi_rmsimulation
source /opt/ros/humble/setup.bash
source install/setup.bash
# 若 shell 里有 conda/miniconda 干扰 ROS 工具：export PATH=/usr/bin:$PATH
# 若终端启动时提示 xxx/vision_ws/install/setup.bash 不存在：那是 ~/.bashrc 里遗留的旧工作区路径，无害，可自行清理
```

### 0.2 清理残留进程（切换场景前执行一次）
```bash
pkill -f gzserver; pkill -f gzclient; pkill -f rviz2
pkill -f fastlio_mapping; pkill -f pointlio_mapping
pkill -f cartographer_node; pkill -f slam_toolbox
pkill -f controller_server; pkill -f planner_server; pkill -f bt_navigator
sleep 2
```

### 0.3 通用约定
- **终端 A** 跑 launch（会弹 Gazebo/RViz），**终端 B** 跑检查命令；一个场景 `Ctrl+C` 结束后再跑下一个；
- `bringup_sim.launch.py` 常用参数与默认值：

| 参数 | 取值 | 默认 |
|---|---|---|
| `world` | `RMUC` / `RMUL` / `RMUL2026` | `RMUL2026` |
| `mode` | **`mapping`（纯建图）/ `slam_nav`（边建边导）/ `nav`（先建后导）** | 空（**必填**） |
| `lio` | `fastlio` / `pointlio` / `none` / `cartographer`（全包，见 §6.1） | `fastlio` |
| `localization` | `amcl` / `slam_toolbox` / `icp` / `cartographer`（**仅 `mode:=nav`** 生效） | 空 |

> ⚠️ **常见误用（2026-09-24 实录）**：把 `amcl` 传给 **`lio`** 槽（例：`lio:=amcl`）⇒ 没有任何分支匹配它 ⇒ **FAST-LIO 与 AMCL 都不会启动**，`/odom` 与 `odom→base_link` 全部缺失，costmap 报 `Tf has two or more unconnected trees` 并每 0.5 s 超时一次（进程列表里查不到 `fastlio_mapping` 即为铁证）。
> **`amcl` 只能给 `localization`**；`lio` 只接受 `fastlio` / `pointlio` / `none` / `cartographer`。
> 正确写法：`world:=RMUL2026 mode:=nav lio:=fastlio localization:=amcl nav:=rpp spin_speed:=0.0`

| `nav` | `rpp` / `dwb` / `teb`（`nav` / `slam_nav` 生效） | `rpp` |
| `mapper` | `slam_toolbox` / `cartographer`（`mapping` / `slam_nav` 生效） | `slam_toolbox` |
| `lio_rviz` / `nav_rviz` | `True` / `False` | `False` / `True` |
| `spin_speed` | 任意（rad/s） | `5.0` |
| **`global_obstacle`** | `stvl`（3D 体素层）/ `scan`（2D，与 local 同源）/ `none` | `stvl` |
| **`local_obstacle`** | `scan`（`/scan`，原行为）/ `cloud`（3D 点云直投，不经 `p2l`）/ `both`（scan+cloud 双源冗余）/ `stvl`（3D 体素层，带时间衰减）/ `stvl_both`（stvl+scan+cloud） | `scan` |

### 0.4 看哪块 RViz（别把两个都关掉）

| 你想看什么 | 用哪个 | 打开的配置 | 里面有什么 |
|---|---|---|---|
| **LIO 点云**（任何模式） | `lio_rviz:=True` | `rm_nav_bringup/rviz/fastlio.rviz` / `pointlio.rviz` | 原始点云、`cloud_registered`、体素地图 |
| **建图 / 导航效果**（**推荐**） | `nav_rviz:=True`（默认） | `rm_navigation/rviz/nav2.rviz` | `RobotModel`(`/robot_description`)、`Map`(`/map`)、本地/全局代价地图、`/scan`、TF、**Navigation2 面板**、**2D Pose Estimate / 2D Goal Pose 工具**（Fixed Frame = `map`） |

- `mode:=mapping`（纯建图，不起 nav2）下 `nav_rviz:=True` 也会给一块 RViz（2026-09 起 bringup 单独补的），能看 `/map` 边建边长；
- ⚠️ **两个都给 `False` = 屏幕上什么可视化都没有**（Gazebo 里还有机器人，但没法判断导航效果）；
- 两个都给 `True` = 两个 RViz 同时抢 GPU，RMUL2026 这种重场景容易卡死/闪退，**只开一个**；
- 定目标/给初值：nav 模式下直接用 `nav2.rviz` 的工具栏按钮，比命令行方便。

### 0.4.1 `spin_speed`：小陀螺在仿真里的陷阱

`fake_vel_transform` 把 nav2 的角速度指令**替换成固定角速度** `spin_speed`（`/cmd_vel` 里角速度非零 → `/cmd_vel_chassis` 就用 `spin_speed`）。

- **真实哨兵**：电控本来就让底盘持续自转，nav2 的角速度只是"增减"信号，云台机械补偿保证雷达朝向稳定；
- **仿真**：`planar_move` 没有基线自转，任何一点角速度修正都会让底盘以 **5 rad/s（≈286°/s）** 旋转，而**仿真里没有云台补偿，雷达跟着底盘一起转** → 10 Hz 的 FAST-LIO 每帧要承受约 29° 旋转，跟踪容易退化，进而"控制器看不到进展 → 进恢复行为 → 越转越糟"。

所以排查导航问题时**先用 `spin_speed:=0.0`**（角速度直通，等价普通 nav2），确认基础导航链路通了之后，再打开 `5.0` 做小陀螺对比实验。日志判据：`/cmd_vel_chassis` 里出现 `angular.z: 5.0` 就说明小陀螺在动作（那是 `spin_speed` 的值，不是 nav2 发的角速度）。

### 0.5 三个场地的地图资产现状（`rm_nav_bringup/map/`）

| world | 栅格地图 | `origin` / 尺寸 | map 系约定 | **AMCL 初值（map 系）** | 可用重定位 | 备注 |
|---|---|---|---|---|---|---|
| `RMUC` | `RMUC.pgm` 577×301 | `(-6.35,-7.6)` | **出生点系** | `(0, 0, 0)` | AMCL / slam_toolbox(`.posegraph`) / ICP(`.data`) | 最全 |
| `RMUL` | `RMUL.pgm` 272×210 | `(-3.75,-4.54)` | **出生点系** | `(0, 0, 0)` | AMCL / slam_toolbox(`.posegraph`) | |
| `RMUL2026` | `RMUL2026.pgm` 240×169 | `(2.68, 0.228)` | **世界系** ⚠️ 与上两者不同 | `(4.3, 3.35, 0)` | **仅 AMCL** | 无 `.posegraph`、无 `.pcd`；`RMUL2026.pbstream` 仅 526 B（空，cartographer 纯定位不可用）。⚠️ 用 cartographer/slam_toolbox **重建**后新图变**出生点系**（初值改 `(0,0,0)`）；新旧图差 `(4.3,3.35)`，**不可混用**（§5/§6） |

#### 怎么判定出来的（别再靠猜）

用**场地 STL 的世界包围盒**与 **pgm 已知区域（非 205 像素）包围盒**比对，尺寸与位置两项都对上才算数：

| world | 场地 mesh 世界包围盒 | pgm 已知区域 bbox | 判定 |
|---|---|---|---|
| `RMUC` | x[0.00, 29.20] y[0.00, 15.20] | x[-6.35, 22.50] y[-7.60, 7.45] | mesh − spawn(6.35,7.6) = x[-6.35,22.85] y[-7.60,7.60] ≈ pgm → **出生点系** |
| `RMUL` | x[0.51, 14.23] y[-1.25, 9.30] | x[-3.75, 9.80] y[-4.49, 5.96] | mesh − spawn(4.30,3.35) = x[-3.79,9.93] y[-4.60,5.95] ≈ pgm → **出生点系** |
| `RMUL2026` | x[2.20, 14.80] y[0.20, 8.80] | x[2.68, 14.68] y[0.23, 8.43] | mesh **直接**等于 pgm（无 spawn 偏移）→ **世界系** |

推论与注意事项：
1. **`RMUL`/`RMUC` 的 pgm 是 sim 建图导出**（slam_toolbox 的 map 原点 = LIO 起点 = 机器人出生点），所以地图坐标里机器人在 `(0,0)`。之前按世界出生点 `(4.3,3.35)` 给 AMCL 初值是**错的**，会把机器人放到地图里偏差 5.4 m 的位置——这与你「rviz 看不出来、判断不了效果」直接相关。
2. `RMUL`/`RMUC` 的 `.posegraph`（slam_toolbox 纯定位的 `map_start_pose`）与 `.data`/pcd（ICP 的 `initial_pose`）本来就是 `[0,0,0]`，与 ① 一致 → **这两套资产内部自洽，不要动**。
3. ⚠️ **`RMUL2026` 是唯一的例外**：pgm 是世界系，而它没有 posegraph/pcd。将来若用 sim 建图给 RMUL2026 生成 `.posegraph`/`.pcd`，那些产物会变成**出生点系**，与现有 pgm 差 `(4.3, 3.35)` → 三种重定位模式会各自用不同的 map 系。**建议：做一次 RMUL2026 建图，用 `tools/scripts/mapping/save_grid_map.sh` 重新导出 pgm+posegraph，让该场地也统一成出生点系**（届时把 `bringup_sim.launch.py` 里 `amcl_init_x/y` 的 RMUL2026 也改成 `0.0`）。
4. 当前初值由 `bringup_sim.launch.py` 按 `world` **自动注入**（`amcl.ros__parameters.initial_pose.*` + `set_initial_pose: true`），无需手动发 `/initialpose`；运行中仍可用 `/initialpose` 或 RViz 的 `2D Pose Estimate` 覆盖。


### 0.6 ⚠️ 选测试目标点：**先验连通域，别靠眼估**

nav2 全局规划把「障碍 + `robot_radius` 内切膨胀带」视为不可通行，**地图上 1 像素（5cm）宽的虚线就足以把一条走廊彻底封死** → `planner_server: failed to generate a valid path` → BT 反复跑恢复行为（`/cmd_vel` 只会出现 `-0.05` 的 BackUp 和 `spin_speed` 的旋转）。

```bash
# 检查"某个目标点从起点是否可达"（最常用）
/usr/bin/python3 tools/check_map_reachable.py --map src/bringup/rm_nav_bringup/map/RMUL2026.yaml \
  --start 4.3 3.35 --goal 6.0 3.4
# 让工具推荐一批可达且离墙够远的目标点
/usr/bin/python3 tools/check_map_reachable.py --map src/bringup/rm_nav_bringup/map/RMUL2026.yaml --start 4.3 3.35
```
> `--start` 用**该场地 map 系**的起点：`RMUC/RMUL` 是 `0 0`，`RMUL2026` 是 `4.3 3.35`（依据见 §0.5）。
> 工具还会报"地图被切成几块"——切成多块时，目标点跨块必然规划失败。

**RMUL2026 的实测结论（重要）**：该场地被切成 3 块（#2=24246 格 / #1=3692 格 / #0=693 格），
起点在 #2，而 **`(6.0, 3.4)` 落在 #1 → 必然规划失败**。原因是 `RMUL2026.pgm` 里存在一道 **x≈5.2 的竖直虚线（1 像素宽、带小缺口，从 y≈1.8 延伸到 6.0）**，
而 **RMUL2026 世界网格在该处完全没有几何**（逐顶点核对：x∈[5.05,5.35]、y∈[2.8,4.0] 的顶点数 = 0）——
即 **pgm 里有世界不存在的"幽灵墙"**，膨胀后把走廊封死。这与 §0.5 的结论一致：`RMUL2026.pgm` 更像**官方场地平面图**（含分区虚线），而不是在 sim 里建出来的图。

**因此：**
1. 立刻验证导航链路 → 用**同一连通域**的目标点，例如 `(4.01, 4.70)`（1.38m）、`(3.91, 5.40)`（2.09m）、`(8.26, 5.45)`（4.48m，会绕行，可验证长路径）；
2. 彻底解决 → **在 sim 里重新给 RMUL2026 建一次图**（`mode:=mapping` + `tools/scripts/mapping/save_grid_map.sh`），
   得到与世界一致、且与 RMUL/RMUC 统一成"出生点系"的 `pgm+posegraph`（同时替换掉 526 B 的空 `RMUL2026.pbstream`），
   之后把 `bringup_sim.launch.py` 里 RMUL2026 的 `amcl_init_x/y` 改回 `0.0`。

---

### 0.7 三种场景形态（`mode`）各起什么 —— 别混用

| `mode` | 中文 | Gazebo+LIO | 在线 SLAM 后端 | 导航栈 nav2 | `map_server` | 重定位模块 | `map→odom` 来源 |
|---|---|---|---|---|---|---|---|
| `mapping` | **纯建图** | ✅ | ✅ `mapper:=slam_toolbox\|cartographer` | ❌ **不启动** | ❌ | ❌ | 在线 SLAM（离线保存用） |
| `slam_nav` | **边建图边导航** | ✅ | ✅ 同上 | ✅ | ❌ | ❌ | 在线 SLAM 直接喂 costmap |
| `nav` | **先建图后导航** | ✅ | ❌ | ✅ | 仅 `localization:=icp` 或留空时 | `localization:=amcl\|slam_toolbox\|icp` | 所选重定位模块 |

要点：
- `mapping` 不启动 nav2（2026-09 起）：省下 7 个 nav2 节点 + 两张 costmap，重场景下建图帧率明显更稳；
- `slam_nav` 与 `nav` 的区别**只在"地图从哪来"**：前者是在线 SLAM（含回环优化，`map→odom` 会随优化跳变），后者是磁盘地图 + 重定位（`map→odom` 由重定位模块平滑给出）；
- `localization` 只在 `mode:=nav` 生效（`slam_nav`/`mapping` 传了也会被忽略）；
- `mode:=nav` 留空 `localization` 是**回退用法**：直接用 LIO 当绝对定位，并由 `camera_init→map`、`body→odom` 两条静态桥补帧（只在 `mode=='nav' and localization==''` 时启动）；
- 三种形态的 `/map` 发布者都只有一个（在线 SLAM 或 map_server），2026-09 已修掉建图模式下 `map_server` 抢 `/map` 的问题（见 §10）。

### 0.8 文档地图 & 当前推荐顺序

| 文档 | 用途 |
|---|---|
| `docs/smoke_test_runbook.md` | **本文件**：怎么跑、怎么判、错了怎么查 |
| `docs/algorithm_matrix.md` | **算法现状唯一真值来源**：有哪些算法/组合、资产齐不齐、实测状态、阻塞项 |
| `docs/architecture.md` | 目录/分层/参数/概念（§3.2.1–3.2.7 是概念长文） |
| `docs/glossary.md` | **术语表**：名词一句话定义 + 指向详述（子图/回环/ESDF/2.5D/降维/代价…） |
| `docs/glossary.md` | **术语表**：名词一句话定义 + 指向详述（子图/回环/ESDF/2.5D/降维/代价…） |
| `docs/issues_and_findings.md` | 踩过的坑与根因汇总（含 RMUL2026 幽灵墙证据链） |
| `docs/tf_interface_contract.md` | TF/接口契约（T1–T5 已实施，T6 撤销） |

**当前推荐顺序**（按资产齐备度排，先解开阻塞再看矩阵）：
1. **重建 RMUL2026 地图**（§1 建图 + §1.1 落盘三件套）← 解开该场地**所有** nav 组合；
2. `nav + amcl` @ RMUL2026 发目标（§2 + §0.6 选点）；
3. `nav + slam_toolbox` @ RMUL（§4）与 `nav + icp` @ RMUL（§3）；
4. `slam_nav` 边建边导（§1.2）；5. cartographer 建图/纯定位（§5/§6，纯定位需先生成 pbstream）；6. `nav:=dwb|teb`（§7）。

---

### 0.9 5 分钟上手（TL;DR）

```bash
# 0) 环境 + 清理（见 §0.1 / §0.2）
cd ~/HzMi_rmsimulation && source /opt/ros/humble/setup.bash && source install/setup.bash
pkill -f gzserver; pkill -f gzclient; pkill -f rviz2; pkill -f component_container; sleep 2

# 1) 起仿真（资产最全、最稳的组合：先建后导 + AMCL）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=rpp spin_speed:=0.0

# 2) 等 30~60 s（RMUL 世界重），另开终端跑就绪检查（§9.1 是完整版）
ros2 lifecycle get /controller_server      # 期望 active
ros2 run tf2_ros tf2_echo map odom         # 有输出（≈AMCL 初值）
ros2 topic hz /scan                        # 有稳定频率

# 3) 先验可达性再发目标（§0.6，别靠眼估）
/usr/bin/python3 tools/check_map_reachable.py --map src/bringup/rm_nav_bringup/map/RMUL.yaml --start 0 0 --goal 1.68 3.44
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 1.68, y: 3.44, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```
**判据**：`/cmd_vel` 出现**正向** `linear.x`、Gazebo 里车在动、RViz 里 scan 与地图墙体重合。
**不通时**：先按 §10.1「现象 → 归属」分类，再查 §10 具体行。

### 0.11 启动参数总表（`bringup_sim.launch.py`，全部原样可用）

| 参数 | 取值 | 默认 | 说明 |
|---|---|---|---|
| `world` | `RMUC` / `RMUL` / `RMUL2026` | `RMUL2026` | 场地；同时决定 `map/<world>.*` 与 `PCD/<world>.pcd` 前缀 |
| `mode` | `mapping` / `slam_nav` / `nav` | 空（**必填**） | 场景形态，决定启动哪套节点集（§0.7） |
| `lio` | `fastlio` / `pointlio` / `none` / `cartographer` | `fastlio` | 里程计实现；`none` 需外部提供 odom/TF；`cartographer` = 全包形态（兼任里程计源，见 §6.1） |
| `localization` | `amcl` / `slam_toolbox` / `icp` / `cartographer` / 空 | 空 | **仅 `mode:=nav` 生效**；空 = 回退用法（LIO 当绝对定位 + 静态桥） |
| `mapper` | `slam_toolbox` / `cartographer` | `slam_toolbox` | 在线 2D 建图后端；`mapping`/`slam_nav` 生效 |
| `nav` | `rpp` / `dwb` / `teb` | `rpp` | 局部规划器变体；`nav`/`slam_nav` 生效 |
| **`global_obstacle`** | `stvl` / `scan` / `none` | `stvl` | 全局代价地图的实时障碍来源（A/B 槽位，见 `docs/3d_to_2d_survey.md` §七） |
| **`local_obstacle`** | `scan` / `cloud` / `both` / `stvl` / `stvl_both` | `scan` | 局部代价地图障碍来源（A/B 槽位，§7.2）：`cloud` 用点云直投破 `p2l` 单点；`both` = scan+cloud 双源冗余；`stvl` = 3D 体素层（带 0.5 s 时间衰减，= COD 2026 双图做法，实测见 `docs/stvl_local_costmap.md`）；`stvl_both` = stvl+scan+cloud |
| `spin_speed` | 任意（rad/s） | `5.0` | `fake_vel_transform` 小陀螺固定角速度；排查导航先用 `0.0`（§0.4.1） |
| `lio_rviz` | `True` / `False` | `False` | 开 LIO 点云 RViz |
| `nav_rviz` | `True` / `False` | `True` | 开 nav2 RViz（`mode:=mapping` 也会给一块） |
| `use_sim_time` | `True` / `False` | `True` | 仿真是 `True` |
| **`map_name`** | 任意名字 | `''` ⇒ 取 `<world>` | **存档基名**（续建/保存都用它，同名覆盖）。★ 新增，见 `docs/continue_mapping.md` |
| **`map_autocontinue`** | `True` / `False` | `True` | 同名 `map/<map_name>.posegraph` 存在 ⇒ **反序列化接着建**；不存在 ⇒ 从零建（日志写明路径）。★ 新增 |
| **`map_start_pose`** | `[x, y, θ]` | `[0.0, 0.0, 0.0]` | 续建时"机器人在旧图里的位姿"（`map` 系 = 出生点相对系 ⇒ 出生点起步就是 0,0,0）。★ 新增 |
| **`map_allow_world_mismatch`** | `True` / `False` | `False` | `False` = 存档的 world/出生点与本次不一致（或无 sidecar 的旧存档）⇒ **拒绝续建并终止 launch**。★ 新增 |
| **`cloud_accumulator`** | `True` / `False` | `False` | 仅 `mode:=mapping`：起 3D 点云累加器，让 `PCD/<map_name>.pcd` 也跨会话续建。★ 新增 |

> ★ **2026-10-06：`mode:=mapping` 支持"在上次基础上继续建图"**（同名存档 + 场地隔离守卫）。
> 流程：起栈（给 `map_name`）→ 走 → `tools/scripts/mapping/map_archive.sh save` → 关栈 →
> 下次同一条命令自动续建。**换 world 续建同一份存档会被默认拒绝**（防串场地）。
> 全部命令、拒绝消息原文、实测数字与未验证项见 **`docs/continue_mapping.md`**。

> 另有两个**只对 cartographer 生效**的参数（`cartographer_sim.launch.py`）：
> `load_state_filename`（pbstream 路径，留空=纯建图）与 `load_frozen_state`，以及
> `occupancy_grid_topic`（栅格输出话题，默认 `map`；纯定位另有 map_server 时传 `/cartographer_map`）。

### 0.10 场景 × 资产前提 × 命令（总览）

| 场景 | **必需资产** | 命令要点 | 关键判据 |
|---|---|---|---|
| §1 纯建图 | 无（从零） | `mode:=mapping mapper:=slam_toolbox\|cartographer` | `/map` 只有 1 个发布者且边长；落盘见 §1.1 |
| §1.2 边建图边导航 | 无 | `mode:=slam_nav mapper:=...` | `node list` **无** amcl/map_server；`map→odom` 由在线 SLAM |
| §2 nav+AMCL | `.pgm`+`.yaml` | `mode:=nav localization:=amcl` | `map→odom` 由 amcl；`/amcl_pose`≈注入初值 |
| §3 nav+ICP | **`PCD/<world>.pcd`**（仅 RMUC/RMUL） | `mode:=nav localization:=icp` | `map→odom` 由 `icp_registration`（T5） |
| §4 nav+slam_toolbox | **`.posegraph`**（仅 RMUC/RMUL） | `mode:=nav localization:=slam_toolbox` | `map→odom` 由 slam_toolbox；无 map_server/amcl |
| §5 cartographer 建图 | 无 | `mode:=mapping mapper:=cartographer` | `/map` 1 个发布者（cartographer occupancy_grid） |
| §6 cartographer 纯定位 | **`.pbstream`（三个场地都缺，需先用 §5 生成）** | `localization:=cartographer`（一键） | 加载状态成功、`map→odom` 由 cartographer、`/map` 仍是 map_server 的先验图 |
| §7 局部规划变体 | 同 §2 | `nav:=rpp\|dwb\|teb` | 三个都 active 且能发目标 |

> 资产盘点与"哪些组合真能跑"以 `docs/algorithm_matrix.md` §三/§四 为准。

---

## 1. 场景一：mapping + fastlio（建图 / 重建地图）

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=mapping lio:=fastlio mapper:=slam_toolbox spin_speed:=0.0
```
> `spin_speed:=0.0`：遥控建图时若底盘按 5 rad/s 跟着角速度指令转，雷达会一起转，LIO 容易退化（见 §0.4.1）。
> 建图模式（`mapping`）**既不起 `map_server`、也不起导航栈**（2026-09 起）：
> 前者会拿磁盘旧 pgm 抢 `/map`（`map_saver_cli` 可能存下旧图），后者白白吃掉 7 个 nav2 节点 + 两张 costmap 的 CPU。

**终端 B（检查）**
```bash
ros2 topic hz /odom                          # 期望 ≈10Hz（fastlio）
ros2 topic hz /livox/lidar                   # 期望 10Hz（CustomMsg，FAST-LIO 的输入）
ros2 topic echo /scan --once                 # 期望：一帧 LaserScan
ros2 run tf2_ros tf2_echo odom base_link     # 期望：持续输出，无 Invalid frame ID
ros2 topic echo /map --once --field info     # 期望：宽高随建图增长（只有 slam_toolbox 一个发布者）
```

**终端 C（遥控走遍场地）**
```bash
tools/scripts/control/improved_teleop.sh      # 键盘遥控（建议速度慢一点、覆盖整场、回到起点附近收尾）
```

**键位（2026-09-21 改成方向键）**：

| 键 | 作用 |
|---|---|
| `↑` / `↓` | 前进 / 后退（默认 **0.5 m/s**） |
| `←` / `→` | 左转 / 右转（默认 **1.0 rad/s**） |
| `空格` | **紧急停止** |
| `>` / `<` | 线速度 +0.1 / −0.1（0.1 ~ 2.0 m/s） |
| `.` / `,` | 角速度 +0.1 / −0.1（0.2 ~ 3.0 rad/s） |
| `0` | 速度重置为默认值 |
| `s` / `h` | 系统状态 / 帮助 |
| `q` | 退出（退出前自动发一次零速） |
| 兼容旧键 | `i/k/j/l` = 前进/后退/左转/右转；`[` / `]` = 角速度 ± |

> ⚠️ **按一下只发一次 Twist，底盘会保持该速度** → 停车必须按 `空格`（`q` 退出时也会自动停）。
> 默认发到 **`/cmd_vel_chassis`**（直连底盘，绕过 `fake_vel_transform`）；
> 想让指令走 nav2 那条链（经 `fake_vel_transform`、受 `spin_speed` 影响）时用：
> `TELEOP_TOPIC=/cmd_vel tools/scripts/control/improved_teleop.sh`
> 建图建议：线速 0.3–0.5 m/s、转向 0.5–0.8 rad/s，绕场一圈、回到起点附近收尾。

### 1.1 落盘三件套（.pgm/.yaml + .posegraph + .pcd）

> ⚠️ **先备份**：`save_grid_map.sh` 会**覆盖** `map/RMUL2026.{pgm,yaml}`。

```bash
cp src/bringup/rm_nav_bringup/map/RMUL2026.pgm  src/bringup/rm_nav_bringup/map/RMUL2026_official.pgm
cp src/bringup/rm_nav_bringup/map/RMUL2026.yaml src/bringup/rm_nav_bringup/map/RMUL2026_official.yaml

# ① 栅格地图（map_saver_cli -f src/bringup/rm_nav_bringup/map/<world>）
tools/scripts/mapping/save_grid_map.sh

# ② 位姿图（slam_toolbox 纯定位与 nav 模式 map_start_pose 都靠它）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: 'src/bringup/rm_nav_bringup/map/RMUL2026'}"

# ③ 点云底图（ICP 重定位用；路径由 bringup 按 world 自动设为 PCD/<world>.pcd）
ros2 service call /map_save std_srvs/srv/Trigger
ls -l src/bringup/rm_nav_bringup/map/RMUL2026.* src/bringup/rm_nav_bringup/PCD/RMUL2026.pcd
```

> ★ **2026-10-06 更省事的做法（推荐）**：起栈时给一个 `map_name:=<名字>`，走完直接
> `tools/scripts/mapping/map_archive.sh save` —— 它会 ① 调 `serialize_map` 写同名
> `.posegraph/.data`、② 刷新 sidecar `<名字>.meta.yaml`（记 world/出生点，供下次**拒绝错场地**续建）、
> ③ 若 `cloud_accumulator:=True` 在跑，顺带存 `PCD/<名字>.pcd`。下次同一条命令即**接着上次建**。
> 细节见 `docs/continue_mapping.md`（含"旧版存档没有 sidecar 会被拒绝"的处理）。

建完图后**务必做两件事**：
1. 用可达性工具确认全场连通、记下可用的测试目标点：
   `tools/check_map_reachable.py --map src/bringup/rm_nav_bringup/map/RMUL2026.yaml --start 0 0`
2. 新图是**出生点系**（与 RMUL/RMUC 一致）→ 把 `bringup_sim.launch.py` 里 `amcl_init_x/y` 的 **RMUL2026 也改成 `0.0`**
   （同时 `docs/smoke_test_runbook.md` §0.5 的表格要同步更新）。

### 1.1.1 另一条拿到 2D 地图的路线：LIO 点云切层投影

`FAST-LIO/Point-LIO` 本身就是 LiDAR-Inertial **SLAM**：跑里程计的同时维护着一张三 3D 点云地图，
`/map_save` 落盘的 `PCD/<world>.pcd`（例：RMUL 是 **159 万点**）就是它建的图。**它不只能做定位**。
把它沿 z 切一层投影，就能直接得到 nav2/AMCL 能吃的 2D 栅格图（**不依赖 slam_toolbox/cartographer**）：

```bash
/usr/bin/python3 tools/pcd_to_grid_map.py --pcd src/bringup/rm_nav_bringup/PCD/RMUL.pcd \
  --out src/bringup/rm_nav_bringup/map/RMUL_frompcd --min-z 0.10 --max-z 0.50 \
  --compare src/bringup/rm_nav_bringup/map/RMUL.yaml
```

实跑验证（RMUL.pcd → 2D 图 vs slam_toolbox 的 `map/RMUL.pgm`）：
- 两图**同一坐标系**（bbox 重合，最佳平移仅 1~2 格 = 5~10cm）；
- **±1 格容差下参考图的墙被覆盖 84.7%** → 场地结构一致；
- 严格 IoU 只有 0.21，因为 3D→2D 投影的墙更厚、且把切层内的立体结构都算进来了。

**这工具的第二个用途：独立核对地图资产。** 怀疑某张官方图有"世界不存在的墙"（§0.6 的幽灵墙）时，
用 LIO 的 pcd 投影一张同场地的图，一比就知道那道墙在**实际观测**里存不存在。
切层用 `--min-z/--max-z` 控制（贴近平面的 0.10~0.50 最接近 2D 雷达视角）；自由空间由起点洪泛得到，
所以**若墙有缺口会外漏**，必要时先看投影图的墙是否闭合。

### 1.2 边建图边导航（`mode:=slam_nav`）

与 §2 的「先建图后导航」是**两条不同的启动装配**（见 §0.7）：这里**不加载磁盘地图、不起任何重定位模块**，
costmap 的 `static_layer` 直接吃在线 SLAM 发布的 `/map`，`map→odom` 也由在线 SLAM 提供。

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=slam_nav lio:=fastlio mapper:=slam_toolbox nav:=rpp spin_speed:=0.0
```

```bash
ros2 topic info /map --verbose | grep -c "PUBLISHER"   # 期望 1（在线 SLAM，无 map_server）
ros2 run tf2_ros tf2_echo map odom                     # 期望：由 slam_toolbox 提供，随建图/回环缓慢变化
ros2 lifecycle get /controller_server                  # 期望 active（nav2 延后 10s 起，等 /map 与 map→odom）
ros2 node list | grep -E "amcl|map_server"             # 期望：**空**（这条形态不该有它们）
```

判据与注意：
- RViz 里地图**边建边长**，可以直接用工具栏 `2D Goal Pose` 发目标让车一边探索一边走；
- 未探索区域是 unknown，`GridBased.allow_unknown: true` 允许穿越未知区；
- **回环优化会让 `map→odom` 跳变**（SLAM 修正累积误差的正常行为），此时车在 map 里的位置会"瞬移"一下，costmap 随之更新 —— 这与 §2/§3 里重定位模块给出的平滑 `map→odom` 是本质区别。

---

## 2. 场景二：nav + AMCL

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=rpp
```

**终端 B**
```bash
ros2 run tf2_ros tf2_echo map odom           # 期望：由 amcl 提供
ros2 lifecycle get /controller_server        # 期望：active
ros2 lifecycle get /planner_server           # 期望：active
ros2 topic echo /map --once --field info     # 期望：272 X 210（map_server 已激活）
ros2 action list | grep navigate_to_pose
```

> ✅ **初值已自动化**：`amcl` 参数里 `set_initial_pose: true`，初值由 `bringup_sim` 按 `world` 注入
> （`RMUC/RMUL → (0,0,0)`，`RMUL2026 → (4.3,3.35)`，依据见 §0.5），**不需要再手敲 `/initialpose`**。
> 想手动纠正（例如机器人被撞偏了）：RViz 工具栏 **"2D Pose Estimate"**，或
> ```bash
> # 以 RMUL 为例：map 系原点就是出生点，所以给 (0,0)
> ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
> "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.0685,0,0,0, 0,0,0,0,0.0685,0, 0,0,0,0,0,0.0685, 0,0,0,0,0,0]}}"
> ```
>
> 📌 若在启动早期手动发 `/initialpose`，可能看到
> `amcl: Failed to transform initial pose in time (Lookup would require extrapolation into the future ...)` —
> 位姿时间戳取 `now()`，而 10 Hz 的 `odom→base_link` 最新帧落后几十毫秒，tf2 不外推未来。
> **可忽略**（下一行仍会 `Setting pose (...)`）；想彻底没有这条告警，等 `tf2_echo odom base_link` 能持续输出后再发。

---

## 3. 场景三：nav + ICP（T5 关键验证点）

**资产前提**：ICP 吃 **3D 点云图** `PCD/<world>.pcd` → 目前只有 **RMUC / RMUL** 有（RMUL2026 需先用 `/map_save` 生成）。

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=icp nav:=rpp spin_speed:=0.0
```
> ⚠️ **RMUL 世界重（44 万三角面）：启动后等 30~60 s 再判断**（ICP 节点本身延后 7 s 启动；
> 过早判断会看到 `odom` 帧尚不存在、Nav2 无法激活，误以为链路坏了）。

**终端 B（就绪判据）**
```bash
ros2 node list | grep icp_registration            # ICP 是否起来
ros2 topic hz /livox/lidar/pointcloud             # ★ ICP 的输入（必须持续有数据）
ros2 run tf2_ros tf2_echo odom base_link          # LIO 里程计；无 → LIO 未初始化
ros2 run tf2_ros tf2_echo map odom                # ★ 关键：应由 icp_registration 发布（T5）
ros2 topic echo /map --once --field info          # 静态图（RMUL = 272×210）
ros2 lifecycle get /controller_server             # active
```
**逐段排查**（哪段先没数据，问题就在那一段）：`/livox/lidar`(CustomMsg) → `/livox/imu` → `/imu/data` → `/odom` → `map→odom`。
若 `/livox/lidar` 有数据而 `/odom` 长时间没有 → 换 `lio:=pointlio` 做 A/B（它吃原始 `/livox/imu`），
以区分是"雷达/世界侧"还是"FAST-LIO + 互补滤波支路"的问题。

---

## 4. 场景四：nav + slam_toolbox（纯定位）

**资产前提**：`localization:=slam_toolbox` 需要 `.posegraph` → **RMUC / RMUL 有，RMUL2026 没有**（要先建图，见 §1.1）。

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=slam_toolbox nav:=rpp spin_speed:=0.0
```
```bash
ros2 node list | grep -E "map_server|amcl"   # 期望：空（这条形态不需要它们）
ros2 run tf2_ros tf2_echo map odom           # 由 slam_toolbox 提供
ros2 lifecycle get /controller_server        # active
```

---

## 5. 场景五：cartographer 建图

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=mapping lio:=fastlio mapper:=cartographer spin_speed:=0.0
```
> 建图模式**不起 nav2**，但 `nav_rviz:=True` 仍会给一块 RViz（能看 `/map` 边建边长）。
> 栅格话题 **2026-09 已修正为 `/map`**（原先硬 remap 到 `/cartographer_map`，会让 nav2 的 `static_layer`、
> `map_saver_cli`、RViz 的 Map 显示项全都吃不到图）。

> **cartographer 2D 三个 mode 的跑通顺序**（严格按序；中间产物不可跨次混用）：
>
> | 序 | 命令 | 通过判据 | 落盘 |
> |---|---|---|---|
> | ① | `mode:=mapping mapper:=cartographer`（本节） | `/map` 宽高随建图增长；`/scan` 的订阅者里有 `cartographer_node`；`tf2_echo map odom` 有输出 | **pgm + pbstream，必须同一次运行存** |
> | ② | 同①但 `mode:=slam_nav`（是否 `spin_speed:=0.0` 按 §0.4.1 决定） | nav2 起来；`ros2 topic info /map --verbose` 订阅者含 `global_costmap`；`ros2 lifecycle get /planner_server` = active；发目标能规划出路径 | 不落盘（边建边导） |
> | ③ | §6：`mode:=nav localization:=cartographer` | pbstream 加载成功；`map→odom` 由 cartographer 发；`/map` 来自 `map_server`；`/cartographer_map` 是 cartographer 自己那张 | — |
>
> ⚠️ **pgm 与 pbstream 必须来自同一次建图**：cartographer 的 `map` 系原点 = 建图时机器人的起点（**出生点系**），
> 所以它导出的 pgm 与 pbstream 天然同一套坐标；若拿**旧 pgm**（现有 `RMUL2026.pgm` 是**世界系**）配**新 pbstream**，
> 两者差一个常量平移（RMUL2026 是 `(4.3, 3.35)`）→ 现象是 RViz 里先验图整体偏移、costmap 与定位错位。

**终端 B**
```bash
ros2 node list | grep cartographer                    # cartographer_node + cartographer_occupancy_grid_node
ros2 topic info /scan --verbose | grep "Node name"    # ★ 必须看到 cartographer_node（证明输入类型/话题对上了）
ros2 topic info /map --verbose | grep -c PUBLISHER    # 期望 1（cartographer occupancy_grid）
ros2 run tf2_ros tf2_echo map odom                    # 有输出（由 cartographer 发）
python3 tools/monitor_map_odom.py                     # ★ 50Hz 采样 + 判读：平滑漂移 / 锯齿矫正 / 高频抖动
                                                      #   （tf2_echo 每 ~0.8s 一行，看不出高频抖动；判据见 tools/README.md）
ros2 topic echo /map --once --field info              # 宽高随建图增长
ros2 run tf2_tools view_frames                        # ★ 帧树：body 只能有一个父(camera_init)，odom 下只应有 base_link
```
> **2026-09 修了四处致命对接问题**（这就是仓库里 `RMUL2026.pbstream` 只有 526 B 的原因）：
> ① 输入类型：`points2` 原 remap 到 `/livox/lidar`（**CustomMsg**，cartographer_ros 不支持）→ 改为吃
> `/scan`（`num_laser_scans=1`）；② 帧契约：原 `published_frame="body"`+`provide_odom_frame=true`
> 会与 FAST-LIO 争 `body` 的子帧 → 改为 `published_frame="odom"`+`provide_odom_frame=false`（只发 `map→odom`）；
> ③ 高度带：`min_z/max_z` 是**相对传感器**的（不是相对地面），原 0.05~0.8 只切到墙顶 → 改回 -0.8~2.0；
> ④ **IMU 帧必须与 `tracking_frame` 重合**：`sensor_bridge.cpp:136` 有硬 CHECK（平移 < 1e-5 m），
> 不满足**直接 abort（exit -6）**。我们 URDF 的 `livox_frame` 与 `imu_link` 相差 5cm（该 5cm 是
> FAST-LIO `extrinsic_T` 依赖的，不能改 URDF）→ `tracking_frame` 由 `livox_frame` 改为 **`imu_link`**
> （官方 `mir-100-mapping.lua` 同做法）。同时 `TRAJECTORY_BUILDER_2D.min_range` 0.45→0.2（与 p2l 对齐）。

**落盘**（三件套都可用；字段名已核对）
```bash
tools/scripts/mapping/save_grid_map.sh                                            # /map -> map/<world>.pgm+yaml
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: 'src/bringup/rm_nav_bringup/map/RMUL2026.pbstream', include_unfinished_submaps: false}"
# 可选：直接导出子图栅格
ros2 service call /write_assets cartographer_ros_msgs/srv/WriteAssets "{stem: '/tmp/<world>_carto', image_format: 'png'}"
```
> 注：`cartographer_occupancy_grid_node` 的 `/map` 发布器是 `QoS(10).transient_local()`（已核对源码
> `occupancy_grid_node_main.cpp`）→ nav2 `static_layer`（默认 `map_subscribe_transient_local: true`）与
> `map_saver_cli` 都能收到，nav2 晚启动也不会漏图。

---

## 6. 场景六：cartographer 纯定位（`localization:=cartographer`，2026-09 接入槽位）

**资产前提**：需要 `map/<world>.pbstream`。先用**场景五**跑一次 cartographer 建图并 `finish_trajectory` +
`write_state` 生成（三个场地目前都还没有）。

**终端 A（一键，与 amcl/slam_toolbox/icp 同构）**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=nav lio:=fastlio localization:=cartographer nav:=rpp spin_speed:=0.0
```
接线（槽位实现细节）：
- cartographer 加载 `map/<world>.pbstream`（`load_frozen_state:=true`）→ **只发 `map→odom`**（契约与 amcl/icp 一致）；
- `odom→base_link` 仍由 LIO 提供；
- 它的栅格发到 **`/cartographer_map`**，`/map` 留给 `map_server` 的先验栅格图（避免双发布者）；
- 纯定位默认以 `map` 原点为起始猜测；而 cartographer 的 `map` 系原点 = **建图时的起点**，所以
  **建图与定位两次的出生点应一致**（`RMUL` / `RMUL2026` 都是 world 出生点 `(4.3, 3.35)`，两次同点起步即天然吻合）。
  出生点不同时只能靠全局搜索（`POSE_GRAPH.global_constraint_search_after_n_seconds`）——慢且可能失败；
  此时可用 `start_trajectory` 服务显式给初值（该 srv 有 `use_initial_pose` + `initial_pose` 字段）。
- ⚠️ **不要混用坐标系**：现有 `RMUL2026.pgm` 是**世界系**，而 cartographer 导出的 pgm/pbstream 是**出生点系**。
  重建 RMUL2026 后，§0.5 表里的 AMCL 初值也要从 `(4.3, 3.35)` 改成 `(0, 0)`。

**终端 B**
```bash
ros2 node list | grep cartographer                    # cartographer_node + occupancy_grid_node
ros2 run tf2_ros tf2_echo map odom                    # 由 cartographer 提供
ros2 topic echo /map --once --field info              # 先验栅格图（map_server，宽高=场地尺寸）
ros2 topic echo /cartographer_map --once --field info # cartographer 自己的栅格（可视/对比用）
ros2 lifecycle get /controller_server                 # active
```

> 若想脱离 bringup 单独调试 cartographer，可仍用
> `ros2 launch rm_nav_bringup cartographer_sim.launch.py configuration_basename:=cartographer_localization.lua load_state_filename:=<abs>.pbstream load_frozen_state:=true`
> （但那样没有仿真/LIO/nav2，仅适合看 cartographer 自身日志）。

> ⚠️ **资产前提（当前阻塞）**：三个场地**都没有可用的 pbstream** —— RMUL/RMUC 没有该文件，
> `RMUL2026.pbstream` 仅 526 B（空壳）。**本场景必须先自己生成** —— 方法就在本场景下方
> （`/finish_trajectory` → `/write_state` 两条服务调用，无需任何脚本）。
> 纯定位时若另有 `map_server` 在发先验 `/map`，请加 `occupancy_grid_topic:=/cartographer_map`，
> 避免两个发布者抢 `/map`（该参数默认已改为 `map`）。

---

### 6.1 全包形态：`lio:=cartographer`（cartographer 兼任里程计源，2026-09 新增）

**它是什么**：`lio` 槽取 `cartographer` 时，同一个 `cartographer_node` 既发 `odom→base_link`
（`provide_odom_frame=true` + `published_frame="base_link"`），又发 `map→odom`。
于是 `mapper` 槽、`localization` 槽、`lio_tf_adapter`、T1 静态桥**全部跳过**（launch 已按 `lio` 门控）。

```bash
# 建图（不写 mapper，写了也会被跳过）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping \
  lio:=cartographer spin_speed:=0.0 nav_rviz:=True
# 边建边导
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
  lio:=cartographer nav:=rpp spin_speed:=0.0 nav_rviz:=True
# 纯定位（需 map/RMUL2026.pbstream；localization 必须留空）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=cartographer nav:=rpp spin_speed:=0.0 nav_rviz:=True
```

**验收**：
```bash
ros2 run tf2_ros tf2_echo odom base_link       # 由 cartographer 发（不是 lio_tf_adapter）
ros2 run tf2_ros tf2_echo map odom
ros2 node list | grep -c fastlio               # 期望 0
ros2 node list | grep lio_tf_adapter           # 期望空
```

**三个限制**（详见 `docs/tf_interface_contract.md` §八）：
① `odom` 由 cartographer 的 pose extrapolator 提供，弱于 FAST-LIO 的紧耦合 IEKF
   → **2026-09-21 已缓解**：全包形态现在接一路底盘里程计（`use_odometry=true`，仿真用
   `/odom_ground_truth`、实车用下位机轮速 odom），见下面那段；
② **没有 `/odom` 话题** → `nav:=teb` 不适用（用 `rpp`/`dwb`）；
③ 失去独立故障域：cartographer 挂了 `odom` 与 `map` 一起没。

**⚠️ 全包形态必须有 odom（2026-09-21 实测修复）**

不接 odom 时，`odom→base_link` 只能靠 pose extrapolator（IMU 二次积分），**静止也会漂**：

| 实测（RMUL2026，车静止） | 值 |
|---|---|
| `odom→base_link` 平移漂移 | x +3.1 cm/s、y −2.5 cm/s → 合 **≈4 cm/s** |
| `odom→base_link` yaw 漂移 | 2.21° → 1.72° / 2.3 s ≈ **13°/min** |
| `map→odom` | 恒定 `(0.088, 0.075, −0.37°)`（说明**不是回环问题**，位姿图没动） |
| `/odom_ground_truth` twist | 全 0（车确实没动） |

后果：漂移被当作扫描匹配的初始猜测 → 地图被拖着走（RViz 里"车自己在动、地图跟着小车走"）。

**修法**：给 cartographer 接一路底盘里程计。配置已在仓库里（`cartographer_lio*.lua` 里
`use_odometry = true`，bringup 用 `odom_topic:=/odom_ground_truth` remap）——**直接重跑上面的命令即可**。
验证：

```bash
ros2 topic list | grep odom_ground_truth      # 确认有这路 odom
ros2 run tf2_ros tf2_echo odom base_link      # 静止 30 s：应几乎不动（之前是 ~4 cm/s 匀速漂）
```
> 仿真这路是 Gazebo 底盘真值（相当于**理想轮速里程计**）。实车换成下位机轮速 odom 时，
> 因为有滑移/噪声，要把 `POSE_GRAPH.optimization_problem.odometry_translation/rotation_weight`
> 从 `1e5` 调小（例如 `1e3`），否则回环拉不动轨迹。

---

## 7. 局部规划器变体（RPP / DWB / TEB）

```bash
# 统一加 spin_speed:=0.0（先把小陀螺关掉，只比控制器本身，见 §0.4.1）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=rpp spin_speed:=0.0
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=dwb spin_speed:=0.0
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=teb spin_speed:=0.0
# 想对比"全局障碍来源"再叠加：global_obstacle:=stvl（默认）/ scan / none
```

> 变体文件位置：`src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_{rpp,dwb,teb}.yaml`。
> **比较控制器时要固定其他维度**（同一场地/同一 LIO/同一重定位/同一 `spin_speed`/同一目标点），
> 否则比出来的差异不归控制器。

---

### 7.1 全局障碍来源 A/B（`global_obstacle`，2026-09 新增槽位）

```bash
# A：3D 体素层（默认）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 global_obstacle:=stvl
# B：2D /scan（与 local 同源）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 global_obstacle:=scan
# C：只用 static + inflation（全局不看实时障碍）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 global_obstacle:=none
```
**运行期也能切**（不用重启整场仿真，两个图层都支持 `enabled` 动态参数）：
```bash
ros2 param set /global_costmap/global_costmap.stvl_layer.enabled false
ros2 param set /global_costmap/global_costmap.obstacle_layer.enabled true
```
**要比较的三件事**（结果记入 `docs/algorithm_matrix.md` §四）：
1. **一致性**：造一个 0.1 m 矮台用例，看 local 与 global 判断是否一致（`scan` 模式应一致）；
2. **行为**：global 路径是否绕开临时障碍、是否出现"幽灵障碍"（`scan` 模式量程已修为 10 m）；
3. **资源**：`top` 看 `component_container_mt` 的 CPU（`stvl` 明显更重）。

---

### 7.2 局部障碍来源 A/B（`local_obstacle`，2026-09 新增槽位）

```bash
# A：只吃 /scan（默认 = 原行为）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 local_obstacle:=scan
# B：只吃点云直投（不经 p2l → 无 45cm 盲区）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 local_obstacle:=cloud
# C：双源冗余（任一路挂掉仍能避障）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 local_obstacle:=both
# D：局部也上 3D 体素层（STVL，带 0.5 s 时间衰减；= COD 2026 双图做法）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=small_point_lio \
  localization:=gicp nav:=mppi planner:=smac2d spin_speed:=0.0 local_obstacle:=stvl
# E：三层全开（stvl + scan + cloud；最贵，未做 bench）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=small_point_lio \
  localization:=gicp nav:=mppi planner:=smac2d spin_speed:=0.0 local_obstacle:=stvl_both
```

`stvl` / `stvl_both` 的实测结论（A/B 四次跑，含贴墙终点专项）、参数来源、代价与未验证清单：
**`docs/stvl_local_costmap.md`**。一句话：导航结果与 `scan` 不可区分、RTF 代价 1~3%，
但局部图 lethal 核心会从连续线变成周期 0.05 m 的点状线 ⇒ 默认仍推荐 `scan`。
槽位 ↔ 图层开关的唯一真值表：`rm_navigation/launch/navigation_launch.py` 的
`LOCAL_OBSTACLE_LAYER_TABLE`；不起栈就能自查：`python3 tools/scripts/regress/local_obstacle_truth_table.py`。

**这一槽位解决什么**：`/scan` 是**串行单点**（插件 → `linefit` → `p2l`）；
`cloud` 这一路直接吃 `/segmentation/obstacle`（3D 障碍点云），**绕过 `p2l`** → 破单点。
注意它是**部分冗余**：仍依赖 `linefit`（点云来源），只是不再依赖 `p2l`。

> ★ 2026-10-05 更正：原文写『`p2l` 有 45 cm 盲区』——`p2l` 的 `range_min` 已从 0.45 降到 **0.05**
> （2026-09-26），盲区**已不存在**：实测车停到离墙 0.35 m 时 `/scan` 仍有 **35 个回波**、
> local costmap 也照样标到墙（见 `docs/stvl_local_costmap.md` §3.2）。`cloud`/`both` 现在的
> 价值主要在**破 `p2l` 单点**，不再在『消盲区』。

运行期也能切（两个图层都支持 `enabled` 动态参数，不必重启整场仿真）：
```bash
ros2 param set /local_costmap/local_costmap.obstacle_layer.enabled false
ros2 param set /local_costmap/local_costmap.obstacle_cloud_layer.enabled true
```

**验证要点**：
1. `ros2 topic info /segmentation/obstacle --verbose | grep "Node name"` → 应出现 local costmap（`both`/`cloud` 时）；
2. 把车停到**离墙 0.3 m**（`scan` 模式看不见）→ `cloud`/`both` 模式下 local costmap 应出现障碍格；
3. 比 CPU（多一个 3D 点云消费者）。

---

### 7.3 失效检测 / 降级验证（`expected_update_rate`，2026-09 修复）

**为什么要有**：障碍源停发时 nav2 默认**不检查**（`expected_update_rate: 0`），缓存会永远重放最后一帧
→ costmap 冻住、无报错、车继续走（**静默失效**）。修复后应变成"报警 + 停车"。

```bash
# ① 正常：/scan 在发
ros2 topic hz /scan
# ② 手动断掉感知链（也可以断更上游：pkill -f ground_segmentation）
pkill -f pointcloud_to_laserscan
# ③ 期望：
#    - 日志出现 The /scan observation buffer has not been updated for X seconds,
#      and it should be updated every 0.50 seconds.
#    - ~1 s 内 /cmd_vel 归零（velocity_smoother 的 velocity_timeout: 1.0）
ros2 topic hz /cmd_vel
```

---

## 8. 全矩阵指令

> 下面矩阵是**核心维度**（场地 × LIO × 重定位/建图后端 × 局部规划器）。另外三个装配级开关会成倍影响行为，
> A/B 时**一次只动一个**：`spin_speed`（5.0 哨兵小陀螺 / 0.0 直通，见 §0.4.1）、
> `global_obstacle`（stvl / scan / none，见 §0.7 与 `docs/3d_to_2d_survey.md` §七）、
> `local_obstacle`（scan / cloud / both，见 §7.2）。

### 8.1 mapping（3 场地 × 2 LIO）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping lio:=fastlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping lio:=pointlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL     mode:=mapping lio:=fastlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL     mode:=mapping lio:=pointlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC     mode:=mapping lio:=fastlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC     mode:=mapping lio:=pointlio
```

### 8.2 nav × RMUL（2 LIO × 3 重定位）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio  localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio  localization:=slam_toolbox
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio  localization:=icp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=pointlio localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=pointlio localization:=slam_toolbox
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=pointlio localization:=icp
```

### 8.3 nav × RMUC（2 LIO × 3 重定位）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=fastlio  localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=fastlio  localization:=slam_toolbox
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=fastlio  localization:=icp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=pointlio localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=pointlio localization:=slam_toolbox
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=pointlio localization:=icp
```

### 8.4 nav × RMUL2026（仅 AMCL：无 posegraph、无 pcd）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio  localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=pointlio localization:=amcl
```

### 8.4.1 slam_nav（边建图边导航：3 场地 × 2 LIO × 2 建图后端）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav lio:=fastlio  mapper:=slam_toolbox  nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav lio:=pointlio mapper:=slam_toolbox  nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav lio:=fastlio  mapper:=cartographer   nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL     mode:=slam_nav lio:=fastlio  mapper:=slam_toolbox  nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL     mode:=slam_nav lio:=fastlio  mapper:=cartographer   nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC     mode:=slam_nav lio:=fastlio  mapper:=slam_toolbox  nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC     mode:=slam_nav lio:=fastlio  mapper:=cartographer   nav:=rpp
# 局部规划器变体同样适用：在 slam_nav 下追加 nav:=dwb / nav:=teb
```

### 8.5 特殊场景
```bash
# 关闭 LIO（预期：无 /odom、无 odom→base_link；nav 报 TF 超时 = 关闭生效）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=none localization:=amcl
# 不开 RViz
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav_rviz:=False
# 开 LIO 点云可视化
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl lio_rviz:=True
```

---

## 9. 判据速查

### 9.1 「导航到底就绪没有」一把梭（在终端 B 依次跑）

```bash
ros2 node list | grep -E "amcl|map_server|controller_server|planner_server"
ros2 lifecycle get /controller_server; ros2 lifecycle get /planner_server     # 期望 active
ros2 topic echo /map --once --field info                                     # 期望 宽x高 与你选的场地一致
ros2 topic hz /scan                                                          # ★ 期望稳定有频率（关键！见 9.2）
ros2 run tf2_ros tf2_echo odom base_link                                     # LIO 里程计
ros2 run tf2_ros tf2_echo map odom                                           # ★ 重定位模块输出
ros2 topic echo /amcl_pose --once                                            # amcl 模式下：position 应≈初值
ros2 topic info /scan --verbose                                              # 看订阅者与其 QoS（排查 incompatible QoS）
# 全局障碍来源确认（global_obstacle 槽位，二选一生效）
ros2 param get /global_costmap/global_costmap.stvl_layer.enabled              # stvl 模式应为 True
ros2 param get /global_costmap/global_costmap.obstacle_layer.enabled          # scan 模式应为 True
```

### 9.2 ⚠️ `map→odom` 依赖 `/scan`（源码级结论，实测确认）

`nav2_amcl` 里 `sendMapToOdomTransform()`（`amcl_node.cpp` L1007）**只被 `laserReceived()` 调用**（L716/L728），
且开头 `if (!initial_pose_is_known_) return;`。所以：

> **amcl 必须同时满足 ①已加载地图 ②已知初始位姿 ③收到 `/scan`，才会发布 `map→odom`。三条缺一，`map` 帧就根本不存在。**

沙箱无头实测（只跑 map_server+amcl，无 Gazebo）：

| 条件 | `tf2_echo map odom` |
|---|---|
| 有地图 + 自动初值，**无 `/scan`** | `Invalid frame ID "map" ... frame does not exist` ← 与"看起来没反应"完全一致 |
| 再补上 `/scan` | 立刻输出 `map→odom = 平移(4.3, 3.35)`，`/amcl_pose` = `x:4.30 y:3.35` ✅ |

所以出现 `map` 帧不存在时，**先查 `ros2 topic hz /scan`**，而不是怀疑初值。

### 9.3 常规判据

| 检查 | 正常表现 |
|---|---|
| `tf2_echo odom base_link` | 持续输出、随车移动，无 `Invalid frame ID` |
| `tf2_echo map odom`（nav 模式） | 由所选重定位模块提供（amcl / slam_toolbox / icp / cartographer）；**前提见 9.2** |
| `topic hz /odom` | fastlio ≈10Hz；pointlio 更高（数十~100Hz） |
| `view_frames` | 主链 `map→odom→base_link→base_link_fake→…`、`base_link→livox_frame…`；`camera_init→body` 为孤岛（正常） |
| `lifecycle get /controller_server` | `active` |
| `/map` 的 `info` | 宽高 = 对应场地的 pgm 尺寸（map_server 加载**磁盘上的既有地图**，不是实时扫描结果） |
| `tools/check_map_reachable.py` | 选目标点前跑：能判"起点/目标是否同一连通域"、推荐可达点、也能当**幽灵墙检测器**（连通域被切成多块=异常） |
| `tools/pcd_to_grid_map.py` | 离线把 LIO 的 `.pcd` 投成 `.pgm`（第三条建图路线）；`--compare` 可与已有栅格图比对结构一致性 |
| `global_obstacle` 生效确认 | `ros2 param get /global_costmap/global_costmap.{stvl_layer,obstacle_layer}.enabled` —— 两个里恰有一个 True |
| `local_obstacle` 生效确认 | `ros2 param get /local_costmap/local_costmap.{obstacle_layer,obstacle_cloud_layer}.enabled` —— `scan` → 前者 True；`both` → 都 True |
| `tools/scripts/regress/nav_smoke_regression.py` | ⭐ **P0 验收闸门**（链路体检 → 发目标 → 命令链/结果断言 → JSON 快照）；发目标语义已对齐"人在 RViz 里点目标"，见 **§9.4** |

### 9.4 P0 回归工具 `nav_smoke_regression.py`：**发目标语义 = 人在 RViz 里的做法**（2026-09 新增）

`tools/scripts/regress/nav_smoke_regression.py` 是每个候选组合的验收闸门（链路就绪 → 发目标 → 四跳命令链/真值 → 结果断言 → JSON 快照）。
2026-09 之前它"链路刚就绪就发目标 + 目标朝向写死 `w=1.0`（yaw=0，即到点朝地图东）"，实测产生过**误导性 FAIL**：
同一条链路，人手动在 RViz 里点 `2D Goal Pose` 却正常。现在它按人类的三个习惯来（三个新旗标）：

| 新旗标 | 默认 | 为什么存在（人类语义 / 发目标时机 / 目标可用性） |
|---|---|---|
| `--yaw {auto\|<rad>}` | `auto` | **人类语义**：人在 RViz 里拖 `2D Goal Pose`，箭头默认朝**车当前朝向**，而旧工具写死 yaw=0。`auto` 用工具本来就在跟踪的三条 TF（`map→odom` ∘ `odom→base_link` ∘ `base_link→base_link_fake`）做**平面复合**（只算 yaw：`p_map=p_a+R(yaw_a)·p_b`、`yaw=yaw_a+yaw_b`，**不引入 tf_transformations**）取车在 `map` 系里的当前朝向；TF 还不全时依次退回"车→目标方向"、"yaw 0"，并在输出与 JSON 里写明来源（`yaw_source: tf\|to_goal\|fallback`，`--yaw <rad>` 时为 `explicit`）。注意 goal checker 已是 `PositionGoalChecker`（位置-only）⇒ 朝向只影响"像不像人"，不影响成败判定；`--yaw 0` 可复现旧行为做 A/B |
| `--settle <sec>` / `--settle-timeout <sec>` | `3.0` / `30.0` | **发目标时机**：人点目标前会先确认定位稳了（粒子云收敛、ICP 配准完、SLAM 没在跳变）。链路就绪后、发目标前，等 `map→odom` 在最近 `--settle` 秒内平移漂移 ≤0.02 m、yaw 漂移 ≤0.01 rad（~10 Hz 轮询 + 每秒打印进度）；超过 `--settle-timeout` 只打 **WARNING** 并继续发目标（JSON 记 `settled:false` + 实测漂移），**不会静默跳过**。理由是实测里"在 localizer 收敛瞬态发目标"会被 nav2 ABORT，看起来像导航坏了。`--settle 0` = 不等（旧行为） |
| `--no-precheck`（配套 `--map-topic` / `--map-timeout`） | 关（即默认**做**预检） | **目标可用性**：先读先验地图 `/map`（RELIABLE + TRANSIENT_LOCAL，map_server 是 latched），按 `info.resolution/origin/data` 把目标格分类 `free`/`occupied`/`unknown`/`out_of_map`（0=free、100=occupied、-1=unknown，与 `tools/scripts/nav/segment_goal_navigator.py` 同一套规则）。**非 free 就打印可操作错误并 exit 3、不发目标**（"请换一个 free 点；这不算导航失败"）—— 把"选点不对"和"导航链坏了"彻底分开；`/map` 收不到（话题名不同 / 发布端非 latched）只告警并跳过预检，仍然发目标 |

**用法示例**（栈先在终端 A 跑起来；选点先看 §0.6，同一连通域）：

```bash
# 默认：等定位稳定 → 预检目标 → 以"车当前朝向"发目标（复跑曾经误导的用例）
python3 tools/scripts/regress/nav_smoke_regression.py --goal 1.0 -1.0
# A/B 与单独用法
python3 tools/scripts/regress/nav_smoke_regression.py --goal 1.0 -1.0 --yaw 1.57 --settle 5
python3 tools/scripts/regress/nav_smoke_regression.py --skip-goal                 # 只体检链路（不停留等稳定/预检）
python3 tools/scripts/regress/nav_smoke_regression.py --goal 1.0 -1.0 --no-precheck
```

**退出码**：`0`=PASS；`1`=FAIL（链路/导航，看 `fails[0]` 首个断点）；**`3`=目标在图上 occupied/unknown/图外（未发目标，不算导航失败）**
—— 这一种情况下 JSON 里 `pass:false`（没验证过导航）但 `nav_failure:false`、`result:"goal_rejected"`、`abort` 写明原因，别把它当成"导航失败"。
**JSON 快照**（`.tmp_bags/regress_<ts>.json`）在原有字段（`pass/fails/notes/rtf/hz/cmd/goal/...` 名字与含义不变）之外新增：
`goal_yaw` / `goal_yaw_deg` / `yaw_arg` / `yaw_source` / `yaw_note`、`settle{enabled,settled,waited,drift_xy,drift_yaw,samples,settle_sec,timeout_sec}`、
`map_check{enabled,checked,topic,state,value,cell,map}`、`result` / `exit_code` / `nav_failure` / `abort`。


---

## 10. 错误对照

| 现象 | 原因 | 处理 |
|---|---|---|
| `Package 'rm_nav_bringup' not found ... searching: ['/opt/ros/humble']` | 新终端**只 source 了 /opt/ros，没 source 工作区** | `source ~/HzMi_rmsimulation/install/setup.bash`（见 §0.1；可写进 `~/.bashrc`） |
| `view_frames` 里看到 `camera_init→body` 孤立小岛 | LIO 仍广播内部帧；T3 后导航层不再使用（`odom→base_link` 由 `lio_tf_adapter` 提供） | 正常现象，无需处理（回退用法 `localization:=''` 仍依赖它；**T6 已撤销**，`base_link_fake` 不再移除） |
| `Robot is out of bounds of the costmap!`（仅启动时出现几次） | slam_toolbox 地图尚在生长、global_costmap 正在 resize 的瞬态 | 若**持续刷屏**再排查 `map→odom`（`ros2 run tf2_ros tf2_echo map base_link`） |
| RViz 里**车/雷达看起来是斜的**，但 Gazebo 里车是正的 | spawn 高度过高：RMUL/RMUL2026 原来写 `z=1.16`，而地面在 z≈0（轮半径 0.06 → 落地时 base_link 仅 0.06 m），机器人**悬空 1.1 m 落下**，FAST-LIO 在坠落中做重力初始化 → 地图/位姿倾斜 | **已于 2026-09 修复**：`rm_simulation.launch.py` 中 RMUL / RMUL2026 的 spawn `z` 改为 **0.2**。验证：`ros2 run tf2_ros tf2_echo odom base_link` 的 roll/pitch 应≈0 |
| nav 模式卡在 `amcl: Waiting for map....` / `global_costmap: Invalid frame ID "map"` | ① `map_server_launch.py` 与 `localization_amcl_launch.py` **各起了一个同名 `lifecycle_manager_localization`**（冲突）；② `nav2_params_sim_*.yaml` 里 `yaml_filename` 被注释掉，而 nav2 的 `RewrittenYaml` **只替换已存在的键** → map_server 报 `parameter 'yaml_filename' is not initialized` | **已于 2026-09 修复**：① amcl launch 现在用**单一 lifecycle_manager 同时管理 `map_server`+`amcl`**，bringup 仅在 `icp`/未选重定位时单独起 map_server；② 三份 nav2 参数恢复 `yaml_filename: ""` 键（launch 会注入实际地图路径） |
| Nav2 在 `odom`/`map` 出现前就激活，刷 `Timed out waiting for transform ...` | Gazebo 生成机器人 + LIO 初始化需要数秒，而 Nav2 立即启动 | **已于 2026-09 缓解**：`bringup_sim` 中定位链延后 **4s**、mapping 后端延后 **4s**、Nav2 延后 **10s** 启动 |
| `spawn_entity: Spawn status: ... timed out waiting for entity to appear` | RMUL2026 世界加载慢，spawn 默认超时过短（实体其实已生成） | **已于 2026-09 修复**：spawn 参数加 `-timeout 60.0` |
| `cartographer_node` 启动几秒后 `exit code -6`，日志 `Check failed: ... The IMU frame must be colocated with the tracking frame` | `tracking_frame="livox_frame"`，而 `/livox/imu` 的 frame 是 `imu_link`，URDF 里两者差 5cm → cartographer 的 **IMU 共位硬 CHECK** 失败（`sensor_bridge.cpp:136`） | **2026-09-21 已修**：`tracking_frame = "imu_link"`（官方 `mir-100-mapping.lua` 同做法）。沙箱 A/B 验证：隔离 domain + 合成 TF/IMU 消息下，旧配置必 abort、新配置通过 |
| `cartographer_node` 加载纯定位 lua 即 FATAL：`Key 'pure_localization' was used the wrong number of times` | `cartographer_localization.lua` 把 `pure_localization`/`pure_localization_trimmer` 写在 `TRAJECTORY_BUILDER_2D`，而 cartographer 从**顶层 `TRAJECTORY_BUILDER`** 读（`trajectory_builder_interface.cc` 的 `kDictionaryKey`）；写错层级的键永不被读，而它要求"每键读恰好一次" | **2026-09-21 已修**：改成 `TRAJECTORY_BUILDER.pure_localization_trimmer = {...}`（官方写法）；deprecated 的 bool 不再设 |
| `spawn_entity: Spawn service failed. Exiting.` + 随后 `local_costmap: ... "odom" ... frame does not exist` 一直刷、Nav2 永不激活 | `spawn_entity` 放弃后**机器人其实晚了 3~5 秒才被插入**（gzserver 仍会打印 `mecanum_controller: Subscribed to [/cmd_vel_chassis]`、`LivoxPointsPlugin: ros topic name: /livox/lidar`）。而 Nav2 在 `odom` 帧出现前激活不了；若在此期间 LIO 还没吐 `/odom`，就一直是这个循环。**RMUL 场地网格 44 万三角面（RMUL2026 仅 3187），加载明显更慢** | 不要 20 秒就下结论：**给 30~60 秒**再判断。用 `ros2 topic hz /livox/lidar`（CustomMsg 10Hz）→ `/livox/imu`（100Hz）→ `/imu/data`（互补滤波输出 100Hz，FAST-LIO 的 `imu_topic`）→ `/odom`（≈10Hz）**逐段定位**；`/odom` 一出，`odom` 帧即有、local_costmap 立即恢复、Nav2 自行激活。若 `/livox/lidar` 有数据而 `/odom` 始终没有 → 换 `lio:=pointlio` 做 A/B（它用原始 `/livox/imu`）以区分是雷达侧还是 FAST-LIO+互补滤波支路 |
| `Timed out waiting for transform from base_link to map` | `map→odom` 缺失 | nav 模式必须指定 `localization:=amcl\|slam_toolbox\|icp\|cartographer` |
| `Invalid frame ID "base_link"` / fake_vel 报 `Could not transform odom to base_link` | `/odom` 无数据 → LIO 或 `lio_tf_adapter` 未启动 | 查终端 A 是否打印 `lio_tf_adapter 启动` |
| `Found two parents` / 帧树分叉 | 旧进程残留 | 执行 §0.2 清理后重跑 |
| `nav:=teb` 插件找不到 | 未 source 工作区 | `source install/setup.bash`（teb 已编译） |
| `TF_OLD_DATA` | 仿真时间不一致 | 确认 `use_sim_time:=True`（launch 默认） |
| `Unable to parse the value of parameter robot_description as yaml` | launch_ros 把 URDF(XML) 当 YAML 解析（Humble 行为） | **已于 2026-09 修复**：`hzmi_rm_simulation/launch/rm_simulation.launch.py` 用 `ParameterValue(robot_description, value_type=str)` 包裹；若仍出现，说明用的是修复前的 launch |
| `ModuleNotFoundError: No module named 'numpy'` 且 `spawn_entity` 退出 | 用了 conda 的 python（ROS Humble 需系统 python 3.10） | 见 §0.0①：`conda deactivate` 或 `export PATH=/usr/bin:$PATH` |
| `Could not find requested resource in ament index`（nav2 组件成批加载失败） | Nav2 主体包未安装 | 见 §0.0②：apt 安装 navigation2 / nav2-bringup / nav2-rviz-plugins |
| RViz 报 `nav2_rviz_plugins/... does not exist` | 缺 `nav2-rviz-plugins` | 同上 |
| `New subscription discovered on topic '/scan', requesting incompatible QoS` | ① costmap `obstacle_layer` 默认 reliable，而 `/scan` 是 best-effort；②**残留的上一轮进程/RViz** 用旧参数（reliable）订阅 | ① **已于 2026-09 修复**：三份 `nav2_params_sim_{rpp,dwb,teb}.yaml` 的 scan 源加了 `reliability_policy/qos_policy: best_effort`；② 跑之前先执行 §0.2 清理；③ 用 `ros2 topic info /scan --verbose` 看清订阅者，本仓库内 `nav2.rviz` 对 `/scan`、`/particle_cloud` 都声明的是 **Best Effort**（与 best-effort 发布方兼容），这条告警多为 RViz 启动瞬间的瞬态，不影响链路 |
| `tf2_echo map odom` 报 `Invalid frame ID "map" ... frame does not exist`，但 `/map` 有数据 | **amcl 还没发布 `map→odom`**：缺 `/scan`、或初值未知、或地图未收到 —— 三者任一都会导致 `map` 帧不存在（源码级解释与实测见 §9.2） | 先 `ros2 topic hz /scan`；再 `ros2 param get /amcl set_initial_pose`；再确认 amcl 日志有 `Received a W X H map` 与 `Setting pose (...)` |
| `global_obstacle:=scan` 下 global 图上出现幽灵障碍（规划绕开不存在的东西） | 照抄了 local 的 `raytrace_max_range: 6.0`，而 global 是**全图**（13×10 m）→ 6 m 以外的旧标记永不被清除 | **2026-09 已修**：global 的 scan 源量程改为 **10.0 m**（与 p2l 的 `range_max` 对齐）—— 原则是「扫描能看到多远，就要能清多远」 |
| **RViz 里一开始就有很完整的地图**，不是慢慢扫出来的 | nav 模式下那是 `map_server` 加载的**磁盘既有 pgm**（`src/bringup/rm_nav_bringup/map/<world>.pgm`），不是实时建图 | 正常。想看实时建图用 `mode:=mapping`（slam_toolbox/cartographer）；nav 模式要判断定位对不对，看 **scan/costmap 与这张地图的墙体是否重合** |
| costmap 刷屏 `Sensor origin at (x,y) is out of map bounds`（数值在 spawn 坐标与 LIO 估计间跳变） | **同一 `odom` 有两个来源**：Gazebo 真值 + LIO 都发到 `/odom`，`lio_tf_adapter` 交替收到两者 → `odom→base_link` 抖动 | **已于 2026-09 修复**：`sentry_robot_sim.xacro` 把 Gazebo 真值 remap 到 **`/odom_ground_truth`**（`publish_odom_tf=false`），`/odom` 由 LIO 独占。对比真值请看 `/odom_ground_truth` |
| RViz 报 `Message Filter dropping message ... queue is full` 且退出时 `rviz2 exit code -11` | RViz 在高频 TF/点云负载下丢帧，退出时崩溃（常见现象，不影响仿真链路） | 可先 `nav_rviz:=False` 验证导航链路；或减少 RViz 中 PointCloud2/STVL 显示项 |
| **发目标后车不走**，`/cmd_vel` 长时间只有 `linear.x: -0.05`（偶尔 `0` + 角速度） | `-0.05` = nav2 自带 BT `BackUp` 的 `backup_speed="0.05"`（见 `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`）→ **BT 进了恢复行为循环**（清代价地图→Spin→Wait→BackUp 轮转），说明 `ComputePathToPose` 或 `FollowPath` 连续失败 | 按顺序查：① 终端 A 里 `planner_server` / `controller_server` 的 WARN/ERROR（**最直接**，会写明原因）；② `ros2 topic echo /plan --once --field poses`（空=规划失败）；③ `ros2 topic echo /odom_ground_truth --field pose.pose.position --once` 与 `/amcl_pose` 对比（RMUL2026 的 map 系=世界系，两者必须接近）；④ `spin_speed:=0.0` 重跑（见 §0.4.1） |
| `mapper:=cartographer` 时 costmap 没有静态图 / `map_saver_cli` 存不到图 | cartographer 的栅格被硬 remap 到 `/cartographer_map`，而 nav2 `static_layer` 与 `map_saver_cli` 都订阅 `/map` | **已于 2026-09 修复**：`cartographer_sim.launch.py` 默认发 `/map`；纯定位另有 `map_server` 时用 `occupancy_grid_topic:=/cartographer_map` 覆盖 |
| 纯建图 `mode:=mapping` 下 `nav_rviz:=True` 却没有任何 RViz | 三种形态拆分后 `mapping` 不再启动 nav2，而 RViz 原先由 nav2 的 `rviz_launch` 带起 | **已于 2026-09 修复**：bringup 为 `mode=='mapping' and nav_rviz=='True'` 单独补一个 RViz（`nav2.rviz`） |
| `mapper:=cartographer` 建出来的图**几乎是空的**（526 B pbstream / `/map` 没东西） | 三处对接错误：① `points2` 被 remap 到 `/livox/lidar`（**CustomMsg**，cartographer_ros 只支持 `sensor_msgs/PointCloud2`）→ **静默收不到数据**；② `published_frame="body"` + `provide_odom_frame=true` 与 FAST-LIO 争 `body` 子帧；③ `min_z/max_z` 被当成“相对地面”（实际**相对传感器**）→ 只切到墙顶 | **已于 2026-09 修复**（改走 `/scan`、`published_frame="odom"`、带改回 -0.8~2.0）；判据：`ros2 topic info /scan --verbose` 里应出现 `cartographer_node` |
| `planner_server: GridBased: failed to create plan with tolerance 0.50` / `Planning algorithm GridBased failed to generate a valid path to (x, y)` | **目标点与起点不在同一连通域**（中间被墙隔开），或起点/终点落在膨胀带内。地图上 1 像素宽的虚线经 `robot_radius` 膨胀后就能封死走廊 | 先跑 `tools/check_map_reachable.py --map <map.yaml> --start <map 系起点> --goal <目标>` 判定；换用同一连通域的目标点（§0.6）。若是**地图资产**问题（例：RMUL2026.pgm 的 x≈5.2 幽灵虚线，世界网格里没有实体）→ 重新建图 |
| 参数写在 `nav2_params_*.yaml` 里却"没生效" | **节点名对不上**：nav2 跨版本改过名（如 Galactic `recoveries_server` → Humble **`behavior_server`**；`recovery_plugins` → `behavior_plugins`；插件类型 `nav2_recoveries/*` → `nav2_behaviors/*`）。对不上的整段被**静默忽略**，节点改用内置默认值 | 拿 `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml` 的顶层键做参照逐个核对；**已于 2026-09 修复**三份 sim 变体的 `recoveries_server` 段。判断某段是否生效的最快办法：看节点启动日志（如 `behavior_server: Creating behavior plugin ...` 的**个数/名字**是否与 yaml 一致） |
| FAST-LIO 在 `Ctrl+C` 时报 `exit code -11` | FAST-LIO 已知的退出崩溃 | 忽略；不影响运行期 |
| cartographer 纯定位报找不到状态文件 | **三个场地都没有可用 pbstream**（RMUL/RMUC 无该文件；`RMUL2026.pbstream` 仅 526 B 空壳） | 先按 §5 跑一次 cartographer 建图并 `finish_trajectory` + `write_state` 导出（路径用绝对路径，见 §6） |

### 10.1 现象 → 归属（先分类，再查上表）

| 现象 | 归属 | 一句话判据 |
|---|---|---|
| 点云转瞬即逝 / 手遮挡很明显 | **RViz 显示设置** | PointCloud2 的 Decay Time = 0（只画最新帧），不是地图问题 |
| 改了"刷新率"后点留住了 | **RViz 显示缓冲** | 那是屏幕缓冲，不是建图（见 `docs/architecture.md` §3.2.6） |
| 房间点云**跟着车跑** | **帧树 / 显示系 / 里程计** | 显示项是不是车体系？Fixed Frame 对吗？`tf2_echo` 链通吗？ |
| 地图**缓慢弯折 / 闭环处双墙** | **建图（缺回环）** | 绕一圈看闭环误差与双墙；LIO 无回环 |
| 位姿**突然跳到另一处** | **重定位** | `tf2_echo map odom` 是"跳"；看 `/particle_cloud` 是否多峰 |
| 位姿**高频小抖**（cm/度级来回） | **时序 / 刷新** | 时间戳不同步、TF 与 costmap 更新节奏不匹配 |
| 位姿**连续缓慢漂** | **里程计（LIO）** | 与 `/odom_ground_truth` 比，误差随距离增长 |
| 目标点规划失败（`failed to generate a valid path`） | **地图资产 / 选点** | 先 `check_map_reachable.py`；再看地图有没有幽灵结构 |
| **costmap 冻住**：车照原速继续走、日志无报错、RViz 里 costmap 还在刷新 | **静默失效**：障碍源（`/scan` 链）已断，而 nav2 的 `expected_update_rate` 默认 `0` = 不检查；`ObservationBuffer` 会一直重放最后一帧 | **2026-09 已修**：障碍源加 `expected_update_rate: 0.5` → 源停即 WARN + 拒绝算速度 + 1 s 内停车（`velocity_timeout`）。定位顺序：`ros2 topic hz /scan`（发了吗）→ `ros2 topic info /scan --verbose`（谁在收）→ `ros2 node list`（`linefit`/`p2l` 还在吗）→ §7.3 |
| 贴身 0.3 m 的障碍看不见、且那个方向被清成 free | `p2l` 的 `range_min`（原 0.45）把近点**丢弃**，bin 保持 `inf`，而 `inf_is_valid: true` 让 nav2 当"10 m 处有回波"→ 沿射线清空 | **2026-09 已修**：`range_min: 0.45 → 0.2`（与车体半径/`linefit r_min` 对齐）；需要更早发现就用 `local_obstacle:=cloud\|both`（点云直投无此盲区） |
| RViz 里**整张地图跟着小车转/平移，还留残影** | 先分清"显示"还是"真转"：① RViz `Global Options → Fixed Frame` 不是 `map`（或 `Views → Current View → Target Frame` 选了 `/base_link`）时，整个场景会跟着车体坐标系转 —— **地图其实没动**（`/map` 的单元格固定发布在 `map` 系）；② `ros2 run tf2_ros tf2_echo map odom` 若在转弯/回环时**突然跳到几十度**，那才是位姿图在修正 = **误回环**（RM 场地小且对称），地图整体被拧并留残影 | ① 把 `Fixed Frame` 设成 `map`、`Target Frame` 设成 `<Fixed Frame>`；② 误回环 → **2026-09-21 已收紧回环参数**：`max_constraint_distance 10→4`、`min_score 0.55→0.72`、`global_localization_min_score 0.6→0.8`、`fast_correlative_scan_matcher` 搜索窗 `5m/20°→2m/10°`、`global_constraint_search_after_n_seconds 10→30`。判据：重跑后 `map→odom` 应始终 ≈0 |
| RViz 里**栅格地图绕着小车转**（尤其车原地转时），但 `map→odom` 一直是 ≈0/恒定 | **`lio_tf_adapter` 丢了杆臂**：它把 LIO 的 `body`（= 雷达/IMU，在 `base_link` 前 0.12 m、上 0.125 m）位姿直接改名成 `base_link` 发布（配置里 `xyz: [0,0,0]`）→ 车原地转时"base_link"绕真实中心画半径 **≈0.17 m 的圆**，地图静止 → 相对运动看起来就是地图在转。同根因公开案例：nav2 issue [#4135](https://github.com/ros-navigation/navigation2/issues/4135)（雷达装偏 → 转圈时 map 跟着画圈） | **2026-09-21 已修**：`lio_tf_adapter/config/lio_tf_adapter.yaml` 的 `xyz: [-0.12, 0.0, -0.125]`（FAST-LIO 与 Point-LIO 的 `extrinsic_T` 都是 `[0,0,0.05]`，同一套杆臂，改一处即可）。**验证**：原地转 360°，`ros2 run tf2_ros tf2_echo map base_link` 的 x/y 应基本不动（修前会画圆） |
| 车不走但 `/cmd_vel` 有 -0.05 + 旋转 | **BT 恢复行为循环** | 说明规划或控制连续失败（见上表对应行） |
| `/cmd_vel_chassis` 出现 `angular.z: 5.0` | **小陀螺在动作** | 那是 `spin_speed`，不是 nav2 发的角速度（§0.4.1） |

---

## 11. 出问题时提供什么

```bash
# 1) 运行日志
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl 2>&1 | tee /tmp/run.log

# 2) 三条关键输出（各贴几行即可）
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /odom
ls -l ~/HzMi_rmsimulation/frames.pdf

# 3) 或录 20 秒包供离线分析（Ctrl+C 结束）
ros2 bag record -o /tmp/smoke /odom /tf /tf_static /scan /cmd_vel
```

**按"归属"收集证据（配合 §10.1）**

```bash
# 时序/刷新类：看 map→odom 的时间戳是否连续、频率是否稳定
ros2 topic hz /tf; ros2 run tf2_ros tf2_echo map odom
# 里程计类：与真值对比（RMUL2026 的 map 系=世界系，可直接比；RMUL/RMUC 只比相对变化）
ros2 topic echo /odom_ground_truth --field pose.pose.position --once
ros2 topic echo /amcl_pose --field pose.pose.position --once
# 重定位类：粒子云是否多峰（RViz 看 /particle_cloud），或 ICP 是否在配准
ros2 node list | grep -E "amcl|icp_registration"
# 建图类：闭环误差（绕一圈回起点）与"双墙"
```

**当前优先验证顺序**（按资产齐备度，见 `docs/algorithm_matrix.md` §五）：
1. **重建 RMUL2026 地图**（§1 + §1.1）← 解开该场地所有 nav 组合；
2. `nav+amcl` @ RMUL2026 发目标（§2 + §0.6）；
3. `nav+slam_toolbox` @ RMUL（§4）、`nav+icp` @ RMUL（§3）；
4. `slam_nav`（§1.2）→ cartographer 建图/纯定位（§5/§6）→ `nav:=dwb|teb`（§7）。

---

## 12. 实测记录

> **实测状态的唯一真值来源是 `docs/algorithm_matrix.md` §四**（每跑通一个组合在那里加一行）。
> 本表只保留"实跑时特有的、值得复现的现象"。

| 日期 | 场景 | 命令要点 | 结果 |
|---|---|---|---|
| 2026-09 | 一 mapping+fastlio | `mode:=mapping lio:=fastlio` | ✅ `/odom` 连续、`odom→base_link` 连续；曾修：Gazebo 里程计抢 `/odom`（remap 到 `/odom_ground_truth`）、出生 z=1.16 自由落体致 RViz 车体倾斜（改 z=0.2） |
| 2026-09 | 二 nav+amcl | `mode:=nav localization:=amcl nav:=rpp` | ✅ `Managed nodes are active`；`map_server 272×210` → `amcl Received a 272 X 210 map`。启动期约 4s 刷 `Timed out waiting for transform from base_link_fake to map` 与 `extrapolation` 属**瞬态**（TF 各帧刚建立、10Hz LIO TF 略滞后于 `now()`），给完 `/initialpose` 后自行恢复，不影响激活 |
| 2026-09 | 二 关机 | Ctrl-C | `fastlio_mapping` / `component_container_mt` 退出码 **-11** 属 Humble 关机期已知现象（进程已 Deactivate/Cleanup，非运行期崩溃） |
| 2026-09 | 二 地图系排查 | `world:=RMUL` 下按「世界出生点」给 AMCL 初值 | ❌ **判断错误已修正**：RMUL/RMUC 的 pgm 是**出生点系**（初值必须 `(0,0,0)`），只有 RMUL2026 是**世界系**（`(4.3,3.35)`）。判定方法与证据见 §0.5；已改为 launch 按 `world` 自动注入初值 |
| 2026-09 | 无头 amcl 单测（砂箱，无 Gazebo/RViz） | 只起 map_server+amcl，注入 `initial_pose_x/y=4.3/3.35` | ✅ **自动初值生效**（`/amcl_pose` = 4.30, 3.35）；且证明 **无 `/scan` 时 `map` 帧根本不存在**，补上 `/scan` 后立刻出现 `map→odom`（见 §9.2） |
| 2026-09 | 二 nav+amcl @ **RMUL2026**（用户机实跑） | `world:=RMUL2026 mode:=nav lio:=fastlio localization:=amcl nav:=rpp nav_rviz:=True` | ✅ **全链路通过**：`/controller_server`+`/planner_server` = active；`/map` 240×169 origin(2.68,0.228)；`map→odom` = **(4.294, 3.357, 0.052)**、RPY (-0.30°,-0.03°,-0.07°)；`/amcl_pose` = **(4.300,3.350)**（=注入初值）；`/scan` 1 pub + 3 sub（amcl/local_costmap/rviz）**QoS 全 BEST_EFFORT 匹配** |
| 2026-09 | 二 `/amcl_pose` 的 covariance ≈ 0 | nav2 `set_initial_pose` 路径**不填协方差**（`amcl_node.cpp` L271-281 只设 position/orientation） | 正常现象：初始粒子云是**零散布单点**。位置给对无影响；若初值给错，AMCL 难以自行纠回 → 必须用 RViz `2D Pose Estimate` 重给 |
| 2026-09 | 二 发目标 @ RMUL2026 | `/cmd_vel` 长时间 `linear.x: -0.05`、`/cmd_vel_chassis` 出现 `angular.z: 5.0` | ⚠️ **未通过**：`-0.05` = BT `BackUp` 的 `backup_speed`，`5.0` = `fake_vel_params.yaml` 的 `spin_speed` → 命令链通、小陀螺生效，但 Nav2 卡在**恢复行为循环**（路径/控制失败）。顺带查出 `recoveries_server` 段（Galactic 名字）被 Humble 静默忽略，已改为 `behavior_server` |
| 2026-09 | 二 发目标 @ RMUL2026（第二轮，带日志） | `planner_server: failed to generate a valid path to (6.00, 3.40)`；`odom_ground_truth=(4.2226,3.3442)` vs `amcl_pose=(4.300,3.350)` 差 8cm | ✅ **根因确定**：定位/地图坐标系没问题（真值与 amcl 一致），失败在于**目标点被地图上不存在的"幽灵墙"隔开**——`RMUL2026.pgm` 在 x≈5.2 有一条 1 像素宽虚线（y≈1.8→6.0），而世界网格在该处**零顶点**；经 `robot_radius=0.2m` 膨胀后走廊被封死（r=0.15m 时同一点即可达），地图被切成 3 块、目标落在另一块。已新增 `tools/check_map_reachable.py` 用于选点前验证 |
| 2026-09-16 | 文档/架构（非实跑） | 新增 `docs/algorithm_matrix.md`（算法现状唯一真值来源）与 `docs/issues_and_findings.md`（问题汇总）；决策**不按 2D/3D 物理重组目录**（维度只做文档标注） | ✅ 已登记到 architecture 索引 |
| 2026-09-16 | cartographer 话题修正 | 栅格输出由 `/cartographer_map` 改回标准 `/map`（原 remap 导致 `static_layer` 与 `map_saver_cli` 静默失效） | ✅ 可用 `occupancy_grid_topic` 覆盖；场景五判据已加 `topic info /map` |
| 2026-09-16 | 纯建图 RViz | 形态拆分后 `mode:=mapping` 丢失 RViz | ✅ bringup 单独补 RViz（`nav2.rviz`） |
| 2026-09 | 三 nav+ICP @ RMUL（第一次） | `spawn_entity: Spawn service failed` + `odom` 帧 13 秒不存在 | ⚠️ **未进入 ICP**：机器人插件晚 4 s 加载，LIO 尚未吐 `/odom`，Nav2 无法激活（现场 20 s 就 Ctrl-C 了）。ICP 本身正常：`pcd point size: 97642, 4866`、`pointcloud_topic: /livox/lidar/pointcloud`、`icp_registration initialized`。结论：RMUL 世界重（44 万面），需给 30~60 s |



---


## 来源 2/6：`sim/machine_hygiene_runbook.md`


> 适用机器：`weicheng` 的 32 GB / 28 核工作站（kernel `6.8.0-146-generic`，systemd `249.11-0ubuntu3.23`，
> root 文件系统 `ext4`，`/swapfile` 4 GiB）。
> 本文分两部分：
> **Part 1**（§0–§7）交换区体检 + 安全处置 + **不该做什么**；
> **Part 2**（§8）本仓 bench/ab 脚本的**收尾作用域**改造（只杀自己的进程）与实测证据。
>
> **每一条命令都标了状态**：
> - **【已执行】** —— 本次在沙箱里真跑过，输出见正文；
> - **【未执行·需 root】** —— 会改系统状态且要 root，**没有跑**，命令原样给出；
> - **【未执行·会打扰用户】** —— 会动用户的进程/窗口，**没有跑**；
> - **【沙箱限制】** —— 本会话的 bash 跑在 `bwrap --unshare-pid` 的独立 PID 命名空间里，
>   只能看见 4 个进程（`ls -d /proc/[0-9]* | wc -l` → 4），**看不到宿主机进程**，故这类量测只能给命令。

---

## 0. 结论速查（2026-10-18 04:23 CST 实测）

| 项 | 实测值 | 命令状态 |
|---|---|---|
| `MemTotal` / `MemAvailable` | 31.1 GiB / **16.8 GiB** | 【已执行】 |
| swap 总量 / 已用 / 剩余 | 4095 MiB / **194 MiB** / 3901 MiB | 【已执行】 |
| swap 用量（本次测试**前**，04:09） | **4.3 MiB**（`pswpout=652` 页 ≈ 2.5 MB） | 【已执行】 |
| `vm.swappiness` | **60**（默认） | 【已执行】 |
| zram | **不可用**：`/dev/zram*` 不存在、`zram` 模块**未加载**、无 `zram-generator` | 【已执行】 |
| root/sudo | 账号在 `sudo` 组，但**沙箱内无法 sudo**（`no new privileges`） | 【已执行】 |
| `systemd-oomd` | **active**；user 会话被发行版设为 `ManagedOOMMemoryPressure=kill` + **`Limit=50%`** | 【已执行】 |
| 历史 OOM | 10-06 11:30:37 `async_slam_tool`（anon-rss **11.85 GB**，全局 OOM）；10-01 21:44:25 `vte-spawn-*.scope`（**25 个进程**被 oomd 整组杀） | 【已执行】 |

**三句话**：
1. **当前 swap 没有耗尽**（194 MiB / 4095 MiB，`MemAvailable` 16.8 GiB）。这次实测里
   **`swap` 从 4.3 MiB 涨到 194 MiB**，是在我跑了 6 次真实 gazebo 短测（Part 2 的验证）期间发生的
   —— 即**桌面应用（firefox / chrome / gnome-software / packagekit）被换出**，不是 gazebo 常驻占着。
2. 用户此前看到的 **“`SwapFree ≈ 1 MiB`” 这个快照本次复现不出来**。硬证据：本次开机
   （2026-10-16 16:06 起，已 1 天 12 小时）在我跑测试**之前** `pswpout` 只有 **652 页（2.5 MB）**
   —— 如果这台机器在这次开机里真的被换满 4 GB，`pswpout` 会是 **~100 万页**量级。
   ⇒ 那个快照应属于 **10-16 16:06 那次重启之前**的开机（换页统计随重启清零）。
3. **真正该防的不是“swap 满”，是“oomd 整组杀”**：10-01 那次是 oomd 因为 user slice 的
   **内存压力 72.74% > 50% 持续 20 s** 把整个 `vte-spawn-*.scope`（25 个进程）一起杀的；
   10-06 那次是**真·全局 OOM**（`async_slam_tool` anon-rss 11.85 GB），oomd 只是恰好是触发分配的任务。
   两者的处置完全不同（见 §5）。

---

## 1. 怎么量：先看全机，再找"谁在 swap 里"

### 1.1 全机口径【已执行】

```bash
free -m
swapon --show
cat /proc/sys/vm/swappiness
grep -E 'Swap(Total|Free|Cached)|Zswap|Zswapped' /proc/meminfo
grep -E 'pswpin|pswpout' /proc/vmstat        # 本次开机累计换入/换出页数（1 页=4 KiB）
cat /proc/pressure/memory                    # PSI：oomd 判据的原始输入
```

本次输出（04:09 与 04:23 两次，中间跑过 6 次 gazebo 短测）：

```
# 04:09（测试前）
内存： 31866 total / 14309 used / 3282 free / 14274 buff-cache / 16756 available
交换：  4095 total /     4 used / 4091 free
NAME      TYPE SIZE USED PRIO
/swapfile file   4G 4.3M   -2
swappiness = 60
SwapTotal: 4194300 kB / SwapFree: 4189948 kB / SwapCached: 2280 kB / Zswap: 0 kB
pswpin 80 / pswpout 652

# 04:23（6 次短测之后）
交换：  4095 total /   194 used / 3901 free
pswpin 80 / pswpout 49041          # 多换出 48389 页 ≈ 189 MB
```

> **怎么读 `pswpout`**：它是**本次开机**累计值。判断“swap 有没有被换满过”，这条比
> `free` 的瞬时值可靠得多。

### 1.2 每进程 `VmSwap` top 15【未执行·沙箱限制（无宿主机 PID 视图）】

在**你自己的终端**（不是本 agent 的沙箱）里跑：

```bash
# ① 每进程 VmSwap（kB → MB），top 15；带 PID 与进程名
for p in /proc/[0-9]*/status; do
  pid=${p#/proc/}; pid=${pid%/status}
  awk -v pid="$pid" '
    /^Name:/{n=$2}
    /^VmSwap:/{ if ($2+0 > 0) printf "%9.2f MB  pid=%-8s %-24s\n", $2/1024, pid, n }
  ' "$p" 2>/dev/null
done | sort -rn | head -15

# ② 顺带看 RSS（谁在吃内存）与 swap 的对比
for p in /proc/[0-9]*/status; do
  pid=${p#/proc/}; pid=${pid%/status}
  awk -v pid="$pid" '
    /^Name:/{n=$2}
    /^VmRSS:/{r=$2}
    /^VmSwap:/{ if ($2+0 > 0) printf "%9.2f MB swap / %9.2f MB rss  pid=%-8s %-24s\n", $2/1024, r/1024, pid, n }
  ' "$p" 2>/dev/null
done | sort -rn | head -15
```

本次在沙箱里跑 ①（同一段 python 逻辑）得到的结果是 **0 行**：

```
# 【已执行】沙箱内 /proc 只有 4 个 PID（bwrap 自己 + 本次 bash + 它的子进程）
$ ls -d /proc/[0-9]* | wc -l
4
$ <扫描 /proc/*/status 的 VmSwap>
processes with VmSwap>0: 0    sum VmSwap = 0.0 MB
```

⇒ “top 15 VmSwap 表”在当前沙箱里**在原理上就取不到**（不是 0，而是**看不见**）。
**替代口径**见 §1.3（cgroup 级，能真正回答“谁在 swap 里”）。

### 1.3 按 cgroup 的 swap（可用的替代口径）【已执行】

cgroup v2 的 `memory.swap.current` 是**宿主机**视角的，且能一路下钻到 scope/unit：

```bash
for f in $(find /sys/fs/cgroup -maxdepth 7 -name memory.swap.current -readable 2>/dev/null); do
  s=$(cat "$f" 2>/dev/null || echo 0)
  [ "${s:-0}" -gt 0 ] || continue
  printf '%9.2f MB swap %8.0f MB mem  %s\n' \
    "$(echo "scale=2;$s/1048576" | bc)" \
    "$(echo "$(cat $(dirname "$f")/memory.current 2>/dev/null || echo 0)/1048576" | bc)" \
    "$(dirname "$f" | sed 's|/sys/fs/cgroup||')"
done | sort -rn | head -15
```

本次输出（04:23，合计 ≈ 194 MB，与 `free` 的 194 MiB 吻合）：

| swap | 所在 cgroup（= 谁的） | 进程侧内存 |
|---:|---|---:|
| **102.35 MB** | `.../app.slice/snap.firefox.firefox-*.scope` | 816 MB |
| **34.98 MB** | `.../app.slice/app-com.google.Chrome-6144.scope` | 231 MB |
| **21.27 MB** | `.../app.slice/app-gnome-gnome-software-*.scope` | 111 MB |
| **9.65 MB** | `/system.slice/packagekit.service` | 480 MB |
| 4.26 MB | `.../app.slice/app-gnome-google-chrome-6144.scope` | 2750 MB |
| 1.99 MB | `.../app.slice/app-code-7884.scope`（VS Code） | 6682 MB |
| 1.49 MB | `.../session.slice/org.gnome.SettingsDaemon.Datetime.service` | 1 MB |
| 1.04 MB | `.../session.slice/org.gnome.SettingsDaemon.Keyboard.service` | 6 MB |
| 1.02 MB | `.../session.slice/pipewire.service` | 0 MB |

顶层：`user.slice` **181.5 MB**、`system.slice` **9.9 MB**、`init.scope` 0 MB。

> **结论**：这次 swap 里装的是**桌面应用**（firefox / Chrome / gnome-software / packagekit），
> 不是 gazebo。要想知道“具体是哪个 PID”，必须在**非沙箱**终端用 §1.2 的脚本 —— 或者直接
> `systemctl status <scope>` 看这个 scope 里有哪些进程。

### 1.4 zram 可用性【已执行】

```bash
ls -l /dev/zram*            # 本次：No such file or directory
command -v zramctl          # 本次：/usr/sbin/zramctl（工具在）
modinfo zram                # 本次：模块在（/lib/modules/6.8.0-146-generic/.../zram.ko）
lsmod | grep '^zram'        # 本次：空（未加载）
ls /usr/lib/systemd/systemd-zram-setup /usr/lib/systemd/system/zram-generator* 2>/dev/null
                            # 本次：都没有 ⇒ 没装 systemd-zram-generator
```

⇒ **zram 现在不可用**（模块在、工具在，但没加载、没有设备、也没有开机自动配置它的 generator）。
启用见 §3.2（需 root）。

### 1.5 root / sudo【已执行】

```bash
id -u                       # 1000
id                          # uid=1000(weicheng) gid=1000(weicheng) 组=1000(weicheng),65534(nogroup)
sudo -n true; echo $?       # 1
getent group sudo           # sudo:x:27:weicheng   ← 账号本身在 sudo 组
```

`sudo` 的真实报错：

```
sudo: /etc/sudo.conf is owned by uid 65534, should be 0
sudo: The "no new privileges" flag is set, which prevents sudo from running as root.
```

> **【沙箱限制】**：这是 bwrap 沙箱的 `no-new-privs` + ID 映射（`nobody/nogroup`）造成的，
> **不代表真机没有 sudo**。真机上请在**自己的终端**确认：
> `sudo -n true && echo HAVE_ROOT || echo NO_PASSWORDLESS_SUDO`。
> 本文所有 “需 root” 的命令**我一条都没有执行**。

### 1.6 systemd-oomd 现状 + 两次历史 OOM 的原始行【已执行】

```bash
systemctl is-active systemd-oomd
systemctl show user@1000.service -p ManagedOOMSwap -p ManagedOOMMemoryPressure -p ManagedOOMMemoryPressureLimit
systemctl cat user@1000.service | grep -n -A2 oomd
systemd-analyze cat-config systemd/oomd.conf
journalctl --no-pager -k | grep -iE 'Out of memory: Killed|oom-kill' | tail
journalctl --no-pager | grep -iE 'oomd.*(kill|Killed)' | tail
```

本次输出：

```
active
ManagedOOMSwap=auto
ManagedOOMMemoryPressure=kill
ManagedOOMMemoryPressureLimit=2147483648        # systemd 的百分比编码：2147483648 = 50%
ManagedOOMPreference=none

# 发行版 drop-in（这是"50%"的来源，不是 /etc/systemd/oomd.conf 的 60%）：
/usr/lib/systemd/system/user@.service.d/10-oomd-user-service-defaults.conf
  [Service]
  ManagedOOMMemoryPressure=kill
  ManagedOOMMemoryPressureLimit=50%

# /etc/systemd/oomd.conf 里只有注释（编译期默认值）：
#SwapUsedLimit=90%
#DefaultMemoryPressureLimit=60%
#DefaultMemoryPressureDurationSec=30s
# 另有 /usr/lib/systemd/oomd.conf.d/10-oomd-defaults.conf:
[OOM]
DefaultMemoryPressureDurationSec=20s
```

历史 OOM 原始行（`journalctl`）：

```
10月 01 21:44:25 systemd-oomd[802]: Killed /user.slice/.../vte-spawn-aabb8d4b-....scope
  due to memory pressure for /user.slice/user-1000.slice/user@1000.service
  being 72.74% > 50.00% for > 20s with reclaim activity
10月 01 21:44:25 systemd[2156]: vte-spawn-aabb8d4b-....scope: systemd-oomd killed 25 process(es) in this unit.

10月 06 11:30:36 kernel: systemd-oomd invoked oom-killer: ... oom_score_adj=-900
10月 06 11:30:37 kernel: oom-kill:constraint=CONSTRAINT_NONE,...,global_oom,
  task_memcg=/user.slice/.../vte-spawn-964a1bbf-....scope,task=async_slam_tool,pid=264550,uid=1000
10月 06 11:30:37 kernel: Out of memory: Killed process 264550 (async_slam_tool)
  total-vm:18841480kB, anon-rss:11854480kB, file-rss:1568kB, shmem-rss:0kB, UID:1000
10月 06 11:30:36 systemd[1]: user@1000.service: A process of this unit has been killed by the OOM killer.
```

**两次的性质不同**（要点）：

| 日期 | 谁杀的 | 判据 | 被杀单位 |
|---|---|---|---|
| 10-01 21:44 | **systemd-oomd（用户态）** | user slice PSI > **50%** 持续 **20 s** 且**有 reclaim 活动** | 整个 `vte-spawn-*.scope`（**25 个进程**，含终端里所有 job） |
| 10-06 11:30 | **内核 OOM killer**（`global_oom`；调用的任务是 oomd） | 全机真的分配不出页（`async_slam_tool` 单进程 anon-rss **11.85 GB**） | 单进程 `async_slam_tool` |

⇒ 10-01 那种“终端整组没了”，是 **oomd 的粒度**（scope 级）导致的，不是内存真不够；
10-06 那种是**真不够**，换 swap/zram 只能拖延，救不了。

---

## 2. 安全地"释放 swap"（不动别人的进程、不 `swapoff`）

### 2.1 先判断“要不要动”

```bash
free -m | tail -1
pswpout=$(awk '/pswpout/{print $2}' /proc/vmstat); echo "本次开机累计换出 $((pswpout*4/1024)) MB"
```

- `SwapFree` 还有一半以上 ⇒ **不用动**（换出几百 MB 是内核正常行为，反而省内存）。
- `SwapFree` < 200 MB 且 `MemAvailable` 也不多 ⇒ 按 §2.2 处理，**不要** §2.3 的 `swapoff`。

### 2.2 正解：重启**那一个**长命进程

swap 里的页只有两种消失方式：**进程退出**（页被丢弃）或 **swapoff**（页被换回内存）。
所以“释放 swap”= 让持有它的长命进程**正常重启**：

```bash
# ① 找到是谁（cgroup 级，可在沙箱/普通终端跑）
systemctl status <scope 名，例如 app-com.google.Chrome-6144.scope> --no-pager | head -20

# ② 找到同 scope 里具体哪个 PID 的 VmSwap 最大（真机终端，见 §1.2）

# ③ 只重启/关闭那一个东西（用户自己的会话操作，本 agent 不会替你做）
#    - 桌面应用：正常关窗口（不要 kill -9，浏览器会丢会话）
#    - 我们自己的栈（gzserver/slam_toolbox/...）：重跑那个 bench 脚本即可，
#      脚本收尾现在只会清"自己这次起的"进程（见 Part 2 §8）
```

> **重启整个 bench 栈的推荐做法**：用 Part 2 改造后的脚本重跑（或直接 `Ctrl-C` 让它走 trap）。
> **不要**用 `pkill -9 -x gzserver` —— 见 §6。

### 2.3 只有在这种条件下才考虑 `swapoff`【未执行·会打扰用户 / 有 OOM 风险】

`swapoff` 要把 swap 里的**全部**页换回 RAM；`SwapUsed` 必须**明显小于** `MemAvailable`，
并留足余量（经验：`SwapUsed_MB * 2 < MemAvailable_MB`），且**没有任何 gazebo/建图在跑**
（否则一换回就把刚空出来的内存又吃掉，直接 OOM）：

```bash
# 手动、在真机终端、确认上面的条件后（本 agent 没有执行，也不建议脚本化）
free -m
sudo swapoff /swapfile && sudo swapon /swapfile
free -m
```

**绝对不要**在 `SwapFree` 只剩几 MB 时跑 `swapoff -a`：4 GB 换回需要 4 GB 空闲内存，
不够就是一轮 OOM（10-06 那种 `global_oom` 会重演）。

---

## 3. 扩容/加 swap、开 zram（**需 root，一条都没有执行**）

### 3.1 加一个 swapfile【未执行·需 root】

```bash
# 0) 看空间与文件系统（本次实测：ext4，/ 上还有 138 GB 空闲）
df -T / ; df -h /
# 1) 建 8 GiB（ext4 用 fallocate 即可；btrfs 要先 chattr +C 且不要用 fallocate）
sudo fallocate -l 8G /swapfile8
sudo chmod 600 /swapfile8
sudo mkswap /swapfile8
# 2) 启用（优先级给低一点，让内核优先用原来的 /swapfile 也行；这里只加容量）
sudo swapon /swapfile8
sudo swapon --show
free -m
# 3) 持久化
echo '/swapfile8 none swap sw,nofail 0 0' | sudo tee -a /etc/fstab
# 4) 回滚
sudo swapoff /swapfile8 && sudo rm -f /swapfile8   # 再从 /etc/fstab 删掉那一行
```

**注意**：

- 不要放在 `/tmp`（本机 `/tmp` 在部分运行环境里是 tmpfs，等于放内存里）、不要放 NFS。
- NVMe 上写 swap 有寿命成本；4 GiB → 8 GiB 只解决“换出容量”，不解决“anon 11.85 GB”那种峰值。
- 加完之后 `vm.swappiness` 仍是 60，内核会更积极换出（见 §4）。

### 3.2 开 zram（压缩内存块设备）【未执行·需 root】

```bash
# 一次性（重启失效）
sudo modprobe zram
sudo zramctl --find --size 8G --algorithm zstd      # 设备名通常是 /dev/zram0
sudo mkswap /dev/zram0
sudo swapon --priority 100 /dev/zram0
zramctl ; swapon --show ; free -m
# 回滚
sudo swapoff /dev/zram0 && sudo zramctl --reset /dev/zram0 && sudo modprobe -r zram
```

```bash
# 持久化（推荐；本机**没装** systemd-zram-generator）
sudo apt-get install -y systemd-zram-generator
sudo tee /etc/systemd/zram-generator.conf >/dev/null <<'EOF'
[zram0]
zram-size = min(ram / 2, 8192)
compression-algorithm = zstd
swap-priority = 100
EOF
sudo systemctl daemon-reload
sudo systemctl start systemd-zram-setup@zram0.service
zramctl ; swapon --show
```

**注意**：

- zram **不增加**总内存，只是把换出的页压缩后放回 RAM（典型 2–3×）。它适合“换出量小但频繁”，
  **不适合** 10-06 那种单进程 11.85 GB 匿名内存的峰值。
- zram 里放 swap 时，社区通常把 `vm.swappiness` 调到 100–180（更愿意压缩换出）；**本机没开 zram，
  所以这条先别照抄**。
- 已有一个 4 GiB 的 `/swapfile`；两者可以并存（zram 优先级更高即可）。

---

## 4. `vm.swappiness` 怎么给

现状：**60**（内核默认）。含义：内存紧张时，内核在“丢 page cache”与“换出匿名页”之间的相对权重。

| 场景 | 建议 | 理由 |
|---|---|---|
| 桌面/交互（本机） | **10–20** | 少换出匿名页 ⇒ 切窗口/编译不卡；代价是可用内存变少、page cache 更早被丢 |
| 服务器/批处理 | 60（默认） | 更愿意换出冷页，保住 cache |
| 用了 zram 做 swap | 100–180 | 换出=压缩，仍在本机内存里，代价低 |
| `0` | **不建议** | 现代内核仍会在必要时换出；`0` 容易造成直接 reclaim 卡顿，且不会减少 OOM |

```bash
# 临时（重启失效）【未执行·需 root】
sudo sysctl -w vm.swappiness=10
# 永久【未执行·需 root】
echo 'vm.swappiness=10' | sudo tee /etc/sysctl.d/99-swappiness.conf
sudo sysctl --system
cat /proc/sys/vm/swappiness
```

> 提醒：**调 swappiness 不解决 swap 满**。swap 满 = 匿名内存 > RAM + swap；
> 唯一正解是减少常驻（关掉不用的栈/浏览器标签）或加容量（§3）。

---

## 5. `systemd-oomd` 调参（先看它现在有多激进）

### 5.1 它现在为什么这么激进【已执行】

- user 会话被发行版 drop-in 设成 **`ManagedOOMMemoryPressure=kill` + `ManagedOOMMemoryPressureLimit=50%`**
  （`/usr/lib/systemd/system/user@.service.d/10-oomd-user-service-defaults.conf`）；
- 触发条件：**user slice 的 PSI > 50% 持续 20 s 且有 reclaim 活动** ⇒ **杀整个 scope**；
- 10-01 21:44 就是这么把 `vte-spawn-*.scope` 里 **25 个进程**一起杀的（用户当时正在终端里干活）。
- 另一条触发线是 `ManagedOOMSwap=auto` + `oomd.conf` 的 `SwapUsedLimit=90%`：
  **swap 用到 90% 时，oomd 会在 user slice 里挑一个 cgroup 整组杀**。本机 swap 只有 4 GiB，
  一旦被桌面应用 + gazebo 填到 90%（≈3.6 GB），这条线就会启动。

### 5.2 三个可调项（按“改动范围从小到大”）

**(a) 只把 user 会话的阈值放宽（最小改动）**【未执行·需 root】

```bash
sudo mkdir -p /etc/systemd/system/user@1000.service.d
sudo tee /etc/systemd/system/user@1000.service.d/50-oomd.conf >/dev/null <<'EOF'
[Service]
# 保留 oomd 兜底，但不再 50% 就整组杀；80% 才动手
ManagedOOMMemoryPressure=kill
ManagedOOMMemoryPressureLimit=80%
EOF
sudo systemctl daemon-reload
systemctl show user@1000.service -p ManagedOOMMemoryPressure -p ManagedOOMMemoryPressureLimit
# 期望：kill / 3435973836（=80%）
```

**(b) 让 user 会话不再被 oomd 主动杀（更彻底，但失去这层保护）**【未执行·需 root】

```bash
sudo tee /etc/systemd/system/user@1000.service.d/50-oomd.conf >/dev/null <<'EOF'
[Service]
ManagedOOMMemoryPressure=auto
ManagedOOMSwap=auto
EOF
sudo systemctl daemon-reload
systemctl show user@1000.service -p ManagedOOMMemoryPressure   # 期望：auto
```

> `auto` 的语义是“由上层/默认策略决定”，对 `user@.service` 来说等于**不再由这条 drop-in 主动 kill**。
> 是否真的完全关闭，以 `systemctl show` 的结果为准（别只看文件）。

**(c) 改 oomd 全局默认（影响所有 unit，包括系统服务）**【未执行·需 root】

```bash
sudo tee /etc/systemd/oomd.conf >/dev/null <<'EOF'
[OOM]
SwapUsedLimit=95%
DefaultMemoryPressureLimit=80%
DefaultMemoryPressureDurationSec=30s
EOF
sudo systemctl restart systemd-oomd
systemd-analyze cat-config systemd/oomd.conf | tail -20
systemctl status systemd-oomd --no-pager | head -8
```

### 5.3 调参的代价（务必知道）

- oomd 的粒度是 **cgroup**：它杀的是**整个 scope/unit**，不是单个进程。放宽它 ⇒ **终端/整个栈一起死**
  的概率下降；但内存真的不够时，内核 OOM killer 会接手，它的粒度是**单进程**、选择更“随机”
  （10-06 挑中的就是 `async_slam_tool`，而它恰好是干活的那个）。
- `ManagedOOMMemoryPressure=kill` 是**兜底**。如果直接删掉，swap 满时可能先经历长时间卡顿
  （直接 reclaim / thrashing）再被内核杀 —— **“卡死”比“被杀”更难恢复**。
- 想保护某个具体 scope（比如正在跑的 bench 栈）：

  ```bash
  systemctl set-property --runtime <scope>.scope ManagedOOMPreference=avoid
  ```

  这是**提示**不是保证（10-01 被杀的那个 scope 没有任何 preference）。

---

## 6. **不要做**的事（血泪清单）

| ❌ 不要 | 为什么 |
|---|---|
| **`swapoff -a` / `swapoff /swapfile`（swap 快满时）** | 要把 4 GB 页换回 RAM；没有 4 GB 空闲 ⇒ 直接触发全局 OOM（10-06 那种）。**本 agent 一条都没跑** |
| **全机 `pkill -9 -x gzserver` / `gzclient`** | 不看 `GAZEBO_MASTER_URI`，会杀掉同机**别人**正在跑/正在看的栈（也正是本仓旧脚本的问题，Part 2 已改） |
| `pkill -9 -f 'ros2 launch'` / `pkill -f ros` | 连别人的 launch / 所有 ROS 进程一起杀；本仓旧脚本里有，Part 2 已清掉 9 个脚本里的这类写法 |
| `pkill -f "gzserver.*$PORT"` 当成“只杀自己” | **空操作**：`gzserver` 的 cmdline 里只有 world 文件，端口在 `GAZEBO_MASTER_URI` 环境变量里 ⇒ 永远匹配不到（`gzclient` 同理） |
| 因为 Gazebo 卡就手动 `pkill -9` | 用户 10-06/10-07 的 `gzclient exit code -9` 就是这么来的（外部 SIGKILL），而“卡”的根因是 §5.4/§4.2 里那两类（模型库阻塞 / RTF 低）。要停就 `Ctrl-C` 让 launch 走正常收尾 |
| `echo 3 > /proc/sys/vm/drop_caches` “清内存” | 只丢 page cache，**不释放 swap**；之后所有东西重新读盘，反而更慢，还可能引发换入风暴 |
| `rm /swapfile` 而不先 `swapoff` | 内核仍占着被删的 inode，swap 不可回收，直到重启 |
| 把 swapfile 建在 `/tmp`、tmpfs、NFS 上 | 不生效/极慢/占内存 |
| `systemd-run --scope` 里跑 gazebo 后又 `systemctl kill` 整个 scope | 会连带杀掉该 scope 里别人的进程；杀之前先 `systemctl status` 看清 |

---

## 7. 本次“执行 / 未执行”总表

| 命令/动作 | 状态 | 备注 |
|---|---|---|
| `free -m`、`swapon --show`、`/proc/meminfo`、`/proc/vmstat`、`/proc/pressure/memory` | **已执行** | §1.1 |
| 扫 `/proc/*/status` 的 `VmSwap` | **已执行（沙箱内 0 行）** | 只有 4 个 PID 可见 ⇒ §1.2 的限制 |
| 扫 `cgroup memory.swap.current` | **已执行** | §1.3，给出了真正“谁在 swap 里” |
| `id -u`、`id`、`sudo -n true`、`getent group sudo` | **已执行** | §1.5 |
| `ls /dev/zram*`、`command -v zramctl`、`modinfo zram`、`lsmod` | **已执行** | §1.4 |
| `systemctl show user@1000.service ...`、`systemctl cat`、`systemd-analyze cat-config` | **已执行** | §5.1 |
| `journalctl` 查历史 OOM | **已执行** | §1.6 |
| `modprobe zram` / `zramctl --find` / `mkswap` / `swapon` | **未执行·需 root** | §3.2 |
| `fallocate` 新 swapfile / `mkswap` / `swapon` / 改 `/etc/fstab` | **未执行·需 root** | §3.1 |
| `sysctl -w vm.swappiness=...` / 写 `/etc/sysctl.d` | **未执行·需 root** | §4 |
| 写 oomd drop-in / 改 `/etc/systemd/oomd.conf` / `systemctl restart systemd-oomd` | **未执行·需 root** | §5.2 |
| `swapoff`（任何形式） | **未执行**（有 OOM 风险） | §2.3 |
| 重启/杀**用户的**任何进程 | **未执行**（只清理过本 agent 自己起的测试栈，见 Part 2） | §2.2 / §8 |

---

## 8. Part 2：本仓脚本收尾的**作用域**改造（只杀自己起的进程）

> 背景：旧脚本收尾是**全机** `pkill -9 -x gzserver` / `gzclient` + `pkill -9 -f 'ros2 launch'`，
> 不看 `GAZEBO_MASTER_URI`。同机有兄弟任务在跑时，这会杀掉**别人**的栈
> （`docs/gazebo_gui_troubleshooting.md` §4.2 / §5.4 记录过这条候选路径）。
> 另有一类"看着像保护、其实永远匹配不到"的写法：`pkill -f "gzserver.*$GZ_PORT"` /
> `pkill -f "gzclient.*$GZ_PORT"` —— **gzserver/gzclient 的 cmdline 里没有端口**
> （端口在 `GAZEBO_MASTER_URI` 环境变量里）⇒ 这两条是**空操作**，等于没保护。

### 8.1 判定规则（三条，**任一**成立才算"自己的"）

实现：**`tools/scripts/lib/scoped_cleanup.sh`**（新增，14 个脚本 `source` 它）。

| # | 判据 | 怎么做的 |
|---|---|---|
| ① | **进程组**（首选，无需名字匹配） | 起栈用 `setsid ros2 launch ... &` ⇒ `PGID=$!`；收尾 `kill -INT -- "-$PGID"` / `kill -9 -- "-$PGID"` |
| ② | **环境变量** | 读 `/proc/<pid>/environ`，`GAZEBO_MASTER_URI` 必须与本 run **逐字相等** |
| ③ | **血缘** | PID 是 `$$`（本脚本）/ `$LPID` / `$LAUNCH_PID` 的**后代** |

- **名字/命令行只用来挑候选**（`comm` 是 `gzserver`/`gzclient`，或 cmdline 匹配某 ERE），
  **绝不单独作为杀的理由**。
- 自伤保护：`$$` 本身、`$BASHPID`、以及 `$$` 的**所有祖先**永不杀
  （防 `bash -c "... pkill ..."` 那种把自己/调用者杀掉的老坑）。
- 对外 API（脚本里就这 5 个，其余是内部实现）：

  | 函数 | 作用 |
  |---|---|
  | `sc_own_gz_pids` | 列出本 run 的 `gzserver`/`gzclient` PID（comm 精确匹配 + 归属判定） |
  | `sc_kill_own_gz <SIG>` | 只杀上面这些 |
  | `sc_kill_own_pattern <SIG> <ERE>` | 只杀"本 run 里 cmdline 匹配"的进程（节点名走这条） |
  | `sc_kill_own_ros2_launch <SIG> [ERE]` | 只杀本 run 的 `ros2 launch`（默认 ERE=`ros2 launch`） |
  | `sc_own_leftovers_desc [ERE]` / `sc_own_gz_count` | 收尾报告用（**作用域内**的残留，不再把别人的进程报成"残留"） |

- 排障开关：`SC_SCOPE_VERBOSE=1` 会把每个"own/skip + 原因"打到 stderr。

### 8.2 逐文件改动

> 说明：`setsid`/`PGID` 这套**本来就有**的脚本只替换了"按名字全机杀"的那几行；
> `run_robot_model_probe.sh`、`robot11_gui_bench.sh` 原来没有 `setsid`，这次一并补上。

| 文件 | 行（改前） | 改成了什么 |
|---|---|---|
| `tools/scripts/diag/run_drift_diag.sh` | 129–134 | `pkill -9 -f 'ros2 launch'` → `sc_kill_own_ros2_launch 9`；`pkill -9 -x gzserver/gzclient` → `sc_kill_own_gz 9`；两条节点 `pkill -9 -f` → `sc_kill_own_pattern 9`；残留复查改为作用域内计数 |
| `tools/scripts/diag/run_nav_lio_timeline.sh` | 169–172、185–188 | 同上；跑前"残留守卫"由 `pgrep -a -x gzserver` + 全机 `pkill` → `sc_own_gz_pids` + `sc_kill_own_gz 9`（**只清本 URI 的**） |
| `tools/scripts/localization/run_gicp_nav_ab.sh` | 92–96、115–118 | 同上（跑前守卫 + 收尾） |
| `tools/scripts/localization/run_nav_smoke_regress.sh` | 44、60–62 | 跑前"残留 gzserver"改为 `sc_own_gz_pids`/`sc_kill_own_gz`；收尾三条 `pkill`（含 `ros2 launch`）→ `sc_kill_own_gz` + `sc_kill_own_pattern` + `sc_kill_own_ros2_launch` |
| `tools/scripts/mapping/mapping_ab_run.sh` | 106、109–112 | `pkill -f 'ramp_section_probe|map_odom_jump_gate'`（TERM）→ `sc_kill_own_pattern 15`；其余三条 → `sc_kill_own_ros2_launch` / `sc_kill_own_gz` / `sc_kill_own_pattern` |
| `tools/scripts/mapping/nav_map_ab_run.sh` | 65–68 | 三条 `pkill` → `sc_kill_own_ros2_launch` / `sc_kill_own_gz` / 两条 `sc_kill_own_pattern` |
| `tools/scripts/mapping/run_mapping_headless.sh` | 132–137 | 同上 + 残留复查改为作用域内 |
| `tools/scripts/mapping/sltune/run_sltune_ab.sh` | 104–107 | `pkill -9 -x gz*` → `sc_kill_own_gz`；节点（原来把 `ros2 launch` 混在 alternation 里）→ `sc_kill_own_pattern` + 单独 `sc_kill_own_ros2_launch`；残留复查改为作用域内 |
| `tools/scripts/mapping/sltune/run_dropab.sh` | 148–151 | 同上（含 `truth_odom` 残留复查） |
| `tools/scripts/regress/run_remap_lowterrain.sh` | 64、78–82 | 跑前守卫 + 收尾整段（含 `pkill -9 -f 'ros2 launch rm_nav_bringup'`）→ 作用域化 |
| `tools/scripts/regress/run_nav_clearance_ab.sh` | 134–137、150–155 | 跑前守卫 + 收尾整段（`ros2 launch` 从 alternation 里拆出来单独 scope） |
| `tools/scripts/regress/run_low_terrain_capture.sh` | 61–64、77、81–85 | 跑前守卫；`pkill -INT -f "bag record"` → `sc_kill_own_pattern 2 'bag record'`；收尾三条 → 作用域化 |
| `tools/scripts/regress/run_robot_model_probe.sh` | 79–80、83、88–89 | **删掉空操作** `pkill -f "gzserver.*$GZ_PORT"` / `gzclient.*`，改 `sc_kill_own_gz 9`；跑前清理同上；残留计数由"永远 0"的 `pgrep -fa "gzserver.*$GZ_PORT"` 改成 `sc_own_gz_count`；**补 `setsid` + `PGID`**，收尾再加 `sc_kill_own_ros2_launch` / `sc_kill_own_pattern`（组杀之后兜底） |
| `tools/scripts/regress/robot11_gui_bench.sh` | 102–105、110、129–143 | **删掉空操作**的 `pkill -f "gzserver.*$GZ_PORT"`/`gzclient.*`，改 `sc_kill_own_gz 9`；`pkill -f rviz2` → `sc_kill_own_pattern 9 'rviz2'`；起窗口/采样时的 `pgrep -x gzclient|rviz2`（全机口径，会把别人的窗口当成"起来了"）→ `sc_own_pids_by_pattern` |

每个改动点都留了一行注释说明作用域规则（`grep -rn '作用域' tools/scripts/*/` 可看全）。

**行为其它方面不变**：参数、输出路径、退出码、trap 时机、`sleep`/等待次数都没有改；
唯一"多出来"的行为是：**跑前也会清掉同 URI 的上一次残留**（原来全机 `pkill` 意外做到的这件事，
现在按 URI 做，仍然做得到）。

### 8.3 证据 A：合成判定矩阵（10 个 decoy，最快、最直接）

脚本：`.tmp_scopetest/harness.sh`（未入库，**只有 decoy + 一个 source 了 lib 的小 SUT**）。
它模拟"同机别人"的 5 类进程，然后让 SUT 用**自己的 URI** 跑一遍收尾：

```bash
$ bash .tmp_scopetest/harness.sh
=== 收尾前（这是 SUT 之外、模拟同机别人的进程）===
  A_other_uri_gz    pid=13  comm=gzserver  uri=http://127.0.0.1:11990
  B_same_uri_gz     pid=14  comm=gzserver  uri=http://127.0.0.1:11999
  C_plain_sleep     pid=15  comm=sleep      uri=http://127.0.0.1:11990
  D_other_launch    pid=16  comm=sleep      uri=http://127.0.0.1:11990   # cmdline="ros2 launch ..."
  E_same_launch     pid=17  comm=sleep      uri=http://127.0.0.1:11999   # cmdline="ros2 launch ..."
  F_other_slam      pid=18  comm=sleep      uri=http://127.0.0.1:11990   # cmdline="slam_toolbox ..."
  G_same_slam       pid=19  comm=sleep      uri=http://127.0.0.1:11999   # cmdline="slam_toolbox ..."
  H_other_gzclient  pid=20  comm=gzclient   uri=http://127.0.0.1:11990
  [J] 自伤测试：自己还活着（pid=67）

=== 收尾后 ===
  ✅ A_other_uri_gz       pid=13   alive（期望 alive）    # 别人的 gzserver：不同 URI ⇒ 活
  ✅ H_other_gzclient     pid=20   alive（期望 alive）    # 别人的 gzclient
  ✅ B_same_uri_gz        pid=14   dead （期望 dead）     # 本 URI 的 gzserver（非后代）⇒ 按 environ 认领
  ✅ C_plain_sleep        pid=15   alive（期望 alive）    # 无关进程
  ✅ D_other_launch       pid=16   alive（期望 alive）    # 别人的 ros2 launch
  ✅ E_same_launch        pid=17   dead （期望 dead）     # 本 URI 的 ros2 launch
  ✅ F_other_slam         pid=18   alive（期望 alive）    # 别人的 slam_toolbox
  ✅ G_same_slam          pid=19   dead （期望 dead）     # 本 URI 的 slam_toolbox
  ✅ I_own_group_gz       pid=21   dead （期望 dead）     # 本 run 进程组里、URI 是别人的 ⇒ 按**进程组**认领
  ✅ J_self_shell         pid=66   alive（期望 alive）    # cmdline 含 slam_toolbox 的调用者自己 ⇒ 自伤保护

RESULT: PASS
```

> 关键点：**B/E/G 被清掉**证明"`/proc/environ` 核对**真的能匹配**"（旧脚本的
> `pkill -f "gzserver.*$PORT"` 在这里会是**空的**）；**A/D/F/H/C 全活**证明不误伤；
> **I 被清掉**证明进程组这条兜底有效；**J 活着**证明自伤保护有效。

### 8.4 证据 B：3 个真实脚本（真 gazebo + 真 ros2 launch + decoy）

做法（`verify_real.sh`）：先起 5 个 decoy（别人的 gzserver/gzclient/"ros2 launch"/"slam_toolbox" +
无关 sleep），再**真跑**目标脚本；等它自己的 gzserver 起来（按 URI 认），给它发 **SIGTERM**
（等价"跑一半被打断"⇒ 走 trap 收尾），然后查"自己的栈是否清干净 / decoy 是否全活"。

```bash
# ① 最便宜：nav_smoke_regress（单次 launch + 回归）
$ bash .tmp_scopetest/verify_real.sh navsmoke3 http://127.0.0.1:11898 150 \
    tools/scripts/localization/run_nav_smoke_regress.sh --tag scv_nav3 --domain 198 --port 11898 --settle 1 --goal 0.0 0.0
[navsmoke3] 本 run 的 gzserver 起来=1（3 s）
[navsmoke3] 起来时本 run 的 gz: 200 208
[navsmoke3] decoys 存活: alive alive alive alive alive
  ✅ 本 run 剩余 gzserver/gzclient  0
  ✅ 本 run 剩余 ros2 launch        0
  ✅ 本 run 剩余节点进程            0
  ✅ decoy 存活数（应为 5/5）       5
[navsmoke3] RESULT: PASS

# ② run_remap_lowterrain（先算覆盖路线再起建图栈）
$ bash .tmp_scopetest/verify_real.sh remap2 http://127.0.0.1:11897 180 \
    tools/scripts/regress/run_remap_lowterrain.sh --tag scv_remap2 --domain 197 --port 11897
[remap2] 本 run 的 gzserver 起来=1（5 s）
  ✅ 本 run 剩余 gzserver/gzclient  0
  ✅ 本 run 剩余 ros2 launch        0
  ✅ 本 run 剩余节点进程            0
  ✅ decoy 存活数（应为 5/5）       5
[remap2] RESULT: PASS

# ③ run_robot_model_probe（原来用**永远匹配不到**的 pkill -f "gzserver.*$PORT"；连跑 3 次）
$ for i in 1 2 3; do bash .tmp_scopetest/verify_real.sh probe$i http://127.0.0.1:1188$i 240 \
    tools/scripts/regress/run_robot_model_probe.sh scv_probe_r$i --duration 5; done
[probe1] gzserver 起来=1（4 s） … [probe1] RESULT: PASS
[probe2] gzserver 起来=1（4 s） … [probe2] RESULT: PASS
[probe3] gzserver 起来=1（4 s） … [probe3] RESULT: PASS
```

> 说明：`probe` 第一次跑时曾出现 1 个 `ros2 launch` + 1 个节点残留（launch 被 SIGKILL 太早、
> 有节点脱组）——这就是为什么给 `run_robot_model_probe.sh` 在组杀之后**又加了**
> `sc_kill_own_ros2_launch` / `sc_kill_own_pattern` 兜底；加完连跑 3 次全 PASS。

### 8.5 证据 C：**跑前残留清理**也只清自己的

场景：先放一个 **同 URI** 的 gzserver（模拟"上一次跑崩了、残留占着同一个端口"）+ 4 个**别的 URI** 的
decoy，然后跑脚本：

```bash
$ bash .tmp_scopetest/verify_startup.sh startup_nav http://127.0.0.1:11895 \
    tools/scripts/localization/run_nav_smoke_regress.sh --tag scv_st --domain 195 --port 11895 --settle 1 --goal 0.0 0.0
[startup_nav] 跑前：本 URI 残留=11 别人=12 13 14 15 16
[startup_nav] 8s 后：本 URI 残留=dead  别人的 gzserver=alive gzclient=alive launch=alive slam=alive sleep=alive
[startup_nav] 收尾后：别人的 gzserver=alive gzclient=alive launch=alive slam=alive sleep=alive
[startup_nav] RESULT: PASS
# 脚本自己的日志也印证了"只认本 URI"：
[regress] 残留 gzserver（本 master URI）：11
```

### 8.6 证据 D：不再有全机 gazebo 清理

```bash
$ grep -rn "pkill -9 -x gz" tools/ scripts/ 2>/dev/null
$ echo "matches=$(grep -rn 'pkill -9 -x gz' tools/ scripts/ 2>/dev/null | wc -l)"
matches=0
```

（本仓没有 `scripts/` 目录，`grep` 会对它报 "No such file or directory" 并返回 2；
上面用 `2>/dev/null` + 计数，`tools/` 下命中 **0** 条。）

**改前 → 改后**（同样口径）：

| 站点（`git grep … HEAD -- tools/` 口径） | 改前 | 现在 |
|---|---:|---:|
| `pkill -9 -x gzserver` / `gzclient` | **18** 行（其中 **11** 行在本次点名的 9 个脚本里；另 7 行在同族的 4 个脚本里，也一并改了） | **0** |
| `pkill -9 -f '…ros2 launch…'`（含混在 alternation 里的） | **11** 行 | **0** |
| `pkill -f "gzserver.*$GZ_PORT"` / `"gzclient.*$GZ_PORT"`（**空操作**） | **7** 行可执行（`run_robot_model_probe.sh` 4 + `robot11_gui_bench.sh` 3；另 2 行是**注释**，保留作为"此路不通"的说明） | **0** |
| `pkill -f rviz2`（全机） | **1** 行 | **0**（改成 `sc_kill_own_pattern`） |

### 8.7 诚实的边界（本 part **没有**验证到的）

1. **`tools/scripts/control/start_sentinel.sh` 没改**（不在本次 9 个脚本清单里，是"一键启动"人工工具）。
   它仍然是：

   ```
   165:  pkill -f gzserver
   166:  pkill -f gzclient
   169:  pkill -f ros          ← 这条会杀掉机器上**所有** ROS 进程（包括别人的）
   172:  pkill -f rviz2
   ```

   ⇒ **建议**：人工启动一次前先确认没有别人的栈在跑；要彻底解决，把它也改成
   "`setsid` + 本 URI 核对"（本次按"不动范围外文件"处理）。
2. `robot11_gui_bench.sh` 里 `ps -o rss= -C gzclient|gzserver|rviz2` 的**采样**仍是全机口径
   （**只读、不杀**）。它需要真显示器，本沙箱验证不了，所以只改了"起窗口判定"和"清理"。
3. 除上面 3 个真跑脚本外，其余 11 个只做了 **`bash -n` 语法检查 + 合成判定**，
   **没有逐个跑完整流程**（它们动辄 5–10 分钟）。逻辑改动是同一套 `sc_*` 调用，风险面一样。
4. **沙箱限制**：本 agent 的 bash 在独立 PID 命名空间里，**看不到宿主机进程** ⇒
   无法在"宿主机上真的有一个别人的 gazebo"的条件下演戏；证据是"同命名空间内、不同 URI 的 decoy 全活"，
   加上规则本身**只按进程组/环境变量/血缘**判定（不依赖"谁的机器"）。
5. 两个**同脚本**并发跑会共用同一个 `GAZEBO_MASTER_URI`（端口硬编码），仍会互相抢端口 ——
   这是端口独占问题，不是 kill 作用域问题（脚本本来就会在端口占用时退出 3）。
6. 所有测试都在本 agent 自己的 PID 命名空间里，**跑完进程随命名空间销毁**；
   `ps`/`pgrep` 复查残留：**(none)**。


---


## 来源 3/6：`sim/rviz_guide.md`


> 目标：一眼看清「规划器眼里的世界」和「它打算怎么走」。
> 顺序按排查价值排列。

## 必加（8 个）

| # | Display 名 | 类型 | Topic | 关键设置 | 看什么 |
|---|---|---|---|---|---|
| 1 | **PlanGrid** | Map | `/tdt/plan_grid` | Color scheme: costmap；Alpha 0.6 | **规划器实际用的障碍**（静态∪ROGMap∪mask） |
| 2 | **Plan** | Path | `/plan` | 线宽 0.05，绿色 | 最终轨迹（含骨架尾段） |
| 3 | **Traj** | Path | `/trajectory_plan` | 若 RViz 无自定义类型，跳过 | 局部平滑轨迹 |
| 4 | **Map** | Map | `/map` | 灰，Alpha 0.4 | 先验静态图 |
| 5 | **Robot** | RobotModel | — | — | 车体姿态 |
| 6 | **TF** | TF | — | 只显示 base_link 相关 | 坐标系是否连上 |
| 7 | **Scan** | LaserScan | `/scan` | 蓝，点 0.02 | 实时障碍（只进 costmap） |
| 8 | **Cloud** | PointCloud2 | `/localization/lio_world_cloud` | 绿，尺寸 0.03，Decay 0 | zlio 原始点云 |

## 诊断用（按需开）

| Display | 类型 | Topic | 用途 |
|---|---|---|---|
| ROGMap 占据 | PointCloud2 / Map | `/rog_map/occupied` | 看 3D 占据原始结果 |
| ROGMap 语义 | Map | `/rog_map/layer_value` | 看 FREE/PASSABLE/OCCUPIED |
| ESDF | PointCloud2 | `/rog_map/field` | 只有 `esdf_en: true` 才有 |
| 骨架 | Path | 无（内部缓存） | 当前不发；需要可加 |
| 局部代价图 | Map | `/local_costmap/costmap` | 确认 stvl 已关 |
| 全局代价图 | Map | `/global_costmap/costmap` | 同上 |

## 排查对照

| 现象 | 先看哪个 display | 说明 |
|---|---|---|
| 车不动 | **PlanGrid** + Plan | 看车是不是被障碍包住、路径是否穿过障碍 |
| 路径绕远 | PlanGrid | 看 ROGMap 是否把地面误判成障碍 |
| 高台边缘 | PlanGrid | 边缘是否被标成障碍（负障碍生效） |
| 卡住后退 | Plan（颜色/时间） | 逃逸段是否变成一条短路径 |
| 点云偏移 | Cloud + TF | 定位是否跳变 |

## 注意

- `/tdt/plan_grid` 只在**有订阅者时**才构造发布（省 CPU），所以先加 display 再看；
- 分辨率 0.05、原点与 `/map` 一致，可以直接叠在 Map 上对比；
- 若 `mask_impl:=mask|terrain`，1 号显示同样可用（那条路也会填 `a_occ`）。


---


## 来源 4/6：`sim/gz_acceptance_checklist.md`


> 目的：把"gz 后端能用了吗"变成**可判定的几条数字**。每一项都给出：怎么量、判据、失败时先看哪。
> 本仓已完成的部分（截至提交 `3ab6d1b`）：底盘驱动插件、2027 世界渲染、chassis 模型生成、
> native 传感器链路（合成数据实测到 patchwork 与 small_point_lio 初始化）。
> **唯一没做过的**：真跑一次 gz 渲染 —— 这一步必须在本机之外的带桌面会话完成。

---

## 0. 体检（先跑这个，30 秒）

```bash
tools/scripts/gzsim/preflight.sh        # 7 组 PASS/FAIL，每项带修复命令
```

判据：**必须全 PASS**（`Embree4` 那条 WARN 可接受——rmagine configure 会联网拉）。

---

## 1. 两条路，选一条

| | A · 第三方后端（保真，**推荐**） | B · native（无第三方依赖，保底） |
|---|---|---|
| 前提 | `~/gzsim_ws/install/lib/libmid360_system.so` 存在 | 无需任何第三方 |
| 点云 | MID-360 非重复扫描图案 + **逐点时间** | 规则网格，**无逐点时间**（LIO 不去畸变） |
| 启动 | `ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz …` | 同上 + 前置 `GZ_SENSOR=native` |
| 用途 | 正式仿真 | 只验"管线能跑" |

完整命令（A）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  sim_backend:=gz world:=RMUC2027 robot:=chassis \
  mode:=mapping lio:=small_point_lio gravity:=tf mapper:=slam_toolbox \
  ground:=patchwork gui:=True spin_speed:=0.0
```

B 只多一行环境变量：`export GZ_SENSOR=native`。

---

## 2. 五项硬指标（逐条打勾）

| # | 量什么 | 命令 | 判据 | 不达标先看 |
|---|---|---|---|---|
| **1** | 仿真实时率 | `ros2 topic hz /livox/lidar` | **≈9~10 Hz**（RTF≈1.0；Classic 同机只有 6.9 Hz / 4.9 Hz） | `ign gazebo` 进程 CPU；`gz_sensor` 是否 native（规则网格也可能因渲染慢掉帧） |
| **2** | 点云进链路 | `ros2 topic hz /livox/lidar/pointcloud` | 与 #1 同量级 | native 模式看 `/gz/lidar` 有没有；桥表是否 `ros_gz_bridge_native.yaml` |
| **3** | IMU | `ros2 topic hz /livox/imu` | **≈100 Hz** | native：`ign topic -l \| grep imu`；A：插件的 imu_block |
| **4** | 底盘能动 | `ros2 topic pub -1 /cmd_vel_chassis geometry_msgs/Twist '{linear: {x: 0.3}}'`，再看 `ros2 run tf2_ros tf2_echo odom base_link` | 位置**确实变化**；`ros2 topic echo /odom_ground_truth --once` 有输出 | 插件是否加载（`<plugin filename="librm_gz_planar_move.so">`）；`GZ_SIM_SYSTEM_PLUGIN_PATH` 是否含 `<prefix>/lib` |
| **5** | LIO 出里程计 | `ros2 topic hz /Odometry` | ≥7 Hz，且 `tf2_echo odom base_link` 连续 | 见 §4 故障表 |

---

## 3. 建图/导航的判定（#1~#5 全绿之后）

### 建图（mode:=mapping）
```bash
ros2 run nav2_map_server map_saver_cli -f src/bringup/rm_nav_bringup/map/RMUC2027_gz
python3 tools/scripts/mapping/compare_2d_maps.py \
  src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm src/bringup/rm_nav_bringup/map/RMUC2027_gz.pgm
```
判据：**墙不歪、通道不糊**；与 v2 的差异主要在"墙厚"而不是"整片错位"。

### 导航（mode:=nav）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz world:=RMUC2027 mode:=nav \
  lio:=small_point_lio gravity:=tf localization:=beluga nav:=mppi planner:=smac2d \
  ground:=patchwork local_obstacle:=stvl global_obstacle:=stvl robot:=chassis \
  gui:=True map_yaml:=$HOME/HzMi_rmsimulation/src/bringup/rm_nav_bringup/map/RMUC2027_v2.yaml
```
判据：能到第一个目标点、`/plan` 有输出、不出现 `Resulting plan has 0 poses`。

---

## 4. 故障对照表（都是本仓踩过的）

| 现象 | 根因 | 处理 |
|---|---|---|
| 点云没有 / 话题空 | gz transport **partition 串了**（同机另一世界） | launch 每次随机 `GZ_PARTITION`；别手动固定 |
| `small_point_lio` 每帧 `imu loop back`、无 odom | 同上（partition）或 `custom_msg_tag≠0` | 确认 `GZ_PARTITION/IGN_PARTITION` 唯一；`--custom-msg-tag 0` |
| native：CustomMsg 有但 frame 不对 | gz 桥出来的 frame 是 **scoped name** | `pc2_to_custom -p frame_id:=livox_frame`（launch 已设） |
| native：`/gz/lidar` 空 | `gpu_lidar` 需要 GL 上下文 | 用带桌面的会话（本仓实测无 GL 时报 `X Error 152 GLX`） |
| 车不动 | 驱动插件没加载 / `--drive false` | `GZ_SIM_SYSTEM_PLUGIN_PATH` 加 `<prefix>/lib`；生成器 `--drive true` |
| 出生就掉/穿地 | 出生 z 不对 | chassis = **0.098**、robot11 = **0.1025**（`ROBOT_SPAWN_Z`） |
| 场地模型找不到 | `model://RMUC2027_world` 不在解析根 | `hzmi_rm_simulation/world` 必须在 `GZ_SIM_RESOURCE_PATH`（launch 已加） |

---

## 5. 回退（任何一步不对都能秒退）

```bash
# 默认仍是 Classic：不传 sim_backend 就是 classic
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2027 mode:=mapping …
# 清掉 gz 的临时目录（世界渲染/模型生成都在 $TMPDIR/rm_livox_gz_sim_*）
rm -rf /tmp/rm_livox_gz_sim_*
```

证据（默认路径零改动）：`git diff --name-only 447275a~1..HEAD` 只涉及 gz opt-in 分支的 6 个文件 +
2 个 docs，**没有** `bringup_sim.launch.py` / `nav2_params*` / `laser plugin`。


---


## 来源 5/6：`mapping/runbook_2027_mapping.md`

################################################################################
# RMUC2027 —— 建图/导航 Runbook（2026-11-03，v2 完善版）
#
# 设计原则（2026-11-03 拍板）
#   · 2D 与 3D **分开做**：2D 只由 slam_toolbox 出 PGM；3D 只由 LIO-SAM 出 PCD
#   · 两者唯一耦合 = `map` 系是**出生点相对系**（spawn 固定）⇒ 产物天然同帧
#   · 方向只有 3D→2D（投影）；2D→3D 不可能（PGM 无高度）
#   · 命名全程一致（见下），否则 nav 找不到图
#
# 命名约定（**launch 里 `map_name` 决定一切**）
#   2D : map/<map_name>.pgm / .yaml / .posegraph / .data
#   3D : PCD/<map_name>.pcd        （launch 的 pcd_path = PCD/<map_name>.pcd，已核对 launch:4263）
#   SC : PCD/<map_name>_sc_db.npz
#   本次统一用 map_name := RMUC2027_v5
################################################################################

cd ~/HzMi_rmsimulation && source install/setup.bash


# ---------------------------- 产物归属（谁消费谁）----------------------------
#   PGM(RMUC2027_v2.*) ── nav 必需：map_server → 全局代价图 → SMAC2D 规划；
#                         2D 定位时还给 AMCL 匹配。**3D 模块不消费它。**
#                       已实测：门洞 100% 自由、走廊 86.8% 自由、格式 P5/三态合规。
#   PCD(*)             ── 只有 3D 定位用：small_gicp/gicp 精配；SC 库只对它有意义。
#   SC 库(.npz)        ── 只有 scan_context 用（给 3D 定位一个粗初值）。
#   两条线可并行：2D 现在就能导航；3D 建图当独立一条线慢慢做。
# ---------------------------------------------------------------------------


# ##############################################################################
# A · 2D 建图（slam_toolbox）
# ##############################################################################
# 只开 Gazebo + LIO(供 odom/TF) + slam_toolbox；**不开** LIO-SAM / cloud_accumulator。

# 2D 建图用 15000 射线（RTF≈0.69；2D 不需要堆点）
sed -i 's/name="samples" value="[0-9]*"/name="samples" value="15000"/' \
  src/simulation/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro

ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2027 map_name:=RMUC2027_v5 mode:=mapping \
  lio:=small_point_lio gravity:=tf mapper:=slam_toolbox \
  ground:=patchwork robot:=chassis \
  gui:=True gazebo_offline:=True nav_rviz:=True spin_speed:=0.0

# 开一圈：慢速、**少原地旋转**（slam_toolbox 对旋转最敏感）。

# ---- 保存：**车停稳、launch 仍开着**时另开终端执行 ----
#      -f 的名字必须 == map_name（否则后面 nav 的 map_yaml 对不上）
ros2 run nav2_map_server map_saver_cli -f src/bringup/rm_nav_bringup/map/RMUC2027_v5
tools/scripts/mapping/map_archive.sh save --name RMUC2027_v5 --world RMUC2027

# ---- 验收 2D 图 ----
python3 tools/scripts/mapping/compare_2d_maps.py \
  src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm src/bringup/rm_nav_bringup/map/RMUC2027_v5.pgm
#   看墙有没有变厚/重影；差异越小越好（v2 = 已知可用那版）
#   另可查关键通道是否通 —— 见文末「附 B · 通道抽查」


# ##############################################################################
# B · 3D 建图（LIO-SAM，带回环）—— 独立的一次跑，只出 PCD
# ##############################################################################
# slam_toolbox 仍要起（本仓 mode:=mapping 需要它提供 map→odom，否则栈不完整），
# 但 LIO-SAM 的 TF 走 /lio_sam/tf*，**不读也不写**主栈 map→odom。

# 3D 用 30000 射线（点多；RTF≈0.49，慢但可接受）
sed -i 's/name="samples" value="[0-9]*"/name="samples" value="30000"/' \
  src/simulation/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro
export LIVOX_INTRA_FRAME_TIME=1        # ★ 逐点时间（必须在**起仿真前** export，否则点云不带 offset_time）

# ---- 终端 1：仿真 + 2D 后端（只为栈完整/TF；本段不保存 2D）----
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2027 map_name:=RMUC2027_v5_3d mode:=mapping \
  lio:=small_point_lio gravity:=tf mapper:=slam_toolbox \
  ground:=patchwork robot:=chassis \
  gui:=True gazebo_offline:=True nav_rviz:=True spin_speed:=0.0

# ---- 终端 2：LIO-SAM（自己延迟 6 s 启动，等 spawn 落定 ⇒ map 原点不被 0.2 m 下落污染）----
export LIVOX_INTRA_FRAME_TIME=1
ros2 launch lio_sam sim.launch.py

# ---- 终端 3：体检（全绿再开圈）----
ros2 topic hz /livox/lidar          # ≈4.9 Hz（30000）／≈6.9 Hz（15000）
ros2 topic hz /lio_sam/points       # 应≈上面那个值；差太多 ⇒ 适配节点跟不上
ros2 topic hz /imu/data             # ≥100 Hz
ros2 topic hz /lio_sam/odometry/imu # 有输出 ⇒ IMU 预积分在跑

# ---- 终端 3（续）：录 bag（给 SC 建库；必须与 3D 同一次跑）----
mkdir -p ~/HzMi_rmsimulation/bag
ros2 bag record -o ~/HzMi_rmsimulation/bag/raw3d_$(date +%Y%m%d_%H%M) \
  /cloud_registered /tf /tf_static /scan /odom

# ---- 开一圈：慢速、单趟、尽量走满（来回走必双层）----

# ---- 保存 3D（另开终端）----
# ⚠️ LIO-SAM 会**先删掉整个 savePCDDirectory 再重建**（上游行为），别指向重要目录
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.05}"
#   → $HOME/HzMi_rmsimulation/export/lio_sam_pcd/{CornerMap,SurfMap,trajectory,transformations}.pcd

# ---- 合并两份特征云（LIO-SAM 存的是 Corner + Surf 两份，不是一份）----
python3 tools/scripts/diag/pcd_merge.py \
  export/lio_sam_pcd/merged.pcd \
  export/lio_sam_pcd/CornerMap.pcd export/lio_sam_pcd/SurfMap.pcd --voxel 0.05

# ---- 验收 3D（不达标就别改名，先查根因）----
python3 tools/scripts/diag/pcd_quality.py export/lio_sam_pcd/merged.pcd
#   判据：地面厚度 < 0.10 m、双层比例 < 20%（旧图对照：0.357 m / 83.5%）
python3 tools/scripts/diag/pcd_preview.py export/lio_sam_pcd/merged.pcd --out export/lio_sam_preview
#   看 export/lio_sam_preview/xz.png：地面是"一条线"还是"两条线"
python3 tools/scripts/diag/pcd2ply.py export/lio_sam_pcd/merged.pcd
CloudCompare export/lio_sam_pcd/merged.ply        # 可选：与旧图做 Cloud/Cloud Dist

# ---- 达标后落盘成代码认的名字（launch 的 pcd_path = PCD/<map_name>.pcd）----
cp export/lio_sam_pcd/merged.pcd src/bringup/rm_nav_bringup/PCD/RMUC2027_v5.pcd

# ---- 用**同一份 bag** 建 SC 库 ----
BAG=$(ls -dt ~/HzMi_rmsimulation/bag/raw3d_* | head -1); echo "用 bag: $BAG"
ros2 run scan_context_relocalizer build_sc_db -- \
  --bag "$BAG" --world RMUC2027 \
  --out src/bringup/rm_nav_bringup/PCD/RMUC2027_v5_sc_db.npz --min-move 0.35


# ##############################################################################
# C · 可选：让 3D 直接生成 2D（"3D 权威"路线，与 A 段**二选一**）
# ##############################################################################
# ⚠️ 输出前缀必须和 A 段不同（否则覆盖 A 段的图，无法对比）
python3 tools/scripts/mapping/pcd_to_nav2_map.py \
  --pcd src/bringup/rm_nav_bringup/PCD/RMUC2027_v5.pcd \
  --out src/bringup/rm_nav_bringup/map/RMUC2027_v5_from3d

# 对比后再决定用哪张（差异小 ⇒ 可用 3D 派生版，对坡更"懂"）
python3 tools/scripts/mapping/compare_2d_maps.py \
  src/bringup/rm_nav_bringup/map/RMUC2027_v5.pgm src/bringup/rm_nav_bringup/map/RMUC2027_v5_from3d.pgm


# ##############################################################################
# D · 导航
# ##############################################################################
# 导航回 15000 射线（RTF 0.69）
sed -i 's/name="samples" value="[0-9]*"/name="samples" value="15000"/' \
  src/simulation/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro

# ---- D1. 2D 定位（最稳，先跑这档）----
#   gravity:=tf 与 A/B 段一致（odom_level 的 z 原点=地面）。
#   若你平时用 gravity:=lio：2D 图/规划不受影响，但 3D 定位的 z 有 0.138 m 口径差
#   ⇒ **3D 定位请统一 gravity:=tf**。
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2027 map_name:=RMUC2027_v5 mode:=nav \
  lio:=small_point_lio gravity:=tf localization:=beluga \
  nav:=mppi planner:=smac2d ground:=patchwork \
  local_obstacle:=stvl global_obstacle:=stvl robot:=chassis \
  spin_speed:=0.0 nav_rviz:=True gui:=True gazebo_offline:=True \
  map_yaml:=$HOME/HzMi_rmsimulation/src/bringup/rm_nav_bringup/map/RMUC2027_v5.yaml

# ---- D2. 3D 定位（B 段验收通过后用）----
#   读 PCD/<map_name>.pcd + PCD/<map_name>_sc_db.npz（已核对 launch 的 name_v 语义）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2027 map_name:=RMUC2027_v5 mode:=nav \
  lio:=small_point_lio gravity:=tf localization:=small_gicp \
  global_relocalizer:=scan_context \
  nav:=mppi planner:=smac2d ground:=patchwork \
  local_obstacle:=stvl global_obstacle:=stvl robot:=chassis \
  spin_speed:=0.0 nav_rviz:=True gui:=True gazebo_offline:=True \
  map_yaml:=$HOME/HzMi_rmsimulation/src/bringup/rm_nav_bringup/map/RMUC2027_v5.yaml


# ##############################################################################
# 附 A · 诊断
# ##############################################################################
python3 tools/scripts/diag/odom_error.py                     # LIO 漂移/跳变（对比 /odom_ground_truth）
python3 tools/scripts/diag/pcd_quality.py <pcd>             # 3D 四个硬指标
python3 tools/scripts/diag/pcd_preview.py <pcd> --out <dir> # 出 PNG（不用装查看器）
python3 tools/scripts/diag/pcd2ply.py <pcd>                 # 转 PLY（给 CloudCompare）
python3 tools/scripts/diag/perception_watch.py              # 感知在 0~18cm 标了什么（2Hz+CSV）
python3 tools/scripts/diag/costmap_probe.py --at -1.2 4.1   # 定点查世界坐标处的代价
ros2 topic hz /livox/lidar                                  # = 10Hz × RTF
ros2 run tf2_ros tf2_echo odom odom_level                   # 期望恒定 [0,0,-0.138]
ros2 run tf2_ros tf2_echo map odom                          # 这个**会**变（SLAM/AMCL 在修正，正常）


# ##############################################################################
# 附 B · 通道抽查（2D 图关键通道是否通）
# ##############################################################################
python3 - <<'PY'
import numpy as np, yaml
y=yaml.safe_load(open('src/bringup/rm_nav_bringup/map/RMUC2027_v5.yaml'))
with open('src/bringup/rm_nav_bringup/map/'+y['image'],'rb') as f:
    f.readline(); tok=[]
    while len(tok)<3:
        ln=f.readline()
        if ln.startswith(b'#'): continue
        tok+=ln.split()
    w,h=int(tok[0]),int(tok[1]); img=np.frombuffer(f.read(w*h),np.uint8).reshape(h,w)
res=y['resolution']; ox,oy=y['origin'][:2]
def probe(name,x0,x1,y0,y1):
    i0=int((x0-ox)/res); i1=int((x1-ox)/res)
    j0=int((y0-oy)/res); j1=int((y1-oy)/res)
    sub=img[max(0,h-1-j1):h-j0, i0:i1]
    if sub.size==0: print('  %-22s 窗口越界' % name); return
    print('  %-22s 占用 %5.1f%%  自由 %5.1f%%' % (name,(sub<100).mean()*100,(sub>=200).mean()*100))
print('== 关键通道（黑=占用/白=自由）==')
probe('门洞(x-1.56..-0.88)', -1.56,-0.88, 3.9,4.3)
probe('走廊(y-6.68..-7.48)', -5.4,-4.6, -7.48,-6.68)
probe('出生点附近', -0.5,0.5, -0.5,0.5)
PY


# ##############################################################################
# 附 C · 常见故障
# ##############################################################################
# | 现象                              | 原因/处理                                                       |
# |-----------------------------------|-----------------------------------------------------------------|
# | LIO-SAM 报 Unknown sensor type    | params 没生效；确认 sim.launch.py 默认加载 params_sim.yaml       |
# | /lio_sam/points 没数据            | 适配节点没收到 CustomMsg；查 /livox/lidar 是否在发               |
# | 点云 offset_time 全 0             | 忘了在**起仿真前** export LIVOX_INTRA_FRAME_TIME=1                |
# | /livox/lidar 只有 ~3.8 Hz         | 30000 太重；回 15000 或 20000                                    |
# | map_saver 报错找不到 /map         | 图还没出来；等 map_update_interval（2027 槽 1.5 s）               |
# | nav 起不来：map_yaml 不存在       | map_saver -f 的名字与 map_name/map_yaml 不一致                    |
# | D2 起不来/无先验                  | PCD/<map_name>.pcd 或 _sc_db.npz 不存在（B 段没做完）             |
# | 保存 3D 后目录被清空              | 正常：LIO-SAM 先删 savePCDDirectory 再重建                       |

## 附录 C　看点云用什么工具（借鉴 T-DT 2026 雷达开源）

T-DT 那套（`T-DT-Algorithm-2026/T-DT_Radar`，实测仓库结构）点云侧是：

| 环节 | 他们的做法 | 证据 |
|---|---|---|
| 地图资产 | 一个场地 **PCD** 文件 | `config/RM2026.pcd` |
| 配准 | **PCL GICP** | `src/lidar/localization/src/localization.cpp`：`#include <pcl/registration/gicp.h>`、`pcl::io::loadPCDFile`、`pcl::toROSMsg` |
| 目标提取 | 欧式聚类 | `src/lidar/cluster/` |
| 动态点剔除 | 单独包 | `src/lidar/dynamic_cloud/` |
| **看点云** | **就是 RViz**（发布 `PointCloud2`） | 仓库里只有 `livox_*.rviz` 配置，没有别的查看器 |

结论：**他们没用什么特殊工具，就是 PCL 处理 + RViz 看**。离线看质量时才需要 CloudCompare / `pcl_viewer`。

### C.1 我们这边"一条命令看任意 PCD"

```bash
tools/scripts/diag/view_pcd.sh export/lio_sam_pcd/CornerMap.pcd      # → RViz，话题 /cloud_pcd
tools/scripts/diag/view_pcd.sh a.pcd b.pcd                           # 多文件先体素合并
FRAME=odom tools/scripts/diag/view_pcd.sh x.pcd                      # 换 frame
RV=0 tools/scripts/diag/view_pcd.sh x.pcd                            # 只发话题，自己开 RViz
```

底层就是（本机 `pcl_ros` 已装）：

```bash
ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:=$PWD/export/lio_sam_pcd/CornerMap.pcd -p tf_frame:=map -p publishing_period_ms:=1000
# 实测：Publishing data with 229057 points ... on topic /cloud_pcd in frame map.
```

RViz 里加 PointCloud2 → **`/cloud_pcd`**（不是 `/pcd_view`！pcl_ros 的话题名是固定的），
配色选 `AxisColor` + `Z` 轴。

### C.2 三种看法怎么选

| 工具 | 适合 | 命令 |
|---|---|---|
| **RViz** | 和话题/TF 一起看（在线建图） | `lio_sam_rviz:=True`（3D 建图专用配置） |
| **CloudCompare** | **离线量厚度/算重合度**（最有用） | `CloudCompare a.pcd b.pcd`，然后 `Tools ▸ Distances ▸ Cloud/Cloud Dist` |
| **pcl_viewer** | 最快瞄一眼 | `pcl_viewer x.pcd` |
| `pcd_preview.py` | 无显示器/写文档出 PNG | `python3 tools/scripts/diag/pcd_preview.py x.pcd --out .tmp_cache/prev` |

> **CloudCompare 的 Cloud-to-Cloud 距离** = `pcd_quality.py` 里"地面厚度/双层比例"的可视化版：
> 选中两朵点云（比如 `CornerMap.pcd` 与 `SurfMap.pcd`，或新旧两版地图）→
> `Tools ▸ Distances ▸ Cloud/Cloud Dist`，直方图就是"同一个面被扫了几层"的分布。

## 附录 D　从场地网格（STL/OBJ）生成 PCD

### D.1 能不能"STL → OBJ → PCD"

**能，但中间格式不是关键，"有没有对表面采样"才是关键。**

| 常见做法 | 你拿到的点 | 评价 |
|---|---|---|
| `pcl::io::loadOBJFile()` → `savePCDFile()` | **只有 OBJ 的顶点** | ❌ 平面处只有几个角点，GICP 会飘、法向估计失败 |
| 各种 `obj2pcd` 小脚本（多数只解 `v`） | 同上，顶点 | ❌ 同上 |
| CloudCompare：打开网格 → `Edit ▸ Mesh ▸ Sample points` | 表面均匀点 | ✅ |
| MeshLab：`Filters ▸ Sampling ▸ Poisson-disk Sampling` | 表面均匀点 | ✅ |
| 本仓 `mesh_to_pcd.py` | 面积加权表面采样 + 体素 | ✅ |

**走 OBJ 的真实好处**：OBJ 更通用（CloudCompare/MeshLab/trimesh 都认、能带材质），
STL→OBJ 再采样是常见流程；**但它不会替你解决采样问题**。

### D.2 一条命令（支持 .stl 与 .obj）

```bash
python3 tools/scripts/diag/mesh_to_pcd.py \
  src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUC2027_world/meshes/RMUC2027.stl \
  -o PCD/RMUC2027_cad.pcd --points 1500000 --voxel 0.05
```

实测（RMUC2027 场地）：

```
网格: 378666 三角形，bbox=28.00 x 16.00 x 3.16 m（单位缩放 x0.001 ⇒ 自动识别 mm）
表面采样: 1500000 点
体素 0.050 m 后: 488855 点
写出 .tmp_cache/RMUC2027_cad.pcd（binary PCD）
```

参数：`--points`（采样数，300 万够）、`--voxel`（0.03~0.05 m）、
`--unit auto|mm|m`、`--zmin/--zmax`（只留可观测高度带）。

### D.3 ⚠️ 别用 `pcd_quality.py` 评价 CAD 图

CAD 网格本身**含多层结构**（地面、底面、台阶侧面、护栏上下沿），所以：

```
python3 tools/scripts/diag/pcd_quality.py .tmp_cache/RMUC2027_cad.pcd
→ 双层比例 97.3%   ← 这是**正常**的，不是质量问题
```

`pcd_quality.py` 的"地面厚度/双层比例"只对**扫描出来的图**有意义
（判"同一个面被扫了几层"）。CAD 图的正确用途只有两个：
1. **GICP 的 target（先验）** —— 几何精确，比实扫图更稳；
2. **C2C 距离的基准（ground truth）** —— 但要知道它含场地施工误差（几 cm）。

### D.4 已生成：`PCD/RMUC2027_cad.pcd`（给 GICP 当先验用）

```bash
python3 tools/scripts/diag/mesh_to_pcd.py \
  src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUC2027_world/meshes/RMUC2027.stl \
  -o src/bringup/rm_nav_bringup/PCD/RMUC2027_cad.pcd \
  --points 4000000 --voxel 0.03 --zmin -0.30 --zmax 2.80
# → 378666 三角形 → 4.0M 采样 → 0.03 m 体素 → **1 356 637 点 / 15.5 MB**
```

同时**复制成 `PCD/RMUC2027.pcd`**：GICP 的 target 路径是 `PCD/<world>.pcd`
（`bringup_sim.launch.py:2731`，与 `map_name` 无关），所以这份 CAD 图会**直接生效**。
想换回扫描图就把 `PCD/RMUC2027_scan.pcd`（旧图，在 `PCD/attic/`）复制回来。

**验证 GICP 用它配准**：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz \
  world:=RMUC2027 map_name:=RMUC2027_v2 mode:=slam_nav \
  lio:=small_point_lio gravity:=tf localization:=gicp \
  ...
# 然后发一个初值（GICP 必须有初值）
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 5.9, y: 2.72, z: 0}, orientation: {w: 1.0}}}}'
```

### D.5 分辨率该取多少：实测数据（2026-11-05）

在 4 m × 4 m 的地面片区内、**全分辨率**下测最近邻间距（`scipy.cKDTree`）：

| 点云 | 总点数 | 体积 | 片区内点数 | **NN 中位** | NN p90 |
|---|---|---|---|---|---|
| CAD `--voxel 0.01` | 7 733 333 | 89 MB | 361 365 | **0.73 cm** | 1.12 cm |
| CAD `--voxel 0.02` | 2 979 714 | 34 MB | 135 431 | **1.26 cm** | 1.86 cm |
| CAD `--voxel 0.05` | 536 447 | 6.1 MB | 23 722 | **3.00 cm** | 4.41 cm |
| **实扫图** `RMUC2027_v2.pcd` | 229 057 | —— | 7 278 | **6.12 cm** | 9.27 cm |

**结论**：

1. **实扫图的间距是 ~6 cm**（且该图很可能已被降采样；即便原始更密，扫描噪声 ~1~2 cm 也限制了收益上限）；
2. CAD 0.02 版（1.26 cm）已经是实扫数据的 **~5 倍精细** ⇒ 对 GICP 而言**信息足够**；
3. 再细到 0.01（7.7M 点 / 89 MB / 间距 0.73 cm）只会让 **kd-tree 构建与近邻查询慢 2.6~3 倍**，精度无实质提升；
4. **0.01 真正有意义的场景**只有：几何测量（门宽、离障距离）、亚厘米级 C2C 评估、细杆件可视化 —— 都不是 GICP。

⇒ **GICP target 用 0.02 m；粗配准/可视化用 0.05 m；0.01 仅按需临时生成。**

### D.6 现实（实车）也**不需要** 0.01 —— 而且理由更强

仿真里 0.01 只是"没用"，现实里是"**被别的误差淹没**"。真实误差源的量级：

| 误差源 | 典型量级 | 折算到 10 m 处的横向误差 |
|---|---|---|
| 雷达-IMU **外参**标定误差 0.5° | 0.5° | **~8.7 cm** |
| **时间同步**误差 10 ms（车 2 m/s） | 2 cm | ~2 cm |
| **去畸变**残差（加减速/转向） | 1~3 cm | 1~3 cm |
| 雷达**测距噪声**（MID-360 类） | 1~2 cm | 1~2 cm |
| **建图漂移**（回环后残差） | 3~10 cm | 3~10 cm |
| （对照）CAD 目标 0.01 的分辨率 | **0.7 cm** | 0.7 cm |

⇒ **目标做到 7 mm，比误差源小一个数量级，纯属浪费**（还慢 2.6~3 倍）。

**但现实里有两条相反的需求**：

| 需求 | 结论 |
|---|---|
| **细结构不能丢**（护栏、杆件、5 cm 立柱） | ✅ 支持用 **0.02** 而不是 0.05（0.05 会把细杆打散/打没） |
| **嵌入式算力**（Jetson/工控机） | ❌ 反对 0.01（89 MB + kd-tree 构建很贵） |

**现实里的正确做法不是"加密整图"，而是"局部裁剪 + 多分辨率"**：

```
全局重定位：0.10 m 粗图（收敛域大、快）
   ↓ 拿到粗位姿
局部精配准：在预测位姿周围裁 r=10~15 m 的 0.02 m 子图（点数恒定，与全图大小无关）
```

（这正是 z-localization "选取附近先验子图" 的做法。）

**还有一条更根本的**：现实中先验图应该用**实测扫描**建，CAD 只用于仿真/基准；
而实测图的分辨率上限 = 扫描间距（我们实测 ~6 cm），**再细的地图也没有对应的数据支撑**。


### D.7 更正：CAD 图**可以**用来生成 z-localization 的 prior map bundle

我早前说"CAD 不能建 SC 库"，**这个结论要更正**。依据是他们的 `src/backend/README.md`：

> `build_pcd_map_bundle` | **从全局 PCD 模拟局部扫描**，生成关键帧与描述子地图包

也就是说：**描述子不是从"扫描数据"来的，而是从图里"仿真出局部扫描"再算的**。
⇒ 输入只要是几何准确的全局 PCD（**CAD 完全符合**），就能生成 bundle。

**先验地图包的格式**（不是裸 .pcd）：

```
prior_map/
├── keyframes.txt        # 关键帧位姿
├── clouds/              # 每帧局部点云
├── <描述子数据文件>      # scan_context 等
└── map.pcd              # 可选，仅用于显示
```

**他们的参数示例**（README）：

```bash
ros2 run backend build_pcd_map_bundle   --input CAD.pcd --out-bundle prior_map   --x-min -14 --x-max 14 --y-min -6.4 --y-max 9.6   --sensor-z 0.157 --bundle-step 1.5   --map-voxel 0.15 --scan-voxel 0.4   --descriptor-type scan_context --sc-lidar-height 0.098
```

⚠️ 注意 **`--map-voxel 0.15`**：他们的先验图体素是 **15 cm**（比我们的 0.02 m CAD 粗 7 倍）。
⇒ 再次印证：**不必纠结 CAD 要到 0.01**。

**相关工具**：`transform_map_bundle`（刚体/四点/配准变换）、`filter_map_bundle`、`clean_map_bundle`、
`rebuild_scan_context_descriptors`、`body_calibration_node`（标定传感器→车体）。

**定位状态机**（精确名字）：`NORMAL` / `OBS_DEGRADED` / `SUSPECT` / `LOST`。


---


## 来源 6/6：`mapping/README.md`


> ⚠️ **状态说明**：本目录里的建图快照是**当时那次会话的状态记录**（开头常写"✅ 已启动…"），
> **不是现行操作手册**。2026-10 已把两份快照**合并为一份**并归档原文。现行流程以 `docs/smoke_test_runbook.md` 为准：
> - 建图（slam_toolbox / cartographer）→ runbook **§1**（含落盘三件套 §1.1）
> - 边建图边导航 → runbook **§1.2**
> - cartographer 建图与纯定位 → runbook **§5 / §6**（纯定位需先生成 pbstream，三个场地目前都缺）

| 文件 | 内容 | 状态 |
|---|---|---|
| **`mapping_session_snapshots_merged.md`** | **合并版**：同一会话两份建图快照（完整步骤 + "立即开始"）→ 键位/路线/观察/排错/保存 pbstream + §9 与现行流程的差异 | ★ `现行（合并版；操作细节为 2026-09 快照）` |
| [`../archive/mapping/`](05-operations.md) | 原文 `建图操作指南.md` / `立即开始建图.md`（`git mv` 保留历史 + 顶部 `STATUS`） | `历史（已归档）` |
| `CARTOGRA_PBSTREAM_GUIDE.md` | Cartographer `.pbstream` 生成思路 | **需谨慎**：现行做法是"用 cartographer 建图后 `finish_trajectory` + `write_state` 导出"（见 runbook §5，三条原生 ros2 命令）；该文里的"由 `.png/.yaml` 转换"路线属绕过性尝试，未验证可作为纯定位输入 |
| `快速参考卡片.txt` | 实为**离线配置包**的快速参考卡片 | 分类归属应为 `docs/package/`（未移动，仅说明） |

> 落盘相关脚本：`tools/scripts/mapping/{save_grid_map.sh, save_pcd.sh}`（pbstream 不用脚本，见 runbook §5）。

## 落盘步骤（cartographer 建图，2026-09 现场确认可用）

前提：`mode:=mapping` + `lio:=fastlio` + `mapper:=cartographer` 跑完，RViz 里墙是连续深色线。

```bash
# ① 2D 栅格（nav2/AMCL 用）—— 它订阅 /map，cartographer 正好发在 /map
tools/scripts/mapping/save_grid_map.sh          # = map_saver_cli -f src/bringup/rm_nav_bringup/map/RMUL2026

# ② pbstream（cartographer 纯定位用；两条原生服务，见 runbook §5）
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: 'src/bringup/rm_nav_bringup/map/RMUL2026.pbstream', include_unfinished_submaps: false}"

# ③ 3D 点云（ICP 纯定位用；需要 LIO 还在跑）
tools/scripts/mapping/save_pcd.sh               # = ros2 service call /map_save std_srvs/srv/Trigger
```

**⚠️ 坐标系必须注意**：cartographer 的 `map` 系是**出生点相对系**（`map→odom ≈ 恒等`），
而现有 `map/RMUL2026.pgm` 是世界系（`origin: [2.68, 0.228]`）——两者相差 ≈`(4.3, 3.35)`（出生点坐标）。
所以：
- 别直接覆盖 `RMUL2026.pgm`：先 `-f .../RMUL2026_carto` 另存，跟旧图比一比；
- 若确定用新图，把 `bringup_sim.launch.py` 里的 `amcl_init_x/y` 从 `4.3/3.35` 改为 **`0.0/0.0`**（新图原点=出生点）；
- 详见 `docs/issues_and_findings.md` §3.0。

**建图参数（本次定稿，见 `cartographer.lua`）**：`hit 0.68 / miss 0.49 / num_range_data 3000 /
min_probability_to_clear 0.80`（最后一项需要 `src/localization/rm_localization/cartographer/` 这份 fork 核心）。

### 落盘踩坑：`Failed to write map ... Magick: Unable to open file (.../RMUL2026.pgm)`

**不是权限问题**（我已实测 `map/RMUL2026.pgm` 是 `-rw-rw-r--` 可写）。真因：`tools/scripts/{mapping,control}/*.sh`
里的 `PROJECT_ROOT` 少算了一层 —— 这些脚本原来在仓库根目录的 `scripts/` 下（`../..` 正好是根），
2026-09 合并进 `tools/scripts/` 后**深度多了一层**，`${SCRIPT_DIR}/../..` 算出来是 `.../HzMi_rmsimulation/tools`，
于是 `map_saver_cli -f src/bringup/rm_nav_bringup/map/RMUL2026` 实际指向 `tools/src/bringup/rm_nav_bringup/map/...`（不存在）
⇒ ImageMagick 报 `Unable to open file`，而 `map_saver` 只打 `Failed to save the map`。

**已修**（2026-09-23）：`start_sentinel.sh` / `improved_teleop.sh` / `save_pcd.sh` / `save_grid_map.sh`
的 `PROJECT_ROOT` 改成 `${SCRIPT_DIR}/../../..`（`tools/scripts/build.sh` 等深度 2 的脚本本来是对的）。
同样的原因还在 `tools/` 下留下了空的 `build/ install/ log/`（可删）。

## 📌 本次落盘记录（2026-09-23，RMUL2026，patched 核心）

| 资产 | 路径 | 大小/体检 |
|---|---|---|
| 2D 栅格 | `src/bringup/rm_nav_bringup/map/RMUL2026.pgm` + `.yaml` | 249×173 @0.05 m；**占据 8.6% / 自由 79.4% / 未知 12.0%** ⇒ 可用底图（此前"全灰"时未知占绝大多数）|
| 旧图备份（世界系） | `map/RMUL2026_world_backup.pgm` + `.yaml` | 覆盖前先备份 |
| pbstream | `map/RMUL2026.pbstream` | **1.73 MB**（原先是 526 B 空壳）⇒ cartographer 纯定位可用 |
| 3D 点云 | `src/bringup/rm_nav_bringup/PCD/RMUL2026.pcd` | 53164 点 / 1.70 MB |

**建图配置**：`mode:=mapping` + `lio:=fastlio` + `mapper:=cartographer`；
`hit 0.68 / miss 0.49 / num_range_data 3000 / min_probability_to_clear 0.80`（最后一项依赖
`src/localization/rm_localization/cartographer/` 这份 fork 核心）。

**⚠️ 坐标系（落盘后必须做的一步）**：新 `.yaml` 的 `origin: [-2.2, -3.15, 0]` 是**出生点相对系**，
与旧图（世界系 `[2.68, 0.228]`）差 ≈`(4.3, 3.35)`。⇒ 用新图做导航/定位前，把
`bringup_sim.launch.py` 的 `amcl_init_x/y` 从 `4.3/3.35` 改成 **`0.0/0.0`**（新图原点 = 出生点）；
否则 AMCL 初值会偏 4 米多。若要继续用旧图，就用 `RMUL2026_world_backup.*` 覆盖回来。

**⚠️ PCD 落盘路径的坑**：`save_pcd.sh`/`/map_save` 由 LIO 节点写盘，实际落在
`install/rm_nav_bringup/share/rm_nav_bringup/PCD/RMUL2026.pcd`（install 树，`rm -rf build install` 就没了）。
本次已手动复制到 `src/bringup/rm_nav_bringup/PCD/RMUL2026.pcd`；以后存完记得 `cp` 一次（或把
`fastlio_mid360_sim.yaml` 的 `pcd_save`/`map_file_path` 指到 `src/` 下的绝对路径）。

## ✅ nav 模式（先建图后导航）验收 gate —— 2026-09-23 实测

`world:=RMUL2026 mode:=nav lio:=fastlio localization:=amcl nav:=rpp spin_speed:=0.0`

| 检查 | 命令 | 实测 |
|---|---|---|
| 雷达点云 | `ros2 topic hz /livox/lidar/pointcloud` | 启动窗口内会显示 1 Hz 左右（**这是 `ros2 topic hz` 反序列化 30000 点大消息跟不上的假象**，不是真丢；稳定后正常） |
| 2D 扫描 | `ros2 topic hz /scan` | 启动那窗会显示 0.2 Hz（含 ~14.7 s 启动空洞），**稳定后 7.9 Hz（墙钟）≈ 10 Hz（仿真钟）** ✔ |
| 地图 | `ros2 lifecycle get /map_server` | `active [3]` ✔ |
| 定位 | `ros2 lifecycle get /amcl` | `active [3]` ✔ |
| `map→odom` | `ros2 run tf2_ros tf2_echo map odom` | 存在且 **稳定：平移 ≈ (0.116, 0.002, 0.132)**（x/z 偏移 = `base_link→imu_link/livox_frame` 的安装偏移，**不是误差**）；机器人静止时数值不漂 ✔ |
| AMCL 位姿 | `ros2 topic echo /amcl_pose --once` | `(0,0,0)` @ `frame_id: map` ✔ = 新图（出生点系）下出生点坐标；**注意**：机器人不动时 AMCL 不满足 `update_min_d/a` ⇒ `/amcl_pose` 会停在启动那一帧（协方差全 0），**不是坏了**，`map→odom` 仍在正常发 |

**踩过的坑（两次启动撞车）**：上一次 launch 的 `gzserver/rviz2/component_container` 没清干净就再启动，
会出现"第一次能用、第二次 RViz 加载不出来 + `/scan` 停更 14 s + `map` 系不存在"。
⇒ **每次必须先** `pkill -f gzserver; pkill -f gzclient; pkill -f rviz2; pkill -f component_container; sleep 3`。
（`map` 系由 **AMCL 收到激光后**才发 `map→odom`；`/scan` 一停 ⇒ 没有 `map` 系 ⇒ costmap 变换超时 ⇒ 目标点必然不执行。）


---

---

# 来源 7/7：`2026-11 新增操作面`（本轮现场总结，之前没有文档化）

> 本节记录**本轮新增/改过**的操作入口：三套模式、地图从点云到栅格的完整流程、日志与诊断脚本。
> 命令均在本机跑过；参数以脚本 `--help` 为准。

## 1. 三套模式（一个 launch 切）

```bash
# 直接 launch
ros2 launch rm_nav_bringup bringup_mode.launch.py mode:=tdt     # T-DT 主线（默认）
ros2 launch rm_nav_bringup bringup_mode.launch.py mode:=bit     # BIT 的 MINCO 插件对（实验）
ros2 launch rm_nav_bringup bringup_mode.launch.py mode:=std     # 原版 nav2（navfn+dwb），对照组
```

| mode | planner | controller | mask_impl | 局部障碍 | 状态 |
|---|---|---|---|---|---|
| `tdt` | `tdt` | `rm_tdt_local` | `costmap` | `stvl` | **已验证**（主线）|
| `bit` | `bit`（minco_planner）| `bit`（minco_controller）| `costmap` | `stvl` | 实验：插件能加载、数据已通，控制未调 |
| `std` | `navfn` | `dwb` | `costmap` | `stvl` | 可起，未做性能对比 |

槽位参数**显式覆盖优先**：`mode:=std planner:=smac2d` 会用 smac2d。

## 2. `run_sim.sh` 的全部开关

```bash
./tools/scripts/run_sim.sh [局部障碍层] [gui|nogui]

局部障碍层: stvl(默认) | scan | stvl_both      # rogmap / terrain 档已废弃，传了直接报错退出
```

环境变量（`VAR=值 ./tools/scripts/run_sim.sh`）：

| 变量 | 作用 | 例 |
|---|---|---|
| `MODE` | 走 `bringup_mode.launch.py` | `MODE=std` |
| `MAP_YAML` | 换静态地图（绝对路径）| `MAP_YAML=$PWD/src/bringup/rm_nav_bringup/map/RMUC2027_cad.yaml` |
| `UWB=1` | 起 `/odom_ground_truth → /uwb_pose` 桥 + 换后端配置 | `UWB=1` |
| `BACKEND_CONFIG` | 换 zlio 后端配置 | `BACKEND_CONFIG=tools/configs/zlio_backend_sim_uwb.yaml` |
| `GZ_CORES` / `GZCLIENT_CORES` | 绑核 | `GZ_CORES=0-7 GZCLIENT_CORES=8-11` |

**启动后必须等绿底 READY 横幅**（约 178 s，8 个 lifecycle 节点逐个激活）；在它之前发的目标会被算出来但丢掉（`/cmd_vel` 还没有发布者）。

## 3. 地图：点云 → 栅格（完整流程）

```bash
# ① STL/网格 → 点云（双面 CAD 必须 --drop-downward，否则地面/平台被判成障碍）
python3 tools/scripts/diag/mesh_to_pcd.py src/simulation/rm_simulation/.../RMUC2027.stl \
    -o .tmp_diag/gridmap/cad.pcd --points 8000000 --voxel 0.02 --drop-downward

# ② 点云 → 栅格（默认判据 = clearance 车高带；坡/低矮空间可走）
python3 tools/scripts/mapping/pcd_to_nav2_map.py --pcd .tmp_diag/gridmap/cad.pcd \
    --out .tmp_diag/gridmap/cad --resolution 0.05 \
    --origin-shift -5.90 -2.72          # ★ CAD 坐标 → 仿真世界原点，见 §3.1
```

**★ 3.1 最容易错的地方：origin**
CAD 网格的坐标原点和仿真世界原点**不一致**（RMUC2027 实测差 `(-5.90, -2.72)`）。
不修的话机器人出生点会落在墙里（实测 ±1 m 内 48% 是障碍；修完 1%）。
判据：拿生成的图与 `src/bringup/rm_nav_bringup/map/RMUC2027_v2_thick1.pgm` 比，
**基准墙漏判率 <30% 才算对齐**（`map_check.py` 会直接给结论）。

```bash
# ③ 自查（改完图必跑）：origin / 出生点 / 对齐度
python3 tools/scripts/mapping/map_check.py .tmp_diag/gridmap/cad.pgm --pts 0,0

# ④ 手工修（GUI：左键涂黑、右键擦白、滚轮缩放、Ctrl+Z 撤销、保存自动备份）
python3 tools/scripts/mapping/map_brush.py .tmp_diag/gridmap/cad.pgm

#    批处理（世界坐标矩形）
python3 tools/scripts/mapping/map_edit.py erase 图.pgm --rect 2.0 -1.0 3.0 2.0 --world
python3 tools/scripts/mapping/map_edit.py add   图.pgm --rect -5.0 2.0 1.0 4.0 --world

# ⑤ 只加粗"薄"的墙（1~2 格的墙会被 T-DT 非严格碰撞允许的 ≤2 格容差穿过去）
python3 tools/scripts/mapping/thicken_thin.py --in 图.pgm --out 图_thick.pgm --cells 1 --preview

# ⑥ 用
MAP_YAML=$PWD/图.yaml ./tools/scripts/run_sim.sh
```

**当前默认图**：`src/bringup/rm_nav_bringup/map/RMUC2027_cad.yaml`（origin `-20.40 / -9.62`，人工 P 过）。

## 4. 日志与事后分析

```
.tmp_diag/run/<时间戳>/
├── launch.log        全量（所有节点 stdout）
├── roslog/           ROS 原生按节点分文件
├── layers.csv        四层状态，2 s 一行
├── failures.log      分类计数 + 原文（post_run 自动生成）
└── by_layer/         按层切分 00_感知 / 10_全局 / 20_局部 / 30_执行 / 40_TF / 50_状态 / 60_nav2 / 70_定位
```

```bash
L=$(ls -1dt .tmp_diag/run/*/ | head -1)

grep "问题清单" -A20 $L/failures.log      # 先看这张表
grep -c "MinimumSnap 失败" $L/launch.log  # 单点计数
grep "收到路径" $L/launch.log | tail -5    # 控制器收到的路径形态（点数/长度/两端距离）
grep -oE "车身命令 [0-9.]+" $L/launch.log | awk '{c=$2+0;n++;s+=c;if(c<0.2)lo++} END{printf "均 %.2f | <0.2 占 %.0f%%\n", s/n, 100*lo/n}'
```

**Ctrl-C 一定能把进程树收干净**（launch 在独立进程组，收尾 TERM→KILL）；异常退出会直接把 `[ERROR]` 原文打出来。

## 5. 诊断脚本速查

| 脚本 | 用途 |
|---|---|
| `tools/scripts/nav/check_bit_inputs.sh [秒]` | `MODE=bit` 时查 5 条话题在不在（点云/里程计是硬指标）|
| `tools/scripts/nav/split_run_log.py <run_dir>` | 手动重切某次运行的日志 |
| `tools/scripts/nav/wait_ready.sh` | 轮询 8 个 lifecycle 节点，READY 后打横幅 |
| `tools/scripts/nav/tdt_goal.py x y --send` | 发 nav2 目标（任何 mode 都能用）|
| `tools/scripts/nav/sim_uwb_from_gt.py --rate 20 --noise 0.02` | `/odom_ground_truth → /uwb_pose`，单独跑也用这个 |
| `tools/scripts/nav/topic_relay.py --type odom --src A --dst B` | 通用话题中继（本机没装 topic_tools）|
| `tools/scripts/mapping/diag_map.py --pcd … --grid … --out png` | 地形分区着色：红=坡被当障碍、橙=平地被当障碍 |
| `tools/scripts/mapping/probe_cell.py --pcd … --x … --y …` | 点一个坐标查"为什么被判障碍" |
| `build/rm_tdt_planner/minisnap_points_probe --pts "x,y …" [--pgm 图] [--dilate N]` | 离线复现 MinSnap 失败（含 `--strict/--no-strict`）|

## 6. UWB（仿真）

```bash
UWB=1 ./tools/scripts/run_sim.sh        # 自动起桥 + 换后端配置
```

| 前提 | 说明 |
|---|---|
| 数据源 | `/odom_ground_truth`（Gazebo 世界真值）→ `/uwb_pose`（frame=`map`）|
| 后端配置 | `tools/configs/zlio_backend_sim_uwb.yaml`（`uwb.enabled=true`、`zero_pose=0`、`estimate_zero_pose_online=false`）|
| 建图模式 | **必须 `UWB=0`**，否则后端卡在 Initializing、一帧关键帧都建不出来 |

## 7. BIT 模式的前提（跑之前先确认）

1. 插件加载：日志里要有
   `Created global planner plugin GridBased of type minco_planner/MincoPlanner` 和
   `Created controller : FollowPath of type minco_controller/MincoMpcController`；
2. 两条中继要起来（`topic_relay.py`）：`/localization/odometry → /aft_mapped_to_init`、`/cmd_vel_mpc → /cmd_vel`；
3. **ROGMap 是 minco_planner 在进程内自建的**（不是外部节点），参数在 `GridBased.rog_map.*`：
   `ros_callback.cloud_topic=/localization/lio_world_cloud`、`ros_callback.odom_topic=/localization/odometry`、
   `esdf.enable=true`、`projection.prior_map.yaml_path=…/RMUC2027_cad.yaml`；
4. 数据自查：`./tools/scripts/nav/check_bit_inputs.sh 5`，前两行（点云 1.9 Hz、里程计 18.7 Hz）都要有 Hz。

## 8. 本轮废弃/退役的入口（别再用）

| 入口 | 状态 |
|---|---|
| `run_sim.sh rogmap`、`run_sim.sh terrain` | **直接报错退出**（rogmap 2026-11-11 废弃；terrain 2026-11-13 退役）|
| `mask_impl:=terrain` | 节点已从 launch 移除 |
| `/localization/lio_world_cloud` 当 mask 输入 | 已改用 `plan_mask_from_costmap`（STVL→costmap→/plan_mask 桥）|
