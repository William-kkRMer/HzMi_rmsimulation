# rm_mpc_controller —— 走廊约束 MPC（与 tdt-nav-kit 规划器对接）

**状态**：离线可跑（无 ROS / Gazebo）。链路已通：
`A* / KinodynamicAstar`（tdt-nav-kit）→ `SfcSquare` 走廊 → `mpc_core`（OSQP）→ 控制量。

## 目录

```
src/tdt/                  第三方规划内核（MIT，原样拷贝，见其 README）
src/mpc_core/             我们的 QP-TS 求解器（Eigen + OSQP，4 状态）
test/offline_integration.cpp   端到端离线测试（自建栅格 → 规划 → 走廊 → MPC 闭环）
test/build.sh             需要 tools/mpc_proto/.deps 里的 OSQP（不需要 sudo）
```

## 构建与运行

```bash
tools/mpc_proto/build.sh          # 首次：取 OSQP 到 tools/mpc_proto/.deps
src/nav/rm_mpc_controller/test/build.sh
src/nav/rm_mpc_controller/test/offline_integration
```

## 实测（30 m × 20 m 栅格，两道墙+缺口，10 cm 膨胀）

软约束版（当前）：

```
A*: 481 点, 长度 29.18 m
KinodynamicAstar: success=1 samples=603 nodes=3077 耗时 ~7 ms
走廊: 62 个 box（maxRange 3 m, shrink 5 cm, 抽稀每 10 样本；其中退化 6 个）
MPC: steps=400 走廊不可行=0 仍失败=0  RMSE=0.0296 m  max走廊违背=0.016 m
     求解 0.11 ms 均值 / 0.18 ms 最大  迭代均值 51  slack 均值 0.001 / 最大 0.024 m
```

参数：`dt=0.05`、`N=20`、`vmax=2.0`、`amax=4.0`、`w_slack=2000`。

### 软约束（slack）怎么做的

每个 stage 给 `x/y` 的上下界各配一个松弛变量 `s≥0`：

```
x ≤ xmax + s_hi,   x ≥ xmin - s_lo,   y ≤ ymax + s_hi,   y ≥ ymin - s_lo
代价 += w_slack * Σs²        （w_slack=2000，远大于跟踪权重 80）
```

好处：**硬的不可行永远变成"贵一点"的解**——不会出现 `OSQP` 直接 `primal infeasible`。
副作用：极端场景会允许小量越界（实测最大 1.6 cm）。

### 退化 box 的处理（关键坑）

`SfcSquare::getCorridor` 在**贴墙的点**上会返回**零尺寸 box**（如 `x[14.89,14.89]`）。
如果直接拿来当约束，等于把状态"钉死在一个点"⇒ 必然不可行/大幅越界。
处理：**宽度或高度 < 0.10 m 的 box 跳过该 stage 的走廊约束**（置为无限），
该段只靠参考轨迹保证安全（此时实测越界从 0.22 m 降到 0.016 m）。

## 关键设计决定（踩坑记录）

| 问题 | 处理 |
|---|---|
| 网格约定相反 | **YAstar: 0=free**；**SfcSquare: 0=障碍（纯黑不可通行）** ⇒ 共用栅格必须取反 |
| 走廊 box 太多（603 个） | 轨迹**抽稀每 10 个样本**再求走廊 ⇒ 62 个 |
| 走廊过紧导致 QP 不可行 | **外扩 0.25 m**；并保留**回退**（去走廊再解，计数） |
| tdt 源码缺 `<iostream>` | 编译时 `-include iostream -include algorithm -include cmath`（**不改上游文件**） |
| tdt 需要 C++20 | `std::span` / `<ranges>` ⇒ `-std=c++20` |
| 时序 | 每个 stage 的 box 按**该 stage 时间对应的轨迹样本**取，不要"最近中心"（会跳到远处 box） |

## 下一步

1. **走廊松弛变量（slack）**：把"外扩 0.25 m"换成带惩罚的软约束，避免硬约束不可行；
2. 接 ROS：`nav2_core::Controller` 插件（`FollowPath`），或独立节点 `/plan` → `/cmd_vel`；
3. 用 `YAstar` 的 Mask 层接**动态障碍**（对手车）；
4. 与 MPPI 在同场景 A/B。

## nav2 Controller 插件（2026-11-06 已通）

```bash
colcon build --packages-select rm_mpc_controller
ros2 run rm_mpc_controller load_test        # ✓ 插件加载成功: rm_mpc_controller/MpcController
```

在 nav2 里启用：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "rm_mpc_controller/MpcController"
```

本仓已加 `nav:=rm_mpc` 槽位（`src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_controller_rm_mpc.yaml`）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=classic world:=RMUC2027 \
  map_name:=RMUC2027_v2 mode:=nav map_yaml:=$PWD/src/bringup/rm_nav_bringup/map/RMUC2027_v2.yaml \
  localization:=beluga nav:=rm_mpc planner:=smac2d ground:=patchwork \
  local_obstacle:=stvl global_obstacle:=stvl robot:=chassis spin_speed:=0.0 gui:=False nav_rviz:=False
```

Classic 实测（2026-11-06）：

```
[controller_server]: Created controller : FollowPath of type rm_mpc_controller/MpcController
[rm_mpc] configured: N=20 dt=0.050 vmax=2.00 amax=4.00 v_ref=1.00 half_w=0.35
/cmd_vel 采样: x=0.183, y=-0.072  （有横向分量 ⇒ 全向输出生效）
QP 失败次数: 0
```

## Classic A/B（2026-11-06 首轮，非受控）

| 控制器 | 结果 | 移动距离 | 距目标 | 直线效率 |
|---|---|---|---|---|
| **MPPI**（`nav:=mppi`） | **SUCCEEDED** | 8.08 m | 0.38 m | **95.5 %** |
| **rm_mpc**（`nav:=rm_mpc`，首轮有效） | 超时未完成 | 6.26 m | 2.23 m | 74.1 % |

> ⚠️ 这组数**不是受控对比**：两次运行的 RTF 不同、机器人出生时刻不同、单次超时也不同。
> 但可以确认：**MPC 能驱动车走（6+ m），只是效率低于 MPPI**。

**已修的 bug**：航向误差原先取"最近路径点方向"，最近点常在车正下方 ⇒ `atan2(0,0)` 方向乱 ⇒ 车画龙。
改为取**路径切线**（前视 6 个点）。

**待调参数**（按影响排序）：
1. `v_ref`（当前 1.0 m/s）与 `w_u`（当前 4.0）——决定加速快慢；
2. `horizon`（20）与前视点数（6）；
3. `corridor_half_width`（0.35）——过窄会把车"夹"住。

**下一步**：写一个**受控 A/B 脚本**（固定仿真时长、同起点、同目标、按仿真时间记录进度），
再决定是继续调 MPC 还是先用 MPPI。

## Classic 受控测量（2026-11-06 晚，同一目标 (8.0,2.7)，同窗口 ~16 s 仿真时间）

| 控制器 | 移动 | 等效速度 | 距目标 | 备注 |
|---|---|---|---|---|
| **MPPI** | **8.14 m** | **0.51 m/s** | 0.44 m | 正常到达 |
| rm_mpc（修复前） | 1.52 m | 0.10 m/s | 7.04 m | — |
| rm_mpc（世界系速度修复后） | 1.53 m | 0.10 m/s | 7.02 m | 无变化 |
| rm_mpc（内部速度估计后） | 0.54 m | 0.03 m/s | 7.96 m | 更差 |

**实测命令**：`/cmd_vel` 只有 **0.13~0.22 m/s**（MPPI 是 ~1 m/s），
而 `/odom` 的 **twist 恒为 0**（只有 pose 在变）。

**已确认的两个问题**：
1. nav2 传进来的 `velocity`（车体系 twist）**不可用于速度反馈**（本栈 `/odom` twist 全 0）
   ⇒ 命令被卡在 `amax*dt = 0.2 m/s`；
2. 加了"内部速度估计"后反而更差 ⇒ 说明**命令生成链路还有别的 bug**，不能靠猜。

**下一步（必须先加观测再改）**：
在 `computeVelocityCommands` 里把 `v_now / a0 / ref[0] / v_cmd` 每个周期发到一个 debug 话题，
跑一次 30 s 就能看出是 `a0` 太小、还是 `v_now` 被错误重置、还是参考本身有问题。

## 调试话题 `/mpc_debug`（Float64MultiArray，30 Hz）

字段顺序：

```
[0] vx_now  [1] vy_now（世界系，实际用于求解的速度）
[2] ax0     [3] ay0（QP 第 0 段加速度，世界系）
[4] ref_x   [5] ref_y   [6] ref_vx  [7] ref_vy（第 0 段参考）
[8] cmd_vx  [9] cmd_vy  [10] cmd_wz（最终下发）
[11] near_idx [12] s_now [13] 求解 ms [14] 迭代 [15] ok
```

**用它抓到的两个真 bug**：

1. **`near_idx` 恒为 0、`s_now` 恒为 0** —— 投影一直停在路径起点（参考点永远在 ~0.2 m 处，
   车永远"追不上"参考）；
2. **`vx_now/vy_now` 出现 0.69/0.68 的异常值**（车实际只走 0.03 m/s）⇒ nav2 传入的 twist 不可信。

**已改**：命令不再用 `v_now + a·dt`，改成**直接下发 QP 第 0 段的预测速度**：

| 版本 | cmd_vel | 15 s 移动 | 等效速度 |
|---|---|---|---|
| 旧（v_now+a·dt） | 0.13~0.22 m/s | 1.52 m | 0.10 m/s |
| **新（预测速度直发）** | **0.40 m/s** | **4.15 m** | **0.28 m/s** |
| MPPI（对照） | ~1.0 m/s | 8.14 m | 0.51 m/s |

⇒ 提速 2.7 倍，但仍只有 MPPI 的一半。

**下一步（明确）**：
1. 修 `near_idx` 投影（路径起点 vs 车 pose 打印出来看，怀疑坐标系或 `s_` 计算）；
2. 命令改取**第 5 段**（0.25 s 前视）的速度，而不是第 0 段（受加速度限制）；
3. 或直接把 `ref[0].vx/vy` 当巡航速度下发（运动学底盘可跟随）。

## 关键重构：**速度空间 MPC**（对齐 prox_mpc 的公式化）

**证据**（prox_mpc 源码 `prox_mpc_core/include/prox_mpc/models/unicycle.hpp`）：

```cpp
setN(3);  // state: [x, y, theta]
setM(2);  // control: [v, omega]          ← **控制量是速度**
setIneq("u", 0, -3.0, 3.0);   // 速度上下限
setIneq("du", 0, -.5, .5);    // 加速度 = 控制的"变化率"约束
// toTwist(): linear.x = v, angular.z = omega
```

我最初的实现是**加速度控制 + 积分成速度**（那是给动力学底盘写的），
对"速度跟随型"底盘（Classic 运动学底盘、真车麦轮）就是错位：
输出被 `amax·dt` 卡住、还依赖不可靠的 `/odom` twist。

**改为**：控制 = `[vx, vy]`，加速度作为 `|v_k − v_{k−1}| ≤ amax·dt`，输出 `v_0` 直接下发。

| 版本 | cmd_vel | 16 s 移动 | 等效速度 | 距目标 |
|---|---|---|---|---|
| 加速度空间 | 0.13~0.22 | 1.52 m | 0.10 m/s | 7.0 m |
| 加速度空间 + 预测速度直发 | 0.40 | 4.15 m | 0.28 m/s | 4.3 m |
| **速度空间（当前）** | **0.60 / −0.34** | **5.54 m** | **0.35 m/s** | **2.9 m** |
| MPPI（对照） | ~1.0 | 8.14 m | 0.51 m/s | 0.44 m |

**期间修掉的 QP bug**：`solveVelocity` 的 `P` 原先按 triplet 拼装，
既有**重复对角项**又有**下三角项**（OSQP 只接受上三角）⇒ 解恒为 0（cmp_vel 全 0）。
改成"稠密矩阵组装 + 只取上三角"后恢复正常。

**仍待修**：`near_idx` 投影仍未推进（debug 里恒 0），参考点永远在路径起点附近 ~0.2 m 处。

### 投影不是 bug + 前视改成按弧长（2026-11-06 最终）

**先纠正我自己的判断**：`near_idx` 恒 0 **不是 bug**。
加了"路径首点"日志后实测：

```
车 pose = (-0.01, 0.00)
setPlan: 157 点, 长度 8.92 m, 首点=(-0.02, -0.02)   ← 路径就是从车开始
setPlan: 153 点, 长度 8.66 m, 首点=( 0.18,  0.13)   ← 车动，首点跟着动
setPlan: 149 点, 长度 8.37 m, 首点=( 0.38,  0.33)
```

nav2 每次重规划都把路径起点放在车当前位置 ⇒ 最近点当然一直是 0。**这是正常行为。**

**真正的两个问题是**：
1. 控制量选成了加速度（已改成速度空间）；
2. 航向误差用**固定 6 个路径点**当前视 —— 点间距随规划器变化 ⇒ 前视距离时大时小 ⇒ 航向抖 ⇒ 车画龙。
   改成**按弧长 0.8 m** 取前视点。

**最终实测（Classic，仿真 12 s）**：

| 指标 | rm_mpc | MPPI（对照） |
|---|---|---|
| 累计行程 | **6.64 m** | 8.14 m（16 s） |
| 行程速度 | **0.55 m/s** | 0.51 m/s |
| 直线度（位移/行程） | 0.92 | — |
| 距目标 | 2.09 m（还在走） | 0.44 m |

⇒ **速度已与 MPPI 同档（0.55 vs 0.51 m/s）**，剩下的差距主要是"还没到点"（12 s vs 16 s 窗口）。

## MinimumSnap 实测定论（2026-11-06，离线）

之前"故意没测"的理由不充分，现在补测了。

**构建**：`OsqpEigen` 用源码本地编（不需要 sudo），OSQP 用 `tools/mpc_proto/.deps` 里的 vendor 版。

**实测**（30×20 m 栅格、两道墙、A* 481 点/29.18 m）：

```
MinimumSnap: success=1  点=4801  走廊=481  段=0  总时长=0.00 s  求解=62.14 ms  迭代=1
起点 (2.00, 2.00) → 终点 (26.00, 10.00)
```

**结论（关键）**：

| 能力 | 有吗 |
|---|---|
| 生成**走廊约束下的平滑路径** | ✅（4801 点，穿 481 个 box） |
| 输出**逐点速度/加速度/时间** | ❌ `SolveOutput` 只有 `path`（位置）；`getTimeAllocated()` **为空**（除非自己传进去） |
| 输出**多项式系数**（可自行求导） | ❌ 不外露 |
| 求解耗时 | ⚠️ 62 ms/次（1 Hz 重规划可以，10 Hz 不行） |

⇒ **它本质是一个"带走廊约束的路径平滑器"，不是"轨迹生成器"（对我们而言）。**
对"速度空间 MPC"的价值 = 提供更平滑的参考路径（可选）；
**拿不到时间剖面**，所以无法替代我们自己的"按弧长 + v_ref"参考。

**什么时候值得用**：A* 路径太锯齿、需要保证走廊内平滑时。
**什么时候不该用**：想要时间参数化轨迹（应该用 `KinodynamicAstar`，它给 `Sample{t,x,y,vx,vy,ax,ay}`）。
