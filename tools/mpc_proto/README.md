# 离线 MPC 原型（走廊约束 + 参考轨迹）

**目的**：验证"用 T-DT 规划结果（带时间轨迹 + 走廊）驱动 MPC"是否可行，并给出可量化指标。
**不依赖 ROS / Gazebo**，纯 C++17 + Eigen + OSQP。

## 构建（不需要 sudo）

```bash
./build.sh          # 首次会自动 apt-get download ros-humble-osqp-vendor 并解到 .deps/
```

## 运行

```bash
./mpc_proto straight        # 直道
./mpc_proto narrow          # 中段窄通道（半宽 0.35 m）
./mpc_proto turn            # 90° 弯
./mpc_proto tight           # 极窄走廊（半宽 0.06 m）
MPC_DISTURB=1 ./mpc_proto tight   # 加横向扰动（模拟打滑/推挤）
```

## 模型与设计

- 状态 `[x, y, θ, vx, vy, ω]`（世界系速度），控制 `[ax, ay, α]`，**线性化后的双积分器**；
- 代价：位置/姿态/速度跟踪 + 控制量，终端位置权重 ×8；
- 约束：动力学等式 + 控制上下限（|a|≤4, |α|≤12） + 速度上下限（|v|≤2, |ω|≤6）
  + **每个 stage 的走廊 box**（`xmin/xmax/ymin/ymax`）；
- 求解：**OSQP**（稀疏 QP，N=20，n=180，m=320）。

## 实测结果（本机，240 步 @ 20 Hz）

| 场景 | RMSE (x / y) | max\|ey\| | 走廊违背 | 求解均值 / 最大 | 迭代 |
|---|---|---|---|---|---|
| straight | 0.030 / 0.000 m | 0.000 | 0 | **0.13 / 0.21 ms** | 49 |
| narrow (0.35) | 0.030 / 0.000 | 0.000 | 0 | 0.14 / 0.18 ms | 49 |
| turn (90°) | 0.041 / 0.031 | 0.105 | 0 | 0.14 / 0.26 ms | 49 |
| tight (0.06) | 0.030 / 0.000 | 0.000 | 0 | 0.18 / 0.19 ms | 49 |
| **tight + 扰动** | 0.030 / 0.023 | **0.065** | 0.005 m | 0.35 / 13.3 ms | 92 |
| *对照*：无走廊 + 扰动 | 0.030 / 0.037 | **0.106** | — | — | — |

**结论**：
1. QP 在 **0.15 ms 量级**求解（远低于 20 Hz 周期）；
2. **走廊约束确实起效**：同样扰动下，有走廊时横向误差被压到 0.065 m（无走廊 0.106 m）；
3. 极窄走廊 + 扰动时迭代数升到 92、单次最坏 13 ms ⇒ **重规划频率要留余量**（或在硬场景降 N）。

## 下一步（与 tdt-nav-kit 对接）

1. 用 `KinodynamicAstar::Result::trajectory`（`Sample{t, x, y, vx, vy, ax, ay}`）替换这里的 `ref_pose()`；
2. 用 `SfcSquare`/`MinimumSnap::SolveOutput::corridor`（`PointPair{xmin,ymin,xmax,ymax}`）替换这里的解析走廊；
3. 控制量从世界系加速度转到**车体系**（`ax_body = cosθ·ax + sinθ·ay`，`ay_body = -sinθ·ax + cosθ·ay`）；
4. 再接 ROS 节点 / nav2 `Controller` 插件（这步才需要仿真）。
