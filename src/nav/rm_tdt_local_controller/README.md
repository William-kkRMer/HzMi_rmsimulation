# rm_tdt_local_controller — T-DT "局部规划即控制器"

## 架构（与 T-DT `doc/Astar.md` 的"混合搜索"一致）

```
全局：rm_tdt_planner（YAstar A*，整场 → Path + 走廊）
   │  投影当前位姿到 Path，取前方 lookahead(默认 4 m) 处作**局部目标**
   ▼
局部：KinodynamicAstar（Kastar，状态 [x,y,vx,vy]）
       在**局部 costmap**（lethal→二值，外扩 2 格边框）上搜索
       → 输出带时间轨迹 Sample{t, x, y, vx, vy, ax, ay}
   ▼
执行：按轨迹**前馈**（取最近样本 +2 的速度）+ 航向反馈
      失败 ⇒ 回退 pure-pursuit（沿 Path 前视点）
```

## 参数（`nav2_params_sim_controller_rm_tdt_local.yaml`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `lookahead` | 4.0 | 局部目标距离（他们文档建议 4 m） |
| `max_vel` / `max_acc` | 1.5 / 3.0 | Kastar 约束 |
| `max_w` / `k_theta` | 3.0 / 2.0 | 角速度上限与航向增益 |
| `max_nodes` / `sample_time` | 20000 / 0.02 | Kastar 搜索预算 |
| `use_kastar` | true | 关掉即退化为 pure-pursuit |

## 用法

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py ... nav:=rm_tdt_local planner:=tdt
```

## 实测（Classic，2026-11-06）

```
Created controller : FollowPath of type rm_tdt_local_controller/TdtLocalController
[tdt-local] configured: lookahead=4.0 vmax=1.50 amax=3.00 kastar=1
kastar ok 9 次 / 30 s，cmd_vel ≈ (0.54, 0.60) m/s
```
