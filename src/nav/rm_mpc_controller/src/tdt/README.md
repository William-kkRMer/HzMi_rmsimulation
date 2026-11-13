# tdt/ —— 第三方规划内核（vendored，MIT）

来源：**T-DT-Algorithm-2026/tdt-nav-kit**（MIT，Copyright (c) 2026 Rongxuan Ye and BoLin Li）
https://github.com/T-DT-Algorithm-2026/tdt-nav-kit

本目录**只放原样拷贝**（未改动），用途：给 `rm_mpc_controller` 提供
「路径搜索 + 时间参数化轨迹 + 安全走廊（SFC）」。

| 文件 | 作用 | 依赖 |
|---|---|---|
| `yastar.{hpp,cpp}` | A* / 栅格搜索（含障碍距离势场、Mask 层、路径简化/加密） | Eigen，**C++20**（`std::span`） |
| `kinodynamicAstar.{hpp,cpp}` | 带动力学约束的搜索，输出 `Sample{t, state[x,y,vx,vy], acceleration}` | Eigen，C++20 |
| `sfcSquare.{hpp,cpp}` | 沿路径生成凸走廊（`SfcSquare::Map = Eigen::Matrix<unsigned char,...,RowMajor>`） | Eigen，C++20（`<ranges>`） |

## 为什么不整仓引入

- `MinimumSnapOsqp/`（min-snap）需要 **OSQP + OsqpEigen**，且它的输出是**位置点 + 分段时长**
  （`SolveOutput` 无逐点速度）；我们用它下游的 MPC 已经自带平滑，暂不需要；
- `main.cpp` 是 OpenCV demo，与我们无关。

## 版权与改动约定

- `LICENSE.tdt-nav-kit` 为上游 MIT 原文，**保留**；
- 本目录**保持原样**；需要改动请复制到 `src/nav/rm_mpc_controller/src/` 下的自有文件里改，
  这样 `diff` 能看出我们改了什么（与 `third_party/` 的"只读对照"原则一致）。
