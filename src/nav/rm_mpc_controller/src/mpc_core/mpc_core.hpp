// mpc_core —— 走廊约束 MPC 的核心（Eigen + OSQP，无 ROS 依赖）
// 模型（世界系速度，4 状态）： x' = x + vx*dt, y' = y + vy*dt
//                              vx' = vx + ax*dt, vy' = vy + ay*dt
#pragma once
#include <Eigen/Dense>
#include <vector>

namespace rm_mpc {

struct RefStage { double x = 0, y = 0, vx = 0, vy = 0; };

// 与 tdt-nav-kit 的 SfcSquare::PointPair 同构：(xmin, ymin, xmax, ymax)
struct Box { double xmin = -1e9, ymin = -1e9, xmax = 1e9, ymax = 1e9; };

struct Params {
  double dt = 0.05;
  int N = 20;
  double vmax = 2.0;
  double amax = 4.0;
  double w_pos = 80.0, w_vel = 20.0, w_u = 4.0;
  double w_terminal = 8.0;
  double w_slack = 2000.0;   // 走廊软约束惩罚（越大越"硬"）
  bool use_corridor = true;
};

struct Solution {
  bool ok = false;
  const char* status = "";
  double ms = 0.0;
  int iters = 0;
  std::vector<double> ax, ay;   // 长度 N
  std::vector<double> px, py;   // 长度 N（预测状态）
  double vx0 = 0.0, vy0 = 0.0;  // 第 0 段预测速度（世界系）——直接作为速度指令
  double max_slack = 0.0;       // 本次求解用掉的最大走廊松弛量（米）
};

// 每个 stage 一个参考 + 一个走廊 box（长度必须 = p.N；box 可为无限大）
Solution solve(const Params& p, const Eigen::Vector4d& s0,   // [x, y, vx, vy]
               const std::vector<RefStage>& ref, const std::vector<Box>& box);

// ============================================================================
// **速度空间** MPC（对齐 prox_mpc 的公式化：控制量就是速度，加速度是"控制变化率"）
//   state   [x, y]            控制 [vx, vy]
//   动态    x_k = x_{k-1} + vx_k·dt
//   约束    |v| ≤ vmax；|v_k − v_{k-1}| ≤ amax·dt；走廊 box（软）
//   输出    v_0 **直接就是下发给底盘的速度**（不再需要 v_now 状态）
// ============================================================================
struct VelParams {
  double dt = 0.05;
  int N = 20;
  double vmax = 2.0;
  double amax = 4.0;        // 作为 |dv|/dt 的上限
  double w_pos = 80.0, w_vel = 20.0, w_du = 4.0, w_slack = 2000.0;
  bool use_corridor = true;
};

struct VelSolution {
  bool ok = false;
  const char* status = "";
  double ms = 0.0;
  int iters = 0;
  std::vector<double> vx, vy, px, py;   // 长度 N
  double vx0 = 0.0, vy0 = 0.0;          // 世界系第 0 段速度（= 指令）
  double max_slack = 0.0;
};

VelSolution solveVelocity(const VelParams& p, const Eigen::Vector2d& p0,
                          const Eigen::Vector2d& v_prev,
                          const std::vector<RefStage>& ref,
                          const std::vector<Box>& box);

}  // namespace rm_mpc
