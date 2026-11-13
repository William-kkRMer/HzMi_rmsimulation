// =============================================================================
// mpc.cpp —— 离线 MPC 原型（走廊约束 + 参考轨迹），不依赖 ROS / Gazebo
//
// 目的：验证“用 T-DT 的规划结果（带时间轨迹 + 走廊）驱动 MPC”是否可行，
//       并给出可量化的指标：跟踪误差、走廊违背、求解耗时。
//
// 模型（世界系速度，线性）：  x' = x + vx*dt,  vx' = vx + ax*dt
//                             y' = y + vy*dt,  vy' = vy + ay*dt
//                             θ' = θ + ω*dt,   ω'  = ω  + α*dt
// 决策变量 z = [s_1..s_N (6N), u_0..u_{N-1} (3N)]
// 约束：动力学等式 + 控制上下限 + 速度上下限 + 每个 stage 的走廊 box
// 求解：OSQP
// =============================================================================
#include <osqp/osqp.h>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {
constexpr int NX = 6;   // x, y, th, vx, vy, w
constexpr int NU = 3;   // ax, ay, alpha

struct Params {
  double dt = 0.05;        // 控制周期
  int N = 20;              // 预测步数
  double vmax = 2.0, wmax = 6.0;
  double amax = 4.0, alpha_max = 12.0;
  double w_pos = 80.0, w_vel = 1.0, w_th = 1.0, w_w = 1.0;
  double w_u_lin = 4.0, w_u_ang = 1.0;
  double w_terminal = 8.0;  // 终端位置权重倍数
  bool use_corridor = true;
};

struct Ref {  // 每个 stage 的参考与走廊
  Eigen::VectorXd rx, ry, rth, rvx, rvy;
  Eigen::VectorXd xmin, xmax, ymin, ymax;
};

struct Scenario {
  std::string name;
  double length = 100.0;   // 测试窗口内不饱和（只测跟踪，不测停车）
  double corr_y = 0.8;     // 走廊半宽
  double narrow_from = 1e9, narrow_to = -1e9, narrow_y = 0.45;
  bool turn = false;
};


void ref_pose(const Scenario& sc, double t, double* px, double* py, double* pth) {
  const double vmax = 1.5, a = 2.0;
  const double ta = vmax / a;                       // 加速到 1.5 m/s 的时间
  auto dist = [&](double tt) {                      // 梯形速度下的行驶距离
    if (tt <= 0) return 0.0;
    if (tt < ta) return 0.5 * a * tt * tt;
    return 0.5 * a * ta * ta + vmax * (tt - ta);
  };
  const double s = dist(t);
  if (!sc.turn) {
    *px = std::min(s, sc.length); *py = 0.0; *pth = 0.0;
    return;
  }
  const double d1 = 4.0;                            // 直道 4 m
  const double d2 = M_PI_2 * 1.0;                   // 90° 弧，R=1
  if (s < d1) { *px = s; *py = 0.0; *pth = 0.0; }
  else if (s < d1 + d2) { const double ang = (s - d1); *px = 4.0 + std::sin(ang); *py = 1.0 - std::cos(ang); *pth = ang; }
  else { *px = 5.0; *py = 1.0 + (s - d1 - d2); *pth = M_PI_2; }
}

Ref make_ref(const Scenario& sc, const Params& p, double t0) {
  Ref r;
  r.rx.resize(p.N); r.ry.resize(p.N); r.rth.resize(p.N);
  r.rvx.resize(p.N); r.rvy.resize(p.N);
  r.xmin.resize(p.N); r.xmax.resize(p.N); r.ymin.resize(p.N); r.ymax.resize(p.N);
  for (int k = 0; k < p.N; ++k) {
    const double t = t0 + (k + 1) * p.dt;
    double x, y, th;
    ref_pose(sc, t, &x, &y, &th);
    r.rx[k] = x; r.ry[k] = y; r.rth[k] = th;
    double half = sc.corr_y;
    if (x > sc.narrow_from && x < sc.narrow_to) half = sc.narrow_y;
    r.xmin[k] = x - 1.0; r.xmax[k] = x + 1.0;
    r.ymin[k] = y - half; r.ymax[k] = y + half;
  }
  // 速度参考：对位置参考做差分（终端用最后一段）
  for (int k = 0; k < p.N; ++k) {
    const int a = k, b = std::min(k + 1, p.N - 1);
    const double d = (b > a) ? (b - a) * p.dt : p.dt;
    r.rvx[k] = (r.rx[b] - r.rx[a]) / d;
    r.rvy[k] = (r.ry[b] - r.ry[a]) / d;
  }
  return r;
}

struct Triplet { int r, c; double v; };

void triplets_to_csc(int n, int m, const std::vector<Triplet>& trips,
                     std::vector<c_int>* p, std::vector<c_int>* i, std::vector<c_float>* x) {
  (void)m;
  p->assign(n + 1, 0);
  for (const auto& t : trips) p->at(t.c + 1)++;
  for (int c = 0; c < n; ++c) p->at(c + 1) += p->at(c);
  i->resize(trips.size()); x->resize(trips.size());
  std::vector<c_int> cur = *p;
  for (const auto& t : trips) {
    const int idx = cur[t.c]++;
    i->at(idx) = (c_int)t.r; x->at(idx) = (c_float)t.v;
  }
}

struct SolveResult {
  bool ok = false;
  std::string status;
  double ms = 0.0;
  int iters = 0;
  Eigen::MatrixXd states;   // N x 6
  Eigen::MatrixXd controls; // N x 3
};

SolveResult solve_mpc(const Params& p, const Eigen::VectorXd& s0, const Ref& ref) {
  const int n = (NX + NU) * p.N;
  const int m_eq = NX * p.N, m_ub = NU * p.N, m_vb = 3 * p.N, m_corr = 4 * p.N;
  const int m = m_eq + m_ub + m_vb + m_corr;

  std::vector<Triplet> P, A;
  std::vector<c_float> q(n, 0.0f), l(m, -OSQP_INFTY), u(m, OSQP_INFTY);

  auto sidx = [&](int k, int c) { return k * NX + c; };          // state k (1..N) -> index
  auto uidx = [&](int k, int c) { return NX * p.N + k * NU + c; }; // control k (0..N-1)

  // ---- 代价 ----
  for (int k = 0; k < p.N; ++k) {
    const double wp = (k == p.N - 1) ? p.w_pos * p.w_terminal : p.w_pos;
    P.push_back({sidx(k, 0), sidx(k, 0), wp});
    P.push_back({sidx(k, 1), sidx(k, 1), p.w_pos});
    P.push_back({sidx(k, 2), sidx(k, 2), p.w_th});
    P.push_back({sidx(k, 3), sidx(k, 3), p.w_vel});
    P.push_back({sidx(k, 4), sidx(k, 4), p.w_vel});
    q[sidx(k, 3)] = (c_float)(-p.w_vel * ref.rvx[k]);
    q[sidx(k, 4)] = (c_float)(-p.w_vel * ref.rvy[k]);
    P.push_back({sidx(k, 5), sidx(k, 5), p.w_w});
    q[sidx(k, 0)] = (c_float)(-wp * ref.rx[k]);
    q[sidx(k, 1)] = (c_float)(-p.w_pos * ref.ry[k]);
    q[sidx(k, 2)] = (c_float)(-p.w_th * ref.rth[k]);
    P.push_back({uidx(k, 0), uidx(k, 0), p.w_u_lin});
    P.push_back({uidx(k, 1), uidx(k, 1), p.w_u_lin});
    P.push_back({uidx(k, 2), uidx(k, 2), p.w_u_ang});
  }

  // ---- 动力学等式（每个 stage 的 NX 行）----
  int row = 0;
  for (int k = 0; k < p.N; ++k) {
    for (int c = 0; c < NX; ++c) {
      const int r = row + c;
      // s_{k+1}[c] - s_k[c] - (c>=3 ? u_k[c-3]*dt : s_k[c+3]*dt) = 0
      A.push_back({r, sidx(k, c), 1.0});
      if (k > 0) A.push_back({r, sidx(k - 1, c), -1.0});
      if (c < 3) A.push_back({r, sidx(k, c + 3), -p.dt});
      else A.push_back({r, uidx(k, c - 3), -p.dt});
      double rhs = 0.0;
      if (k == 0) rhs = s0[c];  // s_1 - s_0 - ... = 0
      l[r] = (c_float)rhs; u[r] = (c_float)rhs;
    }
    row += NX;
  }

  // ---- 控制上下限 ----
  for (int k = 0; k < p.N; ++k) {
    const double lo[NU] = {-p.amax, -p.amax, -p.alpha_max};
    const double hi[NU] = {p.amax, p.amax, p.alpha_max};
    for (int c = 0; c < NU; ++c) {
      A.push_back({row, uidx(k, c), 1.0});
      l[row] = (c_float)lo[c]; u[row] = (c_float)hi[c];
      ++row;
    }
  }
  // ---- 速度上下限 ----
  for (int k = 0; k < p.N; ++k) {
    const double lo[3] = {-p.vmax, -p.vmax, -p.wmax};
    const double hi[3] = {p.vmax, p.vmax, p.wmax};
    for (int c = 0; c < 3; ++c) {
      A.push_back({row, sidx(k, 3 + c), 1.0});
      l[row] = (c_float)lo[c]; u[row] = (c_float)hi[c];
      ++row;
    }
  }
  // ---- 走廊 box 约束 ----
  for (int k = 0; k < p.N; ++k) {
    A.push_back({row, sidx(k, 0), 1.0}); l[row] = (c_float)ref.xmin[k]; u[row] = (c_float)ref.xmax[k]; ++row;
    A.push_back({row, sidx(k, 1), 1.0}); l[row] = (c_float)ref.ymin[k]; u[row] = (c_float)ref.ymax[k]; ++row;
    // 空白占位：为保持 m 行数一致（不做额外约束）——直接给很宽范围
    A.push_back({row, sidx(k, 2), 1.0}); l[row] = -1e3; u[row] = 1e3; ++row;
    A.push_back({row, sidx(k, 5), 1.0}); l[row] = -1e3; u[row] = 1e3; ++row;
  }

  std::vector<c_int> Pp, Pi, Ap, Ai; std::vector<c_float> Px, Ax;
  triplets_to_csc(n, n, P, &Pp, &Pi, &Px);
  triplets_to_csc(n, m, A, &Ap, &Ai, &Ax);

  csc Pc{}, Ac{}, qc{};
  Pc.nzmax = (int)Px.size(); Pc.m = n; Pc.n = n; Pc.p = Pp.data(); Pc.i = Pi.data(); Pc.x = Px.data(); Pc.nz = -1;
  Ac.nzmax = (int)Ax.size(); Ac.m = m; Ac.n = n; Ac.p = Ap.data(); Ac.i = Ai.data(); Ac.x = Ax.data(); Ac.nz = -1;
  qc.nzmax = n; qc.m = n; qc.n = 1; qc.p = nullptr; qc.i = nullptr; qc.x = q.data(); qc.nz = -1;

  OSQPData data{}; data.n = n; data.m = m; data.P = &Pc; data.A = &Ac; data.q = q.data(); data.l = l.data(); data.u = u.data();
  OSQPSettings settings; osqp_set_default_settings(&settings);
  settings.verbose = 0; settings.eps_abs = 1e-4; settings.eps_rel = 1e-4; settings.max_iter = 4000;

  OSQPWorkspace* work = nullptr;
  SolveResult out;
  if (osqp_setup(&work, &data, &settings) != 0) { out.status = "setup_failed"; return out; }
  const auto t0 = std::chrono::steady_clock::now();
  osqp_solve(work);
  const auto t1 = std::chrono::steady_clock::now();
  out.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  out.status = work->info->status;
  out.iters = (int)work->info->iter;
  if (work->info->status_val == OSQP_SOLVED || work->info->status_val == OSQP_SOLVED_INACCURATE) {
    out.ok = true;
    out.states.resize(p.N, NX); out.controls.resize(p.N, NU);
    for (int k = 0; k < p.N; ++k) {
      for (int c = 0; c < NX; ++c) out.states(k, c) = (double)work->solution->x[sidx(k, c)];
      for (int c = 0; c < NU; ++c) out.controls(k, c) = (double)work->solution->x[uidx(k, c)];
    }
  }
  osqp_cleanup(work);
  return out;
}
}  // namespace

int main(int argc, char** argv) {
  Params p;
  Scenario sc;
  sc.name = argc > 1 ? argv[1] : "straight";
  if (sc.name == "narrow") { sc.narrow_from = 3.0; sc.narrow_to = 7.0; sc.narrow_y = 0.35; }
  if (sc.name == "tight")  { sc.narrow_from = -1e9; sc.narrow_to = 1e9; sc.narrow_y = 0.06; sc.corr_y = 0.06; }
  if (sc.name == "turn")   { sc.turn = true; sc.corr_y = 1.5; sc.length = 6.0; }


  Eigen::VectorXd s(NX); s << 0, 0, 0, 0, 0, 0;
  double sum_iter = 0.0;
  double sum_sq = 0.0, sum_sq_x = 0.0, sum_sq_y = 0.0, max_abs_y = 0.0, max_viol = 0.0, sum_ms = 0.0, max_ms = 0.0;
  const bool disturb = std::getenv("MPC_DISTURB") != nullptr;
  int steps = 0, fails = 0;
  const int total = 240;  // 12 s @ 20 Hz
  for (int step = 0; step < total; ++step) {
    Ref ref = make_ref(sc, p, step * p.dt);   // ★ 参考随当前时间平移
    SolveResult r = solve_mpc(p, s, ref);
    if (!r.ok) { ++fails; s(3) = s(4) = s(5) = 0.0; continue; }
    sum_ms += r.ms; max_ms = std::max(max_ms, r.ms); sum_iter += r.iters;
    // 施加第一步控制，按模型推进一步（含小幅扰动，模拟打滑/外推误差）
    const double ax = r.controls(0, 0), ay = r.controls(0, 1), al = r.controls(0, 2);
    s(0) += s(3) * p.dt; s(1) += s(4) * p.dt; s(2) += s(5) * p.dt;
    s(3) += ax * p.dt; s(4) += ay * p.dt; s(5) += al * p.dt;
    if (disturb && step > 20 && step < 60) s(4) += 0.02;   // 短暂横向扰动（MPC_DISTURB=1 时启用）
    // 参考（与状态同一时刻）
    double rx, ry, rth;
    ref_pose(sc, (step + 1) * p.dt, &rx, &ry, &rth);
    const double ex = s(0) - rx, ey = s(1) - ry;
    sum_sq += ex * ex + ey * ey; sum_sq_x += ex * ex; sum_sq_y += ey * ey;
    max_abs_y = std::max(max_abs_y, std::fabs(ey));
    // 走廊违背（用同一 half 规则）
    double half = sc.corr_y;
    if (s(0) > sc.narrow_from && s(0) < sc.narrow_to) half = sc.narrow_y;
    max_viol = std::max(max_viol, std::max(0.0, std::fabs(s(1) - ry) - half));
    ++steps;
  }
  const double n = std::max(1, steps);
  std::printf("[%s%s] steps=%d fails=%d  RMSE=%.4f (x %.4f, y %.4f) m  max|ey|=%.4f  走廊违背=%.4f m  求解 均值 %.2f / 最大 %.2f ms  迭代均值 %.0f\n",
              sc.name.c_str(), disturb ? "+disturb" : "", steps, fails,
              std::sqrt(sum_sq / n), std::sqrt(sum_sq_x / n), std::sqrt(sum_sq_y / n), max_abs_y, max_viol,
              sum_ms / n, max_ms, sum_iter / n);
  return 0;
}
