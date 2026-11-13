#include "mpc_core.hpp"

#include <osqp/osqp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace rm_mpc {
namespace {
constexpr int NX = 4;  // x, y, vx, vy
constexpr int NU = 2;  // ax, ay

struct Triplet { int r, c; double v; };

void to_csc(int n, const std::vector<Triplet>& t, std::vector<c_int>* p,
            std::vector<c_int>* i, std::vector<c_float>* x) {
  p->assign(n + 1, 0);
  for (const auto& e : t) p->at(e.c + 1)++;
  for (int c = 0; c < n; ++c) p->at(c + 1) += p->at(c);
  i->resize(t.size());
  x->resize(t.size());
  std::vector<c_int> cur = *p;
  for (const auto& e : t) {
    const int k = cur[e.c]++;
    i->at(k) = (c_int)e.r;
    x->at(k) = (c_float)e.v;
  }
}
}  // namespace

Solution solve(const Params& p, const Eigen::Vector4d& s0,
               const std::vector<RefStage>& ref, const std::vector<Box>& box) {
  Solution out;
  constexpr int NS = 4;  // 每个 stage 的松弛量：x_min, x_max, y_min, y_max
  const int n = (NX + NU + NS) * p.N;
  const int m_eq = NX * p.N, m_ctrl = NU * p.N, m_vel = 2 * p.N;
  const int m_box = 4 * p.N, m_slack = 4 * p.N;
  const int m = m_eq + m_ctrl + m_vel + m_box + m_slack;

  std::vector<Triplet> P, A;
  std::vector<c_float> q(n, 0.0f), l(m, -OSQP_INFTY), u(m, OSQP_INFTY);
  auto si = [&](int k, int c) { return k * NX + c; };
  auto ui = [&](int k, int c) { return NX * p.N + k * NU + c; };
  auto sl = [&](int k, int c) { return (NX + NU) * p.N + k * NS + c; };  // 0=xlo 1=xhi 2=ylo 3=yhi

  for (int k = 0; k < p.N; ++k) {
    const double wp = (k == p.N - 1) ? p.w_pos * p.w_terminal : p.w_pos;
    P.push_back({si(k, 0), si(k, 0), wp});
    P.push_back({si(k, 1), si(k, 1), p.w_pos});
    P.push_back({si(k, 2), si(k, 2), p.w_vel});
    P.push_back({si(k, 3), si(k, 3), p.w_vel});
    q[si(k, 0)] = (c_float)(-wp * ref[k].x);
    q[si(k, 1)] = (c_float)(-p.w_pos * ref[k].y);
    q[si(k, 2)] = (c_float)(-p.w_vel * ref[k].vx);
    q[si(k, 3)] = (c_float)(-p.w_vel * ref[k].vy);
    P.push_back({ui(k, 0), ui(k, 0), p.w_u});
    P.push_back({ui(k, 1), ui(k, 1), p.w_u});
    for (int c = 0; c < NS; ++c) P.push_back({sl(k, c), sl(k, c), 2.0 * p.w_slack});
  }

  int row = 0;
  for (int k = 0; k < p.N; ++k) {
    for (int c = 0; c < NX; ++c) {
      const int r = row + c;
      A.push_back({r, si(k, c), 1.0});
      if (k > 0) A.push_back({r, si(k - 1, c), -1.0});
      if (c < 2) A.push_back({r, si(k, c + 2), -p.dt});
      else A.push_back({r, ui(k, c - 2), -p.dt});
      const double rhs = (k == 0) ? s0[c] : 0.0;
      l[r] = (c_float)rhs;
      u[r] = (c_float)rhs;
    }
    row += NX;
  }
  for (int k = 0; k < p.N; ++k) {
    for (int c = 0; c < NU; ++c) {
      A.push_back({row, ui(k, c), 1.0});
      l[row] = (c_float)(-p.amax);
      u[row] = (c_float)(p.amax);
      ++row;
    }
  }
  for (int k = 0; k < p.N; ++k) {
    for (int c = 0; c < 2; ++c) {
      A.push_back({row, si(k, 2 + c), 1.0});
      l[row] = (c_float)(-p.vmax);
      u[row] = (c_float)(p.vmax);
      ++row;
    }
  }
  for (int k = 0; k < p.N; ++k) {
    const Box& b = p.use_corridor ? box[k] : Box{};
    // x ≤ xmax + s_hi  ⇒  x - s_hi ≤ xmax
    A.push_back({row, si(k, 0), 1.0}); A.push_back({row, sl(k, 1), -1.0});
    l[row] = -OSQP_INFTY; u[row] = (c_float)b.xmax; ++row;
    // x ≥ xmin - s_lo  ⇒  -x - s_lo ≤ -xmin
    A.push_back({row, si(k, 0), -1.0}); A.push_back({row, sl(k, 0), -1.0});
    l[row] = -OSQP_INFTY; u[row] = (c_float)(-b.xmin); ++row;
    // y ≤ ymax + s_hi
    A.push_back({row, si(k, 1), 1.0}); A.push_back({row, sl(k, 3), -1.0});
    l[row] = -OSQP_INFTY; u[row] = (c_float)b.ymax; ++row;
    // y ≥ ymin - s_lo
    A.push_back({row, si(k, 1), -1.0}); A.push_back({row, sl(k, 2), -1.0});
    l[row] = -OSQP_INFTY; u[row] = (c_float)(-b.ymin); ++row;
  }
  // 松弛量 ≥ 0（否则可以用负松弛"作弊"）
  for (int k = 0; k < p.N; ++k)
    for (int c = 0; c < NS; ++c) {
      A.push_back({row, sl(k, c), 1.0});
      l[row] = 0.0; u[row] = OSQP_INFTY; ++row;
    }

  std::vector<c_int> Pp, Pi, Ap, Ai;
  std::vector<c_float> Px, Ax;
  to_csc(n, P, &Pp, &Pi, &Px);
  to_csc(n, A, &Ap, &Ai, &Ax);

  csc Pc{}, Ac{};
  Pc.nzmax = (c_int)Px.size(); Pc.m = n; Pc.n = n; Pc.p = Pp.data(); Pc.i = Pi.data(); Pc.x = Px.data(); Pc.nz = -1;
  Ac.nzmax = (c_int)Ax.size(); Ac.m = m; Ac.n = n; Ac.p = Ap.data(); Ac.i = Ai.data(); Ac.x = Ax.data(); Ac.nz = -1;

  OSQPData data{};
  data.n = n; data.m = m; data.P = &Pc; data.A = &Ac; data.q = q.data(); data.l = l.data(); data.u = u.data();
  OSQPSettings st; osqp_set_default_settings(&st);
  st.verbose = 0; st.eps_abs = 1e-4; st.eps_rel = 1e-4; st.max_iter = 4000;

  OSQPWorkspace* w = nullptr;
  if (osqp_setup(&w, &data, &st) != 0) { out.status = "setup_failed"; return out; }
  const auto t0 = std::chrono::steady_clock::now();
  osqp_solve(w);
  out.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  out.status = w->info->status;
  out.iters = (int)w->info->iter;
  if (w->info->status_val == OSQP_SOLVED || w->info->status_val == OSQP_SOLVED_INACCURATE) {
    out.ok = true;
    out.ax.resize(p.N); out.ay.resize(p.N); out.px.resize(p.N); out.py.resize(p.N);
    for (int k = 0; k < p.N; ++k) {
      out.ax[k] = (double)w->solution->x[ui(k, 0)];
      out.ay[k] = (double)w->solution->x[ui(k, 1)];
      out.px[k] = (double)w->solution->x[si(k, 0)];
      out.py[k] = (double)w->solution->x[si(k, 1)];
      if (k == 0) {
        out.vx0 = (double)w->solution->x[si(k, 2)];
        out.vy0 = (double)w->solution->x[si(k, 3)];
      }
      for (int c = 0; c < NS; ++c)
        out.max_slack = std::max(out.max_slack, (double)w->solution->x[sl(k, c)]);
    }
  }
  osqp_cleanup(w);
  return out;
}


// ============================================================================
// 速度空间 MPC（控制量 = 速度；加速是 |Δv|/dt 约束）
// ============================================================================
namespace {
struct VelIdx {
  int N;
  int px(int k) const { return k; }
  int py(int k) const { return N + k; }
  int vx(int k) const { return 2 * N + k; }
  int vy(int k) const { return 3 * N + k; }
  int sl(int k, int c) const { return 4 * N + k * 4 + c; }
  int n() const { return 8 * N; }
};
}  // namespace

VelSolution solveVelocity(const VelParams& p, const Eigen::Vector2d& p0,
                          const Eigen::Vector2d& v_prev,
                          const std::vector<RefStage>& ref,
                          const std::vector<Box>& box) {
  VelSolution out;
  VelIdx I{p.N};
  const int nv = I.n();
  const int m_eq = 2 * p.N, m_rate = 4 * p.N, m_vlim = 2 * p.N;
  const int m_box = 4 * p.N, m_slack = 4 * p.N;
  const int m = m_eq + m_rate + m_vlim + m_box + m_slack;

  std::vector<Triplet> A;
  std::vector<c_float> q(nv, 0.0f), l(m, -OSQP_INFTY), u(m, OSQP_INFTY);
  Eigen::MatrixXd Pd = Eigen::MatrixXd::Zero(nv, nv);   // ★ 稠密组装，最后只取上三角

  for (int k = 0; k < p.N; ++k) {
    const double wp = (k == p.N - 1) ? p.w_pos * 4.0 : p.w_pos;
    Pd(I.px(k), I.px(k)) += wp;
    Pd(I.py(k), I.py(k)) += wp;
    q[I.px(k)] = (c_float)(-wp * ref[k].x);
    q[I.py(k)] = (c_float)(-wp * ref[k].y);
    Pd(I.vx(k), I.vx(k)) += p.w_vel;
    Pd(I.vy(k), I.vy(k)) += p.w_vel;
    q[I.vx(k)] = (c_float)(-p.w_vel * ref[k].vx);
    q[I.vy(k)] = (c_float)(-p.w_vel * ref[k].vy);
    if (k > 0) {
      for (int c = 0; c < 2; ++c) {
        const int a = (c == 0) ? I.vx(k) : I.vy(k);
        const int b = (c == 0) ? I.vx(k - 1) : I.vy(k - 1);
        Pd(a, a) += 2.0 * p.w_du;
        Pd(b, b) += 2.0 * p.w_du;
        Pd(a, b) -= 2.0 * p.w_du;
        Pd(b, a) -= 2.0 * p.w_du;
      }
    }
    for (int c = 0; c < 4; ++c) Pd(I.sl(k, c), I.sl(k, c)) += 2.0 * p.w_slack;
  }
  q[I.vx(0)] += (c_float)(-2.0 * p.w_du * v_prev.x());
  q[I.vy(0)] += (c_float)(-2.0 * p.w_du * v_prev.y());

  std::vector<Triplet> P;
  for (int c = 0; c < nv; ++c)
    for (int r2 = 0; r2 <= c; ++r2)    // 上三角（row ≤ col）
      if (std::abs(Pd(r2, c)) > 1e-12) P.push_back({r2, c, Pd(r2, c)});

  int row = 0;
  // 动态：x_k - x_{k-1} - vx_k*dt = 0（x_0 = p0）
  for (int k = 0; k < p.N; ++k) {
    A.push_back({row, I.px(k), 1.0});
    A.push_back({row, I.vx(k), -p.dt});
    if (k > 0) A.push_back({row, I.px(k - 1), -1.0});
    const double rhs = (k == 0) ? p0.x() : 0.0;
    l[row] = (c_float)rhs; u[row] = (c_float)rhs; ++row;
    A.push_back({row, I.py(k), 1.0});
    A.push_back({row, I.vy(k), -p.dt});
    if (k > 0) A.push_back({row, I.py(k - 1), -1.0});
    const double rhs_y = (k == 0) ? p0.y() : 0.0;
    l[row] = (c_float)rhs_y; u[row] = (c_float)rhs_y; ++row;
  }
  // 速率：|v_k - v_{k-1}| ≤ amax*dt
  const double dv = p.amax * p.dt;
  for (int k = 0; k < p.N; ++k) {
    for (int c = 0; c < 2; ++c) {
      const int vi = (c == 0) ? I.vx(k) : I.vy(k);
      A.push_back({row, vi, 1.0});
      double lo = -dv, hi = dv;
      if (k > 0) A.push_back({row, (c == 0) ? I.vx(k - 1) : I.vy(k - 1), -1.0});
      else { lo += (c == 0 ? v_prev.x() : v_prev.y()); hi += (c == 0 ? v_prev.x() : v_prev.y()); }
      l[row] = (c_float)lo; u[row] = (c_float)hi; ++row;
    }
  }
  // 速度上限
  for (int k = 0; k < p.N; ++k) {
    A.push_back({row, I.vx(k), 1.0}); l[row] = (c_float)(-p.vmax); u[row] = (c_float)p.vmax; ++row;
    A.push_back({row, I.vy(k), 1.0}); l[row] = (c_float)(-p.vmax); u[row] = (c_float)p.vmax; ++row;
  }
  // 走廊（软）
  for (int k = 0; k < p.N; ++k) {
    const Box& b = p.use_corridor ? box[k] : Box{};
    A.push_back({row, I.px(k), 1.0}); A.push_back({row, I.sl(k, 1), -1.0});
    l[row] = -OSQP_INFTY; u[row] = (c_float)b.xmax; ++row;
    A.push_back({row, I.px(k), -1.0}); A.push_back({row, I.sl(k, 0), -1.0});
    l[row] = -OSQP_INFTY; u[row] = (c_float)(-b.xmin); ++row;
    A.push_back({row, I.py(k), 1.0}); A.push_back({row, I.sl(k, 3), -1.0});
    l[row] = -OSQP_INFTY; u[row] = (c_float)b.ymax; ++row;
    A.push_back({row, I.py(k), -1.0}); A.push_back({row, I.sl(k, 2), -1.0});
    l[row] = -OSQP_INFTY; u[row] = (c_float)(-b.ymin); ++row;
  }
  for (int k = 0; k < p.N; ++k)
    for (int c = 0; c < 4; ++c) {
      A.push_back({row, I.sl(k, c), 1.0}); l[row] = 0.0; u[row] = OSQP_INFTY; ++row;
    }

  std::vector<c_int> Pp, Pi, Ap, Ai; std::vector<c_float> Px, Ax;
  to_csc(nv, P, &Pp, &Pi, &Px);
  to_csc(nv, A, &Ap, &Ai, &Ax);
  csc Pc{}, Ac{};
  Pc.nzmax = (c_int)Px.size(); Pc.m = nv; Pc.n = nv; Pc.p = Pp.data(); Pc.i = Pi.data(); Pc.x = Px.data(); Pc.nz = -1;
  Ac.nzmax = (c_int)Ax.size(); Ac.m = m; Ac.n = nv; Ac.p = Ap.data(); Ac.i = Ai.data(); Ac.x = Ax.data(); Ac.nz = -1;
  OSQPData data{}; data.n = nv; data.m = m; data.P = &Pc; data.A = &Ac; data.q = q.data(); data.l = l.data(); data.u = u.data();
  OSQPSettings st; osqp_set_default_settings(&st);
  st.verbose = 0; st.eps_abs = 1e-4; st.eps_rel = 1e-4; st.max_iter = 4000;
  OSQPWorkspace* w = nullptr;
  if (osqp_setup(&w, &data, &st) != 0) { out.status = "setup_failed"; return out; }
  const auto t0 = std::chrono::steady_clock::now();
  osqp_solve(w);
  out.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  out.status = w->info->status; out.iters = (int)w->info->iter;
  if (w->info->status_val == OSQP_SOLVED || w->info->status_val == OSQP_SOLVED_INACCURATE) {
    out.ok = true;
    out.vx.resize(p.N); out.vy.resize(p.N); out.px.resize(p.N); out.py.resize(p.N);
    for (int k = 0; k < p.N; ++k) {
      out.vx[k] = (double)w->solution->x[I.vx(k)];
      out.vy[k] = (double)w->solution->x[I.vy(k)];
      out.px[k] = (double)w->solution->x[I.px(k)];
      out.py[k] = (double)w->solution->x[I.py(k)];
      for (int c = 0; c < 4; ++c)
        out.max_slack = std::max(out.max_slack, (double)w->solution->x[I.sl(k, c)]);
    }
    out.vx0 = out.vx[0]; out.vy0 = out.vy[0];
  }
  osqp_cleanup(w);
  return out;
}

}  // namespace rm_mpc
