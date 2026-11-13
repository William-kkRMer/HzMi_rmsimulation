// 集成测试：tdt-nav-kit（A* / KinodynamicAstar / SfcSquare）+ 我们的 mpc_core
// 全程离线：自建栅格 → 规划 → 走廊 → MPC 闭环 → 指标
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "kinodynamicAstar.hpp"
#include "mpc_core.hpp"
#include "sfcSquare.hpp"
#include "yastar.hpp"

namespace {
constexpr int W = 600, H = 400;      // 30 m x 20 m
constexpr float RES = 0.05f;

struct Grid {
  std::vector<int8_t> occ;           // 0=free, 1=obstacle（YAstar 约定）
  Grid() : occ(W * H, 0) {}
  void wall_v(float x, float y0, float y1) {
    const int ix = (int)(x / RES);
    for (int iy = (int)(y0 / RES); iy <= (int)(y1 / RES); ++iy)
      occ[iy * W + ix] = 1;
  }
  void inflate(int r) {
    auto src = occ;
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        if (src[y * W + x] == 0) continue;
        for (int dy = -r; dy <= r; ++dy)
          for (int dx = -r; dx <= r; ++dx) {
            const int nx = x + dx, ny = y + dy;
            if (nx >= 0 && nx < W && ny >= 0 && ny < H) occ[ny * W + nx] = 1;
          }
      }
  }
};

// 轨迹插值（按时间）
struct Traj {
  std::vector<KinodynamicAstar::Sample> s;
  int index_at(double t) const {
    int best = 0; double bd = 1e9;
    for (size_t i = 0; i < s.size(); ++i) {
      const double d = std::fabs(s[i].time - t);
      if (d < bd) { bd = d; best = (int)i; }
    }
    return best;
  }
  bool at(double t, Eigen::Vector2d* p, Eigen::Vector2d* v) const {
    if (s.empty()) return false;
    if (t <= s.front().time) { *p = s.front().state.head<2>(); *v = s.front().state.tail<2>(); return true; }
    if (t >= s.back().time) { *p = s.back().state.head<2>(); *v = s.back().state.tail<2>(); return true; }
    for (size_t i = 1; i < s.size(); ++i) {
      if (t <= s[i].time) {
        const double a = (t - s[i - 1].time) / std::max(1e-9, s[i].time - s[i - 1].time);
        *p = s[i - 1].state.head<2>() * (1 - a) + s[i].state.head<2>() * a;
        *v = s[i - 1].state.tail<2>() * (1 - a) + s[i].state.tail<2>() * a;
        return true;
      }
    }
    return false;
  }
};
}  // namespace

int main() {
  Grid g;
  g.wall_v(15.0f, 0.0f, 7.5f);      // 下墙（留出 7.5~20 的缺口）
  g.wall_v(15.0f, 10.5f, 20.0f);    // 上墙
  g.wall_v(22.0f, 0.0f, 5.0f);      // 第二道墙
  g.wall_v(22.0f, 8.0f, 20.0f);
  g.inflate(2);                     // 10 cm 膨胀

  // ---- YAstar ----
  YAstar astar;
  astar.setMapping(RES);
  astar.setOriginPos(0.f, 0.f);
  astar.setMap(W, H, RES, 0.f, 0.f, g.occ);
  const Eigen::Vector2f start(2.f, 2.f), goal(26.f, 10.f);
  auto path = astar.search(start, goal);
  std::printf("A*: %zu 点, 长度 %.2f m\n", path.size(), YAstar::getLength(path));

  // ---- KinodynamicAstar ----
  KinodynamicAstar kd;
  kd.setMapping(RES);
  kd.setOriginPos(0.f, 0.f);
  kd.setMap(W, H, RES, 0.f, 0.f, g.occ);
  kd.config.maxVel = 1.5;
  kd.config.maxAcc = 2.0;
  kd.config.sampleTime = 0.05;
  kd.config.maxNodes = 30000;
  const auto t0 = std::chrono::steady_clock::now();
  auto kres = kd.search(start.cast<double>(), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 0), goal.cast<double>());
  const double plan_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::printf("KinodynamicAstar: success=%d samples=%zu nodes=%zu 耗时 %.1f ms\n",
              (int)kres.success, kres.trajectory.size(), kres.nodes, plan_ms);
  if (!kres.success || kres.trajectory.empty()) { std::printf("规划失败，退出\n"); return 1; }
  Traj traj{kres.trajectory};

  // ---- SfcSquare 走廊（约定：0=障碍，255=自由）----
  SfcSquare::Map sm(H, W);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) sm(y, x) = g.occ[y * W + x] ? 0 : 255;
  SfcSquare sfc;
  sfc.setMap(sm, RES, Eigen::Vector2f(0.f, 0.f));
  std::vector<Eigen::Vector2f> pts;
  for (size_t i = 0; i < kres.trajectory.size(); i += 10)   // ★ 抽稀：每 10 个样本一个走廊点
    pts.emplace_back((float)kres.trajectory[i].state.x(), (float)kres.trajectory[i].state.y());
  if (pts.empty() || pts.back() != Eigen::Vector2f((float)kres.trajectory.back().state.x(), (float)kres.trajectory.back().state.y()))
    pts.emplace_back((float)kres.trajectory.back().state.x(), (float)kres.trajectory.back().state.y());
  auto cor = sfc.getCorridor(pts, 3.0f, 0.05f);
  std::printf("走廊: %zu 个 box（最大扩张 3 m，收缩 5 cm）\n", cor.corridor.size());

  // 轨迹样本索引 → 走廊点索引（pts 每 10 个样本取一个）
  std::vector<int> sample_to_pts(kres.trajectory.size(), 0);
  for (size_t i = 0; i < sample_to_pts.size(); ++i)
    sample_to_pts[i] = std::min<int>((int)i / 10, (int)pts.size() - 1);
  // ★ 走廊 box 与 pts 的对应要看 cor.index（_removeRedundant 会删点，下标不是一一对应）
  std::vector<int> pts_to_box(pts.size(), -1);
  for (size_t j = 0; j < cor.index.size(); ++j) {
    const int pi = cor.index[j];
    if (pi >= 0 && pi < (int)pts.size()) pts_to_box[pi] = (int)j;
  }
  // 统计退化 box（宽度或高度 < 0.10 m）——它们会让约束变成"钉死一个点"
  {
    int bad = 0;
    for (const auto& pp : cor.corridor)
      if ((pp[2] - pp[0]) < 0.10f || (pp[3] - pp[1]) < 0.10f) ++bad;
    std::printf("走廊: 其中退化 box %d / %zu（< 0.10 m）\n", bad, cor.corridor.size());
  }
  std::vector<int> keep_box(cor.corridor.size(), 0);
  {
    int last = -1;
    for (size_t j = 0; j < cor.corridor.size(); ++j) {
      const auto& pp = cor.corridor[j];
      const bool degenerate = (pp[2] - pp[0]) < 0.10f || (pp[3] - pp[1]) < 0.10f;
      if (!degenerate) last = (int)j;
      keep_box[j] = last;
    }
    if (last < 0) { std::printf("全部走廊退化，改用大 box\n"); for (auto& v : keep_box) v = -1; }
  }
  auto box_for_pts = [&](int pi) -> int {
    for (int d = 0; d < (int)pts.size(); ++d) {
      if (pi - d >= 0 && pts_to_box[pi - d] >= 0) return pts_to_box[pi - d];
      if (pi + d < (int)pts.size() && pts_to_box[pi + d] >= 0) return pts_to_box[pi + d];
    }
    return 0;
  };

  // ---- MPC 闭环 ----
  rm_mpc::Params p;
  p.dt = 0.05; p.N = 16; p.vmax = 2.0; p.amax = 4.0;
  Eigen::Vector2d s(2.0, 2.0), v(0.0, 0.0);
  const double total = std::min(20.0, kres.trajectory.back().time + 2.0);
  double sum_sq = 0, max_viol = 0, sum_ms = 0, max_ms = 0, sum_iter = 0, max_slack_all = 0, sum_slack = 0;
  int steps = 0, fails = 0, fallbacks = 0;
  for (double t = 0.0; t < total; t += p.dt) {
    std::vector<rm_mpc::RefStage> ref(p.N);
    std::vector<rm_mpc::Box> box(p.N, rm_mpc::Box{});
    for (int k = 0; k < p.N; ++k) {
      Eigen::Vector2d rp, rv;
      if (!traj.at(t + (k + 1) * p.dt, &rp, &rv)) { rp = s; rv = v; }
      ref[k].x = rp.x(); ref[k].y = rp.y(); ref[k].vx = rv.x(); ref[k].vy = rv.y();
      // 按 stage 时间对应的轨迹样本取走廊 box（而不是最近中心，避免跳到远处 box）
      const int si = traj.index_at(t + (k + 1) * p.dt);
      const int bi = box_for_pts(sample_to_pts[si]);
      const auto& pp = cor.corridor[bi];
      const bool degenerate = (pp[2] - pp[0]) < 0.10f || (pp[3] - pp[1]) < 0.10f;
      // ★ 退化 box 不下约束（该点附近没有可用凸区域），而不是拿远处 box 顶替
      box[k] = degenerate ? rm_mpc::Box{}
                          : rm_mpc::Box{(double)pp[0], (double)pp[1], (double)pp[2], (double)pp[3]};
    }
    Eigen::Vector4d s0;
    s0 << s.x(), s.y(), v.x(), v.y();
    rm_mpc::Solution sol = rm_mpc::solve(p, s0, ref, box);
    if (!sol.ok) {                     // ★ 回退：去掉走廊约束再解一次
      ++fails;
      rm_mpc::Params p2 = p; p2.use_corridor = false;
      sol = rm_mpc::solve(p2, s0, ref, box);
      ++fallbacks;
    }
    if (!sol.ok) { v.setZero(); continue; }
    sum_ms += sol.ms; max_ms = std::max(max_ms, sol.ms); sum_iter += sol.iters;
    max_slack_all = std::max(max_slack_all, sol.max_slack); sum_slack += sol.max_slack;
    v.x() += sol.ax[0] * p.dt;
    v.y() += sol.ay[0] * p.dt;
    s += v * p.dt;
    if (steps > 40 && steps < 90) v.y() += 0.01;  // 轻微扰动
    Eigen::Vector2d rp, rv;
    traj.at(t + p.dt, &rp, &rv);
    const double ex = s.x() - rp.x(), ey = s.y() - rp.y();
    sum_sq += ex * ex + ey * ey;
    // 走廊违背：对当前这一步实际使用的 box（k=0）
    {
      const auto& b0 = box[0];
      const bool constrained = b0.xmin > -1e8;   // 无限 box = 该步无走廊约束
      const double vx = constrained ? std::max(0.0, std::max(b0.xmin - s.x(), s.x() - b0.xmax)) : 0.0;
      const double vy = constrained ? std::max(0.0, std::max(b0.ymin - s.y(), s.y() - b0.ymax)) : 0.0;
      const double vi = std::hypot(vx, vy);
      if (vi > max_viol) {
        max_viol = vi;
        if (std::getenv("MPC_DEBUG")) {
          std::printf("  [viol] t=%.2f p=(%.2f,%.2f) box x[%.2f,%.2f] y[%.2f,%.2f] slack=%.3f\n",
                      t, s.x(), s.y(), b0.xmin, b0.xmax, b0.ymin, b0.ymax, sol.max_slack);
        }
      }
    }
    ++steps;
  }
  const double n = std::max(1, steps);
  std::printf("MPC: steps=%d 走廊不可行=%d(已回退) 仍失败=%d  RMSE=%.4f m max走廊违背=%.4f m 求解 %.2f/%.2f ms 迭代 %.0f  slack 均值 %.3f / 最大 %.3f m\n",
              steps, fallbacks, fails - fallbacks, std::sqrt(sum_sq / n), max_viol, sum_ms / n, max_ms, sum_iter / n,
              sum_slack / n, max_slack_all);
  return 0;
}
