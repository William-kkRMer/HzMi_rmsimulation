// 离线测试：YAstar 路径 → MinimumSnap 轨迹（含 SFC 走廊）→ 输出点/速度/时间分配
#include <cstdio>
#include <vector>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>
#include "minimumSnap.hpp"
#include "sfcSquare.hpp"
#include "yastar.hpp"

namespace {
constexpr int W = 600, H = 400;
constexpr float RES = 0.05f;
}

int main() {
  std::vector<int8_t> occ(W * H, 0);
  auto wall_v = [&](float x, float y0, float y1) {
    const int ix = (int)(x / RES);
    for (int iy = (int)(y0 / RES); iy <= (int)(y1 / RES); ++iy) occ[iy * W + ix] = 1;
  };
  wall_v(15.f, 0.f, 7.5f); wall_v(15.f, 10.5f, 20.f);
  wall_v(22.f, 0.f, 5.f);  wall_v(22.f, 8.f, 20.f);
  // 膨胀 2 格
  { auto src = occ;
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) if (src[y*W+x])
      for (int dy=-2; dy<=2; ++dy) for (int dx=-2; dx<=2; ++dx) {
        int nx=x+dx, ny=y+dy; if (nx>=0&&nx<W&&ny>=0&&ny<H) occ[ny*W+nx]=1; } }

  YAstar astar;
  astar.setMapping(RES); astar.setOriginPos(0.f, 0.f);
  astar.setMap(W, H, RES, 0.f, 0.f, occ);
  auto path = astar.search(Eigen::Vector2f(2.f, 2.f), Eigen::Vector2f(26.f, 10.f));
  std::printf("A*: %zu 点, 长度 %.2f m\n", path.size(), YAstar::getLength(path));
  if (path.size() < 3) return 1;

  // 走廊用的地图：0=障碍, 255=自由
  SfcSquare::Map sm(H, W);
  for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) sm(y, x) = occ[y*W+x] ? 0 : 255;

  MinimumSnap ms;
  ms.setOrder(4);          // 4 次多项式
  ms.setDt(0.05f);
  ms.setMap(sm, RES, 0.f, 0.f);

  MinimumSnap::SolveInput in;
  in.setPath(path);
  in.setMaxSpeed(1.5f);
  in.setMaxAcc(2.0f);
  in.setCollisionCheckIter(30);
  in.setBackend(MinimumSnap::Backend::OSQPCorridor);

  const auto t0 = std::chrono::steady_clock::now();
  auto out = ms.solve(in);
  const double ms_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  const auto& ta = in.getTimeAllocated();
  double total_t = 0.0;
  for (double v : ta) total_t += v;
  std::printf("MinimumSnap: success=%d 点=%zu 走廊=%zu 段=%zu 总时长=%.2f s 求解=%.2f ms 迭代=%d\n",
              (int)out.success, out.path.size(), out.corridor.size(), ta.size(), total_t, ms_ms, out.iter);

  if (!out.success || out.path.size() < 2) return 2;
  // 用"分段长度/分段时长"估速度剖面（out.path 每段 11 个采样点）
  double vmax = 0.0, vsum = 0.0; int nv = 0;
  for (size_t i = 1; i < out.path.size(); ++i) {
    const double d = (out.path[i] - out.path[i-1]).norm();
    double dt = 0.05;
    if (!ta.empty()) {
      const size_t seg = std::min(ta.size() - 1, (i - 1) * ta.size() / std::max<size_t>(1, out.path.size() - 1));
      dt = ta[seg] / std::max<size_t>(1, (out.path.size() - 1) / ta.size());
    }
    const double v = d / std::max(1e-6, dt);
    vmax = std::max(vmax, v); vsum += v; ++nv;
  }
  std::printf("估速度: 均值 %.2f m/s，最大 %.2f m/s（限制 1.5）\n", vsum / std::max(1, nv), vmax);
  std::printf("起点 (%.2f, %.2f) → 终点 (%.2f, %.2f)\n", out.path.front().x(), out.path.front().y(),
              out.path.back().x(), out.path.back().y());
  return 0;
}
