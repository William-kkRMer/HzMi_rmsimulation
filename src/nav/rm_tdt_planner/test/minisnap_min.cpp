// 最小复现：空白地图 + 直线路径，验证 MinSnap 是否本身可用
#include <cstdio>
#include <vector>
#include <Eigen/Dense>
#include "minimumSnap.hpp"

int main() {
  const int W = 400, H = 200; const float res = 0.05f;
  SfcSquare::Map m(H, W); m.setConstant(255);

  std::vector<Eigen::Vector2f> path{{-5.f, 0.f}, {-2.f, 0.f}, {0.f, 0.f}, {2.f, 0.f}, {5.f, 0.f}};
  for (int variant = 0; variant < 4; ++variant) {
    MinimumSnap ms;
    ms.setOrder(6); ms.setMaxDx(3); ms.setDt(0.1);
    ms.setMap(m, res, 5.f, 0.f);      // 世界(-5..5) → 索引(0..200)
    ms.setTL(10.f);
    MinimumSnap::SolveInput in;
    in.setPath(path);
    in.setCollisionCheckIter(6);
    in.setMaxSpeed(variant == 3 ? 8.f : 1.5f);
    in.setMaxAcc(variant == 3 ? 2.f : 2.0f);
    in.setMaxCorridorRange(2.5f);
    in.setCorridorShrink(0.f);
    in.setNormTime(variant == 1 || variant == 2);
    in.setBackend(variant == 2 ? MinimumSnap::Backend::Close : MinimumSnap::Backend::OSQPCorridor);
    if (variant != 1) in.setInitVel({0.f, 0.f});
    auto tt = MinimumSnap::trapezoidalTimeAllocation(path, variant == 3 ? 8.f : 1.5f, 2.f);
    in.setTimeAllocated(tt);
    auto r = ms.solve(in);
    std::printf("variant %d: success=%d iter=%d %.1fms path=%zu\n", variant, (int)r.success, r.iter, r.time*1e3, r.path.size());
  }
  return 0;
}
