// 他们的地图 + 他们的起终点 + 他们的参数（不用 OpenCV，读 PGM）
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "yastar.hpp"
#include "sfcSquare.hpp"
#include "minimumSnap.hpp"

int main(int argc, char** argv) {
  std::ifstream f(argv[1], std::ios::binary);
  std::string magic; int W, H, maxv;
  f >> magic >> W >> H >> maxv; f.get();
  std::vector<unsigned char> img(W * H);
  f.read(reinterpret_cast<char*>(img.data()), W * H);
  const float res = 0.05f;
  YAstar astar(W, H, res, 0.f, 0.f);
  astar.setMap(W, H, img.data());
  astar.setMaskNumLayers(4);
  astar.setCostField(1.0f, [](float x) { return 0.5f / (0.1f + x); });
  astar.initCostMap();
  Eigen::Vector2f start(25.2695f, 14.14615f), end(1.47266f, 9.1749f);
  start.y() = (H - 1) * res - start.y();
  end.y() = (H - 1) * res - end.y();
  auto path = astar.search(start, end);
  path = astar.simplifyPathHypot(astar.simplifyPath(path, 0.1f));
  std::printf("their-map A* %zu 点\n", path.size());

  SfcSquare::Map sm(H, W);
  for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) sm(y, x) = img[y * W + x];
  for (int nt = 0; nt < 2; ++nt) {
    MinimumSnap ms;
    ms.setOrder(6); ms.setMaxDx(3); ms.setDt(0.1);
    ms.setMap(sm, res, 0.f, 0.f);
    ms.setTL(10);
    MinimumSnap::SolveInput in;
    in.setPath(path);
    in.setCollisionCheckIter(6);
    in.setMaxSpeed(3.f); in.setMaxAcc(1.f);
    in.setMaxCorridorRange(2.5f); in.setCorridorShrink(0.0f);
    in.setNormTime(nt == 0);
    in.setBackend(MinimumSnap::Backend::OSQPCorridor);
    in.setInitVel({0.f, 0.f});
    auto r = ms.solve(in);
    std::printf("their-map norm=%d: success=%d iter=%d %.1fms path=%zu corridor=%zu\n",
                nt == 0, (int)r.success, r.iter, r.time * 1e3, r.path.size(), r.corridor.size());
  }
  return 0;
}
