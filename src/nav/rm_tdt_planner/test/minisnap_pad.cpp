// 验证：给地图加 2 格障碍边框（并把 unknown 当障碍）后，MinSnap 是否成功
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "yastar.hpp"
#include "sfcSquare.hpp"
#include "minimumSnap.hpp"

int main(int argc, char** argv) {
  const std::string pgm = argv[1];
  const float res = std::atof(argv[2]), ox = std::atof(argv[3]), oy = std::atof(argv[4]);
  const float sx = std::atof(argv[5]), sy = std::atof(argv[6]), gx = std::atof(argv[7]), gy = std::atof(argv[8]);
  std::ifstream f(pgm, std::ios::binary);
  std::string magic; int W, H, maxv;
  f >> magic >> W >> H >> maxv; f.get();
  std::vector<unsigned char> img(W * H);
  f.read(reinterpret_cast<char*>(img.data()), W * H);

  const int PAD = 2;
  const int WP = W + 2 * PAD, HP = H + 2 * PAD;
  const float oxp = ox - PAD * res, oyp = oy - PAD * res;   // 原点外扩
  std::vector<int8_t> occ(WP * HP, 1);                       // 边框默认障碍
  SfcSquare::Map sm(HP, WP); sm.setConstant(0);
  for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
    // 地图语义：0=障碍(黑)，205=unknown，254/255=自由 ⇒ 与 nav2 allow_unknown 一致：>=200 视为可走
    const bool b = img[y * W + x] < 200;
    occ[(y + PAD) * WP + (x + PAD)] = b ? 1 : 0;
    sm(y + PAD, x + PAD) = b ? 0 : 255;
  }

  YAstar astar;
  astar.setMapping(res); astar.setOriginPos(oxp, oyp);
  astar.setMaskNumLayers(4);
  astar.setMap(WP, HP, res, oxp, oyp, occ);
  astar.setCostField(1.0f, [](float d) { return 0.5f / (0.1f + d); });
  astar.initCostMap();
  auto path = astar.search(Eigen::Vector2f(sx, sy), Eigen::Vector2f(gx, gy));
  auto p3 = astar.simplifyPathHypot(astar.simplifyPath(path, 0.1f));
  std::printf("地图 %dx%d(+pad) A* %zu → 简化 %zu 点\n", WP, HP, path.size(), p3.size());

  struct V { float range, shrink; bool norm; };
  const V vs[] = {{2.5f,0.f,false},{2.5f,0.2f,false},{2.5f,0.2f,true},{5.0f,0.3f,false}};
  for (int i = 0; i < 4; ++i) {
    MinimumSnap ms;
    ms.setOrder(6); ms.setMaxDx(3); ms.setDt(0.1);
    ms.setMap(sm, res, -oxp, -oyp);
    ms.setTL(10.f);
    MinimumSnap::SolveInput in;
    in.setPath(p3);
    in.setCollisionCheckIter(6);
    in.setMaxSpeed(1.5f); in.setMaxAcc(2.0f);
    in.setMaxCorridorRange(vs[i].range);
    in.setCorridorShrink(vs[i].shrink);
    in.setNormTime(vs[i].norm);
    in.setBackend(MinimumSnap::Backend::OSQPCorridor);
    in.setInitVel({0.f, 0.f});
    auto tt = MinimumSnap::trapezoidalTimeAllocation(p3, 1.5f, 2.0f);
    in.setTimeAllocated(tt);
    auto r = ms.solve(in);
    std::printf("range=%.1f shrink=%.1f norm=%d -> success=%d iter=%d %.1fms path=%zu corridor=%zu\n",
                vs[i].range, vs[i].shrink, (int)vs[i].norm, (int)r.success, r.iter, r.time*1e3,
                r.path.size(), r.corridor.size());
  }
  return 0;
}
