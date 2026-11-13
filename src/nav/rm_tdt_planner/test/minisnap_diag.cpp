// MinSnap 失败诊断：枚举 origin 约定 × normTime × shrink
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

  std::vector<int8_t> occ(W * H, 0);
  SfcSquare::Map sm(H, W);
  for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
    const bool b = img[y * W + x] < 200;
    occ[y * W + x] = b ? 1 : 0; sm(y, x) = b ? 0 : 255;
  }
  YAstar astar;
  astar.setMapping(res); astar.setOriginPos(ox, oy);
  astar.setMaskNumLayers(4);
  astar.setMap(W, H, res, ox, oy, occ);
  astar.setCostField(1.0f, [](float d) { return 0.5f / (0.1f + d); });
  astar.initCostMap();
  auto path = astar.search(Eigen::Vector2f(sx, sy), Eigen::Vector2f(gx, gy));
  auto p3 = astar.simplifyPathHypot(astar.simplifyPath(path, 0.1f));
  std::printf("A* %zu → 简化 %zu 点\n", path.size(), p3.size());

  struct V { float range; float shrink; bool initvel; bool norm; };
  const V vs[] = {{2.5f,0.0f,true,false},{5.0f,0.0f,true,false},{2.5f,0.2f,true,false},
                  {2.5f,0.0f,false,false},{5.0f,0.2f,true,true},{5.0f,0.5f,true,false}};
  for (int vi = 0; vi < 6; ++vi) {
    const V& v = vs[vi];
    MinimumSnap ms;
    ms.setOrder(6); ms.setMaxDx(3); ms.setDt(0.1);
    ms.setMap(sm, res, -ox, -oy);
    ms.setTL(10.f);
    MinimumSnap::SolveInput in;
    in.setPath(p3);
    in.setCollisionCheckIter(6);
    in.setMaxSpeed(1.5f); in.setMaxAcc(2.0f);
    in.setMaxCorridorRange(v.range);
    in.setCorridorShrink(v.shrink);
    in.setNormTime(v.norm);
    in.setBackend(MinimumSnap::Backend::OSQPCorridor);
    if (v.initvel) in.setInitVel({0.f, 0.f});
    auto tt = MinimumSnap::trapezoidalTimeAllocation(p3, 1.5f, 2.0f);
    in.setTimeAllocated(tt);
    auto r = ms.solve(in);
    std::printf("range=%.1f shrink=%.1f initvel=%d norm=%d -> success=%d iter=%d %.1fms path=%zu\n",
                v.range, v.shrink, (int)v.initvel, (int)v.norm, (int)r.success, r.iter, r.time*1e3, r.path.size());
  }
  for (int sign = 0; sign < 0; ++sign) {
    const float mx = sign ? ox : -ox, my = sign ? oy : -oy;
    for (int nt = 0; nt < 2; ++nt) {
      for (int sh = 0; sh < 2; ++sh) {
        MinimumSnap ms;
        ms.setOrder(6); ms.setMaxDx(3); ms.setDt(0.1);
        ms.setMap(sm, res, mx, my);
        ms.setTL(10.f);
        MinimumSnap::SolveInput in;
        in.setPath(p3);
        in.setCollisionCheckIter(6);
        in.setMaxSpeed(1.5f); in.setMaxAcc(2.0f);
        in.setMaxCorridorRange(2.5f);
        in.setCorridorShrink(sh ? 0.2f : 0.0f);
        in.setNormTime(nt == 1);
        in.setBackend(MinimumSnap::Backend::OSQPCorridor);
        in.setInitVel({0.f, 0.f});
        auto tt = MinimumSnap::trapezoidalTimeAllocation(p3, 1.5f, 2.0f);
        in.setTimeAllocated(tt);
        auto r = ms.solve(in);
        std::printf("origin=%s norm=%d shrink=%.1f -> success=%d iter=%d %.1fms path=%zu\n",
                    sign ? "(+ox)" : "(-ox)", nt, sh ? 0.2 : 0.0, (int)r.success, r.iter, r.time * 1e3, r.path.size());
      }
    }
  }
  return 0;
}
