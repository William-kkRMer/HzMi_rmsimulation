// 用真实场地栅格跑 T-DT 链路，输出 path + corridor，供 Python 画图
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "yastar.hpp"
#include "sfcSquare.hpp"

int main(int argc, char** argv) {
  if (argc < 8) { std::fprintf(stderr, "用法: %s map.pgm res ox oy sx sy gx gy\n", argv[0]); return 1; }
  const std::string pgm = argv[1];
  const float res = std::atof(argv[2]);
  const float ox = std::atof(argv[3]), oy = std::atof(argv[4]);
  const float sx = std::atof(argv[5]), sy = std::atof(argv[6]);
  const float gx = std::atof(argv[7]), gy = std::atof(argv[8]);

  std::ifstream f(pgm, std::ios::binary);
  std::string magic; int W, H, maxv;
  f >> magic >> W >> H >> maxv; f.get();
  std::vector<unsigned char> img(W * H);
  f.read(reinterpret_cast<char*>(img.data()), W * H);

  std::vector<int8_t> occ(W * H, 0);      // YAstar: 0=free,1=occ
  SfcSquare::Map sm(H, W);                // SfcSquare: 0=障碍
  for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
    const bool blocked = img[y * W + x] < 200;
    occ[y * W + x] = blocked ? 1 : 0;
    sm(y, x) = blocked ? 0 : 255;
  }

  YAstar astar;
  astar.setMapping(res); astar.setOriginPos(ox, oy);
  astar.setMap(W, H, res, ox, oy, occ);
  auto path = astar.search(Eigen::Vector2f(sx, sy), Eigen::Vector2f(gx, gy));

  SfcSquare sfc;
  sfc.setMap(sm, res, Eigen::Vector2f(-ox, -oy));   // SfcSquare: idx=(x+origin)/res ⇒ 与 YAstar 反号
  const float shrink = 0.25f;
  auto cor = sfc.getCorridor(path, 3.0f, shrink);   // ⚠️ maxRange=1e9 会 int 溢出 ⇒ 全部退化成点

  // 输出
  std::printf("# meta %d %d %f %f %f %zu %zu\n", W, H, res, ox, oy, path.size(), cor.corridor.size());
  std::printf("# path\n");
  for (auto& p : path) std::printf("%.4f %.4f\n", p.x(), p.y());
  std::printf("# corridor\n");
  for (auto& b : cor.corridor) std::printf("%.4f %.4f %.4f %.4f\n", b[0], b[1], b[2], b[3]);
  return 0;
}
