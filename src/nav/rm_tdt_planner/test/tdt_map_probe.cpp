// tdt_map_probe —— **用 T-DT 自己的库**检查一张 PGM 地图，并把 A* 结果画回一张 pgm。
//
// 为什么写它：我们有一条"0.02 精度 CAD 点云 → 2D 栅格图"的链路
// （tools/scripts/mapping/pcd_to_nav2_map.py），但那张图**能不能被 T-DT 直接吃**、
// T-DT 在里面走出来的路是什么样，只有让它自己的 YAstar 读一遍才算数。
//
// T-DT 的取值约定（`yastar.cpp: setMap(int,int,u_char*)`，实测确认）：
//     mapData == 0  → 障碍
//     mapData != 0  → 自由
// 与 ROS/nav2 的 PGM 约定（黑=占，白=空）**一致** ⇒ pcd_to_nav2_map.py 的输出可直接喂。
//
// 用法：
//   tdt_map_probe <map.pgm> [sx sy gx gy] [--out overlay.pgm]
//     坐标单位：米，原点 = 图左下角（与 T-DT 的 origin(0,0) 约定一致，y 向上）
//     不给起终点时自动取"最左下/最右上的两个自由格中心"
//
// 退出码：0 = A* 找到路；1 = 没找到；2 = 参数/文件问题
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "yastar.hpp"

static bool loadPGM(const std::string & path, int & w, int & h, std::vector<unsigned char> & px)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) { std::printf("读不到 %s\n", path.c_str()); return false; }
  std::string magic; int maxv = 0;
  f >> magic >> w >> h >> maxv; f.get();
  if (magic != "P5" || w <= 0 || h <= 0) { std::printf("不是 P5 PGM\n"); return false; }
  px.resize(static_cast<size_t>(w) * h);
  f.read(reinterpret_cast<char *>(px.data()), static_cast<std::streamsize>(px.size()));
  return static_cast<bool>(f);
}

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::printf("用法: tdt_map_probe <map.pgm> [sx sy gx gy] [--out overlay.pgm]\n");
    return 2;
  }
  const std::string map_path = argv[1];
  std::string out_path;
  std::vector<float> coords;
  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) { out_path = argv[++i]; }
    else { coords.push_back(std::strtof(argv[i], nullptr)); }
  }

  int W = 0, H = 0;
  std::vector<unsigned char> img;
  if (!loadPGM(map_path, W, H, img)) return 2;
  const float res = 0.05f;

  int n_occ = 0;
  for (auto v : img) if (v == 0) ++n_occ;
  std::printf("[tdt-probe] 图 %dx%d @ %.2f m = %.1f x %.1f m | 障碍 %d 格 (%.1f%%)\n",
              W, H, res, W * res, H * res, n_occ, 100.0 * n_occ / (W * H));

  // 起终点：给的坐标是**米**，转成 T-DT 的栅格坐标（它与 main.cpp 一致：
  //   row = H-1 - y/res ⇒ 这里用同一个换算，避免 y 方向反了还看不出来）
  auto to_grid = [&](float xm, float ym) -> Eigen::Vector2f {
      return Eigen::Vector2f(xm, static_cast<float>(H - 1) * res - ym);
    };
  auto is_free = [&](int gx, int gy) {
      if (gx < 0 || gy < 0 || gx >= W || gy >= H) return false;
      return img[static_cast<size_t>(gy) * W + gx] != 0;
    };

  Eigen::Vector2f start, end;
  if (coords.size() >= 4) {
    start = to_grid(coords[0], coords[1]);
    end = to_grid(coords[2], coords[3]);
  } else {
    // 自动：取最左下的自由格 与 最右上的自由格
    bool s_ok = false, e_ok = false;
    for (int y = 0; y < H && !s_ok; ++y)
      for (int x = 0; x < W && !s_ok; ++x)
        if (is_free(x, y)) { start = Eigen::Vector2f(x * res, y * res); s_ok = true; }
    for (int y = H - 1; y >= 0 && !e_ok; --y)
      for (int x = W - 1; x >= 0 && !e_ok; --x)
        if (is_free(x, y)) { end = Eigen::Vector2f(x * res, y * res); e_ok = true; }
    if (!s_ok || !e_ok) { std::printf("[tdt-probe] 图里没有自由格\n"); return 2; }
  }
  std::printf("[tdt-probe] 起点 (%.2f, %.2f) → 终点 (%.2f, %.2f) [T-DT 栅格坐标]\n",
              start.x(), start.y(), end.x(), end.y());

  YAstar astar(W, H, res, 0.f, 0.f);
  astar.setMap(W, H, img.data());
  astar.setMaskNumLayers(4);
  astar.setCostField(1.0f, [](float x) { return 0.5f / (0.1f + x); });
  astar.initCostMap();

  const auto t0 = std::chrono::steady_clock::now();
  auto path = astar.search(start, end);
  const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) len += (path[i] - path[i - 1]).norm();
  std::printf("[tdt-probe] A* %zu 点, 长度 %.2f m, 耗时 %.1f ms\n", path.size(), len, ms);

  if (path.size() >= 2) {
    auto simp = astar.simplifyPathHypot(astar.simplifyPath(path, 0.1f));
    std::printf("[tdt-probe] 简化后 %zu 点\n", simp.size());
    if (!simp.empty()) path = simp;
  }

  if (!out_path.empty()) {
    std::vector<unsigned char> vis = img;              // 0=障碍 / 255=自由
    // ★ 必须**逐段连线**：path 经过简化后只剩几个拐点，
    //   只画点会出现"只有 4 个像素"的假象（实测踩过）。
    auto plot = [&](float x, float y) {
        const int gx = static_cast<int>(x / res), gy = static_cast<int>(y / res);
        if (gx >= 0 && gy >= 0 && gx < W && gy < H)
          vis[static_cast<size_t>(gy) * W + gx] = 128;    // 路径 = 中灰
      };
    for (size_t i = 1; i < path.size(); ++i) {
      const float ax = path[i - 1].x(), ay = path[i - 1].y();
      const float bx = path[i].x(), by = path[i].y();
      const float n = std::max(1.0f, std::hypot(bx - ax, by - ay) / (res * 0.5f));
      for (int k = 0; k <= static_cast<int>(n); ++k) {
        const float t = static_cast<float>(k) / n;
        plot(ax + t * (bx - ax), ay + t * (by - ay));
      }
    }
    std::ofstream o(out_path, std::ios::binary);
    o << "P5\n" << W << " " << H << "\n255\n";
    o.write(reinterpret_cast<const char *>(vis.data()),
            static_cast<std::streamsize>(vis.size()));
    std::printf("[tdt-probe] 叠加图 → %s（0=障碍 128=路径 255=自由）\n", out_path.c_str());
  }

  return path.size() >= 2 ? 0 : 1;
}
