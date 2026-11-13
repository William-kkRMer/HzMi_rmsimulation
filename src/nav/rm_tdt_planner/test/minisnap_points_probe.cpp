// minisnap_points_probe —— 用 **T-DT 的 MinSnap** 复现"给定控制点就失败"的问题。
//
// 为什么写它（2026-11-12 实测）：
//   仿真实测日志：
//     `[tdt] MinimumSnap 失败(snap=0, iter=6, 1.4ms) ⇒ 回退 A* 简化路径(4 点)`
//     `[tdt] plan: 4 点, ... A* 215→简化 4, snap=0`
//   1.4 ms 说明是 **QP 立即不可行**（不是迭代超时）。T-DT 文档（doc/MinimumSnap.md:196）
//   明确写着方形走廊"**不能保证走廊是相接的**" ⇒ 控制点太稀（段长 2~3 m）时，
//   相邻控制点的方形走廊接不上 ⇒ 硬约束矛盾 ⇒ 不可行。
//
//   本工具让你**离线**验证这个假设：同一组点，换 3 个条件跑：
//     · 自由图（无障碍）      —— 若仍失败 ⇒ 是几何/条件数问题，与障碍无关
//     · 真实图                —— 若失败 ⇒ 走廊被障碍卡住
//     · 加密控制点（--densify）—— 若成功 ⇒ 证明"点太稀"是根因
//
// 用法：
//   minisnap_points_probe --pts "1,1 4,1 8,1 11,1"
//   minisnap_points_probe --pts "..." --pgm map.pgm
//   minisnap_points_probe --pts "..." --densify 4        # 每段插 3 个点
//   （坐标单位米；PGM 的 origin 视作 (0,0)、y 向上，与 planner 的局部坐标一致）
//
// 参数与 planner.cpp 完全一致：order6 maxDx3 dt0.1 TL10 speed3 acc1
//   corridorRange2.5 shrink0.15 iter6 normTime=true backend=OSQPCorridor
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "sfcSquare.hpp"
#include "minimumSnap.hpp"

static bool loadPGM(const std::string & path, int & w, int & h, std::vector<unsigned char> & px)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::string magic; int maxv = 0;
  f >> magic >> w >> h >> maxv; f.get();
  if (magic != "P5" || w <= 0 || h <= 0) return false;
  px.resize(static_cast<size_t>(w) * h);
  f.read(reinterpret_cast<char *>(px.data()), static_cast<std::streamsize>(px.size()));
  return static_cast<bool>(f);
}

int main(int argc, char ** argv)
{
  std::string pts_arg, pgm;
  int densify = 1;
  bool strict = true;      // 默认与 planner.yaml 的 strict_collision 一致
  bool flip_y = true;
  int dilate_cells = 0;    // ★ 与规划器的 dilate_cells 对齐（PGM 里是**未膨胀**的原始障碍）      // ★ PGM 第 0 行是**顶部**，而规划器局部栅格第 0 行是**底部**
                           //   ⇒ 必须在装入时上下翻转，否则点会落到镜像位置（实测踩过：
                           //   不翻转时"净空 0.00 m"，翻转后才是真实数值）。
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--pts") && i + 1 < argc) pts_arg = argv[++i];
    else if (!std::strcmp(argv[i], "--pgm") && i + 1 < argc) pgm = argv[++i];
    else if (!std::strcmp(argv[i], "--densify") && i + 1 < argc) densify = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--no-strict")) strict = false;
    else if (!std::strcmp(argv[i], "--strict")) strict = true;
    else if (!std::strcmp(argv[i], "--noflip")) flip_y = false;
    else if (!std::strcmp(argv[i], "--dilate") && i + 1 < argc) dilate_cells = std::atoi(argv[++i]);
  }
  if (pts_arg.empty()) {
    std::printf("用法: minisnap_points_probe --pts \"x,y x,y ...\" [--pgm m.pgm] [--densify N]\n");
    return 2;
  }

  // 解析控制点
  std::vector<Eigen::Vector2f> pts;
  {
    std::string s = pts_arg;
    for (auto & c : s) if (c == ',' || c == ';') c = ' ';
    const char * p = s.c_str();
    float x = 0, y = 0; int n = 0;
    while (std::sscanf(p, "%f %f%n", &x, &y, &n) == 2) {
      pts.emplace_back(x, y);
      p += n;
    }
  }
  if (pts.size() < 2) { std::printf("至少要 2 个点\n"); return 2; }

  // 加密
  if (densify > 1 && pts.size() >= 2) {
    std::vector<Eigen::Vector2f> dense;
    for (size_t i = 1; i < pts.size(); ++i) {
      for (int k = 0; k < densify; ++k) {
        const float t = static_cast<float>(k) / densify;
        dense.push_back(pts[i - 1] + t * (pts[i] - pts[i - 1]));
      }
    }
    dense.push_back(pts.back());
    pts = dense;
  }

  // 地图：自由图 or 给定 pgm
  int W = 0, H = 0;
  std::vector<unsigned char> img;
  const float res = 0.05f;
  if (!pgm.empty()) {
    if (!loadPGM(pgm, W, H, img)) { std::printf("读不到 %s\n", pgm.c_str()); return 2; }
    if (flip_y) {
      std::vector<unsigned char> f(static_cast<size_t>(W) * H);
      for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
          f[static_cast<size_t>(y) * W + x] = img[static_cast<size_t>(H - 1 - y) * W + x];
      img.swap(f);
    }
    std::printf("[probe] 地图 %s %dx%d @%.2f\n", pgm.c_str(), W, H, res);
  } else {
    float mx = 0, my = 0;
    for (auto & q : pts) { mx = std::max(mx, q.x()); my = std::max(my, q.y()); }
    W = static_cast<int>((mx + 2.0f) / res);
    H = static_cast<int>((my + 2.0f) / res);
    img.assign(static_cast<size_t>(W) * H, 255);   // 全自由
    std::printf("[probe] 自由图(无障碍) %dx%d @%.2f\n", W, H, res);
  }

  std::printf("[probe] 控制点 %zu 个:", pts.size());
  for (size_t i = 0; i < pts.size() && i < 10; ++i)
    std::printf(" (%.2f,%.2f)", pts[i].x(), pts[i].y());
  std::printf("%s\n", pts.size() > 10 ? " …" : "");

  // ★ 可选：把障碍按规划器同样的方式膨胀 N 格（4 邻域），否则"静态图 vs 合并栅格"
  //   的对比就不是同一口径（合并栅格里已经含 dilate）。
  if (dilate_cells > 0) {
    for (int it = 0; it < dilate_cells; ++it) {
      std::vector<unsigned char> b = img;
      for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
          if (img[static_cast<size_t>(y) * W + x] != 0) continue;
          const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
          for (int k = 0; k < 4; ++k) {
            const int nx = x + dx[k], ny = y + dy[k];
            if (nx >= 0 && ny >= 0 && nx < W && ny < H) b[static_cast<size_t>(ny) * W + nx] = 0;
          }
        }
      }
      img.swap(b);
    }
    std::printf("[probe] 已按规划器口径膨胀 %d 格\n", dilate_cells);
  }

  SfcSquare::Map sm(H, W);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x)
      sm(y, x) = img[static_cast<size_t>(y) * W + x];

  MinimumSnap ms;
  ms.setOrder(6);
  ms.setMaxDx(3);
  ms.setDt(0.1);
  ms.setMap(sm, res, 0.f, 0.f);
  ms.setTL(10);
  ms.setStrictCollision(strict);

  MinimumSnap::SolveInput in;
  in.setPath(pts);
  in.setCollisionCheckIter(6);
  in.setMaxSpeed(3.f);
  in.setMaxAcc(1.f);
  in.setMaxCorridorRange(2.5f);
  in.setCorridorShrink(0.15f);
  in.setNormTime(true);
  in.setBackend(MinimumSnap::Backend::OSQPCorridor);
  in.setInitVel({0.f, 0.f});

  // ★ --sdf：解完之后算轨迹的**最小净空**（到最近障碍格的欧氏距离）。
  //   这样"非严格模式到底会不会擦墙"就有数字，而不是靠感觉。
  auto sdf_min = [&](const std::vector<Eigen::Vector2f> & traj) -> double {
      std::vector<int> dist(static_cast<size_t>(W) * H, -1);
      std::deque<std::pair<int,int>> q;
      for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
          if (img[static_cast<size_t>(y) * W + x] == 0) { dist[y*W+x] = 0; q.push_back({x,y}); }
      while (!q.empty()) {
        auto c = q.front(); q.pop_front();
        const int d = dist[c.second*W + c.first];
        const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
        for (int k = 0; k < 4; ++k) {
          const int nx = c.first + dx[k], ny = c.second + dy[k];
          if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
          if (dist[ny*W + nx] >= 0) continue;
          dist[ny*W + nx] = d + 1; q.push_back({nx, ny});
        }
      }
      double m = std::numeric_limits<double>::infinity();
      for (auto & q2 : traj) {
        const int cx = static_cast<int>(q2.x() / res), cy = static_cast<int>(q2.y() / res);
        if (cx < 0 || cy < 0 || cx >= W || cy >= H) continue;
        const int d = dist[cy*W + cx];
        if (d >= 0) m = std::min(m, d * static_cast<double>(res));
      }
      return m;
    };

  auto r = ms.solve(in);
  double sdfm = -1.0;
  if (!r.path.empty()) sdfm = sdf_min(r.path);
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) len += (pts[i] - pts[i - 1]).norm();
  std::printf("[probe] 输入长度 %.2f m, 平均段长 %.2f m\n", len, len / (pts.size() - 1));
  std::printf("[probe] strict=%d 结果: snap=%d iter=%d 路径 %zu 点 耗时 %.2f ms | 轨迹最小净空 %.2f m\n",
              static_cast<int>(strict),
              static_cast<int>(r.success), r.iter, r.path.size(), r.time * 1e3, sdfm);
  return r.success ? 0 : 1;
}
