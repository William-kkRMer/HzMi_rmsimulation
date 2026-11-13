// 极简 OpenCV 桩：仅满足 tdt-nav-kit/main.cpp 的调用（无 GUI/无绘图）
#pragma once
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <cstdint>

namespace cv {
using uchar = unsigned char;
enum { IMREAD_GRAYSCALE = 0, COLOR_GRAY2BGR = 1, LINE_AA = 16, CV_8UC1 = 0, CV_8UC3 = 16 };
struct Vec3b {
  unsigned char v[3] = {0, 0, 0};
  Vec3b() = default;
  Vec3b(unsigned char a, unsigned char b, unsigned char c) { v[0]=a; v[1]=b; v[2]=c; }
  unsigned char& operator[](int i) { return v[i]; }
};

struct Point { int x = 0, y = 0; Point() = default; Point(int x_, int y_) : x(x_), y(y_) {} };
struct Rect2f {
  float x = 0, y = 0, width = 0, height = 0;
  Rect2f() = default;
  Rect2f(float a, float b, float c, float d) : x(a), y(b), width(c), height(d) {}
};
struct Size { int width = 0, height = 0; Size() = default; Size(int w, int h) : width(w), height(h) {} };
struct Scalar {
  double v[4] = {0, 0, 0, 0};
  Scalar(double a = 0, double b = 0, double c = 0, double d = 0) { v[0]=a; v[1]=b; v[2]=c; v[3]=d; }
};

struct Mat {
  int rows = 0, cols = 0, channels_ = 1;
  std::vector<unsigned char> buf;
  unsigned char* data = nullptr;
  Mat() = default;
  Mat(int r, int c, int type) { resize(r, c, type == CV_8UC3 ? 3 : 1); }
  Mat(int r, int c, int type, void* ext) { rows = r; cols = c; channels_ = (type == CV_8UC3 ? 3 : 1);
    buf.assign(static_cast<size_t>(r) * c * channels_, 0); data = static_cast<unsigned char*>(ext); }
  Mat(int r, int c, int type, const Scalar& s) { resize(r, c, type == CV_8UC3 ? 3 : 1);
    for (size_t i = 0; i < buf.size(); ++i) buf[i] = static_cast<unsigned char>(s.v[i % 3]); }
  void resize(int r, int c, int ch = 1) {
    rows = r; cols = c; channels_ = ch; buf.assign(static_cast<size_t>(r) * c * ch, 0); data = buf.data();
  }
  bool empty() const { return rows == 0 || cols == 0; }
  unsigned char& at(int y, int x) { return buf[static_cast<size_t>(y) * cols * channels_ + x * channels_]; }
  template <typename T> T& at(int y, int x) {
    return *reinterpret_cast<T*>(&buf[(static_cast<size_t>(y) * cols + x) * channels_]);
  }
};

// 读 "<path>.pgm"（P5），保持 main.cpp 的路径不变
inline Mat imread(const std::string& path, int = IMREAD_GRAYSCALE) {
  Mat m;
  std::FILE* f = std::fopen((path + ".pgm").c_str(), "rb");
  if (!f) f = std::fopen(path.c_str(), "rb");
  if (!f) { std::printf("[cv-stub] 打不开 %s\n", path.c_str()); return m; }
  char magic[3] = {0}; int w = 0, h = 0, maxv = 0;
  if (std::fscanf(f, "%2s %d %d %d", magic, &w, &h, &maxv) != 4 || std::strcmp(magic, "P5") != 0) {
    std::fclose(f); return m;
  }
  std::fgetc(f);
  m.resize(h, w);
  const size_t n = std::fread(m.data, 1, static_cast<size_t>(w) * h, f);
  std::fclose(f);
  if (n != static_cast<size_t>(w) * h) m = Mat();
  return m;
}
inline void cvtColor(const Mat& s, Mat& d, int) { d = s; }
inline void polylines(Mat&, const std::vector<Point>&, bool, const Scalar&, int = 1, int = 8) {}
inline void rectangle(Mat&, Point, Point, const Scalar&, int = 1, int = 8) {}
inline void rectangle(Mat&, Rect2f, const Scalar&, int = 1, int = 8) {}
inline void circle(Mat&, Point, int, const Scalar&, int = 1, int = 8) {}
inline void arrowedLine(Mat&, Point, Point, const Scalar&, int = 1, int = 8, int = 0, double = 0.3) {}
inline void resize(const Mat& s, Mat& d, Size) { d = s; }
inline void fillPoly(Mat&, const std::vector<std::vector<Point>>&, const Scalar&) {}
inline void fillPoly(Mat&, const std::vector<Point>&, const Scalar&) {}
inline void imshow(const std::string&, const Mat&) {}
inline int waitKey(int = 0) { return 0; }
}
using cv::CV_8UC3;
using cv::CV_8UC1;
using cv::Vec3b;
using cv::Point;
using cv::Size;
using cv::Scalar;
using cv::Mat;
using cv::Rect2f;
using cv::uchar;
