#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "pattern.hpp"

// 从录制的视频 + 四元数日志中挑选“检测成功、清晰、且云台静止”的帧。
// 用法: extract_frames [video] [qlog] [outdir] [config] [min_sharp] [gap] [max_save] [still_deg] [still_win]

struct Q
{
  double w = 1, x = 0, y = 0, z = 0;
};

int main(int argc, char ** argv)
{
  const std::string video = argc > 1 ? argv[1] : "assets/calib.avi";
  const std::string qlog = argc > 2 ? argv[2] : "assets/calib_q.txt";
  const std::string outdir = argc > 3 ? argv[3] : "assets/img_with_q";
  const std::string cfg = argc > 4 ? argv[4] : "configs/calibration.yaml";
  const double min_sharp = argc > 5 ? std::stod(argv[5]) : 12.0;
  const int gap = argc > 6 ? std::stoi(argv[6]) : 8;
  const int max_save = argc > 7 ? std::stoi(argv[7]) : 40;
  const double still_deg = argc > 8 ? std::stod(argv[8]) : 0.6;  // 云台静止阈值(度)
  const int still_win = argc > 9 ? std::stoi(argv[9]) : 3;       // 前后各看几帧

  std::filesystem::create_directories(outdir);
  auto pattern = calibration::load_pattern(cfg);
  fmt::print("pattern {} {}x{} spacing {}mm\n", pattern.type, pattern.size.width, pattern.size.height, pattern.spacing_mm);

  std::vector<Q> qs;
  {
    std::ifstream f(qlog);
    int idx;
    Q q;
    while (f >> idx >> q.w >> q.x >> q.y >> q.z) {
      if (static_cast<int>(qs.size()) <= idx) qs.resize(idx + 1);
      qs[idx] = q;
    }
  }
  if (qs.empty()) {
    fmt::print("empty qlog {}\n", qlog);
    return 1;
  }

  auto ang_deg = [](const Q & a, const Q & b) {
    double d = std::abs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
    d = std::min(1.0, d);
    return 2.0 * std::acos(d) * 180.0 / M_PI;
  };
  auto is_still = [&](int i) {
    if (i - still_win < 0 || i + still_win >= static_cast<int>(qs.size())) return false;
    for (int k = 1; k <= still_win; ++k) {
      if (ang_deg(qs[i], qs[i - k]) > still_deg) return false;
      if (ang_deg(qs[i], qs[i + k]) > still_deg) return false;
    }
    return true;
  };
  fmt::print("still filter: <={} deg over +/-{} frames\n", still_deg, still_win);

  cv::VideoCapture cap(video);
  if (!cap.isOpened()) {
    fmt::print("cannot open {}\n", video);
    return 1;
  }

  cv::Mat img;
  int idx = -1, saved = 0, last = -100000;
  while (cap.read(img)) {
    idx++;
    if (idx >= static_cast<int>(qs.size())) break;
    if (idx - last < gap) continue;
    if (!is_still(idx)) continue;  // 云台在动，跳过

    std::vector<cv::Point2f> corners;
    if (!calibration::detect_pattern(img, pattern, corners)) continue;

    cv::Mat gray, lap;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar lap_mean, lap_std;
    cv::meanStdDev(lap, lap_mean, lap_std);
    const double sharp = lap_std[0] * lap_std[0];
    if (sharp < min_sharp) continue;

    last = idx;
    saved++;
    const auto jpg = fmt::format("{}/{}.jpg", outdir, saved);
    const auto txt = fmt::format("{}/{}.txt", outdir, saved);
    cv::imwrite(jpg, img);
    std::ofstream tf(txt);
    tf << qs[idx].w << ' ' << qs[idx].x << ' ' << qs[idx].y << ' ' << qs[idx].z;

    if (saved <= 5 || saved % 10 == 0)
      fmt::print("[{}] video frame {} sharp {:.1f}\n", saved, idx, sharp);

    if (saved >= max_save) break;
  }

  fmt::print("extracted {} frames from {} (scanned {} frames)\n", saved, video, idx + 1);
  return 0;
}
