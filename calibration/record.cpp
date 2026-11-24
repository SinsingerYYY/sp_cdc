#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/minipc/minipc_io.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

// 录视频 + 每帧（抽帧后）对应的 CDC 四元数，供离线挑帧标定。
// 用法: record [config] [video] [qlog] [every] [seconds]
//   every: 每 N 帧录一帧（相机 ~150fps，N=10 → 15fps）
//   seconds: 录多久（秒），结束后自动退出；期间可 Ctrl+C 提前结束

int main(int argc, char ** argv)
{
  const std::string config = argc > 1 ? argv[1] : "configs/sentry.yaml";
  const std::string video = argc > 2 ? argv[2] : "assets/calib.avi";
  const std::string qlog = argc > 3 ? argv[3] : "assets/calib_q.txt";
  const int every = argc > 4 ? std::stoi(argv[4]) : 10;
  const double seconds = argc > 5 ? std::stod(argv[5]) : 30.0;

  tools::Exiter exiter;
  io::MinipcIO minipc(config);
  io::Camera camera(config);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  camera.read(img, t);
  if (img.empty()) {
    tools::logger()->error("[record] empty frame");
    return 1;
  }

  const int fps = 150 / every;
  cv::VideoWriter writer(
    video, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps > 0 ? fps : 15, img.size());
  if (!writer.isOpened()) {
    tools::logger()->error("[record] cannot open writer {}", video);
    return 1;
  }
  std::ofstream qf(qlog);
  if (!qf.is_open()) {
    tools::logger()->error("[record] cannot open {}", qlog);
    return 1;
  }

  tools::logger()->info("[record] {} -> {} / {} (every={}, {:.0f}s)", config, video, qlog, every, seconds);

  int i = 0, out = 0;
  auto start = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    camera.read(img, t);
    if ((i++ % every) != 0) continue;

    Eigen::Quaterniond q = minipc.q();
    writer.write(img);
    qf << out << ' ' << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << '\n';
    out++;

    try {
      cv::Mat show;
      cv::resize(img, show, {}, 0.5, 0.5);
      cv::imshow("record", show);
      cv::waitKey(1);
    } catch (const cv::Exception &) {
    }

    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > seconds)
      break;
  }

  writer.release();
  qf.close();
  fmt::print("recorded {} frames -> {}\n", out, video);
  return 0;
}
