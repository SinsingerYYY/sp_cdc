#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/minipc/minipc_io.hpp"
#include "pattern.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// 与 capture.cpp 相同，但四元数来自 CDC（rmpp / mc02）而不是 CAN 的 C 板。

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 相机/CDC 配置 }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & pattern_config,
  const std::string & output_folder)
{
  io::MinipcIO minipc(config_path);
  io::Camera camera(config_path);
  auto pattern = calibration::load_pattern(pattern_config);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  tools::logger()->info("等待 CDC 姿态... conn={}", minipc.connected());

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = minipc.q();

    // 在图像上显示欧拉角，用来判断 imuabs 系的 xyz 正方向
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> centers_2d;
    auto success = calibration::detect_pattern(img, pattern, centers_2d);
    cv::drawChessboardCorners(img_with_ypr, pattern.size, centers_2d, success);
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);

    // 按“s”保存图片和对应四元数，按“q”退出程序
    // Qt 后端自带 Ctrl+S 保存快捷键，会抛异常，这里捕获掉，不影响主循环
    int key = -1;
    try {
      cv::imshow("Press s to save, q to quit", img_with_ypr);
      key = cv::waitKey(1);
    } catch (const cv::Exception & e) {
      tools::logger()->warn("[capture] imshow/waitKey: {}", e.what());
      key = -1;
    }
    if (key == 'q' || key == 'Q')
      break;
    else if (key != 's' && key != 'S' && key != 19)
      continue;

    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  const std::string pattern_config = "configs/calibration.yaml";
  const std::string output_folder = "assets/img_with_q";

  std::filesystem::create_directory(output_folder);

  tools::logger()->info("config={} pattern={} output={}", config_path, pattern_config, output_folder);
  try {
    capture_loop(config_path, pattern_config, output_folder);
  } catch (const std::exception & e) {
    tools::logger()->error("[capture] exception: {}", e.what());
  }

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
