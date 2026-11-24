#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  // 相机倒装（安装方向相反）时，把画面整体旋转 180°
  rotate_180_ = yaml["rotate_180"] ? yaml["rotate_180"].as<bool>() : false;

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    std::vector<double> wb{1.0, 1.0, 1.0};
    if (yaml["wb_gains"] && yaml["wb_gains"].size() == 3)
      wb = yaml["wb_gains"].as<std::vector<double>>();
    tools::logger()->info("[Camera] wb_gains = {} {} {}", wb[0], wb[1], wb[2]);
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, wb[0], wb[1], wb[2]);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
  if (rotate_180_ && !img.empty()) cv::rotate(img, img, cv::ROTATE_180);
}

}  // namespace io