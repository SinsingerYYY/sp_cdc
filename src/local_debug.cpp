#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

static void print_usage(const char * name)
{
  fmt::print(
    "Usage: {} [options]\n"
    "  -c, --config-path <path>    yaml配置文件路径 (default: configs/local.yaml)\n"
    "  -v, --video <path>          视频路径（传入则回放视频，否则读相机）\n"
    "  -t, --tradition             使用传统方法识别\n"
    "  -d, --debug                 打开 imshow 可视化（默认关闭，仅终端打印帧率）\n"
    "  -D, --device <CPU|GPU>      覆盖配置中的推理设备（默认用配置文件里的 device）\n"
    "  -h, --help                  显示本帮助\n"
    "说明: 本地无下位机调试，姿态恒为单位四元数，不向下位机发送控制。\n",
    name);
}

int main(int argc, char * argv[])
{
  std::string config_path = "configs/local.yaml";
  std::string video_path;
  std::string device_override;
  bool use_tradition = false;
  bool debug_show = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    std::string key = arg, value;
    auto eq = arg.find('=');
    if (eq != std::string::npos) {
      key = arg.substr(0, eq);
      value = arg.substr(eq + 1);
    }

    auto need_value = [&]() -> std::string {
      if (!value.empty()) return value;
      if (i + 1 >= argc) return "";
      std::string nxt = argv[i + 1];
      if (nxt == "=" && i + 2 < argc) return std::string(argv[i += 2]);
      return std::string(argv[++i]);
    };

    if (key == "-h" || key == "--help" || key == "-?" || key == "--usage") {
      print_usage(argv[0]);
      return 0;
    } else if (key == "-c" || key == "--config-path") {
      config_path = need_value();
    } else if (key == "-v" || key == "--video") {
      video_path = need_value();
    } else if (key == "-t" || key == "--tradition") {
      use_tradition = true;
    } else if (key == "-d" || key == "--debug" || key == "--show") {
      debug_show = true;
    } else if (key == "--device" || key == "-D") {
      device_override = need_value();
    }
  }

  if (!std::filesystem::exists(config_path)) {
    fmt::print(stderr, "Error: config file not found: {}\n", config_path);
    return 1;
  }
  if (!video_path.empty() && !std::filesystem::exists(video_path)) {
    fmt::print(stderr, "Error: video file not found: {}\n", video_path);
    return 1;
  }

  // --device 覆盖配置中的推理设备（CPU / GPU），其余参数不变
  std::string effective_config = config_path;
  if (!device_override.empty()) {
    auto y = tools::load(config_path);
    y["device"] = device_override;
    effective_config = fmt::format("/tmp/sp_vision_local_{}.yaml", device_override);
    std::ofstream(effective_config) << YAML::Dump(y);
    tools::logger()->info("Device override: {} (config: {})", device_override, effective_config);
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::VideoCapture cap;
  bool use_quat = false;
  std::ifstream quat_file;
  if (!video_path.empty()) {
    cap.open(video_path);
    tools::logger()->info("Replaying video: {}", video_path);

    auto dot = video_path.find_last_of('.');
    auto text_path = (dot == std::string::npos ? video_path : video_path.substr(0, dot)) + ".txt";
    if (std::filesystem::exists(text_path)) {
      quat_file.open(text_path);
      use_quat = quat_file.is_open();
      tools::logger()->info("Using recorded quaternion: {}", text_path);
    }
  } else {
    tools::logger()->info("Opening camera from config: {}", effective_config);
  }
  if (!video_path.empty() && !cap.isOpened()) {
    tools::logger()->error("Failed to open input source!");
    return 1;
  }

  double video_frame_interval_us = 0.0;
  if (!video_path.empty()) {
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps > 1.0) video_frame_interval_us = 1e6 / fps;
  }

  // 相机（来自配置文件），仅在没有视频回放时创建，避免无相机时反复重试
  std::unique_ptr<io::Camera> camera;
  if (video_path.empty()) {
    camera = std::make_unique<io::Camera>(effective_config);
  }

  // debug 开关同时控制识别器内部的可视化
  auto_aim::YOLO yolo(effective_config, debug_show);
  auto_aim::Detector detector(effective_config, debug_show);
  auto_aim::Solver solver(effective_config);
  auto_aim::Tracker tracker(effective_config, solver);
  auto_aim::Aimer aimer(effective_config);
  auto_aim::Shooter shooter(effective_config);

  auto yaml_cfg = tools::load(effective_config);
  auto bullet_speed = yaml_cfg["bullet_speed"] ? yaml_cfg["bullet_speed"].as<double>() : 27.0;

  tools::logger()->info("Display (imshow): {}", debug_show ? "ON" : "OFF");

  // 本地无 IMU/云台，姿态恒为单位四元数 => 世界系 == 云台系
  const Eigen::Quaterniond identity_q = Eigen::Quaterniond::Identity();
  solver.set_R_gimbal2world(identity_q);

  cv::Mat img;
  int frame_count = 0;
  auto last_t = std::chrono::steady_clock::now();
  auto t0 = std::chrono::steady_clock::now();

  // 终端帧率统计（1 秒窗口）
  int win_frames = 0;
  double win_cam = 0, win_det = 0, win_trk = 0, win_aim = 0, win_show = 0;
  auto win_start = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    std::chrono::steady_clock::time_point t;

    auto c0 = std::chrono::steady_clock::now();
    if (!video_path.empty()) {
      cap.read(img);
    } else {
      camera->read(img, t);
    }
    if (img.empty()) {
      tools::logger()->warn("Empty frame, exit.");
      break;
    }
    auto c1 = std::chrono::steady_clock::now();

    if (!video_path.empty()) {
      t = std::chrono::steady_clock::now();
    }
    auto dt = tools::delta_time(t, last_t);
    last_t = t;

    if (use_quat) {
      double qt, qw, qx, qy, qz;
      if (quat_file >> qt >> qw >> qx >> qy >> qz) {
        solver.set_R_gimbal2world({qw, qx, qy, qz});
        t = t0 + std::chrono::microseconds(int(qt * 1e6));
      }
    } else if (video_frame_interval_us > 0.0) {
      t = t0 + std::chrono::microseconds(int(frame_count * video_frame_interval_us));
    }

    auto d0 = std::chrono::steady_clock::now();
    std::list<auto_aim::Armor> armors;
    if (use_tradition)
      armors = detector.detect(img, frame_count);
    else
      armors = yolo.detect(img, frame_count);
    auto d1 = std::chrono::steady_clock::now();

    // tracker.track 会按 enemy_color 原地过滤 armors，这里先记录原始识别数量
    const int armor_num = static_cast<int>(armors.size());

    auto targets = tracker.track(armors, t);
    auto d2 = std::chrono::steady_clock::now();

    auto command = aimer.aim(targets, t, bullet_speed, video_path.empty());
    auto d3 = std::chrono::steady_clock::now();

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    auto fire = shooter.shoot(command, aimer, targets, gimbal_pos);
    auto d4 = std::chrono::steady_clock::now();

    // 累加统计
    win_frames++;
    win_cam += tools::delta_time(c1, c0) * 1e3;
    win_det += tools::delta_time(d1, d0) * 1e3;
    win_trk += tools::delta_time(d2, d1) * 1e3;
    win_aim += tools::delta_time(d3, d2) * 1e3;
    (void)d4;

    // 发布 EKF / 调试数据到 PlotJuggler（UDP 9870），与 imshow 无关
    {
      nlohmann::json data;
      data["armor_num"] = armor_num;
      data["target_num"] = static_cast<int>(targets.size());
      data["state"] = tracker.state();
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["shoot"] = fire;
      data["fps"] = dt > 0 ? 1.0 / dt : 0.0;

      if (!targets.empty()) {
        auto target = targets.front();
        Eigen::VectorXd x = target.ekf_x();
        data["x"] = x[0];
        data["vx"] = x[1];
        data["y"] = x[2];
        data["vy"] = x[3];
        data["z"] = x[4];
        data["vz"] = x[5];
        data["a"] = x[6] * 57.3;
        data["w"] = x[7];
        data["r"] = x[8];
        data["l"] = x[9];
        data["h"] = x[10];
        data["last_id"] = target.last_id;

        data["residual_yaw"] = target.ekf().data.at("residual_yaw");
        data["residual_pitch"] = target.ekf().data.at("residual_pitch");
        data["residual_distance"] = target.ekf().data.at("residual_distance");
        data["residual_angle"] = target.ekf().data.at("residual_angle");
        data["nis"] = target.ekf().data.at("nis");
        data["nees"] = target.ekf().data.at("nees");
        data["nis_fail"] = target.ekf().data.at("nis_fail");
        data["nees_fail"] = target.ekf().data.at("nees_fail");
        data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
      }

      plotter.plot(data);
    }

    if (debug_show) {
      auto cx = img.cols / 2, cy = img.rows / 2;
      cv::line(img, {cx - 20, cy}, {cx + 20, cy}, {255, 255, 255}, 2);
      cv::line(img, {cx, cy - 20}, {cx, cy + 20}, {255, 255, 255}, 2);
      cv::circle(img, {cx, cy}, 6, {255, 255, 255}, 1);

      for (const auto & armor : armors) {
        tools::draw_points(img, armor.points, {0, 255, 0});
      }

      if (!targets.empty()) {
        auto target = targets.front();
        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          // EKF 预测框：蓝色
          tools::draw_points(img, image_points, {255, 0, 0});
        }

        auto aim_point = aimer.debug_aim_point;
        Eigen::Vector4d aim_xyza = aim_point.xyza;
        auto image_points =
          solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
        // 瞄准点：红色（无效时用暗红区分）
        if (aim_point.valid)
          tools::draw_points(img, image_points, {0, 0, 255});
        else
          tools::draw_points(img, image_points, {0, 0, 128});

        Eigen::VectorXd x = target.ekf_x();
        auto distance = std::sqrt(x[0] * x[0] + x[2] * x[2] + x[4] * x[4]);
        tools::draw_text(img, fmt::format("dist {:.2f} m", distance), {10, 120}, {255, 255, 255});
      }

      tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});
      tools::draw_text(img, fmt::format("fps {:.1f}", 1.0 / dt), {10, 60}, {255, 255, 255});
      tools::draw_text(
        img,
        fmt::format(
          "cmd yaw {:.2f} deg, pitch {:.2f} deg, shoot {}", command.yaw * 57.3,
          command.pitch * 57.3, fire ? "ON" : "OFF"),
        {10, 90}, fire ? cv::Scalar{0, 0, 255} : cv::Scalar{154, 50, 205});

      auto s0 = std::chrono::steady_clock::now();
      cv::resize(img, img, {}, 0.5, 0.5);
      cv::imshow("local_aim", img);
      auto key = cv::waitKey(1);
      auto s1 = std::chrono::steady_clock::now();
      win_show += tools::delta_time(s1, s0) * 1e3;
      if (key == 'q') break;
    }

    frame_count++;

    // 每秒打印一次终端帧率
    auto now = std::chrono::steady_clock::now();
    auto elapsed = tools::delta_time(now, win_start);
    if (elapsed >= 1.0) {
      tools::logger()->info(
        "[fps] {:.1f} | armors {} | targets {} | cam {:.1f}ms | det {:.1f}ms | trk {:.1f}ms | aim {:.1f}ms | show {:.1f}ms",
        win_frames / elapsed, armor_num, targets.size(), win_cam / win_frames, win_det / win_frames,
        win_trk / win_frames, win_aim / win_frames, win_show / win_frames);
      win_frames = 0;
      win_cam = win_det = win_trk = win_aim = win_show = 0;
      win_start = now;
    }
  }

  return 0;
}
