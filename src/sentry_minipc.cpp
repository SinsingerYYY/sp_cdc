#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/minipc/minipc_io.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono;

// 哨兵自瞄（无 ROS）：视觉 <-> 下位机 走 CherryUSB CDC + 固定长度协议。
// 下行 21B: yaw/pitch/roll + is_red + bullet_speed
// 上行 18B: is_detect + yaw/pitch + is_fire + distance

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

constexpr double RAD2DEG = 57.29577951308232;

int main(int argc, char * argv[])
{
  tools::Exiter exiter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto yaml = tools::load(config_path);
  const double bullet_speed_default =
    yaml["bullet_speed_default"] ? yaml["bullet_speed_default"].as<double>() : 20.0;

  const std::string plotter_host =
    yaml["plotter_host"] ? yaml["plotter_host"].as<std::string>() : "127.0.0.1";
  const int plotter_port = yaml["plotter_port"] ? yaml["plotter_port"].as<int>() : 9870;
  tools::Plotter plotter(plotter_host, plotter_port);

  std::filesystem::create_directories("logs");
  std::ofstream csv("logs/aim.csv");
  csv << "t,armor_x,armor_y,armor_z,armor_dist,aim_x,aim_y,aim_z,aim_dist,imu_yaw,imu_pitch,"
         "imu_roll,is_detect,cmd_yaw,cmd_pitch,spin_rpm,sel,plate_yaw_deg\n";
  const auto t_csv = std::chrono::steady_clock::now();

  io::MinipcIO minipc(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  const bool use_mpc = yaml["use_mpc"] ? yaml["use_mpc"].as<bool>() : false;
  std::unique_ptr<auto_aim::Planner> planner;
  if (use_mpc) planner = std::make_unique<auto_aim::Planner>(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  nlohmann::json data;

  while (!exiter.exit()) {
    camera.read(img, t);

    // 下位机姿态
    auto q = minipc.q();
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 由 is_red 决定敌方颜色：己方红 -> 敌方蓝
    tracker.set_enemy_color(minipc.is_red() ? auto_aim::Color::blue : auto_aim::Color::red);

    // 识别 -> 跟踪 -> 瞄准
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);

    double bullet_speed = minipc.bullet_speed();
    if (bullet_speed <= 0) bullet_speed = bullet_speed_default;  // 下位机未给弹速时用配置值

    io::Command command;
    if (use_mpc && planner) {
      std::optional<auto_aim::Target> tgt;
      if (!targets.empty()) tgt = targets.front();
      auto plan = planner->plan(tgt, bullet_speed);
      command.control = plan.control;
      command.shoot = plan.fire;
      command.yaw = plan.control ? plan.yaw : 0.0;
      command.pitch = plan.control ? plan.pitch : 0.0;
    } else {
      command = aimer.aim(targets, t, bullet_speed, true);
      command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
    }

    double spin_rpm = 0.0;
    if (!targets.empty()) spin_rpm = targets.front().ekf_x()[7] * 60.0 / (2.0 * M_PI);

    // 要打的瞄准点（world/P 系，m）
    Eigen::Vector3d aim_xyz{0, 0, 0};
    bool aim_ok = false;
    std::string sel = "none";
    double plate_yaw_deg = 0.0;
    if (use_mpc && planner) {
      aim_ok = command.control;
      if (aim_ok) {
        aim_xyz = planner->debug_xyza.head(3);
        plate_yaw_deg = planner->debug_xyza[3] * RAD2DEG;
      }
      sel = "mpc";
    } else {
      aim_ok = !targets.empty() && aimer.debug_aim_point.valid;
      if (aim_ok) aim_xyz = aimer.debug_aim_point.xyza.head(3);
      if (aimer.select_mode == auto_aim::Aimer::SelectMode::straight) sel = "straight";
      else if (aimer.select_mode == auto_aim::Aimer::SelectMode::spin) sel = "spin";
      plate_yaw_deg = aim_ok ? aimer.debug_aim_point.xyza[3] * RAD2DEG : 0.0;
    }

    double distance = 0;
    Eigen::Vector3d cam_pos{0, 0, 0};
    Eigen::Vector3d armor_w{0, 0, 0};
    std::string front_type = "-", front_name = "-";
    if (!armors.empty()) {
      distance = armors.front().ypd_in_world[2];
      cam_pos = armors.front().xyz_in_camera;  // 相机光学系：x右 y下 z前 (m)
      armor_w = armors.front().xyz_in_world;    // world/P 系 (m)
      front_type = auto_aim::ARMOR_TYPES[static_cast<int>(armors.front().type)];
      front_name = auto_aim::ARMOR_NAMES[static_cast<int>(armors.front().name)];
    }

    minipc.send_auto_aim(
      command.control, command.yaw * RAD2DEG, command.pitch * RAD2DEG, command.shoot, distance);

    // 调试输出
    data["armor_num"] = static_cast<int>(armors.size());
    data["target_num"] = static_cast<int>(targets.size());
    data["state"] = tracker.state();
    data["cmd_yaw_deg"] = command.yaw * RAD2DEG;
    data["cmd_pitch_deg"] = command.pitch * RAD2DEG;  // 线上低头正
    data["is_detect"] = command.control;
    data["is_fire"] = command.shoot;
    data["distance"] = distance;
    data["cam_x"] = cam_pos.x();
    data["cam_y"] = cam_pos.y();
    data["cam_z"] = cam_pos.z();
    data["aim_x"] = aim_xyz.x();
    data["aim_y"] = aim_xyz.y();
    data["aim_z"] = aim_xyz.z();
    data["aim_ok"] = aim_ok;
    data["select_mode"] = sel;
    data["plate_yaw_deg"] = plate_yaw_deg;
    data["armor_x"] = armor_w.x();
    data["armor_y"] = armor_w.y();
    data["armor_z"] = armor_w.z();
    data["armor_dist"] = distance;
    data["aim_dist"] = aim_ok ? aim_xyz.norm() : 0.0;
    data["imu_yaw"] = minipc.imu_yaw();
    data["imu_pitch"] = minipc.imu_pitch();
    data["imu_roll"] = minipc.imu_roll();
    data["spin_rpm"] = spin_rpm;
    data["bullet_speed"] = bullet_speed;
    plotter.plot(data);

    // 逐帧写 CSV 供离线画曲线
    {
      const double ts =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_csv).count();
      csv << fmt::format(
        "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"
        "{:.3f},{:.3f},{:.3f},{},{:.3f},{:.3f},{:.1f},{},{:.1f}\n",
        ts, armor_w.x(), armor_w.y(), armor_w.z(), distance, aim_xyz.x(), aim_xyz.y(),
        aim_xyz.z(), (aim_ok ? aim_xyz.norm() : 0.0), minipc.imu_yaw(), minipc.imu_pitch(),
        minipc.imu_roll(), (command.control ? 1 : 0), command.yaw * RAD2DEG,
        command.pitch * RAD2DEG, spin_rpm, sel, plate_yaw_deg);
      csv.flush();
    }

    // 1Hz 联调日志
    static auto last_log = std::chrono::steady_clock::now();
    if (tools::delta_time(std::chrono::steady_clock::now(), last_log) >= 1.0) {
      last_log = std::chrono::steady_clock::now();
      tools::logger()->info(
        "[sentry-minipc] is_red={} bullet={:.2f} conn={} "
        "imu=({:.2f},{:.2f},{:.2f}) q=({:.3f},{:.3f},{:.3f},{:.3f}) | "
        "armors={} targets={} state={} | detect={} fire={} cmd_yaw={:.2f} cmd_pitch={:.2f} "
        "dist={:.2f} spin={:.0f}rpm sel={} pyaw={:.1f} type={} name={} cam=({:.3f},{:.3f},{:.3f}) aim=({:.3f},{:.3f},{:.3f}) aim_ok={}",
        minipc.is_red(), bullet_speed, minipc.connected(), minipc.imu_yaw(),
        minipc.imu_pitch(), minipc.imu_roll(), q.w(), q.x(), q.y(), q.z(),
        static_cast<int>(armors.size()), static_cast<int>(targets.size()), tracker.state(),
        command.control, command.shoot, command.yaw * RAD2DEG, command.pitch * RAD2DEG, distance,
        spin_rpm, sel, plate_yaw_deg, front_type, front_name, cam_pos.x(), cam_pos.y(), cam_pos.z(), aim_xyz.x(),
        aim_xyz.y(), aim_xyz.z(), aim_ok);
    }
  }

  return 0;
}
