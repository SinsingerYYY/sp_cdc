#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/mavlink/mavlink_io.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono;

// 哨兵自瞄（无 ROS）：视觉 <-> C 板 直接走 USB CDC + MAVLink 2。
// 发送 auto_aim(2)：is_detect, yaw(deg), pitch(deg), is_fire, robot_id, wr(rpm), distance(m)
// 接收 imu(0)、referee(1)。

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

constexpr double RAD2DEG = 57.29577951308232;

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  io::MavlinkIO mavlink(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  nlohmann::json data;

  while (!exiter.exit()) {
    camera.read(img, t);

    // C 板姿态
    auto q = mavlink.q();
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 由 C 板 referee.is_red 决定敌方颜色：己方红 → 敌方蓝
    tracker.set_enemy_color(mavlink.is_red() ? auto_aim::Color::blue : auto_aim::Color::red);

    // 识别 -> 跟踪 -> 瞄准
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);

    double bullet_speed = mavlink.bullet_speed();
    if (bullet_speed <= 0) bullet_speed = 25.0;

    io::Command command = aimer.aim(targets, t, bullet_speed, true);
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    // 组 auto_aim 报文
    int robot_id = 0;
    double distance = 0;
    if (!armors.empty()) {
      robot_id = static_cast<int>(armors.front().name) + 1;
      distance = armors.front().ypd_in_world[2];
    }
    double wr = 0;
    if (!targets.empty()) {
      auto x = targets.front().ekf_x();
      wr = -x[7] * 60.0 / (2 * CV_PI);
    }

    mavlink.send_auto_aim(
      command.control, command.yaw * RAD2DEG, command.pitch * RAD2DEG, command.shoot, robot_id, wr,
      distance);

    // 调试输出
    data["armor_num"] = static_cast<int>(armors.size());
    data["target_num"] = static_cast<int>(targets.size());
    data["state"] = tracker.state();
    data["cmd_yaw_deg"] = command.yaw * RAD2DEG;
    data["cmd_pitch_deg"] = command.pitch * RAD2DEG;
    data["is_detect"] = command.control;
    data["is_fire"] = command.shoot;
    data["robot_id"] = robot_id;
    data["wr"] = wr;
    data["distance"] = distance;
    data["bullet_speed"] = bullet_speed;
    plotter.plot(data);

    // 1Hz 联调日志
    static auto last_log = std::chrono::steady_clock::now();
    if (tools::delta_time(std::chrono::steady_clock::now(), last_log) >= 1.0) {
      last_log = std::chrono::steady_clock::now();
      tools::logger()->info(
        "[sentry] is_red={} bullet={:.2f} conn={} q=({:.3f},{:.3f},{:.3f},{:.3f}) | "
        "armors={} targets={} state={} | detect={} fire={} yaw={:.2f} pitch={:.2f} id={} wr={:.1f} "
        "dist={:.2f}",
        mavlink.is_red(), mavlink.bullet_speed(), mavlink.connected(), q.w(), q.x(), q.y(), q.z(),
        static_cast<int>(armors.size()), static_cast<int>(targets.size()), tracker.state(),
        command.control, command.shoot, command.yaw * RAD2DEG, command.pitch * RAD2DEG, robot_id, wr,
        distance);
    }
  }

  return 0;
}
