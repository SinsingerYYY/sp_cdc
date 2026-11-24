#include "mavlink_io.hpp"

#include <cmath>

#include "lib/msg/mavlink.h"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
constexpr uint8_t SYSTEM_ID = 1;
constexpr uint8_t COMPONENT_ID = 1;
constexpr double DEG2RAD = M_PI / 180.0;
}  // namespace

MavlinkIO::MavlinkIO(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  com_port_ = yaml["com_port"] ? yaml["com_port"].as<std::string>() : std::string("/dev/c_board");

  try {
    serial_.setPort(com_port_);
    serial_.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Mavlink] Failed to open serial {}: {}", com_port_, e.what());
    exit(1);
  }

  tools::logger()->info("[Mavlink] Opened {} (MAVLink/CDC)", com_port_);
  thread_ = std::thread(&MavlinkIO::read_thread, this);
}

MavlinkIO::~MavlinkIO()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  try {
    serial_.close();
  } catch (...) {
  }
}

Eigen::Quaterniond MavlinkIO::q() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return q_;
}

bool MavlinkIO::is_red() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_red_;
}

double MavlinkIO::bullet_speed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return bullet_speed_;
}

bool MavlinkIO::connected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

void MavlinkIO::handle_imu(double yaw_deg, double pitch_deg, double roll_deg)
{
  // 与 potential-mavlink 一致：pitch 取负，再按 Rz(yaw)*Ry(pitch)*Rx(roll) 合成
  const double yaw = yaw_deg * DEG2RAD;
  const double pitch = -pitch_deg * DEG2RAD;
  const double roll = roll_deg * DEG2RAD;

  Eigen::Quaterniond q =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  std::lock_guard<std::mutex> lock(mutex_);
  q_ = q.normalized();
  connected_ = true;
}

void MavlinkIO::handle_referee(bool is_red, double bullet_speed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_red_ = is_red;
  bullet_speed_ = bullet_speed;
  connected_ = true;
}

void MavlinkIO::send_auto_aim(
  bool is_detect, double yaw_deg, double pitch_deg, bool is_fire, int robot_id, double wr,
  double distance)
{
  mavlink_message_t msg;
  mavlink_msg_auto_aim_pack(
    SYSTEM_ID, COMPONENT_ID, &msg, is_detect ? 1 : 0, static_cast<float>(yaw_deg),
    static_cast<float>(pitch_deg), is_fire ? 1 : 0, static_cast<uint8_t>(robot_id),
    static_cast<float>(wr), static_cast<float>(distance));

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  try {
    serial_.write(buf, len);
  } catch (const std::exception & e) {
    tools::logger()->warn("[Mavlink] write failed: {}", e.what());
  }
}

bool MavlinkIO::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    return false;
  }
}

void MavlinkIO::read_thread()
{
  tools::logger()->info("[Mavlink] read_thread started.");
  uint8_t byte;
  int error_count = 0;

  while (!quit_) {
    if (!read(&byte, 1)) {
      if (++error_count > 5000) {
        error_count = 0;
        reconnect();
      }
      continue;
    }
    error_count = 0;

    mavlink_message_t msg;
    mavlink_status_t status;
    if (!mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) continue;

    switch (msg.msgid) {
      case MAVLINK_MSG_ID_imu: {
        mavlink_imu_t imu;
        mavlink_msg_imu_decode(&msg, &imu);
        handle_imu(imu.yaw, imu.pitch, imu.roll);
        break;
      }
      case MAVLINK_MSG_ID_referee: {
        mavlink_referee_t referee;
        mavlink_msg_referee_decode(&msg, &referee);
        handle_referee(referee.is_red != 0, referee.bullet_speed);
        break;
      }
      default:
        break;
    }
  }

  tools::logger()->info("[Mavlink] read_thread stopped.");
}

void MavlinkIO::reconnect()
{
  tools::logger()->warn("[Mavlink] reconnecting {}...", com_port_);
  try {
    serial_.close();
  } catch (...) {
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  try {
    serial_.open();
    tools::logger()->info("[Mavlink] reconnected.");
  } catch (const std::exception & e) {
    tools::logger()->warn("[Mavlink] reconnect failed: {}", e.what());
  }
}

}  // namespace io
