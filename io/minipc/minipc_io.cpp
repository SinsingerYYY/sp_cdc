#include "minipc_io.hpp"

#include <cmath>
#include <stdexcept>

#include "protocol.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
constexpr std::size_t MAX_BUFFER = 256;
}  // namespace

MinipcIO::MinipcIO(const std::string & config_path)
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
    tools::logger()->error("[Minipc] Failed to open serial {}: {}", com_port_, e.what());
    exit(1);
  }

  tools::logger()->info("[Minipc] Opened {} (CDC, A1 17B / A2 24B)", com_port_);
  buffer_.reserve(MAX_BUFFER);
  thread_ = std::thread(&MinipcIO::read_thread, this);
}

MinipcIO::~MinipcIO()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  try {
    serial_.close();
  } catch (...) {
  }
}

Eigen::Quaterniond MinipcIO::q() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return q_;
}

bool MinipcIO::is_red() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_red_;
}

double MinipcIO::bullet_speed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return bullet_speed_;
}

bool MinipcIO::connected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_) return false;
  const auto elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_rx_).count();
  return elapsed < 0.1;
}

double MinipcIO::imu_yaw() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_yaw_;
}

double MinipcIO::imu_pitch() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_pitch_;
}

double MinipcIO::imu_roll() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_roll_;
}

// 自瞄上行 0xA2：is_red, bullet_speed, qx, qy, qz, qw
void MinipcIO::handle_down(const uint8_t * frame)
{
  const bool is_red = frame[1] != 0;
  const double bullet_speed = minipc::read_f32_le(frame + 2);
  const double qx = minipc::read_f32_le(frame + 6);
  const double qy = minipc::read_f32_le(frame + 10);
  const double qz = minipc::read_f32_le(frame + 14);
  const double qw = minipc::read_f32_le(frame + 18);

  Eigen::Quaterniond q(qw, qx, qy, qz);
  q.normalize();
  const Eigen::Vector3d ypr = tools::eulers(q, 2, 1, 0) * RAD2DEG;  // ZYX -> (yaw,pitch,roll)

  std::lock_guard<std::mutex> lock(mutex_);
  q_ = q;
  is_red_ = is_red;
  bullet_speed_ = bullet_speed;
  imu_yaw_ = ypr[0];
  imu_pitch_ = ypr[1];
  imu_roll_ = ypr[2];
  connected_ = true;
  last_rx_ = std::chrono::steady_clock::now();
}

// 自瞄下行 0xA1：is_detect, is_fire, yaw, pitch(下正), distance
void MinipcIO::send_auto_aim(
  bool is_detect, double yaw_deg, double pitch_deg, bool is_fire, double distance)
{
  uint8_t buf[minipc::AIM_DOWN_LEN];
  buf[0] = minipc::SOF_AIM_DOWN;
  buf[1] = is_detect ? 1 : 0;
  buf[2] = is_fire ? 1 : 0;
  minipc::write_f32_le(buf + 3, static_cast<float>(yaw_deg));
  // 新协议线上 pitch 即为低头正，视觉内部同样是低头正，直接发送
  minipc::write_f32_le(buf + 7, static_cast<float>(pitch_deg));
  minipc::write_f32_le(buf + 11, static_cast<float>(distance));
  minipc::write_u16_le(buf + 15, minipc::crc16_modbus(buf, minipc::AIM_DOWN_CRC_LEN));

  try {
    serial_.write(buf, sizeof(buf));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Minipc] write auto_aim failed: {}", e.what());
  }
}

// 导航下行 0xA3（自瞄不用，保留测试）：is_through_hole, vx, vy, yaw, x, y
void MinipcIO::send_chassis_speed(float vx, float vy, float yaw_rad, uint8_t action_flag)
{
  uint8_t buf[minipc::NAV_DOWN_LEN];
  buf[0] = minipc::SOF_NAV_DOWN;
  buf[1] = action_flag != 0 ? 1 : 0;
  minipc::write_f32_le(buf + 2, vx);
  minipc::write_f32_le(buf + 6, vy);
  minipc::write_f32_le(buf + 10, yaw_rad);
  minipc::write_f32_le(buf + 14, 0.0f);
  minipc::write_f32_le(buf + 18, 0.0f);
  minipc::write_u16_le(buf + 22, minipc::crc16_modbus(buf, minipc::NAV_DOWN_CRC_LEN));

  try {
    serial_.write(buf, sizeof(buf));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Minipc] write chassis_speed failed: {}", e.what());
  }
}

bool MinipcIO::read_byte(uint8_t & byte)
{
  try {
    return serial_.read(&byte, 1) == 1;
  } catch (const std::exception &) {
    return false;
  }
}

void MinipcIO::read_thread()
{
  tools::logger()->info("[Minipc] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    uint8_t byte;
    if (!read_byte(byte)) {
      if (++error_count > 5000) {
        error_count = 0;
        reconnect();
      }
      continue;
    }
    error_count = 0;
    buffer_.push_back(byte);

    while (true) {
      if (buffer_.empty()) break;
      const uint8_t * frame = buffer_.data();
      const uint8_t sof = frame[0];

      if (sof == minipc::SOF_AIM_UP) {
        if (buffer_.size() < static_cast<std::size_t>(minipc::AIM_UP_LEN)) break;
        const uint16_t crc = minipc::crc16_modbus(frame, minipc::AIM_UP_CRC_LEN);
        const uint16_t recv = minipc::read_u16_le(frame + minipc::AIM_UP_CRC_LEN);
        if (crc == recv) {
          handle_down(frame);
          buffer_.erase(buffer_.begin(), buffer_.begin() + minipc::AIM_UP_LEN);
        } else {
          buffer_.erase(buffer_.begin());  // 失步，滑动 1 字节
        }
      } else if (sof == minipc::SOF_NAV_UP) {
        if (buffer_.size() < static_cast<std::size_t>(minipc::NAV_UP_LEN)) break;
        const uint16_t crc = minipc::crc16_modbus(frame, minipc::NAV_UP_CRC_LEN);
        const uint16_t recv = minipc::read_u16_le(frame + minipc::NAV_UP_CRC_LEN);
        buffer_.erase(
          buffer_.begin(),
          buffer_.begin() + (crc == recv ? minipc::NAV_UP_LEN : 1));  // 导航上行：跳过
      } else {
        buffer_.erase(buffer_.begin());  // 未知帧头，滑动 1 字节重新对齐
      }
    }

    if (buffer_.size() > MAX_BUFFER) buffer_.erase(buffer_.begin());
  }

  tools::logger()->info("[Minipc] read_thread stopped.");
}

void MinipcIO::reconnect()
{
  tools::logger()->warn("[Minipc] reconnecting {}...", com_port_);
  try {
    serial_.close();
  } catch (...) {
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  try {
    serial_.open();
    tools::logger()->info("[Minipc] reconnected.");
  } catch (const std::exception & e) {
    tools::logger()->warn("[Minipc] reconnect failed: {}", e.what());
  }
}

}  // namespace io
