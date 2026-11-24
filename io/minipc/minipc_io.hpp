#ifndef IO__MINIPC__MINIPC_IO_HPP
#define IO__MINIPC__MINIPC_IO_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "serial/serial.h"

namespace io
{
// 哨兵 vision <-> 下位机 interface 层（rmpp，CherryUSB CDC）。
// 协议见 io/minipc/protocol.hpp（自瞄只处理 0xA1 下行 / 0xA2 上行）：
//   0xA1 视觉->板 17B: is_detect, is_fire, yaw, pitch(下正), distance
//   0xA2 板->视觉 24B: is_red, bullet_speed, qx, qy, qz, qw
//   0xA3/0xA4 为导航帧，自瞄不主动发 0xA3，只跳过 0xA4。
class MinipcIO
{
public:
  explicit MinipcIO(const std::string & config_path);
  ~MinipcIO();

  // 下位机 -> 视觉
  Eigen::Quaterniond q() const;  // 上行四元数（机体系 -> 世界系）
  bool is_red() const;           // 己方是否红方
  double bullet_speed() const;   // m/s
  bool connected() const;        // 100ms 内是否收到有效上行帧
  double imu_yaw() const;        // 由四元数解出的 yaw，deg
  double imu_pitch() const;      // 由四元数解出的 pitch，deg
  double imu_roll() const;       // 由四元数解出的 roll，deg

  // 视觉 -> 下位机（自瞄下行 0xA1）
  void send_auto_aim(
    bool is_detect, double yaw_deg, double pitch_deg, bool is_fire, double distance);
  // 导航下行 0xA3（自瞄不用；保留给测试）。x/y 固定 0。
  void send_chassis_speed(float vx, float vy, float yaw_rad, uint8_t action_flag);

private:
  void read_thread();
  bool read_byte(uint8_t & byte);
  void reconnect();
  void handle_down(const uint8_t * frame);

  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_{false};
  mutable std::mutex mutex_;

  Eigen::Quaterniond q_{1, 0, 0, 0};
  bool is_red_{true};
  double bullet_speed_{0};
  double imu_yaw_{0}, imu_pitch_{0}, imu_roll_{0};
  bool connected_{false};
  std::chrono::steady_clock::time_point last_rx_{};

  std::vector<uint8_t> buffer_;
  std::string com_port_;
};

}  // namespace io

#endif  // IO__MINIPC__MINIPC_IO_HPP
