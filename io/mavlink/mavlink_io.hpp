#ifndef IO__MAVLINK_IO_HPP
#define IO__MAVLINK_IO_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "serial/serial.h"

namespace io
{
// 与哨兵 C 板（rmpp）通过 USB CDC + MAVLink 2 通信，无 ROS。
// 协议见 rmpp/lib/mavlink/msg.xml：
//   C -> 视觉: imu(0)        yaw/pitch/roll (deg)
//              referee(1)    is_red, game_progress, stage_remain_time, bullet_speed
//              target_position(4)
//   视觉 -> C: auto_aim(2)   is_detect, yaw(deg), pitch(deg), is_fire, robot_id, wr(rpm), distance(m)
class MavlinkIO
{
public:
  explicit MavlinkIO(const std::string & config_path);
  ~MavlinkIO();

  // C 板 -> 视觉
  Eigen::Quaterniond q() const;  // 由 imu(yaw/pitch/roll) 得到，pitch 取负
  bool is_red() const;           // 己方是否红方（决定目标颜色）
  double bullet_speed() const;   // m/s
  bool connected() const;

  // 视觉 -> C 板
  void send_auto_aim(
    bool is_detect, double yaw_deg, double pitch_deg, bool is_fire, int robot_id, double wr,
    double distance);

private:
  void read_thread();
  bool read(uint8_t * buffer, size_t size);
  void reconnect();

  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_{false};
  mutable std::mutex mutex_;

  Eigen::Quaterniond q_{1, 0, 0, 0};
  bool is_red_{true};
  double bullet_speed_{0};
  bool connected_{false};

  std::string com_port_;

  void handle_imu(double yaw_deg, double pitch_deg, double roll_deg);
  void handle_referee(bool is_red, double bullet_speed);
};

}  // namespace io

#endif  // IO__MAVLINK_IO_HPP
