#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

#include <cmath>
#include <string>
#include <vector>

namespace io
{
// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
};

// 失效安全命令：不接管云台、不开火，不跟随任何旧目标
inline Command safe_command() { return {false, false, 0, 0, 0}; }

// 判断命令是否可以安全发送：所有数值必须有限，且不允许在未控制时开火
inline bool is_safe(const Command & command)
{
  const bool finite = std::isfinite(command.yaw) && std::isfinite(command.pitch) &&
                      std::isfinite(command.horizon_distance);
  return finite && !(command.shoot && !command.control);
}

// 不安全的命令一律替换为失效安全命令
inline Command sanitize(const Command & command)
{
  return is_safe(command) ? command : safe_command();
}

}  // namespace io

#endif  // IO__COMMAND_HPP