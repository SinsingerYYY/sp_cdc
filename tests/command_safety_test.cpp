#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "io/command.hpp"

namespace
{
int failures = 0;

void expect(bool condition, const std::string & message)
{
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  ++failures;
}

bool is_safe_command(const io::Command & command)
{
  return !command.control && !command.shoot && command.yaw == 0 && command.pitch == 0 &&
         command.horizon_distance == 0;
}
}  // namespace

int main()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  expect(is_safe_command(io::safe_command()), "safe command must not control or shoot");

  const io::Command normal{true, true, 0.5, -0.2};
  const auto kept = io::sanitize(normal);
  expect(
    kept.control && kept.shoot && kept.yaw == 0.5 && kept.pitch == -0.2,
    "finite command must pass through unchanged");

  expect(is_safe_command(io::sanitize({true, true, nan, 0})), "NaN yaw must be replaced");
  expect(is_safe_command(io::sanitize({true, true, 0, inf})), "Inf pitch must be replaced");
  expect(
    is_safe_command(io::sanitize({true, false, 0, 0, -inf})),
    "Inf horizon distance must be replaced");
  expect(
    is_safe_command(io::sanitize({false, true, 0.1, 0.1})),
    "shooting without control must be replaced");

  const auto idle = io::sanitize({false, false, 0, 0});
  expect(is_safe_command(idle), "idle command must remain the safe command");

  if (failures == 0) std::cout << "Command safety tests passed\n";
  return failures == 0 ? 0 : 1;
}
