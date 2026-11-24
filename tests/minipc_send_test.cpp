#include <chrono>
#include <thread>

#include "io/minipc/minipc_io.hpp"

int main(int argc, char ** argv)
{
  io::MinipcIO io(argv[1]);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  io.send_auto_aim(true, 12.34, -5.67, true, 3.21);
  io.send_auto_aim(false, 100.0, 10.0, false, 8.5);
  io.send_chassis_speed(1.5f, -0.5f, 0.785f, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  return 0;
}
