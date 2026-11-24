#include "exiter.hpp"

#include <csignal>
#include <stdexcept>

namespace tools
{
bool exit_ = false;
bool exiter_inited_ = false;

Exiter::Exiter()
{
  if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");
  auto handler = [](int) { exit_ = true; };
  std::signal(SIGINT, handler);
  // 同时处理 SIGTERM，保证 timeout / systemctl stop 能走正常析构，避免相机等设备未关闭
  std::signal(SIGTERM, handler);
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_; }

}  // namespace tools