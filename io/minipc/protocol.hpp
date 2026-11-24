#ifndef IO__MINIPC__PROTOCOL_HPP
#define IO__MINIPC__PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace io
{
namespace minipc
{

// 视觉 <-> 哨兵下位机（rmpp，CherryUSB CDC）的新协议（只实现自瞄帧）。
// 每帧 = 1 字节帧头 + 载荷 + CRC16（低字节先），帧头互不相同。
// 浮点为小端 IEEE754 float32；CRC-16/MODBUS（init 0xFFFF，多项式 0xA001），
// 校验范围 = 从帧头起、到 CRC 前的全部字节。
//
//   0xA1 视觉->板 自瞄下行 17B: is_detect, is_fire, yaw, pitch(下正), distance
//   0xA2 板->视觉 自瞄上行 24B: is_red, bullet_speed, qx, qy, qz, qw
//   0xA3 视觉->板 导航下行 24B: is_through_hole, vx, vy, yaw, x, y   （自瞄不用）
//   0xA4 板->视觉 导航上行 11B: x, y                                 （自瞄不用，仅跳过）

constexpr uint8_t SOF_AIM_DOWN = 0xA1;  // 视觉 -> 板
constexpr uint8_t SOF_AIM_UP = 0xA2;    // 板 -> 视觉
constexpr uint8_t SOF_NAV_DOWN = 0xA3;  // 视觉 -> 板（自瞄不用）
constexpr uint8_t SOF_NAV_UP = 0xA4;    // 板 -> 视觉（自瞄不用，仅跳过）

// 自瞄下行：1 帧头 + 1 is_detect + 1 is_fire + 4 yaw + 4 pitch + 4 distance + 2 crc
constexpr int AIM_DOWN_CRC_LEN = 15;  // 帧头起、CRC 前
constexpr int AIM_DOWN_LEN = AIM_DOWN_CRC_LEN + 2;  // 17

// 自瞄上行：1 帧头 + 1 is_red + 4 bullet_speed + 16 quat(x,y,z,w) + 2 crc
constexpr int AIM_UP_CRC_LEN = 22;
constexpr int AIM_UP_LEN = AIM_UP_CRC_LEN + 2;  // 24

// 导航下行：1 帧头 + 1 flag + 4*5 + 2 crc
constexpr int NAV_DOWN_CRC_LEN = 22;
constexpr int NAV_DOWN_LEN = NAV_DOWN_CRC_LEN + 2;  // 24

// 导航上行：1 帧头 + 4 x + 4 y + 2 crc
constexpr int NAV_UP_CRC_LEN = 9;
constexpr int NAV_UP_LEN = NAV_UP_CRC_LEN + 2;  // 11

inline uint16_t crc16_modbus(const uint8_t * data, std::size_t len)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint8_t>(data[i]);
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                           : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

inline void write_f32_le(uint8_t * p, float v)
{
  uint32_t u;
  std::memcpy(&u, &v, sizeof(u));
  p[0] = static_cast<uint8_t>(u);
  p[1] = static_cast<uint8_t>(u >> 8);
  p[2] = static_cast<uint8_t>(u >> 16);
  p[3] = static_cast<uint8_t>(u >> 24);
}

inline float read_f32_le(const uint8_t * p)
{
  uint32_t u = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  float v;
  std::memcpy(&v, &u, sizeof(v));
  return v;
}

inline void write_u16_le(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}

inline uint16_t read_u16_le(const uint8_t * p)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

}  // namespace minipc
}  // namespace io

#endif  // IO__MINIPC__PROTOCOL_HPP
