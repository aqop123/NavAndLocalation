// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef SERIAL_DRIVER__PACKET_HPP_
#define SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace serial_driver
{

struct ReceivePacket
{
  uint8_t header = '[';
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t game_time;  // (s) game time [0, 450]
  uint32_t timestamp;  // (ms) board time
  uint8_t ender = ']';
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = '[';
  float x;                 
  float y;                 
  float yaw;               
  uint8_t ender = ']';
} __attribute__((packed));

// 复制收到的信息
inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

// 复制要发送的数据
inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace serial_driver

#endif  // SERIAL_DRIVER__PACKET_HPP_
