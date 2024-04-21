// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver {
struct ReceivePacket {
    uint8_t header = 0xA5;
    uint8_t task_mode;
    uint8_t shoot_speed;
    float yaw;
    float roll;
    float pitch;
    uint8_t ender = 0xFF;
    // uint8_t robot_color : 1;
    // uint8_t task_mode : 2;
    // uint8_t reserved : 5;
    // float pitch;
    // float yaw;
    // float aim_x;
    // float aim_y;
    // float aim_z;
    // uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket {
    uint8_t header = 0xA5;
    float pitch;
    float yaw;
    float distance;
    uint8_t is_middle;
    uint8_t is_find_target;
    uint8_t is_find_buff;
    uint8_t ender = 0xFF;
    // uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> &data) {
    ReceivePacket packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
    return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket &data) {
    std::vector<uint8_t> packet(sizeof(SendPacket));
    std::copy(reinterpret_cast<const uint8_t *>(&data),
              reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket),
              packet.begin());
    return packet;
}

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__PACKET_HPP_
