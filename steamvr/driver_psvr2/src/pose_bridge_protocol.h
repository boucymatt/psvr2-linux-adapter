// SPDX-License-Identifier: GPL-2.0
#pragma once

#include <cstdint>

namespace psvr2_bridge
{
static constexpr uint32_t kMagic = 0x32525650; // "PVR2"
static constexpr uint16_t kVersion = 1;
static constexpr uint16_t kLeftPort = 29761;
static constexpr uint16_t kRightPort = 29762;

#pragma pack(push, 1)
struct PosePacket
{
    uint32_t magic = kMagic;
    uint16_t version = kVersion;
    uint8_t hand = 0; // 0 left, 1 right
    uint8_t valid = 0;
    uint64_t sequence = 0;
    double position[3]{};
    double orientation[4]{1.0, 0.0, 0.0, 0.0}; // w, x, y, z
    double linear_velocity[3]{};
    double angular_velocity[3]{};
};
#pragma pack(pop)
} // namespace psvr2_bridge
