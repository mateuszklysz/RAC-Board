// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Between Ratchet & Clank's world and Jak II's: units and axes.
//
// Jak II's world is shrunk to Ratchet's size: one game unit of Ratchet & Clank is 4096 / kJakScale of
// Jak II's units. kJakScale is Ratchet's shoulder height over Jak's (0.86 / 2.04 game units, Jak's
// from his skeleton's bind pose on the disc), so the board's speeds, jumps, collision and camera keep
// Jak II's proportions to the rider. Time is not scaled. Ratchet & Clank
// is z-up with x right and y forward; Jak II is y-up with x right and z forward. The map swaps y
// and z, so right, up and forward stay right, up and forward on screen: the board turns the way
// the stick points. The swap is a mirror, which turns a triangle's winding round (rac_board.cpp,
// RacWorld, keeps the corners and turns the normal outward).
#pragma once

#include <cmath>

#include "jak/basic_types.hpp"

namespace rac1 {

constexpr float kJakScale = 0.42f;           // Ratchet's size over Jak's
constexpr float kUnit = 4096.0f / kJakScale; // Jak II units in one game unit

inline jak::Vec to_jak_point(const float p[3]) {
    return jak::Vec{p[0] * kUnit, p[2] * kUnit, p[1] * kUnit, 1.0f};
}
inline jak::Vec to_jak_dir(const float d[3]) { return jak::Vec{d[0], d[2], d[1], 0.0f}; }
inline void to_game_point(const jak::Vec& v, float out[3]) {
    out[0] = v.x / kUnit;
    out[1] = v.z / kUnit;
    out[2] = v.y / kUnit;
}
inline void to_game_dir(const jak::Vec& v, float out[3]) {
    out[0] = v.x;
    out[1] = v.z;
    out[2] = v.y;
}

// The hero's yaw (radians about z, 0 facing +x) as a Jak II forward vector, and back.
inline jak::Vec yaw_to_jak_forward(float yaw) {
    return jak::Vec{std::cos(yaw), 0.0f, std::sin(yaw), 0.0f};
}
inline float jak_forward_to_yaw(const jak::Vec& f) { return std::atan2(f.z, f.x); }

} // namespace rac1
