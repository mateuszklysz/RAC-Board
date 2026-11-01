// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The GOAL types the Jak II code is written in, as the port spells them (THIRD_PARTY.md: the names
// and meanings are OpenGOAL's, jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team).
//
// Units are Jak II's throughout the board: 4096 units = 1 meter (`meters`), 65536 = one turn
// (`degrees`, "rotation units"), 300 = one second (`time-frame`). The adapter (src/rac1) converts at
// the edge, never inside the ported code.
#pragma once

#include <cstdint>
#include <cstring>

namespace jak {

// A vector of four floats, GOAL's `vector` (the w lane is kept: some routines read or write it).
struct Vec {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
};

// GOAL's `quaternion`: x, y, z the vector part, w the scalar.
struct Quat {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;
};

// GOAL's `matrix`: four row vectors, `(-> m vector 2)` is `m.v[2]`; vectors multiply as rows.
struct Mat {
    Vec v[4];
};

// GOAL's `time-frame`: ticks of 1/300 second.
using TimeFrame = int64_t;
constexpr TimeFrame seconds(double s) { return static_cast<TimeFrame>(s * 300.0); }
constexpr float meters(double m) { return static_cast<float>(m * 4096.0); }
constexpr float degrees(double d) { return static_cast<float>(d * (65536.0 / 360.0)); }

// A GOAL symbol used as a value ('air, 'ride, #f): compared by name. #f is the null symbol.
struct Symbol {
    const char* name = nullptr;
    constexpr Symbol() = default;
    constexpr Symbol(const char* n) : name(n) {}
    explicit operator bool() const { return name != nullptr; }
    bool operator==(Symbol o) const {
        return name == o.name || (name && o.name && std::strcmp(name, o.name) == 0);
    }
    bool operator!=(Symbol o) const { return !(*this == o); }
};

// A placeholder for a field of a GOAL type the port does not use (a process, a sound id, a handle).
struct Opaque {
    uint64_t value = 0;
};

// pat-surface (collide/pat-h.gc): the collision polygon's attributes in one word.
// bits 0-6 skip (noentity 0, nocamera 1, noedge 2, nogrind 3, nojak 4, noboard 5, nopilot 6),
// 7-9 mode, 10-15 material, 16 camera/nolineofsight, 18-23 event, 24-28 skip2 (probe 24, nomech 25,
// noproj 26, noendlessfall 27, noprobe 28).
struct PatSurface {
    uint32_t bits = 0;
    uint32_t mode() const { return (bits >> 7) & 7u; }
    uint32_t material() const { return (bits >> 10) & 63u; }
    uint32_t event() const { return (bits >> 18) & 63u; }
    bool noboard() const { return (bits >> 5) & 1u; }
    bool nogrind() const { return (bits >> 3) & 1u; }
    bool noedge() const { return (bits >> 2) & 1u; }
};

// pat-mode (pat-h.gc).
enum PatMode : uint32_t { kPatGround = 0, kPatWall = 1, kPatObstacle = 2, kPatHalfpipe = 3 };

// collide-status (collide-shape-h.gc), a bit field.
using CollideStatus = uint64_t;
namespace cstat {
constexpr CollideStatus on_surface = 1ull << 0, on_ground = 1ull << 1, touch_surface = 1ull << 2,
                        touch_wall = 1ull << 3, touch_ceiling = 1ull << 4, touch_actor = 1ull << 5,
                        on_special_surface = 1ull << 6, touch_edge = 1ull << 7,
                        no_touch = 1ull << 8, blocked = 1ull << 9, on_water = 1ull << 10,
                        impact_surface = 1ull << 11, touch_background = 1ull << 12,
                        stuck = 1ull << 13, touch_ceiling_sticky = 1ull << 14, glance = 1ull << 15,
                        probe_hit = 1ull << 16;
} // namespace cstat

// surface-flag (surface-h.gc), a bit field.
using SurfaceFlag = uint32_t;
namespace sflag {
constexpr SurfaceFlag look_around = 1u << 0, xz_local = 1u << 1, smooth_collision = 1u << 2,
                      no_turn_around = 1u << 3, turn_to_pad = 1u << 4, turn_to_vel = 1u << 5,
                      no_jump = 1u << 6, no_attack = 1u << 7, no_hands = 1u << 8,
                      no_feet = 1u << 9, check_edge = 1u << 10, air = 1u << 11,
                      attack = 1u << 12, duck = 1u << 13, momentum = 1u << 14,
                      turn_when_centered = 1u << 15, turn_to_alt = 1u << 16, spin = 1u << 17,
                      super = 1u << 18, gun_inactive = 1u << 19, gun_hide = 1u << 20,
                      gun_off = 1u << 21, gun_fast_exit = 1u << 22, gun_direct = 1u << 23,
                      gun_strafe = 1u << 24, gun_no_twist = 1u << 25, no_board = 1u << 27;
} // namespace sflag

// board-tricks (board-h.gc).
enum class BoardTricks : uint32_t {
    none = 0, board_spin = 1, board_boost = 2, board_flip = 3, board_rail = 4,
    board_rail_jump = 5, board_nosegrab = 6, board_method = 7, board_board_spin = 8,
    board_board_flip = 9, board_noseflip = 10, board_kickflip = 11, board_jump = 12,
    board_duck_jump = 13, board_quick_jump = 14, darkjak = 15, darkjak_bomb0 = 16,
    darkjak_bomb1 = 17, darkjak_giant = 18,
};

// ground-tween-info (game-h.gc).
struct GroundTweenInfo {
    uint8_t chan[3] = {};
    float blend[3] = {};
    uint32_t group[5] = {};
};

struct Surface;
struct CpadInfo;
struct CollideShapePrim;

// A surface's hooks: GOAL lambdas with `self` the target. The port passes the target explicitly.
struct Target;
using SurfaceHook = void (*)(Target& self);
// mult-hook: (arg0 the surface being built, arg1, arg2, arg3 the pass) as surface-mult! calls it.
using SurfaceMultHook = void (*)(Target& self, Surface& a0, const Surface& a1, const Surface& a2,
                                 int a3);

} // namespace jak
