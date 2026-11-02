// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/world/ground.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "jak/math/math.hpp"
#include "jak/movement.hpp"
#include "rac1/context.hpp"
#include "rac1/convert.hpp"

namespace rac1 {

namespace {

constexpr float kAbove = 0.5f; // how far over the feet a floor may be (game units)
constexpr float kBelow = 0.6f; // and under them

// Whether (x, y) is inside the triangle's shadow on the ground plane.
bool inside_xy(const openrac_collision_triangle& t, float x, float y) {
    const float* a = t.corners[0];
    const float* b = t.corners[1];
    const float* c = t.corners[2];
    const auto side = [](const float* p, const float* q, float x, float y) {
        return (q[0] - p[0]) * (y - p[1]) - (q[1] - p[1]) * (x - p[0]);
    };
    const float d1 = side(a, b, x, y), d2 = side(b, c, x, y), d3 = side(c, a, x, y);
    const bool neg = d1 < 0.0f || d2 < 0.0f || d3 < 0.0f;
    const bool pos = d1 > 0.0f || d2 > 0.0f || d3 > 0.0f;
    return !(neg && pos);
}

// The moby the board stood on last frame, and where it was.
struct Carried {
    int moby = -1;
    float position[3] = {};
    float yaw = 0.0f;
};
Carried g_on;
uint32_t g_traced_surface = ~0u;

bool moby_place(int slot, float position[3], float& yaw) {
    for (const openrac_moby& m : mod().moby_list()) {
        if (m.slot != slot) continue;
        if (m.flags & OPENRAC_MOBY_DEAD) return false;
        for (int k = 0; k < 3; ++k) position[k] = m.position[k];
        yaw = m.rotation[2];
        return true;
    }
    return false;
}

} // namespace

Ground ground_under(const float feet[3]) {
    Ground g;
    const float lo[3] = {feet[0] - 0.05f, feet[1] - 0.05f, feet[2] - kBelow};
    const float hi[3] = {feet[0] + 0.05f, feet[1] + 0.05f, feet[2] + kAbove};
    float best = 1e30f;
    const auto tris = mod().collision_triangles(lo, hi, 64);
    for (const openrac_collision_triangle& t : tris) {
        // A floor: its outward side up (the game's normal faces into the solid side).
        if (-t.normal[2] < 0.3f || !inside_xy(t, feet[0], feet[1])) continue;
        // The floor's height under the feet, on the triangle's plane.
        const float* a = t.corners[0];
        const float nz = t.normal[2];
        const float z = a[2] - (t.normal[0] * (feet[0] - a[0]) + t.normal[1] * (feet[1] - a[1])) / nz;
        const float d = feet[2] - z;
        if (d < -kAbove || d > kBelow || std::fabs(d) >= best) continue;
        best = std::fabs(d);
        g.found = true;
        g.surface = t.type & 0x1Fu;
        g.moby = t.moby;
    }
    if (g.found && g.surface != g_traced_surface) {
        g_traced_surface = g.surface;
        trace("ground surface " + std::to_string(g.surface) + " moby " + std::to_string(g.moby));
    }
    return g;
}

float fall_death_height() {
    // The level's fall height (NTSC-U D_L00_0015F638, PAL D_L00_0015F718), where the level keeps it.
    return mod().memory().read<float>(mod().relocate_level(0, ctx().pal ? 0x0015F718u : 0x0015F638u));
}

bool game_hazard(uint32_t surface) {
    // The ground kinds the hero's frame acts on (hero_frame_update.c, hero_idle_sequence.c): 0 a
    // water surface (he falls in and swims), 1 the death floor (death descent), 3, 11 and 13 the
    // liquids he sinks or drowns in.
    return surface == 0 || surface == 1 || surface == 3 || surface == 11 || surface == 13;
}

void carry(jak::Target& t) {
    if (g_on.moby < 0) return;
    float now[3], yaw;
    if (!moby_place(g_on.moby, now, yaw)) {
        g_on.moby = -1;
        return;
    }
    float d = yaw - g_on.yaw;
    while (d > 3.14159265f) d -= 6.2831853f;
    while (d < -3.14159265f) d += 6.2831853f;
    // Turned about the moby's centre by its turn, then moved with it.
    float p[3];
    to_game_point(t.control.trans, p);
    const float rx = p[0] - g_on.position[0], ry = p[1] - g_on.position[1];
    const float c = std::cos(d), s = std::sin(d);
    p[0] = now[0] + rx * c - ry * s;
    p[1] = now[1] + rx * s + ry * c;
    p[2] = p[2] + (now[2] - g_on.position[2]);
    const jak::Vec moved = to_jak_point(p);
    t.control.trans.x = moved.x;
    t.control.trans.y = moved.y;
    t.control.trans.z = moved.z;
    if (d != 0.0f) {
        // The game's turn about z is Jak II's about y the other way (the axes' mirror).
        jak::Quat r, q;
        jak::quaternion_vector_angle(r, jak::Vec{0.0f, 1.0f, 0.0f, 0.0f}, -d * (65536.0f / 6.2831853f));
        jak::quaternion_mul(q, r, t.control.quat);
        jak::quaternion_normalize(q);
        jak::set_quaternion(t, q);
    }
    g_on.moby = -1;
}

void note_ground(const jak::Target& t, const Ground& g) {
    g_on.moby = -1;
    if (!g.found || g.moby < 0 || !(t.control.status & jak::cstat::on_surface)) return;
    if (moby_place(g.moby, g_on.position, g_on.yaw)) g_on.moby = g.moby;
}

void forget_ground() {
    g_on.moby = -1;
    g_traced_surface = ~0u;
}

} // namespace rac1
