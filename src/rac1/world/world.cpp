// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/world/world.hpp"

#include <cstring>
#include <vector>

#include "rac1/context.hpp"
#include "rac1/convert.hpp"
#include "rac1/world/rails.hpp"
#include "rac1/world/stairs.hpp"

namespace rac1 {

namespace {
constexpr uint32_t kPatNogrind = 0x8u; // pat-surface nogrind
} // namespace

int World::triangles(const jak::Vec& lo, const jak::Vec& hi, jak::CollideTri* out, int max) {
    // The swap keeps each axis's min a min, so the box maps corner to corner.
    const float glo[3] = {lo.x / kUnit, lo.z / kUnit, lo.y / kUnit};
    const float ghi[3] = {hi.x / kUnit, hi.z / kUnit, hi.y / kUnit};
    const std::vector<openrac_collision_triangle> tris = mod().collision_triangles(glo, ghi, max);
    std::vector<StairTri> shapes(tris.size());
    for (size_t i = 0; i < tris.size(); ++i) {
        std::memcpy(shapes[i].corners, tris[i].corners, sizeof shapes[i].corners);
        shapes[i].up = -tris[i].normal[2];
    }
    const std::vector<bool> stairs = find_stairs(shapes);
    int n = 0;
    for (const openrac_collision_triangle& t : tris) {
        if (n >= max) break;
        jak::CollideTri& c = out[n++];
        // The game's corners turn the other way round from Jak II's: its normal, (b - a) x (c - a),
        // faces into the solid side. Mirrored into Jak II's axes (convert.hpp) the same order gives
        // the outward normal by Jak II's cross product, and the normal is turned outward.
        c.vertex[0] = to_jak_point(t.corners[0]);
        c.vertex[1] = to_jak_point(t.corners[1]);
        c.vertex[2] = to_jak_point(t.corners[2]);
        const float out_normal[3] = {-t.normal[0], -t.normal[1], -t.normal[2]};
        c.normal = to_jak_dir(out_normal);
        // Every surface is ground to Jak II's code: it tells walls from floors by their slope.
        c.pat = jak::PatSurface{};
        // The steps of stairs are no ledges to grind (Jak II's levels flag such surfaces nogrind).
        if (stairs[&t - tris.data()]) c.pat.bits |= kPatNogrind;
    }
    // The game's grind rails, which have no collision: a bar along each for the board to grind.
    return n + rail_triangles(lo, hi, out + n, max - n);
}

} // namespace rac1
