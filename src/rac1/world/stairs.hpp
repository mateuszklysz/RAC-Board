// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Stairs in the game's collision. Jak II's levels flag the surfaces the board must not grind
// (pat nogrind); Ratchet & Clank's have no such flag, so Jak's edge search would take every step's
// nose for a ledge. A step is found by the shape of the floors: a flat piece with another a step's
// height above it and one a step's height below it, close across.
#pragma once

#include <vector>

namespace rac1 {

struct StairTri {
    float corners[3][3]; // the game's units and axes (z up)
    float up;            // the outward normal's z: 1 a floor, 0 a wall
};

// Which triangles belong to stairs: the steps' floors and the short walls up to them.
std::vector<bool> find_stairs(const std::vector<StairTri>& tris);

} // namespace rac1
