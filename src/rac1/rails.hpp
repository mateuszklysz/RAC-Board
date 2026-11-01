// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Ratchet & Clank's grind rails: the level's splines the hero grinds along with the Grindboots. The
// game keeps them as paths, with no collision; the board finds its rails as edges of the level's
// collision (Jak II's ride-edge), so each rail is given to it as a thin bar along its path.
#pragma once

#include <vector>

#include "jak/host.hpp"

namespace rac1 {

struct Rail {
    std::vector<jak::Vec> points; // game units, z up
    float lo[3], hi[3];           // its bounds
    bool loop = false;            // the last point joins the first
};

// The rails of the level loaded now (read once a level: the table moves with each level).
const std::vector<Rail>& rails();
// Adds the bars of the rails inside the box (Jak II's units) to `out`, up to `max`; returns how
// many it added.
int rail_triangles(const jak::Vec& lo, const jak::Vec& hi, jak::CollideTri* out, int max);
// Logs the level's spline table (trace).
void trace_rails();

} // namespace rac1
