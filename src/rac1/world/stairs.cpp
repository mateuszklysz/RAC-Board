// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/world/stairs.hpp"

#include <algorithm>
#include <cmath>

namespace rac1 {

namespace {

// A step's rise in the game's units (Ratchet is about 1.5 tall), and how close two steps' floors
// come across.
constexpr float kRiseMin = 0.08f;
constexpr float kRiseMax = 0.6f;
constexpr float kGap = 0.4f;
constexpr float kFloor = 0.9f; // the up of a floor
constexpr float kWall = 0.3f;  // at most the up of a wall

struct Box {
    float lo[3], hi[3];
};

Box box_of(const StairTri& t) {
    Box b{};
    for (int k = 0; k < 3; ++k) {
        b.lo[k] = std::min({t.corners[0][k], t.corners[1][k], t.corners[2][k]});
        b.hi[k] = std::max({t.corners[0][k], t.corners[1][k], t.corners[2][k]});
    }
    return b;
}

// How far apart two boxes are across (x and y).
float gap(const Box& a, const Box& b) {
    const float dx = std::max({0.0f, a.lo[0] - b.hi[0], b.lo[0] - a.hi[0]});
    const float dy = std::max({0.0f, a.lo[1] - b.hi[1], b.lo[1] - a.hi[1]});
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

std::vector<bool> find_stairs(const std::vector<StairTri>& tris) {
    const size_t n = tris.size();
    std::vector<bool> stair(n, false);
    std::vector<Box> boxes(n);
    std::vector<size_t> floors;
    for (size_t i = 0; i < n; ++i) {
        boxes[i] = box_of(tris[i]);
        // Flat floors only: a ramp's slope is no step.
        if (tris[i].up >= kFloor && boxes[i].hi[2] - boxes[i].lo[2] < kRiseMin) floors.push_back(i);
    }
    const auto height = [&](size_t i) { return 0.5f * (boxes[i].lo[2] + boxes[i].hi[2]); };
    const auto step = [&](size_t i, size_t j) {
        const float rise = std::fabs(height(i) - height(j));
        return rise >= kRiseMin && rise <= kRiseMax && gap(boxes[i], boxes[j]) <= kGap;
    };
    // Whether each floor has a step neighbour above it and below it.
    std::vector<bool> above(n, false), below(n, false);
    for (size_t a = 0; a < floors.size(); ++a) {
        for (size_t b = a + 1; b < floors.size(); ++b) {
            const size_t i = floors[a], j = floors[b];
            if (!step(i, j)) continue;
            (height(j) > height(i) ? above[i] : below[i]) = true;
            (height(i) > height(j) ? above[j] : below[j]) = true;
        }
    }
    // A step has one of each (the floor below the stairs and the landing above them do not: a
    // landing's nose stays a ledge).
    for (size_t i : floors) stair[i] = above[i] && below[i];
    // The risers: short walls whose top meets a step's floor.
    for (size_t i = 0; i < n; ++i) {
        if (std::fabs(tris[i].up) > kWall || boxes[i].hi[2] - boxes[i].lo[2] > kRiseMax + kRiseMin) continue;
        for (size_t j : floors) {
            if (stair[j] && std::fabs(boxes[i].hi[2] - height(j)) < kRiseMin && gap(boxes[i], boxes[j]) <= kGap) {
                stair[i] = true;
                break;
            }
        }
    }
    return stair;
}

} // namespace rac1
