// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Stairs (rac1/world/stairs.*): the steps of a flight are found, a lone curb, a ramp and a ledge are not.
#include <vector>

#include "check.hpp"
#include "rac1/world/stairs.hpp"

using namespace rac1;

namespace {

// A flat floor piece from x0 to x1 (y 0 to 2) at height z.
StairTri floor_at(float x0, float x1, float z) { return StairTri{{{x0, 0.0f, z}, {x1, 0.0f, z}, {x1, 2.0f, z}}, 1.0f}; }

// A wall across y at x, from z0 up to z1.
StairTri wall_at(float x, float z0, float z1) { return StairTri{{{x, 0.0f, z0}, {x, 2.0f, z0}, {x, 2.0f, z1}}, 0.0f}; }

} // namespace

int main() {
    // A flight: the ground, three steps of 0.3 rise and 0.4 run, then a landing.
    std::vector<StairTri> flight = {floor_at(-5.0f, 0.0f, 0.0f), floor_at(0.0f, 0.4f, 0.3f), floor_at(0.4f, 0.8f, 0.6f),
                                    floor_at(0.8f, 1.2f, 0.9f),  floor_at(1.2f, 6.0f, 1.2f), wall_at(0.0f, 0.0f, 0.3f),
                                    wall_at(0.4f, 0.3f, 0.6f),   wall_at(1.2f, 0.9f, 1.2f)};
    std::vector<bool> s = find_stairs(flight);
    CHECK(!s[0]); // the ground
    CHECK(s[1] && s[2] && s[3]);
    CHECK(!s[4]);     // the landing: its nose stays a ledge
    CHECK(s[5] && s[6]);
    CHECK(!s[7]); // the wall up to the landing

    // A lone curb: one rise, no step.
    s = find_stairs({floor_at(-5.0f, 0.0f, 0.0f), floor_at(0.0f, 5.0f, 0.3f), wall_at(0.0f, 0.0f, 0.3f)});
    CHECK(!s[0] && !s[1] && !s[2]);

    // A ledge high over the floor, and a ramp: no stairs.
    s = find_stairs({floor_at(-5.0f, 0.0f, 0.0f), floor_at(0.0f, 5.0f, 2.0f), wall_at(0.0f, 0.0f, 2.0f),
                     StairTri{{{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.3f}, {1.0f, 2.0f, 0.3f}}, 0.95f}});
    CHECK(!s[0] && !s[1] && !s[2] && !s[3]);
    TEST_MAIN_END();
}
