// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Ratchet's riding animation: our own key poses (ride_poses.inc, made by tools/poses), picked by
// what the board's logic plays and blended over time, as joint offsets over his standing pose. The
// whole body turns with the board in flips and spins, about a point at his middle.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rac1 {

// One joint's turn (x, y, z, w) on top of his standing pose.
struct JointTurn {
    uint32_t joint;
    float q[4];
};
// A key pose: its turned joints and how far his hips move (model units).
struct KeyPose {
    const JointTurn* turns;
    std::size_t count;
    float pelvis[3];
};

enum class Look : uint8_t {
    stance, lean_left, lean_right, duck, jump, tuck, nosegrab, method, kickflip, land, kick, hit,
    count
};
const KeyPose& key_pose(Look look);

// What the board shows this tick.
struct LookTarget {
    Look pose = Look::stance;
    float lean = 0.0f;        // -1 (into a left turn) to 1 (right), on the stance only
    float flip = 0.0f;        // radians, the body and the board about his left (nose down > 0)
    float spin = 0.0f;        // radians, the body about up (the board has its own)
};

// A joint offset as hero_set_joints takes it.
struct JointOut {
    uint32_t joint;
    float q[4];
    float t[3];
};

class RideAnimator {
  public:
    // Back to standing, so the next ride blends in from his own pose.
    void reset();
    // One tick of `dt` seconds toward `target`.
    void update(const LookTarget& target, float dt);
    // The offsets of this tick; flips and spins turn the root about `pivot` (model units).
    std::vector<JointOut> joints(const float pivot[3]) const;
    // The root's turn (x, y, z, w) of this tick, for what turns with him.
    void root_turn(float q[4]) const;
    // The flip of this tick (radians).
    float flip() const { return m_flip; }

  private:
    float m_weight[static_cast<int>(Look::count)] = {};
    float m_flip = 0.0f;
    float m_spin = 0.0f;
};

} // namespace rac1
