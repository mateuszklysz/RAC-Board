// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Ratchet's riding animation: the clips' pose the board's logic plays (ride_clip.hpp), and on top
// of it what Jak has no joints for: springs on his ears, tail and face, driven by how the board
// moves (speed, turns, bumps, landings). All of it is joint offsets over his standing pose. The
// whole body turns with the board in spins, about a point at his middle.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rac1/ride_clip.hpp"

namespace rac1 {

// One joint of a motion channel: turned about `axis` (its own frame) by `degrees` at value 1.
struct ChannelTurn {
    uint32_t joint;
    float axis[3];
    float degrees;
};
struct Chan {
    const ChannelTurn* turns;
    std::size_t count;
};

// The secondary-motion channels, in tools/poses/export.py's order.
enum class Channel : uint8_t {
    ear_l_up, ear_r_up, ear_l_back, ear_r_back, tail_side, tail_tip,
    tail_up, jaw, blink, brows, snarl, upper_twist,
    count
};

// How the board moves this tick, in the game's units (a second), in his heading's frame.
struct Motion {
    float speed = 0.0f;       // along the ground
    float accel_fwd = 0.0f;   // speeding up > 0
    float accel_side = 0.0f;  // to his left > 0
    float vz = 0.0f;          // up > 0
    float az = 0.0f;
    float yaw_rate = 0.0f;    // radians a second, left > 0
    bool grounded = true;
};

// A joint offset as hero_set_joints takes it.
struct JointOut {
    uint32_t joint;
    float q[4];
    float t[3];
};

// A damped spring toward a target: x'' = w^2 (target - x) - 2 zeta w x'.
struct Spring {
    float x = 0.0f;
    float v = 0.0f;
    void step(float target, float hz, float zeta, float dt);
};

class RideAnimator {
  public:
    RideAnimator() { m_pose.clear(); }
    // Back to standing, so the next ride blends in from his own pose.
    void reset();
    // One tick of `dt` seconds: the clips' pose of this tick (mix), moving as `motion` says, turned
    // by the board's spin and his chest by its upper-body twist (radians about up).
    void update(const MixPose& pose, const Motion& motion, float spin, float upper_twist, float dt);
    // The offsets of this tick; spins turn the root about up.
    std::vector<JointOut> joints() const;
    // The spin of this tick (radians about up; the body turns the other way, see joints()).
    float spin() const { return m_spin; }
    float channel(Channel c) const { return m_chan[static_cast<int>(c)].x; }
    // The clips' pose of this tick (the board's place in it).
    const MixPose& pose() const { return m_pose; }

  private:
    MixPose m_pose{};
    Spring m_chan[static_cast<int>(Channel::count)];
    float m_spin = 0.0f;
    float m_time = 0.0f;
    float m_blink_at = 2.0f;
    uint32_t m_seed = 0x2545F491u;
    bool m_was_grounded = true;
};

} // namespace rac1
