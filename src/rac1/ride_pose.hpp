// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Ratchet's riding animation: our own key poses (ride_poses.inc, made by tools/poses), picked by
// what the board's logic plays, and on top of them secondary motion that keeps him alive: springs
// on his knees, arms, elbows, wrists, hips, back, head, ears, tail and face, driven by how the
// board moves (speed, turns, bumps, landings) and a slow balance sway, so nothing snaps or holds
// still. All of it is joint offsets over his standing pose. The whole body turns with the board in
// flips and spins, about a point at his middle.
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

enum class Look : uint8_t {
    stance, lean_left, lean_right, duck, jump, tuck, nosegrab, method, kickflip, land, kick, hit,
    count
};
const KeyPose& key_pose(Look look);

// The secondary-motion channels, in tools/poses/poses.py's order.
enum class Channel : uint8_t {
    crouch, arms_out, arms_fwd, arm_r_out, arm_l_out, elbows, wrists, hip_roll, spine_pump,
    head_yaw, head_nod, head_tilt, ear_l_up, ear_r_up, ear_l_back, ear_r_back, tail_side, tail_tip,
    tail_up, jaw, blink, brows, snarl,
    count
};

// What the board shows this tick.
struct LookTarget {
    Look pose = Look::stance;
    float lean = 0.0f;        // -1 (into a left turn) to 1 (right), on the stance only
    float flip = 0.0f;        // radians, the body and the board about his left (nose down > 0)
    float spin = 0.0f;        // radians, the body about up (the board has its own)
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
    // Back to standing, so the next ride blends in from his own pose.
    void reset();
    // One tick of `dt` seconds toward `target`, moving as `motion` says.
    void update(const LookTarget& target, const Motion& motion, float dt);
    // The offsets of this tick; flips and spins turn the root about `pivot` (model units).
    std::vector<JointOut> joints(const float pivot[3]) const;
    // The root's turn (x, y, z, w) of this tick, for what turns with him.
    void root_turn(float q[4]) const;
    // The flip of this tick (radians).
    float flip() const { return m_flip; }
    float channel(Channel c) const { return m_chan[static_cast<int>(c)].x; }

  private:
    Spring m_weight[static_cast<int>(Look::count)];
    Spring m_chan[static_cast<int>(Channel::count)];
    Spring m_lean;
    float m_flip = 0.0f;
    float m_spin = 0.0f;
    float m_time = 0.0f;
    float m_blink_at = 2.0f;
    uint32_t m_seed = 0x2545F491u;
    bool m_was_grounded = true;
};

} // namespace rac1
