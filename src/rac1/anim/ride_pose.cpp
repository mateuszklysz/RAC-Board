// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/anim/ride_pose.hpp"

#include <algorithm>
#include <cmath>

#include "rac1/anim/ride_poses.inc"

namespace rac1 {

namespace {

constexpr int kChannels = static_cast<int>(Channel::count);
constexpr float kTwoPi = 6.2831853f;
// The highest joint the clips turn, plus one.
constexpr uint32_t kJoints = kPoseJoints;

void qmul(const float a[4], const float b[4], float out[4]) {
    const float r[4] = {a[3] * b[0] + b[3] * a[0] + (a[1] * b[2] - a[2] * b[1]),
                        a[3] * b[1] + b[3] * a[1] + (a[2] * b[0] - a[0] * b[2]),
                        a[3] * b[2] + b[3] * a[2] + (a[0] * b[1] - a[1] * b[0]),
                        a[3] * b[3] - (a[0] * b[0] + a[1] * b[1] + a[2] * b[2])};
    std::copy(r, r + 4, out);
}

} // namespace

static_assert(sizeof(poses::kChannels) / sizeof(poses::kChannels[0]) == kChannels,
              "one channel table per rac1::Channel");

void Spring::step(float target, float hz, float zeta, float dt) {
    const float w = kTwoPi * hz;
    const float a = w * w * (target - x) - 2.0f * zeta * w * v;
    v += a * dt;
    x += v * dt;
}

void RideAnimator::reset() {
    *this = RideAnimator();
}

void RideAnimator::update(const MixPose& pose, const Motion& m, float spin, float upper_twist, float dt) {
    // The clips' pose, followed closely rather than copied: a clip that starts without one of Jak's
    // blends (a one-frame jump, the first clip after his own standing pose) flows in instead of
    // snapping. Legs and board ease alike, so his feet stay on it.
    if (dt > 0.0f) {
        const float a = 1.0f - std::exp(-dt / 0.04f);
        for (int j = 0; j < kPoseJoints; ++j) {
            if (!pose.used[j] && !m_pose.used[j]) continue;
            const float id[4] = {0.0f, 0.0f, 0.0f, 1.0f};
            nlerp(m_pose.q[j], pose.used[j] ? pose.q[j] : id, a);
            m_pose.used[j] = true;
        }
        for (int c = 0; c < 3; ++c) m_pose.hips[c] += a * (pose.hips[c] - m_pose.hips[c]);
        blend_board(m_pose.board, pose.board, a);
    } else {
        m_pose = pose;
    }
    m_spin = spin;
    // The upper-body joint-mod's twist, as the logic sets it (degrees on the channel).
    m_chan[static_cast<int>(Channel::upper_twist)].x = upper_twist * (360.0f / kTwoPi);
    if (dt <= 0.0f) return;
    m_time += dt;
    const float t = m_time;
    const auto wave = [t](float hz, float phase) { return std::sin(kTwoPi * hz * t + phase); };
    const auto clampf = [](float x, float lo, float hi) { return std::clamp(x, lo, hi); };

    // Ears, tail and face: Jak has none of them, so they move on their own: blown by the speed,
    // thrown by turns, bumps, landings and the fall.
    const float wind = clampf(m.speed / 18.0f, 0.0f, 1.0f);
    const float bump = clampf(-m.az * 0.012f, -0.6f, 0.9f);
    const float turn = clampf(m.yaw_rate * 0.4f, -1.0f, 1.0f);
    const bool landed = m.grounded && !m_was_grounded;
    const bool took_off = !m.grounded && m_was_grounded && m.vz > 1.0f;
    m_was_grounded = m.grounded;
    Spring* c = m_chan;
    auto ch = [c](Channel k) -> Spring& { return c[static_cast<int>(k)]; };
    if (landed) {
        const float hit = clampf(-m.vz * 0.3f, 1.0f, 6.0f);
        ch(Channel::ear_l_up).v -= 0.9f * hit;
        ch(Channel::ear_r_up).v -= 0.8f * hit;
        ch(Channel::tail_up).v -= 0.6f * hit;
        ch(Channel::jaw).v += 0.3f * hit;
    }
    if (took_off) {
        const float pop = clampf(m.vz * 0.3f, 1.0f, 4.0f);
        ch(Channel::ear_l_up).v += 0.8f * pop;
        ch(Channel::ear_r_up).v += 0.7f * pop;
        ch(Channel::tail_up).v += 0.6f * pop;
        ch(Channel::jaw).v += 0.3f * pop;
    }
    // Ears: blown back by the speed, floppy, thrown by the turns, the bumps and the fall.
    ch(Channel::ear_l_back).step(0.9f * wind + 0.3f * turn, 2.7f, 0.2f, dt);
    ch(Channel::ear_r_back).step(0.9f * wind - 0.3f * turn, 3.0f, 0.2f, dt);
    const float fall = clampf(-m.vz * 0.04f, -1.0f, 1.0f);
    ch(Channel::ear_l_up).step(-0.4f * wind + 0.5f * fall - bump + 0.1f * wave(0.33f, 0.0f), 2.6f, 0.18f, dt);
    ch(Channel::ear_r_up).step(-0.4f * wind + 0.5f * fall - bump + 0.1f * wave(0.29f, 1.0f), 2.9f, 0.18f, dt);
    // Tail: swings out of the turn, waves, its tip behind its root, lifts with the speed.
    ch(Channel::tail_side).step(-turn + 0.3f * wave(0.65f, 0.0f), 2.0f, 0.25f, dt);
    ch(Channel::tail_tip).step(-0.3f * ch(Channel::tail_side).v + 0.35f * wave(0.65f, -1.3f), 2.6f, 0.22f, dt);
    ch(Channel::tail_up).step(0.5f * wind + 0.15f * wave(0.4f, 0.9f), 2.2f, 0.25f, dt);
    // Face: the clips carry it; the springs only answer the landing. His model has no eyelids
    // (joints 19 and 20 turn the irises), so he does not blink.
    ch(Channel::jaw).step(0.0f, 4.5f, 0.55f, dt);
    ch(Channel::snarl).step(0.0f, 4.0f, 0.6f, dt);
    ch(Channel::brows).step(0.0f, 3.5f, 0.5f, dt);
}

std::vector<JointOut> RideAnimator::joints() const {
    const float* hips = m_pose.hips;
    std::vector<JointOut> out;
    // The spin: the root turned about up (z) through his feet.
    if (m_spin != 0.0f) out.push_back(JointOut{0, {0.0f, 0.0f, std::sin(m_spin * 0.5f), std::cos(m_spin * 0.5f)}, {}});
    // The channels' turns, per joint, after the pose's (in the joint's frame as posed).
    float extra[kJoints][4];
    bool moved[kJoints] = {};
    for (int c = 0; c < kChannels; ++c) {
        const float v = m_chan[c].x;
        if (std::fabs(v) < 1e-4f) continue;
        const Chan& ch = poses::kChannels[c];
        for (std::size_t n = 0; n < ch.count; ++n) {
            const ChannelTurn& turn = ch.turns[n];
            if (turn.joint >= kJoints) continue;
            const float half = 0.5f * turn.degrees * v * (kTwoPi / 360.0f);
            const float s = std::sin(half);
            const float q[4] = {turn.axis[0] * s, turn.axis[1] * s, turn.axis[2] * s, std::cos(half)};
            if (!moved[turn.joint]) {
                std::copy(q, q + 4, extra[turn.joint]);
                moved[turn.joint] = true;
            } else {
                qmul(extra[turn.joint], q, extra[turn.joint]);
            }
        }
    }

    for (uint32_t j = 1; j < kJoints; ++j) {
        if (!m_pose.used[j] && !moved[j] && j != 1) continue;
        float q[4] = {m_pose.q[j][0], m_pose.q[j][1], m_pose.q[j][2], m_pose.q[j][3]};
        if (moved[j]) qmul(q, extra[j], q);
        JointOut o{j, {q[0], q[1], q[2], q[3]}, {}};
        if (j == 1) std::copy(hips, hips + 3, o.t);
        out.push_back(o);
    }
    return out;
}

} // namespace rac1
