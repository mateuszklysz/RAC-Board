// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/ride_pose.hpp"

#include <algorithm>
#include <cmath>

#include "rac1/ride_poses.inc"

namespace rac1 {

namespace {

constexpr int kLooks = static_cast<int>(Look::count);
constexpr int kChannels = static_cast<int>(Channel::count);
constexpr float kTwoPi = 6.2831853f;
// How far his hips drop with the knees' channel (model units).
constexpr float kCrouchDrop = 0.35f;
// The highest joint the poses turn, plus one.
constexpr uint32_t kJoints = 128;

void qmul(const float a[4], const float b[4], float out[4]) {
    const float r[4] = {a[3] * b[0] + b[3] * a[0] + (a[1] * b[2] - a[2] * b[1]),
                        a[3] * b[1] + b[3] * a[1] + (a[2] * b[0] - a[0] * b[2]),
                        a[3] * b[2] + b[3] * a[2] + (a[0] * b[1] - a[1] * b[0]),
                        a[3] * b[3] - (a[0] * b[0] + a[1] * b[1] + a[2] * b[2])};
    std::copy(r, r + 4, out);
}

void rotate(const float q[4], const float v[3], float out[3]) {
    const float p[4] = {v[0], v[1], v[2], 0.0f};
    const float c[4] = {-q[0], -q[1], -q[2], q[3]};
    float t[4], r[4];
    qmul(q, p, t);
    qmul(t, c, r);
    std::copy(r, r + 3, out);
}

} // namespace

const KeyPose& key_pose(Look look) {
    static const KeyPose* const kPoses[kLooks] = {
        &poses::stance, &poses::lean_left, &poses::lean_right, &poses::duck, &poses::jump,
        &poses::tuck,   &poses::nosegrab,  &poses::method,     &poses::kickflip, &poses::land,
        &poses::kick,   &poses::hit};
    return *kPoses[static_cast<int>(look)];
}

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

void RideAnimator::update(const LookTarget& target, const Motion& m, float dt) {
    if (dt <= 0.0f) return;
    m_time += dt;
    const float t = m_time;
    const auto wave = [t](float hz, float phase) { return std::sin(kTwoPi * hz * t + phase); };
    const auto clampf = [](float x, float lo, float hi) { return std::clamp(x, lo, hi); };

    // The poses: each weight on a spring, a little past and back (follow-through), not a snap.
    float want[kLooks] = {};
    m_lean.step(std::clamp(target.lean, -1.0f, 1.0f), 2.2f, 0.7f, dt);
    const float lean = std::clamp(m_lean.x, -1.0f, 1.0f);
    if (target.pose == Look::stance) {
        want[static_cast<int>(Look::stance)] = 1.0f - std::fabs(lean);
        want[static_cast<int>(lean < 0.0f ? Look::lean_left : Look::lean_right)] = std::fabs(lean);
    } else {
        want[static_cast<int>(target.pose)] = 1.0f;
    }
    for (int i = 0; i < kLooks; ++i) {
        m_weight[i].step(want[i], 3.6f, 0.62f, dt);
        m_weight[i].x = std::clamp(m_weight[i].x, -0.1f, 1.15f);
    }
    m_flip = target.flip;
    m_spin = target.spin;

    const Look pose = target.pose;
    const bool air = !m.grounded;
    const bool trick = pose == Look::nosegrab || pose == Look::method || pose == Look::kickflip ||
                       target.flip != 0.0f;
    const float wind = clampf(m.speed / 18.0f, 0.0f, 1.0f);
    const float bump = clampf(-m.az * 0.012f, -0.6f, 0.9f);

    // A landing: the knees take it, arms and ears drop and come back, he blinks.
    const bool landed = m.grounded && !m_was_grounded;
    m_was_grounded = m.grounded;
    Spring* c = m_chan;
    auto ch = [c](Channel k) -> Spring& { return c[static_cast<int>(k)]; };
    if (landed) {
        const float hit = clampf(-m.vz * 0.35f, 1.0f, 7.0f);
        ch(Channel::crouch).v += hit;
        ch(Channel::arms_out).v += 0.6f * hit;
        ch(Channel::head_nod).v += 0.5f * hit;
        ch(Channel::ear_l_up).v -= 0.9f * hit;
        ch(Channel::ear_r_up).v -= 0.8f * hit;
        ch(Channel::tail_up).v -= 0.6f * hit;
        ch(Channel::jaw).v += 0.4f * hit;
        m_blink_at = std::min(m_blink_at, t);
    }

    // Knees: a soft suspension under the bumps, a slow balance in them.
    ch(Channel::crouch).step((air ? -0.15f : 0.12f + 0.07f * wave(0.5f, 0.0f)) + bump, 3.0f, 0.32f,
                             dt);
    // Arms: out for balance (more in turns and in the air), each its own sway, back with the
    // speed-up; elbows and wrists trail them.
    ch(Channel::arms_out)
        .step(0.12f * wave(0.45f, 0.3f) + 0.45f * std::fabs(lean) + (air ? 0.55f : 0.0f) +
                  clampf(m.az * 0.006f, -0.3f, 0.6f),
              2.1f, 0.3f, dt);
    ch(Channel::arm_r_out).step(0.3f * wave(0.37f, 1.0f) + 0.55f * lean, 1.7f, 0.38f, dt);
    ch(Channel::arm_l_out).step(0.3f * wave(0.41f, 0.0f) - 0.55f * lean, 1.9f, 0.38f, dt);
    ch(Channel::arms_fwd)
        .step(clampf(-m.accel_fwd * 0.03f, -0.9f, 0.9f) + 0.12f * wave(0.9f, 2.0f) +
                  (air ? 0.25f : 0.0f),
              1.9f, 0.33f, dt);
    ch(Channel::elbows)
        .step(0.3f - 0.13f * ch(Channel::arms_out).v + 0.12f * wave(0.6f, 1.4f) +
                  (pose == Look::duck ? 0.5f : 0.0f),
              2.8f, 0.3f, dt);
    ch(Channel::wrists).step(-0.18f * ch(Channel::elbows).v + 0.15f * wave(0.8f, 0.5f), 3.6f, 0.25f,
                             dt);
    // Hips and back: roll into the turn, pump with the knees.
    ch(Channel::hip_roll).step(0.8f * lean + 0.12f * wave(0.3f, 0.7f), 2.4f, 0.5f, dt);
    ch(Channel::spine_pump)
        .step(0.7f * ch(Channel::crouch).x + (air ? -0.3f : 0.0f) + 0.08f * wave(0.5f, 1.8f), 2.8f,
              0.42f, dt);
    // Head: looks into the turn, keeps level against the lean, nods with the knees.
    ch(Channel::head_yaw)
        .step(clampf(m.yaw_rate * 0.55f, -1.3f, 1.3f) + 0.18f * wave(0.17f, 0.0f), 2.0f, 0.7f, dt);
    ch(Channel::head_tilt).step(-0.7f * lean + 0.1f * wave(0.23f, 2.5f), 2.3f, 0.6f, dt);
    ch(Channel::head_nod)
        .step(-0.12f * ch(Channel::crouch).v + (air ? 0.35f : 0.0f) + 0.08f * wave(0.5f, 0.4f), 3.2f,
              0.4f, dt);
    // Ears: blown back and down by the speed, floppy (little damping), each a little apart, thrown
    // by the head's turn and the bumps.
    const float head_swing = ch(Channel::head_yaw).v;
    ch(Channel::ear_l_back).step(1.3f * wind + 0.35f * head_swing, 2.7f, 0.2f, dt);
    ch(Channel::ear_r_back).step(1.3f * wind - 0.35f * head_swing, 3.0f, 0.2f, dt);
    ch(Channel::ear_l_up)
        .step(-0.7f * wind + 0.6f * clampf(-m.vz * 0.04f, -1.0f, 1.0f) - bump + 0.15f * wave(0.33f, 0.0f),
              2.6f, 0.18f, dt);
    ch(Channel::ear_r_up)
        .step(-0.7f * wind + 0.6f * clampf(-m.vz * 0.04f, -1.0f, 1.0f) - bump + 0.15f * wave(0.29f, 1.0f),
              2.9f, 0.18f, dt);
    // Tail: swings out of the turn, waves, its tip behind its root, lifts with the speed.
    ch(Channel::tail_side)
        .step(-clampf(m.yaw_rate * 0.7f, -1.5f, 1.5f) + 0.45f * wave(0.65f, 0.0f), 2.0f, 0.25f, dt);
    ch(Channel::tail_tip)
        .step(-0.3f * ch(Channel::tail_side).v + 0.5f * wave(0.65f, -1.3f), 2.6f, 0.22f, dt);
    ch(Channel::tail_up).step(0.8f * wind - 0.5f * ch(Channel::crouch).x + 0.2f * wave(0.4f, 0.9f),
                              2.2f, 0.25f, dt);
    // Face: mouth open on tricks, hits and landings, teeth on a duck or a kick, brows up in the air,
    // down in a duck; a blink every few seconds.
    float jaw = 0.05f, snarl = 0.0f, brows = -0.25f;
    if (air) jaw = 0.3f, brows = 0.7f;
    if (trick) jaw = 0.75f, brows = 1.0f;
    if (pose == Look::hit) jaw = 1.0f, brows = 1.2f;
    if (pose == Look::land) jaw = 0.4f;
    if (pose == Look::duck) snarl = 0.9f, brows = -0.8f;
    if (pose == Look::kick) snarl = 0.8f, jaw = 0.5f, brows = -0.6f;
    ch(Channel::jaw).step(jaw, 4.5f, 0.55f, dt);
    ch(Channel::snarl).step(snarl, 4.0f, 0.6f, dt);
    ch(Channel::brows).step(brows, 3.5f, 0.5f, dt);
    float blink = 0.0f;
    if (t >= m_blink_at) {
        blink = 1.0f;
        if (t >= m_blink_at + 0.11f) {
            m_seed = m_seed * 1664525u + 1013904223u;
            m_blink_at = t + 2.2f + 3.5f * static_cast<float>(m_seed >> 8) / 16777216.0f;
        }
    }
    ch(Channel::blink).step(blink, 9.0f, 0.9f, dt);
}

void RideAnimator::root_turn(float q[4]) const {
    // Spin about up (z), then the flip about his left (y): q = spin * flip.
    const float s[4] = {0.0f, 0.0f, std::sin(m_spin * 0.5f), std::cos(m_spin * 0.5f)};
    const float f[4] = {0.0f, std::sin(m_flip * 0.5f), 0.0f, std::cos(m_flip * 0.5f)};
    qmul(s, f, q);
}

std::vector<JointOut> RideAnimator::joints(const float pivot[3]) const {
    // Each joint: the poses' turns weighted (the rest of the weight his own pose), normalised.
    float sum[kJoints][4] = {};
    bool used[kJoints] = {};
    float total = 0.0f, hips[3] = {};
    for (int i = 0; i < kLooks; ++i) {
        const float w = m_weight[i].x;
        if (std::fabs(w) < 1e-4f) continue;
        total += w;
        const KeyPose& p = key_pose(static_cast<Look>(i));
        for (std::size_t n = 0; n < p.count; ++n) {
            const JointTurn& j = p.turns[n];
            if (j.joint >= kJoints) continue;
            used[j.joint] = true;
            for (int c = 0; c < 4; ++c) sum[j.joint][c] += w * j.q[c];
        }
        for (int c = 0; c < 3; ++c) hips[c] += w * p.pelvis[c];
    }
    const float rest = std::max(0.0f, 1.0f - total);

    std::vector<JointOut> out;
    float root[4];
    root_turn(root);
    if (root[3] < 0.99999f) {
        // The root turns about the pivot: moved by pivot - R pivot.
        JointOut r{0, {root[0], root[1], root[2], root[3]}, {}};
        float rp[3];
        rotate(root, pivot, rp);
        for (int c = 0; c < 3; ++c) r.t[c] = pivot[c] - rp[c];
        out.push_back(r);
    }
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
    hips[2] -= kCrouchDrop * m_chan[static_cast<int>(Channel::crouch)].x;

    for (uint32_t j = 1; j < kJoints; ++j) {
        if (!used[j] && !moved[j] && j != 1) continue;
        float q[4] = {sum[j][0], sum[j][1], sum[j][2], sum[j][3] + rest};
        const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n < 1e-6f) continue;
        for (float& k : q) k /= n;
        if (moved[j]) qmul(q, extra[j], q);
        JointOut o{j, {q[0], q[1], q[2], q[3]}, {}};
        if (j == 1) std::copy(hips, hips + 3, o.t);
        out.push_back(o);
    }
    return out;
}

} // namespace rac1
