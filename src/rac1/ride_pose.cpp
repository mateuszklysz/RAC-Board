// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/ride_pose.hpp"

#include <algorithm>
#include <cmath>

#include "rac1/ride_poses.inc"

namespace rac1 {

namespace {

constexpr int kLooks = static_cast<int>(Look::count);
// How fast a pose blends in: about 90% in 0.12 s.
constexpr float kBlendRate = 18.0f;
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

void RideAnimator::reset() {
    std::fill(m_weight, m_weight + kLooks, 0.0f);
    m_flip = 0.0f;
    m_spin = 0.0f;
}

void RideAnimator::update(const LookTarget& target, float dt) {
    float want[kLooks] = {};
    if (target.pose == Look::stance) {
        const float lean = std::clamp(target.lean, -1.0f, 1.0f);
        want[static_cast<int>(Look::stance)] = 1.0f - std::fabs(lean);
        want[static_cast<int>(lean < 0.0f ? Look::lean_left : Look::lean_right)] = std::fabs(lean);
    } else {
        want[static_cast<int>(target.pose)] = 1.0f;
    }
    const float k = 1.0f - std::exp(-kBlendRate * dt);
    for (int i = 0; i < kLooks; ++i) m_weight[i] += (want[i] - m_weight[i]) * k;
    m_flip = target.flip;
    m_spin = target.spin;
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
        const float w = m_weight[i];
        if (w < 1e-4f) continue;
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
    for (uint32_t j = 1; j < kJoints; ++j) {
        if (!used[j]) continue;
        float q[4] = {sum[j][0], sum[j][1], sum[j][2], sum[j][3] + rest};
        const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n < 1e-6f) continue;
        JointOut o{j, {q[0] / n, q[1] / n, q[2] / n, q[3] / n}, {}};
        if (j == 1) std::copy(hips, hips + 3, o.t);
        out.push_back(o);
    }
    return out;
}

} // namespace rac1
