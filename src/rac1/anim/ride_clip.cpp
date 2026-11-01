// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/anim/ride_clip.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "rac1/anim/ride_clips.inc"

namespace rac1 {

void nlerp(float a[4], const float b[4], float w) {
    const float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    const float s = d < 0.0f ? -1.0f : 1.0f;
    float n = 0.0f;
    for (int c = 0; c < 4; ++c) {
        a[c] = a[c] + w * (s * b[c] - a[c]);
        n += a[c] * a[c];
    }
    n = std::sqrt(n);
    if (n > 1e-6f) {
        for (int c = 0; c < 4; ++c) a[c] /= n;
    }
}

void blend_board(float a[kBoardTracks], const float b[kBoardTracks], float w) {
    nlerp(a, b, w);
    for (int c = kPx; c <= kPz; ++c) a[c] += w * (b[c] - a[c]);
}

namespace {

// The deepest a stack of groups goes: each push adds one, each stack takes one.
constexpr int kStack = 8;

} // namespace

void MixPose::clear() {
    for (int j = 0; j < kPoseJoints; ++j) {
        q[j][0] = q[j][1] = q[j][2] = 0.0f;
        q[j][3] = 1.0f;
        used[j] = false;
    }
    std::fill(hips, hips + 3, 0.0f);
    std::fill(board, board + kBoardTracks, 0.0f);
    board[kQw] = 1.0f;
}

const Clip* find_clip(jak::AnimId id) {
    if (id == jak::AnimId::none) return nullptr;
    // Looked up once per animation, by Jak's name.
    static const Clip* table[static_cast<int>(jak::AnimId::count)];
    static bool ready = false;
    if (!ready) {
        for (int i = 0; i < static_cast<int>(jak::AnimId::count); ++i) {
            table[i] = nullptr;
            const char* name = jak::anim_info(static_cast<jak::AnimId>(i)).name;
            for (const Clip& c : clips::kClips) {
                if (std::strcmp(c.name, name) == 0) table[i] = &c;
            }
        }
        ready = true;
    }
    const int i = static_cast<int>(id);
    return i >= 0 && i < static_cast<int>(jak::AnimId::count) ? table[i] : nullptr;
}

void sample(const Clip& clip, float frame, MixPose& out) {
    out.clear();
    const float f = std::clamp(frame, 0.0f, static_cast<float>(clip.frames - 1));
    const int f0 = static_cast<int>(f);
    const int f1 = std::min(f0 + 1, clip.frames - 1);
    const float w = f - static_cast<float>(f0);
    for (int k = 0; k < clip.joints; ++k) {
        const int j = clip.joint[k];
        if (j >= kPoseJoints) continue;
        std::memcpy(out.q[j], clip.q + (f0 * clip.joints + k) * 4, sizeof(float) * 4);
        if (w > 0.0f) nlerp(out.q[j], clip.q + (f1 * clip.joints + k) * 4, w);
        out.used[j] = true;
    }
    for (int c = 0; c < 3; ++c) {
        out.hips[c] = clip.hips[f0 * 3 + c] + w * (clip.hips[f1 * 3 + c] - clip.hips[f0 * 3 + c]);
    }
    const float* b0 = clip.board + f0 * kBoardTracks;
    std::memcpy(out.board, b0, sizeof(float) * kBoardTracks);
    if (w > 0.0f) blend_board(out.board, clip.board + f1 * kBoardTracks, w);
}

void blend(MixPose& a, const MixPose& b, float w) {
    if (w <= 0.0f) return;
    for (int j = 0; j < kPoseJoints; ++j) {
        if (!a.used[j] && !b.used[j]) continue;
        nlerp(a.q[j], b.q[j], w);
        a.used[j] = true;
    }
    for (int c = 0; c < 3; ++c) a.hips[c] += w * (b.hips[c] - a.hips[c]);
    blend_board(a.board, b.board, w);
}

void mix(const jak::JointControl& jc, MixPose& out) {
    static MixPose stack[kStack];
    static MixPose one;
    int top = -1;
    const auto clip_pose = [](const jak::JointControlChannel& ch, MixPose& p) {
        const Clip* c = find_clip(ch.frame_group);
        if (!c) c = find_clip(jak::AnimId::jakb_board_stance);
        if (c) {
            sample(*c, ch.frame_num, p);
        } else {
            p.clear();
        }
    };
    const int n = static_cast<int>(std::min<uint32_t>(jc.active_channels, jak::JointControl::kAllocated));
    for (int i = 0; i < n; ++i) {
        const jak::JointControlChannel& ch = jc.channel[i];
        switch (ch.command) {
        case jak::jcc::push:
        case jak::jcc::push1:
            if (top + 1 < kStack) {
                ++top;
                clip_pose(ch, stack[top]);
            }
            break;
        case jak::jcc::blend:
            if (top >= 0) {
                clip_pose(ch, one);
                blend(stack[top], one, ch.frame_interp[0]);
            }
            break;
        case jak::jcc::stack:
        case jak::jcc::stack1:
            // The group on top blends in over the one under it by this channel's weight.
            if (top >= 1) {
                blend(stack[top - 1], stack[top], ch.frame_interp[0]);
                --top;
            }
            break;
        default: break;
        }
    }
    if (top >= 0) {
        out = stack[top];
    } else {
        const Clip* c = find_clip(jak::AnimId::jakb_board_stance);
        if (c) {
            sample(*c, 0.0f, out);
        } else {
            out.clear();
        }
    }
}

} // namespace rac1
