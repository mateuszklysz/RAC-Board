// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Ratchet's riding clips (ride_clips.inc, made by tools/poses/clips.py), one per animation of Jak's
// the board's logic plays, with Jak's name and number of frames, and the mixer that poses him from
// the logic's joint-control as Jak II's animation code poses Jak: each channel's clip sampled at its
// frame, then the channels' commands in order (push a group's first channel, blend the next ones
// into it by their weight, stack a new group over the one under it by its blend's weight).
#pragma once

#include <cstddef>
#include <cstdint>

#include "board/anim.hpp"

namespace rac1 {

struct Clip {
    const char* name;  // Jak II's name, "jakb-board-jump-ja"
    int frames;
    int joints;
    const uint8_t* joint;  // the joints it turns
    const float* q;        // frames * joints offsets (x, y, z, w) over his standing pose
    const float* hips;     // frames * 3: the hips' move (model units)
    const float* board;    // frames * 7: the board's turn (x, y, z, w) and place (model units: x his
                           // heading, y left, z up), as it moves with him
};
// The clip of one of Jak's animations, or null when there is none of ours yet.
const Clip* find_clip(jak::AnimId id);

constexpr int kPoseJoints = 128;
enum BoardTrack { kQx, kQy, kQz, kQw, kPx, kPy, kPz, kBoardTracks };

// A pose of the clips: each joint's offset (identity where no clip turns it).
struct MixPose {
    float q[kPoseJoints][4];
    bool used[kPoseJoints];
    float hips[3];
    float board[kBoardTracks];
    void clear();
};

// One clip at a frame (between two frames: blended), as Jak's frames are read.
void sample(const Clip& clip, float frame, MixPose& out);
// a toward b by w (0 a, 1 b): the joints' turns along the shorter way, normalised.
void blend(MixPose& a, const MixPose& b, float w);
// a turned toward b by w (0 a, 1 b): quaternions, the shorter way, normalised.
void nlerp(float a[4], const float b[4], float w);
// The board's turn toward b's by w (the shorter way, normalised) and its place.
void blend_board(float a[kBoardTracks], const float b[kBoardTracks], float w);
// The joint-control's channels, mixed in their order by their commands.
void mix(const jak::JointControl& jc, MixPose& out);

} // namespace rac1
