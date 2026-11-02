// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Ratchet's clip mixer (rac1/ride_clip.*): a clip read at its frames, the commands of Jak II's
// joint-control (push, blend, stack) in order, and turns blended the short way round.
#include <cmath>
#include <initializer_list>

#include "check.hpp"
#include "rac1/anim/ride_clip.hpp"

using namespace jak;
using namespace rac1;

int main() {
    // Every animation the stance and the jump play has a clip of Jak's length.
    for (AnimId id : {AnimId::jakb_board_stance, AnimId::jakb_board_turn, AnimId::jakb_board_turn_up,
                      AnimId::jakb_board_turn_down, AnimId::jakb_board_turn_left, AnimId::jakb_board_turn_right,
                      AnimId::jakb_board_duck_turn, AnimId::jakb_board_jump, AnimId::jakb_board_jump_loop,
                      AnimId::jakb_board_get_on_land}) {
        const Clip* c = find_clip(id);
        CHECK(c != nullptr);
        if (c) CHECK(c->frames == anim_info(id).num_frames);
    }
    const Clip& turn = *find_clip(AnimId::jakb_board_turn);
    // A whole frame reads that frame's keys.
    MixPose a, b;
    sample(turn, 3.0f, a);
    const int j = turn.joint[0];
    CHECK_NEAR(a.q[j][3], turn.q[(3 * turn.joints) * 4 + 3], 1e-6);
    CHECK_NEAR(a.hips[2], turn.hips[3 * 3 + 2], 1e-6);
    // Half way between frames: between the two.
    sample(turn, 3.5f, b);
    const float lo = std::fmin(turn.board[3 * kBoardTracks + kPz], turn.board[4 * kBoardTracks + kPz]);
    const float hi = std::fmax(turn.board[3 * kBoardTracks + kPz], turn.board[4 * kBoardTracks + kPz]);
    CHECK(lo - 1e-6f <= b.board[kPz] && b.board[kPz] <= hi + 1e-6f);
    const float n = b.board[kQx] * b.board[kQx] + b.board[kQy] * b.board[kQy] + b.board[kQz] * b.board[kQz] +
                    b.board[kQw] * b.board[kQw];
    CHECK_NEAR(n, 1.0f, 1e-5);

    // One group: push the turn, blend the duck in by half.
    JointControl jc;
    jc.active_channels = 2;
    jc.channel[0] = JointControlChannel{};
    jc.channel[0].frame_group = AnimId::jakb_board_turn;
    jc.channel[0].frame_num = 5.0f;
    jc.channel[0].command = jcc::push;
    jc.channel[1] = jc.channel[0];
    jc.channel[1].frame_group = AnimId::jakb_board_duck_turn;
    jc.channel[1].command = jcc::blend;
    jc.channel[1].frame_interp[0] = 0.5f;
    MixPose m, t, d;
    mix(jc, m);
    sample(turn, 5.0f, t);
    sample(*find_clip(AnimId::jakb_board_duck_turn), 5.0f, d);
    CHECK_NEAR(m.hips[2], 0.5f * (t.hips[2] + d.hips[2]), 1e-5);
    // A new group stacked over it at weight 0 leaves it; at 1 it is all the new group.
    jc.active_channels = 4;
    jc.channel[2] = JointControlChannel{};
    jc.channel[2].frame_group = AnimId::jakb_board_jump;
    jc.channel[2].command = jcc::push1;
    jc.channel[3] = JointControlChannel{};
    jc.channel[3].command = jcc::stack1;
    jc.channel[3].frame_interp[0] = 0.0f;
    MixPose s0, s1, jump;
    mix(jc, s0);
    CHECK_NEAR(s0.hips[2], m.hips[2], 1e-6);
    jc.channel[3].frame_interp[0] = 1.0f;
    mix(jc, s1);
    sample(*find_clip(AnimId::jakb_board_jump), 0.0f, jump);
    CHECK_NEAR(s1.hips[2], jump.hips[2], 1e-6);

    // The board's turn blends the shorter way: q and -q are the same turn.
    MixPose x, y;
    x.clear();
    y.clear();
    y.board[kQw] = -1.0f;
    blend(x, y, 0.5f);
    CHECK_NEAR(std::fabs(x.board[kQw]), 1.0f, 1e-6);
    TEST_MAIN_END();
}
