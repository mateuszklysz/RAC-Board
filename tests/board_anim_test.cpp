// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The joint-control emulation (process-drawable.gc, joint.gc): ja-channel-push!'s blend, the seek!
// num-func, ja-done?, the ja macro's frame-group rule.
#include <cmath>

#include "board_fakes.hpp"
#include "check.hpp"

using namespace jak;

int main() {
    JointControl jc;
    AnimClock clk;
    clk.now = 3000;
    ja_channel_set(jc, 1, clk);
    Ja().group(AnimId::jakb_board_jump).seek().frame_num(0.0f).no_eval().run(jc, clk);
    CHECK(ja_group(jc, 0) == AnimId::jakb_board_jump);
    CHECK(jc.root_channel(0).frame_num == 0.0f);
    const AnimInfo& info = anim_info(AnimId::jakb_board_jump);
    const float last = static_cast<float>(info.num_frames - 1);
    // seek! to the last frame: the art's speed in frames a tick (time-adjust-ratio 1).
    Ja().seek().run(jc, clk);
    CHECK_NEAR(jc.root_channel(0).frame_num, info.speed, 1e-5);
    CHECK(!ja_done(jc, 0));
    int ticks = 1;
    while (!ja_done(jc, 0) && ticks < 1000) {
        Ja().seek().run(jc, clk);
        ++ticks;
    }
    CHECK(ja_done(jc, 0));
    CHECK_NEAR(jc.root_channel(0).frame_num, last, 1e-5);
    CHECK(std::fabs(ticks - last / info.speed) <= 1.0f);

    // A push of 1 over 0.05 s (15 ticks): a blend channel with param 0 = 5 / (5 + 15) = 0.25.
    ja_channel_push(jc, 1, seconds(0.05), clk);
    CHECK(jc.active_channels >= 2);
    bool found = false;
    for (uint32_t i = 0; i < jc.active_channels; i++) {
        if (jc.channel[i].num_func == NumFunc::blend_in) {
            CHECK_NEAR(jc.channel[i].param[0], 0.25, 1e-7);
            found = true;
        }
    }
    CHECK(found);
    // Without a group or a frame, the ja macro leaves the group to group-eval: the new top channel
    // gets the jump loop, the one under it keeps the jump.
    Ja().group(AnimId::jakb_board_jump_loop).run(jc, clk);
    CHECK(ja_group(jc, 0) == AnimId::jakb_board_jump_loop);
    // What the adapter reads.
    const AnimPlaying p = anim_playing(jc);
    CHECK(p.group == AnimId::jakb_board_jump_loop);
    TEST_MAIN_END();
}
