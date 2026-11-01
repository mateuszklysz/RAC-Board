// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's animation channels without poses, from OpenGOAL (jak-project, ISC License, Copyright (c)
// 2020-2026 OpenGOAL Team; THIRD_PARTY.md): goal_src/jak2/engine/process-drawable/
// process-drawable-h.gc (num-funcs, ja macros), process-drawable.gc (ja-done?, ja-aframe,
// ja-channel-set!, ja-channel-push!, joint-control-reset!, ja-eval, ja-blend-eval,
// evaluate-joint-control), engine/anim/joint.gc (joint-control-channel-group-eval!) and
// engine/target/target-util.gc (ground-tween-initialize, ground-tween-update).
//
// GOAL addresses channels by pointer into one array; the port uses indices into the same array, and
// the qmem copies become element copies. The board never has float channels, so their copies drop.
#include "board/anim.hpp"

#include <cmath>

#include "board/runtime.hpp"
#include "jak/math/math.hpp"

namespace jak {

namespace {

const AnimInfo& group_info(AnimId g) { return anim_info(g); }

float num_frames_minus_1(AnimId g) {
    return static_cast<float>(group_info(g).num_frames + -1);
}

// eval-time is a uint32 compared with the 64-bit (current-time): equal only while the clock fits.
bool evaluated_now(const JointControlChannel& ch, const AnimClock& clk) {
    return static_cast<int64_t>(ch.eval_time) == clk.now;
}

} // namespace

// The num-funcs (process-drawable-h.gc): num-func-none, -+!, --!, -loop!, -seek!, -blend-in!,
// -chan, -identity. Each returns the value it sets.
float num_func_eval(JointControl& jc, int index, const AnimClock& clk) {
    JointControlChannel& ch = jc.channel[index];
    const float a1 = ch.param[0], a2 = ch.param[1];
    switch (ch.num_func) {
    case NumFunc::none:
    case NumFunc::identity:
        return ch.frame_num;
    case NumFunc::plus: {
        const float f = ch.frame_num + a1 * (group_info(ch.frame_group).speed * clk.time_adjust_ratio);
        ch.frame_num = f;
        return f;
    }
    case NumFunc::minus: {
        const float f = ch.frame_num - a1 * (group_info(ch.frame_group).speed * clk.time_adjust_ratio);
        ch.frame_num = f;
        return f;
    }
    case NumFunc::loop: {
        const float f0 = num_frames_minus_1(ch.frame_group);
        const float f1 =
            ch.frame_num + f0 + a1 * (group_info(ch.frame_group).speed * clk.time_adjust_ratio);
        const float f3 = f1 - static_cast<float>(static_cast<int32_t>(goal_div(f1, f0))) * f0;
        ch.frame_num = f3;
        return f3;
    }
    case NumFunc::seek: {
        const float f = seek(ch.frame_num, a1,
                             a2 * (group_info(ch.frame_group).speed * clk.time_adjust_ratio));
        ch.frame_num = f;
        return f;
    }
    case NumFunc::blend_in: {
        const float f30 = seek(ch.frame_interp[0], 1.0f, a1 * clk.time_adjust_ratio);
        ch.frame_interp[0] = f30;
        ch.frame_interp[1] = f30;
        if (f30 == 1.0f) {
            joint_control_reset(jc, index);
        }
        return f30;
    }
    case NumFunc::chan: {
        // The channel (arg1 - group-sub-index) places away, within this channel's group.
        const int other = index + (static_cast<int32_t>(a1) - ch.group_sub_index);
        const float f = jc.channel[other].frame_num;
        ch.frame_num = f;
        return f;
    }
    }
    return ch.frame_num;
}

// joint-control-channel-eval (joint.gc).
static float joint_control_channel_eval(JointControl& jc, int index, const AnimClock& clk) {
    const float f = num_func_eval(jc, index, clk);
    // A blend-in that finished may have moved the channels; GOAL writes the time through its
    // pointer all the same, into whatever channel now sits there.
    jc.channel[index].eval_time = static_cast<uint32_t>(clk.now);
    return f;
}

// joint-control-channel-group-eval! (joint.gc).
void joint_control_channel_group_eval(JointControl& jc, int index, AnimId group, NumFunc nf,
                                      const AnimClock& clk) {
    JointControlChannel& ch = jc.channel[index];
    ch.num_func = nf;
    if (ch.command == jcc::stack) {
        return;
    }
    if (group != AnimId::none) {
        ch.frame_group = group;
    }
    num_func_eval(jc, index, clk);
    jc.channel[index].eval_time = static_cast<uint32_t>(clk.now);
}

// joint-control-channel-group! (joint.gc).
void joint_control_channel_group(JointControl& jc, int index, AnimId group, NumFunc nf) {
    JointControlChannel& ch = jc.channel[index];
    ch.num_func = nf;
    if (ch.command == jcc::stack) {
        return;
    }
    if (group != AnimId::none) {
        ch.frame_group = group;
    }
}

// joint-control-reset! (process-drawable.gc): a finished blend drops the groups under it and itself.
void joint_control_reset(JointControl& jc, int blend) {
    const int v1 = blend - jc.channel[blend].group_size;  // the group the blend brought in
    const int s5 = v1;                                    // channels under it
    const int s4 = blend - v1;                            // its size
    if (s5 > 0) {
        if (jc.channel[v1].command == jcc::push1) {
            jc.channel[v1].command = jcc::push;
        }
        if (jc.root == v1) {
            jc.root = 0;
        } else {
            jc.root = jc.root - (s5 + 1);
        }
        const int total = static_cast<int>(jc.active_channels + jc.float_channels);
        // qmem-copy<-! channel[0..] from channel[v1..], (total - s5) channels.
        for (int i = 0; i < total - s5; i++) {
            jc.channel[i] = jc.channel[v1 + i];
        }
        // Then the blend channel (now at s4) goes: channel[s4..] from channel[s4 + 1..].
        const int n = -1 - s5 - s4 + total;
        for (int i = 0; i < n; i++) {
            jc.channel[s4 + i] = jc.channel[s4 + 1 + i];
        }
        jc.active_channels = jc.active_channels - static_cast<uint32_t>(s5 + 1);
    }
}

// ja-channel-set! (process-drawable.gc).
int ja_channel_set(JointControl& jc, int count, const AnimClock& clk) {
    // (the float channels would be copied here; the board has none)
    jc.active_channels = static_cast<uint32_t>(count);
    jc.root = 0;
    jc.blend_index = -1;
    for (int i = 0; i < count; i++) {
        JointControlChannel& ch = jc.channel[i];
        ch.eval_time = static_cast<uint32_t>(clk.now);
        ch.group_sub_index = static_cast<int8_t>(i);
        ch.command = i == 0 ? jcc::push : jcc::blend;
        ch.frame_interp[0] = 0.0f;
        ch.frame_interp[1] = 0.0f;
        ch.frame_num = 0.0f;
        ch.frame_group = AnimId::none;
        ch.num_func = NumFunc::none;
        ch.group_size = static_cast<int8_t>(count);
    }
    return count;
}

// ja-channel-push! (process-drawable.gc): a new group of `count` channels on top, blended in over
// `blend` time-frames by a blend channel above it.
int ja_channel_push(JointControl& jc, int count, TimeFrame blend, const AnimClock& clk) {
    if (jc.active_channels == 0 || blend == 0 ||
        static_cast<int>(jc.active_channels + jc.float_channels) + count >=
            JointControl::kAllocated) {
        // (GOAL prints a warning when the channels run out)
        return ja_channel_set(jc, count, clk);
    }
    if (jc.root_channel(0).frame_group == AnimId::none) {
        // The current group never got an animation: it is dropped rather than blended from.
        jc.active_channels = static_cast<uint32_t>(jc.root);
        if (jc.active_channels == 0) {
            return ja_channel_set(jc, count, clk);
        }
    }
    const int s3 = static_cast<int>(jc.active_channels);
    jc.root = s3;
    jc.active_channels = static_cast<uint32_t>(count + 1) + jc.active_channels;
    for (int i = 0; i < count; i++) {
        JointControlChannel& ch = jc.root_channel(i);
        ch.eval_time = static_cast<uint32_t>(clk.now);
        ch.group_sub_index = static_cast<int8_t>(i);
        ch.command = i == 0 ? jcc::push : jcc::blend;
        ch.frame_interp[0] = 0.0f;
        ch.frame_interp[1] = 0.0f;
        ch.frame_num = 0.0f;
        ch.frame_group = AnimId::none;
        ch.num_func = NumFunc::none;
        ch.group_size = static_cast<int8_t>(count);
    }
    JointControlChannel& b = jc.root_channel(count);
    b.eval_time = 0;
    b.group_sub_index = static_cast<int8_t>(count);
    jc.blend_index = static_cast<int32_t>(jc.active_channels) + -1;
    b.frame_interp[0] = 0.0f;
    b.frame_interp[1] = 0.0f;
    b.frame_num = 0.0f;
    b.frame_group = AnimId::none;
    b.group_size = static_cast<int8_t>(count);
    b.param[0] = goal_div(5.0f, 5.0f + static_cast<float>(blend));
    b.num_func = NumFunc::blend_in;
    if (count == 1) {
        b.command = jcc::stack1;
        jc.root_channel(0).command = jcc::push1;
    } else {
        b.command = jcc::stack;
    }
    return count;
}

// ja-done? (process-drawable.gc): a seek has reached its target; any other num-func is "done".
bool ja_done(const JointControl& jc, int chan) {
    const JointControlChannel& ch = jc.root_channel(chan);
    if (jc.active_channels == 0) {
        return true;
    }
    if (ch.num_func == NumFunc::seek) {
        return ch.frame_num == ch.param[0];
    }
    return true;
}

float ja_frame_num(const JointControl& jc, int chan) { return jc.root_channel(chan).frame_num; }

// ja-aframe-num (process-drawable.gc): the frame in the artist's numbering.
float ja_aframe_num(const JointControl& jc, int chan) {
    const JointControlChannel& ch = jc.root_channel(chan);
    if (ch.frame_group != AnimId::none) {
        const AnimInfo& a = group_info(ch.frame_group);
        return ch.frame_num * a.artist_step + a.artist_base;
    }
    return 0.0f;
}

// ja-aframe (process-drawable.gc): an artist frame to a frame of the channel's animation.
float ja_aframe(const JointControl& jc, float aframe, int chan) {
    const JointControlChannel& ch = jc.root_channel(chan);
    if (ch.frame_group != AnimId::none) {
        const AnimInfo& a = group_info(ch.frame_group);
        return goal_div(aframe - a.artist_base, a.artist_step);
    }
    return aframe;
}

int ja_num_frames(const JointControl& jc, int chan) {
    return group_info(jc.root_channel(chan).frame_group).num_frames + -1;
}

// The ja-group macro: the channel's animation while the channel is active.
AnimId ja_group(const JointControl& jc, int chan) {
    if (static_cast<int>(jc.active_channels) > chan) {
        return jc.root_channel(chan).frame_group;
    }
    return AnimId::none;
}

// ja-group-size (process-drawable.gc).
int ja_group_size(const JointControl& jc) {
    if (jc.root < static_cast<int>(jc.active_channels)) {
        return jc.root_channel(0).group_size;
    }
    return 0;
}

// ja-eval (process-drawable.gc): the root group and above, once a frame.
void ja_eval(JointControl& jc, const AnimClock& clk) {
    const int end = static_cast<int>(jc.active_channels + jc.float_channels);
    for (int i = jc.root; i < end; i++) {
        if (jc.channel[i].command & jcc::command_4) {
            continue;
        }
        if (!evaluated_now(jc.channel[i], clk)) {
            joint_control_channel_eval(jc, i, clk);
        }
    }
}

// ja-blend-eval (process-drawable.gc): the groups under the root, once a frame.
void ja_blend_eval(JointControl& jc, const AnimClock& clk) {
    if (jc.active_channels != 0 && jc.root != 0) {
        for (int i = 0; i < jc.root; i++) {
            if (jc.channel[i].command & jcc::command_4) {
                continue;
            }
            if (!evaluated_now(jc.channel[i], clk)) {
                joint_control_channel_eval(jc, i, clk);
            }
        }
    }
}

// evaluate-joint-control (process-drawable.gc), the channel part: the stack (blend) and float
// channels run their num-func once a frame, stack1 hands its blend to the channel under it, the
// frames clamp to the animation and the blends to [0, 1]. Repeats while a blend finishing changes
// the channel count. (The decompressing, the art check and the blend shapes are the renderer's.)
void evaluate_joint_control(JointControl& jc, const AnimClock& clk) {
    uint32_t s4 = 0xffffffffu;
    while (s4 != jc.active_channels + jc.float_channels) {
        s4 = jc.active_channels + jc.float_channels;
        for (int s2 = 0; s2 < static_cast<int>(s4); s2++) {
            JointControlChannel& ch = jc.channel[s2];
            if ((ch.command & jcc::thirty_two) && !evaluated_now(ch, clk)) {
                joint_control_channel_eval(jc, s2, clk);
            }
            JointControlChannel& c = jc.channel[s2];
            if (c.command == jcc::stack) {
                continue;
            }
            if (c.command == jcc::stack1) {
                jc.channel[s2 + -1].frame_interp[0] = c.frame_interp[0];
                jc.channel[s2 + -1].frame_interp[1] = c.frame_interp[1];
            } else if (c.frame_group != AnimId::none) {
                // (with no animation GOAL sends the process to its art-error state; the board's
                // code always sets one before the frame ends)
                c.frame_num = std::fmax(0.0f, std::fmin(c.frame_num, num_frames_minus_1(c.frame_group)));
            }
        }
        for (int i = 0; i < static_cast<int>(s4); i++) {
            jc.channel[i].frame_interp[0] = std::fmax(0.0f, std::fmin(1.0f, jc.channel[i].frame_interp[0]));
            jc.channel[i].frame_interp[1] = std::fmax(0.0f, std::fmin(1.0f, jc.channel[i].frame_interp[1]));
        }
    }
}

// The ja macro's expansion (process-drawable-h.gc), step by step in its order.
void Ja::run(JointControl& jc, const AnimClock& clk) const {
    const int index = jc.root + chan_;
    JointControlChannel* ch = &jc.channel[index];
    auto value = [&](const JaVal& v) { return v.aframe ? ja_aframe(jc, v.v, chan_) : v.v; };

    NumFunc nf = NumFunc::none;
    bool has_nf = true;
    switch (num_) {
    case Num::unset: has_nf = false; break;
    case Num::seek: nf = NumFunc::seek; break;
    case Num::loop: nf = NumFunc::loop; break;
    case Num::min:
    case Num::max:
    case Num::identity: nf = NumFunc::identity; break;
    case Num::chan: nf = NumFunc::chan; break;
    }
    const bool has_p0 = num_ == Num::seek || num_ == Num::loop || num_ == Num::chan;
    const bool has_p1 = num_ == Num::seek;
    // frame-group is set here only when a param, a frame-num or no num-func comes with it;
    // otherwise the group-eval below sets it.
    if ((has_p0 || has_p1 || has_frame_ || !has_nf) && group_ != AnimId::none) {
        ch->frame_group = group_;
    }
    if (has_p0) {
        float p0 = 1.0f;
        if (num_ == Num::seek) {
            if (has_a0_ && !seek_max_) {
                p0 = value(a0_);
            } else {
                p0 = num_frames_minus_1(group_ != AnimId::none ? group_ : ch->frame_group);
            }
        } else if (num_ == Num::chan) {
            p0 = a0_.v;
        }
        ch->param[0] = p0;
    }
    if (has_p1) {
        const float p1 = has_a1_ ? a1_ : 1.0f;
        ch->param[1] = p1;
        ch->param[2] = p1;
    }
    if (has_num_func_) {
        ch->num_func = num_func_;
    }
    if (has_frame_) {
        ch->frame_num = value(frame_);
    }
    if (has_nf) {
        if (eval_) {
            joint_control_channel_group_eval(jc, index, group_, nf, clk);
        } else {
            joint_control_channel_group(jc, index, group_, nf);
        }
        ch = &jc.channel[index];
    }
    if (num_ == Num::min) {
        ch->frame_num = 0.0f;
    } else if (num_ == Num::max) {
        ch->frame_num = num_frames_minus_1(group_ != AnimId::none ? group_ : ch->frame_group);
    } else if (num_ == Num::identity) {
        ch->frame_num = value(a0_);
    }
}

JointControl& skel(Target& self) { return runtime(self).skel; }

AnimClock anim_clock(const Target& self) {
    return AnimClock{current_time(self), self.clock.time_adjust_ratio};
}

void ja_post(Target& self) { evaluate_joint_control(skel(self), anim_clock(self)); }

// ground-tween-initialize (target-util.gc): three channels on the first animation, at one frame.
GroundTweenInfo& ground_tween_initialize(Target& self, GroundTweenInfo& info, uint32_t chan0,
                                         AnimId g0, AnimId g1, AnimId g2, AnimId g3, AnimId g4,
                                         float frame) {
    JointControl& jc = skel(self);
    info.group[0] = static_cast<uint32_t>(g0);
    info.group[1] = static_cast<uint32_t>(g1);
    info.group[2] = static_cast<uint32_t>(g2);
    info.group[3] = static_cast<uint32_t>(g3);
    info.group[4] = static_cast<uint32_t>(g4);
    for (int i = 0; i < 3; i++) {
        info.chan[i] = static_cast<uint8_t>(chan0 + i);
        const int index = jc.root + info.chan[i];
        const float f = std::fabs(info.blend[i]);
        jc.channel[index].frame_interp[1] = f;
        jc.channel[index].frame_interp[0] = f;
        joint_control_channel_group_eval(jc, index, g0, NumFunc::identity, anim_clock(self));
        jc.channel[index].frame_num = frame;
    }
    return info;
}

// ground-tween-update (target-util.gc): channel 1 leans up or down, channel 2 right or left.
void ground_tween_update(Target& self, GroundTweenInfo& info, float x, float z, float speed) {
    JointControl& jc = skel(self);
    const float f0 = std::fmax(-1.0f, std::fmin(1.0f, x));
    const float f30 = std::fmax(-1.0f, std::fmin(1.0f, z));
    {
        const float d = std::fabs(f0 - info.blend[1]);
        info.blend[1] = seek(info.blend[1], f0, std::fmax(std::fmin(d * speed, 0.8f * speed), speed / 5.0f));
    }
    {
        const float d = std::fabs(f30 - info.blend[2]);
        info.blend[2] = seek(info.blend[2], f30, std::fmax(std::fmin(d * speed, 0.8f * speed), speed / 5.0f));
    }
    JointControlChannel& c1 = jc.root_channel(info.chan[1]);
    c1.frame_interp[1] = std::fabs(info.blend[1]);
    c1.frame_interp[0] = std::fabs(info.blend[1]);
    c1.frame_group = static_cast<AnimId>(info.blend[1] >= 0.0f ? info.group[1] : info.group[2]);
    JointControlChannel& c2 = jc.root_channel(info.chan[2]);
    c2.frame_interp[1] = std::fabs(info.blend[2]);
    c2.frame_interp[0] = std::fabs(info.blend[2]);
    c2.frame_group = static_cast<AnimId>(info.blend[2] >= 0.0f ? info.group[4] : info.group[3]);
}

AnimPlaying anim_playing(const JointControl& jc) {
    AnimPlaying p;
    if (jc.active_channels == 0) {
        return p;
    }
    p.group = jc.root_channel(0).frame_group;
    p.frame = jc.root_channel(0).frame_num;
    p.aframe = ja_aframe_num(jc, 0);
    return p;
}

} // namespace jak

namespace jak {

// mod-var-jump's look-up of the playing group's 'collide-offset lump (core's
// board_from_core.hpp): 0 when there is no (ja-group) or no lump table, 1 with the offset, 2 when the
// table lacks the lump.
// None of the board's animations (jak-board in ART.CGO) has an extra res table on the disc, so every
// group the board plays reads as having no table (0), as in the game.
int ja_group_collide_offset(Target& self, Vec& out) {
    (void)out;
    if (ja_group(skel(self), 0) == AnimId::none) {
        return 0;
    }
    return 0;
}

} // namespace jak
