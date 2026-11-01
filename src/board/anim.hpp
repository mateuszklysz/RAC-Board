// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's animation channels as logic only: the joint-control and its channels (joint-h.gc), the
// num-funcs and the ja macros (process-drawable-h.gc), ja-done?, ja-aframe, ja-channel-push! and the
// rest (process-drawable.gc), ported from OpenGOAL (jak-project, ISC License, Copyright (c)
// 2020-2026 OpenGOAL Team; THIRD_PARTY.md). No pose is computed: only frame numbers and blend
// weights, because the board's states branch and time themselves on them. The adapter reads which
// animation and frame play (anim_playing) to pick Ratchet's own animations.
#pragma once

#include <cstdint>

#include "jak/basic_types.hpp"
#include "jak/target.hpp"

namespace jak {

// The animations the board's code names (art-elts.gc, jakb-ag and board-ag). Their frame counts and
// artist frame numbering are art data (anim_table.cpp).
enum class AnimId : int16_t {
    none = -1,
    jakb_board_stance, jakb_board_duck_turn, jakb_board_turn, jakb_board_jump_kick,
    jakb_board_jump, jakb_board_jump_loop, jakb_board_hit, jakb_board_flip_forward,
    jakb_board_turn_left, jakb_board_turn_right, jakb_board_turn_up, jakb_board_turn_down,
    jakb_board_hit_forward, jakb_board_ride_turn_back, jakb_board_ride_turn_front,
    jakb_board_ride_turn_right, jakb_board_ride_turn_left, jakb_board_get_on, jakb_board_get_off,
    jakb_board_grenade, jakb_board_nosegrab, jakb_board_method, jakb_board_kickflip_a,
    jakb_board_spin, jakb_board_air_turn, jakb_board_flip_backward, jakb_board_flip_forward_loop,
    jakb_board_flip_backward_loop, jakb_board_method_loop, jakb_board_method_end,
    jakb_board_nosegrab_loop, jakb_board_nosegrab_end, jakb_board_noseflip,
    jakb_board_jump_kickoff, jakb_board_hit_elec, jakb_board_get_on_land, jakb_board_get_off_pre,
    jakb_board_kickflip_b, jakb_board_kickflip_c, jakb_board_hit_get_off,
    jakb_board_attack_pegasus,
    // The board's own skeleton (board-ag): board-util.gc's idle and use states.
    board_board_idle, board_open, board_close,
    count
};

// art-joint-anim, the part the logic reads: (-> frames num-frames), artist-base, artist-step, speed.
struct AnimInfo {
    const char* name;  // Jak II's name, "jakb-board-jump-ja"
    int num_frames;
    float artist_base;
    float artist_step;
    float speed;
    bool placeholder;  // true while the numbers are a guess (anim_table.cpp)
};
const AnimInfo& anim_info(AnimId id);

// The channel's num-func (process-drawable-h.gc).
enum class NumFunc : uint8_t { none, plus, minus, loop, seek, blend_in, chan, identity };

// joint-control-command (joint-h.gc).
namespace jcc {
constexpr int64_t zero = 0, push = 1, blend = 2, command_4 = 4, eight = 8, push1 = 19,
                  thirty_two = 32, stack = 36, float_ = 42, stack1 = 52;
}

// joint-control-channel (joint-h.gc).
struct JointControlChannel {
    AnimId frame_group = AnimId::none;
    float frame_num = 0.0f;
    float dist = 0.0f;
    NumFunc num_func = NumFunc::none;
    float param[3] = {};
    float frame_interp[2] = {};
    int64_t command = 0;
    int8_t group_sub_index = 0;
    int8_t group_size = 0;
    uint32_t eval_time = 0;
};

// joint-control (joint-h.gc): the channel array, the root channel as an index into it.
struct JointControl {
    // TODO(art): the channel count is the skeleton's 'joint-channel res (art data, default 6 in
    // initialize-skeleton); Jak's is not in the OpenGOAL repo. 16 is a guess large enough for the
    // board's pushes (4 channels + blend on top of the previous group).
    static constexpr int kAllocated = 16;
    JointControlChannel channel[kAllocated];
    int root = 0;  // root-channel = &channel[root]
    uint32_t active_channels = 0;
    uint32_t float_channels = 0;  // the board never adds float channels; kept for the arithmetic
    int32_t blend_index = -1;
    JointControlChannel& root_channel(int i) { return channel[root + i]; }
    const JointControlChannel& root_channel(int i) const { return channel[root + i]; }
};

// What the num-funcs read from the process: (current-time) and the clock's time-adjust-ratio.
struct AnimClock {
    TimeFrame now = 0;
    float time_adjust_ratio = 1.0f;
};

// ---- process-drawable.gc / joint.gc, on any joint-control ----
float num_func_eval(JointControl& jc, int index, const AnimClock& clk); // the channel's num-func
void joint_control_channel_group_eval(JointControl& jc, int index, AnimId group, NumFunc nf,
                                      const AnimClock& clk);
void joint_control_channel_group(JointControl& jc, int index, AnimId group, NumFunc nf);
void joint_control_reset(JointControl& jc, int blend_index);
int ja_channel_set(JointControl& jc, int count, const AnimClock& clk);
int ja_channel_push(JointControl& jc, int count, TimeFrame blend, const AnimClock& clk);
bool ja_done(const JointControl& jc, int chan);
float ja_frame_num(const JointControl& jc, int chan);
float ja_aframe_num(const JointControl& jc, int chan);
float ja_aframe(const JointControl& jc, float aframe, int chan);
int ja_num_frames(const JointControl& jc, int chan);
AnimId ja_group(const JointControl& jc, int chan = 0); // the ja-group macro: none when inactive
int ja_group_size(const JointControl& jc);
void ja_eval(JointControl& jc, const AnimClock& clk);
void ja_blend_eval(JointControl& jc, const AnimClock& clk);
// evaluate-joint-control (process-drawable.gc), what ja-post runs: blend-ins advance, frames clamp.
void evaluate_joint_control(JointControl& jc, const AnimClock& clk);

// A value for the ja macro: a number, or (ja-aframe v chan) worked out when the macro uses it.
struct JaVal {
    float v = 0.0f;
    bool aframe = false;
    JaVal(float x) : v(x) {}
    JaVal(float x, bool af) : v(x), aframe(af) {}
};
inline JaVal aframe(float v) { return JaVal(v, true); }

// The ja / ja-no-eval macros (process-drawable-h.gc), as a builder that expands the same way:
// Ja().group(g).seek().frame_num(0.0f).no_eval().run(jc, clk) is
// (ja-no-eval :group! g :num! (seek!) :frame-num 0.0).
class Ja {
  public:
    Ja& chan(int c) { chan_ = c; return *this; }
    Ja& group(AnimId g) { group_ = g; return *this; }
    Ja& seek() { num_ = Num::seek; return *this; }                       // (seek!)
    Ja& seek(JaVal target) { num_ = Num::seek; a0_ = target; has_a0_ = true; return *this; }
    Ja& seek(JaVal target, float speed) { seek(target); a1_ = speed; has_a1_ = true; return *this; }
    Ja& seek_max(float speed) { num_ = Num::seek; seek_max_ = true; a1_ = speed; has_a1_ = true; return *this; }
    Ja& loop() { num_ = Num::loop; return *this; }                       // (loop!)
    Ja& min() { num_ = Num::min; return *this; }
    Ja& max() { num_ = Num::max; return *this; }
    Ja& identity(JaVal f) { num_ = Num::identity; a0_ = f; has_a0_ = true; return *this; }
    Ja& from_chan(int c) { num_ = Num::chan; a0_ = JaVal(static_cast<float>(c)); has_a0_ = true; return *this; }
    Ja& num_func(NumFunc nf) { num_func_ = nf; has_num_func_ = true; return *this; }
    Ja& frame_num(JaVal f) { frame_ = f; has_frame_ = true; return *this; }
    Ja& no_eval() { eval_ = false; return *this; }
    void run(JointControl& jc, const AnimClock& clk) const;

  private:
    enum class Num { unset, seek, loop, min, max, identity, chan };
    int chan_ = 0;
    AnimId group_ = AnimId::none;
    Num num_ = Num::unset;
    JaVal a0_{0.0f};
    float a1_ = 1.0f;
    bool has_a0_ = false, has_a1_ = false, seek_max_ = false;
    NumFunc num_func_ = NumFunc::none;
    bool has_num_func_ = false;
    JaVal frame_{0.0f};
    bool has_frame_ = false;
    bool eval_ = true;
};

// ---- the target's own channels ----
JointControl& skel(Target& self);
AnimClock anim_clock(const Target& self);
inline void ja(Target& self, const Ja& j) { j.run(skel(self), anim_clock(self)); }
// ja-post (process-drawable.gc): only the channel evaluation, the pose is the adapter's.
void ja_post(Target& self);

// ground-tween-initialize and ground-tween-update (target-util.gc): the stance's three blended
// channels. Only animation, so they live with the channels.
GroundTweenInfo& ground_tween_initialize(Target& self, GroundTweenInfo& info, uint32_t chan0,
                                         AnimId g0, AnimId g1, AnimId g2, AnimId g3, AnimId g4,
                                         float frame);
void ground_tween_update(Target& self, GroundTweenInfo& info, float x, float z, float speed);

// What plays now, for the adapter: the root channel's animation and frame (artist frame too), and
// every active channel with its blend weight.
struct AnimPlaying {
    AnimId group = AnimId::none;
    float frame = 0.0f;
    float aframe = 0.0f;
};
AnimPlaying anim_playing(const JointControl& jc);

} // namespace jak
