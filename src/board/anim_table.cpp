// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The animations the board's code names, with what the logic needs of each: the frame count
// (art-joint-anim frames num-frames), the artist numbering (artist-base, artist-step: ja-aframe) and
// the speed. The names and their order are OpenGOAL's (goal_src/jak2/engine/data/art-elts.gc,
// jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
//
// The numbers are Jak II's art data, which lives in the game's files and not in OpenGOAL's code or
// configuration (art-elts.gc and art-group-info.min.json give only names and indices). Until they are
// filled in from a legally owned copy, every entry is a placeholder: 30 frames, artist numbering from
// 0 in steps of 1, speed 1, except where the code's own ja-aframe calls show the range it expects
// (the turn animations span the bank's turn-frames, -10..10; the rail ones -5..5).
#include "board/anim.hpp"

namespace jak {

namespace {

// clang-format off
const AnimInfo kAnims[] = {
    // TODO(art): jakb-board-stance-ja: frames, artist-base, artist-step, speed unknown; identity at aframe 0 in target-board-grab.
    {"jakb-board-stance-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-duck-turn-ja: frames, artist-base, artist-step, speed unknown; the stance's duck channel (num-func-chan of channel 0); halfpipe falling plays aframe 0.
    {"jakb-board-duck-turn-ja", 21, -10.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-turn-ja: frames, artist-base, artist-step, speed unknown; target-board-turn-anim plays aframes -10..10 (turn-frames).
    {"jakb-board-turn-ja", 21, -10.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-jump-kick-ja: frames, artist-base, artist-step, speed unknown; played to the end.
    {"jakb-board-jump-kick-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-jump-ja: frames, artist-base, artist-step, speed unknown; aframes 8, 10, 12 in the halfpipe; the jump code reads (- 10 aframe).
    {"jakb-board-jump-ja", 21, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-jump-loop-ja: frames, artist-base, artist-step, speed unknown; looped.
    {"jakb-board-jump-loop-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-hit-ja: frames, artist-base, artist-step, speed unknown; played to the end; also target-hit-move's length.
    {"jakb-board-hit-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-flip-forward-ja: frames, artist-base, artist-step, speed unknown; seeks aframe 15, then on from aframe 15.
    {"jakb-board-flip-forward-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-turn-left-ja: frames, artist-base, artist-step, speed unknown; ground-tween channel 2 (left), frames from channel 0.
    {"jakb-board-turn-left-ja", 21, -10.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-turn-right-ja: frames, artist-base, artist-step, speed unknown; ground-tween channel 2 (right).
    {"jakb-board-turn-right-ja", 21, -10.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-turn-up-ja: frames, artist-base, artist-step, speed unknown; ground-tween channel 1 (up).
    {"jakb-board-turn-up-ja", 21, -10.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-turn-down-ja: frames, artist-base, artist-step, speed unknown; ground-tween channel 1 (down).
    {"jakb-board-turn-down-ja", 21, -10.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-hit-forward-ja: frames, artist-base, artist-step, speed unknown; only compared with.
    {"jakb-board-hit-forward-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-ride-turn-back-ja: frames, artist-base, artist-step, speed unknown; ride-edge plays aframes -5..5.
    {"jakb-board-ride-turn-back-ja", 11, -5.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-ride-turn-front-ja: frames, artist-base, artist-step, speed unknown; ride-edge plays aframes -5..5.
    {"jakb-board-ride-turn-front-ja", 11, -5.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-ride-turn-right-ja: frames, artist-base, artist-step, speed unknown; ride-edge plays aframes -5..5.
    {"jakb-board-ride-turn-right-ja", 11, -5.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-ride-turn-left-ja: frames, artist-base, artist-step, speed unknown; ride-edge plays aframes -5..5.
    {"jakb-board-ride-turn-left-ja", 11, -5.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-get-on-ja: frames, artist-base, artist-step, speed unknown; played to the end at 1-2x.
    {"jakb-board-get-on-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-get-off-ja: frames, artist-base, artist-step, speed unknown; aframes 11, 14 and 24 matter.
    {"jakb-board-get-off-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-grenade-ja: frames, artist-base, artist-step, speed unknown; seeks aframe 19 (story only).
    {"jakb-board-grenade-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-nosegrab-ja: frames, artist-base, artist-step, speed unknown; played to the end.
    {"jakb-board-nosegrab-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-method-ja: frames, artist-base, artist-step, speed unknown; played to the end.
    {"jakb-board-method-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-kickflip-a-ja: frames, artist-base, artist-step, speed unknown; played to the end.
    {"jakb-board-kickflip-a-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-spin-ja: frames, artist-base, artist-step, speed unknown; played to the end at 0.95x.
    {"jakb-board-spin-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-air-turn-ja: frames, artist-base, artist-step, speed unknown; the stance's duck channel when ducking below 0.
    {"jakb-board-air-turn-ja", 21, -10.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-flip-backward-ja: frames, artist-base, artist-step, speed unknown; seeks aframe 10, then on from aframe 10.
    {"jakb-board-flip-backward-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-flip-forward-loop-ja: frames, artist-base, artist-step, speed unknown; played to the end per extra flip.
    {"jakb-board-flip-forward-loop-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-flip-backward-loop-ja: frames, artist-base, artist-step, speed unknown; played to the end per extra flip.
    {"jakb-board-flip-backward-loop-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-method-loop-ja: frames, artist-base, artist-step, speed unknown; looped.
    {"jakb-board-method-loop-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-method-end-ja: frames, artist-base, artist-step, speed unknown; played to the end at 1.5x.
    {"jakb-board-method-end-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-nosegrab-loop-ja: frames, artist-base, artist-step, speed unknown; looped.
    {"jakb-board-nosegrab-loop-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-nosegrab-end-ja: frames, artist-base, artist-step, speed unknown; played to the end at 1.5x.
    {"jakb-board-nosegrab-end-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-noseflip-ja: frames, artist-base, artist-step, speed unknown; seeks aframe 20.
    {"jakb-board-noseflip-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-jump-kickoff-ja: frames, artist-base, artist-step, speed unknown; the wall kick lands after aframe 31.
    {"jakb-board-jump-kickoff-ja", 40, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-hit-elec-ja: frames, artist-base, artist-step, speed unknown; target-hit-move's length when shocked.
    {"jakb-board-hit-elec-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-get-on-land-ja: frames, artist-base, artist-step, speed unknown; played to the end at 1.8x.
    {"jakb-board-get-on-land-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-get-off-pre-ja: frames, artist-base, artist-step, speed unknown; played to the end at 1.5x.
    {"jakb-board-get-off-pre-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-kickflip-b-ja: frames, artist-base, artist-step, speed unknown; played to the end at 1.05x.
    {"jakb-board-kickflip-b-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-kickflip-c-ja: frames, artist-base, artist-step, speed unknown; played to the end.
    {"jakb-board-kickflip-c-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-hit-get-off-ja: frames, artist-base, artist-step, speed unknown; seeks aframe 24 at 2.5x.
    {"jakb-board-hit-get-off-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): jakb-board-attack-pegasus-ja: frames, artist-base, artist-step, speed unknown; aframes 0..30, 17 closes the board (story only).
    {"jakb-board-attack-pegasus-ja", 31, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): board-board-idle-ja: frames, artist-base, artist-step, speed unknown; the board's skeleton (board-ag).
    {"board-board-idle-ja", 2, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): board-open-ja: frames, artist-base, artist-step, speed unknown; the board's skeleton, played to the end.
    {"board-open-ja", 30, 0.0f, 1.0f, 1.0f, true},
    // TODO(art): board-close-ja: frames, artist-base, artist-step, speed unknown; the board's skeleton, played to the end.
    {"board-close-ja", 30, 0.0f, 1.0f, 1.0f, true},
};
// clang-format on
static_assert(sizeof(kAnims) / sizeof(kAnims[0]) == static_cast<int>(AnimId::count),
              "one entry per AnimId");

// What a channel without an animation reads: GOAL would read through #f; nothing the board does
// gets there, so this only keeps a stray read harmless.
const AnimInfo kNone = {"none", 1, 0.0f, 1.0f, 1.0f, true};

} // namespace

const AnimInfo& anim_info(AnimId id) {
    if (id == AnimId::none || id >= AnimId::count) {
        return kNone;
    }
    return kAnims[static_cast<int>(id)];
}

} // namespace jak
