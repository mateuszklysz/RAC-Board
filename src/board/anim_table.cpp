// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The animations the board's code names, with what the logic needs of each: the frame count
// (art-joint-anim frames num-frames), the artist numbering (artist-base, artist-step: ja-aframe) and
// the speed. The names and their order are OpenGOAL's (goal_src/jak2/engine/data/art-elts.gc,
// jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
//
// The numbers are Jak II's art data, which lives in the game's files and not in OpenGOAL's code or
// configuration (art-elts.gc and art-group-info.min.json give only names and indices). They were read
// from a legally owned copy (NTSC-U, ART.CGO: jak-board+0-ag and board-ag): only these numbers, no
// animation. jakb-board-attack-pegasus is in a story level's art and stays a placeholder (31 frames,
// artist numbering from 0 in steps of 1, speed 1).
#include "board/anim.hpp"

namespace jak {

namespace {

// clang-format off
const AnimInfo kAnims[] = {
    // jakb-board-stance-ja: identity at aframe 0 in target-board-grab.
    {"jakb-board-stance-ja", 1, 0.0f, 1.0f, 0.5f, false},
    // jakb-board-duck-turn-ja: the stance's duck channel (num-func-chan of channel 0); halfpipe falling plays aframe 0.
    {"jakb-board-duck-turn-ja", 11, -10.0f, 2.0f, 0.5000009536743164f, false},
    // jakb-board-turn-ja: target-board-turn-anim plays aframes -10..10 (turn-frames).
    {"jakb-board-turn-ja", 11, -10.0f, 2.0f, 0.5000009536743164f, false},
    // jakb-board-jump-kick-ja: played to the end.
    {"jakb-board-jump-kick-ja", 16, 0.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-jump-ja: aframes 8, 10, 12 in the halfpipe; the jump code reads (- 10 aframe).
    {"jakb-board-jump-ja", 11, 0.0f, 2.0999999046325684f, 0.4761914312839508f, false},
    // jakb-board-jump-loop-ja: looped.
    {"jakb-board-jump-loop-ja", 6, 21.0f, 2.0f, 0.5000004768371582f, false},
    // jakb-board-hit-ja: played to the end; also target-hit-move's length.
    {"jakb-board-hit-ja", 10, 0.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-flip-forward-ja: seeks aframe 15, then on from aframe 15.
    {"jakb-board-flip-forward-ja", 11, 0.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-turn-left-ja: ground-tween channel 2 (left), frames from channel 0.
    {"jakb-board-turn-left-ja", 11, -10.0f, 2.0f, 0.5000009536743164f, false},
    // jakb-board-turn-right-ja: ground-tween channel 2 (right).
    {"jakb-board-turn-right-ja", 11, -10.0f, 2.0f, 0.5000009536743164f, false},
    // jakb-board-turn-up-ja: ground-tween channel 1 (up).
    {"jakb-board-turn-up-ja", 11, -10.0f, 2.0f, 0.5000009536743164f, false},
    // jakb-board-turn-down-ja: ground-tween channel 1 (down).
    {"jakb-board-turn-down-ja", 11, -10.0f, 2.0f, 0.5000009536743164f, false},
    // jakb-board-hit-forward-ja: only compared with.
    {"jakb-board-hit-forward-ja", 10, 0.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-ride-turn-back-ja: ride-edge plays aframes -5..5.
    {"jakb-board-ride-turn-back-ja", 6, -5.0f, 2.0f, 0.5000004768371582f, false},
    // jakb-board-ride-turn-front-ja: ride-edge plays aframes -5..5.
    {"jakb-board-ride-turn-front-ja", 6, -5.0f, 2.0f, 0.5000004768371582f, false},
    // jakb-board-ride-turn-right-ja: ride-edge plays aframes -5..5.
    {"jakb-board-ride-turn-right-ja", 6, -5.0f, 2.0f, 0.5000004768371582f, false},
    // jakb-board-ride-turn-left-ja: ride-edge plays aframes -5..5.
    {"jakb-board-ride-turn-left-ja", 6, -5.0f, 2.0f, 0.5000004768371582f, false},
    // jakb-board-get-on-ja: played to the end at 1-2x.
    {"jakb-board-get-on-ja", 14, 1.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-get-off-ja: aframes 11, 14 and 24 matter.
    {"jakb-board-get-off-ja", 11, 10.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-grenade-ja: seeks aframe 19 (story only).
    {"jakb-board-grenade-ja", 13, 0.0f, 3.0f, 0.1666671484708786f, false},
    // jakb-board-nosegrab-ja: played to the end.
    {"jakb-board-nosegrab-ja", 7, 0.0f, 2.0f, 0.2500002384185791f, false},
    // jakb-board-method-ja: played to the end.
    {"jakb-board-method-ja", 5, 1.0f, 2.0f, 0.2500002384185791f, false},
    // jakb-board-kickflip-a-ja: played to the end.
    {"jakb-board-kickflip-a-ja", 3, 0.0f, 2.5f, 0.2000001221895218f, false},
    // jakb-board-spin-ja: played to the end at 0.95x.
    {"jakb-board-spin-ja", 12, 0.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-air-turn-ja: the stance's duck channel when ducking below 0.
    {"jakb-board-air-turn-ja", 11, -10.0f, 2.0f, 0.5000009536743164f, false},
    // jakb-board-flip-backward-ja: seeks aframe 10, then on from aframe 10.
    {"jakb-board-flip-backward-ja", 11, 0.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-flip-forward-loop-ja: played to the end per extra flip.
    {"jakb-board-flip-forward-loop-ja", 7, 15.0f, 2.0f, 0.2500002384185791f, false},
    // jakb-board-flip-backward-loop-ja: played to the end per extra flip.
    {"jakb-board-flip-backward-loop-ja", 7, 10.0f, 2.0f, 0.2500002384185791f, false},
    // jakb-board-method-loop-ja: looped.
    {"jakb-board-method-loop-ja", 9, 9.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-method-end-ja: played to the end at 1.5x.
    {"jakb-board-method-end-ja", 5, 25.0f, 2.25f, 0.2222224622964859f, false},
    // jakb-board-nosegrab-loop-ja: looped.
    {"jakb-board-nosegrab-loop-ja", 13, 12.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-nosegrab-end-ja: played to the end at 1.5x.
    {"jakb-board-nosegrab-end-ja", 4, 13.0f, 2.0f, 0.25000011920928955f, false},
    // jakb-board-noseflip-ja: seeks aframe 20.
    {"jakb-board-noseflip-ja", 15, 0.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-jump-kickoff-ja: the wall kick lands after aframe 31.
    {"jakb-board-jump-kickoff-ja", 12, 16.0f, 2.090909004211426f, 0.2391309142112732f, false},
    // jakb-board-hit-elec-ja: target-hit-move's length when shocked.
    {"jakb-board-hit-elec-ja", 9, 0.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-get-on-land-ja: played to the end at 1.8x.
    {"jakb-board-get-on-land-ja", 3, 1.0f, 1.0f, 0.33333298563957214f, false},
    // jakb-board-get-off-pre-ja: played to the end at 1.5x.
    {"jakb-board-get-off-pre-ja", 6, 0.0f, 2.0f, 0.2500002384185791f, false},
    // jakb-board-kickflip-b-ja: played to the end at 1.05x.
    {"jakb-board-kickflip-b-ja", 12, 5.0f, 2.0f, 0.2500004768371582f, false},
    // jakb-board-kickflip-c-ja: played to the end.
    {"jakb-board-kickflip-c-ja", 1, 28.0f, 1.0f, 0.5f, false},
    // jakb-board-hit-get-off-ja: seeks aframe 24 at 2.5x.
    {"jakb-board-hit-get-off-ja", 16, 0.0f, 2.0f, 0.2500004768371582f, false},
    // TODO(art): jakb-board-attack-pegasus-ja: frames, artist-base, artist-step, speed unknown; aframes 0..30, 17 closes the board (story only).
    {"jakb-board-attack-pegasus-ja", 31, 0.0f, 1.0f, 1.0f, true},
    // board-board-idle-ja: the board's skeleton (board-ag).
    {"board-board-idle-ja", 1, 0.0f, 1.0f, 1.0f, false},
    // board-open-ja: the board's skeleton, played to the end.
    {"board-open-ja", 16, 0.0f, 1.0f, 0.5f, false},
    // board-close-ja: the board's skeleton, played to the end.
    {"board-close-ja", 8, 15.0f, 1.0f, 0.5f, false},
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
