// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// What the target's movement core (src/jak/movement*) calls in the board's part (src/board/*),
// declared here and implemented there: the state machine and the animation channels. Each line
// names the GOAL expression it stands for (OpenGOAL, jak-project, ISC License, Copyright (c)
// 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
#pragma once

#include "jak/basic_types.hpp"

namespace jak {

struct Target;

// owned by board: (-> self next-state name), or #f (null Symbol) when there is no next state.
Symbol next_state_name(const Target& self);
// owned by board: (ja-post): the animation's root motion and joint update after the move.
void ja_post(Target& self);
// owned by board: (joint-points) (logic-target.gc): the joint positions the target exposes.
void joint_points(Target& self);
// owned by board: (target-board-joint-points) (target-board.gc), the end of target-no-ja-move-post.
void target_board_joint_points(Target& self);
// owned by board: mod-var-jump's look-up of the current animation group's 'collide-offset lump at
// (ja-aframe-num 0). Returns 0 when there is no (ja-group) or it has no extra lump table, 1 with the
// offset in `out` when the lump is there, 2 when the table is there but the lump is not.
int ja_group_collide_offset(Target& self, Vec& out);

} // namespace jak
