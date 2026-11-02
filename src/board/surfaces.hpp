// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The JET-Board's surfaces (*board-*-mods*, target-board.gc), ported from OpenGOAL (jak-project, ISC
// License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). A board state sets
// `self.control.mod_surface` to one of them; build-conversions combines it with the ground's
// (src/jak/surface.hpp). They are mutable, as in GOAL.
#pragma once

#include "jak/surface.hpp"

namespace jak {

extern Surface board_walk_mods;      // *board-walk-mods*
extern Surface board_duck_mods;      // *board-duck-mods*
extern Surface board_air_mods;       // *board-air-mods*
extern Surface board_jump_mods;      // *board-jump-mods*
extern Surface board_ride_jump_mods; // *board-ride-jump-mods*
extern Surface& board_duck_jump_mods; // *board-duck-jump-mods*: the same object as *board-jump-mods*
extern Surface board_spin_mods;      // *board-spin-mods*: a copy of *board-jump-mods*, changed
extern Surface board_spin_post_mods; // *board-spin-post-mods*: a copy of *board-jump-mods*, changed
extern Surface board_flip_mods;      // *board-flip-mods*: a copy of *board-duck-jump-mods*, changed
extern Surface board_wall_kick_mods; // *board-wall-kick-mods*
extern Surface board_halfpipe_mods;  // *board-halfpipe-mods*
extern Surface board_turn_to_mods;   // *board-turn-to-mods*
extern Surface board_ride_mods;      // *board-ride-mods*

// The mult-hook lambdas (shared as GOAL shares them: walk's is also duck's and turn-to's).
void board_walk_mult_hook(Target& self, Surface& a0, const Surface& a1, const Surface& a2, int a3);
void board_ride_jump_mult_hook(Target& self, Surface& a0, const Surface& a1, const Surface& a2,
                               int a3);
void board_wall_kick_mult_hook(Target& self, Surface& a0, const Surface& a1, const Surface& a2,
                               int a3);
void board_ride_mult_hook(Target& self, Surface& a0, const Surface& a1, const Surface& a2, int a3);

} // namespace jak
