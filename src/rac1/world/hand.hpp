// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// What Ratchet holds in his hand (the wrench, a weapon or a gadget): put away while he rides, as Jak
// puts his gun away on the board.
#pragma once

#include <cstdint>

namespace rac1 {

// Each frame he rides, before the game's item code (hero_set_pose runs it): the wrench in his hand.
// A weapon or gadget he got on with is put away (the game swaps it while he rides, its pieces
// hidden), and an item asked for meanwhile (the pause menu, a pickup) waits until he gets off.
// `item` is the one in his hand now (`item` as the hero's pose has it, -1 for none).
void hold_hand(int32_t item);
// The ride is over: the item asked for while he rode is asked for again, and a wrench request of the
// mod's still open is taken back.
void release_hold();

// Finds the hand's pieces near `hero` (game units), by class: as the ride starts, and with `add`
// for an item he takes out while riding (added to those found).
void find_hand(const float hero[3], bool add = false);
// Hides or shows the pieces found.
void hide_hand(bool hidden);
// Shows them again and forgets them.
void release_hand();

} // namespace rac1
