// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// What Ratchet holds in his hand (the wrench, a weapon or a gadget): put away while he rides, as Jak
// puts his gun away on the board.
#pragma once

#include <cstdint>

namespace rac1 {

// The item in his hand (`item` as the hero's pose has it, -1 for none) is the wrench.
bool is_wrench(int32_t item);
// Nothing in his hand to put away before he rides: the wrench, or no item.
bool hand_free(int32_t item);
// Asks the game to put the item in his hand away and take the wrench, as it does itself for its
// own vehicles; `false` takes back a request of the mod's the game has not carried out yet.
void ask_for_wrench(bool ask);

// Finds the hand's pieces near `hero` (game units), by class: as the ride starts, and with `add`
// for an item he takes out while riding (added to those found).
void find_hand(const float hero[3], bool add = false);
// Hides or shows the pieces found.
void hide_hand(bool hidden);
// Shows them again and forgets them.
void release_hand();

} // namespace rac1
