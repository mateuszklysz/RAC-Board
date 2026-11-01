// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// What Ratchet holds in his hand (the wrench, a weapon or a gadget): put away while he rides, as Jak
// puts his gun away on the board.
#pragma once

namespace rac1 {

// Finds the hand's pieces near `hero` (game units), by class: once, as the ride starts.
void find_hand(const float hero[3]);
// Hides or shows the pieces found.
void hide_hand(bool hidden);
// Shows them again and forgets them.
void release_hand();

} // namespace rac1
