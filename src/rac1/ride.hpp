// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The ride: Jak II's target on the board in Ratchet's place, from getting on to getting off.
#pragma once

namespace rac1 {

bool riding();
// Gets on when want-to-board? says yes (the button pressed, Ratchet free and with room); false
// otherwise, and the game's update runs as usual.
bool try_start_ride();
// One frame on the board; false once the board's code has left for one of the target's own states.
bool tick_ride(bool toggle);
// Ratchet where the target is, in his riding pose, and the board under his feet.
void show_ride();
void end_ride();
// A frame off the board: notes when Ratchet stands on the ground (want-to-board? needs it).
void note_on_foot();
// The level the ride started in is gone.
bool ride_level_changed();

} // namespace rac1
