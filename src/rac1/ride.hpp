// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The ride: Jak II's target on the board in Ratchet's place, from getting on to getting off.
#pragma once

namespace rac1 {

bool riding();
// Gets on when want-to-board? says yes (the button pressed, Ratchet free and with room); false
// otherwise, and the game's update runs as usual.
bool try_start_ride();
enum class RideStep {
    riding, // still on the board
    left,   // the board's code got off (or the board met one of the game's hazards)
    taken,  // the game took Ratchet over (a scene, a death, a move of its own): he stays where it put him
};
// One frame on the board.
RideStep tick_ride(bool toggle);
// Ratchet where the target is, in his riding pose, and the board under his feet.
// `last`: the ride's last frame, his animation handed back to the game's.
void show_ride(bool last);
void end_ride();
// A frame off the board: the riding pose eased out over the game's animation after a ride.
void pose_out_tick();
// A frame off the board: notes when Ratchet stands on the ground (want-to-board? needs it).
void note_on_foot();
// The level the ride started in is gone.
bool ride_level_changed();

} // namespace rac1
