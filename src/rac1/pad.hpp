// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The pad: what port 0 sends, the board's own button, and the pad as Jak II's board reads it.
#pragma once

#include <cstdint>

#include "jak/target.hpp"

namespace rac1 {

// on_pad's hook: keeps port 0's report, and hides the board's button from the game while it is the mod's.
void on_pad(int port, uint8_t* report, int size);
// The board's button (or key) went down this frame. Call once a frame.
bool toggle_pressed();
// The right stick's x as port 0 last sent it (128 the middle), for the board's camera.
uint8_t pad_right_x();
// The pad into the target's cpad for one frame (read-pad's input), the board's button as R2.
void feed_pad(jak::Target& t, bool toggle);

} // namespace rac1
