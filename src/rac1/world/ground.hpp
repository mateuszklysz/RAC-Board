// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// What the board stands on, as Ratchet & Clank sees it: the surface's kind (the game's own hazards)
// and the moby it belongs to (a moving platform).
#pragma once

#include <cstdint>

#include "jak/target.hpp"

namespace rac1 {

struct Ground {
    bool found = false;
    uint32_t surface = 0; // the collision byte's low 5 bits: the surface the hero's states read
    int moby = -1;        // the moby slot it moves with, -1 for the level's own collision
};

// The floor right under `feet` (game units), within a short reach.
Ground ground_under(const float feet[3]);

// A surface the game itself acts on when Ratchet touches it: water, a death floor, quicksand and
// the other sinking liquids. The board gives Ratchet back to the game there, so the game's own
// rules (drowning, falling to death, respawning) apply as they do on foot.
bool game_hazard(uint32_t surface);

// The height under which the game ends a fall (hero_update_transitions.c: below it, with the ground
// over 2 units away, Ratchet falls to his death).
float fall_death_height();

// Moving platforms: the board moves and turns with the moby it stands on, as Jak II's riders do.
// carry() before the board's frame, note() after it.
void carry(jak::Target& t);
void note_ground(const jak::Target& t, const Ground& g);
void forget_ground();

} // namespace rac1
