// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// What every part of the adapter shares: the mod's API, the disc, the settings and the board's clock.
#pragma once

#include <string>
#include <vector>

#include <openrac/mod.hpp>

#include "jak/basic_types.hpp"

namespace rac1 {

// The button that gets on and off the board (setting board_button).
enum class Toggle { down, l2r2, r2 };

// The mod's settings (openrac-mod.json).
struct Settings {
    Toggle toggle = Toggle::down;
    std::vector<int> keys;     // keyboard binds that also toggle the board
    bool swap_l1_r1 = true;    // duck on R1, spin on L1
    bool trick_display = true; // trick names and points on screen
    bool board_camera = true;  // Jak II's board camera while riding
    bool heat_haze = true;     // the jet's heat haze behind the board
    bool trace = false;        // log states and hits
};

struct Context {
    openrac::Mod* mod = nullptr;
    bool pal = false;
    Settings settings;
    jak::TimeFrame clock = 3000; // the board's clock: 300 ticks a second, on all the time
};

Context& ctx();
inline openrac::Mod& mod() { return *ctx().mod; }
inline const Settings& settings() { return ctx().settings; }
// One game frame: in seconds, and in the board's clock ticks.
inline float frame_seconds() { return ctx().pal ? 1.0f / 50.0f : 1.0f / 60.0f; }
inline jak::TimeFrame frame_ticks() { return ctx().pal ? 6 : 5; }

// Reads the settings (openrac-mod.json) into ctx().settings.
void load_settings(openrac::Mod& mod);
// In a level, not paused or in a menu.
bool playing();
// A line in the trace log, when the trace setting is on.
void trace(const std::string& line);

} // namespace rac1
