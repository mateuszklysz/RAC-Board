// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// What every part of the adapter shares: the mod's API, the disc, the settings and the board's clock.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <openrac/mod.hpp>

#include "jak/basic_types.hpp"

namespace rac1 {

// The button that gets on and off the board (setting board_button).
enum class Toggle { up, down, l2r2, r2 };

// A board button as pad buttons (Jak II's pad bits): one of `any` (if any) and all of `all`.
struct PadButtons {
    uint16_t any = 0;
    uint16_t all = 0;
    bool held(uint16_t pad) const {
        return (any | all) != 0 && (any == 0 || (pad & any) != 0) && (pad & all) == all;
    }
    uint16_t bits() const { return static_cast<uint16_t>(any | all); }
};

// The mod's settings (openrac-mod.json).
struct Settings {
    Toggle toggle = Toggle::up;
    std::vector<int> keys;     // keyboard binds that also toggle the board
    // The board's buttons (settings trick_button, grind_button, grab_button) for Jak II's R1 (spins,
    // flips), Circle (grind) and L1 (duck, grabs).
    PadButtons trick;
    PadButtons grind;
    PadButtons grab;
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
// A field of the hero's record, g_hero (PAL D_0013F450, NTSC-U D_0013F350).
openrac::gaddr hero_field(uint32_t offset);

} // namespace rac1
