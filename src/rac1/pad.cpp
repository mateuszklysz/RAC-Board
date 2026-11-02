// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/pad.hpp"

#include <cstring>
#include <utility>

#include "jak/cpad.hpp"
#include "rac1/context.hpp"
#include "rac1/ride.hpp"

namespace rac1 {

namespace {

// Port 0 as on_pad last saw it: buttons active high, in pad-buttons order.
struct PadState {
    uint16_t held = 0;
    uint8_t sticks[4] = {128, 128, 128, 128}; // right x, y, left x, y
    uint8_t pressure[12] = {};
    bool valid = false;
};
PadState g_pad;
bool g_was_held = false;

bool toggle_held(uint16_t held) {
    switch (settings().toggle) {
    case Toggle::up: return held & jak::pad::up;
    case Toggle::down: return held & jak::pad::down;
    case Toggle::r2: return held & jak::pad::r2;
    case Toggle::l2r2: return (held & jak::pad::l2) && (held & jak::pad::r2);
    }
    return false;
}

uint16_t toggle_bits() {
    switch (settings().toggle) {
    case Toggle::up: return jak::pad::up;
    case Toggle::down: return jak::pad::down;
    case Toggle::r2: return jak::pad::r2;
    case Toggle::l2r2: return jak::pad::l2 | jak::pad::r2;
    }
    return 0;
}

// Ratchet & Clank's buttons moved where the board expects them.
uint16_t jak_buttons(bool toggle) {
    const uint16_t held = g_pad.held;
    uint16_t b = held & static_cast<uint16_t>(~(jak::pad::r2 | jak::pad::l2 | toggle_bits()));
    if (settings().swap_l1_r1) {
        b &= static_cast<uint16_t>(~(jak::pad::l1 | jak::pad::r1));
        if (held & jak::pad::r1) b |= jak::pad::l1; // duck and grabs
        if (held & jak::pad::l1) b |= jak::pad::r1; // spins and flips
    }
    if (toggle) b |= jak::pad::r2; // Jak II gets on and off the board on R2
    return b;
}

} // namespace

void on_pad(int port, uint8_t* report, int size) {
    if (port != 0 || size < 18) return;
    g_pad.held = jak::ps2_buttons(report[0], report[1]);
    std::memcpy(g_pad.sticks, report + 2, 4);
    std::memcpy(g_pad.pressure, report + 6, 12);
    g_pad.valid = true;
    // The board's button is the mod's: in play the game does not see it (D-pad Up or Down at any time,
    // R2 or L2 and R2 only while riding, so strafing still works on foot). Menus keep it.
    const bool dpad = settings().toggle == Toggle::up || settings().toggle == Toggle::down;
    const bool take = dpad ? playing() : riding();
    if (take && (settings().toggle != Toggle::l2r2 || toggle_held(g_pad.held))) {
        const uint16_t bits = toggle_bits();
        report[0] |= static_cast<uint8_t>(bits & 0xFF);
        report[1] |= static_cast<uint8_t>(bits >> 8);
    }
}

bool toggle_pressed() {
    const bool held = g_pad.valid && toggle_held(g_pad.held);
    bool pressed = held && !g_was_held;
    g_was_held = held;
    if (mod().has_input() && mod().input().focused()) {
        for (int bind : settings().keys) pressed = pressed || mod().input().pressed(bind);
    }
    return pressed;
}

uint8_t pad_right_x() { return g_pad.valid ? g_pad.sticks[0] : 128; }

void feed_pad(jak::Target& t, bool toggle) {
    jak::cpad_set_hw(t.pad, jak_buttons(toggle), g_pad.sticks[0], g_pad.sticks[1], g_pad.sticks[2], g_pad.sticks[3]);
    std::memcpy(t.pad.abutton, g_pad.pressure, sizeof t.pad.abutton);
    if (settings().swap_l1_r1) std::swap(t.pad.abutton[jak::kAbL1], t.pad.abutton[jak::kAbR1]);
    jak::CpadService s;
    s.current_time = ctx().clock;
    s.target_fps = ctx().pal ? 50 : 60;
    jak::service_cpad(t.pad, s);
}

} // namespace rac1
