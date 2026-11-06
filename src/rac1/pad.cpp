// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/pad.hpp"

#include <cstring>

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

// The pad as the board reads it: Cross jumps, the trick, grind and grab buttons are Jak II's R1,
// Circle and L1. The rest (Circle, Square, Triangle) stay the game's.
uint16_t jak_buttons(bool toggle) {
    const uint16_t held = g_pad.held & static_cast<uint16_t>(~toggle_bits());
    uint16_t b = held & jak::pad::x;
    const Settings& s = settings();
    const bool grind = s.grind.held(held);
    // R1 and Cross grind, not flip: a trick button inside the grind's buttons does no trick then.
    const bool trick = s.trick.held(held) && !(grind && (s.grind.bits() & s.trick.bits()));
    if (trick) b |= jak::pad::r1;                // spins and flips
    if (grind) b |= jak::pad::circle;            // grind
    if (s.grab.held(held)) b |= jak::pad::l1;    // duck and grabs
    if (toggle) b |= jak::pad::r2;                                // Jak II gets on and off the board on R2
    return b;
}

// Ratchet stands where the game steers him and reads the D-pad itself (the Trespasser puzzle).
bool in_scripted_stand() {
    openrac_pose pose{};
    return mod().hero_pose(pose) && pose.state == kHeroScriptedStand;
}

// The game sees `bits` released in a pad report (its buttons are active low).
void release(uint8_t* report, uint16_t bits) {
    report[0] |= static_cast<uint8_t>(bits & 0xFF);
    report[1] |= static_cast<uint8_t>(bits >> 8);
}

} // namespace

void on_pad(int port, uint8_t* report, int size) {
    if (port != 0 || size < 18) return;
    g_pad.held = jak::ps2_buttons(report[0], report[1]);
    std::memcpy(g_pad.sticks, report + 2, 4);
    std::memcpy(g_pad.pressure, report + 6, 12);
    g_pad.valid = true;
    // The board's button is the mod's: in play the game does not see it (D-pad Up or Down at any time,
    // R2 or L2 and R2 only while riding, so strafing still works on foot). Menus keep it, and so does
    // the Trespasser puzzle (it picks a ring with D-pad Up and Down). Only a held button needs
    // hiding, so the hero's state is read only then.
    if (toggle_held(g_pad.held)) {
        const bool dpad = settings().toggle == Toggle::up || settings().toggle == Toggle::down;
        const bool take = dpad ? playing() && (riding() || !in_scripted_stand()) : riding();
        if (take) release(report, toggle_bits());
    }
    // While riding, L1, L2, R1 and R2 are the board's (tricks, grind, grabs), and Circle and Triangle
    // are nobody's: he rides with the wrench put away, so no weapon or gadget comes out and the quick
    // select stays shut. The game, and other mods (lock-strafe turns the camera ahead on L2 or R2),
    // see them released until he gets off. The board read them above. The pause menu keeps them.
    if (riding() && mod().play_status().phase != OPENRAC_PLAY_MENU) {
        constexpr uint16_t kept = jak::pad::l1 | jak::pad::l2 | jak::pad::r1 | jak::pad::r2 |
                                  jak::pad::circle | jak::pad::triangle;
        release(report, kept);
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
    jak::CpadService s;
    s.current_time = ctx().clock;
    s.target_fps = ctx().pal ? 50 : 60;
    jak::service_cpad(t.pad, s);
}

} // namespace rac1
