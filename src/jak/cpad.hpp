// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's pad (engine/ps2/pad.gc): cpad-info, the per-frame service that turns the pad's bytes into
// held and pressed buttons and the left stick's direction and speed, and the routines the target
// reads them with. Ported from OpenGOAL (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL
// Team; THIRD_PARTY.md).
//
// The adapter (src/rac1) fills the hardware part of a CpadInfo each tick (cpad_set_hw) and calls
// service_cpad once per game frame, as service-cpads runs once per frame in Jak II; the target reads
// it through `self.control.cpad`.
#pragma once

#include "jak/basic_types.hpp"

namespace jak {

// pad-buttons (pad.gc): bits 0-15 are the PS2 pad's buttons in the order the pad sends them (the
// first byte select, L3, R3, start, up, right, down, left; the second L2, R2, L1, R1, triangle,
// circle, cross, square), 16-23 the sticks pushed past 30/225, 24 the menu's confirm.
namespace pad {
constexpr uint32_t select = 1u << 0, l3 = 1u << 1, r3 = 1u << 2, start = 1u << 3, up = 1u << 4,
                   right = 1u << 5, down = 1u << 6, left = 1u << 7, l2 = 1u << 8, r2 = 1u << 9,
                   l1 = 1u << 10, r1 = 1u << 11, triangle = 1u << 12, circle = 1u << 13,
                   x = 1u << 14, square = 1u << 15, l_analog_up = 1u << 16,
                   l_analog_right = 1u << 17, l_analog_down = 1u << 18, l_analog_left = 1u << 19,
                   r_analog_up = 1u << 20, r_analog_right = 1u << 21, r_analog_down = 1u << 22,
                   r_analog_left = 1u << 23, confirm = 1u << 24;
} // namespace pad

// abutton-idx (pad.gc): the pressure bytes' order.
enum AbuttonIdx : int {
    kAbRight = 0, kAbLeft, kAbUp, kAbDown, kAbTriangle, kAbCircle, kAbX, kAbSquare, kAbL1, kAbR1,
    kAbL2, kAbR2
};

// cpad-info (pad.gc) with its hw-cpad part, and the cpad-history the PC port keeps per pad for its
// high frame rate fix (recently-pressed?). The vibration fields are left out: the board never buzzes.
struct CpadInfo {
    // hw-cpad: what the pad sent this frame.
    uint8_t valid = 0;   // bit 7 set: no pad
    uint8_t status = 0x73; // high nibble 7: a DualShock in analog mode
    uint16_t button0 = 0; // held buttons, bit i = pad-buttons bit i (active high)
    uint8_t rightx = 128, righty = 128, leftx = 128, lefty = 128; // 0..255, 128 centred, y down
    uint8_t abutton[12] = {}; // pressure, abutton-idx order

    int32_t number = 0;
    uint32_t button0_abs[3] = {};       // held: this frame, the one before, the one before that
    uint32_t button0_shadow_abs[1] = {};
    uint32_t button0_rel[3] = {};       // went down: this frame, the one before, ...
    float stick0_dir = 0.0f;   // the left stick's direction, rotation units, 0 = up, + = left
    float stick0_speed = 0.0f; // its push, 0..1, 0 inside the dead zone
    TimeFrame change_time = 0;
    uint8_t old_rightx[2] = {}, old_righty[2] = {}, old_leftx[2] = {}, old_lefty[2] = {};

    // cpad-history (pad.gc, PC port): 15 frames of held and pressed buttons.
    uint32_t history_abs[15] = {};
    uint32_t history_rel[15] = {};
};

// What service-cpads reads from the PC port's settings and the clock.
struct CpadService {
    TimeFrame current_time = 0; // (get-current-time)
    int target_fps = 60;        // (-> *pc-settings* target-fps)
    float stick_deadzone = 0.3f; // (-> *pc-settings* stick-deadzone), OpenGOAL's default
};

// The hardware bytes, as the adapter has them: held buttons active high in pad-buttons order, the
// sticks 0..255 (x right, y down). A PS2 pad's two button bytes are active low: pass
// ps2_buttons(byte0, byte1).
void cpad_set_hw(CpadInfo& pad, uint16_t buttons, uint8_t rightx, uint8_t righty, uint8_t leftx,
                 uint8_t lefty);
inline uint16_t ps2_buttons(uint8_t byte0, uint8_t byte1) {
    return static_cast<uint16_t>(~(byte0 | (byte1 << 8)));
}

void cpad_invalid(CpadInfo& pad);                       // cpad-invalid!
void service_cpad(CpadInfo& pad, const CpadService& s); // service-cpads, for one pad
float analog_input(int in, float offset, float center_val, float max_val, float out_range);

// The macros (pad.gc): cpad-hold?, cpad-pressed?, recently-pressed? (frames 15 by default),
// cpad-clear!.
inline bool cpad_hold_p(const CpadInfo& pad, uint32_t buttons) {
    return (pad.button0_abs[0] & buttons) != 0;
}
inline bool cpad_pressed_p(const CpadInfo& pad, uint32_t buttons) {
    return (pad.button0_rel[0] & buttons) != 0;
}
bool recently_pressed_p(const CpadInfo& pad, uint32_t buttons, int frames = 15);
inline void cpad_clear(CpadInfo& pad, uint32_t buttons) {
    pad.button0_rel[0] &= ~buttons;
    pad.button0_abs[0] &= ~buttons;
}

} // namespace jak
