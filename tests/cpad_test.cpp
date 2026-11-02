// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's pad service (src/jak/cpad.cpp) against values worked out from pad.gc: the stick's
// direction and speed, the dead zone, held and pressed buttons, the stick buttons, the PC port's
// recently-pressed? history and analog-input.
#include "check.hpp"
#include "jak/cpad.hpp"

using namespace jak;

int main() {
    CpadService s;
    s.current_time = 1000;

    // The PS2's active-low bytes: only cross down in the second byte.
    CHECK(ps2_buttons(0xFF, 0xBF) == pad::x);

    // Stick full right: x = 127/128, y = (127 - 128)/128; dir = atan(-x, y), both negative:
    // -32768 + atan0(0.9921875, 0.0078125) = -32768 + 16302.0 (0.007874 rad = 82.0 units off a
    // quarter turn).
    CpadInfo p;
    cpad_set_hw(p, 0, 128, 128, 255, 128);
    service_cpad(p, s);
    CHECK_NEAR(p.stick0_dir, -16466.0, 1.5);
    CHECK_NEAR(p.stick0_speed, std::sqrt(0.9921875 * 0.9921875 + 0.0078125 * 0.0078125), 1e-6);
    CHECK(p.button0_abs[0] & pad::l_analog_right);

    // Centred: y is -1/128 (127 - 128), inside the 0.3 dead zone: speed 0; the direction is
    // atan(-0.0, -0.0078125): -0.0 is not < 0, so 32768 - atan0(-0.0, ...) = 32768.
    cpad_set_hw(p, 0, 128, 128, 128, 128);
    service_cpad(p, s);
    CHECK(p.stick0_speed == 0.0f);
    CHECK_NEAR(p.stick0_dir, 32768.0, 1.0);

    // Full up (lefty 0): dir 0, speed 127/128.
    cpad_set_hw(p, 0, 128, 128, 128, 0);
    service_cpad(p, s);
    CHECK_NEAR(p.stick0_dir, 0.0, 1.0);
    CHECK_NEAR(p.stick0_speed, 0.9921875, 1e-6);
    CHECK(p.button0_abs[0] & pad::l_analog_up);

    // Not an analog pad: sticks centred, speed 0.
    p.status = 0x41;
    cpad_set_hw(p, 0, 0, 0, 0, 0);
    service_cpad(p, s);
    CHECK(p.leftx == 128 && p.stick0_speed == 0.0f);
    p.status = 0x73;

    // Cross: pressed on the first frame (with confirm), held after; recently-pressed? sees it for
    // three frames at 60 fps (history entries 0, 1, 2).
    CpadInfo q;
    cpad_set_hw(q, pad::x, 128, 128, 128, 128);
    service_cpad(q, s);
    CHECK(cpad_pressed_p(q, pad::x));
    CHECK(cpad_hold_p(q, pad::x | pad::confirm) && (q.button0_abs[0] & pad::confirm));
    CHECK(q.abutton[kAbX] == 0); // the pressure fake reads last frame's held buttons
    CHECK(recently_pressed_p(q, pad::x));
    service_cpad(q, s);
    CHECK(!cpad_pressed_p(q, pad::x) && cpad_hold_p(q, pad::x));
    CHECK(q.abutton[kAbX] == 255);
    CHECK(recently_pressed_p(q, pad::x));
    service_cpad(q, s);
    CHECK(recently_pressed_p(q, pad::x));
    service_cpad(q, s);
    CHECK(!recently_pressed_p(q, pad::x));
    cpad_clear(q, pad::x);
    CHECK(!cpad_hold_p(q, pad::x));

    // At 120 fps the history keeps entries 3-5 too: six frames.
    CpadInfo r;
    s.target_fps = 120;
    cpad_set_hw(r, pad::square, 128, 128, 128, 128);
    service_cpad(r, s);
    cpad_set_hw(r, 0, 128, 128, 128, 128);
    for (int i = 0; i < 5; i++) {
        service_cpad(r, s);
    }
    CHECK(recently_pressed_p(r, pad::square));
    service_cpad(r, s);
    CHECK(!recently_pressed_p(r, pad::square));

    // analog-input: 200 - 128 = 72, past the 10 dead band by 62 of 90.
    CHECK_NEAR(analog_input(200, 128.0f, 10.0f, 100.0f, 1.0f), 62.0 / 90.0, 1e-6);
    CHECK_NEAR(analog_input(50, 128.0f, 10.0f, 100.0f, 1.0f), -68.0 / 90.0, 1e-6);
    CHECK(analog_input(130, 128.0f, 10.0f, 100.0f, 1.0f) == 0.0f);
    CHECK(analog_input(255, 128.0f, 10.0f, 100.0f, 2.0f) == 2.0f);

    // No pad: everything released.
    r.valid = 128;
    service_cpad(r, s);
    CHECK(r.button0_abs[0] == 0 && r.stick0_speed == 0.0f);
    TEST_MAIN_END();
}
