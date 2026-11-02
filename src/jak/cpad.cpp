// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's pad service, from OpenGOAL's goal_src/jak2/engine/ps2/pad.gc (jak-project, ISC License,
// Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
#include "jak/cpad.hpp"

#include <cmath>

#include "jak/math/math.hpp"

namespace jak {

void cpad_set_hw(CpadInfo& pad, uint16_t buttons, uint8_t rightx, uint8_t righty, uint8_t leftx,
                 uint8_t lefty) {
    pad.valid = 0;
    pad.button0 = buttons;
    pad.rightx = rightx;
    pad.righty = righty;
    pad.leftx = leftx;
    pad.lefty = lefty;
}

// cpad-invalid!: no pad, everything released and centred. The vibration fields are not kept.
void cpad_invalid(CpadInfo& pad) {
    pad.valid |= 128;
    pad.button0 = 0;
    pad.button0_abs[0] = 0;
    pad.button0_shadow_abs[0] = 0;
    pad.button0_rel[0] = 0;
    for (uint8_t& a : pad.abutton) {
        a = 0;
    }
    pad.stick0_dir = 0.0f;
    pad.stick0_speed = 0.0f;
    pad.rightx = 128;
    pad.righty = 128;
    pad.leftx = 128;
    pad.lefty = 128;
}

// analog-input.
float analog_input(int in, float offset, float center_val, float max_val, float out_range) {
    const float f1 = static_cast<float>(in) - offset;
    const float f0 = std::fabs(f1) - center_val;
    const float range = max_val - center_val;
    if (f1 < 0.0f) {
        out_range = -out_range;
    }
    if (0.0f >= f0) {
        return 0.0f;
    }
    if (f0 >= range) {
        return out_range;
    }
    return goal_div(f0 * out_range, range);
}

// service-cpads, the body for one pad. cpad-get-data is the adapter's cpad_set_hw;
// adjust-to-screen-flip (the mirrored world) and gun-trigger-buttons-swap! (a PC setting for the
// gun) are left out, as is the vibration (the board never buzzes) and *cpad-debug*. The territory is
// taken as not Japanese or Korean, so cross is confirm.
void service_cpad(CpadInfo& pad, const CpadService& s) {
    pad.old_leftx[1] = pad.old_leftx[0];
    pad.old_leftx[0] = pad.leftx;
    pad.old_lefty[1] = pad.old_lefty[0];
    pad.old_lefty[0] = pad.lefty;
    pad.old_rightx[1] = pad.old_rightx[0];
    pad.old_rightx[0] = pad.rightx;
    pad.old_righty[1] = pad.old_righty[0];
    pad.old_righty[0] = pad.righty;
    if (pad.valid & 128) {
        cpad_invalid(pad);
        return;
    }
    pad.button0_abs[2] = pad.button0_abs[1];
    pad.button0_abs[1] = pad.button0_shadow_abs[0];
    pad.button0_rel[2] = pad.button0_rel[1];
    pad.button0_rel[1] = pad.button0_rel[0];

    // The PC port's high frame rate fix: the longer history only moves at those rates.
    uint32_t* ha = pad.history_abs;
    uint32_t* hr = pad.history_rel;
    if (s.target_fps >= 300) {
        for (int i = 14; i >= 12; i--) {
            ha[i] = ha[i - 1];
        }
        for (int i = 14; i >= 12; i--) {
            hr[i] = hr[i - 1];
        }
    }
    if (s.target_fps >= 240) {
        for (int i = 11; i >= 9; i--) {
            ha[i] = ha[i - 1];
        }
        for (int i = 11; i >= 9; i--) {
            hr[i] = hr[i - 1];
        }
    }
    if (s.target_fps >= 165) {
        for (int i = 8; i >= 6; i--) {
            ha[i] = ha[i - 1];
        }
        for (int i = 8; i >= 6; i--) {
            hr[i] = hr[i - 1];
        }
    }
    if (s.target_fps >= 120) {
        ha[5] = ha[4];
        ha[4] = ha[3];
        hr[5] = hr[4];
        hr[4] = hr[3];
    }
    if (s.target_fps >= 75) {
        ha[3] = ha[2];
        hr[3] = hr[2];
    }
    ha[2] = ha[1];
    ha[1] = pad.button0_shadow_abs[0];
    hr[2] = hr[1];
    hr[1] = pad.button0_rel[0];

    // status 115 (0x73, a DualShock without pressure) fakes the pressure bytes from the buttons.
    if (pad.status == 115) {
        const uint32_t b = pad.button0_abs[0];
        pad.abutton[kAbRight] = (b & pad::right) ? 255 : 0;
        pad.abutton[kAbLeft] = (b & pad::left) ? 255 : 0;
        pad.abutton[kAbUp] = (b & pad::up) ? 255 : 0;
        pad.abutton[kAbDown] = (b & pad::down) ? 255 : 0;
        pad.abutton[kAbX] = (b & pad::x) ? 255 : 0;
        pad.abutton[kAbCircle] = (b & pad::circle) ? 255 : 0;
        pad.abutton[kAbTriangle] = (b & pad::triangle) ? 255 : 0;
        pad.abutton[kAbSquare] = (b & pad::square) ? 255 : 0;
        pad.abutton[kAbL1] = (b & pad::l1) ? 255 : 0;
        pad.abutton[kAbL2] = (b & pad::l2) ? 255 : 0;
        pad.abutton[kAbR1] = (b & pad::r1) ? 255 : 0;
        pad.abutton[kAbR2] = (b & pad::r2) ? 255 : 0;
    }
    uint32_t pushed = pad.button0;
    if (pad.lefty < 30) {
        pushed |= pad::l_analog_up;
    } else if (225 < pad.lefty) {
        pushed |= pad::l_analog_down;
    }
    if (pad.leftx < 30) {
        pushed |= pad::l_analog_left;
    } else if (225 < pad.leftx) {
        pushed |= pad::l_analog_right;
    }
    if (pad.righty < 30) {
        pushed |= pad::r_analog_up;
    } else if (225 < pad.righty) {
        pushed |= pad::r_analog_down;
    }
    if (pad.rightx < 30) {
        pushed |= pad::r_analog_left;
    } else if (225 < pad.rightx) {
        pushed |= pad::r_analog_right;
    }
    if (pushed & pad::x) {
        pushed |= pad::confirm;
    }
    pad.button0_shadow_abs[0] = pushed;
    pad.button0_abs[0] = pushed;
    ha[0] = pad.button0_abs[0];
    pad.button0_rel[0] = pad.button0_abs[0] & ~pad.button0_abs[1];
    hr[0] = pad.button0_rel[0];

    pad.stick0_speed = 1.0f;
    if ((pad.status >> 4) == 7) {
        const float f30 = 0.0078125f * static_cast<float>(static_cast<int>(pad.leftx) + -128);
        const float f28 = 0.0078125f * static_cast<float>(127 - static_cast<int>(pad.lefty));
        pad.stick0_dir = atan(-f30, f28);
        pad.stick0_speed = std::fmin(1.0f, std::sqrt(f30 * f30 + f28 * f28));
        if (pad.stick0_speed < s.stick_deadzone) {
            pad.stick0_speed = 0.0f;
        }
    } else {
        pad.leftx = 128;
        pad.lefty = 128;
        pad.rightx = 128;
        pad.righty = 128;
        pad.stick0_dir = 0.0f;
        pad.stick0_speed = 0.0f;
    }
    if (pad.button0_abs[0] != pad.button0_abs[1] || s.stick_deadzone < pad.stick0_speed ||
        pad.change_time == 0) {
        pad.change_time = s.current_time;
    }
}

// recently-pressed? (pad.gc, PC port): pressed in any of the last `frames` history entries.
bool recently_pressed_p(const CpadInfo& pad, uint32_t buttons, int frames) {
    uint32_t any = 0;
    for (int i = 0; i < frames && i < 15; i++) {
        any |= pad.history_rel[i];
    }
    return (any & buttons) != 0;
}

} // namespace jak
