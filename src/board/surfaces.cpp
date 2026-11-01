// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The JET-Board's surfaces, from OpenGOAL's goal_src/jak2/engine/target/board/target-board.gc
// (*board-walk-mods* to *board-ride-mods*, with their mult-hook lambdas and the copies made with
// `copy`) (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). The
// definitions are in GOAL's order, so each copy is taken after the surface it copies is built.
#include "board/surfaces.hpp"

#include <cmath>

#include "jak/math.hpp"
#include "jak/target.hpp"

namespace jak {

// *board-walk-mods*' mult-hook (also *board-duck-mods*' and *board-turn-to-mods*'): the ground's
// slope factors win, the slip follows the slope across the heading, and for 2 s after a spin
// (600 time-frames) the seeks and turn come back from a tenth.
void board_walk_mult_hook(Target& self, Surface& a0, const Surface& a1, const Surface&, int a3) {
    if (a3 != 1) {
        return;
    }
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    if (0.9f < c.surface_angle) {
        b.on_flat_time = current_time(self);
    }
    b.slip_factor = lerp_scale(1.0f, a0.slip_factor, std::fabs(c.ctrl_slope_heading), 0.0f, 1.0f);
    a0.slip_factor = b.slip_factor;
    a0.slope_up_factor = a1.slope_up_factor;
    a0.slope_down_factor = a1.slope_down_factor;
    a0.seek0 = lerp_scale(a0.seek0 / 10.0f, a0.seek0,
                          static_cast<float>(current_time(self) - b.spin_time), 0.0f, 600.0f);
    a0.seek90 = lerp_scale(a0.seek90 / 10.0f, a0.seek90,
                           static_cast<float>(current_time(self) - b.spin_time), 0.0f, 600.0f);
    a0.vel_turn = lerp_scale(131072.0f, a1.vel_turn,
                             static_cast<float>(current_time(self) - b.spin_time), 0.0f, 600.0f);
    a0.turnv = lerp_scale(91022.22f, a1.turnv,
                          static_cast<float>(current_time(self) - b.spin_time), 0.0f, 600.0f);
    if (!time_elapsed(self, b.spin_ground_start_time, seconds(0.3))) {
        c.last_attack_end_time = current_time(self);
        b.spin_ground_time = current_time(self);
        a0.seek0 = a0.seek0 / 10.0f;
        a0.seek90 = a0.seek90 / 10.0f;
        a0.vel_turn = 131072.0f;
        a0.turnv = 91022.22f;
    }
}

// *board-ride-jump-mods*' mult-hook: in a jump off a rail the stick pushes sideways to the rail.
void board_ride_jump_mult_hook(Target& self, Surface&, const Surface&, const Surface&, int a3) {
    if (a3 != 1) {
        return;
    }
    ControlInfo& c = self.control;
    if (c.cpad->stick0_speed != 0.0f) {
        const float f30 = (122880.0f * c.turn_to_magnitude) * seconds_per_frame(self);
        Vec v1{};
        vector_flatten(v1, c.to_target_pt_xz, self.board->ride_dir);
        vector_add_float_mul(c.transv, c.transv, v1, f30);
    }
}

// *board-wall-kick-mods*' mult-hook: no turning for the kick's first 0.05 s.
void board_wall_kick_mult_hook(Target& self, Surface& a0, const Surface&, const Surface&, int a3) {
    if (a3 != 1) {
        return;
    }
    if (!time_elapsed(self, self.state_time, seconds(0.05))) {
        a0.turnv = 0.0f;
        a0.turnvf = 0.0f;
    }
}

// *board-ride-mods*' mult-hook: the ground's slope factors win.
void board_ride_mult_hook(Target&, Surface& a0, const Surface& a1, const Surface&, int a3) {
    if (a3 == 1) {
        a0.slope_up_factor = a1.slope_up_factor;
        a0.slope_down_factor = a1.slope_down_factor;
    }
}

Surface board_walk_mods = [] {
    Surface s{};
    s.name = Symbol("run");
    s.turnv = 32768.0f;
    s.turnvv = 131072.0f;
    s.tiltv = 131072.0f;
    s.tiltvv = 2621440.0f;
    s.transv_max = 143360.0f;
    s.target_speed = 102400.0f;
    s.seek0 = 0.5f;
    s.seek90 = 0.5f;
    s.seek180 = 0.5f;
    s.fric = 0.2f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = -0.125f;
    s.slide_factor = 1.0f;
    s.slope_up_factor = 8192.0f;
    s.slope_down_factor = 24576.0f;
    s.slope_slip_angle = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.slope_change_preserve = 1.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.tiltvf = 15.0f;
    s.tiltvvf = 15.0f;
    s.vel_turn = 65536.0f;
    s.mult_hook = board_walk_mult_hook;
    s.flags = sflag::turn_to_pad;
    return s;
}();

Surface board_duck_mods = [] {
    Surface s{};
    s.name = Symbol("duck");
    s.turnv = 21845.334f;
    s.turnvv = 131072.0f;
    s.tiltv = 131072.0f;
    s.tiltvv = 2621440.0f;
    s.transv_max = 151552.0f;
    s.target_speed = 102400.0f;
    s.seek0 = 0.5f;
    s.seek90 = 0.5f;
    s.seek180 = 0.5f;
    s.fric = 0.2f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 0.5f;
    s.slide_factor = 1.0f;
    s.slope_up_factor = 8192.0f;
    s.slope_down_factor = 24576.0f;
    s.slope_slip_angle = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.slope_change_preserve = 0.75f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.tiltvf = 15.0f;
    s.tiltvvf = 15.0f;
    s.vel_turn = 65536.0f;
    s.flags = sflag::no_turn_around | sflag::turn_to_pad | sflag::duck;
    s.mult_hook = board_walk_mods.mult_hook; // (set! (-> *board-duck-mods* mult-hook) ...)
    return s;
}();

Surface board_air_mods = [] {
    Surface s{};
    s.name = Symbol("air");
    s.turnv = 49152.0f;
    s.turnvv = 131072.0f;
    s.tiltv = 16384.0f;
    s.tiltvv = 131072.0f;
    s.transv_max = 143360.0f;
    s.target_speed = 102400.0f;
    s.seek0 = 0.8f;
    s.seek90 = 0.8f;
    s.seek180 = 0.8f;
    s.fric = 0.2f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_up_factor = 8192.0f;
    s.slope_down_factor = 24576.0f;
    s.slope_slip_angle = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.slope_change_preserve = 1.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.tiltvf = 150.0f;
    s.tiltvvf = 60.0f;
    s.vel_turn = 65536.0f;
    s.mode = surface_mode::air;
    s.flags = sflag::no_turn_around | sflag::turn_to_pad | sflag::check_edge | sflag::air;
    return s;
}();

Surface board_jump_mods = [] {
    Surface s{};
    s.name = Symbol("jump");
    s.turnv = 49152.0f;
    s.turnvv = 524288.0f;
    s.tiltv = 32768.0f;
    s.tiltvv = 131072.0f;
    s.transv_max = 143360.0f;
    s.target_speed = 102400.0f;
    s.fric = 0.2f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_slip_angle = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.tiltvf = 30.0f;
    s.tiltvvf = 18.0f;
    s.vel_turn = 10922.667f;
    s.mode = surface_mode::air;
    s.flags = sflag::no_turn_around | sflag::turn_to_pad | sflag::check_edge | sflag::air;
    return s;
}();

Surface board_ride_jump_mods = [] {
    Surface s{};
    s.name = Symbol("jump");
    s.turnv = 49152.0f;
    s.turnvv = 32768.0f;
    s.tiltv = 32768.0f;
    s.tiltvv = 131072.0f;
    s.transv_max = 143360.0f;
    s.target_speed = 102400.0f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_slip_angle = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 300.0f;
    s.tiltvf = 30.0f;
    s.tiltvvf = 18.0f;
    s.mult_hook = board_ride_jump_mult_hook;
    s.mode = surface_mode::air;
    s.flags = sflag::no_turn_around | sflag::turn_to_pad | sflag::check_edge | sflag::air;
    return s;
}();

// (define *board-duck-jump-mods* *board-jump-mods*): the same surface, not a copy.
Surface& board_duck_jump_mods = board_jump_mods;

Surface board_spin_mods = [] {
    Surface s = board_jump_mods; // (copy *board-jump-mods* 'global)
    s.name = Symbol("spin");
    s.flags = sflag::no_turn_around | sflag::check_edge | sflag::air;
    s.seek0 = 0.0f;
    s.seek90 = 0.0f;
    s.seek180 = 0.0f;
    s.vel_turn = 0.0f;
    s.turnv = 0.0f;
    s.turnvv = 0.0f;
    s.tiltv = 0.0f;
    s.tiltvf = 0.0f;
    return s;
}();

Surface board_spin_post_mods = [] {
    Surface s = board_jump_mods;
    s.seek0 = 0.0f;
    s.seek90 = 0.0f;
    s.seek180 = 0.0f;
    s.vel_turn = 0.0f;
    return s;
}();

Surface board_flip_mods = [] {
    Surface s = board_duck_jump_mods;
    s.flags = sflag::no_turn_around | sflag::turn_to_vel | sflag::air;
    s.seek0 = 0.0f;
    s.seek90 = 0.0f;
    s.seek180 = 0.0f;
    s.vel_turn = 0.0f;
    s.turnv = 49152.0f;
    s.turnvf = 30.0f;
    s.turnvv = 0.0f;
    s.turnvvf = 0.0f;
    return s;
}();

Surface board_wall_kick_mods = [] {
    Surface s{};
    s.name = Symbol("jump");
    s.tiltv = 65536.0f;
    s.tiltvv = 262144.0f;
    s.transv_max = 143360.0f;
    s.target_speed = 102400.0f;
    s.seek180 = 0.8f;
    s.fric = 0.2f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_slip_angle = 1.0f;
    s.impact_fric = 0.8f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.tiltvf = 150.0f;
    s.tiltvvf = 60.0f;
    s.mult_hook = board_wall_kick_mult_hook;
    s.mode = surface_mode::air;
    s.flags = sflag::no_turn_around | sflag::turn_to_vel | sflag::air;
    return s;
}();

Surface board_halfpipe_mods = [] {
    Surface s{};
    s.name = Symbol("jump");
    s.turnv = 131072.0f;
    s.turnvv = 524288.0f;
    s.tiltv = 16384.0f;
    s.tiltvv = 131072.0f;
    s.transv_max = 143360.0f;
    s.target_speed = 102400.0f;
    s.fric = 0.2f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_slip_angle = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.tiltvf = 150.0f;
    s.tiltvvf = 60.0f;
    s.mode = surface_mode::air;
    s.flags = sflag::no_turn_around | sflag::turn_to_vel | sflag::air | sflag::turn_when_centered;
    return s;
}();

Surface board_turn_to_mods = [] {
    Surface s{};
    s.name = Symbol("run");
    s.turnv = 524288.0f;
    s.tiltv = 131072.0f;
    s.tiltvv = 2621440.0f;
    s.transv_max = 143360.0f;
    s.target_speed = 102400.0f;
    s.seek0 = 0.5f;
    s.seek90 = 0.5f;
    s.seek180 = 0.5f;
    s.fric = 0.2f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 0.5f;
    s.slide_factor = 1.0f;
    s.slope_up_factor = 8192.0f;
    s.slope_down_factor = 24576.0f;
    s.slope_slip_angle = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.slope_change_preserve = 1.0f;
    s.turnvf = 30.0f;
    s.tiltvf = 15.0f;
    s.tiltvvf = 15.0f;
    s.vel_turn = 65536.0f;
    s.flags = sflag::no_turn_around | sflag::turn_to_vel | sflag::turn_when_centered;
    s.mult_hook = board_walk_mods.mult_hook;
    return s;
}();

Surface board_ride_mods = [] {
    Surface s{};
    s.name = Symbol("run");
    s.turnv = 218453.33f;
    s.turnvv = 131072.0f;
    s.tiltv = 131072.0f;
    s.tiltvv = 262144.0f;
    s.transv_max = 143360.0f;
    s.target_speed = 40960.0f;
    s.seek0 = 0.5f;
    s.seek90 = 0.5f;
    s.seek180 = 0.5f;
    s.fric = 0.1f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 0.6f;
    s.slide_factor = 1.0f;
    s.slope_up_factor = 24576.0f;
    s.slope_down_factor = 49152.0f;
    s.slope_slip_angle = 1.0f;
    s.impact_fric = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.tiltvf = 60.0f;
    s.tiltvvf = 30.0f;
    s.mult_hook = board_ride_mult_hook;
    s.mode = surface_mode::ride;
    s.flags = sflag::no_turn_around | sflag::turn_to_pad | sflag::check_edge |
              sflag::turn_when_centered;
    return s;
}();

} // namespace jak
