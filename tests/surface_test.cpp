// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The surface engine (src/jak/surface.cpp) and the board's surfaces (src/board/surfaces.cpp)
// against values worked out from surface-h.gc and target-board.gc.
#include "board/surfaces.hpp"
#include "check.hpp"
#include "core_fakes.hpp"
#include "jak/surface.hpp"

using namespace jak;

namespace {
int hook_calls = 0;
void counting_hook(Target&, Surface& a0, const Surface&, const Surface&, int pass) {
    hook_calls++;
    a0.fric = static_cast<float>(pass);
}
void touch_a(Target&) {}
} // namespace

int main() {
    Target t;
    t.clock.frame_counter = 100000;
    Surface out{};

    // *board-walk-mods* on stone, long after a spin: products, then the hook's ground slopes.
    t.board->spin_time = 0;
    t.board->spin_ground_start_time = 0;
    t.control.surface_angle = 1.0f;
    t.control.ctrl_slope_heading = 0.0f;
    surface_mult(t, out, board_walk_mods, stone_surface);
    CHECK(out.target_speed == 102400.0f);
    CHECK(out.transv_max == 143360.0f);
    CHECK(out.fric == 0.2f * 153600.0f);
    CHECK(out.seek0 == 0.5f * 153600.0f);
    CHECK(out.seek180 == 0.5f * 256000.0f);
    CHECK(out.slope_up_factor == 8192.0f);   // the hook puts the mod's back (stone's is 0)
    CHECK(out.slope_down_factor == 24576.0f);
    CHECK(out.slip_factor == 1.0f);          // lerp-scale 1 .. -0.125 at heading 0
    CHECK(t.board->slip_factor == 1.0f);
    CHECK(out.vel_turn == 65536.0f);
    CHECK(out.turnv == 32768.0f);
    CHECK(out.flags == sflag::turn_to_pad);
    CHECK(!out.mode);                        // mode and flags' base come from the ground surface
    CHECK(t.board->on_flat_time == 100000);
    CHECK(out.mult_hook == board_walk_mult_hook);

    // Across the slope the slip goes to the mod's (-0.125 x stone's 1).
    t.control.ctrl_slope_heading = -1.0f;
    surface_mult(t, out, board_walk_mods, stone_surface);
    CHECK(out.slip_factor == -0.125f);

    // Just after a spin: a tenth of the seeks, the fast turn; within 0.3 s of a ground spin the
    // seeks are cut again and the attack timer reset.
    t.control.ctrl_slope_heading = 0.0f;
    t.board->spin_time = 100000;
    surface_mult(t, out, board_walk_mods, stone_surface);
    CHECK(out.seek0 == 76800.0f / 10.0f);
    CHECK(out.vel_turn == 131072.0f);
    CHECK(out.turnv == 91022.22f);
    t.board->spin_time = 100000 - 300; // half way: lerp-scale 0.5
    surface_mult(t, out, board_walk_mods, stone_surface);
    CHECK_NEAR(out.seek0, 0.5 * 7680.0 + 0.5 * 76800.0, 1e-2);
    t.board->spin_ground_start_time = 100000 - 10;
    surface_mult(t, out, board_walk_mods, stone_surface);
    CHECK_NEAR(out.seek0, (0.5 * 7680.0 + 0.5 * 76800.0) / 10.0, 1e-3);
    CHECK(t.control.last_attack_end_time == 100000 && t.board->spin_ground_time == 100000);
    CHECK(out.turnv == 91022.22f);

    // Duck and turn-to share walk's hook; duck-jump is jump itself; the copies.
    CHECK(board_duck_mods.mult_hook == board_walk_mult_hook);
    CHECK(board_turn_to_mods.mult_hook == board_walk_mult_hook);
    CHECK(&board_duck_jump_mods == &board_jump_mods);
    CHECK(board_spin_mods.name == Symbol("spin") && board_spin_mods.turnvv == 0.0f &&
          board_spin_mods.tiltvv == 131072.0f && board_spin_mods.tiltvf == 0.0f &&
          board_spin_mods.flags == (sflag::no_turn_around | sflag::check_edge | sflag::air));
    CHECK(board_spin_post_mods.name == Symbol("jump") && board_spin_post_mods.seek0 == 0.0f &&
          board_spin_post_mods.turnv == 49152.0f);
    CHECK(board_flip_mods.turnvv == 0.0f && board_flip_mods.turnv == 49152.0f &&
          (board_flip_mods.flags & sflag::turn_to_vel) && board_flip_mods.tiltvvf == 18.0f);

    // *board-ride-mods* on ice: speeds 1.5x, flags joined, mode from the ice (none).
    surface_mult(t, out, board_ride_mods, ice_surface);
    CHECK(out.transv_max == 143360.0f * 1.5f);
    CHECK(out.slope_up_factor == 24576.0f && out.slope_down_factor == 49152.0f);
    CHECK(out.slip_factor == 0.6f * 0.7f);
    CHECK(out.flags == (board_ride_mods.flags | sflag::no_turn_around | sflag::turn_to_pad));
    CHECK(!out.mode);

    // Wall kick: no turning for 0.05 s (15 time-frames) after the state starts.
    t.state_time = 100000 - 10;
    surface_mult(t, out, board_wall_kick_mods, stone_surface);
    CHECK(out.turnv == 0.0f && out.turnvf == 0.0f);
    t.state_time = 100000 - 15;
    surface_mult(t, out, board_wall_kick_mods, stone_surface);
    CHECK(out.turnvf == 0.0f * 1.0f); // the mod's turnvf is 0 anyway
    CHECK(out.tiltvf == 150.0f);

    // Ride jump: the stick pushes 122880 * magnitude * spf along to-target flattened off the rail.
    t.pad.stick0_speed = 1.0f;
    t.control.cpad = &t.pad;
    t.control.turn_to_magnitude = 1.0f;
    t.control.to_target_pt_xz = Vec{1.0f, 0.0f, 1.0f, 1.0f};
    t.board->ride_dir = Vec{0.0f, 0.0f, 1.0f, 1.0f};
    t.control.transv = Vec{};
    surface_mult(t, out, board_ride_jump_mods, stone_surface);
    CHECK_NEAR(t.control.transv.x, 122880.0f * (1.0f / 60.0f), 1e-2);
    CHECK_NEAR(t.control.transv.z, 0.0, 1e-3);

    // Hooks: the ground's when set, else the mod's; gravel's do-nothing touch hides the mod's.
    Surface mod = walk_mods;
    mod.touch_hook = touch_a;
    surface_mult(t, out, mod, stone_surface);
    CHECK(out.touch_hook == touch_a);
    surface_mult(t, out, mod, gravel_surface);
    CHECK(out.touch_hook == gravel_surface.touch_hook && out.touch_hook != touch_a);
    surface_mult(t, out, mod, rail_surface);
    CHECK(out.touch_hook == rail_surface.touch_hook && (out.flags & sflag::no_attack));

    // The mult hook runs once with pass 1 (the ground's wins over the mod's).
    Surface g = stone_surface;
    g.mult_hook = counting_hook;
    surface_mult(t, out, walk_mods, g);
    CHECK(hook_calls == 1 && out.fric == 1.0f);

    // surface-interp!: a * 0.75 + b * 0.25; b's mode and flags.
    surface_interp(out, walk_mods, board_air_mods, 0.25f);
    CHECK(out.turnv == 49152.0f * 0.25f + 131072.0f * 0.75f);
    CHECK(out.tiltvf == 150.0f * 0.25f + 150.0f * 0.75f);
    CHECK(out.mode == surface_mode::air && out.flags == board_air_mods.flags);

    // surface-clamp-speed only on pass 1.
    out.transv_max = 90000.0f;
    out.target_speed = 10.0f;
    surface_clamp_speed(t, out, walk_mods, stone_surface, 1);
    CHECK(out.transv_max == 40960.0f && out.target_speed == 10.0f);
    out.transv_max = 90000.0f;
    surface_clamp_speed(t, out, walk_mods, stone_surface, 2);
    CHECK(out.transv_max == 90000.0f);

    // The data order: entry 4 transv-max, 27 vel-turn.
    CHECK(surface_data(walk_mods, 4) == 40960.0f && surface_data(stone_surface, 27) == 1.0f);

    // calc-terminal-vel: ((600/60 - 1) / 0.5) * 0.5 - 1 = 8; terminal2: sqrt(18) - (1 + 0.5 * 18).
    CHECK_NEAR(calc_terminal_vel(600.0f, 1.0f, 0.5f), 8.0, 1e-6);
    CHECK_NEAR(calc_terminal2_vel(600.0f, 1.0f, 0.5f, 0.0f), std::sqrt(18.0) - 10.0, 1e-5);
    CHECK_NEAR(calc_terminal4_vel(600.0f, 1.0f, 0.5f), std::sqrt(std::sqrt(18.0)) - 10.0, 1e-5);
    TEST_MAIN_END();
}
