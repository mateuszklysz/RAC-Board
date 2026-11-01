// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The target movement core (src/jak/movement.cpp, math/core.cpp) against values worked out from
// logic-target.gc, target-util.gc and target.gc, and the on-foot pipeline (target-post) running on
// a flat floor (tests/core_fakes.hpp).
#include <cstring>

#include "check.hpp"
#include "core_fakes.hpp"
#include "jak/math/core.hpp"
#include "jak/movement.hpp"
#include "jak/surface.hpp"

using namespace jak;

namespace {

// A camera pitched 30 degrees down: rows right, up, forward. (A camera whose up is exactly the
// floor's normal makes warp-vector-into-surface!'s axis 0/0, NaN, in OpenGOAL as here.)
void pitch_camera(Target& t) {
    t.camera_local_to_world.v[1] = Vec{0.0f, 0.8660254f, 0.5f, 0.0f};
    t.camera_local_to_world.v[2] = Vec{0.0f, -0.5f, 0.8660254f, 0.0f};
}

void make(Target& t) {
    pitch_camera(t);
    t.clock.frame_counter = 3000;
    movement_init(t, Vec{0.0f, 0.0f, 0.0f, 1.0f});
    t.control.local_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
    t.control.gspot_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
    update_rates(t, 1.0f);
}

} // namespace

int main() {
    // ---- maths ----
    CHECK_NEAR(jak::acos(0.0f), 16384.0, 1.0);
    CHECK(jak::acos(1.0f) == 0.00000000001f); // the PC port never returns 0
    CHECK_NEAR(jak::asin(-1.0f), -16384.0, 1.0);
    CHECK(jak::asin(2.0f) == 16383.996f);
    {
        Mat m;
        // A quarter turn about y: row 2 is (ax az (1 - c) - ay s, ...), so +z goes to -x.
        matrix_axis_sin_cos(m, Vec{0.0f, 1.0f, 0.0f, 0.0f}, 1.0f, 0.0f);
        Vec v;
        vector_matrix_mul(v, Vec{0.0f, 0.0f, 1.0f, 0.0f}, m);
        CHECK_NEAR(v.x, -1.0, 1e-6);
        CHECK_NEAR(v.z, 0.0, 1e-6);
        matrix_axis_sin_cos(m, Vec{0.0f, 0.0f, 0.0f, 5.0f}, 1.0f, 0.0f); // zero axis: identity
        CHECK(m.v[0].x == 1.0f && m.v[0].z == 0.0f);
        Vec r;
        vector_reflect_flat(r, Vec{0.0f, -5.0f, 0.0f, 1.0f}, Vec{0.0f, 1.0f, 0.0f, 1.0f});
        CHECK(r.x == 0.0f && r.y == 1.0f && r.z == 0.0f); // the normal part replaced by n
        CHECK(vector_vector_xz_distance(Vec{1, 7, 1, 0}, Vec{4, -3, 5, 0}) == 5.0f);
    }

    // ---- clock ----
    Target t;
    make(t);
    CHECK(t.clock.time_adjust_ratio == 1.0f);
    CHECK(t.clock.seconds_per_frame == 0.016666668f);
    t.display_time_factor = 6.0f; // PAL
    update_rates(t, 1.0f);
    CHECK_NEAR(t.clock.time_adjust_ratio, 1.2, 1e-6);
    t.display_time_factor = 5.0f;
    update_rates(t, 1.0f);

    // ---- set-up ----
    CHECK(t.control.mod_surface == &walk_mods);
    CHECK(t.control.surf == &stone_surface);
    CHECK(t.control.dynam->gravity_length == 245760.0f);
    CHECK(t.control.last_trans_any_surf.y == -819200.0f);
    CHECK(t.control.bend_speed == 32.0f);
    CHECK(target_bank.ground_timeout == 60 && target_bank.stuck_time == 90);
    CHECK(target_bank.body_radius == 2867.2f);

    // ---- build-conversions / reverse-conversions with no rotation ----
    t.control.transv = Vec{1.0f, 2.0f, 3.0f, 1.0f};
    build_conversions(t, t.control.transv);
    CHECK(t.control.current_surface->target_speed == 40960.0f);
    CHECK(t.control.current_surface->fric == 153600.0f);
    CHECK(t.control.transv_ctrl.x == 1.0f && t.control.transv_ctrl.z == 3.0f);
    CHECK(t.control.time_of_last_surface_change == 3000);
    Vec back;
    reverse_conversions(t, back);
    CHECK(back.y == 2.0f);
    CHECK(t.control.ctrl_xz_vel == std::sqrt(10.0f));

    // Facing +x: the control frame's z is world x.
    vector_turn_to(t, Vec{5.0f, 0.0f, 0.0f, 1.0f});
    t.control.transv = Vec{100.0f, 0.0f, 0.0f, 1.0f};
    build_conversions(t, t.control.transv);
    CHECK_NEAR(t.control.transv_ctrl.z, 100.0, 1e-3);
    CHECK_NEAR(t.control.transv_ctrl.x, 0.0, 1e-3);
    // set-forward-vel keeps y, sets the control z and clears x.
    t.control.transv = Vec{100.0f, 7.0f, 50.0f, 1.0f};
    set_forward_vel(t, 1000.0f);
    CHECK_NEAR(t.control.transv.x, 1000.0, 1e-2);
    CHECK_NEAR(t.control.transv.y, 7.0, 1e-3);
    CHECK_NEAR(t.control.transv.z, 0.0, 1e-2);
    // delete-back-vel: the backward part along dir-targ (+x now) goes, the rest stays.
    t.control.transv = Vec{-300.0f, 0.0f, 40.0f, 1.0f};
    delete_back_vel(t);
    CHECK_NEAR(t.control.transv.x, 0.0, 1e-3);
    CHECK_NEAR(t.control.transv.z, 40.0, 1e-3);

    // ---- the pad ----
    Target p;
    make(p);
    p.pad.stick0_dir = 0.0f;
    p.pad.stick0_speed = 1.0f;
    Vec dir;
    read_pad(p, dir);
    CHECK_NEAR(dir.z, 0.8660254, 1e-6); // the camera's forward row
    CHECK(p.control.pad_magnitude == 1.0f);
    build_conversions(p, p.control.transv);
    turn_to_vector(p, dir, 0.5f);
    CHECK_NEAR(p.control.turn_to_angle, 0.0, 1.0);
    CHECK(p.control.turn_to_magnitude == 0.5f);
    CHECK_NEAR(p.control.target_transv.z, 0.5 * 40960.0, 0.1);
    CHECK_NEAR(p.control.to_target_pt_xz.z, 1.0, 1e-5);
    CHECK(debounce_speed(0.5f, 0.2f, Vec{0, 0, 1, 0}, Vec{0, 0, -1, 0}) == 0.0f);
    CHECK(debounce_speed(0.8f, 0.2f, Vec{0, 0, 1, 0}, Vec{0, 0, -1, 0}) == 0.8f);

    // ---- gravity ----
    // In the air: -245760 * spf, which is 4096.0002 a tick; the fall stops at gravity-max.
    Target g;
    make(g);
    // (A velocity with no part across gravity makes add-gravity's f1/f2 0/0: give it some.)
    g.control.trans.y = 100000.0f;
    g.control.transv = Vec{100.0f, 0.0f, 0.0f, 1.0f};
    build_conversions(g, g.control.transv);
    add_gravity(g);
    CHECK_NEAR(g.control.transv_ctrl.y, -245760.0 * 0.016666668, 1e-3);
    CHECK_NEAR(g.control.transv_ctrl.x, 100.0, 1e-4);
    g.control.transv_ctrl.y = -200000.0f;
    add_gravity(g);
    CHECK(g.control.transv_ctrl.y == -163840.0f);
    // On the floor with stone's slip 1: -gravity minus reflect-flat's n: (0, -245761, 0).
    g.control.status = cstat::on_surface;
    g.control.transv_ctrl = Vec{100.0f, 0.0f, 0.0f, 1.0f};
    add_gravity(g);
    CHECK_NEAR(g.control.transv_ctrl.y, -245761.0 * 0.016666668, 1e-3);

    // target-time-to-ground from 1 m, still: 12 ticks of 5 (4096 - 68.27 * m(m - 1)/2 > 0 up to
    // m = 12).
    g.control.trans.y = 4096.0f;
    g.control.gspot_pos.y = 0.0f;
    g.control.transv = Vec{};
    CHECK(target_height_above_ground(g) == 4096.0f);
    CHECK(target_time_to_ground(g) == 60);

    // ---- slopes: a 45-degree slope rising ahead ----
    Target s;
    make(s);
    const float r = std::sqrt(0.5f);
    s.control.local_normal = Vec{0.0f, r, -r, 1.0f};
    build_conversions(s, s.control.transv);
    target_compute_slopes(s, s.control.dynam->gravity_normal);
    CHECK_NEAR(s.control.local_slope_z, r, 1e-5);
    CHECK_NEAR(s.control.local_slope_x, 0.0, 1e-6);

    // ---- jumps ----
    Target j;
    make(j);
    j.control.status = cstat::on_surface;
    j.control.transv = Vec{};
    Vec jv{1000.0f, 0.0f, 0.0f, 1.0f};
    init_var_jump(j, target_bank.jump_height_min, target_bank.jump_height_max, true, false, jv, 0.0f);
    float low;
    std::memcpy(&low, &j.control.unknown_word04, 4);
    CHECK(low == target_bank.jump_height_min);
    CHECK(j.control.did_move_to_pole_or_max_jump_height == target_bank.jump_height_max);
    // sqrt(2 g h) + g / 120.
    CHECK_NEAR(jv.y, std::sqrt(2.0 * 245760.0 * 4136.96) + 245760.0 * 0.008333334, 0.05);
    init_var_jump(j, 4096.0f, 8192.0f, false, true, jv, 0.0f);
    std::memcpy(&low, &j.control.unknown_word04, 4);
    CHECK_NEAR(low, 4096.0 - (-409.6 + 2867.2), 1e-3);
    // mod-var-jump half way through (15 of 30 time-frames): the top is half way between.
    j.state_time = j.clock.frame_counter - 15;
    init_var_jump(j, 4096.0f, 8192.0f, false, false, jv, 0.0f);
    jv = Vec{1000.0f, 0.0f, 0.0f, 1.0f};
    mod_var_jump(j, true, false, true, jv);
    CHECK(j.control.unknown_float36 == 0.5f);
    CHECK_NEAR(jv.y, std::sqrt(2.0 * 245760.0 * 6144.0) + 245760.0 * 0.008333334, 0.05);
    mod_var_jump(j, true, false, false, jv);
    CHECK(j.control.unknown_float36 == -1.0f);
    // A push from a moving platform under the jump: half its rise is added.
    j.control.rider_time = j.clock.frame_counter;
    j.control.rider_last_move = Vec{0.0f, 2000.0f, 0.0f, 1.0f};
    init_var_jump(j, 4096.0f, 8192.0f, false, false, jv, 0.0f);
    std::memcpy(&low, &j.control.unknown_word04, 4);
    CHECK(low == 5096.0f);
    CHECK(j.control.additional_decaying_velocity.y == 2000.0f);
    CHECK(j.control.additional_decaying_velocity_decay_start_time == j.clock.frame_counter + 30000);

    // can-jump?
    CHECK(can_jump_p(j, Symbol("board")));
    j.control.poly_pat.bits = 13u << 18; // pat-event rail
    CHECK(can_jump_p(j, Symbol("board")) && !can_jump_p(j, Symbol()));
    j.control.poly_pat.bits = 0;
    j.control.current_surface->flags = sflag::no_jump;
    CHECK(!can_jump_p(j, Symbol("board")));
    j.control.current_surface->flags = 0;
    j.control.status = 0;
    j.control.last_time_on_surface = j.clock.frame_counter - 59;
    CHECK(can_jump_p(j, Symbol()));
    j.control.last_time_on_surface = j.clock.frame_counter - 60;
    CHECK(!can_jump_p(j, Symbol()));
    j.board->unknown_time_frame00 = j.clock.frame_counter - 10;
    CHECK(can_jump_p(j, Symbol("board")));

    // can-feet?: the attack timeout (90).
    j.control.last_attack_end_time = j.clock.frame_counter - 50;
    CHECK(!can_feet_p(j, Symbol()));
    CHECK(j.control.last_feet_attempt_time == j.clock.frame_counter);
    j.control.last_attack_end_time = j.clock.frame_counter - 90;
    CHECK(can_feet_p(j, Symbol()));
    CHECK(can_exit_duck_p(j));

    // ---- collision spheres ----
    target_collide_set(j, Symbol("board"), 0.0f);
    CHECK(j.control.penetrate_using == (penetrate::touch | penetrate::board));
    CHECK(fake::prims[4].as == cspec::jak);
    CHECK(fake::prims[0].ti == -2);
    target_collide_set(j, Symbol("normal"), 0.0f);
    CHECK(fake::prims[1].local.y == 2867.2f + 2867.2f && fake::prims[1].local.w == 2867.2f);
    CHECK(fake::prims[10].local.y == 4915.2f && fake::prims[10].local.w == 9011.2f);
    target_collide_set(j, Symbol("duck"), 1.0f);
    CHECK(fake::prims[2].local.y == 2867.2f && j.control.duck_gun_tube_transision == 1.0f);
    j.control.default_collide_as_fgnd = 7;
    target_danger_set(j, Symbol("board-spin"), Symbol());
    CHECK(j.focus_status & focus::dangerous);
    CHECK(fake::prims[6].local.w == 6963.2f && fake::prims[6].as == 7);
    CHECK(fake::prims[10].local.w == 9830.4f);
    CHECK(j.control.danger_mode == Symbol("board-spin"));
    target_danger_set(j, Symbol("harmless"), Symbol());
    CHECK(!(j.focus_status & focus::dangerous) && !j.control.danger_mode);
    CHECK(fake::prims[6].as == 0 && fake::prims[6].local.w == 0.0f);
    CHECK(fake::prims[2].local.y == 2867.2f); // back to the collide mode (duck)

    // ---- timers ----
    target_timed_invulnerable(seconds(1), j, 1);
    CHECK(j.state_flags & state_flags::tinvul1);
    j.clock.frame_counter += 300;
    post_flag_setup(j);
    CHECK(!(j.state_flags & state_flags::tinvul1));
    target_exit(j);
    CHECK(j.control.mod_surface == &walk_mods && j.control.draw_offset.y == 0.0f);

    // ---- target-move-dist: standing still logs 0 ----
    Target m;
    make(m);
    for (int i = 0; i < 20; i++) {
        m.clock.frame_counter += 5;
        target_log_trans(m);
    }
    CHECK(target_move_dist(m, 90) == 0.0f);
    m.control.status = 0;
    CHECK(jump_hit_ground_stuck_p(m) && m.control.last_time_of_stuck == m.clock.frame_counter);

    // ---- the on-foot pipeline on a flat floor: stick forward for 2 s ----
    Target w;
    make(w);
    w.control.trans.y = 1000.0f; // drop onto the floor first, the stick held forward
    w.pad.stick0_dir = 0.0f;
    w.pad.stick0_speed = 1.0f;
    for (int i = 0; i < 30; i++) {
        w.clock.frame_counter += 5;
        target_post(w);
    }
    CHECK(w.control.trans.y == 0.0f);
    CHECK(w.control.status & cstat::on_surface);
    CHECK(fake::ground_finds > 0);
    float z0 = w.control.trans.z;
    for (int i = 0; i < 120; i++) {
        w.clock.frame_counter += 5;
        w.real_frame_counter += 5;
        target_post(w);
    }
    // *walk-mods* on stone: target speed 40960 (10 m/s) straight ahead, reached in 0.27 s at
    // seek0 153600/s.
    CHECK_NEAR(w.control.ctrl_xz_vel, 40960.0, 50.0);
    CHECK(w.control.trans.z - z0 > 40960.0f * 1.5f);
    CHECK_NEAR(w.control.trans.x, 0.0, 1.0);
    CHECK(w.control.trans_log_idx == 150 % 128);

    // GOAL's division by zero (goal_div): FLT_MAX signed by the operands, so a zero vector
    // normalizes to zero and a level camera over a flat floor turns nothing.
    CHECK(goal_div(0.0f, 0.0f) == 3.40282347e38f);
    CHECK(goal_div(-1.0f, 0.0f) == -3.40282347e38f);
    CHECK(goal_div(1.0f, -0.0f) == -3.40282347e38f);
    Vec zero{};
    vector_normalize(zero, 1.0f);
    CHECK(zero.x == 0.0f && zero.y == 0.0f && zero.z == 0.0f);
    Target lv;
    make(lv);
    lv.camera_local_to_world = Mat{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    lv.pad.stick0_speed = 1.0f;
    for (int i = 0; i < 10; i++) {
        lv.clock.frame_counter += 5;
        target_post(lv);
    }
    CHECK(lv.control.quat_for_control.w == 1.0f);
    CHECK(lv.control.trans.z > 0.0f && lv.control.trans.x == 0.0f);
    TEST_MAIN_END();
}
