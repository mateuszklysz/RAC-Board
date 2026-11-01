// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The board's states: goal_src/jak2/engine/target/board/board-states.gc from OpenGOAL
// (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md), in the file's
// order: its helpers (vector-vector-angle .. target-board-jump-trans), then each defstate as its
// event, enter, exit, trans, code and post. A code is a coroutine (runtime.hpp): its locals across
// a (suspend) live in Co, and CO_SUSPEND is the suspend.
//
// Left out, each marked where it was: debug-mode (*cheat-mode* 'debug) branches stay but read false
// in play; the debug formats; the demo's talker in target-board-hit.
#include <cmath>
#include <initializer_list>

#include "board/anim.hpp"
#include "board/board.hpp"
#include "board/collide.hpp"
#include "board/core.hpp"
#include "board/surfaces.hpp"
#include "board/target_hit.hpp"
#include "jak/math/board.hpp"
#include "jak/math/core.hpp"
#include "jak/math/math.hpp"

namespace jak {

namespace {

const Symbol kTrue("#t");

const Vec& gravity_normal(const Target& self) { return self.control.dynam->gravity_normal; }
float spf(const Target& self) { return seconds_per_frame(self); }
JointControl& jc(Target& self) { return skel(self); }
void push(Target& self, int n, TimeFrame t) { ja_channel_push(skel(self), n, t, anim_clock(self)); }
bool done(Target& self) { return ja_done(skel(self), 0); }
AnimId group(Target& self) { return ja_group(skel(self), 0); }
Interaction& io(Target& self) { return *runtime(self).io; }

// The stick bytes as the tricks read them: -(byte - 128) / 128, then back to an int (the int (* 128 x)).
int stick_in(uint8_t byte) {
    const float x = -0.0078125f * (-128.0f + static_cast<float>(byte));
    return static_cast<int32_t>(128.0f * x);
}

bool next_in(const Target& self, std::initializer_list<StateId> ids) {
    const StateId n = runtime(self).next_state;
    for (StateId i : ids) {
        if (n == i) {
            return true;
        }
    }
    return false;
}

void set_mods(Target& self, Surface* s) {
    self.control.mod_surface = s;
    runtime(self).mods_backup = s;
}

StateArgs jump_args(float a, float b, Symbol sym = Symbol(), Surface* surf = nullptr) {
    StateArgs r;
    r.f0 = a;
    r.f1 = b;
    r.sym = sym;
    r.surf = surf;
    return r;
}

StateArgs turn_to_args(const Vec& v, TimeFrame t) {
    StateArgs r;
    r.vec = v;
    r.time = t;
    return r;
}

Symbol halfpipe_sym(const Target& self) {
    return focus_test(self, focus::halfpipe) ? Symbol("halfpipe") : Symbol();
}

void particles_at_lift(Target& self, const char* group) {
    Vec gp = self.control.trans;
    gp.y = gp.y + 2048.0f;
    particles(self, group, gp);
    io(self).spawn_blast(gp, self.control.quat_for_control);
}

} // namespace

// ---- vector-vector-angle: b's heading seen from a's frame ----
float vector_vector_angle(const Vec& arg0, const Vec& arg1) {
    Mat gp, s4;
    forward_up_nopitch_to_inv_matrix(gp, Vec{0.0f, 0.0f, 1.0f, 1.0f}, arg1);
    matrix_transpose(s4, gp);
    Vec s3;
    vector_matrix_mul(s3, arg0, s4);
    return vector_y_angle(s3);
}

// ---- target-board-anim-trans: the lean target from the turn, the turn sound, the duck spring ----
void target_board_anim_trans(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    const float tf = kTargetBoardBank.turn_frames;
    {
        const float f26 = deg_minus(quaternion_vector_y_angle(c.dir_targ, c.local_normal),
                                    quaternion_vector_y_angle(c.quat_for_control, c.local_normal));
        const float f28 = deg_minus(quaternion_vector_y_angle(c.quat_for_control, c.local_normal),
                                    quaternion_vector_y_angle(c.last_quat_for_control, c.local_normal)) *
                          frames_per_second(self);
        const float f30 = b.turn_anim_targ;
        {
            const float f0_1 = lerp_scale(1.0f, -1.0f, f28, -32768.0f, 32768.0f);
            if (std::fabs(f0_1) < 0.1f || c.turn_to_magnitude < 0.3f) {
                b.turn_anim_mag = seek(b.turn_anim_mag, 0.0f, 8.0f * spf(self));
            } else {
                b.turn_anim_mag = seek(b.turn_anim_mag, f0_1, 2.0f * spf(self));
            }
        }
        b.turn_anim_targ = lerp_scale(tf / 2.0f, tf / -2.0f, f26, -5461.3335f, 5461.3335f) +
                           lerp_scale(0.3f * tf, -0.3f * tf, f28, -32768.0f, 32768.0f) +
                           lerp_scale(tf / 5.0f, tf / -5.0f, b.turn_anim_mag, 1.0f, -1.0f);
        if ((f30 * b.turn_anim_targ < 0.0f || f30 == 0.0f) &&
            (tf / 5.0f < std::fabs(f30 - b.turn_anim_targ) && current_time(self) >= c.turn_lockout_end_time)) {
            sound_fade(self, rt.unknown_sound_id00, 120);
            rt.unknown_sound_id00 = sound_play(self, "board-turn");
        }
    }
    b.turn_anim_duck_vel = 0.98f * b.turn_anim_duck_vel;
    b.turn_anim_duck_vel = b.turn_anim_duck_vel + -8.0f * spf(self);
    if (board_on_ground_p(self) && time_elapsed(self, b.unknown_time_frame02, seconds(0.2))) {
        if (c.status & cstat::impact_surface) {
            b.turn_anim_duck_vel =
                b.turn_anim_duck_vel + lerp_scale(0.0f, 15.0f, c.normal_impact_vel, 0.0f, 81920.0f);
        }
        const float f0_38 = vector_dot(b.up_vector[0], b.up_vector[1]);
        if (f0_38 < 1.0f) {
            b.turn_anim_duck_vel = b.turn_anim_duck_vel + lerp_scale(400.0f, 0.0f, f0_38, 0.6f, 1.0f) * spf(self);
        }
    }
}

// ---- target-board-spin-check: R1 spins (and flips with the stick), L1 grabs (hold, trickx) ----
void target_board_spin_check(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    const bool debug_r2 = rt.io->cheat_mode_debug() && target_cpad_hold(self, boardpad::r2);
    if ((target_cpad_pressed(self, boardpad::r1) ||
         (target_cpad_hold(self, boardpad::r1) && time_elapsed(self, b.spin_check_time, seconds(0.3)))) &&
        !(next_in(self, {StateId::target_board_trickx, StateId::target_board_hold}) &&
          !time_elapsed(self, b.spin_start_time, seconds(0.5))) &&
        !debug_r2) {
        set_time(self, b.spin_start_time);
        b.spin_start_dir = rt.io->joint_transform(3).v[2];
        vector_flatten(c.turn_to_alt_heading, c.c_R_w.v[2], Vec{0.0f, 1.0f, 0.0f, 1.0f});
        vector_normalize(c.turn_to_alt_heading, 1.0f);
        rot_to_dir_targ(self);
        b.trotyv_max = lerp_scale(218453.33f, 91022.22f,
                                  static_cast<float>(current_time(self) - c.last_time_on_surface), 0.0f, 300.0f);
        b.trotyv = 0.0f;
        b.troty = 0.0f;
        b.troty_cum = 0.0f;
        b.unknown_deg00 = 0.0f;
        rt.spin_sound_id = 0;
        c.mod_surface = &board_spin_mods;
    }
    if (target_cpad_hold(self, boardpad::r1) &&
        ((c.mod_surface->name == Symbol("spin") || !time_elapsed(self, b.spin_time, seconds(0.05))) && !debug_r2)) {
        b.turn_anim_tilt_p = Symbol();
        c.mod_surface = &board_spin_mods;
        set_time(self, b.spin_time);
        const float f28 = analog_input(stick_in(target_cpad_leftx(self)), 0.0f, 64.0f, 110.0f, 2184533.2f);
        const float f30 = analog_input(stick_in(target_cpad_lefty(self)), 0.0f, 96.0f, 110.0f, 1.0f);
        if (rt.spin_sound_id == 0 && 5461.3335f < std::fabs(b.trotyv)) {
            rt.spin_sound_id = sound_play(self, "board-spin-loop");
        }
        if (0.9f * std::fabs(b.spin_control) < std::fabs(f28) && b.spin_control * f28 >= 0.0f) {
            b.spin_control = f28;
        }
        if (!c.danger_mode) {
            target_danger_set(self, Symbol("board-spin"), Symbol());
        }
        // (PC port: the high frame rate fix's seconds-per-frame)
        b.trotyv = b.trotyv + b.spin_control * seconds_per_frame_high_fps(self);
        if (std::fabs(f28) < 1092266.6f && std::fabs(b.spin_control) < 1092266.6f &&
            0.9f * std::fabs(b.flip_control) < std::fabs(f30) && b.flip_control * f30 >= 0.0f &&
            !board_on_ground_p(self) && 0.0f < vector_dot(gravity_normal(self), c.transv) &&
            target_height_above_ground(self) >= 4096.0f) {
            b.flip_control = f30;
            go(self, StateId::target_board_flip,
               jump_args(kTargetBoardBank.jump_height_min, kTargetBoardBank.jump_height_max, halfpipe_sym(self)));
        }
    } else {
        c.mod_surface = rt.mods_backup;
    }
    set_time(self, b.spin_check_time);
    if ((target_cpad_pressed(self, boardpad::l1) || c.last_time_on_surface < b.unknown_time_frame06) && !debug_r2) {
        b.turn_anim_tilt_p = Symbol();
        c.mod_surface = &board_spin_mods;
        set_time(self, b.unknown_time_frame03);
        const float f30_1 = analog_input(stick_in(target_cpad_leftx(self)), 0.0f, 64.0f, 110.0f, 1.0f);
        const float f28_1 = analog_input(stick_in(target_cpad_lefty(self)), 0.0f, 96.0f, 110.0f, 1.0f);
        if (std::fabs(f30_1) < 0.5f && std::fabs(b.unknown_float00) < 0.5f && b.unknown_float01 == 0.0f &&
            0.9f * std::fabs(b.unknown_float01) < std::fabs(f28_1) && b.unknown_float01 * f28_1 >= 0.0f &&
            (!board_on_ground_p(self) && 0.0f < vector_dot(gravity_normal(self), c.transv) &&
             (target_height_above_ground(self) >= 4096.0f && seconds(0.165) < target_time_to_ground(self) &&
              time_elapsed(self, b.unknown_time_frame05, seconds(0.05)) &&
              b.unknown_time_frame05 < b.unknown_time_frame06))) {
            b.unknown_float01 = f28_1;
            go(self, StateId::target_board_hold,
               jump_args(kTargetBoardBank.tricky_jump_height_min, kTargetBoardBank.tricky_jump_height_max,
                         halfpipe_sym(self)));
        }
        if (std::fabs(f28_1) < 0.5f && std::fabs(b.unknown_float01) < 0.5f && b.unknown_float00 == 0.0f &&
            0.9f * std::fabs(b.unknown_float00) < std::fabs(f30_1) && b.unknown_float00 * f30_1 >= 0.0f &&
            (!board_on_ground_p(self) && 0.0f < vector_dot(gravity_normal(self), c.transv) &&
             (target_height_above_ground(self) >= 4096.0f && seconds(0.165) < target_time_to_ground(self) &&
              time_elapsed(self, b.unknown_time_frame05, seconds(0.05))))) {
            b.unknown_float00 = f30_1;
            go(self, StateId::target_board_trickx,
               jump_args(kTargetBoardBank.trickx_jump_height_min, kTargetBoardBank.trickx_jump_height_max,
                         halfpipe_sym(self)));
        }
    }
}

// ---- target-board-halfpipe-trans: the camera looks across the halfpipe ----
void target_board_halfpipe_trans(Target& self) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    Vec s5, cam14, local_down;
    if (focus_test(self, focus::halfpipe) && rt.io->camera_vectors(s5, cam14, local_down)) {
        Vec gp;
        vector_flatten(gp, c.edge_grab_across_edge_dir, local_down);
        if (vector_dot(gp, s5) < 0.0f) {
            const float f0_3 = vector_dot(c.edge_grab_edge_dir, cam14);
            Vec s4;
            vector_float_mul(s4, c.edge_grab_edge_dir, f0_3);
            vector_flatten(s4, s4, local_down);
            if (0.0f < vector_dot(s4, s5)) {
                vector_negate(gp, s4);
            }
        }
        vector_normalize(gp, 8192.0f);
        gp.y = gp.y + 40960.0f;
        vector_add(gp, gp, rt.io->get_trans(6));
        vector_sub(gp, gp, rt.io->get_trans(0));
        rt.io->camera_relative_position(&gp);
        rt.io->set_setting("matrix-blend-max-angle", Symbol("abs"), degrees(12.000001));
        rt.io->set_setting("matrix-blend-max-partial", Symbol("abs"), 0.5f);
    }
}

// ---- target-board-resolve-points: spin points on landing, then the list's flush ----
void target_board_resolve_points(Target& self) {
    BoardInfo& b = *self.board;
    if (runtime(self).main.mode == JointModMode::rotate) {
        const float f0_1 = std::fabs(b.troty_cum);
        if (f0_1 >= 191146.67f) {
            add_to_trick_list(self, BoardTricks::board_spin, 4000.0f);
        } else if (f0_1 >= 126520.89f) {
            add_to_trick_list(self, BoardTricks::board_spin, 2000.0f);
        } else if (f0_1 >= 49152.0f) {
            add_to_trick_list(self, BoardTricks::board_spin, 500.0f);
        }
    }
    b.troty_cum = 0.0f;
    b.flip_count = 0;
    if (time_elapsed(self, b.in_air_time, seconds(0.2))) {
        flush_trick_list(self);
    }
}

// ---- target-board-ground-check: landing a spin straight gives a boost ----
void target_board_ground_check(Target& self) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    if (board_on_ground_p(self)) {
        if (b.troty_cum != 0.0f) {
            const float f30 = std::fabs(b.troty_cum);
            float f28 = vector_y_angle(c.transv);
            const float f0_2 = control_y_angle(self);
            if (!time_elapsed(self, b.halfpipe_time, seconds(0.1))) {
                f28 = 32768.0f + f28;
            }
            const float f0_5 = std::fabs(deg_diff(f0_2, f28));
            if (f30 >= 49152.0f && 4551.1113f >= f0_5) {
                if (f30 >= 191146.67f) {
                    add_to_trick_list(self, BoardTricks::board_boost, 2000.0f);
                } else if (f30 >= 126520.89f) {
                    add_to_trick_list(self, BoardTricks::board_boost, 1000.0f);
                } else {
                    add_to_trick_list(self, BoardTricks::board_boost, 500.0f);
                }
                const float f0_8 = lerp_scale(20480.0f, 40960.0f, f30, 49152.0f, 182044.44f);
                vector_add_float_mul(c.transv, c.transv, c.c_R_w.v[2], f0_8);
                if (vector_length(c.transv) < 114688.0f) {
                    vector_normalize(c.transv, 114688.0f);
                }
                if (!(c.old_status & cstat::on_surface)) {
                    particles_at_lift(self, "group-board-land-straight");
                    io(self).cpad_buzz(1, 255, seconds(0.3));
                    sound_play(self, "board-boost");
                }
            }
        }
        target_board_resolve_points(self);
    }
    b.turn_anim_tilt_p = kTrue;
}

// ---- target-board-halfpipe-check: rolling up a halfpipe wall starts target-board-halfpipe ----
void target_board_halfpipe_check(Target& self) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    Vec gp;
    vector_flatten(gp, gravity_normal(self), c.ground_contact_normal);
    vector_normalize(gp, 1.0f);
    if (board_on_ground_p(self) && c.ground_pat.mode() == kPatHalfpipe && 0.0f < vector_dot(c.transv, gp) &&
        (c.surface_angle < 0.3f && b.halfpipe_side_time == 0)) {
        set_time(self, b.halfpipe_side_time);
        root_prim_action(self) |= caction::no_normal_reset;
    } else {
        if (!board_on_ground_p(self) && b.halfpipe_side_time != 0 &&
            time_elapsed(self, b.halfpipe_side_time, seconds(0.05))) {
            go(self, StateId::target_board_halfpipe);
        }
        if (time_elapsed(self, b.halfpipe_side_time, seconds(0.2))) {
            b.halfpipe_side_time = 0;
            root_prim_action(self) &= ~caction::no_normal_reset;
        }
    }
}

// ---- target-board-jump-trans: landing, wall smacks, the kick, the spin check ----
void target_board_jump_trans(Target& self) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    if (self.state_time != current_time(self) && jump_hit_ground_stuck_p(self)) {
        set_time(self, b.jump_land_time);
        go(self, StateId::target_board_hit_ground);
    }
    if (time_elapsed(self, self.state_time, seconds(0.1))) {
        target_board_smack_surface_p(self);
    }
    if (target_cpad_pressed(self, boardpad::x) && vector_dot(gravity_normal(self), c.transv) < 12288.0f &&
        -61440.0f < vector_dot(gravity_normal(self), c.transv) &&
        (!(io(self).water_flags() & waterflag::touch_water) && !(self.state_flags & sflags::prevent_jump) &&
         !next_state_is(self, StateId::target_board_wall_kick))) {
        go(self, StateId::target_board_jump_kick);
    }
    target_board_spin_check(self);
}

// ============================================================================================
// The states. Each GOAL defstate becomes functions named <state>_<hook>; state_def at the end
// ties them into the table process.cpp runs.
// ============================================================================================

namespace {

void post_target(Target& self) { target_post(self); }
void post_board(Target& self) { target_board_post(self); }
void exit_board(Target& self) { target_board_exit(self); }
Symbol event_board(Target& self, const EventMessage& e) { return target_board_handler(self, e); }

// The ground-check / mods choice ending stance's and duck-stance's trans.
void stance_mods(Target& self, Surface* ground, Surface* jump) {
    ControlInfo& c = self.control;
    if (board_on_ground_p(self)) {
        target_board_ground_check(self);
        set_mods(self, ground);
    } else if (time_elapsed(self, c.last_time_on_surface, seconds(0.1)) &&
               (c.mod_surface->name == Symbol("spin") || 4096.0f < target_height_above_ground(self))) {
        set_mods(self, jump);
        target_board_spin_check(self);
    } else {
        set_mods(self, &board_air_mods);
    }
}

// The x press both stances share: a halfpipe boost, or the jump (`duck` picks duck-stance's).
void stance_jump_press(Target& self, bool duck) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    // og:preserve-this High FPS Fix (recently-pressed?), as OpenGOAL.
    if (target_recently_pressed(self, boardpad::x) && can_jump_p(self, Symbol("board"))) {
        Vec gp;
        vector_flatten(gp, gravity_normal(self), c.ground_contact_normal);
        vector_normalize(gp, 1.0f);
        if (!time_elapsed(self, b.halfpipe_jump_time, seconds(0.5)) && c.ground_pat.mode() == kPatHalfpipe) {
        } else if (c.ground_pat.mode() == kPatHalfpipe && 0.0f < vector_dot(c.transv, gp) &&
                   c.surface_angle < 0.5f) {
            set_time(self, b.halfpipe_jump_time);
            vector_float_mul(c.transv, c.transv, 1.5f);
        } else {
            flush_trick_list(self);
            if (duck) {
                // (deg-diff (vector-y-angle c-R-w z) (vector-y-angle to-target-pt-xz)): result unused.
                go(self, StateId::target_board_jump,
                   jump_args(kTargetBoardBank.duck_jump_height_min, kTargetBoardBank.duck_jump_height_max,
                             Symbol("surface"), &board_duck_jump_mods));
            } else {
                go(self, StateId::target_board_jump,
                   jump_args(kTargetBoardBank.jump_height_min, kTargetBoardBank.jump_height_max));
            }
        }
    }
}

void stance_spin_ground(Target& self) {
    BoardInfo& b = *self.board;
    if (target_cpad_pressed(self, boardpad::r1)) {
        set_time(self, b.spin_ground_press_time);
    }
    if (target_cpad_hold(self, boardpad::r1) && can_feet_p(self, kTrue) &&
        !time_elapsed(self, b.spin_ground_press_time, seconds(0.3)) && turn_around_p(self)) {
        set_time(self, b.spin_ground_start_time);
    }
}

// ---- target-board-start ----
void start_code(Target& self, Co& co, const StateArgs&) {
    CO_BEGIN(co);
    target_board_init(self);
    remove_exit(self);
    go(self, StateId::target_board_get_on);
    CO_END(co);
}

// ---- target-board-stance ----
Symbol stance_event(Target& self, const EventMessage& e) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    if (e.message == Symbol("lip")) {
        if (c.ground_pat.mode() == kPatHalfpipe && c.poly_angle < 0.5f) {
            Vec s5;
            vector_flatten(s5, gravity_normal(self), c.ground_contact_normal);
            vector_normalize(s5, 1.0f);
            if (0.0f < vector_dot(c.transv, s5)) {
                set_time(self, b.halfpipe_lip_time);
                set_time(self, b.halfpipe_side_time);
                b.halfpipe_lip_event = e.sym[0];
                root_prim_action(self) |= caction::no_normal_reset;
                return kTrue;
            }
        }
        return Symbol();
    }
    if (e.message == Symbol("jump")) {
        go(self, StateId::target_board_jump, jump_args(e.f[0], e.f[1], e.sym[2]));
        return Symbol();
    }
    return target_board_handler(self, e);
}

void stance_enter(Target& self, const StateArgs&) { set_mods(self, &board_walk_mods); }

void stance_exit(Target& self) {
    BoardInfo& b = *self.board;
    if (!next_in(self, {StateId::target_board_stance, StateId::target_board_duck_stance})) {
        b.halfpipe_side_time = 0;
    }
    root_prim_action(self) &= ~caction::no_normal_reset;
    b.turn_anim_tilt_p = Symbol();
    if (!next_in(self, {StateId::target_board_stance, StateId::target_board_duck_stance,
                        StateId::target_board_ride_edge})) {
        b.turn_anim_frame = 0.0f;
        b.turn_anim_mag = 0.0f;
        b.turn_anim_vel = 0.0f;
        b.turn_anim_duck = 0.0f;
    }
    target_board_exit(self);
}

void stance_trans(Target& self) {
    ControlInfo& c = self.control;
    if (target_cpad_hold(self, boardpad::l1) && !(self.state_flags & sflags::prevent_duck) &&
        !time_elapsed(self, c.last_time_on_surface, target_bank.ground_timeout)) {
        go(self, StateId::target_board_duck_stance);
    }
    stance_jump_press(self, false);
    stance_spin_ground(self);
    target_board_halfpipe_check(self);
    target_board_smack_surface_p(self);
    target_board_anim_trans(self);
    stance_mods(self, &board_walk_mods, &board_jump_mods);
}

// Channel 3: the duck (or, ducking negative, the air turn) blended over the tween by turn-anim-duck.
void stance_duck_channel(Target& self) {
    BoardInfo& b = *self.board;
    JointControl& j = jc(self);
    const int idx = j.root + 3;
    JointControlChannel& a0 = j.channel[idx];
    if (b.turn_anim_duck >= 0.0f) {
        const float f0 = b.turn_anim_duck;
        a0.frame_interp[1] = f0;
        a0.frame_interp[0] = f0;
        a0.frame_group = AnimId::jakb_board_duck_turn;
        a0.param[0] = 0.0f;
        joint_control_channel_group_eval(j, idx, AnimId::jakb_board_duck_turn, NumFunc::chan, anim_clock(self));
    } else {
        const float f0 = -b.turn_anim_duck;
        a0.frame_interp[1] = f0;
        a0.frame_interp[0] = f0;
        a0.frame_group = AnimId::jakb_board_air_turn;
        a0.param[0] = 0.0f;
        joint_control_channel_group_eval(j, idx, AnimId::jakb_board_air_turn, NumFunc::chan, anim_clock(self));
    }
}

void stance_code(Target& self, Co& co, const StateArgs&) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    CO_BEGIN(co);
    co.f[0] = 0.0f;  // f30-0
    if (!(group(self) == AnimId::jakb_board_turn_left && ja_group_size(jc(self)) == 4)) {
        if (group(self) == AnimId::jakb_board_get_on) {
            ja_channel_set(jc(self), 1, anim_clock(self));
            ja(self, Ja().group(AnimId::jakb_board_get_on_land).seek_max(1.8f).frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek_max(1.8f));
            }
            CO_SUSPEND(co);
            b.turn_anim_duck = 1.0f;
            b.turn_anim_duck_vel = 15.0f;
        }
        push(self, 4, 30);
        co.f[0] = ja_frame_num(jc(self), 0);
    }
    ground_tween_initialize(self, b.stance_info, 0, AnimId::jakb_board_turn, AnimId::jakb_board_turn_up,
                            AnimId::jakb_board_turn_down, AnimId::jakb_board_turn_right,
                            AnimId::jakb_board_turn_left, co.f[0]);
    ja(self, Ja().chan(3).group(AnimId::jakb_board_duck_turn));
    while (true) {
        target_board_turn_anim(self, 0);
        {
            float f0 = 1.6f * c.ctrl_slope_z;
            float f1 = 1.6f * c.ctrl_slope_x;
            if (!b.turn_anim_tilt_p) {
                f0 = 0.0f;
                f1 = 0.0f;
            } else if (c.ground_pat.mode() == kPatHalfpipe) {
                f0 = 0.0f;
            }
            ground_tween_update(self, b.stance_info, f0, f1, 0.1f);
        }
        ja(self, Ja().chan(1).from_chan(0));
        ja(self, Ja().chan(2).from_chan(0));
        stance_duck_channel(self);
        CO_SUSPEND(co);
        can_play_stance_ambient(self);
    }
    CO_END(co);
}

// ---- target-board-duck-stance ----
void duck_stance_enter(Target& self, const StateArgs&) { set_mods(self, &board_duck_mods); }

void duck_stance_exit(Target& self) {
    self.board->turn_anim_duck_vel = -8.0f;
    stance_exit(self);
}

void duck_stance_trans(Target& self) {
    ControlInfo& c = self.control;
    if ((!target_cpad_hold(self, boardpad::l1) || (self.state_flags & sflags::prevent_duck) ||
         time_elapsed(self, c.last_time_on_surface, target_bank.ground_timeout)) &&
        can_exit_duck_p(self)) {
        go(self, StateId::target_board_stance);
    }
    stance_jump_press(self, true);
    stance_spin_ground(self);
    target_board_halfpipe_check(self);
    target_board_smack_surface_p(self);
    target_board_anim_trans(self);
    self.board->turn_anim_duck_vel = 8.0f;
    stance_mods(self, &board_duck_mods, &board_duck_jump_mods);
}

// ---- target-board-jump ----
// arg2 is a symbol or a surface: StateArgs::sym "surface" with surf set means a surface.
Symbol jump_event(Target& self, const EventMessage& e) {
    if (e.message == Symbol("edge-grab") && !time_elapsed(self, self.state_time, seconds(0.1))) {
        return Symbol();
    }
    return target_board_handler(self, e);
}

void jump_enter(Target& self, const StateArgs& args) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    Symbol arg2 = args.sym;
    Surface* arg2_surf = args.surf;
    if (arg2 == Symbol("hit")) {
        arg2 = Symbol();
        arg2_surf = nullptr;
    }
    if (!time_elapsed(self, b.ride_time, seconds(0.5))) {
        arg2 = Symbol("surface");
        arg2_surf = &board_ride_jump_mods;
        Vec s2;
        vector_normalize_copy(s2, c.transv, 1.0f);
        Vec up;
        vector_y_quaternion(up, c.dir_targ);
        forward_up_nopitch_to_quaternion(c.dir_targ, s2, up);
        c.turn_lockout_end_time =
            current_time(self) +
            static_cast<TimeFrame>(lerp_scale(225.0f, 0.0f, vector_dot(s2, c.c_R_w.v[2]), -1.0f, 1.0f));
    }
    c.dynam->gravity_length = 245760.0f;
    set_time(self, self.state_time);
    float f30 = 0.0f;
    if (0.0f < b.shock_offsetv && !time_elapsed(self, b.jump_land_time, seconds(0.5)) &&
        time_elapsed(self, b.ride_time, seconds(0.5))) {
        particles_at_lift(self, "group-board-quick-jump");
        f30 = 8192.0f + f30;
        add_to_trick_list(self, BoardTricks::board_quick_jump, 0.0f);
        sound_play(self, "board-q-jump");
    } else {
        if (arg2_surf == &board_duck_jump_mods) {
            add_to_trick_list(self, BoardTricks::board_duck_jump, 0.0f);
        } else {
            add_to_trick_list(self, BoardTricks::board_jump, 0.0f);
        }
        rt.unknown_sound_id02 = sound_play(self, "board-jump");
    }
    init_var_jump(self, args.f0 + f30, args.f1 + f30, true, false, c.transv,
                  time_elapsed(self, b.unknown_time_frame00, seconds(0.1)) ? 2.0f : 0.0f);
    c.status &= ~(cstat::on_surface | cstat::on_ground | cstat::touch_surface);
    // (if arg2 arg2 *board-jump-mods*): a truthy non-surface symbol would be stored as the surface in
    // GOAL; the board's callers pass #f, 'hit (made #f above) or a surface.
    Surface* s = arg2_surf ? arg2_surf : &board_jump_mods;
    set_mods(self, s);
    b.shock_offsetv = 0.0f;
    c.unknown_float36 = std::fmax(0.0f, std::fmin(1.0f, 0.00004359654f * (-11468.8f + c.ctrl_xz_vel)));
    b.slow_transv = c.transv;
}

void jump_trans(Target& self) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    target_board_jump_trans(self);
    // og:preserve-this abutton indexing
    c.unknown_float36 = std::fmax(c.unknown_float36,
                                  0.003921569f * static_cast<float>(target_cpad_abutton_x(self)));
    mod_var_jump(self, true, false, target_cpad_hold(self, boardpad::x), c.transv);
    b.slow_transv = c.transv;
    b.shock_offset = 0.8f * b.shock_offset;
}

void jump_code(Target& self, Co& co, const StateArgs& args) {
    ControlInfo& c = self.control;
    CO_BEGIN(co);
    if (args.sym == Symbol("hit")) {
        push(self, 1, seconds(0.05));
        ja(self, Ja().group(AnimId::jakb_board_hit).seek().frame_num(0.0f).no_eval());
        while (!done(self)) {
            CO_SUSPEND(co);
            ja(self, Ja().seek());
        }
        push(self, 1, seconds(0.05));
    } else {
        push(self, 1, seconds(0.05));
        ja(self, Ja().group(AnimId::jakb_board_jump).min());
        CO_SUSPEND(co);
        while (!done(self)) {
            {
                const float f30 = vector_dot(gravity_normal(self), c.transv);
                const float f0 = 10.0f - ja_aframe_num(jc(self), 0);
                JointControl& j = jc(self);
                JointControlChannel& gp = j.root_channel(0);
                gp.param[0] = static_cast<float>(anim_info(gp.frame_group).num_frames + -1);
                const bool v1 = 0.0f < f30 && 0.0f < f0;
                if (v1) {
                    gp.param[1] = std::fmin(std::fmin(1.5f, f0),
                                            goal_div(5.0f * f0, static_cast<float>(time_to_apex(f30, -245760.0f))));
                } else if (seconds(0.165) < target_time_to_ground(self)) {
                    gp.param[1] = 0.5f;
                } else {
                    gp.param[1] = 0.25f;
                }
                joint_control_channel_group_eval(j, j.root, AnimId::none, NumFunc::seek, anim_clock(self));
            }
            CO_SUSPEND(co);
        }
        push(self, 1, seconds(0.05));
    }
    ja(self, Ja().group(AnimId::jakb_board_jump_loop));
    while (true) {
        CO_SUSPEND(co);
        ja(self, Ja().loop());
    }
    CO_END(co);
}

// ---- target-board-halfpipe ----
// The let* that keeps a speed's part across `dir` at `along` and the rest as it is.
void set_across(Vec& transv, const Vec& dir, float along) {
    Vec v1;
    const float f0 = vector_dot(dir, transv);
    vector_float_mul(v1, dir, f0);
    vector_sub(v1, transv, v1);
    const float f0_3 = vector_length(v1);
    const float f1_1 = f0_3;
    Vec t;
    vector_float_mul(t, dir, along);
    vector_float_mul(v1, v1, goal_div(f0_3, f1_1));
    vector_add(transv, t, v1);
}

Symbol halfpipe_event(Target& self, const EventMessage& e) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    if (e.message == Symbol("edge-grab")) {
        // (format #t "halfpipe found edge ~m~%" ...): debug output, left out.
        rt.word04.set(true);
        const EdgeGrabInfo& v1 = edge_grab_info();
        vector_sub(c.edge_grab_edge_dir, v1.world_vertex[0], v1.world_vertex[1]);
        vector_normalize(c.edge_grab_edge_dir, 1.0f);
        vector_cross(c.edge_grab_across_edge_dir, c.edge_grab_edge_dir, gravity_normal(self));
        set_across(c.transv, c.edge_grab_across_edge_dir, -2048.0f);
        return kTrue;
    }
    if (e.message == Symbol("ramp")) {
        if (!rt.did_move_to_pole) {
            rt.did_move_to_pole = kTrue;
            return kTrue;
        }
        return Symbol();
    }
    if (e.message == Symbol("grenade")) {
        if (e.f[0] < vector_dot(gravity_normal(self), c.transv)) {
            StateArgs a;
            a.handle = e.from;
            go(self, StateId::target_board_grenade, a);
        }
        return Symbol();
    }
    return target_board_handler(self, e);
}

void halfpipe_enter(Target& self, const StateArgs&) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    self.focus_status |= focus::halfpipe;
    set_time(self, self.state_time);
    root_prim_action(self) |= caction::no_normal_reset;
    set_mods(self, &board_halfpipe_mods);
    c.sliding_start_time = 0;
    if (vector_dot(c.c_R_w.v[1], gravity_normal(self)) < 0.7f) {
        vector_matrix_mul(c.transv_ctrl, c.transv, c.w_R_c);
        c.transv_ctrl.y = 0.0f;
        vector_matrix_mul(c.transv, c.transv_ctrl, c.c_R_w);
    }
    rt.word04.set(false);
    rt.did_move_to_pole = Symbol();
    do_edge_grabs(self, collide_edge_board_halfpipe_spec());
    if (!rt.word04.truthy()) {
        Vec& gp = vector_flatten(c.edge_grab_across_edge_dir, gravity_normal(self), c.ground_contact_normal);
        gp.y = 0.0f;
        vector_normalize(gp, 1.0f);
        vector_rotate_y(c.edge_grab_edge_dir, c.edge_grab_across_edge_dir, 16384.0f);
        set_across(c.transv, gp, -2048.0f);
    }
    Vec z, n;
    vector_z_quaternion(z, c.quat_for_control);
    vector_normalize_copy(n, c.transv, 1.0f);
    if (vector_dot(gravity_normal(self), c.transv) < 20480.0f || vector_dot(z, n) < 0.3f) {
        go(self, StateId::target_board_turn_to, turn_to_args(c.transv, seconds(0.5)));
    }
}

void halfpipe_exit(Target& self) {
    if (!next_in(self, {StateId::target_board_flip, StateId::target_board_hold, StateId::target_board_trickx,
                        StateId::target_board_grenade, StateId::target_board_falling})) {
        self.focus_status &= ~focus::halfpipe;
    }
    io(self).camera_relative_position(nullptr);
    io(self).remove_setting("matrix-blend-max-angle");
    io(self).remove_setting("matrix-blend-max-partial");
    root_prim_action(self) &= ~caction::no_normal_reset;
    target_board_exit(self);
}

void halfpipe_trans(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    if ((rt.did_move_to_pole == kTrue ||
         (!time_elapsed(self, b.halfpipe_lip_time, seconds(0.1)) && b.halfpipe_lip_event == Symbol("lipramp"))) &&
        0.0f < vector_dot(gravity_normal(self), c.transv)) {
        rt.did_move_to_pole = Symbol("done");
        Vec v1;
        const float f0 = vector_dot(gravity_normal(self), c.transv);
        vector_float_mul(v1, gravity_normal(self), f0);
        vector_sub(v1, c.transv, v1);
        float f1 = vector_length(v1);
        const float f2 = f1;
        if (f1 != 0.0f) {
            f1 = 0.0f;
        }
        Vec t;
        vector_float_mul(t, gravity_normal(self), f0);
        vector_float_mul(v1, v1, goal_div(f1, f2));
        vector_add(c.transv, t, v1);
    }
    set_time(self, b.halfpipe_time);
    if (c.gspot_pat_surfce.mode() == kPatHalfpipe) {
        set_time(self, b.halfpipe_gspot_time);
    }
    if (jump_hit_ground_stuck_p(self)) {
        vector_float_mul(c.transv, c.transv, 1.5f);
        go(self, StateId::target_board_turn_to, turn_to_args(c.transv, seconds(0.5)));
    }
    if (target_cpad_pressed(self, boardpad::x) && time_elapsed(self, b.halfpipe_jump_time, seconds(0.6)) &&
        !time_elapsed(self, self.state_time, target_bank.ground_timeout) &&
        0.0f < vector_dot(gravity_normal(self), c.transv)) {
        set_time(self, b.halfpipe_jump_time);
        vector_float_mul(c.transv, c.transv, 1.5f);
    }
    target_board_spin_check(self);
    if (c.sliding_start_time > 0) {
        if (0.7f < c.turn_to_magnitude) {
            Vec gp = c.edge_grab_across_edge_dir;
            gp.y = 0.0f;
            vector_normalize(gp, 1.0f);
            if (0.7f < vector_dot(c.to_target_pt_xz, gp)) {
                c.sliding_start_time = c.sliding_start_time + (current_time(self) - old_frame_counter(self));
            } else {
                c.sliding_start_time = 1;
            }
        } else {
            c.sliding_start_time = 1;
        }
    } else if (target_cpad_stick0_speed(self) == 0.0f &&
               b.halfpipe_lip_time < self.state_time + seconds(-0.2)) {
        c.sliding_start_time = 1;
    }
    if (c.sliding_start_time >= seconds(0.05) ||
        (time_elapsed(self, b.halfpipe_gspot_time, seconds(0.5)) &&
         b.halfpipe_lip_time < self.state_time + seconds(-0.2))) {
        c.transv.x = c.transv.x + 20480.0f * c.edge_grab_across_edge_dir.x;
        c.transv.z = c.transv.z + 20480.0f * c.edge_grab_across_edge_dir.z;
        go(self, StateId::target_board_stance);
    }
    target_board_halfpipe_trans(self);
}

void halfpipe_code(Target& self, Co& co, const StateArgs& args) {
    BoardRuntime& rt = runtime(self);
    CO_BEGIN(co);
    if (!time_elapsed(self, self.board->halfpipe_jump_time, seconds(0.5))) {
        push(self, 1, seconds(0.05));
        ja(self, Ja().group(AnimId::jakb_board_jump).seek(aframe(8.0f), 0.5f).frame_num(0.0f).no_eval());
        while (!done(self)) {
            CO_SUSPEND(co);
            ja(self, Ja().seek(aframe(8.0f), 0.5f));
        }
        ja(self, Ja().group(AnimId::jakb_board_jump).seek(aframe(10.0f), 0.2f).frame_num(aframe(8.0f)).no_eval());
        while (!done(self)) {
            CO_SUSPEND(co);
            ja(self, Ja().seek(aframe(10.0f), 0.2f));
        }
        ja(self, Ja().group(AnimId::jakb_board_jump).seek(aframe(12.0f), 0.1f).frame_num(aframe(10.0f)).no_eval());
        while (!done(self)) {
            CO_SUSPEND(co);
            ja(self, Ja().seek(aframe(12.0f), 0.1f));
        }
        sleep_code(self);
        CO_SUSPEND(co);
    } else {
        // ((-> target-board-stance code)): the stance's code runs as this one, on its own resume point.
        rt.sub_co = Co();
        while (true) {
            stance_code(self, rt.sub_co, args);
            CO_SUSPEND(co);
        }
    }
    CO_END(co);
}

// ---- target-board-falling ----
void falling_enter(Target& self, const StateArgs&) {
    if (io(self).cheat_mode_debug() && target_cpad_hold(self, boardpad::r2) && !io(self).pause_lock()) {
        self.control.mod_surface = &board_air_mods;
    } else {
        self.control.mod_surface = &board_jump_mods;
    }
    runtime(self).mods_backup = self.control.mod_surface;
}

void falling_trans(Target& self) {
    target_board_jump_trans(self);
    if (focus_test(self, focus::halfpipe)) {
        target_board_halfpipe_trans(self);
    }
    if (io(self).cheat_mode_debug() && target_cpad_hold(self, boardpad::r2) && !io(self).pause_lock()) {
        set_mods(self, &board_air_mods);
    }
}

void falling_code(Target& self, Co& co, const StateArgs&) {
    CO_BEGIN(co);
    {
        const AnimId g = group(self);
        if (g == AnimId::jakb_board_jump_loop) {
        } else if (g == AnimId::jakb_board_noseflip) {
            ja(self, Ja().seek().no_eval());
            push(self, 1, seconds(0.5));
        } else if (g == AnimId::jakb_board_flip_backward || g == AnimId::jakb_board_flip_backward_loop ||
                   g == AnimId::jakb_board_flip_forward || g == AnimId::jakb_board_flip_forward_loop) {
            ja(self, Ja().seek().no_eval());
            push(self, 1, seconds(0.2));
        } else if (g == AnimId::jakb_board_spin) {
            ja(self, Ja().seek().no_eval());
            push(self, 1, seconds(0.1));
        } else {
            push(self, 1, seconds(0.5));
        }
    }
    if (focus_test(self, focus::halfpipe)) {
        ja(self, Ja().group(AnimId::jakb_board_duck_turn).identity(aframe(0.0f)));
        while (true) {
            CO_SUSPEND(co);
            ja_blend_eval(jc(self), anim_clock(self));
        }
    } else {
        ja(self, Ja().group(AnimId::jakb_board_jump_loop));
        while (true) {
            CO_SUSPEND(co);
            ja_blend_eval(jc(self), anim_clock(self));
            ja(self, Ja().loop());
        }
    }
    CO_END(co);
}

// ---- target-board-jump-kick ----
void jump_kick_enter(Target& self, const StateArgs&) {
    set_time(self, self.state_time);
    set_mods(self, &board_jump_mods);
    sound_play(self, "board-k-jump");
}

void jump_kick_trans(Target& self) {
    BoardInfo& b = *self.board;
    if (jump_hit_ground_stuck_p(self)) {
        go(self, StateId::target_board_hit_ground);
    }
    target_board_smack_surface_p(self);
    if (!time_elapsed(self, b.smack_surface_time, seconds(0.2))) {
        StateArgs a;
        a.vec = b.smack_normal;
        a.f0 = b.smack_speed;
        go(self, StateId::target_board_wall_kick, a);
    } else if (!time_elapsed(self, b.glance_time, seconds(0.2))) {
        StateArgs a;
        a.vec = vector_normalize_copy(self.control.unknown_vector38, b.glance_out_transv, 1.0f);
        a.f0 = b.glance_speed;
        go(self, StateId::target_board_wall_kick, a);
    }
}

void jump_kick_code(Target& self, Co& co, const StateArgs&) {
    CO_BEGIN(co);
    push(self, 1, seconds(0.1));
    ja(self, Ja().group(AnimId::jakb_board_jump_kick).seek().frame_num(0.0f).no_eval());
    while (!done(self)) {
        CO_SUSPEND(co);
        ja(self, Ja().seek());
    }
    push(self, 1, seconds(2));
    ja(self, Ja().group(AnimId::jakb_board_jump_loop));
    while (true) {
        CO_SUSPEND(co);
        ja(self, Ja().loop());
    }
    CO_END(co);
}

// ---- target-board-wall-kick (arg0 StateArgs::vec, arg1 StateArgs::f0) ----
void wall_kick_enter(Target& self, const StateArgs& args) {
    ControlInfo& c = self.control;
    sound_play(self, "board-kick");
    c.transv = args.vec;
    vector_xz_normalize(c.transv, 0.8f * args.f0);
    // (the-as uint (fmax ...)): the float's bits in the overlay word, read back as a float in code.
    runtime(self).word04.set_float(std::fmax(81920.0f, 0.8f * args.f0));
    c.unknown_vector38 = args.vec;
    c.transv.y = 0.0f;
    Vec n;
    vector_negate(n, c.transv);
    vector_turn_to(self, n);
    c.transv.y = std::sqrt(24576.0f * c.dynam->gravity_length) - 0.008333334f * -c.dynam->gravity_length;
    set_time(self, self.state_time);
    set_mods(self, &board_wall_kick_mods);
    self.board->smack_surface_time = 0;
}

void wall_kick_code(Target& self, Co& co, const StateArgs&) {
    ControlInfo& c = self.control;
    CO_BEGIN(co);
    ja(self, Ja().group(AnimId::jakb_board_jump_kickoff).seek().frame_num(0.0f).no_eval());
    while (!done(self)) {
        if (jump_hit_ground_stuck_p(self) && 31.0f < ja_aframe_num(jc(self), 0)) {
            go(self, StateId::target_board_hit_ground);
        }
        {
            const Vec& v1 = c.unknown_vector38;
            c.transv.x = v1.x;
            c.transv.z = v1.z;
        }
        vector_xz_normalize(c.transv, runtime(self).word04.as_float());
        // (compute-alignment! (-> self align)) (align! ... adjust-quat 1 1 1): the animation's root
        // turn, which needs the art's root channel; left to the adapter (no art data in the port).
        CO_SUSPEND(co);
        ja(self, Ja().seek());
    }
    // enter-state: a bare symbol in the GOAL source, nothing.
    go(self, StateId::target_board_falling);
    CO_END(co);
}

// ---- target-board-flip (arg0, arg1 heights, arg2 symbol) ----
void flip_enter(Target& self, const StateArgs& args) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    rt.word04.set(args.sym);
    Vec up;
    vector_y_quaternion(up, c.dir_targ);
    forward_up_nopitch_to_quaternion(c.dir_targ, c.transv, up);
    c.dynam->gravity_length = 245760.0f;
    set_time(self, self.state_time);
    if (args.sym == Symbol("halfpipe")) {
        self.focus_status |= focus::halfpipe;
        target_board_halfpipe_trans(self);
    } else {
        init_var_jump(self, args.f0, args.f1, true, false, c.transv, 1.0f);
        c.status &= ~(cstat::on_surface | cstat::on_ground | cstat::touch_surface);
        c.unknown_float36 = std::fmax(0.0f, std::fmin(1.0f, 0.00004359654f * (-11468.8f + c.ctrl_xz_vel)));
    }
    set_mods(self, &board_flip_mods);
    sound_fade(self, rt.unknown_sound_id02, 72);
    rt.control_unknown_sound_id00 = sound_play(self, "board-flip");
    target_start_attack(self);
    target_danger_set(self, Symbol("board-spin"), Symbol());
}

void flip_exit(Target& self) {
    BoardInfo& b = *self.board;
    target_danger_set(self, Symbol("harmless"), Symbol());
    b.flip_control = 0.0f;
    add_to_trick_list(self, BoardTricks::board_flip,
                      500.0f * static_cast<float>(b.flip_count) + (b.flip_count >= 2 ? 2000.0f : 0.0f));
    sound_fade(self, runtime(self).control_unknown_sound_id00, 24);
    halfpipe_exit(self);
}

void flip_trans(Target& self) {
    ControlInfo& c = self.control;
    if (runtime(self).word04.symbol() == Symbol("halfpipe")) {
        target_board_halfpipe_trans(self);
    } else {
        // og:preserve-this abutton indexing
        c.unknown_float36 = std::fmax(c.unknown_float36,
                                      0.003921569f * static_cast<float>(target_cpad_abutton_x(self)));
        mod_var_jump(self, true, false, target_cpad_hold(self, boardpad::x), c.transv);
    }
    self.board->slow_transv = c.transv;
}

// co.b[0] gp-0 (forward), co.f[0] s5-0's z, co.f[1] f0-9, co.i[0] s4-0.
void flip_code(Target& self, Co& co, const StateArgs&) {
    BoardInfo& b = *self.board;
    CO_BEGIN(co);
    co.b[0] = b.flip_control >= 0.0f;
    co.f[0] = -0.0078125f * (-128.0f + static_cast<float>(target_cpad_lefty(self)));
    co.f[1] = analog_input(static_cast<int32_t>(128.0f * co.f[0]), 0.0f, 96.0f, 110.0f, 1.0f);
    co.i[0] = 0;
    while (!((!target_cpad_hold(self, boardpad::r1) ||
              (co.f[1] == 0.0f || jump_hit_ground_stuck_p(self) || target_time_to_ground(self) < seconds(0.5))) &&
             co.i[0] != 0)) {
        if (co.b[0]) {
            if (co.i[0] == 0) {
                push(self, 1, seconds(0.1));
                ja(self, Ja().group(AnimId::jakb_board_flip_forward).seek(aframe(15.0f)).frame_num(0.0f).no_eval());
                while (!done(self)) {
                    CO_SUSPEND(co);
                    ja(self, Ja().seek(aframe(15.0f)));
                }
            } else {
                ja(self, Ja().group(AnimId::jakb_board_flip_forward_loop).seek().frame_num(0.0f).no_eval());
                while (!done(self)) {
                    CO_SUSPEND(co);
                    ja(self, Ja().seek());
                }
            }
        } else if (co.i[0] == 0) {
            push(self, 1, seconds(0.1));
            ja(self, Ja().group(AnimId::jakb_board_flip_backward).seek(aframe(10.0f)).frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek(aframe(10.0f)));
            }
        } else {
            ja(self, Ja().group(AnimId::jakb_board_flip_backward_loop).seek().frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek());
            }
        }
        b.flip_count = b.flip_count + 1;
        co.f[1] = analog_input(static_cast<int32_t>(128.0f * co.f[0]), 0.0f, 96.0f, 110.0f, 1.0f);
        co.i[0] = co.i[0] + 1;
    }
    if (co.b[0]) {
        ja(self, Ja().group(AnimId::jakb_board_flip_forward).seek().frame_num(aframe(15.0f)).no_eval());
        while (!done(self)) {
            if (jump_hit_ground_stuck_p(self)) {
                go(self, StateId::target_board_hit_ground);
            }
            CO_SUSPEND(co);
            ja(self, Ja().seek());
        }
    } else {
        ja(self, Ja().group(AnimId::jakb_board_flip_backward).seek().frame_num(aframe(10.0f)).no_eval());
        while (!done(self)) {
            if (jump_hit_ground_stuck_p(self)) {
                go(self, StateId::target_board_hit_ground);
            }
            CO_SUSPEND(co);
            ja(self, Ja().seek());
        }
    }
    go(self, StateId::target_board_falling);
    CO_END(co);
}

// ---- target-board-hold (arg0, arg1 heights, arg2 symbol) ----
void hold_enter(Target& self, const StateArgs& args) {
    ControlInfo& c = self.control;
    runtime(self).word04.set(args.sym);
    Vec up;
    vector_y_quaternion(up, c.dir_targ);
    forward_up_nopitch_to_quaternion(c.dir_targ, c.transv, up);
    c.dynam->gravity_length = 245760.0f;
    set_time(self, self.state_time);
    set_time(self, self.board->unknown_time_frame04);
    if (args.sym == Symbol("halfpipe")) {
        self.focus_status |= focus::halfpipe;
        target_board_halfpipe_trans(self);
    } else {
        init_var_jump(self, args.f0, args.f1, true, false, c.transv, 1.0f);
        c.status &= ~(cstat::on_surface | cstat::on_ground | cstat::touch_surface);
        c.unknown_float36 = std::fmax(0.0f, std::fmin(1.0f, 0.00004359654f * (-11468.8f + c.ctrl_xz_vel)));
    }
    set_mods(self, &board_flip_mods);
    c.dynam->gravity_length = 147456.0f;
}

void hold_exit(Target& self) {
    self.board->unknown_float01 = 0.0f;
    set_time(self, self.board->unknown_time_frame05);
    sound_fade(self, runtime(self).unknown_sound_id01, 24);
    if (group(self) == AnimId::jakb_board_noseflip) {
        sound_play(self, "board-boots");
    }
    halfpipe_exit(self);
}

void hold_trans(Target& self) {
    ControlInfo& c = self.control;
    target_board_spin_check(self);
    if (runtime(self).word04.symbol() == Symbol("halfpipe")) {
        target_board_halfpipe_trans(self);
    } else {
        // og:preserve-this abutton indexing
        c.unknown_float36 = std::fmax(c.unknown_float36,
                                      0.003921569f * static_cast<float>(target_cpad_abutton_x(self)));
        mod_var_jump(self, true, false, target_cpad_hold(self, boardpad::x), c.transv);
    }
    self.board->slow_transv = c.transv;
}

// co.f[0] f30-0.
void hold_code(Target& self, Co& co, const StateArgs&) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    CO_BEGIN(co);
    {
        const float z = -0.0078125f * (-128.0f + static_cast<float>(target_cpad_lefty(self)));
        co.f[0] = analog_input(static_cast<int32_t>(128.0f * z), 0.0f, 96.0f, 110.0f, 1.0f);
    }
    while (!(!target_cpad_hold(self, boardpad::l1) || jump_hit_ground_stuck_p(self) ||
             target_time_to_ground(self) < seconds(0.3))) {
        if (co.f[0] < 0.0f) {
            if (group(self) == AnimId::jakb_board_method_loop) {
                ja(self, Ja().loop());
            } else {
                add_to_trick_list(self, BoardTricks::board_method, 500.0f);
                rt.unknown_sound_id01 = sound_play(self, "board-method");
                push(self, 1, seconds(0.08));
                ja(self, Ja().group(AnimId::jakb_board_method).seek().frame_num(0.0f).no_eval());
                while (!done(self)) {
                    CO_SUSPEND(co);
                    ja(self, Ja().seek());
                }
                ja(self, Ja().group(AnimId::jakb_board_method_loop).min());
            }
        } else if (group(self) == AnimId::jakb_board_nosegrab_loop) {
            ja(self, Ja().loop());
        } else if (40960.0f < target_height_above_ground(self)) {
            add_to_trick_list(self, BoardTricks::board_nosegrab, 500.0f);
            rt.unknown_sound_id01 = sound_play(self, "board-nosegrab");
            push(self, 1, seconds(0.08));
            ja(self, Ja().group(AnimId::jakb_board_nosegrab).seek().frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek());
            }
            ja(self, Ja().group(AnimId::jakb_board_nosegrab_loop).min());
        } else {
            add_to_trick_list(self, BoardTricks::board_noseflip, 500.0f);
            rt.unknown_sound_id01 = sound_play(self, "board-noseflip");
            push(self, 1, seconds(0.08));
            ja(self, Ja().group(AnimId::jakb_board_noseflip).seek(aframe(20.0f)).frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek(aframe(20.0f)));
            }
            goto cfg_36;
        }
        CO_SUSPEND(co);
    }
cfg_36:
    if (group(self) == AnimId::jakb_board_method_loop) {
        push(self, 1, seconds(0.08));
        ja(self, Ja().group(AnimId::jakb_board_method_end).seek_max(1.5f).frame_num(0.0f).no_eval());
        while (!done(self)) {
            CO_SUSPEND(co);
            ja(self, Ja().seek_max(1.5f));
        }
    } else if (group(self) == AnimId::jakb_board_nosegrab_loop) {
        push(self, 1, seconds(0.08));
        ja(self, Ja().group(AnimId::jakb_board_nosegrab_end).seek_max(1.5f).frame_num(0.0f).no_eval());
        while (!done(self)) {
            CO_SUSPEND(co);
            ja(self, Ja().seek_max(1.5f));
        }
    }
    {
        Vec v1;
        float f0 = vector_dot(gravity_normal(self), c.transv);
        vector_float_mul(v1, gravity_normal(self), f0);
        vector_sub(v1, c.transv, v1);
        const float f1 = vector_length(v1);
        const float f2 = f1;
        if (0.0f < f0) {
            f0 = 0.0f;
        }
        Vec t;
        vector_float_mul(t, gravity_normal(self), f0);
        vector_float_mul(v1, v1, goal_div(f1, f2));
        vector_add(c.transv, t, v1);
    }
    go(self, StateId::target_board_falling);
    CO_END(co);
}

// ---- target-board-trickx ----
void trickx_exit(Target& self) {
    io(self).sidekick_matrix(Symbol("board"));
    sound_play(self, "board-boots");
    hold_exit(self);
}

void trickx_trans(Target& self) {
    ControlInfo& c = self.control;
    target_board_spin_check(self);
    if (runtime(self).word04.symbol() != Symbol("halfpipe")) {
        // og:preserve-this abutton indexing
        c.unknown_float36 = std::fmax(c.unknown_float36,
                                      0.003921569f * static_cast<float>(target_cpad_abutton_x(self)));
        mod_var_jump(self, true, false, target_cpad_hold(self, boardpad::x), c.transv);
    }
    if (jump_hit_ground_stuck_p(self)) {
        set_time(self, self.board->jump_land_time);
        go(self, StateId::target_board_hit_ground);
    }
    self.board->slow_transv = c.transv;
}

// co.b[0] gp-0, co.f[0] s5-0's z, co.f[1] f0-9, co.i[0] s4-0.
void trickx_code(Target& self, Co& co, const StateArgs&) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    CO_BEGIN(co);
    co.b[0] = b.unknown_float00 >= 0.0f;
    co.f[0] = -0.0078125f * (-128.0f + static_cast<float>(target_cpad_lefty(self)));
    co.f[1] = analog_input(static_cast<int32_t>(128.0f * co.f[0]), 0.0f, 96.0f, 110.0f, 1.0f);
    co.i[0] = 0;
    while (!((!target_cpad_hold(self, boardpad::r1) ||
              (co.f[1] == 0.0f || jump_hit_ground_stuck_p(self) || target_time_to_ground(self) < seconds(0.5))) &&
             co.i[0] != 0)) {
        if (co.b[0]) {
            add_to_trick_list(self, BoardTricks::board_kickflip, 500.0f);
            rt.unknown_sound_id01 = sound_play(self, "board-kickflip");
            push(self, 1, seconds(0.07));
            ja(self, Ja().group(AnimId::jakb_board_kickflip_a).seek().frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek());
            }
            io(self).sidekick_matrix(Symbol());
            ja(self, Ja().group(AnimId::jakb_board_kickflip_b).seek_max(1.05f).frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek_max(1.05f));
            }
            io(self).sidekick_matrix(Symbol("board"));
            ja(self, Ja().group(AnimId::jakb_board_kickflip_c).seek().frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek());
            }
        } else {
            add_to_trick_list(self, BoardTricks::board_board_spin, 500.0f);
            rt.unknown_sound_id01 = sound_play(self, "board-boardspin");
            push(self, 1, seconds(0.05));
            ja(self, Ja().group(AnimId::jakb_board_spin).seek_max(0.95f).frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek_max(0.95f));
            }
        }
        b.trickx_count = b.trickx_count + 1;
        co.f[1] = analog_input(static_cast<int32_t>(128.0f * co.f[0]), 0.0f, 96.0f, 110.0f, 1.0f);
        co.i[0] = co.i[0] + 1;
    }
    go(self, StateId::target_board_falling);
    CO_END(co);
}

// ---- target-board-hit-ground ----
void hit_ground_enter(Target& self, const StateArgs&) {
    self.focus_status &= ~focus::halfpipe;
    if (self.control.ground_pat.mode() == kPatHalfpipe) {
        go(self, StateId::target_board_turn_to, turn_to_args(self.control.transv, seconds(0.5)));
    }
}

// The x press of hit-ground and turn-to.
void jump_on_x(Target& self) {
    // og:preserve-this High FPS Fix (recently-pressed?), as OpenGOAL.
    if (target_recently_pressed(self, boardpad::x) && can_jump_p(self, Symbol("board"))) {
        flush_trick_list(self);
        go(self, StateId::target_board_jump,
           jump_args(kTargetBoardBank.jump_height_min, kTargetBoardBank.jump_height_max));
    }
}

void hit_ground_trans(Target& self) {
    jump_on_x(self);
    if (target_cpad_hold(self, boardpad::l1) && !(self.state_flags & sflags::prevent_duck)) {
        go(self, StateId::target_board_duck_stance);
    }
    target_board_smack_surface_p(self);
}

void hit_ground_code(Target& self, Co& co, const StateArgs&) {
    CO_BEGIN(co);
    self.control.mod_surface = &board_walk_mods;
    co.i[0] = static_cast<int>(group(self));
    {
        if (co.i[0] == static_cast<int>(AnimId::jakb_board_method_end) ||
            co.i[0] == static_cast<int>(AnimId::jakb_board_nosegrab_end)) {
            ja(self, Ja().seek_max(1.5f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja_eval(jc(self), anim_clock(self));
            }
        } else if (co.i[0] == static_cast<int>(AnimId::jakb_board_method_loop)) {
            push(self, 1, seconds(0.08));
            ja(self, Ja().group(AnimId::jakb_board_method_end).seek_max(1.5f).frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek_max(1.5f));
            }
        } else if (co.i[0] == static_cast<int>(AnimId::jakb_board_nosegrab_loop)) {
            push(self, 1, seconds(0.08));
            ja(self, Ja().group(AnimId::jakb_board_nosegrab_end).seek_max(1.5f).frame_num(0.0f).no_eval());
            while (!done(self)) {
                CO_SUSPEND(co);
                ja(self, Ja().seek_max(1.5f));
            }
        }
    }
    go(self, StateId::target_board_stance);
    CO_END(co);
}

// ---- target-board-turn-to (arg0 StateArgs::vec, arg1 StateArgs::time) ----
void turn_to_enter(Target& self, const StateArgs& args) {
    ControlInfo& c = self.control;
    self.focus_status &= ~focus::halfpipe;
    if (c.dynam->gravity_length != 245760.0f) {
        Vec v1;
        float f0 = vector_dot(gravity_normal(self), c.transv);
        vector_float_mul(v1, gravity_normal(self), f0);
        vector_sub(v1, c.transv, v1);
        const float f1 = vector_length(v1);
        const float f2 = f1;
        if (f0 < 0.0f) {
            f0 = 5.0f * f0;
        }
        Vec t;
        vector_float_mul(t, gravity_normal(self), f0);
        vector_float_mul(v1, v1, goal_div(f1, f2));
        vector_add(c.transv, t, v1);
    }
    set_time(self, self.state_time);
    set_mods(self, &board_turn_to_mods);
    c.sliding_start_time = args.time;
    vector_normalize_copy(c.unknown_vector38, args.vec, 1.0f);
    Vec up;
    vector_y_quaternion(up, c.dir_targ);
    forward_up_nopitch_to_quaternion(c.dir_targ, c.unknown_vector38, up);
    c.turn_lockout_end_time = current_time(self) + args.time;
}

void turn_to_trans(Target& self) {
    BoardInfo& b = *self.board;
    jump_on_x(self);
    if (time_elapsed(self, self.state_time, self.control.sliding_start_time)) {
        go(self, StateId::target_board_stance);
    }
    target_board_anim_trans(self);
    if (board_on_ground_p(self)) {
        const Symbol gp = b.turn_anim_tilt_p;
        target_board_ground_check(self);
        b.turn_anim_tilt_p = gp;
    }
    b.turn_anim_targ = 10.0f * b.turn_anim_targ;
}

// ---- target-board-ride-edge (arg0 symbol: #t when it came from the air) ----
int ride_seconds(Target& self) {
    return static_cast<int>((current_time(self) - self.board->ride_start_time) / 300);
}

Symbol ride_edge_event(Target& self, const EventMessage& e) {
    if (e.message == Symbol("edge-grab") || e.message == Symbol("push-transv") ||
        e.message == Symbol("push-trans")) {
        return Symbol();
    }
    if (e.message == Symbol("end-mode")) {
        if (e.truthy[0]) {
            const int v1 = ride_seconds(self);
            if (v1 > 0) {
                add_to_trick_list(self, BoardTricks::board_rail, 100.0f * static_cast<float>(v1));
            }
        }
        go(self, StateId::target_board_turn_to, turn_to_args(self.control.transv, seconds(0.2)));
        return Symbol();
    }
    return target_board_handler(self, e);
}

void ride_edge_enter(Target& self, const StateArgs& args) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    set_time(self, self.state_time);
    self.focus_status |= focus::rail;
    root_prim_action(self) |= caction::can_ride;
    set_mods(self, &board_ride_mods);
    {
        const Vec& v1 = c.transv_ctrl;
        b.ride_speed = std::sqrt(v1.x * v1.x + v1.z * v1.z);
    }
    b.ride_lean_targ = 0.0f;
    b.ride_lean = 0.0f;
    b.ride_leanv = 0.0f;
    b.ride_tilt_targ = 0.0f;
    b.ride_tilt = 0.0f;
    b.ride_tiltv = 0.0f;
    rt.neck_flex_blend = 0.0f;
    b.spin_control = 0.0f;
    b.flip_control = 0.0f;
    b.flip_count = 0;
    b.unknown_float00 = 0.0f;
    b.unknown_float01 = 0.0f;
    rt.word04.set(true);
    target_board_compute_edge(self);
    const float f0 = vector_dot(c.c_R_w.v[2], c.edge_grab_edge_dir);
    if (0.7f < f0) {
        b.ride_mode = 0;
        b.ride_rot = 0.0f;
    } else if (f0 < -0.7f) {
        b.ride_mode = 1;
        b.ride_rot = 32768.0f;
    } else {
        const float f0_21 = vector_dot(c.c_R_w.v[0], c.edge_grab_edge_dir);
        if (0.7f < f0_21) {
            b.ride_mode = 2;
            b.ride_rot = -16384.0f;
        } else {
            b.ride_mode = 3;
            b.ride_rot = 16384.0f;
        }
    }
    float f24 = 0.0f;
    if (args.sym) {
        f24 = 250.0f + f24;
    }
    if (b.ride_rot_old != -1.0f && b.ride_rot != b.ride_rot_old) {
        f24 = 500.0f + f24;
    }
    add_to_trick_list(self, BoardTricks::board_rail_jump, f24);
}

void ride_edge_exit(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    c.status &= ~cstat::probe_hit;
    sound_fade(self, rt.ride_sound_id, 24);
    stance_exit(self);
    rt.neck_flex_blend = 1.0f;
    self.focus_status &= ~focus::rail;
    root_prim_action(self) &= ~caction::can_ride;
    b.ride_rot_old = b.ride_rot;
    Vec gp = c.trans;
    {
        CollideQuery s5;
        if (find_ground(self, s5, root_prim_collide_with(self) & ~cspec::water, 8192.0f, 81920.0f, 1024.0f)) {
            c.gspot_pat_surfce = s5.best_pat;
        }
    }
    if (c.trans.y < c.gspot_pos.y) {
        gp = c.gspot_pos;
    }
    Vec p;
    vector_add(p, gp, Vec{0.0f, 204.8f, 0.0f, 1.0f});
    move_to_point(self, p);
}

void ride_edge_trans(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    // og:preserve-this High FPS Fix (recently-pressed?), as OpenGOAL.
    if (target_recently_pressed(self, boardpad::x) && can_jump_p(self, Symbol("board"))) {
        const int v1 = ride_seconds(self);
        if (v1 > 0) {
            add_to_trick_list(self, BoardTricks::board_rail, 100.0f * static_cast<float>(v1));
        }
        c.trans.y = c.trans.y + 1228.8f;
        go(self, StateId::target_board_jump,
           jump_args(kTargetBoardBank.jump_height_min, kTargetBoardBank.jump_height_max, Symbol("surface"),
                     &board_ride_jump_mods));
    }
    if (target_cpad_pressed(self, boardpad::circle | boardpad::square)) {
        rt.word04.set(false);
    }
    if (rt.word04.truthy()) {
        set_time(self, b.ride_button_time);
    }
    b.turn_anim_targ = b.ride_lean * -kTargetBoardBank.turn_frames;
    if (b.ride_speed == 0.0f) {
        sound_fade(self, rt.ride_sound_id, 24);
    } else {
        const float f30 = lerp_scale(0.7f, 1.0f, std::fabs(b.ride_lean), 0.0f, 1.0f) *
                          lerp_scale(0.8f, 1.0f, std::fabs(b.ride_speed), 0.0f, c.mod_surface->transv_max / 2.0f);
        const float f0 = lerp_scale(0.0f, 0.05f, std::fabs(b.ride_lean), 0.0f, 1.0f) +
                         lerp_scale(-0.05f, 0.0f, std::fabs(b.ride_speed), 0.0f, 0.3f * c.mod_surface->transv_max);
        if (self.effects) {
            self.effects->sound_loop("board-rail", rt.ride_sound_id,
                                     static_cast<float>(static_cast<int32_t>(1024.0f * f30)) / 1024.0f,
                                     static_cast<float>(static_cast<int32_t>(1524.0f * f0)));
        }
    }
    if (4096.0f < b.ride_speed) {
        Vec gp;
        vector_add_float_mul(gp, c.trans, b.ride_dir, -4096.0f);
        // (set! (-> *part-id-table* 431 init-specs 1 initial-valuef) (lerp-scale ...)): the sparks'
        // count by speed.
        particles(self, "part-431", gp,
                  lerp_scale(0.1f, 2.0f, std::fabs(b.ride_speed), 0.0f, 0.3f * c.mod_surface->transv_max));
    }
}

void ride_edge_code(Target& self, Co& co, const StateArgs&) {
    CO_BEGIN(co);
    {
        const uint32_t v1 = self.board->ride_mode;
        const AnimId gp = v1 == 0   ? AnimId::jakb_board_ride_turn_front
                          : v1 == 1 ? AnimId::jakb_board_ride_turn_back
                          : v1 == 2 ? AnimId::jakb_board_ride_turn_left
                          : v1 == 3 ? AnimId::jakb_board_ride_turn_right
                                    : AnimId::none;
        if (group(self) != gp) {
            push(self, 1, seconds(0.1));
            jc(self).root_channel(0).frame_group = gp;
        }
    }
    while (true) {
        ja(self, Ja().num_func(NumFunc::identity)
                     .frame_num(aframe(std::fmax(
                         -5.0f, std::fmin(5.0f, 3.0f * self.board->ride_lean +
                                                    0.5f * sin(145.63556f * static_cast<float>(
                                                                                current_time(self) - self.state_time)))))));
        CO_SUSPEND(co);
    }
    CO_END(co);
}

void ride_edge_post(Target& self) { target_board_ride_post(self); }

// ---- target-board-grenade (arg0 handle) ----
Symbol grenade_event(Target& self, const EventMessage& e) {
    if (e.message == Symbol("target")) {
        // (handle->process (-> self control unknown-handle02)): the process, truthy when alive.
        return runtime(self).unknown_handle02 ? kTrue : Symbol();
    }
    return target_board_handler(self, e);
}

void grenade_enter(Target& self, const StateArgs& args) {
    ControlInfo& c = self.control;
    self.focus_status |= focus::halfpipe;
    runtime(self).unknown_handle02 = args.handle;
    set_time(self, self.state_time);
    root_prim_action(self) |= caction::no_normal_reset;
    set_mods(self, &board_halfpipe_mods);
    Vec v1;
    const float f0 = vector_dot(gravity_normal(self), c.transv);
    vector_float_mul(v1, gravity_normal(self), f0);
    vector_sub(v1, c.transv, v1);
    float f1 = vector_length(v1);
    const float f2 = f1;
    if (f1 != 0.0f && 2.0f < goal_div(f0, f1)) {
        f1 = 0.0f;
    }
    Vec t;
    vector_float_mul(t, gravity_normal(self), f0);
    vector_float_mul(v1, v1, goal_div(f1, f2));
    vector_add(c.transv, t, v1);
}

void grenade_exit(Target& self) {
    io(self).send_exit(runtime(self).unknown_handle02);
    halfpipe_exit(self);
}

void grenade_trans(Target& self) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    set_time(self, b.halfpipe_time);
    if (c.gspot_pat_surfce.mode() == kPatHalfpipe) {
        set_time(self, b.halfpipe_gspot_time);
    }
    if (!time_elapsed(self, self.state_time, seconds(1))) {
        vector_add_float_mul(c.transv, c.transv, gravity_normal(self), 122880.0f * spf(self));
    }
    if (jump_hit_ground_stuck_p(self)) {
        vector_float_mul(c.transv, c.transv, 1.5f);
        go(self, StateId::target_board_turn_to, turn_to_args(c.transv, seconds(0.5)));
    }
    if (time_elapsed(self, b.halfpipe_gspot_time, seconds(0.5))) {
        c.transv.x = c.transv.x + 20480.0f * c.edge_grab_across_edge_dir.x;
        c.transv.z = c.transv.z + 20480.0f * c.edge_grab_across_edge_dir.z;
        go(self, StateId::target_board_stance);
    }
}

void grenade_code(Target& self, Co& co, const StateArgs& args) {
    CO_BEGIN(co);
    push(self, 1, seconds(0.05));
    ja(self, Ja().group(AnimId::jakb_board_grenade).seek(aframe(19.0f)).frame_num(0.0f).no_eval());
    while (!done(self)) {
        CO_SUSPEND(co);
        ja(self, Ja().seek(aframe(19.0f)));
    }
    // The projectile-init-by-other-params (charge 0.1, timeout 4 s, from the extra joint, the next
    // attack id) and spawn-projectile grenade: the adapter's. The attack id is taken here as GOAL does.
    io(self).next_attack_id();
    io(self).spawn_grenade(self, args.handle);
    ja(self, Ja().seek().no_eval());
    while (!done(self)) {
        CO_SUSPEND(co);
        ja_eval(jc(self), anim_clock(self));
    }
    sleep_code(self);
    CO_SUSPEND(co);
    CO_END(co);
}

// ---- target-board-get-on ----
Symbol get_on_event(Target& self, const EventMessage& e) {
    if (e.message == Symbol("attack") || e.message == Symbol("attack-or-shove") ||
        e.message == Symbol("attack-invinc")) {
        return target_attacked(self, e.message, *e.attack, e.from, StateId::target_hit);
    }
    return io(self).generic_event_handler(self, e);
}

// The let* get-on and get-off repeat: the speed along gravity becomes `vertical` (from the old one),
// the rest stays.
template <class F>
void set_vertical(Target& self, F vertical) {
    ControlInfo& c = self.control;
    Vec v1;
    const float f0 = vector_dot(gravity_normal(self), c.transv);
    vector_float_mul(v1, gravity_normal(self), f0);
    vector_sub(v1, c.transv, v1);
    const float f0_3 = vector_length(v1);
    const float f1 = f0_3;
    const float f2 = vertical();
    Vec t;
    vector_float_mul(t, gravity_normal(self), f2);
    vector_float_mul(v1, v1, goal_div(f0_3, f1));
    vector_add(c.transv, t, v1);
}

float up_speed_clamped(Target& self) {
    return std::fmax(0.0f, std::fmin(40960.0f, vector_dot(gravity_normal(self), self.control.transv)));
}

void get_on_enter(Target& self, const StateArgs&) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    self.board->shock_offsetv = 0.0f;
    set_time(self, self.state_time);
    c.status &= ~(cstat::on_surface | cstat::on_ground | cstat::touch_surface);
    set_mods(self, &board_jump_mods);
    const TimeFrame a0 = target_time_to_ground(self);
    if (waterflag::touch_water & io(self).water_flags()) {
        set_vertical(self, [&] { return up_speed_clamped(self) + 0.33f * c.dynam->gravity_length; });
        rt.word04.set(true);
    } else if (a0 < seconds(0.25)) {
        set_vertical(self, [&] {
            return up_speed_clamped(self) +
                   0.0016666667f * static_cast<float>(seconds(0.66) - a0) * c.dynam->gravity_length;
        });
        rt.word04.set(true);
    } else {
        set_vertical(self, [&] { return up_speed_clamped(self); });
        rt.word04.set(false);
    }
    {
        Vec v1;
        const float f0 = vector_dot(gravity_normal(self), c.transv);
        vector_float_mul(v1, gravity_normal(self), f0);
        vector_sub(v1, c.transv, v1);
        float f2 = vector_length(v1);
        const float f1 = f2;
        if (81920.0f < f2) {
            f2 = 81920.0f;
        }
        Vec t;
        vector_float_mul(t, gravity_normal(self), f0);
        vector_float_mul(v1, v1, goal_div(f2, f1));
        vector_add(c.transv, t, v1);
    }
    self.board->slow_transv = c.transv;
}

// co.f[0] f30-1.
void get_on_code(Target& self, Co& co, const StateArgs&) {
    BoardRuntime& rt = runtime(self);
    CO_BEGIN(co);
    board_process_event(self, Symbol("open"));
    push(self, 1, rt.word04.truthy() ? 9 : 60);
    co.f[0] = rt.word04.truthy()
                  ? 1.25f
                  : std::fmax(1.0f, std::fmin(2.0f, goal_div(249.99f, static_cast<float>(target_time_to_ground(self)))));
    ja(self, Ja().group(AnimId::jakb_board_get_on).seek_max(co.f[0]).frame_num(0.0f).no_eval());
    while (!done(self)) {
        if (self.control.status & cstat::on_surface) {
            self.board->turn_anim_duck_vel = 0.0f;
            goto cfg_19;
        }
        CO_SUSPEND(co);
        ja(self, Ja().seek_max(co.f[0]));
    }
    self.board->turn_anim_duck_vel = 15.0f;
    while (!jump_hit_ground_stuck_p(self)) {
        CO_SUSPEND(co);
    }
cfg_19:
    io(self).sidekick_matrix(Symbol("board"));
    self.control.status |= cstat::on_surface;
    go(self, StateId::target_board_hit_ground);
    CO_END(co);
}

// ---- target-board-pegasus (arg0 handle) ----
Symbol pegasus_event(Target& self, const EventMessage& e) {
    if (e.message == Symbol("end-mode")) {
        go(self, StateId::target_jump, jump_args(16384.0f, 16384.0f));
        return Symbol();
    }
    return io(self).generic_event_handler(self, e);
}

void pegasus_enter(Target& self, const StateArgs&) {
    self.focus_status &= ~focus::halfpipe;
    self.board->shock_offsetv = 0.0f;
    set_time(self, self.state_time);
    self.control.status &= ~(cstat::on_surface | cstat::on_ground | cstat::touch_surface);
}

// co.f[0] f30-0, co.b[0] s5-0.
void pegasus_code(Target& self, Co& co, const StateArgs& args) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    CO_BEGIN(co);
    co.f[0] = 0.0f;
    co.b[0] = true;
    push(self, 1, seconds(0.05));
    c.mod_surface = &empty_mods;
    rt.neck_flex_blend = 0.0f;
    c.unknown_vector37 = c.trans;
    c.unknown_vector39 = Vec{c.quat.x, c.quat.y, c.quat.z, c.quat.w};
    ja(self, Ja().group(AnimId::jakb_board_attack_pegasus).seek().frame_num(0.0f));
    while (true) {
        {
            // (get-trans s4-1 0), (get-quat s4-1 0) and its (ja-aframe-num 0): the adapter's.
            Vec otrans;
            Quat oquat;
            float oaframe = 0.0f;
            const bool s4 = io(self).grenade_target_trans(args.handle, otrans, oquat, oaframe);
            if (s4) {
                c.unknown_vector38 = otrans;
                c.unknown_vector38.y = c.unknown_vector38.y + 4096.0f;
                c.unknown_vector40 = Vec{oquat.x, oquat.y, oquat.z, oquat.w};
            }
            {
                const float f28 = sin(lerp_scale(0.0f, 16384.0f, ja_aframe_num(jc(self), 0), 0.0f, 30.0f));
                const float f26 = f28;
                vector_lerp(c.trans, c.unknown_vector37, c.unknown_vector38, f28);
                c.trans.y = lerp(c.unknown_vector37.y, c.unknown_vector38.y, f26);
                const Quat q39{c.unknown_vector39.x, c.unknown_vector39.y, c.unknown_vector39.z, c.unknown_vector39.w};
                const Quat q40{c.unknown_vector40.x, c.unknown_vector40.y, c.unknown_vector40.z, c.unknown_vector40.w};
                quaternion_slerp(c.quat_for_control, q39, q40, f28);
            }
            rot_to_dir_targ(self);
            if (s4) {
                co.f[0] = oaframe;
                ja(self, Ja().num_func(NumFunc::identity).frame_num(aframe(co.f[0])));
            }
        }
        if (co.b[0] && co.f[0] >= 17.0f) {
            board_process_event(self, Symbol("close"));
            self.focus_status &= ~focus::board;
            co.b[0] = false;
        }
        CO_SUSPEND(co);
    }
    CO_END(co);
}

void pegasus_post(Target& self) {
    ControlInfo& c = self.control;
    c.bend_speed = 0.0f;
    c.bend_target = 0.0f;
    vector_seek(c.draw_offset, Vec{0.0f, 0.0f, 0.0f, 1.0f}, 16384.0f * spf(self));
    target_no_move_post(self);
    target_board_effect(self);
}

// ---- target-board-get-off (arg0 object: StateArgs::handle, arg1 symbol) ----
// The (new 'static 'surface :name 'jump ...) of get-off's enter: one object, as GOAL's static.
Surface& get_off_surface() {
    static Surface s = [] {
        Surface r{};
        r.name = Symbol("jump");
        r.turnv = 131072.0f;
        r.turnvv = 18204.445f;
        r.tiltv = 131072.0f;
        r.tiltvv = 262144.0f;
        r.transv_max = 65536.0f;
        r.target_speed = 65536.0f;
        r.seek0 = 0.3f;
        r.seek90 = 0.3f;
        r.seek180 = 0.3f;
        r.fric = 0.05f;
        r.nonlin_fric_dist = 1.0f;
        r.slip_factor = 1.0f;
        r.slide_factor = 1.0f;
        r.slope_up_factor = 1.0f;
        r.slope_down_factor = 1.0f;
        r.slope_slip_angle = 1.0f;
        r.impact_fric = 1.0f;
        r.bend_factor = 1.0f;
        r.bend_speed = 1.0f;
        r.alignv = 1.0f;
        r.slope_up_traction = 1.0f;
        r.align_speed = 1.0f;
        r.turnvf = 30.0f;
        r.turnvvf = 30.0f;
        r.tiltvf = 30.0f;
        r.tiltvvf = 15.0f;
        r.mode = Symbol("air");
        r.flags = sflag::check_edge | sflag::air;
        return r;
    }();
    return s;
}

void get_off_enter(Target& self, const StateArgs& args) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    if (args.sym == Symbol("pegasus")) {
        StateArgs a;
        a.handle = args.handle;
        go(self, StateId::target_board_pegasus, a);
    }
    self.focus_status &= ~focus::halfpipe;
    self.board->shock_offsetv = 0.0f;
    set_time(self, self.state_time);
    c.status &= ~(cstat::on_surface | cstat::on_ground | cstat::touch_surface);
    c.mod_surface = &get_off_surface();
    c.dynam->gravity_length = c.standard_dynamics->gravity_length;
    const TimeFrame a0 = target_time_to_ground(self);
    if (a0 < seconds(0.207)) {
        set_vertical(self, [&] {
            return up_speed_clamped(self) +
                   0.0016666667f * static_cast<float>(seconds(0.66) - a0) * c.dynam->gravity_length;
        });
        rt.word04.set(true);
    } else {
        set_vertical(self, [&] { return up_speed_clamped(self); });
        rt.word04.set(false);
    }
    self.board->slow_transv = c.transv;
}

// co.f[0] f30-0.
void get_off_code(Target& self, Co& co, const StateArgs& args) {
    BoardRuntime& rt = runtime(self);
    CO_BEGIN(co);
    co.f[0] = 1.5151515f;
    if (args.sym == Symbol("hit")) {
        push(self, 1, seconds(0.1));
        ja(self, Ja().group(AnimId::jakb_board_hit_get_off).seek(aframe(24.0f), 2.5f).frame_num(0.0f).no_eval());
        while (!done(self)) {
            CO_SUSPEND(co);
            ja(self, Ja().seek(aframe(24.0f), 2.5f));
        }
    } else if (rt.word04.truthy()) {
        push(self, 1, seconds(0.1));
        ja(self, Ja().group(AnimId::jakb_board_get_off_pre).seek_max(co.f[0]).frame_num(0.0f).no_eval());
        while (!done(self)) {
            CO_SUSPEND(co);
            ja(self, Ja().seek_max(co.f[0]));
        }
        io(self).sidekick_matrix(Symbol("normal"));
        ja(self, Ja().group(AnimId::jakb_board_get_off).seek(aframe(24.0f), co.f[0]).frame_num(0.0f).no_eval());
        while (!done(self)) {
            if (jump_hit_ground_stuck_p(self) && ja_aframe_num(jc(self), 0) >= 14.0f) {
                go(self, StateId::target_falling);
            }
            CO_SUSPEND(co);
            ja(self, Ja().seek(aframe(24.0f), co.f[0]));
        }
    } else {
        co.f[0] = std::fmax(0.5f, std::fmin(2.0f, goal_div(150.0f, static_cast<float>(target_time_to_ground(self)))));
        push(self, 1, seconds(0.1));
        io(self).sidekick_matrix(Symbol("normal"));
        ja(self, Ja().group(AnimId::jakb_board_get_off).seek(aframe(24.0f), co.f[0]).frame_num(aframe(11.0f)).no_eval());
        while (!done(self)) {
            if (jump_hit_ground_stuck_p(self) && ja_aframe_num(jc(self), 0) >= 14.0f) {
                go(self, StateId::target_falling);
            }
            CO_SUSPEND(co);
            ja(self, Ja().seek(aframe(24.0f), co.f[0]));
        }
    }
    board_process_event(self, Symbol("close"));
    self.focus_status &= ~focus::board;
    ja(self, Ja().seek_max(co.f[0]).no_eval());
    while (!done(self)) {
        if (jump_hit_ground_stuck_p(self)) {
            go(self, StateId::target_falling);
        }
        CO_SUSPEND(co);
        ja_eval(jc(self), anim_clock(self));
    }
    self.board->turn_anim_duck_vel = 15.0f;
    if (board_on_ground_p(self) && !((waterflag::jump_out & io(self).water_flags()) ||
                                     (self.control.status & cstat::on_water))) {
        // (go target-hit-ground (the-as symbol a0-35)): a0-35 holds 0 here (the on-water bits it
        // last read), which GOAL passes as a symbol that is not #f.
        StateArgs a;
        a.sym = Symbol("0");
        go(self, StateId::target_hit_ground, a);
    } else {
        StateArgs a;
        a.sym = args.sym == Symbol("hit") ? Symbol("hit") : Symbol();
        go(self, StateId::target_falling, a);
    }
    CO_END(co);
}

void get_off_post(Target& self) {
    ControlInfo& c = self.control;
    c.bend_speed = 0.0f;
    c.bend_target = 0.0f;
    vector_seek(c.draw_offset, Vec{0.0f, 0.0f, 0.0f, 1.0f}, 16384.0f * spf(self));
    target_post(self);
    target_board_effect(self);
}

// ---- target-board-grab ----
Symbol grab_event(Target& self, const EventMessage& e) {
    if (e.message == Symbol("query") && e.sym[0] == Symbol("mode")) {
        return Symbol(state_name(runtime(self).state));
    }
    if (e.message == Symbol("end-mode")) {
        go(self, StateId::target_board_stance);
        return Symbol();
    }
    if (e.message == Symbol("clone-anim")) {
        StateArgs a;
        a.handle = e.from;
        go(self, StateId::target_board_clone_anim, a);
        return Symbol();
    }
    if (e.message == Symbol("change-mode")) {
        if (e.sym[0] == Symbol("normal")) {
            StateArgs a;
            a.sym = Symbol("stance");
            go(self, StateId::target_grab, a);
        }
        return Symbol();
    }
    return io(self).generic_event_handler(self, e);
}

void grab_enter(Target& self, const StateArgs&) {
    self.control.mod_surface = &grab_mods;
    runtime(self).neck_flex_blend = 0.0f;
    self.state_flags |= sflags::sf2;
    self.focus_status |= focus::grabbed;
    self.focus_status &= ~focus::halfpipe;
    self.board->stick_off = kTrue;
}

void grab_exit(Target& self) {
    self.board->stick_off = Symbol();
    self.state_flags &= ~sflags::sf2;
    self.focus_status &= ~focus::grabbed;
    io(self).water_clear_jump_out();
    target_board_exit(self);
}

void grab_code(Target& self, Co& co, const StateArgs&) {
    CO_BEGIN(co);
    if (group(self) != AnimId::jakb_board_stance) {
        push(self, 1, seconds(0.1));
        ja(self, Ja().group(AnimId::jakb_board_stance).identity(aframe(0.0f)));
    }
    while (true) {
        set_forward_vel(self, 0.0f);
        CO_SUSPEND(co);
    }
    CO_END(co);
}

// ---- target-board-clone-anim (arg0 handle) ----
Symbol clone_anim_event(Target& self, const EventMessage& e) {
    if (e.message == Symbol("trans") && e.sym[0] == Symbol("restore")) {
        runtime(self).word04.set(false);
    }
    return grab_event(self, e);
}

void clone_anim_enter(Target& self, const StateArgs& args) { io(self).clone_anim_enter(self, args.handle); }

void clone_anim_exit(Target& self) {
    ControlInfo& c = self.control;
    c.draw_offset.y = runtime(self).word04.as_float();
    c.cspace_offset.y = c.draw_offset.y;
    io(self).sidekick_matrix(Symbol("board"));
    io(self).clone_anim_exit(self);
    // ((-> target-board-start exit)): target-board-exit.
    target_board_exit(self);
    vector_reset(c.transv);
}

void clone_anim_code(Target& self, Co& co, const StateArgs&) {
    ControlInfo& c = self.control;
    CO_BEGIN(co);
    runtime(self).word04.set_float(c.draw_offset.y);
    c.draw_offset.y = 0.0f;
    io(self).sidekick_matrix(Symbol("play-anim"));
    // (clone-anim arg0 #t ""): the clone's playback is the adapter's (clone_anim_enter); the port
    // goes on as GOAL does once it returns.
    go(self, StateId::target_board_stance);
    CO_END(co);
}

void clone_anim_post(Target& self) { target_no_ja_move_post(self); }

// ---- target-board-hit (arg0 symbol 'attack / 'shove: StateArgs::sym, arg1 StateArgs::attack) ----
void hit_exit(Target& self) {
    if (!next_in(self, {StateId::target_death, StateId::target_board_get_off})) {
        self.focus_status &= ~(focus::dead | focus::hit);
        self.state_flags &= ~sflags::disable_attacks;
    }
    {
        const bool gp = focus_test(self, focus::hit);
        target_exit(self);
        if (gp) {
            self.focus_status |= focus::hit;
        }
    }
    self.state_flags |= sflags::lleg_no_ik | sflags::rleg_no_ik;
    target_board_exit(self);
}

void hit_trans(Target& self) {
    if (io(self).cheat_mode_debug()) {
        if (!io(self).pause_lock() && target_cpad_hold(self, boardpad::r2)) {
            set_time(self, self.control.time_of_last_debug_heal);
            io(self).pickup_health(100.0f);
            go(self, StateId::target_board_stance);
        }
    }
}

// target-hit-move (target-death.gc) inlined, its locals in co: co.v[1] s3-0, co.m s2-1, co.f[0]
// f30-1, co.f[1] f28-1, co.s s1-1, co.b[0] arg1, co.v[0] s5-0.
void hit_code(Target& self, Co& co, const StateArgs& args) {
    ControlInfo& c = self.control;
    Interaction& ia = io(self);
    CO_BEGIN(co);
    ia.water_clear_jump_out();
    self.focus_status &= ~focus::halfpipe;
    set_time(self, self.state_time);
    {
        AttackInfo& gp = runtime(self).attack_info;
        gp.attacker = 0;
        gp.mode = Symbol("generic");
        gp.shove_back = 6144.0f;
        gp.shove_up = 4915.2f;
        gp.angle = Symbol();
        gp.trans = c.trans;
        gp.control = 0.0f;
        gp.invinc_time = target_bank.hit_invulnerable_timeout;
        gp.speed = 1.0f;
        gp.damage = fact_health_default_inc();
        gp.knock = 0;
        if (args.sym == Symbol("shove")) {
            gp.shove_back = target_bank.smack_surface_dist;
            gp.shove_up = target_bank.smack_surface_height;
            gp.angle = Symbol("shove");
        }
        if (args.attack) {
            attack_info_combine(self, gp, *args.attack);
        }
        if (!(gp.mask & attack_mask::vector)) {
            vector_z_quaternion(gp.vector, c.quat_for_control);
            vector_xz_normalize(gp.vector, -std::fabs(gp.shove_back));
            gp.vector.y = gp.shove_up;
        }
        co.v[0] = gp.vector;
        {
            Vec n, z;
            vector_normalize_copy(n, co.v[0], 1.0f);
            vector_z_quaternion(z, c.quat_for_control);
            const float f0 = vector_dot(n, z);
            if (!gp.angle) {
                gp.angle = 0.0f >= f0 ? Symbol("front") : Symbol("back");
            }
        }
        if (args.sym == Symbol("attack")) {
            self.focus_status |= focus::hit;
            ia.set_hit_time();
            const Symbol m = gp.mode;
            if (m == Symbol("endlessfall")) {
                if (ia.game_mode() == Symbol("debug")) {
                    co.v[1] = c.last_trans_on_ground;
                    ja_channel_set(jc(self), 0, anim_clock(self));
                    co.i[0] = static_cast<int>(current_time(self));
                    goto endlessfall_debug;  // the suspend-for, outside the scopes it cannot cross
                } else {
                    ia.pickup_health(-1000.0f);
                    StateArgs a;
                    a.sym = runtime(self).attack_info.mode;
                    go(self, StateId::target_death, a);
                }
            } else if (m == Symbol("drown-death") || m == Symbol("sharkey") || m == Symbol("lava") ||
                       m == Symbol("dark-eco-pool") || m == Symbol("melt") || m == Symbol("instant-death") ||
                       m == Symbol("crush") || m == Symbol("death") || m == Symbol("grenade") ||
                       m == Symbol("big-explosion") || m == Symbol("bot")) {
                ia.pickup_health(-1000.0f);
                if (ia.game_mode() == Symbol("play")) {
                    StateArgs a;
                    a.sym = m;
                    go(self, StateId::target_death, a);
                }
            } else {
                // (('death) ...) after the list that already has 'death: never reached.
                ia.pickup_health(-gp.damage);
            }
            target_hit_effect(self, runtime(self).attack_info);
        } else {
            if (gp.mode == Symbol("burn") || gp.mode == Symbol("burnup")) {
                sound_play(self, "get-burned");
            }
        }
    }
    {
        AttackInfo& gp = runtime(self).attack_info;
        if (!(ia.game_mode() == Symbol("play") && 0.0f >= ia.health())) {
            if (gp.knock == 8) {
                set_quaternion(self, c.dir_targ);
                set_forward_vel(self, -3.0f * gp.shove_back);
                // (demo?) talker-spawn-func speech 78 and hide-hud-quick: the demo's, left out.
                StateArgs a;
                a.sym = Symbol("hit");
                go(self, StateId::target_board_get_off, a);
            }
            if (gp.control == 1.0f) {
                go(self, StateId::target_board_jump, jump_args(gp.shove_up, gp.shove_up, Symbol("hit")));
            }
        }
        c.mod_surface = &smack_mods;
        {
            const AnimId v1 = group(self);
            if (!(v1 == AnimId::jakb_board_hit || v1 == AnimId::jakb_board_hit_forward)) {
                push(self, 1, seconds(0.075));
                if (gp.mode == Symbol("shock") || gp.mode == Symbol("shock-red") || gp.mode == Symbol("shockup")) {
                    ja(self, Ja().group(AnimId::jakb_board_hit_elec).min());
                } else {
                    ja(self, Ja().group(AnimId::jakb_board_hit).min());
                }
            }
        }
        co.b[0] = target_hit_orient(self, gp, co.v[0]);
        // ---- target-hit-move gp-0, arg1 co.b[0], arg2 target-board-falling-anim-trans, arg3 speed ----
        vector_identity(c.transv);
        set_vertical(self, [&] { return 2048.0f + std::sqrt(491520.0f * runtime(self).attack_info.shove_up); });
        co.v[1] = c.trans;
        matrix_rotate_y(co.m, 32768.0f + vector_y_angle(gp.vector));
        co.f[0] = 0.0f;
        set_quaternion(self, c.dir_targ);
        {
            const JointControlChannel& ch = jc(self).root_channel(0);
            const AnimInfo& ai = anim_info(ch.frame_group);
            co.f[1] = 1.05f * goal_div(-60.0f * gp.speed,
                                       static_cast<float>(ai.num_frames + -1) * ai.artist_step);
        }
    }
    do {
        {
            AttackInfo& gp = runtime(self).attack_info;
            co.f[0] = co.f[0] + gp.shove_back * co.f[1] * spf(self);
            co.s = target_hit_push(self, co.v[1], co.m, co.f[0], gp.shove_back * co.f[1], gp);
        }
        CO_SUSPEND(co);
        ja(self, Ja().seek());
    } while (!(done(self) || (co.b[0] && (c.status & cstat::on_surface))));
    while ((!(c.status & cstat::on_surface) || co.s) && co.s != Symbol("stuck")) {
        target_board_falling_anim_trans(self);
        {
            AttackInfo& gp = runtime(self).attack_info;
            co.f[0] = co.f[0] + gp.shove_back * co.f[1] * spf(self);
            co.s = target_hit_push(self, co.v[1], co.m, co.f[0], gp.shove_back * co.f[1], gp);
        }
        if (!co.s) {
            self.focus_status &= ~focus::hit;
        }
        CO_SUSPEND(co);
    }
    // ---- end of target-hit-move ----
    if (false) {
    endlessfall_debug:
        while (!time_elapsed(self, co.i[0], seconds(1))) {
            CO_SUSPEND(co);
        }
        move_to_point(self, co.v[1]);
        // (set! camera-pos trans) and (send-event *camera* 'teleport): the camera is the host's.
        go(self, StateId::target_stance);
    }
    if (ia.game_mode() == Symbol("play") && 0.0f >= ia.health()) {
        StateArgs a;
        a.sym = runtime(self).attack_info.mode;
        go(self, StateId::target_death, a);
    }
    // enter-state: a bare symbol in the GOAL source, nothing.
    go(self, StateId::target_board_hit_ground);
    CO_END(co);
}

void hit_post(Target& self) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    c.dynam->gravity_max = c.standard_dynamics->gravity_max;
    c.dynam->gravity_length = c.standard_dynamics->gravity_length;
    vector_float_mul(c.dynam->gravity, c.dynam->gravity_normal, c.dynam->gravity_length);
    target_post(self);
    target_board_effect(self);
    const Symbol m = rt.attack_info.mode;
    if (m == Symbol("shock") || m == Symbol("shock-red") || m == Symbol("shockup")) {
        const TimeFrame now = rt.io->game_clock_frame_counter();
        if (now - rt.shock_effect_time >= seconds(0.03)) {
            rt.shock_effect_time = now;
            // (send-event self 'color-effect 'shock (seconds 0.2)) and process-drawable-shock-effect.
            rt.io->shock_effect();
        }
    }
    target_board_joint_points(self);
}

} // namespace

// ---- the table: each defstate's hooks (nullptr where the defstate has none) ----
const StateDef& state_def(StateId id) {
    static const StateDef kDefs[] = {
        {StateId::none, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
        {StateId::target_board_start, event_board, nullptr, exit_board, nullptr, start_code, post_target},
        {StateId::target_board_stance, stance_event, stance_enter, stance_exit, stance_trans, stance_code, post_board},
        {StateId::target_board_duck_stance, stance_event, duck_stance_enter, duck_stance_exit, duck_stance_trans,
         stance_code, post_board},
        {StateId::target_board_jump, jump_event, jump_enter, exit_board, jump_trans, jump_code, post_board},
        {StateId::target_board_halfpipe, halfpipe_event, halfpipe_enter, halfpipe_exit, halfpipe_trans,
         halfpipe_code, post_board},
        {StateId::target_board_falling, event_board, falling_enter, halfpipe_exit, falling_trans, falling_code,
         post_board},
        {StateId::target_board_jump_kick, event_board, jump_kick_enter, exit_board, jump_kick_trans,
         jump_kick_code, post_board},
        {StateId::target_board_wall_kick, event_board, wall_kick_enter, exit_board, target_board_jump_trans,
         wall_kick_code, post_board},
        {StateId::target_board_flip, jump_event, flip_enter, flip_exit, flip_trans, flip_code, post_board},
        {StateId::target_board_hold, jump_event, hold_enter, hold_exit, hold_trans, hold_code, post_board},
        {StateId::target_board_trickx, jump_event, hold_enter, trickx_exit, trickx_trans, trickx_code, post_board},
        {StateId::target_board_hit_ground, event_board, hit_ground_enter, exit_board, hit_ground_trans,
         hit_ground_code, post_board},
        {StateId::target_board_turn_to, event_board, turn_to_enter, exit_board, turn_to_trans, stance_code,
         post_board},
        {StateId::target_board_ride_edge, ride_edge_event, ride_edge_enter, ride_edge_exit, ride_edge_trans,
         ride_edge_code, ride_edge_post},
        {StateId::target_board_grenade, grenade_event, grenade_enter, grenade_exit, grenade_trans, grenade_code,
         post_board},
        {StateId::target_board_get_on, get_on_event, get_on_enter, exit_board, nullptr, get_on_code, post_board},
        {StateId::target_board_pegasus, pegasus_event, pegasus_enter, exit_board, nullptr, pegasus_code,
         pegasus_post},
        {StateId::target_board_get_off, get_on_event, get_off_enter, exit_board, nullptr, get_off_code,
         get_off_post},
        {StateId::target_board_grab, grab_event, grab_enter, grab_exit, nullptr, grab_code, post_board},
        {StateId::target_board_clone_anim, clone_anim_event, clone_anim_enter, clone_anim_exit, nullptr,
         clone_anim_code, clone_anim_post},
        {StateId::target_board_hit, event_board, nullptr, hit_exit, hit_trans, hit_code, hit_post},
        // The target's own states: run by the adapter (Interaction::go_target_state).
        {StateId::target_falling, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
        {StateId::target_hit_ground, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
        {StateId::target_jump, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
        {StateId::target_stance, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
        {StateId::target_grab, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
        {StateId::target_death, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
        {StateId::target_hit, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
        {StateId::target_attack_air, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
        {StateId::target_flop, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
    };
    static_assert(sizeof(kDefs) / sizeof(kDefs[0]) == static_cast<size_t>(StateId::count), "one per state");
    return kDefs[static_cast<size_t>(id)];
}

} // namespace jak
