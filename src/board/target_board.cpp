// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The board's physics and helpers: goal_src/jak2/engine/target/board/target-board.gc, from
// OpenGOAL (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md),
// function by function in the file's order. The surface tables at its top (*board-*-mods*) are
// core's (surfaces.cpp); *collide-edge-board-halfpipe-spec* is collide's.
//
// Left out, each marked where it was: the debug drawing (add-debug-*), the profiler bars
// (*debug-segment*), the debug formats (format #t), *target-rc-board-controls* (a debug option, #f).
#include <cmath>

#include "board/anim.hpp"
#include "board/board.hpp"
#include "board/collide.hpp"
#include "board/core.hpp"
#include "board/surfaces.hpp"
#include "board/target_hit.hpp"
#include "jak/math.hpp"
#include "jak/math_board.hpp"
#include "jak/math_core.hpp"

namespace jak {

namespace {

const Symbol kTrue("#t");


Vec sub(const Vec& a, const Vec& b) {
    Vec r;
    vector_sub(r, a, b);
    return r;
}


// pat-surface bits as the code shifts them out of a word.
uint32_t pat_mode_bits(uint32_t pat) { return (pat << 22) >> 29; }   // (shr (shl x 54) 61)
uint32_t pat_event_bits(uint32_t pat) { return (pat << 8) >> 26; }   // (shr (shl x 40) 58)
uint32_t pat_skip_bits(uint32_t pat) { return pat & 0x7fu; }         // (shr (shl x 57) 57)

// The pat-surface the board ignores: noentity, noboard, probe (target-board-init, probe-test calls).
constexpr uint32_t kPatNoentityNoboardProbe = (1u << 0) | (1u << 5) | (1u << 24);
// noentity, nojak, probe (target-board-exit).
constexpr uint32_t kPatNoentityNojakProbe = (1u << 0) | (1u << 4) | (1u << 24);

bool board_next_p(const Target& self) {
    switch (runtime(self).next_state) {
    case StateId::target_board_stance: case StateId::target_board_jump:
    case StateId::target_board_hit_ground: case StateId::target_board_jump_kick:
    case StateId::target_board_wall_kick: case StateId::target_board_ride_edge:
    case StateId::target_board_duck_stance: case StateId::target_board_falling:
    case StateId::target_board_flip: case StateId::target_board_trickx:
    case StateId::target_board_hold: case StateId::target_board_halfpipe:
    case StateId::target_board_grenade: case StateId::target_board_turn_to:
    case StateId::target_board_hit: case StateId::target_board_get_off:
    case StateId::target_board_pegasus: case StateId::target_board_grab:
    case StateId::target_board_clone_anim:
        return true;
    default:
        return false;
    }
}

uint32_t new_sound_id(Target& self) {
    // new-sound-id: a fresh id the looping sounds play under.
    static uint32_t next = 0x10000;
    (void)self;
    return ++next;
}

} // namespace

const TargetBoardBank kTargetBoardBank{};

// ---- small shared helpers ----

uint32_t sound_play(Target& self, const char* name, float volume, float pitch) {
    return self.effects ? self.effects->sound_play(name, volume, pitch) : 0;
}

void sound_stop(Target& self, uint32_t& id) {
    if (self.effects && id != 0) {
        self.effects->sound_stop(id);
    }
}

// The set-param (volume -4, auto-time, auto-from 2) the board sends to fade a sound out: on the host
// a stop. auto_time is the fade's length in GOAL's sound ticks.
void sound_fade(Target& self, uint32_t id, int auto_time) {
    (void)auto_time;
    if (self.effects && id != 0) {
        self.effects->sound_stop(id);
    }
}

void particles(Target& self, const char* group, const Vec& at, float rate) {
    if (self.effects) {
        self.effects->particles(group, at, rate);
    }
}

// The y-angle method of trsqv (transformq.gc): (quaternion-y-angle (get-quaternion this)).
float control_y_angle(const Target& self) { return quaternion_y_angle(self.control.quat); }

// ---- target-board-handler ----
Symbol target_board_handler(Target& self, const EventMessage& e) {
    BoardRuntime& rt = runtime(self);
    const Symbol m = e.message;
    if (m == Symbol("query") && e.sym[0] == Symbol("mode")) {
        return Symbol("board");
    }
    if (m == Symbol("end-mode")) {
        StateArgs a;
        a.handle = e.from;
        go(self, StateId::target_board_get_off, a);
        return kTrue;
    }
    if (m == Symbol("change-mode")) {
        const Symbol v1 = e.sym[0];
        if (v1 == Symbol("grab")) {
            if (focus_test(self, focus::dead)) {
                return Symbol();
            }
            if (!e.truthy[1]) {
                return kTrue;
            }
            go(self, StateId::target_board_grab, StateArgs());
            return kTrue;
        }
        if (v1 == Symbol("normal")) {
            StateArgs a;
            a.handle = e.from;
            go(self, StateId::target_board_get_off, a);
            return kTrue;
        }
        if (v1 == Symbol("falling")) {
            go(self, StateId::target_board_falling);
            return kTrue;
        }
        if (v1 == Symbol("pegasus")) {
            StateArgs a;
            a.handle = e.from;
            a.sym = Symbol("pegasus");
            go(self, StateId::target_board_get_off, a);
            return kTrue;
        }
        if (v1 == Symbol("pilot")) {
            return rt.io->standard_event_handler(self, e);
        }
        return Symbol();
    }
    if (m == Symbol("swim") || m == Symbol("wade") || m == Symbol("slide") || m == Symbol("pole-grab")) {
        return Symbol();
    }
    if (m == Symbol("clone-anim")) {
        StateArgs a;
        a.handle = e.proc;
        go(self, StateId::target_board_clone_anim, a);
        return kTrue;
    }
    if (m == Symbol("edge-grab")) {
        EdgeGrabInfo& gp = edge_grab_info();
        Vec d;
        vector_normalize(vector_sub(self.control.edge_grab_edge_dir, gp.world_vertex[1], gp.world_vertex[0]), 1.0f);
        (void)d;
        Vec s5;
        vector_add_float_mul(s5, self.control.trans, self.control.c_R_w.v[2], 40960.0f * seconds_per_frame(self));
        const float f0 = vector_segment_overlap(s5, gp.world_vertex[0], gp.world_vertex[1]);
        if (time_elapsed(self, self.board->ride_time, seconds(0.4)) && f0 >= 0.0f && 1.0f >= f0 &&
            !next_state_is(self, StateId::target_board_duck_stance)) {
            StateArgs a;
            a.sym = (self.control.mod_surface->flags & sflag::air) ? kTrue : Symbol();
            go(self, StateId::target_board_ride_edge, a);
            return kTrue;
        }
        return Symbol();
    }
    if (m == Symbol("touched")) {
        if (self.control.danger_mode && rt.io->prims_touching(e.touching, 1920)) {
            if (rt.io->target_send_attack(e.from, self.control.danger_mode, e.touching,
                                          self.control.target_attack_id, self.control.attack_count,
                                          self.control.penetrate_using)) {
                particles(self, "group-board-spin-attack", self.control.trans);
                target_timed_invulnerable(seconds(0.5), self, 2);
                return kTrue;
            }
            return Symbol();
        }
        const bool v0 = rt.io->send_board_attack(e.from, e.touching, self.board->attack_id,
                                                 self.control.penetrate_using);
        if (v0) {
            self.board->attack_id = rt.io->next_attack_id();
        }
        return v0 ? kTrue : Symbol();
    }
    if (m == Symbol("attack") || m == Symbol("attack-or-shove") || m == Symbol("attack-invinc")) {
        return target_attacked(self, m, *e.attack, e.from, StateId::target_board_hit);
    }
    if (m == Symbol("shove")) {
        if (!focus_test(self, focus::hit)) {
            AttackInfo* rec = &rt.attack_info_rec;
            *rec = *e.attack;
            if (!(rec->mask & attack_mask::attacker)) {
                rec->attacker = e.from;
                rec->mask |= attack_mask::attacker;
            }
            StateArgs a;
            a.sym = Symbol("shove");
            a.attack = rec;
            go(self, StateId::target_board_hit, a);
            return kTrue;
        }
        return Symbol();
    }
    return rt.io->standard_event_handler(self, e);
}

// ---- target-board-setup ----
void target_board_setup(Target& self, bool spawn) {
    BoardRuntime& rt = runtime(self);
    if (!rt.board_setup) {
        rt.board_setup = true;
        self.board->latch_p = Symbol();
        rt.engine_sound_id = new_sound_id(self);
        rt.ride_sound_id = new_sound_id(self);
        rt.spin_sound_id = 0;
        rt.wind_sound_id = new_sound_id(self);
        rt.bank_sound_id = new_sound_id(self);
        self.board->attack_id = rt.io->next_attack_id();
        rt.board_proc.alive = false;
    }
    if (spawn) {
        if (!rt.board_proc.alive) {
            board_init(self);
        }
    } else if (rt.board_proc.alive) {
        board_deactivate(self);
    }
}

// ---- target-board-init ----
void target_board_init(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    target_gun_end_mode(self, false);
    target_exit(self);
    target_board_setup(self, true);
    b.latch_p = Symbol();
    // (the joint-mods and the part-control are created once; here they are always there)
    rt.part_control = true;
    rt.main.mode = JointModMode::flex_blend;
    rt.upper_body.mode = JointModMode::flex_blend;
    b.engine_sound_pitch = 0.0f;
    b.engine_sound_volume = 1.0f;
    b.bank_sound_pitch = 0.0f;
    b.bank_sound_volume = 0.0f;
    b.wind_sound_pitch = 0.0f;
    b.wind_sound_volume = 1.0f;
    b.spin_sound_volume = 0.0f;
    b.sound_air_knob = 0.0f;
    b.sound_bank_knob = 0.0f;
    rt.unknown_sound_id00 = 0;
    rt.io->set_setting("mode-sound-bank", Symbol("board"), 0.0f);
    rt.io->set_setting("sound-flava", Symbol(), 30.0f);
    set_time(self, b.board_get_on_time);
    b.stick_lock = Symbol();
    b.stick_off = Symbol();
    set_time(self, b.unstuck_time);
    b.stuck_count = 0;
    b.slip_factor = 1.0f;
    b.unknown_symbol00 = Symbol();
    b.halfpipe_side_time = 0;
    b.spin_control = 0.0f;
    b.trotyv = 0.0f;
    b.troty = 0.0f;
    b.troty_cum = 0.0f;
    b.unknown_deg00 = 0.0f;
    b.flip_control = 0.0f;
    b.flip_count = 0;
    b.unknown_float00 = 0.0f;
    b.unknown_float01 = 0.0f;
    reset_combo(self); // PC port: the trick display starts over
    b.trick_count = 0;
    for (int i = 15; i >= 0; i--) {
        b.trick_list[0] = BoardTricks::none; // (GOAL clears entry 0 sixteen times)
    }
    b.upper_body_rotyv = 0.0f;
    b.upper_body_roty = 0.0f;
    b.upper_body_rotyv_max = 91022.22f;
    b.ride_lock = Symbol();
    b.ride_lock_on = Symbol();
    b.ride_rot_old = -1.0f;
    // (control reaction = target-collision-reaction: collide's default)
    self.control.pat_ignore_mask.bits = kPatNoentityNoboardProbe;
    self.focus_status |= focus::board;
    root_prim_action(self) &= ~caction::can_ride;
    self.control.bend_target = 1.0f;
    // (the root cspace follows the world trans: drawing only)
    target_collide_set(self, Symbol("board"), 0.0f);
    self.state_flags |= sflags::lleg_no_ik | sflags::rleg_no_ik;
    b.thrust_scale = 1.0f;
    b.turn_anim_targ = 0.0f;
    b.turn_anim_frame = 0.0f;
    b.turn_anim_vel = 0.0f;
    b.turn_anim_duck = 0.0f;
    b.turn_anim_tilt_p = Symbol();
    b.cushion_base = 0.0f;
    b.cushion_offset = 0.0f;
    b.shock_offset = 0.0f;
    b.shock_offsetv = 0.0f;
    b.shock_rotx = 0.0f;
    Dynamics& d = *self.control.dynam;
    d.gravity_max = self.control.standard_dynamics->gravity_max;
    d.gravity_length = self.control.standard_dynamics->gravity_length;
    vector_float_mul(d.gravity, d.gravity_normal, d.gravity_length);
    rt.io->set_setting("string-spline-max-move", Symbol("abs"), meters(2));
    rt.io->set_setting("string-spline-accel", Symbol("abs"), meters(0.045));
    rt.io->set_setting("string-spline-max-move-player", Symbol("abs"), meters(1.5));
    rt.io->set_setting("string-spline-accel-player", Symbol("abs"), meters(0.035));
}

// ---- target-board-exit: undoes the board unless the next state is one of the board's ----
void target_board_exit(Target& self) {
    if (board_next_p(self)) {
        return;
    }
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    b.latch_p = Symbol();
    rt.main.mode = JointModMode::flex_blend;
    rt.upper_body.mode = JointModMode::flex_blend;
    Dynamics& d = *self.control.dynam;
    d.gravity_max = self.control.standard_dynamics->gravity_max;
    d.gravity_length = self.control.standard_dynamics->gravity_length;
    vector_float_mul(d.gravity, d.gravity_normal, d.gravity_length);
    self.focus_status &= ~focus::board;
    root_prim_action(self) &= ~(caction::stuck_wall_escape | caction::no_normal_reset);
    root_prim_action(self) |= caction::can_ride;
    self.control.mod_surface = &walk_mods;
    self.state_flags &= ~sflags::sf6;
    rt.io->remove_setting("string-spline-max-move");
    rt.io->remove_setting("string-spline-accel");
    rt.io->remove_setting("string-spline-max-move-player");
    rt.io->remove_setting("string-spline-accel-player");
    // (the root cspace goes back to the trans: drawing only)
    target_collide_set(self, Symbol("normal"), 0.0f);
    self.control.pat_ignore_mask.bits = kPatNoentityNojakProbe;
    sound_stop(self, rt.wind_sound_id);
    sound_stop(self, rt.engine_sound_id);
    sound_stop(self, rt.bank_sound_id);
    sound_stop(self, rt.ride_sound_id);
    sound_stop(self, rt.spin_sound_id);
    rt.spin_sound_id = 0;
    rt.io->sidekick_matrix(Symbol("normal"));
    self.control.cspace_offset = Vec{};
    rt.io->remove_setting("sound-flava");
    rt.io->remove_setting("mode-sound-bank");
    target_exit(self);
}

// ---- target-board-falling-anim-trans ----
void target_board_falling_anim_trans(Target& self) {
    JointControl& jc = skel(self);
    if (ja_group(jc) != AnimId::jakb_board_jump_loop) {
        ja_channel_push(jc, 1, seconds(0.33), anim_clock(self));
        ja(self, Ja().group(AnimId::jakb_board_jump_loop));
    } else {
        ja(self, Ja().loop());
    }
}

// ---- board-on-ground? ----
bool board_on_ground_p(const Target& self) { return (self.control.status & cstat::on_surface) != 0; }

// ---- target-board-smack-surface? ----
bool target_board_smack_surface_p(Target& self) {
    ControlInfo& c = self.control;
    if ((c.status & cstat::touch_wall) && current_time(self) + seconds(-0.05) < c.last_time_touching_actor &&
        0.7f < c.touch_angle && 73728.0f < c.ctrl_xz_vel &&
        vector_dot(c.wall_contact_normal, c.dynam->gravity_normal) < 0.3f &&
        !(c.status & cstat::touch_actor)) {
        set_time(self, self.board->smack_surface_time);
        self.board->smack_speed = c.ctrl_xz_vel;
        self.board->smack_normal = c.wall_contact_normal;
        return true;
    }
    return false;
}

// ---- board-add-thrust: the board's own push along the facing, in control space ----
void board_add_thrust(Target& self) {
    ControlInfo& c = self.control;
    BoardInfo& b = *self.board;
    const Surface& cs = *c.current_surface;
    Vec& gp = c.transv_ctrl;
    const float spf = seconds_per_frame(self);
    float f0_7;
    if (std::sqrt(gp.x * gp.x + gp.z * gp.z) == 0.0f) {
        f0_7 = 0.0f;
    } else {
        f0_7 = goal_div(gp.z, std::sqrt(gp.x * gp.x + gp.z * gp.z));
    }
    const float f30 = f0_7 >= 0.0f ? f0_7 * cs.seek0 + (1.0f - f0_7) * cs.seek90
                                   : std::fabs(f0_7) * cs.seek180 + (1.0f + f0_7) * cs.seek90;
    float f28;
    if (b.stick_off) {
        f28 = 0.0f;
    } else if (c.ground_pat.mode() == kPatHalfpipe) {
        f28 = 1.0f;
    } else if (gp.z < 0.0f) {
        // (*target-rc-board-controls*, a debug option, would give 1.0 before this)
        f28 = std::fmax(0.75f, c.turn_to_magnitude);
    } else {
        f28 = c.turn_to_magnitude;
    }
    if (!(cs.vel_turn == 0.0f || gp.z < 0.0f)) {
        const float f0_22 = atan(gp.x, gp.z);
        vector_rotate_y(gp, gp, std::fmin(0.03f * self.clock.time_adjust_ratio * -f0_22, cs.vel_turn * spf));
    }
    float f0_28 = f30 * b.thrust_scale * lerp_scale(0.4f, 1.0f, f28, 0.3f, 1.0f);
    const float f1_22 = cs.target_speed;
    float f1_23 = c.local_slope_z < 0.0f ? f1_22 + -c.local_slope_z * cs.slope_down_factor
                                         : f1_22 + -c.local_slope_z * cs.slope_up_factor;
    if (b.stick_off) {
        f0_28 = 0.0f;
        f1_23 = 0.0f;
    }
    if (!time_elapsed(self, c.last_time_touching_actor, seconds(1)) && f28 >= 0.5f &&
        vector_dot(c.wall_contact_normal, c.to_target_pt_xz) < -0.7f &&
        (c.mod_surface->flags & sflag::air) && 0.0f < gp.y) {
        if (time_elapsed(self, c.last_time_touching_actor, seconds(0.1)) && 0.3f < c.blocked_factor) {
            f0_28 = f1_23;
        } else if (gp.z < 0.0f) {
            gp.z = 0.0f;
        }
    }
    gp.z = gp.z + f0_28 * spf;
    if (f1_23 != 0.0f) {
        const float f0_33 = 1.0f - goal_div(f30 * spf, f1_23);
        gp.x = gp.x * f0_33;
        gp.z = gp.z * f0_33;
    }
    b.transv_max = f1_23;
    const float f0_35 = cs.transv_max;
    if (std::sqrt(gp.x * gp.x + gp.z * gp.z) >= f0_35) {
        vector_xz_normalize(gp, f0_35);
    }
    // (add-debug-text-sphere "ltransv" / "btransv": debug drawing)
}

// ---- segment-array-vector->index: where a point is along a polyline, in segments ----
float segment_array_vector_to_index(const Vec* arr, int len, const Vec& pt, float add) {
    int gp = -1;
    float f30 = 0.0f;
    float f0 = 0.0f;
    while (gp + 1 < len) {
        gp += 1;
        f30 = vector_segment_overlap(pt, arr[gp], arr[gp + 1]);
        f0 = goal_div(add, vector_vector_distance(arr[gp], arr[gp + 1]));
        if (1.0f >= f30 + f0) {
            return static_cast<float>(gp) + f30 + f0;
        }
    }
    return static_cast<float>(gp) + f30 + f0;
}

// ---- segment-array-index->vector: the point at an index along a polyline ----
int segment_array_index_to_vector(const Vec* arr, int len, float index, Vec& out) {
    int v0 = static_cast<int32_t>(index);
    float f0_4 = index - static_cast<float>(static_cast<int32_t>(index));
    if (v0 >= len) {
        v0 = len + -1;
        f0_4 = index - static_cast<float>(v0);
    }
    const Vec& v1 = arr[v0];
    const Vec& a1 = arr[v0 + 1];
    vector_add_float_mul(out, v1, sub(a1, v1), f0_4);
    return v0;
}

// ---- target-board-exit-check: R2 (or losing the board) gets off once on the ground a while ----
void target_board_exit_check(Target& self) {
    BoardRuntime& rt = runtime(self);
    const bool pressed = target_cpad_pressed(self, boardpad::r2) &&
                         (!rt.io->cheat_mode_debug() || !target_cpad_hold(self, boardpad::l2)) &&
                         !rt.io->pause_lock();
    if ((pressed || (self.state_flags & sflags::prevent_board) || !rt.io->game_feature_board()) &&
        time_elapsed(self, self.board->board_get_on_time, seconds(1)) &&
        self.board->board_get_on_time <
            std::max(self.control.list_time_on_ground, self.control.last_time_of_stuck) &&
        !next_state_is(self, StateId::target_board_grab)) {
        StateArgs a;
        a.handle = 0; // (process->handle self)
        go(self, StateId::target_board_get_off, a);
    }
}

// ---- target-board-effect: the engine, bank, wind and spin sounds, the upper body, the dust ----
void target_board_effect(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    const float spf = seconds_per_frame(self);
    int gp = 0;
    if (focus_test(self, focus::rail)) {
        gp = 10;
    } else if (c.mod_surface->name == Symbol("spin")) {
        gp = 3;
    } else if (next_state_is(self, StateId::target_board_flip)) {
        gp = 4;
    } else if ((c.mod_surface->flags & sflag::air) && !(c.status & cstat::on_surface)) {
        gp = 1;
    } else if (!(c.status & cstat::on_surface)) {
        gp = 2;
    }
    // (gp is the engine sound's reg0, its mode; Effects has no register to pass it in)
    (void)gp;
    if (board_on_ground_p(self)) {
        b.sound_air_knob = seek(b.sound_air_knob, 0.0f, 16.0f * spf);
    } else {
        b.sound_air_knob = seek(b.sound_air_knob,
                                lerp_scale(0.0f, 1.0f, target_height_above_ground(self), 409.6f, 28672.0f),
                                2.0f * spf);
    }
    {
        const float f30 = std::fabs(goal_div(b.turn_anim_frame, kTargetBoardBank.turn_frames));
        b.sound_bank_knob = seek(b.sound_bank_knob, f30, b.sound_bank_knob < f30 ? 0.5f * spf : spf);
        b.engine_sound_volume = lerp_scale(0.8f, 1.0f, f30, 0.0f, 1.0f);
    }
    b.engine_sound_pitch = lerp_scale(0.0f, -0.1f, b.sound_bank_knob, 0.0f, 1.0f) +
                           lerp_scale(-0.15f, 0.0f, b.shock_offset, -3072.0f, 52.0192f);
    b.bank_sound_volume = lerp_scale(0.0f, 1.0f, b.sound_bank_knob, 0.0f, 1.0f);
    b.bank_sound_pitch = lerp_scale(0.0f, -0.2f, b.sound_bank_knob, 0.0f, 1.0f);
    {
        const float f0_25 = b.sound_air_knob;
        if (0.0f < f0_25) {
            b.engine_sound_volume = lerp_scale(b.engine_sound_volume, 0.3f, f0_25, 0.0f, 1.0f);
        }
    }
    // sound-play-by-spec "board-steady": volume and pitch-mod as the ints the spec holds.
    if (self.effects) {
        self.effects->sound_loop(
            "board-steady", rt.engine_sound_id,
            static_cast<float>(static_cast<int32_t>(1024.0f * b.engine_sound_volume)) / 1024.0f,
            static_cast<float>(static_cast<int32_t>(1524.0f * b.engine_sound_pitch)));
    }
    if (b.bank_sound_volume < 0.1f) {
        if (self.effects) {
            self.effects->sound_stop(rt.bank_sound_id);
        }
    } else if (self.effects) {
        self.effects->sound_loop(
            "board-bank", rt.bank_sound_id,
            static_cast<float>(static_cast<int32_t>(1024.0f * b.bank_sound_volume)) / 1024.0f,
            static_cast<float>(static_cast<int32_t>(1524.0f * b.bank_sound_pitch)));
    }
    // "board-wind": the spec takes (the int wind-sound-volume) unscaled, so 1.0 plays at 1/1024.
    if (self.effects) {
        self.effects->sound_loop("board-wind", rt.wind_sound_id,
                                 static_cast<float>(static_cast<int32_t>(b.wind_sound_volume)) / 1024.0f,
                                 static_cast<float>(static_cast<int32_t>(b.wind_sound_pitch)));
    }
    if (c.mod_surface->name == Symbol("spin")) {
        b.spin_sound_volume = seek(b.spin_sound_volume,
                                   lerp_scale(0.0f, 1.0f, std::fabs(b.trotyv), 0.0f, 91022.22f), 8.0f * spf);
        b.spin_sound_pitch = seek(b.spin_sound_pitch,
                                  lerp_scale(0.0f, 0.3f, std::fabs(b.trotyv), 36408.89f, 91022.22f), spf);
        // set-param volume of the spin loop (mask 1: the volume only)
        if (self.effects && rt.spin_sound_id != 0) {
            self.effects->sound_loop(
                "board-spin-loop", rt.spin_sound_id,
                static_cast<float>(static_cast<int32_t>(1024.0f * b.spin_sound_volume)) / 1024.0f, 0.0f);
        }
    } else {
        b.spin_sound_volume = 0.0f;
    }
    if (b.upper_body_rotyv != 0.0f) {
        rt.upper_body.mode = JointModMode::rotate;
    }
    if (rt.upper_body.mode == JointModMode::rotate) {
        b.upper_body_rotyv =
            0.95f * std::fmax(std::fmin(b.upper_body_rotyv, b.upper_body_rotyv_max), -b.upper_body_rotyv_max);
        b.upper_body_rotyv = b.upper_body_rotyv + -10.0f * spf * b.upper_body_roty;
        const float f1_21 = b.upper_body_rotyv * spf;
        const float f0_65 = b.upper_body_roty;
        b.upper_body_roty = b.upper_body_roty + f1_21;
        if (f0_65 * b.upper_body_roty < 0.0f) {
            b.upper_body_roty = 0.0f;
            b.upper_body_rotyv = 0.0f;
            rt.upper_body.mode = JointModMode::flex_blend;
        }
        rt.upper_body.twist_y = b.upper_body_roty;
    }
    // (the draw-status no-draw bits are the adapter's; the target is drawn while it rides)
    if (board_on_ground_p(self) && skel(self).active_channels != 0) {
        particles(self, "group-target-board", c.trans);       // the part-control spawn
        particles(self, "effect-board-poof", c.trans);        // do-effect 'effect-board-poof
        if (b.shock_offset < -2048.0f) {
            particles(self, "effect-board-poof", c.trans);
        }
    }
    vector_lerp(b.slow_transv, b.slow_transv, c.transv, 0.2f);
    if (b.slow_transv.y < 0.0f) {
        b.slow_transv.y = 0.0f;
    }
}

// ---- target-board-physics: the board's spring under Jak (shock-offset) and the cushion ----
void target_board_physics(Target& self, const Vec& arg0) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    const float spf = seconds_per_frame(self);
    {
        const float f30 = 0.5f;
        if (time_elapsed(self, b.unknown_time_frame02, seconds(0.3))) {
            b.shock_offsetv = b.shock_offsetv + (arg0.y - c.transv.y) * f30;
        } else {
            b.up_vector[1] = b.up_vector[0];
        }
        b.shock_offsetv = b.shock_offsetv + (-10.0f * spf * b.shock_offset +
                                             (board_on_ground_p(self) ? c.dynam->gravity_length * spf * f30 : 0.0f));
    }
    if (c.mod_surface->mode == Symbol("ride")) {
        b.shock_offsetv = 0.0f;
        b.shock_offset = 0.96f * b.shock_offset;
    } else if ((!time_elapsed(self, c.last_time_on_surface, seconds(0.2)) || b.shock_offset < 0.0f) &&
               c.mod_surface->mode != Symbol("air")) {
        b.shock_offset = b.shock_offset + b.shock_offsetv * spf;
    }
    b.shock_offset = b.shock_offset * lerp_scale(0.99f, 0.98f, c.ctrl_xz_vel, 0.0f, 28672.0f);
    if (40960.0f < b.shock_offset) {
        b.shock_offset = 40960.0f;
        b.shock_offsetv = 0.0f;
    } else if (b.shock_offset < -3072.0f) {
        if (b.shock_offsetv < -12288.0f) {
            if (c.ground_pat.material() == 3) { // pat-material waterbottom
                // sound-play :pitch -0.5, then faded out over 480 (the set-param)
                const uint32_t id = sound_play(self, "swim-enter", 1.0f, static_cast<float>(static_cast<int32_t>(1524.0f * -0.5f)));
                (void)id;
            } else {
                sound_play(self, "board-bounce",
                           static_cast<float>(static_cast<int32_t>(
                               1024.0f * lerp_scale(1.0f, 0.3f, b.shock_offsetv, -40960.0f, -12288.0f))) /
                               1024.0f,
                           0.0f);
            }
        }
        b.shock_offset = -3072.0f;
        b.shock_offsetv = 0.0f;
        particles(self, "group-land-poof-stn", c.trans);
    }
    c.draw_offset.y = b.cushion_base + b.cushion_offset + b.shock_offset;
    if (c.draw_offset.y < 0.0f) {
        c.draw_offset.y = 0.0f;
    }
}

// ---- target-board-collision: integrate and collide, edge grabs, glancing off walls ----
void target_board_collision(Target& self) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    BoardRuntime& rt = runtime(self);
    Vec gp = c.transv;
    if (focus_test(self, focus::halfpipe) && rt.word04.truthy()) {
        // Keep only a small push off the halfpipe's edge across it.
        Vec v1;
        float f0_1 = vector_dot(c.edge_grab_across_edge_dir, c.transv);
        vector_float_mul(v1, c.edge_grab_across_edge_dir, f0_1);
        vector_sub(v1, c.transv, v1);
        const float f1_2 = vector_length(v1);
        const float f2_0 = f1_2;
        if (0.0f < f0_1) {
            f0_1 = -2048.0f;
        }
        vector_float_mul(c.transv, c.edge_grab_across_edge_dir, f0_1);
        vector_float_mul(v1, v1, goal_div(f1_2, f2_0));
        vector_add(c.transv, c.transv, v1);
    }
    // (profiler bar 'target: debug)
    {
        CollideQuery q;
        q.collide_with = root_prim_collide_with(self);
        q.ignore_self = true;
        q.ignore_pat = c.pat_ignore_mask;
        q.action_mask = caction::solid;
        fill_cache_integrate_and_collide(self, c.transv, q, meters(1));
    }
    if (root_prim_action(self) & caction::check_edge) {
        CollideEdgeSpec& spec = collide_edge_board_spec();
        edge_spec_flags(spec) &= ~edgeflag::one;
        edge_spec_set_touching_segment(spec, nullptr);
        do_edge_grabs(self, spec);
    }
    target_board_physics(self, gp);
    // ((deg-diff (vector-y-angle gp) (vector-y-angle transv)): computed and dropped)
    const float f30 = std::sqrt(gp.x * gp.x + gp.z * gp.z);
    if ((c.status & cstat::touch_wall) && 16384.0f < f30 &&
        // (next-state 'target-board-smack: there is no such state, so never)
        !focus_test(self, focus::halfpipe) && c.ground_pat.mode() != 3 &&
        time_elapsed(self, b.halfpipe_time, seconds(0.1))) {
        Vec s5 = c.wall_contact_normal;
        vector_normalize(s5, 1.0f);
        Vec s3;
        vector_reflect(s3, gp, s5);
        Vec s4 = c.c_R_w.v[2];
        s3.y = 0.0f;
        vector_normalize(s3, 1.0f);
        s4.y = 0.0f;
        vector_normalize(s4, 1.0f);
        // ((deg-diff (y-angle control) (vector-y-angle s3)): dropped)
        Vec s2;
        vector_matrix_mul(s2, c.transv, c.w_R_c);
        Vec s3_3;
        vector_matrix_mul(s3_3, s3, c.w_R_c);
        c.status |= cstat::glance;
        set_time(self, b.glance_time);
        b.glance_speed = std::sqrt(s2.x * s2.x + s2.z * s2.z);
        b.glance_normal = s5;
        b.glance_in_transv = gp;
        s3_3.y = s2.y;
        vector_xz_normalize(s3_3, b.glance_speed);
        vector_matrix_mul(c.transv, s3_3, c.c_R_w);
        b.glance_out_transv = c.transv;
        {
            Vec fwd, up;
            vector_normalize_copy(fwd, c.transv, 1.0f);
            vector_y_quaternion(up, c.dir_targ);
            forward_up_nopitch_to_quaternion(c.dir_targ, fwd, up);
        }
        if (vector_dot(s4, s5) < -0.77f && !(self.focus_status & (focus::dead | focus::hit))) {
            if (!((c.mod_surface->flags & sflag::air) && !(c.status & cstat::on_surface))) {
                if (32768.0f < f30) {
                    sound_play(self, "board-glance");
                }
                StateArgs a;
                a.vec = c.transv;
                a.time = seconds(0.2);
                go(self, StateId::target_board_turn_to, a);
            } else {
                c.turn_lockout_end_time = std::max(current_time(self) + seconds(0.1), c.turn_lockout_end_time);
            }
        }
    }
    {
        // The probe ahead and down for the ground the board will meet (pre-move's lift).
        CollideQuery q;
        q.radius = 1638.4f;
        q.collide_with = root_prim_collide_with(self) & ~cspec::crate;
        q.ignore_self = true;
        q.ignore_pat = c.pat_ignore_mask;
        q.action_mask = caction::solid;
        vector_add_float_mul(q.start_pos, c.trans, c.c_R_w.v[2], 10240.0f);
        vector_add_float_mul(q.start_pos, q.start_pos, c.dynam->gravity_normal, 8192.0f);
        vector_float_mul(q.move_dist, c.dynam->gravity_normal, -49152.0f);
        if (fill_and_probe_using_line_sphere(self, q) >= 0.0f) {
            b.unknown_vector00 = q.best_intersect;
            b.unknown_vector01 = q.best_normal;
            b.unknown_int00 = q.best_pat.bits;
            b.unknown_symbol00 = kTrue;
        } else {
            b.unknown_symbol00 = Symbol();
        }
    }
    const TargetBank& tb = target_bank;
    if (target_move_dist(self, tb.stuck_time) < tb.stuck_distance &&
        !(rt.io->cheat_mode_debug() && target_cpad_hold(self, boardpad::r2))) {
        set_time(self, c.last_time_of_stuck);
    }
}

// ---- target-board-joint-points: the board follows joint 25 ----
void target_board_joint_points(Target& self) {
    BoardRuntime& rt = runtime(self);
    if (rt.board_proc.alive) {
        // matrix->transformq of the joint's matrix: the trans (the quaternion and scale are the
        // drawing's: the adapter reads the joint itself).
        const Mat m = rt.io->joint_transform(25);
        const float q = goal_div(1.0f, m.v[3].w);
        self.board->board_trans = Vec{m.v[3].x * q, m.v[3].y * q, m.v[3].z * q, 1.0f};
        self.board->board_scale = self.control.scale;
        // (*board-shadow-control* copies the target's shadow direction: drawing)
    }
}

// ---- target-board-pre-move: the bend, the gravity, and the lift over a coming step ----
void target_board_pre_move(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    const Vec& gn = c.dynam->gravity_normal;
    const float spf = seconds_per_frame(self);
    const StateId nx = rt.next_state;
    bool flying = false;
    if ((c.mod_surface->flags & sflag::air) && !(c.status & cstat::on_surface)) {
        set_time(self, b.in_air_time);
        if (!(nx == StateId::target_board_stance || nx == StateId::target_board_duck_stance ||
              nx == StateId::target_board_turn_to)) {
            const bool rising =
                (20480.0f < vector_dot(gn, sub(c.trans, c.gspot_pos)) || 0.0f < vector_dot(gn, c.transv)) &&
                c.gspot_slope_z < 0.0f;
            const uint32_t gmode = c.gspot_pat_surfce.mode();
            const bool odd_ground =
                !(gmode == kPatGround || gmode == kPatHalfpipe) ||
                (c.gspot_pat_surfce.event() == 13 ||
                 ((rt.io->water_flags() & waterflag::active) && c.gspot_pos.y < rt.io->water_surface_height()) ||
                 ((nx == StateId::target_board_hold || nx == StateId::target_board_trickx) &&
                  !(self.focus_status & focus::halfpipe)) ||
                 (gmode == kPatHalfpipe && std::fabs(vector_dot(c.gspot_normal, gn)) < 0.7f &&
                  !(self.focus_status & focus::halfpipe)));
            flying = (rising || odd_ground) && time_elapsed(self, b.unknown_time_frame00, seconds(0.2));
        }
    }
    if (flying) {
        c.bend_speed = 0.0f;
        c.bend_target = 0.0f;
    } else {
        c.bend_speed = 1024.0f;
        c.bend_target = 1.0f;
        if (!time_elapsed(self, b.unknown_time_frame00, seconds(0.1))) {
            Vec z;
            vector_z_quaternion(z, c.dir_targ);
            forward_up_nopitch_to_quaternion(c.dir_targ, z, c.gspot_normal);
            quaternion_slerp(c.quat_for_control, c.quat_for_control, c.dir_targ, 0.1f);
        }
    }
    if (target_cpad_hold(self, boardpad::circle | boardpad::square) && !b.ride_lock) {
        root_prim_action(self) |= caction::check_edge;
    } else {
        root_prim_action(self) &= ~caction::check_edge;
    }
    if (!target_cpad_hold(self, boardpad::circle | boardpad::square)) {
        b.ride_lock = Symbol();
    }
    Dynamics& d = *c.dynam;
    if (nx == StateId::target_board_get_on || nx == StateId::target_board_get_off) {
    } else if (nx == StateId::target_board_trickx || nx == StateId::target_board_hold) {
        d.gravity_length = seek(d.gravity_length, 245760.0f, 30.0f * spf);
    } else if (c.mod_surface->name == Symbol("spin")) {
        d.gravity_length = lerp_scale(245760.0f, 204800.0f, std::fabs(b.trotyv), 0.0f, 145635.56f);
    } else if (nx == StateId::target_board_halfpipe) {
        d.gravity_length = 0.0f >= vector_dot(gn, c.transv) ? 245760.0f : 163840.0f;
    } else if (nx == StateId::target_board_stance || nx == StateId::target_board_duck_stance ||
               nx == StateId::target_board_turn_to) {
        if (time_elapsed(self, c.last_time_on_surface, seconds(0.1))) {
            d.gravity_length = seek(d.gravity_length, 245760.0f, 245760.0f * spf);
        } else {
            d.gravity_length = 81920.0f;
        }
    } else {
        d.gravity_length = 245760.0f;
    }
    vector_float_mul(d.gravity, d.gravity_normal, d.gravity_length);
    if (c.force_turn_to_strength < 0.0f) {
        c.force_turn_to_strength = 1.0f - target_cpad_stick0_speed(self);
    }
    if (b.unknown_symbol00 && !(self.focus_status & focus::halfpipe)) {
        const float f30 = vector_dot(gn, sub(b.unknown_vector00, c.trans));
        const float f26 = vector_dot(gn, sub(b.unknown_vector00, c.gspot_pos));
        const float f28 = vector_dot(gn, c.transv);
        Vec flat;
        vector_flatten(flat, sub(c.trans, b.unknown_vector00), c.standard_dynamics->gravity_normal);
        const float f0_36 = c.local_slope_z * vector_length(flat);
        if (pat_mode_bits(b.unknown_int00) == 0 && pat_event_bits(b.unknown_int00) != 13 &&
            time_elapsed(self, b.unknown_time_frame02, seconds(0.2)) && 819.2f < f30 - f0_36 &&
            (b.unknown_time_frame01 < c.last_time_on_surface ||
             !time_elapsed(self, b.unknown_time_frame01, seconds(0.2))) &&
            0.98f < vector_dot(b.unknown_vector01, c.standard_dynamics->gravity_normal) && f26 < 8192.0f &&
            f30 < 8192.0f &&
            (f28 / 5.0f < f30 || !time_elapsed(self, b.unknown_time_frame00, seconds(0.1)))) {
            // A step up ahead: lift the board onto it.
            vector_add_float_mul(c.transv, c.transv, gn, 7.0f * std::fmin(4096.0f, f30));
            if (time_elapsed(self, b.unknown_time_frame00, seconds(0.1))) {
                set_time(self, b.unknown_time_frame01);
            }
            set_time(self, b.unknown_time_frame00);
        }
        if (!(c.status & cstat::on_surface) && 0.0f < f28 &&
            (((target_height_above_ground(self) < 4096.0f ||
               !time_elapsed(self, b.unknown_time_frame00, seconds(0.1))) &&
              8192.0f < f28 - vector_dot(gn, b.slow_transv)) ||
             !time_elapsed(self, b.unknown_time_frame02, seconds(0.1))) &&
            204.8f >= f30 && !next_state_is(self, StateId::target_board_jump)) {
            // Thrown up by the step: damp the rise.
            set_time(self, b.unknown_time_frame02);
            Vec v1;
            const float f0_51 = vector_dot(gn, c.transv);
            vector_float_mul(v1, gn, f0_51);
            vector_sub(v1, c.transv, v1);
            const float f1_24 = vector_length(v1);
            const float f2_0 = f1_24;
            const float f0_52 = 0.9f * f0_51;
            vector_float_mul(c.transv, gn, f0_52);
            vector_float_mul(v1, v1, goal_div(f1_24, f2_0));
            vector_add(c.transv, c.transv, v1);
            b.slow_transv = c.transv;
            b.turn_anim_duck_vel = 0.0f;
        }
    }
}

// ---- target-board-real-post: the board's movement, once per time-adjust step ----
void target_board_real_post(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    const float f30 = clock_ratio(self);
    {
        int gp = std::max(1, static_cast<int>(static_cast<int32_t>(self.clock.time_adjust_ratio)));
        update_rates(self, goal_div(f30, static_cast<float>(gp)));
        while (gp != 0) {
            gp += -1;
            const float spf = seconds_per_frame(self);
            c.remaining_ctrl_iterations = gp;
            flag_setup(self);
            target_board_pre_move(self);
            build_conversions(self, c.transv);
            do_rotations1(self);
            {
                Vec s5{};
                read_pad(self, s5);
                if (b.stick_off) {
                    target_cpad_stick0_speed(self) = 0.0f;
                    c.pad_magnitude = 0.0f;
                }
                // (*target-rc-board-controls*: a debug option, off)
                turn_to_vector(self, s5, c.pad_magnitude);
            }
            if (c.pad_magnitude == 0.0f && current_time(self) >= c.turn_lockout_end_time &&
                !(c.current_surface->flags & sflag::turn_to_vel) &&
                time_elapsed(self, b.unknown_time_frame00, seconds(0.1))) {
                rot_to_dir_targ(self);
            }
            if (!time_elapsed(self, c.last_time_on_surface, seconds(0.1)) &&
                current_time(self) >= c.turn_lockout_end_time &&
                !(c.current_surface->flags & sflag::turn_to_vel) &&
                time_elapsed(self, b.unknown_time_frame00, seconds(0.1))) {
                // Turn down the slope a little.
                Vec neg, s3, up;
                vector_negate(neg, c.dynam->gravity_normal);
                vector_flatten(s3, neg, c.local_normal);
                vector_y_quaternion(up, c.dir_targ);
                Quat a2;
                forward_up_nopitch_to_quaternion(a2, s3, up);
                quaternion_slerp(c.dir_targ, c.dir_targ, a2, 0.05f * std::fabs(c.ctrl_slope_heading));
            }
            if (target_cpad_pressed(self, boardpad::l1)) {
                set_time(self, b.unknown_time_frame06);
            }
            board_add_thrust(self);
            add_gravity(self);
            if (c.mod_surface->name == Symbol("spin")) {
                b.trotyv = 0.95f * std::fmax(std::fmin(b.trotyv, b.trotyv_max), -b.trotyv_max);
                const float f0_15 = b.trotyv * spf;
                // (sar (shl (the int x) 48) 48): wrapped to a signed 16-bit angle.
                b.troty = static_cast<float>(static_cast<int16_t>(
                    static_cast<uint16_t>(static_cast<int32_t>(b.troty + f0_15) & 0xffff)));
                b.unknown_deg00 = b.troty_cum;
                b.troty_cum = b.troty_cum + f0_15;
                b.trotyv_max = seek(b.trotyv_max, 91022.22f, 91022.22f * spf);
                rt.main.mode = JointModMode::rotate;
                rt.main.twist_y = b.troty;
                rt.board_proc.main.mode = JointModMode::rotate;
                rt.board_proc.main.twist_y = b.troty;
            } else {
                if (rt.main.mode != JointModMode::flex_blend) {
                    // The spin ended: points by how far it turned, the facing where the board points.
                    const float f0_24 = std::fabs(b.troty_cum);
                    if (f0_24 >= 191146.67f) {
                        add_to_trick_list(self, BoardTricks::board_spin, 4000.0f);
                    } else if (f0_24 >= 126520.89f) {
                        add_to_trick_list(self, BoardTricks::board_spin, 2000.0f);
                    } else if (f0_24 >= 49152.0f) {
                        add_to_trick_list(self, BoardTricks::board_spin, 500.0f);
                    }
                    sound_fade(self, rt.spin_sound_id, 120);
                    rt.spin_sound_id = 0;
                    rt.main.mode = JointModMode::flex_blend;
                    rt.board_proc.main.mode = JointModMode::flex_blend;
                    Vec s5 = rt.io->joint_transform(3).v[2];
                    vector_rotate_y(s5, s5, 15109.688f);
                    const StateId nx = rt.next_state;
                    if (!(nx == StateId::target_board_flip || nx == StateId::target_board_hold ||
                          nx == StateId::target_board_trickx)) {
                        Vec up;
                        vector_y_quaternion(up, c.quat_for_control);
                        forward_up_nopitch_to_quaternion(c.quat_for_control, s5, up);
                    }
                    Vec fwd, up;
                    vector_normalize_copy(fwd, c.transv, 1.0f);
                    vector_y_quaternion(up, c.dir_targ);
                    forward_up_nopitch_to_quaternion(c.dir_targ, fwd, up);
                    b.upper_body_rotyv = b.trotyv;
                    b.troty = 0.0f;
                    b.trotyv = 0.0f;
                } else if (c.danger_mode == Symbol("board-spin") &&
                           (board_on_ground_p(self) || time_elapsed(self, b.spin_time, seconds(0.5)))) {
                    target_danger_set(self, Symbol("harmless"), Symbol());
                }
                if (board_on_ground_p(self)) {
                    b.spin_control = 0.0f;
                    b.flip_control = 0.0f;
                    b.unknown_float00 = 0.0f;
                    b.unknown_float01 = 0.0f;
                    b.ride_rot_old = -1.0f;
                }
            }
            do_rotations2(self);
            reverse_conversions(self, c.transv);
            pre_collide_setup(self);
            if (next_state_is(self, StateId::target_board_get_off)) {
                b.cushion_offset = seek(b.cushion_offset, 0.0f, 20480.0f * spf);
            } else if ((c.mod_surface->flags & sflag::air) && !(c.status & cstat::on_surface)) {
                b.cushion_offset =
                    seek(b.cushion_offset,
                         lerp_scale(kTargetBoardBank.cushion, 0.0f, target_height_above_ground(self), 0.0f, 12288.0f),
                         20480.0f * spf);
            } else if (focus_test(self, focus::on_water)) {
                b.cushion_offset = seek(b.cushion_offset, 2048.0f + kTargetBoardBank.cushion, 20480.0f * spf);
            } else {
                b.cushion_offset = seek(b.cushion_offset, kTargetBoardBank.cushion, 8192.0f * spf);
            }
            target_board_collision(self);
            if (c.bend_speed != 0.0f) {
                bend_gravity(self);
            }
            post_flag_setup(self);
        }
    }
    update_rates(self, f30);
    // (profiler bar 'target-post: debug)
    ja_post(self);
    joint_points(self);
    do_target_gspot(self);
    target_powerup_process(self);
    set_time(self, b.board_time);
    target_board_exit_check(self);
    target_board_effect(self);
    if (board_on_ground_p(self)) {
        b.up_vector[1] = b.up_vector[0];
        b.up_vector[0] = c.local_normal;
        if (b.troty_cum == 0.0f) {
            b.ground_on_dir = rt.io->joint_transform(3).v[2];
        }
    }
}

// ---- probe-test: the first three collision spheres swept along a move ----
Symbol probe_test(Target& self, const Vec& move, PatSurface ignore) {
    for (int i = 0; i < 3; i++) {
        CollideQuery q;
        const Vec sph = collision_sphere(static_cast<const Target&>(self), i);
        q.start_pos = sph;
        q.move_dist = move;
        q.radius = sph.w / 2.0f;
        q.collide_with = collision_sphere_collide_with(self, i);
        q.ignore_self = true;
        q.ignore_pat = ignore;
        q.action_mask = caction::solid;
        if (fill_and_probe_using_line_sphere(self, q) >= 0.0f) {
            return q.best_pat.event() == 13 ? Symbol("rail") : kTrue;
        }
    }
    return Symbol();
}

// ---- target-board-compute-edge: follow the rail (*edge-grab-info*) and decide when it ends ----
void target_board_compute_edge(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    EdgeGrabInfo& s4 = edge_grab_info();
    const float spf = seconds_per_frame(self);
    const Vec gp = c.trans;
    const Vec s5 = c.transv;
    if (!edge_grab_info_method_9(s4)) {
        // ("exit update")
        EventMessage e;
        e.message = Symbol("end-mode");
        e.num_params = 1;
        send_event(self, e, true);
        return;
    }
    vector_normalize_copy(b.ride_dir, b.ride_segment, 1.0f);
    if (time_elapsed(self, b.ride_time, seconds(0.2))) {
        b.ride_lock_on = Symbol();
        set_time(self, b.ride_start_time);
        vector_normalize_copy(b.ride_dir, c.transv, 1.0f);
        for (int i = 0; i < 3; i++) {
            vector_reset(b.ride_vertex_old[i]);
        }
    }
    b.ride_rot_abs[1] = b.ride_rot_abs[0];
    {
        const float f30_0 = deg_minus(vector_y_angle(b.ride_dir), b.ride_rot_abs[0]);
        const float f30_1 = f30_0 * lerp_scale(10.0f, 100.0f, std::fabs(f30_0), 1820.4445f, 3640.889f);
        b.ride_rtv_abs = b.ride_rtv_abs + f30_1 * spf;
        b.ride_rtv_abs = 0.9f * b.ride_rtv_abs;
        b.ride_rot_abs[0] = b.ride_rot_abs[0] + b.ride_rtv_abs * spf;
        if (0.0f >= f30_1 * deg_minus(vector_y_angle(b.ride_dir), b.ride_rot_abs[0])) {
            b.ride_rtv_abs = 0.0f;
            b.ride_rot_abs[0] = vector_y_angle(b.ride_dir);
        }
    }
    c.ground_pat.bits = s4.edge_tri_pat;
    vector_normalize(vector_sub(c.edge_grab_edge_dir, s4.world_vertex[1], s4.world_vertex[0]), 1.0f);
    c.edge_grab_across_edge_dir = s4.hanging_matrix_row0;
    if (vector_dot(c.edge_grab_edge_dir, b.ride_dir) < 0.0f) {
        // Riding the edge backwards: turn it round, with its neighbours and their flags.
        Vec t = s4.world_vertex[0];
        s4.world_vertex[0] = s4.world_vertex[1];
        s4.world_vertex[1] = t;
        vector_normalize(vector_sub(c.edge_grab_edge_dir, s4.world_vertex[1], s4.world_vertex[0]), 1.0f);
        t = s4.adjacent_edge_left_vertex;
        s4.adjacent_edge_left_vertex = s4.adjacent_edge_right_vertex;
        s4.adjacent_edge_right_vertex = t;
        const bool v1_69 = (s4.status & 2) != 0;
        if (s4.status & 1) {
            s4.status |= 2;
        } else {
            s4.status &= ~2u;
        }
        if (v1_69) {
            s4.status |= 1;
        } else {
            s4.status &= ~1u;
        }
    }
    for (int i = 0; i < 3; i++) {
        b.ride_vertex_old[i] = b.ride_vertex[i];
    }
    b.ride_segment_old = b.ride_segment;
    b.ride_vertex_length_old = b.ride_vertex_length;
    b.ride_vertex[0] = s4.world_vertex[0];
    b.ride_vertex[1] = s4.world_vertex[1];
    b.ride_vertex[2] = Vec{};
    b.ride_vertex_length = 1;
    if (s4.status & 2) {
        vector_normalize(vector_sub(c.unknown_vector35, s4.adjacent_edge_right_vertex, s4.world_vertex[1]), 1.0f);
        b.ride_vertex[2] = s4.adjacent_edge_right_vertex;
        b.ride_vertex_length += 1;
    } else {
        c.unknown_vector35 = Vec{};
    }
    Vec s4_1;
    {
        int s3 = 0;
        for (;;) {
            b.ride_vertex_index = segment_array_vector_to_index(b.ride_vertex, b.ride_vertex_length, c.trans, 0.0f);
            if (b.ride_vertex_index < 0.0f && !time_elapsed(self, b.ride_time, seconds(0.2)) && s3 == 0) {
                // Behind the new edge: back to last frame's vertices, once.
                s3 += 1;
                for (int i = 0; i < 3; i++) {
                    b.ride_vertex[i] = b.ride_vertex_old[i];
                }
                b.ride_vertex_length = b.ride_vertex_length_old;
                continue;
            }
            break;
        }
    }
    b.ride_vertex_index = std::fmax(0.0f, b.ride_vertex_index);
    b.ride_vertex_base = static_cast<int16_t>(
        segment_array_index_to_vector(b.ride_vertex, b.ride_vertex_length, b.ride_vertex_index, s4_1));
    vector_sub(b.ride_segment, b.ride_vertex[b.ride_vertex_base + 1], b.ride_vertex[b.ride_vertex_base]);
    for (int i = 0; i < 3; i++) {
        b.ride_vertex[i].w = 1.0f;
    }
    if (time_elapsed(self, b.ride_time, seconds(0.2))) {
        b.ride_rot_abs[0] = vector_y_angle(b.ride_segment);
        b.ride_rtv_abs = 0.0f;
    }
    {
        // Pull onto the rail, at most 3 m/s until locked on.
        const float f0_25 = vector_vector_distance(c.trans, s4_1);
        Vec s3_5 = sub(s4_1, c.trans);
        const float f1_9 = 12288.0f * spf;
        if (!b.ride_lock_on && f1_9 < f0_25) {
            vector_normalize(s3_5, f1_9);
        } else {
            b.ride_lock_on = kTrue;
        }
        PatSurface ig;
        ig.bits = kPatNoentityNoboardProbe;
        if (probe_test(self, s3_5, ig)) {
            c.status |= cstat::probe_hit;
        } else {
            move_by_vector(self, s3_5);
        }
    }
    const float f30_2 = b.ride_speed;
    vector_normalize_copy(c.to_target_pt_xz, b.ride_segment, 1.0f);
    vector_normalize_copy(c.transv, b.ride_segment, f30_2);
    vector_rotate_y(b.ride_dir_lean, c.to_target_pt_xz, -4551.1113f * b.ride_lean);
    if (b.ride_vertex_base < b.ride_vertex_length + -1) {
        // The next segment bends too sharply or is too short: the rail ends at this one.
        const Vec& a0 = b.ride_vertex[b.ride_vertex_base + 1];
        int s4_2 = 0;
        const Vec s3_7 = sub(b.ride_vertex[b.ride_vertex_base + 2], a0);
        if (14563.556f < std::fabs(deg_diff(vector_y_angle(s3_7), vector_y_angle(c.to_target_pt_xz)))) {
            s4_2 |= 2;
        }
        if (vector_length(s3_7) < std::fmin(1024.0f, 0.25f * vector_length(b.ride_segment))) {
            s4_2 |= 32;
        }
        if (s4_2 != 0) {
            b.ride_vertex_length += -1;
        }
    }
    int s4_3 = 0;
    if (std::fabs(f30_2) < 16384.0f && !time_elapsed(self, b.ride_start_time, seconds(0.1))) {
        s4_3 |= 1; // too slow
    }
    {
        Vec seg_old;
        vector_normalize_copy(seg_old, b.ride_segment_old, 1.0f);
        bool angle = vector_dot(seg_old, c.to_target_pt_xz) < 0.02f;
        if (!angle) {
            const float f28_2 = deg_diff(b.ride_rot_abs[0], b.ride_rot_abs[1]);
            const float f26_0 = 1820.4445f * b.ride_lean;
            if (0.0f < f26_0 * f28_2 && b.ride_rtv_abs != 0.0f) {
                b.ride_rtv_abs = 0.0f;
                b.ride_rot_abs[0] = vector_y_angle(b.ride_segment);
            }
            // ((format #t "angle ...") returns 0, which is true: the test is the comparison before it)
            angle = b.ride_lock_on &&
                    ((((f26_0 * f28_2 < 0.0f || f26_0 == 0.0f) && 91.022224f < std::fabs(f28_2)) &&
                      163.84f < std::fabs(deg_diff(f26_0, f28_2))) ||
                     lerp_scale(910.2222f, 2002.4889f, std::fabs(f30_2), 8192.0f, 24576.0f) < std::fabs(f26_0));
        }
        if (angle) {
            s4_3 |= 2;
        }
    }
    {
        const int v1_227 = std::min(static_cast<int>(static_cast<int32_t>(b.ride_vertex_index)),
                                    b.ride_vertex_length + -1);
        const float f28_3 = vector_length(sub(b.ride_vertex[v1_227 + 1], b.ride_vertex[v1_227]));
        float f26_1 = 81920.0f;
        uint32_t sv_768 = 8;
        CollideQuery q;
        q.radius = 1228.8f;
        q.collide_with = root_prim_collide_with(self) & ~cspec::crate;
        q.ignore_self = true;
        q.ignore_pat = c.pat_ignore_mask;
        q.action_mask = caction::solid;
        vector_add_float_mul(q.start_pos, c.trans, b.ride_dir, 3276.8f);
        vector_add_float_mul(q.start_pos, q.start_pos, c.dynam->gravity_normal, 5324.8f);
        vector_float_mul(q.move_dist, c.dynam->gravity_normal, -49152.0f);
        if (fill_and_probe_using_line_sphere(self, q) >= 0.0f) {
            f26_1 = c.trans.y - q.best_intersect.y;
            sv_768 = q.best_pat.bits;
        }
        if (static_cast<float>(b.ride_vertex_length) + goal_div(0.0f, f28_3) <
                b.ride_vertex_index + goal_div(b.ride_speed * spf, f28_3) &&
            f28_3 >= 3686.4f &&
            ((f26_1 >= 6963.2f || (f26_1 >= 2048.0f && pat_mode_bits(sv_768) == 0)) &&
             (pat_skip_bits(sv_768) & 8))) {
            // ("end with length ...")
            s4_3 |= 4; // past the end
        }
    }
    {
        const Vec& s3_9 = b.ride_vertex[b.ride_vertex_base];
        const Vec& s3_9n = b.ride_vertex[b.ride_vertex_base + 1];
        if (!(vector_equal(s3_9, b.ride_vertex_old[0]) || vector_equal(s3_9, b.ride_vertex_old[1]) ||
              vector_equal(s3_9, b.ride_vertex_old[2]) || vector_equal(s3_9n, b.ride_vertex_old[0]) ||
              vector_equal(s3_9n, b.ride_vertex_old[1]) || vector_equal(s3_9n, b.ride_vertex_old[2]))) {
            s4_3 |= 8; // a different rail
        }
    }
    if (time_elapsed(self, b.ride_button_time, seconds(0.05))) {
        s4_3 |= 16; // the button let go
    }
    if (vector_length(b.ride_segment) < std::fmin(1024.0f, 0.25f * vector_length(b.ride_segment_old))) {
        s4_3 |= 32; // too short
    }
    if (c.status & cstat::probe_hit) {
        s4_3 |= 64; // blocked
    }
    c.status &= ~cstat::probe_hit;
    if (!time_elapsed(self, b.ride_time, seconds(0.2)) && s4_3 != 0) {
        // (the "exit ..." formats: debug)
        if (s4_3 & 64) {
            b.ride_lock_on = Symbol();
        }
        if ((s4_3 & 1) || ((s4_3 & 2) && std::fabs(f30_2) < 32768.0f)) {
            // Thrown off sideways, to the side Jak leans.
            const Vec& gp1 = c.edge_grab_across_edge_dir;
            Vec a0;
            vector_rotate_y(a0, b.ride_dir, -16384.0f);
            const float f28_4 = b.ride_lean * vector_dot(a0, gp1);
            vector_normalize_copy(c.transv, gp1,
                                  lerp_scale(8192.0f, 32768.0f, std::fabs(b.ride_lean), 0.0f, 1.0f) *
                                      (f28_4 >= 0.0f ? 1.0f : -1.0f));
            c.to_target_pt_xz = gp1;
        } else if (s4_3 & 42) {
            move_to_point(self, gp);
            c.transv = s5;
        }
        if (!(s4_3 & 64)) {
            Vec a1;
            vector_float_mul(a1, c.dynam->gravity_normal, 819.2f);
            move_by_vector(self, a1);
        }
        EventMessage e;
        e.message = Symbol("end-mode");
        e.num_params = 1;
        e.truthy[0] = !(s4_3 & 75);
        send_event(self, e, true);
    }
    vector_rotate_y(c.to_target_pt_xz, c.to_target_pt_xz, b.ride_rot + -2730.6667f * b.ride_lean);
    set_time(self, b.ride_time);
    b.ride_speed = f30_2;
    (void)rt;
    set_time(self, c.last_time_on_surface);
}

// ---- board-ride-add-thrust: the speed along the rail ----
void board_ride_add_thrust(Target& self, const Vec& arg0, float arg1) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    const Surface& cs = *c.current_surface;
    const float spf = seconds_per_frame(self);
    const float f30_0 = b.ride_speed;
    const float f0_4 = lerp_scale(cs.fric, cs.fric / 5.0f, f30_0, 0.0f, cs.transv_max);
    const float f30_1 = seek(f30_0, 0.0f, f0_4 * spf);
    const float f28_0 = vector_dot(b.ride_dir, arg0);
    const float f0_19 =
        f30_1 + (0.0f < f28_0
                     ? f28_0 * lerp_scale(cs.target_speed, 0.0f, b.ride_speed, 0.0f, cs.transv_max) * arg1
                     : 40960.0f * arg1 * f28_0) *
                    spf;
    const float f1_6 = vector_dot(c.dynam->gravity_normal, c.to_target_pt_xz);
    const float f0_20 = f1_6 < 0.0f ? f0_19 + -f1_6 * cs.slope_down_factor * spf
                                    : f0_19 + -f1_6 * cs.slope_up_factor * spf;
    b.ride_speed = std::fmax(0.0f, std::fmin(f0_20, cs.transv_max));
}

// ---- target-board-ride-post: the post while riding a rail ----
void target_board_ride_post(Target& self) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    const float f30 = clock_ratio(self);
    {
        int gp = std::max(1, static_cast<int>(static_cast<int32_t>(self.clock.time_adjust_ratio)));
        update_rates(self, goal_div(f30, static_cast<float>(gp)));
        while (gp != 0) {
            gp += -1;
            const float spf = seconds_per_frame(self);
            c.remaining_ctrl_iterations = gp;
            flag_setup(self);
            c.bend_speed = 32.0f;
            c.bend_target = 1.0f;
            root_prim_action(self) |= caction::check_edge;
            if (c.force_turn_to_strength < 0.0f) {
                c.force_turn_to_strength = 1.0f - target_cpad_stick0_speed(self);
            }
            build_conversions(self, c.transv);
            do_rotations1(self);
            {
                Vec s5{};
                read_pad(self, s5);
                turn_to_vector(self, s5, c.pad_magnitude);
                if (0.0f < c.turn_to_magnitude) {
                    b.ride_pad_vector[0] = c.to_target_pt_xz;
                }
                board_ride_add_thrust(self, s5, c.turn_to_magnitude);
                Vec s5_1;
                vector_rotate_y(s5_1, s5, 16384.0f);
                s5_1.y = 0.0f;
                vector_xz_normalize(s5_1, c.turn_to_magnitude);
                float f0_16 = vector_dot(s5_1, b.ride_dir);
                if (0.0f >= c.turn_to_magnitude) {
                    f0_16 = 0.0f;
                }
                b.ride_lean_targ = f0_16;
                b.ride_leanv = b.ride_leanv + 100.0f * spf * (f0_16 - b.ride_lean);
            }
            b.ride_leanv = 0.95f * b.ride_leanv;
            b.ride_lean_mag = seek(
                b.ride_lean_mag,
                std::fabs(analog_input(static_cast<int>(target_cpad_leftx(self)), 128.0f, 32.0f, 110.0f, 1.0f)),
                2.0f * spf);
            b.ride_lean = b.ride_lean + b.ride_leanv * spf;
            if (b.ride_lean < -1.0f) {
                b.ride_lean = -1.0f;
                b.ride_leanv = 0.0f * b.ride_leanv;
            } else if (1.0f < b.ride_lean) {
                b.ride_lean = 1.0f;
                b.ride_leanv = 0.0f * b.ride_leanv;
            }
            add_gravity(self);
            reverse_conversions(self, c.transv);
            target_board_compute_edge(self);
            do_rotations2(self);
            pre_collide_setup(self);
            b.cushion_offset = seek(b.cushion_offset, 1638.4f, 16384.0f * spf);
            {
                const Vec s5_3 = c.transv;
                const Vec s4_1 = c.trans;
                // (profiler bar 'target: debug)
                {
                    CollideQuery q;
                    q.collide_with = root_prim_collide_with(self);
                    q.ignore_self = true;
                    q.ignore_pat = c.pat_ignore_mask;
                    q.action_mask = caction::solid;
                    fill_cache_integrate_and_collide(self, c.transv, q, meters(1));
                }
                if (root_prim_action(self) & caction::check_edge) {
                    CollideEdgeSpec& spec = collide_edge_board_spec();
                    edge_spec_flags(spec) &= ~edgeflag::two;
                    if (time_elapsed(self, b.ride_time, seconds(0.2))) {
                        edge_spec_flags(spec) &= ~edgeflag::one;
                        edge_spec_set_touching_segment(spec, nullptr);
                    } else {
                        edge_spec_flags(spec) |= edgeflag::one;
                        for (int i = 0; i < 2; i++) {
                            b.ride_touch_segment[i] = b.ride_vertex[b.ride_vertex_base + i];
                        }
                        edge_spec_set_touching_segment(spec, b.ride_touch_segment);
                    }
                    do_edge_grabs(self, spec);
                }
                if (focus_test(self, focus::rail)) {
                    // Back onto the rail where the speed puts the board along it.
                    Vec s3_2;
                    const float f28_1 = std::fmax(
                        0.0f, segment_array_vector_to_index(b.ride_vertex, b.ride_vertex_length, s4_1,
                                                            b.ride_speed * spf));
                    b.ride_vertex_base2 = static_cast<int16_t>(
                        segment_array_index_to_vector(b.ride_vertex, b.ride_vertex_length, f28_1, s3_2));
                    b.ride_vertex_index2 = f28_1;
                    PatSurface ig;
                    ig.bits = kPatNoentityNoboardProbe;
                    if (probe_test(self, sub(s3_2, c.trans), ig)) {
                        c.status |= cstat::probe_hit;
                    }
                    if (b.ride_lock_on && !(c.status & cstat::probe_hit)) {
                        move_to_point(self, s3_2);
                    }
                    c.transv = s5_3;
                }
                target_board_physics(self, s5_3);
            }
            bend_gravity(self);
            post_flag_setup(self);
        }
    }
    update_rates(self, f30);
    // (profiler bar 'target-post: debug)
    ja_post(self);
    joint_points(self);
    do_target_gspot(self);
    vector_cross(c.local_normal, c.edge_grab_across_edge_dir, c.edge_grab_edge_dir);
    if (vector_dot(c.local_normal, c.dynam->gravity_normal) < 0.0f) {
        vector_negate(c.local_normal, c.local_normal);
    }
    c.gspot_normal = c.local_normal;
    target_powerup_process(self);
    target_board_exit_check(self);
    target_board_effect(self);
    set_time(self, b.board_time);
}

// ---- target-board-post ----
void target_board_post(Target& self) { target_board_real_post(self); }

// ---- target-board-center-anim ----
void target_board_center_anim(Target& self) { self.board->turn_anim_frame = 0.0f; }

// ---- target-board-turn-anim: the lean animation on channel `chan`, and the duck ----
void target_board_turn_anim(Target& self, int arg0) {
    BoardInfo& b = *self.board;
    ControlInfo& c = self.control;
    const float tf = kTargetBoardBank.turn_frames;
    const float f30 = clock_ratio(self);
    int s5 = std::max(1, static_cast<int>(static_cast<int32_t>(self.clock.time_adjust_ratio)));
    update_rates(self, goal_div(f30, static_cast<float>(s5)));
    while (s5 != 0) {
        s5 += -1;
        const float spf = seconds_per_frame(self);
        c.remaining_ctrl_iterations = s5;
        b.turn_anim_targ = std::fmax(-10.0f, std::fmin(10.0f, b.turn_anim_targ));
        // ((or ...) on the target and frame: computed and dropped)
        b.turn_anim_vel =
            b.turn_anim_vel +
            (b.turn_anim_targ - b.turn_anim_frame) *
                lerp_scale(20.0f, std::fabs(b.turn_anim_frame) < std::fabs(b.turn_anim_targ) ? 30.0f : 60.0f,
                           c.ctrl_xz_vel, 0.0f, 0.3f * b.transv_max) *
                spf;
        b.turn_anim_vel = std::fmax(
            -100.0f,
            std::fmin(100.0f, b.turn_anim_vel * lerp_scale(0.96f, 0.9f, c.ctrl_xz_vel, 0.0f, 0.3f * b.transv_max)));
        b.turn_anim_frame = b.turn_anim_frame + b.turn_anim_vel * spf;
        b.turn_anim_frame = std::fmax(std::fmin(b.turn_anim_frame, tf), -tf);
        if (b.turn_anim_frame >= tf && b.turn_anim_vel >= 0.0f) {
            b.turn_anim_vel = 0.0f;
        } else if (-tf >= b.turn_anim_frame && 0.0f >= b.turn_anim_vel) {
            b.turn_anim_vel = 0.0f;
        }
        {
            const float wobble =
                sin(145.63556f * static_cast<float>(current_time(self) - self.state_time)) *
                lerp_scale(1.0f, 2.0f, std::fabs(b.turn_anim_frame), 0.0f, tf);
            const float af = std::fmax(
                std::fmin(std::fmax(std::fmin(b.turn_anim_frame, -1.0f + tf), -(-1.0f + tf)) + wobble, tf), -tf);
            ja(self, Ja().chan(arg0).num_func(NumFunc::identity).frame_num(ja_aframe(skel(self), af, arg0)));
        }
        {
            const float f0_59 = std::fmin(
                (c.status & cstat::on_surface) ? seek(b.turn_anim_duck, 0.0f, spf) : b.turn_anim_duck,
                lerp_scale(0.0f, -1.0f, static_cast<float>(current_time(self) - c.last_time_on_surface), 30.0f,
                           120.0f));
            b.turn_anim_duck = b.turn_anim_duck + b.turn_anim_duck_vel * spf;
            if (b.turn_anim_duck < f0_59) {
                b.turn_anim_duck = f0_59;
                b.turn_anim_duck_vel = 0.0f;
            } else if (1.0f < b.turn_anim_duck) {
                b.turn_anim_duck = 1.0f;
                b.turn_anim_duck_vel = 0.0f;
            }
        }
    }
    update_rates(self, f30);
}

} // namespace jak
