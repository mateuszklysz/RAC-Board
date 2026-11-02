// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's target movement core, from OpenGOAL's goal_src/jak2/engine/target/logic-target.gc,
// target-util.gc, target.gc and target-handler.gc, with *standard-dynamics* (physics/dynamics-h.gc)
// and update-rates! (ps2/timer.gc) (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL
// Team; THIRD_PARTY.md). Each function says which GOAL one it follows. Parts of Jak II the mod has
// no counterpart for (eco, dark Jak, the gun, the sidekick, levels, hair, IK, debug drawing and
// cheats) are left out with a comment where they were. GOAL's float `/` by a variable is goal_div
// (math.hpp): a zero divisor gives FLT_MAX, not inf or NaN, which the code relies on (a zero vector
// normalizes to zero).
#include "jak/movement.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "board/board_from_core.hpp"
#include "jak/collide/from_core.hpp"
#include "jak/math/core.hpp"
#include "jak/math/math.hpp"
#include "jak/surface.hpp"

namespace jak {

namespace {

constexpr Symbol kHarmless{"harmless"};

TimeFrame now(const Target& self) { return current_time(self); }
float spf(const Target& self) { return seconds_per_frame(self); }

// (the int f): truncation toward zero.
int to_int(float f) { return static_cast<int>(f); }

float xz_len(const Vec& v) { return std::sqrt(v.x * v.x + v.z * v.z); }

// sphere<-vector+r!
void set_sphere(Vec& s, const Vec& v, float r) { s = Vec{v.x, v.y, v.z, r}; }

bool next_state_is(const Target& self, std::initializer_list<const char*> names) {
    const Symbol n = next_state_name(self);
    if (!n) {
        return false;
    }
    for (const char* name : names) {
        if (n == Symbol(name)) {
            return true;
        }
    }
    return false;
}

} // namespace

// *standard-dynamics*
const Dynamics standard_dynamics = [] {
    Dynamics d{};
    d.name = Symbol("standard");
    d.gravity_max = meters(40);
    d.gravity_length = meters(60);
    d.gravity = Vec{0.0f, 245760.0f, 0.0f, 1.0f};
    d.gravity_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
    d.walk_distance = meters(2);
    d.run_distance = meters(5);
    return d;
}();

// *TARGET-bank*
const TargetBank target_bank = [] {
    TargetBank b{};
    b.jump_collide_offset = meters(0.7);
    b.jump_height_min = meters(1.01);
    b.jump_height_max = meters(3.5);
    b.double_jump_height_min = meters(1);
    b.double_jump_height_max = meters(2.5);
    b.flip_jump_height_min = meters(5);
    b.flip_jump_height_max = meters(7);
    b.duck_jump_height_min = meters(7);
    b.duck_jump_height_max = meters(7);
    b.flop_jump_height_min = meters(5);
    b.flop_jump_height_max = meters(7);
    b.attack_jump_height_min = meters(5);
    b.attack_jump_height_max = meters(6.5);
    b.edge_grab_jump_height_min = meters(1.7);
    b.edge_grab_jump_height_max = meters(1.7);
    b.swim_jump_height_min = meters(5);
    b.swim_jump_height_max = meters(5);
    b.tube_jump_height_min = meters(1.75);
    b.tube_jump_height_max = meters(2.5);
    b.carry_jump_height_min = meters(1.75);
    b.carry_jump_height_max = meters(2);
    b.mech_jump_height_min = meters(1.5);
    b.mech_jump_height_max = meters(1.5);
    b.mech_carry_jump_height_min = meters(2.5);
    b.mech_carry_jump_height_max = meters(3);
    b.indax_jump_height_min = meters(1.01);
    b.indax_jump_height_max = meters(3.5);
    b.indax_double_jump_height_min = meters(1);
    b.indax_double_jump_height_max = meters(2.5);
    b.roll_duration = 0x96;
    b.roll_jump_pre_window = 0x12c;
    b.roll_jump_post_window = 0x1e;
    b.roll_timeout = 0; // not given in *TARGET-bank*
    b.roll_speed_min = meters(11.5);
    b.roll_speed_inc = meters(1.5);
    b.roll_flip_duration = 0xd2;
    b.roll_flip_height = meters(3.52);
    b.roll_flip_dist = meters(17.3);
    b.roll_flip_art_height = meters(3.2969);
    b.roll_flip_art_dist = meters(12.5);
    b.duck_slide_distance = meters(5.75);
    b.fall_far = meters(30);
    b.fall_far_inc = meters(20);
    b.attack_timeout = 0x5a;
    b.ground_timeout = 0x3c;
    b.slide_down_timeout = 0x3c;
    b.fall_timeout = 0x12c;
    b.fall_stumble_threshold = meters(39.9);
    b.yellow_projectile_speed = meters(60);
    b.hit_invulnerable_timeout = 0x12c;
    b.same_attack_invulnerable_timeout = 0x258;
    b.run_cycle_length = 60.0f;
    b.walk_cycle_dist = meters(2.8);
    b.walk_up_cycle_dist = meters(2.8);
    b.walk_down_cycle_dist = meters(2.8);
    b.walk_side_cycle_dist = meters(2.8);
    b.run_cycle_dist = meters(6.25);
    b.run_up_cycle_dist = meters(5);
    b.run_down_cycle_dist = meters(5);
    b.run_side_cycle_dist = meters(6.25);
    b.run_wall_cycle_dist = meters(2.8);
    b.duck_walk_cycle_dist = meters(3.25);
    b.wade_shallow_walk_cycle_dist = meters(6);
    b.wade_deep_walk_cycle_dist = meters(6);
    b.mech_walk_cycle_dist = meters(4);
    b.mech_run_cycle_dist = meters(8);
    b.smack_surface_dist = meters(1.25);
    b.smack_surface_height = 0.0f; // not given in *TARGET-bank*
    b.min_dive_depth = meters(2);
    b.root_radius = meters(2.2);
    b.root_offset = Vec{0.0f, 4915.2f, 0.0f, 1.0f};
    b.body_radius = meters(0.7);
    b.edge_radius = meters(0.35);
    b.edge_offset = Vec{0.0f, 4915.2f, 4096.0f, 1.0f};
    b.edge_grab_height_off_ground = meters(2.3);
    b.head_radius = meters(0.7);
    b.head_height = meters(1.4);
    b.head_offset = Vec{0.0f, 4915.2f, 0.0f, 1.0f};
    b.spin_radius = meters(2.2);
    b.spin_offset = Vec{0.0f, 6553.6f, 0.0f, 1.0f};
    b.duck_spin_radius = meters(1.2);
    b.duck_spin_offset = Vec{0.0f, 4096.0f, 0.0f, 1.0f};
    b.punch_radius = meters(1.3);
    b.punch_offset = Vec{0.0f, 5324.8f, 0.0f, 1.0f};
    b.uppercut_radius = meters(1);
    b.uppercut0_offset = Vec{0.0f, 3276.8f, 0.0f, 1.0f};
    b.uppercut1_offset = Vec{0.0f, 9011.2f, 0.0f, 1.0f};
    b.flop_radius = meters(1.4);
    b.flop0_offset = Vec{0.0f, 3276.8f, 0.0f, 1.0f};
    b.flop1_offset = Vec{0.0f, 9011.2f, 0.0f, 1.0f};
    b.stuck_time = seconds(0.3);
    b.stuck_timeout = seconds(2);
    b.stuck_distance = meters(0.05);
    b.tongue_pull_speed_min = 0.15f;
    b.tongue_pull_speed_max = 0.22f;
    b.yellow_attack_timeout = 0x3c;
    b.fall_height = meters(1);
    b.mech_jump_thrust_fuel = 900.0f;
    b.strafe_jump_pre_window = 0x96;
    b.dark_jump_height_min = meters(20.5);
    b.dark_jump_height_max = meters(20.5);
    return b;
}();

// ---- set-up ----

// init-target (logic-target.gc), the control's movement fields. The shape itself (prims, collide
// specs, pat-ignore-mask) is the collision part's.
void movement_init(Target& self, const Vec& trans) {
    ControlInfo& c = self.control;
    self.dynam_storage = standard_dynamics; // (copy *standard-dynamics* 'process)
    c.dynam = &self.dynam_storage;
    c.max_iteration_count = 8;
    move_to_point(self, trans);
    c.camera_pos = trans;
    c.cpad = &self.pad;
    c.current_surface = &self.current_surface_storage;
    c.current_surface->name = Symbol("current");
    c.current_surface->active_hook = nullptr;
    c.current_surface->touch_hook = nullptr;
    reset_target_state(self, true);
    c.last_trans_any_surf = c.trans;
    c.last_trans_any_surf.y += -819200.0f;
}

// reset-target-state (logic-target.gc). The camera mode, handles, yellow eco and the pad's buzz are
// left out.
void reset_target_state(Target& self, bool full) {
    ControlInfo& c = self.control;
    if (full) {
        c.scale = Vec{1.0f, 1.0f, 1.0f, 1.0f};
        quaternion_identity(c.quat);
        quaternion_identity(c.quat_for_control);
        quaternion_identity(c.dir_targ);
        c.transv = Vec{};
        c.camera_pos = c.trans;
    }
    target_exit(self);
    target_timed_invulnerable_off(self, 0);
    target_timed_invulnerable_off(self, 2);
    c.status = 0;
    c.standard_dynamics = const_cast<Dynamics*>(&standard_dynamics);
    c.surf = standard_ground_surface();
    c.bent_gravity_normal = c.standard_dynamics->gravity_normal;
    quaternion_identity(c.override_quat);
    c.override_quat_alpha = 0.0f;
    c.last_time_on_surface = now(self);
    c.bend_amount = 0.0f;
    c.bend_speed = 32.0f;
}

// update-rates! (timer.gc) on the target's clock, with *display*'s time-factor and dog-ratio.
void update_rates(Target& self, float ratio) {
    self.clock_ratio = ratio;
    const float f0 = (self.display_time_factor * self.display_dog_ratio) * ratio;
    self.clock.time_adjust_ratio = 0.2f * f0;
    self.clock.seconds_per_frame = 0.016666668f * self.clock.time_adjust_ratio;
    self.clock.frames_per_second =
        self.clock.time_adjust_ratio == 0.0f ? 0.0f : 60.0f * goal_div(1.0f, self.clock.time_adjust_ratio);
}

// ---- logic-target.gc ----

// build-conversions: current-surface = mod-surface x surf, then the control frame (c-R-w: forward
// from quat-for-control, up the local normal) and the velocity in it.
void build_conversions(Target& self, const Vec& v) {
    ControlInfo& c = self.control;
    if (c.prev_surf != c.surf) {
        c.prev_surf = c.surf;
        c.time_of_last_surface_change = now(self);
    }
    surface_mult(self, *c.current_surface, *c.mod_surface, *c.surf);
    // Blue eco and dark Jak's speed-ups: neither exists in the mod.
    const float f0 = self.speed_mult;
    c.current_surface->target_speed = c.current_surface->target_speed * f0;
    c.current_surface->transv_max = c.current_surface->transv_max * f0;
    // PC_PORT turbo-board cheat: off (cheats are left out).
    Vec s5{};
    vector_z_quaternion(s5, c.quat_for_control);
    if (c.mod_surface->flags & sflag::xz_local) {
        vector_flatten(s5, s5, c.dynam->gravity_normal);
        vector_normalize(s5, 1.0f);
    }
    forward_up_nopitch_to_inv_matrix(c.c_R_w, s5, c.local_normal);
    matrix_transpose(c.w_R_c, c.c_R_w);
    vector_matrix_mul(c.transv_ctrl, v, c.w_R_c);
    c.last_gravity_normal = c.gravity_normal;
    c.gravity_normal = c.dynam->gravity_normal;
}

// vector-turn-to: face along dir at once.
void vector_turn_to(Target& self, const Vec& dir) {
    ControlInfo& c = self.control;
    Vec f, up;
    vector_normalize_copy(f, dir, 1.0f);
    vector_y_quaternion(up, c.dir_targ);
    forward_up_nopitch_to_quaternion(c.dir_targ, f, up);
    c.quat_for_control = c.dir_targ; // set-quaternion! on control-info's (get-quaternion)
    build_conversions(self, c.transv);
}

// reverse-conversions: the control-frame velocity back to world.
void reverse_conversions(Target& self, Vec& out) {
    ControlInfo& c = self.control;
    const Vec& v = c.transv_ctrl;
    c.ctrl_xz_vel = std::sqrt(v.x * v.x + v.z * v.z);
    vector_matrix_mul(out, c.transv_ctrl, c.c_R_w);
    c.old_status = c.status;
}

// read-pad: the stick's direction in the camera's frame.
Vec& read_pad(Target& self, Vec& out) {
    ControlInfo& c = self.control;
    if (c.time_of_last_pad_read != self.real_frame_counter) {
        c.last_pad_xz_dir = c.pad_xz_dir;
        c.last_pad_magnitude = c.pad_magnitude;
        c.time_of_last_pad_read = self.real_frame_counter;
    }
    out.x = sin(c.cpad->stick0_dir);
    out.y = 0.0f;
    out.z = cos(c.cpad->stick0_dir);
    out.w = 0.0f;
    c.pad_xz_dir = out;
    c.pad_magnitude = c.cpad->stick0_speed;
    if (c.update_cam_R_w_start_time < now(self)) {
        const Mat& m = self.camera_local_to_world; // (matrix-local->world #t #f)
        c.cam_R_w.v[0] = m.v[0];
        c.cam_R_w.v[1] = m.v[1];
        c.cam_R_w.v[2] = m.v[2];
        c.cam_R_w.v[3] = m.v[3];
    }
    return vector_matrix_mul(out, out, c.cam_R_w);
}

// warp-vector-into-surface!: rotate in from m's up to the surface normal.
Vec& warp_vector_into_surface(Vec& out, const Vec& in, const Vec& normal, const Mat& m) {
    Mat r{};
    matrix_from_two_vectors(r, m.v[1], normal);
    return vector_matrix_mul(out, in, r);
}

// vector<-pad-in-surface!
Vec& vector_pad_in_surface(Target& self, Vec& out, bool scale) {
    Vec p{};
    read_pad(self, p);
    warp_vector_into_surface(out, p, self.control.local_normal, self.control.cam_R_w);
    if (scale) {
        vector_float_mul(out, out, self.control.pad_magnitude);
    }
    return out;
}

// local-pad-angle
float local_pad_angle(Target& self) {
    Vec p{}, z{};
    vector_pad_in_surface(self, p, false);
    vector_normalize(p, 1.0f);
    vector_z_quaternion(z, self.control.quat_for_control);
    return vector_dot(p, z);
}

// turn-around?. GOAL keeps the time the surface last forbade turning around in a static array of
// one, shared by every call; and the history shift writes one entry past transv-history (Target).
bool turn_around_p(Target& self) {
    static TimeFrame no_turn_time = 0;
    ControlInfo& c = self.control;
    Vec p{};
    vector_pad_in_surface(self, p, false);
    vector_normalize(p, 1.0f);
    Vec tv = c.transv;
    vector_normalize(tv, 1.0f);
    const float f0 = vector_dot(p, tv);
    const Vec h15 = c.transv_history[14];
    for (int i = 13; i >= 0; i--) {
        c.transv_history[i + 1] = c.transv_history[i];
    }
    c.average_xz_vel = h15.x;
    std::memcpy(&c.idx_of_fastest_xz_vel, &h15.y, 4);
    c.hand_to_edge_dist = h15.z;
    self.transv_history_15_w = h15.w;
    c.transv_history[0] = c.transv;
    c.transv_history[0].w = c.ctrl_xz_vel;
    float f2 = -1000.0f, f1 = 0.0f;
    int a0 = -1;
    for (int i = 0; i < 16; i++) {
        const float w = i < 15 ? c.transv_history[i].w : self.transv_history_15_w;
        f1 += w;
        if (f2 < w) {
            a0 = i;
            f2 = w;
        }
    }
    const float avg = f1 / 16.0f;
    c.idx_of_fastest_xz_vel = a0;
    c.average_xz_vel = avg;
    if (c.current_surface->flags & sflag::no_turn_around) {
        no_turn_time = now(self);
    }
    return static_cast<uint64_t>(now(self) - no_turn_time) >= 300u && f0 < 0.0f &&
           32768.0f < avg && 0.7f < c.pad_magnitude &&
           time_elapsed(self, c.last_time_touching_actor, seconds(0.3)) &&
           time_elapsed(self, c.time_of_last_lc, seconds(0.3)) && (c.status & cstat::on_surface) &&
           0.7f < c.surface_angle;
}

// target-log-trans
void target_log_trans(Target& self) {
    ControlInfo& c = self.control;
    const int i = c.trans_log_idx;
    c.trans_log_times[i] = now(self);
    c.trans_log_trans[i] = c.trans;
    c.trans_log_idx = (i + 1) & 127;
}

// target-move-dist: the mean distance from the mean position over the logged positions younger than
// `window`. The walk back steps by the shrinking counter, not by one, as GOAL's does.
float target_move_dist(Target& self, TimeFrame window) {
    ControlInfo& c = self.control;
    Vec s5{};
    float f30 = 0.0f;
    int gp = 0;
    int v1 = 127;
    int a1 = (c.trans_log_idx + v1) & 127;
    while (!time_elapsed(self, c.trans_log_times[a1], window) && v1 > 0) {
        vector_add(s5, s5, c.trans_log_trans[a1]);
        gp++;
        v1--;
        a1 = (a1 + v1) & 127;
    }
    vector_float_div(s5, s5, static_cast<float>(gp));
    for (int s3 = 128 - gp; s3 < 128; s3++) {
        f30 += vector_vector_distance(s5, c.trans_log_trans[(c.trans_log_idx + s3) & 127]);
    }
    return goal_div(f30, static_cast<float>(gp));
}

// turn-to-vector: where the target wants to go (turn-to-target, target-transv in the control frame,
// turn-to-angle and turn-to-magnitude).
void turn_to_vector(Target& self, const Vec& dir, float magnitude) {
    ControlInfo& c = self.control;
    Vec gp{};
    warp_vector_into_surface(gp, dir, c.local_normal, c.cam_R_w);
    c.last_turn_to_magnitude = c.turn_to_magnitude;
    c.last_turn_to_angle = c.turn_to_angle;
    c.last_to_target_pt_xz = c.to_target_pt_xz;
    c.last_turn_to_target = c.turn_to_target;
    vector_float_mul(c.turn_to_target, gp, magnitude);
    if (0.0f < magnitude) {
        const Vec up{0.0f, 1.0f, 0.0f, 1.0f}; // *up-vector*
        warp_vector_into_surface(c.to_target_pt_xz, dir, up, c.cam_R_w);
    }
    // GOAL's loop copies forward, so every entry ends up equal to the old first one; its last write
    // (entry 7) lands in padding after the array.
    for (int i = 0; i < 6; i++) {
        c.turn_history_ctrl[i + 1] = c.turn_history_ctrl[i];
    }
    c.turn_history_ctrl[0] = vector_matrix_mul(gp, gp, c.w_R_c);
    const float a = atan(gp.x, gp.z);
    c.turn_to_magnitude = magnitude;
    c.turn_to_angle = a;
    const float f0 = magnitude * c.current_surface->target_speed;
    c.target_transv = vector_normalize(gp, f0);
    // *display-target-marks* debug sphere left out.
}

// target-bend-vel-turn
Vec& target_bend_vel_turn(Target& self, Vec& v) {
    const Surface& s = *self.control.current_surface;
    if (s.vel_turn == 0.0f) {
    } else if (s.vel_turn < 0.0f) {
        const float f0 = std::sqrt(v.x * v.x + v.z * v.z);
        v = Vec{0.0f, v.y, f0, 1.0f};
    } else {
        const float f0 = atan(v.x, v.z);
        vector_rotate_y(v, v,
                        std::fmin((0.03f * self.clock.time_adjust_ratio) * -f0, s.vel_turn * spf(self)));
    }
    return v;
}

// target-add-slide-factor
Vec& target_add_slide_factor(Target& self, Vec& v) {
    ControlInfo& c = self.control;
    const Surface& s = *c.current_surface;
    Vec ng{}, a1{}, s4{}, s5{};
    vector_negate(ng, c.dynam->gravity_normal);
    ng.w = 1.0f;
    vector_flatten(a1, ng, c.local_normal);
    vector_matrix_mul(s4, a1, c.w_R_c);
    vector_normalize_copy(s5, s4, 1.0f);
    vector_add_float_mul(v, v, s4, s.slide_factor);
    const Vec& tangent = c.low_coverage_tangent;
    if (!((1.0f - s.slope_up_traction) == 0.0f || vector_length(s4) < 0.1f)) {
        Vec a0{}, v1{}, t{};
        vector_matrix_mul(a0, tangent, c.w_R_c);
        float f0 = vector_dot(a0, v);
        vector_sub(v1, v, vector_float_mul(t, a0, f0));
        const float f1 = vector_length(v1);
        const float f2 = f1;
        if (f0 < 0.0f) {
            f0 = f0 * s.slope_up_traction;
        }
        const float f06 = f0 + s.slope_down_factor;
        Vec p{}, q{};
        vector_float_mul(p, a0, f06);
        vector_float_mul(q, v1, goal_div(f1, f2));
        vector_add(v, p, q);
    }
    Vec d = v;
    vector_xz_normalize(d, 1.0f);
    d.y = 0.0f;
    const float f0 = vector_dot(s5, d);
    const float f1 = std::sqrt(v.x * v.x + v.z * v.z);
    if (f0 >= 0.0f) {
        vector_xz_normalize(v, std::fmax(0.0f, f1 + f0 * s.slope_down_factor));
    } else {
        vector_xz_normalize(v, std::fmax(0.0f, f1 - (-f0) * s.slope_up_factor));
    }
    return v;
}

// add-thrust: the target's own thrust (on foot); the board has its own (board-add-thrust).
void add_thrust(Target& self) {
    ControlInfo& c = self.control;
    const Surface& s = *c.current_surface;
    Vec& s5 = c.target_transv;
    Vec& gp = c.transv_ctrl;
    const Vec s4 = c.target_transv;
    target_bend_vel_turn(self, gp);
    target_add_slide_factor(self, s5);
    Vec s3 = s5;
    vector_xz_normalize(s3, 1.0f);
    s3.y = 0.0f;
    const float f0 = s3.z;
    const float f1 = xz_len(gp);
    float f30;
    if (f1 >= xz_len(s5)) {
        f30 = s.fric * std::fmax(1.0f, goal_div(xz_len(gp), s.nonlin_fric_dist));
    } else if (f0 >= 0.0f) {
        f30 = f0 * s.seek0 + (1.0f - f0) * s.seek90;
    } else {
        f30 = std::fabs(f0) * s.seek180 + (1.0f + f0) * s.seek90;
    }
    Vec& s2 = c.btransv;
    Vec b{};
    vector_matrix_mul(b, s2, c.w_R_c);
    for (int i = 0; i < 3; i++) {
        if (s4[i] < 0.0f) {
            b[i] = std::fmax(std::fmin(0.0f, b[i]), s4[i]);
        } else {
            b[i] = std::fmax(0.0f, std::fmin(b[i], s4[i]));
        }
    }
    if (0.2f < c.blocked_factor) {
        vector_seek(b, s4, 122880.0f * spf(self));
    }
    vector_matrix_mul(s2, b, c.c_R_w);
    const float f28 = vector_vector_xz_distance(b, s4);
    if (!(c.status & cstat::touch_surface) && xz_len(gp) < xz_len(b)) {
        float f37 = lerp_scale(163840.0f, 0.0f, f28, 0.0f, 20480.0f);
        if (f28 < 20480.0f && s4.z < 0.0f) {
            f37 = 2.0f * f37;
        }
        f30 = f30 + f37;
    }
    if (!(c.status & cstat::touch_wall) && (c.old_status & cstat::touch_wall) &&
        (c.mod_surface->flags & sflag::air) && 0.0f < gp.y) {
        Vec d{};
        vector_sub(d, prim_world_sphere(collision_sphere(self, 1)), c.wall_contact_pt);
        if (0.0f < vector_dot(c.dynam->gravity_normal, d)) {
            c.time_of_last_clear_wall_in_jump = now(self);
        }
    }
    if (!time_elapsed(self, c.time_of_last_clear_wall_in_jump, seconds(0.2))) {
        f30 = 204800.0f + f30;
    }
    if (!(c.status & cstat::touch_wall) && (c.old_status & cstat::touch_wall) &&
        (c.mod_surface->flags & sflag::air) && 0.0f < gp.y && gp.z < 0.0f) {
        gp.z = 0.0f;
    }
    Vec d{};
    vector_sub(d, s5, gp);
    const float f45 = f30 * spf(self);
    d.y = 0.0f;
    if (f45 < xz_len(d)) {
        vector_xz_normalize(d, f45);
    }
    vector_add(gp, gp, d);
    c.velocity_after_thrust = vector_length(gp);
    // *display-target-marks* debug spheres left out.
}

// add-gravity: compute-acc-due-to-gravity (collide-shape.gc) then the fall speed capped at
// gravity-max.
void add_gravity(Target& self) {
    ControlInfo& c = self.control;
    {
        Vec s4{}, refl{}, acc{}, gp{};
        vector_negate(s4, c.dynam->gravity);
        s4.w = 1.0f;
        vector_reflect_flat(refl, s4, c.local_normal);
        vector_sub_float_mul(acc, s4, refl,
                             (c.status & cstat::on_surface) ? c.current_surface->slip_factor : 0.0f);
        vector_matrix_mul(gp, acc, c.w_R_c);
        vector_v_inc(c.transv_ctrl, gp, spf(self));
    }
    Vec a0{}, v1{}, t{};
    vector_matrix_mul(a0, c.dynam->gravity_normal, c.w_R_c);
    float f0 = vector_dot(a0, c.transv_ctrl);
    vector_sub(v1, c.transv_ctrl, vector_float_mul(t, a0, f0));
    float f1 = vector_length(v1);
    const float f2 = f1;
    if (f1 < 0.00004096f) {
        f1 = 0.0f;
    }
    if (f0 < -c.dynam->gravity_max) {
        f0 = -c.dynam->gravity_max;
    }
    vector_float_mul(c.transv_ctrl, a0, f0);
    vector_float_mul(v1, v1, goal_div(f1, f2));
    vector_add(c.transv_ctrl, c.transv_ctrl, v1);
}

// target-compute-slopes
void target_compute_slopes(Target& self, const Vec& up) {
    ControlInfo& c = self.control;
    Vec gp{}, s4{}, s5{};
    Mat s3{};
    vector_y_quaternion(s4, c.quat_for_control);
    vector_flatten(s5, c.dynam->gravity_normal, c.local_normal);
    forward_up_nopitch_to_inv_matrix(s3, c.c_R_w.v[2], up);
    gp = Vec{0.0f, 0.0f, 1.0f, 1.0f};
    vector_matrix_mul(gp, gp, s3);
    c.surface_slope_z = -vector_dot(c.surface_normal, gp);
    c.local_slope_z = -vector_dot(c.local_normal, gp);
    c.gspot_slope_z = -vector_dot(c.gspot_normal, gp);
    c.ctrl_slope_z = -vector_dot(s4, gp);
    gp = Vec{1.0f, 0.0f, 0.0f, 1.0f};
    vector_matrix_mul(gp, gp, s3);
    c.surface_slope_x = -vector_dot(c.surface_normal, gp);
    c.local_slope_x = -vector_dot(c.local_normal, gp);
    c.gspot_slope_x = -vector_dot(c.gspot_normal, gp);
    c.ctrl_slope_x = -vector_dot(s4, gp);
    gp = Vec{1.0f, 0.0f, 0.0f, 1.0f};
    vector_matrix_mul(gp, gp, c.c_R_w);
    c.ctrl_slope_heading = vector_dot(gp, s5);
}

// do-rotations1: tilt quat-for-control's up toward dir-targ's.
void do_rotations1(Target& self) {
    ControlInfo& c = self.control;
    rotate_toward_orientation(c.quat_for_control, c.dir_targ, 0.0f, c.current_surface->tiltv, 150,
                              to_int(c.current_surface->tiltvf), 0.0f, spf(self));
}

// do-rotations2: turn dir-targ toward the stick (or the velocity), then quat-for-control toward
// dir-targ, then the slopes.
void do_rotations2(Target& self) {
    ControlInfo& c = self.control;
    const Surface& s = *c.current_surface;
    Vec gp{};
    vector_z_quaternion(gp, c.dir_targ);
    const Vec* s5;
    if (sflag::turn_to_alt & s.flags) {
        s5 = &c.turn_to_alt_heading;
    } else if ((!((c.status | c.old_status) & (cstat::on_surface | cstat::touch_surface)) ||
                !time_elapsed(self, c.last_time_touching_actor, seconds(0.5)) ||
                !next_state_is(self, {"target-walk", "target-gun-walk"}) ||
                !time_elapsed(self, self.state_time, seconds(0.5)) ||
                !time_elapsed(self, c.time_of_last_lc, seconds(0.5)) ||
                (s.flags & sflag::turn_to_pad) || c.force_turn_to_strength != 0.0f) &&
               !(s.flags & sflag::turn_to_vel)) {
        s5 = &c.to_target_pt_xz;
    } else {
        s5 = &c.transv;
    }
    {
        const Vec& s3 = c.bent_gravity_normal;
        Quat s4{0, 0, 0, 0}, s31{0, 0, 0, 0};
        forward_up_nopitch_to_quaternion(s4, gp, s3);
        forward_up_nopitch_to_quaternion(s31, *s5, s3);
        const float f0 = acos(vector_dot(gp, *s5));
        const float f1 = s.turnvv * spf(self);
        float t;
        if ((0.0f >= c.turn_to_magnitude || now(self) < c.turn_lockout_end_time) &&
            !(s.flags & sflag::turn_when_centered)) {
            t = 0.0f;
        } else if (f0 < f1) {
            t = 1.0f;
        } else {
            t = goal_div(f1, f0);
        }
        quaternion_slerp(c.dir_targ, s4, s31, t);
    }
    // *display-target-marks* debug vectors left out.
    rotate_toward_orientation(c.quat_for_control, c.dir_targ, s.turnv, 0.0f, to_int(s.turnvf), 150,
                              c.turn_go_the_long_way, spf(self));
    if (c.turn_go_the_long_way != 0.0f &&
        std::fabs(deg_diff(quaternion_y_angle(c.quat_for_control), quaternion_y_angle(c.dir_targ))) <
            182.04445f) {
        c.turn_go_the_long_way = 0.0f;
    }
    quaternion_slerp(c.quat, c.quat_for_control, c.override_quat, c.override_quat_alpha);
    target_compute_slopes(self, c.dynam->gravity_normal);
}

// pre-collide-setup: the decaying push velocity (push-transv) into collide-extra-velocity. A zero
// push stays zero: vector-normalize!'s 0/0 is GOAL's division, 0 * FLT_MAX.
void pre_collide_setup(Target& self) {
    ControlInfo& c = self.control;
    Vec& adv = c.additional_decaying_velocity;
    if (now(self) >= c.additional_decaying_velocity_decay_start_time) {
        vector_normalize(adv, seek(vector_length(adv), 0.0f, 40960.0f * spf(self)));
    }
    if (c.old_status & cstat::on_surface) {
        if (now(self) + seconds(-0.035) >= c.additional_decaying_velocity_end_time) {
            vector_reset(adv);
        }
    } else {
        float f30 = vector_length(adv);
        Vec n{};
        vector_normalize_copy(n, adv, 1.0f);
        const float f0 = vector_dot(c.transv, n);
        if (f0 < 0.0f) {
            f30 = seek(f30, 0.0f, (-1.0f * spf(self)) * f0);
        }
        vector_normalize(adv, seek(f30, 0.0f, 28672.0f * spf(self)));
    }
    c.collide_extra_velocity = adv;
    c.pre_collide_local_normal = c.local_normal;
}

// flag-setup: the per-tick bookkeeping before the move. level-setup, the R3 look-around camera, the
// sidekick and board set-up (the adapter mounts the board), the mirror, the debug float cheat, the
// endless-fall death (R&C has its own) and the beard are left out.
void flag_setup(Target& self) {
    ControlInfo& c = self.control;
    if (c.cpad->stick0_speed == 0.0f) {
        if (c.time_between_zero_inputs == 0) {
            c.time_between_zero_inputs = now(self) - c.time_of_last_zero_input;
        }
        c.time_of_last_zero_input = now(self);
        c.last_nonzero_input_dir_targ = c.dir_targ;
    } else {
        c.time_of_last_nonzero_input = now(self);
        c.time_between_zero_inputs = 0;
    }
    c.last_transv = c.transv;
    c.last_quat_for_control = c.quat_for_control;
    if (c.current_surface->active_hook) {
        c.current_surface->active_hook(self);
    }
    if (c.status & cstat::on_surface) {
        c.last_time_on_surface = now(self);
        c.last_trans_any_surf = c.trans;
        if (c.coverage >= 1.0f && !(c.status & (cstat::touch_actor | cstat::on_water)) &&
            (c.status & cstat::on_ground)) {
            c.last_trans_on_ground = c.trans;
        }
        if (c.current_surface->touch_hook) {
            c.current_surface->touch_hook(self);
        }
    } else {
        const Vec& v1 = c.trans;
        if (c.old_status & cstat::on_surface) {
            c.last_trans_leaving_surf = c.last_trans_any_surf;
            c.highest_jump_mark = c.last_trans_any_surf;
        }
        c.highest_jump_mark.x = v1.x;
        c.highest_jump_mark.z = v1.z;
        Vec d{};
        if (vector_dot(c.dynam->gravity_normal, vector_sub(d, c.highest_jump_mark, v1)) < 0.0f) {
            c.highest_jump_mark.y = v1.y;
        }
    }
    const float f17 = vector_dot(c.dynam->gravity_normal, c.transv);
    // The debug-cheat R2 test is #f outside the debug cheat mode.
    if (((c.current_surface->flags & sflag::check_edge) ||
         (next_state_is(self, {"target-walk", "target-gun-walk"}) &&
          !(c.status & cstat::on_surface))) &&
        (f17 < 0.0f &&
         !(self.focus_status & (focus::flut | focus::pilot | focus::mech | focus::indax)) &&
         !(self.state_flags & state_flags::prevent_jump))) {
        prim_action(root_prim(self)) |= caction::check_edge;
    } else {
        prim_action(root_prim(self)) &= ~caction::check_edge;
    }
    c.bend_speed = (c.status & cstat::on_surface) ? 32.0f : 2.0f;
}

// post-flag-setup: timers after the move. The gun joints, pre-joint-hook, hair and leg IK are the
// animation's and are left out.
void post_flag_setup(Target& self) {
    ControlInfo& c = self.control;
    if (c.status & (cstat::touch_wall | cstat::touch_actor)) {
        c.last_time_touching_actor = now(self);
    }
    if (self.state_flags & state_flags::tinvul1) {
        if (((now(self) - c.invul1_on_time) & 3) < 1) {
            self.draw_status |= draw_status::no_draw_bounds;
        } else {
            self.draw_status &= ~draw_status::no_draw_bounds;
        }
        if (time_elapsed(self, c.invul1_on_time, c.invul1_off_time)) {
            target_timed_invulnerable_off(self, 1);
        }
    }
    if (state_flags::tinvul2 & self.state_flags) {
        if (time_elapsed(self, c.invul2_on_time, c.invul2_off_time)) {
            target_timed_invulnerable_off(self, 2);
        }
    }
    c.jump_kind = Symbol();
    target_log_trans(self);
}

// bend-gravity: the bent gravity normal leans toward the ground under the target.
bool bend_gravity(Target& self) {
    ControlInfo& c = self.control;
    if ((prim_action(root_prim(self)) & caction::no_normal_reset) &&
        !(c.status & cstat::on_surface)) {
        return false;
    }
    const float f0 = ((c.status & cstat::touch_wall) && !(c.status & cstat::on_surface))
                         ? 0.0f
                         : c.bend_target;
    c.bend_amount = seek(c.bend_amount, f0, c.bend_speed * spf(self));
    c.dynam->gravity_normal = c.standard_dynamics->gravity_normal;
    vector_float_mul(c.dynam->gravity, c.dynam->gravity_normal, c.dynam->gravity_length);
    Vec s5{};
    const float f30 = c.current_surface->tiltvv;
    vector_deg_slerp(s5, c.standard_dynamics->gravity_normal, c.gspot_normal, c.bend_amount);
    Mat m{};
    matrix_from_two_vectors_smooth(m, c.bent_gravity_normal, s5, f30,
                                   to_int(c.current_surface->tiltvvf), spf(self));
    vector_matrix_mul(c.bent_gravity_normal, c.bent_gravity_normal, m);
    vector_normalize(c.bent_gravity_normal, 1.0f);
    // *display-target-marks* debug vectors left out.
    return true;
}

// do-target-gspot: the ground point under the target.
void do_target_gspot(Target& self) {
    ControlInfo& c = self.control;
    if ((c.status & cstat::on_surface) && c.mod_surface->mode != surface_mode::swim &&
        c.mod_surface->mode != surface_mode::dive && !next_state_is(self, {"target-flop"}) &&
        !(self.draw_status &
          (draw_status::no_draw | draw_status::no_draw_temp | draw_status::no_draw_bounds))) {
        c.gspot_pos = c.trans;
        c.gspot_normal = c.ground_poly_normal;
        c.gspot_pat_surfce = c.ground_pat;
    } else {
        PatSurface pat;
        if (find_ground(self, pat, prim_collide_with(root_prim(self)) & ~cspec::water, 8192.0f,
                        81920.0f, 1024.0f)) {
            c.gspot_pat_surfce = pat;
        }
    }
}

// target-real-post: the on-foot post function.
void target_real_post(Target& self) {
    ControlInfo& c = self.control;
    const float f30 = self.clock_ratio;
    int gp = std::max(1, to_int(self.clock.time_adjust_ratio));
    update_rates(self, f30 / static_cast<float>(gp));
    while (gp != 0) {
        gp--;
        c.remaining_ctrl_iterations = gp;
        flag_setup(self);
        if (c.force_turn_to_strength < 0.0f) {
            c.force_turn_to_strength = 1.0f - c.cpad->stick0_speed;
        }
        build_conversions(self, c.transv);
        do_rotations1(self);
        Vec s5{};
        read_pad(self, s5);
        float f28 = debounce_speed(c.pad_magnitude, c.last_pad_magnitude, c.pad_xz_dir,
                                   c.last_pad_xz_dir);
        if (c.force_turn_to_strength != 0.0f) {
            const float f12 = std::fmin(1.0f, c.force_turn_to_strength);
            c.force_turn_to_strength = f12;
            Vec a1{}, a2{};
            vector_float_mul(a1, f28 == 0.0f ? Vec{} : s5, f28);
            vector_float_mul(a2, c.force_turn_to_direction, c.force_turn_to_speed);
            vector_lerp(s5, a1, a2, f12);
            f28 = vector_length(s5);
            vector_normalize(s5, 1.0f);
        }
        turn_to_vector(self, s5, f28);
        add_thrust(self);
        add_gravity(self);
        do_rotations2(self);
        reverse_conversions(self, c.transv);
        pre_collide_setup(self);
        target_integrate_and_collide(self, c.transv, meters(1));
        Vec d{};
        if ((prim_action(root_prim(self)) & caction::check_edge) &&
            vector_dot(c.dynam->gravity_normal, vector_sub(d, c.trans, c.gspot_pos)) >=
                target_bank.edge_grab_height_off_ground) {
            do_edge_grabs_target(self);
        }
        bend_gravity(self);
        post_flag_setup(self);
    }
    update_rates(self, f30);
    ja_post(self);
    joint_points(self);
    do_target_gspot(self);
    // target-powerup-process (eco, dark Jak) left out.
}

// target-post
void target_post(Target& self) { target_real_post(self); }

// target-no-move-post: rotations and bookkeeping without a move.
void target_no_move_post(Target& self) {
    ControlInfo& c = self.control;
    const float f30 = self.clock_ratio;
    int gp = std::max(1, to_int(self.clock.time_adjust_ratio));
    update_rates(self, f30 / static_cast<float>(gp));
    while (gp != 0) {
        gp--;
        c.remaining_ctrl_iterations = gp;
        flag_setup(self);
        build_conversions(self, c.transv);
        do_rotations1(self);
        do_rotations2(self);
        reverse_conversions(self, c.transv);
        vector_sub(c.cspace_offset, c.draw_offset, c.anim_collide_offset_world);
        find_overlapping_shapes_target(self);
        post_flag_setup(self);
    }
    update_rates(self, f30);
    ja_post(self);
    joint_points(self);
    // *debug-segment* history records left out.
    do_target_gspot(self);
    // target-powerup-process left out.
}

// ---- target-util.gc ----

// debounce-speed: a half push that reverses direction is read as none.
float debounce_speed(float mag, float last_mag, const Vec& dir, const Vec& last_dir) {
    if (0.3f < mag && mag < 0.7f && 0.0f < last_mag && vector_dot(dir, last_dir) < 0.2f) {
        mag = 0.0f;
    }
    return mag;
}

// target-start-attack. (-> *game-info* attack-id) is a counter of the whole game: here a static.
// anim-seed is the animation's (left out).
void target_start_attack(Target& self) {
    static uint32_t attack_id = 0;
    attack_id = attack_id + 1;
    self.control.target_attack_id = attack_id;
    self.control.attack_count = 0;
    self.control.bomb_scale = 0.0f;
}

// target-danger-set!: the attack spheres for a danger mode. Dark Jak's sizes and eco's penetrate
// bits are left out (neither exists in the mod).
void target_danger_set(Target& self, Symbol mode, Symbol variant) {
    ControlInfo& c = self.control;
    const TargetBank& b = target_bank;
    CollideShapePrim* s4 = collision_sphere(self, 6);
    CollideShapePrim* s5 = collision_sphere(self, 7);
    CollideShapePrim* gp = collision_sphere(self, 8);
    CollideShapePrim* s2 = collision_sphere(self, 9);
    CollideShapePrim* root = root_prim(self);
    float f30 = (variant == Symbol("cone") || variant == Symbol("eco-red")) ? 2.0f : 1.0f;
    c.penetrate_using = penetrate::touch;
    c.penetrated_by = penetrate::lunge;
    set_sphere(prim_local_sphere(root), b.root_offset, b.root_radius);
    c.danger_mode = mode;
    self.focus_status &= ~focus::dangerous;
    for (CollideShapePrim* p : {s4, s5, gp, s2}) {
        prim_transform_index(p) = -2;
        prim_collide_as(p) = 0;
        prim_collide_with(p) = 0;
        prim_local_sphere(p).w = 0.0f;
    }
    auto arm = [&](CollideShapePrim* p) {
        prim_collide_as(p) = c.default_collide_as_fgnd;
        prim_collide_with(p) = c.default_collide_with_fgnd;
    };
    const Vec null_vec{};
    const Vec root66{0.0f, 6553.6f, 0.0f, 1.0f};
    if (mode == kHarmless || !mode) {
        c.danger_mode = Symbol();
        target_collide_set(self, c.collide_mode, c.collide_mode_transition);
    } else if (mode == Symbol("carry?")) {
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.spin_offset, 8192.0f + b.spin_radius);
        prim_local_sphere(root) = prim_local_sphere(s4);
    } else if (mode == Symbol("spin") || mode == Symbol("spin-air")) {
        set_sphere(prim_local_sphere(root), root66, 9830.4f);
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.spin_offset, b.spin_radius);
        c.penetrate_using = penetrate::touch | penetrate::spin;
    } else if (mode == Symbol("board-spin")) {
        set_sphere(prim_local_sphere(root), root66, 9830.4f);
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.spin_offset, 6963.2f);
        c.penetrate_using = penetrate::touch | penetrate::spin | penetrate::board;
    } else if (mode == Symbol("duck-spin")) {
        set_sphere(prim_local_sphere(root), root66, 10649.6f);
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.duck_spin_offset, b.duck_spin_radius);
        arm(s5);
        set_sphere(prim_local_sphere(s5), null_vec, b.duck_spin_radius);
        prim_transform_index(s5) = 35;
        c.penetrate_using = penetrate::touch | penetrate::spin;
    } else if (mode == Symbol("duck-slide")) {
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.duck_spin_offset, b.duck_spin_radius);
        arm(s5);
        set_sphere(prim_local_sphere(s5), null_vec, b.duck_spin_radius);
        prim_transform_index(s5) = 28;
        c.penetrate_using = penetrate::touch | penetrate::punch;
    } else if (mode == Symbol("roll-solid") || mode == Symbol("flip")) {
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.duck_spin_offset, 819.2f + b.duck_spin_radius);
        arm(s5);
        set_sphere(prim_local_sphere(s5), null_vec, b.duck_spin_radius);
        prim_transform_index(s5) = 8;
        c.danger_mode = mode == Symbol("flip") ? Symbol("flip") : Symbol("roll");
        c.penetrate_using = penetrate::touch | penetrate::roll;
    } else if (mode == Symbol("punch")) {
        set_sphere(prim_local_sphere(root), root66, 11878.4f);
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.punch_offset, b.punch_radius);
        if (!(self.focus_status & focus::indax)) {
            arm(s5);
            set_sphere(prim_local_sphere(s5), null_vec, b.punch_radius);
            prim_transform_index(s5) = 21;
        }
        c.penetrate_using = penetrate::touch | penetrate::punch;
    } else if (mode == Symbol("uppercut")) {
        set_sphere(prim_local_sphere(root), root66, 11878.4f);
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.uppercut0_offset, b.uppercut_radius);
        arm(s5);
        set_sphere(prim_local_sphere(s5), null_vec, b.uppercut_radius);
        prim_transform_index(s5) = 21;
        arm(gp);
        set_sphere(prim_local_sphere(gp), b.uppercut1_offset, b.uppercut_radius);
        c.penetrate_using = penetrate::touch | penetrate::uppercut;
    } else if (mode == Symbol("flop")) {
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.flop0_offset, b.flop_radius);
        f30 = 1.0f;
        c.danger_mode = Symbol("flop");
        c.penetrate_using = penetrate::touch | penetrate::flop;
    } else if (mode == Symbol("flop-down")) {
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.flop0_offset, b.flop_radius);
        arm(s5);
        set_sphere(prim_local_sphere(s5), b.flop1_offset, b.flop_radius);
        if (f30 != 1.0f) {
            f30 = 2.1f;
        }
        c.danger_mode = Symbol("flop");
        c.penetrate_using = penetrate::touch | penetrate::flop;
    } else if (mode == Symbol("flut-attack")) {
        set_sphere(prim_local_sphere(root), root66, 12288.0f);
        self.focus_status |= focus::dangerous;
        arm(s4);
        set_sphere(prim_local_sphere(s4), b.spin_offset, 2048.0f + b.punch_radius);
        arm(s5);
        set_sphere(prim_local_sphere(s5), null_vec, 1228.8f + b.punch_radius);
        prim_transform_index(s5) = 8;
        c.penetrate_using = penetrate::touch | penetrate::flut_attack;
    } else if (mode == Symbol("mech-punch")) {
        self.focus_status |= focus::dangerous;
        c.penetrate_using = penetrate::touch | penetrate::mech;
    } else if (mode == Symbol("bomb")) {
        const float f28 = lerp_scale(b.root_radius, 122880.0f, c.bomb_scale, 0.0f, 1.0f);
        set_sphere(prim_local_sphere(root), b.root_offset, f28);
        self.focus_status |= focus::dangerous;
        prim_collide_as(s4) = c.default_collide_as_fgnd;
        prim_collide_with(s4) = c.default_collide_with_fgnd & ~cspec::collectable;
        set_sphere(prim_local_sphere(s4), b.root_offset, f28);
        c.penetrate_using = penetrate::touch | penetrate::dark_bomb;
    }
    // else: GOAL prints "unknown danger mode" and goes on.
    if (variant == Symbol("cone")) {
        const float f28 = 16384.0f;
        set_sphere(prim_local_sphere(root), b.root_offset, b.root_radius + 2.0f * f28);
        self.focus_status |= focus::dangerous;
        prim_collide_as(s2) = c.default_collide_as_fgnd;
        prim_collide_with(s2) = c.default_collide_with_fgnd & ~cspec::collectable;
        prim_transform_index(s2) = 0;
        set_sphere(prim_local_sphere(s2), Vec{0.0f, 12288.0f, 16384.0f, 1.0f}, f28);
        c.penetrate_using = penetrate::touch | penetrate::dark_bomb;
    }
    if (f30 != 1.0f && !(mode == kHarmless || !mode)) {
        prim_local_sphere(root).w = prim_local_sphere(root).w * f30;
        prim_local_sphere(s4).w = prim_local_sphere(s4).w * f30;
        prim_local_sphere(s5).w = prim_local_sphere(s5).w * f30;
        prim_local_sphere(gp).w = prim_local_sphere(gp).w * f30;
    }
}

// target-collide-set!: the body spheres for a collide mode. The gun (enabled-gun?, using-gun?) and
// dark Jak's giant scale never apply in the mod.
int target_collide_set(Target& self, Symbol mode, float transition) {
    ControlInfo& c = self.control;
    const TargetBank& b = target_bank;
    c.collide_mode = mode;
    c.collide_mode_transition = transition;
    c.penetrate_using &= ~(penetrate::tube | penetrate::vehicle | penetrate::board | penetrate::mech);
    c.penetrated_by = penetrate::lunge;
    for (int i = 0; i < 4; i++) {
        prim_transform_index(collision_sphere(self, i)) = -2;
    }
    prim_collide_as(collision_sphere(self, 3)) = 0;
    prim_collide_with(collision_sphere(self, 3)) = 0;
    prim_collide_as(collision_sphere(self, 4)) = cspec::jak;
    prim_collide_with(collision_sphere(self, 4)) =
        cspec::backgnd | cspec::bot | cspec::crate | cspec::civilian | cspec::enemy |
        cspec::obstacle | cspec::hit_by_player_list | cspec::hit_by_others_list |
        cspec::player_list | cspec::water | cspec::collectable | cspec::blocking_plane |
        cspec::tobot | cspec::pusher | cspec::vehicle_mesh | cspec::obstacle_for_jak;
    Vec& s0 = prim_local_sphere(collision_sphere(self, 0));
    Vec& s1 = prim_local_sphere(collision_sphere(self, 1));
    Vec& s2 = prim_local_sphere(collision_sphere(self, 2));
    Vec& root = prim_local_sphere(root_prim(self));
    const float br = b.body_radius;
    auto body = [&](float t) { // the standing stack, t = how far it is stretched up
        c.draw_offset.y = 0.0f;
        set_sphere(root, b.root_offset, b.root_radius);
        s0 = Vec{0.0f, br, 0.0f * t, br};
        s1 = Vec{0.0f, br + 2867.2f * t, 0.0f, br};
        s2 = Vec{0.0f, br + 5734.4f * t, 0.0f, br};
    };
    if (mode == Symbol("pole")) {
        s0 = Vec{0.0f, 0.0f, 0.0f, br};
        s1 = Vec{0.0f, 0.0f, 0.0f, br};
        s2 = Vec{0.0f, 0.0f, 0.0f, br};
        prim_transform_index(collision_sphere(self, 0)) = 28;
        prim_transform_index(collision_sphere(self, 1)) = 26;
        prim_transform_index(collision_sphere(self, 2)) = 8;
    } else if (mode == Symbol("board")) {
        c.penetrate_using |= penetrate::touch | penetrate::board;
    } else if (mode == Symbol("racer")) {
        c.draw_offset.y = 4096.0f;
        set_sphere(root, Vec{0.0f, 8192.0f, 0.0f, 1.0f}, 16384.0f);
        s0 = Vec{0.0f, 6553.6f, 0.0f, 6553.6f};
        s1 = Vec{0.0f, 9420.8f, 0.0f, 6553.6f};
        s2 = Vec{0.0f, 12288.0f, 0.0f, 6553.6f};
        c.penetrate_using |= penetrate::touch | penetrate::vehicle;
    } else if (mode == Symbol("flut")) {
        set_sphere(root, Vec{0.0f, 8192.0f, 0.0f, 1.0f}, 12288.0f);
        s0 = Vec{0.0f, 4915.2f, 0.0f, 4915.2f};
        s1 = Vec{0.0f, 7782.4004f, 0.0f, 4915.2f};
        s2 = Vec{0.0f, 10649.6f, 0.0f, 4915.2f};
    } else if (mode == Symbol("mech")) {
        set_sphere(root, Vec{0.0f, 8192.0f, 0.0f, 1.0f}, 16384.0f);
        s0 = Vec{0.0f, 4096.0f, 0.0f, 4096.0f};
        s1 = Vec{0.0f, 8601.6f, 0.0f, 6553.6f};
        s2 = Vec{0.0f, 14745.6f, 0.0f, 9420.8f};
        c.penetrate_using |= penetrate::touch | penetrate::mech;
    } else if (mode == Symbol("mech-carry")) {
        c.draw_offset.y = 0.0f;
        set_sphere(root, Vec{0.0f, 12288.0f, 0.0f, 1.0f}, 20480.0f);
        s0 = Vec{0.0f, 4096.0f, 0.0f, 4096.0f};
        s1 = Vec{0.0f, 8601.6f, 0.0f, 6553.6f};
        s2 = Vec{0.0f, 5734.4f + lerp_scale(9011.2f, 12288.0f, transition, 0.0f, 1.0f), 0.0f,
                 lerp_scale(9420.8f, 12697.6f, transition, 0.0f, 1.0f)};
    } else if (mode == Symbol("duck")) {
        c.duck_gun_tube_transision = transition;
        body(1.0f - transition); // using-gun? is never true in the mod
    } else if (mode == Symbol("indax")) {
        body(0.2f);
        prim_collide_as(collision_sphere(self, 4)) = 0;
        prim_collide_with(collision_sphere(self, 4)) = 0;
    } else if (mode == Symbol("tube")) {
        c.duck_gun_tube_transision = transition;
        body(1.0f - transition);
        c.penetrate_using |= penetrate::touch | penetrate::tube;
    } else if (mode == Symbol("carry")) {
        c.draw_offset.y = 0.0f;
        set_sphere(root, b.root_offset, b.root_radius);
        s0 = Vec{0.0f, br, 0.0f, br};
        s1 = Vec{0.0f, 2867.2f + br, 0.0f, br};
        s2 = Vec{0.0f, 5734.4f + br, 0.0f, lerp_scale(br, 7372.8f, transition, 0.0f, 1.0f)};
    } else if (mode == Symbol("gun")) {
        c.duck_gun_tube_transision = transition;
        c.draw_offset.y = 0.0f;
        set_sphere(root, b.root_offset, b.root_radius);
        s0 = Vec{0.0f, br, 0.0f, br};
        s1 = Vec{0.0f, 2867.2f + br, 0.0f, br};
        s2 = Vec{0.0f, 5734.4f + br, 0.0f, br};
        prim_local_sphere(collision_sphere(self, 3)) =
            Vec{0.0f, 5734.4f + br, 0.0f, lerp_scale(br, 4915.2f, transition, 0.0f, 1.0f)};
        prim_collide_with(collision_sphere(self, 3)) =
            cspec::backgnd | cspec::bot | cspec::crate | cspec::obstacle |
            cspec::hit_by_player_list | cspec::hit_by_others_list | cspec::player_list |
            cspec::water | cspec::collectable | cspec::blocking_plane | cspec::tobot |
            cspec::pusher | cspec::vehicle_mesh | cspec::obstacle_for_jak;
    } else {
        c.duck_gun_tube_transision = 0.0f;
        c.draw_offset.y = 0.0f;
        set_sphere(root, b.root_offset, b.root_radius);
        s0 = Vec{0.0f, br, 0.0f, br};
        s1 = Vec{0.0f, 2867.2f + br, 0.0f, br};
        s2 = Vec{0.0f, 5734.4f + br, 0.0f, br};
    }
    return 0;
}

// target-height-above-ground
float target_height_above_ground(Target& self) {
    return self.control.trans.y - self.control.gspot_pos.y;
}

// can-jump?
bool can_jump_p(Target& self, Symbol kind) {
    ControlInfo& c = self.control;
    const bool board = kind == Symbol("board");
    const bool grounded =
        (c.status & cstat::on_surface) ||
        !time_elapsed(self, c.last_time_on_surface, target_bank.ground_timeout) ||
        ((c.status & cstat::on_surface) && 0.866f < c.surface_angle) ||
        (board && (!time_elapsed(self, self.board->unknown_time_frame00, seconds(0.05)) ||
                   (!time_elapsed(self, c.last_time_of_stuck, seconds(0.5)) &&
                    target_height_above_ground(self) < 2048.0f)));
    return grounded && !(c.current_surface->flags & sflag::no_jump) &&
           !(self.state_flags & state_flags::prevent_jump) &&
           (board || c.poly_pat.event() != 13) &&
           (kind == Symbol("target-roll-flip") ? 0.5f >= c.local_slope_z : true);
}

// move-legs?
bool move_legs_p(Target& self) { return self.control.cpad->stick0_speed != 0.0f; }

// jump-hit-ground-stuck?. The cheat-mode R2 escape: *cheat-mode* is #f in a released game.
bool jump_hit_ground_stuck_p(Target& self) {
    ControlInfo& c = self.control;
    if (c.status & cstat::on_surface) {
        return true;
    }
    if (target_move_dist(self, target_bank.stuck_time) < target_bank.stuck_distance) {
        c.last_time_of_stuck = now(self);
        return true;
    }
    return false;
}

// target-time-to-ground: ticks of 1/60 s (5 time-frames each) until the fall reaches the gspot.
TimeFrame target_time_to_ground(Target& self) {
    ControlInfo& c = self.control;
    float f0 = target_height_above_ground(self);
    float f2 = vector_dot(c.dynam->gravity_normal, c.transv);
    float f1 = 0.0f;
    while (0.0f < f0) {
        f1 = 5.0f + f1;
        f0 = f0 + f2 * spf(self);
        f2 = f2 - c.dynam->gravity_length * spf(self);
    }
    return static_cast<TimeFrame>(f1);
}

// can-exit-duck?: room to stand up.
bool can_exit_duck_p(Target& self) {
    const Vec& t = self.control.trans;
    const float br = target_bank.body_radius;
    Vec s[2];
    s[0] = t;
    s[0].y = (5734.4f + br) + s[0].y;
    s[0].w = br;
    s[1] = t;
    s[1].y = (2867.2f + br) + s[1].y;
    s[1].w = br;
    const uint32_t with = prim_collide_with(root_prim(self)) &
                          ~(cspec::civilian | cspec::enemy | cspec::vehicle_sphere |
                            cspec::projectile);
    return !fill_and_probe_using_spheres(self, s, with);
}

// can-feet? (the argument is unused in GOAL too)
bool can_feet_p(Target& self, Symbol) {
    ControlInfo& c = self.control;
    if ((self.state_flags & state_flags::prevent_attack) ||
        (c.current_surface->flags & (sflag::no_attack | sflag::no_feet))) {
        return false;
    }
    if (time_elapsed(self, c.last_attack_end_time, target_bank.attack_timeout)) {
        return true;
    }
    c.last_feet_attempt_time = now(self);
    return false;
}

// set-forward-vel
void set_forward_vel(Target& self, float vel) {
    ControlInfo& c = self.control;
    Vec gp{};
    vector_matrix_mul(gp, c.transv, c.w_R_c);
    gp.z = vel;
    gp.x = 0.0f;
    vector_matrix_mul(c.transv, gp, c.c_R_w);
}

// delete-back-vel
void delete_back_vel(Target& self) {
    ControlInfo& c = self.control;
    Vec gp{}, v1{}, t{};
    vector_z_quaternion(gp, c.dir_targ);
    float f0 = vector_dot(gp, c.transv);
    vector_sub(v1, c.transv, vector_float_mul(t, gp, f0));
    const float f1 = vector_length(v1);
    const float f2 = f1;
    if (f0 < 0.0f) {
        f0 = 0.0f;
    }
    vector_float_mul(c.transv, gp, f0);
    vector_float_mul(v1, v1, goal_div(f1, f2));
    vector_add(c.transv, c.transv, v1);
}

// target-timed-invulnerable
void target_timed_invulnerable(TimeFrame duration, Target& self, int which) {
    ControlInfo& c = self.control;
    if (which == 1) {
        self.state_flags |= state_flags::tinvul1;
        c.invul1_on_time = now(self);
        c.invul1_off_time = duration;
    } else if (which == 2) {
        self.state_flags |= state_flags::tinvul2;
        c.invul2_on_time = now(self);
        c.invul2_off_time = duration;
    }
    modify_collide_as(self, 2, cspec::jak_vulnerable, 0);
    self.focus_status |= focus::ignore;
}

// target-timed-invulnerable-off
void target_timed_invulnerable_off(Target& self, int which) {
    if (which == 0) {
        self.state_flags &= ~(state_flags::tinvul1 | state_flags::tinvul2);
    } else if (which == 1) {
        self.state_flags &= ~state_flags::tinvul1;
    } else if (which == 2) {
        self.state_flags &= ~state_flags::tinvul2;
    }
    if (!((state_flags::tinvul1 | state_flags::tinvul2) & self.state_flags)) {
        self.draw_status &= ~draw_status::no_draw_bounds;
        modify_collide_as(self, 2, 0, cspec::jak_vulnerable);
    }
}

// ---- target.gc ----

// init-var-jump: the jump's heights (kept in unknown-word04, as float bits, and
// did-move-to-pole-or-max-jump-height), and the launch speed when set_vel. Dark Jak's giant scale
// is left out.
const Vec& init_var_jump(Target& self, float min_height, float max_height, bool set_vel,
                         bool collide_offset, Vec& transv, float extra) {
    ControlInfo& c = self.control;
    const float gl = c.dynam->gravity_length;
    c.status &= ~cstat::touch_ceiling_sticky;
    c.unknown_time_frame19 = now(self);
    delete_back_vel(self);
    const float f2 = gl / 600.0f + std::fmax(0.0f, vector_dot(c.dynam->gravity_normal, c.transv));
    const float f05 = extra * std::fmax(0.0f, goal_div((f2 / 2.0f) * f2, gl));
    float s3 = min_height + f05;
    float s2 = max_height + f05;
    if (!time_elapsed(self, c.rider_time, seconds(0.05))) {
        const float f8 = std::fmax(
            0.0f, std::fmin(28672.0f, 0.5f * vector_dot(c.dynam->gravity_normal, c.rider_last_move)));
        s3 = s3 + f8;
        s2 = s2 + f8;
        target_push_transv(self, c.rider_last_move, seconds(100));
    }
    c.unknown_float36 = 0.0f;
    c.unknown_float37 = 0.0f;
    float low;
    if (collide_offset) {
        low = s3 - (-409.6f + target_bank.jump_collide_offset);
        c.did_move_to_pole_or_max_jump_height = s2 - target_bank.jump_collide_offset;
    } else {
        low = s3;
        c.did_move_to_pole_or_max_jump_height = s2;
    }
    std::memcpy(&c.unknown_word04, &low, 4); // (the-as uint float)
    if (set_vel) {
        Vec v1{}, t{};
        const float f19 = vector_dot(c.dynam->gravity_normal, transv);
        vector_sub(v1, transv, vector_float_mul(t, c.dynam->gravity_normal, f19));
        const float f20 = vector_length(v1);
        const float f16 = f20;
        const float f13 = std::sqrt((2.0f * gl) * s3) - 0.008333334f * -gl;
        vector_float_mul(transv, c.dynam->gravity_normal, f13);
        vector_float_mul(v1, v1, goal_div(f20, f16));
        vector_add(transv, transv, v1);
    }
    c.unknown_vector37 = c.trans;
    return c.unknown_vector37;
}

// mod-var-jump: while `active` (the button held) within the state's first 30 time-frames, the
// jump's top rises from the low height toward the high one and the upward speed follows it.
void mod_var_jump(Target& self, bool mod_vel, bool anim_offset, bool active, Vec& transv) {
    ControlInfo& c = self.control;
    const float f01 = 0.033333335f * static_cast<float>(now(self) - self.state_time);
    if (1.0f < f01 || c.unknown_float36 < 0.0f || !active) {
        c.unknown_float36 = -1.0f;
    } else {
        c.unknown_float36 = f01;
        c.unknown_float37 = f01;
        if (mod_vel) {
            const float gl = c.dynam->gravity_length;
            Vec s3{}, s4{}, t{};
            vector_sub(s3, c.trans, c.unknown_vector37);
            const float f15 = vector_dot(c.dynam->gravity_normal, transv);
            vector_sub(s4, transv, vector_float_mul(t, c.dynam->gravity_normal, f15));
            const float f30 = vector_length(s4);
            const float f28 = f30;
            float low;
            std::memcpy(&low, &c.unknown_word04, 4);
            const float h = lerp_scale(low, c.did_move_to_pole_or_max_jump_height, f01, 0.0f, 1.0f);
            const float f07 =
                std::sqrt((2.0f * gl) * (h - vector_dot(c.dynam->gravity_normal, s3))) -
                0.008333334f * -gl;
            vector_float_mul(transv, c.dynam->gravity_normal, f07);
            vector_float_mul(s4, s4, goal_div(f30, f28));
            vector_add(transv, transv, s4);
        }
    }
    if (anim_offset) {
        Vec off{};
        const int r = ja_group_collide_offset(self, off);
        if (r == 1) {
            c.anim_collide_offset_local = off;
        } else if (r == 2) {
            c.anim_collide_offset_local = Vec{};
        }
    }
}

// ---- target-handler.gc ----

// target-exit: what every state's exit undoes. The water, neck, skeleton, impact control, combo
// tracker and effect flags are left out (not in the mod).
void target_exit(Target& self) {
    ControlInfo& c = self.control;
    c.mod_surface = &walk_mods;
    c.anim_collide_offset_local = Vec{};
    c.anim_collide_offset_world = Vec{};
    c.old_anim_collide_offset_world = Vec{};
    c.draw_offset = Vec{};
    c.force_turn_to_strength = 0.0f;
    c.bend_target = 0.0f;
    c.bomb_scale = 0.0f;
    self.state_flags &=
        ~(state_flags::sf1 | state_flags::sf5 | state_flags::sf13 | state_flags::sf14 |
          state_flags::sf15 | state_flags::lleg_still | state_flags::rleg_still |
          state_flags::lleg_no_ik | state_flags::rleg_no_ik);
    self.focus_status &= ~(focus::hit | focus::grabbed | focus::in_head | focus::teleporting);
    target_danger_set(self, kHarmless, Symbol());
    c.duck_gun_tube_transision = 0.0f;
    c.status &= ~cstat::touch_ceiling_sticky;
}

// 'push-transv (target-standard-event-handler).
void target_push_transv(Target& self, const Vec& v, TimeFrame decay_start) {
    ControlInfo& c = self.control;
    c.additional_decaying_velocity = v;
    c.additional_decaying_velocity_end_time = now(self);
    c.additional_decaying_velocity_decay_start_time = now(self) + decay_start;
}

// ---- the clock (gkernel-h.gc, timer.gc) ----

float frames_per_second(const Target& self) { return self.clock.frames_per_second; }
TimeFrame old_frame_counter(const Target& self) { return self.clock.old_frame_counter; }

// seconds-per-frame-high-fps: a fixed 16.6 ms frame in the PC port's custom video mode.
float seconds_per_frame_high_fps(const Target& self) {
    return self.custom_video_mode ? 0.016666668f : self.clock.seconds_per_frame;
}

// rot->dir-targ! (control-info): dir-targ = get-quaternion, which is quat-for-control.
void rot_to_dir_targ(Target& self) { self.control.dir_targ = self.control.quat_for_control; }

// set-quaternion! (control-info): only quat-for-control; the draw quat follows in the post.
void set_quaternion(Target& self, const Quat& q) { self.control.quat_for_control = q; }

// (-> *FACT-bank* health-default-inc) (fact-h.gc).
float fact_health_default_inc() { return 2.0f; }

// ---- logic-target.gc ----

// target-no-ja-move-post: the post of states that drive no animation root motion.
void target_no_ja_move_post(Target& self) {
    ControlInfo& c = self.control;
    vector_sub(c.cspace_offset, c.draw_offset, c.anim_collide_offset_world);
    find_overlapping_shapes_target(self);
    // (target-calc-camera-pos): the camera's follow point; the port has no camera, left out.
    do_target_gspot(self);
    target_powerup_process(self);
    target_board_joint_points(self);
}

// ---- powerups.gc ----

// target-powerup-process. Nothing in it changes the board's movement, so the port runs none of it:
// - water-control-method-10 (with sphere 0 grown 1 m on the board) and the under-water times: no
//   water in the port;
// - the ice-loop sound and toe particles on ice: sounds and particles are the game's;
// - target-darkjak-process: no dark Jak;
// - the trick-point judge timeout ('notify 'trick-judge): the fact's trick points are the game's;
// - the stop-time and fast-clock cheats.
void target_powerup_process(Target&) {}

// ---- target-gun.gc ----

// target-gun-end-mode: everything in it is under (using-gun? self), and the port has no gun, so
// the test is always false and it does nothing.
void target_gun_end_mode(Target&, bool) {}

// ---- target-util.gc ----

// can-play-stance-amibent?: may the idle stance play an ambient animation now. The non-kiosk
// branch (the kiosk one is a demo build's). The game-side tests the port cannot see take their
// in-play values: not paused, no movie, not tobot, the talking/spooling/movie/hint settings off,
// and the sidekick feature on.
bool can_play_stance_ambient(Target& self) {
    constexpr uint64_t kDead = 1u << 1; // focus-status dead
    const ControlInfo& c = self.control;
    if (!time_elapsed(self, self.ambient_time, 9000)) return false; // (seconds 30)
    if (c.status & cstat::touch_actor) return false;
    if (!(c.status & cstat::on_surface)) return false;
    if (c.status & cstat::on_water) return false; // touch-water: no water in the port
    if (self.focus_status & (kDead | focus::hit | focus::grabbed | focus::in_head | focus::edge_grab |
                             focus::pole | focus::flut | focus::tube | focus::board | focus::mech |
                             focus::dark | focus::indax | focus::teleporting))
        return false;
    if (self.base_frame_counter - self.pad.change_time < 9000) return false;
    if (self.focus_status & (kDead | focus::hit | focus::grabbed)) return false;
    return true;
}

} // namespace jak
