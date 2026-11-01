// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's target movement core (engine/target/logic-target.gc, target-util.gc, target.gc,
// target-handler.gc), the part the JET-Board's post functions and states call, ported from OpenGOAL
// (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
//
// One tick of a target post function runs, per clock step: flag-setup, build-conversions (world
// velocity into the control frame, current-surface = mod-surface x surf), do-rotations1 (tilt),
// read-pad + turn-to-vector (where the stick points), thrust and gravity in the control frame,
// do-rotations2 (heading), reverse-conversions (back to world), pre-collide-setup, the collision
// move (the collide part), bend-gravity, post-flag-setup.
#pragma once

#include "jak/collide_mem.hpp"
#include "jak/target.hpp"

namespace jak {

// state-flags (game-h.gc).
namespace state_flags {
constexpr uint64_t sf1 = 1u << 1, tinvul1 = 1u << 3, sf5 = 1u << 5, prevent_jump = 1u << 7,
                   prevent_attack = 1u << 8, prevent_duck = 1u << 9, sf13 = 1u << 13,
                   sf14 = 1u << 14, sf15 = 1u << 15, prevent_board = 1u << 17,
                   lleg_still = 1u << 19, rleg_still = 1u << 20, lleg_no_ik = 1u << 21,
                   rleg_no_ik = 1u << 22, tinvul2 = 1u << 25;
} // namespace state_flags

// focus-status (process-focusable.gc).
namespace focus {
constexpr uint64_t ignore = 1u << 2, dangerous = 1u << 4, in_air = 1u << 5, hit = 1u << 6,
                   grabbed = 1u << 7, in_head = 1u << 8, on_water = 1u << 10, edge_grab = 1u << 12,
                   pole = 1u << 13, flut = 1u << 15, tube = 1u << 16, board = 1u << 18,
                   pilot = 1u << 20, mech = 1u << 21, dark = 1u << 22, rail = 1u << 23,
                   halfpipe = 1u << 24, indax = 1u << 28, teleporting = 1u << 30;
} // namespace focus

// collide-action and collide-spec: the collision's own (collide_mem.hpp).

// penetrate (penetrate-h.gc).
namespace penetrate {
constexpr uint64_t touch = 1ull << 0, lunge = 1ull << 2, flop = 1ull << 3, punch = 1ull << 4,
                   spin = 1ull << 5, roll = 1ull << 6, uppercut = 1ull << 7, tube = 1ull << 9,
                   vehicle = 1ull << 10, flut_attack = 1ull << 11, board = 1ull << 12,
                   mech = 1ull << 13, dark_bomb = 1ull << 18;
} // namespace penetrate

// draw-control-status bits the movement core reads or writes on Target::draw_status. The port
// keeps only these and numbers them itself (the adapter and the board are the only other readers).
namespace draw_status {
constexpr uint32_t no_draw = 1u << 1, no_draw_temp = 1u << 2, no_draw_bounds = 1u << 5;
} // namespace draw_status

// target-bank (target-util.gc) and *TARGET-bank*. The uint64 timeouts are time-frames.
struct TargetBank {
    float jump_collide_offset, jump_height_min, jump_height_max, double_jump_height_min,
        double_jump_height_max, flip_jump_height_min, flip_jump_height_max, duck_jump_height_min,
        duck_jump_height_max, flop_jump_height_min, flop_jump_height_max, attack_jump_height_min,
        attack_jump_height_max, edge_grab_jump_height_min, edge_grab_jump_height_max,
        swim_jump_height_min, swim_jump_height_max, tube_jump_height_min, tube_jump_height_max,
        carry_jump_height_min, carry_jump_height_max, mech_jump_height_min, mech_jump_height_max,
        mech_carry_jump_height_min, mech_carry_jump_height_max, indax_jump_height_min,
        indax_jump_height_max, indax_double_jump_height_min, indax_double_jump_height_max;
    TimeFrame roll_duration, roll_jump_pre_window, roll_jump_post_window, roll_timeout;
    float roll_speed_min, roll_speed_inc;
    TimeFrame roll_flip_duration;
    float roll_flip_height, roll_flip_dist, roll_flip_art_height, roll_flip_art_dist,
        duck_slide_distance, fall_far, fall_far_inc;
    TimeFrame attack_timeout, ground_timeout, slide_down_timeout, fall_timeout;
    float fall_stumble_threshold, yellow_projectile_speed;
    TimeFrame hit_invulnerable_timeout, same_attack_invulnerable_timeout;
    float run_cycle_length, walk_cycle_dist, walk_up_cycle_dist, walk_down_cycle_dist,
        walk_side_cycle_dist, run_cycle_dist, run_up_cycle_dist, run_down_cycle_dist,
        run_side_cycle_dist, run_wall_cycle_dist, duck_walk_cycle_dist,
        wade_shallow_walk_cycle_dist, wade_deep_walk_cycle_dist, mech_walk_cycle_dist,
        mech_run_cycle_dist, smack_surface_dist, smack_surface_height, min_dive_depth,
        root_radius;
    Vec root_offset;
    float body_radius, edge_radius;
    Vec edge_offset;
    float edge_grab_height_off_ground, head_radius, head_height;
    Vec head_offset;
    float spin_radius;
    Vec spin_offset;
    float duck_spin_radius;
    Vec duck_spin_offset;
    float punch_radius;
    Vec punch_offset;
    float uppercut_radius;
    Vec uppercut0_offset, uppercut1_offset;
    float flop_radius;
    Vec flop0_offset, flop1_offset;
    TimeFrame stuck_time, stuck_timeout;
    float stuck_distance, tongue_pull_speed_min, tongue_pull_speed_max;
    TimeFrame yellow_attack_timeout;
    float fall_height, mech_jump_thrust_fuel;
    TimeFrame strafe_jump_pre_window;
    Symbol strafe_jump, strafe_duck_jump;
    float dark_jump_height_min, dark_jump_height_max;
};
extern const TargetBank target_bank; // *TARGET-bank*

// *standard-dynamics* (dynamics-h.gc).
extern const Dynamics standard_dynamics;

// ---- set-up (logic-target.gc init-target, reset-target-state) ----
// The movement part of init-target: the control's dynamics copy, pad, current surface, then
// reset-target-state #t and the start of last-trans-any-surf, at `trans`.
void movement_init(Target& self, const Vec& trans);
void reset_target_state(Target& self, bool full);

// ---- clock (timer.gc) ----
void update_rates(Target& self, float ratio); // (update-rates! (-> self clock) ratio)

// ---- logic-target.gc ----
void build_conversions(Target& self, const Vec& v);
void vector_turn_to(Target& self, const Vec& dir);
void reverse_conversions(Target& self, Vec& out);
Vec& read_pad(Target& self, Vec& out);
Vec& warp_vector_into_surface(Vec& out, const Vec& in, const Vec& normal, const Mat& m);
Vec& vector_pad_in_surface(Target& self, Vec& out, bool scale); // vector<-pad-in-surface!
float local_pad_angle(Target& self);
bool turn_around_p(Target& self);
void target_log_trans(Target& self);
float target_move_dist(Target& self, TimeFrame window);
void turn_to_vector(Target& self, const Vec& dir, float magnitude);
Vec& target_bend_vel_turn(Target& self, Vec& v);
Vec& target_add_slide_factor(Target& self, Vec& v);
void add_thrust(Target& self);
void add_gravity(Target& self);
void target_compute_slopes(Target& self, const Vec& up);
void do_rotations1(Target& self);
void do_rotations2(Target& self);
void pre_collide_setup(Target& self);
void flag_setup(Target& self);
void post_flag_setup(Target& self);
bool bend_gravity(Target& self);
void do_target_gspot(Target& self);
void target_real_post(Target& self);
void target_post(Target& self);
void target_no_move_post(Target& self);
void target_no_ja_move_post(Target& self);

// ---- the clock and the control's rotation ----
float frames_per_second(const Target& self);          // (-> self clock frames-per-second)
TimeFrame old_frame_counter(const Target& self);      // (-> self clock old-frame-counter)
float seconds_per_frame_high_fps(const Target& self); // (seconds-per-frame-high-fps), gkernel-h.gc
void rot_to_dir_targ(Target& self);                   // rot->dir-targ! of the control
void set_quaternion(Target& self, const Quat& q);     // set-quaternion! of the control
float fact_health_default_inc();                      // (-> *FACT-bank* health-default-inc)

// ---- powerups.gc, target-gun.gc ----
void target_powerup_process(Target& self);
void target_gun_end_mode(Target& self, bool a0);

// ---- target-util.gc ----
float debounce_speed(float mag, float last_mag, const Vec& dir, const Vec& last_dir);
void target_start_attack(Target& self);
void target_danger_set(Target& self, Symbol mode, Symbol variant);
int target_collide_set(Target& self, Symbol mode, float transition);
float target_height_above_ground(Target& self);
bool can_jump_p(Target& self, Symbol kind);
bool move_legs_p(Target& self);
bool jump_hit_ground_stuck_p(Target& self);
bool can_play_stance_ambient(Target& self); // can-play-stance-amibent? (GOAL's spelling)
TimeFrame target_time_to_ground(Target& self);
bool can_exit_duck_p(Target& self);
bool can_feet_p(Target& self, Symbol kind);
void set_forward_vel(Target& self, float vel);
void delete_back_vel(Target& self);
void target_timed_invulnerable(TimeFrame duration, Target& self, int which);
void target_timed_invulnerable_off(Target& self, int which);

// ---- target.gc ----
const Vec& init_var_jump(Target& self, float min_height, float max_height, bool set_vel,
                         bool collide_offset, Vec& transv, float extra);
void mod_var_jump(Target& self, bool mod_vel, bool anim_offset, bool active, Vec& transv);

// ---- target-handler.gc ----
void target_exit(Target& self);
// The 'push-transv event as target-standard-event-handler handles it.
void target_push_transv(Target& self, const Vec& v, TimeFrame decay_start);

} // namespace jak
