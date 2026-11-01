// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The board's routines: target-board.gc, the helpers of board-states.gc, board-h.gc and
// board-util.gc, ported from OpenGOAL (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL
// Team; THIRD_PARTY.md) with GOAL's names. The states themselves are in board_states.cpp.
#pragma once

#include "board/runtime.hpp"
#include "jak/movement.hpp"
#include "jak/target.hpp"

namespace jak {

// focus-status bits: core's focus:: (jak/movement.hpp), and the one it lacks.
namespace focus {
constexpr uint64_t dead = 1u << 1;
}
// state-flags bits (game-h.gc).
namespace sflags {
constexpr uint64_t sf2 = 1u << 2, sf6 = 1u << 6, prevent_jump = 1u << 7, prevent_duck = 1u << 9,
                   prevent_board = 1u << 17, lleg_no_ik = 1u << 21, rleg_no_ik = 1u << 22,
                   disable_attacks = 1u << 24;
}
// water-flags bits (water-h.gc) the board tests.
namespace waterflag {
constexpr uint32_t active = 1u << 0, touch_water = 1u << 1, under_water = 1u << 3,
                   swim_ground = 1u << 4, jump_out = 1u << 20;
}

// *TARGET_BOARD-bank* (board-h.gc).
struct TargetBoardBank {
    float jump_height_min = meters(1.01);
    float jump_height_max = meters(3.5);
    float duck_jump_height_min = meters(2.5);
    float duck_jump_height_max = meters(5);
    float turn_frames = 10.0f;
    TimeFrame wall_kick_window = seconds(0.05);
    float cushion = meters(1);
    float trickx_jump_height_min = meters(0.9);
    float trickx_jump_height_max = meters(1.2);
    float tricky_jump_height_min = meters(0.9);
    float tricky_jump_height_max = meters(1.2);
};
extern const TargetBoardBank kTargetBoardBank;

// ---- board-h.gc ----
bool want_to_board_p(Target& self);
// board-info methods.
void add_to_trick_list(Target& self, BoardTricks trick, float points);
void flush_trick_list(Target& self);
// The PC port's trick tracker (board-trick-tracker methods).
void reset_combo(Target& self);
void add_trick_to_combo(Target& self, BoardTricks trick, float points);
void end_combo(Target& self);

// ---- target-board.gc ----
Symbol target_board_handler(Target& self, const EventMessage& e);
void target_board_setup(Target& self, bool spawn_board);
void target_board_init(Target& self);
void target_board_exit(Target& self);
void target_board_falling_anim_trans(Target& self);
bool board_on_ground_p(const Target& self);
bool target_board_smack_surface_p(Target& self);
void board_add_thrust(Target& self);
float segment_array_vector_to_index(const Vec* arr, int len, const Vec& pt, float add);
int segment_array_index_to_vector(const Vec* arr, int len, float index, Vec& out);
void target_board_exit_check(Target& self);
void target_board_effect(Target& self);
void target_board_physics(Target& self, const Vec& old_transv);
void target_board_collision(Target& self);
void target_board_joint_points(Target& self);
void target_board_pre_move(Target& self);
void target_board_real_post(Target& self);
Symbol probe_test(Target& self, const Vec& move, PatSurface ignore);
void target_board_compute_edge(Target& self);
void board_ride_add_thrust(Target& self, const Vec& dir, float mag);
void target_board_ride_post(Target& self);
void target_board_post(Target& self);
void target_board_center_anim(Target& self);
void target_board_turn_anim(Target& self, int chan);

// ---- board-states.gc helpers ----
float vector_vector_angle(const Vec& a, const Vec& b);
void target_board_anim_trans(Target& self);
void target_board_spin_check(Target& self);
void target_board_halfpipe_trans(Target& self);
void target_board_resolve_points(Target& self);
void target_board_ground_check(Target& self);
void target_board_halfpipe_check(Target& self);
void target_board_jump_trans(Target& self);

// ---- board-util.gc: the board's own process ----
void board_init(Target& self);       // spawns it (board-init: idle #t)
void board_deactivate(Target& self);
void board_process_event(Target& self, Symbol message); // 'open / 'close
void board_process_tick(Target& self);

// What core calls in the board (board_from_core.hpp).
void joint_points(Target& self);
int ja_group_collide_offset(Target& self, Vec& out);

// Small GOAL idioms the code repeats.
inline TimeFrame& set_time(Target& self, TimeFrame& t) { t = current_time(self); return t; }
inline bool focus_test(const Target& self, uint64_t bits) { return (self.focus_status & bits) != 0; }
// The y-angle method of trsqv (transformq.gc) on the control.
float control_y_angle(const Target& self);
uint32_t sound_play(Target& self, const char* name, float volume = 1.0f, float pitch = 0.0f);
void sound_stop(Target& self, uint32_t& id);
// The set-param that fades a sound out over `time` (volume -4, auto-time): a stop for the port.
void sound_fade(Target& self, uint32_t id, int auto_time);
void particles(Target& self, const char* group, const Vec& at, float rate = 1.0f);

} // namespace jak
