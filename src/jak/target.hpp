// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The target: Jak II's player process (target-h.gc), the part of it the board reads and writes.
// The ported code takes it as `self`: `(-> self control trans)` is `self.control.trans`.
//
// Who owns what (each part adds its fields in its own block, so their work merges cleanly):
//   movement core  (src/jak/movement*): the clock, the pad, the target's flags and timers
//   collision      (src/jak/collide/):  the collide cache and the edge-grab info
//   board          (src/board/*):       the state machine, the animation channels, the board
#pragma once

#include "jak/basic_types.hpp"
#include "jak/cpad.hpp"
#include "jak/host.hpp"
#include "jak/types_gen.hpp"

namespace jak {

// The process clock (clock-h.gc), as the board reads it: `(current-time)` is frame_counter,
// `(seconds-per-frame)` and `time-adjust-ratio` as the display sets them (60 or 50 Hz).
struct Clock {
    TimeFrame frame_counter = 0;
    float seconds_per_frame = 1.0f / 60.0f;
    float time_adjust_ratio = 1.0f;
    float frames_per_second = 60.0f;  // the inverse of seconds-per-frame, set by update-rates!
    TimeFrame old_frame_counter = -1; // frame_counter on the last engine iteration (the adapter sets it)
};

struct Target {
    ControlInfo control;
    BoardInfo board_info;
    BoardInfo* board = &board_info; // `(-> self board)`; never null in the port
    Clock clock;
    TimeFrame state_time = 0;    // `(-> self state-time)`, set on entering a state
    uint64_t focus_status = 0;   // focus-status bits (target-h.gc)
    uint64_t state_flags = 0;    // state-flags bits (target-h.gc)

    World* world = nullptr;      // the collision geometry (host.hpp)
    Effects* effects = nullptr;  // sounds, particles, tricks (host.hpp)

    // ---- movement core ----
    // Storage the control's pointers point at (movement_init): the surface build-conversions
    // multiplies into, the target's copy of *standard-dynamics*, and the pad (the adapter fills it).
    Surface current_surface_storage{};
    Dynamics dynam_storage{};
    CpadInfo pad{};
    // What read-pad and the clock read from outside the target, set by the adapter each frame:
    // (matrix-local->world #t #f), the camera's rotation (rows: right, up, forward; trans 0, w 1);
    // the display's real clock (*display* real-clock frame-counter); and the clock's rate inputs
    // (update-rates!: clock-ratio, *display* time-factor 5 at 60 Hz or 6 at 50 Hz, dog-ratio for lag).
    Mat camera_local_to_world{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    TimeFrame real_frame_counter = 0;
    float clock_ratio = 1.0f;
    float display_time_factor = 5.0f;
    float display_dog_ratio = 1.0f;
    // (-> *setting-control* user-current speed-mult), 1.0 in the game.
    float speed_mult = 1.0f;
    // (get-video-mode) is 'custom (the PC port's high frame rates): seconds-per-frame-high-fps is
    // then a fixed 1/60.
    bool custom_video_mode = false;
    // (-> *display* base-clock frame-counter) and (-> self ambient-time), for can-play-stance-amibent?.
    TimeFrame base_frame_counter = 0;
    TimeFrame ambient_time = 0;
    // (-> self draw status), draw-control-status bits (do-target-gspot reads no-draw ones).
    uint32_t draw_status = 0;
    // turn-around? shifts 16 entries into the 15 of control transv-history: the 16th lands on the
    // fields after it (average-xz-vel, idx-of-fastest-xz-vel, hand-to-edge-dist and
    // unknown-symbol000). Its w, read back as a float next time, is kept here.
    float transv_history_15_w = 0.0f;
    // ---- collision ----
    // send-event from the collision code (collide.hpp, CollideEvent) to the target's handler;
    // returns the handler's answer (#f as false). Null: every event answers #f.
    bool (*collide_event)(Target& self, const struct CollideEvent& ev) = nullptr;
    // (-> self node-list data i bone transform), the joint's world matrix, for the prims whose
    // transform-index is a joint (sphere 4 follows joint 6). Null: those prims are not moved by
    // update-transforms (move-by-vector! still carries them along).
    const Mat* (*joint_transform)(Target& self, int index) = nullptr;
    // ---- board ----
    // The board's state machine, animation channels and the fields GOAL overlays on others
    // (src/board/runtime.hpp); set by board::attach, null until then.
    struct BoardRuntime* board_rt = nullptr;
};

// (current-time) and (time-elapsed? t d), as the board uses them.
inline TimeFrame current_time(const Target& self) { return self.clock.frame_counter; }
inline bool time_elapsed(const Target& self, TimeFrame t, TimeFrame d) {
    return current_time(self) - t >= d;
}
inline float seconds_per_frame(const Target& self) { return self.clock.seconds_per_frame; }

} // namespace jak
