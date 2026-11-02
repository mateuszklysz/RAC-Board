// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The trick list and its points (board-info's add-to-trick-list and flush-trick-list,
// target-board.gc), the PC port's board-trick-tracker (target-board.gc, its display through
// jak::Effects) and want-to-board? (board-h.gc), from OpenGOAL (jak-project, ISC License, Copyright
// (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
#include <cmath>
#include <cstring>

#include "board/board.hpp"
#include "board/collide.hpp"
#include "board/core.hpp"
#include "jak/math/math.hpp"

namespace jak {

namespace {

// trick-points-array is followed in board-info by trick-list (offset 3272 + 64 = 3336), and
// trick-count is never capped: flush-trick-list reads entries past 16 out of trick-list as floats
// (tiny denormals). Past trick-list (index 32 and on) lies pad and the end of the structure; the
// port reads 0 there.
float trick_points_raw(const BoardInfo& b, int i) {
    if (i < 16) {
        return b.trick_points_array[i];
    }
    if (i < 32) {
        float f;
        const uint32_t u = static_cast<uint32_t>(b.trick_list[i - 16]);
        std::memcpy(&f, &u, 4);
        return f;
    }
    return 0.0f;
}

} // namespace

// ---- board-trick-tracker (PC port) ----
void reset_combo(Target& self) {
    BoardTrickTracker& t = runtime(self).trick_tracker;
    t.num_tricks_in_combo = 0;
    t.points_in_combo = 0.0f;
    for (int i = 15; i >= 0; i--) {
        t.tricks_in_combo[i] = BoardTricks::none;
    }
}

void add_trick_to_combo(Target& self, BoardTricks trick, float points) {
    BoardTrickTracker& t = runtime(self).trick_tracker;
    if (!t.combo_in_progress) {
        t.combo_in_progress = true;
        reset_combo(self);
    }
    if (t.num_tricks_in_combo < 16) {
        t.tricks_in_combo[t.num_tricks_in_combo] = trick;
        t.num_tricks_in_combo = t.num_tricks_in_combo + 1;
        t.points_in_combo = t.points_in_combo + points;
        // render-combo is the display's: the host shows the trick with its points.
        if (self.effects) {
            self.effects->trick(trick, points);
        }
    }
}

void end_combo(Target& self) {
    runtime(self).trick_tracker.combo_in_progress = false;
    if (self.effects) {
        self.effects->combo_end();
    }
}

// ---- add-to-trick-list: a trick and its points, less for repeats ----
void add_to_trick_list(Target& self, BoardTricks arg0, float arg1) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    rt.io->notify_trick_point(arg0);
    if (b.trick_count < 16 && (0.0f < arg1 && rt.io->trick_points_enabled())) {
        int s4 = 0;
        if (rt.io->game_clock_frame_counter() - rt.io->trick_point_pickup_time() >= seconds(30)) {
            for (int i = 15; i >= 0; i--) {
                b.trick_list[0] = BoardTricks::none; // (GOAL clears entry 0 sixteen times)
            }
        }
        for (int v1 = 14; v1 >= 0; v1--) {
            if (b.trick_list[v1] == arg0) {
                s4 += 1;
            }
            b.trick_list[v1 + 1] = b.trick_list[v1];
        }
        b.trick_list[0] = arg0;
        const float a0_24 = arg1 * lerp_scale(1.0f, 0.1f, static_cast<float>(s4), 2.0f, 6.0f);
        // Rounded up to a multiple of 25: (* 25 (/ (+ (the int x) 24) 25)), integer division.
        const float s3_1 = static_cast<float>(25 * ((static_cast<int32_t>(a0_24) + 24) / 25));
        sound_play(self, "do-trick");
        // (format "trick: ~S (~D) ~,,0f": debug)
        float total_points = s3_1;
        if (b.trick_count != 0) {
            // The combo: this trick's points plus the last one's, and the last one's double.
            total_points = total_points + b.trick_points_array[b.trick_count + -1];
            b.trick_points_array[b.trick_count + -1] =
                b.trick_points_array[b.trick_count + -1] + b.trick_points_array[b.trick_count + -1];
        }
        b.trick_array[b.trick_count] = arg0;
        b.trick_points_array[b.trick_count] = s3_1;
        add_trick_to_combo(self, arg0, total_points); // PC port
    }
    b.trick_count = b.trick_count + 1;
}

// ---- flush-trick-list: give out the points ----
void flush_trick_list(Target& self) {
    BoardRuntime& rt = runtime(self);
    BoardInfo& b = *self.board;
    float f30 = 0.0f;
    for (int i = 0; i < b.trick_count; i++) {
        f30 = f30 + trick_points_raw(b, i);
    }
    if (0.0f < f30) {
        rt.io->notify_trick_flush(f30, b.trick_list, b.trick_count);
        rt.io->get_pickup_trick_point(f30);
        sound_play(self, "trick-score");
        end_combo(self); // PC port
    }
    b.trick_count = 0;
}

// ---- want-to-board? (board-h.gc): R2 pressed (or latched) and room to get on ----
bool want_to_board_p(Target& self) {
    BoardRuntime& rt = runtime(self);
    ControlInfo& c = self.control;
    if (!rt.io->game_feature_board()) {
        return false;
    }
    const bool press = target_cpad_pressed(self, boardpad::r2) &&
                       (!rt.io->cheat_mode_debug() || !target_cpad_hold(self, boardpad::l2)) &&
                       !rt.io->pause_lock() && time_elapsed(self, c.time_of_last_debug_heal, seconds(0.1)) &&
                       c.last_time_on_surface >= c.time_of_last_debug_float;
    if (!(press || self.board->latch_p)) {
        return false;
    }
    if (self.focus_status & (focus::dead | focus::hit | focus::grabbed | focus::in_head | focus::edge_grab |
                             focus::pole | focus::board | focus::pilot | focus::mech | focus::dark)) {
        return false;
    }
    // ((zero? (-> self board)): the port's board-info always exists, so only the time counts)
    if (!(!rt.board_setup || time_elapsed(self, self.board->board_time, seconds(0.5)))) {
        return false;
    }
    if (self.state_flags & sflags::prevent_board) {
        return false;
    }
    if (!(self.board->board_time < c.list_time_on_ground)) {
        return false;
    }
    if (c.current_surface->flags & sflag::no_board) {
        return false;
    }
    if ((c.current_surface->flags & sflag::duck) && !can_exit_duck_p(self)) {
        return false;
    }
    const uint32_t wf = rt.io->water_flags();
    if ((wf & waterflag::under_water) && !(wf & waterflag::swim_ground)) {
        return false;
    }
    if (rt.io->artist_fix_visible()) {
        return false;
    }
    // Room for Jak standing on the board: three spheres above the feet.
    Vec s5[3];
    Vec v1 = c.trans;
    if (focus_test(self, focus::on_water)) {
        v1.y = rt.io->water_height();
    }
    s5[0] = v1;
    s5[0].y = s5[0].y + 8192.0f;
    s5[0].w = 2867.2f;
    s5[1] = v1;
    s5[1].y = s5[1].y + 12288.0f;
    s5[1].w = 2867.2f;
    s5[2] = v1;
    s5[2].y = s5[2].y + 16384.0f;
    s5[2].w = 2867.2f;
    CollideQuery q;
    q.spheres = s5;
    q.num_spheres = 3;
    q.collide_with = root_prim_collide_with(self) &
                     ~(cspec::civilian | cspec::enemy | cspec::vehicle_sphere | cspec::projectile);
    q.ignore_self = false; // ignore-process0 #f
    q.ignore_pat.bits = (1u << 0) | (1u << 4) | (1u << 24) | (1u << 27); // noentity nojak probe noendlessfall
    q.action_mask = caction::solid;
    if (fill_and_probe_using_spheres(self, q)) {
        return false;
    }
    self.board->latch_p = Symbol("#t");
    return time_elapsed(self, rt.io->gun_time(), seconds(0.4));
}

} // namespace jak
