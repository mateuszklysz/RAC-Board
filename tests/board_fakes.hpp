// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// What the board's tests stand in for: an Interaction that records what the board asks of
// the game, a flat world, and a target set up on it.
#pragma once

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "board/board.hpp"
#include "board/collide.hpp"
#include "board/core.hpp"
#include "board/runtime.hpp"
#include "jak/cpad.hpp"
#include "jak/host.hpp"
#include "jak/math/math.hpp"
#include "jak/movement.hpp"
#include "jak/surface.hpp"
#include "jak/target.hpp"

namespace fake {
using namespace jak;

struct Io : Interaction {
    TimeFrame clock = 0;
    float hp = 3.0f;
    uint32_t attack_id = 0;
    std::vector<BoardTricks> trick_points;
    float flushed = 0.0f;
    int flushes = 0;
    std::vector<StateId> off_board;
    AttackInfo info, rec;

    bool game_feature_board() override { return true; }
    uint32_t next_attack_id() override { return ++attack_id; }
    float health() override { return hp; }
    void pickup_health(float a) override { hp += a; }
    TimeFrame game_clock_frame_counter() override { return clock; }
    void notify_trick_point(BoardTricks t) override { trick_points.push_back(t); }
    void get_pickup_trick_point(float p) override {
        flushed = p;
        flushes++;
    }
    Mat joint_transform(int) override {
        Mat m;
        matrix_identity(m);
        return m;
    }
    Vec get_trans(int) override { return Vec{0.0f, 0.0f, 0.0f, 1.0f}; }
    bool prims_touching(const void*, uint32_t) override { return false; }
    bool target_send_attack(Handle, Symbol, const void*, uint32_t, uint64_t, uint64_t) override { return false; }
    bool send_board_attack(Handle, const void*, uint32_t, uint64_t) override { return false; }
    Symbol standard_event_handler(Target&, const EventMessage&) override { return Symbol(); }
    Symbol generic_event_handler(Target&, const EventMessage&) override { return Symbol(); }
    void go_target_state(Target&, StateId s, const StateArgs&) override { off_board.push_back(s); }
};

struct Fx : Effects {
    std::vector<std::string> sounds;
    std::vector<std::pair<BoardTricks, float>> tricks;
    uint32_t sound_play(const char* n, float, float) override {
        sounds.push_back(n);
        return static_cast<uint32_t>(sounds.size());
    }
    void sound_loop(const char*, uint32_t, float, float) override {}
    void sound_stop(uint32_t) override {}
    void particles(const char*, const Vec&, float) override {}
    void trick(BoardTricks t, float p) override { tricks.emplace_back(t, p); }
    void combo_end() override {}
};

// A floor at y = 0, 40 m each way.
struct Floor : World {
    int triangles(const Vec&, const Vec&, CollideTri* out, int max) override {
        const float e = 163840.0f;
        const Vec n{0.0f, 1.0f, 0.0f, 0.0f};
        const CollideTri t0{{{-e, 0, -e, 1}, {-e, 0, e, 1}, {e, 0, e, 1}}, n, PatSurface{}};
        const CollideTri t1{{{-e, 0, -e, 1}, {e, 0, e, 1}, {e, 0, -e, 1}}, n, PatSurface{}};
        int k = 0;
        if (k < max) out[k++] = t0;
        if (k < max) out[k++] = t1;
        return k;
    }
};

// A target standing on the floor with the board attached, at frame 3000.
inline void make(Target& t, Floor& w, Io& io, Fx& fx, BoardRuntime& rt) {
    t.world = &w;
    t.effects = &fx;
    t.clock.frame_counter = 3000;
    io.clock = 3000;
    movement_init(t, Vec{0.0f, 0.0f, 0.0f, 1.0f});
    target_collide_init(t);
    t.control.local_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
    t.control.gspot_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
    update_rates(t, 1.0f);
    attach(t, rt, io);
}

// One frame: the pad (buttons held this frame), the clocks, then the board's frame.
inline void frame(Target& t, Io& io, uint16_t buttons = 0) {
    t.clock.frame_counter += 5;
    io.clock += 5;
    cpad_set_hw(t.pad, buttons, 128, 128, 128, 128);
    CpadService s;
    s.current_time = t.clock.frame_counter;
    service_cpad(t.pad, s);
    board_tick(t);
}
} // namespace fake
