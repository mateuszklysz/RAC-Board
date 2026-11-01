// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The board's states on a flat floor (collide's real collision over tests/board_fakes.hpp's world):
// getting on, standing, a jump and its landing, the go rules (gstate.gc) for a go from another
// process, and target-board-physics' shock and the cushion, against numbers worked out from
// target-board.gc with the same float operations.
#include <algorithm>
#include <string>

#include "board/surfaces.hpp"
#include "board_fakes.hpp"
#include "check.hpp"

using namespace jak;

namespace {
bool run_until(Target& t, fake::Io& io, StateId want, int max, uint16_t buttons = 0) {
    for (int i = 0; i < max; i++) {
        if (runtime(t).state == want) return true;
        fake::frame(t, io, buttons);
    }
    return runtime(t).state == want;
}
bool played(const fake::Fx& fx, const char* name) {
    return std::find(fx.sounds.begin(), fx.sounds.end(), std::string(name)) != fx.sounds.end();
}
} // namespace

int main() {
    Target t;
    fake::Floor w;
    fake::Io io;
    fake::Fx fx;
    BoardRuntime rt;
    fake::make(t, w, io, fx, rt);
    BoardInfo& b = *t.board;

    // ---- target-board-start: init, then get-on at once (a go from code changes state in-frame) ----
    go(t, StateId::target_board_start);
    CHECK(rt.state == StateId::none);            // a go from outside waits for the main thread
    CHECK(next_state_is(t, StateId::target_board_start));
    fake::frame(t, io);
    CHECK(rt.state == StateId::target_board_get_on);
    CHECK(t.focus_status & focus::board);
    CHECK(anim_playing(rt.skel).group == AnimId::jakb_board_get_on);
    // get-on's hop: time-to-ground is long on a floor, so no extra; the hop rises then lands.
    CHECK(0.0f < t.control.transv.y);
    // ---- landing: get-on -> hit-ground -> stance (hit-ground's code goes on in the same frame) ----
    CHECK(run_until(t, io, StateId::target_board_stance, 120));
    CHECK(t.control.status & cstat::on_surface);
    CHECK(anim_playing(rt.skel).group == AnimId::jakb_board_get_on_land);
    CHECK(t.control.mod_surface == &board_walk_mods);

    // ---- the cushion: on the ground it seeks to the bank's cushion (1 m) at 8192 * spf a frame ----
    for (int i = 0; i < 60; i++) fake::frame(t, io);
    CHECK(b.cushion_offset == kTargetBoardBank.cushion);

    // ---- x: a jump (recently-pressed?, can-jump? 'board) ----
    fx.sounds.clear();
    fake::frame(t, io, pad::x);
    CHECK(rt.state == StateId::target_board_jump);
    CHECK(played(fx, "board-jump"));
    CHECK(0.0f < t.control.transv.y);
    CHECK(t.control.mod_surface == &board_jump_mods);
    CHECK(b.trick_count == 1);  // board-jump, no points
    CHECK(anim_playing(rt.skel).group == AnimId::jakb_board_jump);  // the code ran in the same frame
    // in the air the cushion follows the height: lerp-scale cushion..0 over 0..3 m.
    for (int i = 0; i < 10; i++) fake::frame(t, io);
    CHECK(b.cushion_offset < kTargetBoardBank.cushion);
    // (the jump's art plays at a speed from time-to-apex: the jump or its loop by now)
    CHECK(anim_playing(rt.skel).group == AnimId::jakb_board_jump ||
          anim_playing(rt.skel).group == AnimId::jakb_board_jump_loop);
    // ---- the landing: jump -> hit-ground -> stance, with the shock pushed down ----
    CHECK(run_until(t, io, StateId::target_board_stance, 240));
    CHECK(b.jump_land_time != 0);
    CHECK(b.shock_offset <= 0.0f);

    // ---- 'jump from another process: deferred to the next main run, next-state set at once ----
    {
        EventMessage e;
        e.message = Symbol("jump");
        e.f[0] = meters(1);
        e.f[1] = meters(2);
        send_event(t, e);
        CHECK(rt.state == StateId::target_board_stance);
        CHECK(next_state_is(t, StateId::target_board_jump));
        fake::frame(t, io);
        CHECK(rt.state == StateId::target_board_jump);
    }
    CHECK(run_until(t, io, StateId::target_board_stance, 240));

    // ---- L1 held on the ground: duck-stance; let go: stance ----
    fake::frame(t, io, pad::l1);
    CHECK(rt.state == StateId::target_board_duck_stance);
    CHECK(t.control.mod_surface == &board_duck_mods);
    fake::frame(t, io);
    CHECK(rt.state == StateId::target_board_stance);

    // ---- an attack: target-board-hit, then back on the ground ----
    {
        AttackInfo a;
        a.mode = Symbol("generic");
        EventMessage e;
        e.message = Symbol("attack");
        e.attack = &a;
        const float hp = io.hp;
        send_event(t, e);
        fake::frame(t, io);
        CHECK(rt.state == StateId::target_board_hit);
        CHECK(io.hp == hp - 2.0f);  // health-default-inc
        CHECK(run_until(t, io, StateId::target_board_stance, 400));
    }

    // ---- target-board-physics: the shock of a landing at -10 m/s ----
    {
        Target p;
        fake::Floor w2;
        fake::Io io2;
        fake::Fx fx2;
        BoardRuntime rt2;
        fake::make(p, w2, io2, fx2, rt2);
        BoardInfo& pb = *p.board;
        ControlInfo& c = p.control;
        c.mod_surface = &board_walk_mods;
        c.status = cstat::on_surface;
        c.last_time_on_surface = current_time(p);
        c.transv = Vec{0.0f, 0.0f, 0.0f, 1.0f};
        c.ctrl_xz_vel = 0.0f;
        pb.unknown_time_frame02 = 0;  // long ago: the landing counts
        const float spf = seconds_per_frame(p);
        target_board_physics(p, Vec{0.0f, -40960.0f, 0.0f, 1.0f});
        float v = 0.0f + (-40960.0f - 0.0f) * 0.5f;
        v = v + (-10.0f * spf * 0.0f + c.dynam->gravity_length * spf * 0.5f);
        float o = 0.0f + v * spf;
        o = o * lerp_scale(0.99f, 0.98f, 0.0f, 0.0f, 28672.0f);
        CHECK(pb.shock_offsetv == v);
        CHECK(pb.shock_offset == o);
        CHECK_NEAR(pb.shock_offset, -304.128, 0.01);
        CHECK(c.draw_offset.y == 0.0f);  // cushion 0 + shock < 0: clamped to 0
        // A hard one: past -3072 with the speed under -12288: clamped, a bounce at
        // lerp-scale(1, 0.3, v, -40960, -12288) volume.
        pb.shock_offset = -3000.0f;
        pb.shock_offsetv = 0.0f;
        fx2.sounds.clear();
        target_board_physics(p, Vec{0.0f, -40960.0f, 0.0f, 1.0f});
        CHECK(pb.shock_offset == -3072.0f);
        CHECK(pb.shock_offsetv == 0.0f);
        CHECK(played(fx2, "board-bounce"));
        // In the air with the 'air surface the offset only decays.
        c.mod_surface = &board_air_mods;
        c.status = 0;
        c.last_time_on_surface = current_time(p) - seconds(1);
        pb.shock_offset = 1000.0f;
        pb.shock_offsetv = 0.0f;
        pb.unknown_time_frame02 = current_time(p);  // within 0.3 s: no landing push
        target_board_physics(p, Vec{0.0f, -40960.0f, 0.0f, 1.0f});
        CHECK(pb.shock_offsetv == -10.0f * spf * 1000.0f);
        CHECK(pb.shock_offset == 1000.0f * 0.99f);
        // cushion + offsets is the draw offset.
        pb.cushion_base = 100.0f;
        pb.cushion_offset = 4096.0f;
        CHECK(c.draw_offset.y == 0.0f + 0.0f + 990.0f);
    }
    TEST_MAIN_END();
}
