// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// How the target takes a hit on the board (src/board/target_hit.cpp): target-attacked's checks,
// target-log-attack's same-id timeout and combine!'s fields.
#include <algorithm>
#include <string>

#include "board/target_hit.hpp"
#include "board_fakes.hpp"
#include "check.hpp"

using namespace jak;

namespace {
// An attack as the adapter sends one: an id, a mode and a damage.
AttackInfo attack(uint32_t id, const char* mode, float damage) {
    AttackInfo a;
    a.id = id;
    a.mode = Symbol(mode);
    a.damage = damage;
    a.mask = attack_mask::id | attack_mask::mode | attack_mask::damage;
    return a;
}
} // namespace

int main() {
    // ---- attack-mode-is-invinc ----
    CHECK(attack_mode_is_invinc(Symbol("lava")));
    CHECK(attack_mode_is_invinc(Symbol("death")));
    CHECK(!attack_mode_is_invinc(Symbol("generic")));
    CHECK(!attack_mode_is_invinc(Symbol()));

    // ---- target-attacked: a hit goes to target-board-hit and logs the id ----
    {
        Target t;
        fake::Floor w;
        fake::Io io;
        fake::Fx fx;
        BoardRuntime rt;
        fake::make(t, w, io, fx, rt);
        io.hp = 8.0f;
        const AttackInfo a = attack(7, "generic", 1.0f);
        CHECK(target_attacked(t, Symbol("attack"), a, Handle{}, StateId::target_board_hit) == Symbol("#t"));
        CHECK(next_state_is(t, StateId::target_board_hit));
        CHECK(focus_test(t, focus::hit));
        CHECK(rt.attack_info_rec.id == 7);
        CHECK(rt.attack_info_rec.attack_time == io.clock);  // compute-intersect-info sets it
        CHECK(t.control.attacked_by_id == 7);
    }

    // ---- target-log-attack: the same id again within the timeout is turned away ----
    {
        Target t;
        fake::Floor w;
        fake::Io io;
        fake::Fx fx;
        BoardRuntime rt;
        fake::make(t, w, io, fx, rt);
        io.hp = 8.0f;
        const AttackInfo a = attack(9, "damage", 1.0f);
        // 'damage with health left: no state change, only the health, the sound and the flash.
        CHECK(target_attacked(t, Symbol("attack"), a, Handle{}, StateId::target_board_hit) == Symbol("#t"));
        CHECK(io.hp == 7.0f);
        CHECK(!next_state_is(t, StateId::target_board_hit));
        CHECK(std::find(fx.sounds.begin(), fx.sounds.end(), std::string("oof")) != fx.sounds.end());
        // The same id a frame later: ignored.
        io.clock += 5;
        CHECK(target_attacked(t, Symbol("attack"), a, Handle{}, StateId::target_board_hit) == Symbol());
        CHECK(io.hp == 7.0f);
        // After same-attack-invulnerable-timeout it counts again (if not invulnerable by then).
        io.clock += target_bank.same_attack_invulnerable_timeout;
        t.state_flags &= ~(state_flags::tinvul1 | state_flags::tinvul2 | sflags::sf2);
        CHECK(target_attacked(t, Symbol("attack"), a, Handle{}, StateId::target_board_hit) == Symbol("#t"));
        CHECK(io.hp == 6.0f);
        // A 'test attack only asks (past the invulnerable time the last hit started).
        t.state_flags &= ~(state_flags::tinvul1 | state_flags::tinvul2 | sflags::sf2);
        AttackInfo test = attack(11, "generic", 1.0f);
        test.mask |= attack_mask::test;
        CHECK(target_attacked(t, Symbol("attack"), test, Handle{}, StateId::target_board_hit) == Symbol("#t"));
        CHECK(!next_state_is(t, StateId::target_board_hit));
    }

    // ---- combine!: the fields the mask names, and the shove from the vector ----
    {
        Target t;
        fake::Floor w;
        fake::Io io;
        fake::Fx fx;
        BoardRuntime rt;
        fake::make(t, w, io, fx, rt);
        AttackInfo dst;
        dst.damage = 5.0f;
        dst.angle = Symbol("up");
        AttackInfo src = attack(3, "generic", 2.0f);
        src.vector = Vec{3.0f, 2.0f, 4.0f, 1.0f};
        src.mask |= attack_mask::vector;
        attack_info_combine(t, dst, src);
        CHECK(dst.id == 3 && dst.mode == Symbol("generic") && dst.damage == 2.0f);
        CHECK(dst.angle == Symbol("up"));     // not in the mask: kept
        CHECK(dst.shove_back == 5.0f);        // the vector's length in xz
        CHECK(dst.shove_up == 2.0f);          // its y
    }

    TEST_MAIN_END();
}
