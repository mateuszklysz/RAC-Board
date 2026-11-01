// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Ported from OpenGOAL (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team;
// THIRD_PARTY.md): target-handler.gc, target-util.gc and target-death.gc, as target_hit.hpp lists.
//
// Left out, each marked where it was: dark Jak, the mech, eco and the shield (none on the board in
// the mod), the Daxter lines, compute-intersect-info's touch point (the mod has no touching shapes).
#include "board/target_hit.hpp"

#include <cmath>

#include "board/board.hpp"
#include "board/core.hpp"
#include "jak/collide_from_core.hpp"
#include "jak/collide_mem.hpp"
#include "jak/math.hpp"
#include "jak/math_core.hpp"
#include "jak/surface.hpp"

namespace jak {

namespace {

const Symbol kTrue("#t");

Interaction& io(Target& self) { return *runtime(self).io; }

// (-> self control root-prim prim-core): the body's sphere.
Vec root_center(Target& self) {
    CollideShapePrim* p = root_prim(self);
    return p ? p->prim_core.world_sphere : self.control.trans;
}

// target-log-attack (target-util.gc): false for an attack of the same id too soon after the last.
// 'test only asks; anything else records it over the oldest.
bool target_log_attack(Target& self, const AttackInfo& info, Symbol how) {
    BoardRuntime& rt = runtime(self);
    if (how == Symbol("background") && info.id == 2) {
        return true;
    }
    AttackInfo* oldest = &rt.attack_info_old[0];
    for (AttackInfo& old : rt.attack_info_old) {
        if (info.id == old.id) {
            // (PC port: the base clock, not the target's)
            if (!(io(self).game_clock_frame_counter() - old.attack_time >=
                  target_bank.same_attack_invulnerable_timeout)) {
                return false;
            }
            if (how != Symbol("test")) {
                old = info;
            }
            return true;
        }
        if (old.attack_time < oldest->attack_time) {
            oldest = &old;
        }
    }
    if (how != Symbol("test")) {
        *oldest = info;
    }
    return true;
}

// compute-intersect-info (target-util.gc), without the touch point.
void compute_intersect_info(Target& self, AttackInfo& info, Handle from) {
    if (!(info.mask & attack_mask::attacker)) {
        info.attacker = from;
        info.mask |= attack_mask::attacker;
    }
    if (!(info.mask & attack_mask::attack_time)) {
        info.attack_time = io(self).game_clock_frame_counter();
        info.mask |= attack_mask::attack_time;
    }
    if (!(info.mask & attack_mask::damage)) {
        info.damage = fact_health_default_inc();
    }
}

// velocity-set-to-target! (target-death.gc)
Vec velocity_set_to_target(Target& self, const Vec& to, float speed, const AttackInfo& info) {
    ControlInfo& c = self.control;
    Vec d;
    vector_sub(d, to, c.trans);
    d.y = 0.0f;
    c.force_turn_to_direction = d;
    vector_xz_normalize(c.force_turn_to_direction, 1.0f);
    c.force_turn_to_speed = 1.0f;
    c.unknown_floatiujh1bnb2n3i1 = speed;
    c.force_turn_to_strength = 1.0f - info.control;
    return d;
}

} // namespace

bool attack_mode_is_invinc(Symbol m) {
    for (const char* name : {"dark-eco-pool", "endlessfall", "melt", "lava", "bot", "instant-death", "grenade",
                             "drill-mech-master", "centipede", "drown-death", "crush", "death"}) {
        if (m == Symbol(name)) {
            return true;
        }
    }
    return false;
}

Symbol target_attacked(Target& self, Symbol message, const AttackInfo& info, Handle from, StateId hit_state) {
    BoardRuntime& rt = runtime(self);
    Interaction& ia = io(self);
    const Symbol mode = (info.mask & attack_mask::mode) ? info.mode : Symbol();
    const Symbol first = message;
    if (!((!focus_test(self, focus::hit) && !(self.state_flags & sflags::disable_attacks)) ||
          mode == Symbol("endlessfall"))) {
        return {};
    }
    // ('endlessfall: the user's endlessfall setting, on.)
    // The game's secrets (invulnerable), sf4, the shield and dark eco: none on the board in the mod.
    if (self.state_flags & (sflags::sf2 | state_flags::tinvul1 | state_flags::tinvul2)) {
        if (message == Symbol("attack-invinc")) {
            message = Symbol("attack");
        } else if (message != Symbol("attack-or-shove")) {
            return {};
        }
    } else if (message == Symbol("attack-or-shove") || message == Symbol("attack-invinc")) {
        message = Symbol("attack");
    }
    rt.attack_info_rec = info;
    compute_intersect_info(self, rt.attack_info_rec, from);
    // (not mech)
    if (!target_log_attack(self, rt.attack_info_rec,
                           (info.mask & attack_mask::test) ? Symbol("test") : Symbol("log")) &&
        first != Symbol("attack-invinc")) {
        return {};
    }
    if (info.mask & attack_mask::test) {
        return kTrue;
    }
    self.control.attacked_by_id = static_cast<int32_t>(rt.attack_info_rec.id);
    const AttackInfo& rec = rt.attack_info_rec;
    if ((rec.mask & attack_mask::mode) && rec.mode == Symbol("damage") &&
        !(ia.game_mode() == Symbol("play") && 0.0f >= ia.health() - rec.damage)) {
        ia.pickup_health((rec.mask & attack_mask::damage) ? -rec.damage : -1.0f); // health-single-inc
        particles(self, "group-target-hit",
                  (rec.mask & attack_mask::intersection) ? rec.intersection : root_center(self));
        target_timed_invulnerable(
            (rec.mask & attack_mask::invinc_time) ? rec.invinc_time : target_bank.hit_invulnerable_timeout, self, 1);
        ia.cpad_buzz(0, 255, seconds(0.5));
        sound_play(self, "oof");
        return kTrue;
    }
    self.focus_status |= focus::ignore | focus::hit;
    if (ia.game_mode() == Symbol("play") &&
        ((0.0f >= ia.health() - rec.damage) || attack_mode_is_invinc(mode)) && message == Symbol("attack")) {
        self.focus_status |= focus::dead;
        self.state_flags |= sflags::disable_attacks;
    }
    StateArgs a;
    a.sym = message;
    a.attack = &rt.attack_info_rec;
    go(self, hit_state, a);
    return kTrue;
}

AttackInfo& attack_info_combine(Target& self, AttackInfo& dst, const AttackInfo& src) {
    Interaction& ia = io(self);
    const uint32_t m = src.mask;
    dst.mask = src.mask;
    if (m & attack_mask::attacker) dst.attacker = src.attacker;
    if (m & attack_mask::mode) dst.mode = src.mode;
    if (m & attack_mask::angle) dst.angle = src.angle;
    if (m & attack_mask::dist) dst.dist = src.dist;
    if (m & attack_mask::control) dst.control = src.control;
    if (m & attack_mask::speed) dst.speed = src.speed;
    if (m & attack_mask::penetrate_using) dst.penetrate_using = src.penetrate_using;
    if (m & attack_mask::damage) dst.damage = src.damage;
    if (m & attack_mask::shield_damage) dst.shield_damage = src.shield_damage;
    if (m & attack_mask::knock) dst.knock = src.knock;
    if (m & attack_mask::count) dst.count = src.count;
    if (m & attack_mask::id) dst.id = src.id;
    if (m & attack_mask::shove_back) dst.shove_back = src.shove_back;
    if (m & attack_mask::shove_up) dst.shove_up = src.shove_up;
    if (m & attack_mask::invinc_time) dst.invinc_time = src.invinc_time;
    if (m & attack_mask::rotate_to) dst.rotate_to = src.rotate_to;
    if (m & attack_mask::intersection) dst.intersection = src.intersection;
    if (m & attack_mask::attacker_velocity) dst.attacker_velocity = src.attacker_velocity;
    Vec at;
    if (!(m & attack_mask::vector)) {
        // No vector: away from the attacker, if it is a process-drawable.
        if (ia.process_trans(dst.attacker, at)) {
            dst.trans = at;
            vector_sub(dst.vector, self.control.trans, at);
            dst.mask |= attack_mask::vector;
        }
    } else {
        if (ia.process_trans(dst.attacker, at)) {
            dst.trans = at;
        }
        dst.vector = src.vector;
        if (!(m & attack_mask::shove_back)) {
            dst.shove_back = std::sqrt(dst.vector.x * dst.vector.x + dst.vector.z * dst.vector.z);
        }
        if (!(m & attack_mask::shove_up)) {
            dst.shove_up = dst.vector.y;
        }
    }
    if (!(dst.mask & attack_mask::dist)) {
        dst.dist = std::fabs(dst.shove_back);
    }
    if (m & attack_mask::trans) {
        dst.trans = src.trans;
    }
    return dst;
}

void target_hit_effect(Target& self, const AttackInfo& info) {
    Interaction& ia = io(self);
    particles(self, "group-target-hit", (info.mask & attack_mask::intersection) ? info.intersection : root_center(self));
    const Symbol m = info.mode;
    if (!(m == Symbol("endlessfall") || m == Symbol("instant-death") || m == Symbol("crush"))) {
        target_timed_invulnerable(info.invinc_time, self, 1);
        ia.cpad_buzz(0, 255, seconds(0.5));
        // (focus-test? self indax): never on the board, so Jak's own cry.
        sound_play(self, info.angle == Symbol("up") ? "hit-up" : "oof");
    }
    if (m == Symbol("burn") || m == Symbol("burnup") || m == Symbol("lava")) {
        ia.burn_effect();
    } else if (m == Symbol("tar")) {
        sound_play(self, "get-burned");
    }
}

Symbol target_hit_push(Target& self, Vec& start, const Mat& rot, float dist, float speed, const AttackInfo& info) {
    BoardRuntime& rt = runtime(self);
    Interaction& ia = io(self);
    ControlInfo& c = self.control;
    const Symbol m = info.mode;
    if (m == Symbol("shock") || m == Symbol("shockup") || m == Symbol("shock-red") || m == Symbol("shock-green")) {
        const TimeFrame now = ia.game_clock_frame_counter();
        if (now - rt.shock_effect_time >= seconds(0.03)) {
            rt.shock_effect_time = now;
            // (send-event self 'color-effect 'shock (seconds 0.2)) and the lightning by mode.
            ia.shock_effect();
        }
    }
    const Symbol a = info.angle;
    if (a == Symbol("jump") || a == Symbol("up") || a == Symbol("up-forward")) {
        // (not flut pilot mech indax): none of them on the board.
        if (!(ia.game_mode() == Symbol("play") && 0.0f >= ia.health())) {
            if (target_cpad_pressed(self, pad::circle) && can_feet_p(self, Symbol())) {
                go(self, StateId::target_attack_air);
            }
            const float up = vector_dot(c.dynam->gravity_normal, c.transv);
            // (not (and (not (using-gun? self)) (!= top-anim interp 0.0))): no top animation on the board.
            if (target_cpad_pressed(self, pad::square) && up < 26624.0f && -61440.0f < up &&
                time_elapsed(self, c.last_time_of_stuck, target_bank.stuck_timeout) &&
                !(self.state_flags & state_flags::prevent_attack) &&
                !(c.current_surface->flags & (sflag::no_attack | sflag::no_hands))) {
                go(self, StateId::target_flop);
            }
        }
    }
    if (target_move_dist(self, target_bank.stuck_time) < target_bank.stuck_distance &&
        time_elapsed(self, self.state_time, target_bank.stuck_time) && !ia.cheat_mode_debug()) {
        c.status |= cstat::on_surface;
        return Symbol("stuck");
    }
    Vec to{0.0f, 0.0f, dist, 1.0f};
    vector_matrix_mul(to, to, rot);
    vector_add(to, to, start);
    if (vector_vector_distance(start, to) < std::fabs(info.dist)) {
        velocity_set_to_target(self, to, std::fabs(speed), info);
        return kTrue;
    }
    c.unknown_floatiujh1bnb2n3i1 = 0.0f;
    c.force_turn_to_strength = 1.0f - info.control;
    return {};
}

bool target_hit_orient(Target& self, const AttackInfo& info, const Vec& dir) {
    ControlInfo& c = self.control;
    bool landed = false;
    if (info.angle != Symbol("front") && info.angle != Symbol("shove") && (info.mask & attack_mask::vector) &&
        info.shove_back != 0.0f) {
        forward_up_nopitch_to_quaternion(c.dir_targ, dir, c.dynam->gravity_normal);
    }
    const Symbol a = info.angle;
    if (a == Symbol("back") || a == Symbol("air")) {
        landed = true;
    } else if (a == Symbol("up-forward")) {
        c.mod_surface = &smack_up_mods;
    } else if (a == Symbol("front")) {
        if ((info.mask & attack_mask::vector) && info.shove_back != 0.0f) {
            Vec back;
            vector_negate(back, dir);
            forward_up_nopitch_to_quaternion(c.dir_targ, back, c.dynam->gravity_normal);
        }
        landed = true;
    } else if (a == Symbol("jump")) {
        landed = true;
        c.mod_surface = &forward_high_jump_mods;
    }
    return landed;
}

} // namespace jak
