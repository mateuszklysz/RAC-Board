// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's surface engine and the target's own surfaces, from OpenGOAL's
// goal_src/jak2/engine/target/surface-h.gc (and *no-walk-surface* from
// engine/collide/collide-edge-grab.gc) (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL
// Team; THIRD_PARTY.md).
#include "jak/surface.hpp"

#include <cmath>

#include "jak/collide/from_core.hpp"
#include "jak/math/math.hpp"
#include "jak/target.hpp"

namespace jak {

namespace {

// The data array's order (surface-h.gc deftype).
float Surface::*const kData[kSurfaceDataCount] = {
    &Surface::turnv,          &Surface::turnvv,           &Surface::tiltv,
    &Surface::tiltvv,         &Surface::transv_max,       &Surface::target_speed,
    &Surface::seek0,          &Surface::seek90,           &Surface::seek180,
    &Surface::fric,           &Surface::nonlin_fric_dist, &Surface::slip_factor,
    &Surface::slide_factor,   &Surface::slope_up_factor,  &Surface::slope_down_factor,
    &Surface::slope_slip_angle, &Surface::impact_fric,    &Surface::bend_factor,
    &Surface::bend_speed,     &Surface::alignv,           &Surface::slope_up_traction,
    &Surface::align_speed,    &Surface::slope_change_preserve, &Surface::turnvf,
    &Surface::turnvvf,        &Surface::tiltvf,           &Surface::tiltvvf,
    &Surface::vel_turn,
};

// The hook choice of surface-interp! and surface-mult!: b's when it is set (nonzero and not
// `nothing`; the port has null for both), else a's, else none.
template <typename H> H pick(H a, H b) { return b ? b : (a ? a : nullptr); }

void pick_hooks(Surface& out, const Surface& a, const Surface& b) {
    out.active_hook = pick(a.active_hook, b.active_hook);
    out.touch_hook = pick(a.touch_hook, b.touch_hook);
    out.impact_hook = pick(a.impact_hook, b.impact_hook);
    out.mult_hook = pick(a.mult_hook, b.mult_hook);
    out.exit_hook = pick(a.exit_hook, b.exit_hook);
}

// gravel-surface-touch (collide-reaction-target.gc) does nothing, but the hook is set, so it hides
// the mod's touch hook in surface-mult!.
void gravel_touch(Target&) {}

// rail-surface-touch (collide-reaction-target.gc): the collision's.
void rail_touch(Target& self) { rail_surface_touch(self); }

Surface ground(const char* name) {
    Surface s{};
    s.name = Symbol(name);
    return s;
}

} // namespace

float& surface_data(Surface& s, int i) { return s.*kData[i]; }
float surface_data(const Surface& s, int i) { return s.*kData[i]; }

// surface-interp!
Surface& surface_interp(Surface& out, const Surface& a, const Surface& b, float t) {
    for (int i = 0; i < kSurfaceDataCount; i++) {
        surface_data(out, i) = surface_data(b, i) * t + surface_data(a, i) * (1.0f - t);
    }
    pick_hooks(out, a, b);
    out.mode = b.mode;
    out.flags = b.flags;
    return out;
}

// surface-mult!
Surface& surface_mult(Target& self, Surface& out, const Surface& a, const Surface& b) {
    for (int i = 0; i < kSurfaceDataCount; i++) {
        surface_data(out, i) = surface_data(b, i) * surface_data(a, i);
    }
    pick_hooks(out, a, b);
    out.mode = b.mode;
    out.flags = b.flags;
    out.flags = a.flags | b.flags;
    if (out.mult_hook) {
        out.mult_hook(self, out, a, b, 1);
    }
    return out;
}

// surface-clamp-speed
void surface_clamp_speed(Target&, Surface& out, const Surface& a, const Surface&, int pass) {
    if (pass == 1) {
        out.transv_max = std::fmin(out.transv_max, a.transv_max);
        out.target_speed = std::fmin(out.target_speed, a.target_speed);
    }
}

// calc-terminal-vel
float calc_terminal_vel(float a0, float a1, float a2) {
    return goal_div((a0 / 60.0f) - a1, a2) * (1.0f - a2) - a1;
}

// calc-terminal2-vel
float calc_terminal2_vel(float a0, float a1, float a2, float) {
    const float f0 = std::sqrt(goal_div((a0 / 60.0f) - a1, a2));
    return f0 - (a1 + a2 * (f0 * f0));
}

// calc-terminal4-vel
float calc_terminal4_vel(float a0, float a1, float a2) {
    const float f0 = std::sqrt(std::sqrt(goal_div((a0 / 60.0f) - a1, a2)));
    return f0 - (a1 + a2 * (((f0 * f0) * f0) * f0));
}

// *walk-mods*
Surface walk_mods = [] {
    Surface s{};
    s.name = Symbol("run");
    s.turnv = 131072.0f;
    s.turnvv = 524288.0f;
    s.tiltv = 65536.0f;
    s.tiltvv = 262144.0f;
    s.transv_max = 40960.0f;
    s.target_speed = 40960.0f;
    s.seek0 = 1.0f;
    s.seek90 = 1.0f;
    s.seek180 = 1.0f;
    s.fric = 1.0f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_up_factor = 1.0f;
    s.slope_down_factor = 1.0f;
    s.slope_slip_angle = 1.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.tiltvf = 150.0f;
    s.tiltvvf = 15.0f;
    s.flags = sflag::look_around;
    return s;
}();

// *empty-mods*
Surface empty_mods = [] {
    Surface s{};
    s.name = Symbol("empty");
    s.seek0 = 1.0f;
    s.seek90 = 1.0f;
    s.seek180 = 1.0f;
    s.fric = 1.0f;
    s.flags = sflag::gun_off;
    return s;
}();

// *neutral-mods*
Surface neutral_mods = [] {
    Surface s{};
    s.name = Symbol("walk");
    s.turnv = 131072.0f;
    s.turnvv = 524288.0f;
    s.tiltv = 65536.0f;
    s.tiltvv = 262144.0f;
    s.transv_max = 40960.0f;
    s.target_speed = 40960.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_up_factor = 1.0f;
    s.slope_down_factor = 1.0f;
    s.slope_slip_angle = 1.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.tiltvf = 150.0f;
    s.tiltvvf = 15.0f;
    return s;
}();

// *stone-surface*
Surface stone_surface = [] {
    Surface s = ground("*stone-surface*");
    s.turnv = 1.0f;
    s.turnvv = 1.0f;
    s.tiltv = 1.0f;
    s.tiltvv = 1.0f;
    s.transv_max = 1.0f;
    s.target_speed = 1.0f;
    s.seek0 = 153600.0f;
    s.seek90 = 153600.0f;
    s.seek180 = 256000.0f;
    s.fric = 153600.0f;
    s.nonlin_fric_dist = 5120.0f;
    s.slip_factor = 1.0f;
    s.slope_down_factor = 10240.0f;
    s.slope_slip_angle = 8192.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 0.8f;
    s.bend_speed = 4.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.slope_change_preserve = 1.0f;
    s.turnvf = 1.0f;
    s.turnvvf = 1.0f;
    s.tiltvf = 1.0f;
    s.tiltvvf = 1.0f;
    s.vel_turn = 1.0f;
    return s;
}();

// *gravel-surface*
Surface gravel_surface = [] {
    Surface s = ground("*gravel-surface*");
    s.turnv = 1.0f;
    s.turnvv = 1.0f;
    s.tiltv = 1.0f;
    s.tiltvv = 1.0f;
    s.transv_max = 1.0f;
    s.target_speed = 1.0f;
    s.seek0 = 153600.0f;
    s.seek90 = 153600.0f;
    s.seek180 = 256000.0f;
    s.fric = 153600.0f;
    s.nonlin_fric_dist = 5120.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_down_factor = 32768.0f;
    s.slope_slip_angle = 8192.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 0.8f;
    s.bend_speed = 4.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.slope_change_preserve = 1.0f;
    s.turnvf = 1.0f;
    s.turnvvf = 1.0f;
    s.tiltvf = 1.0f;
    s.tiltvvf = 1.0f;
    s.vel_turn = 1.0f;
    s.touch_hook = gravel_touch;
    return s;
}();

// *edge-surface*
Surface edge_surface = [] {
    Surface s = ground("*edge-surface*");
    s.turnv = 1.0f;
    s.turnvv = 1.0f;
    s.tiltv = 1.0f;
    s.tiltvv = 1.0f;
    s.transv_max = 1.0f;
    s.target_speed = 1.0f;
    s.seek0 = 153600.0f;
    s.seek90 = 153600.0f;
    s.seek180 = 256000.0f;
    s.fric = 30720.0f;
    s.nonlin_fric_dist = 5120.0f;
    s.slip_factor = 1.0f;
    s.slope_down_factor = 18432.0f;
    s.slope_slip_angle = 8192.0f;
    s.bend_factor = 0.8f;
    s.bend_speed = 4.0f;
    s.alignv = 1.0f;
    s.align_speed = 1.0f;
    s.turnvf = 1.0f;
    s.turnvvf = 1.0f;
    s.tiltvf = 1.0f;
    s.tiltvvf = 1.0f;
    s.vel_turn = 1.0f;
    return s;
}();

// *wade-surface*
Surface wade_surface = [] {
    Surface s = ground("*wade-surface*");
    s.turnv = 1.0f;
    s.turnvv = 1.0f;
    s.tiltv = 1.0f;
    s.tiltvv = 1.0f;
    s.transv_max = 0.7f;
    s.target_speed = 0.7f;
    s.seek0 = 153600.0f;
    s.seek90 = 153600.0f;
    s.seek180 = 256000.0f;
    s.fric = 153600.0f;
    s.nonlin_fric_dist = 5120.0f;
    s.slip_factor = 1.0f;
    s.slope_down_factor = 10240.0f;
    s.slope_slip_angle = 8192.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 0.8f;
    s.bend_speed = 4.0f;
    s.alignv = 0.6f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 0.6f;
    s.turnvf = 1.0f;
    s.turnvvf = 1.0f;
    s.tiltvf = 1.0f;
    s.tiltvvf = 1.0f;
    s.vel_turn = 1.0f;
    return s;
}();

// *quicksand-surface*
Surface quicksand_surface = [] {
    Surface s = ground("*quicksand-surface*");
    s.turnv = 0.25f;
    s.turnvv = 1.0f;
    s.tiltv = 1.0f;
    s.tiltvv = 1.0f;
    s.transv_max = 0.5f;
    s.target_speed = 0.5f;
    s.seek0 = 81920.0f;
    s.seek90 = 83968.0f;
    s.seek180 = 165888.0f;
    s.fric = 329728.0f;
    s.nonlin_fric_dist = 5120.0f;
    s.slip_factor = 1.0f;
    s.slope_down_factor = 10240.0f;
    s.slope_slip_angle = 8192.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 0.8f;
    s.bend_speed = 4.0f;
    s.alignv = 0.5f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 0.5f;
    s.turnvf = 1.0f;
    s.turnvvf = 1.0f;
    s.tiltvf = 1.0f;
    s.tiltvvf = 1.0f;
    s.vel_turn = 1.0f;
    return s;
}();

// *tar-surface*
Surface tar_surface = [] {
    Surface s = ground("*tar-surface*");
    s.turnv = 0.25f;
    s.turnvv = 1.0f;
    s.tiltv = 1.0f;
    s.tiltvv = 1.0f;
    s.transv_max = 0.4f;
    s.target_speed = 0.4f;
    s.seek0 = 81920.0f;
    s.seek90 = 83968.0f;
    s.seek180 = 165888.0f;
    s.fric = 329728.0f;
    s.nonlin_fric_dist = 5120.0f;
    s.slip_factor = 1.0f;
    s.slope_down_factor = 10240.0f;
    s.slope_slip_angle = 8192.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 0.8f;
    s.bend_speed = 4.0f;
    s.alignv = 0.5f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 0.5f;
    s.turnvf = 1.0f;
    s.turnvvf = 1.0f;
    s.tiltvf = 1.0f;
    s.tiltvvf = 1.0f;
    s.vel_turn = 1.0f;
    return s;
}();

// *ice-surface*
Surface ice_surface = [] {
    Surface s = ground("*ice-surface*");
    s.turnv = 0.5f;
    s.turnvv = 1.0f;
    s.tiltv = 1.0f;
    s.tiltvv = 1.0f;
    s.transv_max = 1.5f;
    s.target_speed = 1.5f;
    s.seek0 = 24576.0f;
    s.seek90 = 24576.0f;
    s.seek180 = 24576.0f;
    s.fric = 23756.8f;
    s.nonlin_fric_dist = 4091904.0f;
    s.slip_factor = 0.7f;
    s.slope_slip_angle = 16384.0f;
    s.bend_speed = 4.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.turnvf = 1.0f;
    s.turnvvf = 1.0f;
    s.tiltvf = 1.0f;
    s.tiltvvf = 1.0f;
    s.vel_turn = 1.0f;
    s.flags = sflag::no_turn_around | sflag::turn_to_pad;
    return s;
}();

// *rail-surface*
Surface rail_surface = [] {
    Surface s = ground("*rail-surface*");
    s.turnv = 1.0f;
    s.turnvv = 1.0f;
    s.tiltv = 1.0f;
    s.tiltvv = 1.0f;
    s.transv_max = 1.0f;
    s.target_speed = 1.0f;
    s.seek0 = 153600.0f;
    s.seek90 = 153600.0f;
    s.seek180 = 256000.0f;
    s.fric = 153600.0f;
    s.nonlin_fric_dist = 5120.0f;
    s.slip_factor = 1.0f;
    s.slope_down_factor = 10240.0f;
    s.slope_slip_angle = 8192.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 0.8f;
    s.bend_speed = 4.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.slope_change_preserve = 1.0f;
    s.turnvf = 1.0f;
    s.turnvvf = 1.0f;
    s.tiltvf = 1.0f;
    s.tiltvvf = 1.0f;
    s.vel_turn = 1.0f;
    s.flags = sflag::no_attack;
    s.touch_hook = rail_touch;
    return s;
}();

// *no-walk-surface* (collide-edge-grab.gc)
Surface no_walk_surface = [] {
    Surface s = ground("*no-walk-surface*");
    s.turnv = 0.5f;
    s.turnvv = 1.0f;
    s.tiltv = 1.0f;
    s.tiltvv = 1.0f;
    s.transv_max = 0.7f;
    s.target_speed = 0.7f;
    s.seek0 = 24576.0f;
    s.seek90 = 24576.0f;
    s.seek180 = 24576.0f;
    s.fric = 23756.8f;
    s.nonlin_fric_dist = 4091904.0f;
    s.slope_slip_angle = 16384.0f;
    s.bend_speed = 4.0f;
    s.alignv = 0.5f;
    s.slope_up_traction = 0.9f;
    s.align_speed = 1.0f;
    s.turnvf = 0.5f;
    s.turnvvf = 1.0f;
    s.tiltvf = 1.0f;
    s.tiltvvf = 1.0f;
    s.flags = sflag::no_turn_around | sflag::turn_to_vel;
    return s;
}();

// *grab-mods*: (copy *walk-mods*) with the name grab and no flags.
Surface grab_mods = [] {
    Surface s = walk_mods;
    s.name = Symbol("grab");
    s.flags = 0;
    return s;
}();

namespace {
// *smack-mods*'s mult-hook: on pass 1, pull the seeks and top speeds toward a forced turn's values
// by its strength.
void smack_mult_hook(Target& self, Surface& a0, const Surface&, const Surface&, int a3) {
    if (a3 == 1) {
        const float f30 = self.control.unknown_floatiujh1bnb2n3i1;
        const float f28 = self.control.force_turn_to_strength;
        a0.seek0 = lerp(a0.seek0, 409600.0f, f28);
        a0.seek90 = lerp(a0.seek90, 409600.0f, f28);
        a0.seek180 = lerp(a0.seek180, 409600.0f, f28);
        a0.target_speed = lerp(a0.target_speed, f30, f28);
        a0.transv_max = lerp(a0.transv_max, f30, f28);
    }
}
} // namespace

// *smack-mods* (target-death.gc)
Surface smack_mods = [] {
    Surface s{};
    s.name = Symbol("jump");
    s.tiltv = 32768.0f;
    s.tiltvv = 131072.0f;
    s.transv_max = 40960.0f;
    s.target_speed = 40960.0f;
    s.seek0 = 0.3f;
    s.seek90 = 0.3f;
    s.seek180 = 0.3f;
    s.fric = 0.05f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_up_factor = 1.0f;
    s.slope_down_factor = 1.0f;
    s.slope_slip_angle = 1.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.tiltvf = 150.0f;
    s.tiltvvf = 15.0f;
    s.mult_hook = smack_mult_hook;
    s.mode = Symbol("air");
    s.flags = sflag::air | sflag::gun_inactive | sflag::gun_no_twist;
    return s;
}();

// *smack-up-mods* (target-death.gc): *smack-mods* that turns to the pad.
Surface smack_up_mods = [] {
    Surface s = smack_mods;
    s.turnv = 131072.0f;
    s.turnvv = 32768.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.flags = sflag::no_turn_around | sflag::turn_to_pad | sflag::air | sflag::gun_inactive | sflag::gun_no_twist;
    return s;
}();

// *forward-high-jump-mods* (surface-h.gc)
Surface forward_high_jump_mods = [] {
    Surface s{};
    s.name = Symbol("high-jump");
    s.turnv = 131072.0f;
    s.turnvv = 65536.0f;
    s.tiltv = 32768.0f;
    s.tiltvv = 262144.0f;
    s.transv_max = 45056.0f;
    s.target_speed = 45056.0f;
    s.seek0 = 0.9f;
    s.seek90 = 0.9f;
    s.seek180 = 0.9f;
    s.fric = 0.3f;
    s.nonlin_fric_dist = 1.0f;
    s.slip_factor = 1.0f;
    s.slide_factor = 1.0f;
    s.slope_up_factor = 1.0f;
    s.slope_down_factor = 1.0f;
    s.slope_slip_angle = 1.0f;
    s.impact_fric = 1.0f;
    s.bend_factor = 1.0f;
    s.bend_speed = 1.0f;
    s.alignv = 1.0f;
    s.slope_up_traction = 1.0f;
    s.align_speed = 1.0f;
    s.turnvf = 30.0f;
    s.turnvvf = 30.0f;
    s.tiltvf = 150.0f;
    s.tiltvvf = 15.0f;
    s.mode = Symbol("air");
    s.flags = sflag::check_edge | sflag::air;
    return s;
}();

} // namespace jak
