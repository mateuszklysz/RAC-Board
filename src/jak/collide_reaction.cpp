// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The target's reaction to a collision (collide-reaction-target.gc), ported from OpenGOAL
// (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). The
// *debug-segment* history records are left out.
#include <cmath>

#include "jak/collide_extern.hpp"
#include "jak/collide_internal.hpp"
#include "jak/math.hpp"
#include "jak/math_collide.hpp"
#include "jak/target.hpp"

namespace jak {

namespace {
CollideShapePrim& best_my_prim(GoalCollideQuery& q) {
    return goal_ref<CollideShapePrim>(q.best_my_prim() - 4);
}
} // namespace

// target-collision-low-coverage
void target_collision_low_coverage(Target& self, GoalCollideQuery& arg1, const Vec& contact_normal,
                                   uint32_t& sv20, CollideStatus& sv24, bool& sv28) {
    ControlInfo& c = self.control;
    const Vec& gn = c.dynam->gravity_normal;
    uint32_t sv32 = sv20;
    CollideStatus sv40 = sv24;
    bool sv48 = sv28;
    Vec& tangent = c.low_coverage_tangent;
    {
        Vec& overhang = vector_cross(c.low_coverage_overhang_plane_normal, c.poly_normal, contact_normal);
        vector_normalize(overhang, 1.0f);
        if (std::fabs(vector_dot(gn, overhang)) >= 0.866f) sv32 |= csrf(9);
        vector_cross(tangent, contact_normal, overhang);
    }
    if (0.0f < vector_dot(gn, tangent)) vector_negate(tangent, tangent);
    vector_normalize(tangent, 1.0f);
    {
        GoalStackFrame f;
        GoalCollideQuery& s4 = new_stack_query(f);
        Vec& s3 = vector_flatten(c.low_coverage_tangent_xz, tangent, gn);
        s4.radius() = 409.6f;
        s4.collide_with = best_my_prim(arg1).prim_core.collide_with;
        s4.ignore_process0 = kGoalFalse;
        s4.ignore_process1 = kGoalFalse;
        s4.ignore_pat = c.pat_ignore_mask;
        s4.action_mask = caction::solid;
        vector_normalize(s3, 1.0f);
        if (vector_dot(s3, contact_normal) < 0.0f) vector_negate(s3, s3);
        c.low_coverage_slope_to_next1 = 4095996000.0f;
        c.low_coverage_dist_to_next2 = 4095996000.0f;
        vector_add_float_mul(s4.start_pos, arg1.bot_intersect, s3, 2867.2f);
        vector_normalize_copy(s4.move_dist, gn, -20480.0f);
        vector_add_float_mul(s4.move_dist, s4.move_dist, s3, 4096.0f);
        if (probe_using_line_sphere(s4) >= 0.0f) {
            Vec d;
            c.low_coverage_slope_to_next1 = vector_dot(gn, vector_sub(d, arg1.bot_intersect, s4.bot_intersect));
            c.low_coverage_pat_next1 = s4.bot_pat;
            c.low_coverage_norm_of_next1 = s4.bot_normal;
        }
        vector_add_float_mul(s4.start_pos, arg1.bot_intersect, gn, 819.2f);
        vector_normalize_copy(s4.move_dist, gn, -20480.0f);
        vector_add_float_mul(s4.start_pos, s4.start_pos, s3, -819.2f);
        vector_add_float_mul(s4.move_dist, s4.move_dist, s3, -4096.0f);
        if (probe_using_line_sphere(s4) >= 0.0f) {
            c.low_coverage_dist_to_next2 = vector_vector_distance(s4.start_pos, s4.bot_intersect);
            c.low_coverage_pat_next2 = s4.bot_pat;
            c.low_coverage_norm_of_next2 = s4.bot_normal;
        }
    }
    const float r = best_my_prim(arg1).local_sphere.w;
    const uint32_t m1 = c.low_coverage_pat_next1.mode();
    const uint32_t m2 = c.low_coverage_pat_next2.mode();
    if (!(sv32 & csrf(9)) &&
        ((1.25f * r < c.low_coverage_slope_to_next1 || m1 == kPatWall) &&
         (c.low_coverage_dist_to_next2 < 2.0f * r && (m2 == kPatGround || m2 == kPatHalfpipe)))) {
        sv32 |= csrf(7);
        c.time_of_last_lc_touch_edge = current_time(self);
        sv40 |= cstat::touch_edge;
        c.time_of_last_lc = current_time(self);
        const float f30 = vector_dot(tangent, c.turn_to_target);
        const float f0 = (sv32 & csrf(1)) ? cos(16384.0f - acos(c.coverage)) : c.coverage;
        Vec d;
        const float f1 = vector_dot(gn, vector_sub(d, c.trans, arg1.bot_intersect));
        if (!(sv32 & csrf(5)) || 0.5f < f0) sv48 = false;
        if (((f0 < 0.95f && f30 >= 0.0f) || ((sv32 & csrf(5)) && f0 < 0.3f) ||
             f1 < best_my_prim(arg1).local_sphere.w / -4.0f) &&
            vector_dot(tangent, contact_normal) >= -0.000001f) {
            c.surf = &edge_surface;
            sv32 |= csrf(10);
            sv48 = false;
            if (sv32 & csrf(5)) {
                sv48 = true;
                sv32 |= csrf(12);
            }
        }
    }
    if (c.surface_angle < 0.0f) sv48 = true;
    if (sv48) {
        const uint32_t pm = c.poly_pat.mode();
        if ((pm == kPatGround || pm == kPatHalfpipe ||
             ((sv32 & csrf(3)) && 1.25f * r >= c.low_coverage_slope_to_next1 &&
              (m1 == kPatGround || m1 == kPatHalfpipe) && 0.3f < std::fabs(c.surface_angle))) &&
            !(sv32 & csrf(7))) {
            sv32 |= csrf(6);
            sv40 |= cstat::touch_edge;
            c.time_of_last_lc = current_time(self);
            sv48 = false;
        } else {
            sv32 |= csrf(6);
            sv40 |= cstat::touch_edge;
            c.time_of_last_lc = current_time(self);
        }
    }
    sv24 = sv40;
    sv20 = sv32;
    sv28 = sv48;
}

// target-collision-reaction. arg2 (out) and arg3 (in) are the same vector when step-collison!
// runs it; the incoming velocity is copied first, as GOAL does.
CollideStatus target_collision_reaction(Target& self, GoalCollideQuery& arg1, Vec& arg2,
                                        const Vec& arg3) {
    ControlInfo& c = self.control;
    const Vec& gn = c.dynam->gravity_normal;
    Vec sv80{}, sv84{};
    Vec sv88[2] = {arg3, arg3};
    CollideStatus sv96 = 0;
    uint32_t sv104 = 0;
    {
        Vec a1;
        vector_float_mul(a1, arg1.move_dist, arg1.best_dist());
        move_by_vector(self, a1);
    }
    sv104 |= react_to_pat(self, arg1.bot_pat);
    if (c.poly_pat.mode() == kPatWall) sv104 |= csrf(0);
    if (c.mod_surface->flags & sflag::air) sv104 |= csrf(5);
    CollideShapePrim& me = best_my_prim(arg1);
    {
        const Vec v1 = me.prim_core.world_sphere;
        vector_sub(sv80, v1, arg1.bot_intersect);
    }
    vector_normalize(sv80, 1.0f);
    c.coverage = vector_dot(sv80, arg1.bot_normal);
    if (c.coverage < 0.0f) {
        c.coverage = 0.0f;
        vector_flatten(sv80, sv80, arg1.bot_normal);
        vector_normalize(sv80, 1.0f);
    }
    if (c.coverage < 0.9999f) sv104 |= csrf(3) | csrf(4);
    sv84 = sv80;
    if (arg1.best_dist() == 0.0f) {
        Vec n{};
        move_by_vector(self, vector_normalize_copy(n, sv84, 3.0f));
    }
    c.poly_normal = arg1.bot_normal;
    collide_shape_moving_angle_set(self, sv84, sv88[0]);
    if (c.poly_angle < -0.2f) sv96 |= cstat::touch_ceiling;
    if (c.poly_angle < 0.0f) sv96 |= cstat::touch_ceiling_sticky;
    if (me.prim_id != 6) sv104 |= csrf(15);
    if (c.cur_pat.mode() == kPatHalfpipe && c.poly_angle < 0.75f) {
        CollideEvent e;
        e.message = "slide";
        send_event(self, e);
    }
    bool sv240 = std::fabs(c.surface_angle) < pat_mode_wall_angle(c.cur_pat.mode());
    if (sv240 && (c.mod_surface->mode == Symbol("dive") && 0.4f < std::fabs(c.surface_angle))) {
        sv240 = false;
    }
    if ((sv104 & csrf(15)) && (sv104 & csrf(5)) && c.poly_angle >= 0.0f && !sv240) sv240 = true;
    if (sv240) sv104 |= csrf(1);
    if (sv104 & csrf(3)) target_collision_low_coverage(self, arg1, sv84, sv104, sv96, sv240);
    if (!(c.prev_status & cstat::on_surface)) {
        c.transv_on_last_impact = c.transv;
        c.ground_impact_vel = -vector_dot(c.transv, gn);
        c.normal_impact_vel = -vector_dot(c.transv, sv84);
        sv96 |= cstat::impact_surface;
        // impact-hook gets (control, &velocity, normal) in GOAL; the port's hooks take the target
        if (c.current_surface->impact_hook) c.current_surface->impact_hook(self);
        if (!sv240) {
            const float f0 = 1.0f - c.current_surface->impact_fric;
            if (f0 < 1.0f) {
                Vec v1{};
                float f1 = vector_dot(gn, sv88[0]);
                Vec t;
                vector_sub(v1, sv88[0], vector_float_mul(t, gn, f1));
                const float f2 = vector_length(v1);
                const float f3 = f2;
                if (f1 < 0.0f) f1 = f1 * f0;
                vector_add(sv88[0], vector_float_mul(sv88[0], gn, f1),
                           vector_float_mul(v1, v1, goal_div(f2, f3)));
            }
        }
    }
    sv96 |= cstat::touch_surface;
    if (arg1.num_spheres() != kGoalFalse) {
        // an actor's prim (touch-actor, actor-contact-*): never, the cache holds no actors
        sv96 |= cstat::touch_actor;
        c.actor_contact_pt = arg1.bot_intersect;
        c.actor_contact_normal = c.poly_normal;
    } else if (c.poly_pat.material() == 3) { // waterbottom
    } else {
        sv96 |= cstat::touch_background;
    }
    if (sv240) {
        sv104 |= csrf(2);
        sv96 |= cstat::touch_wall;
        c.cur_pat.bits = (c.cur_pat.bits & ~(7u << 7)) | (1u << 7);
        c.wall_contact_pt = arg1.bot_intersect;
        c.wall_contact_poly_normal = c.poly_normal;
        c.wall_contact_normal = sv84;
        c.wall_contact_pat = arg1.bot_pat;
        if ((self.focus_status & fstat::mech) && (sv104 & csrf(15)) && (sv104 & csrf(5)) &&
            0.0f < vector_dot(sv84, gn) && 0.0f < vector_dot(sv88[0], gn)) {
            c.surf = &edge_surface;
            sv104 |= csrf(16);
            Vec n;
            vector_flatten(arg2, vector_negate(n, gn), sv84);
            vector_normalize(arg2, vector_dot(arg2, sv88[0]));
        } else {
            vector_reflect_flat(arg2, sv88[0], sv84);
        }
        Vec d1, d2;
        if ((((me.local_sphere.w >=
               vector_dot(c.ground_poly_normal, vector_sub(d1, arg1.bot_intersect, c.grount_touch_point))) &&
              !time_elapsed(self, c.list_time_on_ground, seconds(0.3))) ||
             (sv104 & csrf(15))) &&
            vector_dot(gn, vector_sub(d2, arg1.bot_intersect, c.grount_touch_point)) >= 0.0f &&
            (0.0f < vector_dot(c.ground_poly_normal, arg2) && !(sv104 & csrf(5)))) {
            sv104 |= csrf(8);
            sv104 &= ~csrf(6);
            Vec s3;
            vector_cross(s3, c.poly_normal, c.ground_poly_normal);
            vector_normalize(s3, 1.0f);
            vector_float_mul(arg2, s3, vector_dot(sv88[0], s3));
            vector_add(arg2, arg2, c.poly_normal);
        }
    } else {
        sv96 |= cstat::on_surface;
        c.cur_pat.bits &= ~(7u << 7);
        if (me.prim_id == 6) c.local_normal = sv84;
        if ((self.focus_status & fstat::board) && !(self.focus_status & fstat::halfpipe) &&
            c.cur_pat.mode() != 3 && c.ground_pat.mode() != 3) {
            vector_reflect_flat_gravity(arg2, sv88[0], sv84, gn);
        } else {
            vector_reflect_flat(arg2, sv88[0], sv84);
        }
        vector_add(arg2, arg2, sv84);
        const float f0 = c.current_surface->slope_change_preserve;
        if (0.0f < f0) {
            Vec v1{};
            const float f1a = vector_dot(c.pre_collide_local_normal, sv88[0]);
            Vec t;
            vector_sub(v1, sv88[0], vector_float_mul(t, c.pre_collide_local_normal, f1a));
            const float f1 = vector_length(v1);
            const float f2 = vector_length(arg2);
            if (409.6f < std::fabs(f2 - f1) && f2 < f1) {
                vector_normalize(arg2, lerp_scale(f2, f1, f0, 0.0f, 1.0f));
            }
        }
        c.grount_touch_point.w = 0.0f;
        if (!(sv104 & (csrf(0) | csrf(1) | csrf(2) | csrf(3) | csrf(14)))) {
            sv96 |= cstat::on_ground;
            c.ground_poly_normal = c.poly_normal;
            c.ground_contact_normal = sv84;
            c.ground_local_norm_dot_grav = vector_dot(sv84, gn);
            c.list_time_on_ground = current_time(self);
            c.ground_pat = c.poly_pat;
            c.grount_touch_point = arg1.bot_intersect;
            c.ground_contact_sphere_center = me.prim_core.world_sphere;
            sv104 |= csrf(11);
            if (c.poly_pat.material() == 3) sv96 |= cstat::on_water;
        }
    }
    c.status |= sv96;
    c.reaction_flag = sv104;
    return sv96;
}

// target-collision-no-reaction
void target_collision_no_reaction(Target& self, GoalCollideQuery&, Vec&, const Vec&) {
    if (self.control.mod_surface->mode == Symbol("air")) self.control.reaction_flag |= csrf(5);
}

} // namespace jak
