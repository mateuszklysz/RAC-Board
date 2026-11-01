// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Stand-ins for what the movement core calls in the collision and board parts
// (jak/collide_from_core.hpp, board/board_from_core.hpp), so the core's tests run alone: a target
// shape of plain records, a flat floor at y = 0 for the move and the ground probe. They are weak, so
// the real ones win when the library has them.
#pragma once

#include "board/board_from_core.hpp"
#include "jak/collide_from_core.hpp"
#include "jak/math.hpp"
#include "jak/target.hpp"

#define CORE_FAKE __attribute__((weak))

namespace fake {
struct Prim {
    uint32_t action = 0, as = 0, with = 0;
    jak::Vec local{}, world{};
    int8_t ti = 0;
};
inline Prim prims[11]; // 0-9 the collision spheres, 10 the root
inline jak::Symbol next_state{};
inline int ground_finds = 0;
inline jak::CollideShapePrim* cp(Prim* p) { return reinterpret_cast<jak::CollideShapePrim*>(p); }
inline Prim* pp(jak::CollideShapePrim* p) { return reinterpret_cast<Prim*>(p); }
} // namespace fake

namespace jak {
CORE_FAKE CollideShapePrim* root_prim(Target&) { return fake::cp(&fake::prims[10]); }
CORE_FAKE CollideShapePrim* collision_sphere(Target&, int i) { return fake::cp(&fake::prims[i]); }
CORE_FAKE uint32_t& prim_action(CollideShapePrim* p) { return fake::pp(p)->action; }
CORE_FAKE uint32_t& prim_collide_as(CollideShapePrim* p) { return fake::pp(p)->as; }
CORE_FAKE uint32_t& prim_collide_with(CollideShapePrim* p) { return fake::pp(p)->with; }
CORE_FAKE Vec& prim_local_sphere(CollideShapePrim* p) { return fake::pp(p)->local; }
CORE_FAKE const Vec& prim_world_sphere(CollideShapePrim* p) { return fake::pp(p)->world; }
CORE_FAKE int8_t& prim_transform_index(CollideShapePrim* p) { return fake::pp(p)->ti; }
CORE_FAKE void modify_collide_as(Target&, int, uint32_t, uint32_t) {}
CORE_FAKE void move_to_point(Target& self, const Vec& p) { self.control.trans = p; }
// The floor y = 0 under everything.
CORE_FAKE bool find_ground(Target& self, PatSurface& pat, uint32_t, float, float, float) {
    fake::ground_finds++;
    self.control.gspot_pos = self.control.trans;
    self.control.gspot_pos.y = 0.0f;
    self.control.gspot_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
    pat = PatSurface{};
    return true;
}
CORE_FAKE bool fill_and_probe_using_spheres(Target&, const Vec*, uint32_t) { return false; }
// The move on a flat floor at y = 0: integrate, stop on the floor and stand on it.
CORE_FAKE void target_integrate_and_collide(Target& self, const Vec& transv, float) {
    ControlInfo& c = self.control;
    c.status = 0;
    vector_add_float_mul(c.trans, c.trans, transv, seconds_per_frame(self));
    if (c.trans.y <= 0.0f) {
        c.trans.y = 0.0f;
        if (c.transv.y < 0.0f) {
            c.transv.y = 0.0f;
        }
        c.status = cstat::on_surface | cstat::on_ground | cstat::touch_surface;
        c.local_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
        c.ground_poly_normal = c.local_normal;
        c.surface_angle = 1.0f;
        c.coverage = 1.0f;
    }
}
CORE_FAKE bool enabled_gun_p(Target&) { return false; }
CORE_FAKE void do_edge_grabs_target(Target&) {}
CORE_FAKE void find_overlapping_shapes_target(Target&) {}
CORE_FAKE void rail_surface_touch(Target&) {}
CORE_FAKE void target_board_joint_points(Target&) {}
CORE_FAKE Symbol next_state_name(const Target&) { return fake::next_state; }
CORE_FAKE void ja_post(Target&) {}
CORE_FAKE void joint_points(Target&) {}
CORE_FAKE int ja_group_collide_offset(Target&, Vec&) { return 0; }
} // namespace jak
