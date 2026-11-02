// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The collision code's own routines on GOAL structures in the arena (collide/mem.hpp), shared by
// the files in src/jak/collide/ (OpenGOAL, jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL
// Team; THIRD_PARTY.md).
#pragma once

#include "jak/collide/collide.hpp"
#include "jak/collide/mem.hpp"

namespace jak {

class World;

// The target do-edge-grabs and the cache fills last ran for (*target*), and its world.
extern Target* g_collide_target;

// cshape-reaction-flags (collide-shape-h.gc): csrf00 .. csrf16 are bits 0 .. 16.
constexpr uint32_t csrf(int n) { return 1u << n; }

// focus-status bits (process-focusable.gc) the collision reads.
namespace fstat {
constexpr uint64_t edge_grab = 1ull << 12, board = 1ull << 18, pilot = 1ull << 20,
                   mech = 1ull << 21, dark = 1ull << 22, rail = 1ull << 23, halfpipe = 1ull << 24;
}

// ---- collide-cache.gc ----
void fill_using_bounding_box(World* world, GoalCollideQuery& q);
void fill_using_line_sphere(World* world, GoalCollideQuery& q);
float probe_using_line_sphere(GoalCollideQuery& q);
float fill_and_probe_using_line_sphere(World* world, GoalCollideQuery& q);
void fill_using_spheres(World* world, GoalCollideQuery& q);
bool probe_using_spheres(GoalCollideQuery& q);
bool fill_and_probe_using_spheres(World* world, GoalCollideQuery& q);
// A fresh collide-query on the GOAL stack, as (new 'stack-no-clear 'collide-query) leaves it: the
// arena stack is not cleared between uses, so whatever an earlier query left is still there.
GoalCollideQuery& new_stack_query(GoalStackFrame& f);

// ---- collide-shape.gc ----
CollideShapePrim& target_prim(int child);       // 0 the root group, 1.. the spheres in order
CollideShapePrim* target_collision_sphere(int i); // (-> control collision-spheres i), null for 5
void sync_cshape(Target& self);                  // control trans into the arena's image
void fill_cache_for_shape(Target& self, float pad, GoalCollideQuery& q);
bool build_bounding_box_for_shape(Target& self, BoundingBox& box, float pad, uint32_t collide_with);
void fill_cache_integrate_and_collide(Target& self, Vec& transv, GoalCollideQuery& q, float pad);
void integrate_and_collide_control(Target& self, Vec& transv);
void integrate_and_collide_moving(Target& self, Vec& transv);
float step_collison(Target& self, Vec& out_vel, Vec& in_vel, float frac, int iter);
void collide_shape_moving_angle_set(Target& self, const Vec& normal, const Vec& vel);
bool find_ground(Target& self, GoalCollideQuery& q, uint32_t collide_with, float a2, float a3,
                 float a4);

// ---- collide-reaction-target.gc ----
CollideStatus target_collision_reaction(Target& self, GoalCollideQuery& q, Vec& out_vel,
                                        const Vec& in_vel);
void target_collision_no_reaction(Target& self, GoalCollideQuery& q, Vec& out_vel,
                                  const Vec& in_vel);
void target_collision_low_coverage(Target& self, GoalCollideQuery& q, const Vec& contact_normal,
                                   uint32_t& flags, CollideStatus& status, bool& wall);

// ---- collide-edge-grab.gc ----
void copy_edge_grab_info_out();

} // namespace jak
