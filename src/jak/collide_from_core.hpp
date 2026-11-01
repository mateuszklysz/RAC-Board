// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// What the target's movement core (src/jak/movement*, src/jak/surface*) calls in the collision code,
// declared here and implemented by the collision part (src/jak/collide*). Each line names the GOAL
// routine or field it stands for (OpenGOAL, jak-project, ISC License, Copyright (c) 2020-2026
// OpenGOAL Team; THIRD_PARTY.md).
#pragma once

#include "jak/basic_types.hpp"

namespace jak {

struct Target;

// ---- the target's collision shape ----
// owned by collide: (-> self control root-prim), the group prim of the target's shape.
CollideShapePrim* root_prim(Target& self);
// owned by collide: (-> self control collision-spheres i), i 0..9 (target's prims: 0-2 body, 3 gun,
// 4 the jak-vulnerable sphere, 6-9 attack spheres; logic-target.gc init-target).
CollideShapePrim* collision_sphere(Target& self, int i);
// owned by collide: a prim's prim-core action (collide-action bits), collide-as and collide-with
// (collide-spec bits), local-sphere (xyz offset, w radius), world-sphere and transform-index.
uint32_t& prim_action(CollideShapePrim* prim);
uint32_t& prim_collide_as(CollideShapePrim* prim);
uint32_t& prim_collide_with(CollideShapePrim* prim);
Vec& prim_local_sphere(CollideShapePrim* prim);
const Vec& prim_world_sphere(CollideShapePrim* prim);
int8_t& prim_transform_index(CollideShapePrim* prim);
// owned by collide: (modify-collide-as! (-> self control) arg0 arg1 arg2), collide-shape method.
void modify_collide_as(Target& self, int arg0, uint32_t clear_as, uint32_t set_as);
// owned by collide: (move-to-point! (-> self control) p), collide-shape method.
void move_to_point(Target& self, const Vec& p);

// ---- queries ----
// owned by collide: (find-ground (-> self control) cquery collide_with a2 a3 a4), collide-shape-moving
// method (collide-shape.gc): sets control gspot-pos and gspot-normal; returns #t on a hit and then
// the hit triangle's pat in best_pat.
bool find_ground(Target& self, PatSurface& best_pat, uint32_t collide_with, float above,
                 float below, float radius);
// owned by collide: can-exit-duck?'s probe (target-util.gc): fill-and-probe-using-spheres with the
// two spheres (xyz centre, w radius) against collide_with, ignore-pat noentity|nojak|probe|
// noendlessfall, action solid; returns #t when a sphere touches something.
bool fill_and_probe_using_spheres(Target& self, const Vec spheres[2], uint32_t collide_with);

// ---- the move ----
// owned by collide: target-real-post's move: control reaction set to target-collision-reaction,
// then (fill-cache-integrate-and-collide (-> self control) transv cquery (meters 1)) with cquery
// collide-with = root-prim's collide-with, ignore self, ignore-pat = control pat-ignore-mask,
// action-mask solid.
void target_integrate_and_collide(Target& self, const Vec& transv, float padding);
// owned by collide: (do-edge-grabs *target* *collide-cache* *collide-edge-spec*).
void do_edge_grabs_target(Target& self);
// owned by collide: (find-overlapping-shapes (-> self control) params) with options oo0, filter -1,
// tlist *touching-list*, as target-no-move-post calls it.
void find_overlapping_shapes_target(Target& self);
// owned by collide: rail-surface-touch (collide-reaction-target.gc), *rail-surface*'s touch-hook.
void rail_surface_touch(Target& self);

} // namespace jak
