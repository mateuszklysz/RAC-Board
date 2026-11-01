// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The collide cache (collide-cache.gc), ported from OpenGOAL (jak-project, ISC License, Copyright
// (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). The background triangles come from jak::World in
// place of the level's collide fragments (collide-hash.gc and its mips2c fill-bg-using-box);
// from there on it is Jak II's code. Water (fill-from-water) and the other actors' prims
// (fill-from-fg-boxes, fill-from-fg-line-sphere) are not ported: the port has neither.
#include <cmath>

#include "jak/collide_internal.hpp"
#include "jak/collide_mips2c.hpp"
#include "jak/host.hpp"
#include "jak/math_collide.hpp"
#include "jak/target.hpp"

namespace jak {

namespace {

// What jak::World is asked for at most per fill; the cache keeps 460 of them (below).
constexpr int kWorldTrisMax = 2048;
CollideTri g_world_tris[kWorldTrisMax];

uint64_t bits_of(float f) {
    int32_t i;
    std::memcpy(&i, &f, 4);
    return static_cast<uint64_t>(static_cast<int64_t>(i));
}
float float_of(uint64_t r) {
    uint32_t u = static_cast<uint32_t>(r);
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}

// fill-from-bg with collide-list-fill-bg-using-box (collide-hash.gc; game/mips2c/jak2_functions/
// collide_hash.cpp fill_bg_using_box_new), on the world's triangles. Per triangle, as the mips2c
// loop: skipped when its pat has a bit of the query's ignore-pat, or when its integer box misses
// the cache's collide-box4w (the original tests in the fragment's quantised local space; the port
// tests the truncated world coordinates); stored while the cache holds fewer than 460 (the bgez on
// num-tris - 460), with collide-ptr #f (no instance), prim-index 0 and the vertices' w 1.0.
void fill_from_bg(World* world, CollideCache& cc, GoalCollideQuery& q) {
    if (!world) return;
    const Vec lo{q.bbox.min.x, q.bbox.min.y, q.bbox.min.z, 1.0f};
    const Vec hi{q.bbox.max.x, q.bbox.max.y, q.bbox.max.z, 1.0f};
    const int n = world->triangles(lo, hi, g_world_tris, kWorldTrisMax);
    if (n <= 0) return; // (when (> (-> *collide-list* num-items) 0) ...)
    for (int i = 0; i < n; i++) {
        const CollideTri& t = g_world_tris[i];
        if (q.ignore_pat.bits & t.pat.bits) continue;
        int32_t tmin[3], tmax[3];
        for (int k = 0; k < 3; k++) {
            const float a = (&t.vertex[0].x)[k], b = (&t.vertex[1].x)[k], c = (&t.vertex[2].x)[k];
            const int32_t ia = mips::f2i_x86(a), ib = mips::f2i_x86(b), ic = mips::f2i_x86(c);
            tmin[k] = std::min(std::min(ia, ib), ic);
            tmax[k] = std::max(std::max(ia, ib), ic);
        }
        bool miss = false;
        for (int k = 0; k < 3; k++) {
            if (tmin[k] > cc.collide_box4w.max[k] || cc.collide_box4w.min[k] > tmax[k]) miss = true;
        }
        if (miss) continue;
        if (cc.num_tris - 460 >= 0) break; // too many tris: the rest is dropped
        CollideCacheTri& ct = cc.tris[cc.num_tris];
        cc.num_tris++;
        for (int k = 0; k < 3; k++) {
            ct.vertex[k] = Vec{t.vertex[k].x, t.vertex[k].y, t.vertex[k].z, 1.0f};
        }
        ct.pat = t.pat;
        ct.collide_ptr = kGoalFalse;
        ct.prim_index = 0;
        ct.user16 = 0;
        ct.user32 = 0;
    }
    const int a0 = cc.num_tris;
    if (a0 > 0) {
        CollideCachePrim& p = cc.prims[0];
        CollideShapePrim& bg = prim_backgnd();
        p.num_tris = static_cast<uint16_t>(a0);
        p.prim = goal_addr_basic(&bg);
        cc.num_prims = 1;
        p.first_tri = 0;
        p.ccache = collide_cache_addr();
        p.prim_core = bg.prim_core;
    }
}

} // namespace

GoalCollideQuery& new_stack_query(GoalStackFrame& f) { return f.alloc_no_clear<GoalCollideQuery>(); }

// (method 10 collide-cache) reset.
void collide_cache_reset() {
    collide_arena_init();
    CollideCache& cc = collide_cache();
    cc.num_tris = 0;
    cc.num_prims = 0;
    cc.ignore_processes[0] = kGoalFalse;
    cc.ignore_processes[1] = kGoalFalse;
}

// (method 21 collide-cache) fill-using-bounding-box.
void fill_using_bounding_box(World* world, GoalCollideQuery& q) {
    collide_arena_init();
    CollideCache& cc = collide_cache();
    Vec v1 = q.bbox.min;
    Vec v2 = q.bbox.max;
    cc.ignore_processes[0] = q.ignore_process0;
    cc.ignore_processes[1] = q.ignore_process1;
    v1.w = 1.0f; // .mov.vf.w from vf0
    v2.w = 1.0f;
    cc.ignore_mask = q.ignore_pat;
    BoundingBox4w b4;
    for (int i = 0; i < 4; i++) {
        b4.min[i] = mips::f2i_x86((&v1.x)[i]);
        b4.max[i] = mips::f2i_x86((&v2.x)[i]);
    }
    cc.num_tris = 0;
    cc.collide_box.min = v1;
    cc.collide_box.max = v2;
    cc.collide_box4w = b4;
    q.bbox4w = b4;
    cc.num_prims = 0;
    cc.collide_with = q.collide_with;
    if (q.collide_with & cspec::backgnd) fill_from_bg(world, cc, q);
    // water (fill-from-water) and hit-by-player/others/player-list (fill-from-fg-boxes): no water
    // and no actors in the port.
}

// (method 12 collide-cache) fill-using-line-sphere. When fewer than two axes of move-dist are
// short (< 1 m) Jak II fills along the line through the collide-hash's oriented search; the port's
// world has no such search, so it fills the line's padded box (what the short case does, and a
// superset of the line's triangles).
void fill_using_line_sphere(World* world, GoalCollideQuery& q) {
    set_from_point_offset_pad(q.bbox, q.start_pos, q.move_dist, q.radius());
    fill_using_bounding_box(world, q);
}

// (method 13 collide-cache) probe-using-line-sphere.
float probe_using_line_sphere(GoalCollideQuery& q) {
    collide_arena_init();
    CollideCache& cc = collide_cache();
    GoalStackFrame f;
    CollideCacheTri& s5 = f.alloc_no_clear<CollideCacheTri>();
    std::memcpy(&s5.vertex[0].x, &q.ignore_pat.bits, 4);
    Vec v3 = q.start_pos;
    v3.w = 1.0f * q.radius(); // (.mul.w.vf.w vf3 vf0 vf4)
    s5.vertex[2] = q.move_dist;
    s5.vertex[1] = v3;
    float f30 = kCollisionMiss;
    for (int i = 0; i < cc.num_prims; i++) {
        CollideCachePrim& p = cc.prims[i];
        if ((q.collide_with & p.prim_core.collide_as) && (q.action_mask & p.prim_core.action)) {
            if (p.prim_core.prim_type >= 0) {
                const uint64_t args[6] = {goal_addr(&p), goal_addr(&q), goal_addr(&s5.vertex[1]),
                                          goal_addr(&s5.vertex[2]), bits_of(f30), caction::solid};
                const float f0 = float_of(goal_call(m2c_resolve_moving_sphere_tri, args, 6));
                if (f0 >= 0.0f) f30 = f0;
            }
            // else: resolve-moving-sphere-sphere (method 10 collide-cache-prim) against an
            // actor's sphere prim; the cache never holds one in the port.
        }
    }
    return f30;
}

// (method 11 collide-cache) fill-and-probe-using-line-sphere.
float fill_and_probe_using_line_sphere(World* world, GoalCollideQuery& q) {
    fill_using_line_sphere(world, q);
    return probe_using_line_sphere(q);
}

// (method 16 collide-cache) fill-using-spheres.
void fill_using_spheres(World* world, GoalCollideQuery& q) {
    set_from_spheres(q.bbox, &goal_ref<Vec>(q.spheres()), static_cast<int>(q.num_spheres()));
    fill_using_bounding_box(world, q);
}

// (method 17 collide-cache) probe-using-spheres, mips2c.
bool probe_using_spheres(GoalCollideQuery& q) {
    const uint64_t args[2] = {collide_cache_addr(), goal_addr(&q)};
    return goal_call(m2c_probe_using_spheres, args, 2) != kGoalFalse;
}

// (method 15 collide-cache) fill-and-probe-using-spheres.
bool fill_and_probe_using_spheres(World* world, GoalCollideQuery& q) {
    fill_using_spheres(world, q);
    return probe_using_spheres(q);
}

// ---- The host-side query ----

namespace {

// A stand-in for the target's process pointer in ignore-process0: only compared with #f.
constexpr uint32_t kGoalSelfProcess = 0x200;

void query_in(GoalCollideQuery& g, const CollideQuery& q) {
    g.start_pos = q.start_pos;
    g.move_dist = q.move_dist;
    g.radius() = q.radius;
    g.collide_with = q.collide_with;
    g.ignore_process0 = q.ignore_self ? kGoalSelfProcess : kGoalFalse;
    g.ignore_process1 = kGoalFalse;
    g.ignore_pat = q.ignore_pat;
    g.action_mask = q.action_mask;
    g.bbox = q.bbox;
}
void query_out(CollideQuery& q, const GoalCollideQuery& g) {
    for (int i = 0; i < 3; i++) q.best_vertex[i] = g.bot_vertex[i];
    q.best_intersect = g.bot_intersect;
    q.best_normal = g.bot_normal;
    q.best_pat = g.bot_pat;
    q.bbox = g.bbox;
}

} // namespace

void fill_using_bounding_box(Target& self, CollideQuery& q) {
    collide_arena_init();
    g_collide_target = &self;
    GoalStackFrame f;
    GoalCollideQuery& g = new_stack_query(f);
    query_in(g, q);
    fill_using_bounding_box(self.world, g);
    query_out(q, g);
}

float probe_using_line_sphere(Target& self, CollideQuery& q) {
    collide_arena_init();
    g_collide_target = &self;
    GoalStackFrame f;
    GoalCollideQuery& g = new_stack_query(f);
    query_in(g, q);
    const float r = probe_using_line_sphere(g);
    query_out(q, g);
    return r;
}

float fill_and_probe_using_line_sphere(Target& self, CollideQuery& q) {
    collide_arena_init();
    g_collide_target = &self;
    GoalStackFrame f;
    GoalCollideQuery& g = new_stack_query(f);
    query_in(g, q);
    const float r = fill_and_probe_using_line_sphere(self.world, g);
    query_out(q, g);
    return r;
}

bool fill_and_probe_using_spheres(Target& self, CollideQuery& q) {
    collide_arena_init();
    g_collide_target = &self;
    GoalStackFrame f;
    const int n = q.num_spheres < 0 ? 0 : (q.num_spheres > 64 ? 64 : q.num_spheres);
    Vec* spheres = &goal_ref<Vec>(0);
    for (int i = n - 1; i >= 0; i--) {
        Vec& s = f.alloc_no_clear<Vec>();
        s = q.spheres[i];
        spheres = &s;
    }
    GoalCollideQuery& g = new_stack_query(f);
    query_in(g, q);
    g.spheres() = goal_addr(spheres);
    g.num_spheres() = static_cast<uint32_t>(n);
    g.best_my_prim() = kGoalTrue; // solid-only, as every caller in Jak II's board path sets it
    const bool r = fill_and_probe_using_spheres(self.world, g);
    query_out(q, g);
    return r;
}

// can-exit-duck?'s probe (target-util.gc), as the movement core asks for it.
bool fill_and_probe_using_spheres(Target& self, const Vec spheres[2], uint32_t collide_with) {
    CollideQuery q;
    q.spheres = spheres;
    q.num_spheres = 2;
    q.collide_with = collide_with;
    q.ignore_self = false;
    q.ignore_pat.bits = 0x1 | 0x10 | (1u << 24) | (1u << 27); // noentity nojak probe noendlessfall
    q.action_mask = caction::solid;
    return fill_and_probe_using_spheres(self, q);
}

// *pat-mode-info* (pat-h.gc): the wall-angle of ground, wall, obstacle and halfpipe.
float pat_mode_wall_angle(uint32_t mode) {
    static const float kWallAngle[4] = {0.2f, 2.0f, 0.82f, -2.0f};
    // Modes 4..7 index past the 4-entry table in GOAL; no triangle the port makes has one.
    return mode < 4 ? kWallAngle[mode] : 0.0f;
}

} // namespace jak
