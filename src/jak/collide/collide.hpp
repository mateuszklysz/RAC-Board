// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's collision for the target, as the JET-Board uses it (OpenGOAL, jak-project, ISC License,
// Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md):
//   cache.cpp      collide-cache.gc: filling the cache from jak::World, the probes
//   shape.cpp      collide-shape.gc: the target's prims, integrate-and-collide!, react-to-pat!,
//                  find-ground; target-util.gc target-collide-set!
//   reaction.cpp   collide-reaction-target.gc: the target's collision reaction
//   edge_grab.cpp  collide-edge-grab.gc: edge grabs (the board's rails and halfpipe lips)
// The triangles come from jak::World (host.hpp); from the moment the cache holds them everything is
// Jak II's code. Collision against other actors (moby prims, the touching list, riders) is not
// ported: the port has no actors, so their fields stay but nothing fills them.
#pragma once

#include "jak/basic_types.hpp"
#include "jak/collide/from_core.hpp"
#include "jak/collide/mem.hpp"

namespace jak {

struct Target;
class World;

// ---- events ----

// What the collision code sends the target with send-event (react-to-pat!, target-attack-up,
// the reaction's 'slide, do-edge-grabs' 'edge-grab, rail-surface-touch's 'push-transv). The target
// gets it through Target::collide_event; the return value is send-event's (the handler's answer,
// #f as false). Message and mode are GOAL symbol names ("attack", "deadly").
struct CollideEvent {
    const char* message = nullptr; // 'attack, 'attack-or-shove, 'attack-invinc, 'slide, 'lip,
                                   // 'edge-grab, 'push-transv
    const char* param = nullptr;   // 'lip's argument: "lip" or "lipramp"
    // the attack-info (static-attack-info) of the attack messages; mask says which are set
    bool has_attack = false;
    uint32_t id = 0;
    const char* mode = nullptr;    // 'deadly, 'burn, 'deadlyup, 'shockup, 'burnup, 'melt, ...
    bool test = false;             // (test #t): only asks whether the target would take it
    bool has_vector = false;
    Vec vector{};
    bool has_shove_up = false;
    float shove_up = 0.0f;
    const char* angle = nullptr;   // 'up
    bool has_control = false;
    float control = 0.0f;
    // 'push-transv: the velocity and the time
    Vec push{};
    TimeFrame time = 0;
};
bool send_event(Target& self, const CollideEvent& ev);

// ---- The query, as the rest of the port fills and reads it ----

// collide-query (collide-h.gc), the fields callers set and the result they read. The collision
// code copies it into a GOAL collide-query on the arena's stack (collide/mem.hpp) and back.
struct CollideQuery {
    Vec start_pos{};
    Vec move_dist{};
    float radius = 0.0f;
    uint32_t collide_with = 0;
    bool ignore_self = true;  // ignore-process0 self, ignore-process1 #f
    PatSurface ignore_pat{};
    uint32_t action_mask = caction::solid;
    // fill-and-probe-using-spheres: the spheres (xyz centre, w radius), at most 64.
    const Vec* spheres = nullptr;
    int num_spheres = 0;
    BoundingBox bbox{};       // fill-using-bounding-box's box; set by the fills that build one
    // The result: the best triangle hit (best-other-tri).
    Vec best_vertex[3]{};
    Vec best_intersect{};
    Vec best_normal{};
    PatSurface best_pat{};
};

// ---- collide-cache.gc, on the target's world ----
void collide_cache_reset();                                            // (reset *collide-cache*)
void fill_using_bounding_box(Target& self, CollideQuery& q);           // q.bbox
float probe_using_line_sphere(Target& self, CollideQuery& q);          // on the cache as it is
float fill_and_probe_using_line_sphere(Target& self, CollideQuery& q); // fraction, < 0 on a miss
bool fill_and_probe_using_spheres(Target& self, CollideQuery& q);      // #t when a sphere touches
// fill-cache-integrate-and-collide of the control (target-real-post's move): q gives collide-with,
// ignore-pat and action-mask; transv is updated in place. Moves self.control.trans, sets status,
// the contacts and the surface.
void fill_cache_integrate_and_collide(Target& self, Vec& transv, const CollideQuery& q, float pad);
bool find_ground(Target& self, CollideQuery& q, uint32_t collide_with, float a2, float a3, float a4);

// ---- *pat-mode-info* (pat-h.gc), the wall angle per pat-mode ----
float pat_mode_wall_angle(uint32_t mode);

// ---- The target's shape (collide-shape.gc, target-util.gc, logic-target.gc) ----
// The collision part of init-target (logic-target.gc): the group prim and its nine spheres, the
// reaction and no-reaction, max-iteration-count 8. Done on first use of the target's shape. There
// is one target, as in Jak II (*target*): its prims live at a fixed place in the arena.
void target_collide_init(Target& self);
// target-collide-set! is the movement core's (movement.cpp).
void update_transforms(Target& self);                     // update-transforms
void move_by_vector(Target& self, const Vec& v);          // move-by-vector!
uint32_t& root_prim_action(Target& self);                 // (-> control root-prim prim-core action)
uint32_t root_prim_collide_with(const Target& self);
Vec collision_sphere(const Target& self, int i);          // its world-sphere
uint32_t collision_sphere_collide_with(const Target& self, int i);
uint32_t react_to_pat(Target& self, PatSurface pat);      // react-to-pat!: cshape-reaction-flags
bool find_ground_point(Target& self, Vec& out, float a2, float a3); // false for #f
void target_attack_up(Target& self, const char* message, const char* mode);

// ---- collide-edge-grab.gc ----
// collide-edge-spec-flags (collide-edge-grab-h.gc).
namespace edgeflag {
constexpr uint64_t find_adjacent_edge = 1u << 0, one = 1u << 1, two = 1u << 2, send_event = 1u << 3;
}
// collide-edge-spec (collide-edge-grab-h.gc). touching_segment is #f (null) or two points.
struct CollideEdgeSpec {
    float split_dists[2];
    Vec outward_offset;
    uint64_t flags;
    PatSurface ignore_pat;
    float max_dist_sqrd_to_outward_pt;
    float max_dir_cosa_delta;
    float max_dir_cosa_player;
    const Vec* touching_segment;
    BoundingBox local_cache_fill_box;
    BoundingBox local_within_reach_box;
    Vec local_player_spheres[12];
};
CollideEdgeSpec& collide_edge_spec();                // *collide-edge-spec*
CollideEdgeSpec& collide_edge_board_spec();          // *collide-edge-board-spec*
CollideEdgeSpec& collide_edge_board_halfpipe_spec(); // *collide-edge-board-halfpipe-spec*
uint64_t& edge_spec_flags(CollideEdgeSpec& spec);
void edge_spec_set_touching_segment(CollideEdgeSpec& spec, const Vec* segment);

// *edge-grab-info* (collide-edge-grab-h.gc), copied out of the arena after each do-edge-grabs and
// edge-grab-info-method-9.
struct EdgeGrabInfo {
    Vec world_vertex[8]{};  // edge 0-1, center-hold 2, tri-vertex 3-5, adjacent left 6, right 7
    Vec hanging_matrix[4]{};
    Vec hanging_matrix_row0{}; // (-> info hanging-matrix quad 0)
    Vec left_hand_hold{};
    Vec right_hand_hold{};
    Vec center_hold_old{};
    Vec adjacent_edge_left_vertex{};
    Vec adjacent_edge_right_vertex{};
    uint32_t edge_tri_pat = 0;
    uint32_t status = 0;    // bit 0 the left adjacent edge found, bit 1 the right one
    bool found_edge = false; // found-edge?
};
EdgeGrabInfo& edge_grab_info();
// (do-edge-grabs *target* *collide-cache* spec): looks for an edge to grab or ride; sends the
// target 'edge-grab when the spec says so and one is found.
void do_edge_grabs(Target& self, CollideEdgeSpec& spec);
// edge-grab-info-method-9 (mips2c): probes the hanging spheres where they were left; false when
// they now hit something. Works on *target*, the target do-edge-grabs last ran for.
bool edge_grab_info_method_9(EdgeGrabInfo& info);
// rail-surface-touch (collide-reaction-target.gc), *rail-surface*'s touch-hook.
void rail_surface_touch(Target& self);

} // namespace jak
