// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The collision system's memory: Jak II's collision structures with their GOAL layouts
// (collide-cache-h.gc, collide-h.gc, collide-shape-h.gc, collide-edge-grab-h.gc; jak-project, ISC
// License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md), living in one arena so the
// assembly routines OpenGOAL kept as mips2c (collide/mips2c.cpp) read them at the offsets they were
// written for. A GOAL address is an offset into the arena (collide/vu.hpp, g_mem).
//
// Offsets are counted from the start of each structure. A `basic` (a structure with a type tag,
// such as collide-cache or collide-shape-prim) is addressed in GOAL four bytes past its start, so
// its GOAL address is its arena offset plus 4 (goal_addr_basic).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "jak/basic_types.hpp"
#include "jak/collide/vu.hpp"

namespace jak {

// ---- The arena ----

// GOAL's symbol table: #f is the address held in s7, #t the word after it.
constexpr uint32_t kGoalFalse = 0x100;
constexpr uint32_t kGoalTrue = kGoalFalse + 4;

// Where each global lives (fixed, as GOAL's globals are).
constexpr uint32_t kAddrPrimBackgnd = 0x1000;  // *collide-shape-prim-backgnd* (80 bytes)
constexpr uint32_t kAddrCollideCache = 0x2000; // *collide-cache* (34416 bytes)
constexpr uint32_t kAddrEdgeWork = 0xb000;     // *collide-edge-work* (9920 bytes)
constexpr uint32_t kAddrEdgeGrabInfo = 0xe000; // *edge-grab-info* (496 bytes)
constexpr uint32_t kAddrTargetPrims = 0xe400;  // the target's 10 collide-shape-prims (800 bytes)
constexpr uint32_t kAddrCshape = 0xe800;       // the target's control-info, the fields the
                                               // assembly reads (collide/mem.cpp, sync_cshape)
constexpr uint32_t kAddrTouchingSegment = 0xea00; // a copy of the spec's touching-segment
constexpr uint32_t kAddrScratchpad = 0xf000;   // *fake-scratchpad-data* (collide-puss-work)
constexpr uint32_t kAddrStackBottom = 0x10000;
constexpr uint32_t kAddrStackTop = 0x40000;    // the GOAL stack grows down from here
constexpr uint32_t kArenaSize = kAddrStackTop;

template <typename T>
T& goal_ref(uint32_t addr) {
    return *reinterpret_cast<T*>(mips::g_mem + addr);
}
inline uint32_t goal_addr(const void* p) {
    return static_cast<uint32_t>(static_cast<const uint8_t*>(p) - mips::g_mem);
}
inline uint32_t goal_addr_basic(const void* p) { return goal_addr(p) + 4; }
inline bool in_arena(const void* p) {
    const uint8_t* b = static_cast<const uint8_t*>(p);
    return b >= mips::g_mem && b < mips::g_mem + kArenaSize;
}

// The GOAL stack, for the `(new 'stack ...)` structures the assembly is handed: a frame takes
// space below the stack pointer and gives it back when it goes out of scope.
extern uint32_t g_goal_sp;
class GoalStackFrame {
  public:
    GoalStackFrame() : saved_(g_goal_sp) {}
    ~GoalStackFrame() { g_goal_sp = saved_; }
    GoalStackFrame(const GoalStackFrame&) = delete;
    GoalStackFrame& operator=(const GoalStackFrame&) = delete;
    // `(new 'stack-no-clear 'type)`: 16-byte aligned, contents left as they are.
    template <typename T>
    T& alloc_no_clear() {
        g_goal_sp = (g_goal_sp - static_cast<uint32_t>(sizeof(T))) & ~15u;
        return goal_ref<T>(g_goal_sp);
    }
    // `(new 'stack 'type)`: cleared.
    template <typename T>
    T& alloc() {
        T& r = alloc_no_clear<T>();
        std::memset(static_cast<void*>(&r), 0, sizeof(T));
        return r;
    }

  private:
    uint32_t saved_;
};

// Sets the arena up (the symbol table, *collide-shape-prim-backgnd*, the caches' type tags); done
// once, on first use.
void collide_arena_init();

// ---- Shapes (geometry/bounding-box-h.gc) ----

struct BoundingBox {
    Vec min;
    Vec max;
};
struct BoundingBox4w {
    int32_t min[4];
    int32_t max[4];
};

// ---- collide-prim-core, collide-shape-prim (collide-shape-h.gc) ----

// collide-spec bits (collide-shape-h.gc) the collision code names.
namespace cspec {
constexpr uint32_t backgnd = 1u << 0, jak = 1u << 1, bot = 1u << 2, crate = 1u << 3,
                   civilian = 1u << 4, enemy = 1u << 5, obstacle = 1u << 6,
                   vehicle_sphere = 1u << 7, hit_by_player_list = 1u << 8,
                   hit_by_others_list = 1u << 9, player_list = 1u << 10, water = 1u << 11,
                   collectable = 1u << 12, blocking_plane = 1u << 13, projectile = 1u << 14,
                   jak_vulnerable = 1u << 15, camera_blocker = 1u << 16, tobot = 1u << 18,
                   pusher = 1u << 19, vehicle_mesh = 1u << 20, obstacle_for_jak = 1u << 25;
}
// collide-action bits (collide-shape-h.gc).
namespace caction {
constexpr uint32_t solid = 1u << 0, semi_solid = 1u << 1, rideable = 1u << 2, can_ride = 1u << 3,
                   deadly = 1u << 6, check_edge = 1u << 11, check_stuck = 1u << 12,
                   stuck_wall_escape = 1u << 13, no_normal_reset = 1u << 14,
                   edge_grabbed = 1u << 15;
}
// prim-type (collide-shape-h.gc).
enum PrimType : int8_t { kPrimPrim = -2, kPrimSphere = -1, kPrimGroup = 0, kPrimMesh = 1, kPrimFake = 2 };

struct CollidePrimCore {
    Vec world_sphere;      // 0
    uint32_t collide_as;   // 16
    uint32_t collide_with; // 20
    uint32_t action;       // 24
    int8_t prim_type;      // 28
    uint8_t unused1[3];
};
static_assert(sizeof(CollidePrimCore) == 32, "collide-prim-core");

// collide-shape-prim and its children (sphere, mesh, group): 80 bytes, a basic.
struct CollideShapePrim {
    uint32_t type_tag;       // 0
    uint32_t cshape;         // 4: GOAL address of the collide-shape, or #f
    uint32_t prim_id;        // 8
    int8_t transform_index;  // 12
    int8_t unused2[3];
    CollidePrimCore prim_core; // 16
    Vec local_sphere;        // 48
    uint8_t specific[16];    // 64
    // collide-shape-prim-sphere
    PatSurface& pat() { return *reinterpret_cast<PatSurface*>(&specific[0]); }
    // collide-shape-prim-group
    uint8_t& num_children() { return specific[0]; }
    uint32_t& child() { return *reinterpret_cast<uint32_t*>(&specific[4]); }
};
static_assert(sizeof(CollideShapePrim) == 80, "collide-shape-prim");
static_assert(offsetof(CollideShapePrim, prim_core) == 16, "collide-shape-prim");

// ---- collide-cache (collide-cache-h.gc) ----

struct CollideCacheTri {
    Vec vertex[3];        // 0
    PatSurface pat;       // 48
    uint32_t collide_ptr; // 52
    uint16_t prim_index;  // 56
    uint16_t user16;      // 58
    uint32_t user32;      // 60
};
static_assert(sizeof(CollideCacheTri) == 64, "collide-cache-tri");

struct CollideCachePrim {
    CollidePrimCore prim_core; // 0
    uint32_t ccache;           // 32
    uint32_t prim;             // 36: GOAL address of the collide-shape-prim
    uint16_t first_tri;        // 40
    uint16_t num_tris;         // 42
    uint8_t unused[4];
};
static_assert(sizeof(CollideCachePrim) == 48, "collide-cache-prim");

constexpr int kCacheMaxPrims = 100;
constexpr int kCacheMaxTris = 461;

struct CollideCache {
    uint32_t type_tag;            // 0
    int32_t num_tris;             // 4
    int32_t num_prims;            // 8
    PatSurface ignore_mask;       // 12
    uint32_t ignore_processes[2]; // 16
    uint32_t pad0[2];
    BoundingBox collide_box;      // 32
    BoundingBox4w collide_box4w;  // 64
    uint32_t collide_with;        // 96
    uint32_t unused;              // 100
    uint32_t pad1[2];
    CollideCachePrim prims[kCacheMaxPrims]; // 112
    CollideCacheTri tris[kCacheMaxTris];    // 4912
};
static_assert(offsetof(CollideCache, prims) == 112, "collide-cache");
static_assert(offsetof(CollideCache, tris) == 4912, "collide-cache");

// ---- collide-query (collide-h.gc) ----

// collide-tri-result: the triangle a probe hit and where.
struct CollideTriResult {
    Vec vertex[3];        // 0
    Vec intersect;        // 48
    Vec normal;           // 64
    PatSurface pat;       // 80
    uint32_t collide_ptr; // 84
    uint32_t pad[2];      // (the structure is 88 bytes; the next quadword)
};

struct GoalCollideQuery {
    // best-other-tri (collide-tri-result), inline, 88 bytes
    Vec bot_vertex[3];       // 0
    Vec bot_intersect;       // 48
    Vec bot_normal;          // 64
    PatSurface bot_pat;      // 80
    uint32_t bot_collide_ptr; // 84
    uint32_t ignore_process0; // 88
    uint32_t ignore_process1; // 92
    PatSurface ignore_pat;   // 96
    uint32_t collide_with;   // 100
    uint32_t pad0[2];
    uint32_t overlay_params[3]; // 112: spheres / best-dist, num-spheres / best-other-prim,
                                //      solid-only / best-my-prim
    uint32_t pad1;
    BoundingBox bbox;        // 128
    BoundingBox4w bbox4w;    // 160
    Vec bsphere;             // 192
    Vec start_pos;           // 208
    Vec move_dist;           // 224
    Vec rlength;             // 240
    Vec exit_planes[2];      // 256; radius overlays exit-planes 0 w (268)
    Mat inv_mat;             // 288
    uint32_t action_mask;    // 352
    uint32_t pad2[3];
    BoundingBox4w local_box4w; // 368
    BoundingBox4w search_box;  // 400
    int32_t search_vector[4];  // 432
    Mat instance_mat;        // 448
    uint32_t instance_ptr;   // 512
    uint32_t x_addr, x_step, y_addr, y_step, z_addr, z_step; // 516
    uint32_t pad3;

    CollideTriResult& best_other_tri() { return *reinterpret_cast<CollideTriResult*>(this); }
    float& radius() { return exit_planes[0].w; }
    float& best_dist() { return *reinterpret_cast<float*>(&overlay_params[0]); }
    uint32_t& spheres() { return overlay_params[0]; }
    uint32_t& num_spheres() { return overlay_params[1]; }      // also best-other-prim
    uint32_t& best_other_prim() { return overlay_params[1]; }
    uint32_t& best_my_prim() { return overlay_params[2]; }     // also solid-only
};
static_assert(offsetof(GoalCollideQuery, ignore_process0) == 88, "collide-query");
static_assert(offsetof(GoalCollideQuery, overlay_params) == 112, "collide-query");
static_assert(offsetof(GoalCollideQuery, start_pos) == 208, "collide-query");
static_assert(offsetof(GoalCollideQuery, exit_planes) == 256, "collide-query");
static_assert(offsetof(GoalCollideQuery, inv_mat) == 288, "collide-query");
static_assert(offsetof(GoalCollideQuery, action_mask) == 352, "collide-query");
static_assert(offsetof(GoalCollideQuery, instance_ptr) == 512, "collide-query");
static_assert(sizeof(GoalCollideQuery) == 544, "collide-query");

// ---- collide-puss-work (collide-cache-h.gc), on the scratchpad ----
struct CollidePussSphere {
    Vec bsphere;
    BoundingBox4w bbox4w;
};
struct CollidePussWork {
    Vec closest_pt;              // 0
    Vec tri_normal;              // 16
    BoundingBox4w tri_bbox4w;    // 32
    BoundingBox4w spheres_bbox4w; // 64
    CollidePussSphere spheres[64]; // 96
};
static_assert(offsetof(CollidePussWork, spheres) == 96, "collide-puss-work");

// ---- edge grabs (collide-edge-grab-h.gc) ----

struct PilotEdgeGrabInfo {
    Vec local_pos;
    Vec local_dir;
    uint64_t handle;
    uint64_t pad;
};

struct GoalEdgeGrabInfo {
    Vec world_vertex[8];       // 0: edge-vertex 0-1, center-hold 2, tri-vertex 3-5,
                               //    adjacent-edge-left-vertex 6, adjacent-edge-right-vertex 7
    Vec local_vertex[8];       // 128
    uint64_t status;           // 256
    int32_t actor_cshape_prim_offset; // 264
    uint32_t pad0;
    uint64_t actor_handle;     // 272
    uint64_t pad1;
    Mat hanging_matrix;        // 288
    Vec left_hand_hold;        // 352
    Vec right_hand_hold;       // 368
    Vec center_hold_old;       // 384
    uint32_t edge_tri_pat;     // 400
    uint32_t found_edge_p;     // 404 (symbol)
    uint32_t pilot_edge_grab_p; // 408 (symbol)
    uint32_t pad2;
    PilotEdgeGrabInfo pilot_edge_grab; // 416
    Vec pilot_start_grab_pos;  // 464
    float pilot_grab_interp;   // 480
    uint32_t pad3[3];

    Vec& center_hold() { return world_vertex[2]; }
    Vec* tri_vertex() { return &world_vertex[3]; }
    Vec& adjacent_edge_left_vertex() { return world_vertex[6]; }
    Vec& adjacent_edge_right_vertex() { return world_vertex[7]; }
    bool found_edge() const { return found_edge_p != kGoalFalse; }
};
static_assert(offsetof(GoalEdgeGrabInfo, status) == 256, "edge-grab-info");
static_assert(offsetof(GoalEdgeGrabInfo, hanging_matrix) == 288, "edge-grab-info");
static_assert(offsetof(GoalEdgeGrabInfo, center_hold_old) == 384, "edge-grab-info");
static_assert(offsetof(GoalEdgeGrabInfo, pilot_edge_grab) == 416, "edge-grab-info");

struct CollideEdgeTri {
    uint32_t ctri;  // 0: GOAL address of the collide-cache-tri
    uint32_t pad[3];
    Vec normal;     // 16
};
static_assert(sizeof(CollideEdgeTri) == 32, "collide-edge-tri");

struct CollideEdgeEdge {
    uint32_t ignore;        // 0 (symbol)
    uint32_t etri;          // 4
    uint32_t vertex_ptr[2]; // 8
    Vec outward;            // 16
    Vec edge_vec_norm;      // 32
};
static_assert(sizeof(CollideEdgeEdge) == 48, "collide-edge-edge");

struct CollideEdgeHoldItem {
    uint32_t next;   // 0
    float rating;    // 4
    int8_t split;    // 8
    uint8_t pad0[3];
    uint32_t edge;   // 12
    Vec center_pt;   // 16
    Vec outward_pt;  // 32
};
static_assert(sizeof(CollideEdgeHoldItem) == 48, "collide-edge-hold-item");

struct CollideEdgeHoldList {
    uint32_t num_allocs;   // 0
    uint32_t num_attempts; // 4
    uint32_t head;         // 8
    uint32_t pad;
    CollideEdgeHoldItem items[32]; // 16
    Vec attempts[32];      // 1552
};
static_assert(offsetof(CollideEdgeHoldList, attempts) == 1552, "collide-edge-hold-list");

// collide-edge-spec as GOAL lays it out (320 bytes).
struct CollideEdgeSpecGoal {
    float split_dists[2];    // 0
    uint32_t pad0[2];
    Vec outward_offset;      // 16
    uint64_t flags;          // 32
    PatSurface ignore_pat;   // 40
    float max_dist_sqrd_to_outward_pt; // 44
    float max_dir_cosa_delta; // 48
    float max_dir_cosa_player; // 52
    uint32_t touching_segment; // 56 (symbol, or the address of two vectors)
    uint32_t pad1;
    BoundingBox local_cache_fill_box;   // 64
    BoundingBox local_within_reach_box; // 96
    Vec local_player_spheres[12];       // 128: hanging spheres 0-5, leap-up spheres 6-11
};
static_assert(sizeof(CollideEdgeSpecGoal) == 320, "collide-edge-spec");

struct CollideEdgeWork {
    uint32_t ccache;          // 0
    uint32_t cshape;          // 4
    uint32_t num_verts;       // 8
    uint32_t num_edges;       // 12
    uint32_t num_tris;        // 16
    uint32_t pad0[3];
    BoundingBox cache_fill_box;     // 32
    BoundingBox within_reach_box;   // 64
    BoundingBox4w within_reach_box4w; // 96
    Vec search_pt;            // 128
    Vec search_dir_vec;       // 144
    Vec world_player_spheres[12]; // 160: hanging 0-5, leap-up 6-11
    CollideEdgeSpecGoal spec; // 352
    uint32_t process;         // 672
    uint32_t pad1[3];
    Vec verts[64];            // 688
    CollideEdgeEdge edges[96]; // 1712
    CollideEdgeTri tris[48];  // 6320
    CollideEdgeHoldList hold_list; // 7856
};
static_assert(offsetof(CollideEdgeWork, spec) == 352, "collide-edge-work");
static_assert(offsetof(CollideEdgeWork, verts) == 688, "collide-edge-work");
static_assert(offsetof(CollideEdgeWork, edges) == 1712, "collide-edge-work");
static_assert(offsetof(CollideEdgeWork, tris) == 6320, "collide-edge-work");
static_assert(offsetof(CollideEdgeWork, hold_list) == 7856, "collide-edge-work");

// The control-info image the assembly reads through collide-edge-work's cshape: a collide-shape
// basic with trans at 16 (GOAL address + 12) and root-prim at 160.
struct CshapeImage {
    uint32_t type_tag; // 0
    uint32_t pad0[3];
    Vec trans;         // 16
    uint8_t pad1[160 - 32];
    uint32_t root_prim; // 160
    uint32_t pad2[3];
};
static_assert(offsetof(CshapeImage, root_prim) == 160, "collide-shape");

// The globals.
inline CollideCache& collide_cache() { return goal_ref<CollideCache>(kAddrCollideCache); }
inline uint32_t collide_cache_addr() { return kAddrCollideCache + 4; }
inline CollideEdgeWork& collide_edge_work() { return goal_ref<CollideEdgeWork>(kAddrEdgeWork); }
inline GoalEdgeGrabInfo& goal_edge_grab_info() { return goal_ref<GoalEdgeGrabInfo>(kAddrEdgeGrabInfo); }
inline CollideShapePrim& prim_backgnd() { return goal_ref<CollideShapePrim>(kAddrPrimBackgnd); }

} // namespace jak
