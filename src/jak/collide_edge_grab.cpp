// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Edge grabs (collide-edge-grab.gc, collide-edge-grab-h.gc), the board's edge specs
// (collide-reaction-target.gc *collide-edge-board-spec*, target-board.gc
// *collide-edge-board-halfpipe-spec*) and rail-surface-touch, ported from OpenGOAL (jak-project,
// ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). find-grabbable-tris,
// find-grabbable-edges, should-add-to-list?, add-to-list! and find-best-grab! are the mips2c
// translations in collide_mips2c.cpp; the debug drawing methods are left out.
#include <cmath>

#include "jak/collide_extern.hpp"
#include "jak/collide_internal.hpp"
#include "jak/collide_mips2c.hpp"
#include "jak/cpad.hpp"
#include "jak/math.hpp"
#include "jak/math_collide.hpp"
#include "jak/target.hpp"

namespace jak {

namespace {

constexpr uint32_t kProbeIgnorePat = 0x1u | 0x10u | (1u << 24) | (1u << 27); // #x9000011
constexpr uint32_t kTargetProcess = 0x200u; // stand-in for the target's process pointer

Vec sph(float x, float y, float r) { return Vec{x, y, 0.0f, r}; }

CollideEdgeSpec g_collide_edge_spec = {
    {1024.0f, 1433.6f},
    {708.608f, 13312.0f, 0.0f, 1.0f},
    edgeflag::send_event,
    {0x1u | 0x4u | 0x10u | (1u << 24) | (1u << 27)}, // noentity noedge nojak probe noendlessfall
    37748736.0f,
    0.6f,
    0.0f,
    nullptr,
    {{-8192.0f, -11059.2f, -8192.0f, 1.0f}, {8192.0f, 24576.0f, 8192.0f, 1.0f}},
    {{-6144.0f, 5324.8f, -6144.0f, 1.0f}, {6144.0f, 11059.2f, 6144.0f, 1.0f}},
    {sph(1720.32f, -819.2f, 1433.6f), sph(2293.76f, -3276.8f, 1884.16f),
     sph(1966.08f, -6144.0f, 1556.48f), sph(1966.08f, -8601.6f, 1556.48f),
     sph(1761.28f, -11059.2f, 1351.68f), sph(1679.36f, -13312.0f, 1269.76f),
     sph(-737.28f, 4096.0f, 3072.0f), sph(-737.28f, 6553.6f, 3072.0f),
     sph(-737.28f, 9420.8f, 3072.0f), sph(1720.32f, 3686.4f, 2949.12f),
     sph(1720.32f, 5734.4f, 2949.12f), sph(1720.32f, 8601.6f, 2949.12f)},
};

CollideEdgeSpec g_collide_edge_board_spec = {
    {1024.0f, 1433.6f},
    {708.608f, 13312.0f, 0.0f, 1.0f},
    edgeflag::find_adjacent_edge | edgeflag::send_event,
    {0x1u | 0x8u | (1u << 24)}, // noentity nogrind probe
    67108864.0f,
    -1.0f,
    -2.0f,
    nullptr,
    {{-12288.0f, -11059.2f, -12288.0f, 1.0f}, {12288.0f, 16384.0f, 12288.0f, 1.0f}},
    {{-11878.4f, -2867.2f, -11878.4f, 1.0f}, {11878.4f, 2867.2f, 11878.4f, 1.0f}},
    {sph(2539.52f, -819.2f, 1433.6f), sph(2539.52f, -819.2f, 1433.6f),
     sph(2539.52f, -819.2f, 1433.6f), sph(2539.52f, -819.2f, 1433.6f),
     sph(2539.52f, -819.2f, 1433.6f), sph(2539.52f, -819.2f, 1433.6f),
     sph(-737.28f, 4096.0f, 3072.0f), sph(-737.28f, 6553.6f, 3072.0f),
     sph(-737.28f, 9420.8f, 3072.0f), sph(1720.32f, 3686.4f, 2949.12f),
     sph(1720.32f, 5734.4f, 2949.12f), sph(1720.32f, 8601.6f, 2949.12f)},
};

CollideEdgeSpec g_collide_edge_board_halfpipe_spec = {
    {1024.0f, 1433.6f},
    {708.608f, 13312.0f, 0.0f, 1.0f},
    edgeflag::send_event,
    {0},
    150994940.0f,
    -1.0f,
    -2.0f,
    nullptr,
    {{-8192.0f, -16384.0f, -8192.0f, 1.0f}, {8192.0f, 16384.0f, 8192.0f, 1.0f}},
    {{-7987.2f, -16179.2f, -7987.2f, 1.0f}, {7987.2f, 16179.2f, 7987.2f, 1.0f}},
    {sph(1720.32f, -819.2f, 1433.6f), sph(1720.32f, -819.2f, 1433.6f),
     sph(1720.32f, -819.2f, 1433.6f), sph(1720.32f, -819.2f, 1433.6f),
     sph(1720.32f, -819.2f, 1433.6f), sph(1720.32f, -819.2f, 1433.6f),
     sph(-737.28f, 4096.0f, 3072.0f), sph(-737.28f, 6553.6f, 3072.0f),
     sph(-737.28f, 9420.8f, 3072.0f), sph(1720.32f, 3686.4f, 2949.12f),
     sph(1720.32f, 5734.4f, 2949.12f), sph(1720.32f, 8601.6f, 2949.12f)},
};

EdgeGrabInfo g_edge_grab_info_host;

// (mem-copy! (-> *collide-edge-work* spec) spec 320): the spec in GOAL's layout. The touching
// segment, when there is one, is copied next to it in the arena.
void copy_spec_in(CollideEdgeSpecGoal& g, const CollideEdgeSpec& s) {
    std::memset(&g, 0, sizeof(g));
    g.split_dists[0] = s.split_dists[0];
    g.split_dists[1] = s.split_dists[1];
    g.outward_offset = s.outward_offset;
    g.flags = s.flags;
    g.ignore_pat = s.ignore_pat;
    g.max_dist_sqrd_to_outward_pt = s.max_dist_sqrd_to_outward_pt;
    g.max_dir_cosa_delta = s.max_dir_cosa_delta;
    g.max_dir_cosa_player = s.max_dir_cosa_player;
    if (s.touching_segment) {
        goal_ref<Vec>(kAddrTouchingSegment) = s.touching_segment[0];
        goal_ref<Vec>(kAddrTouchingSegment + 16) = s.touching_segment[1];
        g.touching_segment = kAddrTouchingSegment;
    } else {
        g.touching_segment = kGoalFalse;
    }
    g.local_cache_fill_box = s.local_cache_fill_box;
    g.local_within_reach_box = s.local_within_reach_box;
    for (int i = 0; i < 12; i++) g.local_player_spheres[i] = s.local_player_spheres[i];
}

const Vec& gravity_normal() { return g_collide_target->control.dynam->gravity_normal; }

GoalCollideQuery& sphere_query(GoalStackFrame& f, uint32_t spheres, uint32_t n) {
    GoalCollideQuery& q = new_stack_query(f);
    CollideEdgeWork& w = collide_edge_work();
    const CshapeImage& cs = goal_ref<CshapeImage>(w.cshape - 4);
    q.spheres() = spheres;
    q.num_spheres() = n;
    q.collide_with = goal_ref<CollideShapePrim>(cs.root_prim - 4).prim_core.collide_with;
    q.ignore_process0 = kGoalFalse;
    q.ignore_process1 = kGoalFalse;
    q.ignore_pat.bits = kProbeIgnorePat;
    q.best_my_prim() = kGoalTrue;
    q.action_mask = caction::solid;
    return q;
}

// (method 13 collide-edge-work) compute-center-point!, on the VU: the length and the projection
// are taken in x and z only (.add.z.vf.x), as the original does.
void compute_center_point(CollideEdgeWork& w, CollideEdgeEdge& e, Vec& out) {
    const Vec& v2 = goal_ref<Vec>(e.vertex_ptr[0]);
    const Vec& v3 = goal_ref<Vec>(e.vertex_ptr[1]);
    const Vec& v1 = w.search_pt;
    const Vec v4{v1.x - v2.x, v1.y - v2.y, v1.z - v2.z, v1.w - v2.w};
    const Vec v5{v3.x - v2.x, v3.y - v2.y, v3.z - v2.z, v3.w - v2.w};
    float v6x = v5.x * v5.x;
    const float v6z = v5.z * v5.z;
    v6x = v6x + v6z;
    const float f1 = std::sqrt(std::fabs(v6x)); // .sqrt.vf takes |x|
    const float f2 = 1.0f / f1;                 // .div.vf Q vf0 vf6: 1.0 / x
    const Vec v9{v5.x * f2, v5.y * f2, v5.z * f2, v5.w * f2};
    float v10x = v9.x * v4.x;
    v10x = v10x + v9.z * v4.z;
    float f3 = v10x;
    if (f3 < 0.0f) f3 = 0.0f;
    else if (f1 < f3) f3 = f1;
    const float t = f3 * f2;
    out = Vec{v5.x * t + v2.x, v5.y * t + v2.y, v5.z * t + v2.z, 1.0f};
}

// (method 9 collide-edge-work) search-for-edges
void search_for_edges(CollideEdgeWork& w, CollideEdgeHoldList& list) {
    list.num_allocs = 0;
    list.num_attempts = 0;
    list.head = kGoalFalse;
    int item = 0;
    for (uint32_t i = 0; i < w.num_edges; i++) {
        CollideEdgeEdge& e = w.edges[i];
        if (e.ignore != kGoalFalse) continue;
        CollideEdgeHoldItem& s4 = list.items[item];
        compute_center_point(w, e, s4.center_pt);
        const uint64_t a[3] = {goal_addr(&w), goal_addr(&s4), goal_addr(&e)};
        if (goal_call(m2c_should_add_to_list, a, 3) != kGoalFalse) {
            const uint64_t b[2] = {goal_addr(&list), goal_addr(&s4)};
            goal_call(m2c_add_to_list, b, 2);
            list.num_allocs++;
            if (list.num_allocs == 32) {
                // "ERROR: Reached limit of edge grab hold items!"
                return;
            }
            item++;
        }
    }
}

// (method 14 collide-edge-work) get-best-hand-point
float get_best_hand_point(CollideEdgeWork& w, Vec& arg0, const Vec& arg1, int arg2) {
    float f30 = -1.0f;
    Vec s2{};
    for (int i = 0; i < static_cast<int>(w.num_edges); i++) {
        CollideEdgeEdge& e = w.edges[i];
        if (e.ignore != kGoalFalse) continue;
        const CollideEdgeTri& et = goal_ref<CollideEdgeTri>(e.etri);
        if (goal_ref<CollideCacheTri>(et.ctri).prim_index != arg2) continue;
        const float f0 = vector_segment_distance_point(arg1, goal_ref<Vec>(e.vertex_ptr[0]),
                                                       goal_ref<Vec>(e.vertex_ptr[1]), &s2);
        if (f30 < 0.0f || f0 < f30) {
            f30 = f0;
            arg0 = s2;
        }
    }
    return f30;
}

// (method 9 collide-edge-edge) no-collision-at-edge
bool no_collision_at_edge(CollideEdgeEdge& e, CollideEdgeWork& w) {
    GoalStackFrame f;
    Mat& s4 = f.alloc_no_clear<Mat>();
    struct Six { Vec s[6]; };
    Six& s5 = f.alloc_no_clear<Six>();
    const Vec& p0 = goal_ref<Vec>(e.vertex_ptr[0]);
    const Vec& p1 = goal_ref<Vec>(e.vertex_ptr[1]);
    s4.v[1] = gravity_normal();
    vector_normalize(vector_sub(s4.v[2], p1, p0), 1.0f);
    vector_normalize(vector_cross(s4.v[0], s4.v[2], s4.v[1]), 1.0f);
    vector_cross(s4.v[2], s4.v[0], s4.v[1]);
    vector_average(s4.v[3], p1, p0);
    transform_vectors(s4, s5.s, w.spec.local_player_spheres, 6);
    GoalCollideQuery& q = sphere_query(f, goal_addr(&s5), 6);
    return !probe_using_spheres(q);
}

// (method 15 collide-edge-work) find-adjacent-edge
void find_adjacent_edge(CollideEdgeWork& w, CollideEdgeHoldItem& item, GoalEdgeGrabInfo& info) {
    const CollideEdgeEdge& v1 = goal_ref<CollideEdgeEdge>(item.edge);
    const uint32_t s3 = v1.vertex_ptr[0];
    const uint32_t s2 = v1.vertex_ptr[1];
    bool found_left = false, found_right = false;
    float left_dot = 0.0f, right_dot = 0.0f; // stack-no-clear; only read once found
    Vec hold{}, adj{};
    vector_sub(hold, goal_ref<Vec>(s2), goal_ref<Vec>(s3));
    vector_normalize(hold, 1.0f);
    for (uint32_t i = 0; i < w.num_edges; i++) {
        CollideEdgeEdge& s1 = w.edges[i];
        if (s1.ignore != kGoalFalse) continue;
        if (s1.vertex_ptr[1] == s3) {
            vector_sub(adj, goal_ref<Vec>(s1.vertex_ptr[1]), goal_ref<Vec>(s1.vertex_ptr[0]));
            vector_normalize(adj, 1.0f);
            const float f30 = vector_dot(adj, hold);
            if ((!found_left || left_dot < f30 || -0.7f < f30) && no_collision_at_edge(s1, w)) {
                left_dot = f30;
                found_left = true;
                info.adjacent_edge_left_vertex() = goal_ref<Vec>(s1.vertex_ptr[0]);
            }
        }
        if (s1.vertex_ptr[0] == s2) {
            vector_sub(adj, goal_ref<Vec>(s1.vertex_ptr[1]), goal_ref<Vec>(s1.vertex_ptr[0]));
            vector_normalize(adj, 1.0f);
            const float f30 = vector_dot(adj, hold);
            if ((!found_right || right_dot < f30 || -0.7f < f30) && no_collision_at_edge(s1, w)) {
                right_dot = f30;
                found_right = true;
                info.adjacent_edge_right_vertex() = goal_ref<Vec>(s1.vertex_ptr[1]);
            }
        }
    }
    uint64_t st = info.status;
    if (found_left) st |= 1;
    if (found_right) st |= 2;
    info.status = st;
}

Vec hold_point(const Vec& c, const Vec& n) {
    const float k = 1105.92f;
    return Vec{n.x * k + c.x * 1.0f, n.y * k + c.y * 1.0f, n.z * k + c.z * 1.0f, 1.0f};
}

} // namespace

// (method 20 collide-edge-work) check-grab-for-collisions, called from find-best-grab!.
uint64_t goal_check_grab_for_collisions(mips::Ctx* c) {
    CollideEdgeWork& w = goal_ref<CollideEdgeWork>(static_cast<uint32_t>(c->sgpr64(mips::a0)));
    CollideEdgeHoldItem& item =
        goal_ref<CollideEdgeHoldItem>(static_cast<uint32_t>(c->sgpr64(mips::a1)));
    GoalEdgeGrabInfo& info = goal_ref<GoalEdgeGrabInfo>(static_cast<uint32_t>(c->sgpr64(mips::a2)));
    CollideEdgeEdge& s2 = goal_ref<CollideEdgeEdge>(item.edge);
    CollideCacheTri& s1 = goal_ref<CollideCacheTri>(goal_ref<CollideEdgeTri>(s2.etri).ctri);
    const int s4 = s1.prim_index;
    {
        const Vec s0 = hold_point(item.center_pt, s2.edge_vec_norm);
        if (491.52f < get_best_hand_point(w, info.right_hand_hold, s0, s4)) return kGoalFalse;
    }
    {
        Vec neg;
        vector_negate(neg, s2.edge_vec_norm);
        const Vec s0 = hold_point(item.center_pt, neg);
        if (491.52f < get_best_hand_point(w, info.left_hand_hold, s0, s4)) return kGoalFalse;
    }
    info.tri_vertex()[0] = s1.vertex[0];
    info.world_vertex[4] = s1.vertex[1];
    info.world_vertex[5] = s1.vertex[2];
    info.edge_tri_pat = s1.pat.bits;
    info.center_hold() = item.center_pt;
    info.world_vertex[0] = goal_ref<Vec>(s2.vertex_ptr[0]);
    info.world_vertex[1] = goal_ref<Vec>(s2.vertex_ptr[1]);
    Mat& m = info.hanging_matrix;
    m.v[1] = gravity_normal();
    vector_normalize(vector_sub(m.v[2], info.world_vertex[1], info.world_vertex[0]), 1.0f);
    vector_normalize(vector_cross(m.v[0], m.v[2], m.v[1]), 1.0f);
    vector_cross(m.v[2], m.v[0], m.v[1]);
    m.v[3] = info.center_hold();
    transform_vectors(m, w.world_player_spheres, w.spec.local_player_spheres, 12);
    {
        GoalStackFrame f;
        GoalCollideQuery& q = sphere_query(f, goal_addr(&w.world_player_spheres[0]), 12);
        if (probe_using_spheres(q)) return kGoalFalse;
    }
    info.status = 0;
    if (w.spec.flags & edgeflag::find_adjacent_edge) find_adjacent_edge(w, item, info);
    // The prim is the background's (cshape #f): an actor's would store its offset, handle and
    // the vertices in its joint's space.
    info.actor_cshape_prim_offset = 0;
    info.actor_handle = 0; // (the-as handle #f)
    return 0;              // (the-as symbol 0)
}

void copy_edge_grab_info_out() {
    GoalEdgeGrabInfo& g = goal_edge_grab_info();
    EdgeGrabInfo& h = g_edge_grab_info_host;
    for (int i = 0; i < 8; i++) h.world_vertex[i] = g.world_vertex[i];
    for (int i = 0; i < 4; i++) h.hanging_matrix[i] = g.hanging_matrix.v[i];
    h.hanging_matrix_row0 = g.hanging_matrix.v[0];
    h.left_hand_hold = g.left_hand_hold;
    h.right_hand_hold = g.right_hand_hold;
    h.center_hold_old = g.center_hold_old;
    h.adjacent_edge_left_vertex = g.adjacent_edge_left_vertex();
    h.adjacent_edge_right_vertex = g.adjacent_edge_right_vertex();
    h.edge_tri_pat = g.edge_tri_pat;
    h.status = static_cast<uint32_t>(g.status);
    h.found_edge = g.found_edge();
}

CollideEdgeSpec& collide_edge_spec() { return g_collide_edge_spec; }
CollideEdgeSpec& collide_edge_board_spec() { return g_collide_edge_board_spec; }
CollideEdgeSpec& collide_edge_board_halfpipe_spec() { return g_collide_edge_board_halfpipe_spec; }
uint64_t& edge_spec_flags(CollideEdgeSpec& spec) { return spec.flags; }
void edge_spec_set_touching_segment(CollideEdgeSpec& spec, const Vec* segment) {
    spec.touching_segment = segment;
}
EdgeGrabInfo& edge_grab_info() { return g_edge_grab_info_host; }

// (method 10 target) do-edge-grabs
void do_edge_grabs(Target& self, CollideEdgeSpec& arg1) {
    target_collide_init(self);
    g_collide_target = &self;
    sync_cshape(self);
    GoalEdgeGrabInfo& info = goal_edge_grab_info();
    info.found_edge_p = kGoalFalse;
    CollideEdgeWork& s5 = collide_edge_work();
    copy_spec_in(s5.spec, arg1);
    s5.process = kTargetProcess;
    s5.num_verts = 0;
    s5.num_edges = 0;
    s5.num_tris = 0;
    const Vec& t = self.control.trans;
    s5.ccache = collide_cache_addr();
    s5.cshape = kAddrCshape + 4;
    auto add3 = [&](const Vec& a) { return Vec{a.x + t.x, a.y + t.y, a.z + t.z, a.w}; };
    s5.cache_fill_box.min = add3(s5.spec.local_cache_fill_box.min);
    s5.cache_fill_box.max = add3(s5.spec.local_cache_fill_box.max);
    const Vec v4 = add3(s5.spec.local_within_reach_box.min);
    const Vec v5 = add3(s5.spec.local_within_reach_box.max);
    s5.within_reach_box.min = v4;
    s5.within_reach_box.max = v5;
    for (int i = 0; i < 4; i++) {
        s5.within_reach_box4w.min[i] = mips::f2i_x86(v4[i]);
        s5.within_reach_box4w.max[i] = mips::f2i_x86(v5[i]);
    }
    {
        GoalStackFrame f;
        GoalCollideQuery& s3 = new_stack_query(f);
        s3.collide_with = self.control.root_prim->prim_core.collide_with;
        s3.ignore_process0 = kTargetProcess;
        s3.ignore_process1 = kGoalFalse;
        s3.ignore_pat = s5.spec.ignore_pat;
        s3.action_mask = caction::solid;
        s3.bbox = s5.cache_fill_box;
        fill_using_bounding_box(self.world, s3);
    }
    const uint64_t a[1] = {goal_addr(&s5)};
    goal_call(m2c_find_grabbable_tris, a, 1);
    if (s5.num_tris != 0) {
        goal_call(m2c_find_grabbable_edges, a, 1);
        if (s5.num_edges != 0) {
            s5.search_pt = self.control.midpoint_of_hands;
            const uint64_t b[3] = {goal_addr(&s5), goal_addr(&s5.hold_list), goal_addr(&info)};
            auto found = [&]() {
                info.found_edge_p = kGoalTrue;
                if (s5.spec.flags & edgeflag::send_event) {
                    CollideEvent e;
                    e.message = "edge-grab";
                    send_event(self, e);
                }
            };
            bool done = false;
            if (self.control.cpad->stick0_speed != 0.0f) {
                s5.search_dir_vec = self.control.to_target_pt_xz;
                search_for_edges(s5, s5.hold_list);
                if (goal_call(m2c_find_best_grab, b, 3) != kGoalFalse) {
                    found();
                    done = true; // (return #f)
                }
            }
            if (!done) {
                vector_z_quaternion(s5.search_dir_vec, self.control.quat_for_control);
                search_for_edges(s5, s5.hold_list);
                if (goal_call(m2c_find_best_grab, b, 3) != kGoalFalse) found();
            }
        }
    }
    copy_edge_grab_info_out();
}

// edge-grab-info-method-9 (mips2c, collide_edge_grab.cpp), the background branch
// (actor-cshape-prim-offset 0). An actor's edge would first be carried along with its joint and
// sent 'edge-grabbed; the port has no actors.
bool edge_grab_info_method_9(EdgeGrabInfo& out) {
    if (!g_collide_target) return true;
    Target& self = *g_collide_target;
    sync_cshape(self);
    GoalEdgeGrabInfo& gp = goal_edge_grab_info();
    gp.center_hold_old = gp.center_hold();
    CollideEdgeWork& w = collide_edge_work();
    // on the board (focus-status board) the hanging spheres; on foot the leap-up ones when the
    // target's next state is target-edge-grab-jump, which the port's target never is.
    const uint32_t spheres = goal_addr(&w.world_player_spheres[0]);
    bool hit;
    {
        GoalStackFrame f;
        GoalCollideQuery& q = sphere_query(f, spheres, 6);
        hit = fill_and_probe_using_spheres(self.world, q);
    }
    copy_edge_grab_info_out();
    out = g_edge_grab_info_host;
    return !hit;
}

// rail-surface-touch (collide-reaction-target.gc)
void rail_surface_touch(Target& self) {
    if (time_elapsed(self, self.control.time_of_last_surface_change, seconds(0.2))) {
        g_collide_edge_board_spec.flags &= ~(edgeflag::one | edgeflag::send_event);
        g_collide_edge_board_spec.touching_segment = nullptr;
        do_edge_grabs(self, g_collide_edge_board_spec);
        g_collide_edge_board_spec.flags |= edgeflag::send_event;
        const EdgeGrabInfo& gp = g_edge_grab_info_host;
        if (gp.found_edge) {
            vector_normalize(vector_sub(self.control.edge_grab_edge_dir, gp.world_vertex[0], gp.world_vertex[1]),
                             1.0f);
            self.control.edge_grab_across_edge_dir = gp.hanging_matrix_row0;
            CollideEvent e;
            e.message = "push-transv";
            vector_float_mul(e.push, self.control.edge_grab_across_edge_dir,
                             seek(vector_length(self.control.additional_decaying_velocity), 40960.0f,
                                  81920.0f * seconds_per_frame(self)));
            e.time = seconds(0.3);
            send_event(self, e);
        }
    }
}

// (do-edge-grabs *target* *collide-cache* *collide-edge-spec*), as the movement core asks for it.
void do_edge_grabs_target(Target& self) { do_edge_grabs(self, g_collide_edge_spec); }

} // namespace jak
