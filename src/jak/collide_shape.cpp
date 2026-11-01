// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The target's collision shape and its move (collide-shape.gc; logic-target.gc init-target;
// target-util.gc target-collide-set!), ported from OpenGOAL (jak-project, ISC License, Copyright
// (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). The prims live in the arena (collide_mem.hpp) at
// kAddrTargetPrims, the root group first and its nine spheres after it, as init-target allocates
// them one after the other; Jak II walks them with a stride of 80 bytes and so does the port.
// The touching list (add-touching-prims, update-from-step-size) and collision against other
// actors' prims are not ported: the port has no other actors.
#include <cmath>

#include "jak/collide_extern.hpp"
#include "jak/collide_internal.hpp"
#include "jak/collide_mips2c.hpp"
#include "jak/math.hpp"
#include "jak/math_collide.hpp"
#include "jak/target.hpp"

namespace jak {

Target* g_collide_target = nullptr;

bool send_event(Target& self, const CollideEvent& ev) {
    return self.collide_event ? self.collide_event(self, ev) : false;
}

namespace {

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

constexpr int kTargetPrims = 10;
// collision-spheres i -> the child it is (slot 5 is never set by init-target).
constexpr int kSphereChild[10] = {1, 2, 3, 4, 5, -1, 6, 7, 8, 9};

// The target's pat-ignore-mask (collide-shape constructor, process type target):
// noentity, nojak, probe.
constexpr uint32_t kTargetPatIgnore = 0x1u | 0x10u | (1u << 24);

float fminf_vu(float a, float b) { return a < b ? a : b; }
float fmaxf_vu(float a, float b) { return a > b ? a : b; }

} // namespace

CollideShapePrim& target_prim(int child) {
    return goal_ref<CollideShapePrim>(kAddrTargetPrims + 80u * static_cast<uint32_t>(child));
}

CollideShapePrim* target_collision_sphere(int i) {
    if (i < 0 || i > 9 || kSphereChild[i] < 0) return nullptr;
    return &target_prim(kSphereChild[i]);
}

// The control-info fields the edge-grab assembly reads through collide-edge-work's cshape.
void sync_cshape(Target& self) {
    CshapeImage& img = goal_ref<CshapeImage>(kAddrCshape);
    img.type_tag = 3;
    img.trans = self.control.trans;
    img.root_prim = goal_addr_basic(&target_prim(0));
}

// ---- init-target (logic-target.gc), the shape ----

void target_collide_init(Target& self) {
    collide_arena_init();
    g_collide_target = &self;
    ControlInfo& c = self.control;
    if (c.root_prim == &target_prim(0)) return;
    const uint32_t cshape = kAddrCshape + 4;
    std::memset(&target_prim(0), 0, 80u * kTargetPrims);
    // (new 'process 'collide-shape-prim-group s0-0 9 1)
    CollideShapePrim& g = target_prim(0);
    g.type_tag = 4;
    g.cshape = cshape;
    g.prim_id = 1;
    g.transform_index = -2;
    g.prim_core.prim_type = kPrimGroup;
    g.num_children() = 9;
    g.child() = goal_addr_basic(&target_prim(1));
    c.total_prims = 10;
    g.prim_core.action = caction::solid | caction::can_ride;
    g.local_sphere = Vec{0.0f, 0.0f, 0.0f, 4096.0f};
    c.root_prim = &g;
    g.prim_core.collide_as = 2;
    g.prim_core.collide_with = 0x21c3f7d;
    // (new 'process 'collide-shape-prim-sphere s0-0 id): transform-index -2, pat obstacle/stone.
    const uint32_t ids[9] = {6, 10, 18, 34, 0, 128, 256, 512, 1024};
    for (int i = 0; i < 9; i++) {
        CollideShapePrim& s = target_prim(1 + i);
        s.type_tag = 5;
        s.cshape = cshape;
        s.prim_id = ids[i];
        s.transform_index = -2;
        s.prim_core.prim_type = kPrimSphere;
        s.pat().bits = (2u << 7) | (23u << 10);
        s.local_sphere = Vec{0.0f, 0.0f, 0.0f, 4096.0f};
    }
    CollideShapePrim* cs[10];
    for (int i = 0; i < 10; i++) cs[i] = target_collision_sphere(i);
    cs[0]->prim_core.action = caction::solid | caction::can_ride;
    cs[0]->prim_core.collide_as = 2;
    cs[0]->prim_core.collide_with = 0x21c3f7d;
    for (int i = 1; i <= 2; i++) {
        cs[i]->prim_core.action = caction::solid;
        cs[i]->prim_core.collide_as = 2;
        cs[i]->prim_core.collide_with = 0x21c3f7d;
    }
    cs[3]->prim_core.action = caction::solid;
    cs[3]->prim_core.collide_as = 0;
    cs[3]->prim_core.collide_with = 0;
    cs[4]->prim_core.action = caction::semi_solid;
    cs[4]->transform_index = 6;
    cs[4]->local_sphere = Vec{0.0f, 0.0f, 0.0f, 2048.0f};
    cs[4]->prim_core.collide_as = 2;
    cs[4]->prim_core.collide_with = 0x21c3f7d;
    for (int i = 6; i <= 9; i++) cs[i]->prim_core.action = caction::deadly;
    c.nav_radius = 0.75f * c.root_prim->local_sphere.w;
    c.backup_collide_as = g.prim_core.collide_as;
    c.backup_collide_with = g.prim_core.collide_with;
    c.event_priority = 9;
    c.default_collide_as_all = 2;
    c.default_collide_with_all = 0x21c3f7d;
    c.default_collide_as_fgnd = 2;
    c.default_collide_with_fgnd = 0x21c377c;
    c.max_iteration_count = 8;
    c.event_self = Symbol("touched");
    // the collide-shape constructor (collide-shape-h.gc) for a process of type target
    c.pat_ignore_mask.bits = kTargetPatIgnore;
    c.trans.w = 1.0f;
}

// ---- target-collide-set! (target-util.gc) ----

// ---- collide-shape methods ----

// update-transforms. The target has a node-list, so the cspace branch: a joint index takes the
// joint's matrix (Target::joint_transform), -2 the root's trans.
void update_transforms(Target& self) {
    target_collide_init(self);
    ControlInfo& c = self.control;
    for (int i = 0; i < c.total_prims; i++) {
        CollideShapePrim& p = target_prim(i);
        const int idx = p.transform_index;
        if (idx >= 0) {
            const Mat* m = self.joint_transform ? self.joint_transform(self, idx) : nullptr;
            if (!m) continue;
            const Vec& l = p.local_sphere;
            const float q = 1.0f / m->v[3].w; // (.div.vf Q vf0 vf5 :fsf #b11 :ftf #b11)
            Vec w = l;
            for (int k = 0; k < 3; k++) {
                float acc = (&m->v[3].x)[k] * 1.0f;
                acc = acc + (&m->v[0].x)[k] * l.x;
                acc = acc + (&m->v[1].x)[k] * l.y;
                (&w.x)[k] = (acc + (&m->v[2].x)[k] * l.z) * q;
            }
            p.prim_core.world_sphere = w;
        } else if (idx == -2) {
            Vec w = p.local_sphere;
            w.x = w.x + c.trans.x;
            w.y = w.y + c.trans.y;
            w.z = w.z + c.trans.z;
            p.prim_core.world_sphere = w;
        }
    }
}

// move-by-vector!
void move_by_vector(Target& self, const Vec& v) {
    target_collide_init(self);
    ControlInfo& c = self.control;
    vector_add(c.trans, c.trans, v);
    for (int i = 0; i < c.total_prims; i++) {
        CollideShapePrim& p = target_prim(i);
        vector_add(p.prim_core.world_sphere, p.prim_core.world_sphere, v);
        p.prim_core.world_sphere.w = p.local_sphere.w;
    }
}

// move-to-point!
void move_to_point(Target& self, const Vec& p) {
    target_collide_init(self);
    ControlInfo& c = self.control;
    Vec d;
    vector_sub(d, p, c.trans);
    c.trans = p;
    for (int i = 0; i < c.total_prims; i++) {
        CollideShapePrim& pr = target_prim(i);
        vector_add(pr.prim_core.world_sphere, pr.prim_core.world_sphere, d);
        pr.prim_core.world_sphere.w = pr.local_sphere.w;
    }
}

// modify-collide-as!
void modify_collide_as(Target& self, int arg0, uint32_t clear_as, uint32_t set_as) {
    target_collide_init(self);
    for (int i = 0; i < self.control.total_prims; i++) {
        CollideShapePrim& p = target_prim(i);
        if (p.prim_id & static_cast<uint32_t>(arg0)) {
            p.prim_core.collide_as = (p.prim_core.collide_as & ~clear_as) | set_as;
        }
    }
}

// build-bounding-box-for-shape, its branches as the assembly takes them.
bool build_bounding_box_for_shape(Target& self, BoundingBox& box, float arg1, uint32_t arg2) {
    target_collide_init(self);
    CollideShapePrim* v1 = self.control.root_prim;
    float vf31x = arg1;
    if (!(v1->prim_core.collide_with & arg2)) return false;
    vf31x = vf31x + 4.096f;
    int a0 = v1->specific[0];
    Vec vf24 = v1->prim_core.world_sphere;
    Vec vf29{}, vf30{};
    auto grow = [&](const Vec& c, float r, Vec& lo, Vec& hi) {
        hi = Vec{c.x + r, c.y + r, c.z + r, hi.w};
        lo = Vec{c.x - r, c.y - r, c.z - r, lo.w};
    };
    if (v1->prim_core.prim_type != kPrimGroup) {
        const float r = vf31x + vf24.w;
        grow(vf24, r, vf29, vf30);
    } else {
        // cfg-3: the first child that collides with arg2 starts the box
        for (;;) {
            if (a0 == 0) return false;
            v1 = reinterpret_cast<CollideShapePrim*>(reinterpret_cast<uint8_t*>(v1) + 80);
            a0--;
            vf24 = v1->prim_core.world_sphere;
            if (v1->prim_core.collide_with & arg2) break;
        }
        grow(vf24, vf31x + vf24.w, vf29, vf30);
        // cfg-6: the others widen it
        for (;;) {
            if (a0 == 0) break;
            v1 = reinterpret_cast<CollideShapePrim*>(reinterpret_cast<uint8_t*>(v1) + 80);
            a0--;
            vf24 = v1->prim_core.world_sphere;
            if (!(v1->prim_core.collide_with & arg2)) continue;
            Vec vf26{}, vf27{};
            grow(vf24, vf31x + vf24.w, vf26, vf27);
            vf29 = Vec{fminf_vu(vf29.x, vf26.x), fminf_vu(vf29.y, vf26.y), fminf_vu(vf29.z, vf26.z), 0};
            vf30 = Vec{fmaxf_vu(vf30.x, vf27.x), fmaxf_vu(vf30.y, vf27.y), fmaxf_vu(vf30.z, vf27.z), 0};
        }
    }
    vf29.w = 1.0f;
    vf30.w = 1.0f;
    box.min = vf29;
    box.max = vf30;
    return true;
}

// fill-cache-for-shape
void fill_cache_for_shape(Target& self, float pad, GoalCollideQuery& q) {
    if (build_bounding_box_for_shape(self, q.bbox, pad, q.collide_with)) {
        fill_using_bounding_box(self.world, q);
        // *display-collide-cache* debug draw left out
    } else {
        collide_cache_reset();
    }
}

// fill-cache-integrate-and-collide
void fill_cache_integrate_and_collide(Target& self, Vec& transv, GoalCollideQuery& q, float pad) {
    const float spf = seconds_per_frame(self);
    const Vec v{transv.x * spf, transv.y * spf, transv.z * spf, 1.0f};
    fill_cache_for_shape(self, vector_length(v) + pad, q);
    integrate_and_collide_control(self, transv);
}

void fill_cache_integrate_and_collide(Target& self, Vec& transv, const CollideQuery& hq, float pad) {
    target_collide_init(self);
    g_collide_target = &self;
    GoalStackFrame f;
    GoalCollideQuery& q = new_stack_query(f);
    q.collide_with = hq.collide_with;
    q.ignore_process0 = hq.ignore_self ? 0x200u : kGoalFalse;
    q.ignore_process1 = kGoalFalse;
    q.ignore_pat = hq.ignore_pat;
    q.action_mask = hq.action_mask;
    fill_cache_integrate_and_collide(self, transv, q, pad);
}

// target-real-post's move, as the movement core asks for it (target-util.gc / logic-target.gc).
void target_integrate_and_collide(Target& self, const Vec& transv, float padding) {
    target_collide_init(self);
    CollideQuery q;
    q.collide_with = self.control.root_prim->prim_core.collide_with;
    q.ignore_self = true;
    q.ignore_pat = self.control.pat_ignore_mask;
    q.action_mask = caction::solid;
    // GOAL passes the control's transv itself, which the move updates in place.
    fill_cache_integrate_and_collide(self, const_cast<Vec&>(transv), q, padding);
}

namespace {

// collide-with-collide-cache-prim-mesh (collide-shape-prim-sphere)
void collide_with_cache_prim_mesh_sphere(CollideShapePrim& self_prim, GoalCollideQuery& q,
                                         CollideCachePrim& cprim) {
    GoalStackFrame f;
    CollideTriResult& gp = f.alloc_no_clear<CollideTriResult>();
    const uint64_t args[6] = {goal_addr(&cprim), goal_addr(&gp), goal_addr(&self_prim.prim_core),
                              goal_addr(&q.move_dist), bits_of(q.best_dist()),
                              self_prim.prim_core.action};
    const float f0 = float_of(goal_call(m2c_resolve_moving_sphere_tri, args, 6));
    if (f0 >= 0.0f) {
        const uint32_t a1 = (self_prim.prim_core.action & cprim.prim_core.action) & caction::solid;
        const CollideShapePrim& other = goal_ref<CollideShapePrim>(cprim.prim - 4);
        const uint32_t v1_5 = other.cshape;
        if (a1 == 0) return;
        uint32_t a2 = cprim.prim;
        if (v1_5 == kGoalFalse) {
            a2 = kGoalFalse;
        }
        // else: (logtest? penetrate-using penetrated-by) against an actor's shape; never here.
        q.best_dist() = f0;
        q.bot_vertex[0] = gp.vertex[0];
        q.bot_vertex[1] = gp.vertex[1];
        q.bot_vertex[2] = gp.vertex[2];
        q.bot_intersect = gp.intersect;
        q.bot_normal = gp.normal;
        q.bot_pat = gp.pat;
        q.bot_collide_ptr = gp.collide_ptr;
        q.num_spheres() = a2;
        q.best_my_prim() = goal_addr_basic(&self_prim);
        // add-touching-prims: only for another shape (v1-5 not #f); the background never is one.
    }
}

// collide-with-collide-cache-prim-mesh (collide-shape-prim-group)
void collide_with_cache_prim_mesh_group(CollideShapePrim& self_prim, GoalCollideQuery& q,
                                        CollideCachePrim& cprim) {
    const uint32_t s4 = cprim.prim_core.collide_as;
    CollideShapePrim* s3 = &goal_ref<CollideShapePrim>(self_prim.child() - 4);
    for (int n = self_prim.num_children(); n != 0; n--) {
        if (s3->prim_core.collide_with & s4) {
            if (s3->prim_core.prim_type == kPrimSphere) {
                collide_with_cache_prim_mesh_sphere(*s3, q, cprim);
            }
            // a mesh or group child: an "ERROR: ... not currently supported!" in Jak II
        }
        s3 = reinterpret_cast<CollideShapePrim*>(reinterpret_cast<uint8_t*>(s3) + 80);
    }
}

} // namespace

// step-collison!
float step_collison(Target& self, Vec& arg0, Vec& arg1, float arg2, int /*arg3*/) {
    ControlInfo& c = self.control;
    GoalStackFrame f;
    GoalCollideQuery& s5 = f.alloc<GoalCollideQuery>();
    Vec s2;
    vector_float_mul(s2, arg1, arg2 * seconds_per_frame(self));
    s5.move_dist = s2;
    s5.best_dist() = kCollisionMiss;
    s5.best_my_prim() = kGoalFalse;
    s5.num_spheres() = kGoalFalse;
    CollideShapePrim& s1 = *c.root_prim;
    CollideCache& cc = collide_cache();
    for (int i = 0, n = cc.num_prims; n != 0; n--, i++) {
        CollideCachePrim& s0 = cc.prims[i];
        if (s1.prim_core.collide_with & s0.prim_core.collide_as) {
            if (s0.prim_core.prim_type >= 0) {
                if (s1.prim_core.prim_type == kPrimGroup) {
                    collide_with_cache_prim_mesh_group(s1, s5, s0);
                } else if (s1.prim_core.prim_type == kPrimSphere) {
                    collide_with_cache_prim_mesh_sphere(s1, s5, s0);
                }
            }
            // else collide-with-collide-cache-prim-sphere: an actor's sphere, never in the cache
        }
    }
    float f30 = s5.best_dist();
    if (f30 >= 0.0f) {
        c.prev_status = target_collision_reaction(self, s5, arg0, arg1);
        // *display-collision-marks* debug drawing left out
    } else {
        c.reaction_flag = 0;
        target_collision_no_reaction(self, s5, arg0, arg1);
        c.prev_status = 0;
        move_by_vector(self, s2);
        arg0 = arg1;
        f30 = 1.0f;
    }
    return f30;
}

// integrate-and-collide! (collide-shape-moving)
void integrate_and_collide_moving(Target& self, Vec& arg0) {
    ControlInfo& c = self.control;
    update_transforms(self);
    c.trans_old_old_old = c.trans_old_old;
    c.trans_old_old = c.trans_old;
    c.trans_old = c.trans;
    c.prev_status = c.status;
    c.status &= ~(cstat::on_surface | cstat::on_ground | cstat::touch_surface | cstat::touch_wall |
                  cstat::touch_ceiling | cstat::touch_actor | cstat::on_special_surface |
                  cstat::touch_edge | cstat::blocked | cstat::on_water | cstat::impact_surface |
                  cstat::touch_background | cstat::stuck | cstat::glance);
    if (!(c.root_prim->prim_core.action & caction::no_normal_reset)) {
        const Vec v1 = c.dynam->gravity_normal;
        c.local_normal = v1;
        c.surface_normal = v1;
        c.poly_normal = v1;
        c.coverage = 0.0f;
        c.touch_angle = 0.0f;
    }
    float f30 = 1.0f;
    int s4 = 0;
    while (0.05f < f30 && s4 < static_cast<int>(c.max_iteration_count) &&
           !(arg0.x == 0.0f && arg0.y == 0.0f && arg0.z == 0.0f)) {
        const float f28 = step_collison(self, arg0, arg0, f30, s4);
        // (update-from-step-size *touching-list* f28): no touching list in the port
        f30 = f30 - f28 * f30;
        s4++;
    }
}

// integrate-and-collide! (control-info)
void integrate_and_collide_control(Target& self, Vec& arg0) {
    ControlInfo& c = self.control;
    const float spf = seconds_per_frame(self);
    if (1638400.0f < vector_length(arg0)) {
        // "WARNING: target vel is ~M m/s, reseting to zero."
        vector_reset(arg0);
    }
    c.old_anim_collide_offset_world = c.anim_collide_offset_world;
    vector_matrix_mul(c.anim_collide_offset_world, c.anim_collide_offset_local, c.ctrl_orientation);
    vector_sub(c.anim_collide_offset_delta_world, c.anim_collide_offset_world,
               c.old_anim_collide_offset_world);
    {
        Vec total;
        vector_sub(total, c.draw_offset, c.anim_collide_offset_world);
        vector_seek(c.cspace_offset, total, 16384.0f * spf);
    }
    {
        Vec bonus{};
        vector_add_float_mul(bonus, c.collide_extra_velocity, c.anim_collide_offset_delta_world, 60.0f);
        if (0.0f < vector_length(bonus)) {
            const uint8_t old_iter = c.max_iteration_count;
            const Vec old_in = arg0;
            const CollideStatus old_stat = c.status;
            integrate_and_collide_moving(self, bonus);
            c.max_iteration_count = old_iter;
            arg0 = old_in;
            c.status |= old_stat;
        }
    }
    Vec regular = arg0;
    const Vec before = arg0;
    integrate_and_collide_moving(self, regular);
    Vec b1 = before;
    Vec a1 = regular;
    const Vec& gn = c.dynam->gravity_normal;
    {
        Vec bn{};
        const float f0 = vector_dot(gn, b1);
        Vec t;
        vector_sub(bn, b1, vector_float_mul(t, gn, f0));
        const float vel = vector_length(bn);
        const float f1 = vel;
        const float f2 = 0.0f;
        Vec g0;
        vector_float_mul(g0, gn, f2);
        vector_float_mul(bn, bn, goal_div(vel, f1));
        vector_add(b1, g0, bn);
    }
    {
        Vec an{};
        const float f0 = vector_dot(gn, a1);
        Vec t;
        vector_sub(an, a1, vector_float_mul(t, gn, f0));
        const float vel = vector_length(an);
        const float f1 = vel;
        const float f2 = 0.0f;
        Vec g0;
        vector_float_mul(g0, gn, f2);
        vector_float_mul(an, an, goal_div(vel, f1));
        vector_add(a1, g0, an);
    }
    vector_normalize(b1, 1.0f);
    vector_normalize(a1, 1.0f);
    const float ba = vector_dot(b1, a1);
    if (vector_length(c.target_transv) != 0.0f &&
        ((c.status & cstat::touch_wall) ? ba < 0.9999f : ba < 0.95f)) {
        c.blocked_factor = seek(c.blocked_factor, 1.0f, 4.0f * spf);
        c.blocked_in_air_factor =
            seek(c.blocked_in_air_factor, c.mod_surface->mode == Symbol("air") ? 1.0f : 0.0f,
                 4.0f * spf);
        c.status |= cstat::blocked;
    } else {
        c.blocked_factor = seek(c.blocked_factor, 0.0f, 2.0f * spf);
        c.blocked_in_air_factor = seek(c.blocked_in_air_factor, 0.0f, 2.0f * spf);
    }
    arg0 = regular;
    if ((c.status & cstat::on_surface) && !(c.status & (cstat::touch_wall | cstat::blocked)) &&
        vector_length(c.btransv) < vector_length(before)) {
        c.btransv = before;
    }
    Vec dir;
    vector_normalize_copy(dir, c.align_xz_vel, 1.0f);
    const float speed = vector_length(c.align_xz_vel);
    c.zx_vel_frac = speed == 0.0f ? 0.0f : std::fmax(0.0f, goal_div(vector_dot(c.transv, dir), speed));
    // the *collide-stats* stopwatch left out
}

// react-to-pat!
uint32_t react_to_pat(Target& self, PatSurface arg0) {
    ControlInfo& c = self.control;
    uint32_t set_flags = 0;
    c.cur_pat = arg0;
    c.poly_pat = arg0;
    switch (arg0.material()) {
    case 1: c.surf = &ice_surface; break;        // ice
    case 14: c.surf = &gravel_surface; break;    // gravel
    case 2: c.surf = &quicksand_surface; break;  // quicksand
    case 18: c.surf = &no_walk_surface; break;   // tube
    default: c.surf = &stone_surface; break;     // *standard-ground-surface*
    }
    auto attack = [&](const char* msg, const char* mode, bool shove) {
        CollideEvent e;
        e.message = msg;
        e.has_attack = true;
        e.id = 2;
        e.mode = mode;
        if (shove) {
            e.has_shove_up = true;
            e.shove_up = meters(3);
        }
        send_event(self, e);
    };
    if (arg0.event() != 0) {
        switch (arg0.event()) {
        case 7: { // slide
            c.surf = &gravel_surface;
            CollideEvent e;
            e.message = "slide";
            send_event(self, e);
            break;
        }
        case 14: c.surf = &gravel_surface; break; // slippery
        case 13:                                  // rail: the target is a process-focusable
            if (!(self.focus_status & fstat::rail)) c.surf = &rail_surface;
            break;
        case 1: set_flags |= csrf(14); attack("attack", "deadly", true); break;
        case 3: set_flags |= csrf(14); attack("attack", "burn", true); break;
        case 4: set_flags |= csrf(14); target_attack_up(self, "attack-or-shove", "deadlyup"); break;
        case 11: set_flags |= csrf(14); target_attack_up(self, "attack-or-shove", "shockup"); break;
        case 5:
            if (!(self.focus_status & fstat::pilot)) {
                set_flags |= csrf(14);
                target_attack_up(self, "attack-or-shove", "burnup");
            }
            break;
        case 6: set_flags |= csrf(14); attack("attack-invinc", "melt", false); break;
        case 2: set_flags |= csrf(14); attack("attack-invinc", "endlessfall", false); break;
        case 10: set_flags |= csrf(14); attack("attack-invinc", "shock", false); break;
        case 8: {
            CollideEvent e;
            e.message = "lip";
            e.param = "lip";
            send_event(self, e);
            break;
        }
        case 9: {
            CollideEvent e;
            e.message = "lip";
            e.param = "lipramp";
            send_event(self, e);
            break;
        }
        default: break;
        }
    }
    return set_flags;
}

// collide-shape-moving-angle-set!
void collide_shape_moving_angle_set(Target& self, const Vec& arg1, const Vec& arg2) {
    ControlInfo& c = self.control;
    c.surface_normal = arg1;
    c.surface_angle = vector_dot(arg1, c.dynam->gravity_normal);
    c.poly_angle = vector_dot(c.poly_normal, c.dynam->gravity_normal);
    Vec n{};
    vector_negate(n, arg2);
    c.touch_angle = std::fmax(c.touch_angle, vector_dot(arg1, vector_normalize(n, 1.0f)));
}

// find-ground (collide-shape-moving)
bool find_ground(Target& self, GoalCollideQuery& arg0, uint32_t arg1, float arg2, float arg3,
                 float arg4) {
    ControlInfo& c = self.control;
    c.gspot_pos = c.trans;
    arg0.start_pos = c.trans;
    vector_reset(arg0.move_dist);
    const float f0 = c.transv.y;
    if (f0 < 0.0f) arg2 = arg2 - std::fmax(-40960.0f, f0 * seconds_per_frame(self));
    arg0.start_pos.y += arg2;
    arg0.move_dist.y = -(arg2 + arg3);
    arg0.radius() = arg4;
    arg0.collide_with = arg1;
    arg0.ignore_process0 = 0x200u; // (-> this process)
    arg0.ignore_process1 = kGoalFalse;
    arg0.ignore_pat.bits = (1u << 27) | c.pat_ignore_mask.bits;
    arg0.action_mask = caction::solid;
    if (fill_and_probe_using_line_sphere(self.world, arg0) >= 0.0f) {
        c.gspot_pos.y = arg0.bot_intersect.y;
        c.gspot_normal = arg0.bot_normal;
        return true;
    }
    c.gspot_pos.y = -40959590.0f;
    c.gspot_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f}; // *y-vector*
    return false;
}

bool find_ground(Target& self, CollideQuery& q, uint32_t collide_with, float a2, float a3, float a4) {
    target_collide_init(self);
    g_collide_target = &self;
    GoalStackFrame f;
    GoalCollideQuery& g = new_stack_query(f);
    const bool r = find_ground(self, g, collide_with, a2, a3, a4);
    for (int i = 0; i < 3; i++) q.best_vertex[i] = g.bot_vertex[i];
    q.best_intersect = g.bot_intersect;
    q.best_normal = g.bot_normal;
    q.best_pat = g.bot_pat;
    q.start_pos = g.start_pos;
    q.move_dist = g.move_dist;
    q.radius = g.radius();
    q.collide_with = g.collide_with;
    q.ignore_pat = g.ignore_pat;
    q.action_mask = g.action_mask;
    q.bbox = g.bbox;
    return r;
}

bool find_ground(Target& self, PatSurface& best_pat, uint32_t collide_with, float above,
                 float below, float radius) {
    CollideQuery q;
    const bool r = find_ground(self, q, collide_with, above, below, radius);
    if (r) best_pat = q.best_pat;
    return r;
}

// find-ground-point
bool find_ground_point(Target& self, Vec& arg1, float arg2, float arg3) {
    ControlInfo& c = self.control;
    const Vec& v1 = c.transv;
    const float f30 = 819.2f < std::sqrt(v1.x * v1.x + v1.z * v1.z)
                          ? vector_y_angle(c.transv)
                          : quaternion_y_angle(c.quat_for_control);
    const Vec s2 = c.trans;
    GoalStackFrame f;
    GoalCollideQuery& s1 = new_stack_query(f);
    s1.collide_with = c.root_prim->prim_core.collide_with;
    s1.ignore_process0 = 0x200u;
    s1.ignore_process1 = kGoalFalse;
    s1.ignore_pat = c.pat_ignore_mask;
    s1.action_mask = caction::solid;
    arg1.w = 0.0f;
    for (int i = 0; i < 3; i++) {
        (&s1.bbox.min.x)[i] = (&s2.x)[i] - arg3;
        (&s1.bbox.max.x)[i] = (&s2.x)[i] + arg3;
    }
    s1.bbox.min.y = -40960.0f + s2.y;
    s1.bbox.max.y = 20480.0f + s2.y;
    fill_using_bounding_box(self.world, s1);
    vector_add(s1.start_pos, s2, Vec{0.0f, 20480.0f, 0.0f, 1.0f});
    s1.radius() = 2048.0f;
    s1.collide_with = c.root_prim->prim_core.collide_with;
    s1.ignore_process0 = 0x200u;
    s1.ignore_process1 = kGoalFalse;
    s1.ignore_pat = c.pat_ignore_mask;
    s1.action_mask = caction::solid;
    for (int s0 = 0; s0 < 8; s0++) {
        const float f28 = f30 + (!(s0 & 1) ? 8192.0f * static_cast<float>(s0 / 2)
                                           : -8192.0f * static_cast<float>(s0 / 2));
        int sv560 = 0;
        float f26 = arg3;
        s1.move_dist = Vec{0.0f, 0.0f, arg3, 1.0f};
        vector_rotate_y(s1.move_dist, s1.move_dist, f28);
        if (probe_using_line_sphere(s1) >= 0.0f) {
            f26 = -6144.0f + vector_vector_xz_distance(s2, s1.bot_intersect);
        }
        bool next_dir = false;
        for (float f24 = arg2; !next_dir && f26 >= f24; f24 = 4096.0f + f24) {
            s1.start_pos = Vec{0.0f, 0.0f, f24, 1.0f};
            vector_rotate_y(s1.start_pos, s1.start_pos, f28);
            vector_add(s1.start_pos, s2, s1.start_pos);
            s1.start_pos.y = 20480.0f + s2.y;
            s1.move_dist = Vec{0.0f, -61440.0f, 0.0f, 1.0f};
            s1.radius() = 10240.0f;
            s1.collide_with = c.root_prim->prim_core.collide_with;
            s1.ignore_process0 = 0x200u;
            s1.ignore_process1 = kGoalFalse;
            s1.ignore_pat = c.pat_ignore_mask;
            s1.action_mask = caction::solid;
            if (probe_using_line_sphere(s1) >= 0.0f) {
                const uint32_t mode = s1.bot_pat.mode();
                if ((mode == kPatGround || mode == kPatHalfpipe) && s1.bot_pat.event() == 0 &&
                    0.7f < s1.bot_normal.y) {
                    arg1 = s1.bot_intersect;
                    sv560++;
                    if (sv560 >= 2) return true;
                } else if (mode == kPatWall && 4096.0f + s2.y < s1.bot_intersect.y) {
                    next_dir = true; // (goto cfg-38)
                }
            }
        }
    }
    return false;
}

// target-attack-up
void target_attack_up(Target& self, const char* arg1, const char* arg2) {
    ControlInfo& c = self.control;
    CollideEvent test;
    test.message = arg1;
    test.has_attack = true;
    test.id = 2;
    test.mode = arg2;
    test.test = true;
    if (!send_event(self, test)) return;
    Vec found{};
    Vec s3 = find_ground_point(self, found, 8192.0f, 40960.0f) ? found : c.last_trans_on_ground;
    Vec s2;
    vector_sub(s2, s3, c.trans);
    const float f30 = std::fmax(8192.0f, std::fmin(40960.0f, std::sqrt(s2.x * s2.x + s2.z * s2.z)));
    Vec d;
    vector_sub(d, s3, c.trans);
    CollideEvent e;
    e.message = arg1;
    e.has_attack = true;
    e.id = 2;
    e.mode = arg2;
    e.angle = "up";
    e.has_vector = true;
    e.has_shove_up = true;
    if (std::fabs(vector_dot(c.dynam->gravity_normal, d)) < 40960.0f) {
        vector_xz_normalize(s2, f30);
        e.vector = s2;
        e.shove_up = lerp_scale(4096.0f, 16384.0f, f30, 4096.0f, 40960.0f) +
                     std::fmax(0.0f, s3.y - c.trans.y);
    } else {
        e.vector = Vec{0.0f, 40960.0f, 0.0f, 1.0f};
        e.shove_up = meters(10);
        e.has_control = true;
        e.control = 1.0f;
    }
    send_event(self, e);
}

// ---- What the movement core and the board read and write of the shape ----

CollideShapePrim* root_prim(Target& self) {
    target_collide_init(self);
    return self.control.root_prim;
}
CollideShapePrim* collision_sphere(Target& self, int i) {
    target_collide_init(self);
    return target_collision_sphere(i);
}
uint32_t& prim_action(CollideShapePrim* p) { return p->prim_core.action; }
uint32_t& prim_collide_as(CollideShapePrim* p) { return p->prim_core.collide_as; }
uint32_t& prim_collide_with(CollideShapePrim* p) { return p->prim_core.collide_with; }
Vec& prim_local_sphere(CollideShapePrim* p) { return p->local_sphere; }
const Vec& prim_world_sphere(CollideShapePrim* p) { return p->prim_core.world_sphere; }
int8_t& prim_transform_index(CollideShapePrim* p) { return p->transform_index; }

uint32_t& root_prim_action(Target& self) { return root_prim(self)->prim_core.action; }
uint32_t root_prim_collide_with(const Target& self) {
    target_collide_init(const_cast<Target&>(self));
    return self.control.root_prim->prim_core.collide_with;
}
Vec collision_sphere(const Target& self, int i) {
    target_collide_init(const_cast<Target&>(self));
    return target_collision_sphere(i)->prim_core.world_sphere;
}
uint32_t collision_sphere_collide_with(const Target& self, int i) {
    target_collide_init(const_cast<Target&>(self));
    return target_collision_sphere(i)->prim_core.collide_with;
}

// find-overlapping-shapes: overlap with other actors' shapes; the port has none, so it finds none.
void find_overlapping_shapes_target(Target& /*self*/) {}

} // namespace jak
