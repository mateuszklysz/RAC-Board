// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The collision part (src/jak/collide*) on a world of its own: a floor, a ramp, a wall and a thin
// rail box. The target falls onto the floor, slides along the wall, a line probe hits the floor
// where it should, and the board's edge spec finds the rail's edge.
#include <cstring>
#include <vector>

#include "check.hpp"
#include "jak/collide/collide.hpp"
#include "jak/collide/mem.hpp"
#include "jak/host.hpp"
#include "jak/math/math.hpp"
#include "jak/surface.hpp"
#include "jak/target.hpp"

using namespace jak;

namespace {

// Every triangle whose box touches the query's.
class TestWorld : public World {
  public:
    std::vector<CollideTri> tris;
    void quad(Vec a, Vec b, Vec c, Vec d, Vec n, uint32_t pat = 0) {
        tris.push_back(CollideTri{{a, b, c}, n, PatSurface{pat}});
        tris.push_back(CollideTri{{a, c, d}, n, PatSurface{pat}});
    }
    int triangles(const Vec& lo, const Vec& hi, CollideTri* out, int max) override {
        int n = 0;
        for (const CollideTri& t : tris) {
            bool miss = false;
            for (int k = 0; k < 3; k++) {
                float mn = t.vertex[0][k], mx = mn;
                for (int v = 1; v < 3; v++) {
                    mn = std::fmin(mn, t.vertex[v][k]);
                    mx = std::fmax(mx, t.vertex[v][k]);
                }
                if (mn > hi[k] || mx < lo[k]) miss = true;
            }
            if (!miss && n < max) out[n++] = t;
        }
        return n;
    }
};

Vec P(float x, float y, float z) { return Vec{x, y, z, 1.0f}; }

// Counter-clockwise seen from the normal's side (moving-sphere-triangle-intersect takes the
// normal from the winding).
void floor_quad(TestWorld& w, float x0, float x1, float z0, float z1, float y) {
    w.quad(P(x0, y, z0), P(x0, y, z1), P(x1, y, z1), P(x1, y, z0), Vec{0, 1, 0, 0});
}

Dynamics g_dynam;
Surface g_mod;

void make(Target& t, TestWorld& w, const Vec& at) {
    t.world = &w;
    g_dynam.gravity_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
    g_dynam.gravity = Vec{0.0f, -245760.0f, 0.0f, 1.0f};
    t.control.dynam = &g_dynam;
    g_mod = Surface{};
    t.control.mod_surface = &g_mod;
    t.control.current_surface = &g_mod;
    t.control.surf = &stone_surface;
    t.control.cpad = &t.pad;
    target_collide_init(t);
    // target-collide-set! 'normal's spheres (the movement core's; set here to keep this test alone)
    const float br = 2867.2f;
    prim_local_sphere(root_prim(t)) = Vec{0.0f, 4915.2f, 0.0f, 9011.2f};
    prim_local_sphere(collision_sphere(t, 0)) = Vec{0.0f, br, 0.0f, br};
    prim_local_sphere(collision_sphere(t, 1)) = Vec{0.0f, 2867.2f + br, 0.0f, br};
    prim_local_sphere(collision_sphere(t, 2)) = Vec{0.0f, 5734.4f + br, 0.0f, br};
    t.control.trans = Vec{};
    move_to_point(t, at);
    update_transforms(t);
}

int g_edge_events = 0;
bool on_event(Target&, const CollideEvent& e) {
    if (e.message && std::strcmp(e.message, "edge-grab") == 0) g_edge_events++;
    return false;
}

} // namespace

int main() {
    TestWorld w;
    floor_quad(w, -81920.0f, 81920.0f, -81920.0f, 81920.0f, 0.0f);
    // a wall at x = 20480 facing -x
    w.quad(P(20480, 0, 81920), P(20480, 40960, 81920), P(20480, 40960, -81920), P(20480, 0, -81920),
           Vec{-1, 0, 0, 0});
    // a ramp rising toward -x past x = -40960
    w.quad(P(-81920, 20480, -20480), P(-81920, 20480, 20480), P(-40960, 0, 20480),
           P(-40960, 0, -20480), Vec{0.4472136f, 0.8944272f, 0, 0});

    // ---- a line probe straight down hits the floor half way ----
    {
        Target t;
        make(t, w, P(0, 0, 0));
        CollideQuery q;
        q.start_pos = P(1000.0f, 8192.0f, 1000.0f);
        q.move_dist = Vec{0.0f, -16384.0f, 0.0f, 1.0f};
        q.radius = 0.0f;
        q.collide_with = cspec::backgnd;
        q.ignore_pat = PatSurface{};
        const float u = fill_and_probe_using_line_sphere(t, q);
        CHECK_NEAR(u, 0.5, 1e-5);
        CHECK_NEAR(q.best_intersect.y, 0.0, 1e-3);
        CHECK_NEAR(q.best_intersect.x, 1000.0, 1e-3);
        CHECK_NEAR(q.best_normal.y, 1.0, 1e-5);
        // and a probe that ends above the floor misses
        q.move_dist = Vec{0.0f, -4096.0f, 0.0f, 1.0f};
        CHECK(fill_and_probe_using_line_sphere(t, q) < 0.0f);
        // find-ground under the target
        CHECK(find_ground(t, q, cspec::backgnd, 8192.0f, 81920.0f, 1024.0f));
        CHECK_NEAR(t.control.gspot_pos.y, 0.0, 1e-2);
    }

    // ---- a falling target stops on the floor ----
    {
        Target t;
        make(t, w, P(0, 8192, 0));
        for (int i = 0; i < 60; i++) {
            t.control.transv = Vec{0.0f, -40960.0f, 0.0f, 1.0f};
            target_integrate_and_collide(t, t.control.transv, 4096.0f);
        }
        CHECK(t.control.status & cstat::on_surface);
        CHECK(t.control.status & cstat::on_ground);
        CHECK_NEAR(t.control.trans.y, 0.0, 16.0);
        CHECK(t.control.transv.y >= -1.0f);
        CHECK_NEAR(t.control.ground_poly_normal.y, 1.0, 1e-4);
    }

    // ---- sliding along the wall ----
    {
        Target t;
        make(t, w, P(16384, 10, 0));
        bool wall = false;
        for (int i = 0; i < 30; i++) {
            t.control.transv = Vec{40960.0f, -4096.0f, 40960.0f, 1.0f};
            target_integrate_and_collide(t, t.control.transv, 4096.0f);
            if (t.control.status & cstat::touch_wall) wall = true;
        }
        CHECK(wall);
        CHECK_NEAR(t.control.wall_contact_normal.x, -1.0, 1e-3);
        CHECK(t.control.trans.x < 20480.0f - 2867.2f + 8.0f);
        CHECK(t.control.trans.z > 16384.0f); // kept sliding along it (30 frames at 10 m/s)
    }

    // ---- the board's spec finds a rail's edge ----
    {
        TestWorld r;
        floor_quad(r, -81920.0f, 81920.0f, -81920.0f, 81920.0f, 0.0f);
        // a thin box along z: top at y = 8192 (the hanging spheres clear the floor), 400 units wide
        const float x0 = -200.0f, x1 = 200.0f, y = 8192.0f, z0 = -40960.0f, z1 = 40960.0f;
        floor_quad(r, x0, x1, z0, z1, y);
        r.quad(P(x1, 0, z0), P(x1, y, z0), P(x1, y, z1), P(x1, 0, z1), Vec{1, 0, 0, 0});
        r.quad(P(x0, 0, z1), P(x0, y, z1), P(x0, y, z0), P(x0, 0, z0), Vec{-1, 0, 0, 0});
        Target t;
        make(t, r, P(0, 8192, 0));
        t.collide_event = on_event;
        t.focus_status |= 1ull << 18; // board
        t.control.midpoint_of_hands = P(0, 8192, 0);
        CollideEdgeSpec& spec = collide_edge_board_spec();
        edge_spec_set_touching_segment(spec, nullptr);
        do_edge_grabs(t, spec);
        const EdgeGrabInfo& info = edge_grab_info();
        CHECK(info.found_edge);
        CHECK(g_edge_events == 1);
        CHECK_NEAR(info.world_vertex[0].y, 8192.0, 1e-3);
        CHECK_NEAR(std::fabs(info.world_vertex[0].x), 200.0, 1e-3);
        CHECK_NEAR(std::fabs(info.hanging_matrix_row0.x), 1.0, 1e-3);
        if (!info.found_edge) std::printf("no edge: %u edges\n", collide_edge_work().num_edges);
    }

    TEST_MAIN_END();
}
