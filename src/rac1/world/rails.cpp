// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/world/rails.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "rac1/context.hpp"
#include "rac1/convert.hpp"

namespace rac1 {

namespace {

// The level's spline table (LevelSpline, 0x20 bytes each: bounds, path, kind) and its count, kept by
// each level's program. Their addresses are Veldin's (NTSC-U D_L00_0015F70C and D_L00_0015F710),
// each moved to where the level loaded now keeps it (a level may not keep the two side by side).
openrac::gaddr table_address() { return mod().relocate_level(0, ctx().pal ? 0x0015F7ECu : 0x0015F70Cu); }
openrac::gaddr count_address() { return mod().relocate_level(0, ctx().pal ? 0x0015F7F0u : 0x0015F710u); }

struct Cache {
    uint32_t loads = ~0u;
    std::vector<Rail> rails;
};
Cache g_cache;

void read_rails() {
    g_cache.rails.clear();
    const openrac::Memory mem = mod().memory();
    const uint32_t table = mem.read<uint32_t>(table_address());
    const int32_t count = mem.read<int32_t>(count_address());
    if (table == 0 || table >= 0x02000000u || count <= 0 || count > 512) return;
    for (int32_t i = 0; i < count; ++i) {
        const uint32_t entry = table + 0x20u * static_cast<uint32_t>(i);
        const uint32_t path = mem.read<uint32_t>(entry + 0x10);
        if (path == 0 || path >= 0x02000000u) continue;
        // A spline without a radius is one the hero never finds (nearest_rail_point skips it).
        if (!(mem.read<float>(entry + 12) > 0.0f)) continue;
        const int32_t n = mem.read<int32_t>(path);
        if (n < 2 || n > 4096) continue;
        Rail r;
        // A kind other than 0 is a loop: the game wraps its last point round to its first.
        r.loop = mem.read<int32_t>(entry + 0x14) != 0;
        r.points.reserve(static_cast<size_t>(n));
        for (int k = 0; k < 3; ++k) r.lo[k] = 1e30f, r.hi[k] = -1e30f;
        for (int32_t j = 0; j < n; ++j) {
            const uint32_t at = path + 0x10u + 0x10u * static_cast<uint32_t>(j);
            const jak::Vec p{mem.read<float>(at), mem.read<float>(at + 4), mem.read<float>(at + 8), 1.0f};
            r.points.push_back(p);
            const float c[3] = {p.x, p.y, p.z};
            for (int k = 0; k < 3; ++k) r.lo[k] = std::min(r.lo[k], c[k]), r.hi[k] = std::max(r.hi[k], c[k]);
        }
        g_cache.rails.push_back(std::move(r));
    }
}

} // namespace

const std::vector<Rail>& rails() {
    const uint32_t loads = mod().play_status().loads;
    if (loads != g_cache.loads) {
        g_cache.loads = loads;
        read_rails();
    }
    return g_cache.rails;
}

namespace {

// The bar's size (game units): about the game's rail, its top at the path (where the boots touch).
constexpr float kHalfWidth = 0.06f;
constexpr float kDepth = 0.12f;

// One triangle in Jak II's axes, wound so Jak II's (b - a) x (c - a) is `outward`.
void add_tri(jak::CollideTri& t, const float a[3], const float b[3], const float c[3], const float outward[3]) {
    jak::Vec va = to_jak_point(a), vb = to_jak_point(b), vc = to_jak_point(c);
    const jak::Vec out = to_jak_dir(outward);
    const jak::Vec e1{vb.x - va.x, vb.y - va.y, vb.z - va.z, 0.0f}, e2{vc.x - va.x, vc.y - va.y, vc.z - va.z, 0.0f};
    jak::Vec n{e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x, 0.0f};
    if (n.x * out.x + n.y * out.y + n.z * out.z < 0.0f) {
        std::swap(vb, vc);
        n = jak::Vec{-n.x, -n.y, -n.z, 0.0f};
    }
    const float l = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    t.vertex[0] = va;
    t.vertex[1] = vb;
    t.vertex[2] = vc;
    t.normal = l > 0.0f ? jak::Vec{n.x / l, n.y / l, n.z / l, 0.0f} : jak::Vec{0.0f, 1.0f, 0.0f, 0.0f};
    t.pat = jak::PatSurface{};
}

// The flat sideways direction at point i of a rail: the mean of its segments' (a mitre), so the
// bars of two segments meet in one edge.
void side_at(const Rail& r, size_t i, float out[2]) {
    const size_t n = r.points.size();
    float sx = 0.0f, sy = 0.0f;
    const auto seg = [&](size_t a, size_t b) {
        const float dx = r.points[b].x - r.points[a].x, dy = r.points[b].y - r.points[a].y;
        const float l = std::sqrt(dx * dx + dy * dy);
        if (l > 1e-5f) sx += -dy / l, sy += dx / l;
    };
    if (i > 0 || r.loop) seg((i + n - 1) % n, i);
    if (i + 1 < n || r.loop) seg(i, (i + 1) % n);
    const float l = std::sqrt(sx * sx + sy * sy);
    out[0] = l > 1e-5f ? sx / l : 1.0f;
    out[1] = l > 1e-5f ? sy / l : 0.0f;
}

} // namespace

int rail_triangles(const jak::Vec& lo, const jak::Vec& hi, jak::CollideTri* out, int max) {
    float glo[3], ghi[3];
    to_game_point(lo, glo);
    to_game_point(hi, ghi);
    for (int k = 0; k < 3; ++k) {
        if (glo[k] > ghi[k]) std::swap(glo[k], ghi[k]);
    }
    const float m = kHalfWidth + kDepth;
    int n = 0;
    for (const Rail& r : rails()) {
        bool apart = false;
        for (int k = 0; k < 3; ++k) apart = apart || r.lo[k] - m > ghi[k] || r.hi[k] + m < glo[k];
        if (apart) continue;
        const size_t count = r.points.size();
        for (size_t i = 0; i + 1 < count + (r.loop ? 1 : 0); ++i) {
            const jak::Vec& p = r.points[i];
            const jak::Vec& q = r.points[(i + 1) % count];
            if (std::min(p.x, q.x) - m > ghi[0] || std::max(p.x, q.x) + m < glo[0] ||
                std::min(p.y, q.y) - m > ghi[1] || std::max(p.y, q.y) + m < glo[1] ||
                std::min(p.z, q.z) - m > ghi[2] || std::max(p.z, q.z) + m < glo[2]) {
                continue;
            }
            if (n + 6 > max) return n;
            float sp[2], sq[2];
            side_at(r, i, sp);
            side_at(r, (i + 1) % count, sq);
            // The bar's corners: top left and right, bottom left and right, at each end.
            const float pl[3] = {p.x + sp[0] * kHalfWidth, p.y + sp[1] * kHalfWidth, p.z};
            const float pr[3] = {p.x - sp[0] * kHalfWidth, p.y - sp[1] * kHalfWidth, p.z};
            const float ql[3] = {q.x + sq[0] * kHalfWidth, q.y + sq[1] * kHalfWidth, q.z};
            const float qr[3] = {q.x - sq[0] * kHalfWidth, q.y - sq[1] * kHalfWidth, q.z};
            const float pbl[3] = {pl[0], pl[1], p.z - kDepth}, pbr[3] = {pr[0], pr[1], p.z - kDepth};
            const float qbl[3] = {ql[0], ql[1], q.z - kDepth}, qbr[3] = {qr[0], qr[1], q.z - kDepth};
            const float up[3] = {0.0f, 0.0f, 1.0f};
            const float left[3] = {sp[0], sp[1], 0.0f}, right[3] = {-sp[0], -sp[1], 0.0f};
            add_tri(out[n++], pl, ql, qr, up);
            add_tri(out[n++], pl, qr, pr, up);
            add_tri(out[n++], pl, pbl, qbl, left);
            add_tri(out[n++], pl, qbl, ql, left);
            add_tri(out[n++], pr, qr, qbr, right);
            add_tri(out[n++], pr, qbr, pbr, right);
        }
    }
    return n;
}

void trace_rails() {
    if (!settings().trace) return;
    const openrac::Memory mem = mod().memory();
    const uint32_t table = mem.read<uint32_t>(table_address());
    const int32_t count = mem.read<int32_t>(count_address());
    trace("splines at " + std::to_string(table) + " count " + std::to_string(count));
    for (const Rail& r : rails()) {
        const jak::Vec& a = r.points.front();
        const jak::Vec& b = r.points.back();
        trace("rail " + std::to_string(r.points.size()) + " from " + std::to_string(a.x) + "," + std::to_string(a.y) +
              "," + std::to_string(a.z) + " to " + std::to_string(b.x) + "," + std::to_string(b.y) + "," +
              std::to_string(b.z) + (r.loop ? " loop" : ""));
    }
}

} // namespace rac1
