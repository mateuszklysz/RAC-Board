// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The collision code's maths, from OpenGOAL's goal_src/jak2/engine (jak-project, ISC License,
// Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). The vector-unit routines are written lane
// by lane in the order OpenGOAL evaluates them; `x * 1.0` products the VU does on the way (vf0.w)
// are exact and left out.
#include "jak/math/collide.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "jak/math/math.hpp"

namespace jak {

namespace {

// The outer product as vopmula / vopmsub (or .outer.product.a / .b) compute it: a x b.
Vec cross_vu(const Vec& a, const Vec& b) {
    Vec r;
    r.x = a.y * b.z - b.y * a.z;
    r.y = a.z * b.x - b.z * a.x;
    r.z = a.x * b.y - b.x * a.y;
    r.w = 0.0f;
    return r;
}

// The sign bit of a float, as GOAL tests a lane moved into an integer register.
bool neg(float f) {
    uint32_t u;
    std::memcpy(&u, &f, 4);
    return (u >> 31) != 0;
}

} // namespace

// vector-average! (vector.gc): acc = a * 0.5, xyz = acc + b * 0.5, w 1.
Vec& vector_average(Vec& out, const Vec& a, const Vec& b) {
    Vec r;
    r.x = a.x * 0.5f + b.x * 0.5f;
    r.y = a.y * 0.5f + b.y * 0.5f;
    r.z = a.z * 0.5f + b.z * 0.5f;
    r.w = 1.0f;
    out = r;
    return out;
}

// transform-vectors! (matrix.gc): acc = row3, += row0 x, += row1 y, xyz = acc + row2 z.
void transform_vectors(const Mat& m, Vec* dst, const Vec* src, int count) {
    for (int i = 0; i < count; i++) {
        const Vec v = src[i];
        Vec r;
        for (int l = 0; l < 3; l++) {
            float acc = m.v[3][l];
            acc += m.v[0][l] * v.x;
            acc += m.v[1][l] * v.y;
            r[l] = acc + m.v[2][l] * v.z;
        }
        r.w = v.w;
        dst[i] = r;
    }
}

// vector-reflect-flat-above! (geometry.gc): the flattened vector, pushed out along n by up to
// 4 m, 16 times how much its length's 2% exceeds its height above the plane.
Vec& vector_reflect_flat_above(Vec& out, const Vec& in, const Vec& n) {
    const Vec t = cross_vu(in, n);
    Vec r = cross_vu(n, t);
    r.w = 1.0f;
    out = r;
    const float len = vector_length(out);
    const float d = vector_dot(out, n);
    const float k = 0.02f * len - d;
    vector_add_float_mul(out, out, n, std::fmin(16384.0f, 16.0f * k));
    return out;
}

// vector-reflect-flat-gravity! (geometry.gc).
Vec& vector_reflect_flat_gravity(Vec& out, const Vec& in, const Vec& n, const Vec& grav) {
    const Vec src = in;
    vector_reflect_flat(out, src, n);
    Vec srcn;
    vector_normalize_copy(srcn, src, 1.0f);
    const float src_len = vector_length(src);
    const float dst_len = vector_length(out);
    const float sdot = vector_dot(srcn, n);
    const float gdot = vector_dot(grav, n);
    if (0.0001f < gdot && 8192.0f < src_len) {
        const float k = -(sdot / gdot);
        vector_add_float_mul(out, src, grav, src_len * k);
        vector_add(out, out, n);
        vector_normalize(out, dst_len);
    }
    return out;
}

// vector-segment-distance-point! (geometry.gc).
float vector_segment_distance_point(const Vec& p, const Vec& a, const Vec& b, Vec* out) {
    Vec d{b.x - a.x, b.y - a.y, b.z - a.z, b.w - a.w};
    const Vec ap{p.x - a.x, p.y - a.y, p.z - a.z, p.w - a.w};
    float s = d.x * d.x;
    s += d.y * d.y;
    s = s + d.z * d.z;
    const float len = std::sqrt(std::fabs(s));
    const float q = 1.0f / len;
    for (int i = 0; i < 4; i++) d[i] = d[i] * q;
    float t = d.x * ap.x + d.y * ap.y;
    t = t + d.z * ap.z;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (len < t) {
        t = len;
    }
    for (int i = 0; i < 4; i++) d[i] = d[i] * t;
    if (out) {
        *out = Vec{a.x + d.x, a.y + d.y, a.z + d.z, 1.0f};
    }
    const Vec e{ap.x - d.x, ap.y - d.y, ap.z - d.z, 0.0f};
    float s2 = e.x * e.x;
    s2 += e.y * e.y;
    s2 = s2 + e.z * e.z;
    return std::sqrt(std::fabs(s2));
}

// vector-line-distance-point! (geometry.gc).
float vector_line_distance_point(const Vec& p, const Vec& a, const Vec& b, Vec* out) {
    Vec dir;
    vector_sub(dir, b, a);
    vector_normalize(dir, 1.0f);
    Vec ap;
    vector_sub(ap, p, a);
    const float t = vector_dot(dir, ap);
    Vec along;
    vector_float_mul(along, dir, t);
    if (out) {
        vector_add(*out, a, along);
    }
    Vec perp;
    vector_sub(perp, ap, along);
    return vector_length(perp);
}

// closest-pt-in-triangle (geometry.gc): which edges p lies outside of, from the signs of three
// triple products, picks the plane projection or the nearest edge(s).
void closest_pt_in_triangle(Vec& out, const Vec& p, const Vec* tri, const Vec& n) {
    const Vec& v0 = tri[0];
    const Vec& v1 = tri[1];
    const Vec& v2 = tri[2];
    const Vec e6{v1.x - v2.x, v1.y - v2.y, v1.z - v2.z, 0.0f};
    const Vec e7{v1.x - p.x, v1.y - p.y, v1.z - p.z, 0.0f};
    const Vec e8{v1.x - v0.x, v1.y - v0.y, v1.z - v0.z, 0.0f};
    const Vec e9{p.x - v2.x, p.y - v2.y, p.z - v2.z, 0.0f};
    const Vec e10{p.x - v0.x, p.y - v0.y, p.z - v0.z, 0.0f};
    const Vec c14 = cross_vu(e7, e8);
    const Vec c15 = cross_vu(e6, e7);
    const Vec c16 = cross_vu(e9, e10);
    // (.mul.vf) then y += x, y += z: the dot products in the y lanes.
    const float d11 = (c14.y * n.y + c14.x * n.x) + c14.z * n.z;
    const float d12 = (c15.y * n.y + c15.x * n.x) + c15.z * n.z;
    const float d13 = (c16.y * n.y + c16.x * n.x) + c16.z * n.z;
    const int code = (neg(d11) ? 1 : 0) | (neg(d12) ? 2 : 0) | (neg(d13) ? 4 : 0);
    Vec tmp;
    switch (code) {
    case 0: {
        const Vec e17{p.x - v0.x, p.y - v0.y, p.z - v0.z, 0.0f};
        const Vec c = cross_vu(e17, n);
        Vec r = cross_vu(n, c);
        r.x = r.x + v0.x;
        r.y = r.y + v0.y;
        r.z = r.z + v0.z;
        r.w = 1.0f;
        out = r;
        break;
    }
    case 1:
        vector_segment_distance_point(p, v0, v1, &out);
        break;
    case 2:
        vector_segment_distance_point(p, v1, v2, &out);
        break;
    case 3: {
        const float d = vector_segment_distance_point(p, v1, v0, &out);
        if (vector_segment_distance_point(p, v1, v2, &tmp) < d) out = tmp;
        break;
    }
    case 4:
        vector_segment_distance_point(p, v2, v0, &out);
        break;
    case 5: {
        const float d = vector_segment_distance_point(p, v0, v1, &out);
        if (vector_segment_distance_point(p, v0, v2, &tmp) < d) out = tmp;
        break;
    }
    default: {
        const float d = vector_segment_distance_point(p, v2, v0, &out);
        if (vector_segment_distance_point(p, v2, v1, &tmp) < d) out = tmp;
        break;
    }
    }
}

// set-from-point-offset-pad! (bounding-box.gc).
void set_from_point_offset_pad(BoundingBox& box, const Vec& p, const Vec& offset, float pad) {
    const Vec e{p.x + offset.x, p.y + offset.y, p.z + offset.z, p.w + offset.w};
    Vec mn, mx;
    for (int i = 0; i < 3; i++) {
        mn[i] = std::fmin(p[i], e[i]) - pad;
        mx[i] = std::fmax(p[i], e[i]) + pad;
    }
    mn.w = 1.0f;
    mx.w = 1.0f;
    box.min = mn;
    box.max = mx;
}

// set-from-spheres! (bounding-box.gc): with no spheres, both corners (0, 0, 0, 1).
void set_from_spheres(BoundingBox& box, const Vec* spheres, int count) {
    Vec mn{0.0f, 0.0f, 0.0f, 1.0f};
    Vec mx{0.0f, 0.0f, 0.0f, 1.0f};
    for (int i = 0; i < count; i++) {
        const Vec& s = spheres[i];
        Vec smin = s, smax = s;
        for (int l = 0; l < 3; l++) {
            smin[l] = s[l] - s.w;
            smax[l] = s[l] + s.w;
        }
        if (i == 0) {
            // (set! current-min sph-min): the whole register, w the sphere's radius.
            mn = smin;
            mx = smax;
        } else {
            for (int l = 0; l < 3; l++) {
                mn[l] = std::min(mn[l], smin[l]);
                mx[l] = std::max(mx[l], smax[l]);
            }
        }
    }
    box.min = mn;
    box.max = mx;
}

// pc-port-raw-ray-sphere-implementation (collide-func.gc). The sign tests read a lane through a
// 64-bit integer move, so they test the y lane, where the sums are kept.
float raw_ray_sphere_intersect(float rad, const Vec& o, const Vec& d) {
    const float r2 = rad * rad;
    const float dd = (d.x * d.x + d.y * d.y) + d.z * d.z;   // vf4.x
    const float oo = (o.y * o.y + o.x * o.x) + (o.z * o.z - r2); // vf6.y: |o|^2 - r^2
    const float od = (d.y * o.y + d.x * o.x) + d.z * o.z;   // vf5.y
    const float q = 1.0f / dd;
    const float c = oo * dd;                                 // vf7.y
    const float disc = od * od - c;                          // vf9.y
    if (neg(oo)) {
        return 0.0f; // inside the sphere
    }
    if (dd == 0.0f) {
        return kCollisionMiss;
    }
    const float sq = std::sqrt(std::fabs(disc));
    if (!neg(od)) {
        return kCollisionMiss; // moving away
    }
    if (neg(disc)) {
        return kCollisionMiss;
    }
    const float e = od + dd;          // vf6.y
    const float f = disc - e * e;     // vf6.y
    const float u = (od + sq) * q;    // vf9.y
    if (neg(e) && neg(f)) {
        return kCollisionMiss; // the sphere is beyond the probe's end
    }
    return 0.0f - u;
}

// ray-sphere-intersect (collide-func.gc).
float ray_sphere_intersect(const Vec& origin, const Vec& dir, const Vec& center, float radius) {
    const Vec o{origin.x - center.x, origin.y - center.y, origin.z - center.z,
                origin.w - center.w};
    return raw_ray_sphere_intersect(radius, o, dir);
}

// ray-cylinder-intersect (collide-func.gc). pt_out is written when the hit is past the far cap
// too: the store sits in the delay slot of that last test.
float ray_cylinder_intersect(const Vec& origin, const Vec& dir, const Vec& cyl_origin,
                             const Vec& axis, float cyl_rad, float cyl_len, Vec& pt_out) {
    const Vec o{origin.x - cyl_origin.x, origin.y - cyl_origin.y, origin.z - cyl_origin.z,
                origin.w - cyl_origin.w};
    const float oa = (o.y * axis.y + o.x * axis.x) + o.z * axis.z;       // vf16.y
    const float da = (dir.y * axis.y + dir.x * axis.x) + dir.z * axis.z; // vf17.y
    const float end = da + oa;                                           // vf18.y
    const float o_top = oa - cyl_len;                                    // vf19.y
    const float end_top = end - cyl_len;                                 // vf20.y
    Vec op, dp;
    for (int i = 0; i < 4; i++) {
        op[i] = o[i] - axis[i] * oa;
        dp[i] = dir[i] - axis[i] * da;
    }
    if (neg(oa) && neg(end)) {
        return kCollisionMiss;
    }
    if (!neg(o_top) && !neg(end_top)) {
        return kCollisionMiss;
    }
    const float u = raw_ray_sphere_intersect(cyl_rad, op, dp);
    if (neg(u)) {
        return kCollisionMiss;
    }
    const float proj = oa + da * u;
    Vec p = cyl_origin;
    p.x = p.x + axis.x * proj;
    p.y = p.y + axis.y * proj;
    p.z = p.z + axis.z * proj;
    if (neg(proj)) {
        return kCollisionMiss;
    }
    pt_out = p;
    if (!neg(proj - cyl_len)) {
        return kCollisionMiss;
    }
    return u;
}

} // namespace jak
