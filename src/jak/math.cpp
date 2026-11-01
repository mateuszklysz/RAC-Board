// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's maths, from OpenGOAL's goal_src/jak2/engine/math and geometry (jak-project, ISC License,
// Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). Each function says which GOAL one it is;
// the vector-unit routines are written lane by lane in the order OpenGOAL evaluates them.
#include "jak/math.hpp"

#include <cmath>
#include <cstring>

namespace jak {

namespace {

float bits(uint32_t u) {
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}

// (the int f): truncation toward zero, as cvt.w.s and OpenGOAL's cvttss2si.
int64_t to_int(float f) { return static_cast<int64_t>(static_cast<int32_t>(f)); }

// sar(shl(x, 48), 48): the low 16 bits as a signed number.
int64_t wrap16(int64_t x) { return static_cast<int16_t>(static_cast<uint16_t>(x & 0xFFFF)); }

const float kRotToRad = bits(0x38c90fdau);

// sincos-rad-asm (trigonometry.gc), with the cosine's first coefficient as the game has it
// (FIX_COSINE_BUG off).
void sincos_rad(Vec& out, float x) {
    const float f1 = x;
    const float f10 = bits(0x3F7FFFDEu), f11 = bits(0xBE2AA8F5u), f12 = bits(0x3C086BF6u),
                f14 = bits(0xB94D2072u), f15 = bits(0x361AA27Fu), f16 = 1.0f,
                f17 = bits(0xBEFF0000u), f18 = bits(0x3D2A7A28u), f19 = bits(0xBAB2BC31u),
                f20 = bits(0x37A933EBu);
    const float f2 = f1 * f1;
    float acc = f1 * f10;
    const float f3 = f2 * f1, f4 = f2 * f2;
    const float f5 = f3 * f2, f6 = f3 * f3, f7 = f4 * f3, f8 = f4 * f4, f9 = f5 * f4;
    acc = acc + f3 * f11;
    acc = acc + f5 * f12;
    acc = acc + f7 * f14;
    const float f21 = acc + f9 * f15;
    acc = f16 * f16;
    acc = acc + f2 * f17;
    acc = acc + f4 * f18;
    acc = acc + f6 * f19;
    const float f22 = acc + f8 * f20;
    out.x = f21;
    out.y = f22;
}

// sincos! (trigonometry.gc): rotation units, unwrapped.
void sincos(Vec& out, float rot) { sincos_rad(out, kRotToRad * unwrap_angle(rot)); }

// atan-series-rad (trigonometry.gc).
float atan_series_rad(float a0) {
    const float f1 = a0 * a0;
    const float f10 = bits(0x3f7ffff5u), f11 = bits(0xbeaaa61cu), f12 = bits(0x3e4c40a6u),
                f13 = bits(0xbe0e6c63u), f14 = bits(0x3dc577dfu), f15 = bits(0xbd6501c4u),
                f16 = bits(0x3cb31652u), f17 = bits(0xbb84d7e7u), f18 = bits(0x3f490fdbu);
    const float f2 = a0 * f1, f3 = f1 * f1;
    const float f4 = f2 * f1, f5 = f3 * f2, f6 = f4 * f3, f7 = f5 * f3, f8 = f6 * f3,
                f9 = f7 * f3;
    const float f0 = a0 * f10;
    float acc = f0 + f18;
    acc = acc + f2 * f11;
    acc = acc + f4 * f12;
    acc = acc + f5 * f13;
    acc = acc + f6 * f14;
    acc = acc + f7 * f15;
    acc = acc + f8 * f16;
    return acc + f9 * f17;
}

// vector-sin-rad! (trigonometry.gc), the four lanes.
void vector_sin_rad(Vec& out, const Vec& in) {
    const float c2 = 0.999998f;
    const float k[4] = {-0.16666014f, 0.008326521f, -0.0001956241f, 0.0000023042373f};
    for (int i = 0; i < 4; i++) {
        const float v1 = in[i], v3 = v1 * v1, v4 = v3 * v1, v5 = v3 * v3, v6 = v4 * v3,
                    v7 = v5 * v4, v8 = v6 * v5;
        float acc = v1 * c2;
        acc = acc + v4 * k[0];
        acc = acc + v6 * k[1];
        acc = acc + v7 * k[2];
        out[i] = acc + v8 * k[3];
    }
}

} // namespace

// ---- Scalars ----

// lerp-scale (math.gc).
float lerp_scale(float out0, float out1, float in, float in0, float in1) {
    const float t = std::fmin(std::fmax(goal_div(in - in0, in1 - in0), 0.0f), 1.0f);
    return (1.0f - t) * out0 + t * out1;
}

// lerp (math.gc).
float lerp(float a, float b, float t) { return a + t * (b - a); }

// seek (math.gc).
float seek(float x, float target, float max_step) {
    const float d = target - x;
    if (max_step >= std::fabs(d)) {
        return target;
    }
    return d >= 0.0f ? x + max_step : x - max_step;
}

// sign (math.gc).
float fsign(float x) { return 0.0f < x ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }

// unwrap-angle (trigonometry.gc).
float unwrap_angle(float a) { return static_cast<float>(wrap16(to_int(a))); }

// sin (trigonometry.gc).
float sin(float rot) {
    const float f2 = 0.000095873795f * unwrap_angle(rot);
    const float f1_4 = 0.999998f * f2;
    const float f0_3 = f2 * f2;
    const float f2_1 = f2 * f0_3;
    const float f1_5 = f1_4 + -0.16666014f * f2_1;
    const float f2_2 = f2_1 * f0_3;
    const float f1_6 = f1_5 + 0.008326521f * f2_2;
    const float f2_3 = f2_2 * f0_3;
    const float f1_7 = f1_6 + -0.0001956241f * f2_3;
    const float f0_4 = f2_3 * f0_3;
    return f1_7 + 0.0000023042373f * f0_4;
}

// cos (trigonometry.gc).
float cos(float rot) { return sin(16384.0f + rot); }

// atan0 (trigonometry.gc).
float atan0(float y, float x) {
    float f1 = goal_div(y - x, y + x);
    const float f19 = bits(0x46000000u), f11 = bits(0x4622f97cu), f12 = bits(0xc55946e1u),
                f13 = bits(0x450207fdu), f14 = bits(0xc4b556ceu), f15 = bits(0x447b6ca4u),
                f16 = bits(0xc411ca52u), f17 = bits(0x43640558u), f18 = bits(0xc2292434u);
    const float f2 = f1 * f1;
    const float f3 = f1 * f2;
    f1 = f1 * f11;
    const float f4 = f2 * f2, f5 = f3 * f2, f6 = f4 * f3, f7 = f5 * f4, f8 = f6 * f4,
                f9 = f7 * f4, f10 = f8 * f4;
    float acc = f1 + f19;
    acc = acc + f3 * f12;
    acc = acc + f5 * f13;
    acc = acc + f6 * f14;
    acc = acc + f7 * f15;
    acc = acc + f8 * f16;
    acc = acc + f9 * f17;
    return acc + f10 * f18;
}

// atan (trigonometry.gc).
float atan(float y, float x) {
    if (x == 0.0f && y == 0.0f) {
        return 0.0f;
    }
    if (x < 0.0f && y < 0.0f) {
        return -32768.0f + atan0(-y, -x);
    }
    if (y < 0.0f) {
        return -atan0(-y, x);
    }
    if (x < 0.0f) {
        return 32768.0f - atan0(y, -x);
    }
    return atan0(y, x);
}

// deg- (trigonometry.gc).
float deg_minus(float a, float b) {
    return static_cast<float>(wrap16(wrap16(to_int(a)) - wrap16(to_int(b))));
}

// deg-diff (trigonometry.gc).
float deg_diff(float a, float b) { return deg_minus(b, a); }

// deg-seek (trigonometry.gc), on the 16-bit integers it shifts into the top of a register.
float deg_seek(float in, float target, float max_diff) {
    const int64_t v1 = static_cast<int64_t>(static_cast<uint64_t>(to_int(in)) << 48);
    const int64_t a0 = static_cast<int64_t>(static_cast<uint64_t>(to_int(target)) << 48);
    const int64_t a1 = static_cast<int64_t>(static_cast<uint64_t>(to_int(max_diff)) << 48);
    const int64_t a2 = static_cast<int64_t>(static_cast<uint64_t>(a0) - static_cast<uint64_t>(v1));
    const int64_t a3 = a2 < 0 ? static_cast<int64_t>(0 - static_cast<uint64_t>(a2)) : a2;
    int64_t r;
    if (a3 < 0) {
        r = static_cast<int64_t>(static_cast<uint64_t>(v1) + static_cast<uint64_t>(a1));
    } else if (a1 >= a3) {
        r = a0;
    } else if (a2 >= 0) {
        r = static_cast<int64_t>(static_cast<uint64_t>(v1) + static_cast<uint64_t>(a1));
    } else {
        r = static_cast<int64_t>(static_cast<uint64_t>(v1) - static_cast<uint64_t>(a1));
    }
    return static_cast<float>(r >> 48);
}

// deg-seek-smooth (trigonometry.gc).
float deg_seek_smooth(float in, float target, float max_diff, float alpha) {
    float d = deg_minus(target, in) * alpha;
    if (max_diff < std::fabs(d)) {
        d = d >= 0.0f ? max_diff : -max_diff;
    }
    return in + d;
}

// ---- Vectors ----

Vec& vector_reset(Vec& v) {
    v = Vec{};
    return v;
}

// vector+! (vector-h.gc): w set to vf0's w, 1.0.
Vec& vector_add(Vec& out, const Vec& a, const Vec& b) {
    out = Vec{a.x + b.x, a.y + b.y, a.z + b.z, 1.0f};
    return out;
}

// vector-! (vector-h.gc): w 1.0.
Vec& vector_sub(Vec& out, const Vec& a, const Vec& b) {
    out = Vec{a.x - b.x, a.y - b.y, a.z - b.z, 1.0f};
    return out;
}

// vector-float*! (vector.gc): w 1.0.
Vec& vector_float_mul(Vec& out, const Vec& a, float s) {
    out = Vec{a.x * s, a.y * s, a.z * s, 1.0f};
    return out;
}

// vector+float*! (vector.gc): xyz a + b * s, w 1.0.
Vec& vector_add_float_mul(Vec& out, const Vec& a, const Vec& b, float s) {
    out = Vec{b.x * s + a.x, b.y * s + a.y, b.z * s + a.z, 1.0f};
    return out;
}

// vector-negate! (vector.gc): xyz negated, w kept.
Vec& vector_negate(Vec& out, const Vec& a) {
    out = Vec{-a.x, -a.y, -a.z, a.w};
    return out;
}

// vector-dot (vector-h.gc).
float vector_dot(const Vec& a, const Vec& b) {
    float r = 0.0f;
    r += a.x * b.x;
    r += a.y * b.y;
    r += a.z * b.z;
    return r;
}

// vector-cross! (vector.gc): outer product; w of the result is a.w*b.w - b.w*a.w = 0 lane junk,
// here 0.
Vec& vector_cross(Vec& out, const Vec& a, const Vec& b) {
    out = Vec{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x, 0.0f};
    return out;
}

// vector-length (vector.gc).
float vector_length(const Vec& v) {
    const float s = (v.x * v.x + v.y * v.y) + v.z * v.z;
    return std::sqrt(s);
}

// vector-xz-length (vector.gc).
float vector_xz_length(const Vec& v) { return std::sqrt(v.x * v.x + v.z * v.z); }

// vector-vector-distance (vector.gc).
float vector_vector_distance(const Vec& a, const Vec& b) {
    Vec d{b.x - a.x, b.y - a.y, b.z - a.z, 0.0f};
    return vector_length(d);
}

// vector-normalize! (vector.gc): xyz to length len, w kept.
Vec& vector_normalize(Vec& v, float len) {
    const float k = goal_div(len, vector_length(v));
    v.x *= k;
    v.y *= k;
    v.z *= k;
    return v;
}

// vector-normalize-copy! (vector.gc): w 1.0; a zero vector is copied as it is.
Vec& vector_normalize_copy(Vec& out, const Vec& in, float len) {
    const float l = vector_length(in);
    if (l == 0.0f) {
        out = in;
    } else {
        const float k = len / l;
        out = Vec{in.x * k, in.y * k, in.z * k, out.w};
    }
    out.w = 1.0f;
    return out;
}

// vector-xz-normalize! (vector.gc).
Vec& vector_xz_normalize(Vec& v, float len) {
    const float l = std::sqrt(v.x * v.x + v.z * v.z);
    if (l != 0.0f) {
        const float k = len / l;
        v.x *= k;
        v.z *= k;
    }
    return v;
}

// vector-flatten! (geometry.gc): n x (in x n) by two outer products; w 1.0.
Vec& vector_flatten(Vec& out, const Vec& in, const Vec& n) {
    Vec t{n.y * in.z - n.z * in.y, n.z * in.x - n.x * in.z, n.x * in.y - n.y * in.x, 1.0f};
    // (in x n) is -(n x in); outer.product(vf1=in, vf2=n) = in x n.
    t = Vec{-t.x, -t.y, -t.z, 1.0f};
    Vec r{n.y * t.z - n.z * t.y, n.z * t.x - n.x * t.z, n.x * t.y - n.y * t.x, 1.0f};
    out = r;
    return out;
}

// vector-reflect! (geometry.gc): in - 2 * (in - flatten(in)), as in - 2 (in.n) n computed through
// the flattened vector: acc = 2 flat; out = in * 1 - acc... reproduced as OpenGOAL's lanes.
Vec& vector_reflect(Vec& out, const Vec& in, const Vec& n) {
    Vec flat;
    vector_flatten(flat, in, n);
    // .add.vf.xyz acc vf3 vf3 ; .sub.mul.w.vf.xyz vf3 vf1 vf0 acc  ->  acc - in * 1.0
    out = Vec{(flat.x + flat.x) - in.x, (flat.y + flat.y) - in.y, (flat.z + flat.z) - in.z, 1.0f};
    return out;
}

// vector-lerp! (vector.gc): w 1.0.
Vec& vector_lerp(Vec& out, const Vec& a, const Vec& b, float t) {
    out = Vec{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, 1.0f};
    return out;
}

// vector-seek! (vector.gc): each of xyz moved toward target by at most max_step; w 1.0.
Vec& vector_seek(Vec& v, const Vec& target, float max_step) {
    for (int i = 0; i < 3; i++) {
        const float d = std::fmax(std::fmin(target[i] - v[i], max_step), 0.0f - max_step);
        v[i] = v[i] + d;
    }
    v.w = 1.0f;
    return v;
}

// vector-segment-overlap (geometry.gc).
float vector_segment_overlap(const Vec& p, const Vec& a, const Vec& b) {
    Vec d;
    vector_sub(d, a, b);
    Vec n;
    vector_normalize_copy(n, d, 1.0f);
    Vec ap;
    vector_sub(ap, a, p);
    return goal_div(vector_dot(n, ap), vector_length(d));
}

// vector-y-angle (quaternion.gc).
float vector_y_angle(const Vec& v) { return atan(v.x, v.z); }

// vector-rotate-y! (quaternion.gc).
Vec& vector_rotate_y(Vec& out, const Vec& in, float rot) {
    Quat q;
    quaternion_vector_angle(q, Vec{0.0f, 1.0f, 0.0f, 1.0f}, rot);
    Mat m;
    quaternion_to_matrix(m, q);
    return vector_matrix_mul(out, in, m);
}

// ---- Matrices and quaternions ----

Mat& matrix_identity(Mat& m) {
    m = Mat{};
    m.v[0].x = m.v[1].y = m.v[2].z = m.v[3].w = 1.0f;
    return m;
}

// vector-matrix*! (matrix.gc): every lane, w included.
Vec& vector_matrix_mul(Vec& out, const Vec& v, const Mat& m) {
    Vec r;
    for (int i = 0; i < 4; i++) {
        float acc = m.v[0][i] * v.x;
        acc = acc + m.v[1][i] * v.y;
        acc = acc + m.v[2][i] * v.z;
        r[i] = acc + m.v[3][i] * v.w;
    }
    out = r;
    return out;
}

// matrix-transpose! (matrix.gc).
Mat& matrix_transpose(Mat& out, const Mat& m) {
    Mat r;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            r.v[i][j] = m.v[j][i];
        }
    }
    out = r;
    return out;
}

Quat& quaternion_identity(Quat& q) {
    q = Quat{};
    return q;
}

// quaternion-vector-angle! (quaternion.gc).
Quat& quaternion_vector_angle(Quat& out, const Vec& axis, float rot) {
    Vec sc;
    sincos(sc, rot / 2.0f);
    out.x = axis.x * sc.x;
    out.y = axis.y * sc.x;
    out.z = axis.z * sc.x;
    out.w = sc.y;
    return out;
}

// quaternion->matrix (quaternion.gc), the vector unit's lanes.
Mat& quaternion_to_matrix(Mat& out, const Quat& q) {
    const Vec v1{q.x, q.y, q.z, q.w};
    const Vec v5{v1.x + v1.x, v1.y + v1.y, v1.z + v1.z, v1.w + v1.w};
    Vec v2{v1.w, v1.z, -v1.y, -1.0f};
    Vec v3{-v1.z, v1.w, v1.x, -1.0f};
    Vec v4{v1.y, -v1.x, v1.w, -1.0f};
    // outer.product.a acc v5 v2 ; outer.product.b v2 v2 v5 acc : v2 = v5 x v2 (xyz), w kept? The
    // outer product writes xyz only; w stays -1 (0 - 1) until the stores below, which keep it.
    auto outer = [](const Vec& a, const Vec& b) {
        return Vec{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x, 0.0f};
    };
    Vec c2 = outer(v5, v2), c3 = outer(v5, v3), c4 = outer(v5, v4);
    c2.w = v2.w;
    c3.w = v3.w;
    c4.w = v4.w;
    c2.x = c2.x + 1.0f;
    c3.y = c3.y + 1.0f;
    c4.z = c4.z + 1.0f;
    // The w lanes: sub.w.vf.w vfN vf0 vf0 = 1 - 1 = 0.
    c2.w = c3.w = c4.w = 0.0f;
    out.v[0] = c2;
    out.v[1] = c3;
    out.v[2] = c4;
    out.v[3] = Vec{0.0f, 0.0f, 0.0f, 1.0f};
    return out;
}

// quaternion-normalize! (quaternion.gc).
Quat& quaternion_normalize(Quat& q) {
    float acc = q.z * q.z + q.w * q.w;
    acc = acc + q.y * q.y;
    const float s = acc + q.x * q.x;
    const float k = 1.0f / std::sqrt(s);
    q = Quat{q.x * k, q.y * k, q.z * k, q.w * k};
    return q;
}

// quaternion*! (quaternion.gc).
Quat& quaternion_mul(Quat& out, const Quat& a, const Quat& b) {
    // acc = a * b.w + b * a.w, then + (a x b) through vf4.
    const float x = (a.x * b.w + b.x * a.w) + (a.y * b.z - a.z * b.y);
    const float y = (a.y * b.w + b.y * a.w) + (a.z * b.x - a.x * b.z);
    const float z = (a.z * b.w + b.z * a.w) + (a.x * b.y - a.y * b.x);
    float w = a.w * b.w + b.w * a.w; // the w lane of acc
    w = w - a.w * b.w;
    w = w - a.z * b.z;
    w = w - a.y * b.y;
    w = w - a.x * b.x;
    out = Quat{x, y, z, w};
    return out;
}

// quaternion-slerp! (quaternion.gc).
Quat& quaternion_slerp(Quat& out, const Quat& a, const Quat& b, float t) {
    float d = ((a.x * b.x + a.y * b.y) + a.z * b.z) + a.w * b.w;
    float sgn = 1.0f;
    if (d < 0.0f) {
        d = -d;
        sgn = -1.0f;
    }
    if (1.0f - d < 0.0001f) {
        const float k1 = 1.0f - t, k2 = t * sgn;
        out = Quat{a.x * k1 + b.x * k2, a.y * k1 + b.y * k2, a.z * k1 + b.z * k2,
                   a.w * k1 + b.w * k2};
        return quaternion_normalize(out);
    }
    const float s = std::sqrt(1.0f - d * d);
    const float r = goal_div(s - d, s + d);
    const float inv = goal_div(1.0f, s);
    const float ang = atan_series_rad(r);
    Vec sv{(1.0f - t) * ang, t * ang * sgn, 0.0f, 0.0f};
    vector_sin_rad(sv, sv);
    const float k1 = sv.x * inv, k2 = sv.y * inv;
    out = Quat{a.x * k1 + b.x * k2, a.y * k1 + b.y * k2, a.z * k1 + b.z * k2, a.w * k1 + b.w * k2};
    return out;
}

// vector-x/y/z-quaternion! (quaternion.gc): a row of quaternion->matrix.
Vec& vector_x_quaternion(Vec& out, const Quat& q) {
    Mat m;
    quaternion_to_matrix(m, q);
    out = m.v[0];
    return out;
}

Vec& vector_y_quaternion(Vec& out, const Quat& q) {
    Mat m;
    quaternion_to_matrix(m, q);
    out = m.v[1];
    return out;
}

Vec& vector_z_quaternion(Vec& out, const Quat& q) {
    Mat m;
    quaternion_to_matrix(m, q);
    out = m.v[2];
    return out;
}

// quaternion-y-angle (quaternion.gc).
float quaternion_y_angle(const Quat& q) {
    Vec z;
    vector_z_quaternion(z, q);
    return atan(z.x, z.z);
}

namespace {

// matrix->quaternion (quaternion.gc).
Quat& matrix_to_quaternion(Quat& out, const Mat& m) {
    const float tr = (m.v[0].x + m.v[1].y) + m.v[2].z;
    if (0.0f < tr) {
        const float s = std::sqrt(1.0f + tr);
        out.w = s / 2.0f;
        const float k = 0.5f / s;
        out.x = k * (m.v[1].z - m.v[2].y);
        out.y = k * (m.v[2].x - m.v[0].z);
        out.z = k * (m.v[0].y - m.v[1].x);
        return out;
    }
    int a2 = 0, a3 = 1, v1 = 2;
    if (m.v[0].x < m.v[1].y) {
        a2 = 1;
        a3 = 2;
        v1 = 0;
    }
    if (m.v[a2][a2] < m.v[2].z) {
        a2 = 2;
        a3 = 0;
        v1 = 1;
    }
    float s = std::sqrt((1.0f - (m.v[a3][a3] + m.v[v1][v1])) + m.v[a2][a2]);
    out[a2] = s / 2.0f;
    if (s != 0.0f) {
        s = 0.5f / s;
    }
    out.w = (m.v[a3][v1] - m.v[v1][a3]) * s;
    out[a3] = (m.v[a2][a3] + m.v[a3][a2]) * s;
    out[v1] = (m.v[a2][v1] + m.v[v1][a2]) * s;
    return out;
}

} // namespace

// forward-up-nopitch->quaternion (geometry.gc), through forward-down-nopitch->inv-matrix.
Quat& forward_up_nopitch_to_quaternion(Quat& out, const Vec& forward, const Vec& up) {
    Vec down;
    vector_negate(down, up);
    Mat m{};
    vector_normalize_copy(m.v[1], down, 1.0f);
    vector_negate(m.v[1], m.v[1]);
    vector_cross(m.v[0], m.v[1], forward);
    vector_normalize(m.v[0], 1.0f);
    vector_cross(m.v[2], m.v[0], m.v[1]);
    vector_normalize(m.v[2], 1.0f);
    m.v[3] = Vec{0.0f, 0.0f, 0.0f, 1.0f};
    m.v[0].w = m.v[1].w = m.v[2].w = 0.0f;
    return matrix_to_quaternion(out, m);
}

} // namespace jak
