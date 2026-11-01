// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's maths (engine/math: math.gc, trigonometry.gc, vector.gc, vector-h.gc, quaternion.gc,
// matrix.gc), ported from OpenGOAL (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL
// Team; THIRD_PARTY.md) operation by operation, with GOAL's names ('-' as '_', '!' dropped, '->'
// as '_to_'). The port reproduces what OpenGOAL computes on a PC: IEEE single floats, no fused
// multiply-add (the build passes -ffp-contract=off), the polynomial sine and arctangent of the game
// rather than the C library's.
//
// Only the routines several parts of the board need are here. A part that needs another ports it
// into its own math file (math_<part>.cpp) the same way.
#pragma once

#include <cstring>

#include "jak/basic_types.hpp"

namespace jak {

// GOAL's `/` on floats (goalc compile_floating_point_division): a divisor of exactly 0 (either
// sign) gives FLT_MAX with the sign of a's sign bit xor b's, as the PS2's FPU does, never inf or NaN.
// Use it for every GOAL `(/ a b)` whose divisor is not a constant; VU divisions (.div.vf, .isqrt)
// are plain IEEE.
inline float goal_div(float a, float b) {
    if (b == 0.0f) {
        uint32_t ua, ub, r;
        std::memcpy(&ua, &a, 4);
        std::memcpy(&ub, &b, 4);
        r = 0x7f7fffffu ^ (ua & 0x80000000u) ^ (ub & 0x80000000u);
        float f;
        std::memcpy(&f, &r, 4);
        return f;
    }
    return a / b;
}

// ---- Scalars (math.gc, trigonometry.gc) ----
float lerp_scale(float out0, float out1, float in, float in0, float in1);
float lerp(float a, float b, float t);
float seek(float x, float target, float max_step);
float fsign(float x); // GOAL's `sign`
float sin(float rot);
float cos(float rot);
float atan0(float y, float x);
float atan(float y, float x); // rotation units, signs as atan2
float deg_minus(float a, float b); // (deg- a b): a - b, wrapped to +-32768
float deg_diff(float a, float b); // (deg-diff a b): b - a, wrapped
float deg_seek(float in, float target, float max_diff);
float deg_seek_smooth(float in, float target, float max_diff, float alpha);
float unwrap_angle(float a);

// ---- Vectors (vector.gc, vector-h.gc) ----
Vec& vector_reset(Vec& v); // xyzw 0
Vec& vector_add(Vec& out, const Vec& a, const Vec& b); // vector+!, w 1? no: GOAL's keeps a.w
Vec& vector_sub(Vec& out, const Vec& a, const Vec& b); // vector-!
Vec& vector_float_mul(Vec& out, const Vec& a, float s); // vector-float*!
Vec& vector_add_float_mul(Vec& out, const Vec& a, const Vec& b, float s); // vector+float*!
Vec& vector_negate(Vec& out, const Vec& a);
float vector_dot(const Vec& a, const Vec& b);
Vec& vector_cross(Vec& out, const Vec& a, const Vec& b);
float vector_length(const Vec& v);
float vector_xz_length(const Vec& v);
float vector_vector_distance(const Vec& a, const Vec& b);
Vec& vector_normalize(Vec& v, float len);
Vec& vector_normalize_copy(Vec& out, const Vec& in, float len);
Vec& vector_xz_normalize(Vec& v, float len);
Vec& vector_flatten(Vec& out, const Vec& in, const Vec& normal);
Vec& vector_reflect(Vec& out, const Vec& in, const Vec& normal);
Vec& vector_lerp(Vec& out, const Vec& a, const Vec& b, float t);
Vec& vector_seek(Vec& v, const Vec& target, float max_step);
float vector_segment_overlap(const Vec& p, const Vec& a, const Vec& b);
float vector_y_angle(const Vec& v);
Vec& vector_rotate_y(Vec& out, const Vec& in, float rot);

// ---- Matrices and quaternions (matrix.gc, quaternion.gc) ----
Mat& matrix_identity(Mat& m);
Vec& vector_matrix_mul(Vec& out, const Vec& v, const Mat& m); // vector-matrix*! (rows, w too)
Mat& matrix_transpose(Mat& out, const Mat& m);
Quat& quaternion_identity(Quat& q);
Quat& quaternion_vector_angle(Quat& out, const Vec& axis, float rot);
Mat& quaternion_to_matrix(Mat& out, const Quat& q);
Quat& quaternion_normalize(Quat& q);
Quat& quaternion_mul(Quat& out, const Quat& a, const Quat& b); // quaternion*!
Quat& quaternion_slerp(Quat& out, const Quat& a, const Quat& b, float t);
Vec& vector_x_quaternion(Vec& out, const Quat& q); // vector-x-quaternion!
Vec& vector_y_quaternion(Vec& out, const Quat& q);
Vec& vector_z_quaternion(Vec& out, const Quat& q);
float quaternion_y_angle(const Quat& q);
Quat& forward_up_nopitch_to_quaternion(Quat& out, const Vec& forward, const Vec& up);

} // namespace jak
