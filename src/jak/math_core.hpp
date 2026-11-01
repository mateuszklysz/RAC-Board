// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The maths the target's movement needs beyond src/jak/math.hpp (trigonometry.gc, vector.gc,
// geometry.gc, matrix.gc, quaternion.gc, transformq.gc), ported from OpenGOAL (jak-project, ISC
// License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md) in the same way as math.cpp.
// Routines that read (seconds-per-frame) take it as their last argument.
#pragma once

#include "jak/basic_types.hpp"
#include "jak/types_gen.hpp"

namespace jak {

float asin(float x); // rotation units
float acos(float x); // rotation units, with the PC port's never-zero result

Vec& vector_float_div(Vec& out, const Vec& a, float d);                    // vector-float/!
Vec& vector_sub_float_mul(Vec& out, const Vec& a, const Vec& b, float s);  // vector--float*!
Vec& vector_v_inc(Vec& v, const Vec& vel, float spf);                      // vector-v++!
float vector_vector_xz_distance(const Vec& a, const Vec& b);
Vec& vector_reflect_flat(Vec& out, const Vec& in, const Vec& n);           // vector-reflect-flat!
Vec& vector_deg_slerp(Vec& out, const Vec& a, const Vec& b, float t);

Mat& matrix_axis_sin_cos(Mat& out, const Vec& axis, float s, float c);     // matrix-axis-sin-cos!
Quat& matrix_to_quaternion_core(Quat& out, const Mat& m);                  // matrix->quaternion
Mat& forward_up_nopitch_to_inv_matrix(Mat& out, const Vec& forward, const Vec& up);
Mat& matrix_from_two_vectors(Mat& out, const Vec& a, const Vec& b);
Mat& matrix_from_two_vectors_smooth(Mat& out, const Vec& a, const Vec& b, float rate, int frames,
                                    float spf);
Mat& matrix_from_two_vectors_the_long_way_smooth(Mat& out, const Vec& a, const Vec& b, float rate,
                                                 int frames, float spf);
Mat& matrix_from_two_vectors_partial_linear(Mat& out, const Vec& a, const Vec& b, float t);
Quat& quaternion_from_two_vectors_smooth(Quat& out, const Vec& a, const Vec& b, float rate,
                                         int frames, float spf);

// rotate-toward-orientation! (transformq.gc), a trsqv method working on (get-quaternion this): pass
// that quaternion (for control-info it is quat-for-control, target-util.gc).
Quat& rotate_toward_orientation(Quat& q, const Quat& target, float turn_rate, float tilt_rate,
                                int turn_frames, int tilt_frames, float long_way, float spf);

} // namespace jak
