// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The board's extra maths, from OpenGOAL's goal_src/jak2/engine (jak-project, ISC License,
// Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). Each function says which GOAL one it is.
#include "jak/math_board.hpp"

#include <cmath>
#include <cstring>

#include "jak/math.hpp"

namespace jak {

// quaternion-vector-y-angle (quaternion.gc): the quaternion's heading against the vector's.
float quaternion_vector_y_angle(const Quat& q, const Vec& v) {
    const float f30 = quaternion_y_angle(q);
    const float f0 = atan(v.x, v.z);
    return deg_diff(f30, f0);
}

// forward-down-nopitch->inv-matrix (geometry.gc).
Mat& forward_down_nopitch_to_inv_matrix(Mat& out, const Vec& forward, const Vec& down) {
    vector_normalize_copy(out.v[1], down, 1.0f);
    vector_negate(out.v[1], out.v[1]);
    vector_cross(out.v[0], out.v[1], forward);
    vector_normalize(out.v[0], 1.0f);
    vector_cross(out.v[2], out.v[0], out.v[1]);
    vector_normalize(out.v[2], 1.0f);
    out.v[3] = Vec{0.0f, 0.0f, 0.0f, 0.0f};
    out.v[0].w = 0.0f;
    out.v[1].w = 0.0f;
    out.v[2].w = 0.0f;
    out.v[3].w = 1.0f;
    return out;
}

// matrix-rotate-y! (matrix.gc).
Mat& matrix_rotate_y(Mat& out, float rot) {
    const float s = sin(rot);
    const float c = cos(rot);
    out = Mat{};
    out.v[0].x = c;
    out.v[0].z = -s;
    out.v[1].y = 1.0f;
    out.v[2].x = s;
    out.v[2].z = c;
    out.v[3].w = 1.0f;
    return out;
}

// vector= (vector.gc): pceqw compares the words as integers, so -0.0 and 0.0 differ and a NaN
// equals itself; only x, y and z count.
bool vector_equal(const Vec& a, const Vec& b) {
    return std::memcmp(&a.x, &b.x, 12) == 0;
}

// vector-identity! (vector.gc): sets all four lanes to 1 (not an identity at all).
Vec& vector_identity(Vec& v) {
    v.x = 1.0f;
    v.y = 1.0f;
    v.z = 1.0f;
    v.w = 1.0f;
    return v;
}

// time-to-apex (dynamics-h.gc): frames (1/300 s) until a vertical speed is used up by gravity.
int64_t time_to_apex(float vel, float gravity) {
    return static_cast<int64_t>(static_cast<int32_t>(vel / -(gravity / 300.0f)));
}

} // namespace jak
