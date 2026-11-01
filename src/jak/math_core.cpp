// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// More of Jak II's maths, for the target's movement, from OpenGOAL's goal_src/jak2/engine/math and
// geometry (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). Each
// function says which GOAL one it is; vector-unit code is written lane by lane as OpenGOAL runs it.
#include "jak/math_core.hpp"

#include <cmath>
#include <cstring>

#include "jak/math.hpp"

namespace jak {

namespace {

uint32_t bits_of(float f) {
    uint32_t u;
    std::memcpy(&u, &f, 4);
    return u;
}

// The PC port's int-to-float-nonzero-hack (geometry.gc): a zero frame count would divide by zero,
// which on the PS2 happened to turn the smoothing off; a tiny count does the same on a PC.
float frames_nonzero(int frames) {
    return frames == 0 ? 0.00000000001f : static_cast<float>(frames);
}

} // namespace

// asin (trigonometry.gc).
float asin(float x) {
    bool neg = false;
    if (x < 0.0f) {
        x = -x;
        neg = true;
    }
    float r;
    if (1.0f < x) {
        r = 16383.996f;
    } else {
        r = atan0(x, std::sqrt(1.0f - x * x));
    }
    return neg ? -r : r;
}

// acos (trigonometry.gc), PC_PORT: a result of exactly 0 becomes 1e-11 (OpenGOAL's fix for a punch
// glitch that came from the PS2's rounding).
float acos(float x) {
    float r = 16384.0f - asin(x);
    if (r == 0.0f) {
        r = 0.00000000001f;
    }
    return r;
}

// vector-float/! (vector.gc): multiplies by the reciprocal (the VU's Q register); w 1.
Vec& vector_float_div(Vec& out, const Vec& a, float d) {
    const float q = 1.0f / d;
    out = Vec{a.x * q, a.y * q, a.z * q, 1.0f};
    return out;
}

// vector--float*! (vector.gc): a - b * s, w 1.
Vec& vector_sub_float_mul(Vec& out, const Vec& a, const Vec& b, float s) {
    out = Vec{a.x - b.x * s, a.y - b.y * s, a.z - b.z * s, 1.0f};
    return out;
}

// vector-v++! (vector.gc): position += velocity * (seconds-per-frame).
Vec& vector_v_inc(Vec& v, const Vec& vel, float spf) { return vector_add_float_mul(v, v, vel, spf); }

// vector-vector-xz-distance (vector.gc).
float vector_vector_xz_distance(const Vec& a, const Vec& b) {
    const float dx = b.x - a.x, dz = b.z - a.z;
    return std::sqrt(dx * dx + dz * dz);
}

// vector-reflect-flat! (geometry.gc): n x (in x n) + n, by two outer products; w 1.
Vec& vector_reflect_flat(Vec& out, const Vec& in, const Vec& n) {
    const Vec c{in.y * n.z - in.z * n.y, in.z * n.x - in.x * n.z, in.x * n.y - in.y * n.x, 1.0f};
    const Vec f{n.y * c.z - n.z * c.y, n.z * c.x - n.x * c.z, n.x * c.y - n.y * c.x, 1.0f};
    out = Vec{f.x + n.x, f.y + n.y, f.z + n.z, 1.0f};
    return out;
}

// matrix-axis-sin-cos! (matrix.gc). An axis whose x, y and z are all bit-zero (pceqw compares the
// words) gives the identity.
Mat& matrix_axis_sin_cos(Mat& out, const Vec& a, float s, float c) {
    if (bits_of(a.x) == 0 && bits_of(a.y) == 0 && bits_of(a.z) == 0) {
        return matrix_identity(out);
    }
    const float omc = 1.0f - c; // vf5.w = vf0.w - cos
    const Vec as{a.x * s, a.y * s, a.z * s, 0.0f};     // vf11
    const Vec ao{a.x * omc, a.y * omc, a.z * omc, 0.0f}; // vf10
    Mat m;
    m.v[0] = Vec{ao.x * a.x + c, ao.y * a.x + (0.0f - as.z), ao.z * a.x + as.y, 0.0f};
    m.v[1] = Vec{ao.x * a.y + as.z, ao.y * a.y + c, ao.z * a.y + (0.0f - as.x), 0.0f};
    m.v[2] = Vec{ao.x * a.z + (0.0f - as.y), ao.y * a.z + as.x, ao.z * a.z + c, 0.0f};
    m.v[3] = Vec{0.0f, 0.0f, 0.0f, 1.0f};
    out = m;
    return out;
}

// matrix->quaternion (quaternion.gc).
Quat& matrix_to_quaternion_core(Quat& out, const Mat& m) {
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
    float* d = &out.x;
    d[a2] = s / 2.0f;
    if (s != 0.0f) {
        s = 0.5f / s;
    }
    out.w = (m.v[a3][v1] - m.v[v1][a3]) * s;
    d[a3] = (m.v[a2][a3] + m.v[a3][a2]) * s;
    d[v1] = (m.v[a2][v1] + m.v[v1][a2]) * s;
    return out;
}

// forward-up-nopitch->inv-matrix through forward-down-nopitch->inv-matrix (geometry.gc).
Mat& forward_up_nopitch_to_inv_matrix(Mat& out, const Vec& forward, const Vec& up) {
    Vec down;
    vector_negate(down, up);
    down.w = 1.0f;
    Mat m{};
    vector_normalize_copy(m.v[1], down, 1.0f);
    vector_negate(m.v[1], m.v[1]);
    vector_cross(m.v[0], m.v[1], forward);
    vector_normalize(m.v[0], 1.0f);
    vector_cross(m.v[2], m.v[0], m.v[1]);
    vector_normalize(m.v[2], 1.0f);
    m.v[3] = Vec{0.0f, 0.0f, 0.0f, 1.0f};
    m.v[0].w = 0.0f;
    m.v[1].w = 0.0f;
    m.v[2].w = 0.0f;
    out = m;
    return out;
}

// matrix-from-two-vectors! (geometry.gc).
Mat& matrix_from_two_vectors(Mat& out, const Vec& a, const Vec& b) {
    Vec axis;
    vector_cross(axis, b, a);
    vector_normalize(axis, 1.0f);
    const float c = vector_dot(a, b);
    const float s = std::sqrt(1.0f - c * c);
    return matrix_axis_sin_cos(out, axis, s, c);
}

// matrix-from-two-vectors-smooth! (geometry.gc): turn a toward b by at most rate per second, and
// slower near the end (frames is the time constant).
Mat& matrix_from_two_vectors_smooth(Mat& out, const Vec& a, const Vec& b, float rate, int frames,
                                    float spf) {
    Vec axis;
    vector_cross(axis, b, a);
    vector_normalize(axis, 1.0f);
    const float ang = acos(vector_dot(a, b));
    const float lim = std::fmin(rate * spf, goal_div(5.0f * std::fabs(ang), frames_nonzero(frames)));
    const float r = std::fmax(std::fmin(ang, lim), -lim);
    return matrix_axis_sin_cos(out, axis, sin(r), cos(r));
}

// matrix-from-two-vectors-the-long-way-smooth! (geometry.gc).
Mat& matrix_from_two_vectors_the_long_way_smooth(Mat& out, const Vec& a, const Vec& b, float rate,
                                                 int frames, float spf) {
    Vec axis;
    vector_cross(axis, b, a);
    vector_normalize(axis, 1.0f);
    const float ang = -acos(vector_dot(a, b));
    const float lim = std::fmin(rate * spf, goal_div(5.0f * std::fabs(ang), frames_nonzero(frames)));
    const float r = std::fmax(std::fmin(ang, lim), -lim);
    return matrix_axis_sin_cos(out, axis, sin(r), cos(r));
}

// matrix-from-two-vectors-partial-linear! (geometry.gc).
Mat& matrix_from_two_vectors_partial_linear(Mat& out, const Vec& a, const Vec& b, float t) {
    Vec axis;
    vector_cross(axis, b, a);
    vector_normalize(axis, 1.0f);
    const float d = vector_dot(a, b);
    if (0.9999f < std::fabs(d)) {
        return matrix_identity(out);
    }
    const float c = cos(t * acos(d));
    return matrix_axis_sin_cos(out, axis, std::sqrt(1.0f - c * c), c);
}

// quaternion-from-two-vectors-smooth! (geometry.gc).
Quat& quaternion_from_two_vectors_smooth(Quat& out, const Vec& a, const Vec& b, float rate,
                                         int frames, float spf) {
    Mat m;
    matrix_from_two_vectors_smooth(m, a, b, rate, frames, spf);
    return matrix_to_quaternion_core(out, m);
}

// vector-deg-slerp (geometry.gc).
Vec& vector_deg_slerp(Vec& out, const Vec& a, const Vec& b, float t) {
    if (0.0f >= t) {
        out = a;
        return out;
    }
    if (t >= 1.0f) {
        out = b;
        return out;
    }
    Mat m;
    Vec na, nb;
    vector_normalize_copy(na, a, 1.0f);
    vector_normalize_copy(nb, b, 1.0f);
    matrix_from_two_vectors_partial_linear(m, na, nb, t);
    return vector_matrix_mul(out, a, m);
}

// rotate-toward-orientation! (transformq.gc): first the up axis toward the target's (tilt, by at
// most tilt_rate per second), then the heading about the new up (turn). long_way nonzero turns the
// other way round when the cross product's side disagrees with it.
Quat& rotate_toward_orientation(Quat& q, const Quat& target, float turn_rate, float tilt_rate,
                                int turn_frames, int tilt_frames, float long_way, float spf) {
    Quat r;
    if (0.0f < tilt_rate) {
        Vec up, tup;
        vector_y_quaternion(up, q);
        vector_y_quaternion(tup, target);
        quaternion_from_two_vectors_smooth(r, up, tup, tilt_rate, tilt_frames, spf);
        quaternion_mul(q, r, q);
        quaternion_normalize(q);
    }
    if (0.0f < turn_rate) {
        Vec up, fwd, tfwd;
        vector_y_quaternion(up, q);
        vector_z_quaternion(fwd, q);
        vector_z_quaternion(tfwd, target);
        Mat m;
        vector_flatten(fwd, fwd, up);
        vector_flatten(tfwd, tfwd, up);
        vector_normalize(fwd, 1.0f);
        vector_normalize(tfwd, 1.0f);
        if (long_way != 0.0f) {
            Vec c;
            vector_cross(c, fwd, tfwd);
            const float side = vector_dot(up, c);
            if (side * long_way < 0.0f) {
                matrix_from_two_vectors_the_long_way_smooth(m, fwd, tfwd, turn_rate, turn_frames,
                                                            spf);
            } else {
                matrix_from_two_vectors_smooth(m, fwd, tfwd, turn_rate, turn_frames, spf);
            }
        } else {
            matrix_from_two_vectors_smooth(m, fwd, tfwd, turn_rate, turn_frames, spf);
        }
        matrix_to_quaternion_core(r, m);
        quaternion_mul(q, r, q);
        quaternion_normalize(q);
    }
    return q;
}

} // namespace jak
