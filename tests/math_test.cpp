// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's maths (src/jak/math/math.cpp) against known values: the polynomial sine and arctangent near
// the C library's, the angle wrapping, and the vector and quaternion identities.
#include "check.hpp"
#include "jak/math/math.hpp"

using namespace jak;

int main() {
    // Rotation units: 16384 is a quarter turn.
    CHECK_NEAR(jak::sin(16384.0f), 1.0, 1e-5);
    CHECK_NEAR(jak::sin(0.0f), 0.0, 1e-7);
    CHECK_NEAR(jak::cos(0.0f), 1.0, 1e-5);
    CHECK_NEAR(jak::sin(8192.0f), std::sqrt(0.5), 1e-5);
    CHECK_NEAR(jak::sin(65536.0f + 8192.0f), std::sqrt(0.5), 1e-5); // unwrapped
    CHECK_NEAR(jak::atan(1.0f, 1.0f), 8192.0, 1.0);
    CHECK_NEAR(jak::atan(1.0f, 0.0f), 16384.0, 1.0);
    CHECK_NEAR(jak::atan(0.0f, -1.0f), 32768.0, 1.0);
    CHECK_NEAR(jak::atan(-1.0f, -1.0f), -24576.0, 1.0);
    CHECK_NEAR(jak::atan(-1.0f, 1.0f), -8192.0, 1.0);

    // Wrapping: deg- and deg-diff keep the result within half a turn.
    CHECK(deg_minus(30000.0f, -30000.0f) == -5536.0f);
    CHECK(deg_diff(-30000.0f, 30000.0f) == -5536.0f);
    CHECK(deg_seek(0.0f, 1000.0f, 100.0f) == 100.0f);
    CHECK(deg_seek(0.0f, 1000.0f, 2000.0f) == 1000.0f);
    CHECK(deg_seek(30000.0f, -30000.0f, 100.0f) == 30100.0f);

    // Scalars.
    CHECK(lerp_scale(0.4f, 1.0f, 0.3f, 0.3f, 1.0f) == 0.4f);
    CHECK(lerp_scale(0.4f, 1.0f, 5.0f, 0.3f, 1.0f) == 1.0f);
    CHECK(seek(0.0f, 10.0f, 3.0f) == 3.0f);
    CHECK(seek(0.0f, -10.0f, 3.0f) == -3.0f);
    CHECK(seek(9.0f, 10.0f, 3.0f) == 10.0f);

    // Vectors: flatten and reflect against the floor, the y-angle, the xz normalize.
    Vec up{0.0f, 1.0f, 0.0f, 1.0f}, v{3.0f, -4.0f, 5.0f, 1.0f}, r;
    vector_flatten(r, v, up);
    CHECK_NEAR(r.x, 3.0, 1e-6);
    CHECK_NEAR(r.y, 0.0, 1e-6);
    CHECK_NEAR(r.z, 5.0, 1e-6);
    vector_reflect(r, v, up);
    CHECK_NEAR(r.x, 3.0, 1e-6); // off the floor: y flips, x and z stay
    CHECK_NEAR(r.y, 4.0, 1e-6);
    CHECK_NEAR(vector_y_angle(Vec{1.0f, 0.0f, 0.0f, 0.0f}), 16384.0, 1.0);
    Vec xz{3.0f, 7.0f, 4.0f, 0.0f};
    vector_xz_normalize(xz, 10.0f);
    CHECK_NEAR(xz.x, 6.0, 1e-5);
    CHECK_NEAR(xz.z, 8.0, 1e-5);
    CHECK(xz.y == 7.0f);

    // A quarter turn about y takes +z to +x; a little long, for the game's cosine coefficient
    // (sincos-rad-asm without FIX_COSINE_BUG), which the board inherits.
    Vec z{0.0f, 0.0f, 1.0f, 1.0f};
    vector_rotate_y(r, z, 16384.0f);
    CHECK_NEAR(r.x, 1.00168622, 1e-6);
    CHECK_NEAR(r.z, 0.0, 1e-4);

    // Quaternion <-> matrix: identity, and a turn's y angle read back.
    Quat q;
    Mat m;
    quaternion_to_matrix(m, q);
    CHECK(m.v[0].x == 1.0f && m.v[1].y == 1.0f && m.v[2].z == 1.0f && m.v[3].w == 1.0f);
    quaternion_vector_angle(q, up, 4096.0f);
    CHECK_NEAR(quaternion_y_angle(q), 4096.0, 2.0);
    Quat q2;
    forward_up_nopitch_to_quaternion(q2, Vec{1.0f, 0.5f, 1.0f, 1.0f}, up);
    CHECK_NEAR(quaternion_y_angle(q2), 8192.0, 2.0);
    Quat qq;
    quaternion_mul(qq, q, q);
    CHECK_NEAR(quaternion_y_angle(qq), 8192.0, 2.0);
    Quat s;
    quaternion_slerp(s, Quat{}, qq, 0.5f);
    CHECK_NEAR(quaternion_y_angle(s), 4096.0, 4.0);

    TEST_MAIN_END();
}
