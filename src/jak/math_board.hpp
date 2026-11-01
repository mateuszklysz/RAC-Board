// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The few maths routines the board needs beyond math.hpp, ported from OpenGOAL (jak-project, ISC
// License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md) the same way as math.cpp.
#pragma once

#include "jak/basic_types.hpp"

namespace jak {

// quaternion-vector-y-angle (quaternion.gc).
float quaternion_vector_y_angle(const Quat& q, const Vec& v);
// forward-down-nopitch->inv-matrix and forward-up-nopitch->inv-matrix (geometry.gc).
Mat& forward_down_nopitch_to_inv_matrix(Mat& out, const Vec& forward, const Vec& down);
// matrix-rotate-y! (matrix.gc).
Mat& matrix_rotate_y(Mat& out, float rot);
// vector= (vector.gc): x, y and z equal bit for bit (pceqw), w ignored.
bool vector_equal(const Vec& a, const Vec& b);
// vector-identity! (vector.gc): all four lanes 1, as the game has it.
Vec& vector_identity(Vec& v);
// time-to-apex (dynamics-h.gc).
int64_t time_to_apex(float vel, float gravity);

} // namespace jak
