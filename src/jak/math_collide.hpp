// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The maths Jak II's collision code needs beyond src/jak/math.hpp: geometry.gc, collide-func.gc,
// bounding-box.gc and a few vector.gc, matrix.gc and trigonometry.gc routines, ported from
// OpenGOAL (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md) lane
// by lane on the vector unit of collide_vu.hpp.
#pragma once

#include "jak/basic_types.hpp"
#include "jak/collide_mem.hpp"
#include "jak/math_core.hpp" // asin, acos, vector-vector-xz-distance, vector-reflect-flat!

namespace jak {

// COLLISION_MISS (collide-func.gc).
constexpr float kCollisionMiss = -100000000.0f;

// ---- vector.gc ----
Vec& vector_average(Vec& out, const Vec& a, const Vec& b);          // vector-average!, w 1

// ---- matrix.gc ----
// transform-vectors!: count vectors as points (w taken as 1), each keeping its own w.
void transform_vectors(const Mat& m, Vec* dst, const Vec* src, int count);

// ---- geometry.gc ----
Vec& vector_reflect_flat_above(Vec& out, const Vec& in, const Vec& normal);     // vector-reflect-flat-above!
Vec& vector_reflect_flat_gravity(Vec& out, const Vec& in, const Vec& normal, const Vec& grav);
// vector-segment-distance-point!: the distance from p to the segment [a, b]; the closest point
// in *out when out is not null.
float vector_segment_distance_point(const Vec& p, const Vec& a, const Vec& b, Vec* out);
// vector-line-distance-point!
float vector_line_distance_point(const Vec& p, const Vec& a, const Vec& b, Vec* out);
// closest-pt-in-triangle: tri is three vertices (a matrix's first three rows).
void closest_pt_in_triangle(Vec& out, const Vec& p, const Vec* tri, const Vec& normal);

// ---- bounding-box.gc ----
void set_from_point_offset_pad(BoundingBox& box, const Vec& p, const Vec& offset, float pad);
void set_from_spheres(BoundingBox& box, const Vec* spheres, int count);

// ---- collide-func.gc ----
// pc-port-raw-ray-sphere-implementation: a sphere of radius rad at the origin, the probe from
// origin o along dir d. Returns the fraction of d, 0 inside, kCollisionMiss on a miss.
float raw_ray_sphere_intersect(float rad, const Vec& o, const Vec& d);
float ray_sphere_intersect(const Vec& origin, const Vec& dir, const Vec& center, float radius);
float ray_cylinder_intersect(const Vec& origin, const Vec& dir, const Vec& cyl_origin,
                             const Vec& cyl_axis, float cyl_rad, float cyl_len, Vec& pt_out);

} // namespace jak
