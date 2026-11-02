// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The collision routines OpenGOAL keeps as assembly translated by mips2c (jak-project
// game/mips2c/jak2_functions/collide_func.cpp, collide_cache.cpp, collide_edge_grab.cpp; ISC License,
// Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md), run on the vector unit of collide/vu.hpp
// against the arena of collide/mem.hpp. Each takes its GOAL arguments in a0..t3 and returns in v0,
// as the PC port's ExecutionContext does; goal_call sets a fresh context up the way it does.
#pragma once

#include <cstdint>

#include "jak/collide/vu.hpp"

namespace jak {

using Mips2cFn = uint64_t (*)(mips::Ctx*);

// Runs fn with the given argument registers (a0, a1, a2, a3, t0, t1, t2, t3), s7 = the symbol table
// and sp = the GOAL stack pointer; returns v0.
uint64_t goal_call(Mips2cFn fn, const uint64_t* args, int nargs);

uint64_t m2c_moving_sphere_triangle_intersect(mips::Ctx* c);
uint64_t m2c_collide_do_primitives(mips::Ctx* c);
uint64_t m2c_resolve_moving_sphere_tri(mips::Ctx* c); // (method 9 collide-cache-prim)
uint64_t m2c_probe_using_spheres(mips::Ctx* c);       // (method 17 collide-cache)
uint64_t m2c_puss_work_mesh(mips::Ctx* c);            // (method 9 collide-puss-work)
uint64_t m2c_puss_work_sphere(mips::Ctx* c);          // (method 10 collide-puss-work)
uint64_t m2c_add_to_list(mips::Ctx* c);               // (method 10 collide-edge-hold-list)
uint64_t m2c_find_best_grab(mips::Ctx* c);            // (method 19 collide-edge-work)
uint64_t m2c_find_grabbable_edges(mips::Ctx* c);      // (method 16 collide-edge-work)
uint64_t m2c_find_grabbable_tris(mips::Ctx* c);       // (method 17 collide-edge-work)
uint64_t m2c_should_add_to_list(mips::Ctx* c);        // (method 18 collide-edge-work)

// (method 20 collide-edge-work) check-grab-for-collisions, GOAL code the assembly calls:
// a0 = edge-work, a1 = hold-item, a2 = edge-grab-info; collide_edge_grab.cpp.
uint64_t goal_check_grab_for_collisions(mips::Ctx* c);

} // namespace jak
