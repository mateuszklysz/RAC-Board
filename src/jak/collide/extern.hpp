// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// What the collision code (src/jak/collide/) calls in the other parts, declared here and
// implemented by their owners. Each line names the GOAL routine or global it stands for (OpenGOAL,
// jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
#pragma once

#include "jak/basic_types.hpp"
#include "jak/types_gen.hpp"

namespace jak {

struct Target;

// owned by core: the ground surfaces react-to-pat! points control surf at (surface-h.gc,
// collide-edge-grab.gc), defined in src/jak/surface.cpp.
extern Surface stone_surface;     // *stone-surface*, also *standard-ground-surface*
extern Surface gravel_surface;    // *gravel-surface*
extern Surface edge_surface;      // *edge-surface*
extern Surface quicksand_surface; // *quicksand-surface*
extern Surface ice_surface;       // *ice-surface*
extern Surface rail_surface;      // *rail-surface*
extern Surface no_walk_surface;   // *no-walk-surface*


} // namespace jak
