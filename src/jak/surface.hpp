// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's surface engine (engine/target/surface-h.gc), ported from OpenGOAL (jak-project, ISC
// License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
//
// A surface is a set of movement parameters with hooks. The target keeps two: `mod-surface`, what
// the current state wants (the board's *board-*-mods*, src/board/surfaces.hpp), and `surf`, the
// ground's (*stone-surface*, *ice-surface*..., picked by the collision from the material it stands
// on). Every tick build-conversions multiplies them field by field into `current-surface`
// (surface-mult!), which the movement code reads; the state's mult-hook then adjusts the product.
// The mods hold absolute values (speeds, rates), the ground surfaces factors (1.0 leaves the mod's
// value as it is), except the fields where the ground gives the value and the mod a factor.
#pragma once

#include "jak/basic_types.hpp"
#include "jak/types_gen.hpp"

namespace jak {

// The 28 float fields of a surface in GOAL's order (`data`, overlaid on turnv). GOAL's data array
// has 30 entries: the last two are padding before the hooks that no code reads, so the port has none.
constexpr int kSurfaceDataCount = 28;
float& surface_data(Surface& s, int i);
float surface_data(const Surface& s, int i);

// surface-interp!: out = a * (1 - t) + b * t field by field; b's hooks where set, else a's; b's mode
// and flags.
Surface& surface_interp(Surface& out, const Surface& a, const Surface& b, float t);
// surface-mult!: out = b * a field by field, hooks as surface-interp!, b's mode, a | b flags; then
// out's mult-hook (b's, else a's) runs with pass 1.
Surface& surface_mult(Target& self, Surface& out, const Surface& a, const Surface& b);
// surface-clamp-speed: a mult-hook that keeps the product's top speeds under the mod's.
void surface_clamp_speed(Target& self, Surface& out, const Surface& a, const Surface& b, int pass);

// calc-terminal-vel, calc-terminal2-vel, calc-terminal4-vel.
float calc_terminal_vel(float a0, float a1, float a2);
float calc_terminal2_vel(float a0, float a1, float a2, float a3);
float calc_terminal4_vel(float a0, float a1, float a2);

// The target's own mods the board path uses (surface-h.gc). target-exit puts *walk-mods* back.
extern Surface walk_mods;    // *walk-mods*
extern Surface empty_mods;   // *empty-mods*
extern Surface neutral_mods; // *neutral-mods*
extern Surface grab_mods;    // *grab-mods*: *walk-mods* named grab, no flags
extern Surface smack_mods;   // *smack-mods* (target-death.gc), the air surface of a knock-back

// The ground surfaces (surface-h.gc; *no-walk-surface* is in collide-edge-grab.gc). The collision
// (collide-shape.gc) points `control.surf` at one of them from the ground's material.
extern Surface stone_surface;     // *stone-surface*, also *standard-ground-surface*, *swim-surface*
extern Surface gravel_surface;    // *gravel-surface*
extern Surface edge_surface;      // *edge-surface*
extern Surface wade_surface;      // *wade-surface*
extern Surface quicksand_surface; // *quicksand-surface*
extern Surface tar_surface;       // *tar-surface*
extern Surface ice_surface;       // *ice-surface*
extern Surface rail_surface;      // *rail-surface*
extern Surface no_walk_surface;   // *no-walk-surface*
inline Surface* standard_ground_surface() { return &stone_surface; }

// The surfaces' symbols, compared by name (Symbol).
namespace surface_mode {
constexpr Symbol air{"air"}, ride{"ride"}, swim{"swim"}, dive{"dive"};
} // namespace surface_mode

} // namespace jak
