// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// What the ported Jak II code needs from the world it runs in, and nothing else: the collision
// geometry around the board, and somewhere to send sounds, particles and trick points. The game's
// adapter (src/rac1) implements it on OpenRAC's mod interface; the tests implement it on geometry
// of their own (tests/world_fixtures.hpp). Everything is in Jak II units (basic_types.hpp).
#pragma once

#include "jak/basic_types.hpp"

namespace jak {

// One collision triangle as the collide cache holds it (collide-cache-tri, collide-cache-h.gc):
// its corners, its normal (unit, facing out of the solid side), and its pat-surface.
struct CollideTri {
    Vec vertex[3];
    Vec normal;
    PatSurface pat;
};

// The collision geometry: the level's and the solid objects', as triangles.
class World {
  public:
    virtual ~World() = default;
    // The triangles that touch the box [lo, hi] (corners in world space), at most `max` of them
    // written to `out`; returns how many were written. The order is the source's own, stable from
    // one call to the next for the same geometry (the cache's order matters to the collision code).
    virtual int triangles(const Vec& lo, const Vec& hi, CollideTri* out, int max) = 0;
};

// Sounds, particles and the trick display: side effects the logic never reads back.
class Effects {
  public:
    virtual ~Effects() = default;
    // sound-play: a one-shot sound by Jak II's name ("board-bounce"); volume 0..1, pitch in
    // GOAL's units (1024 an octave, 0 as recorded). Returns an id for sound_stop / sound_set.
    virtual uint32_t sound_play(const char* name, float volume, float pitch) = 0;
    // A looping sound's volume and pitch (the engine, wind, bank and ride loops), started on first use.
    virtual void sound_loop(const char* name, uint32_t id, float volume, float pitch) = 0;
    virtual void sound_stop(uint32_t id) = 0;
    // part-tracker-spawn / launch-particles: a particle group by name at a point.
    virtual void particles(const char* group, const Vec& at) = 0;
    // A trick added to the combo, and the combo's end (board-trick-tracker, the PC port's display).
    virtual void trick(BoardTricks trick, float points) = 0;
    virtual void combo_end() = 0;
};

} // namespace jak
