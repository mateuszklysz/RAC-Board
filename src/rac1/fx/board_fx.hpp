// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The board's particles: Jak II's board effects (board-part.gc and the target's, as the board's
// logic launches them by name) drawn with the mod's own sprites (assets/fx, tools/make_fx.py) as
// OpenRAC props, glowing (added) or fading (blended). Each part's numbers are Jak II's init-specs,
// in metres and frames at 60 a second, turned into Ratchet's units by kJakScale.
#pragma once

#include <cstdint>
#include <vector>

namespace openrac {
class Mod;
}

namespace rac1 {

class BoardFx {
  public:
    // A group of Jak II's by name ("group-target-board", "part-431", ...) at `at` (game units),
    // the board heading `yaw`; `rate` scales how many it launches (the rail sparks' speed).
    void launch(const char* group, const float at[3], float yaw, float rate = 1.0f);
    // The hit's lightning ('shock modes'): sparks over his body for a moment.
    void shock(const float at[3]);
    // One tick: moves, fades and draws every particle alive.
    void update(openrac::Mod& mod, float dt);
    // All gone at once (a new level: its props go with it).
    void clear(openrac::Mod& mod);
    // Whether the heat haze (part 432, a distort sprite) is drawn (OpenRAC's haze_set).
    bool haze = true;

    enum Sprite { glow, streak, ring, puff, burst, kSprites };
    struct Particle {
        Sprite sprite = glow;
        bool flat = false;     // lies in the ground's plane (is-3d, birth-func-target-orient)
        bool along = false;    // drawn along its velocity (sparticle-motion-blur)
        bool add = true;       // added (a glow) or blended at its alpha (dust)
        float pos[3]{}, vel[3]{};
        float accel = 0.0f;    // up, a frame
        float friction = 1.0f; // velocity kept a frame
        float sx = 1.0f, sy = 1.0f, ds = 0.0f, dsy = 0.0f;
        float rot = 0.0f, drot = 0.0f;
        float rgba[4]{}, fade[4]{};
        float life = 0.0f;     // seconds left
    };

  private:
    // A patch of heat haze: where, how big, how strong at birth, and its life.
    struct Haze {
        float pos[3]{};
        float radius = 0.0f, strength = 0.0f;
        float life = 0.0f, full = 1.0f;
    };
    void update_haze(openrac::Mod& mod, float dt);
    std::vector<Haze> m_haze;
    bool m_haze_shown = false;

    float rand(float lo, float range);
    void spawn(const Particle& p);
    std::vector<Particle> m_parts;
    std::vector<int> m_props[kSprites];
    uint32_t m_seed = 0x1234567u;
    float m_carry = 0.0f;  // the rail sparks' part of a particle left over
};

} // namespace rac1
