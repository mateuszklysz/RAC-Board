// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The parts' numbers are Jak II's (board-part.gc, target-part.gc), from OpenGOAL (jak-project, ISC
// License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). The sprites are the mod's own.
#include "rac1/board_fx.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <openrac/mod.hpp>

#include "rac1/convert.hpp"

namespace rac1 {

namespace {

constexpr float kDeg = 3.14159265f / 180.0f;
constexpr const char* kMeshes[] = {"assets/fx/glow.glb", "assets/fx/streak.glb", "assets/fx/ring.glb",
                                   "assets/fx/puff.glb", "assets/fx/burst.glb"};
// The most of each sprite alive at once (all mods share OPENRAC_PROPS_MAX props).
constexpr std::size_t kMost[] = {56, 96, 12, 40, 4};

// Jak II's metres in game units.
float m(float metres) { return metres * kJakScale; }

} // namespace

float BoardFx::rand(float lo, float range) {
    m_seed = m_seed * 1664525u + 1013904223u;
    return lo + range * static_cast<float>(m_seed >> 8) / 16777216.0f;
}

void BoardFx::spawn(const Particle& p) {
    std::size_t alive = 0;
    for (const Particle& q : m_parts) alive += q.sprite == p.sprite;
    if (alive >= kMost[p.sprite]) return;
    m_parts.push_back(p);
}

namespace {

// A part's colour as the GS takes it (128 is 1), scaled into 0..1 so its hue stays (a channel past
// 1 would only whiten it under the port's tone mapping).
void colour(BoardFx::Particle& p, float r, float g, float b, float a) {
    const float k = std::max({128.0f, r, g, b});
    p.rgba[0] = r / k;
    p.rgba[1] = g / k;
    p.rgba[2] = b / k;
    p.rgba[3] = a / 128.0f;
}

// conerot: from up, `x` degrees over, turned `y` degrees about up; `yaw` the heading it is turned by.
void cone(float out[3], float x, float y) {
    out[0] = std::sin(x * kDeg) * std::cos(y * kDeg);
    out[1] = std::sin(x * kDeg) * std::sin(y * kDeg);
    out[2] = std::cos(x * kDeg);
}

} // namespace

void BoardFx::launch(const char* group, const float at[3], float yaw, float rate) {
    const auto place = [&](Particle& p, float up) {
        std::copy(at, at + 3, p.pos);
        p.pos[2] += m(up);
    };
    if (!std::strcmp(group, "group-target-board")) {
        // 433: the hover's glow on the ground under the board, shrinking.
        Particle g;
        g.sprite = glow;
        g.flat = true;
        place(g, 0.25f);
        g.sx = g.sy = m(rand(4.0f, 2.0f));
        g.ds = g.dsy = m(rand(-0.02f, -0.006666667f));
        g.rot = yaw;
        const float c = rand(64.0f, 128.0f);
        colour(g, c, c, 255.0f, rand(8.0f, 8.0f));
        g.fade[3] = -0.1f / 128.0f;
        g.life = 0.535f;
        spawn(g);
        // 434: blue sparks thrown out round the board, falling, going blue as they fade.
        Particle s;
        s.sprite = streak;
        s.along = true;
        const float c2 = rand(128.0f, 64.0f);
        colour(s, c2, c2, 255.0f, rand(64.0f, 32.0f));
        float d[3];
        cone(d, rand(80.0f, 10.0f), rand(0.0f, 360.0f));
        const float r0 = m(rand(0.0f, 0.5f)), v = m(rand(0.033333335f, 0.033333335f));
        place(s, 0.0f);
        for (int k = 0; k < 3; ++k) {
            s.pos[k] += d[k] * r0;
            s.vel[k] = d[k] * v;
        }
        s.sx = m(2.5f);
        s.sy = m(0.05f);
        s.fade[0] = s.fade[1] = -3.0f / 128.0f;
        s.fade[3] = -rand(0.21333334f, 0.21333334f) / 128.0f;
        s.accel = -m(rand(0.00033333333f, 0.00033333333f));
        s.friction = 0.95f;
        s.life = rand(0.167f, 0.497f);
        spawn(s);
        // 432 is a heat haze (distort): the sprites have none.
    } else if (!std::strcmp(group, "part-431")) {
        // 431: the rail's sparks, white going orange then red, more the faster he grinds.
        m_carry += rate;
        while (m_carry >= 1.0f) {
            m_carry -= 1.0f;
            Particle s;
            s.sprite = streak;
            s.along = true;
            colour(s, 255.0f, 255.0f, 255.0f, rand(64.0f, 32.0f));
            float d[3];
            cone(d, rand(45.0f, 90.0f), rand(0.0f, 360.0f));
            const float v = m(rand(0.033333335f, 0.033333335f));
            place(s, 0.0f);
            for (int k = 0; k < 3; ++k) s.vel[k] = d[k] * v;
            s.sx = m(2.5f);
            s.sy = m(0.05f);
            s.fade[1] = -rand(0.85f, 1.7f) / 128.0f;
            s.fade[2] = -8.0f / 128.0f;
            s.fade[3] = -rand(0.16f, 0.16f) / 128.0f;
            s.accel = -m(rand(0.0016666667f, 0.00066666666f));
            s.friction = 0.95f;
            s.life = rand(0.167f, 0.497f);
            spawn(s);
        }
    } else if (!std::strcmp(group, "group-land-poof-stn")) {
        // 112 and 113: a ring of dust where he lands hard.
        for (int big = 1; big >= 0; --big) {
            const int n = big ? 16 : 12;
            for (int i = 0; i < n; ++i) {
                Particle p;
                p.sprite = puff;
                p.add = false;
                place(p, 0.0f);
                p.sx = p.sy = big ? m(rand(1.0f, 1.0f)) : m(rand(0.5f, 0.25f));
                p.ds = p.dsy = big ? m(0.0033333334f) : m(0.0016666667f);
                p.rot = rand(0.0f, 360.0f) * kDeg;
                p.drot = rand(-0.2f, 0.4f) * kDeg;
                colour(p, rand(96.0f, 32.0f), 96.0f, 96.0f, big ? rand(16.0f, 32.0f) : rand(16.0f, 16.0f));
                p.fade[3] = -(big ? 0.16f : 0.21333334f) / 128.0f;
                float d[3];
                cone(d, rand(60.0f, 30.0f), rand(0.0f, 360.0f));
                const float v = big ? m(rand(0.02f, 0.01f)) : m(rand(0.053333335f, 0.02f));
                for (int k = 0; k < 3; ++k) p.vel[k] = d[k] * v;
                p.accel = -m(0.00033333333f);
                p.friction = 0.95f;
                p.life = big ? 1.0f : 0.5f;
                spawn(p);
            }
        }
    } else if (!std::strcmp(group, "group-board-land-straight") || !std::strcmp(group, "group-board-quick-jump")) {
        // The boost's burst (the generic-blast Jak II plays with these groups, whose own part is
        // empty): a flash, a ring out over the ground and sparks thrown round.
        Particle f;
        f.sprite = burst;
        place(f, 0.4f);
        f.sx = f.sy = m(2.0f);
        f.ds = f.dsy = m(0.25f);
        colour(f, 160.0f, 200.0f, 255.0f, 128.0f);
        f.fade[3] = -12.0f / 128.0f;
        f.life = 0.2f;
        spawn(f);
        Particle r;
        r.sprite = ring;
        r.flat = true;
        place(r, 0.1f);
        r.sx = r.sy = m(1.0f);
        r.ds = r.dsy = m(0.35f);
        colour(r, 96.0f, 192.0f, 255.0f, 128.0f);
        r.fade[3] = -6.0f / 128.0f;
        r.life = 0.35f;
        spawn(r);
        for (int i = 0; i < 12; ++i) {
            Particle s;
            s.sprite = streak;
            s.along = true;
            colour(s, 160.0f, 200.0f, 255.0f, rand(64.0f, 32.0f));
            float d[3];
            cone(d, rand(60.0f, 30.0f), i * 30.0f + rand(0.0f, 20.0f));
            const float v = m(rand(0.1f, 0.06f));
            place(s, 0.2f);
            for (int k = 0; k < 3; ++k) s.vel[k] = d[k] * v;
            s.sx = m(2.0f);
            s.sy = m(0.06f);
            s.fade[0] = s.fade[1] = -3.0f / 128.0f;
            s.fade[3] = -0.3f / 128.0f;
            s.accel = -m(0.001f);
            s.friction = 0.93f;
            s.life = rand(0.25f, 0.25f);
            spawn(s);
        }
    } else if (!std::strcmp(group, "group-board-spin-attack")) {
        // 436, 437 and 438: a blue ring growing out round him, a flat flash and a soft glow.
        Particle r;
        r.sprite = ring;
        r.flat = true;
        place(r, 0.6f);
        r.sx = m(1.4f);
        r.sy = m(0.9f);
        r.ds = r.dsy = m(0.3f);
        colour(r, 0.0f, rand(128.0f, 128.0f), 255.0f, 255.0f);
        r.fade[3] = -3.1875f / 128.0f;
        r.life = 0.267f;
        spawn(r);
        Particle h;
        h.sprite = glow;
        h.flat = true;
        place(h, 0.6f);
        h.sx = m(12.0f);
        h.sy = m(9.0f);
        h.ds = -m(0.086666666f);
        h.dsy = -m(0.09f);
        colour(h, 0.0f, rand(128.0f, 128.0f), 255.0f, 128.0f);
        h.life = 0.167f;
        spawn(h);
        Particle g;
        g.sprite = glow;
        place(g, 0.8f);
        g.sx = m(rand(3.0f, 0.5f));
        g.sy = m(rand(4.0f, 0.5f));
        colour(g, 0.0f, 128.0f, 255.0f, 64.0f);
        g.fade[3] = -0.4f / 128.0f;
        g.life = 0.267f;
        spawn(g);
    }
}

void BoardFx::shock(const float at[3]) {
    for (int i = 0; i < 3; ++i) {
        Particle s;
        s.sprite = streak;
        s.along = true;
        colour(s, 128.0f, 192.0f, 255.0f, 128.0f);
        float d[3];
        cone(d, rand(0.0f, 180.0f), rand(0.0f, 360.0f));
        std::copy(at, at + 3, s.pos);
        s.pos[2] += m(rand(0.3f, 1.6f));
        for (int k = 0; k < 3; ++k) s.vel[k] = d[k] * m(0.03f);
        s.sx = m(rand(0.6f, 0.8f));
        s.sy = m(0.06f);
        s.life = 0.05f;
        spawn(s);
    }
}

void BoardFx::update(openrac::Mod& mod, float dt) {
    const float k = dt * 60.0f;  // the parts' numbers are a frame's at 60 a second
    float cam[3] = {0.0f, 0.0f, 0.0f};
    const bool have_cam = mod.camera_position(cam);
    std::size_t used[kSprites] = {};
    std::size_t w = 0;
    for (std::size_t i = 0; i < m_parts.size(); ++i) {
        Particle p = m_parts[i];
        p.life -= dt;
        if (p.life <= 0.0f) continue;
        const float fr = std::pow(p.friction, k);
        p.vel[2] += p.accel * k;
        for (int c = 0; c < 3; ++c) {
            p.vel[c] *= fr;
            p.pos[c] += p.vel[c] * k;
        }
        p.sx = std::max(0.0f, p.sx + p.ds * k);
        p.sy = std::max(0.0f, p.sy + (p.dsy != 0.0f ? p.dsy : p.ds) * k);
        p.rot += p.drot * k;
        for (int c = 0; c < 4; ++c) p.rgba[c] = std::max(0.0f, p.rgba[c] + p.fade[c] * k);
        if (p.rgba[3] <= 0.0f || p.sx <= 0.0f) continue;
        m_parts[w++] = p;

        // Its prop: one of its sprite's, made the first time so many are wanted.
        std::vector<int>& props = m_props[p.sprite];
        const std::size_t n = used[p.sprite]++;
        if (n >= props.size()) {
            const int id = mod.prop_create(kMeshes[p.sprite]);
            if (id < 0) continue;
            props.push_back(id);
        }
        // Its axes: across (x), up the sprite (y) and its face (z).
        float x[3], y[3], z[3];
        if (p.flat) {
            const float c = std::cos(p.rot), s = std::sin(p.rot);
            x[0] = c, x[1] = s, x[2] = 0.0f;
            y[0] = -s, y[1] = c, y[2] = 0.0f;
            z[0] = 0.0f, z[1] = 0.0f, z[2] = 1.0f;
        } else {
            z[0] = cam[0] - p.pos[0], z[1] = cam[1] - p.pos[1], z[2] = cam[2] - p.pos[2];
            float l = std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
            if (!have_cam || l < 1e-4f) z[0] = 1.0f, z[1] = 0.0f, z[2] = 0.0f, l = 1.0f;
            for (float& v : z) v /= l;
            float a[3] = {0.0f, 0.0f, 1.0f};
            if (p.along) std::copy(p.vel, p.vel + 3, a);
            // x: along a (up for a sprite, the velocity for a streak) across the face.
            const float d = a[0] * z[0] + a[1] * z[1] + a[2] * z[2];
            for (int c = 0; c < 3; ++c) x[c] = a[c] - d * z[c];
            l = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
            if (l < 1e-6f) x[0] = -z[1], x[1] = z[0], x[2] = 0.0f, l = std::sqrt(x[0] * x[0] + x[1] * x[1]) + 1e-9f;
            for (float& v : x) v /= l;
            if (!p.along && p.rot != 0.0f) {
                const float yv[3] = {z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]};
                const float c = std::cos(p.rot), s = std::sin(p.rot);
                for (int q = 0; q < 3; ++q) x[q] = c * x[q] + s * yv[q];
            }
            y[0] = z[1] * x[2] - z[2] * x[1];
            y[1] = z[2] * x[0] - z[0] * x[2];
            y[2] = z[0] * x[1] - z[1] * x[0];
        }
        // A streak is long along its velocity: its length grows with its speed, as motion blur.
        float sx = p.sx, sy = p.sy;
        if (p.along) {
            const float v = std::sqrt(p.vel[0] * p.vel[0] + p.vel[1] * p.vel[1] + p.vel[2] * p.vel[2]);
            sx = std::clamp(v * 12.0f, 2.0f * p.sy, p.sx);
        }
        const float mat[16] = {x[0] * sx, x[1] * sx, x[2] * sx, 0.0f, y[0] * sy, y[1] * sy, y[2] * sy, 0.0f,
                               z[0],      z[1],      z[2],      0.0f, p.pos[0],  p.pos[1],  p.pos[2],  1.0f};
        const float tint[4] = {p.rgba[0], p.rgba[1], p.rgba[2], std::min(p.rgba[3], 4.0f)};
        mod.prop_pose(props[n], mat, tint, p.add ? OPENRAC_PROP_ADD : OPENRAC_PROP_ALPHA);
        mod.prop_hide(props[n], false);
    }
    m_parts.resize(w);
    for (int s = 0; s < kSprites; ++s) {
        for (std::size_t i = used[s]; i < m_props[s].size(); ++i) mod.prop_hide(m_props[s][i], true);
    }
}

void BoardFx::clear(openrac::Mod& mod) {
    m_parts.clear();
    for (std::vector<int>& props : m_props) {
        for (int id : props) mod.prop_remove(id);
        props.clear();
    }
}

} // namespace rac1
