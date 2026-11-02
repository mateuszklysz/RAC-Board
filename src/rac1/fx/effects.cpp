// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The combo line follows render-combo of OpenGOAL's PC port (target-board.gc; jak-project, ISC
// License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md), and the trick names are its
// English texts (trick-board-*).
#include "rac1/fx/effects.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "rac1/context.hpp"
#include "rac1/convert.hpp"

namespace rac1 {

namespace {

// The names in board-tricks order (text ids trick-board-none and on).
constexpr const char* kTrickNames[] = {"",          "Spin",       "Boost",      "Flip",     "Rail Grind",
                                       "Rail Jump", "Nosegrab",   "Method",     "Board Spin", "Board Flip",
                                       "Noseflip",  "Kickflip",   "Jump",       "Duck Jump",  "Quick Jump"};
constexpr int kTrickNameCount = sizeof(kTrickNames) / sizeof(kTrickNames[0]);

// How long the last combo stays on screen once it is over.
constexpr float kComboShown = 3.0f;

// GOAL's pitch-mod: 1024 an octave.
float pitch_ratio(float pitch) {
    const float r = std::exp2(pitch / 1024.0f);
    return r < 0.25f ? 0.25f : (r > 4.0f ? 4.0f : r);
}

} // namespace

Effects& effects() {
    static Effects e;
    return e;
}

int Effects::sound(const char* name) {
    auto it = m_sounds.find(name);
    if (it != m_sounds.end()) return it->second;
    const int s = mod().sound_load(std::string("assets/sounds/") + name + ".wav");
    m_sounds[name] = s;
    return s;
}

uint32_t Effects::sound_play(const char* name, float volume, float pitch) {
    const int s = sound(name);
    if (s < 0) return 0;
    const uint32_t voice = mod().sound_play(s, volume, pitch_ratio(pitch), false);
    if (voice == 0) return 0;
    const uint32_t id = m_next++;
    m_voices[id] = voice;
    return id;
}

void Effects::sound_loop(const char* name, uint32_t id, float volume, float pitch) {
    auto it = m_voices.find(id);
    if (it != m_voices.end() && mod().sound_set(it->second, volume, pitch_ratio(pitch))) return;
    const int s = sound(name);
    if (s < 0) return;
    const uint32_t voice = mod().sound_play(s, volume, pitch_ratio(pitch), true);
    if (voice != 0) m_voices[id] = voice;
}

void Effects::sound_stop(uint32_t id) {
    auto it = m_voices.find(id);
    if (it == m_voices.end()) return;
    mod().sound_stop(it->second);
    m_voices.erase(it);
}

void Effects::stop_sounds() {
    for (const auto& v : m_voices) mod().sound_stop(v.second);
    m_voices.clear();
}

void Effects::particles(const char* group, const jak::Vec& at, float rate) {
    float p[3];
    to_game_point(at, p);
    m_fx.launch(group, p, yaw, rate);
}

void Effects::shock(const float at[3]) { m_fx.shock(at); }

void Effects::clear_particles() { m_fx.clear(mod()); }

// add-trick-to-combo!: a new combo starts over, then the trick and its points go on the list.
void Effects::trick(jak::BoardTricks t, float points) {
    if (!m_in_combo) {
        m_in_combo = true;
        m_count = 0;
        m_points = 0.0f;
    }
    if (m_count < 16) {
        m_tricks[m_count++] = t;
        m_points += points;
    }
    show_combo();
}

// end-combo!: the line stays, as the PC port leaves it, until the next combo (or a while).
void Effects::combo_end() {
    m_in_combo = false;
    m_shown = kComboShown;
}

// render-combo: each trick once, in the order done, " x N" when done more than once, then the points.
void Effects::show_combo() {
    if (!settings().trick_display || m_count == 0) return;
    std::string line;
    bool printed[kTrickNameCount] = {};
    for (int i = 0; i < m_count; ++i) {
        const int t = static_cast<int>(m_tricks[i]);
        if (t <= 0 || t >= kTrickNameCount || printed[t]) continue;
        printed[t] = true;
        int times = 0;
        for (int j = 0; j < m_count; ++j) times += m_tricks[j] == m_tricks[i];
        if (!line.empty()) line += " + ";
        line += kTrickNames[t];
        if (times > 1) line += " x " + std::to_string(times);
    }
    // The HUD's line holds 63 characters: a long combo keeps its last tricks.
    constexpr std::size_t kMost = 60;
    if (line.size() > kMost) {
        std::size_t cut = line.find(" + ", line.size() - kMost + 4);
        line = "... " + (cut == std::string::npos ? line.substr(line.size() - kMost + 4) : line.substr(cut + 1));
    }
    m_line = line;
    m_score = m_points > 0.0f ? std::to_string(static_cast<int>(m_points)) : std::string();
    draw_combo();
}

// The combo in the game's HUD, in its own font, low in the middle of the screen as Jak II has it:
// the tricks, the points under them, fading out at the end of the time the line stays.
void Effects::draw_combo() {
    const bool shown = settings().trick_display && (m_in_combo || m_shown > 0.0f) && !m_line.empty();
    if (!shown) {
        if (m_drawn) mod().hud_text_set({});
        m_drawn = false;
        return;
    }
    const float fade = m_in_combo ? 1.0f : std::min(1.0f, m_shown / 0.5f);
    const auto a = static_cast<uint8_t>(255.0f * fade + 0.5f);
    std::vector<openrac_hud_text> lines;
    const auto add = [&](const std::string& text, float y, float scale, uint8_t r, uint8_t g, uint8_t b) {
        if (text.empty()) return;
        openrac_hud_text l{};
        std::snprintf(l.text, sizeof l.text, "%s", text.c_str());
        l.x = 0.5f;
        l.y = y;
        l.scale = scale;
        l.rgba[0] = r, l.rgba[1] = g, l.rgba[2] = b, l.rgba[3] = a;
        l.flags = OPENRAC_HUD_SHADOW;
        lines.push_back(l);
    };
    add(m_line, 0.76f, 0.8f, 255, 255, 255);
    add(m_score, 0.83f, 1.25f, 255, 190, 40);
    m_drawn = mod().hud_text_set(lines);
}

void Effects::update() {
    m_fx.haze = settings().heat_haze && mod().has_camera();
    m_fx.update(mod(), frame_seconds());
    if (m_shown > 0.0f) m_shown = std::max(0.0f, m_shown - frame_seconds());
    if (m_drawn || m_in_combo || m_shown > 0.0f) draw_combo();
}

} // namespace rac1
