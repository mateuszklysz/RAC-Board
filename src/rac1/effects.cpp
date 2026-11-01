// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The combo line follows render-combo of OpenGOAL's PC port (target-board.gc; jak-project, ISC
// License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md), and the trick names are its
// English texts (trick-board-*).
#include "rac1/effects.hpp"

#include <cmath>

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
    if (m_points > 0.0f) line += "    " + std::to_string(static_cast<int>(m_points));
    mod().play_notice(line);
}

void Effects::update() {
    m_fx.update(mod(), frame_seconds());
    if (m_shown > 0.0f) {
        m_shown -= frame_seconds();
        if (m_shown <= 0.0f && !m_in_combo) mod().play_notice("");
    }
}

} // namespace rac1
