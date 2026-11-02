// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// What the board shows and plays: the mod's sounds by Jak II's names, the particles (board_fx) and
// the trick combo line (the PC port's render-combo).
#pragma once

#include <cstdint>
#include <map>
#include <string>

#include "jak/host.hpp"
#include "rac1/fx/board_fx.hpp"

namespace rac1 {

class Effects : public jak::Effects {
  public:
    uint32_t sound_play(const char* name, float volume, float pitch) override;
    void sound_loop(const char* name, uint32_t id, float volume, float pitch) override;
    void sound_stop(uint32_t id) override;
    void particles(const char* group, const jak::Vec& at, float rate) override;
    void trick(jak::BoardTricks trick, float points) override;
    void combo_end() override;
    // The ride is over: a combo still running ends (its line fades out).
    // The pause menu: the board's own sounds stop (its loops start again with the next frame that
    // plays them); Ratchet's, the game's, pause with the game.
    void pause_sounds();
    void ride_over() {
        if (m_in_combo) combo_end();
    }

    // One frame: the particles move, the combo line goes after a while.
    void update();
    // The ride is over: every sound stops.
    void stop_sounds();
    // A new level: the particles go with its props.
    void clear_particles();
    // The lightning of a shock hit, at `at` (game units).
    void shock(const float at[3]);

    float yaw = 0.0f; // the board's heading, for the parts that lie along it

  private:
    void show_combo();
    void draw_combo();
    int sound(const char* name);

    BoardFx m_fx;
    std::map<std::string, int> m_sounds;
    std::map<uint32_t, uint32_t> m_voices;
    std::map<uint32_t, int> m_hero_voices; // the game's own sounds on Ratchet, by the same ids
    uint32_t m_next = 0x80000000u; // apart from new-sound-id's ids (0x10000 up)
    // The combo as add-trick-to-combo! keeps it: up to 16 tricks and their points.
    jak::BoardTricks m_tricks[16] = {};
    int m_count = 0;
    float m_points = 0.0f;
    bool m_in_combo = false;
    float m_shown = 0.0f; // seconds the line stays after the combo ends
    std::string m_line;   // the tricks, as render-combo writes them
    std::string m_score;  // and the points
    bool m_drawn = false; // whether the HUD has the lines now
};

Effects& effects();

} // namespace rac1
