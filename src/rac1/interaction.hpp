// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The rest of the game as the board's code reaches it (jak::Interaction): Ratchet's health and
// state, the game's hits on him while he rides, and his attacks on its creatures.
#pragma once

#include <map>

#include <openrac/mod.hpp>

#include "board/interaction.hpp"
#include "jak/target.hpp"

namespace rac1 {

class Interaction : public jak::Interaction {
  public:
    // The ride's target (null between rides).
    jak::Target* target = nullptr;
    // Ratchet's state as he got on (openrac_pose.state), for the water test.
    int hero_state = 0;

    bool game_feature_board() override { return true; }
    uint32_t next_attack_id() override { return ++m_attack_id; }
    float health() override;
    void pickup_health(float amount) override;
    jak::TimeFrame game_clock_frame_counter() override;
    uint32_t water_flags() override;
    jak::Mat joint_transform(int joint) override;
    jak::Vec get_trans(int joint) override;
    void joint_points(jak::Target& t) override;
    bool camera_vectors(jak::Vec& view_flat, jak::Vec& target_move, jak::Vec& local_down) override;
    void camera_relative_position(const jak::Vec* v) override;
    bool prims_touching(const void* touching, uint32_t mask) override;
    bool target_send_attack(jak::Handle to, jak::Symbol mode, const void* touching, uint32_t id, uint64_t count,
                            uint64_t penetrate) override;
    bool send_board_attack(jak::Handle to, const void* touching, uint32_t id, uint64_t penetrate) override;
    jak::Symbol standard_event_handler(jak::Target&, const jak::EventMessage&) override { return {}; }
    jak::Symbol generic_event_handler(jak::Target&, const jak::EventMessage&) override { return {}; }
    bool process_trans(jak::Handle h, jak::Vec& out) override;
    void shock_effect() override;
    void go_target_state(jak::Target& t, jak::StateId state, const jak::StateArgs& args) override;

    // Whether the board's code left for one of the target's own states (the ride is over).
    bool left = false;

  private:
    uint32_t m_attack_id = 0;
};

Interaction& interaction();

// on_moby_hit's hook: the game's hits on Ratchet while he rides, kept for the board.
int on_moby_hit(void* user, const openrac_hit* hit);
// Gives the board the hit the game posted on Ratchet since the last frame, if any.
bool deliver_hit(jak::Target& t);
// Sends 'touched for each creature he rides into (at most once each half second).
void touch_creatures(jak::Target& t);

// Where the target is drawn: its trans lifted by the draw offset (the cushion and the shock).
jak::Vec drawn_trans(const jak::Target& t);

} // namespace rac1
