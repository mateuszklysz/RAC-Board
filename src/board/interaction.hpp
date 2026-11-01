// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// What the board's code reaches outside the target's movement and collision: the other processes
// (attacks, the sidekick, the camera, pickups), the game's state (features, health, water) and the
// target's states off the board (target-falling, target-hit-ground, ...). The adapter (src/rac1)
// implements it on Ratchet & Clank; the tests on stubs. Each method names the GOAL it stands for
// (OpenGOAL, jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
#pragma once

#include <cstdint>

#include "jak/basic_types.hpp"

namespace jak {

struct Target;
struct StateArgs;
enum class StateId : uint8_t;

// A process as GOAL's handle: 0 is #f.
using Handle = uint64_t;

// attack-mask (game-h.gc): which fields of an attack-info are set.
namespace attack_mask {
constexpr uint32_t trans = 1u << 0, vector = 1u << 1, intersection = 1u << 2, attacker = 1u << 3,
                   attack_time = 1u << 4, invinc_time = 1u << 5, mode = 1u << 6, shove_back = 1u << 7,
                   shove_up = 1u << 8, speed = 1u << 9, dist = 1u << 10, control = 1u << 11,
                   angle = 1u << 12, rotate_to = 1u << 13, prev_state = 1u << 14, id = 1u << 15,
                   count = 1u << 16, penetrate_using = 1u << 17, attacker_velocity = 1u << 18,
                   damage = 1u << 19, shield_damage = 1u << 20, knock = 1u << 21, test = 1u << 22;
}

// attack-info (game-h.gc).
struct AttackInfo {
    Vec trans;
    Vec vector;
    Vec attacker_velocity;
    Vec intersection;
    Handle attacker = 0;
    TimeFrame attack_time = 0;
    TimeFrame invinc_time = 0;
    uint32_t mask = 0;  // attack_mask bits
    Symbol mode{};
    float shove_back = 0.0f;
    float shove_up = 0.0f;
    float speed = 0.0f;
    float dist = 0.0f;
    float control = 0.0f;
    Symbol angle{};
    float rotate_to = 0.0f;
    uint32_t id = 0;
    uint32_t count = 0;
    uint64_t penetrate_using = 0;
    float damage = 0.0f;
    float shield_damage = 0.0f;
    int knock = 0;  // knocked-type
    bool test = false;
};

// An event as send-event delivers it (event-message-block): the message and its parameters, typed
// by what the board's handlers read from them.
struct EventMessage {
    Symbol message{};
    Handle from = 0;           // the sender, 0 for the target itself
    int num_params = 0;
    Symbol sym[3] = {};        // symbol parameters (param 0, 1, 2 as the message uses them)
    float f[3] = {};           // float parameters
    bool truthy[3] = {};       // a parameter only tested for #f
    const void* touching = nullptr;   // touching-shapes-entry (param 0 of touched / attack)
    const AttackInfo* attack = nullptr; // attack-info (param 1 of attack / shove)
    Handle proc = 0;           // a process parameter (clone-anim)
};

class Interaction {
  public:
    virtual ~Interaction() = default;

    // ---- the game (game-info, settings, cheats) ----
    virtual bool game_feature_board() = 0;   // (logtest? (-> self game features) (game-feature board))
    virtual bool cheat_mode_debug() { return false; }  // (= *cheat-mode* 'debug): never in play
    virtual bool pause_lock() { return false; }        // *pause-lock*
    virtual bool artist_fix_visible() { return false; } // *artist-fix-visible*
    virtual Symbol game_mode() { return Symbol("play"); } // (-> self game mode)
    virtual bool demo_p() { return false; }             // (demo?)
    virtual uint32_t next_attack_id() = 0;   // (+! (-> *game-info* attack-id) 1), the new id
    virtual float health() = 0;              // (-> self fact health)
    virtual void pickup_health(float amount) = 0; // pickup-collectable! health
    virtual void set_hit_time() {}           // (set-time! (-> self game hit-time))
    virtual TimeFrame gun_time() { return -1000000; } // (-> self gun gun-time)
    // (-> self fact trick-point-duration) nonzero, and (-> self fact trick-point-pickup-time).
    virtual bool trick_points_enabled() { return true; }
    virtual TimeFrame trick_point_pickup_time() { return 0; }
    virtual TimeFrame game_clock_frame_counter() = 0; // (-> *display* game-clock frame-counter)
    // flush-trick-list's sends: 'notify 'trick-point / 'trick-flush to the notify handle, and the
    // 'get-pickup trick-point to the target. Defaults do nothing.
    virtual void notify_trick_point(BoardTricks) {}
    virtual void notify_trick_flush(float, const BoardTricks*, int) {}
    virtual void get_pickup_trick_point(float) {}
    // set-setting! / remove-setting! (settings.gc), by name; the values are the board's numbers.
    virtual void set_setting(const char*, Symbol, float) {}
    virtual void remove_setting(const char*) {}

    // ---- water (water-h.gc) ----
    virtual uint32_t water_flags() { return 0; }  // (-> self water flags), water-flags bits
    virtual float water_height() { return 0.0f; } // (-> self water height)
    virtual float water_surface_height() { return 0.0f; }
    virtual void water_clear_jump_out() {}        // (logclear! (-> self water flags) jump-out)

    // ---- the body (node-list joints) and the camera ----
    // (-> self node-list data i bone transform): the joint's world matrix (rows), for joint 3 (the
    // main joint: vector 2 is the facing) and 25 (the board joint).
    virtual Mat joint_transform(int joint) = 0;
    virtual Vec get_trans(int joint) = 0;          // (get-trans self i)
    // The camera, for target-board-halfpipe-trans: *camera* exists, camera-master-method-15 and -14,
    // (-> *camera* local-down); false when there is no camera.
    virtual bool camera_vectors(Vec&, Vec&, Vec&) { return false; }
    virtual void camera_relative_position(const Vec*) {} // (send-event *camera* 'relative-position)
    virtual void sidekick_matrix(Symbol) {}       // (send-event sidekick 'matrix mode)
    // joint-points (logic-target.gc): the excitement, neck and upper-body blends, the wade surface
    // and the joint positions the target exposes, all off the skeleton and water: the adapter's.
    virtual void joint_points(Target&) {}
    virtual void cpad_buzz(int, int, TimeFrame) {} // cpad-set-buzz!
    virtual void spawn_blast(const Vec&, const Quat&) {} // the manipy skel-generic-blast at a point

    // ---- other processes ----
    // touched with danger-mode on: prims-touching? (mask 1920) then target-send-attack; true when
    // the attack landed.
    virtual bool prims_touching(const void* touching, uint32_t mask) = 0;
    virtual bool target_send_attack(Handle to, Symbol mode, const void* touching, uint32_t id,
                                    uint64_t count, uint64_t penetrate) = 0;
    // touched otherwise: send 'attack (static-attack-info id, mode 'board, penetrate-using) to the
    // toucher; its reply.
    virtual bool send_board_attack(Handle to, const void* touching, uint32_t id, uint64_t penetrate) = 0;
    // target-standard-event-handler / target-generic-event-handler (target-handler.gc).
    virtual Symbol standard_event_handler(Target& self, const EventMessage& e) = 0;
    virtual Symbol generic_event_handler(Target& self, const EventMessage& e) = 0;
    // (-> (handle->process h) root trans): where a process is, false for none (combine!).
    virtual bool process_trans(Handle, Vec&) { return false; }
    virtual void shock_effect() {}  // the 'shock modes' lightning (process-drawable-shock-effect)
    virtual void burn_effect() {}   // process-drawable-burn-effect, for a burn or lava hit

    // ---- the target's states off the board ----
    // The board's code leaves to one of the target's own states (target-falling, target-hit-ground,
    // target-jump, target-stance, target-grab, target-death, target-hit): the board's exit has run,
    // the adapter takes over from here.
    virtual void go_target_state(Target& self, StateId state, const StateArgs& args) = 0;
    // target-clone-anim's enter and exit, clone-anim (story scenes only).
    virtual void clone_anim_enter(Target&, Handle) {}
    virtual void clone_anim_exit(Target&) {}
    virtual void spawn_grenade(Target&, Handle) {}  // target-board-grenade's projectile
    virtual bool grenade_target_trans(Handle, Vec&, Quat&, float&) { return false; } // pegasus
    virtual void send_exit(Handle) {}               // (send-event proc 'exit)
};

} // namespace jak
