// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/interaction.hpp"

#include <algorithm>
#include <string>

#include "board/board.hpp"
#include "board/runtime.hpp"
#include "jak/math/math.hpp"
#include "jak/movement.hpp"
#include "openrac/games/rac1.hpp"
#include "rac1/camera.hpp"
#include "rac1/context.hpp"
#include "rac1/convert.hpp"
#include "rac1/fx/effects.hpp"

namespace rac1 {

namespace {

// Ratchet & Clank's hero states in water (HeroStateId): no board there, as under water in Jak II.
bool in_water(int state) {
    return state == 0x12 || (state >= 0x33 && state <= 0x37) || state == 0x6A || state == 0x75 || state == 0x7F;
}

// The game's own hit on a creature, as the wrench's: the bit every enemy reads (0x10000).
bool hit_creature(const jak::Target& t, jak::Handle to) {
    if (to == 0) return false;
    openrac_hit h{};
    h.size = sizeof h;
    h.victim = static_cast<int32_t>(to - 1);
    h.attacker = mod().figure_slot(-1);
    h.mask = 0x10000u;
    h.damage = 1.0f;
    to_game_point(t.control.trans, h.position);
    to_game_dir(t.control.transv, h.velocity);
    for (float& v : h.velocity) v /= kUnit;
    return mod().moby_hit(h);
}

// The last hit the game posted on Ratchet while he rides (his own update, which reads it, does not
// run then).
struct PendingHit {
    bool pending = false;
    int attacker = -1;
    float position[3]{};
    float damage = 1.0f;
};
PendingHit g_hit;

// When each creature was last touched (by slot), so each touch counts once a half second.
std::map<int, jak::TimeFrame> g_touched;

} // namespace

Interaction& interaction() {
    static Interaction io;
    return io;
}

// Ratchet's own health: a hit on the board takes it off as the game's own hits do.
float Interaction::health() {
    openrac_pose pose{};
    return mod().hero_pose(pose) ? static_cast<float>(pose.health) : 1.0f;
}

void Interaction::pickup_health(float amount) {
    if (amount >= 0.0f) return;
    static const auto take =
        mod().function<openrac::rac1::Hero_TakeDamage::fn>(openrac::rac1::Hero_TakeDamage::name);
    // Hero_TakeDamage takes one hit point a call; the instant deaths take them all.
    const int n = static_cast<int>(std::min(-amount, 100.0f) + 0.5f);
    for (int i = 0; i < std::max(n, 1) && take; ++i) take(1);
}

jak::TimeFrame Interaction::game_clock_frame_counter() { return ctx().clock; }

uint32_t Interaction::water_flags() {
    return in_water(hero_state) ? jak::waterflag::under_water : 0u;
}

// The body's matrix at its drawn place (rows: right, up, forward, trans), for every joint but 3.
// Joint 3 is Jak's board joint: the spin's twist turns it, and its forward is Jak's side-on stance,
// the facing less 15109.688 (the spin's end turns it back by that much and takes it as the facing).
// Without the two, every spin's end turned him 83 degrees: R1 taps in the air snapped him round.
jak::Mat Interaction::joint_transform(int joint) {
    jak::Mat m;
    jak::quaternion_to_matrix(m, target->control.quat);
    if (joint == 3) {
        const float twist = target->board ? target->board->troty : 0.0f;
        for (int r = 0; r < 3; ++r) jak::vector_rotate_y(m.v[r], jak::Vec(m.v[r]), twist - 15109.688f);
    }
    m.v[3] = drawn_trans(*target);
    return m;
}

jak::Vec Interaction::get_trans(int joint) {
    jak::Vec p = target->control.trans;
    if (joint == 6) p.y = p.y + 0.8f * kUnit; // joint 6 is the chest, 0 the root
    return p;
}

// joint-points (logic-target.gc), the parts the board reads: the head and the midpoint of the hands,
// which the edge search starts from (an edge counts within 2 m of it). Jak's skeleton is not the
// mod's, so the hands are where Jak holds them on the board: 0.75 m above his feet, centred.
void Interaction::joint_points(jak::Target& t) {
    jak::Mat m;
    jak::quaternion_to_matrix(m, t.control.quat);
    const auto local = [&](const jak::Vec& v) {
        jak::Vec out;
        jak::vector_matrix_mul(out, jak::Vec{v.x, v.y, v.z, 0.0f}, m);
        return out;
    };
    t.control.ctrl_to_head_offset = local(jak::target_bank.head_offset);
    t.control.ctrl_to_hands_offset = local(jak::Vec{0.0f, jak::meters(0.75f), 0.0f, 0.0f});
    const jak::Vec& p = t.control.trans;
    const jak::Vec& h = t.control.ctrl_to_hands_offset;
    t.control.midpoint_of_hands = jak::Vec{p.x + h.x, p.y + h.y, p.z + h.z, 1.0f};
}

// A creature touched (touch_creatures): its slot + 1 is the handle.
bool Interaction::prims_touching(const void* touching, uint32_t) { return touching != nullptr; }

bool Interaction::target_send_attack(jak::Handle to, jak::Symbol, const void*, uint32_t, uint64_t, uint64_t) {
    return hit_creature(*target, to);
}

bool Interaction::send_board_attack(jak::Handle to, const void*, uint32_t, uint64_t) {
    return hit_creature(*target, to);
}

bool Interaction::process_trans(jak::Handle h, jak::Vec& out) {
    if (h == 0 || !mod().has_world()) return false;
    for (const openrac_moby& m : mod().moby_list()) {
        if (static_cast<jak::Handle>(m.slot) + 1 != h) continue;
        out = to_jak_point(m.position);
        return true;
    }
    return false;
}

void Interaction::shock_effect() {
    float p[3];
    to_game_point(target->control.trans, p);
    effects().shock(p);
}

// The board's code left for one of the target's own states: the game's update takes over.
void Interaction::go_target_state(jak::Target&, jak::StateId, const jak::StateArgs&) { left = true; }

int on_moby_hit(void*, const openrac_hit* hit) {
    if (hit == nullptr) return 0;
    trace("hit " + std::to_string(hit->victim) + " by " + std::to_string(hit->attacker) + " mask " +
          std::to_string(hit->mask) + " damage " + std::to_string(hit->damage));
    if (interaction().target == nullptr || hit->victim < 0 || hit->victim != mod().figure_slot(-1)) return 0;
    if (!g_hit.pending || hit->damage > g_hit.damage) {
        g_hit.pending = true;
        g_hit.attacker = hit->attacker;
        std::copy(hit->position, hit->position + 3, g_hit.position);
        g_hit.damage = hit->damage > 0.0f ? hit->damage : 1.0f;
    }
    return 1; // taken: the board's hit state deals with it
}

// The hit as Jak II's enemies send theirs: 'attack with an attack-info of a new id, its mode and
// damage, and a vector that shoves him away from where it came from.
void deliver_hit(jak::Target& t) {
    if (!g_hit.pending) return;
    g_hit.pending = false;
    namespace am = jak::attack_mask;
    jak::AttackInfo a;
    a.id = interaction().next_attack_id();
    a.mode = jak::Symbol("generic");
    a.damage = g_hit.damage;
    a.mask = am::id | am::mode | am::damage;
    if (g_hit.attacker >= 0) {
        a.attacker = static_cast<jak::Handle>(g_hit.attacker) + 1;
        a.mask |= am::attacker;
    }
    const jak::Vec from = to_jak_point(g_hit.position);
    jak::Vec d{t.control.trans.x - from.x, 0.0f, t.control.trans.z - from.z, 0.0f};
    if (d.x * d.x + d.z * d.z > 1.0f) {
        a.vector = d;
        jak::vector_xz_normalize(a.vector, 6144.0f); // the board's own shove (target-board-hit)
        a.vector.y = 4915.2f;
        a.mask |= am::vector;
    }
    jak::EventMessage e;
    e.message = jak::Symbol("attack");
    e.from = a.attacker;
    e.num_params = 2;
    e.attack = &a;
    jak::send_event(t, e);
}

// Each creature he rides into gets the board's 'touched, as Jak II's collision sends it: a spin or
// a kick attacks it (target-send-attack), the board alone knocks it (send-board-attack).
void touch_creatures(jak::Target& t) {
    float hero[3];
    to_game_point(t.control.trans, hero);
    const int self = mod().figure_slot(-1);
    for (const openrac_moby& m : mod().moby_list()) {
        if (!(m.flags & OPENRAC_MOBY_CREATURE) || (m.flags & OPENRAC_MOBY_DEAD) || m.slot == self) continue;
        const float dx = m.position[0] - hero[0], dy = m.position[1] - hero[1], dz = m.position[2] - hero[2];
        if (dx * dx + dy * dy > 1.2f * 1.2f || dz < -1.0f || dz > 2.0f) continue;
        auto it = g_touched.find(m.slot);
        if (it != g_touched.end() && ctx().clock - it->second < jak::seconds(0.5)) continue;
        g_touched[m.slot] = ctx().clock;
        static const int kTouching = 0; // a touching-shapes-entry the adapter does not read
        jak::EventMessage e;
        e.message = jak::Symbol("touched");
        e.from = static_cast<jak::Handle>(m.slot) + 1;
        e.num_params = 1;
        e.touching = &kTouching;
        jak::send_event(t, e);
    }
}

jak::Vec drawn_trans(const jak::Target& t) {
    jak::Vec p = t.control.trans;
    p.y = p.y + t.control.draw_offset.y;
    return p;
}

bool Interaction::camera_vectors(jak::Vec& view_flat, jak::Vec& target_move, jak::Vec& local_down) {
    return rac1::camera_vectors(view_flat, target_move, local_down);
}

void Interaction::camera_relative_position(const jak::Vec* v) { camera_relative(v); }

} // namespace rac1
