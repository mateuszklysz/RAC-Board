// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
//
// RAC-Board in Ratchet & Clank: the adapter between the game and the board's logic (src/jak and
// src/board, Jak II's code ported from OpenGOAL). While the hero rides, the mod takes his update
// (rac1.player.update): it runs one frame of Jak II's target on the board, then puts Ratchet where
// the target is (hero_set_pose), in his riding pose (ride_pose.hpp, hero_set_joints), and the board
// prop under him. Off the board the game's own update runs, untouched.
//
// What it gives the board: the level's collision triangles (World), the mod's own sounds and the
// trick names (Effects), the pad with the buttons moved to fit Ratchet & Clank, and a camera matrix
// from the game's camera. Units and axes: convert.hpp.
//
// Settings (openrac-mod.json): board_button, board_key, swap_l1_r1, trick_display, trace.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "board/board.hpp"
#include "board/core.hpp"
#include "board/interaction.hpp"
#include "board/runtime.hpp"
#include "jak/collide.hpp"
#include "jak/cpad.hpp"
#include "jak/host.hpp"
#include "jak/math.hpp"
#include "jak/movement.hpp"
#include "jak/target.hpp"
#include "openrac/games/rac1.hpp"
#include "openrac/mod.hpp"
#include "rac1/convert.hpp"
#include "rac1/ride_pose.hpp"

namespace {

using jak::Vec;

openrac::Mod* g_mod = nullptr;

// ---- settings ----
enum class Toggle { down, l2r2, r2 };
struct Settings {
    Toggle toggle = Toggle::down;
    std::vector<int> keys;  // the keyboard binds that toggle the board
    bool swap_l1_r1 = true;
    bool trick_display = true;
    bool trace = false;
};
Settings g_set;

// ---- the pad, as on_pad last saw port 0 (buttons active high, pad-buttons order) ----
struct PadState {
    uint16_t held = 0;
    uint8_t sticks[4] = {128, 128, 128, 128};  // right x, y, left x, y
    uint8_t pressure[12] = {};
    bool valid = false;
};
PadState g_pad;
bool g_toggle_was_held = false;

bool toggle_held(uint16_t held) {
    switch (g_set.toggle) {
    case Toggle::down: return held & jak::pad::down;
    case Toggle::r2: return held & jak::pad::r2;
    case Toggle::l2r2: return (held & jak::pad::l2) && (held & jak::pad::r2);
    }
    return false;
}
uint16_t toggle_bits() {
    switch (g_set.toggle) {
    case Toggle::down: return jak::pad::down;
    case Toggle::r2: return jak::pad::r2;
    case Toggle::l2r2: return jak::pad::l2 | jak::pad::r2;
    }
    return 0;
}

bool playing() {
    const openrac_play_status s = g_mod->play_status();
    return s.phase == OPENRAC_PLAY_PLAYING;
}

// ---- World: the level's collision, in Jak II's units ----
class RacWorld : public jak::World {
  public:
    int triangles(const Vec& lo, const Vec& hi, jak::CollideTri* out, int max) override {
        // The swap keeps each axis's min a min, so the box maps corner to corner.
        const float glo[3] = {lo.x / rac1::kUnit, lo.z / rac1::kUnit, lo.y / rac1::kUnit};
        const float ghi[3] = {hi.x / rac1::kUnit, hi.z / rac1::kUnit, hi.y / rac1::kUnit};
        const std::vector<openrac_collision_triangle> tris = g_mod->collision_triangles(glo, ghi, max);
        int n = 0;
        for (const openrac_collision_triangle& t : tris) {
            if (n >= max) break;
            jak::CollideTri& c = out[n++];
            // The game's corners turn the other way round from Jak II's: its normal, (b - a) x (c - a),
            // faces into the solid side. Mirrored into Jak II's axes (convert.hpp) the same order
            // gives the outward normal by Jak II's cross product, and the normal is turned outward.
            c.vertex[0] = rac1::to_jak_point(t.corners[0]);
            c.vertex[1] = rac1::to_jak_point(t.corners[1]);
            c.vertex[2] = rac1::to_jak_point(t.corners[2]);
            const float out_normal[3] = {-t.normal[0], -t.normal[1], -t.normal[2]};
            c.normal = rac1::to_jak_dir(out_normal);
            // Every surface is ground to Jak II's code: it tells walls from floors by their slope.
            c.pat = jak::PatSurface{};
        }
        return n;
    }
};

// ---- Effects: the mod's sounds by Jak II's names, and the trick names ----
const char* trick_name(jak::BoardTricks t) {
    using jak::BoardTricks;
    switch (t) {
    case BoardTricks::board_spin: return "Spin";
    case BoardTricks::board_boost: return "Boost";
    case BoardTricks::board_flip: return "Flip";
    case BoardTricks::board_rail: return "Rail";
    case BoardTricks::board_rail_jump: return "Rail jump";
    case BoardTricks::board_nosegrab: return "Nose grab";
    case BoardTricks::board_method: return "Method";
    case BoardTricks::board_board_spin: return "Board spin";
    case BoardTricks::board_board_flip: return "Board flip";
    case BoardTricks::board_noseflip: return "Nose flip";
    case BoardTricks::board_kickflip: return "Kick flip";
    case BoardTricks::board_jump: return "Jump";
    case BoardTricks::board_duck_jump: return "Duck jump";
    case BoardTricks::board_quick_jump: return "Quick jump";
    default: return "Trick";
    }
}

class RacEffects : public jak::Effects {
  public:
    uint32_t sound_play(const char* name, float volume, float pitch) override {
        const int s = sound(name);
        if (s < 0) return 0;
        const uint32_t voice = g_mod->sound_play(s, volume, ratio(pitch), false);
        if (voice == 0) return 0;
        const uint32_t id = m_next++;
        m_voices[id] = voice;
        return id;
    }
    void sound_loop(const char* name, uint32_t id, float volume, float pitch) override {
        auto it = m_voices.find(id);
        if (it != m_voices.end() && g_mod->sound_set(it->second, volume, ratio(pitch))) return;
        const int s = sound(name);
        if (s < 0) return;
        const uint32_t voice = g_mod->sound_play(s, volume, ratio(pitch), true);
        if (voice != 0) m_voices[id] = voice;
    }
    void sound_stop(uint32_t id) override {
        auto it = m_voices.find(id);
        if (it == m_voices.end()) return;
        g_mod->sound_stop(it->second);
        m_voices.erase(it);
    }
    void particles(const char*, const Vec&) override {}  // no particles yet
    void trick(jak::BoardTricks t, float points) override {
        m_combo += points;
        if (g_set.trick_display) {
            g_mod->play_notice(std::string(trick_name(t)) + "  " + std::to_string(static_cast<int>(points)));
        }
    }
    void combo_end() override {
        if (g_set.trick_display && m_combo > 0.0f) {
            g_mod->play_notice("Combo  " + std::to_string(static_cast<int>(m_combo)));
        }
        m_combo = 0.0f;
    }
    void stop_all() {
        for (const auto& v : m_voices) g_mod->sound_stop(v.second);
        m_voices.clear();
        m_combo = 0.0f;
    }

  private:
    // GOAL's pitch-mod: 1024 an octave.
    static float ratio(float pitch) {
        const float r = std::exp2(pitch / 1024.0f);
        return r < 0.25f ? 0.25f : (r > 4.0f ? 4.0f : r);
    }
    int sound(const char* name) {
        auto it = m_sounds.find(name);
        if (it != m_sounds.end()) return it->second;
        const int s = g_mod->sound_load(std::string("assets/sounds/") + name + ".wav");
        m_sounds[name] = s;
        return s;
    }
    std::map<std::string, int> m_sounds;
    std::map<uint32_t, uint32_t> m_voices;
    uint32_t m_next = 0x80000000u;  // apart from new-sound-id's ids (0x10000 up)
    float m_combo = 0.0f;
};

// ---- the ride ----
struct Ride {
    std::unique_ptr<jak::Target> t;
    std::unique_ptr<jak::BoardRuntime> rt;
    bool leave = false;
    uint32_t loads = 0;
    jak::StateId traced = jak::StateId::none;
    rac1::RideAnimator look;
    float last_yaw = 0.0f;
    float last_vel[3] = {};
    bool has_yaw = false;
};

RacWorld g_world;
RacEffects g_fx;
Ride g_ride;
int g_prop = -1;
jak::TimeFrame g_clock = 3000;  // the board's clock, on across rides (the 0.5 s between rides)
bool g_pal = false;

bool riding() { return g_ride.t != nullptr; }

// Where the target is drawn: its trans lifted by the draw offset (the cushion and the shock).
Vec drawn_trans(const jak::Target& t) {
    Vec p = t.control.trans;
    p.y = p.y + t.control.draw_offset.y;
    return p;
}

// The target's body matrix at its drawn place (rows: right, up, forward, trans).
jak::Mat body_matrix(const jak::Target& t) {
    jak::Mat m;
    jak::quaternion_to_matrix(m, t.control.quat);
    m.v[3] = drawn_trans(t);
    return m;
}

// ---- Interaction: the rest of the game, as far as Ratchet & Clank has it ----
class RacIo : public jak::Interaction {
  public:
    bool game_feature_board() override { return true; }
    uint32_t next_attack_id() override { return ++m_attack_id; }
    float health() override { return 3.0f; }
    void pickup_health(float) override {}
    jak::TimeFrame game_clock_frame_counter() override { return g_clock; }
    jak::Mat joint_transform(int) override { return body_matrix(*g_ride.t); }
    Vec get_trans(int joint) override {
        Vec p = g_ride.t->control.trans;
        // Joint 6 is the chest, 0 the root.
        if (joint == 6) p.y = p.y + 0.8f * rac1::kUnit;
        return p;
    }
    bool prims_touching(const void*, uint32_t) override { return false; }
    bool target_send_attack(jak::Handle, jak::Symbol, const void*, uint32_t, uint64_t, uint64_t) override {
        return false;
    }
    bool send_board_attack(jak::Handle, const void*, uint32_t, uint64_t) override { return false; }
    jak::Symbol standard_event_handler(jak::Target&, const jak::EventMessage&) override { return {}; }
    jak::Symbol generic_event_handler(jak::Target&, const jak::EventMessage&) override { return {}; }
    // Nothing in Ratchet & Clank attacks the target: the game's own damage reaches Ratchet when
    // he is off the board.
    jak::Symbol target_attacked(jak::Target&, jak::Symbol, const jak::AttackInfo&, jak::Handle, const void*,
                                jak::StateId) override {
        return {};
    }
    jak::AttackInfo* attack_info_rec() override { return &m_rec; }
    jak::AttackInfo* attack_info() override { return &m_info; }
    void attack_info_combine(jak::AttackInfo& d, const jak::AttackInfo& s) override {
        if (s.mask & jak::kAttackMaskVector) {
            d.vector = s.vector;
            d.mask |= jak::kAttackMaskVector;
        }
        if (s.mode) d.mode = s.mode;
    }
    void target_hit_effect(const jak::AttackInfo&) override {}
    bool target_hit_orient(const jak::AttackInfo&, const Vec&) override { return false; }
    jak::Symbol target_hit_push(Vec&, const jak::Mat&, float, float, const jak::AttackInfo&) override { return {}; }
    // The board's code left for one of the target's own states: the game's update takes over.
    void go_target_state(jak::Target&, jak::StateId, const jak::StateArgs&) override { g_ride.leave = true; }

  private:
    uint32_t m_attack_id = 0;
    jak::AttackInfo m_info, m_rec;
};
RacIo g_io;

// The camera's rotation for read-pad: forward from the game's camera to the hero, flat.
jak::Mat camera_matrix(const float hero[3], float hero_yaw) {
    float fx = std::cos(hero_yaw), fy = std::sin(hero_yaw);
    float cam[3];
    if (g_mod->camera_position(cam)) {
        const float dx = hero[0] - cam[0], dy = hero[1] - cam[1];
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len > 0.01f) {
            fx = dx / len;
            fy = dy / len;
        }
    }
    // read-pad turns stick right into minus row 0 (Jak II's camera x points left), so row 0 is the
    // game's left: up x forward, (-fy, fx, 0).
    jak::Mat m;
    jak::matrix_identity(m);
    m.v[0] = Vec{-fy, 0.0f, fx, 0.0f};
    m.v[1] = Vec{0.0f, 1.0f, 0.0f, 0.0f};
    m.v[2] = Vec{fx, 0.0f, fy, 0.0f};
    return m;
}

// The pad as Jak II's board reads it: R&C's buttons moved where the board expects them.
uint16_t jak_buttons(bool toggle_pressed) {
    const uint16_t held = g_pad.held;
    uint16_t b = held & static_cast<uint16_t>(~(jak::pad::r2 | jak::pad::l2 | toggle_bits()));
    if (g_set.swap_l1_r1) {
        b &= static_cast<uint16_t>(~(jak::pad::l1 | jak::pad::r1));
        if (held & jak::pad::r1) b |= jak::pad::l1;  // duck and spin
        if (held & jak::pad::l1) b |= jak::pad::r1;  // grab
    }
    // Jak II gets off the board on R2.
    if (toggle_pressed) b |= jak::pad::r2;
    return b;
}

void hide_prop() {
    if (g_prop >= 0) g_mod->prop_hide(g_prop, true);
}

// What he holds in his hand (the wrench, a weapon or gadget): put away while he rides, as Jak puts
// his gun away. The hand's pieces are found once, as the ride starts, by class and nearness.
std::vector<int> g_hand;

void find_hand(const float hero[3]) {
    g_hand.clear();
    openrac_held held;
    if (!g_mod->has_world() || !g_mod->hero_held(held)) return;
    for (const openrac_moby& m : g_mod->moby_list()) {
        if (m.flags & OPENRAC_MOBY_HIDDEN) continue;
        for (int i = 0; i < held.count; ++i) {
            const openrac_held_piece& p = held.pieces[i];
            if (p.slot != 0 || (p.flags & (OPENRAC_HELD_HIDDEN | OPENRAC_HELD_LOOSE)) || p.oclass != m.oclass) continue;
            const float dx = m.position[0] - hero[0], dy = m.position[1] - hero[1], dz = m.position[2] - hero[2];
            if (dx * dx + dy * dy + dz * dz < 9.0f) g_hand.push_back(m.slot);
            break;
        }
    }
}

void hide_hand(bool hidden) {
    for (int slot : g_hand) g_mod->moby_hide(slot, hidden);
}

void end_ride() {
    g_mod->hero_set_joints({});
    hide_hand(false);
    g_hand.clear();
    g_ride.look.reset();
    g_ride.has_yaw = false;
    g_fx.stop_all();
    hide_prop();
    g_ride.t.reset();
    g_ride.rt.reset();
    g_ride.leave = false;
}

bool start_ride() {
    openrac_pose pose{};
    if (!g_mod->hero_pose(pose)) return false;
    find_hand(pose.position);
    if (g_set.trace) g_mod->info("hand pieces " + std::to_string(g_hand.size()));
    g_ride.t = std::make_unique<jak::Target>();
    g_ride.rt = std::make_unique<jak::BoardRuntime>();
    g_ride.leave = false;
    g_ride.loads = g_mod->play_status().loads;
    g_ride.traced = jak::StateId::none;
    jak::Target& t = *g_ride.t;
    t.world = &g_world;
    t.effects = &g_fx;
    t.display_time_factor = g_pal ? 6.0f : 5.0f;
    t.clock.frame_counter = g_clock;
    jak::movement_init(t, rac1::to_jak_point(pose.position));
    jak::target_collide_init(t);
    t.control.local_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
    t.control.gspot_normal = Vec{0.0f, 1.0f, 0.0f, 1.0f};
    jak::update_rates(t, 1.0f);
    jak::attach(t, *g_ride.rt, g_io);
    jak::Quat q;
    jak::forward_up_nopitch_to_quaternion(q, rac1::yaw_to_jak_forward(pose.rotation[2]), Vec{0, 1, 0, 0});
    t.control.quat = q;
    jak::set_quaternion(t, q);
    jak::rot_to_dir_targ(t);
    Vec v = rac1::to_jak_dir(pose.velocity);
    t.control.transv = Vec{v.x * rac1::kUnit, v.y * rac1::kUnit, v.z * rac1::kUnit, 0.0f};
    // Jak walks onto the board with his ground already found; get-on's hop is timed from it.
    {
        jak::CollideQuery q;
        if (jak::find_ground(t, q, jak::root_prim_collide_with(t) & ~jak::cspec::water, 8192.0f, 81920.0f,
                             1024.0f)) {
            t.control.gspot_pat_surfce = q.best_pat;
        }
    }
    jak::go(t, jak::StateId::target_board_start);
    return true;
}

// One frame of the target on the board.
void tick_ride(bool toggle_pressed) {
    jak::Target& t = *g_ride.t;
    const jak::TimeFrame step = g_pal ? 6 : 5;
    t.clock.old_frame_counter = g_clock;
    g_clock += step;
    t.clock.frame_counter = g_clock;
    t.real_frame_counter += step;
    jak::update_rates(t, 1.0f);

    float hero[3];
    rac1::to_game_point(t.control.trans, hero);
    Vec fwd;
    jak::vector_z_quaternion(fwd, t.control.quat);
    t.camera_local_to_world = camera_matrix(hero, rac1::jak_forward_to_yaw(fwd));

    jak::cpad_set_hw(t.pad, jak_buttons(toggle_pressed), g_pad.sticks[0], g_pad.sticks[1], g_pad.sticks[2],
                     g_pad.sticks[3]);
    std::memcpy(t.pad.abutton, g_pad.pressure, sizeof t.pad.abutton);
    if (g_set.swap_l1_r1) std::swap(t.pad.abutton[jak::kAbL1], t.pad.abutton[jak::kAbR1]);
    jak::CpadService s;
    s.current_time = g_clock;
    s.target_fps = g_pal ? 50 : 60;
    jak::service_cpad(t.pad, s);

    jak::board_tick(t);

    if (g_set.trace && g_ride.rt->state != g_ride.traced) {
        g_ride.traced = g_ride.rt->state;
        g_mod->info(std::string("state ") + jak::state_name(g_ride.traced));
    }
}

constexpr float kTwoPi = 6.2831853f;

// The board's spin (its main joint's twist) turns him with it; Jak's y turn is the game's z turn the
// other way (the axes' mirror).
float board_spin() {
    const jak::BoardRuntime& rt = *g_ride.rt;
    return rt.main.mode == jak::JointModMode::rotate ? -rt.main.twist_y * (kTwoPi / 65536.0f) : 0.0f;
}

void qrot(const float q[4], const float v[3], float out[3]) {
    // v + 2w (u x v) + 2 u x (u x v)
    const float u[3] = {q[0], q[1], q[2]};
    const float c1[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    const float c2[3] = {u[1] * c1[2] - u[2] * c1[1], u[2] * c1[0] - u[0] * c1[2], u[0] * c1[1] - u[1] * c1[0]};
    for (int k = 0; k < 3; ++k) out[k] = v[k] + 2.0f * (q[3] * c1[k] + c2[k]);
}

// How far the board is open (0 folded, 1 under his feet), from the frame of its own skeleton's
// open or close animation.
float board_open(const jak::BoardRuntime& rt) {
    const jak::BoardProcess& b = rt.board_proc;
    if (!b.alive || b.state == jak::BoardProcState::hidden) return 0.0f;
    const jak::JointControl& jc = b.skel;
    const int n = static_cast<int>(std::min<uint32_t>(jc.active_channels, jak::JointControl::kAllocated));
    if (n <= 0) return 0.0f;
    // The channel pushed last is the one playing; the one under it blends out in 0.1 s.
    const jak::JointControlChannel& ch = jc.channel[n - 1];
    const float last = static_cast<float>(jak::anim_info(ch.frame_group).num_frames - 1);
    const float f = last > 0.0f ? std::clamp(ch.frame_num / last, 0.0f, 1.0f) : 1.0f;
    if (ch.frame_group == jak::AnimId::board_open) return f;
    if (ch.frame_group == jak::AnimId::board_close) return 1.0f - f;
    return 0.0f;
}

// Ratchet where the target is, in his riding pose, and the board under his feet.
void show_ride() {
    const jak::Target& t = *g_ride.t;
    openrac_pose pose{};
    g_mod->hero_pose(pose);
    const float scale = pose.scale > 0.0f ? pose.scale : 1.0f;
    rac1::to_game_point(drawn_trans(t), pose.position);
    Vec fwd;
    jak::vector_z_quaternion(fwd, t.control.quat);
    const float yaw = rac1::jak_forward_to_yaw(fwd);
    pose.rotation[0] = 0.0f;
    pose.rotation[1] = 0.0f;
    pose.rotation[2] = yaw;
    rac1::to_game_dir(t.control.transv, pose.velocity);
    for (float& v : pose.velocity) v /= rac1::kUnit;
    // His standing pose, held: the riding poses are offsets over it.
    pose.seq = 0;
    pose.frame = 0;
    pose.anim_speed = 1.0f;
    g_mod->hero_set_pose(pose);

    // The riding pose.
    const float dt = 1.0f / (g_pal ? 50.0f : 60.0f);
    float rate = 0.0f;
    if (g_ride.has_yaw) {
        float d = yaw - g_ride.last_yaw;
        while (d > 3.14159265f) d -= kTwoPi;
        while (d < -3.14159265f) d += kTwoPi;
        rate = d / dt;
    }
    // How the board moves, in his heading's frame, for the secondary motion.
    rac1::Motion motion;
    {
        const float c = std::cos(yaw), s = std::sin(yaw);
        const float* v = pose.velocity;
        float acc[3] = {};
        if (g_ride.has_yaw) {
            for (int k = 0; k < 3; ++k) acc[k] = (v[k] - g_ride.last_vel[k]) / dt;
        }
        motion.speed = std::sqrt(v[0] * v[0] + v[1] * v[1]);
        motion.accel_fwd = acc[0] * c + acc[1] * s;
        motion.accel_side = -acc[0] * s + acc[1] * c;
        motion.vz = v[2];
        motion.az = acc[2];
        motion.yaw_rate = rate;
        const jak::StateId st = g_ride.rt->state;
        motion.grounded = st == jak::StateId::target_board_stance ||
                          st == jak::StateId::target_board_duck_stance ||
                          st == jak::StateId::target_board_hit_ground ||
                          st == jak::StateId::target_board_turn_to ||
                          st == jak::StateId::target_board_ride_edge ||
                          st == jak::StateId::target_board_start ||
                          st == jak::StateId::target_board_get_on ||
                          st == jak::StateId::target_board_get_off;
        std::copy(v, v + 3, g_ride.last_vel);
    }
    g_ride.last_yaw = yaw;
    g_ride.has_yaw = true;
    // Ratchet's clips, mixed as the board's logic plays Jak's.
    static rac1::MixPose mixed;
    rac1::mix(g_ride.rt->skel, mixed);
    const jak::BoardRuntime& rt = *g_ride.rt;
    const float upper = rt.upper_body.mode == jak::JointModMode::rotate
                            ? -rt.upper_body.twist_y * (kTwoPi / 65536.0f)
                            : 0.0f;
    g_ride.look.update(mixed, motion, board_spin(), upper, dt);
    const std::vector<rac1::JointOut> js = g_ride.look.joints();
    std::vector<openrac_joint_offset> offsets(js.size());
    for (size_t i = 0; i < js.size(); ++i) {
        offsets[i].joint = js[i].joint;
        std::memcpy(offsets[i].rotation, js[i].q, sizeof js[i].q);
        std::memcpy(offsets[i].translation, js[i].t, sizeof js[i].t);
    }
    g_mod->hero_set_joints(offsets);

    // The board: where the clip has it (its turn and place in his model space: x his heading, y his
    // left, z up), turned with him by the spin, then into the game's axes. The mesh is y-up with its
    // nose on +z (tools/make_board.py); its x is mirrored so the matrix keeps its handedness (the
    // board is the same both sides).
    // His model's frame (the pose above): at his feet, turned by his heading only.
    const float* p = pose.position;
    const float f[3] = {std::cos(yaw), std::sin(yaw), 0.0f}, l[3] = {-std::sin(yaw), std::cos(yaw), 0.0f};
    const float u[3] = {0.0f, 0.0f, 1.0f};
    const float* bt = g_ride.look.pose().board;
    const float bq[4] = {bt[rac1::kQx], bt[rac1::kQy], bt[rac1::kQz], bt[rac1::kQw]};
    // The body turns by the conjugate of its root's spin (see RideAnimator::joints): so does the board.
    const float hs = -0.5f * g_ride.look.spin();
    const float sq[4] = {0.0f, 0.0f, std::sin(hs), std::cos(hs)};
    // The board's open and close (Jak II's board-open / board-close on the board's own skeleton):
    // it grows from folded as it opens, and folds back the same way.
    constexpr float kFolded = 0.35f;
    const float open = board_open(*g_ride.rt);
    const float k = open * open * (3.0f - 2.0f * open);
    const float size = kFolded + (1.0f - kFolded) * k;
    const auto to_game = [&](const float v[3], float out[3]) {
        float a[3], b[3];
        qrot(sq, v, a);
        std::copy(a, a + 3, b);
        for (int c = 0; c < 3; ++c) out[c] = b[0] * f[c] + b[1] * l[c] + b[2] * u[c];
    };
    const float nose_m[3] = {1.0f, 0.0f, 0.0f}, left_m[3] = {0.0f, 1.0f, 0.0f}, up_m[3] = {0.0f, 0.0f, 1.0f};
    float nose[3], left[3], up[3], o[3];
    qrot(bq, nose_m, o);
    to_game(o, nose);
    qrot(bq, left_m, o);
    to_game(o, left);
    qrot(bq, up_m, o);
    to_game(o, up);
    const float place[3] = {bt[rac1::kPx] * scale, bt[rac1::kPy] * scale, bt[rac1::kPz] * scale};
    to_game(place, o);
    const float mat[16] = {size * left[0], size * left[1], size * left[2], 0.0f,
                           size * up[0],   size * up[1],   size * up[2],   0.0f,
                           size * nose[0], size * nose[1], size * nose[2], 0.0f,
                           p[0] + o[0],    p[1] + o[1],    p[2] + o[2],    1.0f};
    if (g_prop < 0 || !g_mod->prop_pose(g_prop, mat)) {
        g_prop = g_mod->prop_create("assets/board.glb");
        if (g_set.trace) g_mod->info("board prop " + std::to_string(g_prop) + " hero scale " + std::to_string(scale));
        if (g_prop >= 0) g_mod->prop_pose(g_prop, mat);
    }
    if (g_prop >= 0) g_mod->prop_hide(g_prop, false);
    hide_hand(true);
}

// rac1.player.update: the board's frame while riding, the game's otherwise.
void hero_update(const openrac::Original<void()>& original) {
    const bool held = g_pad.valid && toggle_held(g_pad.held);
    bool pressed = held && !g_toggle_was_held;
    g_toggle_was_held = held;
    if (g_mod->has_input() && g_mod->input().focused()) {
        for (int bind : g_set.keys) pressed = pressed || g_mod->input().pressed(bind);
    }

    if (riding() && g_mod->play_status().loads != g_ride.loads) end_ride();  // a new level
    if (!riding()) {
        if (!(pressed && playing() && start_ride())) {
            original();
            return;
        }
        pressed = false;  // the press that got on is not also the one that gets off
    }
    tick_ride(pressed);
    if (g_ride.leave) {
        // Off the board: the game's update goes on from where the board left Ratchet.
        show_ride();
        end_ride();
        original();
        return;
    }
    show_ride();
}

void parse_keys(const std::string& text) {
    g_set.keys.clear();
    size_t start = 0;
    while (start <= text.size()) {
        const size_t comma = text.find(',', start);
        const std::string name = text.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
        const int bind = openrac::bind_from_name(name);
        if (bind != OPENRAC_BIND_NONE) g_set.keys.push_back(bind);
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
}

int load(openrac::Mod& mod) {
    if (mod.game() != "rac1" || (mod.version() != "pal" && mod.version() != "ntsc")) {
        mod.fail("RAC-Board knows Ratchet & Clank's PAL and NTSC-U discs only");
        return 1;
    }
    if (!mod.has_api_12()) {
        mod.fail("RAC-Board needs OpenRAC's mod interface 1.12 (update OpenRAC)");
        return 1;
    }
    g_mod = &mod;
    g_pal = mod.version() == "pal";

    const std::string button = mod.setting<std::string>("board_button");
    g_set.toggle = button == "r2" ? Toggle::r2 : (button == "l2r2" ? Toggle::l2r2 : Toggle::down);
    parse_keys(mod.setting<std::string>("board_key"));
    g_set.swap_l1_r1 = mod.setting<bool>("swap_l1_r1");
    g_set.trick_display = mod.setting<bool>("trick_display");
    g_set.trace = mod.setting<bool>("trace");

    mod.on_pad([](int port, uint8_t* report, int size) {
        if (port != 0 || size < 18) return;
        g_pad.held = jak::ps2_buttons(report[0], report[1]);
        std::memcpy(g_pad.sticks, report + 2, 4);
        std::memcpy(g_pad.pressure, report + 6, 12);
        g_pad.valid = true;
        // The board's button is the mod's: in play the game does not see it (D-pad Down at any time,
        // R2 or L2 and R2 only while riding, so strafing still works on foot). Menus keep it.
        const bool take = g_set.toggle == Toggle::down ? playing() : riding();
        if (take) {
            const uint16_t bits = toggle_bits();
            if (g_set.toggle != Toggle::l2r2 || toggle_held(g_pad.held)) {
                report[0] |= static_cast<uint8_t>(bits & 0xFF);
                report[1] |= static_cast<uint8_t>(bits >> 8);
            }
        }
    });
    mod.hook<openrac::rac1::Hero_Update>([](const auto& original) { hero_update(original); });
    mod.info("RAC-Board ready");
    return 0;
}

} // namespace

OPENRAC_MOD_CPP("rac-board", "0.1.0", load)
