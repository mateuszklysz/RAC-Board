// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/ride.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "board/board.hpp"
#include "board/core.hpp"
#include "board/runtime.hpp"
#include "jak/collide/collide.hpp"
#include "jak/math/math.hpp"
#include "jak/movement.hpp"
#include "jak/target.hpp"
#include "rac1/anim/ride_pose.hpp"
#include "rac1/context.hpp"
#include "rac1/convert.hpp"
#include "rac1/fx/effects.hpp"
#include "rac1/interaction.hpp"
#include "rac1/pad.hpp"
#include "rac1/camera.hpp"
#include "rac1/world/ground.hpp"
#include "rac1/world/hand.hpp"
#include "rac1/world/rails.hpp"
#include "rac1/world/world.hpp"

namespace rac1 {

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kTwoPi = 6.2831853f;

struct Ride {
    std::unique_ptr<jak::Target> t;
    std::unique_ptr<jak::BoardRuntime> rt;
    uint32_t loads = 0; // play_status().loads as the ride started
    jak::StateId traced = jak::StateId::none;
    RideAnimator look;
    float last_yaw = 0.0f;
    float last_vel[3] = {};
    bool has_yaw = false;
    float shown[3] = {};  // where Ratchet was put last frame
    bool has_shown = false;
    float pose_in = 0.0f; // how far the riding pose has blended in (0 to 1)
    // The animation Ratchet had as he got on, the one his state (kept through the ride) plays.
    uint32_t start_seq = 0;
    float start_speed = 1.0f;
};
Ride g_ride;
World g_world;
int g_board_prop = -1;

// The riding pose eased in as he gets on and out after he gets off, over these seconds, so his body
// does not jump between the game's animation and the board's pose.
constexpr float kPoseIn = 0.2f;
constexpr float kPoseOut = 0.3f;
std::vector<openrac_joint_offset> g_last_offsets; // the riding pose's last frame
float g_pose_out = 0.0f;                         // seconds of easing out left

// The offsets `w` of the way from none (0) to all of them (1): rotations along the shorter arc,
// translations in a line.
std::vector<openrac_joint_offset> scaled(std::vector<openrac_joint_offset> js, float w) {
    const float e = w * w * (3.0f - 2.0f * w);
    for (openrac_joint_offset& j : js) {
        float* q = j.rotation;
        const float s = q[3] < 0.0f ? -1.0f : 1.0f; // the identity's side
        float n = 0.0f;
        for (int k = 0; k < 4; ++k) {
            q[k] = (k == 3 ? 1.0f - e : 0.0f) + e * s * q[k];
            n += q[k] * q[k];
        }
        n = n > 1e-12f ? 1.0f / std::sqrt(n) : 1.0f;
        for (int k = 0; k < 4; ++k) q[k] *= n;
        for (float& v : j.translation) v *= e;
    }
    return js;
}

// What want-to-board? reads off a board-info that outlives the ride: the last frame on the board,
// whether there was a ride yet, and the last frame Ratchet stood on the ground since.
jak::TimeFrame g_board_time = 0;
bool g_rode = false;
jak::TimeFrame g_ground_time = 0;

// ---- Ratchet's states (HeroStateId, Lombyte's rnc/hero/hero_state.h) as Jak II's focus ----
bool hero_on_ground(int s) {
    switch (s) {
    case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x08: case 0x13: case 0x14: case 0x15:
    case 0x17: case 0x1D: case 0x1E: case 0x1F: case 0x20: case 0x21: case 0x23: case 0x27: case 0x2E:
    case 0x2F: case 0x30: case 0x31: case 0x32: case 0x40: case 0x41: case 0x59: case 0x5F: case 0x63:
    case 0x72: case 0x73: case 0x79:
        return true;
    default:
        return false;
    }
}
bool hero_in_air(int s) {
    return (s >= 0x06 && s <= 0x11 && s != 0x08) || s == 0x22 || s == 0x2D;
}
// The focus bits that keep Jak off the board (want-to-board?) for a state he cannot get on from.
uint64_t hero_focus(int s) {
    if (hero_on_ground(s) || hero_in_air(s) || s == 0x12 || (s >= 0x33 && s <= 0x37)) return 0; // water: water_flags
    switch (s) {
    case 0x16: case 0x3C: case 0x46: case 0x56: case 0x5D: case 0x75: case 0x76:
        return jak::focus::hit;
    case 0x18: case 0x19: case 0x1A: case 0x1B: case 0x1C: case 0x4A: case 0x4B: case 0x4C: case 0x4D: case 0x4E:
        return jak::focus::edge_grab;
    case 0x3E: case 0x6B: case 0x6C: case 0x6D: case 0x6E: case 0x6F:
        return jak::focus::board; // the game's own hoverboard
    case 0x3D: case 0x47: case 0x57: case 0x62: case 0x64: case 0x6A: case 0x77: case 0x7C: case 0x7D: case 0x7F:
    case 0x80: case 0x82:
        return jak::focus::dead;
    default:
        if (s >= 0x43 && s <= 0x62) return jak::focus::pilot; // Clank and Giant Clank
        return jak::focus::grabbed; // grinds, swings, ladders, cranks, paths, scenes
    }
}

// The camera's rotation for read-pad: forward from the game's camera to the hero, flat.
jak::Mat camera_matrix(const float hero[3], float hero_yaw) {
    float fx = std::cos(hero_yaw), fy = std::sin(hero_yaw);
    float cam[3];
    if (camera_eye(cam) || mod().camera_position(cam)) {
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
    m.v[0] = jak::Vec{-fy, 0.0f, fx, 0.0f};
    m.v[1] = jak::Vec{0.0f, 1.0f, 0.0f, 0.0f};
    m.v[2] = jak::Vec{fx, 0.0f, fy, 0.0f};
    return m;
}

void advance_clock(jak::Target& t) {
    t.clock.old_frame_counter = ctx().clock;
    t.clock.frame_counter = ctx().clock;
    t.real_frame_counter += frame_ticks();
    jak::update_rates(t, 1.0f);
}

// The board's spin (its main joint's twist) turns him with it; Jak's y turn is the game's z turn the
// other way (the axes' mirror).
float board_spin() {
    const jak::BoardRuntime& rt = *g_ride.rt;
    return rt.main.mode == jak::JointModMode::rotate ? -rt.main.twist_y * (kTwoPi / 65536.0f) : 0.0f;
}

// Turns `v` by the unit quaternion `q` (x, y, z, w).
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

bool grounded(jak::StateId st) {
    using jak::StateId;
    return st == StateId::target_board_stance || st == StateId::target_board_duck_stance ||
           st == StateId::target_board_hit_ground || st == StateId::target_board_turn_to ||
           st == StateId::target_board_ride_edge || st == StateId::target_board_start ||
           st == StateId::target_board_get_on || st == StateId::target_board_get_off;
}

// Ratchet's body: his place and heading, held in his standing pose, the riding pose over it.
void show_ratchet(openrac_pose& pose, float yaw, bool last) {
    const jak::Target& t = *g_ride.t;
    to_game_point(drawn_trans(t), pose.position);
    pose.rotation[0] = 0.0f;
    pose.rotation[1] = 0.0f;
    pose.rotation[2] = yaw;
    to_game_dir(t.control.transv, pose.velocity);
    for (float& v : pose.velocity) v /= kUnit;
    // Held in his standing pose under the riding one; on the last frame, back to the animation of
    // the state the game goes on in, blended in by the game as the riding pose eases out.
    pose.seq = last ? g_ride.start_seq : 0;
    pose.frame = 0;
    pose.anim_speed = last ? g_ride.start_speed : 1.0f;
    mod().hero_set_pose(pose);
    std::copy(pose.position, pose.position + 3, g_ride.shown);
    g_ride.has_shown = true;

    const float dt = frame_seconds();
    float rate = 0.0f;
    if (g_ride.has_yaw) {
        float d = yaw - g_ride.last_yaw;
        while (d > kPi) d -= kTwoPi;
        while (d < -kPi) d += kTwoPi;
        rate = d / dt;
    }
    // How the board moves, in his heading's frame, for the secondary motion.
    Motion motion;
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
    motion.grounded = grounded(g_ride.rt->state);
    std::copy(v, v + 3, g_ride.last_vel);
    g_ride.last_yaw = yaw;
    g_ride.has_yaw = true;

    // Ratchet's clips, mixed as the board's logic plays Jak's.
    static MixPose mixed;
    mix(g_ride.rt->skel, mixed);
    const jak::BoardRuntime& rt = *g_ride.rt;
    const float upper = rt.upper_body.mode == jak::JointModMode::rotate ? -rt.upper_body.twist_y * (kTwoPi / 65536.0f)
                                                                         : 0.0f;
    g_ride.look.update(mixed, motion, board_spin(), upper, dt);
    const std::vector<JointOut> js = g_ride.look.joints();
    std::vector<openrac_joint_offset> offsets(js.size());
    for (size_t i = 0; i < js.size(); ++i) {
        offsets[i].joint = js[i].joint;
        std::memcpy(offsets[i].rotation, js[i].q, sizeof js[i].q);
        std::memcpy(offsets[i].translation, js[i].t, sizeof js[i].t);
    }
    g_last_offsets = offsets;
    g_ride.pose_in = std::min(1.0f, g_ride.pose_in + dt / kPoseIn);
    mod().hero_set_joints(g_ride.pose_in < 1.0f ? scaled(offsets, g_ride.pose_in) : offsets);
}

// The board: where the clip has it (its turn and place in his model space: x his heading, y his
// left, z up), turned with him by the spin, then into the game's axes. The mesh is y-up with its
// nose on +z (tools/make_board.py); its x is mirrored so the matrix keeps its handedness (the board
// is the same both sides).
void show_board(const openrac_pose& pose, float yaw) {
    const float scale = pose.scale > 0.0f ? pose.scale : 1.0f;
    const float* p = pose.position;
    const float f[3] = {std::cos(yaw), std::sin(yaw), 0.0f}, l[3] = {-std::sin(yaw), std::cos(yaw), 0.0f};
    const float u[3] = {0.0f, 0.0f, 1.0f};
    const float* bt = g_ride.look.pose().board;
    const float bq[4] = {bt[kQx], bt[kQy], bt[kQz], bt[kQw]};
    // The body turns by the conjugate of its root's spin (see RideAnimator::joints): so does the board.
    const float hs = -0.5f * g_ride.look.spin();
    const float sq[4] = {0.0f, 0.0f, std::sin(hs), std::cos(hs)};
    // The board's open and close (Jak II's board-open / board-close on its own skeleton): it grows
    // from folded as it opens, and folds back the same way.
    constexpr float kFolded = 0.35f;
    const float open = board_open(*g_ride.rt);
    const float k = open * open * (3.0f - 2.0f * open);
    const float size = kFolded + (1.0f - kFolded) * k;
    const auto to_game = [&](const float v[3], float out[3]) {
        float a[3];
        qrot(sq, v, a);
        for (int c = 0; c < 3; ++c) out[c] = a[0] * f[c] + a[1] * l[c] + a[2] * u[c];
    };
    const float nose_m[3] = {1.0f, 0.0f, 0.0f}, left_m[3] = {0.0f, 1.0f, 0.0f}, up_m[3] = {0.0f, 0.0f, 1.0f};
    float nose[3], left[3], up[3], o[3];
    qrot(bq, nose_m, o);
    to_game(o, nose);
    qrot(bq, left_m, o);
    to_game(o, left);
    qrot(bq, up_m, o);
    to_game(o, up);
    const float place[3] = {bt[kPx] * scale, bt[kPy] * scale, bt[kPz] * scale};
    to_game(place, o);
    const float mat[16] = {size * left[0], size * left[1], size * left[2], 0.0f,
                           size * up[0],   size * up[1],   size * up[2],   0.0f,
                           size * nose[0], size * nose[1], size * nose[2], 0.0f,
                           p[0] + o[0],    p[1] + o[1],    p[2] + o[2],    1.0f};
    if (g_board_prop < 0 || !mod().prop_pose(g_board_prop, mat)) {
        // None yet, or gone with its level: a new one.
        g_board_prop = mod().prop_create("assets/board.glb");
        trace("board prop " + std::to_string(g_board_prop));
        if (g_board_prop >= 0) mod().prop_pose(g_board_prop, mat);
    }
    if (g_board_prop >= 0) mod().prop_hide(g_board_prop, false);
}

} // namespace

bool riding() { return g_ride.t != nullptr; }

void pose_out_tick() {
    if (g_pose_out <= 0.0f) return;
    g_pose_out -= frame_seconds();
    if (g_pose_out <= 0.0f) {
        g_pose_out = 0.0f;
        g_last_offsets.clear();
        mod().hero_set_joints({});
        return;
    }
    mod().hero_set_joints(scaled(g_last_offsets, g_pose_out / kPoseOut));
}

bool ride_level_changed() { return riding() && mod().play_status().loads != g_ride.loads; }

void note_on_foot() {
    openrac_pose pose{};
    if (mod().hero_pose(pose) && hero_on_ground(pose.state)) g_ground_time = ctx().clock;
}

bool try_start_ride() {
    openrac_pose pose{};
    if (!playing() || !mod().hero_pose(pose)) return false;
    auto t = std::make_unique<jak::Target>();
    auto rt = std::make_unique<jak::BoardRuntime>();
    t->world = &g_world;
    t->effects = &effects();
    t->display_time_factor = ctx().pal ? 6.0f : 5.0f;
    t->clock.frame_counter = ctx().clock;
    jak::movement_init(*t, to_jak_point(pose.position));
    jak::target_collide_init(*t);
    t->control.local_normal = jak::Vec{0.0f, 1.0f, 0.0f, 1.0f};
    t->control.gspot_normal = jak::Vec{0.0f, 1.0f, 0.0f, 1.0f};
    jak::update_rates(*t, 1.0f);
    Interaction& io = interaction();
    io.target = t.get();
    io.hero_state = pose.state;
    const uint32_t start_seq = pose.seq;
    const float start_speed = pose.anim_speed > 0.0f ? pose.anim_speed : 1.0f;
    io.left = false;
    jak::attach(*t, *rt, io);

    // want-to-board?, with what Ratchet is doing as Jak's focus and the board-info kept between rides.
    t->focus_status |= hero_focus(pose.state);
    t->control.list_time_on_ground = g_ground_time;
    t->board->board_time = g_board_time;
    rt->board_setup = g_rode;
    feed_pad(*t, true);
    const bool ok = jak::want_to_board_p(*t);
    rt->board_setup = false; // target-board-setup's own first time, as each ride makes a new target
    if (!ok) {
        trace("no board in hero state " + std::to_string(pose.state));
        io.target = nullptr;
        return false;
    }
    t->focus_status &= ~(jak::focus::hit | jak::focus::edge_grab | jak::focus::grabbed | jak::focus::dead);

    jak::Quat q;
    jak::forward_up_nopitch_to_quaternion(q, yaw_to_jak_forward(pose.rotation[2]), jak::Vec{0, 1, 0, 0});
    t->control.quat = q;
    jak::set_quaternion(*t, q);
    jak::rot_to_dir_targ(*t);
    const jak::Vec v = to_jak_dir(pose.velocity);
    t->control.transv = jak::Vec{v.x * kUnit, v.y * kUnit, v.z * kUnit, 0.0f};
    // Jak walks onto the board with his ground already found; get-on's hop is timed from it.
    jak::CollideQuery cq;
    if (jak::find_ground(*t, cq, jak::root_prim_collide_with(*t) & ~jak::cspec::water, 8192.0f, 81920.0f, 1024.0f)) {
        t->control.gspot_pat_surfce = cq.best_pat;
    }

    find_hand(pose.position);
    if (settings().trace && mod().has_world()) {
        for (const openrac_moby& m : mod().moby_list()) {
            if (!(m.flags & OPENRAC_MOBY_CREATURE)) continue;
            trace("creature " + std::to_string(m.slot) + " class " + std::to_string(m.oclass) + " at " +
                  std::to_string(m.position[0]) + "," + std::to_string(m.position[1]) + "," +
                  std::to_string(m.position[2]));
        }
    }
    trace("fall height " + std::to_string(fall_death_height()));
    trace_rails();
    g_ride.t = std::move(t);
    g_ride.rt = std::move(rt);
    g_ride.loads = mod().play_status().loads;
    g_ride.traced = jak::StateId::none;
    g_ride.start_seq = start_seq;
    g_ride.start_speed = start_speed;
    // Back on while the last ride's pose eases out: on from as far as it is.
    g_ride.pose_in = g_pose_out / kPoseOut;
    g_pose_out = 0.0f;
    g_rode = true;
    jak::go(*g_ride.t, jak::StateId::target_board_start);
    if (settings().board_camera) camera_start(*g_ride.t);
    return true;
}

// Whether the game took Ratchet over while he rode: a scene, a vendor, a death or a move from its
// own code changes his state or puts him somewhere else. The board then lets him go.
bool game_took_over() {
    openrac_pose pose{};
    if (!mod().hero_pose(pose)) return true;
    if (pose.state != interaction().hero_state) {
        trace("hero state " + std::to_string(pose.state) + " from the game: off the board");
        return true;
    }
    if (g_ride.has_shown) {
        const float dx = pose.position[0] - g_ride.shown[0], dy = pose.position[1] - g_ride.shown[1],
                    dz = pose.position[2] - g_ride.shown[2];
        if (dx * dx + dy * dy + dz * dz > 1.0f) {
            trace("moved by the game: off the board");
            return true;
        }
    }
    return false;
}

RideStep tick_ride(bool toggle) {
    if (game_took_over()) return RideStep::taken;
    jak::Target& t = *g_ride.t;
    advance_clock(t);
    float hero[3];
    to_game_point(t.control.trans, hero);
    jak::Vec fwd;
    jak::vector_z_quaternion(fwd, t.control.quat);
    t.camera_local_to_world = camera_matrix(hero, jak_forward_to_yaw(fwd));
    feed_pad(t, toggle);

    deliver_hit(t);
    if (mod().has_world()) touch_creatures(t);
    carry(t);
    jak::board_tick(t);
    float feet[3];
    to_game_point(t.control.trans, feet);
    const Ground ground = ground_under(feet);
    note_ground(t, ground);

    if (g_ride.rt->state != g_ride.traced) {
        g_ride.traced = g_ride.rt->state;
        trace(std::string("state ") + jak::state_name(g_ride.traced) + " height " +
              std::to_string(t.control.trans.y / kUnit));
    }
    if (feet[2] < fall_death_height() && !(t.control.status & jak::cstat::on_surface)) {
        // Fallen out of the level: the game's own fall to death takes over.
        trace("below the fall height: off the board");
        return RideStep::left;
    }
    if (ground.found && game_hazard(ground.surface) && (t.control.status & jak::cstat::on_surface)) {
        // Water, a death floor or a sinking liquid: the game's own rules take over, as on foot.
        trace("ground hazard " + std::to_string(ground.surface) + ": off the board");
        return RideStep::left;
    }
    if (interaction().left) return RideStep::left;
    camera_tick(t);
    return RideStep::riding;
}

void show_ride(bool last) {
    const jak::Target& t = *g_ride.t;
    openrac_pose pose{};
    mod().hero_pose(pose);
    jak::Vec fwd;
    jak::vector_z_quaternion(fwd, t.control.quat);
    const float yaw = jak_forward_to_yaw(fwd);
    effects().yaw = yaw;
    show_ratchet(pose, yaw, last);
    show_board(pose, yaw);
    hide_hand(true);
}

void end_ride() {
    if (!riding()) return;
    camera_stop(false);
    g_board_time = g_ride.t->board->board_time;
    // The riding pose eases out over the game's animation (pose_out_tick).
    g_pose_out = g_last_offsets.empty() ? 0.0f : kPoseOut;
    if (g_pose_out <= 0.0f) mod().hero_set_joints({});
    release_hand();
    g_ride.look.reset();
    g_ride.has_yaw = false;
    g_ride.has_shown = false;
    effects().stop_sounds();
    forget_ground();
    if (g_board_prop >= 0) mod().prop_hide(g_board_prop, true);
    interaction().target = nullptr;
    interaction().left = false;
    g_ride.t.reset();
    g_ride.rt.reset();
}

} // namespace rac1
