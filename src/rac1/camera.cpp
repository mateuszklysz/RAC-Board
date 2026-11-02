// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/camera.hpp"

#include <algorithm>
#include <cmath>

#include "rac1/context.hpp"
#include "rac1/convert.hpp"
#include "rac1/pad.hpp"

namespace rac1 {

namespace {

// Jak II's numbers (camera-defs-h.gc *CAMERA-bank*, *CAMERA_MASTER-bank*), in meters, then scaled
// to Ratchet (convert.hpp).
constexpr float kMeter = kJakScale;                // one of Jak II's meters in game units
constexpr float kMinLength = 5.0f * kMeter;        // default-string-min-z
constexpr float kMaxLength = 12.5f * kMeter;       // default-string-max-z
constexpr float kPushLength = 10.0f * kMeter;      // default-string-push-z
constexpr float kMinHeight = 1.0f * kMeter;        // default-string-min-y
constexpr float kMaxHeight = 3.0f * kMeter;        // default-string-max-y
constexpr float kTargetHeight = 2.15f * kMeter;    // target-height
constexpr float kMoveRadius = 1638.4f / 4096.0f * kMeter; // collide-move-rad
constexpr float kStickTurn = 120.0f;               // degrees a second at full right stick
constexpr float kBlendSeconds = 0.5f;              // the hand back to the game's camera

struct V3 {
    float x = 0, y = 0, z = 0;
};
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float length(V3 a) { return std::sqrt(dot(a, a)); }
V3 normalized(V3 a, float to = 1.0f) {
    const float n = length(a);
    return n > 1e-6f ? a * (to / n) : V3{};
}
V3 lerp(V3 a, V3 b, float t) { return a + (b - a) * t; }
V3 from(const float p[3]) { return {p[0], p[1], p[2]}; }

struct Camera {
    bool on = false;
    V3 tpos, tpos_old;       // the target the string hangs from (tpos-curr-adj), and last frame's
    V3 view_flat;            // the camera from the target, flat (cam-string's view-flat)
    float min_z_override = kMaxLength;
    bool relative = false;   // 'relative-position: the camera at the target plus `rel`
    V3 rel;
    V3 eye, forward;
    // The hand back: the board's last camera, blended out to the game's own.
    float blend = 0.0f;
};
Camera g_cam;


// The hero's place (feet) in game units.
V3 feet(const jak::Target& t) {
    float p[3];
    to_game_point(t.control.trans, p);
    return from(p);
}

// Where along the segment a -> b the level's collision first comes within the camera's radius
// (0 to 1), 1 when it never does: the line-sphere probe of cam-string-move, as a line.
float probe(V3 a, V3 b) {
    const V3 d = b - a;
    const float lo[3] = {std::min(a.x, b.x) - kMoveRadius, std::min(a.y, b.y) - kMoveRadius,
                         std::min(a.z, b.z) - kMoveRadius};
    const float hi[3] = {std::max(a.x, b.x) + kMoveRadius, std::max(a.y, b.y) + kMoveRadius,
                         std::max(a.z, b.z) + kMoveRadius};
    float best = 1.0f;
    for (const openrac_collision_triangle& tri : mod().collision_triangles(lo, hi, 512)) {
        if ((tri.type & 0x80u) != 0) continue; // faces the game's own queries leave out
        // Möller-Trumbore, both sides.
        const V3 p0 = from(tri.corners[0]), e1 = from(tri.corners[1]) - p0, e2 = from(tri.corners[2]) - p0;
        const V3 h = cross(d, e2);
        const float det = dot(e1, h);
        if (std::fabs(det) < 1e-8f) continue;
        const float inv = 1.0f / det;
        const V3 s = a - p0;
        const float u = dot(s, h) * inv;
        if (u < 0.0f || u > 1.0f) continue;
        const V3 q = cross(s, e1);
        const float v = dot(d, q) * inv;
        if (v < 0.0f || u + v > 1.0f) continue;
        const float t = dot(e2, q) * inv;
        if (t > 0.0f && t < best) best = t;
    }
    // Kept the camera's radius short of the wall.
    const float len = length(d);
    return len > 1e-6f && best < 1.0f ? std::max(0.0f, best - kMoveRadius / len) : best;
}

// The right stick's turn this frame, in radians (analog-input-horizontal-third: a dead zone of 32
// around 128, full at 110 from it).
float stick_turn() {
    const float x = static_cast<float>(pad_right_x()) - 128.0f;
    const float mag = std::fabs(x);
    if (mag < 32.0f) return 0.0f;
    const float f = std::min(1.0f, (mag - 32.0f) / (110.0f - 32.0f));
    return std::copysign(f, x) * kStickTurn * frame_seconds() * (3.14159265f / 180.0f);
}

void set_game_camera(V3 eye, V3 forward) {
    const V3 f = normalized(forward);
    V3 left = normalized(cross(V3{0, 0, 1}, f));
    if (length(left) < 0.5f) left = V3{0, 1, 0};
    const V3 up = cross(f, left);
    openrac_camera c{};
    c.eye[0] = eye.x, c.eye[1] = eye.y, c.eye[2] = eye.z;
    c.forward[0] = f.x, c.forward[1] = f.y, c.forward[2] = f.z;
    c.left[0] = left.x, c.left[1] = left.y, c.left[2] = left.z;
    c.up[0] = up.x, c.up[1] = up.y, c.up[2] = up.z;
    mod().camera_set(c);
}

} // namespace

bool camera_start(const jak::Target& t) {
    openrac_camera view{};
    if (!mod().has_camera() || !mod().camera_view(view)) return false;
    g_cam = Camera{};
    g_cam.on = true;
    g_cam.tpos = feet(t) + V3{0, 0, kTargetHeight};
    g_cam.tpos_old = g_cam.tpos;
    g_cam.eye = from(view.eye);
    g_cam.forward = from(view.forward);
    V3 flat = g_cam.eye - g_cam.tpos;
    flat.z = 0.0f;
    g_cam.view_flat = length(flat) > 0.01f ? flat : V3{-1, 0, 0} * kMinLength;
    g_cam.min_z_override = std::clamp(length(g_cam.view_flat), kMinLength, kMaxLength);
    return true;
}

void camera_tick(const jak::Target& t) {
    if (!g_cam.on) return;
    Camera& c = g_cam;

    // The target the camera follows (tpos-curr-adj): the hero, his target height up.
    const V3 target = feet(t) + V3{0, 0, kTargetHeight};
    c.tpos_old = c.tpos;
    c.tpos = target;
    if (length(c.tpos - c.tpos_old) > kMaxLength) c.tpos_old = c.tpos; // a teleport

    V3 desired;
    if (c.relative) {
        desired = feet(t) + c.rel; // (get-trans self 0) plus the event's vector
    } else {
        // cam-string-follow: the camera stays where it was as the target moves; the string's
        // length is kept between the push length and the longest.
        V3 moved = c.tpos - c.tpos_old;
        moved.z = 0.0f;
        c.view_flat = c.view_flat - moved;
        c.view_flat.z = 0.0f;
        // cam-string-joystick: the right stick swings the camera about the target.
        const float a = -stick_turn();
        if (a != 0.0f) {
            const float cs = std::cos(a), sn = std::sin(a);
            c.view_flat = V3{c.view_flat.x * cs - c.view_flat.y * sn, c.view_flat.x * sn + c.view_flat.y * cs, 0};
        }
        // A string shorter than the push length (taken from the game's camera at the start, or
        // pulled in) eases back out to it, so the camera does not stay close.
        if (c.min_z_override < kPushLength)
            c.min_z_override = std::min(kPushLength, c.min_z_override + kPushLength * frame_seconds());
        const float len = length(c.view_flat);
        const float shortest = std::max(kMinLength, std::min({kPushLength, kMaxLength, c.min_z_override}));
        if (len < shortest) {
            c.view_flat = normalized(c.view_flat.x != 0 || c.view_flat.y != 0 ? c.view_flat : c.forward * -1.0f, shortest);
            c.view_flat.z = 0.0f;
        } else if (len > kMaxLength) {
            c.view_flat = normalized(c.view_flat, kMaxLength);
            c.min_z_override = kMaxLength;
        } else {
            c.min_z_override = len;
        }
        // The height over the target grows with the string's length (view-off y).
        const float f = std::clamp((length(c.view_flat) - kMinLength) / (kMaxLength - kMinLength), 0.0f, 1.0f);
        const float height = kMinHeight + (kMaxHeight - kMinHeight) * f;
        desired = c.tpos + c.view_flat + V3{0, 0, height};
    }

    // cam-string-move: the camera kept out of the level's collision, pulled in toward the target
    // for this frame (the string keeps its length, so it swings back out past the wall).
    const float k = probe(c.tpos, desired);
    c.eye = lerp(c.tpos, desired, k);
    c.forward = normalized(c.tpos - c.eye);
    if (length(c.forward) < 0.5f) c.forward = V3{1, 0, 0};
    set_game_camera(c.eye, c.forward);
}

void camera_stop(bool now) {
    if (!g_cam.on) return;
    g_cam.on = false;
    g_cam.relative = false;
    if (now) {
        g_cam.blend = 0.0f;
        mod().camera_release();
    } else {
        g_cam.blend = kBlendSeconds;
    }
}

void camera_blend_tick() {
    if (g_cam.on || g_cam.blend <= 0.0f) return;
    g_cam.blend -= frame_seconds();
    openrac_camera view{};
    if (g_cam.blend <= 0.0f || !mod().camera_view(view)) {
        g_cam.blend = 0.0f;
        mod().camera_release();
        return;
    }
    // Eased from the board's last camera to the game's own.
    const float s = 1.0f - g_cam.blend / kBlendSeconds;
    const float e = s * s * (3.0f - 2.0f * s);
    set_game_camera(lerp(g_cam.eye, from(view.eye), e), normalized(lerp(g_cam.forward, from(view.forward), e)));
}

bool camera_vectors(jak::Vec& view_flat, jak::Vec& target_move, jak::Vec& local_down) {
    if (!g_cam.on) return false;
    const float vf[3] = {g_cam.view_flat.x * kUnit, g_cam.view_flat.y * kUnit, g_cam.view_flat.z * kUnit};
    const V3 m = g_cam.tpos - g_cam.tpos_old;
    const float mv[3] = {m.x * kUnit, m.y * kUnit, m.z * kUnit};
    view_flat = to_jak_dir(vf);
    target_move = to_jak_dir(mv);
    local_down = jak::Vec{0.0f, -1.0f, 0.0f, 0.0f};
    return true;
}

void camera_relative(const jak::Vec* v) {
    if (v == nullptr) {
        g_cam.relative = false;
        return;
    }
    float p[3];
    to_game_dir(*v, p);
    g_cam.rel = V3{p[0], p[1], p[2]} * (1.0f / kUnit);
    g_cam.relative = true;
}

bool camera_eye(float out[3]) {
    if (!g_cam.on) return false;
    out[0] = g_cam.eye.x, out[1] = g_cam.eye.y, out[2] = g_cam.eye.z;
    return true;
}

} // namespace rac1
