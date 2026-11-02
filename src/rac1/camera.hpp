// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The board's camera: Jak II's string camera (cam-states.gc, cam-string, with *CAMERA-bank*'s
// lengths and heights), in Ratchet & Clank's world, put in place of the game's own
// camera while Ratchet rides (OpenRAC's camera_set, mod interface 1.12).
#pragma once

#include "jak/target.hpp"

namespace rac1 {

// Takes the game's camera over from where it is now. False on a port without the camera calls.
bool camera_start(const jak::Target& t);
// One frame of the camera after the board's, set as the game's camera.
void camera_tick(const jak::Target& t);
// Lets the game's camera back: blended for a moment from the board's to the game's own (each
// frame, ridden or not, camera_blend_tick), or at once (`now`, when the game took Ratchet over).
void camera_stop(bool now);
void camera_blend_tick();

// For target-board-halfpipe-trans, in Jak II's units and axes: the string's flat offset (camera
// from target, camera-master-method-15), the target's move this frame (method-14) and the camera's
// local-down. False when the board's camera is not running.
bool camera_vectors(jak::Vec& view_flat, jak::Vec& target_move, jak::Vec& local_down);
// (send-event *camera* 'relative-position v): the camera put at the target plus `v` (Jak II's
// units and axes), or back on its string (null).
void camera_relative(const jak::Vec* v);

// The camera's eye now (game units), for read-pad's camera matrix. False when not running.
bool camera_eye(float out[3]);

} // namespace rac1
