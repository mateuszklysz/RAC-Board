// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// What the board calls in the target's movement core: core's own headers (jak/movement.hpp,
// jak/cpad.hpp), the board's short pad readers over (-> self control cpad), and the few routines
// core does not have yet, each "owned by core" with the GOAL routine it stands for (OpenGOAL,
// jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
#pragma once

#include "jak/basic_types.hpp"
#include "jak/cpad.hpp"
#include "jak/movement.hpp"
#include "jak/target.hpp"

namespace jak {

// ---- the pad (pad.gc), read through (-> self control cpad) ----
namespace boardpad = pad;
inline const CpadInfo& target_pad(const Target& self) { return self.control.cpad ? *self.control.cpad : self.pad; }
inline CpadInfo& target_pad(Target& self) { return self.control.cpad ? *self.control.cpad : self.pad; }
// (cpad-hold? (-> self control cpad number) buttons...)
inline bool target_cpad_hold(const Target& self, uint32_t b) { return cpad_hold_p(target_pad(self), b); }
// (cpad-pressed? (-> self control cpad number) buttons...)
inline bool target_cpad_pressed(const Target& self, uint32_t b) { return cpad_pressed_p(target_pad(self), b); }
// (recently-pressed? buttons) (PC port).
inline bool target_recently_pressed(const Target& self, uint32_t b) { return recently_pressed_p(target_pad(self), b); }
inline uint8_t target_cpad_leftx(const Target& self) { return target_pad(self).leftx; }
inline uint8_t target_cpad_lefty(const Target& self) { return target_pad(self).lefty; }
inline float& target_cpad_stick0_speed(Target& self) { return target_pad(self).stick0_speed; }
// (-> *cpad-list* cpads n abutton (abutton-idx x))
inline uint8_t target_cpad_abutton_x(const Target& self) { return target_pad(self).abutton[kAbX]; }

// ---- the clock (clock-h.gc) ----
// (-> self clock clock-ratio): the adapter's input on Target.
inline float clock_ratio(const Target& self) { return self.clock_ratio; }
// owned by core: (-> self clock frames-per-second).
float frames_per_second(const Target& self);
// owned by core: (-> self clock old-frame-counter).
TimeFrame old_frame_counter(const Target& self);
// owned by core: (seconds-per-frame-high-fps) (PC port): 1/60 in the custom video mode, else
// (seconds-per-frame).
float seconds_per_frame_high_fps(const Target& self);

// ---- what core's headers do not have yet ----
void rot_to_dir_targ(Target& self);                // owned by core: rot->dir-targ! of the control
void target_powerup_process(Target& self);         // owned by core: target-powerup-process
void target_no_ja_move_post(Target& self);         // owned by core: target-no-ja-move-post
void set_quaternion(Target& self, const Quat& q);  // owned by core: set-quaternion! of the control
void target_gun_end_mode(Target& self, bool a0);   // owned by core: target-gun-end-mode
bool can_play_stance_ambient(Target& self);        // owned by core: can-play-stance-amibent? (target-util.gc)
float fact_health_default_inc();                   // owned by core: (-> *FACT-bank* health-default-inc)
extern Surface grab_mods;                          // owned by core: *grab-mods*
extern Surface smack_mods;                         // owned by core: *smack-mods*

} // namespace jak
