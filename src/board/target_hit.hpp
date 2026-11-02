// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// How the target takes a hit: target-attacked (target-handler.gc), the attack-info methods and
// target-log-attack (target-util.gc), and the knock-back helpers of target-death.gc. Ported from
// OpenGOAL (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
#pragma once

#include "board/interaction.hpp"
#include "board/runtime.hpp"
#include "jak/target.hpp"

namespace jak {

// attack-mode-is-invinc (target-handler.gc)
bool attack_mode_is_invinc(Symbol mode);
// target-attacked (target-handler.gc): the hit taken (to `hit_state`) or not; its reply.
Symbol target_attacked(Target& self, Symbol message, const AttackInfo& info, Handle from, StateId hit_state);
// combine! (target-util.gc): `src`'s fields over `dst`'s, the vector worked out when missing.
AttackInfo& attack_info_combine(Target& self, AttackInfo& dst, const AttackInfo& src);
// target-hit-effect (target-death.gc): the flash, the invulnerable time, the buzz and the sound.
void target_hit_effect(Target& self, const AttackInfo& info);
// target-hit-orient (target-death.gc): faces the hit; true when the knock-back lands on the ground.
bool target_hit_orient(Target& self, const AttackInfo& info, const Vec& dir);
// target-hit-push (target-death.gc): one frame of the knock-back; #f done, #t on, 'stuck.
Symbol target_hit_push(Target& self, Vec& start, const Mat& rot, float dist, float speed, const AttackInfo& info);

} // namespace jak
