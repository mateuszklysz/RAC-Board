// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/hand.hpp"

#include <vector>

#include "rac1/context.hpp"

namespace rac1 {

namespace {
std::vector<int> g_hand; // the pieces' moby slots
}

void find_hand(const float hero[3]) {
    g_hand.clear();
    openrac_held held;
    if (!mod().has_world() || !mod().hero_held(held)) return;
    for (const openrac_moby& m : mod().moby_list()) {
        if (m.flags & OPENRAC_MOBY_HIDDEN) continue;
        for (int i = 0; i < held.count; ++i) {
            const openrac_held_piece& p = held.pieces[i];
            if (p.slot != 0 || (p.flags & (OPENRAC_HELD_HIDDEN | OPENRAC_HELD_LOOSE)) || p.oclass != m.oclass) continue;
            const float dx = m.position[0] - hero[0], dy = m.position[1] - hero[1], dz = m.position[2] - hero[2];
            if (dx * dx + dy * dy + dz * dz < 9.0f) g_hand.push_back(m.slot);
            break;
        }
    }
    trace("hand pieces " + std::to_string(g_hand.size()));
}

void hide_hand(bool hidden) {
    for (int slot : g_hand) mod().moby_hide(slot, hidden);
}

void release_hand() {
    hide_hand(false);
    g_hand.clear();
}

} // namespace rac1
