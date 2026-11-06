// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/world/hand.hpp"

#include <algorithm>
#include <vector>

#include "rac1/context.hpp"

namespace rac1 {

namespace {

std::vector<int> g_hand; // the pieces' moby slots

constexpr int32_t kWrench = 8; // the wrench's item id (ITEM_WRENCH)
bool g_asked = false;          // the wrench request below is the mod's
int32_t g_held_request = 0;    // an item asked for while he rode, given back when he gets off

// A field of the hero's record, g_hero (PAL D_0013F450, NTSC-U D_0013F350).
openrac::gaddr hero_field(uint32_t offset) {
    return mod().relocate(ctx().pal ? 0x0013F450u : 0x0013F350u) + offset;
}
// force_slot0_item8: the game's own request to put the item in his hand away for the wrench. It
// clears it once done.
openrac::gaddr wrench_request() { return hero_field(0x20A6u); }
// The item asked for in his hand (the first of his item slots' requests, 0 for none): set by the
// pause menu and by pickups, carried out by the game's item code.
openrac::gaddr hand_request() { return hero_field(0x20B8u); }

} // namespace

bool is_wrench(int32_t item) { return item == kWrench; }

bool hand_free(int32_t item) { return item < 0 || is_wrench(item); }

void ask_for_wrench(bool ask) {
    const openrac::Memory mem = mod().memory();
    const openrac::gaddr at = wrench_request();
    if (ask) {
        // A request the game made itself stays the game's.
        g_asked = g_asked || mem.read<uint8_t>(at) == 0;
        mem.write<uint8_t>(at, 1);
    } else {
        if (g_asked && mem.read<uint8_t>(at) != 0) mem.write<uint8_t>(at, 0);
        g_asked = false;
    }
}

void hold_hand(int32_t item) {
    const openrac::Memory mem = mod().memory();
    const int32_t asked = mem.read<int32_t>(hand_request());
    if (asked != 0 && !is_wrench(asked)) {
        g_held_request = asked;
        mem.write<int32_t>(hand_request(), 0);
    }
    if (!hand_free(item)) ask_for_wrench(true);
}

void release_hold() {
    ask_for_wrench(false);
    // A request the game made since stays: it is the newer.
    const openrac::Memory mem = mod().memory();
    if (g_held_request != 0 && mem.read<int32_t>(hand_request()) == 0) {
        mem.write<int32_t>(hand_request(), g_held_request);
    }
    g_held_request = 0;
}

void find_hand(const float hero[3], bool add) {
    if (!add) g_hand.clear();
    openrac_held held;
    if (!mod().has_world() || !mod().hero_held(held)) return;
    for (const openrac_moby& m : mod().moby_list()) {
        if (m.flags & OPENRAC_MOBY_HIDDEN) continue;
        for (int i = 0; i < held.count; ++i) {
            const openrac_held_piece& p = held.pieces[i];
            if (p.slot != 0 || (p.flags & (OPENRAC_HELD_HIDDEN | OPENRAC_HELD_LOOSE)) || p.oclass != m.oclass) continue;
            const float dx = m.position[0] - hero[0], dy = m.position[1] - hero[1], dz = m.position[2] - hero[2];
            if (dx * dx + dy * dy + dz * dz < 9.0f &&
                std::find(g_hand.begin(), g_hand.end(), m.slot) == g_hand.end()) {
                g_hand.push_back(m.slot);
            }
            break;
        }
    }
    if (!add) trace("hand pieces " + std::to_string(g_hand.size()));
}

void hide_hand(bool hidden) {
    for (int slot : g_hand) mod().moby_hide(slot, hidden);
}

void release_hand() {
    hide_hand(false);
    g_hand.clear();
}

} // namespace rac1
