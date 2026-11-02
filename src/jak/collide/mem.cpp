// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The collision arena (collide/mem.hpp) and the globals Jak II's collide-cache.gc and
// collide-shape.gc set up at load time (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL
// Team; THIRD_PARTY.md).
#include "jak/collide/mem.hpp"

namespace jak {

namespace {
// Room past the stack's top: the MIPS loops (mips2c.cpp, probe-using-spheres) load the quadword
// after the last one they use before they test their count, as the console's code does.
constexpr uint32_t kArenaSlack = 64;
alignas(16) uint8_t g_arena[kArenaSize + kArenaSlack];
bool g_arena_ready = false;
} // namespace

namespace mips {
uint8_t* g_mem = g_arena;
}

uint32_t g_goal_sp = kAddrStackTop;

void collide_arena_init() {
    if (g_arena_ready) return;
    g_arena_ready = true;
    std::memset(g_arena, 0, sizeof(g_arena));
    // Type tags: only told apart from zero by the code here.
    collide_cache().type_tag = 1;
    // *collide-shape-prim-backgnd* (collide-shape.gc): no cshape, a sphere around everything,
    // collide-as backgnd, action solid, prim-type fake-prim.
    CollideShapePrim& bg = prim_backgnd();
    bg.type_tag = 2;
    bg.cshape = kGoalFalse;
    bg.prim_id = 0;
    bg.transform_index = 0;
    bg.prim_core.world_sphere = Vec{0.0f, 0.0f, 0.0f, 204800000.0f};
    bg.prim_core.collide_as = cspec::backgnd;
    bg.prim_core.collide_with = 0;
    bg.prim_core.action = caction::solid;
    bg.prim_core.prim_type = kPrimFake;
    bg.local_sphere = Vec{0.0f, 0.0f, 0.0f, 204800000.0f};
}

} // namespace jak
