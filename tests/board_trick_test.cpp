// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The board's trick points (add-to-trick-list, flush-trick-list, board-h.gc / target-board.gc):
// the repeat discount, the round-up to 25, the combo's doubling, and trick-count's overflow into
// trick-list, worked out by hand from the GOAL.
#include "board_fakes.hpp"
#include "check.hpp"

using namespace jak;

int main() {
    Target t;
    fake::Io io;
    fake::Fx fx;
    BoardRuntime rt;
    t.effects = &fx;
    attach(t, rt, io);
    BoardInfo& b = *t.board;

    // A first method: 500, alone.
    add_to_trick_list(t, BoardTricks::board_method, 500.0f);
    CHECK(b.trick_count == 1);
    CHECK(b.trick_points_array[0] == 500.0f);
    CHECK(fx.tricks.size() == 1 && fx.tricks[0].second == 500.0f);
    // Again: one repeat is still under lerp-scale's 2, so 500; the last trick doubles, the combo
    // shows 500 + 500.
    add_to_trick_list(t, BoardTricks::board_method, 500.0f);
    CHECK(b.trick_points_array[0] == 1000.0f);
    CHECK(b.trick_points_array[1] == 500.0f);
    CHECK(fx.tricks[1].second == 1000.0f);
    // A third (2 repeats: factor 1) and a fourth (3 repeats: lerp-scale 1 -> 0.1 over 2..6 gives
    // 0.775, 387.5 -> int 387, (387 + 24) / 25 * 25 = 400).
    add_to_trick_list(t, BoardTricks::board_method, 500.0f);
    add_to_trick_list(t, BoardTricks::board_method, 500.0f);
    CHECK(b.trick_points_array[2] == 1000.0f);
    CHECK(b.trick_points_array[3] == 400.0f);
    // A trick of no points only counts.
    add_to_trick_list(t, BoardTricks::board_jump, 0.0f);
    CHECK(b.trick_count == 5);
    CHECK(io.trick_points.size() == 5);
    // Flush: 1000 + 1000 + 1000 + 400 + 0 (entry 4 was never written).
    flush_trick_list(t);
    CHECK(io.flushes == 1);
    CHECK(io.flushed == 3400.0f);
    CHECK(b.trick_count == 0);

    // A rail ride for 2 s gives 200, rounded: (int 200) + 24 = 224 / 25 = 8 -> 200.
    add_to_trick_list(t, BoardTricks::board_rail, 100.0f * 2.0f);
    CHECK(b.trick_points_array[0] == 200.0f);
    flush_trick_list(t);

    // trick-count is never capped: past 16 the flush reads trick-list's words as floats.
    for (int i = 0; i < 17; i++) {
        add_to_trick_list(t, BoardTricks::board_jump, 0.0f);
    }
    CHECK(b.trick_count == 17);
    b.trick_list[0] = static_cast<BoardTricks>(0x3f800000u); // reads as 1.0f
    flush_trick_list(t);
    // trick-points-array keeps the earlier points (nothing clears it): 200 (the rail over entry 0),
    // 1000, 1000, 400, then 0s, and entry 16 is trick-list[0]'s bits.
    CHECK(io.flushed == 2601.0f);
    TEST_MAIN_END();
}
