// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The board's own process: goal_src/jak2/engine/target/board/board-util.gc from OpenGOAL
// (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). It follows
// the board joint (board-post) and plays its open and close animations (idle, use, hidden). It is a
// child process of the target, so it runs right after it each frame.
#include "board/anim.hpp"
#include "board/board.hpp"

namespace jak {

namespace {

AnimClock board_clock(Target& self) { return anim_clock(self); }

void board_go(Target& self, BoardProcState s, bool arg = false) {
    BoardProcess& p = runtime(self).board_proc;
    p.state = s;
    p.idle_arg = arg;
    p.co = Co();
    p.sleeping = false;
}

// board-post: the board copies the target's board-trans / quat / scale, then ja-post.
void board_post(Target& self) {
    BoardProcess& p = runtime(self).board_proc;
    p.trans = self.board->board_trans;
    p.quat = Quat{self.board->board_quat.x, self.board->board_quat.y, self.board->board_quat.z,
                  self.board->board_quat.w};
    p.scale = self.board->board_scale;
    // (the lights, colours, fade, death effect and shadow follow the target's: drawing)
    evaluate_joint_control(p.skel, board_clock(self));
}

// The idle state's code: close (at once when arg is #t), then sleep.
void idle_code(Target& self, BoardProcess& p) {
    Co& co = p.co;
    JointControl& jc = p.skel;
    CO_BEGIN(co);
    ja_channel_push(jc, 1, seconds(0.1), board_clock(self));
    if (p.idle_arg) {
        Ja().group(AnimId::board_close).max().run(jc, board_clock(self));
    } else {
        Ja().group(AnimId::board_close).seek().frame_num(0.0f).no_eval().run(jc, board_clock(self));
        while (!ja_done(jc, 0)) {
            CO_SUSPEND(co);
            Ja().seek().run(jc, board_clock(self));
        }
    }
    p.shadow = false;
    p.sleeping = true;
    CO_END(co);
}

// The use state's code: open, then sleep.
void use_code(Target& self, BoardProcess& p) {
    Co& co = p.co;
    JointControl& jc = p.skel;
    CO_BEGIN(co);
    p.shadow = true;
    ja_channel_push(jc, 1, seconds(0.1), board_clock(self));
    Ja().group(AnimId::board_open).seek().frame_num(0.0f).no_eval().run(jc, board_clock(self));
    while (!ja_done(jc, 0)) {
        CO_SUSPEND(co);
        Ja().seek().run(jc, board_clock(self));
    }
    p.sleeping = true;
    CO_END(co);
}

} // namespace

// board-init: the skeleton with no channels, then idle #t (closed).
void board_init(Target& self) {
    BoardProcess& p = runtime(self).board_proc;
    p = BoardProcess();
    p.alive = true;
    ja_channel_set(p.skel, 0, board_clock(self));
    board_go(self, BoardProcState::idle, true);
}

void board_deactivate(Target& self) { runtime(self).board_proc.alive = false; }

// The board's event handlers: idle takes 'open, use takes 'close.
void board_process_event(Target& self, Symbol message) {
    BoardProcess& p = runtime(self).board_proc;
    if (!p.alive) {
        return;
    }
    if (p.state == BoardProcState::idle && message == Symbol("open")) {
        board_go(self, BoardProcState::use);
    } else if (p.state == BoardProcState::use && message == Symbol("close")) {
        board_go(self, BoardProcState::idle, false);
    }
}

// One frame of the board: its trans (in-head hides it), code and post.
void board_process_tick(Target& self) {
    BoardProcess& p = runtime(self).board_proc;
    if (!p.alive) {
        return;
    }
    const bool in_head = focus_test(self, focus::in_head);
    switch (p.state) {
    case BoardProcState::hidden:
        if (!in_head) {
            board_go(self, BoardProcState::idle, false);
        }
        break;
    case BoardProcState::idle:
        if (in_head) {
            board_go(self, BoardProcState::hidden);
        }
        break;
    case BoardProcState::use:
        if (in_head) {
            board_go(self, BoardProcState::hidden);
        } else if (!focus_test(self, focus::board)) {
            board_go(self, BoardProcState::idle, false);
        }
        break;
    case BoardProcState::none:
        break;
    }
    if (!p.sleeping) {
        switch (p.state) {
        case BoardProcState::idle: idle_code(self, p); break;
        case BoardProcState::use: use_code(self, p); break;
        case BoardProcState::hidden:
            // hidden's code: no channels, one ja-post, then sleep (it has no post).
            ja_channel_set(p.skel, 0, board_clock(self));
            evaluate_joint_control(p.skel, board_clock(self));
            p.sleeping = true;
            return;
        case BoardProcState::none: break;
        }
    }
    if (p.state == BoardProcState::idle || p.state == BoardProcState::use) {
        board_post(self);
    }
}

} // namespace jak
