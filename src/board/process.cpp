// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// GOAL's process states for the target on the board: enter-state and go-hook (gstate.gc), the
// trans / main thread / post order of run-logic (gkernel.gc) and send-event-function (gstate.gc),
// ported from OpenGOAL (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team;
// THIRD_PARTY.md). runtime.hpp explains the rules.
#include "board/runtime.hpp"

#include "board/board.hpp"
#include "board/collide.hpp"

namespace jak {

namespace {

// The non-local transfers: a go in the main thread abandons the code (GoSignal), a go in trans
// abandons the trans thread (AbandonThread).
struct GoSignal {};
struct AbandonThread {};

const char* const kStateNames[] = {
    nullptr,
    "target-board-start", "target-board-stance", "target-board-duck-stance", "target-board-jump",
    "target-board-halfpipe", "target-board-falling", "target-board-jump-kick",
    "target-board-wall-kick", "target-board-flip", "target-board-hold", "target-board-trickx",
    "target-board-hit-ground", "target-board-turn-to", "target-board-ride-edge",
    "target-board-grenade", "target-board-get-on", "target-board-pegasus", "target-board-get-off",
    "target-board-grab", "target-board-clone-anim", "target-board-hit",
    "target-falling", "target-hit-ground", "target-jump", "target-stance", "target-grab",
    "target-death", "target-hit", "target-attack-air", "target-flop",
};
static_assert(sizeof(kStateNames) / sizeof(kStateNames[0]) == static_cast<int>(StateId::count),
              "one name per state");

// enter-state (gstate.gc), in the main thread: the exits on the stack, then the new state's hooks,
// enter, trans and code. A go inside any of them starts over with the state it names.
void enter_state(Target& self, StateId id, StateArgs args) {
    BoardRuntime& rt = runtime(self);
    for (;;) {
        try {
            rt.sleeping = false;
            rt.state = id;
            if (rt.exit_frame != StateId::none) {
                const StateId e = rt.exit_frame;
                rt.exit_frame = StateId::none;
                state_def(e).exit(self);
            }
            if (!board_state_p(id)) {
                // One of the target's own states: the board's part is over.
                rt.event_hook = nullptr;
                rt.trans_hook = nullptr;
                rt.post_hook = nullptr;
                rt.code = nullptr;
                rt.off_board = true;
                rt.io->go_target_state(self, id, args);
                return;
            }
            const StateDef& def = state_def(id);
            rt.off_board = false;
            rt.event_hook = def.event;
            rt.exit_frame = def.exit ? id : StateId::none;
            rt.post_hook = def.post;
            rt.trans_hook = def.trans;
            rt.code = def.code;
            rt.args = args;
            rt.co = Co();
            rt.sub_co = Co();
            if (def.enter) {
                def.enter(self, rt.args);
            }
            if (def.trans) {
                def.trans(self);
            }
            if (def.code) {
                def.code(self, rt.co, rt.args);
            }
            return;
        } catch (const GoSignal&) {
            id = rt.pending_state;
            args = rt.pending_args;
            rt.pending = false;
        }
    }
}

// The main thread's run: a waiting go first, else the code resumes.
void run_main(Target& self) {
    BoardRuntime& rt = runtime(self);
    const Ctx saved = rt.ctx;
    rt.ctx = Ctx::main;
    try {
        if (rt.pending) {
            rt.pending = false;
            enter_state(self, rt.pending_state, rt.pending_args);
        } else if (!rt.sleeping && rt.code) {
            rt.code(self, rt.co, rt.args);
        }
    } catch (const GoSignal&) {
        rt.pending = false;
        enter_state(self, rt.pending_state, rt.pending_args);
    }
    rt.ctx = saved;
}

} // namespace

const char* state_name(StateId id) { return kStateNames[static_cast<int>(id)]; }

namespace {
// The collision code's send-event to the target (collide.hpp, Target::collide_event), as the board's
// event: the target sends it itself, from the thread it runs in. Its attack-info is the static one
// the reaction builds (static-attack-info), its mask naming the fields it sets.
bool board_collide_event(Target& self, const CollideEvent& ev) {
    EventMessage e;
    e.message = Symbol(ev.message ? ev.message : "");
    AttackInfo info;
    if (ev.param) {
        e.sym[0] = Symbol(ev.param);
        e.num_params = 1;
    }
    if (ev.has_attack) {
        info.id = ev.id;
        info.mask |= attack_mask::id;
        if (ev.mode) {
            info.mode = Symbol(ev.mode);
            info.mask |= attack_mask::mode;
        }
        if (ev.test) {
            info.test = true;
            info.mask |= attack_mask::test;
        }
        if (ev.has_vector) {
            info.mask |= attack_mask::vector;
            info.vector = ev.vector;
        }
        if (ev.has_shove_up) {
            info.shove_up = ev.shove_up;
            info.mask |= attack_mask::shove_up;
        }
        if (ev.angle) {
            info.angle = Symbol(ev.angle);
            info.mask |= attack_mask::angle;
        }
        if (ev.has_control) {
            info.control = ev.control;
            info.mask |= attack_mask::control;
        }
        e.attack = &info;
        e.num_params = 2;
    }
    if (e.message == Symbol("push-transv")) {
        // the velocity and time: the handler's 'push-transv reads them as the vector and the time.
        info.vector = ev.push;
        info.invinc_time = ev.time;
        e.attack = &info;
    }
    return static_cast<bool>(send_event(self, e, true));
}
} // namespace

void attach(Target& self, BoardRuntime& rt, Interaction& io) {
    self.board_rt = &rt;
    rt.io = &io;
    self.collide_event = board_collide_event;
}

void go(Target& self, StateId state, const StateArgs& args) {
    BoardRuntime& rt = runtime(self);
    // go-hook: next-state at once; enter-state wakes the code up.
    rt.next_state = state;
    rt.sleeping = false;
    rt.pending = true;
    rt.pending_state = state;
    rt.pending_args = args;
    switch (rt.ctx) {
    case Ctx::main:
        throw GoSignal();
    case Ctx::trans:
        rt.trans_hook = nullptr;
        throw AbandonThread();
    case Ctx::post:
    case Ctx::external:
        // set-to-run: the main thread changes state on its next run; no trans before that.
        rt.trans_hook = nullptr;
        return;
    }
}

void board_tick(Target& self) {
    BoardRuntime& rt = runtime(self);
    if (rt.off_board && !rt.pending) {
        return;
    }
    if (rt.trans_hook) {
        rt.ctx = Ctx::trans;
        try {
            rt.trans_hook(self);
        } catch (const AbandonThread&) {
        }
    }
    run_main(self);
    if (!rt.off_board && rt.post_hook) {
        rt.ctx = Ctx::post;
        rt.post_hook(self);
    }
    rt.ctx = Ctx::external;
    // The board's own process runs after its parent, the target.
    board_process_tick(self);
}

// send-event-function (gstate.gc): the state's event hook, run as the receiver.
Symbol send_event(Target& self, const EventMessage& e, bool from_self) {
    BoardRuntime& rt = runtime(self);
    if (!rt.event_hook) {
        return Symbol();
    }
    const Ctx saved = rt.ctx;
    if (!from_self) {
        rt.ctx = Ctx::external;
    }
    struct Restore {
        BoardRuntime& rt;
        Ctx c;
        ~Restore() { rt.ctx = c; }
    } restore{rt, saved};
    return rt.event_hook(self, e);
}

Symbol next_state_name(const Target& self) {
    const StateId n = runtime(self).next_state;
    return n == StateId::none ? Symbol() : Symbol(state_name(n));
}

bool next_state_is(const Target& self, StateId id) { return runtime(self).next_state == id; }

void remove_exit(Target& self) { runtime(self).exit_frame = StateId::none; }

void sleep_code(Target& self) { runtime(self).sleeping = true; }

} // namespace jak

namespace jak {

// joint-points (logic-target.gc), what core's post calls: the adapter's (Interaction::joint_points),
// when the board is attached.
void joint_points(Target& self) {
    if (self.board_rt && self.board_rt->io) {
        self.board_rt->io->joint_points(self);
    }
}

} // namespace jak
