// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The board's process machinery: GOAL's states (gstate.gc, gkernel.gc) as the target runs them on
// the board, and everything the board keeps beside the generated types. Ported from OpenGOAL
// (jak-project, ISC License, Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md).
//
// How GOAL runs a process each frame (gkernel.gc, run-logic): the state's trans in a thread of its
// own, then the main thread (the state's code, resumed where it last suspended), then the post in a
// thread of its own. `go` (enter-state, gstate.gc) behaves by where it is called:
//   - in the main thread (code, enter, or an event the process sends itself from there): the state
//     changes at once: the old state's exit, the new enter, the new trans, then the new code from its
//     start; the old code is abandoned (here: an exception unwinds it).
//   - in trans: the trans is abandoned, the trans hook cleared, and the main thread does the change
//     right after, in the same frame.
//   - in post, or in an event from another process: the change waits for the main thread's next run
//     (next frame), the trans hook is cleared so that frame runs no trans; the caller carries on.
// next-state is set by every go at once, so `(-> self next-state name)` reads the coming state.
//
// A state's code is a coroutine that suspends once a frame: here a step function with a resume point
// (Co::pc) and its locals in Co; CO_BEGIN / CO_SUSPEND / CO_END spell GOAL's (suspend).
#pragma once

#include <cstdint>

#include "board/anim.hpp"
#include "board/interaction.hpp"
#include "jak/basic_types.hpp"
#include "jak/target.hpp"

namespace jak {

// The states the board's code names: its own (board-states.gc) and the target's it goes to.
enum class StateId : uint8_t {
    none,
    target_board_start, target_board_stance, target_board_duck_stance, target_board_jump,
    target_board_halfpipe, target_board_falling, target_board_jump_kick, target_board_wall_kick,
    target_board_flip, target_board_hold, target_board_trickx, target_board_hit_ground,
    target_board_turn_to, target_board_ride_edge, target_board_grenade, target_board_get_on,
    target_board_pegasus, target_board_get_off, target_board_grab, target_board_clone_anim,
    target_board_hit,
    // The target's states off the board (the adapter's): go hands over to Interaction.
    target_falling, target_hit_ground, target_jump, target_stance, target_grab, target_death,
    target_hit,
    count
};
const char* state_name(StateId id);
inline bool board_state_p(StateId id) {
    return id >= StateId::target_board_start && id <= StateId::target_board_hit;
}

// The arguments a go passes on to enter and code, by what each state takes.
struct StateArgs {
    float f0 = 0.0f, f1 = 0.0f;   // heights (jump, flip, hold, trickx), the wall kick's speed
    Symbol sym{};                 // a symbol argument ('hit, 'halfpipe, #t for ride-edge's air, ...)
    Surface* surf = nullptr;      // target-board-jump's arg2 when it is a surface
    Vec vec;                      // turn-to's direction, the wall kick's normal
    TimeFrame time = 0;           // turn-to's time
    Handle handle = 0;            // get-off, grenade, pegasus, clone-anim
    const AttackInfo* attack = nullptr; // target-board-hit's attack-info
};

// A state's coroutine: where its code resumes and the locals it keeps across suspends.
struct Co {
    int pc = 0;
    float f[4] = {};
    int i[4] = {};
    bool b[4] = {};
    Vec v[2];
    Mat m;
    Symbol s{};
};
#define CO_BEGIN(co) switch ((co).pc) { case 0:
#define CO_SUSPEND(co) do { (co).pc = __LINE__; return; case __LINE__:; } while (0)
#define CO_END(co) } (co).pc = -1;

using EventFn = Symbol (*)(Target&, const EventMessage&);
using EnterFn = void (*)(Target&, const StateArgs&);
using HookFn = void (*)(Target&);
using CodeFn = void (*)(Target&, Co&, const StateArgs&);

struct StateDef {
    StateId id;
    EventFn event;
    EnterFn enter;
    HookFn exit;
    HookFn trans;
    CodeFn code;
    HookFn post;
};
const StateDef& state_def(StateId id);

// joint-mod-mode (joint-mod-h.gc) the board uses, and what the adapter reads off a joint-mod.
enum class JointModMode : uint8_t { flex_blend, rotate };
struct JointMod {
    JointModMode mode = JointModMode::flex_blend;
    float twist_y = 0.0f;  // twist-set! y, rotation units
};

// The board's own process (board-util.gc): its skeleton opens and closes, and follows the target.
enum class BoardProcState : uint8_t { none, idle, use, hidden };
struct BoardProcess {
    bool alive = false;
    BoardProcState state = BoardProcState::none;
    bool sleeping = false;
    Co co;
    bool idle_arg = false;
    JointControl skel;
    JointMod main;
    bool shadow = false;
    Vec trans;
    Quat quat;
    Vec scale;
};

// The PC port's board-trick-tracker (target-board.gc): the combo shown on screen.
struct BoardTrickTracker {
    BoardTricks tricks_in_combo[16] = {};
    uint32_t num_tricks_in_combo = 0;
    float points_in_combo = 0.0f;
    bool combo_in_progress = false;
};

// The control-info overlay at unknown-word04 (collide-target-h.gc): one word GOAL reads as a symbol
// (unknown-spool-anim00), a float or an int by turns. The port keeps what was last written; a float
// read as a symbol is "true" (its bits are never #f's address), as on OpenGOAL.
struct Word04 {
    bool is_float = false;
    Symbol sym{};
    float f = 0.0f;
    void set(Symbol s) { is_float = false; sym = s; }
    void set(bool b) { set(b ? Symbol("#t") : Symbol()); }
    void set_float(float x) { is_float = true; f = x; }
    bool truthy() const { return is_float || static_cast<bool>(sym); }
    Symbol symbol() const { return is_float ? Symbol("float") : sym; }
    float as_float() const { return is_float ? f : 0.0f; }
};

enum class Ctx : uint8_t { external, trans, main, post };

struct BoardRuntime {
    Interaction* io = nullptr;

    // ---- the process (gkernel.gc, gstate.gc) ----
    StateId state = StateId::none;
    StateId next_state = StateId::none;
    StateId exit_frame = StateId::none;  // the state whose exit is on the stack (stack-frame-top)
    EventFn event_hook = nullptr;
    HookFn trans_hook = nullptr;
    HookFn post_hook = nullptr;
    CodeFn code = nullptr;
    bool sleeping = false;               // sleep-code
    Co co;                               // the code's resume point and locals
    Co sub_co;                           // a code another code runs as its own (halfpipe -> stance)
    StateArgs args;                      // what the state was entered with
    bool pending = false;                // a go waiting for the main thread (set-to-run enter-state)
    StateId pending_state = StateId::none;
    StateArgs pending_args;
    Ctx ctx = Ctx::external;
    bool off_board = false;              // the target went to one of its own states

    // ---- the animation (joint-control) ----
    JointControl skel;
    JointMod main;          // (-> self board main), joint 3
    JointMod upper_body;    // (-> self board upper-body), joint 5
    float neck_flex_blend = 1.0f; // (-> self neck flex-blend)

    // ---- what GOAL keeps in fields of other types ----
    Surface* mods_backup = nullptr;  // (-> self board mods-backup)
    Word04 word04;                   // (-> self control unknown-word04 / unknown-spool-anim00)
    Symbol did_move_to_pole{};       // (-> self control did-move-to-pole-or-max-jump-height) as a symbol
    Handle unknown_handle02 = 0;     // (-> self control unknown-handle02)
    // sound ids: the board's loops and one-shots (sound-id fields of board-info and control-info).
    uint32_t engine_sound_id = 0, ride_sound_id = 0, spin_sound_id = 0, wind_sound_id = 0,
             bank_sound_id = 0, unknown_sound_id00 = 0, unknown_sound_id01 = 0,
             unknown_sound_id02 = 0, control_unknown_sound_id00 = 0;
    bool board_setup = false;        // board-info allocated (target-board-setup's (zero? board))
    bool part_control = false;       // (-> self board part-control) created
    TimeFrame shock_effect_time = 0;
    BoardProcess board_proc;         // (-> self board board), the board's own process
    BoardTrickTracker trick_tracker; // *board-trick-tracker*
};

inline BoardRuntime& runtime(Target& self) { return *self.board_rt; }
inline const BoardRuntime& runtime(const Target& self) { return *self.board_rt; }

// ---- the process, for the adapter and the rest of the board ----
// Ties a runtime to the target (the board-info is self.board_info).
void attach(Target& self, BoardRuntime& rt, Interaction& io);
// One frame of the target while it is on the board: trans, code, post, then the board's process.
void board_tick(Target& self);
// go: see the top of this file.
void go(Target& self, StateId state, const StateArgs& args = StateArgs());
// send-event to the target (from another process, or the target itself: `from_self`).
Symbol send_event(Target& self, const EventMessage& e, bool from_self = false);
// (-> self next-state name): the state's name, or #f.
Symbol next_state_name(const Target& self);
bool next_state_is(const Target& self, StateId id);
// remove-exit: the current state's exit leaves the stack.
void remove_exit(Target& self);
// sleep-code: the code stops resuming until the next go.
void sleep_code(Target& self);

} // namespace jak
