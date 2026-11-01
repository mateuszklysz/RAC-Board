# RAC-Board

Jak II's JET-Board in Ratchet & Clank, for OpenRAC's native port (PAL and NTSC-U discs).
Ratchet gets on a hoverboard and rides it with Jak II's own logic: the same physics, the same
states and tricks (ollie, duck jump, spins, flips, grabs, grinds on rails, wall kicks, halfpipes)
and the same quirks, because it is Jak II's code, ported from
[OpenGOAL](https://github.com/open-goal/jak-project) (THIRD_PARTY.md). Clank stays on Ratchet's
back while he rides, as Daxter stays on Jak's shoulder.

Ratchet rides in his own animation, made for this mod in the style of Jak's: side-on stance with
bent knees and arms out, leaning into turns, ducking, tucks and grabs in the air, kicks, and the
whole body turning with the board in flips and spins. The animation lengths the logic times its
tricks by are Jak II's (only the numbers, read from a disc).

**Work in progress.** It needs OpenRAC with mod interface 1.12 (below).

## Controls

On the board Ratchet does not fire, swing the wrench or open the Quick Select, so the board takes
those buttons only while he rides; on foot the game's controls are as they always are.

| Action | Pad (default) | Jak II |
|---|---|---|
| Get on and off | D-pad Down (setting: L2 and R2 together, or R2) | R2 |
| Jump (hold for higher) | Cross | Cross |
| Duck, duck jump; in the air with the stick: grabs and tricks | R1 | L1 |
| Spin (stick left or right), flip (stick up or down) | L1 | R1 |
| Hold on to a rail | Circle or Square | Circle or Square |

**Crouch on R1** (on) swaps L1 and R1 so ducking is where Ratchet crouches; off gives Jak II's
buttons (duck and grabs on L1, spins and flips on R1). The keyboard gets on and off with **B** (rebindable); with the keyboard-mouse mod the
rest goes through the pad buttons it already maps.

## What it needs from OpenRAC

Mod interface 1.12: the hook on the hero's whole update (`rac1.player.update`), `hero_set_pose`
and `hero_set_joints` (the riding poses), the level's collision triangles, props (the board model,
with the hero's shadow) and the mod's own sounds. Until an OpenRAC release has it, build against the SDK folder of an OpenRAC checkout that
does (`-DOPENRAC_SDK=<checkout>/sdk`).

## Build

    python3 tools/fetch_sdk.py        # the SDK sdk.lock pins, into .sdk/
    cmake -S . -B build -G Ninja
    cmake --build build
    ctest --test-dir build

Push a tag `v<version>` and the build workflow publishes the release OpenRAC's launcher installs
from (paste this repository's address in the launcher's Mods tab).

## How it is made

| Folder | What |
|---|---|
| `src/jak` | Jak II's engine routines the board uses: maths, the target's movement, collision |
| `src/board` | the board: its states, physics and tricks |
| `src/rac1` | the adapter: Ratchet & Clank's hero, pad, collision and drawing |
| `tools` | generators: `gen_jak_types.py` (the GOAL structures), the board's model and sounds, `poses/` (Ratchet's riding poses) |
| `tests` | the ported code against values worked out from the GOAL |

`src/jak/types_gen.hpp` is written by `tools/gen_jak_types.py` from an OpenGOAL checkout, and
`src/rac1/ride_poses.inc` by `tools/poses/export.py` from the poses in `tools/poses/poses.py` (it
reads Ratchet's skeleton from your own OpenRAC level data; `poses.py` previews them to a PNG).

## Contributing

Commits and pull requests follow [CONTRIBUTING.md](CONTRIBUTING.md): Conventional Commits, no AI
agent trailers, footers, session links or agent identities. The `ai-rules` workflow checks every
pull request; `scripts/commit-msg.py` is the same check as a local git hook.

## Licence

MIT (LICENSE). The files ported from OpenGOAL are also under OpenGOAL's ISC licence
(THIRD_PARTY.md).
