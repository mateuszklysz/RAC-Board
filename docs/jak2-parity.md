# Jak II parity

What the mod takes from Jak II as it is, what it changes for Ratchet & Clank, and what is missing.

## Same as Jak II

These run Jak II's own code, ported line by line from OpenGOAL.

| Part | Jak II source |
|---|---|
| All board states: get on, stance, jump, duck jump, air, tricks, grind, halfpipe, wall kick, hit, get off | `board-states.gc` |
| Physics: speed, turning, air control, landing shock, the cushion | `target-board.gc` |
| Board surfaces (walk, ride, duck, air, jump, spin, flip, grind) | `target-board.gc` |
| Tricks, the trick list and points; the boost for landing a spin straight | `target-board.gc`, `board-states.gc` |
| Rail grinds and halfpipe lips (edge grabs) | `collide-edge-grab.gc` |
| Collision: the collide cache, the target's shape, its reaction | `collide-cache.gc`, `collide-shape.gc`, `collide-reaction-target.gc` |
| Taking a hit: invulnerable time, same-attack timeout, knock-back | `target-handler.gc`, `target-util.gc`, `target-death.gc` |
| When the board may be summoned | `want-to-board?` in `board-h.gc` |
| The pad: buttons, the stick's angle and speed | `pad.gc` |

Because it is the same code, the board keeps Jak II's quirks and glitches too.
See [glitches.md](glitches.md).

## Changed for Ratchet & Clank

| What | How |
|---|---|
| Size | Jak II's world is scaled to Ratchet (0.42), so the board feels the same for his size |
| Ground | The level's collision from OpenRAC. Every surface counts as plain ground (no ice, tar or sand) |
| Rails | The game's grind rails become thin bars the board can grind |
| Buttons | Get on and off with D-pad Up (or D-pad Down, L2+R2, or R2). Duck and spin can swap (L1/R1) |
| Animations | Our own clips for Ratchet, made in the style of Jak's and mixed by Jak II's joint control |
| Effects | Particles and sounds made for the mod, shaped after Jak II's |
| Heat haze | The air behind the jet wobbles (part 432), drawn with OpenRAC's heat haze |
| Board camera | Jak II's string camera with the board's settings follows the board; the right stick swings it. A wall pulls it in toward Ratchet (Jak II swings round the wall instead) |
| Trick names | Shown as one line with the combo and points, cleared 3 s after the combo ends |
| Health | Ratchet's own nanotech. A hit while riding takes his health like Jak's |
| Hits on enemies | The board's spin and touch attacks hit the game's creatures |
| Sidekick | Clank stays on Ratchet's back, as Daxter stays on Jak's shoulder |
| Water, death floors, deep falls | Ratchet gets off and the game's own rules run (he swims, sinks or respawns), as on foot |
| Game scenes | When the game moves Ratchet or changes his state (a cutscene, a teleport), he gets off |
| Moving platforms | The board moves and turns with the platform it stands on |

## Missing, and why

| Missing | Why |
|---|---|
| Rumble | OpenRAC does not drive the pad's motors for Ratchet & Clank yet |
| Board music | Jak II switches the sound bank and music. There is no music to switch to |
| Jak's voice ("oof" on a hit) | No sound from either game's disc is shipped |
| Ice and tar surfaces | Ratchet & Clank's slippery ground has no Jak II material to map |
| Trick-point pickups and races | These belong to Jak II's levels, not to the board |
| Board grenade, cutscene clones | Jak II story features with no place in Ratchet & Clank |
