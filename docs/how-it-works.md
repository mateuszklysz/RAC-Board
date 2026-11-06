# How it works

RAC-Board runs Jak II's board code inside Ratchet & Clank.
The board code is ported from [OpenGOAL](https://github.com/open-goal/jak-project).
A small adapter connects it to the game.

## One frame

1. OpenRAC calls the mod instead of Ratchet's own update (`rac1.player.update`).
2. Not riding: the game's update runs as normal.
3. Riding: the mod feeds the pad to Jak II's pad code.
4. Jak II's target runs one frame: its state, physics and collision.
5. The mod draws Ratchet in his riding pose and the board under him.
6. The board's camera (Jak II's string camera) puts the game's camera behind him.
7. When the board's code leaves the board, the game's update takes over again.

## Folders

| Folder | What is in it |
|---|---|
| `src/jak` | Jak II's engine parts the board needs: the target's movement, the pad, surfaces |
| `src/jak/collide` | Jak II's collision: the collide cache, the target's shape, edge grabs |
| `src/jak/math` | Jak II's maths |
| `src/board` | The board itself: states, physics, tricks, hits |
| `src/rac1` | The adapter to Ratchet & Clank: pad, ride, camera, health, hits |
| `src/rac1/world` | The level's collision, the ground under the board, grind rails, the held item |
| `src/rac1/fx` | Sounds, particles and the trick line |
| `src/rac1/anim` | Ratchet's riding clips |
| `tests` | Unit tests for the ported code |
| `tools` | Generators for types, the board model, sounds and clips |

Each ported file names the GOAL file it comes from at the top.

## The two sides

The board code never calls the game directly. It talks to three interfaces:

- `jak::World` gives collision triangles (`src/jak/host.hpp`).
- `jak::Effects` plays sounds and particles (`src/jak/host.hpp`).
- `jak::Interaction` is everything else: health, hits, other actors (`src/board/interaction.hpp`).

The adapter in `src/rac1` implements all three. The tests implement them with fakes.

## Units and axes

- Jak II uses 4096 units per metre and y up.
- Ratchet & Clank uses game units and z up.
- Jak II's world is scaled to Ratchet's size (0.42), so jumps and speeds keep Jak's feel.
- `src/rac1/convert.hpp` does every conversion.

## Grind rails

Ratchet & Clank's rails are paths, not collision. Jak II grinds on edges of collision.
So the mod turns each rail into a thin bar (`src/rac1/world/rails.cpp`).
The board finds the bar's top edge and grinds it, as it would in Jak II.

- The rails come from the level's own rail table, so every level with rails works
  (Kerwan, Blarg, Batalia, Oltanis, Kalebo III, Veldin 2).
- Closed rails (loops) join their last point to their first, as the game does.
- Hold R1 or R2 with Cross to grab a rail (the grind_button setting; Jak II's Circle is an option).

## The game's own rules

The board hands Ratchet back to the game when the game should decide (`src/rac1/ride.cpp`):

- Water, death floors and sinking liquids under the board (`src/rac1/world/ground.cpp`).
- A fall below the level's fall height.
- The game moving Ratchet or changing his state (cutscenes, teleports).

Ratchet rides with the wrench, and gets on at once. A weapon or gadget in his hand is put away by
the game's own request (the one its vehicles use, `src/rac1/world/hand.cpp`), set each frame he
rides: the game's item code still runs during the ride (`hero_set_pose` runs it), so it swaps the
item for the wrench with the hand's pieces hidden. A weapon or gadget picked in the pause menu, or
given by a pickup, waits until he gets off. Circle and Triangle are kept from the game while he
rides, so nothing is fired and the quick select stays shut. An attack under way as he gets on (a
wrench swing, control mode 6) is ended there, as `hero_set_pose` ends a slam, since the game would
play it on as a new swing after the ride.

On a moving platform the board moves and turns with the platform's moby.
