# Third-party code

## OpenGOAL (jak-project)

The board's logic is Jak II's, ported to C++ from OpenGOAL's decompilation of Jak II
(`goal_src/jak2/engine/target/board/`, and the engine routines it uses in
`goal_src/jak2/engine/math`, `geometry`, `collide`, `physics` and `target`):
<https://github.com/open-goal/jak-project>. Files under `src/jak/` and `src/board/` that carry
`SPDX-License-Identifier: MIT AND ISC` are such ports; each says which GOAL file and
function it follows.

    ISC License

    Copyright (c) 2020-2026 OpenGOAL Team

    Permission to use, copy, modify, and/or distribute this software for any
    purpose with or without fee is hereby granted, provided that the above
    copyright notice and this permission notice appear in all copies.

    THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
    WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
    MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
    ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
    WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
    ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
    OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

Nothing from Jak II's or Ratchet & Clank's discs is in this mod: no model, texture, sound,
animation or text. What the port keeps of Jak II is its code's logic and the numbers in it
(speeds, heights, times).
