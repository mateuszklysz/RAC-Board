#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Mateusz Kłysz
"""Writes the RAC-Board's sounds: assets/sounds/<name>.wav, one for each sound Jak II's board code
plays that Ratchet has none of his own for, under the same name (board-bounce, board-wind, ...), so
the ported code asks for them as it asks for Jak II's. The jumps, the tricks' whooshes and the rail
are Ratchet's own sounds, the game's (src/rac1/fx/effects.cpp); Jak's trick chimes are left out.

Every sound is made here from oscillators, noise and envelopes (nothing recorded, nothing from
either game). The loops (engine, wind, bank, spin) are whole cycles long, so they repeat
without a click. 22050 Hz, 16-bit, mono. A seeded noise source makes the files the same on every run.

    python3 tools/make_sounds.py

Python 3 standard library only.
"""
import math
import random
import struct
import wave
from pathlib import Path

RATE = 22050
OUT = Path(__file__).resolve().parent.parent / "assets" / "sounds"
TAU = 2 * math.pi


def noise(n, seed):
    rnd = random.Random(seed)
    return [rnd.uniform(-1.0, 1.0) for _ in range(n)]


def lowpass(x, cutoff):
    """One-pole low-pass, run twice (and once more around the loop's end, for the loops)."""
    a = 1.0 - math.exp(-TAU * cutoff / RATE)
    y, out = 0.0, []
    for _ in range(2):
        out = []
        for s in x:
            y += a * (s - y)
            out.append(y)
        x = out
    return out


def highpass(x, cutoff):
    lp = lowpass(x, cutoff)
    return [a - b for a, b in zip(x, lp)]


def resonator(x, freq, q):
    """A two-pole band-pass at freq (a ringing metal tone out of noise)."""
    w = TAU * freq / RATE
    r = math.exp(-w / (2 * q))
    c1, c2 = 2 * r * math.cos(w), -r * r
    y1 = y2 = 0.0
    out = []
    for s in x:
        y = (1 - r) * s + c1 * y1 + c2 * y2
        out.append(y)
        y2, y1 = y1, y
    return out


def env(n, attack, release, shape=2.0):
    """An attack then a decay to silence over n samples."""
    a = max(1, int(attack * RATE))
    out = []
    for i in range(n):
        if i < a:
            out.append(i / a)
        else:
            t = (i - a) / max(1, n - a)
            out.append((1 - t) ** shape if release else 1.0)
    return out


def normalise(x, peak=0.8):
    m = max(1e-9, max(abs(s) for s in x))
    return [s * peak / m for s in x]


def mix(*parts):
    n = max(len(p) for p in parts)
    return [sum(p[i] for p in parts if i < len(p)) for i in range(n)]


def mul(a, b):
    return [x * y for x, y in zip(a, b)]


def tone(n, f0, f1=None, harmonics=(1.0,), vibrato=0.0, vib_rate=6.0):
    """A tone gliding from f0 to f1 (exponentially), with the given harmonics."""
    f1 = f1 or f0
    ph, out = 0.0, []
    for i in range(n):
        t = i / max(1, n - 1)
        f = f0 * (f1 / f0) ** t * (1 + vibrato * math.sin(TAU * vib_rate * i / RATE))
        ph += TAU * f / RATE
        out.append(sum(a * math.sin(ph * (k + 1)) for k, a in enumerate(harmonics)))
    return out


def seamless(x, fade):
    """Makes a loop: its last `fade` samples crossfaded into its first."""
    f = int(fade * RATE)
    body = x[:-f]
    for i in range(f):
        t = i / f
        body[i] = body[i] * t + x[len(x) - f + i] * (1 - t)
    return body


def whoosh(dur, f0, f1, seed, peak=0.7):
    n = int(dur * RATE)
    nz = noise(n, seed)
    # A sweeping band: low-pass whose cutoff rises from f0 to f1 and falls back.
    out, y, y2 = [], 0.0, 0.0
    for i, s in enumerate(nz):
        t = i / n
        cut = f0 + (f1 - f0) * math.sin(math.pi * t)
        a = 1.0 - math.exp(-TAU * cut / RATE)
        y += a * (s - y)
        y2 += a * (y - y2)
        out.append(y2)
    return normalise(mul(highpass(out, 120.0), env(n, dur * 0.3, True, 1.5)), peak)


def sounds():
    s = {}
    # The loops: the hover engine's hum, the wind, the bank's hiss, the rail's grind, the spin.
    n = int(1.0 * RATE)
    hum = tone(n + int(0.1 * RATE), 72.0, harmonics=(1.0, 0.5, 0.25, 0.12), vibrato=0.01, vib_rate=5.0)
    buzz = lowpass(noise(len(hum), 1), 300.0)
    s["board-steady"] = normalise(seamless(mix(hum, [0.6 * b for b in buzz]), 0.1), 0.5)
    s["board-wind"] = normalise(seamless(highpass(lowpass(noise(n + 2205, 2), 1800.0), 300.0), 0.1), 0.5)
    s["board-bank"] = normalise(seamless(highpass(lowpass(noise(n + 2205, 3), 4500.0), 1500.0), 0.1), 0.45)
    spin = mul(lowpass(noise(n + 2205, 5), 2500.0),
               [0.5 + 0.5 * math.sin(TAU * 8.0 * i / RATE) for i in range(n + 2205)])
    s["board-spin-loop"] = normalise(seamless(spin, 0.1), 0.5)
    s["board-boost"] = normalise(mix(mul(tone(int(0.55 * RATE), 120, 480, (1.0, 0.5, 0.25)), env(int(0.55 * RATE), 0.05, True, 1.2)),
                                     whoosh(0.55, 500, 5000, 13)), 0.75)
    # Impacts: the landing's thump, the wall's clang, the boots on the deck.
    m = int(0.22 * RATE)
    s["board-bounce"] = normalise(mix(mul(tone(m, 95, 55), env(m, 0.002, True, 3.0)),
                                      mul(lowpass(noise(m, 20), 900.0), env(m, 0.001, True, 6.0))), 0.8)
    m = int(0.45 * RATE)
    clang = mix(*[mul(tone(m, f), env(m, 0.001, True, sh)) for f, sh in ((523, 3), (1187, 4), (1931, 5), (2711, 6))])
    s["board-glance"] = normalise(mix(clang, mul(highpass(noise(m, 21), 2000.0), env(m, 0.001, True, 8.0))), 0.7)
    m = int(0.08 * RATE)
    s["board-boots"] = normalise(mul(resonator(noise(m, 23), 2400.0, 8.0), env(m, 0.001, True, 4.0)), 0.6)
    s["board-turn"] = whoosh(0.22, 300, 2200, 24, 0.5)
    # Water and fire under the board.
    m = int(0.5 * RATE)
    s["swim-enter"] = normalise(mul(lowpass(noise(m, 50), 1500.0), env(m, 0.01, True, 2.0)), 0.6)
    s["get-burned"] = normalise(mul(highpass(noise(m, 51), 2500.0),
                                    [0.6 + 0.4 * math.sin(TAU * 31 * i / RATE) for i in range(m)]), 0.5)
    return s


def write(path, samples):
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(b"".join(struct.pack("<h", int(max(-1.0, min(1.0, v)) * 32767)) for v in samples))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name, samples in sorted(sounds().items()):
        write(OUT / f"{name}.wav", samples)
        print(f"{name}.wav: {len(samples) / RATE:.2f} s")


if __name__ == "__main__":
    main()
