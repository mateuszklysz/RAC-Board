#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Mateusz Kłysz
"""Writes the board's effect sprites: assets/fx/<name>.glb and its <name>.png, each a flat square (x right, y up, 1 game
unit across, facing +z) with a white texture of its own, coloured and faded by the mod as it draws
it (src/rac1/board_fx.cpp). Made here from numbers, so they are the mod's own: a soft glow, a spark
streak, a ring, a puff of dust and a burst.

    python3 tools/make_fx.py

Python 3 standard library only.
"""
import json
import math
import random
import struct
import zlib
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "assets" / "fx"
N = 64


def png(px):
    raw = b"".join(b"\x00" + bytes(c for p in row for c in p) for row in px)

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", N, N, 8, 6, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")


def image(alpha):
    """White texels, alpha from alpha(x, y) with x and y in -1..1."""
    rows = []
    for j in range(N):
        y = (j + 0.5) / N * 2 - 1
        row = []
        for i in range(N):
            x = (i + 0.5) / N * 2 - 1
            a = max(0.0, min(1.0, alpha(x, y)))
            row.append((255, 255, 255, int(round(255 * a))))
        rows.append(row)
    return rows


def glow(x, y):
    r = math.hypot(x, y)
    return (1 - r) ** 2 if r < 1 else 0


def streak(x, y):
    # Long across x, thin across y, brightest in the middle.
    return max(0.0, 1 - abs(x)) ** 0.7 * max(0.0, 1 - abs(y) * 1.1) ** 3


def ring(x, y):
    r = math.hypot(x, y)
    return math.exp(-((r - 0.82) / 0.09) ** 2)


_rng = random.Random(7)
_blobs = [(_rng.uniform(-0.45, 0.45), _rng.uniform(-0.45, 0.45), _rng.uniform(0.3, 0.55)) for _ in range(9)]


def puff(x, y):
    a = 0.0
    for bx, by, br in _blobs:
        d = math.hypot(x - bx, y - by) / br
        if d < 1:
            a = max(a, (1 - d * d) ** 1.5)
    return a * (1 - min(1.0, math.hypot(x, y)) ** 4)


def burst(x, y):
    # A bright core with rays: the boost's flash.
    r = math.hypot(x, y)
    t = math.atan2(y, x)
    rays = 0.5 + 0.5 * math.cos(12 * t) ** 8
    return max(0.0, 1 - r) ** 1.5 * (0.35 + 0.65 * rays) + max(0.0, 1 - r * 3) ** 2


SPRITES = {"glow": glow, "streak": streak, "ring": ring, "puff": puff, "burst": burst}


def glb(name):
    pos = [(-0.5, -0.5, 0.0), (0.5, -0.5, 0.0), (0.5, 0.5, 0.0), (-0.5, 0.5, 0.0)]
    nrm = [(0.0, 0.0, 1.0)] * 4
    uv = [(0.0, 1.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0)]
    idx = [0, 1, 2, 0, 2, 3]
    blobs = [b"".join(struct.pack("<3f", *p) for p in pos), b"".join(struct.pack("<3f", *n) for n in nrm),
             b"".join(struct.pack("<2f", *t) for t in uv), struct.pack("<6I", *idx)]
    views, data = [], b""
    for i, b in enumerate(blobs):
        views.append({"buffer": 0, "byteOffset": len(data), "byteLength": len(b),
                      "target": 34963 if i == 3 else 34962})
        data += b + b"\x00" * (-len(b) % 4)
    doc = {
        "asset": {"version": "2.0", "generator": "rac-board tools/make_fx.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"name": name, "mesh": 0}],
        "meshes": [{"name": name, "primitives": [{
            "attributes": {"POSITION": 0, "NORMAL": 1, "TEXCOORD_0": 2}, "indices": 3, "material": 0}]}],
        "materials": [{"name": "fx_" + name, "alphaMode": "BLEND", "doubleSided": True,
                       "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}, "metallicFactor": 0.0,
                                                "roughnessFactor": 1.0}}],
        "textures": [{"source": 0, "sampler": 0}],
        "samplers": [{"magFilter": 9729, "minFilter": 9729, "wrapS": 33071, "wrapT": 33071}],
        "images": [{"uri": name + ".png"}],
        "buffers": [{"byteLength": len(data)}],
        "bufferViews": views,
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3",
             "min": [-0.5, -0.5, 0.0], "max": [0.5, 0.5, 0.0]},
            {"bufferView": 1, "componentType": 5126, "count": 4, "type": "VEC3"},
            {"bufferView": 2, "componentType": 5126, "count": 4, "type": "VEC2"},
            {"bufferView": 3, "componentType": 5125, "count": 6, "type": "SCALAR"},
        ],
    }
    js = json.dumps(doc, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    body = struct.pack("<II", len(js), 0x4E4F534A) + js + struct.pack("<II", len(data), 0x004E4942) + data
    return struct.pack("<III", 0x46546C67, 2, 12 + len(body)) + body


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name in SPRITES:
        (OUT / f"{name}.glb").write_bytes(glb(name))
        (OUT / f"{name}.png").write_bytes(png(image(SPRITES[name])))
    print("fx:", ", ".join(SPRITES))


if __name__ == "__main__":
    main()
