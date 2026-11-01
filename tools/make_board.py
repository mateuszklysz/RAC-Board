#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Mateusz Kłysz
"""Writes the RAC-Board's model: assets/board.glb and its texture assets/board.png.

Everything is made here from numbers, so the model is the mod's own (nothing from either game):
a deck with a kicked nose and tail, a grip stripe, a hover body under it with two glowing jets and
tail fins, and an orange and grey paint. The model is in metres, glTF's axes: +y up, +z the nose, the deck's top
at y = 0 (where the rider's feet go), centred on the origin.

    python3 tools/make_board.py            # writes assets/board.glb and assets/board.png

Python 3 standard library only.
"""
import json
import math
import struct
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
OUT = HERE / "assets"

LENGTH = 1.10       # nose to tail
WIDTH = 0.36
THICK = 0.05
KICK = 0.07         # how far the nose and tail rise
KICK_START = 0.36   # from the middle, where the kick begins (fraction of half the length)
BODY_RADIUS = 0.10  # the hover body under the deck
BODY_FLAT = 0.45    # its height as a part of its width
BODY_LENGTH = 0.80
BODY_DROP = 0.055   # its axis below the deck's underside
JET_Z = 0.24        # the two jets, ahead of and behind the middle

# The texture: a 64x64 atlas of flat regions, u right, v down.
TEX = 64
REGIONS = {  # name: (u0, v0, u1, v1) in texels
    "top": (0, 0, 32, 32),
    "side": (32, 0, 64, 16),
    "bottom": (32, 16, 64, 32),
    "pod": (0, 32, 32, 64),
    "glow": (32, 32, 64, 64),
}


def texture():
    """The atlas as RGBA rows: painted per region."""
    px = [[(0, 0, 0, 255)] * TEX for _ in range(TEX)]

    def fill(region, fn):
        u0, v0, u1, v1 = REGIONS[region]
        for y in range(v0, v1):
            for x in range(u0, u1):
                px[y][x] = fn((x - u0) / (u1 - u0), (y - v0) / (v1 - v0))

    def top(u, v):
        # Orange paint with a dark grip stripe down the middle and two thin cream lines.
        if abs(u - 0.5) < 0.18:
            noise = ((int(u * 97) * 31 + int(v * 89) * 17) % 7) * 3
            return (52 + noise, 50 + noise, 56 + noise, 255)
        if abs(abs(u - 0.5) - 0.24) < 0.025:
            return (240, 226, 190, 255)
        shade = int(18 * v)
        return (236 - shade, 120 - shade // 2, 28, 255)

    fill("top", top)
    fill("side", lambda u, v: (150 - int(30 * v), 156 - int(30 * v), 166 - int(30 * v), 255))
    fill("bottom", lambda u, v: (70, 74, 82, 255) if (int(u * 8) + int(v * 2)) % 2 else (82, 86, 94, 255))
    fill("pod", lambda u, v: (96 + int(60 * abs(math.sin(v * math.pi))),) * 3 + (255,))
    fill("glow", lambda u, v: (90, 230, 255, 255) if abs(v - 0.5) < 0.3 else (40, 140, 200, 255))
    return px


def png(px):
    raw = b"".join(b"\x00" + bytes(c for p in row for c in p) for row in px)

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", TEX, TEX, 8, 6, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")


class Mesh:
    def __init__(self):
        self.pos, self.nrm, self.uv, self.idx = [], [], [], []

    def uvmap(self, region, s, t):
        u0, v0, u1, v1 = REGIONS[region]
        # Half a texel in from each edge, so filtering never reads the next region.
        s = min(max(s, 0.0), 1.0)
        t = min(max(t, 0.0), 1.0)
        return ((u0 + 0.5 + s * (u1 - u0 - 1)) / TEX, (v0 + 0.5 + t * (v1 - v0 - 1)) / TEX)

    def vert(self, p, n, region, s, t):
        self.pos.append(p)
        ln = math.sqrt(sum(c * c for c in n)) or 1.0
        self.nrm.append(tuple(c / ln for c in n))
        self.uv.append(self.uvmap(region, s, t))
        return len(self.pos) - 1

    def quad(self, a, b, c, d):
        self.idx += [a, b, c, a, c, d]


def outline(n=40):
    """The deck's outline in (x, z): a rounded rectangle (superellipse ends)."""
    pts = []
    hl, hw = LENGTH / 2, WIDTH / 2
    for i in range(n):
        a = 2 * math.pi * i / n
        c, s = math.cos(a), math.sin(a)
        x = hw * math.copysign(abs(c) ** 0.5, c)
        z = hl * math.copysign(abs(s) ** 0.25, s)
        pts.append((x, z))
    return pts


def kick(z):
    """How far the deck rises at z: flat in the middle, a smooth curve up at both ends."""
    f = abs(z) / (LENGTH / 2)
    if f <= KICK_START:
        return 0.0
    t = (f - KICK_START) / (1 - KICK_START)
    return KICK * t * t


def deck(m):
    ring = outline()
    n = len(ring)
    hl, hw = LENGTH / 2, WIDTH / 2
    # Top and bottom as fans from the middle line: grid rows along z for the kick's curve.
    for side, y_off, sign, region in ((0, 0.0, 1, "top"), (1, -THICK, -1, "bottom")):
        centre = m.vert((0.0, y_off, 0.0), (0, sign, 0), region, 0.5, 0.5)
        first = len(m.pos)
        for x, z in ring:
            y = kick(z) + y_off
            m.vert((x, y, z), (0, sign, -math.copysign(1, z) * (0.6 if kick(z) else 0.0)), region,
                   x / WIDTH + 0.5, z / LENGTH + 0.5)
        for i in range(n):
            a, b = first + i, first + (i + 1) % n
            m.idx += [centre, b, a] if sign > 0 else [centre, a, b]
    # The rim: a band between the top and bottom outlines.
    for i in range(n):
        x0, z0 = ring[i]
        x1, z1 = ring[(i + 1) % n]
        nx, nz = (z1 - z0), -(x1 - x0)
        s0, s1 = i / n, (i + 1) / n
        a = m.vert((x0, kick(z0), z0), (nx, 0, nz), "side", s0, 0.0)
        b = m.vert((x1, kick(z1), z1), (nx, 0, nz), "side", s1, 0.0)
        c = m.vert((x1, kick(z1) - THICK, z1), (nx, 0, nz), "side", s1, 1.0)
        d = m.vert((x0, kick(z0) - THICK, z0), (nx, 0, nz), "side", s0, 1.0)
        m.quad(a, d, c, b)
    _ = hl, hw


def underbody(m):
    """The hover body: a flattened capsule along the deck's length under it, two glowing jets on
    its underside (front and back) and two small fins at the tail."""
    seg, rings = 18, 14
    yc = -THICK - BODY_DROP
    hl = BODY_LENGTH / 2
    # Rings along z; each ring's radius narrows toward the ends (a capsule's profile).
    grid = []
    for r in range(rings + 1):
        z = -hl + BODY_LENGTH * r / rings
        f = abs(z) / hl
        rad = BODY_RADIUS * math.sqrt(max(0.0, 1 - f ** 4))
        row = []
        for k in range(seg + 1):
            a = 2 * math.pi * k / seg
            cx, cy = math.sin(a), math.cos(a)
            region = "pod"
            # The jets: the bottom of the body near each pod station glows.
            if cy < -0.55 and any(abs(z - zc) < 0.07 for zc in (-JET_Z, JET_Z)):
                region = "glow"
            row.append(m.vert((rad * cx, yc + rad * cy * BODY_FLAT, z), (cx, cy / BODY_FLAT, 0), region,
                              k / seg, r / rings))
        grid.append(row)
    for r in range(rings):
        for k in range(seg):
            m.quad(grid[r][k], grid[r][k + 1], grid[r + 1][k + 1], grid[r + 1][k])
    # Two struts joining the body to the deck.
    w = 0.025
    for zc in (-JET_Z, JET_Z):
        corners = [(-w, -THICK, zc - w), (w, -THICK, zc - w), (w, -THICK, zc + w), (-w, -THICK, zc + w)]
        low = [(x, yc, z) for x, _, z in corners]
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            c, d = low[(i + 1) % 4], low[i]
            nx, nz = (b[2] - a[2]), -(b[0] - a[0])
            q = [m.vert(v, (nx, 0, nz), "side", s, t) for v, s, t in ((a, 0, 0), (b, 1, 0), (c, 1, 1), (d, 0, 1))]
            m.quad(q[0], q[3], q[2], q[1])
    # Tail fins: thin plates angled out and down, both faces.
    for sx in (-1, 1):
        root0 = (sx * 0.06, -THICK, -hl + 0.02)
        root1 = (sx * 0.06, -THICK, -hl + 0.20)
        tip = (sx * 0.17, -THICK - 0.10, -hl - 0.02)
        n = (sx * 0.7, -0.7, 0.0)
        a, b, c = (m.vert(v, n, "side", s, t) for v, s, t in ((root0, 0, 0), (root1, 1, 0), (tip, 0, 1)))
        m.idx += [a, b, c] if sx > 0 else [a, c, b]
        n2 = (-n[0], -n[1], 0.0)
        a, b, c = (m.vert(v, n2, "side", s, t) for v, s, t in ((root0, 0, 0), (root1, 1, 0), (tip, 0, 1)))
        m.idx += [a, c, b] if sx > 0 else [a, b, c]


def glb(m):
    def pack(fmt, rows):
        return b"".join(struct.pack(fmt, *r) for r in rows)

    pos = pack("<3f", m.pos)
    nrm = pack("<3f", m.nrm)
    uv = pack("<2f", m.uv)
    idx = struct.pack(f"<{len(m.idx)}I", *m.idx)
    blobs, views, offset = [pos, nrm, uv, idx], [], 0
    data = b""
    for i, b in enumerate(blobs):
        views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(b),
                      "target": 34963 if i == 3 else 34962})
        data += b
        offset += len(b)
    lo = [min(p[i] for p in m.pos) for i in range(3)]
    hi = [max(p[i] for p in m.pos) for i in range(3)]
    doc = {
        "asset": {"version": "2.0", "generator": "rac-board tools/make_board.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"name": "rac_board", "mesh": 0}],
        "meshes": [{"name": "rac_board", "primitives": [{
            "attributes": {"POSITION": 0, "NORMAL": 1, "TEXCOORD_0": 2},
            "indices": 3, "material": 0}]}],
        "materials": [{"name": "rac_board", "pbrMetallicRoughness": {
            "baseColorTexture": {"index": 0}, "metallicFactor": 0.0, "roughnessFactor": 0.8}}],
        "textures": [{"source": 0, "sampler": 0}],
        "samplers": [{"magFilter": 9729, "minFilter": 9729, "wrapS": 33071, "wrapT": 33071}],
        "images": [{"uri": "board.png"}],
        "buffers": [{"byteLength": len(data)}],
        "bufferViews": views,
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": len(m.pos), "type": "VEC3", "min": lo, "max": hi},
            {"bufferView": 1, "componentType": 5126, "count": len(m.nrm), "type": "VEC3"},
            {"bufferView": 2, "componentType": 5126, "count": len(m.uv), "type": "VEC2"},
            {"bufferView": 3, "componentType": 5125, "count": len(m.idx), "type": "SCALAR"},
        ],
    }
    js = json.dumps(doc, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    data += b"\x00" * (-len(data) % 4)
    body = struct.pack("<II", len(js), 0x4E4F534A) + js + struct.pack("<II", len(data), 0x004E4942) + data
    return struct.pack("<III", 0x46546C67, 2, 12 + len(body)) + body


def main():
    m = Mesh()
    deck(m)
    underbody(m)
    OUT.mkdir(exist_ok=True)
    (OUT / "board.glb").write_bytes(glb(m))
    (OUT / "board.png").write_bytes(png(texture()))
    print(f"board.glb: {len(m.pos)} vertices, {len(m.idx) // 3} triangles")


if __name__ == "__main__":
    main()
