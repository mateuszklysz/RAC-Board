#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Mateusz Kłysz
"""RAC-Board's riding key poses for Ratchet, our own: turns of his joints in model space (x
forward, y left, z up) over his standing pose, and how far his hips drop.

    python3 tools/poses/poses.py <levels>/level_01/mobys/moby_0.glb preview.png [pose ...]
"""
import math, sys, numpy as np
from posetool import *

def rz(deg, v):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([c*v[0]-s*v[1], s*v[0]+c*v[1], v[2]])

Z = np.array([0, 0, 1.0]); X = np.array([1.0, 0, 0]); Y = np.array([0, 1.0, 0])
YAW = -65
F = rz(YAW, X); L = rz(YAW, Y)   # body forward (his chest) and body left after the turn

def add(w, j, *turns):
    w.setdefault(j, []).extend(turns)

def base(crouch=1.0, lean=0.0, hip_tip=8, chest_tip=6, head=-10, twist=30, armsL=(55, 15, 10), armsR=(55, 15, 10)):
    bend = 28*crouch
    w = {}
    add(w, 1, (Z, YAW), (L, hip_tip), (X, lean))
    add(w, 2, (Z, twist*0.4))
    add(w, 4, (Z, twist*0.6), (L, chest_tip))
    add(w, 8, (Z, 10), (L, head))
    add(w, 98, (L, -(bend + 14)), (F, 8)); add(w, 99, (L, 2*bend)); add(w, 100, (L, -bend + 6))
    add(w, 92, (L, -(bend - 16)), (F, -8)); add(w, 93, (L, 2*bend)); add(w, 94, (L, -bend - 6))
    a, b, c = armsR; add(w, 53, (F, -a), (L, -b)); add(w, 54, (F, -c))
    a, b, c = armsL; add(w, 73, (F, a), (L, b)); add(w, 74, (F, c))
    return w, {1: (0, 0, -0.55*crouch)}

POSES = {}
POSES['stance'] = base()
POSES['lean_left'] = base(lean=-16, armsL=(70, 30, 15), armsR=(35, -10, 5))   # into a left turn
POSES['lean_right'] = base(lean=16, armsL=(35, -10, 5), armsR=(70, 30, 15))   # into a right turn
w, t = base(crouch=1.8, hip_tip=20, chest_tip=22, head=-20, armsL=(30, 30, 40), armsR=(30, 30, 40)); t[1] = (0, 0, -1.15)
POSES['duck'] = (w, t)
w, t = base(crouch=0.5, hip_tip=4, chest_tip=0, head=-5, armsL=(95, -10, 20), armsR=(95, -10, 20)); t[1] = (0, 0, -0.2)
POSES['jump'] = (w, t)
w, t = base(crouch=1.6, hip_tip=14, chest_tip=12, head=-14, armsL=(60, 20, 30), armsR=(60, 20, 30)); t[1] = (0, 0, -0.6)
POSES['tuck'] = (w, t)
# nose grab: deep tuck, the front (left) hand down to the board's nose by the front foot
w, t = base(crouch=1.9, hip_tip=18, chest_tip=16, head=-20, armsR=(100, -20, 30), armsL=(10, -20, 0)); t[1] = (0, 0, -0.95)
add(w, 2, (F, 15)); add(w, 4, (F, 15))
POSES['nosegrab'] = (w, t)
# method: back (right) hand grabs the heel edge behind, knees bent back, chest arched, front arm up
w, t = base(crouch=1.4, hip_tip=-6, chest_tip=-18, head=-6, armsL=(120, 30, 20), armsR=(0, 0, 0)); t[1] = (0, 0, -0.75)
add(w, 53, (L, -45), (F, 15)); add(w, 54, (L, -30))
add(w, 1, (F, 10))
POSES['method'] = (w, t)
# kickflip: legs pulled up, arms wide
w, t = base(crouch=2.0, hip_tip=6, chest_tip=8, head=-10, armsL=(100, -30, 10), armsR=(100, -30, 10)); t[1] = (0, 0, -0.7)
POSES['kickflip'] = (w, t)
# landing squash
w, t = base(crouch=2.1, hip_tip=26, chest_tip=20, head=-22, armsL=(40, 40, 50), armsR=(40, 40, 50)); t[1] = (0, 0, -1.3)
POSES['land'] = (w, t)
# kick: front leg kicks out ahead, arms back
w, t = base(crouch=0.6, hip_tip=-4, chest_tip=-6, head=-8, armsL=(40, -40, 10), armsR=(40, -40, 10)); t[1] = (0, 0, -0.3)
add(w, 98, (L, -60)); add(w, 99, (L, -50))
POSES['kick'] = (w, t)
# hit: knocked back, arms flung up
w, t = base(crouch=0.8, hip_tip=-12, chest_tip=-20, head=12, armsL=(150, -20, 10), armsR=(150, -20, 10)); t[1] = (0, 0, -0.4)
POSES['hit'] = (w, t)

if __name__ == "__main__":
    r = Rig(sys.argv[1])
    names = sys.argv[3:] or list(POSES)
    rows = []
    for n in names:
        w, t = POSES[n]
        d = r.deltas_from_world(0, 0, w)
        rows.append(r.render(0, 0, d, trans=t, size=260, views=("side", "front", "q"), title=n))
    cols = 2
    out = Image.new('RGB', (780*cols, 260*((len(rows)+cols-1)//cols)), 'white')
    for i, im in enumerate(rows): out.paste(im, ((i % cols)*780, 260*(i//cols)))
    out.save(sys.argv[2])
