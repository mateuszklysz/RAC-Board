#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Mateusz Kłysz
"""RAC-Board's riding clips for Ratchet, our own animation in Jak II's style: one clip per animation
the board's logic plays, with the same name and number of frames as Jak's (src/board/anim_table.cpp),
so the logic's frame numbers, seeks and blends drive them as they drive Jak's.

A clip is a few keyframes of pose parameters (pose() below: knees, hips, back, arms, head, face,
ears, tail), the board's place under his feet and his whole-body flip. Every parameter is a smooth
curve through its keys (Catmull-Rom), sampled at each frame; looped clips wrap.

    python3 tools/poses/clips.py <levels>/level_01/mobys/moby_0.glb strip.png [clip ...]
    python3 tools/poses/clips.py <levels>/level_01/mobys/moby_0.glb src/rac1/ride_clips.inc
"""
import math, sys, numpy as np
from posetool import *
from poses import CHANNELS, POSES, Z, X, Y, YAW, add, rz

FRAMES = {}  # name -> number of frames, read from anim_table.cpp


def read_frames(path):
    import re
    for m in re.finditer(r'\{"([a-z0-9-]+)-ja", (\d+),', open(path).read()):
        FRAMES[m.group(1)] = int(m.group(2))


# ---- the pose: every parameter has a default; a key sets some of them ----
DEFAULT = dict(
    crouch=1.0, crouch_f=0.0, crouch_b=0.0, lean=0.0, hip_tip=3.0, chest_tip=3.0, head=-10.0, twist=30.0,
    head_turn=10.0, tilt=0.0, sway=0.0,
    l_out=55.0, l_fwd=15.0, l_elbow=10.0, r_out=55.0, r_fwd=15.0, r_elbow=10.0,
    lift=0.0,  # feet (and the board) up towards the hips, model units
    # face, ears and tail: the secondary-motion channels' values (poses.py CHANNELS)
    jaw=0.05, blink=0.0, brows=-0.2, snarl=0.0, ear_l_up=0.0, ear_r_up=0.0, ear_l_back=0.3,
    ear_r_back=0.3, tail_side=0.0, tail_tip=0.0, tail_up=0.2,
    # the board under his feet: up (z, model units), nose up (pitch), toe edge down (roll), yaw
    b_pitch=0.0, b_roll=0.0, b_yaw=0.0,
    # the whole body (and the board) turned about his middle: forward flip (degrees, nose down)
    flip=0.0,
    yaw=YAW,  # the hips turned side-on to the board (degrees about up)
)
FACE = ['jaw', 'blink', 'brows', 'snarl', 'ear_l_up', 'ear_r_up', 'ear_l_back', 'ear_r_back',
        'tail_side', 'tail_tip', 'tail_up']


def pose(p):
    """World turns per joint (model space over his standing pose) and the hips' move."""
    w = {}
    c = p['crouch']
    # his chest's forward and left after the hips' turn: the limbs bend about these, so they stay
    # his own while the turn goes from facing ahead (get-on, get-off) to side-on
    F = rz(p['yaw'], X); L = rz(p['yaw'], Y)
    add(w, 1, (Z, p['yaw']), (L, p['hip_tip']), (X, p['lean']))
    add(w, 2, (Z, p['twist'] * 0.4), (X, p['sway']))
    add(w, 4, (Z, p['twist'] * 0.6), (L, p['chest_tip']), (X, -0.5 * p['lean']))
    add(w, 8, (Z, p['head_turn']), (L, p['head']), (X, p['tilt'] - 0.4 * p['lean']))
    bf, bb = 28 * (c + p['crouch_f']), 28 * (c + p['crouch_b'])
    add(w, 98, (L, -(bf + 14)), (F, 8)); add(w, 99, (L, 2 * bf)); add(w, 100, (L, -bf + 6))
    add(w, 92, (L, -(bb - 16)), (F, -8)); add(w, 93, (L, 2 * bb)); add(w, 94, (L, -bb - 6))
    add(w, 53, (F, -p['r_out']), (L, -p['r_fwd'])); add(w, 54, (F, -p['r_elbow']))
    add(w, 73, (F, p['l_out']), (L, p['l_fwd'])); add(w, 74, (F, p['l_elbow']))
    drop = -0.55 * (c + 0.5 * (p['crouch_f'] + p['crouch_b'])) + p['lift']
    return w, (0.0, 0.0, drop)


def catmull(keys, t, loop, length):
    """keys: [(frame, value)] sorted; a smooth curve through them at frame t."""
    if len(keys) == 1:
        return keys[0][1]
    fs = [k[0] for k in keys]; vs = [k[1] for k in keys]
    if loop:
        fs = [fs[-1] - length] + fs + [fs[0] + length, fs[1] + length if len(fs) > 1 else fs[0] + 2 * length]
        vs = [vs[-1]] + vs + [vs[0], vs[1] if len(vs) > 1 else vs[0]]
        t = (t - fs[1]) % length + fs[1]
        i = 1
    else:
        if t <= fs[0]: return vs[0]
        if t >= fs[-1]: return vs[-1]
        fs = [2 * fs[0] - fs[1]] + fs + [2 * fs[-1] - fs[-2]]
        vs = [vs[0]] + vs + [vs[-1]]
        i = 1
    while i < len(fs) - 2 and t > fs[i + 1]:
        i += 1
    p0, p1, p2, p3 = vs[i - 1], vs[i], vs[i + 1], vs[i + 2]
    span = fs[i + 1] - fs[i]
    u = 0.0 if span == 0 else (t - fs[i]) / span
    # a centripetal-free uniform Catmull-Rom, scaled by the neighbouring spans' lengths
    m1 = (p2 - p0) / max(fs[i + 1] - fs[i - 1], 1e-6) * span
    m2 = (p3 - p1) / max(fs[i + 2] - fs[i], 1e-6) * span
    u2, u3 = u * u, u * u * u
    return (2 * u3 - 3 * u2 + 1) * p1 + (u3 - 2 * u2 + u) * m1 + (-2 * u3 + 3 * u2) * p2 + (u3 - u2) * m2


class Clip:
    def __init__(self, name, keys, loop=False, by_frame=None):
        self.name, self.keys, self.loop, self.by_frame = name, keys, loop, by_frame

    def params(self, f):
        n = FRAMES[self.name]
        out = dict(DEFAULT)
        if self.by_frame:
            out.update(self.by_frame(f, n))
            return out
        names = set(k for _, d in self.keys for k in d)
        for k in names:
            ks = [(fr, d[k]) for fr, d in self.keys if k in d]
            out[k] = catmull(ks, f, self.loop, n)
        return out


def merge(*ds):
    o = {}
    for d in ds: o.update(d)
    return o


# ---- the looks the clips are made from ----
STANCE = dict()
DUCK = dict(crouch=1.8, hip_tip=20, chest_tip=22, head=-20, l_out=30, l_fwd=30, l_elbow=40, r_out=30, r_fwd=30,
            r_elbow=40, snarl=0.9, brows=-0.8, jaw=0.15, ear_l_back=0.8, ear_r_back=0.8, tail_up=-0.2)
TUCK = dict(crouch=1.6, hip_tip=14, chest_tip=12, head=-14, l_out=60, l_fwd=20, l_elbow=30, r_out=60, r_fwd=20,
            r_elbow=30, lift=0.55, jaw=0.35, brows=0.7, ear_l_up=0.5, ear_r_up=0.4, tail_up=0.6)


def lean_of(f, n):
    """The turn clips' frame to a lean: frame 0 all the way into a left turn, the middle level."""
    return (f - (n - 1) / 2) / ((n - 1) / 2)


def turn_look(l, extra=None):
    """Into a turn: hips roll in, the inside arm drops and the outside one swings out, the head looks
    in, the tail swings out, the board's edge digs in."""
    d = dict(lean=16 * l, l_out=55 - 20 * l, l_fwd=15 - 25 * l, l_elbow=10 + 5 * abs(l),
             r_out=55 + 20 * l, r_fwd=15 + 25 * l, r_elbow=10 + 5 * abs(l),
             head_turn=10 - 22 * l, tilt=-6 * l, crouch=1.0 + 0.25 * abs(l), tail_side=0.9 * l,
             b_roll=-7 * l, ear_l_back=0.3 + 0.3 * abs(l), ear_r_back=0.3 + 0.3 * abs(l))
    if extra:
        for k, v in extra.items():
            d[k] = d.get(k, DEFAULT[k]) + v if k in ('crouch', 'hip_tip', 'chest_tip', 'head') else v
    return d


CLIPS = {}


def clip(name, keys=None, loop=False, by_frame=None):
    CLIPS[name] = Clip(name, keys or [], loop, by_frame)


clip('jakb-board-stance', [(0, STANCE)])
# ground-tween channel 0: the frame is the turn
clip('jakb-board-turn', by_frame=lambda f, n: turn_look(lean_of(f, n)))
# channel 1: the board nose up (uphill: forward over the front foot) or down (back on the rear)
clip('jakb-board-turn-up', by_frame=lambda f, n: merge(turn_look(lean_of(f, n)),
     dict(crouch_f=0.5, crouch_b=-0.2, hip_tip=20, chest_tip=16, head=-18, b_pitch=14, l_fwd=30, r_fwd=30)))
clip('jakb-board-turn-down', by_frame=lambda f, n: merge(turn_look(lean_of(f, n)),
     dict(crouch_f=-0.25, crouch_b=0.45, hip_tip=-4, chest_tip=-8, head=-2, b_pitch=-14, l_fwd=-10, r_fwd=-10,
          l_out=75, r_out=75)))
# channel 2: the board's toe edge down (leaning over the toes) or heel edge down (back on the heels)
clip('jakb-board-turn-right', by_frame=lambda f, n: merge(turn_look(lean_of(f, n)),
     dict(hip_tip=18, chest_tip=14, sway=-6, b_roll=14, l_out=40, r_out=40, crouch=1.3)))
clip('jakb-board-turn-left', by_frame=lambda f, n: merge(turn_look(lean_of(f, n)),
     dict(hip_tip=-6, chest_tip=-6, sway=6, b_roll=-14, l_out=80, r_out=80, l_fwd=-15, r_fwd=-15, crouch=1.2)))
# channel 3: the duck, the same turn
clip('jakb-board-duck-turn', by_frame=lambda f, n: merge(turn_look(lean_of(f, n)), DUCK,
     dict(lean=12 * lean_of(f, n), head_turn=10 - 18 * lean_of(f, n))))
# in the air the stance's duck channel plays this: tucked, the same turn
clip('jakb-board-air-turn', by_frame=lambda f, n: merge(turn_look(lean_of(f, n)), TUCK,
     dict(lean=12 * lean_of(f, n), head_turn=10 - 18 * lean_of(f, n), b_roll=-4 * lean_of(f, n))))

# The jump: down to load, the pop (board nose up, legs long), the feet and board come up, tucked.
clip('jakb-board-jump', [
    (0, dict(crouch=1.9, hip_tip=22, chest_tip=18, head=-18, l_out=35, r_out=35, l_fwd=35, r_fwd=35, l_elbow=40,
             r_elbow=40, jaw=0.1, snarl=0.5, ear_l_up=-0.4, ear_r_up=-0.4, tail_up=-0.3)),
    (2, dict(crouch=0.35, hip_tip=2, chest_tip=-4, head=-2, l_out=105, r_out=100, l_fwd=-15, r_fwd=-10,
             l_elbow=25, r_elbow=20, b_pitch=20, jaw=0.5, snarl=0.0, brows=1.0, ear_l_up=0.6, ear_r_up=0.5,
             tail_up=0.9, lift=0.15)),
    (5, dict(crouch=1.2, hip_tip=10, chest_tip=8, head=-10, l_out=80, r_out=75, l_fwd=10, r_fwd=10, b_pitch=-6,
             lift=0.45, jaw=0.4, ear_l_up=0.8, ear_r_up=0.7)),
    (10, merge(TUCK, dict(b_pitch=0))),
])
clip('jakb-board-jump-loop', [
    (0, merge(TUCK, dict(l_out=62, r_out=58))),
    (2, merge(TUCK, dict(l_out=70, r_out=52, crouch=1.5, ear_l_up=0.7, tail_side=0.3, b_pitch=2))),
    (4, merge(TUCK, dict(l_out=55, r_out=66, crouch=1.65, ear_r_up=0.6, tail_side=-0.3, b_pitch=-2))),
], loop=True)
# Landing on the board: squashed, then up.
clip('jakb-board-get-on-land', [
    (0, dict(crouch=2.1, hip_tip=26, chest_tip=20, head=-22, l_out=40, r_out=40, l_fwd=40, r_fwd=40, l_elbow=50,
             r_elbow=50, jaw=0.45, blink=0.7, ear_l_up=-0.9, ear_r_up=-0.8, tail_up=-0.6)),
    (2, dict(crouch=1.3, hip_tip=12, chest_tip=9, head=-12, l_out=60, r_out=58, jaw=0.15, ear_l_up=0.2,
             ear_r_up=0.15)),
])
# Getting on: a hop onto the board (the board comes up under him at the end).
clip('jakb-board-get-on', [
    # frame 0 is his own standing pose (facing forward), so the ride starts from where he stood
    (0, dict(yaw=0, crouch=0.0, hip_tip=0, chest_tip=0, twist=0, head=0, head_turn=0, l_out=0, r_out=0, l_fwd=0,
             r_fwd=0, l_elbow=0, r_elbow=0, b_pitch=0, lift=0.0)),
    (4, dict(yaw=-20, crouch=1.5, hip_tip=16, chest_tip=14, head=-14, twist=10, l_out=40, r_out=40, l_fwd=30, r_fwd=30)),
    (8, dict(yaw=-55, crouch=0.6, hip_tip=4, twist=25, l_out=100, r_out=95, l_fwd=-10, r_fwd=-10, lift=0.3, jaw=0.5,
             brows=0.9, ear_l_up=0.6, ear_r_up=0.6)),
    (13, dict(yaw=YAW, crouch=1.3, hip_tip=10, twist=30, l_out=60, r_out=58)),
])

# His own standing pose, as at get-on's first frame: getting off ends here, so his own animation
# takes over from where the clip leaves him.
STAND = dict(yaw=0, crouch=0.0, hip_tip=0, chest_tip=0, twist=0, head=0, head_turn=0, l_out=0, r_out=0, l_fwd=0,
             r_fwd=0, l_elbow=0, r_elbow=0, b_pitch=0, lift=0.0, ear_l_back=0.0, ear_r_back=0.0, tail_up=0.0)
# Before a jump off from a hop: down to load, arms back.
clip('jakb-board-get-off-pre', [
    (0, dict(crouch=1.4, hip_tip=12)),
    (5, dict(crouch=2.0, hip_tip=24, chest_tip=20, head=-18, l_out=30, r_out=30, l_fwd=-35, r_fwd=-35, l_elbow=30,
             r_elbow=30, snarl=0.4, ear_l_up=-0.4, ear_r_up=-0.4, tail_up=-0.3)),
])
# Off the board: a pop up and forward off it, turning to face ahead, down onto his feet.
clip('jakb-board-get-off', [
    (0, dict(crouch=2.0, hip_tip=24, chest_tip=20, head=-18, l_out=30, r_out=30, l_fwd=-35, r_fwd=-35)),
    (3, dict(yaw=-40, crouch=0.3, hip_tip=0, chest_tip=-6, head=0, twist=15, l_out=95, r_out=90, l_fwd=25,
             r_fwd=20, l_elbow=20, r_elbow=20, b_pitch=-10, jaw=0.5, brows=0.9, ear_l_up=0.7, ear_r_up=0.6,
             tail_up=0.8)),
    (7, dict(yaw=-8, crouch=0.6, hip_tip=6, chest_tip=4, head=-6, twist=4, l_out=45, r_out=42, l_fwd=15, r_fwd=12,
             l_elbow=15, r_elbow=15, jaw=0.2)),
    (10, STAND),
])
# Knocked off: thrown back, arms flung out, then onto his feet facing ahead.
clip('jakb-board-hit-get-off', [
    (0, dict(crouch=0.8, hip_tip=-14, chest_tip=-18, head=18, l_out=110, r_out=105, l_fwd=-30, r_fwd=-25,
             l_elbow=10, r_elbow=10, jaw=1.0, brows=1.2, blink=0.6, ear_l_up=0.9, ear_r_up=0.9, tail_up=0.9)),
    (5, dict(yaw=-45, crouch=0.6, hip_tip=-8, chest_tip=-10, head=10, twist=-10, l_out=125, r_out=80, l_fwd=20,
             r_fwd=-40, jaw=0.8, brows=1.0, ear_l_up=0.5, ear_r_up=0.8, tail_side=0.6)),
    (10, dict(yaw=-15, crouch=1.4, hip_tip=14, chest_tip=10, head=-10, l_out=60, r_out=55, l_fwd=30, r_fwd=30,
              l_elbow=30, r_elbow=30, jaw=0.3, brows=0.4)),
    (15, STAND),
])


# Flips: the whole body turns over the clip (flip, degrees) as Jak's flip clips turn him. Forward:
# arms swing up and over, a tight tuck holding the knees, then open to land. Backward: head and
# arms thrown back, the tuck, then open.
def flip_keys(n, sign, keys):
    return [(f, merge(d, dict(flip=sign * 360.0 * f / (n - 1)))) for f, d in keys]


TIGHT = dict(crouch=2.2, hip_tip=28, chest_tip=24, head=-28, l_out=25, r_out=25, l_fwd=70, r_fwd=70, l_elbow=75,
             r_elbow=75, lift=0.75, jaw=0.2, brows=0.6, ear_l_back=0.9, ear_r_back=0.9, tail_up=-0.3, twist=10)
clip('jakb-board-flip-forward', flip_keys(11, 1, [
    (0, dict(crouch=1.3, hip_tip=14, chest_tip=10, head=-12, l_out=60, r_out=55, l_fwd=60, r_fwd=55, l_elbow=20,
             r_elbow=20, lift=0.3, jaw=0.4, ear_l_up=0.6, ear_r_up=0.5)),
    (3, TIGHT),
    (7, merge(TIGHT, dict(crouch=2.3, head=-32))),
    (10, dict(crouch=1.3, hip_tip=8, chest_tip=4, head=-8, l_out=95, r_out=90, l_fwd=10, r_fwd=10, l_elbow=20,
              r_elbow=20, lift=0.3, jaw=0.3, brows=0.8, ear_l_up=0.5, ear_r_up=0.4)),
]))
clip('jakb-board-flip-forward-loop', flip_keys(7, 1, [(0, TIGHT), (3, merge(TIGHT, dict(crouch=2.35))), (6, TIGHT)]))
clip('jakb-board-flip-backward', flip_keys(11, -1, [
    (0, dict(crouch=0.9, hip_tip=-6, chest_tip=-16, head=18, l_out=120, r_out=115, l_fwd=-40, r_fwd=-35,
             l_elbow=15, r_elbow=15, lift=0.3, jaw=0.6, brows=1.0, ear_l_up=0.8, ear_r_up=0.7, tail_up=0.8)),
    (3, merge(TIGHT, dict(head=-10, hip_tip=18))),
    (7, merge(TIGHT, dict(head=-14, hip_tip=20))),
    (10, dict(crouch=1.3, hip_tip=6, chest_tip=2, head=-6, l_out=95, r_out=90, l_fwd=0, r_fwd=0, l_elbow=20,
              r_elbow=20, lift=0.3, jaw=0.3, brows=0.8, ear_l_up=0.4, ear_r_up=0.4)),
]))
clip('jakb-board-flip-backward-loop', flip_keys(7, -1, [(0, merge(TIGHT, dict(head=-12))),
                                                        (3, merge(TIGHT, dict(head=-14, crouch=2.35))),
                                                        (6, merge(TIGHT, dict(head=-12)))]))
# The spin (the board's twist turns him): wound up, arms flung out and round, back into a crouch.
clip('jakb-board-spin', [
    (0, dict(crouch=1.5, twist=-25, head_turn=-15, l_out=40, r_out=80, l_fwd=40, r_fwd=-20, lift=0.35)),
    (3, dict(crouch=1.2, twist=45, head_turn=30, l_out=110, r_out=100, l_fwd=-10, r_fwd=10, l_elbow=10,
             r_elbow=10, lift=0.5, jaw=0.5, brows=0.9, ear_l_back=0.9, ear_r_back=0.9, tail_side=-0.8)),
    (7, dict(crouch=1.5, twist=35, head_turn=20, l_out=90, r_out=95, lift=0.5, tail_side=0.6)),
    (11, merge(TUCK, dict(twist=20))),
])


# The tricks and hits, until their own clips come: one held look (poses.py), and the flips' whole
# turn over the clip.
HELD = {
    'jakb-board-jump-kick': 'kick', 'jakb-board-hit': 'hit', 'jakb-board-hit-forward': 'hit',
    'jakb-board-hit-elec': 'hit', 'jakb-board-hit-get-off': 'hit', 'jakb-board-nosegrab': 'nosegrab',
    'jakb-board-nosegrab-loop': 'nosegrab', 'jakb-board-nosegrab-end': 'nosegrab',
    'jakb-board-noseflip': 'nosegrab', 'jakb-board-method': 'method', 'jakb-board-method-loop': 'method',
    'jakb-board-method-end': 'method', 'jakb-board-kickflip-a': 'kickflip', 'jakb-board-kickflip-b': 'kickflip',
    'jakb-board-kickflip-c': 'kickflip', 'jakb-board-spin': 'tuck', 'jakb-board-jump-kickoff': 'jump',
    'jakb-board-flip-forward': 'tuck', 'jakb-board-flip-forward-loop': 'tuck',
    'jakb-board-flip-backward': 'tuck', 'jakb-board-flip-backward-loop': 'tuck',
}
FLIPS = {'jakb-board-flip-forward': 360, 'jakb-board-flip-forward-loop': 360,
         'jakb-board-flip-backward': -360, 'jakb-board-flip-backward-loop': -360}


FEET = (94, 95, 100, 101)


def feet(rig, d, t):
    """The feet's middle (x, y) and their lowest point (z) in a pose."""
    rot, tr = rig.local_pose(0, 0)
    rot = [qmul(rot[k], d[k]) if k in d else rot[k] for k in range(len(rot))]
    tr = [tr[k] + np.array(t) if k == 1 else tr[k] for k in range(len(tr))]
    G = rig.globals(rot, tr)
    P = np.array([G[j][:3, 3] for j in FEET])
    return P[:, 0].mean(), P[:, 1].mean(), min(G[95][2, 3], G[101][2, 3])


def plant(rig, d, t, lift=None):
    """The hips moved so his feet stay where they stand on the board: bent knees take the hips
    down and back over the feet (as a rider's do) instead of swinging the feet out from under him.
    With `lift` the lowest foot is that far up; without it the hips keep their height."""
    sx, sy, sz = feet(rig, {}, (0.0, 0.0, 0.0))
    x, y, z = feet(rig, d, t)
    nz = t[2] if lift is None else t[2] + (sz + lift - z)
    return (t[0] + sx - x, t[1] + sy - y, nz)


def frame_pose(rig, clipname, f):
    """(local joint deltas {joint: q}, hips' move, board (pitch, roll, yaw, lift), flip degrees)."""
    if clipname in CLIPS:
        p = CLIPS[clipname].params(f)
        w, t = pose(p)
        d = rig.deltas_from_world(0, 0, w)
        t = plant(rig, d, t, p['lift'])
        face = {k: p[k] for k in FACE}
        board = (p['b_pitch'], p['b_roll'], p['b_yaw'], p['lift'])
        flip = p['flip']
    else:
        look = HELD[clipname]
        w, t = POSES[look]
        t = t[1]
        d = rig.deltas_from_world(0, 0, w)
        t = plant(rig, d, t)
        face = dict((k, DEFAULT[k]) for k in FACE)
        face.update(dict(jaw=0.75, brows=1.0) if look in ('nosegrab', 'method', 'kickflip') else {})
        if look == 'hit': face.update(jaw=1.0, brows=1.2)
        if look == 'kick': face.update(snarl=0.8, jaw=0.5, brows=-0.6)
        lift = 0.55 if look in ('tuck', 'nosegrab', 'method', 'kickflip') else 0.0
        board = (0.0, 0.0, 0.0, lift)
        n = FRAMES[clipname]
        flip = FLIPS.get(clipname, 0) * (f / (n - 1) if n > 1 else 0.0)
    # face, ears and tail: each channel's turns in the joint's own frame, after the pose's
    rot, tr = rig.local_pose(0, 0)
    G = rig.globals(rot, tr)
    for ch, v in face.items():
        if abs(v) < 1e-4: continue
        for j, axis, deg in CHANNELS[ch]:
            a = np.asarray(axis, float); a = a / np.linalg.norm(a)
            loc = G[j][:3, :3].T @ a
            q = qaxis(loc, deg * v)
            d[j] = qmul(d[j], q) if j in d else q
    return d, t, board, flip


def names():
    return list(CLIPS) + [n for n in HELD if n not in CLIPS]


def export(rig, path):
    """The clips as the mod's table (src/rac1/ride_clips.inc): per clip, the joints it turns, and per
    frame each one's local offset (q * d) over his standing pose, the hips' move, the board's
    (pitch, roll, yaw in degrees, lift) and the flip."""
    out = ["// SPDX-License-Identifier: MIT", "// Copyright (c) 2026 Mateusz Kłysz",
           "// Ratchet's riding clips, our own, named and timed as Jak II's board animations.",
           "// Generated by tools/poses/clips.py from its clip table; do not edit by hand.", "#pragma once", "",
           '#include "rac1/ride_clip.hpp"', "", "namespace rac1::clips {", ""]
    entries = []
    for k, n in enumerate(names()):
        frames = [frame_pose(rig, n, f) for f in range(FRAMES[n])]
        joints = sorted(set(j for d, _, _, _ in frames for j in d))
        q, hips, board = [], [], []
        for d, t, b, flip in frames:
            for j in joints:
                v = d.get(j, np.array([0, 0, 0, 1.0]))
                if v[3] < 0: v = -v
                q.extend(v)
            hips.extend(t)
            board.extend(list(b) + [flip])
        fl = lambda xs: ", ".join(f"{x:.5f}f" for x in xs)
        out.append(f"// {n}")
        out.append(f"constexpr uint8_t j{k}[] = {{{', '.join(str(j) for j in joints)}}};")
        out.append(f"constexpr float q{k}[] = {{{fl(q)}}};")
        out.append(f"constexpr float h{k}[] = {{{fl(hips)}}};")
        out.append(f"constexpr float b{k}[] = {{{fl(board)}}};")
        entries.append(f'    {{"{n}-ja", {FRAMES[n]}, {len(joints)}, j{k}, q{k}, h{k}, b{k}}},')
        out.append("")
    out.append("constexpr Clip kClips[] = {")
    out.extend(entries)
    out.append("};")
    out.append("")
    out.append("} // namespace rac1::clips")
    open(path, 'w').write("\n".join(out) + "\n")


if __name__ == "__main__":
    import os
    read_frames(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'board', 'anim_table.cpp'))
    r = Rig(sys.argv[1])
    if sys.argv[2].endswith('.inc'):
        export(r, sys.argv[2])
        sys.exit(0)
    want = sys.argv[3:] or list(CLIPS)
    rows = []
    for n in want:
        k = FRAMES[n]
        step = max(1, k // 6)
        tiles = []
        for f in range(0, k, step):
            d, t, board, flip = frame_pose(r, n, f)
            views = tuple(os.environ.get('VIEWS', 'side').split(','))
            tiles.append(r.render(0, 0, d, trans={1: t}, size=int(os.environ.get('SIZE', 200)), views=views, title=f"{n[11:]} {f}"))
        tw = tiles[0].width
        row = Image.new('RGB', (tw * len(tiles), tiles[0].height), 'white')
        for i, im in enumerate(tiles): row.paste(im, (tw * i, 0))
        rows.append(row)
    W = max(im.width for im in rows)
    H = rows[0].height
    out = Image.new('RGB', (W, H * len(rows)), 'white')
    for i, im in enumerate(rows): out.paste(im, (0, H * i))
    out.save(sys.argv[2])

