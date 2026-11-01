#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Mateusz Kłysz
"""Ratchet's skeleton for authoring poses: his standing pose (seq 0) as the base, joint offsets on
top (local q * d, as the port's mode-0 joint modifier), and a flat-shaded preview to PNG. It reads
his model from your own extracted level data (OpenRAC's level_01/mobys/moby_0.glb); nothing of it
is written out, only the offsets."""
import json, struct, math
import numpy as np
from PIL import Image, ImageDraw



def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw*bx + bw*ax + (ay*bz - az*by), aw*by + bw*ay + (az*bx - ax*bz),
                     aw*bz + bw*az + (ax*by - ay*bx), aw*bw - (ax*bx + ay*by + az*bz)])


def qinv(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qaxis(axis, deg):
    axis = np.asarray(axis, float); axis = axis / np.linalg.norm(axis)
    s = math.sin(math.radians(deg) / 2)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(math.radians(deg) / 2)])


def qmat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def mat_q(m):
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([(m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s, (m[1,0]-m[0,1])/s, 0.25*s])
    i = int(np.argmax(np.diag(m)))
    j, k = (i+1) % 3, (i+2) % 3
    s = math.sqrt(1 + m[i,i] - m[j,j] - m[k,k]) * 2
    q = [0, 0, 0, 0]
    q[i] = 0.25*s; q[j] = (m[j,i]+m[i,j])/s; q[k] = (m[k,i]+m[i,k])/s; q[3] = (m[k,j]-m[j,k])/s
    return np.array(q)


class Rig:
    def __init__(self, path):
        b = open(path, 'rb').read()
        n = struct.unpack_from('<I', b, 12)[0]
        self.j = json.loads(b[20:20+n])
        off = 20 + n
        bl = struct.unpack_from('<I', b, off)[0]
        self.bin = b[off+8:off+8+bl]
        j = self.j
        self.joints = j['skins'][0]['joints']
        self.idx = {nd: k for k, nd in enumerate(self.joints)}
        self.parent = {}
        for i, nd in enumerate(j['nodes']):
            for c in nd.get('children', []):
                self.parent[c] = i
        self.rest_t = [np.array(j['nodes'][nd].get('translation', [0, 0, 0]), float) for nd in self.joints]
        self.ibm = self.acc(j['skins'][0]['inverseBindMatrices']).reshape(-1, 4, 4).transpose(0, 2, 1)
        prim = j['meshes'][0]['primitives']
        self.prims = []
        for p in prim:
            a = p['attributes']
            self.prims.append((self.acc(a['POSITION']).reshape(-1, 3), self.acc(a['JOINTS_0']).reshape(-1, 4).astype(int),
                               self.acc(a['WEIGHTS_0']).reshape(-1, 4), self.acc(p['indices']).reshape(-1, 3).astype(int)))
        self.anims = {a['name']: a for a in j['animations']}

    def acc(self, i):
        a = self.j['accessors'][i]
        bv = self.j['bufferViews'][a['bufferView']]
        comp = {5126: ('f', 4), 5123: ('H', 2), 5121: ('B', 1), 5125: ('I', 4),
                5122: ('h', 2), 5120: ('b', 1)}[a['componentType']]
        cnt = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}[a['type']]
        start = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        stride = bv.get('byteStride', 0) or comp[1]*cnt
        out = []
        for k in range(a['count']):
            out.extend(struct.unpack_from('<%d%s' % (cnt, comp[0]), self.bin, start + k*stride))
        arr = np.array(out, float if comp[0] == 'f' else int)
        if a.get('normalized') and comp[0] != 'f':
            arr = np.maximum(arr / {'B': 255.0, 'H': 65535.0, 'b': 127.0, 'h': 32767.0, 'I': 4294967295.0}[comp[0]], -1.0)
        return arr

    def seq_name(self, seq):
        for name in self.anims:
            if int(name.split('_')[2]) == seq:
                return name

    def seq_frames(self, seq):
        a = self.anims[self.seq_name(seq)]
        return len(self.acc(a['samplers'][0]['input']))

    def local_pose(self, seq, frame):
        """Each joint's local (rotation, translation) at key `frame` of `seq`."""
        a = self.anims[self.seq_name(seq)]
        rot = [np.array([0, 0, 0, 1.0]) for _ in self.joints]
        tr = [t.copy() for t in self.rest_t]
        for ch in a['channels']:
            nd = ch['target']['node']
            if nd not in self.idx:
                continue
            s = a['samplers'][ch['sampler']]
            out = self.acc(s['output'])
            if ch['target']['path'] == 'rotation':
                v = out.reshape(-1, 4); rot[self.idx[nd]] = v[min(frame, len(v)-1)]
            elif ch['target']['path'] == 'translation':
                v = out.reshape(-1, 3); tr[self.idx[nd]] = v[min(frame, len(v)-1)]
        return rot, tr

    def globals(self, rot, tr):
        G = [None]*len(self.joints)
        for k, nd in enumerate(self.joints):
            m = np.eye(4); m[:3, :3] = qmat(rot[k]); m[:3, 3] = tr[k]
            p = self.parent.get(nd)
            G[k] = G[self.idx[p]] @ m if p in self.idx else m
        return G

    def deltas_from_world(self, seq, frame, world):
        """Local deltas (d per joint, q*d) that turn joints by world-space (model-space) rotations:
        world = {joint: (axis, degrees)}, applied root first, each bone turned about its own pivot
        with its children carried along."""
        rot, tr = self.local_pose(seq, frame)
        G = self.globals(rot, tr)
        newrot = list(rot)
        Gn = [None]*len(self.joints)
        for k, nd in enumerate(self.joints):
            p = self.parent.get(nd)
            Pn = Gn[self.idx[p]] if p in self.idx else np.eye(4)
            P0 = G[self.idx[p]] if p in self.idx else np.eye(4)
            # The joint's world rotation: its parent's new one times its own local, then the extra
            # world turn of this joint.
            R = Pn[:3, :3] @ qmat(rot[k])
            if k in world:
                turns = world[k] if isinstance(world[k], list) else [world[k]]
                for axis, deg in turns:
                    R = qmat(qaxis(axis, deg)) @ R
            loc = np.linalg.inv(Pn[:3, :3]) @ R
            newrot[k] = mat_q(loc)
            m = np.eye(4); m[:3, :3] = loc; m[:3, 3] = tr[k]
            Gn[k] = Pn @ m
        d = {}
        for k in range(len(self.joints)):
            dq = qmul(qinv(rot[k]), newrot[k])
            if abs(dq[3]) < 0.99999 and np.linalg.norm(dq[:3]) > 1e-4:
                d[k] = dq / np.linalg.norm(dq)
        return d

    def render(self, seq, frame, deltas=None, trans=None, size=360, views=("side", "front"), title=""):
        rot, tr = self.local_pose(seq, frame)
        if deltas:
            rot = [qmul(rot[k], deltas[k]) if k in deltas else rot[k] for k in range(len(rot))]
        if trans:
            tr = [tr[k] + np.array(trans[k]) if k in trans else tr[k] for k in range(len(tr))]
        G = self.globals(rot, tr)
        skin = np.array([G[k] @ self.ibm[k] for k in range(len(G))])
        tris = []
        for pos, jo, we, ind in self.prims:
            P = np.c_[pos, np.ones(len(pos))]
            V = np.zeros((len(pos), 3))
            for c in range(4):
                M = skin[jo[:, c]]
                V += we[:, c:c+1] * np.einsum('nij,nj->ni', M, P)[:, :3]
            tris.append(V[ind])
        T = np.concatenate(tris)
        imgs = []
        for view in views:
            # forward of the model is +x (game yaw 0); side looks from -y, front from +x.
            if view == "side":
                X, Y, D = T[..., 0], T[..., 2], -T[..., 1]
            elif view == "front":
                X, Y, D = -T[..., 1], T[..., 2], T[..., 0]
            elif view == "back":
                X, Y, D = T[..., 1], T[..., 2], -T[..., 0]
            else:  # three-quarter
                c, s = math.cos(math.radians(35)), math.sin(math.radians(35))
                X, Y, D = T[..., 0]*c - T[..., 1]*s, T[..., 2], -(T[..., 0]*s + T[..., 1]*c)
            im = Image.new('RGB', (size, size), (235, 235, 240))
            dr = ImageDraw.Draw(im)
            sc = size / 11.0
            ox, oy = size/2, size - 10
            n = np.cross(T[:, 1]-T[:, 0], T[:, 2]-T[:, 0])
            order = np.argsort(D.mean(axis=1))
            for t in order:
                nn = np.linalg.norm(n[t]) or 1
                if view == "side":
                    l = abs(-n[t][1])/nn
                elif view in ("front", "back"):
                    l = abs(n[t][0])/nn
                else:
                    l = abs(n[t][0]*s + n[t][1]*c)/nn
                g = int(80 + 150*l)
                pts = [(ox + X[t][i]*sc, oy - Y[t][i]*sc) for i in range(3)]
                dr.polygon(pts, fill=(g, int(g*0.9), int(g*0.7)))
            dr.line([(0, oy), (size, oy)], fill=(120, 120, 120))
            dr.text((4, 4), f"{title} {view}", fill=(0, 0, 0))
            imgs.append(im)
        out = Image.new('RGB', (size*len(imgs), size), 'white')
        for i, im in enumerate(imgs):
            out.paste(im, (i*size, 0))
        return out
