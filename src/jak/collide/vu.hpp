// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// The register machine Jak II's assembly collision routines run on in OpenGOAL: the EE's
// general-purpose and float registers and the VU0 macro-mode vector unit, as OpenGOAL's mips2c
// ExecutionContext defines them (game/mips2c/mips2c_private.h, jak-project, ISC License, Copyright
// (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md). Every instruction here does what OpenGOAL's does,
// lane by lane, so the routines ported from OpenGOAL's mips2c output (collide/mips2c.cpp) compute
// the same floats and take the same branches as OpenGOAL on a PC.
//
// Memory is the collision arena (collide/mem.hpp): 32-bit GOAL addresses are offsets into it, so
// the structures the assembly reads keep their GOAL layout. The GOAL `rlet` code (inline vector
// assembly) uses the same machine for its vector registers.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace jak::mips {

enum Gpr {
    r0 = 0, at = 1, v0 = 2, v1 = 3, a0 = 4, a1 = 5, a2 = 6, a3 = 7, t0 = 8, t1 = 9, t2 = 10,
    t3 = 11, t4 = 12, t5 = 13, t6 = 14, t7 = 15, s0 = 16, s1 = 17, s2 = 18, s3 = 19, s4 = 20,
    s5 = 21, s6 = 22, s7 = 23, t8 = 24, t9 = 25, k0 = 26, k1 = 27, gp = 28, sp = 29, fp = 30,
    ra = 31,
};

enum VfName {
    vf0 = 0, vf1, vf2, vf3, vf4, vf5, vf6, vf7, vf8, vf9, vf10, vf11, vf12, vf13, vf14, vf15,
    vf16, vf17, vf18, vf19, vf20, vf21, vf22, vf23, vf24, vf25, vf26, vf27, vf28, vf29, vf30,
    vf31,
};

enum FprName {
    f0 = 0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14, f15, f16, f17, f18,
    f19, f20, f21, f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
};

// Lane masks, x the least significant bit (OpenGOAL's encoding, not the PS2's).
enum class DEST {
    NONE = 0, x = 1, y = 2, xy = 3, z = 4, xz = 5, yz = 6, xyz = 7, w = 8, xw = 9, yw = 10,
    xyw = 11, zw = 12, xzw = 13, yzw = 14, xyzw = 15
};
enum class BC { x = 0, y = 1, z = 2, w = 3 };

union Reg128 {
    uint64_t du64[2];
    int64_t ds64[2];
    uint32_t du32[4];
    int32_t ds32[4];
    uint16_t du16[8];
    int16_t ds16[8];
    uint8_t du8[16];
    float f[4];
};

// The base of the collision arena: GOAL address a is g_mem + a (collide/mem.cpp).
extern uint8_t* g_mem;

// float -> int32 as x86's cvttss2si: truncation, and 0x80000000 for NaN and out-of-range values
// (OpenGOAL's vftoi0 converts with a C++ cast, which compiles to that instruction).
inline int32_t f2i_x86(float f) {
    if (!(f > -2147483904.0f && f < 2147483648.0f)) {
        return INT32_MIN;
    }
    return static_cast<int32_t>(f);
}

struct Ctx {
    Reg128 gprs[32];
    float fprs[32];
    Reg128 vfs[32];
    Reg128 acc;
    float Q = 0.0f;
    Reg128 hi, lo;

    // OpenGOAL hands each assembly routine a fresh context on the C++ stack, never cleared; the
    // routines ported here read no register they have not written, so the port clears it.
    Ctx() {
        std::memset(gprs, 0, sizeof(gprs));
        std::memset(fprs, 0, sizeof(fprs));
        std::memset(vfs, 0, sizeof(vfs));
        std::memset(&acc, 0, sizeof(acc));
        std::memset(&hi, 0, sizeof(hi));
        std::memset(&lo, 0, sizeof(lo));
    }

    Reg128 vf_src(int idx) const {
        if (idx == 0) {
            Reg128 r;
            r.f[0] = 0.0f;
            r.f[1] = 0.0f;
            r.f[2] = 0.0f;
            r.f[3] = 1.0f;
            return r;
        }
        return vfs[idx];
    }
    Reg128 gpr_src(int idx) const {
        if (idx == 0) {
            Reg128 r;
            r.du64[0] = 0;
            r.du64[1] = 0;
            return r;
        }
        return gprs[idx];
    }
    uint64_t sgpr64(int idx) const { return gpr_src(idx).du64[0]; }
    uint32_t gpr_addr(int idx) const { return gpr_src(idx).du32[0]; }
    uint8_t* at_(uint32_t addr) const { return g_mem + addr; }

    // ---- loads and stores ----
    void lb(int dst, int off, int src) {
        int8_t v;
        std::memcpy(&v, at_(gpr_addr(src) + off), 1);
        gprs[dst].du64[0] = static_cast<uint64_t>(static_cast<int64_t>(v));
    }
    void lhu(int dst, int off, int src) {
        uint16_t v;
        std::memcpy(&v, at_(gpr_addr(src) + off), 2);
        gprs[dst].du64[0] = v;
    }
    void lw(int dst, int off, int src) {
        int32_t v;
        std::memcpy(&v, at_(gpr_addr(src) + off), 4);
        gprs[dst].ds64[0] = v;
    }
    void lwu(int dst, int off, int src) {
        uint32_t v;
        std::memcpy(&v, at_(gpr_addr(src) + off), 4);
        gprs[dst].du64[0] = v;
    }
    void ld(int dst, int off, int src) { std::memcpy(&gprs[dst].du64[0], at_(gpr_addr(src) + off), 8); }
    void lq(int dst, int off, int src) {
        std::memcpy(&gprs[dst].du64[0], at_((gpr_addr(src) + off) & ~15u), 16);
    }
    void lqc2(int vf, int off, int src) { std::memcpy(&vfs[vf], at_(gpr_addr(src) + off), 16); }
    void lwc1(int dst, int off, int src) { std::memcpy(&fprs[dst], at_(gpr_addr(src) + off), 4); }
    void sb(int src, int off, int addr) {
        Reg128 s = gpr_src(src);
        std::memcpy(at_(gpr_addr(addr) + off), &s.du32[0], 1);
    }
    void sw(int src, int off, int addr) {
        Reg128 s = gpr_src(src);
        std::memcpy(at_(gpr_addr(addr) + off), &s.du32[0], 4);
    }
    void sd(int src, int off, int addr) {
        Reg128 s = gpr_src(src);
        std::memcpy(at_(gpr_addr(addr) + off), &s.du32[0], 8);
    }
    void sq(int src, int off, int addr) {
        Reg128 s = gpr_src(src);
        std::memcpy(at_((gpr_addr(addr) + off) & ~15u), &s.du32[0], 16);
    }
    void sqc2(int src, int off, int addr) {
        Reg128 s = vf_src(src);
        std::memcpy(at_(gpr_addr(addr) + off), &s.du32[0], 16);
    }
    void swc1(int src, int off, int addr) { std::memcpy(at_(gpr_addr(addr) + off), &fprs[src], 4); }

    // ---- integer ----
    void mov64(int dst, int src) { gprs[dst].ds64[0] = static_cast<int64_t>(gpr_src(src).du64[0]); }
    void daddu(int dst, int a, int b) { gprs[dst].du64[0] = sgpr64(a) + sgpr64(b); }
    void daddiu(int dst, int a, int64_t imm) { gprs[dst].du64[0] = sgpr64(a) + static_cast<uint64_t>(imm); }
    void addiu(int dst, int a, int64_t imm) {
        const int32_t t = static_cast<int32_t>(sgpr64(a) + static_cast<uint64_t>(imm));
        gprs[dst].ds64[0] = t;
    }
    void dsubu(int dst, int a, int b) { gprs[dst].du64[0] = sgpr64(a) - sgpr64(b); }
    void subu(int dst, int a, int b) {
        gprs[dst].ds64[0] = static_cast<int32_t>(static_cast<uint32_t>(gpr_src(a).ds32[0]) -
                                                 static_cast<uint32_t>(gpr_src(b).ds32[0]));
    }
    void lui(int dst, uint32_t imm) { gprs[dst].ds64[0] = static_cast<int32_t>(imm << 16); }
    void ori(int dst, int src, uint64_t imm) { gprs[dst].du64[0] = gpr_src(src).du64[0] | imm; }
    void andi(int dst, int src, uint64_t imm) { gprs[dst].du64[0] = gpr_src(src).du64[0] & imm; }
    void and_(int dst, int a, int b) { gprs[dst].du64[0] = gpr_src(a).du64[0] & gpr_src(b).du64[0]; }
    void or_(int dst, int a, int b) { gprs[dst].du64[0] = sgpr64(a) | sgpr64(b); }
    void sll(int dst, int src, int sa) {
        const uint32_t v = gpr_src(src).du32[0] << sa;
        gprs[dst].ds64[0] = static_cast<int32_t>(v);
    }
    void sllv(int dst, int src, int sa) {
        gprs[dst].ds64[0] = static_cast<int32_t>(static_cast<uint32_t>(gpr_src(src).ds32[0])
                                                 << (gpr_src(sa).du32[0] & 31u));
    }
    void dsll(int dst, int src, int sa) { gprs[dst].du64[0] = gpr_src(src).du64[0] << sa; }
    void dsll32(int dst, int src, int sa) { gprs[dst].du64[0] = gpr_src(src).du64[0] << (32 + sa); }
    void dsra32(int dst, int src, int sa) { gprs[dst].ds64[0] = gpr_src(src).ds64[0] >> (32 + sa); }
    void dsrl32(int dst, int src, int sa) { gprs[dst].du64[0] = gpr_src(src).du64[0] >> (32 + sa); }
    void slt(int dst, int a, int b) { gprs[dst].du64[0] = gpr_src(a).ds64[0] < gpr_src(b).ds64[0] ? 1 : 0; }
    void slti(int dst, int a, int64_t imm) { gprs[dst].du64[0] = gpr_src(a).ds64[0] < imm ? 1 : 0; }
    void movz(int dst, int a, int b) {
        if (sgpr64(b) == 0) gprs[dst].du64[0] = sgpr64(a);
    }
    void movn(int dst, int a, int b) {
        if (sgpr64(b) != 0) gprs[dst].du64[0] = sgpr64(a);
    }
    void mult3(int dst, int a, int b) {
        const int64_t r = static_cast<int64_t>(gpr_src(a).ds32[0]) * gpr_src(b).ds32[0];
        lo.ds64[0] = static_cast<int32_t>(r & 0xffffffff);
        hi.ds64[0] = static_cast<int32_t>(r >> 32);
        gprs[dst].du64[0] = lo.du64[0];
    }

    // ---- 128-bit integer (MMI) ----
    void pcgtw(int rd, int rs, int rt) {
        const Reg128 s = gpr_src(rs), t = gpr_src(rt);
        for (int i = 0; i < 4; i++) gprs[rd].du32[i] = s.ds32[i] > t.ds32[i] ? 0xffffffffu : 0u;
    }
    void ppach(int rd, int rs, int rt) {
        const Reg128 s = gpr_src(rs), t = gpr_src(rt);
        gprs[rd].du16[0] = t.du16[0];
        gprs[rd].du16[1] = t.du16[2];
        gprs[rd].du16[2] = t.du16[4];
        gprs[rd].du16[3] = t.du16[6];
        gprs[rd].du16[4] = s.du16[0];
        gprs[rd].du16[5] = s.du16[2];
        gprs[rd].du16[6] = s.du16[4];
        gprs[rd].du16[7] = s.du16[6];
    }
    void por(int dst, int a, int b) {
        const Reg128 s0 = gpr_src(a), s1 = gpr_src(b);
        gprs[dst].du64[0] = s0.du64[0] | s1.du64[0];
        gprs[dst].du64[1] = s0.du64[1] | s1.du64[1];
    }
    void pcpyud(int dst, int a, int b) {
        const Reg128 s0 = gpr_src(a), s1 = gpr_src(b);
        gprs[dst].du64[0] = s0.du64[1];
        gprs[dst].du64[1] = s1.du64[1];
    }

    // ---- COP1 ----
    void mtc1(int dst, int src) {
        const uint32_t v = gpr_src(src).du32[0];
        std::memcpy(&fprs[dst], &v, 4);
    }
    void mfc1(int dst, int src) {
        int32_t v;
        std::memcpy(&v, &fprs[src], 4);
        gprs[dst].ds64[0] = v;
    }
    void sqrts(int dst, int src) { fprs[dst] = std::sqrt(std::fabs(fprs[src])); }
    void abss(int dst, int src) { fprs[dst] = std::fabs(fprs[src]); }

    // ---- COP2 moves ----
    void mov128_vf_gpr(int dst, int src) { std::memcpy(vfs[dst].f, gpr_src(src).f, 16); }
    void mov128_gpr_vf(int dst, int src) { std::memcpy(gprs[dst].f, vf_src(src).f, 16); }

    // ---- VU0 macro mode ----
#define JAK_VU_LANES for (int i = 0; i < 4; i++) if (static_cast<unsigned>(mask) & (1u << i))
    void vadd(DEST mask, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = s0.f[i] + s1.f[i]; }
    void vsub(DEST mask, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = s0.f[i] - s1.f[i]; }
    void vmul(DEST mask, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = s0.f[i] * s1.f[i]; }
    void vmini(DEST mask, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = std::min(s0.f[i], s1.f[i]); }
    void vmax(DEST mask, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = std::max(s0.f[i], s1.f[i]); }
    void vadd_bc(DEST mask, BC bc, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = s0.f[i] + s1.f[static_cast<int>(bc)]; }
    void vsub_bc(DEST mask, BC bc, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = s0.f[i] - s1.f[static_cast<int>(bc)]; }
    void vmul_bc(DEST mask, BC bc, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = s0.f[i] * s1.f[static_cast<int>(bc)]; }
    void vmini_bc(DEST mask, BC bc, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = std::min(s0.f[i], s1.f[static_cast<int>(bc)]); }
    void vmax_bc(DEST mask, BC bc, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = std::max(s0.f[i], s1.f[static_cast<int>(bc)]); }
    void vmula(DEST mask, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES acc.f[i] = s0.f[i] * s1.f[i]; }
    void vmula_bc(DEST mask, BC bc, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES acc.f[i] = s0.f[i] * s1.f[static_cast<int>(bc)]; }
    void vadda_bc(DEST mask, BC bc, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES acc.f[i] = s0.f[i] + s1.f[static_cast<int>(bc)]; }
    void vmadda(DEST mask, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES acc.f[i] += s0.f[i] * s1.f[i]; }
    void vmadda_bc(DEST mask, BC bc, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES acc.f[i] += s0.f[i] * s1.f[static_cast<int>(bc)]; }
    void vmsuba(DEST mask, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES acc.f[i] -= s0.f[i] * s1.f[i]; }
    void vmadd(DEST mask, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = acc.f[i] + s0.f[i] * s1.f[i]; }
    void vmadd_bc(DEST mask, BC bc, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = acc.f[i] + s0.f[i] * s1.f[static_cast<int>(bc)]; }
    void vmsub_bc(DEST mask, BC bc, int d, int a, int b) { const Reg128 s0 = vf_src(a), s1 = vf_src(b); JAK_VU_LANES vfs[d].f[i] = acc.f[i] - s0.f[i] * s1.f[static_cast<int>(bc)]; }
    void vmulq(DEST mask, int d, int a) { const Reg128 s0 = vf_src(a); JAK_VU_LANES vfs[d].f[i] = s0.f[i] * Q; }
    void vmove(DEST mask, int d, int a) { const Reg128 s0 = vf_src(a); JAK_VU_LANES vfs[d].f[i] = s0.f[i]; }
    void vftoi0(DEST mask, int d, int a) { const Reg128 s0 = vf_src(a); JAK_VU_LANES vfs[d].ds32[i] = f2i_x86(s0.f[i]); }
#undef JAK_VU_LANES
    void vdiv(int a, BC ba, int b, BC bb) { Q = vf_src(a).f[static_cast<int>(ba)] / vf_src(b).f[static_cast<int>(bb)]; }
    void vrsqrt(int a, BC ba, int b, BC bb) {
        const float s = std::sqrt(std::fabs(vf_src(b).f[static_cast<int>(bb)]));
        Q = s == 0.0f ? 0.0f : vf_src(a).f[static_cast<int>(ba)] / s;
    }
    void vsqrt(int a, BC ba) { Q = std::sqrt(std::fabs(vf_src(a).f[static_cast<int>(ba)])); }
    void vwaitq() {}
    // vopmula / vopmsub: the two halves of the outer product (cross product a x b as
    // vopmula(a, b) then vopmsub(d, b, a)).
    void vopmula(int a, int b) {
        const Reg128 s0 = vf_src(a), s1 = vf_src(b);
        acc.f[0] = s0.f[1] * s1.f[2];
        acc.f[1] = s0.f[2] * s1.f[0];
        acc.f[2] = s0.f[0] * s1.f[1];
    }
    void vopmsub(int d, int a, int b) {
        const Reg128 s0 = vf_src(a), s1 = vf_src(b);
        vfs[d].f[0] = acc.f[0] - s0.f[1] * s1.f[2];
        vfs[d].f[1] = acc.f[1] - s0.f[2] * s1.f[0];
        vfs[d].f[2] = acc.f[2] - s0.f[0] * s1.f[1];
    }

    // ---- helpers for the GOAL rlet code (not instructions) ----
    void set_vf(int d, float x, float y, float z, float w) {
        vfs[d].f[0] = x;
        vfs[d].f[1] = y;
        vfs[d].f[2] = z;
        vfs[d].f[3] = w;
    }
    // (.mov vfN float): the float in the x lane (OpenGOAL's movd zeroes the rest).
    void mov_float(int d, float x) { set_vf(d, x, 0.0f, 0.0f, 0.0f); }
    float lane(int vf, int i) const { return vf_src(vf).f[i]; }
};

} // namespace jak::mips
