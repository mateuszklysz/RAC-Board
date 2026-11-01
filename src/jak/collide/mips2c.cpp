// SPDX-License-Identifier: MIT AND ISC
// Copyright (c) 2026 Mateusz Kłysz
// Jak II's collision assembly as OpenGOAL's mips2c translated it (jak-project
// game/mips2c/jak2_functions/collide_func.cpp, collide_cache.cpp, collide_edge_grab.cpp; ISC License,
// Copyright (c) 2020-2026 OpenGOAL Team; THIRD_PARTY.md), instruction for instruction. Calls to GOAL
// functions go through the goal_* adapters below, which read the arguments from the registers as
// the PC port's call convention passes them (floats as their bits in GPRs) and call the C++ ports.
// The "ERROR:" prints are the `format` calls of the originals.
#include "jak/collide/mips2c.hpp"

#include <cstdio>
#include <cstring>

#include "jak/collide/mem.hpp"
#include "jak/math/collide.hpp"
#include "jak/math/math.hpp"

namespace jak {

using namespace jak::mips;

namespace {

float f_of(uint64_t r) {
    uint32_t u = static_cast<uint32_t>(r);
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}
uint64_t r_of(float f) {
    int32_t i;
    std::memcpy(&i, &f, 4);
    return static_cast<uint64_t>(static_cast<int64_t>(i)); // mfc1 sign-extends
}
Vec& vec_at(uint64_t addr) { return goal_ref<Vec>(static_cast<uint32_t>(addr)); }

// A jalr: the callee gets a fresh context with the argument registers, s6, s7 and sp; its v0
// comes back. (The PC port's mips2c functions each make their own ExecutionContext too.)
void call(Ctx* c, Mips2cFn fn) {
    Ctx callee;
    for (int r : {a0, a1, a2, a3, t0, t1, t2, t3, s6, s7, sp}) callee.gprs[r] = c->gprs[r];
    c->gprs[v0].du64[0] = fn(&callee);
}

// (ray-sphere-intersect ray-origin ray-dir sph-origin radius)
uint64_t goal_ray_sphere_intersect(Ctx* c) {
    return r_of(ray_sphere_intersect(vec_at(c->sgpr64(a0)), vec_at(c->sgpr64(a1)),
                                     vec_at(c->sgpr64(a2)), f_of(c->sgpr64(a3))));
}
// (ray-cylinder-intersect ray-origin ray-dir cyl-origin cyl-axis cyl-rad cyl-len pt-out)
uint64_t goal_ray_cylinder_intersect(Ctx* c) {
    return r_of(ray_cylinder_intersect(vec_at(c->sgpr64(a0)), vec_at(c->sgpr64(a1)),
                                       vec_at(c->sgpr64(a2)), vec_at(c->sgpr64(a3)),
                                       f_of(c->sgpr64(t0)), f_of(c->sgpr64(t1)),
                                       vec_at(c->sgpr64(t2))));
}
// (closest-pt-in-triangle out pt tri-matrix normal)
uint64_t goal_closest_pt_in_triangle(Ctx* c) {
    closest_pt_in_triangle(vec_at(c->sgpr64(a0)), vec_at(c->sgpr64(a1)), &vec_at(c->sgpr64(a2)),
                           vec_at(c->sgpr64(a3)));
    return kGoalFalse;
}
// (vector-vector-distance a b)
uint64_t goal_vector_vector_distance(Ctx* c) {
    return r_of(vector_vector_distance(vec_at(c->sgpr64(a0)), vec_at(c->sgpr64(a1))));
}

} // namespace

uint64_t goal_call(Mips2cFn fn, const uint64_t* args, int nargs) {
    static const int regs[8] = {a0, a1, a2, a3, t0, t1, t2, t3};
    Ctx c;
    for (int i = 0; i < nargs && i < 8; i++) c.gprs[regs[i]].du64[0] = args[i];
    c.gprs[s7].du64[0] = kGoalFalse;
    c.gprs[sp].du64[0] = g_goal_sp;
    return fn(&c);
}

// moving-sphere-triangle-intersect (collide-func.gc); OpenGOAL game/mips2c/jak2_functions/collide_func.cpp.
uint64_t m2c_moving_sphere_triangle_intersect(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  c->daddiu(sp, sp, -16);
  c->sd(ra, 0, sp);
  c->lqc2(vf14, 0, a0);
  c->lqc2(vf15, 0, a1);
  c->lqc2(vf11, 0, a3);
  c->lqc2(vf12, 16, a3);
  c->lqc2(vf13, 32, a3);
  c->mov128_vf_gpr(vf1, a2);
  c->vadd(DEST::xyzw, vf6, vf14, vf15);
  c->vsub(DEST::xyzw, vf11, vf11, vf12);
  c->vsub(DEST::xyzw, vf13, vf13, vf12);
  c->vsub(DEST::xyzw, vf14, vf14, vf12);
  c->vsub(DEST::xyzw, vf6, vf6, vf12);
  c->vmini(DEST::xyzw, vf2, vf11, vf0);
  c->vopmula(vf13, vf11);
  c->vopmsub(vf16, vf11, vf13);
  c->vmax(DEST::xyzw, vf3, vf11, vf0);
  c->vmini(DEST::xyzw, vf4, vf14, vf6);
  c->vmax(DEST::xyzw, vf5, vf14, vf6);
  c->vmul(DEST::xyzw, vf7, vf16, vf16);
  c->vmini(DEST::xyzw, vf2, vf2, vf13);
  c->vmax(DEST::xyzw, vf3, vf3, vf13);
  c->vsub_bc(DEST::xyzw, BC::x, vf4, vf4, vf1);
  c->vadd_bc(DEST::x, BC::y, vf7, vf7, vf7);
  c->vadd_bc(DEST::xyzw, BC::x, vf5, vf5, vf1);
  c->vsub(DEST::xyzw, vf3, vf3, vf4);
  c->vadd_bc(DEST::x, BC::z, vf7, vf7, vf7);
  c->vsub(DEST::xyzw, vf5, vf5, vf2);
  c->mov128_gpr_vf(t2, vf3);
  c->mov128_gpr_vf(v1, vf5);
  c->vrsqrt(vf0, BC::w, vf7, BC::x);
  c->or_(v1, t2, v1);
  c->pcgtw(v1, r0, v1);
  c->ppach(v1, r0, v1);
  c->dsll(v1, v1, 16);
  bc = c->sgpr64(v1) != 0;
  if (bc) {goto block_14;}

  c->vmul(DEST::xyzw, vf2, vf16, vf15);
  c->vmul(DEST::xyzw, vf3, vf16, vf14);
  c->vadd_bc(DEST::x, BC::y, vf2, vf2, vf2);
  c->vsub_bc(DEST::y, BC::y, vf3, vf0, vf3);
  c->vadd_bc(DEST::x, BC::z, vf2, vf2, vf2);
  c->vsub_bc(DEST::y, BC::x, vf3, vf3, vf3);
  c->vsub_bc(DEST::y, BC::z, vf3, vf3, vf3);
  c->vadd_bc(DEST::x, BC::x, vf3, vf0, vf0);
  c->vadd_bc(DEST::x, BC::x, vf4, vf0, vf0);
  c->vwaitq();
  c->vmulq(DEST::xyzw, vf16, vf16);
  c->vmove(DEST::w, vf16, vf0);
  c->vmulq(DEST::xyzw, vf2, vf2);
  c->vmulq(DEST::xyzw, vf4, vf3);
  c->vmulq(DEST::xyzw, vf3, vf3);
  c->sqc2(vf16, 0, t1);
  c->vdiv(vf0, BC::w, vf2, BC::x);
  c->vadd_bc(DEST::y, BC::x, vf3, vf3, vf1);
  c->vsub_bc(DEST::y, BC::x, vf4, vf4, vf1);
  c->vwaitq();
  c->vmulq(DEST::xyzw, vf3, vf3);
  c->vmulq(DEST::xyzw, vf4, vf4);
  c->mov128_gpr_vf(v1, vf3);
  c->mov128_gpr_vf(t1, vf4);
  bc = ((int64_t)c->sgpr64(v1)) < 0;
  if (bc) {goto block_11;}

  bc = ((int64_t)c->sgpr64(t1)) < 0;
  if (bc) {goto block_12;}

  c->dsubu(v1, v1, t1);
  bc = ((int64_t)c->sgpr64(v1)) < 0;
  if (bc) {goto block_6;}

  c->vsub_bc(DEST::xyzw, BC::w, vf2, vf4, vf0);
  c->mov128_gpr_vf(v1, vf2);
  bc = ((int64_t)c->sgpr64(v1)) >= 0;
  if (bc) {goto block_14;}

  c->mov128_gpr_vf(v0, vf4);
  c->vmula_bc(DEST::xyzw, BC::w, vf14, vf0);
  c->vmadd_bc(DEST::xyzw, BC::y, vf8, vf15, vf4);
  goto block_8;


  block_6:
  c->vsub_bc(DEST::xyzw, BC::w, vf2, vf3, vf0);
  c->mov128_gpr_vf(v1, vf2);
  bc = ((int64_t)c->sgpr64(v1)) >= 0;
  if (bc) {goto block_14;}

  c->vmula_bc(DEST::xyzw, BC::w, vf14, vf0);
  c->vmadd_bc(DEST::xyzw, BC::y, vf8, vf15, vf3);
  c->mov128_gpr_vf(v0, vf3);

  block_8:
  c->dsra32(v0, v0, 0);
  c->vsub(DEST::xyzw, vf9, vf8, vf13);
  c->vsub(DEST::xyzw, vf10, vf8, vf11);
  c->vopmula(vf13, vf8);
  c->vopmsub(vf5, vf8, vf13);
  c->vopmula(vf8, vf11);
  c->vopmsub(vf6, vf11, vf8);
  c->vopmula(vf9, vf10);
  c->vopmsub(vf7, vf10, vf9);
  c->vmul(DEST::xyzw, vf5, vf5, vf16);
  c->vmul(DEST::xyzw, vf6, vf6, vf16);
  c->vmul(DEST::xyzw, vf7, vf7, vf16);
  c->vadd_bc(DEST::y, BC::x, vf5, vf5, vf5);
  c->vadd_bc(DEST::y, BC::x, vf6, vf6, vf6);
  c->vadd_bc(DEST::y, BC::x, vf7, vf7, vf7);
  c->vadd_bc(DEST::y, BC::z, vf5, vf5, vf5);
  c->vadd_bc(DEST::y, BC::z, vf6, vf6, vf6);
  c->vadd_bc(DEST::y, BC::z, vf7, vf7, vf7);
  c->mov128_gpr_vf(t1, vf5);
  c->mov128_gpr_vf(t2, vf6);
  c->mov128_gpr_vf(v1, vf7);
  c->or_(t1, t1, t2);
  c->or_(v1, t1, v1);
  bc = ((int64_t)c->sgpr64(v1)) < 0;
  if (bc) {goto block_10;}

  c->vopmula(vf8, vf16);
  c->vopmsub(vf5, vf16, vf8);
  c->vopmula(vf16, vf5);
  c->vopmsub(vf5, vf5, vf16);
  c->vadd(DEST::xyzw, vf5, vf5, vf12);
  c->sqc2(vf5, 0, t0);
  goto block_15;


  block_10:
  call(c, m2c_collide_do_primitives);
  c->mov64(v1, v0);
  goto block_15;


  block_11:
  bc = ((int64_t)c->sgpr64(t1)) < 0;
  if (bc) {goto block_14;}


  block_12:
  c->vsub(DEST::xyzw, vf9, vf14, vf13);
  c->vsub(DEST::xyzw, vf10, vf14, vf11);
  c->vopmula(vf13, vf14);
  c->vopmsub(vf5, vf14, vf13);
  c->vopmula(vf14, vf11);
  c->vopmsub(vf6, vf11, vf14);
  c->vopmula(vf9, vf10);
  c->vopmsub(vf7, vf10, vf9);
  c->vmul(DEST::xyzw, vf5, vf5, vf16);
  c->vmul(DEST::xyzw, vf6, vf6, vf16);
  c->vmul(DEST::xyzw, vf7, vf7, vf16);
  c->vadd_bc(DEST::y, BC::x, vf5, vf5, vf5);
  c->vadd_bc(DEST::y, BC::x, vf6, vf6, vf6);
  c->vadd_bc(DEST::y, BC::x, vf7, vf7, vf7);
  c->vadd_bc(DEST::y, BC::z, vf5, vf5, vf5);
  c->vadd_bc(DEST::y, BC::z, vf6, vf6, vf6);
  c->vadd_bc(DEST::y, BC::z, vf7, vf7, vf7);
  c->mov128_gpr_vf(t1, vf5);
  c->mov128_gpr_vf(t2, vf6);
  c->mov128_gpr_vf(v1, vf7);
  c->or_(t1, t1, t2);
  c->or_(v1, t1, v1);
  bc = ((int64_t)c->sgpr64(v1)) < 0;
  if (bc) {goto block_10;}

  c->vopmula(vf14, vf16);
  c->vopmsub(vf5, vf16, vf14);
  c->vopmula(vf16, vf5);
  c->vopmsub(vf5, vf5, vf16);
  c->vadd(DEST::xyzw, vf5, vf5, vf12);
  c->sqc2(vf5, 0, t0);
  c->addiu(v0, r0, 0);
  goto block_15;


  block_14:
  c->lui(v0, -13122);
  c->ori(v0, v0, 48160);

  block_15:
  c->ld(ra, 0, sp);
  c->daddiu(sp, sp, 16);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// collide-do-primitives (collide-func.gc); OpenGOAL game/mips2c/jak2_functions/collide_func.cpp.
uint64_t m2c_collide_do_primitives(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  [[maybe_unused]] bool cop1_bc = false;
  c->daddiu(sp, sp, -144);
  c->sd(ra, 0, sp);
  c->sq(s1, 32, sp);
  c->sq(s2, 48, sp);
  c->sq(s3, 64, sp);
  c->sq(s4, 80, sp);
  c->sq(s5, 96, sp);
  c->sq(gp, 112, sp);
  c->swc1(f28, 128, sp);
  c->swc1(f30, 132, sp);
  c->mov64(s4, a0);
  c->mov64(s3, a1);
  c->mov64(s1, a2);
  c->mov64(s5, a3);
  c->mov64(gp, t0);
  c->daddiu(s2, sp, 16);
  c->lui(v1, 16384);
  c->mtc1(f31, v1);
  c->mtc1(f28, r0);
  c->mov64(a0, s4);
  c->mov64(a1, s3);
  c->daddu(a2, r0, s5);
  c->mov64(a3, s1);
  call(c, goal_ray_sphere_intersect);
  c->mov64(v1, v0);
  c->mtc1(f30, v0);
  cop1_bc = c->fprs[f30] < c->fprs[f28];
  bc = cop1_bc;
  if (bc) {goto block_2;}

  c->mtc1(f31, v0);
  c->lqc2(vf31, 0, s5);

  block_2:
  c->mov64(a0, s4);
  c->mov64(a1, s3);
  c->daddiu(a2, s5, 16);
  c->mov64(a3, s1);
  call(c, goal_ray_sphere_intersect);
  c->mov64(v1, v0);
  c->mtc1(f30, v0);
  cop1_bc = c->fprs[f30] < c->fprs[f28];
  bc = cop1_bc;
  cop1_bc = c->fprs[f30] < c->fprs[f31];
  if (bc) {goto block_5;}

  bc = !cop1_bc;
  if (bc) {goto block_5;}

  c->mtc1(f31, v0);
  c->lqc2(vf31, 16, s5);

  block_5:
  c->mov64(a0, s4);
  c->mov64(a1, s3);
  c->daddiu(a2, s5, 32);
  c->mov64(a3, s1);
  call(c, goal_ray_sphere_intersect);
  c->mov64(v1, v0);
  c->mtc1(f30, v0);
  cop1_bc = c->fprs[f30] < c->fprs[f28];
  bc = cop1_bc;
  cop1_bc = c->fprs[f30] < c->fprs[f31];
  if (bc) {goto block_8;}

  bc = !cop1_bc;
  if (bc) {goto block_8;}

  c->mtc1(f31, v0);
  c->lqc2(vf31, 32, s5);

  block_8:
  c->lqc2(vf1, 0, s5);
  c->lqc2(vf2, 16, s5);
  c->vsub(DEST::xyzw, vf2, vf2, vf1);
  c->vmul(DEST::xyzw, vf3, vf2, vf2);
  c->vadd_bc(DEST::x, BC::y, vf3, vf3, vf3);
  c->vadd_bc(DEST::x, BC::z, vf3, vf3, vf3);
  c->vrsqrt(vf0, BC::w, vf3, BC::x);
  c->mov128_gpr_vf(v1, vf3);
  c->mtc1(f30, v1);
  c->sqrts(f30, f30);
  c->vwaitq();
  c->vmulq(DEST::xyzw, vf2, vf2);
  c->sqc2(vf2, 0, s2);
  c->mfc1(t1, f30);
  c->mov64(a0, s4);
  c->mov64(a1, s3);
  c->daddu(a2, r0, s5);
  c->mov64(a3, s2);
  c->mov64(t0, s1);
  c->mov64(t2, gp);
  call(c, goal_ray_cylinder_intersect);
  c->mov64(v1, v0);
  c->mtc1(f30, v0);
  cop1_bc = c->fprs[f30] < c->fprs[f28];
  bc = cop1_bc;
  cop1_bc = c->fprs[f30] < c->fprs[f31];
  if (bc) {goto block_11;}

  bc = !cop1_bc;
  if (bc) {goto block_11;}

  c->mtc1(f31, v0);
  c->lqc2(vf31, 0, gp);

  block_11:
  c->lqc2(vf1, 16, s5);
  c->lqc2(vf2, 32, s5);
  c->vsub(DEST::xyzw, vf2, vf2, vf1);
  c->vmul(DEST::xyzw, vf3, vf2, vf2);
  c->vadd_bc(DEST::x, BC::y, vf3, vf3, vf3);
  c->vadd_bc(DEST::x, BC::z, vf3, vf3, vf3);
  c->vrsqrt(vf0, BC::w, vf3, BC::x);
  c->mov128_gpr_vf(v1, vf3);
  c->mtc1(f30, v1);
  c->sqrts(f30, f30);
  c->vwaitq();
  c->vmulq(DEST::xyzw, vf2, vf2);
  c->sqc2(vf2, 0, s2);
  c->mfc1(t1, f30);
  c->mov64(a0, s4);
  c->mov64(a1, s3);
  c->daddiu(a2, s5, 16);
  c->mov64(a3, s2);
  c->mov64(t0, s1);
  c->mov64(t2, gp);
  call(c, goal_ray_cylinder_intersect);
  c->mov64(v1, v0);
  c->mtc1(f30, v0);
  cop1_bc = c->fprs[f30] < c->fprs[f28];
  bc = cop1_bc;
  cop1_bc = c->fprs[f30] < c->fprs[f31];
  if (bc) {goto block_14;}

  bc = !cop1_bc;
  if (bc) {goto block_14;}

  c->mtc1(f31, v0);
  c->lqc2(vf31, 0, gp);

  block_14:
  c->lqc2(vf1, 32, s5);
  c->lqc2(vf2, 0, s5);
  c->vsub(DEST::xyzw, vf2, vf2, vf1);
  c->vmul(DEST::xyzw, vf3, vf2, vf2);
  c->vadd_bc(DEST::x, BC::y, vf3, vf3, vf3);
  c->vadd_bc(DEST::x, BC::z, vf3, vf3, vf3);
  c->vrsqrt(vf0, BC::w, vf3, BC::x);
  c->mov128_gpr_vf(v1, vf3);
  c->mtc1(f30, v1);
  c->sqrts(f30, f30);
  c->vwaitq();
  c->vmulq(DEST::xyzw, vf2, vf2);
  c->sqc2(vf2, 0, s2);
  c->mfc1(t1, f30);
  c->daddiu(a2, s5, 32);
  c->mov64(t2, gp);
  c->mov64(a0, s4);
  c->mov64(a1, s3);
  c->mov64(a3, s2);
  c->mov64(t0, s1);
  call(c, goal_ray_cylinder_intersect);
  c->mov64(v1, v0);
  c->mtc1(f30, v0);
  cop1_bc = c->fprs[f30] < c->fprs[f28];
  bc = cop1_bc;
  cop1_bc = c->fprs[f30] < c->fprs[f31];
  if (bc) {goto block_17;}

  bc = !cop1_bc;
  if (bc) {goto block_17;}

  c->mtc1(f31, v0);
  c->lqc2(vf31, 0, gp);

  block_17:
  c->lui(v1, 16256);
  c->mtc1(f0, v1);
  c->lui(v1, -13122);
  c->ori(v1, v1, 48160);
  cop1_bc = c->fprs[f0] < c->fprs[f31];
  if (cop1_bc) {
    c->mtc1(f31, v1);
    goto block_19;
  }

  block_19:
  c->mfc1(v0, f31);
  c->sqc2(vf31, 0, gp);
  c->ld(ra, 0, sp);
  c->lwc1(f30, 132, sp);
  c->lwc1(f28, 128, sp);
  c->lq(gp, 112, sp);
  c->lq(s5, 96, sp);
  c->lq(s4, 80, sp);
  c->lq(s3, 64, sp);
  c->lq(s2, 48, sp);
  c->lq(s1, 32, sp);
  c->daddiu(sp, sp, 144);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// (method 9 collide-cache-prim), resolve-moving-sphere-tri; OpenGOAL game/mips2c/jak2_functions/collide_cache.cpp.
uint64_t m2c_resolve_moving_sphere_tri(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  [[maybe_unused]] bool cop1_bc = false;
  c->daddiu(sp, sp, -672);
  c->sd(ra, 0, sp);
  c->sq(s1, 576, sp);
  c->sq(s2, 592, sp);
  c->sq(s3, 608, sp);
  c->sq(s4, 624, sp);
  c->sq(s5, 640, sp);
  c->sq(gp, 656, sp);
  c->mov64(gp, a1);
  c->mov64(s5, a2);
  c->mov64(s4, a3);
  c->daddiu(s3, sp, 16);
  c->mtc1(f0, r0);
  c->sw(t1, 8, s3);
  c->mtc1(f1, t0);
  c->lhu(v1, 40, a0);
  c->lwu(a1, 32, a0);
  cop1_bc = c->fprs[f0] <= c->fprs[f1];
  c->dsll(v1, v1, 6);
  bc = cop1_bc;
  c->daddiu(a1, a1, 4908);
  if (bc) {goto block_2;}

  c->lui(a2, 16384);
  c->mtc1(f1, a2);

  block_2:
  c->daddu(s2, a1, v1);
  c->swc1(f1, 0, s3);
  c->lhu(s1, 42, a0);
  c->swc1(f1, 4, s3);

  block_3:
  bc = c->sgpr64(s1) == 0;
  c->daddiu(s1, s1, -1);
  if (bc) {goto block_14;}

  c->mov64(a0, s5);
  c->mov64(a1, s4);
  c->lwc1(f0, 12, s5);
  c->mfc1(a2, f0);
  c->daddu(a3, r0, s2);
  c->daddiu(t0, s3, 64);
  c->daddiu(t1, s3, 80);
  call(c, m2c_moving_sphere_triangle_intersect);
  c->mov64(v1, v0);
  c->mtc1(f0, r0);
  c->lqc2(vf1, 0, s4);
  c->mtc1(f2, v1);
  c->lwc1(f1, 0, s3);
  cop1_bc = c->fprs[f2] < c->fprs[f0];
  c->lqc2(vf2, 80, s3);
  if (cop1_bc) {
    c->daddiu(s2, s2, 64);
    goto block_3;
  }

  cop1_bc = c->fprs[f1] <= c->fprs[f2];
  c->lqc2(vf3, 64, s3);
  if (cop1_bc) {
    c->daddiu(s2, s2, 64);
    goto block_3;
  }

  c->vmul(DEST::xyzw, vf5, vf1, vf2);
  c->lqc2(vf4, 0, s5);
  c->lwu(v1, 8, s3);
  c->vsub(DEST::xyzw, vf7, vf4, vf3);
  c->andi(v1, v1, 1);
  bc = c->sgpr64(v1) == 0;
  c->vadd_bc(DEST::x, BC::y, vf5, vf5, vf5);
  if (bc) {goto block_13;}

  c->vmul(DEST::xyzw, vf6, vf7, vf2);
  c->vadd_bc(DEST::x, BC::z, vf5, vf5, vf5);
  c->vadd_bc(DEST::x, BC::y, vf6, vf6, vf6);
  c->mov128_gpr_vf(v1, vf5);
  c->mtc1(f3, v1);
  cop1_bc = c->fprs[f0] <= c->fprs[f3];
  if (cop1_bc) {
    c->daddiu(s2, s2, 64);
    goto block_3;
  }

  c->vadd_bc(DEST::x, BC::z, vf6, vf6, vf6);
  c->mov128_gpr_vf(v1, vf6);
  c->mtc1(f4, v1);
  cop1_bc = c->fprs[f4] < c->fprs[f0];
  if (cop1_bc) {
    c->daddiu(s2, s2, 64);
    goto block_3;
  }

  block_13:
  c->lqc2(vf8, 0, s2);
  c->lqc2(vf9, 16, s2);
  c->lqc2(vf10, 32, s2);
  c->lwu(v1, 48, s2);
  c->lw(a0, 52, s2);
  c->swc1(f2, 0, s3);
  c->sqc2(vf3, 48, gp);
  c->sqc2(vf2, 64, gp);
  c->sqc2(vf8, 0, gp);
  c->sqc2(vf9, 16, gp);
  c->sqc2(vf10, 32, gp);
  c->sw(v1, 80, gp);
  c->sw(a0, 84, gp);
  c->daddiu(s2, s2, 64);
  goto block_3;


  block_14:
  c->lwc1(f1, 0, s3);
  c->lwc1(f5, 4, s3);
  cop1_bc = c->fprs[f1] == c->fprs[f5];
  if (!cop1_bc) {
    c->mfc1(v0, f1);
    goto block_17;
  }

  c->lui(v0, -13122);
  c->ori(v0, v0, 48160);

  block_17:
  c->ld(ra, 0, sp);
  c->lq(gp, 656, sp);
  c->lq(s5, 640, sp);
  c->lq(s4, 624, sp);
  c->lq(s3, 608, sp);
  c->lq(s2, 592, sp);
  c->lq(s1, 576, sp);
  c->daddiu(sp, sp, 672);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// (method 17 collide-cache), probe-using-spheres; OpenGOAL game/mips2c/jak2_functions/collide_cache.cpp.
uint64_t m2c_probe_using_spheres(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  c->daddiu(sp, sp, -96);
  c->sd(ra, 0, sp);
  c->sd(fp, 8, sp);
  c->mov64(fp, t9);
  c->sq(s2, 16, sp);
  c->sq(s3, 32, sp);
  c->sq(s4, 48, sp);
  c->sq(s5, 64, sp);
  c->sq(gp, 80, sp);
  c->mov64(gp, a1);
  c->gprs[s5].du64[0] = kAddrScratchpad;  // *fake-scratchpad-data*
  c->addiu(a3, r0, 64);
  c->lwu(a2, 116, gp);
  c->daddiu(v1, s5, 96);
  c->lwu(a1, 112, gp);
  c->dsubu(a3, a2, a3);
  bc = ((int64_t)c->sgpr64(a3)) > 0;
  if (bc) {goto block_20;}

  bc = c->sgpr64(a2) == 0;
  c->lqc2(vf1, 0, a1);
  if (bc) {goto block_5;}

  c->daddiu(a2, a2, -1);
  c->daddiu(a1, a1, 16);
  c->vsub_bc(DEST::xyz, BC::w, vf2, vf1, vf1);
  c->sqc2(vf1, 0, v1);
  c->vadd_bc(DEST::xyz, BC::w, vf3, vf1, vf1);
  c->daddiu(v1, v1, 48);
  c->vftoi0(DEST::xyzw, vf4, vf2);
  c->vftoi0(DEST::xyzw, vf5, vf3);
  c->sqc2(vf4, -32, v1);
  c->sqc2(vf5, -16, v1);

  block_3:
  bc = c->sgpr64(a2) == 0;
  c->lqc2(vf1, 0, a1);
  if (bc) {goto block_5;}

  c->daddiu(a2, a2, -1);
  c->daddiu(a1, a1, 16);
  c->vsub_bc(DEST::xyz, BC::w, vf4, vf1, vf1);
  c->sqc2(vf1, 0, v1);
  c->vadd_bc(DEST::xyz, BC::w, vf5, vf1, vf1);
  c->vmini(DEST::xyz, vf2, vf2, vf4);
  c->vmax(DEST::xyz, vf3, vf3, vf5);
  c->vftoi0(DEST::xyzw, vf4, vf4);
  c->vftoi0(DEST::xyzw, vf5, vf5);
  c->sqc2(vf4, 16, v1);
  c->sqc2(vf5, 32, v1);
  c->daddiu(v1, v1, 48);
  goto block_3;


  block_5:
  c->vftoi0(DEST::xyzw, vf2, vf2);
  c->vftoi0(DEST::xyzw, vf3, vf3);
  c->sqc2(vf2, 64, s5);
  c->sqc2(vf3, 80, s5);
  c->daddiu(s4, a0, 108);
  c->lwu(s3, 100, gp);
  c->lw(s2, 4, a0);
  goto block_18;


  block_6:
  c->daddiu(s2, s2, -1);
  c->lwu(v1, 16, s4);
  c->and_(v1, s3, v1);
  bc = c->sgpr64(v1) == 0;
  c->mov64(v1, s7);
  if (bc) {goto block_17;}

  c->lwu(v1, 120, gp);
  if (((int64_t)c->sgpr64(s7)) == ((int64_t)c->sgpr64(v1))) {
    c->daddiu(v1, s7, 4);
    goto block_10;
  }

  c->daddiu(v1, s7, 4);
  c->lwu(a0, 24, s4);
  c->andi(a0, a0, 1);
  c->movz(v1, s7, a0);

  block_10:
  bc = c->sgpr64(s7) == c->sgpr64(v1);
  c->mov64(v1, s7);
  if (bc) {goto block_17;}

  c->mov64(v1, s7);
  c->lb(v1, 28, s4);
  c->slt(v1, v1, r0);
  bc = c->sgpr64(v1) != 0;
  if (bc) {goto block_13;}

  c->mov64(a0, s5);
  c->mov64(a1, s4);
  c->mov64(a2, gp);
  call(c, m2c_puss_work_mesh);
  c->mov64(v1, v0);
  c->mov64(a0, v1);
  goto block_14;


  block_13:
  c->mov64(a0, s5);
  c->mov64(a1, s4);
  c->mov64(a2, gp);
  call(c, m2c_puss_work_sphere);
  c->mov64(v1, v0);
  c->mov64(a0, v1);

  block_14:
  bc = c->sgpr64(s7) == c->sgpr64(v1);
  c->mov64(v1, s7);
  if (bc) {goto block_17;}

  c->daddiu(v1, s7, 4);
  c->mov64(v0, v1);
  goto block_22;

  c->gprs[v1].du64[0] = 0;

  block_17:
  c->daddiu(s4, s4, 48);

  block_18:
  bc = c->sgpr64(s2) != 0;
  if (bc) {goto block_6;}

  c->mov64(v1, s7);
  c->mov64(v1, s7);
  goto block_21;


  block_20:
  c->addiu(a0, r0, 0);
  // daddiu a1, fp, L167                               // daddiu a1, fp, L167
  printf("ERROR: Exceeded max # of spheres in collide-cache::probe-using-spheres!\n");

  block_21:
  c->mov64(v0, s7);

  block_22:
  c->ld(ra, 0, sp);
  c->ld(fp, 8, sp);
  c->lq(gp, 80, sp);
  c->lq(s5, 64, sp);
  c->lq(s4, 48, sp);
  c->lq(s3, 32, sp);
  c->lq(s2, 16, sp);
  c->daddiu(sp, sp, 96);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// (method 9 collide-puss-work): spheres against a prim's triangles; OpenGOAL game/mips2c/jak2_functions/collide_cache.cpp.
uint64_t m2c_puss_work_mesh(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  c->daddiu(sp, sp, -128);
  c->sd(ra, 0, sp);
  c->sq(s0, 16, sp);
  c->sq(s1, 32, sp);
  c->sq(s2, 48, sp);
  c->sq(s3, 64, sp);
  c->sq(s4, 80, sp);
  c->sq(s5, 96, sp);
  c->sq(gp, 112, sp);
  c->mov64(gp, a0);
  c->mov64(s5, a2);
  c->lwu(a0, 32, a1);
  c->lhu(v1, 40, a1);
  c->daddiu(a0, a0, 4908);
  c->lhu(s4, 42, a1);
  c->dsll(v1, v1, 6);
  c->daddu(s3, a0, v1);

  block_1:
  bc = c->sgpr64(s4) == 0;
  c->lqc2(vf1, 0, s3);
  if (bc) {goto block_12;}

  c->daddiu(s4, s4, -1);
  c->lqc2(vf2, 16, s3);
  c->lqc2(vf3, 32, s3);
  c->vsub(DEST::xyzw, vf4, vf2, vf1);
  c->lq(a1, 64, gp);
  c->vsub(DEST::xyzw, vf5, vf3, vf1);
  c->lq(v1, 80, gp);
  c->vmini(DEST::xyzw, vf6, vf1, vf2);
  c->vmax(DEST::xyzw, vf7, vf1, vf2);
  c->vopmula(vf4, vf5);
  c->vmove(DEST::xyzw, vf8, vf0);
  c->vmini(DEST::xyzw, vf6, vf6, vf3);
  c->vmax(DEST::xyzw, vf7, vf7, vf3);
  c->vopmsub(vf8, vf5, vf4);
  c->vftoi0(DEST::xyzw, vf6, vf6);
  c->vftoi0(DEST::xyzw, vf7, vf7);
  c->vmul(DEST::xyzw, vf9, vf8, vf8);
  c->mov128_gpr_vf(a0, vf6);
  c->mov128_gpr_vf(a2, vf7);
  c->vmula_bc(DEST::w, BC::x, vf0, vf9);
  c->sqc2(vf6, 32, gp);
  c->vmadda_bc(DEST::w, BC::y, vf0, vf9);
  c->sqc2(vf7, 48, gp);
  c->vmadd_bc(DEST::w, BC::z, vf9, vf0, vf9);
  c->pcgtw(a1, a1, a2);
  c->mfc1(r0, f31);
  c->pcgtw(v1, a0, v1);
  c->mfc1(r0, f31);
  c->por(v1, a1, v1);
  c->mfc1(r0, f31);
  c->vrsqrt(vf0, BC::w, vf9, BC::w);
  c->ppach(v1, r0, v1);
  c->mfc1(r0, f31);
  c->dsll(v1, v1, 16);
  if (((int64_t)c->sgpr64(v1)) != ((int64_t)0)) {
    c->daddiu(s3, s3, 64);
    goto block_1;
  }

  c->vwaitq();
  c->vmulq(DEST::xyz, vf8, vf8);
  c->daddiu(s2, gp, 96);
  c->lwu(s1, 116, s5);
  c->gprs[s0].du64[0] = 0;
  c->sqc2(vf8, 16, gp);

  block_5:
  if (((int64_t)c->sgpr64(s0)) == ((int64_t)c->sgpr64(s1))) {
    c->daddiu(s3, s3, 64);
    goto block_1;
  }

  c->daddiu(s0, s0, 1);
  c->lq(a1, 16, s2);
  c->lq(a2, 48, gp);
  c->lq(v1, 32, s2);
  c->lq(a0, 32, gp);
  c->pcgtw(a1, a1, a2);
  c->mfc1(r0, f31);
  c->pcgtw(v1, a0, v1);
  c->mfc1(r0, f31);
  c->por(v1, a1, v1);
  c->mfc1(r0, f31);
  c->ppach(v1, r0, v1);
  c->mfc1(r0, f31);
  c->dsll(v1, v1, 16);
  if (((int64_t)c->sgpr64(v1)) != ((int64_t)0)) {
    c->daddiu(s2, s2, 48);
    goto block_5;
  }

  c->daddu(a0, r0, gp);
  c->daddu(a1, r0, s2);
  c->daddu(a2, r0, s3);
  c->daddiu(a3, gp, 16);
  call(c, goal_closest_pt_in_triangle);
  c->lqc2(vf10, 0, gp);
  c->lqc2(vf11, 0, s2);
  c->daddiu(s2, s2, 48);
  c->vsub(DEST::xyz, vf9, vf10, vf11);
  c->vmul(DEST::w, vf11, vf11, vf11);
  c->vmul(DEST::xyzw, vf9, vf9, vf9);
  c->vmula_bc(DEST::w, BC::x, vf0, vf9);
  c->vmadda_bc(DEST::w, BC::y, vf0, vf9);
  c->vmadd_bc(DEST::w, BC::z, vf9, vf0, vf9);
  c->vsub(DEST::w, vf9, vf9, vf11);
  c->mov128_gpr_vf(v1, vf9);
  c->pcpyud(v1, v1, v1);
  if (((int64_t)c->sgpr64(v1)) > 0) {
    goto block_5;
  }

  c->daddiu(v1, s7, 4);
  c->mov64(v0, v1);
  goto block_14;


  block_12:
  c->mov64(v0, s7);
  goto block_14;

  c->gprs[v0].du64[0] = 0;

  block_14:
  c->ld(ra, 0, sp);
  c->lq(gp, 112, sp);
  c->lq(s5, 96, sp);
  c->lq(s4, 80, sp);
  c->lq(s3, 64, sp);
  c->lq(s2, 48, sp);
  c->lq(s1, 32, sp);
  c->lq(s0, 16, sp);
  c->daddiu(sp, sp, 128);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// (method 10 collide-puss-work): spheres against a sphere prim; OpenGOAL game/mips2c/jak2_functions/collide_cache.cpp.
uint64_t m2c_puss_work_sphere(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  c->lwu(v1, 116, a2);
  c->daddiu(a2, a0, 96);
  c->lqc2(vf1, 0, a1);
  bc = c->sgpr64(v1) == 0;
  c->lq(a1, 64, a0);
  if (bc) {goto block_12;}

  c->vmax_bc(DEST::xyzw, BC::w, vf4, vf0, vf0);
  c->lq(a3, 80, a0);
  c->vsub_bc(DEST::xyz, BC::w, vf2, vf1, vf1);
  c->gprs[a0].du64[0] = 0;
  c->vadd_bc(DEST::xyz, BC::w, vf3, vf1, vf1);
  c->lqc2(vf5, 0, a2);
  c->vftoi0(DEST::xyzw, vf2, vf2);
  c->lqc2(vf6, 48, a2);
  c->vftoi0(DEST::xyzw, vf3, vf3);
  c->lqc2(vf7, 96, a2);
  c->vsub_bc(DEST::w, BC::w, vf1, vf0, vf1);
  c->mov128_gpr_vf(t0, vf2);
  c->lqc2(vf8, 144, a2);
  c->mov128_gpr_vf(t1, vf3);
  c->pcgtw(a1, a1, t1);
  c->mfc1(r0, f31);
  c->pcgtw(a3, t0, a3);
  c->mfc1(r0, f31);
  c->por(a1, a1, a3);
  c->mfc1(r0, f31);
  c->ppach(a1, r0, a1);
  c->mfc1(r0, f31);
  c->dsll(a1, a1, 16);
  bc = c->sgpr64(a1) != 0;
  if (bc) {goto block_12;}


  block_2:
  c->vsub(DEST::xyzw, vf9, vf5, vf1);
  c->daddiu(a2, a2, 192);
  c->vsub(DEST::xyzw, vf10, vf6, vf1);
  c->lqc2(vf5, 0, a2);
  c->vsub(DEST::xyzw, vf11, vf7, vf1);
  c->lqc2(vf6, 48, a2);
  c->vsub(DEST::xyzw, vf12, vf8, vf1);
  c->lqc2(vf7, 96, a2);
  c->vmul(DEST::xyzw, vf9, vf9, vf9);
  c->lqc2(vf8, 144, a2);
  c->vmul(DEST::xyzw, vf10, vf10, vf10);
  c->vmul(DEST::xyzw, vf11, vf11, vf11);
  c->vmul(DEST::xyzw, vf12, vf12, vf12);
  c->vmula_bc(DEST::xyzw, BC::x, vf4, vf9);
  c->vmadda_bc(DEST::xyzw, BC::y, vf4, vf9);
  c->vmadda_bc(DEST::xyzw, BC::z, vf4, vf9);
  c->vmsub_bc(DEST::xyzw, BC::w, vf9, vf4, vf9);
  c->vmula_bc(DEST::xyzw, BC::x, vf4, vf10);
  c->vmadda_bc(DEST::xyzw, BC::y, vf4, vf10);
  c->vmadda_bc(DEST::xyzw, BC::z, vf4, vf10);
  c->vmsub_bc(DEST::xyzw, BC::w, vf10, vf4, vf10);
  c->mov128_gpr_vf(a1, vf9);
  c->daddiu(a0, a0, 1);
  bc = ((int64_t)c->sgpr64(a1)) <= 0;
  c->vmula_bc(DEST::xyzw, BC::x, vf4, vf11);
  if (bc) {goto block_11;}

  bc = c->sgpr64(a0) == c->sgpr64(v1);
  c->vmadda_bc(DEST::xyzw, BC::y, vf4, vf11);
  if (bc) {goto block_12;}

  c->vmadda_bc(DEST::xyzw, BC::z, vf4, vf11);
  c->vmsub_bc(DEST::xyzw, BC::w, vf11, vf4, vf11);
  c->mov128_gpr_vf(a1, vf10);
  c->daddiu(a0, a0, 1);
  bc = ((int64_t)c->sgpr64(a1)) <= 0;
  c->vmula_bc(DEST::xyzw, BC::x, vf4, vf12);
  if (bc) {goto block_11;}

  bc = c->sgpr64(a0) == c->sgpr64(v1);
  c->vmadda_bc(DEST::xyzw, BC::y, vf4, vf12);
  if (bc) {goto block_12;}

  c->vmadda_bc(DEST::xyzw, BC::z, vf4, vf12);
  c->vmsub_bc(DEST::xyzw, BC::w, vf12, vf4, vf12);
  c->mov128_gpr_vf(a1, vf11);
  c->daddiu(a0, a0, 1);
  bc = ((int64_t)c->sgpr64(a1)) <= 0;
  if (bc) {goto block_11;}

  bc = c->sgpr64(a0) == c->sgpr64(v1);
  if (bc) {goto block_12;}

  c->mov128_gpr_vf(a1, vf12);
  c->daddiu(a0, a0, 1);
  bc = ((int64_t)c->sgpr64(a1)) <= 0;
  if (bc) {goto block_11;}

  bc = c->sgpr64(a0) != c->sgpr64(v1);
  if (bc) {goto block_2;}

  goto block_12;


  block_11:
  c->daddiu(v1, s7, 4);
  c->mov64(v0, v1);
  goto block_14;


  block_12:
  c->mov64(v0, s7);
  goto block_14;

  c->gprs[v0].du64[0] = 0;

  block_14:
  c->daddu(sp, sp, r0);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// (method 10 collide-edge-hold-list), add-to-list!; OpenGOAL game/mips2c/jak2_functions/collide_edge_grab.cpp.
uint64_t m2c_add_to_list(Ctx* c) {
  [[maybe_unused]] bool cop1_bc = false;
  c->daddiu(a2, a0, 1552);
  c->lwu(t0, 4, a0);
  c->lqc2(vf1, 16, a1);
  c->dsll(a3, t0, 4);
  c->lwu(v1, 8, a0);
  c->daddiu(t0, t0, 1);
  c->lwc1(f0, 4, a1);
  c->daddu(a2, a2, a3);
  c->sw(t0, 4, a0);
  c->sqc2(vf1, 0, a2);
  if (((int64_t)c->sgpr64(v1)) == ((int64_t)c->sgpr64(s7))) {
    c->sw(a1, 8, a0);
    goto block_10;
  }

  c->lwc1(f1, 4, v1);
  cop1_bc = c->fprs[f0] < c->fprs[f1];
  if (cop1_bc) {
    c->sw(a1, 8, a0);
    goto block_11;
  }

  c->mov64(a0, v1);

  block_5:
  c->lwu(v1, 0, v1);
  if (((int64_t)c->sgpr64(v1)) == ((int64_t)c->sgpr64(s7))) {
    c->sw(a1, 0, a0);
    goto block_12;
  }

  c->lwc1(f1, 4, v1);
  cop1_bc = c->fprs[f0] < c->fprs[f1];
  if (!cop1_bc) {
    c->mov64(a0, v1);
    goto block_5;
  }

  c->sw(a1, 0, a0);
  c->sw(v1, 0, a1);
  goto block_13;


  block_10:
  c->sw(s7, 0, a1);
  goto block_13;


  block_11:
  c->sw(v1, 0, a1);
  goto block_13;


  block_12:
  c->sw(s7, 0, a1);

  block_13:
  c->gprs[v0].du64[0] = 0;
  c->daddu(sp, sp, r0);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// (method 19 collide-edge-work), find-best-grab!; OpenGOAL game/mips2c/jak2_functions/collide_edge_grab.cpp.
uint64_t m2c_find_best_grab(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  c->daddiu(sp, sp, -176);
  c->sd(ra, 0, sp);
  c->sq(s0, 64, sp);
  c->sq(s1, 80, sp);
  c->sq(s2, 96, sp);
  c->sq(s3, 112, sp);
  c->sq(s4, 128, sp);
  c->sq(s5, 144, sp);
  c->sq(gp, 160, sp);
  c->mov64(gp, a0);
  c->mov64(s5, a1);
  c->mov64(s4, a2);
  c->daddiu(s3, sp, 16);
  c->sd(r0, 256, s4);
  c->addiu(s2, r0, 16);

  block_1:
  bc = c->sgpr64(s2) == 0;
  c->daddiu(s2, s2, -1);
  if (bc) {goto block_25;}

  c->lwu(s1, 8, s5);
  bc = c->sgpr64(s1) == c->sgpr64(s7);
  if (bc) {goto block_25;}

  c->mov64(a0, gp);
  c->mov64(a1, s1);
  c->mov64(a2, s4);
  call(c, goal_check_grab_for_collisions);
  c->mov64(v1, v0);
  bc = c->sgpr64(v1) == c->sgpr64(s7);
  c->lwu(v1, 0, s1);
  if (bc) {goto block_5;}

  c->daddiu(v1, s7, 4);
  c->mov64(v0, v1);
  goto block_26;


  block_5:
  c->lb(s0, 8, s1);
  c->sw(v1, 8, s5);
  bc = ((int64_t)c->sgpr64(s0)) > 0;
  c->addiu(v1, r0, 2);
  if (bc) {goto block_17;}

  if (((int64_t)c->sgpr64(s0)) < 0) {
    c->dsubu(s0, r0, s0);
    goto block_21;
  }

  c->dsll(v1, s0, 2);
  c->daddiu(a0, gp, 352);
  c->daddu(v1, a0, v1);
  c->lw(v1, 0, v1);
  c->mov128_vf_gpr(vf1, v1);
  c->lwu(a2, 12, s1);
  c->vmove(DEST::xyzw, vf5, vf0);
  c->lqc2(vf3, 16, s1);
  c->vmove(DEST::xyzw, vf6, vf0);
  c->lqc2(vf4, 32, a2);
  c->lwu(v1, 12, a2);
  c->vmul_bc(DEST::xyz, BC::x, vf2, vf4, vf1);
  c->vadd(DEST::xyz, vf5, vf3, vf2);
  c->vsub(DEST::xyz, vf6, vf3, vf2);
  c->lqc2(vf8, 0, v1);
  c->sqc2(vf6, 16, s3);
  c->sqc2(vf2, 32, s3);
  c->sw(a2, 0, s3);
  c->sw(s7, 4, s3);
  c->vsub(DEST::xyz, vf9, vf8, vf5);
  c->vmul(DEST::xyz, vf9, vf9, vf2);
  c->vadd_bc(DEST::x, BC::y, vf9, vf9, vf9);
  c->vadd_bc(DEST::x, BC::z, vf9, vf9, vf9);
  c->mov128_gpr_vf(v1, vf9);
  bc = ((int64_t)c->sgpr64(v1)) < 0;
  if (bc) {goto block_12;}

  c->sqc2(vf5, 16, s1);
  c->mov64(a0, gp);
  c->mov64(a1, s1);
  call(c, m2c_should_add_to_list);
  c->mov64(v1, v0);
  bc = c->sgpr64(v1) == c->sgpr64(s7);
  if (bc) {goto block_12;}

  c->mov64(a0, s5);
  c->mov64(a1, s1);
  call(c, m2c_add_to_list);
  c->mov64(v1, v0);
  c->addiu(v1, r0, 1);
  c->sb(v1, 8, s1);
  c->addiu(v1, r0, 32);
  c->lwu(a0, 0, s5);
  c->addiu(a1, r0, 48);
  bc = c->sgpr64(a0) == c->sgpr64(v1);
  c->mult3(v1, a0, a1);
  if (bc) {goto block_1;}

  c->daddu(v1, v1, s5);
  c->sw(r0, 4, s3);
  c->daddiu(s1, v1, 16);

  block_12:
  c->lwu(a2, 0, s3);
  c->lqc2(vf6, 16, s3);
  c->lqc2(vf2, 32, s3);
  c->lwu(v1, 8, a2);
  c->lqc2(vf7, 0, v1);
  c->vsub(DEST::xyz, vf9, vf6, vf7);
  c->vmul(DEST::xyz, vf9, vf9, vf2);
  c->vadd_bc(DEST::x, BC::y, vf9, vf9, vf9);
  c->vadd_bc(DEST::x, BC::z, vf9, vf9, vf9);
  c->mov128_gpr_vf(v1, vf9);
  bc = ((int64_t)c->sgpr64(v1)) < 0;
  if (bc) {goto block_1;}

  c->sqc2(vf6, 16, s1);
  c->mov64(a0, gp);
  c->mov64(a1, s1);
  call(c, m2c_should_add_to_list);
  c->mov64(v1, v0);
  bc = c->sgpr64(v1) == c->sgpr64(s7);
  if (bc) {goto block_1;}

  c->mov64(a0, s5);
  c->mov64(a1, s1);
  call(c, m2c_add_to_list);
  c->mov64(v1, v0);
  c->lwu(v1, 4, s3);
  bc = c->sgpr64(v1) == c->sgpr64(s7);
  c->lwu(v1, 0, s5);
  if (bc) {goto block_16;}

  c->daddiu(v1, v1, 1);
  c->sw(v1, 0, s5);

  block_16:
  c->addiu(v1, r0, -1);
  c->sb(v1, 8, s1);
  goto block_1;


  block_17:
  bc = c->sgpr64(s0) == c->sgpr64(v1);
  c->lwu(a2, 12, s1);
  if (bc) {goto block_1;}

  c->dsll(v1, s0, 2);
  c->daddiu(a0, gp, 352);
  c->daddu(v1, a0, v1);
  c->lw(v1, 0, v1);
  c->mov128_vf_gpr(vf1, v1);
  c->lqc2(vf3, 16, s1);
  c->vmove(DEST::xyzw, vf5, vf0);
  c->lqc2(vf4, 32, a2);
  c->lwu(v1, 12, a2);
  c->vmul_bc(DEST::xyz, BC::x, vf2, vf4, vf1);
  c->vadd(DEST::xyz, vf5, vf3, vf2);
  c->lqc2(vf8, 0, v1);
  c->vsub(DEST::xyz, vf9, vf8, vf5);
  c->vmul(DEST::xyz, vf9, vf9, vf2);
  c->vadd_bc(DEST::x, BC::y, vf9, vf9, vf9);
  c->vadd_bc(DEST::x, BC::z, vf9, vf9, vf9);
  c->mov128_gpr_vf(v1, vf9);
  bc = ((int64_t)c->sgpr64(v1)) < 0;
  if (bc) {goto block_1;}

  c->sqc2(vf5, 16, s1);
  c->mov64(a0, gp);
  c->mov64(a1, s1);
  call(c, m2c_should_add_to_list);
  c->mov64(v1, v0);
  bc = c->sgpr64(v1) == c->sgpr64(s7);
  if (bc) {goto block_1;}

  c->mov64(a0, s5);
  c->mov64(a1, s1);
  call(c, m2c_add_to_list);
  c->mov64(v1, v0);
  c->daddiu(v1, s0, 1);
  c->sb(v1, 8, s1);
  goto block_1;


  block_21:
  bc = c->sgpr64(s0) == c->sgpr64(v1);
  c->lwu(a2, 12, s1);
  if (bc) {goto block_1;}

  c->dsll(v1, s0, 2);
  c->daddiu(a0, gp, 352);
  c->daddu(v1, a0, v1);
  c->lw(v1, 0, v1);
  c->mov128_vf_gpr(vf1, v1);
  c->lqc2(vf3, 16, s1);
  c->vmove(DEST::xyzw, vf6, vf0);
  c->lqc2(vf4, 32, a2);
  c->lwu(v1, 8, a2);
  c->vmul_bc(DEST::xyz, BC::x, vf2, vf4, vf1);
  c->vsub(DEST::xyz, vf6, vf3, vf2);
  c->lqc2(vf7, 0, v1);
  c->vsub(DEST::xyz, vf9, vf6, vf7);
  c->vmul(DEST::xyz, vf9, vf9, vf2);
  c->vadd_bc(DEST::x, BC::y, vf9, vf9, vf9);
  c->vadd_bc(DEST::x, BC::z, vf9, vf9, vf9);
  c->mov128_gpr_vf(v1, vf9);
  bc = ((int64_t)c->sgpr64(v1)) < 0;
  if (bc) {goto block_1;}

  c->sqc2(vf6, 16, s1);
  c->mov64(a0, gp);
  c->mov64(a1, s1);
  call(c, m2c_should_add_to_list);
  c->mov64(v1, v0);
  bc = c->sgpr64(v1) == c->sgpr64(s7);
  if (bc) {goto block_1;}

  c->mov64(a0, s5);
  c->mov64(a1, s1);
  call(c, m2c_add_to_list);
  c->mov64(v1, v0);
  c->daddiu(v1, s0, 1);
  c->dsubu(v1, r0, v1);
  c->sb(v1, 8, s1);
  goto block_1;


  block_25:
  c->mov64(v0, s7);

  block_26:
  c->ld(ra, 0, sp);
  c->lq(gp, 160, sp);
  c->lq(s5, 144, sp);
  c->lq(s4, 128, sp);
  c->lq(s3, 112, sp);
  c->lq(s2, 96, sp);
  c->lq(s1, 80, sp);
  c->lq(s0, 64, sp);
  c->daddiu(sp, sp, 176);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// (method 16 collide-edge-work), find-grabbable-edges; OpenGOAL game/mips2c/jak2_functions/collide_edge_grab.cpp.
static void sub_l20_b15(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  [[maybe_unused]] bool cop1_bc = false;
  c->mov64(t4, t1);
  c->lwu(t3, 12, a0);
  c->dsll32(t4, t4, 0);
  c->gprs[t5].du64[0] = 0;
  c->or_(t6, t4, t2);
  c->daddiu(t4, a0, 1712);

  block_16:
  bc = c->sgpr64(t5) == c->sgpr64(t3);
  c->ld(t7, 8, t4);
  if (bc) {goto block_20;}

  c->daddiu(t5, t5, 1);
  if (((int64_t)c->sgpr64(t6)) != ((int64_t)c->sgpr64(t7))) {
    c->daddiu(t4, t4, 48);
    goto block_16;
  }

  c->sw(r0, 0, t4);
  goto block_26;


  block_20:
  c->addiu(t5, r0, 96);
  if (((int64_t)c->sgpr64(t3)) == ((int64_t)c->sgpr64(t5))) {
    c->mov64(t4, s7);
    goto block_26;
  }

  c->daddiu(t3, t3, 1);
  c->sw(s7, 0, t4);
  c->sw(t1, 8, t4);
  c->sw(t2, 12, t4);
  c->sw(a1, 4, t4);
  c->sw(t3, 12, a0);
  c->vmove(DEST::xyzw, vf13, vf0);
  c->lqc2(vf16, 0, t1);
  c->lqc2(vf17, 0, t2);
  c->vsub(DEST::xyzw, vf18, vf17, vf16);
  c->vsub(DEST::xyzw, vf19, vf1, vf16);
  c->vmul(DEST::xyzw, vf20, vf18, vf18);
  c->vsub_bc(DEST::x, BC::z, vf13, vf0, vf18);
  c->vadd_bc(DEST::x, BC::y, vf20, vf20, vf20);
  c->vadd_bc(DEST::z, BC::x, vf13, vf0, vf18);
  c->vadd_bc(DEST::x, BC::z, vf20, vf20, vf20);
  c->vmul(DEST::xyzw, vf15, vf13, vf19);
  c->vmul(DEST::xyzw, vf14, vf13, vf13);
  c->vadd_bc(DEST::x, BC::z, vf15, vf15, vf15);
  c->vadd_bc(DEST::x, BC::z, vf14, vf14, vf14);
  c->mov128_gpr_vf(t2, vf15);
  c->vrsqrt(vf0, BC::w, vf14, BC::x);
  c->mtc1(f3, t2);
  cop1_bc = c->fprs[f4] < c->fprs[f2];
  bc = cop1_bc;
  if (bc) {goto block_25;}

  cop1_bc = c->fprs[f3] < c->fprs[f4];
  if (cop1_bc) {
    c->sw(r0, 0, t4);
    goto block_26;
  }

  block_25:
  c->vwaitq();
  c->vmulq(DEST::xyz, vf13, vf13);
  c->vrsqrt(vf0, BC::w, vf20, BC::x);
  c->vmul_bc(DEST::w, BC::w, vf18, vf0, vf0);
  c->sqc2(vf13, 16, t4);
  c->vwaitq();
  c->vmulq(DEST::xyz, vf18, vf18);
  c->sqc2(vf18, 32, t4);

  block_26:
  ;
}

static void sub_l25_b27(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  [[maybe_unused]] bool cop1_bc = false;
  c->gprs[t2].du64[0] = 0;
  c->lwu(t1, 8, a0);
  c->daddiu(t0, a0, 688);

  block_28:
  bc = c->sgpr64(t2) == c->sgpr64(t1);
  c->lqc2(vf9, 0, t0);
  if (bc) {goto block_37;}

  c->daddiu(t2, t2, 1);
  c->lqc2(vf10, 16, t0);
  c->vsub(DEST::xyzw, vf9, vf9, vf8);
  c->lqc2(vf11, 32, t0);
  c->vsub(DEST::xyzw, vf10, vf10, vf8);
  c->lqc2(vf12, 48, t0);
  c->vsub(DEST::xyzw, vf11, vf11, vf8);
  c->vsub(DEST::xyzw, vf12, vf12, vf8);
  c->vmul(DEST::xyzw, vf9, vf9, vf9);
  c->vmul(DEST::xyzw, vf10, vf10, vf10);
  c->vmul(DEST::xyzw, vf11, vf11, vf11);
  c->vmul(DEST::xyzw, vf12, vf12, vf12);
  c->vadd_bc(DEST::x, BC::y, vf9, vf9, vf9);
  c->vadd_bc(DEST::x, BC::y, vf10, vf10, vf10);
  c->vadd_bc(DEST::x, BC::y, vf11, vf11, vf11);
  c->vadd_bc(DEST::x, BC::y, vf12, vf12, vf12);
  c->vadd_bc(DEST::x, BC::z, vf9, vf9, vf9);
  c->vadd_bc(DEST::x, BC::z, vf10, vf10, vf10);
  c->vadd_bc(DEST::x, BC::z, vf11, vf11, vf11);
  c->vadd_bc(DEST::x, BC::z, vf12, vf12, vf12);
  c->mov128_gpr_vf(t3, vf9);
  c->mtc1(f1, t3);
  cop1_bc = c->fprs[f1] <= c->fprs[f0];
  bc = cop1_bc;
  if (bc) {goto block_40;}

  bc = c->sgpr64(t2) == c->sgpr64(t1);
  c->daddiu(t0, t0, 16);
  if (bc) {goto block_37;}

  c->mov128_gpr_vf(t3, vf10);
  c->daddiu(t2, t2, 1);
  c->mtc1(f1, t3);
  cop1_bc = c->fprs[f1] <= c->fprs[f0];
  bc = cop1_bc;
  if (bc) {goto block_40;}

  bc = c->sgpr64(t2) == c->sgpr64(t1);
  c->daddiu(t0, t0, 16);
  if (bc) {goto block_37;}

  c->mov128_gpr_vf(t3, vf11);
  c->daddiu(t2, t2, 1);
  c->mtc1(f1, t3);
  cop1_bc = c->fprs[f1] <= c->fprs[f0];
  bc = cop1_bc;
  if (bc) {goto block_40;}

  bc = c->sgpr64(t2) == c->sgpr64(t1);
  c->daddiu(t0, t0, 16);
  if (bc) {goto block_37;}

  c->mov128_gpr_vf(t3, vf12);
  c->daddiu(t2, t2, 1);
  c->mtc1(f1, t3);
  cop1_bc = c->fprs[f1] <= c->fprs[f0];
  bc = cop1_bc;
  if (bc) {goto block_40;}

  c->daddiu(t0, t0, 16);
  goto block_28;


  block_37:
  c->addiu(t2, r0, 64);
  if (((int64_t)c->sgpr64(t2)) == ((int64_t)c->sgpr64(t1))) {
    c->mov64(t0, s7);
    goto block_40;
  }

  c->daddiu(t1, t1, 1);
  c->sqc2(vf8, 0, t0);
  c->sw(t1, 8, a0);

  block_40:
  ;
}

uint64_t m2c_find_grabbable_edges(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  c->daddiu(sp, sp, -16);
  c->sd(ra, 0, sp);
  c->sd(fp, 8, sp);
  c->mov64(fp, t9);
  c->lui(v1, 17617);
  c->ori(v1, v1, 46871);
  c->mtc1(f0, v1);
  c->lwc1(f4, 404, a0);
  c->mtc1(f2, r0);
  c->lwu(a1, 4, a0);
  c->lwu(v1, 16, a0);
  c->lqc2(vf1, 12, a1);
  c->daddiu(a1, a0, 6320);
  c->lqc2(vf6, 64, a0);
  c->lqc2(vf7, 80, a0);
  c->vmove(DEST::xyzw, vf2, vf1);
  c->vadd_bc(DEST::y, BC::y, vf1, vf0, vf6);
  c->vadd_bc(DEST::y, BC::y, vf2, vf0, vf7);

  block_1:
  bc = c->sgpr64(v1) == 0;
  c->lwu(t0, 0, a1);
  if (bc) {goto block_43;}

  c->daddiu(v1, v1, -1);
  c->lqc2(vf3, 0, t0);
  c->lqc2(vf4, 16, t0);
  c->lqc2(vf5, 32, t0);
  // Unknown instr: bgezal r0, L25
  c->vmove(DEST::xyzw, vf8, vf3);
  // if (bc) {goto block_27;}                          // branch non-likely
  sub_l25_b27(c);

  bc = c->sgpr64(t0) == c->sgpr64(s7);
  c->mov64(a2, t0);
  if (bc) {goto block_41;}

  // Unknown instr: bgezal r0, L25
  c->vmove(DEST::xyzw, vf8, vf4);
  // if (bc) {goto block_27;}                          // branch non-likely
  sub_l25_b27(c);

  bc = c->sgpr64(t0) == c->sgpr64(s7);
  c->mov64(a3, t0);
  if (bc) {goto block_41;}

  // Unknown instr: bgezal r0, L25
  c->vmove(DEST::xyzw, vf8, vf5);
  // if (bc) {goto block_27;}                          // branch non-likely
  sub_l25_b27(c);

  bc = c->sgpr64(t0) == c->sgpr64(s7);
  c->mov64(t0, t0);
  if (bc) {goto block_41;}

  c->mov64(t1, a2);
  // Unknown instr: bgezal r0, L20
  c->mov64(t2, a3);
  // if (bc) {goto block_15;}                          // branch non-likely
  sub_l20_b15(c);

  bc = c->sgpr64(t4) == c->sgpr64(s7);
  c->mov64(t1, a3);
  if (bc) {goto block_42;}

  // Unknown instr: bgezal r0, L20
  c->mov64(t2, t0);
  // if (bc) {goto block_15;}                          // branch non-likely
  sub_l20_b15(c);

  bc = c->sgpr64(t4) == c->sgpr64(s7);
  c->mov64(t1, t0);
  if (bc) {goto block_42;}

  // Unknown instr: bgezal r0, L20
  c->mov64(t2, a2);
  // if (bc) {goto block_15;}                          // branch non-likely
  sub_l20_b15(c);

  bc = c->sgpr64(t1) == c->sgpr64(s7);
  if (bc) {goto block_42;}

  c->daddiu(a1, a1, 32);
  goto block_1;

  block_41:
  c->addiu(a0, r0, 0);
  //daddiu a1, fp, L118                               // daddiu a1, fp, L118
  printf("ERROR: Too many edge verts found in edge grab!\n");
  goto block_43;


  block_42:
  c->addiu(a0, r0, 0);
  //daddiu a1, fp, L117                               // daddiu a1, fp, L117
  printf("ERROR: Too many edges found in edge grab!\n");

  block_43:
  c->gprs[v0].du64[0] = 0;
  c->ld(ra, 0, sp);
  c->ld(fp, 8, sp);
  c->daddiu(sp, sp, 16);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// (method 17 collide-edge-work), find-grabbable-tris; OpenGOAL game/mips2c/jak2_functions/collide_edge_grab.cpp.
uint64_t m2c_find_grabbable_tris(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  [[maybe_unused]] bool cop1_bc = false;
  c->daddiu(sp, sp, -32);
  c->sd(ra, 0, sp);
  c->sd(fp, 8, sp);
  c->mov64(fp, t9);
  c->sq(gp, 16, sp);
  c->mov64(gp, a0);
  c->lui(v1, 16180);
  c->ori(v1, v1, 65012);
  c->mtc1(f0, v1);
  c->lwu(v1, 392, gp);
  c->addiu(a0, r0, 896);
  c->lwu(t3, 0, gp);
  c->addiu(a1, r0, 0);
  c->daddiu(a2, gp, 6320);
  c->addiu(a3, r0, 256);
  c->lwu(t0, 0, t3);
  c->gprs[t1].du64[0] = 0;
  c->lq(t2, 96, gp);
  c->daddiu(t3, t3, 4908);
  c->lq(t4, 112, gp);

  block_1:
  bc = c->sgpr64(t0) == 0;
  c->lwu(t5, 48, t3);
  if (bc) {goto block_14;}

  c->daddiu(t0, t0, -1);
  c->lqc2(vf1, 0, t3);
  c->and_(t6, t5, a0);
  c->lqc2(vf2, 16, t3);
  bc = c->sgpr64(t6) == c->sgpr64(a1);
  c->lqc2(vf3, 32, t3);
  if (bc) {goto block_5;}

  if (((int64_t)c->sgpr64(t6)) != ((int64_t)c->sgpr64(a3))) {
    c->daddiu(t3, t3, 64);
    goto block_1;
  }

  block_5:
  c->and_(t6, t5, v1);
  c->vmini(DEST::xyzw, vf7, vf1, vf2);
  bc = c->sgpr64(t6) != 0;
  c->vmax(DEST::xyzw, vf8, vf1, vf2);
  if (bc) {goto block_1;}

  c->vsub(DEST::xyz, vf4, vf2, vf1);
  c->vsub(DEST::xyz, vf5, vf3, vf1);
  c->vmini(DEST::xyzw, vf7, vf7, vf3);
  c->vmax(DEST::xyzw, vf8, vf8, vf3);
  c->vopmula(vf4, vf5);
  c->vftoi0(DEST::xyzw, vf7, vf7);
  c->vftoi0(DEST::xyzw, vf8, vf8);
  c->vopmsub(vf6, vf5, vf4);
  c->mov128_gpr_vf(t7, vf7);
  c->mov128_gpr_vf(t6, vf8);
  c->pcgtw(t7, t7, t4);
  c->mfc1(r0, f31);
  c->vmul(DEST::xyzw, vf9, vf6, vf6);
  c->pcgtw(t6, t2, t6);
  c->mfc1(r0, f31);
  c->por(t6, t7, t6);
  c->mfc1(r0, f31);
  c->ppach(t6, r0, t6);
  c->mfc1(r0, f31);
  c->dsll(t6, t6, 16);
  if (((int64_t)c->sgpr64(t6)) != ((int64_t)0)) {
    c->daddiu(t3, t3, 64);
    goto block_1;
  }

  c->vmula_bc(DEST::w, BC::x, vf0, vf9);
  c->vmadda_bc(DEST::w, BC::y, vf0, vf9);
  c->vmadd_bc(DEST::w, BC::z, vf9, vf0, vf9);
  c->vrsqrt(vf0, BC::w, vf9, BC::w);
  c->dsll32(t5, t5, 8);
  c->dsrl32(t5, t5, 26);
  c->addiu(t6, r0, 2);
  c->vwaitq();
  c->vmulq(DEST::xyz, vf6, vf6);
  c->mov128_gpr_vf(t7, vf6);
  c->dsra32(t7, t7, 0);
  c->mtc1(f1, t7);
  cop1_bc = c->fprs[f1] < c->fprs[f0];
  c->addiu(t7, r0, 48);
  if (cop1_bc) {
    c->daddiu(t3, t3, 64);
    goto block_1;
  }

  if (((int64_t)c->sgpr64(t5)) == ((int64_t)c->sgpr64(t6))) {
    c->daddiu(t3, t3, 64);
    goto block_1;
  }

  bc = c->sgpr64(t1) == c->sgpr64(t7);
  if (bc) {goto block_15;}

  c->daddiu(t1, t1, 1);
  c->sw(t3, 0, a2);
  c->sqc2(vf6, 16, a2);
  c->daddiu(a2, a2, 32);
  c->daddiu(t3, t3, 64);
  goto block_1;


  block_14:
  c->sw(t1, 16, gp);
  goto block_16;


  block_15:
  c->addiu(a0, r0, 0);
  // daddiu a1, fp, L119                               // daddiu a1, fp, L119
  printf("ERROR: Exceeded max # of grabbable tris!\n");
  c->addiu(v1, r0, 48);
  c->sw(v1, 16, gp);

  block_16:
  c->gprs[v0].du64[0] = 0;
  c->ld(ra, 0, sp);
  c->ld(fp, 8, sp);
  c->lq(gp, 16, sp);
  c->daddiu(sp, sp, 32);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

// (method 18 collide-edge-work), should-add-to-list?; OpenGOAL game/mips2c/jak2_functions/collide_edge_grab.cpp.
uint64_t m2c_should_add_to_list(Ctx* c) {
  [[maybe_unused]] bool bc = false;
  [[maybe_unused]] bool cop1_bc = false;
  float acc;
  c->daddiu(sp, sp, -96);
  c->sd(ra, 0, sp);
  c->sq(s3, 16, sp);
  c->sq(s4, 32, sp);
  c->sq(s5, 48, sp);
  c->sq(gp, 64, sp);
  c->swc1(f30, 80, sp);
  c->mov64(gp, a0);
  c->mov64(s5, a1);
  c->lqc2(vf1, 16, s5);
  c->lq(a0, 96, gp);
  c->lq(a3, 112, gp);
  c->vftoi0(DEST::xyzw, vf2, vf1);
  c->lwu(v1, 4, gp);
  c->mov128_gpr_vf(a1, vf2);
  c->lqc2(vf3, 16, a2);
  c->pcgtw(a3, a1, a3);
  c->lqc2(vf4, 368, gp);
  c->pcgtw(a0, a0, a1);
  c->lqc2(vf5, 12, v1);
  c->por(v1, a3, a0);
  c->lwc1(f0, 396, gp);
  c->ppach(v1, r0, v1);
  c->lwc1(f1, 400, gp);
  c->dsll(v1, v1, 16);
  bc = c->sgpr64(v1) != 0;
  c->lqc2(vf6, 144, gp);
  if (bc) {goto block_24;}

  c->vmul_bc(DEST::xyzw, BC::x, vf10, vf3, vf4);
  c->lqc2(vf11, 128, gp);
  c->vadd(DEST::xyzw, vf10, vf10, vf1);
  c->vsub(DEST::xyzw, vf7, vf5, vf10);
  c->vmul(DEST::xyzw, vf7, vf7, vf7);
  c->vadd_bc(DEST::x, BC::z, vf7, vf7, vf7);
  c->mov128_gpr_vf(v1, vf7);
  c->mtc1(f2, v1);
  cop1_bc = c->fprs[f0] < c->fprs[f2];
  bc = cop1_bc;
  if (bc) {goto block_24;}

  c->vsub(DEST::xyzw, vf8, vf1, vf5);
  c->vmul(DEST::xyzw, vf7, vf8, vf8);
  c->vadd_bc(DEST::x, BC::z, vf7, vf7, vf7);
  c->vrsqrt(vf0, BC::w, vf7, BC::x);
  c->vwaitq();
  c->vmulq(DEST::xz, vf8, vf8);
  c->vmul(DEST::xyzw, vf9, vf8, vf6);
  c->vadd_bc(DEST::x, BC::z, vf9, vf9, vf9);
  c->mov128_gpr_vf(v1, vf9);
  c->mtc1(f3, v1);
  cop1_bc = c->fprs[f3] < c->fprs[f1];
  bc = cop1_bc;
  if (bc) {goto block_24;}

  c->vsub(DEST::xyzw, vf7, vf11, vf1);
  c->sqc2(vf1, 16, s5);
  c->vmul(DEST::xyzw, vf7, vf7, vf7);
  c->sb(r0, 8, s5);
  c->sqc2(vf10, 32, s5);
  c->vadd_bc(DEST::x, BC::z, vf7, vf7, vf7);
  c->sw(a2, 12, s5);
  c->mov128_gpr_vf(v1, vf7);
  c->sw(v1, 4, s5);
  c->ld(v1, 384, gp);
  c->andi(v1, v1, 2);
  bc = c->sgpr64(v1) == 0;
  c->mov64(v1, s7);
  if (bc) {goto block_20;}

  c->lwu(s3, 12, s5);
  c->lwu(s4, 408, gp);
  c->lui(v1, 16256);
  c->mtc1(f30, v1);
  c->lwu(v1, 8, s3);
  c->daddu(a0, r0, v1);
  c->daddu(a1, r0, s4);
  call(c, goal_vector_vector_distance);
  c->mtc1(f0, v0);
  cop1_bc = c->fprs[f0] < c->fprs[f30];
  bc = cop1_bc;
  c->daddiu(v1, s7, 4);
  if (bc) {goto block_6;}

  c->mov64(v1, s7);

  block_6:
  if (((int64_t)c->sgpr64(s7)) != ((int64_t)c->sgpr64(v1))) {
    c->mov64(v1, v1);
    goto block_18;
  }

  c->lwu(v1, 12, s3);
  c->daddu(a0, r0, v1);
  c->daddu(a1, r0, s4);
  call(c, goal_vector_vector_distance);
  c->mtc1(f0, v0);
  cop1_bc = c->fprs[f0] < c->fprs[f30];
  bc = cop1_bc;
  c->daddiu(v1, s7, 4);
  if (bc) {goto block_10;}

  c->mov64(v1, s7);

  block_10:
  if (((int64_t)c->sgpr64(s7)) != ((int64_t)c->sgpr64(v1))) {
    c->mov64(v1, v1);
    goto block_18;
  }

  c->lwu(v1, 8, s3);
  c->daddu(a0, r0, v1);
  c->daddiu(a1, s4, 16);
  call(c, goal_vector_vector_distance);
  c->mtc1(f0, v0);
  cop1_bc = c->fprs[f0] < c->fprs[f30];
  bc = cop1_bc;
  c->daddiu(v1, s7, 4);
  if (bc) {goto block_14;}

  c->mov64(v1, s7);

  block_14:
  if (((int64_t)c->sgpr64(s7)) != ((int64_t)c->sgpr64(v1))) {
    c->mov64(v1, v1);
    goto block_18;
  }

  c->lwu(v1, 12, s3);
  c->daddu(a0, r0, v1);
  c->daddiu(a1, s4, 16);
  call(c, goal_vector_vector_distance);
  c->mtc1(f0, v0);
  cop1_bc = c->fprs[f0] < c->fprs[f30];
  bc = cop1_bc;
  c->daddiu(v1, s7, 4);
  if (bc) {goto block_18;}

  c->mov64(v1, s7);

  block_18:
  bc = c->sgpr64(s7) != c->sgpr64(v1);
  c->mov64(v1, s7);
  if (bc) {goto block_20;}

  goto block_24;


  block_20:
  c->ld(v1, 384, gp);
  c->andi(v1, v1, 4);
  bc = c->sgpr64(v1) == 0;
  c->mov64(v1, s7);
  if (bc) {goto block_23;}

  c->lwu(v1, 12, s5);
  c->daddiu(a0, v1, 16);
  c->daddiu(v1, gp, 144);
  c->lwc1(f0, 0, a0);
  c->lwc1(f1, 4, a0);
  c->lwc1(f2, 8, a0);
  c->lwc1(f3, 0, v1);
  c->lwc1(f4, 4, v1);
  c->lwc1(f5, 8, v1);
  // Unknown instr: mula.s f0, f3
  acc = c->fprs[f0] * c->fprs[f3];
  // Unknown instr: madda.s f1, f4
  acc += c->fprs[f1] * c->fprs[f4];
  // Unknown instr: madd.s f0, f2, f5
  c->fprs[f0] = acc + c->fprs[f2] * c->fprs[f5];
  c->mfc1(v1, f0);
  c->mtc1(f0, v1);
  // Unknown instr: abs.s f0, f0
  c->abss(f0, f0);
  c->lui(v1, 16179);
  c->ori(v1, v1, 13107);
  c->mtc1(f1, v1);
  cop1_bc = c->fprs[f1] < c->fprs[f0];
  bc = !cop1_bc;
  c->mov64(v1, s7);
  if (bc) {goto block_23;}

  goto block_24;


  block_23:
  c->daddiu(v1, s7, 4);
  c->mov64(v0, v1);
  goto block_26;


  block_24:
  c->mov64(v0, s7);
  goto block_26;

  c->gprs[v0].du64[0] = 0;

  block_26:
  c->ld(ra, 0, sp);
  c->lwc1(f30, 80, sp);
  c->lq(gp, 64, sp);
  c->lq(s5, 48, sp);
  c->lq(s4, 32, sp);
  c->lq(s3, 16, sp);
  c->daddiu(sp, sp, 96);
  goto end_of_function;
end_of_function:
  return c->gprs[v0].du64[0];
}

} // namespace jak
