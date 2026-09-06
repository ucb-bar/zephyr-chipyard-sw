/*
 * Production fp16 flight-controller kernels — RVV Zvfh VECTOR unit (ZERO scalar FP).
 *
 * Built -march=rv64imafc_zve64d_zvfh -mabi=lp64. Contains ONLY vector-intrinsic kernels;
 * every scalar (control flow, fp32 accumulator glue, fp16<->fp32 boundary, runtime
 * constants, comparisons) lives in fp16_fc_glue.c (soft-float rv64imac). See fp16_fc_vec.h.
 *
 * The kernels are verbatim relocations of the contiguous vector blocks of the original
 * kernel (same __riscv_* intrinsics / order -> bit-identical results). Runtime constants
 * arrive as integer bit-patterns and are splatted with vmv.v.x (never from a scalar float,
 * which would emit fcvt/fmv). The few spots that originally extracted an fp16 lane to a
 * scalar `_Float16` and rebuilt a small array (which emits flh/fsh) are re-expressed with
 * vse16(vl=1) / vslideup / indexed-load — same numeric values, no scalar-FP.
 *
 * ALGEBRA in fp16 vector (vfmul.vv/vfmacc.vv/vfredusum/vfsqrt.v); ACCUMULATORS in fp32
 * vector (vfLen=64 on this Saturn core); SATURATION clamps bound the mixer/force path.
 */
#include "fp16_fc_kernels.h"
#include "fp16_fc_vec.h"
#include <riscv_vector.h>

typedef vfloat16m1_t vh;
typedef vfloat32m1_t vf;

/* ---- fp16 helpers: integer-bit constant splat, vse16 extract (never scalar fp16) ---- */
static inline uint16_t hb(float c){ union{_Float16 h;uint16_t u;}x; x.h=(_Float16)c; return x.u; }
static inline vh h_splat(float c,size_t vl){ return __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vmv_v_x_u16m1(hb(c),vl)); }
static inline vh h_splat_bits(uint16_t b,size_t vl){ return __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vmv_v_x_u16m1(b,vl)); }
static inline vh h_bcast_lane(vh v,int k,size_t vl){ return __riscv_vrgather_vx_f16m1(v,(size_t)k,vl); }
static inline void h_store(_Float16*p,vh v,size_t vl){ __riscv_vse16_v_f16m1(p,v,vl); }
static inline void h_st1(_Float16*p,vh v){ __riscv_vse16_v_f16m1(p,v,1); }   /* store lane0 via vector store (no scalar fp16) */
static inline vh h_load(const _Float16*p,size_t vl){ return __riscv_vle16_v_f16m1(p,vl); }
/* dot product (fp16) -> broadcast vector holding the sum in every lane (no scalar extract) */
static inline vh h_dot_bcast(vh a,vh b,size_t n){
    vh p=__riscv_vfmul_vv_f16m1(a,b,n);
    vh r=__riscv_vfredusum_vs_f16m1_f16m1(p,h_splat(0.0f,n),n);   /* sum in lane0 */
    return __riscv_vrgather_vx_f16m1(r,0,n);                       /* broadcast lane0 */
}

/* ---- fp32 vector helpers (accumulators) ---- */
static inline uint32_t fb(float c){ union{float f;uint32_t u;}x; x.f=c; return x.u; }
static inline vf f_splat(float c,size_t vl){ return __riscv_vreinterpret_v_u32m1_f32m1(__riscv_vmv_v_x_u32m1(fb(c),vl)); }
static inline vf f_splat_bits(uint32_t b,size_t vl){ return __riscv_vreinterpret_v_u32m1_f32m1(__riscv_vmv_v_x_u32m1(b,vl)); }
static inline vf f_load(const float*p,size_t vl){ return __riscv_vle32_v_f32m1(p,vl); }
static inline void f_store(float*p,vf v,size_t vl){ __riscv_vse32_v_f32m1(p,v,vl); }
static inline vh f2h(vf v,size_t vl){ return __riscv_vlmul_ext_v_f16mf2_f16m1(__riscv_vfncvt_f_f_w_f16mf2(v,vl)); }
static inline vf h2f(vh v,size_t vl){ return __riscv_vfwcvt_f_f_v_f32m1(__riscv_vlmul_trunc_v_f16m1_f16mf2(v),vl); }
/* fp32 shuffle via gather (indices in a uint32 buffer): no scalar fp */
static inline vf f_shuf(vf v,const uint32_t idx[4],size_t vl){ return __riscv_vrgather_vv_f32m1(v,__riscv_vle32_v_u32m1(idx,vl),vl); }
static inline vh h_shuf(vh v,const uint16_t idx[4],size_t vl){ return __riscv_vrgather_vv_f16m1(v,__riscv_vle16_v_u16m1(idx,vl),vl); }

/* ---- vector-times-CONSTANT via .vv + integer-bit splat (compile-time c folds to vmv.v.x) ---- */
static inline vh hmulc(vh v,float c,size_t vl){ return __riscv_vfmul_vv_f16m1(v,h_splat(c,vl),vl); }
static inline vh haddc(vh v,float c,size_t vl){ return __riscv_vfadd_vv_f16m1(v,h_splat(c,vl),vl); }
static inline vh hsubc(vh v,float c,size_t vl){ return __riscv_vfsub_vv_f16m1(v,h_splat(c,vl),vl); }
static inline vh hminc(vh v,float c,size_t vl){ return __riscv_vfmin_vv_f16m1(v,h_splat(c,vl),vl); }
static inline vh hrecip(vh v,size_t vl){ return __riscv_vfdiv_vv_f16m1(h_splat(1.0f,vl),v,vl); }
static inline vf fmulc(vf v,float c,size_t vl){ return __riscv_vfmul_vv_f32m1(v,f_splat(c,vl),vl); }
static inline vf faddc(vf v,float c,size_t vl){ return __riscv_vfadd_vv_f32m1(v,f_splat(c,vl),vl); }
static inline vf fsubc(vf v,float c,size_t vl){ return __riscv_vfsub_vv_f32m1(v,f_splat(c,vl),vl); }
static inline vf f_clamp(vf v,float lo,float hi,size_t vl){
    return __riscv_vfmin_vv_f32m1(__riscv_vfmax_vv_f32m1(v,f_splat(lo,vl),vl),f_splat(hi,vl),vl); }

#define GRAV 9.81f
#define OFF_Y (-0.016f)
#define ATAU 0.02f
#define ZUPT 0.05f
#define VMAXV 5.0f
#define KP 0.5f
#define ZG 0.5f
#define VZG 0.3f
#ifndef LEAD
#define LEAD 1.0f
#endif
#ifndef LEADA
#define LEADA 0.5f
#endif

/* ================= estimator vector kernels ================= */

/* alpha_f LP (fp32 vector): deriv=(g-wp)*invdt; alpha_f += k*(deriv-alpha_f). (orig lines 139-142) */
void ve_af_lp(const float g_cur[3], const float w_prev[3], float alpha_f[3],
              uint32_t k_bits, uint32_t invdt_bits){
    size_t v3=3;
    vf g32=f_load(g_cur,v3), wp=f_load(w_prev,v3), af=f_load(alpha_f,v3);
    vf deriv=__riscv_vfmul_vv_f32m1(__riscv_vfsub_vv_f32m1(g32,wp,v3),f_splat_bits(invdt_bits,v3),v3);
    af=__riscv_vfmacc_vv_f32m1(af,f_splat_bits(k_bits,v3),__riscv_vfsub_vv_f32m1(deriv,af,v3),v3);
    f_store(alpha_f,af,v3);
}

/* |accel| (fp16) as a bit-pattern (orig lines 153,156-157) */
uint16_t ve_amag(const float acc_corr[3]){
    size_t v3=3;
    vh av=f2h(f_load(acc_corr,v3),v3);
    vh amagb=__riscv_vfsqrt_v_f16m1(h_dot_bcast(av,av,v3),v3);
    union{_Float16 h;uint16_t u;}x; h_st1(&x.h,amagb); return x.u;
}

/* Mahony body-rate w3 = g_cur (+ KP*gate * (an x vu) when do_trim). (orig lines 153-182) */
void ve_mahony_w3(const float acc_corr[3], const float q[4], const float g_cur[3],
                  uint32_t kpgate_bits, int do_trim, float w3_out[3]){
    size_t v3=3,v4=4;
    vf w3=f_load(g_cur,v3);                                    /* body rate (fp32) for quat */
    if (do_trim){
        vh av=f2h(f_load(acc_corr,v3),v3);
        vh amagb=__riscv_vfsqrt_v_f16m1(h_dot_bcast(av,av,v3),v3);
        vh inv=hrecip(amagb,v3);
        vh an=__riscv_vfmul_vv_f16m1(av,inv,v3);              /* normalized accel */
        vh qh=f2h(f_load(q,v4),v4);                          /* [qw,qx,qy,qz] */
        static const uint16_t iA[4]={1,2,1,0}, iB[4]={3,3,1,0};
        static const uint16_t iC[4]={0,0,2,0}, iD[4]={2,1,2,0};
        vh pA=__riscv_vfmul_vv_f16m1(h_shuf(qh,iA,v3),h_shuf(qh,iB,v3),v3); /* [qxqz,qyqz,qx^2] */
        vh pC=__riscv_vfmul_vv_f16m1(h_shuf(qh,iC,v3),h_shuf(qh,iD,v3),v3); /* [qwqy,qwqx,qy^2] */
        static const _Float16 sgn[4]={(_Float16)-1,(_Float16)1,(_Float16)1,0};
        static const _Float16 mul[4]={(_Float16)2,(_Float16)2,(_Float16)-2,0};
        static const _Float16 add[4]={(_Float16)0,(_Float16)0,(_Float16)1,0};
        vh t=__riscv_vfmacc_vv_f16m1(pA,pC,h_load(sgn,v3),v3);            /* pA + sgn*pC */
        vh vu=__riscv_vfmacc_vv_f16m1(h_load(add,v3),t,h_load(mul,v3),v3);/* t*mul + add */
        static const uint16_t c1[4]={1,2,0,0}, c2[4]={2,0,1,0};
        vh e=__riscv_vfsub_vv_f16m1(__riscv_vfmul_vv_f16m1(h_shuf(an,c1,v3),h_shuf(vu,c2,v3),v3),
                                    __riscv_vfmul_vv_f16m1(h_shuf(an,c2,v3),h_shuf(vu,c1,v3),v3),v3);
        w3=__riscv_vfmacc_vv_f32m1(w3,f_splat_bits(kpgate_bits,v3),h2f(e,v3),v3); /* w3 += KP*gate*e */
    }
    f_store(w3_out,w3,v3);
}

/* quaternion integrate + normalize (fp32 vector). step_bits = dt (or lead ha) as fp32 bits;
 * the 0.5 half-step is applied via a compile-time vector multiply. (orig lines 100-123) */
void ve_quat_integrate(float q[4], const float w3_arr[3], uint32_t step_bits){
    size_t v4=4,v3=3;
    vf w3=f_load(w3_arr,v3);
    vf vq=f_load(q,v4);
    static const uint32_t p0[4]={1,0,3,2}; static const float s0[4]={-1,1,1,-1};
    static const uint32_t p1[4]={2,3,0,1}; static const float s1[4]={-1,-1,1,1};
    static const uint32_t p2[4]={3,2,1,0}; static const float s2[4]={-1,1,-1,1};
    vf a0=__riscv_vfmul_vv_f32m1(f_shuf(vq,p0,v4),f_load(s0,v4),v4);
    vf a1=__riscv_vfmul_vv_f32m1(f_shuf(vq,p1,v4),f_load(s1,v4),v4);
    vf a2=__riscv_vfmul_vv_f32m1(f_shuf(vq,p2,v4),f_load(s2,v4),v4);
    vf dq=__riscv_vfmul_vv_f32m1(a0,__riscv_vrgather_vx_f32m1(w3,0,v4),v4);
    dq=__riscv_vfmacc_vv_f32m1(dq,a1,__riscv_vrgather_vx_f32m1(w3,1,v4),v4);
    dq=__riscv_vfmacc_vv_f32m1(dq,a2,__riscv_vrgather_vx_f32m1(w3,2,v4),v4);
    /* q += 0.5*step*dq : half = step*0.5 (single mul, == scalar 0.5*step) */
    vf half=__riscv_vfmul_vv_f32m1(f_splat_bits(step_bits,v4),f_splat(0.5f,v4),v4);
    vq=__riscv_vfmacc_vv_f32m1(vq,dq,half,v4);
    vf sq=__riscv_vfmul_vv_f32m1(vq,vq,v4);
    vf n2=__riscv_vfredusum_vs_f32m1_f32m1(sq,f_splat(0.0f,v4),v4);
    vf inv=__riscv_vfdiv_vv_f32m1(f_splat(1.0f,1),__riscv_vfsqrt_v_f32m1(n2,1),1);
    vf invb=__riscv_vrgather_vx_f32m1(inv,0,v4);
    vq=__riscv_vfmul_vv_f32m1(vq,invb,v4);
    f_store(q,vq,v4);
}

/* rotation matrix (fp16) + world accel (fp32) + body->world flow velocity (fp16).
 * (orig lines 185-224). R lane0 elements are stored via vse16(vl=1) into a stack array,
 * then the columns/2x2 block are gathered with indexed vector loads -> no scalar fp16. */
void ve_rot_wa_flow(const float q[4], const float acc_corr[3], const _Float16 flow_f16[2],
                    float aw_out[3], float vf01_out[2], uint16_t *r8_bits_out){
    size_t v3=3,v4=4,v2=2;
    vh q16=f2h(f_load(q,v4),v4);
    vh qx_=h_bcast_lane(q16,1,v4), qy_=h_bcast_lane(q16,2,v4), qz_=h_bcast_lane(q16,3,v4), qw_=h_bcast_lane(q16,0,v4);
    vh two=h_splat(2.0f,v4), one=h_splat(1.0f,v4);
    vh qxx=__riscv_vfmul_vv_f16m1(qx_,qx_,v4), qyy=__riscv_vfmul_vv_f16m1(qy_,qy_,v4), qzz=__riscv_vfmul_vv_f16m1(qz_,qz_,v4);
    _Float16 R[9];
    h_st1(&R[0],__riscv_vfnmsac_vv_f16m1(one,two,__riscv_vfadd_vv_f16m1(qyy,qzz,v4),v4));
    h_st1(&R[4],__riscv_vfnmsac_vv_f16m1(one,two,__riscv_vfadd_vv_f16m1(qxx,qzz,v4),v4));
    h_st1(&R[8],__riscv_vfnmsac_vv_f16m1(one,two,__riscv_vfadd_vv_f16m1(qxx,qyy,v4),v4));
    {
      vh xy=__riscv_vfmul_vv_f16m1(qx_,qy_,v4), wz=__riscv_vfmul_vv_f16m1(qw_,qz_,v4);
      vh xz=__riscv_vfmul_vv_f16m1(qx_,qz_,v4), wy=__riscv_vfmul_vv_f16m1(qw_,qy_,v4);
      vh yz=__riscv_vfmul_vv_f16m1(qy_,qz_,v4), wx=__riscv_vfmul_vv_f16m1(qw_,qx_,v4);
      h_st1(&R[1],__riscv_vfmul_vv_f16m1(two,__riscv_vfsub_vv_f16m1(xy,wz,v4),v4));
      h_st1(&R[2],__riscv_vfmul_vv_f16m1(two,__riscv_vfadd_vv_f16m1(xz,wy,v4),v4));
      h_st1(&R[3],__riscv_vfmul_vv_f16m1(two,__riscv_vfadd_vv_f16m1(xy,wz,v4),v4));
      h_st1(&R[5],__riscv_vfmul_vv_f16m1(two,__riscv_vfsub_vv_f16m1(yz,wx,v4),v4));
      h_st1(&R[6],__riscv_vfmul_vv_f16m1(two,__riscv_vfsub_vv_f16m1(xz,wy,v4),v4));
      h_st1(&R[7],__riscv_vfmul_vv_f16m1(two,__riscv_vfadd_vv_f16m1(yz,wx,v4),v4));
    }
    { union{_Float16 h;uint16_t u;}x; x.h=R[8]; *r8_bits_out=x.u; }   /* fp16 load(lh)+store, integer -- no scalar fp */

    /* world accel = R*a (columns via indexed load) - gravity, integrate large-large in fp32 */
    vh av=f2h(f_load(acc_corr,v3),v3);
    static const uint16_t iCol0[4]={0,6,12,0}, iCol1[4]={2,8,14,0}, iCol2[4]={4,10,16,0}; /* byte offsets of R cols */
    vh Rc0=__riscv_vluxei16_v_f16m1(R,__riscv_vle16_v_u16m1(iCol0,v3),v3);
    vh Rc1=__riscv_vluxei16_v_f16m1(R,__riscv_vle16_v_u16m1(iCol1,v3),v3);
    vh Rc2=__riscv_vluxei16_v_f16m1(R,__riscv_vle16_v_u16m1(iCol2,v3),v3);
    vh awv=__riscv_vfmul_vv_f16m1(Rc0,h_bcast_lane(av,0,v3),v3);
    awv=__riscv_vfmacc_vv_f16m1(awv,Rc1,h_bcast_lane(av,1,v3),v3);
    awv=__riscv_vfmacc_vv_f16m1(awv,Rc2,h_bcast_lane(av,2,v3),v3);
    static const float gsub[4]={0.0f,0.0f,GRAV,0.0f};
    vf aw32=__riscv_vfsub_vv_f32m1(h2f(awv,v3),f_load(gsub,v3),v3);
    f_store(aw_out,aw32,v3);

    /* body->world flow velocity: [dot([R0,R1],flow), dot([R3,R4],flow)] (fp16) -> fp32.
     * f0=[R0,R1] bytes [0,2]; f1=[R3,R4] bytes [6,8]. Assemble [d0,d1] via slideup (no scalar). */
    static const uint16_t if0[4]={0,2,0,0}, if1[4]={6,8,0,0};
    vh fvv=h_load(flow_f16,v2);
    vh f0=__riscv_vluxei16_v_f16m1(R,__riscv_vle16_v_u16m1(if0,v2),v2);
    vh f1=__riscv_vluxei16_v_f16m1(R,__riscv_vle16_v_u16m1(if1,v2),v2);
    vh d0=h_dot_bcast(f0,fvv,v2), d1=h_dot_bcast(f1,fvv,v2);
    vh vf01=__riscv_vslideup_vx_f16m1(d0,d1,1,v2);              /* [d0, d1] */
    f_store(vf01_out,h2f(vf01,v2),v2);
}

/* vvel = vel + aw*dt (fp32 vector FMA). (orig line 216) */
void ve_vel_predict(const float vel[3], const float aw[3], uint32_t dt_bits, float vvel_out[3]){
    size_t v3=3;
    f_store(vvel_out,__riscv_vfmacc_vv_f32m1(f_load(vel,v3),f_load(aw,v3),f_splat_bits(dt_bits,v3),v3),v3);
}

/* horizontal-velocity backstop clamp (fp32 vector, +-VMAXV). (orig line 230) */
void ve_clampV(float v[3]){ size_t v3=3; f_store(v,f_clamp(f_load(v,v3),-VMAXV,VMAXV,v3),v3); }

/* pos += vvel*dt (fp32 vector FMA). (orig line 233) */
void ve_pos_integrate(float pos[3], const float vvel[3], uint32_t dt_bits){
    size_t v3=3;
    f_store(pos,__riscv_vfmacc_vv_f32m1(f_load(pos,v3),f_load(vvel,v3),f_splat_bits(dt_bits,v3),v3),v3);
}

/* get_state lead (fp32 vector FMA): so_pos=pos+vel*hl ; so_vel=vel+aw*hl. (orig lines 244,246) */
void ve_lead_out(const float pos[3], const float vel[3], const float aw[3],
                 uint32_t hl_bits, float so_pos[3], float so_vel[3]){
    size_t v3=3;
    vf vhl=f_splat_bits(hl_bits,v3);
    vf vel_v=f_load(vel,v3);
    f_store(so_pos,__riscv_vfmacc_vv_f32m1(f_load(pos,v3),vel_v,vhl,v3),v3);
    f_store(so_vel,__riscv_vfmacc_vv_f32m1(vel_v,f_load(aw,v3),vhl,v3),v3);
}

/* ================= controller vector kernels ================= */
#define NF 2.0f
#define VTC 0.5f
#define VIM 2.0f
#define VTM 0.26f
#define TR 0.10f
#define TY 0.25f
#define TRR 0.025f
#define TYR 0.05f
#define J0 16e-6f
#define J1 16e-6f
#define J2 29e-6f
#define LARM 33e-3f
#define KDRAG 0.01f
#define TA 26.919633f
#define TB 35.754861f
#define PROP 1.33f
#define GPN 101.9368f
#define MOT 4.0f
#ifndef FCLAMP
#define FCLAMP 600.0f
#endif

/* cos via fp16 polynomial (vector): 1 - x^2/2 + x^4/24 for the two angles. (orig lines 283-290) */
void vc_cos2(const _Float16 rp_f16[2], float cr_cp_out[2]){
    size_t v2=2;
    vh x=h_load(rp_f16,v2); vh x2=__riscv_vfmul_vv_f16m1(x,x,v2);
    vh x4=__riscv_vfmul_vv_f16m1(x2,x2,v2);
    vh c=__riscv_vfadd_vv_f16m1(__riscv_vfsub_vv_f16m1(h_splat(1.0f,v2),hmulc(x2,0.5f,v2),v2),
                                hmulc(x4,1.0f/24.0f,v2),v2);
    f_store(cr_cp_out,h2f(c,v2),v2);
}

/* horizontal velocity loop (fp32 vector, length-2) + clamp + slew -> desRP. (orig lines 318-333) */
void vc_velloop(const float desV[2], const float vv[2], float vel_int[2],
                uint32_t kivdt_bits, int grounded, const float prev[2],
                uint32_t dmax_bits, float desRP_out[2]){
    size_t n2=2;
    vf e=__riscv_vfsub_vv_f32m1(f_load(desV,n2),f_load(vv,n2),n2);            /* desVel - estVel */
    vf viv = grounded ? f_splat(0.0f,n2)
                      : f_clamp(__riscv_vfmacc_vv_f32m1(f_load(vel_int,n2),f_splat_bits(kivdt_bits,n2),e,n2),-VIM,VIM,n2);
    f_store(vel_int,viv,n2);
    vf desAcc=__riscv_vfmacc_vv_f32m1(viv,f_splat(1.0f/VTC,n2),e,n2);          /* (1/VTC)*e + vel_int */
    static const uint32_t sw[4]={1,0,0,0}; static const float sc[2]={-1.0f/GRAV,1.0f/GRAV};
    vf desRP=f_clamp(__riscv_vfmul_vv_f32m1(f_shuf(desAcc,sw,n2),f_load(sc,n2),n2),-VTM,VTM,n2); /* [-dA2,dA1]/g */
    vf pv=f_load(prev,n2);
    vf dmax=f_splat_bits(dmax_bits,n2);
    desRP=__riscv_vfmin_vv_f32m1(__riscv_vfmax_vv_f32m1(desRP,__riscv_vfsub_vv_f32m1(pv,dmax,n2),n2),
                                 __riscv_vfadd_vv_f32m1(pv,dmax,n2),n2);       /* slew */
    f_store(desRP_out,desRP,n2);
}

/* mixer-output ctrl [c0..c3] (fp16) -> u_out[4] (perm + scale + force->duty incl vfsqrt + clamps).
 * (orig f2v4, lines 293-304) */
static void f2v4(vh ctrl, float u_out[4]){
    size_t v4=4;
    static const uint16_t perm[4]={1,2,3,0};
    static const _Float16 scl[4]={(_Float16)0.9f,(_Float16)0.9f,(_Float16)(0.9f*0.87f),(_Float16)(0.9f*0.87f)};
    vh in=__riscv_vfmul_vv_f16m1(h_shuf(ctrl,perm,v4),h_load(scl,v4),v4);
    in=__riscv_vfmax_vv_f16m1(in,h_splat(0.0f,v4),v4);                 /* force<0 -> 0 */
    vh t=hminc(hmulc(in,GPN*MOT/PROP,v4),FCLAMP,v4);
    vh disc=hminc(haddc(hmulc(t,4.0f*TA,v4),TB*TB,v4),60000.0f,v4);    /* fp16 overflow guard */
    vh d=hmulc(hsubc(__riscv_vfsqrt_v_f16m1(disc,v4),TB,v4),1.0f/(2.0f*TA),v4);
    d=__riscv_vfmax_vv_f16m1(__riscv_vfmin_vv_f16m1(d,h_splat(1.0f,v4),v4),h_splat(0.0f,v4),v4); /* duty in [0,1] */
    f_store(u_out,h2f(hsubc(d,0.583f,v4),v4),v4);                      /* -0.583 -> normalized-thrust */
}

/* attitude + rate loops (3-lane fp16), 4x4 mixer, force->duty. (orig lines 336-361) */
void vc_attitude(const float state[12], const float tgt3_f32[3], uint16_t u0_f16bits, float u_out[4]){
    size_t v3n=3, v4=4;
    vh est3=f2h(f_load(state+3,v3n),v3n);             /* [estRoll,estPitch,estYaw] */
    vh tgt3=f2h(f_load(tgt3_f32,v3n),v3n);            /* [desRoll,desPitch,yawTgt] */
    static const _Float16 tauA[4]={(_Float16)(-1.0f/TR),(_Float16)(-1.0f/TR),(_Float16)(-1.0f/TY),0};
    static const _Float16 tauB[4]={(_Float16)(-1.0f/TRR),(_Float16)(-1.0f/TRR),(_Float16)(-1.0f/TYR),0};
    vh ratetgt=__riscv_vfmul_vv_f16m1(__riscv_vfsub_vv_f16m1(est3,tgt3,v3n),h_load(tauA,v3n),v3n);
    vh g3=f2h(f_load(state+9,v3n),v3n);
    vh cmd=__riscv_vfmul_vv_f16m1(__riscv_vfsub_vv_f16m1(g3,ratetgt,v3n),h_load(tauB,v3n),v3n);
    /* u = [desNorm*MASS, rc*J0, pc*J1, yc*J2]; prepend u0 to (cmd*J) via slideup (no scalar fp16) */
    static const _Float16 Jv[4]={(_Float16)J0,(_Float16)J1,(_Float16)J2,0};
    vh utv=__riscv_vfmul_vv_f16m1(cmd,h_load(Jv,v3n),v3n);
    vh u=__riscv_vslideup_vx_f16m1(h_splat_bits(u0_f16bits,v4),utv,1,v4);   /* [u0, rc*J0, pc*J1, yc*J2] */
    static const _Float16 Mc0[4]={(_Float16)0.25f,(_Float16)0.25f,(_Float16)0.25f,(_Float16)0.25f};
    static const _Float16 Mc1[4]={(_Float16)(0.25f/LARM),(_Float16)(-0.25f/LARM),(_Float16)(-0.25f/LARM),(_Float16)(0.25f/LARM)};
    static const _Float16 Mc2[4]={(_Float16)(-0.25f/LARM),(_Float16)(-0.25f/LARM),(_Float16)(0.25f/LARM),(_Float16)(0.25f/LARM)};
    static const _Float16 Mc3[4]={(_Float16)(0.25f/KDRAG),(_Float16)(-0.25f/KDRAG),(_Float16)(0.25f/KDRAG),(_Float16)(-0.25f/KDRAG)};
    vh ctrl=__riscv_vfmul_vv_f16m1(h_load(Mc0,v4),h_bcast_lane(u,0,v4),v4);
    ctrl=__riscv_vfmacc_vv_f16m1(ctrl,h_load(Mc1,v4),h_bcast_lane(u,1,v4),v4);
    ctrl=__riscv_vfmacc_vv_f16m1(ctrl,h_load(Mc2,v4),h_bcast_lane(u,2,v4),v4);
    ctrl=__riscv_vfmacc_vv_f16m1(ctrl,h_load(Mc3,v4),h_bcast_lane(u,3,v4),v4);
    f2v4(ctrl,u_out);
}
