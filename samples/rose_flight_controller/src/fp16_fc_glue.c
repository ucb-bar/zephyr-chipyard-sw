/*
 * fp16 flight-controller kernels -- SOFT-FLOAT GLUE / ORCHESTRATOR unit.
 *
 * Built with the global rv64imac soft-float march (NO zve/zvfh/f), so all scalar
 * float here becomes soft-float libcalls -- never a scalar-FP instruction that would
 * trap on the misa.F=0 At35 core. Holds kfc_init/kfc_estimate/kfc_control: the float
 * interface, the scalar-float cascade, all int64 fixed-point accumulator bookkeeping,
 * comparisons/branches, and the runtime fp16 constants (passed to the VEC unit as
 * uint16 bit patterns via hb()). The fp16-vector algebra + int<->fp16 vfcvt live in
 * the VEC unit (fp16_fc_kernels.c, Zvfh), called via ve_/vc_ over memory arrays.
 *
 * Orchestration + all scalar math are VERBATIM from a1198297's single-file kernel
 * @ ddcd5d3; only the vector blocks were factored out to the vec helpers.
 */
#include "fp16_fc_kernels.h"
#include "fp16_fc_vec.h"
#include <math.h>
#include <stdint.h>

/* Q-scales / increment scales / constants -- verbatim from the source. */
#define Q_QUAT 30
#define Q_VEL  28
#define Q_POS  27
#define Q_ALTI 29
#define Q_VELI 30
#define Q_RATE 25
#define Q_TILT 31
#define M_QUAT 18
#define M_ALPHA 7
#define M_ALTI 22
#define M_VELI 21

#define GRAV 9.81f
#define OFF_Y (-0.016f)
#define ATAU 0.02f
#define ZUPT 0.05f
#define VMAXV 5.0f
#define ZG 0.5f
#define VZG 0.3f
#ifndef LEAD
#define LEAD 1.0f
#endif
#ifndef LEADA
#define LEADA 0.5f
#endif

void kfc_init(kfc_state *s, float x0, float y0, float z0){
    for(int i=0;i<4;i++) s->q[i]=0; s->q[0]=(int64_t)1<<Q_QUAT;
    for(int i=0;i<3;i++){ s->vel[i]=0; s->alpha[i]=0; s->wprev[i]=0; s->g_cur[i]=0; s->aw[i]=0; }
    s->pos[0]=(int64_t)llroundf(x0*(float)((int64_t)1<<Q_POS));
    s->pos[1]=(int64_t)llroundf(y0*(float)((int64_t)1<<Q_POS));
    s->pos[2]=(int64_t)llroundf(z0*(float)((int64_t)1<<Q_POS));
    s->alt_int=0; s->vel_int[0]=s->vel_int[1]=0; s->desprev[0]=s->desprev[1]=0;
    s->dt_last=0; s->have_wprev=0;
}

/* fp16 quaternion integrate by body rate w3 into int64 q (Q30), then renormalize.
 * Scalar/int part here; the fp16 dq computation + rsqrt inverse are in the vec unit. */
static void quat_step_glue(int64_t q[4], const int16_t q_i16[4], const _Float16 w3[4], float dt){
    int16_t dq_i16[8];
    ve_quat_dq_i16(dq_i16, q_i16, w3, hb(0.5f*dt), 4);
    for(int i=0;i<4;i++) q[i]+=(int64_t)((int32_t)dq_i16[i]<<(Q_QUAT-M_QUAT));
    int64_t n2=0; for(int i=0;i<4;i++) n2+=q[i]*q[i];
    int16_t n16=(int16_t)(n2>>(2*Q_QUAT-14));
    int16_t invi; ve_quat_inv_i16(&invi,&n16);
    for(int i=0;i<4;i++) q[i]=(int64_t)((q[i]*(int64_t)invi)>>14);
}

void kfc_estimate(kfc_state *s, const float accel[3], const float gyro[3],
                  const float flow[2], int flow_valid, float height, int tof_valid,
                  float dt, float state_out[KEST_NSTATES]){
    s->g_cur[0]=gyro[0]; s->g_cur[1]=gyro[1]; s->g_cur[2]=gyro[2];
    _Float16 gbf[4], abf[4];
    for(int i=0;i<3;i++){ gbf[i]=(_Float16)gyro[i]; abf[i]=(_Float16)accel[i]; } gbf[3]=abf[3]=0;

    /* ---- lever-arm: alpha LP int Q25, accel correction fp16 ---- */
    if (dt>1e-6f){
        if (s->have_wprev){
            float k=dt/(ATAU+dt);
            int16_t wprev_i16[4], alpha_i16[4];
            for(int i=0;i<3;i++){ wprev_i16[i]=(int16_t)(s->wprev[i]>>(Q_RATE-10));
                                  alpha_i16[i]=(int16_t)(s->alpha[i]>>(Q_RATE-10)); }
            wprev_i16[3]=0; alpha_i16[3]=0;
            int16_t inc_i16[8];
            ve_alpha_inc(inc_i16, gbf, wprev_i16, alpha_i16, hb(1.0f/dt), hb(k), 3);
            for(int i=0;i<3;i++) s->alpha[i]+=(int32_t)inc_i16[i]<<(Q_RATE-M_ALPHA);
        }
        for(int i=0;i<3;i++) s->wprev[i]=(int64_t)llroundf((float)gbf[i]*(float)((int64_t)1<<Q_RATE));
        s->have_wprev=1;
        int16_t alpha_i16[4]; for(int i=0;i<3;i++) alpha_i16[i]=(int16_t)(s->alpha[i]>>(Q_RATE-10)); alpha_i16[3]=0;
        _Float16 prs[4], als[4];
        ve_lever_pr(prs, als, gbf, alpha_i16);
        _Float16 cc[4];
        cc[0]=(_Float16)(OFF_Y*(-(float)als[2]+(float)prs[0]));
        cc[1]=(_Float16)(OFF_Y*(-((float)prs[3]+(float)prs[2])));
        cc[2]=(_Float16)(OFF_Y*((float)als[0]+(float)prs[1])); cc[3]=0;
        ve_av_sub(abf, cc, 3);
    }

    /* ---- Mahony gravity-trim + quaternion integrate ---- */
    int16_t q_i16[4]; for(int i=0;i<4;i++) q_i16[i]=(int16_t)(s->q[i]>>(Q_QUAT-14));
    _Float16 w3[4]; ve_mahony(w3, abf, q_i16, gbf, 3); w3[3]=0;
    quat_step_glue(s->q, q_i16, w3, dt);

    /* ---- rotation matrix (fp16) ---- */
    int16_t q_i16b[4]; for(int i=0;i<4;i++) q_i16b[i]=(int16_t)(s->q[i]>>(Q_QUAT-14));
    _Float16 R[9]; ve_rotmat(R, q_i16b);

    int tof_vert = tof_valid && ((float)R[8] > 0.05f);
    float h_vert = height*(float)R[8];

    /* ---- gravity chain INTEGER: R(Q14) x a(Q10) -> Q24, -g, integrate vel/pos ---- */
    int16_t Ri[9]; ve_h_scale_to_i16(Ri, R, 14, 9);
    int16_t ai[4]; ve_h_scale_to_i16(ai, abf, 10, 3);
    const int64_t gQ24=(int64_t)llroundf(GRAV*(float)((int64_t)1<<24));
    int64_t axw=(int64_t)Ri[0]*ai[0]+(int64_t)Ri[1]*ai[1]+(int64_t)Ri[2]*ai[2];
    int64_t ayw=(int64_t)Ri[3]*ai[0]+(int64_t)Ri[4]*ai[1]+(int64_t)Ri[5]*ai[2];
    int64_t azw=(int64_t)Ri[6]*ai[0]+(int64_t)Ri[7]*ai[1]+(int64_t)Ri[8]*ai[2]-gQ24;
    s->aw[0]=(float)axw/(float)((int64_t)1<<24); s->aw[1]=(float)ayw/(float)((int64_t)1<<24);
    s->aw[2]=(float)azw/(float)((int64_t)1<<24); s->dt_last=dt;
    int64_t dtQ30=(int64_t)llroundf(dt*(float)((int64_t)1<<30));
    s->vel[0]+=(int64_t)(((__int128)axw*dtQ30)>>26);
    s->vel[1]+=(int64_t)(((__int128)ayw*dtQ30)>>26);
    s->vel[2]+=(int64_t)(((__int128)azw*dtQ30)>>26);

    if (flow_valid){
        _Float16 fv[4]={(_Float16)flow[0],(_Float16)flow[1],0,0};
        _Float16 rf0[2]={R[0],R[1]}, rf1[2]={R[3],R[4]};
        int16_t vfx16, vfy16;
        ve_rflow_i16(&vfx16, rf0, fv);
        ve_rflow_i16(&vfy16, rf1, fv);
        s->vel[0]=(int64_t)vfx16<<16; s->vel[1]=(int64_t)vfy16<<16;
    } else if (tof_vert && h_vert<ZUPT){ s->vel[0]=0; s->vel[1]=0; }
    const int64_t vmaxQ=(int64_t)VMAXV<<Q_VEL;
    if(s->vel[0]> vmaxQ)s->vel[0]= vmaxQ; else if(s->vel[0]<-vmaxQ)s->vel[0]=-vmaxQ;
    if(s->vel[1]> vmaxQ)s->vel[1]= vmaxQ; else if(s->vel[1]<-vmaxQ)s->vel[1]=-vmaxQ;

    s->pos[0]+=(int64_t)(((__int128)s->vel[0]*dtQ30)>>31);
    s->pos[1]+=(int64_t)(((__int128)s->vel[1]*dtQ30)>>31);
    s->pos[2]+=(int64_t)(((__int128)s->vel[2]*dtQ30)>>31);

    if (tof_vert){
        float z=(float)s->pos[2]/(float)((int64_t)1<<Q_POS); float rz=h_vert-z;
        s->pos[2]+=(int64_t)llroundf(ZG*rz*(float)((int64_t)1<<Q_POS));
        s->vel[2]+=(int64_t)llroundf(VZG*rz*(float)((int64_t)1<<Q_VEL));
    }

    /* ---- get_state lead + rodrigues (float boundary) ---- */
    float hl=LEAD*s->dt_last, ha=LEADA*s->dt_last;
    int64_t qq[4]={s->q[0],s->q[1],s->q[2],s->q[3]};
    int16_t qq_i16[4]; for(int i=0;i<4;i++) qq_i16[i]=(int16_t)(qq[i]>>(Q_QUAT-14));
    quat_step_glue(qq, qq_i16, gbf, ha);
    float qsc=(float)((int64_t)1<<Q_QUAT);
    float pw=(float)qq[0]/qsc, px=(float)qq[1]/qsc, py=(float)qq[2]/qsc, pz=(float)qq[3]/qsc;
    float qwv=(fabsf(pw)<1e-9f)?(pw>=0?1e-9f:-1e-9f):pw;
    float X=(float)s->pos[0]/(float)((int64_t)1<<Q_POS),Y=(float)s->pos[1]/(float)((int64_t)1<<Q_POS),Z=(float)s->pos[2]/(float)((int64_t)1<<Q_POS);
    float VX=(float)s->vel[0]/(float)((int64_t)1<<Q_VEL),VY=(float)s->vel[1]/(float)((int64_t)1<<Q_VEL),VZ=(float)s->vel[2]/(float)((int64_t)1<<Q_VEL);
    state_out[0]=X+VX*hl; state_out[1]=Y+VY*hl; state_out[2]=Z+VZ*hl;
    state_out[3]=px/qwv; state_out[4]=py/qwv; state_out[5]=pz/qwv;
    state_out[6]=VX+s->aw[0]*hl; state_out[7]=VY+s->aw[1]*hl; state_out[8]=VZ+s->aw[2]*hl;
    state_out[9]=s->g_cur[0]; state_out[10]=s->g_cur[1]; state_out[11]=s->g_cur[2];
}

/* ================= PID (fp16 algebra; alt_int/vel_int/tilt int fixed-point) ================= */
#define NF 2.0f
#define DH 0.7f
#define KIH 1.5f
#define AIM 3.0f
#define AGND 0.05f
#define KIV 0.8f
#define VIM 2.0f
#define VTC 0.5f
#define VTM 0.26f
#define SLEW 1.0f
#define MASSK 0.060f
static inline float clampf(float x,float lo,float hi){ return x<lo?lo:(x>hi?hi:x); }

void kfc_control(kfc_state *s, const float state[KEST_NSTATES],
                 const float setpoint[KEST_NSTATES], float u_out[KCTRL_NACTIONS], float dt){
    const float estRoll=state[3],estPitch=state[4],estYaw=state[5];
    const float estVel_1=state[6],estVel_2=state[7],estVel_3=state[8];
    const float gyroX=state[9],gyroY=state[10],gyroZ=state[11];
    const float estHeight=state[2];
    const float desHeight=setpoint[2]; float desVel_1=setpoint[6],desVel_2=setpoint[7]; const float yaw_tgt=setpoint[5];
    int grounded=(estHeight<AGND);

    /* altitude integrator: int Q29 (fp16 increment) */
    if(grounded) s->alt_int=0;
    else{ uint16_t a1[1]={hb(desHeight-estHeight)}, b1[1]={hb(KIH*dt)};
          int16_t iq[8]; vc_inc(iq, a1, b1, M_ALTI, 1); s->alt_int+=(int32_t)iq[0]<<(Q_ALTI-M_ALTI);
          int64_t aim=(int64_t)llroundf(AIM*(float)((int64_t)1<<Q_ALTI));
          if(s->alt_int>aim)s->alt_int=aim; else if(s->alt_int<-aim)s->alt_int=-aim; }
    float alt_i=(float)s->alt_int/(float)((int64_t)1<<Q_ALTI);

    /* desAcc3 + desNorm (fp16 cos) */
    _Float16 crcp[2]; vc_cos2(crcp, hb(estRoll), hb(estPitch));
    float cr=(float)crcp[0], cp=(float)crcp[1];
    float desAcc3=-2.0f*DH*NF*estVel_3 - NF*NF*(estHeight-desHeight) + alt_i;
    float desNorm=(GRAV+desAcc3)/((float)cr*(float)cp);

    /* velocity loop: int Q30 integrators, fp16 tilt */
    float verr1=desVel_1-estVel_1, verr2=desVel_2-estVel_2;
    if(grounded){ s->vel_int[0]=0; s->vel_int[1]=0; }
    else{ uint16_t a2[2]={hb(verr1),hb(verr2)}, b2[1]={hb(KIV*dt)};
          int16_t iq[8]; vc_inc(iq, a2, b2, M_VELI, 2);
          s->vel_int[0]+=(int32_t)iq[0]<<(Q_VELI-M_VELI); s->vel_int[1]+=(int32_t)iq[1]<<(Q_VELI-M_VELI);
          int64_t vim=(int64_t)llroundf(VIM*(float)((int64_t)1<<Q_VELI));
          for(int i=0;i<2;i++){ if(s->vel_int[i]>vim)s->vel_int[i]=vim; else if(s->vel_int[i]<-vim)s->vel_int[i]=-vim; } }
    float vi1=(float)s->vel_int[0]/(float)((int64_t)1<<Q_VELI), vi2=(float)s->vel_int[1]/(float)((int64_t)1<<Q_VELI);
    float desAcc1=(1.0f/VTC)*verr1+vi1, desAcc2=(1.0f/VTC)*verr2+vi2;
    float desRoll=clampf(-desAcc2/GRAV,-VTM,VTM), desPitch=clampf(desAcc1/GRAV,-VTM,VTM);
    { const float dmax=SLEW*dt;
      if(grounded){ s->desprev[0]=0; s->desprev[1]=0; }
      float pr=(float)s->desprev[0]/(float)((int64_t)1<<Q_TILT), pp=(float)s->desprev[1]/(float)((int64_t)1<<Q_TILT);
      desRoll=clampf(desRoll,pr-dmax,pr+dmax); desPitch=clampf(desPitch,pp-dmax,pp+dmax);
      s->desprev[0]=(int64_t)llroundf(desRoll*(float)((int64_t)1<<Q_TILT));
      s->desprev[1]=(int64_t)llroundf(desPitch*(float)((int64_t)1<<Q_TILT)); }

    /* attitude + rate loops (fp16 3-lane) */
    _Float16 est3[4]={(_Float16)estRoll,(_Float16)estPitch,(_Float16)estYaw,0};
    _Float16 tgt3[4]={(_Float16)desRoll,(_Float16)desPitch,(_Float16)yaw_tgt,0};
    _Float16 g3[4]={(_Float16)gyroX,(_Float16)gyroY,(_Float16)gyroZ,0};
    _Float16 cm[4]; vc_attitude(cm, est3, tgt3, g3); cm[3]=0;

    /* mixing (fp16 4x4) + force->duty */
    _Float16 uo[4]; vc_mixer_force(uo, cm, hb(desNorm*MASSK));
    for(int i=0;i<4;i++) u_out[i]=(float)uo[i];   /* fp16 -> float (interface boundary) */
}
