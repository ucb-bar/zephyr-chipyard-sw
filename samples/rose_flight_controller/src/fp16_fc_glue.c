/*
 * fp16 flight-controller kernels — SOFT-FLOAT ORCHESTRATOR unit.
 *
 * Built -march=rv64imac -mabi=lp64 (NO f / zve / zvfh) so every scalar `float` op becomes
 * a soft-float libcall (__addsf3/__mulsf3/__divsf3/__extendhfsf2/__truncsfhf2 ...) that
 * runs on the integer core -- it NEVER emits a hardware scalar-FP instruction, so it can
 * never trap on the misa.F=0 Saturn core. It owns kfc_init and the kfc_estimate/kfc_control
 * orchestration: all control flow, comparisons, fp32 accumulator glue, the fp16<->fp32
 * boundary conversions, and the runtime scalar constants. The vector-heavy algebra runs on
 * the Saturn vector unit via the Zvfh kernels in fp16_fc_kernels.c (see fp16_fc_vec.h).
 *
 * The scalar expressions here are relocated VERBATIM from the original single-file kernel,
 * so the values are identical (soft-float is IEEE per-op like the hardware). The explicit
 * vector-FMA accumulators (quaternion integrate, alpha_f LP, velocity/position predict,
 * lead, velocity loop) stay in the vector unit precisely so their single-rounding FMA
 * semantics are preserved bit-for-bit.
 */
#include "fp16_fc_kernels.h"
#include "fp16_fc_vec.h"
#include <math.h>
#include <stdint.h>

/* bit-pattern reinterprets (soft-float unit: these are plain moves/loads, no hardware FP) */
static inline uint32_t fbits(float c){ union{float f;uint32_t u;}x; x.f=c; return x.u; }
static inline uint16_t hbits(float c){ union{_Float16 h;uint16_t u;}x; x.h=(_Float16)c; return x.u; }
static inline float    fromh(uint16_t b){ union{_Float16 h;uint16_t u;}x; x.u=b; return (float)x.h; }

#define GRAV 9.81f
#define OFF_Y (-0.016f)
#define ATAU 0.02f
#define ZUPT 0.05f
#define KP 0.5f
#define ZG 0.5f
#define VZG 0.3f
#ifndef LEAD
#define LEAD 1.0f
#endif
#ifndef LEADA
#define LEADA 0.5f
#endif

void kfc_init(kfc_state *s,float x0,float y0,float z0){
    s->q[0]=1;s->q[1]=s->q[2]=s->q[3]=0;
    s->vel[0]=s->vel[1]=s->vel[2]=0; s->pos[0]=x0;s->pos[1]=y0;s->pos[2]=z0;
    for(int i=0;i<3;i++){s->alpha_f[i]=0;s->w_prev[i]=0;s->g_cur[i]=0;s->aw[i]=0;}
    s->alt_int=s->vel_int_1=s->vel_int_2=0;s->desRoll_prev=s->desPitch_prev=0;
    s->dt_last=0;s->have_wprev=0;
}

void kfc_estimate(kfc_state *s,const float accel[3],const float gyro[3],
                  const float flow[2],int flow_valid,float height,int tof_valid,
                  float dt,float state_out[KEST_NSTATES]){
    s->g_cur[0]=gyro[0];s->g_cur[1]=gyro[1];s->g_cur[2]=gyro[2];

    /* ---- lever-arm compensation: alpha_f LP (fp32 vector) + small accel correction ---- */
    float acc_corr[3]={accel[0],accel[1],accel[2]};
    if (dt>1e-6f){
        if (s->have_wprev){
            float k=dt/(ATAU+dt);
            float invdt=1.0f/dt;
            ve_af_lp(s->g_cur,s->w_prev,s->alpha_f,fbits(k),fbits(invdt));  /* alpha_f += k*(deriv-alpha_f) */
        }
        s->w_prev[0]=s->g_cur[0];s->w_prev[1]=s->g_cur[1];s->w_prev[2]=s->g_cur[2];
        s->have_wprev=1;
        /* products [gx*gy, gy*gz, gz^2, gx^2] (matches the fp32 gathered-mul in the original) */
        float pr0=s->g_cur[0]*s->g_cur[1], pr1=s->g_cur[1]*s->g_cur[2];
        float pr2=s->g_cur[2]*s->g_cur[2], pr3=s->g_cur[0]*s->g_cur[0];
        float alx=s->alpha_f[0], alz=s->alpha_f[2];
        acc_corr[0]-=OFF_Y*(-alz+pr0);
        acc_corr[1]-=OFF_Y*(-(pr2+pr3));
        acc_corr[2]-=OFF_Y*(alx+pr1);
    }

    /* ---- Mahony gravity trim (fp16 vector), quaternion integration+norm (fp32 vector) ---- */
    float amag=fromh(ve_amag(acc_corr));
    int do_trim=0; uint32_t kpgate_bits=0;
    if (amag>1e-3f){
        float gate=1.0f-fabsf(amag-9.81f)/(0.5f*9.81f);
        if (gate>0.0f){ do_trim=1; kpgate_bits=fbits(KP*gate); }
    }
    float w3[3];
    ve_mahony_w3(acc_corr,s->q,s->g_cur,kpgate_bits,do_trim,w3);
    ve_quat_integrate(s->q,w3,fbits(dt));

    /* ---- rotation matrix + world accel + flow velocity (fp16/fp32 vector) ---- */
    _Float16 flow_f16[2]={(_Float16)flow[0],(_Float16)flow[1]};
    float vf01[2]; uint16_t r8_bits;
    ve_rot_wa_flow(s->q,acc_corr,flow_f16,s->aw,vf01,&r8_bits);
    s->dt_last=dt;
    float ctilt=fromh(r8_bits);
    int tof_vert = tof_valid && ctilt>0.05f;
    float h_vert=height*ctilt;

    /* ---- velocity predict + flow/ZUPT overwrite + clamp + position integrate (fp32 vector) ---- */
    float vvel[3];
    ve_vel_predict(s->vel,s->aw,fbits(dt),vvel);
    if (flow_valid){
        vvel[0]=vf01[0]; vvel[1]=vf01[1];          /* flow replaces horizontal velocity */
    } else if (tof_vert && h_vert<ZUPT){
        vvel[0]=0.0f; vvel[1]=0.0f;
    }
    ve_clampV(vvel);
    s->vel[0]=vvel[0]; s->vel[1]=vvel[1]; s->vel[2]=vvel[2];
    ve_pos_integrate(s->pos,vvel,fbits(dt));
    if (tof_vert){ float rz=h_vert-s->pos[2]; s->pos[2]+=ZG*rz; s->vel[2]+=VZG*rz; }

    /* ---- get_state lead (fp32 vector) ---- */
    float hl=LEAD*s->dt_last, ha=LEADA*s->dt_last;
    float qq[4]={s->q[0],s->q[1],s->q[2],s->q[3]};
    ve_quat_integrate(qq,s->g_cur,fbits(ha));      /* lead: predict quaternion by ha */
    float pw=qq[0]; float qwv=(fabsf(pw)<1e-9f)?(pw>=0?1e-9f:-1e-9f):pw;
    float inv_qwv=1.0f/qwv;
    float rvec[3]={ qq[1]*inv_qwv, qq[2]*inv_qwv, qq[3]*inv_qwv };  /* [px,py,pz]/qwv */
    float so_pos[3], so_vel[3];
    ve_lead_out(s->pos,s->vel,s->aw,fbits(hl),so_pos,so_vel);
    state_out[0]=so_pos[0];state_out[1]=so_pos[1];state_out[2]=so_pos[2];
    state_out[3]=rvec[0];state_out[4]=rvec[1];state_out[5]=rvec[2];
    state_out[6]=so_vel[0];state_out[7]=so_vel[1];state_out[8]=so_vel[2];
    state_out[9]=s->g_cur[0];state_out[10]=s->g_cur[1];state_out[11]=s->g_cur[2];
}

/* ================= PID ================= */
#define NF 2.0f
#define DH 0.7f
#define KIH 1.5f
#define AIM 3.0f
#define AGND 0.05f
#define KIV 0.8f
#define SLEW 1.0f
#define MASSK 0.060f

static inline float clampf(float x,float lo,float hi){ return x<lo?lo:(x>hi?hi:x); }

void kfc_control(kfc_state *s,const float state[KEST_NSTATES],
                 const float setpoint[KEST_NSTATES],float u_out[KCTRL_NACTIONS],float dt){
    float estR=state[3],estP=state[4];
    float v3=state[8];
    float estH=state[2];
    float desH=setpoint[2],desV1=setpoint[6],desV2=setpoint[7],yawT=setpoint[5];

    if(estH<AGND)s->alt_int=0; else s->alt_int=clampf(s->alt_int+KIH*(desH-estH)*dt,-AIM,AIM);
    float desAcc3=-2.0f*DH*NF*v3-NF*NF*(estH-desH)+s->alt_int;
    _Float16 rp_f16[2]={(_Float16)estR,(_Float16)estP};
    float cr_cp[2]; vc_cos2(rp_f16,cr_cp);
    float desNorm=(GRAV+desAcc3)/(cr_cp[0]*cr_cp[1]);

    /* horizontal velocity loop (fp32 vector, length-2) */
    int grounded=(estH<AGND);
    float desV[2]={desV1,desV2}, vv[2]={state[6],state[7]};
    float vi[2]={s->vel_int_1,s->vel_int_2};
    float prev[2]={ grounded?0.0f:s->desRoll_prev, grounded?0.0f:s->desPitch_prev };
    float desRP[2];
    vc_velloop(desV,vv,vi,fbits(KIV*dt),grounded,prev,fbits(SLEW*dt),desRP);
    s->vel_int_1=vi[0]; s->vel_int_2=vi[1];
    s->desRoll_prev=desRP[0]; s->desPitch_prev=desRP[1];

    /* attitude + rate loops, 4x4 mixer, force->duty (fp16 vector) */
    float tgt3[3]={desRP[0],desRP[1],yawT};
    vc_attitude(state,tgt3,hbits(desNorm*MASSK),u_out);
}
