/*
 * fc_rocc_fw.c -- on-target firmware driver for the hardened FC RoCC accelerator.
 *
 * Runs the estimator + PID on the FcRoCC (custom-0 opcode) instead of the scalar
 * core / Saturn. All integer: the RoCC is fed Q24.24 fixed-point (the FC firmware
 * converts raw sensor ints -> Q24.24 with integer mul/shift, NO FPU), and returns
 * Q24.24 motor duties. Validated off-board bit-for-bit by fc_rocc_asm.c (same
 * PUSH/RUN/POP sequence through the datapath interpreter, open 6.79um / closed 51um
 * vs the fp32 golden; the RTL fc_rocc_core.sv reproduces it in xsim).
 *
 * custom-0 ISA (func7 = funct; func3 encodes xd/xs1/xs2):
 *   PUSH (funct 0): rf[rs1] = rs2                 (stream a Q24.24 input)   xd0 xs1 xs2
 *   RUN  (funct 1): run program at rs1 until HALT; blocks (xd1) -> done token
 *   POP  (funct 2): rd = rf[rs1]                  (read a Q24.24 output)    xd1 xs1
 *   CFG  (funct 3): rf[rs1] = rs2                 (boot-load const/state)   xd0 xs1 xs2
 *
 * Per tick: PUSH 12 sensor Q24 -> RUN(EST) -> POP 12 state_out ; PUSH 4 setpoint+dt
 * -> RUN(CTRL) -> POP 4 duties. Boot once: CFG-load the FCR_INIT[] image.
 */
#include "fc_rocc_fw.h"
#include <stdint.h>

#define FCR_OP_PUSH 0
#define FCR_OP_RUN  1
#define FCR_OP_POP  2
#define FCR_OP_CFG  3

/* .insn r opcode=0x0B(custom0), func3, func7=funct, rd, rs1, rs2.
 * func3=3 => xd=0,xs1=1,xs2=1 (two src, no dest);  func3=6 => xd=1,xs1=1,xs2=0. */
#define FCR_SS(funct, rs1, rs2) \
  asm volatile(".insn r 0x0B, 3, " #funct ", x0, %0, %1" :: "r"(rs1), "r"(rs2))
#define FCR_DS(rd, funct, rs1) \
  asm volatile(".insn r 0x0B, 6, " #funct ", %0, %1, x0" : "=r"(rd) : "r"(rs1))

static inline void fcr_push(uint32_t idx, int64_t val){ FCR_SS(0, (uint64_t)idx, (uint64_t)val); }
static inline void fcr_cfg (uint32_t idx, int64_t val){ FCR_SS(3, (uint64_t)idx, (uint64_t)val); }
static inline int64_t fcr_pop(uint32_t idx){ uint64_t r; FCR_DS(r, 2, (uint64_t)idx); return (int64_t)(r<<16)>>16; /* sign-extend 48->64 */ }
static inline void fcr_run(uint32_t start){ uint64_t d; FCR_DS(d, 1, (uint64_t)start); (void)d; /* blocks until HALT */ }

/* boot: load consts + initial state into the RoCC regfile (once, at startup). */
void fcr_boot(void){
    for (uint32_t r = 0; r < FCR_NREG; r++) fcr_cfg(r, (int64_t)FCR_INIT[r]);
}

/* re-init just the pose (x,y,z,quat) if the estimator is reset in flight.
 * (reg indices: from FCR_INIT the state regs keep their boot values; to reset
 *  pose, CFG-write the pos/quat regs -- exposed via the map if needed.) */

/* ---- fixed-point helpers (integer only) ---- */
static inline int64_t f2q(int32_t raw, int shift){ return (int64_t)raw << shift; } /* raw*2^shift */

/* Estimator step: sensor inputs already in Q24.24 (accel/gyro/flow/height/dt),
 * flags 0/1. Fills state_out_q[12] (pos Q_POS, att QA, vel Q_VEL, rates QA). */
void fc_rocc_estimate_q(const int64_t in_q[10], int flow_valid, int tof_valid,
                        int64_t state_out_q[12]){
    fcr_push(FCR_IN_AX, in_q[0]); fcr_push(FCR_IN_AY, in_q[1]); fcr_push(FCR_IN_AZ, in_q[2]);
    fcr_push(FCR_IN_GX, in_q[3]); fcr_push(FCR_IN_GY, in_q[4]); fcr_push(FCR_IN_GZ, in_q[5]);
    fcr_push(FCR_IN_F0, in_q[6]); fcr_push(FCR_IN_F1, in_q[7]);
    fcr_push(FCR_IN_H,  in_q[8]); fcr_push(FCR_IN_DT, in_q[9]);
    fcr_push(FCR_IN_FV, flow_valid ? 1 : 0); fcr_push(FCR_IN_TV, tof_valid ? 1 : 0);
    fcr_run(FCR_EST_START);
    for (int i = 0; i < 12; i++) state_out_q[i] = fcr_pop(FCR_SOUT[i]);
}

/* Controller step: setpoint (desH, desV1, desV2, yawT) + dt in Q24.24.
 * Fills u_out_q[4] = normalized-thrust duties in Q24.24 (duty-0.583). */
void fc_rocc_control_q(int64_t desH_q, int64_t desV1_q, int64_t desV2_q,
                       int64_t yawT_q, int64_t dt_q, int64_t u_out_q[4]){
    fcr_push(FCR_IN_DESH, desH_q); fcr_push(FCR_IN_DESV1, desV1_q);
    fcr_push(FCR_IN_DESV2, desV2_q); fcr_push(FCR_IN_YAWT, yawT_q);
    fcr_push(FCR_IN_DT, dt_q);
    fcr_run(FCR_CTRL_START);
    u_out_q[0] = fcr_pop(FCR_U0); u_out_q[1] = fcr_pop(FCR_U1);
    u_out_q[2] = fcr_pop(FCR_U2); u_out_q[3] = fcr_pop(FCR_U3);
    (void)f2q;
}

/*
 * INTEGRATION (for the FC agent):
 *   - Call fcr_boot() once after reset (before the control loop).
 *   - Per tick: convert raw sensors -> Q24.24 (int shifts on the driver counts),
 *     call fc_rocc_estimate_q(...), then fc_rocc_control_q(...); convert
 *     u_out_q[i] (Q24.24) -> PWM duty with an integer scale (duty = (u>>k)...),
 *     no FPU. state_out_q feeds telemetry/watchdog.
 *   - Wrap as Fp16-style drop-ins (IStateEstimator/IController) selected by
 *     -DROSE_USE_ROCC=1 in estimator_factory/controller_factory. No main.cpp change.
 *   - REQUIRES a bitstream whose FcRoCC holds the assembled ROM (fc_rocc_rom.hex)
 *     + boot image (fc_rocc_init.hex) -- i.e. the validated-core combined build.
 *   - Host twin (validated): samples/fp16_estim_feasibility/src/fc_rocc_asm.c runs
 *     the identical PUSH/RUN/POP sequence through the datapath interpreter and
 *     matches the fp32 golden (open 6.79um / closed 51um).
 */
