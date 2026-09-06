/*
 * Saturn fp16 vector-op capability probe for the At35 combined bitstream
 * (RocketArty200TDroneGemminiSaturnFp16At35Config, misa.F=0).
 *
 * Runs ONE Saturn vector-fp op at a time, printing a marker BEFORE and an "OK"
 * AFTER each. On the bench, the LAST "OK" printed before an illegal-instruction
 * (mcause 2) trap names the first op the At35 Saturn does NOT implement.
 *
 * Motivation: the fp16 FC (build_fp16_flight) trapped on vfncvt.f.f.w (fp32->fp16
 * vector convert) in ve_amag. This probe maps exactly what the vector unit supports:
 *   (a) vfmul.vv  e16      -- fp16 vector ARITHMETIC
 *   (b) vfncvt.f.f.w       -- fp32 -> fp16 narrowing convert (the op that trapped)
 *   (c) vfwcvt.f.f.v       -- fp16 -> fp32 widening convert
 *   (d) vfcvt.f.x / .x.f   -- int16 <-> fp16 convert (for an int32-fixed-point fallback)
 *   (e) vfadd.vv e32       -- fp32 vector arithmetic (the fp16 kernels' accumulators)
 *
 * If (a) works but (b)/(c) don't, an ALL-fp16 (no fp32 accumulators, no converts)
 * or int32-fixed-point FC redesign could run WITHOUT a bitstream change.
 *
 * Built with a Zvfh -march; contains ZERO scalar float (buffers are integer bit
 * patterns), so nothing here can raise a scalar-FP trap -- any trap is the vector op.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <riscv_vector.h>

/* fp16 1.0 = 0x3c00 ; fp32 1.0 = 0x3f800000 -- benign non-zero, non-NaN inputs. */
static uint16_t f16buf[16] __attribute__((aligned(64)));
static uint32_t f32buf[16] __attribute__((aligned(64)));
static int16_t  i16buf[16] __attribute__((aligned(64)));

int main(void)
{
	printk("RVVFP16PROBE: start (At35 Saturn fp16 vector capability map)\n");
	for (int i = 0; i < 16; i++) { f16buf[i] = 0x3c00; f32buf[i] = 0x3f800000; i16buf[i] = 2; }
	const size_t vl = 8;

	printk("probe a: vfmul.vv e16 (fp16 vector ARITH)\n");
	{
		vfloat16m1_t a = __riscv_vle16_v_f16m1((const _Float16 *)f16buf, vl);
		vfloat16m1_t r = __riscv_vfmul_vv_f16m1(a, a, vl);
		__riscv_vse16_v_f16m1((_Float16 *)f16buf, r, vl);
	}
	printk("probe a OK: fp16 vfmul supported\n");

	/* Order: fp16 arith (a) then the UNKNOWN converts (d int16<->fp16, c fp16->fp32)
	 * BEFORE the ops already known to trap (e fp32 arith, b fp32->fp16), so one run
	 * maps everything -- a trap halts the probe, so knowns go last. */
	printk("probe d: vfcvt int16<->fp16 (for an int/fixed-point fp16 path)\n");
	{
		vint16m1_t x = __riscv_vle16_v_i16m1(i16buf, vl);
		vfloat16m1_t f = __riscv_vfcvt_f_x_v_f16m1(x, vl);   /* int16 -> fp16 */
		vint16m1_t y = __riscv_vfcvt_x_f_v_i16m1(f, vl);     /* fp16 -> int16 */
		__riscv_vse16_v_i16m1(i16buf, y, vl);
	}
	printk("probe d OK: vfcvt int16<->fp16 supported\n");

	printk("probe c: vfwcvt.f.f.v (fp16->fp32 widening convert)\n");
	{
		vfloat16mf2_t a = __riscv_vle16_v_f16mf2((const _Float16 *)f16buf, vl);
		vfloat32m1_t r = __riscv_vfwcvt_f_f_v_f32m1(a, vl);
		__riscv_vse32_v_f32m1((float *)f32buf, r, vl);
	}
	printk("probe c OK: vfwcvt.f.f.v supported\n");

	printk("probe e: vfadd.vv e32 (fp32 vector ARITH -- known to trap on At35)\n");
	{
		vfloat32m1_t a = __riscv_vle32_v_f32m1((const float *)f32buf, vl);
		vfloat32m1_t r = __riscv_vfadd_vv_f32m1(a, a, vl);
		__riscv_vse32_v_f32m1((float *)f32buf, r, vl);
	}
	printk("probe e OK: fp32 vfadd supported\n");

	printk("probe b: vfncvt.f.f.w (fp32->fp16 narrowing convert -- known to trap)\n");
	{
		vfloat32m1_t a = __riscv_vle32_v_f32m1((const float *)f32buf, vl);
		vfloat16mf2_t r = __riscv_vfncvt_f_f_w_f16mf2(a, vl);
		__riscv_vse16_v_f16mf2((_Float16 *)f16buf, r, vl);
	}
	printk("probe b OK: vfncvt.f.f.w supported\n");

	printk("RVVFP16PROBE: ALL OPS OK (a,e,d,c,b) -- full fp16+fp32 vector supported\n");
	/* keep the value live so nothing is optimized away */
	printk("RVVFP16PROBE: sink f16[0]=0x%04x f32[0]=0x%08x i16[0]=%d\n",
	       (unsigned)f16buf[0], (unsigned)f32buf[0], (int)i16buf[0]);
	for (;;) { k_msleep(1000); }
	return 0;
}
