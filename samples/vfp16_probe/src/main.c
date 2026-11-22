/*
 * Minimal probe: does the bench core execute VECTOR fp16 (Zvfh) ops?
 * Uses ONLY vector fp16 (vle16 / vfmacc.vv / vse16); NO scalar fp16 (no flh/
 * fsh/fadd.h). Inputs/outputs are raw uint16_t bit patterns so the compiler
 * emits no scalar _Float16 loads. Computes c += a*b elementwise with
 * a=1.0(0x3C00), b=2.0(0x4000), c=3.0(0x4200) -> expect 5.0 (0x4500).
 */
#include <zephyr/kernel.h>
#include <stdint.h>
#include <riscv_vector.h>

static uint16_t a[8], b[8], c[8], out[8];

int main(void)
{
	/* enable FS+VS (no fcsr) */
	unsigned long ms;
	__asm__ volatile("csrr %0, mstatus" : "=r"(ms));
	ms |= (0x6000UL | 0x600UL); /* FS=Initial|Dirty bits, VS bits */
	__asm__ volatile("csrw mstatus, %0" ::"r"(ms));

	for (int i = 0; i < 8; i++) { a[i] = 0x3C00; b[i] = 0x4000; c[i] = 0x4200; }

	printk("VFP16_PROBE: doing vle16 + vfmacc.vv + vse16 (SEW=16)\n");

	size_t vl = __riscv_vsetvl_e16m1(8);
	vfloat16m1_t va = __riscv_vle16_v_f16m1((const _Float16 *)a, vl);
	vfloat16m1_t vb = __riscv_vle16_v_f16m1((const _Float16 *)b, vl);
	vfloat16m1_t vc = __riscv_vle16_v_f16m1((const _Float16 *)c, vl);
	vc = __riscv_vfmacc_vv_f16m1(vc, va, vb, vl);
	__riscv_vse16_v_f16m1((_Float16 *)out, vc, vl);

	printk("VFP16_PROBE: out[0]=0x%04x out[7]=0x%04x (expect 0x4500)\n",
	       out[0], out[7]);
	printk("VFP16_PROBE: %s\n", (out[0] == 0x4500) ? "VECTOR_FP16_OK" : "VECTOR_FP16_WRONG");

	while (1) { k_msleep(1000); }
	return 0;
}
