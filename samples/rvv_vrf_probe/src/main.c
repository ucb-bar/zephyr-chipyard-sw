/*
 * Saturn VRF disambiguation probe for the misaVfix RoCC vcore.
 *
 * The eager-V context switch's `vle8.v v24 (e8,m8)` traps ILLEGAL (mcause 2) on
 * a2042d3d's misaVfix Saturn, while `vle8.v v0/v8/v16 (e8,m8)` execute (same VLEN,
 * vlenb=16). This probe pins the cause: MISSING top registers (v24-v31 dropped by
 * the FP-strip) vs an m8-GROUP-DECODE quirk for the top group.
 *
 * Runs ONE vector op at a time with a marker before/after. The LAST "OK" before an
 * illegal-instruction (mcause 2) trap names the first unsupported case:
 *   a: vle8.v v0  @m1   -- basic single-reg load (baseline, must work)
 *   b: vle8.v v16 @m8   -- top-half m8 group that WORKS on-bench (baseline)
 *   c: vle8.v v24 @m1   -- does the register v24 EXIST at all?  (m1 = just v24)
 *   d: vle8.v v28 @m1   -- does v28 exist?
 *   e: vle8.v v31 @m1   -- does v31 exist?
 *   f: vle8.v v24 @m2   -- v24-v25
 *   g: vle8.v v24 @m4   -- v24-v27
 *   h: vle8.v v24 @m8   -- v24-v31 : the FAILING case
 * If c/d/e (m1) trap -> registers v24-v31 are MISSING (VRF truncated).
 * If c/d/e work but h traps -> the m8 top-GROUP decode is the quirk (regs exist).
 *
 * Pure integer vector (zve64x) + integer buffer: no scalar float, no fp vector, so
 * any trap is the vle8.v itself. Needs misa.V=1 (vsetvli decodes) -- run on the
 * misaVfix (or -1) bit.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

static uint8_t vbuf[2048] __attribute__((aligned(128)));   /* enough for e8,m8 (128 B/grp) */

#define VLD(mnem, lmul, vreg) \
	__asm volatile(".option push\n\t.option arch, +v\n\t" \
		       "vsetvli t0, x0, e8, " lmul ", ta, ma\n\t" \
		       "vle8.v " vreg ", (%0)\n\t" \
		       ".option pop\n\t" :: "r"(vbuf) : "t0", "memory")

int main(void)
{
	printk("RVVVRFPROBE: start (Saturn VRF disambiguation: missing-reg vs m8-top-group)\n");
	for (int i = 0; i < (int)sizeof(vbuf); i++) vbuf[i] = (uint8_t)i;

	printk("probe a: vle8.v v0 @m1 (baseline single-reg)\n");   VLD("a", "m1", "v0");  printk("probe a OK\n");
	printk("probe b: vle8.v v16 @m8 (known-good top-half m8)\n"); VLD("b", "m8", "v16"); printk("probe b OK\n");
	printk("probe c: vle8.v v24 @m1 (does v24 exist?)\n");        VLD("c", "m1", "v24"); printk("probe c OK: v24 exists\n");
	printk("probe d: vle8.v v28 @m1 (does v28 exist?)\n");        VLD("d", "m1", "v28"); printk("probe d OK: v28 exists\n");
	printk("probe e: vle8.v v31 @m1 (does v31 exist?)\n");        VLD("e", "m1", "v31"); printk("probe e OK: v31 exists\n");
	printk("probe f: vle8.v v24 @m2 (v24-v25)\n");                VLD("f", "m2", "v24"); printk("probe f OK: v24@m2\n");
	printk("probe g: vle8.v v24 @m4 (v24-v27)\n");                VLD("g", "m4", "v24"); printk("probe g OK: v24@m4\n");
	printk("probe h: vle8.v v24 @m8 (v24-v31, the FAILING case)\n"); VLD("h", "m8", "v24"); printk("probe h OK: v24@m8 works!\n");

	printk("RVVVRFPROBE: ALL OK -- v24-v31 present AND m8-top-group decodes (quirk not reproduced)\n");
	printk("RVVVRFPROBE: sink=%d\n", (int)vbuf[0]);
	for (;;) { k_msleep(1000); }
	return 0;
}
