/*
 * vstart!=0 trigger probe for the misaVfix RoCC vcore.
 *
 * `vle8.v v24 @m8` runs CLEAN standalone (VRF probe: all cases OK) but traps
 * mcause-2 illegal inside the eager-V context-switch restore. Hypothesis: a
 * non-zero vstart (a partially-executed/preempted vector op left vstart!=0) makes
 * the Saturn reject the m8 unit-stride reload. This probe presets vstart and re-runs
 * the exact instruction standalone to confirm/deny vstart!=0 as the trigger.
 *
 *   A: vsetvli e8,m8; vstart=0;  vle8.v v24   -- baseline (known clean)
 *   B: vsetvli e8,m8; vstart=1;  vle8.v v24   -- THE TEST: if this traps, vstart!=0 confirmed
 *   C: vsetvli e8,m8; vstart=64; vle8.v v24   -- another vstart!=0 point
 *   D: vsetvli e8,m8; vstart=1;  vle8.v v0    -- control (is it v24-specific or any m8 reload?)
 * Last "OK" before an mcause-2 trap names the trigger. Pure integer vector (zve64x);
 * needs misa.V=1. vsetvli does NOT clear vstart (RVV spec), so the csrw persists to vle8.v.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

static uint8_t vbuf[2048] __attribute__((aligned(128)));

#define VLD_VSTART(vs, vreg) \
	__asm volatile(".option push\n\t.option arch, +v\n\t" \
		       "vsetvli t0, x0, e8, m8, ta, ma\n\t" \
		       "li t1, " vs "\n\t" \
		       "csrw vstart, t1\n\t" \
		       "vle8.v " vreg ", (%0)\n\t" \
		       ".option pop\n\t" :: "r"(vbuf) : "t0", "t1", "memory")

int main(void)
{
	printk("RVVVSTART: start (is vstart!=0 the vle8.v v24@m8 trap trigger?)\n");
	for (int i = 0; i < (int)sizeof(vbuf); i++) vbuf[i] = (uint8_t)i;

	printk("probe A: vsetvli e8,m8; vstart=0; vle8.v v24 (baseline)\n");
	VLD_VSTART("0", "v24");  printk("probe A OK (vstart=0)\n");

	printk("probe B: vsetvli e8,m8; vstart=1; vle8.v v24 (THE TEST)\n");
	VLD_VSTART("1", "v24");  printk("probe B OK: v24@m8 TOLERATES vstart=1\n");

	printk("probe C: vsetvli e8,m8; vstart=64; vle8.v v24\n");
	VLD_VSTART("64", "v24"); printk("probe C OK: vstart=64\n");

	printk("probe D: vsetvli e8,m8; vstart=1; vle8.v v0 (control)\n");
	VLD_VSTART("1", "v0");   printk("probe D OK: vstart=1 on v0\n");

	printk("RVVVSTART: ALL OK -- vstart!=0 is NOT the trigger (m8 reload tolerates it)\n");
	printk("RVVVSTART: sink=%d\n", (int)vbuf[0]);
	for (;;) { k_msleep(1000); }
	return 0;
}
