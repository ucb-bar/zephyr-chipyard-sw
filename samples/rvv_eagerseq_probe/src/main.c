/* Replicate the EXACT eager-V restore reload: 4 consecutive m8 vle8.v
 * (v0->v8->v16->v24) at vstart=0, the sequence where only v24 (4th) traps
 * inside z_riscv_vstate_restore_thread. If v24 traps here standalone, the trigger
 * is the consecutive-4x-m8-load pattern (a Saturn pipeline/scheduler bug), not
 * vstart/decode/context. Pure integer vector; needs misa.V=1. */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
static uint8_t vbuf[4096] __attribute__((aligned(128)));
int main(void){
	printk("RVVEAGERSEQ: start (exact eager-V reload: 4 consecutive vle8.v v0/v8/v16/v24 @m8)\n");
	for (int i=0;i<(int)sizeof(vbuf);i++) vbuf[i]=(uint8_t)i;
	printk("seq: csrw vstart,0; vsetvli e8,m8; vle8.v v0; +vl; v8; +vl; v16; +vl; v24\n");
	__asm volatile(
		".option push\n\t.option arch, +v\n\t"
		"csrw vstart, x0\n\t"
		"vsetvli t0, x0, e8, m8, ta, ma\n\t"
		"mv t2, %0\n\t"
		"vle8.v v0,  (t2)\n\t add t2, t2, t0\n\t"
		"vle8.v v8,  (t2)\n\t add t2, t2, t0\n\t"
		"vle8.v v16, (t2)\n\t add t2, t2, t0\n\t"
		"vle8.v v24, (t2)\n\t"
		".option pop\n\t" :: "r"(vbuf) : "t0","t2","memory");
	printk("RVVEAGERSEQ: OK -- 4x consecutive m8 vle8.v executed, v24 did NOT trap standalone\n");
	printk("RVVEAGERSEQ: sink=%d\n",(int)vbuf[0]);
	for(;;) k_msleep(1000);
	return 0;
}
