/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/arch/cpu.h> // Required for arch_curr_cpu()->id

#define NUM_HARTS   CONFIG_MP_MAX_NUM_CPUS // Number of available harts (cores)
#define NUM_THREADS NUM_HARTS * 4          // Number of threads to create
#define STACK_SIZE  1024                   // Stack size per thread

K_THREAD_STACK_ARRAY_DEFINE(thread_stacks, NUM_THREADS, STACK_SIZE);
struct k_thread thread_data[NUM_THREADS];
k_tid_t thread_ids[NUM_THREADS];

struct k_mutex print_mutex;

void thread_function(void *arg1, void *arg2, void *arg3)
{
	int thread_id = (int)(uintptr_t)arg1;
	int hart_id;

	k_mutex_lock(&print_mutex, K_FOREVER);
	hart_id = arch_curr_cpu()->id;
	printf("Thread %d running on hart %d\n", thread_id, hart_id);
	k_mutex_unlock(&print_mutex);

	unsigned char buf[4]
		__attribute__((aligned(16))) = {thread_id, thread_id, thread_id, thread_id};

	/* Load v1 with this thread's id, then yield so other threads on the
	 * same hart run (and overwrite v1 in hardware).  arch/riscv/core/v.c
	 * must save/restore v1 per-thread across the context switch for the
	 * value to survive to the vse32.v below.
	 *
	 * NOTE: the v-register clobber ("v1") is intentionally omitted — the
	 * Zephyr SDK 0.17.0 GCC 12.2.0 inline-asm parser does not know RVV
	 * register names (that landed in GCC 14). volatile + the "memory"
	 * clobber keep both asm blocks in order around k_yield(), and GCC
	 * 12.2.0 emits no vector code of its own here, so v1 is touched only
	 * by these two blocks — the same convention arch/riscv/core/v.c uses. */
	__asm volatile(".option push\n\t"
		       ".option arch, +v\n\t"
		       "vsetivli x0, 4, e32, m1\n\t"
		       "vle32.v v1, (%[buf])\n\t"
		       ".option pop"
		       :
		       : [buf] "r"(buf)
		       : "memory");

	k_yield();

	__asm volatile(".option push\n\t"
		       ".option arch, +v\n\t"
		       "vse32.v v1, (%[buf])\n\t"
		       ".option pop"
		       :
		       : [buf] "r"(buf)
		       : "memory");

	k_mutex_lock(&print_mutex, K_FOREVER);
	hart_id = arch_curr_cpu()->id;
	printf("Thread %d on hart %d finished execution\n", thread_id, hart_id);

	int error = 0;
	for (int i = 0; i < 4; i++) {
		if (buf[i] != thread_id) {
			error = 1;
			printf("error: buf[%d] = 0x%08X\n", i, buf[i]);
		}
	}

	if (!error) {
		printf("Success\n");
	}

	k_mutex_unlock(&print_mutex);
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	printf("Zephyr is running on %d CPUs\n", CONFIG_MP_MAX_NUM_CPUS);
	k_mutex_init(&print_mutex);

	for (int i = 0; i < NUM_THREADS; i++) {
		int hart_id = i % NUM_HARTS;

		k_tid_t tid = k_thread_create(&thread_data[i], thread_stacks[i], STACK_SIZE,
					      thread_function, (void *)(uintptr_t)i, NULL, NULL,
					      K_PRIO_PREEMPT(1), 0, K_FOREVER);
		// k_thread_cpu_mask_clear(tid) ;
		k_thread_cpu_pin(tid, hart_id);
		k_thread_start(tid);
		thread_ids[i] = tid;

		k_mutex_lock(&print_mutex, K_FOREVER);
		printf("Thread %d assigned to hart %d\n", i, hart_id);
		k_mutex_unlock(&print_mutex);
	}

	for (int i = 0; i < NUM_THREADS; i++) {
		k_thread_join(thread_ids[i], K_FOREVER);
	}

	printf("All threads finished execution.\n");
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}
