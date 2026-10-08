/*
 * merlin_hetero_runner — parallel async inferences across both Shuttle harts.
 *
 * Two pinned worker threads (hart 0 + hart 1), each owning its own
 * iree_vm_context + iree_hal_device built at startup. main() submits
 * MERLIN_JOBS inferences (round-robin across harts) to their per-hart
 * k_msgq, awaits completion via a per-job k_sem, and at the end dumps
 * per-job + per-hart cycle stats.
 *
 * Per-job cost in steady state ~= iree_vm_invoke + d2h + hash. The 575 M
 * setup cost is paid ONCE per hart at boot, NOT per job.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include "iree/base/api.h"

#include "model_info.h"
#include "hetero_worker.h"

/* Embedded VMFBs (one per hart's ISA). xxd emits:
 *   unsigned char merlin_vmfb_hX[]   = { ... };
 *   unsigned int  merlin_vmfb_hX_len = ...;
 */
#include "merlin_vmfb_h0.h"
#include "merlin_vmfb_h1.h"

#ifndef MERLIN_JOBS
#define MERLIN_JOBS 32
#endif
#ifndef MERLIN_BACKEND_H0
#define MERLIN_BACKEND_H0 "scalar"
#endif
#ifndef MERLIN_BACKEND_H1
#define MERLIN_BACKEND_H1 "rvv"
#endif
#ifndef MERLIN_NEEDS_VECTOR_H0
#define MERLIN_NEEDS_VECTOR_H0 0
#endif
#ifndef MERLIN_NEEDS_VECTOR_H1
#define MERLIN_NEEDS_VECTOR_H1 1
#endif

#if defined(MERLIN_MODEL_MLP_WIDE)
#define MERLIN_MODEL_NAME "mlp_wide"
#elif defined(MERLIN_MODEL_DRONET)
#define MERLIN_MODEL_NAME "dronet"
#elif defined(MERLIN_MODEL_YOLOV8N)
#define MERLIN_MODEL_NAME "yolov8n"
#else
#define MERLIN_MODEL_NAME "unknown"
#endif

#define HART_COUNT 2

static inline uint64_t rdmcycle_m(void) {
  uint64_t c;
  __asm__ __volatile__("csrr %0, mcycle" : "=r"(c));
  return c;
}

/* ---------- per-job result table ---------- */
typedef struct {
  uint64_t cycles;
  uint64_t hash;
  uint32_t hart_id;
  int32_t  rc;
} merlin_job_result_t;

static merlin_job_result_t g_job_results[MERLIN_JOBS];

/* Called by the worker via extern. */
void merlin_record_job_result(uint32_t job_id, uint64_t cycles, uint64_t hash,
                              int rc, uint32_t hart_id) {
  if (job_id < MERLIN_JOBS) {
    g_job_results[job_id].cycles = cycles;
    g_job_results[job_id].hash = hash;
    g_job_results[job_id].hart_id = hart_id;
    g_job_results[job_id].rc = rc;
  }
}

/* ---------- worker state + Zephyr resources ---------- */
static merlin_worker_state_t g_worker_state[HART_COUNT];
/* 2026-05-17 ROOT-CAUSE FIX (long-standing dronet × Gemmini hang):
 * IREE-generated dispatch ELFs allocate the matmul output buffer as a
 * stack `alloca` in the dispatch function's prologue. For dronet's
 * conv1 matmul (3136 × 32 i32), each dispatch invocation needs
 * 3136*32*4 = 401,408 bytes on the stack — TWICE that for the bias-add
 * temp + main output = ~800 KiB. The previous 96 KiB worker stack
 * silently overflowed and the dispatch hung after `[entry] o=12`,
 * which we mis-attributed to a Gemmini RoCC emission bug for many
 * hours (tried OS path, bulk LOOP_WS, per-tile LOOP_WS — all hung at
 * the same point because the hang is in the prologue, before our
 * matmul code runs). mlp_wide passes with 96 KiB because its 16×32
 * matmul output is only 2 KiB stack. yolov8n will need even more
 * headroom — bumping to 4 MiB covers every workload we currently
 * compile. */
#define MERLIN_WORKER_STACK_SIZE (4 * 1024 * 1024)
K_THREAD_STACK_ARRAY_DEFINE(g_worker_stack, HART_COUNT,
                            MERLIN_WORKER_STACK_SIZE);
static struct k_thread g_worker_thread[HART_COUNT];

#define MERLIN_MSGQ_CAPACITY 64  /* >= MERLIN_JOBS/HART_COUNT + slack */
K_MSGQ_DEFINE(g_job_msgq_h0, sizeof(merlin_job_t), MERLIN_MSGQ_CAPACITY, 8);
K_MSGQ_DEFINE(g_job_msgq_h1, sizeof(merlin_job_t), MERLIN_MSGQ_CAPACITY, 8);

static struct k_sem g_completion_sems[MERLIN_JOBS];

/* PHASE-D diagnostic: live progress marker. The merlin gemmini plugin
 * emits stores to this variable from inside dispatch ELFs (one store
 * before each (i0,j0,k0) outer tile call to spTiledMatmul, plus extra
 * stores between MVIN-D/MVIN-B/MVIN-A/COMPUTE/MVOUT sub-blocks of
 * spTiledMatmulOs). The address of this symbol must be hardcoded
 * into the plugin (since the dispatch ELF doesn't link Zephyr).
 *
 * After Zephyr is built, get the actual address via:
 *   riscv64-unknown-elf-nm zephyr.elf | grep merlin_progress_marker
 * and update the plugin's emitMarker function in
 * `compiler/src/merlin/Dialect/Gemmini/Transforms/LegalizeForLLVMExport.cpp`
 * to write to that address.
 *
 * A watchdog thread (started below from main) polls this marker
 * every 100ms and prints to UART so we get LIVE progress visible in
 * the FireSim uartlog. If the dispatch hangs, the last printed
 * marker value reveals the LAST (i0,j0,k0) tile that executed, and
 * the LAST sub-block within that tile.
 */
volatile uint64_t merlin_progress_marker = 0;

static struct k_thread g_watchdog_thread;
#define MERLIN_WATCHDOG_STACK_SIZE 4096
K_THREAD_STACK_DEFINE(g_watchdog_stack, MERLIN_WATCHDOG_STACK_SIZE);

static void merlin_watchdog_entry(void *a, void *b, void *c) {
  (void)a; (void)b; (void)c;
  uint64_t last = 0;
  uint32_t tick = 0;
  while (1) {
    uint64_t cur = merlin_progress_marker;
    if (cur != last) {
      printk("[mrk] %llx (tick=%u)\n",
             (unsigned long long)cur, (unsigned)tick);
      last = cur;
    }
    /* Periodic alive print every ~5s (50 ticks @ 100ms) — proves
     * the watchdog thread is being scheduled even if the marker
     * never changes. Distinguishes "marker never written" from
     * "watchdog never runs". */
    if ((tick % 50) == 0) {
      printk("[mrk-alive] tick=%u mrk=%llx\n",
             (unsigned)tick, (unsigned long long)cur);
    }
    tick++;
    k_sleep(K_MSEC(100));
  }
}

int main(void) {
  printf("=== merlin_hetero_runner ===\n");
  printf("model=%s jobs=%d hart0=%s hart1=%s\n",
         MERLIN_MODEL_NAME, MERLIN_JOBS,
         MERLIN_BACKEND_H0, MERLIN_BACKEND_H1);
  printf("vmfb_h0=%u bytes  vmfb_h1=%u bytes\n",
         (unsigned)merlin_vmfb_h0_len, (unsigned)merlin_vmfb_h1_len);

  /* ---- configure per-hart state ---- */
  g_worker_state[0] = (merlin_worker_state_t){
      .hart_id = 0,
      .needs_vector = MERLIN_NEEDS_VECTOR_H0,
      .backend_name = MERLIN_BACKEND_H0,
      .vmfb = {merlin_vmfb_h0, merlin_vmfb_h0_len},
      .job_msgq = &g_job_msgq_h0,
  };
  g_worker_state[1] = (merlin_worker_state_t){
      .hart_id = 1,
      .needs_vector = MERLIN_NEEDS_VECTOR_H1,
      .backend_name = MERLIN_BACKEND_H1,
      .vmfb = {merlin_vmfb_h1, merlin_vmfb_h1_len},
      .job_msgq = &g_job_msgq_h1,
  };
  k_sem_init(&g_worker_state[0].startup_sem, 0, 1);
  k_sem_init(&g_worker_state[1].startup_sem, 0, 1);

  /* ---- per-job completion sems ---- */
  for (int j = 0; j < MERLIN_JOBS; ++j) {
    k_sem_init(&g_completion_sems[j], 0, 1);
  }

  /* Enable the upstream merlin per-dispatch debug print
   * (embedded_elf_loader.c emits "[dn] o=N sym=S wg_count=X,Y,Z" on the
   * first workgroup of every dispatch). The last [dn] line printed
   * before UART silence pinpoints a hanging dispatch. */
  extern int iree_merlin_dispatch_debug_enabled;
  iree_merlin_dispatch_debug_enabled = 1;

  /* Phase-2 alignment debug: print binding pointer + alignment + length
   * per dispatch. Use to verify Zephyr's binding base pointers are sane
   * for a hanging dispatch (the Gemmini DMA needs 16-byte aligned
   * source addresses, and zero-length / NaN-pointer bindings would
   * otherwise produce a silent hang inside the dispatch). */
  extern int iree_merlin_binding_debug_enabled;
  iree_merlin_binding_debug_enabled = 1;

  /* (Watchdog thread DISABLED — the COOP-priority polling fought with
   * hart 1's worker thread and stalled OPU init at encoding_8 wg=21.
   * Keep the merlin_progress_marker symbol so the plugin's marker
   * stores remain harmless, but don't print live progress for now.) */

  /* ---- spawn pinned workers ---- */
  uint64_t spawn_t0 = rdmcycle_m();
  k_tid_t tids[HART_COUNT];
  for (int h = 0; h < HART_COUNT; ++h) {
    tids[h] = k_thread_create(
        &g_worker_thread[h], g_worker_stack[h], MERLIN_WORKER_STACK_SIZE,
        merlin_worker_entry, &g_worker_state[h], NULL, NULL,
        /*prio=*/K_PRIO_COOP(0), /*opts=*/0, /*delay=*/K_FOREVER);
    int pin_rc = k_thread_cpu_pin(tids[h], h);
    if (pin_rc != 0) {
      printf("[hetero] FAIL k_thread_cpu_pin hart %d rc=%d\n", h, pin_rc);
      sys_reboot(SYS_REBOOT_COLD);
      return 1;
    }
    k_thread_start(tids[h]);
  }

  /* ---- wait for both workers to finish init ---- */
  k_sem_take(&g_worker_state[0].startup_sem, K_FOREVER);
  k_sem_take(&g_worker_state[1].startup_sem, K_FOREVER);
  uint64_t init_t1 = rdmcycle_m();
  printf("[hetero] both harts initialized cycles=%llu  "
         "init_h0=%llu init_h1=%llu\n",
         (unsigned long long)(init_t1 - spawn_t0),
         (unsigned long long)g_worker_state[0].cycles_init,
         (unsigned long long)g_worker_state[1].cycles_init);

  /* (Watchdog spawn intentionally disabled — see comment at line 130 in
   * merlin_watchdog_entry. Spawning the watchdog appears to correlate
   * with the misaligned-load fault in iree_hal_elf_executable_issue_call
   * after dispatch_3 of mlp_wide × Gemmini on FireSim (s3 corrupted by
   * unknown mechanism). The plugin's emitMarker stores to BSS still
   * fire harmlessly even without the watchdog; recovering the marker
   * values is just not surfaced live anymore. We don't need it for
   * production paths now that the root-cause hang (stack overflow) is
   * understood and fixed via MERLIN_WORKER_STACK_SIZE = 4 MiB. */

  /* ---- submit MERLIN_JOBS jobs round-robin ---- */
  printf("[hetero] submitting %d jobs (round-robin)\n", MERLIN_JOBS);
  uint64_t submit_t0 = rdmcycle_m();
  for (int j = 0; j < MERLIN_JOBS; ++j) {
    merlin_job_t job = {
        .job_id = (uint32_t)j,
        .completion_sem = &g_completion_sems[j],
    };
    int target_hart = j % HART_COUNT;
    int rc = k_msgq_put(g_worker_state[target_hart].job_msgq,
                        &job, K_NO_WAIT);
    if (rc != 0) {
      printf("[hetero] FAIL k_msgq_put job %d hart %d rc=%d\n",
             j, target_hart, rc);
      sys_reboot(SYS_REBOOT_COLD);
      return 1;
    }
  }

  /* ---- wait for all completions ---- */
  for (int j = 0; j < MERLIN_JOBS; ++j) {
    k_sem_take(&g_completion_sems[j], K_FOREVER);
  }
  uint64_t submit_t1 = rdmcycle_m();
  uint64_t parallel_wall = submit_t1 - submit_t0;

  /* ---- send drain signals + join ---- */
  merlin_job_t drain = {.job_id = UINT32_MAX, .completion_sem = NULL};
  k_msgq_put(g_worker_state[0].job_msgq, &drain, K_NO_WAIT);
  k_msgq_put(g_worker_state[1].job_msgq, &drain, K_NO_WAIT);
  k_thread_join(tids[0], K_FOREVER);
  k_thread_join(tids[1], K_FOREVER);

  /* ---- dump per-job and per-hart stats ---- */
  printf("[hetero] === per-job log ===\n");
  uint64_t sum_h0 = 0, sum_h1 = 0;
  uint32_t n_h0 = 0, n_h1 = 0;
  int n_ok = 0;
  for (int j = 0; j < MERLIN_JOBS; ++j) {
    const merlin_job_result_t* r = &g_job_results[j];
    printf("[hetero] job=%d hart=%u cycles=%llu hash=0x%016llx rc=%d\n",
           j, (unsigned)r->hart_id,
           (unsigned long long)r->cycles,
           (unsigned long long)r->hash, r->rc);
    if (r->rc == 0) {
      n_ok++;
      if (r->hart_id == 0) { sum_h0 += r->cycles; n_h0++; }
      else                 { sum_h1 += r->cycles; n_h1++; }
    }
  }

  /* ---- per-dispatch cycle decomposition ---- */
  /* 2026-05-17: dump per-ordinal cycle accumulation so we can produce
   * a per-dispatch performance breakdown after the run. Output lines
   * look like:
   *   CYC, <ord>, <symbol>, <total_cycles>, <wg_count>
   * Parseable via tools/perf_decompose.py to produce a percentage
   * breakdown per (dispatch, accelerator). */
  extern void iree_merlin_dump_cycles(void);
  iree_merlin_dump_cycles();

  uint64_t sum_total = sum_h0 + sum_h1;
  printf("[hetero] === summary ===\n");
  printf("[hetero] jobs_ok=%d / %d\n", n_ok, MERLIN_JOBS);
  printf("[hetero] hart0 jobs=%u total_invoke_cycles=%llu  avg=%llu\n",
         n_h0, (unsigned long long)sum_h0,
         n_h0 ? (unsigned long long)(sum_h0 / n_h0) : 0ULL);
  printf("[hetero] hart1 jobs=%u total_invoke_cycles=%llu  avg=%llu\n",
         n_h1, (unsigned long long)sum_h1,
         n_h1 ? (unsigned long long)(sum_h1 / n_h1) : 0ULL);
  printf("[hetero] parallel_wall_cycles=%llu  serial_sum_cycles=%llu  "
         "speedup_x1000=%llu\n",
         (unsigned long long)parallel_wall,
         (unsigned long long)sum_total,
         parallel_wall ? (unsigned long long)((sum_total * 1000ULL) /
                                              parallel_wall) : 0ULL);
  printf("[hetero] init_h0=%llu init_h1=%llu  amortized_per_job_h0=%llu  "
         "amortized_per_job_h1=%llu\n",
         (unsigned long long)g_worker_state[0].cycles_init,
         (unsigned long long)g_worker_state[1].cycles_init,
         n_h0 ? (unsigned long long)(g_worker_state[0].cycles_init / n_h0)
              : 0ULL,
         n_h1 ? (unsigned long long)(g_worker_state[1].cycles_init / n_h1)
              : 0ULL);

  if (n_ok == MERLIN_JOBS) {
    printf(">>> MERLIN HETERO RUN PASSED <<<\n");
  } else {
    printf(">>> MERLIN HETERO RUN FAILED (%d/%d) <<<\n", n_ok, MERLIN_JOBS);
  }
  sys_reboot(SYS_REBOOT_COLD);
  return n_ok == MERLIN_JOBS ? 0 : 1;
}
