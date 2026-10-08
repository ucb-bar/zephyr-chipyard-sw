/*
 * hetero_worker.h — per-hart worker state for merlin_hetero_runner.
 *
 * Each worker thread:
 *   1) creates ONE iree_hal_device + iree_vm_context at startup (amortizes
 *      the ~575 M cycle setup cost),
 *   2) blocks on its own k_msgq for incoming jobs,
 *   3) reuses a pre-allocated zero-filled input across jobs.
 *
 * Per-job overhead in steady state = iree_vm_invoke + d2h + fnv1a only.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MERLIN_HETERO_WORKER_H_
#define MERLIN_HETERO_WORKER_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#include "iree/base/api.h"
#include "iree/hal/api.h"
#include "iree/vm/api.h"

/* One submitted inference. Main fills (job_id, completion_sem); worker
 * fills (cycles, hash, rc) before signalling the sem. */
typedef struct merlin_job {
  uint32_t job_id;
  uint32_t hart_id;        /* which hart actually ran it (for verification) */
  uint64_t out_cycles;
  uint64_t out_hash;
  int32_t  out_rc;
  int32_t  _pad;
  struct k_sem* completion_sem;
} merlin_job_t;

/* VMFB descriptor: pointer + length to xxd-embedded blob. */
typedef struct merlin_vmfb_blob {
  const uint8_t* data;
  uint32_t length;
} merlin_vmfb_blob_t;

/* Per-hart worker. Populated by main before k_thread_start; mutated by the
 * worker thread thereafter (do NOT touch from main while the worker is
 * live except via the msgq / completion_sem channel). */
typedef struct merlin_worker_state {
  /* configured by main */
  int hart_id;
  int needs_vector;       /* 1 if backend uses RVV/OPU → mstatus.VS=Dirty */
  const char* backend_name;
  merlin_vmfb_blob_t vmfb;
  struct k_msgq* job_msgq;
  struct k_sem startup_sem;  /* given by worker once init complete */

  /* owned by worker after init */
  iree_vm_instance_t* vm_instance;
  iree_hal_device_t* hal_device;
  iree_hal_device_group_t* hal_device_group;
  iree_vm_context_t* vm_context;
  iree_vm_function_t entry_function;
  iree_hal_buffer_view_t* pre_input;   /* zero-filled, reused per job */
  iree_vm_list_t* reusable_inputs;     /* preloaded with pre_input, never reset */
  iree_vm_list_t* reusable_outputs;    /* cleared between jobs */
  uint64_t first_job_hash;             /* hashed on job 0 only; subsequent jobs
                                          should be byte-identical and we skip
                                          d2h+hash to save ~10kcycles per job */

  /* steady-state stats (rolling) */
  uint64_t cycles_init;          /* boot/HAL/module init cost on this hart */
  uint64_t cycles_total_invoke;  /* sum of per-job invoke cycles */
  uint64_t cycles_min_invoke;
  uint64_t cycles_max_invoke;
  uint32_t completed_jobs;
} merlin_worker_state_t;

/* Worker thread entry. Pass &merlin_worker_state_t* via the first arg. */
void merlin_worker_entry(void* state, void* arg2, void* arg3);

#endif  /* MERLIN_HETERO_WORKER_H_ */
