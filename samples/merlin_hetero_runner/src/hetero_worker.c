/*
 * hetero_worker.c — per-hart inference worker.
 *
 * Init once → consume jobs from msgq → invoke + hash + signal completion.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hetero_worker.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/sys/printk.h>

#include "iree/base/api.h"
#include "iree/hal/api.h"
#include "iree/hal/device_group.h"
#include "iree/modules/hal/module.h"
#include "iree/vm/api.h"
#include "iree/vm/bytecode/module.h"

#include "model_info.h"

extern iree_status_t create_sample_device(iree_allocator_t host_allocator,
                                          iree_hal_device_t** out_device);

static inline uint64_t rdmcycle_w(void) {
  uint64_t c;
  __asm__ __volatile__("csrr %0, mcycle" : "=r"(c));
  return c;
}

static inline void set_vs_dirty_w(void) {
  unsigned long vs = (3UL << 9);
  __asm__ __volatile__("csrs mstatus, %0" :: "r"(vs));
}

/* FNV-1a 64 over a byte span. */
static uint64_t fnv1a64_w(const void* data, size_t len) {
  const uint8_t* p = data;
  uint64_t h = 0xcbf29ce484222325ULL;
  for (size_t i = 0; i < len; ++i) { h ^= p[i]; h *= 0x100000001b3ULL; }
  return h;
}

/* One-shot init: vm_instance, hal_device, hal_module, bytecode_module,
 * context, function resolve, pre-allocate zero-filled input.
 * Returns 0 on success. */
static int worker_init(merlin_worker_state_t* st) {
  iree_allocator_t host = iree_allocator_system();

  if (!iree_status_is_ok(iree_vm_instance_create(
          IREE_VM_TYPE_CAPACITY_DEFAULT, host, &st->vm_instance))) {
    printk("[hetero h%d] FAIL vm_instance_create\n", st->hart_id);
    return -1;
  }
  if (!iree_status_is_ok(
          iree_hal_module_register_all_types(st->vm_instance))) {
    printk("[hetero h%d] FAIL hal_register_types\n", st->hart_id);
    return -1;
  }

  if (!iree_status_is_ok(create_sample_device(host, &st->hal_device))) {
    printk("[hetero h%d] FAIL create_sample_device\n", st->hart_id);
    return -1;
  }
  if (!iree_status_is_ok(iree_hal_device_group_create_from_device(
          st->hal_device, host, &st->hal_device_group))) {
    printk("[hetero h%d] FAIL device_group_create\n", st->hart_id);
    return -1;
  }

  iree_vm_module_t* hal_module = NULL;
  if (!iree_status_is_ok(iree_hal_module_create(
          st->vm_instance, iree_hal_module_device_policy_default(),
          st->hal_device_group, IREE_HAL_MODULE_FLAG_SYNCHRONOUS,
          iree_hal_module_debug_sink_null(), host, &hal_module))) {
    printk("[hetero h%d] FAIL hal_module_create\n", st->hart_id);
    return -1;
  }

  iree_const_byte_span_t module_data = {
      .data = st->vmfb.data, .data_length = st->vmfb.length};
  iree_vm_module_t* bytecode_module = NULL;
  if (!iree_status_is_ok(iree_vm_bytecode_module_create(
          st->vm_instance, IREE_VM_BYTECODE_MODULE_FLAG_NONE, module_data,
          iree_allocator_null(), host, &bytecode_module))) {
    printk("[hetero h%d] FAIL bytecode_module_create\n", st->hart_id);
    return -1;
  }

  iree_vm_module_t* modules[] = {hal_module, bytecode_module};
  iree_status_t ctx_status = iree_vm_context_create_with_modules(
      st->vm_instance, IREE_VM_CONTEXT_FLAG_NONE, IREE_ARRAYSIZE(modules),
      &modules[0], host, &st->vm_context);
  if (!iree_status_is_ok(ctx_status)) {
    iree_status_code_t code = iree_status_code(ctx_status);
    char buf[512];
    iree_host_size_t buf_len = 0;
    iree_status_format(ctx_status, sizeof(buf), buf, &buf_len);
    buf[buf_len < sizeof(buf) ? buf_len : sizeof(buf) - 1] = '\0';
    printk("[hetero h%d] FAIL context_create code=%d msg=%s\n", st->hart_id,
           (int)code, buf);
    iree_status_ignore(ctx_status);
    return -1;
  }
  iree_vm_module_release(hal_module);
  iree_vm_module_release(bytecode_module);

  if (!iree_status_is_ok(iree_vm_context_resolve_function(
          st->vm_context,
          iree_make_cstring_view(MERLIN_ENTRY_FUNCTION_NAME),
          &st->entry_function))) {
    printk("[hetero h%d] FAIL resolve_function\n", st->hart_id);
    return -1;
  }

  /* Pre-allocate a deterministic non-zero input buffer view for input[0].
   * 2026-05-20: switched from calloc to a fp32 all-ones fill so that
   * activations entering the FC matmul are non-zero. With a zero input,
   * A·B = 0 regardless of B, and the Gemmini subspan-byte-offset bug is
   * masked (we can't distinguish correct-B from garbage-B). The dronet
   * input is f32 NCHW 1×3×112×112; we treat all inputs as f32 (matches
   * mlp_wide / dronet / dronet_with_intermediate / yolov8n in model_info.h)
   * and fill every element with 1.0f. */
  const merlin_tensor_spec_t* spec = &merlin_inputs[0];
  void* zero_buf = malloc(spec->byte_count);
  if (!zero_buf) {
    printk("[hetero h%d] FAIL malloc(%u) for input\n",
           st->hart_id, (unsigned)spec->byte_count);
    return -1;
  }
  {
    /* All inputs in this sample are fp32; fill with 1.0f. */
    float* fbuf = (float*)zero_buf;
    size_t nelem = spec->byte_count / sizeof(float);
    for (size_t i = 0; i < nelem; ++i) fbuf[i] = 1.0f;
  }
  iree_hal_buffer_params_t bparams = {
      .type = IREE_HAL_MEMORY_TYPE_DEVICE_LOCAL,
      .usage = IREE_HAL_BUFFER_USAGE_DEFAULT,
  };
  if (!iree_status_is_ok(iree_hal_buffer_view_allocate_buffer_copy(
          st->hal_device, iree_hal_device_allocator(st->hal_device),
          spec->rank, spec->shape, spec->element_type,
          IREE_HAL_ENCODING_TYPE_DENSE_ROW_MAJOR, bparams,
          iree_make_const_byte_span(zero_buf, spec->byte_count),
          &st->pre_input))) {
    free(zero_buf);
    printk("[hetero h%d] FAIL pre_input allocate\n", st->hart_id);
    return -1;
  }
  free(zero_buf);

  /* Pre-allocate input + output vm_lists ONCE. The input list is
   * permanently loaded with a retained ref to pre_input; the output
   * list is cleared (resize 0) between jobs. This eliminates ~6 heap
   * allocs/frees and 2 ref ops per dispatch. */
  if (!iree_status_is_ok(iree_vm_list_create(
          iree_vm_make_undefined_type_def(), MERLIN_INPUT_COUNT, host,
          &st->reusable_inputs))) {
    printk("[hetero h%d] FAIL reusable_inputs create\n", st->hart_id);
    return -1;
  }
  iree_hal_buffer_view_retain(st->pre_input);
  iree_vm_ref_t input_ref = iree_hal_buffer_view_move_ref(st->pre_input);
  if (!iree_status_is_ok(iree_vm_list_push_ref_move(st->reusable_inputs,
                                                    &input_ref))) {
    printk("[hetero h%d] FAIL push pre_input\n", st->hart_id);
    return -1;
  }
  if (!iree_status_is_ok(iree_vm_list_create(
          iree_vm_make_undefined_type_def(), MERLIN_OUTPUT_COUNT, host,
          &st->reusable_outputs))) {
    printk("[hetero h%d] FAIL reusable_outputs create\n", st->hart_id);
    return -1;
  }
  return 0;
}

/* One inference invocation — minimal-overhead steady state.
 *
 * Reuses pre-allocated input + output vm_lists (no heap alloc per call).
 * Hashes outputs only on the first job (job_seq==0); subsequent jobs are
 * byte-identical with zero-filled inputs so we skip d2h + hashing to keep
 * the steady-state dispatch loop tight.
 */
static int worker_invoke_once(merlin_worker_state_t* st, uint32_t job_seq,
                              uint64_t* out_cycles, uint64_t* out_hash) {
  unsigned int irq_key = 0;
  if (st->needs_vector) {
    set_vs_dirty_w();
    irq_key = irq_lock();
    set_vs_dirty_w();
  }

  uint64_t t0 = rdmcycle_w();
  iree_status_t s = iree_vm_invoke(st->vm_context, st->entry_function,
                                   IREE_VM_INVOCATION_FLAG_NONE,
                                   /*policy=*/NULL, st->reusable_inputs,
                                   st->reusable_outputs,
                                   iree_allocator_system());
  uint64_t t1 = rdmcycle_w();

  if (st->needs_vector) {
    irq_unlock(irq_key);
  }

  if (!iree_status_is_ok(s)) {
    iree_status_free(s);
    /* Reset outputs list even on failure so subsequent calls aren't
     * polluted. */
    iree_vm_list_clear(st->reusable_outputs);
    return -1;
  }

  uint64_t hash = 0;
  /* On job 0, walk outputs once to verify correctness via FNV-1a hash.
   * On jobs 1..N, skip — zero-filled inputs produce byte-identical
   * outputs (deterministic dispatch), and we already verified on job 0. */
  if (job_seq == 0) {
    hash = 0xcbf29ce484222325ULL;
    for (int i = 0; i < MERLIN_OUTPUT_COUNT; ++i) {
      iree_hal_buffer_view_t* bv =
          iree_vm_list_get_buffer_view_assign(st->reusable_outputs, i);
      if (!bv) { hash = 0; break; }
      const merlin_tensor_spec_t* spec = &merlin_outputs[i];
      void* host_buf = malloc(spec->byte_count);
      if (!host_buf) { hash = 0; break; }
      iree_status_t d2h_s = iree_hal_device_transfer_d2h(
          st->hal_device, iree_hal_buffer_view_buffer(bv), 0, host_buf,
          spec->byte_count, IREE_HAL_TRANSFER_BUFFER_FLAG_DEFAULT,
          iree_infinite_timeout());
      if (!iree_status_is_ok(d2h_s)) {
        iree_status_free(d2h_s);
        free(host_buf);
        hash = 0;
        break;
      }
      uint64_t h_i = fnv1a64_w(host_buf, spec->byte_count);
      /* 2026-05-19 diagnostic: print PER-OUTPUT hash to localize which
       * intermediate checkpoint first diverges from the ONNX-runtime
       * golden (see tools/diff_dronet_dispatches.py).                  */
      printk("[hetero h%d] out[%d] %s hash=0x%016llx bytes=%u\n",
             st->hart_id, i, spec->name, (unsigned long long)h_i,
             (unsigned)spec->byte_count);
      hash ^= h_i + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
      free(host_buf);
    }
    st->first_job_hash = hash;
  } else {
    /* Echo the job-0 hash so the per-job log shows a stable identifier
     * (we believe the output is byte-identical to job 0). */
    hash = st->first_job_hash;
  }

  /* Clear output list so the next invoke starts clean. The buffer
   * views are dropped; the underlying buffers are freed by ref count.
   * This is O(MERLIN_OUTPUT_COUNT), no heap alloc. */
  iree_vm_list_clear(st->reusable_outputs);

  *out_cycles = t1 - t0;
  *out_hash = hash;
  return 0;
}

void merlin_worker_entry(void* state, void* arg2, void* arg3) {
  (void)arg2;
  (void)arg3;
  merlin_worker_state_t* st = (merlin_worker_state_t*)state;
  int hart_now = arch_curr_cpu()->id;
  printk("[hetero h%d] worker on hart %d (expected %d), backend=%s\n",
         st->hart_id, hart_now, st->hart_id, st->backend_name);
  if (hart_now != st->hart_id) {
    printk("[hetero h%d] WARN wrong hart\n", st->hart_id);
  }

  if (st->needs_vector) set_vs_dirty_w();

  /* One-shot init. Measure for the per-hart phase summary. */
  uint64_t init_t0 = rdmcycle_w();
  if (worker_init(st) != 0) {
    /* Mark failure by leaving startup_sem ungiven; main will time out. */
    return;
  }
  uint64_t init_t1 = rdmcycle_w();
  st->cycles_init = init_t1 - init_t0;
  st->cycles_min_invoke = UINT64_MAX;
  printk("[hetero h%d] init done cycles=%llu\n",
         st->hart_id, (unsigned long long)st->cycles_init);
  k_sem_give(&st->startup_sem);

  /* Job loop. A job_id of UINT32_MAX is the shutdown sentinel. */
  for (;;) {
    merlin_job_t job;
    int rc = k_msgq_get(st->job_msgq, &job, K_FOREVER);
    if (rc != 0) continue;
    if (job.job_id == UINT32_MAX) {
      /* Drain signal; cleanup and exit. */
      break;
    }

    uint64_t cyc = 0, hash = 0;
    /* st->completed_jobs is the per-worker job sequence (0-based);
     * worker_invoke_once uses this to decide whether to hash outputs. */
    int inv_rc = worker_invoke_once(st, st->completed_jobs, &cyc, &hash);
    job.out_cycles = cyc;
    job.out_hash = hash;
    job.out_rc = inv_rc;
    job.hart_id = (uint32_t)st->hart_id;

    if (inv_rc == 0) {
      st->cycles_total_invoke += cyc;
      st->completed_jobs += 1;
      if (cyc < st->cycles_min_invoke) st->cycles_min_invoke = cyc;
      if (cyc > st->cycles_max_invoke) st->cycles_max_invoke = cyc;
    }

    /* Signal main. The job descriptor was passed by value; we report
     * back via the completion_sem and a shared in-memory result table
     * indexed by job_id (filled by main from the worker's view of the
     * job struct via the sem-take side; here we just signal). */
    if (job.completion_sem) {
      /* Stash the worker's view into a global result table for main
       * to read after sem_take. We can't write back into the caller's
       * stack-allocated job_t directly via the msgq (it was copied). */
      extern void merlin_record_job_result(uint32_t job_id,
                                           uint64_t cycles,
                                           uint64_t hash, int rc,
                                           uint32_t hart_id);
      merlin_record_job_result(job.job_id, cyc, hash, inv_rc,
                               (uint32_t)st->hart_id);
      k_sem_give(job.completion_sem);
    }
  }

  /* Per-worker cleanup. Releases happen in reverse order of acquisition;
   * however since we run to shutdown anyway, leaking is acceptable in
   * this bare-metal sample. */
}
