// Copyright 2026 The IREE Authors
//
// Licensed under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

// NOTE: must be first to ensure that we can define settings for all includes.
#include "iree/base/threading/thread_impl.h"

#if defined(IREE_PLATFORM_ZEPHYR)

#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "iree/base/internal/atomics.h"
#include "iree/base/threading/call_once.h"
#include "iree/base/threading/notification.h"
#include "iree/base/threading/thread.h"

// Zephyr requires statically-sized stacks. We pre-allocate a fixed pool of
// stacks at module scope; iree_thread_create() claims a slot from the pool.
// Sized for IREE's task executor: one worker per Shuttle hart on this
// bitstream is 2, the upstream task scheduler in long-running setups may
// spawn a few more transient threads. 8 covers either case with headroom.
#ifndef IREE_ZEPHYR_THREAD_POOL_SIZE
#define IREE_ZEPHYR_THREAD_POOL_SIZE 8
#endif
#ifndef IREE_ZEPHYR_THREAD_STACK_SIZE
#define IREE_ZEPHYR_THREAD_STACK_SIZE (32 * 1024)
#endif

K_THREAD_STACK_ARRAY_DEFINE(iree_zephyr_thread_stacks,
                            IREE_ZEPHYR_THREAD_POOL_SIZE,
                            IREE_ZEPHYR_THREAD_STACK_SIZE);

// Per-slot ownership: 0 = free, 1 = in use. Single global lock guards
// allocation since create/release are infrequent.
static iree_atomic_int32_t iree_zephyr_thread_slot_used[IREE_ZEPHYR_THREAD_POOL_SIZE];

struct iree_thread_t {
  iree_atomic_ref_count_t ref_count;
  iree_allocator_t allocator;

  /* Zephyr's optional CONFIG_THREAD_NAME exposes CONFIG_THREAD_MAX_NAME_LEN
   * (only when set), so we just hardcode a 32-byte slot which is comfortably
   * larger than any IREE-emitted name (IREE truncates to <= 32). */
  char name[32];

  // Zephyr thread storage. `slot` indexes into the static stack array.
  struct k_thread thread_data;
  int slot;

  iree_thread_entry_t entry;
  void* entry_arg;

  // Suspended-thread coordination: when create_suspended=true the thread is
  // launched but waits on `suspend_sem` before invoking entry().
  struct k_sem suspend_sem;
  bool is_suspended;
};

static int iree_zephyr_claim_slot(void) {
  for (int i = 0; i < IREE_ZEPHYR_THREAD_POOL_SIZE; ++i) {
    int32_t expected = 0;
    if (iree_atomic_compare_exchange_strong(
            &iree_zephyr_thread_slot_used[i], &expected, 1,
            iree_memory_order_acq_rel, iree_memory_order_relaxed)) {
      return i;
    }
  }
  return -1;
}

static void iree_zephyr_release_slot(int slot) {
  if (slot >= 0 && slot < IREE_ZEPHYR_THREAD_POOL_SIZE) {
    iree_atomic_store(&iree_zephyr_thread_slot_used[slot], 0,
                      iree_memory_order_release);
  }
}

// Map IREE priority class -> Zephyr priority value. Zephyr semantics:
// lower number = higher priority; preemptible threads have priority >= 0.
static int iree_zephyr_priority_for_class(
    iree_thread_priority_class_t priority_class) {
  // We stay in the preemptible range (>= 0). Default is 5 (= NORMAL).
  switch (priority_class) {
    case IREE_THREAD_PRIORITY_CLASS_LOWEST:
      return 14;
    case IREE_THREAD_PRIORITY_CLASS_LOW:
      return 9;
    case IREE_THREAD_PRIORITY_CLASS_NORMAL:
      return 5;
    case IREE_THREAD_PRIORITY_CLASS_HIGH:
      return 2;
    case IREE_THREAD_PRIORITY_CLASS_HIGHEST:
      return 0;
    default:
      return 5;
  }
}

static void iree_zephyr_thread_entry(void* p1, void* p2, void* p3) {
  iree_thread_t* thread = (iree_thread_t*)p1;
  (void)p2;
  (void)p3;

#if defined(CONFIG_THREAD_NAME)
  k_thread_name_set(k_current_get(), thread->name);
#endif

  // Wait for resume if started in suspended state.
  if (thread->is_suspended) {
    k_sem_take(&thread->suspend_sem, K_FOREVER);
    thread->is_suspended = false;
  }

  iree_thread_entry_t entry = thread->entry;
  void* entry_arg = thread->entry_arg;
  thread->entry = NULL;
  thread->entry_arg = NULL;

  // Run the user entry. Zephyr threads exit by returning from this function.
  (void)entry(entry_arg);
}

iree_status_t iree_thread_create(iree_thread_entry_t entry, void* entry_arg,
                                 iree_thread_create_params_t params,
                                 iree_allocator_t allocator,
                                 iree_thread_t** out_thread) {
  *out_thread = NULL;

  iree_thread_t* thread = NULL;
  IREE_RETURN_IF_ERROR(
      iree_allocator_malloc(allocator, sizeof(*thread), (void**)&thread));
  iree_atomic_ref_count_init(&thread->ref_count);
  thread->allocator = allocator;
  thread->entry = entry;
  thread->entry_arg = entry_arg;
  iree_strncpy_s(thread->name, sizeof(thread->name), params.name.data,
                 iree_min(params.name.size, sizeof(thread->name) - 1));
  thread->is_suspended = params.create_suspended;
  k_sem_init(&thread->suspend_sem, 0, 1);

  thread->slot = iree_zephyr_claim_slot();
  if (thread->slot < 0) {
    iree_allocator_free(allocator, thread);
    return iree_make_status(
        IREE_STATUS_RESOURCE_EXHAUSTED,
        "iree_thread pool exhausted (max %d concurrent threads)",
        IREE_ZEPHYR_THREAD_POOL_SIZE);
  }

  int prio = iree_zephyr_priority_for_class(params.priority_class);

  // K_NO_WAIT starts the thread immediately. The `is_suspended` flag in
  // iree_zephyr_thread_entry handles user-visible suspension via the sem.
  k_tid_t tid = k_thread_create(
      &thread->thread_data, iree_zephyr_thread_stacks[thread->slot],
      IREE_ZEPHYR_THREAD_STACK_SIZE, iree_zephyr_thread_entry, thread, NULL,
      NULL, prio, 0, K_NO_WAIT);
  if (tid == NULL) {
    iree_zephyr_release_slot(thread->slot);
    iree_allocator_free(allocator, thread);
    return iree_make_status(IREE_STATUS_INTERNAL,
                            "k_thread_create returned NULL");
  }

  *out_thread = thread;
  return iree_ok_status();
}

static void iree_thread_delete(iree_thread_t* thread) {
  // Make sure the thread isn't waiting on resume — wake it so it can exit.
  iree_thread_resume(thread);
  // Join terminates only if the thread actually ran to completion. The IREE
  // task executor's worker loop exits after a stop signal, so this typically
  // returns quickly. For long-running threads the user must signal first.
  k_thread_join(&thread->thread_data, K_FOREVER);
  iree_zephyr_release_slot(thread->slot);
  iree_allocator_free(thread->allocator, thread);
}

void iree_thread_retain(iree_thread_t* thread) {
  if (thread) iree_atomic_ref_count_inc(&thread->ref_count);
}

void iree_thread_release(iree_thread_t* thread) {
  if (thread && iree_atomic_ref_count_dec(&thread->ref_count) == 1) {
    iree_thread_delete(thread);
  }
}

uintptr_t iree_thread_id(iree_thread_t* thread) {
  return (uintptr_t)&thread->thread_data;
}

// Priority override and affinity hints are best-effort on Zephyr. We
// honor priority_class via k_thread_priority_set when an override begins
// and reset to NORMAL when it ends. Affinity (when SMP) routes through
// k_thread_cpu_pin if available; otherwise it's a no-op.
struct iree_thread_override_t {
  iree_thread_t* thread;
  iree_thread_priority_class_t base_class;
};

iree_thread_override_t* iree_thread_priority_class_override_begin(
    iree_thread_t* thread, iree_thread_priority_class_t priority_class) {
  iree_thread_override_t* override = NULL;
  if (iree_status_is_ok(iree_allocator_malloc(
          thread->allocator, sizeof(*override), (void**)&override))) {
    override->thread = thread;
    override->base_class = IREE_THREAD_PRIORITY_CLASS_NORMAL;
    k_thread_priority_set(&thread->thread_data,
                          iree_zephyr_priority_for_class(priority_class));
  }
  return override;
}

void iree_thread_override_end(iree_thread_override_t* override_token) {
  if (!override_token) return;
  k_thread_priority_set(&override_token->thread->thread_data,
                        iree_zephyr_priority_for_class(override_token->base_class));
  iree_allocator_free(override_token->thread->allocator, override_token);
}

void iree_thread_request_affinity(iree_thread_t* thread,
                                  iree_thread_affinity_t affinity) {
#if defined(CONFIG_SCHED_CPU_MASK) && defined(CONFIG_SMP)
  if (affinity.id_assigned) {
    (void)k_thread_cpu_pin(&thread->thread_data, (int)affinity.id);
  }
#else
  (void)thread;
  (void)affinity;
#endif
}

void iree_thread_resume(iree_thread_t* thread) {
  if (thread && thread->is_suspended) {
    thread->is_suspended = false;
    k_sem_give(&thread->suspend_sem);
  }
}

void iree_thread_join(iree_thread_t* thread) {
  if (thread) {
    k_thread_join(&thread->thread_data, K_FOREVER);
  }
}

void iree_thread_yield(void) { k_yield(); }

#endif  // IREE_PLATFORM_ZEPHYR
