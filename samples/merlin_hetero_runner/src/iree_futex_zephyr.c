// Copyright 2026 The IREE Authors
//
// Licensed under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// Zephyr emulation of the iree_futex_* API.
//
// Zephyr has no kernel futex syscall, so we provide a userspace emulation
// backed by a fixed-size pool of (k_mutex, k_condvar) bucket pairs hashed by
// the futex word's address. The implementation mirrors the classic
// "park-on-condvar with double-check inside the mutex" pattern used by
// other lock-free libraries on platforms without native futex support.
//
// Lost-wakeup avoidance:
//   - iree_futex_wait acquires the bucket mutex, re-reads *address, and
//     only descends into k_condvar_wait if it still equals expected_value.
//     k_condvar_wait atomically releases the mutex and waits.
//   - iree_futex_wake acquires the bucket mutex (so it cannot race the
//     read in wait), then signals/broadcasts. After release, woken waiters
//     reacquire the mutex and re-check *address.
//
// Bucket-pool size is set high enough that contention is rare; collisions
// produce spurious wakes (waiters re-check and possibly wait again),
// which is correct but slightly less efficient.

#include "iree/base/target_platform.h"

#if defined(IREE_PLATFORM_ZEPHYR)

#include <stdatomic.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "iree/base/threading/futex.h"

#ifndef IREE_FUTEX_ZEPHYR_BUCKET_COUNT
// Must be a power of two so the hash reduction is a single mask.
#define IREE_FUTEX_ZEPHYR_BUCKET_COUNT 32
#endif

typedef struct {
  struct k_mutex lock;
  struct k_condvar cv;
} iree_futex_zephyr_bucket_t;

static iree_futex_zephyr_bucket_t
    iree_futex_zephyr_buckets[IREE_FUTEX_ZEPHYR_BUCKET_COUNT];
static atomic_int iree_futex_zephyr_initialized = 0;

static void iree_futex_zephyr_ensure_initialized(void) {
  // Double-checked init. The first thread to flip the flag from 0 -> 1 does
  // the bucket init; subsequent threads pick up the published state.
  int expected = 0;
  if (atomic_compare_exchange_strong(&iree_futex_zephyr_initialized, &expected,
                                     1)) {
    for (int i = 0; i < IREE_FUTEX_ZEPHYR_BUCKET_COUNT; ++i) {
      k_mutex_init(&iree_futex_zephyr_buckets[i].lock);
      k_condvar_init(&iree_futex_zephyr_buckets[i].cv);
    }
    atomic_store(&iree_futex_zephyr_initialized, 2);
  } else {
    // Wait for the racing initializer to finish publishing.
    while (atomic_load(&iree_futex_zephyr_initialized) != 2) {
      k_yield();
    }
  }
}

static iree_futex_zephyr_bucket_t* iree_futex_zephyr_bucket_for(void* address) {
  // FNV-style folding of the address bits. The bucket count is a power of
  // two so the final mask is cheap.
  uintptr_t a = (uintptr_t)address;
  uintptr_t h = a ^ (a >> 17) ^ (a >> 31);
  return &iree_futex_zephyr_buckets[h & (IREE_FUTEX_ZEPHYR_BUCKET_COUNT - 1)];
}

// Convert IREE absolute-deadline (nanoseconds since epoch) to a Zephyr
// k_timeout_t. IREE_TIME_INFINITE_FUTURE maps to K_FOREVER.
static k_timeout_t iree_futex_zephyr_timeout(iree_time_t deadline_ns) {
  if (deadline_ns == IREE_TIME_INFINITE_FUTURE) return K_FOREVER;
  // Compute relative timeout from current uptime in nanoseconds.
  int64_t now_ns = (int64_t)k_uptime_get() * 1000000LL;
  int64_t rel_ns = (int64_t)deadline_ns - now_ns;
  if (rel_ns <= 0) return K_NO_WAIT;
  return K_NSEC(rel_ns);
}

iree_status_code_t iree_futex_wait_impl_zephyr(void* address,
                                               uint32_t expected_value,
                                               iree_time_t deadline_ns) {
  iree_futex_zephyr_ensure_initialized();
  iree_futex_zephyr_bucket_t* b = iree_futex_zephyr_bucket_for(address);
  k_mutex_lock(&b->lock, K_FOREVER);

  // Double-check inside the bucket lock so we don't miss a concurrent wake.
  uint32_t observed = atomic_load((_Atomic uint32_t*)address);
  if (observed != expected_value) {
    k_mutex_unlock(&b->lock);
    return IREE_STATUS_UNAVAILABLE;
  }

  int rc = k_condvar_wait(&b->cv, &b->lock,
                          iree_futex_zephyr_timeout(deadline_ns));
  k_mutex_unlock(&b->lock);

  if (rc == -EAGAIN) return IREE_STATUS_DEADLINE_EXCEEDED;
  return IREE_STATUS_OK;
}

void iree_futex_wake_impl_zephyr(void* address, int32_t count) {
  iree_futex_zephyr_ensure_initialized();
  iree_futex_zephyr_bucket_t* b = iree_futex_zephyr_bucket_for(address);
  k_mutex_lock(&b->lock, K_FOREVER);
  if (count >= IREE_ALL_WAITERS || count > 1) {
    k_condvar_broadcast(&b->cv);
  } else if (count == 1) {
    k_condvar_signal(&b->cv);
  }
  k_mutex_unlock(&b->lock);
}

#endif  // IREE_PLATFORM_ZEPHYR
