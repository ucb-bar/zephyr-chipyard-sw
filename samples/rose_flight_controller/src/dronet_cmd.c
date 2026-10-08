/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * See dronet_cmd.h. A spinlock rather than a mutex: the critical section is a small copy, and the
 * control loop (prio 0) must never block behind the lower-priority inference thread.
 */
#include "dronet_cmd.h"

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

static struct k_spinlock lock;
static struct dronet_cmd latest;

void dronet_cmd_publish(int8_t steer_q, int8_t coll_q, int64_t t_capture_ms)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	latest.seq++;
	latest.t_ms = t_capture_ms;
	latest.steer_q = steer_q;
	latest.coll_q = coll_q;
	latest.valid = true;
	k_spin_unlock(&lock, key);
}

void dronet_cmd_invalidate(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	latest.valid = false;
	k_spin_unlock(&lock, key);
}

bool dronet_cmd_latest(struct dronet_cmd *out)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	*out = latest;
	k_spin_unlock(&lock, key);
	return out->seq != 0 && out->valid;
}
