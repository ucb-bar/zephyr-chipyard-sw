/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Target-resident XRCE-DDS broker. Speaks the agent side of the protocol on
 * the loopback transport defined in transport_loopback.h.
 *
 * Multi-session: one broker thread serves all LOOPBACK_MAX_SESSIONS clients
 * via k_poll over their input queues. Topic-name-based routing fans out
 * WRITE_DATA from a publisher on session A to subscribed datareaders on any
 * session (including A itself). The broker thread runs as a single
 * Zephyr thread regardless of how many sessions are active.
 */
#ifndef MICRO_ROS_LOCAL_BROKER_H
#define MICRO_ROS_LOCAL_BROKER_H

/* Spawn the broker thread, unpinned (lets the scheduler place it). */
void broker_start(void);

/* Spawn the broker thread pinned to a specific hart. Useful in SMP builds
 * to keep it off the harts running per-node executors.  Pass -1 to leave
 * it floating (equivalent to broker_start()).  Requires CONFIG_SCHED_CPU_MASK=y. */
void broker_start_pinned(int cpu);

#endif
