/*
 * Copyright (c) 2025, Dima Nikiforov <vnikiforov@berkeley.edu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */



#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/timing/timing.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include "admm.hpp"
#include "problem_data/quadrotor_50hz_params_constrained.hpp"
// #include "problem_data/quadrotor_50hz_params_custom.hpp"
// #include "problem_data/quadrotor_50hz_params_wide.hpp"
// #include "problem_data/quadrotor_50hz_params_hawk.hpp"
// #include "problem_data/quadrotor_50hz_params_racer.hpp"
// #include "problem_data/quadrotor_50hz_params_custom_tweak.hpp"
// #include "problem_data/quadrotor_50hz_params.hpp"
#include "glob_opts.hpp"

#define NUM_DRONES    4
#define STACK_SIZE    (8192*2)
#define NSTATES       12
#define NACTIONS      4

/* 4-byte header marker */
#define HDR0 0xDE
#define HDR1 0xAD
#define HDR2 0xBE
#define HDR3 0xEF
static const uint8_t HEADER[4] = { HDR0, HDR1, HDR2, HDR3 };

/* Zephyr/UART */
static const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
static struct k_mutex      uart_mutex;

/* TinyMPC globals */
TinyCache    caches[NUM_DRONES];
TinyWorkspace works[NUM_DRONES];
TinySettings  settings[NUM_DRONES];
TinySolver    solvers[NUM_DRONES];

K_THREAD_STACK_ARRAY_DEFINE(drone_stacks, NUM_DRONES, STACK_SIZE);
struct k_thread drone_threads[NUM_DRONES];
k_tid_t drone_thread_ids[NUM_DRONES];

/* Task state + flag */
struct DroneTask {
    float    obs[NSTATES];
    atomic_t new_state;
};
static struct DroneTask drone_tasks[NUM_DRONES];

/* Helper: send binary response [HEADER][id][4 floats] */
static void send_response(uint8_t id, float *u, uint32_t ns)
{
    k_mutex_lock(&uart_mutex, K_FOREVER);
    for (int i = 0; i < 4; i++) {
        uart_poll_out(uart0, HEADER[i]);
    }
    uart_poll_out(uart0, id);
    for (int j = 0; j < NACTIONS; j++) {
        uint8_t *pb = (uint8_t *)&u[j];
        for (int b = 0; b < sizeof(float); b++) {
            uart_poll_out(uart0, pb[b]);
        }
    }
    /* 4-byte int32 ns (little-endian) */
    uint8_t *ns_bytes = (uint8_t *)&ns;
    for (int b = 0; b < sizeof(ns); b++) uart_poll_out(uart0, ns_bytes[b]);
    k_mutex_unlock(&uart_mutex);
}

/* MPC worker thread: solve only on new state */
void drone_worker(void *id_ptr, void *, void *)
{
    int id = (int)(uintptr_t)id_ptr;
    enable_vector_operations();

    /* TinyMPC init */
    TinySolver    *solver = &solvers[id];
    TinyCache     *cache  = &caches[id];
    TinyWorkspace *work   = &works[id];
    TinySettings  *setting= &settings[id];

    solver->cache    = cache;
    solver->work     = work;
    solver->settings = setting;
    tiny_init(solver);

    init_VectorNx(&work->x1);
    init_VectorNx(&work->x2);
    init_VectorNx(&work->x3);
    init_VectorNu(&work->u1);
    init_VectorNu(&work->u2);

    cache->rho = rho_value;
    matsetv(cache->Kinf.data, Kinf_data, cache->Kinf.outer, cache->Kinf.inner);
    transpose(cache->Kinf.data, cache->KinfT.data, NINPUTS, NSTATES);
    matsetv(cache->Pinf.data, Pinf_data, cache->Pinf.outer, cache->Pinf.inner);
    transpose(cache->Pinf.data, cache->PinfT.data, NSTATES, NSTATES);
    matsetv(cache->Quu_inv.data, Quu_inv_data, cache->Quu_inv.outer, cache->Quu_inv.inner);
    matsetv(cache->AmBKt.data, AmBKt_data, cache->AmBKt.outer, cache->AmBKt.inner);
    transpose(cache->AmBKt.data, cache->AmBKtT.data, NSTATES, NSTATES);
    matsetv(cache->coeff_d2p.data, coeff_d2p_data, cache->coeff_d2p.outer, cache->coeff_d2p.inner);

    matsetv(work->Adyn.data, Adyn_data, work->Adyn.outer, work->Adyn.inner);
    transpose(work->Adyn.data, work->AdynT.data, NSTATES, NSTATES);
    matsetv(work->Bdyn.data, Bdyn_data, work->Bdyn.outer, work->Bdyn.inner);
    transpose(work->Bdyn.data, work->BdynT.data, NSTATES, NINPUTS);
    matsetv(work->Q.data, Q_data, work->Q.outer, work->Q.inner);
    matsetv(work->R.data, R_data, work->R.outer, work->R.inner);

    matset(work->u_min.data, -0.583, work->u_min.outer, work->u_min.inner);
    matset(work->u_max.data, 1 - 0.583, work->u_max.outer, work->u_max.inner);

    // HAWK
    matset(work->u_min.data, -0.0625, work->u_min.outer, work->u_min.inner);
    matset(work->u_max.data, 1 - 0.0625, work->u_max.outer, work->u_max.inner);

    // // RACER
    // matset(work->u_min.data, -0.2398f,
    //    work->u_min.outer, work->u_min.inner);
    // matset(work->u_max.data,  0.7602f,
    //    work->u_max.outer, work->u_max.inner);

    matset(work->x_min.data, -5, work->x_min.outer, work->x_min.inner);
    matset(work->x_max.data, 5, work->x_max.outer, work->x_max.inner);

    float Xref_origin[NSTATES] = {0};
    for (int j = 0; j < NHORIZON; j++) {
        matsetv(work->Xref.vector[j], Xref_origin, 1, NSTATES);
    }

    /* Worker loop: wait for new_state event */
    float current_state[NSTATES];
    // send_response((uint8_t)id, work->u.vector[0]);

    volatile timing_t start_counter, end_counter;
    uint64_t cycles;
    uint32_t ns;
    while (1) {
        /* spin until a new state arrives */
        // DISABLE fow power meas
        while (atomic_cas(&drone_tasks[id].new_state, 1, 0) != 1) {
            // k_yield();
        }
        /* copy fresh state */
        memcpy(current_state, drone_tasks[id].obs, sizeof(current_state));

        /* solve and send */
        matsetv(work->x.vector[0], current_state, 1, NSTATES);
        matset(work->y.data, 0.0, work->y.outer, work->y.inner);
        matset(work->g.data, 0.0, work->g.outer, work->g.inner);

        start_counter = timing_counter_get();
        tiny_solve(solver);
        end_counter = timing_counter_get();
        
        cycles = timing_cycles_get(&start_counter, &end_counter);
        ns = (uint32_t) timing_cycles_to_ns(cycles);

        float u_out[NACTIONS];
        for (int i = 0; i < NACTIONS; i++) {
            u_out[i] = work->u.vector[0][i];
        }
        send_response((uint8_t)id, u_out, ns);
    }
}

    unsigned long start_rd, end_rd, cycles_rd;

/* Spawn worker threads (suspended), then pin, then start */
static void spawn_drone_workers(void)
{
    for (int i = 0; i < NUM_DRONES; i++) {
        atomic_set(&drone_tasks[i].new_state, 0);
        drone_thread_ids[i] = k_thread_create(
            &drone_threads[i], drone_stacks[i], STACK_SIZE,
            drone_worker, (void *)(uintptr_t)i, NULL, NULL,
            K_PRIO_PREEMPT(1), 0, K_FOREVER);
        k_thread_cpu_pin(drone_thread_ids[i], (i + 5) % CONFIG_MP_MAX_NUM_CPUS);
    }
    for (int i = 0; i < NUM_DRONES; i++) {
        k_thread_start(drone_thread_ids[i]);
    }
}

int main(void)
{
    if (!device_is_ready(uart0)) {
        return -1;
    }
    k_mutex_init(&uart_mutex);
    enable_vector_operations();
    //initialize atomics
    for (int i = 0; i < NUM_DRONES; i++) {
        atomic_set(&drone_tasks[i].new_state, 0);
    }
    spawn_drone_workers();

    size_t match_idx = 0;
    uint8_t rx;

    float dummy_actions[NACTIONS] = {0.0f, 1.0f, 2.0f, 3.0f};

    while (1) {
        /* sync header */
        while (match_idx < sizeof(HEADER)) {
            if (uart_poll_in(uart0, &rx) == 0) {
                match_idx = (rx == HEADER[match_idx]) ? match_idx + 1
                           : ((rx == HEADER[0]) ? 1 : 0);
            }
        }
        /* read num_drones */
        uint8_t num_drones;
        while (uart_poll_in(uart0, &rx) != 0) {}
        num_drones = rx;
        if (num_drones > NUM_DRONES) {
            size_t skip = (size_t)num_drones * NSTATES * sizeof(float);
            for (size_t i = 0; i < skip; i++) {
                while (uart_poll_in(uart0, &rx) != 0) {}
            }
            match_idx = 0;
            continue;
        }
        /* read and asynchronously store states */
        for (uint8_t id = 0; id < num_drones; id++) {
            for (int j = 0; j < NSTATES; j++) {
                float value;
                uint8_t *pv = (uint8_t *)&value;
                for (int b = 0; b < sizeof(float); b++) {
                    while (uart_poll_in(uart0, &rx) != 0) {}
                    pv[b] = rx;
                }
                drone_tasks[id].obs[j] = value;
            }
            atomic_set(&drone_tasks[id].new_state, 1);
        }
        match_idx = 0;
    }

    return 0;
}
