# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rose_flight_controller)

# Reuse the TinyMPC controller from samples/drone_control (requires:
#   git -C soc/sw/xpu-rt/zephyr-chipyard-sw submodule update --init
#   samples/drone_control/tinympc  -- see README.rst).
set(TINYMPC ${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/tinympc)

if(RISCV_VECTOR)
  set(USE_CPU OFF CACHE BOOL "Enable RISCV Scalar support")
  set(USE_RVV ON  CACHE BOOL "Enable RVV support")
  set(USE_HANDOPT ON CACHE BOOL "Enable manual optimizations")
else()
  set(USE_CPU ON  CACHE BOOL "Enable RISCV Scalar support")
  set(USE_RVV OFF CACHE BOOL "Enable RVV support")
  set(USE_HANDOPT OFF CACHE BOOL "Enable manual optimizations")
endif()
set(USE_TYPE "float32" CACHE STRING "Floating point type for TinyMPC")

include(${TINYMPC}/external/matlib/matlib_header.cmake)

# State estimator selection (modular IStateEstimator; both are compiled, factory picks):
#   -DROSE_USE_EKF=0 -> complementary (quaternion Mahony + fixed-gain + lead compensation)
#   -DROSE_USE_EKF=1 -> EKF (quaternion Mahony + per-axis Kalman)
# Estimator selection (both compiled; factory picks). Default is the complementary filter,
# which currently holds the ALTITUDE loop best (its explicit vz correction beats the EKF's
# Kalman vz for control). The EKF is the robustness target (Q/R for noise) but its vz needs
# a direct velocity measurement to match; both use model-based delay compensation.
set(ROSE_USE_EKF 0 CACHE STRING "1=EKF, 0=complementary filter")
# Controller selection (modular IController; both are compiled, factory picks):
#   -DROSE_USE_PID=0 -> TinyMPC constrained MPC (default)
#   -DROSE_USE_PID=1 -> hierarchical PID cascade (ported from the old riskybird FreeRTOS firmware)
set(ROSE_USE_PID 0 CACHE STRING "1=hierarchical PID, 0=TinyMPC")
# Architecture knobs: ROSE_THREADED (1=modular threaded blocks, 0=single loop) and
# ROSE_CTRL_DIV (control runs every Nth estimator tick -> control rate = estimation / N).
set(ROSE_THREADED 0 CACHE STRING "1=threaded blocks, 0=single loop (default; see main.cpp note)")
set(ROSE_CTRL_DIV 1 CACHE STRING "control every Nth estimator tick (multi-rate)")
# Bench safety: stop driving motors this many ms after boot (0 = never). Real-HW bench
# bring-up builds with 10000 so the props auto-stop 10 s after reset. Keep 0 for flight.
set(ROSE_ACTUATE_TIMEOUT_MS 0 CACHE STRING "stop motor actuation N ms after boot (0=never)")
# Control-loop iteration count. Default 5000 suits the RoSE co-sim; on real HW the loop runs
# ~1 kHz, so raise this for a longer bench run (e.g. 120000 ~= 2 min).
set(CTRL_ITERS 5000 CACHE STRING "control-loop iteration count")
# Side-ToF wall "bumper": 1 = bring up the 4 VL53L5CX + apply bounded wall-repulsion lean.
# Adds ~12 s to boot (4x firmware upload). 0 = skip entirely (side_tof.c compiled but idle).
set(ROSE_BUMPER 0 CACHE STRING "1=enable side-ToF wall bumper, 0=disabled")

target_sources(app PRIVATE
  src/main.cpp
  src/estimator_complementary.cpp
  src/estimator_ekf.cpp
  src/estimator_factory.cpp
  src/controller_tinympc.cpp
  src/controller_pid.cpp
  src/controller_factory.cpp
  src/flightlog.c
  src/side_tof.c
  ${TINYMPC}/external/admm/src/admm_rvv.cpp)

target_compile_definitions(app PRIVATE ROSE_USE_EKF=${ROSE_USE_EKF}
  ROSE_USE_PID=${ROSE_USE_PID}
  ROSE_THREADED=${ROSE_THREADED} ROSE_CTRL_DIV=${ROSE_CTRL_DIV}
  ROSE_ACTUATE_TIMEOUT_MS=${ROSE_ACTUATE_TIMEOUT_MS} CTRL_ITERS=${CTRL_ITERS}
  ROSE_BUMPER=${ROSE_BUMPER})

target_include_directories(app PRIVATE
  ${TINYMPC}/external/admm/include
  ${TINYMPC}/external/matlib/include
  ${TINYMPC}/include
  ${TINYMPC}/examples)
