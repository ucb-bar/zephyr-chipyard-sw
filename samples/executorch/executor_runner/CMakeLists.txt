# Copyright 2023-2024 Arm Limited and/or its affiliates.
#
# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

cmake_minimum_required(VERSION 3.20)


add_compile_definitions(_POSIX_C_SOURCE=200809L)
add_compile_definitions(__POSIX_VISIBLE=200809L)

find_program(FLATC_EXECUTABLE flatc)
set(ET_DIR_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../third-party/executorch" CACHE PATH "Path to ExecuTorch dir")
set(ET_BUILD_DIR_PATH "${ET_DIR_PATH}/cmake-out" CACHE PATH "Path to ExecuTorch build dir")
set(ET_INCLUDE_PATH "${ET_DIR_PATH}/.." CACHE PATH "Path to ExecuTorch headers")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(riscv_executor_runner)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(EXECUTORCH_BUILD_EXECUTOR_RUNNER OFF CACHE BOOL "")
set(EXECUTORCH_BUILD_EXTENSION_RUNNER_UTIL ON CACHE BOOL "")
set(EXECUTORCH_BUILD_KERNELS_QUANTIZED ON CACHE BOOL "")
set(EXECUTORCH_ENABLE_LOGGING ON CACHE BOOL "")
set(EXECUTORCH_BUILD_CPUINFO OFF CACHE BOOL "")
set(EXECUTORCH_BUILD_PTHREADPOOL OFF CACHE BOOL "")
set(EXECUTORCH_BUILD_GFLAGS OFF CACHE BOOL "")

set(EXECUTORCH_BUILD_HOST_TARGETS OFF CACHE BOOL "")

set(EXECUTORCH_BUILD_XNNPACK ON CACHE BOOL "")
set(XNNPACK_ENABLE_RISCV_GEMMINI ON CACHE BOOL "Build XNNPACK with RISC-V Gemmini micro-kernels")
set(XNNPACK_ENABLE_RISCV_VECTOR ON CACHE BOOL "Build XNNPACK with RISC-V Vector micro-kernels")
set(XNNPACK_BUILD_TESTS OFF CACHE BOOL "Build XNNPack unit tests")
set(XNNPACK_BUILD_BENCHMARKS OFF CACHE BOOL "Build XNNPack benchmarks")
set(FLATCC_RTONLY ON CACHE BOOL "enable build of runtime library of flatbuffers only")
# add_definitions(-DFLATBUFFERS_NO_ABSOLUTE_PATH_RESOLUTION)



get_filename_component(ET_BUILD_DIR_PATH ${ET_BUILD_DIR_PATH} REALPATH)
get_filename_component(ET_DIR_PATH ${ET_DIR_PATH} REALPATH)
get_filename_component(ET_INCLUDE_PATH ${ET_INCLUDE_PATH} REALPATH)

# Operator-level profiling: makes the XNNPACK delegate create its runtime with
# XNN_FLAG_BASIC_PROFILING and print per-operator timings (`>>, <op>, <ticks>`)
# on delegate destroy — the ExecuTorch counterpart to modelblaster's per-kernel
# cycle CSV. Applies to the executorch/xnnpack_backend libs in the subbuild.
# Per-op profiling emits ET_LOG(">>, <op>, <cycles>") every invoke over the HTIF
# console, which is very slow on FireSim (each tohost write costs ~millions of
# cycles) and inflates the execute() measurement. Keep it ON for op-level
# characterization; build -DMB_XNN_PROFILE=OFF for a clean end-to-end timing.
option(MB_XNN_PROFILE "Enable XNNPACK per-op profiling (adds HTIF log overhead)" ON)
if(MB_XNN_PROFILE)
  add_compile_definitions(ENABLE_XNNPACK_PROFILING)
endif()

# Opt-in: TACIT / L-Trace of the (warm) method->execute() iteration. Pass
# -DMB_TACIT_TRACE_MODEL=1 and run on the TACIT spike with `--trace=l`.
# See samples/tacit/TACIT_TRACING.md.
if(MB_TACIT_TRACE_MODEL)
  add_compile_definitions(MB_TACIT_TRACE_MODEL=1)
  message(STATUS "executor_runner: TACIT model tracing enabled")
endif()

# The ExecuTorch/XNNPACK targets below are added as a plain subdirectory and do
# NOT link Zephyr's `zephyr_interface`, so they would otherwise compile with the
# SDK toolchain's default multilib (rv32 + soft-float) and fail the final link
# against Zephyr's rv64/lp64d objects with "file in wrong format". Force the
# spike_riscv64 target ABI into the subbuild. `-mcmodel=medany` matches Zephyr's
# rv64 code model and keeps R_RISCV_CALL_PLT relocations in range for the large
# executorch+XNNPACK image.
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=rv64imafdc -mabi=lp64d -mcmodel=medany")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=rv64imafdc -mabi=lp64d -mcmodel=medany")

add_subdirectory(${ET_DIR_PATH} ${ET_BUILD_DIR_PATH})



add_library(compile_commands INTERFACE)
target_include_directories(compile_commands INTERFACE $<TARGET_PROPERTY:app,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(compile_commands INTERFACE $<TARGET_PROPERTY:app,INTERFACE_COMPILE_OPTIONS>)
target_compile_definitions(compile_commands INTERFACE $<TARGET_PROPERTY:app,INTERFACE_COMPILE_DEFINITIONS>)
add_dependencies(compile_commands zephyr_interface)
add_dependencies(compile_commands zephyr_property_target)


target_link_libraries(pthreadpool PUBLIC compile_commands)

# --- Threadpool variant toggle (vanilla vs modelblaster-style spin) ---------
# XNNPACK parallelizes every operator through pthreadpool, whose wait loop
# spins PTHREADPOOL_SPIN_WAIT_ITERATIONS (=1000 here) times then falls to a
# futex/condvar wait. On bare-metal Zephyr that futex wait blocks until the
# next 100 Hz system tick (~10 ms), so each op eats ~one tick — the dominant
# cost for small models (see modelblaster/microbench/threadpool: pthreadpool
# ~13M cyc/dispatch vs a raw spin rendezvous ~20k). modelblaster's lightweight
# pool avoids this by never sleeping. We can't drop modelblaster_pool into
# XNNPACK (it only implements parallelize_1d; XNNPACK needs 30+ variants), so
# we bring modelblaster's *property* — spin instead of tick-sleep — by bumping
# the spin-iteration ceiling via the same -include override the microbench
# uses. Default OFF = vanilla pthreadpool (the baseline to compare against).
option(MB_XNNPACK_SPIN_THREADPOOL
       "Use spin-wait pthreadpool (modelblaster-style, no condvar/tick sleep)" OFF)
if(MB_XNNPACK_SPIN_THREADPOOL)
  target_compile_options(pthreadpool PRIVATE
    -include "${CMAKE_CURRENT_SOURCE_DIR}/../../../modelblaster/microbench/threadpool/spin_overrides/spin_inject.h")
  message(STATUS "executor_runner: pthreadpool variant = SPIN (modelblaster-style)")
else()
  message(STATUS "executor_runner: pthreadpool variant = vanilla (condvar/tick fallback)")
endif()





target_link_libraries(app PRIVATE
  # "-Wl,--whole-archive"
  extension_runner_util executorch executorch_core quantized_ops_lib portable_ops_lib quantized_kernels portable_kernels
  # microkernels-prod was renamed xnnpack-microkernels-prod in the XNNPACK
  # used by ExecuTorch 1.0.1 (ucb-bar torch-1.0.0-bump fork).
  xnnpack_backend XNNPACK xnnpack-microkernels-prod
  # "-Wl,--no-whole-archive"`
)

target_include_directories(app PRIVATE ${ET_INCLUDE_PATH} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
# MB_MULTI_MODEL=ON bakes a registry of N models (models_pte.c, generated by
# model/gen_multi_pte.py) and the runner loops over them under one boot /
# FireSim infrasetup. Default (off) is the single-model model_pte.c path.
if(MB_MULTI_MODEL)
    target_sources(app PRIVATE riscv_executor_runner.cpp models_pte.c)
    target_compile_definitions(app PRIVATE MB_MULTI_MODEL=1)
    message(STATUS "modelblaster ET runner: MB_MULTI_MODEL (batched multi-.pte)")
else()
    target_sources(app PRIVATE riscv_executor_runner.cpp model_pte.c)
endif()

# ET method_allocation_pool size (MiB). Default 96; bump for the 256MB-io
# default sizing (needs ram0 bumped too, via -DEXTRA_DTC_OVERLAY_FILE=<ram0>).
if(DEFINED MB_ET_POOL_MB)
    target_compile_definitions(app PRIVATE MB_ET_POOL_MB=${MB_ET_POOL_MB})
    message(STATUS "modelblaster ET runner: method pool = ${MB_ET_POOL_MB} MiB")
endif()

# Number of rdcycle-bracketed execute() iterations per model (default 5 in the
# .cpp; iter 0 is cold, the rest warm). Lower for sweeps to cut FireSim time.
if(DEFINED MB_EXEC_ITERS)
    target_compile_definitions(app PRIVATE MB_EXEC_ITERS=${MB_EXEC_ITERS})
    message(STATUS "modelblaster ET runner: exec iters = ${MB_EXEC_ITERS}")
endif()
zephyr_linker_sources(DATA_SECTIONS  custom-sections.ld)

