# SPDX-License-Identifier: Apache-2.0
#
# Integrated flight-controller + DroNet build for the riskybird v3 FPGA Rocket
# COMBINED core RocketArty200TDroneGemminiSaturnFp16At35Config.
#
# Combines, in ONE Zephyr binary flashed to ONE hart:
#   * the full flight controller (samples/rose_flight_controller) -- PID control
#     loop in main() at high (prio 0) priority, sensors, estimator, camera/ToF
#     background threads; and
#   * DroNet inference (the 13.31 fps integrated Gemmini+Saturn build,
#     modelblaster riskybird-fps-integrated @ 10b50ba: NHWC + conv0-pool fusion +
#     rvv_seg relayouts, grayscale, bit-exact) driven by src/dronet_thread.c on a
#     LOW-priority (prio 13) background thread.
#
# Build (see scripts/build_fc_dronet.sh for the canonical invocation):
#   west build -p -b chipyard_riscv64 samples/rose_fc_dronet -- \
#     -DMB_REPO=<abs path to a modelblaster checkout @10b50ba/equivalent> \
#     -DMODEL_DIR=<MB_REPO>/examples/dronet/int8/generated_gray/hetero_tiled_rvvadd \
#     -DMODELBLASTER_KERNEL_CFLAGS="-march=rv64imac_zve64x;-mabi=lp64;..." \
#     -DEXTRA_CPPFLAGS="-DROSE_MOTORS_INHIBIT=1 -DROSE_PROFILE=1"

cmake_minimum_required(VERSION 3.20.0)

# ---------------------------------------------------------------------------
# Locations (FC must be known before find_package so its dts/bindings -- e.g.
# riskybird,pwm-motors.yaml -- are on the binding search path via DTS_ROOT).
# ---------------------------------------------------------------------------
get_filename_component(FC ${CMAKE_CURRENT_SOURCE_DIR}/../rose_flight_controller ABSOLUTE)
get_filename_component(TINYMPC ${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/tinympc ABSOLUTE)
list(APPEND DTS_ROOT ${FC})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rose_fc_dronet)

# DroNet model + modelblaster runtime. MB_REPO/MODEL_DIR default to this
# worktree's own modelblaster submodule but are meant to be passed in by
# build_fc_dronet.sh so the exact 10b50ba integrated files are used.
if(NOT DEFINED MB_REPO)
  set(MB_REPO ${CMAKE_CURRENT_SOURCE_DIR}/../../modelblaster)
endif()
get_filename_component(MB_REPO "${MB_REPO}" ABSOLUTE)
if(NOT DEFINED MODEL_DIR)
  set(MODEL_DIR ${MB_REPO}/examples/dronet/int8/generated_gray/hetero_tiled_rvvadd)
endif()
get_filename_component(MODEL_DIR "${MODEL_DIR}" ABSOLUTE)

foreach(_f model.c kernels.c weights.c buffers.c model.h kernels.h test_io.h test_io.S)
  if(NOT EXISTS "${MODEL_DIR}/${_f}")
    message(FATAL_ERROR "MODEL_DIR='${MODEL_DIR}' missing ${_f}. "
                        "Run scripts/gen_dronet_model.sh or pass -DMODEL_DIR=.")
  endif()
endforeach()
message(STATUS "rose_fc_dronet: FC=${FC}")
message(STATUS "rose_fc_dronet: MODEL_DIR=${MODEL_DIR}")

# ---------------------------------------------------------------------------
# TinyMPC matlib: force SCALAR (the At35 core enables V for the DroNet kernels,
# but V codegen must stay confined to kernels.c -- see CONFIG_RISCV_V_KERNEL_ONLY.
# This build uses the lighter PID controller anyway, so TinyMPC is scalar like
# the original FullDDR bring-up.)
# ---------------------------------------------------------------------------
set(USE_CPU     ON  CACHE BOOL "" FORCE)
set(USE_RVV     OFF CACHE BOOL "" FORCE)
set(USE_HANDOPT OFF CACHE BOOL "" FORCE)
set(USE_TYPE "float32" CACHE STRING "" FORCE)
include(${TINYMPC}/external/matlib/matlib_header.cmake)

# ---------------------------------------------------------------------------
# FC build knobs (defaults chosen for the At35 bench build). ROSE_USE_PID=1 is
# the light hierarchical PID; motors inhibited / profiling on come via
# -DEXTRA_CPPFLAGS. Camera off in Phase 1 (turned on for Phase 2).
# ---------------------------------------------------------------------------
set(ROSE_USE_EKF 1 CACHE STRING "1=EKF, 0=complementary")
set(ROSE_USE_PID 1 CACHE STRING "1=hierarchical PID, 0=TinyMPC")
set(ROSE_THREADED 0 CACHE STRING "1=threaded blocks, 0=single loop")
set(ROSE_CTRL_DIV 1 CACHE STRING "control every Nth estimator tick")
set(ROSE_ACTUATE_TIMEOUT_MS 0 CACHE STRING "stop actuation N ms after boot (0=never)")
set(CTRL_ITERS 0 CACHE STRING "control-loop iteration count (0=forever)")
set(ROSE_BUMPER 0 CACHE STRING "1=side-ToF wall bumper")
set(ROSE_BUMPER_READDRESS_ONLY 0 CACHE STRING "1=re-address sides only")
set(ROSE_FLOW 0 CACHE STRING "1=PMW3901 optical flow")
set(ROSE_BARO 1 CACHE STRING "1=BMP388 baro altitude fusion")

# ---------------------------------------------------------------------------
# Sources: the full FC + the DroNet model/kernels/runtime + the DroNet thread.
# ---------------------------------------------------------------------------
target_sources(app PRIVATE
  # ---- flight controller (from samples/rose_flight_controller) ----
  ${FC}/src/main.cpp
  ${FC}/src/estimator_complementary.cpp
  ${FC}/src/estimator_ekf.cpp
  ${FC}/src/estimator_factory.cpp
  ${FC}/src/controller_tinympc.cpp
  ${FC}/src/controller_pid.cpp
  ${FC}/src/controller_factory.cpp
  ${FC}/src/flightlog.c
  ${FC}/src/telem_uart.c
  ${FC}/src/camera_dma.c
  ${FC}/src/side_tof.c
  ${FC}/src/flow.c
  ${FC}/src/pmw3901.c
  ${TINYMPC}/external/admm/src/admm_rvv.cpp
  # ---- DroNet inference (generated) + the background thread ----
  src/dronet_thread.c
  ${MODEL_DIR}/model.c
  ${MODEL_DIR}/kernels.c
  ${MODEL_DIR}/weights.c
  ${MODEL_DIR}/buffers.c
  ${MODEL_DIR}/test_io.S
)
set_source_files_properties(${MODEL_DIR}/test_io.S PROPERTIES LANGUAGE ASM)

# DroNet weights push .sdata past the gp-relative window; disable relaxation
# (same treatment the modelblaster harness uses).
zephyr_link_libraries(-Wl,--no-relax)

# The generated DroNet kernels.c is the ONLY V (Saturn RVV) translation unit:
# give it the Gemmini+RVV march + include/-D set. Everything else builds no-V.
if(NOT DEFINED MODELBLASTER_KERNEL_CFLAGS OR "${MODELBLASTER_KERNEL_CFLAGS}" STREQUAL "")
  set(MODELBLASTER_KERNEL_CFLAGS
    "-march=rv64imac_zve64x;-mabi=lp64;\
-isystem${MB_REPO}/cores/gemmini/include/per_config/default16x16;\
-isystem${MB_REPO}/cores/gemmini/include;\
-isystem${MB_REPO}/cores/gemmini;\
-DGEMMINI_ROCC;-DBAREMETAL;\
-DMODELBLASTER_GEMMINI_HWIO_WEIGHTS=1;-DMODELBLASTER_GEMMINI_Q31_ACC_SCALE=1")
endif()
set_source_files_properties(${MODEL_DIR}/kernels.c PROPERTIES
  COMPILE_OPTIONS "${MODELBLASTER_KERNEL_CFLAGS}")
message(STATUS "rose_fc_dronet kernels.c cflags=${MODELBLASTER_KERNEL_CFLAGS}")

target_compile_definitions(app PRIVATE
  ROSE_USE_EKF=${ROSE_USE_EKF}
  ROSE_USE_PID=${ROSE_USE_PID}
  ROSE_THREADED=${ROSE_THREADED} ROSE_CTRL_DIV=${ROSE_CTRL_DIV}
  ROSE_ACTUATE_TIMEOUT_MS=${ROSE_ACTUATE_TIMEOUT_MS} CTRL_ITERS=${CTRL_ITERS}
  ROSE_BUMPER=${ROSE_BUMPER} ROSE_FLOW=${ROSE_FLOW} ROSE_BARO=${ROSE_BARO}
  ROSE_BUMPER_READDRESS_ONLY=${ROSE_BUMPER_READDRESS_ONLY})

target_include_directories(app PRIVATE
  ${FC}/src
  ${MODEL_DIR}
  ${TINYMPC}/external/admm/include
  ${TINYMPC}/external/matlib/include
  ${TINYMPC}/include
  ${TINYMPC}/examples)
