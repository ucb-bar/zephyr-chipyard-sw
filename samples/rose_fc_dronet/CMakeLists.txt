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
# fp16(Zvfh)+fp32-vector estimator+PID (runs the FC algebra on the Saturn vector unit to
# free the scalar core for co-resident DroNet). Selected in the estimator/controller
# factories; takes precedence over ROSE_USE_EKF/ROSE_USE_PID. The fp16 kernels are split
# into a pure-VECTOR unit (fp16_fc_kernels.c, built with a Zvfh -march below) and a
# soft-float SCALAR-glue unit (fp16_fc_glue.c, built with the default rv64imac soft-float
# march) so no scalar-FP instruction is emitted (the At35 core is misa.F=0 -> scalar F traps).
set(ROSE_USE_FP16 0 CACHE STRING "1=fp16(Zvfh)+fp32-vector estimator+PID, 0=scalar")
# FcRoCC (custom-0) estimator+PID: runs the whole FC on the hardened RoCC accelerator
# in Q24.24 integer (no FPU, no Saturn V). Selected in the factories; takes precedence
# over FP16/EKF/PID. Only the float<->Q24.24 boundary is scalar soft-float. REQUIRES the
# validated-core RoCC bitstream (assembled ROM+boot image) to actually compute.
set(ROSE_USE_ROCC 0 CACHE STRING "1=FcRoCC(Q24.24) estimator+PID on the custom-0 accelerator, 0=off")

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

# ---------------------------------------------------------------------------
# fp16(Zvfh) FC kernels (-DROSE_USE_FP16=1). Compile-unit split for misa.F=0:
#   * fp16_fc_kernels.c -> the pure-VECTOR unit: built with the DroNet kernel march
#     + zvfh (rv64imac_zve64x_zvfh). It contains ZERO scalar float (objdump-verified:
#     0 scalar-FP, 127 vector-FP ops), so no scalar fadd.s/flw is emitted -> no trap.
#     The Saturn vector unit (vfh=true) executes the vfh vector ops on misa.V. Using
#     the zve64x base (no scalar 'f') matches kernels.c and avoids a hard-float ELF
#     attribute at link. Also #includes the internal fp16_fc_vec.h (no CMake entry).
#   * fp16_fc_glue.c   -> the SCALAR-glue unit: left on the default global
#     rv64imac(_zicsr_zifencei) soft-float march, so its scalar float becomes
#     soft-float libcalls (no scalar-FP instruction). No per-file override needed.
# ---------------------------------------------------------------------------
if(ROSE_USE_FP16)
  target_sources(app PRIVATE
    ${FC}/src/fp16_fc_kernels.c
    ${FC}/src/fp16_fc_glue.c)
  # -fno-tree-vectorize is REQUIRED on the fp16 vector TU: without it gcc auto-
  # vectorizes the scalar boundary/int<->fp16 converts into vfncvt.f.f.w / vfwcvt.f.f.v,
  # which are BLOCKED (illegal-instr) on the fp16-only At35 Saturn. a1198297's int-accum
  # kernel is hand-vectorized with only the At35-supported ops; this flag stops gcc from
  # re-introducing the blocked ones (objdump-verified 0 blocked ops).
  set(FP16_VEC_CFLAGS "-march=rv64imac_zve64x_zvfh;-mabi=lp64;-fno-tree-vectorize")
  set_source_files_properties(${FC}/src/fp16_fc_kernels.c PROPERTIES
    COMPILE_OPTIONS "${FP16_VEC_CFLAGS}")
  message(STATUS "rose_fc_dronet fp16_fc_kernels.c (Zvfh) cflags=${FP16_VEC_CFLAGS}")
endif()

# ---------------------------------------------------------------------------
# FcRoCC FC firmware (-DROSE_USE_ROCC=1): the custom-0 driver + boot image.
# Compiles on the default global rv64imac soft-float march -- the custom-0 ops are
# .insn r 0x0B (no ISA extension needed); the Q24.24 boundary is soft-float. The
# estimator_rocc.hpp/controller_rocc.hpp drop-ins are headers (no CMake entry).
# ---------------------------------------------------------------------------
if(ROSE_USE_ROCC)
  target_sources(app PRIVATE ${FC}/src/fc_rocc_fw.c)
  message(STATUS "rose_fc_dronet: FcRoCC FC firmware (ROSE_USE_ROCC=1)")
endif()

target_compile_definitions(app PRIVATE
  ROSE_USE_EKF=${ROSE_USE_EKF}
  ROSE_USE_PID=${ROSE_USE_PID}
  ROSE_USE_FP16=${ROSE_USE_FP16}
  ROSE_USE_ROCC=${ROSE_USE_ROCC}
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
