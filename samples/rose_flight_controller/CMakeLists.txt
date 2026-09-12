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
set(ROSE_USE_EKF 1 CACHE STRING "1=EKF, 0=complementary filter")
# Controller selection (modular IController; both are compiled, factory picks):
#   -DROSE_USE_PID=0 -> TinyMPC constrained MPC (default)
#   -DROSE_USE_PID=1 -> hierarchical PID cascade (ported from the old riskybird FreeRTOS firmware)
set(ROSE_USE_PID 0 CACHE STRING "1=hierarchical PID, 0=TinyMPC")
# Architecture knobs: ROSE_THREADED (1=modular threaded blocks, 0=single loop) and
# ROSE_CTRL_DIV (control runs every Nth estimator tick -> control rate = estimation / N).
set(ROSE_THREADED 0 CACHE STRING "1=threaded blocks, 0=single loop (default; see main.cpp note)")
set(ROSE_CTRL_DIV 1 CACHE STRING "control every Nth estimator tick (multi-rate)")
# Estimator-only: run sensors + estimator + telemetry and never call the
# controller. For characterising the estimator by itself; see solve_control().
set(ROSE_ESTIMATOR_ONLY 0 CACHE STRING "1=estimator only (controller never runs)")
# Bench safety: stop driving motors this many ms after boot (0 = never). Real-HW bench
# bring-up builds with 10000 so the props auto-stop 10 s after reset. Keep 0 for flight.
set(ROSE_ACTUATE_TIMEOUT_MS 0 CACHE STRING "stop motor actuation N ms after boot (0=never)")
# Control-loop iteration count. Default 5000 suits the RoSE co-sim; on real HW the loop runs
# ~1 kHz, so raise this for a longer bench run (e.g. 120000 ~= 2 min).
set(CTRL_ITERS 5000 CACHE STRING "control-loop iteration count")
# Side-ToF wall "bumper": 1 = bring up the 4 VL53L5CX + apply bounded wall-repulsion lean.
# Adds ~12 s to boot (4x firmware upload). 0 = skip entirely (side_tof.c compiled but idle).
set(ROSE_BUMPER 0 CACHE STRING "1=enable side-ToF wall bumper, 0=disabled")
# Fast-boot variant: with ROSE_BUMPER=1, re-address the 4 sides off 0x29 (keeps the I2C bus clear for
# the down VL53L1X) but SKIP the ~84 KB-per-sensor ULD firmware upload + ranging. No wall data, but
# boot is ~0.5 s instead of ~12 s. Use when the FC doesn't consume walls (it currently doesn't).
set(ROSE_BUMPER_READDRESS_ONLY 0 CACHE STRING "1=re-address sides only (fast boot, no ranging)")
# Optical flow: 1 = bring up the PMW3901 (SPI2) + feed body-frame velocity to the estimator.
set(ROSE_FLOW 0 CACHE STRING "1=enable PMW3901 optical flow, 0=disabled")
# Barometer altitude fusion: 1 = read the BMP388 (needs a `baro` DT alias + CONFIG_BMP388) and fuse
# its smooth relative altitude with the down-ToF in the estimator (robust altitude under tilt /
# obstacle step-jumps). 0 = ToF-only (unchanged). Fine-tuning knobs (BARO_Z_GAIN, BARO_VZ_GAIN,
# BARO_BIAS_GAIN, BARO_TOF_STEP_M, R_BARO) are passed via -DEXTRA_CPPFLAGS like the flow knobs.
set(ROSE_BARO 0 CACHE STRING "1=enable BMP388 barometer altitude fusion, 0=disabled")

# Flight knobs. These existed in main.cpp with #ifndef defaults but were absent
# from target_compile_definitions below, so -DROSE_AUTOFLIGHT=1 set a cache
# variable that nothing read: the build silently kept the bench path and the
# arming gate was never compiled in. Verified by symbol table -- rose_cmd_reset
# and g_arm_enabled were both absent from an image built with the flag set.
# That is the failure FLIGHT_BUILD.md warns about, so the knobs are declared
# here where they are visibly wired to the definitions.
#
# Defaults match main.cpp's #ifndef values exactly, so adding them changes no
# existing build.
set(ROSE_AUTOFLIGHT 0 CACHE STRING "1=lift-and-place arm gate + autonomous altitude profile")
set(ROSE_BATT_SENSE 0 CACHE STRING "1=battery ADC: arm gate, in-flight cutoff, thrust sag comp")
set(ROSE_FLIGHTLOG  0 CACHE STRING "1=background flight log to RAM, flushed at flight end")
set(ROSE_TELEM      1 CACHE STRING "1=in-loop console telemetry (MUST be 0 untethered)")
set(ROSE_TELEM_DIV  10 CACHE STRING "print one telemetry line every Nth iteration")
# Bench escape hatch: auto-arm after ARM_SETTLE_MS of level+still, with no
# lift-and-place gesture AND no on-ground check. main.cpp already names the
# reason -- "near-field ToF validity when sitting flush is unreliable, and it
# was the likely arm blocker". Measured on riskybird v3: a drone set down reads
# 190-310 mm on the down-ToF against an ARM_MAX_HEIGHT_M of 20 mm, so `ground`
# is never true and the gesture can never complete.
#
# Deliberately NOT part of any mode. It removes the interlock that makes a
# glitch-reboot safe, so it has to be an explicit choice at the command line
# rather than something a mode name quietly implies:
#   RB_CMAKE_ARGS="-DROSE_ARM_NO_GESTURE=1" ./rb debug flight_controller ...
set(ROSE_ARM_NO_GESTURE 0 CACHE STRING "1=auto-arm on level+still, no gesture, no ground check")

# Flight envelope. All three are already #ifndef-guarded in main.cpp but were
# not reachable from the build, so tuning them meant editing the source -- the
# one thing you cannot do safely between two attempts on a bench.
#
# ARM_MAX_HEIGHT_M especially: at 0.020 it is tighter than the hardware's own
# resting reading. Measured on riskybird v3, drone flat on the bench, the
# down-ToF reports 0.022 m -- an honest number, the sensor sits about that far
# above the surface -- so the "on the ground" test failed by 2 mm and the
# lift-and-place gesture could never complete.
set(ARM_MAX_HEIGHT_M    0.020f CACHE STRING "on-ground height below which arming is allowed (m)")
set(HOVER_Z_M           0.30f  CACHE STRING "autoflight hover altitude target (m)")
set(AUTOFLIGHT_MAX_DUTY 0.45f  CACHE STRING "per-motor duty ceiling in flight (hover ~0.15)")

# Airframe property, measured per drone -- see the note in main.cpp. Driving a
# motor below its break-away duty stalls it, and the ESC then latches until the
# battery is pulled, so this is not a comfort setting.
set(MOTOR_BREAKAWAY_DUTY 0.15f CACHE STRING "lowest duty at which every motor reliably starts")
set(MOTOR_CHIRP_MS       400   CACHE STRING "chirp pulse length, ms")
# NOTE: the bench cap below defaults to 0.10, which on this airframe is BELOW
# break-away -- so `--mode motors` cannot start motor 4 at the default. Raise it
# to at least MOTOR_BREAKAWAY_DUTY to drive motors in that mode. Left alone here
# rather than raised silently, because it is a thrust-limiting safety cap.
set(MOTOR_MAX_DUTY       0.10f CACHE STRING "bench-mode duty ceiling (non-autoflight)")

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
  src/flow.c
  src/pmw3901.c
  ${TINYMPC}/external/admm/src/admm_rvv.cpp)

# WiFi SoftAP UDP telemetry downlink (docs/TELEMETRY_PLAN.md phase 2). Compiled ONLY when the
# opt-in telem.conf enables CONFIG_WIFI, so the normal flight build never sees it and stays
# byte-identical. All call sites in main.cpp are likewise `#if defined(CONFIG_WIFI)`.
if(CONFIG_WIFI)
  target_sources(app PRIVATE src/telem_wifi.c)
endif()

target_compile_definitions(app PRIVATE ROSE_USE_EKF=${ROSE_USE_EKF}
  ROSE_USE_PID=${ROSE_USE_PID}
  ROSE_THREADED=${ROSE_THREADED} ROSE_CTRL_DIV=${ROSE_CTRL_DIV}
  ROSE_ACTUATE_TIMEOUT_MS=${ROSE_ACTUATE_TIMEOUT_MS} CTRL_ITERS=${CTRL_ITERS}
  ROSE_BUMPER=${ROSE_BUMPER} ROSE_FLOW=${ROSE_FLOW} ROSE_BARO=${ROSE_BARO}
  ROSE_ESTIMATOR_ONLY=${ROSE_ESTIMATOR_ONLY}
  ROSE_BUMPER_READDRESS_ONLY=${ROSE_BUMPER_READDRESS_ONLY}
  ROSE_AUTOFLIGHT=${ROSE_AUTOFLIGHT} ROSE_BATT_SENSE=${ROSE_BATT_SENSE}
  ROSE_FLIGHTLOG=${ROSE_FLIGHTLOG}
  ROSE_TELEM=${ROSE_TELEM} ROSE_TELEM_DIV=${ROSE_TELEM_DIV}
  ROSE_ARM_NO_GESTURE=${ROSE_ARM_NO_GESTURE}
  ARM_MAX_HEIGHT_M=${ARM_MAX_HEIGHT_M} HOVER_Z_M=${HOVER_Z_M}
  AUTOFLIGHT_MAX_DUTY=${AUTOFLIGHT_MAX_DUTY}
  MOTOR_BREAKAWAY_DUTY=${MOTOR_BREAKAWAY_DUTY} MOTOR_CHIRP_MS=${MOTOR_CHIRP_MS}
  MOTOR_MAX_DUTY=${MOTOR_MAX_DUTY})

target_include_directories(app PRIVATE
  ${TINYMPC}/external/admm/include
  ${TINYMPC}/external/matlib/include
  ${TINYMPC}/include
  ${TINYMPC}/examples)
