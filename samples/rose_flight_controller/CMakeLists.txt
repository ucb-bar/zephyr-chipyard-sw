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
#
# READ THE DEFAULT. It is 0, which is DISABLED, in every build that does not override it -- and
# nothing in tools/rb/boards.py overrides it for any mode. main.cpp guards the feature with
# `#if ROSE_ACTUATE_TIMEOUT_MS > 0`, so with the default the whole cutoff is preprocessed out of
# the image. There is no "#ifndef" fallback in the source; this line is the only definition, and
# it turns the feature off. That is why the commitment gate below exists and is NOT defaulted off.
set(ROSE_ACTUATE_TIMEOUT_MS 0 CACHE STRING "stop motor actuation N ms after boot (0=never)")

# ---- Commitment gate ---------------------------------------------------------------------------
# How long motors may run without a deliberate commitment, and what committing takes.
#
# ROSE_FLIGHT_COMMIT is the BUILD-TIME half of a two-part interlock and is deliberately NOT set by
# any --mode in tools/rb/boards.py: no mode name may quietly imply permission to fly. It has to be
# typed, as RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1". With it at 0 the run-time commit variable is not
# compiled into the image at all, so hardware/flight/commit.gdb fails with "No symbol" -- the
# absence of that symbol is the proof that the image cannot be made to fly.
#
# The RUN-TIME half is the operator writing the key from a debugger, on the bench. Neither act alone
# does anything.
set(ROSE_FLIGHT_COMMIT   0     CACHE STRING "1=build CAN be committed to a flight test at run time")
set(ROSE_MOTOR_WINDOW_MS 5000  CACHE STRING "uncommitted: ms of motor-live time before the latch")
set(ROSE_COMMIT_MAX_MS   60000 CACHE STRING "committed: ms of motor-live time before the latch")
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
# Which pin on the `gpio0` controller is the PMW3901's software chip-select.
#
# 19 is the ESP32-C6's, and is the value the untethered hops in docs/FLIGHT_BUILD.md flew with,
# so the default changes no existing build. The RiskyBird FPGA carrier's DroneLogic GPIO block is
# a different controller with its own numbering and puts the flow CS at index 0 -- see
# hardware/zephyr/targets/fpga/workloads/pmw3901_test.overlay in the riskybird tree, which is the
# wiring the standalone PMW3901 bring-up reads the chip ID through.
#
# Declared here for the same reason as PID_MASS_KG below: flight_controller-flow.overlay and
# docs/flight-on-fpga.md both instruct the operator to pass -DCS_GPIO_PIN=0 for an FPGA flow
# build, and until flow.c grew an #ifndef around its default that -D was silently discarded. A
# bare -D on the CMake line is STILL discarded unless the name is listed in
# target_compile_definitions, which is what this pair of lines fixes -- so a --mode can now carry
# the right chip-select instead of it being a remembered command-line flag.
set(CS_GPIO_PIN 19 CACHE STRING "PMW3901 software chip-select pin on the gpio0 controller")
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
# Non-autoflight hover setpoint. 1.0 m is the co-sim's number and is a 1 m step error for a drone
# sitting on a bench, which saturates the altitude loop from the first iteration. Reachable here so
# a bench run can regulate to where the drone actually is (-DTARGET_Z=0.0f) rather than to a
# simulator's start height. Default matches main.cpp, so no existing build changes.
set(TARGET_Z            1.0f   CACHE STRING "non-autoflight hover setpoint altitude (m)")
set(AUTOFLIGHT_MAX_DUTY 0.45f  CACHE STRING "per-motor duty ceiling in flight (hover ~0.15)")

# Airframe mass, in kg, feeding the PID cascade's altitude thrust feedforward.
#
# This is the SAME class of trap as ROSE_AUTOFLIGHT above, and it caught the flight build:
# PID_MASS_KG existed ONLY as an `#ifndef` inside controller_pid.cpp and appeared nowhere in
# target_compile_definitions, so `-DPID_MASS_KG=0.060f` on the CMake line set a cache variable
# that nothing read and the image silently kept the 32 g Crazyflie default. docs/FLIGHT_BUILD.md
# works around that by smuggling it through EXTRA_CPPFLAGS, under a heading that says in capitals
# that these are NOT CMake knobs. Declaring it here makes it one, so a --mode can set it.
#
# It matters because the altitude loop is a PD with no integrator: the feedforward IS how the
# loop reaches hover thrust, and a mass that is half the truth cannot get there at all. riskybird
# v3 measures ~60 g against the 32 g default.
#
# The default below matches controller_pid.cpp's #ifndef exactly, so adding this changes no
# existing build.
set(PID_MASS_KG         0.032f CACHE STRING "airframe mass for the PID altitude feedforward (kg)")

# Airframe property, measured per drone -- see the note in main.cpp. Driving a
# motor below its break-away duty stalls it, and a stalled BRUSHED motor on a
# low-side SI2302 is a resistor across the pack: motor and FET heat within
# seconds. (There are no ESCs on this airframe; nothing latches. The old comment
# here said otherwise.) So this is not a comfort setting.
set(MOTOR_BREAKAWAY_DUTY 0.15f CACHE STRING "lowest duty at which every motor reliably starts")
set(MOTOR_CHIRP_MS       400   CACHE STRING "chirp pulse length, ms")
# NOTE: the bench cap below defaults to 0.10, which on this airframe is BELOW
# break-away -- so `--mode motors` cannot start motor 4 at the default. Raise it
# to at least MOTOR_BREAKAWAY_DUTY to drive motors in that mode. Left alone here
# rather than raised silently, because it is a thrust-limiting safety cap.
set(MOTOR_MAX_DUTY       0.10f CACHE STRING "bench-mode duty SCALE (non-autoflight)")

# THE KNOB THAT MEANS WHAT "CAP" SOUNDS LIKE -- read the long note beside
# MOTOR_MAX_DUTY in main.cpp before setting either.
#
# MOTOR_MAX_DUTY above is a SCALE applied last, to a duty that is already
# physical: the controller's u + 0.583 round-trips controller_pid.cpp's
# forceToVoltage() exactly. Hover on this airframe needs ~0.65-0.71 duty
# (docs/FLIGHT_BUILD.md "hover needs ~65%", docs/FLIGHT_TUNING_LOG.md "~71 %
# hover duty"), so setting MOTOR_MAX_DUTY to 0.75 "as a 75 % cap" delivers
# 0.71 x 0.75 = 0.53 at the controller's hover command -- about 55 % of hover
# THRUST, since thrust goes as duty^2. The drone cannot hover, and it fails by
# porpoising rather than by sitting still: saturated while the altitude error is
# large, collapsing as the command falls back toward hover.
#
# MOTOR_DUTY_CEILING is the ceiling the bench path's attitude-priority
# anti-saturation cut works against, so exceeding it lowers the COLLECTIVE and
# preserves the roll/pitch/yaw differential -- the semantics AUTOFLIGHT_MAX_DUTY
# already has on the autoflight path.
#
# Default 1.0f = the hard-coded value it replaces, so every existing build is
# bit-identical. A flight attempt sets BOTH:
#
#   RB_CMAKE_ARGS="-DMOTOR_MAX_DUTY=1.0f -DMOTOR_DUTY_CEILING=0.75f \
#                  -DROSE_FLIGHT_COMMIT=1"
#
# and raises workloads/esp_motors' ESP_MOTORS_MAX_DUTY_PCT in the same session,
# because that ceiling is independent and the LOWER of the two wins.
set(MOTOR_DUTY_CEILING   1.0f  CACHE STRING "collective duty ceiling, differential-preserving (non-autoflight)")

# Motor offload to the ESP32-C6. FPGA ball F13 -- motor4's gate -- is dead as an
# output, so the FPGA can never drive all four motors. With this set the actuator
# layer drives NO pin at all: it encodes the same duties into a 14-byte frame and
# sends them on uart1 to the ESP, which owns every gate and generates the PWM.
# The wire format is shared with workloads/esp_motors through the header in
# ROSE_MOTOR_LINK_DIR, so neither end can drift from the other.
set(ROSE_ESP_MOTORS 0 CACHE STRING "1=send motor duty frames on uart1 instead of driving PWM")
set(ROSE_MOTOR_LINK_DIR "" CACHE STRING "riskybird integration/zephyr/motor_link directory")
set(ROSE_ESP_MOTORS_HZ 50 CACHE STRING "motor duty frame rate on the ESP link, Hz")

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
  CS_GPIO_PIN=${CS_GPIO_PIN}
  ROSE_ESTIMATOR_ONLY=${ROSE_ESTIMATOR_ONLY}
  ROSE_BUMPER_READDRESS_ONLY=${ROSE_BUMPER_READDRESS_ONLY}
  ROSE_AUTOFLIGHT=${ROSE_AUTOFLIGHT} ROSE_BATT_SENSE=${ROSE_BATT_SENSE}
  ROSE_FLIGHTLOG=${ROSE_FLIGHTLOG}
  ROSE_TELEM=${ROSE_TELEM} ROSE_TELEM_DIV=${ROSE_TELEM_DIV}
  ROSE_ARM_NO_GESTURE=${ROSE_ARM_NO_GESTURE}
  ARM_MAX_HEIGHT_M=${ARM_MAX_HEIGHT_M} HOVER_Z_M=${HOVER_Z_M} TARGET_Z=${TARGET_Z}
  AUTOFLIGHT_MAX_DUTY=${AUTOFLIGHT_MAX_DUTY}
  MOTOR_BREAKAWAY_DUTY=${MOTOR_BREAKAWAY_DUTY} MOTOR_CHIRP_MS=${MOTOR_CHIRP_MS}
  MOTOR_MAX_DUTY=${MOTOR_MAX_DUTY} MOTOR_DUTY_CEILING=${MOTOR_DUTY_CEILING}
  PID_MASS_KG=${PID_MASS_KG}
  ROSE_ESP_MOTORS=${ROSE_ESP_MOTORS} ROSE_ESP_MOTORS_HZ=${ROSE_ESP_MOTORS_HZ}
  ROSE_FLIGHT_COMMIT=${ROSE_FLIGHT_COMMIT}
  ROSE_MOTOR_WINDOW_MS=${ROSE_MOTOR_WINDOW_MS}
  ROSE_COMMIT_MAX_MS=${ROSE_COMMIT_MAX_MS})

if(ROSE_ESP_MOTORS)
  # Named rather than assumed: this sample is built in place from a submodule and
  # has no fixed relationship to the riskybird checkout around it. Failing loudly
  # here beats a "no such file: riskybird/motor_link.h" 200 lines into main.cpp.
  if(NOT ROSE_MOTOR_LINK_DIR)
    message(FATAL_ERROR
      "ROSE_ESP_MOTORS=1 needs -DROSE_MOTOR_LINK_DIR=<riskybird>/integration/zephyr/motor_link")
  endif()
  target_include_directories(app PRIVATE ${ROSE_MOTOR_LINK_DIR}/include)
endif()

target_include_directories(app PRIVATE
  ${TINYMPC}/external/admm/include
  ${TINYMPC}/external/matlib/include
  ${TINYMPC}/include
  ${TINYMPC}/examples)
