#!/usr/bin/env bash
#
# riskybird v3 flight controller -- NAMED BUILD PRESETS (single source of truth).
# The parameters differ a lot between these (loop duration, autoflight vs bench, motor duty,
# watchdog, flightlog), so each config lives here by name instead of a hand-typed -D soup.
#
#   tools/build.sh <preset> [feel: motor_duty]
#
#   viz    -> build_viz  : VISUALIZE ONLY. Continuous loop, motors OFF (AUTOFLIGHT_MAX_DUTY=0),
#                          EKF + flow + fast boot. For tethered state_viz sensor/pose/state work.
#   feel   -> build_feel : HAND-HELD FEEL TEST. Bench always-armed (no gesture -- the down-ToF
#                          can't do lift detection), motors SCALED to <motor_duty> (default 0.10),
#                          LOOSE orientation watchdog (flip ~upside-down -> motors cut = manual
#                          kill), rate/vel watchdogs off, continuous, no flightlog. Hold the caged
#                          drone + feel the response; watch it in state_viz.
#   flight -> build_fly  : FULL UNTETHERED FLIGHT. Autoflight (lift-and-place) + both chirps +
#                          on-board flightlog, motors on (cap 0.8, hover 20 cm), 20 s loop cap,
#                          tight tilt/rate watchdog. (Blocked by the down-ToF fly-away -- see
#                          docs; fix that before trusting it.)
#   dump   -> build_dump : DUMP the on-board flight log as CSV over USB, then idle. No sensors.
#
# All presets: ESP32-C6 hpcore, PID + EKF + optical flow + fast side-ToF readdress (no 12 s blob).
# Flash any:   west flash -d <build_dir>    then power-cycle (d5:00 needs a POR to boot the app).
#
set +u   # source the env BEFORE `set -e` (activate_conda.sh returns nonzero during init -> 127)
source /home/cobble/.claude/projects/-home-cobble-Tools-riskybird/esp32env.sh >/dev/null 2>&1
set -e
cd /home/cobble/Tools/zephyr-chipyard-sw

B="esp32c6_devkitc/esp32c6/hpcore"
A="samples/rose_flight_controller"
# common to the flight-controller presets (dump is minimal and sets its own):
C="-DROSE_USE_PID=1 -DROSE_USE_EKF=1 -DROSE_FLOW=1 -DROSE_BUMPER=1 -DROSE_BUMPER_READDRESS_ONLY=1 -DROSE_ACTUATE_TIMEOUT_MS=0"

case "${1:-}" in
  viz)
    DIR=build_viz
    west build -b "$B" "$A" -d "$DIR" -- $C -DCTRL_ITERS=0 \
      -DEXTRA_CPPFLAGS="-DROSE_AUTOFLIGHT=1 -DVL53L1X_TIMING_BUDGET_US=100000 -DHOVER_Z_M=0.20f \
        -DT_CLIMB_MS=1500 -DT_HOVER_MS=1500 -DT_DESCEND_MS=1500 -DAUTOFLIGHT_MAX_DUTY=0.0f \
        -DPID_MASS_KG=0.060f -DSAFE_MAX_VEL_MPS=1000.0f"
    NOTE="motors OFF, continuous" ;;
  feel)
    DIR=build_feel
    DUTY="${2:-0.10}"
    west build -b "$B" "$A" -d "$DIR" -- $C -DCTRL_ITERS=0 \
      -DEXTRA_CPPFLAGS="-DMOTOR_MAX_DUTY=${DUTY}f -DSAFE_MAX_TILT_RAD=2.5f \
        -DSAFE_MAX_RATE_RADPS=1000.0f -DSAFE_MAX_VEL_MPS=1000.0f"
    NOTE="bench always-armed, motors scaled to ${DUTY}, flip-upside-down kill (~136 deg)" ;;
  flight)
    DIR=build_fly
    # === WORKING position-hold config -- 2026-08-13 milestone (~1 m drift). BARE drone (cage OFF). ===
    # Fix stack that got here (see docs/FLIGHT_BUILD.md): fly bare (the prop cage was too heavy ->
    # thrust-marginal -> altitude saturation -> tip); altitude INTEGRAL (KI_HEIGHT) auto-calibrates
    # hover thrust (no more PID_MASS_KG guessing); gyro-bias STARTUP CAL (GYRO_CAL_*) kills attitude
    # jitter so the flow stays valid; flow Y-axis noise handled by a SENSOR-RATE low-pass (FLOW_LP_TAU_Y)
    # + gyro-rate flow gate (FLOW_GYRO_MAX) + tilt slew limit (TILT_SLEW_RADPS) to stop the roll<->flow
    # oscillation; velocity PI (KI_VEL) auto-trims the constant tilt and kills the steady drift.
    # NOTE: ROSE_USE_EKF=0 is REQUIRED -- the EKF's tof_gate decouples altitude and climbs away.
    west build -b "$B" "$A" -d "$DIR" -- \
      -DROSE_USE_PID=1 -DROSE_USE_EKF=0 -DROSE_FLOW=1 -DROSE_BUMPER=1 -DROSE_BUMPER_READDRESS_ONLY=1 \
      -DROSE_ACTUATE_TIMEOUT_MS=0 -DCTRL_ITERS=20000 \
      -DEXTRA_CPPFLAGS="-DROSE_AUTOFLIGHT=1 -DROSE_ARM_NO_GESTURE=1 -DROSE_FLIGHTLOG=1 -DROSE_FLIGHTLOG_ERASE=1 \
        -DROSE_TELEM=0 -DROSE_VEL_LOOP=1 -DROSE_FLOW_FUSE=1 \
        -DFLOW_LP_TAU_X=0.05f -DFLOW_LP_TAU_Y=0.15f -DFLOW_GYRO_MAX=1.2f \
        -DKI_HEIGHT=1.5f -DALT_INT_MAX=3.0f -DKI_VEL=0.8f -DVEL_INT_MAX=2.0f \
        -DVEL_TC=0.5f -DVEL_TILT_MAX=0.26f -DTILT_SLEW_RADPS=1.0f -DATT_GAIN=1.0f \
        -DGYRO_CAL_SECONDS=2.0f -DGYRO_CAL_STILL_RADPS=0.30f \
        -DVL53L1X_TIMING_BUDGET_US=100000 -DHOVER_Z_M=0.70f \
        -DT_CLIMB_MS=2000 -DT_HOVER_MS=4000 -DT_DESCEND_MS=2000 -DFLIGHT_MAX_MS=12000 \
        -DSAFE_MAX_HEIGHT_M=1.2f -DAUTOFLIGHT_MAX_DUTY=0.95f -DPID_MASS_KG=0.038f -DSAFE_MAX_VEL_MPS=2.0f"
    NOTE="WORKING position-hold: bare drone, complementary, gyro-cal + alt/vel integrators + flow filtering" ;;
  dump)
    DIR=build_dump
    west build -b "$B" "$A" -d "$DIR" -- -DROSE_USE_PID=1 \
      -DEXTRA_CPPFLAGS="-DROSE_FLIGHTLOG_DUMP=1"
    NOTE="flightlog CSV dump" ;;
  *)
    echo "usage: tools/build.sh <viz|feel|flight|dump> [feel: motor_duty, e.g. 0.10]"; exit 1 ;;
esac
echo "build.sh: '$1' [$NOTE] -> $DIR/zephyr/zephyr.elf   (flash: west flash -d $DIR, then POR)"
