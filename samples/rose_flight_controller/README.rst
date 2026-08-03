.. _rose_flight_controller:

RoSE Flight Controller (state estimator + TinyMPC)
##################################################

Overview
********

A step toward prepping a **real** flight controller with RoSE. Where
``samples/rose/drone_control`` is handed the full ground-truth state (which no real
vehicle has), this sample receives only what onboard **sensors** measure and runs a
**state estimator** to reconstruct the state before TinyMPC.

Sensors are read through the **standard Zephyr sensor API** (``DEVICE_DT_GET(DT_ALIAS(...))``
+ ``sensor_sample_fetch`` / ``sensor_channel_get``), so this ONE application builds two ways
-- ``main`` / estimator / TinyMPC are byte-for-byte identical; only the board overlay +
``.conf`` differ (see ``docs/ROSE_SENSOR_ABSTRACTION.md``):

* **RoSE co-sim** (``-b spike_riscv64``): aliases bind to the virtual ``ucbbar,rose-*``
  sensor drivers (in the ``zephyr-rose`` module), which fetch from the RoSE bridge / Isaac
  Sim virtual sensors. ``boards/spike_riscv64.{overlay,conf}``.
* **Real hardware** (``-b esp32c6_devkitc/esp32c6/hpcore``, the "riskybird" PCB): aliases
  bind to the real ``bosch,bmi08x-*`` IMU over I2C. ``boards/esp32c6_devkitc_hpcore.{overlay,conf}``.

Underlying wire packets (RoSE build): IMU ``0x12`` ch2 ``[ax,ay,az, gx,gy,gz]``; optical
flow ``0x13`` ch1 ``[vx,vy]``; low-rate ToF ``0x14`` ch1 ``[h]``. Data flow:

.. code-block:: none

   IMU  (BMI088)  : accel [ax,ay,az] + gyro [gx,gy,gz]   (SENSOR_CHAN_ACCEL_XYZ/GYRO_XYZ)
   FLOW (PMW3901) : body-frame horizontal velocity [vx,vy]  (private FLOW_VX/VY channels)
   ToF  (VL53L1x) : downward height [h], LOW-RATE          (SENSOR_CHAN_DISTANCE, -EAGAIN)
        |  (Zephyr sensor API: same calls on RoSE and real hardware)
        v
   StateEstimator.update()  ->  12-DoF state  [x,y,z, r1,r2,r3, vx,vy,vz, wx,wy,wz]
        |
        v
   err = state - hover setpoint  ->  TinyMPC  ->  4 normalized thrusts

The estimator (``src/estimator.{hpp,cpp}``) is a complementary filter + dead-reckoning,
extending ``samples/flight_controller``'s ``attitude_estimator.c``:

* **attitude** — gyro integration trimmed toward the accelerometer gravity vector
  (roll/pitch); yaw from gyro only.
* **horizontal velocity** — accel integration fused with the optical-flow measurement.
* **altitude (z, vz)** — accel integration corrected by the downward ToF height (a
  fixed-gain observer), so altitude is observable and the hover holds.
* **x/y position + yaw** — pure integration (no absolute reference -> drift).

Two design points make this actually hover:

* **ToF height is required.** IMU + optical-flow *velocity* alone leave altitude
  unobservable — dead-reckoned height locks in the takeoff-transient error and the
  vehicle hovers at the wrong altitude. The Flow deck's downward ToF fixes that (local
  height, not global pose).
* **Regulate horizontal velocity, not position.** With no absolute x/y reference, the
  dead-reckoned position drifts; regulating it makes TinyMPC chase a phantom and
  destabilize. The controller zeroes the x/y *position* error and regulates x/y
  *velocity* instead, so the vehicle holds level and drifts slowly.

There is still **no absolute horizontal-position or heading reference** (no GPS / mocap /
magnetometer), so x/y position and yaw drift over time — the vehicle holds a hover for
~1-1.5 s, then slowly wanders and the accumulating IMU-only attitude/yaw error eventually
degrades it. That drift is expected and is exactly what this setup exists to study.

Pairs with the ``IsaacCrazyflieSensorEnv-v0`` environment, which synthesizes these
sensors from the IsaacLab Crazyflie ground truth.

Build
*****

Requires the TinyMPC submodule (shared with ``samples/drone_control``):

.. code-block:: console

   git -C soc/sw/xpu-rt/zephyr-chipyard-sw submodule update --init samples/drone_control/tinympc

Then, from the RoSE repo root (uses the in-tree Zephyr toolchain):

.. code-block:: console

   soc/sim/build_zephyr_rose.sh rose_flight_controller
   # -> soc/sim/zephyr_rose_builds/rose_flight_controller/zephyr/zephyr.elf

Build for real hardware (ESP32C6 riskybird)
-------------------------------------------

The SAME sources build for the ESP32C6 board with the real BMI088 IMU. ESP32C6 SoC support
needs the Espressif HAL, which the lean RISC-V RoSE workspace does not fetch by default;
add it once (copy ``modules/hal/espressif`` from an espressif-enabled Zephyr workspace into
this workspace, or ``west update`` against the full manifest) and ``pip install
"esptool>=5.0.2"``. Then:

.. code-block:: console

   soc/sim/build_zephyr_esp32c6.sh
   # -> soc/sim/zephyr_rose_builds/rose_flight_controller_esp32c6/zephyr/zephyr.elf

Builds today with the real ``bosch,bmi08x`` IMU (SRAM ~90% of the C6's 437 KB). The ToF
(``st,vl53l1x``) additionally needs ``hal_st`` and optical flow needs a PMW3901 driver
(neither vendored here) -- they are guarded off (``HAVE_TOF``/``HAVE_FLOW``) until added;
see the notes in ``boards/esp32c6_devkitc_hpcore.overlay``. Actuator output (motors) is the
only other target-specific piece and is a no-op stub on real HW pending a PWM ``motors`` node.

Control rate (important)
************************

The controller runs at **200 Hz**, not 50 Hz. With the sensor-based estimator in the loop,
50 Hz is too slow for the Crazyflie's fast attitude dynamics (the estimator's latency
erodes the phase margin and the attitude loop rings, then diverges). The TinyMPC LQR gain
is rate-tolerant, so running the 50 Hz-designed policy at 200 Hz simply tightens the loop
and yields a stable >10 s hover. The estimator+solve costs ~0.69 M cycles/step, so 200 Hz
uses ~14% of a 5 M-cycle budget at 1 GHz (headroom to ~1.4 kHz). Set in the configs:

* ``config_gym_IsaacCrazyflieSensorEnv-v0.yaml``: ``gym_timestep: 0.005``, ``ctrl_freq: 200``
* ``config_deploy_gym.yaml``: ``firesim_freq: 1_000_000_000``, ``firesim_step: 5_000_000``
  (5M / 1e9 = 0.005 s = 200 Hz, one control per env step)
* the guest's ``CTRL_DT`` must match (0.005 s)

Result: stable hover for >10 s -- altitude z in [1.015, 1.023] m, horizontal drift < 0.1 mm
(velocity held ~0 by optical flow; x/y position dead-reckoned, drifts sub-mm).

Run (Spike, closed loop)
************************

.. code-block:: console

   # Terminal 1 — synchronizer with the sensor env (Isaac-capable Python), 1 GHz SoC:
   #   set gym_env: 'IsaacCrazyflieSensorEnv-v0' + the 1 GHz timing in
   #   deploy/config/config_deploy_gym.yaml (see config_gym_IsaacCrazyflieSensorEnv-v0.yaml)
   cd deploy/hephaestus
   ROSE_DIR=$(git rev-parse --show-toplevel) <env_isaaclab>/bin/python run_sync_only.py

   # Terminal 2 — Spike lockstep bridge with this guest:
   soc/sim/run_spike_rose_lockstep.sh \
       soc/sim/zephyr_rose_builds/rose_flight_controller/zephyr/zephyr.elf 1

Expected: the guest prints ``estimator + TinyMPC ready`` then per-iter ``z_est``/``z_err``
tracking the hover setpoint. The drone holds altitude near 1.0 m (ToF-observed) for
~1-1.5 s, then slowly drifts in x/y and yaw (dead-reckoned, unobservable) until the
accumulating IMU-only attitude error degrades the hover — the "drift without global pose
feedback" this sample demonstrates. It starts near the setpoint (``START_Z`` = 0.9 m,
matching the env ``start_height``) because from estimated state the controller cannot
brake a hard takeoff without overshoot.
