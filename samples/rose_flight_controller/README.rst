.. _rose_flight_controller:

RoSE Flight Controller (state estimator + TinyMPC)
##################################################

Overview
********

A step toward prepping a **real** flight controller with RoSE. Where
``samples/rose/drone_control`` is handed the full ground-truth state (which no real
vehicle has), this sample receives only what onboard **sensors** measure over the RoSE
bridge and runs a **state estimator** to reconstruct the state before TinyMPC:

.. code-block:: none

   IMU  (0x12, ch2): [ax,ay,az, gx,gy,gz]  accelerometer specific force + rate gyro (body)
   FLOW (0x13, ch1): [vx,vy, h]            optical-flow horizontal velocity + ToF height
                                           (models a Crazyflie Flow deck v2)
        |
        v
   StateEstimator.update()  ->  12-DoF state  [x,y,z, r1,r2,r3, vx,vy,vz, wx,wy,wz]
        |
        v
   err = state - hover setpoint  ->  TinyMPC  ->  4 normalized thrusts (0x20)

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
