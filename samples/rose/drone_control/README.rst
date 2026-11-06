.. _rose_drone_control:

RoSE Drone Control (TinyMPC over the RoSE bridge)
#################################################

Overview
********

A RoSE co-simulation port of ``samples/drone_control`` — the TinyMPC quadrotor HIL
controller. The upstream sample exchanges state/control with a physics host over a
**physical UART** on a prototype board. This version talks to the **RoSE bridge**
instead, and pulls the **full simulator state** directly (the 12-DoF linearized
quadrotor state) rather than going through a rose-imu sensor abstraction.

Each 50 Hz control step, paired with the ``PyBulletDroneMPCEnv-v0`` physics env:

1. request the full state — reqrsp ``cmd 0x12`` → 12 ``float32`` on channel 2
2. solve TinyMPC
3. return 4 normalized motor thrusts — TX ``cmd 0x20`` → applied as the env action

State vector (matches ``samples/drone_control/scripts/pybullet_hil.py`` and the env)::

    [x, y, z, r1, r2, r3, vx, vy, vz, dphi, dtheta, dpsi]   (Rodrigues attitude)

Prerequisites
*************

* The TinyMPC controller is reused from the sibling sample; initialize its submodule::

    git -C soc/sw/xpu-rt/zephyr-chipyard-sw submodule update --init \
        samples/drone_control/tinympc

* The RoSE Zephyr module (driver + protocol) is supplied at build time via
  ``-DZEPHYR_EXTRA_MODULES=<rose>/soc/sw/zephyr-rose`` — ``soc/sim/build_zephyr_rose.sh``
  passes this automatically.
* By default this builds **scalar** for ``spike_riscv64``. To use the RVV solver path,
  build with ``-DRISCV_VECTOR=1`` on a vector-capable target and enable the vector
  Kconfig in ``prj.conf``.

Build & run
***********

::

    # build (once the tinympc submodule + Zephyr toolchain are in place)
    soc/sim/build_zephyr_rose.sh drone_control

    # terminal 1 — physics
    cd deploy/hephaestus
    # set gym_env: 'PyBulletDroneMPCEnv-v0' in deploy/config/config_deploy_gym.yaml
    ROSE_DIR=$(git rev-parse --show-toplevel) ../.venv-rose/bin/python run_sync_only.py

    # terminal 2 — SoC (Spike lockstep tier)
    soc/sim/run_spike_rose_lockstep.sh \
        soc/sim/zephyr_rose_builds/drone_control/zephyr/zephyr.elf 1

The controller regulates the quadrotor to the env's hover setpoint; the synchronizer
logs the drone trajectory to ``deploy/hephaestus/logs/``.
