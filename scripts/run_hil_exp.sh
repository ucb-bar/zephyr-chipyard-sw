#!/bin/bash

# Build the ELF
west build -b chipyard_cygnus samples/drone_control/ -p

python3 scripts/reset_soc.py

bash scripts/flash_chip.sh

# run the pybullet simulation
xvfb-run -a python3 samples/drone_control/scripts/pybullet_hil_mt_binary_sweep_forces.py --traj_file samples/drone_control/scripts/traj/auto_maneuvers_3d.json --dist_file samples/drone_control/scripts/disturbances/undisturbed.json --output_folder data/test_xvfb-run