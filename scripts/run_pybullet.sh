#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <output_path>"
    exit 1
fi

OUTPUT_PATH=$1

xvfb-run -a python3 samples/drone_control/scripts/pybullet_hil_mt_binary_sweep_forces.py --traj_file samples/drone_control/scripts/traj/auto_maneuvers_3d.json --dist_file samples/drone_control/scripts/disturbances/undisturbed.json --output_folder "$OUTPUT_PATH"