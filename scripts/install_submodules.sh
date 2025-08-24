#!/bin/bash

# Create a conda environment
conda create -yn zephyr python=3.12
conda activate zephyr

# install the west dependencies
pip3 install west pyelftools rich

# install the plotting dependencies
pip3 install matplotlib

# Init submodules
git submodule update --init ./tools/pyuartsi
git submodule update --init ./tools/gym-pybullet-drones
git submodule update --init ./tools/picolibc
git submodule update --init ./tools/riscv-gnu-toolchain
git submodule update --init ./zephyr_ws/zephyr

pip3 install ./tools/pyuartsi
pip3 install -e ./tools/gym-pybullet-drones

# Initialize west workspace
cd zephyr_ws/zephyr
west init -l .
west config manifest.file west-riscv.yml
west update
cd ../..

git submodule update --init --recursive samples/drone_control

# create data directories
mkdir -p data
mkdir -p results


# TODO set user permissions
# sudo usermod -aG plugdev,dialout "$USER"
# newgrp plugdev; newgrp dialout
