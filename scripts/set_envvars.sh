#!/bin/bash
export ZEPHYR_BASE=$PWD/zephyr_ws/zephyr
export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
export CROSS_COMPILE=$PWD/tools/riscv-install/bin/riscv64-unknown-elf-
