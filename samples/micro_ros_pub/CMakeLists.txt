# SPDX-License-Identifier: Apache-2.0
#
# Skeleton micro-ROS sample for spike_riscv64 / chipyard_riscv64.
# Step 3 of bring-up: validate that libmicroros.a + microros_transports
# compile and link cleanly into a Zephyr image. main() makes no micro-ROS
# API calls yet — that comes once the HTIF custom transport is in place.

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third-party/micro_ros_zephyr_module/modules/libmicroros)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(micro_ros_pub)

# shims.c provides isatty() — referenced by libmicroros (rcutils logging) but
# not surfaced by this SDK + picolibc combo. See zephyr_sdk_compat.h in the
# libmicroros submodule for context.
target_sources(app PRIVATE
    src/main.c
    src/shims.c)
