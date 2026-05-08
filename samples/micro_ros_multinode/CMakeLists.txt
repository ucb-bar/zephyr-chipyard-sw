# SPDX-License-Identifier: Apache-2.0
#
# Two-node micro-ROS smoke test for spike_riscv64 / chipyard_riscv64.
# Talks to ros-jazzy-micro-ros-agent on the host via tools/microros/.

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third-party/micro_ros_zephyr_module/modules/libmicroros)

# Same SDK 1.0.0-beta1 newlib + Annex K dance the libmicroros build needs.
# Our sources transitively include <rcl/rcl.h> which pulls rcutils'
# error_handling.h which sets __STDC_WANT_LIB_EXT1__=1 then includes <stdlib.h>,
# tripping the same header-guard ordering bug. Force the macro to 0 for the
# outer Zephyr build so rcutils' #ifndef fast-path keeps it 0.
add_compile_definitions(__STDC_WANT_LIB_EXT1__=0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(micro_ros_multinode)

target_sources(app PRIVATE
    src/main.c
    src/transport_htif.c
    src/shims.c)
