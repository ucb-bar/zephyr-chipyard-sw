# SPDX-License-Identifier: Apache-2.0
#
# Self-contained two-node micro-ROS sample with a target-resident broker.

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third-party/micro_ros_zephyr_module/modules/libmicroros)

# Same SDK 1.0.0-beta1 + picolibc + rcl-headers Annex K dance.
add_compile_definitions(__STDC_WANT_LIB_EXT1__=0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(micro_ros_local)

target_sources(app PRIVATE
    src/main.c
    src/broker.c
    src/transport_loopback.c
    src/shims.c)
