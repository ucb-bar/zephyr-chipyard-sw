# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rose_drone_control)

# Reuse the TinyMPC controller from the sibling samples/drone_control sample
# (requires: git -C soc/sw/xpu-rt/zephyr-chipyard-sw submodule update --init
#  samples/drone_control/tinympc  -- see README.rst).
set(TINYMPC ${CMAKE_CURRENT_SOURCE_DIR}/../../drone_control/tinympc)

if(RISCV_VECTOR)
  set(USE_CPU OFF CACHE BOOL "Enable RISCV Scalar support")
  set(USE_RVV ON  CACHE BOOL "Enable RVV support")
  set(USE_HANDOPT ON CACHE BOOL "Enable manual optimizations")
else()
  set(USE_CPU ON  CACHE BOOL "Enable RISCV Scalar support")
  set(USE_RVV OFF CACHE BOOL "Enable RVV support")
  set(USE_HANDOPT OFF CACHE BOOL "Enable manual optimizations")
endif()
set(USE_TYPE "float32" CACHE STRING "Floating point type for TinyMPC")

include(${TINYMPC}/external/matlib/matlib_header.cmake)

target_sources(app PRIVATE
  src/main.cpp
  ${TINYMPC}/external/admm/src/admm_rvv.cpp)

target_include_directories(app PRIVATE
  ${TINYMPC}/external/admm/include
  ${TINYMPC}/external/matlib/include
  ${TINYMPC}/include
  ${TINYMPC}/examples)
