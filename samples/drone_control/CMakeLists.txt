# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(drone_control)


if(RISCV_VECTOR)
  set(USE_CPU OFF CACHE BOOL "Enable RISCV Scalar support")
  set(USE_RVV ON CACHE BOOL "Enable RVV support")
  set(USE_HANDOPT ON CACHE BOOL "Enable manual optimizations")
else()
  set(USE_CPU ON CACHE BOOL "Enable RISCV Scalar support")
  set(USE_RVV OFF CACHE BOOL "Enable RVV support")
  set(USE_HANDOPT OFF CACHE BOOL "Enable manual optimizations")
endif()



set(MEASURE_CYCLES OFF CACHE BOOL "Automatically Append Cycle Logging")
set(USE_TYPE "float32" CACHE STRING "Set floating point type for TinyMPC dependencies")

# Build the matlib dependency.
include(${CMAKE_CURRENT_SOURCE_DIR}/tinympc/external/matlib/matlib_header.cmake)


# Your main application sources.
target_sources(app PRIVATE src/main_mt_binary.cpp ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/external/admm/src/admm_rvv.cpp)
# target_sources(app PRIVATE src/main_test_binary.cpp ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/external/admm/src/admm_rvv.cpp)
# target_sources(app PRIVATE src/main_tracking.cpp ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/external/admm/src/admm_rvv.cpp)

# Include directories for admm and matlib headers.
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/external/admm/include
  ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/external/matlib/include
  ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/include
  ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/examples 
)

# Link the admm library into your Zephyr app.
target_link_libraries(app PRIVATE)
