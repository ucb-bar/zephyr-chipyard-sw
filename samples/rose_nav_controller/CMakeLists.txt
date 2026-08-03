# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rose_nav_controller)

# Reuse the flight controller's estimator (incl. the wall-fusion EKF) + TinyMPC.
set(FC ${CMAKE_CURRENT_SOURCE_DIR}/../rose_flight_controller)
set(TINYMPC ${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/tinympc)

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

# EKF is required for wall fusion (fuse_walls is a no-op in the complementary filter).
set(ROSE_USE_EKF 1 CACHE STRING "1=EKF (wall fusion), 0=complementary")

# Navigation mode: 0 = waypoint (ramp an x position setpoint), 1 = velocity (constant cruise).
set(NAV_MODE 0 CACHE STRING "0=waypoint position control, 1=velocity/cruise control")

target_sources(app PRIVATE
  src/main.cpp
  ${FC}/src/estimator_complementary.cpp
  ${FC}/src/estimator_ekf.cpp
  ${FC}/src/estimator_factory.cpp
  ${TINYMPC}/external/admm/src/admm_rvv.cpp)

target_compile_definitions(app PRIVATE ROSE_USE_EKF=${ROSE_USE_EKF} NAV_MODE=${NAV_MODE})

target_include_directories(app PRIVATE
  ${FC}/src
  ${TINYMPC}/external/admm/include
  ${TINYMPC}/external/matlib/include
  ${TINYMPC}/include
  ${TINYMPC}/examples)
