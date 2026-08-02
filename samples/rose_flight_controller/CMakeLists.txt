# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rose_flight_controller)

# Reuse the TinyMPC controller from samples/drone_control (requires:
#   git -C soc/sw/xpu-rt/zephyr-chipyard-sw submodule update --init
#   samples/drone_control/tinympc  -- see README.rst).
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

# State estimator selection (modular IStateEstimator; both are compiled, factory picks):
#   -DROSE_USE_EKF=0 -> complementary (quaternion Mahony + fixed-gain + lead compensation)
#   -DROSE_USE_EKF=1 -> EKF (quaternion Mahony + per-axis Kalman)
# Default is the complementary filter: with lead compensation it holds a clean altitude
# hover; the EKF is provided as a pluggable option (still being tuned for the horizontal
# channel). Both share the Mahony attitude core (attitude_mahony.hpp).
set(ROSE_USE_EKF 0 CACHE STRING "1=EKF, 0=complementary filter")

target_sources(app PRIVATE
  src/main.cpp
  src/estimator_complementary.cpp
  src/estimator_ekf.cpp
  src/estimator_factory.cpp
  ${TINYMPC}/external/admm/src/admm_rvv.cpp)

target_compile_definitions(app PRIVATE ROSE_USE_EKF=${ROSE_USE_EKF})

target_include_directories(app PRIVATE
  ${TINYMPC}/external/admm/include
  ${TINYMPC}/external/matlib/include
  ${TINYMPC}/include
  ${TINYMPC}/examples)
