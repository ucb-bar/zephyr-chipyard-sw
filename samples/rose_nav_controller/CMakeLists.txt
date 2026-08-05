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

# Optional demo bounds (override the settle window / iteration count for a bounded co-sim run).
if(DEFINED SETTLE_ITERS)
  target_compile_definitions(app PRIVATE SETTLE_ITERS=${SETTLE_ITERS})
endif()
if(DEFINED CTRL_ITERS)
  target_compile_definitions(app PRIVATE CTRL_ITERS=${CTRL_ITERS})
endif()

# DroNet vision navigation (-DROSE_DRONET_NAV=1). Compiles the generated quantized DroNet C
# (pass -DMODEL_DIR=<ModelBlaster generated/<target> dir>, -DMB_POOL_INC=<modelblaster_pool>)
# into the controller; a forward FPV frame -> DroNet -> setpoint (see src/main.cpp).
set(ROSE_DRONET_NAV 0 CACHE STRING "1=drive the TinyMPC setpoint from DroNet vision")
if(ROSE_DRONET_NAV)
  if(NOT DEFINED MODEL_DIR)
    message(FATAL_ERROR "ROSE_DRONET_NAV=1 needs -DMODEL_DIR=<generated DroNet C dir>")
  endif()
  get_filename_component(MODEL_DIR "${MODEL_DIR}" ABSOLUTE)
  target_sources(app PRIVATE
    ${MODEL_DIR}/model.c ${MODEL_DIR}/kernels.c ${MODEL_DIR}/weights.c ${MODEL_DIR}/buffers.c)
  target_include_directories(app PRIVATE ${MODEL_DIR})
  if(DEFINED MB_POOL_INC)
    target_include_directories(app PRIVATE ${MB_POOL_INC})
  endif()
  target_compile_definitions(app PRIVATE ROSE_DRONET_NAV=1)
  # DroNet conv weights overflow the gp-relative window — disable linker relaxation.
  zephyr_link_libraries(-Wl,--no-relax)
endif()

target_include_directories(app PRIVATE
  ${FC}/src
  ${TINYMPC}/external/admm/include
  ${TINYMPC}/external/matlib/include
  ${TINYMPC}/include
  ${TINYMPC}/examples)
