# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flight_controller)

target_compile_definitions(app PRIVATE
    USE_CPU
    USE_TYPE=float32
)

set(ACCEL_TINY_MPC_DIR /home/mihir/Accelerated-TinyMPC)

# Add all source files
target_sources(app PRIVATE
    src/main.c
    src/attitude_estimator_madgwick.c
    src/control_task.cpp
    src/state.c
    ../drone_control/madgwick_test/src/madgwick_wrapper.cc
    #../drone_control/tinympc/external/admm/src/admm_rvv.cpp
    #../drone_control/tinympc/external/matlib/src/matlib.cpp
    #../drone_control/src/main_tracking.cpp 
    ${ACCEL_TINY_MPC_DIR}/external/admm/src/admm_matlib.cpp
    ${ACCEL_TINY_MPC_DIR}/external/matlib/src/matlib.cpp
)

#target_compile_definitions(app PRIVATE
#    USE_RVV
#)



# Include directories
target_include_directories(app PRIVATE
    #${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/eigen_shim
    #${ZEPHYR_BASE}/../modules/lib/eigen

    ${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/tinympc/examples
    ${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/tinympc/examples/problem_data

    ${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/madgwick_test/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/ento-bench/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../drone_control
    #${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/tinympc/include
    #${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/tinympc/external/admm/include
    #${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/tinympc/external/matlib/include

    ${ACCEL_TINY_MPC_DIR}/include
    ${ACCEL_TINY_MPC_DIR}/external/admm/include
    ${ACCEL_TINY_MPC_DIR}/external/matlib/include
    ${ACCEL_TINY_MPC_DIR}/external/matlib/include/float32
    #${ACCEL_TINY_MPC_DIR}/external/matlib/include/float64
    #${ACCEL_TINY_MPC_DIR}/external/matlib/include/int32
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../drone_control/eigen_shim
    ${ZEPHYR_BASE}/../modules/lib/eigen
)

# add_subdirectory(../drone_control/tinympc tinympc_build)

