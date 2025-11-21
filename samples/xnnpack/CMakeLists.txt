# # SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(XNNPACK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third-party/XNNPACK)

# Set XNNPACK options before find_package(Zephyr) so we can use them to set Kconfig
# Use set() with CACHE to allow command-line overrides with -D
# Command-line values (passed with -D) will take precedence over these defaults
set(XNNPACK_ENABLE_RISCV_VECTOR ON CACHE BOOL "Build XNNPACK with RISC-V Vector micro-kernels")
set(XNNPACK_ENABLE_RISCV_GEMMINI OFF CACHE BOOL "Build XNNPACK with RISC-V Gemmini micro-kernels")
set(XNNPACK_BUILD_TESTS OFF CACHE BOOL "Build XNNPack unit tests")
set(XNNPACK_BUILD_BENCHMARKS OFF CACHE BOOL "Build XNNPack benchmarks")

# Conditionally include configuration fragment based on CMake flag
# This must be done before find_package(Zephyr) for Zephyr's Kconfig system to pick it up
set(CONF_FILE prj.conf)
if(XNNPACK_ENABLE_RISCV_VECTOR)
  list(APPEND CONF_FILE riscv_vector.conf)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xnnpack)

add_subdirectory(${XNNPACK_DIR} xnnpack_build)



add_library(compile_commands INTERFACE)
target_include_directories(compile_commands INTERFACE $<TARGET_PROPERTY:app,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(compile_commands INTERFACE $<TARGET_PROPERTY:app,INTERFACE_COMPILE_OPTIONS>)
target_compile_definitions(compile_commands INTERFACE $<TARGET_PROPERTY:app,INTERFACE_COMPILE_DEFINITIONS>)
add_dependencies(compile_commands zephyr_interface)
add_dependencies(compile_commands zephyr_property_target)
target_link_libraries(pthreadpool PUBLIC compile_commands)


target_include_directories(app PRIVATE ${XNNPACK_DIR}/include)
target_link_libraries(app PRIVATE XNNPACK pthreadpool)
target_sources(app PRIVATE src/main.c)



