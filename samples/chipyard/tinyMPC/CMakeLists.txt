# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lqr_riccati_rvv)

# Include directories
target_include_directories(app PRIVATE 
    .
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../kris/matlib/include/float32
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../kris/matlib/include
)

# Compiler definitions for RVV optimizations
target_compile_definitions(app PRIVATE 
    BATCH=4
)

# Add RVV-specific compiler flags
target_compile_options(app PRIVATE
    -march=rv64gcv
    -mabi=lp64d
    -O3
)

# Source files - use the standalone RVV implementation
target_sources(app PRIVATE 
    src/main.cpp
)