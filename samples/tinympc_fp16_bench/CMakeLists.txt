# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tinympc_fp16_bench)

set(TINY ${CMAKE_CURRENT_SOURCE_DIR}/tinympc)

# fp16 vectorized matlib path:
#  USE_RVV       -> RVV kernels
#  USE_TYPE_FP16 -> tinytype = _Float16 (matlib_types.h)
#  MATLIB_NARROW_ACC -> fp16 dot-product accumulator (no scalar fp32 -> no trap on misa.F=0)
add_compile_definitions(
  USE_RVV=1
  USE_MATVEC=1
  USE_LMUL=1
  MEMORY=1
  OPTIMIZED=1
  USE_TYPE_FP16=1
  MATLIB_NARROW_ACC=1
)

target_sources(app PRIVATE
  src/main.cpp
  ${TINY}/external/admm/src/admm_rvv.cpp)

target_include_directories(app PRIVATE
  ${TINY}/external/admm/include
  ${TINY}/external/matlib/include
  ${TINY}/examples)
