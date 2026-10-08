# Minimal Zephyr app that calls gemmini-rocc-tests's tiled_matmul_auto from
# inside a Zephyr thread. The point: prove that LOOP_WS works (or fails) under
# Zephyr's runtime context, independent of IREE/ELF-loading.
#
# Uses gemmini.h from gemmini-rocc-tests directly.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_gemmini_canonical_test)

set(GEMMINI_ROCC_TESTS_ROOT
    /scratch2/agustin/chipyard/generators/gemmini/software/gemmini-rocc-tests)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE
  ${GEMMINI_ROCC_TESTS_ROOT}
  ${GEMMINI_ROCC_TESTS_ROOT}/include
)
