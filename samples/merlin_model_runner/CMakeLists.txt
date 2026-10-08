# merlin_model_runner — runs a pre-compiled Merlin VMFB on a FireSim Shuttle
# hart and reports an output hash + cycle count.
#
# Parameters (-D...):
#   MERLIN_VMFB           absolute path to the .vmfb to embed (REQUIRED)
#   MERLIN_MODEL          mlp_wide | dronet | yolov8n (REQUIRED)
#   MERLIN_BACKEND        scalar | rvv | opu | gemmini (log-only string)
#   MERLIN_HART           0 or 1 (defaults to 0)
#   MERLIN_CPU_FEATURES   IREE bare-metal feature bitmask, e.g. 0x11 for OPU
#                         (defaults to 0x01 = RVV only)
#
# IREE runtime is pulled in via the `merlin-iree` Zephyr module that ships
# in chipyard's zephyrproject (matching the convention scripts/zephyr_e2e.sh
# uses). Prereq: `./merlin build --profile zephyr` populates
# $MERLIN_ROOT/build/zephyr-vanilla-release/ before this sample is built.
#
# West invocation:
#   west build -p -b chipyard_riscv64/rocketchip_virt_riscv64 \
#       samples/merlin_model_runner \
#       --build-dir <build> \
#       -- -DZEPHYR_EXTRA_MODULES=$CHIPYARD_ROOT/software/zephyrproject/modules/merlin-iree \
#          -DMERLIN_BUILD_DIR=$MERLIN_ROOT/build/zephyr-vanilla-release \
#          -DMERLIN_IREE_HEADERS_DIR=$MERLIN_ROOT/third_party/iree_bar/runtime/src \
#          -DMERLIN_VMFB=<path>.vmfb \
#          -DMERLIN_MODEL=<name> -DMERLIN_BACKEND=<name> -DMERLIN_HART=<0|1> \
#          -DMERLIN_CPU_FEATURES=<hex>

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(merlin_model_runner)

# ----- required + defaulted parameters ----------------------------------
if(NOT DEFINED MERLIN_VMFB)
  message(FATAL_ERROR "MERLIN_VMFB must be set (path to compiled .vmfb)")
endif()
if(NOT EXISTS ${MERLIN_VMFB})
  message(FATAL_ERROR "MERLIN_VMFB does not exist: ${MERLIN_VMFB}")
endif()

if(NOT DEFINED MERLIN_MODEL)
  message(FATAL_ERROR
    "MERLIN_MODEL must be set (one of: mlp_wide, dronet, yolov8n)")
endif()
if(MERLIN_MODEL STREQUAL "tiny_matmul")
  set(MERLIN_MODEL_DEFINE MERLIN_MODEL_TINY_MATMUL)
elseif(MERLIN_MODEL STREQUAL "mlp_wide")
  set(MERLIN_MODEL_DEFINE MERLIN_MODEL_MLP_WIDE)
elseif(MERLIN_MODEL STREQUAL "dronet")
  set(MERLIN_MODEL_DEFINE MERLIN_MODEL_DRONET)
elseif(MERLIN_MODEL STREQUAL "yolov8n")
  set(MERLIN_MODEL_DEFINE MERLIN_MODEL_YOLOV8N)
else()
  message(FATAL_ERROR
    "MERLIN_MODEL=${MERLIN_MODEL} invalid; must be tiny_matmul|mlp_wide|dronet|yolov8n")
endif()

if(NOT DEFINED MERLIN_BACKEND)
  set(MERLIN_BACKEND "unknown")
endif()
if(NOT DEFINED MERLIN_HART)
  set(MERLIN_HART 0)
endif()
if(NOT DEFINED MERLIN_CPU_FEATURES)
  set(MERLIN_CPU_FEATURES "0x01")
endif()

# rvv/opu backends need mstatus.VS=Dirty per lesson 4 of the 2026-05-13 blog.
if(MERLIN_BACKEND STREQUAL "rvv" OR MERLIN_BACKEND STREQUAL "opu")
  set(MERLIN_NEEDS_VECTOR 1)
else()
  set(MERLIN_NEEDS_VECTOR 0)
endif()

message(STATUS "merlin_model_runner config:")
message(STATUS "  MERLIN_VMFB          = ${MERLIN_VMFB}")
message(STATUS "  MERLIN_MODEL         = ${MERLIN_MODEL} (-> ${MERLIN_MODEL_DEFINE})")
message(STATUS "  MERLIN_BACKEND       = ${MERLIN_BACKEND}")
message(STATUS "  MERLIN_HART          = ${MERLIN_HART}")
message(STATUS "  MERLIN_CPU_FEATURES  = ${MERLIN_CPU_FEATURES}")
message(STATUS "  MERLIN_NEEDS_VECTOR  = ${MERLIN_NEEDS_VECTOR}")

# ----- embed VMFB as a C header via xxd ---------------------------------
find_program(XXD_EXECUTABLE xxd REQUIRED)

set(MERLIN_VMFB_LOCAL  ${CMAKE_CURRENT_BINARY_DIR}/merlin_model.vmfb)
set(MERLIN_VMFB_HEADER ${CMAKE_CURRENT_BINARY_DIR}/merlin_vmfb.h)

add_custom_command(
  OUTPUT ${MERLIN_VMFB_LOCAL} ${MERLIN_VMFB_HEADER}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${MERLIN_VMFB} ${MERLIN_VMFB_LOCAL}
  COMMAND ${XXD_EXECUTABLE} -i -n merlin_vmfb merlin_model.vmfb > ${MERLIN_VMFB_HEADER}
  DEPENDS ${MERLIN_VMFB}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Embedding VMFB ${MERLIN_VMFB} -> merlin_vmfb.h"
  VERBATIM)
add_custom_target(merlin_vmfb_header DEPENDS ${MERLIN_VMFB_HEADER})

# ----- application sources ----------------------------------------------
# iree_thread_zephyr.c / iree_futex_zephyr.c provide the implementations
# of iree_thread_* / iree_futex_*_impl_zephyr referenced by the IREE
# archives (which were built with IREE_PLATFORM_ZEPHYR + threading=ON
# but without Zephyr's kernel headers). Linked at the final app link.
target_sources(app PRIVATE
  src/main.c
  src/device_stubs.c
  src/iree_thread_zephyr.c     # iree_thread_* -> k_thread_*
  src/iree_futex_zephyr.c      # iree_futex_*  -> k_mutex+k_condvar bucket pool
  src/iree_picolibc_compat.c   # newlib -> picolibc symbol bridge
)
add_dependencies(app merlin_vmfb_header)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}    # for generated merlin_vmfb.h
  ${CMAKE_CURRENT_LIST_DIR}/src  # for model_info.h
)

target_compile_definitions(app PRIVATE
  ${MERLIN_MODEL_DEFINE}
  MERLIN_BACKEND="${MERLIN_BACKEND}"
  MERLIN_HART=${MERLIN_HART}
  MERLIN_NEEDS_VECTOR=${MERLIN_NEEDS_VECTOR}
)

# 2026-05-22: enable per-dispatch cycle profiling. This wraps every ELF
# call in embedded_elf_loader.c with rdcycle, accumulates into
# iree_merlin_cycles_per_ordinal[]. iree_merlin_dump_cycles() is then
# invoked from main.c to emit `CYC, ord, sym, total_cycles, wg_count`
# lines we can parse offline into a per-dispatch profile table.
# The merlin_hetero_runner sample already had this; merlin_model_runner
# did not.
target_compile_definitions(merlin_iree INTERFACE MERLIN_PROFILE_CYCLES=1)

# Per-dispatch Gemmini hardware counter profiling. Opt-in to match the
# runtime archive build (see iree_bar runtime CMakeLists.txt for the
# MERLIN_GEMMINI_COUNTERS option). Enable with
# `west build -- -DMERLIN_GEMMINI_COUNTERS=ON`.
if(MERLIN_GEMMINI_COUNTERS)
  target_compile_definitions(merlin_iree INTERFACE MERLIN_PROFILE_COUNTERS=1)
endif()

# ----- merlin-iree Zephyr module (provides IREE static runtime) ---------
# The merlin-iree Kconfig option is auto-enabled by our prj.conf
# (CONFIG_MERLIN_IREE=y). The module CMakeLists imports the IREE static
# archives from $MERLIN_BUILD_DIR (or $MERLIN_INSTALL_DIR), exposes them as
# the `merlin_iree` interface library, and sets the necessary
# IREE_PLATFORM_GENERIC / IREE_FILE_IO_ENABLE / IREE_WAIT_UNTIL_FN /
# atomics-shim defines on the consumer. No ExternalProject build, no
# duplicated IREE compilation.
target_link_libraries(app PRIVATE merlin_iree)

# IREE_BARE_METAL_CPU_DATA0 must be visible to consumer code too (the
# inline ukernel dispatcher reads it from cpu_data). The merlin-iree
# archives are pre-baked with whatever bitmask `./merlin build --profile
# zephyr` was invoked with; setting it here matches that baseline.
target_compile_definitions(app PRIVATE
  IREE_BARE_METAL_CPU_DATA0=${MERLIN_CPU_FEATURES}
)
