# merlin_hetero_runner — runs the same model in parallel on BOTH harts of the
# FireSim Shuttle bitstream (hart 0 = scalar/Gemmini, hart 1 = RVV/OPU).
#
# Each hart gets its own ISA-specific VMFB; both run independently via a
# per-hart pinned worker thread + a per-hart k_msgq/k_sem job queue.
# Init (VM instance, HAL device, parsed bytecode) happens ONCE at worker
# startup so the ~575 M setup cycles are amortized across N inferences.
#
# Parameters (-D...):
#   MERLIN_VMFB_H0    absolute path to .vmfb compiled for hart 0 (REQUIRED)
#   MERLIN_VMFB_H1    absolute path to .vmfb compiled for hart 1 (REQUIRED)
#   MERLIN_MODEL      mlp_wide | dronet | yolov8n (REQUIRED, both VMFBs must
#                     be the same model)
#   MERLIN_BACKEND_H0 scalar | rvv | opu | gemmini (default: scalar)
#   MERLIN_BACKEND_H1 scalar | rvv | opu | gemmini (default: rvv)
#   MERLIN_JOBS       total jobs to dispatch across both harts (default: 32)
#   MERLIN_CPU_FEATURES  IREE bare-metal feature bitmask (default: 0x11 = V+XOPU)

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(merlin_hetero_runner)

# ----- required + defaulted parameters ----------------------------------
foreach(_var IN ITEMS MERLIN_VMFB_H0 MERLIN_VMFB_H1 MERLIN_MODEL)
  if(NOT DEFINED ${_var})
    message(FATAL_ERROR "${_var} must be set")
  endif()
endforeach()
foreach(_path IN ITEMS ${MERLIN_VMFB_H0} ${MERLIN_VMFB_H1})
  if(NOT EXISTS ${_path})
    message(FATAL_ERROR "Missing VMFB: ${_path}")
  endif()
endforeach()

if(MERLIN_MODEL STREQUAL "mlp_wide")
  set(MERLIN_MODEL_DEFINE MERLIN_MODEL_MLP_WIDE)
elseif(MERLIN_MODEL STREQUAL "dronet")
  set(MERLIN_MODEL_DEFINE MERLIN_MODEL_DRONET)
elseif(MERLIN_MODEL STREQUAL "dronet_with_intermediate")
  set(MERLIN_MODEL_DEFINE MERLIN_MODEL_DRONET_WITH_INTERMEDIATE)
elseif(MERLIN_MODEL STREQUAL "yolov8n")
  set(MERLIN_MODEL_DEFINE MERLIN_MODEL_YOLOV8N)
else()
  message(FATAL_ERROR "MERLIN_MODEL=${MERLIN_MODEL} invalid")
endif()

if(NOT DEFINED MERLIN_BACKEND_H0)
  set(MERLIN_BACKEND_H0 "scalar")
endif()
if(NOT DEFINED MERLIN_BACKEND_H1)
  set(MERLIN_BACKEND_H1 "rvv")
endif()
if(NOT DEFINED MERLIN_JOBS)
  set(MERLIN_JOBS 32)
endif()
if(NOT DEFINED MERLIN_CPU_FEATURES)
  set(MERLIN_CPU_FEATURES "0x11")
endif()

# Hart 0 is scalar/gemmini (no V); hart 1 is rvv/opu (V active).
set(MERLIN_NEEDS_VECTOR_H0 0)
if(MERLIN_BACKEND_H1 STREQUAL "rvv" OR MERLIN_BACKEND_H1 STREQUAL "opu")
  set(MERLIN_NEEDS_VECTOR_H1 1)
else()
  set(MERLIN_NEEDS_VECTOR_H1 0)
endif()

message(STATUS "merlin_hetero_runner config:")
message(STATUS "  MODEL              = ${MERLIN_MODEL}")
message(STATUS "  VMFB_H0            = ${MERLIN_VMFB_H0}  (backend ${MERLIN_BACKEND_H0})")
message(STATUS "  VMFB_H1            = ${MERLIN_VMFB_H1}  (backend ${MERLIN_BACKEND_H1})")
message(STATUS "  JOBS               = ${MERLIN_JOBS}")
message(STATUS "  CPU_FEATURES       = ${MERLIN_CPU_FEATURES}")

# ----- embed both VMFBs as C headers via xxd ----------------------------
find_program(XXD_EXECUTABLE xxd REQUIRED)

set(_H0_LOCAL  ${CMAKE_CURRENT_BINARY_DIR}/merlin_vmfb_h0.vmfb)
set(_H1_LOCAL  ${CMAKE_CURRENT_BINARY_DIR}/merlin_vmfb_h1.vmfb)
set(_H0_HEADER ${CMAKE_CURRENT_BINARY_DIR}/merlin_vmfb_h0.h)
set(_H1_HEADER ${CMAKE_CURRENT_BINARY_DIR}/merlin_vmfb_h1.h)

add_custom_command(
  OUTPUT ${_H0_LOCAL} ${_H0_HEADER}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${MERLIN_VMFB_H0} ${_H0_LOCAL}
  COMMAND ${XXD_EXECUTABLE} -i -n merlin_vmfb_h0 merlin_vmfb_h0.vmfb > ${_H0_HEADER}
  DEPENDS ${MERLIN_VMFB_H0}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Embedding hart-0 VMFB ${MERLIN_VMFB_H0}"
  VERBATIM)
add_custom_command(
  OUTPUT ${_H1_LOCAL} ${_H1_HEADER}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${MERLIN_VMFB_H1} ${_H1_LOCAL}
  COMMAND ${XXD_EXECUTABLE} -i -n merlin_vmfb_h1 merlin_vmfb_h1.vmfb > ${_H1_HEADER}
  DEPENDS ${MERLIN_VMFB_H1}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Embedding hart-1 VMFB ${MERLIN_VMFB_H1}"
  VERBATIM)
add_custom_target(merlin_vmfb_headers DEPENDS ${_H0_HEADER} ${_H1_HEADER})

# ----- application sources ----------------------------------------------
target_sources(app PRIVATE
  src/main.c
  src/hetero_worker.c
  src/device_stubs.c
  src/iree_thread_zephyr.c
  src/iree_futex_zephyr.c
  src/iree_picolibc_compat.c
)
add_dependencies(app merlin_vmfb_headers)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_definitions(app PRIVATE
  ${MERLIN_MODEL_DEFINE}
  MERLIN_BACKEND_H0="${MERLIN_BACKEND_H0}"
  MERLIN_BACKEND_H1="${MERLIN_BACKEND_H1}"
  MERLIN_NEEDS_VECTOR_H0=${MERLIN_NEEDS_VECTOR_H0}
  MERLIN_NEEDS_VECTOR_H1=${MERLIN_NEEDS_VECTOR_H1}
  MERLIN_JOBS=${MERLIN_JOBS}
)
# 2026-05-17: enable per-dispatch cycle profiling so [dc] o=N wg=...
# cyc=C lines have real values. This wraps every ELF call with
# rdcycle and accumulates into iree_merlin_cycles_per_ordinal[].
target_compile_definitions(merlin_iree INTERFACE MERLIN_PROFILE_CYCLES=1)

target_link_libraries(app PRIVATE merlin_iree)
target_compile_definitions(app PRIVATE
  IREE_BARE_METAL_CPU_DATA0=${MERLIN_CPU_FEATURES}
)
