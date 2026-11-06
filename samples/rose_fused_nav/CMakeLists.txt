# SPDX-License-Identifier: Apache-2.0
#
# RoSE fused-vision nav controller (M1 transport bring-up). Compiles the generated
# fused_full model C (int8 encoders + fp16 tail) into a Zephyr app that runs it on the
# SoC in the warehouse-gate co-sim. Build:
#
#   west build -p always -b spike_riscv64 samples/rose_fused_nav -- \
#     -DZEPHYR_EXTRA_MODULES=<zephyr-rose> \
#     -DMODEL_DIR=<gen dir> -DMB_POOL_INC=<modelblaster_pool dir>
#
# The model uses a Zfh fp16 tail: prj.conf sets CONFIG_RISCV_ISA_EXT_ZFH=y (threads _zfh
# into -march); run spike with --isa=rv64gc_zfh.
cmake_minimum_required(VERSION 3.20.0)
# RVV build (-DROSE_RVV=1): splice the V+Zfh+Zvfh overlay BEFORE find_package so its CONFIG_*
# reach Kconfig (target_riscv.cmake then threads v/_zfh/_zvfh into -march). Use with the curated
# rvv_f16 gen (MODEL_DIR=.../rvv_f16) and run spike with --isa=rv64gcv_zicntr_zihpm_zfh_zvfh.
if(ROSE_RVV)
  set(EXTRA_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/rvv.conf)
endif()
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rose_fused_nav)

if(ROSE_RVV)
  target_compile_definitions(app PRIVATE ROSE_RVV=1)
endif()

if(NOT DEFINED MODEL_DIR)
  message(FATAL_ERROR "rose_fused_nav needs -DMODEL_DIR=<generated fused_full C dir>")
endif()
get_filename_component(MODEL_DIR "${MODEL_DIR}" ABSOLUTE)

target_sources(app PRIVATE
  src/main.cpp
  ${MODEL_DIR}/model.c
  ${MODEL_DIR}/kernels.c
  ${MODEL_DIR}/weights.c
  ${MODEL_DIR}/buffers.c)

target_include_directories(app PRIVATE ${MODEL_DIR})
if(DEFINED MB_POOL_INC)
  target_include_directories(app PRIVATE ${MB_POOL_INC})
endif()

# 25 MB of fused_full weights overflow the gp-relative window — disable linker relaxation
# (same as the DroNet nav / modelblaster harness).
zephyr_link_libraries(-Wl,--no-relax)

# Bounded co-sim run length (override with -DCTRL_ITERS=N).
if(DEFINED CTRL_ITERS)
  target_compile_definitions(app PRIVATE CTRL_ITERS=${CTRL_ITERS})
endif()
