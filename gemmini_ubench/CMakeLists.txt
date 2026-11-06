# SPDX-License-Identifier: Apache-2.0
#
# Standalone Gemmini conv microbenchmark for the riskybird v3 FPGA Rocket
# combined config (RocketArty200TDroneGemminiSaturnFp16At35Config, DIM=16,
# Q0.31 acc-scale, Gemmini + Saturn both live).
#
# Compares, per conv shape, three ways of running the SAME convolution and
# prints rdcycle deltas as CSV:
#   A) direct LoopConv        : tiled_conv_auto            (HW im2col+GEMM+requant)
#   B) LoopMatmul (baseline)  : SW im2col (DIM-16 row tiles) + tiled_matmul_auto
#   C) LoopMatmul (1 big GEMM): SW im2col (all rows)        + one tiled_matmul_auto
#
# Self-contained: the board conf + DT overlay for this bitstream are applied
# automatically from boards/chipyard_riscv64.{conf,overlay}. Just:
#   west build -p -b chipyard_riscv64 gemmini_ubench     (see build.sh)

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gemmini_conv_ubench)

target_sources(app PRIVATE src/main.c src/bench.c)

# Same linker treatment the modelblaster harness uses: big .bss buffers push
# past the gp-relative window, so disable relaxation.
zephyr_link_libraries(-Wl,--no-relax)

# Gemmini kernel flags — identical to the FPGA hetero build's kernels.c
# (compile_commands.json from build_mb_dronet_hetero_fpga): Q0.31 acc-scale,
# HWIO weights, integer ISA (no hardware F/D on this bitstream). bench.c is
# SCALAR (no vector) on purpose — the software rearrangement it does is not
# the RVV path the shipped im2col kernel uses; only the GEMM / tiled_conv
# hardware numbers are load-bearing (see the header in src/bench.c).
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set_source_files_properties(src/bench.c PROPERTIES COMPILE_OPTIONS
  "-march=rv64imac_zicsr_zifencei;-mabi=lp64;\
-isystem${REPO}/modelblaster/cores/gemmini/include/per_config/default16x16;\
-isystem${REPO}/cores/gemmini/include;\
-isystem${REPO}/cores/gemmini;\
-DGEMMINI_ROCC;-DBAREMETAL;\
-DMODELBLASTER_GEMMINI_HWIO_WEIGHTS=1;-DMODELBLASTER_GEMMINI_Q31_ACC_SCALE=1")
