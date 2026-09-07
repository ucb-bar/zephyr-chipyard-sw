# SPDX-License-Identifier: Apache-2.0
# Saturn VRF disambiguation probe (missing v24-v31 vs m8-top-group decode).
# Pure integer vector (zve64x) -> built whole with the DroNet kernel march; any
# trap on the bench is the vle8.v itself. Needs misa.V=1 at runtime.
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rvv_vstart_probe)

target_sources(app PRIVATE src/main.c)
set_source_files_properties(src/main.c PROPERTIES
  COMPILE_OPTIONS "-march=rv64imac_zve64x;-mabi=lp64")
