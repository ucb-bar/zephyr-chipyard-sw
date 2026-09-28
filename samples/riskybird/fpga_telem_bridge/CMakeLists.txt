# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fpga_telem_bridge)

target_sources(app PRIVATE src/main.c)

# Embed the shared mobile flight page (gzipped) so the ESP serves it from flash at GET /.
set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated)
generate_inc_file_for_target(app
  ${CMAKE_CURRENT_SOURCE_DIR}/../../rose_flight_controller/tools/groundstation/mobile/mobile.html
  ${gen_dir}/mobile_html.gz.inc --gzip)

# Per-service iterable ROM section for the HTTP resources (see src/sections-rom.ld).
zephyr_linker_sources(SECTIONS src/sections-rom.ld)
zephyr_linker_section(NAME http_resource_desc_rb_http KVMA RAM_REGION GROUP RODATA_REGION)
