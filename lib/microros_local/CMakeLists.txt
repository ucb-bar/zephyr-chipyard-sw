# SPDX-License-Identifier: Apache-2.0
#
# microros_local — in-target XRCE-DDS broker + Zephyr-k_msgq custom transport
# for self-contained micro-ROS apps.
#
# Wired in as a Zephyr module via lib/microros_local/zephyr/module.yml. A
# consumer sample lists it in ZEPHYR_EXTRA_MODULES alongside libmicroros,
# before find_package(Zephyr) runs:
#
#   list(APPEND ZEPHYR_EXTRA_MODULES
#       ${REPO_ROOT}/third-party/micro_ros_zephyr_module/modules/libmicroros
#       ${REPO_ROOT}/lib/microros_local)
#
# That way the library is processed in the same phase as libmicroros (so the
# `microros` interface library is in scope for include propagation), and the
# zephyr_library_named'd library is auto-linked into the final image.
#
# Public API headers live under include/microros_local/ — consumers include
# them as <microros_local/broker.h>, <microros_local/transport_loopback.h>.

zephyr_library_named(microros_local)

zephyr_library_sources(
    src/broker.c
    src/transport_loopback.c
)

# Public include path — both the library's own sources and any consumer
# of microros_local pick up <microros_local/*.h> from the same place.
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Propagate libmicroros's include dirs (ucdr/microcdr.h, rmw_microros/*, etc.).
zephyr_library_link_libraries(microros)
