# SPDX-License-Identifier: Apache-2.0
#
# Generic target toolchain configuration dispatcher for Zephyr SDK
# This file routes to either gnu or llvm target toolchain configuration
# based on the ZEPHYR_SDK_TOOLCHAIN_VARIANT variable or defaults to gnu

# Check if a specific SDK toolchain variant is requested
# This can be set via environment variable or CMake variable
if(DEFINED ENV{ZEPHYR_SDK_TOOLCHAIN_VARIANT})
  set(_sdk_toolchain_variant $ENV{ZEPHYR_SDK_TOOLCHAIN_VARIANT})
elseif(DEFINED ZEPHYR_SDK_TOOLCHAIN_VARIANT)
  set(_sdk_toolchain_variant ${ZEPHYR_SDK_TOOLCHAIN_VARIANT})
else()
  # Default to gnu if not specified
  set(_sdk_toolchain_variant "gnu")
endif()

# Include the appropriate target toolchain configuration
if(_sdk_toolchain_variant STREQUAL "llvm")
  include(${CMAKE_CURRENT_LIST_DIR}/llvm/target.cmake)
elseif(_sdk_toolchain_variant STREQUAL "gnu")
  include(${CMAKE_CURRENT_LIST_DIR}/gnu/target.cmake)
else()
  message(FATAL_ERROR "ZEPHYR_SDK_TOOLCHAIN_VARIANT must be either 'gnu' or 'llvm', got '${_sdk_toolchain_variant}'")
endif()

