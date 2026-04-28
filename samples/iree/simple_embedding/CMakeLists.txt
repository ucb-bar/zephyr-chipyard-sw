cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(iree_simple_embedding)

# XPURT repo root (four levels up from this CMakeLists.txt).
set(XPURT_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)

# IREE runtime headers from the merlin third_party checkout.
set(IREE_RUNTIME_INCLUDE_DIR
    ${XPURT_ROOT}/merlin/third_party/iree_bar/runtime/src)

option(IREE_BUILD_RUNTIME_WITH_WEST "Build IREE runtime as part of west build" ON)

set(IREE_SRC_DIR ${XPURT_ROOT}/merlin/third_party/iree_bar)
set(MERLIN_ZEPHYR_STANDALONE_CMAKE ${XPURT_ROOT}/merlin/zephyr-rt/CMakeLists_standalone.cmake)
set(IREE_COMPILE_EXE
    ${XPURT_ROOT}/merlin/build/host-vanilla-release/install/bin/iree-compile)
set(SIMPLE_EMBEDDING_TEST_MLIR
    ${XPURT_ROOT}/merlin/third_party/iree_bar/samples/simple_embedding/simple_embedding_test.mlir)
set(SIMPLE_EMBEDDING_VMFB
    ${CMAKE_CURRENT_LIST_DIR}/src/model.vmfb)
set(SIMPLE_EMBEDDING_VMFB_HEADER
    ${CMAKE_CURRENT_LIST_DIR}/src/model_vmfb.h)

find_program(XXD_EXECUTABLE xxd REQUIRED)

#
# VMFB codegen knobs
# ------------------
option(IREE_ENABLE_RISCV_VECTOR "Compile VMFB with RISC-V Vector extension (+v)" OFF)
if(IREE_ENABLE_RISCV_VECTOR)
  set(IREE_RISCV_CPU_FEATURES "+m,+a,+f,+d,+c,+v,+zvl128b")
  # Keep a fixed vector width for deterministic codegen.
  set(IREE_RISCV_VECTOR_WIDTH_FLAG "--iree-llvmcpu-target-vector-width-in-bytes=16")
else()
  set(IREE_RISCV_CPU_FEATURES "+m,+a,+f,+d,+c")
  set(IREE_RISCV_VECTOR_WIDTH_FLAG "")
endif()

if(NOT EXISTS ${IREE_COMPILE_EXE})
  message(FATAL_ERROR
    "Missing iree-compile at ${IREE_COMPILE_EXE}. "
    "Build host tools first: python3 tools/build.py --target host --config release")
endif()

add_custom_command(
  OUTPUT
    ${SIMPLE_EMBEDDING_VMFB}
    ${SIMPLE_EMBEDDING_VMFB_HEADER}
  COMMAND
    ${IREE_COMPILE_EXE}
    ${SIMPLE_EMBEDDING_TEST_MLIR}
    --iree-hal-target-device=local
    --iree-hal-local-target-device-backends=llvm-cpu
    --iree-llvmcpu-target-triple=riscv64-unknown-elf
    --iree-llvmcpu-target-cpu-features=${IREE_RISCV_CPU_FEATURES}
    --iree-llvmcpu-target-abi=lp64d
    ${IREE_RISCV_VECTOR_WIDTH_FLAG}
    --iree-vm-bytecode-module-strip-source-map=true
    --iree-vm-emit-polyglot-zip=false
    -o ${SIMPLE_EMBEDDING_VMFB}
  COMMAND
    ${XXD_EXECUTABLE} -i model.vmfb > model_vmfb.h
  DEPENDS
    ${SIMPLE_EMBEDDING_TEST_MLIR}
    ${IREE_COMPILE_EXE}
  WORKING_DIRECTORY
    ${CMAKE_CURRENT_LIST_DIR}/src
  COMMENT
    "Generating embedded VMFB and C header for simple_embedding"
  VERBATIM
)

add_custom_target(iree_simple_embedding_vmfb
  DEPENDS
    ${SIMPLE_EMBEDDING_VMFB}
    ${SIMPLE_EMBEDDING_VMFB_HEADER}
)

target_sources(app PRIVATE
  src/simple_embedding.c
  src/simple_embedding_zephyr_stubs.c
)
add_dependencies(app iree_simple_embedding_vmfb)

target_include_directories(app PRIVATE
  ${IREE_RUNTIME_INCLUDE_DIR}
)

# Build IREE as a generic bare-metal-style platform when used under Zephyr.
target_compile_definitions(app PRIVATE
  IREE_PLATFORM_GENERIC=1
  IREE_ALLOCATOR_SYSTEM_CTL=iree_allocator_libc_ctl
)

if(IREE_BUILD_RUNTIME_WITH_WEST)
  include(ExternalProject)

  # Build IREE runtime in a sub-build so it uses Zephyr's toolchain settings.
  # Outputs are then linked into the Zephyr app.
  set(IREE_SUBBUILD_DIR ${CMAKE_BINARY_DIR}/iree_runtime_subbuild)
  set(IREE_HOST_BIN_DIR
      ${XPURT_ROOT}/merlin/build/host-vanilla-release/install/bin)
  set(IREE_ZEPHYR_COMPAT_HEADER
      ${CMAKE_CURRENT_LIST_DIR}/src/iree_zephyr_compat.h)
  # Explicit ISA/ABI flags to match spike_riscv64 Zephyr builds.
  # The ExternalProject configure does not inherit app target compile options.
  set(IREE_SUBBUILD_ARCH_FLAGS
      "-march=rv64imafdc_zicsr_zifencei -mabi=lp64d -mcmodel=medany")
  # Preserve Zephyr-selected ISA/ABI flags (e.g. -march/-mabi) for the
  # ExternalProject sub-build. Without this, GCC may default to rv32, causing
  # ELFCLASS32 vs ELFCLASS64 link incompatibilities.
  set(IREE_SUBBUILD_C_FLAGS
      "${CMAKE_C_FLAGS} ${IREE_SUBBUILD_ARCH_FLAGS} -DIREE_PLATFORM_GENERIC=1 -Wno-error=format -include ${IREE_ZEPHYR_COMPAT_HEADER}")
  set(IREE_SUBBUILD_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} ${IREE_SUBBUILD_ARCH_FLAGS} -DIREE_PLATFORM_GENERIC=1 -Wno-error=format -include ${IREE_ZEPHYR_COMPAT_HEADER}")
  set(IREE_RUNTIME_LIB
      ${IREE_SUBBUILD_DIR}/runtime/src/iree/runtime/libzephyr_iree_runtime_standalone.a)
  set(IREE_FLATCC_LIB
      ${IREE_SUBBUILD_DIR}/build_tools/third_party/flatcc/libflatcc_parsing.a)

  ExternalProject_Add(iree_runtime_ep
    SOURCE_DIR ${IREE_SRC_DIR}
    BINARY_DIR ${IREE_SUBBUILD_DIR}
    CMAKE_ARGS
      -G Ninja
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_DISABLE_FIND_PACKAGE_CUDAToolkit=ON
      -DCMAKE_SYSTEM_NAME=Generic
      -DCMAKE_SYSTEM_PROCESSOR=riscv64
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_ASM_COMPILER=${CMAKE_ASM_COMPILER}
      -DCMAKE_AR=${CMAKE_AR}
      -DCMAKE_RANLIB=${CMAKE_RANLIB}
      -DCMAKE_STRIP=${CMAKE_STRIP}
      -DCMAKE_C_FLAGS=${IREE_SUBBUILD_C_FLAGS}
      -DCMAKE_CXX_FLAGS=${IREE_SUBBUILD_CXX_FLAGS}
      -DMERLIN_XPU_RT_STANDALONE_CMAKE=${MERLIN_ZEPHYR_STANDALONE_CMAKE}
      -DMERLIN_BUILD_SPACEMITX60=OFF
      -DMERLIN_BUILD_SATURN_OPU=OFF
      -DIREE_ARCH=riscv_64
      -DIREE_HOST_BIN_DIR=${IREE_HOST_BIN_DIR}
      -DIREE_BUILD_COMPILER=OFF
      -DIREE_BUILD_SAMPLES=OFF
      -DIREE_BUILD_TESTS=OFF
      -DIREE_BUILD_WARNINGS_AS_ERRORS=OFF
      -DIREE_ENABLE_COMPILER_WARNINGS=OFF
      -DIREE_BUILD_BINDINGS_TFLITE=OFF
      -DIREE_BUILD_BINDINGS_TFLITE_JAVA=OFF
      -DIREE_ENABLE_THREADING=OFF
      -DIREE_HAL_DRIVER_LOCAL_SYNC=ON
      -DIREE_HAL_DRIVER_DEFAULTS=OFF
      -DIREE_HAL_EXECUTABLE_LOADER_DEFAULTS=OFF
      -DIREE_HAL_EXECUTABLE_LOADER_EMBEDDED_ELF=ON
      -DIREE_HAL_EXECUTABLE_PLUGIN_DEFAULTS=OFF
      -DIREE_HAL_EXECUTABLE_PLUGIN_EMBEDDED_ELF=ON
    BUILD_COMMAND ${CMAKE_COMMAND} --build ${IREE_SUBBUILD_DIR} --target zephyr_iree_runtime_standalone -v
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS
      ${IREE_RUNTIME_LIB}
      ${IREE_FLATCC_LIB}
  )

  add_library(iree_runtime_standalone STATIC IMPORTED GLOBAL)
  set_target_properties(iree_runtime_standalone PROPERTIES IMPORTED_LOCATION ${IREE_RUNTIME_LIB})
  add_dependencies(iree_runtime_standalone iree_runtime_ep)

  add_library(iree_flatcc_parsing STATIC IMPORTED GLOBAL)
  set_target_properties(iree_flatcc_parsing PROPERTIES IMPORTED_LOCATION ${IREE_FLATCC_LIB})
  add_dependencies(iree_flatcc_parsing iree_runtime_ep)

  target_link_libraries(app PRIVATE
    iree_runtime_standalone
    iree_flatcc_parsing
  )
else()
  # Prebuilt standalone IREE runtime archive for Zephyr (manual build).
  set(IREE_ZEPHYR_RUNTIME_LIB
      ${XPURT_ROOT}/merlin/build-iree-zephyr-runtime-zephyr/runtime/src/iree/runtime/libzephyr_iree_runtime_standalone.a)
  set(IREE_FLATCC_RUNTIME_LIB
      ${XPURT_ROOT}/merlin/build-iree-zephyr-runtime-zephyr/build_tools/third_party/flatcc/libflatcc_parsing.a)
  target_link_libraries(app PRIVATE
    ${IREE_ZEPHYR_RUNTIME_LIB}
    ${IREE_FLATCC_RUNTIME_LIB}
  )
endif()


