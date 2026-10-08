# Injected via -DCMAKE_PROJECT_INCLUDE into the IREE ExternalProject sub-build.
#
# Upstream `iree_bar/runtime/src/iree/builtins/ukernel/arch/riscv_64/CMakeLists.txt`
# calls `check_c_source_compiles(...)` (line 153) without an
# `include(CheckCSourceCompiles)`. The same file uses
# `check_cxx_compiler_flag` without including `CheckCXXCompilerFlag`.
# In a normal IREE host build these modules are picked up incidentally
# by some other unrelated `include(...)` higher in the tree, but in our
# minimal ExternalProject sub-build they are not, and CMake aborts with:
#
#   CMake Error at .../ukernel/arch/riscv_64/CMakeLists.txt:153
#     (check_c_source_compiles):
#     Unknown CMake command "check_c_source_compiles".
#
# Pre-loading both modules at project() time sidesteps the issue without
# patching the iree_bar submodule.
include(CheckCSourceCompiles)
include(CheckCXXCompilerFlag)
