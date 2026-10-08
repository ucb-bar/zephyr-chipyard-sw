#!/bin/bash
# Set environment variables for Zephyr SDK installation

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Set Zephyr base directory
export ZEPHYR_BASE="${REPO_ROOT}/zephyr_ws/zephyr"

# Set Zephyr SDK installation directory
export ZEPHYR_SDK_INSTALL_DIR="${REPO_ROOT}/tools-manual/zephyr-sdk-1.0.0-beta1"

# Set toolchain variant to zephyr (uses SDK toolchain)
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr

# Optional: Add SDK tools to PATH if they exist
if [ -d "${ZEPHYR_SDK_INSTALL_DIR}/hosttools/sysroots" ]; then
    # Find the hosttools sysroot directory
    HOSTTOOLS_SYSROOT=$(find "${ZEPHYR_SDK_INSTALL_DIR}/hosttools/sysroots" -mindepth 1 -maxdepth 1 -type d | head -n 1)
    if [ -n "${HOSTTOOLS_SYSROOT}" ] && [ -d "${HOSTTOOLS_SYSROOT}/usr/bin" ]; then
        export PATH="${HOSTTOOLS_SYSROOT}/usr/bin:${PATH}"
    fi
fi

# Optional: Add GNU toolchain to PATH if it exists
if [ -d "${ZEPHYR_SDK_INSTALL_DIR}/gnu/riscv64-zephyr-elf/bin" ]; then
    export PATH="${ZEPHYR_SDK_INSTALL_DIR}/gnu/riscv64-zephyr-elf/bin:${PATH}"
fi

# Optional: Add LLVM toolchain to PATH if it exists
if [ -d "${ZEPHYR_SDK_INSTALL_DIR}/llvm/bin" ]; then
    export PATH="${ZEPHYR_SDK_INSTALL_DIR}/llvm/bin:${PATH}"
fi

echo "Zephyr SDK environment variables set:"
echo "  ZEPHYR_BASE=${ZEPHYR_BASE}"
echo "  ZEPHYR_SDK_INSTALL_DIR=${ZEPHYR_SDK_INSTALL_DIR}"
echo "  ZEPHYR_TOOLCHAIN_VARIANT=${ZEPHYR_TOOLCHAIN_VARIANT}"

# Make the modelblaster package importable without a pip editable install
# (the conda `zephyr` env is Dima's, read-only, so we can't `pip install -e`
# into it). modelblaster uses a src/ shim whose __path__ points back at the
# repo root; putting src/ on PYTHONPATH mirrors the editable install so
# `python -m modelblaster.pipeline...` works from any cwd (the per-model
# run.sh scripts cd into the modelblaster repo root before invoking it).
MODELBLASTER_SRC="${REPO_ROOT}/modelblaster/src"
if [ -d "${MODELBLASTER_SRC}" ]; then
    export PYTHONPATH="${MODELBLASTER_SRC}${PYTHONPATH:+:${PYTHONPATH}}"
    echo "  PYTHONPATH+=${MODELBLASTER_SRC}"
fi

# Put a gemmini+vector-capable spike first on PATH. The conda `zephyr`
# env ships a plain `spike` that lacks the gemmini RoCC extension lib
# (libcustomext.so), so `shutil.which("spike")` in spike_runner would
# pick it and gemmini_q31 runs fail with "couldn't find shared library
# libcustomext.so". tools-manual/bin/spike -> Dima's misc_sw/spike
# (1.1.1-dev) supports both --extension=gemmini and --isa=rv64gcv.
if [ -x "${REPO_ROOT}/tools-manual/bin/spike" ]; then
    export PATH="${REPO_ROOT}/tools-manual/bin:${PATH}"
    echo "  PATH+=${REPO_ROOT}/tools-manual/bin (spike)"
fi

