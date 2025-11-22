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

