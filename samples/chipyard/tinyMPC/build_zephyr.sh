#!/bin/bash

# Zephyr build script for RVV LQR Riccati implementation
# Based on Chipyard Zephyr documentation

set -e

echo "=== Zephyr RVV LQR Riccati Build Script ==="
echo ""

# Check if we're in the right directory
if [[ ! -f "CMakeLists.txt" ]]; then
    echo "Error: CMakeLists.txt not found. Run this script from the tinyMPC directory."
    exit 1
fi

# Set required environment variables
echo "Setting up Zephyr environment..."

# Get the absolute path to the zephyr base directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ZEPHYR_BASE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

export ZEPHYR_BASE="$ZEPHYR_BASE_DIR"
export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile

# Set the cross-compiler based on system (adjust paths as needed)
if [[ -d "/scratch/vnikiforov/riscv-bin-14/bin" ]]; then
    # On gym
    export CROSS_COMPILE="/scratch/vnikiforov/riscv-bin-14/bin/riscv64-unknown-elf-"
    echo "Using gym toolchain: $CROSS_COMPILE"
elif [[ -d "/scratch/jinshengxu/test/riscv/bin" ]]; then
    # On garden  
    export CROSS_COMPILE="/scratch/jinshengxu/test/riscv/bin/riscv64-unknown-elf-"
    echo "Using garden toolchain: $CROSS_COMPILE"
else
    # Try to find riscv64-unknown-elf-gcc in PATH
    if command -v riscv64-unknown-elf-gcc &> /dev/null; then
        FOUND_GCC=$(which riscv64-unknown-elf-gcc)
        export CROSS_COMPILE="${FOUND_GCC%-gcc}-"
        echo "Using toolchain from PATH: $CROSS_COMPILE"
    else
        echo "Error: RISC-V toolchain not found!"
        echo "Please install or set CROSS_COMPILE manually:"
        echo "  export CROSS_COMPILE=/path/to/riscv64-unknown-elf-"
        exit 1
    fi
fi

echo "Environment variables:"
echo "  ZEPHYR_BASE=$ZEPHYR_BASE"
echo "  ZEPHYR_TOOLCHAIN_VARIANT=$ZEPHYR_TOOLCHAIN_VARIANT"
echo "  CROSS_COMPILE=$CROSS_COMPILE"
echo ""

# Check if west is available
if ! command -v west &> /dev/null; then
    echo "Error: west build tool not found!"
    echo "Please install with: pip install west"
    exit 1
fi

# Check if the toolchain is available
if ! command -v "${CROSS_COMPILE}gcc" &> /dev/null; then
    echo "Error: Cross-compiler not found: ${CROSS_COMPILE}gcc"
    echo "Please check your CROSS_COMPILE path"
    exit 1
fi

echo "✓ Toolchain found: $(${CROSS_COMPILE}gcc --version | head -n1)"
echo ""

# Build the Zephyr application
echo "Building Zephyr RVV LQR application..."
echo "Command: west build -p -b spike_riscv64 . --build-dir build"
echo ""

# Build with RVV optimizations
west build -p -b spike_riscv64 . \
    --build-dir build

# Check if build was successful
if [[ $? -eq 0 ]]; then
    echo ""
    echo "✓ Build successful!"
    echo "ELF file: build/zephyr/zephyr.elf"
    echo "Size information:"
    ls -lh build/zephyr/zephyr.elf
    echo ""
    echo "To run on Spike simulator:"
    echo "  spike -p4 --isa=rv64gcv_zicntr build/zephyr/zephyr.elf"
    echo ""
    echo "To run on Chipyard Saturn:"
    echo "  # Transfer zephyr.elf to your Chipyard system and run"
    echo ""
else
    echo "✗ Build failed!"
    exit 1
fi