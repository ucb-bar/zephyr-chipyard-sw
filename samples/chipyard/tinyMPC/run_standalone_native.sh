#!/bin/bash

# Script to compile and run the standalone RVV LQR implementation
# This version tries multiple compiler options and can fall back to native compilation

echo "=== Detecting Available Compilers ==="

# Try to find RISC-V compilers
COMPILERS=(
    "riscv64-unknown-linux-gnu-g++"
    "riscv64-linux-gnu-g++"
    "riscv64-unknown-elf-g++"
    "riscv64-elf-g++"
    "g++"  # Fallback to native compiler
)

FOUND_COMPILER=""
for compiler in "${COMPILERS[@]}"; do
    if command -v "$compiler" &> /dev/null; then
        echo "✓ Found: $compiler"
        FOUND_COMPILER="$compiler"
        break
    else
        echo "✗ Not found: $compiler"
    fi
done

if [ -z "$FOUND_COMPILER" ]; then
    echo "✗ No suitable compiler found!"
    exit 1
fi

echo "Using compiler: $FOUND_COMPILER"
echo

# Set flags based on compiler type
if [[ "$FOUND_COMPILER" == *"riscv"* ]]; then
    # RISC-V cross-compiler
    CXXFLAGS="-march=rv64gcv -mabi=lp64d -O3 -std=c++17"
    LIBS="-lm"
    echo "Mode: RISC-V Cross-compilation"
else
    # Native compiler (for testing/debugging)
    CXXFLAGS="-O3 -std=c++17 -DNATIVE_BUILD"
    LIBS="-lm"
    echo "Mode: Native compilation (for testing)"
    echo "Warning: This will NOT use RVV instructions!"
fi

# Source file and output
SOURCE="lqr_riccati_rvv_basic_standalone.cpp"
OUTPUT="lqr_riccati_rvv_basic_standalone"

echo "Flags: $CXXFLAGS"
echo "Source: $SOURCE"
echo "Output: $OUTPUT"
echo

# Compile the program
echo "Compiling..."
$FOUND_COMPILER $CXXFLAGS $SOURCE -o $OUTPUT $LIBS

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "✓ Compilation successful!"
    echo
    
    if [[ "$FOUND_COMPILER" == *"riscv"* ]]; then
        echo "=== Cross-compiled for RISC-V ==="
        echo "To run on RISC-V target:"
        echo "  scp $OUTPUT your-riscv-target:/path/"
        echo "  ssh your-riscv-target"
        echo "  /path/$OUTPUT"
    else
        echo "=== Running Native Implementation ==="
        echo "Note: This uses scalar operations, not RVV"
        echo "Executing: ./$OUTPUT"
        echo
        
        # Run the program and save output
        ./$OUTPUT | tee results_native_test.csv
        
        echo
        echo "✓ Execution complete!"
        echo "Results saved to: results_native_test.csv"
    fi
    
else
    echo "✗ Compilation failed!"
    exit 1
fi