#!/bin/bash

# Script to run the Zephyr RVV LQR application on Spike simulator
# Based on Chipyard Zephyr documentation

set -e

echo "=== Running Zephyr RVV LQR on Spike Simulator ==="
echo ""

# Check if the ELF file exists
ELF_FILE="build/zephyr/zephyr.elf"

if [[ ! -f "$ELF_FILE" ]]; then
    echo "Error: $ELF_FILE not found!"
    echo "Please build the application first:"
    echo "  ./build_zephyr.sh"
    exit 1
fi

# Check if spike is available
if ! command -v spike &> /dev/null; then
    echo "Error: spike simulator not found!"
    echo "Please install Spike or add it to your PATH"
    exit 1
fi

echo "✓ Found ELF file: $ELF_FILE"
echo "✓ Found Spike simulator: $(which spike)"
echo ""

# Display file information
echo "ELF file information:"
file "$ELF_FILE"
ls -lh "$ELF_FILE"
echo ""

# Run the application on Spike
echo "Running on Spike with RVV support..."
echo "Command: spike -p4 --isa=rv64gcv_zicntr $ELF_FILE"
echo ""
echo "Output:"
echo "======="

# Run with timeout to prevent hanging
timeout 60s spike -p4 --isa=rv64gcv_zicntr "$ELF_FILE" 2>&1 | tee spike_output.log

# Check exit status
if [[ $? -eq 0 ]]; then
    echo ""
    echo "✓ Execution completed successfully!"
    echo "Output saved to: spike_output.log"
elif [[ $? -eq 124 ]]; then
    echo ""
    echo "⚠ Execution timed out after 60 seconds"
    echo "Output saved to: spike_output.log"
else
    echo ""
    echo "✗ Execution failed"
    echo "Output saved to: spike_output.log"
fi

echo ""
echo "To run manually:"
echo "  spike -p4 --isa=rv64gcv_zicntr $ELF_FILE"