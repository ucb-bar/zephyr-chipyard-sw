#!/bin/bash

# ============================================================================
# OBSOLETE: This script is for Linux builds and is NO LONGER USED
# ============================================================================

echo "============================================================================"
echo "OBSOLETE SCRIPT - USE ZEPHYR BUILD SYSTEM"
echo "============================================================================"
echo ""
echo "This script was designed for Linux compilation and is no longer used."
echo "This project has been converted to use Zephyr RTOS build system."
echo ""
echo "NEW BUILD PROCESS:"
echo "=================="
echo ""
echo "1. Build the Zephyr application:"
echo "   ./build_zephyr.sh"
echo ""
echo "2. Run on Spike simulator:"
echo "   ./run_spike.sh"
echo ""
echo "3. Or use west directly:"
echo "   west build -p -b spike_riscv64 ."
echo "   spike -p4 --isa=rv64gcv_zicntr build/zephyr/zephyr.elf"
echo ""
echo "See README_Zephyr.md for complete instructions."
echo ""
echo "============================================================================"