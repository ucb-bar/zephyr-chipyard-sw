# RISC-V RVV Vectorized LQR Riccati Implementation

This directory contains RISC-V Vector Extension (RVV) optimized implementations of the Linear Quadratic Regulator (LQR) Riccati equation solver, specifically designed for the Chipyard Saturn platform.

## Files Overview

### Main Implementations
\- **`tinyMPC/src/main.cpp`** - RVV implementation with advanced memory management
- **`lqr_riccati.cpp`** - Original Eigen-based implementation (for comparison)

### Supporting Files
- **`Makefile.rvv`** - Build system for RVV implementations

## Building and Running

### Prerequisites
```bash
# RISC-V toolchain with RVV support
export RISCV=/path/to/riscv-toolchain
export PATH=$RISCV/bin:$PATH

# Required libraries
- libgemmini (for accelerated operations)
- matlib_rvv (RVV matrix library)
- Eigen3 (for comparison with original implementation)
```

### Compilation
```bash
# Build all implementations
west build -p -b spike_riscv64 .
```

### Execution on Spike
```bash
# Run optimized RVV implementation
../../../spike --isa=rv64gcv_zicntr /scratch/kris/zephyr/samples/chipyard/tinyMPC/build/zephyr/zephyr.elf
```
