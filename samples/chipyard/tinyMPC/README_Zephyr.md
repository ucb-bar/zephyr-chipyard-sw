# Zephyr RVV LQR Riccati Implementation

This is a **Zephyr RTOS** application implementing LQR Riccati equation solver using **RISC-V Vector Extension (RVV)** optimizations, designed for **Chipyard Saturn** platform.

## Overview

This implementation integrates the LQR (Linear Quadratic Regulator) Riccati equation solver with:
- **Zephyr RTOS** kernel and services
- **RISC-V Vector Extension** for SIMD optimizations
- **Self-contained RVV matrix operations** (no external dependencies)
- **Proper Zephyr memory management** and timing
- **Chipyard Saturn** platform support

## Files Structure

```
zephyr/samples/chipyard/tinyMPC/
├── CMakeLists.txt          # Zephyr build configuration
├── prj.conf                # Project configuration
├── src/main.cpp            # Zephyr application entry point
├── build_zephyr.sh         # Build script (recommended)
├── run_spike.sh            # Spike execution script
├── README_Zephyr.md        # This file
└── build/                  # Build output directory
    └── zephyr/
        └── zephyr.elf      # Final executable
```

## Prerequisites

Based on [Chipyard Zephyr documentation](https://chipyard.readthedocs.io/en/latest/Software/Zephyr.html):

### Environment Setup

```bash
# Set environment variables (in your zephyr directory)
export ZEPHYR_BASE=$(pwd)
export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile

# Set cross-compiler (adjust path for your system)
# On gym:
export CROSS_COMPILE=/scratch/vnikiforov/riscv-bin-14/bin/riscv64-unknown-elf-
# On garden:
export CROSS_COMPILE=/scratch/jinshengxu/test/riscv/bin/riscv64-unknown-elf-
```

### Dependencies

```bash
# Install pip dependencies
python -m pip install executorch==0.5.0 west pyelftools zstd

# Install cmake (if needed)
conda install anaconda::cmake
```

## Building

### Method 1: Using Build Script (Recommended)

```bash
# Make executable and run
chmod +x build_zephyr.sh
./build_zephyr.sh
```

### Method 2: Using west directly

```bash
# Build with Zephyr
west build -p -b spike_riscv64 . \
    --build-dir build \
    -- \
    -DCONFIG_COMPILER_OPT="-march=rv64gcv -mabi=lp64d -O3" \
    -DCONFIG_RVV_ENABLED=ON
```

## Running

### Method 1: Using Run Script (Recommended)

```bash
# Make executable and run
chmod +x run_spike.sh
./run_spike.sh
```

### Method 2: Direct Spike execution

```bash
# Run on Spike simulator with RVV support
spike -p4 --isa=rv64gcv_zicntr build/zephyr/zephyr.elf
```

### Method 3: On Chipyard Saturn Hardware

```bash
# Transfer the ELF file to your Chipyard system
scp build/zephyr/zephyr.elf your-chipyard-system:/path/

# Run on actual hardware
ssh your-chipyard-system
/path/zephyr.elf
```

## Expected Output

The application will output CSV format performance data:

```
=== Zephyr RVV LQR Riccati Implementation ===
Running on Chipyard Saturn with RISC-V Vector Extension

state_space_size,action_space_size,horizon_length,time_cycles,result_norm
4,4,2,1234567,2.345678
4,8,2,2345678,3.456789
8,4,2,3456789,4.567890
8,8,2,4567890,5.678901

=== RVV LQR Riccati Test Complete ===
```

## Key Features

### RVV Optimizations
- **Vectorized Matrix Operations**: Add, subtract, multiply using RVV intrinsics
- **Tiled Matrix Multiplication**: Cache-friendly blocked algorithms  
- **Strided Memory Access**: Efficient transpose operations
- **Vector Reductions**: Optimized dot products and sums
- **Adaptive Vector Length**: Automatically uses available vector width

### Zephyr Integration
- **Kernel Memory Management**: Uses `k_aligned_alloc()` and `k_free()`
- **Timing Subsystem**: Accurate cycle counting with `timing_*()` APIs
- **Console Output**: Proper logging via `printk()`
- **Random Numbers**: Secure random generation with `sys_rand32_get()`
- **C++ Support**: Full C++17 support in Zephyr environment

### Self-Contained Implementation
- **No External Dependencies**: All RVV operations included directly
- **Portable**: Works on any RVV-capable RISC-V system
- **Optimized Memory Layout**: Row-major matrices with proper alignment

## Configuration

### Project Configuration (`prj.conf`)

Key settings for optimal performance:
```ini
# RVV and performance
CONFIG_SPEED_OPTIMIZATIONS=y
CONFIG_COMPILER_OPT="-O3"
CONFIG_FPU=y

# Memory management
CONFIG_HEAP_MEM_POOL_SIZE=262144
CONFIG_MAIN_STACK_SIZE=16384

# C++ support
CONFIG_CPP=y
CONFIG_STD_CPP17=y
CONFIG_LIB_CPLUSPLUS=y
```

### Build Configuration (`CMakeLists.txt`)

RVV-specific compiler flags:
```cmake
target_compile_options(app PRIVATE
    -march=rv64gcv    # Enable RVV
    -mabi=lp64d       # 64-bit ABI with double precision
    -O3               # Maximum optimization
)
```

## Performance Characteristics

### Expected Speedups (vs scalar)
- **128-bit vectors**: 2-4x speedup
- **256-bit vectors**: 4-8x speedup  
- **512-bit vectors**: 6-12x speedup

### Benchmarking
The application outputs cycle counts which can be used for performance analysis:
- `time_cycles`: Total cycles for LQR computation
- `result_norm`: Frobenius norm of result matrix (sanity check)

## Troubleshooting

### Build Issues

1. **Toolchain not found**:
   ```bash
   export CROSS_COMPILE=/path/to/your/riscv64-unknown-elf-
   ```

2. **west not found**:
   ```bash
   pip install west
   ```

3. **CMake too old**:
   ```bash
   conda install anaconda::cmake
   ```

### Runtime Issues

1. **Spike not found**:
   ```bash
   # Install Spike or add to PATH
   export PATH=/path/to/spike/bin:$PATH
   ```

2. **RVV instructions not working**:
   - Verify `--isa=rv64gcv_zicntr` flag
   - Check that toolchain supports RVV

3. **Memory allocation failures**:
   - Increase `CONFIG_HEAP_MEM_POOL_SIZE` in `prj.conf`

## Differences from Linux Version

| Aspect | Linux Version | Zephyr Version |
|--------|---------------|----------------|
| Build System | Makefile + gcc | west + CMake |
| Memory Management | malloc/free | k_aligned_alloc/k_free |
| Timing | rdcycle asm | Zephyr timing APIs |
| Output | cout/printf | printk() |
| Random Numbers | rand() | sys_rand32_get() |
| Entry Point | main() | Zephyr main() |

## Development Notes

### Adding New Matrix Operations
1. Follow RVV intrinsics pattern in existing functions
2. Use proper error checking and memory management
3. Test on different vector widths

### Debugging
```bash
# Build with debug symbols
west build -p -b spike_riscv64 . -- -DCONFIG_DEBUG=y

# Run with GDB
spike-dasm --isa=rv64gcv_zicntr build/zephyr/zephyr.elf
```

### Performance Tuning
- Adjust `BATCH` size for your target vector width
- Tune `CONFIG_HEAP_MEM_POOL_SIZE` for your matrix sizes
- Profile with different optimization levels

## References

- [Chipyard Zephyr Documentation](https://chipyard.readthedocs.io/en/latest/Software/Zephyr.html)
- [RISC-V Vector Extension Specification](https://github.com/riscv/riscv-v-spec)
- [Zephyr Project Documentation](https://docs.zephyrproject.org/)
- [Saturn Vector Processor](https://github.com/ucb-bar/saturn-vectors)