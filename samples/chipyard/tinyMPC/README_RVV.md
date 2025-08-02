# RISC-V RVV Vectorized LQR Riccati Implementation

This directory contains RISC-V Vector Extension (RVV) optimized implementations of the Linear Quadratic Regulator (LQR) Riccati equation solver, specifically designed for the Chipyard Saturn platform.

## Files Overview

### Main Implementations
- **`lqr_riccati_rvv.cpp`** - Basic RVV vectorized implementation
- **`lqr_riccati_rvv_optimized.cpp`** - Highly optimized RVV implementation with advanced memory management
- **`lqr_riccati.cpp`** - Original Eigen-based implementation (for comparison)

### Supporting Files
- **`rvv_utils.h`** - Utility functions for RVV matrix operations
- **`Makefile.rvv`** - Build system for RVV implementations
- **`README_RVV.md`** - This documentation file

## Key Features

### RVV Vectorization Benefits
1. **SIMD Parallelism**: Leverages RISC-V vector instructions for parallel floating-point operations
2. **Adaptive Vector Length**: Automatically adapts to the hardware's vector register width
3. **Memory Efficiency**: Optimized memory access patterns with vectorized loads/stores
4. **Cache-Friendly**: Tiled matrix operations reduce cache misses

### Optimizations Implemented
1. **Memory Pool Management**: Pre-allocated working matrices to minimize allocation overhead
2. **Transpose Optimization**: Efficient handling of matrix transpose operations using RVV
3. **Tiled Matrix Multiplication**: Cache-friendly blocked algorithms
4. **Branch Reduction**: Vectorized conditional operations where possible

## Technical Details

### Matrix Operations Using RVV
The implementation replaces Eigen matrix operations with vectorized equivalents:

```cpp
// Original Eigen operation
Matrix result = A.transpose() * B;

// RVV vectorized equivalent
float* AT = alloc_matrix_rvv(cols, rows);
transpose_rvv(A, AT, rows, cols);
enhanced_matmul_rvv(AT, B, result, cols, result_cols, rows);
```

### Memory Layout
- All matrices use **row-major** layout for optimal cache performance
- Aligned memory allocation (32-byte alignment) for vectorized operations
- Contiguous memory blocks to maximize vector load/store efficiency

### Numerical Methods
Two solving approaches are provided:
1. **Matrix Inversion**: Direct computation using Gauss-Jordan elimination
2. **LU Decomposition**: More numerically stable for ill-conditioned systems

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
make -f Makefile.rvv all

# Build only optimized RVV version
make -f Makefile.rvv lqr_riccati_rvv_optimized

# Build with debug symbols
make -f Makefile.rvv debug
```

### Execution
```bash
# Run optimized RVV implementation
./lqr_riccati_rvv_optimized

# Run performance comparison
make -f Makefile.rvv perf_compare

# Generate CSV results for all implementations
make -f Makefile.rvv run_all
```

## Performance Characteristics

### Expected Performance Gains
Based on RVV vector width and optimization level:

| Vector Width | Expected Speedup | Notes |
|--------------|------------------|-------|
| 128-bit      | 2-4x            | Basic vectorization |
| 256-bit      | 4-8x            | Good vector utilization |
| 512-bit      | 6-12x           | Optimal for larger matrices |

### Benchmarking
The implementations output performance data in CSV format:
```
state_space_size,action_space_size,horizon_length,time_rvv_opt,time_rvv_lu,error_norm
4,4,2,1234,1456,1.23e-12
8,8,2,4567,5123,2.45e-11
```

## Algorithm Details

### LQR Riccati Equation
The discrete-time finite-horizon LQR problem solves:
```
P(k) = Q + A^T * P(k+1) * A - A^T * P(k+1) * B * (R + B^T * P(k+1) * B)^(-1) * B^T * P(k+1) * A
K(k) = (R + B^T * P(k+1) * B)^(-1) * B^T * P(k+1) * A
```

### RVV Optimization Strategy
1. **Vectorized Matrix Multiplication**: Core operation using `matmul_rvv`
2. **Transpose Operations**: Efficient in-place/out-of-place transposes
3. **Element-wise Operations**: Add/subtract using `matadd_rvv`/`matsub_rvv`
4. **Reduction Operations**: Norms and traces using vector reductions

## Hardware Compatibility

### Chipyard Saturn Platform
- **CPU**: RISC-V with RVV 1.0 support
- **Vector Width**: Configurable (128-bit to 1024-bit)
- **Memory**: Hierarchical cache system optimized for vector operations
- **Accelerators**: Integration with Gemmini systolic array

### RVV Instruction Usage
Key RVV instructions utilized:
- `vle32.v` / `vse32.v` - Vector load/store
- `vfmul.vv` / `vfadd.vv` - Vector arithmetic
- `vfmacc.vv` - Vector multiply-accumulate
- `vfredusum.vs` - Vector reduction operations

## Error Handling and Numerical Stability

### Numerical Considerations
1. **Condition Number Monitoring**: Check matrix conditioning before inversion
2. **Regularization**: Add small diagonal terms for stability
3. **Pivoting**: Full pivoting in LU decomposition for robustness
4. **Error Propagation**: Track and report numerical errors

### Debugging Features
```cpp
#define DEBUG_RVV  // Enable detailed logging
#define VERIFY_RESULTS  // Cross-check with reference implementation
```

## Future Optimizations

### Potential Improvements
1. **Mixed Precision**: Use FP16 for intermediate computations
2. **Sparse Matrix Support**: Optimized sparse matrix operations
3. **Multi-threading**: Parallel processing of independent matrix blocks
4. **Custom Instructions**: Hardware-specific optimizations for Saturn

### Integration Opportunities
1. **Gemmini Acceleration**: Hybrid RVV+systolic array computation
2. **DMA Transfers**: Asynchronous memory operations
3. **Prefetching**: Software-managed cache optimization

## References

1. RISC-V Vector Extension Specification v1.0
2. "Optimal Control Theory" - Anderson & Moore
3. Chipyard Documentation: https://chipyard.readthedocs.io/
4. Saturn Vector Processor: https://github.com/ucb-bar/saturn-vectors

## Contributing

When modifying the RVV implementations:
1. Maintain memory alignment requirements
2. Test with various matrix sizes and horizon lengths
3. Verify numerical accuracy against reference implementations
4. Profile performance on actual hardware when available

## License

This implementation follows the same license as the parent TinyMPC project.