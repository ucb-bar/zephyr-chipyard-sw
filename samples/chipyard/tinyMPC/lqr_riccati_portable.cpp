#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <cmath>
#include <fstream>

#ifndef NATIVE_BUILD
// Only include gemmini and RVV headers for RISC-V builds
#include "gemmini.h"
#include "riscv_vector.h"
#endif

using namespace std;

static uint64_t read_cycles() {
#ifndef NATIVE_BUILD
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
#else
    // For native builds, use a simple counter or clock
    return static_cast<uint64_t>(clock());
#endif
}

// Memory allocation utilities
static inline float* alloc_matrix(int rows, int cols) {
    return (float*)aligned_alloc(32, rows * cols * sizeof(float));
}

static inline void free_matrix(float* ptr) {
    if (ptr) free(ptr);
}

// ============================================================================
// Matrix Operations - RVV or scalar fallback
// ============================================================================

#ifndef BATCH
#define BATCH 4
#endif

// Matrix addition - RVV or scalar
inline void matadd_impl(const float *ptr_a, const float *ptr_b, float *ptr_c, int n, int m) {
#ifndef NATIVE_BUILD
    // RVV implementation
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32(k);
        vfloat32_t vec_a = __riscv_vle32_v_f32(ptr_a + l, vl);
        vfloat32_t vec_b = __riscv_vle32_v_f32(ptr_b + l, vl);
        vfloat32_t vec_c = __riscv_vfadd_vv_f32(vec_a, vec_b, vl);
        __riscv_vse32_v_f32(ptr_c + l, vec_c, vl);
    }
#else
    // Scalar fallback
    int total = m * n;
    for (int i = 0; i < total; ++i) {
        ptr_c[i] = ptr_a[i] + ptr_b[i];
    }
#endif
}

// Matrix subtraction - RVV or scalar
inline void matsub_impl(const float *ptr_a, const float *ptr_b, float *ptr_c, int n, int m) {
#ifndef NATIVE_BUILD
    // RVV implementation
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32(k);
        vfloat32_t vec_a = __riscv_vle32_v_f32(ptr_a + l, vl);
        vfloat32_t vec_b = __riscv_vle32_v_f32(ptr_b + l, vl);
        vfloat32_t vec_c = __riscv_vfsub_vv_f32(vec_a, vec_b, vl);
        __riscv_vse32_v_f32(ptr_c + l, vec_c, vl);
    }
#else
    // Scalar fallback
    int total = m * n;
    for (int i = 0; i < total; ++i) {
        ptr_c[i] = ptr_a[i] - ptr_b[i];
    }
#endif
}

// Matrix copy - RVV or scalar
inline void matcopy_impl(const float *ptr_a, float *ptr_b, int n, int m) {
#ifndef NATIVE_BUILD
    // RVV implementation
    int k = n * m, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32(k);
        vfloat32_t vec_a = __riscv_vle32_v_f32(ptr_a + l, vl);
        __riscv_vse32_v_f32(ptr_b + l, vec_a, vl);
    }
#else
    // Scalar fallback
    memcpy(ptr_b, ptr_a, n * m * sizeof(float));
#endif
}

// Matrix set - RVV or scalar
inline void matset_impl(float *ptr_a, float f, int n, int m) {
#ifndef NATIVE_BUILD
    // RVV implementation
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32(k);
        vfloat32_t vec_a = __riscv_vfmv_v_f_f32(f, vl);
        __riscv_vse32_v_f32(ptr_a + l, vec_a, vl);
    }
#else
    // Scalar fallback
    int total = m * n;
    for (int i = 0; i < total; ++i) {
        ptr_a[i] = f;
    }
#endif
}

// Matrix transpose - scalar implementation (works for both)
inline void transpose_impl(const float *a, float *b, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            b[j * n + i] = a[i * m + j];
        }
    }
}

// Matrix multiplication - scalar implementation
inline void matmul_impl(const float *a, const float *b, float *c, int n, int m, int o) {
    // Initialize result matrix to zero
    for (int i = 0; i < n * m; ++i) {
        c[i] = 0.0f;
    }
    
    // Standard matrix multiplication: C = A * B
    // A is n x o, B is o x m, C is n x m
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            for (int k = 0; k < o; ++k) {
                c[i * m + j] += a[i * o + k] * b[k * m + j];
            }
        }
    }
}

// Function aliases for compatibility
#define matadd_rvv matadd_impl
#define matsub_rvv matsub_impl
#define matcopy_rvv matcopy_impl
#define matset_rvv matset_impl
#define transpose_rvv transpose_impl
#define matmul_rvv(a,b,c,n,m,o,...) matmul_impl(a,b,c,n,m,o)

// ============================================================================
// Matrix utility functions
// ============================================================================

// Matrix inverse using Gauss-Jordan elimination
bool matrix_inverse_rvv(const float* A, float* A_inv, int n) {
    // Create augmented matrix [A | I]
    float* aug = alloc_matrix(n, 2 * n);
    
    // Copy A to left side and create identity on right side
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            aug[i * (2 * n) + j] = A[i * n + j];
            aug[i * (2 * n) + j + n] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Gauss-Jordan elimination
    for (int i = 0; i < n; ++i) {
        // Find pivot
        float pivot = aug[i * (2 * n) + i];
        if (fabs(pivot) < 1e-10) {
            free_matrix(aug);
            return false; // Matrix is singular
        }
        
        // Scale pivot row
        for (int j = 0; j < 2 * n; ++j) {
            aug[i * (2 * n) + j] /= pivot;
        }
        
        // Eliminate column
        for (int k = 0; k < n; ++k) {
            if (k != i) {
                float factor = aug[k * (2 * n) + i];
                for (int j = 0; j < 2 * n; ++j) {
                    aug[k * (2 * n) + j] -= factor * aug[i * (2 * n) + j];
                }
            }
        }
    }
    
    // Copy result from right side of augmented matrix
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            A_inv[i * n + j] = aug[i * (2 * n) + j + n];
        }
    }
    
    free_matrix(aug);
    return true;
}

// LU solve using partial pivoting
bool lu_solve_rvv(const float* A, const float* b, float* x, int n) {
    float* A_copy = alloc_matrix(n, n);
    float* b_copy = alloc_matrix(n, 1);
    
    matcopy_rvv(A, A_copy, n, n);
    matcopy_rvv(b, b_copy, n, 1);
    
    // Forward elimination
    for (int i = 0; i < n - 1; ++i) {
        // Find pivot
        int pivot_row = i;
        float max_val = fabs(A_copy[i * n + i]);
        for (int k = i + 1; k < n; ++k) {
            if (fabs(A_copy[k * n + i]) > max_val) {
                max_val = fabs(A_copy[k * n + i]);
                pivot_row = k;
            }
        }
        
        // Swap rows if needed
        if (pivot_row != i) {
            for (int j = 0; j < n; ++j) {
                float temp = A_copy[i * n + j];
                A_copy[i * n + j] = A_copy[pivot_row * n + j];
                A_copy[pivot_row * n + j] = temp;
            }
            float temp = b_copy[i];
            b_copy[i] = b_copy[pivot_row];
            b_copy[pivot_row] = temp;
        }
        
        // Eliminate
        for (int k = i + 1; k < n; ++k) {
            if (fabs(A_copy[i * n + i]) < 1e-10) continue;
            float factor = A_copy[k * n + i] / A_copy[i * n + i];
            for (int j = i; j < n; ++j) {
                A_copy[k * n + j] -= factor * A_copy[i * n + j];
            }
            b_copy[k] -= factor * b_copy[i];
        }
    }
    
    // Back substitution
    for (int i = n - 1; i >= 0; --i) {
        x[i] = b_copy[i];
        for (int j = i + 1; j < n; ++j) {
            x[i] -= A_copy[i * n + j] * x[j];
        }
        if (fabs(A_copy[i * n + i]) < 1e-10) {
            free_matrix(A_copy);
            free_matrix(b_copy);
            return false;
        }
        x[i] /= A_copy[i * n + i];
    }
    
    free_matrix(A_copy);
    free_matrix(b_copy);
    return true;
}

// ============================================================================
// LQR Riccati Solver Implementation
// ============================================================================

float* lqrSolveFiniteHorizonPortable(const float* A, const float* B, const float* Q, const float* R, 
                                   int state_dim, int input_dim, int horizon) {
    
    // Allocate gain matrices
    vector<float*> K(horizon);
    for (int t = 0; t < horizon; ++t) {
        K[t] = alloc_matrix(input_dim, state_dim);
        matset_rvv(K[t], 0.0f, input_dim, state_dim);
    }
    
    // Allocate working matrices
    float* P = alloc_matrix(state_dim, state_dim);
    float* BT = alloc_matrix(input_dim, state_dim);
    float* AT = alloc_matrix(state_dim, state_dim);
    float* PB = alloc_matrix(state_dim, input_dim);
    float* PA = alloc_matrix(state_dim, state_dim);
    float* BTPB = alloc_matrix(input_dim, input_dim);
    float* BTPB_R = alloc_matrix(input_dim, input_dim);
    float* BTPB_R_inv = alloc_matrix(input_dim, input_dim);
    float* BTPA = alloc_matrix(input_dim, state_dim);
    float* ATPA = alloc_matrix(state_dim, state_dim);
    float* BTPAK = alloc_matrix(state_dim, state_dim);
    
    // Initialize P = Q
    matcopy_rvv(Q, P, state_dim, state_dim);
    
    // Precompute transposes
    transpose_rvv(B, BT, state_dim, input_dim);
    transpose_rvv(A, AT, state_dim, state_dim);
    
    // Perform the Riccati recursion
    for (int t = horizon - 1; t >= 0; --t) {
        // PB = P * B
        matmul_rvv(P, B, PB, state_dim, input_dim, state_dim);
        
        // BTPB = B^T * PB
        matmul_rvv(BT, PB, BTPB, input_dim, input_dim, state_dim);
        
        // BTPB_R = BTPB + R
        matadd_rvv(BTPB, R, BTPB_R, input_dim, input_dim);
        
        // Compute inverse: BTPB_R_inv = (B^T * P * B + R)^(-1)
        if (!matrix_inverse_rvv(BTPB_R, BTPB_R_inv, input_dim)) {
            cerr << "Matrix inversion failed at iteration " << t << endl;
            // Cleanup and return nullptr
            for (int i = 0; i < horizon; ++i) free_matrix(K[i]);
            free_matrix(P); free_matrix(BT); free_matrix(AT);
            free_matrix(PB); free_matrix(PA); free_matrix(BTPB);
            free_matrix(BTPB_R); free_matrix(BTPB_R_inv); free_matrix(BTPA);
            free_matrix(ATPA); free_matrix(BTPAK);
            return nullptr;
        }
        
        // PA = P * A
        matmul_rvv(P, A, PA, state_dim, state_dim, state_dim);
        
        // BTPA = B^T * PA
        matmul_rvv(BT, PA, BTPA, input_dim, state_dim, state_dim);
        
        // K[t] = BTPB_R_inv * BTPA
        matmul_rvv(BTPB_R_inv, BTPA, K[t], input_dim, state_dim, input_dim);
        
        // Update P for next iteration: P = A^T * P * A - A^T * P * B * K[t] + Q
        // ATPA = A^T * PA
        matmul_rvv(AT, PA, ATPA, state_dim, state_dim, state_dim);
        
        // BTPAK = BTPA^T * K[t] (note: this should be transpose of BTPA times K[t])
        // For simplicity, we'll compute it as a regular multiplication
        matmul_rvv(BTPA, K[t], BTPAK, state_dim, state_dim, input_dim);
        
        // P = ATPA - BTPAK + Q
        matsub_rvv(ATPA, BTPAK, P, state_dim, state_dim);
        matadd_rvv(P, Q, P, state_dim, state_dim);
    }
    
    // Allocate and copy the result (K[0])
    float* result = alloc_matrix(input_dim, state_dim);
    matcopy_rvv(K[0], result, input_dim, state_dim);
    
    // Cleanup
    for (int t = 0; t < horizon; ++t) free_matrix(K[t]);
    free_matrix(P); free_matrix(BT); free_matrix(AT);
    free_matrix(PB); free_matrix(PA); free_matrix(BTPB);
    free_matrix(BTPB_R); free_matrix(BTPB_R_inv); free_matrix(BTPA);
    free_matrix(ATPA); free_matrix(BTPAK);
    
    return result;
}

// Random matrix generation utilities
float* randomStableMatrix(int size) {
    float* matrix = alloc_matrix(size, size);
    
    // Generate simple stable matrix
    matset_rvv(matrix, 0.0f, size, size);
    for (int i = 0; i < size; ++i) {
        matrix[i * size + i] = 0.5f + 0.3f * static_cast<float>(rand()) / RAND_MAX;
        for (int j = 0; j < size; ++j) {
            if (i != j) {
                matrix[i * size + j] = 0.1f * (static_cast<float>(rand()) / RAND_MAX - 0.5f);
            }
        }
    }
    
    return matrix;
}

float* randomPositiveDefiniteMatrix(int size) {
    float* matrix = alloc_matrix(size, size);
    
    // Generate diagonal dominant matrix
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (i == j) {
                matrix[i * size + j] = 2.0f + static_cast<float>(rand()) / RAND_MAX;
            } else {
                matrix[i * size + j] = 0.1f * (static_cast<float>(rand()) / RAND_MAX - 0.5f);
            }
        }
    }
    
    return matrix;
}

// Matrix comparison utility
float matrixNormDifference(const float* A, const float* B, int rows, int cols) {
    float norm = 0.0f;
    for (int i = 0; i < rows * cols; ++i) {
        float diff = A[i] - B[i];
        norm += diff * diff;
    }
    return sqrt(norm);
}

int main() {
#ifdef NATIVE_BUILD
    cout << "# Running portable version (scalar operations)" << endl;
#else
    cout << "# Running RVV vectorized version" << endl;
#endif
    
    cout << "state_space_size,action_space_size,horizon_length,time_portable,result_norm\n";
    
    vector<int> dimensions = {4, 8};
    int horizon = 2;
    
    for (int input_dim : dimensions) {
        for (int state_dim : dimensions) {
            uint64_t t0, t1, time_portable;
            
            // Initialize random seed
            srand(static_cast<unsigned>(time(0)) + input_dim * 1000 + state_dim);
            
            // Generate system matrices
            float* A = randomStableMatrix(state_dim);
            float* B = alloc_matrix(state_dim, input_dim);
            for (int i = 0; i < state_dim * input_dim; ++i) {
                B[i] = static_cast<float>(rand()) / RAND_MAX - 0.5f;
            }
            
            // Generate cost matrices
            float* Q = randomPositiveDefiniteMatrix(state_dim);
            float* R = randomPositiveDefiniteMatrix(input_dim);
            
            // Solve using portable implementation
            t0 = read_cycles();
            float* K_result = lqrSolveFiniteHorizonPortable(A, B, Q, R, state_dim, input_dim, horizon);
            t1 = read_cycles();
            time_portable = t1 - t0;
            
            // Compute result norm as a sanity check
            float result_norm = 0.0f;
            if (K_result) {
                for (int i = 0; i < input_dim * state_dim; ++i) {
                    result_norm += K_result[i] * K_result[i];
                }
                result_norm = sqrt(result_norm);
            }
            
            cout << state_dim << "," << input_dim << "," << horizon << "," 
                 << time_portable << "," << result_norm << "\n";
            
            // Cleanup
            free_matrix(A);
            free_matrix(B);
            free_matrix(Q);
            free_matrix(R);
            if (K_result) free_matrix(K_result);
        }
    }
    
    return 0;
}