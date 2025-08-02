#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <cmath>
#include <fstream>
#include "gemmini.h"

// RISC-V Vector Extension intrinsics
#include "riscv_vector.h"

using namespace std;

static uint64_t read_cycles() {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

// Memory allocation utilities
static inline float* alloc_matrix_rvv(int rows, int cols) {
    return (float*)aligned_alloc(32, rows * cols * sizeof(float));
}

static inline void free_matrix_rvv(float* ptr) {
    if (ptr) free(ptr);
}

// ============================================================================
// RVV Matrix Operations (copied from matlib_rvv.h)
// ============================================================================

#ifndef BATCH
#define BATCH 4
#endif

// Matrix addition using RVV
inline void matadd_rvv(const float *ptr_a, const float *ptr_b, float *ptr_c, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32(k);
        vfloat32_t vec_a = __riscv_vle32_v_f32(ptr_a + l, vl);
        vfloat32_t vec_b = __riscv_vle32_v_f32(ptr_b + l, vl);
        vfloat32_t vec_c = __riscv_vfadd_vv_f32(vec_a, vec_b, vl);
        __riscv_vse32_v_f32(ptr_c + l, vec_c, vl);
    }
}

// Matrix subtraction using RVV
inline void matsub_rvv(const float *ptr_a, const float *ptr_b, float *ptr_c, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32(k);
        vfloat32_t vec_a = __riscv_vle32_v_f32(ptr_a + l, vl);
        vfloat32_t vec_b = __riscv_vle32_v_f32(ptr_b + l, vl);
        vfloat32_t vec_c = __riscv_vfsub_vv_f32(vec_a, vec_b, vl);
        __riscv_vse32_v_f32(ptr_c + l, vec_c, vl);
    }
}

// Matrix copy using RVV
inline void matcopy_rvv(const float *ptr_a, float *ptr_b, int n, int m) {
    int k = n * m, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32(k);
        vfloat32_t vec_a = __riscv_vle32_v_f32(ptr_a + l, vl);
        __riscv_vse32_v_f32(ptr_b + l, vec_a, vl);
    }
}

// Matrix set to scalar value using RVV
inline void matset_rvv(float *ptr_a, float f, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32(k);
        vfloat32_t vec_a = __riscv_vfmv_v_f_f32(f, vl);
        __riscv_vse32_v_f32(ptr_a + l, vec_a, vl);
    }
}

// Matrix transpose using RVV
inline void transpose_rvv(const float *a, float *b, int n, int m) {
    for (int j = 0; j < m; ++j) {
        const float *ptr_a = a + j;
        float *ptr_b = b + j * n;
        int k = n;
        int l = 0;
        for (size_t vl; k > 0; k -= vl, l += vl, ptr_a = a + l * m + j, ptr_b += vl) {
            vl = __riscv_vsetvl_e32(k);
            vfloat32_t vec_a = __riscv_vlse32_v_f32(ptr_a, sizeof(float) * m, vl);
            __riscv_vse32_v_f32(ptr_b, vec_a, vl);
        }
    }
}

// Tiled matrix multiplication helper
inline void matmul_rvvt(const float *a, const float *b, float *c, int i, int j, int k, 
                       int n, int m, int o, int tile_size, int *ind_a) {
    vfloat32m1_t v1, v2, v3, v4, v5, v6, v7, v8;
    vfloat32m1_t *vec_r[8] = { &v1, &v2, &v3, &v4, &v5, &v6, &v7, &v8 };
    const float *A = a + (ind_a ? 0 : i * o) + k;
    const float *B = b + j * o + k;
    float *C = c + i * m + j;
    int N = i + tile_size <= n ? tile_size : n % tile_size;
    int M = j + tile_size <= m ? tile_size : m % tile_size;
    int O = k + tile_size <= o ? tile_size : o % tile_size;
    
    size_t vlmax = __riscv_vsetvlmax_e32();
    vfloat32m1_t vec_zero = __riscv_vfmv_v_f_f32m1(0, vlmax);
    
    for (int I = 0; I < N; I++) {
        const float *ptr_a_0 = A + (ind_a ? ind_a[I + i] : I * o);
        
        for (int J = 0; J < M; J += BATCH) {
            int P = J + BATCH <= M ? BATCH : M - J;
            const float *ptr_a = ptr_a_0;
            const float *ptr_b = B + J * o;
            int K = O;
            
            for (int L = 0; L < P; L++) {
                *(vec_r[L]) = __riscv_vfmv_v_f_f32(0, vlmax);
            }
            
            for (size_t vl; K > 0; K -= vl, ptr_a += vl, ptr_b += vl) {
                vl = __riscv_vsetvl_e32(K);
                vfloat32_t vec_a = __riscv_vle32_v_f32(ptr_a, vl);
                for (int L = 0; L < P; L++) {
                    vfloat32_t vec_b = __riscv_vle32_v_f32(ptr_b + L * o, vl);
                    *(vec_r[L]) = __riscv_vfmacc_vv_f32(*(vec_r[L]), vec_a, vec_b, vl);
                }
            }
            
            for (int L = 0; L < P; L++) {
                vfloat32m1_t vec_sum = __riscv_vfredusum_vs_f32_f32(*(vec_r[L]), vec_zero, vlmax);
                float sum = __riscv_vfmv_f_s_f32m1_f32(vec_sum);
                C[I * m + J + L] = k == 0 ? sum : C[I * m + J + L] + sum;
            }
        }
    }
}

// Matrix multiplication using RVV
inline void matmul_rvv(const float *a, const float *b, float *c, int n, int m, int o, int tile_size = -1, int *ind_a = nullptr) {
    if (tile_size == -1) {
        size_t vlmax = __riscv_vsetvlmax_e32();
        vfloat32m1_t vec_zero = __riscv_vfmv_v_f_f32m1(0, vlmax);
        
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                const float *ptr_a = a + i * o; // row major
                const float *ptr_b = b + j * o; // column major
                int k = o;
                vfloat32_t vec_s = __riscv_vfmv_v_f_f32(0, vlmax);
                
                for (size_t vl; k > 0; k -= vl, ptr_a += vl, ptr_b += vl) {
                    vl = __riscv_vsetvl_e32(k);
                    vfloat32_t vec_a = __riscv_vle32_v_f32(ptr_a, vl);
                    vfloat32_t vec_b = __riscv_vle32_v_f32(ptr_b, vl);
                    vec_s = __riscv_vfmacc_vv_f32(vec_s, vec_a, vec_b, vl);
                }
                
                vfloat32m1_t vec_sum = __riscv_vfredusum_vs_f32_f32(vec_s, vec_zero, vlmax);
                float sum = __riscv_vfmv_f_s_f32m1_f32(vec_sum);
                c[i * m + j] = sum;
            }
        }
    } else {
        for (int i = 0; i < n; i += tile_size) {
            for (int j = 0; j < m; j += tile_size) {
                for (int k = 0; k < o; k += tile_size) {
                    matmul_rvvt(a, b, c, i, j, k, n, m, o, tile_size, ind_a);
                }
            }
        }
    }
}

// ============================================================================
// Matrix utility functions
// ============================================================================

// Simple matrix inverse using Gauss-Jordan elimination
bool matrix_inverse_simple_rvv(const float* A, float* A_inv, int n) {
    const float EPS = 1e-10f;
    
    // Create augmented matrix [A | I]
    float* aug = alloc_matrix_rvv(n, 2 * n);
    if (!aug) return false;
    
    // Initialize augmented matrix
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
        if (fabs(pivot) < EPS) {
            free_matrix_rvv(aug);
            return false;
        }
        
        // Scale pivot row
        float inv_pivot = 1.0f / pivot;
        for (int j = 0; j < 2 * n; ++j) {
            aug[i * (2 * n) + j] *= inv_pivot;
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
    
    // Extract inverse from right side
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            A_inv[i * n + j] = aug[i * (2 * n) + j + n];
        }
    }
    
    free_matrix_rvv(aug);
    return true;
}

// Enhanced matrix multiplication that can handle transposes
void enhanced_matmul_rvv(const float* A, const float* B, float* C, 
                        int n, int m, int k, bool transpose_A = false, bool transpose_B = false) {
    if (transpose_A || transpose_B) {
        // For transpose operations, use pure RVV
        if (transpose_A && !transpose_B) {
            // A^T * B: need to handle transpose of A
            float* AT = alloc_matrix_rvv(k, n);
            transpose_rvv(A, AT, n, k);
            matmul_rvv(AT, B, C, k, m, n);
            free_matrix_rvv(AT);
        } else if (!transpose_A && transpose_B) {
            // A * B^T: need to handle transpose of B
            float* BT = alloc_matrix_rvv(k, m);
            transpose_rvv(B, BT, m, k);
            matmul_rvv(A, BT, C, n, m, k);
            free_matrix_rvv(BT);
        } else {
            // A^T * B^T: handle both transposes
            float* AT = alloc_matrix_rvv(k, n);
            float* BT = alloc_matrix_rvv(k, m);
            transpose_rvv(A, AT, n, k);
            transpose_rvv(B, BT, m, k);
            matmul_rvv(AT, BT, C, k, m, n);
            free_matrix_rvv(AT);
            free_matrix_rvv(BT);
        }
    } else {
        // Standard multiplication, use optimized RVV with tiling
        int tile_size = 8; // Optimal tile size for RVV
        matmul_rvv(A, B, C, n, m, k, tile_size);
    }
}

// Matrix comparison
float matrix_frobenius_norm_diff_rvv(const float* A, const float* B, int rows, int cols) {
    float norm_sq = 0.0f;
    for (int i = 0; i < rows * cols; ++i) {
        float diff = A[i] - B[i];
        norm_sq += diff * diff;
    }
    return sqrtf(norm_sq);
}

// Random matrix generation
void generate_random_matrix_rvv(float* mat, int rows, int cols, float scale = 1.0f) {
    for (int i = 0; i < rows * cols; ++i) {
        mat[i] = scale * (2.0f * (float)rand() / RAND_MAX - 1.0f);
    }
}

void make_positive_definite_rvv(float* mat, int n, float regularization = 1.0f) {
    // Add regularization to diagonal to ensure positive definiteness
    for (int i = 0; i < n; ++i) {
        mat[i * n + i] += regularization;
    }
}

// ============================================================================
// LQR Riccati Solver Implementation
// ============================================================================

// Optimized LQR solver that minimizes memory allocations and uses efficient RVV operations
float* lqrSolveFiniteHorizonRVVOptimized(const float* A, const float* B, const float* Q, const float* R, 
                                        int state_dim, int input_dim, int horizon) {
    
    // Pre-allocate all working matrices to minimize allocation overhead
    float* P = alloc_matrix_rvv(state_dim, state_dim);
    float* P_next = alloc_matrix_rvv(state_dim, state_dim);
    float* BT = alloc_matrix_rvv(input_dim, state_dim);
    float* AT = alloc_matrix_rvv(state_dim, state_dim);
    
    // Intermediate computation matrices
    float* temp1 = alloc_matrix_rvv(max(state_dim, input_dim), max(state_dim, input_dim));
    float* temp2 = alloc_matrix_rvv(max(state_dim, input_dim), max(state_dim, input_dim));
    float* temp3 = alloc_matrix_rvv(max(state_dim, input_dim), max(state_dim, input_dim));
    
    // Riccati equation specific matrices
    float* PB = alloc_matrix_rvv(state_dim, input_dim);
    float* PA = alloc_matrix_rvv(state_dim, state_dim);
    float* BTPB_R = alloc_matrix_rvv(input_dim, input_dim);
    float* BTPB_R_inv = alloc_matrix_rvv(input_dim, input_dim);
    float* BTPA = alloc_matrix_rvv(input_dim, state_dim);
    
    // Result matrix
    float* K_result = alloc_matrix_rvv(input_dim, state_dim);
    
    if (!P || !P_next || !BT || !AT || !temp1 || !temp2 || !temp3 || 
        !PB || !PA || !BTPB_R || !BTPB_R_inv || !BTPA || !K_result) {
        // Cleanup on allocation failure
        free_matrix_rvv(P); free_matrix_rvv(P_next); free_matrix_rvv(BT); free_matrix_rvv(AT);
        free_matrix_rvv(temp1); free_matrix_rvv(temp2); free_matrix_rvv(temp3);
        free_matrix_rvv(PB); free_matrix_rvv(PA); free_matrix_rvv(BTPB_R);
        free_matrix_rvv(BTPB_R_inv); free_matrix_rvv(BTPA); free_matrix_rvv(K_result);
        return nullptr;
    }
    
    // Initialize P = Q and precompute transposes
    matcopy_rvv(Q, P, state_dim, state_dim);
    transpose_rvv(B, BT, state_dim, input_dim);
    transpose_rvv(A, AT, state_dim, state_dim);
    
    // Riccati recursion
    for (int t = horizon - 1; t >= 0; --t) {
        // Step 1: Compute PB = P * B using vectorized multiplication
        enhanced_matmul_rvv(P, B, PB, state_dim, input_dim, state_dim);
        
        // Step 2: Compute BTPB = B^T * PB
        enhanced_matmul_rvv(BT, PB, temp1, input_dim, input_dim, state_dim);
        
        // Step 3: BTPB_R = BTPB + R
        matadd_rvv(temp1, R, BTPB_R, input_dim, input_dim);
        
        // Step 4: Compute PA = P * A
        enhanced_matmul_rvv(P, A, PA, state_dim, state_dim, state_dim);
        
        // Step 5: Compute BTPA = B^T * PA  
        enhanced_matmul_rvv(BT, PA, BTPA, input_dim, state_dim, state_dim);
        
        // Step 6: Solve (BTPB + R) * K = BTPA for K
        if (!matrix_inverse_simple_rvv(BTPB_R, BTPB_R_inv, input_dim)) {
            cerr << "Matrix inversion failed at iteration " << t << endl;
            break;
        }
        
        // K = BTPB_R_inv * BTPA
        enhanced_matmul_rvv(BTPB_R_inv, BTPA, K_result, input_dim, state_dim, input_dim);
        
        // Step 7: Update P for next iteration
        // P_next = A^T * P * A - A^T * P * B * K + Q
        
        // Compute A^T * PA = A^T * P * A
        enhanced_matmul_rvv(AT, PA, temp2, state_dim, state_dim, state_dim);
        
        // Compute A^T * P * B * K = BTPA^T * K (since (A^T * P * B)^T = B^T * P * A)
        enhanced_matmul_rvv(BTPA, K_result, temp3, state_dim, state_dim, input_dim, true, false);
        
        // P_next = (A^T * P * A) - (A^T * P * B * K) + Q
        matsub_rvv(temp2, temp3, P_next, state_dim, state_dim);
        matadd_rvv(P_next, Q, P_next, state_dim, state_dim);
        
        // Swap P and P_next for next iteration
        float* temp_ptr = P;
        P = P_next;
        P_next = temp_ptr;
    }
    
    // Copy final result
    float* final_result = alloc_matrix_rvv(input_dim, state_dim);
    if (final_result) {
        matcopy_rvv(K_result, final_result, input_dim, state_dim);
    }
    
    // Cleanup
    free_matrix_rvv(P); free_matrix_rvv(P_next); free_matrix_rvv(BT); free_matrix_rvv(AT);
    free_matrix_rvv(temp1); free_matrix_rvv(temp2); free_matrix_rvv(temp3);
    free_matrix_rvv(PB); free_matrix_rvv(PA); free_matrix_rvv(BTPB_R);
    free_matrix_rvv(BTPB_R_inv); free_matrix_rvv(BTPA); free_matrix_rvv(K_result);
    
    return final_result;
}

// ============================================================================
// Test matrix generation functions
// ============================================================================

float* createRandomStableMatrix(int size) {
    float* matrix = alloc_matrix_rvv(size, size);
    if (!matrix) return nullptr;
    
    // Generate random matrix with eigenvalues < 1 for stability
    generate_random_matrix_rvv(matrix, size, size, 0.5f);
    
    // Make it more stable by scaling diagonal
    for (int i = 0; i < size; ++i) {
        matrix[i * size + i] *= 0.8f;
    }
    
    return matrix;
}

float* createRandomPositiveDefiniteMatrix(int size) {
    float* temp = alloc_matrix_rvv(size, size);
    float* result = alloc_matrix_rvv(size, size);
    float* temp_T = alloc_matrix_rvv(size, size);
    
    if (!temp || !result || !temp_T) {
        free_matrix_rvv(temp);
        free_matrix_rvv(result);
        free_matrix_rvv(temp_T);
        return nullptr;
    }
    
    generate_random_matrix_rvv(temp, size, size, 1.0f);
    transpose_rvv(temp, temp_T, size, size);
    enhanced_matmul_rvv(temp_T, temp, result, size, size, size);
    make_positive_definite_rvv(result, size, 1.0f);
    
    free_matrix_rvv(temp);
    free_matrix_rvv(temp_T);
    
    return result;
}

// ============================================================================
// Main function
// ============================================================================

int main() {
    cout << "state_space_size,action_space_size,horizon_length,time_rvv_standalone,error_check\n";
    
    vector<int> dimensions = {4, 8, 16};
    int horizon = 2;
    
    for (int input_dim : dimensions) {
        for (int state_dim : dimensions) {
            uint64_t t0, t1, time_rvv_standalone;
            
            srand(static_cast<unsigned>(time(0)));
            
            // Generate test matrices
            float* A = createRandomStableMatrix(state_dim);
            float* B = alloc_matrix_rvv(state_dim, input_dim);
            float* Q = createRandomPositiveDefiniteMatrix(state_dim);
            float* R = createRandomPositiveDefiniteMatrix(input_dim);
            
            if (!A || !B || !Q || !R) {
                cerr << "Memory allocation failed for dimension " << state_dim << "x" << input_dim << endl;
                continue;
            }
            
            generate_random_matrix_rvv(B, state_dim, input_dim, 1.0f);
            
            // Test standalone RVV implementation
            t0 = read_cycles();
            float* K_standalone = lqrSolveFiniteHorizonRVVOptimized(A, B, Q, R, state_dim, input_dim, horizon);
            t1 = read_cycles();
            time_rvv_standalone = t1 - t0;
            
            // Basic error check (verify K is not null and has reasonable values)
            float error_check = 0.0f;
            if (K_standalone) {
                // Simple sanity check - compute norm of K
                for (int i = 0; i < input_dim * state_dim; ++i) {
                    error_check += K_standalone[i] * K_standalone[i];
                }
                error_check = sqrtf(error_check);
            }
            
            cout << state_dim << "," << input_dim << "," << horizon << "," 
                 << time_rvv_standalone << "," << error_check << "\n";
            
            // Cleanup
            free_matrix_rvv(A);
            free_matrix_rvv(B);
            free_matrix_rvv(Q);
            free_matrix_rvv(R);
            free_matrix_rvv(K_standalone);
        }
    }
    
    return 0;
}