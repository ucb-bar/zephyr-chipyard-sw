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
static inline float* alloc_matrix(int rows, int cols) {
    return (float*)aligned_alloc(32, rows * cols * sizeof(float));
}

static inline void free_matrix(float* ptr) {
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

// Matrix inverse using Gauss-Jordan elimination (since not provided in RVV lib)
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

// LU solve using partial pivoting (basic implementation)
bool lu_solve_rvv(const float* A, const float* b, float* x, int n) {
    // Simple LU solve - could be optimized further with RVV
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
        x[i] /= A_copy[i * n + i];
    }
    
    free_matrix(A_copy);
    free_matrix(b_copy);
    return true;
}

// ============================================================================
// LQR Riccati Solver Implementation
// ============================================================================

// RVV-vectorized LQR solver using matrix inverse
float* lqrSolveFiniteHorizonRVV(const float* A, const float* B, const float* Q, const float* R, 
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
        
        // BTPAK = BTPA^T * K[t] = (A^T * P * B) * K[t]  
        matmul_rvv(BTPA, K[t], BTPAK, state_dim, state_dim, input_dim, -1, nullptr);
        
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

// RVV-vectorized LQR solver using LU decomposition
float* lqrSolveFiniteHorizonRVVLU(const float* A, const float* B, const float* Q, const float* R, 
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
        
        // PA = P * A
        matmul_rvv(P, A, PA, state_dim, state_dim, state_dim);
        
        // BTPA = B^T * PA
        matmul_rvv(BT, PA, BTPA, input_dim, state_dim, state_dim);
        
        // Solve BTPB_R * K[t]^T = BTPA for K[t]^T using LU decomposition
        for (int i = 0; i < state_dim; ++i) {
            float* btpa_col = alloc_matrix(input_dim, 1);
            float* k_col = alloc_matrix(input_dim, 1);
            
            // Extract column i from BTPA
            for (int j = 0; j < input_dim; ++j) {
                btpa_col[j] = BTPA[j * state_dim + i];
            }
            
            // Solve for column i of K[t]^T
            lu_solve_rvv(BTPB_R, btpa_col, k_col, input_dim);
            
            // Store result in K[t] (transposed)
            for (int j = 0; j < input_dim; ++j) {
                K[t][j * state_dim + i] = k_col[j];
            }
            
            free_matrix(btpa_col);
            free_matrix(k_col);
        }
        
        // Update P for next iteration
        // ATPA = A^T * PA
        matmul_rvv(AT, PA, ATPA, state_dim, state_dim, state_dim);
        
        // BTPAK = BTPA^T * K[t]
        matmul_rvv(BTPA, K[t], BTPAK, state_dim, state_dim, input_dim, -1, nullptr);
        
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
    free_matrix(BTPB_R); free_matrix(BTPA);
    free_matrix(ATPA); free_matrix(BTPAK);
    
    return result;
}

// Random matrix generation utilities
float* randomStableMatrixRVV(int size) {
    float* D = alloc_matrix(size, size);
    float* P = alloc_matrix(size, size);
    float* P_inv = alloc_matrix(size, size);
    float* A = alloc_matrix(size, size);
    float* temp = alloc_matrix(size, size);
    
    // Create diagonal matrix with eigenvalues < 1
    matset_rvv(D, 0.0f, size, size);
    for (int i = 0; i < size; ++i) {
        D[i * size + i] = 0.8f * static_cast<float>(rand()) / RAND_MAX;
    }
    
    // Create random transformation matrix P
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            P[i * size + j] = static_cast<float>(rand()) / RAND_MAX - 0.5f;
        }
    }
    
    // Compute A = P^(-1) * D * P
    if (matrix_inverse_rvv(P, P_inv, size)) {
        matmul_rvv(P_inv, D, temp, size, size, size);
        matmul_rvv(temp, P, A, size, size, size);
    } else {
        // Fallback: just use D
        matcopy_rvv(D, A, size, size);
    }
    
    free_matrix(D);
    free_matrix(P);
    free_matrix(P_inv);
    free_matrix(temp);
    
    return A;
}

float* randomPositiveSemidefiniteMatrixRVV(int size) {
    float* M = alloc_matrix(size, size);
    float* MT = alloc_matrix(size, size);
    float* Q = alloc_matrix(size, size);
    
    // Generate random matrix
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            M[i * size + j] = static_cast<float>(rand()) / RAND_MAX - 0.5f;
        }
    }
    
    // Q = M^T * M
    transpose_rvv(M, MT, size, size);
    matmul_rvv(MT, M, Q, size, size, size);
    
    free_matrix(M);
    free_matrix(MT);
    
    return Q;
}

float* randomPositiveDefiniteMatrixRVV(int size) {
    float* Q = randomPositiveSemidefiniteMatrixRVV(size);
    
    // Add identity to ensure positive definiteness
    for (int i = 0; i < size; ++i) {
        Q[i * size + i] += 1.0f;
    }
    
    return Q;
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
    cout << "state_space_size,action_space_size,horizon_length,time_rvv,time_rvv_lu,error_norm_rvv\n";
    
    vector<int> dimensions = {4, 8};
    int horizon = 2;
    
    for (int input_dim : dimensions) {
        for (int state_dim : dimensions) {
            uint64_t t0, t1, time_rvv, time_rvv_lu;
            
            // Initialize random seed
            srand(static_cast<unsigned>(time(0)));
            
            // Generate system matrices
            float* A = randomStableMatrixRVV(state_dim);
            float* B = alloc_matrix(state_dim, input_dim);
            for (int i = 0; i < state_dim * input_dim; ++i) {
                B[i] = static_cast<float>(rand()) / RAND_MAX - 0.5f;
            }
            
            // Generate cost matrices
            float* Q = randomPositiveDefiniteMatrixRVV(state_dim);
            float* R = randomPositiveSemidefiniteMatrixRVV(input_dim);
            
            // Solve using RVV implementation
            t0 = read_cycles();
            float* K_rvv = lqrSolveFiniteHorizonRVV(A, B, Q, R, state_dim, input_dim, horizon);
            t1 = read_cycles();
            time_rvv = t1 - t0;
            
            // Solve using RVV LU implementation
            t0 = read_cycles();
            float* K_rvv_lu = lqrSolveFiniteHorizonRVVLU(A, B, Q, R, state_dim, input_dim, horizon);
            t1 = read_cycles();
            time_rvv_lu = t1 - t0;
            
            // Compare results
            float error_norm_rvv = 0.0f;
            if (K_rvv && K_rvv_lu) {
                error_norm_rvv = matrixNormDifference(K_rvv, K_rvv_lu, input_dim, state_dim);
            }
            
            cout << state_dim << "," << input_dim << "," << horizon << "," 
                 << time_rvv << "," << time_rvv_lu << "," << error_norm_rvv << "\n";
            
            // Cleanup
            free_matrix(A);
            free_matrix(B);
            free_matrix(Q);
            free_matrix(R);
            if (K_rvv) free_matrix(K_rvv);
            if (K_rvv_lu) free_matrix(K_rvv_lu);
        }
    }
    
    return 0;
}