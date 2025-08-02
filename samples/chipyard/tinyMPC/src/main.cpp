#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>
#include <zephyr/random/random.h>

#include <stdlib.h>
#include <string.h>
#include <math.h>

// RISC-V Vector Extension intrinsics
#include "riscv_vector.h"

// Simple vector replacement for std::vector
template<typename T>
class simple_vector {
private:
    T* data_;
    size_t size_;
    size_t capacity_;
    
public:
    simple_vector(size_t initial_size = 0) : size_(initial_size), capacity_(initial_size) {
        data_ = initial_size > 0 ? (T*)k_aligned_alloc(32, initial_size * sizeof(T)) : nullptr;
    }
    
    ~simple_vector() {
        if (data_) k_free(data_);
    }
    
    T& operator[](size_t index) { return data_[index]; }
    const T& operator[](size_t index) const { return data_[index]; }
    size_t size() const { return size_; }
};

// ============================================================================
// Memory allocation utilities for Zephyr
// ============================================================================

static inline float* alloc_matrix(int rows, int cols) {
    return (float*)k_aligned_alloc(32, rows * cols * sizeof(float));
}

static inline void free_matrix(float* ptr) {
    if (ptr) k_free(ptr);
}

// ============================================================================
// RVV Matrix Operations (self-contained)
// ============================================================================

#ifndef BATCH
#define BATCH 4
#endif
static inline void enable_vector_operations() {
    unsigned long mstatus;

    // Read current mstatus
    asm volatile("csrr %0, mstatus" : "=r"(mstatus));

    // Set VS field to Dirty (11)
    mstatus |= MSTATUS_VS | MSTATUS_FS | MSTATUS_XS;

    // Write back updated mstatus
    asm volatile("csrw mstatus, %0"::"r"(mstatus));
}
// Matrix addition using RVV
inline void matadd_rvv(const float *ptr_a, const float *ptr_b, float *ptr_c, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32m1(k);
        vfloat32m1_t vec_a = __riscv_vle32_v_f32m1(ptr_a + l, vl);
        vfloat32m1_t vec_b = __riscv_vle32_v_f32m1(ptr_b + l, vl);
        vfloat32m1_t vec_c = __riscv_vfadd_vv_f32m1(vec_a, vec_b, vl);
        __riscv_vse32_v_f32m1(ptr_c + l, vec_c, vl);
    }
}

// Matrix subtraction using RVV
inline void matsub_rvv(const float *ptr_a, const float *ptr_b, float *ptr_c, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32m1(k);
        vfloat32m1_t vec_a = __riscv_vle32_v_f32m1(ptr_a + l, vl);
        vfloat32m1_t vec_b = __riscv_vle32_v_f32m1(ptr_b + l, vl);
        vfloat32m1_t vec_c = __riscv_vfsub_vv_f32m1(vec_a, vec_b, vl);
        __riscv_vse32_v_f32m1(ptr_c + l, vec_c, vl);
    }
}

// Matrix copy using RVV
inline void matcopy_rvv(const float *ptr_a, float *ptr_b, int n, int m) {
    int k = n * m, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32m1(k);
        vfloat32m1_t vec_a = __riscv_vle32_v_f32m1(ptr_a + l, vl);
        __riscv_vse32_v_f32m1(ptr_b + l, vec_a, vl);
    }
}

// Matrix set to scalar value using RVV
inline void matset_rvv(float *ptr_a, float f, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = __riscv_vsetvl_e32m1(k);
        vfloat32m1_t vec_a = __riscv_vfmv_v_f_f32m1(f, vl);
        __riscv_vse32_v_f32m1(ptr_a + l, vec_a, vl);
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
            vl = __riscv_vsetvl_e32m1(k);
            vfloat32m1_t vec_a = __riscv_vlse32_v_f32m1(ptr_a, sizeof(float) * m, vl);
            __riscv_vse32_v_f32m1(ptr_b, vec_a, vl);
        }
    }
}

// Tiled matrix multiplication helper (optimized RVV version)
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
    
    size_t vlmax = __riscv_vsetvlmax_e32m1();
    vfloat32m1_t vec_zero = __riscv_vfmv_v_f_f32m1(0, vlmax);
    
    for (int I = 0; I < N; I++) {
        const float *ptr_a_0 = A + (ind_a ? ind_a[I + i] : I * o);
        
        for (int J = 0; J < M; J += BATCH) {
            int P = J + BATCH <= M ? BATCH : M - J;
            const float *ptr_a = ptr_a_0;
            const float *ptr_b = B + J * o;
            int K = O;
            
            for (int L = 0; L < P; L++) {
                *(vec_r[L]) = __riscv_vfmv_v_f_f32m1(0, vlmax);
            }
            
            for (size_t vl; K > 0; K -= vl, ptr_a += vl, ptr_b += vl) {
                vl = __riscv_vsetvl_e32m1(K);
                vfloat32m1_t vec_a = __riscv_vle32_v_f32m1(ptr_a, vl);
                for (int L = 0; L < P; L++) {
                    vfloat32m1_t vec_b = __riscv_vle32_v_f32m1(ptr_b + L * o, vl);
                    *(vec_r[L]) = __riscv_vfmacc_vv_f32m1(*(vec_r[L]), vec_a, vec_b, vl);
                }
            }
            
            for (int L = 0; L < P; L++) {
                vfloat32m1_t vec_sum = __riscv_vfredusum_vs_f32m1_f32m1(*(vec_r[L]), vec_zero, vlmax);
                float sum = __riscv_vfmv_f_s_f32m1_f32(vec_sum);
                C[I * m + J + L] = k == 0 ? sum : C[I * m + J + L] + sum;
            }
        }
    }
}

// Matrix multiplication using RVV
inline void matmul_rvv(const float *a, const float *b, float *c, int n, int m, int o, int tile_size = -1, int *ind_a = nullptr) {
    if (tile_size == -1) {
        size_t vlmax = __riscv_vsetvlmax_e32m1();
        vfloat32m1_t vec_zero = __riscv_vfmv_v_f_f32m1(0, vlmax);
        
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                const float *ptr_a = a + i * o; // row major
                const float *ptr_b = b + j * o; // column major
                int k = o;
                vfloat32m1_t vec_s = __riscv_vfmv_v_f_f32m1(0, vlmax);
                
                for (size_t vl; k > 0; k -= vl, ptr_a += vl, ptr_b += vl) {
                    vl = __riscv_vsetvl_e32m1(k);
                    vfloat32m1_t vec_a = __riscv_vle32_v_f32m1(ptr_a, vl);
                    vfloat32m1_t vec_b = __riscv_vle32_v_f32m1(ptr_b, vl);
                    vec_s = __riscv_vfmacc_vv_f32m1(vec_s, vec_a, vec_b, vl);
                }
                
                vfloat32m1_t vec_sum = __riscv_vfredusum_vs_f32m1_f32m1(vec_s, vec_zero, vlmax);
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

// Matrix inverse using Gauss-Jordan elimination
bool matrix_inverse_rvv(const float* A, float* A_inv, int n) {
    float* aug = alloc_matrix(n, 2 * n);
    if (!aug) return false;
    
    // Copy A to left side and create identity on right side
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            aug[i * (2 * n) + j] = A[i * n + j];
            aug[i * (2 * n) + j + n] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Gauss-Jordan elimination
    for (int i = 0; i < n; ++i) {
        float pivot = aug[i * (2 * n) + i];
        if (fabsf(pivot) < 1e-10f) {
            free_matrix(aug);
            return false;
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
    
    // Copy result from right side
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            A_inv[i * n + j] = aug[i * (2 * n) + j + n];
        }
    }
    
    free_matrix(aug);
    return true;
}

// ============================================================================
// LQR Riccati Solver Implementation
// ============================================================================

float* lqrSolveFiniteHorizonRVV(const float* A, const float* B, const float* Q, const float* R, 
                               int state_dim, int input_dim, int horizon) {
    
    // Allocate gain matrices
    simple_vector<float*> K(horizon);
    for (int t = 0; t < horizon; ++t) {
        K[t] = alloc_matrix(input_dim, state_dim);
        if (!K[t]) return nullptr;
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
    
    if (!P || !BT || !AT || !PB || !PA || !BTPB || !BTPB_R || !BTPB_R_inv || !BTPA || !ATPA || !BTPAK) {
        // Cleanup on allocation failure
        for (int i = 0; i < horizon; ++i) free_matrix(K[i]);
        free_matrix(P); free_matrix(BT); free_matrix(AT);
        free_matrix(PB); free_matrix(PA); free_matrix(BTPB);
        free_matrix(BTPB_R); free_matrix(BTPB_R_inv); free_matrix(BTPA);
        free_matrix(ATPA); free_matrix(BTPAK);
        return nullptr;
    }
    
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
            printk("Matrix inversion failed at iteration %d\n", t);
            // Cleanup and return nullptr
            for (int i = 0; i < (int)K.size(); ++i) free_matrix(K[i]);
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
        matmul_rvv(BTPA, K[t], BTPAK, state_dim, state_dim, input_dim);
        
        // P = ATPA - BTPAK + Q
        matsub_rvv(ATPA, BTPAK, P, state_dim, state_dim);
        matadd_rvv(P, Q, P, state_dim, state_dim);
    }
    
    // Allocate and copy the result (K[0])
    float* result = alloc_matrix(input_dim, state_dim);
    if (result) {
        matcopy_rvv(K[0], result, input_dim, state_dim);
    }
    
    // Cleanup
    for (int t = 0; t < (int)K.size(); ++t) free_matrix(K[t]);
    free_matrix(P); free_matrix(BT); free_matrix(AT);
    free_matrix(PB); free_matrix(PA); free_matrix(BTPB);
    free_matrix(BTPB_R); free_matrix(BTPB_R_inv); free_matrix(BTPA);
    free_matrix(ATPA); free_matrix(BTPAK);
    
    return result;
}

// ============================================================================
// Test matrix generation functions
// ============================================================================

float* randomStableMatrix(int size) {
    float* matrix = alloc_matrix(size, size);
    if (!matrix) return nullptr;
    
    // Generate simple stable matrix
    matset_rvv(matrix, 0.0f, size, size);
    for (int i = 0; i < size; ++i) {
        matrix[i * size + i] = 0.5f + 0.3f * (float)sys_rand32_get() / UINT32_MAX;
        for (int j = 0; j < size; ++j) {
            if (i != j) {
                matrix[i * size + j] = 0.1f * ((float)sys_rand32_get() / UINT32_MAX - 0.5f);
            }
        }
    }
    
    return matrix;
}

float* randomPositiveDefiniteMatrix(int size) {
    float* matrix = alloc_matrix(size, size);
    if (!matrix) return nullptr;
    
    // Generate diagonal dominant matrix
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (i == j) {
                matrix[i * size + j] = 2.0f + (float)sys_rand32_get() / UINT32_MAX;
            } else {
                matrix[i * size + j] = 0.1f * ((float)sys_rand32_get() / UINT32_MAX - 0.5f);
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
    return sqrtf(norm);
}

// ============================================================================
// Zephyr Application Entry Point
// ============================================================================

int main(void) {
    printk("=== Zephyr RVV LQR Riccati Implementation ===\n");
    printk("Running on Chipyard Saturn with RISC-V Vector Extension\n\n");
    
    // Enable vector operations
    // enable_vector_operations();
    
    // Initialize timing
    timing_init();
    timing_start();
    
    printk("state_space_size,action_space_size,horizon_length,time_cycles,result_norm\n");
    
    int dimensions[] = {4, 8};
    int num_dims = sizeof(dimensions) / sizeof(dimensions[0]);
    int horizon = 2;
    
    for (int i = 0; i < num_dims; ++i) {
        for (int j = 0; j < num_dims; ++j) {
            int input_dim = dimensions[i];
            int state_dim = dimensions[j];
            
            uint64_t t0, t1;
            uint64_t time_cycles;
            
            // Generate system matrices
            float* A = randomStableMatrix(state_dim);
            float* B = alloc_matrix(state_dim, input_dim);
            if (!A || !B) {
                printk("Memory allocation failed for dimension %dx%d\n", state_dim, input_dim);
                continue;
            }
            
            for (int k = 0; k < state_dim * input_dim; ++k) {
                B[k] = (float)sys_rand32_get() / UINT32_MAX - 0.5f;
            }
            
            // Generate cost matrices
            float* Q = randomPositiveDefiniteMatrix(state_dim);
            float* R = randomPositiveDefiniteMatrix(input_dim);
            if (!Q || !R) {
                printk("Memory allocation failed for cost matrices\n");
                free_matrix(A);
                free_matrix(B);
                continue;
            }
            
            // Solve using RVV implementation
            timing_start();
            t0 = k_cycle_get_64();
            float* K_result = lqrSolveFiniteHorizonRVV(A, B, Q, R, state_dim, input_dim, horizon);
            t1 = k_cycle_get_64();
            time_cycles = t1 - t0;
            
            // Compute result norm as a sanity check
            float result_norm = 0.0f;
            if (K_result) {
                for (int k = 0; k < input_dim * state_dim; ++k) {
                    result_norm += K_result[k] * K_result[k];
                }
                result_norm = sqrt(result_norm);
            }
            
            printk("%d,%d,%d,%llu,%.6f\n", 
                   state_dim, input_dim, horizon, time_cycles, (double)result_norm);
            
            // Cleanup
            free_matrix(A);
            free_matrix(B);
            free_matrix(Q);
            free_matrix(R);
            if (K_result) free_matrix(K_result);
        }
    }
    
    printk("\n=== RVV LQR Riccati Test Complete ===\n");
    
    return 0;
}