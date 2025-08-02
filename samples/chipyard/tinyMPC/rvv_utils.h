#ifndef RVV_UTILS_H
#define RVV_UTILS_H

#include <cstdlib>
#include <cstring>
#include <cmath>

#ifdef __cplusplus
extern "C" {
#endif

// Memory allocation utilities for RVV matrices
static inline float* alloc_matrix_rvv(int rows, int cols) {
    return (float*)aligned_alloc(32, rows * cols * sizeof(float));
}

static inline void free_matrix_rvv(float* ptr) {
    if (ptr) free(ptr);
}

// Matrix utilities not provided in matlib_rvv.h
static inline void matset_identity_rvv(float* mat, int n) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            mat[i * n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

// Simple matrix inverse using Gauss-Jordan (for small matrices)
static inline bool matrix_inverse_simple_rvv(const float* A, float* A_inv, int n) {
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

// LU decomposition with partial pivoting
static inline bool lu_decomp_rvv(const float* A, float* L, float* U, int* P, int n) {
    const float EPS = 1e-10f;
    
    // Initialize
    float* A_copy = alloc_matrix_rvv(n, n);
    if (!A_copy) return false;
    
    memcpy(A_copy, A, n * n * sizeof(float));
    
    // Initialize L as identity and P as identity permutation
    for (int i = 0; i < n; ++i) {
        P[i] = i;
        for (int j = 0; j < n; ++j) {
            L[i * n + j] = (i == j) ? 1.0f : 0.0f;
            U[i * n + j] = 0.0f;
        }
    }
    
    // LU decomposition with partial pivoting
    for (int i = 0; i < n; ++i) {
        // Find pivot
        int pivot_row = i;
        float max_val = fabs(A_copy[i * n + i]);
        for (int k = i + 1; k < n; ++k) {
            if (fabs(A_copy[k * n + i]) > max_val) {
                max_val = fabs(A_copy[k * n + i]);
                pivot_row = k;
            }
        }
        
        if (max_val < EPS) {
            free_matrix_rvv(A_copy);
            return false;
        }
        
        // Swap rows
        if (pivot_row != i) {
            int temp_p = P[i];
            P[i] = P[pivot_row];
            P[pivot_row] = temp_p;
            
            for (int j = 0; j < n; ++j) {
                float temp = A_copy[i * n + j];
                A_copy[i * n + j] = A_copy[pivot_row * n + j];
                A_copy[pivot_row * n + j] = temp;
            }
        }
        
        // Store U row
        for (int j = i; j < n; ++j) {
            U[i * n + j] = A_copy[i * n + j];
        }
        
        // Compute L column and update A
        for (int k = i + 1; k < n; ++k) {
            L[k * n + i] = A_copy[k * n + i] / A_copy[i * n + i];
            for (int j = i; j < n; ++j) {
                A_copy[k * n + j] -= L[k * n + i] * A_copy[i * n + j];
            }
        }
    }
    
    free_matrix_rvv(A_copy);
    return true;
}

// Forward and back substitution for LU solve
static inline void forward_substitution_rvv(const float* L, const float* b, float* y, int n) {
    for (int i = 0; i < n; ++i) {
        y[i] = b[i];
        for (int j = 0; j < i; ++j) {
            y[i] -= L[i * n + j] * y[j];
        }
    }
}

static inline void back_substitution_rvv(const float* U, const float* y, float* x, int n) {
    for (int i = n - 1; i >= 0; --i) {
        x[i] = y[i];
        for (int j = i + 1; j < n; ++j) {
            x[i] -= U[i * n + j] * x[j];
        }
        x[i] /= U[i * n + i];
    }
}

// Random matrix generation
static inline void generate_random_matrix_rvv(float* mat, int rows, int cols, float scale = 1.0f) {
    for (int i = 0; i < rows * cols; ++i) {
        mat[i] = scale * (2.0f * (float)rand() / RAND_MAX - 1.0f);
    }
}

static inline void make_positive_definite_rvv(float* mat, int n, float regularization = 1.0f) {
    // Add regularization to diagonal to ensure positive definiteness
    for (int i = 0; i < n; ++i) {
        mat[i * n + i] += regularization;
    }
}

// Matrix comparison
static inline float matrix_frobenius_norm_diff_rvv(const float* A, const float* B, int rows, int cols) {
    float norm_sq = 0.0f;
    for (int i = 0; i < rows * cols; ++i) {
        float diff = A[i] - B[i];
        norm_sq += diff * diff;
    }
    return sqrtf(norm_sq);
}

#ifdef __cplusplus
}
#endif

#endif // RVV_UTILS_H