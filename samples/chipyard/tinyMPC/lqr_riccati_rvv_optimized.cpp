#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <fstream>
#include "gemmini.h"
#include "rvv_utils.h"

// Include the RVV matrix library
extern "C" {
#include "matlib_rvv.h"
}

using namespace std;

static uint64_t read_cycles() {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

// Enhanced matrix multiplication that can use both RVV and Gemmini
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

// LU-based solver for comparison and robustness
float* lqrSolveFiniteHorizonRVVLU(const float* A, const float* B, const float* Q, const float* R, 
                                 int state_dim, int input_dim, int horizon) {
    
    // Allocate working matrices
    float* P = alloc_matrix_rvv(state_dim, state_dim);
    float* BT = alloc_matrix_rvv(input_dim, state_dim);
    float* AT = alloc_matrix_rvv(state_dim, state_dim);
    float* PB = alloc_matrix_rvv(state_dim, input_dim);
    float* PA = alloc_matrix_rvv(state_dim, state_dim);
    float* BTPB_R = alloc_matrix_rvv(input_dim, input_dim);
    float* BTPA = alloc_matrix_rvv(input_dim, state_dim);
    float* K_result = alloc_matrix_rvv(input_dim, state_dim);
    
    // LU decomposition matrices
    float* L = alloc_matrix_rvv(input_dim, input_dim);
    float* U = alloc_matrix_rvv(input_dim, input_dim);
    int* P_perm = (int*)malloc(input_dim * sizeof(int));
    float* y = alloc_matrix_rvv(input_dim, 1);
    float* x = alloc_matrix_rvv(input_dim, 1);
    float* b_col = alloc_matrix_rvv(input_dim, 1);
    
    if (!P || !BT || !AT || !PB || !PA || !BTPB_R || !BTPA || !K_result ||
        !L || !U || !P_perm || !y || !x || !b_col) {
        goto cleanup;
    }
    
    // Initialize
    matcopy_rvv(Q, P, state_dim, state_dim);
    transpose_rvv(B, BT, state_dim, input_dim);
    transpose_rvv(A, AT, state_dim, state_dim);
    
    for (int t = horizon - 1; t >= 0; --t) {
        // Compute Riccati equation components
        enhanced_matmul_rvv(P, B, PB, state_dim, input_dim, state_dim);
        enhanced_matmul_rvv(BT, PB, BTPB_R, input_dim, input_dim, state_dim);
        matadd_rvv(BTPB_R, R, BTPB_R, input_dim, input_dim);
        
        enhanced_matmul_rvv(P, A, PA, state_dim, state_dim, state_dim);
        enhanced_matmul_rvv(BT, PA, BTPA, input_dim, state_dim, state_dim);
        
        // LU decomposition of BTPB_R
        if (!lu_decomp_rvv(BTPB_R, L, U, P_perm, input_dim)) {
            cerr << "LU decomposition failed at iteration " << t << endl;
            break;
        }
        
        // Solve for each column of K
        for (int col = 0; col < state_dim; ++col) {
            // Extract column from BTPA with permutation
            for (int row = 0; row < input_dim; ++row) {
                b_col[row] = BTPA[P_perm[row] * state_dim + col];
            }
            
            // Forward substitution: L * y = b
            forward_substitution_rvv(L, b_col, y, input_dim);
            
            // Back substitution: U * x = y
            back_substitution_rvv(U, y, x, input_dim);
            
            // Store result in K
            for (int row = 0; row < input_dim; ++row) {
                K_result[row * state_dim + col] = x[row];
            }
        }
        
        // Update P for next iteration (similar to optimized version)
        float* temp1 = alloc_matrix_rvv(state_dim, state_dim);
        float* temp2 = alloc_matrix_rvv(state_dim, state_dim);
        
        if (temp1 && temp2) {
            enhanced_matmul_rvv(AT, PA, temp1, state_dim, state_dim, state_dim);
            enhanced_matmul_rvv(BTPA, K_result, temp2, state_dim, state_dim, input_dim, true, false);
            matsub_rvv(temp1, temp2, P, state_dim, state_dim);
            matadd_rvv(P, Q, P, state_dim, state_dim);
        }
        
        free_matrix_rvv(temp1);
        free_matrix_rvv(temp2);
    }
    
cleanup:
    // Allocate final result
    float* final_result = nullptr;
    if (K_result) {
        final_result = alloc_matrix_rvv(input_dim, state_dim);
        if (final_result) {
            matcopy_rvv(K_result, final_result, input_dim, state_dim);
        }
    }
    
    // Cleanup all matrices
    free_matrix_rvv(P); free_matrix_rvv(BT); free_matrix_rvv(AT);
    free_matrix_rvv(PB); free_matrix_rvv(PA); free_matrix_rvv(BTPB_R);
    free_matrix_rvv(BTPA); free_matrix_rvv(K_result);
    free_matrix_rvv(L); free_matrix_rvv(U); free_matrix_rvv(y);
    free_matrix_rvv(x); free_matrix_rvv(b_col);
    if (P_perm) free(P_perm);
    
    return final_result;
}

// Utility functions for matrix generation
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

int main() {
    cout << "state_space_size,action_space_size,horizon_length,time_rvv_opt,time_rvv_lu,error_norm\n";
    
    vector<int> dimensions = {4, 8, 16};
    int horizon = 2;
    
    for (int input_dim : dimensions) {
        for (int state_dim : dimensions) {
            uint64_t t0, t1, time_rvv_opt, time_rvv_lu;
            
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
            
            // Test optimized RVV implementation
            t0 = read_cycles();
            float* K_opt = lqrSolveFiniteHorizonRVVOptimized(A, B, Q, R, state_dim, input_dim, horizon);
            t1 = read_cycles();
            time_rvv_opt = t1 - t0;
            
            // Test LU-based RVV implementation
            t0 = read_cycles();
            float* K_lu = lqrSolveFiniteHorizonRVVLU(A, B, Q, R, state_dim, input_dim, horizon);
            t1 = read_cycles();
            time_rvv_lu = t1 - t0;
            
            // Compare results
            float error_norm = 0.0f;
            if (K_opt && K_lu) {
                error_norm = matrix_frobenius_norm_diff_rvv(K_opt, K_lu, input_dim, state_dim);
            }
            
            cout << state_dim << "," << input_dim << "," << horizon << "," 
                 << time_rvv_opt << "," << time_rvv_lu << "," << error_norm << "\n";
            
            // Cleanup
            free_matrix_rvv(A);
            free_matrix_rvv(B);
            free_matrix_rvv(Q);
            free_matrix_rvv(R);
            free_matrix_rvv(K_opt);
            free_matrix_rvv(K_lu);
        }
    }
    
    return 0;
}