/*
 * Copyright (c) 2025 Vikram Jain, Dima Nikiforov
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <zephyr/sys/reboot.h>

#include "include/gemmini.h"
#include "include/rerocc.h"

#define CPU_VERIFY_TOLERANCE 1e-1f
#define USE_GEMMINI 1

inline unsigned long read_cycles(void)
{
    unsigned long cc;
    __asm__ volatile("rdcycle %0" : "=r"(cc));
    return cc;
}

//================================================================================
// CPU "Golden" Scalar Implementation (operates on _Float16, computes with float)
//================================================================================

void rmsnorm_cpu(int M, int N, const _Float16* x, const _Float16* weight, _Float16* out) {
    for (int m = 0; m < M; m++) {
        const _Float16* x_row = x + m * N;
        _Float16* out_row = out + m * N;
        float ss = 0.0f;
        for (int n = 0; n < N; n++) {
            float val = (float)x_row[n];
            ss += val * val;
        }
        ss /= (float)N;
        ss += 1e-5f;
        ss = 1.0f / sqrtf(ss);
        for (int n = 0; n < N; n++) {
            float w   = (float)weight[n];
            float val = (float)x_row[n];
            out_row[n] = (_Float16)(w * (val * ss));
        }
    }
}

void precompute_rope_freqs_cpu(int dim, int seq_len, float theta, _Float16* freqs) {
    for (int i = 0; i < dim; i += 2) {
        float base = 1.0f / powf(theta, (float)i / (float)dim);
        for (int j = 0; j < seq_len; j++) {
            float a = j * base;
            freqs[(j * dim) + i]     = (_Float16)cosf(a);
            freqs[(j * dim) + i + 1] = (_Float16)sinf(a);
        }
    }
}

void apply_rope_cpu(int seq_len, int hidden_dim, int num_heads,
                    const _Float16* freqs, _Float16* q, _Float16* k) {
    int head_dim = hidden_dim / num_heads;
    for (int h = 0; h < num_heads; h++) {
        for (int s = 0; s < seq_len; s++) {
            for (int d = 0; d < head_dim; d += 2) {
                int q_idx = (s * hidden_dim) + (h * head_dim) + d;
                int k_idx = (s * hidden_dim) + (h * head_dim) + d;
                int f_idx = (s * head_dim) + d;  // matches how freqs is laid out for each head

                float q0 = (float)q[q_idx];
                float q1 = (float)q[q_idx + 1];
                float k0 = (float)k[k_idx];
                float k1 = (float)k[k_idx + 1];
                float f_cos = (float)freqs[f_idx];
                float f_sin = (float)freqs[f_idx + 1];

                q[q_idx]     = (_Float16)(q0 * f_cos - q1 * f_sin);
                q[q_idx + 1] = (_Float16)(q0 * f_sin + q1 * f_cos);
                k[k_idx]     = (_Float16)(k0 * f_cos - k1 * f_sin);
                k[k_idx + 1] = (_Float16)(k0 * f_sin + k1 * f_cos);
            }
        }
    }
}

void swiglu_cpu(int M, int N, const _Float16* x1, const _Float16* x3, _Float16* out) {
    for (int i = 0; i < M * N; i++) {
        float v1 = (float)x1[i];
        float v3 = (float)x3[i];
        float swish = v1 * (1.0f / (1.0f + expf(-v1)));
        out[i] = (_Float16)(swish * v3);
    }
}



void llama_matmul_cpu(int M, int N, int K, const _Float16* A, const _Float16* B, _Float16* C, bool transB) {
    for (int m = 0; m < M; m++) {
        for (int n = 0; n < N; n++) {
            float result = 0.0f;
            for (int k = 0; k < K; k++) {
                float a_val = (float)A[m * K + k];
                float b_val = (float)(transB ? B[n * K + k] : B[k * N + n]);
                result += a_val * b_val;
            }
            C[m * N + n] = (_Float16)result;
        }
    }
}

void softmax_cpu(int M, int N, float scale, _Float16* C) {
    for (int m = 0; m < M; m++) {
        float max_val = -FLT_MAX;
        for (int n = 0; n < N; n++) {
            float v = (float)C[m * N + n];
            if (v > max_val) max_val = v;
        }
        float sum = 0.0f;
        for (int n = 0; n < N; n++) {
            float v = (float)C[m * N + n];
            sum += expf((v - max_val) * scale);
        }
        float inv_sum = 1.0f / sum;
        for (int n = 0; n < N; n++) {
            float v = (float)C[m * N + n];
            float e = expf((v - max_val) * scale) * inv_sum;
            C[m * N + n] = (_Float16)e;
        }
    }
}

void llama_resadd_cpu(int M, int N, const _Float16* A, const _Float16* B, _Float16* C) {
    for (int i = 0; i < M * N; i++) {
        C[i] = (_Float16)((float)A[i] + (float)B[i]);
    }
}

// ---- Attention ----

typedef struct {
    int hidden_dim, num_heads, seq_len;
    const _Float16* freqs;
    const _Float16* input;
    _Float16* out;
    const _Float16* Wq;
    const _Float16* Wk;
    const _Float16* Wv;
    const _Float16* Wo;
    const _Float16* attn_norm_w;
    _Float16* q_buf;
    _Float16* k_buf;
    _Float16* v_buf;
    _Float16* attn_buf;
    _Float16* out_buf;
} AttentionGoldenArgs;

void attention_cpu_golden(const AttentionGoldenArgs* args) {
    int head_dim = args->hidden_dim / args->num_heads;

    _Float16* cpu_q_buf =
        (_Float16*)malloc((size_t)args->seq_len * (size_t)args->hidden_dim * sizeof(_Float16));
    if (cpu_q_buf == NULL) { printf("Failed to allocate cpu_q_buf in attention\n"); exit(1); }

    rmsnorm_cpu(args->seq_len, args->hidden_dim, args->input, args->attn_norm_w, args->q_buf);
    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->hidden_dim, args->q_buf, args->Wq, cpu_q_buf, false);
    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->hidden_dim, args->q_buf, args->Wk, args->k_buf, false);
    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->hidden_dim, args->q_buf, args->Wv, args->v_buf, false);

    apply_rope_cpu(args->seq_len, args->hidden_dim, args->num_heads, args->freqs, cpu_q_buf, args->k_buf);

    for (int h = 0; h < args->num_heads; h++) {
        llama_matmul_cpu(args->seq_len, args->seq_len, head_dim,
                         cpu_q_buf + h * head_dim,
                         args->k_buf + h * head_dim,
                         args->attn_buf + (size_t)h * args->seq_len * args->seq_len,
                         true);
    }

    softmax_cpu(args->seq_len, args->seq_len * args->num_heads, 1.0f / sqrtf((float)head_dim), args->attn_buf);

    for (int h = 0; h < args->num_heads; h++) {
        llama_matmul_cpu(args->seq_len, head_dim, args->seq_len,
                         args->attn_buf + (size_t)h * args->seq_len * args->seq_len,
                         args->v_buf + h * head_dim,
                         args->out_buf + h * head_dim,
                         false);
    }

    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->hidden_dim, args->out_buf, args->Wo, args->out, false);
    llama_resadd_cpu(args->seq_len, args->hidden_dim, args->input, args->out, args->out);

    free(cpu_q_buf);
}

// ---- FFN ----

typedef struct {
    int hidden_dim, ffn_hidden_dim, seq_len;
    const _Float16* input;
    _Float16* out;
    const _Float16* W1;
    const _Float16* W2;
    const _Float16* W3;
    const _Float16* ffn_norm_w;
    _Float16* x1_buf;
    _Float16* x3_buf;
} FFNGoldenArgs;

void ffn_cpu_golden(const FFNGoldenArgs* args) {
    _Float16* norm_out_buf =
        (_Float16*)malloc((size_t)args->seq_len * (size_t)args->hidden_dim * sizeof(_Float16));
    if (norm_out_buf == NULL && args->seq_len * args->hidden_dim > 0) {
        printf("Failed to allocate norm_out_buf\n"); exit(1);
    }
    rmsnorm_cpu(args->seq_len, args->hidden_dim, args->input, args->ffn_norm_w, norm_out_buf);
    llama_matmul_cpu(args->seq_len, args->ffn_hidden_dim, args->hidden_dim, norm_out_buf, args->W1, args->x1_buf, false);
    llama_matmul_cpu(args->seq_len, args->ffn_hidden_dim, args->hidden_dim, norm_out_buf, args->W3, args->x3_buf, false);
    swiglu_cpu(args->seq_len, args->ffn_hidden_dim, args->x1_buf, args->x3_buf, args->x1_buf);
    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->ffn_hidden_dim, args->x1_buf, args->W2, args->out, false);
    llama_resadd_cpu(args->seq_len, args->hidden_dim, args->input, args->out, args->out);
    free(norm_out_buf);
}

//================================================================================
// Verification, Randomization, and Main Execution
//================================================================================

#define HIDDEN_DIM      128
#define FFN_HIDDEN_DIM  256
#define NUM_HEADS       1
#define SEQ_LEN         128

void verify_results(const char* name, const _Float16* gemmini_output,
                    const _Float16* cpu_output, size_t size) {
    int mismatches = 0;
    int mismatches_to_print = 10;

    for (size_t i = 0; i < size; i++) {
        float a = (float)gemmini_output[i];
        float b = (float)cpu_output[i];
        if (fabsf(a - b) > CPU_VERIFY_TOLERANCE) {
            mismatches++;
        }
    }

    if (mismatches == 0) {
        printf("%s: ✅ Results Verified Correctly!\n\n", name);
    } else {
        printf("%s: ❌ Verification FAILED! Mismatches: %d / %d\n", name, mismatches, (int)size);
        printf("--- Mismatch Details (up to %d) ---\n", mismatches_to_print);
        printf("Index | Gemmini Output     | Golden CPU Output   | Diff\n");
        printf("---------------------------------------------------------------\n");
        int printed = 0;
        for (size_t i = 0; i < size && printed < mismatches_to_print; i++) {
            float a = (float)gemmini_output[i];
            float b = (float)cpu_output[i];
            float d = fabsf(a - b);
            if (d > CPU_VERIFY_TOLERANCE) {
                printf("%5d | % .7f | % .7f | % .7f\n", (int)i, a, b, d);
                printed++;
            }
        }
        printf("---------------------------------------------------------------\n\n");
    }
}

void randomize__Float16_array(_Float16 *arr, size_t n) {
    for (size_t i = 0; i < n; i++) {
        float f = (float)( (rand() % 20 - 10) ) / 10.0f; // [-1.0, 1.0)
        arr[i] = (_Float16)f;
    }
}

/* Row-major CPU reference: C[MxN] = A[MxK] * B[KxN] */
static void gold_matmul_cpu(int M, int N, int K,
                       const _Float16 *A, const _Float16 *B, _Float16 *C)
{
    for (int m = 0; m < M; ++m) {
        for (int n = 0; n < N; ++n) {
            float acc = 0.0f;
            for (int k = 0; k < K; ++k) {
                acc += (float)A[m*K + k] * (float)B[k*N + n];
            }
            C[m*N + n] = (_Float16)acc;
        }
    }
}


static int verify(const char *name, const _Float16 *gold, const _Float16 *test,
                  int M, int N)
{
    int mismatches = 0;
    for (int i = 0; i < M*N; ++i) {
        float g = (float)gold[i];
        float t = (float)test[i];
        if (fabsf(g - t) > CPU_VERIFY_TOLERANCE) mismatches++;
    }
    if (mismatches == 0) {
        printf("%s: ✅ match (M=%d N=%d)\n", name, M, N);
    } else {
        printf("%s: ❌ mismatches = %d / %d (tol=%.3g)\n",
               name, mismatches, M*N, CPU_VERIFY_TOLERANCE);
    }
    return mismatches;
}

// Make the buffers static and aligned like gemmini-rocc-tests:
enum { M = 4, N = 4, K = 4 };
static row_align(1) _Float16 A[M*K];
static row_align(1) _Float16 B[K*N];
static row_align(1) _Float16 C_cpu[M*N];
static row_align(1) _Float16 C_gemmini[M*N];


int main(void)
{
    printf("Minimal FP16 GEMM test on: %s\n", CONFIG_BOARD_TARGET);

    /* Small, RTL-sim friendly sizes */
    enum { M = 4, N = 4, K = 4 };

    /* Static buffers (no malloc) */
    _Float16 A[M*K], B[K*N], C_cpu[M*N], C_gemmini[M*N];

    /* Deterministic, bounded inputs (no rand) */
    for (int i = 0; i < M*K; ++i) A[i] = (_Float16)((i % 7 - 3) / 4.0f);   // [-0.75, 1.0)
    for (int i = 0; i < K*N; ++i) B[i] = (_Float16)((i % 5 - 2) / 3.0f);   // [-0.66.., 1.0)

    /* Clear outputs */
    for (int i = 0; i < M*N; ++i) C_cpu[i] = 0.0f;
    for (int i = 0; i < M*N; ++i) C_gemmini[i] = 0.0f;

    /* CPU reference */
    gold_matmul_cpu(M, N, K, A, B, C_cpu);

#ifdef USE_GEMMINI
    /* Gemmini run */
    if (!rr_acquire_single(0, 0)) {
        printf("Gemmini acquire failed; skipping Gemmini run.\n");
    } else {
        uint64_t t0 = read_cycles();

        rr_set_opc(XCUSTOM_ACC, 0);
        gemmini_flush(0);

        
        // // This function runs a tiled matrix multiplication, with automatically
        // // calculated tiling factors
        // static void tiled_matmul_auto(size_t dim_I, size_t dim_J, size_t dim_K,
        //         const elem_t* A, const elem_t* B,
        //         const void * D, void * C,
        //         size_t stride_A, size_t stride_B, size_t stride_D, size_t stride_C,
        //         scale_t A_scale_factor, scale_t B_scale_factor, scale_acc_t D_scale_factor,
        //         int act, acc_scale_t scale, acc_scale_t bert_scale,
        //         bool repeating_bias,
        //         bool transpose_A, bool transpose_B,
        //         bool full_C, bool low_D,
        //         uint8_t weightA,
        //         enum tiled_matmul_type_t tiled_matmul_type) {

        /* Row-major strides: A lda=K, B ldb=N, C ldc/out_stride=N */
        tiled_matmul_auto(M, N, K,
                          (elem_t *) A, (elem_t *) B, NULL, (elem_t *) C_gemmini,
                          K, N, N, N,
                          MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
                          NO_ACTIVATION, ACC_SCALE_IDENTITY, 0,
                          false, false, false, false, false, 0, WS);

        rr_fence(0);
        uint64_t cycles = read_cycles() - t0;
        rr_release(0);

        printf("Gemmini cycles: %llu\n", (unsigned long long)cycles);

        /* Compare */
        (void)verify("CPU vs Gemmini", C_cpu, C_gemmini, M, N);
    }
#else
    printf("Gemmini disabled (build without USE_GEMMINI).\n");
#endif

#ifdef USE_GEMMINI
    printf("C_gemmini[0..3]: ");
    for (int i = 0; i < 4; ++i) printf("% .5f ", (float)C_gemmini[i]);
    printf("\n");
    printf("C_gemmini[4..7]: ");
    for (int i = 4; i < 8; ++i) printf("% .5f ", (float)C_gemmini[i]);
    printf("\n");
    printf("C_gemmini[8..11]: ");
    for (int i = 8; i < 12; ++i) printf("% .5f ", (float)C_gemmini[i]);
    printf("\n");
    printf("C_gemmini[12..15]: ");
    for (int i = 12; i < 16; ++i) printf("% .5f ", (float)C_gemmini[i]);
    printf("\n");
    printf("\n");
#endif


    /* Optionally dump a tiny slice for debug */
    printf("C_cpu[0..3]: ");
    for (int i = 0; i < 4; ++i) printf("% .5f ", (float)C_cpu[i]);
    printf("\n");
    printf("C_cpu[4..7]: ");  
    for (int i = 4; i < 8; ++i) printf("% .5f ", (float)C_cpu[i]);
    printf("\n");
    printf("C_cpu[8..11]: ");
    for (int i = 8; i < 12; ++i) printf("% .5f ", (float)C_cpu[i]);
    printf("\n");
    printf("C_cpu[12..15]: ");
    for (int i = 12; i < 16; ++i) printf("% .5f ", (float)C_cpu[i]);
    printf("\n");

    /* End sim */
    sys_reboot(SYS_REBOOT_COLD);
    return 0;
}

// int main(void)
// {
//     const char* name = "Llama2-small-FP16-compute";
//     const int hidden_dim = HIDDEN_DIM;
//     const int ffn_hidden_dim = FFN_HIDDEN_DIM;
//     const int num_heads = NUM_HEADS;
//     const int seq_len = SEQ_LEN;

//     printf("--- Running Benchmark (CPU default): %s on %s ---\n",
//            name, CONFIG_BOARD_TARGET);
//     printf("hidden_dim=%d, ffn_hidden_dim=%d, num_heads=%d, seq_len=%d\n",
//            hidden_dim, ffn_hidden_dim, num_heads, seq_len);

//     // ---- allocate all buffers ----
//     size_t hd = (size_t)hidden_dim, fhd = (size_t)ffn_hidden_dim, sl = (size_t)seq_len;
//     _Float16 *input =        (_Float16*)malloc(sl * hd * sizeof(_Float16));
//     _Float16 *output_golden =(_Float16*)malloc(sl * hd * sizeof(_Float16));
//     _Float16 *output_gemmini=(_Float16*)malloc(sl * hd * sizeof(_Float16));
//     _Float16 *Wq = (_Float16*)malloc(hd * hd * sizeof(_Float16));
//     _Float16 *Wk = (_Float16*)malloc(hd * hd * sizeof(_Float16));
//     _Float16 *Wv = (_Float16*)malloc(hd * hd * sizeof(_Float16));
//     _Float16 *Wo = (_Float16*)malloc(hd * hd * sizeof(_Float16));
//     _Float16 *attn_norm_w =  (_Float16*)malloc(hd * sizeof(_Float16));
//     _Float16 *W1 = (_Float16*)malloc(fhd * hd * sizeof(_Float16));
//     _Float16 *W2 = (_Float16*)malloc(hd * fhd * sizeof(_Float16));
//     _Float16 *W3 = (_Float16*)malloc(fhd * hd * sizeof(_Float16));
//     _Float16 *ffn_norm_w =   (_Float16*)malloc(hd * sizeof(_Float16));
//     _Float16 *freqs =        (_Float16*)malloc(sl * (hd / num_heads) * sizeof(_Float16));
//     _Float16 *attn_res_buf = (_Float16*)malloc(sl * hd * sizeof(_Float16));
//     _Float16 *q_buf =        (_Float16*)malloc(sl * hd * sizeof(_Float16));
//     _Float16 *k_buf =        (_Float16*)malloc(sl * hd * sizeof(_Float16));
//     _Float16 *v_buf =        (_Float16*)malloc(sl * hd * sizeof(_Float16));
//     _Float16 *attn_buf =     (_Float16*)malloc((size_t)num_heads * sl * sl * sizeof(_Float16));
//     _Float16 *attn_out_buf = (_Float16*)malloc(sl * hd * sizeof(_Float16));
//     _Float16 *ffn_x1_buf =   (_Float16*)malloc(sl * fhd * sizeof(_Float16));
//     _Float16 *ffn_x3_buf =   (_Float16*)malloc(sl * fhd * sizeof(_Float16));

//     if (!input || !output_golden || !Wq || !Wk || !Wv || !Wo || !attn_norm_w ||
//         !W1 || !W2 || !W3 || !ffn_norm_w || !freqs || !attn_res_buf ||
//         !q_buf || !k_buf || !v_buf || !attn_buf || !attn_out_buf ||
//         !ffn_x1_buf || !ffn_x3_buf) {
//         printf("Out of memory!\n");
//         goto out;
//     }

//     // ---- initialize data ----
//     // Deterministic seed (no timers): fixed seed is fine for reproducibility
//     printf("Randomizing input data...\n");
//     srand(1);
//     randomize__Float16_array(Wq, hd * hd);
//     randomize__Float16_array(Wk, hd * hd);
//     randomize__Float16_array(Wv, hd * hd);
//     randomize__Float16_array(Wo, hd * hd);
//     randomize__Float16_array(attn_norm_w, hd);
//     randomize__Float16_array(W1, fhd * hd);
//     randomize__Float16_array(W2, hd * fhd);
//     randomize__Float16_array(W3, fhd * hd);
//     randomize__Float16_array(ffn_norm_w, hd);
//     precompute_rope_freqs_cpu(hidden_dim / num_heads, seq_len, 10000.0f, freqs);
//     randomize__Float16_array(input, sl * hd);

//     // ---- CPU (golden) path ----
//     {
//         printf("--- Running CPU Golden Implemetation ---\n");
//         AttentionGoldenArgs attn_args = {
//             .hidden_dim = hidden_dim, .num_heads = num_heads, .seq_len = seq_len,
//             .freqs = freqs, .input = input, .out = attn_res_buf,
//             .Wq = Wq, .Wk = Wk, .Wv = Wv, .Wo = Wo,
//             .attn_norm_w = attn_norm_w,
//             .q_buf = q_buf, .k_buf = k_buf, .v_buf = v_buf,
//             .attn_buf = attn_buf, .out_buf = attn_out_buf
//         };
//         attention_cpu_golden(&attn_args);

//         FFNGoldenArgs ffn_args = {
//             .hidden_dim = hidden_dim, .ffn_hidden_dim = ffn_hidden_dim, .seq_len = seq_len,
//             .input = attn_res_buf, .out = output_golden,
//             .W1 = W1, .W2 = W2, .W3 = W3, .ffn_norm_w = ffn_norm_w,
//             .x1_buf = ffn_x1_buf, .x3_buf = ffn_x3_buf
//         };
//         ffn_cpu_golden(&ffn_args);
//         printf("--- CPU Golden Implementation completed ---\n");
//     }

// #ifdef USE_GEMMINI
//     // ---- Gemmini path (optional) ----
//     {
//         _Float16 *gemmini_q_buf = (_Float16*)malloc(sl * hd * sizeof(_Float16));
//         if (!gemmini_q_buf || !output_gemmini) {
//             printf("Out of memory (Gemmini buffers)\n");
//             goto verify_or_exit;
//         }

//         printf("--- Acquiring Gemmini ---\n");
//         if (!rr_acquire_single(0, 0)) { printf("Failed to acquire Gemmini!\n"); goto gemmini_cleanup; }
//         uint64_t start_cycles = read_cycles();

//         printf("--- Running Gemmini Implementation ---\n");

//         // Attention block
//         rmsnorm_cpu(seq_len, hidden_dim, input, attn_norm_w, q_buf);

//         rr_set_opc(XCUSTOM_ACC, 0);
//         gemmini_flush(0);
//         tiled_matmul_auto(seq_len, hidden_dim, hidden_dim,
//                           q_buf, Wq, NULL, gemmini_q_buf,
//                           hidden_dim, hidden_dim, hidden_dim, hidden_dim,
//                           (_Float16)1, (_Float16)1, (_Float16)1,
//                           NO_ACTIVATION, (_Float16)1, 0,
//                           false, false, false, false, false, 0, WS);

//         tiled_matmul_auto(seq_len, hidden_dim, hidden_dim,
//                           q_buf, Wk, NULL, k_buf,
//                           hidden_dim, hidden_dim, hidden_dim, hidden_dim,
//                           (_Float16)1, (_Float16)1, (_Float16)1,
//                           NO_ACTIVATION, (_Float16)1, 0,
//                           false, false, false, false, false, 0, WS);

//         tiled_matmul_auto(seq_len, hidden_dim, hidden_dim,
//                           q_buf, Wv, NULL, v_buf,
//                           hidden_dim, hidden_dim, hidden_dim, hidden_dim,
//                           (_Float16)1, (_Float16)1, (_Float16)1,
//                           NO_ACTIVATION, (_Float16)1, 0,
//                           false, false, false, false, false, 0, WS);

//         rr_fence(0);
//         apply_rope_cpu(seq_len, hidden_dim, num_heads, freqs, gemmini_q_buf, k_buf);

//         rr_set_opc(XCUSTOM_ACC, 0);
//         gemmini_flush(0);
//         for (int h = 0; h < num_heads; h++) {
//             int hd_per = hidden_dim / num_heads;
//             tiled_matmul_auto(seq_len, seq_len, hd_per,
//                               gemmini_q_buf + h * hd_per,
//                               k_buf + h * hd_per, NULL,
//                               attn_buf + (size_t)h * seq_len * seq_len,
//                               hd_per, seq_len, seq_len, seq_len,
//                               (_Float16)1, (_Float16)1, (_Float16)1,
//                               NO_ACTIVATION, (_Float16)1, 0,
//                               false, false, true, false, false, 0, WS);
//         }
//         rr_fence(0);
//         softmax_cpu(seq_len, seq_len * num_heads,
//                     1.0f / sqrtf((float)(hidden_dim / num_heads)), attn_buf);

//         rr_set_opc(XCUSTOM_ACC, 0);
//         gemmini_flush(0);
//         for (int h = 0; h < num_heads; h++) {
//             int hd_per = hidden_dim / num_heads;
//             tiled_matmul_auto(seq_len, hd_per, seq_len,
//                               attn_buf + (size_t)h * seq_len * seq_len,
//                               v_buf + h * hd_per, NULL,
//                               attn_out_buf + h * hd_per,
//                               seq_len, hd_per, hd_per, hd_per,
//                               (_Float16)1, (_Float16)1, (_Float16)1,
//                               NO_ACTIVATION, (_Float16)1, 0,
//                               false, false, false, false, false, 0, WS);
//         }
//         rr_fence(0);

//         rr_set_opc(XCUSTOM_ACC, 0);
//         gemmini_flush(0);
//         tiled_matmul_auto(seq_len, hidden_dim, hidden_dim,
//                           attn_out_buf, Wo, NULL, attn_res_buf,
//                           hidden_dim, hidden_dim, hidden_dim, hidden_dim,
//                           (_Float16)1, (_Float16)1, (_Float16)1,
//                           NO_ACTIVATION, (_Float16)1, 0,
//                           false, false, false, false, false, 0, WS);
//         rr_fence(0);
//         llama_resadd_cpu(seq_len, hidden_dim, input, attn_res_buf, attn_res_buf);

//         // FFN block
//         rmsnorm_cpu(seq_len, hidden_dim, attn_res_buf, ffn_norm_w, output_gemmini);

//         rr_set_opc(XCUSTOM_ACC, 0);
//         gemmini_flush(0);
//         tiled_matmul_auto(seq_len, ffn_hidden_dim, hidden_dim,
//                           output_gemmini, W1, NULL, ffn_x1_buf,
//                           hidden_dim, ffn_hidden_dim, ffn_hidden_dim, ffn_hidden_dim,
//                           (_Float16)1, (_Float16)1, (_Float16)1,
//                           NO_ACTIVATION, (_Float16)1, 0,
//                           false, false, false, false, false, 0, WS);

//         tiled_matmul_auto(seq_len, ffn_hidden_dim, hidden_dim,
//                           output_gemmini, W3, NULL, ffn_x3_buf,
//                           hidden_dim, ffn_hidden_dim, ffn_hidden_dim, ffn_hidden_dim,
//                           (_Float16)1, (_Float16)1, (_Float16)1,
//                           NO_ACTIVATION, (_Float16)1, 0,
//                           false, false, false, false, false, 0, WS);

//         rr_fence(0);
//         swiglu_cpu(seq_len, ffn_hidden_dim, ffn_x1_buf, ffn_x3_buf, ffn_x1_buf);

//         rr_set_opc(XCUSTOM_ACC, 0);
//         gemmini_flush(0);
//         tiled_matmul_auto(seq_len, hidden_dim, ffn_hidden_dim,
//                           ffn_x1_buf, W2, NULL, output_gemmini,
//                           ffn_hidden_dim, hidden_dim, hidden_dim, hidden_dim,
//                           (_Float16)1, (_Float16)1, (_Float16)1,
//                           NO_ACTIVATION, (_Float16)1, 0,
//                           false, false, false, false, false, 0, WS);
//         rr_fence(0);
//         llama_resadd_cpu(seq_len, hidden_dim, attn_res_buf, output_gemmini, output_gemmini);

//         uint64_t total_cycles = read_cycles() - start_cycles;
//         rr_release(0);
//         printf("%s Gemmini cycles: %lld\n", name, total_cycles);

// gemmini_cleanup:
//         free(gemmini_q_buf);
//     }
// verify_or_exit:
//     // Compare Gemmini vs CPU only if Gemmini ran
//     if (output_gemmini) {
//         verify_results(name, output_gemmini, output_golden, sl * hd);
//     }
// #else
//     printf("(Gemmini disabled: build without USE_GEMMINI)\n");
// #endif

// out:
//     // ---- free buffers (best-effort) ----
//     free(input); free(output_golden); free(output_gemmini);
//     free(Wq); free(Wk); free(Wv); free(Wo);
//     free(attn_norm_w); free(W1); free(W2); free(W3); free(ffn_norm_w);
//     free(freqs); free(attn_res_buf); free(q_buf); free(k_buf); free(v_buf);
//     free(attn_buf); free(attn_out_buf); free(ffn_x1_buf); free(ffn_x3_buf);

//     // End the sim
//     sys_reboot(SYS_REBOOT_COLD);
//     return 0;
// }

