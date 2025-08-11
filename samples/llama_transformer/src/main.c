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

void rmsnorm_cpu(int M, int N, const elem_t* x, const elem_t* weight, elem_t* out) {
    for (int m = 0; m < M; m++) {
        const elem_t* x_row = x + m * N;
        elem_t* out_row = out + m * N;
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

void precompute_rope_freqs_cpu(int dim, int seq_len, float theta, elem_t* freqs) {
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
                    const elem_t* freqs, elem_t* q, elem_t* k) {
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

void swiglu_cpu(int M, int N, const elem_t* x1, const elem_t* x3, elem_t* out) {
    for (int i = 0; i < M * N; i++) {
        float v1 = (float)x1[i];
        float v3 = (float)x3[i];
        float swish = v1 * (1.0f / (1.0f + expf(-v1)));
        out[i] = (_Float16)(swish * v3);
    }
}

void llama_matmul_cpu(int M, int N, int K, const elem_t* A, const elem_t* B, elem_t* C, bool transB) {
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

void softmax_cpu(int M, int N, float scale, elem_t* C) {
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

void llama_resadd_cpu(int M, int N, const elem_t* A, const elem_t* B, elem_t* C) {
    for (int i = 0; i < M * N; i++) {
        C[i] = (_Float16)((float)A[i] + (float)B[i]);
    }
}

// ---- Attention ----

typedef struct {
    int hidden_dim, num_heads, seq_len;
    const elem_t* freqs;
    const elem_t* input;
    elem_t* out;
    const elem_t* Wq;
    const elem_t* Wk;
    const elem_t* Wv;
    const elem_t* Wo;
    const elem_t* attn_norm_w;
    elem_t* q_buf;
    elem_t* k_buf;
    elem_t* v_buf;
    elem_t* attn_buf;
    elem_t* out_buf;
} AttentionGoldenArgs;

void attention_cpu_golden(const AttentionGoldenArgs* args) {
    int head_dim = args->hidden_dim / args->num_heads;

    elem_t* cpu_q_buf =
        (elem_t*)malloc((size_t)args->seq_len * (size_t)args->hidden_dim * sizeof(elem_t));
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
    const elem_t* input;
    elem_t* out;
    const elem_t* W1;
    const elem_t* W2;
    const elem_t* W3;
    const elem_t* ffn_norm_w;
    elem_t* x1_buf;
    elem_t* x3_buf;
} FFNGoldenArgs;

void ffn_cpu_golden(const FFNGoldenArgs* args) {
    elem_t* norm_out_buf =
        (elem_t*)malloc((size_t)args->seq_len * (size_t)args->hidden_dim * sizeof(elem_t));
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

void verify_results(const char* name, const elem_t* gemmini_output,
                    const elem_t* cpu_output, size_t size) {
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

void randomize_elem_t_array(elem_t *arr, size_t n) {
    for (size_t i = 0; i < n; i++) {
        float f = (float)( (rand() % 20 - 10) ) / 10.0f; // [-1.0, 1.0)
        arr[i] = (_Float16)f;
    }
}

int main(void)
{
    const char* name = "Llama2-small-FP16-compute";
    const int hidden_dim = HIDDEN_DIM;
    const int ffn_hidden_dim = FFN_HIDDEN_DIM;
    const int num_heads = NUM_HEADS;
    const int seq_len = SEQ_LEN;

    printf("--- Running Benchmark (CPU default): %s on %s ---\n",
           name, CONFIG_BOARD_TARGET);
    printf("hidden_dim=%d, ffn_hidden_dim=%d, num_heads=%d, seq_len=%d\n",
           hidden_dim, ffn_hidden_dim, num_heads, seq_len);

    // ---- allocate all buffers ----
    size_t hd = (size_t)hidden_dim, fhd = (size_t)ffn_hidden_dim, sl = (size_t)seq_len;
    elem_t *input =        (elem_t*)malloc(sl * hd * sizeof(elem_t));
    elem_t *output_golden =(elem_t*)malloc(sl * hd * sizeof(elem_t));
    elem_t *output_gemmini=(elem_t*)malloc(sl * hd * sizeof(elem_t));
    elem_t *Wq = (elem_t*)malloc(hd * hd * sizeof(elem_t));
    elem_t *Wk = (elem_t*)malloc(hd * hd * sizeof(elem_t));
    elem_t *Wv = (elem_t*)malloc(hd * hd * sizeof(elem_t));
    elem_t *Wo = (elem_t*)malloc(hd * hd * sizeof(elem_t));
    elem_t *attn_norm_w =  (elem_t*)malloc(hd * sizeof(elem_t));
    elem_t *W1 = (elem_t*)malloc(fhd * hd * sizeof(elem_t));
    elem_t *W2 = (elem_t*)malloc(hd * fhd * sizeof(elem_t));
    elem_t *W3 = (elem_t*)malloc(fhd * hd * sizeof(elem_t));
    elem_t *ffn_norm_w =   (elem_t*)malloc(hd * sizeof(elem_t));
    elem_t *freqs =        (elem_t*)malloc(sl * (hd / num_heads) * sizeof(elem_t));
    elem_t *attn_res_buf = (elem_t*)malloc(sl * hd * sizeof(elem_t));
    elem_t *q_buf =        (elem_t*)malloc(sl * hd * sizeof(elem_t));
    elem_t *k_buf =        (elem_t*)malloc(sl * hd * sizeof(elem_t));
    elem_t *v_buf =        (elem_t*)malloc(sl * hd * sizeof(elem_t));
    elem_t *attn_buf =     (elem_t*)malloc((size_t)num_heads * sl * sl * sizeof(elem_t));
    elem_t *attn_out_buf = (elem_t*)malloc(sl * hd * sizeof(elem_t));
    elem_t *ffn_x1_buf =   (elem_t*)malloc(sl * fhd * sizeof(elem_t));
    elem_t *ffn_x3_buf =   (elem_t*)malloc(sl * fhd * sizeof(elem_t));

    if (!input || !output_golden || !Wq || !Wk || !Wv || !Wo || !attn_norm_w ||
        !W1 || !W2 || !W3 || !ffn_norm_w || !freqs || !attn_res_buf ||
        !q_buf || !k_buf || !v_buf || !attn_buf || !attn_out_buf ||
        !ffn_x1_buf || !ffn_x3_buf) {
        printf("Out of memory!\n");
        goto out;
    }

    // ---- initialize data ----
    // Deterministic seed (no timers): fixed seed is fine for reproducibility
    printf("Randomizing input data...\n");
    srand(1);
    randomize_elem_t_array(Wq, hd * hd);
    randomize_elem_t_array(Wk, hd * hd);
    randomize_elem_t_array(Wv, hd * hd);
    randomize_elem_t_array(Wo, hd * hd);
    randomize_elem_t_array(attn_norm_w, hd);
    randomize_elem_t_array(W1, fhd * hd);
    randomize_elem_t_array(W2, hd * fhd);
    randomize_elem_t_array(W3, fhd * hd);
    randomize_elem_t_array(ffn_norm_w, hd);
    precompute_rope_freqs_cpu(hidden_dim / num_heads, seq_len, 10000.0f, freqs);
    randomize_elem_t_array(input, sl * hd);

    // ---- CPU (golden) path ----
    {
        printf("--- Running CPU Golden Implemetation ---\n");
        AttentionGoldenArgs attn_args = {
            .hidden_dim = hidden_dim, .num_heads = num_heads, .seq_len = seq_len,
            .freqs = freqs, .input = input, .out = attn_res_buf,
            .Wq = Wq, .Wk = Wk, .Wv = Wv, .Wo = Wo,
            .attn_norm_w = attn_norm_w,
            .q_buf = q_buf, .k_buf = k_buf, .v_buf = v_buf,
            .attn_buf = attn_buf, .out_buf = attn_out_buf
        };
        attention_cpu_golden(&attn_args);

        FFNGoldenArgs ffn_args = {
            .hidden_dim = hidden_dim, .ffn_hidden_dim = ffn_hidden_dim, .seq_len = seq_len,
            .input = attn_res_buf, .out = output_golden,
            .W1 = W1, .W2 = W2, .W3 = W3, .ffn_norm_w = ffn_norm_w,
            .x1_buf = ffn_x1_buf, .x3_buf = ffn_x3_buf
        };
        ffn_cpu_golden(&ffn_args);
        printf("--- CPU Golden Implementation completed ---\n");
    }

#ifdef USE_GEMMINI
    // ---- Gemmini path (optional) ----
    {
        elem_t *gemmini_q_buf = (elem_t*)malloc(sl * hd * sizeof(elem_t));
        if (!gemmini_q_buf || !output_gemmini) {
            printf("Out of memory (Gemmini buffers)\n");
            goto verify_or_exit;
        }

        printf("--- Acquiring Gemmini ---\n");
        if (!rr_acquire_single(0, 0)) { printf("Failed to acquire Gemmini!\n"); goto gemmini_cleanup; }
        uint64_t start_cycles = read_cycles();

        printf("--- Running Gemmini Implementation ---\n");

        // Attention block
        rmsnorm_cpu(seq_len, hidden_dim, input, attn_norm_w, q_buf);

        rr_set_opc(XCUSTOM_ACC, 0);
        gemmini_flush(0);
        tiled_matmul_auto(seq_len, hidden_dim, hidden_dim,
                          q_buf, Wq, NULL, gemmini_q_buf,
                          hidden_dim, hidden_dim, hidden_dim, hidden_dim,
                          (_Float16)1, (_Float16)1, (_Float16)1,
                          NO_ACTIVATION, (_Float16)1, 0,
                          false, false, false, false, false, 0, WS);

        tiled_matmul_auto(seq_len, hidden_dim, hidden_dim,
                          q_buf, Wk, NULL, k_buf,
                          hidden_dim, hidden_dim, hidden_dim, hidden_dim,
                          (_Float16)1, (_Float16)1, (_Float16)1,
                          NO_ACTIVATION, (_Float16)1, 0,
                          false, false, false, false, false, 0, WS);

        tiled_matmul_auto(seq_len, hidden_dim, hidden_dim,
                          q_buf, Wv, NULL, v_buf,
                          hidden_dim, hidden_dim, hidden_dim, hidden_dim,
                          (_Float16)1, (_Float16)1, (_Float16)1,
                          NO_ACTIVATION, (_Float16)1, 0,
                          false, false, false, false, false, 0, WS);

        rr_fence(0);
        apply_rope_cpu(seq_len, hidden_dim, num_heads, freqs, gemmini_q_buf, k_buf);

        rr_set_opc(XCUSTOM_ACC, 0);
        gemmini_flush(0);
        for (int h = 0; h < num_heads; h++) {
            int hd_per = hidden_dim / num_heads;
            tiled_matmul_auto(seq_len, seq_len, hd_per,
                              gemmini_q_buf + h * hd_per,
                              k_buf + h * hd_per, NULL,
                              attn_buf + (size_t)h * seq_len * seq_len,
                              hd_per, seq_len, seq_len, seq_len,
                              (_Float16)1, (_Float16)1, (_Float16)1,
                              NO_ACTIVATION, (_Float16)1, 0,
                              false, false, true, false, false, 0, WS);
        }
        rr_fence(0);
        softmax_cpu(seq_len, seq_len * num_heads,
                    1.0f / sqrtf((float)(hidden_dim / num_heads)), attn_buf);

        rr_set_opc(XCUSTOM_ACC, 0);
        gemmini_flush(0);
        for (int h = 0; h < num_heads; h++) {
            int hd_per = hidden_dim / num_heads;
            tiled_matmul_auto(seq_len, hd_per, seq_len,
                              attn_buf + (size_t)h * seq_len * seq_len,
                              v_buf + h * hd_per, NULL,
                              attn_out_buf + h * hd_per,
                              seq_len, hd_per, hd_per, hd_per,
                              (_Float16)1, (_Float16)1, (_Float16)1,
                              NO_ACTIVATION, (_Float16)1, 0,
                              false, false, false, false, false, 0, WS);
        }
        rr_fence(0);

        rr_set_opc(XCUSTOM_ACC, 0);
        gemmini_flush(0);
        tiled_matmul_auto(seq_len, hidden_dim, hidden_dim,
                          attn_out_buf, Wo, NULL, attn_res_buf,
                          hidden_dim, hidden_dim, hidden_dim, hidden_dim,
                          (_Float16)1, (_Float16)1, (_Float16)1,
                          NO_ACTIVATION, (_Float16)1, 0,
                          false, false, false, false, false, 0, WS);
        rr_fence(0);
        llama_resadd_cpu(seq_len, hidden_dim, input, attn_res_buf, attn_res_buf);

        // FFN block
        rmsnorm_cpu(seq_len, hidden_dim, attn_res_buf, ffn_norm_w, output_gemmini);

        rr_set_opc(XCUSTOM_ACC, 0);
        gemmini_flush(0);
        tiled_matmul_auto(seq_len, ffn_hidden_dim, hidden_dim,
                          output_gemmini, W1, NULL, ffn_x1_buf,
                          hidden_dim, ffn_hidden_dim, ffn_hidden_dim, ffn_hidden_dim,
                          (_Float16)1, (_Float16)1, (_Float16)1,
                          NO_ACTIVATION, (_Float16)1, 0,
                          false, false, false, false, false, 0, WS);

        tiled_matmul_auto(seq_len, ffn_hidden_dim, hidden_dim,
                          output_gemmini, W3, NULL, ffn_x3_buf,
                          hidden_dim, ffn_hidden_dim, ffn_hidden_dim, ffn_hidden_dim,
                          (_Float16)1, (_Float16)1, (_Float16)1,
                          NO_ACTIVATION, (_Float16)1, 0,
                          false, false, false, false, false, 0, WS);

        rr_fence(0);
        swiglu_cpu(seq_len, ffn_hidden_dim, ffn_x1_buf, ffn_x3_buf, ffn_x1_buf);

        rr_set_opc(XCUSTOM_ACC, 0);
        gemmini_flush(0);
        tiled_matmul_auto(seq_len, hidden_dim, ffn_hidden_dim,
                          ffn_x1_buf, W2, NULL, output_gemmini,
                          ffn_hidden_dim, hidden_dim, hidden_dim, hidden_dim,
                          (_Float16)1, (_Float16)1, (_Float16)1,
                          NO_ACTIVATION, (_Float16)1, 0,
                          false, false, false, false, false, 0, WS);
        rr_fence(0);
        llama_resadd_cpu(seq_len, hidden_dim, attn_res_buf, output_gemmini, output_gemmini);

        uint64_t total_cycles = read_cycles() - start_cycles;
        rr_release(0);
        printf("%s Gemmini cycles: %lld\n", name, total_cycles);

gemmini_cleanup:
        free(gemmini_q_buf);
    }
verify_or_exit:
    // Compare Gemmini vs CPU only if Gemmini ran
    if (output_gemmini) {
        verify_results(name, output_gemmini, output_golden, sl * hd);
    }
#else
    printf("(Gemmini disabled: build without USE_GEMMINI)\n");
#endif

out:
    // ---- free buffers (best-effort) ----
    free(input); free(output_golden); free(output_gemmini);
    free(Wq); free(Wk); free(Wv); free(Wo);
    free(attn_norm_w); free(W1); free(W2); free(W3); free(ffn_norm_w);
    free(freqs); free(attn_res_buf); free(q_buf); free(k_buf); free(v_buf);
    free(attn_buf); free(attn_out_buf); free(ffn_x1_buf); free(ffn_x3_buf);

    // End the sim
    sys_reboot(SYS_REBOOT_COLD);
    return 0;
}

