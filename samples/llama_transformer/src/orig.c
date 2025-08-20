#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <float.h>
#include <stdint.h> // For uint16_t

#ifndef BAREMETAL
// #include <sys/mman.h>
#endif
#include "gemmini.h"
// #include "include/gemmini_nn.h"
#include "gemmini_testutils.h" // For rr_* functions

// Define the primary data type as uint16_t, representing FP16 data.
typedef uint16_t elem_t;
// Define the new float16_t type for compatibility with the provided functions
typedef uint16_t float16_t;
typedef uint16_t acc_t;

#define CPU_VERIFY_TOLERANCE 1e-1 // Adjusted tolerance for FP comparisons

#define ROW_ALIGN __attribute__((aligned(64)))

// Define the size of our static memory pool (adjust as needed)
#define MEMORY_POOL_SIZE (1024 * 1024 * 128)  // 128MB static memory pool

// Static memory pool
static unsigned char memory_pool[MEMORY_POOL_SIZE] __attribute__((aligned(8)));
static size_t current_offset = 0;

// Redirect standard malloc to our custom implementation
#define malloc(x) malloc_(x)
#define calloc(x,y) calloc_(x,y)

//================================================================================
// Custom Memory Allocator
//================================================================================

void* malloc_(size_t size) {
    size = (size + 7) & ~7;
    if (current_offset + size > MEMORY_POOL_SIZE) return NULL;
    void* ptr = &memory_pool[current_offset];
    current_offset += size;
    return ptr;
}

void* calloc_(size_t nmemb, size_t size) {
    size_t total_size = nmemb * size;
    if (size != 0 && total_size / size != nmemb) return NULL;
    void* ptr = malloc_(total_size);
    if (ptr == NULL) return NULL;
    memset(ptr, 0, total_size);
    return ptr;
}

static void printFPArray(elem_t m[DIM]) {
    for (size_t j = 0; j < DIM; ++j) {
        NN_printFloat(NN_halfToFloat((float16_t) (m[j])), 5);
        printf(" ");
    }
    printf("\n");
}

//================================================================================
// Type Casting Helpers (FP32 <-> FP16)
//================================================================================

// typedef union {
//   uint32_t i;
//   float    f;
// } float_uint32_union_t;

// // from https://github.com/AcademySoftwareFoundation/Imath/blob/main/src/Imath/half.h

// static inline float NN_halfToFloat(float16_t h) {
//   float_uint32_union_t v;
//   uint32_t hexpmant = ((uint32_t) (h) << 17) >> 4;
//   v.i               = ((uint32_t) (h >> 15)) << 31;

//   if ((hexpmant >= 0x00800000)) {
//     v.i |= hexpmant;
//     if ((hexpmant < 0x0f800000)) {
//       v.i += 0x38000000;
//     }
//     else {
//       v.i |= 0x7f800000;
//     }
//   }
//   else if (hexpmant != 0) {
//     uint32_t lc;
//     lc = 0;
//     while (0 == ((hexpmant << lc) & 0x80000000)) {
//       lc += 1;
//     }
//     lc -= 8;
//     v.i |= 0x38800000;
//     v.i |= (hexpmant << lc);
//     v.i -= (lc << 23);
//   }
//   return v.f;
// }

// static inline float16_t NN_floatToHalf(float f) {
//   float_uint32_union_t  v;
//   float16_t ret;
//   uint32_t e, m, ui, r, shift;

//   v.f = f;

//   ui  = (v.i & ~0x80000000);
//   ret = ((v.i >> 16) & 0x8000);

//   if (ui >= 0x38800000) {
//     if (ui >= 0x7f800000) {
//       ret |= 0x7c00;
//       if (ui == 0x7f800000) {
//         return ret;
//       }
//       m = (ui & 0x7fffff) >> 13;
//       return ret | (uint16_t) m | (uint16_t) (m == 0);
//     }

//     if (ui > 0x477fefff) {
//       return ret | 0x7c00;
//     }

//     ui -= 0x38000000;
//     ui = ((ui + 0x00000fff + ((ui >> 13) & 1)) >> 13);
//     return ret | (uint16_t) ui;
//   }

//   if (ui < 0x33000001) {
//     return ret;
//   }

//   e      = (ui >> 23);
//   shift = 0x7e - e;
//   m      = 0x800000 | (ui & 0x7fffff);
//   r      = m << (32 - shift);
//   ret  |= (m >> shift);
//   if (r > 0x80000000 || (r == 0x80000000 && (ret & 0x1) != 0)) {
//     ret += 1;
//   }
//   return ret;
// }


//================================================================================
// CPU "Golden" Scalar Implementation (operates on uint16_t, computes with float)
//================================================================================

void rmsnorm_cpu(int M, int N, const elem_t* x, const elem_t* weight, elem_t* out) {
    for (int m = 0; m < M; m++) {
        const elem_t* x_row = x + m * N;
        elem_t* out_row = out + m * N;
        float ss = 0.0f;
        for (int n = 0; n < N; n++) {
            float val = NN_halfToFloat(x_row[n]);
            ss += val * val;
        }
        ss /= N;
        ss += 1e-5f;
        ss = 1.0f / sqrtf(ss);
        for (int n = 0; n < N; n++) {
            float w = NN_halfToFloat(weight[n]);
            float val = NN_halfToFloat(x_row[n]);
            out_row[n] = NN_floatToHalf(w * (val * ss));
        }
    }
}

void precompute_rope_freqs_cpu(int dim, int seq_len, float theta, elem_t* freqs) {
    for (int i = 0; i < dim; i += 2) {
        float val = 1.0f / powf(theta, (float)i / (float)dim);
        for (int j = 0; j < seq_len; j++) {
            freqs[(j * dim) + i] = NN_floatToHalf(cosf(j * val));
            freqs[(j * dim) + i + 1] = NN_floatToHalf(sinf(j * val));
        }
    }
}

void apply_rope_cpu(int seq_len, int hidden_dim, int num_heads, const elem_t* freqs, elem_t* q, elem_t* k) {
    int head_dim = hidden_dim / num_heads;
    for (int h = 0; h < num_heads; h++) {
        for (int s = 0; s < seq_len; s++) {
            for (int d = 0; d < head_dim; d += 2) {
                int q_idx = (s * hidden_dim) + (h * head_dim) + d;
                int k_idx = (s * hidden_dim) + (h * head_dim) + d;
                int f_idx = (s * head_dim) + d;
                float q0 = NN_halfToFloat(q[q_idx]);
                float q1 = NN_halfToFloat(q[q_idx + 1]);
                float k0 = NN_halfToFloat(k[k_idx]);
                float k1 = NN_halfToFloat(k[k_idx + 1]);
                float f_cos = NN_halfToFloat(freqs[f_idx]);
                float f_sin = NN_halfToFloat(freqs[f_idx + 1]);
                q[q_idx] = NN_floatToHalf(q0 * f_cos - q1 * f_sin);
                q[q_idx + 1] = NN_floatToHalf(q0 * f_sin + q1 * f_cos);
                k[k_idx] = NN_floatToHalf(k0 * f_cos - k1 * f_sin);
                k[k_idx + 1] = NN_floatToHalf(k0 * f_sin + k1 * f_cos);
            }
        }
    }
}

void swiglu_cpu(int M, int N, const elem_t* x1, const elem_t* x3, elem_t* out) {
    for (int i = 0; i < M * N; i++) {
        float val1 = NN_halfToFloat(x1[i]);
        float val3 = NN_halfToFloat(x3[i]);
        float swish = val1 * (1.0f / (1.0f + expf(-val1)));
        out[i] = NN_floatToHalf(swish * val3);
    }
}

void llama_matmul_cpu(int M, int N, int K, const elem_t* A, const elem_t* B, elem_t* C, bool transB) {
    for (int m = 0; m < M; m++) {
        for (int n = 0; n < N; n++) {
            float result = 0.0f;
            for (int k = 0; k < K; k++) {
                float a_val = NN_halfToFloat(A[m * K + k]);
                float b_val = NN_halfToFloat(transB ? B[n * K + k] : B[k * N + n]);
                result += a_val * b_val;
            }
            C[m * N + n] = NN_floatToHalf(result);
        }
    }
}

void softmax_cpu(int M, int N, float scale, elem_t* C) {
    for (int m = 0; m < M; m++) {
        float max_val = -FLT_MAX;
        for (int n = 0; n < N; n++) {
            float val = NN_halfToFloat(C[m * N + n]);
            if (val > max_val) max_val = val;
        }
        float sum = 0.0f;
        for (int n = 0; n < N; n++) {
            float val = NN_halfToFloat(C[m * N + n]);
            sum += expf((val - max_val) * scale);
        }
        for (int n = 0; n < N; n++) {
            float val = NN_halfToFloat(C[m * N + n]);
            float exp_val = expf((val - max_val) * scale);
            C[m * N + n] = NN_floatToHalf(exp_val / sum);
        }
    }
}

void llama_resadd_cpu(int M, int N, const elem_t* A, const elem_t* B, elem_t* C) {
    for (int i = 0; i < M * N; i++) {
        float a_val = NN_halfToFloat(A[i]);
        float b_val = NN_halfToFloat(B[i]);
        C[i] = NN_floatToHalf(a_val + b_val);
    }
}

// Struct to hold arguments for attention_cpu_golden to avoid stack overflow
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
    
    // Use a temporary buffer to store the q output
    elem_t* cpu_q_buf = (elem_t*)malloc(args->seq_len * args->hidden_dim * sizeof(elem_t));
    if (cpu_q_buf == NULL) { printf("Failed to allocate cpu_q_buf in attention\n"); exit(1); }

    rmsnorm_cpu(args->seq_len, args->hidden_dim, args->input, args->attn_norm_w, args->q_buf);
    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->hidden_dim, args->q_buf, args->Wq, cpu_q_buf, false);
    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->hidden_dim, args->q_buf, args->Wk, args->k_buf, false);
    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->hidden_dim, args->q_buf, args->Wv, args->v_buf, false);
    apply_rope_cpu(args->seq_len, args->hidden_dim, args->num_heads, args->freqs, cpu_q_buf, args->k_buf);
    for (int h = 0; h < args->num_heads; h++) {
        llama_matmul_cpu(args->seq_len, args->seq_len, head_dim, cpu_q_buf + h * head_dim, args->k_buf + h * head_dim, args->attn_buf + h * args->seq_len * args->seq_len, true);
    }
    softmax_cpu(args->seq_len, args->seq_len * args->num_heads, 1.0f / sqrtf(head_dim), args->attn_buf);
    for (int h = 0; h < args->num_heads; h++) {
        llama_matmul_cpu(args->seq_len, head_dim, args->seq_len, args->attn_buf + h * args->seq_len * args->seq_len, args->v_buf + h * head_dim, args->out_buf + h * head_dim, false);
    }
    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->hidden_dim, args->out_buf, args->Wo, args->out, false);
    llama_resadd_cpu(args->seq_len, args->hidden_dim, args->input, args->out, args->out);
}

// Struct to hold arguments for ffn_cpu_golden
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
    elem_t* norm_out_buf = (elem_t*)malloc(args->seq_len * args->hidden_dim * sizeof(elem_t));
    if (norm_out_buf == NULL && args->seq_len * args->hidden_dim > 0) { printf("Failed to allocate norm_out_buf\n"); exit(1); }
    rmsnorm_cpu(args->seq_len, args->hidden_dim, args->input, args->ffn_norm_w, norm_out_buf);
    llama_matmul_cpu(args->seq_len, args->ffn_hidden_dim, args->hidden_dim, norm_out_buf, args->W1, args->x1_buf, false);
    llama_matmul_cpu(args->seq_len, args->ffn_hidden_dim, args->hidden_dim, norm_out_buf, args->W3, args->x3_buf, false);
    swiglu_cpu(args->seq_len, args->ffn_hidden_dim, args->x1_buf, args->x3_buf, args->x1_buf);
    llama_matmul_cpu(args->seq_len, args->hidden_dim, args->ffn_hidden_dim, args->x1_buf, args->W2, args->out, false);
    llama_resadd_cpu(args->seq_len, args->hidden_dim, args->input, args->out, args->out);
}

//================================================================================
// Verification, Randomization, and Main Execution
//================================================================================
#define HIDDEN_DIM 128
#define FFN_HIDDEN_DIM 256
#define NUM_HEADS 1
#define SEQ_LEN 128

void verify_results(const char* name, const elem_t* gemmini_output, const elem_t* cpu_output, size_t size) {
    int mismatches = 0;
    int mismatches_to_print = 10; // Limit the number of printed mismatches

    for (size_t i = 0; i < size; i++) {
        if (fabsf(NN_halfToFloat(gemmini_output[i]) - NN_halfToFloat(cpu_output[i])) > CPU_VERIFY_TOLERANCE) {
            mismatches++;
        }
    }

    // printf("Gemmini\n");
    // printFPArray(gemmini_output);
    // printf("CPU\n");
    // printFPArray(cpu_output); 

    if (mismatches == 0) {
        printf("%s: ✅ Results Verified Correctly!\n\n", name);
    } else {
        printf("%s: ❌ Verification FAILED! Mismatches: %d / %d\n", name, mismatches, size);
        printf("--- Mismatch Details (showing up to %d) ---\n", mismatches_to_print);
        printf("Index | Gemmini Output | Golden CPU Output | Difference\n");
        printf("--------------------------------------------------------\n");
        int printed_count = 0;
        for (size_t i = 0; i < size && printed_count < mismatches_to_print; i++) {
            float gemmini_f = NN_halfToFloat(gemmini_output[i]);
            float cpu_f = NN_halfToFloat(cpu_output[i]);
            float diff = fabsf(gemmini_f - cpu_f);
            if (diff > CPU_VERIFY_TOLERANCE) {
                printf("%d | %d | %d | %d\n",
                       i, (int)gemmini_f, (int)cpu_f, (int)diff);
                printed_count++;
            }
        }
        printf("--------------------------------------------------------\n\n");
    }
}

void randomize_elem_t_array(elem_t * arr, size_t n) {
    for (size_t i = 0; i < n; i++) arr[i] = NN_floatToHalf(((rand() % 20 - 10) / 10.0f)); //NN_floatToHalf(1);
}

int main (int argc, char * argv[]) {
    srand(time(NULL));

    const char* name = "Llama2-small-FP16-compute";
    int hidden_dim = HIDDEN_DIM, ffn_hidden_dim = FFN_HIDDEN_DIM, num_heads = NUM_HEADS, seq_len = SEQ_LEN;

    // Allocate buffers
    elem_t *input = (elem_t*)malloc(seq_len * hidden_dim * sizeof(elem_t));
    elem_t *output_gemmini = (elem_t*)malloc(seq_len * hidden_dim * sizeof(elem_t));
    elem_t *output_golden = (elem_t*)malloc(seq_len * hidden_dim * sizeof(elem_t));
    elem_t *Wq = (elem_t*)malloc(hidden_dim * hidden_dim * sizeof(elem_t));
    elem_t *Wk = (elem_t*)malloc(hidden_dim * hidden_dim * sizeof(elem_t));
    elem_t *Wv = (elem_t*)malloc(hidden_dim * hidden_dim * sizeof(elem_t));
    elem_t *Wo = (elem_t*)malloc(hidden_dim * hidden_dim * sizeof(elem_t));
    elem_t *attn_norm_w = (elem_t*)malloc(hidden_dim * sizeof(elem_t));
    elem_t *W1 = (elem_t*)malloc(ffn_hidden_dim * hidden_dim * sizeof(elem_t));
    elem_t *W2 = (elem_t*)malloc(hidden_dim * ffn_hidden_dim * sizeof(elem_t));
    elem_t *W3 = (elem_t*)malloc(ffn_hidden_dim * hidden_dim * sizeof(elem_t));
    elem_t *ffn_norm_w = (elem_t*)malloc(hidden_dim * sizeof(elem_t));
    elem_t *freqs = (elem_t*)malloc(seq_len * (hidden_dim / num_heads) * sizeof(elem_t));
    elem_t *attn_res_buf = (elem_t*)malloc(seq_len * hidden_dim * sizeof(elem_t));
    elem_t *q_buf = (elem_t*)malloc(seq_len * hidden_dim * sizeof(elem_t));
    elem_t *gemmini_q_buf = (elem_t*)malloc(seq_len * hidden_dim * sizeof(elem_t));
    elem_t *k_buf = (elem_t*)malloc(seq_len * hidden_dim * sizeof(elem_t));
    elem_t *v_buf = (elem_t*)malloc(seq_len * hidden_dim * sizeof(elem_t));
    elem_t *attn_buf = (elem_t*)malloc(num_heads * seq_len * seq_len * sizeof(elem_t));
    elem_t *attn_out_buf = (elem_t*)malloc(seq_len * hidden_dim * sizeof(elem_t));
    elem_t *ffn_x1_buf = (elem_t*)malloc(seq_len * ffn_hidden_dim * sizeof(elem_t));
    elem_t *ffn_x3_buf = (elem_t*)malloc(seq_len * ffn_hidden_dim * sizeof(elem_t));

    if (ffn_x3_buf == NULL) { printf("Out of memory!\n"); exit(1); }

    randomize_elem_t_array(Wq, hidden_dim * hidden_dim);
    randomize_elem_t_array(Wk, hidden_dim * hidden_dim);
    randomize_elem_t_array(Wv, hidden_dim * hidden_dim);
    randomize_elem_t_array(Wo, hidden_dim * hidden_dim);
    randomize_elem_t_array(attn_norm_w, hidden_dim);
    randomize_elem_t_array(W1, ffn_hidden_dim * hidden_dim);
    randomize_elem_t_array(W2, hidden_dim * ffn_hidden_dim);
    randomize_elem_t_array(W3, ffn_hidden_dim * hidden_dim);
    randomize_elem_t_array(ffn_norm_w, hidden_dim);
    precompute_rope_freqs_cpu(hidden_dim / num_heads, seq_len, 10000.0f, freqs);
    randomize_elem_t_array(input, seq_len * hidden_dim);

    printf("--- Running Benchmark: %s ---\n", name);
    printf("hidden_dim=%d, ffn_hidden_dim=%d, num_heads=%d, seq_len=%d\n", hidden_dim, ffn_hidden_dim, num_heads, seq_len);
    
    if (!rr_acquire_single(0, 0)) { printf("Failed to acquire Gemmini accelerator!\n"); exit(1); }
    uint64_t start_cycles = read_cycles();
    
    // --- Attention Block ---
    rmsnorm_cpu(seq_len, hidden_dim, input, attn_norm_w, q_buf);
    rr_set_opc(XCUSTOM_ACC, 0);
    gemmini_flush(0);
    tiled_matmul_auto(seq_len, hidden_dim, hidden_dim, q_buf, Wq, NULL, gemmini_q_buf, hidden_dim, hidden_dim, hidden_dim, hidden_dim, NN_floatToHalf(1), NN_floatToHalf(1), NN_floatToHalf(1), NO_ACTIVATION, NN_floatToHalf(1), 0, false, false, false, false, false, 0, WS);
    tiled_matmul_auto(seq_len, hidden_dim, hidden_dim, q_buf, Wk, NULL, k_buf, hidden_dim, hidden_dim, hidden_dim, hidden_dim, NN_floatToHalf(1), NN_floatToHalf(1), NN_floatToHalf(1), NO_ACTIVATION, NN_floatToHalf(1), 0, false, false, false, false, false, 0, WS);
    tiled_matmul_auto(seq_len, hidden_dim, hidden_dim, q_buf, Wv, NULL, v_buf, hidden_dim, hidden_dim, hidden_dim, hidden_dim, NN_floatToHalf(1), NN_floatToHalf(1), NN_floatToHalf(1), NO_ACTIVATION, NN_floatToHalf(1), 0, false, false, false, false, false, 0, WS);
    rr_fence(0);
    apply_rope_cpu(seq_len, hidden_dim, num_heads, freqs, gemmini_q_buf, k_buf);
    rr_set_opc(XCUSTOM_ACC, 0);
    gemmini_flush(0);
    for (int h = 0; h < num_heads; h++) {
        tiled_matmul_auto(seq_len, seq_len, hidden_dim/num_heads, gemmini_q_buf + h*(hidden_dim/num_heads), k_buf + h*(hidden_dim/num_heads), NULL, attn_buf + h*seq_len*seq_len, hidden_dim/num_heads, seq_len, seq_len, seq_len, NN_floatToHalf(1), NN_floatToHalf(1), NN_floatToHalf(1), NO_ACTIVATION, NN_floatToHalf(1), 0, false, false, true, false, false, 0, WS);
    }
    rr_fence(0);
    softmax_cpu(seq_len, seq_len * num_heads, 1.0f / sqrtf(hidden_dim/num_heads), attn_buf);
    rr_set_opc(XCUSTOM_ACC, 0);
    gemmini_flush(0);
    for (int h = 0; h < num_heads; h++) {
        tiled_matmul_auto(seq_len, hidden_dim/num_heads, seq_len, attn_buf + h*seq_len*seq_len, v_buf + h*(hidden_dim/num_heads), NULL, attn_out_buf + h*(hidden_dim/num_heads), seq_len, hidden_dim/num_heads, hidden_dim/num_heads, hidden_dim/num_heads, NN_floatToHalf(1), NN_floatToHalf(1), NN_floatToHalf(1), NO_ACTIVATION, NN_floatToHalf(1), 0, false, false, false, false, false, 0, WS);
    }
    rr_fence(0);
    rr_set_opc(XCUSTOM_ACC, 0);
    gemmini_flush(0);
    tiled_matmul_auto(seq_len, hidden_dim, hidden_dim, attn_out_buf, Wo, NULL, attn_res_buf, hidden_dim, hidden_dim, hidden_dim, hidden_dim, NN_floatToHalf(1), NN_floatToHalf(1), NN_floatToHalf(1), NO_ACTIVATION, NN_floatToHalf(1), 0, false, false, false, false, false, 0, WS);
    rr_fence(0);
    llama_resadd_cpu(seq_len, hidden_dim, input, attn_res_buf, attn_res_buf);
    
    // --- FFN Block ---
    rmsnorm_cpu(seq_len, hidden_dim, attn_res_buf, ffn_norm_w, output_gemmini);
    rr_set_opc(XCUSTOM_ACC, 0);
    gemmini_flush(0);
    tiled_matmul_auto(seq_len, ffn_hidden_dim, hidden_dim, output_gemmini, W1, NULL, ffn_x1_buf, hidden_dim, ffn_hidden_dim, ffn_hidden_dim, ffn_hidden_dim, NN_floatToHalf(1), NN_floatToHalf(1), NN_floatToHalf(1), NO_ACTIVATION, NN_floatToHalf(1), 0, false, false, false, false, false, 0, WS);
    tiled_matmul_auto(seq_len, ffn_hidden_dim, hidden_dim, output_gemmini, W3, NULL, ffn_x3_buf, hidden_dim, ffn_hidden_dim, ffn_hidden_dim, ffn_hidden_dim, NN_floatToHalf(1), NN_floatToHalf(1), NN_floatToHalf(1), NO_ACTIVATION, NN_floatToHalf(1), 0, false, false, false, false, false, 0, WS);
    rr_fence(0);
    swiglu_cpu(seq_len, ffn_hidden_dim, ffn_x1_buf, ffn_x3_buf, ffn_x1_buf);
    rr_set_opc(XCUSTOM_ACC, 0);
    gemmini_flush(0);
    tiled_matmul_auto(seq_len, hidden_dim, ffn_hidden_dim, ffn_x1_buf, W2, NULL, output_gemmini, ffn_hidden_dim, hidden_dim, hidden_dim, hidden_dim, NN_floatToHalf(1), NN_floatToHalf(1), NN_floatToHalf(1), NO_ACTIVATION, NN_floatToHalf(1), 0, false, false, false, false, false, 0, WS);
    rr_fence(0);
    llama_resadd_cpu(seq_len, hidden_dim, attn_res_buf, output_gemmini, output_gemmini);
    
    uint64_t total_cycles = read_cycles() - start_cycles;
    rr_release(0);
    printf("%s Gemmini cycles: %lu\n", name, total_cycles);
    
    /* ====== CPU Golden Path ====== */
    AttentionGoldenArgs attn_args = {hidden_dim, num_heads, seq_len, freqs, input, attn_res_buf, Wq, Wk, Wv, Wo, attn_norm_w, q_buf, k_buf, v_buf, attn_buf, attn_out_buf};
    attention_cpu_golden(&attn_args);
    
    FFNGoldenArgs ffn_args = {hidden_dim, ffn_hidden_dim, seq_len, attn_res_buf, output_golden, W1, W2, W3, ffn_norm_w, ffn_x1_buf, ffn_x3_buf};
    ffn_cpu_golden(&ffn_args);
    
    /* ====== Verification ====== */
    verify_results(name, output_gemmini, output_golden, seq_len * hidden_dim);

    exit(0);
}