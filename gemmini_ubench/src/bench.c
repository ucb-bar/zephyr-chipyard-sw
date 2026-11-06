/*
 * Gemmini conv microbenchmark (bench.c) — compiled with the Gemmini Q0.31
 * kernel cflags (see CMakeLists.txt), so it sees the SAME gemmini.h /
 * gemmini_params.h / weight-layout contract as the modelblaster FPGA build
 * that produced the 2.48x im2col-vs-tiled_conv number.
 *
 * For each conv shape, runs the identical convolution three ways and prints
 * rdcycle deltas as CSV:
 *   A) tiled_conv_auto                 (LoopConv: HW im2col+GEMM+requant)
 *   B) SW im2col in DIM=16 row tiles + tiled_matmul_auto per tile
 *      (LoopMatmul, byte-faithful to modelblaster gemmini_im2col_full_C)
 *   C) SW im2col over ALL rows once  + ONE tiled_matmul_auto
 *      (gap-narrowing variant: one big-M GEMM instead of 196 skinny ones)
 *
 * Path B is broken into transpose / im2col-gather / GEMM / requantize so we
 * can see whether the LoopMatmul penalty is software rearrangement or the
 * GEMM itself. All three are checked against a scalar golden.
 *
 * N is always 1 (DroNet inference). Weights are HWIO [KH*KW*IC, OC], the
 * layout tiled_conv_auto AND tiled_matmul_auto both consume directly.
 */
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <gemmini.h>
#include <gemmini_params.h>

/* NOTE: the software rearrangement (NCHW->NHWC transpose, im2col gather,
 * Q0.31 requantize) is done here in SCALAR C. The shipped modelblaster
 * im2col_full_C kernel vectorises these with Saturn RVV; on this bitstream
 * the boot hart's V-state needs harness setup we deliberately avoid, so the
 * SW-rearrangement cycles reported below are an UPPER BOUND relative to the
 * RVV baseline. The GEMM (tiled_matmul_auto) and tiled_conv_auto numbers are
 * exact hardware measurements and are unaffected by this choice. */

/* ---- timing ------------------------------------------------------------- */
static inline uint64_t rdc(void)
{
    uint64_t c;
    asm volatile("rdcycle %0" : "=r"(c));
    return c;
}

/* ---- static workspaces (DDR .bss). Sized for the worst DroNet layer plus
 * the clean 32x32 test layer. ------------------------------------------------
 *   in_nchw      max IC*IH*IW      = conv0 3*112*112       = 37,632
 *   weight_hwio  max KH*KW*IC*OC   = conv8 9*128*128       = 147,456
 *   ws_input     max IH*IW*IC (NHWC) = 37,632
 *   ws_out_nhwc  max OH*OW*OC       = conv0 32*56*56        = 100,352
 *   im2col_full  max total_out*K_in = clean 1024*144        = 147,456
 *   acc_full     max total_out*OC   = conv0 3136*32         = 100,352 (int32)
 *   out_*        max OC*OH*OW       = 100,352
 */
enum {
    B_I8   = 160 * 1024,   /* generic int8 buffers */
    B_IM   = 192 * 1024,   /* full im2col matrix */
    B_ACC  = 128 * 1024,   /* int32 accumulator elements */
};
static elem_t in_nchw   [B_I8]  __attribute__((aligned(64)));
static elem_t weight_hwio[B_I8] __attribute__((aligned(64)));
static acc_t  bias_buf  [1024]  __attribute__((aligned(64)));
static elem_t ws_input  [B_I8]  __attribute__((aligned(64)));   /* NHWC input */
static elem_t ws_out_nhwc[B_ACC] __attribute__((aligned(64)));  /* tiled_conv out */
static elem_t ws_im2col [B_IM]  __attribute__((aligned(64)));   /* im2col matrix */
static acc_t  ws_acc    [B_ACC] __attribute__((aligned(64)));   /* int32 GEMM out */
static elem_t out_gold  [B_ACC] __attribute__((aligned(64)));
static elem_t out_a     [B_ACC] __attribute__((aligned(64)));
static elem_t out_b     [B_ACC] __attribute__((aligned(64)));
static elem_t out_c     [B_ACC] __attribute__((aligned(64)));

/* ---- scalar byte-move helpers (memcpy/memset equivalents) -------------- */
static inline void vcopy_i8(elem_t *dst, const elem_t *src, size_t n)
{ memcpy(dst, src, n); }
static inline void vzero_i8(elem_t *dst, size_t n)
{ memset(dst, 0, n); }

/* NCHW -> NHWC transpose (scalar), N==1, into ws_input.
 * dst[(h*IW+w)*IC + c] = input[(c*IH+h)*IW + w].  Both A and B pay this cost,
 * exactly as every DroNet conv layer does. */
static void transpose_nchw_nhwc(const elem_t *input, int IC, int IH, int IW)
{
    for (int c = 0; c < IC; c++) {
        const elem_t *src = input + (size_t)c * IH * IW;
        for (int h = 0; h < IH; h++) {
            elem_t *dp = ws_input + ((size_t)h * IW) * IC + c;
            const elem_t *sp = src + (size_t)h * IW;
            for (int w = 0; w < IW; w++) dp[(size_t)w * IC] = sp[w];
        }
    }
}

/* Build one im2col row (output position out_idx) from NHWC ws_input into `row`.
 * Byte-identical to the modelblaster kernel's per-row gather. */
static inline void im2col_row(elem_t *row, int out_idx,
                              int IC, int IH, int IW, int OW,
                              int KH, int KW, int SH, int SW, int PH, int PW)
{
    int ow_idx = out_idx % OW;
    int oh_idx = out_idx / OW;
    const int iw0 = ow_idx * SW - PW;
    int kw_lo = -iw0 > 0 ? -iw0 : 0;
    int kw_hi = (IW - iw0) < KW ? (IW - iw0) : KW;
    if (kw_lo > KW) kw_lo = KW;
    if (kw_hi < kw_lo) kw_hi = kw_lo;
    for (int kh = 0; kh < KH; kh++) {
        int ih = oh_idx * SH - PH + kh;
        elem_t *base = row + (size_t)kh * KW * IC;
        if (ih < 0 || ih >= IH) { vzero_i8(base, (size_t)KW * IC); continue; }
        if (kw_lo > 0) vzero_i8(base, (size_t)kw_lo * IC);
        if (kw_hi > kw_lo) {
            const elem_t *src = &ws_input[(((size_t)ih * IW) + (size_t)(iw0 + kw_lo)) * IC];
            vcopy_i8(base + (size_t)kw_lo * IC, src, (size_t)(kw_hi - kw_lo) * IC);
        }
        if (kw_hi < KW)
            vzero_i8(base + (size_t)kw_hi * IC, (size_t)(KW - kw_hi) * IC);
    }
}

/* Scalar two-stage Q0.31 requantize of `rows` accumulator rows starting at
 * out row base_row, from acc[] (row-major, OC per row) into NCHW `out`. */
static void requant_rows(const acc_t *acc, elem_t *out, int base_row, int rows,
                         int OC, int OH, int OW,
                         int mult, int shift, int off, int amin, int amax)
{
    for (int i = 0; i < rows; i++) {
        int out_idx = base_row + i;
        int ow_idx  = out_idx % OW;
        int oh_idx  = out_idx / OW;
        elem_t *dst0 = out + (size_t)oh_idx * OW + ow_idx;
        for (int oc = 0; oc < OC; oc++) {
            int32_t a = acc[(size_t)i * OC + oc];
            int64_t prod = ((int64_t)a * (int64_t)mult + ((int64_t)1 << 30)) >> 31;
            int32_t s = (int32_t)prod;
            if (shift > 0) s = (int32_t)(((int64_t)s + ((int64_t)1 << (shift-1))) >> shift);
            else if (shift < 0) s <<= (-shift);
            s += off;
            if (s < amin) s = amin;
            if (s > amax) s = amax;
            dst0[(size_t)oc * OH * OW] = (int8_t)s;
        }
    }
}

/* ---- scalar golden (exact two-stage Q0.31), NCHW out ------------------- */
static void golden(const elem_t *input, const elem_t *weight, const acc_t *bias,
                   elem_t *out, int IC, int IH, int IW, int OC,
                   int KH, int KW, int SH, int SW, int PH, int PW,
                   int mult, int shift, int off, int amin, int amax)
{
    int OH = (IH + 2*PH - KH) / SH + 1;
    int OW = (IW + 2*PW - KW) / SW + 1;
    for (int oc = 0; oc < OC; oc++)
    for (int oh = 0; oh < OH; oh++)
    for (int ow = 0; ow < OW; ow++) {
        int32_t acc = bias ? bias[oc] : 0;
        for (int ic = 0; ic < IC; ic++)
        for (int kh = 0; kh < KH; kh++) {
            int ih = oh * SH - PH + kh;
            for (int kw = 0; kw < KW; kw++) {
                int iw = ow * SW - PW + kw;
                int32_t in_v = (ih < 0 || ih >= IH || iw < 0 || iw >= IW)
                    ? 0 : input[((size_t)ic*IH+ih)*IW+iw];
                acc += in_v * (int32_t)weight[(((size_t)kh*KW+kw)*IC+ic)*OC+oc];
            }
        }
        int64_t prod = (int64_t)acc * (int64_t)mult;
        prod = (prod + ((int64_t)1 << 30)) >> 31;
        int32_t s = (int32_t)prod;
        if (shift > 0) s = (int32_t)(((int64_t)s + ((int64_t)1 << (shift-1))) >> shift);
        else if (shift < 0) s <<= (-shift);
        s += off;
        if (s < amin) s = amin;
        if (s > amax) s = amax;
        out[((size_t)oc*OH+oh)*OW+ow] = (int8_t)s;
    }
}

static int max_abs_err(const elem_t *a, const elem_t *b, size_t n)
{
    int m = 0;
    for (size_t i = 0; i < n; i++) {
        int d = (int)a[i] - (int)b[i];
        if (d < 0) d = -d;
        if (d > m) m = d;
    }
    return m;
}

/* ---- shape table -------------------------------------------------------- */
struct shape { const char *name; int IC, IH, IW, OC, KH, KW, SH, SW, PH, PW; };
static const struct shape shapes[] = {
    /* DroNet's 10 conv layers (from con_build_mb_dronet_hetero_fpga.log) */
    {"c0",  3, 112, 112,  32, 3,3, 2,2, 1,1},
    {"c1", 32,  27,  27,  32, 3,3, 2,2, 1,1},
    {"c2", 32,  14,  14,  32, 3,3, 1,1, 1,1},
    {"c3", 32,  27,  27,  32, 1,1, 2,2, 0,0},
    {"c4", 32,  14,  14,  64, 3,3, 2,2, 1,1},
    {"c5", 64,   7,   7,  64, 3,3, 1,1, 1,1},
    {"c6", 32,  14,  14,  64, 1,1, 2,2, 0,0},
    {"c7", 64,   7,   7, 128, 3,3, 2,2, 1,1},
    {"c8",128,   4,   4, 128, 3,3, 1,1, 1,1},
    {"c9", 64,   7,   7, 128, 1,1, 2,2, 0,0},
    /* one clean square layer with a healthy output height (good GEMM M) */
    {"big",16,  32,  32,  64, 3,3, 1,1, 1,1},
};
enum { NSHAPE = sizeof(shapes)/sizeof(shapes[0]) };

/* Requant params (fixed; symmetric per-tensor, offsets 0 for the HW paths).
 * shift=1 exercises the two-stage rounding that makes tiled_conv (folded
 * Q0.31) drift <=1 LSB while the CPU-requant im2col paths stay bit-exact. */
enum { R_MULT = 0x60000000, R_SHIFT = 1, R_OFF = 0, R_MIN = -128, R_MAX = 127 };

static uint32_t rng;
static inline int8_t rnd8(int lo, int span)
{ rng = rng*1664525u + 1013904223u; return (int8_t)(lo + (int)((rng>>17)%(unsigned)span)); }

static void gemmini_on(void)
{
    asm volatile("csrs mstatus, %0" : : "r"(0x18000) : "memory");
    gemmini_flush(0);
}

extern void bench_print(const char *s);   /* thin printk wrapper in main.c */
static char line[256];
static void emit(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    vsnprintk(line, sizeof(line), fmt, ap);
    va_end(ap);
    bench_print(line);
}

void bench_run_shape(const struct shape *s)
{
    int IC=s->IC, IH=s->IH, IW=s->IW, OC=s->OC;
    int KH=s->KH, KW=s->KW, SH=s->SH, SW=s->SW, PH=s->PH, PW=s->PW;
    int OH = (IH + 2*PH - KH) / SH + 1;
    int OW = (IW + 2*PW - KW) / SW + 1;
    int K_inner = IC * KH * KW;
    int total_out = OH * OW;                 /* N==1 */
    size_t out_elems = (size_t)OC * OH * OW;

    /* deterministic data (same for every path) */
    rng = 0x1234567u ^ (uint32_t)(IC*131 + OC*17 + IH);
    for (size_t i = 0; i < (size_t)IC*IH*IW; i++) in_nchw[i] = rnd8(-4, 8);
    for (size_t i = 0; i < (size_t)K_inner*OC; i++) weight_hwio[i] = rnd8(-4, 8);
    for (int oc = 0; oc < OC; oc++) bias_buf[oc] = rnd8(-16, 32);

    golden(in_nchw, weight_hwio, bias_buf, out_gold,
           IC,IH,IW,OC,KH,KW,SH,SW,PH,PW, R_MULT,R_SHIFT,R_OFF,R_MIN,R_MAX);

    uint64_t t0,t1,t2,t3;

    /* ------- Path A: tiled_conv_auto (LoopConv) ------- */
    gemmini_on();
    t0 = rdc();
    transpose_nchw_nhwc(in_nchw, IC, IH, IW);
    asm volatile("fence" ::: "memory");
    t1 = rdc();
    /* fold (mult,shift) -> single Q0.31 scale for HW mvout requant */
    int32_t scale_q31 = R_SHIFT == 0 ? R_MULT
        : (int32_t)(((int64_t)R_MULT + ((int64_t)1 << (R_SHIFT-1))) >> R_SHIFT);
    tiled_conv_auto(
        1, IH, IW, IC, OC, OH, OW,
        SH, 1, 1, PH, KH,
        false, false, false, false, false,
        ws_input, weight_hwio, bias_buf, ws_out_nhwc,
        NO_ACTIVATION, (acc_scale_t)scale_q31,
        0, 0, 0, WS);
    gemmini_fence();
    gemmini_flush(0);
    t2 = rdc();
    /* NHWC -> NCHW out_a */
    for (int oc = 0; oc < OC; oc++)
        for (int p = 0; p < OH*OW; p++)
            out_a[(size_t)oc*OH*OW + p] = ws_out_nhwc[(size_t)p*OC + oc];
    t3 = rdc();
    uint64_t A_tr=t1-t0, A_conv=t2-t1, A_out=t3-t2, A_tot=t3-t0;

    /* ------- Path B: SW im2col in DIM tiles + tiled_matmul_auto per tile ------- */
    uint64_t B_tr=0, B_im=0, B_gemm=0, B_req=0;
    gemmini_on();
    t0 = rdc();
    transpose_nchw_nhwc(in_nchw, IC, IH, IW);
    asm volatile("fence" ::: "memory");
    t1 = rdc(); B_tr = t1 - t0;
    for (int ti = 0; ti < total_out; ti += DIM) {
        int rows = total_out - ti < DIM ? total_out - ti : DIM;
        t0 = rdc();
        for (int i = 0; i < DIM; i++) {
            elem_t *row = &ws_im2col[(size_t)i * K_inner];
            if (i >= rows) { vzero_i8(row, (size_t)K_inner); continue; }
            im2col_row(row, ti+i, IC,IH,IW,OW, KH,KW,SH,SW,PH,PW);
        }
        asm volatile("fence" ::: "memory");
        t1 = rdc(); B_im += t1 - t0;
        tiled_matmul_auto(
            DIM, OC, K_inner,
            ws_im2col, weight_hwio, (const void*)bias_buf, (void*)ws_acc,
            K_inner, OC, OC, OC,
            MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, (scale_acc_t)1,
            NO_ACTIVATION, ACC_SCALE_IDENTITY, (acc_scale_t)0,
            bias_buf != NULL, false, false, true, false, 0, WS);
        gemmini_fence();
        gemmini_flush(0);
        t2 = rdc(); B_gemm += t2 - t1;
        requant_rows(ws_acc, out_b, ti, rows, OC,OH,OW, R_MULT,R_SHIFT,R_OFF,R_MIN,R_MAX);
        t3 = rdc(); B_req += t3 - t2;
    }
    uint64_t B_tot = B_tr + B_im + B_gemm + B_req;

    /* ------- Path C: SW im2col over ALL rows + ONE big tiled_matmul_auto ------- */
    uint64_t C_tr=0, C_im=0, C_gemm=0, C_req=0;
    gemmini_on();
    t0 = rdc();
    transpose_nchw_nhwc(in_nchw, IC, IH, IW);
    asm volatile("fence" ::: "memory");
    t1 = rdc(); C_tr = t1 - t0;
    for (int r = 0; r < total_out; r++)
        im2col_row(&ws_im2col[(size_t)r * K_inner], r, IC,IH,IW,OW, KH,KW,SH,SW,PH,PW);
    asm volatile("fence" ::: "memory");
    t2 = rdc(); C_im = t2 - t1;
    tiled_matmul_auto(
        (size_t)total_out, (size_t)OC, (size_t)K_inner,
        ws_im2col, weight_hwio, (const void*)bias_buf, (void*)ws_acc,
        (size_t)K_inner, (size_t)OC, (size_t)OC, (size_t)OC,
        MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, (scale_acc_t)1,
        NO_ACTIVATION, ACC_SCALE_IDENTITY, (acc_scale_t)0,
        bias_buf != NULL, false, false, true, false, 0, WS);
    gemmini_fence();
    gemmini_flush(0);
    t3 = rdc(); C_gemm = t3 - t2;
    requant_rows(ws_acc, out_c, 0, total_out, OC,OH,OW, R_MULT,R_SHIFT,R_OFF,R_MIN,R_MAX);
    C_req = rdc() - t3;
    uint64_t C_tot = C_tr + C_im + C_gemm + C_req;

    int errA = max_abs_err(out_a, out_gold, out_elems);
    int errB = max_abs_err(out_b, out_gold, out_elems);
    int errC = max_abs_err(out_c, out_gold, out_elems);

    /* CSV: one machine-parseable line per shape, then a human summary. */
    emit("UB,%s,IC=%d,IH=%d,IW=%d,OC=%d,OH=%d,OW=%d,K=%d,S=%d,P=%d,"
         "A_tot=%llu,A_tr=%llu,A_conv=%llu,A_out=%llu,"
         "B_tot=%llu,B_tr=%llu,B_im=%llu,B_gemm=%llu,B_req=%llu,"
         "C_tot=%llu,C_tr=%llu,C_im=%llu,C_gemm=%llu,C_req=%llu,"
         "errA=%d,errB=%d,errC=%d\n",
         s->name, IC,IH,IW,OC,OH,OW,KH,SH,PH,
         (unsigned long long)A_tot,(unsigned long long)A_tr,
         (unsigned long long)A_conv,(unsigned long long)A_out,
         (unsigned long long)B_tot,(unsigned long long)B_tr,(unsigned long long)B_im,
         (unsigned long long)B_gemm,(unsigned long long)B_req,
         (unsigned long long)C_tot,(unsigned long long)C_tr,(unsigned long long)C_im,
         (unsigned long long)C_gemm,(unsigned long long)C_req,
         errA,errB,errC);
}

void bench_all(void)
{
    bench_print("=== GEMMINI_UBENCH_BEGIN ===\n");
    emit("cols: name,shape, A(LoopConv) tot/tr/conv/out, "
         "B(LoopMatmul tiled) tot/tr/im2col/gemm/req, "
         "C(LoopMatmul 1-GEMM) tot/tr/im2col/gemm/req, errA/B/C\n");
    for (int i = 0; i < NSHAPE; i++)
        bench_run_shape(&shapes[i]);
    bench_print("=== GEMMINI_UBENCH_END ===\n");
}
