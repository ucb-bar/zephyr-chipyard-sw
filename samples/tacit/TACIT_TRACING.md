# TACIT / L-Trace tracing on spike — working flow

TACIT ("Timestamp Annotated Core Instruction Traces", a.k.a. L-Trace;
https://ucb-bar.gitbook.io/tacit/) is UCB-BAR's open-source RISC-V processor
tracer. This sample traces a hello-world + FOC workload on spike and decodes it.

## Pipeline

    build (spike_riscv64, CONFIG_STARTUP_TACIT=y)
      -> spike --trace=l           # emits tacit.out (encoded), tacit.log, tacit.debug
      -> tacit_decoder --to-txt    # decodes to a control-flow trace (trace.txt)

One-shot: `bash samples/tacit/run_trace.sh`

Components:
- **Zephyr side**: `soc/rocketchip/virt_riscv/common/tacit/tacit.h` — the legacy
  MMIO encoder driver (`l_trace_encoder_*`, register block at `0x3000000`,
  `0x1000`/hart). `reset.S` (`CONFIG_STARTUP_TACIT`) enables the encoder at boot;
  `main.c` brackets the workload with `l_trace_encoder_start/stop`.
- **spike**: chipyard-fsim `riscv-isa-sim` @ branch `l_trace`. `--trace=<l|e>`
  selects the encoder; the per-hart commit hook feeds it; the `l` encoder writes
  `tacit.out`/`tacit.log`/`tacit.debug` in the CWD.
- **decoder**: `chipyard-fsim/software/tacit_decoder` @ branch `misc_decoders`
  (Rust). Handles Zephyr-specific control flow (hook function pointers, goto-based
  kernel code / stack unwinding). Run with `--to-txt` / `--to-json` / etc.

## What was broken (2026-07-29) and the fixes

Recent changes to the encoder (spike) drifted from the legacy MMIO interface and
the decoder's packet parser, breaking tracing. Three fixes restored it:

1. **Sample build** (`src/main.c`): missing `#include <math.h>` (the FOC workload
   uses `sqrtf`/`sin`/`cos`/`fmaxf`/`fminf`; newer toolchains error on the implicit
   declarations).

2. **spike lost the legacy MMIO encoder interface** (the "legacy tacit interface"
   breakage). The `--trace` refactor made the encoder flag-driven and *removed* the
   `0x3000000` register block, so the Zephyr guest's writes (`TR_TE_CTRL`,
   `TR_TE_TARGET`, ...) took a **store access fault → boot hang**, and the encoder's
   internal `enabled` flag (set only via `TR_TE_CTRL` bit 1) was never set → nothing
   recorded. Fix (in the sim, not this repo):
   - New device `riscv/trace_encoder_mmio.h` re-maps `0x3000000`: `TR_TE_CTRL` bit 1
     → `encoder->set_enable()`, `TR_TE_BRANCH_MODE` → `set_br_mode()`, rest absorbed;
     loads return the last written value (so the guest's RMW on `TR_TE_CTRL` works).
     Registered in `sim.cc`. **NB: spike's `bus_t` passes device-relative offsets to
     `load()/store()` — do not subtract the base again.**
   - `trace_encoder_l` left `runtime_cfg.br_mode` uninitialized → default it to
     `BR_TARG` in the ctor + `reset()`.

3. **decoder packet parser out of sync with the encoder** (`src/frontend/packet.rs`).
   The encoder's sync packet is `[header][prv byte][ctx varlen][runtime_cfg byte
   (S_START only)][target varlen][timestamp varlen]`, and trap packets carry a `prv`
   byte (+ conditional `ctx`). The decoder skipped `prv`/`runtime_cfg` and read a
   nonexistent `from_address` in sync, so every field shifted and `target_address`
   decoded to garbage (`0x400001ce80`) → immediate divergence. Fixed `FSync`/`FTrap`
   to match the encoder layout.

## Verified

`spike --trace=l` on the hello-world ELF: boots, prints `Hello World!`, exits
cleanly, traces 541,930 instructions (`tacit.out` 258 KB). The decoder decodes all
251,269 packets to a 793,201-line control-flow trace; decoded PCs match the
encoder's ground-truth `tacit.debug` exactly (reset vector `0x800001fe` → ... →
`End`). Both full-boot (`STARTUP_TACIT=y`) and main-region-only
(`STARTUP_TACIT=n`, start/stop-bracketed) traces decode.

Branch mode: encoder + decoder both default to **target mode** (`BR_TARG` / `--br-mode 0`).
For branch-predictor mode, set `TR_TE_BRANCH_MODE` in the guest and pass the matching
`--br-mode` to the decoder.

## Tracing a ModelBlaster run

The same flow traces any Zephyr ELF on the rocketchip/spike SoC, including
ModelBlaster model harnesses. Build the harness for `spike_riscv64` with
`CONFIG_STARTUP_TACIT=y` (an overlay), run on the TACIT spike, decode with the
harness ELF:

    KCFLAGS=$(python -c "from modelblaster.pipeline.backends import get; \
      print(';'.join(get('rvv').resolved_kernel_cflags('$PWD')))")
    west build -p -b spike_riscv64 modelblaster/harness -d <build> -- \
      -DMODEL_DIR=$PWD/modelblaster/examples/lenet/int8/generated/rvv \
      -DMODELBLASTER_BACKEND=rvv -DMODELBLASTER_KERNEL_CFLAGS="$KCFLAGS" \
      -DEXTRA_CONF_FILE=<CONFIG_STARTUP_TACIT=y overlay>
    ( cd <run> && spike --isa=rv64gcv_zicntr --trace=l <build>/zephyr/zephyr.elf )   # rvv: needs V+zicntr
    ltrace-decoder --binary <build>/zephyr/zephyr.elf --encoded-trace <run>/tacit.out --to-txt

**Validated (2026-07-29):**
- **LeNet int8 (rvv)**: harness runs bit-exact (`max_abs_err=0`), 5.46M instrs
  traced; decoder decodes all 2.40M packets (7.86M-line trace). RVV kernel
  instructions decode fine (rvdasm covers them). The decoded control flow matches
  LeNet's architecture exactly — `run_model_lenet` ×1, `kernel_conv2d_s8_lenet` ×2
  (conv1/conv2), `kernel_maxpool2d_s8_lenet` ×2, `kernel_linear_s8_lenet` ×3
  (fc1/fc2/fc3). `--to-stack-txt` resolves those kernel frames by name (stack
  unwinder + Zephyr goto handling work on the MB call stack).
- **Perfetto** (`--to-perfetto` → `trace.perfetto.json`, Chrome/Perfetto JSON
  trace format; load in https://ui.perfetto.dev): works. Emits call-stack slices
  (B/E per frame, driven by the StackUnwinder) with `ts` + `addr` args. The LeNet
  run yields the correct nested slices — `run_model_lenet` → conv → pool → conv →
  pool → fc×3 with per-frame timestamps, so the stacktrace is examinable in the
  Perfetto UI. (The only frames left open at trace end are `sys_reboot` /
  `sys_arch_reboot` — the HTIF exit path that never returns; Perfetto auto-closes
  them, harmless.)
- **DroNet int8 (rvv), inference-only** (substantial model): built with
  `-DMB_TACIT_TRACE_MODEL=1` (brackets only `run_model`, no boot). PASS bit-exact;
  29.9M instrs traced (`tacit.out` 5.7 MB); decoder decodes all 5.66M packets.
  Perfetto (`--to-perfetto` → `/tmp/mb_dronet_run/dronet.perfetto.json`) captures
  the full DroNet call stack, matching the architecture exactly: `run_model_dronet`
  ×1 → per-op `dispatch_dronet_0..29` → `kernel_conv2d_s8_dronet` ×10,
  `kernel_batchnorm2d_s8_dronet` ×6, `kernel_relu_s8_dronet` ×7,
  `kernel_maxpool2d_s8_dronet` ×1, `kernel_add_s8_dronet` ×3,
  `kernel_linear_s8_dronet` ×2, `kernel_sigmoid_s8_dronet` ×1. Timestamps are
  monotonic/absolute (span 29.9M), so slice durations are correct — conv
  dominates, first conv (conv0, 112²) is the largest slice (~6.95M ts).
- **DroNet int8 (scalar)**: runs to completion (467M instrs traced) — the flow
  works, but a full-boot trace of a large scalar model is enormous (tens of GB of
  the per-instruction `tacit.debug` reference).

### Stack-unwinder fixes (RVV exception handling)

Two `tacit_decoder` (`misc_decoders`, `src/backend/stack_unwinder.rs`) bugs made
the unwinder lose the caller frame (e.g. `run_model_dronet`) the moment an RVV/FPU
exception fired mid-inference; both fixed:

1. **Traps didn't isolate the handler stack.** A hardware trap (the RISC-V V/FPU
   lazy-context exception via `_isr_wrapper` → `z_riscv_fpu_trap`) pushed handler
   frames onto the same flat stack, and the handler's `ret`s cascade-popped the
   *trapped* frames (`dispatch_*`, `run_model_*`); `mret` only popped one, so the
   context was never restored. Fix: track a `trap_boundaries` stack — record the
   frame depth at each trap entry (`step_ij` for `TrapException`/`TrapInterrupt`),
   forbid normal returns from unwinding below it, and have `mret` (`TrapReturn`)
   truncate back to exactly that depth.

2. **Calls to symbol-line-info-less functions were misclassified as returns.**
   `func_symbol_map` only included symbols with DWARF line info, so libm/asm
   functions (`roundf`, `_isr_wrapper`) were absent. `step_uj`'s `is_call` test
   keys off symbol presence, so a real `jalr ra`→`roundf` (from a kernel's
   requantize) looked like a return and unwound the whole stack. Fix: include all
   code symbols regardless of line info (line info best-effort), and fix an
   alias-dedup bug that reused `next_index` and corrupted the index→address map.

Verified: DroNet inference decodes with `run_model_dronet` spanning the full trace,
all 30 kernels nested under it, and each V/FPU trap a clean bounded `_isr_wrapper`
region that returns to the correct depth.

## Tracing ExecuTorch execution

The ExecuTorch runner (`samples/executorch/executor_runner`) also supports
`-DMB_TACIT_TRACE_MODEL=1` (brackets the final, warm `method->execute()`).
Two build knobs matter on spike:
- **Build 1-core** (`CONFIG_MP_MAX_NUM_CPUS=1`, `CONFIG_SMP=n` overlay). The
  default ExecuTorch build is SMP/4-core; XNNPACK sizes its pthreadpool to the
  core count, and multi-core per-op dispatch is pathologically slow for small
  models (the ~175x tick-sleep effect) — the run won't even reach the traced
  iteration. 1-core runs each op inline (LeNet execute ~551k cycles).
- Build `-DMB_XNN_PROFILE=OFF` (skip the per-op HTIF logging).

Recipe:

    python samples/executorch/model/gen_pte_model.py --model lenet --quant int8 \
        --pte samples/executorch/model/lenet_int8.pte
    python samples/executorch/model/pte_to_header.py \
        --pte .../lenet_int8.pte --out .../executor_runner/model_pte.c
    west build -p -b spike_riscv64 samples/executorch/executor_runner/ -d <build> -- \
        -DXNNPACK_ENABLE_RISCV_VECTOR=ON -DXNNPACK_ENABLE_RISCV_GEMMINI=OFF \
        -DPYTHON_EXECUTABLE=<localpy> -DMB_XNN_PROFILE=OFF \
        -DMB_TACIT_TRACE_MODEL=1 -DEXTRA_CONF_FILE=<1-core overlay>
    ( cd <run> && spike --isa=rv64gcv_zicntr --trace=l <build>/zephyr/zephyr.elf )
    ltrace-decoder --binary <build>/zephyr/zephyr.elf --encoded-trace <run>/tacit.out --to-perfetto

**Validated (LeNet int8, 2026-07-29):** warm `execute()` traced = 551,167
instrs; decoder decodes all 136,272 packets (balanced call stack, depth 20). The
reconstructed stack shows the full ExecuTorch→XNNPACK dispatch into the RVV
microkernel:

    Method::execute -> execute_instruction -> BackendDelegate::Execute
      -> XnnpackBackend::execute -> XNNExecutor::forward -> xnn_invoke_runtime
      -> xnn_run_operator_with_index -> pthreadpool_parallelize_2d_tile_2d_dynamic
      -> xnn_compute_igemm -> xnn_qs8_qc8w_igemm_minmax_fp32_ukernel_4x4v__rvv

(ExecuTorch code sits high in memory, ~0x8077_xxxx; the runner ELF is ~184 MB
because of the embedded .pte + method allocator pool. The stack-unwinder fixes
above handle the C++/XNNPACK/pthreadpool call graph and its V-context traps.)

### `MB_TACIT_TRACE_MODEL` — trace just the inference (preferred for big models)

`harness/src/main.c` + `harness/CMakeLists.txt` support an opt-in build flag
`-DMB_TACIT_TRACE_MODEL=1` that brackets **only** `run_model()` with the encoder
start/stop (via `<tacit/tacit.h>`). This traces the model's control flow without
boot — far smaller than `CONFIG_STARTUP_TACIT` and the right choice for
substantial models. Default off (no effect on normal runs). Run the resulting ELF
on the TACIT spike with `--trace=l` (rvv: `--isa=rv64gcv_zicntr`).
