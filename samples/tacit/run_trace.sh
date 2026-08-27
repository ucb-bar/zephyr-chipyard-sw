#!/usr/bin/env bash
# End-to-end TACIT / L-Trace tracing on spike for the tacit hello-world sample.
#
#   build (spike_riscv64) -> spike --trace=l  (emits tacit.out) -> tacit_decoder
#
# Requires the TACIT-enabled spike (chipyard-fsim riscv-isa-sim @ branch l_trace,
# with the legacy encoder MMIO block re-mapped -- see trace_encoder_mmio.h) and
# the tacit_decoder (@ misc_decoders). Both were fixed on 2026-07-29 to restore
# the legacy Zephyr encoder interface + align the sync/trap packet format.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ENVBIN="$REPO/tools/miniforge3/envs/zephyr/bin"
SPIKE="${TACIT_SPIKE:-/scratch2/dima/chipyard-fsim/toolchains/riscv-tools/riscv-isa-sim/build/spike}"
DECODER_DIR="${TACIT_DECODER_DIR:-/scratch2/dima/chipyard-fsim/software/tacit_decoder}"
BUILD="${BUILD_DIR:-/tmp/tacit_hello_build}"
RUN="${RUN_DIR:-/tmp/tacit_run}"

echo "[1/3] build (spike_riscv64)"
source "$REPO/scripts/set_envvars_sdk.sh" >/dev/null 2>&1
export PATH="$ENVBIN:$PATH" WEST_PYTHON="$ENVBIN/python"
west build -p -b spike_riscv64 "$REPO/samples/tacit" -d "$BUILD" >/dev/null
ELF="$BUILD/zephyr/zephyr.elf"

echo "[2/3] spike --trace=l  (emit tacit.out)"
rm -rf "$RUN"; mkdir -p "$RUN"; ( cd "$RUN" && "$SPIKE" --trace=l "$ELF" )
echo "      tacit.out=$(stat -c%s "$RUN/tacit.out") B, $(wc -l < "$RUN/tacit.debug") instrs traced"

echo "[3/3] decode (tacit_decoder, misc_decoders)"
( cd "$DECODER_DIR" && cargo build --release >/dev/null 2>&1 \
  && ./target/release/ltrace-decoder --binary "$ELF" \
       --encoded-trace "$RUN/tacit.out" --to-txt )
echo "      decoded -> $DECODER_DIR/trace.txt ($(wc -l < "$DECODER_DIR/trace.txt") lines)"
echo "DONE: TACIT tracing works end-to-end."
