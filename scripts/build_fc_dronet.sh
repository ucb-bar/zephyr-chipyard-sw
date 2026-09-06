#!/usr/bin/env bash
# Build the integrated FC + DroNet ELF for the riskybird v3 FPGA Rocket combined
# core RocketArty200TDroneGemminiSaturnFp16At35Config (Gemmini + Saturn + full
# drone periphery). Off-bench; produces build_fc_dronet/zephyr/zephyr.elf.
#
# Env overrides:
#   MB_REPO   modelblaster checkout to pull the DroNet model + gemmini runtime
#             from (default: the sibling zephyr-chipyard-sw checkout's modelblaster,
#             which is 10b50ba-equivalent and carries the generated model).
#   MODEL_DIR the generated DroNet model dir (default: the 13.31 fps grayscale
#             integrated build, generated_gray/hetero_tiled_rvvadd).
#   IRQ_GATE  1 => build the mid-V-preemption-safe fallback (DRONET_IRQ_GATE=1).
set -uo pipefail

WT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"          # this worktree
MAIN="${MAIN:-/home/cobble/Tools/zephyr-chipyard-sw}"           # conda + SDK live here
MB_REPO="${MB_REPO:-${MAIN}/modelblaster}"
MODEL_DIR="${MODEL_DIR:-${MB_REPO}/examples/dronet/int8/generated_gray/hetero_tiled_rvvadd}"
BUILDDIR="${BUILDDIR:-${WT}/build_fc_dronet}"
IRQ_GATE="${IRQ_GATE:-0}"

# Toolchain + west workspace come from the main checkout (a valid west workspace
# whose Zephyr submodule is the same riskybirdv3-bringup tip, bfd185f, with the
# FC driver fixes + the fp16 boot fix). The build reads it read-only; the sample,
# FC sources and build dir all live in this worktree.
source "${MAIN}/scripts/activate_conda.sh"
source "${MAIN}/scripts/set_envvars_sdk.sh"
export PATH="/usr/bin:${PATH}"   # avoid stale Vitis cmake

KCFLAGS="-march=rv64imac_zve64x;-mabi=lp64"
KCFLAGS="${KCFLAGS};-isystem${MB_REPO}/cores/gemmini/include/per_config/default16x16"
KCFLAGS="${KCFLAGS};-isystem${MB_REPO}/cores/gemmini/include"
KCFLAGS="${KCFLAGS};-isystem${MB_REPO}/cores/gemmini"
KCFLAGS="${KCFLAGS};-DGEMMINI_ROCC;-DBAREMETAL"
KCFLAGS="${KCFLAGS};-DMODELBLASTER_GEMMINI_HWIO_WEIGHTS=1;-DMODELBLASTER_GEMMINI_Q31_ACC_SCALE=1"

echo "=== FC+DroNet build ==="
echo "  worktree = ${WT}"
echo "  MB_REPO  = ${MB_REPO}"
echo "  MODEL_DIR= ${MODEL_DIR}"
echo "  builddir = ${BUILDDIR}  irq_gate=${IRQ_GATE}"

cd "${MAIN}"
west build -p -b chipyard_riscv64 "${WT}/samples/rose_fc_dronet" --build-dir "${BUILDDIR}" -- \
  -DMB_REPO="${MB_REPO}" \
  -DMODEL_DIR="${MODEL_DIR}" \
  -DMODELBLASTER_KERNEL_CFLAGS="${KCFLAGS}" \
  -DEXTRA_CPPFLAGS="-DROSE_MOTORS_INHIBIT=1 -DROSE_PROFILE=1 -DDRONET_IRQ_GATE=${IRQ_GATE}"
rc=$?
echo "=== BUILD EXIT ${rc} ==="
ls -la "${BUILDDIR}/zephyr/zephyr.elf" 2>&1
exit ${rc}
