#!/usr/bin/env bash
set -euo pipefail

# --- Path resolution (repo-root aware) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TOOLS_DIR="${REPO_ROOT}/tools"
RGT_DIR="${TOOLS_DIR}/riscv-gnu-toolchain"
PICOLIBC_DIR="${TOOLS_DIR}/picolibc"
INSTALL_DIR="${TOOLS_DIR}/riscv-install"

JOBS="${JOBS:-$(nproc)}"

log() { printf "\n\033[1;32m[%s]\033[0m %s\n" "$(date +%H:%M:%S)" "$*"; }
die() { echo "ERROR: $*" >&2; exit 1; }

command -v meson >/dev/null 2>&1 || die "meson not found. Try: pip install meson ninja"
command -v ninja >/dev/null 2>&1 || die "ninja not found. Try: pip install ninja"

# --- Build riscv-gnu-toolchain (GCC+newlib) ---
log "Configuring submodules for riscv-gnu-toolchain (skipping dejagnu)…"
cd "${RGT_DIR}"
# Only init the submodules we actually need to build a newlib toolchain.
git submodule update --init --depth=1 gcc binutils gdb newlib || true

log "Configuring riscv-gnu-toolchain…"
./configure \
  --prefix="${INSTALL_DIR}" \
  --with-arch=rv64gc \
  --with-abi=lp64d \
  --enable-multilib \
  --with-multilib-generator="rv64gc_zfh-lp64d--;rv64gcv_zfh-lp64d--" \
  --with-cmodel=medany

log "Building riscv-gnu-toolchain (forcing system /usr/bin ahead of Conda for gettext/msgfmt)…"
ORIG_PATH="$PATH"
export PATH="/usr/local/bin:/usr/bin:/bin:${ORIG_PATH}"
make -j"${JOBS}"
export PATH="${ORIG_PATH}"

# --- Optional: apply Zephyr compatibility patch (reverse) after install ---
PATCH_FILE="${TOOLS_DIR}/gcc15.patch"
if [[ -f "${PATCH_FILE}" ]]; then
  log "Applying reverse patch to install tree: ${PATCH_FILE}"
  patch -d "${INSTALL_DIR}" -p1 -R < "${PATCH_FILE}"
else
  log "No ${PATCH_FILE} found — skipping post-install patch step."
fi

# Put new toolchain on PATH for Picolibc build
export PATH="${INSTALL_DIR}/bin:${PATH}"

# --- Build Picolibc (separate install tree under ${INSTALL_DIR}/picolibc/…) ---
log "Preparing Picolibc build…"
cd "${PICOLIBC_DIR}"
git submodule update --init --recursive || true

BUILD_DIR="${PICOLIBC_DIR}/build-riscv64-unknown-elf"
mkdir -p "${BUILD_DIR}"

CROSS_FILE="${BUILD_DIR}/cross-riscv64-unknown-elf.txt"
cat > "${CROSS_FILE}" <<EOF
[binaries]
c = 'riscv64-unknown-elf-gcc'
ar = 'riscv64-unknown-elf-ar'
as = 'riscv64-unknown-elf-as'
ld = 'riscv64-unknown-elf-ld'
strip = 'riscv64-unknown-elf-strip'

[host_machine]
system = 'unknown'
cpu_family = 'riscv'
cpu = 'riscv'
endian = 'little'

[properties]
needs_exe_wrapper = true
skip_sanity_check = true

[built-in options]
c_args = ['-nostdlib','-msave-restore','-fno-common','-fpic']
EOF

# Auto-pick a scalar + vector multilib that include lp64d (prefer Zfh)
log "Selecting Picolibc multilibs from GCC…"
mapfile -t MLIBS < <(riscv64-unknown-elf-gcc --print-multi-lib | awk -F';' '{print $1}')
pick_scalar=""
pick_vector=""

for ml in "${MLIBS[@]}"; do
  [[ "$ml" == "." ]] && continue
  if [[ "$ml" == *"/lp64d"* && "$ml" == *"rv64"* ]]; then
    if [[ -z "$pick_vector" && "$ml" =~ v ]]; then
      pick_vector="$ml"
    elif [[ -z "$pick_scalar" && ! "$ml" =~ v ]]; then
      pick_scalar="$ml"
    fi
  fi
done

# Prefer Zfh variants if alternatives exist
pref() {
  local a="$1" b="$2"
  if [[ -n "$b" && "$b" == *"zfh"* ]]; then echo "$b"; else echo "$a"; fi
}
# Try to upgrade choices to zfh versions if present
for ml in "${MLIBS[@]}"; do
  [[ "$ml" == "." ]] && continue
  if [[ -n "$pick_scalar" && "$ml" == *"/lp64d"* && "$ml" =~ rv64 && ! "$ml" =~ v && "$ml" == *"zfh"* ]]; then
    pick_scalar="$(pref "$pick_scalar" "$ml")"
  fi
  if [[ -n "$pick_vector" && "$ml" == *"/lp64d"* && "$ml" =~ rv64 && "$ml" =~ v && "$ml" == *"zfh"* ]]; then
    pick_vector="$(pref "$pick_vector" "$ml")"
  fi
done

ML_FLAG=""
if [[ -n "$pick_scalar" || -n "$pick_vector" ]]; then
  sel=()
  [[ -n "$pick_scalar" ]] && sel+=("$pick_scalar")
  [[ -n "$pick_vector" ]] && sel+=("$pick_vector")
  ML_FLAG="-Dmultilib-list=$(IFS=,; echo "${sel[*]}")"
  log "Using Picolibc multilib list: ${ML_FLAG#*=}"
else
  log "No specific multilibs selected; Picolibc will auto-detect all supported multilibs."
fi

log "Configuring Picolibc with Meson…"
cd "${BUILD_DIR}"
meson setup .. \
  --cross-file="${CROSS_FILE}" \
  -Dprefix="${INSTALL_DIR}" \
  ${ML_FLAG} \
  -Dthread-local-storage=true \
  -Dtls-model=local-exec \
  -Dnewlib-global-errno=false

log "Building and installing Picolibc…"
ninja -j"${JOBS}"
ninja install

log "Done. Toolchain and Picolibc installed to: ${INSTALL_DIR}"
log "Binaries: ${INSTALL_DIR}/bin  •  Picolibc sysroot: ${INSTALL_DIR}/picolibc/riscv64-unknown-elf"
