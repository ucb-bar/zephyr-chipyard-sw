#!/usr/bin/env bash
set -euo pipefail

# --- Path resolution (repo-root aware) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TOOLS_MANUAL_DIR="${REPO_ROOT}/tools-manual"
PATCHES_DIR="${REPO_ROOT}/tools/patches"
LOG_FILE="${REPO_ROOT}/.install_toolchain_sdk.log"

SDK_VERSION="1.0.0-beta1"
SDK_NAME="zephyr-sdk-${SDK_VERSION}"
SDK_MINIMAL_TARBALL="${SDK_NAME}_linux-x86_64_minimal.tar.xz"
SDK_URL="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${SDK_VERSION}/${SDK_MINIMAL_TARBALL}"
SDK_INSTALL_DIR="${TOOLS_MANUAL_DIR}/${SDK_NAME}"

log() {
  local msg="[$(date +%H:%M:%S)] $*"
  printf "\n\033[1;32m%s\033[0m\n" "$msg"
  echo "$msg" >> "${LOG_FILE}" 2>/dev/null || true
}
die() {
  local msg="[$(date +%H:%M:%S)] ERROR: $*"
  printf "\n\033[1;31m%s\033[0m\n" "$msg" >&2
  echo "$msg" >> "${LOG_FILE}" 2>/dev/null || true
  exit 1
}

# Check basic host dependencies (no root required for the rest)
command -v wget >/dev/null 2>&1 || die "wget not found. Please install wget."
command -v cmake >/dev/null 2>&1 || die "cmake not found. Please install cmake."
command -v tar >/dev/null 2>&1 || die "tar not found. Please install tar."

# Ensure we have conda and the 'zephyr' env so we can use its libidn without root
CONDA_RUN=""

# Try to find conda.sh in common locations (same logic as install_submodules.sh)
if ! command -v conda >/dev/null 2>&1; then
  CONDA_SH=""
  if [[ -f "${REPO_ROOT}/tools/miniforge3/etc/profile.d/conda.sh" ]]; then
    CONDA_SH="${REPO_ROOT}/tools/miniforge3/etc/profile.d/conda.sh"
  elif [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
    CONDA_SH="${HOME}/miniforge3/etc/profile.d/conda.sh"
  elif [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
    CONDA_SH="${HOME}/anaconda3/etc/profile.d/conda.sh"
  fi

  if [[ -n "${CONDA_SH}" ]]; then
    log "Sourcing conda.sh from ${CONDA_SH}"
    # shellcheck disable=SC1090
    source "${CONDA_SH}" 2>>"${LOG_FILE}" || true
  fi
fi

if ! command -v conda >/dev/null 2>&1; then
  die "conda not found in PATH. Please run 'bash scripts/install_submodules.sh' first."
fi

# Check that the 'zephyr' env exists (created by install_submodules.sh)
if ! conda env list 2>/dev/null | grep -q "^zephyr[[:space:]]"; then
  die "Conda environment 'zephyr' not found. Please run 'bash scripts/install_submodules.sh' first."
fi

CONDA_RUN="conda run -n zephyr --no-capture-output"

# Create tools-manual directory if it doesn't exist
mkdir -p "${TOOLS_MANUAL_DIR}"
cd "${TOOLS_MANUAL_DIR}"

# Download minimal SDK if not already present
if [ ! -f "${SDK_MINIMAL_TARBALL}" ]; then
  log "Downloading Zephyr SDK minimal tarball..."
  wget -q --show-progress -N "${SDK_URL}"
else
  log "SDK tarball already exists, skipping download."
fi

# Extract SDK if not already extracted
if [ ! -d "${SDK_INSTALL_DIR}" ]; then
  log "Extracting Zephyr SDK..."
  tar xf "${SDK_MINIMAL_TARBALL}"
else
  log "SDK directory already exists, skipping extraction."
fi

# Change to SDK directory
cd "${SDK_INSTALL_DIR}"

# Run setup script to install:
# - GNU toolchain for riscv64 only (-t riscv64-zephyr-elf)
# - LLVM toolchain (-l)
# - Host tools (-h)
# - CMake package registration (-c)
# We run this inside the 'zephyr' conda environment so that the SDK's CMake
# can see libidn from conda instead of requiring a system-wide libidn.so.11.
log "Installing SDK components (GNU riscv64, LLVM, host tools) inside 'zephyr' conda env..."
${CONDA_RUN} bash -lc 'export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"; ./setup.sh -t riscv64-zephyr-elf -l -h -c'

# Copy cmake files from tools/patches to cmake/zephyr/ in SDK
if [ -d "${PATCHES_DIR}" ]; then
  log "Copying CMake patches to SDK..."
  SDK_CMAKE_DIR="${SDK_INSTALL_DIR}/cmake/zephyr"
  
  if [ ! -d "${SDK_CMAKE_DIR}" ]; then
    die "SDK cmake directory not found: ${SDK_CMAKE_DIR}"
  fi
  
  # Copy generic.cmake and target.cmake from patches
  for patch_file in generic.cmake target.cmake; do
    if [ -f "${PATCHES_DIR}/${patch_file}" ]; then
      log "Copying ${patch_file} to ${SDK_CMAKE_DIR}/"
      cp "${PATCHES_DIR}/${patch_file}" "${SDK_CMAKE_DIR}/${patch_file}"
    else
      log "Warning: ${PATCHES_DIR}/${patch_file} not found, skipping."
    fi
  done
else
  log "Warning: patches directory not found: ${PATCHES_DIR}"
fi

log "Done. Zephyr SDK installed to: ${SDK_INSTALL_DIR}"
log "SDK includes:"
log "  - GNU toolchain: riscv64-zephyr-elf"
log "  - LLVM toolchain"
log "  - Host tools"
log "  - CMake package registered"
