#!/usr/bin/env bash
set -euo pipefail

# --- Path resolution (repo-root aware) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

log() { printf "\n\033[1;32m[%s]\033[0m %s\n" "$(date +%H:%M:%S)" "$*"; }
die() { echo "ERROR: $*" >&2; exit 1; }

TOOLS_MANUAL_DIR="${REPO_ROOT}/tools-manual"
PATCHES_DIR="${REPO_ROOT}/tools/patches"

SDK_VERSION="${RB_SDK_VERSION:-1.0.0-beta1}"

# Which prebuilt SDK to fetch. The upstream release publishes one tarball per
# host, and picking the wrong one installs a toolchain that cannot execute --
# on macOS the Linux tarball extracts happily and then every tool fails with
# "bad CPU type in executable", which reads as a corrupt download.
#
# Detected rather than hardcoded so a Mac needs no local edit to this file:
# an edited installer is a permanently dirty working tree that drifts from
# upstream. Override with RB_SDK_HOST for a host this does not know, e.g.
#   RB_SDK_HOST=linux-aarch64 ./scripts/install_toolchain_sdk.sh
if [ -n "${RB_SDK_HOST:-}" ]; then
  SDK_HOST="${RB_SDK_HOST}"
else
  case "$(uname -s)-$(uname -m)" in
    Linux-x86_64)          SDK_HOST="linux-x86_64"   ;;
    Linux-aarch64)         SDK_HOST="linux-aarch64"  ;;
    Darwin-arm64)          SDK_HOST="macos-aarch64"  ;;
    Darwin-x86_64)         SDK_HOST="macos-x86_64"   ;;
    *) die "unsupported host $(uname -s)-$(uname -m); set RB_SDK_HOST to one of
       linux-x86_64 linux-aarch64 macos-aarch64 macos-x86_64" ;;
  esac
fi
# The pinned v1.0.0-beta1 release publishes a ZERO-BYTE macos tarball -- the
# asset exists and downloads 200 OK, so the failure is a silent empty extract
# rather than a 404. v1.0.0 and later ship a real macOS build. Move macOS
# forward rather than letting it install nothing, loudly, because it means the
# Mac's toolchain is not the version Linux pins.
case "${SDK_HOST}" in
  macos-*)
    if [ "${SDK_VERSION}" = "1.0.0-beta1" ] && [ -z "${RB_SDK_VERSION:-}" ]; then
      SDK_VERSION="1.0.1"
      echo "NOTE: zephyr-sdk 1.0.0-beta1 publishes an empty macOS tarball;" >&2
      echo "      using ${SDK_VERSION} instead, which ships a real macOS build." >&2
      echo "      Override with RB_SDK_VERSION if you need a specific one." >&2
    fi ;;
esac
SDK_NAME="zephyr-sdk-${SDK_VERSION}"
SDK_MINIMAL_TARBALL="${SDK_NAME}_${SDK_HOST}_minimal.tar.xz"
SDK_URL="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${SDK_VERSION}/${SDK_MINIMAL_TARBALL}"
SDK_INSTALL_DIR="${TOOLS_MANUAL_DIR}/${SDK_NAME}"

# Check dependencies
command -v wget >/dev/null 2>&1 || die "wget not found. Please install wget."
command -v cmake >/dev/null 2>&1 || die "cmake not found. Please install cmake."
command -v tar >/dev/null 2>&1 || die "tar not found. Please install tar."

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
log "Installing SDK components (GNU riscv64, LLVM, host tools)..."
./setup.sh -t riscv64-zephyr-elf -l -h -c

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
