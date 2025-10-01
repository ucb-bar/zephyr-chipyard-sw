#!/usr/bin/env bash
# scripts/install_conda.sh
# Install Miniforge3 (Linux/x86_64) into ./tools/miniforge3 non-interactively,
# then configure solver and install clang (lib + python bindings) + dotmap.
# Re-runnable and idempotent.

set -euo pipefail

# Resolve repo root and target dirs (script may be called from anywhere)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TOOLS_DIR="${REPO_ROOT}/tools"
INSTALL_DIR="${TOOLS_DIR}/miniforge3"
INSTALLER="${TOOLS_DIR}/Miniforge3-Linux-x86_64.sh"
MINIFORGE_URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh"

# Versions to keep clang pieces consistent (avoid solver conflicts)
CLANG_MAJOR="18"

# --- prereqs ---
if ! command -v wget >/dev/null 2>&1; then
  echo "Error: wget is required but not found. Please install wget and re-run." >&2
  return 1 2>/dev/null || exit 1
fi

mkdir -p "${TOOLS_DIR}"

# --- download installer (if needed) ---
if [[ ! -f "${INSTALLER}" ]]; then
  echo "Downloading Miniforge installer..."
  wget -q --show-progress -O "${INSTALLER}" "${MINIFORGE_URL}"
  chmod +x "${INSTALLER}"
else
  echo "Installer already exists at ${INSTALLER} (skipping download)."
fi

# --- install Miniforge (idempotent) ---
if [[ -d "${INSTALL_DIR}" ]]; then
  echo "Miniforge appears installed at ${INSTALL_DIR} (skipping install)."
else
  echo "Installing Miniforge to ${INSTALL_DIR} ..."
  bash "${INSTALLER}" -b -p "${INSTALL_DIR}"
fi

# --- init conda in this shell ---
CONDA_SH="${INSTALL_DIR}/etc/profile.d/conda.sh"
if [[ -f "${CONDA_SH}" ]]; then
  # shellcheck disable=SC1090
  source "${CONDA_SH}"
  conda activate base
else
  export PATH="${INSTALL_DIR}/bin:${PATH}"
  echo "Warning: ${CONDA_SH} not found. Added ${INSTALL_DIR}/bin to PATH."
fi

# --- configure channels + solver (idempotent) ---
conda config --add channels conda-forge >/dev/null 2>&1 || true
conda config --set channel_priority strict
# libmamba (faster/more robust solver)
if ! conda list -n base | grep -q '^conda-libmamba-solver'; then
  conda install -y -n base conda-libmamba-solver
fi
conda config --set solver libmamba

echo "Active conda env: $(conda info --json | tr -d '\n' | sed -n 's/.*"active_prefix_name":"\([^"]*\)".*/\1/p')"
echo


# --- activation hooks to expose libclang at runtime ---
# Ensure LIBCLANG_PATH + LD_LIBRARY_PATH point to this env's lib, so clang.cindex finds the right libclang.so.
ACTIVATE_D="${CONDA_PREFIX}/etc/conda/activate.d"
DEACTIVATE_D="${CONDA_PREFIX}/etc/conda/deactivate.d"
mkdir -p "${ACTIVATE_D}" "${DEACTIVATE_D}"

# Friendly reminder if script wasn’t sourced
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "NOTE: You executed the script. Env activation only applied to this process."
  echo "To get the env in your current shell, run:  source scripts/install_conda.sh"
fi
