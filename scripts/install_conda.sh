#!/usr/bin/env bash
# scripts/install_conda.sh
# Install Miniforge3 (Linux/x86_64) into ./tools/miniforge3 non-interactively,
# then configure solver and install clang (lib + python bindings) + dotmap.
# Re-runnable and idempotent.
scripts/install_conda.sh#
# NOTE: This script is meant to be SOURCED, not executed.
# When sourced, we must be careful not to set options that affect the parent shell.

# Save current shell options
if [[ -o errexit ]]; then
  ORIG_ERREXIT=true
else
  ORIG_ERREXIT=false
fi

# Only set -e if script is executed (not sourced)
# When sourced, we don't want to exit the parent shell on errors
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  # Script is being executed, safe to use set -e
  set -euo pipefail
else
  # Script is being sourced, only set safe options
  set -uo pipefail
  # Don't set -e as it would cause parent shell to exit on errors
fi

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

# --- activate zephyr environment if it exists ---
if conda env list | grep -q "^zephyr "; then
  echo "Activating zephyr conda environment..."
  if conda activate zephyr 2>/dev/null; then
    echo "✓ zephyr environment activated"
  else
    echo "Note: Could not activate zephyr environment (may need to be created first)"
    echo "      You can activate it manually with: conda activate zephyr"
  fi
else
  echo "Note: zephyr environment does not exist yet. It will be created by install_submodules.sh"
fi
echo

# --- activation hooks to expose libclang at runtime ---
# Ensure LIBCLANG_PATH + LD_LIBRARY_PATH point to this env's lib, so clang.cindex finds the right libclang.so.
ACTIVATE_D="${CONDA_PREFIX}/etc/conda/activate.d"
DEACTIVATE_D="${CONDA_PREFIX}/etc/conda/deactivate.d"
mkdir -p "${ACTIVATE_D}" "${DEACTIVATE_D}"

# Restore original errexit setting if script was sourced
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Script was sourced - restore original errexit setting
  if [[ "$ORIG_ERREXIT" == "true" ]]; then
    set -e
  else
    set +e
  fi
else
  # Script was executed
  echo "NOTE: You executed the script. Env activation only applied to this process."
  echo "To get the env in your current shell, run:  source scripts/install_conda.sh"
fi
