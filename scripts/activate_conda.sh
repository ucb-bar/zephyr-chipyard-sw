scripts/activate_conda.sh#!/usr/bin/env bash
# scripts/activate_conda.sh
# Safe conda activation script that can be sourced
# Usage: source scripts/activate_conda.sh

# Resolve repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Find conda.sh
CONDA_SH=""
if [[ -f "${REPO_ROOT}/tools/miniforge3/etc/profile.d/conda.sh" ]]; then
  CONDA_SH="${REPO_ROOT}/tools/miniforge3/etc/profile.d/conda.sh"
elif [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  CONDA_SH="${HOME}/miniforge3/etc/profile.d/conda.sh"
elif [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
  CONDA_SH="${HOME}/anaconda3/etc/profile.d/conda.sh"
fi

if [[ -z "${CONDA_SH}" ]]; then
  echo "ERROR: conda.sh not found. Please run 'source scripts/install_conda.sh' first." >&2
  return 1 2>/dev/null || exit 1
fi

# Source conda.sh
if ! source "${CONDA_SH}" 2>/dev/null; then
  echo "ERROR: Failed to source conda.sh from ${CONDA_SH}" >&2
  return 1 2>/dev/null || exit 1
fi

# Verify conda is available
if ! command -v conda >/dev/null 2>&1; then
  echo "ERROR: conda command not found after sourcing conda.sh" >&2
  return 1 2>/dev/null || exit 1
fi

# Activate zephyr environment
if conda activate zephyr 2>/dev/null; then
  echo "✓ Conda environment 'zephyr' activated successfully"
  return 0
else
  echo "ERROR: Failed to activate conda environment 'zephyr'" >&2
  echo "Try running: bash scripts/debug_conda.sh to diagnose the issue" >&2
  return 1 2>/dev/null || exit 1
fi

