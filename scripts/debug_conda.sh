#!/usr/bin/env bash
# scripts/debug_conda.sh
# Debug script to safely test conda activation without crashing the terminal

# Ensure script is not being sourced
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  echo "ERROR: This script must be executed, not sourced. Use: bash scripts/debug_conda.sh" >&2
  return 1 2>/dev/null || exit 1
fi

set -u

# Resolve repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_FILE="${REPO_ROOT}/.debug_conda.log"

echo "=== Conda Debug Session started at $(date) ===" | tee "${LOG_FILE}"
echo ""

# Step 1: Check if conda.sh exists
echo "Step 1: Checking for conda.sh..."
CONDA_SH=""
if [[ -f "${REPO_ROOT}/tools/miniforge3/etc/profile.d/conda.sh" ]]; then
  CONDA_SH="${REPO_ROOT}/tools/miniforge3/etc/profile.d/conda.sh"
  echo "  ✓ Found: ${CONDA_SH}"
elif [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  CONDA_SH="${HOME}/miniforge3/etc/profile.d/conda.sh"
  echo "  ✓ Found: ${CONDA_SH}"
elif [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
  CONDA_SH="${HOME}/anaconda3/etc/profile.d/conda.sh"
  echo "  ✓ Found: ${CONDA_SH}"
else
  echo "  ✗ ERROR: conda.sh not found in expected locations"
  exit 1
fi

# Step 2: Source conda.sh with error handling
echo ""
echo "Step 2: Sourcing conda.sh..."
if source "${CONDA_SH}" 2>&1 | tee -a "${LOG_FILE}"; then
  echo "  ✓ conda.sh sourced successfully"
else
  echo "  ✗ ERROR: Failed to source conda.sh"
  echo "  Check ${LOG_FILE} for details"
  exit 1
fi

# Step 3: Check if conda command is available
echo ""
echo "Step 3: Checking conda command..."
if command -v conda >/dev/null 2>&1; then
  echo "  ✓ conda command found: $(which conda)"
  conda --version 2>&1 | tee -a "${LOG_FILE}"
else
  echo "  ✗ ERROR: conda command not found in PATH"
  echo "  PATH: ${PATH}"
  exit 1
fi

# Step 4: Check conda info
echo ""
echo "Step 4: Checking conda info..."
if conda info 2>&1 | tee -a "${LOG_FILE}"; then
  echo "  ✓ conda info retrieved successfully"
else
  echo "  ✗ ERROR: Failed to get conda info"
  exit 1
fi

# Step 5: List environments
echo ""
echo "Step 5: Listing conda environments..."
if conda env list 2>&1 | tee -a "${LOG_FILE}"; then
  echo "  ✓ Environment list retrieved"
else
  echo "  ✗ ERROR: Failed to list environments"
  exit 1
fi

# Step 6: Check if zephyr environment exists
echo ""
echo "Step 6: Checking for 'zephyr' environment..."
if conda env list | grep -q "^zephyr "; then
  echo "  ✓ zephyr environment exists"
else
  echo "  ✗ WARNING: zephyr environment not found"
  echo "  Available environments:"
  conda env list
  exit 1
fi

# Step 7: Try to get environment info without activating
echo ""
echo "Step 7: Getting zephyr environment info (without activating)..."
if conda env list | grep "^zephyr " | tee -a "${LOG_FILE}"; then
  echo "  ✓ Environment info retrieved"
else
  echo "  ✗ ERROR: Failed to get environment info"
  exit 1
fi

# Step 8: Test conda run (safer than activate)
echo ""
echo "Step 8: Testing conda run (safer alternative to activate)..."
if conda run -n zephyr python --version 2>&1 | tee -a "${LOG_FILE}"; then
  echo "  ✓ conda run works successfully"
else
  echo "  ✗ ERROR: conda run failed"
  exit 1
fi

# Step 9: Check environment variables that might cause issues
echo ""
echo "Step 9: Checking environment variables..."
echo "  CONDA_DEFAULT_ENV: ${CONDA_DEFAULT_ENV:-<not set>}"
echo "  CONDA_PREFIX: ${CONDA_PREFIX:-<not set>}"
echo "  CONDA_PROMPT_MODIFIER: ${CONDA_PROMPT_MODIFIER:-<not set>}"
echo "  PS1: ${PS1:-<not set>}"

# Step 10: Try to activate in a subshell (safest test)
echo ""
echo "Step 10: Testing activation in a subshell..."
echo "  Running: bash -c 'source ${CONDA_SH} && conda activate zephyr && echo SUCCESS'"
if bash -c "source ${CONDA_SH} && conda activate zephyr && echo SUCCESS" 2>&1 | tee -a "${LOG_FILE}"; then
  echo "  ✓ Activation in subshell succeeded"
else
  echo "  ✗ ERROR: Activation in subshell failed"
  echo "  This indicates a problem with the environment"
  exit 1
fi

echo ""
echo "=== Debug session completed at $(date) ===" | tee -a "${LOG_FILE}"
echo ""
echo "If all steps passed, the issue might be with your shell configuration."
echo "Try activating manually with:"
echo "  source ${CONDA_SH}"
echo "  conda activate zephyr"
echo ""
echo "If that crashes, check your shell's PS1 or other prompt settings."
echo "Full log saved to: ${LOG_FILE}"

