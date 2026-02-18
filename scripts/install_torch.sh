#!/usr/bin/env bash
# scripts/install_torch.sh
# Install Executorch dependencies and setup torch-related submodules.
# This script should be EXECUTED (not sourced) to avoid shell exit on errors.

# Ensure script is not being sourced
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  echo "ERROR: This script must be executed, not sourced. Use: bash scripts/install_torch.sh" >&2
  return 1 2>/dev/null || exit 1
fi

# Use set -u to catch undefined variables, but NOT set -e to avoid killing parent shell
set -u

# Resolve repo root (script may be called from anywhere)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_FILE="${REPO_ROOT}/.install_torch.log"

# Logging functions
log() { 
  local msg="[$(date +%H:%M:%S)] $*"
  printf "\n\033[1;32m%s\033[0m\n" "$msg"
  echo "$msg" >> "${LOG_FILE}" 2>/dev/null || true
}

log_error() {
  local msg="[$(date +%H:%M:%S)] ERROR: $*"
  printf "\n\033[1;31m%s\033[0m\n" "$msg" >&2
  echo "$msg" >> "${LOG_FILE}" 2>/dev/null || true
}

# Safe exit function that won't kill parent shell
safe_exit() {
  local exit_code="${1:-0}"
  log "Script exiting with code: $exit_code"
  exit "$exit_code"
}

# Error handling wrapper - explicitly check command success
run_cmd() {
  local cmd="$*"
  log "Running: $cmd"
  if ! eval "$cmd" >> "${LOG_FILE}" 2>&1; then
    log_error "Command failed: $cmd"
    log_error "Check ${LOG_FILE} for details"
    safe_exit 1
  fi
}

# Main installation function
main() {
  # Initialize log file
  echo "=== Torch installation started at $(date) ===" > "${LOG_FILE}"
  log "Starting torch installation. Log file: ${LOG_FILE}"
  
  # Track warnings
  WARNINGS=()

  # Change to repo root
  if ! cd "${REPO_ROOT}"; then
    log_error "Failed to cd to repo root: ${REPO_ROOT}"
    safe_exit 1
  fi

  # Initialize conda if not already initialized
  if [[ -z "${CONDA_DEFAULT_ENV:-}" ]]; then
    # Try to find conda.sh in common locations
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

  # Verify conda is available
  if ! command -v conda >/dev/null 2>&1; then
    log_error "conda not found in PATH. Please run 'source scripts/install_conda.sh' first."
    safe_exit 1
  fi

  # Verify zephyr conda environment exists
  if ! conda env list 2>/dev/null | grep -q "^zephyr "; then
    log_error "Conda environment 'zephyr' not found. Please run 'bash scripts/install_submodules.sh' first."
    safe_exit 1
  fi

  # Use conda run instead of activate - safer for non-interactive scripts
  log "Using conda run to execute commands in zephyr environment..."
  CONDA_RUN="conda run -n zephyr --no-capture-output"

  # Verify we're on torch-bump-testing branch (user must checkout manually)
  log "Verifying torch-bump-testing branch..."
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "")
  if [[ "${CURRENT_BRANCH}" != "torch-bump-testing" ]]; then
    log_error "Not on torch-bump-testing branch. Current branch: '${CURRENT_BRANCH}'"
    log_error "Please checkout the torch-bump-testing branch first:"
    log_error "  git fetch origin"
    log_error "  git checkout torch-bump-testing"
    log_error "Then run this script again."
    safe_exit 1
  fi
  log "Confirmed on torch-bump-testing branch"

  # Update submodules to match the commits referenced by torch-bump-testing branch
  log "Updating submodules to match torch-bump-testing branch commits..."
  run_cmd "git submodule update --init --recursive zephyr_ws/zephyr third-party/executorch third-party/XNNPACK"

  # Install executorch dependencies
  log "Installing executorch dependencies (executorch, zstd, torchvision)..."
  run_cmd "${CONDA_RUN} python -m pip install executorch==1.0.1 zstd torchvision"

  # Setup Executorch repos (sync and update submodules)
  log "Setting up Executorch repos (syncing and updating submodules)..."
  if [ -d "third-party/executorch" ]; then
    EXECUTORCH_DIR="${REPO_ROOT}/third-party/executorch"
    if ! cd "${EXECUTORCH_DIR}"; then
      log_error "Failed to cd to ${EXECUTORCH_DIR}"
      safe_exit 1
    fi
    run_cmd "git submodule sync"
    run_cmd "git submodule update --init --recursive"
    if ! cd "${REPO_ROOT}"; then
      log_error "Failed to return to repo root"
      safe_exit 1
    fi
  else
    log_error "third-party/executorch directory not found"
    safe_exit 1
  fi

  # Install additional executorch Python dependencies (optional - only needed if building Python bindings or using Python tools)
  log "Installing additional executorch Python dependencies (optional)..."
  if [ -f "third-party/executorch/install_requirements.sh" ]; then
    log "Running install_requirements.sh..."
    if ! eval "${CONDA_RUN} bash third-party/executorch/install_requirements.sh" >> "${LOG_FILE}" 2>&1; then
      WARNINGS+=("executorch install_requirements.sh failed. This is optional and may not be needed for basic usage.")
      log_error "executorch install_requirements.sh failed - continuing with installation"
      log "You can try running it manually later: conda run -n zephyr bash third-party/executorch/install_requirements.sh"
    else
      log "executorch install_requirements.sh completed successfully"
    fi
  else
    log "install_requirements.sh not found in third-party/executorch, skipping (this is optional)"
  fi

  log "Torch installation complete!"
  
  # Show warnings if any
  if [[ ${#WARNINGS[@]} -gt 0 ]]; then
    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo "⚠️  WARNINGS:"
    echo "═══════════════════════════════════════════════════════════════"
    for warning in "${WARNINGS[@]}"; do
      echo "  • $warning"
    done
    echo "═══════════════════════════════════════════════════════════════"
    echo ""
  fi
  
  if [[ ${#WARNINGS[@]} -gt 0 ]]; then
    echo "=== Torch installation completed with warnings at $(date) ===" >> "${LOG_FILE}"
    log "Torch installation completed with ${#WARNINGS[@]} warning(s)"
  else
    echo "=== Torch installation completed successfully at $(date) ===" >> "${LOG_FILE}"
  fi
  
  safe_exit 0
}

# Run main function and catch any unexpected errors
if ! main "$@"; then
  log_error "Main function returned non-zero exit code"
  safe_exit 1
fi
