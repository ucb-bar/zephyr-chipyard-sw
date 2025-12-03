#!/usr/bin/env bash
# scripts/install_submodules.sh
# Install submodules, Python dependencies, and initialize west workspace.
# This script should be EXECUTED (not sourced) to avoid shell exit on errors.

# Ensure script is not being sourced
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  echo "ERROR: This script must be executed, not sourced. Use: bash scripts/install_submodules.sh" >&2
  return 1 2>/dev/null || exit 1
fi

# Use set -u to catch undefined variables, but NOT set -e to avoid killing parent shell
set -u

# Resolve repo root (script may be called from anywhere)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_FILE="${REPO_ROOT}/.install_submodules.log"

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
  echo "=== Installation started at $(date) ===" > "${LOG_FILE}"
  log "Starting installation. Log file: ${LOG_FILE}"
  
  # Track warnings
  WARNINGS=()

  # Change to repo root
  if ! cd "${REPO_ROOT}"; then
    log_error "Failed to cd to repo root: ${REPO_ROOT}"
    safe_exit 1
  fi

  # Initialize conda if not already initialized
  # We need conda in PATH, but we'll use conda run instead of activate to avoid issues
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
      # Source conda.sh but don't fail if it has issues - we'll use conda run anyway
      source "${CONDA_SH}" 2>>"${LOG_FILE}" || true
    fi
  fi

  # Verify conda is available
  if ! command -v conda >/dev/null 2>&1; then
    log_error "conda not found in PATH. Please run 'source scripts/install_conda.sh' first."
    safe_exit 1
  fi

  # Create conda environment if it doesn't exist
  log "Setting up conda environment 'zephyr'..."
  if conda env list 2>/dev/null | grep -q "^zephyr "; then
    log "Conda environment 'zephyr' already exists (skipping creation)."
  else
    run_cmd "conda create -yn zephyr python=3.12"
  fi

  # Use conda run instead of activate - safer for non-interactive scripts
  # This avoids issues with conda activate in scripts that can cause shell exits
  log "Using conda run to execute commands in zephyr environment..."
  CONDA_RUN="conda run -n zephyr --no-capture-output"

  # Install the west dependencies
  log "Installing west dependencies..."
  run_cmd "${CONDA_RUN} pip install west pyelftools rich jsonschema"

  # Install toolchain build dependencies (meson and ninja)
  log "Installing toolchain build dependencies (meson, ninja)..."
  run_cmd "${CONDA_RUN} pip install meson ninja"

  # Install the plotting dependencies
  log "Installing plotting dependencies..."
  run_cmd "${CONDA_RUN} pip install matplotlib"

  # Init submodules
  log "Initializing git submodules..."
  run_cmd "git submodule update --init ./tools/pyuartsi"
  run_cmd "git submodule update --init ./tools/gym-pybullet-drones"
  run_cmd "git submodule update --init ./tools/picolibc"
  run_cmd "git submodule update --init ./tools/riscv-gnu-toolchain"
  run_cmd "git submodule update --init ./zephyr_ws/zephyr"

  # Install Python packages from submodules
  log "Installing pyuartsi..."
  run_cmd "${CONDA_RUN} pip install ./tools/pyuartsi"

  # Install gym-pybullet-drones (optional - may fail due to pybullet build issues)
  log "Installing gym-pybullet-drones (optional - may fail if pybullet cannot be built)..."
  if ! eval "${CONDA_RUN} pip install -e ./tools/gym-pybullet-drones" >> "${LOG_FILE}" 2>&1; then
    WARNINGS+=("gym-pybullet-drones installation failed (pybullet build error). This is optional and can be installed later if needed.")
    log_error "gym-pybullet-drones installation failed - continuing with other installations"
    log "You can try installing it later manually: conda run -n zephyr pip install -e ./tools/gym-pybullet-drones"
  else
    log "gym-pybullet-drones installed successfully"
  fi

  # Initialize west workspace
  log "Initializing west workspace..."
  if ! cd zephyr_ws/zephyr; then
    log_error "Failed to cd to zephyr_ws/zephyr"
    safe_exit 1
  fi
  
  # Check if west is already initialized
  WEST_INITIALIZED=false
  if [[ -d .west ]] || [[ -d ../.west ]] || [[ -d ../../.west ]]; then
    # Check if west can list projects (indicates it's initialized)
    if ${CONDA_RUN} west list >/dev/null 2>&1; then
      WEST_INITIALIZED=true
      log "West workspace already initialized, skipping west init"
    fi
  fi
  
  if [[ "$WEST_INITIALIZED" == "false" ]]; then
    if [[ -f .west/config ]]; then
      log "West config file exists, skipping west init"
    else
      log "Initializing west workspace..."
      # Try to initialize, but handle the case where it's already initialized
      if ! eval "${CONDA_RUN} west init -l ." >> "${LOG_FILE}" 2>&1; then
        # Check if the error is because it's already initialized
        if grep -q "already initialized" "${LOG_FILE}" 2>/dev/null; then
          log "West workspace was already initialized (detected from error message), continuing..."
        else
          log_error "Failed to initialize west workspace"
          log_error "Check ${LOG_FILE} for details"
          safe_exit 1
        fi
      fi
    fi
    
    # Set manifest file if not already set
    if [[ ! -f .west/config ]] || ! grep -q "west-riscv.yml" .west/config 2>/dev/null; then
      log "Configuring west manifest file..."
      if ! eval "${CONDA_RUN} west config manifest.file west-riscv.yml" >> "${LOG_FILE}" 2>&1; then
        log_error "Failed to set west manifest file"
        log_error "Check ${LOG_FILE} for details"
        safe_exit 1
      fi
    fi
  fi
  
  # Update west workspace
  log "Updating west workspace..."
  if ! eval "${CONDA_RUN} west update" >> "${LOG_FILE}" 2>&1; then
    # Check if the error is about specific projects failing (non-fatal)
    if grep -q "ERROR: update failed for project" "${LOG_FILE}" 2>/dev/null; then
      WARNINGS+=("Some west projects failed to update (check ${LOG_FILE} for details). This may be non-critical - the workspace may still be usable.")
      log_error "Some west projects failed to update - continuing with installation"
      log "You can try updating specific projects later with: conda run -n zephyr west update <project-name>"
      log "Or update all projects individually if needed"
    else
      log_error "Failed to update west workspace"
      log_error "Check ${LOG_FILE} for details"
      safe_exit 1
    fi
  else
    log "West workspace updated successfully"
  fi
  
  if ! cd "${REPO_ROOT}"; then
    log_error "Failed to return to repo root"
    safe_exit 1
  fi

  # Initialize drone_control submodule
  log "Initializing drone_control submodule..."
  run_cmd "git submodule update --init --recursive samples/drone_control"

  # Check for dtc (device tree compiler) - needed for spike
  log "Checking for dtc (device tree compiler)..."
  if command -v dtc >/dev/null 2>&1; then
    log "dtc found: $(which dtc)"
  else
    # Try to install via conda
    log "dtc not found in PATH, attempting to install via conda..."
    if eval "${CONDA_RUN} conda install -y -c conda-forge dtc" >> "${LOG_FILE}" 2>&1; then
      log "dtc installed successfully via conda"
    else
      WARNINGS+=("dtc (device tree compiler) not found and could not be installed automatically. You may need to install it manually for spike support: sudo apt-get install device-tree-compiler (or equivalent for your system)")
      log_error "dtc installation failed - continuing without it"
      log "You can install dtc manually: sudo apt-get install device-tree-compiler"
    fi
  fi

  # Create data directories
  log "Creating data directories..."
  if ! mkdir -p data; then
    log_error "Failed to create data directory"
    safe_exit 1
  fi
  if ! mkdir -p results; then
    log_error "Failed to create results directory"
    safe_exit 1
  fi

  log "Installation complete!"
  
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
  
  echo ""
  echo "NOTE: If you need USB/serial device access, you may need to add your user"
  echo "      to the plugdev and dialout groups:"
  echo "      sudo usermod -aG plugdev,dialout \"\$USER\""
  echo "      (You'll need to log out and back in for this to take effect)"
  
  if [[ ${#WARNINGS[@]} -gt 0 ]]; then
    echo "=== Installation completed with warnings at $(date) ===" >> "${LOG_FILE}"
    log "Installation completed with ${#WARNINGS[@]} warning(s)"
  else
    echo "=== Installation completed successfully at $(date) ===" >> "${LOG_FILE}"
  fi
  
  safe_exit 0
}

# Run main function and catch any unexpected errors
if ! main "$@"; then
  log_error "Main function returned non-zero exit code"
  safe_exit 1
fi
