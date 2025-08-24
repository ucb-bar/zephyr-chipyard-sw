#!/usr/bin/env bash
set -euo pipefail

# ---- Config (override via env) ----
REPLAY_FREQ="${REPLAY_FREQ:-800}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
SCRIPT_PATH="${SCRIPT_PATH:-samples/drone_control/scripts/replay_drone_log.py}"
INPUT_ROOT="${INPUT_ROOT:-data}"
OUTPUT_ROOT="${OUTPUT_ROOT:-results/video}"
XVFB_RUN_STR="${XVFB_RUN:-xvfb-run -a}"

# Optional extras (only used if set):
#   SKIP_EXISTING=1    -> skip if output file already exists

# Parse XVFB cmd into an array safely
read -r -a XVFB_ARR <<< "${XVFB_RUN_STR}"

process_dir() {
  local subdir="$1"  # "rvv" or "scalar"
  local in_dir="${INPUT_ROOT}/${subdir}"
  local out_dir="${OUTPUT_ROOT}/${subdir}"

  if [[ ! -d "$in_dir" ]]; then
    echo "[WARN] Input directory not found: $in_dir — skipping"
    return 0
  fi

  mkdir -p "$out_dir"

  # Find all *.npy logs recursively
  while IFS= read -r -d '' in_file; do
    local base
    base="$(basename "$in_file")"
    local out_file="${out_dir}/${base%.npy}.mp4"

    if [[ -n "${SKIP_EXISTING:-}" && -f "$out_file" ]]; then
      echo "[SKIP] $out_file (exists)"
      continue
    fi

    echo "[RUN ] ${subdir}: ${base} -> ${out_file}"

    # Ensure parent dir exists (defense in depth; script also ensures it)
    mkdir -p "$(dirname "$out_file")"

    # Build command
    cmd=( "${XVFB_ARR[@]}"
          "${PYTHON_BIN}" "${SCRIPT_PATH}"
          "$in_file"
          --replay_freq "$REPLAY_FREQ"
          --video_out "$out_file"
        )


    # Execute
    "${cmd[@]}"
  done < <(find "$in_dir" -type f -name '*.npy' -print0 | sort -z)
}

process_dir "rvv"
process_dir "scalar"
echo "[DONE] All videos written under ${OUTPUT_ROOT}/"
