#!/usr/bin/env bash
# End-to-end runner: spike + HDLC proxy + ros-jazzy-micro-ros-agent.
#
# Usage:
#   tools/microros/run_with_agent.sh [path/to/zephyr.elf]
#
# Defaults to build/zephyr/zephyr.elf. Requires ROS 2 Jazzy + the agent
# installed on the host (run tools/microros/install_agent.sh once).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ELF="${1:-${REPO_ROOT}/build/zephyr/zephyr.elf}"

if [[ ! -f "$ELF" ]]; then
    echo "ERROR: ELF not found: $ELF" >&2
    echo "Build it first:  west build -p -b spike_riscv64 samples/micro_ros_multinode" >&2
    exit 1
fi

# Source ROS 2 Jazzy if it exists; the agent needs it.
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
else
    echo "WARNING: /opt/ros/jazzy/setup.bash not found." >&2
    echo "Install with: bash tools/microros/install_agent.sh" >&2
fi

if ! command -v micro-ros-agent >/dev/null; then
    echo "ERROR: micro-ros-agent not on PATH after sourcing /opt/ros/jazzy." >&2
    exit 1
fi

if ! command -v spike >/dev/null; then
    echo "ERROR: spike not on PATH. Activate the conda zephyr env first:" >&2
    echo "  source ${REPO_ROOT}/tools/miniforge3/etc/profile.d/conda.sh && conda activate zephyr" >&2
    exit 1
fi

echo "[run_with_agent] elf:   $ELF"
echo "[run_with_agent] agent: $(command -v micro-ros-agent)"
echo "[run_with_agent] spike: $(command -v spike)"
echo

exec python3 "${REPO_ROOT}/tools/microros/htif_proxy.py" \
    --elf "$ELF" \
    --spike "$(command -v spike)" \
    --agent "$(command -v micro-ros-agent)" \
    --agent-verbosity 4
