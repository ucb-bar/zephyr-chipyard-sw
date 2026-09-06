#!/usr/bin/env bash
# JTAG-load + run the integrated FC + DroNet ELF on the ALREADY-FLASHED riskybird
# v3 FPGA combined core (RocketArty200TDroneGemminiSaturnFp16At35Config). NO
# reflash -- the At35 bitstream is already in SPI flash / SRAM on the drone board.
#
# Console prints BOTH:
#   * the FC's ROSE_PROFILE loop-rate lines  (PID Hz + per-phase us) and it= count
#   * the DRONET: ... fps / err= lines from src/dronet_thread.c
# so one capture gives the concurrent PID-Hz-vs-DroNet-fps Phase 1 numbers.
#
# BENCH GOTCHAS (see memory riskybird-v3-fc-on-fpga / -fpga-bringup):
#   * The two FTDIs share 0403:6010/6011 and the ttyUSB numbering RESHUFFLES on
#     every replug. Re-probe `lsusb -t` for the Rocket-debug FTDI's USB path and
#     which ttyUSB is its channel-1 console BEFORE trusting the defaults below.
#   * cfg has `reset_config none`, so loading over a RUNNING instance needs
#     `reg mstatus 0; reg mie 0` before resume (done here) or a stale IRQ hangs boot.
#   * Do NOT run this while another JTAG session (e.g. the DroNet-perf agent) is
#     live -- not simultaneous. Check: ps aux | grep -E 'openocd|west build'.
set -uo pipefail

WT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RB="${RB:-/home/cobble/Tools/riskybird}"                 # openocd cfg lives here
CFG="${CFG:-${RB}/scripts/openocd/arty200t_rocket.cfg}"
ELF="${ELF:-${WT}/build_fc_dronet/zephyr/zephyr.elf}"
ENTRY="${ENTRY:-0x80000000}"
OPENOCD="${OPENOCD:-openocd}"
FTDI_LOCATION="${FTDI_LOCATION:-}"                        # e.g. 3-1 / 3-6; disambiguate the two FTDIs
CONSOLE_DEV="${CONSOLE_DEV:-/dev/ttyUSB1}"
CONSOLE_BAUD="${CONSOLE_BAUD:-115200}"
CAPTURE_SECS="${CAPTURE_SECS:-40}"                        # how long to stream the console
OUT="${OUT:-${WT}/fc_dronet_run.log}"

echo "=== bench pre-flight ==="
ps aux 2>/dev/null | grep -E 'openocd|west build|openFPGALoader' | grep -v grep && {
  echo "!! another bench/JTAG job is running -- ABORTING (you are lower priority)."; exit 3; }
echo "-- FTDI topology (pick the Rocket-debug FTDI path + its console ttyUSB) --"
lsusb -t 2>/dev/null | grep -iE "ftdi|0403|ttyUSB" || lsusb | grep -i 0403
[ -n "${FTDI_LOCATION}" ] && export FTDI_LOCATION
[ -f "$ELF" ] || { echo "ELF not found: $ELF (build first)"; exit 1; }
[ -f "$CFG" ] || { echo "openocd cfg not found: $CFG"; exit 1; }

# Clear stale interrupt CSRs before resume (reset_config none) then load + run.
OOCMD="init; halt; reg mstatus 0; reg mie 0;"
OOCMD="${OOCMD} load_image {${ELF}}; echo {--- loaded, resuming @ ${ENTRY} ---};"
OOCMD="${OOCMD} resume ${ENTRY}; shutdown"

echo "=== load ${ELF} -> Rocket @ ${ENTRY} (FTDI_LOCATION='${FTDI_LOCATION:-auto}') ==="
# Open the console BEFORE resume so the boot banner + first samples are captured.
stty -F "$CONSOLE_DEV" "$CONSOLE_BAUD" raw -echo 2>/dev/null || echo "(warn: stty $CONSOLE_DEV failed)"
( timeout "${CAPTURE_SECS}" cat "$CONSOLE_DEV" | tee "$OUT" ) & CATPID=$!
"$OPENOCD" -f "$CFG" -c "$OOCMD"
wait "$CATPID" 2>/dev/null || true
echo "=== capture -> ${OUT} ==="
echo "--- PID loop rate (FC ROSE_PROFILE) ---"; grep -iE "PROFILE|loop|it=" "$OUT" | tail -5
echo "--- DroNet fps under contention ---";      grep -iE "DRONET" "$OUT" | tail -5
