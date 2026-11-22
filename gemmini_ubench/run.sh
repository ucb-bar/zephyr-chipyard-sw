#!/usr/bin/env bash
# Load the microbench ELF onto the FPGA Rocket via debug JTAG (FTDI USB 3-6)
# and capture the console (uart0 -> /dev/ttyUSB1 @115200). Usage: ./run.sh [secs]
#
# SHARED BENCH: before running, check `ps -ef|grep -E "openocd|vivado"` and
# `fuser /dev/ttyUSB1`; wait if busy. Does NOT reflash SPI (SRAM load only).
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
SECS="${1:-60}"
ELF="$REPO/build_gemmini_ubench/zephyr/zephyr.elf"
OCD_CFG="$REPO/../riskybird/scripts/openocd/arty200t_rocket.cfg"
CON=/dev/ttyUSB1
CONLOG="$HERE/con_ubench.log"
echo "=== ELF: $ELF ==="; ls -la "$ELF"
stty -F "$CON" 115200 raw -echo -echoe -echok -ixon -crtscts 2>&1 || true
: > "$CONLOG"
timeout "$SECS" cat "$CON" > "$CONLOG" 2>/dev/null &
CATPID=$!
sleep 1
echo "=== openocd load + resume (pinned FTDI 3-6) ==="
FTDI_LOCATION=3-6 FTDI_SPEED=1000 timeout 120 openocd -f "$OCD_CFG" \
  -c "init; halt; reg mstatus 0x0; reg mie 0x0; load_image $ELF; resume 0x80000000; shutdown" 2>&1 \
  | grep -iE "halted|Examined|misa|hart|Error|Warn|Loading|wrote|shutdown|downloaded|Failed" | head -20
echo "=== waiting for console ($SECS s) ==="
wait $CATPID 2>/dev/null
echo "=== CONSOLE ($CONLOG) ==="
cat "$CONLOG"
