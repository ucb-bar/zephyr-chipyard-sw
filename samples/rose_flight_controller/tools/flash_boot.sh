#!/usr/bin/env bash
#
# Flash a build to the riskybird v3 (ESP32-C6) and BOOT THE APP -- no manual power-cycle.
#
# Why this exists: the Zephyr esp32 west runner hardcodes `esptool ... --after hard-reset` (see
# scripts/west_commands/runners/esp32.py). On the C6's built-in USB-Serial/JTAG, the stub-based
# write-flash followed by that RTS reset drops the chip into ROM DOWNLOAD mode ("boot:0x14
# DOWNLOAD / waiting for download") instead of booting -- so `west flash` alone leaves the board
# dark until a manual power-cycle / SW1 tap. A SEPARATE esptool reset afterward boots it cleanly.
# (A UART-bridge board resets fine from RTS, which is why this never happened on earlier hardware.)
#
# Usage: tools/flash_boot.sh [build_dir] [port]
#   build_dir  default: build_flow
#   port       default: /dev/ttyACM0
set -e
BUILD="${1:-build_flow}"
PORT="${2:-/dev/ttyACM0}"

west flash -d "$BUILD"
# Standalone reset -> app boots (flash-id is just a trivial op to carry the --after reset).
esptool --port "$PORT" --before default-reset --after hard-reset flash-id >/dev/null 2>&1 || true
echo "flash_boot: $BUILD flashed + reset -> app booting on $PORT (no power-cycle needed)"
