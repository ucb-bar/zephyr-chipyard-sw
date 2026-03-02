#!/bin/bash
# Helper script to find and set ESP32C6 port
# Usage: source set_esp32_port.sh

# Find ESP32C6 device (looks for common ESP32 USB serial patterns)
find_esp32_port() {
    # Try to find by USB vendor/product ID or device description
    for port in /dev/ttyUSB* /dev/ttyACM*; do
        if [ -e "$port" ]; then
            # Check if it's an ESP32 device (you can customize this)
            # Common ESP32 USB-to-serial chips: CP210x, CH340, FTDI
            if udevadm info "$port" 2>/dev/null | grep -qiE "(cp210|ch340|ftdi|silicon|espressif)"; then
                echo "$port"
                return 0
            fi
        fi
    done
    
    # Fallback: use first available USB serial port
    for port in /dev/ttyUSB* /dev/ttyACM*; do
        if [ -e "$port" ]; then
            echo "$port"
            return 0
        fi
    done
    
    return 1
}

PORT=$(find_esp32_port)

if [ -n "$PORT" ]; then
    export ESPTOOL_PORT="$PORT"
    echo "Set ESPTOOL_PORT=$PORT"
    echo "You can now run: west flash && west espressif monitor"
else
    echo "No ESP32 device found. Please connect your device and try again."
    echo "Or manually set: export ESPTOOL_PORT=/dev/ttyUSB0"
fi
