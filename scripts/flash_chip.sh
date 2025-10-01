#!/bin/bash

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
python3 -m pyuartsi --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AV0KRXBH-if00-port0 --elf "$SCRIPT_DIR/../build/zephyr/zephyr.elf" --load --selfcheck --hart0_msip