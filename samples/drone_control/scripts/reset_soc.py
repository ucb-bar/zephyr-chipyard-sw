#!/usr/bin/env python3
import argparse
import sys
import time

import serial  # pip install pyserial

def main():
    ap = argparse.ArgumentParser(description="Send 'R', wait for 'D' over UART.")
    ap.add_argument("-p", "--port", default="/dev/ttyUSB3", help="Serial port (default: /dev/ttyUSB0)")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    ap.add_argument("-t", "--timeout", type=float, default=5.0, help="Timeout in seconds waiting for 'D' (default: 5)")
    ap.add_argument("--open-delay", type=float, default=0.3, help="Delay after opening port before send (sec)")
    args = ap.parse_args()

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,          # non-blocking reads; we’ll loop up to args.timeout
            rtscts=False,
            dsrdtr=False,
            xonxoff=False,
        )
    except Exception as e:
        print(f"ERROR: could not open {args.port}: {e}", file=sys.stderr)
        return 2

    try:
        # Give the MCU a moment in case opening toggled DTR/RTS and caused a reset.
        time.sleep(args.open_delay)

        # Clear any stale input from previous sessions.
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Send 'R'
        ser.write(b"R")
        ser.flush()

        # Wait for 'D'
        deadline = time.time() + args.timeout
        while time.time() < deadline:
            b = ser.read(1)
            if not b:
                continue
            if b == b"D":
                # Success
                return 0
            # Ignore any other stray bytes

        print("ERROR: timed out waiting for 'D'", file=sys.stderr)
        return 1

    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    sys.exit(main())
