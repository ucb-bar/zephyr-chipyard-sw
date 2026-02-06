#!/usr/bin/env python3

import serial
import time

def main():
    # Serial port configuration
    port = '/dev/ttyUSB0'
    baudrate = 9600
    timeout = 1  # seconds

    try:
        # Open serial port
        ser = serial.Serial(port, baudrate, timeout=timeout, dsrdtr=False)
        ser.dtr = True
        ser.rts = True
        print(f"Connected to {port} at {baudrate} baud")

        # Wait a bit for connection (Arduino may reset and take time to boot)
        time.sleep(5)

        # Send "all"
        ser.write('all\n'.encode('utf-8'))
        ser.flush()
        print("Sent: all")

        # Wait for "Completed: all"
        while True:
            line = ser.readline().decode('utf-8').strip()
            print(f"Received: {repr(line)}")  # Print all received data
            if line:
                print(f"Stripped: {line}")
                if line == "Completed: all":
                    print("Received completion signal. Exiting.")
                    break
            time.sleep(0.1)  # Small delay to avoid busy waiting

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.dtr = False  # Prevent Arduino reset on close
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()