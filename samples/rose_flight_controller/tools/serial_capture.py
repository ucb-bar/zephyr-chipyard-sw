#!/usr/bin/env python
"""
Capture the flight-controller USB serial stream to stdout + a file, with timestamps.

Reconnects automatically across USB re-enumeration (e.g. after a flash/reset), so you can
start it and then reset the board. Useful for watching live telemetry (the periodic
"flight_controller: it=... roll=... z=... tofv=... u=[...]" lines, side-ToF "walls[...]"
/ "GRID ..." lines, AUTOFLIGHT arming/landing messages, etc.).

Usage:
    python serial_capture.py [seconds] [port] [out.txt]
    python serial_capture.py 60 /dev/ttyACM0 run.log
"""
import sys, time, serial


def open_noreset(port):
    """Open WITHOUT asserting DTR/RTS. On the ESP32-C6's native USB-Serial/JTAG those lines map to
    BOOT/EN, so pyserial's default open (DTR/RTS asserted) RESETS the chip into ROM download mode --
    the port then goes silent because the app isn't running. Deassert them before opening."""
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.3
    s.dtr = False
    s.rts = False
    s.open()
    return s


DUR  = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
PORT = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"
OUT  = sys.argv[3] if len(sys.argv) > 3 else None

t0 = time.time()
out = open(OUT, "w") if OUT else None
buf = b""
s = None
while time.time() - t0 < DUR:
    try:
        if s is None:
            s = open_noreset(PORT)
        d = s.read(256)
    except Exception:
        s = None
        time.sleep(0.2)
        continue
    if not d:
        continue
    buf += d
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        rec = f"[{time.time()-t0:6.2f}] {line.decode('utf-8', 'replace').rstrip()}"
        print(rec, flush=True)
        if out:
            out.write(rec + "\n"); out.flush()
print(f"[done {time.time()-t0:.1f}s]")
