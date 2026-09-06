#!/usr/bin/env python
"""
Capture the flight-controller telemetry stream to stdout + a file, with timestamps.

Two sources, same line-based output:
  * SERIAL (default) -- the USB-Serial/JTAG console. Reconnects automatically across USB
    re-enumeration (e.g. after a flash/reset), so you can start it and then reset the board.
  * UDP (--udp) -- the WiFi SoftAP telemetry DOWNLINK (docs/TELEMETRY_PLAN.md phase 2). Join the
    drone's AP ("riskybird-<id>"), then read the 50 Hz UDP datagrams on port 14550. Each datagram
    is one telemetry line in the SAME v1 text format as the serial console
    ("flight_controller: it=... roll=... z=... tofv=... u=[...]"), so downstream parsers are
    unchanged -- this is the plan's one-function swap (open_noreset() -> a UDP recvfrom).

Usage:
    python serial_capture.py [seconds] [port] [out.txt]              # serial
    python serial_capture.py 60 /dev/ttyACM0 run.log                 # serial
    python serial_capture.py --udp [seconds] [out.txt]              # WiFi downlink (:14550)
    python serial_capture.py --udp 14550 120 run.log                # explicit UDP port
"""
import sys, time


def open_noreset(port):
    """Open WITHOUT asserting DTR/RTS. On the ESP32-C6's native USB-Serial/JTAG those lines map to
    BOOT/EN, so pyserial's default open (DTR/RTS asserted) RESETS the chip into ROM download mode --
    the port then goes silent because the app isn't running. Deassert them before opening."""
    import serial  # lazy: not needed (and not required to be installed) for the --udp path
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.3
    s.dtr = False
    s.rts = False
    s.open()
    return s


def parse_args(argv):
    """Support the positional serial form and an optional `--udp [port]` flag, without breaking
    the existing serial invocation."""
    udp = False
    udp_port = 14550
    if "--udp" in argv:
        udp = True
        i = argv.index("--udp")
        argv.pop(i)
        # optional numeric port immediately after --udp
        if i < len(argv) and argv[i].isdigit():
            udp_port = int(argv.pop(i))
    return udp, udp_port, argv


def emit(line, t0, out):
    rec = f"[{time.time()-t0:6.2f}] {line.decode('utf-8', 'replace').rstrip()}"
    print(rec, flush=True)
    if out:
        out.write(rec + "\n"); out.flush()


def run_serial(dur, port, out):
    t0 = time.time()
    buf = b""
    s = None
    while time.time() - t0 < dur:
        try:
            if s is None:
                s = open_noreset(port)
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
            emit(line, t0, out)


def run_udp(dur, udp_port, out):
    """Read the WiFi telemetry downlink. Bind 0.0.0.0:<port> and recvfrom; each datagram is one
    line (still split on '\\n' in case datagrams are ever coalesced)."""
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", udp_port))
    sock.settimeout(0.5)
    print(f"# listening for UDP telemetry on 0.0.0.0:{udp_port} "
          f"(join the 'riskybird-<id>' SoftAP first)", flush=True)
    t0 = time.time()
    while time.time() - t0 < dur:
        try:
            d, _addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except Exception:
            time.sleep(0.2)
            continue
        if not d:
            continue
        # The firmware sends one complete telemetry line per datagram (ending in '\n'); split in
        # case a datagram ever carries more than one line, and drop empty trailing fragments.
        for line in d.split(b"\n"):
            if line.strip():
                emit(line, t0, out)


udp, udp_port, argv = parse_args(sys.argv[1:])
DUR = float(argv[0]) if len(argv) > 0 else 60.0
if udp:
    OUT = argv[1] if len(argv) > 1 else None
else:
    PORT = argv[1] if len(argv) > 1 else "/dev/ttyACM0"
    OUT = argv[2] if len(argv) > 2 else None

t0 = time.time()
out = open(OUT, "w") if OUT else None
if udp:
    run_udp(DUR, udp_port, out)
else:
    run_serial(DUR, PORT, out)
print(f"[done {time.time()-t0:.1f}s]")
