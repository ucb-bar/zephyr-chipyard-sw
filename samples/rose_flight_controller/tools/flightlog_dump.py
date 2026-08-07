#!/usr/bin/env python
"""
Dump + analyze the rose_flight_controller on-board flash flight log.

The firmware must be flashed with the DUMP build, which prints the whole log as CSV
once at boot then idles:
    west build -b esp32c6_devkitc/esp32c6/hpcore samples/rose_flight_controller -d build_dump -- \
        -DROSE_USE_PID=1 -DEXTRA_CPPFLAGS="-DROSE_FLIGHTLOG_DUMP=1"
    west flash -d build_dump

Then run this. It resets the board via DTR/RTS *while attached* (the dump prints ~0.5 s
after boot, before the host would re-enumerate, so we must be listening across the reset),
captures the CSV, and prints a per-flight analysis. The log is append-mode and survives
resets/reflashes, so it accumulates every flight until erased (-DROSE_FLIGHTLOG_ERASE=1).

CSV columns (see flightlog.h flight_rec):
    t_ms, roll_mrad, pitch_mrad, yaw_mrad, z_mm, vz_mmps, duty0..3 (0.5% units), flags

Usage:
    python flightlog_dump.py [port] [out.csv]
    python flightlog_dump.py /dev/ttyACM0 flightlog.csv
"""
import sys, time, re, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
OUT  = sys.argv[2] if len(sys.argv) > 2 else "flightlog.csv"
CAPTURE_S = 15

# motor index -> corner (see main.cpp): 0=front-right 1=rear-right (RIGHT pair);
#                                       2=rear-left  3=front-left (LEFT pair)
def capture():
    s = serial.Serial(PORT, 115200, timeout=0.3)
    # reset the ESP32-C6 via the CDC control lines so the one-shot dump prints while we listen
    s.setDTR(False); s.setRTS(True); time.sleep(0.15)
    s.setRTS(False); time.sleep(0.05)
    end = time.time() + CAPTURE_S
    buf = b""; lines = []
    while time.time() < end:
        try:
            d = s.read(1024)
        except Exception:
            try: s = serial.Serial(PORT, 115200, timeout=0.3)
            except Exception: time.sleep(0.2)
            continue
        if d:
            buf += d
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                lines.append(ln.decode("utf-8", "replace").rstrip())
    return lines


def parse(lines):
    rows = []
    for l in lines:
        if not l[:1].isdigit():
            continue
        p = l.split(",")
        if len(p) < 11:
            continue
        try:
            rows.append([int(x) for x in p[:11]])
        except ValueError:
            pass
    return rows


def segment(rows):
    """Split into flights/boots: t_ms is k_uptime, so it resets (drops) each boot."""
    flights, cur, last = [], [], -1
    for r in rows:
        if r[0] < last - 500:
            if cur: flights.append(cur)
            cur = []
        cur.append(r); last = r[0]
    if cur: flights.append(cur)
    return flights


def analyze(flights):
    for i, f in enumerate(flights):
        air = [r for r in f if r[4] > 100]   # z_mm > 100 = airborne
        if len(air) < 15:
            continue
        n = len(air)
        m = lambda idx: sum(r[idx] for r in air) / n
        d = [m(6), m(7), m(8), m(9)]
        right, left = (d[0] + d[1]) / 2, (d[2] + d[3]) / 2
        fwd, aft = (d[0] + d[3]) / 2, (d[1] + d[2]) / 2   # front pair vs rear pair
        zmax = max(r[4] for r in air)
        dur = air[-1][0] - air[0][0]
        print(f"flight {i}: {n} airborne recs, {dur} ms, zmax={zmax} mm")
        print(f"   est roll={m(1):.0f} mrad  pitch={m(2):.0f} mrad")
        print(f"   duty(0.5%): FR={d[0]:.0f} RR={d[1]:.0f} RL={d[2]:.0f} FL={d[3]:.0f}")
        print(f"   roll asym RIGHT-LEFT={right-left:+.0f}   pitch asym FRONT-REAR={fwd-aft:+.0f}")


if __name__ == "__main__":
    lines = capture()
    with open(OUT, "w") as fh:
        fh.write("\n".join(lines))
    rows = parse(lines)
    flights = segment(rows)
    print(f"captured {len(lines)} lines -> {OUT}; {len(rows)} records in {len(flights)} boots\n")
    analyze(flights)
