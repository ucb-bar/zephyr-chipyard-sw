#!/usr/bin/env python
"""
Live optical-flow visualizer for the riskybird PMW3901 (pmw3901_test firmware).

Reads the "Motion: deltaX=.. deltaY=.. SQUAL=.. Shutter=.." stream over USB and shows:
  LEFT  - a MOTION TRAIL: deltaX/deltaY integrated into a path. Slide the sensor over a
          textured surface and the trail draws where it "thinks" it moved -> this validates
          the AXES (does +deltaX go right? does sliding forward move +deltaY?) and that the
          magnitudes are sensible. Current point is colored by SQUAL; a live arrow shows the
          instantaneous flow vector. Press 'r' to reset the origin.
  RIGHT - live readout: SQUAL (quality, color-coded), Shutter (exposure), and dX/dY.

Axes are the RAW sensor axes (no remap) so you can map them to the drone frame yourself.

IMPORTANT: opens the port with DTR/RTS DEASSERTED -- on the ESP32-C6 USB-Serial/JTAG those map
to BOOT/EN, so a default open resets the board into ROM download mode (silent). See
tools/serial_capture.py open_noreset().

Run in the conda 'zephyr' env:
    python live_flow_plot.py            # /dev/ttyACM0
    python live_flow_plot.py /dev/ttyACM1
"""
import sys, re, time, threading, collections
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
TRAIL = 4000          # max trail points kept
SQUAL_GOOD, SQUAL_OK = 40, 20   # quality thresholds

LINE_RE = re.compile(
    r"deltaX=\s*(-?\d+).*?deltaY=\s*(-?\d+).*?SQUAL=\s*(\d+).*?Shutter=\s*(\d+)")

lock = threading.Lock()
state = {"dx": 0, "dy": 0, "squal": 0, "shutter": 0, "seq": 0}
trail = collections.deque(maxlen=TRAIL)   # (posX, posY, squal)
pos = {"x": 0.0, "y": 0.0}
reset_flag = {"do": False}
stop = threading.Event()


def open_noreset(port):
    s = serial.Serial()
    s.port = port; s.baudrate = 115200; s.timeout = 0.3
    s.dtr = False; s.rts = False
    s.open()
    return s


def reader():
    s = None
    while not stop.is_set():
        try:
            if s is None:
                s = open_noreset(PORT)
            d = s.read(256)
        except Exception:
            s = None; time.sleep(0.2); continue
        if not d:
            continue
        for line in d.decode("utf-8", "replace").splitlines():
            m = LINE_RE.search(line)
            if not m or "Motion:" not in line:
                continue
            dx, dy, sq, sh = (int(m.group(1)), int(m.group(2)),
                              int(m.group(3)), int(m.group(4)))
            with lock:
                if reset_flag["do"]:
                    pos["x"] = pos["y"] = 0.0; trail.clear(); reset_flag["do"] = False
                pos["x"] += dx; pos["y"] += dy
                trail.append((pos["x"], pos["y"], sq))
                state.update(dx=dx, dy=dy, squal=sq, shutter=sh, seq=state["seq"] + 1)


def squal_color(sq):
    if sq >= SQUAL_GOOD: return "#1c9c66"      # green - reliable
    if sq >= SQUAL_OK:   return "#e08a1e"      # amber - marginal
    return "#d23b3b"                            # red - unreliable


plt.rcParams["figure.facecolor"] = "#f4f5f7"
fig = plt.figure(figsize=(12, 6.5))
fig.canvas.manager.set_window_title("riskybird PMW3901 optical-flow (live)")
gs = fig.add_gridspec(1, 2, width_ratios=[2.2, 1.0])
axT = fig.add_subplot(gs[0, 0])   # trail
axR = fig.add_subplot(gs[0, 1])   # readout

# --- trail panel ---
axT.set_aspect("equal")
axT.set_title("motion trail  (slide over texture to validate axes)  ·  press 'r' to reset",
              fontsize=11)
axT.axhline(0, color="#c9ced4", lw=0.8); axT.axvline(0, color="#c9ced4", lw=0.8)
axT.set_xlabel("integrated  deltaX  →", fontsize=10)
axT.set_ylabel("integrated  deltaY  ↑", fontsize=10)
(trail_line,) = axT.plot([], [], "-", color="#2a78d6", lw=1.6, alpha=0.9)
cur_dot = axT.scatter([0], [0], s=90, c="#1c9c66", zorder=5, edgecolors="white", linewidths=1.5)
start_dot = axT.scatter([0], [0], s=40, c="#9aa3ad", zorder=3, marker="o")
arrow = axT.annotate("", xy=(0, 0), xytext=(0, 0),
                     arrowprops=dict(arrowstyle="-|>", color="#eb6834", lw=2.2))

# --- readout panel ---
axR.axis("off")
def txt(y, s, size=13, color="#11161c", weight="normal", family="monospace"):
    return axR.text(0.02, y, s, transform=axR.transAxes, fontsize=size,
                    color=color, weight=weight, family=family, va="center")
r_squal_lbl = txt(0.92, "SQUAL", 11, "#79858f")
r_squal = txt(0.85, "--", 30, weight="bold")
axR.add_patch(mpatches.Rectangle((0.02, 0.775), 0.96, 0.03, transform=axR.transAxes,
                                 color="#e3e6ea", zorder=1))
r_squalbar = mpatches.Rectangle((0.02, 0.775), 0.0, 0.03, transform=axR.transAxes,
                                color="#1c9c66", zorder=2)
axR.add_patch(r_squalbar)
r_shut_lbl = txt(0.60, "SHUTTER (exposure)", 11, "#79858f")
r_shut = txt(0.53, "--", 22, family="monospace")
r_dxdy_lbl = txt(0.36, "INSTANT FLOW", 11, "#79858f")
r_dx = txt(0.29, "dX --", 18, family="monospace")
r_dy = txt(0.22, "dY --", 18, family="monospace")
r_rate = txt(0.06, "-- Hz", 11, "#79858f", family="monospace")

_last = {"t": time.time(), "seq": 0, "hz": 0.0}


def on_key(ev):
    if ev.key == "r":
        with lock: reset_flag["do"] = True
fig.canvas.mpl_connect("key_press_event", on_key)


def update(_):
    with lock:
        st = dict(state); tr = list(trail)
    if tr:
        xs = [p[0] for p in tr]; ys = [p[1] for p in tr]
        trail_line.set_data(xs, ys)
        cur_dot.set_offsets([[xs[-1], ys[-1]]]); cur_dot.set_color(squal_color(st["squal"]))
        start_dot.set_offsets([[xs[0], ys[0]]])
        # instant flow arrow from current point (scaled up for visibility)
        k = 6
        arrow.set_position((xs[-1], ys[-1]))
        arrow.xy = (xs[-1] + st["dx"] * k, ys[-1] + st["dy"] * k)
        # autoscale with margin
        pad = max(20, (max(xs) - min(xs)) * 0.1, (max(ys) - min(ys)) * 0.1)
        axT.set_xlim(min(xs) - pad, max(xs) + pad)
        axT.set_ylim(min(ys) - pad, max(ys) + pad)
    # readout
    sq = st["squal"]
    r_squal.set_text(str(sq)); r_squal.set_color(squal_color(sq))
    r_squalbar.set_width(min(sq, 128) / 128 * 0.96); r_squalbar.set_color(squal_color(sq))
    r_shut.set_text(str(st["shutter"]))
    r_dx.set_text(f"dX {st['dx']:+d}"); r_dy.set_text(f"dY {st['dy']:+d}")
    now = time.time()
    if now - _last["t"] >= 1.0:
        _last["hz"] = (st["seq"] - _last["seq"]) / (now - _last["t"])
        _last["seq"] = st["seq"]; _last["t"] = now
    r_rate.set_text(f"{_last['hz']:.0f} Hz   seq={st['seq']}")
    return [trail_line, cur_dot, start_dot]


th = threading.Thread(target=reader, daemon=True); th.start()
from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, interval=80, blit=False, cache_frame_data=False)
plt.tight_layout()
try:
    plt.show()
finally:
    stop.set()
