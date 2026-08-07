#!/usr/bin/env python
"""
Live 8x8 heatmaps of the 4 riskybird side-ToF sensors, for occlusion mapping.

The ROSE_BUMPER_GRID build streams, per sensor per cycle:
    GRID <name>: v0 v1 ... v63        (row-major 8x8; mm where valid, -1 = invalid/masked)
plus the usual walls[...] summary line (ignored here).

Shows one heatmap per sensor (front/back/left/right). Near = red, far = green,
invalid/occluded zones = grey. Occlusion from the PCB/components shows up as
persistent grey or persistently-near (red) zones toward an edge/corner.

Run in the conda 'zephyr' env:
    python live_tof_plot.py            # /dev/ttyACM0
    python live_tof_plot.py /dev/ttyACM1
Close the window (or Ctrl-C) to stop. Reconnects if the port drops (reflash/reset).
"""
import sys, re, time, threading
import serial
import numpy as np
import matplotlib.pyplot as plt

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 115200
VMIN, VMAX = 0, 1000     # mm color range

GRID_RE = re.compile(r"GRID\s+(\w+):\s*(-?\d[\d\s\-]*)")
NAMES = ("front", "back", "left", "right")

lock = threading.Lock()
grids = {n: np.full((8, 8), np.nan) for n in NAMES}
counts = {n: 0 for n in NAMES}
stop = threading.Event()


def serial_reader():
    while not stop.is_set():
        try:
            s = serial.Serial(PORT, BAUD, timeout=0.3)
        except Exception:
            time.sleep(0.3)
            continue
        buf = b""
        while not stop.is_set():
            try:
                d = s.read(512)
            except Exception:
                break
            if not d:
                continue
            buf += d
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                m = GRID_RE.search(line.decode("utf-8", "replace"))
                if not m:
                    continue
                name = m.group(1)
                if name not in NAMES:
                    continue
                # robust to occasional serial truncation/interleave: pull clean int tokens
                vals = [int(x) for x in re.findall(r"-?\d+", m.group(2))]
                if len(vals) not in (16, 64):   # drop corrupted/partial lines
                    continue
                n = 8 if len(vals) == 64 else 4
                arr = np.array(vals[:n * n], dtype=float).reshape(n, n)
                arr[arr < 0] = np.nan            # invalid -> masked
                if n == 4:                       # upscale 4x4 -> 8x8 for a common canvas
                    arr = np.kron(arr, np.ones((2, 2)))
                with lock:
                    grids[name] = arr
                    counts[name] += 1


cmap = plt.get_cmap("RdYlGn").copy()  # near=red, far=green
cmap.set_bad("lightgrey")             # invalid/occluded

fig, axes = plt.subplots(2, 2, figsize=(10, 9))
fig.canvas.manager.set_window_title("riskybird side-ToF 8x8 grids (live)")
fig.suptitle("near = red   far = green   grey = invalid/occluded   (mm)")
axpos = dict(zip(NAMES, axes.flat))
ims, titles = {}, {}
for name, ax in axpos.items():
    im = ax.imshow(np.full((8, 8), np.nan), cmap=cmap, vmin=VMIN, vmax=VMAX,
                   interpolation="nearest", origin="upper")
    ax.set_xticks(range(8)); ax.set_yticks(range(8))
    ax.tick_params(labelsize=6)
    titles[name] = ax.set_title(name)
    ims[name] = im
fig.colorbar(ims["front"], ax=axes, shrink=0.7, label="distance (mm)")


def update(_):
    with lock:
        snap = {n: grids[n].copy() for n in NAMES}
        cnt = dict(counts)
    for name in NAMES:
        ims[name].set_data(snap[name])
        finite = snap[name][np.isfinite(snap[name])]
        # firmware policy: mean of VALID zones over the top 2 rows (indices 0..15)
        clear = snap[name][0:2, :]
        clear_valid = clear[np.isfinite(clear)]
        wall = int(round(clear_valid.mean())) if clear_valid.size else -1
        titles[name].set_text(f"{name}   valid={finite.size}/64   "
                              f"WALL(rows0-1 avg)={wall} mm  [{clear_valid.size}/16]")
    return list(ims.values())


reader = threading.Thread(target=serial_reader, daemon=True)
reader.start()

from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, interval=150, blit=False, cache_frame_data=False)
try:
    plt.show()
finally:
    stop.set()
