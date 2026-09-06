#!/usr/bin/env python
"""
Flow-calibration RECORDER GUI for the riskybird FC (ROSE_FLOW + enhanced telemetry build).

Reads the flight-controller "flow:" line (raw angular flow, gyro, compensated velocity, squal)
and the "it=" height, plots them live, and lets YOU control recording: type a label, press REC
(or Spacebar), do the motion, press STOP -> a labeled CSV lands in tools/recordings/ for me to
analyze. This removes the timing-sync problem: opening the port reboots the FC (~13 s), so just
wait for the status to say STREAMING, then record whenever you're ready.

Calibration motions:
  label "rot"       : rotate in place (pitch, then roll). I regress aRaw vs gyro -> RAD_PER_COUNT.
  label "slide30fwd": slide a MEASURED distance (put the cm + direction in the label) at a steady
                      height. I integrate the velocity over the segment -> RAD_PER_COUNT.

Opens the port with DTR/RTS DEASSERTED (ESP32-C6 USB-JTAG maps them to BOOT/EN; a normal open
would drop it into ROM download). See serial_capture.py open_noreset().

Run in the conda 'zephyr' env:
    python flow_record.py            # /dev/ttyACM0
    python flow_record.py /dev/ttyACM1
"""
import sys, os, re, time, threading, collections, csv
import serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
from matplotlib.animation import FuncAnimation

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
REC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
os.makedirs(REC_DIR, exist_ok=True)
WIN_S = 12.0   # live plot window (seconds)

FLOW_RE = re.compile(
    r"aRaw=\[(-?\d+\.\d+) (-?\d+\.\d+)\] gyro=\[(-?\d+\.\d+) (-?\d+\.\d+)\] "
    r"v=\[(-?\d+\.\d+) (-?\d+\.\d+)\] sq=(\d+) (\w+)")
TOFH_RE = re.compile(r"tofh=(-?\d+\.\d+)")

lock = threading.Lock()
buf = collections.deque(maxlen=2000)     # (t, ax,ay,gx,gy,vx,vy,h,sq,ok)
rec = {"on": False, "rows": [], "label": "rot"}
stat = {"last_rx": 0.0, "n": 0}
stop = threading.Event()
T0 = time.time()


def open_noreset(port):
    s = serial.Serial(); s.port = port; s.baudrate = 115200; s.timeout = 0.2
    s.dtr = False; s.rts = False; s.open(); return s


def reader():
    s = None; last_h = 0.0
    while not stop.is_set():
        try:
            if s is None:
                s = open_noreset(PORT)
            d = s.read(400)
        except Exception:
            s = None; time.sleep(0.2); continue
        if not d:
            continue
        for line in d.decode("utf-8", "replace").splitlines():
            mh = TOFH_RE.search(line)
            if mh:
                last_h = float(mh.group(1))
            m = FLOW_RE.search(line)
            if not m:
                continue
            row = (time.time() - T0,
                   float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)),
                   float(m.group(5)), float(m.group(6)), last_h, int(m.group(7)), m.group(8) == "ok")
            with lock:
                buf.append(row)
                stat["last_rx"] = time.time(); stat["n"] += 1
                if rec["on"]:
                    rec["rows"].append(row)


threading.Thread(target=reader, daemon=True).start()

# ---------- figure ----------
fig, (axR, axV, axH) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.canvas.manager.set_window_title("riskybird flow recorder")
fig.subplots_adjust(left=0.09, right=0.98, top=0.94, bottom=0.22, hspace=0.28)

ln = {}
ln["ax"], = axR.plot([], [], color="#2a78d6", lw=1.4, label="aRaw fwd")
ln["ay"], = axR.plot([], [], color="#e0603e", lw=1.4, label="aRaw left")
ln["gy"], = axR.plot([], [], color="#7aa9e6", lw=1.2, ls="--", label="gyro pitch")
ln["gx"], = axR.plot([], [], color="#f0a080", lw=1.2, ls="--", label="gyro roll")
axR.set_ylabel("rad/s"); axR.grid(alpha=0.3); axR.legend(loc="upper left", fontsize=8, ncol=2)
axR.set_title("raw angular flow vs gyro   (rotate: solid should track dashed once calibrated)")

ln["vx"], = axV.plot([], [], color="#2a78d6", lw=1.4, label="v fwd")
ln["vy"], = axV.plot([], [], color="#e0603e", lw=1.4, label="v left")
axV.set_ylabel("m/s"); axV.grid(alpha=0.3); axV.legend(loc="upper left", fontsize=8)
axV.set_title("compensated body velocity   (slide: this should track the motion)")

ln["h"], = axH.plot([], [], color="#3aa66f", lw=1.4, label="height (m)")
ln["sq"], = axH.plot([], [], color="#b080d0", lw=1.2, label="squal/100")
axH.set_ylabel("m / -"); axH.set_xlabel("time (s)"); axH.grid(alpha=0.3); axH.legend(loc="upper left", fontsize=8)

status = fig.text(0.02, 0.115, "", fontsize=10, family="monospace")

# ---------- widgets ----------
tb = TextBox(fig.add_axes([0.11, 0.03, 0.18, 0.045]), "label ", initial="rot")
btn = Button(fig.add_axes([0.72, 0.03, 0.14, 0.05]), "● REC")


def on_label(text):
    rec["label"] = text.strip() or "seg"
tb.on_submit(on_label)


def toggle(event=None):
    with lock:
        if not rec["on"]:
            rec["on"] = True; rec["rows"] = []; rec["label"] = (tb.text.strip() or "seg")
            btn.label.set_text("■ STOP"); rows = None
        else:
            rec["on"] = False; rows = list(rec["rows"])
            btn.label.set_text("● REC")
    if rows is None:
        return
    if not rows:
        print("(nothing recorded)"); return
    base = os.path.join(REC_DIR, rec["label"]); path = base + ".csv"; k = 1
    while os.path.exists(path):
        path = "%s_%d.csv" % (base, k); k += 1
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "ax", "ay", "gx", "gy", "vx", "vy", "height", "squal", "ok"])
        for r in rows:
            w.writerow(["%.4f" % r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], int(r[9])])
    print("SAVED %s  (%d samples, %.1fs)" % (path, len(rows), rows[-1][0] - rows[0][0]))
btn.on_clicked(toggle)


def on_key(e):
    if e.key == " ":
        toggle()
fig.canvas.mpl_connect("key_press_event", on_key)


def yfit(ax, data, idxs, tmin):
    vals = [d[i] for d in data if d[0] >= tmin for i in idxs]
    if vals:
        lo, hi = min(vals), max(vals); pad = max(0.1, 0.12 * (hi - lo))
        ax.set_ylim(lo - pad, hi + pad)


def update(_):
    with lock:
        data = list(buf); rec_on = rec["on"]; rec_n = len(rec["rows"]); lbl = rec["label"]
        last_rx = stat["last_rx"]; ntot = stat["n"]
    if data:
        t = [d[0] for d in data]; tmax = t[-1]; tmin = tmax - WIN_S
        for key, idx in (("ax", 1), ("ay", 2), ("gx", 3), ("gy", 4), ("vx", 5), ("vy", 6), ("h", 7)):
            ln[key].set_data(t, [d[idx] for d in data])
        ln["sq"].set_data(t, [d[8] / 100.0 for d in data])
        for a in (axR, axV, axH):
            a.set_xlim(tmin, tmax)
        yfit(axR, data, (1, 2, 3, 4), tmin); yfit(axV, data, (5, 6), tmin); yfit(axH, data, (7, 8), tmin)
    alive = last_rx and (time.time() - last_rx) < 1.5
    board = "STREAMING ✓" if alive else ("waiting for board (reboots ~13 s on open)…" if ntot == 0 else "no data")
    recs = ("● RECORDING '%s'  n=%d" % (lbl, rec_n)) if rec_on else "idle (type label, then REC / Spacebar)"
    live = ""
    if data:
        d = data[-1]
        live = "aRaw[%+.2f %+.2f] gyro[%+.2f %+.2f] v[%+.2f %+.2f] h=%.2f sq=%d" % (d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8])
    status.set_text("board: %s      %s\n%s\nCSV -> %s" % (board, recs, live, REC_DIR))
    return list(ln.values())


ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
plt.show()
stop.set()
