#!/usr/bin/env python
"""
riskybird v3 -- live 3D STATE / SENSOR debug panel  (tethered over USB).

Parses the flight controller's telemetry stream and shows, live:
  * a 3D drone POSE (body frame rotated by the estimated roll/pitch/yaw),
  * raw + fused SENSORS: down-ToF height, optical-flow (raw angular vs gyro vs compensated
    velocity, SQUAL) and the estimator's horizontal velocity, side-ToF walls, motor duties,
  * the FLIGHT/ARM STATE machine (booting / IMU-fail / waiting-for-gesture / lift-detected /
    ARMED-flying / ESTOP+reason / landed), driven by the AUTOFLIGHT log events.

It only READS the serial port -- it never resets the board (DTR/RTS deasserted, see
serial_capture.py). Works with any build that emits the `it=` / `flow:` / `walls[]` telemetry.

SAFE TETHERED DEBUGGING: with a motors-active autoflight build + props on, completing the
lift-and-place will ARM and spin the props while tethered. To exercise the full state machine
safely on the bench, build with motors capped off:
    ... -DEXTRA_CPPFLAGS="... -DAUTOFLIGHT_MAX_DUTY=0.0f ..."
then the state transitions + telemetry all run with NO prop spin. (Or just pull the props.)

Run in the conda 'zephyr' env:
    python state_viz.py            # /dev/ttyACM0
    python state_viz.py /dev/ttyACM1
Keys:  q = quit,  r = clear the time-series trails.
"""
import sys, re, time, threading, collections, math
import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)
import matplotlib.cm as cm
from matplotlib.colors import Normalize

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
WIN = 12.0  # seconds of history in the time-series panels
# per-motor thrust vectors (FIXED scale -- a given length/color always means the same force)
FORCE_CMAP = cm.turbo
FORCE_NORM = Normalize(0.0, 1.0)   # motor duty 0..1 = force magnitude, fixed (not auto-ranged)
FORCE_LEN = 0.85                   # world length of a full-thrust (duty=1.0) vector

# ---- telemetry line parsers ------------------------------------------------
RE_IT = re.compile(
    r"it=(\d+) dt=(\d+)ms roll=(-?\d+\.\d+) pitch=(-?\d+\.\d+) yaw=(-?\d+\.\d+) "
    r"z=(-?\d+\.\d+) tofv=(\d+) tofh=(-?\d+\.\d+) "
    r"u=\[(-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+)\]")
RE_FLOW = re.compile(
    r"flow: aRaw=\[(-?\d+\.\d+) (-?\d+\.\d+)\] gyro=\[(-?\d+\.\d+) (-?\d+\.\d+)\] "
    r"v=\[(-?\d+\.\d+) (-?\d+\.\d+)\] sq=(\d+) (\w+) \| est v=\[(-?\d+\.\d+) (-?\d+\.\d+)\]")
RE_WALLS = re.compile(r"walls\[seq=(\d+)\]: front=(-?\d+) back=(-?\d+) left=(-?\d+) right=(-?\d+)")

lock = threading.Lock()
S = {  # latest scalars
    "t0": time.time(), "last_rx": 0.0, "dt": 0, "iter": 0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "z": 0.0, "tofv": 0, "tofh": 0.0,
    "u": [0, 0, 0, 0],
    "araw": [0, 0], "gyro": [0, 0], "vflow": [0, 0], "sq": 0, "flowok": False,
    "evx": 0.0, "evy": 0.0,
    "walls": {"front": -1, "back": -1, "left": -1, "right": -1},
    "phase": "BOOTING", "armed": False, "estop": False, "wd_reason": "", "note": "",
}
TS = {k: collections.deque(maxlen=1500) for k in
      ("t", "tofh", "z", "evx", "evy", "vfwd", "vleft", "sq", "gpitch", "groll")}
stop = threading.Event()


def open_noreset(port):
    s = serial.Serial(); s.port = port; s.baudrate = 115200; s.timeout = 0.2
    s.dtr = False; s.rts = False; s.open(); return s


def set_phase(ph, **kw):
    S["phase"] = ph
    for k, v in kw.items():
        S[k] = v


def reader():
    s = None; buf = b""
    while not stop.is_set():
        try:
            if s is None:
                s = open_noreset(PORT)
            d = s.read(400)
        except Exception:
            s = None; time.sleep(0.2); continue
        if not d:
            continue
        buf += d
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            line = ln.decode("utf-8", "replace")
            now = time.time(); t = now - S["t0"]
            with lock:
                S["last_rx"] = now
                m = RE_IT.search(line)
                if m:
                    g = m.groups()
                    S["iter"] = int(g[0]); S["dt"] = int(g[1])
                    S["roll"], S["pitch"], S["yaw"] = float(g[2]), float(g[3]), float(g[4])
                    S["z"] = float(g[5]); S["tofv"] = int(g[6]); S["tofh"] = float(g[7])
                    S["u"] = [float(g[8]), float(g[9]), float(g[10]), float(g[11])]
                    TS["t"].append(t); TS["tofh"].append(S["tofh"]); TS["z"].append(S["z"])
                    TS["evx"].append(S["evx"]); TS["evy"].append(S["evy"])
                    TS["vfwd"].append(S["vflow"][0]); TS["vleft"].append(S["vflow"][1])
                    TS["sq"].append(S["sq"])
                    TS["gpitch"].append(S["gyro"][1]); TS["groll"].append(S["gyro"][0])
                    continue
                m = RE_FLOW.search(line)
                if m:
                    g = m.groups()
                    S["araw"] = [float(g[0]), float(g[1])]; S["gyro"] = [float(g[2]), float(g[3])]
                    S["vflow"] = [float(g[4]), float(g[5])]; S["sq"] = int(g[6])
                    S["flowok"] = (g[7] == "ok"); S["evx"] = float(g[8]); S["evy"] = float(g[9])
                    continue
                m = RE_WALLS.search(line)
                if m:
                    g = m.groups()
                    S["walls"] = {"front": int(g[1]), "back": int(g[2]),
                                  "left": int(g[3]), "right": int(g[4])}
                    continue
                # ---- state-machine events ----
                if "IMU not ready" in line:
                    set_phase("IMU FAIL", note="power-cycle + retry")
                elif "lift-and-place to arm" in line or "no-gesture" in line:
                    set_phase("WAITING GESTURE", armed=False, estop=False)
                elif "lift detected" in line:
                    set_phase("LIFT DETECTED")
                elif "ARMED -- taking off" in line:
                    set_phase("ARMED / FLYING", armed=True)
                elif "EMERGENCY CUTOFF" in line:
                    mm = re.search(r"CUTOFF -- (\w+) limit", line)
                    set_phase("ESTOP", estop=True, armed=False,
                              wd_reason=(mm.group(1) if mm else "?"))
                elif "landed" in line or "disarm" in line or "flight complete" in line:
                    set_phase("LANDED / DISARMED", armed=False)
                elif "estimator=" in line and "ready" in line:
                    if S["phase"] in ("BOOTING",):
                        set_phase("READY (booting sensors)")
                elif "optical flow up" in line or "down-ToF ranging" in line \
                        or "side-ToF bumper" in line:
                    pass  # boot progress; keep phase


threading.Thread(target=reader, daemon=True).start()

# ---- drone body geometry (FLU: +x fwd, +y left, +z up), X-config -----------
A = 0.72
ROTORS = {  # body-frame rotor positions; idx maps to u[]: 0=FR 1=RR 2=RL 3=FL
    "FR": np.array([A, -A, 0]), "RR": np.array([-A, -A, 0]),
    "RL": np.array([-A, A, 0]), "FL": np.array([A, A, 0]),
}


def R_rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r); cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


PHASE_COLOR = {
    "BOOTING": "#888", "READY (booting sensors)": "#7aa9e6", "IMU FAIL": "#d43a3a",
    "WAITING GESTURE": "#e6b800", "LIFT DETECTED": "#e07b2a",
    "ARMED / FLYING": "#2ea44f", "ESTOP": "#d43a3a", "LANDED / DISARMED": "#2a78d6",
}

# ---- figure ----------------------------------------------------------------
#  row0: [ 3D pose ][ state text ]
#  row1: [ 3D pose ][ velocity   ]   (m/s)
#  row2: [ altitude][ gyro       ]   (rad/s -- own axis, separate from velocity)
#  row3: [ motors  ][ walls      ]
fig = plt.figure(figsize=(14, 9.5))
fig.canvas.manager.set_window_title("riskybird state / sensor panel")
gs = GridSpec(4, 4, figure=fig, hspace=0.55, wspace=0.35,
              left=0.05, right=0.98, top=0.96, bottom=0.06)
ax3d = fig.add_subplot(gs[0:2, 0:2], projection="3d")
axst = fig.add_subplot(gs[0, 2:4]); axst.axis("off")
axvel = fig.add_subplot(gs[1, 2:4])    # est/raw horizontal velocity (m/s)
axalt = fig.add_subplot(gs[2, 0:2])    # altitude (m)
axgyro = fig.add_subplot(gs[2, 2:4])   # body rates (rad/s) -- own axis vs velocity
axmot = fig.add_subplot(gs[3, 0:2])
axwall = fig.add_subplot(gs[3, 2:4])

# 3D static setup
for aa in (ax3d,):
    aa.set_xlim(-1.2, 1.2); aa.set_ylim(-1.2, 1.2); aa.set_zlim(-1.2, 1.2)
    aa.set_xlabel("fwd (x)"); aa.set_ylabel("left (y)"); aa.set_zlabel("up (z)")
    aa.set_title("drone pose (est roll/pitch/yaw)")
arm_lines = [ax3d.plot([], [], [], lw=3)[0] for _ in range(4)]  # 4 arms
rotor_pts = [ax3d.plot([], [], [], "o", ms=9)[0] for _ in range(4)]
nose_line, = ax3d.plot([], [], [], color="#d43a3a", lw=3)   # forward heading
up_line, = ax3d.plot([], [], [], color="#2a78d6", lw=2)     # body-up
# per-motor thrust vectors: an arrow from each rotor along body +z (thrust), length + color = force
force_lines = [ax3d.plot([], [], [], lw=4, solid_capstyle="round")[0] for _ in range(4)]
force_tips = [ax3d.plot([], [], [], "^", ms=7)[0] for _ in range(4)]
_sm = cm.ScalarMappable(cmap=FORCE_CMAP, norm=FORCE_NORM); _sm.set_array([])
_cb = fig.colorbar(_sm, ax=ax3d, location="left", fraction=0.026, pad=0.02)
_cb.set_label("motor force (duty, fixed 0-1)", fontsize=7); _cb.ax.tick_params(labelsize=6)
# faint world reference triad
for vec, c in (([1, 0, 0], "#c33"), ([0, 1, 0], "#3a3"), ([0, 0, 1], "#33c")):
    ax3d.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], color=c, lw=1, alpha=0.25)

status_txt = axst.text(0.02, 0.98, "", va="top", ha="left", family="monospace", fontsize=11)
phase_txt = axst.text(0.98, 0.98, "", va="top", ha="right", family="monospace",
                      fontsize=15, fontweight="bold")

# velocity panel (m/s): estimator vx/vy + the raw optical-flow velocity as a light reference
ln_evx, = axvel.plot([], [], color="#2a78d6", lw=1.6, label="est vx (fwd)")
ln_evy, = axvel.plot([], [], color="#e0603e", lw=1.6, label="est vy (left)")
ln_vfwd, = axvel.plot([], [], color="#8fb8e6", ls=":", lw=1, label="raw flow vx")
ln_vleft, = axvel.plot([], [], color="#f0b090", ls=":", lw=1, label="raw flow vy")
axvel.set_title("horizontal velocity"); axvel.set_ylabel("m/s")
axvel.grid(alpha=0.3); axvel.legend(fontsize=7, ncol=2, loc="upper left")

# gyro panel (rad/s): body rates on their OWN axis, so the m/s vs rad/s magnitude gap can't flatten them
ln_gr, = axgyro.plot([], [], color="#7a4fd6", lw=1.4, label="gyro roll")
ln_gp, = axgyro.plot([], [], color="#2ea44f", lw=1.4, label="gyro pitch")
axgyro.set_title("body rates (gyro)"); axgyro.set_xlabel("t (s)"); axgyro.set_ylabel("rad/s")
axgyro.grid(alpha=0.3); axgyro.legend(fontsize=7, loc="upper left")

ln_tofh, = axalt.plot([], [], color="#3aa66f", label="down-ToF RAW slant (m)")
ln_z, = axalt.plot([], [], color="#2a78d6", label="est VERTICAL z (m)")
axalt.set_title("altitude (raw slant vs tilt-corrected estimate)")
axalt.set_xlabel("t (s)"); axalt.set_ylabel("m")
axalt.grid(alpha=0.3); axalt.legend(fontsize=7, loc="upper left")

mot_bars = axmot.bar(["FR", "RR", "RL", "FL"], [0, 0, 0, 0], color="#2a78d6")
axmot.set_title("motor cmd u[]"); axmot.set_ylim(-0.6, 0.5); axmot.grid(alpha=0.3, axis="y")

wall_bars = axwall.bar(["F", "B", "L", "R"], [0, 0, 0, 0], color="#b080d0")
axwall.set_title("side ToF (mm)"); axwall.set_ylim(0, 400); axwall.grid(alpha=0.3, axis="y")


def on_key(e):
    if e.key == "q":
        stop.set(); plt.close(fig)
    elif e.key == "r":
        for dq in TS.values():
            dq.clear()
fig.canvas.mpl_connect("key_press_event", on_key)


def update(_):
    with lock:
        s = dict(S)
        t = list(TS["t"])
        series = {k: list(TS[k]) for k in TS}
    # ---- 3D pose ----
    R = R_rpy(s["roll"], s["pitch"], s["yaw"])
    rot = {k: R @ v for k, v in ROTORS.items()}
    order = ["FR", "RR", "RL", "FL"]
    for i, name in enumerate(order):
        p = rot[name]
        arm_lines[i].set_data_3d([0, p[0]], [0, p[1]], [0, p[2]])
        arm_lines[i].set_color("#2ea44f" if name in ("FR", "FL") else "#555")  # front=green
        rotor_pts[i].set_data_3d([p[0]], [p[1]], [p[2]])
        # motor thrust force (proxy = commanded duty, u ~ [-0.583..0.417] -> 0..1)
        force = max(0.0, min(1.0, s["u"][i] + 0.583))
        col = FORCE_CMAP(FORCE_NORM(force))
        rotor_pts[i].set_color(col)
        # thrust vector: body +z (up) scaled by force, rotated into world, drawn from the rotor
        tip = p + R @ np.array([0.0, 0.0, FORCE_LEN * force])
        force_lines[i].set_data_3d([p[0], tip[0]], [p[1], tip[1]], [p[2], tip[2]])
        force_lines[i].set_color(col)
        force_tips[i].set_data_3d([tip[0]], [tip[1]], [tip[2]])
        force_tips[i].set_color(col)
    nose = R @ np.array([1.05, 0, 0]); nose_line.set_data_3d([0, nose[0]], [0, nose[1]], [0, nose[2]])
    up = R @ np.array([0, 0, 0.7]); up_line.set_data_3d([0, up[0]], [0, up[1]], [0, up[2]])
    # ---- status text ----
    alive = s["last_rx"] and (time.time() - s["last_rx"] < 1.5)
    ph = s["phase"]
    phase_txt.set_text(ph + ("" if alive else "  (no link)"))
    phase_txt.set_color(PHASE_COLOR.get(ph, "#000") if alive else "#999")
    axst.set_facecolor((0.97, 0.97, 0.97) if alive else (0.9, 0.9, 0.9))
    wd = ("TRIPPED: %s" % s["wd_reason"]) if s["estop"] else "ok"
    status_txt.set_text(
        "link   : %s   loop %d ms   it=%d\n"
        "armed  : %s        watchdog: %s\n"
        "attitude  roll %+6.3f  pitch %+6.3f  yaw %+6.3f  rad\n"
        "height    z %+6.3f m   down-ToF %.3f m  (valid=%d)\n"
        "flow      est v=[%+.2f %+.2f] m/s   raw v=[%+.2f %+.2f]\n"
        "          SQUAL %3d  %s\n"
        "%s"
        % ("UP" if alive else "-- STALE --", s["dt"], s["iter"],
           "YES" if s["armed"] else "no", wd,
           s["roll"], s["pitch"], s["yaw"],
           s["z"], s["tofh"], s["tofv"],
           s["evx"], s["evy"], s["vflow"][0], s["vflow"][1],
           s["sq"], "OK" if s["flowok"] else "gated/stale",
           ("note: " + s["note"]) if s["note"] else ""))
    # ---- time series ----
    if t:
        tmax = t[-1]; tmin = tmax - WIN
        ln_evx.set_data(t, series["evx"]); ln_evy.set_data(t, series["evy"])
        ln_vfwd.set_data(t, series["vfwd"]); ln_vleft.set_data(t, series["vleft"])
        ln_gr.set_data(t, series["groll"]); ln_gp.set_data(t, series["gpitch"])
        ln_tofh.set_data(t, series["tofh"]); ln_z.set_data(t, series["z"])
        for ax in (axvel, axgyro, axalt):
            ax.set_xlim(tmin, tmax)

        def yl(ax, keys, floor=0.05):
            vals = [v for k in keys for v, tt in zip(series[k], t) if tt >= tmin]
            if vals:
                lo, hi = min(vals), max(vals); pad = max(floor, 0.12 * (hi - lo))
                ax.set_ylim(lo - pad, hi + pad)
        yl(axvel, ("evx", "evy", "vfwd", "vleft"))
        yl(axgyro, ("groll", "gpitch"), floor=0.02)   # tighter floor -> small gyro detail visible
        yl(axalt, ("tofh", "z"))
    # ---- bars ----
    for b, val in zip(mot_bars, s["u"]):
        b.set_height(val)
    w = s["walls"]
    for b, key in zip(wall_bars, ("front", "back", "left", "right")):
        v = w[key]; b.set_height(v if v >= 0 else 0)
        b.set_color("#ccc" if v < 0 else ("#d43a3a" if v < 80 else "#b080d0"))
    return arm_lines


ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
plt.show()
stop.set()
