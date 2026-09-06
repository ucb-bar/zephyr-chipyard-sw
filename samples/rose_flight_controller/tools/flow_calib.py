#!/usr/bin/env python
"""
Analyze a flow_record.py CSV to calibrate FLOW_RAD_PER_COUNT.

Rotation (gyro is ground truth: during rotation the angular flow == body rate):
    python flow_calib.py recordings/rot.csv
    -> regresses aRaw vs gyro; slope = current/true scale; new = 0.021/slope.
       Also reports the gyro-comp sign (slope>0 => subtract, current +1 is right).

Translation (integrate the measured velocity over a known slide):
    python flow_calib.py recordings/slide30fwd.csv --dist 0.30
    -> net displacement from integrating v; new = 0.021 * actual/|disp|.

--cur overrides the scale the data was captured at (default 0.021).
"""
import sys, csv, argparse


def load(path):
    out = []
    with open(path) as f:
        for d in csv.DictReader(f):
            if int(d.get("ok", "1")) != 1:
                continue
            out.append({k: float(v) for k, v in d.items()})
    return out


def regress(xs, ys):
    n = len(xs); mx = sum(xs) / n; my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    syy = sum((y - my) ** 2 for y in ys)
    slope = sxy / sxx if sxx else 0.0
    r2 = (sxy * sxy) / (sxx * syy) if sxx and syy else 0.0
    return slope, r2, n


ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--cur", type=float, default=0.021)
ap.add_argument("--dist", type=float, default=None, help="translation: actual slide distance (m)")
ap.add_argument("--gate", type=float, default=0.3, help="rotation: min |gyro| rad/s to include")
a = ap.parse_args()

data = load(a.csv)
if not data:
    print("no valid samples in", a.csv); sys.exit(1)
dur = data[-1]["t"] - data[0]["t"]
print("%s: %d valid samples, %.1fs" % (a.csv, len(data), dur))

if a.dist is None:
    print("ROTATION regression (aRaw vs gyro, |gyro|>%.1f):" % a.gate)
    for name, gk, ak in (("pitch->fwd", "gy", "ax"), ("roll->left", "gx", "ay")):
        pts = [(d[gk], d[ak]) for d in data if abs(d[gk]) > a.gate]
        if len(pts) < 15:
            print("  %-11s only %d rotation samples -- rotate harder/longer" % (name, len(pts))); continue
        slope, r2, n = regress([p[0] for p in pts], [p[1] for p in pts])
        newk = a.cur / slope if slope else 0.0
        print("  %-11s n=%d  slope=%+.2f  R2=%.2f  ->  RAD_PER_COUNT=%.6f   comp-sign=%s"
              % (name, n, slope, r2, newk, "+ keep" if slope > 0 else "- FLIP"))
else:
    dx = dy = 0.0
    for i in range(1, len(data)):
        dt = data[i]["t"] - data[i - 1]["t"]
        if dt > 0.2:
            continue
        dx += 0.5 * (data[i]["vx"] + data[i - 1]["vx"]) * dt
        dy += 0.5 * (data[i]["vy"] + data[i - 1]["vy"]) * dt
    axis = "x/fwd" if abs(dx) >= abs(dy) else "y/left"
    disp = max(abs(dx), abs(dy))
    hs = sorted(d["height"] for d in data)
    print("TRANSLATION integrate: net dx=%.3f dy=%.3f m  (dominant %s, |disp|=%.3f, med h=%.2f)"
          % (dx, dy, axis, disp, hs[len(hs) // 2]))
    if disp > 1e-6:
        print("  actual=%.3f m  ->  RAD_PER_COUNT=%.6f  (= %.4f * %.3f/%.3f)"
              % (a.dist, a.cur * a.dist / disp, a.cur, a.dist, disp))
