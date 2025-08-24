import os
import re
import numpy as np
import matplotlib.pyplot as plt
import json



import matplotlib
matplotlib.use("Agg")   # headless backend

import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from matplotlib.lines import Line2D

power_data = {
    "scalar": {
        50: 0.076,
        75: 0.085,
        100: 0.093,
        125: 0.101,
        150: 0.111,
        200: 0.127,
        250: 0.144,
        375: 0.188,
        500: 0.227,
        600: 0.259,
    },
    "vector": {
        50: 0.075,
        75: 0.084,
        100: 0.092,
        125: 0.101,
        150: 0.109,
        200: 0.125,
        250: 0.142,
        375: 0.183,
        500: 0.224,
        600: 0.260,
    }
}

directories = [
    "data/scalar", 
    "data/rvv", 
]

def trajectory_passed(data,
                      z_threshold=0.05,
                      xy_limit=10.0,
                      z_index=2):
    """
    Given a structured-array `data` with fields:
      - data['state'] shape (N,12)
      - data['tx'], data['ty'], data['tz'] shape (N,)
    returns True if:
      1) reconstructed Z = state[:,z_index] + tz never dips below z_threshold
      2) reconstructed X,Y = state[:,0:2] + [tx,ty] always lie inside
         [-xy_limit, xy_limit] × [-xy_limit, xy_limit]
    """
    # world-frame Z
    rel_z    = data['state'][:, z_index]
    actual_z = rel_z + data['tz']
    if np.any(actual_z < z_threshold):
        return False

    # world-frame XY
    rel_xy    = data['state'][:, :2]
    target_xy = np.stack([data['tx'], data['ty']], axis=1)
    actual_xy = rel_xy + target_xy
    if np.any(np.abs(actual_xy) > xy_limit):
        return False

    return True

def trajectory_passed_file(path, **kwargs):
    """
    Loads the .npy at `path` and applies trajectory_passed.
    All kwargs (z_threshold, xy_limit, z_index) are passed through.
    """
    data = np.load(path, allow_pickle=False)
    return trajectory_passed(data, **kwargs)

def avg_power_for_file(path, hover_rpm, hover_power, max_rpm, prop_scale):
    """
    Compute the average total power for a single flight log (.npy file).
    Returns None if the trajectory failed or file is invalid.
    """
    if not trajectory_passed_file(path):
        return None

    data = np.load(path, allow_pickle=False)
    rpm = data['rpm'].astype(np.float64)
    rpm_clipped = np.clip(rpm, 0.0, max_rpm)

    # per-motor power with prop scaling
    P_motor = hover_power * (rpm_clipped / hover_rpm)**3 * prop_scale**5  # shape (steps,4)

    # total power per timestep
    P_total = P_motor.sum(axis=1)  # shape (steps,)

    # return average over all timesteps
    return P_total.mean()


def compute_avg_power(directory,
                      hover_rpm=14000,
                      hover_power=1.25,
                      max_rpm=21702,
                      prop_scale=1.0):
    """
    Groups logs in `directory` by difficulty, uses avg_power_for_file()
    on each valid .npy file, and returns a dict of average total power per difficulty.
    """
    results = {}
    for diff in ('easy', 'medium', 'hard'):
        prefix = diff + '_'
        power_vals = []

        for fn in os.listdir(directory):
            if not fn.endswith('.npy') or not fn.startswith(prefix):
                continue
            path = os.path.join(directory, fn)
            avg_p = avg_power_for_file(path, hover_rpm, hover_power, max_rpm, prop_scale)
            if avg_p is not None:
                power_vals.append(avg_p)

        results[diff] = np.nanmean(power_vals) if power_vals else np.nan

    return results

def plot_summary(directories,
                 power_data,
                 hover_rpm=14000,
                 hover_power=1.25,
                 max_rpm=21702,
                 outdir="results/hil-results",
                 outfile="hil-results.png"):
    """
    Two groups on x-axis: Scalar and RVV (100 MHz).
    Within each group, three bars: Easy (green), Medium (blue), Hard (red).
    Subplots:
      (a) median solve time (ms) with IQR error bars
      (b) successful trajectories
      (c) total power (W) = actuator(diff) + CPU(100 MHz)
    """
    os.makedirs(outdir, exist_ok=True)

    # --- select dirs ---
    scalar_dirs = [d for d in directories if "scalar" in d]
    rvv_dirs    = [d for d in directories if "rvv" in d]
    if not scalar_dirs or not rvv_dirs:
        raise ValueError("Need one directory containing 'data_scalar' and one containing 'data_rvv'.")
    d_scalar = scalar_dirs[0]
    d_rvv    = rvv_dirs[0]

    FREQ_MHZ = 100.0
    diffs    = ["easy", "medium", "hard"]
    colors   = {"easy": "green", "medium": "blue", "hard": "red"}

    # ---------- helpers ----------
    def load_ns_for_diff(dirpath, diff):
        """Concatenate 'ns' (nanoseconds) from <diff>_*.npz/.npy files."""
        out = []
        for fn in os.listdir(dirpath):
            if not (fn.startswith(f"{diff}_") and (fn.endswith(".npz") or fn.endswith(".npy"))):
                continue
            path = os.path.join(dirpath, fn)
            data = np.load(path, allow_pickle=False)
            try:
                if isinstance(data, np.lib.npyio.NpzFile):
                    if 'ns' not in data.files:
                        continue
                    ns = data['ns']
                else:
                    arr = data
                    if getattr(arr, "dtype", None) is not None and arr.dtype.fields and ('ns' in arr.dtype.fields):
                        ns = arr['ns']
                    else:
                        ns = arr
                ns = np.asarray(ns)
                if ns.size:
                    valid = ns[ns >= 0]
                    if valid.size:
                        out.append(valid.astype(np.float64))
            finally:
                if isinstance(data, np.lib.npyio.NpzFile):
                    data.close()
        return np.concatenate(out) if out else np.array([], dtype=np.float64)

    def total_passes_for_diff(dirpath, diff):
        cnt = 0
        for fn in os.listdir(dirpath):
            if not (fn.startswith(f"{diff}_") and (fn.endswith(".npz") or fn.endswith(".npy"))):
                continue
            if trajectory_passed_file(os.path.join(dirpath, fn)):
                cnt += 1
        return cnt

    def actuator_power_for_diff(dirpath, diff):
        ap = compute_avg_power(dirpath, hover_rpm, hover_power, max_rpm)  # dict per diff
        val = ap.get(diff)
        return float(val) if val is not None else np.nan

    # ---------- metrics per (series, difficulty) ----------
    metrics = {"Scalar": {}, "RVV": {}}
    for tag, d in [("Scalar", d_scalar), ("RVV", d_rvv)]:
        cpu_map = power_data.get('scalar' if tag == 'Scalar' else 'vector', {})
        cpu_w   = cpu_map.get(FREQ_MHZ, np.nan)

        for diff in diffs:
            ns = load_ns_for_diff(d, diff)
            if ns.size:
                ms = ns * 1e-6
                q1, med, q3 = np.percentile(ms, [25, 50, 75])
                iqr = q3 - q1
            else:
                med, iqr = np.nan, np.nan

            passes = total_passes_for_diff(d, diff)
            act_w  = actuator_power_for_diff(d, diff)
            total_w = act_w + cpu_w if (not np.isnan(act_w) and not np.isnan(cpu_w)) else np.nan

            metrics[tag][diff] = {
                "med_ms": med,
                "iqr_ms": iqr,
                "passes": passes,
                "total_w": total_w,
            }

    # ---------- plotting: grouped by compute target ----------
    fig, axes = plt.subplots(3, 1, figsize=(7.5, 10), sharex=True)

    group_labels = ["Scalar", "RVV"]
    group_centers = np.arange(len(group_labels))  # [0,1]
    width = 0.22
    offsets = (-width, 0.0, width)  # easy, medium, hard

    def _bar_with_optional_yerr(ax, xpos, heights, yerr):
        """Plot bars; if any yerr is NaN, omit error bars for that bar."""
        # Matplotlib doesn't support per-bar None in a single call cleanly,
        # so draw each bar separately.
        for xi, h, e in zip(xpos, heights, yerr):
            if np.isfinite(e):
                ax.bar(xi, h, width, yerr=e, capsize=5)
            else:
                ax.bar(xi, h, width)

    # (a) Solve time
    ax = axes[0]
    for k, diff in enumerate(diffs):
        xpos = group_centers + offsets[k]
        heights = [metrics["Scalar"][diff]["med_ms"], metrics["RVV"][diff]["med_ms"]]
        yerr    = [metrics["Scalar"][diff]["iqr_ms"], metrics["RVV"][diff]["iqr_ms"]]
        # draw with color per difficulty
        for xi, h, e in zip(xpos, heights, yerr):
            if np.isfinite(e):
                ax.bar(xi, h, width, yerr=e, capsize=5, color=colors[diff], edgecolor='black')
            else:
                ax.bar(xi, h, width, color=colors[diff], edgecolor='black')
    ax.set_ylabel("Solve Time (ms)")
    ax.set_title("(a) Median MPC Solve Time @ 100 MHz")
    ax.grid(True, axis='y', linestyle=':', alpha=0.6)

    # (b) Successful trajectories
    ax = axes[1]
    for k, diff in enumerate(diffs):
        xpos = group_centers + offsets[k]
        heights = [metrics["Scalar"][diff]["passes"], metrics["RVV"][diff]["passes"]]
        ax.bar(xpos, heights, width, color=colors[diff], edgecolor='black')
    ax.set_ylabel("Successful Trajectories")
    ax.set_title("(b) Successful Trajectories")
    ax.yaxis.set_major_locator(MultipleLocator(2))
    ax.grid(True, axis='y', linestyle=':', alpha=0.6)

    # (c) Total power
    ax = axes[2]
    for k, diff in enumerate(diffs):
        xpos = group_centers + offsets[k]
        heights = [metrics["Scalar"][diff]["total_w"], metrics["RVV"][diff]["total_w"]]
        ax.bar(xpos, heights, width, color=colors[diff], edgecolor='black')
    ax.set_ylabel("Power (W)")
    ax.set_title("(c) System Power @ 100 MHz")
    ax.grid(True, axis='y', linestyle=':', alpha=0.6)

    # shared x labels
    for ax in axes:
        ax.set_xticks(group_centers, group_labels)

    # Legend: difficulty colors
    color_handles = [Line2D([0],[0], color=colors[d], lw=6, label=d.capitalize()) for d in diffs]
    axes[2].legend(color_handles, [d.capitalize() for d in diffs],
                   title="Difficulty", loc='lower center', bbox_to_anchor=(0.5, -0.35), ncol=3)

    plt.tight_layout(rect=[0, 0.05, 1, 1])
    outpath = os.path.join(outdir, outfile)
    fig.savefig(outpath, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved figure to {outpath}")

if __name__ == "__main__":
    plot_summary(directories, power_data)