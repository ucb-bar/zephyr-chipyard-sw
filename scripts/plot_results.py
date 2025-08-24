import os
import re
import numpy as np
import matplotlib.pyplot as plt
import json



import matplotlib
matplotlib.use("Agg")   # headless backend

from matplotlib.ticker import MaxNLocator
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
    "data/full/data_ideal",
    "data/full/data_scalar_50mhz",
    "data/full/data_scalar_75mhz",
    "data/full/data_scalar_100mhz",
    "data/full/data_scalar_125mhz", 
    "data/full/data_scalar_150mhz", 
    "data/full/data_scalar_200mhz", 
    "data/full/data_scalar_250mhz", 
    "data/full/data_scalar_375mhz", 
    "data/full/data_scalar_500mhz", 
    "data/full/data_rvv_handopt_50mhz",
    "data/full/data_rvv_handopt_75mhz",
    "data/full/data_rvv_handopt_100mhz", 
    "data/full/data_rvv_handopt_125mhz", 
    "data/full/data_rvv_handopt_150mhz", 
    "data/full/data_rvv_handopt_200mhz", 
    "data/full/data_rvv_handopt_250mhz", 
    "data/full/data_rvv_handopt_375mhz", 
    "data/full/data_rvv_handopt_500mhz", 
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
                 outdir="results/fig16",
                 outfile="fig16.png"):
    """
    Three‐panel figure:
      (a) median solve time ± IQR vs frequency
      (b) successful trajectories vs frequency (scalar/vector + ideal)
      (c) total power vs frequency with shaded gap + ideal baselines
    
    A shared legend is placed below all subplots. All text is scaled
    by a common `font_scale` factor for easy resizing.
    """
    # --- font scaling ---
    font_scale    = 1.5
    base_fs       = plt.rcParams.get('font.size', 12)
    big_fs        = base_fs * font_scale

    # --- ideal actuator‐only baseline ---
    ideal_dirs = [d for d in directories if 'ideal' in d]
    if not ideal_dirs:
        raise ValueError("No 'ideal' directory found")
    ideal_avg = compute_avg_power(ideal_dirs[0],
                                  hover_rpm, hover_power, max_rpm)

    # --- parse directories for (dir, freq(MHz), impl) ---
    # Example matches: "_400mhz_", "_400mhz", "_400.0mhz_", "_400.0mhz"
    mhz_re = re.compile(r'_(\d+(?:\.\d+)?)mhz(?:_|$)', re.IGNORECASE)

    impls = []
    freqs = set()
    for d in directories:
        if 'ideal' in d:
            continue
        if 'scalar' in d:
            meas = 'scalar'
        elif 'rvv_handopt' in d:
            meas = 'vector'
        else:
            continue

        m = mhz_re.search(d)
        if not m:
            # Skip if we can't parse MHz from the directory name
            continue

        freq = float(m.group(1))  # already MHz; no scale division now
        impls.append((d, freq, meas))
        freqs.add(freq)

    freqs = sorted(freqs)
    diffs  = ['easy','medium','hard']
    colors = {'easy':'green','medium':'blue','hard':'red'}

    # --- 1) ideal pass counts + actual pass counts ---
    ideal_dir = ideal_dirs[0]
    ideal_pass_counts = {}
    for diff in diffs:
        cnt = 0
        for fn in os.listdir(ideal_dir):
            if fn.startswith(diff + '_') and fn.endswith('.npy'):
                if trajectory_passed_file(os.path.join(ideal_dir, fn)):
                    cnt += 1
        ideal_pass_counts[diff] = cnt

    pass_counts = {meas:{diff:[] for diff in diffs}
                   for meas in ('scalar','vector')}
    for freq in freqs:
        for meas in ('scalar','vector'):
            ds = [d for d,f,m in impls if m==meas and f==freq]
            for diff in diffs:
                cnt = 0
                for d in ds:
                    for fn in os.listdir(d):
                        if fn.startswith(diff + '_') and fn.endswith('.npy'):
                            if trajectory_passed_file(os.path.join(d, fn)):
                                cnt += 1
                pass_counts[meas][diff].append(cnt)

    # --- 2) solve‐time quantiles (ms) ---
    solve_stats = {(meas, diff): []
                   for meas in ('scalar','vector') for diff in diffs}
    for d, freq, meas in impls:
        for diff in diffs:
            all_ns = []
            for fn in os.listdir(d):
                if not fn.startswith(diff + '_') or not fn.endswith('.npy'):
                    continue
                data = np.load(os.path.join(d, fn), allow_pickle=False)
                valid = data['ns'][data['ns'] >= 0]
                if valid.size:
                    all_ns.append(valid)
            if not all_ns:
                continue
            all_ns = np.concatenate(all_ns) * 1e-6  # ms
            all_ns = all_ns * 0.2 # apply time scale
            q1, med, q3 = np.percentile(all_ns, [25,50,75])
            solve_stats[(meas, diff)].append((freq, med, q1, q3))

    # --- 3) total power (actuator + CPU) ---
    power_tot = {(meas, diff): []
                 for meas in ('scalar','vector') for diff in diffs}
    for d, freq, meas in impls:
        cpu_map = power_data.get(meas, {})
        cpu_p   = cpu_map.get(freq)
        if cpu_p is None:
            continue
        ap = compute_avg_power(d, hover_rpm, hover_power, max_rpm)
        for diff in diffs:
            act_p = ap[diff]
            if np.isnan(act_p):
                continue
            power_tot[(meas, diff)].append((freq, act_p + cpu_p))

    # --- build figure & axes ---
    # fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 12), sharex=True)
    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1,
        figsize=(9, 10),
        sharex=True,
        gridspec_kw={'height_ratios': [0.8, 0.8, 1]}  # e.g. 4:2:1 ratio
    )

    # scale tick labels
    for ax in (ax1, ax2, ax3):
        ax.tick_params(axis='both', which='major', labelsize=big_fs)

    # --- subplot (a): median solve time ± IQR ---
    for meas, ls in [('scalar','--'), ('vector','-')]:
        for diff in diffs:
            stats = sorted(solve_stats[(meas, diff)], key=lambda x: x[0])
            if not stats:
                continue
            xs, meds, p25s, p75s = zip(*stats)
            ax1.fill_between(xs, p25s, p75s,
                             color=colors[diff], alpha=0.2)
            ax1.plot(xs, meds,
                     linestyle=ls,
                     linewidth=2,
                     color=colors[diff],
                     label=f"{diff.capitalize()} {meas}")

    # draw simulation latency line at 1.25 ms (black dotted)
    lat_line = ax1.axhline(
        1.25,
        color='k',
        linestyle=':',
        linewidth=2,
        label='Simulation Latency'
    )

    ax1.set_ylabel("Solve Time (ms)", fontsize=big_fs)
    ax1.set_title("(a) MPC Solve Time vs Frequency (median ± IQR)",
                  fontsize=big_fs)
    ax1.set_yscale('log')
    ax1.grid(True)

    # legend only for the latency line, in upper right
    ax1.legend(
        handles=[lat_line],
        loc='upper right',
        fontsize=big_fs * 0.8,
        title=None
    )

    # --- subplot (b): pass counts + ideal lines ---
    for diff, col in colors.items():
        ax2.axhline(ideal_pass_counts[diff],
                    color=col,
                    linestyle=':',
                    linewidth=2,
                    label=f"{diff.capitalize()} ideal")
    for meas, ls in [('scalar','--'), ('vector','-')]:
        for diff in diffs:
            ax2.plot(freqs,
                     pass_counts[meas][diff],
                     linestyle=ls,
                     color=colors[diff],
                     linewidth=2,
                     marker='o',
                     label=f"{diff.capitalize()} {meas}")
    ax2.set_ylabel("Successful Trajectories", fontsize=big_fs)
    ax2.set_title("(b) Successful Trajectories vs Frequency",
                  fontsize=big_fs)
    ax2.yaxis.set_major_locator(MultipleLocator(2))
    ax2.set_ylim(-1, 21)
    ax2.grid(True)

    # --- subplot (c): total power + shaded gap + ideal ---
    for diff in diffs:
        ax3.axhline(ideal_avg[diff],
                    color=colors[diff],
                    linestyle=':',
                    linewidth=2,
                    label=f"{diff.capitalize()} ideal")
    for meas, ls in [('scalar','--'), ('vector','-')]:
        for diff in diffs:
            data = sorted(power_tot[(meas, diff)], key=lambda x: x[0])
            if not data:
                continue
            xs, Pt = zip(*data)
            # compute corresponding actuator-only for shading
            Pa = []
            for x in xs:
                d_list = [d for d,f,m in impls if m==meas and f==x]
                Pa.append(compute_avg_power(d_list[0],
                                           hover_rpm, hover_power, max_rpm)[diff])
            ax3.fill_between(xs, Pa, Pt,
                             color=colors[diff], alpha=0.2, edgecolor='none')
            ax3.plot(xs, Pt,
                     linestyle=ls,
                     color=colors[diff],
                     linewidth=2,
                     label=f"{diff.capitalize()} {meas} total")

    ax3.set_xlabel("SoC Frequency (MHz)", fontsize=big_fs)
    ax3.set_ylabel("Power (W)",               fontsize=big_fs)
    ax3.set_title("(c) System Power Consumption vs Frequency",
                  fontsize=big_fs)
    ax3.grid(True)

    # --- shared legend below ---
    handles, labels = ax3.get_legend_handles_labels()
    ordered = []
    for diff in diffs:
        for tag in ['ideal', 'scalar total', 'vector total']:
            name = f"{diff.capitalize()} {tag}"
            if name in labels:
                idx = labels.index(name)
                ordered.append((handles[idx], labels[idx]))
    hs, ls = zip(*ordered)
    # fig.legend(hs, ls,
    #            loc='lower center',
    #            bbox_to_anchor=(0.5, -0.02),
    #            ncol=3,
    #            frameon=False,
    #            fontsize=big_fs)

    # 1) Create proxy artists for difficulty (colors)
    difficulty_handles = [
        Line2D([0], [0], color=colors[diff], lw=4) 
        for diff in diffs
    ]
    difficulty_labels = [diff.capitalize() for diff in diffs]

    # 2) Create proxy artists for line‐types (patterns), all in black
    pattern_handles = [
        Line2D([0],[0], color='k', lw=2, linestyle=':'),
        Line2D([0],[0], color='k', lw=2, linestyle='--'),
        Line2D([0],[0], color='k', lw=2, linestyle='-'),
    ]
    pattern_labels = ['Ideal', 'Scalar', 'Vector']

    # 3) Add the first legend (colors) and keep it
    leg1 = ax3.legend(
        difficulty_handles, difficulty_labels,
        title="Scenario Difficulty",
        loc='lower left',
        bbox_to_anchor=(0, -0.5),
        ncol=3,
        fontsize=big_fs * 0.8,
        title_fontsize=big_fs * 0.9
    )
    ax3.add_artist(leg1)

    # 4) Add the second legend (patterns)
    ax3.legend(
        pattern_handles, pattern_labels,
        title="Compute Target",
        loc='lower right',
        bbox_to_anchor=(1, -0.5),
        ncol=3,
        fontsize=big_fs * 0.8,
        title_fontsize=big_fs * 0.9
    )


    # plt.tight_layout()
    plt.tight_layout(rect=[0, 0.1, 1, 1])
    plt.subplots_adjust(bottom=0.15)

    # save instead of show
    os.makedirs(outdir, exist_ok=True)
    outpath = os.path.join(outdir, outfile)
    fig.savefig(outpath, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved HIL results to {outpath}")

if __name__ == "__main__":
    plot_summary(directories, power_data)