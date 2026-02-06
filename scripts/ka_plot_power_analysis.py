#!/usr/bin/env python3

import os
import re
import csv
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path
from collections import defaultdict

plt.rcParams.update({
    "font.size": 20,
    "axes.titlesize": 20,
    "axes.labelsize": 20,
    "xtick.labelsize": 20,
    "ytick.labelsize": 20,
    "legend.fontsize": 20,
    'font.family': 'sans-serif',
    'font.sans-serif': ['Arial', 'DejaVu Sans', 'Lucida Grande', 'Verdana'] # Arial as the first choice
})

def parse_uart_file(txt_path):
    """Parse UART file and extract timestamps for measurement window markers"""
    start_time = None
    end_time = None
    
    with open(txt_path, 'r') as f:
        for line in f:
            # Match timestamp pattern: [timestamp] === Starting measurement window...
            match_start = re.match(r'\[([\d.]+)\]\s+=== Starting measurement window', line)
            if match_start:
                start_time = float(match_start.group(1))
            
            # Match timestamp pattern: [timestamp] === Ending measurement window ===
            match_end = re.match(r'\[([\d.]+)\]\s+=== Ending measurement window ===', line)
            if match_end:
                end_time = float(match_end.group(1))
    
    if start_time is not None and end_time is not None:
        return start_time, end_time
    
    return None, None

def parse_power_csv(csv_path):
    """Parse power CSV file and return arrays"""
    wall_times = []
    elapsed_times = []
    voltages = []
    currents = []
    powers = []
    
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            wall_times.append(float(row['WallTime (s)']))
            elapsed_times.append(float(row['Elapsed (s)']))
            voltages.append(float(row['Voltage (V)']))
            currents.append(float(row['Current (A)']))
            powers.append(float(row['Power (W)']))
    
    return np.array(wall_times), np.array(elapsed_times), np.array(voltages), np.array(currents), np.array(powers)

def parse_perf_from_uart(txt_path):
    """Parse UART file and compute average cycle count from the final block.

    We look for the line:
      'Clocks taken (showing last N of M iterations):'
    and then parse the subsequent 'Clocks taken (idx): value' lines.

    Glitches/artifacts are filtered out by removing values that are
    obvious outliers (e.g., orders of magnitude larger than the median).
    """
    in_block = False
    cycles = []

    with open(txt_path, 'r') as f:
        for line in f:
            if "Clocks taken (showing last" in line:
                # Start of the final block; reset any previous data
                in_block = True
                cycles = []
                continue

            if in_block:
                # Lines look like:
                # [timestamp] Clocks taken (N): VALUE
                m = re.match(r'\[([\d.]+)\]\s+Clocks taken \(\d+\):\s+(\d+)', line)
                if m:
                    value_str = m.group(2)
                    try:
                        value = int(value_str)
                        cycles.append(value)
                    except ValueError:
                        # Ignore malformed values
                        continue

    if not cycles:
        return None

    arr = np.array(cycles, dtype=np.float64)

    # Robust outlier filtering:
    # - Compute median
    # - Keep values within a reasonable factor of the median
    median = np.median(arr)
    if median <= 0:
        # Fallback: just use raw mean
        return float(arr.mean())

    # Keep values that are positive and not huge outliers.
    # Glitches in the data tend to be many orders of magnitude larger.
    # A 10x threshold relative to the median is conservative and should
    # safely remove artifacts like 4e8 or 1e19 cycles.
    lower = median / 10.0
    upper = median * 10.0
    mask = (arr > lower) & (arr < upper)
    filtered = arr[mask]

    if filtered.size == 0:
        # If filtering removed everything, fall back to unfiltered data
        filtered = arr

    return float(filtered.mean())

def extract_window_data(wall_times, powers, start_time, end_time):
    """Extract power data within the specified time window.
    With increased measurement iterations, the window is now long enough
    to capture power samples directly within it."""
    # Use only samples strictly within the measurement window
    # Since power samples are taken every ~0.25s and measurement windows are now
    # much longer (due to increased iterations), we should have samples within the window
    mask = (wall_times >= start_time) & (wall_times <= end_time)
    
    # If no samples found within window, warn but don't expand
    # This shouldn't happen with longer measurement windows
    if np.sum(mask) == 0:
        print(f"    Warning: No power samples found within measurement window "
              f"({start_time:.6f} to {end_time:.6f})")
    
    return wall_times[mask], powers[mask]

def find_matching_files():
    """Find all matching pairs of CSV and TXT files"""
    uart_files = glob.glob('uart_out/*.txt')
    power_files = glob.glob('power_traces/*.csv')
    
    matches = []
    for uart_file in uart_files:
        # Extract base name: q8_gemm_minmax_bs64_ic32_oc32_opu
        base_name = Path(uart_file).stem
        
        # Find matching CSV file
        power_file = f'power_traces/{base_name}.csv'
        if os.path.exists(power_file):
            matches.append((uart_file, power_file, base_name))
    
    return matches

def plot_full_trace(wall_times, powers, output_path):
    """Plot full power trace over time"""
    plt.figure(figsize=(12, 6))
    plt.plot(wall_times - wall_times[0], powers, linewidth=0.5, alpha=0.7)
    plt.xlabel('Time (s)', fontsize=20)
    plt.ylabel('Power (W)', fontsize=20)
    plt.title('Power Trace - Full Sample', fontsize=20)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()

def plot_window_trace(wall_times, powers, start_time, end_time, output_path):
    """Plot power trace for the specific window, showing context around it"""
    # Extract strict window data (for highlighting)
    window_times, window_powers = extract_window_data(wall_times, powers, start_time, end_time)
    
    if len(window_times) == 0:
        print(f"Warning: No data found in window for {output_path}")
        return
    
    # Calculate window duration and create a buffer for context
    window_duration = end_time - start_time
    # Show 50% of window duration on each side for context
    buffer = window_duration * 0.5
    context_start = start_time - buffer
    context_end = end_time + buffer
    
    # Extract context data (wider region for visualization)
    context_mask = (wall_times >= context_start) & (wall_times <= context_end)
    context_times = wall_times[context_mask]
    context_powers = powers[context_mask]
    
    plt.figure(figsize=(12, 6))
    
    # Plot context region (lighter, thinner)
    if len(context_times) > 0:
        plt.plot(context_times - start_time, context_powers, 
                linewidth=0.5, color='gray', alpha=0.5, label='Context')
    
    # Plot measurement window (highlighted)
    plt.plot(window_times - start_time, window_powers, 
            linewidth=2.0, color='red', marker='o', markersize=5, label='Measurement window')
    
    # Add vertical lines marking the actual benchmark window boundaries
    plt.axvline(x=0, color='green', linestyle='--', linewidth=2, label='Window start')
    plt.axvline(x=end_time - start_time, color='green', linestyle='--', linewidth=2, label='Window end')
    
    # Shade the measurement window region
    plt.axvspan(0, end_time - start_time, alpha=0.1, color='green', label='Measurement window')
    
    plt.xlabel('Time (s) relative to measurement window start', fontsize=20)
    plt.ylabel('Power (W)', fontsize=20)
    plt.title(f'Power Trace - Measurement Window\n(Benchmark: {start_time:.6f} to {end_time:.6f})', fontsize=20)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()

def create_heatmap(scalar_data, opu_data, output_path):
    """Create heatmap comparing scalar vs opu average power"""
    # Extract unique input and output channel sizes
    all_ic = sorted(set([ic for ic, oc, _ in scalar_data.keys()] + [ic for ic, oc, _ in opu_data.keys()]))
    all_oc = sorted(set([oc for ic, oc, _ in scalar_data.keys()] + [oc for ic, oc, _ in opu_data.keys()]))
    
    # Create matrices for scalar and opu
    scalar_matrix = np.full((len(all_ic), len(all_oc)), np.nan)
    opu_matrix = np.full((len(all_ic), len(all_oc)), np.nan)
    
    for (ic, oc, mode), avg_power in scalar_data.items():
        i = all_ic.index(ic)
        j = all_oc.index(oc)
        scalar_matrix[i, j] = avg_power
    
    for (ic, oc, mode), avg_power in opu_data.items():
        i = all_ic.index(ic)
        j = all_oc.index(oc)
        opu_matrix[i, j] = avg_power
    
    # Create subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    
    # Find common scale for both plots
    all_values = [v for v in scalar_data.values() if not np.isnan(v)] + [v for v in opu_data.values() if not np.isnan(v)]
    vmin = min(all_values) if all_values else 0
    vmax = max(all_values) if all_values else 1
    
    # Scalar heatmap
    im1 = ax1.imshow(scalar_matrix, aspect='auto', cmap='YlOrRd', vmin=vmin, vmax=vmax)
    ax1.set_title('Scalar Average Power (W)', fontsize=20)
    ax1.set_xlabel('Output Channels', fontsize=20)
    ax1.set_ylabel('Input Channels', fontsize=20)
    ax1.set_xticks(range(len(all_oc)))
    ax1.set_xticklabels(all_oc)
    ax1.set_yticks(range(len(all_ic)))
    ax1.set_yticklabels(all_ic)
    plt.colorbar(im1, ax=ax1, label='Power (W)')
    
    # Add text annotations
    for i in range(len(all_ic)):
        for j in range(len(all_oc)):
            if not np.isnan(scalar_matrix[i, j]):
                ax1.text(j, i, f'{scalar_matrix[i, j]:.3f}', 
                        ha='center', va='center', fontsize=20, color='black')
    
    # OPU (vector) heatmap
    im2 = ax2.imshow(opu_matrix, aspect='auto', cmap='YlOrRd', vmin=vmin, vmax=vmax)
    ax2.set_title('OPU (Vector) Average Power (W)', fontsize=20)
    ax2.set_xlabel('Output Channels', fontsize=20)
    ax2.set_ylabel('Input Channels', fontsize=20)
    ax2.set_xticks(range(len(all_oc)))
    ax2.set_xticklabels(all_oc)
    ax2.set_yticks(range(len(all_ic)))
    ax2.set_yticklabels(all_ic)
    plt.colorbar(im2, ax=ax2, label='Power (W)')
    
    # Add text annotations
    for i in range(len(all_ic)):
        for j in range(len(all_oc)):
            if not np.isnan(opu_matrix[i, j]):
                ax2.text(j, i, f'{opu_matrix[i, j]:.3f}', 
                        ha='center', va='center', fontsize=20, color='black')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()

def create_perf_heatmaps(scalar_perf, opu_perf, output_dir):
    """Create separate heatmaps for raw scalar perf, raw OPU perf, and speedup."""
    if not scalar_perf and not opu_perf:
        return

    os.makedirs(output_dir, exist_ok=True)

    # Collect all dimensions seen in either scalar or opu data
    all_ic = sorted(
        set([ic for (ic, oc) in scalar_perf.keys()] +
            [ic for (ic, oc) in opu_perf.keys()])
    )
    all_oc = sorted(
        set([oc for (ic, oc) in scalar_perf.keys()] +
            [oc for (ic, oc) in opu_perf.keys()])
    )

    if not all_ic or not all_oc:
        return

    scalar_matrix = np.full((len(all_ic), len(all_oc)), np.nan)
    opu_matrix = np.full((len(all_ic), len(all_oc)), np.nan)
    speedup_matrix = np.full((len(all_ic), len(all_oc)), np.nan)

    # Fill scalar and opu matrices
    for (ic, oc), val in scalar_perf.items():
        i = all_ic.index(ic)
        j = all_oc.index(oc)
        scalar_matrix[i, j] = val

    for (ic, oc), val in opu_perf.items():
        i = all_ic.index(ic)
        j = all_oc.index(oc)
        opu_matrix[i, j] = val

    # Compute speedup where both scalar and opu values are present
    for i, ic in enumerate(all_ic):
        for j, oc in enumerate(all_oc):
            s = scalar_matrix[i, j]
            o = opu_matrix[i, j]
            if not np.isnan(s) and not np.isnan(o) and o > 0:
                # Speedup: scalar cycles / opu cycles (>1 means opu is faster)
                speedup_matrix[i, j] = s / o

    # Common axis labels
    def _style_axis(ax, title, cbar_label):
        ax.set_title(title, fontsize=20)
        ax.set_xlabel('Output Channels', fontsize=20)
        ax.set_ylabel('Input Channels', fontsize=20)
        ax.set_xticks(range(len(all_oc)))
        ax.set_xticklabels(all_oc)
        ax.set_yticks(range(len(all_ic)))
        ax.set_yticklabels(all_ic)

    # 1) Scalar perf heatmap
    fig, ax = plt.subplots(figsize=(12, 4))
    if np.isfinite(scalar_matrix).any():
        vmin = np.nanmin(scalar_matrix)
        vmax = np.nanmax(scalar_matrix)
    else:
        vmin, vmax = 0, 1
    im = ax.imshow(scalar_matrix, aspect='auto', cmap='viridis', vmin=vmin, vmax=vmax)
    _style_axis(ax, 'Scalar Average Cycles', 'Cycles')
    cbar = plt.colorbar(im, ax=ax, label='Cycles')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'perf_scalar_heatmap.png'), dpi=150)
    plt.close(fig)

    # 2) OPU perf heatmap
    fig, ax = plt.subplots(figsize=(12, 4))
    if np.isfinite(opu_matrix).any():
        vmin = np.nanmin(opu_matrix)
        vmax = np.nanmax(opu_matrix)
    else:
        vmin, vmax = 0, 1
    im = ax.imshow(opu_matrix, aspect='auto', cmap='viridis', vmin=vmin, vmax=vmax)
    _style_axis(ax, 'OPU Average Cycles', 'Cycles')
    plt.colorbar(im, ax=ax, label='Cycles')
    plt.tight_layout(pad=2.0)
    plt.savefig(os.path.join(output_dir, 'perf_opu_heatmap.png'), dpi=300)
    plt.close(fig)

    # 3) Speedup heatmap (scalar / opu)
    fig, ax = plt.subplots(figsize=(12, 4))
    if np.isfinite(speedup_matrix).any():
        vmin = np.nanmin(speedup_matrix)
        vmax = np.nanmax(speedup_matrix)
    else:
        vmin, vmax = 1, 1
    im = ax.imshow(speedup_matrix, aspect='auto', cmap='plasma', vmin=vmin, vmax=vmax)
    _style_axis(ax, 'Speedup (Scalar Cycles / OPU Cycles)', 'Speedup')
    plt.colorbar(im, ax=ax, label='Speedup')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'perf_speedup_heatmap.png'), dpi=150)
    plt.close(fig)

def main():
    # Find all matching file pairs
    matches = find_matching_files()
    
    if not matches:
        print("No matching CSV and TXT files found!")
        return
    
    print(f"Found {len(matches)} matching file pairs")
    
    # Data structures for power heatmap
    scalar_data = {}  # (ic, oc, mode) -> average power
    opu_data = {}     # (ic, oc, mode) -> average power

    # Data structures for performance heatmaps (cycles)
    scalar_perf = {}  # (ic, oc) -> avg cycles
    opu_perf = {}     # (ic, oc) -> avg cycles
    
    # Process each pair
    for uart_file, power_file, base_name in matches:
        print(f"Processing {base_name}...")
        
        # Parse files
        start_time, end_time = parse_uart_file(uart_file)
        if start_time is None or end_time is None:
            print(f"  Warning: Could not find measurement window markers in {uart_file}, skipping")
            continue
        
        wall_times, elapsed_times, voltages, currents, powers = parse_power_csv(power_file)
        
        if len(wall_times) == 0:
            print(f"  Warning: No power data in {power_file}, skipping")
            continue
        
        print(f"  Benchmark window: {start_time:.6f} to {end_time:.6f} (duration: {end_time-start_time:.6f}s)")
        print(f"  Power data range: {wall_times[0]:.6f} to {wall_times[-1]:.6f} ({len(wall_times)} samples)")
        
        # Extract parameters from filename
        # Format: q8_gemm_minmax_bs64_ic32_oc32_opu
        match = re.match(r'q8_gemm_minmax_bs64_ic(\d+)_oc(\d+)_(scalar|opu)', base_name)
        if not match:
            print(f"  Warning: Could not parse filename {base_name}, skipping")
            continue
        
        ic = int(match.group(1))
        oc = int(match.group(2))
        mode = match.group(3)
        
        # Create output directory
        plot_dir = f'plots/{base_name}'
        os.makedirs(plot_dir, exist_ok=True)
        
        # Plot 1: Full power trace
        full_trace_path = f'{plot_dir}/power_trace_full.png'
        plot_full_trace(wall_times, powers, full_trace_path)
        print(f"  Generated: {full_trace_path}")
        
        # Plot 2: Window power trace
        window_trace_path = f'{plot_dir}/power_trace_window.png'
        plot_window_trace(wall_times, powers, start_time, end_time, window_trace_path)
        print(f"  Generated: {window_trace_path}")
        
        # Extract window data and calculate average power
        window_times, window_powers = extract_window_data(wall_times, powers, start_time, end_time)
        if len(window_powers) > 0:
            print(f"  Window power samples: {len(window_powers)} samples")
            avg_power = np.mean(window_powers)
            print(f"  Average power: {avg_power:.6f} W")
            if mode == 'scalar':
                scalar_data[(ic, oc, mode)] = avg_power
            elif mode == 'opu':
                opu_data[(ic, oc, mode)] = avg_power
        else:
            print(f"  Warning: No power samples found in window")

        # Parse performance (cycles) from UART and store for perf heatmaps
        avg_cycles = parse_perf_from_uart(uart_file)
        if avg_cycles is not None:
            print(f"  Average cycles (filtered): {avg_cycles:.2f}")
            if mode == 'scalar':
                scalar_perf[(ic, oc)] = avg_cycles
            elif mode == 'opu':
                opu_perf[(ic, oc)] = avg_cycles
    
    # Create power heatmap if we have data
    if scalar_data or opu_data:
        os.makedirs('plots', exist_ok=True)
        heatmap_path = 'plots/power_heatmap_comparison.png'
        create_heatmap(scalar_data, opu_data, heatmap_path)
        print(f"\nGenerated: {heatmap_path}")

    # Create performance heatmaps if we have data
    if scalar_perf or opu_perf:
        create_perf_heatmaps(scalar_perf, opu_perf, 'plots')
    
    print(f"\nDone! Processed {len(matches)} file pairs")

if __name__ == "__main__":
    main()
