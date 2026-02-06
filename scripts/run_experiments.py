#!/usr/bin/env python3

import argparse
import subprocess
import os
import signal
import pty
import select
import fcntl

def main():
    parser = argparse.ArgumentParser(description="Run benchmark experiments")
    parser.add_argument('--quick', action='store_true', help="Run only the smallest OPU benchmark for testing")
    args = parser.parse_args()

    input_sizes = [32, 64, 128, 256, 512, 1024, 2048]
    output_sizes = [32, 64, 128, 256]

    if args.quick:
        # Run only smallest OPU
        runs = [(32, 32, 'opu')]
    else:
        # Full loop: input outer, output inner, alternate scalar/opu
        runs = []
        for ic in input_sizes:
            for oc in output_sizes:
                runs.append((ic, oc, 'scalar'))
                runs.append((ic, oc, 'opu'))

    os.makedirs('uart_out', exist_ok=True)
    os.makedirs('power_traces', exist_ok=True)

    for ic, oc, mode in runs:
        elf_path = f"build_bmark_long/q8_gemm_minmax_bs64_ic{ic}_oc{oc}_{mode}.elf"
        if not os.path.exists(elf_path):
            print(f"Skipping {elf_path}, file not found")
            continue

        print(f"Running {ic}x{oc} {mode}")

        # Reset devices
        print("Resetting devices...")
        subprocess.run(['python3', 'scripts/send_all.py'], check=True)

        # Start power measurement in parallel
        power_file = f"power_traces/q8_gemm_minmax_bs64_ic{ic}_oc{oc}_{mode}.csv"
        print(f"Starting power measurement, output to {power_file}")
        power_proc = subprocess.Popen([
            'python3', 'scripts/record_power.py',
            '--output', power_file
        ], stderr=subprocess.PIPE, text=True)
        
        # Give power measurement a moment to initialize
        import time
        time.sleep(0.5)

        # Run benchmark
        out_file = f"uart_out/q8_gemm_minmax_bs64_ic{ic}_oc{oc}_{mode}.txt"
        print(f"Flashing and running {elf_path}, output to {out_file}")
        
        import time
        import sys
        
        # Use pseudo-terminal to force line buffering (subprocess thinks it's writing to a terminal)
        master_fd, slave_fd = pty.openpty()
        
        proc = subprocess.Popen([
            'pyuartsi',
            '--port', '/dev/ttyUSB2',
            '--elf', elf_path,
            '--load',
            '--hart0_msip',
            '--fesvr',
            '--baudrate', '57600',
            '--cflush_addr', '0x2010200',
            '--use_symbols',
            '--selfcheck'
        ], stdout=slave_fd, stderr=slave_fd, stdin=subprocess.DEVNULL)
        
        # Close slave_fd in parent process
        os.close(slave_fd)
        
        # Make master_fd non-blocking for better responsiveness
        import fcntl
        flags = fcntl.fcntl(master_fd, fcntl.F_GETFL)
        fcntl.fcntl(master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        
        with open(out_file, 'w', buffering=1) as f:  # Line buffered file
            buffer = b''
            while True:
                # Check if process has finished
                if proc.poll() is not None:
                    # Read any remaining data
                    try:
                        remaining = os.read(master_fd, 4096)
                        if remaining:
                            buffer += remaining
                    except OSError:
                        pass
                    # Process remaining buffer
                    if buffer:
                        for line in buffer.decode('utf-8', errors='replace').splitlines(keepends=True):
                            timestamp = time.time()
                            timestamped_line = f"[{timestamp:.6f}] {line}"
                            print(line, end='', flush=True)
                            f.write(timestamped_line)
                            f.flush()
                    break
                
                # Use select to wait for data (with timeout for responsiveness)
                ready, _, _ = select.select([master_fd], [], [], 0.1)
                if ready:
                    try:
                        data = os.read(master_fd, 4096)
                        if not data:
                            break
                        buffer += data
                        
                        # Process complete lines
                        while b'\n' in buffer:
                            line_bytes, buffer = buffer.split(b'\n', 1)
                            line = line_bytes.decode('utf-8', errors='replace') + '\n'
                            
                            # Timestamp immediately when line is received
                            timestamp = time.time()
                            timestamped_line = f"[{timestamp:.6f}] {line}"
                            print(line, end='', flush=True)
                            f.write(timestamped_line)
                            f.flush()
                    except OSError:
                        break
        
        os.close(master_fd)
        
        proc.wait()
        if proc.returncode != 0:
            # Stop power measurement before raising error
            power_proc.terminate()
            power_proc.wait()
            raise subprocess.CalledProcessError(proc.returncode, proc.args)

        # Stop power measurement
        print("Stopping power measurement...")
        power_proc.terminate()
        try:
            power_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            # Force kill if it doesn't terminate gracefully
            power_proc.kill()
            power_proc.wait()
        
        # Print any error output from power measurement
        _, stderr = power_proc.communicate()
        if stderr:
            print("Power measurement output:", stderr)

        print(f"Completed {ic}x{oc} {mode}")

if __name__ == "__main__":
    main()