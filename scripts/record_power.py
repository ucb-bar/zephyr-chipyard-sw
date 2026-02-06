import pyvisa
import time
import csv
import argparse
import signal
import sys

# Global flag for graceful shutdown
shutdown_requested = False

def signal_handler(sig, frame):
    global shutdown_requested
    shutdown_requested = True

# Register signal handlers for graceful shutdown
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser(description="Record power measurements from PSU")
parser.add_argument('--output', '-o', default='power_log.csv', help='Output CSV file path')
args = parser.parse_args()

rm = pyvisa.ResourceManager()
psu = rm.open_resource('USB0::0x2A8D::0x8F01::CN63420433::INSTR')

psu.timeout = 10000      # ms

# Select channel
psu.write('INST:NSEL 1')

# Set measurement speed to fastest
psu.write('SENS:AVER OFF')

# Try to use combined measurement if available, otherwise fall back to separate queries
# Many PSUs support MEAS:ALL? or similar combined commands
try:
    # Test if combined measurement works
    test_result = psu.query('MEAS:ALL?')
    use_combined = True
    print("Using combined MEAS:ALL? command for faster sampling", file=sys.stderr)
except:
    use_combined = False
    print("Using separate MEAS:VOLT? and MEAS:CURR? commands", file=sys.stderr)

# Buffer for CSV writes to reduce I/O overhead
csv_buffer = []
buffer_size = 100  # Flush every N samples
print_interval = 0.1  # Print every 0.1 seconds
last_print_time = 0

with open(args.output, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['WallTime (s)', 'Elapsed (s)', 'Voltage (V)', 'Current (A)', 'Power (W)'])

    start_time = time.time()
    sample_count = 0

    print(f"Recording power to {args.output}...", file=sys.stderr)

    try:
        while not shutdown_requested:
            sample_start = time.time()
            
            if use_combined:
                # Try to parse combined measurement (format varies by instrument)
                # Common formats: "V,I" or "V I" or "V;I"
                try:
                    result = psu.query('MEAS:ALL?')
                    # Try different delimiters
                    if ',' in result:
                        parts = result.strip().split(',')
                    elif ';' in result:
                        parts = result.strip().split(';')
                    else:
                        parts = result.strip().split()
                    voltage = float(parts[0])
                    current = float(parts[1])
                except:
                    # Fallback to separate queries if parsing fails
                    voltage = float(psu.query('MEAS:VOLT?'))
                    current = float(psu.query('MEAS:CURR?'))
            else:
                voltage = float(psu.query('MEAS:VOLT?'))
                current = float(psu.query('MEAS:CURR?'))
            
            power = voltage * current
            wall_time = time.time()  # Wall clock time for synchronization
            elapsed = wall_time - start_time

            # Buffer CSV writes - include both wall time and elapsed time
            csv_buffer.append([wall_time, elapsed, voltage, current, power])
            
            # Flush buffer periodically
            if len(csv_buffer) >= buffer_size:
                writer.writerows(csv_buffer)
                f.flush()  # Ensure data is written to disk
                csv_buffer = []

            # Print less frequently to reduce overhead
            current_time = time.time()
            if current_time - last_print_time >= print_interval:
                print(f"{elapsed:.2f}s | {voltage:.3f} V | {current:.3f} A | {power:.3f} W | {sample_count/elapsed:.1f} Hz", file=sys.stderr)
                last_print_time = current_time

            sample_count += 1
            
            # Remove sleep - let the instrument and system determine the maximum rate
            # The bottleneck will be the instrument's measurement speed, not sleep

    except (KeyboardInterrupt, SystemExit):
        shutdown_requested = True
    
    # Flush remaining buffer
    if csv_buffer:
        writer.writerows(csv_buffer)
        f.flush()
    
    if sample_count > 0:
        print(f"Recording stopped. Recorded {sample_count} samples over {elapsed:.2f}s ({sample_count/elapsed:.1f} Hz average)", file=sys.stderr)

