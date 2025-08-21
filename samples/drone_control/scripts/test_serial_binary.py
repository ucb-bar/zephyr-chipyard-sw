import serial
import threading
import struct
import sys

# ==== CONFIGURE THIS ====
PORT      = "/dev/ttyUSB1"  # Update for your board
BAUDRATE  = 115200
HEADER    = b'\xDE\xAD\xBE\xEF'  # 4-byte packet marker; change as needed
HDR_LEN   = len(HEADER)
STATE_LEN = 12   # number of floats per drone state
ACT_LEN   = 4    # number of floats in action response

# Now expecting 4 floats + 4-byte int32 (ns)
TIMING_LEN = 4   # int32 nanoseconds

# ==== Connect ====
ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)

def read_from_serial():
    buf = bytearray()
    while True:
        try:
            chunk = ser.read(64)
            if not chunk:
                continue
            buf.extend(chunk)
            # search for HEADER in buffer
            while True:
                idx = buf.find(HEADER)
                if idx < 0:
                    # keep last few bytes in case HEADER spans reads
                    if len(buf) > HDR_LEN:
                        buf[:] = buf[-HDR_LEN:]
                    break
                # after header: 1 byte ID + 4*4 floats + 4 bytes timing
                packet_len = HDR_LEN + 1 + ACT_LEN*4 + TIMING_LEN
                if len(buf) < idx + packet_len:
                    # wait for more data
                    break
                # extract packet
                packet = buf[idx:idx+packet_len]
                del buf[:idx+packet_len]

                # parse payload
                # HEADER | id | actions_blob | timing_bytes
                id_off = HDR_LEN
                actions_off = id_off + 1
                timing_off = actions_off + ACT_LEN*4

                drone_id = packet[id_off]
                actions_blob = packet[actions_off:actions_off + ACT_LEN*4]
                timing_blob  = packet[timing_off:timing_off + TIMING_LEN]

                actions = struct.unpack('<4f', actions_blob)
                ns = struct.unpack('<i', timing_blob)[0]
                # print
                print(f"\r[SoC] ID={drone_id}  actions={list(actions)}  time_ns={ns}\n> ", end="")
        except serial.SerialException:
            break

# ==== Start background RX thread ====
rx_thread = threading.Thread(target=read_from_serial, daemon=True)
rx_thread.start()

print("=== TinyMPC Serial Console (binary mode) ===")
print("Enter: <num_drones> <12 floats> ; Ctrl+C to quit.\n")

try:
    while True:
        line = input("> ").strip()
        if not line:
            continue

        parts = line.split()
        if len(parts) != 1 + STATE_LEN:
            print(f"⚠️  Expected {1+STATE_LEN} numbers, got {len(parts)}")
            continue

        # parse user input
        num_drones = int(parts[0])
        state = [float(x) for x in parts[1:]]

        # build payload:
        #   1 byte num_drones,
        #   then STATE_LEN floats, repeated num_drones times
        payload = struct.pack('<B', num_drones)
        one_block = struct.pack(f'<{STATE_LEN}f', *state)
        payload += one_block * num_drones

        packet = HEADER + payload
        ser.write(packet)

except KeyboardInterrupt:
    print("\nExiting serial console.")
finally:
    ser.close()
