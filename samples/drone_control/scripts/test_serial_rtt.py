import numpy as np
import time
import struct
from serial import Serial

# ==== CONFIG ====
# PORT       = "/dev/ttyUSB1"
PORT = "/dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if01-port0"
BAUDRATE   = 115200
HEADER     = b'\xDE\xAD\xBE\xEF'
NSTATES    = 12  # floats per drone state
NACTIONS   = 4   # floats in action response
N_TEST_MESSAGES = 50

# UART timing
BITS_PER_BYTE = 10  # 8N1 (1 start + 8 data + 1 stop)
SECONDS_PER_BYTE = BITS_PER_BYTE / BAUDRATE

# Compute packet sizes
TX_PACKET_BYTES = len(HEADER) + 1 + 4 * NSTATES
RX_PACKET_BYTES = len(HEADER) + 1 + 4 * NACTIONS + 4  # 4 bytes timestamp (int32)

TX_TIME_SEC = TX_PACKET_BYTES * SECONDS_PER_BYTE
RX_TIME_SEC = RX_PACKET_BYTES * SECONDS_PER_BYTE
TOTAL_UART_TIME_MS = (TX_TIME_SEC + RX_TIME_SEC) * 1000.0  # in ms

# ==== Interface ====
class TinyMPCSerialInterface:
    def __init__(self, port, baudrate=BAUDRATE, timeout=0, debug=True):
        self.ser = Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(1)
        self.ser.reset_input_buffer()
        self._last_rx_time = {}
        self.debug = debug
        if self.debug:
            print(f"[TinyMPC] Serial opened on {port}@{baudrate}")

    def send_states_all(self, obs_array):
        num = obs_array.shape[0]
        packet = bytearray(HEADER)
        packet += struct.pack('<B', num)
        for i in range(num):
            packet += struct.pack(f'<{NSTATES}f', *obs_array[i])
        self.ser.write(packet)
        if self.debug:
            print(f"[TinyMPC][TX] Sent {num} states")

    def read_action(self):
        total_len = len(HEADER) + 1 + NACTIONS * 4 + 4
        if self.ser.in_waiting < total_len:
            return None
        packet = self.ser.read(total_len)
        if len(packet) < total_len or packet[:len(HEADER)] != HEADER:
            return None
        offset = len(HEADER)
        drone_id = packet[offset]
        offset += 1
        forces = struct.unpack('<4f', packet[offset:offset + NACTIONS * 4])
        offset += NACTIONS * 4
        ns = struct.unpack('<i', packet[offset:offset + 4])[0]
        return drone_id, np.array(forces), ns

    def close(self):
        self.ser.close()

# ==== RTT TEST LOGIC ====
if __name__ == "__main__":
    iface = TinyMPCSerialInterface(PORT, BAUDRATE, timeout=0, debug=False)

    dummy_obs = np.zeros((1, NSTATES), dtype=np.float32)
    rtts_ms = []
    uart_percentages = []

    print(f"Running {N_TEST_MESSAGES} UART RTT tests...")

    for i in range(N_TEST_MESSAGES):
        iface.ser.reset_input_buffer()

        iface.send_states_all(dummy_obs)
        t_start = time.perf_counter()

        # Poll for response
        deadline = t_start + 1.0
        response = None
        while time.perf_counter() < deadline:
            result = iface.read_action()
            if result is not None:
                response = result
                break
            time.sleep(0.001)

        t_end = time.perf_counter()

        if response is not None:
            rtt = (t_end - t_start) * 1000  # ms
            rtts_ms.append(rtt)
            uart_ratio = 100.0 * TOTAL_UART_TIME_MS / rtt
            uart_percentages.append(uart_ratio)
            print(f"[{i+1:02d}/{N_TEST_MESSAGES}] RTT: {rtt:.3f} ms | UART %: {uart_ratio:.2f}")
        else:
            print(f"[{i+1:02d}/{N_TEST_MESSAGES}] ❌ No response")
            rtts_ms.append(None)
            uart_percentages.append(None)

    iface.close()

    # ==== Summary ====
    rtts_valid = [r for r in rtts_ms if r is not None]
    perc_valid = [p for p in uart_percentages if p is not None]

    if rtts_valid:
        avg = np.mean(rtts_valid)
        std = np.std(rtts_valid)
        avg_uart_ratio = np.mean(perc_valid)
        print(f"\n✅ Avg RTT: {avg:.3f} ms | Std Dev: {std:.3f} ms | Success: {len(rtts_valid)}/{N_TEST_MESSAGES}")
        print(f"🧮 Avg % of RTT from physical UART: {avg_uart_ratio:.2f}% (ideal TX+RX time = {TOTAL_UART_TIME_MS:.3f} ms)")
    else:
        print("\n❌ No successful round-trip responses")
