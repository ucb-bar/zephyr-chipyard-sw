#!/usr/bin/env python3
"""
riskybird v3 -- CONTROLLER / VISUALIZER ground-station panel (WiFi).

Join the drone's SoftAP ("riskybird-<id>", open network -- see telem_wifi.c), then run this and
open the printed URL in a browser. It is a self-contained bridge + dashboard:

  * binds UDP :14550 and parses the 50 Hz telemetry downlink (the v1.1 text line from
    telem_wifi.c: it=/roll=/.../u=[...] vx=/vy=/vz=/zsp=/vbat=/st=),
  * serves a live dashboard over HTTP (attitude, altitude, drift/velocity, motors, battery,
    flight state, link stats) via Server-Sent Events -- NO external deps (stdlib only),
  * sends uplink commands to UDP :14551 (ESTOP / DISARM / HOVER_Z <mm> / PING) from the
    dashboard's buttons and shows the firmware's ACK.

Stdlib only (socket, threading, http.server, json) -- runs anywhere Python 3.8+ is, no pip.

Usage:
    python3 riskybird_panel.py                     # defaults: telem :14550, cmd->192.168.4.1:14551, UI :8080
    python3 riskybird_panel.py --drone 192.168.4.1 --http 8080
    python3 riskybird_panel.py --telem-port 14550 --cmd-port 14551

Then open  http://127.0.0.1:8080  (the URL is printed on start).
"""
import argparse
import csv
import datetime
import json
import math
import os
import re
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- telemetry wire format (telem_wifi.c telem_format_v1) -------------------------------------
_F = r"(-?\d+\.\d+)"
CORE_RE = re.compile(
    r"it=(-?\d+)\s+dt=(-?\d+)ms\s+"
    r"roll=" + _F + r"\s+pitch=" + _F + r"\s+yaw=" + _F + r"\s+"
    r"z=" + _F + r"\s+tofv=(\d+)\s+tofh=" + _F + r"\s+"
    r"u=\[" + _F + r"\s+" + _F + r"\s+" + _F + r"\s+" + _F + r"\]"
)
# enriched tail (present on v1.1 firmware; optional so older builds still parse)
TAIL_RE = re.compile(
    r"vx=" + _F + r"\s+vy=" + _F + r"\s+vz=" + _F + r"\s+"
    r"zsp=" + _F + r"\s+vbat=" + _F + r"\s+st=(\d+)"
)
# dead-reckoned position (present on firmware >= the 3D-view build; \b so it doesn't match vx=)
POS_RE = re.compile(r"\bx=" + _F + r"\s+y=" + _F)

FLAG_ARMED, FLAG_ESTOP, FLAG_ARMING, FLAG_CALDONE = 1, 2, 4, 8


def gibbs_to_quat_euler(rx, ry, rz):
    """The FC emits attitude as the Gibbs/Rodrigues vector (q_xyz / q_w), which is singular near
    180 deg (q_w -> 0). Reconstruct the true unit quaternion (exact) + proper ZYX Euler for a
    display that is correct at ALL attitudes (incl. inverted on the bench)."""
    n2 = rx * rx + ry * ry + rz * rz
    qw = 1.0 / math.sqrt(1.0 + n2)
    qx, qy, qz = rx * qw, ry * qw, rz * qw
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return qw, qx, qy, qz, roll, pitch, yaw


def parse_line(line):
    m = CORE_RE.search(line)
    if not m:
        return None
    g = m.groups()
    d = {
        "it": int(g[0]), "dt_ms": int(g[1]),
        "roll": float(g[2]), "pitch": float(g[3]), "yaw": float(g[4]),
        "z": float(g[5]), "tofv": int(g[6]), "tofh": float(g[7]),
        "u": [float(g[8]), float(g[9]), float(g[10]), float(g[11])],
        "vx": None, "vy": None, "vz": None, "zsp": None, "vbat": None, "st": 0,
        "x": None, "y": None,
    }
    # roll/pitch/yaw from the wire are the Gibbs vector -> reconstruct the true quaternion + Euler.
    qw, qx, qy, qz, roll, pitch, yaw = gibbs_to_quat_euler(d["roll"], d["pitch"], d["yaw"])
    d["qw"], d["qx"], d["qy"], d["qz"] = qw, qx, qy, qz
    d["roll"], d["pitch"], d["yaw"] = roll, pitch, yaw
    t = TAIL_RE.search(line)
    if t:
        tg = t.groups()
        d.update(vx=float(tg[0]), vy=float(tg[1]), vz=float(tg[2]),
                 zsp=float(tg[3]), vbat=float(tg[4]), st=int(tg[5]))
    p = POS_RE.search(line)
    if p:
        d["x"] = float(p.group(1)); d["y"] = float(p.group(2))
    st = d["st"]
    d["armed"] = bool(st & FLAG_ARMED)
    d["estop"] = bool(st & FLAG_ESTOP)
    d["arming"] = bool(st & FLAG_ARMING)
    d["caldone"] = bool(st & FLAG_CALDONE)
    return d


# ---- shared state ----------------------------------------------------------------------------
class Hub:
    def __init__(self):
        self.cond = threading.Condition()
        self.latest = None          # last parsed telemetry dict
        self.count = 0              # total telemetry packets
        self.rate = 0.0             # EMA packets/sec
        self.last_rx = 0.0          # monotonic time of last packet
        self.last_client = None     # (ip, port) the telemetry came from (for reference)
        self._prev_rx = None

    def on_packet(self, d, src):
        now = time.monotonic()
        with self.cond:
            if self._prev_rx is not None:
                dt = now - self._prev_rx
                if dt > 0:
                    inst = 1.0 / dt
                    self.rate = inst if self.rate == 0 else (0.85 * self.rate + 0.15 * inst)
            self._prev_rx = now
            self.last_rx = now
            self.count += 1
            self.latest = d
            self.last_client = src
            self.cond.notify_all()

    def snapshot(self):
        now = time.monotonic()
        with self.cond:
            age_ms = None if self.last_rx == 0 else int((now - self.last_rx) * 1000)
            connected = age_ms is not None and age_ms < 750
            link = {
                "connected": connected,
                "rate": round(self.rate, 1) if connected else 0.0,
                "count": self.count,
                "age_ms": age_ms,
                "quality": min(100, int(round(self.rate / 50.0 * 100))) if connected else 0,
            }
            return {"link": link, "t": self.latest}

    def wait(self, timeout):
        with self.cond:
            self.cond.wait(timeout=timeout)


HUB = Hub()


# ---- UDP telemetry receiver ------------------------------------------------------------------
# ---- session CSV log (timestamped, one file per run) -----------------------------------------
LOG_FIELDS = ["host_ts", "host_iso", "it", "dt_ms", "roll", "pitch", "yaw",
              "qw", "qx", "qy", "qz", "x", "y", "z",
              "tofv", "tofh", "vx", "vy", "vz", "zsp", "vbat",
              "u0", "u1", "u2", "u3", "st", "armed", "estop", "arming", "caldone"]


class Logger:
    """Append every parsed telemetry packet to a timestamped CSV (host clock + all fields)."""
    def __init__(self, path):
        self.path = path
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(LOG_FIELDS)
        self.n = 0

    def write(self, d):
        now = time.time()
        self.w.writerow([
            f"{now:.3f}", datetime.datetime.now().isoformat(timespec="milliseconds"),
            d["it"], d["dt_ms"], d["roll"], d["pitch"], d["yaw"],
            d["qw"], d["qx"], d["qy"], d["qz"], d["x"], d["y"], d["z"],
            d["tofv"], d["tofh"], d["vx"], d["vy"], d["vz"], d["zsp"], d["vbat"],
            d["u"][0], d["u"][1], d["u"][2], d["u"][3], d["st"],
            int(d["armed"]), int(d["estop"]), int(d["arming"]), int(d["caldone"]),
        ])
        self.n += 1
        if self.n % 25 == 0:      # flush a few times/second so a kill loses < 0.5 s
            self.f.flush()


LOGGER = None   # set in main() unless --no-log


def udp_rx_thread(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    print(f"[panel] telemetry: listening on UDP 0.0.0.0:{port}")
    while True:
        try:
            data, src = s.recvfrom(2048)
        except OSError:
            continue
        for line in data.decode("utf-8", "replace").splitlines():
            d = parse_line(line)
            if d:
                HUB.on_packet(d, f"{src[0]}:{src[1]}")
                if LOGGER is not None:
                    try:
                        LOGGER.write(d)
                    except Exception:
                        pass


# ---- uplink command sender -------------------------------------------------------------------
def send_command(drone_ip, cmd_port, text):
    """Send one command datagram to the drone and wait briefly for the firmware ACK."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.4)
    try:
        s.sendto(text.encode("ascii", "ignore"), (drone_ip, cmd_port))
        try:
            reply, _ = s.recvfrom(256)
            return reply.decode("utf-8", "replace").strip()
        except socket.timeout:
            return "(no ACK -- sent, drone may not be joined)"
    except OSError as e:
        return f"(send error: {e})"
    finally:
        s.close()


# ---- HTTP server -----------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass  # quiet

    def _send(self, code, ctype, body, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            try:
                with open(os.path.join(HERE, "panel.html"), "rb") as f:
                    body = f.read()
            except OSError:
                body = b"<h1>panel.html not found next to riskybird_panel.py</h1>"
            self._send(200, "text/html; charset=utf-8", body)
        elif self.path == "/stream":
            self._stream()
        elif self.path == "/config":
            body = json.dumps({"drone": self.server.drone_ip, "cmd_port": self.server.cmd_port}).encode()
            self._send(200, "application/json", body)
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self):
        if self.path != "/cmd":
            self._send(404, "text/plain", b"not found")
            return
        n = int(self.headers.get("Content-Length", 0))
        cmd = self.rfile.read(n).decode("utf-8", "replace").strip()
        # whitelist: only forward known commands
        head = cmd.split()[0].upper() if cmd else ""
        if head not in ("ESTOP", "DISARM", "RESET", "HOVER_Z", "PROFILE", "PING"):
            self._send(400, "application/json", json.dumps({"ack": "(rejected: unknown command)"}).encode())
            return
        ack = send_command(self.server.drone_ip, self.server.cmd_port, cmd)
        self._send(200, "application/json", json.dumps({"sent": cmd, "ack": ack}).encode())

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                payload = json.dumps(HUB.snapshot())
                msg = f"data: {payload}\n\n".encode("utf-8")
                self.wfile.write(msg)
                self.wfile.flush()
                HUB.wait(0.5)   # wake on a fresh packet, else heartbeat every 0.5 s
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


def main():
    ap = argparse.ArgumentParser(description="riskybird ground-station panel")
    ap.add_argument("--drone", default="192.168.4.1", help="drone SoftAP IP (command target)")
    ap.add_argument("--telem-port", type=int, default=14550)
    ap.add_argument("--cmd-port", type=int, default=14551)
    ap.add_argument("--http", type=int, default=8080, help="dashboard HTTP port")
    ap.add_argument("--bind", default="127.0.0.1", help="dashboard bind address")
    ap.add_argument("--log-dir", default=os.path.join(HERE, "logs"),
                    help="directory for timestamped session CSV logs (default: ./logs)")
    ap.add_argument("--no-log", action="store_true", help="disable session logging")
    args = ap.parse_args()

    global LOGGER
    if not args.no_log:
        os.makedirs(args.log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        logpath = os.path.join(args.log_dir, f"riskybird_{stamp}.csv")
        LOGGER = Logger(logpath)
        print(f"[panel] logging:    {logpath}")

    threading.Thread(target=udp_rx_thread, args=(args.telem_port,), daemon=True).start()

    httpd = ThreadingHTTPServer((args.bind, args.http), Handler)
    httpd.drone_ip = args.drone
    httpd.cmd_port = args.cmd_port
    url = f"http://{args.bind}:{args.http}"
    print(f"[panel] dashboard:  {url}")
    print(f"[panel] commands ->  udp {args.drone}:{args.cmd_port}   (ESTOP / DISARM / HOVER_Z / PING)")
    print("[panel] join the drone AP 'riskybird-<id>' first; Ctrl-C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[panel] bye")


if __name__ == "__main__":
    main()
