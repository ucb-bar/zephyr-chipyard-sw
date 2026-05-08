#!/usr/bin/env python3
"""
HDLC-on-HTIF proxy for the micro_ros_multinode sample.

  spike (HTIF stdout/stdin)  <==> this proxy <==> /dev/pts/N <==> micro-ros-agent
                                       |
                                       +--> stdout (printk text outside frames)

What it does:

  1. Forks spike as a child, captures spike.stdout / drives spike.stdin.
  2. Forks the agent in `pseudoterminal` mode; the agent allocates its own pty
     and prints the slave path on its stderr, which we parse out and open.
  3. Bidirectionally routes bytes:
       spike.stdout  ->  HDLC parser:
            in-frame, CRC-good   -> writes payload to the agent's pty (raw)
            out-of-frame         -> proxies to this process's stdout, so
                                    Zephyr's printk output is still visible
                                    to the user
       agent pty     ->  spike.stdin (raw — the target's HDLC parser handles
                                      framing of bytes the agent emits)

Frame format (matches samples/micro_ros_multinode/src/transport_htif.c):

    0x7E  <escaped payload>  <escaped CRC16-CCITT, big-endian>  0x7E

  escape:  0x7E -> 0x7D 0x5E,  0x7D -> 0x7D 0x5D
"""

from __future__ import annotations

import argparse
import os
import pty
import re
import select
import signal
import subprocess
import sys
import threading
import time


FLAG = 0x7E
ESC = 0x7D
ESC_XOR = 0x20


def crc16_ccitt(data: bytes) -> int:
    """CRC-CCITT, poly 0x1021, init 0xFFFF, no reflect, no xorout."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def hdlc_encode(payload: bytes) -> bytes:
    """Wrap payload in one 0x7E-delimited frame with escaped bytes + CRC trailer."""
    crc = crc16_ccitt(payload)
    raw = payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    out = bytearray([FLAG])
    for b in raw:
        if b == FLAG or b == ESC:
            out.append(ESC)
            out.append(b ^ ESC_XOR)
        else:
            out.append(b)
    out.append(FLAG)
    return bytes(out)


class HdlcParser:
    """Byte-by-byte parser; yields ('console', byte) for out-of-frame data
    and ('frame', bytes) for CRC-validated frame payloads."""
    def __init__(self):
        self._state = 'OUT'
        self._buf = bytearray()

    def feed(self, b: int):
        if self._state == 'OUT':
            if b == FLAG:
                self._buf = bytearray()
                self._state = 'IN'
            else:
                yield ('console', bytes([b]))
        elif self._state == 'IN':
            if b == FLAG:
                if len(self._buf) >= 3:
                    payload, crc_bytes = bytes(self._buf[:-2]), bytes(self._buf[-2:])
                    crc_got = (crc_bytes[0] << 8) | crc_bytes[1]
                    if crc_got == crc16_ccitt(payload):
                        yield ('frame', payload)
                self._buf = bytearray()
                self._state = 'IN'
            elif b == ESC:
                self._state = 'IN_ESC'
            else:
                self._buf.append(b)
        elif self._state == 'IN_ESC':
            self._buf.append(b ^ ESC_XOR)
            self._state = 'IN'


_ANSI_RE = re.compile(rb'\x1b\[[0-9;]*[A-Za-z]')
# The agent emits a line like "...Pseudoterminal opened at | dev: /dev/pts/N",
# but with ANSI color codes interleaved. Match loosely after stripping ANSI.
_PTY_RE = re.compile(rb'dev:\s*(/dev/pts/\d+)')


def wait_for_agent_pty(agent_proc: subprocess.Popen, timeout: float = 10.0) -> str:
    """Block until the agent prints its allocated pty path. Returns the path."""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        # The agent prints to stderr; we read both fds.
        for fd in (agent_proc.stderr, agent_proc.stdout):
            if fd is None:
                continue
            r, _, _ = select.select([fd], [], [], 0.1)
            if fd in r:
                chunk = os.read(fd.fileno(), 4096)
                if chunk:
                    buf += chunk
                    sys.stderr.buffer.write(chunk)
                    sys.stderr.flush()
                    clean = _ANSI_RE.sub(b'', bytes(buf))
                    m = _PTY_RE.search(clean)
                    if m:
                        return m.group(1).decode()
        if agent_proc.poll() is not None:
            raise RuntimeError(f'agent exited with rc={agent_proc.returncode}')
    raise TimeoutError("Couldn't find pty path in agent output within timeout")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--elf', required=True, help='Path to the Zephyr ELF to run in spike')
    ap.add_argument('--spike', default='spike', help='Path to spike binary')
    ap.add_argument('--isa', default='rv64gc', help='ISA string for spike')
    ap.add_argument('--agent', default='micro-ros-agent',
                    help='Path to micro-ros-agent binary (snap mode, broken on this host)')
    ap.add_argument('--agent-mode', choices=['snap', 'docker'], default='docker',
                    help='How to launch the agent: '
                         '"snap" (broken — pty in snap confinement is inaccessible), '
                         '"docker" (microros/micro-ros-agent:jazzy with host pty bind)')
    ap.add_argument('--agent-image', default='microros/micro-ros-agent:jazzy',
                    help='Docker image for --agent-mode docker')
    ap.add_argument('--no-agent', action='store_true',
                    help='Do not launch the agent; use --agent-pty instead')
    ap.add_argument('--agent-pty', default=None,
                    help='Path to an already-allocated agent pty (skips agent launch)')
    ap.add_argument('--agent-verbosity', default='6',
                    help='-v argument to micro-ros-agent (0..6)')
    args = ap.parse_args()

    # 1. Get an agent — and a pty between the agent and us.
    agent_proc = None
    agent_fd = None
    if args.agent_pty:
        agent_pty_path = args.agent_pty
        print(f'[proxy] using existing agent pty: {agent_pty_path}', file=sys.stderr)
        agent_fd = os.open(agent_pty_path, os.O_RDWR | os.O_NOCTTY)
    elif args.no_agent:
        print('ERROR: --no-agent requires --agent-pty <path>', file=sys.stderr)
        return 1
    elif args.agent_mode == 'docker':
        # Create a pty pair on the host. The agent inside the Docker
        # container opens the slave path in `serial` mode; we hold the
        # master fd here. We KEEP the slave fd open in this process too —
        # closing it can hang up the pty as the container starts.
        agent_fd, slave_fd = pty.openpty()
        slave_path = os.ttyname(slave_fd)
        print(f'[proxy] host pty pair: master_fd={agent_fd}, slave={slave_path}',
              file=sys.stderr)
        # `-v /dev/pts:/dev/pts` shares the host's pts namespace with the
        # container, so /dev/pts/<N> on host and inside container reference
        # the same kernel-allocated pty (cleaner than --device which needs
        # capabilities to create a device node).
        # Pipe agent stdout/stderr; the agent_to_stderr thread below drains
        # them and tees to our stderr (file-redirection via subprocess
        # caused symptoms where agent appeared to receive no traffic — likely
        # libc block-buffering on the file fd masking real-time activity).
        agent_proc = subprocess.Popen(
            ['docker', 'run', '--rm',
             '-v', '/dev/pts:/dev/pts',
             '-i',
             args.agent_image,
             'serial', '--dev', slave_path,
             '-b', '115200', '-v', args.agent_verbosity],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        print(f'[proxy] launched docker agent pid={agent_proc.pid}', file=sys.stderr)
        # Start draining agent's stdout/stderr immediately so its log writes
        # don't back up on the subprocess pipes during container startup.
        def _drain_to_stderr(fd):
            try:
                while True:
                    chunk = os.read(fd, 4096)
                    if not chunk:
                        return
                    sys.stderr.buffer.write(chunk)
                    sys.stderr.flush()
            except (OSError, ValueError):
                return
        threading.Thread(target=_drain_to_stderr,
                         args=(agent_proc.stdout.fileno(),),
                         daemon=True).start()
        threading.Thread(target=_drain_to_stderr,
                         args=(agent_proc.stderr.fileno(),),
                         daemon=True).start()
        # Give the container time to start and the agent to open /dev/pts/N.
        time.sleep(3.0)
    else:
        # snap mode (kept for reference, broken under snap confinement)
        agent_proc = subprocess.Popen(
            [args.agent, 'pseudoterminal',
             '-b', '115200',
             '-v', args.agent_verbosity],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        print(f'[proxy] launched {args.agent} pid={agent_proc.pid}', file=sys.stderr)
        agent_pty_path = wait_for_agent_pty(agent_proc)
        print(f'[proxy] agent pty: {agent_pty_path}', file=sys.stderr)
        agent_fd = os.open(agent_pty_path, os.O_RDWR | os.O_NOCTTY)

    # 3. Spike subprocess — full bidirectional pipes.
    spike = subprocess.Popen(
        [args.spike, f'--isa={args.isa}', args.elf],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=sys.stderr,
        bufsize=0,
    )
    print(f'[proxy] launched spike pid={spike.pid}', file=sys.stderr)

    parser = HdlcParser()
    stop = threading.Event()

    def on_signal(_sig, _frame):
        stop.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    frames_to_agent = [0]
    bytes_from_agent = [0]
    debug = bool(int(os.environ.get('HTIF_PROXY_DEBUG', '0')))

    def spike_to_proxy():
        """Read spike stdout, demux frames to agent pty, console bytes to stdout."""
        try:
            while not stop.is_set():
                chunk = spike.stdout.read(1)
                if not chunk:
                    return
                for kind, data in parser.feed(chunk[0]):
                    if kind == 'console':
                        sys.stdout.buffer.write(data)
                        sys.stdout.flush()
                    else:
                        os.write(agent_fd, data)
                        frames_to_agent[0] += 1
                        if debug and (frames_to_agent[0] <= 3 or
                                      frames_to_agent[0] % 10 == 0):
                            hex_preview = data[:16].hex(' ')
                            print(f'[proxy] spike->agent frame#{frames_to_agent[0]} '
                                  f'len={len(data)} hex_head={hex_preview}',
                                  file=sys.stderr)
        except Exception as e:
            print(f'[proxy] spike->agent thread: {e}', file=sys.stderr)

    def agent_to_spike():
        """Read agent pty output, HDLC-encode each chunk, feed to spike stdin."""
        try:
            while not stop.is_set():
                try:
                    chunk = os.read(agent_fd, 4096)
                except OSError:
                    return
                if not chunk:
                    return
                bytes_from_agent[0] += len(chunk)
                spike.stdin.write(hdlc_encode(chunk))
                spike.stdin.flush()
        except Exception as e:
            print(f'[proxy] agent->spike thread: {e}', file=sys.stderr)

    def agent_to_stderr():
        """Tee remaining agent stderr/stdout to our stderr so we can see its log."""
        if agent_proc is None:
            return
        try:
            while not stop.is_set():
                for fd in (agent_proc.stderr, agent_proc.stdout):
                    if fd is None:
                        continue
                    r, _, _ = select.select([fd], [], [], 0.1)
                    if fd in r:
                        chunk = os.read(fd.fileno(), 4096)
                        if not chunk:
                            return
                        sys.stderr.buffer.write(chunk)
                        sys.stderr.flush()
                if agent_proc.poll() is not None:
                    return
        except Exception:
            pass

    spike_stdin_lock = threading.Lock()

    def heartbeat_to_spike():
        """Continuously feed a 0x00 byte into spike's stdin so the target's
        uart_poll_in -> spike HTIF GETC -> read(stdin) chain doesn't block
        when the agent has nothing to send. The target's HDLC parser sees
        0x00 in OUT_OF_FRAME state and discards it. When the agent does have
        something to send, agent_to_spike grabs the lock and sends a real
        HDLC-framed payload; the heartbeat resumes after that frame finishes."""
        try:
            while not stop.is_set():
                with spike_stdin_lock:
                    try:
                        spike.stdin.write(b'\x00')
                        spike.stdin.flush()
                    except (BrokenPipeError, OSError):
                        return
                time.sleep(0.001)   # ~1000 Hz; cheap relative to spike speed
        except Exception as e:
            print(f'[proxy] heartbeat: {e}', file=sys.stderr)

    agent_frames_sent = [0]
    # Wrap agent_to_spike's write under the same lock so heartbeats and
    # frame writes don't interleave bytes.
    def agent_to_spike():  # noqa: F811
        try:
            while not stop.is_set():
                try:
                    chunk = os.read(agent_fd, 4096)
                except OSError:
                    return
                if not chunk:
                    return
                bytes_from_agent[0] += len(chunk)
                agent_frames_sent[0] += 1
                with spike_stdin_lock:
                    spike.stdin.write(hdlc_encode(chunk))
                    spike.stdin.flush()
                if debug and (agent_frames_sent[0] <= 3 or
                              agent_frames_sent[0] % 10 == 0):
                    hex_preview = chunk[:16].hex(' ')
                    print(f'[proxy] agent->spike frame#{agent_frames_sent[0]} '
                          f'len={len(chunk)} hex_head={hex_preview}',
                          file=sys.stderr)
        except Exception as e:
            print(f'[proxy] agent->spike thread: {e}', file=sys.stderr)

    threads = [
        threading.Thread(target=spike_to_proxy, name='spike->agent', daemon=True),
        threading.Thread(target=agent_to_spike, name='agent->spike', daemon=True),
        threading.Thread(target=heartbeat_to_spike, name='heartbeat', daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        rc = spike.wait()
        print(f'[proxy] spike exited {rc}', file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for p in (spike, agent_proc):
            if p and p.poll() is None:
                p.terminate()
        for p in (spike, agent_proc):
            if p:
                try: p.wait(timeout=2)
                except subprocess.TimeoutExpired: p.kill()
        try: os.close(agent_fd)
        except OSError: pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
