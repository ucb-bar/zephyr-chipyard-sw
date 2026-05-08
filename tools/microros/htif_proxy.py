#!/usr/bin/env python3
"""
HDLC-on-HTIF proxy for the micro_ros_multinode sample.

  spike (HTIF stdout/stdin)  <==> this proxy <==> /dev/pts/N <==> micro-ros-agent
                                       |
                                       +--> stdout (printk text outside frames)

What it does:

  1. Forks spike as a child, captures spike.stdout / drives spike.stdin.
  2. Opens a pty pair, exposes the slave path (e.g. /dev/pts/12) so the agent
     can be launched against it like a real serial port.
  3. Bidirectionally routes bytes:
       spike.stdout  ->  HDLC parser:
            in-frame, CRC-good   -> writes payload (re-framed in the same
                                    HDLC the agent expects) to pty master
            out-of-frame         -> proxies to this process's stdout, so
                                    Zephyr's printk output is still visible
                                    to the user
       pty master    ->  spike.stdin (HDLC-framed; the target's transport
                                      reader will unwrap)

Frame format (matches samples/micro_ros_multinode/src/transport_htif.c):

    0x7E  <escaped payload>  <escaped CRC16-CCITT, big-endian>  0x7E

  escape:  0x7E -> 0x7D 0x5E,  0x7D -> 0x7D 0x5D
"""

from __future__ import annotations

import argparse
import os
import pty
import select
import signal
import struct
import subprocess
import sys
import threading


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
    """Byte-by-byte parser that yields (kind, byte_or_payload).

    kind == 'console' -> single byte, out-of-frame, should be tee'd to stdout
    kind == 'frame'   -> bytes, CRC-validated payload to deliver to the agent
    """
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
                    # else: bad CRC - silently drop, likely stray printk byte
                self._buf = bytearray()
                self._state = 'IN'  # trailing FLAG = leading FLAG of next frame
            elif b == ESC:
                self._state = 'IN_ESC'
            else:
                self._buf.append(b)
        elif self._state == 'IN_ESC':
            self._buf.append(b ^ ESC_XOR)
            self._state = 'IN'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--elf', required=True, help='Path to the Zephyr ELF to run in spike')
    ap.add_argument('--spike', default='spike', help='Path to spike binary')
    ap.add_argument('--isa', default='rv64gc', help='ISA string for spike')
    ap.add_argument('--agent', default='micro-ros-agent',
                    help='Path to micro-ros-agent binary (or omit to print pty path only)')
    ap.add_argument('--no-agent', action='store_true',
                    help='Do not launch the agent; print pty path and wait')
    ap.add_argument('--agent-verbosity', default='4',
                    help='-v argument to micro-ros-agent (0..6)')
    args = ap.parse_args()

    # 1. pty pair for the agent
    agent_master, agent_slave = pty.openpty()
    agent_slave_path = os.ttyname(agent_slave)
    print(f'[proxy] agent pty: {agent_slave_path}', file=sys.stderr)

    # 2. spike subprocess — full bidirectional pipes
    spike = subprocess.Popen(
        [args.spike, f'--isa={args.isa}', args.elf],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=sys.stderr,
        bufsize=0,
    )

    # 3. agent subprocess (optional)
    agent_proc = None
    if not args.no_agent:
        agent_proc = subprocess.Popen(
            [args.agent, 'serial', '--dev', agent_slave_path,
             '-b', '115200', '-v', args.agent_verbosity],
            stdin=subprocess.DEVNULL,
            stdout=sys.stderr,
            stderr=sys.stderr,
        )
        print(f'[proxy] launched {args.agent} on {agent_slave_path}', file=sys.stderr)

    parser = HdlcParser()
    stop = threading.Event()

    def on_signal(_sig, _frame):
        stop.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

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
                    else:  # 'frame' — re-encode and forward to agent
                        os.write(agent_master, hdlc_encode(data))
        except Exception as e:
            print(f'[proxy] spike->agent thread: {e}', file=sys.stderr)

    def agent_to_spike():
        """Read agent pty output, HDLC-encode (it's already framed by the agent
        in serial mode, but we re-frame to match the on-target parser), feed to
        spike stdin."""
        try:
            agent_parser = HdlcParser()
            while not stop.is_set():
                try:
                    chunk = os.read(agent_master, 4096)
                except OSError:
                    return
                if not chunk:
                    return
                # The agent already emits HDLC-framed bytes when configured
                # in serial mode. We just pass them through to the target —
                # the target's HdlcParser will validate and unwrap.
                spike.stdin.write(chunk)
                spike.stdin.flush()
        except Exception as e:
            print(f'[proxy] agent->spike thread: {e}', file=sys.stderr)

    t1 = threading.Thread(target=spike_to_proxy, name='spike->agent', daemon=True)
    t2 = threading.Thread(target=agent_to_spike, name='agent->spike', daemon=True)
    t1.start(); t2.start()

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
        try: os.close(agent_master)
        except OSError: pass
        try: os.close(agent_slave)
        except OSError: pass


if __name__ == '__main__':
    main()
