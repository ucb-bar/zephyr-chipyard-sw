#!/usr/bin/env python3
"""
Decode HDLC-framed topic-data records emitted by the in-target broker
(samples/micro_ros_local/) on its HTIF console.

Console output and topic-data frames share spike's stdout; the parser
demuxes by HDLC delimiter (0x7E) + CRC validation. Anything that fails
CRC is treated as console text and forwarded to our stdout (so printk
debug lines stay visible). Valid frames are decoded as:

    [name_len:1] [name_bytes] [payload_bytes] [CRC16:2]

and printed in a `--%s payload_hex` form to stdout (or as JSON with
`--json`).

Usage:
    spike build/zephyr/zephyr.elf | tools/microros/topic_log_decoder.py
    tools/microros/topic_log_decoder.py < captured_spike_stdout.bin
"""

from __future__ import annotations

import argparse
import json
import sys


FLAG = 0x7E
ESC = 0x7D
ESC_XOR = 0x20


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


class HdlcParser:
    """Yields ('console', byte) for out-of-frame data, ('frame', bytes)
    for CRC-validated frames. Frames whose CRC fails are re-emitted as
    'console' bytes (best effort — matches printk's `~` text)."""

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
                emitted_frame = False
                if len(self._buf) >= 3:
                    payload, crc_bytes = bytes(self._buf[:-2]), bytes(self._buf[-2:])
                    crc_got = (crc_bytes[0] << 8) | crc_bytes[1]
                    if crc_got == crc16_ccitt(payload):
                        yield ('frame', payload)
                        emitted_frame = True
                    else:
                        # Bad CRC — assume what we accumulated was actually
                        # console text that happened to be sandwiched between
                        # two stray '~' bytes. Re-emit as console.
                        yield ('console', bytes([FLAG]))
                        for x in self._buf:
                            yield ('console', bytes([x]))
                        yield ('console', bytes([FLAG]))
                self._buf = bytearray()
                # If we emitted a real frame, drop back to OUT_OF_FRAME so any
                # printk bytes that arrive before the next frame's start FLAG
                # go straight to console instead of being mistaken for the
                # body of a new frame.
                self._state = 'OUT' if emitted_frame else 'IN'
            elif b == ESC:
                self._state = 'IN_ESC'
            else:
                self._buf.append(b)
        elif self._state == 'IN_ESC':
            self._buf.append(b ^ ESC_XOR)
            self._state = 'IN'


def decode_frame(frame: bytes):
    if len(frame) < 1:
        return None
    name_len = frame[0]
    if 1 + name_len > len(frame):
        return None
    name = frame[1:1 + name_len].decode('utf-8', errors='replace')
    payload = frame[1 + name_len:]
    return name, payload


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', '-i', default=None,
                    help='File to read from (default: stdin)')
    ap.add_argument('--json', action='store_true',
                    help='Emit one JSON object per topic record on stdout')
    ap.add_argument('--silent-console', action='store_true',
                    help='Drop console (non-frame) bytes instead of forwarding')
    args = ap.parse_args()

    if args.input:
        src = open(args.input, 'rb')
    else:
        src = sys.stdin.buffer

    parser = HdlcParser()
    while True:
        chunk = src.read(1)
        if not chunk:
            break
        for kind, data in parser.feed(chunk[0]):
            if kind == 'console':
                if not args.silent_console:
                    sys.stderr.buffer.write(data)
                    sys.stderr.flush()
            else:
                rec = decode_frame(data)
                if rec is None:
                    continue
                name, payload = rec
                if args.json:
                    print(json.dumps({'topic': name, 'len': len(payload),
                                      'hex': payload.hex()}))
                else:
                    print(f'-- {name} -- {len(payload)} B  {payload.hex()}')
                sys.stdout.flush()


if __name__ == '__main__':
    main()
