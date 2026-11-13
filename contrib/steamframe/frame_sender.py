#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
#
# Reference sender for the Steam Frame -> SpaceSync UDP link (issue #18).
#
# Not production code. This script fakes a Steam Frame service by firing
# protocol-compliant UDP packets at the SpaceSync driver, so the receive path
# can be exercised on a regular PC without the real headset.
#
# Usage:
#   python frame_sender.py --host 127.0.0.1
#
# On the SpaceSync side, start SteamVR with the SpaceSync add-on enabled and
# tail %LOCALAPPDATA%\SpaceSync\driver.log — you should see "FrameLink: hello"
# followed by poses.

import argparse
import math
import socket
import struct
import time

MAGIC = 0x4B4C4653       # 'SFLK'
VERSION = 1
PORT_DEFAULT = 29485

MSG_HELLO       = 1
MSG_POSE        = 2
MSG_GRAVITY     = 3
MSG_RELOCALIZE  = 4


def hdr(msg_type: int, seq: int) -> bytes:
    return struct.pack("<IHHI", MAGIC, VERSION, msg_type, seq)


def pack_hello(seq: int, product: str, clock_hz: int, now_ticks: int) -> bytes:
    name = product.encode("utf-8")[:32].ljust(32, b"\0")
    return hdr(MSG_HELLO, seq) + name + struct.pack("<QQ", clock_hz, now_ticks)


def pack_pose(seq: int, capture_ticks: int, device_id: int, flags: int, conf: float,
              pos, orient, lin, ang) -> bytes:
    return (
        hdr(MSG_POSE, seq)
        + struct.pack("<Q", capture_ticks)
        + struct.pack("<II", device_id, flags)
        + struct.pack("<f", conf)
        + struct.pack("<fff", *pos)
        + struct.pack("<ffff", *orient)
        + struct.pack("<fff", *lin)
        + struct.pack("<fff", *ang)
    )


def pack_gravity(seq: int, capture_ticks: int, direction, conf: float) -> bytes:
    return (
        hdr(MSG_GRAVITY, seq)
        + struct.pack("<Q", capture_ticks)
        + struct.pack("<fff", *direction)
        + struct.pack("<f", conf)
    )


def pack_relocalize(seq: int, capture_ticks: int, translation, rotation) -> bytes:
    return (
        hdr(MSG_RELOCALIZE, seq)
        + struct.pack("<Q", capture_ticks)
        + struct.pack("<fff", *translation)
        + struct.pack("<ffff", *rotation)
    )


def main() -> None:
    p = argparse.ArgumentParser(description="Steam Frame link reference sender")
    p.add_argument("--host", default="127.0.0.1", help="SpaceSync PC address")
    p.add_argument("--port", type=int, default=PORT_DEFAULT)
    p.add_argument("--hz", type=float, default=72.0, help="pose rate")
    p.add_argument("--duration", type=float, default=10.0, help="seconds to run, 0 = forever")
    p.add_argument("--reloc-after", type=float, default=0.0,
                   help="seconds; if > 0, inject one relocalize event at that time")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)
    print(f"sending to {addr[0]}:{addr[1]} @ {args.hz:.1f} Hz")

    clock_hz = 10_000_000  # 100 ns ticks, same as Windows QPC-ish
    start = time.time()
    seq = 0

    sock.sendto(pack_hello(seq, "steam-frame-sim", clock_hz, 0), addr); seq += 1
    sock.sendto(pack_gravity(seq, 0, (0.0, -1.0, 0.0), 1.0), addr); seq += 1

    period = 1.0 / args.hz
    reloc_sent = False
    next_t = start
    while True:
        now = time.time()
        if args.duration > 0 and now - start >= args.duration:
            break
        if now >= next_t:
            t = now - start
            ticks = int(t * clock_hz)
            # Lazy circular motion so the log can tell samples are moving.
            pos = (0.3 * math.cos(t), 1.6, 0.3 * math.sin(t))
            yaw = t * 0.5
            orient = (math.cos(yaw * 0.5), 0.0, math.sin(yaw * 0.5), 0.0)
            lin = (-0.3 * math.sin(t), 0.0, 0.3 * math.cos(t))
            ang = (0.0, 0.5, 0.0)
            sock.sendto(pack_pose(seq, ticks, 0, 0b11, 0.95, pos, orient, lin, ang), addr); seq += 1

            if args.reloc_after > 0 and not reloc_sent and t >= args.reloc_after:
                sock.sendto(
                    pack_relocalize(seq, ticks, (0.02, 0.0, -0.01), (1.0, 0.0, 0.0, 0.0)),
                    addr,
                )
                seq += 1
                reloc_sent = True
                print(f"  injected relocalize at t={t:.2f}s")

            next_t += period
        else:
            time.sleep(min(period * 0.25, 0.005))

    sock.close()
    print("done")


if __name__ == "__main__":
    main()
