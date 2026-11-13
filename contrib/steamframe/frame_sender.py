#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
#
# Reference sender for the Steam Frame -> SpaceSync UDP link (issue #18).
#
# Not production code. Two modes:
#
#   default         — synthetic circle motion, useful to prove the pipe works
#                     end-to-end without any VR runtime. Calibration won't
#                     succeed with this (coplanar / mismatched motion), but
#                     the "Steam Frame data used for N / M samples" log line
#                     is enough proof that the IPC is alive.
#
#   --replay-from-driver
#                   — attach to a running SteamVR session as a background
#                     app, read the real HMD pose each tick (raw uncalibrated
#                     universe, same frame of reference the SpaceSync
#                     calibrator polls), and resend it as if it came off a
#                     Steam Frame service. With this flag you can actually
#                     complete a calibration pass, which exercises the full
#                     substitution end-to-end.
#                     Requires `pip install openvr`.

import argparse
import math
import socket
import struct
import sys
import time

MAGIC = 0x4B4C4653       # 'SFLK'
VERSION = 1
PORT_DEFAULT = 29485

MSG_HELLO       = 1
MSG_POSE        = 2
MSG_GRAVITY     = 3
MSG_RELOCALIZE  = 4

# framelink::PoseFlag
FLAG_POSITION   = 1 << 0
FLAG_ROTATION   = 1 << 1
FLAG_INERTIAL   = 1 << 2


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


# -----------------------------------------------------------------------------
# Replay-from-driver: pull real HMD poses out of SteamVR and re-emit them.

def mat34_to_pos_quat(m):
    # m is a 3x4 row-major matrix. Position is the last column; quaternion
    # from the top-left 3x3 using Shoemake's branchless method.
    px, py, pz = m[0][3], m[1][3], m[2][3]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = 2.0 * math.sqrt(1.0 + tr)
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    # Normalize just in case of accumulated error from s ≈ 0.
    n = math.sqrt(w*w + x*x + y*y + z*z) or 1.0
    return (px, py, pz), (w / n, x / n, y / n, z / n)


class DriverReplay:
    def __init__(self):
        try:
            import openvr
        except ImportError:
            print("replay-from-driver needs pyopenvr: pip install openvr", file=sys.stderr)
            raise
        self.openvr = openvr
        # Background app = we attach to an already-running SteamVR session
        # instead of starting our own.
        self.sys = openvr.init(openvr.VRApplication_Background)
        print("replay: attached to SteamVR as Background app")

    def read_hmd_with_result(self):
        """Returns (sample_or_None, eTrackingResult_or_None)."""
        openvr = self.openvr
        poses = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
        # Match the calibrator's polling: raw uncalibrated universe, zero prediction.
        openvr.VRSystem().getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseRawAndUncalibrated, 0.0, poses
        )
        p = poses[openvr.k_unTrackedDeviceIndex_Hmd]
        if not p.bPoseIsValid:
            return None, int(p.eTrackingResult)
        m = [[p.mDeviceToAbsoluteTracking[r][c] for c in range(4)] for r in range(3)]
        pos, quat = mat34_to_pos_quat(m)
        v = p.vVelocity
        av = p.vAngularVelocity
        return {
            "pos": pos,
            "quat": quat,
            "lin": (v[0], v[1], v[2]),
            "ang": (av[0], av[1], av[2]),
        }, int(p.eTrackingResult)

    def shutdown(self):
        self.openvr.shutdown()


# -----------------------------------------------------------------------------

def main() -> None:
    p = argparse.ArgumentParser(description="Steam Frame link reference sender")
    p.add_argument("--host", default="127.0.0.1", help="SpaceSync PC address")
    p.add_argument("--port", type=int, default=PORT_DEFAULT)
    p.add_argument("--hz", type=float, default=72.0, help="pose rate")
    p.add_argument("--duration", type=float, default=10.0, help="seconds to run, 0 = forever")
    p.add_argument("--reloc-after", type=float, default=0.0,
                   help="seconds; if > 0, inject one relocalize event at that time")
    p.add_argument("--replay-from-driver", action="store_true",
                   help="read real HMD poses from a running SteamVR session and resend them")
    p.add_argument("--confidence", type=float, default=0.95,
                   help="confidence value to stamp on outgoing poses (0..1)")
    p.add_argument("--hello-every", type=float, default=1.0,
                   help="resend Hello at this interval (seconds) so a late-starting driver still sees the handshake")
    args = p.parse_args()

    # Only import pyopenvr when actually asked, so the synthetic mode still
    # runs on a plain Python install.
    replay = None
    product = "steam-frame-sim"
    if args.replay_from_driver:
        replay = DriverReplay()
        product = "steam-frame-replay"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)
    print(f"sending to {addr[0]}:{addr[1]} @ {args.hz:.1f} Hz  mode={'replay' if replay else 'synthetic'}")

    clock_hz = 10_000_000  # 100 ns ticks, same as Windows QPC-ish
    start = time.time()
    seq = 0

    sock.sendto(pack_hello(seq, product, clock_hz, 0), addr); seq += 1
    sock.sendto(pack_gravity(seq, 0, (0.0, -1.0, 0.0), 1.0), addr); seq += 1

    period = 1.0 / args.hz
    reloc_sent = False
    next_t = start
    last_hello = start
    last_status = start
    stats_invalid = 0
    stats_sent = 0
    last_result = -1
    try:
        while True:
            now = time.time()
            if args.duration > 0 and now - start >= args.duration:
                break

            if args.hello_every > 0 and now - last_hello >= args.hello_every:
                sock.sendto(pack_hello(seq, product, clock_hz, 0), addr); seq += 1
                last_hello = now

            # Replay mode: every 2s, print a one-line status so you can tell
            # whether the HMD is actually being tracked. Running_OK = 200.
            if replay is not None and now - last_status >= 2.0:
                last_status = now
                result_name = {
                    1: "Uninitialized", 100: "Calibrating_InProgress",
                    101: "Calibrating_OutOfRange", 200: "Running_OK",
                    201: "Running_OutOfRange", 300: "Fallback_RotationOnly",
                }.get(last_result, f"result={last_result}")
                print(f"  status sent={stats_sent} invalid={stats_invalid}  HMD: {result_name}")

            if now >= next_t:
                t = now - start
                ticks = int(t * clock_hz)

                if replay is not None:
                    sample, result = replay.read_hmd_with_result()
                    last_result = result if result is not None else last_result
                    if sample is None:
                        stats_invalid += 1
                    else:
                        flags = FLAG_POSITION | FLAG_ROTATION
                        sock.sendto(
                            pack_pose(
                                seq, ticks, 0, flags, args.confidence,
                                sample["pos"], sample["quat"],
                                sample["lin"], sample["ang"],
                            ),
                            addr,
                        )
                        seq += 1
                        stats_sent += 1
                else:
                    pos = (0.3 * math.cos(t), 1.6, 0.3 * math.sin(t))
                    yaw = t * 0.5
                    orient = (math.cos(yaw * 0.5), 0.0, math.sin(yaw * 0.5), 0.0)
                    lin = (-0.3 * math.sin(t), 0.0, 0.3 * math.cos(t))
                    ang = (0.0, 0.5, 0.0)
                    sock.sendto(
                        pack_pose(seq, ticks, 0, FLAG_POSITION | FLAG_ROTATION, args.confidence,
                                  pos, orient, lin, ang),
                        addr,
                    )
                    seq += 1
                    stats_sent += 1

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
    finally:
        sock.close()
        if replay is not None:
            replay.shutdown()
        print(f"done  sent={stats_sent} invalid={stats_invalid}")


if __name__ == "__main__":
    main()
