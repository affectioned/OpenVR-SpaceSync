#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
#
# SpaceSync FrameLink service — runs on the Steam Frame, sends raw HMD poses
# with capture timestamps to the PC's SpaceSync driver (issue #18).
#
# Design notes:
#
# - Backends are plug-in: we try them in order and use the first that works.
#   Default order is openxr, then "stub". "stub" only emits Hello so you can
#   confirm the service is running and reachable from the PC, even before
#   we've figured out how to actually read Frame poses.
#
# - Clock: we timestamp samples with CLOCK_MONOTONIC_RAW in nanoseconds. The
#   driver treats these as opaque "Frame clock" ticks — alignment to PC QPC
#   happens via the ClockPing/Pong exchange (scaffolded, not wired yet).
#
# - Transport: plain UDP to <pc-host>:29485, same wire format as the
#   PC-side reference sender.

from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import socket
import struct
import sys
import time
from abc import ABC, abstractmethod

MAGIC = 0x4B4C4653
VERSION = 1
PORT_DEFAULT = 29485

MSG_HELLO       = 1
MSG_POSE        = 2
MSG_GRAVITY     = 3
MSG_RELOCALIZE  = 4

FLAG_POSITION   = 1 << 0
FLAG_ROTATION   = 1 << 1
FLAG_INERTIAL   = 1 << 2

log = logging.getLogger("framelink")


# -------- wire format ---------------------------------------------------------

def _hdr(msg_type: int, seq: int) -> bytes:
    return struct.pack("<IHHI", MAGIC, VERSION, msg_type, seq)


def pack_hello(seq: int, product: str, clock_hz: int, now_ticks: int) -> bytes:
    name = product.encode("utf-8")[:32].ljust(32, b"\0")
    return _hdr(MSG_HELLO, seq) + name + struct.pack("<QQ", clock_hz, now_ticks)


def pack_pose(seq: int, capture_ticks: int, device_id: int, flags: int, conf: float,
              pos, orient, lin, ang) -> bytes:
    return (
        _hdr(MSG_POSE, seq)
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
        _hdr(MSG_GRAVITY, seq)
        + struct.pack("<Q", capture_ticks)
        + struct.pack("<fff", *direction)
        + struct.pack("<f", conf)
    )


# -------- Frame clock ---------------------------------------------------------

# CLOCK_MONOTONIC_RAW in ns. Linux-only. Falls back to perf_counter_ns.
def now_ticks_ns() -> int:
    try:
        return time.clock_gettime_ns(getattr(time, "CLOCK_MONOTONIC_RAW"))
    except (AttributeError, OSError):
        return time.perf_counter_ns()


# -------- backend interface ---------------------------------------------------

class Backend(ABC):
    name: str = "base"

    @abstractmethod
    def init(self) -> bool:
        """Return True if the backend can produce data."""

    @abstractmethod
    def poll_hmd(self):
        """
        Return dict with keys pos (3-float), quat (4-float wxyz), lin (3-float),
        ang (3-float), flags (int), confidence (float 0..1), capture_ticks (int)
        OR None if no pose is currently available.
        """

    @abstractmethod
    def poll_gravity(self):
        """Return dict with keys direction (3-float), confidence (float) or None."""

    def shutdown(self) -> None:
        pass


# ---- OpenXR backend (most likely path on Valve's SteamOS) --------------------

class OpenXRBackend(Backend):
    name = "openxr"

    def init(self) -> bool:
        try:
            import xr  # pyopenxr
        except ImportError:
            log.info("pyopenxr not installed; skipping OpenXR backend")
            return False
        self.xr = xr

        try:
            # Headless-ish usage: we only need tracking, no rendering.
            app_info = xr.ApplicationInfo(
                application_name="spacesync-framelink",
                application_version=1,
                engine_name="spacesync",
                engine_version=1,
                api_version=xr.Version(1, 0, 0),
            )
            create_info = xr.InstanceCreateInfo(application_info=app_info)
            self.instance = xr.create_instance(create_info)
            self.system_id = xr.get_system(self.instance, xr.SystemGetInfo(
                form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY))

            # Session without a graphics binding is non-standard. On SteamOS/Monado
            # we may succeed; if not, this backend bails out and the stub takes over.
            try:
                session_create = xr.SessionCreateInfo(
                    system_id=self.system_id,
                    next=None,
                )
                self.session = xr.create_session(self.instance, session_create)
            except Exception as e:
                log.warning("openxr create_session rejected without graphics binding: %s", e)
                return False

            self.world_space = xr.create_reference_space(
                self.session,
                xr.ReferenceSpaceCreateInfo(
                    reference_space_type=xr.ReferenceSpaceType.STAGE,
                    pose_in_reference_space=xr.Posef(),
                ),
            )
            self.view_space = xr.create_reference_space(
                self.session,
                xr.ReferenceSpaceCreateInfo(
                    reference_space_type=xr.ReferenceSpaceType.VIEW,
                    pose_in_reference_space=xr.Posef(),
                ),
            )
            log.info("openxr backend initialized")
            return True
        except Exception as e:
            log.warning("openxr init failed: %s", e)
            return False

    def _xr_now(self) -> int:
        # Convert monotonic ns to an XrTime (int64 ns since some epoch).
        return now_ticks_ns()

    def poll_hmd(self):
        xr = self.xr
        try:
            loc = xr.locate_space(self.view_space, self.world_space, self._xr_now())
            flags = int(loc.location_flags)
            tracked = bool(flags & xr.SPACE_LOCATION_POSITION_TRACKED_BIT) and \
                      bool(flags & xr.SPACE_LOCATION_ORIENTATION_TRACKED_BIT)
            if not tracked:
                return None
            p = loc.pose.position
            o = loc.pose.orientation
            # OpenXR quaternion order is xyzw; our wire format is wxyz.
            return {
                "pos": (float(p.x), float(p.y), float(p.z)),
                "quat": (float(o.w), float(o.x), float(o.y), float(o.z)),
                "lin": (0.0, 0.0, 0.0),   # not available without a frame loop
                "ang": (0.0, 0.0, 0.0),
                "flags": FLAG_POSITION | FLAG_ROTATION,
                "confidence": 0.95,
                "capture_ticks": self._xr_now(),
            }
        except Exception as e:
            log.debug("locate_space failed: %s", e)
            return None

    def poll_gravity(self):
        return None

    def shutdown(self) -> None:
        try:
            self.xr.destroy_session(self.session)
            self.xr.destroy_instance(self.instance)
        except Exception:
            pass


# ---- Stub backend: handshake only, no poses ---------------------------------
#
# Keeps the service useful while we figure out the real pose source. The driver
# will log Hello and keep accepting the service as "present"; no poses means
# the overlay stays on OpenVR's HMD, as usual.

class StubBackend(Backend):
    name = "stub"

    def init(self) -> bool:
        log.info("stub backend active — handshake only, no poses")
        return True

    def poll_hmd(self):
        return None

    def poll_gravity(self):
        return None


BACKENDS = {
    "openxr": OpenXRBackend,
    "stub":   StubBackend,
}


# -------- main loop ----------------------------------------------------------

def choose_backend(preferred: list[str]) -> Backend:
    for name in preferred:
        cls = BACKENDS.get(name)
        if not cls:
            log.warning("unknown backend %r, skipping", name)
            continue
        inst = cls()
        if inst.init():
            log.info("using backend: %s", inst.name)
            return inst
        log.info("backend %s unavailable, trying next", name)
    log.error("no backend could initialize; exiting")
    sys.exit(2)


def _pose_moved(a, b, pos_eps: float, ang_eps_quat: float) -> bool:
    """Cheap motion check: position delta > pos_eps or quaternion inner product
    further from 1 than ang_eps_quat. The quat threshold corresponds to roughly
    0.5 degrees for ang_eps_quat=1e-5, which is well under tracking noise."""
    dp2 = sum((a[i] - b[i]) ** 2 for i in range(3))
    if dp2 > pos_eps * pos_eps:
        return True
    qa = a[3]; qb = b[3]
    # quaternion dot product, handle double cover
    dot = abs(qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3])
    return (1.0 - dot) > ang_eps_quat


def run(host: str, port: int, hz: float, backends: list[str], product: str,
        efficient: bool, idle_hz: float) -> None:
    backend = choose_backend(backends)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, port)
    log.info("sending to %s:%d @ %.1f Hz (idle %.1f Hz, efficient=%s), backend=%s",
             host, port, hz, idle_hz, efficient, backend.name)

    # Clock_hz is 10^9 because our ticks are nanoseconds.
    clock_hz = 1_000_000_000
    seq = 0
    last_hello = 0.0
    last_gravity = 0.0

    stopping = False

    def _stop(signum, _frame):
        nonlocal stopping
        log.info("signal %d — shutting down", signum)
        stopping = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    active_period = 1.0 / hz
    idle_period = 1.0 / max(idle_hz, 0.1)
    period = active_period
    next_t = time.monotonic()

    pose_sent = 0
    pose_missed = 0
    pose_skipped = 0

    # Motion-gate state: when the HMD stops moving, drop to idle_hz.
    last_sample_tuple = None   # (pos3, quat4) of the last *sent* pose
    last_motion_time = time.monotonic()
    idle_hold_secs = 1.0       # motion must be absent this long to drop to idle
    pos_eps = 0.0005           # 0.5 mm
    ang_eps_quat = 1e-5        # ~0.5 degrees

    try:
        while not stopping:
            now = time.monotonic()

            # Heartbeat Hello every second so a late-starting driver still handshakes.
            if now - last_hello >= 1.0:
                sock.sendto(pack_hello(seq, product, clock_hz, now_ticks_ns()), addr)
                seq += 1
                last_hello = now

            if now >= next_t:
                sample = backend.poll_hmd()
                if sample is not None:
                    pos = sample["pos"]; quat = sample["quat"]
                    moved = (last_sample_tuple is None or
                             _pose_moved((*pos, quat), (*last_sample_tuple[0], last_sample_tuple[1]),
                                         pos_eps, ang_eps_quat))

                    # In efficient mode, when the HMD hasn't moved we keep ticking
                    # at the idle rate but still emit a pose (so the driver's
                    # staleness gate stays happy and calibration has recent data).
                    # When it HAS moved, flip back to the active rate immediately.
                    if moved:
                        last_motion_time = now
                        period = active_period
                    elif efficient and (now - last_motion_time) > idle_hold_secs:
                        period = idle_period

                    sock.sendto(
                        pack_pose(
                            seq, sample["capture_ticks"], 0,
                            sample["flags"], sample["confidence"],
                            pos, quat, sample["lin"], sample["ang"],
                        ),
                        addr,
                    )
                    seq += 1
                    pose_sent += 1
                    last_sample_tuple = (pos, quat)
                else:
                    pose_missed += 1
                    # Nothing to send — stay in idle cadence to preserve power.
                    if efficient:
                        period = idle_period

                # Gravity about once a second is plenty — it barely changes.
                if now - last_gravity >= 1.0:
                    g = backend.poll_gravity()
                    if g is not None:
                        sock.sendto(
                            pack_gravity(seq, now_ticks_ns(),
                                         g["direction"], g["confidence"]),
                            addr,
                        )
                        seq += 1
                    last_gravity = now

                next_t += period
                if next_t < now - period * 4:
                    # If we lagged (system paused, laptop lid closed), resync
                    # instead of firing a burst to catch up.
                    next_t = now + period

            # Precise single sleep until the next tick — one wakeup per tick,
            # not four. On Linux this maps to a nanosleep syscall and the CPU
            # stays idle in between.
            delay = next_t - time.monotonic()
            if delay > 0:
                # Cap the sleep so Ctrl-C / SIGTERM latency is still reasonable.
                time.sleep(min(delay, 0.5))
    finally:
        sock.close()
        backend.shutdown()
        log.info("done — poses sent=%d missed=%d skipped=%d", pose_sent, pose_missed, pose_skipped)


def main() -> None:
    p = argparse.ArgumentParser(description="SpaceSync FrameLink service (Steam Frame side)")
    p.add_argument("--host", required=True, help="PC address running the SpaceSync driver")
    p.add_argument("--port", type=int, default=PORT_DEFAULT)
    p.add_argument("--hz", type=float, default=30.0,
                   help="active pose rate while the head is moving (default 30)")
    p.add_argument("--idle-hz", type=float, default=5.0,
                   help="pose rate once the head has been still ~1s (default 5)")
    p.add_argument("--no-efficient", action="store_true",
                   help="disable the motion-gated rate drop; stream at --hz regardless")
    p.add_argument("--backends", default="openxr,stub",
                   help="comma-separated backend preference list (default: openxr,stub)")
    p.add_argument("--product", default="steam-frame")
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )

    run(args.host, args.port, args.hz,
        [b.strip() for b in args.backends.split(",")],
        args.product, efficient=not args.no_efficient, idle_hz=args.idle_hz)


if __name__ == "__main__":
    main()
