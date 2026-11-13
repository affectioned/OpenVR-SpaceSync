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

def _mat34_to_pos_quat(m):
    """OpenVR HmdMatrix34_t → (position tuple, wxyz quat tuple)."""
    import math
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
        w = (m[2][1] - m[1][2]) / s; x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s; z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        w = (m[0][2] - m[2][0]) / s; y = 0.25 * s
        x = (m[0][1] + m[1][0]) / s; z = (m[1][2] + m[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        w = (m[1][0] - m[0][1]) / s; z = 0.25 * s
        x = (m[0][2] + m[2][0]) / s; y = (m[1][2] + m[2][1]) / s
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (px, py, pz), (w / n, x / n, y / n, z / n)


# ---- OpenVR Background backend ----------------------------------------------
#
# This is how Babble-style tools avoid the "Next up" compositor queue: attach
# as VRApplication_Background, which is explicitly a client that doesn't take
# a focus slot. We also skip OpenXR entirely, which is the cheapest path on
# a headless service.

class OpenVRBackend(Backend):
    name = "openvr"

    def init(self) -> bool:
        try:
            import openvr
        except ImportError:
            log.info("pyopenvr not installed; skipping OpenVR backend")
            return False
        self.openvr = openvr

        try:
            self.vr = openvr.init(openvr.VRApplication_Background)
        except Exception as e:
            log.warning("openvr init failed: %s", e)
            return False
        self._pose_buf = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
        log.info("openvr backend initialized (VRApplication_Background)")
        return True

    def poll_hmd(self):
        openvr = self.openvr
        try:
            openvr.VRSystem().getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseRawAndUncalibrated, 0.0, self._pose_buf
            )
            p = self._pose_buf[openvr.k_unTrackedDeviceIndex_Hmd]
        except Exception as e:
            log.debug("getDeviceToAbsoluteTrackingPose failed: %s", e)
            return None

        if not p.bPoseIsValid:
            # Connected but not tracking — surface as inertial-only so the
            # overlay drops the sample from calibration instead of trusting it.
            return {
                "pos": (0.0, 0.0, 0.0),
                "quat": (1.0, 0.0, 0.0, 0.0),
                "lin": (0.0, 0.0, 0.0),
                "ang": (0.0, 0.0, 0.0),
                "flags": FLAG_INERTIAL,
                "confidence": 0.2,
                "capture_ticks": now_ticks_ns(),
            }

        pos, quat = _mat34_to_pos_quat(p.mDeviceToAbsoluteTracking)
        v = p.vVelocity
        av = p.vAngularVelocity
        # Running_OK = 200, Running_OutOfRange = 201, Fallback_RotationOnly = 300
        conf = 0.95 if int(p.eTrackingResult) == 200 else 0.6
        return {
            "pos": pos,
            "quat": quat,
            "lin": (float(v[0]), float(v[1]), float(v[2])),
            "ang": (float(av[0]), float(av[1]), float(av[2])),
            "flags": FLAG_POSITION | FLAG_ROTATION,
            "confidence": conf,
            "capture_ticks": now_ticks_ns(),
        }

    def poll_gravity(self):
        # TODO once we have an IMU tap or a static-calibration estimate.
        return None

    def shutdown(self) -> None:
        try:
            self.openvr.shutdown()
        except Exception:
            pass


class OpenXRBackend(Backend):
    name = "openxr"

    # Runtime-specific headless extensions, in preference order. Monado and
    # Valve's SteamVR/OpenXR both ship XR_MND_headless — it's the Monado
    # extension that lets a session skip the graphics binding, which is
    # exactly what we need to pull head poses from a service.
    HEADLESS_EXTENSIONS = ("XR_MND_headless",)

    def init(self) -> bool:
        try:
            import xr  # pyopenxr
        except ImportError:
            log.info("pyopenxr not installed; skipping OpenXR backend")
            return False
        self.xr = xr

        try:
            avail = {e.extension_name.decode()
                     for e in xr.enumerate_instance_extension_properties()}
            headless = [e for e in self.HEADLESS_EXTENSIONS if e in avail]
            if not headless:
                log.warning("openxr runtime has no headless extension; "
                            "skipping (available: %d extensions)", len(avail))
                return False

            app_info = xr.ApplicationInfo(
                application_name="spacesync-framelink",
                application_version=1,
                engine_name="spacesync",
                engine_version=1,
                api_version=xr.Version(1, 0, 0),
            )
            self.instance = xr.create_instance(xr.InstanceCreateInfo(
                application_info=app_info,
                enabled_extension_names=headless,
            ))

            props = xr.get_instance_properties(self.instance)
            log.info("openxr runtime: %s %s, headless ext: %s",
                     props.runtime_name.decode(), props.runtime_version,
                     ",".join(headless))

            self.system_id = xr.get_system(self.instance, xr.SystemGetInfo(
                form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY))

            self.session = xr.create_session(
                self.instance, xr.SessionCreateInfo(system_id=self.system_id))

            self.stage_space = xr.create_reference_space(
                self.session, xr.ReferenceSpaceCreateInfo(
                    reference_space_type=xr.ReferenceSpaceType.STAGE,
                    pose_in_reference_space=xr.Posef(),
                ))
            self.view_space = xr.create_reference_space(
                self.session, xr.ReferenceSpaceCreateInfo(
                    reference_space_type=xr.ReferenceSpaceType.VIEW,
                    pose_in_reference_space=xr.Posef(),
                ))

            # Deliberately NOT calling begin_session.
            #
            # If we call begin_session with PRIMARY_STEREO on SteamVR's runtime,
            # the compositor queues us like a VR game — the headset shows a
            # "Next up: spacesync-framelink" stall until the service exits.
            # With XR_MND_headless, locate_space returns valid flags=0xf poses
            # directly on the created session, so we skip the activation step
            # entirely and stay out of the compositor's app list.
            # Verified on SteamVR/OpenXR 2.18.2 on Steam Frame.

            # Position+Orientation valid + tracked mask (bits 0..3).
            self.required_flags = (
                xr.SPACE_LOCATION_POSITION_VALID_BIT
                | xr.SPACE_LOCATION_ORIENTATION_VALID_BIT
                | xr.SPACE_LOCATION_POSITION_TRACKED_BIT
                | xr.SPACE_LOCATION_ORIENTATION_TRACKED_BIT
            )
            log.info("openxr backend initialized (headless)")
            return True
        except Exception as e:
            log.warning("openxr init failed: %s", e)
            return False

    def _xr_time(self):
        return self.xr.Time(now_ticks_ns())

    def poll_hmd(self):
        xr = self.xr
        try:
            loc = xr.locate_space(self.view_space, self.stage_space, self._xr_time())
            flags = int(loc.location_flags)
            if (flags & self.required_flags) != self.required_flags:
                # Position or orientation isn't tracked — treat as
                # inertial-only so the PC-side calibrator discards it.
                return {
                    "pos": (0.0, 0.0, 0.0),
                    "quat": (1.0, 0.0, 0.0, 0.0),
                    "lin": (0.0, 0.0, 0.0),
                    "ang": (0.0, 0.0, 0.0),
                    "flags": FLAG_INERTIAL,
                    "confidence": 0.2,
                    "capture_ticks": now_ticks_ns(),
                }
            p = loc.pose.position
            o = loc.pose.orientation
            # OpenXR quaternion is xyzw in Posef.orientation; our wire format
            # is wxyz. Linear/angular velocity aren't delivered by
            # locate_space — reserved for a future frame-loop upgrade.
            return {
                "pos": (float(p.x), float(p.y), float(p.z)),
                "quat": (float(o.w), float(o.x), float(o.y), float(o.z)),
                "lin": (0.0, 0.0, 0.0),
                "ang": (0.0, 0.0, 0.0),
                "flags": FLAG_POSITION | FLAG_ROTATION,
                "confidence": 0.95,
                "capture_ticks": now_ticks_ns(),
            }
        except Exception as e:
            log.debug("locate_space failed: %s", e)
            return None

    def poll_gravity(self):
        # TODO: pull gravity from the runtime if an extension exposes it.
        # For now the PC side is happy without it — the solver falls back to
        # the existing lighthouse gravity path.
        return None

    def shutdown(self) -> None:
        try:
            # The right teardown is request_exit_session -> event loop ->
            # end_session, but a long-lived service is almost always killed
            # by SIGTERM; skip the dance and let the runtime clean up when
            # the process exits.
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
    "openvr": OpenVRBackend,
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
    last_stats = time.monotonic()

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

            # Periodic stats so you can see what's actually flowing without
            # tcpdump. Rate-limited to once every 5s to stay quiet.
            if now - last_stats >= 5.0:
                log.info("stats: poses sent=%d missed=%d skipped=%d current_rate=%.1fHz",
                         pose_sent, pose_missed, pose_skipped, 1.0 / period if period else 0.0)
                last_stats = now

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
    p.add_argument("--backends", default="openvr,openxr,stub",
                   help="comma-separated backend preference list (default: openvr,openxr,stub). "
                        "openvr uses VRApplication_Background and avoids SteamVR's compositor "
                        "queue; openxr needs XR_MND_headless and currently stalls the headset "
                        "on Valve's SteamVR/OpenXR runtime.")
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
