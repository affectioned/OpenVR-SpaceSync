"""Relocalization detection in the Steam Frame service (issue #18).

Feeds synthetic raw HMD streams through RelocDetector and checks that a
held origin shift is reported once with the right shift, while normal head
motion, a one-sample glitch and tracking loss are not. Also checks the
Relocalize packet size against FrameLinkProtocol.h.
"""
import importlib.util
import math
import re
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SERVICE = ROOT / "contrib" / "steamframe" / "service" / "spacesync-framelink.py"
HEADER = ROOT / "include" / "shared" / "FrameLinkProtocol.h"

spec = importlib.util.spec_from_file_location("framelink_service", SERVICE)
svc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(svc)

HZ = 30.0
DT = 1.0 / HZ


def q_yaw(deg):
    h = math.radians(deg) / 2.0
    return (math.cos(h), 0.0, math.sin(h), 0.0)


def sample(pos, quat, lin=(0.0, 0.0, 0.0), ang=(0.0, 0.0, 0.0), flags=None, conf=0.95):
    return {
        "pos": pos, "quat": quat, "lin": lin, "ang": ang,
        "flags": svc.FLAG_POSITION | svc.FLAG_ROTATION if flags is None else flags,
        "confidence": conf, "capture_ticks": 0,
    }


def run(stream):
    det = svc.RelocDetector()
    hits = []
    for i, s in enumerate(stream):
        r = det.feed(i * DT, s)
        if r is not None:
            hits.append((i, r))
    return hits


def walking(n, start=0.0, speed=0.8, yaw_rate=40.0):
    """Head walking along x at `speed` m/s while turning at `yaw_rate` deg/s."""
    out = []
    for i in range(n):
        t = (start + i) * DT
        out.append(sample((speed * t, 1.7, 0.0), q_yaw(yaw_rate * t),
                          lin=(speed, 0.0, 0.0), ang=(0.0, math.radians(yaw_rate), 0.0)))
    return out


def shifted(s, yaw_deg, t):
    q = q_yaw(yaw_deg)
    p = svc._q_rotate(q, s["pos"])
    return dict(s, pos=(p[0] + t[0], p[1] + t[1], p[2] + t[2]), quat=svc._q_mul(q, s["quat"]))


def test_motion_is_quiet():
    assert run(walking(150)) == [], "normal walking/turning must not trigger"


def test_origin_shift_reported_once():
    base = walking(120)
    stream = base[:60] + [shifted(s, 25.0, (0.4, 0.0, -0.5)) for s in base[60:]]
    hits = run(stream)
    assert len(hits) == 1, hits
    i, (_, trans, rot) = hits[0]
    assert i == 61, f"confirmed on the sample after the step, got {i}"
    # Recovered shift maps an old-frame point onto the new frame like the
    # injected one does (velocity prediction makes it approximate).
    yaw = math.degrees(2.0 * math.atan2(rot[2], rot[0]))
    # Rotation isn't predicted across the step, so one sample of head turn
    # (40 deg/s / 30 Hz = 1.3 deg) ends up in the recovered yaw.
    assert abs(yaw - 25.0) < 2.0, yaw
    p = base[60]["pos"]
    want = shifted(base[60], 25.0, (0.4, 0.0, -0.5))["pos"]
    rp = svc._q_rotate(rot, p)
    got = (rp[0] + trans[0], rp[1] + trans[1], rp[2] + trans[2])
    err = math.dist(got, want)
    assert err < 0.03, err


def test_glitch_dropped():
    stream = walking(60)
    stream[30] = shifted(stream[30], 0.0, (0.5, 0.0, 0.0))
    assert run(stream) == [], "one-sample glitch that snaps back must not be reported"


def test_tracking_loss_resets():
    base = walking(90)
    lost = sample((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), flags=svc.FLAG_INERTIAL, conf=0.2)
    stream = base[:40] + [lost] * 30 + [shifted(s, 30.0, (1.0, 0.0, 0.0)) for s in base[70:]]
    assert run(stream) == [], "a step across a tracking gap is not measurable"


def test_packet_size_matches_header():
    text = HEADER.read_text()
    m = re.search(r"struct FRAMELINK_PACKED Relocalize\s*\{(.*?)\};", text, re.S)
    assert m, "Relocalize struct not found"
    body = m.group(1)
    assert "uint64_t captureTicks" in body and "float    translation[3]" in body and "float    rotation[4]" in body
    m = re.search(r"struct FRAMELINK_PACKED RelocalizeExt\s*\{(.*?)\};", text, re.S)
    assert m, "RelocalizeExt struct not found"
    body = m.group(1)
    assert "Relocalize base" in body and "uint32_t sources" in body
    assert "float    slamTranslationM" in body and "float    slamRotationDeg" in body
    for name, val in (("RelocSourcePose", svc.RELOC_SOURCE_POSE), ("RelocSourceSlamLog", svc.RELOC_SOURCE_SLAM_LOG)):
        assert re.search(rf"{name}\s*=\s*1u << {val.bit_length() - 1}\b", text), name

    pkt = svc.pack_relocalize(7, 123, (0.1, 0.2, 0.3), (1.0, 0.0, 0.0, 0.0),
                              sources=svc.RELOC_SOURCE_SLAM_LOG, slam_translation_m=1.5, slam_rotation_deg=60.0)
    base = struct.calcsize("<IHHI") + 8 + 12 + 16
    assert len(pkt) == base + 12, len(pkt)
    magic, version, mtype, seq = struct.unpack_from("<IHHI", pkt)
    assert (magic, version, mtype, seq) == (svc.MAGIC, svc.VERSION, svc.MSG_RELOCALIZE, 7)
    assert struct.unpack_from("<Iff", pkt, base) == (svc.RELOC_SOURCE_SLAM_LOG, 1.5, 60.0)


# Lines as XRService wrote them on a Steam Frame (2026-11-24), journalctl -o cat.
XR_LINES = [
    "Started failing: Just for logging: Stricter criterion for successfull tracking.",
    "SLAMConsole: [IMUFallback] Enabled, type: DIPr",
    "SLAMConsole: [IMUFallback] Enabled, type: 3DoF",
    "[DCU] [IMUFallback] Disabled with latest prediction:",
    "SLAMConsole: [IMUFallback] Disabled",
    "SLAMConsole: [Tracking] Internal reloc to previous map session (7322)",
    "Releasing map-VIO jump; justRecoveryRelocalized: 1, translation: 0.472464, rotation: 60.987299",
    "Large shown Map-VIO jump: shownT: 1.673187, shownR: 0.054275, diffT: 0.000000, diffR: 0.000000",
]


def test_slam_log_parsing():
    clock = [100.0]
    w = svc.SlamLogWatcher(clock=lambda: clock[0])
    assert not w.fallback
    w.feed_line(XR_LINES[0])
    assert not w.fallback
    w.feed_line(XR_LINES[1])
    assert w.fallback
    w.feed_line(XR_LINES[2])
    assert w.fallback and w.fallback_changes == 1
    w.feed_line(XR_LINES[3])
    assert not w.fallback
    w.feed_line(XR_LINES[4])
    assert w.fallback_changes == 2
    for line in XR_LINES[5:]:
        w.feed_line(line)
    jumps = w.take_jumps()
    assert len(jumps) == 1, jumps   # the "Large shown" repeat is not a new jump
    t, tm, rd = jumps[0]
    assert (t, round(tm, 6), round(rd, 6)) == (100.0, 0.472464, 60.987299)
    assert w.take_jumps() == []


def test_reporter_sends_once_per_jump():
    rep = svc.RelocReporter(match_secs=1.0)
    step = (5, (0.4, 0.0, 0.2), (1.0, 0.0, 0.0, 0.0))
    # pose first, SLAM log 40 ms later: one packet, from the pose
    r = rep.on_pose_step(10.00, step)
    assert r is not None and r["sources"] == svc.RELOC_SOURCE_POSE
    assert rep.on_slam_jump(10.04, 6, 0.45, 2.0) is None
    # SLAM log first, pose later: one packet, from the log
    r = rep.on_slam_jump(20.00, 7, 1.67, 0.05)
    assert r is not None and r["sources"] == svc.RELOC_SOURCE_SLAM_LOG and r["slam_translation_m"] == 1.67
    assert rep.on_pose_step(20.03, step) is None
    # far apart: two jumps
    assert rep.on_pose_step(30.0, step) is not None
    assert rep.on_slam_jump(32.0, 8, 0.5, 1.0) is not None


def test_fallback_samples_become_inertial():
    s = sample((1.0, 1.7, 2.0), q_yaw(10.0))
    f = svc.mark_fallback(s)
    assert f["flags"] & svc.FLAG_INERTIAL and not f["flags"] & svc.FLAG_POSITION
    assert f["flags"] & svc.FLAG_ROTATION and f["confidence"] == 0.2
    assert svc.mark_fallback(None) is None
    # and the detector ignores them: a runaway + snap back during fallback
    # (the 503 cm event in the 2026-11-24 test) is not reported
    base = walking(120)
    stream = base[:40] + [svc.mark_fallback(shifted(s, 0.0, (0.0, 0.0, 0.05 * k * k)))
                          for k, s in enumerate(base[40:80])] + base[80:]
    assert run(stream) == []


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"ok: {t.__name__}")
    print("\nframelink reloc tests passed")


if __name__ == "__main__":
    main()
