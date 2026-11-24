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
    pkt = svc.pack_relocalize(7, 123, (0.1, 0.2, 0.3), (1.0, 0.0, 0.0, 0.0))
    assert len(pkt) == struct.calcsize("<IHHI") + 8 + 12 + 16, len(pkt)
    magic, version, mtype, seq = struct.unpack_from("<IHHI", pkt)
    assert (magic, version, mtype, seq) == (svc.MAGIC, svc.VERSION, svc.MSG_RELOCALIZE, 7)


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"ok: {t.__name__}")
    print("\nframelink reloc tests passed")


if __name__ == "__main__":
    main()
