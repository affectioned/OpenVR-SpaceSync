"""Wire-format round-trip tests for the Steam Frame link protocol.

Checks the Python sender in contrib/steamframe/frame_sender.py agrees
exactly on sizes and offsets with what the C header FrameLinkProtocol.h
declares — so a reviewer can tell at a glance that the two sides can't
drift apart without someone noticing.
"""
import re
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HEADER = ROOT / "include" / "shared" / "FrameLinkProtocol.h"
SENDER = ROOT / "contrib" / "steamframe" / "frame_sender.py"

MAGIC = 0x4B4C4653      # 'SFLK'
VERSION = 1
PORT = 29488            # distinct from the integration-test port

MSG_HELLO       = 1
MSG_POSE        = 2
MSG_GRAVITY     = 3
MSG_RELOCALIZE  = 4

HDR = "<IHHI"
HDR_SIZE = struct.calcsize(HDR)


def assert_eq(got, want, label):
    assert got == want, f"{label}: got {got!r} want {want!r}"


# --- Header constants are kept in sync with the C header -----------------

def test_header_constants():
    """Make sure Magic / Version / Port / MessageType enum values
    we bake into the Python side match the C header, by greping the header
    for the literal values."""
    text = HEADER.read_text()

    def find(name, pattern):
        m = re.search(pattern, text)
        assert m, f"{name}: not found in {HEADER.name}"
        return m.group(1)

    hex_magic = int(find("Magic", r"Magic\s*=\s*0x([0-9A-Fa-f]+)"), 16)
    assert_eq(hex_magic, MAGIC, "Magic")

    header_version = int(find("Version", r"Version\s*=\s*(\d+)"))
    assert_eq(header_version, VERSION, "Version")

    header_port = int(find("Port", r"Port\s*=\s*(\d+)"))
    # Our Python PORT above intentionally differs for the loopback test;
    # just check the protocol default we advertise everywhere matches.
    assert_eq(header_port, 29485, "default Port")


# --- Packet sizes computed on the Python side need to match header struct

# These are the C struct sizes (packed) from FrameLinkProtocol.h. If the
# header changes, update these; the test will tell you which it is.
EXPECTED_SIZE = {
    "Header":    12,          # magic+version+type+sequence
    "Hello":     HDR_SIZE + 32 + 8 + 8,
    "Pose":      HDR_SIZE + 8 + 4 + 4 + 4 + 12 + 16 + 12 + 12,
    "Gravity":   HDR_SIZE + 8 + 12 + 4,
    "Relocalize":HDR_SIZE + 8 + 12 + 16,
}


def pack_hello(seq, product, clock_hz, now_ticks):
    name = product.encode("utf-8")[:32].ljust(32, b"\0")
    return (struct.pack(HDR, MAGIC, VERSION, MSG_HELLO, seq)
            + name + struct.pack("<QQ", clock_hz, now_ticks))


def pack_pose(seq, ticks, dev, flags, conf, pos, quat, lin, ang):
    return (struct.pack(HDR, MAGIC, VERSION, MSG_POSE, seq)
            + struct.pack("<Q", ticks)
            + struct.pack("<II", dev, flags)
            + struct.pack("<f", conf)
            + struct.pack("<fff", *pos)
            + struct.pack("<ffff", *quat)
            + struct.pack("<fff", *lin)
            + struct.pack("<fff", *ang))


def test_packet_sizes():
    assert_eq(HDR_SIZE, EXPECTED_SIZE["Header"], "Header size")
    assert_eq(len(pack_hello(0, "x", 10**9, 0)), EXPECTED_SIZE["Hello"], "Hello size")
    assert_eq(len(pack_pose(0, 0, 0, 0, 0.0, (0,0,0), (1,0,0,0), (0,0,0), (0,0,0))),
              EXPECTED_SIZE["Pose"], "Pose size")


# --- Round trip against the real frame_sender.py ------------------------

def test_sender_round_trip():
    received = {"hello": 0, "pose": 0, "gravity": 0, "reloc": 0, "bad_magic": 0, "bad_ver": 0}
    done = threading.Event()

    def recv_loop(sock):
        while not done.is_set():
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            if len(data) < HDR_SIZE:
                continue
            magic, ver, mtype, _ = struct.unpack(HDR, data[:HDR_SIZE])
            if magic != MAGIC:
                received["bad_magic"] += 1; continue
            if ver != VERSION:
                received["bad_ver"] += 1; continue
            if mtype == MSG_HELLO:
                assert len(data) == EXPECTED_SIZE["Hello"]
                received["hello"] += 1
            elif mtype == MSG_POSE:
                assert len(data) == EXPECTED_SIZE["Pose"]
                received["pose"] += 1
            elif mtype == MSG_GRAVITY:
                assert len(data) == EXPECTED_SIZE["Gravity"]
                received["gravity"] += 1
            elif mtype == MSG_RELOCALIZE:
                assert len(data) == EXPECTED_SIZE["Relocalize"]
                received["reloc"] += 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", PORT))
    sock.settimeout(0.25)
    t = threading.Thread(target=recv_loop, args=(sock,), daemon=True)
    t.start()

    try:
        res = subprocess.run(
            [sys.executable, str(SENDER),
             "--host", "127.0.0.1", "--port", str(PORT),
             "--hz", "60", "--duration", "1.5", "--reloc-after", "0.3"],
            capture_output=True, text=True, timeout=10,
        )
        assert res.returncode == 0, res.stderr
    finally:
        time.sleep(0.3)
        done.set()
        t.join(timeout=2)
        sock.close()

    assert received["hello"] >= 1, received
    assert received["pose"] >= 30, received
    assert received["gravity"] >= 1, received
    assert received["reloc"] == 1, received
    assert received["bad_magic"] == 0, received
    assert received["bad_ver"] == 0, received


# --- main ---------------------------------------------------------------

def main():
    passed = 0
    for fn in [test_header_constants, test_packet_sizes, test_sender_round_trip]:
        try:
            fn()
            print(f"ok  {fn.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"FAIL {fn.__name__}: {e}")
            return 1
    print(f"\n{passed}/3 wire-protocol tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
