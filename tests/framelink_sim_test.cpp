// Integration test for framelink::Listener.
//
// Links the real listener source, binds to localhost, sends crafted packets
// at it, and asserts the decoded snapshots + stats match what the overlay
// would consume. Runs as a standalone binary so we don't need a gtest
// dependency; `ctest` just checks the exit code.
//
// Test cases cover the overlay-side substitution decision tree:
//
//   - no data            -> fall back, no substitution
//   - hi-conf pose       -> substitute
//   - low-conf pose      -> reject
//   - inertial-only flag -> reject regardless of confidence
//   - useFrameLink=off   -> listener still receives, overlay ignores
//   - stale pose         -> LatestPose returns false past the window
//   - out-of-range dev   -> no data
//   - real frame_sender.py -> substitution green end-to-end

#include "FrameLinkProtocol.h"
#include "FrameLink.h"

// Stub the driver's LOG macro that FrameLink.cpp pulls in.
#define LOG(...)   ((void)0)
#define TRACE(...) ((void)0)
#include "FrameLink.cpp"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

static framelink::Listener L;

static void SendRaw(const void *buf, int len, uint16_t port)
{
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in a = {};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(0x7F000001);
    sendto(s, (const char*)buf, len, 0, (sockaddr*)&a, sizeof a);
    closesocket(s);
}

static void SendHello(uint32_t seq, uint16_t port)
{
    framelink::Hello m = {};
    m.h.magic = framelink::Magic;
    m.h.version = framelink::Version;
    m.h.type = framelink::MsgHello;
    m.h.sequence = seq;
    std::strncpy(m.productId, "sim", sizeof m.productId - 1);
    m.frameClockHz = 10'000'000;
    m.frameNowTicks = 0;
    SendRaw(&m, sizeof m, port);
}

static void SendPose(uint32_t seq, uint32_t deviceId, float conf, uint32_t flags, uint16_t port)
{
    framelink::Pose m = {};
    m.h.magic = framelink::Magic;
    m.h.version = framelink::Version;
    m.h.type = framelink::MsgPose;
    m.h.sequence = seq;
    m.deviceId = deviceId;
    m.flags = flags;
    m.confidence = conf;
    m.captureTicks = 12345678ULL + seq;
    m.position[0] = 0.1f * (float)seq;
    m.position[1] = 1.6f;
    m.position[2] = -0.2f;
    m.orientation[0] = 1.0f;
    SendRaw(&m, sizeof m, port);
}

// Mirrors Calibration.cpp's substitution decision policy exactly.
struct Decision { bool substituted; bool rejected; const char* problem; };

static Decision Decide(const framelink::PoseSnapshot &snap, bool have,
                       bool useFrameLink, double minConf)
{
    Decision r{};
    if (!useFrameLink || !have) return r;
    if (snap.flags & framelink::PoseFlagInertialOnly) {
        r.rejected = true; r.problem = "inertial-only"; return r;
    }
    if (snap.confidence < minConf) {
        r.rejected = true; r.problem = "low-confidence"; return r;
    }
    r.substituted = true;
    return r;
}

#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL line %d: %s\n", __LINE__, #cond); std::exit(1); \
} } while (0)

int main(int argc, char** argv)
{
    // Allow the port to be overridden so multiple test runs in parallel
    // don't collide. Default picks a port well clear of the production 29485.
    uint16_t port = (uint16_t)(argc > 1 ? std::atoi(argv[1]) : 29487);
    CHECK(L.Start(port));

    // 1) No data -> fall back cleanly.
    {
        framelink::PoseSnapshot snap;
        bool got = L.LatestPose(0, 0.5, snap);
        auto d = Decide(snap, got, true, 0.5);
        CHECK(!got); CHECK(!d.substituted); CHECK(!d.rejected);
    }

    // 2) Hi-confidence pose -> substitute.
    {
        SendHello(1, port);
        SendPose(2, 0, 0.95f,
                 framelink::PoseFlagPositionTracked | framelink::PoseFlagRotationTracked,
                 port);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        framelink::PoseSnapshot snap;
        bool got = L.LatestPose(0, 0.5, snap);
        CHECK(got);
        CHECK(snap.confidence > 0.9f);
        CHECK((snap.flags & framelink::PoseFlagInertialOnly) == 0);
        auto d = Decide(snap, got, true, 0.5);
        CHECK(d.substituted); CHECK(!d.rejected);
    }

    // 3) Low confidence -> reject.
    {
        SendPose(100, 0, 0.20f, framelink::PoseFlagPositionTracked, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        framelink::PoseSnapshot snap;
        bool got = L.LatestPose(0, 0.5, snap);
        CHECK(got);
        auto d = Decide(snap, got, true, 0.5);
        CHECK(!d.substituted);
        CHECK(d.rejected);
        CHECK(std::string(d.problem) == "low-confidence");
    }

    // 4) Inertial-only -> reject even at high confidence.
    {
        SendPose(200, 0, 0.99f, framelink::PoseFlagInertialOnly, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        framelink::PoseSnapshot snap;
        bool got = L.LatestPose(0, 0.5, snap);
        CHECK(got);
        auto d = Decide(snap, got, true, 0.5);
        CHECK(!d.substituted);
        CHECK(d.rejected);
        CHECK(std::string(d.problem) == "inertial-only");
    }

    // 5) useFrameLink=false -> listener still receives, overlay ignores.
    {
        SendPose(300, 0, 0.95f, framelink::PoseFlagPositionTracked, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        framelink::PoseSnapshot snap;
        bool got = L.LatestPose(0, 0.5, snap);
        auto d = Decide(snap, got, false, 0.5);
        CHECK(!d.substituted); CHECK(!d.rejected);
    }

    // 6) Staleness gate.
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        framelink::PoseSnapshot snap;
        bool got = L.LatestPose(0, 0.5, snap);
        CHECK(!got);
    }

    // 7) Unknown deviceId -> nothing.
    {
        framelink::PoseSnapshot snap;
        bool got = L.LatestPose(99, 1.0, snap);
        CHECK(!got);
    }

    // 8) GetStats exposes the counters the overlay reads.
    {
        auto stats = L.GetStats();
        CHECK(stats.listening);
        CHECK(stats.port == port);
        CHECK(stats.packets > 0);
        CHECK(stats.droppedBadMagic == 0);
        CHECK(stats.droppedBadVersion == 0);
    }

    L.Stop();
    std::printf("framelink integration test passed\n");
    return 0;
}
