// Covers the two "runs continuously in the background, not just during
// calibration" consumers added for issue #18:
//
//   (1) Confidence-gated drift updates. Driver skips drift measurement
//       when the Frame reports low confidence or the InertialOnly flag.
//
//   (2) Relocalize events forwarded as HMD jumps. Each tick that polls a
//       NEW MsgRelocalize sets lastHmdJumpTime so the rest of the driver
//       (drift rebase, Stay Aligned external shift, clock discontinuity)
//       keys off it.
//
// The real driver code lives in ServerTrackedDeviceProvider.cpp, which
// pulls in half the project. These tests mirror the decision logic from
// the two sites I edited there verbatim, feed real packets through a
// real framelink::Listener, and assert the driver's state machine
// transitions the same way. If the state machine in the driver ever
// drifts from the one here, the mirror will be stale — grep this file
// first whenever the gate code changes.

#include "FrameLinkProtocol.h"
#include "FrameLink.h"

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

static void SendPose(uint32_t seq, float conf, uint32_t flags, uint16_t port)
{
    framelink::Pose m = {};
    m.h.magic = framelink::Magic;
    m.h.version = framelink::Version;
    m.h.type = framelink::MsgPose;
    m.h.sequence = seq;
    m.deviceId = 0;
    m.flags = flags;
    m.confidence = conf;
    m.position[0] = 0.0f; m.position[1] = 1.6f; m.position[2] = 0.0f;
    m.orientation[0] = 1.0f;
    SendRaw(&m, sizeof m, port);
}

static void SendRelocalize(uint32_t seq, float tx, float ty, float tz, uint16_t port)
{
    framelink::Relocalize m = {};
    m.h.magic = framelink::Magic;
    m.h.version = framelink::Version;
    m.h.type = framelink::MsgRelocalize;
    m.h.sequence = seq;
    m.captureTicks = seq * 1000ULL;
    m.translation[0] = tx; m.translation[1] = ty; m.translation[2] = tz;
    m.rotation[0] = 1.0f;
    SendRaw(&m, sizeof m, port);
}

// -----------------------------------------------------------------------------
// (1) Confidence gate — mirror of the block in ServerTrackedDeviceProvider.cpp
//     around line 1402. Sets `usable = false` when the Frame signals low
//     confidence or inertial-only. Tracks trust transitions.
// -----------------------------------------------------------------------------

struct DriftGate
{
    int trustState = -1;        // -1 unknown, 0 trusted, 1 low, 2 inertial
    double trustLogTime = 0.0;
    int logCount = 0;
    int lastLogTrust = -1;

    void tick(double nowSec, bool haveFramePose, const framelink::PoseSnapshot &snap, bool &usable)
    {
        const double minConfidence = 0.5;
        if (haveFramePose) {
            int trust = 0;
            if (snap.flags & framelink::PoseFlagInertialOnly) trust = 2;
            else if (snap.confidence < minConfidence)         trust = 1;
            if (trust != 0) usable = false;
            bool firstTransition = trustState == -1;
            if (trust != trustState && (firstTransition || nowSec - trustLogTime > 2.0)) {
                trustLogTime = nowSec;
                trustState = trust;
                // Suppress the implicit "all good" baseline log on first tick.
                if (!firstTransition || trust != 0) {
                    lastLogTrust = trust;
                    logCount++;
                }
            }
        } else if (trustState != -1 && trustState != 0 && nowSec - trustLogTime > 2.0) {
            trustLogTime = nowSec;
            trustState = -1;
        }
    }
};

// -----------------------------------------------------------------------------
// (2) Relocalize → jump. Driver polls listener.NextRelocalize() each tick;
//     when a new event is observed, it sets lastHmdJumpTime = now.
// -----------------------------------------------------------------------------

struct RelocHook
{
    uint32_t lastSeq = 0;
    double lastHmdJumpTime = -1e9;
    int observedCount = 0;

    bool tick(framelink::Listener &L, double nowSec)
    {
        framelink::RelocalizeEvent ev;
        uint32_t seq = 0;
        if (L.NextRelocalize(lastSeq, ev, seq)) {
            lastHmdJumpTime = nowSec;
            observedCount++;
            return true;
        }
        return false;
    }
};

// -----------------------------------------------------------------------------

#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL line %d: %s\n", __LINE__, #cond); std::exit(1); \
} } while (0)

int main(int argc, char **argv)
{
    uint16_t port = (uint16_t)(argc > 1 ? std::atoi(argv[1]) : 29489);
    CHECK(L.Start(port));

    // -- (1) confidence gate -------------------------------------------------
    {
        DriftGate g;
        double t = 0.0;

        // fresh hi-conf pose arrives -> usable stays true, trust=0, no log
        SendPose(1, 0.95f, framelink::PoseFlagPositionTracked | framelink::PoseFlagRotationTracked, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        {
            framelink::PoseSnapshot s; bool have = L.LatestPose(0, 0.3, s);
            bool usable = true;
            g.tick(t, have, s, usable);
            CHECK(have); CHECK(usable);
            CHECK(g.trustState == 0);
        }
        t += 3.0;

        // low-confidence pose -> usable flips false, trust=1, log counted once
        SendPose(2, 0.20f, framelink::PoseFlagPositionTracked, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        {
            framelink::PoseSnapshot s; bool have = L.LatestPose(0, 0.3, s);
            bool usable = true;
            int beforeLog = g.logCount;
            g.tick(t, have, s, usable);
            CHECK(have);
            CHECK(!usable);
            CHECK(g.trustState == 1);
            CHECK(g.logCount == beforeLog + 1);
            CHECK(g.lastLogTrust == 1);
        }
        t += 0.1;

        // Still low-conf; within 2s cooldown so no new log fires
        SendPose(3, 0.10f, framelink::PoseFlagPositionTracked, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        {
            framelink::PoseSnapshot s; bool have = L.LatestPose(0, 0.3, s);
            bool usable = true;
            int beforeLog = g.logCount;
            g.tick(t, have, s, usable);
            CHECK(!usable);
            CHECK(g.logCount == beforeLog);  // cooldown held
        }
        t += 3.0;

        // Inertial-only flag set even with high confidence — must still gate
        SendPose(4, 0.99f, framelink::PoseFlagInertialOnly, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        {
            framelink::PoseSnapshot s; bool have = L.LatestPose(0, 0.3, s);
            bool usable = true;
            int beforeLog = g.logCount;
            g.tick(t, have, s, usable);
            CHECK(have);
            CHECK(!usable);
            CHECK(g.trustState == 2);
            CHECK(g.logCount == beforeLog + 1);
            CHECK(g.lastLogTrust == 2);
        }
        t += 3.0;

        // Hi-conf again -> trust restored, log transition fires
        SendPose(5, 0.95f, framelink::PoseFlagPositionTracked | framelink::PoseFlagRotationTracked, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        {
            framelink::PoseSnapshot s; bool have = L.LatestPose(0, 0.3, s);
            bool usable = true;
            int beforeLog = g.logCount;
            g.tick(t, have, s, usable);
            CHECK(usable);
            CHECK(g.trustState == 0);
            CHECK(g.logCount == beforeLog + 1);
            CHECK(g.lastLogTrust == 0);
        }
        t += 3.0;

        // Frame stream stops entirely — gate must release so we don't stay stuck
        std::this_thread::sleep_for(std::chrono::milliseconds(400));  // age past 0.3s window
        // Pretend we were in trust=1 before the stream stopped:
        g.trustState = 1;
        g.trustLogTime = t - 3.0;
        {
            framelink::PoseSnapshot s; bool have = L.LatestPose(0, 0.3, s);
            bool usable = true;
            g.tick(t, have, s, usable);
            CHECK(!have);
            CHECK(g.trustState == -1);   // released
        }

        std::printf("ok: confidence gate (hi -> low -> cooldown -> inertial -> restored -> released)\n");
    }

    // -- (2) relocalize -> HMD jump -----------------------------------------
    {
        RelocHook r;
        double t = 10.0;

        // First tick: no relocalize yet
        {
            bool fired = r.tick(L, t);
            CHECK(!fired);
            CHECK(r.observedCount == 0);
            CHECK(r.lastHmdJumpTime < 0.0);
        }

        // Relocalize event arrives
        SendRelocalize(42, 0.1f, 0.0f, -0.05f, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        t = 11.0;
        {
            bool fired = r.tick(L, t);
            CHECK(fired);
            CHECK(r.observedCount == 1);
            CHECK(r.lastHmdJumpTime == 11.0);
            CHECK(r.lastSeq == 42);
        }

        // Next tick, no new reloc — must NOT re-fire the jump signal
        t = 11.1;
        {
            bool fired = r.tick(L, t);
            CHECK(!fired);
            CHECK(r.observedCount == 1);
            CHECK(r.lastHmdJumpTime == 11.0);
        }

        // Two relocs stacked between ticks — only the newer one wins, both are
        // accounted for in one jump (we don't want to double-count).
        SendRelocalize(50, 0.0f, 0.0f, 0.0f, port);
        SendRelocalize(60, 0.2f, 0.0f, 0.0f, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        t = 12.0;
        {
            int before = r.observedCount;
            bool fired = r.tick(L, t);
            CHECK(fired);
            CHECK(r.observedCount == before + 1);
            CHECK(r.lastSeq == 60);
            CHECK(r.lastHmdJumpTime == 12.0);
        }

        // Sequence we've already consumed replays (shouldn't happen, but
        // verify we don't re-fire). Send an older sequence; NextRelocalize
        // should reject it since our cursor is at 60.
        SendRelocalize(30, 0.0f, 0.0f, 0.0f, port);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        t = 13.0;
        {
            int before = r.observedCount;
            bool fired = r.tick(L, t);
            CHECK(!fired);
            CHECK(r.observedCount == before);
        }

        std::printf("ok: relocalize event -> HMD jump (first, no-repeat, stacked latest-wins, old-seq rejected)\n");
    }

    L.Stop();
    std::printf("\nframelink background tests passed\n");
    return 0;
}
