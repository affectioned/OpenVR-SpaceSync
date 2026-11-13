// SPDX-License-Identifier: AGPL-3.0-only
// Steam Frame → PC data link for SpaceSync. See issue #18.
//
// This header is the wire contract between a small SteamOS-side service and
// SpaceSync's driver. Both sides include it. Numbers are little-endian; structs
// are packed. Transport is unreliable UDP — every message is self-contained,
// carries a header with magic/version/sequence, and consumers are expected to
// cope with drops, reorder, and silence.
//
// Default UDP port: 29485 (SpaceSync listens; Frame sends).
//
// If the service never shows up the driver falls back to the normal flow, so
// nothing changes for non-Frame users.

#pragma once

#include <cstdint>

#ifdef _MSC_VER
#pragma pack(push, 1)
#define FRAMELINK_PACKED
#else
#define FRAMELINK_PACKED __attribute__((packed))
#endif

namespace framelink
{
	constexpr uint16_t Port = 29485;

	// 'SFLK' little-endian.
	constexpr uint32_t Magic = 0x4B4C4653u;

	// Bump on any struct layout change.
	constexpr uint16_t Version = 1;

	enum MessageType : uint16_t
	{
		MsgInvalid     = 0,
		MsgHello       = 1,   // Frame → PC. Sent periodically (≈1 Hz) while the service is alive.
		MsgPose        = 2,   // Frame → PC. One per device sample, raw + confidence + capture ticks.
		MsgGravity     = 3,   // Frame → PC. Gravity in Frame's SLAM space.
		MsgRelocalize  = 4,   // Frame → PC. A SLAM relocalization just happened.
		MsgClockPing   = 5,   // PC → Frame. Clock-sync probe.
		MsgClockPong   = 6,   // Frame → PC. Clock-sync reply.
	};

	// Pose::flags
	enum PoseFlag : uint32_t
	{
		PoseFlagPositionTracked = 1 << 0,
		PoseFlagRotationTracked = 1 << 1,
		PoseFlagInertialOnly    = 1 << 2,  // the Frame is coasting on IMU only
	};

	struct FRAMELINK_PACKED Header
	{
		uint32_t magic;      // Magic
		uint16_t version;    // Version
		uint16_t type;       // MessageType
		uint32_t sequence;   // monotonic per sender, wraps freely
	};

	struct FRAMELINK_PACKED Hello
	{
		Header h;
		char     productId[32];    // free-form, e.g. "steam-frame"
		uint64_t frameClockHz;     // ticks per second of the clock used in all `*Ticks` fields below
		uint64_t frameNowTicks;    // Frame's clock at send time; paired with ClockPing/Pong for offset
	};

	struct FRAMELINK_PACKED Pose
	{
		Header   h;
		uint64_t captureTicks;     // Frame clock, when the sample was actually captured (not predicted)
		uint32_t deviceId;         // Frame-local id. 0 = HMD, 1+ = controllers/trackers
		uint32_t flags;            // PoseFlag bitmask
		float    confidence;       // 0..1; calibration can drop samples below a threshold
		float    position[3];      // meters, Frame SLAM space
		float    orientation[4];   // wxyz quaternion, Frame SLAM space
		float    linVelocity[3];   // m/s, Frame SLAM space
		float    angVelocity[3];   // rad/s, Frame SLAM space
	};

	struct FRAMELINK_PACKED Gravity
	{
		Header   h;
		uint64_t captureTicks;
		float    direction[3];     // unit vector in Frame SLAM space, points "down"
		float    confidence;       // 0..1
	};

	struct FRAMELINK_PACKED Relocalize
	{
		Header   h;
		uint64_t captureTicks;
		float    translation[3];   // origin shift in Frame SLAM space
		float    rotation[4];      // wxyz; rotation applied to the frame origin
	};

	struct FRAMELINK_PACKED ClockProbe
	{
		Header   h;
		uint64_t pcTicks;          // QPC ticks at PC when ping was sent
		uint64_t frameTicks;       // Frame clock when the frame handled the ping (0 on ping)
	};
}

#ifdef _MSC_VER
#pragma pack(pop)
#endif
#undef FRAMELINK_PACKED
