// SPDX-License-Identifier: AGPL-3.0-only
// Driver-side UDP listener for Steam Frame data. See issue #18 and shared/FrameLink.h.
//
// Scaffold only: receives packets, parses them, stores the latest snapshot in a
// mutex-guarded cache, and logs enough to confirm the pipeline is alive. Hooking
// the snapshot into the calibrator is a follow-up — on purpose, so the wire
// contract can be iterated on without churning the calibration code.

#pragma once

#include "FrameLinkProtocol.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace framelink
{
	struct PoseSnapshot
	{
		bool     valid = false;
		uint32_t deviceId = 0;
		uint32_t flags = 0;
		float    confidence = 0.0f;
		float    position[3] = { 0, 0, 0 };
		float    orientation[4] = { 1, 0, 0, 0 };
		float    linVelocity[3] = { 0, 0, 0 };
		float    angVelocity[3] = { 0, 0, 0 };
		uint64_t captureTicks = 0;  // Frame clock, raw
		double   pcSeconds = 0.0;   // PC QPC seconds at receive time
	};

	struct GravitySnapshot
	{
		bool   valid = false;
		float  direction[3] = { 0, -1, 0 };
		float  confidence = 0.0f;
		double pcSeconds = 0.0;
	};

	struct RelocalizeEvent
	{
		uint64_t captureTicks = 0;
		float    translation[3] = { 0, 0, 0 };
		float    rotation[4] = { 1, 0, 0, 0 };
		double   pcSeconds = 0.0;
	};

	class Listener
	{
	public:
		Listener() = default;
		~Listener() { Stop(); }

		bool Start(uint16_t port = Port);
		void Stop();
		bool Running() const { return running.load(); }

		// Thread-safe snapshot accessors. Return false when no data has ever arrived
		// or when the last sample is older than `maxAgeSeconds`.
		bool LatestPose(uint32_t deviceId, double maxAgeSeconds, PoseSnapshot& out) const;
		bool LatestGravity(double maxAgeSeconds, GravitySnapshot& out) const;

		// Latest relocalize event, strictly newer than the sequence the caller has seen.
		// Starting value for `lastSeq` is 0. The event's own sequence is returned via `outSeq`.
		bool NextRelocalize(uint32_t& lastSeq, RelocalizeEvent& out, uint32_t& outSeq) const;

		// For logging / UI.
		std::string PeerDescription() const;

		struct Stats
		{
			bool listening = false;
			uint16_t port = 0;
			bool hasPeer = false;
			std::string peer;
			std::string product;
			uint64_t frameClockHz = 0;
			uint64_t packets = 0;
			uint64_t droppedBadMagic = 0;
			uint64_t droppedBadVersion = 0;
			double lastPacketSec = -1.0;
			double lastHelloSec = -1.0;
			double lastPoseSec = -1.0;
			double lastPoseConfidence = 0.0;
		};
		Stats GetStats() const;

	private:
		void Loop();
		void Handle(const uint8_t* buf, int len, double nowSec, const std::string& peer);

		std::atomic<bool> running{ false };
		std::thread worker;
		// Raw socket handle. void* to avoid dragging winsock.h into the header.
		std::atomic<uintptr_t> sock{ (uintptr_t)-1 };

		static const int MaxPoseDevices = 16;
		mutable std::mutex mutex;
		PoseSnapshot poses[MaxPoseDevices];
		GravitySnapshot gravity;
		RelocalizeEvent relocQueue[8];
		uint32_t relocSeq[8] = {};    // Header.sequence of each stored event
		uint32_t relocWrite = 0;      // next write index
		uint32_t relocLast = 0;       // highest sequence seen so far
		std::string peer;
		std::string product;
		uint64_t frameClockHz = 0;
		uint64_t packets = 0;
		uint64_t droppedBadMagic = 0;
		uint64_t droppedBadVersion = 0;
		uint16_t boundPort = 0;
		double lastPacketSec = -1.0;
		double lastHelloSec = -1.0;
		double lastPoseSec = -1.0;
		double lastPoseConfidence = 0.0;
	};
}
