// SPDX-License-Identifier: AGPL-3.0-only

#include "FrameLink.h"
#include "Logging.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <cstdio>
#include <cstring>

namespace framelink
{
	namespace
	{
		double QpcSeconds()
		{
			LARGE_INTEGER now, freq;
			QueryPerformanceCounter(&now);
			QueryPerformanceFrequency(&freq);
			return now.QuadPart / (double)freq.QuadPart;
		}

		bool EnsureWinsock()
		{
			static bool initialized = false;
			static bool ok = false;
			if (initialized)
				return ok;
			WSADATA data;
			ok = WSAStartup(MAKEWORD(2, 2), &data) == 0;
			initialized = true;
			return ok;
		}

		std::string FormatAddr(const sockaddr_in& addr)
		{
			char buf[INET_ADDRSTRLEN] = {};
			inet_ntop(AF_INET, &addr.sin_addr, buf, sizeof buf);
			char out[64];
			std::snprintf(out, sizeof out, "%s:%u", buf, ntohs(addr.sin_port));
			return out;
		}
	}

	bool Listener::Start(uint16_t port)
	{
		if (running.load())
			return true;
		if (!EnsureWinsock())
		{
			LOG("FrameLink: WSAStartup failed");
			return false;
		}
		SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (s == INVALID_SOCKET)
		{
			LOG("FrameLink: socket() failed (%d)", WSAGetLastError());
			return false;
		}
		DWORD timeoutMs = 500;
		setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutMs, sizeof timeoutMs);

		sockaddr_in addr = {};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_port = htons(port);
		if (bind(s, (sockaddr*)&addr, sizeof addr) == SOCKET_ERROR)
		{
			LOG("FrameLink: bind() on port %u failed (%d) — Steam Frame integration disabled", port, WSAGetLastError());
			closesocket(s);
			return false;
		}

		sock.store((uintptr_t)s);
		running.store(true);
		{
			std::lock_guard<std::mutex> lk(mutex);
			boundPort = port;
		}
		worker = std::thread(&Listener::Loop, this);
		LOG("FrameLink: listening on UDP %u", port);
		return true;
	}

	void Listener::Stop()
	{
		if (!running.exchange(false))
			return;
		SOCKET s = (SOCKET)sock.exchange((uintptr_t)-1);
		if (s != (SOCKET)-1 && s != INVALID_SOCKET)
			closesocket(s);
		if (worker.joinable())
			worker.join();
		LOG("FrameLink: stopped (packets=%llu, bad-magic=%llu, bad-version=%llu)",
			(unsigned long long)packets,
			(unsigned long long)droppedBadMagic,
			(unsigned long long)droppedBadVersion);
	}

	void Listener::Loop()
	{
		uint8_t buf[2048];
		SOCKET s = (SOCKET)sock.load();
		while (running.load())
		{
			sockaddr_in src = {};
			int srcLen = sizeof src;
			int n = recvfrom(s, (char*)buf, sizeof buf, 0, (sockaddr*)&src, &srcLen);
			if (n == SOCKET_ERROR)
			{
				int err = WSAGetLastError();
				if (err == WSAETIMEDOUT || err == WSAEINTR) continue;
				if (!running.load()) break;
				LOG("FrameLink: recvfrom error %d, stopping", err);
				break;
			}
			if (n <= 0) continue;
			Handle(buf, n, QpcSeconds(), FormatAddr(src));
		}
	}

	void Listener::Handle(const uint8_t* buf, int len, double nowSec, const std::string& senderPeer)
	{
		if (len < (int)sizeof(Header))
			return;
		Header h;
		std::memcpy(&h, buf, sizeof h);
		if (h.magic != Magic)
		{
			std::lock_guard<std::mutex> lk(mutex);
			droppedBadMagic++;
			return;
		}
		if (h.version != Version)
		{
			std::lock_guard<std::mutex> lk(mutex);
			droppedBadVersion++;
			return;
		}

		std::lock_guard<std::mutex> lk(mutex);
		packets++;
		peer = senderPeer;
		lastPacketSec = nowSec;

		switch (h.type)
		{
		case MsgHello:
		{
			if (len < (int)sizeof(Hello)) break;
			Hello msg;
			std::memcpy(&msg, buf, sizeof msg);
			frameClockHz = msg.frameClockHz;
			char productBuf[33] = {};
			std::memcpy(productBuf, msg.productId, sizeof msg.productId);
			productBuf[32] = 0;
			bool firstHello = product.empty();
			product = productBuf;
			lastHelloSec = nowSec;
			if (firstHello)
				LOG("FrameLink: hello from %s product=\"%s\" clockHz=%llu",
					senderPeer.c_str(), productBuf, (unsigned long long)msg.frameClockHz);
			break;
		}
		case MsgPose:
		{
			if (len < (int)sizeof(Pose)) break;
			Pose msg;
			std::memcpy(&msg, buf, sizeof msg);
			if (msg.deviceId >= (uint32_t)MaxPoseDevices) break;
			PoseSnapshot& snap = poses[msg.deviceId];
			snap.valid = true;
			snap.deviceId = msg.deviceId;
			snap.flags = msg.flags;
			snap.confidence = msg.confidence;
			std::memcpy(snap.position, msg.position, sizeof snap.position);
			std::memcpy(snap.orientation, msg.orientation, sizeof snap.orientation);
			std::memcpy(snap.linVelocity, msg.linVelocity, sizeof snap.linVelocity);
			std::memcpy(snap.angVelocity, msg.angVelocity, sizeof snap.angVelocity);
			snap.captureTicks = msg.captureTicks;
			snap.pcSeconds = nowSec;
			lastPoseSec = nowSec;
			lastPoseConfidence = msg.confidence;
			break;
		}
		case MsgGravity:
		{
			if (len < (int)sizeof(Gravity)) break;
			Gravity msg;
			std::memcpy(&msg, buf, sizeof msg);
			gravity.valid = true;
			std::memcpy(gravity.direction, msg.direction, sizeof gravity.direction);
			gravity.confidence = msg.confidence;
			gravity.pcSeconds = nowSec;
			break;
		}
		case MsgRelocalize:
		{
			if (len < (int)sizeof(Relocalize)) break;
			Relocalize msg;
			std::memcpy(&msg, buf, sizeof msg);
			int slot = relocWrite % 8;
			relocQueue[slot] = {};
			relocQueue[slot].captureTicks = msg.captureTicks;
			std::memcpy(relocQueue[slot].translation, msg.translation, sizeof relocQueue[slot].translation);
			std::memcpy(relocQueue[slot].rotation, msg.rotation, sizeof relocQueue[slot].rotation);
			relocQueue[slot].pcSeconds = nowSec;
			relocSeq[slot] = h.sequence;
			relocWrite++;
			if (h.sequence > relocLast) relocLast = h.sequence;
			LOG("FrameLink: relocalize seq=%u dt=%.3f m", h.sequence,
				(double)(msg.translation[0] * msg.translation[0] + msg.translation[1] * msg.translation[1] + msg.translation[2] * msg.translation[2]));
			break;
		}
		case MsgClockPong:
		{
			// Scaffold: not driving a clock aligner yet. Log for diagnostics.
			if (len < (int)sizeof(ClockProbe)) break;
			ClockProbe msg;
			std::memcpy(&msg, buf, sizeof msg);
			LOG("FrameLink: clock pong pc=%llu frame=%llu",
				(unsigned long long)msg.pcTicks, (unsigned long long)msg.frameTicks);
			break;
		}
		default:
			break;
		}
	}

	bool Listener::LatestPose(uint32_t deviceId, double maxAgeSeconds, PoseSnapshot& out) const
	{
		if (deviceId >= (uint32_t)MaxPoseDevices) return false;
		std::lock_guard<std::mutex> lk(mutex);
		const PoseSnapshot& snap = poses[deviceId];
		if (!snap.valid) return false;
		if (QpcSeconds() - snap.pcSeconds > maxAgeSeconds) return false;
		out = snap;
		return true;
	}

	bool Listener::LatestGravity(double maxAgeSeconds, GravitySnapshot& out) const
	{
		std::lock_guard<std::mutex> lk(mutex);
		if (!gravity.valid) return false;
		if (QpcSeconds() - gravity.pcSeconds > maxAgeSeconds) return false;
		out = gravity;
		return true;
	}

	bool Listener::NextRelocalize(uint32_t& lastSeq, RelocalizeEvent& out, uint32_t& outSeq) const
	{
		std::lock_guard<std::mutex> lk(mutex);
		uint32_t bestSeq = lastSeq;
		int bestSlot = -1;
		for (int i = 0; i < 8; i++)
		{
			if (relocSeq[i] == 0 && relocQueue[i].captureTicks == 0) continue;
			if (relocSeq[i] > bestSeq)
			{
				bestSeq = relocSeq[i];
				bestSlot = i;
			}
		}
		if (bestSlot < 0) return false;
		out = relocQueue[bestSlot];
		outSeq = bestSeq;
		lastSeq = bestSeq;
		return true;
	}

	Listener::Stats Listener::GetStats() const
	{
		std::lock_guard<std::mutex> lk(mutex);
		Stats s;
		s.listening = running.load();
		s.port = boundPort;
		s.hasPeer = !peer.empty();
		s.peer = peer;
		s.product = product;
		s.frameClockHz = frameClockHz;
		s.packets = packets;
		s.droppedBadMagic = droppedBadMagic;
		s.droppedBadVersion = droppedBadVersion;
		s.lastPacketSec = lastPacketSec;
		s.lastHelloSec = lastHelloSec;
		s.lastPoseSec = lastPoseSec;
		s.lastPoseConfidence = lastPoseConfidence;
		return s;
	}

	std::string Listener::PeerDescription() const
	{
		std::lock_guard<std::mutex> lk(mutex);
		if (peer.empty()) return "(none)";
		char buf[128];
		std::snprintf(buf, sizeof buf, "%s packets=%llu", peer.c_str(), (unsigned long long)packets);
		return buf;
	}
}
