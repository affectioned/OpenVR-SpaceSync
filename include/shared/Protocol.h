// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <cstdint>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif

#define SPACESYNC_PIPE_NAME "\\\\.\\pipe\\SpaceSyncCom"

namespace protocol
{
	const uint32_t Version = 21;

	enum RequestType
	{
		RequestInvalid,
		RequestHandshake,
		RequestSetDeviceTransform,
		RequestSetHmdTracker,
		RequestSetSlamSync,
		RequestSetOneEuro,
		RequestGetStatus,
		RequestSetUniverseLock,
		RequestGetFrameLinkSample,
		RequestGetFrameLinkStatus,
	};

	enum ResponseType
	{
		ResponseInvalid,
		ResponseHandshake,
		ResponseSuccess,
		ResponseStatus,
		ResponseFrameLinkSample,
		ResponseFrameLinkStatus,
	};

	struct Protocol
	{
		uint32_t version = Version;
	};

	struct SetDeviceTransform
	{
		uint32_t openVRID;
		bool enabled;
		bool updateTranslation;
		bool updateRotation;
		bool updateScale;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;

		SetDeviceTransform(uint32_t id, bool enabled) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(false), updateScale(false), translation(translation) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(true), updateScale(false), rotation(rotation) { }

		SetDeviceTransform(uint32_t id, bool enabled, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(true), scale(scale) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(false), translation(translation), rotation(rotation) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(true), translation(translation), rotation(rotation), scale(scale) { }
	};

	struct SetHmdTracker
	{
		uint32_t hmdID;
		uint32_t trackerID;
		bool enabled;
		bool slamFallback;
		bool enableAngularVelocity;
		float predictionTime;
		vr::HmdQuaternion_t offsetRotation;
		vr::HmdVector3d_t offsetTranslation;
		vr::HmdQuaternion_t calibrationRotation;
		vr::HmdVector3d_t calibrationTranslation;
		double calibrationScale;
		// Headset tracking space scale relative to the head tracker's lighthouse space.
		double hmdScale;
		// Follow SLAM HMD: headset keeps its SLAM pose, lighthouse devices (incl. head tracker) follow it.
		bool followSlamHmd;
		// Follow mode only.
		bool hideHeadTracker;
		bool tiltSeedValid;
		vr::HmdVector3d_t tiltSeed;
		bool stayAligned;
		uint32_t stayHipID;
		bool calibrationLatencyValid;
		double calibrationLatencyRot;
		double calibrationLatencyPos;
	};

	struct SetSlamSync
	{
		uint32_t openVRID;
		bool enabled;
	};

	struct OneEuroParams
	{
		double minCutoff;
		double beta;
		double dCutoff;
	};

	struct SetOneEuro
	{
		bool headEnabled;
		OneEuroParams head;
		OneEuroParams drift;
		double deviceSmoothing;
		double latencyCompensation;
	};

	struct SetUniverseLock
	{
		bool enabled;
		bool calibrating;
		uint32_t command;
	};

	struct DriverStatus
	{
		bool driftValid;
		bool mountShiftSuspected;
		double tiltDeg;
		double translationDeviationM;
		double latencyMs;
		double latencyPosMs;
		uint32_t jumpsCompensated;
		double sigmaYawDeg;
		double sigmaTranslationM;
		double calmSeconds;
		uint32_t refinementSolves;
		uint32_t refinementTranslationSolves;
		bool refinementValid;
		vr::HmdQuaternion_t offsetRotation;
		vr::HmdVector3d_t offsetTranslation;
		double hmdScale;
		bool slamUpValid;
		vr::HmdVector3d_t slamUpInLighthouse;
		uint32_t tiltSteps;
		bool stayActive;
		bool stayHipFound;
		bool stayBodyReady;
		bool stayRescuing;
		uint32_t stayRecenters;
		uint32_t stayHeld;
		uint32_t stayRescues;
		double stayYawDeg;
		double stayShiftM;
		int32_t lockState;
		uint32_t lockBases;
		uint32_t lockHeldJumps;
		double lockLargestHeldM;
		double lockOffsetM;
		double lockOffsetDeg;
		double lockInconsistentM;
	};

	// Steam Frame link snapshot published to the overlay. See issue #18.
	struct FrameLinkSample
	{
		bool valid;              // false = no Frame data, calibrator falls back to OpenVR pose
		uint32_t deviceId;
		uint32_t flags;          // framelink::PoseFlag bitmask
		double confidence;       // 0..1
		double position[3];
		double orientation[4];   // wxyz
		uint64_t captureTicks;   // Frame clock
		double ageMs;            // PC-side age at the moment of this response
	};

	struct GetFrameLinkSample
	{
		uint32_t deviceId;       // Frame-local id, 0 = HMD
	};

	// Snapshot of the FrameLink UDP listener's state, for the overlay's stats tab.
	struct FrameLinkStatus
	{
		bool listening;                // true if the driver bound the UDP socket
		uint16_t port;                 // bound port
		bool hasPeer;                  // have we received from someone?
		char peer[64];                 // sender's ip:port (null-terminated)
		char product[32];              // last Hello.productId (null-terminated)
		uint64_t frameClockHz;         // last Hello.frameClockHz
		uint64_t packets;              // total packets accepted
		uint64_t droppedBadMagic;      // UDP junk / wrong protocol
		uint64_t droppedBadVersion;    // right magic, wrong version
		double lastPacketAgeMs;        // -1 if no packet ever
		double lastHelloAgeMs;         // -1 if no hello ever
		double lastPoseAgeMs;          // -1 if no pose ever
		double lastPoseConfidence;     // 0..1 for the most recent pose
		// Latest HMD pose from the Frame (deviceId=0). Zeroed when hasPose==false.
		bool hasPose;
		uint32_t poseFlags;
		double posePosition[3];
		double poseOrientation[4];     // wxyz
		double poseLinVelocity[3];
		double poseAngVelocity[3];
	};

	struct Request
	{
		RequestType type;

		union {
			SetDeviceTransform setDeviceTransform;
			SetHmdTracker setHmdTracker;
			SetSlamSync setSlamSync;
			SetOneEuro setOneEuro;
			SetUniverseLock setUniverseLock;
			GetFrameLinkSample getFrameLinkSample;
		};

		Request() : type(RequestInvalid) { }
		Request(RequestType type) : type(type) { }
	};

	struct Response
	{
		ResponseType type;

		union {
			Protocol protocol;
			DriverStatus status;
			FrameLinkSample frameLinkSample;
			FrameLinkStatus frameLinkStatus;
		};

		Response() : type(ResponseInvalid) { }
		Response(ResponseType type) : type(type) { }
	};
}