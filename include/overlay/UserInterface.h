// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <string>
#include <vector>

#include <openvr.h>

#include "TrackingPreview.h"

struct VRDevice
{
	int id = -1;
	vr::TrackedDeviceClass deviceClass;
	std::string model = "";
	std::string manufacturer = "";
	std::string serial = "";
	std::string trackingSystem = "";
	vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;

	// Human-readable name for display. Falls back cleanly when model data
	// isn't populated (bare "HMD", "Controller", etc.).
	std::string DisplayName() const
	{
		auto trim = [](std::string s) {
			size_t a = s.find_first_not_of(" \t"); size_t b = s.find_last_not_of(" \t");
			return (a == std::string::npos) ? std::string() : s.substr(a, b - a + 1);
		};
		std::string m = trim(model);
		std::string v = trim(manufacturer);
		if (!m.empty() && !v.empty())
		{
			// Avoid "Valve Valve Index" style duplication.
			if (m.size() >= v.size()
			    && std::equal(v.begin(), v.end(), m.begin(),
			                  [](char a, char b){ return std::tolower((unsigned char)a) == std::tolower((unsigned char)b); }))
				return m;
			return v + " " + m;
		}
		if (!m.empty()) return m;
		if (!v.empty()) return v;
		switch (deviceClass)
		{
		case vr::TrackedDeviceClass_HMD: return "HMD";
		case vr::TrackedDeviceClass_Controller: return "Controller";
		case vr::TrackedDeviceClass_GenericTracker: return "Tracker";
		default: return "Device";
		}
	}
};

struct VRState
{
	std::vector<std::string> trackingSystems;
	std::vector<VRDevice> devices;
};

class UserInterface
{
public:
	enum class WindowAction { None, Minimize, Close };
	static constexpr float DesignWidth = 1080.0f;
	static constexpr float DesignHeight = 700.0f;
	static constexpr float TitleBarHeight = 40.0f;
	static constexpr float TitleBarButtonWidth = 40.0f;
	static constexpr int TitleBarButtonCount = 2;
	static float sLangZoneMinX;
	static float sLangZoneMaxX;

	WindowAction Render(bool runningInOverlay);
	void SetClosing(bool closing) { closing_ = closing; }
	void SetOverlayInput(bool overlayInput) { overlayInput_ = overlayInput; }

private:
	enum class Tab { Calibration, Preview, Smoothing, Lighthouse, FrameLink, Settings };

	struct Status
	{
		const VRDevice* hmd = nullptr;
		const VRDevice* tracker = nullptr;
		std::string headline; 
		std::string detail;    
		unsigned color = 0;     
		bool ok = false;
	};

	void CollectDevices(VRState& state) const;
	Status BuildStatus(const VRState& state) const;

	WindowAction RenderTitleBar();
	void RenderTabs();
	void RenderCalibration(const Status& status);
	void RenderPreview();
	void RenderLighthouse();
	void RenderEdit(const Status& status);
	void RenderSmoothing();
	void RenderFrameLink(const Status& status);
	void RenderSettings();
	void RenderFooter();
	void RenderWizard();
	void RenderConfirm();
	void RenderClosingOverlay();

	Tab tab_ = Tab::Calibration;
	TrackingPreview preview_;
	bool closing_ = false;
	bool overlayInput_ = false;
	bool smoothingDirty_ = false;
	bool editView_ = false;
	double editStep_ = 0.1;
	bool confirmRemove_ = false;
	bool wizardOpen_ = false;
	float scrollTarget_ = 0.0f;
	float scrollApplied_ = 0.0f;
	int scrollTab_ = -1;
	float contentTop_ = 0.0f;
	float contentHeight_ = 0.0f;
};
