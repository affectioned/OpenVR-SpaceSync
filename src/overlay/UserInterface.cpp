// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "Lighthouse.h"
#include "Localization.h"
#include "Theme.h"
#include "Version.h"

#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <imgui.h>
#include <imgui_internal.h>
#include <SDL3/SDL.h>

using namespace ui;

float UserInterface::sLangZoneMinX = 0.0f;
float UserInterface::sLangZoneMaxX = 0.0f;

static const char* kCreditLine = "SpaceSync Beta by Shinyflvres. A modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). Thanks to tach/pushrax for SpaceCalibrator.";

namespace
{
	struct WizardStep { const char* title; Icon icon; };
	const WizardStep kSteps[] = {
		{ "Look left", Icon::ArrowLeft },
		{ "Look straight ahead", Icon::Dot },
		{ "Look right", Icon::ArrowRight },
		{ "Look straight ahead", Icon::Dot },
		{ "Look up", Icon::ArrowUp },
		{ "Look straight ahead", Icon::Dot },
		{ "Look down", Icon::ArrowDown },
		{ "Look straight ahead", Icon::Dot },
	};
	constexpr int kStepCount = (int)(sizeof(kSteps) / sizeof(kSteps[0]));

	// Last progress + last log line from the calibration.
	void CalibrationFeedback(float& fraction, std::string& lastLine)
	{
		fraction = 0.0f;
		lastLine.clear();
		for (auto it = CalCtx.messages.rbegin(); it != CalCtx.messages.rend(); ++it)
		{
			if (it->type == CalibrationContext::Message::Progress)
			{
				if (it->target > 0)
					fraction = (float)it->progress / (float)it->target;
				break;
			}
		}
		for (auto it = CalCtx.messages.rbegin(); it != CalCtx.messages.rend() && lastLine.empty(); ++it)
		{
			if (it->type != CalibrationContext::Message::String)
				continue;
			const std::string& s = it->str;
			size_t end = s.find_last_not_of("\r\n");
			if (end == std::string::npos)
				continue;
			size_t start = s.find_last_of("\r\n", end);
			lastLine = s.substr(start == std::string::npos ? 0 : start + 1, end - (start == std::string::npos ? 0 : start + 1) + 1);
		}
	}

	// Text segments on one line with mixed fonts/colours.
	struct Segment { ImFont* font; float size; unsigned color; std::string text; };
	void InlineText(const std::vector<Segment>& segments)
	{
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 p = ImGui::GetCursorScreenPos();
		float x = p.x;
		float maxH = 0.0f;
		std::vector<ImVec2> sizes;
		for (auto& s : segments)
		{
			ImVec2 sz = TextSize(s.font, s.size, s.text.c_str());
			sizes.push_back(sz);
			maxH = std::max(maxH, sz.y);
		}
		for (size_t i = 0; i < segments.size(); i++)
		{
			const Segment& s = segments[i];
			DrawText(dl, s.font, s.size, ImVec2(x, p.y + (maxH - sizes[i].y) * 0.5f), s.color, s.text.c_str());
			x += sizes[i].x;
		}
		ImGui::Dummy(ImVec2(x - p.x, maxH));
	}
}

// -----------------------------------------------------------------------------------------

void UserInterface::CollectDevices(VRState& state) const
{
	if (!vr::VRSystem())
		return;

	auto& trackingSystems = state.trackingSystems;
	char buffer[vr::k_unMaxPropertyStringSize];

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid || deviceClass == vr::TrackedDeviceClass_TrackingReference)
			continue;

		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);
		if (err != vr::TrackedProp_Success)
			continue;

		std::string system(buffer);
		auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), system);
		if (existing != trackingSystems.end())
		{
			if (deviceClass == vr::TrackedDeviceClass_HMD)
			{
				trackingSystems.erase(existing);
				trackingSystems.insert(trackingSystems.begin(), system);
			}
		}
		else
		{
			trackingSystems.push_back(system);
		}

		VRDevice device;
		device.id = id;
		device.deviceClass = deviceClass;
		device.trackingSystem = system;

		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
		device.model = std::string(buffer);

		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
		device.serial = std::string(buffer);

		device.controllerRole = (vr::ETrackedControllerRole)vr::VRSystem()->GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32, &err);
		state.devices.push_back(device);
	}
}

UserInterface::Status UserInterface::BuildStatus(const VRState& state) const
{
	Status s;
	for (auto& device : state.devices)
	{
		if (device.id == vr::k_unTrackedDeviceIndex_Hmd)
			s.hmd = &device;
		if (!CalCtx.trackerSerial.empty() && device.serial == CalCtx.trackerSerial)
			s.tracker = &device;
	}

	if (vr::VRSystem() && !DriverConnected())
	{
		s.headline = "SpaceSync driver not loaded";
		s.detail = "Restart SteamVR and enable the SpaceSync add-on";
		s.color = P.danger;
	}
	else if (!CalCtx.validProfile)
	{
		s.headline = "No calibration yet";
		s.detail = "Click the circle to calibrate";
		s.color = P.textMuted;
	}
	else if (!CalCtx.noHeadTracker && CalCtx.calibratedNoTracker)
	{
		s.headline = "Calibrated without a head tracker";
		s.detail = "Calibrate again with the tracker on your head";
		s.color = P.danger;
	}
	else if (!s.tracker && !CalCtx.noHeadTracker)
	{
		s.headline = "Headset tracker not connected";
		s.detail = CalCtx.trackerSerial;
		s.color = P.danger;
	}
	else if (CalCtx.noHeadTracker)
	{
		s.headline = "HMD Driven, one-time calibration";
		s.detail = "no running alignment";
		s.color = P.green;
		s.ok = true;
	}
	else if (!CalCtx.enabled)
	{
		s.headline = "Override disabled";
		s.detail = "HMD tracking system changed?";
		s.color = P.danger;
	}
	else if (CalCtx.followSlamHmd)
	{
		s.headline = "HMD Driven active";
		s.detail = s.tracker->serial;
		s.color = P.green;
		s.ok = true;
	}
	else
	{
		s.headline = "Lighthouse Driven active";
		s.detail = s.tracker->serial;
		s.color = P.green;
		s.ok = true;
	}
	return s;
}

// -----------------------------------------------------------------------------------------

// Content width in design px (minus the 24px padding).
static float AvailDesignWidth()
{
	return std::max(200.0f, (ImGui::GetIO().DisplaySize.x - 2.0f * px(24.0f)) / S());
}

static const float PageWidth = 900.0f;

static float PageInset()
{
	return std::max(0.0f, (AvailDesignWidth() - PageWidth) * 0.5f);
}

static float PageDesignWidth()
{
	return std::min(PageWidth, AvailDesignWidth());
}

UserInterface::WindowAction UserInterface::Render(bool runningInOverlay)
{
	ui::SetContentScale(CalCtx.uiScale);

	ImGuiIO& io = ImGui::GetIO();
	const float W = io.DisplaySize.x;
	const float H = io.DisplaySize.y;

	VRState state;
	CollectDevices(state);
	Status status = BuildStatus(state);

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(io.DisplaySize);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);

	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus |
		ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;

	WindowAction action = WindowAction::None;

	if (ImGui::Begin("SpaceSync", nullptr, flags))
	{
		ImDrawList* dl = ImGui::GetWindowDrawList();
		dl->AddRectFilled(ImVec2(0, 0), ImVec2(W, H), Col(P.card));

		float y = 0.0f;
		if (!runningInOverlay)
		{
			action = RenderTitleBar();
			y = px(TitleBarHeight);
		}

		ImGui::SetCursorPos(ImVec2(0.0f, y));
		RenderTabs();
		y = ImGui::GetCursorPosY();

		const float footerH = px(9.0f) + TextSize(F.regular, 11.5f, "X").y + px(9.0f);
		contentTop_ = y;
		contentHeight_ = H - footerH - y;

		ImGui::SetCursorPos(ImVec2(0.0f, y));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(24.0f), px(24.0f)));
		if (ImGui::BeginChild("content", ImVec2(W, contentHeight_), ImGuiChildFlags_AlwaysUseWindowPadding, ImGuiWindowFlags_NoScrollWithMouse))
		{
			const float current = ImGui::GetScrollY();

			if (scrollTab_ != (int)tab_ || std::abs(current - scrollApplied_) > 0.5f)
			{
				scrollTab_ = (int)tab_;
				scrollTarget_ = current;
			}

			const float maxScroll = ImGui::GetScrollMaxY();
			if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && io.MouseWheel != 0.0f)
				scrollTarget_ -= io.MouseWheel * px(110.0f);
			scrollTarget_ = std::min(maxScroll, std::max(0.0f, scrollTarget_));
			const float blend = 1.0f - std::exp(-io.DeltaTime / 0.055f);
			const float next = current + (scrollTarget_ - current) * blend;
			scrollApplied_ = std::abs(scrollTarget_ - next) < 0.5f ? scrollTarget_ : next;
			ImGui::SetScrollY(scrollApplied_);

			const float inset = px(PageInset());
			if (inset > 0.0f)
				ImGui::Indent(inset);

			switch (tab_)
			{
			case Tab::Calibration:
				if (editView_) RenderEdit(status); else RenderCalibration(status);
				break;
			case Tab::Preview:
				RenderPreview();
				break;
			case Tab::Smoothing:
				RenderSmoothing();
				break;
			case Tab::Lighthouse:
				RenderLighthouse();
				break;
			case Tab::FrameLink:
				RenderFrameLink(status);
				break;
			case Tab::Settings:
				RenderSettings();
				break;
			}

			if (inset > 0.0f)
				ImGui::Unindent(inset);
		}
		ImGui::EndChild();
		ImGui::PopStyleVar();

		ImGui::SetCursorPos(ImVec2(0.0f, H - footerH));
		RenderFooter();

		// outer border
		dl->AddRect(ImVec2(0.5f, 0.5f), ImVec2(W - 0.5f, H - 0.5f), Col(P.borderStrong));

		RenderWizard();
		RenderConfirm();
		RenderClosingOverlay();
	}
	ImGui::End();
	ImGui::PopStyleVar(3);

	if (closing_)
		action = WindowAction::None;
	return action;
}

// -----------------------------------------------------------------------------------------

UserInterface::WindowAction UserInterface::RenderTitleBar()
{
	const float W = ImGui::GetIO().DisplaySize.x;
	const float h = px(TitleBarHeight);
	ImDrawList* dl = ImGui::GetWindowDrawList();

	dl->AddRectFilled(ImVec2(0, 0), ImVec2(W, h), Col(P.titleBar));
	dl->AddRectFilled(ImVec2(0, h - 1.0f), ImVec2(W, h), Col(P.border));

	ImVec2 ts = TextSize(F.semibold, 12.5f, "SpaceSync");
	DrawText(dl, F.semibold, 12.5f, ImVec2(px(14.0f), (h - ts.y) * 0.5f), P.textTitle, "SpaceSync");

	{
		static const char* kLangNames[3] = { "English", "\xE6\x97\xA5\xE6\x9C\xAC\xE8\xAA\x9E", "\xE4\xB8\xAD\xE6\x96\x87" };
		float lx = px(14.0f) + ts.x + px(10.0f);
		ImVec2 ds = TextSize(F.regular, 12.0f, "-");
		DrawText(dl, F.regular, 12.0f, ImVec2(lx, (h - ds.y) * 0.5f), P.textFooter, "-");
		lx += ds.x + px(10.0f);
		sLangZoneMinX = lx - px(4.0f);
		for (int i = 0; i < 3; i++)
		{
			ImVec2 ns = TextSize(F.medium, 12.0f, kLangNames[i]);
			char id[16];
			std::snprintf(id, sizeof id, "##lang%d", i);
			ImGui::SetCursorPos(ImVec2(lx - px(3.0f), (h - ns.y) * 0.5f - px(4.0f)));
			ImGui::InvisibleButton(id, ImVec2(ns.x + px(6.0f), ns.y + px(8.0f)));
			bool langHovered = ImGui::IsItemHovered();
			if (langHovered)
				ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
			bool active = (int)loc::Current() == i;
			if (ImGui::IsItemClicked() && !active)
			{
				loc::SetLanguage((loc::Lang)i);
				CalCtx.language = i;
				SaveProfile(CalCtx);
			}
			DrawText(dl, active ? F.semibold : F.medium, 12.0f, ImVec2(lx, (h - ns.y) * 0.5f), active ? P.textStrong : (langHovered ? P.textTitle : P.textFooter), kLangNames[i]);
			lx += ns.x + px(8.0f);
			if (i < 2)
			{
				ImVec2 ss = TextSize(F.regular, 12.0f, "|");
				DrawText(dl, F.regular, 12.0f, ImVec2(lx, (h - ss.y) * 0.5f), P.textFooter, "|");
				lx += ss.x + px(8.0f);
			}
		}
		sLangZoneMaxX = lx;
	}

	WindowAction action = WindowAction::None;
	const float bw = px(TitleBarButtonWidth), bh = px(32.0f);
	float x = W - px(4.0f) - bw * TitleBarButtonCount;
	const float by = (h - bh) * 0.5f;

	struct Btn { const char* id; Icon icon; float size; unsigned hoverBg; unsigned hoverFg; WindowAction action; };
	const Btn buttons[TitleBarButtonCount] = {
		{ "##min", Icon::Minus, 12.0f, P.button, P.textStrong, WindowAction::Minimize },
		{ "##close", Icon::Cross, 10.0f, P.danger, 0xffffff, WindowAction::Close },
	};

	for (const Btn& b : buttons)
	{
		if (overlayInput_)
			break;
		ImGui::SetCursorPos(ImVec2(x, by));
		ImGui::InvisibleButton(b.id, ImVec2(bw, bh));
		bool hovered = ImGui::IsItemHovered();
		if (ImGui::IsItemClicked())
			action = b.action;
		if (hovered)
			dl->AddRectFilled(ImVec2(x, by), ImVec2(x + bw, by + bh), Col(b.hoverBg));
		DrawIcon(dl, b.icon, ImVec2(x + bw * 0.5f, by + bh * 0.5f), b.size, Col(hovered ? b.hoverFg : P.textIcon), 1.3f);
		x += bw;
	}
	return action;
}

void UserInterface::RenderTabs()
{
	const float W = ImGui::GetIO().DisplaySize.x;
	const float y0 = ImGui::GetCursorPosY();
	const float h = TextSize(F.semibold, 13.0f, "X").y + px(22.0f);
	ImDrawList* dl = ImGui::GetWindowDrawList();
	ImVec2 wp = ImGui::GetWindowPos();

	dl->AddRectFilled(ImVec2(wp.x, wp.y + y0 + h - 1.0f), ImVec2(wp.x + W, wp.y + y0 + h), Col(P.border));

	{
		const bool vrRunning = vr::VRSystem() != nullptr;
		const char* badgeText = vrRunning ? "SteamVR Running" : "SteamVR Not Running";
		ImVec2 ts = TextSize(F.medium, 11.5f, badgeText);
		const float dotD = px(7.0f);
		const float padX = px(11.0f), padY = px(6.0f), gap = px(7.0f);
		const float bw = padX * 2.0f + dotD + gap + ts.x;
		const float bh = ts.y + 2.0f * padY;
		float bx = wp.x + W - px(24.0f + PageInset()) - bw;
		float by = wp.y + y0 + (h - 1.0f - bh) * 0.5f;
		unsigned dotCol = vrRunning ? P.green : P.danger;
		dl->AddRectFilled(ImVec2(bx, by), ImVec2(bx + bw, by + bh), Col(P.button), px(4.0f));
		dl->AddRect(ImVec2(bx, by), ImVec2(bx + bw, by + bh), Col(P.borderButton), px(4.0f));
		dl->AddCircleFilled(ImVec2(bx + padX + dotD * 0.5f, by + bh * 0.5f), dotD * 0.5f, Col(dotCol), 16);
		DrawText(dl, F.medium, 11.5f, ImVec2(bx + padX + dotD + gap, by + padY), vrRunning ? P.textStrong : P.textMuted, badgeText);
	}

	ImGui::SetCursorPos(ImVec2(px(24.0f + PageInset()), y0));
	if (TabItem("Calibration", tab_ == Tab::Calibration)) tab_ = Tab::Calibration;
	ImGui::SameLine();
	if (TabItem("Tracking Preview", tab_ == Tab::Preview)) tab_ = Tab::Preview;
	ImGui::SameLine();
	if (TabItem("Smoothing", tab_ == Tab::Smoothing)) tab_ = Tab::Smoothing;
	ImGui::SameLine();
	if (TabItem("Basestations", tab_ == Tab::Lighthouse)) tab_ = Tab::Lighthouse;
	ImGui::SameLine();
	if (TabItem("Frame Link", tab_ == Tab::FrameLink)) tab_ = Tab::FrameLink;
	ImGui::SameLine();
	if (TabItem("Settings", tab_ == Tab::Settings)) tab_ = Tab::Settings;

	ImGui::SetCursorPos(ImVec2(0.0f, y0 + h));
	ImGui::Dummy(ImVec2(0.0f, 0.0f));
}

void UserInterface::RenderFooter()
{
	const float W = ImGui::GetIO().DisplaySize.x;
	const float y0 = ImGui::GetCursorPosY();
	ImVec2 wp = ImGui::GetWindowPos();
	const float H = ImGui::GetIO().DisplaySize.y;
	ImDrawList* dl = ImGui::GetWindowDrawList();

	dl->AddRectFilled(ImVec2(wp.x, wp.y + y0), ImVec2(wp.x + W, wp.y + H), Col(P.titleBar));
	dl->AddRectFilled(ImVec2(wp.x, wp.y + y0), ImVec2(wp.x + W, wp.y + y0 + 1.0f), Col(P.border));
	DrawText(dl, F.regular, 11.5f, ImVec2(wp.x + px(24.0f + PageInset()), wp.y + y0 + px(9.0f)), P.textFooter, kCreditLine);
	ImGui::Dummy(ImVec2(W, H - y0));
}

// -----------------------------------------------------------------------------------------

void UserInterface::RenderCalibration(const Status& status)
{
	ImDrawList* dl = ImGui::GetWindowDrawList();
	const bool calibrated = CalCtx.validProfile;
	const float W = ImGui::GetIO().DisplaySize.x;
	// 400px circle, shrunk if the content area is smaller.
	const float buttonH = TextSize(F.medium, 13.0f, "X").y + px(18.0f);
	const float maxD = contentHeight_ - 2.0f * px(24.0f) - px(16.0f + 28.0f + 16.0f) - buttonH;
	const float d = std::max(px(260.0f), std::min(px(400.0f), maxD));

	// Big circle
	ImVec2 origin = ImGui::GetCursorScreenPos();
	ImVec2 c(ImGui::GetWindowPos().x + W * 0.5f, origin.y + px(16.0f) + d * 0.5f);

	const bool vrReady = vr::VRSystem() != nullptr;
	ImGui::SetCursorScreenPos(ImVec2(c.x - d * 0.5f, c.y - d * 0.5f));
	ImGui::InvisibleButton("##circle", ImVec2(d, d));
	bool hovered = false;
	if (vrReady && ImGui::IsItemHovered())
	{
		ImVec2 m = ImGui::GetIO().MousePos;
		float dx = m.x - c.x, dy = m.y - c.y;
		hovered = dx * dx + dy * dy <= (d * 0.5f) * (d * 0.5f);
		if (hovered)
			ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
	}
	if (hovered && ImGui::IsItemClicked() && vrReady && CalCtx.state == CalibrationState::None && !wizardOpen_)
	{
		StartCalibration();
		wizardOpen_ = true;
	}

	const float ringWidth = px(3.0f);
	dl->AddCircleFilled(c, d * 0.5f - ringWidth * 0.5f, Col(hovered ? 0x131316 : P.inputBg), 128);
	DrawGradientRing(dl, c, d * 0.5f, ringWidth);

	// Text stack inside the circle
	const float lineH = px(18.0f);  // 12px text with line-height 1.5
	const float stackH = px(28 + 14 + 19 + 6 + 12.5f + 20 + 1 + 16) + 3.0f * lineH + px(12.0f) + 2.0f * lineH;
	float y = c.y - stackH * 0.5f;

	DrawIcon(dl, calibrated ? Icon::Check : Icon::Ring, ImVec2(c.x, y + px(14.0f)), 26.0f, Col(calibrated ? P.green : P.link), 2.2f);
	y += px(28.0f + 14.0f);
	DrawTextCentered(dl, F.semibold, 19.0f, ImVec2(c.x, y + px(9.5f)), P.textBright, calibrated ? "Calibrated" : "Calibrate");
	y += px(19.0f + 6.0f);
	DrawTextCentered(dl, F.regular, 12.5f, ImVec2(c.x, y + px(6.25f)), P.textMuted,
		!vrReady ? "Start SteamVR to calibrate" : (calibrated ? "Click to calibrate again" : "Click to start"));
	y += px(12.5f + 20.0f);
	dl->AddRectFilled(ImVec2(c.x - px(75.0f), y), ImVec2(c.x + px(75.0f), y + 1.0f), Col(P.border));
	y += 1.0f + px(16.0f);

	DrawTextCentered(dl, F.regular, 12.0f, ImVec2(c.x, y + lineH * 0.5f), P.textMuted, "HMD");
	y += lineH;
	DrawTextCentered(dl, F.mono, 12.0f, ImVec2(c.x, y + lineH * 0.5f), P.text, status.hmd ? status.hmd->serial.c_str() : "not detected");
	y += lineH;
	DrawTextCentered(dl, F.regular, 12.0f, ImVec2(c.x, y + lineH * 0.5f), P.textMuted, status.hmd ? status.hmd->trackingSystem.c_str() : "");
	y += lineH + px(12.0f);

	DrawTextCentered(dl, F.regular, 12.0f, ImVec2(c.x, y + lineH * 0.5f), status.color, status.headline.c_str());
	y += lineH;
	DrawTextCentered(dl, F.mono, 12.0f, ImVec2(c.x, y + lineH * 0.5f), status.color, status.detail.c_str());

	// Buttons below the circle
	ImGui::SetCursorScreenPos(ImVec2(c.x, c.y + d * 0.5f + px(28.0f)));
	ButtonOpts edit;
	edit.enabled = calibrated;
	ButtonOpts remove;
	remove.enabled = calibrated;
	remove.hoverBorder = P.dangerBorder;
	float editW = TextSize(F.medium, 13.0f, "Edit Calibration").x + px(36.0f);
	float removeW = TextSize(F.medium, 13.0f, "Remove Calibration").x + px(36.0f);
	float total = editW + px(8.0f) + removeW;
	ImGui::SetCursorScreenPos(ImVec2(c.x - total * 0.5f, ImGui::GetCursorScreenPos().y));
	if (Button("Edit Calibration", edit))
	{
		editView_ = true;
		CalCtx.state = CalibrationState::Editing;
	}
	ImGui::SameLine(0.0f, px(8.0f));
	if (Button("Remove Calibration", remove))
		confirmRemove_ = true;

	if (calibrated && CalCtx.driverStatus.mountShiftSuspected)
	{
		VSpace(18.0f);
		char warn[192];
		std::snprintf(warn, sizeof warn, loc::tr("The head tracker seems to have moved on the headset. The driver corrected %.1f deg / %.1f cm so far, a fresh calibration is the clean fix."),
			CalCtx.driverStatus.tiltDeg, CalCtx.driverStatus.translationDeviationM * 100.0);
		ImVec2 ws = TextSize(F.regular, 13.0f, warn);
		ImGui::SetCursorScreenPos(ImVec2(c.x - ws.x * 0.5f, ImGui::GetCursorScreenPos().y));
		Text(F.regular, 13.0f, P.yellow, warn);
	}

	VSpace(16.0f);
}

void UserInterface::RenderEdit(const Status& status)
{
	// Header lines
	{
		std::vector<Segment> line1 = {
			{ F.regular, 13.0f, P.textMuted, "HMD: " },
			{ F.mono, 12.5f, P.text, status.hmd ? status.hmd->serial : std::string("not detected") },
			{ F.regular, 13.0f, P.textMuted, status.hmd ? " (" + status.hmd->trackingSystem + ")" : std::string() },
		};
		InlineText(line1);
		VSpace(5.0f);
		std::vector<Segment> line2 = {
			{ F.regular, 13.0f, status.color, std::string(loc::tr(status.headline.c_str())) + (status.detail.empty() ? "" : loc::tr(" via ")) },
			{ F.mono, 12.5f, status.color, status.detail },
		};
		InlineText(line2);
		VSpace(22.0f);
	}

	const float maxW = std::min(760.0f, PageDesignWidth());
	const float x0 = ImGui::GetCursorPosX();

	// Back link + step size pills
	{
		float y0 = ImGui::GetCursorPosY();
		if (Link("Back"))
		{
			LoadProfile(CalCtx);
			CalCtx.state = CalibrationState::None;
			editView_ = false;
			return;
		}
		float rowH = ImGui::GetItemRectSize().y;

		const char* labels[3] = { "0.01", "0.1", "1.0" };
		const double steps[3] = { 0.01, 0.1, 1.0 };
		float pillW[3], totalW = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			pillW[i] = TextSize(F.mono, 12.0f, labels[i]).x + px(20.0f);
			totalW += pillW[i] + (i ? px(6.0f) : 0.0f);
		}
		float x = x0 + px(maxW) - totalW;
		float pillH = TextSize(F.mono, 12.0f, "0").y + px(10.0f);
		for (int i = 0; i < 3; i++)
		{
			ImGui::SetCursorPos(ImVec2(x, y0 + (rowH - pillH) * 0.5f));
			if (Pill(labels[i], editStep_ == steps[i]))
				editStep_ = steps[i];
			x += pillW[i] + px(6.0f);
		}
		ImGui::SetCursorPos(ImVec2(x0, y0 + std::max(rowH, pillH)));
		VSpace(18.0f);
	}

	struct Field { const char* label; double* value; double stepMul; bool worksInHmdDriven; };
	Field rows[3][3] = {
		{ { "Yaw", &CalCtx.calibratedRotation(1), 1.0, false }, { "Pitch", &CalCtx.calibratedRotation(2), 1.0, true }, { "Roll", &CalCtx.calibratedRotation(0), 1.0, true } },
		{ { "X", &CalCtx.calibratedTranslation(0), 1.0, false }, { "Y", &CalCtx.calibratedTranslation(1), 1.0, false }, { "Z", &CalCtx.calibratedTranslation(2), 1.0, false } },
		{ { "Scale", &CalCtx.calibratedScale, 0.01, false }, { "HMD Scale", &CalCtx.hmdScale, 0.01, true }, { nullptr, nullptr, 0.0, false } },
	};

	const bool runtimeAlignment = CalCtx.followSlamHmd && !CalCtx.noHeadTracker;
	if (runtimeAlignment)
	{
		TextWrapped(F.regular, 12.0f, P.textDim, maxW,
			"Greyed out fields do nothing in HMD Driven mode: the runtime alignment measures yaw and position against your head every frame and undoes those edits within a few seconds. Switch to Lighthouse Driven to use them.");
		VSpace(14.0f);
	}

	const Eigen::Vector3d rotationBefore = CalCtx.calibratedRotation;
	const float colW = (maxW - 2.0f * 14.0f) / 3.0f;
	for (int r = 0; r < 3; r++)
	{
		float yRow = ImGui::GetCursorPosY();
		float labelH = TextSize(F.regular, 12.5f, "X").y;
		for (int col = 0; col < 3; col++)
		{
			const Field& f = rows[r][col];
			if (!f.label)
				continue;
			const bool live = f.worksInHmdDriven || !runtimeAlignment;
			float x = x0 + col * px(colW + 14.0f);
			ImGui::SetCursorPos(ImVec2(x, yRow));
			Text(F.regular, 12.5f, live ? P.textMuted : P.textDisabled, f.label);
			ImGui::SetCursorPos(ImVec2(x, yRow + labelH + px(6.0f)));
			char id[32];
			std::snprintf(id, sizeof id, "##field%d%d", r, col);
			Stepper(id, f.value, editStep_ * f.stepMul, 8, colW, live);
		}
		ImGui::SetCursorPos(ImVec2(x0, yRow + labelH + px(6.0f + 34.0f)));
		ImGui::Dummy(ImVec2(0.0f, 0.0f));
		if (r < 2)
			VSpace(18.0f);
	}
	if (CalCtx.calibratedRotation != rotationBefore)
		KeepCalibrationSpotAfterRotationEdit(rotationBefore);

	VSpace(20.0f);
	ButtonOpts save;
	save.kind = ButtonKind::Primary;
	save.width = maxW;
	save.padY = 10.0f;
	ImGui::SetCursorPosX(x0);
	if (Button("Save Profile", save))
	{
		SaveProfile(CalCtx);
		CalCtx.state = CalibrationState::None;
		editView_ = false;
	}
	VSpace(8.0f);
}

// -----------------------------------------------------------------------------------------

namespace
{
	// label | track | value, optional hint below
	bool SliderRow(const char* label, const char* hint, double* value, double minV, double maxV, const char* fmt, float totalW, float labelW, float valueW)
	{
		const float x0 = ImGui::GetCursorPosX();
		const float y0 = ImGui::GetCursorPosY();
		const float rowH = px(20.0f) + 2.0f * px(7.0f);
		const float sliderW = totalW - labelW - 16.0f - 16.0f - valueW;

		float th = TextSize(F.regular, 13.0f, label).y;
		ImGui::SetCursorPos(ImVec2(x0, y0 + (rowH - th) * 0.5f));
		Text(F.regular, 13.0f, P.text, label);

		ImGui::SetCursorPos(ImVec2(x0 + px(labelW + 16.0f), y0 + px(7.0f)));
		char id[64];
		std::snprintf(id, sizeof id, "##slider_%s_%p", label, (void*)value);
		bool changed = Slider(id, value, minV, maxV, sliderW);

		char buf[48];
		std::snprintf(buf, sizeof buf, fmt, *value);
		ImVec2 vs = TextSize(F.mono, 12.5f, buf);
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 wp = ImGui::GetWindowPos();
		float sx = ImGui::GetScrollX(), sy = ImGui::GetScrollY();
		DrawText(dl, F.mono, 12.5f, ImVec2(wp.x - sx + x0 + px(totalW) - vs.x, wp.y - sy + y0 + (rowH - vs.y) * 0.5f), P.textStrong, buf);

		ImGui::SetCursorPos(ImVec2(x0, y0 + rowH));
		ImGui::Dummy(ImVec2(0.0f, 0.0f));
		if (hint && *hint)
		{
			ImGui::SetCursorPosX(x0 + px(labelW + 16.0f));
			Hint(hint, totalW - labelW - 16.0f);
			VSpace(6.0f);
		}
		return changed;
	}

	void ParamSliders(protocol::OneEuroParams& p, bool& changed, float totalW)
	{
		changed |= SliderRow("minCutoff", "How steady things look when you are not moving. Lower = calmer image, higher = more responsive (drag right if things feel laggy or floaty).", &p.minCutoff, 0.1, 10.0, "%.3f Hz", totalW, 90.0f, 76.0f);
		changed |= SliderRow("beta", "How quickly the smoothing keeps up when you move fast. Drag right if fast movements feel delayed; drag left if they look shaky.", &p.beta, 0.0, 2.0, "%.3f", totalW, 90.0f, 76.0f);
		changed |= SliderRow("dCutoff", "Fine-tunes how the smoothing reacts as your movement speed changes. Most people can leave this alone.", &p.dCutoff, 0.1, 4.0, "%.3f Hz", totalW, 90.0f, 76.0f);
	}
}

void UserInterface::RenderPreview()
{
	ImVec2 avail = ImGui::GetContentRegionAvail();
	float h = std::max(px(220.0f), avail.y);
	preview_.Render(ImVec2(avail.x, h));
}

static ImTextureData* LoadImageTexture(const char* file)
{
	const char* base = SDL_GetBasePath();
	std::string path = std::string(base ? base : "") + "images\\" + file;
	SDL_Surface* loaded = SDL_LoadPNG(path.c_str());
	if (!loaded)
		return nullptr;
	SDL_Surface* rgba = SDL_ConvertSurface(loaded, SDL_PIXELFORMAT_RGBA32);
	SDL_DestroySurface(loaded);
	if (!rgba)
		return nullptr;

	const int target = 160;
	while (rgba->h / 2 >= target)
	{
		SDL_Surface* half = SDL_ScaleSurface(rgba, rgba->w / 2, rgba->h / 2, SDL_SCALEMODE_LINEAR);
		SDL_DestroySurface(rgba);
		if (!half)
			return nullptr;
		rgba = half;
	}
	if (rgba->h != target)
	{
		int w = (int)std::lround((double)rgba->w * target / rgba->h);
		SDL_Surface* sized = SDL_ScaleSurface(rgba, w, target, SDL_SCALEMODE_LINEAR);
		SDL_DestroySurface(rgba);
		if (!sized)
			return nullptr;
		rgba = sized;
	}

	ImTextureData* tex = IM_NEW(ImTextureData)();
	tex->Create(ImTextureFormat_RGBA32, rgba->w, rgba->h);
	for (int y = 0; y < rgba->h; y++)
		std::memcpy(tex->GetPixelsAt(0, y), (const unsigned char*)rgba->pixels + (size_t)y * rgba->pitch, (size_t)rgba->w * 4);
	SDL_DestroySurface(rgba);
	ImGui::RegisterUserTexture(tex);
	return tex;
}

struct StationImages
{
	ImTextureData* body = nullptr;
	ImTextureData* on = nullptr;
	ImTextureData* standby = nullptr;
};

static const StationImages& GetStationImages()
{
	static bool loaded = false;
	static StationImages images;
	if (!loaded)
	{
		loaded = true;
		images.body = LoadImageTexture("Basestation 2.0.png");
		images.on = LoadImageTexture("Basestation 2.0 On Layer.png");
		images.standby = LoadImageTexture("Basestation 2.0 Standby Layer.png");
	}
	return images;
}

void UserInterface::RenderLighthouse()
{
	const float maxW = std::min(820.0f, PageDesignWidth());

	TextWrapped(F.regular, 13.0f, P.textMuted, maxW,
		"Turn your basestations on, into standby, or to sleep without a Lighthouse headset. "
		"Works with V2 basestations over Bluetooth LE.");
	VSpace(18.0f);

	if (CheckboxRow("Basestation Management",
		"SpaceSync connects to your basestations over Bluetooth LE to show and switch their power state. Turn this off if another tool (for example Oyasumi) already controls your basestations, or if the Bluetooth scanning interferes with other BLE devices such as a heart rate monitor used by HRtoOSC.",
		&CalCtx.basestationManagement, maxW))
	{
		lighthouse::SetManaged(CalCtx.basestationManagement);
		if (!CalCtx.basestationManagement)
			lighthouse::SetAutoWake(false);
		SaveProfile(CalCtx);
	}

	if (!CalCtx.basestationManagement)
	{
		VSpace(12.0f);
		TextWrapped(F.regular, 12.5f, P.textDim, maxW, "Basestation management is off. SpaceSync does not scan for or connect to any Bluetooth LE devices, so it will not conflict with tools like Oyasumi or HRtoOSC.");
		return;
	}

	lighthouse::EnsureScanning();
	if (!lighthouse::Available())
	{
		VSpace(12.0f);
		Text(F.regular, 13.0f, P.yellow, "Bluetooth LE is not available on this PC.");
		VSpace(8.0f);
		TextWrapped(F.regular, 12.5f, P.textDim, maxW, "A Bluetooth 4.0+ adapter is required to control basestations.");
		return;
	}

	if (CheckboxRow("Dynamic Power",
		"When enabled, SpaceSync wakes up all basestations as soon as it runs. When SpaceSync gets closed, it puts all basestations into standby or sleep (see Dynamic Power Mode).",
		&CalCtx.dynamicBasestationPower, maxW))
	{
		lighthouse::SetAutoWake(CalCtx.dynamicBasestationPower);
		if (CalCtx.dynamicBasestationPower)
			lighthouse::RequestPowerAll(lighthouse::Power::Awake);
		SaveProfile(CalCtx);
	}
	if (CalCtx.dynamicBasestationPower)
	{
		static const char* const powerModes[] = { "Standby", "Sleep" };
		if (DropdownRow("Dynamic Power Mode",
			"What the basestations do when SpaceSync closes. Standby wakes up faster, sleep uses less power.",
			&CalCtx.dynamicPowerMode, powerModes, 2, maxW))
			SaveProfile(CalCtx);
	}
	VSpace(18.0f);

	auto stations = lighthouse::Stations();

	SectionHeader("Basestations", maxW);
	VSpace(10.0f);

	if (stations.empty())
	{
		Text(F.regular, 12.5f, P.textDim, "Scanning for basestations...");
		VSpace(8.0f);
		TextWrapped(F.regular, 12.5f, P.textDim, maxW, "Make sure the stations have power and are within Bluetooth range.");
		return;
	}

	char buf[64];
	const StationImages& images = GetStationImages();
	ImTextureData* image = images.body;
	for (const auto& s : stations)
	{
		ImGui::PushID((void*)(uintptr_t)s.address);

		const ImVec2 rowStart = ImGui::GetCursorPos();
		const float imageH = px(62.0f);
		if (image)
		{
			const float imageW = imageH * (float)image->Width / (float)image->Height;
			ImGui::Image(image->GetTexRef(), ImVec2(imageW, imageH));
			ImTextureData* led = s.state == lighthouse::Power::Awake ? images.on : (s.state == lighthouse::Power::Standby ? images.standby : nullptr);
			if (led)
				ImGui::GetWindowDrawList()->AddImage(led->GetTexRef(), ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
			ImGui::SetCursorPos(ImVec2(rowStart.x + imageW + px(18.0f), rowStart.y));
		}
		ImGui::BeginGroup();

		Text(F.semibold, 13.5f, P.textStrong, s.name.c_str());
		ImGui::SameLine(0.0f, px(14.0f));
		const char* stateText = "Unknown";
		unsigned stateColor = P.textDim;
		switch (s.state)
		{
		case lighthouse::Power::Awake: stateText = "Awake"; stateColor = P.green; break;
		case lighthouse::Power::Standby: stateText = "Standby"; stateColor = P.yellow; break;
		case lighthouse::Power::Sleep: stateText = "Sleeping"; stateColor = P.textMuted; break;
		default: break;
		}
		Text(F.medium, 12.0f, stateColor, stateText);
		ImGui::SameLine(0.0f, px(14.0f));
		std::snprintf(buf, sizeof buf, "%d dBm", s.rssi);
		Text(F.regular, 11.5f, P.textFooter, buf);
		VSpace(10.0f);

		ButtonOpts small;
		small.fontSize = 12.5f;
		small.padX = 16.0f;
		small.padY = 7.0f;
		small.enabled = !s.busy;

		ButtonOpts primary = small;
		primary.kind = ButtonKind::Primary;

		if (Button("Wake", small))
			lighthouse::RequestPower(s.address, lighthouse::Power::Awake);
		ImGui::SameLine(0.0f, px(10.0f));
		if (Button("Standby", small))
			lighthouse::RequestPower(s.address, lighthouse::Power::Standby);
		ImGui::SameLine(0.0f, px(10.0f));
		if (Button("Sleep", small))
			lighthouse::RequestPower(s.address, lighthouse::Power::Sleep);
		ImGui::SameLine(0.0f, px(18.0f));
		if (Button("Refresh", primary))
			lighthouse::RequestRefresh(s.address);
		if (s.busy)
		{
			ImGui::SameLine(0.0f, px(12.0f));
			Text(F.regular, 12.0f, P.textDim, "working...");
		}
		if (!s.error.empty())
		{
			VSpace(6.0f);
			Text(F.regular, 12.0f, P.danger, s.error.c_str());
		}

		ImGui::EndGroup();
		if (image)
		{
			float missing = rowStart.y + imageH - ImGui::GetCursorPosY();
			if (missing > 0.0f)
				ImGui::Dummy(ImVec2(0.0f, missing));
		}

		ImGui::PopID();

		VSpace(16.0f);
		HLine(maxW);
		VSpace(16.0f);
	}

	SectionHeader("All Basestations", maxW);
	VSpace(10.0f);
	{
		if (Button("Wake all"))
			lighthouse::RequestPowerAll(lighthouse::Power::Awake);
		ImGui::SameLine(0.0f, px(10.0f));
		if (Button("Standby all"))
			lighthouse::RequestPowerAll(lighthouse::Power::Standby);
		ImGui::SameLine(0.0f, px(10.0f));
		if (Button("Sleep all"))
			lighthouse::RequestPowerAll(lighthouse::Power::Sleep);
	}
	VSpace(10.0f);
	TextWrapped(F.regular, 12.0f, P.textDim, maxW,
		"Sleeping or standby stations stop tracking immediately. Standby wakes up faster than sleep; older station firmware only supports sleep.");
}

void UserInterface::RenderClosingOverlay()
{
	if (!closing_)
		return;

	const float W = ImGui::GetIO().DisplaySize.x;
	const float H = ImGui::GetIO().DisplaySize.y;
	ImDrawList* dl = ImGui::GetForegroundDrawList();

	dl->AddRectFilled(ImVec2(0, 0), ImVec2(W, H), Col(P.overlay, 0.97f));

	std::string station;
	for (const auto& s : lighthouse::Stations())
	{
		if (s.busy)
		{
			station = s.name;
			break;
		}
	}
	const bool sleep = CalCtx.dynamicPowerMode == 1;
	char line[256];
	if (!station.empty())
		std::snprintf(line, sizeof line, loc::tr(sleep ? "Setting basestation \"%s\" to sleep..." : "Setting basestation \"%s\" to standby..."), station.c_str());
	else
		std::snprintf(line, sizeof line, "%s", loc::tr(sleep ? "Setting basestations to sleep..." : "Setting basestations to standby..."));

	const float cx = W * 0.5f;
	const float cy = H * 0.5f;
	const float r = px(16.0f);
	const float spinnerY = cy - px(36.0f);
	float a0 = (float)(ImGui::GetTime() * 5.0);
	const int segments = 24;
	dl->PathClear();
	for (int i = 0; i <= segments; i++)
	{
		float a = a0 + (float)i / segments * 4.7f;
		dl->PathLineTo(ImVec2(cx + std::cos(a) * r, spinnerY + std::sin(a) * r));
	}
	dl->PathStroke(Col(P.link), 0, px(3.0f));

	DrawTextCentered(dl, F.semibold, 15.0f, ImVec2(cx, cy + px(6.0f)), P.textBright, line);
	DrawTextCentered(dl, F.regular, 12.0f, ImVec2(cx, cy + px(30.0f)), P.textMuted,
		sleep ? "The window closes when all basestations are asleep." : "The window closes when all basestations are in standby.");
}

void UserInterface::RenderSmoothing()
{
	const float maxW = std::min(820.0f, PageDesignWidth());
	bool changed = false;

	Text(F.regular, 13.0f, P.green, "NOTE: Changes here take effect instantly, no need to re-calibrate.");
	VSpace(18.0f);

	SectionHeader("Lighthouse Trackers & Controllers", maxW);
	VSpace(10.0f);
	TextWrapped(F.regular, 12.5f, P.textMuted, maxW,
		"Smooths all lighthouse devices (Vive/Tundra trackers, Index controllers) so they show less jittery movement, "
		"for example for dancing or full body recordings. Recommended: 25% - smooth movement while keeping latency minimal. "
		"The higher the percentage, the more latency you get on fast movement. 0% turns it off.");
	VSpace(12.0f);
	changed |= SliderRow("Smoothing", "", &CalCtx.lighthouseSmoothing, 0.0, 100.0, "%.0f %%", maxW, 90.0f, 76.0f);
	VSpace(24.0f);

	SectionHeader("Latency Compensation", maxW);
	VSpace(10.0f);
	TextWrapped(F.regular, 12.5f, P.textMuted, maxW,
		"Lighthouse devices reach the headset about 40-60 ms late. SpaceSync predicts where they are right now and learns your movement while you play. "
		"100% removes the delay, 0% turns the prediction off. Lower it if devices overshoot when you stop quickly.");
	VSpace(12.0f);
	changed |= SliderRow("Strength", "", &CalCtx.latencyCompensation, 0.0, 100.0, "%.0f %%", maxW, 90.0f, 76.0f);
	if (CalCtx.noHeadTracker && CalCtx.validProfile && !CalCtx.calibrationLatencyValid)
	{
		VSpace(8.0f);
		TextWrapped(F.regular, 12.5f, P.textDim, maxW, "Not active yet without a head tracker. Calibrate once so SpaceSync can measure the delay.");
	}
	VSpace(24.0f);

	SectionHeader("Headset Tracker", maxW);
	VSpace(10.0f);
	changed |= CheckboxRow("Smooth headset tracker",
		"Steadies what you see through the headset to reduce shaking (HMD Driven mode only). Adds a tiny bit of delay - if the view feels laggy when you move quickly, adjust the sliders below.",
		&CalCtx.headFilterEnabled, maxW);
	VSpace(10.0f);
	ParamSliders(CalCtx.headFilterParams, changed, maxW);

	if (changed)
	{
		SendOneEuroParams();
		smoothingDirty_ = true;
	}
	if (smoothingDirty_ && !ImGui::IsAnyMouseDown())
	{
		SaveProfile(CalCtx);
		smoothingDirty_ = false;
	}
}

void UserInterface::RenderFrameLink(const Status& status)
{
	const float maxW = std::min(820.0f, PageDesignWidth());
	const auto& s = CalCtx.frameLinkStatus;

	TextWrapped(F.regular, 13.0f, P.textMuted, maxW,
		"Live status of the UDP listener that receives data from a Steam Frame service "
		"(issue #18). The Frame-side sender ships raw HMD poses + confidence to the driver, "
		"which the calibration pipeline can substitute in place of OpenVR's predicted pose.");
	VSpace(18.0f);

	// -------- Playspace merge: which HMDs are we lining up? ------------
	SectionHeader("Playspace merge", maxW);
	VSpace(6.0f);
	TextWrapped(F.regular, 12.0f, P.textMuted, maxW,
		"Calibration is what makes these two tracking spaces line up. The local HMD defines the "
		"lighthouse/stage reference; the Frame delivers its own tracked pose. Both must be present "
		"for SpaceSync to solve the transform between them.");
	VSpace(8.0f);
	{
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 wp = ImGui::GetWindowPos();

		const float cardW = (px(maxW) - px(32.0f)) * 0.5f;
		const float cardH = px(96.0f);

		auto hmdCard = [&](const char* role, const char* model, const char* serial, bool connected) {
			ImVec2 p0 = ImGui::GetCursorScreenPos();
			ImVec2 p1(p0.x + cardW, p0.y + cardH);
			unsigned border = connected ? P.borderStrong : P.border;
			dl->AddRectFilled(p0, p1, Col(P.card), px(6.0f));
			dl->AddRect(p0, p1, Col(border), px(6.0f), 0, px(1.0f));

			// A simple HMD silhouette drawn with primitives — one rounded body,
			// two eyes, a strap tab. No asset needed. Tints dim when disconnected.
			unsigned glyph = connected ? P.textStrong : P.textDim;
			float gx = p0.x + px(18.0f) + px(20.0f);
			float gy = p0.y + px(16.0f) + px(16.0f);
			ImVec2 body0(gx - px(22.0f), gy - px(12.0f));
			ImVec2 body1(gx + px(22.0f), gy + px(12.0f));
			dl->AddRectFilled(body0, body1, Col(glyph, 0.18f), px(4.0f));
			dl->AddRect(body0, body1, Col(glyph), px(4.0f), 0, px(1.2f));
			dl->AddCircleFilled(ImVec2(gx - px(10.0f), gy), px(3.0f), Col(glyph));
			dl->AddCircleFilled(ImVec2(gx + px(10.0f), gy), px(3.0f), Col(glyph));
			dl->AddLine(ImVec2(body0.x - px(6.0f), body0.y + px(6.0f)),
				ImVec2(body0.x, body0.y + px(6.0f)), Col(glyph), px(1.5f));
			dl->AddLine(ImVec2(body1.x, body0.y + px(6.0f)),
				ImVec2(body1.x + px(6.0f), body0.y + px(6.0f)), Col(glyph), px(1.5f));

			// Text block to the right of the glyph.
			float tx = gx + px(36.0f);
			DrawText(dl, F.regular, 10.5f, ImVec2(tx, p0.y + px(14.0f)),
				connected ? P.textMuted : P.textDim, role);
			DrawText(dl, F.semibold, 14.0f, ImVec2(tx, p0.y + px(28.0f)),
				connected ? P.textBright : P.textMuted, model && *model ? model : "(disconnected)");
			DrawText(dl, F.mono, 11.0f, ImVec2(tx, p0.y + px(52.0f)),
				connected ? P.textDim : P.textDim, serial && *serial ? serial : "-");

			// Tiny status dot in the corner.
			unsigned dotCol = connected ? P.green : P.danger;
			dl->AddCircleFilled(ImVec2(p1.x - px(14.0f), p0.y + px(14.0f)), px(4.0f), Col(dotCol));

			ImGui::Dummy(ImVec2(cardW, cardH));
		};

		// Local HMD (whatever SteamVR on this PC is currently rendering for).
		const char* localModel = status.hmd ? status.hmd->model.c_str() : "";
		const char* localSerial = status.hmd ? status.hmd->serial.c_str() : "";
		hmdCard("Local HMD", localModel, localSerial, status.hmd != nullptr);

		ImGui::SameLine(0.0f, px(16.0f));

		// Frame HMD identified by the Hello's productId.
		char remoteModel[48];
		if (s.hasPeer && s.product[0])
			std::snprintf(remoteModel, sizeof remoteModel, "%s", s.product);
		else
			std::snprintf(remoteModel, sizeof remoteModel, "%s", "(no Frame)");
		hmdCard("Frame HMD (via FrameLink)", remoteModel, s.hasPeer ? s.peer : "-", s.hasPeer);

		VSpace(8.0f);
		TextWrapped(F.regular, 12.0f, P.textDim, maxW,
			"Calibration aligns the local HMD's tracking space to the Frame's "
			"SLAM space using the head-tracker samples. Both must be present for the "
			"substitution to actually feed poses into the solve.");
	}
	VSpace(18.0f);

	// ---- Connection card -----------------------------------------------
	SectionHeader("Connection", maxW);
	VSpace(6.0f);
	TextWrapped(F.regular, 12.0f, P.textMuted, maxW,
		"The driver's UDP socket for Frame data. Without a peer talking to the listener there's "
		"nothing to substitute into calibration — the solve would fall back to OpenVR's own HMD "
		"pose, which is the current baseline.");
	VSpace(8.0f);

	auto kv = [&](const char* label, const std::string& value, unsigned valueColor) {
		Text(F.regular, 13.0f, P.textMuted, label);
		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(140.0f) - ImGui::CalcTextSize(label).x - px(8.0f));
		Text(F.mono, 12.5f, valueColor, value.c_str());
	};

	const bool listening = s.listening;
	const bool haveRecent = s.lastPacketAgeMs >= 0.0 && s.lastPacketAgeMs < 3000.0;

	char line[160];
	std::snprintf(line, sizeof line, "UDP :%u  %s",
		(unsigned)s.port, listening ? "listening" : "NOT bound");
	kv("Listener", line, listening ? P.green : P.danger);

	kv("Peer", s.hasPeer ? std::string(s.peer) : std::string("(none yet)"),
		s.hasPeer ? P.textStrong : P.textDim);
	kv("Product", s.hasPeer ? std::string(s.product) : std::string("-"),
		s.hasPeer ? P.textStrong : P.textDim);

	if (s.frameClockHz > 0) {
		double mhz = (double)s.frameClockHz / 1e6;
		std::snprintf(line, sizeof line, "%.3f MHz", mhz);
		kv("Frame clock", line, P.textStrong);
	} else {
		kv("Frame clock", "-", P.textDim);
	}

	VSpace(14.0f);

	// ---- Activity ------------------------------------------------------
	SectionHeader("Activity", maxW);
	VSpace(6.0f);
	TextWrapped(F.regular, 12.0f, P.textMuted, maxW,
		"Freshness of the data feeding the calibrator. Samples older than 500 ms are treated as "
		"stale and ignored. Low pose confidence is treated the same way — samples below the "
		"threshold below get rejected from the solve.");
	VSpace(8.0f);

	auto ageText = [&](double ageMs, char* buf, size_t cap) {
		if (ageMs < 0.0) { std::snprintf(buf, cap, "never"); return; }
		if (ageMs < 2000.0) { std::snprintf(buf, cap, "%.0f ms ago", ageMs); return; }
		if (ageMs < 120000.0) { std::snprintf(buf, cap, "%.1f s ago", ageMs / 1000.0); return; }
		std::snprintf(buf, cap, "%.1f min ago", ageMs / 60000.0);
	};
	auto ageColor = [&](double ageMs, double freshMs) {
		if (ageMs < 0.0) return P.textDim;
		if (ageMs < freshMs) return P.green;
		if (ageMs < freshMs * 4.0) return P.yellow;
		return P.danger;
	};

	char ageBuf[64];

	ageText(s.lastPacketAgeMs, ageBuf, sizeof ageBuf);
	kv("Last packet", ageBuf, ageColor(s.lastPacketAgeMs, 500.0));

	ageText(s.lastHelloAgeMs, ageBuf, sizeof ageBuf);
	kv("Last Hello", ageBuf, ageColor(s.lastHelloAgeMs, 2000.0));

	ageText(s.lastPoseAgeMs, ageBuf, sizeof ageBuf);
	kv("Last pose", ageBuf, ageColor(s.lastPoseAgeMs, 500.0));

	std::snprintf(line, sizeof line, "%.2f", s.lastPoseConfidence);
	kv("Pose confidence", line, s.lastPoseConfidence >= 0.5 ? P.green :
	                            s.lastPoseConfidence > 0.0 ? P.yellow : P.textDim);

	VSpace(14.0f);

	// ---- Live pose from the Frame -------------------------------------
	SectionHeader("Live HMD pose from Frame", maxW);
	VSpace(6.0f);
	TextWrapped(F.regular, 12.0f, P.textMuted, maxW,
		"The exact values the calibrator will use when it swaps in a Frame sample. Position / "
		"orientation feed the solve directly; velocity is passed through to SteamVR so prediction "
		"stays sane. If the flags show 'inertial', the Frame is coasting on IMU and the sample is "
		"dropped by the solve.");
	VSpace(8.0f);
	if (s.hasPose)
	{
		std::snprintf(line, sizeof line, "%+6.3f %+6.3f %+6.3f m",
			s.posePosition[0], s.posePosition[1], s.posePosition[2]);
		kv("Position xyz", line, P.textStrong);

		// Convert wxyz -> yaw/pitch/roll (degrees, SpaceSync Y-up convention).
		double qw = s.poseOrientation[0], qx = s.poseOrientation[1];
		double qy = s.poseOrientation[2], qz = s.poseOrientation[3];
		double sinp = 2.0 * (qw * qx - qy * qz);
		double pitch = sinp >=  1.0 ?  3.14159265358979 * 0.5 :
		               sinp <= -1.0 ? -3.14159265358979 * 0.5 : std::asin(sinp);
		double yaw   = std::atan2(2.0 * (qw * qy + qz * qx), 1.0 - 2.0 * (qx * qx + qy * qy));
		double roll  = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qx * qx + qz * qz));
		const double rad2deg = 180.0 / 3.14159265358979;
		std::snprintf(line, sizeof line, "yaw %+6.1f  pitch %+6.1f  roll %+6.1f deg",
			yaw * rad2deg, pitch * rad2deg, roll * rad2deg);
		kv("Orientation", line, P.textStrong);

		double lin = std::sqrt(s.poseLinVelocity[0] * s.poseLinVelocity[0]
		                      + s.poseLinVelocity[1] * s.poseLinVelocity[1]
		                      + s.poseLinVelocity[2] * s.poseLinVelocity[2]);
		std::snprintf(line, sizeof line, "%.3f m/s", lin);
		kv("Linear speed", line, lin > 0.05 ? P.green : P.textDim);

		double ang = std::sqrt(s.poseAngVelocity[0] * s.poseAngVelocity[0]
		                      + s.poseAngVelocity[1] * s.poseAngVelocity[1]
		                      + s.poseAngVelocity[2] * s.poseAngVelocity[2]);
		std::snprintf(line, sizeof line, "%.3f rad/s", ang);
		kv("Angular speed", line, ang > 0.05 ? P.green : P.textDim);

		// Flag breakdown so the user sees when tracking drops to inertial-only.
		char flagBuf[48] = {};
		if (s.poseFlags & 0x1)  std::strcat(flagBuf, "pos ");
		if (s.poseFlags & 0x2)  std::strcat(flagBuf, "rot ");
		if (s.poseFlags & 0x4)  std::strcat(flagBuf, "inertial ");
		if (!flagBuf[0])        std::strcpy(flagBuf, "(none)");
		kv("Pose flags", flagBuf, (s.poseFlags & 0x4) ? P.yellow : P.textStrong);
	}
	else
	{
		TextWrapped(F.regular, 12.5f, P.textDim, maxW,
			"No fresh pose from the Frame. If the service is running, the headset is "
			"probably not currently tracked (out of range, taken off, or Fallback_RotationOnly).");
	}

	VSpace(14.0f);

	// ---- Counters ------------------------------------------------------
	SectionHeader("Counters", maxW);
	VSpace(6.0f);
	TextWrapped(F.regular, 12.0f, P.textMuted, maxW,
		"Wire-level health. Bad-magic drops mean something is sending junk to port 29485 that "
		"isn't our protocol; bad-version drops mean the Frame service is out of date relative to "
		"the driver. Both should stay at 0 — any climb indicates samples the calibrator never "
		"got to see.");
	VSpace(8.0f);

	std::snprintf(line, sizeof line, "%llu", (unsigned long long)s.packets);
	kv("Packets accepted", line, s.packets > 0 ? P.textStrong : P.textDim);

	std::snprintf(line, sizeof line, "%llu", (unsigned long long)s.droppedBadMagic);
	kv("Bad-magic drops", line, s.droppedBadMagic == 0 ? P.textStrong : P.yellow);

	std::snprintf(line, sizeof line, "%llu", (unsigned long long)s.droppedBadVersion);
	kv("Bad-version drops", line, s.droppedBadVersion == 0 ? P.textStrong : P.yellow);

	VSpace(14.0f);

	// ---- Behaviour toggles --------------------------------------------
	SectionHeader("Calibration integration", maxW);
	VSpace(6.0f);
	TextWrapped(F.regular, 12.0f, P.textMuted, maxW,
		"Direct knobs that change how the next calibration run uses Frame data. Turn off the "
		"checkbox to get exactly the old OpenVR-only behaviour. The min confidence slider sets "
		"the floor for accepting a Frame sample — raising it discards noisier samples at the "
		"cost of needing more time to collect enough good ones.");
	VSpace(8.0f);
	if (CheckboxRow("Use Frame data in calibration",
		"When a Frame service is publishing fresh high-confidence poses, substitute them for the OpenVR-polled HMD pose during the next calibration run. Off makes SpaceSync behave exactly as it did before the Steam Frame integration.",
		&CalCtx.useFrameLink, maxW))
		SaveProfile(CalCtx);
	VSpace(10.0f);
	if (SliderRow("Min confidence",
		"Samples below this threshold are rejected (with a visible error in the calibration log) instead of silently used. Raise this if your Frame reports low-confidence samples during normal use.",
		&CalCtx.frameLinkMinConfidence, 0.0, 1.0, "%.2f", maxW, 110.0f, 64.0f))
		SaveProfile(CalCtx);

	VSpace(14.0f);
	if (!haveRecent)
	{
		Text(F.regular, 12.5f, P.textDim, "No Frame service has spoken to this driver yet. "
			"Install contrib/steamframe/service on the Frame to start the pipe.");
	}
}

void UserInterface::RenderSettings()
{
	Text(F.regular, 13.0f, P.yellow, "NOTE: Most settings below require re-calibration to be applied");
	VSpace(16.0f);

	const float fullW = PageDesignWidth();
	const float colW = (fullW - 40.0f) * 0.5f;
	const float cardW = (fullW - 2.0f * 12.0f) / 3.0f;

	SectionHeader("Tracking Method", fullW);
	VSpace(12.0f);

	{
		ImDrawList* dl = ImGui::GetWindowDrawList();

		const float padX = 16.0f, padY = 15.0f, dotD = 15.0f, gap = 10.0f;

		auto cardHeight = [&](float cardW, const char* label, const char* hint, const char* req) -> float
		{
			const float innerW = cardW - padX * 2.0f;
			const float headH = std::max(TextSize(F.semibold, 13.5f, label).y, px(dotD));
			return px(padY) + headH + px(9.0f)
				+ TextSize(F.regular, 12.0f, hint, innerW).y + px(7.0f)
				+ TextSize(F.regular, 12.0f, req, innerW).y + px(padY);
		};

		auto methodCard = [&](float cardW, float h, const char* label, const char* tag, unsigned tagColor,
			const char* hint, const char* req, bool active) -> bool
		{
			const float innerW = cardW - padX * 2.0f;

			ImVec2 labelSize = TextSize(F.semibold, 13.5f, label);
			ImVec2 tagSize = TextSize(F.semibold, 11.0f, tag);
			ImVec2 hintSize = TextSize(F.regular, 12.0f, hint, innerW);

			const float headH = std::max(labelSize.y, px(dotD));

			ImVec2 p = ImGui::GetCursorScreenPos();
			ImGui::PushID(label);
			ImGui::InvisibleButton("##method", ImVec2(px(cardW), h));
			ImGui::PopID();
			HoverHand();
			const bool clicked = ImGui::IsItemClicked();

			ImVec2 p1(p.x + px(cardW), p.y + h);
			dl->AddRectFilled(p, p1, Col(active ? P.cardActive : P.inputBg), px(5.0f));
			dl->AddRect(p, p1, Col(active ? P.accent : P.borderStrong), px(5.0f), 0, px(1.0f));

			float cy = p.y + px(padY) + headH * 0.5f;
			ImVec2 c(p.x + px(padX) + px(dotD) * 0.5f, cy);
			dl->AddCircleFilled(c, px(dotD) * 0.5f, Col(active ? P.accent : P.inputBg));
			dl->AddCircle(c, px(dotD) * 0.5f, Col(active ? P.accent : P.checkBorder), 0, px(1.0f));
			if (active)
				dl->AddCircleFilled(c, px(dotD) * 0.5f - px(3.0f), Col(P.cardActive));

			float tx = p.x + px(padX + dotD + gap);
			DrawText(dl, F.semibold, 13.5f, ImVec2(tx, cy - labelSize.y * 0.5f), P.textStrong, label);

			float bx = tx + labelSize.x + px(gap);
			ImVec2 b0(bx, cy - tagSize.y * 0.5f - px(2.0f));
			ImVec2 b1(bx + tagSize.x + px(14.0f), cy + tagSize.y * 0.5f + px(2.0f));
			dl->AddRectFilled(b0, b1, Col(tagColor, 0.12f), px(3.0f));
			DrawText(dl, F.semibold, 11.0f, ImVec2(bx + px(7.0f), b0.y + px(2.0f)), tagColor, tag);

			float ty = p.y + px(padY) + headH + px(9.0f);
			ImGui::SetCursorScreenPos(ImVec2(p.x + px(padX), ty));
			TextWrapped(F.regular, 12.0f, P.textMuted, innerW, hint);
			ImGui::SetCursorScreenPos(ImVec2(p.x + px(padX), ty + hintSize.y + px(7.0f)));
			TextWrapped(F.regular, 12.0f, P.textDim, innerW, req);

			ImGui::SetCursorScreenPos(ImVec2(p.x, p1.y));
			return clicked;
		};

		const float cardsX = ImGui::GetCursorPosX();
		const float rowY = ImGui::GetCursorPosY();

		const char* hintHmd = "The headset owns the tracking space and your lighthouse devices follow it. A tracker on your head keeps them lined up continuously, even when the headset silently re-centres.";
		const char* hintLighthouse = "The tracker on your head owns the tracking space and the headset follows it. Not recommended on Galaxy XR, Vive Pro or Pico 4, where it can cause jitter.";
		const char* hintNoTracker = "The headset owns the tracking space and your lighthouse devices follow it, but there is no running alignment. You calibrate once by holding a controller or a tracker against your head, then put it back.";
		const char* reqTracker = "Requires a tracker on top of your head.";
		const char* reqNone = "No permanent tracker needed. Drifts over time.";

		const float cardH = std::max(cardHeight(cardW, "HMD Driven", hintHmd, reqTracker),
			std::max(cardHeight(cardW, "Lighthouse Driven", hintLighthouse, reqTracker),
				cardHeight(cardW, "HMD Driven + No Tracker", hintNoTracker, reqNone)));

		const bool isHmd = CalCtx.followSlamHmd && !CalCtx.noHeadTracker;
		const bool isNoTracker = CalCtx.followSlamHmd && CalCtx.noHeadTracker;

		bool pickHmd = methodCard(cardW, cardH, "HMD Driven", "Highly Recommended", P.green,
			hintHmd, reqTracker, isHmd);

		ImGui::SetCursorPos(ImVec2(cardsX + px(cardW + 12.0f), rowY));
		bool pickLighthouse = methodCard(cardW, cardH, "Lighthouse Driven", "Recommended", P.yellow,
			hintLighthouse, reqTracker, !CalCtx.followSlamHmd);

		ImGui::SetCursorPos(ImVec2(cardsX + 2.0f * px(cardW + 12.0f), rowY));
		bool pickNoTracker = methodCard(cardW, cardH, "HMD Driven + No Tracker", "Not Recommended", P.danger,
			hintNoTracker, reqNone, isNoTracker);

		ImGui::SetCursorPos(ImVec2(cardsX, rowY + cardH));

		if ((pickHmd && !isHmd) || (pickNoTracker && !isNoTracker) || (pickLighthouse && CalCtx.followSlamHmd))
		{
			CalCtx.followSlamHmd = pickHmd || pickNoTracker;
			CalCtx.noHeadTracker = pickNoTracker;
			if (CalCtx.validProfile)
				SaveProfile(CalCtx);
		}
	}

	VSpace(28.0f);

	const float x0 = ImGui::GetCursorPosX();
	const float y0 = ImGui::GetCursorPosY();

	// Left column
	CheckboxRow("Fallback to SLAM",
		"Temporarily uses the headset's own (SLAM) tracking if the head tracker loses line of sight.",
		&CalCtx.fallbackToSlam, colW);
	CheckboxRow("Enable Angular Velocity",
		"Passes the tracker's angular velocity through to SteamVR. Off by default, it can cause issues with some devices.",
		&CalCtx.enableAngularVelocity, colW);
	if (CheckboxRow("Allow Same Tracking Space",
		"Normally calibration requires the head tracker to come from a different tracking system than the headset. Turn this on if you want to use a lighthouse tracker as the head tracker for a lighthouse headset (e.g. Vive Tracker + Valve Index). Only enable this if you know what you are doing.",
		&CalCtx.allowSameSpaceCalibration, colW))
		SaveProfile(CalCtx);
	CheckboxRow("Relative Calibration",
		"Continuously re-aligns SLAM-tracked devices (controllers etc.) to the calibrated space by comparing the headset's SLAM pose with the tracker-driven pose.",
		&CalCtx.continuousSync, colW);
	if (CheckboxRow("Hide Head Tracker",
		"Parks the tracker mounted on your headset far out of the way so games and SteamVR stop treating it as a device in your play space. Alignment is unaffected. Needs HMD Driven with a tracker, and pauses itself while you calibrate.",
		&CalCtx.hideHeadTracker, colW) && CalCtx.validProfile)
		SaveProfile(CalCtx);
	if (CheckboxRow("Stay Aligned",
		"Only for HMD Driven + No Tracker, and it still requires at least one Vive/Tundra hip tracker (SteamVR role \"Waist\"). It reduces drift over time when hiccups occur. It will not completely eliminate drift, but it reduces it drastically.",
		&CalCtx.stayAligned, colW) && CalCtx.validProfile)
		SaveProfile(CalCtx);
	if (CalCtx.stayAligned)
	{
		const auto& st = CalCtx.driverStatus;
		VSpace(6.0f);
		if (!(CalCtx.followSlamHmd && CalCtx.noHeadTracker))
			Hint("Inactive: needs HMD Driven + No Tracker.", colW);
		else if (!st.stayActive)
			Hint("Waiting for calibration.", colW);
		else if (!st.stayHipFound)
			Hint("No tracker with SteamVR role Waist found. Only headset recenters and hiccups are handled.", colW);
		else if (!st.stayBodyReady)
			Hint("Learning the hip tracker. Stand normally for about a minute in total.", colW);
		else if (st.stayRescuing)
			Hint("Re-aligning after a headset pause.", colW);
		else
		{
			char line[160];
			std::snprintf(line, sizeof line, "%s %.1f deg / %.1f cm, %s %u, %s %u", loc::tr("Correction"), st.stayYawDeg, st.stayShiftM * 100.0,
				loc::tr("recenters"), st.stayRecenters, loc::tr("hiccups held"), st.stayHeld);
			Hint(line, colW);
		}
		VSpace(6.0f);
	}
	if (CheckboxRow("Lock Base Stations",
		"Keeps your base stations where they were when you calibrated. SteamVR sometimes moves base stations by mistake (for example after a tracker loses tracking or the headset wakes from standby), which makes your hands, feet or the whole view jump. Works in all modes. If you really move a base station, just calibrate again.",
		&CalCtx.lockBaseStations, colW))
	{
		SendUniverseLock();
		if (CalCtx.validProfile)
			SaveProfile(CalCtx);
	}
	if (CalCtx.lockBaseStations)
	{
		const auto& st = CalCtx.driverStatus;
		VSpace(6.0f);
		if (st.lockState == 1)
			Hint("Calibrate once to activate the lock.", colW);
		else if (st.lockState == 3)
			Hint("Paused while calibrating.", colW);
		else if (st.lockState == 2)
		{
			char line[200];
			std::snprintf(line, sizeof line, "%s: %u. %s: %u, %s %.0f cm.", loc::tr("Locked base stations"), st.lockBases,
				loc::tr("SteamVR moves held this session"), st.lockHeldJumps, loc::tr("largest"), st.lockLargestHeldM * 100.0);
			Hint(line, colW);
			if (st.lockOffsetM > 0.02)
			{
				std::snprintf(line, sizeof line, "%s %.0f cm / %.1f deg. %s", loc::tr("SteamVR currently places your devices away from the calibration by"),
					st.lockOffsetM * 100.0, st.lockOffsetDeg, loc::tr("This is held. If you really moved a base station, calibrate again."));
				Hint(line, colW);
			}
			if (st.lockInconsistentM > 0.1)
			{
				std::snprintf(line, sizeof line, "%s (%.0f cm). %s", loc::tr("Your SteamVR base station layout is inconsistent"),
					st.lockInconsistentM * 100.0, loc::tr("Redo SteamVR Room Setup, then calibrate again."));
				Hint(line, colW);
			}
		}
		VSpace(6.0f);
	}
	if (CheckboxRow("Disable Voice Help",
		"Disables the voice that tells you how to calibrate during the calibration.",
		&CalCtx.disableVoiceHelp, colW))
		SaveProfile(CalCtx);

	float leftBottom = ImGui::GetCursorPosY();

	// Right column
	const float rx = x0 + px(colW + 40.0f);
	ImGui::SetCursorPos(ImVec2(rx, y0));
	{
		const float labelW = 110.0f, valueW = 26.0f;
		const float rowH = px(20.0f);
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 wp = ImGui::GetWindowPos();

		auto valueSlider = [&](const char* id, const char* label, const char* hint, double* v, double minV, double maxV, const char* fmt, bool* deactivated) -> bool
		{
			float y = ImGui::GetCursorPosY();
			float th = TextSize(F.regular, 13.0f, label).y;
			ImGui::SetCursorPos(ImVec2(rx, y + (rowH - th) * 0.5f));
			Text(F.regular, 13.0f, P.text, label);
			ImGui::SetCursorPos(ImVec2(rx + px(labelW + 14.0f), y));
			bool changed = Slider(id, v, minV, maxV, colW - labelW - 14.0f - 14.0f - valueW);
			if (deactivated)
				*deactivated = ImGui::IsItemDeactivated();
			char buf[16];
			std::snprintf(buf, sizeof buf, fmt, *v);
			ImVec2 vs = TextSize(F.mono, 12.5f, buf);
			DrawText(dl, F.mono, 12.5f, ImVec2(wp.x - ImGui::GetScrollX() + rx + px(colW) - vs.x, wp.y - ImGui::GetScrollY() + y + (rowH - vs.y) * 0.5f), P.textStrong, buf);
			ImGui::SetCursorPos(ImVec2(rx, y + rowH));
			VSpace(8.0f);
			ImGui::SetCursorPosX(rx);
			Hint(hint, colW);
			VSpace(24.0f);
			return changed;
		};

		double pred = CalCtx.predictionTime;
		if (valueSlider("##prediction", "Prediction Time", "How many frames of prediction SteamVR applies to the tracker. Some wireless solutions may need more prediction to feel smooth.", &pred, 0.0, 10.0, "%.1f", nullptr))
			CalCtx.predictionTime = (float)pred;

		{
			const float scales[4] = { 1.0f, 1.25f, 1.5f, 2.0f };
			const float y = ImGui::GetCursorPosY();
			const float pillH = TextSize(F.mono, 12.0f, "0").y + px(10.0f);
			const float th = TextSize(F.regular, 13.0f, "UI Scale").y;

			ImGui::SetCursorPos(ImVec2(rx, y + (pillH - th) * 0.5f));
			Text(F.regular, 13.0f, P.text, "UI Scale");

			float x = rx + px(labelW + 14.0f);
			for (int i = 0; i < 4; i++)
			{
				char buf[8];
				std::snprintf(buf, sizeof buf, "%.2f", scales[i]);
				const bool active = CalCtx.uiScale > scales[i] - 0.001f && CalCtx.uiScale < scales[i] + 0.001f;

				ImGui::SetCursorPos(ImVec2(x, y));
				if (Pill(buf, active) && !active)
				{
					CalCtx.uiScale = scales[i];
					if (CalCtx.validProfile)
						SaveProfile(CalCtx);
				}
				x += TextSize(F.mono, 12.0f, buf).x + 2.0f * px(10.0f) + px(6.0f);
			}

			ImGui::SetCursorPos(ImVec2(rx, y + pillH));
			VSpace(8.0f);
			ImGui::SetCursorPosX(rx);
			Hint("Size of text and controls in this window and in the SteamVR dashboard overlay.", colW);
			VSpace(24.0f);
		}

		ImGui::SetCursorPosX(rx);
		Text(F.regular, 13.0f, P.textMuted, "Calibration Speed");
		VSpace(10.0f);

		struct SpeedOpt { const char* label; const char* hint; CalibrationContext::Speed speed; };
		const SpeedOpt speeds[3] = {
			{ "Fast", "One pass through the look-around sequence. Quickest option, but small mistakes during calibration show up as inaccuracy.", CalibrationContext::FAST },
			{ "Slow", "Recommended. Two passes through the look-around sequence, so the same samples cover more directions.", CalibrationContext::SLOW },
			{ "Very Slow", "Three passes and the most samples. Use this when you want the most precise result.", CalibrationContext::VERY_SLOW },
		};
		for (int i = 0; i < 3; i++)
		{
			ImGui::SetCursorPosX(rx);
			if (RadioRow(speeds[i].label, speeds[i].hint, CalCtx.calibrationSpeed == speeds[i].speed, colW))
				CalCtx.calibrationSpeed = speeds[i].speed;
		}
	}

	float rightBottom = ImGui::GetCursorPosY();
	ImGui::SetCursorPos(ImVec2(x0, std::max(leftBottom, rightBottom)));
	VSpace(8.0f);
}

// -----------------------------------------------------------------------------------------

void UserInterface::RenderWizard()
{
	const char* popupId = "##calibration_wizard";
	if (wizardOpen_ && !ImGui::IsPopupOpen(popupId))
		ImGui::OpenPopup(popupId);
	if (!wizardOpen_)
		return;

	const float w = 460.0f;
	ImGuiIO& io = ImGui::GetIO();
	ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
	ImGui::SetNextWindowSize(ImVec2(px(w), 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(6.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
	ImGui::PushStyleColor(ImGuiCol_Border, ColV(P.borderButton));
	ImGui::PushStyleColor(ImGuiCol_PopupBg, ColV(P.card));

	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar;

	if (ImGui::BeginPopupModal(popupId, nullptr, flags))
	{
		ImDrawList* dl = ImGui::GetWindowDrawList();
		const ImVec2 wp = ImGui::GetWindowPos();
		const float ww = px(w);

		// --- state ---
		const CalibrationState st = CalCtx.state;
		const bool running = st == CalibrationState::Begin || st == CalibrationState::Detect || st == CalibrationState::Sampling;
		float fraction = 0.0f;
		std::string lastLine;
		CalibrationFeedback(fraction, lastLine);

		std::string counter, title;
		Icon icon = Icon::Dot;
		unsigned iconColor = P.link, fillColor = P.accent;
		float fill = 0.0f;
		bool done = false, failed = false;

		if (st == CalibrationState::Begin)
		{
			counter = "Starting";
			title = "Starting calibration";
			icon = Icon::Ring;
		}
		else if (st == CalibrationState::Detect)
		{
			counter = "Detecting tracker";
			title = "Move your head around";
			icon = Icon::Ring;
			fill = fraction;
		}
		else if (st == CalibrationState::Sampling)
		{
			const int total = CalCtx.sequenceSteps > 0 ? CalCtx.sequenceSteps : CalCtx.SequenceStepCount();
			const int step = std::min(total - 1, std::max(0, CalCtx.sequenceStep));
			char buf[32];
			std::snprintf(buf, sizeof buf, loc::tr("Step %d of %d"), step + 1, total);
			counter = buf;
			title = kSteps[step % kStepCount].title;
			icon = kSteps[step % kStepCount].icon;
			fill = fraction;
		}
		else
		{
			if (CalCtx.lastCalibrationOk)
			{
				done = true;
				counter = "Complete";
				title = "Done";
				icon = Icon::Check;
				iconColor = P.green;
				fillColor = P.green;
				fill = 1.0f;
			}
			else
			{
				failed = true;
				counter = "Aborted";
				title = "Calibration failed";
				icon = Icon::Cross;
				iconColor = P.danger;
				fillColor = P.danger;
				fill = 0.0f;
			}
		}

		// --- header ---
		{
			float h = px(14.0f) + TextSize(F.semibold, 13.0f, "X").y + px(14.0f);
			ImVec2 p = ImGui::GetCursorScreenPos();
			ImGui::Dummy(ImVec2(ww, h));
			DrawText(dl, F.semibold, 13.0f, ImVec2(p.x + px(18.0f), p.y + px(14.0f)), P.textStrong, "Calibration");
			ImVec2 cs = TextSize(F.regular, 12.0f, counter.c_str());
			DrawText(dl, F.regular, 12.0f, ImVec2(p.x + ww - px(18.0f) - cs.x, p.y + px(14.0f) + (TextSize(F.semibold, 13.0f, "X").y - cs.y) * 0.5f), P.textDim, counter.c_str());
			dl->AddRectFilled(ImVec2(p.x, p.y + h - 1.0f), ImVec2(p.x + ww, p.y + h), Col(P.border));
		}

		// --- body ---
		{
			const bool hasMessage = (failed || (running && st == CalibrationState::Sampling && !lastLine.empty() && lastLine.find("samples") != std::string::npos));

			// Wrap the status/error line to the progress-bar band so long messages
			// (e.g. "Calibration quality is too low, ...") stay on-screen.
			const float wrapW = px(w - 48.0f);
			ImFont* msgFont = F.regular;
			const float msgFs = FontPx(12.0f);
			float msgH = 0.0f;
			if (hasMessage)
				msgH = msgFont->CalcTextSizeA(msgFs, FLT_MAX, wrapW, loc::tr(lastLine.c_str())).y;

			float bodyH = px(34.0f + 44.0f + 20.0f + 19.0f + 26.0f + 3.0f + 30.0f)
				+ (hasMessage ? (px(10.0f) + msgH + px(12.0f)) : 0.0f);
			ImVec2 p = ImGui::GetCursorScreenPos();
			ImGui::Dummy(ImVec2(ww, bodyH));
			float cx = p.x + ww * 0.5f;
			float y = p.y + px(34.0f);
			DrawIcon(dl, icon, ImVec2(cx, y + px(22.0f)), 40.0f, Col(iconColor), 3.0f);
			y += px(44.0f + 20.0f);
			DrawTextCentered(dl, F.semibold, 19.0f, ImVec2(cx, y + px(9.5f)), P.textBright, title.c_str());
			y += px(19.0f + 26.0f);
			float bx0 = p.x + px(24.0f), bx1 = p.x + ww - px(24.0f);
			dl->AddRectFilled(ImVec2(bx0, y), ImVec2(bx1, y + px(3.0f)), Col(P.borderStrong), px(2.0f));
			if (fill > 0.0f)
				dl->AddRectFilled(ImVec2(bx0, y), ImVec2(bx0 + (bx1 - bx0) * std::min(1.0f, fill), y + px(3.0f)), Col(fillColor), px(2.0f));
			y += px(3.0f);
			if (hasMessage)
			{
				y += px(10.0f);
				dl->AddText(msgFont, msgFs, ImVec2(bx0, std::floor(y + px(6.0f) + 0.5f)),
					Col(failed ? P.danger : P.textDim), loc::tr(lastLine.c_str()),
					nullptr, bx1 - bx0);
			}
		}

		// --- footer ---
		{
			ButtonOpts secondary;
			secondary.padX = 16.0f;
			secondary.padY = 8.0f;
			float bh = TextSize(F.medium, 13.0f, "X").y + px(16.0f);
			float h = px(14.0f) + bh + px(14.0f);
			ImVec2 p = ImGui::GetCursorScreenPos();
			dl->AddRectFilled(ImVec2(p.x, p.y), ImVec2(p.x + ww, p.y + 1.0f), Col(P.border));

			ImGui::SetCursorScreenPos(ImVec2(p.x + px(18.0f), p.y + px(14.0f)));
			if (Button(running ? "Cancel" : "Close", secondary))
			{
				if (running)
					CancelCalibration();
				wizardOpen_ = false;
				ImGui::CloseCurrentPopup();
			}
			if (done)
			{
				ButtonOpts primary;
				primary.kind = ButtonKind::Primary;
				primary.padX = 18.0f;
				primary.padY = 8.0f;
				float dw = TextSize(F.semibold, 13.0f, "Done").x + px(36.0f);
				ImGui::SetCursorScreenPos(ImVec2(p.x + ww - px(18.0f) - dw, p.y + px(14.0f)));
				if (Button("Done", primary))
				{
					wizardOpen_ = false;
					ImGui::CloseCurrentPopup();
				}
			}
			ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + h));
			ImGui::Dummy(ImVec2(ww, 0.0f));
		}

		ImGui::EndPopup();
	}

	ImGui::PopStyleColor(2);
	ImGui::PopStyleVar(3);
}

void UserInterface::RenderConfirm()
{
	const char* popupId = "##confirm_remove";
	if (confirmRemove_ && !ImGui::IsPopupOpen(popupId))
		ImGui::OpenPopup(popupId);
	if (!confirmRemove_)
		return;

	const float w = 380.0f;
	ImGuiIO& io = ImGui::GetIO();
	ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
	ImGui::SetNextWindowSize(ImVec2(px(w), 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(20.0f), px(20.0f)));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(6.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
	ImGui::PushStyleColor(ImGuiCol_Border, ColV(P.borderButton));
	ImGui::PushStyleColor(ImGuiCol_PopupBg, ColV(P.card));

	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar;

	if (ImGui::BeginPopupModal(popupId, nullptr, flags))
	{
		const float inner = w - 40.0f;
		Text(F.semibold, 14.0f, P.textBright, "Remove Calibration?");
		VSpace(8.0f);
		TextWrapped(F.regular, 13.0f, P.textMuted, inner, "All offsets will be discarded. You will need to calibrate again.");
		VSpace(20.0f);

		ButtonOpts cancel;
		cancel.padX = 16.0f;
		cancel.padY = 8.0f;
		ButtonOpts remove;
		remove.kind = ButtonKind::Danger;
		remove.padX = 16.0f;
		remove.padY = 8.0f;
		float cw = TextSize(F.medium, 13.0f, "Cancel").x + px(32.0f);
		float rw = TextSize(F.semibold, 13.0f, "Remove").x + px(32.0f);
		float x0 = ImGui::GetCursorPosX();
		ImGui::SetCursorPosX(x0 + px(inner) - cw - px(8.0f) - rw);
		if (Button("Cancel", cancel))
		{
			confirmRemove_ = false;
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine(0.0f, px(8.0f));
		if (Button("Remove", remove))
		{
			CalCtx.Clear();
			SaveProfile(CalCtx);
			confirmRemove_ = false;
			ImGui::CloseCurrentPopup();
		}
		ImGui::Dummy(ImVec2(px(inner), 0.0f));
		ImGui::EndPopup();
	}

	ImGui::PopStyleColor(2);
	ImGui::PopStyleVar(3);
}
