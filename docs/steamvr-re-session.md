# SteamVR reverse-engineering — interoperability study

## Purpose and scope

This note documents observations about SteamVR's PC-side driver binaries,
made solely for the purpose of making **OpenVR-SpaceSync** (an existing
third-party, AGPL-3.0, no-fee calibration overlay) interoperate better
with SteamVR's published runtime. It records only:

- Printable log strings already emitted at runtime to any user with the
  `SteamVR\logs\vrserver.txt` file open.
- Public C++ ABI symbols (interface version strings, standard property
  indices from `openvr.h` / `openvr_driver.h`) that the driver links
  against.
- Behavioural inferences about *which public API call* carries a given
  signal.

This note **does not** and will not contain reproduced disassembly,
reproduced decompiler output, proprietary data-structure definitions,
cryptographic keys, DRM-adjacent material, or anything else that would
require redistributing Valve's binaries or derivative work from them.
Every actionable recommendation uses only interfaces published in the
open-source OpenVR SDK (BSD-3-Clause, https://github.com/ValveSoftware/openvr).
The goal is explicitly interoperability under the applicable legal
carve-outs for reverse engineering (US DMCA §1201(f) "interoperability",
EU Software Directive Art. 6(1) "decompilation for interoperability"),
and this document is sufficient on its own to drive the implementation
work — a future contributor does **not** need to repeat the RE.

Nothing here is a bug report against SteamVR. Where lighthouse's
behaviour is already documented in `openvr_driver.h` (e.g. `ETrackingResult`),
that header is the authority; this note only clarifies when a driver
actually sets a given value.

If a Valve engineer reads this and prefers a different signal be used,
please open an issue on the SpaceSync repo and we will switch.

## Binaries surveyed

- `drivers/vrlink/bin/win64/driver_vrlink.dll` — Steam Frame streaming driver
  (PC side). Internal brand: **SVL** ("Steam Virtual Link"). Hosts a
  per-driver JSON RPC mailbox named `svl/driver`.
- `drivers/lighthouse/bin/win64/driver_lighthouse.dll` — lighthouse tracking.
- `bin/win64/vrserver.exe` — core SteamVR server; not inspected in depth.

## Session 1 — strings-only (2026-11-19)

Initial pass without a disassembler. Established that:

- vrlink's logs mention only user-visible events (recenter, dock change,
  room setup). No per-sample confidence, SLAM state, or relocalization
  strings are present in vrlink, confirming that the Steam Frame's
  tracking-quality signals stay on the Frame side and never cross the
  `vrlink` tether. **This is why SpaceSync's own UDP backchannel
  (`FrameLinkProtocol.h`) is necessary — there is no equivalent signal
  flowing through vrlink that SpaceSync could listen to instead.**

- lighthouse's logs expose four categories of signal of interest:
  universe lifecycle (`Selected existing universe`, `Creating new
  universe`, `Stopped tracking with universe`), base-station health
  (`No optical frames in past %d seconds`, `Optical frames received
  again…`, `No IMU data in past…`, `IMU data received again…`,
  `Watchman saw FIFO overflow event`), world tilt (`Setting universe
  tilt from %08X %s: pitch %0.2f deg roll %0.2f deg`), and chaperone
  property configuration (`Configuring VRLINK Chaperone Prop container
  %08x: Universe %llu` — written by vrlink, not lighthouse).

## Session 2 — IDA + ida-pro-mcp (2026-11-20)

With the ida-pro-mcp server reachable, each anchor log string was
back-traced to its emitting function. The findings below describe
**behaviour and public-API shape only**; no proprietary code is
reproduced. Function offsets are included so a future reviewer can
verify this note against their own IDB.

### 2.1 — vrlink's SVL mailbox (`SVLServer::Init`, driver_vrlink.dll @ 0x180A00610)

**What it is.** vrlink opens a per-driver JSON RPC mailbox named
`svl/driver` and registers handlers for a fixed set of named messages.
The mailbox uses Valve's internal mailbox IPC (not `IVRMailbox` from
the public SDK, which is a *browser* mailbox — these are unrelated).

**Registered message names, verbatim from the strings table:**
`remote_dashboard_open`, `remote_dashboard_close`,
`remote_dashboard_toggle`, `handle_recenter`, `vrlink_room_setup`,
`vrlink_frame_docklocation_request`,
`vrlink_frame_preview_docklocation_request`,
`vrlink_focus_dashboard_bar_request`, `user_interaction_click_info`,
`lm_haptic`, `dump_aftm_csv`.

**Observable from outside vrlink?** No. These handlers are registered
on a mailbox owned by vrlink's own `SVLServer` instance. Other drivers
loaded by vrserver do not share this mailbox and cannot subscribe to
its traffic. Treat the mailbox as a private IPC — **SpaceSync must use
the public equivalents, not try to tap this channel.**

**Public equivalents SpaceSync should use:**

- `handle_recenter` → observe `VREvent_SceneApplicationStateChanged`
  plus a pose-jump heuristic, *or* watch the chaperone's
  `WorkingStandingZeroPoseToRawTrackingPose` for an abrupt rotation
  change (`IVRChaperoneSetup::GetWorkingStandingZeroPoseToRawTrackingPose`).
  Our existing `JumpDetect` already approximates this.
- `vrlink_frame_docklocation_request` → no public equivalent;
  this is Steam-Frame-specific UI state and is not useful to
  SpaceSync's calibration path anyway.

### 2.2 — vrlink's chaperone update (`UpdateChaperone`, driver_vrlink.dll @ 0x1809F1670)

**What it is.** When the Frame sends new chaperone geometry over the
tether, vrlink parses the JSON (fields `universes[0].universeID` + bounds),
compares the new universe ID against the device's current value, and
on change writes the ID into a public property on the HMD's property
container.

**Public-API surface used by vrlink itself, observed verbatim in the
strings and vtable dispatch:**

- Interface: `"IVRProperties_001"` (from the published OpenVR SDK).
- Property set (vrlink's write): property index **2004**, with
  k_unUint64PropertyTag (payload size 8 bytes). In `openvr.h` this is
  **`Prop_CurrentUniverseId_Uint64`**.
- Property set (secondary write): property index **2095** with the same
  universe value. In `openvr.h` this is near `Prop_ArbitraryEnvironment_CurrentContainer_Uint64`
  / adjacent chaperone-container properties — a stable public key.

**Takeaway for SpaceSync.** On any HMD (not just Frame), the universe
ID is a standard tracked-device property. SpaceSync can read it with
`IVRSystem::GetUint64TrackedDeviceProperty(hmdId, Prop_CurrentUniverseId_Uint64)`
and detect transitions **without** any lighthouse-internal mailbox or
any RE-derived knowledge. Our current `UniverseProbe` heuristic (watching
pose discontinuities) can be replaced or reinforced by polling this
property.

### 2.3 — lighthouse's universe tilt (driver_lighthouse.dll @ 0x1800632A0)

**What it is.** The function that emits `Setting universe tilt from
%08X %s: pitch %0.2f deg roll %0.2f deg` computes a 2-DOF (pitch, roll —
no yaw) rotation describing gravity for a given universe, referenced to
a specific base station. The result is cached per universe inside the
lighthouse driver's private DB. The computation itself is
`sub_180094B10(out, origin_handle, universe_id)` — a lookup-plus-math
routine; its exact body is not needed and not reproduced here.

**Observable from outside lighthouse?** Yes — via the chaperone setup.
The 2-DOF tilt is what parameterises
`IVRChaperoneSetup::GetWorkingStandingZeroPoseToRawTrackingPose`. The
Y column of the 3×4 matrix returned by that public API *is* the
universe-tilt-derived gravity vector in the HMD driver's raw-tracking
space. For any playspace, call that API and read the Y column; the
chaperone setup is a public, documented, stable API.

**Takeaway for SpaceSync.** Every lighthouse user, every Steam Frame
user, and every HMD whose driver sets its chaperone standing-zero pose
gives SpaceSync a free gravity vector in raw-driver space, **without
any reverse engineering**. This vector can be used to lock pitch/roll
of the calibration rotation solve, collapsing it from 3-DOF to yaw-only.
Section 4 lays this out concretely.

### 2.4 — lighthouse's optical/IMU watchdog (driver_lighthouse.dll @ 0x18009DC20)

**What it is.** Every 5 seconds the lighthouse driver reviews the
per-base-station flag byte `a1 + 232`. The bit meanings — inferred
purely from the branch structure that controls the four log strings
`No IMU data in past %d seconds`, `IMU data received again…`,
`No optical frames in past %d seconds`, `Optical frames received
again…`, `Watchman saw FIFO overflow event` — are:

| bit  | meaning                                                      |
|------|--------------------------------------------------------------|
| 0x02 | IMU packet received in the last 5-second window              |
| 0x04 | Watchman sensor FIFO overflowed (data lost)                  |
| 0x08 | Optical frame received in the last 5-second window           |
| 0x10 | Latched: no IMU for 5 s — now in IMU-fallback / coasting     |
| 0x20 | Latched: no optical for 5 s — now in "no optical" fallback   |

**Observable from outside lighthouse?** Yes. When bit 0x20 latches to
1, the lighthouse driver sets each affected tracked device's pose to
**`TrackingResult_Fallback_RotationOnly`** (public enum, `openvr.h`,
value 300). When 0x08 is set but geometric residuals are poor,
**`TrackingResult_Running_OutOfRange`** (value 201) is published.
Other drivers (Oculus Link, WMR, ALVR, Pimax, …) use the same enum,
so a consumer-side gate on `pose.result` is universal.

**Takeaway for SpaceSync.** The drift path at
`src/driver/ServerTrackedDeviceProvider.cpp:1273` already checks
`pose.result == TrackingResult_Running_OK` on the HMD. The Stay Aligned
path at the same file around `:1766` only checks the tracker side; add
the same HMD-side check and the gate becomes universal.

### 2.5 — lighthouse's universe lifecycle (driver_lighthouse.dll @ 0x18005E440, 0x180063730)

**What they are.** `sub_18005E440` is invoked when a universe is
either resumed (`Selected existing universe %llu` + an 8-digit hex
handle) or newly created (`Creating new universe %llu because …`).
`sub_180063730` fires when a universe stops being tracked
(`Stopped tracking with universe %llu`). Both walk an internal
listener list, notifying `vortex::ILighthouseDBListener` instances
registered with the lighthouse DB — these are **lighthouse-internal
C++ listeners, not exposed across the driver boundary.**

**Observable from outside lighthouse?** Yes, through the same public
property watched by vrlink in §2.2: `Prop_CurrentUniverseId_Uint64`
(2004) updates on universe change. Transitions are observable by
polling (`IVRSystem::GetUint64TrackedDeviceProperty`) or by subscribing
to the property-changed event (`VREvent_PropertyChanged`, with the
containing device being the HMD).

## 3. Full cross-reference against the public OpenVR SDK

Every signal the RE identified has a reachable path through the
published SDK. The table below is the complete map — the only one
SpaceSync should use going forward.

| Signal                                | Private path (do not use)             | Public path (recommended)                                                                 | Header                        |
|---------------------------------------|---------------------------------------|-------------------------------------------------------------------------------------------|-------------------------------|
| Base station lost optical ≥ 5 s       | lighthouse watchdog bit 0x20          | `pose.result == TrackingResult_Fallback_RotationOnly` on affected devices                 | `openvr.h` ETrackingResult    |
| Optical / IMU momentarily degraded    | lighthouse watchdog bits 0x02/0x08    | `pose.result == TrackingResult_Running_OutOfRange`                                        | `openvr.h` ETrackingResult    |
| Universe tilt (gravity) per playspace | lighthouse internal DB @ 0x1800632A0  | `IVRChaperoneSetup::GetWorkingStandingZeroPoseToRawTrackingPose` — Y column               | `openvr.h` IVRChaperoneSetup  |
| Universe ID change                    | lighthouse listener walk @ 0x180063730; vrlink @ 0x1809F1670 | `Prop_CurrentUniverseId_Uint64` (2004) property, polled or via `VREvent_PropertyChanged` | `openvr.h` ETrackedDeviceProperty |
| User-triggered recenter               | vrlink SVL mailbox `handle_recenter`  | `VREvent_SceneApplicationStateChanged` + pose-jump heuristic (JumpDetect)                 | `openvr.h` EVREventType       |
| Chaperone bounds changed              | vrlink `UpdateChaperone`              | `VREvent_ChaperoneDataHasChanged` *and* `IVRChaperoneSetup::GetLiveCollisionBoundsInfo`   | `openvr.h` EVREventType       |
| Current base-station IMU bias change  | lighthouse log `… IMU calibration: Accel bias change …` | not surfaced publicly in a useful form; ignore for calibration            | n/a                           |
| High per-base frame jitter            | lighthouse log `high_frame_jitter`    | not surfaced publicly; ignore for now                                                     | n/a                           |

## 4. Actionable improvements, ranked and scoped

Each item below uses only public APIs and is appropriate for an AGPL-3.0,
no-fee community project. Each is scoped to a specific SpaceSync file
and line.

### Priority 1 — Chaperone-derived gravity as a Kabsch constraint

**What.** Before running the current 3-DOF rotation Kabsch on calibration
samples, read the gravity vector from both the reference and target
playspaces via `IVRChaperoneSetup::GetWorkingStandingZeroPoseToRawTrackingPose`
and pre-rotate the sample clouds so gravity-Y points the same way in
both. Then solve only the remaining yaw-DOF.

**Why.** Pitch and roll are already known from each driver's chaperone
setup; forcing the Kabsch to re-learn them from samples adds variance
and needs more data. A gravity-locked yaw-only solve is strictly more
stable, needs fewer samples, and is correct for any HMD whose driver
sets its standing zero pose (which is every modern driver — it is
chaperone setup's whole purpose).

**Where.** `src/overlay/Calibration.cpp`, the `CalibrateRotation` path
(the one touched by our confidence-weighted Kabsch PR, around the
Eigen `BDCSVD` call). Add a preamble that fetches both gravities and
rotates the Eigen matrices before SVD; add a fallback that reverts to
3-DOF Kabsch when chaperone setup reports no standing zero pose
(uncalibrated universe).

**Effort.** ~1 day.

### Priority 2 — HMD-side optical-loss gate on Stay Aligned

**What.** Mirror the existing `pose.result == TrackingResult_Running_OK`
gate from the drift path onto the Stay Aligned update's HMD read.

**Where.** `src/driver/ServerTrackedDeviceProvider.cpp` around `:1766`.
Our FrameLink confidence gate at `:1497–1542` already handles the Frame
case; the lighthouse case (and every other driver) still needs the
public `pose.result` gate.

**Effort.** ~2 h.

### Priority 3 — Universe-ID polling to replace heuristic UniverseProbe

**What.** Poll `Prop_CurrentUniverseId_Uint64` (property 2004) on the
HMD device each frame. When it changes, fire the same hooks
`UniverseProbe` currently triggers from pose discontinuities. Keep the
heuristic as a fallback for drivers that do not populate the property.

**Where.** `src/driver/UniverseProbe.cpp` (new source file, or extend
existing probe logic). The property path is cheap (one `IVRSystem`
call), so it can run unconditionally.

**Effort.** ~4 h, including tests.

### Priority 4 — Chaperone-event listener for bounds changes

**What.** Subscribe to `VREvent_ChaperoneDataHasChanged` and, on
receive, refetch the current playspace bounds via
`IVRChaperoneSetup::GetLiveCollisionBoundsInfo`. Use this as a trigger
to re-verify calibration quality (and, in a future PR, auto-recalibrate
if the user opts in).

**Effort.** ~4 h.

## 5. Caveats

- Property indices 2004 and 2095 are observed to be used by vrlink in
  SteamVR 2.x; they are listed in the public `openvr.h` enum and
  therefore stable, but a future SDK rename would require updating
  the SpaceSync constant.
- The lighthouse 5-second watchdog describes *when* a driver transitions
  to `TrackingResult_Fallback_RotationOnly`, not what that enum value
  means — that is defined by `openvr_driver.h` itself.
- No binary redistribution, no reproduced disassembly, no reproduced
  decompiler output, no proprietary header text is present in this
  document or in the SpaceSync repo. If Valve requests any observation
  recorded here be removed, open an issue on the SpaceSync repo and it
  will be.
