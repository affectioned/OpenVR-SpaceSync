# Steam Frame integration

**What this does:** lets SpaceSync's calibration use a Steam Frame's own
SLAM pose instead of only the one OpenVR hands it. The Frame runs a small
daemon that ships raw HMD poses to the PC over UDP; the SpaceSync driver
accepts them and the overlay substitutes them into the calibration solve.

Issue: [#18](https://github.com/shinyflvre/OpenVR-SpaceSync/issues/18).
Showcase: [`docs/images/steam-frame-showcase.png`](images/steam-frame-showcase.png).

## The picture

```
┌──────────────── Steam Frame (SteamOS arm64) ────────────────┐
│                                                             │
│  vrserver (SteamVR runtime)                                 │
│        │                                                    │
│        │  OpenVR VRApplication_Background                   │
│        ▼                                                    │
│  spacesync-framelink.service (Python 3)                     │
│        │                                                    │
│        │  pack_pose(seq, capture_ticks, pos, quat, ...)     │
│        ▼                                                    │
│   UDP 29485 ───────┐                                        │
└────────────────────┼────────────────────────────────────────┘
                     │  LAN
                     ▼
┌────────────────── PC (Windows) ─────────────────────────────┐
│                                                             │
│  driver_spacesync.dll (hooked into vrserver)                │
│    framelink::Listener                                      │
│       ├── parse (magic SFLK, version 1)                     │
│       ├── store latest pose per device                      │
│       └── GetFrameLinkStatus / GetFrameLinkSample via IPC   │
│                                                             │
│  SpaceSync.exe (overlay)                                    │
│    CalibrationTick ──► IPC poll ──► Sample substitution     │
│    Frame Link tab   ◄── live stats, HMD cards, toggles      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Why UDP rather than a SteamVR driver on the Frame side?** The Frame is a
separate machine and we're only shipping ~90 bytes every 33 ms. A Linux-arm64
SteamVR driver would be a much bigger project and would only reach the same
information.

**Why OpenVR Background (not OpenXR) on the Frame?** Any OpenXR session —
even headless — makes SteamVR's compositor queue us like a VR game and
the headset shows "Next up: spacesync-framelink". `VRApplication_Background`
is explicitly scoped to clients that don't want focus. Babble Frame Runtime
takes the same route.

## Wire format

Defined in [`include/shared/FrameLinkProtocol.h`](../include/shared/FrameLinkProtocol.h).
All packets start with the same header:

| field    | size | notes                                         |
|----------|------|-----------------------------------------------|
| magic    | 4 B  | `0x4B4C4653` ('SFLK')                         |
| version  | 2 B  | `1`                                           |
| type     | 2 B  | `MsgHello`, `MsgPose`, `MsgGravity`, `MsgRelocalize` |
| sequence | 4 B  | monotonic per-sender, wraps freely            |

`MsgPose` carries position, orientation (wxyz), lin + ang velocity,
capture ticks (sender clock), device id, confidence, and flags. Confidence
is a float 0..1; samples below the configurable floor get rejected from
the calibration solve. Flags carry `PositionTracked`, `RotationTracked`,
and `InertialOnly`.

## Install

**PC side.** Build the usual way (`build.bat`) and copy
`driver_spacesync.dll` + `SpaceSync.exe` into your `C:\Program Files\SpaceSync\`
install. The driver binds UDP 29485 on load; if nothing talks to it you see
the Frame Link tab display an empty peer and the overlay behaves exactly like
before the Steam Frame integration.

**Frame side.** Enable `sshd` on the Frame once
([`SSH.md`](../contrib/steamframe/service/SSH.md)), then from the Frame's
Konsole paste the one-shot:

```
curl -sL https://raw.githubusercontent.com/affectioned/OpenVR-SpaceSync/feat/steam-frame-18/contrib/steamframe/service/bootstrap.sh | bash -s -- <PC-IP>
```

The bootstrap installs pyopenvr, drops the Python daemon in `~/.local/bin`,
installs a systemd unit, points it at your PC's LAN IP. Autostart + restart
on failure are on by default.

## Configuration

The daemon reads `/etc/default/spacesync-framelink` (system install) or
`~/.config/spacesync-framelink/env` (user install):

```
SPACESYNC_HOST=10.0.1.12           # your PC's LAN IP
SPACESYNC_PORT=29485               # change only if 29485 is taken
SPACESYNC_HZ=30                    # active rate (while head is moving)
SPACESYNC_IDLE_HZ=5                # idle rate (head still > 1 s)
SPACESYNC_BACKENDS=openvr,openxr,stub
```

Edit, then `systemctl --user restart spacesync-framelink`.

The PC side has two knobs in the overlay (Frame Link tab → Calibration
integration): **Use Frame data in calibration** (master toggle) and
**Min confidence** (floor below which samples are rejected).

## Power

The Frame is a 21.6 Wh device, so the daemon is written to stay out of
battery-sensitive code paths:

- Motion gated: drops to 5 Hz when the head is still for ~1 s, back to
  30 Hz the instant it moves. Threshold 0.5 mm / 0.5°.
- One `time.sleep(next_t - now)` per tick (no busy loop), so the CPU parks.
- `Nice=5` + `CPUWeight=20` in the unit so games win any contention.
- Measured on live hardware: ~2% of one core, ~35 MB RSS.

## Known limits

- **Capture ticks are "now at send time".** OpenVR's `TrackedDevicePose_t`
  doesn't carry a per-pose timestamp, and the runtime interpolates poses to
  whatever time you ask for. Real last-measurement timestamps would need a
  proper OpenXR headless event loop using `XR_KHR_locate_spaces` — it's on
  the roadmap but nothing in the calibrator uses capture ticks yet so no
  behavior depends on it.
- **No gravity direction.** The protocol has `MsgGravity` but the Frame
  daemon doesn't send it. The solver already has a lighthouse-side gravity
  path and runs fine without.
- **Only the HMD pose is substituted.** Controllers and trackers still come
  through the PC's own OpenVR, which is correct — they're the lighthouse
  side of the calibration.

## What's next

1. **A/B calibration quality.** Measure RMS with Frame data on vs off.
   Without this number we only know the pipe works, not that it helps.
2. **Gravity direction.** Fills `MsgGravity` from a few stationary samples
   and lets the solver constrain to yaw + translation only.
3. **OpenXR headless event loop.** Gives real last-measurement capture
   timestamps and removes our dependency on `VRApplication_Background`.
4. **Split for upstream review.** The branch covers wire protocol,
   calibration integration, UI tab, and the SteamOS service — probably
   one PR per layer.

## Code map

- `include/shared/FrameLinkProtocol.h` — wire structs.
- `include/driver/FrameLink.h` + `src/driver/FrameLink.cpp` — UDP listener.
- `src/driver/ServerTrackedDeviceProvider.cpp` — IPC bridge
  (`GetFrameLinkSample`, `GetFrameLinkStatus`).
- `src/overlay/Calibration.cpp` — `TryFrameHmdPose` + substitution.
- `src/overlay/UserInterface.cpp` — `RenderFrameLink` tab.
- `contrib/steamframe/service/` — Python daemon + systemd unit + installer.
- `contrib/steamframe/frame_sender.py` — PC-side reference sender for
  tests and showcasing.
