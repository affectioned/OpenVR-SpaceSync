# Steam Frame link — scaffold

Prototype plumbing for issue #18. The idea: a small service running on the Steam
Frame streams data that isn't in the normal video stream — raw unpredicted
poses with capture timestamps, per-sample tracking confidence, gravity
direction, relocalize events — and SpaceSync's driver consumes it to tighten up
calibration.

### Status

- Wire protocol: `include/shared/FrameLink.h`.
- Receive path: `framelink::Listener`, started in `ServerTrackedDeviceProvider::Init`.
- Received packets land in a mutex-guarded snapshot (`LatestPose`,
  `LatestGravity`, `NextRelocalize`). **Nothing in the calibrator reads from
  that snapshot yet** — intentionally. The wire contract gets to settle before
  we touch calibration code.
- No sender on the Frame side yet. SteamOS access needs to be checked first,
  per the issue.

### Trying it out

`frame_sender.py` fakes the Frame side. Run it on the PC (or over LAN) and tail
`%LOCALAPPDATA%\SpaceSync\driver.log` while SteamVR is up with SpaceSync
loaded:

```
python frame_sender.py --host 127.0.0.1 --duration 5
```

You should see:

```
FrameLink: listening on UDP 29485
FrameLink: hello from 127.0.0.1:<port> product="steam-frame-sim" clockHz=10000000
```

Add `--reloc-after 2` to also inject a relocalize event at t=2s. The driver
logs these as they arrive.

### Next steps (not done here)

1. A real sender on SteamOS. First task: confirm what a background process can
   read from the compositor while streaming is active.
2. Wire `LatestPose` and `LatestGravity` into `Calibration.cpp`'s sample loop:
   use the Frame's capture ticks to pair SLAM and Lighthouse samples that
   actually happened at the same moment, drop low-confidence samples, and
   constrain the solve by the agreed gravity direction.
3. Handle relocalize events — drop any in-flight calibration samples that
   straddle one, and apply the reported origin shift to an active calibration
   instead of needing a redo.
4. Clock sync via `MsgClockPing` / `MsgClockPong` (fields exist, no driver
   logic yet).
