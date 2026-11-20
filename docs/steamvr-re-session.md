# SteamVR reverse-engineering — short notes

One-session survey of `C:\Program Files (x86)\Steam\steamapps\common\SteamVR`.
Tools: `dumpbin` + Python strings extraction (no IDA this round; MCP servers
unreachable). Goal: find signals SteamVR knows that SpaceSync doesn't, for
**any** HMD not just Steam Frame.

Binaries that matter:

- `drivers/vrlink/bin/win64/driver_vrlink.dll` — Steam Frame streaming driver (PC side). Internal brand: **SVL** ("Steam Virtual Link").
- `drivers/lighthouse/bin/win64/driver_lighthouse.dll` — lighthouse tracking.
- `bin/win64/vrserver.exe` — core SteamVR server.

## What the PC's streamed Frame driver doesn't expose

vrlink has zero strings for `confidence`, `slam`, `relocal`, or per-sample
tracking quality. Those signals live on the Frame side only. **The UDP
backchannel we built is the only path to them.** The architecture holds.

What vrlink *does* expose that we could still tap:

- `handle_recenter` + `vrlink_frame_docklocation_request` — the Frame signals
  user-triggered recenters and dock-change events to vrserver through a
  mailbox IPC. Not advertised in `openvr_driver.h`. If we can observe the
  mailbox messages (strings suggest text names, not binary IDs), SpaceSync
  can react to a user recenter immediately instead of inferring it from
  the sudden pose jump.
- `UpdateChaperone got new universe (len %zu)` — vrlink forwards universe
  changes from the Frame to the compositor. `UniverseProbe` already races
  to detect these; we could hook the same signal directly.
- QoS + retry counters are exposed (`nTimedRetryOption`, `nQoSOption`) —
  the Frame stream's health is observable. Could gate calibration on
  "stream has been clean for N seconds" for free.

## What the lighthouse driver knows that SpaceSync doesn't

Universal — these apply to every lighthouse setup, not just Frame.

### 1. Optical-frame loss on base stations

Strings:
`No optical frames in past %d seconds` / `Optical frames received again...`

When a base station stops seeing a sensor, lighthouse logs it and
dead-reckons from IMU. **SpaceSync's drift update currently trusts the
reconstructed pose anyway.** Same bug our Frame-confidence gate fixed,
but universal — gate drift updates on "all visible base stations
reported an optical frame within the last N ms."

### 2. Universe lifecycle events as first-class signals

Strings: `Selected existing universe %llu` / `Creating new universe %llu` /
`Stopped tracking with universe %llu` / `known_universes`.

The lighthouse driver has clear start/stop/select universe signals.
SpaceSync's `UniverseProbe` currently infers these by watching pose
discontinuities. We could read the event directly via a mailbox or
settings-watch and pre-empt the whole heuristic path.

### 3. Base-station tilt + per-base IMU calibration

Strings: `Setting universe tilt from %08X %s: pitch %0.2f deg roll %0.2f deg`
and `%s: %s IMU calibration: Accel bias change %.2fm/s/s%s`.

Lighthouse holds the "world tilt" relative to each base station's gravity,
plus per-base IMU bias. SpaceSync maintains its own `worldTilt`. If we
read lighthouse's tilt we get gravity for free — this is the same payoff
as `MsgGravity` on the Frame side, for *any* lighthouse setup, with no
new code on the HMD side.

Also: `high_frame_jitter` and `Ignoring unlikely saved IMU calibration`
are logged per base station. Both are "something moved" hints we could
treat as weak recalibration triggers.

### 4. Chaperone geometry already contains per-polygon confidence

Strings: `Configuring VRLINK Chaperone Prop container %08x: Universe %llu`.
Chaperone bounds come through as properties. If the geometry carries any
per-vertex confidence field (we didn't verify binary layout) it'd be a
free "where the user actually walks" prior.

## Concrete next steps, ranked

1. **Optical-frame-loss gate on drift/MountRefiner.** Universal, same shape
   as our Frame-confidence gate. Hook the lighthouse driver's loss signal
   via `IVRMailbox` or by watching the driver log. ~1 day.
2. **Direct universe lifecycle signal instead of heuristic probe.** Drop a
   lot of `UniverseProbe` code and gain correctness. ~1 day; depends on
   finding the mailbox name.
3. **Read lighthouse's world tilt as gravity.** Unlocks yaw+translation
   solve constraint for *every* lighthouse user. ~4 h once we know the
   IVRProperties key.
4. **vrlink recenter signal.** Listen on the vrlink mailbox for
   `handle_recenter`; apply directly instead of inferring. ~4 h.

## Caveats

- All four depend on undocumented mailbox names / property keys. Valve
  can rename them in any SteamVR update. Safe to tap for observability,
  risky as the *only* data path — always keep the existing heuristic
  fallback behind the toggle.
- Everything here was deduced from strings alone. Binary structure
  (prop IDs, message layouts) needs a proper RE pass with IDA before we
  write code against it. When the IDA MCP servers are reachable again,
  the natural follow-up is: dump `CSVLMailboxClient` + `CLighthouseDB`
  class layouts; grep for the exact `IVRMailbox::undocumentedSend` /
  `ReadMessage` call sites.
