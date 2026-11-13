# SpaceSync FrameLink service (Steam Frame side)

> Working with the Frame over SSH for the first time? See [`SSH.md`](SSH.md)
> for the one-time setup (enable sshd, user is `steamos` not `deck`, install
> a key, read-only root quirks, how to tail the right journal).


Runs on the Frame (or any ARM64/x86_64 Linux box with Python 3.11+), opens
a backend that reads the HMD tracking pose, and ships it to the PC's
SpaceSync driver over UDP 29485 using the shared FrameLink wire format.

## Layout

- `spacesync-framelink.py` — the daemon itself. Backends are plug-in. Default
  order tries pyopenxr first, falls back to a `stub` backend that only emits
  Hello (so we can prove the service is installed and reachable from the PC
  even before a real tracking source is wired).
- `spacesync-framelink.service` — systemd unit, reads config from
  `/etc/default/spacesync-framelink`.
- `spacesync-framelink.default` — the config file, holds `SPACESYNC_HOST=...`
  and a couple of knobs.
- `install.sh` — handles the install, including flipping SteamOS's
  read-only root on/off. Falls back to a user-local install if sudo isn't
  passwordless.

## Install on the Frame

From the PC repo root, over SSH:

```
scp -r contrib/steamframe/service deck@<frame-ip>:/tmp/framelink
ssh deck@<frame-ip> 'bash /tmp/framelink/install.sh <your-pc-lan-ip>'
```

If passwordless sudo isn't configured:

```
ssh deck@<frame-ip> 'bash /tmp/framelink/install.sh <your-pc-lan-ip> --user-local'
```

Then watch the PC-side `spacesync_driver.log` for `FrameLink: hello from
<frame-ip>...`. That alone confirms LAN + UDP + protocol.

## Verify poses are flowing

```
# On the Frame:
journalctl --user -u spacesync-framelink -f     # or without --user for system install
# Expected lines: "using backend: openxr" plus no errors.
```

If the openxr backend fails and the service drops to `stub`, no poses go over
the wire — the service still runs, the driver still logs Hello, calibration
on the PC behaves exactly like today.

## Configure

Edit `/etc/default/spacesync-framelink` (or
`~/.config/spacesync-framelink/env` for user installs):

- `SPACESYNC_HOST` — your PC's LAN IP
- `SPACESYNC_PORT` — default 29485
- `SPACESYNC_HZ` — active pose rate while the head is moving; default 30
- `SPACESYNC_IDLE_HZ` — pose rate once the head has been still ~1 s; default 5
- `SPACESYNC_BACKENDS` — preference list, default `openxr,stub`

## Power budget

The Frame's a 21.6 Wh device, so this service is written to stay well out of
the way of games using the battery:

- **Motion-gated rate.** Active at 30 Hz, drops to 5 Hz when the head stops
  moving for a second. Position threshold 0.5 mm, rotation threshold ~0.5°.
  Flips back to 30 Hz the instant motion resumes.
- **One syscall per tick.** The main loop calls `time.sleep(next_t - now)`
  for a single precise wakeup, not a quartered polling loop — CPU stays
  parked between frames.
- **Low per-packet work.** ~90 bytes of UDP every ~33 ms or ~200 ms.
  The hot path is one `locate_space` + one `sendto`, no allocations.
- **Low scheduling priority.** The systemd unit sets `Nice=5` and
  `CPUWeight=20` so the service never contends with the game.
- **Idle-only staleness guarantee.** The driver's staleness gate wants a
  sample at least every 500 ms; 5 Hz idle gives us a comfortable 5× margin.

Back-of-envelope: at ARM A78 class CPU, Python's event loop at 30 Hz with
one syscall per tick is <0.3 % of one core. Running continuously that's ~15
mW out of a 21.6 Wh battery — call it ~75 h of battery for the daemon alone.
Measured numbers will land here once we have them.

Then `sudo systemctl restart spacesync-framelink` (or `systemctl --user
restart ...`).

## Status

Verified end-to-end on real Steam Frame hardware:

- **Runtime:** SteamVR/OpenXR 2.18.2, HMD codename `cv`, vendor Valve.
- **Backend:** `openxr` with the `XR_MND_headless` extension, which lets
  the service create an `xrSession` without a graphics binding. Monado
  ships this extension and Valve's runtime inherits it.
- **Install:** `pip install --user --break-system-packages pyopenxr`
  (PEP 668 on SteamOS; `--break-system-packages` is the documented
  override). pulls in pyopenxr, glfw, numpy, PyOpenGL.
- **Throughput:** 30 Hz active, 5 Hz idle (motion-gated). Service logs
  `poses sent=N missed=M skipped=K` every 5 s so you can confirm flow
  without tcpdump.
- **Fallback:** if the headless extension isn't advertised (non-Valve,
  non-Monado runtimes) or `pyopenxr` is missing, the backend bails and
  the `stub` takes over — the install still comes up cleanly.

## Next steps

- Capture timestamps are currently "now at send time". For truly
  simultaneous sample pairs we want the runtime's last-measurement
  time, which `locate_space` prediction smooths over. Investigate
  `XR_KHR_locate_spaces` (the runtime advertises it) + a real frame
  loop for `wait_frame`/`begin_frame`/`end_frame` even in headless.
- Linear / angular velocity are zero'd today because `locate_space`
  doesn't deliver them; adding a frame loop would also expose
  `xrSpaceVelocity` via `XrSpaceVelocity` next-chain.
- Gravity direction isn't sourced yet; probably needs a motion-sensor
  extension or heuristic from the first stationary samples.
