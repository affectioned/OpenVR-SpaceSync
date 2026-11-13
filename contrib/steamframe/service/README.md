# SpaceSync FrameLink service (Steam Frame side)

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

## What's still unknown

The `openxr` backend assumes pyopenxr is available and that a headless
OpenXR session can be created on the Frame without a graphics binding. This
may or may not work on Valve's runtime — the `stub` fallback means an
install always comes up cleanly even if OpenXR rejects us, and we iterate
from there.

Alternatives to try once we see what the Frame ships with:

- Monado IPC directly (`monado-cli`, `monado-service`).
- A SteamVR Linux driver API (Valve's streaming stack must have one).
- Compositor-side taps via `LD_PRELOAD` and `openxr_overlay` -style hooks.

Those are each much bigger projects than the stub, so getting a clean
install path landed first unblocks everything else.
