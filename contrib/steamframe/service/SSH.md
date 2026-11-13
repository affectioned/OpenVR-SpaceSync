# SSH to the Steam Frame — dev setup

Short version of what we figured out while getting the FrameLink service
on the device. Future devs who want to iterate on SteamOS-side code can
follow these to reach a working `ssh` + `scp` + `journalctl -f` loop
without re-learning the gotchas.

## Enable sshd on the Frame (one-time)

SteamOS ships with OpenSSH installed but `sshd.service` is disabled.
From the Frame itself, in Desktop Mode → Konsole:

```
sudo passwd $USER         # set a login password if you haven't
sudo systemctl enable --now sshd
```

Verify:

```
systemctl status sshd | head -3
ss -tln | grep :22
```

Steam Machine Life has a longer write-up with pictures, same substance:
<https://steammachinelife.com/enable-ssh-on-steam-machine/>

## Find the Frame's IP

- From the Frame, `ip -brief addr show`.
- From the PC, `arp -a` after a `ping -n 4 frame.local`. `frame.local` is
  what SteamOS advertises via mDNS, so the ping also doubles as a sanity
  check that the two boxes can see each other.

## Confirm the default user

The Frame ships with user `steamos`, **not** `deck` like the Steam Deck.
Verify from the Frame:

```
whoami    # -> steamos on Frame, deck on Deck
```

Our systemd unit is written for the `steamos` default; the installer
patches `User=` to whoever invoked it so the Deck case also works
without editing anything.

## Avoid retyping the password — install an SSH key

Password auth is the Frame's `sudo` password, which is fiddly to retype
and a non-starter for scripted deploys. Generate a dedicated key on your
dev box and drop the public half on the Frame:

```powershell
# PC (PowerShell)
ssh-keygen -t ed25519 -N "" -f $env:USERPROFILE\.ssh\framelink

# Copy the key to the Frame (one-time password prompt)
Get-Content $env:USERPROFILE\.ssh\framelink.pub | `
  ssh steamos@<frame-ip> "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```

```bash
# PC (bash/git-bash)
ssh-keygen -t ed25519 -N "" -f ~/.ssh/framelink
ssh-copy-id -i ~/.ssh/framelink.pub steamos@<frame-ip>
```

From then on:

```
ssh -i ~/.ssh/framelink steamos@<frame-ip>
```

Or drop a stanza into `~/.ssh/config` so you don't have to pass `-i`:

```
Host frame
    HostName frame.local
    User steamos
    IdentityFile ~/.ssh/framelink
```

## Sudo over SSH

Our key logs you in as `steamos`. For `sudo`-requiring commands over an
`ssh -t` session (no TTY attached by default), feed the password through
stdin:

```
ssh frame "echo <pw> | sudo -S systemctl restart spacesync-framelink"
```

For heavier automation, a NOPASSWD line scoped to the things you keep
touching is nicer than scattering the password through scripts:

```
# On the Frame, one-time:
sudo install -m 0440 /dev/stdin /etc/sudoers.d/spacesync-framelink <<'EOF'
steamos ALL=(root) NOPASSWD: /usr/bin/systemctl *spacesync-framelink*, \
                             /usr/bin/journalctl *spacesync-framelink*, \
                             /usr/bin/steamos-readonly
EOF
```

## SteamOS read-only root

`/` is immutable on SteamOS. To install anything in `/usr/local/bin` or
`/etc/systemd/system`:

```
sudo steamos-readonly disable
# ...install...
sudo steamos-readonly enable
```

Our `install.sh` handles this with a trap that always re-enables, so a
Ctrl-C mid-install doesn't leave the rootfs writable.

System updates re-flash `/`, which wipes anything in `/usr/local`. If a
Frame update eats the service, re-run the bootstrap one-liner to drop it
back in.

## Running + watching the service

```
systemctl --user restart spacesync-framelink        # user install (default fallback)
sudo systemctl restart spacesync-framelink          # system install
journalctl --user -u spacesync-framelink -f         # user install
sudo journalctl -u spacesync-framelink -f           # system install
```

On the PC side, the service landing is proved by the driver log:

```
tail -F "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\spacesync_driver.log" | Select-String FrameLink
```

With the `stub` backend (the default when pyopenxr isn't installed) the
PC sees one `FrameLink: hello from <frame-ip>` per second and no pose
packets, as expected.

## Copying files to and from the Frame

```
scp -i ~/.ssh/framelink file.py steamos@frame.local:/tmp/
scp -i ~/.ssh/framelink steamos@frame.local:spacesync-framelink-report.txt .
```

Or `sftp`, or WinSCP / FileZilla with Protocol=SFTP and the same key.

## Troubleshooting

- **`Authentication failed`** with password: 99% chance it's a typo in
  the sudo password you set on the Frame. `passwd` doesn't echo. Reset
  with `sudo passwd $USER`, retype carefully, retry.
- **`Permission denied (publickey,password).`**: the key isn't in
  `~/.ssh/authorized_keys` on the Frame, or `~/.ssh` doesn't have `700`
  / `authorized_keys` doesn't have `600`. SSHD rejects loose permissions.
- **Service starts then dies with `status=203/EXEC`**: either the binary
  isn't where the unit says, or systemd hardening (e.g. `ProtectHome=yes`
  with a binary under `/home`) is hiding it from the service's mount
  namespace. We removed those hardening directives from the shared unit
  for exactly this reason.
- **PC's driver log stays empty**: SteamVR isn't up, the SpaceSync
  add-on is disabled, or Windows Firewall is dropping inbound UDP 29485
  on the private profile (rare on default settings).
