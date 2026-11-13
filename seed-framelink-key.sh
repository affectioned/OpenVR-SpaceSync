#!/bin/bash
# Seeds the ed25519 key used by SpaceSync FrameLink install from PC, and
# dumps a diagnostic report to ~/spacesync-framelink-report.txt so the PC
# side can scp it back for inspection. Safe to run multiple times.

mkdir -p ~/.ssh && chmod 700 ~/.ssh
KEY='ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIA8l/wDd2r911mEUkO76xagMuNif+PnGMnJxo24zkw44 spacesync-framelink-install'
grep -qxF "$KEY" ~/.ssh/authorized_keys 2>/dev/null || echo "$KEY" >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys

REPORT="$HOME/spacesync-framelink-report.txt"
{
    echo "=== spacesync-framelink diagnostic report ==="
    echo "generated: $(date -Iseconds)"
    echo
    echo "--- identity ---"
    echo "whoami : $(whoami)"
    echo "hostname: $(hostname)"
    echo "uname  : $(uname -a)"
    if [[ -f /etc/os-release ]]; then
        echo "os     :"; sed 's/^/    /' /etc/os-release
    fi
    echo
    echo "--- network ---"
    ip -brief addr show 2>/dev/null || ifconfig 2>/dev/null | head -20
    echo
    echo "--- ssh ---"
    echo "authorized_keys lines:"; wc -l ~/.ssh/authorized_keys 2>/dev/null
    echo "sshd listening:"; ss -tln 2>/dev/null | grep -E ':22\b' || echo "  (none)"
    echo
    echo "--- spacesync-framelink service state ---"
    echo "system unit file:"; ls -la /etc/systemd/system/spacesync-framelink.service 2>/dev/null || echo "  (absent)"
    echo "user unit file:";   ls -la ~/.config/systemd/user/spacesync-framelink.service 2>/dev/null || echo "  (absent)"
    echo "binary (/usr/local/bin):"; ls -la /usr/local/bin/spacesync-framelink 2>/dev/null || echo "  (absent)"
    echo "binary (~/.local/bin):";   ls -la ~/.local/bin/spacesync-framelink 2>/dev/null || echo "  (absent)"
    echo "defaults (system):"; ls -la /etc/default/spacesync-framelink 2>/dev/null || echo "  (absent)"
    echo "config (user):";   ls -la ~/.config/spacesync-framelink/env 2>/dev/null || echo "  (absent)"
    echo
    echo "--- systemctl status (system) ---"
    systemctl status --no-pager spacesync-framelink 2>&1 | head -30 || true
    echo
    echo "--- systemctl status (user) ---"
    systemctl --user status --no-pager spacesync-framelink 2>&1 | head -30 || true
    echo
    echo "--- journal (last 40 lines, system then user) ---"
    journalctl -u spacesync-framelink -n 40 --no-pager 2>&1 | tail -40 || true
    echo "- - -"
    journalctl --user -u spacesync-framelink -n 40 --no-pager 2>&1 | tail -40 || true
    echo
    echo "--- pyopenxr available? ---"
    python3 -c "import xr; print('xr version:', getattr(xr, '__version__', '?'))" 2>&1 || echo "pyopenxr not importable"
    echo
    echo "--- what talks to the PC? ---"
    echo "SPACESYNC_HOST configured (system): $(grep ^SPACESYNC_HOST /etc/default/spacesync-framelink 2>/dev/null || echo 'absent')"
    echo "SPACESYNC_HOST configured (user):   $(grep ^SPACESYNC_HOST ~/.config/spacesync-framelink/env 2>/dev/null || echo 'absent')"
    echo
    echo "--- recent bootstrap output (if we ran it to /tmp/framelink) ---"
    ls -la /tmp/framelink 2>/dev/null || echo "  (no /tmp/framelink)"
    echo
    echo "=== end of report ==="
} > "$REPORT" 2>&1

echo "OK — PC install key added"
echo "diagnostic report written to: $REPORT"
