#!/bin/bash
# Install the SpaceSync FrameLink service on a Steam Frame (or any SteamOS /
# generic systemd Linux box).
#
# Steam Frame's rootfs is read-only; /usr/local is part of the writable
# overlay via steamos-readonly disable/enable. We try the overlay path first
# and fall back to ~/.local/bin if it's unavailable.
#
# Usage:
#   ./install.sh <PC-IP>
#
# For now the service won't produce poses until a real tracking backend is
# wired in, but it will at least register with the driver and give us a
# clean install path to iterate on.

set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 <PC-IP> [--user-local]" >&2
    exit 1
fi

PC_IP="$1"
MODE="${2:-auto}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

PY="$SCRIPT_DIR/spacesync-framelink.py"
SVC="$SCRIPT_DIR/spacesync-framelink.service"
DEF="$SCRIPT_DIR/spacesync-framelink.default"

for f in "$PY" "$SVC" "$DEF"; do
    [[ -f "$f" ]] || { echo "missing $f" >&2; exit 1; }
done

readonly_was_enabled=0
undo_readonly() {
    if [[ $readonly_was_enabled -eq 1 ]]; then
        echo "re-enabling SteamOS readonly root"
        sudo steamos-readonly enable || true
    fi
}
trap undo_readonly EXIT

system_install() {
    if command -v steamos-readonly >/dev/null 2>&1; then
        echo "SteamOS detected — disabling readonly root"
        sudo steamos-readonly disable
        readonly_was_enabled=1
    fi

    sudo install -Dm755 "$PY"  /usr/local/bin/spacesync-framelink
    sudo install -Dm644 "$SVC" /etc/systemd/system/spacesync-framelink.service
    # Patch User= to whoever actually invoked this installer (Frame ships
    # with `steamos`, Deck ships with `deck`).
    sudo sed -i "s/^User=.*/User=${USER}/" /etc/systemd/system/spacesync-framelink.service

    if [[ ! -f /etc/default/spacesync-framelink ]]; then
        sudo install -Dm644 "$DEF" /etc/default/spacesync-framelink
        sudo sed -i "s/^SPACESYNC_HOST=.*/SPACESYNC_HOST=${PC_IP}/" /etc/default/spacesync-framelink
    else
        echo "/etc/default/spacesync-framelink exists — leaving config alone"
    fi

    sudo systemctl daemon-reload
    sudo systemctl enable --now spacesync-framelink
    echo "installed. status:"
    systemctl status --no-pager spacesync-framelink || true
}

user_install() {
    mkdir -p "$HOME/.local/bin" "$HOME/.config/systemd/user" "$HOME/.config/spacesync-framelink"
    install -Dm755 "$PY" "$HOME/.local/bin/spacesync-framelink"
    sed -e "s|^EnvironmentFile=.*|EnvironmentFile=-$HOME/.config/spacesync-framelink/env|" \
        -e "s|^ExecStart=.*|ExecStart=$HOME/.local/bin/spacesync-framelink --host \${SPACESYNC_HOST} --port \${SPACESYNC_PORT} --hz \${SPACESYNC_HZ} --backends \${SPACESYNC_BACKENDS}|" \
        -e "/^User=/d" \
        -e "/^WantedBy=/c\\WantedBy=default.target" \
        "$SVC" > "$HOME/.config/systemd/user/spacesync-framelink.service"

    if [[ ! -f "$HOME/.config/spacesync-framelink/env" ]]; then
        cp "$DEF" "$HOME/.config/spacesync-framelink/env"
        sed -i "s/^SPACESYNC_HOST=.*/SPACESYNC_HOST=${PC_IP}/" "$HOME/.config/spacesync-framelink/env"
    fi

    systemctl --user daemon-reload
    systemctl --user enable --now spacesync-framelink
    echo "installed (user mode). status:"
    systemctl --user status --no-pager spacesync-framelink || true
}

case "$MODE" in
    --user-local)
        user_install
        ;;
    auto)
        if sudo -n true 2>/dev/null; then
            system_install
        else
            echo "no passwordless sudo — falling back to user-local install"
            user_install
        fi
        ;;
    *)
        echo "unknown mode: $MODE" >&2; exit 1 ;;
esac
