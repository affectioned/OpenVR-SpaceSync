#!/bin/bash
# One-shot bootstrap: pulls the service files down from GitHub and runs the
# installer. Meant to be curl'd straight off the branch:
#
#   curl -sL https://raw.githubusercontent.com/affectioned/OpenVR-SpaceSync/feat/steam-frame-18/contrib/steamframe/service/bootstrap.sh | bash -s -- <PC-IP>
#
# Also drops the PC's install SSH key into ~/.ssh/authorized_keys so updates
# from the PC side work without typing a password each time.

set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: bootstrap.sh <PC-IP>" >&2
    exit 2
fi

PC_IP="$1"
BRANCH="${SPACESYNC_BRANCH:-feat/steam-frame-18}"
RAW="https://raw.githubusercontent.com/affectioned/OpenVR-SpaceSync/${BRANCH}"

STAGE="/tmp/framelink"
mkdir -p "$STAGE"
cd "$STAGE"

echo "[bootstrap] fetching service files from $BRANCH"
for f in spacesync-framelink.py spacesync-framelink.service spacesync-framelink.default install.sh; do
    curl -fsSL "$RAW/contrib/steamframe/service/$f" -o "$f"
done
chmod +x install.sh spacesync-framelink.py

echo "[bootstrap] seeding PC install key"
curl -fsSL "$RAW/seed-framelink-key.sh" | bash || true

echo "[bootstrap] running installer against PC at $PC_IP"
bash ./install.sh "$PC_IP"
