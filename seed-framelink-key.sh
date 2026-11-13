#!/bin/bash
# Seeds the ed25519 key used by SpaceSync FrameLink install from PC.
# Safe to run multiple times. Public key only — no secret material.
mkdir -p ~/.ssh && chmod 700 ~/.ssh
KEY='ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIA8l/wDd2r911mEUkO76xagMuNif+PnGMnJxo24zkw44 spacesync-framelink-install'
grep -qxF "$KEY" ~/.ssh/authorized_keys 2>/dev/null || echo "$KEY" >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
echo "OK — PC install key added"
