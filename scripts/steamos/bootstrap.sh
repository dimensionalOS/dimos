#!/usr/bin/env bash
# SteamOS bootstrap from a usb stick: password, sshd, ssh key, hostname. Run in Konsole as deck.
# Put this directory on the stick with your ssh public key next to it as `key`.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
sudo -n true 2>/dev/null || { echo "new password for $USER:"; passwd; }
sudo -v
sudo systemctl enable --now sshd
if [ -f "$HERE/key" ]; then
  mkdir -p ~/.ssh && chmod 700 ~/.ssh
  grep -qxF "$(cat "$HERE/key")" ~/.ssh/authorized_keys 2>/dev/null || cat "$HERE/key" >> ~/.ssh/authorized_keys
  chmod 600 ~/.ssh/authorized_keys
fi
id=$(cat /sys/class/net/wl*/address 2>/dev/null | head -1 | tr -d : | tail -c 5)
sudo hostnamectl set-hostname "deck-${id:-$RANDOM}"
echo "READY $(cat /etc/hostname) $(ip -4 -br addr show scope global | awk '{print $3}' | head -1)"
