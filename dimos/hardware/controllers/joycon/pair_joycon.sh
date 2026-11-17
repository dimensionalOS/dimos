#!/usr/bin/env bash
# Pairs and connects Nintendo Joy-Con $1 (L or R) with this computer over Bluetooth, forgetting any
# old pairing first. The first run also asks to let bluez accept Joy-Cons and lets this user read
# them; both need root, so they run under sudo. Run it once per Joy-Con; after that it reconnects when
# woken by any button.
set -euo pipefail
side=${1:?usage: pair_joycon.sh <L or R>}
case $side in
    L | R) ;;
    *) echo "pair_joycon.sh: give L or R, not $side" >&2; exit 1 ;;
esac
name="Joy-Con ($side)"
input_conf=/etc/bluetooth/input.conf

# bluez refuses a Joy-Con's input connection unless ClassicBondedOnly is off. That setting covers
# every classic Bluetooth input device on this computer, so ask first.
if ! grep -q '^ClassicBondedOnly=false' "$input_conf"; then
    echo "Joy-Cons need ClassicBondedOnly=false in $input_conf. It lets any classic Bluetooth"
    echo "keyboard, mouse or controller connect without bonding first, not only Joy-Cons."
    read -rp "Change it and restart Bluetooth? [y/N] " answer
    case $answer in
        y | Y) ;;
        *) echo "pair_joycon.sh: left unchanged, so the Joy-Con cannot connect" >&2; exit 1 ;;
    esac
    if grep -q 'ClassicBondedOnly=' "$input_conf"; then
        sudo sed -i 's/^#\?ClassicBondedOnly=.*/ClassicBondedOnly=false/' "$input_conf"
    else
        sudo sed -i '/^\[General\]/a ClassicBondedOnly=false' "$input_conf"
    fi
    sudo systemctl restart bluetooth
    sleep 2
fi
# The Joy-Con's /dev/input/event* device belongs to the input group.
if ! id -nG "$USER" | grep -qw input; then
    sudo usermod -aG input "$USER"
    echo "Added $USER to the input group: log in again (or run 'newgrp input') before dimos run."
fi

find_mac() { bluetoothctl devices | awk -v n="$name" 'index($0, n) {print $2}' | head -n 1; }

old=$(find_mac)
if [ -n "$old" ]; then
    bluetoothctl remove "$old" >/dev/null || true
fi

read -rp "Press the sync button on $name once (its lights start running), then press Enter "
bluetoothctl --timeout 20 scan on >/dev/null &
scan=$!
mac=""
for _ in $(seq 20); do
    sleep 1
    mac=$(find_mac)
    [ -n "$mac" ] && break
done
kill "$scan" 2>/dev/null || true
if [ -z "$mac" ]; then
    echo "pair_joycon.sh: $name not found: press its sync button and run again" >&2
    exit 1
fi

echo "Found $name at $mac"
bluetoothctl --timeout 15 pair "$mac" || true
bluetoothctl trust "$mac" || true
bluetoothctl --timeout 15 connect "$mac" || true
for _ in $(seq 10); do
    if grep -qF "Name=\"$name\"" /proc/bus/input/devices; then
        echo "$name is connected."
        exit 0
    fi
    sleep 0.5
done
echo "pair_joycon.sh: $name paired but not connected: press its sync button, then: bluetoothctl connect $mac" >&2
exit 1
