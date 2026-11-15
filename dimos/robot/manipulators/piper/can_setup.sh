#!/usr/bin/env bash
# Brings CAN port $1 (e.g. can0) up at 1 Mbit/s for a Piper, unless it is up already. A CANable with
# slcan firmware (a serial port, /dev/ttyACM*) is made into the port with slcand; an adapter with its
# own CAN driver (gs_usb, PEAK) only needs its bitrate set. The commands need root, so they run under sudo.
set -euo pipefail
port=${1:?usage: can_setup.sh <CAN port, e.g. can0>}
# The SDK sends commands in bursts: slcan's default queue of 10 frames drops some.
queue=1000

is_up() {
    [ -e "/sys/class/net/$port" ] &&
        ip link show "$port" | grep -q '[<,]UP[,>]' &&
        [ "$(cat "/sys/class/net/$port/tx_queue_len")" -ge "$queue" ]
}
is_up && exit 0

if [ ! -e "/sys/class/net/$port" ]; then
    serial=$(ls /dev/serial/by-id/*CANable* 2>/dev/null | head -n 1 || true)
    if [ -z "$serial" ]; then
        echo "can_setup.sh: no $port and no CANable under /dev/serial/by-id: is the Piper's USB-CAN adapter plugged in?" >&2
        exit 1
    fi
    if ! command -v slcand >/dev/null; then
        echo "can_setup.sh: slcand is missing: sudo apt install can-utils" >&2
        exit 1
    fi
    # -s8 is 1 Mbit/s.
    sudo slcand -o -c -s8 "$serial" "$port"
    for _ in $(seq 50); do
        [ -e "/sys/class/net/$port" ] && break
        sleep 0.1
    done
elif [ -e "/sys/class/net/$port/device" ]; then
    # An adapter with its own CAN driver; an slcan port has no device and takes no bitrate.
    sudo ip link set "$port" down
    sudo ip link set "$port" type can bitrate 1000000
fi
sudo ip link set "$port" txqueuelen "$queue"
sudo ip link set "$port" up
