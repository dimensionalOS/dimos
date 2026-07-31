#!/usr/bin/env bash
# Bind the Galaxea HHS USB-CANFD adapter to Linux SocketCAN and give it a
# stable name. Run after boot or after connecting the adapter.
set -euo pipefail

USB_VENDOR_ID="a8fa"
USB_PRODUCT_ID="8598"
CAN_INTERFACE="a1zcan"
CAN_BITRATE="1000000"
# An empty frame on the maximum extended CAN ID cannot match the A1Z's
# standard-ID motor commands, but it still exercises the USB transmit path.
CAN_PROBE_FRAME="1FFFFFFF#"
CAN_PROBE_ATTEMPTS="20"
CAN_PROBE_POLL_SECONDS="0.05"

read_can_counter() {
    local counter="$1"
    cat "/sys/class/net/$CAN_INTERFACE/statistics/$counter"
}

usb_bulk_out_endpoint() {
    local endpoint

    for endpoint in "$usb_device":*/ep_*; do
        [[ -r "$endpoint/direction" && -r "$endpoint/type" ]] || continue
        [[ "$(<"$endpoint/direction")" == "out" ]] || continue
        [[ "$(<"$endpoint/type")" == "Bulk" ]] || continue
        printf '0x%s' "$(<"$endpoint/bEndpointAddress")"
        return
    done
    printf 'unknown'
}

print_driver_compatibility_error() {
    local tx_packets_before="$1"
    local tx_packets_after="$2"
    local tx_dropped_before="$3"
    local tx_dropped_after="$4"
    local send_error="$5"
    local host_os="unknown Linux distribution"
    local driver_module="unknown"
    local vendor_patch_url="https://galaxea-ai.feishu.cn/docx/XF2ed4pmhoervNxODlfc11Gvnbb"
    local upstream_fix_url="https://github.com/torvalds/linux/commit/889b2ae9139a87b3390f7003cb1bb3d65bf90a26"

    if [[ -r /etc/os-release ]]; then
        host_os="$(. /etc/os-release; printf '%s' "${PRETTY_NAME:-unknown Linux distribution}")"
    fi
    if [[ -e "/sys/class/net/$CAN_INTERFACE/device/driver/module" ]]; then
        driver_module="$(readlink -f "/sys/class/net/$CAN_INTERFACE/device/driver/module")"
    fi

    cat >&2 <<EOF

================================================================================
GALAXEA A1Z CAN SETUP FAILED: THE LINUX DRIVER CANNOT TRANSMIT
================================================================================

Do not start DimOS yet. Changing --can-port, reconnecting to a different Wi-Fi
network, or restarting the blueprint will not fix this failure.

What the test found
-------------------
The HHS USB-CANFD adapter was detected and Linux created '$CAN_INTERFACE', but a
harmless test frame could not be transmitted through the adapter.

  Computer:             $host_os
  CPU architecture:     $(uname -m)
  Kernel:               $(uname -r)
  Loaded driver:        $driver_module
  Adapter USB ID:       $USB_VENDOR_ID:$USB_PRODUCT_ID
  Adapter bulk OUT:     $(usb_bulk_out_endpoint)
  CAN interface:        $CAN_INTERFACE at $CAN_BITRATE bit/s
  TX packets:           $tx_packets_before -> $tx_packets_after
  TX packets dropped:   $tx_dropped_before -> $tx_dropped_after
  Test-send error:      ${send_error:-none reported}

What this means
---------------
Galaxea supplies this arm with an HHS USB-CANFD adapter whose Linux support is
poor. The adapter is not compatible with the 'gs_usb' driver shipped in some
Linux kernels. In the common failure, the adapter transmits on USB endpoint
0x01 while the old driver incorrectly hard-codes endpoint 0x02.

This is especially confusing because the bad driver still creates a normal-
looking CAN interface. 'ip link' therefore says the interface is UP, while
every command sent to the robot is silently dropped. This is a known Galaxea /
HHS driver compatibility defect, not a DimOS control, URDF, or CAN-port error.

How to fix it
-------------
Choose ONE of the following options, then reboot and run this setup script
again. Do not continue until this script prints "A1Z CAN setup passed".

OPTION A — ordinary x86-64 Ubuntu computer (recommended)

  Upgrade to a distribution kernel that includes the corrected gs_usb driver.
  Galaxea currently recommends Linux kernel 6.8.0-124 or newer.

  1. Record the current kernel:
       uname -r
  2. Install all normal OS/kernel updates using the distribution's updater.
  3. Reboot.
  4. Confirm the new kernel is running:
       uname -r
  5. Run this script again:
       sudo ./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z_can.sh

  If the reported kernel is still older than 6.8.0-124, ask the event organizer
  for the supported Ubuntu kernel package instead of guessing at kernel
  packages during the hackathon.

OPTION B — NVIDIA Jetson, a pinned kernel, or a machine that cannot be upgraded

  Do NOT install a generic desktop Ubuntu kernel on a Jetson. The gs_usb module
  must instead be patched for this machine's exact running kernel. Kernel
  modules are kernel- and architecture-specific: never copy a random gs_usb.ko
  from another computer.

  Galaxea's kernel patch guide:
    $vendor_patch_url

  The upstream Linux endpoint-discovery fix:
    $upstream_fix_url

  The patched module must be installed persistently under /lib/modules/\$(uname
  -r), followed by 'sudo depmod -a' and a reboot. Merely running 'insmod' on a
  file under /tmp is temporary and will stop working after the next reboot.

  If you are not comfortable building a kernel module, give this entire error
  report to the event organizer. This is a host-driver installation task; it
  cannot be repaired with a DimOS configuration override.

After applying either option
----------------------------
Run only this command first:

  sudo ./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z_can.sh

When it prints "A1Z CAN setup passed", the stable interface is '$CAN_INTERFACE'
and DimOS can be started without --can-port.
================================================================================
EOF
}

verify_can_transmit() {
    local tx_packets_before
    local tx_packets_after
    local tx_dropped_before
    local tx_dropped_after
    local send_error=""
    local attempt

    if ! command -v cansend >/dev/null 2>&1; then
        cat >&2 <<'EOF'
ERROR: The CAN health check requires the 'cansend' command.

Install the standard Linux CAN utilities, then run this script again:

  Ubuntu / Debian:  sudo apt install can-utils
  Fedora:           sudo dnf install can-utils
  Arch Linux:       sudo pacman -S can-utils

The setup script intentionally stops here because seeing an UP interface is not
enough to prove that the Galaxea adapter can actually transmit.
EOF
        exit 1
    fi

    tx_packets_before="$(read_can_counter tx_packets)"
    tx_dropped_before="$(read_can_counter tx_dropped)"
    if ! send_error="$(cansend "$CAN_INTERFACE" "$CAN_PROBE_FRAME" 2>&1)"; then
        send_error="${send_error:-cansend exited unsuccessfully without an error message}"
    fi
    for ((attempt = 0; attempt < CAN_PROBE_ATTEMPTS; attempt++)); do
        tx_packets_after="$(read_can_counter tx_packets)"
        tx_dropped_after="$(read_can_counter tx_dropped)"
        if ((tx_packets_after > tx_packets_before || tx_dropped_after > tx_dropped_before)); then
            break
        fi
        sleep "$CAN_PROBE_POLL_SECONDS"
    done

    if ((tx_dropped_after > tx_dropped_before)); then
        print_driver_compatibility_error \
            "$tx_packets_before" "$tx_packets_after" \
            "$tx_dropped_before" "$tx_dropped_after" "$send_error"
        exit 2
    fi

    if ((tx_packets_after <= tx_packets_before)); then
        cat >&2 <<EOF

ERROR: '$CAN_INTERFACE' did not complete the CAN transmission health check.

The kernel did not report a dropped USB transmission, so this is not the known
gs_usb endpoint-driver failure. The usual causes are:

  1. The Galaxea arm is not powered on.
  2. The CAN cable is loose or disconnected.
  3. The CAN termination resistor is missing or incorrect.
  4. Another process is already using or repeatedly resetting the adapter.

Check power, cabling, and termination, stop other robot/CAN processes, and run:

  sudo ./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z_can.sh

Diagnostic counters:
  TX packets:         $tx_packets_before -> $tx_packets_after
  TX packets dropped: $tx_dropped_before -> $tx_dropped_after
  Test-send error:    ${send_error:-none reported}

Do not start DimOS until this setup script passes.
EOF
        exit 3
    fi
}

if ((EUID != 0)); then
    echo "Run this script with sudo." >&2
    exit 1
fi

modprobe gs_usb

usb_device=""
for device in /sys/bus/usb/devices/*; do
    [[ -r "$device/idVendor" && -r "$device/idProduct" ]] || continue
    [[ "$(<"$device/idVendor")" == "$USB_VENDOR_ID" ]] || continue
    [[ "$(<"$device/idProduct")" == "$USB_PRODUCT_ID" ]] || continue
    usb_device="$device"
    break
done
[[ -n "$usb_device" ]] || { echo "HHS USB-CANFD adapter not found." >&2; exit 1; }

find_can_interface() {
    for interface in "$usb_device":*/net/*; do
        [[ -e "$interface" ]] || continue
        basename "$interface"
        return
    done
}

can_interface="$(find_can_interface)"
if [[ -z "$can_interface" ]]; then
    printf '%s %s\n' "$USB_VENDOR_ID" "$USB_PRODUCT_ID" \
        > /sys/bus/usb/drivers/gs_usb/new_id
    udevadm settle --timeout=3
    for _ in {1..30}; do
        can_interface="$(find_can_interface)"
        [[ -n "$can_interface" ]] && break
        sleep 0.1
    done
fi
[[ -n "$can_interface" ]] || { echo "gs_usb did not create a CAN interface." >&2; exit 1; }

ip link set "$can_interface" down
if [[ "$can_interface" != "$CAN_INTERFACE" ]]; then
    [[ ! -e "/sys/class/net/$CAN_INTERFACE" ]] || { echo "$CAN_INTERFACE already exists." >&2; exit 1; }
    ip link set "$can_interface" name "$CAN_INTERFACE"
fi
ip link set "$CAN_INTERFACE" type can bitrate "$CAN_BITRATE"
ip link set "$CAN_INTERFACE" up
verify_can_transmit

echo "A1Z CAN setup passed: '$CAN_INTERFACE' transmitted successfully at $CAN_BITRATE bit/s."
ip -details link show "$CAN_INTERFACE"
