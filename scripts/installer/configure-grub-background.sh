#!/bin/bash
# Configure GRUB background image
# This script removes any existing GRUB_BACKGROUND setting and adds the correct one

set -e

GRUB_BACKGROUND_PATH="/usr/share/backgrounds/dimos-grub-4096.png"

# Remove any existing GRUB_BACKGROUND line
sed -i '/^GRUB_BACKGROUND=/d' /etc/default/grub
echo "GRUB_BACKGROUND=${GRUB_BACKGROUND_PATH}" >> /etc/default/grub
# Remove any existing GRUB_TIMEOUT line and set boot delay to 3 seconds
sed -i '/^GRUB_TIMEOUT=/d' /etc/default/grub
echo "GRUB_TIMEOUT=3" >> /etc/default/grub

# Update-grub doesn't work in the chroot.
# update-grub
