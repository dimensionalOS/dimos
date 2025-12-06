#!/bin/bash
# Configure desktop background image
# This script sets the desktop wallpaper for the default user (ubuntu)
# Sets both picture-uri and picture-uri-dark for light and dark mode support

set -e

DESKTOP_BACKGROUND_PATH="/usr/share/backgrounds/dimos-grub-4096.png"

# Verify the background image exists
if [ ! -f "$DESKTOP_BACKGROUND_PATH" ]; then
    echo "Error: Background image not found at $DESKTOP_BACKGROUND_PATH" >&2
    exit 1
fi

# Configure dconf system database for system-wide background settings
# This ensures the background is set for all users and persists across sessions
# The dconf system will auto-read and refresh on boot if configured properly
DCONF_PROFILE="/etc/dconf/profile/user"
DCONF_DB_DIR="/etc/dconf/db/local.d"
DCONF_KEY_FILE="${DCONF_DB_DIR}/00-background"

# Create dconf profile if it doesn't exist
if [ ! -f "$DCONF_PROFILE" ]; then
    mkdir -p "$(dirname "$DCONF_PROFILE")"
    cat > "$DCONF_PROFILE" <<EOF
user-db:user
system-db:local
EOF
fi

# Create dconf database directory if it doesn't exist
mkdir -p "$DCONF_DB_DIR"

# Create key file with background settings
cat > "$DCONF_KEY_FILE" <<EOF
[org/gnome/desktop/background]
picture-uri='file://${DESKTOP_BACKGROUND_PATH}'
picture-uri-dark='file://${DESKTOP_BACKGROUND_PATH}'
EOF

# Update dconf system databases
if command -v dconf >/dev/null 2>&1; then
    dconf update
else
    echo "Warning: dconf command not found, skipping dconf update" >&2
fi

# Set both picture-uri and picture-uri-dark using gsettings
# This ensures the background works in both light and dark mode
if command -v gsettings >/dev/null 2>&1; then
    gsettings set org.gnome.desktop.background picture-uri "file://${DESKTOP_BACKGROUND_PATH}"
    gsettings set org.gnome.desktop.background picture-uri-dark "file://${DESKTOP_BACKGROUND_PATH}"
else
    echo "Warning: gsettings command not found, skipping gsettings configuration" >&2
fi
