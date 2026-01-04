#!/bin/bash
# Setup script for LCM Lua bindings
# Clones official LCM repo and builds Lua bindings

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LCM_DIR="$SCRIPT_DIR/lcm"

echo "=== LCM Lua Setup ==="

# Clone LCM if not present
if [ ! -d "$LCM_DIR" ]; then
    echo "Cloning LCM..."
    git clone --depth 1 https://github.com/lcm-proj/lcm.git "$LCM_DIR"
else
    echo "LCM already cloned"
fi

# Build Lua bindings using cmake
echo "Building LCM Lua bindings..."
cd "$LCM_DIR"
mkdir -p build && cd build

# Configure with Lua support (using system compiler, not nix)
CC=/usr/bin/gcc CXX=/usr/bin/g++ cmake .. \
    -DLCM_ENABLE_LUA=ON \
    -DLCM_ENABLE_PYTHON=OFF \
    -DLCM_ENABLE_JAVA=OFF \
    -DLCM_ENABLE_TESTS=OFF \
    -DLCM_ENABLE_EXAMPLES=OFF \
    -DLUA_INCLUDE_DIR=/usr/include \
    -DLUA_LIBRARY=/usr/lib/liblua.so

# Build just the lua target
make lcm-lua -j4

# Install the lua module
LUA_CPATH_DIR="/usr/local/lib/lua/5.4"
echo "Installing lcm.so to $LUA_CPATH_DIR"
sudo mkdir -p "$LUA_CPATH_DIR"
sudo cp lcm-lua/lcm.so "$LUA_CPATH_DIR/"

# Get dimos-lcm message definitions
DIMOS_LCM_DIR="$SCRIPT_DIR/dimos-lcm"
MSGS_DST="$SCRIPT_DIR/msgs"

echo "Getting message definitions..."
if [ -d "$DIMOS_LCM_DIR" ]; then
    echo "Updating dimos-lcm..."
    cd "$DIMOS_LCM_DIR" && git pull
else
    echo "Cloning dimos-lcm..."
    git clone --depth 1 https://github.com/dimensionalOS/dimos-lcm.git "$DIMOS_LCM_DIR"
fi

# Link/copy messages
rm -rf "$MSGS_DST"
cp -r "$DIMOS_LCM_DIR/generated/lua_lcm_msgs" "$MSGS_DST"
echo "Messages installed to $MSGS_DST"

echo ""
echo "=== Setup complete ==="
echo "Run: lua main.lua"
