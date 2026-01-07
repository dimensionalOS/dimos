#!/bin/bash
# Setup script for LCM Lua bindings
# Clones official LCM repo and builds Lua bindings
#
# Tested on: Arch Linux, Ubuntu, macOS (with Homebrew)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LCM_DIR="$SCRIPT_DIR/lcm"

echo "=== LCM Lua Setup ==="

# Detect Lua version
if command -v lua &>/dev/null; then
    LUA_VERSION=$(lua -v 2>&1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
    echo "Detected Lua version: $LUA_VERSION"
else
    echo "Error: lua not found in PATH"
    exit 1
fi

# Detect Lua paths using pkg-config if available
if command -v pkg-config &>/dev/null && pkg-config --exists "lua$LUA_VERSION" 2>/dev/null; then
    LUA_INCLUDE_DIR=$(pkg-config --variable=includedir "lua$LUA_VERSION")
    LUA_LIBRARY=$(pkg-config --libs "lua$LUA_VERSION" | grep -oE '/[^ ]+\.so' | head -1 || echo "")
elif command -v pkg-config &>/dev/null && pkg-config --exists lua 2>/dev/null; then
    LUA_INCLUDE_DIR=$(pkg-config --variable=includedir lua)
    LUA_LIBRARY=$(pkg-config --libs lua | grep -oE '/[^ ]+\.so' | head -1 || echo "")
fi

# Platform-specific defaults
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS with Homebrew
    LUA_INCLUDE_DIR="${LUA_INCLUDE_DIR:-$(brew --prefix lua 2>/dev/null)/include/lua}"
    LUA_LIBRARY="${LUA_LIBRARY:-$(brew --prefix lua 2>/dev/null)/lib/liblua.dylib}"
    LUA_CPATH_BASE="${LUA_CPATH_BASE:-/usr/local/lib/lua}"
else
    # Linux defaults
    LUA_INCLUDE_DIR="${LUA_INCLUDE_DIR:-/usr/include}"
    LUA_LIBRARY="${LUA_LIBRARY:-/usr/lib/liblua.so}"
    LUA_CPATH_BASE="${LUA_CPATH_BASE:-/usr/local/lib/lua}"
fi

echo "Lua include: $LUA_INCLUDE_DIR"
echo "Lua library: $LUA_LIBRARY"

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

# Configure with Lua support
cmake .. \
    -DLCM_ENABLE_LUA=ON \
    -DLCM_ENABLE_PYTHON=OFF \
    -DLCM_ENABLE_JAVA=OFF \
    -DLCM_ENABLE_TESTS=OFF \
    -DLCM_ENABLE_EXAMPLES=OFF \
    -DLUA_INCLUDE_DIR="$LUA_INCLUDE_DIR" \
    -DLUA_LIBRARY="$LUA_LIBRARY"

# Build just the lua target
make lcm-lua -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Install the lua module
LUA_CPATH_DIR="$LUA_CPATH_BASE/$LUA_VERSION"
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
