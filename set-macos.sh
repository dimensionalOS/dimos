#!/bin/bash
# Script to set up DimOS for Mac ARM (Apple Silicon)
set -e

echo "🚀 Setting up DimOS for Mac ARM..."

# ============================================
# 1. Check Prerequisites
# ============================================
if ! command -v brew &> /dev/null; then
    echo "❌ Homebrew not found. Install from: https://brew.sh/"
    exit 1
fi

if ! xcode-select -p &>/dev/null; then
    echo "❌ Xcode Command Line Tools not found."
    echo "Run: xcode-select --install"
    exit 1
fi

# ============================================
# 2. Install System Dependencies
# ============================================
echo "📦 Installing system dependencies via Homebrew..."

# Build chain
brew install cmake pkg-config

# BLAS / LAPACK
brew install openblas lapack gfortran

# Media I/O
brew install ffmpeg libvpx opus libsndfile

# Python package dependencies
brew install jpeg-turbo zlib freetype portaudio fftw

# Git LFS
if ! command -v git-lfs &> /dev/null; then
    echo "Installing Git LFS..."
    brew install git-lfs
fi
git lfs install

# ============================================
# 3. Optional: X11 Support (if needed)
# ============================================
read -p "Do you need X11/GUI support (RViz, Qt5)? [y/N] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Installing XQuartz and related libraries..."
    brew install --cask xquartz
    brew install fontconfig
fi

# ============================================
# 4. Install uv Package Manager
# ============================================
if ! command -v uv &> /dev/null; then
    echo "Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
fi

# ============================================
# 5. Python Environment Setup
# ============================================
echo "🐍 Setting up Python 3.12 environment..."
uv python install 3.12

if [ ! -d ".venv" ]; then
    echo "Creating virtual environment..."
    uv venv --python 3.12
fi

# ============================================
# 6. Configure Build Environment
# ============================================
echo "⚙️  Configuring build environment..."

# Set environment variables for keg-only packages
export LDFLAGS="-L/opt/homebrew/opt/openblas/lib -L/opt/homebrew/opt/lapack/lib -L/opt/homebrew/opt/jpeg-turbo/lib -L/opt/homebrew/opt/zlib/lib $LDFLAGS"
export CPPFLAGS="-I/opt/homebrew/opt/openblas/include -I/opt/homebrew/opt/lapack/include -I/opt/homebrew/opt/jpeg-turbo/include -I/opt/homebrew/opt/zlib/include $CPPFLAGS"
export PKG_CONFIG_PATH="/opt/homebrew/opt/openblas/lib/pkgconfig:/opt/homebrew/opt/lapack/lib/pkgconfig:/opt/homebrew/opt/jpeg-turbo/lib/pkgconfig:/opt/homebrew/opt/zlib/lib/pkgconfig:$PKG_CONFIG_PATH"
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/openblas:/opt/homebrew/opt/lapack:$CMAKE_PREFIX_PATH"

# Activate virtual environment
source .venv/bin/activate

# ============================================
# 7. Install DimOS
# ============================================
echo "📥 Installing DimOS with CPU dependencies..."
uv pip install -e .[cpu,dev]

# ============================================
# 8. Success Message
# ============================================
echo ""
echo "✅ Setup complete!"
echo ""
echo "To activate the environment in future sessions:"
echo "  source .venv/bin/activate"