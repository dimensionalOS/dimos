#!/bin/bash

set -euo pipefail

readonly COLOR_RESET='\033[0m'
readonly COLOR_RED='\033[0;31m'
readonly COLOR_GREEN='\033[0;32m'
readonly COLOR_YELLOW='\033[0;33m'
readonly COLOR_BLUE='\033[0;34m'
readonly COLOR_CYAN='\033[0;36m'
readonly COLOR_BOLD='\033[1m'

log_info() {
  echo -e "${COLOR_BLUE}${COLOR_BOLD}==>${COLOR_RESET} ${COLOR_BOLD}$*${COLOR_RESET}"
}

log_success() {
  echo -e "${COLOR_GREEN}${COLOR_BOLD}✓${COLOR_RESET} ${COLOR_GREEN}$*${COLOR_RESET}"
}

log_warning() {
  echo -e "${COLOR_YELLOW}${COLOR_BOLD}⚠${COLOR_RESET} ${COLOR_YELLOW}$*${COLOR_RESET}"
}

log_error() {
  echo -e "${COLOR_RED}${COLOR_BOLD}✗${COLOR_RESET} ${COLOR_RED}$*${COLOR_RESET}" >&2
}

log_prompt() {
  echo -e "${COLOR_CYAN}${COLOR_BOLD}?${COLOR_RESET} ${COLOR_CYAN}$*${COLOR_RESET}"
}

# Check Ubuntu version
IS_COMPATIBLE=false
if [ -f /etc/os-release ]; then
  . /etc/os-release
  if [[ "$ID" == "ubuntu" ]]; then
    UBUNTU_VERSION=$(echo "$VERSION_ID" | cut -d. -f1)
    if [[ "$UBUNTU_VERSION" == "22" || "$UBUNTU_VERSION" == "24" ]]; then
      IS_COMPATIBLE=true
      log_success "Detected Ubuntu $VERSION_ID - compatible version"
    fi
  fi
fi

if [[ "$IS_COMPATIBLE" == "false" ]]; then
  log_warning "This script is designed for Ubuntu 22.04 or 24.04. It might not work correctly on your system."
  read -p "Do you want to continue anyway? (y/N) " -n 1 -r
  echo
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    log_error "Installation cancelled"
    exit 1
  fi
fi

log_info "Installing system dependencies..."

sudo apt update

ubuntu_packages=(
  build-essential # Development tools
  git-lfs # For LFS.
  libturbojpeg0-dev # For fast encoding JPEG in transports
  portaudio19-dev
  python3-pyaudio
  python3-venv # For the virtual environment
)

sudo apt install -y "${ubuntu_packages[@]}"
log_success "System dependencies installed"

log_info "Installing git lfs..."
git lfs install
log_success "Git LFS installed"

log_info "Creating the python virtual environment..."
python3 -m venv venv
source venv/bin/activate
log_success "Python virtual environment created and activated"

log_info "Installing python dependencies..."
pip install -e .[cpu,dev,sim] 'mmengine>=0.10.3' 'mmcv>=2.1.0'
log_success "Python dependencies installed"

log_info "Downloading MuJoCo Menagerie..."
python -m mujoco_playground || true
log_success "MuJoCo Menagerie downloaded"

log_info "Copying default .env file..."
cp default.env .env
log_success "Default .env file copied"

# Check if Foxglove is installed and ask user if they want to install it
if ! command -v foxglove-studio >/dev/null 2>&1; then
  log_warning "Foxglove Studio is not installed"
  log_prompt "Would you like to install Foxglove Studio? (y/N)"
  read -p "" -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    log_info "Downloading Foxglove Studio..."
    wget https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-amd64.deb
    log_info "Installing Foxglove Studio..."
    sudo apt install -y ./foxglove-studio-*.deb
    rm ./foxglove-studio-*.deb
    log_success "Foxglove Studio installed"
  else
    log_info "Skipping Foxglove Studio installation"
  fi
else
  log_success "Foxglove Studio is already installed"
fi

log_success "Repository bootstrap complete!"
log_info "To activate the virtual environment in the future, run: source venv/bin/activate"
