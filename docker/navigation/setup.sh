#!/bin/bash
set -e
set -o pipefail

################################################################################
# DimOS Navigation Setup Script
#
# Usage: ./setup.sh [OPTIONS]
#   --install-dir DIR   Installation directory (default: ~/dimos)
#   --skip-docker       Skip Docker installation
#   --skip-build        Skip building Docker images
#   --help              Show this help message
#
################################################################################

# Color codes for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m'
readonly BOLD='\033[1m'

# Configuration
INSTALL_DIR="${HOME}/dimos"
SKIP_DOCKER=false
SKIP_BUILD=false
LOG_FILE="${HOME}/dimos-setup.log"
SCRIPT_START_TIME=$(date +%s)

# Step tracking
CURRENT_STEP=0
TOTAL_STEPS=8

################################################################################
# Utility Functions
################################################################################

log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[${timestamp}] [${level}] ${message}" >> "${LOG_FILE}"
}

print_banner() {
    echo -e "${CYAN}${BOLD}"
    cat << "EOF"
    ____  _ __  ___  ____  _____
   / __ \(_)  |/  / / __ \/ ___/
  / / / / / /|_/ / / / / /\__ \
 / /_/ / / /  / / / /_/ /___/ /
/_____/_/_/  /_/  \____//____/

   Navigation Setup Script
EOF
    echo -e "${NC}"
    echo -e "${BLUE}This script will set up your Ubuntu system for DimOS Navigation${NC}"
    echo -e "${BLUE}Installation may take 20-30 minutes depending on your connection${NC}"
    echo ""
}

step() {
    CURRENT_STEP=$((CURRENT_STEP + 1))
    echo ""
    echo -e "${CYAN}${BOLD}[Step ${CURRENT_STEP}/${TOTAL_STEPS}]${NC} ${BOLD}$1${NC}"
    log "INFO" "Step ${CURRENT_STEP}/${TOTAL_STEPS}: $1"
}

info() {
    echo -e "${BLUE}ℹ${NC} $1"
    log "INFO" "$1"
}

success() {
    echo -e "${GREEN}✓${NC} $1"
    log "SUCCESS" "$1"
}

warning() {
    echo -e "${YELLOW}⚠${NC} $1"
    log "WARNING" "$1"
}

error() {
    echo -e "${RED}✗${NC} $1"
    log "ERROR" "$1"
}

fatal() {
    error "$1"
    echo ""
    echo -e "${RED}${BOLD}Installation failed.${NC}"
    echo -e "Check the log file for details: ${LOG_FILE}"
    echo ""
    exit 1
}

confirm() {
    local prompt="$1"
    local default="${2:-n}"
    local response

    if [[ "${default}" == "y" ]]; then
        prompt="${prompt} [Y/n]: "
    else
        prompt="${prompt} [y/N]: "
    fi

    read -r -p "$(echo -e "${YELLOW}${prompt}${NC}")" response
    response=${response:-${default}}

    [[ "${response,,}" =~ ^y(es)?$ ]]
}

check_command() {
    command -v "$1" >/dev/null 2>&1
}

################################################################################
# Pre-flight Checks
################################################################################

preflight_checks() {
    step "Running pre-flight checks"

    if [[ "$(uname -s)" != "Linux" ]]; then
        fatal "This script is designed for Linux systems only"
    fi

    if ! check_command apt-get; then
        fatal "This script requires Ubuntu or Debian-based system"
    fi

    if [[ -f /etc/os-release ]]; then
        source /etc/os-release
        info "Detected: ${PRETTY_NAME}"

        OS_VERSION_CODENAME="${VERSION_CODENAME:-}"

        VERSION_NUM=$(echo "${VERSION_ID:-0}" | cut -d. -f1)
        if ! [[ "${VERSION_NUM}" =~ ^[0-9]+$ ]]; then
            warning "Unable to determine Ubuntu version number"
            VERSION_NUM=0
        fi

        if [[ "${VERSION_NUM}" -ne 0 ]] && [[ "${VERSION_NUM}" -lt 24 ]]; then
            warning "Ubuntu 24.04 is required. You have ${VERSION_ID}"
            if ! confirm "Continue anyway?"; then
                exit 0
            fi
        fi
    fi

    if [[ $EUID -eq 0 ]]; then
        fatal "This script should NOT be run as root. Run as a regular user with sudo access."
    fi

    if ! sudo -n true 2>/dev/null; then
        info "This script requires sudo access. You may be prompted for your password."
        if ! sudo true; then
            fatal "Failed to obtain sudo access"
        fi
    fi

    local target_dir=$(dirname "${INSTALL_DIR}")
    mkdir -p "${target_dir}" 2>/dev/null || target_dir="${HOME}"
    local available_space=$(df -BG "${target_dir}" 2>/dev/null | awk 'NR==2 {print $4}' | sed 's/G//' || echo "0")
    info "Available disk space at ${target_dir}: ${available_space}GB"
    if [[ "${available_space}" -lt 50 ]]; then
        warning "Low disk space detected. At least 50GB is recommended."
        warning "Docker images and builds will require significant space."
        if ! confirm "Continue anyway?"; then
            exit 0
        fi
    fi

    info "Checking internet connectivity..."
    if ! ping -c 1 8.8.8.8 >/dev/null 2>&1; then
        fatal "No internet connection detected. Please check your network."
    fi

    success "Pre-flight checks passed"
}

################################################################################
# System Setup
################################################################################

update_system() {
    step "Updating system packages"

    info "Running apt-get update..."
    if sudo apt-get update -y >> "${LOG_FILE}" 2>&1; then
        success "Package lists updated"
    else
        warning "Package update had some warnings (check log)"
    fi
}

install_base_tools() {
    step "Installing base tools"

    local packages=(
        "git"
        "ssh"
        "zip"
        "curl"
        "wget"
        "jq"
        "nano"
        "vim"
        "htop"
        "ca-certificates"
        "gnupg"
    )

    info "Installing: ${packages[*]}"

    if sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${packages[@]}" >> "${LOG_FILE}" 2>&1; then
        success "Base tools installed"
    else
        fatal "Failed to install base tools"
    fi

    if check_command ufw; then
        info "Configuring firewall (UFW)..."
        if sudo ufw status | grep -q "Status: active"; then
            info "UFW is active, ensuring SSH is allowed..."
            sudo ufw allow 22/tcp >> "${LOG_FILE}" 2>&1 || true
        else
            info "UFW is inactive, skipping firewall configuration"
        fi
    fi
}

################################################################################
# Docker Installation
################################################################################

install_docker() {
    if [[ "${SKIP_DOCKER}" == true ]]; then
        info "Skipping Docker installation (--skip-docker flag)"
        return
    fi

    step "Installing Docker"

    if check_command docker; then
        local docker_version=$(docker --version 2>/dev/null || echo "unknown")
        success "Docker is already installed: ${docker_version}"

        if docker compose version >/dev/null 2>&1; then
            success "Docker Compose plugin is available"
        else
            warning "Docker Compose plugin not found, will attempt to install"
        fi

        if ! confirm "Reinstall Docker anyway?" "n"; then
            return
        fi
    fi

    info "Adding Docker's official GPG key..."
    sudo install -m 0755 -d /etc/apt/keyrings

    if curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg 2>> "${LOG_FILE}"; then
        sudo chmod a+r /etc/apt/keyrings/docker.gpg
        success "Docker GPG key added"
    else
        fatal "Failed to add Docker GPG key"
    fi

    info "Adding Docker repository..."
    local version_codename="${OS_VERSION_CODENAME}"
    if [[ -z "${version_codename}" ]] && [[ -f /etc/os-release ]]; then
        version_codename=$(. /etc/os-release && echo "$VERSION_CODENAME")
    fi

    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      ${version_codename} stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    success "Docker repository added"

    info "Updating package lists..."
    sudo apt-get update -y >> "${LOG_FILE}" 2>&1

    info "Installing Docker packages (this may take a few minutes)..."
    local docker_packages=(
        "docker-ce"
        "docker-ce-cli"
        "containerd.io"
        "docker-buildx-plugin"
        "docker-compose-plugin"
    )

    if sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${docker_packages[@]}" >> "${LOG_FILE}" 2>&1; then
        success "Docker installed successfully"
    else
        fatal "Failed to install Docker packages"
    fi

    info "Configuring Docker group permissions..."

    if ! getent group docker >/dev/null; then
        sudo groupadd docker
    fi

    if sudo usermod -aG docker "${USER}"; then
        success "User ${USER} added to docker group"
    else
        warning "Failed to add user to docker group"
    fi

    info "Verifying Docker installation..."
    if sudo docker run --rm hello-world >> "${LOG_FILE}" 2>&1; then
        success "Docker is working correctly"
    else
        warning "Docker verification failed, but installation may still be successful"
    fi

    warning "Docker group changes require logout/login to take effect"
    info "For now, we'll use 'sudo docker' commands"
}

################################################################################
# Git LFS Setup
################################################################################

install_git_lfs() {
    step "Installing Git LFS"

    if check_command git-lfs; then
        success "Git LFS is already installed"
        return
    fi

    info "Adding Git LFS repository..."
    if curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash >> "${LOG_FILE}" 2>&1; then
        success "Git LFS repository added"
    else
        fatal "Failed to add Git LFS repository"
    fi

    info "Installing Git LFS..."
    if sudo apt-get install -y git-lfs >> "${LOG_FILE}" 2>&1; then
        success "Git LFS installed"
    else
        fatal "Failed to install Git LFS"
    fi

    info "Configuring Git LFS..."
    if git lfs install >> "${LOG_FILE}" 2>&1; then
        success "Git LFS configured"
    else
        warning "Git LFS configuration had issues (may already be configured)"
    fi
}

################################################################################
# SSH Key Configuration
################################################################################

setup_ssh_keys() {
    step "Configuring GitHub SSH access"

    info "Testing GitHub SSH connection..."
    if timeout 10 ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new -T git@github.com 2>&1 | grep -q "successfully authenticated"; then
        success "GitHub SSH access is already configured"
        return
    fi

    warning "GitHub SSH access is not configured"
    echo ""
    echo -e "${YELLOW}${BOLD}SSH Key Setup Required${NC}"
    echo ""
    echo "To clone the private DimOS repository, you need SSH access to GitHub."
    echo ""

    if [[ -f "${HOME}/.ssh/id_rsa.pub" ]] || [[ -f "${HOME}/.ssh/id_ed25519.pub" ]]; then
        info "Existing SSH key found"
        echo ""

        if [[ -f "${HOME}/.ssh/id_ed25519.pub" ]]; then
            echo -e "${CYAN}Your public key (id_ed25519.pub):${NC}"
            cat "${HOME}/.ssh/id_ed25519.pub"
        elif [[ -f "${HOME}/.ssh/id_rsa.pub" ]]; then
            echo -e "${CYAN}Your public key (id_rsa.pub):${NC}"
            cat "${HOME}/.ssh/id_rsa.pub"
        fi

        echo ""
        echo -e "${YELLOW}Please add this key to your GitHub account:${NC}"
        echo "  1. Go to: https://github.com/settings/keys"
        echo "  2. Click 'New SSH key'"
        echo "  3. Paste the key above"
        echo "  4. Click 'Add SSH key'"
        echo ""
    else
        info "No SSH key found. Let's create one."
        echo ""

        if confirm "Generate a new SSH key?" "y"; then
            local email
            echo -n "Enter your GitHub email address: "
            read -r email

            info "Generating SSH key..."
            if ssh-keygen -t ed25519 -C "${email}" -f "${HOME}/.ssh/id_ed25519" -N "" >> "${LOG_FILE}" 2>&1; then
                success "SSH key generated"

                eval "$(ssh-agent -s)" > /dev/null
                if ssh-add "${HOME}/.ssh/id_ed25519" 2>> "${LOG_FILE}"; then
                    success "SSH key added to agent"
                else
                    warning "Could not add key to ssh-agent (non-critical)"
                fi

                echo ""
                echo -e "${CYAN}Your new public key:${NC}"
                cat "${HOME}/.ssh/id_ed25519.pub"
                echo ""
                echo -e "${YELLOW}Please add this key to your GitHub account:${NC}"
                echo "  1. Go to: https://github.com/settings/keys"
                echo "  2. Click 'New SSH key'"
                echo "  3. Paste the key above"
                echo "  4. Click 'Add SSH key'"
                echo ""
            else
                fatal "Failed to generate SSH key"
            fi
        else
            echo ""
            error "SSH key is required to continue"
            echo "Please set up SSH access manually and run this script again."
            exit 1
        fi
    fi

    echo ""
    if ! confirm "Have you added the SSH key to GitHub?" "n"; then
        echo ""
        warning "Setup paused. Please add the SSH key and run this script again."
        exit 0
    fi

    info "Testing GitHub SSH connection..."
    if timeout 10 ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new -T git@github.com 2>&1 | grep -q "successfully authenticated"; then
        success "GitHub SSH access verified!"
    else
        error "GitHub SSH connection failed"
        echo ""
        echo "Please verify:"
        echo "  1. The SSH key was added to GitHub correctly"
        echo "  2. You're using the correct GitHub account"
        echo "  3. Try: ssh -T git@github.com"
        echo ""
        if ! confirm "Continue anyway?" "n"; then
            exit 1
        fi
    fi
}

################################################################################
# Repository Setup
################################################################################

clone_repository() {
    step "Cloning DimOS repository"

    if [[ -d "${INSTALL_DIR}" ]]; then
        if [[ -d "${INSTALL_DIR}/.git" ]]; then
            success "Repository already exists at ${INSTALL_DIR}"

            local remote_url=$(git -C "${INSTALL_DIR}" remote get-url origin 2>/dev/null || echo "")
            if [[ "${remote_url}" =~ "dimos" ]]; then
                info "Existing repository verified"
                return
            else
                warning "Directory exists but doesn't appear to be the DimOS repo"
                if ! confirm "Remove and re-clone?" "n"; then
                    fatal "Cannot proceed with existing directory"
                fi
                rm -rf "${INSTALL_DIR}"
            fi
        else
            warning "Directory ${INSTALL_DIR} exists but is not a git repository"
            if ! confirm "Remove and re-clone?" "n"; then
                fatal "Cannot proceed with existing directory"
            fi
            rm -rf "${INSTALL_DIR}"
        fi
    fi

    info "Cloning to ${INSTALL_DIR}..."
    if git clone git@github.com:dimensionalOS/dimos.git "${INSTALL_DIR}" >> "${LOG_FILE}" 2>&1; then
        success "Repository cloned successfully"
    else
        fatal "Failed to clone repository. Check your SSH access."
    fi

    info "Pulling Git LFS files (this may take several minutes)..."
    if git -C "${INSTALL_DIR}" lfs pull >> "${LOG_FILE}" 2>&1; then
        success "LFS files downloaded"
    else
        warning "Some LFS files may not have downloaded correctly"
    fi
}

################################################################################
# Build and Launch
################################################################################

build_docker_images() {
    if [[ "${SKIP_BUILD}" == true ]]; then
        info "Skipping Docker build (--skip-build flag)"
        return
    fi

    step "Building Docker images"

    local build_dir="${INSTALL_DIR}/docker/navigation"
    if [[ ! -d "${build_dir}" ]]; then
        fatal "Directory not found: ${build_dir}"
    fi

    if [[ ! -f "${build_dir}/build.sh" ]]; then
        fatal "build.sh not found in ${build_dir}"
    fi

    echo ""
    warning "Building Docker images will take 10-15 minutes and download ~30GB"
    info "This step will:"
    echo "  • Clone the ROS navigation autonomy stack"
    echo "  • Build a large Docker image with ROS Jazzy"
    echo "  • Install all dependencies"
    echo ""

    if ! confirm "Start the build now?" "y"; then
        warning "Build skipped. You can build later with:"
        echo "  cd ${build_dir}"
        echo "  ./build.sh"
        return
    fi

    info "Starting build process..."
    echo ""

    pushd "${build_dir}" >> "${LOG_FILE}" 2>&1 || fatal "Failed to change to ${build_dir}"

    ./build.sh 2>&1 | tee -a "${LOG_FILE}"
    local build_status=${PIPESTATUS[0]}

    popd >> "${LOG_FILE}" 2>&1 || true

    if [[ ${build_status} -eq 0 ]]; then
        success "Docker images built successfully"
    else
        fatal "Docker build failed. Check the log for details."
    fi
}

################################################################################
# Completion
################################################################################

print_summary() {
    local elapsed=$(($(date +%s) - SCRIPT_START_TIME))
    local minutes=$((elapsed / 60))
    local seconds=$((elapsed % 60))

    echo ""
    echo ""
    echo -e "${GREEN}${BOLD}╔══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}${BOLD}║                                                          ║${NC}"
    echo -e "${GREEN}${BOLD}║           Setup completed successfully! 🎉               ║${NC}"
    echo -e "${GREEN}${BOLD}║                                                          ║${NC}"
    echo -e "${GREEN}${BOLD}╚══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}Installation time: ${minutes}m ${seconds}s${NC}"
    echo -e "${CYAN}Installation directory: ${INSTALL_DIR}${NC}"
    echo -e "${CYAN}Log file: ${LOG_FILE}${NC}"
    echo ""
    echo -e "${BOLD}Next steps:${NC}"
    echo ""
    echo "  1. If Docker commands failed, log out and back in for group changes"
    echo "     Or run: newgrp docker"
    echo ""
    echo "  2. Navigate to the project:"
    echo "     cd ${INSTALL_DIR}/docker/navigation"
    echo ""
    echo "  3. Start the demo:"
    echo "     ./start.sh --all"
    echo ""
    echo "  4. Or get an interactive shell:"
    echo "     ./start.sh"
    echo ""
    echo -e "${CYAN}For more information, see the README.md in docker/navigation/${NC}"
    echo ""
}

################################################################################
# Argument Parsing
################################################################################

parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --install-dir)
                if [[ -z "$2" ]] || [[ "$2" == --* ]]; then
                    error "Error: --install-dir requires a directory path"
                    echo "Run '$0 --help' for usage information"
                    exit 1
                fi
                INSTALL_DIR="$2"
                shift 2
                ;;
            --skip-docker)
                SKIP_DOCKER=true
                shift
                ;;
            --skip-build)
                SKIP_BUILD=true
                shift
                ;;
            --help)
                print_banner
                cat << EOF
Usage: $0 [OPTIONS]

Options:
  --install-dir DIR   Installation directory (default: ~/dimos)
  --skip-docker       Skip Docker installation
  --skip-build        Skip building Docker images
  --help              Show this help message

Examples:
  $0                                # Full installation
  $0 --install-dir /opt/dimos       # Install to custom directory
  $0 --skip-docker                  # Skip Docker installation
  $0 --skip-docker --skip-build     # Only clone repository

After installation, navigate to the project and start the demo:
  cd ~/dimos/docker/navigation
  ./start.sh --all

For more information, visit:
  https://github.com/dimensionalOS/dimos

EOF
                exit 0
                ;;
            *)
                error "Unknown option: $1"
                echo "Run '$0 --help' for usage information"
                exit 1
                ;;
        esac
    done
}

################################################################################
# Main
################################################################################

main() {
    log "INFO" "DimOS Navigation Setup Script started"
    log "INFO" "User: ${USER}"
    log "INFO" "Install directory: ${INSTALL_DIR}"

    print_banner

    echo -e "${YELLOW}This script will:${NC}"
    echo "  • Update your system"
    echo "  • Install Docker and dependencies"
    echo "  • Configure Git LFS"
    echo "  • Set up GitHub SSH access"
    echo "  • Clone the DimOS repository"
    echo "  • Build Docker images (~30GB, 10-15 minutes)"
    echo ""

    if ! confirm "Continue with installation?" "y"; then
        echo "Installation cancelled."
        exit 0
    fi

    preflight_checks
    update_system
    install_base_tools
    install_docker
    install_git_lfs
    setup_ssh_keys
    clone_repository
    build_docker_images

    print_summary

    log "INFO" "Setup completed successfully"
}

parse_arguments "$@"
main
