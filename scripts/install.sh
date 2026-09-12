#!/usr/bin/env bash
# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0
#
# Interactive installer for DimOS — the agentive operating system for generalist robotics.
#
# Usage:
#   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
#   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --help
#
# Non-interactive:
#   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --non-interactive --mode library --project-dir ./dimos-app --capabilities navigation
#
# Prompts read /dev/tty explicitly. Parse the entire script before main runs so
# child processes cannot consume the script when invoked through curl | bash.
{
set -euo pipefail
trap 'exit 130' INT
trap 'exit 143' TERM

INSTALLER_VERSION="0.4.0"

# package lists (edit these when dependencies change)
UBUNTU_PACKAGES="ca-certificates curl git g++ portaudio19-dev git-lfs libturbojpeg pre-commit libgl1 libegl1 libglib2.0-0 ffmpeg libsndfile1 pkg-config"
MACOS_PACKAGES="gnu-sed gcc portaudio git-lfs libjpeg-turbo pre-commit ffmpeg libsndfile pkg-config"

INSTALL_MODE="${DIMOS_INSTALL_MODE:-}"
CAPABILITIES="${DIMOS_CAPABILITIES:-}"
EXTRAS=""
BACKEND="cpu"
NON_INTERACTIVE="${DIMOS_NO_PROMPT:-0}"
GIT_BRANCH="${DIMOS_BRANCH:-main}"
NO_CUDA="${DIMOS_NO_CUDA:-0}"
CONFIGURE_NETWORK="${DIMOS_CONFIGURE_NETWORK:-0}"
DRY_RUN="${DIMOS_DRY_RUN:-0}"
PROJECT_DIR="${DIMOS_PROJECT_DIR:-}"
VERBOSE=0
USE_NIX="${DIMOS_USE_NIX:-0}"
NO_NIX="${DIMOS_NO_NIX:-0}"
HAS_NIX=0
SETUP_METHOD=""
INSTALL_DIR=""
INSTALL_PYTHON="3.12"
GUM=""
NEEDED_PACKAGES=()
CHILD_PID=""

if [[ -t 1 ]] && command -v tput &>/dev/null && [[ $(tput colors 2>/dev/null || echo 0) -ge 8 ]]; then
    CYAN=$'\033[38;5;44m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; RED=$'\033[31m'
    BOLD=$'\033[1m'; DIM=$'\033[2m'; RESET=$'\033[0m'
else
    CYAN="" GREEN="" YELLOW="" RED="" BOLD="" DIM="" RESET=""
fi

info()  { printf "%s▸%s %s\n" "$CYAN" "$RESET" "$*"; }
ok()    { printf "%s✓%s %s\n" "$GREEN" "$RESET" "$*"; }
warn()  { printf "%s⚠%s %s\n" "$YELLOW" "$RESET" "$*" >&2; }
err()   { printf "%s✗%s %s\n" "$RED" "$RESET" "$*" >&2; }
die()   { err "$@"; exit 1; }
# Cancelled exit code — used by prompt functions to signal Ctrl+C
readonly CANCELLED_EXIT=130
dim()   { printf "%s%s%s\n" "$DIM" "$*" "$RESET"; }

run_cmd() {
    if [[ "$DRY_RUN" == "1" ]]; then dim "[dry-run] $*"; return 0; fi
    [[ "$VERBOSE" == "1" ]] && dim "$ $*"
    "$@"
}

# Execute argv in the selected environment without interpolating paths into code.
project_cmd() (
    if [[ "$DRY_RUN" == "1" ]]; then
        dim "[dry-run] in $INSTALL_DIR: $*"
        return
    fi
    cd "$INSTALL_DIR" || exit
    if [[ "$USE_NIX" == "1" ]]; then
        exec nix develop --command "$@"
    else
        exec "$@"
    fi
)

has_cmd() { command -v "$1" &>/dev/null; }

# prompt wrappers (gum with fallback)

prompt_select() {
    local msg="$1"; shift
    local -a options=("$@")
    [[ "$NON_INTERACTIVE" != 1 ]] || die "--mode must be specified"
    printf "\n" >/dev/tty
    if [[ -n "$GUM" ]]; then
        local tmpf; tmpf=$(mktemp)
        local ec=0
        "$GUM" choose --header "$msg" \
            --cursor "● " --cursor.foreground="44" \
            --header.foreground="255" --header.bold \
            --selected.foreground="44" \
            "${options[@]}" </dev/tty >"$tmpf" || ec=$?
        PROMPT_RESULT=$(<"$tmpf"); rm -f "$tmpf"
        if [[ $ec -ne 0 ]]; then die "cancelled"; fi
    else
        printf "%s%s%s\n" "$BOLD" "$msg" "$RESET" >/dev/tty
        local i=1
        for opt in "${options[@]}"; do
            printf "  %s%d)%s %s\n" "$CYAN" "$i" "$RESET" "$opt" >/dev/tty
            ((i++))
        done
        printf "  choice [1]: " >/dev/tty
        local choice; read -r choice </dev/tty || die "cancelled"
        choice="${choice:-1}"
        [[ "$choice" =~ ^[0-9]+$ ]] || die "enter a menu number"
        local idx=$((10#$choice - 1))
        if [[ $idx -ge 0 ]] && [[ $idx -lt ${#options[@]} ]]; then
            PROMPT_RESULT="${options[$idx]}"
        else
            die "enter a valid menu number"
        fi
    fi
}

prompt_multi() {
    local msg="$1"; shift
    local -a options=("$@")
    [[ "$NON_INTERACTIVE" != 1 ]] || die "capabilities must be specified"
    printf "\n" >/dev/tty
    if [[ -n "$GUM" ]]; then
        local tmpf; tmpf=$(mktemp)
        local ec=0
        "$GUM" choose --no-limit --selected="" --header "$msg  (space to toggle, enter to confirm)" \
            --cursor "❯ " --cursor.foreground="44" \
            --header.foreground="255" --header.bold \
            --selected.foreground="44" \
            "${options[@]}" </dev/tty >"$tmpf" || ec=$?
        PROMPT_RESULT=$(<"$tmpf"); rm -f "$tmpf"
        if [[ $ec -ne 0 ]]; then die "cancelled"; fi
    else
        printf "%s%s%s (comma-separated; choose at least one)\n" "$BOLD" "$msg" "$RESET" >/dev/tty
        local i=1
        for opt in "${options[@]}"; do
            printf "  %s%d)%s %s\n" "$CYAN" "$i" "$RESET" "$opt" >/dev/tty
            ((i++))
        done
        printf "  selection: " >/dev/tty
        local sel; read -r sel </dev/tty || die "cancelled"
        if [[ -z "$sel" ]]; then
            die "select navigation, manipulation, or both"
        else
            local out=""
            IFS=',' read -ra nums <<< "$sel"
            for n in "${nums[@]}"; do
                n="${n// /}"
                [[ "$n" =~ ^[0-9]+$ ]] || die "enter comma-separated menu numbers"
                local idx=$((10#$n - 1))
                if [[ $idx -ge 0 ]] && [[ $idx -lt ${#options[@]} ]]; then
                    [[ -n "$out" ]] && out+=$'\n'
                    out+="${options[$idx]}"
                else
                    die "invalid selection: $n"
                fi
            done
            PROMPT_RESULT="$out"
        fi
    fi
}

prompt_confirm() {
    local msg="$1" default="${2:-yes}"
    if [[ "$NON_INTERACTIVE" == "1" ]]; then [[ "$default" == "yes" ]]; return; fi
    if [[ -n "$GUM" ]]; then
        local flag; [[ "$default" == "yes" ]] && flag="--default=yes" || flag="--default=no"
        "$GUM" confirm "$msg" $flag --prompt.foreground="44" --selected.background="44" </dev/tty
        local ec=$?
        # gum confirm: 0=yes, 1=no, 130=ctrl+c
        [[ $ec -eq 130 ]] && { printf "\n" >/dev/tty; die "cancelled"; }
        return $ec
    else
        local yn
        if [[ "$default" == "yes" ]]; then printf "%s [Y/n] " "$msg" >/dev/tty
        else printf "%s [y/N] " "$msg" >/dev/tty; fi
        read -r yn </dev/tty || die "cancelled"
        yn="${yn:-$([ "$default" == "yes" ] && echo "y" || echo "n")}"
        [[ "$yn" =~ ^[Yy] ]]
    fi
}

# ascii banner
show_banner() {
    if [[ "$NON_INTERACTIVE" == "1" ]] && [[ -z "${DIMOS_SHOW_BANNER:-}" ]]; then return; fi
    # stty </dev/tty works when stdin is a pipe (curl | bash), tput needs a real stdin
    local cols
    cols=$(stty size </dev/tty 2>/dev/null | awk '{print $2}') \
        || cols=$(tput cols 2>/dev/null) \
        || cols=80

    local banner
    if [[ $cols -ge 90 ]]; then
        banner='   ▇▇▇▇▇▇╗ ▇▇╗▇▇▇╗   ▇▇▇╗▇▇▇▇▇▇▇╗▇▇▇╗   ▇▇╗▇▇▇▇▇▇▇╗▇▇╗ ▇▇▇▇▇▇╗ ▇▇▇╗   ▇▇╗ ▇▇▇▇▇╗ ▇▇╗
   ▇▇╔══▇▇╗▇▇║▇▇▇▇╗ ▇▇▇▇║▇▇╔════╝▇▇▇▇╗  ▇▇║▇▇╔════╝▇▇║▇▇╔═══▇▇╗▇▇▇▇╗  ▇▇║▇▇╔══▇▇╗▇▇║
   ▇▇║  ▇▇║▇▇║▇▇╔▇▇▇▇╔▇▇║▇▇▇▇▇╗  ▇▇╔▇▇╗ ▇▇║▇▇▇▇▇▇▇╗▇▇║▇▇║   ▇▇║▇▇╔▇▇╗ ▇▇║▇▇▇▇▇▇▇║▇▇║
   ▇▇║  ▇▇║▇▇║▇▇║╚▇▇╔╝▇▇║▇▇╔══╝  ▇▇║╚▇▇╗▇▇║╚════▇▇║▇▇║▇▇║   ▇▇║▇▇║╚▇▇╗▇▇║▇▇╔══▇▇║▇▇║
   ▇▇▇▇▇▇╔╝▇▇║▇▇║ ╚═╝ ▇▇║▇▇▇▇▇▇▇╗▇▇║ ╚▇▇▇▇║▇▇▇▇▇▇▇║▇▇║╚▇▇▇▇▇▇╔╝▇▇║ ╚▇▇▇▇║▇▇║  ▇▇║▇▇▇▇▇▇▇╗
   ╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝'
    elif [[ $cols -ge 45 ]]; then
        banner='  ▇▇▇▇▇▇╗ ▇▇╗▇▇▇╗   ▇▇▇╗ ▇▇▇▇▇▇╗ ▇▇▇▇▇▇▇╗
  ▇▇╔══▇▇╗▇▇║▇▇▇▇╗ ▇▇▇▇║▇▇╔═══▇▇╗▇▇╔════╝
  ▇▇║  ▇▇║▇▇║▇▇╔▇▇▇▇╔▇▇║▇▇║   ▇▇║▇▇▇▇▇▇▇╗
  ▇▇║  ▇▇║▇▇║▇▇║╚▇▇╔╝▇▇║▇▇║   ▇▇║╚════▇▇║
  ▇▇▇▇▇▇╔╝▇▇║▇▇║ ╚═╝ ▇▇║╚▇▇▇▇▇▇╔╝▇▇▇▇▇▇▇║
  ╚═════╝ ╚═╝╚═╝     ╚═╝ ╚═════╝ ╚══════╝'
    else
        printf "\n  %s%sDimOS Installer%s v%s\n\n" "$CYAN" "$BOLD" "$RESET" "$INSTALLER_VERSION"
        return
    fi
    if [[ -n "$GUM" ]]; then
        printf "\n"
        "$GUM" style --foreground 44 --bold "$banner"
        printf "\n"
        "$GUM" style --faint "   the agentive operating system for generalist robotics  ·  installer v${INSTALLER_VERSION}"
        printf "\n"
    else
        printf "\n"
        while IFS= read -r line; do printf "%s%s%s\n" "$CYAN" "$line" "$RESET"; done <<< "$banner"
        printf "\n   %sthe agentive operating system for generalist robotics%s\n" "$DIM" "$RESET"
        printf "   %sinstaller v%s%s\n\n" "$DIM" "$INSTALLER_VERSION" "$RESET"
    fi
}

# argument parsing
usage() {
    cat <<EOF
DimOS Installer v${INSTALLER_VERSION}

Usage: bash scripts/install.sh [OPTIONS]

    --mode library|dev       Published package or contributor checkout
    --project-dir <path>     Installation directory (existing environments reused)
    --capabilities <list>    navigation, manipulation, or navigation,manipulation
    --non-interactive        Require mode, directory, capabilities; never prompt
    --branch <branch>        Branch to clone for a new checkout (default: main)
    --no-cuda                Use CPU dependencies instead of detected CUDA
    --configure-network      Apply and persist LCM UDP buffer tuning (Linux)
    --use-nix                Use Nix instead of the platform package manager
    --no-nix                 Use apt/brew, or preinstalled dependencies on other Linux
    --dry-run                Preview without changes; requires the same explicit choices
    --verbose                Show commands
    --help                   Show this help

Interactive: choose mode, directory, and capabilities, then confirm once.
Both capabilities include agents, perception, visualization, and simulation.
Developer mode also installs contributor test/lint dependencies.
Verification checks dependencies and CLI; it does not start robots or download models.

Non-interactive example:
    bash scripts/install.sh --non-interactive --mode dev --project-dir . --capabilities navigation,manipulation

Environment equivalents (flags override):
    DIMOS_INSTALL_MODE, DIMOS_PROJECT_DIR, DIMOS_CAPABILITIES, DIMOS_BRANCH
    DIMOS_NO_PROMPT, DIMOS_NO_CUDA, DIMOS_CONFIGURE_NETWORK,
    DIMOS_USE_NIX, DIMOS_NO_NIX, DIMOS_DRY_RUN (boolean values: 0 or 1)
EOF
    exit 0
}

validate_capabilities() {
    case "$CAPABILITIES" in
        navigation|manipulation|navigation,manipulation) ;;
        manipulation,navigation) CAPABILITIES="navigation,manipulation" ;;
        *) die "select --capabilities navigation, manipulation, or navigation,manipulation" ;;
    esac
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --mode|--capabilities|--branch|--project-dir)
                [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 requires a value"
                ;;
        esac
        case "$1" in
            --mode)              INSTALL_MODE="$2"; shift 2 ;;
            --capabilities)      CAPABILITIES="$2"; shift 2 ;;
            --branch)            GIT_BRANCH="$2"; shift 2 ;;
            --project-dir)       PROJECT_DIR="$2"; shift 2 ;;
            --non-interactive)   NON_INTERACTIVE=1; shift ;;
            --no-cuda)           NO_CUDA=1; shift ;;
            --configure-network) CONFIGURE_NETWORK=1; shift ;;
            --use-nix)           USE_NIX=1; shift ;;
            --no-nix)            NO_NIX=1; shift ;;
            --dry-run)           DRY_RUN=1; shift ;;
            --verbose)           VERBOSE=1; shift ;;
            --help|-h)           usage ;;
            *)                   die "unknown option: $1" ;;
        esac
    done
    local value
    for value in "$NON_INTERACTIVE" "$NO_CUDA" "$CONFIGURE_NETWORK" "$USE_NIX" "$NO_NIX" "$DRY_RUN"; do
        [[ "$value" == 0 || "$value" == 1 ]] || die "boolean options must be 0 or 1"
    done
    case "$INSTALL_MODE" in ""|library|dev) ;; *) die "invalid mode: $INSTALL_MODE";; esac
    [[ "$USE_NIX" != 1 || "$NO_NIX" != 1 ]] || die "--use-nix and --no-nix cannot be combined"
    if [[ "$DRY_RUN" == 1 ]]; then NON_INTERACTIVE=1; fi
    if [[ "$NON_INTERACTIVE" == 1 ]]; then
        [[ -n "$INSTALL_MODE" ]] || die "--non-interactive requires --mode library|dev"
        [[ -n "$PROJECT_DIR" ]] || die "--non-interactive requires --project-dir"
        validate_capabilities
    elif [[ -n "$CAPABILITIES" ]]; then
        validate_capabilities
    fi
}

# detection
DETECTED_OS="" DETECTED_OS_VERSION="" DETECTED_ARCH=""
DETECTED_GPU="" DETECTED_CUDA=""
DETECTED_PYTHON_VER=""
DETECTED_RAM_GB=0 DETECTED_DISK_GB=0

detect_os() {
    DETECTED_ARCH="$(uname -m)"
    local uname_s; uname_s="$(uname -s)"
    if [[ "$uname_s" == "Darwin" ]]; then
        DETECTED_OS="macos"
        DETECTED_OS_VERSION="$(sw_vers -productVersion 2>/dev/null || echo "unknown")"
    elif [[ "$uname_s" == "Linux" ]]; then
        if grep -qi microsoft /proc/version 2>/dev/null; then DETECTED_OS="wsl"
        elif [[ -f /etc/NIXOS ]] || has_cmd nixos-version; then DETECTED_OS="nixos"
        elif grep -qEi 'debian|ubuntu' /etc/os-release 2>/dev/null; then DETECTED_OS="ubuntu"
        else DETECTED_OS="linux"; fi
        DETECTED_OS_VERSION="$(. /etc/os-release 2>/dev/null && echo "${VERSION_ID:-unknown}" || echo "unknown")"
    else
        die "unsupported operating system: $uname_s"
    fi
    if [[ "$uname_s" == "Darwin" ]]; then
        DETECTED_RAM_GB=$(( $(sysctl -n hw.memsize 2>/dev/null || echo 0) / 1073741824 ))
        DETECTED_DISK_GB=$(df -g "${HOME}" 2>/dev/null | awk 'NR==2 {print $4}' || echo 0)
    else
        DETECTED_RAM_GB=$(( $(grep MemTotal /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0) / 1048576 ))
        DETECTED_DISK_GB=$(df -BG "${HOME}" 2>/dev/null | awk 'NR==2 {gsub(/G/,"",$4); print $4}' || echo 0)
    fi
}

detect_gpu() {
    if [[ "$DETECTED_OS" == "macos" ]]; then
        [[ "$DETECTED_ARCH" == "arm64" ]] && DETECTED_GPU="apple-silicon" || DETECTED_GPU="none"
    elif has_cmd nvidia-smi && nvidia-smi --query-gpu=name --format=csv,noheader >/dev/null 2>&1; then
        DETECTED_GPU="nvidia"
        DETECTED_CUDA="$(nvidia-smi 2>/dev/null | grep -oP 'CUDA Version: \K[0-9.]+' || echo "")"
    else
        DETECTED_GPU="none"
    fi
}

detect_python() {
    for cmd in python3.12 python3.11 python3.10 python3; do
        if has_cmd "$cmd"; then
            local ver; ver="$("$cmd" --version 2>&1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1 || echo "")"
            if [[ -n "$ver" ]]; then
                local major minor; major="$(echo "$ver" | cut -d. -f1)"; minor="$(echo "$ver" | cut -d. -f2)"
                if [[ "$major" -eq 3 ]] && [[ "$minor" -ge 10 && "$minor" -lt 13 ]]; then
                    DETECTED_PYTHON_VER="$ver"; return
                fi
            fi
        fi
    done
    DETECTED_PYTHON_VER=""
}

detect_nix() {
    if has_cmd nix; then HAS_NIX=1
    elif [[ -f /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh ]]; then
        . /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh 2>/dev/null || true
        if has_cmd nix; then HAS_NIX=1; fi
    fi
}

print_sysinfo() {
    printf "\n"; info "detecting system..."; printf "\n"
    local os_display gpu_display python_display nix_display
    case "$DETECTED_OS" in
        ubuntu) os_display="Ubuntu ${DETECTED_OS_VERSION} (${DETECTED_ARCH})" ;;
        macos)  os_display="macOS ${DETECTED_OS_VERSION} (${DETECTED_ARCH})" ;;
        nixos)  os_display="NixOS ${DETECTED_OS_VERSION} (${DETECTED_ARCH})" ;;
        wsl)    os_display="WSL2 / Ubuntu ${DETECTED_OS_VERSION} (${DETECTED_ARCH})" ;;
        linux)
            local distro_name
            distro_name="$(. /etc/os-release 2>/dev/null && echo "${PRETTY_NAME:-Linux}" || echo "Linux")"
            os_display="${distro_name} (${DETECTED_ARCH})" ;;
        *)      os_display="Unknown" ;;
    esac
    case "$DETECTED_GPU" in
        nvidia)
            local gpu_name; gpu_name="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo "NVIDIA GPU")"
            gpu_display="${gpu_name} (CUDA ${DETECTED_CUDA})" ;;
        apple-silicon) gpu_display="Apple Silicon (Metal/MPS)" ;;
        none)          gpu_display="CPU only" ;;
    esac
    [[ -n "$DETECTED_PYTHON_VER" ]] && python_display="$DETECTED_PYTHON_VER" || python_display="${YELLOW}not found (uv will install 3.12)${RESET}"
    [[ "$HAS_NIX" == "1" ]] && nix_display="${GREEN}$(nix --version 2>/dev/null | head -1)${RESET}" || nix_display="not installed"

    printf "  %sOS:%s       %s\n" "$DIM" "$RESET" "$os_display"
    printf "  %sPython:%s   %s\n" "$DIM" "$RESET" "$python_display"
    printf "  %sGPU:%s      %s\n" "$DIM" "$RESET" "$gpu_display"
    printf "  %sNix:%s      %s\n" "$DIM" "$RESET" "$nix_display"
    printf "  %sRAM:%s      %s GB\n" "$DIM" "$RESET" "$DETECTED_RAM_GB"
    printf "  %sDisk:%s     %s GB free\n" "$DIM" "$RESET" "$DETECTED_DISK_GB"
    printf "\n"

}

# nix support
require_admin() {
    [[ "$DRY_RUN" == 1 || $(id -u) == 0 ]] && return
    has_cmd sudo || die "administrator access required for $*; install prerequisites as administrator, then rerun"
    if [[ "$NON_INTERACTIVE" == 1 ]]; then
        sudo -n -v 2>/dev/null || die "administrator access required for $*; run sudo -v in a terminal or provision prerequisites first, then rerun"
    else
        sudo -v
    fi
}

run_privileged() {
    if [[ $(id -u) == 0 ]]; then
        run_cmd "$@"
    else
        # require_admin handles authentication; individual commands never prompt.
        run_cmd sudo -n "$@"
    fi
}

install_nix() {
    info "installing Nix via the official multi-user installer..."
    if [[ "$DRY_RUN" == 1 ]]; then
        dim "[dry-run] install Nix with --daemon --yes; enable flakes"
        return
    fi
    require_admin "Nix installation"
    local installer
    installer=$(curl --proto '=https' --tlsv1.2 -fsSL https://nixos.org/nix/install)
    # Run the bootstrap with privileges once, avoiding nested sudo password prompts.
    run_privileged sh -c "$installer" -- --daemon --yes </dev/null
    detect_nix
    has_cmd nix || die "Nix installation failed; install Nix from https://nixos.org/download/ and rerun"
    mkdir -p "$HOME/.config/nix"
    if ! grep -q "experimental-features.*flakes" "$HOME/.config/nix/nix.conf" 2>/dev/null; then
        echo "experimental-features = nix-command flakes" >> "$HOME/.config/nix/nix.conf"
    fi
    ok "Nix installed"
}

select_setup_method() {
    if [[ "$NO_NIX" == 1 ]]; then
        case "$DETECTED_OS" in
            ubuntu|wsl|macos) SETUP_METHOD="system" ;;
            *) SETUP_METHOD="manual" ;;
        esac
    elif [[ "$USE_NIX" == 1 || "$DETECTED_OS" == nixos || "$DETECTED_OS" == linux ]]; then
        SETUP_METHOD="nix"
        USE_NIX=1
    else
        SETUP_METHOD="system"
    fi
    [[ "$DETECTED_OS" != nixos || "$HAS_NIX" == 1 || "$NO_NIX" == 1 ]] || die "NixOS detected but nix is unavailable; restore Nix before installing"
}

verify_nix_develop() {
    info "verifying nix develop environment..."
    if [[ "$DRY_RUN" == 1 ]]; then return; fi
    # A shell hook may print setup messages before the command's final line.
    INSTALL_PYTHON=$(project_cmd sh -c 'command -v gcc >/dev/null && python3 -c "import sys; print(sys.executable)"' | tail -n 1) || die "nix develop verification failed"
    [[ "$INSTALL_PYTHON" == /nix/store/* ]] || die "Nix setup must provide its own Python"
}

# system dependencies
find_system_packages() {
    NEEDED_PACKAGES=()
    [[ "$SETUP_METHOD" == system ]] || return 0
    local pkg
    case "$DETECTED_OS" in
        ubuntu|wsl)
            for pkg in $UBUNTU_PACKAGES; do
                if [[ "$(dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null || true)" != "install ok installed" ]]; then
                    NEEDED_PACKAGES+=("$pkg")
                fi
            done ;;
        macos)
            if ! has_cmd brew; then
                case "$DETECTED_ARCH" in
                    arm64) export PATH="/opt/homebrew/bin:$PATH" ;;
                    *) export PATH="/usr/local/bin:$PATH" ;;
                esac
            fi
            for pkg in $MACOS_PACKAGES; do
                if ! has_cmd brew || ! brew list --versions "$pkg" >/dev/null 2>&1; then
                    NEEDED_PACKAGES+=("$pkg")
                fi
            done ;;
    esac
}

install_system_deps() {
    if [[ "$SETUP_METHOD" == nix ]]; then
        if [[ "$HAS_NIX" != 1 ]]; then install_nix; fi
        return
    fi
    if [[ "$SETUP_METHOD" == manual ]]; then
        info "using preinstalled system dependencies; verification will check native libraries"
        return
    fi
    case "$DETECTED_OS" in
        ubuntu|wsl)
            if [[ ${#NEEDED_PACKAGES[@]} -gt 0 ]]; then
                require_admin "apt packages: ${NEEDED_PACKAGES[*]}"
                run_privileged apt-get update
                run_privileged /usr/bin/env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get install -y "${NEEDED_PACKAGES[@]}"
            fi ;;
        macos)
            if ! has_cmd brew; then
                require_admin "Homebrew installation"
                if [[ "$DRY_RUN" == 1 ]]; then
                    dim "[dry-run] install Homebrew"
                else
                    local installer
                    installer=$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)
                    /usr/bin/env -u INTERACTIVE -u SUDO_ASKPASS NONINTERACTIVE=1 /bin/bash -c "$installer" </dev/null
                fi
            fi
            if [[ ${#NEEDED_PACKAGES[@]} -gt 0 ]]; then
                run_cmd brew install "${NEEDED_PACKAGES[@]}"
            fi ;;
    esac
    ok "system dependencies ready"
}

install_uv() {
    local version=""
    if has_cmd uv; then version=$(uv --version | awk '{print $2}'); fi
    if [[ -n "$version" ]] && awk -v version="$version" 'BEGIN {
        split(version, v, "."); exit !(v[1] > 0 || v[2] > 9 || (v[2] == 9 && v[3] >= 25))
    }'; then
        ok "uv already installed ($version)"
        return
    fi
    info "installing uv >=0.9.25..."
    if [[ "$DRY_RUN" == 1 ]]; then dim "[dry-run] install uv"; return; fi
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
    hash -r
    has_cmd uv || die "uv installation failed — install manually: https://docs.astral.sh/uv/"
    ok "uv installed ($(uv --version))"
}

# installation choices
prompt_install_mode() {
    [[ -n "$INSTALL_MODE" ]] && return
    local choice
    prompt_select "How do you want to use DimOS?" \
        "Library — pip install into your project (recommended)" \
        "Developer — git clone + editable install (contributors)"
    choice="$PROMPT_RESULT"
    case "$choice" in *Library*) INSTALL_MODE="library";; *) INSTALL_MODE="dev";; esac
}

prompt_capabilities() {
    [[ -z "$CAPABILITIES" ]] || return 0
    prompt_multi "What will you use DimOS for?" \
        "Navigation — Unitree, mapping, drones" \
        "Manipulation — arm control and planning"
    local line
    while IFS= read -r line; do
        case "$line" in
            Navigation*) CAPABILITIES="${CAPABILITIES:+$CAPABILITIES,}navigation" ;;
            Manipulation*) CAPABILITIES="${CAPABILITIES:+$CAPABILITIES,}manipulation" ;;
        esac
    done <<< "$PROMPT_RESULT"
    validate_capabilities
}

prompt_install_dir() {
    local default="$1" mode="$2"
    [[ "$NON_INTERACTIVE" != 1 ]] || die "--project-dir must be specified"

    local hint
    [[ "$mode" == "dev" ]] && hint="git clone destination" || hint="project directory"

    if [[ -n "$GUM" ]]; then
        local result
        result=$("$GUM" input --header "Where should we install DimOS? (${hint})" \
            --placeholder "$default" --value "$default" \
            --header.foreground="255" --header.bold --cursor.foreground="44" </dev/tty) || { printf "\n" >/dev/tty; exit $CANCELLED_EXIT; }
        [[ -z "$result" ]] && result="$default"
        echo "$result"
    else
        printf "\n%sWhere should we install DimOS?%s (%s)\n" "$BOLD" "$RESET" "$hint" >/dev/tty
        printf "  path [%s]: " "$default" >/dev/tty
        local result
        read -r result </dev/tty || exit "$CANCELLED_EXIT"
        [[ -z "$result" ]] && result="$default"
        echo "$result"
    fi
}

# installation
resolve_capabilities() {
    validate_capabilities
    EXTRAS=""
    if [[ ",$CAPABILITIES," == *,navigation,* ]]; then EXTRAS="unitree,sim,drone"; fi
    if [[ ",$CAPABILITIES," == *,manipulation,* ]]; then EXTRAS="${EXTRAS:+$EXTRAS,}manipulation"; fi
    BACKEND="cpu"
    if [[ "$NO_CUDA" != 1 && "$DETECTED_GPU" == nvidia && "$DETECTED_ARCH" == x86_64 && "$DETECTED_OS" != macos ]]; then
        BACKEND="cuda"
    fi
    EXTRAS="$EXTRAS,$BACKEND"
}

prepare_directory() {
    if [[ -z "$PROJECT_DIR" ]]; then
        local default="$PWD/dimensional-applications"
        if [[ "$INSTALL_MODE" == dev ]]; then default="$PWD/dimos"; fi
        PROJECT_DIR=$(prompt_install_dir "$default" "$INSTALL_MODE") || die "cancelled"
    fi
    case "$PROJECT_DIR" in /*) ;; *) PROJECT_DIR="$PWD/$PROJECT_DIR" ;; esac
    if [[ -d "$PROJECT_DIR" ]]; then
        PROJECT_DIR=$(cd "$PROJECT_DIR" && pwd -P)
    elif [[ -e "$PROJECT_DIR" ]]; then
        die "project directory is not a directory: $PROJECT_DIR"
    fi
    INSTALL_DIR="$PROJECT_DIR"
    if [[ "$INSTALL_MODE" == dev && -d "$PROJECT_DIR" ]]; then
        if [[ -e "$PROJECT_DIR/.git" ]]; then
            git -C "$PROJECT_DIR" rev-parse --is-inside-work-tree >/dev/null || die "invalid Git checkout: $PROJECT_DIR"
            [[ -f "$PROJECT_DIR/pyproject.toml" && -f "$PROJECT_DIR/uv.lock" && -f "$PROJECT_DIR/dimos/__init__.py" ]] || die "expected a DimOS checkout at $PROJECT_DIR"
        elif [[ -n "$(ls -A "$PROJECT_DIR")" ]]; then
            die "developer destination must be an existing DimOS checkout or an empty directory: $PROJECT_DIR"
        fi
    fi
}

check_disk_space() {
    local dir="$INSTALL_DIR"
    while [[ ! -d "$dir" ]]; do dir=$(dirname "$dir"); done
    local available
    available=$(df -Pk "$dir" | awk 'NR == 2 {print $4}')
    if [[ "$available" =~ ^[0-9]+$ && "$available" -lt 10485760 ]]; then
        warn "less than 10GB free at $dir"
        [[ "$DRY_RUN" == 1 ]] || die "free at least 10GB on the destination filesystem before installing"
    fi
}

check_existing_environment() {
    [[ -e "$INSTALL_DIR/.venv" || -L "$INSTALL_DIR/.venv" ]] || return 0
    [[ -x "$INSTALL_DIR/.venv/bin/python" ]] || die "existing .venv has no usable Python; choose a different --project-dir or repair it explicitly"
    "$INSTALL_DIR/.venv/bin/python" -c 'import sys; raise SystemExit(sys.version_info[:2] != (3, 12))' || \
        die "existing .venv must use Python 3.12; choose a different --project-dir to preserve it"
    if [[ "$USE_NIX" == 1 ]]; then
        "$INSTALL_DIR/.venv/bin/python" -c 'import os, sys; raise SystemExit(not os.path.realpath(sys.executable).startswith("/nix/store/"))' || \
            die "existing .venv was not created with Nix Python; choose a different --project-dir for Nix setup"
    fi
}

print_install_summary() {
    info "installation summary"
    printf '  Mode: %s\n  Directory: %s\n  Capabilities: %s\n  Backend: %s\n  System setup: %s\n' \
        "$INSTALL_MODE" "$INSTALL_DIR" "$CAPABILITIES" "$BACKEND" "$SETUP_METHOD"
    if [[ "$INSTALL_MODE" == dev ]]; then dim "  Includes contributor test/lint dependencies"; fi
    if [[ -e "$INSTALL_DIR/.git" ]]; then dim "  Reuse existing checkout; keep its current branch"; fi
    if [[ -d "$INSTALL_DIR/.venv" ]]; then dim "  Reuse existing virtual environment"; fi
    if [[ "$SETUP_METHOD" == nix && "$HAS_NIX" != 1 ]]; then dim "  Install Nix (multi-user) and enable flakes"; fi
    if [[ "$SETUP_METHOD" == system && "$DETECTED_OS" == macos ]] && ! has_cmd brew; then dim "  Install Homebrew"; fi
    if [[ ${#NEEDED_PACKAGES[@]} -gt 0 ]]; then dim "  Required packages: ${NEEDED_PACKAGES[*]}"; fi
    dim "  Ensure uv >=0.9.25 and Python 3.12 are available"
    if [[ "$CONFIGURE_NETWORK" == 1 ]]; then dim "  Apply and persist LCM UDP buffer tuning"; fi
}

do_install_library() {
    local dir="${PROJECT_DIR:-}"
    INSTALL_DIR="$dir"
    info "library install → $dir"
    run_cmd mkdir -p "$dir"
    if [[ "$USE_NIX" == 1 ]]; then
        local base="https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/$GIT_BRANCH"
        local file
        for file in flake.nix flake.lock; do
            if [[ ! -f "$dir/$file" ]]; then run_cmd curl -fsSL "$base/$file" -o "$dir/$file"; fi
        done
        if [[ ! -e "$dir/.git" ]]; then run_cmd git -C "$dir" init -q; fi
        run_cmd git -C "$dir" add flake.nix flake.lock
        verify_nix_develop
    fi
    if [[ -d "$dir/.venv" ]]; then
        info "keeping existing .venv"
    else
        project_cmd uv venv --python "$INSTALL_PYTHON"
    fi
    local backend=cpu
    if [[ ",$EXTRAS," == *,cuda,* ]]; then backend=cu128; fi
    project_cmd uv pip install --python .venv/bin/python --torch-backend "$backend" "dimos[$EXTRAS]"
    ok "dimos installed in $dir"
}

do_install_dev() {
    local dir="${PROJECT_DIR:-}"
    INSTALL_DIR="$dir"
    info "developer install → $dir"
    if [[ -e "$dir/.git" ]]; then
        git -C "$dir" rev-parse --is-inside-work-tree >/dev/null || die "invalid Git checkout: $dir"
        info "using existing checkout at $(git -C "$dir" rev-parse --short HEAD)"
    else
        run_cmd /usr/bin/env GIT_LFS_SKIP_SMUDGE=1 git clone -b "$GIT_BRANCH" https://github.com/dimensionalOS/dimos.git "$dir"
    fi
    if [[ "$USE_NIX" == 1 ]]; then verify_nix_develop; fi
    local python="$INSTALL_PYTHON"
    if [[ -x "$dir/.venv/bin/python" ]]; then python="$dir/.venv/bin/python"; fi
    local -a sync_args=(--locked --python "$python" --group tests --group lint)
    local -a extras=()
    local extra
    IFS=',' read -r -a extras <<< "$EXTRAS"
    for extra in "${extras[@]}"; do sync_args+=(--extra "$extra"); done
    dim "will run: uv sync ${sync_args[*]}"
    project_cmd /usr/bin/env UV_PROJECT_ENVIRONMENT=.venv uv sync "${sync_args[@]}"
    ok "developer environment ready in $dir"
}

do_install() {
    case "$INSTALL_MODE" in library) do_install_library;; dev) do_install_dev;; *) die "invalid mode: $INSTALL_MODE";; esac
}

# system configuration
configure_system() {
    [[ "$CONFIGURE_NETWORK" == 1 ]] || return 0
    [[ "$DETECTED_OS" != macos ]] || die "--configure-network is supported only on Linux"
    if [[ "$DETECTED_OS" == nixos ]]; then
        die "set networking.kernel.sysctl net.core.rmem_max and net.core.rmem_default to 67108864 in configuration.nix, then rerun without --configure-network"
    fi
    require_admin "LCM network configuration"
    run_privileged sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864
    if [[ "$DRY_RUN" == 1 ]]; then
        dim "[dry-run] write /etc/sysctl.d/99-dimos.conf"
    else
        printf '# DimOS LCM transport buffers\nnet.core.rmem_max=67108864\nnet.core.rmem_default=67108864\n' | \
            run_privileged tee /etc/sysctl.d/99-dimos.conf >/dev/null
    fi
    ok "LCM buffers configured and persisted"
}

# verification
# Python's timeout works on macOS too. Each command owns a process group so
# timeout and interruption also stop workers started by a blueprint.
run_bounded() {
    info "checking (timeout ${1}s): ${*:2}"
    project_cmd .venv/bin/python - "$@" <<'PYTHON' &
import os
import signal
import subprocess
import sys

seconds = float(sys.argv[1])
process = subprocess.Popen(sys.argv[2:], start_new_session=True)

def interrupted(signum, frame):
    raise SystemExit(128 + signum)

signal.signal(signal.SIGINT, interrupted)
signal.signal(signal.SIGTERM, interrupted)
try:
    try:
        status = process.wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        status = 124
finally:
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        pass
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()
sys.exit(status)
PYTHON
    CHILD_PID=$!
    local status=0
    wait "$CHILD_PID" || status=$?
    CHILD_PID=""
    return "$status"
}

check_install() {
    local label="$1"; shift
    run_bounded 60 "$@" || die "verification failed: $label"
    ok "$label"
}

verify_install() {
    [[ "$DRY_RUN" != 1 ]] || return 0
    info "verifying installation..."
    check_install "CLI help" .venv/bin/dimos --help
    check_install "blueprint listing" .venv/bin/dimos list
    check_install "native libraries" .venv/bin/python -c 'import sqlite3, cv2, open3d; from turbojpeg import TurboJPEG; TurboJPEG()'
    check_install "shared capability dependencies" .venv/bin/python -c 'import langchain, fastapi, transformers, chromadb, onnxruntime, mujoco, rerun'
    if [[ ",$CAPABILITIES," == *,navigation,* ]]; then
        check_install "navigation dependencies" .venv/bin/python -c 'import gtsam; from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection; from pymavlink import mavutil'
    fi
    if [[ ",$CAPABILITIES," == *,manipulation,* ]]; then
        check_install "manipulation dependencies" .venv/bin/python -c 'import roboplan.core, roboplan.rrt, roboplan.toppra, viser; from xarm.wrapper import XArmAPI'
    fi
    if [[ "$BACKEND" == cuda ]]; then
        check_install "PyTorch CUDA" .venv/bin/python -c 'import torch; assert torch.cuda.is_available(); assert (torch.ones(1, device="cuda") + 1).item() == 2'
    else
        check_install "PyTorch CPU" .venv/bin/python -c 'import torch; assert (torch.ones(1, device="cpu") + 1).item() == 2'
    fi
    ok "installation verified"
}

print_quickstart() {
    if [[ "$DRY_RUN" == 1 ]]; then info "dry-run complete; no installation performed"; return; fi
    printf '\nInstallation complete\n  Mode: %s\n  Directory: %s\n  Capabilities: %s\n  Backend: %s\n' \
        "$INSTALL_MODE" "$INSTALL_DIR" "$CAPABILITIES" "$BACKEND"
    dim "  Passed: CLI, native libraries, selected capability dependencies, PyTorch $BACKEND"
    printf '\nActivate:\n  cd %q\n' "$INSTALL_DIR"
    if [[ "$USE_NIX" == 1 ]]; then printf '  nix develop\n'; fi
    printf '  source .venv/bin/activate\n\nNext commands:\n  dimos list\n'
    if [[ ",$CAPABILITIES," == *,navigation,* ]]; then
        printf '  dimos --viewer none --replay run unitree-go2\n'
    fi
    if [[ ",$CAPABILITIES," == *,manipulation,* ]]; then
        printf '  dimos --simulation run xarm-perception-sim\n'
    fi
    dim "Runtime commands may download models/assets; perception may require CUDA or MPS."
    if [[ "$INSTALL_MODE" == dev ]]; then printf '  uv run --no-sync pytest dimos\n'; fi
    printf '\nSetup and hardware requirements: https://github.com/dimensionalOS/dimos/blob/main/docs/installation/index.md\n'
}

# cleanup
cleanup() {
    local ec=$?
    if [[ -n "$CHILD_PID" ]]; then
        kill -TERM "$CHILD_PID" 2>/dev/null || true
        wait "$CHILD_PID" 2>/dev/null || true
    fi
    [[ $ec -eq 130 ]] && { warn "interrupted"; }
    [[ $ec -ne 0 ]] && [[ $ec -ne 130 ]] && { printf "\n"; err "installation failed (exit ${ec})"; err "help: https://github.com/dimensionalOS/dimos/issues"; }
    return 0
}
trap cleanup EXIT

# main
main() {
    parse_args "$@"
    if [[ "$NON_INTERACTIVE" == 1 ]]; then
        # Never let child processes consume curl's input or prompt for Git credentials.
        exec </dev/null
        export GIT_TERMINAL_PROMPT=0 GIT_ASKPASS=/bin/false SSH_ASKPASS=/bin/false
    elif ! (true </dev/tty) 2>/dev/null; then
        die "no terminal available; use --non-interactive with --mode, --project-dir, and --capabilities"
    fi
    if has_cmd gum && [[ "$NON_INTERACTIVE" != 1 ]]; then GUM=$(command -v gum); fi
    show_banner
    detect_os; detect_gpu; detect_python; detect_nix
    print_sysinfo
    if [[ "$DETECTED_OS" == macos ]]; then
        local mac_major="${DETECTED_OS_VERSION%%.*}"
        [[ "$mac_major" =~ ^[0-9]+$ ]] || die "could not determine macOS version"
        [[ "$mac_major" -ge 14 ]] || die "macOS ${DETECTED_OS_VERSION} too old — 14+ required"
    fi
    if [[ "$CONFIGURE_NETWORK" == 1 ]]; then
        case "$DETECTED_OS" in
            macos) die "--configure-network is supported only on Linux" ;;
            nixos) die "configure LCM buffers through networking.kernel.sysctl in configuration.nix instead" ;;
        esac
    fi
    prompt_install_mode
    prepare_directory
    check_disk_space
    prompt_capabilities
    resolve_capabilities
    select_setup_method
    check_existing_environment
    find_system_packages
    print_install_summary
    if [[ "$DRY_RUN" != 1 ]]; then prompt_confirm "Install this environment?" yes || die "installation cancelled"; fi
    if [[ "$CONFIGURE_NETWORK" == 1 ]]; then require_admin "LCM network configuration"; fi
    if [[ "$USE_NIX" == 1 ]]; then
        export UV_PYTHON_PREFERENCE=only-system UV_PYTHON_DOWNLOADS=never
    else
        export UV_PYTHON_PREFERENCE=only-managed
    fi
    install_system_deps
    install_uv
    do_install
    configure_system
    verify_install
    print_quickstart
}

if [[ "${BASH_SOURCE[0]:-$0}" == "$0" ]]; then
    main "$@"
fi
}
