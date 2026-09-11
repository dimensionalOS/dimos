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
#   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --non-interactive --mode library --extras base,unitree
#
# Prompts read /dev/tty explicitly. Parse the entire script before main runs so
# child processes cannot consume the script when invoked through curl | bash.
{
set -euo pipefail
trap 'exit 130' INT
trap 'exit 143' TERM

INSTALLER_VERSION="0.3.0"

# ─── package lists (edit these when dependencies change) ──────────────────────
UBUNTU_PACKAGES="ca-certificates curl git g++ portaudio19-dev git-lfs libturbojpeg python3-dev pre-commit libgl1 libegl1 libglib2.0-0 ffmpeg libsndfile1 pkg-config"
MACOS_PACKAGES="gnu-sed gcc portaudio git-lfs libjpeg-turbo python pre-commit ffmpeg libsndfile pkg-config"

INSTALL_MODE="${DIMOS_INSTALL_MODE:-}"
EXTRAS="${DIMOS_EXTRAS:-}"
NON_INTERACTIVE="${DIMOS_NO_PROMPT:-0}"
GIT_BRANCH="${DIMOS_BRANCH:-main}"
NO_CUDA="${DIMOS_NO_CUDA:-0}"
NO_SYSCTL="${DIMOS_NO_SYSCTL:-0}"
DRY_RUN="${DIMOS_DRY_RUN:-0}"
PROJECT_DIR="${DIMOS_PROJECT_DIR:-}"
VERBOSE=0
USE_NIX="${DIMOS_USE_NIX:-0}"
NO_NIX="${DIMOS_NO_NIX:-0}"
SKIP_TESTS="${DIMOS_SKIP_TESTS:-0}"
HAS_NIX=0
SETUP_METHOD=""
INSTALL_DIR=""
GUM=""
INSTALL_DEPS=1
DEV_TOOLS=0
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

# ─── gum bootstrap ───────────────────────────────────────────────────────────
GUM_VERSION="0.17.0"

install_gum() {
    if has_cmd gum; then GUM="$(command -v gum)"; return 0; fi

    local arch os gum_os gum_arch tmpdir url bin
    arch="$(uname -m)"; os="$(uname -s)"
    case "$os" in Linux) gum_os="Linux";; Darwin) gum_os="Darwin";; *) return 1;; esac
    case "$arch" in
        x86_64|amd64)   gum_arch="x86_64";;
        aarch64|arm64)  gum_arch="arm64";;
        armv7*|armhf)   gum_arch="armv7";;
        *)              return 1;;
    esac

    tmpdir="$(mktemp -d /tmp/gum-install.XXXXXX)"
    url="https://github.com/charmbracelet/gum/releases/download/v${GUM_VERSION}/gum_${GUM_VERSION}_${gum_os}_${gum_arch}.tar.gz"

    if curl -fsSL "$url" | tar xz -C "$tmpdir" 2>/dev/null; then
        bin="$(find "$tmpdir" -name gum -type f 2>/dev/null | head -1)"
        if [[ -n "$bin" ]] && chmod +x "$bin" && [[ -x "$bin" ]]; then
            GUM="$bin"; return 0
        fi
    fi
    rm -rf "$tmpdir"; return 1
}

# ─── prompt wrappers (gum with fallback) ─────────────────────────────────────

prompt_select() {
    local msg="$1"; shift
    local -a options=("$@")
    if [[ "$NON_INTERACTIVE" == "1" ]]; then PROMPT_RESULT="${options[0]}"; return; fi
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
            PROMPT_RESULT="${options[0]}"
        fi
    fi
}

prompt_multi() {
    local msg="$1"; shift
    local -a options=("$@")
    if [[ "$NON_INTERACTIVE" == "1" ]]; then PROMPT_RESULT=$(printf '%s\n' "${options[@]}"); return; fi
    printf "\n" >/dev/tty
    if [[ -n "$GUM" ]]; then
        local tmpf; tmpf=$(mktemp)
        local ec=0
        "$GUM" choose --no-limit --header "$msg  (space to toggle, enter to confirm)" \
            --cursor "❯ " --cursor.foreground="44" \
            --header.foreground="255" --header.bold \
            --selected.foreground="44" \
            "${options[@]}" </dev/tty >"$tmpf" || ec=$?
        PROMPT_RESULT=$(<"$tmpf"); rm -f "$tmpf"
        if [[ $ec -ne 0 ]]; then die "cancelled"; fi
    else
        printf "%s%s%s (comma-separated, enter for all)\n" "$BOLD" "$msg" "$RESET" >/dev/tty
        local i=1
        for opt in "${options[@]}"; do
            printf "  %s%d)%s %s\n" "$CYAN" "$i" "$RESET" "$opt" >/dev/tty
            ((i++))
        done
        printf "  selection: " >/dev/tty
        local sel; read -r sel </dev/tty || sel=""
        if [[ -z "$sel" ]]; then
            PROMPT_RESULT=$(printf '%s\n' "${options[@]}")
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
        read -r yn </dev/tty || yn=""
        yn="${yn:-$([ "$default" == "yes" ] && echo "y" || echo "n")}"
        [[ "$yn" =~ ^[Yy] ]]
    fi
}

# ─── ascii banner ─────────────────────────────────────────────────────────────
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

# ─── argument parsing ─────────────────────────────────────────────────────────
usage() {
    cat <<EOF
${BOLD}DimOS Interactive Installer${RESET} v${INSTALLER_VERSION}

${BOLD}USAGE${RESET}
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- [OPTIONS]

${BOLD}OPTIONS${RESET}
    --mode library|dev     Install mode (default: interactive prompt)
    --extras <list>        Comma-separated pip extras
    --branch <branch>      Git branch for dev mode (default: main)
    --project-dir <path>   Project directory
    --non-interactive      Accept defaults, no prompts
    --no-cuda              Use CPU inference dependencies (skip CUDA extras)
    --no-sysctl            Skip LCM sysctl configuration
    --use-nix              Force Nix-based setup
    --no-nix               Skip Nix entirely
    --skip-tests           Skip the optional replay smoke test
    --dry-run              Print commands without executing
    --verbose              Show all commands
    --help                 Show this help

${BOLD}EXAMPLES${RESET}
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --mode dev --no-cuda
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --non-interactive --extras base,unitree
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --dry-run
EOF
    exit 0
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --mode|--extras|--branch|--project-dir)
                [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 requires a value"
                ;;
        esac
        case "$1" in
            --mode)            INSTALL_MODE="$2"; shift 2 ;;
            --extras)          EXTRAS="$2"; shift 2 ;;
            --branch)          GIT_BRANCH="$2"; shift 2 ;;
            --project-dir)     PROJECT_DIR="$2"; shift 2 ;;
            --non-interactive) NON_INTERACTIVE=1; shift ;;
            --no-cuda)         NO_CUDA=1; shift ;;
            --no-sysctl)       NO_SYSCTL=1; shift ;;
            --use-nix)         USE_NIX=1; shift ;;
            --no-nix)          NO_NIX=1; shift ;;
            --skip-tests)      SKIP_TESTS=1; shift ;;
            --dry-run)         DRY_RUN=1; NON_INTERACTIVE=1; shift ;;
            --verbose)         VERBOSE=1; shift ;;
            --help|-h)         usage ;;
            *)                 die "unknown option: $1" ;;
        esac
    done
    case "$INSTALL_MODE" in ""|library|dev) ;; *) die "invalid mode: $INSTALL_MODE";; esac
    [[ "$USE_NIX" != 1 || "$NO_NIX" != 1 ]] || die "--use-nix and --no-nix cannot be combined"
    if [[ "$DRY_RUN" == 1 ]]; then NON_INTERACTIVE=1; fi
}

# ─── detection ────────────────────────────────────────────────────────────────
DETECTED_OS="" DETECTED_OS_VERSION="" DETECTED_ARCH=""
DETECTED_GPU="" DETECTED_CUDA=""
DETECTED_PYTHON="" DETECTED_PYTHON_VER=""
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
                    DETECTED_PYTHON="$(command -v "$cmd")"; DETECTED_PYTHON_VER="$ver"; return
                fi
            fi
        fi
    done
    DETECTED_PYTHON=""; DETECTED_PYTHON_VER=""
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

    if [[ "$DETECTED_DISK_GB" -lt 10 ]] 2>/dev/null; then
        warn "only ${DETECTED_DISK_GB}GB disk space free — DimOS needs at least 10GB (50GB+ recommended)"
        if [[ "$DRY_RUN" != "1" ]]; then
            prompt_confirm "Continue with low disk space?" "no" || die "not enough disk space"
        fi
    fi
}

# ─── nix support ──────────────────────────────────────────────────────────────
install_nix() {
    info "Nix is not installed. See: https://nixos.org/download/"
    printf "\n"

    if ! prompt_confirm "Install Nix now? (official nixos.org multi-user installer)" "yes"; then
        if [[ "$DETECTED_OS" == "linux" ]]; then
            warn "skipping Nix — see https://github.com/dimensionalOS/dimos/?tab=readme-ov-file#installation"
            SETUP_METHOD="manual"
        else
            warn "skipping Nix installation — falling back to system packages"
            SETUP_METHOD="system"
        fi
        return
    fi

    info "installing Nix via official installer..."
    if [[ "$DRY_RUN" == "1" ]]; then
        dim "[dry-run] sh <(curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install) --daemon"
        HAS_NIX=1; return
    fi

    if [[ "$NON_INTERACTIVE" == 1 ]]; then
        (cd /tmp && sh <(curl --proto '=https' --tlsv1.2 -fL https://nixos.org/nix/install) --daemon --yes)
    else
        (cd /tmp && sh <(curl --proto '=https' --tlsv1.2 -fL https://nixos.org/nix/install) --daemon) </dev/tty
    fi

    [[ -f /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh ]] && \
        . /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
    mkdir -p "$HOME/.config/nix"
    grep -q "experimental-features.*flakes" "$HOME/.config/nix/nix.conf" 2>/dev/null || \
        echo "experimental-features = nix-command flakes" >> "$HOME/.config/nix/nix.conf"
    has_cmd nix || die "Nix installation failed — 'nix' not found after install"
    HAS_NIX=1; ok "Nix installed ($(nix --version 2>/dev/null))"
}

prompt_setup_method() {
    if [[ "$NO_NIX" == "1" ]]; then
        if [[ "$DETECTED_OS" == "linux" ]]; then SETUP_METHOD="manual"
        else SETUP_METHOD="system"; fi
        return
    fi
    if [[ "$USE_NIX" == "1" ]]; then
        [[ "$HAS_NIX" == "1" ]] && { ok "Nix detected — using for system deps"; SETUP_METHOD="nix"; USE_NIX=1; return; }
        install_nix
        if [[ "$HAS_NIX" == 1 ]]; then SETUP_METHOD="nix"; else USE_NIX=0; fi
        return
    fi

    local choice
    if [[ "$DETECTED_OS" == "linux" ]]; then
        if [[ "$HAS_NIX" == "1" ]]; then
            prompt_select "How should we set up system dependencies?" \
                "Nix — nix develop (recommended for your distro)" \
                "Manual — skip, install dependencies yourself"
        else
            prompt_select "How should we set up system dependencies?" \
                "Install Nix — nix develop (recommended for your distro)" \
                "Manual — skip, install dependencies yourself"
        fi
        choice="$PROMPT_RESULT"
    elif [[ "$HAS_NIX" == "1" ]]; then
        prompt_select "How should we set up system dependencies?" \
            "System packages — apt/brew (simpler)" \
            "Nix — nix develop (reproducible)"
        choice="$PROMPT_RESULT"
    elif [[ "$DETECTED_OS" == "nixos" ]]; then
        die "NixOS detected but 'nix' command not found."
    else
        prompt_select "How should we set up system dependencies?" \
            "System packages — apt/brew (recommended)" \
            "Install Nix — nix develop (reproducible, installs Nix first)"
        choice="$PROMPT_RESULT"
    fi

    case "$choice" in
        *Nix*|*nix*)
            [[ "$HAS_NIX" != "1" ]] && install_nix
            if [[ "$HAS_NIX" == 1 ]]; then
                SETUP_METHOD="nix"; USE_NIX=1; ok "will use Nix for system dependencies"
            else
                USE_NIX=0
            fi ;;
        *Manual*)
            SETUP_METHOD="manual"
            info "see https://github.com/dimensionalOS/dimos/?tab=readme-ov-file#installation" ;;
        *)
            SETUP_METHOD="system"; ok "will use system package manager" ;;
    esac
}

verify_nix_develop() {
    info "verifying nix develop environment..."
    project_cmd sh -c 'command -v python3 && command -v gcc' || die "nix develop verification failed"
}

# ─── system dependencies ─────────────────────────────────────────────────────
install_system_deps() {
    info "checking system dependencies..."

    case "$DETECTED_OS" in
        ubuntu|wsl)
            local -a needed=() privilege=(env)
            if [[ $(id -u) != 0 ]]; then privilege=(sudo); fi
            for pkg in $UBUNTU_PACKAGES; do
                if [[ "$(dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null || true)" != "install ok installed" ]]; then
                    needed+=("$pkg")
                fi
            done
            if [[ ${#needed[@]} -eq 0 ]]; then
                ok "all system dependencies already installed"
                return
            fi
            info "need to install: ${needed[*]}"
            if ! prompt_confirm "Install these packages via apt?" "yes"; then
                warn "skipping system dependencies — some features may not work"
                return
            fi
            run_cmd "${privilege[@]}" apt-get update
            run_cmd "${privilege[@]}" env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get install -y "${needed[@]}"
            ;;
        macos)
            # Homebrew may exist outside PATH, including immediately after bootstrap.
            if ! has_cmd brew; then
                case "$DETECTED_ARCH" in
                    arm64) export PATH="/opt/homebrew/bin:$PATH" ;;
                    x86_64) export PATH="/usr/local/bin:$PATH" ;;
                esac
            fi
            if ! has_cmd brew; then
                info "installing homebrew..."
                if [[ "$DRY_RUN" == 1 ]]; then
                    dim "[dry-run] install Homebrew"
                else
                    local installer
                    installer=$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)
                    if [[ "$NON_INTERACTIVE" == 1 ]]; then
                        NONINTERACTIVE=1 /bin/bash -c "$installer"
                    else
                        /bin/bash -c "$installer" </dev/tty
                    fi
                fi
            fi
            local -a needed=()
            for pkg in $MACOS_PACKAGES; do
                if ! brew list --versions "$pkg" >/dev/null 2>&1; then needed+=("$pkg"); fi
            done
            if [[ ${#needed[@]} -eq 0 ]]; then
                ok "all system dependencies already installed"
                return
            fi
            info "need to install via brew: ${needed[*]}"
            if ! prompt_confirm "Install these packages via brew?" "yes"; then
                warn "skipping system dependencies — some features may not work"
                return
            fi
            run_cmd brew install "${needed[@]}"
            ;;
        nixos)
            info "NixOS detected — system deps managed via nix develop"
            warn "you declined Nix setup; run 'nix develop' manually for system deps"
            ;;
        linux)
            info "see https://github.com/dimensionalOS/dimos/?tab=readme-ov-file#installation"
            warn "install system dependencies manually, then re-run this script"
            return
            ;;
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

# ─── install mode + extras ───────────────────────────────────────────────────
prompt_install_mode() {
    [[ -n "$INSTALL_MODE" ]] && return
    local choice
    prompt_select "How do you want to use DimOS?" \
        "Library — pip install into your project (recommended)" \
        "Developer — git clone + editable install (contributors)"
    choice="$PROMPT_RESULT"
    case "$choice" in *Library*) INSTALL_MODE="library";; *) INSTALL_MODE="dev";; esac
}

prompt_extras() {
    [[ -n "$EXTRAS" ]] && return
    if [[ "$INSTALL_MODE" == "dev" ]]; then EXTRAS="all"; info "developer mode: all extras (except dds)"; return; fi

    local -a platform_sel=() feature_sel=()
    local _platforms _features
    prompt_multi \
        "Which robot platforms will you use?" \
        "Unitree (Go2, G1, B1)" "Drone (Mavlink / DJI)" "Manipulators (xArm, Piper, OpenARMs)"
    _platforms="$PROMPT_RESULT"
    while IFS= read -r line; do [[ -n "$line" ]] && platform_sel+=("$line"); done <<< "$_platforms"

    prompt_multi \
        "Which features do you need?" \
        "AI Agents (LangChain, voice control)" "Perception (object detection, VLMs)" \
        "Visualization (Rerun 3D viewer)" "Simulation (MuJoCo)" \
        "Web Interface (FastAPI dashboard)" "Misc (extra ML models)"
    _features="$PROMPT_RESULT"
    while IFS= read -r line; do [[ -n "$line" ]] && feature_sel+=("$line"); done <<< "$_features"

    local -a extras_list=()
    for p in "${platform_sel[@]}"; do
        case "$p" in *Unitree*) extras_list+=("unitree");; *Drone*) extras_list+=("drone");; *Manipulator*) extras_list+=("manipulation");; esac
    done
    for f in "${feature_sel[@]}"; do
        case "$f" in *Agent*) extras_list+=("agents");; *Perception*) extras_list+=("perception");; *Visualization*) extras_list+=("visualization");;
            *Simulation*) extras_list+=("sim");; *Web*) extras_list+=("web");; *Misc*) extras_list+=("misc");; esac
    done

    if [[ "$DETECTED_GPU" == "nvidia" ]] && [[ "$NO_CUDA" != "1" ]]; then
        prompt_confirm "NVIDIA GPU detected — install CUDA support?" "yes" && extras_list+=("cuda") || extras_list+=("cpu")
    else
        extras_list+=("cpu")
    fi

    if prompt_confirm "Include development tools (ruff, pytest, mypy)?" "no"; then DEV_TOOLS=1; fi

    [[ ${#extras_list[@]} -eq 0 ]] && extras_list=("base")
    EXTRAS="$(IFS=,; echo "${extras_list[*]}")"
    printf "\n"; ok "selected extras: ${CYAN}${EXTRAS}${RESET}"
}

prompt_install_dir() {
    local default="$1" mode="$2"
    if [[ "$NON_INTERACTIVE" == "1" ]]; then echo "$default"; return; fi

    local hint
    [[ "$mode" == "dev" ]] && hint="git clone destination" || hint="project directory"

    if [[ -n "$GUM" ]]; then
        local result
        result=$("$GUM" input --header "Where should we install DimOS? (${hint})"             --placeholder "$default" --value "$default"             --header.foreground="255" --header.bold             --cursor.foreground="44" </dev/tty) || { printf "\n" >/dev/tty; exit $CANCELLED_EXIT; }
        [[ -z "$result" ]] && result="$default"
        echo "$result"
    else
        printf "\n%sWhere should we install DimOS?%s (%s)\n" "$BOLD" "$RESET" "$hint" >/dev/tty
        printf "  path [%s]: " "$default" >/dev/tty
        local result
        read -r result </dev/tty || result=""
        [[ -z "$result" ]] && result="$default"
        echo "$result"
    fi
}

# ─── installation ─────────────────────────────────────────────────────────────
# Expand the aggregate so --no-cuda and platform restrictions also apply to all.
resolve_extras() {
    local requested="$EXTRAS" extra
    local -a selected=() inputs=()
    IFS=',' read -r -a inputs <<< "$requested"
    for extra in "${inputs[@]}"; do
        if [[ "$extra" == all ]]; then
            selected+=(agents apriltag base drone manipulation misc perception sim unitree visualization web webrtc)
            if [[ "$DETECTED_OS" != macos && "$DETECTED_ARCH" == aarch64 ]]; then
                info "scene is unavailable on Linux ARM64; excluding it from all"
            else
                selected+=(scene)
            fi
        else
            [[ "$extra" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || die "invalid extra: $extra"
            if [[ "$extra" == scene && "$DETECTED_OS" != macos && "$DETECTED_ARCH" == aarch64 ]]; then
                die "scene requires usd-core, which has no Linux ARM64 wheel"
            fi
            if [[ "$extra" == cuda && "$NO_CUDA" == 1 ]]; then continue; fi
            selected+=("$extra")
        fi
    done
    EXTRAS="$(IFS=,; echo "${selected[*]:-}")"
    if [[ ",$EXTRAS," == *,cuda,* ]]; then
        [[ "$DETECTED_OS" != macos && "$DETECTED_ARCH" == x86_64 ]] || die "CUDA installation requires Linux x86_64; Jetson CUDA is not supported"
        [[ ",$EXTRAS," != *,cpu,* ]] || die "select either cpu or cuda, not both"
    elif [[ ",$EXTRAS," != *,cpu,* ]]; then
        if [[ "$NO_CUDA" != 1 && "$DETECTED_GPU" == nvidia && "$DETECTED_ARCH" == x86_64 ]]; then
            EXTRAS="${EXTRAS:+$EXTRAS,}cuda"
        else
            EXTRAS="${EXTRAS:+$EXTRAS,}cpu"
        fi
    fi
    info "installing extras: $EXTRAS"
}

do_install_library() {
    local dir="${PROJECT_DIR:-}"
    if [[ -z "$dir" ]]; then dir=$(prompt_install_dir "$PWD/dimensional-applications" library) || die "cancelled"; fi
    INSTALL_DIR="$dir"
    info "library install → $dir"
    run_cmd mkdir -p "$dir"
    if ! prompt_confirm "Install dependencies now?" yes; then
        INSTALL_DEPS=0
        dim "to install later: uv venv --python 3.12 && uv pip install 'dimos[$EXTRAS]'"
        return
    fi
    if [[ "$USE_NIX" == 1 ]]; then
        local base="https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/$GIT_BRANCH"
        run_cmd curl -fsSL "$base/flake.nix" -o "$dir/flake.nix"
        run_cmd curl -fsSL "$base/flake.lock" -o "$dir/flake.lock"
        if [[ ! -e "$dir/.git" ]]; then run_cmd git -C "$dir" init -q; fi
        run_cmd git -C "$dir" add flake.nix flake.lock
        verify_nix_develop
    fi
    if [[ -d "$dir/.venv" && "$DRY_RUN" != 1 ]]; then
        if prompt_confirm "Replace existing virtual environment?" no; then
            project_cmd env UV_VENV_CLEAR=1 uv venv --python 3.12
        else
            info "keeping existing .venv"
        fi
    else
        project_cmd uv venv --python 3.12
    fi
    local backend=cpu
    if [[ ",$EXTRAS," == *,cuda,* ]]; then backend=cu128; fi
    project_cmd uv pip install --python .venv/bin/python --torch-backend "$backend" "dimos[$EXTRAS]"
    if [[ "$DEV_TOOLS" == 1 ]]; then
        project_cmd uv pip install --python .venv/bin/python ruff pytest mypy
    fi
    ok "dimos installed in $dir"
}

do_install_dev() {
    local dir="${PROJECT_DIR:-}"
    if [[ -z "$dir" ]]; then dir=$(prompt_install_dir "$PWD/dimos" dev) || die "cancelled"; fi
    INSTALL_DIR="$dir"
    info "developer install → $dir"
    if [[ -e "$dir/.git" ]]; then
        git -C "$dir" rev-parse --is-inside-work-tree >/dev/null || die "invalid Git checkout: $dir"
        info "using existing checkout at $(git -C "$dir" rev-parse --short HEAD)"
    else
        run_cmd env GIT_LFS_SKIP_SMUDGE=1 git clone -b "$GIT_BRANCH" https://github.com/dimensionalOS/dimos.git "$dir"
    fi
    local -a sync_args=(--locked --python 3.12 --group tests --group lint)
    local -a extras=()
    local extra
    IFS=',' read -r -a extras <<< "$EXTRAS"
    for extra in "${extras[@]}"; do sync_args+=(--extra "$extra"); done
    dim "will run: uv sync ${sync_args[*]}"
    if ! prompt_confirm "Install dependencies now?" yes; then INSTALL_DEPS=0; return; fi
    if [[ "$USE_NIX" == 1 ]]; then verify_nix_develop; fi
    project_cmd uv sync "${sync_args[@]}"
    ok "developer environment ready in $dir"
}

do_install() {
    case "$INSTALL_MODE" in library) do_install_library;; dev) do_install_dev;; *) die "invalid mode: $INSTALL_MODE";; esac
}

# ─── system configuration ────────────────────────────────────────────────────
configure_system() {
    [[ "$NO_SYSCTL" == "1" ]] && { dim "  skipping sysctl (--no-sysctl)"; return; }
    [[ "$DETECTED_OS" == "macos" ]] && return
    if [[ "$DETECTED_OS" == "nixos" ]]; then
        info "NixOS: add to configuration.nix:"
        dim "  networking.kernel.sysctl.\"net.core.rmem_max\" = 67108864;"
        dim "  networking.kernel.sysctl.\"net.core.rmem_default\" = 67108864;"
        return
    fi
    local current_rmem; current_rmem="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
    if [[ "$current_rmem" -ge 67108864 ]]; then ok "LCM buffers already configured"; return; fi

    printf "\n"
    info "DimOS uses LCM transport which needs larger UDP buffers:"
    dim "  sudo sysctl -w net.core.rmem_max=67108864"
    dim "  sudo sysctl -w net.core.rmem_default=67108864"
    printf "\n"

    if prompt_confirm "Apply sysctl changes?" "yes"; then
        run_cmd sudo sysctl -w net.core.rmem_max=67108864
        run_cmd sudo sysctl -w net.core.rmem_default=67108864
        if prompt_confirm "Persist across reboots?" "yes"; then
            if [[ "$DRY_RUN" != "1" ]]; then
                printf "# DimOS LCM transport buffers\nnet.core.rmem_max=67108864\nnet.core.rmem_default=67108864\n" | sudo tee /etc/sysctl.d/99-dimos.conf >/dev/null
            else dim "[dry-run] would write /etc/sysctl.d/99-dimos.conf"; fi
        fi
        ok "LCM buffers configured"
    fi
}

# ─── verification ─────────────────────────────────────────────────────────────
# Python's timeout works on macOS too. Each command owns a process group so
# timeout and interruption also stop workers started by a blueprint.
run_bounded() {
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

verify_install() {
    if [[ "$INSTALL_DEPS" != 1 || "$DRY_RUN" == 1 ]]; then return; fi
    info "verifying installation..."
    run_bounded 60 .venv/bin/dimos --help
    run_bounded 60 .venv/bin/dimos list
    run_bounded 60 .venv/bin/python -c 'import sqlite3, cv2, open3d; from turbojpeg import TurboJPEG; TurboJPEG()'
    if [[ ",$EXTRAS," == *,cuda,* ]]; then
        run_bounded 60 .venv/bin/python -c 'import torch; assert torch.cuda.is_available(); assert (torch.ones(1, device="cuda") + 1).item() == 2'
    elif [[ "$NO_CUDA" == 1 ]]; then
        run_bounded 60 .venv/bin/python -c 'import torch; assert (torch.ones(1, device="cpu") + 1).item() == 2'
    fi
    ok "installation verified"
}

run_post_install_tests() {
    [[ "$INSTALL_DEPS" != 1 || "$SKIP_TESTS" == 1 || "$DRY_RUN" == 1 ]] && return 0
    [[ ",$EXTRAS," == *,unitree,* ]] || return 0
    prompt_confirm "Run a quick smoke test? (starts unitree-go2 replay for 60s)" yes || return 0
    local exit_code=0
    run_bounded 60 .venv/bin/dimos --viewer none --replay run unitree-go2 || exit_code=$?
    case "$exit_code" in
        0|124) ok "smoke test passed" ;;
        *) die "smoke test failed (exit $exit_code)" ;;
    esac
}

# ─── quickstart ───────────────────────────────────────────────────────────────
print_quickstart() {
    local dir="$INSTALL_DIR"
    if [[ "$DRY_RUN" == 1 ]]; then info "dry-run complete; no installation performed"; return; fi
    printf "\n  %s%s🎉 installation complete!%s\n\n  %sget started:%s\n\n" "$BOLD" "$GREEN" "$RESET" "$BOLD" "$RESET"

    if [[ "$USE_NIX" == "1" ]]; then
        printf "    %s# enter nix shell + activate python%s\n    cd %s && nix develop --command bash -c 'source .venv/bin/activate && exec bash'\n\n" "$DIM" "$RESET" "$dir"
        printf "    %s# or in two steps:%s\n    cd %s && nix develop\n    %s# then inside nix shell:%s\n    source .venv/bin/activate\n\n" "$DIM" "$RESET" "$dir" "$DIM" "$RESET"
    else
        printf "    %s# activate the environment%s\n    cd %s && source .venv/bin/activate\n\n" "$DIM" "$RESET" "$dir"
    fi

    if [[ "$EXTRAS" == *"unitree"* ]] || [[ "$EXTRAS" == "all" ]] || [[ "$EXTRAS" == *"base"* ]]; then
        printf "    %s# simulation%s\n    dimos --simulation run unitree-go2\n\n" "$DIM" "$RESET"
        printf "    %s# real hardware%s\n    ROBOT_IP=192.168.1.100 dimos run unitree-go2\n\n" "$DIM" "$RESET"
    fi
    if [[ "$EXTRAS" == *"sim"* ]] || [[ "$EXTRAS" == "all" ]]; then
        printf "    %s# MuJoCo simulation%s\n    dimos --simulation run unitree-go2\n\n" "$DIM" "$RESET"
    fi
    if [[ "$INSTALL_MODE" == "dev" ]]; then
        printf "    %s# tests%s\n    uv run pytest dimos\n\n    %s# type check%s\n    uv run mypy dimos\n\n" "$DIM" "$RESET" "$DIM" "$RESET"
    fi
    if [[ "$USE_NIX" == "1" ]]; then
        printf "  %s⚠%s open a %snew terminal%s first, then run 'nix develop' before working with DimOS\n" "$YELLOW" "$RESET" "$BOLD" "$RESET"
        printf "  %s⚠%s or run: %sexec bash -l%s  to reload this shell\n\n" "$YELLOW" "$RESET" "$CYAN" "$RESET"
    fi
    printf "  %sdocs:%s       https://github.com/dimensionalOS/dimos\n" "$DIM" "$RESET"
    printf "  %sdiscord:%s    https://discord.gg/dimos\n\n" "$DIM" "$RESET"
}

# ─── cleanup ─────────────────────────────────────────────────────────────────
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

# ─── main ─────────────────────────────────────────────────────────────────────
main() {
    parse_args "$@"
    if [[ "$NON_INTERACTIVE" != 1 ]] && ! (true </dev/tty) 2>/dev/null; then
        die "no terminal available; use --non-interactive"
    fi
    export UV_PYTHON_PREFERENCE=only-managed

    if [[ "$NON_INTERACTIVE" != "1" ]]; then
        if install_gum 2>/dev/null; then
            dim "  using gum for interactive prompts"
        else
            dim "  using basic prompts (install gum for a better experience)"
        fi
    fi

    show_banner
    detect_os; detect_gpu; detect_python; detect_nix
    print_sysinfo

    if [[ "$DETECTED_OS" == "ubuntu" ]] || [[ "$DETECTED_OS" == "wsl" ]]; then
        local ver_major; ver_major="$(echo "$DETECTED_OS_VERSION" | cut -d. -f1)"
        if [[ "$ver_major" =~ ^[0-9]+$ ]] && [[ "$ver_major" -lt 22 ]]; then
            warn "Ubuntu ${DETECTED_OS_VERSION} — 22.04+ recommended"
        fi
    fi
    if [[ "$DETECTED_OS" == "macos" ]]; then
        local mac_major; mac_major="$(echo "$DETECTED_OS_VERSION" | cut -d. -f1)"
        if [[ "$mac_major" =~ ^[0-9]+$ ]] && [[ "$mac_major" -lt 12 ]]; then
            die "macOS ${DETECTED_OS_VERSION} too old — 12.6+ required"
        fi
    fi

    prompt_setup_method
    if [[ "$SETUP_METHOD" != "nix" ]]; then install_system_deps; fi
    install_uv

    if [[ -z "$DETECTED_PYTHON" ]]; then
        detect_python
        [[ -z "$DETECTED_PYTHON" ]] && info "python 3.12 will be installed by uv automatically"
    fi

    prompt_install_mode

    # Warn about known Nix + library + old glibc issue
    if [[ "$USE_NIX" == "1" ]] && [[ "$INSTALL_MODE" == "library" ]]; then
        if [[ "$DETECTED_OS" == "ubuntu" ]] || [[ "$DETECTED_OS" == "wsl" ]]; then
            local glibc_ver; glibc_ver=$(ldd --version 2>&1 | head -1 | grep -oP '[0-9]+\.[0-9]+$' || echo "0")
            if awk "BEGIN{exit !($glibc_ver < 2.38)}"; then
                warn "Nix + library install on glibc ${glibc_ver} may have issues"
                dim "  Nix's LD_LIBRARY_PATH can conflict with PyPI wheels on glibc < 2.38"
                dim "  if you hit import errors, try: system packages instead of Nix"
                dim "  or upgrade to Ubuntu 24.04+ (glibc 2.39)"
            fi
        fi
    fi

    prompt_extras
    resolve_extras
    do_install
    if [[ "$INSTALL_DEPS" != 1 ]]; then
        info "setup prepared; dependency installation was skipped"
        return
    fi
    configure_system
    verify_install
    run_post_install_tests
    print_quickstart
}

if [[ "${BASH_SOURCE[0]:-$0}" == "$0" ]]; then
    main "$@"
fi
}
