#!/usr/bin/env bash
# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0
#
# Interactive installer for DimOS — the agentive operating system for generalist robotics.
#
# Usage:
#   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dev/scripts/install.sh | bash
#   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dev/scripts/install.sh | bash -s -- --help
#
# Non-interactive:
#   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dev/scripts/install.sh | bash -s -- --non-interactive --mode library --extras base,unitree
#
set -euo pipefail


# ─── signal handling (must be early so Ctrl+C works everywhere) ────────────────
trap 'printf "\n"; exit 130' INT

# If piped from curl (stdin is not a TTY and $0 is the shell),
# save to temp file and re-execute so interactive prompts get proper TTY input.
if [ ! -t 0 ] && { [ "$0" = "bash" ] || [ "$0" = "-bash" ] || [ "$0" = "/bin/bash" ] || [ "$0" = "/usr/bin/bash" ] || [ "$0" = "sh" ] || [ "$0" = "/bin/sh" ]; }; then
    TMPSCRIPT="$(mktemp /tmp/dimos-install.XXXXXX.sh)"
    cat > "$TMPSCRIPT"
    chmod +x "$TMPSCRIPT"
    exec bash "$TMPSCRIPT" "$@"
fi

INSTALLER_VERSION="0.3.0"

# ─── package lists (edit these when dependencies change) ──────────────────────
UBUNTU_PACKAGES="curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev pre-commit libgl1 libegl1"
MACOS_PACKAGES="gnu-sed gcc portaudio git-lfs libjpeg-turbo python pre-commit"

INSTALL_MODE="${DIMOS_INSTALL_MODE:-}"
EXTRAS="${DIMOS_EXTRAS:-}"
NON_INTERACTIVE="${DIMOS_NO_PROMPT:-0}"
GIT_BRANCH="${DIMOS_BRANCH:-dev}"
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
check_cancel() { [[ $? -eq $CANCELLED_EXIT ]] && { err "cancelled"; exit 1; }; return 0; }
dim()   { printf "%s%s%s\n" "$DIM" "$*" "$RESET"; }

run_cmd() {
    if [[ "$DRY_RUN" == "1" ]]; then dim "[dry-run] $*"; return 0; fi
    [[ "$VERBOSE" == "1" ]] && dim "$ $*"
    eval "$@"
}

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
        "$GUM" choose --header "$msg" \
            --cursor "● " --cursor.foreground="44" \
            --header.foreground="255" --header.bold \
            --selected.foreground="44" \
            "${options[@]}" </dev/tty >"$tmpf"
        local ec=$?; PROMPT_RESULT=$(<"$tmpf"); rm -f "$tmpf"
        [[ $ec -ne 0 ]] && die "cancelled"
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
        local idx=$((choice - 1))
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
        "$GUM" choose --no-limit --header "$msg  (space to toggle, enter to confirm)" \
            --cursor "❯ " --cursor.foreground="44" \
            --header.foreground="255" --header.bold \
            --selected.foreground="44" \
            "${options[@]}" </dev/tty >"$tmpf"
        local ec=$?; PROMPT_RESULT=$(<"$tmpf"); rm -f "$tmpf"
        [[ $ec -ne 0 ]] && die "cancelled"
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
                n="${n// /}"; local idx=$((n - 1))
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

prompt_spin() {
    local title="$1"; shift
    if [[ "$DRY_RUN" == "1" ]]; then dim "[dry-run] $*"; return 0; fi
    if [[ -n "$GUM" ]] && [[ "$VERBOSE" != "1" ]]; then
        "$GUM" spin --title "$title" --spinner dot --spinner.foreground="44" -- bash -c "$*"
    else
        [[ "$VERBOSE" == "1" ]] && dim "$ $*"
        info "$title"
        eval "$@"
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
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dev/scripts/install.sh | bash
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dev/scripts/install.sh | bash -s -- [OPTIONS]

${BOLD}OPTIONS${RESET}
    --mode library|dev     Install mode (default: interactive prompt)
    --extras <list>        Comma-separated pip extras
    --branch <branch>      Git branch for dev mode (default: dev)
    --project-dir <path>   Project directory
    --non-interactive      Accept defaults, no prompts
    --no-cuda              Force CPU-only
    --no-sysctl            Skip LCM sysctl configuration
    --use-nix              Force Nix-based setup
    --no-nix               Skip Nix entirely
    --skip-tests           Skip post-install verification
    --dry-run              Print commands without executing
    --verbose              Show all commands
    --help                 Show this help

${BOLD}EXAMPLES${RESET}
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dev/scripts/install.sh | bash
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dev/scripts/install.sh | bash -s -- --mode dev --no-cuda
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dev/scripts/install.sh | bash -s -- --non-interactive --extras base,unitree
    curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/dev/scripts/install.sh | bash -s -- --dry-run
EOF
    exit 0
}

parse_args() {
    while [[ $# -gt 0 ]]; do
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
            *)                 warn "unknown option: $1"; shift ;;
        esac
    done
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
        else die "unsupported Linux distribution (only Debian/Ubuntu-based distros supported)"; fi
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
    elif has_cmd nvidia-smi; then
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
                if [[ "$major" -eq 3 ]] && [[ "$minor" -ge 10 ]]; then
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
        has_cmd nix && HAS_NIX=1
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
        warn "skipping Nix installation — falling back to system packages"
        SETUP_METHOD="system"
        return
    fi

    info "installing Nix via official installer..."
    if [[ "$DRY_RUN" == "1" ]]; then
        dim "[dry-run] sh <(curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install) --daemon"
        HAS_NIX=1; return
    fi

    # Clean stale Nix backup files that block reinstall
    for rc in /etc/bashrc /etc/bash.bashrc /etc/zshrc; do
        if [[ -f "${rc}.backup-before-nix" ]]; then
            sudo mv "${rc}.backup-before-nix" "$rc"
        fi
    done

    # Run from /tmp (CWD may not exist) with TTY so nix installer can prompt
    (cd /tmp && sh <(curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install) --daemon) </dev/tty

    [[ -f /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh ]] && \
        . /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
    mkdir -p "$HOME/.config/nix"
    grep -q "experimental-features.*flakes" "$HOME/.config/nix/nix.conf" 2>/dev/null || \
        echo "experimental-features = nix-command flakes" >> "$HOME/.config/nix/nix.conf"
    has_cmd nix || die "Nix installation failed — 'nix' not found after install"
    HAS_NIX=1; ok "Nix installed ($(nix --version 2>/dev/null))"
}

prompt_setup_method() {
    if [[ "$NO_NIX" == "1" ]]; then SETUP_METHOD="system"; return; fi
    if [[ "$USE_NIX" == "1" ]]; then
        [[ "$HAS_NIX" == "1" ]] && { ok "Nix detected — using for system deps"; SETUP_METHOD="nix"; USE_NIX=1; return; }
        install_nix; SETUP_METHOD="nix"; USE_NIX=1; return
    fi

    local choice
    if [[ "$HAS_NIX" == "1" ]]; then
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
            SETUP_METHOD="nix"; USE_NIX=1; ok "will use Nix for system dependencies" ;;
        *)
            SETUP_METHOD="system"; ok "will use system package manager" ;;
    esac
}

verify_nix_develop() {
    local dir="$1"
    info "verifying nix develop environment (first run downloads dependencies, may take a few minutes)..."
    if [[ "$DRY_RUN" == "1" ]]; then ok "nix develop verification skipped (dry-run)"; return; fi
    local tmpf; tmpf=$(mktemp)
    # Run with live output so user sees download/build progress
    (cd "$dir" && nix develop --command bash -c '
        echo "python3=$(which python3 2>/dev/null || echo MISSING)"
        echo "gcc=$(which gcc 2>/dev/null || echo MISSING)"
    ' >"$tmpf") || true
    local nix_check; nix_check=$(<"$tmpf"); rm -f "$tmpf"
    echo "$nix_check" | grep -q "python3=MISSING" && warn "nix develop: python3 not found" || ok "nix develop: python3 available"
    echo "$nix_check" | grep -q "gcc=MISSING" && warn "nix develop: gcc not found" || ok "nix develop: gcc available"
}

# ─── system dependencies ─────────────────────────────────────────────────────
install_system_deps() {
    info "checking system dependencies..."

    case "$DETECTED_OS" in
        ubuntu|wsl)
            local needed=""
            for pkg in $UBUNTU_PACKAGES; do
                if ! dpkg -s "$pkg" &>/dev/null; then
                    needed+=" $pkg"
                fi
            done
            if [[ -z "$needed" ]]; then
                ok "all system dependencies already installed"
                return
            fi
            info "need to install:${needed}"
            if ! prompt_confirm "Install these packages via apt?" "yes"; then
                warn "skipping system dependencies — some features may not work"
                return
            fi
            run_cmd "sudo apt-get update"
            run_cmd "sudo DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get install -y $needed"
            ;;
        macos)
            if ! has_cmd brew; then
                info "installing homebrew..."
                run_cmd '/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"'
            fi
            local needed=""
            for pkg in $MACOS_PACKAGES; do
                if ! brew list "$pkg" &>/dev/null 2>&1; then
                    needed+=" $pkg"
                fi
            done
            if [[ -z "$needed" ]]; then
                ok "all system dependencies already installed"
                return
            fi
            info "need to install via brew:${needed}"
            if ! prompt_confirm "Install these packages via brew?" "yes"; then
                warn "skipping system dependencies — some features may not work"
                return
            fi
            run_cmd "brew install $needed"
            ;;
        nixos)
            info "NixOS detected — system deps managed via nix develop"
            warn "you declined Nix setup; run 'nix develop' manually for system deps"
            ;;
    esac
    ok "system dependencies ready"
}

install_uv() {
    if has_cmd uv; then ok "uv already installed ($(uv --version 2>/dev/null))"; return; fi
    info "installing uv..."
    run_cmd 'curl -LsSf https://astral.sh/uv/install.sh | sh'
    export PATH="$HOME/.local/bin:$PATH"
    if ! has_cmd uv; then
        for rc in "$HOME/.bashrc" "$HOME/.zshrc" "$HOME/.profile" "$HOME/.cargo/env"; do
            [[ -f "$rc" ]] && source "$rc" 2>/dev/null || true
        done
    fi
    has_cmd uv || die "uv installation failed — install manually: https://docs.astral.sh/uv/"
    ok "uv installed ($(uv --version 2>/dev/null))"
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

    prompt_confirm "Include development tools (ruff, pytest, mypy)?" "no" && extras_list+=("dev")

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
do_install_library() {
    local dir="${PROJECT_DIR:-}"
    if [[ -z "$dir" ]]; then dir=$(prompt_install_dir "$PWD/dimensional-applications" "library") || die "cancelled"; fi
    INSTALL_DIR="$dir"
    info "library install → ${dir}"
    run_cmd "mkdir -p '$dir'"

    # Show what we're about to do
    printf "\n"
    info "next step: create .venv and install Python packages"
    if [[ "$USE_NIX" == "1" ]]; then
        dim "  will run: ${CYAN}cd ${dir} && nix develop --command bash -c \"uv venv && uv pip install 'dimos[${EXTRAS}]'\"${RESET}"
        dim "  nix develop provides system libraries (libGL, mesa, etc.)"
    else
        dim "  will run: ${CYAN}cd ${dir} && uv venv --python 3.12 && uv pip install \"dimos[${EXTRAS}]\"${RESET}"
    fi
    printf "\n"

    if ! prompt_confirm "Install dependencies now?" "yes"; then
        dim "  skipping dependency install"
        printf "\n"
        dim "  to install later, run:"
        dim "    cd ${dir}"
        if [[ "$USE_NIX" == "1" ]]; then
            dim "    nix develop"
            dim "    uv venv --python 3.12"
            dim "    source .venv/bin/activate"
            dim "    uv pip install \"dimos[${EXTRAS}]\""
        else
            dim "    uv venv --python 3.12"
            dim "    source .venv/bin/activate"
            dim "    uv pip install \"dimos[${EXTRAS}]\""
        fi
        ok "project directory created at ${dir}"
        return
    fi

    if [[ "$USE_NIX" == "1" ]]; then
        info "downloading flake files..."
        local base="https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/${GIT_BRANCH}"
        if [[ "$DRY_RUN" == "1" ]]; then dim "[dry-run] curl flake.nix + flake.lock"
        else
            curl -fsSL "${base}/flake.nix" -o "${dir}/flake.nix"
            curl -fsSL "${base}/flake.lock" -o "${dir}/flake.lock"
        fi
        [[ "$DRY_RUN" != "1" ]] && [[ ! -d "${dir}/.git" ]] && (cd "$dir" && git init -q && git add flake.nix flake.lock && git commit -q -m "init" --allow-empty)
        verify_nix_develop "$dir"
        info "installing dimos[${EXTRAS}] via nix develop..."
        if [[ "$DRY_RUN" == "1" ]]; then dim "[dry-run] nix develop + uv venv + uv pip install"
        else
            local venv_flag=""
            if [[ -d "${dir}/.venv" ]]; then
                warn "existing .venv found in ${dir}/"
                if prompt_confirm "Replace existing virtual environment?" "no"; then
                    venv_flag="UV_VENV_CLEAR=1"
                else
                    info "keeping existing .venv"
                    venv_flag="SKIP_VENV=1"
                fi
            fi
            (cd "$dir" && nix develop --command bash -c "set -euo pipefail; [[ \"\${SKIP_VENV:-}\" != 1 ]] && ${venv_flag} uv venv --python 3.12; source .venv/bin/activate; uv pip install 'dimos[${EXTRAS}]'")
        fi
    else
        if [[ -d "${dir}/.venv" ]] && [[ "$DRY_RUN" != "1" ]]; then
            warn "existing .venv found in ${dir}/"
            if prompt_confirm "Replace existing virtual environment?" "no"; then
                info "replacing virtual environment..."
                (cd "$dir" && UV_VENV_CLEAR=1 uv venv --python 3.12)
            else
                info "keeping existing .venv — skipping venv creation"
            fi
        else
            info "creating virtual environment (python 3.12)..."
            if [[ "$DRY_RUN" == "1" ]]; then dim "[dry-run] uv venv --python 3.12"
            else pushd "$dir" >/dev/null && uv venv --python 3.12 && popd >/dev/null; fi
        fi
        info "installing dimos[${EXTRAS}]..."
        if [[ "$DRY_RUN" == "1" ]]; then dim "[dry-run] uv pip install 'dimos[${EXTRAS}]'"
        else pushd "$dir" >/dev/null && source .venv/bin/activate && uv pip install "dimos[${EXTRAS}]" && popd >/dev/null; fi
    fi
    ok "dimos installed in ${dir}"
}

do_install_dev() {
    local dir="${PROJECT_DIR:-}"
    if [[ -z "$dir" ]]; then dir=$(prompt_install_dir "$PWD/dimos" "dev") || die "cancelled"; fi
    INSTALL_DIR="$dir"
    info "developer install → ${dir}"

    # Step 1: Clone or pull
    if [[ -d "$dir/.git" ]]; then
        info "existing clone found, pulling latest..."
        run_cmd "cd '$dir' && git pull --rebase origin $GIT_BRANCH"
    else
        info "cloning dimos (branch: ${GIT_BRANCH})..."
        run_cmd "GIT_LFS_SKIP_SMUDGE=1 git clone -b $GIT_BRANCH https://github.com/dimensionalOS/dimos.git '$dir'"
    fi

    # Step 2: Install dependencies (ask first — this is the heavy part)
    printf "\n"
    info "next step: create .venv and install all Python dependencies"
    if [[ "$USE_NIX" == "1" ]]; then
        dim "  will run: ${CYAN}nix develop --command bash -c \"uv sync --all-extras --no-extra dds\"${RESET}"
    else
        dim "  will run: ${CYAN}cd ${dir} && uv sync --all-extras --no-extra dds${RESET}"
    fi
    dim "  this creates a .venv and installs ~430 packages (may take several minutes)"
    printf "\n"

    if ! prompt_confirm "Install dependencies now?" "yes"; then
        dim "  skipping dependency install"
        printf "\n"
        dim "  to install later, run:"
        dim "    cd ${dir}"
        if [[ "$USE_NIX" == "1" ]]; then
            dim "    nix develop --command bash -c \"uv sync --all-extras --no-extra dds\""
        else
            dim "    uv sync --all-extras --no-extra dds"
        fi
        ok "repository cloned to ${dir}"
        return
    fi

    if [[ "$USE_NIX" == "1" ]]; then
        verify_nix_develop "$dir"
        if [[ "$DRY_RUN" == "1" ]]; then dim "[dry-run] nix develop + uv sync --all-extras --no-extra dds"
        else (cd "$dir" && nix develop --command bash -c "set -euo pipefail && uv sync --all-extras --no-extra dds"); fi
    else
        if [[ "$DRY_RUN" == "1" ]]; then dim "[dry-run] uv sync --all-extras --no-extra dds"
        else pushd "$dir" >/dev/null && uv sync --all-extras --no-extra dds && popd >/dev/null; fi
    fi
    ok "developer environment ready in ${dir}"
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
        run_cmd "sudo sysctl -w net.core.rmem_max=67108864"
        run_cmd "sudo sysctl -w net.core.rmem_default=67108864"
        if prompt_confirm "Persist across reboots?" "yes"; then
            if [[ "$DRY_RUN" != "1" ]]; then
                printf "# DimOS LCM transport buffers\nnet.core.rmem_max=67108864\nnet.core.rmem_default=67108864\n" | sudo tee /etc/sysctl.d/99-dimos.conf >/dev/null
            else dim "[dry-run] would write /etc/sysctl.d/99-dimos.conf"; fi
        fi
        ok "LCM buffers configured"
    fi
}

# ─── verification ─────────────────────────────────────────────────────────────
verify_install() {
    info "verifying installation..."
    local dir="$INSTALL_DIR"
    if [[ "$DRY_RUN" == "1" ]]; then ok "verification skipped (dry-run)"; return; fi
    local venv_python="${dir}/.venv/bin/python3"
    [[ ! -f "$venv_python" ]] && { warn "venv not found, skipping verification"; return; }

    if [[ "$USE_NIX" == "1" ]]; then
        (cd "$dir" && nix develop --command bash -c "source .venv/bin/activate && python3 -c 'import dimos'" 2>/dev/null) && ok "python import: dimos ✓" || warn "import check failed"
    else
        "$venv_python" -c "import dimos" 2>/dev/null && ok "python import: dimos ✓" || warn "import check failed"
    fi

    [[ -x "${dir}/.venv/bin/dimos" ]] && ok "dimos CLI available" || dim "  activate venv for CLI: source .venv/bin/activate"

    if [[ "$DETECTED_GPU" == "nvidia" ]] && [[ "$NO_CUDA" != "1" ]] && [[ "$EXTRAS" == *"cuda"* ]]; then
        "$venv_python" -c "import torch; assert torch.cuda.is_available()" 2>/dev/null && ok "CUDA available" || dim "  CUDA not available in this environment"
    fi
    printf "\n"
}

run_post_install_tests() {
    [[ "$SKIP_TESTS" == "1" ]] && { dim "  skipping tests (--skip-tests)"; return; }
    [[ "$DRY_RUN" == "1" ]] && { dim "[dry-run] would run post-install tests"; return; }

    local dir="$INSTALL_DIR"
    local venv="${dir}/.venv/bin/activate"
    [[ ! -f "$venv" ]] && { warn "venv not found, skipping tests"; return; }

    # Only offer smoke test if unitree extras installed
    if ! { [[ "$EXTRAS" == *"unitree"* ]] || [[ "$EXTRAS" == "all" ]]; }; then
        dim "  no smoke tests available for selected extras"
        return
    fi

    if ! prompt_confirm "Run a quick smoke test? (starts unitree-go2 for 60s)" "yes"; then
        dim "  skipping smoke test"
        return
    fi

    printf "\n"; info "${BOLD}post-install smoke test${RESET}"; printf "\n"

    local cmd
    if [[ "$EXTRAS" == *"sim"* ]] || [[ "$EXTRAS" == "all" ]]; then
        cmd="dimos --simulation run unitree-go2"
    else
        cmd="dimos --replay run unitree-go2"
    fi

    info "running: ${DIM}${cmd}${RESET} (Ctrl+C to stop)"

    local exit_code=0
    pushd "$dir" >/dev/null
    source "$venv"
    $cmd &
    local pid=$!
    # wait allows bash to process INT trap immediately
    wait $pid || exit_code=$?
    popd >/dev/null

    printf "\n"
    if [[ $exit_code -eq 124 ]]; then ok "smoke test: ran 60s without crash ✓"
    elif [[ $exit_code -eq 130 ]] || [[ $exit_code -eq 137 ]]; then
        ok "smoke test: stopped by user"
    elif [[ $exit_code -eq 0 ]]; then ok "smoke test: completed ✓"
    else warn "smoke test exited with code ${exit_code} (may be expected headless)"
    fi
}

# ─── quickstart ───────────────────────────────────────────────────────────────
print_quickstart() {
    local dir="$INSTALL_DIR"
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
    [[ $ec -eq 130 ]] && { warn "interrupted"; }
    [[ $ec -ne 0 ]] && [[ $ec -ne 130 ]] && { printf "\n"; err "installation failed (exit ${ec})"; err "help: https://github.com/dimensionalOS/dimos/issues"; }
}
trap cleanup EXIT

# ─── main ─────────────────────────────────────────────────────────────────────
main() {
    parse_args "$@"

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
        [[ "$ver_major" -lt 22 ]] 2>/dev/null && warn "Ubuntu ${DETECTED_OS_VERSION} — 22.04+ recommended"
    fi
    [[ "$DETECTED_OS" == "macos" ]] && {
        local mac_major; mac_major="$(echo "$DETECTED_OS_VERSION" | cut -d. -f1)"
        [[ "$mac_major" -lt 12 ]] 2>/dev/null && die "macOS ${DETECTED_OS_VERSION} too old — 12.6+ required"
    }

    prompt_setup_method
    [[ "$SETUP_METHOD" != "nix" ]] && install_system_deps
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
    do_install
    configure_system
    verify_install
    run_post_install_tests
    print_quickstart
}

main "$@"
