#!/usr/bin/env bash
# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0
# Fast checks: bash scripts/test-install.sh
# Real installation on a disposable host: INSTALL_TEST_ROOT=/tmp/dimos-test bash scripts/test-install.sh library|dev
# Test snippets expand inside the fresh Bash process.
# shellcheck disable=SC2016
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)

if [[ $# -gt 0 ]]; then
    [[ $# == 1 ]] || { printf "Usage: %s [library|dev]\n" "$0" >&2; exit 2; }
    mode=${1:?expected library or dev}
    case "$mode" in library|dev) ;; *) exit 2;; esac
    : "${INSTALL_TEST_ROOT:?set INSTALL_TEST_ROOT to an empty temporary directory}"
    mkdir -p "$INSTALL_TEST_ROOT/logs"
    project="$INSTALL_TEST_ROOT/$mode"
    export GIT_LFS_SKIP_SMUDGE=1
    export UV_PYTHON_PREFERENCE=only-managed
    export CUDA_VISIBLE_DEVICES=""
    unset VIRTUAL_ENV PYTHONPATH

    if [[ "$mode" == dev ]]; then
        expected_commit=$(git -C "$repo" rev-parse HEAD)
        git clone --no-hardlinks --no-checkout "$repo" "$project"
        git -C "$project" checkout --detach "$expected_commit"
    fi

    # The installer performs bounded CLI and capability dependency verification.
    # No blueprint is started. pipefail preserves installation errors.
    cat "$repo/scripts/install.sh" | /bin/bash -s -- \
        --non-interactive --no-nix --no-cuda --capabilities navigation,manipulation \
        --mode "$mode" --project-dir "$project" \
        2>&1 | tee "$INSTALL_TEST_ROOT/logs/install.log"
    exit
fi

installer="$repo/scripts/install.sh"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
export INSTALL_CLI_TEST_ROOT="$work"
# Ignore installer preferences inherited from the invoking shell.
for variable in DIMOS_INSTALL_MODE DIMOS_PROJECT_DIR DIMOS_CAPABILITIES DIMOS_BRANCH \
    DIMOS_NO_PROMPT DIMOS_NO_CUDA DIMOS_CONFIGURE_NETWORK DIMOS_USE_NIX DIMOS_NO_NIX DIMOS_DRY_RUN; do
    unset "$variable"
done

run() {
    bash -euo pipefail -c 'source "$1"; eval "$2"' _ "$installer" "$1"
}

pass() { printf 'PASS: %s\n' "$1"; }

reject() {
    local expected="$1" script="$2"
    if run "$script" >"$work/output" 2>&1; then
        printf 'Expected failure: %s\n' "$script" >&2
        exit 1
    fi
    if ! grep -F -- "$expected" "$work/output" >/dev/null; then
        cat "$work/output" >&2
        exit 1
    fi
}

for capabilities in navigation manipulation navigation,manipulation manipulation,navigation; do
    for mode in library dev; do
        run "parse_args --non-interactive --mode '$mode' --project-dir /tmp/example --capabilities '$capabilities'
            DETECTED_OS=ubuntu; DETECTED_ARCH=x86_64; DETECTED_GPU=none
            resolve_capabilities
            case '$capabilities' in
                navigation) expected=unitree,sim,drone,cpu ;;
                manipulation) expected=manipulation,cpu ;;
                *) expected=unitree,sim,drone,manipulation,cpu ;;
            esac
            [[ \"\$EXTRAS\" == \"\$expected\" ]]"
    done
done
pass 'capability mapping in both modes'

reject --mode 'parse_args --non-interactive --project-dir /tmp/example --capabilities navigation'
reject --project-dir 'parse_args --non-interactive --mode dev --capabilities navigation'
reject --capabilities 'parse_args --non-interactive --mode dev --project-dir /tmp/example'
for capabilities in '' all base 'navigation,' navigation,,manipulation navigation,navigation navigation,unknown; do
    reject 'select --capabilities' "CAPABILITIES='$capabilities'; validate_capabilities"
done
for flag in --extras --skip-tests --no-sysctl; do reject 'unknown option' "parse_args '$flag'"; done
reject 'requires a value' 'parse_args --capabilities --mode dev'
reject 'cannot be combined' 'parse_args --use-nix --no-nix'
reject 'boolean options' 'NO_CUDA=maybe; parse_args'
run 'CAPABILITIES=manipulation; parse_args --mode dev --project-dir . --capabilities navigation; [[ "$CAPABILITIES" == navigation ]]'
pass 'required arguments, invalid selections, removed flags, and flag precedence'

run 'CAPABILITIES=navigation; DETECTED_OS=ubuntu; DETECTED_ARCH=x86_64; DETECTED_GPU=nvidia
    resolve_capabilities; [[ "$BACKEND" == cuda ]]
    NO_CUDA=1; resolve_capabilities; [[ "$BACKEND" == cpu ]]
    NO_CUDA=0; DETECTED_ARCH=aarch64; resolve_capabilities; [[ "$BACKEND" == cpu ]]
    DETECTED_OS=macos; DETECTED_ARCH=arm64; DETECTED_GPU=apple-silicon
    resolve_capabilities; [[ "$BACKEND" == cpu ]]'
run 'NON_INTERACTIVE=1
    prompt_select() { exit 99; }
    for os in ubuntu wsl macos linux nixos; do
        DETECTED_OS=$os; HAS_NIX=1; USE_NIX=0; NO_NIX=0
        select_setup_method
        case "$os" in linux|nixos) [[ "$SETUP_METHOD" == nix ]] ;; *) [[ "$SETUP_METHOD" == system ]] ;; esac
    done
    DETECTED_OS=ubuntu; USE_NIX=1; select_setup_method; [[ "$SETUP_METHOD" == nix ]]
    USE_NIX=0; NO_NIX=1; DETECTED_OS=linux; select_setup_method; [[ "$SETUP_METHOD" == manual ]]'
run 'NON_INTERACTIVE=0; HAS_NIX=1
    for os in ubuntu wsl macos linux; do
        DETECTED_OS=$os; USE_NIX=0; NO_NIX=0
        prompt_select() { PROMPT_RESULT="$2"; }
        select_setup_method
        if [[ "$os" == linux ]]; then [[ "$SETUP_METHOD" == nix && "$USE_NIX" == 1 ]]
        else [[ "$SETUP_METHOD" == system && "$USE_NIX" == 0 ]]; fi
        USE_NIX=0
        prompt_select() { PROMPT_RESULT="$3"; }
        select_setup_method
        if [[ "$os" == linux ]]; then [[ "$SETUP_METHOD" == manual && "$USE_NIX" == 0 ]]
        else [[ "$SETUP_METHOD" == nix && "$USE_NIX" == 1 ]]; fi
    done
    prompt_select() { exit 99; }
    DETECTED_OS=nixos; USE_NIX=0; select_setup_method; [[ "$SETUP_METHOD" == nix ]]
    DETECTED_OS=ubuntu; USE_NIX=1; select_setup_method; [[ "$SETUP_METHOD" == nix ]]
    USE_NIX=0; NO_NIX=1; select_setup_method; [[ "$SETUP_METHOD" == system ]]'
pass 'backend and platform defaults and overrides'

# Cover fresh machines without downloading a tool during the fast tests.
mkdir -p "$work/gum-archive"
printf '#!/bin/sh\nexit 0\n' > "$work/gum-archive/gum"
tar czf "$work/gum.tar.gz" -C "$work/gum-archive" gum
run 'has_cmd() { [[ "$1" != gum ]] && command -v "$1" >/dev/null; }
    curl() { cat "$INSTALL_CLI_TEST_ROOT/gum.tar.gz"; }
    install_gum
    [[ -x "$GUM" ]]
    printf "%s" "$GUM_TEMP_DIR" > "$INSTALL_CLI_TEST_ROOT/gum-directory"'
[[ ! -e "$(cat "$work/gum-directory")" ]]
run 'gum() { :; }; install_gum; [[ "$GUM" == gum && -z "$GUM_TEMP_DIR" ]]'
pass 'Gum bootstrap, reuse, and temporary helper cleanup'


reject 'administrator access required' 'NON_INTERACTIVE=1; id() { echo 1000; }; sudo() { [[ "$*" == "-n -v" ]]; return 1; }; require_admin apt'
run 'id() { echo 1000; }; run_cmd() { [[ "$*" == "sudo -n apt-get update" ]]; }; run_privileged apt-get update'
run 'id() { echo 0; }; run_cmd() { [[ "$*" == "apt-get update" ]]; }; run_privileged apt-get update'
reject 'STOP_APT' 'SETUP_METHOD=system; DETECTED_OS=ubuntu; NEEDED_PACKAGES=(example)
    require_admin() { :; }; run_privileged() { die STOP_APT; }; install_system_deps; echo UNREACHABLE'
run 'CONFIGURE_NETWORK=0; run_privileged() { exit 99; }; configure_system'
run 'CONFIGURE_NETWORK=1; DETECTED_OS=ubuntu; DRY_RUN=1
    configure_system' >"$work/output"
grep -F 'net.core.rmem_max=67108864 net.core.rmem_default=67108864' "$work/output" >/dev/null
grep -F '/etc/sysctl.d/99-dimos.conf' "$work/output" >/dev/null
pass 'privilege failures, subprocess failures, and opt-in network changes'

# Existing Nix installations may have both experimental features disabled.
mkdir -p "$work/nix-bin"
cat > "$work/nix-bin/nix" <<'NIX'
#!/usr/bin/env bash
set -eu
[[ "$1" == --extra-experimental-features && "$2" == 'nix-command flakes' ]]
shift 2
[[ "$1" == develop && "$2" == --command ]]
shift 2
exec "$@"
NIX
chmod +x "$work/nix-bin/nix"
run 'export PATH="$INSTALL_CLI_TEST_ROOT/nix-bin:$PATH"
    USE_NIX=1; INSTALL_DIR="$INSTALL_CLI_TEST_ROOT"
    [[ "$(project_cmd printf "%s" "argument with spaces")" == "argument with spaces" ]]'
pass 'Nix commands explicitly enable required features and preserve arguments'

# Exercise the real main flow with dependency execution mocked, not the argument parser.
run 'detect_os() { DETECTED_OS=ubuntu; DETECTED_OS_VERSION=24.04; DETECTED_ARCH=x86_64; DETECTED_DISK_GB=50; }
    detect_gpu() { DETECTED_GPU=none; }; detect_python() { :; }; detect_nix() { :; }
    find_system_packages() { NEEDED_PACKAGES=(); }
    install_uv() { :; }
    main --dry-run --mode library --project-dir "$INSTALL_CLI_TEST_ROOT/new project" --capabilities navigation
    [[ ! -e "$INSTALL_CLI_TEST_ROOT/new project" ]]' >"$work/output"
grep -F 'dimos[unitree,sim,drone,cpu]' "$work/output" >/dev/null
pass 'main dry-run leaves project absent'

run 'mkdir -p "$INSTALL_CLI_TEST_ROOT/existing/.venv"
    INSTALL_DIR="$INSTALL_CLI_TEST_ROOT/existing"; PROJECT_DIR="$INSTALL_DIR"; EXTRAS=manipulation,cpu
    project_cmd() { [[ "$1 $2" == "uv pip" ]]; }
    do_install_library' >"$work/output"
run 'INSTALL_MODE=dev; PROJECT_DIR=$PWD; prepare_directory
    branch=$(git branch --show-current); EXTRAS=unitree,sim,drone,cpu
    project_cmd() { [[ "$*" == "/usr/bin/env UV_PROJECT_ENVIRONMENT=.venv uv sync --locked --python 3.12 --group tests --group lint --extra unitree --extra sim --extra drone --extra cpu" ]]; }
    do_install_dev; [[ "$(git branch --show-current)" == "$branch" ]]' >"$work/output"
reject 'no usable Python' 'INSTALL_DIR="$INSTALL_CLI_TEST_ROOT/existing"; check_existing_environment'
run 'mkdir -p "$INSTALL_CLI_TEST_ROOT/invalid/.venv/bin"
    printf "#!/bin/sh\nexit 1\n" > "$INSTALL_CLI_TEST_ROOT/invalid/.venv/bin/python"
    chmod +x "$INSTALL_CLI_TEST_ROOT/invalid/.venv/bin/python"'
reject 'must use Python 3.12' 'INSTALL_DIR="$INSTALL_CLI_TEST_ROOT/invalid"; check_existing_environment'
pass 'reuse environments/checkouts and retain contributor groups'

reject 'verification failed: native libraries' 'run_bounded() { return 8; }; check_install "native libraries" false'
run 'CAPABILITIES=manipulation; BACKEND=cpu; INSTALL_DIR=/tmp/example
    run_bounded() { printf "%s\n" "$*"; }
    verify_install' >"$work/output"
grep -F 'roboplan.core' "$work/output" >/dev/null
if grep -E 'unitree_webrtc|--replay|from_file|\.build\(' "$work/output"; then exit 1; fi
pass 'bounded failure propagation and capability-specific asset-free verification'

for capabilities in navigation manipulation navigation,manipulation; do
    for nix in 0 1; do
        run "CAPABILITIES='$capabilities'; USE_NIX=$nix; INSTALL_DIR='/tmp/dimos example'; print_quickstart" >"$work/output"
        if [[ ",$capabilities," == *,navigation,* ]]; then
            grep -Fx '  dimos --replay run unitree-go2' "$work/output" >/dev/null
        elif grep -F 'unitree-go2' "$work/output"; then exit 1; fi
        if [[ ",$capabilities," == *,manipulation,* ]]; then
            grep -Fx '  dimos run keyboard-teleop-xarm7' "$work/output" >/dev/null
        elif grep -F 'keyboard-teleop-xarm7' "$work/output"; then exit 1; fi
        if [[ "$nix" == 1 ]]; then grep -Fx '  nix --extra-experimental-features "nix-command flakes" develop' "$work/output" >/dev/null
        elif grep -F 'nix --extra-experimental-features' "$work/output"; then exit 1; fi
        if grep -E -- '--viewer none|xarm-perception-sim' "$work/output"; then exit 1; fi
    done
done
grep -F 'dimos --replay run unitree-go2' "$repo/README.md" >/dev/null
grep -F 'dimos run keyboard-teleop-xarm7' "$repo/README.md" >/dev/null
pass 'README examples and activation instructions for each capability/setup combination'

run 'install_gum() { exit 99; }; prompt_select() { exit 99; }
    detect_os() { DETECTED_OS=ubuntu; DETECTED_OS_VERSION=24.04; DETECTED_ARCH=x86_64; }
    detect_gpu() { DETECTED_GPU=none; }; detect_python() { :; }; detect_nix() { :; }
    check_disk_space() { :; }; find_system_packages() { NEEDED_PACKAGES=(); }
    install_system_deps() { :; }; install_uv() { :; }; do_install() { :; }
    verify_install() { echo VERIFIED; }
    main --non-interactive --mode library --project-dir "$INSTALL_CLI_TEST_ROOT/unattended" --capabilities navigation,manipulation' </dev/null >"$work/output"
grep -Fx VERIFIED "$work/output" >/dev/null
if grep -F 'Install this environment?' "$work/output"; then exit 1; fi
pass 'unattended setup verifies without prompting'
