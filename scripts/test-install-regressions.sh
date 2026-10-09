#!/usr/bin/env bash
# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0
# Temporary Git repositories and mocked package commands; no network or system changes.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
installer="$repo/scripts/install.sh"
root=$(mktemp -d)
trap 'rm -rf "$root"' EXIT
fail() { printf 'FAIL: %s\n' "$1" >&2; exit 1; }

seed="$root/seed"
remote="$root/remote.git"
git init -q -b main "$seed"
printf 'first\n' > "$seed/version"
printf 'keep\n' > "$seed/local"
git -C "$seed" add .
git -C "$seed" -c user.name=Test -c user.email=test@example.invalid -c commit.gpgsign=false commit -qm first
first=$(git -C "$seed" rev-parse HEAD)
printf 'second\n' > "$seed/version"
git -C "$seed" -c user.name=Test -c user.email=test@example.invalid -c commit.gpgsign=false commit -qam second
second=$(git -C "$seed" rev-parse HEAD)
git clone -q --bare "$seed" "$remote"
# Redirect the production clone URL without altering the user's Git configuration.
export GIT_CONFIG_COUNT=1
export GIT_CONFIG_KEY_0="url.file://$remote.insteadOf"
export GIT_CONFIG_VALUE_0="https://github.com/dimensionalOS/dimos.git"

run_dev() {
    local project=$1 pin=$2
    DIMOS_PROJECT_DIR="$project" DIMOS_COMMIT="$pin" SYNC_LOG="$project.sync" \
        bash -c '
            source "$1"
            EXTRAS=base USE_NIX=0 INSTALL_PYTHON=python3 DRY_RUN=0
            prompt_confirm() { return 0; }
            project_cmd() {
                [[ "$1 $2" == "uv sync" ]] || exit 1
                git -C "$INSTALL_DIR" rev-parse HEAD > "$SYNC_LOG"
            }
            do_install_dev
        ' _ "$installer" > "$root/output" 2>&1
}
assert_synced() {
    [[ $(cat "$1.sync") == "$2" ]] || fail "uv sync used the wrong revision"
    [[ $(git -C "$1" rev-parse HEAD) == "$2" ]] || fail "wrong checkout revision"
}

run_dev "$root/fresh install" "$first"
assert_synced "$root/fresh install" "$first"
printf 'PASS: fresh checkout uses the pin before sync\n'

git clone -q --depth 1 "file://$remote" "$root/shallow"
if git -C "$root/shallow" cat-file -e "${first}^{commit}" 2>/dev/null; then
    fail "shallow fixture unexpectedly has the old revision"
fi
run_dev "$root/shallow" "$first"
assert_synced "$root/shallow" "$first"
printf 'PASS: existing checkout fetches a missing pin\n'

git clone -q "$remote" "$root/unpinned"
run_dev "$root/unpinned" ""
assert_synced "$root/unpinned" "$second"
[[ $(git -C "$root/unpinned" symbolic-ref --short HEAD) == main ]] || fail "unpinned branch changed"
printf 'PASS: unpinned existing checkout preserves its branch\n'

run_dev "$root/fresh-unpinned" ""
assert_synced "$root/fresh-unpinned" "$second"
[[ $(git -C "$root/fresh-unpinned" symbolic-ref --short HEAD) == main ]] || fail "unpinned clone detached"
printf 'PASS: unpinned fresh checkout preserves its branch\n'

git clone -q "$remote" "$root/conflict"
printf 'local work\n' > "$root/conflict/version"
if run_dev "$root/conflict" "$first"; then fail "conflicting local changes were accepted"; fi
[[ ! -e "$root/conflict.sync" ]] || fail "sync ran after a checkout conflict"
[[ $(cat "$root/conflict/version") == 'local work' ]] || fail "local work was overwritten"
[[ $(git -C "$root/conflict" rev-parse HEAD) == "$second" ]] || fail "conflict changed HEAD"
printf 'PASS: conflicting local changes fail before sync and remain intact\n'

git clone -q "$remote" "$root/dirty-safe"
printf 'keep local work\n' > "$root/dirty-safe/local"
run_dev "$root/dirty-safe" "$first"
assert_synced "$root/dirty-safe" "$first"
[[ $(cat "$root/dirty-safe/local") == 'keep local work' ]] || fail "nonconflicting local work was lost"
printf 'PASS: nonconflicting local changes remain intact\n'

git clone -q "$remote" "$root/missing"
if run_dev "$root/missing" aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa; then fail "missing pin succeeded"; fi
[[ ! -e "$root/missing.sync" ]] || fail "sync ran after a missing pin"
[[ $(git -C "$root/missing" rev-parse HEAD) == "$second" ]] || fail "missing pin changed HEAD"
printf 'PASS: unavailable pin fails before sync\n'

# Simulate an interrupted/no-op checkout and prove the explicit HEAD guard stops sync.
real_git=$(command -v git)
mkdir "$root/mock-bin"
# shellcheck disable=SC2016
printf '#!/bin/sh\nif [ "$3" = checkout ]; then exit 0; fi\nexec "%s" "$@"\n' "$real_git" > "$root/mock-bin/git"
chmod +x "$root/mock-bin/git"
git clone -q "$remote" "$root/no-checkout"
if PATH="$root/mock-bin:$PATH" run_dev "$root/no-checkout" "$first"; then fail "incorrect HEAD was accepted"; fi
[[ ! -e "$root/no-checkout.sync" ]] || fail "HEAD mismatch reached sync"
printf 'PASS: explicit HEAD verification rejects an unsuccessful checkout\n'

# Check the CLI and environment pin interfaces without running installation.
DIMOS_COMMIT="$first" bash -c 'source "$1"; [[ "$GIT_COMMIT" == "$2" ]]; parse_args --commit "$3"; [[ "$GIT_COMMIT" == "$3" ]]' \
    _ "$installer" "$first" "$second"
printf 'PASS: --commit overrides the environment pin\n'

mkdir -p "$root/include/linux"
touch "$root/include/stdio.h" "$root/include/linux/types.h" "$root/include/portaudio.h"
sed "s@/usr/include/@$root/include/@g" "$installer" > "$root/probe-installer.sh"
run_probe() {
    CPP_AVAILABLE=$1 CPP_STATUS=$2 CPP_SOURCE="$root/cpp-source" PACKAGES_LOG="$root/packages" \
        bash -c '
            source "$1"
            has_cmd() { [[ "$1" != g++ || "$CPP_AVAILABLE" == 1 ]]; }
            g++() { cat > "$CPP_SOURCE"; return "$CPP_STATUS"; }
            id() { echo 0; }
            steamos-readonly() { echo disabled; }
            prompt_confirm() { return 0; }
            run_cmd() { printf "%s\n" "$*" >> "$PACKAGES_LOG"; }
            install_steamos_deps
        ' _ "$root/probe-installer.sh" > "$root/output" 2>&1
}
run_probe 1 0
[[ ! -e "$root/packages" ]] || fail "working toolchain triggered packages"
grep -q '#include <vector>' "$root/cpp-source" || fail "C++ standard-library header was not compiled"
printf 'PASS: working C++ standard library skips package installation\n'

rm "$root/cpp-source"
run_probe 0 0
[[ -s "$root/packages" && ! -e "$root/cpp-source" ]] || fail "missing g++ skipped package installation"
printf 'PASS: missing g++ triggers package installation\n'

rm "$root/packages"
run_probe 1 1
[[ -s "$root/packages" && -s "$root/cpp-source" ]] || fail "broken C++ headers skipped package installation"
printf 'PASS: failing C++ compile triggers package installation\n'
