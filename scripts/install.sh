#!/usr/bin/env bash
# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0
#
# Bootstrapper for the DimOS CLI: download the `dimos` binary for this platform and run `dimos setup`.
#
#   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
#   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --mode dev --extras base
#
# Arguments after `-s --` are passed verbatim to `dimos setup`.
#
# DIMOS_VERSION        release tag to install (default: latest)
# DIMOS_INSTALLER_URL  directory holding dimos-<target> and dimos-<target>.sha256 (LAN/CI override)

set -euo pipefail
trap 'exit 130' INT

need() { command -v "$1" >/dev/null 2>&1 || { echo "$1 is required" >&2; exit 1; }; }
need curl

case "$(uname -s)-$(uname -m)" in
    Darwin-arm64)              target=aarch64-apple-darwin ;;
    Linux-x86_64)              target=x86_64-linux-musl ;;
    Linux-aarch64|Linux-arm64) target=aarch64-linux-musl ;;
    Darwin-x86_64)
        echo "no Intel macOS build. Install by hand:" >&2
        echo "  uv venv --python 3.12 && uv pip install 'dimos[base]'" >&2
        exit 1 ;;
    *)
        echo "unsupported platform: $(uname -s) $(uname -m) (supported: macOS arm64, Linux x86_64/aarch64)" >&2
        exit 1 ;;
esac

# library mode installs dimos==<version> from PyPI, so the tag must already carry a published release
if [ -n "${DIMOS_INSTALLER_URL:-}" ]; then
    base="$DIMOS_INSTALLER_URL"
elif [ -n "${DIMOS_VERSION:-}" ]; then
    base="https://github.com/dimensionalOS/dimos/releases/download/v${DIMOS_VERSION#v}"
else
    base="https://github.com/dimensionalOS/dimos/releases/latest/download"
fi

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

asset="dimos-$target"
for f in "$asset" "$asset.sha256"; do
    curl -fsSL --retry 3 "$base/$f" -o "$tmp/$f" || {
        echo "download failed: $base/$f" >&2
        echo "no installer asset in this release: set DIMOS_VERSION to a tag that has one, or DIMOS_INSTALLER_URL to a directory holding it" >&2
        exit 1
    }
done
[ -s "$tmp/$asset" ] || { echo "empty download: $base/$asset" >&2; exit 1; }

want=$(awk '{print tolower($1)}' "$tmp/$asset.sha256")
if command -v sha256sum >/dev/null 2>&1; then
    got=$(sha256sum "$tmp/$asset" | awk '{print $1}')
else
    need shasum
    got=$(shasum -a 256 "$tmp/$asset" | awk '{print $1}')
fi
[ "$got" = "$want" ] || { echo "sha256 mismatch for $asset: got $got want $want" >&2; exit 1; }
chmod 0755 "$tmp/$asset"

echo "-> $("$tmp/$asset" --version)"

rc=0
# curl | bash leaves stdin on the pipe; /dev/tty is what keeps setup's prompts readable
if [ ! -t 0 ] && ( : </dev/tty ) 2>/dev/null; then
    "$tmp/$asset" setup "$@" </dev/tty || rc=$?
else
    "$tmp/$asset" setup "$@" || rc=$?
fi
exit "$rc"
