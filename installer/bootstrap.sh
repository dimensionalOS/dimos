#!/usr/bin/env bash
# This release template is stamped by run/package. It installs no permanent CLI.
set -euo pipefail
release='@DIMOS_RELEASE@'
base="https://github.com/dimensionalOS/dimos/releases/download/$release"
case "$(uname -s)/$(uname -m)" in
    Linux/x86_64) binary=create-dimos-linux-x86_64 ;;
    Linux/aarch64|Linux/arm64) binary=create-dimos-linux-aarch64 ;;
    *) echo 'Supported targets: Linux x86_64 and Jetson AGX Orin / JetPack 6.2.' >&2; exit 1 ;;
esac
if [[ "$release" == @* ]]; then
    echo 'Use the versioned bootstrap from a DimOS release, not the source template.' >&2
    exit 1
fi
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
for asset in "$binary" SHA256SUMS; do
    curl --fail --show-error --location --proto '=https' --tlsv1.2 \
        --connect-timeout 30 --max-time 180 "$base/$asset" --output "$work/$asset"
done
(cd "$work" && sha256sum --check --ignore-missing SHA256SUMS)
chmod +x "$work/$binary"
# A piped bootstrap consumes stdin. Reconnect the creator to the terminal for
# prompts when one exists; unattended users pass directory/profile explicitly.
if [[ -t 0 ]]; then
    "$work/$binary" "$@"
elif (exec 3</dev/tty) 2>/dev/null; then
    "$work/$binary" "$@" </dev/tty
else
    "$work/$binary" "$@"
fi
