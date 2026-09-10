#!/usr/bin/env bash
# Stamped with immutable release asset URLs by installer/run/package.
set -euo pipefail
wheel_url='@DIMUP_WHEEL_URL@'
wheel_sha='@DIMUP_WHEEL_SHA@'
if [[ "$wheel_url" == @* ]]; then
    echo 'Use dimup.sh from a published DimOS release.' >&2
    exit 1
fi
if ! command -v uv >/dev/null 2>&1; then
    curl --fail --show-error --location --proto '=https' --tlsv1.2 \
        https://astral.sh/uv/install.sh | env UV_NO_MODIFY_PATH=1 sh
    export PATH="$HOME/.local/bin:$PATH"
fi
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
curl --fail --show-error --location --proto '=https' --tlsv1.2 \
    "$wheel_url" --output "$work/${wheel_url##*/}"
case "$(uname -s)" in
    Darwin) actual=$(shasum -a 256 "$work/${wheel_url##*/}") ;;
    Linux) actual=$(sha256sum "$work/${wheel_url##*/}") ;;
    *) echo 'Supported platforms: Ubuntu and Apple Silicon macOS.' >&2; exit 1 ;;
esac
[[ "${actual%% *}" == "$wheel_sha" ]] || { echo 'dimup checksum mismatch' >&2; exit 1; }
uv tool install --force --python 3.12 "$work/${wheel_url##*/}"
dimup_bin="$(uv tool dir --bin)/dimup"
if [[ -t 0 ]]; then
    "$dimup_bin" setup
elif (exec 3</dev/tty) 2>/dev/null; then
    "$dimup_bin" setup </dev/tty
else
    "$dimup_bin" setup
fi
printf '\nInstalled dimup. To use the prepared tools in this terminal, run:\nexport PATH="%s:$HOME/.local/bin:$HOME/.cargo/bin:/opt/homebrew/bin:$PATH"\n' "$(uv tool dir --bin)"
