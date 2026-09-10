#!/usr/bin/env bash
# Works directly from the repository; release packaging stamps a verified wheel.
set -euo pipefail
wheel_url='@DIMUP_WHEEL_URL@'
wheel_sha='@DIMUP_WHEEL_SHA@'
source_ref='8409a5c7fd9f88abb3fa855cd4bb38e06c2a3f4c'
if ! command -v uv >/dev/null 2>&1; then
    curl --fail --show-error --location --proto '=https' --tlsv1.2 \
        https://astral.sh/uv/install.sh | env UV_NO_MODIFY_PATH=1 sh
    export PATH="$HOME/.local/bin:$PATH"
fi
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
if [[ "$wheel_url" == @* ]]; then
    # A source archive needs neither Git nor a published release on a new machine.
    curl --fail --show-error --location --proto '=https' --tlsv1.2 \
        "https://codeload.github.com/dimensionalOS/dimos/tar.gz/$source_ref" \
        --output "$work/source.tar.gz"
    mkdir "$work/source"
    tar -xzf "$work/source.tar.gz" --strip-components=1 -C "$work/source"
    uv tool install --force --python 3.12 "$work/source/installer"
else
    curl --fail --show-error --location --proto '=https' --tlsv1.2 \
        "$wheel_url" --output "$work/${wheel_url##*/}"
    case "$(uname -s)" in
        Darwin) actual=$(shasum -a 256 "$work/${wheel_url##*/}") ;;
        Linux) actual=$(sha256sum "$work/${wheel_url##*/}") ;;
        *) echo 'Supported platforms: Ubuntu and Apple Silicon macOS.' >&2; exit 1 ;;
    esac
    [[ "${actual%% *}" == "$wheel_sha" ]] || { echo 'dimup checksum mismatch' >&2; exit 1; }
    uv tool install --force --python 3.12 "$work/${wheel_url##*/}"
fi
dimup_bin="$(uv tool dir --bin)/dimup"
if [[ -t 0 ]]; then
    "$dimup_bin" setup
elif (exec 3</dev/tty) 2>/dev/null; then
    "$dimup_bin" setup </dev/tty
else
    "$dimup_bin" setup
fi
printf '\nInstalled dimup. To use dimup in this terminal, run:\nexport PATH="%s:$HOME/.local/bin:$HOME/.cargo/bin:/opt/homebrew/bin:$PATH"\n' "$(uv tool dir --bin)"
