#!/usr/bin/env bash
# Install dimup from this repository. DIMUP_REF selects a branch, tag, or commit.
set -euo pipefail
source_ref="${DIMUP_REF:-main}"
if ! command -v uv >/dev/null 2>&1; then
    curl --fail --show-error --location --proto '=https' --tlsv1.2 \
        https://astral.sh/uv/install.sh | env UV_NO_MODIFY_PATH=1 sh
    export PATH="$HOME/.local/bin:$PATH"
fi
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
curl --fail --show-error --location --proto '=https' --tlsv1.2 \
    "https://codeload.github.com/dimensionalOS/dimos/tar.gz/$source_ref" \
    --output "$work/source.tar.gz"
mkdir "$work/source"
tar -xzf "$work/source.tar.gz" --strip-components=1 -C "$work/source"
uv tool install --force --python 3.12 "$work/source/installer"
dimup_bin="$(uv tool dir --bin)/dimup"
if [[ -t 0 ]]; then
    "$dimup_bin" setup
elif (exec 3</dev/tty) 2>/dev/null; then
    "$dimup_bin" setup </dev/tty
else
    "$dimup_bin" setup
fi
printf '\nInstalled dimup. To use dimup in this terminal, run:\nexport PATH="%s:$HOME/.local/bin:$HOME/.cargo/bin:/opt/homebrew/bin:$PATH"\n' "$(uv tool dir --bin)"
