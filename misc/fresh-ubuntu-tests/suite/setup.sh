#!/usr/bin/env bash
#
# Sets up the whole system in the fresh VM: the documented dimos install flow
# (docs/installation/ubuntu.md). Run first by run.sh; if it fails, no tests run.
# uv is already on PATH (run.sh sets it).

set -euxo pipefail
export GIT_LFS_SKIP_SMUDGE=1

# system dependencies (docs/installation/ubuntu.md)
sudo apt-get update
sudo apt-get install -y ca-certificates curl git g++ portaudio19-dev git-lfs libturbojpeg pre-commit libgl1 libegl1 libglib2.0-0 ffmpeg libsndfile1 pkg-config

# uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# clone + base sync
git clone https://github.com/dimensionalOS/dimos.git "$HOME/dimos"
cd "$HOME/dimos"
uv sync --all-groups
