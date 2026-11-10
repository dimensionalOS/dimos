#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Set up this PC to host the Metal autoresearch lab and its Desktop UI.
#
# Needs DimOS Desktop installed and running, plus git, uv, nix and npm, and about 15 GB free.
# Usage: bash setup_autoresearch_host.sh [dimos folder, default ~/dimos-autoresearch]
# Then put an LLM key in <dimos folder>/.env and open Desktop -> Manipulation.
set -euo pipefail

DIMOS_DIR="${1:-$HOME/dimos-autoresearch}"
DIMOS_BRANCH="hackathon/feat/metal-autoresearch"
APP_DIR="$HOME/.dimos/apps/dim-manipulation"
APP_REF="mb/feat/metal-autoresearch"
DESKTOP="http://127.0.0.1:5555"

step() { printf '\n==> %s\n' "$*"; }

for tool in git uv nix npm curl dimos-desktop; do
    command -v "$tool" >/dev/null || { echo "missing: $tool" >&2; exit 1; }
done
curl -fsS -o /dev/null "$DESKTOP/" || { echo "DimOS Desktop is not running at $DESKTOP" >&2; exit 1; }

step "dimos at $DIMOS_BRANCH in $DIMOS_DIR"
if [ -d "$DIMOS_DIR/.git" ]; then
    git -C "$DIMOS_DIR" fetch origin "$DIMOS_BRANCH"
    git -C "$DIMOS_DIR" checkout "$DIMOS_BRANCH"
    git -C "$DIMOS_DIR" pull --ff-only origin "$DIMOS_BRANCH"
else
    GIT_LFS_SKIP_SMUDGE=1 git clone -b "$DIMOS_BRANCH" https://github.com/dimensionalOS/dimos.git "$DIMOS_DIR"
fi

step "Python environment (Desktop runs $DIMOS_DIR/.venv/bin/dimos)"
(cd "$DIMOS_DIR" && uv venv --allow-existing --python 3.12 && uv sync --locked --python 3.12 --extra all)

step "Obstacle-map helper (Rust, built with nix)"
(cd "$DIMOS_DIR/dimos/mapping/ray_tracing/rust" && nix --extra-experimental-features 'nix-command flakes' build -L path:.)

step "Pi, the coding agent that writes the programs"
command -v pi >/dev/null || npm install -g @earendil-works/pi-coding-agent

step "Offscreen rendering for the simulator (Desktop's service has no display)"
touch "$DIMOS_DIR/.env"
grep -q '^MUJOCO_GL=' "$DIMOS_DIR/.env" || echo 'MUJOCO_GL=egl' >> "$DIMOS_DIR/.env"

step "Manipulation app at $APP_REF"
if [ -d "$APP_DIR/.git" ]; then
    git -C "$APP_DIR" fetch origin "$APP_REF"
    git -C "$APP_DIR" checkout "$APP_REF"
    git -C "$APP_DIR" pull --ff-only origin "$APP_REF"
    curl -fsS -X POST "$DESKTOP/gateways/desktop/management/apps/dim-manipulation/reload" >/dev/null
else
    dimos-desktop install --name dim-manipulation --ref "$APP_REF" https://github.com/dimensionalOS/dim-manipulation
fi

step "Point Desktop at $DIMOS_DIR"
curl -fsS -X PUT -H 'content-type: application/json' \
    "$DESKTOP/gateways/desktop/management/gateways/dimos/checkout" \
    -d "{\"dir\": \"$DIMOS_DIR\"}" >/dev/null

cat <<EOF

Done. Next:
  1. Add your LLM key:  echo 'OPENAI_API_KEY=sk-...' >> $DIMOS_DIR/.env
  2. Share it:          dimos-desktop password   (others open http://<this PC>:5555)
  3. Desktop -> Manipulation -> Application: Autoresearch, Robot: Metal -> Launch session
EOF
