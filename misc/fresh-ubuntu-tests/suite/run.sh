#!/usr/bin/env bash
#
# Runs the fresh-ubuntu test suite INSIDE the VM (invoked by vmtest.sh).
#
# setup.sh must pass, or the suite stops. Then every tests/*.sh runs in turn,
# each to its own log in ./logs/, and a failing test does NOT stop the others.
# Add a test by dropping another tests/NN-name.sh -- NN sets the order, which can
# matter since the tests share one VM (e.g. they reuse the same .venv).
#
# run.sh provides the environment so each test can be just the command(s): uv is
# on PATH, and tests run with the cwd set to the cloned repo.

set -uo pipefail
shopt -s nullglob

SUITE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; readonly SUITE_DIR
readonly LOGS="$SUITE_DIR/logs"
readonly REPO="$HOME/dimos"
export PATH="$HOME/.local/bin:$PATH"

rm -rf "$LOGS"; mkdir -p "$LOGS"
hdr() { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }

# setup is gating: if it fails, run nothing else.
hdr "setup"
if ! bash "$SUITE_DIR/setup.sh" 2>&1 | tee "$LOGS/setup.log"; then
  echo "setup failed -- aborting suite"
  exit 1
fi

# each test is independent: run it in the repo, log it, keep going on failure.
names=(); codes=()
for t in "$SUITE_DIR"/tests/*.sh; do
  name="$(basename "$t" .sh)"
  hdr "test: $name"
  if (cd "$REPO" && bash "$t") 2>&1 | tee "$LOGS/$name.log"; then
    codes+=(0)
  else
    codes+=("${PIPESTATUS[0]}")
  fi
  names+=("$name")
done

hdr "summary"
fail=0
for i in "${!names[@]}"; do
  if [[ "${codes[$i]}" -eq 0 ]]; then
    printf '  PASS  %s\n' "${names[$i]}"
  else
    printf '  FAIL  %s (exit %s)\n' "${names[$i]}" "${codes[$i]}"
    fail=1
  fi
done
exit "$fail"
