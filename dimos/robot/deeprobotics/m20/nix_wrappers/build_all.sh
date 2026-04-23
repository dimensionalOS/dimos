#!/bin/bash
# Build all M20 NOS nix module wrappers, relink into dimos tree, pin as GC roots.
#
# Run this on NOS after a fresh checkout or after nix-collect-garbage has
# removed the module build outputs.
#
# Expects:
#   - /nix mounted on ext4 (bind-mounted from /var/opt/robot/data/nix)
#   - Internet access (to fetch upstream module sources)
#   - This checkout at /var/opt/robot/data/dimos (or DIMOS_ROOT env var)
set -e

DIMOS_ROOT="${DIMOS_ROOT:-/var/opt/robot/data/dimos}"
WRAPPER_DIR="${DIMOS_ROOT}/dimos/robot/deeprobotics/m20/nix_wrappers"
NIX_BIN="${NIX_BIN:-/nix/store/l3gszbzvrwsj98yijsn87r5sc8kwd83b-nix-2.20.9/bin/nix}"

if ! [ -x "$NIX_BIN" ]; then
    NIX_BIN="$(command -v nix)" || {
        echo "[build_all] ERROR: nix binary not found. Set NIX_BIN env var." >&2
        exit 1
    }
fi

mkdir -p /nix/var/nix/gcroots/custom

for mod in local_planner arise_slam terrain_analysis path_follower; do
    echo "=== Building $mod ==="
    cd "$WRAPPER_DIR/$mod"
    time "$NIX_BIN" \
        --extra-experimental-features 'nix-command flakes' \
        --option sandbox false \
        build . \
        --no-write-lock-file

    BIN_DIR="${DIMOS_ROOT}/dimos/navigation/smart_nav/modules/${mod}/result/bin"
    mkdir -p "$BIN_DIR"
    RESOLVED="$(readlink -f "$WRAPPER_DIR/$mod/result")"
    ln -sfn "$RESOLVED/bin/$mod" "$BIN_DIR/$mod"
    ln -sfn "$RESOLVED" "/nix/var/nix/gcroots/custom/$mod-current"
    echo "[build_all] $mod → $RESOLVED/bin/$mod (pinned)"
done

echo
echo "=== Done. Bindings: ==="
for mod in local_planner arise_slam terrain_analysis path_follower; do
    readlink "${DIMOS_ROOT}/dimos/navigation/smart_nav/modules/${mod}/result/bin/${mod}"
done
