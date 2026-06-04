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

if [ "$#" -gt 0 ]; then
    MODULES=("$@")
else
    MODULES=(local_planner arise_slam terrain_analysis path_follower far_planner tare_planner pgo)
fi

for mod in "${MODULES[@]}"; do
    echo "=== Building $mod ==="
    BUILD_DIR="/tmp/m20-nix-wrapper-${mod}"
    rm -rf "$BUILD_DIR"
    cp -R "$WRAPPER_DIR/$mod" "$BUILD_DIR"
    if [ "$mod" = "pgo" ]; then
        mkdir -p "$BUILD_DIR/source"
        cp -R "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/pgo/cpp/." "$BUILD_DIR/source/"
        rm -rf "$BUILD_DIR/source/result" "$BUILD_DIR/source/build"
    fi
    cd "$BUILD_DIR"
    time "$NIX_BIN" \
        --extra-experimental-features 'nix-command flakes' \
        --option sandbox false \
        build "path:$BUILD_DIR" \
        --no-write-lock-file \
        -o "$WRAPPER_DIR/$mod/result"

    RESOLVED="$(readlink -f "$WRAPPER_DIR/$mod/result")"
    link_bin() {
        local bin_name="$1"
        local bin_dir="$2"
        mkdir -p "$bin_dir"
        ln -sfn "$RESOLVED/bin/$bin_name" "$bin_dir/$bin_name"
    }

    case "$mod" in
        pgo)
            link_bin pgo "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/pgo/cpp/result/bin"
            ;;
        far_planner)
            link_bin far_planner "${DIMOS_ROOT}/dimos/navigation/smart_nav/modules/far_planner/result/bin"
            if [ -f "$RESOLVED/bin/far_planner_native" ]; then
                link_bin far_planner_native "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/far_planner/result/bin"
            else
                mkdir -p "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/far_planner/result/bin"
                ln -sfn "$RESOLVED/bin/far_planner" \
                    "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/far_planner/result/bin/far_planner_native"
            fi
            ;;
        arise_slam)
            link_bin "$mod" "${DIMOS_ROOT}/dimos/navigation/smart_nav/modules/${mod}/result/bin"
            ;;
        *)
            link_bin "$mod" "${DIMOS_ROOT}/dimos/navigation/smart_nav/modules/${mod}/result/bin"
            if [ -d "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/${mod}" ]; then
                link_bin "$mod" "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/${mod}/result/bin"
            fi
            ;;
    esac
    ln -sfn "$RESOLVED" "/nix/var/nix/gcroots/custom/$mod-current"
    echo "[build_all] $mod → $RESOLVED/bin/$mod (pinned)"
done

echo
echo "=== Done. Bindings: ==="
for mod in "${MODULES[@]}"; do
    case "$mod" in
        pgo)
            readlink "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/pgo/cpp/result/bin/pgo"
            ;;
        far_planner)
            readlink "${DIMOS_ROOT}/dimos/navigation/smart_nav/modules/far_planner/result/bin/far_planner"
            readlink "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/far_planner/result/bin/far_planner_native"
            ;;
        arise_slam)
            readlink "${DIMOS_ROOT}/dimos/navigation/smart_nav/modules/${mod}/result/bin/${mod}"
            ;;
        *)
            readlink "${DIMOS_ROOT}/dimos/navigation/smart_nav/modules/${mod}/result/bin/${mod}"
            if [ -d "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/${mod}" ]; then
                readlink "${DIMOS_ROOT}/dimos/navigation/nav_stack/modules/${mod}/result/bin/${mod}"
            fi
            ;;
    esac
done
