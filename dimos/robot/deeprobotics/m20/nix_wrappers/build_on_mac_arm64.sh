#!/bin/bash
# Build M20 aarch64-linux nix wrappers on an Apple Silicon Mac via OrbStack/Docker.
#
# This creates a persistent linux/arm64 Nix container, runs build_all.sh inside
# it, and can optionally copy the resulting /nix/store closures to NOS.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

CONTAINER="${M20_NIX_BUILDER_CONTAINER:-m20-nix-builder}"
IMAGE="${M20_NIX_BUILDER_IMAGE:-nixos/nix:latest}"
CONTAINER_DIMOS_ROOT="/work/dimos"
COPY_TO_NOS=0
ENSURE_ONLY=0
REMOTE_HOST=""
NOS_HOST="${NOS_HOST:-10.21.31.106}"
NOS_USER="${NOS_USER:-user}"
AOS_WIFI_JUMP="${AOS_WIFI_JUMP:-user@10.21.41.1}"
NOS_NIX_STORE_BIN="${NOS_NIX_STORE_BIN:-/nix/store/l3gszbzvrwsj98yijsn87r5sc8kwd83b-nix-2.20.9/bin/nix-store}"

usage() {
    cat <<EOF
Usage: $0 [options] [module ...]

Build M20 native module wrappers in a persistent linux/arm64 Nix container.
Default modules match build_all.sh.

Options:
  --copy-to-nos          Copy built Nix closures to NOS and relink/pin outputs
  --host <hostname>      Robot Tailscale host for ProxyJump, e.g. m20-770-gogo
  --nos-host <host>      NOS host inside the dog network (default: ${NOS_HOST})
  --nos-user <user>      NOS SSH user (default: ${NOS_USER})
  --container <name>     Docker container name (default: ${CONTAINER})
  --image <image>        Docker image (default: ${IMAGE})
  --ensure-only          Create/start the builder and print its platform, no build
  -h, --help             Show this help

Examples:
  $0 --ensure-only
  $0 pgo
  $0 --copy-to-nos --host m20-770-gogo pgo
EOF
}

positional=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --copy-to-nos)
            COPY_TO_NOS=1
            shift
            ;;
        --host)
            REMOTE_HOST="$2"
            shift 2
            ;;
        --nos-host)
            NOS_HOST="$2"
            shift 2
            ;;
        --nos-user)
            NOS_USER="$2"
            shift 2
            ;;
        --container)
            CONTAINER="$2"
            shift 2
            ;;
        --image)
            IMAGE="$2"
            shift 2
            ;;
        --ensure-only)
            ENSURE_ONLY=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            positional+=("$1")
            shift
            ;;
    esac
done

if [ "${#positional[@]}" -gt 0 ]; then
    MODULES=("${positional[@]}")
else
    MODULES=(local_planner arise_slam terrain_analysis path_follower far_planner tare_planner pgo)
fi

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "[build_on_mac_arm64] ERROR: missing required command: $1" >&2
        exit 1
    fi
}

shell_quote_join() {
    local out=""
    local arg
    for arg in "$@"; do
        printf -v out "%s %q" "$out" "$arg"
    done
    printf '%s' "$out"
}

ensure_builder() {
    require_cmd docker
    if ! docker info >/dev/null 2>&1; then
        echo "[build_on_mac_arm64] ERROR: Docker/OrbStack is not running." >&2
        echo "Start OrbStack, then rerun this script." >&2
        exit 1
    fi

    if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
        docker pull --platform linux/arm64 "$IMAGE"
    fi

    if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
        ssh_mount=()
        if [ -d "${HOME}/.ssh" ]; then
            ssh_mount=(-v "${HOME}/.ssh:/root/.ssh:ro")
        fi
        docker create \
            --platform linux/arm64 \
            --name "$CONTAINER" \
            -v "${DIMOS_ROOT}:${CONTAINER_DIMOS_ROOT}" \
            "${ssh_mount[@]}" \
            -w "$CONTAINER_DIMOS_ROOT" \
            "$IMAGE" \
            sleep infinity >/dev/null
    fi

    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != "true" ]; then
        docker start "$CONTAINER" >/dev/null
    fi
}

docker_exec() {
    docker exec \
        -e DIMOS_ROOT="$CONTAINER_DIMOS_ROOT" \
        -e NIX_CONFIG="experimental-features = nix-command flakes" \
        "$CONTAINER" "$@"
}

remote_jump() {
    if [ -n "$REMOTE_HOST" ]; then
        printf '%s@%s' "$NOS_USER" "$REMOTE_HOST"
    else
        printf '%s' "$AOS_WIFI_JUMP"
    fi
}

resolve_host_ip() {
    local host="$1"
    if command -v dscacheutil >/dev/null 2>&1; then
        dscacheutil -q host -a name "$host" 2>/dev/null | awk '/ip_address:/ {print $2; exit}'
    fi
}

container_remote_jump() {
    if [ -n "$REMOTE_HOST" ]; then
        local jump_addr
        jump_addr="$(resolve_host_ip "$REMOTE_HOST")"
        printf '%s@%s' "$NOS_USER" "${jump_addr:-$REMOTE_HOST}"
    else
        printf '%s' "$AOS_WIFI_JUMP"
    fi
}

remote_ssh() {
    ssh -o "ProxyJump=$(remote_jump)" "${NOS_USER}@${NOS_HOST}" "$@"
}

module_result_paths() {
    local module_args
    module_args="$(shell_quote_join "${MODULES[@]}")"
    docker_exec bash -lc "
        set -e
        wrapper_dir=${CONTAINER_DIMOS_ROOT}/dimos/robot/deeprobotics/m20/nix_wrappers
        for mod in${module_args}; do
            printf '%s %s\n' \"\$mod\" \"\$(readlink -f \"\$wrapper_dir/\$mod/result\")\"
        done
    "
}

copy_to_nos() {
    local paths=()
    local line mod path
    while read -r mod path; do
        [ -n "$mod" ] || continue
        paths+=("$path")
    done < <(module_result_paths)

    if [ "${#paths[@]}" -eq 0 ]; then
        echo "[build_on_mac_arm64] ERROR: no result paths found to copy." >&2
        exit 1
    fi

    local path_args
    path_args="$(shell_quote_join "${paths[@]}")"
    local ssh_opts
    ssh_opts="-o ProxyJump=$(container_remote_jump) -o UserKnownHostsFile=/tmp/m20-known-hosts -o StrictHostKeyChecking=accept-new"
    local dest_store
    dest_store="ssh://${NOS_USER}@${NOS_HOST}?remote-program=${NOS_NIX_STORE_BIN}"

    echo "=== Copying Nix closures to NOS ==="
    docker_exec env NIX_SSHOPTS="$ssh_opts" DEST_STORE="$dest_store" bash -lc "
        set -e
        nix --extra-experimental-features 'nix-command flakes' \
            copy --to \"\$DEST_STORE\"${path_args}
    "

    echo "=== Relinking and pinning NOS outputs ==="
    while read -r mod path; do
        [ -n "$mod" ] || continue
        relink_module_on_nos "$mod" "$path"
    done < <(module_result_paths)
}

relink_module_on_nos() {
    local mod="$1"
    local resolved="$2"
    local q_resolved q_deploy
    printf -v q_resolved '%q' "$resolved"
    printf -v q_deploy '%q' "/var/opt/robot/data/dimos"

    case "$mod" in
        pgo)
            remote_ssh "
                set -e
                resolved=$q_resolved
                deploy=$q_deploy
                mkdir -p \"\$deploy/dimos/navigation/nav_stack/modules/pgo/cpp/result/bin\" /nix/var/nix/gcroots/custom
                ln -sfn \"\$resolved/bin/pgo\" \"\$deploy/dimos/navigation/nav_stack/modules/pgo/cpp/result/bin/pgo\"
                ln -sfn \"\$resolved\" /nix/var/nix/gcroots/custom/pgo-current
            "
            ;;
        far_planner)
            remote_ssh "
                set -e
                resolved=$q_resolved
                deploy=$q_deploy
                mkdir -p \
                    \"\$deploy/dimos/navigation/smart_nav/modules/far_planner/result/bin\" \
                    \"\$deploy/dimos/navigation/nav_stack/modules/far_planner/result/bin\" \
                    /nix/var/nix/gcroots/custom
                ln -sfn \"\$resolved/bin/far_planner\" \"\$deploy/dimos/navigation/smart_nav/modules/far_planner/result/bin/far_planner\"
                if [ -f \"\$resolved/bin/far_planner_native\" ]; then
                    ln -sfn \"\$resolved/bin/far_planner_native\" \"\$deploy/dimos/navigation/nav_stack/modules/far_planner/result/bin/far_planner_native\"
                else
                    ln -sfn \"\$resolved/bin/far_planner\" \"\$deploy/dimos/navigation/nav_stack/modules/far_planner/result/bin/far_planner_native\"
                fi
                ln -sfn \"\$resolved\" /nix/var/nix/gcroots/custom/far_planner-current
            "
            ;;
        arise_slam)
            remote_ssh "
                set -e
                resolved=$q_resolved
                deploy=$q_deploy
                mkdir -p \"\$deploy/dimos/navigation/smart_nav/modules/arise_slam/result/bin\" /nix/var/nix/gcroots/custom
                ln -sfn \"\$resolved/bin/arise_slam\" \"\$deploy/dimos/navigation/smart_nav/modules/arise_slam/result/bin/arise_slam\"
                ln -sfn \"\$resolved\" /nix/var/nix/gcroots/custom/arise_slam-current
            "
            ;;
        *)
            remote_ssh "
                set -e
                mod=$mod
                resolved=$q_resolved
                deploy=$q_deploy
                mkdir -p \"\$deploy/dimos/navigation/smart_nav/modules/\$mod/result/bin\" /nix/var/nix/gcroots/custom
                ln -sfn \"\$resolved/bin/\$mod\" \"\$deploy/dimos/navigation/smart_nav/modules/\$mod/result/bin/\$mod\"
                if [ -d \"\$deploy/dimos/navigation/nav_stack/modules/\$mod\" ]; then
                    mkdir -p \"\$deploy/dimos/navigation/nav_stack/modules/\$mod/result/bin\"
                    ln -sfn \"\$resolved/bin/\$mod\" \"\$deploy/dimos/navigation/nav_stack/modules/\$mod/result/bin/\$mod\"
                fi
                ln -sfn \"\$resolved\" \"/nix/var/nix/gcroots/custom/\$mod-current\"
            "
            ;;
    esac
    echo "[build_on_mac_arm64] $mod → $resolved copied, relinked, pinned"
}

ensure_builder
docker_exec uname -a
docker_exec nix --version

if [ "$ENSURE_ONLY" -eq 1 ]; then
    echo "[build_on_mac_arm64] Builder ready: ${CONTAINER}"
    exit 0
fi

module_args="$(shell_quote_join "${MODULES[@]}")"
docker_exec bash -lc "
    set -e
    cd ${CONTAINER_DIMOS_ROOT}
    DIMOS_ROOT=${CONTAINER_DIMOS_ROOT} ./dimos/robot/deeprobotics/m20/nix_wrappers/build_all.sh${module_args}
"

if [ "$COPY_TO_NOS" -eq 1 ]; then
    copy_to_nos
else
    echo
    echo "=== Built in ${CONTAINER}. Result paths: ==="
    module_result_paths
    echo
    echo "To install on NOS:"
    echo "  $0 --copy-to-nos --host m20-770-gogo ${MODULES[*]}"
fi
