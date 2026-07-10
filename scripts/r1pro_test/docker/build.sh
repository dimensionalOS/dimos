#!/usr/bin/env bash
# Build the on-robot dimos image. Context is always the repo root.
#
#   ./build.sh                                   # native build (run ON the robot)
#   ./build.sh --cross                           # cross-build arm64 from an x86 laptop
#   ROBOT=user@192.168.123.150 ./build.sh --cross  # cross-build + ship to robot
set -e
cd "$(dirname "$0")/../../.."
TAG="${TAG:-dimos-r1pro:latest}"
DOCKERFILE=scripts/r1pro_test/docker/Dockerfile

# --network=host: the build's apt step needs DNS. The host resolves via the
# systemd-resolved stub (127.0.0.53), which lives on the host loopback and is
# unreachable from Docker's default bridge netns — apt then fails with
# "Temporary failure resolving 'ports.ubuntu.com'". Host networking shares the
# working resolver (same as run.sh runs the container). Override the resolver
# explicitly instead if your site blocks host-net builds:
#   docker build --add-host ... / configure /etc/docker/daemon.json "dns".
if [ "${1:-}" = "--cross" ]; then
    # Needs binfmt/qemu once: docker run --privileged --rm tonistiigi/binfmt --install arm64
    docker buildx build --network=host --platform linux/arm64 -f "${DOCKERFILE}" -t "${TAG}" --load .
    if [ -n "${ROBOT:-}" ]; then
        echo "Shipping ${TAG} to ${ROBOT}..."
        docker save "${TAG}" | ssh "${ROBOT}" docker load
    fi
else
    docker build --network=host -f "${DOCKERFILE}" -t "${TAG}" .
fi

echo "Built ${TAG}"
