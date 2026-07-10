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

if [ "${1:-}" = "--cross" ]; then
    # Needs binfmt/qemu once: docker run --privileged --rm tonistiigi/binfmt --install arm64
    docker buildx build --platform linux/arm64 -f "${DOCKERFILE}" -t "${TAG}" --load .
    if [ -n "${ROBOT:-}" ]; then
        echo "Shipping ${TAG} to ${ROBOT}..."
        docker save "${TAG}" | ssh "${ROBOT}" docker load
    fi
else
    docker build -f "${DOCKERFILE}" -t "${TAG}" .
fi

echo "Built ${TAG}"
