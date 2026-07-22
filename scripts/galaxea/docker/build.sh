#!/bin/bash
# Local build wrapper for the R1 Lite runtime image (iteration and bring-up).
set -e

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

DIMOS_VERSION="$(grep -m1 '^version' pyproject.toml | sed 's/.*"\(.*\)".*/\1/')"
REV="${1:-1}"
TAG="dimos-r1lite:${DIMOS_VERSION}-r1lite.${REV}"

docker build --network=host \
    -f docker/galaxea-r1lite/Dockerfile \
    -t "$TAG" \
    --label org.opencontainers.image.source="https://github.com/dimensionalOS/dimos" \
    --label org.opencontainers.image.revision="$(git rev-parse --short HEAD)" \
    .

echo
echo "[build] built $TAG"
docker image inspect "$TAG" --format '[build] size: {{.Size}} bytes'
