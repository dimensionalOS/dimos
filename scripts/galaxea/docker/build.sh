#!/bin/bash
# Local build wrapper for the R1 Lite runtime image (Final plan Step 1).
#
#   ./scripts/galaxea/docker/build.sh [revision]     # default revision: 1
#
# Thin by design: it invokes the SAME build definition CI uses
# (docker/galaxea-r1lite/Dockerfile, context = repo root, filtered by
# docker/galaxea-r1lite/Dockerfile.dockerignore). Local builds are for
# iteration and bring-up; CI builds define official release artifacts —
# this script must never grow release semantics.
#
# Builds the WORKING TREE (the dockerignore keeps the context ~200MB; the old
# git-archive staging that this replaces existed only to dodge a 35GB context).
#
# --network=host: build steps use the host's own DNS — guest/corp networks
# (e.g. on-site at vendors) often block docker's default 8.8.8.8.
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
echo "[build] export for robots without registry access:"
echo "    docker save $TAG | gzip > dimos-r1lite_${DIMOS_VERSION}-r1lite.${REV}.tar.gz"
