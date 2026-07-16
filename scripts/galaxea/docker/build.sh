#!/bin/bash
# Build the dimos-r1lite runtime image from a CLEAN git-archive context.
#
#   ./scripts/galaxea/docker/build.sh [revision]     # default revision: 1
#
# Why the staging dir: the repo's .dockerignore re-includes data/.lfs (25GB),
# so building from the repo root ships a giant context. `git archive HEAD`
# stages exactly the committed tree (LFS files as small pointers — fine:
# r1lite blueprints don't load LFS assets).
# NOTE: builds the last COMMIT — uncommitted changes are not included.
set -e

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

DIMOS_VERSION="$(grep -m1 '^version' pyproject.toml | sed 's/.*"\(.*\)".*/\1/')"
REV="${1:-1}"
TAG="dimos-r1lite:${DIMOS_VERSION}-r1lite.${REV}"

CTX="$(mktemp -d)"
trap 'rm -rf "$CTX"' EXIT
echo "[build] staging clean context from HEAD ($(git rev-parse --short HEAD)) -> $CTX"
git archive HEAD | tar -x -C "$CTX"
# git archive materialises LFS content, so data/ lands as ~33GB of real
# assets. The r1lite blueprints load none of it; drop it or every build
# spends ~100s just transferring context to the daemon.
rm -rf "$CTX/data"

# --network=host: build steps use the host's own DNS/network — guest/corp
# networks (e.g. on-site at vendors) often block docker's default 8.8.8.8.
docker build --network=host \
    -f "$CTX/scripts/galaxea/docker/Dockerfile" \
    -t "$TAG" \
    --label org.opencontainers.image.source="https://github.com/dimensionalOS/dimos" \
    --label org.opencontainers.image.revision="$(git rev-parse --short HEAD)" \
    "$CTX"

echo
echo "[build] built $TAG"
docker image inspect "$TAG" --format '[build] size: {{.Size}} bytes'
echo "[build] export for robots without registry access:"
echo "    docker save $TAG | gzip > dimos-r1lite_${DIMOS_VERSION}-r1lite.${REV}.tar.gz"
