#!/bin/bash

build_image() {
    docker rm -f dimos-dev
    docker build \
        --build-arg GIT_COMMIT=$(git rev-parse --short HEAD) \
        --build-arg GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD) \
        -t dimensionalos/dev-base docker/dev/base/
}

if [ "$1" == "build" ]; then
    build_image
else
    # Check if image exists
    if ! docker image inspect dimensionalos/dev-base &>/dev/null; then
        echo "Image dimensionalos/dev-base not found. Building..."
        build_image
    fi
fi

docker compose -f docker/dev/base/docker-compose.yaml up -d && docker exec -it dimos-dev bash
