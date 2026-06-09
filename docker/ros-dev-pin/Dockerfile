# Digest pin for ghcr.io/dimensionalos/ros-dev:latest consumed by every CI
# and release job that uses the ros-dev container. This Dockerfile is NEVER
# BUILT. It exists solely as a target for Dependabot's `docker` ecosystem,
# which scans FROM directives and opens PRs to bump the @sha256:... digest
# when the `:latest` tag moves (rebuilt from main by docker-build.yml).
#
# The `compute-pin` job in .github/workflows/ci.yml and .github/workflows/
# release.yml greps the digest out of this file and feeds it to consuming jobs'
# `container:` fields.
#
# IMPORTANT: do NOT remove the `:latest` tag from the FROM line — Dependabot
# uses it to anchor which channel to track. With digest-only, it has nothing
# to follow and stops updating.
FROM ghcr.io/dimensionalos/ros-dev:latest@sha256:0496bc4075010606afff818c2c40676a86d4e3b8d7c8455dc4fd0f7fa68af700
