# Runtime image for dimos on Galaxea R1 Lite onboard PCs (Final plan Step 1).
#
# Build from the REPO ROOT (build.sh wraps this; CI calls the same definition):
#     docker build -f docker/galaxea-r1lite/Dockerfile .
# Dockerfile.dockerignore (next to this file) keeps the context small and
# deterministic; there is no git-archive staging step.
#
# Design (see docs/deployment/galaxea-r1lite.md and the Final plan):
# - FROM the PUBLIC ROS Humble base: no private-registry dependency at build
#   or pull time.
# - dimos installed as a wheel into the image's system Python 3.10, which is
#   Humble's rclpy Python: no venv, no py-version mismatch possible.
# - Dependencies come from uv.lock, exported in a stage that copies ONLY
#   pyproject.toml and uv.lock. That makes the (large) dependency layer's
#   cache key the LOCKFILE, not the source tree: a code-only change rebuilds
#   just the dimos wheel and the small layers after it. This is the property
#   the whole OTA design leans on (small warm updates), so the layer order
#   below is load-bearing.
# - The wheel is installed with --no-deps so nothing can re-resolve: pyproject
#   sets exclude-newer and [tool.uv] override-dependencies, and any resolver
#   run against the exported closure rediscovers "conflicts" the lock already
#   settled (this image once shipped typer 0.27 against a lock pinning 0.23.1
#   that way; `dimos list` died with "issubclass() arg 1 must be a class").
# - Runs with network_mode:host + ipc:host (compose.yaml) so FastDDS
#   shared-memory + multicast reach the vendor stack on the same box.

# Base pinned by DIGEST, not tag. `ros:humble-ros-base-jammy` is mutable and
# gets rebuilt regularly; the digest is what makes "same commit -> same image"
# true down to the OS.
#
# Digest below == ros:humble-ros-base-jammy as of 2026-07-16.
# To move it forward deliberately:
#     docker pull ros:humble-ros-base-jammy
#     docker image inspect ros:humble-ros-base-jammy --format '{{index .RepoDigests 0}}'
# then paste the result here and rebuild.
ARG ROS_BASE=ros@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16

# uv pinned exactly (Final plan D2): an unconstrained "latest uv" is the same
# reproducibility hole as an unpinned base image, one tool down.
ARG UV_VERSION=0.11.17

# ---- lockexport: requirements from pyproject + uv.lock ONLY ----------------
# Copies exactly two files, so this stage's outputs change only when the lock
# changes. Everything the runtime installs is derived here.
FROM ${ROS_BASE} AS lockexport
ARG UV_VERSION
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir "uv==${UV_VERSION}"
COPY pyproject.toml uv.lock /src/
# --no-default-groups, NOT --no-dev: pyproject sets [tool.uv]
# default-groups = ["tests"], so --no-dev alone still exports the whole test
# suite (torch, mujoco, ultralytics, pyaudio — pyaudio ships no linux wheel
# and would break the build outright). Core closure: 149 packages, not 377.
#
# pygame goes in a SEPARATE file: r1lite-keyboard-teleop needs it, it is only
# packaged in the heavy `sim` extra, and the version must come from the lock
# (D2), not a hand pin. Separate file because the main export carries hashes,
# which puts the installer in require-hashes mode; one un-hashed pin in the
# same file would fail the whole install.
RUN cd /src && uv export --frozen --no-emit-project --no-default-groups \
        --format requirements-txt -o /requirements.txt \
    && test -s /requirements.txt \
    && grep -q '^typer==' /requirements.txt \
    && ! grep -qE '^(torch|mujoco|pyaudio)==' /requirements.txt \
    && uv export --frozen --no-emit-project --no-default-groups --extra sim \
        --format requirements-txt \
        | grep -E '^pygame==' | cut -d' ' -f1 > /pygame.txt \
    && [ "$(grep -c '^pygame==' /pygame.txt)" = "1" ]

# ---- builder: compiles the dimos wheel (throwaway stage) -------------------
FROM ${ROS_BASE} AS builder
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential python3-dev python3-pip \
    && rm -rf /var/lib/apt/lists/*
# CIBUILDWHEEL=1 makes setup.py drop -march=native so the C++ extension is
# portable across customer x86-64 CPUs (same switch the PyPI release uses).
ENV CIBUILDWHEEL=1
# Build deps installed explicitly + --no-build-isolation: jammy ships
# setuptools 59, which silently ignores pyproject's [project] table and
# produces an UNKNOWN-0.0.0 wheel with no packages. packaging>=24 is not
# optional: setuptools>=70 calls canonicalize_version() with a kwarg jammy's
# packaging 21.3 doesn't have.
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir "setuptools>=70" "packaging>=24" wheel "pybind11>=2.12" tomli
COPY . /src
RUN pip wheel --no-deps --no-build-isolation --wheel-dir /wheels /src \
    && test -n "$(find /wheels -name 'dimos-*.whl' -print -quit)"

# ---- runtime ---------------------------------------------------------------
FROM ${ROS_BASE}
ARG UV_VERSION
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python-is-python3 \
        libturbojpeg0-dev \
        liblcm-dev \
        libgl1 \
        libglib2.0-0 \
        iproute2 \
    && rm -rf /var/lib/apt/lists/*

# THE dependency layer (~3GB). Its only inputs are the lockexport stage files,
# i.e. the lockfile — bind-mounted, not COPYed, so no extra layer exists to
# invalidate. A dimos code change must leave this layer's digest untouched
# (proven by the two-build test in the Final plan).
#
# uv, not pip, and --no-deps, and both halves matter:
#   uv: the export carries hashes, which flips pip into --require-hashes mode,
#       where transitive extras (chromadb -> uvicorn[standard]>=0.18.3) abort.
#   --no-deps: the export IS the closure; resolving would rediscover conflicts
#       the lock already settled via [tool.uv] override-dependencies.
# uv is removed afterwards: build tool, no business in a runtime image.
RUN --mount=type=bind,from=lockexport,source=/requirements.txt,target=/tmp/requirements.txt \
    --mount=type=bind,from=lockexport,source=/pygame.txt,target=/tmp/pygame.txt \
    pip install --no-cache-dir "uv==${UV_VERSION}" \
    && uv pip install --system --no-cache --no-deps -r /tmp/requirements.txt \
    && uv pip install --system --no-cache --no-deps -r /tmp/pygame.txt \
    && pip uninstall -y uv

# The application layer: the dimos wheel and nothing else. This is what a
# code-only OTA update actually ships.
RUN --mount=type=bind,from=builder,source=/wheels,target=/tmp/wheels \
    pip install --no-cache-dir --no-deps /tmp/wheels/dimos-*.whl

COPY docker/galaxea-r1lite/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Run as uid 1000, NOT root. Two reasons, and the first is a data-loss bug:
# 1. FastDDS delivers same-host data by writing into the READER's /dev/shm
#    segment. The Galaxea stack runs as `r1lite` (uid 1000). A root container
#    creates root-owned reader segments the vendor's uid-1000 publishers
#    cannot write into: topics visible, zero messages, silently. Matching the
#    vendor's uid keeps zero-copy shared memory working.
# 2. Root in a container with --network=host and --ipc=host is root on most
#    of the robot. There is no reason for it.
# A robot whose vendor uid differs overrides DIMOS_UID/DIMOS_GID in
# /opt/dimos/.env, and must then make $HOME writable for that uid (dimos
# keeps logs and its run registry under $HOME/.local/state/dimos).
RUN useradd --uid 1000 --user-group --create-home --shell /bin/bash dimos
USER dimos
ENV HOME=/home/dimos

ENV PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=2

# Smoke-test at BUILD time, as the runtime user, so a broken image cannot be
# produced. `dimos list` proves the CLI; the blueprint imports pull the real
# module graph (connection, coordinator, vis) — the thing the robot runs.
RUN dimos list | grep -q r1lite-coordinator \
    && python3 -c "from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import r1lite_coordinator" \
    && python3 -c "from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_keyboard_teleop import r1lite_keyboard_teleop"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["run", "r1lite-coordinator"]
