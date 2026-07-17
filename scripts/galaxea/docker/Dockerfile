# Runtime image for dimos on Galaxea R1 Lite onboard PCs.
#
# Built via build.sh (which stages a clean `git archive` context — do not
# build from the repo root: .dockerignore re-includes 25GB of data/.lfs).
#
# Design (see docs/deployment/galaxea-r1lite.md):
# - FROM the PUBLIC ROS Humble base — no private-registry dependency at
#   build or pull time.
# - dimos installed as a wheel into the image's system Python 3.10, which
#   is Humble's rclpy Python: no venv, no py-version mismatch possible.
# - Dependencies come from uv.lock (exported to a pinned requirements.txt),
#   NOT from pip's own resolver. pyproject deliberately sets
#   exclude-newer = "7 days", and a bare `pip install dimos.whl` ignores both
#   the lock and that policy — it resolves latest-wins and silently ships
#   versions nobody has ever tested. That is how this image shipped
#   typer 0.27.0 against a lock pinning 0.23.1, and `dimos list` died with
#   "issubclass() arg 1 must be a class". Pinning from the lock makes the
#   image reproducible and identical to what CI and the dev venv run.
# - Runs with network_mode:host + ipc:host (compose.yaml) so FastDDS
#   shared-memory + multicast reach the vendor stack on the same box.

ARG ROS_BASE=ros:humble-ros-base-jammy

# ---- builder: compiles the dimos wheel (throwaway layer) -------------------
FROM ${ROS_BASE} AS builder
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential python3-dev python3-pip \
    && rm -rf /var/lib/apt/lists/*
# CIBUILDWHEEL=1 makes setup.py drop -march=native so the C++ extension is
# portable across customer x86-64 CPUs (same switch the PyPI release uses).
ENV CIBUILDWHEEL=1
# Build deps installed explicitly + --no-build-isolation: jammy ships
# setuptools 59, which silently ignores pyproject's [project] table and
# produces an UNKNOWN-0.0.0 wheel with no packages (find_packages() finds
# nothing — the repo has no __init__.py files, so the name/packages MUST
# come from pyproject). Pinning setuptools>=70 here makes that impossible.
# packaging>=24 is not optional: setuptools>=70 calls canonicalize_version()
# with a kwarg jammy's packaging 21.3 doesn't have (TypeError at metadata
# generation). pip is upgraded too — jammy's 22.0.2 predates PEP 660 fixes.
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir "setuptools>=70" "packaging>=24" wheel "pybind11>=2.12" tomli uv
COPY . /src
RUN pip wheel --no-deps --no-build-isolation --wheel-dir /wheels /src \
    && ls -l /wheels \
    && test -n "$(find /wheels -name 'dimos-*.whl' -print -quit)"
# Freeze dependencies from uv.lock. --no-emit-project drops dimos itself (we
# install the wheel built above). No extras are requested, so this is core
# deps only -- what keeps the image ~6GB rather than ~15GB.
#
# --no-default-groups, NOT --no-dev: pyproject sets [tool.uv]
# default-groups = ["tests"], so --no-dev alone (which only drops the `dev`
# group) still exports the whole test suite -- torch, mujoco, ultralytics,
# pyaudio. That both bloats the image and breaks the build outright: pyaudio
# ships no linux wheel and needs portaudio.h plus a compiler, neither of which
# belongs in a runtime image. 149 packages, not 377.
#
# Markers survive the export, so the cp310-correct build is chosen at install
# time (numpy 2.2.6, not 2.3.5).
RUN cd /src && uv export --frozen --no-emit-project --no-default-groups \
        --format requirements-txt -o /wheels/requirements.txt \
    && test -s /wheels/requirements.txt \
    && grep -q '^typer==' /wheels/requirements.txt \
    && ! grep -qE '^(torch|mujoco|pyaudio)==' /wheels/requirements.txt

# ---- runtime ---------------------------------------------------------------
FROM ${ROS_BASE}
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python-is-python3 \
        libturbojpeg0-dev \
        liblcm-dev \
        libgl1 \
        libglib2.0-0 \
        iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Install the LOCKED dependency set first, then dimos itself with --no-deps so
# nothing can re-resolve and drag in a newer version. pygame is pinned to the
# lock's version by hand: r1lite-keyboard-teleop needs it, but it is only
# packaged in the heavy `sim` extra, which we do not want in this image.
#
# Installed with `uv --no-deps`, and both halves of that matter:
#
#   uv, not pip: uv export emits hashes, which flip pip into --require-hashes
#   mode, where every requirement must be pinned with `==`. Transitive extras
#   are not -- chromadb asks for `uvicorn[standard]>=0.18.3` -- so pip aborts.
#
#   --no-deps: the export is already the complete closure (377 packages), so
#   there is nothing to resolve. Resolving would actively break: pyproject uses
#   [tool.uv] override-dependencies to force versions past what some packages
#   declare (moondream pins pillow<11; the lock ships pillow 12). Those
#   overrides live in pyproject, not in the exported file, so any resolver
#   handed this list rediscovers the "conflict" the lock already settled.
#
# uv is removed afterwards: it is a build tool with no business in a runtime
# image. Verified as exit=0 in ros:humble-ros-base-jammy before committing.
COPY --from=builder /wheels /tmp/wheels
RUN pip install --no-cache-dir uv \
    && uv pip install --system --no-cache --no-deps -r /tmp/wheels/requirements.txt \
    && uv pip install --system --no-cache --no-deps /tmp/wheels/dimos-*.whl "pygame==2.6.1" \
    && pip uninstall -y uv \
    && rm -rf /tmp/wheels
# Fail the build here rather than on a robot: exercise the CLI the way the
# entrypoint will. `dimos list` imports the whole blueprint registry, which is
# what caught the unpinned-typer breakage.
RUN dimos list | grep -q r1lite-coordinator

COPY scripts/galaxea/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=2

ENTRYPOINT ["/entrypoint.sh"]
CMD ["run", "r1lite-coordinator"]
