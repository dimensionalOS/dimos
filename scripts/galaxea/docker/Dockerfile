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
    && pip install --no-cache-dir "setuptools>=70" "packaging>=24" wheel "pybind11>=2.12" tomli
COPY . /src
RUN pip wheel --no-deps --no-build-isolation --wheel-dir /wheels /src \
    && ls -l /wheels \
    && test -n "$(find /wheels -name 'dimos-*.whl' -print -quit)"

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

# pygame: r1lite-keyboard-teleop (only packaged in the heavy `sim` extra,
# so installed explicitly). Everything else r1lite needs is a core dep.
COPY --from=builder /wheels /tmp/wheels
RUN pip install --no-cache-dir /tmp/wheels/dimos-*.whl pygame \
    && rm -rf /tmp/wheels

COPY scripts/galaxea/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=2

ENTRYPOINT ["/entrypoint.sh"]
CMD ["run", "r1lite-coordinator"]
