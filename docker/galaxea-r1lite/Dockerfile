# Development image for dimos on the Galaxea R1 Lite.
# Build with docker/galaxea-r1lite/build.sh (refuses a dirty tree, pins
# the platform, stamps OCI labels). Not a production deployment artifact.

# Bump the base deliberately:
#   docker image inspect ros:humble-ros-base-jammy --format '{{index .RepoDigests 0}}'
ARG ROS_BASE=ros@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16
ARG UV_VERSION=0.11.17

FROM ${ROS_BASE} AS lockexport
ARG UV_VERSION
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir "uv==${UV_VERSION}"
COPY pyproject.toml uv.lock /src/
# Two lockfile exports, both hash-complete: the base project closure and
# the r1lite-runtime dependency group (teleop solver stack plus the
# web/vis packages the blueprints import at module scope). The group is
# maintained in pyproject.toml, so its transitive closure and hashes
# come from uv.lock rather than hand-enumerated pins.
RUN cd /src && uv export --frozen --no-emit-project --no-default-groups \
        --format requirements-txt -o /requirements.txt \
    && test -s /requirements.txt \
    && grep -q '^typer==' /requirements.txt \
    && ! grep -qE '^(torch|mujoco|pyaudio)==' /requirements.txt \
    && uv export --frozen --no-emit-project --no-default-groups \
        --group r1lite-runtime \
        --format requirements-txt -o /r1lite-runtime.txt \
    && grep -q '^pin-pink==' /r1lite-runtime.txt \
    && grep -q '^fastapi==' /r1lite-runtime.txt \
    && ! grep -qE '^(torch|mujoco|pyaudio)==' /r1lite-runtime.txt

FROM ${ROS_BASE} AS builder
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential python3-dev python3-pip \
    && rm -rf /var/lib/apt/lists/*
ENV CIBUILDWHEEL=1
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir "setuptools>=70" "packaging>=24" wheel "pybind11>=2.12" tomli
COPY . /src
RUN pip wheel --no-deps --no-build-isolation --wheel-dir /wheels /src \
    && test -n "$(find /wheels -name 'dimos-*.whl' -print -quit)"

FROM ${ROS_BASE}
ARG UV_VERSION
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python-is-python3 \
        libturbojpeg0-dev \
        liblcm-dev \
        libgl1 \
        libglib2.0-0 \
        libsndfile1 \
        iproute2 \
        tcpdump \
        git \
        git-lfs \
    && rm -rf /var/lib/apt/lists/*

# Dependency layer, keyed on the lockfile via bind mount. --no-deps: each
# export is a complete closure slice and re-resolving breaks on
# override-dependencies.
RUN --mount=type=bind,from=lockexport,source=/requirements.txt,target=/tmp/requirements.txt \
    --mount=type=bind,from=lockexport,source=/r1lite-runtime.txt,target=/tmp/r1lite-runtime.txt \
    pip install --no-cache-dir "uv==${UV_VERSION}" \
    && uv pip install --system --no-cache --no-deps -r /tmp/requirements.txt \
    && uv pip install --system --no-cache --no-deps -r /tmp/r1lite-runtime.txt \
    && pip uninstall -y uv

RUN --mount=type=bind,from=builder,source=/wheels,target=/tmp/wheels \
    pip install --no-cache-dir --no-deps /tmp/wheels/dimos-*.whl

COPY docker/galaxea-r1lite/entrypoint.sh /entrypoint.sh
COPY docker/galaxea-r1lite/dds_selftest.sh /usr/local/bin/dds-selftest
RUN chmod +x /entrypoint.sh /usr/local/bin/dds-selftest

# Session tooling for interactive shells: preflight/arming and the test
# helpers live under a fixed path so `r1 shell` users need no repo
# checkout. The blueprint's log/recording paths point at /app/logs, which
# the launcher bind-mounts host-side.
COPY scripts/r1lite_test /opt/dimos/scripts/r1lite_test
RUN mkdir -p /app/logs && chown -R 1000:1000 /app

# The teleop web server generates its self-signed TLS certs under the
# package's assets dir (a repo-checkout assumption); pre-create it
# writable for the runtime user so the headset endpoint can start from a
# wheel install.
RUN mkdir -p /usr/local/lib/python3.10/dist-packages/assets/teleop_certs \
    && chown -R 1000:1000 /usr/local/lib/python3.10/dist-packages/assets

# Every bash in this image — interactive shells, docker exec, the r1
# launcher's dev door — gets the ROS environment without the entrypoint.
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
    && echo 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"' >> /etc/bash.bashrc

# uid 1000 matches the vendor stack so shared-memory DDS delivery works.
RUN useradd --uid 1000 --user-group --create-home --shell /bin/bash dimos
USER dimos
ENV HOME=/home/dimos
ENV PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=2

# Metadata consistency gate. Exactly two failures are known and accepted:
# the open3d wheel declares ipywidgets while the locked closure does not
# include it (a lock-vs-wheel metadata mismatch; the r1lite path never
# imports it), and colcon-core comes from the base image's apt with a
# setuptools<80 bound while the locked closure pins a newer setuptools.
# Fail closed on everything else: any other line, any stderr output, and
# any exit status besides the documented 0 (clean) or 1 (conflicts
# found, which must then all be in the accepted list).
RUN python3 -m pip check > /tmp/pipcheck.txt 2>/tmp/pipcheck_err.txt; \
    status=$?; \
    if [ "$status" -ne 0 ] && [ "$status" -ne 1 ]; then \
        echo "pip check itself failed with status $status:"; \
        cat /tmp/pipcheck.txt /tmp/pipcheck_err.txt; exit 1; \
    fi; \
    if [ -s /tmp/pipcheck_err.txt ]; then \
        echo "pip check wrote to stderr:"; cat /tmp/pipcheck_err.txt; exit 1; \
    fi; \
    if [ "$status" -eq 1 ] && [ ! -s /tmp/pipcheck.txt ]; then \
        echo "pip check reported conflicts but produced no output"; exit 1; \
    fi; \
    grep -vE '^(open3d [0-9.]+ requires ipywidgets, which is not installed\.|colcon-core [0-9.]+ has requirement setuptools(<80,)?>=30\.3\.0, but you have setuptools [0-9.]+\.)$' /tmp/pipcheck.txt > /tmp/pipcheck_unexpected.txt || true; \
    if [ -s /tmp/pipcheck_unexpected.txt ]; then \
        echo "pip check found undocumented inconsistencies:"; \
        cat /tmp/pipcheck_unexpected.txt; exit 1; \
    fi

# Build-time gate, run as the runtime user. Constructing a teleop task
# builds the real Pink control pipeline, which imports pink and verifies
# the QP backend; a missing or downgraded solver fails the build here
# instead of at robot startup. The hosted stack's WebRTC deps are
# imported explicitly because the providers load them lazily on start(),
# where a blueprint-import check cannot see them.
RUN dimos list | grep -q r1lite-coordinator \
    && dimos list | grep -q r1lite-quest-teleop \
    && dimos list | grep -q r1lite-quest-teleop-hosted \
    && python3 -c "from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import r1lite_coordinator" \
    && python3 -c "from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_keyboard_teleop import r1lite_keyboard_teleop" \
    && python3 -c "from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop_hosted import r1lite_quest_teleop_hosted" \
    && python3 -c "import pink, proxsuite" \
    && python3 -c "import aiortc, aiohttp, av" \
    && python3 -c "\
from dimos.control.tasks.teleop_task.teleop_task import TeleopControlIKConfig, TeleopIKTask, create_task; \
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import _teleop_tasks; \
tasks = [create_task(c, None) for c in _teleop_tasks()]; \
assert tasks and all(isinstance(t, TeleopIKTask) and isinstance(t._config.control_ik, TeleopControlIKConfig) for t in tasks)"

# The validated stop path is the SIGINT sequence (the connection streams
# bounded chassis zeros and reports settling); docker stop must trigger
# exactly that.
STOPSIGNAL SIGINT

ENTRYPOINT ["/entrypoint.sh"]
CMD ["run", "r1lite-coordinator"]
