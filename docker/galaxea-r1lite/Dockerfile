# Runtime image for dimos on the Galaxea R1 Lite.
# Build from the repo root: docker build -f docker/galaxea-r1lite/Dockerfile .

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
# pygame is exported separately: it comes from the sim extra, and mixing an
# unhashed pin into the hashed main export fails the install.
RUN cd /src && uv export --frozen --no-emit-project --no-default-groups \
        --format requirements-txt -o /requirements.txt \
    && test -s /requirements.txt \
    && grep -q '^typer==' /requirements.txt \
    && ! grep -qE '^(torch|mujoco|pyaudio)==' /requirements.txt \
    && uv export --frozen --no-emit-project --no-default-groups --extra sim \
        --format requirements-txt \
        | grep -E '^pygame==' | cut -d' ' -f1 > /pygame.txt \
    && [ "$(grep -c '^pygame==' /pygame.txt)" = "1" ]

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
        iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Dependency layer, keyed on the lockfile via bind mount. --no-deps: the export
# is the full closure and re-resolving breaks on override-dependencies.
RUN --mount=type=bind,from=lockexport,source=/requirements.txt,target=/tmp/requirements.txt \
    --mount=type=bind,from=lockexport,source=/pygame.txt,target=/tmp/pygame.txt \
    pip install --no-cache-dir "uv==${UV_VERSION}" \
    && uv pip install --system --no-cache --no-deps -r /tmp/requirements.txt \
    && uv pip install --system --no-cache --no-deps -r /tmp/pygame.txt \
    && pip uninstall -y uv

RUN --mount=type=bind,from=builder,source=/wheels,target=/tmp/wheels \
    pip install --no-cache-dir --no-deps /tmp/wheels/dimos-*.whl

COPY docker/galaxea-r1lite/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# uid 1000 matches the vendor stack so shared-memory DDS delivery works.
RUN useradd --uid 1000 --user-group --create-home --shell /bin/bash dimos
USER dimos
ENV HOME=/home/dimos
ENV PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=2

RUN dimos list | grep -q r1lite-coordinator \
    && python3 -c "from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import r1lite_coordinator" \
    && python3 -c "from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_keyboard_teleop import r1lite_keyboard_teleop"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["run", "r1lite-coordinator"]
