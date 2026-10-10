# Build both targets before starting an evaluation; runtime has no downloads.
FROM ubuntu:24.04 AS base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates libgl1 libegl1 libosmesa6 libturbojpeg portaudio19-dev \
    libglib2.0-0 libgomp1 libstdc++6 libxcb1 \
    && rm -rf /var/lib/apt/lists/*

FROM base AS build
RUN apt-get update && apt-get install -y --no-install-recommends git \
    && rm -rf /var/lib/apt/lists/*
COPY --from=ghcr.io/astral-sh/uv:0.10.0 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/python UV_PROJECT_ENVIRONMENT=/opt/venv
WORKDIR /app
COPY pyproject.toml uv.lock README.md setup.py ./
RUN uv python install 3.12 && uv sync --frozen --no-install-project --group tests-self-hosted
RUN git clone https://github.com/xArm-Developer/xarm_ros2 /models/xarm_ros2 \
    && git -C /models/xarm_ros2 checkout 5bb832f72ca665f1236a9d8ed1c3a82f308db489
COPY dimos /source/dimos
# No scene data or Git history enters either final image. Private assets are mounted
# only into the trusted container, by the host evaluator.
RUN cp -a /source /robot && rm -rf /robot/dimos/evals /robot/dimos/simulation \
    /robot/dimos/experimental/scene_cooking \
    && find /robot -type f \( -name 'test_*.py' -o -name 'conftest.py' \) -delete \
    && find /robot -type d -name '__pycache__' -prune -exec rm -rf {} + \
    && cp -a /opt/venv /robot-venv \
    && rm -rf /robot-venv/lib/python*/site-packages/mujoco*

FROM node:22-bookworm-slim AS pi
RUN npm install -g @earendil-works/pi-coding-agent

FROM base AS runtime
COPY --from=build /opt/python /opt/python
ENV PATH=/opt/venv/bin:$PATH PYTHONPATH=/app PYTHONUNBUFFERED=1 \
    DIMOS_TRANSPORT=zenoh DIMOS_ZENOH_MULTICAST=false DIMOS_ZENOH_GOSSIP=false \
    DIMOS_ZENOH_SCOUTING=false VIEWER=none XDG_CACHE_HOME=/tmp/cache \
    XDG_STATE_HOME=/tmp/state XDG_CONFIG_HOME=/tmp/config HOME=/tmp/home
WORKDIR /app
USER 1000:1000

FROM runtime AS trusted
COPY --from=build /opt/venv /opt/venv
COPY --from=build /source/dimos /app/dimos
ENV MUJOCO_GL=osmesa
ENTRYPOINT ["python", "-m", "dimos.evals.isolated_mujoco.runtime"]
CMD ["/private/episode.json"]

FROM runtime AS robot
COPY --from=build /robot-venv /opt/venv
COPY --from=build /robot/dimos /app/dimos
COPY --from=build /models/xarm_ros2/xarm_description /opt/models/xarm_ros2/xarm_description
COPY --from=pi /usr/local/bin/node /usr/local/bin/node
COPY --from=pi /usr/local/lib/node_modules /usr/local/lib/node_modules
COPY --from=pi /usr/local/bin/pi /usr/local/bin/pi
COPY --chmod=755 docker/mujoco-eval/dimos.py /usr/local/bin/dimos
COPY docker/mujoco-eval/probe.py /app/probe.py
# Agent controls this container. It receives public IDs and an endpoint only.
CMD ["python", "/app/probe.py"]
