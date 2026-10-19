# The sandbox's robot side: dimOS, for the robot container (any blueprint behind a zenoh
# router, recording itself) and the model trace proxy. See docs/usage/evals-sandbox.md.
# Build from the repo root:
#   docker build -f docker/evals_sandbox/robot.Dockerfile -t dimensional/evals-robot .
FROM ubuntu:24.04

ARG PYTHON_VERSION=3.12

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git build-essential pkg-config \
        libgl1 libglib2.0-0 libgfortran5 libgomp1 libturbojpeg portaudio19-dev iproute2 \
    && rm -rf /var/lib/apt/lists/*

ENV XDG_CACHE_HOME=/cache \
    XDG_STATE_HOME=/state \
    XDG_CONFIG_HOME=/config

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

WORKDIR /app

# Dependencies first so code edits don't invalidate the deps layer.
COPY pyproject.toml uv.lock README.md setup.py ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv python install "${PYTHON_VERSION}" \
    && uv sync --frozen --no-install-project --python "${PYTHON_VERSION}"

COPY . /app
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --python "${PYTHON_VERSION}"

ENV PATH="/app/.venv/bin:${PATH}" \
    VIEWER=none \
    DIMOS_TRANSPORT=zenoh \
    PYTHONUNBUFFERED=1

# dimOS reads data from its checkout's data/ when it finds a .git there, and otherwise clones
# a second copy. This image is a checkout without history whose data/ is mounted in
# (DimosRobot.data), so mark it as the checkout.
RUN mkdir -p /state /config /app/.git /app/data
CMD ["bash"]
