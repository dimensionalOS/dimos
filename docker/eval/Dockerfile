# One eval case per container: dimOS in a venv, Habitat native, DimSim's browser,
# Pi and dimcode, and a virtual display for viewer capture.
#   docker build -f docker/eval/Dockerfile -t dimos-eval .
#   dimos evals run <suite> --agent <agent> --parallel 4 --container dimos-eval
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive PATH=/app/.venv/bin:/root/.local/bin:$PATH
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates git git-lfs build-essential pkg-config \
    xvfb ffmpeg mesa-vulkan-drivers libegl1 libgl1 libglib2.0-0 libnss3 libatk-bridge2.0-0 \
    libxkbcommon0 libgbm1 libasound2 fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*
RUN curl -LsSf https://astral.sh/uv/install.sh | sh \
    && curl -fsSL https://deb.nodesource.com/setup_22.x | bash - && apt-get install -y nodejs \
    && npm install -g @earendil-works/pi-coding-agent@0.85.1 \
    && curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xvj -C /usr/local bin/micromamba
WORKDIR /app
COPY . .
RUN uv sync --no-dev
# Habitat native: the same install.sh the host uses, micromamba already on PATH.
RUN cd dimos/simulation/habitat/nix && ./install.sh
# DimSim's Deno and Chromium at build time, not first run.
RUN python -c "from dimos.utils.deno import ensure_deno; from dimos.simulation.dimsim.deno_utils import ensure_playwright_chromium; ensure_playwright_chromium(ensure_deno())"
ENV DIMOS_TRANSPORT=lcm CI=1 PYTEST_VERSION=1
ENTRYPOINT []
