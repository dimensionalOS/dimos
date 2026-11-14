# Eval worker image for `dimos evals run --docker`; see docs/usage/evals-docker.md.
# Build from the repo root after `git lfs pull --include="misc/DimSim/**"`.
FROM ubuntu:24.04

# Keep in sync with dimos/utils/deno.py (DENO_VERSION) and
# dimos/simulation/dimsim/deno_utils.py (playwright version).
ARG DENO_VERSION=v2.9.8
ARG PLAYWRIGHT_VERSION=1.58.2
ARG PYTHON_VERSION=3.12

ENV DEBIAN_FRONTEND=noninteractive

# The self-hosted CI job's apt list plus Chromium's runtime libs.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl wget git unzip build-essential pkg-config \
    libgl1 libegl1 libopengl0 libturbojpeg portaudio19-dev libusb-1.0-0 \
    libatk1.0-0 libatk-bridge2.0-0 libcairo2 libcups2 libgbm1 \
    libpango-1.0-0 libxcomposite1 libxdamage1 libxkbcommon0 libxrandr2 \
    libxkbcommon-x11-0 libnss3 libnspr4 libasound2t64 libatspi2.0-0 \
    libxfixes3 libdrm2 libx11-xcb1 libxext6 libxshmfence1 fonts-liberation \
    iproute2 xvfb x11-utils \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo's MUJOCO_GL=egl on the GPU: the NVIDIA runtime mounts libEGL_nvidia only with the
# graphics capability, and glvnd finds it only through this vendor file.
ENV NVIDIA_DRIVER_CAPABILITIES=all
RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' \
    > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Recordings and results land under /state, which compose bind-mounts.
ENV XDG_CACHE_HOME=/cache \
    XDG_STATE_HOME=/state \
    XDG_CONFIG_HOME=/config \
    DENO_DIR=/cache/deno \
    PLAYWRIGHT_BROWSERS_PATH=/cache/ms-playwright

# Deno on PATH: dimos's ensure_deno() prefers `which deno` over downloading.
RUN curl -fsSL "https://github.com/denoland/deno/releases/download/${DENO_VERSION}/deno-x86_64-unknown-linux-gnu.zip" \
        -o /tmp/deno.zip \
    && unzip -q /tmp/deno.zip -d /usr/local/bin && rm /tmp/deno.zip \
    && chmod +x /usr/local/bin/deno

# The Pi eval agent (dimos/evals/agents/pi.py) shells out to the `pi` coding-agent CLI;
# the dimos agent (dimos/evals/agents/dimos_agent.py) needs Node 24 or 26.
ARG NODE_MAJOR=24
RUN curl -fsSL "https://deb.nodesource.com/setup_${NODE_MAJOR}.x" | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && npm install -g @earendil-works/pi-coding-agent \
    && rm -rf /var/lib/apt/lists/* /root/.npm

# The same browser dimos installs before each DimSim launch, so that is a no-op then.
# Through npx: the deno install stalls after the download.
RUN npx -y "playwright@${PLAYWRIGHT_VERSION}" install chromium && rm -rf /root/.npm

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

WORKDIR /app

# Dependencies first so code edits don't invalidate the deps layer.
COPY pyproject.toml uv.lock README.md setup.py ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv python install "${PYTHON_VERSION}" \
    && uv sync --frozen --no-install-project --python "${PYTHON_VERSION}" \
        --group tests-self-hosted

COPY . /app
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --python "${PYTHON_VERSION}" --group tests-self-hosted \
    && uv run python setup.py build_ext --inplace

# DimSim's frontend and bridge dependencies, so nothing downloads at run time.
RUN cd misc/DimSim \
    && deno install --no-lock \
    && deno run -A --no-lock "npm:vite@^5.4.10" build \
    && cd /app && deno install --entrypoint misc/DimSim/cli/cli.ts

# The dimos agent's npm install and build, so its preflight does nothing at run time.
RUN /app/.venv/bin/dimos agent version

# Habitat's conda env (python 3.9) and HM3D example via the module's installer,
# with micromamba in place of its nix shell.
RUN curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest \
        | tar -xj -C /usr/local/bin --strip-components=1 bin/micromamba \
    && bash dimos/simulation/habitat/nix/install.sh \
    && MAMBA_ROOT_PREFIX=/app/target/habitat/mm micromamba clean -a -y

ENV PATH="/app/.venv/bin:${PATH}" \
    VIEWER=none \
    PYTHONUNBUFFERED=1

RUN mkdir -p /state /config

COPY docker/evals/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
