# Eval worker image: one container runs one eval case at a time, booting its own
# dimos + DimSim inside the container's network namespace so N containers never
# share a port or a multicast bus. Driven from the host by docker/evals/dispatch.py,
# which execs docker/evals/run_case.py into a pool of these.
#
# Build from the repo root (the DimSim LFS assets must be real, not pointer stubs):
#   git lfs pull --include="misc/DimSim/**"
#   docker build -f docker/evals/Dockerfile -t dimensional/evals .
FROM ubuntu:24.04

# Keep in sync with dimos/utils/deno.py (DENO_VERSION) and
# dimos/simulation/dimsim/deno_utils.py (playwright version).
ARG DENO_VERSION=v2.6.10
ARG PLAYWRIGHT_VERSION=1.58.2
ARG PYTHON_VERSION=3.12

ENV DEBIAN_FRONTEND=noninteractive

# Mirrors the apt list of the self-hosted CI job that runs DimSim tests, plus the
# rest of Chromium's runtime libs, iproute2 for dimos's LCM network setup, and
# wget for habitat-sim's dataset downloader.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl wget git unzip build-essential pkg-config \
    libgl1 libturbojpeg portaudio19-dev \
    libatk1.0-0 libatk-bridge2.0-0 libcairo2 libcups2 libgbm1 \
    libpango-1.0-0 libxcomposite1 libxdamage1 libxkbcommon0 libxrandr2 \
    libxkbcommon-x11-0 libnss3 libnspr4 libasound2t64 libatspi2.0-0 \
    libxfixes3 libdrm2 libx11-xcb1 libxext6 libxshmfence1 fonts-liberation \
    iproute2 xvfb ffmpeg x11-utils libegl1 libgles2 mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*

# Everything dimos caches or records lives under /cache and /state so a bind
# mount on /state captures recordings, the run registry and eval results.
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

# Headless Chromium, the exact command dimos runs before every DimSim launch, so
# at runtime it finds the browser installed and does nothing.
RUN deno run --allow-all "npm:playwright@${PLAYWRIGHT_VERSION}" install chromium

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

# Rust for the native modules dimos builds on first use (the MLS planner is a
# workspace crate; `cargo build --release` at the repo root puts it in target/release).
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"

# The Pi eval agent (dimos/evals/agents/pi.py) shells out to the `pi` coding-agent CLI.
ARG NODE_MAJOR=22
RUN curl -fsSL "https://deb.nodesource.com/setup_${NODE_MAJOR}.x" | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && npm install -g @earendil-works/pi-coding-agent \
    && rm -rf /var/lib/apt/lists/* /root/.npm

WORKDIR /app

# Dependencies first, project last, so code edits don't invalidate the deps layer.
# The dependency set is the one the self-hosted CI job uses to run DimSim evals.
COPY pyproject.toml uv.lock README.md setup.py ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv python install "${PYTHON_VERSION}" \
    && uv sync --frozen --no-install-project --python "${PYTHON_VERSION}" \
        --group tests-self-hosted

COPY . /app
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --python "${PYTHON_VERSION}" --group tests-self-hosted \
    && uv run python setup.py build_ext --inplace

RUN --mount=type=cache,target=/root/.cargo/registry \
    cargo build --release -p dimos-mls-planner && ls -la target/release/mls_planner

# DimSim's frontend, built the way cli.ts would on first run (dist/ is gitignored),
# and the bridge CLI's own JSR/npm dependencies, cached with the same entrypoint and
# working directory dimos launches it from so nothing downloads at runtime.
RUN cd misc/DimSim \
    && deno install --no-lock \
    && deno run -A --no-lock "npm:vite@^5.4.10" build \
    && cd /app && deno install --entrypoint misc/DimSim/cli/cli.ts

# Habitat. habitat-sim is conda-only (python 3.9), so it lives in its own env under
# target/habitat, built by the module's own installer with micromamba standing in
# for the nix shell that normally provides it. The installer also fetches the
# annotated HM3D example house and writes the `habitat-native` wrapper NativeModule
# treats as the build sentinel, so a run never rebuilds. Headless rendering needs
# EGL from the host's NVIDIA driver at run time: use compose.gpu.yaml.
RUN curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest \
        | tar -xj -C /usr/local/bin --strip-components=1 bin/micromamba \
    && bash dimos/simulation/habitat/nix/install.sh \
    && MAMBA_ROOT_PREFIX=/app/target/habitat/mm micromamba clean -a -y

# `dimos` on PATH for the eval runner's `dimos run` subprocess; no rerun viewer.
ENV PATH="/app/.venv/bin:${PATH}" \
    VIEWER=none \
    PYTHONUNBUFFERED=1

# Headless EGL on the NVIDIA runtime: glvnd needs the vendor ICD to find libEGL_nvidia.
# The viewer (wgpu) needs a Vulkan ICD: NVIDIA's through the runtime, lavapipe otherwise.
RUN mkdir -p /etc/vulkan/icd.d && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libGLX_nvidia.so.0","api_version":"1.3.0"}}\n' > /etc/vulkan/icd.d/nvidia_icd.json
RUN mkdir -p /usr/share/glvnd/egl_vendor.d && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}\n' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json
RUN mkdir -p /state /config

# `dimos evals run --docker` runs one eval per container through this entrypoint;
# the pool mode (dispatch.py) starts it idle and execs cases in.
COPY docker/evals/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
