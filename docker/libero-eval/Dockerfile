FROM docker.io/library/node@sha256:cff78eb5aa1cf27dc2b6aeea9d31366415a43e9a9ea0ddec00d780b2b66fad0f AS node

FROM docker.io/library/python@sha256:72d3d75f2639ab82b34b29390ad3d6e0827c775befee94edda8e9976818f488d

ENV DEBIAN_FRONTEND=noninteractive \
    PATH=/opt/dimos/.venv/bin:/usr/local/bin:$PATH \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git libegl1 libgl1 libglib2.0-0 libglfw3 libosmesa6 podman \
    && rm -rf /var/lib/apt/lists/*

COPY --from=node /usr/local/ /usr/local/
RUN python -m pip install --no-cache-dir uv==0.11.24

WORKDIR /opt/dimos
COPY . /opt/dimos
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --no-default-groups \
        --extra agents \
        --extra misc \
        --extra perception \
        --extra manipulation \
        --extra cuda \
        --extra graspgenx \
    && npm ci --prefix packages/pi-code-policy-extension \
    && npm run build --prefix packages/pi-code-policy-extension

ENTRYPOINT []
