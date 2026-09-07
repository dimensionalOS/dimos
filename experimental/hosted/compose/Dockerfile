FROM python:3.12-slim-bookworm

COPY --from=ghcr.io/astral-sh/uv:0.12.3 /uv /uvx /bin/

ENV DEBIAN_FRONTEND=noninteractive \
    UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PROJECT_ENVIRONMENT=/opt/dimos-venv

RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1 \
    libgl1 \
    libglib2.0-0 \
    libglfw3 \
    libgomp1 \
    libosmesa6 \
    libturbojpeg0 \
    libx11-6 \
    libxext6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY experimental/hosted/compose/requirements.txt /tmp/hosted-requirements.txt

RUN --mount=type=cache,target=/root/.cache/uv \
    uv venv /opt/dimos-venv \
    && uv pip install \
        --no-config \
        --python /opt/dimos-venv/bin/python \
        --requirements /tmp/hosted-requirements.txt

COPY . /app

RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --no-config \
        --python /opt/dimos-venv/bin/python \
        --no-deps \
        .

ENV PATH="/opt/dimos-venv/bin:$PATH" \
    PYTHONUNBUFFERED=1

CMD ["python", "-m", "dimos.hosted.demo_compose_scenarios", "router"]
