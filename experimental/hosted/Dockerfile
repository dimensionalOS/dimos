FROM python:3.12-slim-bookworm

ARG UV_VERSION=0.12.3

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        libgl1 \
        libglib2.0-0 \
        libgomp1 \
        libturbojpeg0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir "uv==${UV_VERSION}"

WORKDIR /app
COPY pyproject.toml uv.lock README.md ./
RUN uv sync --frozen --no-default-groups --no-install-project

COPY . ./
RUN uv sync --frozen --no-default-groups

ENV PATH="/app/.venv/bin:$PATH"
