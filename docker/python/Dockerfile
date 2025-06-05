ARG FROM_IMAGE
FROM ${FROM_IMAGE}

RUN mkdir -p /app/dimos

COPY requirements.txt /app/
COPY base-requirements.txt /app/

WORKDIR /app

RUN --mount=type=cache,target=/root/.cache/pip pip install -r base-requirements.txt

RUN --mount=type=cache,target=/root/.cache/pip pip install -r requirements.txt
