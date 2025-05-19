ARG FROM_TAG=latest
FROM ghcr.io/dimensionalos/base-ros:${FROM_TAG}

RUN mkdir -p /app/dimos

COPY requirements.txt /app/
COPY base-requirements.txt /app/

WORKDIR /app

RUN pip install -r base-requirements.txt

RUN pip install -r requirements.txt
