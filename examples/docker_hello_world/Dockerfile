FROM python:3.12-slim

RUN apt-get update && apt-get install -y \
    iproute2 \
    libx11-6 libgl1 libglib2.0-0 \
    libidn2-0 libgfortran5 libgomp1 \
    cowsay \
    && rm -rf /var/lib/apt/lists/*


# Copy example module so it's importable inside the container
COPY examples/docker_hello_world/hello_docker.py /dimos/source/examples/docker_hello_world/hello_docker.py
RUN touch /dimos/source/examples/__init__.py /dimos/source/examples/docker_hello_world/__init__.py

WORKDIR /app
