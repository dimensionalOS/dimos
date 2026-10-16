# The sandboxed agent: Ubuntu, the Pi coding agent, and a plain Python. No dimOS.
# See docs/usage/evals-sandbox.md. Build from the repo root:
#   docker build -f docker/evals_sandbox/agent.Dockerfile -t dimensional/evals-agent .
FROM ubuntu:24.04

ARG NODE_MAJOR=22
ARG PI_VERSION=1.2.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git unzip build-essential \
        python3 python3-pip python3-venv python3-dev \
    && curl -fsSL "https://deb.nodesource.com/setup_${NODE_MAJOR}.x" | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && npm install -g "@earendil-works/pi-coding-agent@${PI_VERSION}" \
    && rm -rf /var/lib/apt/lists/* /root/.npm

WORKDIR /root/work
# The runner starts Pi with `docker exec`; the container itself only stays up.
CMD ["sleep", "infinity"]
