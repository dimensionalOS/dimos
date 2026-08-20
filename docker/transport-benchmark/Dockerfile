ARG ROS_IMAGE=ros:jazzy-ros-base-noble@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4
FROM ghcr.io/astral-sh/uv:0.11.24 AS uv
FROM ${ROS_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV UV_LINK_MODE=copy

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    iperf3 \
    iproute2 \
    iputils-ping \
    libgl1 \
    libglib2.0-0 \
    liblcm-dev \
    libturbojpeg0-dev \
    portaudio19-dev \
    procps \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY --from=uv /uv /uvx /usr/local/bin/
COPY docker/transport-benchmark/entrypoint.sh /benchmark-entrypoint.sh
RUN chmod +x /benchmark-entrypoint.sh

WORKDIR /app
COPY . /app
RUN uv venv --system-site-packages \
    && uv sync --frozen --extra benchmark --no-default-groups

ENV PATH="/app/.venv/bin:${PATH}"
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

ENTRYPOINT ["/benchmark-entrypoint.sh"]
CMD ["sleep", "infinity"]
