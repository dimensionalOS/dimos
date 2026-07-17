ARG FROM_IMAGE=ghcr.io/dimensionalos/ros:dev
FROM ${FROM_IMAGE}

# Install basic requirements
RUN apt-get update && apt-get install -y \
    python-is-python3 \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    clang \
    portaudio19-dev \
    git \
    mesa-utils \
    libgl1 \
    libgl1-mesa-dri \
    software-properties-common \
    libxcb1-dev \
    libxcb-keysyms1-dev \
    libxcb-util0-dev \
    libxcb-icccm4-dev \
    libxcb-image0-dev \
    libxcb-randr0-dev \
    libxcb-shape0-dev \
    libxcb-xinerama0-dev \
    libxcb-xkb-dev \
    libxkbcommon-x11-dev \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    supervisor \
    iproute2 \
    liblcm-dev \
    libturbojpeg0-dev \
    cyclonedds-dev  # For unitree-sdk2py

# Fix distutils-installed packages that block pip upgrades
RUN apt-get purge -y python3-blinker python3-sympy python3-oauthlib || true

# Setup needed for Python's cyclonedds to work.
RUN mkdir -p /opt/cyclonedds && ln -s /usr/include /opt/cyclonedds/include && ln -s /usr/lib/x86_64-linux-gnu /opt/cyclonedds/lib
ENV CYCLONEDDS_HOME=/opt/cyclonedds

# Install UV for fast Python package management
ENV UV_SYSTEM_PYTHON=1
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

WORKDIR /app
