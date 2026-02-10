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
    iproute2 # for LCM networking system config \
    liblcm-dev

# Fix distutils-installed packages that block pip upgrades
RUN apt-get purge -y python3-blinker python3-sympy python3-oauthlib || true

# Install UV for fast Python package management
ENV UV_SYSTEM_PYTHON=1
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

WORKDIR /app

# Copy entire project first to ensure proper package installation
COPY . /app/

# Install dependencies with UV (10-100x faster than pip)
RUN uv pip install --upgrade 'pip>=24' 'setuptools>=70' 'wheel' 'packaging>=24' && \
    uv pip install '.[misc,cpu,sim,drone,unitree,web,perception,visualization]'
