ARG FROM_IMAGE=ghcr.io/dimensionalos/ros:dev
FROM ${FROM_IMAGE}

# Install basic requirements
RUN apt-get update
RUN apt-get install -y \
    python-is-python3 \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    clang \
    portaudio19-dev \
    git \
    mesa-utils \
    libgl1-mesa-glx \
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

# Copy dependency files first (lockfile enables faster, deterministic installs)
COPY pyproject.toml uv.lock /app/

# Install EXTERNAL dependencies only (cached unless pyproject.toml/uv.lock change)
# --no-install-project skips building the local package (which needs C++ sources)
RUN uv sync --frozen --no-install-project --extra cpu --extra sim --extra drone

# Copy the rest of the project
COPY . /app/

# Install the local package (builds C++ extension, but deps already cached)
RUN uv sync --frozen --extra cpu --extra sim --extra drone
