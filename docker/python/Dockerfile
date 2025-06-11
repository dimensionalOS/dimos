ARG FROM_IMAGE=ghcr.io/dimensionalos/ros:dev
FROM ${FROM_IMAGE}

# Install basic requirements
RUN apt-get update
RUN apt-get install -y \
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
    supervisor

RUN mkdir -p /app/dimos

COPY requirements.txt /app/
COPY base-requirements.txt /app/

WORKDIR /app

RUN --mount=type=cache,target=/root/.cache/pip pip install -r base-requirements.txt

RUN --mount=type=cache,target=/root/.cache/pip pip install -r requirements.txt
