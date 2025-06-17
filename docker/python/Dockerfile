ARG FROM_IMAGE=ghcr.io/dimensionalos/ros:dev
ARG PYTHON_VERSION=3.12
FROM ${FROM_IMAGE}

# Install basic requirements
RUN apt-get update
RUN apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
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
    libreadline8 \
    libreadline-dev

RUN curl -fsSL https://pyenv.run | bash

# Add pyenv to PATH and initialize it in .bashrc first
ENV PATH="/root/.pyenv/bin:$PATH"
RUN echo 'export PATH="/root/.pyenv/bin:$PATH"' >> /root/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> /root/.bashrc && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> /root/.bashrc

# Source .bashrc and install Python version
RUN bash -c "source /root/.bashrc && pyenv install ${PYTHON_VERSION} && pyenv global ${PYTHON_VERSION}"

RUN mkdir -p /app/dimos

COPY requirements.txt /app/
COPY base-requirements.txt /app/

WORKDIR /app

RUN --mount=type=cache,target=/root/.cache/pip pip install -r base-requirements.txt

RUN --mount=type=cache,target=/root/.cache/pip pip install -r requirements.txt
