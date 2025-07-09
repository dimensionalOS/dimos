ARG FROM_IMAGE=ghcr.io/dimensionalos/ros-python:dev
FROM ${FROM_IMAGE}

ARG GIT_COMMIT=unknown
ARG GIT_BRANCH=unknown

RUN apt-get update && apt-get install -y \
    git \
    git-lfs \
    nano \
    vim \
    ccze \
    tmux \
    htop \
    python-is-python3 \
    iputils-ping \
    wget \
    net-tools \
    sudo \
    pre-commit


# Configure git to trust any directory (resolves dubious ownership issues in containers)
RUN git config --global --add safe.directory '*'

COPY . /app/
WORKDIR /app
RUN --mount=type=cache,target=/root/.cache/pip pip install .[dev]

# Copy files and add version to motd
COPY /assets/dimensionalascii.txt /etc/motd
COPY /docker/dev/bash.sh /root/.bash.sh
COPY /docker/dev/tmux.conf /root/.tmux.conf

# Install nodejs (for random devtooling like copilot etc)
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
ENV NVM_DIR=/root/.nvm
RUN bash -c "source $NVM_DIR/nvm.sh && nvm install 24"

# This doesn't work atm
RUN echo "  v_${GIT_BRANCH}:${GIT_COMMIT} | $(date)" >> /etc/motd
RUN echo "echo -e '\033[34m$(cat /etc/motd)\033[0m\n'" >> /root/.bashrc

RUN echo "source /root/.bash.sh" >> /root/.bashrc

COPY /docker/dev/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
