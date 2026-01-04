FROM ubuntu:22.04

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install basic requirements
RUN apt-get update && apt-get install -y \
    python-is-python3 \
    python3-venv \
    curl \
    lsb-release \
    gnupg2 \
    bash \
    && rm -rf /var/lib/apt/lists/*

ENV PATH="/root/.local/bin:$PATH"

# RUN bash -lc 'sh <(curl -fsSL "https://raw.githubusercontent.com/jeff-hykin/mystery_test_1/refs/heads/master/install") --non-interactive --no-env-setup'
ARG DIMOS_FEATURES_FOR_DOCKER=""
ARG DIMOS_REF_FOR_DOCKER="refs/heads/master"
RUN bash -lc 'sh <(curl -fsSL "https://raw.githubusercontent.com/jeff-hykin/mystery_test_1/$DIMOS_REF_FOR_DOCKER/install") --just-system-install --non-interactive --features "$DIMOS_FEATURES_FOR_DOCKER"'

RUN echo '# Dimos auto-setup'                                                                                                                                                                                                >> "$HOME/.bashrc" && \
    echo 'if [ ! -d "$PWD/.dimos" ]; then'                                                                                                                                                                                   >> "$HOME/.bashrc" && \
    echo '    sh <(curl -fsSL "https://raw.githubusercontent.com/jeff-hykin/mystery_test_1/$DIMOS_REF_FOR_DOCKER/install") --no-system-install --non-interactive --features "$DIMOS_FEATURES_FOR_DOCKER" \'                  >> "$HOME/.bashrc" && \
    echo '    && touch "$PWD/.dimos"'                                                                                                                                                                                        >> "$HOME/.bashrc" && \
    echo 'fi'                                                                                                                                                                                                                >> "$HOME/.bashrc" && \
    echo ''                                                                                                                                                                                                                  >> "$HOME/.bashrc" && \
    echo '# Activate virtualenv if present'                                                                                                                                                                                  >> "$HOME/.bashrc" && \
    echo 'if [ -f "$PWD/venv/bin/activate" ]; then'                                                                                                                                                                          >> "$HOME/.bashrc" && \
    echo '    . "$PWD/venv/bin/activate"'                                                                                                                                                                                    >> "$HOME/.bashrc" && \
    echo 'fi'                                                                                                                                                                                                                >> "$HOME/.bashrc" && \
    :

WORKDIR /app/

# Start an interactive login shell by default (loads /etc/profile.d/activate-venv.sh)
CMD ["bash", "-l"]
