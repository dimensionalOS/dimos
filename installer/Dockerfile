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

# Create a virtualenv in a stable location
ENV VENV_PATH=/opt/venv
RUN python3 -m venv "${VENV_PATH}"

# Make the venv the default python/pip for subsequent RUN/CMD
ENV PATH="${VENV_PATH}/bin:/root/.local/bin:${PATH}"

# Auto-activate the venv for interactive shells
RUN printf '%s\n' \
  'if [ -n "$PS1" ] && [ -f "'"${VENV_PATH}"'/bin/activate" ]; then' \
  '  . "'"${VENV_PATH}"'/bin/activate"' \
  'fi' \
  > /etc/profile.d/activate-venv.sh

RUN mkdir -p /app
WORKDIR /app

# Run your installer (now "python"/"pip" point at the venv by default because of PATH)
# RUN bash -lc 'sh <(curl -fsSL "https://raw.githubusercontent.com/jeff-hykin/mystery_test_1/refs/heads/master/install") --non-interactive --no-env-setup'
RUN bash -lc 'sh <(curl -fsSL "https://raw.githubusercontent.com/jeff-hykin/mystery_test_1/9588b0a7abe965311a4bc555cd52e0c4e90987f2/install") --non-interactive --no-env-setup'

# Install UV for fast Python package management (lands in /root/.local/bin, already on PATH above)
ENV UV_SYSTEM_PYTHON=1
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# Start an interactive login shell by default (loads /etc/profile.d/activate-venv.sh)
CMD ["bash", "-l"]


# WORKDIR /app

# # Copy entire project first to ensure proper package installation
# COPY . /app/


# # Install dependencies with UV (10-100x faster than pip)
# RUN uv pip install --upgrade 'pip>=24' 'setuptools>=70' 'wheel' 'packaging>=24' && \
#     uv pip install '.[cpu,sim,drone]'
