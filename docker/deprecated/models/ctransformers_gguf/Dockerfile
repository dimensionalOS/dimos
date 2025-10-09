FROM nvidia/cuda:12.1.0-cudnn8-runtime-ubuntu22.04

# Set up Python environment
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    python3.10-venv \
    python3-dev \
    libgl1-mesa-glx \
    build-essential \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libswscale-dev \
    libpostproc-dev \
    gcc \
    make \
    portaudio19-dev \
    python3-pyaudio \
    python3-all-dev \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create symlink for python
RUN ln -sf /usr/bin/python3.10 /usr/bin/python

# Install UV
ENV UV_SYSTEM_PYTHON=1
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

# Change working directory to /app for proper relative pathing
WORKDIR /app

# Copy project files for dependency installation
COPY pyproject.toml uv.lock ./

# Install dependencies with UV (including CUDA extras)
RUN uv pip install .[cuda]

COPY ./dimos ./dimos

COPY ./tests ./tests

COPY ./dimos/__init__.py ./

# Add CUDA libraries to the path
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

CMD [ "python", "-m", "tests.test_agent_ctransformers_gguf" ]
