FROM python:3.10.12

# Unitree Specific
RUN apt-get update && apt-get install -y \
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
    libopenblas0-openmp \
    curl

# Jetson Orin Nano specific setup
RUN wget https://developer.download.nvidia.com/compute/cusparselt/0.7.0/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb && \
    dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb && \
    cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.7.0/cusparselt-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get install -y libcusparselt0 libcusparselt-dev


# Install UV
ENV UV_SYSTEM_PYTHON=1
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

# Change working directory to /app for proper relative pathing
WORKDIR /app

# Copy project files for dependency installation
COPY pyproject.toml uv.lock ./
COPY ./dimos/perception/external ./dimos/perception/external

# Install dimos requirements with UV (using cuda extras)
RUN uv pip install .[cuda]

COPY ./dimos ./dimos

COPY ./tests ./tests

COPY ./dimos/__init__.py ./

# Copy libopenblas.so.0 from host if it exists (Jetson path)
RUN ldconfig
