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
    curl

# Install UV
ENV UV_SYSTEM_PYTHON=1
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

# Change working directory to /app for proper relative pathing
WORKDIR /app

# Copy project files for dependency installation
COPY pyproject.toml uv.lock ./

# Install dependencies with UV
RUN uv pip install .

COPY ./dimos ./dimos

COPY ./tests ./tests

COPY ./dimos/__init__.py ./

CMD [ "python", "-m", "tests.test_agent_alibaba" ]
