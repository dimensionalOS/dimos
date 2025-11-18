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
    && rm -rf /var/lib/apt/lists/*

# Create symlink for python
RUN ln -sf /usr/bin/python3.10 /usr/bin/python

# Change working directory to /app for proper relative pathing
WORKDIR /app

COPY requirements.txt ./

COPY ./dimos/perception/external ./dimos/perception/external

RUN pip install --no-cache-dir -r requirements.txt

COPY ./dimos ./dimos

COPY ./tests ./tests

COPY ./dimos/__init__.py ./

# Add CUDA libraries to the path
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

CMD [ "python", "-m", "tests.test_agent_ctransformers_gguf" ]
