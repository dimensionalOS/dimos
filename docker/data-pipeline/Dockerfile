FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

# Set the working directory
WORKDIR /app

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget \
    ffmpeg=7:* \
    libsm6=2:* \
    libxext6=2:* \
    git=1:* \
    vim=2:* \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python 3.9
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.9 python3.9-dev python3.9-distutils python3-pip && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
    update-alternatives --set python3 /usr/bin/python3.9

# Upgrade pip and setuptools
RUN python3 -m pip install --upgrade pip setuptools wheel

# Install PyTorch with CUDA support
RUN python3 -m pip install torch==2.0.1+cu118 torchvision==0.15.2+cu118 torchaudio==2.0.2 --extra-index-url https://download.pytorch.org/whl/cu118

# Install all requirements from requirements.txt
COPY requirements.txt /app/requirements.txt
RUN python3 -m pip install -r /app/requirements.txt

# Install additional dependencies from setup.py
COPY setup.py /app/setup.py
COPY . /app
RUN python3 -m pip install .

# Install llama-cpp-python with CUDA support
RUN CMAKE_ARGS="-DLLAMA_CUBLAS=on" python3 -m pip install llama-cpp-python==0.2.45 --force-reinstall --no-cache-dir --no-deps

# Download models and place them in /app/models
WORKDIR /app/models
RUN wget https://huggingface.co/han-cai/efficientvit-sam/resolve/main/l0.pt
RUN wget https://huggingface.co/cjpais/llava-v1.6-34B-gguf/resolve/main/mmproj-model-f16.gguf
RUN wget https://huggingface.co/cjpais/llava-v1.6-34B-gguf/resolve/main/llava-v1.6-34b.Q4_K_M.gguf

# Set the working directory back to /app
WORKDIR /app

# Set environment variables for CUDA
ENV CUDA_HOME=/usr/local/cuda-11.8/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.8/lib64

# Expose any necessary ports (if applicable)
# EXPOSE 8000

# Set the default command to run your pipeline
CMD ["python3", "main.py"]
