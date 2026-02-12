# GraspGen - Grasp Pose Generation Model
# https://github.com/NVlabs/GraspGen
#
# This Dockerfile packages the GraspGen model for grasp pose generation.
# Requires CUDA 12.8+ for PyTorch and CUDA extensions.

FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04

# System dependencies for GraspGen
RUN apt-get update && apt-get install -y \
    git wget curl build-essential \
    libgl1-mesa-glx libglib2.0-0 libsm6 libxext6 libxrender-dev \
    libglu1-mesa libglu1-mesa-dev libegl1-mesa-dev \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Python environment (Miniconda)
ENV CONDA_DIR=/opt/conda
RUN wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p $CONDA_DIR && rm /tmp/miniconda.sh
ENV PATH=$CONDA_DIR/bin:$PATH

RUN conda init bash && \
    conda config --set channel_priority flexible && \
    echo 'yes' | conda tos accept --override-channels --channel defaults 2>/dev/null || true && \
    conda create -n app python=3.10 -y && conda clean -afy

# Clone GraspGen repository
WORKDIR /app
RUN git clone https://github.com/NVlabs/GraspGen.git && cd GraspGen && git checkout main

# Install PyTorch with CUDA 12.8
RUN conda run -n app pip install --no-cache-dir \
    torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128

# Install GraspGen package
WORKDIR /app/GraspGen
RUN conda run -n app pip install --no-cache-dir -e .

# Build CUDA extensions (pointnet2_ops)
RUN conda run -n app bash -c "\
    export TORCH_CUDA_ARCH_LIST='8.0 8.6 8.9 9.0 12.0' && \
    export FORCE_CUDA=1 && \
    export CUDA_HOME=/usr/local/cuda && \
    cd pointnet2_ops && pip install --no-build-isolation ."

# Install torch-scatter and torch-cluster (require CUDA compilation)
RUN conda run -n app pip install --no-cache-dir --no-build-isolation torch-scatter torch-cluster

# Additional dependencies
RUN conda run -n app pip install --no-cache-dir \
    numpy trimesh pillow pyrender imageio scipy

# Model checkpoints from LFS archive
COPY data/.lfs/models_graspgen.tar.gz /tmp/
RUN tar -xzf /tmp/models_graspgen.tar.gz -C /app/GraspGen/ && \
    rm /tmp/models_graspgen.tar.gz

# Verify checkpoints exist
RUN test -f /app/GraspGen/checkpoints/graspgen_robotiq_2f_140_gen.pth || \
    (echo "ERROR: Model checkpoints not found" && exit 1)

# Environment variables for GraspGen
ENV GRASPGEN_PATH=/app/GraspGen
ENV DEFAULT_GRIPPER=robotiq_2f_140
ENV PYOPENGL_PLATFORM=egl
ENV CUDA_LAUNCH_BLOCKING=0
ENV TORCH_CUDA_ARCH_LIST="8.0 8.6 8.9 9.0 12.0"
ENV PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

# Default command
CMD ["conda", "run", "-n", "app", "python", "-c", "print('GraspGen ready')"]
