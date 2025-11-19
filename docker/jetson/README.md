# Jetson Setup Guide

This guide explains how to set up and run local dimOS LLM Agents on NVIDIA Jetson devices.

## Prerequisites

> **Note**: This setup has been tested on:
> - Jetson Orin Nano (8GB)
> - JetPack 6.2 (L4T 36.4.3)
> - CUDA 12.6.68

### Requirements
- NVIDIA Jetson device (Orin/Xavier)
- Docker installed (with GPU support)
- Git installed
- CUDA installed

## Basic Python Setup (Virtual Environment)

### 1. Create a virtual environment:
```bash
python3 -m venv ~/jetson_env
source ~/jetson_env/bin/activate
```

### 2. Install cuSPARSELt:

For PyTorch versions 24.06+ (see [Compatibility Matrix](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html#pytorch-jetson-rel)), cuSPARSELt is required. Install it with the [instructions](https://developer.nvidia.com/cusparselt-downloads) by selecting Linux OS, aarch64-jetson architecture, and Ubuntu distribution

For Jetpack 6.2, Pytorch 2.5, and CUDA 12.6:
```bash
wget https://developer.download.nvidia.com/compute/cusparselt/0.7.0/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb
sudo dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb
sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.7.0/cusparselt-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install libcusparselt0 libcusparselt-dev
```

### 3. Install the Jetson-specific requirements:
```bash
cd /path/to/dimos
pip install -r docker/jetson/jetson_requirements.txt
```

### 4. Run testfile:
```bash
export PYTHONPATH=$PYTHONPATH:$(pwd) 
python3 tests/test_agent_huggingface_local_jetson.py
```

## Docker Setup 
for JetPack 6.2 (L4T 36.4.3), CUDA 12.6.68

### 1. Build and Run using Docker Compose

From the DIMOS project root directory:
```bash
# Build and run the container
sudo docker compose -f docker/jetson/huggingface_local/docker-compose.yml up --build
```

This will:
- Build the Docker image with all necessary dependencies
- Start the container with GPU support
- Run the HuggingFace local agent test script

## Troubleshooting

### Libopenblas or other library errors

Run the Jetson fix script:

```bash
# From the DIMOS project root
chmod +x ./docker/jetson/fix_jetson.sh
./docker/jetson/fix_jetson.sh
```

This script will:
- Install cuSPARSELt library for tensor operations 
- Fix libopenblas.so.0 dependencies
- Configure system libraries

1. If you encounter CUDA/GPU issues:
   - Ensure JetPack is properly installed
   - Check nvidia-smi output
   - Verify Docker has access to the GPU

2. For memory issues:
   - Consider using smaller / quantized models
   - Adjust batch sizes and model parameters
   - Run the jetson in non-GUI mode to maximize ram availability

## Notes

- The setup uses PyTorch built specifically for Jetson
- Models are downloaded and cached locally
- GPU acceleration is enabled by default
