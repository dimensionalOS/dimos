# Jetson Setup Guide

This guide explains how to set up and run local dimOS LLM Agents on NVIDIA Jetson devices.

## Prerequisites

> **Note**: This setup has been tested on:
> - Jetson Orin Nano (8GB)
> - JetPack 6.2 (L4T 36.4.3)
> - CUDA 12.6.68

### Requirements
- NVIDIA Jetson device (Orin/Xavier)
- Docker installed
- Git installed

## Initial Jetson Setup

Before proceeding with any other setup steps, run the Jetson setup script:

```bash
# From the DIMOS project root
./docker/jetson/setup_jetson.sh
```

This script will:
- Install cuSPARSELt library for tensor operations
- Fix libopenblas.so.0 dependencies
- Configure system libraries


## Basic Python Setup (Virtual Environment)

1. Create a virtual environment:
```bash
python3 -m venv ~/jetson_env
source ~/jetson_env/bin/activate
```

2. Install the Jetson-specific requirements:
```bash
cd /path/to/dimos
pip install -r docker/jetson/jetson_requirements.txt
```

## Docker Setup

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

## Testing

The default test script (`test_agent_huggingface_local_jetson.py`) will run automatically when using Docker Compose. 

To run tests manually in your virtual environment:
```bash
python3 tests/test_agent_huggingface_local_jetson.py
```

## Troubleshooting

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
