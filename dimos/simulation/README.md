# Dimensional Streaming Setup

This guide explains how to set up and run the Isaac Sim streaming functionality via Docker. The setup is tested on Ubuntu 22.04 (recommended).

## Prerequisites

1. **NVIDIA Driver**
   - NVIDIA Driver 535 must be installed
   - Check your driver: `nvidia-smi`
   - If not installed:
   ```bash
   sudo apt-get update
   sudo apt install build-essential -y
   sudo apt-get install -y nvidia-driver-535
   sudo reboot
   ```

2. **CUDA Toolkit**
   ```bash
   sudo apt install -y nvidia-cuda-toolkit
   ```

3. **Docker**
   ```bash
   # Install Docker
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh

   # Post-install steps
   sudo groupadd docker
   sudo usermod -aG docker $USER
   newgrp docker
   ```

4. **NVIDIA Container Toolkit**
   ```bash
   # Add NVIDIA Container Toolkit repository
   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
   curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
   sudo apt-get update

   # Install the toolkit
   sudo apt-get install -y nvidia-container-toolkit
   sudo systemctl restart docker

   # Configure runtime
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker

   # Verify installation
   sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
   ```

5. **Pull Isaac Sim Image**
   ```bash
   sudo docker pull nvcr.io/nvidia/isaac-sim:4.2.0
   ```

6. **TO DO: Add ROS2 websocket server for client-side streaming**

## Running the Streaming Example

1. **Navigate to the docker/simulation directory**
   ```bash
   cd docker/simulation
   ```

2. **Build and run with docker-compose**
   ```bash
   docker compose build
   docker compose up
   ```

This will:
- Build the dimos_simulator image with ROS2 and required dependencies
- Start the MediaMTX RTSP server
- Run the test streaming example from `/tests/isaacsim/stream_camera.py`

## Viewing the Stream

The camera stream will be available at: 

- RTSP: `rtsp://localhost:8554/stream` or `rtsp://<STATIC_IP>:8554/stream`

You can view it using VLC or any RTSP-capable player.