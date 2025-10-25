#!/bin/bash

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}Building DimOS + ROS Autonomy Stack Docker Image${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if autonomy stack already exists
if [ ! -d "autonomy_stack_mecanum_wheel_platform" ]; then
    echo -e "${YELLOW}Cloning autonomy_stack_mecanum_wheel_platform repository...${NC}"
    git clone https://github.com/alexlin2/autonomy_stack_mecanum_wheel_platform.git
    echo -e "${GREEN}Repository cloned successfully!${NC}"
fi

# Check if Unity models directory exists (warn if not)
if [ ! -d "unity_models" ]; then
    echo ""
    echo -e "${YELLOW}WARNING: Unity models directory not found!${NC}"
    echo "If you want to use the Unity simulator, please download Unity environment models"
    echo "and extract them to: ${SCRIPT_DIR}/unity_models/"
    echo "Download from: https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs"
    echo ""
    read -p "Continue building without Unity models? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Build the Docker image using docker compose
echo ""
echo -e "${YELLOW}Building Docker image with docker compose...${NC}"
echo "This may take a while as it needs to:"
echo "  - Download base ROS Jazzy image"
echo "  - Install ROS packages and dependencies"
echo "  - Build the autonomy stack"
echo "  - Install Python dependencies for DimOS"
echo ""

# Go to the dimos directory (parent of ros_docker_integration) for the build context
cd ..

# Build using docker compose
docker compose -f ros_docker_integration/docker-compose.yml build

echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}Docker image built successfully!${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""
echo "You can now run the container using:"
echo -e "${YELLOW}  ./start.sh${NC}"
echo ""
echo "Available options:"
echo "  - Run with default configuration (starts bash): ./start.sh"
echo "  - Run with ROS route planner: ./start.sh --ros-planner"
echo "  - Run with DimOS only: ./start.sh --dimos"
echo "  - Run with both: ./start.sh --all"
