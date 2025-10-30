#!/bin/bash

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Building DimOS + ROS Autonomy Stack Docker Image${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ ! -d "ros-navigation-autonomy-stack" ]; then
    echo -e "${YELLOW}Cloning ros-navigation-autonomy-stack repository...${NC}"
    git clone -b jazzy git@github.com:dimensionalOS/ros-navigation-autonomy-stack.git
    echo -e "${GREEN}Repository cloned successfully!${NC}"
fi

if [ ! -d "unity_models" ]; then
    echo -e "${YELLOW}Using office_building_1 as the Unity environment...${NC}"
    tar -xf ../../data/.lfs/office_building_1.tar.gz
    mv office_building_1 unity_models
fi

echo ""
echo -e "${YELLOW}Building Docker image with docker compose...${NC}"
echo "This will take a while as it needs to:"
echo "  - Download base ROS Jazzy image"
echo "  - Install ROS packages and dependencies"
echo "  - Build the autonomy stack"
echo "  - Install Python dependencies for DimOS"
echo ""

cd ../..

docker compose -f docker/navigation/docker-compose.yml build

echo ""
echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}Docker image built successfully!${NC}"
echo -e "${GREEN}================================${NC}"
echo ""
echo "You can now run the container using:"
echo -e "${YELLOW}  ./start.sh --all${NC}"
echo ""
