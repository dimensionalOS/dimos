#!/bin/bash

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

xhost +local:docker 2>/dev/null || true

echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}Starting DimOS + ROS Autonomy Stack Container${NC}"
echo -e "${GREEN}=============================================${NC}"
echo ""

if [ ! -d "unity_models" ] && [[ "$*" == *"--ros-planner"* || "$*" == *"--all"* ]]; then
    echo -e "${YELLOW}WARNING: Unity models directory not found!${NC}"
    echo "The Unity simulator may not work properly."
    echo "Download from: https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs"
    echo ""
fi

if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    export DOCKER_RUNTIME=nvidia
    export NVIDIA_VISIBLE_DEVICES=all
    export NVIDIA_DRIVER_CAPABILITIES=all
    echo -e "${GREEN}GPU mode enabled${NC} (using nvidia runtime)"
    echo ""
else
    export DOCKER_RUNTIME=runc
    export NVIDIA_VISIBLE_DEVICES=
    export NVIDIA_DRIVER_CAPABILITIES=
    echo -e "${YELLOW}CPU-only mode${NC} (no GPU detected)"
    echo ""
fi

MODE="default"
if [[ "$1" == "--ros-planner" ]]; then
    MODE="ros-planner"
elif [[ "$1" == "--dimos" ]]; then
    MODE="dimos"
elif [[ "$1" == "--all" ]]; then
    MODE="all"
elif [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --ros-planner    Start with ROS route planner"
    echo "  --dimos         Start with DimOS Unitree G1 controller"
    echo "  --all           Start both ROS planner and DimOS"
    echo "  --help          Show this help message"
    echo ""
    echo "GPU auto-detection: Automatically uses GPU if nvidia-smi is available"
    echo "Without options, starts an interactive bash shell"
    echo ""
    echo "Example:"
    echo "  $0 --all"
    exit 0
fi

cd ../..

case $MODE in
    "ros-planner")
        echo -e "${YELLOW}Starting with ROS route planner...${NC}"
        CMD="bash -c 'cd /ros2_ws/src/ros-navigation-autonomy-stack && ./system_simulation_with_route_planner.sh'"
        ;;
    "dimos")
        echo -e "${YELLOW}Starting with DimOS navigation bot...${NC}"
        CMD="python /workspace/dimos/dimos/navigation/rosnav/nav_bot.py"
        ;;
    "all")
        echo -e "${YELLOW}Starting both ROS planner and DimOS...${NC}"
        CMD="/usr/local/bin/run_both.sh"
        ;;
    "default")
        echo -e "${YELLOW}Starting interactive bash shell...${NC}"
        echo ""
        echo "You can manually run:"
        echo "  ROS planner: cd /ros2_ws/src/ros-navigation-autonomy-stack && ./system_simulation_with_route_planner.sh"
        echo "  DimOS: python /workspace/dimos/dimos/navigation/rosnav/nav_bot.py"
        echo ""
        CMD="bash"
        ;;
esac

docker compose -f docker/navigation/docker-compose.yml run --rm dimos_autonomy_stack $CMD

xhost -local:docker 2>/dev/null || true

echo ""
echo -e "${GREEN}Container stopped.${NC}"