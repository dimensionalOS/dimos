#!/bin/bash

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Allow X server connection from Docker
xhost +local:docker 2>/dev/null || true

echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}Starting DimOS + ROS Autonomy Stack Container${NC}"
echo -e "${GREEN}(Using improved signal handling)${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""

# Check if Unity models exist (warn if not)
if [ ! -d "unity_models" ] && [[ "$*" == *"--ros-planner"* || "$*" == *"--all"* ]]; then
    echo -e "${YELLOW}WARNING: Unity models directory not found!${NC}"
    echo "The Unity simulator may not work properly."
    echo "Download from: https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs"
    echo ""
fi

# Parse command line arguments
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
    echo "  --all           Start both ROS planner and DimOS (with clean shutdown)"
    echo "  --help          Show this help message"
    echo ""
    echo "Without options, starts an interactive bash shell"
    exit 0
fi

# Go to dimos directory (parent of ros_docker_integration) for docker compose context
cd ..

# Set the command based on mode
case $MODE in
    "ros-planner")
        echo -e "${YELLOW}Starting with ROS route planner...${NC}"
        CMD="bash -c 'cd /ros2_ws/src/autonomy_stack_mecanum_wheel_platform && ./system_simulation_with_route_planner.sh'"
        ;;
    "dimos")
        echo -e "${YELLOW}Starting with DimOS Unitree G1 controller...${NC}"
        CMD="python /home/p/pro/dimensional/dimos/dimos/navigation/rosnav/nav_bot.py"
        ;;
    "all")
        echo -e "${YELLOW}Starting both ROS planner and DimOS with improved signal handling...${NC}"
        # Use the Python wrapper for better signal handling
        CMD="python3 /usr/local/bin/ros_launch_wrapper.py"
        ;;
    "default")
        echo -e "${YELLOW}Starting interactive bash shell...${NC}"
        echo ""
        echo "You can manually run:"
        echo "  ROS planner: cd /ros2_ws/src/autonomy_stack_mecanum_wheel_platform && ./system_simulation_with_route_planner.sh"
        echo "  DimOS: python /home/p/pro/dimensional/dimos/dimos/navigation/rosnav/nav_bot.py"
        echo "  Both (clean shutdown): python3 /usr/local/bin/ros_launch_wrapper.py"
        echo ""
        CMD="bash"
        ;;
esac

# Run the container
docker compose -f ros_docker_integration/docker-compose.yml run --rm dimos_autonomy_stack $CMD

# Revoke X server access when done
xhost -local:docker 2>/dev/null || true

echo ""
echo -e "${GREEN}Container stopped.${NC}"