#!/bin/bash

# Quick script to enter an interactive shell in the container

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

xhost +local:docker 2>/dev/null || true

echo -e "${GREEN}====================================${NC}"
echo -e "${GREEN}Entering DimOS + ROS Container Shell${NC}"
echo -e "${GREEN}====================================${NC}"
echo ""
echo -e "${YELLOW}Environment:${NC}"
echo "  - ROS workspace: /ros2_ws"
echo "  - DimOS path: /workspace/dimos"
echo "  - Python venv: /opt/dimos-venv"
echo ""
echo -e "${YELLOW}Useful commands:${NC}"
echo "  - ros2 topic list"
echo "  - ros2 launch base_autonomy unity_simulation_bringup.launch.py"
echo "  - source /opt/dimos-venv/bin/activate"
echo "  - python /workspace/dimos/dimos/navigation/rosnav/nav_bot.py"
echo ""

cd ..

# Enter interactive shell
docker compose -f ros_docker_integration/docker-compose.yml run --rm dimos_autonomy_stack bash

xhost -local:docker 2>/dev/null || true