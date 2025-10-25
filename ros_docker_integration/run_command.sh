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

# Check if command was provided
if [ $# -eq 0 ]; then
    echo -e "${RED}Error: No command provided${NC}"
    echo ""
    echo "Usage: $0 \"command to run\""
    echo ""
    echo "Examples:"
    echo "  $0 \"ros2 topic list\""
    echo "  $0 \"ros2 launch base_autonomy unity_simulation_bringup.launch.py\""
    echo "  $0 \"python /home/p/pro/dimensional/dimos/dimos/robot/unitree_webrtc/unitree_g1.py\""
    echo "  $0 \"bash\" # For interactive shell"
    exit 1
fi

# Allow X server connection from Docker
xhost +local:docker 2>/dev/null || true

echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}Running command in DimOS + ROS Container${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""
echo -e "${YELLOW}Command: $@${NC}"
echo ""

# Go to dimos directory (parent of ros_docker_integration) for docker compose context
cd ..

# Run the command in the container
docker compose -f ros_docker_integration/docker-compose.yml run --rm dimos_autonomy_stack bash -c "$@"

# Revoke X server access when done
xhost -local:docker 2>/dev/null || true