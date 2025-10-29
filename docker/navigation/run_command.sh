#!/bin/bash

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ $# -eq 0 ]; then
    echo -e "${RED}Error: No command provided${NC}"
    echo ""
    echo "Usage: $0 \"command to run\""
    echo ""
    echo "Examples:"
    echo "  $0 \"ros2 topic list\""
    echo "  $0 \"ros2 launch base_autonomy unity_simulation_bringup.launch.py\""
    echo "  $0 \"python /workspace/dimos/dimos/navigation/rosnav/nav_bot.py\""
    echo "  $0 \"bash\" # For interactive shell"
    exit 1
fi

xhost +local:docker 2>/dev/null || true

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Running command in DimOS + ROS Container${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Command: $@${NC}"
echo ""

cd ../..

# Run the command in the container
docker compose -f docker/navigation/docker-compose.yml run --rm dimos_autonomy_stack bash -c "$@"

xhost -local:docker 2>/dev/null || true