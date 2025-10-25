#!/bin/bash

# Test script to verify ROS-DimOS integration

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}=========================================${NC}"
echo -e "${CYAN}Testing ROS-DimOS Integration${NC}"
echo -e "${CYAN}=========================================${NC}"
echo ""

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Allow X server connection from Docker
xhost +local:docker 2>/dev/null || true

# Go to dimos directory (parent of ros_docker_integration) for docker compose context
cd ..

echo -e "${YELLOW}Test 1: Checking ROS environment...${NC}"
docker compose -f ros_docker_integration/docker-compose.yml run --rm dimos_autonomy_stack bash -c "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    echo 'ROS_DISTRO: \$ROS_DISTRO' &&
    echo 'ROS workspace: /ros2_ws' &&
    ros2 pkg list | grep -E '(base_autonomy|vehicle_simulator)' | head -5
"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ ROS environment is properly configured${NC}"
else
    echo -e "${RED}✗ ROS environment check failed${NC}"
fi

echo ""
echo -e "${YELLOW}Test 2: Checking DimOS Python environment...${NC}"
docker compose -f ros_docker_integration/docker-compose.yml run --rm dimos_autonomy_stack bash -c "
    source /home/p/pro/dimensional/dimos/.venv/bin/activate &&
    python -c '
import sys
print(f\"Python: {sys.version}\")
try:
    import dimos
    print(\"✓ DimOS package is importable\")
except ImportError as e:
    print(f\"✗ DimOS import failed: {e}\")
    sys.exit(1)

try:
    import cv2
    print(\"✓ OpenCV is available\")
except ImportError:
    print(\"✗ OpenCV import failed\")

try:
    import torch
    print(f\"✓ PyTorch is available\")
except ImportError:
    print(\"✓ PyTorch not installed (CPU mode)\")
'
"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ DimOS Python environment is properly configured${NC}"
else
    echo -e "${RED}✗ DimOS Python environment check failed${NC}"
fi

echo ""
echo -e "${YELLOW}Test 3: Checking GPU access...${NC}"
docker compose -f ros_docker_integration/docker-compose.yml run --rm dimos_autonomy_stack bash -c "
    if command -v nvidia-smi &> /dev/null; then
        nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
        echo '✓ GPU is accessible'
    else
        echo '✗ nvidia-smi not found - GPU may not be accessible'
    fi
"

echo ""
echo -e "${YELLOW}Test 4: Checking network configuration...${NC}"
docker compose -f ros_docker_integration/docker-compose.yml run --rm dimos_autonomy_stack bash -c "
    echo 'Network mode: host'
    echo -n 'Can access localhost: '
    if ping -c 1 localhost &> /dev/null; then
        echo '✓ Yes'
    else
        echo '✗ No'
    fi
"

echo ""
echo -e "${YELLOW}Test 5: Checking ROS topic availability...${NC}"
docker compose -f ros_docker_integration/docker-compose.yml run --rm dimos_autonomy_stack bash -c "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    timeout 2 ros2 topic list 2>/dev/null | head -10 || echo 'Note: No ROS nodes running (this is expected)'
"

# Revoke X server access when done
xhost -local:docker 2>/dev/null || true

echo ""
echo -e "${CYAN}=========================================${NC}"
echo -e "${CYAN}Integration Test Complete${NC}"
echo -e "${CYAN}=========================================${NC}"
echo ""
echo "To run the full system:"
echo -e "${YELLOW}  ./start.sh --all${NC}"
echo ""
echo "For interactive testing:"
echo -e "${YELLOW}  ./shell.sh${NC}"