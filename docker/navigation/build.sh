#!/bin/bash

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Default ROS distribution
ROS_DISTRO="humble"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --humble)
            ROS_DISTRO="humble"
            shift
            ;;
        --jazzy)
            ROS_DISTRO="jazzy"
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --humble    Build with ROS 2 Humble (default)"
            echo "  --jazzy     Build with ROS 2 Jazzy"
            echo "  --help, -h  Show this help message"
            echo ""
            echo "The image includes both arise_slam and FASTLIO2."
            echo "Select SLAM method at runtime via LOCALIZATION_METHOD env var."
            echo ""
            echo "Examples:"
            echo "  $0              # Build with ROS Humble"
            echo "  $0 --jazzy      # Build with ROS Jazzy"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Run '$0 --help' for usage information"
            exit 1
            ;;
    esac
done

export ROS_DISTRO
export IMAGE_TAG="${ROS_DISTRO}"

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Building DimOS + ROS Autonomy Stack Docker Image${NC}"
echo -e "${GREEN}ROS Distribution: ${ROS_DISTRO}${NC}"
echo -e "${GREEN}Image Tag: ${IMAGE_TAG}${NC}"
echo -e "${GREEN}SLAM: arise_slam + FASTLIO2 (both included)${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Use fastlio2 branch which has both arise_slam and FASTLIO2
TARGET_BRANCH="fastlio2"
TARGET_REMOTE="origin"
CLONE_URL="https://github.com/dimensionalOS/ros-navigation-autonomy-stack.git"

# Clone or checkout ros-navigation-autonomy-stack
if [ ! -d "ros-navigation-autonomy-stack" ]; then
    echo -e "${YELLOW}Cloning ros-navigation-autonomy-stack repository (${TARGET_BRANCH} branch)...${NC}"
    git clone -b ${TARGET_BRANCH} ${CLONE_URL} ros-navigation-autonomy-stack
    echo -e "${GREEN}Repository cloned successfully!${NC}"
else
    # Directory exists, ensure we're on the correct branch
    cd ros-navigation-autonomy-stack

    CURRENT_BRANCH=$(git branch --show-current)
    if [ "$CURRENT_BRANCH" != "${TARGET_BRANCH}" ]; then
        echo -e "${YELLOW}Switching from ${CURRENT_BRANCH} to ${TARGET_BRANCH} branch...${NC}"
        # Stash any local changes (e.g., auto-generated config files)
        if git stash --quiet 2>/dev/null; then
            echo -e "${YELLOW}Stashed local changes${NC}"
        fi
        git fetch ${TARGET_REMOTE} ${TARGET_BRANCH}
        git checkout -B ${TARGET_BRANCH} ${TARGET_REMOTE}/${TARGET_BRANCH}
        echo -e "${GREEN}Switched to ${TARGET_BRANCH} branch${NC}"
    else
        echo -e "${GREEN}Already on ${TARGET_BRANCH} branch${NC}"
        # Check for local changes before pulling latest
        if ! git diff --quiet || ! git diff --cached --quiet; then
            echo -e "${RED}Local changes detected in ros-navigation-autonomy-stack.${NC}"
            echo -e "${RED}Please commit or discard them before building.${NC}"
            git status --short
            exit 1
        fi
        git fetch ${TARGET_REMOTE} ${TARGET_BRANCH}
        git reset --hard ${TARGET_REMOTE}/${TARGET_BRANCH}
    fi
    cd ..
fi

if [ ! -d "unity_models" ]; then
    echo -e "${YELLOW}Using office_building_1 as the Unity environment...${NC}"
    tar -xf ../../data/.lfs/office_building_1.tar.gz
    mv office_building_1 unity_models
fi

echo ""
echo -e "${YELLOW}Building Docker image with docker compose...${NC}"
echo "This will take a while as it needs to:"
echo "  - Download base ROS ${ROS_DISTRO^} image"
echo "  - Install ROS packages and dependencies"
echo "  - Build the autonomy stack (arise_slam + FASTLIO2)"
echo "  - Build Livox-SDK2 for Mid-360 lidar"
echo "  - Build SLAM dependencies (Sophus, Ceres, GTSAM)"
echo "  - Install Python dependencies for DimOS"
echo ""

cd ../..

docker compose -f docker/navigation/docker-compose.yml build

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}Docker image built successfully!${NC}"
echo -e "${GREEN}Image: dimos_autonomy_stack:${IMAGE_TAG}${NC}"
echo -e "${GREEN}SLAM: arise_slam + FASTLIO2 (both included)${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "To run in SIMULATION mode:"
echo -e "${YELLOW}  ./start.sh --simulation --${ROS_DISTRO}${NC}"
echo ""
echo "To run in HARDWARE mode:"
echo "  1. Configure your hardware settings in .env file"
echo "     (copy from .env.hardware if needed)"
echo "  2. Run the hardware container:"
echo -e "${YELLOW}     ./start.sh --hardware --${ROS_DISTRO}${NC}"
echo ""
echo "To use FASTLIO2 instead of arise_slam, set LOCALIZATION_METHOD:"
echo -e "${YELLOW}     LOCALIZATION_METHOD=fastlio ./start.sh --hardware --${ROS_DISTRO}${NC}"
echo ""
