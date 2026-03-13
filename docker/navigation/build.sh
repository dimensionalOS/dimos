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
CLONE_URL="git@github.com:dimensionalOS/ros-navigation-autonomy-stack.git"

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

# Normalize every tracked file's mtime to the HEAD commit timestamp.
# git clone and reset --hard assign the current wall-clock time as mtime,
# so two identical checkouts produce different mtimes and bust Docker's COPY
# cache even when file content is byte-for-byte identical.  Pinning all mtimes
# to the commit timestamp makes the cache key deterministic: same commit →
# same mtimes → cache hit, regardless of when or how the repo was checked out.
echo -e "${GREEN}Pinning ros-navigation-autonomy-stack file timestamps to HEAD commit...${NC}"
(
    cd ros-navigation-autonomy-stack
    COMMIT_TIME=$(git log -1 --format=%ct)
    git ls-files -z | xargs -0 touch -d "@${COMMIT_TIME}"
)

if [ ! -d "unity_models" ]; then
    echo -e "${YELLOW}Using office_building_1 as the Unity environment...${NC}"
    LFS_ASSET="../../data/.lfs/office_building_1.tar.gz"
    # If the file is still a Git LFS pointer (not yet downloaded), fetch it now.
    if file "$LFS_ASSET" | grep -q "ASCII text"; then
        echo -e "${YELLOW}office_building_1.tar.gz is an LFS pointer — fetching via git lfs...${NC}"
        git -C "$(realpath ../../)" lfs pull --include="data/.lfs/office_building_1.tar.gz"
    fi
    tar -xf "$LFS_ASSET"
    mv office_building_1 unity_models
fi

echo ""
echo -e "${YELLOW}Building Docker image with docker compose...${NC}"
echo "This will take a while as it needs to:"
echo "  - Download base ROS ${ROS_DISTRO} image"
echo "  - Install ROS packages and dependencies"
echo "  - Build the autonomy stack (arise_slam + FASTLIO2)"
echo "  - Build Livox-SDK2 for Mid-360 lidar"
echo "  - Build SLAM dependencies (Sophus, Ceres, GTSAM)"
echo "  - Install Python dependencies for DimOS"
echo ""

cd ../..

# Detect host architecture and pass it as a build arg so the Dockerfile's
# base-${TARGETARCH} stage resolves correctly (the standard docker builder
# does not set TARGETARCH automatically without --platform).
HOST_ARCH=$(uname -m)
case "$HOST_ARCH" in
    x86_64)  TARGETARCH="amd64" ;;
    aarch64|arm64) TARGETARCH="arm64" ;;
    *)       TARGETARCH="$HOST_ARCH" ;;
esac
echo -e "${GREEN}Detected architecture: ${HOST_ARCH} → TARGETARCH=${TARGETARCH}${NC}"

# Prefer the Docker Compose V2 plugin; fall back to the legacy standalone binary.
# Auto-install the plugin if neither is available.
if docker compose version &>/dev/null; then
    COMPOSE_CMD="docker compose"
elif command -v docker-compose &>/dev/null; then
    COMPOSE_CMD="docker-compose"
else
    echo -e "${YELLOW}Docker Compose not found — installing docker-compose-plugin...${NC}"
    sudo apt-get update -qq && sudo apt-get install -y docker-compose-v2 || sudo apt-get install -y docker-compose-plugin
    if docker compose version &>/dev/null; then
        COMPOSE_CMD="docker compose"
    else
        echo -e "${RED}Error: Failed to install Docker Compose.${NC}"
        echo "Please install it manually: sudo apt-get install docker-compose-v2"
        echo "or follow https://docs.docker.com/compose/install/"
        exit 1
    fi
fi
echo -e "${GREEN}Using compose command: ${COMPOSE_CMD}${NC}"

echo $COMPOSE_CMD -f docker/navigation/docker-compose.yml build --build-arg TARGETARCH="$TARGETARCH"
$COMPOSE_CMD -f docker/navigation/docker-compose.yml build --build-arg TARGETARCH="$TARGETARCH"

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}Docker image built successfully!${NC}"
echo -e "${GREEN}Image: dimos_autonomy_stack:${IMAGE_TAG}${NC}"
echo -e "${GREEN}SLAM: arise_slam + FASTLIO2 (both included)${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "To run in SIMULATION mode:"
echo -e "${YELLOW}  ./start.sh --simulation --image ${ROS_DISTRO}${NC}"
echo ""
echo "To run in HARDWARE mode:"
echo "  1. Configure your hardware settings in .env file"
echo "     (copy from .env.hardware if needed)"
echo "  2. Run the hardware container:"
echo -e "${YELLOW}     ./start.sh --hardware --image ${ROS_DISTRO}${NC}"
echo ""
echo "To use FASTLIO2 instead of arise_slam, set LOCALIZATION_METHOD:"
echo -e "${YELLOW}     LOCALIZATION_METHOD=fastlio ./start.sh --hardware --${ROS_DISTRO}${NC}"
echo ""
