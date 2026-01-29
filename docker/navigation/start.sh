#!/bin/bash

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Parse command line arguments
MODE="simulation"
USE_ROUTE_PLANNER="false"
USE_RVIZ="false"
DEV_MODE="false"
ROS_DISTRO="humble"
LOCALIZATION_METHOD="${LOCALIZATION_METHOD:-arise_slam}"
while [[ $# -gt 0 ]]; do
    case $1 in
        --hardware)
            MODE="hardware"
            shift
            ;;
        --simulation)
            MODE="simulation"
            shift
            ;;
        --bagfile)
            MODE="bagfile"
            shift
            ;;
        --route-planner)
            USE_ROUTE_PLANNER="true"
            shift
            ;;
        --rviz)
            USE_RVIZ="true"
            shift
            ;;
        --dev)
            DEV_MODE="true"
            shift
            ;;
        --image)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                echo -e "${RED}--image requires a value (humble or jazzy)${NC}"
                exit 1
            fi
            ROS_DISTRO="$2"
            shift 2
            ;;
        --localization)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                echo -e "${RED}--localization requires a value (arise_slam or fastlio)${NC}"
                exit 1
            fi
            LOCALIZATION_METHOD="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Mode (mutually exclusive):"
            echo "  --simulation              Start simulation container (default)"
            echo "  --hardware                Start hardware container"
            echo "  --bagfile                  Start bagfile playback container (use_sim_time=true)"
            echo ""
            echo "Image and localization:"
            echo "  --image <distro>           ROS 2 distribution: humble (default), jazzy"
            echo "  --localization <method>    SLAM method: arise_slam (default), fastlio"
            echo ""
            echo "Additional options:"
            echo "  --route-planner            Enable FAR route planner (for hardware mode)"
            echo "  --rviz                     Launch RViz2 visualization"
            echo "  --dev                      Development mode (mount src for config editing)"
            echo "  --help, -h                 Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0 --simulation                                        # Start simulation"
            echo "  $0 --hardware --image jazzy                            # Hardware with Jazzy"
            echo "  $0 --hardware --localization fastlio                    # Hardware with FASTLIO2"
            echo "  $0 --hardware --route-planner --rviz                   # Hardware with route planner + RViz"
            echo "  $0 --hardware --dev                                    # Hardware with src mounted"
            echo "  $0 --bagfile                                           # Bagfile playback"
            echo "  $0 --bagfile --localization fastlio --route-planner    # Bagfile with FASTLIO2 + route planner"
            echo ""
            echo "Press Ctrl+C to stop the container"
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
export LOCALIZATION_METHOD
export IMAGE_TAG="${ROS_DISTRO}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Starting DimOS Docker Container${NC}"
echo -e "${GREEN}Mode: ${MODE}${NC}"
echo -e "${GREEN}ROS Distribution: ${ROS_DISTRO}${NC}"
echo -e "${GREEN}ROS Domain ID: ${ROS_DOMAIN_ID:-42}${NC}"
echo -e "${GREEN}Localization: ${LOCALIZATION_METHOD}${NC}"
echo -e "${GREEN}Image Tag: ${IMAGE_TAG}${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""

# Pull image option removed - use build.sh to build locally

# Hardware-specific checks
if [ "$MODE" = "hardware" ]; then
    # Check if .env file exists
    if [ ! -f ".env" ]; then
        if [ -f ".env.hardware" ]; then
            echo -e "${YELLOW}Creating .env from .env.hardware template...${NC}"
            cp .env.hardware .env
            echo -e "${RED}Please edit .env file with your hardware configuration:${NC}"
            echo "  - LIDAR_IP: Full IP address of your Mid-360 lidar"
            echo "  - LIDAR_COMPUTER_IP: IP address of this computer on the lidar subnet"
            echo "  - LIDAR_INTERFACE: Network interface connected to lidar"
            echo "  - MOTOR_SERIAL_DEVICE: Serial device for motor controller"
            echo ""
            echo "After editing, run this script again."
            exit 1
        fi
    fi

    # Source the environment file
    if [ -f ".env" ]; then
        set -a
        source .env
        set +a
    fi

    # Auto-detect group IDs for device permissions
    echo -e "${GREEN}Detecting device group IDs...${NC}"
    export INPUT_GID=$(getent group input | cut -d: -f3 || echo "995")
    export DIALOUT_GID=$(getent group dialout | cut -d: -f3 || echo "20")
    # Warn if fallback values are being used
    if ! getent group input > /dev/null 2>&1; then
        echo -e "${YELLOW}Warning: input group not found, using fallback GID ${INPUT_GID}${NC}"
    fi
    if ! getent group dialout > /dev/null 2>&1; then
        echo -e "${YELLOW}Warning: dialout group not found, using fallback GID ${DIALOUT_GID}${NC}"
    fi
    echo -e "  input group GID: ${INPUT_GID}"
    echo -e "  dialout group GID: ${DIALOUT_GID}"

    if [ -f ".env" ]; then
        # Check for required environment variables
        if [ -z "$LIDAR_IP" ] || [ "$LIDAR_IP" = "192.168.1.116" ]; then
            echo -e "${YELLOW}Warning: LIDAR_IP still using default value in .env${NC}"
            echo "Set LIDAR_IP to the actual IP address of your Mid-360 lidar"
        fi

        if [ -z "$LIDAR_GATEWAY" ]; then
            echo -e "${YELLOW}Warning: LIDAR_GATEWAY not configured in .env${NC}"
            echo "Set LIDAR_GATEWAY to the gateway IP address for the lidar subnet"
        fi

        # Check for robot IP configuration
        if [ -n "$ROBOT_IP" ]; then
            echo -e "${GREEN}Robot IP configured: $ROBOT_IP${NC}"
        else
            echo -e "${YELLOW}Note: ROBOT_IP not configured in .env${NC}"
            echo "Set ROBOT_IP if using network connection to robot"
        fi

        # Check for serial devices
        echo -e "${GREEN}Checking for serial devices...${NC}"
        if [ -e "${MOTOR_SERIAL_DEVICE:-/dev/ttyACM0}" ]; then
            echo -e "  Found device at: ${MOTOR_SERIAL_DEVICE:-/dev/ttyACM0}"
        else
            echo -e "${YELLOW}  Warning: Device not found at ${MOTOR_SERIAL_DEVICE:-/dev/ttyACM0}${NC}"
            echo -e "${YELLOW}  Available serial devices:${NC}"
            ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "    None found"
        fi

        # Check network interface for lidar
        echo -e "${GREEN}Checking network interface for lidar...${NC}"

        # Get available ethernet interfaces
        AVAILABLE_ETH=""
        for i in /sys/class/net/*; do
            if [ "$(cat $i/type 2>/dev/null)" = "1" ] && [ "$i" != "/sys/class/net/lo" ]; then
                interface=$(basename $i)
                if [ -z "$AVAILABLE_ETH" ]; then
                    AVAILABLE_ETH="$interface"
                else
                    AVAILABLE_ETH="$AVAILABLE_ETH, $interface"
                fi
            fi
        done

        if [ -z "$LIDAR_INTERFACE" ]; then
            # No interface configured
            echo -e "${RED}================================================================${NC}"
            echo -e "${RED}    ERROR: ETHERNET INTERFACE NOT CONFIGURED!${NC}"
            echo -e "${RED}================================================================${NC}"
            echo -e "${YELLOW}  LIDAR_INTERFACE not set in .env file${NC}"
            echo ""
            echo -e "${YELLOW}  Your ethernet interfaces: ${GREEN}${AVAILABLE_ETH}${NC}"
            echo ""
            echo -e "${YELLOW}  ACTION REQUIRED:${NC}"
            echo -e "  1. Edit the .env file and set:"
            echo -e "     ${GREEN}LIDAR_INTERFACE=<your_ethernet_interface>${NC}"
            echo -e "  2. Run this script again"
            echo -e "${RED}================================================================${NC}"
            exit 1
        elif ! ip link show "$LIDAR_INTERFACE" &>/dev/null; then
            # Interface configured but doesn't exist
            echo -e "${RED}================================================================${NC}"
            echo -e "${RED}    ERROR: ETHERNET INTERFACE '$LIDAR_INTERFACE' NOT FOUND!${NC}"
            echo -e "${RED}================================================================${NC}"
            echo -e "${YELLOW}  You configured: LIDAR_INTERFACE=$LIDAR_INTERFACE${NC}"
            echo -e "${YELLOW}  But this interface doesn't exist on your system${NC}"
            echo ""
            echo -e "${YELLOW}  Your ethernet interfaces: ${GREEN}${AVAILABLE_ETH}${NC}"
            echo ""
            echo -e "${YELLOW}  ACTION REQUIRED:${NC}"
            echo -e "  1. Edit the .env file and change to one of your interfaces:"
            echo -e "     ${GREEN}LIDAR_INTERFACE=<your_actual_ethernet_interface>${NC}"
            echo -e "  2. Run this script again"
            echo -e "${RED}================================================================${NC}"
            exit 1
        else
            # Interface exists and is configured correctly
            echo -e "  ${GREEN}✓${NC} Network interface $LIDAR_INTERFACE found"
            echo -e "  ${GREEN}✓${NC} Will configure static IP: ${LIDAR_COMPUTER_IP}/24"
            echo -e "  ${GREEN}✓${NC} Will set gateway: ${LIDAR_GATEWAY}"
            echo ""
            echo -e "${YELLOW}  Network configuration mode: Static IP (Manual)${NC}"
            echo -e "  This will temporarily replace DHCP with static IP assignment"
            echo -e "  Configuration reverts when container stops"
        fi
    fi

fi

# Check if the image exists
if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^dimos_autonomy_stack:${IMAGE_TAG}$"; then
    echo -e "${RED}Docker image dimos_autonomy_stack:${IMAGE_TAG} not found.${NC}"
    echo -e "${YELLOW}Please build it first with:${NC}"
    echo -e "  ./build.sh --${ROS_DISTRO}"
    exit 1
fi

# Check for X11 display
if [ -z "$DISPLAY" ]; then
    echo -e "${YELLOW}Warning: DISPLAY not set. GUI applications may not work.${NC}"
    export DISPLAY=:0
else
    echo -e "${GREEN}Using DISPLAY: $DISPLAY${NC}"
fi
export DISPLAY

# Allow X11 connections from Docker
echo -e "${GREEN}Configuring X11 access...${NC}"
xhost +local:docker 2>/dev/null || true

# Setup X11 auth for remote/SSH connections
XAUTH=/tmp/.docker.xauth
touch $XAUTH 2>/dev/null || true
if [ -n "$DISPLAY" ]; then
    xauth nlist $DISPLAY 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - 2>/dev/null || true
    chmod 644 $XAUTH 2>/dev/null || true
    echo -e "${GREEN}X11 auth configured for display: $DISPLAY${NC}"
fi

cleanup() {
    xhost -local:docker 2>/dev/null || true
}

trap cleanup EXIT

# Check for NVIDIA runtime
if docker info 2>/dev/null | grep -q nvidia; then
    echo -e "${GREEN}NVIDIA Docker runtime detected${NC}"
    export DOCKER_RUNTIME=nvidia
    if [ "$MODE" = "hardware" ]; then
        export NVIDIA_VISIBLE_DEVICES=all
        export NVIDIA_DRIVER_CAPABILITIES=all
    fi
else
    echo -e "${YELLOW}NVIDIA Docker runtime not found. GPU acceleration disabled.${NC}"
    export DOCKER_RUNTIME=runc
fi

# Set container name for reference
if [ "$MODE" = "hardware" ]; then
    CONTAINER_NAME="dimos_hardware_container"
elif [ "$MODE" = "bagfile" ]; then
    CONTAINER_NAME="dimos_bagfile_container"
else
    CONTAINER_NAME="dimos_simulation_container"
fi

# Export settings for docker-compose
export USE_ROUTE_PLANNER
export USE_RVIZ

# Print helpful info before starting
echo ""
if [ "$MODE" = "hardware" ]; then
    if [ "$USE_ROUTE_PLANNER" = "true" ]; then
        echo "Hardware mode - Auto-starting ROS real robot system WITH route planner"
        echo ""
        echo "The container will automatically run:"
        echo "  - ROS navigation stack (system_real_robot_with_route_planner.launch)"
        echo "  - FAR Planner for goal-based navigation"
        echo "  - Foxglove Bridge"
    else
        echo "Hardware mode - Auto-starting ROS real robot system (base autonomy)"
        echo ""
        echo "The container will automatically run:"
        echo "  - ROS navigation stack (system_real_robot.launch)"
        echo "  - Foxglove Bridge"
    fi
    if [ "$USE_RVIZ" = "true" ]; then
        echo "  - RViz2 visualization"
    fi
    if [ "$DEV_MODE" = "true" ]; then
        echo ""
        echo -e "  ${YELLOW}Development mode: src folder mounted for config editing${NC}"
    fi
    echo ""
    echo "To enter the container from another terminal:"
    echo -e "    ${YELLOW}docker exec -it ${CONTAINER_NAME} bash${NC}"
elif [ "$MODE" = "bagfile" ]; then
    if [ "$USE_ROUTE_PLANNER" = "true" ]; then
        echo "Bagfile mode - Starting bagfile playback system WITH route planner"
        echo ""
        echo "The container will run (use_sim_time=true):"
        echo "  - ROS navigation stack (system_bagfile_with_route_planner.launch)"
        echo "  - FAR Planner for goal-based navigation"
    else
        echo "Bagfile mode - Starting bagfile playback system (base autonomy)"
        echo ""
        echo "The container will run (use_sim_time=true):"
        echo "  - ROS navigation stack (system_bagfile.launch)"
    fi
    if [ "$USE_RVIZ" = "true" ]; then
        echo "  - RViz2 visualization"
    fi
    echo ""
    echo -e "${YELLOW}Remember to play bagfile with: ros2 bag play --clock <bagfile>${NC}"
    echo ""
    echo "To enter the container from another terminal:"
    echo -e "    ${YELLOW}docker exec -it ${CONTAINER_NAME} bash${NC}"
else
    echo "Simulation mode - Auto-starting ROS simulation and DimOS"
    echo ""
    echo "The container will automatically run:"
    echo "  - ROS navigation stack with route planner"
    echo "  - DimOS navigation demo"
    echo ""
    echo "To enter the container from another terminal:"
    echo "  docker exec -it ${CONTAINER_NAME} bash"
fi

# Note: DISPLAY is now passed directly via environment variable
# No need to write RUNTIME_DISPLAY to .env for local host running

# Create required directories
if [ "$MODE" = "hardware" ]; then
    mkdir -p bagfiles config logs maps
elif [ "$MODE" = "bagfile" ]; then
    mkdir -p bagfiles config maps
fi

# Build compose command
COMPOSE_CMD="docker compose -f docker-compose.yml"
if [ "$DEV_MODE" = "true" ]; then
    COMPOSE_CMD="$COMPOSE_CMD -f docker-compose.dev.yml"
fi

if [ "$MODE" = "hardware" ]; then
    $COMPOSE_CMD --profile hardware up
elif [ "$MODE" = "bagfile" ]; then
    $COMPOSE_CMD --profile bagfile up
else
    $COMPOSE_CMD up
fi
