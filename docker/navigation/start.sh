#!/bin/bash

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Parse command line arguments
MODE="simulation"
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
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --simulation    Start simulation container (default)"
            echo "  --hardware      Start hardware container for real robot"
            echo "  --help, -h      Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                # Start simulation container"
            echo "  $0 --hardware     # Start hardware container"
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

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Starting DimOS Docker Container${NC}"
echo -e "${GREEN}Mode: ${MODE}${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""

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

# Check if unified image exists
if ! docker images | grep -q "dimos_autonomy_stack.*jazzy"; then
    echo -e "${YELLOW}Docker image not found. Building...${NC}"
    ./build.sh
fi

# Check for X11 display
if [ -z "$DISPLAY" ]; then
    echo -e "${YELLOW}Warning: DISPLAY not set. GUI applications may not work.${NC}"
    export DISPLAY=:0
fi

# Allow X11 connections from Docker
echo -e "${GREEN}Configuring X11 access...${NC}"
xhost +local:docker 2>/dev/null || true

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
else
    CONTAINER_NAME="dimos_simulation_container"
fi

# Print helpful info before starting
echo ""
if [ "$MODE" = "hardware" ]; then
    echo "Hardware mode - Interactive shell"
    echo ""
    echo -e "${GREEN}=================================================${NC}"
    echo -e "${GREEN}The container is running. Exec in to run scripts:${NC}"
    echo -e "    ${YELLOW}docker exec -it ${CONTAINER_NAME} bash${NC}"
    echo -e "${GREEN}=================================================${NC}"
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

if [ "$MODE" = "hardware" ]; then
    docker compose -f docker-compose.yml --profile hardware up
else
    docker compose -f docker-compose.yml up
fi
