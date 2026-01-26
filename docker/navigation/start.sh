#!/bin/bash

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Parse command line arguments
MODE="simulation"
USE_ROUTE_PLANNER="false"
USE_EXPLORATION_PLANNER="false"
USE_RVIZ="false"
DEV_MODE="false"
ROS_DISTRO="humble"
BAGFILE_PATH=""
SAVE_EXPLORED_MAP="false"
PLAYBACK_RATE="1.0"
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
        --exploration-planner)
            USE_EXPLORATION_PLANNER="true"
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
        --humble)
            ROS_DISTRO="humble"
            shift
            ;;
        --jazzy)
            ROS_DISTRO="jazzy"
            shift
            ;;
        --save)
            SAVE_EXPLORED_MAP="true"
            shift
            ;;
        -r)
            if [[ -n "$2" && ! "$2" =~ ^- ]]; then
                PLAYBACK_RATE="$2"
                shift 2
            else
                echo -e "${RED}Error: -r requires a rate value (e.g., -r 2.0)${NC}"
                exit 1
            fi
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --simulation          Start simulation container (default)"
            echo "  --hardware            Start hardware container for real robot"
            echo "  --bagfile             Replay a ROS bag file (prompts for selection)"
            echo "  --route-planner       Enable FAR route planner"
            echo "  --exploration-planner Enable TARE exploration planner"
            echo "  --rviz                Launch RViz2 visualization"
            echo "  --save                Save /explored_areas to PLY file after replay"
            echo "  -r <rate>             Playback rate multiplier (e.g., -r 2.0 for 2x speed)"
            echo "  --dev                 Development mode (mount src for config editing)"
            echo "  --humble              Use ROS 2 Humble image (default)"
            echo "  --jazzy               Use ROS 2 Jazzy image"
            echo "  --help, -h            Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Start simulation (Humble)"
            echo "  $0 --jazzy                            # Start simulation (Jazzy)"
            echo "  $0 --hardware                         # Start hardware (base autonomy)"
            echo "  $0 --hardware --route-planner         # Hardware with FAR route planner"
            echo "  $0 --bagfile                          # Replay bagfile (will prompt)"
            echo "  $0 --bagfile --exploration-planner    # Bagfile with TARE exploration"
            echo "  $0 --bagfile -r 2.0                   # Bagfile at 2x speed"
            echo "  $0 --bagfile --save                   # Bagfile and save explored map"
            echo "  $0 --bagfile --exploration-planner --save -r 1.5  # Full example"
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

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Function to select bagfile
select_bagfile() {
    BAGFILES_DIR="$SCRIPT_DIR/bagfiles"

    if [ ! -d "$BAGFILES_DIR" ]; then
        echo -e "${RED}Error: bagfiles directory not found at $BAGFILES_DIR${NC}"
        exit 1
    fi

    # Find all valid bagfiles (folders with metadata.yaml or .mcap files)
    declare -a BAGFILES
    declare -a BAGFILE_NAMES

    # Find ROS2 bag folders (contain metadata.yaml)
    while IFS= read -r -d '' dir; do
        if [ -f "$dir/metadata.yaml" ]; then
            BAGFILES+=("$dir")
            BAGFILE_NAMES+=("$(basename "$dir") (folder)")
        fi
    done < <(find "$BAGFILES_DIR" -maxdepth 2 -type d -print0 2>/dev/null)

    # Find standalone .mcap files
    while IFS= read -r -d '' file; do
        BAGFILES+=("$file")
        BAGFILE_NAMES+=("$(basename "$file")")
    done < <(find "$BAGFILES_DIR" -maxdepth 1 -name "*.mcap" -type f -print0 2>/dev/null)

    # Find standalone .db3 files (ROS2 SQLite format)
    while IFS= read -r -d '' file; do
        BAGFILES+=("$file")
        BAGFILE_NAMES+=("$(basename "$file")")
    done < <(find "$BAGFILES_DIR" -maxdepth 1 -name "*.db3" -type f -print0 2>/dev/null)

    if [ ${#BAGFILES[@]} -eq 0 ]; then
        echo -e "${RED}Error: No bagfiles found in $BAGFILES_DIR${NC}"
        echo "Place your ROS2 bag files (.mcap, .db3, or folders with metadata.yaml) in:"
        echo "  $BAGFILES_DIR"
        exit 1
    fi

    echo -e "${GREEN}================================================${NC}"
    echo -e "${GREEN}Available ROS Bag Files:${NC}"
    echo -e "${GREEN}================================================${NC}"
    echo ""

    for i in "${!BAGFILE_NAMES[@]}"; do
        echo -e "  ${YELLOW}$((i+1)))${NC} ${BAGFILE_NAMES[$i]}"
    done

    echo ""
    read -p "Select bagfile [1-${#BAGFILES[@]}]: " selection

    # Validate selection
    if ! [[ "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -gt ${#BAGFILES[@]} ]; then
        echo -e "${RED}Invalid selection${NC}"
        exit 1
    fi

    BAGFILE_PATH="${BAGFILES[$((selection-1))]}"
    BAGFILE_NAME="${BAGFILE_NAMES[$((selection-1))]}"

    echo ""
    echo -e "${GREEN}Selected: ${BAGFILE_NAME}${NC}"
    echo ""
}

# Bagfile mode - prompt for selection
if [ "$MODE" = "bagfile" ]; then
    select_bagfile
    export BAGFILE_PATH
    # Convert to container path
    BAGFILE_BASENAME=$(basename "$BAGFILE_PATH")
    export CONTAINER_BAGFILE_PATH="/ros2_ws/bagfiles/$BAGFILE_BASENAME"
fi

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Starting DimOS Docker Container${NC}"
echo -e "${GREEN}Mode: ${MODE}${NC}"
echo -e "${GREEN}ROS Distribution: ${ROS_DISTRO}${NC}"
if [ "$MODE" = "bagfile" ]; then
    echo -e "${GREEN}Bagfile: $(basename "$BAGFILE_PATH")${NC}"
fi
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

# Check if the correct ROS distro image exists
if ! docker images | grep -q "dimos_autonomy_stack.*${ROS_DISTRO}"; then
    echo -e "${YELLOW}Docker image for ROS ${ROS_DISTRO} not found. Building...${NC}"
    ./build.sh --${ROS_DISTRO}
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
else
    CONTAINER_NAME="dimos_simulation_container"
fi

# Export settings for docker-compose
export USE_ROUTE_PLANNER
export USE_EXPLORATION_PLANNER
export USE_RVIZ
export SAVE_EXPLORED_MAP
export PLAYBACK_RATE

# Print helpful info before starting
echo ""
if [ "$MODE" = "bagfile" ]; then
    CONTAINER_NAME="dimos_bagfile_container"
    echo "Bagfile mode - Replaying ROS bag with autonomy stack"
    echo ""
    echo "The container will automatically run:"
    if [ "$USE_EXPLORATION_PLANNER" = "true" ]; then
        echo "  - system_bagfile_with_exploration_planner.launch.py"
        echo "  - TARE Exploration Planner"
    elif [ "$USE_ROUTE_PLANNER" = "true" ]; then
        echo "  - system_bagfile_with_route_planner.launch.py"
        echo "  - FAR Route Planner"
    else
        echo "  - system_bagfile.launch.py (base autonomy)"
    fi
    echo "  - ROS2 bag replay: $(basename "$BAGFILE_PATH")"
    if [ "$PLAYBACK_RATE" != "1.0" ]; then
        echo "  - Playback rate: ${PLAYBACK_RATE}x"
    fi
    echo "  - RViz2 + Foxglove Bridge (port 8765)"
    if [ "$SAVE_EXPLORED_MAP" = "true" ]; then
        echo "  - Will save /explored_areas to PLY after replay"
    fi
    echo ""
    echo "To enter the container from another terminal:"
    echo -e "    ${YELLOW}docker exec -it ${CONTAINER_NAME} bash${NC}"
elif [ "$MODE" = "hardware" ]; then
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

# Build compose command with optional dev mode
COMPOSE_CMD="docker compose -f docker-compose.yml"
if [ "$DEV_MODE" = "true" ]; then
    COMPOSE_CMD="$COMPOSE_CMD -f docker-compose.dev.yml"
fi

if [ "$MODE" = "bagfile" ]; then
    $COMPOSE_CMD --profile bagfile up
elif [ "$MODE" = "hardware" ]; then
    $COMPOSE_CMD --profile hardware up
else
    $COMPOSE_CMD up
fi
