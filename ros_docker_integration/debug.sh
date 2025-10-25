#!/bin/bash

# Quick script to debug paths in the container

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Allow X server connection from Docker
xhost +local:docker 2>/dev/null || true

echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}Running debug diagnostics in container${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""

# Go to dimos directory (parent of ros_docker_integration) for docker compose context
cd ..

# Run the debug script in the container
docker compose -f ros_docker_integration/docker-compose.yml run --rm dimos_autonomy_stack /usr/local/bin/debug_paths.sh

# Revoke X server access when done
xhost -local:docker 2>/dev/null || true