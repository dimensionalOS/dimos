#!/usr/bin/env bash
# Load Docker images from a tarball
# This script loads Docker images from a tarball file, typically used during
# the installer image build process to load pre-saved images into the chroot.

set -euo pipefail

# Default image tarball path
DEFAULT_IMAGE_TARBALL="/home/ubuntu/setup/artifacts/dimos-images.tar.gz"

# Parse arguments
IMAGE_TARBALL="$DEFAULT_IMAGE_TARBALL"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --image-tarball)
            if [[ $# -lt 2 ]]; then
                echo "Error: Option --image-tarball requires a value" >&2
                exit 1
            fi
            IMAGE_TARBALL="$2"
            shift 2
            ;;
        --image-tarball=*)
            IMAGE_TARBALL="${1#*=}"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--image-tarball <path>]"
            echo "  --image-tarball <path>  Path to Docker images tarball (default: $DEFAULT_IMAGE_TARBALL)"
            exit 0
            ;;
        *)
            echo "Error: Unknown option: $1" >&2
            exit 1
            ;;
    esac
done

# Print warnings
echo "WARNING: Make sure you have logged in to ghcr.io before running this script"
echo "You can do this by running: docker login ghcr.io"
echo ""
echo "WARNING: If you haven't added yourself to the docker group, you will need"
echo "root privileges to invoke docker commands. I.e, you may need to run this "
echo "script with sudo."
echo ""
echo "NOTE: 'docker load' may take several minutes to begin showing any output."
echo "This delay is normal, as Docker first extracts the tarball before "
echo "displaying any visible progress output."
echo ""


# Validate tarball exists
if [[ ! -f "$IMAGE_TARBALL" ]]; then
    echo "Error: Image tarball not found: $IMAGE_TARBALL" >&2
    exit 1
fi

# Check if docker command is available
if ! command -v docker >/dev/null 2>&1; then
    echo "Error: docker command not found. Please install Docker first." >&2
    exit 1
fi

# Load the images
echo "Loading images from: $IMAGE_TARBALL"
docker load -i "$IMAGE_TARBALL"
