#!/usr/bin/env bash
set -euxo pipefail

# Install the CycloneDDS development library
sudo apt-get install -y cyclonedds-dev

# Create a compatibility directory structure
# (required because Ubuntu's multiarch layout doesn't match the expected CMake layout)
sudo mkdir -p /opt/cyclonedds/{lib,bin,include}
sudo ln -sf /usr/lib/x86_64-linux-gnu/libddsc.so* /opt/cyclonedds/lib/
sudo ln -sf /usr/lib/x86_64-linux-gnu/libcycloneddsidl.so* /opt/cyclonedds/lib/
sudo ln -sf /usr/bin/idlc /opt/cyclonedds/bin/
sudo ln -sf /usr/bin/ddsperf /opt/cyclonedds/bin/
sudo ln -sf /usr/include/dds /opt/cyclonedds/include/

rm -fr .venv
CYCLONEDDS_HOME=/opt/cyclonedds uv sync --all-extras --all-groups
