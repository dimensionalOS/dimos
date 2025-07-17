#!/bin/bash

# Install cuSPARSELt
# wget https://developer.download.nvidia.com/compute/cusparselt/0.7.0/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb
# sudo dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb
# sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.7.0/cusparselt-*-keyring.gpg /usr/share/keyrings/
# sudo apt-get update
# sudo apt-get install libcusparselt0 libcusparselt-dev

# Fixes libopenblas.so.0 import error
sudo rm -r /lib/aarch64-linux-gnu/libopenblas.so.0
sudo apt-get update
sudo apt-get remove --purge libopenblas-dev libopenblas0 libopenblas0-dev
sudo apt-get install libopenblas-dev
sudo apt-get update
sudo apt-get remove --purge libopenblas0-openmp
sudo apt-get install libopenblas0-openmp

# Verify libopenblas.so.0 location and access
ls -l /lib/aarch64-linux-gnu/libopenblas.so.0

