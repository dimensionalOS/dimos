#!/bin/bash

# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
sudo apt update
sudo apt upgrade -y

# Install ROS 2 Humble (latest LTS for Ubuntu 22.04)
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-ros-base
sudo apt install -y ros-dev-tools

# Install additional ROS 2 packages
sudo apt install -y python3-rosdep
sudo apt install -y python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment variables
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional dependencies that might be useful
sudo apt install -y python3-pip
pip3 install --upgrade pip
pip3 install transforms3d numpy scipy
sudo apt install -y python3.10-venv

# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build

# Source the workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Print success message
echo "ROS 2 Humble installation completed successfully!"
