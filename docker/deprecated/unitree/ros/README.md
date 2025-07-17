# Unitree Go2 ROS Docker Setup

This README explains how to run the Unitree Go2 ROS nodes using Docker.

## Prerequisites

- Docker and Docker Compose installed
- A Unitree Go2 robot accessible on your network
- The robot's IP address

## Configuration

The connection can be configured through environment variables in two ways:

1. Setting them before running docker-compose:
   ```bash
   export ROBOT_IP=192.168.9.140
   export CONN_TYPE=webrtc  # or cyclonedds
   ```

2. Hardcoding them directly in `docker/docker-compose.yaml`

## Usage

To run the ROS nodes:

1. Navigate to the docker directory:
   ```bash
   cd docker/unitree/ros
   ```

2. Run with environment variables:
   ```bash
   xhost +local:root # If running locally and desire RVIZ GUI
   ROBOT_IP=<ROBOT_IP> CONN_TYPE=<webrtc/cyclonedds> docker-compose up --build
   ```

   Where:
   - `<ROBOT_IP>` is your Go2's IP address
   - `<webrtc/cyclonedds>` choose either:
     - `webrtc`: For WebRTC video streaming connection
     - `cyclonedds`: For DDS communication

The containers will build and start, establishing connection with your Go2 robot and opening RVIZ. 


## Known Issues

1. If you encounter the error `unitree_ros-1  | exec /entrypoint.sh: no such file or directory`, this can be caused by:
   - Incorrect file permissions
   - Windows-style line endings (CRLF) in the entrypoint script

   To fix:
   1. Ensure the entrypoint script has execute permissions:
      ```bash
      chmod +x entrypoint.sh
      ```
   
   2. If using Windows, convert line endings to Unix format (LF):
      ```bash
      # Using dos2unix
      dos2unix entrypoint.sh
      
      # Or using sed
      sed -i 's/\r$//' entrypoint.sh
      ```



