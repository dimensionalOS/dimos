#!/bin/bash
set -e

git config --global --add safe.directory /workspace/dimos

# Source ROS setup
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE}/install/setup.bash

# Activate Python virtual environment for dimos
source /opt/dimos-venv/bin/activate

# Export ROBOT_CONFIG_PATH for autonomy stack
export ROBOT_CONFIG_PATH="${ROBOT_CONFIG_PATH:-mechanum_drive}"

# Hardware-specific configurations
if [ "${HARDWARE_MODE}" = "true" ]; then
    # Set network buffer sizes for WiFi data transmission (if needed)
    if [ "${ENABLE_WIFI_BUFFER}" = "true" ]; then
        sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864 2>/dev/null || true
        sysctl -w net.core.wmem_max=67108864 net.core.wmem_default=67108864 2>/dev/null || true
    fi
    
    # Configure network interface for Mid-360 lidar if specified
    if [ -n "${LIDAR_INTERFACE}" ] && [ -n "${LIDAR_COMPUTER_IP}" ]; then
        ip addr add ${LIDAR_COMPUTER_IP}/24 dev ${LIDAR_INTERFACE} 2>/dev/null || true
        ip link set ${LIDAR_INTERFACE} up 2>/dev/null || true
        if [ -n "${LIDAR_GATEWAY}" ]; then
            ip route add default via ${LIDAR_GATEWAY} dev ${LIDAR_INTERFACE} 2>/dev/null || true
        fi
    fi
    
    # Generate MID360_config.json if LIDAR_COMPUTER_IP and LIDAR_IP are set
    if [ -n "${LIDAR_COMPUTER_IP}" ] && [ -n "${LIDAR_IP}" ]; then
        cat > ${WORKSPACE}/src/ros-navigation-autonomy-stack/src/utilities/livox_ros_driver2/config/MID360_config.json <<EOF
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "${LIDAR_COMPUTER_IP}",
      "cmd_data_port": 56101,
      "push_msg_ip": "${LIDAR_COMPUTER_IP}",
      "push_msg_port": 56201,
      "point_data_ip": "${LIDAR_COMPUTER_IP}",
      "point_data_port": 56301,
      "imu_data_ip": "${LIDAR_COMPUTER_IP}",
      "imu_data_port": 56401,
      "log_data_ip": "${LIDAR_COMPUTER_IP}",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "${LIDAR_IP}",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
EOF
        echo "Generated MID360_config.json with LIDAR_COMPUTER_IP=${LIDAR_COMPUTER_IP} and LIDAR_IP=${LIDAR_IP}"
    fi
    
    # Display Robot IP configuration if set
    if [ -n "${ROBOT_IP}" ]; then
        echo "Robot IP configured on local network: ${ROBOT_IP}"
    fi
fi

# Execute the command
exec "$@"
