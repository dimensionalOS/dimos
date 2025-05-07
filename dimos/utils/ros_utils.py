import math
import numpy as np
from typing import Tuple
from scipy.spatial.transform import Rotation as R
import logging

logger = logging.getLogger(__name__)

def ros_msg_to_pose_tuple(odom_msg) -> Tuple[float, float, float]:
    """Convert ROS Odometry message to (x, y, theta) tuple"""
    if odom_msg is None:
        return (0.0, 0.0, 0.0)
        
    # Extract position
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    
    # Extract orientation quaternion
    qx = odom_msg.pose.pose.orientation.x
    qy = odom_msg.pose.pose.orientation.y
    qz = odom_msg.pose.pose.orientation.z
    qw = odom_msg.pose.pose.orientation.w
    
    # Use SciPy to convert quaternion to Euler angles (ZYX order, extract yaw)
    try:
        rotation = R.from_quat([qx, qy, qz, qw])
        euler_angles = rotation.as_euler('zyx', degrees=False)
        theta = euler_angles[0]  # Yaw is the first angle
    except Exception as e:
        logger.error(f"Error converting quaternion to Euler angles: {e}")
        theta = 0.0  # Default to 0 yaw on error
    
    return (x, y, theta)

def ros_msg_to_numpy_grid(costmap_msg) -> Tuple[np.ndarray, Tuple[int, int, float], Tuple[float, float, float]]:
    """Convert ROS OccupancyGrid message to numpy array, resolution, and origin pose"""
    if costmap_msg is None:
        return np.zeros((100, 100), dtype=np.int8), (100, 100, 0.1), (0.0, 0.0, 0.0)
        
    width = costmap_msg.info.width
    height = costmap_msg.info.height
    resolution = costmap_msg.info.resolution

    map_width = width * resolution
    map_height = height * resolution
    
    origin_x = costmap_msg.info.origin.position.x
    origin_y = costmap_msg.info.origin.position.y
    
    qx = costmap_msg.info.origin.orientation.x
    qy = costmap_msg.info.origin.orientation.y
    qz = costmap_msg.info.origin.orientation.z
    qw = costmap_msg.info.origin.orientation.w
    origin_theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    
    data = np.array(costmap_msg.data, dtype=np.int8)
    grid = data.reshape((height, width))
    return grid, (map_width, map_height, resolution), (origin_x, origin_y, origin_theta)

def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range"""
    return np.arctan2(np.sin(angle), np.cos(angle))

def distance_angle_to_goal_xy(distance: float, angle: float) -> Tuple[float, float]:
    """Convert distance and angle to goal x, y in robot frame"""
    return distance * np.cos(angle), distance * np.sin(angle)

