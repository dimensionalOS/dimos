import math
import numpy as np
from typing import Tuple, Optional
from scipy.spatial.transform import Rotation as R
import cv2
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

def ros_msg_to_numpy_grid(costmap_msg) -> Tuple[np.ndarray, float, Tuple[float, float, float]]:
    """Convert ROS OccupancyGrid message to numpy array, resolution, and origin pose"""
    if costmap_msg is None:
        return np.zeros((100, 100), dtype=np.int8), 0.1, (0.0, 0.0, 0.0)
        
    width = costmap_msg.info.width
    height = costmap_msg.info.height
    resolution = costmap_msg.info.resolution
    
    origin_x = costmap_msg.info.origin.position.x
    origin_y = costmap_msg.info.origin.position.y
    
    qx = costmap_msg.info.origin.orientation.x
    qy = costmap_msg.info.origin.orientation.y
    qz = costmap_msg.info.origin.orientation.z
    qw = costmap_msg.info.origin.orientation.w
    origin_theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    
    data = np.array(costmap_msg.data, dtype=np.int8)
    grid = data.reshape((height, width))
    
    return grid, resolution, (origin_x, origin_y, origin_theta)

def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range"""
    return np.arctan2(np.sin(angle), np.cos(angle))

def distance_angle_to_goal_xy(distance: float, angle: float) -> Tuple[float, float]:
    """Convert distance and angle to goal x, y in robot frame"""
    return distance * np.cos(angle), distance * np.sin(angle)

def visualize_local_planner_state(
    occupancy_grid: np.ndarray, 
    grid_resolution: float, 
    grid_origin: Tuple[float, float, float], 
    robot_pose: Tuple[float, float, float], 
    visualization_size: int, 
    robot_width: float, 
    robot_length: float,
    map_size_meters: float = 10.0,
    goal_xy: Optional[Tuple[float, float]] = None, 
    histogram: Optional[np.ndarray] = None,
    selected_direction: Optional[float] = None
) -> np.ndarray:
    """Generate a bird's eye view visualization of the local costmap.
    Optionally includes VFH histogram and selected direction.
    """
    
    robot_x, robot_y, robot_theta = robot_pose
    grid_origin_x, grid_origin_y, _ = grid_origin
    vis_size = visualization_size
    scale = vis_size / map_size_meters
    
    vis_img = np.ones((vis_size, vis_size, 3), dtype=np.uint8) * 255
    center_x = vis_size // 2
    center_y = vis_size // 2
    
    grid_height, grid_width = occupancy_grid.shape
    
    # Calculate robot position relative to grid origin
    robot_rel_x = robot_x - grid_origin_x
    robot_rel_y = robot_y - grid_origin_y
    robot_cell_x = int(robot_rel_x / grid_resolution)
    robot_cell_y = int(robot_rel_y / grid_resolution)
    
    half_size_cells = int(map_size_meters / grid_resolution / 2)

    # Draw grid cells (using standard occupancy coloring)
    for y in range(max(0, robot_cell_y - half_size_cells),
                   min(grid_height, robot_cell_y + half_size_cells)):
        for x in range(max(0, robot_cell_x - half_size_cells),
                       min(grid_width, robot_cell_x + half_size_cells)):
            cell_rel_x_meters = (x - robot_cell_x) * grid_resolution
            cell_rel_y_meters = (y - robot_cell_y) * grid_resolution
            
            img_x = int(center_x + cell_rel_x_meters * scale)
            img_y = int(center_y - cell_rel_y_meters * scale)  # Flip y-axis

            if 0 <= img_x < vis_size and 0 <= img_y < vis_size:
                cell_value = occupancy_grid[y, x]
                if cell_value == -1:
                    color = (200, 200, 200)  # Unknown (Light gray)
                elif cell_value == 0:
                    color = (255, 255, 255)  # Free (White)
                else:  # Occupied
                    # Scale darkness based on occupancy value (0-100)
                    darkness = 255 - int(155 * (cell_value / 100)) - 100
                    color = (darkness, darkness, darkness)  # Shades of gray/black
                
                cell_size_px = max(1, int(grid_resolution * scale))
                cv2.rectangle(vis_img, 
                              (img_x - cell_size_px//2, img_y - cell_size_px//2),
                              (img_x + cell_size_px//2, img_y + cell_size_px//2),
                              color, -1)

    # Draw histogram
    if histogram is not None:
        num_bins = len(histogram)
        max_hist_value = np.max(histogram) if np.max(histogram) > 0 else 1.0
        hist_scale = (vis_size / 2) * 0.8 # Scale histogram lines to 80% of half the viz size
        
        for i in range(num_bins):
            # Angle relative to robot's forward direction
            angle_relative_to_robot = (i / num_bins) * 2 * math.pi - math.pi
            # Angle in the visualization frame (relative to image +X axis)
            vis_angle = angle_relative_to_robot + robot_theta 
            
            normalized_val = histogram[i] / max_hist_value
            line_length = normalized_val * hist_scale
            
            # Calculate endpoint using the visualization angle
            end_x = int(center_x + line_length * math.cos(vis_angle))
            end_y = int(center_y - line_length * math.sin(vis_angle)) # Flipped Y
            
            # Color based on value (e.g., grayscale: darker = higher density)
            intensity = 200 - int(normalized_val * 200) # Darker for higher values
            color = (intensity, intensity, intensity)
            
            cv2.line(vis_img, (center_x, center_y), (end_x, end_y), color, 1)

    # Draw robot
    robot_length_px = int(robot_length * scale)
    robot_width_px = int(robot_width * scale)
    robot_pts = np.array([
        [-robot_length_px/2, -robot_width_px/2], [robot_length_px/2, -robot_width_px/2],
        [robot_length_px/2, robot_width_px/2], [-robot_length_px/2, robot_width_px/2]
    ], dtype=np.float32)
    rotation_matrix = np.array([
        [math.cos(robot_theta), -math.sin(robot_theta)],
        [math.sin(robot_theta), math.cos(robot_theta)]
    ])
    robot_pts = np.dot(robot_pts, rotation_matrix.T)
    robot_pts[:, 0] += center_x
    robot_pts[:, 1] = center_y - robot_pts[:, 1]  # Flip y-axis
    cv2.fillPoly(vis_img, [robot_pts.reshape((-1, 1, 2)).astype(np.int32)], (0, 0, 255))  # Red robot

    # Draw robot direction line
    front_x = int(center_x + (robot_length_px/2) * math.cos(robot_theta))
    front_y = int(center_y - (robot_length_px/2) * math.sin(robot_theta))
    cv2.line(vis_img, (center_x, center_y), (front_x, front_y), (255, 0, 0), 2)  # Blue line

    # Draw selected direction
    if selected_direction is not None:
        # selected_direction is relative to robot frame
        # Angle in the visualization frame (relative to image +X axis)
        vis_angle_selected = selected_direction + robot_theta

        # Make slightly longer than max histogram line
        sel_dir_line_length = (vis_size / 2) * 0.9 

        sel_end_x = int(center_x + sel_dir_line_length * math.cos(vis_angle_selected))
        sel_end_y = int(center_y - sel_dir_line_length * math.sin(vis_angle_selected)) # Flipped Y
        
        cv2.line(vis_img, (center_x, center_y), (sel_end_x, sel_end_y), (0, 165, 255), 2) # BGR for Orange

    # Draw goal
    if goal_xy is not None:
        goal_x, goal_y = goal_xy
        goal_rel_x_map = goal_x - robot_x
        goal_rel_y_map = goal_y - robot_y
        goal_img_x = int(center_x + goal_rel_x_map * scale)
        goal_img_y = int(center_y - goal_rel_y_map * scale)  # Flip y-axis
        if 0 <= goal_img_x < vis_size and 0 <= goal_img_y < vis_size:
            cv2.circle(vis_img, (goal_img_x, goal_img_y), 5, (0, 255, 0), -1)  # Green circle
            cv2.circle(vis_img, (goal_img_x, goal_img_y), 8, (0, 0, 0), 1)      # Black outline

    # Add scale bar
    scale_bar_length_px = int(1.0 * scale)
    scale_bar_x = vis_size - scale_bar_length_px - 10
    scale_bar_y = vis_size - 20
    cv2.line(vis_img, (scale_bar_x, scale_bar_y), 
             (scale_bar_x + scale_bar_length_px, scale_bar_y), (0, 0, 0), 2)
    cv2.putText(vis_img, "1m", (scale_bar_x, scale_bar_y - 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

    return vis_img
