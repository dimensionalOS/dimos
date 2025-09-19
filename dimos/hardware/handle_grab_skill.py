#!/usr/bin/env python3
"""
Handle Grabbing Module for Dimos

This module receives ZED camera data via LCM and provides skills for
detecting handles, estimating normals, and positioning the robot for grasping.
"""

import numpy as np
import cv2
import time
import threading
import os
import sys
import torch
import open3d as o3d
from pathlib import Path
from typing import Optional, Tuple, Dict, Any, List
from collections import deque
from dataclasses import dataclass

from dimos.core import Module, In, rpc
from dimos.msgs.sensor_msgs import Image, CameraInfo, PointCloud2
from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

# Add FastSAM to path after dimos imports
sys.path.insert(0, os.path.dirname(__file__))
try:
    from fastsam_wrapper import FastSAMWrapper
except ImportError:
    FastSAMWrapper = None

# Drake imports for IK
from pydrake.all import (
    MultibodyPlant,
    Parser,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    MeshcatVisualizer,
    StartMeshcat,
    RigidTransform,
    RotationMatrix,
    InverseKinematics,
    Solve,
    Box,
    Sphere,
    Cylinder,
    Rgba,
)

logger = setup_logger(__name__)


@dataclass
class HandleDetection:
    """Container for handle detection results."""
    point_3d: np.ndarray  # 3D point on handle (m)
    normal: np.ndarray     # Normal vector at point (unit vector)
    confidence: float      # Detection confidence
    timestamp: float       # Detection timestamp


class HandleGrabModule(Module):
    """Module for detecting and approaching handles using vision and IK."""

    # Input ports for ZED camera data
    color_image: In[Image] = None
    depth_image: In[Image] = None
    camera_info: In[CameraInfo] = None

    def __init__(
        self,
        fastsam_model_path: str = "./weights/FastSAM-x.pt",
        xarm_ip: Optional[str] = None,
        approach_distance: float = 0.25,
        max_iterations: int = 3,
        enable_visualization: bool = True,
    ):
        """
        Initialize the handle grab module.

        Args:
            fastsam_model_path: Path to FastSAM model weights
            xarm_ip: IP address of xARM robot (None for simulation only)
            approach_distance: Distance to approach from handle (m)
            max_iterations: Maximum refinement iterations
            enable_visualization: Whether to enable Drake visualization
        """
        super().__init__()

        self.fastsam_model_path = fastsam_model_path
        self.xarm_ip = xarm_ip
        self.approach_distance = approach_distance
        self.max_iterations = max_iterations
        self.enable_visualization = enable_visualization

        # Model initialization
        self.fastsam_model = None
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # xARM connection
        self.arm = None
        self.xarm_initialized = False

        # Drake simulation
        self.drake_initialized = False
        self.meshcat = None
        self.plant = None
        self.plant_context = None
        self.diagram = None
        self.diagram_context = None

        # Image buffers
        self.latest_color = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.image_lock = threading.Lock()

        # Detection history
        self.detection_history = deque(maxlen=10)

        logger.info("HandleGrabModule initialized")

    def handle_color_image(self, msg: Image):
        """Handle incoming color image."""
        with self.image_lock:
            # Dimos Image has data as numpy array and format as ImageFormat enum
            if msg.format == ImageFormat.RGB:
                self.latest_color = msg.data.copy()
            elif msg.format == ImageFormat.BGR:
                self.latest_color = cv2.cvtColor(msg.data, cv2.COLOR_BGR2RGB)
            elif msg.format == ImageFormat.GRAY:
                self.latest_color = cv2.cvtColor(msg.data, cv2.COLOR_GRAY2RGB)
            else:
                # Convert to RGB using the built-in method
                rgb_image = msg.to_rgb()
                self.latest_color = rgb_image.data.copy()

            logger.debug(f"Received color image: {msg.width}x{msg.height}, format: {msg.format}")

    def handle_depth_image(self, msg: Image):
        """Handle incoming depth image."""
        with self.image_lock:
            # Dimos Image has data as numpy array and format as ImageFormat enum
            if msg.format == ImageFormat.DEPTH:
                # Already float32 in meters
                self.latest_depth = msg.data.copy()
            elif msg.format == ImageFormat.DEPTH16:
                # Convert from millimeters to meters if uint16
                if msg.dtype == np.uint16:
                    self.latest_depth = msg.data.astype(np.float32) / 1000.0
                else:
                    self.latest_depth = msg.data.astype(np.float32)
            else:
                logger.warning(f"Unexpected depth image format: {msg.format}")
                # Try to use as-is
                self.latest_depth = msg.data.astype(np.float32)

            logger.debug(f"Received depth image: {msg.width}x{msg.height}, format: {msg.format}")

    def handle_camera_info(self, msg: CameraInfo):
        """Handle camera calibration info."""
        with self.image_lock:
            self.latest_camera_info = msg
            logger.debug("Received camera info")

    def initialize_fastsam(self):
        """Initialize FastSAM model."""
        if self.fastsam_model and FastSAMWrapper:
            return True

        try:
            if not Path(self.fastsam_model_path).exists():
                logger.warning(f"FastSAM model not found at {self.fastsam_model_path}")
                return False

            if FastSAMWrapper is None:
                logger.warning("FastSAM wrapper not available")
                return False

            self.fastsam_model = FastSAMWrapper(self.fastsam_model_path)
            logger.info(f"FastSAM model loaded on {self.device}")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize FastSAM: {e}")
            return False

    def initialize_drake(self):
        """Initialize Drake simulation for IK."""
        if self.drake_initialized:
            return True

        try:
            logger.info("Initializing Drake simulation...")

            # Start meshcat if visualization is enabled
            if self.enable_visualization:
                self.meshcat = StartMeshcat()
                logger.info(f"Meshcat URL: {self.meshcat.web_url()}")

            # Create diagram builder
            builder = DiagramBuilder()

            # Create plant and scene graph
            self.plant, scene_graph = AddMultibodyPlantSceneGraph(
                builder, time_step=0.001
            )

            # Parse URDF
            parser = Parser(self.plant)
            package_path = os.path.dirname(os.path.abspath(__file__))
            parser.package_map().Add("dim_cpp", os.path.join(package_path, "dim_cpp"))

            # Load the URDF
            urdf_path = os.path.join(package_path, "xarm6_openft_gripper.urdf")
            if os.path.exists(urdf_path):
                model_instances = parser.AddModels(urdf_path)
                logger.info(f"Loaded URDF from {urdf_path}")
            else:
                logger.warning(f"URDF not found at {urdf_path}")

            # Get important frames
            try:
                self.base_frame = self.plant.world_frame()
                self.tool_frame = self.plant.GetFrameByName("link_openft")
                self.zed_frame = self.plant.GetFrameByName("zed_left_camera_optical_frame")
                logger.info("Found required frames in URDF")
            except Exception as e:
                logger.warning(f"Could not find all frames: {e}")
                # Continue without frames for now

            # Finalize the plant
            self.plant.Finalize()

            # Add visualizer
            if self.enable_visualization and self.meshcat:
                visualizer = MeshcatVisualizer.AddToBuilder(
                    builder, scene_graph, self.meshcat
                )

            # Build the diagram
            self.diagram = builder.Build()

            # Create contexts
            self.diagram_context = self.diagram.CreateDefaultContext()
            self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)

            self.drake_initialized = True
            logger.info("Drake initialization complete")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize Drake: {e}")
            return False

    def initialize_xarm(self):
        """Initialize xARM connection."""
        if self.xarm_initialized or not self.xarm_ip:
            return True

        try:
            from xarm.wrapper import XArmAPI

            logger.info(f"Connecting to xARM at {self.xarm_ip}")
            self.arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)
            self.arm.clean_error()
            self.arm.clean_warn()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)  # Position control mode
            self.arm.set_state(0)  # Set to ready state

            # Get current joint positions and update Drake
            code, angles = self.arm.get_servo_angle(is_radian=True)
            if code == 0 and angles and self.plant:
                initial_positions = np.zeros(self.plant.num_positions())
                for i in range(min(6, len(angles))):
                    joint = self.plant.GetJointByName(f"joint{i+1}")
                    initial_positions[joint.position_start()] = angles[i]
                self.plant.SetPositions(self.plant_context, initial_positions)
                logger.info("Set Drake to current xARM positions")

            self.xarm_initialized = True
            logger.info("xARM connection established")
            return True

        except ImportError:
            logger.error("xarm library not installed")
            return False
        except Exception as e:
            logger.error(f"Failed to connect to xARM: {e}")
            return False

    def detect_handle_point(self, color_image: np.ndarray) -> Optional[Tuple[int, int]]:
        """
        Detect a handle point in the image using AI vision.

        For now, this uses center point or could integrate with a vision model.
        In production, this would query an AI model to identify handle location.

        Args:
            color_image: RGB image

        Returns:
            (x, y) pixel coordinates of handle point, or None if not found
        """
        # TODO: Integrate with actual AI vision model (e.g., qwen or other)
        # For now, use center of image as a placeholder
        h, w = color_image.shape[:2]
        center_x, center_y = w // 2, h // 2

        logger.info(f"Using image center as handle point: ({center_x}, {center_y})")
        return (center_x, center_y)

    def segment_handle(self, color_image: np.ndarray, point: Tuple[int, int]) -> Optional[np.ndarray]:
        """Segment the handle using FastSAM."""
        if not self.fastsam_model:
            logger.warning("FastSAM model not initialized")
            return None

        try:
            logger.info(f"Segmenting handle at point {point}")
            mask = self.fastsam_model.segment_with_point(color_image, point, conf=0.1, iou=0.5)

            if mask is not None:
                logger.info(f"Segmentation successful: {np.sum(mask > 0)} pixels")
            else:
                logger.warning("Segmentation failed")

            return mask

        except Exception as e:
            logger.error(f"Segmentation error: {e}")
            return None

    def reconstruct_mesh_and_normal(
        self,
        color_image: np.ndarray,
        depth_image: np.ndarray,
        mask: np.ndarray,
        point_2d: Tuple[int, int]
    ) -> Optional[HandleDetection]:
        """
        Reconstruct mesh from segmented region and compute normal at selected point.

        Returns:
            HandleDetection with 3D point and normal, or None if failed
        """
        try:
            # Extract 3D points from depth
            h, w = depth_image.shape
            points_3d = []
            colors = []

            # Get camera intrinsics (simplified - would use actual camera_info in production)
            fx = fy = 700.0  # Placeholder focal length
            cx, cy = w / 2, h / 2

            mask_indices = np.where(mask > 0)
            for y, x in zip(mask_indices[0], mask_indices[1]):
                z = depth_image[y, x]
                if z > 0 and z < 5.0:  # Valid depth
                    # Back-project to 3D
                    x_3d = (x - cx) * z / fx
                    y_3d = (y - cy) * z / fy
                    z_3d = z

                    points_3d.append([x_3d, y_3d, z_3d])
                    colors.append(color_image[y, x] / 255.0)

            if len(points_3d) < 100:
                logger.warning(f"Too few points: {len(points_3d)}")
                return None

            points_3d = np.array(points_3d)
            colors = np.array(colors)

            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_3d)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            # Downsample
            pcd = pcd.voxel_down_sample(voxel_size=0.005)

            # Estimate normals
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.02, max_nn=30))

            # Get 3D point at selected 2D location
            x_2d, y_2d = point_2d
            z = depth_image[y_2d, x_2d]
            if z <= 0:
                logger.warning("Invalid depth at selected point")
                return None

            selected_3d = np.array([
                (x_2d - cx) * z / fx,
                (y_2d - cy) * z / fy,
                z
            ])

            # Find closest point in point cloud
            pcd_tree = o3d.geometry.KDTreeFlann(pcd)
            _, idx, _ = pcd_tree.search_knn_vector_3d(selected_3d, 1)

            if len(idx) > 0:
                closest_idx = idx[0]
                closest_point = np.asarray(pcd.points)[closest_idx]
                normal = np.asarray(pcd.normals)[closest_idx]

                # Ensure normal points towards camera (negative z)
                if normal[2] > 0:
                    normal = -normal

                detection = HandleDetection(
                    point_3d=closest_point,
                    normal=normal,
                    confidence=1.0,  # Could compute actual confidence
                    timestamp=time.time()
                )

                logger.info(f"Detection: point={closest_point}, normal={normal}")
                return detection

        except Exception as e:
            logger.error(f"Mesh reconstruction failed: {e}")
            return None

    def compute_approach_pose(
        self,
        detection: HandleDetection
    ) -> Tuple[np.ndarray, RotationMatrix]:
        """
        Compute target pose for approaching the handle.

        Args:
            detection: Handle detection with point and normal

        Returns:
            target_position, target_orientation for end-effector
        """
        # Transform from camera frame to base frame
        # This is simplified - would use actual transform from Drake in production
        point_base = detection.point_3d  # Placeholder
        normal_base = detection.normal   # Placeholder

        # Compute approach position
        target_position = point_base + normal_base * self.approach_distance

        # Apply Z offset for gripper
        target_position[2] -= 0.1  # Lower by 10cm

        # Compute orientation (gripper Z-axis aligned with -normal)
        desired_z_axis = -normal_base
        world_z_up = np.array([0, 0, 1])

        # Y-axis should align with world Z
        if abs(np.dot(desired_z_axis, world_z_up)) > 0.95:
            world_ref = np.array([0, 1, 0])
        else:
            world_ref = world_z_up

        # Project to get Y-axis
        proj_on_z = np.dot(world_ref, desired_z_axis) * desired_z_axis
        desired_y_axis = world_ref - proj_on_z
        desired_y_axis = desired_y_axis / np.linalg.norm(desired_y_axis)

        # X-axis from cross product
        desired_x_axis = np.cross(desired_y_axis, desired_z_axis)

        # Build rotation matrix
        R = np.column_stack([desired_x_axis, desired_y_axis, desired_z_axis])
        target_orientation = RotationMatrix(R)

        logger.info(f"Target pose: pos={target_position}, approach dir={-normal_base}")
        return target_position, target_orientation

    def solve_ik(
        self,
        target_position: np.ndarray,
        target_orientation: RotationMatrix
    ) -> Optional[np.ndarray]:
        """Solve inverse kinematics for target pose."""
        if not self.plant:
            logger.warning("Drake not initialized")
            return None

        try:
            ik = InverseKinematics(self.plant, self.plant_context)

            # Add position constraint
            ik.AddPositionConstraint(
                self.tool_frame,
                np.zeros(3),
                self.base_frame,
                target_position - 0.005 * np.ones(3),
                target_position + 0.005 * np.ones(3)
            )

            # Add orientation constraint
            ik.AddOrientationConstraint(
                self.tool_frame,
                RotationMatrix(),
                self.base_frame,
                target_orientation,
                0.1  # ~5.7 degrees tolerance
            )

            # Solve
            q_initial = self.plant.GetPositions(self.plant_context)
            prog = ik.get_mutable_prog()
            prog.SetInitialGuess(ik.q(), q_initial)

            result = Solve(prog)
            if result.is_success():
                q_solution = result.GetSolution(ik.q())
                logger.info("IK solution found")
                return q_solution
            else:
                logger.warning("IK failed")
                return None

        except Exception as e:
            logger.error(f"IK error: {e}")
            return None

    def execute_movement(self, q_solution: np.ndarray) -> bool:
        """Execute movement on real xARM."""
        if not self.arm:
            logger.warning("xARM not connected")
            return False

        try:
            # Extract joint angles
            positions = []
            for i in range(6):
                joint = self.plant.GetJointByName(f"joint{i+1}")
                positions.append(q_solution[joint.position_start()])

            logger.info("Executing movement...")
            code = self.arm.set_servo_angle(
                angle=positions,
                speed=30,
                wait=True,
                is_radian=True
            )

            if code == 0:
                logger.info("Movement successful")
                return True
            else:
                logger.error(f"Movement failed: code {code}")
                return False

        except Exception as e:
            logger.error(f"Movement error: {e}")
            return False

    @rpc
    def start(self):
        """Start the module."""
        logger.info("Starting HandleGrabModule")

        # Subscribe to image streams
        if self.color_image:
            self.color_image.subscribe(self.handle_color_image)
            logger.info("Subscribed to color image")

        if self.depth_image:
            self.depth_image.subscribe(self.handle_depth_image)
            logger.info("Subscribed to depth image")

        if self.camera_info:
            self.camera_info.subscribe(self.handle_camera_info)
            logger.info("Subscribed to camera info")

        # Initialize components
        self.initialize_fastsam()
        self.initialize_drake()

        if self.xarm_ip:
            self.initialize_xarm()

        logger.info("HandleGrabModule started")

    @rpc
    def stop(self):
        """Stop the module."""
        logger.info("Stopping HandleGrabModule")

        if self.arm:
            self.arm.disconnect()

        logger.info("HandleGrabModule stopped")

    @skill()
    def approach_handle(self, iterations: int = 3) -> str:
        """
        Detect and approach a handle iteratively.

        This skill performs multiple iterations of:
        1. Capture current view
        2. Detect handle point
        3. Segment handle
        4. Estimate normal
        5. Move to approach position

        Args:
            iterations: Number of refinement iterations (default: 3)

        Returns:
            Status message
        """
        logger.info(f"Starting handle approach with {iterations} iterations")

        # Ensure we have images
        with self.image_lock:
            if self.latest_color is None or self.latest_depth is None:
                return "Error: No camera data available. Ensure ZED camera is connected."

        successful_iterations = 0

        for i in range(min(iterations, self.max_iterations)):
            logger.info(f"Iteration {i+1}/{iterations}")

            try:
                with self.image_lock:
                    color = self.latest_color.copy()
                    depth = self.latest_depth.copy()

                # Detect handle point
                handle_point = self.detect_handle_point(color)
                if not handle_point:
                    logger.warning("Handle detection failed")
                    continue

                # Segment handle
                mask = self.segment_handle(color, handle_point)
                if mask is None:
                    logger.warning("Segmentation failed")
                    continue

                # Reconstruct mesh and get normal
                detection = self.reconstruct_mesh_and_normal(
                    color, depth, mask, handle_point
                )
                if not detection:
                    logger.warning("Normal estimation failed")
                    continue

                # Compute approach pose
                target_pos, target_rot = self.compute_approach_pose(detection)

                # Solve IK
                q_solution = self.solve_ik(target_pos, target_rot)
                if q_solution is None:
                    logger.warning("IK solution not found")
                    continue

                # Execute movement
                if self.arm:
                    success = self.execute_movement(q_solution)
                    if success:
                        successful_iterations += 1
                        logger.info(f"Iteration {i+1} successful")

                        # Wait for robot to settle
                        time.sleep(2.0)
                    else:
                        logger.warning(f"Iteration {i+1} movement failed")
                else:
                    # Simulation only
                    self.plant.SetPositions(self.plant_context, q_solution)
                    if self.diagram:
                        self.diagram.ForcedPublish(self.diagram_context)
                    successful_iterations += 1
                    logger.info(f"Iteration {i+1} simulated")

            except Exception as e:
                logger.error(f"Iteration {i+1} error: {e}")

        result = (
            f"Handle approach completed: {successful_iterations}/{iterations} successful iterations. "
        )

        if successful_iterations > 0:
            result += "Robot positioned for handle grasping."
        else:
            result += "Failed to approach handle."

        logger.info(result)
        return result

    @skill()
    def get_handle_detection(self) -> str:
        """Get information about the last handle detection."""
        if not self.detection_history:
            return "No handle detections available."

        latest = self.detection_history[-1]
        info = (
            f"Latest Handle Detection:\n"
            f"- Point: [{latest.point_3d[0]:.3f}, {latest.point_3d[1]:.3f}, {latest.point_3d[2]:.3f}] m\n"
            f"- Normal: [{latest.normal[0]:.3f}, {latest.normal[1]:.3f}, {latest.normal[2]:.3f}]\n"
            f"- Confidence: {latest.confidence:.2f}\n"
            f"- Time: {time.time() - latest.timestamp:.1f}s ago"
        )
        return info