#!/usr/bin/env python3
"""
Handle Grab Skill Module for Dimos

This module provides a skill for detecting and grabbing handles using
vision-based normal estimation and IK-based motion planning.
"""

import numpy as np
import cv2
import torch
import open3d as o3d
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import logging
import sys
import os
import time
import re
from typing import Optional, Tuple, Dict, Any
from openai import OpenAI
import ast
import base64
from io import BytesIO
from PIL import Image as PILImage
import threading

# Add FastSAM to path
sys.path.insert(0, os.path.dirname(__file__))
from fastsam_wrapper import FastSAMWrapper

# Drake imports
from pydrake.all import (
    MultibodyPlant,
    Parser,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    MeshcatVisualizer,
    StartMeshcat,
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
    InverseKinematics,
    Solve,
    Box,
    Sphere,
    Cylinder,
    Rgba,
)

# Dimos imports
from dimos.core import Module, In, Out, rpc
from dimos.protocol.skill.skill import skill
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class HandleGrabModule(Module):
    """Module for handle detection and grabbing using vision-based approach."""

    # Input ports for ZED data
    color_image: In[Image] = None
    depth_image: In[Image] = None
    camera_info: In[Any] = None  # Camera calibration info

    def __init__(self,
                 fastsam_model_path: str = "./weights/FastSAM-x.pt",
                 xarm_ip: str = None,
                 test_mode: bool = False,
                 **kwargs):
        """Initialize the handle grab module.

        Args:
            fastsam_model_path: Path to FastSAM model weights
            xarm_ip: IP address of xARM robot (None for simulation only)
            test_mode: If True, only get positions from xARM but don't execute movements
        """
        super().__init__(**kwargs)

        self.fastsam_model_path = fastsam_model_path
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.xarm_ip = xarm_ip
        self.test_mode = test_mode
        self.arm = None
        self.xarm_positions = None

        # Storage for latest ZED data
        self._latest_rgb = None
        self._latest_depth = None
        self._camera_intrinsics = None
        self._data_lock = threading.Lock()
        self._has_data = threading.Event()

        # Store last successful target pose for grab sequence
        self.last_target_position = None
        self.last_target_normal = None
        self.last_q_solution = None
        self.z_offset_applied = 0.1  # Z offset applied in compute_target_pose (10cm)

        # Initialize FastSAM
        self._init_fastsam()

        # Get initial xARM positions if connected
        if self.xarm_ip:
            logger.info(f"\n{'='*60}")
            logger.info(f"Connecting to xARM at {self.xarm_ip}")
            logger.info(f"Test mode: {'ON' if self.test_mode else 'OFF'}")
            logger.info(f"{'='*60}")
            self.xarm_positions = self.get_xarm_positions()
            if self.xarm_positions is None:
                logger.warning("Failed to get xARM positions, using default")
            else:
                logger.info("Successfully retrieved xARM joint positions")

            # Initialize xARM API for movement (if not in test mode)
            if not self.test_mode:
                try:
                    from xarm.wrapper import XArmAPI
                    self.arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)
                    self.arm.clean_error()
                    self.arm.clean_warn()
                    self.arm.motion_enable(enable=True)
                    self.arm.set_mode(0)  # Position control mode
                    self.arm.set_state(0)  # Set to ready state
                    logger.info("xARM API initialized for movement")
                except Exception as e:
                    logger.error(f"Failed to initialize xARM API: {e}")
                    self.arm = None

        # Start meshcat for Drake visualization (disabled in Dimos multiprocess environment)
        self.meshcat = None
        self.enable_meshcat = False  # Disable Meshcat to prevent blocking
        if self.enable_meshcat:
            self.meshcat = StartMeshcat()
            logger.info(f"Meshcat URL: {self.meshcat.web_url()}")
        else:
            logger.info("Meshcat disabled for multiprocess compatibility")

        # Setup Drake simulation
        self.setup_drake_simulation()

    def _init_fastsam(self):
        """Initialize FastSAM model"""
        try:
            if not Path(self.fastsam_model_path).exists():
                logger.warning(f"FastSAM model not found at {self.fastsam_model_path}")
                return

            self.fastsam_model = FastSAMWrapper(self.fastsam_model_path)
            logger.info(f"FastSAM model loaded successfully on {self.device}")

        except Exception as e:
            logger.error(f"Failed to initialize FastSAM: {e}")
            self.fastsam_model = None

    @rpc
    def start(self):
        """Start the module and subscribe to ZED data."""
        logger.info("Starting HandleGrab module")

        # Subscribe to ZED data streams
        if self.color_image:
            self.color_image.subscribe(self._on_color_image)
            logger.info("Subscribed to color image stream")

        if self.depth_image:
            self.depth_image.subscribe(self._on_depth_image)
            logger.info("Subscribed to depth image stream")

        if self.camera_info:
            self.camera_info.subscribe(self._on_camera_info)
            logger.info("Subscribed to camera info stream")

    def _on_color_image(self, msg: Image):
        """Handle incoming color image."""
        try:
            with self._data_lock:
                # Convert Image message data to numpy array (RGB format)
                self._latest_rgb = np.array(msg.data)
                self._has_data.set()
        except Exception as e:
            logger.error(f"Error processing color image: {e}")

    def _on_depth_image(self, msg: Image):
        """Handle incoming depth image."""
        try:
            with self._data_lock:
                # Convert Image message data to numpy array (float32 depth in meters)
                self._latest_depth = np.array(msg.data)
        except Exception as e:
            logger.error(f"Error processing depth image: {e}")

    def _on_camera_info(self, msg):
        """Handle incoming camera calibration info."""
        try:
            with self._data_lock:
                # Extract camera intrinsics from CameraInfo message
                if hasattr(msg, 'K'):
                    # K is a 9-element array for 3x3 intrinsic matrix
                    K = msg.K
                    self._camera_intrinsics = {
                        'fx': K[0],  # Focal length x
                        'fy': K[4],  # Focal length y
                        'cx': K[2],  # Principal point x
                        'cy': K[5],  # Principal point y
                        'width': msg.width,
                        'height': msg.height
                    }
                    logger.debug(f"Updated camera intrinsics: fx={K[0]:.1f}, fy={K[4]:.1f}, cx={K[2]:.1f}, cy={K[5]:.1f}")
        except Exception as e:
            logger.error(f"Error processing camera info: {e}")

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        """Wait for ZED data to be available.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if data is available, False if timeout
        """
        return self._has_data.wait(timeout)

    def get_latest_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get the latest RGB and depth frames from ZED.

        Returns:
            Tuple of (rgb_image, depth_map) or (None, None) if no data
        """
        with self._data_lock:
            return self._latest_rgb.copy() if self._latest_rgb is not None else None, \
                   self._latest_depth.copy() if self._latest_depth is not None else None

    def setup_drake_simulation(self):
        """Setup Drake simulation with the xarm6_openft_gripper robot."""
        # Clear meshcat
        if self.meshcat:
            self.meshcat.Delete()
            self.meshcat.DeleteAddedControls()

        # Create diagram builder
        self.builder = DiagramBuilder()

        # Create plant and scene graph
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder, time_step=0.001
        )

        # Parse URDF
        parser = Parser(self.plant)
        package_path = os.path.dirname(os.path.abspath(__file__))
        parser.package_map().Add("dim_cpp", os.path.join(package_path, "dim_cpp"))

        # Load the URDF
        urdf_path = os.path.join(package_path, "xarm6_openft_gripper.urdf")
        self.model_instances = parser.AddModels(urdf_path)
        self.model_instance = self.model_instances[0] if self.model_instances else None

        # Get important frames
        try:
            self.base_frame = self.plant.world_frame()

            # Try to use link_openft (gripper frame) if available, otherwise use link6
            try:
                self.tool_frame = self.plant.GetFrameByName("link_openft")
                self.tool_body = self.plant.GetBodyByName("link_openft")
                logger.info("Using link_openft as tool frame")
            except:
                self.tool_frame = self.plant.GetFrameByName("link6")
                self.tool_body = self.plant.GetBodyByName("link6")
                logger.info("Using link6 as tool frame")

            # For backward compatibility, keep link6_frame reference
            self.link6_frame = self.tool_frame
            self.link6_body = self.tool_body

            # Note: We'll need to handle camera frame transform differently since we're not directly using ZED
            logger.info("Found required frames: base, tool (link_openft/link6)")
        except Exception as e:
            logger.error(f"Error finding frames: {e}")
            raise

        # Finalize the plant
        self.plant.Finalize()

        # Add visualizer (only if Meshcat is enabled)
        if self.meshcat:
            self.visualizer = MeshcatVisualizer.AddToBuilder(
                self.builder, self.scene_graph, self.meshcat
            )
        else:
            self.visualizer = None

        # Build the diagram
        self.diagram = self.builder.Build()

        # Create contexts
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)

        # Set initial robot configuration
        initial_positions = np.zeros(self.plant.num_positions())

        # Use xARM positions if available
        if self.xarm_positions is not None:
            logger.info("\nSetting Drake to actual xARM joint positions:")
            arm_joint_names = [f"joint{i+1}" for i in range(6)]
            for i, joint_name in enumerate(arm_joint_names):
                try:
                    joint = self.plant.GetJointByName(joint_name)
                    joint_index = joint.position_start()
                    if i < len(self.xarm_positions):
                        initial_positions[joint_index] = self.xarm_positions[i]
                        logger.info(f"  {joint_name}: {np.degrees(self.xarm_positions[i]):.2f} deg")
                except Exception as e:
                    logger.error(f"  Error setting {joint_name}: {e}")

            # Set gripper if we have 7th value
            if len(self.xarm_positions) > 6:
                try:
                    gripper_joint = self.plant.GetJointByName("drive_joint")
                    gripper_index = gripper_joint.position_start()
                    initial_positions[gripper_index] = self.xarm_positions[6]
                    logger.info(f"  gripper: {self.xarm_positions[6]:.3f}")
                except:
                    pass
        else:
            logger.info("Using default (zero) joint positions")

        self.plant.SetPositions(self.plant_context, initial_positions)

        # Set camera view
        if self.meshcat:
            self.meshcat.SetCameraPose(
                camera_in_world=[2.0, 2.0, 1.5],
                target_in_world=[0.0, 0.0, 0.5]
            )

        # Initial publish (only if Meshcat enabled)
        if self.enable_meshcat and self.diagram:
            self.diagram.ForcedPublish(self.diagram_context)

        logger.info("Drake simulation setup complete")

    def select_point(self, rgb_image):
        """Manual point selection (disabled in multiprocess environment)"""
        # In multiprocess environment, we can't use interactive windows
        # Return center of image as a fallback
        logger.warning("Manual point selection not available in multiprocess environment")
        logger.warning("Please use --qwen flag for automatic detection")

        # Save image for reference
        cv2.imwrite("manual_selection_needed.jpg", cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
        logger.info("Saved image to manual_selection_needed.jpg for reference")

        # Return None to skip this iteration
        return None

    def query_qwen_for_point(self, rgb_image):
        """Query Qwen vision model to detect handle point automatically."""
        try:
            # Get API key from environment
            api_key = os.getenv("ALIBABA_API_KEY")
            if not api_key:
                logger.error("ALIBABA_API_KEY environment variable not set")
                logger.info("Falling back to manual point selection")
                return None

            logger.info("Querying Qwen vision model for microwave handle location...")

            # Create Qwen client
            qwen_client = OpenAI(
                base_url="https://dashscope-intl.aliyuncs.com/compatible-mode/v1",
                api_key=api_key,
            )

            # Convert numpy array to PIL Image
            pil_image = PILImage.fromarray(rgb_image)

            # Convert to base64
            buffered = BytesIO()
            pil_image.save(buffered, format="JPEG")
            img_base64 = base64.b64encode(buffered.getvalue()).decode()

            # Create the message with image
            messages = [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": "Please identify a point on the center of the front face of the microwave handle in this image. Return ONLY a point in the center of the front camera facing face of the microwave handle as a tuple in the format (x, y) where x and y are pixel coordinates. Do not include any other text or explanation."
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{img_base64}"
                            }
                        }
                    ]
                }
            ]

            # Query the model
            response = qwen_client.chat.completions.create(
                model="qwen2.5-vl-72b-instruct",
                messages=messages,
                max_tokens=100,
                temperature=0.1
            )

            response_text = response.choices[0].message.content
            logger.info(f"Qwen response: {response_text}")

            # Parse the response to extract coordinates
            coordinates = None

            # Try multiple parsing strategies
            match = re.search(r'\((\d+),\s*(\d+)\)', response_text)
            if match:
                x, y = int(match.group(1)), int(match.group(2))
                coordinates = (x, y)
            else:
                # Try to evaluate as Python literal
                try:
                    clean_text = response_text.strip()
                    if clean_text.startswith('(') and clean_text.endswith(')'):
                        coordinates = ast.literal_eval(clean_text)
                except:
                    pass

                # Look for two numbers
                if coordinates is None:
                    numbers = re.findall(r'\d+', response_text)
                    if len(numbers) >= 2:
                        coordinates = (int(numbers[0]), int(numbers[1]))

            if coordinates:
                x, y = coordinates
                # Validate coordinates are within image bounds
                if 0 <= x < rgb_image.shape[1] and 0 <= y < rgb_image.shape[0]:
                    logger.info(f"Detected handle point at: ({x}, {y})")

                    # Save the detected point image instead of displaying
                    display_img = rgb_image.copy()
                    cv2.circle(display_img, (x, y), 10, (0, 255, 0), -1)
                    cv2.circle(display_img, (x, y), 15, (0, 255, 0), 2)
                    cv2.imwrite("qwen_detected_point.jpg", cv2.cvtColor(display_img, cv2.COLOR_RGB2BGR))
                    logger.info(f"Saved Qwen-detected handle point to qwen_detected_point.jpg")

                    return coordinates
                else:
                    logger.warning(f"Detected coordinates {coordinates} are out of image bounds")
                    return None
            else:
                logger.warning("Could not parse coordinates from Qwen response")
                return None

        except Exception as e:
            logger.error(f"Error querying Qwen: {e}")
            logger.info("Falling back to manual point selection")
            return None

    def segment_with_fastsam(self, rgb_image, point):
        """Segment the handle using FastSAM"""
        if self.fastsam_model is None:
            logger.error("FastSAM model not initialized")
            return None

        # Use single point segmentation
        logger.info(f"Segmenting with FastSAM at point {point}")
        mask = self.fastsam_model.segment_with_point(rgb_image, point, conf=0.1, iou=0.5)

        if mask is not None:
            logger.info(f"Segmentation successful: {np.sum(mask > 0)} pixels")
            # Save mask for debugging
            cv2.imwrite("segmentation_mask.png", mask)
        else:
            logger.warning("Segmentation failed")

        return mask

    def get_xarm_positions(self):
        """Get current joint positions from xARM robot."""
        try:
            from xarm.wrapper import XArmAPI

            logger.info(f"Getting xARM joint positions...")
            arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)

            # Clear any errors
            arm.clean_error()
            arm.clean_warn()

            # Get current joint angles (6 DOF)
            code, angles = arm.get_servo_angle(is_radian=True)

            if code == 0 and angles:
                logger.info(f"Got xARM joint positions:")
                for i, angle in enumerate(angles[:6]):
                    logger.info(f"  joint{i+1}: {angle:.4f} rad ({np.degrees(angle):.2f} deg)")

                # Try to get gripper position
                try:
                    code_gripper, gripper_pos = arm.get_gripper_position()
                    if code_gripper == 0:
                        gripper_rad = gripper_pos / 1000.0  # Rough conversion
                        logger.info(f"  gripper: {gripper_pos:.1f} mm (~{gripper_rad:.3f} rad)")
                        result = list(angles[:6])
                        result.append(gripper_rad)
                        arm.disconnect()
                        return result
                except:
                    pass

                arm.disconnect()
                return angles[:6]
            else:
                logger.error(f"Failed to get xARM positions, code: {code}")

            arm.disconnect()

        except ImportError:
            logger.error("Error: xarm library not installed")
            logger.info("Install with: pip install xarm-python-sdk")
        except Exception as e:
            logger.error(f"Error connecting to xARM: {e}")

        return None

    def execute_xarm_movement(self, q_solution, speed=15):
        """Execute the IK solution on the real xARM robot."""
        if not self.arm:
            logger.warning("xARM API not initialized, cannot execute movement")
            return

        try:
            # Extract joint angles for xARM (first 6 joints)
            arm_joint_names = [f"joint{i+1}" for i in range(6)]
            positions = []

            for joint_name in arm_joint_names:
                joint = self.plant.GetJointByName(joint_name)
                joint_idx = joint.position_start()
                positions.append(q_solution[joint_idx])

            logger.info("Sending joint angles to xARM:")
            for i, angle in enumerate(positions):
                logger.info(f"  joint{i+1}: {np.degrees(angle):.2f} deg")

            # Send command to xARM with specified speed
            logger.info("Sending servo angle command to xARM...")
            code = self.arm.set_servo_angle(angle=positions, speed=speed, wait=True, is_radian=True)
            logger.info(f"Servo angle command returned with code: {code}")

            if code == 0:
                logger.info("Movement executed successfully")
            else:
                logger.error(f"Movement failed with code: {code}")
                # Try to recover
                self.arm.clean_error()
                self.arm.clean_warn()
                self.arm.set_state(0)
                self.arm.set_mode(0)

        except Exception as e:
            logger.error(f"Error executing movement: {e}")

    def extract_segmented_pointcloud(self, depth_map, mask, rgb_image):
        """Extract 3D points from depth map corresponding to the segmented mask.

        Args:
            depth_map: Depth map in meters
            mask: Segmentation mask
            rgb_image: RGB image for colors

        Returns:
            points: 3D points in camera frame (mm)
            colors: RGB colors for points
        """
        mask_indices = np.where(mask > 0)

        points = []
        colors = []

        # Get camera intrinsics from stored calibration or use defaults
        if self._camera_intrinsics is not None:
            fx = self._camera_intrinsics['fx']
            fy = self._camera_intrinsics['fy']
            cx = self._camera_intrinsics['cx']
            cy = self._camera_intrinsics['cy']
            logger.info(f"Using actual camera intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
        else:
            # Fallback to approximate values for HD720 resolution
            fx = 700.0  # Approximate focal length in pixels
            fy = 700.0
            cx = 640.0  # Principal point (image center for 1280x720)
            cy = 360.0
            logger.warning("Using approximate camera intrinsics")

        for y, x in zip(mask_indices[0], mask_indices[1]):
            depth_value = depth_map[y, x]

            # Check for valid depth
            if np.isfinite(depth_value) and depth_value > 0.1 and depth_value < 10.0:
                # Convert to 3D point in camera frame
                z = depth_value * 1000  # Convert to mm
                x_3d = (x - cx) * z / fx
                y_3d = (y - cy) * z / fy

                points.append([x_3d, y_3d, z])

                # Get color from RGB image
                color = rgb_image[y, x] / 255.0
                colors.append(color)

        if len(points) < 100:
            logger.warning(f"Only {len(points)} valid points found")
            return None, None

        logger.info(f"Extracted {len(points)} 3D points from mask")
        return np.array(points), np.array(colors)

    def downsample_pointcloud(self, points, colors, voxel_size=5.0):
        """Downsample point cloud using voxel grid filter"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Downsample
        logger.info(f"Downsampling with voxel size {voxel_size}mm")
        pcd_down = pcd.voxel_down_sample(voxel_size)

        logger.info(f"Points after downsampling: {len(pcd_down.points)}")
        return pcd_down

    def reconstruct_mesh(self, pcd):
        """Reconstruct mesh from point cloud using Poisson reconstruction"""
        logger.info("Computing normals for point cloud...")

        # Estimate normals
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=20.0, max_nn=30))

        # Orient normals consistently
        pcd.orient_normals_consistent_tangent_plane(100)

        # Poisson surface reconstruction
        logger.info("Performing Poisson surface reconstruction...")
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9, width=0, scale=1.1, linear_fit=False)

        # Remove low density vertices to clean up the mesh
        vertices_to_remove = densities < np.quantile(densities, 0.1)
        mesh.remove_vertices_by_mask(vertices_to_remove)

        # Compute mesh normals
        mesh.compute_vertex_normals()

        logger.info(f"Mesh created with {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles")
        return mesh

    def find_normal_at_point(self, mesh, selected_2d_point, depth_map):
        """Find the normal vector at the selected point on the mesh"""
        x_2d, y_2d = selected_2d_point
        depth_value = depth_map[y_2d, x_2d]

        if not np.isfinite(depth_value) or depth_value <= 0:
            logger.warning("Invalid depth at selected position")
            return None, None

        # Convert to 3D point in camera frame
        fx = 700.0
        fy = 700.0
        cx = 640.0
        cy = 360.0

        z = depth_value * 1000  # Convert to mm
        x_3d = (x_2d - cx) * z / fx
        y_3d = (y_2d - cy) * z / fy
        selected_3d = np.array([x_3d, y_3d, z])

        # Find closest vertex on mesh
        mesh_vertices = np.asarray(mesh.vertices)
        distances = np.linalg.norm(mesh_vertices - selected_3d, axis=1)
        closest_vertex_idx = np.argmin(distances)
        closest_distance = distances[closest_vertex_idx]

        logger.info(f"Closest mesh vertex is {closest_distance:.1f}mm away")

        # Get normal at closest vertex
        mesh_normals = np.asarray(mesh.vertex_normals)
        normal = mesh_normals[closest_vertex_idx]

        # Camera looks along positive Z axis, so normal should point back towards camera
        if normal[2] > 0:
            normal = -normal

        # Normalize to ensure unit vector
        normal = normal / np.linalg.norm(normal)

        closest_point = mesh_vertices[closest_vertex_idx]

        logger.info(f"Normal vector in camera frame: [{normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}]")
        logger.info(f"Normal points {'towards' if normal[2] < 0 else 'away from'} camera (z={normal[2]:.3f})")

        return normal, closest_point

    def transform_to_base_frame(self, point_camera, normal_camera):
        """Transform point and normal from camera frame to base frame using Drake robot model.

        This matches the implementation from normal_move_test.py
        """
        # Get current robot configuration
        q = self.plant.GetPositions(self.plant_context)

        # Get the ZED camera frame from the robot model
        try:
            zed_frame = self.plant.GetFrameByName("zed_left_camera_optical_frame")
        except:
            # If the frame doesn't exist in the model, log error and use approximation
            logger.error("zed_left_camera_optical_frame not found in URDF! Using approximation")
            # Fallback to simplified transform
            point_camera_m = point_camera / 1000.0
            camera_position_base = np.array([0.3, 0, 0.5])
            R_base_camera = np.array([
                [0, 0, 1],   # Camera Z -> Base X
                [-1, 0, 0],  # Camera X -> Base -Y
                [0, -1, 0]   # Camera Y -> Base -Z
            ])
            point_base = R_base_camera @ point_camera_m + camera_position_base
            normal_base = R_base_camera @ normal_camera
            normal_base = normal_base / np.linalg.norm(normal_base)
            logger.info(f"Point base (approx): {point_base}")
            logger.info(f"Normal base (approx): {normal_base}")
            return point_base, normal_base

        # Get transform from camera frame to world (base) frame
        X_WC = self.plant.CalcRelativeTransform(
            self.plant_context,
            self.base_frame,
            zed_frame
        )

        # Convert point from mm to m
        point_camera_m = point_camera / 1000.0

        # Transform point to base frame (apply full transform)
        point_base = X_WC @ point_camera_m

        # Transform normal vector (only rotation, as it's a direction)
        normal_base = X_WC.rotation() @ normal_camera

        # Normalize to ensure unit vector
        normal_base = normal_base / np.linalg.norm(normal_base)

        logger.info(f"Camera to base transform:")
        logger.info(f"  Transform position: {X_WC.translation()}")
        logger.info(f"  Transform rotation matrix:\n{X_WC.rotation().matrix()}")
        logger.info(f"  Point camera (m): {point_camera_m}")
        logger.info(f"  Point base: {point_base}")
        logger.info(f"  Normal camera: {normal_camera}")
        logger.info(f"  Normal base: {normal_base}")
        logger.info(f"  Normal magnitude: {np.linalg.norm(normal_base)}")

        return point_base, normal_base

    def compute_target_pose(self, point_base, normal_base, approach_distance=0.25):
        """Compute target pose for tool frame to approach the handle."""
        # Project point along normal by approach_distance
        target_position = point_base + normal_base * approach_distance

        # Lower the target position by 10cm in world Z axis
        z_offset = np.array([0, 0, -0.1])
        target_position = target_position + z_offset

        # Determine orientation based on tool frame convention
        using_openft = "openft" in str(self.tool_frame).lower()

        if using_openft:
            # For link_openft frame
            desired_z_axis = -normal_base  # Points towards handle

            # Y-axis should align with world +Z (upward)
            world_z_up = np.array([0, 0, 1])

            # Check if approach direction is nearly vertical
            z_dot_up = np.dot(desired_z_axis, world_z_up)

            if abs(z_dot_up) > 0.95:
                world_ref = np.array([0, 1, 0])
            else:
                world_ref = world_z_up

            # Project world reference onto plane perpendicular to tool Z-axis
            proj_on_z = np.dot(world_ref, desired_z_axis) * desired_z_axis
            desired_y_axis = world_ref - proj_on_z

            # Check if we got a valid Y-axis
            y_norm = np.linalg.norm(desired_y_axis)
            if y_norm < 0.001:
                if abs(desired_z_axis[0]) < 0.9:
                    desired_y_axis = np.cross(desired_z_axis, np.array([1, 0, 0]))
                else:
                    desired_y_axis = np.cross(desired_z_axis, np.array([0, 1, 0]))

            desired_y_axis = desired_y_axis / np.linalg.norm(desired_y_axis)

            # X-axis is determined by Y and Z
            desired_x_axis = np.cross(desired_y_axis, desired_z_axis)
            desired_x_axis = desired_x_axis / np.linalg.norm(desired_x_axis)

            # Create rotation matrix
            R = np.column_stack([desired_x_axis, desired_y_axis, desired_z_axis])
        else:
            # For link6 frame
            desired_x_axis = -normal_base  # Point towards handle

            world_z = np.array([0, 0, 1])
            world_y = np.array([0, 1, 0])

            if abs(np.dot(desired_x_axis, world_z)) > 0.9:
                desired_y_axis = np.cross(world_y, desired_x_axis)
            else:
                desired_y_axis = np.cross(world_z, desired_x_axis)

            desired_y_axis = desired_y_axis / np.linalg.norm(desired_y_axis)
            desired_z_axis = np.cross(desired_x_axis, desired_y_axis)

            R = np.column_stack([desired_x_axis, desired_y_axis, desired_z_axis])

        target_orientation = RotationMatrix(R)

        logger.info(f"Target position: {target_position}")

        return target_position, target_orientation

    def solve_ik(self, target_position, target_orientation):
        """Solve inverse kinematics for the target pose."""
        # Create IK problem
        ik = InverseKinematics(self.plant, self.plant_context)

        # Add position constraint
        position_tolerance = 0.005  # 5mm tolerance
        ik.AddPositionConstraint(
            self.tool_frame,
            np.zeros(3),
            self.base_frame,
            target_position - position_tolerance * np.ones(3),
            target_position + position_tolerance * np.ones(3)
        )

        # Add orientation constraint
        orientation_tolerance = 0.1  # ~5.7 degrees
        ik.AddOrientationConstraint(
            self.tool_frame,
            RotationMatrix(),
            self.base_frame,
            target_orientation,
            orientation_tolerance
        )

        # Get current joint positions as initial guess
        q_initial = self.plant.GetPositions(self.plant_context)

        # Try multiple initial guesses
        initial_guesses = [
            q_initial,
            np.zeros(self.plant.num_positions()),
        ]

        # Add perturbed version
        q_perturbed = q_initial.copy()
        for i in range(6):
            joint = self.plant.GetJointByName(f"joint{i+1}")
            idx = joint.position_start()
            q_perturbed[idx] += np.random.uniform(-0.1, 0.1)
        initial_guesses.append(q_perturbed)

        prog = ik.get_mutable_prog()

        for attempt, q_guess in enumerate(initial_guesses):
            prog.SetInitialGuess(ik.q(), q_guess)

            logger.info(f"Solving IK (attempt {attempt+1}/{len(initial_guesses)})...")
            result = Solve(prog)

            if result.is_success():
                q_solution = result.GetSolution(ik.q())
                logger.info(f"IK solution found on attempt {attempt+1}!")
                return q_solution

        logger.warning("IK solution not found")
        return None

    def execute_grab_sequence(self):
        """Execute the grab sequence: move up, forward, and close gripper."""
        if self.last_q_solution is None or self.last_target_position is None:
            logger.warning("No successful positioning found, cannot execute grab")
            return False

        if self.test_mode:
            logger.info("Test mode - Would execute grab sequence")
            return True

        if not self.arm:
            logger.warning("xARM API not initialized")
            return False

        try:
            # Step 1: Move up
            logger.info("Moving up 10cm...")
            target_pos_up = np.array(self.last_target_position)
            target_pos_up[2] += self.z_offset_applied

            # Get orientation from last solution
            self.plant.SetPositions(self.plant_context, self.last_q_solution)
            tool_pose = self.plant.CalcRelativeTransform(
                self.plant_context,
                self.base_frame,
                self.tool_frame
            )

            q_up = self.solve_ik(target_pos_up, tool_pose.rotation())
            if q_up is not None:
                self.execute_xarm_movement(q_up, speed=7.5)
                time.sleep(1)

            # Step 2: Move forward
            forward_distance = 0.09
            logger.info(f"Moving forward {forward_distance*100:.0f}cm...")

            approach_direction = -np.array(self.last_target_normal)
            target_pos_forward = target_pos_up + approach_direction * forward_distance

            q_forward = self.solve_ik(target_pos_forward, tool_pose.rotation())
            if q_forward is not None:
                self.execute_xarm_movement(q_forward, speed=7.5)
                time.sleep(1)

            # Step 3: Close gripper
            logger.info("Closing gripper...")
            code = self.arm.set_gripper_position(0, wait=True, speed=5000)

            if code == 0:
                logger.info("Gripper closed successfully")
                return True

        except Exception as e:
            logger.error(f"Error in grab sequence: {e}")

        return False

    def create_debug_visualization(self, rgb_image, mask, selected_point, normal_camera,
                                  point_camera, pcd=None, mesh=None):
        """Create debugging visualization with detection, segmentation, and normals.

        Args:
            rgb_image: Original RGB image
            mask: Segmentation mask
            selected_point: Selected 2D point (x, y)
            normal_camera: Normal vector in camera frame
            point_camera: 3D point in camera frame
            pcd: Open3D point cloud (optional)
            mesh: Open3D mesh (optional)
        """
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        fig = plt.figure(figsize=(20, 10))

        # 1. Original image with detection point
        ax1 = plt.subplot(2, 4, 1)
        ax1.imshow(rgb_image)
        ax1.scatter(selected_point[0], selected_point[1], c='red', s=100, marker='x', linewidths=3)
        ax1.set_title(f'Detection Point: ({selected_point[0]}, {selected_point[1]})')
        ax1.axis('off')

        # 2. Segmentation mask overlay
        ax2 = plt.subplot(2, 4, 2)
        overlay = rgb_image.copy()
        mask_colored = np.zeros_like(rgb_image)
        mask_colored[:, :, 1] = mask * 255  # Green channel for mask
        overlay = cv2.addWeighted(overlay, 0.7, mask_colored, 0.3, 0)
        ax2.imshow(overlay)
        ax2.set_title('Segmentation Mask Overlay')
        ax2.axis('off')

        # 3. Normal vector visualization in 2D
        ax3 = plt.subplot(2, 4, 3)
        ax3.imshow(rgb_image)
        # Project normal to 2D (using first two components)
        normal_2d = normal_camera[:2] / np.linalg.norm(normal_camera[:2]) if np.linalg.norm(normal_camera[:2]) > 0 else normal_camera[:2]
        arrow_scale = 100
        ax3.arrow(selected_point[0], selected_point[1],
                  normal_2d[0] * arrow_scale, -normal_2d[1] * arrow_scale,
                  head_width=20, head_length=10, fc='yellow', ec='yellow', linewidth=3)
        ax3.set_title('Normal Vector (2D projection)')
        ax3.axis('off')

        # 4. 3D visualization of normal in camera frame
        ax4 = plt.subplot(2, 4, 4, projection='3d')
        # Draw coordinate axes
        ax4.quiver(0, 0, 0, 100, 0, 0, color='r', arrow_length_ratio=0.1, label='X')
        ax4.quiver(0, 0, 0, 0, 100, 0, color='g', arrow_length_ratio=0.1, label='Y')
        ax4.quiver(0, 0, 0, 0, 0, 100, color='b', arrow_length_ratio=0.1, label='Z')

        # Draw normal vector
        normal_scaled = normal_camera * 150
        ax4.quiver(0, 0, 0, normal_scaled[0], normal_scaled[1], normal_scaled[2],
                  color='purple', arrow_length_ratio=0.1, linewidth=3, label='Normal')

        ax4.set_xlim([-200, 200])
        ax4.set_ylim([-200, 200])
        ax4.set_zlim([-200, 200])
        ax4.set_xlabel('X (mm)')
        ax4.set_ylabel('Y (mm)')
        ax4.set_zlabel('Z (mm)')
        ax4.legend()
        ax4.set_title('Normal in Camera Frame')

        # 5. Point cloud visualization (if available)
        if pcd is not None:
            ax5 = plt.subplot(2, 4, 5, projection='3d')
            points = np.asarray(pcd.points)

            # Subsample for visualization
            if len(points) > 5000:
                indices = np.random.choice(len(points), 5000, replace=False)
                points = points[indices]

            ax5.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', s=1, alpha=0.3)

            # Add normal vector at point
            if point_camera is not None:
                ax5.scatter(point_camera[0], point_camera[1], point_camera[2], c='r', s=50)
                normal_end = point_camera + normal_camera * 100
                ax5.plot([point_camera[0], normal_end[0]],
                        [point_camera[1], normal_end[1]],
                        [point_camera[2], normal_end[2]], 'g-', linewidth=3)

            ax5.set_xlabel('X (mm)')
            ax5.set_ylabel('Y (mm)')
            ax5.set_zlabel('Z (mm)')
            ax5.set_title('Point Cloud with Normal')

        # 6. Information panel
        ax6 = plt.subplot(2, 4, 6)
        ax6.axis('off')
        info_text = "Detection and Segmentation Info\n" + "=" * 35 + "\n\n"
        info_text += f"Selected Point: ({selected_point[0]}, {selected_point[1]})\n\n"
        info_text += f"Segmented Pixels: {np.sum(mask > 0)}\n\n"
        info_text += f"Point (Camera, mm):\n"
        info_text += f"  X: {point_camera[0]:.1f}\n"
        info_text += f"  Y: {point_camera[1]:.1f}\n"
        info_text += f"  Z: {point_camera[2]:.1f}\n\n"
        info_text += f"Normal (Camera):\n"
        info_text += f"  X: {normal_camera[0]:.3f}\n"
        info_text += f"  Y: {normal_camera[1]:.3f}\n"
        info_text += f"  Z: {normal_camera[2]:.3f}\n"
        info_text += f"  Magnitude: {np.linalg.norm(normal_camera):.3f}\n"

        ax6.text(0.1, 0.9, info_text, transform=ax6.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace')
        ax6.set_title('Detection Info')

        plt.suptitle('Handle Detection Debug Visualization', fontsize=14, fontweight='bold')
        plt.tight_layout()

        # Save the figure
        debug_path = 'debug_visualization.png'
        plt.savefig(debug_path, dpi=150, bbox_inches='tight')
        logger.info(f"Saved debug visualization to {debug_path}")
        plt.close()

    @skill()
    def grab_handle(self,
                    use_qwen: bool = True,
                    loop_count: int = 1,
                    execute_grab: bool = True) -> str:
        """Execute handle detection and grabbing sequence.

        Args:
            use_qwen: If True, use Qwen vision model to automatically detect handle point
            loop_count: Number of times to repeat the detection and movement cycle
            execute_grab: If True, execute grab sequence after positioning

        Returns:
            Status message about the operation
        """
        # Potentially need to set this to just 1
        # loop_count = 1
        try:
            # Wait for ZED data to be available
            if not self.wait_for_data(timeout=10):
                return "Error: No ZED data available"

            successful_iterations = 0

            for iteration in range(1, loop_count + 1):
                logger.info("\n" + "="*60)
                logger.info(f"Iteration {iteration}/{loop_count}")
                logger.info("="*60)

                # Get latest frame from ZED
                rgb_image, depth_map = self.get_latest_frame()

                if rgb_image is None or depth_map is None:
                    logger.error("No frame data available")
                    continue

                # Save debug image
                cv2.imwrite("captured_frame.jpg", cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))

                # Select point (either using Qwen or manual selection)
                if use_qwen:
                    logger.info("Using Qwen vision model for automatic handle detection")
                    selected_point = self.query_qwen_for_point(rgb_image)

                    # If Qwen fails and we're in auto mode, skip this iteration
                    if selected_point is None:
                        logger.warning("Qwen detection failed, skipping iteration")
                        continue
                else:
                    selected_point = self.select_point(rgb_image)

                if selected_point is None:
                    logger.warning("No point selected")
                    continue

                logger.info(f"Selected point: {selected_point}")

                # Segment with FastSAM
                mask = self.segment_with_fastsam(rgb_image, selected_point)
                if mask is None:
                    logger.error("Segmentation failed")
                    continue

                # Extract segmented point cloud
                points, colors = self.extract_segmented_pointcloud(depth_map, mask, rgb_image)
                if points is None:
                    logger.error("Failed to extract point cloud")
                    continue

                # Downsample point cloud - use larger voxel size for faster processing
                pcd = self.downsample_pointcloud(points, colors, voxel_size=10.0)

                # Reconstruct mesh
                mesh = self.reconstruct_mesh(pcd)

                # Find normal at selected point
                normal_camera, closest_point = self.find_normal_at_point(mesh, selected_point, depth_map)
                if normal_camera is None:
                    logger.warning("Could not compute normal vector")
                    continue

                logger.info(f"Normal vector (camera): [{normal_camera[0]:.3f}, {normal_camera[1]:.3f}, {normal_camera[2]:.3f}]")

                # Create debug visualization
                self.create_debug_visualization(rgb_image, mask, selected_point, normal_camera,
                                               closest_point, pcd, mesh)

                # Transform to base frame
                point_base, normal_base = self.transform_to_base_frame(closest_point, normal_camera)

                # Compute target pose (25cm away from surface)
                target_position, target_orientation = self.compute_target_pose(
                    point_base, normal_base, approach_distance=0.25
                )

                # Solve inverse kinematics
                q_solution = self.solve_ik(target_position, target_orientation)

                # Store successful solution for potential grab
                if q_solution is not None:
                    self.last_q_solution = q_solution.copy() if isinstance(q_solution, np.ndarray) else q_solution
                    self.last_target_position = target_position.copy() if isinstance(target_position, np.ndarray) else target_position
                    self.last_target_normal = normal_base.copy() if isinstance(normal_base, np.ndarray) else normal_base

                    # Execute movement if not in test mode
                    if self.arm is not None and not self.test_mode:
                        logger.info("Executing movement to target position")
                        self.execute_xarm_movement(q_solution)
                        logger.info("Movement complete, updating visualization...")
                    elif self.test_mode:
                        logger.info("Test mode - NOT executing movement")

                    # Update Drake visualization (skip if Meshcat disabled)
                    try:
                        self.plant.SetPositions(self.plant_context, q_solution)
                        if self.enable_meshcat and self.diagram:
                            self.diagram.ForcedPublish(self.diagram_context)
                            logger.info("Drake visualization updated")
                        else:
                            logger.info("Drake visualization skipped (Meshcat disabled)")
                    except Exception as e:
                        logger.error(f"Failed to update Drake visualization: {e}")

                    successful_iterations += 1
                    logger.info(f"Iteration {iteration} successful")
                else:
                    logger.warning("No IK solution found for this iteration")
                    continue

                # Wait between iterations if not the last one
                if iteration < loop_count:
                    time.sleep(2)

            logger.info("\n" + "="*60)
            logger.info(f"Completed {successful_iterations}/{loop_count} iterations successfully")
            logger.info("="*60)

            # Execute grab sequence if requested and we had at least one success
            logger.info(f"Checking grab sequence: execute_grab={execute_grab}, successful_iterations={successful_iterations}")
            if execute_grab and successful_iterations > 0:
                logger.info(f"\nExecuting grab sequence...")
                grab_success = self.execute_grab_sequence()
                logger.info(f"Grab sequence result: {grab_success}")
                if grab_success:
                    return f"Grab sequence completed successfully after {successful_iterations} successful iterations"
                else:
                    return f"Grab sequence failed after {successful_iterations} successful iterations"
            elif execute_grab and successful_iterations == 0:
                return "Cannot execute grab sequence - no successful iterations"

            return f"Handle detection completed: {successful_iterations}/{loop_count} successful"

        except Exception as e:
            logger.error(f"Error in grab_handle skill: {e}")
            return f"Error: {str(e)}"

    def cleanup(self):
        """Clean up resources on module destruction."""
        if self.arm:
            self.arm.disconnect()
            logger.info("xARM connection closed")