#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Normal-based IK Motion Test
===========================
This script captures a frame from the ZED camera, allows the user to select
a point on a handle, segments it using FastSAM, performs mesh reconstruction,
calculates the normal vector at the selected point, transforms it to the base frame,
and computes an IK solution to align the end-effector with the handle.
"""

import numpy as np
import cv2
import pyzed.sl as sl
import torch
import open3d as o3d
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from mpl_toolkits.mplot3d import Axes3D
except ImportError:
    # Fallback if 3D plotting has issues
    pass
import logging
import sys
import os
import time
import re
from typing import Optional
from openai import OpenAI
import ast
import base64
from io import BytesIO
from PIL import Image

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

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


class NormalMoveTest:
    """Test normal estimation and IK-based motion planning"""

    def __init__(
        self,
        fastsam_model_path: str = "./weights/FastSAM-x.pt",
        xarm_ip: str = None,
        test_mode: bool = False,
        use_qwen: bool = False,
        loop_count: int = 1,
        execute_grab: bool = False,
    ):
        """Initialize the test system

        Args:
            fastsam_model_path: Path to FastSAM model weights
            xarm_ip: IP address of xARM robot (None for simulation only)
            test_mode: If True, only get positions from xARM but don't execute movements
            use_qwen: If True, use Qwen vision model to automatically detect handle point
            loop_count: Number of times to repeat the detection and movement cycle
            execute_grab: If True, execute grab sequence after final positioning
        """
        self.zed = None
        self.fastsam_model = None
        self.fastsam_model_path = fastsam_model_path
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.xarm_ip = xarm_ip
        self.test_mode = test_mode
        self.use_qwen = use_qwen
        self.loop_count = loop_count
        self.execute_grab = execute_grab
        self.arm = None
        self.xarm_positions = None

        # Store last successful target pose for grab sequence
        self.last_target_position = None
        self.last_target_normal = None
        self.last_q_solution = None
        self.z_offset_applied = 0.1  # Z offset applied in compute_target_pose (10cm)

        # Initialize components
        self._init_camera()
        self._init_fastsam()

        # Storage for current frame data
        self._current_rgb = None
        self._current_depth = None
        self._current_pointcloud = None

        # Get initial xARM positions if connected
        if self.xarm_ip:
            logger.info(f"\n{'=' * 60}")
            logger.info(f"Connecting to xARM at {self.xarm_ip}")
            logger.info(f"Test mode: {'ON' if self.test_mode else 'OFF'}")
            logger.info(f"{'=' * 60}")
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

        # Start meshcat for Drake visualization
        self.meshcat = StartMeshcat()
        logger.info(f"Meshcat URL: {self.meshcat.web_url()}")

        # Setup Drake simulation
        self.setup_drake_simulation()

    def _init_camera(self):
        """Initialize ZED camera"""
        try:
            self.zed = sl.Camera()

            # Configure camera parameters
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps = 30
            init_params.depth_mode = sl.DEPTH_MODE.NEURAL
            init_params.coordinate_units = sl.UNIT.MILLIMETER
            init_params.depth_minimum_distance = 100  # 10cm minimum
            init_params.depth_maximum_distance = 3000  # 3m maximum

            # Open camera
            status = self.zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Failed to open ZED camera: {status}")

            # Set runtime parameters
            self.runtime_params = sl.RuntimeParameters()
            self.runtime_params.confidence_threshold = 50
            self.runtime_params.enable_fill_mode = True

            # Get camera info
            cam_info = self.zed.get_camera_information()
            self.img_width = cam_info.camera_configuration.resolution.width
            self.img_height = cam_info.camera_configuration.resolution.height

            logger.info("ZED camera initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize ZED camera: {e}")
            raise

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

    def setup_drake_simulation(self):
        """Setup Drake simulation with the xarm6_openft_gripper robot."""
        # Clear meshcat
        self.meshcat.Delete()
        self.meshcat.DeleteAddedControls()

        # Create diagram builder
        self.builder = DiagramBuilder()

        # Create plant and scene graph
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.001)

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

            self.zed_frame = self.plant.GetFrameByName("zed_left_camera_optical_frame")
            logger.info(
                "Found all required frames: base, tool (link_openft/link6), zed_left_camera_optical_frame"
            )
        except Exception as e:
            logger.error(f"Error finding frames: {e}")
            raise

        # Finalize the plant
        self.plant.Finalize()

        # Add visualizer
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat
        )

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
            arm_joint_names = [f"joint{i + 1}" for i in range(6)]
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
        self.meshcat.SetCameraPose(camera_in_world=[2.0, 2.0, 1.5], target_in_world=[0.0, 0.0, 0.5])

        # Initial publish
        self.diagram.ForcedPublish(self.diagram_context)

        logger.info("Drake simulation setup complete")

    def capture_frame(self):
        """Capture a frame from the ZED camera"""
        image = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()

        # Warm up camera
        logger.info("Warming up camera...")
        for _ in range(5):
            self.zed.grab(self.runtime_params)

        # Capture frame
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Failed to grab frame from camera")

        # Retrieve data
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        # Convert BGRA to RGB
        image_data = image.get_data()
        if len(image_data.shape) == 3 and image_data.shape[2] == 4:
            rgb_image = cv2.cvtColor(image_data, cv2.COLOR_BGRA2RGB)
        else:
            rgb_image = image_data[:, :, :3]

        # Store current frame data
        self._current_rgb = rgb_image.copy()
        self._current_depth = depth
        self._current_pointcloud = point_cloud

        # Log depth statistics
        depth_data = depth.get_data()
        valid_depth = depth_data[np.isfinite(depth_data)]
        if len(valid_depth) > 0:
            logger.info(f"Depth range: [{np.min(valid_depth):.1f}, {np.max(valid_depth):.1f}] mm")

        return rgb_image, depth, point_cloud

    def select_point(self, rgb_image):
        """Allow user to select a point on the image"""
        selected_point = []

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                selected_point.append((x, y))
                # Draw circle at selected point
                display_img = rgb_image.copy()
                cv2.circle(display_img, (x, y), 5, (0, 255, 0), -1)
                cv2.imshow("Select Point on Handle", display_img)

        # Display image
        cv2.namedWindow("Select Point on Handle", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Select Point on Handle", mouse_callback)
        cv2.imshow("Select Point on Handle", rgb_image)

        logger.info("Click on the handle to select a point. Press 'q' to cancel.")

        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                cv2.destroyAllWindows()
                return None
            elif len(selected_point) > 0:
                cv2.waitKey(500)  # Show selection briefly
                cv2.destroyAllWindows()
                return selected_point[-1]

    def query_qwen_for_point(self, rgb_image):
        """Query Qwen vision model to detect handle point automatically.

        Args:
            rgb_image: RGB image numpy array

        Returns:
            tuple: (x, y) coordinates of the handle point, or None if failed
        """
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
            pil_image = Image.fromarray(rgb_image)

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
                            "text": "Please identify a point on the center of the front face of the microwave handle in this image. Return ONLY a point in the center of the front camera facing face of the microwave handle as a tuple in the format (x, y) where x and y are pixel coordinates. Do not include any other text or explanation.",
                        },
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"},
                        },
                    ],
                }
            ]

            # Query the model
            response = qwen_client.chat.completions.create(
                model="qwen2.5-vl-72b-instruct",  # Using the vision-language model
                messages=messages,
                max_tokens=100,
                temperature=0.1,  # Low temperature for more deterministic output
            )

            response_text = response.choices[0].message.content
            logger.info(f"Qwen response: {response_text}")

            # Parse the response to extract coordinates
            # Try multiple parsing strategies
            coordinates = None

            # Strategy 1: Look for tuple format (x, y)
            match = re.search(r"\((\d+),\s*(\d+)\)", response_text)
            if match:
                x, y = int(match.group(1)), int(match.group(2))
                coordinates = (x, y)
            else:
                # Strategy 2: Try to evaluate as Python literal
                try:
                    # Remove any non-tuple text
                    clean_text = response_text.strip()
                    if clean_text.startswith("(") and clean_text.endswith(")"):
                        coordinates = ast.literal_eval(clean_text)
                except:
                    pass

                # Strategy 3: Look for two numbers
                if coordinates is None:
                    numbers = re.findall(r"\d+", response_text)
                    if len(numbers) >= 2:
                        coordinates = (int(numbers[0]), int(numbers[1]))

            if coordinates:
                x, y = coordinates
                # Validate coordinates are within image bounds
                if 0 <= x < rgb_image.shape[1] and 0 <= y < rgb_image.shape[0]:
                    logger.info(f"Detected handle point at: ({x}, {y})")

                    # Display the detected point briefly
                    display_img = rgb_image.copy()
                    cv2.circle(display_img, (x, y), 10, (0, 255, 0), -1)
                    cv2.circle(display_img, (x, y), 15, (0, 255, 0), 2)
                    cv2.namedWindow("Qwen Detected Handle Point", cv2.WINDOW_NORMAL)
                    cv2.imshow("Qwen Detected Handle Point", display_img)
                    logger.info("Displaying Qwen-detected handle point (green circle)")
                    cv2.waitKey(2000)  # Show for 2 seconds
                    cv2.destroyAllWindows()

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

    def extract_segmented_pointcloud(self, point_cloud_mat, mask):
        """Extract 3D points corresponding to the segmented mask"""
        pc_data = point_cloud_mat.get_data()
        mask_indices = np.where(mask > 0)

        points = []
        colors = []

        for y, x in zip(mask_indices[0], mask_indices[1]):
            point_4d = pc_data[y, x]
            point_3d = point_4d[:3]

            # Check for valid points
            if np.all(np.isfinite(point_3d)) and np.all(np.abs(point_3d) < 10000):
                points.append(point_3d)
                # Get color from RGB image
                color = self._current_rgb[y, x] / 255.0
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
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=20.0, max_nn=30)
        )

        # Orient normals consistently
        pcd.orient_normals_consistent_tangent_plane(100)

        # Poisson surface reconstruction
        logger.info("Performing Poisson surface reconstruction...")
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9, width=0, scale=1.1, linear_fit=False
        )

        # Remove low density vertices to clean up the mesh
        vertices_to_remove = densities < np.quantile(densities, 0.1)
        mesh.remove_vertices_by_mask(vertices_to_remove)

        # Compute mesh normals
        mesh.compute_vertex_normals()

        logger.info(
            f"Mesh created with {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles"
        )
        return mesh

    def find_normal_at_point(self, mesh, selected_2d_point):
        """Find the normal vector at the selected point on the mesh"""
        # Get 3D position of selected point
        pc_data = self._current_pointcloud.get_data()
        x_2d, y_2d = selected_2d_point
        point_4d = pc_data[y_2d, x_2d]
        selected_3d = point_4d[:3]

        if not np.all(np.isfinite(selected_3d)):
            logger.warning("Invalid 3D point at selected position")
            return None, None

        # Find closest vertex on mesh
        mesh_vertices = np.asarray(mesh.vertices)
        distances = np.linalg.norm(mesh_vertices - selected_3d, axis=1)
        closest_vertex_idx = np.argmin(distances)
        closest_distance = distances[closest_vertex_idx]

        logger.info(f"Closest mesh vertex is {closest_distance:.1f}mm away")

        # Get normal at closest vertex
        mesh_normals = np.asarray(mesh.vertex_normals)
        normal = mesh_normals[closest_vertex_idx]

        # ZED camera looks along positive Z axis, so normal should point back towards camera
        # For a handle facing the camera, we want the normal pointing outward from the surface
        # towards the camera (negative Z in camera frame)
        if normal[2] > 0:
            normal = -normal

        # Normalize to ensure unit vector
        normal = normal / np.linalg.norm(normal)

        closest_point = mesh_vertices[closest_vertex_idx]

        logger.info(
            f"Normal vector in camera frame: [{normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}]"
        )
        logger.info(
            f"Normal points {'towards' if normal[2] < 0 else 'away from'} camera (z={normal[2]:.3f})"
        )

        return normal, closest_point

    def transform_to_base_frame(self, point_camera, normal_camera):
        """
        Transform point and normal from camera frame to base frame.

        Args:
            point_camera: 3D point in camera frame (mm)
            normal_camera: Normal vector in camera frame (unit vector)

        Returns:
            point_base: 3D point in base frame (m)
            normal_base: Normal vector in base frame (unit vector)
        """
        # Get current robot configuration
        q = self.plant.GetPositions(self.plant_context)

        # Get transform from camera frame to world (base) frame
        X_WC = self.plant.CalcRelativeTransform(self.plant_context, self.base_frame, self.zed_frame)

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
        """
        Compute target pose for tool frame to approach the handle.

        Args:
            point_base: Handle point in base frame (m)
            normal_base: Handle normal in base frame (unit vector)
            approach_distance: Distance to project along normal (m)

        Returns:
            target_position: Target position for tool frame (m)
            target_orientation: Target orientation for tool frame
        """
        # Project point along normal by approach_distance
        # We move in the direction of the normal (away from the surface)
        target_position = point_base + normal_base * approach_distance

        # Lower the target position by 10cm (0.1m) in world Z axis
        # This helps account for gripper geometry and approach angle
        z_offset = np.array([0, 0, -0.1])  # 10cm down in world frame
        target_position = target_position + z_offset

        logger.info(
            f"Target position before Z offset: {point_base + normal_base * approach_distance}"
        )
        logger.info(f"Applied Z offset: -10cm in world frame")

        # Determine which frame we're using and set tool axis accordingly
        using_openft = "openft" in str(self.tool_frame).lower()

        if using_openft:
            # For link_openft frame (based on continuous_door_opener.py):
            # - Z-axis: Points OUT of the gripper (tool direction)
            # - Y-axis: Should point upward (world +Z) for proper gripper orientation
            # - X-axis: Perpendicular to both (gripper opening direction)

            # Z-axis points towards the handle for approach
            desired_z_axis = -normal_base  # Points towards handle

            # Y-axis should align with world +Z (upward) as much as possible
            # This ensures the gripper fingers are horizontal
            world_z_up = np.array([0, 0, 1])

            # Check if the approach direction is nearly vertical
            z_dot_up = np.dot(desired_z_axis, world_z_up)

            if abs(z_dot_up) > 0.95:  # Nearly vertical approach
                # Use world Y as reference for horizontal approaches
                logger.info("Nearly vertical approach detected, using alternative reference")
                world_ref = np.array([0, 1, 0])
            else:
                # For non-vertical approaches, project world Z onto plane perpendicular to tool Z
                world_ref = world_z_up

            # Project world reference onto plane perpendicular to tool Z-axis
            # This gives us the component of world_ref that is perpendicular to tool Z
            proj_on_z = np.dot(world_ref, desired_z_axis) * desired_z_axis
            desired_y_axis = world_ref - proj_on_z

            # Check if we got a valid Y-axis
            y_norm = np.linalg.norm(desired_y_axis)
            if y_norm < 0.001:
                # Fallback: use any perpendicular direction
                if abs(desired_z_axis[0]) < 0.9:
                    desired_y_axis = np.cross(desired_z_axis, np.array([1, 0, 0]))
                else:
                    desired_y_axis = np.cross(desired_z_axis, np.array([0, 1, 0]))

            desired_y_axis = desired_y_axis / np.linalg.norm(desired_y_axis)

            # X-axis is determined by Y and Z (gripper opening direction)
            desired_x_axis = np.cross(desired_y_axis, desired_z_axis)
            desired_x_axis = desired_x_axis / np.linalg.norm(desired_x_axis)

            # Create rotation matrix [x, y, z] columns
            R = np.column_stack([desired_x_axis, desired_y_axis, desired_z_axis])

            logger.info(f"Using link_openft frame convention:")
            logger.info(f"  Tool Z-axis (approach): {desired_z_axis}")
            logger.info(f"  Tool Y-axis (upward): {desired_y_axis}")
            logger.info(f"  Tool X-axis (gripper): {desired_x_axis}")
            logger.info(
                f"  Y-axis alignment with world Z: {np.dot(desired_y_axis, world_z_up):.3f}"
            )

        else:
            # For link6 frame (original assumption):
            # Assuming X-axis points out of the end-effector
            desired_tool_direction = -normal_base  # Point towards the handle
            desired_x_axis = desired_tool_direction

            # Choose a reasonable z-axis (perpendicular to x)
            world_z = np.array([0, 0, 1])
            world_y = np.array([0, 1, 0])

            if abs(np.dot(desired_x_axis, world_z)) > 0.9:
                desired_y_axis = np.cross(world_y, desired_x_axis)
            else:
                desired_y_axis = np.cross(world_z, desired_x_axis)

            desired_y_axis = desired_y_axis / np.linalg.norm(desired_y_axis)
            desired_z_axis = np.cross(desired_x_axis, desired_y_axis)

            # Create rotation matrix [x, y, z] columns
            R = np.column_stack([desired_x_axis, desired_y_axis, desired_z_axis])

            logger.info(f"Using link6 frame convention:")
            logger.info(f"  Tool X-axis points towards handle: {desired_x_axis}")

        target_orientation = RotationMatrix(R)

        logger.info(f"Target pose computed:")
        logger.info(f"  Position: {target_position}")
        logger.info(f"  Frame: {self.tool_frame}")

        return target_position, target_orientation

    def solve_ik(self, target_position, target_orientation):
        """
        Solve inverse kinematics for the target pose.

        Args:
            target_position: Target position for tool frame (m)
            target_orientation: Target orientation for tool frame

        Returns:
            q_solution: Joint angles solution or None if failed
        """
        # Create IK problem
        ik = InverseKinematics(self.plant, self.plant_context)

        # Add position constraint for tool frame (link_openft or link6)
        # Use a small tolerance for position
        position_tolerance = 0.005  # 5mm tolerance
        ik.AddPositionConstraint(
            self.tool_frame,
            np.zeros(3),  # Point in tool frame
            self.base_frame,
            target_position - position_tolerance * np.ones(3),
            target_position + position_tolerance * np.ones(3),
        )

        # Add orientation constraint with relaxed tolerance
        # Using more relaxed tolerance for orientation to improve success rate
        orientation_tolerance = 0.1  # ~5.7 degrees tolerance
        ik.AddOrientationConstraint(
            self.tool_frame,
            RotationMatrix(),  # Identity in tool frame
            self.base_frame,
            target_orientation,  # Target orientation in base frame
            orientation_tolerance,
        )

        # Get current joint positions as initial guess
        q_initial = self.plant.GetPositions(self.plant_context)

        # Add joint limit costs to prefer solutions within safe ranges
        prog = ik.get_mutable_prog()

        # Try multiple initial guesses if first one fails
        initial_guesses = [
            q_initial,  # Current position
            np.zeros(self.plant.num_positions()),  # Home position
        ]

        # Add a slightly perturbed version of current position
        q_perturbed = q_initial.copy()
        for i in range(6):
            joint = self.plant.GetJointByName(f"joint{i + 1}")
            idx = joint.position_start()
            q_perturbed[idx] += np.random.uniform(-0.1, 0.1)  # Small random perturbation
        initial_guesses.append(q_perturbed)

        for attempt, q_guess in enumerate(initial_guesses):
            prog.SetInitialGuess(ik.q(), q_guess)

            logger.info(
                f"Solving inverse kinematics (attempt {attempt + 1}/{len(initial_guesses)})..."
            )
            result = Solve(prog)

            if result.is_success():
                q_solution = result.GetSolution(ik.q())

                # Verify the solution is reasonable
                joint_limits_ok = True
                for i in range(6):
                    joint = self.plant.GetJointByName(f"joint{i + 1}")
                    idx = joint.position_start()
                    angle = q_solution[idx]
                    # Check if joint is near limits (within 5 degrees)
                    lower = self.plant.GetPositionLowerLimits()[idx]
                    upper = self.plant.GetPositionUpperLimits()[idx]
                    if angle < lower + np.radians(5) or angle > upper - np.radians(5):
                        logger.warning(f"  Joint {i + 1} near limit: {np.degrees(angle):.1f}°")
                        joint_limits_ok = False

                if joint_limits_ok or attempt == len(initial_guesses) - 1:
                    logger.info(f"IK solution found on attempt {attempt + 1}!")
                    return q_solution
                else:
                    logger.info(f"Solution found but joints near limits, trying next guess...")
            else:
                logger.info(f"  Attempt {attempt + 1} failed: {result.get_solution_result()}")

        logger.warning("IK solution not found after all attempts")
        logger.info(f"  Target position: {target_position}")
        logger.info(f"  Tool frame: {self.tool_frame}")

        # Try one more time with just position constraint (no orientation)
        logger.info("Trying position-only IK...")
        ik_pos_only = InverseKinematics(self.plant, self.plant_context)
        ik_pos_only.AddPositionConstraint(
            self.tool_frame,
            np.zeros(3),
            self.base_frame,
            target_position - 0.01 * np.ones(3),
            target_position + 0.01 * np.ones(3),
        )

        prog_pos = ik_pos_only.get_mutable_prog()
        prog_pos.SetInitialGuess(ik_pos_only.q(), q_initial)
        result_pos = Solve(prog_pos)

        if result_pos.is_success():
            logger.info("Position-only IK succeeded (orientation constraint relaxed)")
            return result_pos.GetSolution(ik_pos_only.q())

        return None

    def visualize_open3d_scene(
        self,
        pcd,
        mesh,
        normal_camera,
        normal_point_camera,
        selected_3d_camera,
        normal_base,
        point_base,
        target_position,
    ):
        """
        Visualize the scene with Open3D interactive viewer.

        Args:
            pcd: Point cloud (Open3D)
            mesh: Reconstructed mesh (Open3D)
            normal_camera: Normal vector in camera frame
            normal_point_camera: Point on mesh in camera frame (mm)
            selected_3d_camera: Selected point in camera frame (mm)
            normal_base: Normal vector in base frame
            point_base: Point in base frame (m)
            target_position: Target position in base frame (m)
        """
        import open3d as o3d

        geometries = []

        # 1. Point cloud (in blue)
        pcd_vis = o3d.geometry.PointCloud()
        pcd_vis.points = pcd.points
        pcd_vis.paint_uniform_color([0.3, 0.3, 1.0])
        geometries.append(pcd_vis)

        # 2. Mesh (semi-transparent)
        mesh_vis = o3d.geometry.TriangleMesh()
        mesh_vis.vertices = mesh.vertices
        mesh_vis.triangles = mesh.triangles
        mesh_vis.compute_vertex_normals()
        mesh_vis.paint_uniform_color([0.7, 0.7, 0.7])
        geometries.append(mesh_vis)

        # 3. Normal vector in camera frame as arrow (green)
        if normal_camera is not None and normal_point_camera is not None:
            # Create arrow for normal
            arrow_length = 150  # 150mm
            arrow = o3d.geometry.TriangleMesh.create_arrow(
                cylinder_radius=2,
                cone_radius=4,
                cylinder_height=arrow_length * 0.8,
                cone_height=arrow_length * 0.2,
            )

            # Create rotation matrix to align arrow with normal
            z_axis = np.array([0, 0, 1])
            if not np.allclose(normal_camera, z_axis):
                rotation_axis = np.cross(z_axis, normal_camera)
                if np.linalg.norm(rotation_axis) > 1e-6:
                    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                    rotation_angle = np.arccos(np.clip(np.dot(z_axis, normal_camera), -1, 1))
                    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(
                        rotation_axis * rotation_angle
                    )
                    arrow.rotate(rotation_matrix, center=[0, 0, 0])

            arrow.translate(normal_point_camera)
            arrow.paint_uniform_color([0, 1, 0])  # Green arrow
            geometries.append(arrow)

            # Add sphere at normal point (red)
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=5)
            sphere.translate(normal_point_camera)
            sphere.paint_uniform_color([1, 0, 0])
            geometries.append(sphere)

            # Add sphere at originally selected point (yellow)
            sphere_selected = o3d.geometry.TriangleMesh.create_sphere(radius=5)
            sphere_selected.translate(selected_3d_camera)
            sphere_selected.paint_uniform_color([1, 1, 0])
            geometries.append(sphere_selected)

        # 4. Add coordinate frame at origin
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
        geometries.append(coord_frame)

        # 5. Add text labels (as spheres with different colors)
        # Camera frame origin indicator (cyan)
        camera_origin = o3d.geometry.TriangleMesh.create_sphere(radius=10)
        camera_origin.translate([0, 0, 0])
        camera_origin.paint_uniform_color([0, 1, 1])
        geometries.append(camera_origin)

        logger.info("Open3D Visualization Legend:")
        logger.info("  Blue points: Segmented point cloud")
        logger.info("  Gray mesh: Reconstructed surface")
        logger.info("  Green arrow: Normal vector at handle")
        logger.info("  Red sphere: Closest mesh point to selection")
        logger.info("  Yellow sphere: Original selected point")
        logger.info("  Cyan sphere: Camera origin")
        logger.info("  RGB axes: Camera coordinate frame")

        # Launch interactive visualization
        try:
            logger.info("Launching Open3D interactive visualization...")
            logger.info("Controls: Mouse to rotate, scroll to zoom, Ctrl+mouse to pan")
            o3d.visualization.draw_geometries(
                geometries,
                window_name="Normal Vector Estimation - Camera Frame",
                width=1024,
                height=768,
            )
        except Exception as e:
            logger.warning(f"Open3D visualization failed: {e}")

        return geometries

    def visualize_drake_scene(
        self, point_base, normal_base, target_position, target_orientation, q_solution=None
    ):
        """
        Visualize the scene in Drake with mesh, normal, and IK solution.

        Args:
            point_base: Handle point in base frame (m)
            normal_base: Handle normal in base frame (unit vector)
            target_position: Target position for link6 (m)
            target_orientation: Target orientation for link6
            q_solution: IK solution joint angles (optional)
        """
        # Clear previous visualizations
        self.meshcat.Delete("handle_point")
        self.meshcat.Delete("normal_vector")
        self.meshcat.Delete("target_point")
        self.meshcat.Delete("approach_line")
        self.meshcat.Delete("base_frame_axes")

        # Add base frame axes for reference
        axis_length = 0.3
        axis_radius = 0.003

        # X-axis (red)
        self.meshcat.SetObject(
            "base_frame_axes/x", Cylinder(axis_radius, axis_length), Rgba(1, 0, 0, 0.6)
        )
        x_rot = RotationMatrix.MakeYRotation(np.pi / 2)
        self.meshcat.SetTransform(
            "base_frame_axes/x", RigidTransform(x_rot, [axis_length / 2, 0, 0])
        )

        # Y-axis (green)
        self.meshcat.SetObject(
            "base_frame_axes/y", Cylinder(axis_radius, axis_length), Rgba(0, 1, 0, 0.6)
        )
        y_rot = RotationMatrix.MakeXRotation(-np.pi / 2)
        self.meshcat.SetTransform(
            "base_frame_axes/y", RigidTransform(y_rot, [0, axis_length / 2, 0])
        )

        # Z-axis (blue)
        self.meshcat.SetObject(
            "base_frame_axes/z", Cylinder(axis_radius, axis_length), Rgba(0, 0, 1, 0.6)
        )
        self.meshcat.SetTransform(
            "base_frame_axes/z", RigidTransform(RotationMatrix(), [0, 0, axis_length / 2])
        )

        # Visualize handle point (red sphere)
        self.meshcat.SetObject("handle_point", Sphere(0.01), Rgba(1.0, 0.0, 0.0, 0.8))
        self.meshcat.SetTransform("handle_point", RigidTransform(RotationMatrix(), point_base))

        # Visualize normal vector (green arrow) - make it bigger for visibility
        arrow_length = 0.3  # Increased from 0.2
        arrow_radius = 0.008  # Increased from 0.005
        self.meshcat.SetObject(
            "normal_vector/shaft", Cylinder(arrow_radius, arrow_length), Rgba(0.0, 1.0, 0.0, 0.8)
        )
        self.meshcat.SetObject(
            "normal_vector/head", Sphere(arrow_radius * 3), Rgba(0.0, 1.0, 0.0, 0.9)
        )

        # Position normal arrow starting from handle point
        arrow_center = point_base + normal_base * arrow_length / 2

        # Create rotation to align cylinder with normal
        z_axis = np.array([0, 0, 1])
        if not np.allclose(normal_base, z_axis) and not np.allclose(normal_base, -z_axis):
            axis = np.cross(z_axis, normal_base)
            if np.linalg.norm(axis) > 0.001:
                axis = axis / np.linalg.norm(axis)
                angle = np.arccos(np.clip(np.dot(z_axis, normal_base), -1, 1))
                K = np.array(
                    [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
                )
                R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
                rotation = RotationMatrix(R)
            else:
                rotation = RotationMatrix()
        elif np.allclose(normal_base, -z_axis):
            # Normal points down, rotate 180 degrees around x-axis
            rotation = RotationMatrix.MakeXRotation(np.pi)
        else:
            rotation = RotationMatrix()

        self.meshcat.SetTransform("normal_vector/shaft", RigidTransform(rotation, arrow_center))
        self.meshcat.SetTransform(
            "normal_vector/head",
            RigidTransform(RotationMatrix(), point_base + normal_base * arrow_length),
        )

        # Visualize approach line from handle to target
        line_radius = 0.002
        approach_distance = np.linalg.norm(target_position - point_base)
        self.meshcat.SetObject(
            "approach_line", Cylinder(line_radius, approach_distance), Rgba(0.5, 0.5, 0.5, 0.5)
        )
        line_center = (point_base + target_position) / 2

        # Align line with approach direction
        approach_dir = (target_position - point_base) / approach_distance
        if not np.allclose(approach_dir, z_axis) and not np.allclose(approach_dir, -z_axis):
            axis = np.cross(z_axis, approach_dir)
            if np.linalg.norm(axis) > 0.001:
                axis = axis / np.linalg.norm(axis)
                angle = np.arccos(np.clip(np.dot(z_axis, approach_dir), -1, 1))
                K = np.array(
                    [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
                )
                R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
                line_rotation = RotationMatrix(R)
            else:
                line_rotation = RotationMatrix()
        else:
            line_rotation = RotationMatrix()

        self.meshcat.SetTransform("approach_line", RigidTransform(line_rotation, line_center))

        # Visualize target point (blue sphere)
        self.meshcat.SetObject("target_point", Sphere(0.015), Rgba(0.0, 0.0, 1.0, 0.8))
        self.meshcat.SetTransform("target_point", RigidTransform(RotationMatrix(), target_position))

        # Visualize target frame axes
        frame_axis_length = 0.15
        frame_axis_radius = 0.004

        # Target X-axis (red) - should point towards handle
        target_x = target_orientation.matrix()[:, 0]
        self.meshcat.SetObject(
            "target_frame/x",
            Cylinder(frame_axis_radius, frame_axis_length),
            Rgba(0.8, 0.2, 0.2, 0.9),
        )
        x_center = target_position + target_x * frame_axis_length / 2
        x_rot = self._align_cylinder_with_axis(target_x)
        self.meshcat.SetTransform("target_frame/x", RigidTransform(x_rot, x_center))

        # Target Y-axis (green)
        target_y = target_orientation.matrix()[:, 1]
        self.meshcat.SetObject(
            "target_frame/y",
            Cylinder(frame_axis_radius, frame_axis_length),
            Rgba(0.2, 0.8, 0.2, 0.9),
        )
        y_center = target_position + target_y * frame_axis_length / 2
        y_rot = self._align_cylinder_with_axis(target_y)
        self.meshcat.SetTransform("target_frame/y", RigidTransform(y_rot, y_center))

        # Target Z-axis (blue)
        target_z = target_orientation.matrix()[:, 2]
        self.meshcat.SetObject(
            "target_frame/z",
            Cylinder(frame_axis_radius, frame_axis_length),
            Rgba(0.2, 0.2, 0.8, 0.9),
        )
        z_center = target_position + target_z * frame_axis_length / 2
        z_rot = self._align_cylinder_with_axis(target_z)
        self.meshcat.SetTransform("target_frame/z", RigidTransform(z_rot, z_center))

        # If we have an IK solution, apply it and visualize link6 frame
        if q_solution is not None:
            self.plant.SetPositions(self.plant_context, q_solution)
            logger.info("Applied IK solution to robot")

            # Log joint angles
            joint_names = [f"joint{i + 1}" for i in range(6)]
            for i, name in enumerate(joint_names):
                if i < len(q_solution):
                    logger.info(f"  {name}: {np.degrees(q_solution[i]):.2f} deg")

            # Get actual link6 pose after IK
            link6_pose = self.plant.CalcRelativeTransform(
                self.plant_context, self.base_frame, self.link6_frame
            )
            link6_pos = link6_pose.translation()
            link6_rot = link6_pose.rotation()

            # Visualize link6 frame axes
            link6_axis_length = 0.12
            link6_axis_radius = 0.003

            # Link6 X-axis (red) - this should be the tool axis
            link6_x = link6_rot.matrix()[:, 0]
            self.meshcat.SetObject(
                "link6_frame/x",
                Cylinder(link6_axis_radius, link6_axis_length),
                Rgba(1.0, 0.0, 0.0, 0.7),
            )
            link6_x_center = link6_pos + link6_x * link6_axis_length / 2
            link6_x_rot = self._align_cylinder_with_axis(link6_x)
            self.meshcat.SetTransform("link6_frame/x", RigidTransform(link6_x_rot, link6_x_center))

            # Link6 Y-axis (green)
            link6_y = link6_rot.matrix()[:, 1]
            self.meshcat.SetObject(
                "link6_frame/y",
                Cylinder(link6_axis_radius, link6_axis_length),
                Rgba(0.0, 1.0, 0.0, 0.7),
            )
            link6_y_center = link6_pos + link6_y * link6_axis_length / 2
            link6_y_rot = self._align_cylinder_with_axis(link6_y)
            self.meshcat.SetTransform("link6_frame/y", RigidTransform(link6_y_rot, link6_y_center))

            # Link6 Z-axis (blue)
            link6_z = link6_rot.matrix()[:, 2]
            self.meshcat.SetObject(
                "link6_frame/z",
                Cylinder(link6_axis_radius, link6_axis_length),
                Rgba(0.0, 0.0, 1.0, 0.7),
            )
            link6_z_center = link6_pos + link6_z * link6_axis_length / 2
            link6_z_rot = self._align_cylinder_with_axis(link6_z)
            self.meshcat.SetTransform("link6_frame/z", RigidTransform(link6_z_rot, link6_z_center))

            logger.info(f"\nLink6 actual pose:")
            logger.info(f"  Position: {link6_pos}")
            logger.info(f"  X-axis (tool): {link6_x}")
            logger.info(f"  Y-axis: {link6_y}")
            logger.info(f"  Z-axis: {link6_z}")

            # Check alignment
            x_alignment = np.dot(link6_x, target_orientation.matrix()[:, 0])
            logger.info(f"\nAlignment check:")
            logger.info(f"  X-axis alignment: {x_alignment:.3f} (should be ~1.0)")
            logger.info(
                f"  Position error: {np.linalg.norm(link6_pos - target_position) * 1000:.1f} mm"
            )

        # Log visualization info
        logger.info("\nDrake Visualization Legend:")
        logger.info("  Red sphere: Handle point in base frame")
        logger.info("  Green arrow: Normal vector (pointing away from surface)")
        logger.info("  Blue sphere: Target position for end-effector")
        logger.info("  Gray line: Approach path")
        logger.info("  RGB cylinders: Base frame axes (X=red, Y=green, Z=blue)")
        logger.info("  Target frame axes: Semi-transparent RGB (desired orientation)")
        logger.info("  Link6 frame axes: Solid RGB (actual orientation after IK)")

        # Update visualization
        self.diagram.ForcedPublish(self.diagram_context)
        logger.info("Drake visualization updated")

    def _align_cylinder_with_axis(self, axis):
        """Helper function to create rotation matrix to align cylinder with given axis"""
        z_axis = np.array([0, 0, 1])
        if np.allclose(axis, z_axis) or np.allclose(axis, -z_axis):
            if axis[2] < 0:
                return RotationMatrix.MakeXRotation(np.pi)
            else:
                return RotationMatrix()
        else:
            rotation_axis = np.cross(z_axis, axis)
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            angle = np.arccos(np.clip(np.dot(z_axis, axis), -1, 1))
            K = np.array(
                [
                    [0, -rotation_axis[2], rotation_axis[1]],
                    [rotation_axis[2], 0, -rotation_axis[0]],
                    [-rotation_axis[1], rotation_axis[0], 0],
                ]
            )
            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
            return RotationMatrix(R)

    def create_results_figure(
        self,
        rgb_image,
        mask,
        selected_2d,
        normal_camera,
        point_camera,
        normal_base,
        point_base,
        target_position,
    ):
        """Create a comprehensive figure with all results"""
        fig = plt.figure(figsize=(20, 12))

        # 1. Original RGB with selected point
        ax1 = plt.subplot(2, 4, 1)
        ax1.imshow(rgb_image)
        ax1.scatter(selected_2d[0], selected_2d[1], c="red", s=100, marker="x", linewidths=2)
        ax1.set_title(f"Selected Point: ({selected_2d[0]}, {selected_2d[1]})")
        ax1.axis("off")

        # 2. Segmentation mask
        ax2 = plt.subplot(2, 4, 2)
        ax2.imshow(mask, cmap="gray")
        ax2.set_title(f"Segmentation Mask")
        ax2.axis("off")

        # 3. Normal in camera frame
        ax3 = plt.subplot(2, 4, 3, projection="3d")
        ax3.quiver(0, 0, 0, 50, 0, 0, color="r", arrow_length_ratio=0.1, label="X")
        ax3.quiver(0, 0, 0, 0, 50, 0, color="g", arrow_length_ratio=0.1, label="Y")
        ax3.quiver(0, 0, 0, 0, 0, 50, color="b", arrow_length_ratio=0.1, label="Z")

        normal_scaled = normal_camera * 100
        ax3.quiver(
            0,
            0,
            0,
            normal_scaled[0],
            normal_scaled[1],
            normal_scaled[2],
            color="purple",
            arrow_length_ratio=0.1,
            linewidth=3,
            label="Normal (Camera)",
        )

        ax3.set_xlim([-100, 100])
        ax3.set_ylim([-100, 100])
        ax3.set_zlim([-100, 100])
        ax3.set_xlabel("X (mm)")
        ax3.set_ylabel("Y (mm)")
        ax3.set_zlabel("Z (mm)")
        ax3.legend()
        ax3.set_title("Normal in Camera Frame")

        # 4. Normal in base frame
        ax4 = plt.subplot(2, 4, 4, projection="3d")
        ax4.quiver(0, 0, 0, 0.2, 0, 0, color="r", arrow_length_ratio=0.1, label="X")
        ax4.quiver(0, 0, 0, 0, 0.2, 0, color="g", arrow_length_ratio=0.1, label="Y")
        ax4.quiver(0, 0, 0, 0, 0, 0.2, color="b", arrow_length_ratio=0.1, label="Z")

        normal_base_scaled = normal_base * 0.3
        ax4.quiver(
            0,
            0,
            0,
            normal_base_scaled[0],
            normal_base_scaled[1],
            normal_base_scaled[2],
            color="purple",
            arrow_length_ratio=0.1,
            linewidth=3,
            label="Normal (Base)",
        )

        ax4.set_xlim([-0.3, 0.3])
        ax4.set_ylim([-0.3, 0.3])
        ax4.set_zlim([-0.3, 0.3])
        ax4.set_xlabel("X (m)")
        ax4.set_ylabel("Y (m)")
        ax4.set_zlabel("Z (m)")
        ax4.legend()
        ax4.set_title("Normal in Base Frame")

        # 5. Target position visualization
        ax5 = plt.subplot(2, 4, 5, projection="3d")

        # Plot handle point
        ax5.scatter(
            point_base[0],
            point_base[1],
            point_base[2],
            c="red",
            s=100,
            marker="o",
            label="Handle Point",
        )

        # Plot target position
        ax5.scatter(
            target_position[0],
            target_position[1],
            target_position[2],
            c="blue",
            s=100,
            marker="^",
            label="Target Position",
        )

        # Draw line connecting them
        ax5.plot(
            [point_base[0], target_position[0]],
            [point_base[1], target_position[1]],
            [point_base[2], target_position[2]],
            "g--",
            alpha=0.5,
        )

        # Draw normal vector
        ax5.quiver(
            point_base[0],
            point_base[1],
            point_base[2],
            normal_base[0] * 0.3,
            normal_base[1] * 0.3,
            normal_base[2] * 0.3,
            color="green",
            arrow_length_ratio=0.1,
            linewidth=2,
            label="Normal",
        )

        ax5.set_xlabel("X (m)")
        ax5.set_ylabel("Y (m)")
        ax5.set_zlabel("Z (m)")
        ax5.legend()
        ax5.set_title("Target Planning in Base Frame")

        # 6. Information panel
        ax6 = plt.subplot(2, 4, 6)
        ax6.axis("off")
        info_text = "Transformation Results\n" + "=" * 30 + "\n\n"
        info_text += f"Selected Point (Camera):\n"
        info_text += f"  X: {point_camera[0]:.1f} mm\n"
        info_text += f"  Y: {point_camera[1]:.1f} mm\n"
        info_text += f"  Z: {point_camera[2]:.1f} mm\n\n"
        info_text += f"Normal (Camera):\n"
        info_text += (
            f"  [{normal_camera[0]:.3f}, {normal_camera[1]:.3f}, {normal_camera[2]:.3f}]\n\n"
        )
        info_text += f"Point (Base):\n"
        info_text += f"  X: {point_base[0]:.3f} m\n"
        info_text += f"  Y: {point_base[1]:.3f} m\n"
        info_text += f"  Z: {point_base[2]:.3f} m\n\n"
        info_text += f"Normal (Base):\n"
        info_text += f"  [{normal_base[0]:.3f}, {normal_base[1]:.3f}, {normal_base[2]:.3f}]\n\n"
        info_text += f"Target Position:\n"
        info_text += f"  X: {target_position[0]:.3f} m\n"
        info_text += f"  Y: {target_position[1]:.3f} m\n"
        info_text += f"  Z: {target_position[2]:.3f} m\n"

        ax6.text(
            0.1,
            0.9,
            info_text,
            transform=ax6.transAxes,
            fontsize=10,
            verticalalignment="top",
            fontfamily="monospace",
        )
        ax6.set_title("Transformation Info")

        plt.suptitle("Normal-based IK Motion Planning", fontsize=16, fontweight="bold")
        plt.tight_layout()

        # Save figure
        output_path = "normal_move_results.png"
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        logger.info(f"Saved results figure to {output_path}")
        plt.close()

        return output_path

    def single_iteration(self, iteration_num):
        """Execute a single iteration of detection and movement.

        Args:
            iteration_num: Current iteration number (1-indexed)

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            logger.info("\n" + "=" * 60)
            logger.info(f"Iteration {iteration_num}/{self.loop_count}")
            logger.info("=" * 60)
            # 1. Capture frame
            logger.info("Capturing frame from ZED camera...")
            rgb_image, depth_map, point_cloud = self.capture_frame()

            # Save debug image
            cv2.imwrite("captured_frame.jpg", cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))

            # 2. Select point (either using Qwen or manual selection)
            if self.use_qwen:
                logger.info("Using Qwen vision model for automatic handle detection")
                selected_point = self.query_qwen_for_point(rgb_image)

                # Fall back to manual selection if Qwen fails
                if selected_point is None:
                    logger.info("Qwen detection failed, falling back to manual selection")
                    selected_point = self.select_point(rgb_image)
            else:
                selected_point = self.select_point(rgb_image)

            if selected_point is None:
                logger.warning("No point selected")
                return

            logger.info(f"Selected point: {selected_point}")

            # 3. Segment with FastSAM
            mask = self.segment_with_fastsam(rgb_image, selected_point)
            if mask is None:
                logger.error("Segmentation failed")
                return

            # 4. Extract segmented point cloud
            points, colors = self.extract_segmented_pointcloud(point_cloud, mask)
            if points is None:
                logger.error("Failed to extract point cloud")
                return

            # 5. Downsample point cloud
            pcd = self.downsample_pointcloud(points, colors, voxel_size=5.0)

            # 6. Reconstruct mesh
            mesh = self.reconstruct_mesh(pcd)

            # 7. Find normal at selected point
            pc_data = point_cloud.get_data()
            selected_3d = pc_data[selected_point[1], selected_point[0]][:3]
            normal_camera, closest_point = self.find_normal_at_point(mesh, selected_point)

            if normal_camera is None:
                logger.warning("Could not compute normal vector")
                return

            logger.info(
                f"Normal vector (camera): [{normal_camera[0]:.3f}, {normal_camera[1]:.3f}, {normal_camera[2]:.3f}]"
            )

            # 8. Transform to base frame
            point_base, normal_base = self.transform_to_base_frame(closest_point, normal_camera)

            # 9. Compute target pose (25cm away from surface)
            target_position, target_orientation = self.compute_target_pose(
                point_base, normal_base, approach_distance=0.25
            )

            # 10. Solve inverse kinematics
            q_solution = self.solve_ik(target_position, target_orientation)

            # Store successful solution for potential grab
            if q_solution is not None:
                self.last_q_solution = (
                    q_solution.copy() if isinstance(q_solution, np.ndarray) else q_solution
                )
                self.last_target_position = (
                    target_position.copy()
                    if isinstance(target_position, np.ndarray)
                    else target_position
                )
                self.last_target_normal = (
                    normal_base.copy() if isinstance(normal_base, np.ndarray) else normal_base
                )

                # Execute movement if not in test mode and we have a solution
                if self.arm is not None and not self.test_mode:
                    logger.info("\n" + "=" * 60)
                    logger.info("Executing movement to target position")
                    logger.info("=" * 60)
                    self.execute_xarm_movement(q_solution)
                elif self.test_mode:
                    logger.info("\n" + "=" * 60)
                    logger.info("Test mode - NOT executing movement")
                    logger.info("IK solution found but movement disabled")
                    logger.info("=" * 60)
            else:
                logger.warning("No IK solution found for this iteration")
                return False

            # Only visualize on the last iteration or if only one iteration
            if iteration_num == self.loop_count:
                # 11. Visualize in Open3D first (camera frame)
                logger.info("\n" + "=" * 60)
                logger.info("Open3D Visualization (Camera Frame)")
                logger.info("=" * 60)
                self.visualize_open3d_scene(
                    pcd,
                    mesh,
                    normal_camera,
                    closest_point,
                    selected_3d,
                    normal_base,
                    point_base,
                    target_position,
                )

                # 12. Visualize in Drake
                self.visualize_drake_scene(
                    point_base, normal_base, target_position, target_orientation, q_solution
                )

                # 13. Create results figure
                self.create_results_figure(
                    rgb_image,
                    mask,
                    selected_point,
                    normal_camera,
                    closest_point,
                    normal_base,
                    point_base,
                    target_position,
                )

                # 14. Save mesh for reference
                o3d.io.write_triangle_mesh(f"reconstructed_mesh_iter{iteration_num}.ply", mesh)
                logger.info(f"Saved mesh to reconstructed_mesh_iter{iteration_num}.ply")

            logger.info(f"\nIteration {iteration_num} complete!")
            return True

        except Exception as e:
            logger.error(f"Iteration {iteration_num} failed: {e}")
            return False

    def run(self):
        """Main execution pipeline with looping and grab support."""
        try:
            successful_iterations = 0

            # Execute iterations
            for i in range(1, self.loop_count + 1):
                success = self.single_iteration(i)
                if success:
                    successful_iterations += 1

                # Wait between iterations if not the last one
                if i < self.loop_count:
                    wait_time = 2
                    logger.info(f"\nWaiting {wait_time} seconds before next iteration...")
                    time.sleep(wait_time)

            logger.info("\n" + "=" * 60)
            logger.info(
                f"Completed {successful_iterations}/{self.loop_count} iterations successfully"
            )
            logger.info("=" * 60)

            # Execute grab sequence if requested and we had at least one success
            if self.execute_grab and successful_iterations > 0:
                logger.info("\nExecuting grab sequence...")
                grab_success = self.execute_grab_sequence()
                if grab_success:
                    logger.info("Grab sequence completed successfully!")
                else:
                    logger.warning("Grab sequence failed or incomplete")
            elif self.execute_grab and successful_iterations == 0:
                logger.warning("Cannot execute grab sequence - no successful iterations")

            logger.info("\n" + "=" * 60)
            logger.info("Processing complete!")
            logger.info("Check Meshcat for Drake visualization")
            logger.info("=" * 60)

            # Keep visualization running
            logger.info("Press Ctrl+C to exit...")
            while True:
                time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info("\nExiting...")
        except Exception as e:
            logger.error(f"Processing failed: {e}")
            raise

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
                    logger.info(f"  joint{i + 1}: {angle:.4f} rad ({np.degrees(angle):.2f} deg)")

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
        """Execute the IK solution on the real xARM robot.

        Args:
            q_solution: Joint angles from IK solution
            speed: Movement speed (default 15, use 7.5 for slower movements)
        """
        if not self.arm:
            logger.warning("xARM API not initialized, cannot execute movement")
            return

        try:
            # Extract joint angles for xARM (first 6 joints)
            arm_joint_names = [f"joint{i + 1}" for i in range(6)]
            positions = []

            for joint_name in arm_joint_names:
                joint = self.plant.GetJointByName(joint_name)
                joint_idx = joint.position_start()
                positions.append(q_solution[joint_idx])

            logger.info("Sending joint angles to xARM:")
            for i, angle in enumerate(positions):
                logger.info(f"  joint{i + 1}: {np.degrees(angle):.2f} deg")

            # Send command to xARM with specified speed
            code = self.arm.set_servo_angle(angle=positions, speed=speed, wait=True, is_radian=True)

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

    def execute_grab_sequence(self):
        """Execute the grab sequence: move up, forward, and close gripper.

        This method:
        1. Moves up by the Z offset that was applied (10cm)
        2. Moves forward along the approach direction
        3. Closes the gripper
        """
        if (
            self.last_q_solution is None
            or self.last_target_position is None
            or self.last_target_normal is None
        ):
            logger.warning("No successful positioning found, cannot execute grab sequence")
            return False

        if self.test_mode:
            logger.info("\n" + "=" * 60)
            logger.info("Test mode - Would execute grab sequence:")
            logger.info(f"  1. Move up {self.z_offset_applied * 100:.0f}cm (reverse Z offset)")
            logger.info(
                f"  2. Move forward {forward_distance * 100:.0f}cm along approach direction"
            )
            logger.info(f"  3. Close gripper quickly")
            logger.info("=" * 60)
            return True

        if not self.arm:
            logger.warning("xARM API not initialized, cannot execute grab sequence")
            return False

        try:
            logger.info("\n" + "=" * 60)
            logger.info("Executing grab sequence")
            logger.info("=" * 60)

            # Step 1: Move up by reversing the Z offset
            logger.info(
                f"Step 1: Moving up {self.z_offset_applied * 100:.0f}cm to reverse Z offset"
            )

            # Calculate new target position (move up in world Z)
            target_pos_up = np.array(self.last_target_position, copy=True)
            target_pos_up[2] += self.z_offset_applied  # Move up by the offset amount

            # Solve IK for upward movement
            logger.info(f"  Target position after moving up: {target_pos_up}")

            # We keep the same orientation, just change position
            # Get the last orientation from the plant context
            self.plant.SetPositions(self.plant_context, self.last_q_solution)
            tool_pose = self.plant.CalcRelativeTransform(
                self.plant_context, self.base_frame, self.tool_frame
            )
            target_orientation = tool_pose.rotation()

            # Solve IK for the new position
            q_up = self.solve_ik(target_pos_up, target_orientation)

            if q_up is not None:
                # Execute upward movement at half default speed (7.5 instead of 15)
                self.execute_xarm_movement(q_up, speed=7.5)
                time.sleep(1)  # Wait for movement to complete
                logger.info("  Upward movement completed")
            else:
                logger.warning("  Failed to solve IK for upward movement")

            # Step 2: Move forward 9cm along approach direction
            forward_distance = 0.09  # 9cm
            logger.info(
                f"Step 2: Moving forward {forward_distance * 100:.0f}cm along approach direction"
            )

            # The approach direction is opposite to the normal (we approach towards the surface)
            approach_direction = -np.array(self.last_target_normal)
            target_pos_forward = target_pos_up + approach_direction * forward_distance

            logger.info(f"  Target position after moving forward: {target_pos_forward}")

            # Solve IK for forward movement
            q_forward = self.solve_ik(target_pos_forward, target_orientation)

            if q_forward is not None:
                # Execute forward movement at half default speed (7.5 instead of 15)
                self.execute_xarm_movement(q_forward, speed=7.5)
                time.sleep(1)  # Wait for movement to complete
                logger.info("  Forward movement completed")

                # Update plant context for final position
                self.plant.SetPositions(self.plant_context, q_forward)
            else:
                logger.warning(
                    "  Failed to solve IK for forward movement, continuing with gripper close"
                )

            # Step 3: Close gripper
            logger.info("Step 3: Closing gripper")

            # Close the gripper with faster speed
            code = self.arm.set_gripper_position(
                0, wait=True, speed=5000
            )  # 0 = fully closed, speed increased from 500 to 5000

            if code == 0:
                logger.info("  Gripper closed successfully")
                time.sleep(0.2)  # Reduced pause from 0.5 to 0.2 seconds
            else:
                logger.error(f"  Failed to close gripper, code: {code}")

            logger.info("\n" + "=" * 60)
            logger.info("Grab sequence completed!")
            logger.info("=" * 60)

            return True

        except Exception as e:
            logger.error(f"Error executing grab sequence: {e}")
            return False

    def cleanup(self):
        """Clean up resources"""
        if self.zed:
            self.zed.close()
            logger.info("ZED camera closed")

        if self.arm:
            self.arm.disconnect()
            logger.info("xARM connection closed")


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="Normal-based IK motion planning")
    parser.add_argument(
        "--fastsam-model",
        type=str,
        default="./weights/FastSAM-x.pt",
        help="Path to FastSAM model weights",
    )
    parser.add_argument(
        "--xarm", type=str, default=None, help="xARM IP address (e.g., 192.168.1.100)"
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Test mode: get xARM positions but do not execute movements",
    )
    parser.add_argument(
        "--qwen",
        action="store_true",
        help="Use Qwen vision model to automatically detect handle point instead of manual selection",
    )
    parser.add_argument(
        "--loop",
        type=int,
        default=1,
        help="Number of times to repeat the detection and movement cycle (default: 1)",
    )
    parser.add_argument(
        "--grab",
        action="store_true",
        help="Execute grab sequence after positioning: move up, forward, and close gripper",
    )

    args = parser.parse_args()

    # Create and run test
    test = NormalMoveTest(
        fastsam_model_path=args.fastsam_model,
        xarm_ip=args.xarm,
        test_mode=args.test,
        use_qwen=args.qwen,
        loop_count=args.loop,
        execute_grab=args.grab,
    )

    try:
        test.run()
    finally:
        test.cleanup()


if __name__ == "__main__":
    main()
