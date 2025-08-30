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

import numpy as np
import cv2
import open3d as o3d
from typing import Optional, Dict, Any
import logging
import time
import threading
from reactivex import interval, timer

from dimos.hardware.zed_camera import ZEDCamera

try:
    import pyzed.sl as sl
except ImportError:
    sl = None
    logging.warning("ZED SDK not found. Please install pyzed to use ZED camera functionality.")

from dimos.core import Module, Out, rpc
from dimos.utils.logging_config import setup_logger
from dimos.protocol.tf import TF
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion

# Import LCM message types
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.std_msgs import Header
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage

logger = setup_logger(__name__)


class SpatialMapPublishThread(threading.Thread):
    """Thread to periodically request and publish spatial map from ZED camera."""

    def __init__(self, zed_module, zed_camera=None, publish_interval=0.5, voxel_size=0.5):
        super().__init__()
        self.daemon = True
        self.zed_module = zed_module
        self.zed_camera = zed_camera
        self.publish_interval = publish_interval
        self._stop_event = threading.Event()
        self.last_pointcloud = None
        self.last_capture_time = 0.0
        self.capture_lock = threading.Lock()

    def run(self):
        logger.info(f"SpatialMapPublishThread started with {self.publish_interval}s interval")

        while not self._stop_event.is_set():
            start_time = time.time()

            try:
                # Only process if camera is available and mapping is enabled
                if (
                    self.zed_camera
                    and self.zed_module.spatial_mapping_enabled
                    and self.zed_camera.mapping_enabled
                ):
                    # Capture pointcloud (spatial map)
                    pcd = self._capture_spatial_map()

                    if pcd is not None and len(pcd.points) > 0:
                        # Store the captured pointcloud
                        with self.capture_lock:
                            self.last_pointcloud = pcd
                            self.last_capture_time = time.time()

                        # Publish the pointcloud
                        self._publish_spatial_map(pcd)

                        logger.debug(
                            f"Captured and published spatial map with {len(pcd.points)} points"
                        )

            except Exception as e:
                logger.error(f"Error in spatial map thread: {e}")

            # Calculate processing time and adjust sleep
            processing_time = time.time() - start_time
            sleep_time = max(0.01, self.publish_interval - processing_time)  # Min 10ms sleep

            # Sleep with early exit check
            if self._stop_event.wait(sleep_time):
                break

        logger.info("SpatialMapPublishThread stopped")

    def _capture_spatial_map(self):
        """Capture the spatial map from ZED camera."""
        try:
            self.zed_camera.zed.request_spatial_map_async()

            max_wait = 5  # seconds
            wait_start = time.time()

            while (time.time() - wait_start) < max_wait:
                if (
                    self.zed_camera.zed.get_spatial_map_request_status_async()
                    == sl.ERROR_CODE.SUCCESS
                ):
                    self.zed_camera.zed.retrieve_spatial_map_async(self.zed_camera.fused_pointcloud)
                    self.zed_camera.zed.extract_whole_spatial_map(self.zed_camera.fused_pointcloud)
                    self.zed_camera.fused_pointcloud.update_from_chunklist()

                    vertices = self.zed_camera.fused_pointcloud.vertices
                    if len(vertices) > 0:
                        points = np.array(vertices, dtype=np.float32).reshape(-1, 4)[
                            :, :3
                        ]  # XYZ only

                        valid = np.isfinite(points).all(axis=1)
                        valid_points = points[valid]

                        pcd = o3d.geometry.PointCloud()
                        if len(valid_points) > 0:
                            pcd.points = o3d.utility.Vector3dVector(valid_points)

                            pcd = pcd.voxel_down_sample(voxel_size=0.5)
                            return pcd
                    break

                time.sleep(0.01)

            return None

        except Exception as e:
            logger.error(f"Error capturing spatial map: {e}")
            return None

    def _publish_spatial_map(self, pcd):
        """Publish the spatial map as a LidarMessage."""
        try:
            # Create header with world frame (spatial map is in world coordinates)
            header = Header("world")

            # Create LidarMessage
            lidar_msg = LidarMessage(
                pointcloud=pcd,
                origin=[0.0, 0.0, 0.0],
                resolution=0.5,  # Match Map module voxel_size
                ts=header.ts,
                frame_id="world",
            )

            # Publish if output is available
            if self.zed_module.pointcloud_msg:
                self.zed_module.pointcloud_msg.publish(lidar_msg)
                logger.debug(f"Published spatial map with {len(pcd.points)} points")

        except Exception as e:
            logger.error(f"Error publishing spatial map: {e}")

    def get_latest_pointcloud(self):
        """Get the latest captured pointcloud (thread-safe)."""
        with self.capture_lock:
            return self.last_pointcloud, self.last_capture_time

    def stop(self):
        self._stop_event.set()


class ZEDModule(Module):
    """
    Dask module for ZED camera that publishes sensor data via LCM.

    Publishes:
        - /zed/color_image: RGB camera images
        - /zed/depth_image: Depth images
        - /zed/camera_info: Camera calibration information
        - /zed/pose: Camera pose (if tracking enabled)
    """

    # Define LCM outputs
    color_image: Out[Image] = None
    depth_image: Out[Image] = None
    camera_info: Out[CameraInfo] = None
    pose: Out[PoseStamped] = None
    pointcloud_msg: Out[LidarMessage] = None  # Pointcloud output for mapping

    def __init__(
        self,
        camera_id: int = 0,
        resolution: str = "HD720",
        depth_mode: str = "NEURAL",
        fps: int = 30,
        enable_tracking: bool = True,
        enable_imu_fusion: bool = True,
        set_floor_as_origin: bool = True,
        enable_spatial_mapping: bool = False,  # Enable ZED SDK spatial mapping
        mapping_resolution: str = "MEDIUM",  # Spatial mapping resolution
        mapping_range: str = "MEDIUM",  # Spatial mapping range
        publish_rate: float = 30.0,
        frame_id: str = "camera_link",  # ZED outputs in ROS frame (z-up, x-forward)
        filter_voxel_size: float = 0.5,
        filter_max_distance: float = 5.0,
        filter_min_distance: float = 0.1,
        filter_min_z: float = -1.0,
        filter_max_z: float = 2.0,
        filter_ground_threshold: float = -0.45,
        filter_target_points: int = 50000,
        **kwargs,
    ):
        """
        Initialize ZED Module.

        Args:
            camera_id: Camera ID (0 for first ZED)
            resolution: Resolution string ("HD720", "HD1080", "HD2K", "VGA")
            depth_mode: Depth mode string ("NEURAL", "ULTRA", "QUALITY", "PERFORMANCE")
            fps: Camera frame rate
            enable_tracking: Enable positional tracking
            enable_imu_fusion: Enable IMU fusion for tracking
            set_floor_as_origin: Set floor as origin for tracking
            publish_rate: Rate to publish messages (Hz)
            frame_id: TF frame ID for messages
        """
        super().__init__(**kwargs)

        self.camera_id = camera_id
        self.fps = fps
        self.enable_tracking = enable_tracking
        self.enable_imu_fusion = enable_imu_fusion
        self.set_floor_as_origin = set_floor_as_origin
        self.enable_spatial_mapping = enable_spatial_mapping
        self.spatial_mapping_enabled = False
        self.publish_rate = publish_rate
        self.frame_id = frame_id

        self.filter_voxel_size = filter_voxel_size
        self.filter_max_distance = filter_max_distance
        self.filter_min_distance = filter_min_distance
        self.filter_min_z = filter_min_z
        self.filter_max_z = filter_max_z
        self.filter_ground_threshold = filter_ground_threshold
        self.filter_target_points = filter_target_points

        # Convert string parameters to ZED enums
        self.resolution = getattr(sl.RESOLUTION, resolution, sl.RESOLUTION.HD720)
        self.depth_mode = getattr(sl.DEPTH_MODE, depth_mode, sl.DEPTH_MODE.NEURAL)
        self.mapping_resolution = getattr(
            sl.MAPPING_RESOLUTION, mapping_resolution, sl.MAPPING_RESOLUTION.MEDIUM
        )
        self.mapping_range = getattr(sl.MAPPING_RANGE, mapping_range, sl.MAPPING_RANGE.MEDIUM)

        # Internal state
        self.zed_camera = None
        self._running = False
        self._subscription = None
        self._sequence = 0
        self.spatial_map_publish_thread = None  # Thread for handling spatial map publishing

        # Initialize TF publisher
        self.tf = TF()

        logger.info(f"ZEDModule initialized for camera {camera_id}")

    @rpc
    def start(self):
        """Start the ZED module and begin publishing data."""
        if self._running:
            logger.warning("ZED module already running")
            return

        try:
            # Initialize ZED camera
            self.zed_camera = ZEDCamera(
                camera_id=self.camera_id,
                resolution=self.resolution,
                depth_mode=self.depth_mode,
                fps=self.fps,
            )

            # Open camera
            if not self.zed_camera.open():
                logger.error("Failed to open ZED camera")
                return

            # Enable positional tracking if requested
            if self.enable_tracking:
                success = self.zed_camera.enable_positional_tracking(
                    enable_imu_fusion=self.enable_imu_fusion,
                    set_floor_as_origin=self.set_floor_as_origin,
                    enable_pose_smoothing=True,
                    enable_area_memory=True,
                )
                if not success:
                    logger.warning("Failed to enable positional tracking")
                    self.enable_tracking = False
                else:
                    logger.info("Positional tracking enabled")

            # Start running immediately
            self._running = True

            # Publish camera info once at startup
            self._publish_camera_info()

            # Start periodic frame capture and publishing
            publish_interval = 1.0 / self.publish_rate
            self._subscription = interval(publish_interval).subscribe(
                lambda _: self._capture_and_publish()
            )

            logger.info(f"ZED module started, publishing at {self.publish_rate} Hz")

            # Initialize and start the spatial map publish thread
            if self.enable_tracking and self.enable_spatial_mapping:
                self.spatial_map_publish_thread = SpatialMapPublishThread(
                    self,
                    zed_camera=self.zed_camera,
                    publish_interval=0.5,
                    voxel_size=self.filter_voxel_size,
                )

                # Schedule spatial mapping to be enabled after 1 second (one time only)
                self._spatial_mapping_timer = timer(1.0).subscribe(
                    lambda _: self._enable_spatial_mapping_once()
                )

        except Exception as e:
            logger.error(f"Error starting ZED module: {e}")
            self._running = False

    def _enable_spatial_mapping_once(self):
        """Enable spatial mapping once tracking is ready."""
        try:
            if self.spatial_mapping_enabled:
                logger.debug("Spatial mapping already enabled")
                return

            # Check if tracking is ready
            pose_data = self.zed_camera.get_pose()
            tracking_state = pose_data.get("tracking_state", "UNKNOWN") if pose_data else "NO_POSE"
            logger.info(f"[SPATIAL_MAPPING_INIT] Checking tracking state: {tracking_state}")

            if not pose_data or not pose_data.get("valid", False):
                logger.warning(
                    f"[SPATIAL_MAPPING_INIT] Tracking not ready (state: {tracking_state}), scheduling retry in 2 seconds..."
                )
                # Retry in 1 second
                timer(1.0).subscribe(lambda _: self._enable_spatial_mapping_once())
                return

            # Mark tracking as initialized BEFORE enabling spatial mapping
            self._tracking_initialized = True
            logger.info(
                f"[SPATIAL_MAPPING_INIT] Tracking is OK (state: {tracking_state}), enabling spatial mapping..."
            )

            success = self.zed_camera.enable_spatial_mapping(
                resolution=self.mapping_resolution,
                mapping_range=self.mapping_range,
                max_memory_usage=512,  # Already reduced to avoid memory issues
                save_texture=False,
                use_chunk_only=True,
                reverse_vertex_order=False,
            )

            if success:
                logger.info("[SPATIAL_MAPPING_INIT] Spatial mapping enabled successfully")
                self.spatial_mapping_enabled = True
                self._mapping_initialized = True

                # Check tracking state AFTER enabling spatial mapping
                pose_after = self.zed_camera.get_pose()
                state_after = (
                    pose_after.get("tracking_state", "UNKNOWN") if pose_after else "NO_POSE"
                )
                logger.info(f"[SPATIAL_MAPPING_INIT] Tracking state after enabling: {state_after}")

                if not pose_after or not pose_after.get("valid", False):
                    logger.error(
                        f"[SPATIAL_MAPPING_INIT] Tracking lost after enabling spatial mapping! State: {state_after}"
                    )
                    # Try to recover by disabling spatial mapping
                    logger.warning(
                        "[SPATIAL_MAPPING_INIT] Disabling spatial mapping to recover tracking..."
                    )
                    self.zed_camera.disable_spatial_mapping()
                    self.spatial_mapping_enabled = False
                    self._mapping_initialized = False
                else:
                    # Start the spatial map publish thread
                    logger.info("[SPATIAL_MAPPING_INIT] Starting spatial map publish thread")
                    if (
                        self.spatial_map_publish_thread
                        and not self.spatial_map_publish_thread.is_alive()
                    ):
                        self.spatial_map_publish_thread.start()
            else:
                logger.error("[SPATIAL_MAPPING_INIT] Failed to enable spatial mapping")
                self.enable_spatial_mapping = False

        except Exception as e:
            logger.error(f"Error enabling spatial mapping: {e}")
            self.enable_spatial_mapping = False

    @rpc
    def stop(self):
        """Stop the ZED module."""
        if not self._running:
            return

        self._running = False

        # Stop subscription
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

        # Stop spatial map thread if running
        if self.spatial_map_publish_thread and self.spatial_map_publish_thread.is_alive():
            self.spatial_map_publish_thread.stop()
            self.spatial_map_publish_thread.join(timeout=2.0)
            self.spatial_map_publish_thread = None

        # Close camera
        if self.zed_camera:
            self.zed_camera.close()
            self.zed_camera = None

        logger.info("ZED module stopped")

    def _capture_and_publish(self):
        """Capture frame and publish all data."""
        if not self._running or not self.zed_camera:
            return

        try:
            # Single grab for all data
            if self.zed_camera.zed.grab(self.zed_camera.runtime_params) != sl.ERROR_CODE.SUCCESS:
                return

            # Retrieve all data from the single grab
            # Get left image
            self.zed_camera.zed.retrieve_image(self.zed_camera.image_left, sl.VIEW.LEFT)
            left_img = self.zed_camera.image_left.get_data()[:, :, :3]  # Remove alpha

            # Get depth for visualization (but don't use for pointcloud)
            self.zed_camera.zed.retrieve_measure(self.zed_camera.depth_map, sl.MEASURE.DEPTH)
            depth = self.zed_camera.depth_map.get_data()

            # Get pointcloud at lower resolution - EXACTLY like the demo
            self.zed_camera.zed.retrieve_measure(
                self.zed_camera.point_cloud,
                sl.MEASURE.XYZRGBA,
                sl.MEM.CPU,
                self.zed_camera.pc_resolution,
            )

            # Get pose if tracking enabled
            pose_data = None
            if self.enable_tracking:
                pose_data = self.zed_camera.get_pose()

            # Create header with optical frame
            header = Header(self.frame_id)  # camera_link
            self._sequence += 1

            # Publish all data
            self._publish_color_image(left_img, header)
            self._publish_depth_image(depth, header)
            self._publish_camera_info()

            # Publish pose if tracking enabled and valid
            if self.enable_tracking and pose_data and pose_data.get("valid", False):
                self._publish_pose(pose_data, header)
                self._publish_tf(pose_data, header)

        except Exception as e:
            logger.error(f"Error in capture and publish: {e}")

    def _publish_color_image(self, image: np.ndarray, header: Header):
        """Publish color image as LCM message."""
        try:
            # Convert BGR to RGB if needed
            if len(image.shape) == 3 and image.shape[2] == 3:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                image_rgb = image

            # Create LCM Image message
            msg = Image(
                data=image_rgb,
                format=ImageFormat.RGB,
                frame_id=header.frame_id,
                ts=header.ts,
            )

            self.color_image.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing color image: {e}")

    def _publish_depth_image(self, depth: np.ndarray, header: Header):
        """Publish depth image as LCM message."""
        try:
            # Depth is float32 in meters
            msg = Image(
                data=depth,
                format=ImageFormat.DEPTH,
                frame_id=header.frame_id,
                ts=header.ts,
            )
            self.depth_image.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing depth image: {e}")

    def _publish_camera_info(self):
        """Publish camera calibration information."""
        try:
            info = self.zed_camera.get_camera_info()
            if not info:
                return

            # Get calibration parameters
            left_cam = info.get("left_cam", {})
            resolution = info.get("resolution", {})

            # Create CameraInfo message
            header = Header(self.frame_id)

            # Create camera matrix K (3x3)
            K = [
                left_cam.get("fx", 0),
                0,
                left_cam.get("cx", 0),
                0,
                left_cam.get("fy", 0),
                left_cam.get("cy", 0),
                0,
                0,
                1,
            ]

            # Distortion coefficients
            D = [
                left_cam.get("k1", 0),
                left_cam.get("k2", 0),
                left_cam.get("p1", 0),
                left_cam.get("p2", 0),
                left_cam.get("k3", 0),
            ]

            # Identity rotation matrix
            R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

            # Projection matrix P (3x4)
            P = [
                left_cam.get("fx", 0),
                0,
                left_cam.get("cx", 0),
                0,
                0,
                left_cam.get("fy", 0),
                left_cam.get("cy", 0),
                0,
                0,
                0,
                1,
                0,
            ]

            msg = CameraInfo(
                D_length=len(D),
                header=header,
                height=resolution.get("height", 0),
                width=resolution.get("width", 0),
                distortion_model="plumb_bob",
                D=D,
                K=K,
                R=R,
                P=P,
                binning_x=0,
                binning_y=0,
            )

            self.camera_info.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing camera info: {e}")

    def _publish_pose(self, pose_data: Dict[str, Any], header: Header):
        """Publish camera pose as PoseStamped message only (TF handled by _publish_tf)."""
        try:
            position = pose_data.get("position", [0, 0, 0])
            rotation = pose_data.get("rotation", [0, 0, 0, 1])  # quaternion [x,y,z,w]

            # Create PoseStamped message
            msg = PoseStamped(
                ts=header.ts, position=position, orientation=rotation, frame_id="world"
            )
            self.pose.publish(msg)

            # Note: TF publishing is handled by _publish_tf() to avoid conflicts

        except Exception as e:
            logger.error(f"Error publishing pose: {e}")

    @rpc
    def get_camera_info(self) -> Dict[str, Any]:
        """Get camera information and calibration parameters."""
        if self.zed_camera:
            return self.zed_camera.get_camera_info()
        return {}

    @rpc
    def get_pose(self) -> Optional[Dict[str, Any]]:
        """Get current camera pose if tracking is enabled."""
        if self.zed_camera and self.enable_tracking:
            return self.zed_camera.get_pose()
        return None

    def _publish_tf(self, pose_data: Dict[str, Any], header: Header):
        """Publish complete TF chain: world → base_link → camera_link → camera_optical."""
        try:
            position = pose_data.get("position", [0, 0, 0])
            rotation = pose_data.get("rotation", [0, 0, 0, 1])  # quaternion [x,y,z,w]

            # World → base_link (from ZED tracking)
            # Note: ZED tracking gives camera pose, we need to transform to base_link
            base_tf = Transform(
                translation=Vector3(position),
                rotation=Quaternion(rotation),
                frame_id="world",
                child_frame_id="base_link",
                ts=header.ts,
            )

            # base_link → camera_link (physical offset - camera mounted 30cm forward)
            camera_link = Transform(
                translation=Vector3(0.3, 0.0, 0.1),  # 30cm forward, 10cm up
                rotation=Quaternion.from_euler(Vector3([0, 0, 0])),
                frame_id="base_link",
                child_frame_id="camera_link",
                ts=header.ts,
            )

            # No camera_optical transform needed - ZED already outputs in ROS frame

            # Publish transforms
            self.tf.publish(base_tf)
            self.tf.publish(camera_link)

            logger.debug("Published TF chain: world → base_link → camera_link")

        except Exception as e:
            logger.error(f"Error publishing TF: {e}")

    def cleanup(self):
        """Clean up resources on module destruction."""
        self.stop()
