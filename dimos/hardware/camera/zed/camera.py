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

from types import TracebackType
from typing import Any

import cv2
from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import pyzed.sl as sl  # type: ignore[import-not-found]
from reactivex import interval

from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3

# Import LCM message types
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.msgs.std_msgs import Header
from dimos.protocol.tf import TF
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ZEDCamera:
    """ZED Camera capture node with neural depth processing."""

    def __init__(  # type: ignore[no-untyped-def]
        self,
        camera_id: int = 0,
        resolution: sl.RESOLUTION = sl.RESOLUTION.HD720,
        depth_mode: sl.DEPTH_MODE = sl.DEPTH_MODE.NEURAL,
        fps: int = 30,
        **kwargs,
    ) -> None:
        """
        Initialize ZED Camera.

        Args:
            camera_id: Camera ID (0 for first ZED)
            resolution: ZED camera resolution
            depth_mode: Depth computation mode
            fps: Camera frame rate (default: 30)
        """
        if sl is None:
            raise ImportError("ZED SDK not installed. Please install pyzed package.")

        super().__init__(**kwargs)

        self.camera_id = camera_id
        self.resolution = resolution
        self.depth_mode = depth_mode
        self.fps = fps

        # Initialize ZED camera
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = resolution
        self.init_params.depth_mode = depth_mode
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.camera_fps = fps

        # Set camera ID using the correct parameter name
        if hasattr(self.init_params, "set_from_camera_id"):
            self.init_params.set_from_camera_id(camera_id)
        elif hasattr(self.init_params, "input"):
            self.init_params.input.set_from_camera_id(camera_id)

        # Use enable_fill_mode instead of SENSING_MODE.STANDARD
        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.enable_fill_mode = True  # False = STANDARD mode, True = FILL mode

        # Image containers
        self.image_left = sl.Mat()
        self.image_right = sl.Mat()
        self.depth_map = sl.Mat()
        self.point_cloud = sl.Mat()
        self.confidence_map = sl.Mat()

        # Positional tracking
        self.tracking_enabled = False
        self.tracking_params = sl.PositionalTrackingParameters()
        self.camera_pose = sl.Pose()
        self.sensors_data = sl.SensorsData()

        self.is_opened = False

    def open(self) -> bool:
        """Open the ZED camera."""
        try:
            err = self.zed.open(self.init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                logger.error(f"Failed to open ZED camera: {err}")
                return False

            self.is_opened = True
            logger.info("ZED camera opened successfully")

            # Get camera information
            info = self.zed.get_camera_information()
            logger.info(f"ZED Camera Model: {info.camera_model}")
            logger.info(f"Serial Number: {info.serial_number}")
            logger.info(f"Firmware: {info.camera_configuration.firmware_version}")

            return True

        except Exception as e:
            logger.error(f"Error opening ZED camera: {e}")
            return False

    def enable_positional_tracking(
        self,
        enable_area_memory: bool = False,
        enable_pose_smoothing: bool = True,
        enable_imu_fusion: bool = True,
        set_floor_as_origin: bool = False,
        initial_world_transform: sl.Transform | None = None,
    ) -> bool:
        """
        Enable positional tracking on the ZED camera.

        Args:
            enable_area_memory: Enable area learning to correct tracking drift
            enable_pose_smoothing: Enable pose smoothing
            enable_imu_fusion: Enable IMU fusion if available
            set_floor_as_origin: Set the floor as origin (useful for robotics)
            initial_world_transform: Initial world transform

        Returns:
            True if tracking enabled successfully
        """
        if not self.is_opened:
            logger.error("ZED camera not opened")
            return False

        try:
            # Configure tracking parameters
            self.tracking_params.enable_area_memory = enable_area_memory
            self.tracking_params.enable_pose_smoothing = enable_pose_smoothing
            self.tracking_params.enable_imu_fusion = enable_imu_fusion
            self.tracking_params.set_floor_as_origin = set_floor_as_origin

            if initial_world_transform is not None:
                self.tracking_params.initial_world_transform = initial_world_transform

            # Enable tracking
            err = self.zed.enable_positional_tracking(self.tracking_params)
            if err != sl.ERROR_CODE.SUCCESS:
                logger.error(f"Failed to enable positional tracking: {err}")
                return False

            self.tracking_enabled = True
            logger.info("Positional tracking enabled successfully")
            return True

        except Exception as e:
            logger.error(f"Error enabling positional tracking: {e}")
            return False

    def disable_positional_tracking(self) -> None:
        """Disable positional tracking."""
        if self.tracking_enabled:
            self.zed.disable_positional_tracking()
            self.tracking_enabled = False
            logger.info("Positional tracking disabled")

    def get_pose(
        self, reference_frame: sl.REFERENCE_FRAME = sl.REFERENCE_FRAME.WORLD
    ) -> dict[str, Any] | None:
        """
        Get the current camera pose.

        Args:
            reference_frame: Reference frame (WORLD or CAMERA)

        Returns:
            Dictionary containing:
                - position: [x, y, z] in meters
                - rotation: [x, y, z, w] quaternion
                - euler_angles: [roll, pitch, yaw] in radians
                - timestamp: Pose timestamp in nanoseconds
                - confidence: Tracking confidence (0-100)
                - valid: Whether pose is valid
        """
        if not self.tracking_enabled:
            logger.error("Positional tracking not enabled")
            return None

        try:
            # Get current pose
            tracking_state = self.zed.get_position(self.camera_pose, reference_frame)

            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                # Extract translation
                translation = self.camera_pose.get_translation().get()

                # Extract rotation (quaternion)
                rotation = self.camera_pose.get_orientation().get()

                # Get Euler angles
                euler = self.camera_pose.get_euler_angles()

                return {
                    "position": translation.tolist(),
                    "rotation": rotation.tolist(),  # [x, y, z, w]
                    "euler_angles": euler.tolist(),  # [roll, pitch, yaw]
                    "timestamp": self.camera_pose.timestamp.get_nanoseconds(),
                    "confidence": self.camera_pose.pose_confidence,
                    "valid": True,
                    "tracking_state": str(tracking_state),
                }
            else:
                logger.warning(f"Tracking state: {tracking_state}")
                return {"valid": False, "tracking_state": str(tracking_state)}

        except Exception as e:
            logger.error(f"Error getting pose: {e}")
            return None

    def get_imu_data(self) -> dict[str, Any] | None:
        """
        Get IMU sensor data if available.

        Returns:
            Dictionary containing:
                - orientation: IMU orientation quaternion [x, y, z, w]
                - angular_velocity: [x, y, z] in rad/s
                - linear_acceleration: [x, y, z] in m/s²
                - timestamp: IMU data timestamp
        """
        if not self.is_opened:
            logger.error("ZED camera not opened")
            return None

        try:
            # Get sensors data synchronized with images
            if (
                self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.IMAGE)
                == sl.ERROR_CODE.SUCCESS
            ):
                imu = self.sensors_data.get_imu_data()

                # Get IMU orientation
                imu_orientation = imu.get_pose().get_orientation().get()

                # Get angular velocity
                angular_vel = imu.get_angular_velocity()

                # Get linear acceleration
                linear_accel = imu.get_linear_acceleration()

                return {
                    "orientation": imu_orientation.tolist(),
                    "angular_velocity": angular_vel.tolist(),
                    "linear_acceleration": linear_accel.tolist(),
                    "timestamp": self.sensors_data.timestamp.get_nanoseconds(),
                    "temperature": self.sensors_data.temperature.get(sl.SENSOR_LOCATION.IMU),
                }
            else:
                return None

        except Exception as e:
            logger.error(f"Error getting IMU data: {e}")
            return None

    def capture_frame(
        self,
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:  # type: ignore[type-arg]
        """
        Capture a frame from ZED camera.

        Returns:
            Tuple of (left_image, right_image, depth_map) as numpy arrays
        """
        if not self.is_opened:
            logger.error("ZED camera not opened")
            return None, None, None

        try:
            # Grab frame
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image
                self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
                left_img = self.image_left.get_data()[:, :, :3]  # Remove alpha channel

                # Retrieve right image
                self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)
                right_img = self.image_right.get_data()[:, :, :3]  # Remove alpha channel

                # Retrieve depth map
                self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
                depth = self.depth_map.get_data()

                return left_img, right_img, depth
            else:
                logger.warning("Failed to grab frame from ZED camera")
                return None, None, None

        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return None, None, None

    def capture_pointcloud(self) -> o3d.geometry.PointCloud | None:
        """
        Capture point cloud from ZED camera.

        Returns:
            Open3D point cloud with XYZ coordinates and RGB colors
        """
        if not self.is_opened:
            logger.error("ZED camera not opened")
            return None

        try:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve point cloud with RGBA data
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
                point_cloud_data = self.point_cloud.get_data()

                # Convert to numpy array format
                _height, _width = point_cloud_data.shape[:2]
                points = point_cloud_data.reshape(-1, 4)

                # Extract XYZ coordinates
                xyz = points[:, :3]

                # Extract and unpack RGBA color data from 4th channel
                rgba_packed = points[:, 3].view(np.uint32)

                # Unpack RGBA: each 32-bit value contains 4 bytes (R, G, B, A)
                colors_rgba = np.zeros((len(rgba_packed), 4), dtype=np.uint8)
                colors_rgba[:, 0] = rgba_packed & 0xFF  # R
                colors_rgba[:, 1] = (rgba_packed >> 8) & 0xFF  # G
                colors_rgba[:, 2] = (rgba_packed >> 16) & 0xFF  # B
                colors_rgba[:, 3] = (rgba_packed >> 24) & 0xFF  # A

                # Extract RGB (ignore alpha) and normalize to [0, 1]
                colors_rgb = colors_rgba[:, :3].astype(np.float64) / 255.0

                # Filter out invalid points (NaN or inf)
                valid = np.isfinite(xyz).all(axis=1)
                valid_xyz = xyz[valid]
                valid_colors = colors_rgb[valid]

                # Create Open3D point cloud
                pcd = o3d.geometry.PointCloud()

                if len(valid_xyz) > 0:
                    pcd.points = o3d.utility.Vector3dVector(valid_xyz)
                    pcd.colors = o3d.utility.Vector3dVector(valid_colors)

                return pcd
            else:
                logger.warning("Failed to grab frame for point cloud")
                return None

        except Exception as e:
            logger.error(f"Error capturing point cloud: {e}")
            return None

    def capture_frame_with_pose(
        self,
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, dict[str, Any] | None]:  # type: ignore[type-arg]
        """
        Capture a frame with synchronized pose data.

        Returns:
            Tuple of (left_image, right_image, depth_map, pose_data)
        """
        if not self.is_opened:
            logger.error("ZED camera not opened")
            return None, None, None, None

        try:
            # Grab frame
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Get images and depth
                left_img, right_img, depth = self.capture_frame()

                # Get synchronized pose if tracking is enabled
                pose_data = None
                if self.tracking_enabled:
                    pose_data = self.get_pose()

                return left_img, right_img, depth, pose_data
            else:
                logger.warning("Failed to grab frame from ZED camera")
                return None, None, None, None

        except Exception as e:
            logger.error(f"Error capturing frame with pose: {e}")
            return None, None, None, None

    def close(self) -> None:
        """Close the ZED camera."""
        if self.is_opened:
            # Disable tracking if enabled
            if self.tracking_enabled:
                self.disable_positional_tracking()

            self.zed.close()
            self.is_opened = False
            logger.info("ZED camera closed")

    def get_camera_info(self) -> dict[str, Any]:
        """Get ZED camera information and calibration parameters."""
        if not self.is_opened:
            return {}

        try:
            info = self.zed.get_camera_information()
            calibration = info.camera_configuration.calibration_parameters

            # In ZED SDK 4.0+, the baseline calculation has changed
            # Try to get baseline from the stereo parameters
            try:
                # Method 1: Try to get from stereo parameters if available
                if hasattr(calibration, "getCameraBaseline"):
                    baseline = calibration.getCameraBaseline()
                else:
                    # Method 2: Calculate from left and right camera positions
                    # The baseline is the distance between left and right cameras

                    # Try different ways to get baseline in SDK 4.0+
                    if hasattr(info.camera_configuration, "calibration_parameters_raw"):
                        # Use raw calibration if available
                        raw_calib = info.camera_configuration.calibration_parameters_raw
                        if hasattr(raw_calib, "T"):
                            baseline = abs(raw_calib.T[0])
                        else:
                            baseline = 0.12  # Default ZED-M baseline approximation
                    else:
                        # Use default baseline for ZED-M
                        baseline = 0.12  # ZED-M baseline is approximately 120mm
            except:
                baseline = 0.12  # Fallback to approximate ZED-M baseline

            return {
                "model": str(info.camera_model),
                "serial_number": info.serial_number,
                "firmware": info.camera_configuration.firmware_version,
                "resolution": {
                    "width": info.camera_configuration.resolution.width,
                    "height": info.camera_configuration.resolution.height,
                },
                "fps": info.camera_configuration.fps,
                "left_cam": {
                    "fx": calibration.left_cam.fx,
                    "fy": calibration.left_cam.fy,
                    "cx": calibration.left_cam.cx,
                    "cy": calibration.left_cam.cy,
                    "k1": calibration.left_cam.disto[0],
                    "k2": calibration.left_cam.disto[1],
                    "p1": calibration.left_cam.disto[2],
                    "p2": calibration.left_cam.disto[3],
                    "k3": calibration.left_cam.disto[4],
                },
                "right_cam": {
                    "fx": calibration.right_cam.fx,
                    "fy": calibration.right_cam.fy,
                    "cx": calibration.right_cam.cx,
                    "cy": calibration.right_cam.cy,
                    "k1": calibration.right_cam.disto[0],
                    "k2": calibration.right_cam.disto[1],
                    "p1": calibration.right_cam.disto[2],
                    "p2": calibration.right_cam.disto[3],
                    "k3": calibration.right_cam.disto[4],
                },
                "baseline": baseline,
            }
        except Exception as e:
            logger.error(f"Error getting camera info: {e}")
            return {}

    def calculate_intrinsics(self):  # type: ignore[no-untyped-def]
        """Calculate camera intrinsics from ZED calibration."""
        info = self.get_camera_info()
        if not info:
            return super().calculate_intrinsics()  # type: ignore[misc]

        left_cam = info.get("left_cam", {})
        resolution = info.get("resolution", {})

        return {
            "focal_length_x": left_cam.get("fx", 0),
            "focal_length_y": left_cam.get("fy", 0),
            "principal_point_x": left_cam.get("cx", 0),
            "principal_point_y": left_cam.get("cy", 0),
            "baseline": info.get("baseline", 0),
            "resolution_width": resolution.get("width", 0),
            "resolution_height": resolution.get("height", 0),
        }

    def __enter__(self):  # type: ignore[no-untyped-def]
        """Context manager entry."""
        if not self.open():
            raise RuntimeError("Failed to open ZED camera")
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        """Context manager exit."""
        self.close()


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
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    pose: Out[PoseStamped]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        camera_id: int = 0,
        resolution: str = "HD720",
        depth_mode: str = "NEURAL",
        fps: int = 30,
        enable_tracking: bool = True,
        enable_imu_fusion: bool = True,
        set_floor_as_origin: bool = True,
        publish_rate: float = 30.0,
        recording_path: str | None = None,
        **kwargs,
    ) -> None:
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
            recording_path: Path to save recorded data
        """
        super().__init__(**kwargs)

        self.camera_id = camera_id
        self.fps = fps
        self.enable_tracking = enable_tracking
        self.enable_imu_fusion = enable_imu_fusion
        self.set_floor_as_origin = set_floor_as_origin
        self.publish_rate = publish_rate
        self.recording_path = recording_path

        # Convert string parameters to ZED enums
        self.resolution = getattr(sl.RESOLUTION, resolution, sl.RESOLUTION.HD720)
        self.depth_mode = getattr(sl.DEPTH_MODE, depth_mode, sl.DEPTH_MODE.NEURAL)

        # Internal state
        self.zed_camera = None
        self._running = False
        self._subscription = None
        self._sequence = 0

        # Initialize TF publisher
        self.tf = TF()

        # Initialize storage for recording if path provided
        self.storages: dict[str, Any] | None = None
        if self.recording_path:
            from dimos.utils.testing import TimedSensorStorage

            self.storages = {
                "color": TimedSensorStorage(f"{self.recording_path}/color"),
                "depth": TimedSensorStorage(f"{self.recording_path}/depth"),
                "pose": TimedSensorStorage(f"{self.recording_path}/pose"),
                "camera_info": TimedSensorStorage(f"{self.recording_path}/camera_info"),
            }
            logger.info(f"Recording enabled - saving to {self.recording_path}")

        logger.info(f"ZEDModule initialized for camera {camera_id}")

    @rpc
    def start(self) -> None:
        """Start the ZED module and begin publishing data."""
        if self._running:
            logger.warning("ZED module already running")
            return

        super().start()

        try:
            # Initialize ZED camera
            self.zed_camera = ZEDCamera(  # type: ignore[assignment]
                camera_id=self.camera_id,
                resolution=self.resolution,
                depth_mode=self.depth_mode,
                fps=self.fps,
            )

            # Open camera
            if not self.zed_camera.open():  # type: ignore[attr-defined]
                logger.error("Failed to open ZED camera")
                return

            # Enable tracking if requested
            if self.enable_tracking:
                success = self.zed_camera.enable_positional_tracking(  # type: ignore[attr-defined]
                    enable_imu_fusion=self.enable_imu_fusion,
                    set_floor_as_origin=self.set_floor_as_origin,
                    enable_pose_smoothing=True,
                    enable_area_memory=True,
                )
                if not success:
                    logger.warning("Failed to enable positional tracking")
                    self.enable_tracking = False

            # Publish camera info once at startup
            self._publish_camera_info()

            # Start periodic frame capture and publishing
            self._running = True
            publish_interval = 1.0 / self.publish_rate

            self._subscription = interval(publish_interval).subscribe(  # type: ignore[assignment]
                lambda _: self._capture_and_publish()
            )

            logger.info(f"ZED module started, publishing at {self.publish_rate} Hz")

        except Exception as e:
            logger.error(f"Error starting ZED module: {e}")
            self._running = False

    @rpc
    def stop(self) -> None:
        """Stop the ZED module."""
        if not self._running:
            return

        self._running = False

        # Stop subscription
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

        # Close camera
        if self.zed_camera:
            self.zed_camera.close()
            self.zed_camera = None

        super().stop()

    def _capture_and_publish(self) -> None:
        """Capture frame and publish all data."""
        if not self._running or not self.zed_camera:
            return

        try:
            # Capture frame with pose
            left_img, _, depth, pose_data = self.zed_camera.capture_frame_with_pose()

            if left_img is None or depth is None:
                return

            # Save raw color data if recording
            if self.storages and left_img is not None:
                self.storages["color"].save_one(left_img)

            # Save raw depth data if recording
            if self.storages and depth is not None:
                self.storages["depth"].save_one(depth)

            # Save raw pose data if recording
            if self.storages and pose_data:
                self.storages["pose"].save_one(pose_data)

            # Create header
            header = Header(self.frame_id)
            self._sequence += 1

            # Publish color image
            self._publish_color_image(left_img, header)

            # Publish depth image
            self._publish_depth_image(depth, header)

            # Publish camera info periodically
            self._publish_camera_info()

            # Publish pose if tracking enabled and valid
            if self.enable_tracking and pose_data and pose_data.get("valid", False):
                self._publish_pose(pose_data, header)

        except Exception as e:
            logger.error(f"Error in capture and publish: {e}")

    def _publish_color_image(self, image: np.ndarray, header: Header) -> None:  # type: ignore[type-arg]
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

    def _publish_depth_image(self, depth: np.ndarray, header: Header) -> None:  # type: ignore[type-arg]
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

    def _publish_camera_info(self) -> None:
        """Publish camera calibration information."""
        try:
            info = self.zed_camera.get_camera_info()  # type: ignore[attr-defined]
            if not info:
                return

            # Save raw camera info if recording
            if self.storages:
                self.storages["camera_info"].save_one(info)

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

    def _publish_pose(self, pose_data: dict[str, Any], header: Header) -> None:
        """Publish camera pose as PoseStamped message and TF transform."""
        try:
            position = pose_data.get("position", [0, 0, 0])
            rotation = pose_data.get("rotation", [0, 0, 0, 1])  # quaternion [x,y,z,w]

            # Create PoseStamped message
            msg = PoseStamped(ts=header.ts, position=position, orientation=rotation)
            self.pose.publish(msg)

            # Publish TF transform
            camera_tf = Transform(
                translation=Vector3(position),
                rotation=Quaternion(rotation),
                frame_id="zed_world",
                child_frame_id="zed_camera_link",
                ts=header.ts,
            )
            self.tf.publish(camera_tf)

        except Exception as e:
            logger.error(f"Error publishing pose: {e}")

    @rpc
    def get_camera_info(self) -> dict[str, Any]:
        """Get camera information and calibration parameters."""
        if self.zed_camera:
            return self.zed_camera.get_camera_info()
        return {}

    @rpc
    def get_pose(self) -> dict[str, Any] | None:
        """Get current camera pose if tracking is enabled."""
        if self.zed_camera and self.enable_tracking:
            return self.zed_camera.get_pose()
        return None
