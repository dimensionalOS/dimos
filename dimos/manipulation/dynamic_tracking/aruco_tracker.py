# Copyright 2026 Dimensional Inc.
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

"""ArUco marker tracker for visual servoing.

Detects ArUco markers in camera images, estimates their pose relative to the
camera, publishes transforms to the TF tree, and outputs a target EE pose
(as PoseStamped) for the ControlCoordinator's CartesianIKTask.

The visual servo loop is:
    camera image → ArUco detection → marker pose in camera frame
    → TF lookup (base_link → marker) → compute reach pose
    → publish PoseStamped on cartesian_command → CartesianIKTask handles IK

TF tree (eye-in-hand):
    base_link → ... → ee_link → camera_link → camera_color_optical_frame → aruco_avg
"""

from dataclasses import dataclass, field
import math
from threading import Event, Thread
import time
from typing import Any

import cv2
import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _estimate_marker_poses(
    corners: list[np.ndarray],
    marker_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate per-marker pose via solvePnP (replaces deprecated estimatePoseSingleMarkers).

    Returns rvecs, tvecs each shaped (N, 1, 3) to match the legacy API.
    """
    half = marker_size / 2.0
    # Marker corner order from cv2.aruco matches this object-point layout
    obj_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float32,
    )
    rvecs = np.zeros((len(corners), 1, 3), dtype=np.float64)
    tvecs = np.zeros((len(corners), 1, 3), dtype=np.float64)
    for i, c in enumerate(corners):
        ok, rvec, tvec = cv2.solvePnP(
            obj_points, c.reshape(-1, 2), camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if ok:
            rvecs[i, 0] = rvec.flatten()
            tvecs[i, 0] = tvec.flatten()
    return rvecs, tvecs


@dataclass
class ArucoTrackerConfig(ModuleConfig):
    """Configuration for the ArUco tracker module."""

    # ArUco detection
    marker_size: float = 0.1  # meters
    aruco_dict: int = cv2.aruco.DICT_4X4_50
    expected_marker_count: int = 4

    # Processing
    camera_frame_id: str = "camera_color_optical_frame"
    rate: float = 15.0  # Hz
    max_loops: int = 0  # 0 = run forever

    # Visual servo target
    target_task_name: str = "cartesian_ik_arm"  # CartesianIKTask name in coordinator
    enable_servo: bool = True
    reach_offset: list[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.10]
    )  # xyz offset in base frame
    min_move_distance_m: float = 0.003  # skip moves smaller than this

    # Default orientation for reach pose (RPY in radians, pointing down)
    reach_orientation_rpy: list[float] = field(default_factory=lambda: [math.pi, 0.0, 0.0])
    use_marker_orientation: bool = False


class ArucoTracker(Module[ArucoTrackerConfig]):
    """ArUco marker tracker with visual servoing output.

    Detects ArUco markers, publishes their transforms to the TF tree,
    and outputs a target EE PoseStamped for CartesianIKTask-based servoing.
    """

    # Inputs
    color_image: In[Image]
    camera_info: In[CameraInfo]

    # Outputs
    annotated_image: Out[Image]
    cartesian_command: Out[PoseStamped]  # target EE pose for visual servo

    config: ArucoTrackerConfig
    default_config = ArucoTrackerConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # ArUco detector
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(self.config.aruco_dict)
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)

        # Camera intrinsics (populated from CameraInfo)
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._latest_image: Image | None = None

        # Processing thread
        self._stop_event = Event()
        self._processing_thread: Thread | None = None
        self._loop_count = 0

        # Last published position for min-move filter
        self._last_target_position: np.ndarray | None = None

    # =========================================================================
    # Lifecycle
    # =========================================================================

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.camera_info.observable().subscribe(self._update_camera_info))
        self._disposables.add(self.color_image.observable().subscribe(self._store_latest_image))

        self._loop_count = 0
        self._stop_event.clear()
        self._processing_thread = Thread(
            target=self._processing_loop, daemon=True, name="ArucoTracker"
        )
        self._processing_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._processing_thread is not None and self._processing_thread.is_alive():
            self._processing_thread.join(timeout=2.0)
        super().stop()

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _store_latest_image(self, image: Image) -> None:
        self._latest_image = image

    def _update_camera_info(self, camera_info: CameraInfo) -> None:
        if len(camera_info.K) == 9:
            fx, _, cx, _, fy, cy, _, _, _ = camera_info.K
            self._camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
            self._dist_coeffs = (
                np.array(camera_info.D, dtype=np.float32)
                if camera_info.D
                else np.zeros(5, dtype=np.float32)
            )

    # =========================================================================
    # Processing loop
    # =========================================================================

    def _processing_loop(self) -> None:
        period = 1.0 / self.config.rate
        logger.info(f"ArUco processing loop started at {self.config.rate}Hz")

        while not self._stop_event.is_set():
            if self.config.max_loops > 0 and self._loop_count >= self.config.max_loops:
                break

            loop_start = time.time()
            try:
                if self._latest_image is not None:
                    self._update_tracking(self._latest_image)
                    self._loop_count += 1
            except Exception as e:
                logger.error(f"Error in ArUco processing: {e}")

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)

        logger.info(f"ArUco processing loop completed after {self._loop_count} iterations")

    def _update_tracking(self, image: Image) -> None:
        if self._camera_matrix is None or self._dist_coeffs is None:
            return

        # Convert to grayscale for detection
        if image.format.name == "RGB":
            display_image = image.data.copy()
            gray = cv2.cvtColor(image.data, cv2.COLOR_RGB2GRAY)
        elif image.format.name == "BGR":
            display_image = image.data.copy()
            gray = cv2.cvtColor(image.data, cv2.COLOR_BGR2GRAY)
        else:
            display_image = image.data.copy()
            gray = image.data

        corners, ids, _ = self._detector.detectMarkers(gray)

        num_detected = 0 if ids is None else len(ids)
        if num_detected != self.config.expected_marker_count:
            logger.debug(f"Detected {num_detected}/{self.config.expected_marker_count} markers")
            self._publish_annotated_image(display_image, image.format.name)
            return

        # Estimate poses
        rvecs, tvecs = _estimate_marker_poses(
            list(corners), self.config.marker_size, self._camera_matrix, self._dist_coeffs
        )
        avg_position, avg_quat = self._average_marker_poses(rvecs, tvecs)

        # Publish transform: camera_optical → aruco_avg
        ts = image.ts
        aruco_tf = Transform(
            translation=Vector3(
                float(avg_position[0]), float(avg_position[1]), float(avg_position[2])
            ),
            rotation=Quaternion(
                float(avg_quat[0]), float(avg_quat[1]), float(avg_quat[2]), float(avg_quat[3])
            ),
            frame_id=self.config.camera_frame_id,
            child_frame_id="aruco_avg",
            ts=ts,
        )
        self.tf.publish(aruco_tf)
        rr.log("world/tf/aruco_avg", aruco_tf.to_rerun())

        # Visual servo: compute and publish target EE pose
        if self.config.enable_servo:
            self._servo_to_marker(ts)

        # Draw markers
        self._draw_markers(display_image, corners, ids, rvecs, tvecs, image.format.name)

    # =========================================================================
    # Visual servo
    # =========================================================================

    def _servo_to_marker(self, ts: float) -> None:
        """Compute reach pose from marker and publish as cartesian command."""
        # Look up marker position in base_link frame via TF tree
        aruco_wrt_base = self.tf.get("base_link", "aruco_avg")
        if aruco_wrt_base is None:
            logger.debug("TF lookup base_link→aruco_avg failed, skipping servo")
            return

        t = aruco_wrt_base.translation
        offset = self.config.reach_offset
        target_pos = Vector3(t.x + offset[0], t.y + offset[1], t.z + offset[2])

        # Min-move filter
        pos_array = np.array([target_pos.x, target_pos.y, target_pos.z])
        if self._last_target_position is not None:
            distance = float(np.linalg.norm(pos_array - self._last_target_position))
            if distance < self.config.min_move_distance_m:
                return

        # Orientation: follow marker or use fixed default
        if self.config.use_marker_orientation:
            rpy = aruco_wrt_base.rotation.to_euler()
            roll = ((rpy.x + math.pi + math.pi) % (2 * math.pi)) - math.pi
            pitch = np.clip(rpy.y, -math.pi / 2, math.pi / 2)
            yaw = np.clip(rpy.z + math.pi / 2, -math.pi / 2, math.pi / 2)
            orientation = Quaternion.from_euler(Vector3(roll, pitch, yaw))
        else:
            rpy = self.config.reach_orientation_rpy
            orientation = Quaternion.from_euler(Vector3(rpy[0], rpy[1], rpy[2]))

        # Publish PoseStamped with frame_id = task name (for coordinator routing)
        command = PoseStamped(
            ts=ts,
            frame_id=self.config.target_task_name,
            position=target_pos,
            orientation=orientation,
        )
        self.cartesian_command.publish(command)
        self._last_target_position = pos_array

        logger.debug(f"Servo target: [{target_pos.x:.3f}, {target_pos.y:.3f}, {target_pos.z:.3f}]")

    # =========================================================================
    # ArUco pose computation
    # =========================================================================

    def _average_marker_poses(
        self, rvecs: np.ndarray, tvecs: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Average multiple marker poses into a single pose."""
        rotations = []
        positions = []
        for i in range(len(rvecs)):
            rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
            rotations.append(Rotation.from_matrix(rot_matrix))
            positions.append(tvecs[i][0])

        avg_position = np.mean(positions, axis=0)
        avg_rotation = Rotation.concatenate(rotations).mean()
        avg_quat = avg_rotation.as_quat()  # [x, y, z, w]
        return avg_position, avg_quat

    # =========================================================================
    # Image annotation
    # =========================================================================

    def _publish_annotated_image(self, display_image: np.ndarray, image_format: str) -> None:
        if image_format == "BGR":
            publish_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
        else:
            publish_image = display_image

        annotated_msg = Image(
            data=publish_image,
            format=ImageFormat.RGB,
            frame_id=self.config.camera_frame_id,
            ts=time.time(),
        )
        self.annotated_image.publish(annotated_msg)
        rr.log("aruco/annotated", rr.Image(publish_image))

    def _draw_markers(
        self,
        display_image: np.ndarray,
        corners: list[np.ndarray],
        ids: np.ndarray,
        rvecs: np.ndarray,
        tvecs: np.ndarray,
        image_format: str,
    ) -> None:
        cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
        for i in range(len(ids)):
            cv2.drawFrameAxes(
                display_image,
                self._camera_matrix,
                self._dist_coeffs,
                rvecs[i][0],
                tvecs[i][0],
                self.config.marker_size * 0.5,
            )
        self._publish_annotated_image(display_image, image_format)


aruco_tracker = ArucoTracker.blueprint

__all__ = ["ArucoTracker", "ArucoTrackerConfig", "aruco_tracker"]
