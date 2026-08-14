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

"""Native DimOS module for the benchmark's public one-episode UDS stream."""

import math
from threading import Lock
import time
from typing import Any

import grpc  # type: ignore[import-untyped]
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

from .protocol import vlnce_public_v1_pb2 as pb
from .protocol.contract import (
    BASE_FRAME,
    CAMERA_FRAME,
    CONTROL_PERIOD_SECONDS,
    MAX_ANGULAR_Z,
    MAX_LINEAR_X,
    MAX_LINEAR_Y,
    WORLD_FRAME,
)
from .protocol.vlnce_public_v1_pb2_grpc import VlncePublicGatewayStub

logger = setup_logger()

_HABITAT_TO_DIMOS = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)
_HABITAT_TO_DIMOS_ROTATION = Rotation.from_matrix(_HABITAT_TO_DIMOS)


class VlnceConnectionError(RuntimeError):
    """The public benchmark connection failed or violated its contract."""


class VlnceConnectionConfig(ModuleConfig):
    socket_path: str
    connect_timeout_seconds: float = Field(default=30.0, gt=0.0)


class VlnceConnection(Module):
    """Expose only native public sensor, map, pose, and planar-control interfaces."""

    dedicated_worker = True
    config: VlnceConnectionConfig

    cmd_vel: In[Twist]
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    pointcloud: Out[PointCloud2]
    odometry: Out[Odometry]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    global_costmap: Out[OccupancyGrid]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._state_lock = Lock()
        self._state = "created"
        self._last_observation_sequence = 0
        self._observation_count = 0
        self._accepted_control_count = 0
        self._rejected_control_count = 0
        self._last_control: dict[str, float] | None = None
        self._channel: grpc.Channel | None = None
        self._stub: VlncePublicGatewayStub | None = None
        self._wall_minus_monotonic = time.time() - time.monotonic()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self._channel = grpc.insecure_channel(f"unix://{self.config.socket_path}")
        self._stub = VlncePublicGatewayStub(self._channel)

    @rpc
    def stop(self) -> None:
        try:
            if self._state in {"ready", "running"}:
                try:
                    self.cancel()
                except VlnceConnectionError:
                    logger.info("VLN-CE gateway already stopped")
            if self._channel is not None:
                self._channel.close()
        finally:
            super().stop()

    @rpc
    def wait_ready(self, timeout_seconds: float | None = None) -> bool:
        """Wait for the attempt-local UDS gateway."""

        timeout = timeout_seconds or self.config.connect_timeout_seconds
        if self._channel is None:
            raise VlnceConnectionError("benchmark gateway channel is unavailable")
        try:
            grpc.channel_ready_future(self._channel).result(timeout=timeout)
        except grpc.FutureTimeoutError as error:
            raise VlnceConnectionError("benchmark gateway did not become ready") from error
        with self._state_lock:
            if self._state == "created":
                self._state = "ready"
        return True

    @rpc
    def begin(self) -> bool:
        """Begin the episode and wait for its first coherent public observation."""

        self.wait_ready()
        with self._state_lock:
            if self._state != "ready" or self._stub is None:
                raise VlnceConnectionError("episode can begin only once after readiness")
            try:
                observation = self._stub.Start(
                    pb.StartRequest(), timeout=self.config.connect_timeout_seconds
                )
            except grpc.RpcError as error:
                raise VlnceConnectionError("benchmark rejected episode start") from error
            self._publish_observation(observation)
            self._state = "running"
        return True

    @rpc
    def cancel(self) -> bool:
        """Cancel transport without submitting a route for official scoring."""

        with self._state_lock:
            if self._state not in {"ready", "running"} or self._stub is None:
                return False
            try:
                self._stub.Cancel(pb.CancelRequest(), timeout=self.config.connect_timeout_seconds)
            except grpc.RpcError as error:
                raise VlnceConnectionError("benchmark rejected cancellation") from error
            self._state = "cancelled"
        return True

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send one fixed-period bounded planar velocity command."""

        if duration != 0.0:
            raise VlnceConnectionError("VLN-CE controls use the negotiated fixed period")
        values = (twist.linear.x, twist.linear.y, twist.angular.z)
        if not all(math.isfinite(value) for value in values):
            raise VlnceConnectionError("planar velocity must be finite")
        if (
            abs(twist.linear.x) > MAX_LINEAR_X
            or abs(twist.linear.y) > MAX_LINEAR_Y
            or abs(twist.angular.z) > MAX_ANGULAR_Z
        ):
            raise VlnceConnectionError("planar velocity exceeds the negotiated limit")
        with self._state_lock:
            if self._state != "running" or self._stub is None:
                self._rejected_control_count += 1
                return False
            request = pb.PlanarControl(
                observation_sequence=self._last_observation_sequence,
                linear_x=twist.linear.x,
                linear_y=twist.linear.y,
                angular_z=twist.angular.z,
            )
            try:
                observation = self._stub.StepPlanar(
                    request, timeout=self.config.connect_timeout_seconds
                )
            except grpc.RpcError as error:
                self._rejected_control_count += 1
                raise VlnceConnectionError("benchmark rejected planar control") from error
            self._publish_observation(observation)
            self._accepted_control_count += 1
            self._last_control = {
                "linear_x": twist.linear.x,
                "linear_y": twist.linear.y,
                "angular_z": twist.angular.z,
            }
        return True

    @rpc
    def turn(self, angle_degrees: float) -> str:
        """Turn in place by a relative angle of at most 180 degrees.

        Positive angles turn counterclockwise; negative angles turn clockwise.
        The call blocks until the requested fixed-period controls have completed.
        """

        if not math.isfinite(angle_degrees) or abs(angle_degrees) > 180.0:
            raise VlnceConnectionError("turn angle must be finite and within 180 degrees")
        if angle_degrees == 0.0:
            return "No turn requested."

        remaining = math.radians(angle_degrees)
        while not math.isclose(remaining, 0.0, abs_tol=1e-9):
            angular_z = math.copysign(
                min(MAX_ANGULAR_Z, abs(remaining) / CONTROL_PERIOD_SECONDS),
                remaining,
            )
            if not self.move(Twist(angular=[0.0, 0.0, angular_z])):
                raise VlnceConnectionError("turn was rejected before it completed")
            remaining -= angular_z * CONTROL_PERIOD_SECONDS

        direction = "counterclockwise" if angle_degrees > 0.0 else "clockwise"
        return f"Turned {direction} {abs(angle_degrees):.1f} degrees."

    @skill
    def submit_route(self) -> str:
        """Submit VLN-CE STOP once and end this evaluation.

        Call only when the assigned route is complete. Submission is irreversible
        and acknowledges only that the route was submitted; it never reveals score,
        success, distance, or benchmark progress.
        """

        with self._state_lock:
            if self._state != "running" or self._stub is None:
                raise VlnceConnectionError("route can be submitted only from a settled episode")
            try:
                self._stub.SubmitRoute(
                    pb.SubmitRouteRequest(observation_sequence=self._last_observation_sequence),
                    timeout=self.config.connect_timeout_seconds,
                )
            except grpc.RpcError as error:
                raise VlnceConnectionError("route submission was not acknowledged") from error
            self._state = "submitted"
        return "Route submitted; the VLN-CE evaluation is ending."

    @rpc
    def public_diagnostics(self) -> dict[str, Any]:
        """Return public transport evidence without benchmark-private state."""

        with self._state_lock:
            return {
                "schema_version": "vlnce-public-diagnostics.v1",
                "state": self._state,
                "observation_count": self._observation_count,
                "last_observation_sequence": self._last_observation_sequence,
                "accepted_control_count": self._accepted_control_count,
                "rejected_control_count": self._rejected_control_count,
                "last_control": self._last_control,
                "route_submitted": self._state == "submitted",
                "frames": {
                    "world": WORLD_FRAME,
                    "base": BASE_FRAME,
                    "camera": CAMERA_FRAME,
                },
                "control": {
                    "period_seconds": CONTROL_PERIOD_SECONDS,
                    "max_linear_x": MAX_LINEAR_X,
                    "max_linear_y": MAX_LINEAR_Y,
                    "max_angular_z": MAX_ANGULAR_Z,
                },
            }

    def _on_cmd_vel(self, twist: Twist) -> None:
        bounded = Twist(
            linear=[
                max(-MAX_LINEAR_X, min(MAX_LINEAR_X, twist.linear.x)),
                max(-MAX_LINEAR_Y, min(MAX_LINEAR_Y, twist.linear.y)),
                0.0,
            ],
            angular=[
                0.0,
                0.0,
                max(
                    -MAX_ANGULAR_Z,
                    min(MAX_ANGULAR_Z, twist.angular.z),
                ),
            ],
        )
        if not self.move(bounded):
            logger.debug("VLN-CE command rejected while another command is in flight")

    def _publish_observation(self, observation: pb.Observation) -> None:
        if observation.sequence <= self._last_observation_sequence:
            raise VlnceConnectionError("observation sequence did not increase")
        native = decode_observation(observation, self._wall_minus_monotonic)
        self.tf.publish(native["tf"])
        self.odometry.publish(native["odometry"])
        self.odom.publish(native["odom"])
        self.color_image.publish(native["color_image"])
        self.depth_image.publish(native["depth_image"])
        self.camera_info.publish(native["camera_info"])
        self.depth_camera_info.publish(native["depth_camera_info"])
        self.pointcloud.publish(native["pointcloud"])
        occupancy = native["global_costmap"]
        if occupancy is not None:
            self.global_costmap.publish(occupancy)
        self._last_observation_sequence = observation.sequence
        self._observation_count += 1


def decode_observation(
    observation: pb.Observation,
    wall_minus_monotonic: float,
) -> dict[str, Any]:
    """Validate and translate one complete protocol epoch into native messages."""

    rgb_calibration = observation.rgb_calibration
    depth_calibration = observation.depth_calibration
    expected_rgb_bytes = rgb_calibration.width * rgb_calibration.height * 3
    expected_depth_bytes = depth_calibration.width * depth_calibration.height * 4
    if len(observation.rgb) != expected_rgb_bytes or len(observation.depth) != expected_depth_bytes:
        raise VlnceConnectionError("observation image payload size is invalid")
    rgb = np.frombuffer(observation.rgb, dtype=np.uint8).reshape(
        rgb_calibration.height, rgb_calibration.width, 3
    )
    depth = np.frombuffer(observation.depth, dtype="<f4").reshape(
        depth_calibration.height, depth_calibration.width
    )
    timestamp = wall_minus_monotonic + observation.monotonic_time_ns / 1_000_000_000.0
    color_image = Image.from_numpy(
        rgb.copy(), format=ImageFormat.RGB, frame_id="camera_optical", ts=timestamp
    )
    depth_image = Image.from_numpy(
        depth.copy(), format=ImageFormat.DEPTH, frame_id="camera_optical", ts=timestamp
    )
    camera_info = _camera_info(rgb_calibration, timestamp)
    depth_camera_info = _camera_info(depth_calibration, timestamp)
    position, orientation = _habitat_pose_to_dimos(observation.world_from_base)
    pose = Pose(position=position, orientation=orientation)
    odom = PoseStamped(ts=timestamp, frame_id="world", position=position, orientation=orientation)
    odometry = Odometry(
        ts=timestamp,
        frame_id="world",
        child_frame_id="base_link",
        pose=pose,
    )
    base_from_camera = _base_from_camera(observation.base_from_camera, timestamp)
    transforms = TFMessage(
        Transform(
            translation=Vector3(position),
            rotation=Quaternion(orientation),
            frame_id="world",
            child_frame_id="base_link",
            ts=timestamp,
        ),
        base_from_camera,
    )
    occupancy = (
        _decode_occupancy(observation.static_map, timestamp)
        if observation.HasField("static_map")
        else None
    )
    return {
        "color_image": color_image,
        "depth_image": depth_image,
        "camera_info": camera_info,
        "depth_camera_info": depth_camera_info,
        "pointcloud": _depth_to_pointcloud(depth, depth_calibration, timestamp),
        "odometry": odometry,
        "odom": odom,
        "tf": transforms,
        "global_costmap": occupancy,
    }


def _camera_info(calibration: pb.Calibration, timestamp: float) -> CameraInfo:
    return CameraInfo.from_intrinsics(
        fx=calibration.fx,
        fy=calibration.fy,
        cx=calibration.cx,
        cy=calibration.cy,
        width=calibration.width,
        height=calibration.height,
        frame_id="camera_optical",
    ).with_ts(timestamp)


def _depth_to_pointcloud(
    depth: np.ndarray[Any, np.dtype[np.float32]],
    calibration: pb.Calibration,
    timestamp: float,
) -> PointCloud2:
    rows, columns = np.indices(depth.shape, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0.0)
    z = depth[valid]
    points = np.column_stack(
        (
            (columns[valid] - calibration.cx) * z / calibration.fx,
            (rows[valid] - calibration.cy) * z / calibration.fy,
            z,
        )
    ).astype(np.float32)
    return PointCloud2.from_numpy(points, frame_id="camera_optical", timestamp=timestamp)


def _habitat_pose_to_dimos(pose: pb.Pose) -> tuple[np.ndarray[Any, Any], np.ndarray[Any, Any]]:
    position = _HABITAT_TO_DIMOS @ np.array([pose.x, pose.y, pose.z], dtype=np.float64)
    habitat_rotation = Rotation.from_quat([pose.qx, pose.qy, pose.qz, pose.qw])
    rotation = _HABITAT_TO_DIMOS_ROTATION * habitat_rotation * _HABITAT_TO_DIMOS_ROTATION.inv()
    return position, rotation.as_quat()


def _base_from_camera(pose: pb.Pose, timestamp: float) -> Transform:
    translation = _HABITAT_TO_DIMOS @ np.array([pose.x, pose.y, pose.z], dtype=np.float64)
    habitat_rotation = Rotation.from_quat([pose.qx, pose.qy, pose.qz, pose.qw])
    rotation = (_HABITAT_TO_DIMOS_ROTATION * habitat_rotation).as_quat()
    return Transform(
        translation=Vector3(translation),
        rotation=Quaternion(rotation),
        frame_id="base_link",
        child_frame_id="camera_optical",
        ts=timestamp,
    )


def _decode_occupancy(occupancy: pb.OccupancyMap, timestamp: float) -> OccupancyGrid:
    habitat = np.frombuffer(occupancy.traversability, dtype=np.uint8)
    if habitat.size != occupancy.width * occupancy.height:
        raise VlnceConnectionError("public occupancy payload size is invalid")
    if not np.all((habitat == 0) | (habitat == 1)):
        raise VlnceConnectionError("public occupancy contains non-geometric values")
    habitat = habitat.reshape(occupancy.height, occupancy.width)
    traversability = habitat[::-1, ::-1].T
    grid = np.where(
        traversability == 1,
        int(CostValues.FREE),
        int(CostValues.OCCUPIED),
    ).astype(np.int8)
    upper_x = occupancy.origin.x + occupancy.width * occupancy.resolution
    upper_z = occupancy.origin.z + occupancy.height * occupancy.resolution
    return OccupancyGrid(
        grid=grid,
        resolution=occupancy.resolution,
        origin=Pose(position=[-upper_z, -upper_x, 0.0]),
        frame_id="world",
        ts=timestamp,
    )
