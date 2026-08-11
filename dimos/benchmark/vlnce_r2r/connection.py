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

from collections.abc import Iterator
import math
from queue import Queue
from threading import Event, Lock, Thread
import time
from typing import Any

import grpc  # type: ignore[import-untyped]
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
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
from .protocol.contract import expected_handshake
from .protocol.vlnce_public_v1_pb2_grpc import VlncePublicGatewayStub

logger = setup_logger()

_CLOSED = object()
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
    attempt_id: str
    case_id: str
    episode_id: str
    protocol_revision: str = "vlnce-public.v1"
    connect_timeout_seconds: float = Field(default=30.0, gt=0.0)
    command_queue_capacity: int = Field(default=2, ge=1, le=8)


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
        self._requests: Queue[pb.ClientMessage | object] = Queue(
            maxsize=self.config.command_queue_capacity
        )
        self._expected = expected_handshake(
            {
                "attempt_id": self.config.attempt_id,
                "case_id": self.config.case_id,
                "episode_id": self.config.episode_id,
                "protocol_revision": self.config.protocol_revision,
            }
        )
        self._ready = Event()
        self._begun = Event()
        self._submitted = Event()
        self._closed = Event()
        self._command_ready = Event()
        self._state_lock = Lock()
        self._state = "created"
        self._last_observation_sequence = 0
        self._next_command_sequence = 1
        self._observation_count = 0
        self._accepted_control_count = 0
        self._rejected_control_count = 0
        self._last_control: dict[str, float] | None = None
        self._failure: Exception | None = None
        self._channel: grpc.Channel | None = None
        self._thread: Thread | None = None
        self._wall_minus_monotonic = time.time() - time.monotonic()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self._channel = grpc.insecure_channel(f"unix://{self.config.socket_path}")
        self._thread = Thread(target=self._run_stream, name="vlnce-public-stream", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        try:
            if self._state in {"ready", "running"}:
                self.cancel()
            self._enqueue(_CLOSED)
            if self._thread is not None:
                self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if self._channel is not None:
                self._channel.close()
        finally:
            super().stop()

    @rpc
    def wait_ready(self, timeout_seconds: float | None = None) -> bool:
        """Wait for an exact compatible public handshake."""

        timeout = timeout_seconds or self.config.connect_timeout_seconds
        if not self._ready.wait(timeout=timeout):
            self._raise_if_failed("benchmark gateway did not become ready")
            raise VlnceConnectionError("benchmark gateway did not become ready")
        self._raise_if_failed("benchmark gateway failed during readiness")
        return True

    @rpc
    def begin(self) -> bool:
        """Begin the episode and wait for its first coherent public observation."""

        self.wait_ready()
        with self._state_lock:
            if self._state != "ready":
                raise VlnceConnectionError("episode can begin only once after readiness")
            self._state = "starting"
        self._enqueue(
            pb.ClientMessage(lifecycle=pb.LifecycleCommand(kind=pb.LifecycleCommand.BEGIN))
        )
        if not self._begun.wait(timeout=self.config.connect_timeout_seconds):
            self._raise_if_failed("benchmark did not publish its initial observation")
            raise VlnceConnectionError("benchmark did not publish its initial observation")
        self._raise_if_failed("benchmark failed while beginning the episode")
        return True

    @rpc
    def cancel(self) -> bool:
        """Cancel transport without submitting a route for official scoring."""

        with self._state_lock:
            if self._state not in {"ready", "starting", "running"}:
                return False
            self._state = "cancelled"
        self._enqueue(
            pb.ClientMessage(lifecycle=pb.LifecycleCommand(kind=pb.LifecycleCommand.CANCEL))
        )
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
            abs(twist.linear.x) > self._expected.max_linear_x
            or abs(twist.linear.y) > self._expected.max_linear_y
            or abs(twist.angular.z) > self._expected.max_angular_z
        ):
            raise VlnceConnectionError("planar velocity exceeds the negotiated limit")
        with self._state_lock:
            if self._state != "running" or not self._command_ready.is_set():
                self._rejected_control_count += 1
                return False
            command_sequence = self._next_command_sequence
            self._next_command_sequence += 1
            observation_sequence = self._last_observation_sequence
            self._command_ready.clear()
            self._accepted_control_count += 1
            self._last_control = {
                "linear_x": twist.linear.x,
                "linear_y": twist.linear.y,
                "angular_z": twist.angular.z,
            }
        self._enqueue(
            pb.ClientMessage(
                control=pb.PlanarControl(
                    command_sequence=command_sequence,
                    observation_sequence=observation_sequence,
                    linear_x=twist.linear.x,
                    linear_y=twist.linear.y,
                    angular_z=twist.angular.z,
                )
            )
        )
        return True

    @skill
    def submit_route(self) -> str:
        """Submit VLN-CE STOP once and end this evaluation.

        Call only when the assigned route is complete. Submission is irreversible
        and acknowledges only that the route was submitted; it never reveals score,
        success, distance, or benchmark progress.
        """

        with self._state_lock:
            if self._state != "running" or not self._command_ready.is_set():
                raise VlnceConnectionError("route can be submitted only from a settled episode")
            self._state = "submitting"
            command_sequence = self._next_command_sequence
            self._next_command_sequence += 1
            observation_sequence = self._last_observation_sequence
            self._command_ready.clear()
        self._enqueue(
            pb.ClientMessage(
                submit_route=pb.SubmitRoute(
                    command_sequence=command_sequence,
                    observation_sequence=observation_sequence,
                )
            )
        )
        if not self._submitted.wait(timeout=self.config.connect_timeout_seconds):
            self._raise_if_failed("route submission was not acknowledged")
            raise VlnceConnectionError("route submission was not acknowledged")
        self._raise_if_failed("benchmark failed during route submission")
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
                    "world": self._expected.world_frame,
                    "base": self._expected.base_frame,
                    "camera": self._expected.camera_frame,
                },
                "control": {
                    "period_seconds": self._expected.control_period_seconds,
                    "max_linear_x": self._expected.max_linear_x,
                    "max_linear_y": self._expected.max_linear_y,
                    "max_angular_z": self._expected.max_angular_z,
                },
            }

    def _on_cmd_vel(self, twist: Twist) -> None:
        if not self.move(twist):
            logger.debug("VLN-CE command rejected while another command is in flight")

    def _request_iterator(self) -> Iterator[pb.ClientMessage]:
        yield pb.ClientMessage(handshake=self._expected)
        while True:
            request = self._requests.get()
            if request is _CLOSED:
                return
            if not isinstance(request, pb.ClientMessage):
                raise VlnceConnectionError("invalid internal request")
            yield request

    def _run_stream(self) -> None:
        try:
            if self._channel is None:
                raise VlnceConnectionError("gRPC channel was not initialized")
            responses = VlncePublicGatewayStub(self._channel).Stream(self._request_iterator())
            for response in responses:
                payload = response.WhichOneof("payload")
                if payload == "ready":
                    if response.ready.negotiated != self._expected:
                        raise VlnceConnectionError("gateway negotiated an incompatible contract")
                    with self._state_lock:
                        if self._state != "created":
                            raise VlnceConnectionError("duplicate gateway readiness")
                        self._state = "ready"
                    self._ready.set()
                elif payload == "observation":
                    self._publish_observation(response.observation)
                elif payload == "acknowledgement":
                    self._handle_acknowledgement(response.acknowledgement)
                elif payload == "error":
                    raise VlnceConnectionError(response.error.message)
                else:
                    raise VlnceConnectionError("gateway response has no public payload")
        except Exception as error:
            self._failure = error
            self._ready.set()
            self._begun.set()
            self._submitted.set()
        finally:
            self._closed.set()

    def _handle_acknowledgement(self, acknowledgement: pb.Acknowledgement) -> None:
        if acknowledgement.kind == pb.Acknowledgement.ROUTE_SUBMITTED:
            with self._state_lock:
                if self._state != "submitting":
                    raise VlnceConnectionError("unexpected route-submission acknowledgement")
                self._state = "submitted"
            self._submitted.set()

    def _publish_observation(self, observation: pb.Observation) -> None:
        if observation.sequence <= self._last_observation_sequence:
            raise VlnceConnectionError("observation sequence did not increase")
        if (
            observation.world_frame != self._expected.world_frame
            or observation.base_frame != self._expected.base_frame
            or observation.camera_frame != self._expected.camera_frame
            or observation.rgb_encoding != self._expected.rgb_encoding
            or observation.depth_encoding != self._expected.depth_encoding
        ):
            raise VlnceConnectionError("observation frames or encodings changed after negotiation")
        native = decode_observation(observation, self._wall_minus_monotonic)
        self.color_image.publish(native["color_image"])
        self.depth_image.publish(native["depth_image"])
        self.camera_info.publish(native["camera_info"])
        self.depth_camera_info.publish(native["depth_camera_info"])
        self.pointcloud.publish(native["pointcloud"])
        self.odometry.publish(native["odometry"])
        self.odom.publish(native["odom"])
        self.tf.publish(native["tf"])
        occupancy = native["global_costmap"]
        if occupancy is not None:
            self.global_costmap.publish(occupancy)
        with self._state_lock:
            self._last_observation_sequence = observation.sequence
            self._observation_count += 1
            if self._state == "starting":
                self._state = "running"
                self._begun.set()
            self._command_ready.set()

    def _enqueue(self, request: pb.ClientMessage | object) -> None:
        try:
            self._requests.put(request, timeout=self.config.connect_timeout_seconds)
        except Exception as error:
            raise VlnceConnectionError("public command queue is unavailable") from error

    def _raise_if_failed(self, message: str) -> None:
        if self._failure is not None:
            raise VlnceConnectionError(message) from self._failure


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
    if occupancy.encoding != "uint8_traversable":
        raise VlnceConnectionError("unsupported public occupancy encoding")
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
