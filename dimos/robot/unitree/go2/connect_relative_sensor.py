# Copyright 2025-2026 Dimensional Inc.
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

"""Standalone GO2Connection that re-expresses the Go2's onboard lidar.

A full copy of ``connection.py`` (not a subclass) so it can diverge freely from
the base connection. The only behavioural change is how the lidar cloud is
published. The Go2's onboard stack transforms its lidar into the odom/world
frame and accumulates scans. This undoes both: it applies the inverse of the
robot's current world pose so points land back in ``base_link``, and with
``un_accumulate`` on it subtracts the prior cloud (in the stable world frame,
where accumulated points keep identical coordinates) so only the new points are
published.

An optional rigid ``transform`` (row-major 4x4, 16 floats) is applied to the
base_link cloud before publishing so it can be re-expressed as if it came from
another sensor (e.g. mid360_link).
"""

from __future__ import annotations

from enum import Enum
from importlib import resources
import sys
from threading import Thread
import time
from typing import Any, Protocol

import numpy as np
from pydantic import Field, field_validator
from reactivex import empty
from reactivex.disposable import Disposable
from reactivex.observable import Observable
import rerun.blueprint as rrb

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.resource import CompositeResource
from dimos.core.stream import In, Out
from dimos.memory2.replay import Replay, resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.type.lowstate import LowStateMsg
from dimos.spec.perception import Camera, Pointcloud
from dimos.utils.decorators.decorators import cached_property, simple_mcache
from dimos.utils.logging_config import setup_logger

if sys.version_info < (3, 13):
    from typing_extensions import TypeVar
else:
    from typing import TypeVar

logger = setup_logger()


def _rows_not_in(points: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Rows of ``points`` (Nx3) that aren't byte-identical rows of ``reference``."""
    if reference.size == 0:
        return points
    row_dtype = np.dtype([("", points.dtype)] * points.shape[1])
    points_view = np.ascontiguousarray(points).view(row_dtype).ravel()
    reference_view = np.ascontiguousarray(reference).view(row_dtype).ravel()
    return points[~np.isin(points_view, reference_view)]


class Go2Mode(str, Enum):
    DEFAULT = "default"
    RAGE = "rage"


class RelativeSensorConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)
    mode: Go2Mode = Go2Mode.DEFAULT
    lidar: bool = True
    camera: bool = True
    # "mcf" for stair traversal, "normal" for basic, None to leave it as is
    motion_mode: str | None = None
    # Per-device AES-128 key (Go2 fw >=1.1.15); defaults from GlobalConfig.
    aes_128_key: str | None = Field(default_factory=lambda m: m["g"].unitree_aes_128_key)
    # TF parent frame of the internal odometry (odom_frame_id -> base_frame).
    # Rename (e.g. "go2_odom") when another odom source owns the tree root
    odom_frame_id: str = "world"
    # Body frame this connection publishes its TF + odom under, and the frame the
    # onboard cloud is landed in before any rigid transform. Rename (e.g.
    # "go2_base_link") to keep this subtree from colliding with another odometry
    # source (Point-LIO) that owns its own base_link.
    base_frame: str = "base_link"
    # Frame the published lidar cloud is stamped with. None keeps base_frame; set
    # it to re-express the cloud as if it came from another sensor (e.g.
    # "mid360_link" so Point-LIO sees the L1 lidar in the frame it expects).
    lidar_frame: str | None = None
    # Optional rigid transform applied to the base_frame cloud before publishing:
    # row-major 4x4 (16 floats), None = identity. Pair with lidar_frame to place
    # the cloud at another sensor's mount.
    transform: list[float] | None = None
    # Subtract the previous (accumulated) cloud and publish only the new points.
    un_accumulate: bool = True

    @field_validator("transform")
    @classmethod
    def _validate_transform(cls, value: list[float] | None) -> list[float] | None:
        if value is not None and len(value) != 16:
            raise ValueError(f"transform must be a row-major 4x4 (16 floats), got {len(value)}")
        return value


class Go2ConnectionProtocol(Protocol):
    """Protocol defining the interface for Go2 robot connections."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def lidar_stream(self) -> Observable[PointCloud2]: ...
    def odom_stream(self) -> Observable[PoseStamped]: ...
    def video_stream(self) -> Observable[Image]: ...
    def lowstate_stream(self) -> Observable[LowStateMsg]: ...
    def move(self, twist: Twist, duration: float = 0.0) -> bool: ...
    def standup(self) -> bool: ...
    def liedown(self) -> bool: ...
    def balance_stand(self) -> bool: ...
    def set_obstacle_avoidance(self, enabled: bool = True) -> None: ...
    def set_rage_mode(self, enable: bool) -> bool: ...
    def publish_request(self, topic: str, data: dict) -> dict: ...  # type: ignore[type-arg]


_FRONT_CAMERA_720_YAML = resources.files("dimos.robot.unitree.go2").joinpath(
    "front_camera_720.yaml"
)


def _camera_info_static() -> CameraInfo:
    with resources.as_file(_FRONT_CAMERA_720_YAML) as yaml_path:
        return CameraInfo.from_yaml(str(yaml_path))


# Static camera mount chain: base_link -> camera_link -> camera_optical.
# TODO we need a standardized way to specify this for all cameras in dimos
BASE_TO_OPTICAL: Transform = Transform(
    translation=Vector3(0.3, 0.0, 0.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="base_link",
    child_frame_id="camera_link",
) + Transform(
    translation=Vector3(0.0, 0.0, 0.0),
    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
    frame_id="camera_link",
    child_frame_id="camera_optical",
)


def make_connection(
    ip: str | None,
    cfg: GlobalConfig,
    aes_128_key: str | None = None,
) -> Go2ConnectionProtocol:
    connection_type = cfg.unitree_connection_type.lower()

    if ip in ("fake", "mock", "replay") or connection_type == "replay":
        dataset = cfg.replay_db
        return ReplayConnection(dataset=dataset)
    elif ip == "mujoco" or connection_type in ("mujoco", "true"):
        from dimos.robot.unitree.mujoco_connection import MujocoConnection

        return MujocoConnection(cfg)
    elif connection_type == "dimsim":
        from dimos.robot.unitree.dimsim_connection import DimSimConnection

        return DimSimConnection(cfg)
    elif connection_type == "webrtc":
        assert ip is not None, "IP address must be provided"
        return UnitreeWebRTCConnection(ip, aes_128_key=aes_128_key)
    else:
        raise ValueError(f"Unknown simulator {cfg.simulation!r}. Choose from: mujoco, dimsim")


class ReplayConnection(UnitreeWebRTCConnection, CompositeResource):
    def __init__(  # type: ignore[no-untyped-def]
        self,
        dataset: str = "go2_china_office",
        **kwargs,
    ) -> None:
        self.dataset = dataset
        self._loop = kwargs.get("loop", False)
        self._seek = kwargs.get("seek")
        self._duration = kwargs.get("duration")

    @cached_property
    def replay(self) -> Replay:
        # One shared store + Replay so lidar/odom/video advance against the
        # same wall-clock anchor on subscribe.
        store = self.register_disposable(
            SqliteStore(path=str(resolve_db_path(self.dataset)), must_exist=True)
        )
        store.start()
        return store.replay(loop=self._loop, seek=self._seek, duration=self._duration)

    def connect(self) -> None:
        pass

    def start(self) -> None:
        pass

    def standup(self) -> bool:
        return True

    def liedown(self) -> bool:
        return True

    def balance_stand(self) -> bool:
        return True

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        pass

    def set_motion_mode(self, name: str) -> None:
        pass

    def set_rage_mode(self, enable: bool) -> bool:
        return True

    @simple_mcache
    def lidar_stream(self) -> Observable[PointCloud2]:
        return self.replay.streams.lidar.observable()

    @simple_mcache
    def odom_stream(self) -> Observable[PoseStamped]:
        return self.replay.streams.odom.observable()

    @simple_mcache
    def video_stream(self) -> Observable[Image]:
        return self.replay.streams.color_image.observable()

    @simple_mcache
    def lowstate_stream(self) -> Observable:  # type: ignore[type-arg]
        # Replay datasets carry no low-level state (battery/IMU) — emit nothing.
        return empty()

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        return True

    def publish_request(self, topic: str, data: dict):  # type: ignore[no-untyped-def, type-arg]
        """Fake publish request for testing."""
        return {"status": "ok", "message": "Fake publish"}


_Config = TypeVar("_Config", bound=RelativeSensorConfig, default=RelativeSensorConfig)


class RelativeSensorConnection(Module, Camera, Pointcloud):
    dedicated_worker = True

    config: RelativeSensorConfig
    cmd_vel: In[Twist]
    # Point-LIO's Mid-360 cloud (topic `pointlio_lidar`). Its most recent frame is
    # merged into the re-expressed L1 cloud before publishing on `lidar`.
    pointlio_lidar: In[PointCloud2]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    lidar_relative: Out[PointCloud2]
    lidar_full: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    connection: Go2ConnectionProtocol
    camera_info_static: CameraInfo = _camera_info_static()
    _camera_info_thread: Thread | None = None
    _latest_video_frame: Image | None = None
    _latest_lowstate: LowStateMsg | None = None

    _latest_pose: PoseStamped | None = None
    _transform: Transform | None = None
    _previous_points: np.ndarray | None = None
    _latest_pointlio_cloud: PointCloud2 | None = None

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        """Return Rerun view blueprints for GO2 camera visualization."""
        return [
            rrb.Spatial2DView(
                name="Camera",
                origin="world/robot/camera/rgb",
            ),
        ]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.connection = make_connection(
            self.config.ip, self.config.g, aes_128_key=self.config.aes_128_key
        )

        if hasattr(self.connection, "camera_info_static"):
            self.camera_info_static = self.connection.camera_info_static

        if self.config.transform is not None:
            matrix = np.array(self.config.transform, dtype=float).reshape(4, 4)
            lidar_frame = self.config.lidar_frame or self.config.base_frame
            self._transform = Transform.from_matrix(
                matrix, frame_id=lidar_frame, child_frame_id=self.config.base_frame
            )

    @rpc
    def start(self) -> None:
        super().start()
        if not hasattr(self, "connection"):
            return
        self.connection.start()

        def onimage(image: Image) -> None:
            self.color_image.publish(image)
            self._latest_video_frame = image

        if self.config.lidar:
            self.register_disposable(self.connection.lidar_stream().subscribe(self._publish_lidar))
            self.register_disposable(
                Disposable(self.pointlio_lidar.subscribe(self._on_pointlio_lidar))
            )
        self.register_disposable(self.connection.odom_stream().subscribe(self._publish_tf))
        self.register_disposable(self.connection.lowstate_stream().subscribe(self._on_lowstate))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

        if self.config.camera:
            self.register_disposable(self.connection.video_stream().subscribe(onimage))
            self._camera_info_thread = Thread(
                target=self.publish_camera_info,
                daemon=True,
            )
            self._camera_info_thread.start()

        if self.config.motion_mode and isinstance(self.connection, UnitreeWebRTCConnection):
            self.connection.set_motion_mode(self.config.motion_mode)

        self.standup()
        time.sleep(3)
        self.connection.balance_stand()

        if self.config.mode == Go2Mode.RAGE:
            self.connection.set_rage_mode(True)

        self.connection.set_obstacle_avoidance(self.config.g.obstacle_avoidance)

    @rpc
    def stop(self) -> None:
        self.liedown()

        if self.connection:
            self.connection.stop()

        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        super().stop()

    def _odom_to_tf(self, odom: PoseStamped) -> list[Transform]:
        base_frame = self.config.base_frame
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=base_frame,
            child_frame_id="camera_link",
            ts=odom.ts,
        )

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=odom.ts,
        )

        return [
            Transform.from_pose(base_frame, odom),
            camera_link,
            camera_optical,
        ]

    def _publish_tf(self, msg: PoseStamped) -> None:
        self._latest_pose = msg
        msg.frame_id = self.config.odom_frame_id
        transforms = self._odom_to_tf(msg)
        self.tf.publish(*transforms)
        if self.odom.transport:
            self.odom.publish(msg)

    def _publish_lidar(self, cloud: PointCloud2) -> None:
        pose = self._latest_pose
        if pose is None:
            return
        self.lidar_full.publish(cloud)
        if self.config.un_accumulate:
            cloud = self._only_new_points(cloud)
        # from_pose gives the base frame's pose in world; its inverse maps the
        # world cloud back into the base frame.
        world_to_base = Transform.from_pose(self.config.base_frame, pose).inverse()
        base_cloud = cloud.transform(world_to_base)
        if self._transform is not None:
            base_cloud = base_cloud.transform(self._transform)
        base_cloud.frame_id = self.config.lidar_frame or self.config.base_frame
        self.lidar_relative.publish(base_cloud)
        self.lidar.publish(self._merge_with_pointlio(base_cloud))

    def _merge_with_pointlio(self, cloud: PointCloud2) -> PointCloud2:
        """Concatenate the latest Point-LIO cloud with the re-expressed L1 cloud.

        Both are stamped in the Mid-360 frame here (lidar_frame="mid360_link"),
        so the points share a frame and can be stacked directly.
        """
        latest = self._latest_pointlio_cloud
        if latest is None:
            return cloud
        merged = np.vstack((latest.points_f32(), cloud.points_f32()))
        return PointCloud2.from_numpy(merged, frame_id=cloud.frame_id, timestamp=cloud.ts)

    def _on_pointlio_lidar(self, cloud: PointCloud2) -> None:
        """Cache the most recent Point-LIO (Mid-360) cloud for the next L1 merge."""
        self._latest_pointlio_cloud = cloud

    def _only_new_points(self, cloud: PointCloud2) -> PointCloud2:
        points = cloud.points_f32()
        previous = self._previous_points
        self._previous_points = points
        if previous is None:
            return cloud
        new_points = _rows_not_in(points, previous)
        return PointCloud2.from_numpy(new_points, frame_id=cloud.frame_id, timestamp=cloud.ts)

    def publish_camera_info(self) -> None:
        while True:
            self.camera_info.publish(self.camera_info_static)
            time.sleep(1.0)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to robot."""
        return self.connection.move(twist, duration)

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up."""
        return self.connection.standup()

    @rpc
    def liedown(self) -> bool:
        """Make the robot lie down."""
        return self.connection.liedown()

    @rpc
    def balance_stand(self) -> bool:
        """Enter BalanceStand: neutral state for switching locomotion modes"""
        return self.connection.balance_stand()

    @rpc
    def set_rage_mode(self, enable: bool) -> bool:
        """Toggle Rage Mode on/off (~2.5 m/s envelope when on).
        On the WebRTC backend this re-establishes the BalanceStand
        precondition before toggling; sim backends are no-ops.
        """
        result = self.connection.set_rage_mode(enable)
        logger.info("Rage Mode", enabled=enable)
        return result

    def _on_lowstate(self, msg: LowStateMsg) -> None:
        """Cache the latest low-level state push (battery, IMU, motors, etc.)."""
        self._latest_lowstate = msg

    @skill
    def get_battery_soc(self) -> int | None:
        """Returns the robot's battery state-of-charge as a percentage (0-100).

        Use this skill to answer battery / power / charge questions. Returns
        None if no low-level state has been received yet.
        """
        try:
            return int(self._latest_lowstate["data"]["bms_state"]["soc"])  # type: ignore[index]
        except (KeyError, TypeError, ValueError):
            return None

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        """Publish a request to the WebRTC connection.
        Args:
            topic: The RTC topic to publish to
            data: The data dictionary to publish
        Returns:
            The result of the publish request
        """
        return self.connection.publish_request(topic, data)

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the robot camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame
