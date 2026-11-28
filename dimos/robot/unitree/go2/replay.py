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

"""A Go2 recording replayed with the ports GO2Connection would publish live."""

from __future__ import annotations

from threading import Lock
import time
from typing import Any

from pydantic import Field, model_validator
from reactivex import interval
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.memory.replay_module import ReplayModule, ReplayModuleConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
from dimos.utils.generic import classproperty


class Go2ReplayConfig(ConnectionConfig, ReplayModuleConfig):
    # GO2Connection's settings, so its blueprint arguments and CLI flags still apply.
    dataset: str = Field(default_factory=lambda m: m["g"].replay_db)

    @model_validator(mode="after")
    def _go2_replay_flags(self) -> Go2ReplayConfig:
        self.loop = self.loop or self.replay_loop
        self.exit_on_end = self.exit_on_end or self.replay_exit
        return self


class Go2Replay(ReplayModule):
    """Replays a Go2 recording's lidar, odom and camera, and derives tf, odometry and camera_info.

    Reads streams lidar, odom and color_image; mid360 recordings name the first two
    go2_lidar and go2_odom, and recordings made after the odom -> odometry rename
    hold the pose as go2_odometry or odometry (nav_msgs.Odometry) instead of odom.

    ``GO2Connection.blueprint()`` returns this under ``--replay``, under GO2Connection's name.
    """

    config: Go2ReplayConfig
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    recorded_odometry: Out[Odometry]
    color_image: Out[Image]
    odometry: Out[Odometry]
    joint_state: Out[JointState]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]
    cmd_vel: In[Twist]
    velocity_command: In[JointState]

    @classproperty
    def name(cls) -> str:  # noqa: N805
        # GO2Connection's name: its remaps and --go2connection.* flags apply to this.
        return GO2Connection.name  # type: ignore[no-any-return]

    _publish_from_odom = GO2Connection._publish_from_odom
    _odom_to_tf = GO2Connection._odom_to_tf

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._odom_lock = Lock()
        self._previous_odom: PoseStamped | None = None

    def _stream_name(self, port: str, available: set[str]) -> str | None:
        recorded_odometry = next((n for n in ("go2_odometry", "odometry") if n in available), None)
        if port == "recorded_odometry":
            return recorded_odometry
        if port == "odom" and recorded_odometry is not None:
            return None  # the recorded odometry carries the same poses
        if port not in ("lidar", "odom", "color_image"):
            return None  # derived here, not replayed
        recorded = f"go2_{port}"
        return recorded if recorded in available else port

    def _start(self) -> None:
        super()._start()
        # GO2Connection's own method: this class has the ports and config it uses.
        self.register_disposable(Disposable(self.odom.subscribe(self._publish_from_odom)))  # type: ignore[misc]
        self.register_disposable(
            Disposable(
                self.recorded_odometry.subscribe(
                    lambda msg: self._publish_from_odom(msg.to_pose_stamped())  # type: ignore[misc]
                )
            )
        )
        if self.config.camera:
            info = GO2Connection.camera_info_static
            self.register_disposable(
                interval(1.0).subscribe(
                    lambda _: self.camera_info.publish(info.with_ts(time.time()))
                )
            )

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]:
        """A recording takes no requests."""
        return {"status": "ok", "message": "replay"}

    @rpc
    def battery_soc(self) -> int | None:
        return None
