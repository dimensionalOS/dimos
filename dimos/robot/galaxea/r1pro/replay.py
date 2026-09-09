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

"""Drive the R1 Pro navigation stack from a recording instead of the robot.

Stands in for :class:`R1ProConnection`: same output stream names, same frames,
so everything downstream is unchanged. One :class:`Replay` backs every stream,
so lidar, depth and tf advance against a single wall-clock anchor.

Record from ``r1pro-coordinator``, not from a running ``r1pro-kronknav``: under
kronknav ``lidar`` is a fan-in bus that already carries the head camera's cloud,
so replaying it would feed those points in twice, once recorded and once
rebuilt from ``head_depth``.
"""

from __future__ import annotations

from typing import Any

import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.replay import Replay, resolve_db_path
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.galaxea.r1pro.constants import HEAD_CAMERA_LINK
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Port name -> db stream names to try, in order. The generic recorder slugs the
# zenoh topic (``/r1pro/motor_states`` -> ``r1pro_motor_states``) while a
# Recorder subclass names each stream after its port, so both are accepted.
STREAM_CANDIDATES = {
    "lidar": ("lidar",),
    "head_depth": ("head_depth",),
    "head_camera_info": ("head_camera_info",),
    "motor_states": ("r1pro_motor_states", "motor_states"),
    "chassis_odom": ("chassis_odom", "odom"),
    "tf": ("tf",),
}

CAMERA_INFO_REPUBLISH_S = 1.0


class R1ProReplayConfig(ModuleConfig):
    # Recording stem or path; `.db`, LFS-fetched on miss.
    dataset: str = ""
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    loop: bool = False
    # Port name -> db stream name, for a recording that spells one differently
    # from every entry in STREAM_CANDIDATES.
    stream_remapping: dict[str, str] = {}
    # Recordings made before the connection published intrinsics carry no
    # camera_info stream; point this at the ROS camera_info YAML for the robot
    # that made the recording to supply them.
    head_camera_info_path: str = ""
    head_camera_frame_id: str = HEAD_CAMERA_LINK


class R1ProReplay(Module):
    """Replay an R1 Pro recording onto the streams the live connection drives."""

    config: R1ProReplayConfig

    lidar: Out[PointCloud2]
    head_depth: Out[Image]
    head_camera_info: Out[CameraInfo]
    motor_states: Out[JointState]
    chassis_odom: Out[PoseStamped]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()

        if not self.config.dataset:
            raise ValueError("R1ProReplay needs a recording: pass --dataset <stem-or-path-to-.db>")

        path = resolve_db_path(self.config.dataset)
        store = self.register_disposable(SqliteStore(path=str(path), must_exist=True))
        store.start()
        replay: Replay = store.replay(
            speed=self.config.speed,
            seek=self.config.seek,
            duration=self.config.duration,
            loop=self.config.loop,
        )

        available = set(replay.list_streams())
        played = [
            name for name in STREAM_CANDIDATES if self._play(replay, available, name) is not None
        ]
        logger.info(
            "Replaying %s at %sx: %s (recording holds %s)",
            path.name,
            self.config.speed,
            ", ".join(played) or "nothing",
            ", ".join(sorted(available)),
        )

        if "head_camera_info" not in played:
            self._publish_camera_info_from_yaml()

    def _play(self, replay: Replay, available: set[str], port_name: str) -> str | None:
        """Wire the recording's stream for *port_name* to that Out, if present."""
        override = self.config.stream_remapping.get(port_name)
        candidates = (override,) if override else STREAM_CANDIDATES[port_name]
        name = next((candidate for candidate in candidates if candidate in available), None)
        if name is None:
            logger.warning(
                "%s: no %s stream in the recording (tried %s), so %s publishes nothing",
                type(self).__name__,
                port_name,
                ", ".join(candidates),
                port_name,
            )
            return None

        port: Out[Any] = getattr(self, port_name)
        self.register_disposable(replay.stream(name).observable().subscribe(port.publish))
        return name

    def _publish_camera_info_from_yaml(self) -> None:
        if not self.config.head_camera_info_path:
            logger.warning(
                "%s: the recording has no head_camera_info and no "
                "--head-camera-info-path was given, so the depth cloud will never build. "
                "Pass the camera_info YAML for the robot that made the recording.",
                type(self).__name__,
            )
            return

        info = CameraInfo.from_yaml(
            self.config.head_camera_info_path, frame_id=self.config.head_camera_frame_id
        )
        # Intrinsics are static, but Out streams don't latch, so a consumer that
        # connects late still needs to see one.
        self.register_disposable(
            rx.interval(CAMERA_INFO_REPUBLISH_S).subscribe(
                lambda _tick: self.head_camera_info.publish(info)
            )
        )
