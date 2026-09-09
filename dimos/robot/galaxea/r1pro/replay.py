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

Every frame comes from the recording's own ``tf`` stream, including the head
camera edge the connection derives from joint angles. A recording made before
the connection published that edge replays without it, and the head cloud then
has no transform to resolve.

Intrinsics come from the recording too. Galaxea publishes no camera_info, so
the connection loads the robot's calibration YAML and puts it on
``head_camera_info``; a recording that skipped that stream is a recording to
redo, not something to patch a second YAML onto here.
"""

from __future__ import annotations

from typing import Any

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
from dimos.robot.galaxea.r1pro.topics import recorded_stream_name
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class R1ProReplayConfig(ModuleConfig):
    # Recording stem or path; `.db`, LFS-fetched on miss.
    dataset: str = ""
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    loop: bool = False
    # Port name -> db stream name, for a recording that spells one differently
    # from both names `_stream_candidates` derives.
    stream_remapping: dict[str, str] = {}


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
            port_name
            for port_name, port in self.outputs.items()
            if self._play(replay, available, port_name, port)
        ]
        logger.info(
            "Replaying %s at %sx: %s (recording holds %s)",
            path.name,
            self.config.speed,
            ", ".join(played) or "nothing",
            ", ".join(sorted(available)),
        )

    def _stream_candidates(self, port_name: str) -> tuple[str, ...]:
        """Stream names to look for, in order.

        The generic recorder names a stream after the zenoh topic it subscribed
        to, a Recorder subclass after the port, so both spellings are accepted.
        """
        override = self.config.stream_remapping.get(port_name)
        if override:
            return (override,)
        slug = recorded_stream_name(port_name)
        return (slug,) if slug == port_name else (slug, port_name)

    def _play(self, replay: Replay, available: set[str], port_name: str, port: Out[Any]) -> bool:
        """Wire the recording's stream for *port_name* to that Out, if present."""
        candidates = self._stream_candidates(port_name)
        name = next((candidate for candidate in candidates if candidate in available), None)
        if name is None:
            logger.warning(
                "%s: no %s stream in the recording (tried %s), so %s publishes nothing",
                type(self).__name__,
                port_name,
                ", ".join(candidates),
                port_name,
            )
            return False

        self.register_disposable(replay.stream(name).observable().subscribe(port.publish))
        return True
