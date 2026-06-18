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

"""A Recorder that also records the tf tree and lets subclasses set per-stream poses.

Decorate a method with ``@pose_setter_for("stream")`` to control how a recorded
stream's pose is resolved; streams without a setter fall back to the base
Recorder's tf-based ``world <- frame_id`` lookup::

    class MyRecorder(TfRecorder):
        odometry: In[Odometry]
        lidar: In[PointCloud2]

        @pose_setter_for("odometry")
        def _odom_pose(self, msg):
            self._last_pose = msg.pose.pose
            return self._last_pose

        @pose_setter_for("lidar")
        def _lidar_pose(self, msg):
            return self._last_pose  # stamp the cloud with the latest odom pose
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PoseSetter = Callable[[Any], "Pose | None"]


def pose_setter_for(*stream_names: str) -> Callable[[Any], Any]:
    """Mark a method ``(self, msg) -> Pose | None`` as the pose setter for the
    given recorded stream(s)."""

    def decorate(fn: Any) -> Any:
        fn._pose_setter_for = tuple(stream_names)
        return fn

    return decorate


class TfRecorderConfig(RecorderConfig):
    # Also record the live tf stream alongside the In ports.
    record_tf: bool = True


class TfRecorder(Recorder):
    config: TfRecorderConfig

    _pose_setters: dict[str, PoseSetter] = {}

    @rpc
    def start(self) -> None:
        self._pose_setters = self._collect_pose_setters()
        super().start()
        if self.config.g.replay:
            return
        if self.config.record_tf:
            self._record_tf()

    def _collect_pose_setters(self) -> dict[str, PoseSetter]:
        """Map stream name -> bound @pose_setter_for method."""
        setters: dict[str, PoseSetter] = {}
        for attr_name in dir(type(self)):
            fn = getattr(type(self), attr_name, None)
            for stream in getattr(fn, "_pose_setter_for", ()):
                setters[stream] = getattr(self, attr_name)
        return setters

    def _resolve_pose(self, name: str, msg: Any, ts: float) -> Pose | None:
        """Dispatch to the stream's @pose_setter_for, else the base tf lookup."""
        setter = self._pose_setters.get(name)
        if setter is not None:
            return setter(msg)
        return super()._resolve_pose(name, msg, ts)

    def _record_tf(self) -> None:
        topic = getattr(self.tf.config, "topic", None)
        if not topic:
            logger.warning("TfRecorder: no tf topic configured — not recording tf")
            return
        tf_stream = self.store.stream("tf", TFMessage)

        def on_tf(msg: TFMessage, _topic: object) -> None:
            try:
                for transform in msg.transforms:
                    tf_stream.append(TFMessage(transform), ts=transform.ts, pose=None)
            except Exception:
                # A late LCM callback during teardown can hit an already-closed
                # store; tf is a best-effort quick-look stream, so drop it.
                pass

        self.register_disposable(Disposable(self.tf.pubsub.subscribe(topic, on_tf)))
