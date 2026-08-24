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

"""cuVSLAM fed by a recorded stereo pair instead of a live camera.

    dimos run demo-cuvslam-replay --viewer rerun --dataset sf_office_stairs

``sf_office_stairs`` is the stereo recording this demo was built on.

The right camera_info carries the baseline in ``P[3]``, the only source of metric scale.
"""

from __future__ import annotations

from collections import deque
from functools import partial
import threading
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.mapping.dim_slam.dim_slam import DimSlam
from dimos.mapping.odometry_path import OdometryPath
from dimos.memory.replay import resolve_db_path
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.visualization.vis_module import vis_module

# cuVSLAM's stereo pairing window.
_STEREO_TOLERANCE = 0.001
# Bounds the queue when one imager stalls; its partner would otherwise grow all replay.
_STEREO_QUEUE_DEPTH = 64


class CuvslamReplayConfig(ModuleConfig):
    dataset: str = ""
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    left_stream: str = "realsense_infra_left"
    right_stream: str = "realsense_infra_right"
    left_info_stream: str = "realsense_infra_left_camera_info"
    right_info_stream: str = "realsense_infra_right_camera_info"
    tf_stream: str = "tf"


class CuvslamReplay(Module):
    dedicated_worker = True

    config: CuvslamReplayConfig

    image: Out[Image]
    camera_info: Out[CameraInfo]
    # cuVSLAM resolves the rig from tf; without the recorded chain every image drops.
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.dataset:
            raise ValueError("No recording to replay. Pass --dataset /path/to/recording.db")
        store = self.register_disposable(
            SqliteStore(path=str(resolve_db_path(self.config.dataset)), must_exist=True)
        )
        store.start()
        # Separate Replays would drift and hand cuVSLAM mismatched left/right frames.
        replay = store.replay(
            speed=self.config.speed, seek=self.config.seek, duration=self.config.duration
        )
        # Ordinal pairing desynchronizes permanently on a dropped frame.
        self._pending_lock = threading.Lock()
        self._pending = {
            "left": deque[Image](maxlen=_STEREO_QUEUE_DEPTH),
            "right": deque[Image](maxlen=_STEREO_QUEUE_DEPTH),
        }
        for side, stream_name in (
            ("left", self.config.left_stream),
            ("right", self.config.right_stream),
        ):
            self.register_disposable(
                replay.stream(stream_name)
                .observable()
                .subscribe(on_next=partial(self._queue_stereo_frame, side))
            )
        for stream_name in (self.config.left_info_stream, self.config.right_info_stream):
            self.register_disposable(
                replay.stream(stream_name).observable().subscribe(on_next=self.camera_info.publish)
            )
        self.register_disposable(
            replay.stream(self.config.tf_stream).observable().subscribe(on_next=self.tf.publish)
        )

    def _queue_stereo_frame(self, side: str, frame: Image) -> None:
        paired: list[Image] = []
        with self._pending_lock:
            self._pending[side].append(frame)
            left, right = self._pending["left"], self._pending["right"]
            while left and right:
                skew = left[0].ts - right[0].ts
                if abs(skew) <= _STEREO_TOLERANCE:
                    paired += [left.popleft(), right.popleft()]
                elif skew < 0:
                    left.popleft()
                else:
                    right.popleft()
        for image in paired:
            self.image.publish(image)


def _path_at_true_height(path: Any) -> Any:
    return path.to_rerun(z_offset=0.0, radii=0.02)


demo_cuvslam_replay = autoconnect(
    CuvslamReplay.blueprint(),
    DimSlam.blueprint(use_imu=False),
    OdometryPath.blueprint(),
    vis_module(
        global_config.viewer,
        rerun_config={"visual_override": {"world/path": _path_at_true_height}},
    ),
    # DimSlam is native and speaks LCM only; don't inherit DIMOS_TRANSPORT.
).global_config(transport="lcm", n_workers=4)
