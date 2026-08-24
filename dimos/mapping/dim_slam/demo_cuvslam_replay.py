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

The recorded tf chain places the two imagers; that baseline is the only source of metric
scale.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.mapping.dim_slam.dim_slam import DimSlam
from dimos.mapping.dim_slam.stereo_pairing import stamp_matched_pairs
from dimos.mapping.odometry_path import OdometryPath, path_at_true_height
from dimos.memory.replay import resolve_db_path
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.visualization.vis_module import vis_module


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
        self.register_disposable(
            stamp_matched_pairs(
                replay.stream(self.config.left_stream).observable(),
                replay.stream(self.config.right_stream).observable(),
            ).subscribe(on_next=self._publish_stereo_pair)
        )
        for stream_name in (self.config.left_info_stream, self.config.right_info_stream):
            self.register_disposable(
                replay.stream(stream_name).observable().subscribe(on_next=self.camera_info.publish)
            )
        self.register_disposable(
            replay.stream(self.config.tf_stream).observable().subscribe(on_next=self.tf.publish)
        )

    def _publish_stereo_pair(self, pair: tuple[Image, Image]) -> None:
        for image in pair:
            self.image.publish(image)


demo_cuvslam_replay = autoconnect(
    CuvslamReplay.blueprint(),
    DimSlam.blueprint(),
    OdometryPath.blueprint(),
    vis_module(
        global_config.viewer,
        rerun_config={"visual_override": {"world/path": path_at_true_height}},
    ),
).global_config(n_workers=4)
