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

``--dataset`` takes an absolute .db path or a dataset name; a name is downloaded from
LFS on first use. ``sf_office_stairs`` is the stereo recording this demo was built on.

Same wiring as ``demo-cuvslam-realsense`` with the camera swapped for a replay of a
memory recording. It is the only way to exercise the tracker on a machine with no
camera attached (macOS has no realsense support), and the only way to get a repeatable
trajectory out of it.

The four recorded streams are the ones ``RealSenseCamera`` would have published: the
rectified IR pair and both camera_infos. Both eyes go onto the one ``image`` stream, the
way the live demo remaps them; cuVSLAM tells them apart by ``frame_id``. The right
camera_info carries the baseline in ``P[3]`` and is the only per-unit source of metric
scale, so a recording missing it produces a scale-free trajectory rather than an error.
"""

from __future__ import annotations

from typing import Any

import reactivex as rx

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.mapping.dim_slam.dim_slam import DimSlam
from dimos.mapping.odometry_hist import OdometryHist
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.visualization.vis_module import vis_module


class CuvslamReplayConfig(ModuleConfig):
    # An absolute .db path, or a name resolve_named_path can find. No default: the
    # recordings this replays are too big for LFS, so there is nothing to fall back to.
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
    """Publishes a recorded stereo IR pair on the ports cuVSLAM subscribes to."""

    dedicated_worker = True

    config: CuvslamReplayConfig

    image: Out[Image]
    camera_info: Out[CameraInfo]
    # cuVSLAM places each camera on the rig by tf, so the recorded chain has to be
    # replayed alongside the frames; without it no camera resolves and every image drops.
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
        # One Replay for every stream so they share a wall-clock anchor; separate ones
        # would drift and cuVSLAM would be handed mismatched left/right frames.
        replay = store.replay(
            speed=self.config.speed, seek=self.config.seek, duration=self.config.duration
        )
        # cuVSLAM assembles a frame set by arrival and rejects any whose stamps differ by
        # more than a millisecond. Two independent subscriptions let the scheduler run one
        # eye a frame ahead of the other, so zip them and publish each pair back to back.
        self.register_disposable(
            rx.zip(
                replay.stream(self.config.left_stream).observable(),
                replay.stream(self.config.right_stream).observable(),
            ).subscribe(on_next=self._publish_stereo_pair)
        )
        for stream_name in (self.config.left_info_stream, self.config.right_info_stream):
            self.register_disposable(
                replay.stream(stream_name).observable().subscribe(on_next=self.camera_info.publish)
            )
        # tf has to keep flowing: the tracker looks the rig up at each image's stamp, so
        # stopping once the rig first resolves leaves every later frame unplaceable.
        self.register_disposable(
            replay.stream(self.config.tf_stream).observable().subscribe(on_next=self.tf.publish)
        )

    def _publish_stereo_pair(self, pair: tuple[Image, Image]) -> None:
        for frame in pair:
            self.image.publish(frame)


def _path_at_true_height(path: Any) -> Any:
    return path.to_rerun(z_offset=0.0, radii=0.02)


demo_cuvslam_replay = autoconnect(
    CuvslamReplay.blueprint(),
    # No IMU streaming here, so the filter seeds level off the first tracked pose.
    DimSlam.blueprint(use_imu=False),
    OdometryHist.blueprint(),
    vis_module(
        global_config.viewer,
        rerun_config={"visual_override": {"world/odom_hist": _path_at_true_height}},
    ),
    # DimSlam is a native module and speaks LCM only, so the blueprint pins it rather
    # than inheriting whatever DIMOS_TRANSPORT the shell has (macOS defaults to zenoh).
).global_config(transport="lcm", n_workers=4)
