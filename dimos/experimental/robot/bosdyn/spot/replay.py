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

"""Replay a recorded Spot session to streams, mirroring `SpotHighLevel`'s outputs.

Opens a memory2 SQLite recording (written by `SpotRecorder`) and replays every
camera, depth, and odometry stream onto Out ports named exactly like
`SpotHighLevel`'s, so the same Rerun visualization wires up by name — no robot
required. The recorded ``tf`` tree (odom->base_link plus the base_link->camera
mounts) is republished so every frame stays spatially anchored in 3D.

The travelled trail is not built here: ``odometry`` feeds ``OdometryPath``,
which accumulates it and is equally happy on a live robot.
"""

from __future__ import annotations

from collections.abc import AsyncIterator
import math
from pathlib import Path

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.experimental.robot.bosdyn.spot.config import (
    CAMERA_STREAM_SUFFIXES,
    FRONT_CAMERA_ROTATE_UPRIGHT,
)
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Image Out ports to replay, matching SpotHighLevel / SpotRecorder stream names.
_IMAGE_STREAMS = [
    f"{kind}_image_{suffix}" for kind in ("grayscale", "depth") for suffix in CAMERA_STREAM_SUFFIXES
]
_PLAYBACK_STREAMS = [*_IMAGE_STREAMS, "grayscale_info", "depth_info", "odometry"]

# SpotHighLevel rights the sideways front camera images but leaves their optical
# tf frames at the raw mount orientation, so the 3D frustum + depth
# back-projection land rotated. Roll each front optical frame about its viewing
# (z) axis to bring it back upright. The two front cameras mount mirror-imaged,
# so frontright sits a half turn (2 quarter turns) past frontleft.
_HALF_TURN_QUARTERS = 2
_OPTICAL_FRAME_ROLL_TURNS = {
    "frontleft_camera_optical": FRONT_CAMERA_ROTATE_UPRIGHT,
    "frontright_camera_optical": FRONT_CAMERA_ROTATE_UPRIGHT + _HALF_TURN_QUARTERS,
}


class SpotReplayConfig(ModuleConfig):
    """Where to read the recording from and how to play it back."""

    # An on-disk path, or the name of a dataset to pull from LFS (e.g.
    # ``spot_small_loop.db``). Empty -> newest ``*.db`` in ``dataset_dir``.
    db_path: str = ""
    dataset_dir: str = "~/datasets/spot"

    speed: float = 1.0
    loop: bool = True
    seek: float | None = None
    duration: float | None = None

    # Roll the front optical tf frames upright at replay time. Off by default:
    # recordings are expected to already store upright front frames. Turn on to
    # view an old recording whose tf still holds the raw sideways mount.
    roll_front_frames: bool = False


class SpotReplay(Module):
    """Replays Spot's fisheye + depth cameras, odometry, and tf from a recording."""

    config: SpotReplayConfig
    dedicated_worker = True

    grayscale_image_front_left: Out[Image]
    grayscale_image_front_right: Out[Image]
    grayscale_image_left: Out[Image]
    grayscale_image_right: Out[Image]
    grayscale_image_back: Out[Image]

    depth_image_front_left: Out[Image]
    depth_image_front_right: Out[Image]
    depth_image_left: Out[Image]
    depth_image_right: Out[Image]
    depth_image_back: Out[Image]

    grayscale_info: Out[CameraInfo]
    depth_info: Out[CameraInfo]

    odometry: Out[Odometry]
    tf: Out[TFMessage]

    def _resolve_db_path(self) -> Path:
        if self.config.db_path:
            return resolve_db_path(Path(self.config.db_path).expanduser())
        directory = Path(self.config.dataset_dir).expanduser()
        recordings = sorted(directory.glob("*.db"), key=lambda path: path.stat().st_mtime)
        if not recordings:
            raise FileNotFoundError(f"No .db recordings found in {directory}")
        return recordings[-1]

    def _republish_tf(self, message: TFMessage) -> None:
        self.tf.publish(
            TFMessage(*(self._roll_optical_frame(transform) for transform in message.transforms))
        )

    def _roll_optical_frame(self, transform: Transform) -> Transform:
        if not self.config.roll_front_frames:
            return transform
        turns = _OPTICAL_FRAME_ROLL_TURNS.get(transform.child_frame_id)
        if not turns:
            return transform
        roll = Quaternion.from_euler(Vector3(0.0, 0.0, turns * math.pi / 2))
        return Transform(
            translation=transform.translation,
            rotation=transform.rotation * roll,
            frame_id=transform.frame_id,
            child_frame_id=transform.child_frame_id,
            ts=transform.ts,
        )

    async def main(self) -> AsyncIterator[None]:
        db_path = self._resolve_db_path()
        logger.info(f"Replaying Spot recording from {db_path}")

        store = SqliteStore(path=str(db_path), must_exist=True)
        store.start()
        self.register_disposable(store)

        replay = store.replay(
            speed=self.config.speed,
            loop=self.config.loop,
            seek=self.config.seek,
            duration=self.config.duration,
        )
        available = set(replay.list_streams())

        for name in _PLAYBACK_STREAMS:
            if name not in available:
                logger.warning(f"Spot replay: stream {name!r} missing from recording; skipping")
                continue
            self.register_disposable(
                replay.stream(name).observable().subscribe(getattr(self, name).publish)
            )

        if "tf" in available:
            self.register_disposable(replay.stream("tf").observable().subscribe(self._republish_tf))
        else:
            logger.warning("Spot replay: no tf stream in recording; 3D frames will be missing")

        yield
