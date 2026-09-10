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

Opens a memory SQLite recording (written by `SpotRecorder`) and replays every
camera, depth, and odometry stream onto Out ports named exactly like
`SpotHighLevel`'s, so the same Rerun visualization wires up by name — no robot
required. The recorded ``tf`` tree (odom->base_link plus the base_link->camera
mounts) is republished so every frame stays spatially anchored in 3D.

The travelled trail is not built here: ``odometry`` feeds ``OdometryHist``,
which accumulates it and is equally happy on a live robot.
"""

from __future__ import annotations

from collections.abc import AsyncIterator
from functools import partial
import math
from pathlib import Path

from dimos.constants import RECORDINGS_DIR
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.experimental.robot.bosdyn.spot.config import (
    CAMERA_STREAM_SUFFIXES,
    FRONT_CAMERA_ROTATE_UPRIGHT,
)
from dimos.experimental.robot.bosdyn.spot.utils import (
    QUARTER_TURN,
    roll_optical_frame,
    rotate_camera_info,
    rotate_image,
    upright_roll,
)
from dimos.memory.replay import Replay, resolve_db_path
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
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
_INFO_STREAMS = ["grayscale_info", "depth_info"]
_PLAYBACK_STREAMS = [*_IMAGE_STREAMS, *_INFO_STREAMS, "odometry"]

# Old recordings hold front images turned a quarter turn upright while their
# optical tf frames still sit at the raw sideways mount. Roll those frames to
# match (`roll_front_frames`). The two front cameras mount mirror-imaged, so
# frontright sits a half turn (2 quarter turns) past frontleft.
_HALF_TURN_QUARTERS = 2
_OPTICAL_FRAME_ROLL_TURNS = {
    "frontleft_camera_optical": FRONT_CAMERA_ROTATE_UPRIGHT,
    "frontright_camera_optical": FRONT_CAMERA_ROTATE_UPRIGHT + _HALF_TURN_QUARTERS,
}
# How far into the recording to look for the static camera mounts in ``tf``.
_MOUNT_SCAN_SECONDS = 5.0
# Leans smaller than this (radians) are the side cameras' float noise, not a tilt.
_LEVEL_ROLL_EPSILON = 1e-6


class SpotReplayConfig(ModuleConfig):
    """Where to read the recording from and how to play it back."""

    # An on-disk path, or the name of a dataset to pull from LFS (e.g.
    # ``spot_small_loop.db``). Empty -> newest ``*.db`` in ``dataset_dir``.
    db_path: str = ""
    dataset_dir: str = str(RECORDINGS_DIR / "spot")

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

    # Optical frame -> roll that levels its recorded pixels; filled in by `main`.
    _rolls: dict[str, float]

    def _resolve_db_path(self) -> Path:
        if self.config.db_path:
            return resolve_db_path(Path(self.config.db_path).expanduser())
        directory = Path(self.config.dataset_dir).expanduser()
        recordings = sorted(directory.glob("*.db"), key=lambda path: path.stat().st_mtime)
        if not recordings:
            raise FileNotFoundError(f"No .db recordings found in {directory}")
        return recordings[-1]

    def _legacy_roll(self, transform: Transform) -> Transform:
        """Roll a raw sideways front frame a quarter turn upright (old recordings only)."""
        if not self.config.roll_front_frames:
            return transform
        turns = _OPTICAL_FRAME_ROLL_TURNS.get(transform.child_frame_id, 0)
        return roll_optical_frame(transform, turns * QUARTER_TURN)

    def _level_rolls(self, replay: Replay, available: set[str]) -> dict[str, float]:
        """Per optical frame, the roll that levels the recorded pixels.

        Recorded front images lean by whatever the mount is off a clean quarter
        turn (12.3° on Spot). The mounts sit in the first seconds of ``tf``, so
        read them ahead of playback and measure the lean per camera frame.
        """
        tf_stream = replay.stream("tf")
        first_ts = tf_stream.first_ts() if "tf" in available else None
        if first_ts is None:
            return {}
        camera_frames = {
            image.frame_id
            for name in _IMAGE_STREAMS
            if name in available and (image := replay.stream(name).first()) is not None
        }
        deadline = first_ts + _MOUNT_SCAN_SECONDS
        rolls: dict[str, float] = {}
        for ts, message in tf_stream.iterate_ts():
            for transform in message.transforms:
                if (
                    transform.child_frame_id in camera_frames
                    and transform.child_frame_id not in rolls
                ):
                    rolls[transform.child_frame_id] = upright_roll(self._legacy_roll(transform))
            if ts > deadline or len(rolls) == len(camera_frames):
                break
        for frame_id in camera_frames - set(rolls):
            logger.warning(
                f"Spot replay: no tf mount for {frame_id!r}; its images stay as recorded"
            )
        return rolls

    def _republish_tf(self, message: TFMessage) -> None:
        self.tf.publish(
            TFMessage(
                *(
                    roll_optical_frame(
                        self._legacy_roll(transform),
                        self._rolls.get(transform.child_frame_id, 0.0),
                    )
                    for transform in message.transforms
                )
            )
        )

    def _publish_image(self, name: str, image: Image) -> None:
        getattr(self, name).publish(rotate_image(image, self._rolls.get(image.frame_id, 0.0)))

    def _publish_info(self, name: str, info: CameraInfo) -> None:
        getattr(self, name).publish(rotate_camera_info(info, self._rolls.get(info.frame_id, 0.0)))

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
        self._rolls = {
            frame_id: roll
            for frame_id, roll in self._level_rolls(replay, available).items()
            if abs(roll) > _LEVEL_ROLL_EPSILON
        }
        for frame_id, roll in self._rolls.items():
            logger.info(f"Spot replay: levelling {frame_id} by {math.degrees(roll):.1f} deg")

        for name in _PLAYBACK_STREAMS:
            if name not in available:
                logger.warning(f"Spot replay: stream {name!r} missing from recording; skipping")
                continue
            if name in _IMAGE_STREAMS:
                publish = partial(self._publish_image, name)
            elif name in _INFO_STREAMS:
                publish = partial(self._publish_info, name)
            else:
                publish = getattr(self, name).publish
            self.register_disposable(replay.stream(name).observable().subscribe(publish))

        if "tf" in available:
            self.register_disposable(replay.stream("tf").observable().subscribe(self._republish_tf))
        else:
            logger.warning("Spot replay: no tf stream in recording; 3D frames will be missing")

        yield
