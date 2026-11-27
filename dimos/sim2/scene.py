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

"""The scene a simulated world is built on: the one unmounted HardwareModule, a USD whose spawn points are its mounts."""

from collections.abc import AsyncIterator, Iterator
from functools import cached_property
from pathlib import Path

from pydantic import model_validator

from dimos.core.hardware_module import HardwareModuleConfig, Mount, Mounts
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.assets.model import UsdModel
from dimos.sim2.sensor import SimSensor
from dimos.sim2.spawn import SPAWN_API, scene_spawn_points, spawn_frames_in_file, with_spawn_frames


def scene_usd(path: Path) -> UsdModel:
    """A scene file as a scene's USD: its Git LFS files fetched and each spawn point made a tf frame."""
    from dimos.robot.assets.usd_utils.lfs import pull_lfs_files, pull_usd_assets

    pull_lfs_files([path])
    text = path.read_text()
    pull_usd_assets([text], root=path.parent)
    return UsdModel.from_layer(with_spawn_frames(text, path.parent), path)


class SpawnMounts(Mounts):
    """A scene file's spawn points as its mounts, read from the file the first time one is asked for."""

    def __init__(self, path: Path) -> None:
        self._path = path

    def _read(self) -> None:
        if "_frames" not in vars(self):
            Mounts.__init__(self, *spawn_frames_in_file(self._path))

    def __getattr__(self, name: str) -> Mount:
        if name.startswith("_"):
            raise AttributeError(name)
        self._read()
        return super().__getattr__(name)

    def __iter__(self) -> Iterator[str]:
        self._read()
        return super().__iter__()


class SimSceneConfig(HardwareModuleConfig):
    """A scene's mounts have no reading to ride on: they go out on tf at static_tf_rate_hz."""

    @model_validator(mode="after")
    def _check_spawn_points(self) -> "SimSceneConfig":
        frames = self.usd.tf_frames
        unframed = [
            point.frame
            for point in scene_spawn_points(self.usd.usd, self.usd.asset_root)
            if point.frame not in frames
        ]
        if unframed:
            raise ValueError(
                f"the {SPAWN_API} prims {unframed} are not tf frames; load a scene with "
                f"scene_usd(), which makes each one a frame"
            )
        return self


class SimScene(SimSensor):
    """Static hardware: a room whose spawn points are where robots, sensors and objects start. Its tf goes out stamped with wall time."""

    config: SimSceneConfig

    async def main(self) -> AsyncIterator[None]:
        self._world = self.mirror_world()
        yield

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The edges from the room's root to each mount: a real room publishes no furniture, and the world USD places it."""
        transforms = super()._static_edges
        parent_of = {t.child_frame_id: t.frame_id for t in transforms}
        needed: set[str] = set()
        for mount in self.mounts:
            frame = self.tf_frames[mount]
            while frame in parent_of and frame not in needed:
                needed.add(frame)
                frame = parent_of[frame]
        return [t for t in transforms if t.child_frame_id in needed]
