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

"""Native Unreal RGB-D adapter for the ordinary sim2 camera acquisition module."""

import json
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any
import xml.etree.ElementTree as ET

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.sim_msgs import WorldState
import numpy as np
from numpy.typing import NDArray

from dimos.sim2.description import SceneConfig
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.unreal.assets import (
    export_visuals,
    pose_matrix,
    unreal_pose,
)
from dimos.sim2.sensors.camera.renderers.unreal.client import IDENTITY_FIELDS, RendererProcess


class UnrealRenderer:
    def __init__(
        self,
        executable: Path,
        *,
        project: Path | None = None,
        level: str | None = None,
        authored_instances: tuple[str, ...] = (),
        log_path: Path,
        startup_timeout: float = 180,
        capture_timeout: float = 15,
    ) -> None:
        if authored_instances and level is None:
            raise ValueError("authored instances require an Unreal level")
        self.process = RendererProcess(
            executable,
            log_path,
            project=project,
            level=level,
            startup_timeout=startup_timeout,
            capture_timeout=capture_timeout,
        )
        self.authored_instances = authored_instances
        self._assets: TemporaryDirectory[str] | None = None
        self._sample: WorldState | None = None

    def load(self, scene: SceneConfig, camera: CameraConfig) -> None:
        self.camera = camera
        self.revision = scene.revision
        self.bodies = {
            link.attrib["name"]
            for i in scene.instances
            for link in ET.fromstring(i.description.xml).findall("link")
        }
        if camera.frame_id not in self.bodies:
            raise ValueError(f"unknown camera frame {camera.frame_id}")
        manifest, self.visuals, self.static_bodies = export_visuals(scene, self.authored_instances)
        self._assets = TemporaryDirectory(prefix="dimos-unreal-")
        path = Path(self._assets.name) / "scene.json"
        try:
            path.write_text(json.dumps(manifest, allow_nan=False, separators=(",", ":")))
            self.client = self.process.start()
            self.client.load(path, timeout=self.process.startup_timeout)
        except BaseException:
            self.close()
            raise

    def set_poses(self, sample: WorldState) -> None:
        poses = poses_by_id(sample)
        if sample.model_revision != self.revision or set(poses) != self.bodies:
            raise ValueError("Unreal requires complete poses for the loaded model revision")
        for body in self.static_bodies:
            if not np.allclose(pose_matrix(poses[body]), np.eye(4), atol=1e-7):
                raise ValueError(f"authored background {body} moved relative to its native level")
        self._sample = sample
        self._poses = poses

    def render(self, pose_world: Pose) -> tuple[NDArray[np.uint8], NDArray[np.float32]]:
        if self._sample is None:
            raise RuntimeError("set_poses must precede render")
        sample, c = self._sample, self.camera
        command: dict[str, Any] = {
            key: str(
                sample.model_revision if key == "model_revision" else getattr(sample.header, key)
            )
            for key in IDENTITY_FIELDS
        }
        command["camera"] = {
            **unreal_pose(pose_matrix(pose_world), camera=True),
            "width": c.width,
            "height": c.height,
            "fx": c.fx,
            "fy": c.fy,
            "cx": c.cx,
            "cy": c.cy,
        }
        command["poses"] = [
            {
                "id": v.id,
                **unreal_pose(
                    (np.eye(4) if v.body is None else pose_matrix(self._poses[v.body])) @ v.local
                ),
            }
            for v in self.visuals
        ]
        result = self.client.capture(command)
        return result.rgba[:, :, :3].copy(), result.depth

    def cancel(self) -> None:
        """Wake a camera worker blocked in native startup or GPU acquisition."""
        self.process.cancel()

    def close(self) -> None:
        self.process.close()
        if self._assets is not None:
            self._assets.cleanup()
            self._assets = None
