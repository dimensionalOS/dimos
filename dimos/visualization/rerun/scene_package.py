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

"""Rerun helpers for cooked scene packages."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

from dimos.simulation.scene_assets.spec import SceneMeshAlignment, ScenePackage, load_scene_package
from dimos.simulation.scenes.catalog import resolve_scene_package

StaticRerunFactory = Callable[[Any], Any]


@dataclass(frozen=True)
class SceneVisualFactory:
    visual_path: Path
    alignment: SceneMeshAlignment

    def __call__(self, rr: Any) -> list[Any]:
        if not self.visual_path.exists():
            return []

        return [
            rr.Transform3D(
                translation=list(self.alignment.translation),
                mat3x3=scene_alignment_matrix(self.alignment),
                scale=float(self.alignment.scale),
            ),
            rr.Asset3D(
                contents=self.visual_path.read_bytes(),
                media_type="model/gltf-binary",
            ),
        ]


def scene_package_static_entities(
    scene: str | Path | None,
    *,
    entity_path: str = "world/scene",
) -> dict[str, StaticRerunFactory]:
    """Return static Rerun factories for a cooked scene package visual.

    The scene package visual artifact is stored in source coordinates. The
    returned factory logs the package alignment plus the GLB bytes so native and
    browser viewers do not need direct filesystem access to the asset.
    """
    package = resolve_scene_package_for_rerun(scene)
    visual_path = package.browser_visual_path("rerun") if package is not None else None
    if visual_path is None or not visual_path.exists():
        return {}

    return {entity_path: SceneVisualFactory(visual_path, package.alignment)}


def resolve_scene_package_for_rerun(scene: str | Path | None) -> ScenePackage | None:
    """Resolve normal scene arguments and composed MuJoCo binaries to a package."""
    if scene is None:
        return None

    candidate = Path(str(scene).strip()).expanduser()
    if candidate.suffix.lower() == ".mjb":
        for parent in candidate.resolve().parents:
            metadata_path = parent / "scene.meta.json"
            if metadata_path.exists():
                return load_scene_package(metadata_path)
        return None

    return resolve_scene_package(scene)


def scene_alignment_matrix(alignment: SceneMeshAlignment) -> list[list[float]]:
    """Return the rotation part of SceneMeshAlignment as a Rerun mat3x3."""
    yaw, pitch, roll = [math.radians(float(v)) for v in alignment.rotation_zyx_deg]
    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)
    rz = ((cz, -sz, 0.0), (sz, cz, 0.0), (0.0, 0.0, 1.0))
    ry = ((cy, 0.0, sy), (0.0, 1.0, 0.0), (-sy, 0.0, cy))
    rx = ((1.0, 0.0, 0.0), (0.0, cx, -sx), (0.0, sx, cx))

    matrix = _matmul(_matmul(rz, ry), rx)
    if alignment.y_up:
        y_to_z = ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))
        matrix = _matmul(matrix, y_to_z)
    return [list(row) for row in matrix]


def _matmul(
    a: Sequence[Sequence[float]],
    b: Sequence[Sequence[float]],
) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
