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

from pathlib import Path
import pickle
from typing import Any

import pytest

from dimos.simulation.scene_assets.spec import SceneMeshAlignment, ScenePackage
from dimos.visualization.rerun.scene_package import (
    resolve_scene_package_for_rerun,
    scene_alignment_matrix,
    scene_package_static_entities,
)


class FakeRerun:
    class Transform3D:
        def __init__(self, **kwargs: Any) -> None:
            self.kwargs = kwargs

    class Asset3D:
        def __init__(self, **kwargs: Any) -> None:
            self.kwargs = kwargs


def test_resolves_composed_mjb_to_scene_package_visual(tmp_path: Path) -> None:
    package_dir = tmp_path / "store"
    visual_path = package_dir / "browser" / "visual.glb"
    visual_path.parent.mkdir(parents=True)
    visual_path.write_bytes(b"glb bytes")
    composed_path = package_dir / "mujoco" / "composed" / "g1.mjb"
    composed_path.parent.mkdir(parents=True)
    composed_path.write_bytes(b"mjb bytes")

    alignment = SceneMeshAlignment(
        scale=2.0,
        translation=(1.0, 2.0, 3.0),
        rotation_zyx_deg=(0.0, 0.0, 0.0),
        y_up=True,
    )
    ScenePackage(
        package_dir=package_dir,
        source_path=tmp_path / "source.blend",
        alignment=alignment,
        visual_path=visual_path,
    ).write_metadata()

    package = resolve_scene_package_for_rerun(composed_path)
    assert package is not None
    assert package.visual_path == visual_path.resolve()

    static_entities = scene_package_static_entities(composed_path)
    assert set(static_entities) == {"world/scene"}
    pickle.dumps(static_entities)

    transform, asset = static_entities["world/scene"](FakeRerun)
    assert transform.kwargs == {
        "translation": [1.0, 2.0, 3.0],
        "mat3x3": [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        "scale": 2.0,
    }
    assert asset.kwargs == {
        "contents": b"glb bytes",
        "media_type": "model/gltf-binary",
    }


def test_missing_visual_does_not_add_static_scene(tmp_path: Path) -> None:
    package_dir = tmp_path / "store"
    composed_path = package_dir / "mujoco" / "composed" / "g1.mjb"
    composed_path.parent.mkdir(parents=True)
    composed_path.write_bytes(b"mjb bytes")
    ScenePackage(
        package_dir=package_dir,
        source_path=tmp_path / "source.blend",
        alignment=SceneMeshAlignment(),
        visual_path=None,
    ).write_metadata()

    assert scene_package_static_entities(composed_path) == {}


def test_scene_alignment_matrix_keeps_z_up_when_requested() -> None:
    alignment = SceneMeshAlignment(
        rotation_zyx_deg=(90.0, 0.0, 0.0),
        y_up=False,
    )

    matrix = scene_alignment_matrix(alignment)

    assert matrix[0][0] == pytest.approx(0.0)
    assert matrix[0][1] == pytest.approx(-1.0)
    assert matrix[1][0] == pytest.approx(1.0)
    assert matrix[1][1] == pytest.approx(0.0)
    assert matrix[2] == [0.0, 0.0, 1.0]
