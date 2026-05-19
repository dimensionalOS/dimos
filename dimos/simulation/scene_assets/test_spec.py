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

from __future__ import annotations

import json
from pathlib import Path

import pytest

from dimos.simulation.scene_assets.spec import ARTIFACT_FRAMES, load_scene_package


def _metadata(tmp_path: Path) -> dict[str, object]:
    return {
        "source_path": str(tmp_path / "source.glb"),
        "package_dir": str(tmp_path),
        "alignment": {
            "scale": 1.0,
            "rotation_zyx_deg": [0.0, 0.0, 0.0],
            "translation": [0.0, 0.0, 0.0],
            "y_up": True,
        },
        "artifact_frames": ARTIFACT_FRAMES,
        "artifacts": {
            "browser_visual": str(tmp_path / "visual.glb"),
            "browser_collision": str(tmp_path / "collision.glb"),
            "mujoco_model": str(tmp_path / "compiled.mjb"),
            "mujoco_wrapper": str(tmp_path / "wrapper.xml"),
        },
        "stats": {},
    }


def test_load_scene_package_rejects_missing_artifact_frames(tmp_path: Path) -> None:
    raw = _metadata(tmp_path)
    raw.pop("artifact_frames")
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text(json.dumps(raw))

    with pytest.raises(ValueError, match="missing artifact frame metadata"):
        load_scene_package(metadata_path)


def test_load_scene_package_rejects_mismatched_artifact_frames(tmp_path: Path) -> None:
    raw = _metadata(tmp_path)
    raw["artifact_frames"] = {
        "browser_visual": "dimos_world",
        "browser_collision": "source",
        "mujoco": "dimos_world",
    }
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text(json.dumps(raw))

    with pytest.raises(ValueError, match="artifact frame mismatch"):
        load_scene_package(metadata_path)


def test_load_scene_package_accepts_expected_artifact_frames(tmp_path: Path) -> None:
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text(json.dumps(_metadata(tmp_path)))

    package = load_scene_package(metadata_path)

    assert package.visual_path == tmp_path / "visual.glb"
    assert package.browser_collision_path == tmp_path / "collision.glb"
    assert package.mujoco_model_path == tmp_path / "compiled.mjb"
