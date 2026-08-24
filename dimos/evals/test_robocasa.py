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

"""Optional fixed-seed smoke test for a prepared external RoboCasa install."""

from __future__ import annotations

from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys
from types import ModuleType, SimpleNamespace
from typing import Any

import numpy as np
import pytest

from dimos.evals.generate import VqaGenerationSpec, materialize_vqa_dataset
from dimos.evals.robocasa import (
    ROBOCASA_EXPORT_SCHEMA_VERSION,
    _export,
    _extract_entities,
    _load_export,
    export_robocasa_snapshots,
)


def _spec(output: Path) -> VqaGenerationSpec:
    return VqaGenerationSpec(
        source="robocasa",
        use_case="beverage_inventory_smoke",
        task="BeverageOrganization",
        episodes=3,
        seed=100,
        camera="robot0_agentview_center",
        image_size=(128, 128),
        question_families=("semantic_presence", "spatial_left_right"),
        targets=("can", "liquor", "bottled_water", "milk", "beer", "pitcher"),
        output=output,
        min_visible_pixels=20,
        min_horizontal_separation=0.05,
        max_bbox_overlap=0.9,
    )


def test_extract_entities_maps_geom_instances_to_robocasa_objects() -> None:
    model = SimpleNamespace(
        ngeom=3,
        geom_bodyid=np.array([1, 2, 3]),
        body_parentid=np.array([0, 0, 1, 0]),
    )

    class FakeEnv:
        sim = SimpleNamespace(model=model)
        obj_body_id = {"cup_0": 1, "bottle_0": 3}
        objects = {"cup_0": object(), "bottle_0": object()}

        def get_obj_lang(self, name: str) -> str:
            return {"cup_0": "Coffee Cup", "bottle_0": "Bottle"}[name]

    segmentation = np.zeros((4, 4, 2), dtype=np.int32)
    segmentation[:, :, 0] = 5
    segmentation[0:2, 0:2, 1] = 0
    segmentation[2:4, 0:2, 1] = 1
    segmentation[:, 2:4, 1] = -1
    mujoco = SimpleNamespace(mjtObj=SimpleNamespace(mjOBJ_GEOM=5))

    entities = _extract_entities(FakeEnv(), segmentation, mujoco)

    by_id: dict[str, dict[str, Any]] = {str(entity["id"]): entity for entity in entities}
    assert by_id["cup_0"] == {
        "id": "cup_0",
        "category": "coffee_cup",
        "pixel_area": 8,
        "centroid": [0.5, 1.5],
        "bbox": [0, 0, 1, 3],
    }
    assert by_id["bottle_0"]["pixel_area"] == 0


def test_load_export_rejects_image_outside_export_directory(tmp_path: Path) -> None:
    export = tmp_path / "export"
    export.mkdir()
    np.save(tmp_path / "outside.npy", np.zeros((2, 2, 3), dtype=np.uint8))
    (export / "snapshots.jsonl").write_text(
        '{"schema_version": 1, "image": "../outside.npy", "entities": [], "provenance": {}}\n'
    )

    with pytest.raises(ValueError, match="invalid image path"):
        _load_export(export)


def test_load_export_rejects_unknown_schema(tmp_path: Path) -> None:
    export = tmp_path / "export"
    export.mkdir()
    np.save(export / "frame.npy", np.zeros((2, 2, 3), dtype=np.uint8))
    (export / "snapshots.jsonl").write_text(
        '{"schema_version": 2, "image": "frame.npy", "entities": [], "provenance": {}}\n'
    )

    with pytest.raises(ValueError, match="unsupported schema_version"):
        _load_export(export)


def test_external_exporter_loads_versioned_snapshot(tmp_path: Path, monkeypatch: Any) -> None:
    def run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        export = Path(command[-1])
        export.mkdir()
        np.save(export / "frame.npy", np.zeros((128, 128, 3), dtype=np.uint8))
        row = {
            "schema_version": ROBOCASA_EXPORT_SCHEMA_VERSION,
            "image": "frame.npy",
            "entities": [],
            "provenance": {"seed": 100},
        }
        (export / "snapshots.jsonl").write_text(json.dumps(row) + "\n")
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(subprocess, "run", run)
    spec = replace(_spec(tmp_path / "unused"), episodes=1)

    snapshots = export_robocasa_snapshots(spec, source_python=sys.executable)

    assert len(snapshots) == 1
    assert snapshots[0].provenance["seed"] == 100


def test_external_exporter_rejects_missing_manifest(tmp_path: Path, monkeypatch: Any) -> None:
    def run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(command, 0, "", "export traceback")

    monkeypatch.setattr(subprocess, "run", run)
    spec = replace(_spec(tmp_path / "unused"), episodes=1)

    with pytest.raises(RuntimeError, match=r"failed \(0\).*export traceback"):
        export_robocasa_snapshots(spec, source_python=sys.executable)


def test_export_writes_rgb_entities_and_schema(tmp_path: Path, monkeypatch: Any) -> None:
    model = SimpleNamespace(
        ngeom=1,
        geom_bodyid=np.array([1]),
        body_parentid=np.array([0, 0]),
    )

    class FakeSim:
        def __init__(self) -> None:
            self.model = model

        def render(self, *, width: int, height: int, segmentation: bool = False, **kwargs: object):
            if not segmentation:
                return np.zeros((height, width, 3), dtype=np.uint8)
            value = np.zeros((height, width, 2), dtype=np.int32)
            value[:, :, 0] = 5
            return value

    class FakeEnv:
        sim = FakeSim()
        obj_body_id = {"can_0": 1}
        objects = {"can_0": object()}
        layout_id = np.int64(2)
        style_id = np.int64(3)
        closed = False

        def reset(self) -> None:
            return None

        def close(self) -> None:
            self.closed = True

        def get_obj_lang(self, name: str) -> str:
            return "Can"

    env = FakeEnv()
    mujoco = ModuleType("mujoco")
    mujoco.mjtObj = SimpleNamespace(mjOBJ_GEOM=5)  # type: ignore[attr-defined]
    robocasa = ModuleType("robocasa")
    robocasa.__version__ = "test"  # type: ignore[attr-defined]
    robocasa.__path__ = []  # type: ignore[attr-defined]
    utils = ModuleType("robocasa.utils")
    utils.__path__ = []  # type: ignore[attr-defined]
    env_utils = ModuleType("robocasa.utils.env_utils")
    env_utils.create_env = lambda **kwargs: env  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "mujoco", mujoco)
    monkeypatch.setitem(sys.modules, "robocasa", robocasa)
    monkeypatch.setitem(sys.modules, "robocasa.utils", utils)
    monkeypatch.setitem(sys.modules, "robocasa.utils.env_utils", env_utils)

    config = tmp_path / "config.json"
    config.write_text(
        json.dumps(
            {
                "task": "BeverageOrganization",
                "episodes": 1,
                "seed": 100,
                "camera": "robot0_agentview_center",
                "image_size": [4, 3],
                "split": "target",
                "robot": "PandaOmron",
            }
        )
    )
    output = tmp_path / "export"

    _export(config, output)

    snapshots = _load_export(output)
    assert snapshots[0].image.shape == (3, 4, 3)
    assert snapshots[0].entities[0].category == "can"
    assert snapshots[0].provenance["layout_id"] == 2
    assert env.closed


@pytest.mark.self_hosted
@pytest.mark.mujoco
def test_robocasa_exports_semantic_and_spatial_vqa(tmp_path: Path) -> None:
    source_python = os.environ.get("ROBOCASA_PYTHON")
    if not source_python:
        pytest.skip("set ROBOCASA_PYTHON to a prepared RoboCasa interpreter")

    spec = _spec(tmp_path / "generated")
    snapshots = export_robocasa_snapshots(spec, source_python=source_python)
    assert len(snapshots) == spec.episodes
    assert all(snapshot.image.shape == (128, 128, 3) for snapshot in snapshots)

    output = materialize_vqa_dataset(spec, snapshots)
    assert output.accepted_by_family["semantic_presence"] >= 2
    assert output.accepted_by_family["spatial_left_right"] >= 1
    assert output.manifest.is_file()
    assert output.dataset.is_file()
