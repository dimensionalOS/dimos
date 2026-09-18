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

"""Synthetic HSSD dataset built with trimesh; no real assets or simulator needed."""

import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import trimesh

from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.simulation.habitat import frames
from dimos.simulation.habitat.hssd_ground_truth import (
    HSSD_DATASET_CONFIG_ENV,
    HssdDataset,
    LabelMode,
    habitat_to_ros,
    main,
    object_boxes,
    object_labels,
    scene_detections,
)

WALL_PARENT = "wall_0"
COS45 = math.sqrt(0.5)


def _box(name: str, lo: tuple[float, float, float], hi: tuple[float, float, float]) -> Any:
    extents = tuple(h - l for l, h in zip(lo, hi, strict=True))
    center = tuple((l + h) / 2 for l, h in zip(lo, hi, strict=True))
    mesh = trimesh.creation.box(extents=extents)
    mesh.apply_translation(center)
    mesh.metadata["name"] = name
    return mesh


def _stage_glb(path: Path) -> None:
    scene: Any = trimesh.Scene()

    def add(name: str, mesh: Any, parent: str | None = None) -> None:
        kwargs = {"parent_node_name": parent} if parent else {}
        scene.add_geometry(mesh, node_name=name, geom_name=name, **kwargs)

    # A 2 m wall along x with a 0.7 m doorway: two solid pieces and a header above
    # the door that must not become a wall. Faces live in one merged mesh.
    add(
        "geometry_wall.123",
        trimesh.util.concatenate(
            [
                _box("a1", (0.0, 0.0, 0.0), (0.8, 2.7, 0.1)),
                _box("a2", (1.5, 0.0, 0.0), (2.0, 2.7, 0.1)),
                _box("header", (0.8, 2.1, 0.0), (1.5, 2.7, 0.1)),
            ]
        ),
    )
    # A perpendicular wall touching the end of the first one.
    add("geometry_#FFFFFF", _box("b", (1.95, 0.0, 0.1), (2.05, 2.7, 1.6)))
    # A grouped wall, and a half-height wall.
    scene.graph.update(frame_to=WALL_PARENT, frame_from=scene.graph.base_frame, matrix=np.eye(4))
    add("geometry_wall.9", _box("c", (3.0, 0.0, 0.0), (3.1, 2.7, 2.0)), parent=WALL_PARENT)
    add("geometry_wallTop.9", _box("c-top", (3.0, 2.7, 0.0), (3.1, 2.701, 2.0)), parent=WALL_PARENT)
    add("geometry_wall.77", _box("d", (5.0, 0.0, 0.0), (6.0, 1.0, 0.1)))
    # Floor, ceiling and a wallpaper roll on the floor never reach the slice height.
    add("geometry_floor", _box("floor", (-9, 0.0, -9), (9, 0.001, 9)))
    add("geometry_ceiling", _box("ceiling", (-9, 2.7, -9), (9, 2.8, 9)))
    add("geometry_wallpaper", _box("roll", (-9, 0.0, -9), (9, 0.1, 9)))
    scene.export(str(path))


def _object_glb(path: Path, extents: tuple[float, float, float]) -> None:
    scene: Any = trimesh.Scene()
    mesh = trimesh.creation.box(extents=extents)
    # The node transform lifts the box so its base sits on the asset origin.
    matrix = np.eye(4)
    matrix[1, 3] = extents[1] / 2
    scene.add_geometry(mesh, node_name="mesh", geom_name="mesh", transform=matrix)
    scene.export(str(path))


def _write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


@pytest.fixture
def dataset(tmp_path: Path) -> HssdDataset:
    root = tmp_path / "hssd-hab"
    config = root / "hssd-hab.scene_dataset_config.json"
    _write_json(config, {"stages": {"paths": {".json": ["stages"]}}})
    _write_json(root / "stages/s1.stage_config.json", {"render_asset": "s1.glb"})
    _stage_glb(root / "stages/s1.glb")

    _write_json(root / "objects/a/aaaa.object_config.json", {"render_asset": "aaaa.glb"})
    _object_glb(root / "objects/a/aaaa.glb", (0.2, 0.6, 0.4))
    _write_json(
        root / "objects/decomposed/bbbb/bbbb_part_1.object_config.json",
        {"render_asset": "bbbb_part_1.glb"},
    )
    _object_glb(root / "objects/decomposed/bbbb/bbbb_part_1.glb", (1.0, 0.5, 1.0))
    _write_json(root / "objects/openings/cccc.object_config.json", {"render_asset": "cccc.glb"})
    _object_glb(root / "objects/openings/cccc.glb", (0.1, 2.0, 0.9))

    (root / "semantics").mkdir()
    (root / "semantics/objects.csv").write_text(
        "id,name,main_category,super_category\n"
        'aaaa,"Frosty mug",drinkware,dining_ware\n'
        "bbbb,Table,,furniture\n"
    )
    _write_json(
        root / "scenes/s1.scene_instance.json",
        {
            "stage_instance": {"template_name": "stages/s1"},
            "translation_origin": "asset_local",
            "object_instances": [
                {
                    "template_name": "aaaa",
                    "translation": [1, 0, -2],
                    "rotation": [1, 0, 0, 0],
                    "non_uniform_scale": [1, 1, 1],
                    "motion_type": "STATIC",
                },
                {
                    "template_name": "aaaa",
                    "translation": [0, 0, 0],
                    "rotation": [COS45, 0, COS45, 0],
                    "non_uniform_scale": [2, 1, 1],
                    "motion_type": "STATIC",
                },
                {
                    "template_name": "bbbb_part_1",
                    "translation": [5, 0, 5],
                    "rotation": [1, 0, 0, 0],
                    "non_uniform_scale": [1, 1, 1],
                    "motion_type": "STATIC",
                },
                {
                    "template_name": "cccc",
                    "translation": [-3, 0, 4],
                    "rotation": [1, 0, 0, 0],
                    "non_uniform_scale": [1, 1, 1],
                    "motion_type": "STATIC",
                },
            ],
        },
    )
    return HssdDataset(config)


def _view(detections: Detection3DArray) -> dict[str, tuple[tuple[float, ...], tuple[float, ...]]]:
    out: dict[str, tuple[tuple[float, ...], tuple[float, ...]]] = {}
    for d in detections.detections:
        c, s = d.bbox.center.position, d.bbox.size
        out[d.id] = (
            (round(float(c.x), 6), round(float(c.y), 6), round(float(c.z), 6)),
            (round(float(s.x), 6), round(float(s.y), 6), round(float(s.z), 6)),
        )
    return out


def test_objects_then_walls_with_exact_ros_boxes(dataset: HssdDataset) -> None:
    detections = scene_detections(dataset, "s1")

    assert [d.id for d in detections.detections] == [
        "aaaa@0",
        "aaaa@1",
        "bbbb_part_1@2",
        "cccc@3",
        "wall_000",
        "wall_001",
        "wall_002",
        "wall_003",
        "wall_004",
    ]
    boxes = _view(detections)
    # aaaa@0: habitat x[0.9,1.1] y[0,0.6] z[-2.2,-1.8] -> ROS (-z,-x,y), corners re-sorted.
    assert boxes["aaaa@0"] == ((2.0, -1.0, 0.3), (0.4, 0.2, 0.6))
    # aaaa@1: scaled x[-0.2,0.2], rotated 90 deg about Y so x<->z swap: habitat x[-0.2,0.2] z[-0.2,0.2].
    assert boxes["aaaa@1"] == ((0.0, 0.0, 0.3), (0.4, 0.4, 0.6))
    assert boxes["bbbb_part_1@2"] == ((-5.0, -5.0, 0.25), (1.0, 1.0, 0.5))
    # Walls are the stage sliced 20 cm up, ordered by ROS min corner: the grouped wall
    # along z, the perpendicular wall, the half-height wall (1 m tall), then the two
    # pieces on either side of the doorway; the header above the door is not a wall.
    assert boxes["wall_000"] == ((-1.0, -3.05, 1.35), (2.0, 0.1, 2.7))
    # The perpendicular wall also absorbs its neighbour's end cap at the junction, so
    # the two boxes overlap by the neighbour's thickness there.
    assert boxes["wall_001"] == ((-0.8, -2.0, 1.35), (1.6, 0.1, 2.7))
    assert boxes["wall_002"] == ((-0.05, -5.5, 0.5), (0.1, 1.0, 1.0))
    assert boxes["wall_003"] == ((-0.05, -1.75, 1.35), (0.1, 0.5, 2.7))
    assert boxes["wall_004"] == ((-0.05, -0.4, 1.35), (0.1, 0.8, 2.7))
    assert detections.frame_id == "world"
    assert detections.ts == 0.0
    for d in detections.detections:
        q = d.bbox.center.orientation
        assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)


def test_rotated_object_box_is_tight_not_corner_based(dataset: HssdDataset) -> None:
    instance = dataset.scene_instance("s1")
    instance["object_instances"] = [
        {
            "template_name": "aaaa",
            "translation": [0, 0, 0],
            "rotation": [math.cos(math.pi / 8), 0, math.sin(math.pi / 8), 0],
            "non_uniform_scale": [1, 1, 1],
        }
    ]
    (box,) = object_boxes(dataset, instance)
    # A 0.2 x 0.4 footprint rotated 45 degrees about Y spans (0.2+0.4)/sqrt(2) on x and z.
    span = (0.2 + 0.4) / math.sqrt(2)
    assert box.max[0] - box.min[0] == pytest.approx(span, abs=1e-6)
    assert box.max[2] - box.min[2] == pytest.approx(span, abs=1e-6)
    assert (box.min[1], box.max[1]) == (0.0, 0.6)


@pytest.mark.parametrize(
    ("mode", "expected"),
    [
        (
            "category+name",
            {"aaaa": ("drinkware", "Frosty mug"), "bbbb_part_1": ("Table",), "cccc": ("cccc",)},
        ),
        ("category", {"aaaa": ("drinkware",), "bbbb_part_1": ("Table",), "cccc": ("cccc",)}),
        ("name", {"aaaa": ("Frosty mug",), "bbbb_part_1": ("Table",), "cccc": ("cccc",)}),
    ],
)
def test_object_labels_by_mode(
    dataset: HssdDataset, mode: LabelMode, expected: dict[str, tuple[str, ...]]
) -> None:
    assert {t: object_labels(t, dataset.semantics, mode) for t in expected} == expected


def test_labels_reach_the_detections_and_walls_use_node_names(dataset: HssdDataset) -> None:
    by_id = {d.id: d for d in scene_detections(dataset, "s1").detections}

    assert [r.hypothesis.class_id for r in by_id["aaaa@0"].results] == ["drinkware", "Frosty mug"]
    assert [r.hypothesis.class_id for r in by_id["cccc@3"].results] == ["cccc"]
    assert [r.hypothesis.class_id for r in by_id["wall_000"].results] == ["wall"]


def test_habitat_to_ros_matches_frames_module() -> None:
    rng = np.random.default_rng(0)
    for point in rng.normal(size=(5, 3)):
        assert habitat_to_ros(*point) == pytest.approx(tuple(frames.position_to_ros(point)))


def test_missing_object_config_is_an_error(dataset: HssdDataset) -> None:
    instance = dataset.scene_instance("s1")
    instance["object_instances"][0]["template_name"] = "zzzz"

    with pytest.raises(ValueError, match="zzzz"):
        object_boxes(dataset, instance)


def test_missing_dataset_config_is_an_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(HSSD_DATASET_CONFIG_ENV, str(tmp_path / "nope.json"))
    with pytest.raises(FileNotFoundError, match="nope.json"):
        HssdDataset.from_env()


def test_main_writes_scene_json(dataset: HssdDataset, tmp_path: Path) -> None:
    out = tmp_path / "out"

    main(["--dataset", str(dataset.config_path), "--out", str(out), "--scene", "s1"])

    view = json.loads((out / "s1.json").read_text())
    assert view["dataset"] == "hssd-hab"
    assert view["scene_id"] == "s1"
    assert view["frame_id"] == "world"
    assert view["timestamp"] == 0.0
    assert view["count"] == 9
    assert view["detections"][0]["labels"] == ["drinkware", "Frosty mug"]
    assert "labels" not in view["detections"][4]
    assert view["detections"][4]["id"] == "wall_000"
    assert view["detections"][4]["label"] == "wall"

    flat = json.loads((out / "s1.top_down.json").read_text())
    assert flat["projection"] == "top_down_xy"
    assert flat["scene_id"] == "s1"
    assert flat["count"] == 9
    by_id = {d["id"]: d for d in flat["detections"]}
    assert by_id["aaaa@0"]["center_xy"] == pytest.approx([2.0, -1.0])
    assert by_id["aaaa@0"]["size_xy"] == pytest.approx([0.4, 0.2])
    assert by_id["aaaa@0"]["labels"] == ["drinkware", "Frosty mug"]
    assert by_id["wall_004"]["center_xy"] == pytest.approx([-0.05, -0.4])
    assert by_id["wall_004"]["size_xy"] == pytest.approx([0.1, 0.8])
    assert by_id["wall_004"]["theta"] == 0.0
