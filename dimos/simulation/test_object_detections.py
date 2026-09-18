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

import json
from pathlib import Path

import pytest

from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.simulation.object_detections import (
    GroundTruthBox,
    Point3,
    boxes_to_detection3d_array,
    detection2d_array_to_dict,
    detection3d_array_to_dict,
    read_detection3d_array,
    ros_box,
    top_down,
    write_detection2d_json,
    write_detection3d_array,
    write_detection3d_json,
)


def flip(x: float, y: float, z: float) -> Point3:
    """Habitat-style permutation with sign flips on two axes."""
    return (-z, -x, y)


def same(x: float, y: float, z: float) -> Point3:
    return (x, y, z)


def _xyz(v: object) -> tuple[float, float, float]:
    return (v.x, v.y, v.z)  # type: ignore[attr-defined]


BOX = GroundTruthBox(
    id="sofa", labels=("sofa", "Sectional sofa"), min=(1.0, 2.0, 3.0), max=(2.0, 4.0, 6.0)
)


def test_ros_box_resorts_corners_after_sign_flips() -> None:
    center, size = ros_box((1.0, 2.0, 3.0), (2.0, 4.0, 6.0), flip)

    assert center == (-4.5, -1.5, 3.0)
    assert size == (3.0, 1.0, 2.0)


def test_labels_become_ordered_hypotheses_with_independent_poses() -> None:
    arr = boxes_to_detection3d_array([BOX], to_ros=flip, frame_id="world", ts=0.0)
    (det,) = arr.detections

    assert det.id == "sofa"
    assert det.results_length == 2
    assert [r.hypothesis.class_id for r in det.results] == ["sofa", "Sectional sofa"]
    assert [r.hypothesis.score for r in det.results] == [1.0, 1.0]
    assert _xyz(det.bbox.center.position) == (-4.5, -1.5, 3.0)
    assert _xyz(det.bbox.size) == (3.0, 1.0, 2.0)
    for result in det.results:
        assert _xyz(result.pose.pose.position) == (-4.5, -1.5, 3.0)
    det.results[0].pose.pose.position.x = 99.0
    assert det.results[1].pose.pose.position.x == -4.5
    assert det.bbox.center.position.x == -4.5
    assert arr.frame_id == det.frame_id == "world"
    assert arr.ts == det.ts == 0.0


def test_view_lists_labels_only_for_multiple_hypotheses() -> None:
    single = GroundTruthBox(id="lamp", labels=("lamp",), min=(0.0, 0.0, 0.0), max=(1.0, 1.0, 1.0))
    arr = boxes_to_detection3d_array([BOX, single], to_ros=flip, frame_id="world", ts=2.5)

    view = detection3d_array_to_dict(arr, provenance={"dataset": "hssd-hab", "scene_id": "s1"})

    assert list(view) == ["dataset", "scene_id", "frame_id", "timestamp", "count", "detections"]
    assert view["scene_id"] == "s1"
    assert view["timestamp"] == 2.5
    assert view["count"] == 2
    assert view["detections"][0]["label"] == "sofa"
    assert view["detections"][0]["labels"] == ["sofa", "Sectional sofa"]
    assert "labels" not in view["detections"][1]
    assert view["detections"][1] == {
        "id": "lamp",
        "label": "lamp",
        "score": 1.0,
        "center_xyz": [-0.5, -0.5, 0.5],
        "size_xyz": [1.0, 1.0, 1.0],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


@pytest.mark.parametrize(
    "box",
    [
        GroundTruthBox(id="", labels=("a",), min=(0.0, 0.0, 0.0), max=(1.0, 1.0, 1.0)),
        GroundTruthBox(id="a", labels=(), min=(0.0, 0.0, 0.0), max=(1.0, 1.0, 1.0)),
        GroundTruthBox(id="a", labels=("x", ""), min=(0.0, 0.0, 0.0), max=(1.0, 1.0, 1.0)),
        GroundTruthBox(id="a", labels=("x",), min=(0.0, 2.0, 0.0), max=(1.0, 1.0, 1.0)),
        GroundTruthBox(id="a", labels=("x",), min=(0.0, 0.0, 0.0), max=(1.0, float("nan"), 1.0)),
    ],
)
def test_invalid_boxes_are_rejected(box: GroundTruthBox) -> None:
    with pytest.raises(ValueError):
        boxes_to_detection3d_array([box], to_ros=flip, frame_id="world", ts=0.0)


def test_binary_and_json_round_trip(tmp_path: Path) -> None:
    arr = boxes_to_detection3d_array([BOX], to_ros=flip, frame_id="world", ts=1.5)

    binary = write_detection3d_array(arr, tmp_path / "boxes.bin")
    decoded = read_detection3d_array(binary)
    assert decoded.lcm_encode() == arr.lcm_encode()
    assert decoded.frame_id == "world"
    assert [r.hypothesis.class_id for r in decoded.detections[0].results] == [
        "sofa",
        "Sectional sofa",
    ]

    text = write_detection3d_json(arr, tmp_path / "boxes.json", provenance={"scene_id": "s1"})
    view = json.loads(text.read_text())
    assert view["scene_id"] == "s1"
    assert view["detections"][0]["center_xyz"] == [-4.5, -1.5, 3.0]


LAMP = GroundTruthBox(id="lamp", labels=("lamp",), min=(0.0, 0.0, 0.0), max=(1.0, 1.0, 1.0))


def test_top_down_projects_footprints_and_keeps_labels() -> None:
    arr = boxes_to_detection3d_array([BOX, LAMP], to_ros=flip, frame_id="world", ts=2.5)

    flat = top_down(arr)

    assert isinstance(flat, Detection2DArray)
    assert flat.detections_length == 2
    assert flat.header.frame_id == "world"
    assert flat.ts == 2.5
    sofa, lamp = flat.detections
    assert sofa.id == "sofa"
    assert [r.hypothesis.class_id for r in sofa.results] == ["sofa", "Sectional sofa"]
    assert [r.hypothesis.score for r in sofa.results] == [1.0, 1.0]
    assert (sofa.bbox.center.position.x, sofa.bbox.center.position.y) == (-4.5, -1.5)
    assert sofa.bbox.center.theta == 0.0
    assert (sofa.bbox.size_x, sofa.bbox.size_y) == (3.0, 1.0)
    # The 3D center survives in the hypothesis pose, so height is not lost entirely.
    assert _xyz(sofa.results[0].pose.pose.position) == (-4.5, -1.5, 3.0)
    assert (lamp.bbox.size_x, lamp.bbox.size_y) == (1.0, 1.0)

    sofa.results[0].hypothesis.class_id = "changed"
    lamp.bbox.size_x = 9.0
    assert arr.detections[0].results[0].hypothesis.class_id == "sofa"
    assert sofa.bbox.size_x == 3.0


def test_top_down_drops_boxes_covering_the_scene() -> None:
    floor = GroundTruthBox(
        id="floor", labels=("floor",), min=(-10.0, -10.0, 0.0), max=(10.0, 10.0, 0.1)
    )
    roof = GroundTruthBox(id="roof", labels=("roof",), min=(-9.5, -9.5, 3.0), max=(9.5, 9.5, 3.2))
    rug = GroundTruthBox(id="rug", labels=("rug",), min=(-5.0, -5.0, 0.0), max=(5.0, 5.0, 0.02))
    arr = boxes_to_detection3d_array(
        [floor, roof, rug, LAMP], to_ros=same, frame_id="world", ts=0.0
    )

    assert [d.id for d in top_down(arr).detections] == ["rug", "lamp"]
    assert [d.id for d in top_down(arr, covering_fraction=None).detections] == [
        "floor",
        "roof",
        "rug",
        "lamp",
    ]
    # A lone box is never "covering": there is no scene around it to cover.
    assert [
        d.id
        for d in top_down(
            boxes_to_detection3d_array([floor], to_ros=same, frame_id="world", ts=0.0)
        ).detections
    ] == ["floor"]


def test_top_down_rejects_oriented_boxes() -> None:
    arr = boxes_to_detection3d_array([LAMP], to_ros=same, frame_id="world", ts=0.0)
    arr.detections[0].bbox.center.orientation.z = 1.0
    arr.detections[0].bbox.center.orientation.w = 0.0

    with pytest.raises(ValueError, match="oriented"):
        top_down(arr)


def test_top_down_view_and_json_round_trip(tmp_path: Path) -> None:
    flat = top_down(boxes_to_detection3d_array([BOX, LAMP], to_ros=flip, frame_id="world", ts=2.5))

    view = detection2d_array_to_dict(flat, provenance={"dataset": "hssd-hab", "scene_id": "s1"})

    assert list(view) == [
        "dataset",
        "scene_id",
        "projection",
        "frame_id",
        "timestamp",
        "count",
        "detections",
    ]
    assert view["projection"] == "top_down_xy"
    assert view["count"] == 2
    assert view["detections"][0] == {
        "id": "sofa",
        "label": "sofa",
        "score": 1.0,
        "center_xy": [-4.5, -1.5],
        "size_xy": [3.0, 1.0],
        "theta": 0.0,
        "labels": ["sofa", "Sectional sofa"],
    }
    assert "labels" not in view["detections"][1]

    text = write_detection2d_json(flat, tmp_path / "flat.json", provenance={"scene_id": "s1"})
    assert json.loads(text.read_text())["detections"][1]["center_xy"] == [-0.5, -0.5]
    decoded = Detection2DArray.lcm_decode(flat.lcm_encode())
    assert decoded.detections_length == 2
    assert decoded.detections[0].bbox.size_x == 3.0
