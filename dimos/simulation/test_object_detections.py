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

from dimos.simulation.object_detections import (
    GroundTruthBox,
    Point3,
    boxes_to_detection3d_array,
    detection3d_array_to_dict,
    read_detection3d_array,
    ros_box,
    write_detection3d_array,
    write_detection3d_json,
)


def flip(x: float, y: float, z: float) -> Point3:
    """Habitat-style permutation with sign flips on two axes."""
    return (-z, -x, y)


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
