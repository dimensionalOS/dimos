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
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.simulation.dimsim.object_detections import (
    DIMSIM_WORLD_FRAME,
    read_detection3d_array,
    snapshot_to_detection3d_array,
    write_detection3d_array,
    write_detection3d_json,
)
from dimos.simulation.dimsim.scene_client import SceneClient

# Three.js Y-up boxes as the browser reports them; ROS expectations are (z, x, y).
CAPTURED_AT_MS = 1_758_000_000_500
SNAPSHOT: dict[str, Any] = {
    "capturedAt": CAPTURED_AT_MS,
    "displayed": True,
    "objects": [
        {"id": "sofa", "label": "Sectional sofa", "min": [1.0, 0.0, -3.0], "max": [3.0, 1.0, -1.0]},
        {"id": "lamp", "label": "Floor lamp", "min": [-0.5, 0.0, 2.0], "max": [-0.1, 1.8, 2.4]},
    ],
}


def _xyz(v: Any) -> tuple[float, float, float]:
    return (v.x, v.y, v.z)


def test_center_and_size_are_permuted_to_ros_z_up() -> None:
    sofa, lamp = snapshot_to_detection3d_array(SNAPSHOT).detections

    # Three.js center (2, 0.5, -2) and size (2, 1, 2) become (z, x, y).
    assert _xyz(sofa.bbox.center.position) == (-2.0, 2.0, 0.5)
    assert _xyz(sofa.bbox.size) == (2.0, 2.0, 1.0)
    # Three.js center (-0.3, 0.9, 2.2) and size (0.4, 1.8, 0.4).
    assert _xyz(lamp.bbox.center.position) == pytest.approx((2.2, -0.3, 0.9))
    assert _xyz(lamp.bbox.size) == pytest.approx((0.4, 0.4, 1.8))

    q = sofa.bbox.center.orientation
    assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)


def test_ids_labels_scores_frames_and_timestamps() -> None:
    arr = snapshot_to_detection3d_array(SNAPSHOT)

    assert arr.detections_length == 2
    assert [d.id for d in arr.detections] == ["sofa", "lamp"]
    assert [d.results_length for d in arr.detections] == [1, 1]
    assert [d.results[0].hypothesis.class_id for d in arr.detections] == [
        "Sectional sofa",
        "Floor lamp",
    ]
    assert [d.results[0].hypothesis.score for d in arr.detections] == [1.0, 1.0]

    assert arr.frame_id == DIMSIM_WORLD_FRAME
    assert arr.ts == pytest.approx(CAPTURED_AT_MS / 1000.0)
    for d in arr.detections:
        assert d.frame_id == DIMSIM_WORLD_FRAME
        assert d.ts == pytest.approx(arr.ts)
        assert _xyz(d.results[0].pose.pose.position) == _xyz(d.bbox.center.position)


def test_detections_do_not_share_mutable_state() -> None:
    arr = snapshot_to_detection3d_array(SNAPSHOT)
    sofa, lamp = arr.detections

    sofa.header.frame_id = "elsewhere"
    sofa.bbox.size.x = 99.0
    sofa.bbox.center.position.x = 123.0
    sofa.results[0].hypothesis.class_id = "changed"
    sofa.results.append(sofa.results[0])

    assert arr.header.frame_id == DIMSIM_WORLD_FRAME
    assert lamp.header.frame_id == DIMSIM_WORLD_FRAME
    assert lamp.bbox.size.x == pytest.approx(0.4)
    assert lamp.results[0].hypothesis.class_id == "Floor lamp"
    assert len(lamp.results) == 1
    # The hypothesis pose is a copy of the box center, not the same object.
    assert sofa.results[0].pose.pose.position.x == -2.0

    empty = snapshot_to_detection3d_array({"capturedAt": 0, "objects": []})
    assert empty.detections_length == 0
    assert empty.detections is not Detection3DArray().detections


@pytest.mark.parametrize(
    "snapshot",
    [
        {"objects": []},
        {"capturedAt": float("nan"), "objects": []},
        {"capturedAt": 1.0, "objects": None},
        {
            "capturedAt": 1.0,
            "objects": [{"id": "", "label": "x", "min": [0, 0, 0], "max": [1, 1, 1]}],
        },
        {
            "capturedAt": 1.0,
            "objects": [{"id": "a", "label": "", "min": [0, 0, 0], "max": [1, 1, 1]}],
        },
        {
            "capturedAt": 1.0,
            "objects": [{"id": "a", "label": "x", "min": [0, 0], "max": [1, 1, 1]}],
        },
        {
            "capturedAt": 1.0,
            "objects": [{"id": "a", "label": "x", "min": [0, 0, 0], "max": [1, float("inf"), 1]}],
        },
        {
            "capturedAt": 1.0,
            "objects": [{"id": "a", "label": "x", "min": [0, 2, 0], "max": [1, 1, 1]}],
        },
    ],
)
def test_malformed_snapshot_is_rejected(snapshot: dict[str, Any]) -> None:
    with pytest.raises(ValueError):
        snapshot_to_detection3d_array(snapshot)


def test_binary_file_round_trip(tmp_path: Path) -> None:
    arr = snapshot_to_detection3d_array(SNAPSHOT)
    out = write_detection3d_array(arr, tmp_path / "objects.bin")

    assert out.read_bytes() == arr.lcm_encode()
    decoded = read_detection3d_array(out)
    assert isinstance(decoded, Detection3DArray)
    assert decoded.frame_id == DIMSIM_WORLD_FRAME
    assert decoded.ts == pytest.approx(CAPTURED_AT_MS / 1000.0)
    assert [d.id for d in decoded.detections] == ["sofa", "lamp"]
    assert [d.results[0].hypothesis.class_id for d in decoded.detections] == [
        "Sectional sofa",
        "Floor lamp",
    ]
    sofa = decoded.detections[0]
    assert _xyz(sofa.bbox.center.position) == (-2.0, 2.0, 0.5)
    assert _xyz(sofa.bbox.size) == (2.0, 2.0, 1.0)


def test_json_view_lists_label_center_and_size(tmp_path: Path) -> None:
    arr = snapshot_to_detection3d_array(SNAPSHOT)
    out = write_detection3d_json(arr, tmp_path / "objects.json")

    view = json.loads(out.read_text())
    assert view["frame_id"] == DIMSIM_WORLD_FRAME
    assert view["timestamp"] == pytest.approx(CAPTURED_AT_MS / 1000.0)
    assert view["count"] == 2
    assert view["detections"][0] == {
        "id": "sofa",
        "label": "Sectional sofa",
        "score": 1.0,
        "center_xyz": [-2.0, 2.0, 0.5],
        "size_xyz": [2.0, 2.0, 1.0],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    assert view["detections"][1]["id"] == "lamp"


def test_scene_client_export_picks_json_by_suffix(mocker: MockerFixture, tmp_path: Path) -> None:
    client = SceneClient()
    mocker.patch.object(client, "exec", return_value=SNAPSHOT)
    path = tmp_path / "objects.JSON"

    client.export_object_detections(path)

    assert json.loads(path.read_text())["count"] == 2


def test_scene_client_get_object_detections_uses_sandbox_helper(mocker: MockerFixture) -> None:
    client = SceneClient()
    exec_mock = mocker.patch.object(client, "exec", return_value=SNAPSHOT)

    arr = client.get_object_detections()

    exec_mock.assert_called_once_with("return getObjectAnnotations();")
    assert isinstance(arr, Detection3DArray)
    assert [d.id for d in arr.detections] == ["sofa", "lamp"]


def test_scene_client_export_writes_one_encoded_message(
    mocker: MockerFixture, tmp_path: Path
) -> None:
    client = SceneClient()
    mocker.patch.object(client, "exec", return_value=SNAPSHOT)
    path = tmp_path / "objects.bin"

    arr = client.export_object_detections(path)

    assert path.read_bytes() == arr.lcm_encode()
    assert read_detection3d_array(path).detections_length == 2
