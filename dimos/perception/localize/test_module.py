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

"""PerceptionLocalize as the scene a pick reads, off the models and the ports."""

from __future__ import annotations

import threading
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.manipulation.manipulation_spec import PickSceneSpec
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.localize.module import (
    SCAN_WINDOW_S,
    WATCH_FORGET_S,
    WATCH_WINDOW_S,
    PerceptionLocalize,
    _image_box,
    as_detection_2d_array,
)
from dimos.perception.localize.types import Localization, LocalizeParams
from dimos.protocol.encode.cbor import decode, encode
from dimos.spec.utils import spec_annotation_compliance

WORLD = "odom"


def _hit(
    instance_id: str,
    score: float = 0.5,
    at: tuple[float, float, float] = (0.5, 0.0, 1.0),
    seen: float = 10.0,
) -> Localization:
    return Localization(
        instance_id=instance_id,
        semantic_score=score,
        ambiguity_margin=0.1,
        position_world_xyz=at,
        frame_id=WORLD,
        last_seen_timestamp=seen,
        point_cloud=PointCloud2.from_numpy(
            np.array([[0.5, 0.0, 1.0], [0.6, 0.1, 1.1]], dtype=np.float32),
            frame_id=WORLD,
            timestamp=10.0,
        ),
        n_views=1,
    )


def _localize(answers: list[list[Localization]]) -> Any:
    """What scan_scene reads off the module, with the detector's answer in place of the window."""
    ready = threading.Event()
    ready.set()
    window = MagicMock(
        side_effect=lambda objects, *_: (
            [q for q in objects.split(",") if q],
            answers,
            None,
            SimpleNamespace(world_frame=WORLD),
            (0.0, 0.0, 0.0),
        )
    )
    return SimpleNamespace(
        _ready=ready, _stage="ready", _groups={}, _scanned={}, _watched={}, _localize_window=window
    )


@pytest.mark.parametrize("scene", [PerceptionLocalize, ObjectSceneRegistrationModule])
def test_either_perception_module_is_a_pick_scene(scene: type) -> None:
    assert spec_annotation_compliance(scene, PickSceneSpec)


def test_the_models_get_a_process_of_their_own() -> None:
    assert PerceptionLocalize.dedicated_worker


def test_a_scan_names_each_instance_and_keeps_its_cloud_by_id() -> None:
    can, tape = _hit("can-1"), _hit("tape-1")
    module = _localize([[can], [tape]])
    scan = PerceptionLocalize.scan_scene(module, [" can ", "tape"])

    assert [(d.id, d.results[0].hypothesis.class_id) for d in scan.detections] == [
        ("can-1", "can"),
        ("tape-1", "tape"),
    ]
    assert scan.detections_length == 2
    cloud = PerceptionLocalize.get_object_pointcloud_by_object_id(module, "tape-1")
    assert cloud is tape.point_cloud and cloud.frame_id == WORLD, "PickAndPlace re-expresses it"
    assert PerceptionLocalize.get_object_pointcloud_by_object_id(module, "nope") is None


def test_a_scan_lists_its_objects_the_way_the_scene_registration_does() -> None:
    """The Manipulation app finds its scene by this RPC and draws each object from these fields."""
    module = _localize([[_hit("can-1")]])
    assert PerceptionLocalize.get_detected_objects(module) == []
    PerceptionLocalize.scan_scene(module, ["can"])
    (seen,) = PerceptionLocalize.get_detected_objects(module)
    assert (seen["object_id"], seen["name"], seen["frame_id"]) == ("can-1", "can", WORLD)
    assert seen["confidence"] == 0.5 and seen["detections"] == 1
    assert seen["center"] == pytest.approx([0.55, 0.05, 1.05])
    assert seen["size"] == pytest.approx([0.1, 0.1, 0.1])
    assert seen["last_seen"].endswith("s ago")


def test_a_scan_reads_only_the_last_moments() -> None:
    module = _localize([[]])
    PerceptionLocalize.scan_scene(module, ["can"])
    module._localize_window.assert_called_once_with("can", -SCAN_WINDOW_S, SCAN_WINDOW_S, "")


def test_a_scan_forgets_what_earlier_calls_proved_of_its_labels() -> None:
    """Groups keep an object answerable after it leaves the window: a picked one would be
    scanned where it stood, and the next pick sent there."""
    module = _localize([[]])
    module._groups = {"can": "proved", "tape": "kept"}
    PerceptionLocalize.scan_scene(module, ["can"])
    assert module._groups == {"tape": "kept"}


def test_the_next_scan_replaces_the_last() -> None:
    module = _localize([[_hit("can-1")]])
    PerceptionLocalize.scan_scene(module, ["can"])
    module._localize_window.side_effect = lambda *_: (
        ["can"],
        [[_hit("can-2")]],
        None,
        SimpleNamespace(world_frame=WORLD),
        (0.0, 0.0, 0.0),
    )
    PerceptionLocalize.scan_scene(module, ["can"])
    assert set(module._scanned) == {"can-2"}


def test_a_scan_before_the_models_are_up_says_what_it_is_waiting_for() -> None:
    module = _localize([])
    module._ready.clear()
    module._stage = "loading siglip, owlv2, edgetam weights"
    with pytest.raises(RuntimeError, match="loading siglip"):
        PerceptionLocalize.scan_scene(module, ["can"])


def test_no_labels_is_no_objects() -> None:
    module = _localize([])
    assert PerceptionLocalize.scan_scene(module, None).detections_length == 0


def test_a_pick_runs_without_a_registered_scene() -> None:
    """PickAndPlace skips its grasp contact check on None."""
    assert PerceptionLocalize.get_full_scene_pointcloud(_localize([]), "can-1") is None


def test_a_scan_survives_the_trip_to_the_pick_module() -> None:
    """PickAndPlace is a module of its own: both answers cross the RPC codec."""
    module = _localize([[_hit("can-1")]])
    scan = decode(encode(PerceptionLocalize.scan_scene(module, ["can"])))
    assert [(d.id, d.results[0].hypothesis.class_id) for d in scan.detections] == [("can-1", "can")]
    cloud = decode(encode(PerceptionLocalize.get_object_pointcloud_by_object_id(module, "can-1")))
    assert cloud.frame_id == WORLD and len(cloud) == 2


CAMERA = CameraInfo(
    height=480,
    width=640,
    K=[100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0],
    frame_id="camera_optical",
)


def _watcher() -> Any:
    """What a watch pass reads off the module; ``_watch_once`` hands it the detector's answer."""
    return SimpleNamespace(
        config=SimpleNamespace(params=LocalizeParams(), watch=["can", "box"]),
        index=SimpleNamespace(get_time_range=lambda: (0.0, 10.0)),
        _detect=MagicMock(),
        _watch=["can", "box"],
        _watched={},
        _scanned={},
        _camera=CAMERA,
        detections=MagicMock(),
        detections_2d=MagicMock(),
    )


def _watch_once(
    module: Any, results: list[list[Localization]], head: float = 10.0
) -> list[tuple[str, str]]:
    """One pass with this detector answer: the ``(id, label)`` it published, in order."""
    rig = SimpleNamespace(world_frame=WORLD, world_to_optical=lambda *_: Transform())
    module.index.get_time_range = lambda: (0.0, head)
    module._detect.return_value = (results, None, rig)
    PerceptionLocalize._watch_pass(module, ["can", "box"])
    published = module.detections.publish.call_args.args[0]
    return [(d.id, d.results[0].hypothesis.class_id) for d in published.detections]


def test_a_watch_pass_reads_the_newest_frame_from_nothing() -> None:
    """A cloud folded over many frames would keep a nudged object where it stood."""
    module = _watcher()
    _watch_once(module, [[], []])
    module._detect.assert_called_once_with(
        ["can", "box"], 10.0 - WATCH_WINDOW_S, WATCH_WINDOW_S, {"min_views": 1}, {}
    )


def test_one_object_under_two_labels_is_one_row_under_the_better_one() -> None:
    can = _hit("as-can", score=0.3, at=(0.5, 0.0, 1.0))
    box = _hit("as-box", score=0.6, at=(0.52, 0.0, 1.0))
    assert _watch_once(_watcher(), [[can], [box]]) == [("as-box", "box")]


def test_two_objects_apart_are_two_rows_best_first() -> None:
    can = _hit("can-1", score=0.3, at=(0.5, 0.0, 1.0))
    box = _hit("box-1", score=0.6, at=(0.5, 0.3, 1.0))
    assert _watch_once(_watcher(), [[can], [box]]) == [("box-1", "box"), ("can-1", "can")]


def test_an_object_keeps_its_id_from_pass_to_pass_but_takes_the_new_frame() -> None:
    module = _watcher()
    _watch_once(module, [[_hit("first")], []])
    newer = _hit("second", at=(0.51, 0.0, 1.0), seen=12.0)
    assert _watch_once(module, [[newer], []], head=12.0) == [("first", "can")]
    (_, kept) = module._watched["first"]
    assert kept.point_cloud is newer.point_cloud and kept.last_seen_timestamp == 12.0


def test_an_object_moved_past_the_radius_is_a_new_object() -> None:
    module = _watcher()
    _watch_once(module, [[_hit("here")], []])
    moved = _hit("there", at=(0.8, 0.0, 1.0), seen=12.0)
    assert {i for i, _ in _watch_once(module, [[moved], []], head=12.0)} == {"here", "there"}


def test_a_frame_that_misses_an_object_keeps_it_until_it_is_forgotten() -> None:
    module = _watcher()
    _watch_once(module, [[_hit("can-1", seen=10.0)], []], head=10.0)
    assert _watch_once(module, [[], []], head=10.0 + WATCH_FORGET_S) == [("can-1", "can")]
    assert _watch_once(module, [[], []], head=10.1 + WATCH_FORGET_S) == []
    assert module._watched == {}


def test_nothing_seen_still_goes_out_so_the_boxes_clear() -> None:
    module = _watcher()
    assert _watch_once(module, [[], []]) == []
    assert module.detections_2d.publish.call_args.args[0].detections_length == 0


def test_a_pick_reads_a_watched_object_by_its_id() -> None:
    module = _watcher()
    can = _hit("can-1")
    _watch_once(module, [[can], []])
    assert PerceptionLocalize.get_object_pointcloud_by_object_id(module, "can-1") is can.point_cloud


def test_new_labels_replace_the_watch_and_none_go_back_to_the_config() -> None:
    module = _watcher()
    PerceptionLocalize._on_watch_labels(module, String(" cucumber, ,tape "))
    assert module._watch == ["cucumber", "tape"]
    PerceptionLocalize._on_watch_labels(module, String(""))
    assert module._watch == ["can", "box"]


def test_a_box_is_the_cloud_as_the_camera_sees_it() -> None:
    points = np.array([[0.0, 0.0, 1.0], [0.1, 0.2, 1.0]])
    corners = _image_box(points, Transform().to_matrix(), CAMERA)
    assert corners is not None
    assert corners[0] == pytest.approx([320.0, 240.0])
    assert corners[1] == pytest.approx([330.0, 260.0])


def test_a_box_follows_the_camera() -> None:
    """Two metres further from the camera, the same 1 m offset lands a third as far from centre."""
    points = np.array([[1.0, 0.0, 1.0], [1.3, 0.3, 1.0]])
    back = Transform(translation=Vector3(0.0, 0.0, 2.0)).to_matrix()
    corners = _image_box(points, back, CAMERA)
    assert corners is not None
    assert corners[0] == pytest.approx([320.0 + 100.0 / 3.0, 240.0])
    assert corners[1] == pytest.approx([320.0 + 130.0 / 3.0, 250.0])


@pytest.mark.parametrize(
    "points",
    [
        pytest.param([[0.0, 0.0, -1.0]], id="behind the camera"),
        pytest.param([[50.0, 0.0, 1.0], [51.0, 0.1, 1.0]], id="past the image's edge"),
    ],
)
def test_nothing_in_the_image_is_no_box(points: list[list[float]]) -> None:
    assert _image_box(np.array(points), Transform().to_matrix(), CAMERA) is None


def test_a_box_cut_by_the_edge_stops_at_the_edge() -> None:
    corners = _image_box(np.array([[3.0, 0.0, 1.0], [4.0, 0.1, 1.0]]), np.eye(4), CAMERA)
    assert corners is not None
    assert corners[1][0] == 640.0


def test_the_boxes_in_view_survive_the_bus() -> None:
    can = _hit("can-1", at=(0.0, 0.0, 1.0))
    boxes = as_detection_2d_array([("can", can)], 10.0, np.eye(4), CAMERA)
    (box,) = Detection2DArray.lcm_decode(boxes.lcm_encode()).detections
    assert box.id == "can-1" and box.results[0].hypothesis.class_id == "can"
    assert box.bbox.center.position.x == pytest.approx(372.5, abs=1.0)
    assert box.bbox.size_x > 0 and box.bbox.size_y > 0


def test_the_detector_runs_one_call_at_a_time() -> None:
    """The watch runs beside the RPCs, and ``groups`` is not safe to share."""
    module: Any = SimpleNamespace(
        config=SimpleNamespace(params=LocalizeParams()),
        index=MagicMock(),
        rig=SimpleNamespace(lidar=None),
        detector=MagicMock(),
        _detecting=threading.Lock(),
    )
    held: list[bool] = []
    module.detector.localize.side_effect = lambda *_, **__: held.append(module._detecting.locked())
    PerceptionLocalize._detect(module, ["can"], 0.0, 1.0, None, {})
    assert held == [True] and not module._detecting.locked()
