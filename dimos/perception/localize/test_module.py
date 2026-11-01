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
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.localize.module import SCAN_WINDOW_S, PerceptionLocalize
from dimos.perception.localize.types import Localization
from dimos.protocol.encode.cbor import decode, encode
from dimos.spec.utils import spec_annotation_compliance

WORLD = "odom"


def _hit(instance_id: str) -> Localization:
    return Localization(
        instance_id=instance_id,
        semantic_score=0.5,
        ambiguity_margin=0.1,
        position_world_xyz=(0.5, 0.0, 1.0),
        frame_id=WORLD,
        last_seen_timestamp=10.0,
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
        _ready=ready, _stage="ready", _groups={}, _scanned={}, _localize_window=window
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
