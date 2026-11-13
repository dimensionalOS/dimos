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

import threading
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def localizer(mocker):
    module = ManipulationLocalize()
    for name in ("tf", "color_image", "depth_image", "lidar", "camera_info"):
        mocker.patch.object(
            getattr(module, name), "subscribe", return_value=mocker.Mock(name=f"unsubscribe_{name}")
        )
    mocker.patch("dimos.perception.localize.module.DanDetector", autospec=True)
    yield module
    module.stop()


def _cloud(points):
    return PointCloud2.from_numpy(np.asarray(points, dtype=np.float32), frame_id="world")


def _ready(module, mocker, views):
    """A started module whose index holds one frame per view and whose rig backdrops them."""
    module._ready = threading.Event()
    module._ready.set()
    module._objects = {}
    module.index = mocker.Mock()
    module.index.get_time_range.return_value = (0.0, 10.0)
    module.index.time_range.return_value = [SimpleNamespace(ts=float(i)) for i in range(len(views))]
    module.rig = mocker.Mock(cameras={"camera_color_optical_frame": None}, world_frame="world")
    module.rig.backdrop.side_effect = lambda ts, frame, trunc: views[int(ts)]


def _hit(instance_id, points, position=(0.0, 0.0, 0.0)):
    return SimpleNamespace(
        instance_id=instance_id, point_cloud=_cloud(points), position_world_xyz=position
    )


def test_localized_objects_are_found_by_id_or_unique_label(localizer, mocker):
    _ready(localizer, mocker, [])
    cup = _hit("cup-1", [[0.0, 0.0, 0.0]], (0.4, 0.1, 0.2))
    localizer._objects = {
        "cup-1": ("cup", cup),
        "block-1": ("block", _hit("block-1", [[1.0, 0.0, 0.0]])),
        "block-2": ("block", _hit("block-2", [[2.0, 0.0, 0.0]])),
    }

    assert localizer.get_pointcloud("cup-1") is cup.point_cloud
    assert localizer.get_pointcloud("cup") is cup.point_cloud
    assert localizer.get_position("cup").x == pytest.approx(0.4)
    assert localizer.get_id("cup") == "cup-1"
    with pytest.raises(ValueError, match="block-1, block-2"):
        localizer.get_pointcloud("block")
    with pytest.raises(LookupError, match=r"No 'bowl' among the localized objects\. They are: cup"):
        localizer.get_pointcloud("bowl")
    localizer._objects = {}
    with pytest.raises(LookupError, match="no object is localized"):
        localizer.get_pointcloud("cup")


def test_a_forgotten_object_is_no_longer_found(localizer, mocker):
    _ready(localizer, mocker, [])
    bowl = _hit("bowl-1", [[1.0, 0.0, 0.0]])
    localizer._objects = {
        "cup-1": ("cup", _hit("cup-1", [[0.0, 0.0, 0.0]])),
        "bowl-1": ("bowl", bowl),
    }

    localizer.forget("cup-1")

    assert localizer.get_pointcloud("bowl") is bowl.point_cloud
    with pytest.raises(LookupError, match="No 'cup-1'"):
        localizer.get_pointcloud("cup-1")


def test_scene_cloud_merges_the_views_of_the_window(localizer, mocker):
    table = [[x, 0.0, 0.0] for x in (0.0, 0.5, 1.0)]
    _ready(localizer, mocker, [_cloud(table), _cloud([[0.5, 0.0, 0.02], [0.5, 0.0, 0.04]])])
    localizer.config.scene_voxel_m = 0.001

    assert len(localizer.scene_cloud(start=-5.0, duration=5.0)) == 5
    assert localizer.index.time_range.call_args.args == (5.0, 10.0)


def test_scene_cloud_is_none_before_ready_or_without_depth(localizer, mocker):
    localizer._ready = threading.Event()
    assert localizer.scene_cloud() is None
    _ready(localizer, mocker, [_cloud([[0.0, 0.0, 0.0]])])
    localizer.config.geometry = "lidar"

    assert localizer.scene_cloud() is None


def test_scan_refuses_until_ready(localizer):
    localizer._ready = threading.Event()
    localizer._stage = "loading weights"

    with pytest.raises(RuntimeError, match="loading weights"):
        localizer.scan(["cup"])


def test_scan_replaces_the_objects_of_the_names_asked_and_keeps_the_others(localizer, mocker):
    _ready(localizer, mocker, [])
    localizer._memento = mocker.Mock()
    localizer._color_feed = mocker.Mock()
    localizer.rig.lidar = None
    localizer.rig.cameras = {}
    localizer.index.time_range.return_value = mocker.Mock(count=mocker.Mock(return_value=4))
    localizer.detections = mocker.Mock()
    mocker.patch("dimos.perception.localize.module.as_detection_array")
    block = _hit("block-1", [[0.0, 0.0, 0.0]])
    localizer._objects = {
        "block-1": ("block", block),
        "cup-0": ("cup", _hit("cup-0", [[0.0, 0.0, 0.0]])),
    }
    cup = _hit("cup-1", [[0.4, 0.1, 0.2], [0.5, 0.2, 0.3]], (0.45, 0.15, 0.25))
    cup.semantic_score, cup.n_views, cup.last_seen_timestamp = 0.8, 3, 9.0
    localizer.detector = mocker.Mock()
    localizer.detector.localize.return_value = [[cup], []]

    found = localizer.scan(["cup", "bowl"], window_s=4.0)

    assert found == [
        {
            "id": "cup-1",
            "label": "cup",
            "position": [0.45, 0.15, 0.25],
            "size": pytest.approx([0.1, 0.1, 0.1]),
            "score": 0.8,
            "views": 3,
            "seen_s_ago": 1.0,
        }
    ]
    assert localizer.detector.localize.call_args.kwargs["window"] == (6.0, 10.0)
    assert localizer.get_pointcloud("block") is block.point_cloud
    with pytest.raises(LookupError):
        localizer.get_pointcloud("cup-0")


def test_a_forgotten_object_does_not_come_back_from_its_old_sightings(localizer, mocker):
    _ready(localizer, mocker, [])
    localizer._memento = mocker.Mock()
    localizer._color_feed = mocker.Mock()
    localizer.rig.lidar = None
    localizer.rig.cameras = {}
    localizer.index.time_range.return_value = mocker.Mock(count=mocker.Mock(return_value=4))
    localizer.detections = mocker.Mock()
    mocker.patch("dimos.perception.localize.module.as_detection_array")
    localizer._objects = {"cup-1": ("cup", _hit("cup-1", [[0.4, 0.1, 0.2]], (0.4, 0.1, 0.2)))}
    # Picked when the newest frame was at 10 s.
    localizer.forget("cup-1")
    # Later the window still holds the cup at its old place, and the cup where it was put.
    old = _hit("cup-1", [[0.4, 0.1, 0.2]], (0.41, 0.1, 0.2))
    put = _hit("cup-2", [[0.3, -0.2, 0.2]], (0.3, -0.2, 0.2))
    back = _hit("cup-3", [[0.4, 0.1, 0.2]], (0.4, 0.1, 0.2))
    for hit, seen in ((old, 9.0), (put, 14.0), (back, 15.0)):
        hit.semantic_score, hit.n_views, hit.last_seen_timestamp = 0.8, 2, seen
    localizer.index.get_time_range.return_value = (0.0, 16.0)
    localizer.detector = mocker.Mock()
    localizer.detector.localize.return_value = [[old, put, back]]

    found = localizer.scan(["cup"], window_s=30.0)

    # The old sighting goes; one seen at the old place after the pick stays.
    assert [o["id"] for o in found] == ["cup-2", "cup-3"]
