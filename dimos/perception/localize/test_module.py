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
from reactivex.disposable import Disposable

from dimos.memory.store.memory import MemoryStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.memento import Memento
from dimos.perception.localize.module import PerceptionLocalize


@pytest.fixture
def localizer(mocker):
    module = PerceptionLocalize()
    for name in ("tf", "color_image", "depth_image", "lidar", "camera_info"):
        mocker.patch.object(
            getattr(module, name), "subscribe", return_value=mocker.Mock(name=f"unsubscribe_{name}")
        )
    mocker.patch("dimos.perception.localize.module.DanDetector", autospec=True)
    yield module
    module.stop()


def test_stop_without_start_disposes_registered_resources(localizer, mocker):
    cleanup = mocker.Mock()
    localizer.register_disposable(Disposable(cleanup))

    localizer.stop()

    cleanup.assert_called_once_with()


@pytest.mark.parametrize("geometry", ["depth", "lidar"])
def test_stop_after_subscription_failure_releases_earlier_resources(localizer, mocker, geometry):
    localizer.config.geometry = geometry
    close = mocker.spy(Memento, "close")
    stop_memory = mocker.spy(MemoryStore, "stop")
    localizer.camera_info.subscribe.side_effect = RuntimeError("subscription failed")

    with pytest.raises(RuntimeError, match="subscription failed"):
        localizer.start()
    localizer.stop()
    localizer.stop()

    localizer.tf.subscribe.return_value.assert_called_once_with()
    localizer.color_image.subscribe.return_value.assert_called_once_with()
    geometry_stream = localizer.depth_image if geometry == "depth" else localizer.lidar
    geometry_stream.subscribe.return_value.assert_called_once_with()
    close.assert_called_once_with(localizer._memento)
    stop_memory.assert_called_once_with(localizer._memory)


def test_stop_after_memory_setup_failure_closes_database(localizer, mocker):
    close = mocker.spy(Memento, "close")
    mocker.patch.object(MemoryStore, "stream", side_effect=RuntimeError("memory setup failed"))

    with pytest.raises(RuntimeError, match="memory setup failed"):
        localizer.start()
    localizer.stop()

    close.assert_called_once_with(localizer._memento)


def test_stop_after_thread_start_failure_releases_resources(localizer, mocker):
    close = mocker.spy(Memento, "close")
    start_thread = mocker.patch.object(
        threading.Thread, "start", side_effect=RuntimeError("thread start failed")
    )

    with pytest.raises(RuntimeError, match="thread start failed"):
        localizer.start()
    mocker.stop(start_thread)
    localizer.stop()

    localizer.camera_info.subscribe.return_value.assert_called_once_with()
    close.assert_called_once_with(localizer._memento)


def test_stop_after_start_releases_resources_once(localizer, mocker):
    close = mocker.spy(Memento, "close")

    localizer.start()
    localizer.stop()
    localizer.stop()

    localizer.tf.subscribe.return_value.assert_called_once_with()
    localizer.color_image.subscribe.return_value.assert_called_once_with()
    localizer.depth_image.subscribe.return_value.assert_called_once_with()
    localizer.camera_info.subscribe.return_value.assert_called_once_with()
    close.assert_called_once_with(localizer._memento)
    assert not localizer._thread.is_alive()


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


def test_objects_of_the_last_scan_are_found_by_id_or_unique_label(localizer, mocker):
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
    with pytest.raises(ValueError, match="block-1, block-2"):
        localizer.get_pointcloud("block")
    with pytest.raises(LookupError, match="No 'bowl'"):
        localizer.get_pointcloud("bowl")


def test_scene_cloud_merges_the_window_and_leaves_out_the_excluded_object(localizer, mocker):
    table = [[x, y, 0.0] for x in (0.0, 0.5, 1.0) for y in (0.0, 0.5, 1.0)]
    _ready(localizer, mocker, [_cloud(table), _cloud([[0.5, 0.5, 0.05]])])
    localizer._objects = {"cup-1": ("cup", _hit("cup-1", [[0.5, 0.5, 0.05]]))}
    localizer.config.scene_voxel_m = 0.001

    full = localizer.scene_cloud(start=-5.0, duration=5.0)
    without_cup = localizer.scene_cloud("cup", start=-5.0, duration=5.0)

    assert len(full) == 10
    # Only the cup goes: the table 5 cm under it is outside its box grown by 2 cm.
    assert len(without_cup) == 9
    assert localizer.index.time_range.call_args.args == (5.0, 10.0)


def test_scene_cloud_is_none_before_ready_or_without_depth(localizer, mocker):
    assert localizer.scene_cloud() is None
    _ready(localizer, mocker, [_cloud([[0.0, 0.0, 0.0]])])
    localizer.config.geometry = "lidar"

    assert localizer.scene_cloud() is None


def test_scan_refuses_until_ready(localizer):
    localizer._ready = threading.Event()
    localizer._stage = "loading weights"

    with pytest.raises(RuntimeError, match="loading weights"):
        localizer.scan(["cup"])


def test_scan_reports_each_instance_and_replaces_the_last_answer(localizer, mocker):
    _ready(localizer, mocker, [])
    localizer._memento = mocker.Mock()
    localizer._color_feed = mocker.Mock()
    localizer.rig.lidar = None
    localizer.rig.cameras = {}
    localizer.index.time_range.return_value = mocker.Mock(count=mocker.Mock(return_value=4))
    localizer.detections = mocker.Mock()
    mocker.patch("dimos.perception.localize.module.as_detection_array")
    localizer._objects = {"old-1": ("old", _hit("old-1", [[0.0, 0.0, 0.0]]))}
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
    with pytest.raises(LookupError):
        localizer.get_pointcloud("old-1")
