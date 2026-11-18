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

from dimos.manipulation.manipulation_localize import (
    TABLE,
    ManipulationLocalize,
    pick_table,
    table_boxes,
)
from dimos.mapping.pointclouds.box_filter import inside_boxes
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.transform_utils import pose_to_matrix


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
    # A 4 cm cup at (0.4, 0.1), picked when the newest frame was at 10 s.
    cup = [[0.38, 0.08, 0.18], [0.42, 0.12, 0.22]]
    localizer._objects = {"cup-1": ("cup", _hit("cup-1", cup, (0.4, 0.1, 0.2)))}
    localizer.forget("cup-1")
    # Later the window holds the cup at its old place, under its old id and under a new
    # one; the cup where it was put; a cup seen at the old place after the pick; and a
    # second cup 5 cm away, last seen before the pick.
    same_id = _hit("cup-1", cup, (0.4, 0.1, 0.2))
    new_id = _hit("cup-2", cup, (0.41, 0.1, 0.2))
    put = _hit("cup-3", [[0.3, -0.2, 0.2]], (0.3, -0.2, 0.2))
    back = _hit("cup-4", cup, (0.4, 0.1, 0.2))
    neighbour = _hit("cup-5", [[0.45, 0.1, 0.2]], (0.45, 0.1, 0.2))
    for hit, seen in ((same_id, 9.0), (new_id, 9.0), (put, 14.0), (back, 15.0), (neighbour, 8.0)):
        hit.semantic_score, hit.n_views, hit.last_seen_timestamp = 0.8, 2, seen
    localizer.index.get_time_range.return_value = (0.0, 16.0)
    localizer.detector = mocker.Mock()
    localizer.detector.localize.return_value = [[same_id, new_id, put, back, neighbour]]

    found = localizer.scan(["cup"], window_s=30.0)

    assert [o["id"] for o in found] == ["cup-3", "cup-4", "cup-5"]


def test_find_surface_reports_the_table_in_the_base_frame(localizer, mocker):
    rng = np.random.default_rng(0)
    x, y = np.meshgrid(np.arange(0.1, 0.9, 0.01), np.arange(-0.25, 0.25, 0.01))
    table = np.stack([x.ravel(), y.ravel(), np.zeros(x.size)], axis=1)
    table += rng.normal(0.0, 0.002, table.shape)
    _ready(localizer, mocker, [None] * 5)
    localizer.rig.backdrop.side_effect = None
    localizer.rig.backdrop.return_value = PointCloud2.from_numpy(
        table.astype(np.float32), frame_id="world", timestamp=1.0
    )
    # The base stands 0.3 m above the world origin: world to base lowers every point.
    localizer.rig.tf.get.return_value = Transform(
        translation=Vector3(0.0, 0.0, -0.3), frame_id="link_base", child_frame_id="world"
    )

    answer = localizer.find_surface(frame="link_base")

    localizer.rig.tf.get.assert_called_once_with("link_base", "world", 10.0, mocker.ANY)
    assert answer.startswith("surface 1: height -0.30")
    assert "under the origin of link_base" in answer
    assert "long side at 0 deg, level" in answer
    (surface,) = localizer.surfaces()
    assert surface["height"] == pytest.approx(0.0, abs=0.004)
    assert surface["size"] == pytest.approx([0.8, 0.5], abs=0.03)


def _surface(yaw_deg=0.0, center=(0.5, 0.0, 0.08), size=(0.8, 0.5), points=4000):
    return {
        "height": center[2],
        "center": list(center),
        "size": list(size),
        "yaw_deg": yaw_deg,
        "normal": [0.0, 0.0, 1.0],
        "tilt_deg": 0.0,
        "downhill_deg": 0.0,
        "coefficients": [0.0, 0.0, 1.0, -center[2]],
        "points": points,
        "rms_m": 0.002,
    }


_TABLE_SETTINGS = dict(reach=1.0, thickness=0.03, cut_above=0.01)


def test_table_box_tops_out_on_the_surface_round_the_whole_reach():
    base = np.zeros(3)
    (box_pose, box_size), (cut_pose, cut_size) = table_boxes(_surface(), [base], **_TABLE_SETTINGS)

    # Centred under the base, not on the measured table: its edges are never used.
    assert [box_pose.position.x, box_pose.position.y] == pytest.approx([0.0, 0.0])
    assert box_size == pytest.approx([2.0, 2.0, 0.03])
    assert box_pose.position.z + box_size[2] / 2 == pytest.approx(0.08)
    assert cut_size == pytest.approx([2.0, 2.0, 1.01])
    assert cut_pose.position.z + cut_size[2] / 2 == pytest.approx(0.09)
    assert cut_pose.position.z - cut_size[2] / 2 == pytest.approx(-0.92)
    assert pose_to_matrix(box_pose)[:3, 2] == pytest.approx([0.0, 0.0, 1.0])


def test_two_arms_share_one_table_round_both():
    bases = [np.array([0.0, -0.3, 0.0]), np.array([0.0, 0.3, 0.0])]
    (box_pose, box_size), _ = table_boxes(_surface(), bases, **_TABLE_SETTINGS)

    assert [box_pose.position.x, box_pose.position.y] == pytest.approx([0.0, 0.0])
    assert box_size[:2] == pytest.approx([2.0, 2.6])


def test_the_cut_takes_table_cells_and_what_is_under_and_keeps_what_stands_on_it():
    _, (cut_pose, cut_size) = table_boxes(_surface(), [np.zeros(3)], **_TABLE_SETTINGS)
    cells = np.array(
        [
            [0.5, 0.0, 0.085],  # a 1 cm cell of the table top, its centre half a cell up
            [0.3, 0.2, -0.4],  # under the table
            [0.5, 0.0, 0.105],  # a block's cell 2.5 cm over the table
            [1.2, 0.0, 0.085],  # past the arm's reach
        ]
    )

    inside = inside_boxes(cells, [(pose_to_matrix(cut_pose), np.asarray(cut_size))])

    assert inside.tolist() == [True, True, False, False]


def test_add_table_puts_a_box_in_the_planner_and_cuts_its_cells(localizer, mocker):
    _ready(localizer, mocker, [])
    mocker.patch.object(localizer, "surfaces", return_value=[_surface()])
    localizer._obstacles = mocker.Mock()
    localizer._obstacles.list_planning_groups.return_value = (
        SimpleNamespace(base_frame="link_base"),
    )
    localizer._obstacles.add_obstacle.return_value = TABLE
    localizer._box_filter = mocker.Mock()
    localizer.rig.tf.get.return_value = Transform(
        translation=Vector3(0.0, 0.0, 0.0), frame_id="world", child_frame_id="link_base"
    )

    answer = localizer.add_table()

    assert answer.startswith("table added: top at z=0.080 m in world, 2.00 x 2.00 m")
    name, pose, shape, size, mesh, touch = localizer._obstacles.add_obstacle.call_args.args
    assert (name, shape, mesh, touch) == (TABLE, "box", None, ["link_base"])
    assert size == pytest.approx([2.0, 2.0, 0.03])
    cut_name, _, cut_size = localizer._box_filter.set_box.call_args.args
    assert cut_name == TABLE
    assert cut_size.to_tuple() == pytest.approx((2.0, 2.0, 1.01))
    # A second call replaces the table rather than stacking another.
    localizer._obstacles.remove_obstacle.assert_called_once_with(TABLE)


def test_add_table_without_a_planner_says_so(localizer, mocker):
    _ready(localizer, mocker, [])

    assert "needs a motion planner" in localizer.add_table()


def test_add_table_with_no_surface_in_view_adds_nothing(localizer, mocker):
    _ready(localizer, mocker, [])
    mocker.patch.object(localizer, "surfaces", return_value=[])
    localizer._obstacles = mocker.Mock()
    localizer._box_filter = mocker.Mock()

    assert localizer.add_table().startswith("no table")
    localizer._obstacles.add_obstacle.assert_not_called()
    localizer._box_filter.set_box.assert_not_called()


def test_the_table_is_the_highest_large_surface_not_the_floor_or_an_object_top():
    # The xArm grasp sim: a small table over a floor that fills more of the view.
    floor = _surface(center=(0.55, 0.0, 0.0), points=26390)
    table = _surface(center=(0.52, 0.0, 0.13), points=15000)
    object_tops = _surface(center=(0.5, 0.0, 0.2), points=2110)

    assert pick_table([floor, table, object_tops]) is table
    assert pick_table([floor, table, object_tops], height=0.0) is floor
    assert pick_table([floor, table, object_tops], height=0.5) is None
    assert pick_table([]) is None
