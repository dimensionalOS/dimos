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
    SURFACE,
    ManipulationLocalize,
    pick_surface,
    surface_boxes,
)
from dimos.mapping.pointclouds.box_filter import inside_boxes
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.rig import Rig
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
    module.rig.lidar = None
    module.rig.camera_pose.return_value = None


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


def test_surfaces_on_lidar_read_only_the_window_scans(localizer, mocker):
    _ready(localizer, mocker, [None] * 3)
    lidar = mocker.Mock()
    localizer.rig = Rig(cameras={"camera": mocker.Mock()}, tf=mocker.Mock(), lidar=lidar)
    window_points = mocker.patch(
        "dimos.manipulation.manipulation_localize.window_points", return_value=None
    )

    localizer.surfaces(start=-4.0, duration=3.0)

    lidar.time_range.assert_called_once_with(6.0, 9.0)
    rig = window_points.call_args.args[0]
    assert rig.lidar is lidar.time_range.return_value


def _surface(yaw_deg=0.0, center=(0.5, 0.0, 0.08), size=(0.8, 0.5), points=4000, above=0):
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
        "points_above": above,
    }


_SURFACE_SETTINGS = dict(margin=0.05, thickness=0.03, cut_above=0.01, cut_below=0.02)


def test_table_box_is_the_measured_outline_a_margin_wider():
    (box_pose, box_size), (cut_pose, cut_size) = surface_boxes(_surface(), **_SURFACE_SETTINGS)

    # On the measured table, not round the arm's base.
    assert [box_pose.position.x, box_pose.position.y] == pytest.approx([0.5, 0.0])
    assert box_size == pytest.approx([0.9, 0.6, 0.03])
    assert box_pose.position.z + box_size[2] / 2 == pytest.approx(0.08)
    assert cut_size == pytest.approx([0.9, 0.6, 0.03])
    assert cut_pose.position.z + cut_size[2] / 2 == pytest.approx(0.09)
    assert cut_pose.position.z - cut_size[2] / 2 == pytest.approx(0.06)
    assert pose_to_matrix(box_pose)[:3, 2] == pytest.approx([0.0, 0.0, 1.0])


def test_table_box_turns_with_the_table():
    (box_pose, box_size), _ = surface_boxes(_surface(yaw_deg=30.0), **_SURFACE_SETTINGS)

    long_side = pose_to_matrix(box_pose)[:3, 0]
    assert long_side == pytest.approx([np.cos(np.radians(30)), np.sin(np.radians(30)), 0.0])
    assert box_size[:2] == pytest.approx([0.9, 0.6])


def test_tilted_table_box_lies_in_its_plane():
    normal = np.array([0.0, np.sin(np.radians(10)), np.cos(np.radians(10))])
    surface = {**_surface(), "normal": list(normal)}

    (box_pose, _), _ = surface_boxes(surface, **_SURFACE_SETTINGS)

    rotation = pose_to_matrix(box_pose)[:3, :3]
    assert rotation[:, 2] == pytest.approx(normal)
    assert float(rotation[:, 0] @ normal) == pytest.approx(0.0, abs=1e-9)


def test_the_cut_takes_the_surface_cells_and_keeps_what_is_over_and_under_it():
    _, (cut_pose, cut_size) = surface_boxes(_surface(), **_SURFACE_SETTINGS)
    cells = np.array(
        [
            [0.5, 0.0, 0.085],  # a 1 cm cell of the table top, its centre half a cell up
            [0.5, 0.0, 0.065],  # the table top's cell, its centre half a cell down
            [0.3, 0.2, -0.4],  # under the table: a lower step, mapped behind the box
            [0.5, 0.0, 0.105],  # a block's cell 2.5 cm over the table
            [1.2, 0.0, 0.085],  # past the table's edge
        ]
    )

    inside = inside_boxes(cells, [(pose_to_matrix(cut_pose), np.asarray(cut_size))])

    assert inside.tolist() == [True, True, False, False, False]


def test_add_surface_puts_a_box_in_the_planner_and_cuts_its_cells(localizer, mocker):
    _ready(localizer, mocker, [])
    mocker.patch.object(localizer, "surfaces", return_value=[_surface()])
    localizer._obstacles = mocker.Mock()
    localizer._obstacles.list_planning_groups.return_value = (
        SimpleNamespace(base_frame="link_base"),
    )
    localizer._obstacles.add_obstacle.return_value = SURFACE
    localizer._box_filter = mocker.Mock()

    answer = localizer.add_surface()

    assert answer.startswith("surface added: top at z=0.080 m in world, 0.90 x 0.60 m")
    name, pose, shape, size, mesh, touch = localizer._obstacles.add_obstacle.call_args.args
    assert (name, shape, mesh, touch) == (f"{SURFACE}/1", "box", None, ["link_base"])
    assert size == pytest.approx([0.9, 0.6, 0.03])
    cut_name, _, cut_size = localizer._box_filter.set_box.call_args.args
    assert cut_name == f"{SURFACE}/1"
    assert cut_size.to_tuple() == pytest.approx((0.9, 0.6, 0.03))
    # A second call at the same height replaces the table rather than stacking another.
    localizer.add_surface()
    localizer._obstacles.remove_obstacle.assert_called_once_with(f"{SURFACE}/1")
    assert localizer._surfaces == {f"{SURFACE}/2": pytest.approx(0.08)}


def test_two_surfaces_at_different_heights_stay_in_the_planner(localizer, mocker):
    _ready(localizer, mocker, [])
    low = _surface(center=(0.2, -0.5, 0.04), size=(0.3, 0.3), above=50)
    mocker.patch.object(localizer, "surfaces", return_value=[_surface(above=50), low])
    localizer._obstacles = mocker.Mock()
    localizer._obstacles.list_planning_groups.return_value = (SimpleNamespace(base_frame="world"),)
    localizer._obstacles.add_obstacle.return_value = SURFACE
    localizer._box_filter = mocker.Mock()

    localizer.add_surface(height=0.08)
    answer = localizer.add_surface(height=0.04)

    assert "In the planner: 0.040 m, 0.080 m" in answer
    localizer._obstacles.remove_obstacle.assert_not_called()
    assert localizer.clear_surface(0.04) == "1 surface(s) removed"
    localizer._obstacles.remove_obstacle.assert_called_once_with(f"{SURFACE}/2")
    assert localizer.clear_surface() == "1 surface(s) removed"
    assert localizer.clear_surface() == "no surface to remove"


def test_add_surface_without_a_planner_says_so(localizer, mocker):
    _ready(localizer, mocker, [])

    assert "needs a motion planner" in localizer.add_surface()


def test_add_surface_with_no_surface_in_view_adds_nothing(localizer, mocker):
    _ready(localizer, mocker, [])
    mocker.patch.object(localizer, "surfaces", return_value=[])
    localizer._obstacles = mocker.Mock()
    localizer._box_filter = mocker.Mock()

    assert localizer.add_surface().startswith("no surface")
    localizer._obstacles.add_obstacle.assert_not_called()
    localizer._box_filter.set_box.assert_not_called()


def test_the_table_is_the_widest_surface_with_objects_on_it():
    # An arm at its table: the floor past the edge is wider in view but empty.
    floor = _surface(center=(1.2, 0.0, -0.6), size=(1.2, 0.8), points=26000)
    table = _surface(center=(0.5, 0.0, 0.12), size=(0.9, 0.6), points=15000, above=300)
    tray = _surface(center=(0.45, 0.0, 0.16), size=(0.3, 0.2), points=6000, above=80)

    assert pick_surface([floor, table, tray]) is table
    assert pick_surface([floor, table, tray], height=-0.6) is floor
    assert pick_surface([floor, table, tray], height=0.5) is None
    assert pick_surface([]) is None


def test_with_nothing_standing_anywhere_the_widest_surface_is_the_table():
    floor = _surface(center=(1.2, 0.0, -0.6), size=(1.2, 0.8))
    table = _surface(center=(0.5, 0.0, 0.12), size=(0.9, 0.6))

    assert pick_surface([table, floor]) is floor


def test_the_table_is_what_the_camera_looks_at():
    """On the real rig the floor past the table's edge can be wider in view, with legs on it."""
    floor = _surface(center=(1.2, 0.0, -0.6), size=(1.4, 0.9), points=26000, above=400)
    table = _surface(center=(0.5, 0.0, 0.12), size=(0.6, 0.4), points=9000, above=300)
    down = np.array([0.0, 0.0, -1.0])

    assert pick_surface([floor, table], sight=(np.array([0.5, 0.05, 0.45]), down)) is table
    # Looking past the table's edge, the floor is what the camera sees.
    assert pick_surface([floor, table], sight=(np.array([1.3, 0.0, 0.45]), down)) is floor
    # Looking up at nothing: the area rule.
    assert pick_surface([floor, table], sight=(np.zeros(3), -down)) is floor


def test_at_a_height_the_biggest_surface_there_is_taken():
    table = _surface(center=(0.4, -0.3, 0.046), size=(0.31, 0.26))
    scrap = _surface(center=(0.7, -0.4, 0.053), size=(0.22, 0.11))

    assert pick_surface([scrap, table], height=0.05) is table


def test_add_surface_takes_the_newest_view(localizer, mocker):
    _ready(localizer, mocker, [])
    floor = _surface(center=(1.2, 0.0, -0.6), size=(1.4, 0.9), points=26000, above=400)
    mocker.patch.object(localizer, "surfaces", return_value=[floor, _surface(above=50)])
    localizer._obstacles = mocker.Mock()
    localizer._obstacles.list_planning_groups.return_value = (SimpleNamespace(base_frame="world"),)
    localizer._obstacles.add_obstacle.return_value = SURFACE
    localizer._box_filter = mocker.Mock()
    # The optical frame 40 cm over the table, looking straight down (optical z down).
    localizer.rig.camera_pose.return_value = PoseStamped(
        frame_id="world",
        position=Vector3(0.5, 0.0, 0.5),
        orientation=Quaternion(1.0, 0.0, 0.0, 0.0),
    )

    assert localizer.add_surface().startswith("surface added: top at z=0.080 m")
    localizer.rig.camera_pose.assert_called_once_with(10.0, "camera_color_optical_frame")


def test_add_surface_lists_the_surfaces_it_chose_from(localizer, mocker):
    _ready(localizer, mocker, [])
    floor = _surface(center=(0.5, 0.0, 0.0), size=(1.0, 0.6), points=26000)
    mocker.patch.object(localizer, "surfaces", return_value=[floor, _surface(above=50)])
    localizer._obstacles = mocker.Mock()
    localizer._obstacles.list_planning_groups.return_value = (SimpleNamespace(base_frame="world"),)
    localizer._obstacles.add_obstacle.return_value = SURFACE
    localizer._box_filter = mocker.Mock()

    answer = localizer.add_surface()

    assert answer.startswith("surface added: top at z=0.080 m")
    assert "Surfaces seen: 0.000 m (1.00 x 0.60), 0.080 m (0.80 x 0.50)" in answer
