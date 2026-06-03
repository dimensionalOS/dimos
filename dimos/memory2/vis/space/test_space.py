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

"""Tests for Space builder and element types."""

import numpy as np
import pytest

from dimos.memory2.type.observation import EmbeddedObservation, Observation
from dimos.memory2.vis.space.elements import Arrow, Box3D, Camera, Point, Polyline, Pose, Text
from dimos.memory2.vis.space.space import Space
from dimos.msgs.geometry_msgs.Point import Point as GeoPoint
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path as Path
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection3D import Detection3D


class TestElementTypes:
    """Element types wrap msgs with rendering intent + style."""

    def test_pose_wraps_posestamped(self):
        ps = PoseStamped(3.2, 1.5, 0.0)
        p = Pose(ps, color="red", label="fridge")
        assert p.msg is ps
        assert p.color == "red"
        assert p.label == "fridge"

    def test_arrow_wraps_posestamped(self):
        ps = PoseStamped(1, 2, 0, 0, 0, 0.1, 1)
        a = Arrow(ps, color="orange", length=0.8)
        assert a.msg is ps
        assert a.length == 0.8

    def test_point_wraps_geopoint(self):
        gp = GeoPoint(7.1, 4.3, 0)
        p = Point(gp, color="green", label="bottle")
        assert p.msg is gp
        assert p.msg.x == pytest.approx(7.1)

    def test_point_wraps_posestamped(self):
        ps = PoseStamped(3, 1, 0)
        p = Point(ps, radius=0.5)
        assert p.msg.x == pytest.approx(3.0)

    def test_box3d_from_center_size(self):
        from dimos.msgs.geometry_msgs.Pose import Pose as GeoPose
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        b = Box3D(center=GeoPose(5, 3, 0), size=Vector3(2, 1, 0.5), label="table")
        assert b.center.x == pytest.approx(5.0)
        assert b.size.x == pytest.approx(2.0)
        assert b.label == "table"

    def test_camera_with_image(self):
        ps = PoseStamped(1, 2, 0)
        img = Image(np.zeros((480, 640, 3), dtype=np.uint8))
        c = Camera(pose=ps, image=img, color="purple")
        assert c.pose is ps
        assert c.image is img
        assert c.camera_info is None

    def test_text(self):
        t = Text((1, 8, 0), "exploration run #3")
        assert t.text == "exploration run #3"
        assert t.color == "#333333"


class TestSpaceExplicitElements:
    """Space.add() with explicit element types stores them as-is."""

    def test_add_pose(self):
        s = Space()
        ps = PoseStamped(3, 1, 0)
        pose = Pose(ps, color="red")
        s.add(pose)
        assert len(s) == 1
        assert s.elements[0] is pose

    def test_add_multiple_types(self):
        s = Space()
        ps = PoseStamped(3, 1, 0)
        s.add(Pose(ps, color="red"))
        s.add(Arrow(ps, color="orange"))
        s.add(Point(GeoPoint(1, 2, 0), label="x"))
        s.add(Text((0, 0, 0), "hello"))
        assert len(s) == 4

    def test_chaining(self):
        ps = PoseStamped(1, 1, 0)
        s = Space().add(Pose(ps)).add(Arrow(ps)).add(Text((0, 0, 0), "hi"))
        assert len(s) == 3


class TestSpaceAutoWrap:
    """Space.add() with raw dimos msgs auto-wraps into default element."""

    def test_posestamped_becomes_pose(self):
        s = Space()
        ps = PoseStamped(3.2, 1.5, 0)
        s.add(ps, color="blue", label="auto")
        assert len(s) == 1
        el = s.elements[0]
        assert isinstance(el, Pose)
        assert el.msg is ps
        assert el.color == "blue"
        assert el.label == "auto"

    def test_geopoint_becomes_point(self):
        s = Space()
        gp = GeoPoint(7, 4, 0)
        s.add(gp, color="yellow")
        el = s.elements[0]
        assert isinstance(el, Point)
        assert el.msg is gp
        assert el.color == "yellow"

    def test_path_becomes_polyline(self):
        s = Space()
        p = Path(poses=[PoseStamped(i, 0, 0) for i in range(3)])
        s.add(p, color="blue", width=0.1)
        el = s.elements[0]
        assert isinstance(el, Polyline)
        assert el.color == "blue"
        assert el.width == 0.1
        assert len(el.msg.poses) == 3

    def test_occupancy_grid_passthrough(self):
        s = Space()
        grid = OccupancyGrid()
        s.add(grid)
        assert s.elements[0] is grid

    def test_detection3d_becomes_box3d(self):
        det = Detection3D()
        det.bbox.center.position.x = 5.0
        det.bbox.center.position.y = 3.0
        det.bbox.size.x = 2.0
        det.bbox.size.y = 1.0
        det.bbox.size.z = 0.5

        s = Space()
        s.add(det, color="yellow")
        el = s.elements[0]
        assert isinstance(el, Box3D)
        assert el.center.position.x == pytest.approx(5.0)
        assert el.size.x == pytest.approx(2.0)
        assert el.color == "yellow"

    def test_unknown_type_raises(self):
        s = Space()
        with pytest.raises(TypeError, match="does not know how to handle"):
            s.add(42)


class TestSpaceObservations:
    """Space.add() smart dispatch for Observation types."""

    def test_image_observation_stored_as_observation(self):
        img = Image(np.zeros((480, 640, 3), dtype=np.uint8))
        obs = Observation(id=1, ts=1.0, pose=(3, 1, 0, 0, 0, 0, 1), _data=img)

        s = Space()
        s.add(obs)
        el = s.elements[0]
        assert isinstance(el, Observation)
        assert el.data is img

    def test_non_image_observation_stored_as_observation(self):
        obs = Observation(id=2, ts=2.0, pose=(5, 2, 0, 0, 0, 0, 1), _data="some_data")

        s = Space()
        s.add(obs)
        el = s.elements[0]
        assert isinstance(el, Observation)
        assert el.data == "some_data"

    def test_posestamped_observation_stored_as_observation(self):
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as PS

        obs = Observation(id=3, ts=3.0, pose=(1, 2, 0, 0, 0, 0, 1), _data=PS(5, 2, 0))

        s = Space()
        s.add(obs)
        el = s.elements[0]
        assert isinstance(el, Observation)
        assert el.data.x == pytest.approx(5.0)

    def test_embedded_observation_stored_as_arrow(self):
        obs = EmbeddedObservation(
            id=0,
            ts=0.0,
            pose=(1, 2, 0, 0, 0, 0, 1),
            _data="x",
            similarity=0.8,
        )

        s = Space()
        s.add(obs)
        assert len(s) == 1
        el = s.elements[0]
        assert isinstance(el, EmbeddedObservation)


class TestSpaceConvenience:
    """Space convenience methods: base_map."""

    def test_base_map(self):
        grid = OccupancyGrid()
        s = Space().base_map(grid)
        assert len(s) == 1
        assert isinstance(s.elements[0], OccupancyGrid)

    def test_add_list_of_msgs(self):
        poses = [PoseStamped(i, 0, 0) for i in range(3)]
        s = Space()
        s.add(poses, color="red")
        assert len(s) == 3
        for el in s.elements:
            assert isinstance(el, Pose)
            assert el.color == "red"


class TestSpaceRepr:
    def test_repr_empty(self):
        assert repr(Space()) == "Space()"

    def test_repr_with_elements(self):
        s = Space()
        ps = PoseStamped(0, 0, 0)
        s.add(Pose(ps))
        s.add(Pose(ps))
        s.add(Arrow(ps))
        assert repr(s) == "Space(Arrow=1, Pose=2)"


class TestSVGRender:
    """SVG rendering produces valid SVG with expected elements."""

    def test_empty_space(self):
        svg = Space().to_svg()
        assert svg.startswith("<svg")
        assert svg.endswith("</svg>")

    def test_point_renders_circle(self):
        from dimos.memory2.vis import color

        s = Space()
        s.add(Point(GeoPoint(3, 4, 0), color="red", label="hi"))
        svg = s.to_svg()
        assert "<circle" in svg
        assert color.red.hex() in svg
        assert "hi" in svg

    def test_pose_renders_polygon(self):
        from dimos.memory2.vis import color

        s = Space()
        s.add(Pose(PoseStamped(1, 2, 0), color="blue"))
        svg = s.to_svg()
        assert "<polygon" in svg
        assert color.blue.hex() in svg

    def test_arrow_renders_polygon(self):
        from dimos.memory2.vis import color

        s = Space()
        s.add(Arrow(PoseStamped(0, 0, 0, 0, 0, 0.38, 0.92), color="orange"))
        svg = s.to_svg()
        assert "<polygon" in svg
        assert color.orange.hex() in svg

    def test_polyline_renders(self):
        s = Space()
        s.add(
            Polyline(
                msg=Path(poses=[PoseStamped(i, i * 0.5, 0) for i in range(5)]),
                color="blue",
            )
        )
        svg = s.to_svg()
        assert "<polyline" in svg

    def test_box3d_renders_rect(self):
        from dimos.msgs.geometry_msgs.Pose import Pose as GeoPose
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        s = Space()
        s.add(Box3D(center=GeoPose(5, 3, 0), size=Vector3(2, 1, 0), label="table"))
        svg = s.to_svg()
        assert "<rect" in svg
        assert "table" in svg

    def test_text_renders(self):
        s = Space()
        s.add(Text((1, 1, 0), "hello <world>"))
        svg = s.to_svg()
        assert "<text" in svg
        assert "hello &lt;world&gt;" in svg

    def test_camera_without_info_renders_dot(self):
        from dimos.memory2.vis import color

        s = Space()
        s.add(Camera(pose=PoseStamped(1, 2, 0), color="purple"))
        svg = s.to_svg()
        assert "<circle" in svg
        assert color.purple.hex() in svg

    def test_occupancy_grid_renders_image(self):
        grid = OccupancyGrid(
            grid=np.zeros((10, 10), dtype=np.int8),
            resolution=0.1,
        )
        s = Space().base_map(grid)
        svg = s.to_svg()
        assert "<image" in svg
        assert "data:image/png;base64," in svg

    def test_mixed_space(self):
        s = Space()
        ps = PoseStamped(3, 1, 0)
        s.add(Pose(ps, color="red", label="robot"))
        s.add(Arrow(ps, color="orange"))
        s.add(Point(GeoPoint(5, 5, 0), color="green", label="goal"))
        s.add(Text((0, 0, 0), "test"))
        svg = s.to_svg()
        assert svg.count("<circle") == 1  # point dot
        assert svg.count("<polygon") == 2  # pose + arrow
        assert "<text" in svg

    def test_to_svg_writes_file(self, tmp_path):
        s = Space()
        s.add(Point(GeoPoint(1, 1, 0)))
        out = tmp_path / "test.svg"
        s.to_svg(str(out))
        assert out.exists()
        assert "<svg" in out.read_text()
