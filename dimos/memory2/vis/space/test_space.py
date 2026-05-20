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

import re

import numpy as np
import pytest


def _viewbox(svg: str) -> tuple[float, float, float, float]:
    """Parse the SVG viewBox attribute → (xmin, ymin, width, height)."""
    m = re.search(r'viewBox="([\-\d\.]+) ([\-\d\.]+) ([\-\d\.]+) ([\-\d\.]+)"', svg)
    assert m is not None, "no viewBox in SVG"
    return float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))


from dimos.memory2.type.observation import EmbeddedObservation, Observation
from dimos.memory2.vis.color import ColorRange
from dimos.memory2.vis.space.elements import (
    Arrow,
    Box3D,
    Camera,
    Point,
    Polygon,
    Polyline,
    Pose,
    RasterOverlay,
    Text,
    Wedge,
)
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


class TestPolygonElement:
    """Polygon: closed shape with optional fill + stroke + label."""

    def test_polygon_renders_svg_polygon(self):
        from dimos.memory2.vis import color

        s = Space()
        s.add(
            Polygon(
                vertices=[(0, 0), (4, 0), (4, 3), (0, 3)],
                fill="red",
                stroke="blue",
                label="kitchen",
            )
        )
        svg = s.to_svg()
        assert "<polygon" in svg
        assert color.red.hex() in svg
        assert color.blue.hex() in svg
        assert "fill-opacity" in svg
        assert "kitchen" in svg

    def test_polygon_no_fill(self):
        s = Space()
        s.add(Polygon(vertices=[(0, 0), (1, 0), (1, 1)], stroke="blue"))
        svg = s.to_svg()
        assert 'fill="none"' in svg

    def test_polygon_lt_3_vertices_skipped(self):
        s = Space()
        s.add(Polygon(vertices=[(0, 0), (1, 0)], fill="red"))
        svg = s.to_svg()
        # No <polygon shape emitted; comment-only fallback OK.
        assert "<polygon " not in svg and "<polygon\n" not in svg

    def test_polygon_grows_bounds(self):
        s = Space()
        s.add(Polygon(vertices=[(100, 200), (105, 200), (105, 205)], stroke="red"))
        svg = s.to_svg()
        xmin, ymin, w, h = _viewbox(svg)
        # Polygon spans world x ∈ [100, 105], y ∈ [200, 205]. SVG Y is flipped,
        # so the viewBox covers x ∈ [100, 105] and y ∈ [-205, -200] (± padding).
        assert xmin <= 100.0 and xmin + w >= 105.0
        assert ymin <= -205.0 and ymin + h >= -200.0

    def test_polygon_uses_deferred_color(self):
        s = Space()
        cr = ColorRange("turbo")
        s.add(Polygon(vertices=[(0, 0), (1, 0), (1, 1)], fill=cr(0.1), stroke=cr(0.9)))
        svg = s.to_svg()
        # After to_svg, deferred colors should have resolved to concrete hex.
        assert 'fill="#' in svg
        assert 'stroke="#' in svg


class TestWedgeElement:
    """Wedge: outlined viewing cone."""

    def test_wedge_renders_svg_polygon(self):
        import math as _math

        from dimos.memory2.vis import color

        s = Space()
        s.add(Wedge(origin=(1, 2), yaw=0.0, fov=_math.radians(60), length=3, color="orange"))
        svg = s.to_svg()
        assert "<polygon" in svg
        assert color.orange.hex() in svg

    def test_wedge_label_renders(self):
        import math as _math

        s = Space()
        s.add(Wedge(origin=(0, 0), yaw=0, fov=_math.radians(45), length=2, label="cam0"))
        svg = s.to_svg()
        assert "cam0" in svg

    def test_wedge_zero_fov_skipped(self):
        s = Space()
        s.add(Wedge(origin=(0, 0), yaw=0, fov=0, length=2))
        svg = s.to_svg()
        assert "<polygon" not in svg


class TestRasterOverlayElement:
    """RasterOverlay: world-frame RGBA bitmap."""

    def test_raster_overlay_renders_image_tag(self):
        rgba = np.full((4, 4, 4), 255, dtype=np.uint8)
        s = Space()
        s.add(RasterOverlay(rgba=rgba, origin=(0, 0), resolution=0.1))
        svg = s.to_svg()
        assert "<image" in svg
        assert "data:image/png;base64," in svg

    def test_raster_overlay_grows_bounds(self):
        rgba = np.full((4, 4, 4), 255, dtype=np.uint8)
        s = Space()
        s.add(RasterOverlay(rgba=rgba, origin=(10, 20), resolution=0.1))
        svg = s.to_svg()
        xmin, ymin, w, h = _viewbox(svg)
        # 4×4 grid at 0.1m resolution → world extent x ∈ [10, 10.4], y ∈ [20, 20.4].
        # SVG Y flipped → y ∈ [-20.4, -20] (± padding).
        assert xmin <= 10.0 and xmin + w >= 10.4
        assert ymin <= -20.4 and ymin + h >= -20.0


class TestPointShapes:
    """Point.shape and Point.halo extensions."""

    def test_point_dot_renders_circle(self):
        s = Space()
        s.add(Point(GeoPoint(1, 1, 0), shape="dot"))
        svg = s.to_svg()
        assert "<circle" in svg

    def test_point_cross_renders_lines(self):
        s = Space()
        s.add(Point(GeoPoint(1, 1, 0), shape="cross"))
        svg = s.to_svg()
        assert svg.count("<line") >= 2

    def test_point_x_renders_lines(self):
        s = Space()
        s.add(Point(GeoPoint(1, 1, 0), shape="x"))
        svg = s.to_svg()
        assert svg.count("<line") >= 2

    def test_point_square_renders_rect(self):
        s = Space()
        s.add(Point(GeoPoint(1, 1, 0), shape="square"))
        svg = s.to_svg()
        assert "<rect" in svg

    def test_point_halo_includes_black(self):
        s = Space()
        s.add(Point(GeoPoint(1, 1, 0), shape="dot", halo=True))
        svg = s.to_svg()
        assert "#000000" in svg


class TestSpaceAddDispatch:
    """Space.add() accepts new element types."""

    def test_add_polygon(self):
        p = Polygon(vertices=[(0, 0), (1, 0), (1, 1)], fill="red")
        s = Space().add(p)
        assert s.elements[0] is p

    def test_add_raster_overlay(self):
        rgba = np.zeros((2, 2, 4), dtype=np.uint8)
        ro = RasterOverlay(rgba=rgba, origin=(0, 0), resolution=0.1)
        s = Space().add(ro)
        assert s.elements[0] is ro

    def test_add_wedge(self):
        import math as _math

        w = Wedge(origin=(0, 0), yaw=0, fov=_math.radians(60), length=2)
        s = Space().add(w)
        assert s.elements[0] is w


class TestRasterRender:
    """cv2 raster backend output shape, dtype, and pixel correctness."""

    def test_raster_empty_space(self):
        bgr = Space().to_bgr(width_px=200)
        assert bgr.dtype == np.uint8
        assert bgr.ndim == 3 and bgr.shape[2] == 3
        assert bgr.shape[1] == 200

    def test_raster_width_px(self):
        s = Space()
        s.add(Point(GeoPoint(0, 0, 0)))
        bgr = s.to_bgr(width_px=400)
        assert bgr.shape[1] == 400

    def test_raster_polygon_fill_pixel(self):
        # Polygon covers most of the canvas with opaque red.
        s = Space()
        s.add(
            Polygon(
                vertices=[(-1, -1), (1, -1), (1, 1), (-1, 1)],
                fill="red",
                fill_opacity=1.0,
            )
        )
        bgr = s.to_bgr(width_px=200, padding_m=0.1)
        cy, cx = bgr.shape[0] // 2, bgr.shape[1] // 2
        b, g, r = bgr[cy, cx]
        # Allow large tolerance — the red palette color is ~#e74c3c, mostly red.
        assert int(r) > int(b) and int(r) > int(g)

    def test_raster_pose_marker_is_colored(self):
        s = Space()
        s.add(Pose(PoseStamped(0, 0, 0), color="red"))
        bgr = s.to_bgr(width_px=200)
        # Default background is (248, 248, 248); the Pose marker is not.
        assert (bgr != 248).any()

    def test_raster_to_png_round_trip(self):
        import cv2

        s = Space()
        s.add(Point(GeoPoint(0, 0, 0), color="red"))
        png = s.to_png(width_px=200)
        decoded = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
        bgr_direct = s.to_bgr(width_px=200)
        assert decoded.shape == bgr_direct.shape

    def test_raster_occupancy_grid_used_as_base(self):
        grid_data = np.zeros((10, 10), dtype=np.int8)
        grid_data[3:7, 3:7] = 100  # occupied block
        grid = OccupancyGrid(grid=grid_data, resolution=0.1)
        s = Space().base_map(grid)
        bgr = s.to_bgr(width_px=200)
        # Not the default background everywhere.
        assert (bgr != 248).any()

    def test_raster_raster_overlay_blends(self):
        rgba = np.zeros((10, 10, 4), dtype=np.uint8)
        rgba[..., 0] = 220  # R
        rgba[..., 3] = 255  # opaque alpha
        s = Space()
        s.add(RasterOverlay(rgba=rgba, origin=(0, 0), resolution=0.1))
        bgr = s.to_bgr(width_px=200, padding_m=0.0)
        cy, cx = bgr.shape[0] // 2, bgr.shape[1] // 2
        b, g, r = bgr[cy, cx]
        assert int(r) > 100 and int(r) > int(b) and int(r) > int(g)

    def test_raster_wedge_draws_lines(self):
        import math as _math

        s = Space()
        s.add(Wedge(origin=(0, 0), yaw=0, fov=_math.radians(90), length=3, color="orange"))
        bgr = s.to_bgr(width_px=200)
        assert (bgr != 248).any()

    def test_raster_height_clamp(self):
        # A tall sliver shouldn't blow up the canvas; height clamped to width * 10.
        s = Space()
        s.add(Polygon(vertices=[(0, 0), (0.01, 0), (0.01, 1000)], stroke="red"))
        bgr = s.to_bgr(width_px=100)
        assert bgr.shape[0] <= 100 * 10


class TestSpaceToPNG:
    """PNG serialisation."""

    def test_to_png_returns_bytes_starting_with_png_magic(self):
        s = Space()
        s.add(Point(GeoPoint(0, 0, 0)))
        png = s.to_png(width_px=200)
        assert isinstance(png, (bytes, bytearray))
        assert bytes(png[:8]) == b"\x89PNG\r\n\x1a\n"

    def test_to_png_writes_path(self, tmp_path):
        s = Space()
        s.add(Point(GeoPoint(0, 0, 0)))
        out = tmp_path / "x.png"
        s.to_png(width_px=200, path=str(out))
        assert out.exists()
        assert out.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


class TestResolveDeferredFillStroke:
    """resolve_deferred walks color, fill, and stroke."""

    def test_polygon_deferred_fill_resolves(self):
        from dimos.memory2.vis.color import DeferredColor, resolve_deferred

        cr = ColorRange("turbo")
        p = Polygon(vertices=[(0, 0), (1, 0), (1, 1)], fill=cr(0.0), stroke=cr(1.0))
        assert isinstance(p.fill, DeferredColor)
        assert isinstance(p.stroke, DeferredColor)
        resolve_deferred([p])
        # After resolution both should be concrete Color objects.
        assert not isinstance(p.fill, DeferredColor)
        assert not isinstance(p.stroke, DeferredColor)
