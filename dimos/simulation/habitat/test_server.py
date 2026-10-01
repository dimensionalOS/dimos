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

"""The native encodes with generated CDR; dimos decodes. These pin that contract.

server.py runs under python 3.9 in nix/env, but its encoders are standalone generated CDR
and import fine here, so the wire format is testable without habitat installed.
"""

import ast
from pathlib import Path

from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos_generated.tf2_msgs.msg import TFMessage
import numpy as np
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.image import image_view
from dimos.msgs.pointcloud import pointcloud_xyz
from dimos.simulation.habitat import server

SERVER_PY = Path(server.__file__)


def test_color_image_round_trip():
    rgb = np.arange(8 * 12 * 3, dtype=np.uint8).reshape(8, 12, 3)
    out = Image.decode(server.image_msg(rgb, "rgb8", "camera_optical", 1.5))
    np.testing.assert_array_equal(image_view(out), rgb)
    assert out.header.frame_id == "camera_optical"


def test_depth_image_round_trip():
    depth = np.linspace(0.2, 4.5, 8 * 12, dtype=np.float32).reshape(8, 12)
    out = Image.decode(server.image_msg(depth, "32FC1", "camera_optical", 1.5))
    np.testing.assert_allclose(image_view(out), depth, atol=1e-6)


def test_camera_info_round_trip():
    k = {"fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 180.0, "width": 640.0, "height": 360.0}
    out = CameraInfo.decode(server.camera_info_msg(k, "camera_optical", 1.5))
    matrix = np.asarray(out.k).reshape(3, 3)
    assert (out.width, out.height) == (640, 360)
    np.testing.assert_allclose([matrix[0, 0], matrix[1, 1]], [320.0, 320.0])
    np.testing.assert_allclose([matrix[0, 2], matrix[1, 2]], [320.0, 180.0])


def test_cloud_round_trip_keeps_points_and_colour():
    pts = np.array([[1.0, 2.0, 3.0], [-4.0, 0.5, 0.25]], dtype=np.float32)
    colors = np.array([[255, 0, 0], [0, 128, 64]], dtype=np.uint8)
    out = PointCloud2.decode(server.cloud_msg(pts, colors, "world", 1.5))
    got = pointcloud_xyz(out)
    assert out.header.frame_id == "world"
    np.testing.assert_allclose(np.sort(got, axis=0), np.sort(pts, axis=0), atol=1e-5)


def test_empty_cloud_is_valid():
    out = PointCloud2.decode(
        server.cloud_msg(np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8), "world", 1.0)
    )
    assert out.width == 0 and len(out.data) == 0


def test_odometry_round_trip():
    pos = np.array([1.0, -2.0, 0.5])
    quat = np.array([0.0, 0.0, 0.3826834, 0.9238795])  # 45 deg yaw
    out = Odometry.decode(
        server.odometry_msg(pos, quat, (0.4, 0.0, 0.2), "world", "base_link", 1.5)
    )
    np.testing.assert_allclose(
        [out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z],
        pos,
        atol=1e-6,
    )
    assert out.header.frame_id == "world"
    assert out.child_frame_id == "base_link"


def test_tf_round_trip_carries_the_optical_link():
    links = [
        ("world", "base_link", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ("base_link", "camera", (0.0, 0.0, 0.45), (0.0, 0.0, 0.0, 1.0)),
        ("camera", "camera_optical", (0.0, 0.0, 0.0), server.frames.OPTICAL_QUAT_XYZW),
    ]
    out = TFMessage.decode(server.tf_msg(links, 1.5))
    pairs = {(t.header.frame_id, t.child_frame_id) for t in out.transforms}
    assert ("camera", "camera_optical") in pairs
    assert len(out.transforms) == 3


def test_each_tf_link_keeps_its_own_pose():
    """Regression: TF edges must retain independent nested transform values.

    Mutating ``Transform().translation`` in place made all three links serialize
    the last one's pose, which showed up as the whole map pointing up. Only a
    value check catches it -- the frame names were right the whole time.
    """
    links = [
        ("world", "base_link", (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)),
        ("base_link", "camera", (0.0, 0.0, 0.45), (0.0, 0.0, 0.0, 1.0)),
        ("camera", "camera_optical", (0.0, 0.0, 0.0), server.frames.OPTICAL_QUAT_XYZW),
    ]
    by_child = {t.child_frame_id: t for t in TFMessage.decode(server.tf_msg(links, 1.5)).transforms}

    base = by_child["base_link"]
    np.testing.assert_allclose(
        [base.transform.translation.x, base.transform.translation.y, base.transform.translation.z],
        [1.0, 2.0, 3.0],
    )
    np.testing.assert_allclose(
        [
            base.transform.rotation.x,
            base.transform.rotation.y,
            base.transform.rotation.z,
            base.transform.rotation.w,
        ],
        [0, 0, 0, 1],
    )

    cam = by_child["camera"]
    np.testing.assert_allclose(
        [cam.transform.translation.x, cam.transform.translation.y, cam.transform.translation.z],
        [0.0, 0.0, 0.45],
    )

    opt = by_child["camera_optical"]
    np.testing.assert_allclose(
        [opt.transform.translation.x, opt.transform.translation.y, opt.transform.translation.z],
        [0.0, 0.0, 0.0],
    )
    np.testing.assert_allclose(
        [
            opt.transform.rotation.x,
            opt.transform.rotation.y,
            opt.transform.rotation.z,
            opt.transform.rotation.w,
        ],
        server.frames.OPTICAL_QUAT_XYZW,
    )


def test_tf_chain_puts_the_camera_looking_forward():
    """Composing the published chain must aim optical +z along world +x, not +z."""
    yaw = 0.0
    links = [
        ("world", "base_link", (0.0, 0.0, 0.0), (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2))),
        ("base_link", "camera", (0.0, 0.0, 0.45), (0.0, 0.0, 0.0, 1.0)),
        ("camera", "camera_optical", (0.0, 0.0, 0.0), server.frames.OPTICAL_QUAT_XYZW),
    ]
    by_child = {t.child_frame_id: t for t in TFMessage.decode(server.tf_msg(links, 1.5)).transforms}

    def rot(t):
        x, y, z, w = (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        )
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )

    chain = rot(by_child["base_link"]) @ rot(by_child["camera"]) @ rot(by_child["camera_optical"])
    np.testing.assert_allclose(chain @ [0, 0, 1], [1, 0, 0], atol=1e-9)  # forward
    np.testing.assert_allclose(chain @ [0, 1, 0], [0, 0, -1], atol=1e-9)  # down


def test_unproject_recovers_a_known_point():
    """A pixel at a known depth must land where the pinhole says it does."""
    k = {"fx": 100.0, "fy": 100.0, "cx": 2.0, "cy": 1.0, "width": 4.0, "height": 2.0}
    depth = np.zeros((2, 4), dtype=np.float32)
    depth[1, 3] = 2.0  # one valid pixel, 2 m out
    rgb = np.full((2, 4, 3), 7, dtype=np.uint8)
    pts, colors = server.unproject(depth, rgb, k, trunc=5.0, stride=1)
    assert pts.shape == (1, 3)
    # optical frame: x right of centre, y below centre, z forward
    np.testing.assert_allclose(
        pts[0], [(3 - 2) * 2.0 / 100.0, (1 - 1) * 2.0 / 100.0, 2.0], atol=1e-6
    )
    np.testing.assert_array_equal(colors[0], [7, 7, 7])


def test_unproject_drops_invalid_depth():
    k = {"fx": 10.0, "fy": 10.0, "cx": 1.0, "cy": 1.0, "width": 2.0, "height": 2.0}
    depth = np.array([[0.0, np.nan], [9.0, 1.0]], dtype=np.float32)  # zero, nan, beyond trunc, ok
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    pts, _ = server.unproject(depth, rgb, k, trunc=5.0, stride=1)
    assert len(pts) == 1


def test_server_is_standalone():
    """server.py runs under python 3.9 in the conda env: no dimos imports allowed."""
    tree = ast.parse(SERVER_PY.read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            imported.add(node.module.split(".")[0])
    assert "dimos" not in imported, f"server.py must not import dimos, got {sorted(imported)}"


def test_server_is_py39_compatible():
    """No 3.10+ syntax: the habitat env has no newer interpreter."""
    src = SERVER_PY.read_text()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        assert not isinstance(node, ast.Match), "match statement is not valid on python 3.9"
    assert "from __future__ import annotations" in src


def test_every_frames_attribute_used_by_server_exists():
    """``frames`` is loaded via importlib, so mypy types it Any and typos survive.

    A missing helper then only surfaces as an AttributeError at runtime, inside the
    native, after the sim has already started.
    """
    import dimos.simulation.habitat.frames as frames_mod

    tree = ast.parse(SERVER_PY.read_text())
    used = {
        node.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "frames"
    }
    missing = sorted(name for name in used if not hasattr(frames_mod, name))
    assert not missing, f"server.py calls frames.{missing} which frames.py does not define"


def test_habitat_payloads_decode_with_independent_ros_types():
    types = get_typestore(Stores.ROS2_HUMBLE)
    packets = [
        (
            "sensor_msgs/msg/Image",
            server.image_msg(np.full((2, 3, 3), 7, dtype=np.uint8), "rgb8", "camera", 12.25),
        ),
        (
            "sensor_msgs/msg/CameraInfo",
            server.camera_info_msg(
                {"width": 3, "height": 2, "fx": 10, "fy": 11, "cx": 1, "cy": 1}, "camera", 12.25
            ),
        ),
        (
            "sensor_msgs/msg/PointCloud2",
            server.cloud_msg(
                np.array([[1, 2, 3]], dtype=np.float32),
                np.array([[255, 128, 64]], dtype=np.uint8),
                "world",
                12.25,
            ),
        ),
        (
            "nav_msgs/msg/Odometry",
            server.odometry_msg(
                np.array([1, 2, 3]),
                np.array([0, 0, 0, 1]),
                (0.4, 0.2, 0.3),
                "world",
                "base_link",
                12.25,
            ),
        ),
        (
            "tf2_msgs/msg/TFMessage",
            server.tf_msg([("world", "base_link", (1, 2, 3), (0, 0, 0, 1))], 12.25),
        ),
    ]
    decoded = {}
    for typename, packet in packets:
        message = types.deserialize_cdr(packet, typename)
        header = (
            message.transforms[0].header if typename == "tf2_msgs/msg/TFMessage" else message.header
        )
        assert (header.stamp.sec, header.stamp.nanosec) == (12, 250000000)
        decoded[typename] = message
    assert len(decoded["sensor_msgs/msg/Image"].data) == 18
    assert decoded["sensor_msgs/msg/CameraInfo"].k[0] == 10.0
    assert decoded["sensor_msgs/msg/PointCloud2"].row_step == 16
    assert decoded["nav_msgs/msg/Odometry"].pose.pose.position.z == 3.0
    assert decoded["tf2_msgs/msg/TFMessage"].transforms[0].transform.translation.x == 1.0
