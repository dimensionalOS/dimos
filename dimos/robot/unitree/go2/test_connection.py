# Copyright 2025-2026 Dimensional Inc.
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

"""Tests for go2.connection: make_connection routing and TF frame naming.

The leaf (UnitreeWebRTCConnection.__init__) is covered in
dimos/robot/unitree/test_connection.py; this pins the go2-local routing.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.go2 import connection as go2_conn
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_mid360_record import (
    unitree_go2_mid360_record,
)
from dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nav_3d import (
    unitree_go2_nav_3d,
)
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
from dimos.robot.unitree.go2.go2_mid360_static_transforms import mount_transforms


@pytest.fixture
def stub_webrtc(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace UnitreeWebRTCConnection in go2.connection so the webrtc branch
    runs without dialing out."""
    stub = MagicMock(name="UnitreeWebRTCConnection")
    monkeypatch.setattr(go2_conn, "UnitreeWebRTCConnection", stub)
    return stub


def test_make_connection_webrtc_forwards_aes_128_key(stub_webrtc: MagicMock) -> None:
    """Webrtc branch forwards aes_128_key as a kwarg to UnitreeWebRTCConnection."""
    cfg = SimpleNamespace(unitree_connection_type="webrtc")
    go2_conn.make_connection("192.168.123.161", cfg, aes_128_key="cafe" * 8)
    stub_webrtc.assert_called_once_with(
        "192.168.123.161",
        aes_128_key="cafe" * 8,
        velocity_api=False,
    )


def test_connection_config_aes_key_defaults_from_global_config() -> None:
    """ConnectionConfig.aes_128_key defaults from GlobalConfig.unitree_aes_128_key."""
    g = GlobalConfig(robot_ip="127.0.0.1", unitree_aes_128_key="dd" * 16)
    assert ConnectionConfig(g=g).aes_128_key == "dd" * 16


def test_odom_to_tf_unprefixed_by_default() -> None:
    odom = PoseStamped(ts=1.0, frame_id="world")
    base, camera_link, camera_optical = GO2Connection._odom_to_tf(odom)
    assert (base.frame_id, base.child_frame_id) == ("world", "base_link")
    assert (camera_link.frame_id, camera_link.child_frame_id) == ("base_link", "camera_link")
    assert (camera_optical.frame_id, camera_optical.child_frame_id) == (
        "camera_link",
        "camera_optical",
    )


def _connection(publish_tf: bool) -> GO2Connection:
    conn = object.__new__(GO2Connection)
    conn.config = ConnectionConfig(
        g=GlobalConfig(robot_ip="127.0.0.1"), publish_tf=publish_tf, odom_frame_id="go2_odom"
    )
    conn.tf = MagicMock()
    conn.odom = MagicMock()
    return conn


def test_publish_tf_off_keeps_odometry_on_its_port() -> None:
    """Turning tf off hands the base_link edge to another publisher, not the odom port."""
    conn = _connection(publish_tf=False)
    conn._publish_tf(PoseStamped(ts=1.0, frame_id="ignored"))
    assert conn.tf.publish.call_count == 0
    assert conn.odom.publish.call_count == 1


def test_publish_tf_on_by_default() -> None:
    conn = _connection(publish_tf=True)
    conn._publish_tf(PoseStamped(ts=1.0, frame_id="ignored"))
    assert conn.tf.publish.call_count == 1
    assert conn.odom.publish.call_count == 1


def _go2_connection_publishes_tf(blueprint: Blueprint) -> bool | None:
    for atom in blueprint.blueprints:
        if atom.module is GO2Connection:
            return bool(atom.kwargs.get("publish_tf", True))
    return None


def test_static_tree_and_connection_never_share_a_child_frame() -> None:
    """One publisher per edge: rerun keys tf entities by child, so a frame written
    by two sources flaps between them."""
    odom = PoseStamped(ts=1.0, frame_id="go2_odom")
    connection_children = {t.child_frame_id for t in GO2Connection._odom_to_tf(odom)}
    static_children = {t.child_frame_id for t in mount_transforms()}
    assert connection_children & static_children == {"base_link", "camera_optical"}

    for blueprint in (unitree_go2_nav_3d, unitree_go2_mid360_record):
        assert _go2_connection_publishes_tf(blueprint) is False


def test_odom_to_tf_prefixed() -> None:
    """.namespace() sets frame_id_prefix: robot-local frames get prefixed, the
    odom parent frame stays global so all robots hang off one tree root."""
    odom = PoseStamped(ts=1.0, frame_id="world")
    base, camera_link, camera_optical = GO2Connection._odom_to_tf(odom, prefix="robot0")
    assert (base.frame_id, base.child_frame_id) == ("world", "robot0/base_link")
    assert (camera_link.frame_id, camera_link.child_frame_id) == (
        "robot0/base_link",
        "robot0/camera_link",
    )
    assert (camera_optical.frame_id, camera_optical.child_frame_id) == (
        "robot0/camera_link",
        "robot0/camera_optical",
    )
