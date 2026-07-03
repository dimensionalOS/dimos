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

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.go2 import connection as go2_conn
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection


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


def test_odom_to_tf_prefixed() -> None:
    """namespace() sets frame_id_prefix: robot-local frames get prefixed, the
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
