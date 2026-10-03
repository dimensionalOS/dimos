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


import io
import json
import time
from unittest.mock import MagicMock

import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.raw_manipulation_bridge import RawManipulationBridge, depth_f32


@pytest.fixture
def bridge():
    module = RawManipulationBridge()
    module._topics = MagicMock()
    module.ee_twist_command = MagicMock()
    module.gripper_command = MagicMock()
    yield module
    module.stop()


def _command(module, **payload):
    module._on_command(json.dumps(payload).encode(), None)


def _drive(module, ticks):
    """Run the deadman loop for a fixed number of ticks."""
    stop = module._stop
    module._stop = MagicMock()
    module._stop.wait.side_effect = [False] * ticks + [True]
    module._drive()
    module._stop = stop


def _twists(module):
    return [
        (*call.args[0].linear.to_numpy(), *call.args[0].angular.to_numpy())
        for call in module.ee_twist_command.publish.call_args_list
    ]


def _published(module, key):
    return [
        json.loads(call.args[1])
        for call in module._topics.put.call_args_list
        if call.args[0] == key
    ]


@pytest.mark.parametrize(
    "payload",
    [
        b"not json",
        b'{"kind":"delta","xyz":[0,0,0.1]}',
        b'{"kind":"joints","positions":[0,0]}',
        b'{"kind":"stop"}',
        b'{"kind":"twist","linear":[0,0],"t":1}',
        b'{"kind":"twist","linear":[0,0,1],"t":-1}',
        b'{"kind":"twist","linear":[0,0,1]}',
        b'{"kind":"twist","linear":[0,0,1],"t":1,"id":"x"}',
        b'{"kind":"gripper","opening":1.5}',
        b'{"kind":"gripper","position":0.4}',
    ],
)
def test_invalid_inputs_are_dropped(bridge, payload):
    bridge._on_command(payload, None)
    _drive(bridge, 3)
    bridge.ee_twist_command.publish.assert_not_called()
    bridge.gripper_command.publish.assert_not_called()


def test_twist_is_held_clamped_then_one_zero(bridge, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    _command(bridge, kind="twist", linear=[0.05, 0, 9], angular=[0, 0, -9], t=1.0)
    _drive(bridge, 2)
    assert _twists(bridge) == [(0.05, 0, 0.1, 0, 0, -0.5)] * 2
    now[0] += 1.0
    _drive(bridge, 3)
    assert _twists(bridge)[2:] == [(0, 0, 0, 0, 0, 0)]


def test_hold_time_is_capped_and_latest_twist_wins(bridge, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    _command(bridge, kind="twist", linear=[0, 0, 0.01], t=60)
    _command(bridge, kind="twist", linear=[0.02, 0, 0], t=60)
    _drive(bridge, 1)
    assert _twists(bridge) == [(0.02, 0, 0, 0, 0, 0)]
    now[0] += bridge.config.max_cmd_s
    _drive(bridge, 1)
    assert _twists(bridge)[1:] == [(0, 0, 0, 0, 0, 0)]


def test_gripper_opening_is_forwarded_normalized(bridge):
    _command(bridge, kind="gripper", opening=0.25)
    assert bridge.gripper_command.publish.call_args.args[0].data == pytest.approx(0.25)
    bridge.ee_twist_command.publish.assert_not_called()


def test_state_reports_arm_joints_and_normalized_gripper(bridge):
    bridge._on_joint_state(
        JointState(
            name=["j1", "j2", "arm/gripper"],
            position=[0.1, 0.2, 0.425],
            velocity=[0.5, 0.0, 0.0],
            ts=7.0,
        )
    )
    (state,) = _published(bridge, "arm/state/json")
    assert state == {
        "t": 7.0,
        "joint_names": ["j1", "j2"],
        "positions": [0.1, 0.2],
        "velocities": [0.5, 0.0],
        "gripper_opening": pytest.approx(0.5),
    }


def test_info_is_published_and_only_camera_tfs_are_exported(bridge):
    _drive(bridge, 1)
    (info,) = _published(bridge, "arm/info/json")
    assert info["commands"] == ["twist", "gripper"]
    assert info["gripper"] == {"unit": "normalized", "closed": 0.0, "open": 1.0}
    camera = Transform(
        translation=Vector3(1, 2, 3),
        rotation=Quaternion(0, 0, 0, 1),
        frame_id="world",
        child_frame_id=bridge.config.camera_optical_frame,
        ts=time.time(),
    )
    hidden = Transform(frame_id="world", child_frame_id="cup", ts=time.time())
    bridge._on_tf(TFMessage(camera, hidden))
    (pose,) = _published(bridge, "camera_pose/json")
    assert pose["frame"] == "world" and pose["xyz"] == [1, 2, 3]
    assert pose["quaternion_xyzw"] == [0, 0, 0, 1]
    assert not _published(bridge, "cup")


def test_depth_round_trip_preserves_metric_values_and_invalid_pixels():
    source = np.array([[0.123456, 0.5, 1.25], [np.nan, np.inf, 0]], dtype=">f4")[:, ::-1]
    encoded = depth_f32(Image(data=source, format=ImageFormat.DEPTH))
    np.testing.assert_allclose(
        np.frombuffer(encoded, dtype="<f4").reshape(2, 3), source, equal_nan=True
    )


def test_rgb_topics_and_depth_metadata_keep_capture_timestamps(bridge):
    module = bridge
    module._on_image(Image(data=np.zeros((4, 6, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=10))
    module._on_overview_image(
        Image(data=np.zeros((8, 12, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=20)
    )
    module._on_depth(Image(data=np.ones((2, 3), dtype=np.float32), format=ImageFormat.DEPTH, ts=30))
    first, second, metadata, depth = [call.args for call in module._topics.put.call_args_list]
    assert first[0] == "camera/jpeg" and first[2] == 10
    assert second[0] == "overview/jpeg" and second[2] == 20
    assert PILImage.open(io.BytesIO(first[1])).size == (6, 4)
    assert PILImage.open(io.BytesIO(second[1])).size == (12, 8)
    assert json.loads(metadata[1])["dtype"] == "<f4"
    assert depth[0] == "camera/depth_f32" and depth[2] == metadata[2] == 30
