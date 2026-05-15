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

import math
import pickle
import time

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def test_lcm_encode_decode() -> None:
    """Test encoding and decoding of Pose to/from binary LCM format."""

    pose_source = PoseStamped(
        ts=time.time(),
        position=(1.0, 2.0, 3.0),
        orientation=(0.1, 0.2, 0.3, 0.9),
    )
    binary_msg = pose_source.lcm_encode()
    pose_dest = PoseStamped.lcm_decode(binary_msg)

    assert isinstance(pose_dest, PoseStamped)
    assert pose_dest is not pose_source

    print(pose_source.position)
    print(pose_source.orientation)

    print(pose_dest.position)
    print(pose_dest.orientation)
    assert pose_dest == pose_source


def test_pickle_encode_decode() -> None:
    """Test encoding and decoding of PoseStamped to/from binary LCM format."""

    pose_source = PoseStamped(
        ts=time.time(),
        position=(1.0, 2.0, 3.0),
        orientation=(0.1, 0.2, 0.3, 0.9),
    )
    binary_msg = pickle.dumps(pose_source)
    pose_dest = pickle.loads(binary_msg)
    assert isinstance(pose_dest, PoseStamped)
    assert pose_dest is not pose_source
    assert pose_dest == pose_source


def test_agent_encode_returns_absolute_fields() -> None:
    """Test that agent_encode returns frame_id, position, and full orientation."""

    pose = PoseStamped(
        position=Vector3(1.0, 2.0, 3.0),
        orientation=Quaternion.from_euler(
            Vector3(math.radians(10.0), math.radians(20.0), math.radians(45.0))
        ),
        frame_id="map",
    )
    encoded = pose.agent_encode()

    assert set(encoded.keys()) == {
        "frame_id",
        "x",
        "y",
        "z",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
    }
    assert encoded["frame_id"] == "map"
    assert encoded["x"] == pytest.approx(1.0)
    assert encoded["y"] == pytest.approx(2.0)
    assert encoded["z"] == pytest.approx(3.0)
    assert encoded["roll_deg"] == pytest.approx(10.0, abs=0.1)
    assert encoded["pitch_deg"] == pytest.approx(20.0, abs=0.1)
    assert encoded["yaw_deg"] == pytest.approx(45.0, abs=0.1)


def test_agent_encode_rounds_values() -> None:
    """Test that agent_encode rounds position to 3 dp and angles to 1 dp."""

    pose = PoseStamped(
        position=Vector3(1.23456, 2.34567, 3.45678),
        orientation=Quaternion.from_euler(
            Vector3(math.radians(10.123), math.radians(20.456), math.radians(45.789))
        ),
        frame_id="map",
    )
    encoded = pose.agent_encode()

    assert encoded["x"] == pytest.approx(1.235)
    assert encoded["y"] == pytest.approx(2.346)
    assert encoded["z"] == pytest.approx(3.457)
    assert encoded["roll_deg"] == pytest.approx(10.1, abs=0.05)
    assert encoded["pitch_deg"] == pytest.approx(20.5, abs=0.05)
    assert encoded["yaw_deg"] == pytest.approx(45.8, abs=0.05)
