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

import json
import math
from pathlib import Path
import pickle
import time

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


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


def test_agent_encode_labels_pose_frame_components_and_units() -> None:
    pose = PoseStamped(
        ts=12.5,
        frame_id="world",
        position=(1.0, 2.0, 3.0),
        orientation=(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
    )

    encoded = pose.agent_encode()
    expected = {
        "frame_id": "world",
        "timestamp_s": 12.5,
        "position_m": [1.0, 2.0, 3.0],
        "quaternion_xyzw": [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
        "roll_pitch_yaw_deg": [0.0, 0.0, 90.0],
    }
    required_fields = set(expected)
    allowed_fields = required_fields | {"planar_position_m", "yaw_deg", "heading_xy"}

    assert required_fields <= set(encoded) <= allowed_fields
    assert encoded["frame_id"] == expected["frame_id"]
    assert encoded["timestamp_s"] == expected["timestamp_s"]
    assert encoded["position_m"] == expected["position_m"]
    assert encoded["quaternion_xyzw"] == expected["quaternion_xyzw"]
    assert encoded["roll_pitch_yaw_deg"] == pytest.approx(expected["roll_pitch_yaw_deg"])
    if "planar_position_m" in encoded:
        assert encoded["planar_position_m"] == pytest.approx([1.0, 2.0])
    if "yaw_deg" in encoded:
        assert encoded["yaw_deg"] == pytest.approx(90.0)
    if "heading_xy" in encoded:
        assert encoded["heading_xy"] == pytest.approx([0.0, 1.0])
    assert len(json.dumps(encoded, separators=(",", ":"))) <= 512


@pytest.mark.parametrize(
    ("position", "orientation", "expected_yaw", "expected_heading"),
    [
        ((-4.0, 5.0, 6.0), (0.0, 0.0, 0.0, 1.0), 0.0, (1.0, 0.0)),
        (
            (7.0, -8.0, 9.0),
            (0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)),
            -90.0,
            (0.0, -1.0),
        ),
    ],
)
def test_agent_encode_optional_fields_are_derived_per_pose(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    expected_yaw: float,
    expected_heading: tuple[float, float],
) -> None:
    encoded = PoseStamped(position=position, orientation=orientation).agent_encode()

    if "planar_position_m" in encoded:
        assert encoded["planar_position_m"] == pytest.approx(position[:2])
    if "yaw_deg" in encoded:
        assert encoded["yaw_deg"] == pytest.approx(expected_yaw)
    if "heading_xy" in encoded:
        assert encoded["heading_xy"] == pytest.approx(expected_heading)


def test_agent_encode_records_activity_when_enabled(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("DIMOS_AGENT_ACTIVITY_DIR", str(tmp_path))
    pose = PoseStamped(ts=12.5, frame_id="world", position=(1.0, 2.0, 3.0))

    encoded = pose.agent_encode()

    event = json.loads((tmp_path / "events.jsonl").read_text())
    assert event["event"] == "agent_encode"
    assert event["message_type"] == "PoseStamped"
    assert event["output"] == {"source_timestamp_s": pose.ts, "encoded": encoded}
