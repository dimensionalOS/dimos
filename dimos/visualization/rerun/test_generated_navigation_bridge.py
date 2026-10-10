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

from types import SimpleNamespace
from unittest.mock import patch

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry, Path
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest
import rerun as rr

from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.message_helpers import navigation_archetype

pytestmark = pytest.mark.filterwarnings("error::rerun.error_utils.RerunWarning")


@pytest.mark.parametrize("kind", ["pose", "odom", "point", "path"])
def test_navigation_bridge_preserves_coordinates_and_frame(kind: str) -> None:
    header = Header(frame_id="map", stamp=Time(sec=0, nanosec=0))
    pose = Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0))
    messages = {
        "pose": PoseStamped(header=header, pose=pose),
        "odom": Odometry(
            header=header,
            pose=PoseWithCovariance(pose=pose, covariance=np.zeros(36, dtype=np.float64)),
            child_frame_id="",
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
        ),
        "point": PointStamped(header=header, point=pose.position),
        "path": Path(header=header, poses=[PoseStamped(header=header, pose=pose)]),
    }
    value = messages[kind]
    decoded = cdr_decode(cdr_encode(value), type(value))
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}
    try:
        with patch("rerun.log") as log:
            bridge._on_message(decoded, SimpleNamespace(name="/navigation"))
            output = log.call_args_list[0].args[1]
            if kind in ("pose", "odom"):
                assert isinstance(output, rr.Transform3D)
                assert output.translation.as_arrow_array().to_pylist() == [[1, 2, 3]]
                assert output.parent_frame.as_arrow_array().to_pylist() == ["tf#/map"]
            else:
                assert log.call_args_list[1].args[1].parent_frame.as_arrow_array().to_pylist() == [
                    "tf#/map"
                ]
                if kind == "point":
                    assert output.positions.as_arrow_array().to_pylist() == [[1, 2, 3]]
                else:
                    assert output.strips.as_arrow_array().to_pylist() == [[[1, 2, 3.5]]]
    finally:
        bridge.stop()
    assert cdr_encode(decoded) == cdr_encode(value)


def test_empty_generated_path_clears_geometry() -> None:
    assert (
        navigation_archetype(
            Path(header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""), poses=[])
        )
        .strips.as_arrow_array()
        .to_pylist()
        == []
    )


def test_path_override_preserves_configured_display_height():
    value = Path(
        poses=[
            PoseStamped(
                pose=Pose(
                    position=Point(x=1, y=2, z=3),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            )
        ],
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    result = navigation_archetype(value, z_offset=0.3)
    assert result.strips.as_arrow_array().to_pylist() == [[[1, 2, pytest.approx(3.3)]]]
    assert value.poses[0].pose.position.z == 3
