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

from dimos_generated.geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
)
from dimos_generated.nav_msgs.msg import Odometry, Path
from dimos_generated.std_msgs.msg import Header
import pytest
import rerun as rr

from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.message_helpers import navigation_archetype

pytestmark = pytest.mark.filterwarnings("error::rerun.error_utils.RerunWarning")


@pytest.mark.parametrize("kind", ["pose", "odom", "point", "path"])
def test_navigation_bridge_preserves_coordinates_and_frame(kind: str) -> None:
    header = Header(frame_id="map")
    pose = Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(w=1))
    messages = {
        "pose": PoseStamped(header=header, pose=pose),
        "odom": Odometry(header=header, pose=PoseWithCovariance(pose=pose)),
        "point": PointStamped(header=header, point=pose.position),
        "path": Path(header=header, poses=[PoseStamped(header=header, pose=pose)]),
    }
    value = messages[kind]
    decoded = type(value).decode(value.encode())
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
    assert decoded.encode() == value.encode()


def test_empty_generated_path_clears_geometry() -> None:
    assert navigation_archetype(Path()).strips.as_arrow_array().to_pylist() == []
