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

from dimos_lcm.geometry_msgs import PoseArray as LCMPoseArray
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.std_msgs.Header import Header


def test_lcm_round_trip():
    source = PoseArray(
        Header(12.5, "world"),
        [Pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0), PoseStamped(4.0, 5.0, 6.0, 0.0, 0.0, 1.0, 0.0)],
    )
    back = PoseArray.lcm_decode(source.lcm_encode())

    assert isinstance(back, PoseArray)
    assert back.frame_id == "world"
    assert back.ts == 12.5
    assert all(type(pose) is Pose for pose in back)
    np.testing.assert_array_equal(back.positions(), [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    assert back[1].orientation.z == 1.0


def test_wire_matches_the_generated_type():
    source = PoseArray(Header(12.5, "world"), [Pose(1.0, 2.0, 3.0)])
    raw = LCMPoseArray.lcm_decode(source.lcm_encode())
    assert raw.poses_length == 1
    assert raw.header.frame_id == "world"
    assert raw.poses[0].position.x == 1.0


def test_empty_round_trip_and_render():
    back = PoseArray.lcm_decode(PoseArray(Header(1.0, "world")).lcm_encode())
    assert len(back) == 0
    assert back.positions().shape == (0, 3)
    back.to_rerun()
    PoseArray(Header(1.0, "world"), [Pose(1.0, 2.0, 3.0)]).to_rerun()
