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

import pickle
import time

from dimos.msgs.geometry_msgs import PoseStamped


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
