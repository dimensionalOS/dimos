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

import json

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry


def test_agent_encode_preserves_native_pose_twist_and_frames():
    odometry = Odometry(
        ts=12.5,
        frame_id="map",
        child_frame_id="base_link",
        pose=Pose(position=[1.25, -2.5, 0.75], orientation=[0, 0, 0.6, 0.8]),
        twist=Twist(linear=[1, -2, 3], angular=[-0.1, 0.2, -0.3]),
    )

    encoded = odometry.agent_encode()

    assert json.loads(json.dumps(encoded, allow_nan=False)) == {
        "ts": 12.5,
        "frame_id": "map",
        "child_frame_id": "base_link",
        "position_m": [1.25, -2.5, 0.75],
        "orientation_xyzw": [0.0, 0.0, 0.6, 0.8],
        "linear_velocity_m_s": [1.0, -2.0, 3.0],
        "angular_velocity_rad_s": [-0.1, 0.2, -0.3],
    }
