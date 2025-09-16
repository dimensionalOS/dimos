# Copyright 2025 Dimensional Inc.
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

import numpy as np
from dimos.perception.detection2d.conftest import detections2d, detections3d


def test_detections2d(detections2d):
    print(detections2d)


def test_detections3d(detections3d):
    print(detections3d)


def test_detection3d_to_pose(detections3d):
    """Test converting a Detection3D to PoseStamped."""
    # Get first detection
    if len(detections3d) > 0:
        det = detections3d[0]
        pose = det.to_pose()

        # Check that pose is valid
        assert pose.frame_id == "world"
        assert pose.ts == det.ts

        # Position should be the pointcloud center
        center = det.center()
        assert np.allclose([pose.position.x, pose.position.y, pose.position.z], center)

        # Orientation should be identity quaternion
        assert np.allclose(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            [0.0, 0.0, 0.0, 1.0],
        )

        print(f"Detection {det.name} pose: position={pose.position}")
