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

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.experimental.world_belief.absence import (
    ABSENT,
    OCCLUDED,
    OUT_OF_VIEW,
    PRESENT,
    classify_visibility,
)
from dimos.msgs.camera_info import camera_info_from_intrinsics


@pytest.mark.parametrize(
    "depth,expected", [(3.0, ABSENT), (1.0, PRESENT), (0.5, OCCLUDED), (0.0, OCCLUDED)]
)
def test_visibility_inverts_generated_camera_transform(depth, expected):
    calibration = camera_info_from_intrinsics(
        100, 100, 16, 16, 32, 32, header=Header(frame_id="camera")
    )
    world_from_camera = TransformStamped(
        header=Header(frame_id="world"),
        child_frame_id="camera",
        transform=Transform(translation=Vector3(x=10), rotation=Quaternion(w=1)),
    )
    # In world coordinates x=10; after inverse transform it is on the optical axis.
    center = Vector3(x=10, z=1)
    depth_m = np.full((32, 32), depth, dtype=np.float32)
    assert classify_visibility(center, calibration, world_from_camera, depth_m) == expected
    assert (
        classify_visibility(Vector3(x=10, z=-1), calibration, world_from_camera, depth_m)
        == OUT_OF_VIEW
    )


def test_visibility_requires_all_valid_depth_samples_to_show_absence():
    calibration = camera_info_from_intrinsics(
        100, 100, 16, 16, 32, 32, header=Header(frame_id="camera")
    )
    identity = TransformStamped(
        header=Header(frame_id="world"),
        child_frame_id="camera",
        transform=Transform(rotation=Quaternion(w=1)),
    )
    depth = np.full((32, 32), 3, dtype=np.float32)
    depth[16, 16] = 1
    assert classify_visibility(Vector3(z=1), calibration, identity, depth) == PRESENT
