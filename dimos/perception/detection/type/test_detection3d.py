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

import pytest

from dimos.msgs.geometry import transform_matrix
from dimos.msgs.time import to_seconds

pytestmark = pytest.mark.self_hosted


def test_recording_keeps_source_and_recorded_time_domains(get_moment_2d):
    moment = get_moment_2d()
    image = moment["image_frame"]
    odom = moment["odom_frame"]
    buffer = moment["tf"]
    # These paired recorder entries have different sensor source clocks.
    # Migration must preserve that fact rather than rewrite timestamps.
    assert to_seconds(image.header.stamp) - to_seconds(odom.header.stamp) > 50
    assert buffer.get("camera_optical", "world", to_seconds(image.header.stamp), 5.0) is None
    transform = buffer.get("camera_optical", "world", to_seconds(odom.header.stamp), 1e-6)
    assert transform is not None
    assert transform.header.frame_id == "camera_optical"
    assert transform.child_frame_id == "world"
    assert transform_matrix(transform.transform).shape == (4, 4)
    assert len(moment["detections2d"]) > 0
