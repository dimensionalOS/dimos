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

from dimos.experimental.agent_encode.pointcloud.legend import legend
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_legend_lists_every_operation_and_is_served_by_the_message():
    text = legend()

    for name in (
        "Box",
        "Cylinder",
        "Select",
        "Bounds",
        "Nearest",
        "Sweep",
        "Spacing",
        "Count",
        "ZMin",
        "ZMax",
        "ZPercentile",
        "Occupancy",
        "CameraView",
        "Arrow, Line",
        "Overview",
    ):
        assert f" import {name}\n" in text
    assert "x east, y north, z up" in text
    assert PointCloud2.agent_encode_legend() == text
