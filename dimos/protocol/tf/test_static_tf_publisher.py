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

"""Static frame definitions become generated ROS TF edges without convenience types."""

import math

from dimos_generated.tf2_msgs.msg import TFMessage
import pytest

from dimos.msgs.geometry import compose_transforms
from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms


def test_frame_tree_skips_root_and_composes_rotated_mounts():
    edges = frames_to_edge_transforms(
        [
            ("world", None, (0, 0, 0), (0, 0, 0)),
            ("body", "world", (1, 0, 0), (0, 0, math.pi / 2)),
            ("camera", "body", (2, 0, 3), (0, 0, 0)),
        ]
    )
    assert [(edge.header.frame_id, edge.child_frame_id) for edge in edges] == [
        ("world", "body"),
        ("body", "camera"),
    ]
    decoded = TFMessage.decode(TFMessage(transforms=edges).encode())
    combined = compose_transforms(*decoded.transforms)
    translation = combined.transform.translation
    assert (translation.x, translation.y, translation.z) == pytest.approx((1, 2, 3))
    assert combined.header.stamp.sec == 0
    assert combined.header.stamp.nanosec == 0


def test_empty_frame_tree_produces_no_edges():
    assert frames_to_edge_transforms([]) == []
