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

from dimos_lcm.geometry_msgs.Point import Point
from dimos_lcm.geometry_msgs.Pose import Pose
from dimos_lcm.geometry_msgs.PoseStamped import PoseStamped
from dimos_lcm.geometry_msgs.Quaternion import Quaternion
from dimos_lcm.nav_msgs.Path import Path
from dimos_lcm.std_msgs.Header import Header
from dimos_lcm.std_msgs.Time import Time
import numpy as np
import pytest

from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D


def encode_edges(n_segments: int, frame_ids: list[str] | None = None) -> bytes:
    """A Path of consecutive pose pairs with orientation.w carrying the weight."""
    poses = [
        PoseStamped(
            header=Header(frame_id=frame_ids[i % len(frame_ids)] if frame_ids else "odom"),
            pose=Pose(
                position=Point(float(i), float(i) * 0.5, -1.0),
                orientation=Quaternion(w=float(i // 2) * 0.1),
            ),
        )
        for i in range(2 * n_segments)
    ]
    header = Header(stamp=Time(12, 500_000_000), frame_id="odom")
    return Path(poses_length=len(poses), header=header, poses=poses).lcm_encode()


def expected_segments(n_segments: int) -> np.ndarray:
    i = np.arange(2 * n_segments, dtype=np.float64)
    return np.stack([i, i * 0.5, np.full_like(i, -1.0)], axis=1).reshape(-1, 2, 3)


def test_decode_matches_the_wire_layout() -> None:
    msg = LineSegments3D.lcm_decode(encode_edges(50))
    assert msg.frame_id == "odom"
    assert msg.ts == 12.5
    np.testing.assert_array_equal(msg.segments, expected_segments(50))
    np.testing.assert_allclose(msg.weights, np.arange(50) * 0.1)

    empty = LineSegments3D.lcm_decode(encode_edges(0))
    assert len(empty) == 0
    assert empty.segments.shape == (0, 2, 3)
    assert empty.weights.shape == (0,)


def test_mixed_frame_id_lengths_are_rejected() -> None:
    raw = encode_edges(20, frame_ids=["odom", "map", "base_link"])
    with pytest.raises(ValueError, match="frame_id length"):
        LineSegments3D.lcm_decode(raw)
