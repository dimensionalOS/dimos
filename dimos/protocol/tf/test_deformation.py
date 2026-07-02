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

"""Live-path loop-closure correction (DeformationBuffer)."""

from __future__ import annotations

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.DeformationNode import DeformationNode, tf_id_for
from dimos.protocol.tf.deformation import DeformationBuffer


def _node(tf_id: int, node_id: int, ts: float, xyz: tuple[float, float, float]) -> DeformationNode:
    return DeformationNode(
        id=node_id,
        tf_id=tf_id,
        pose=PoseStamped(ts=ts, frame_id="map", position=list(xyz), orientation=[0, 0, 0, 1]),
    )


def _identity(parent: str, child: str, ts: float) -> Transform:
    return Transform(
        translation=Vector3(0, 0, 0),
        rotation=Quaternion(0, 0, 0, 1),
        frame_id=parent,
        child_frame_id=child,
        ts=ts,
    )


def test_single_keyframe_corrects_matched_edge_only() -> None:
    buffer = DeformationBuffer()
    tf_id = tf_id_for("map", "odom")
    buffer.receive(_node(tf_id, 1, 100.0, (0.0, 0.0, 0.0)))  # original
    buffer.receive(_node(tf_id, 1, 100.0, (1.0, 0.0, 0.0)))  # optimizer moved it +1 in x

    corrected = buffer.correct("map", "odom", _identity("map", "odom", 100.0), 100.0)
    assert abs(corrected.translation.x - 1.0) < 1e-9  # delta = current ∘ inv(original) applied

    # an edge with no matching tf_id passes through untouched
    other = _identity("odom", "base_link", 100.0)
    assert buffer.correct("odom", "base_link", other, 100.0).translation.x == 0.0


def test_blends_between_bracketing_keyframes() -> None:
    buffer = DeformationBuffer()
    tf_id = tf_id_for("map", "odom")
    buffer.receive(_node(tf_id, 1, 100.0, (0.0, 0.0, 0.0)))  # kf A: unmoved
    buffer.receive(_node(tf_id, 1, 100.0, (0.0, 0.0, 0.0)))
    buffer.receive(_node(tf_id, 2, 110.0, (0.0, 0.0, 0.0)))  # kf B: moved +2
    buffer.receive(_node(tf_id, 2, 110.0, (2.0, 0.0, 0.0)))

    corrected = buffer.correct("map", "odom", _identity("map", "odom", 105.0), 105.0)
    assert abs(corrected.translation.x - 1.0) < 1e-9  # midpoint of 0->2


def test_no_nodes_passes_through_identity() -> None:
    buffer = DeformationBuffer()
    raw = _identity("map", "odom", 1.0)
    assert buffer.correct("map", "odom", raw, 1.0) is raw  # no copy, no cost
