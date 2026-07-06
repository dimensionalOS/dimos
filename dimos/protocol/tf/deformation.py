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

"""Loop-closure deformation correction for transform lookups.

A loop-closure backend (e.g. gsc_pgo) publishes a stream of :class:`DeformationNode`
keyframes — one per pose-graph node, re-published (same id) when the optimizer moves
it, tagged with the tf_id of the edge it corrects. This module turns that stream into
a query-time correction on a raw edge transform:

  delta[k] = current[k] ∘ inv(original[k])      # how far keyframe k moved since first seen
  delta(t) = slerp/lerp blend of the two keyframes bracketing t
  corrected_edge = delta(t) ∘ raw_edge

Used by the live tf path (:class:`PubSubTF`) so that a running consumer's
tf lookups reflect loop-closure corrections as the optimizer republishes nodes.
"""

from __future__ import annotations

import bisect
from typing import Any, cast

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.DeformationNode import DeformationNode, tf_id_for


def apply_delta(delta: np.ndarray, transform: Transform, ts: float) -> Transform:
    """Deform ``transform`` by the SE(3) correction ``delta`` on the parent (frame_id)
    side: ``corrected = delta @ raw``. Keeps the edge's frame names."""
    corrected = delta @ transform.to_matrix()
    return Transform.from_matrix(
        corrected,
        ts=ts,
        frame_id=transform.frame_id,
        child_frame_id=transform.child_frame_id,
    )


def quat_slerp(q_lo: np.ndarray, q_hi: np.ndarray, weight: float) -> np.ndarray:
    """Spherical-linear interpolation between two quaternions ``[x, y, z, w]``."""
    dot = float(np.dot(q_lo, q_hi))
    if dot < 0.0:  # take the shorter arc
        q_hi = -q_hi
        dot = -dot
    if dot > 0.9995:  # nearly parallel: lerp + renormalize avoids a divide-by-~0
        result = q_lo + weight * (q_hi - q_lo)
        return cast("np.ndarray", result / np.linalg.norm(result))
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_0 = np.sin(theta_0)
    scale_lo = np.sin((1.0 - weight) * theta_0) / sin_0
    scale_hi = np.sin(weight * theta_0) / sin_0
    return cast("np.ndarray", scale_lo * q_lo + scale_hi * q_hi)


def blend_se3(mat_lo: np.ndarray, mat_hi: np.ndarray, weight: float) -> np.ndarray:
    """Linear-blend-skin two SE(3) deltas by ``weight`` in [0, 1] (0 -> lo, 1 -> hi):
    lerp the translation, slerp the rotation."""
    quat_lo = Quaternion.from_rotation_matrix(mat_lo[:3, :3])
    quat_hi = Quaternion.from_rotation_matrix(mat_hi[:3, :3])
    blended = quat_slerp(
        np.array([quat_lo.x, quat_lo.y, quat_lo.z, quat_lo.w]),
        np.array([quat_hi.x, quat_hi.y, quat_hi.z, quat_hi.w]),
        weight,
    )
    out = np.eye(4)
    out[:3, :3] = Quaternion(blended).to_rotation_matrix()
    out[:3, 3] = (1.0 - weight) * mat_lo[:3, 3] + weight * mat_hi[:3, 3]
    return out


class DeformationBuffer:
    """In-RAM rolling store of pose-graph keyframes, keyed by tf_id (the corrected edge).

    Feed it :class:`DeformationNode` messages via :meth:`receive`; it keeps, per node,
    the original (first-seen) and current (latest) pose. :meth:`correct` applies the
    blended loop-closure delta to a raw edge transform at a query time."""

    def __init__(self) -> None:
        # tf_id -> node_id -> [original_4x4, current_4x4, keyframe_ts]
        self._edges: dict[int, dict[int, list[Any]]] = {}

    def receive(self, node: DeformationNode) -> None:
        matrix = Transform.from_pose(node.pose.frame_id, node.pose).to_matrix()
        edge = self._edges.setdefault(node.tf_id, {})
        state = edge.get(node.id)
        if state is None:
            edge[node.id] = [matrix, matrix, node.pose.ts]  # original, current, ts
        else:
            state[1] = matrix  # optimizer moved it -> update current only

    @property
    def has_corrections(self) -> bool:
        return bool(self._edges)

    def _edge_delta(self, tf_id: int, query_time: float) -> np.ndarray | None:
        edge = self._edges.get(tf_id)
        if not edge:
            return None
        # keyframes sorted by ts; bracket the query time (latest<=t, earliest>=t)
        ordered = sorted(edge.values(), key=lambda state: state[2])
        stamps = [state[2] for state in ordered]
        lo_index = bisect.bisect_right(stamps, query_time) - 1  # last at or before
        hi_index = bisect.bisect_left(stamps, query_time)  # first at or after
        picks = []
        if lo_index >= 0:
            picks.append(ordered[lo_index])
        if hi_index < len(ordered) and (not picks or ordered[hi_index] is not picks[0]):
            picks.append(ordered[hi_index])
        if not picks:
            return None
        # per keyframe: delta = current ∘ inv(original)
        samples = [(state[2], state[1] @ np.linalg.inv(state[0])) for state in picks]
        if len(samples) == 1 or samples[0][0] == samples[1][0]:
            return cast("np.ndarray", samples[0][1])
        (ts_lo, mat_lo), (ts_hi, mat_hi) = samples
        weight = (query_time - ts_lo) / (ts_hi - ts_lo)
        return blend_se3(mat_lo, mat_hi, weight)

    def correct(
        self, frame_from: str, frame_to: str, raw: Transform, query_time: float
    ) -> Transform:
        """Return the loop-closure-corrected ``frame_from <- frame_to`` transform, or
        ``raw`` unchanged if no deformation nodes match that edge."""
        if not self._edges:
            return raw
        delta = self._edge_delta(tf_id_for(frame_from, frame_to), query_time)
        return apply_delta(delta, raw, raw.ts) if delta is not None else raw
