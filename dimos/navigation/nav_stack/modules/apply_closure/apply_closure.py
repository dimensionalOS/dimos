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

"""ApplyClosure: warp a DynamicCloud global map by a pose-graph correction.

Inputs:
- ``global_map``: the voxel map to warp (DynamicCloud)
- ``loop_closure_event``: a GraphDelta3D published by PGO when iSAM2 smooths
  the pose graph. ``nodes[i]`` is the pre-smooth keyframe; ``transforms[i]``
  is the SE(3) delta to apply (left-multiplied: ``post = T_delta @ T_pre``).

Each voxel is bound to the pose-graph timeline by its latest event timestamp
(``per_point_latest_timestamp``), and its warp is a two-nearest-neighbor LBS
blend: lerp on translation, slerp on rotation between the bracketing nodes'
deltas.

The effect: voxels with recent event timestamps follow the latest pose
corrections, older voxels barely move — matching the way pose-graph drift
accumulates along a trajectory.

Voxels without any event (timestamp 0) clip to the earliest node and get the
smallest correction, which is the conservative choice for "unknown age".
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation, Slerp

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.DynamicCloud import DynamicCloud
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Slerp requires strictly increasing input times. If two pose-graph nodes
# share a timestamp (degenerate input), bump later ones by this epsilon so
# the interpolator stays well-defined.
_TIME_DEDUP_EPS = 1e-9


def transform_to_matrix(transform: GraphDelta3D.Transform) -> np.ndarray:
    """Pack a ``GraphDelta3D.Transform`` (translation + quaternion) into 4x4."""
    quat = np.array(
        [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        quat = np.array([0.0, 0.0, 0.0, 1.0])
    else:
        quat = quat / norm
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rotation.from_quat(quat).as_matrix()
    out[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return out


def graph_delta_to_arrays(graph_delta: GraphDelta3D) -> tuple[np.ndarray, np.ndarray]:
    """Return (timestamps[N], deltas[N, 4, 4]) extracted from a GraphDelta3D.

    Node timestamps come from each ``node.pose.ts``; deltas come from each
    ``transforms[i]`` (treated as a world-frame correction per the
    GraphDelta3D ``post = T_delta @ T_pre`` convention).
    """
    n = len(graph_delta.nodes)
    timestamps = np.empty(n, dtype=np.float64)
    deltas = np.empty((n, 4, 4), dtype=np.float64)
    for i, (node, transform) in enumerate(
        zip(graph_delta.nodes, graph_delta.transforms, strict=True)
    ):
        timestamps[i] = float(node.pose.ts)
        deltas[i] = transform_to_matrix(transform)
    return timestamps, deltas


def _dedupe_times(times: np.ndarray) -> np.ndarray:
    """Bump any duplicate timestamps so the sequence is strictly increasing."""
    out = times.astype(np.float64).copy()
    for i in range(1, out.size):
        if out[i] <= out[i - 1]:
            out[i] = out[i - 1] + _TIME_DEDUP_EPS
    return out  # type: ignore[no-any-return]


def lbs_warp_positions(
    positions: np.ndarray,
    position_times: np.ndarray,
    node_times: np.ndarray,
    node_deltas: np.ndarray,
) -> np.ndarray:
    """Apply two-nearest-neighbor LBS to ``positions`` (M, 3) using ``node_deltas``.

    For each point, find the two pose-graph nodes whose timestamps bracket
    the point's time, slerp the rotations and lerp the translations between
    them by the time-fraction, and apply the blended delta. Points outside
    the node-time range clip to the nearest endpoint.

    Args:
        positions: (M, 3) world-space positions.
        position_times: (M,) per-position timestamps (seconds).
        node_times: (N,) strictly increasing node timestamps.
        node_deltas: (N, 4, 4) correction transforms per node.

    Returns:
        (M, 3) warped positions.
    """
    if positions.shape[0] == 0:
        return positions.astype(np.float64, copy=True)
    if node_times.shape[0] == 0:
        return positions.astype(np.float64, copy=True)
    if node_times.shape[0] == 1:
        delta = node_deltas[0]
        homog = np.concatenate(
            [positions, np.ones((positions.shape[0], 1), dtype=positions.dtype)], axis=1
        )
        return (homog @ delta.T)[:, :3]  # type: ignore[no-any-return]

    node_times_safe = _dedupe_times(node_times)
    clipped = np.clip(position_times, node_times_safe[0], node_times_safe[-1])

    node_R = Rotation.from_matrix(node_deltas[:, :3, :3])
    slerp = Slerp(node_times_safe, node_R)
    point_R = slerp(clipped)

    node_t = node_deltas[:, :3, 3]
    tx = np.interp(clipped, node_times_safe, node_t[:, 0])
    ty = np.interp(clipped, node_times_safe, node_t[:, 1])
    tz = np.interp(clipped, node_times_safe, node_t[:, 2])
    translation = np.stack([tx, ty, tz], axis=1)

    return point_R.apply(positions.astype(np.float64)) + translation  # type: ignore[no-any-return]


def merge_duplicate_voxels(
    voxels: np.ndarray,
    quantity: np.ndarray,
    event_indices: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Collapse voxels that share an integer position.

    Two original voxels can warp into the same int32 grid cell. Sum their
    ``quantity``, and remap ``event_indices`` so events still point at the
    surviving merged voxel.

    Returns ``(unique_voxels, merged_quantity, remapped_event_indices)``.
    """
    if voxels.shape[0] == 0:
        return voxels, quantity, event_indices
    unique, inverse = np.unique(voxels, axis=0, return_inverse=True)
    merged_q = np.zeros(unique.shape[0], dtype=np.uint64)
    np.add.at(merged_q, inverse, quantity.astype(np.uint64))
    merged_q = np.minimum(merged_q, np.iinfo(np.uint32).max).astype(np.uint32)
    if event_indices.size == 0:
        new_events = event_indices
    else:
        new_events = inverse[event_indices.astype(np.intp)].astype(np.uint32)
    return unique.astype(np.int32), merged_q, new_events


def apply_closure_to_cloud(
    cloud: DynamicCloud,
    graph_delta: GraphDelta3D,
) -> DynamicCloud:
    """Warp ``cloud`` by the per-node deltas carried in ``graph_delta``.

    A pass-through if ``graph_delta`` has no nodes.
    """
    if len(graph_delta.nodes) == 0:
        return cloud

    node_times, deltas = graph_delta_to_arrays(graph_delta)
    order = np.argsort(node_times, kind="stable")
    node_times = node_times[order]
    deltas = deltas[order]

    world = cloud.world_positions().astype(np.float64)
    latest_ns = cloud.per_point_latest_timestamp()
    point_times = latest_ns.astype(np.float64) / 1_000_000_000.0

    new_world = lbs_warp_positions(world, point_times, node_times, deltas)
    new_voxels = np.rint(new_world / cloud.voxel_size).astype(np.int32)

    voxels, quantity, event_indices = merge_duplicate_voxels(
        new_voxels, cloud.quantity, cloud.event_indices
    )

    # event_timestamps is unchanged (the events still refer to the same physical
    # observations, just at remapped voxel indices). DynamicCloud copies/normalizes
    # the array internally so sharing the reference is safe.
    return DynamicCloud(
        voxels=voxels,
        quantity=quantity,
        event_indices=event_indices,
        event_timestamps=cloud.event_timestamps,
        voxel_size=cloud.voxel_size,
        frame_id=cloud.frame_id,
        ts=cloud.ts,
    )


class ApplyClosureConfig(ModuleConfig):
    world_frame: str = "map"
    # Log a one-line summary every time a correction is applied.
    log_each_apply: bool = True


class ApplyClosure(Module):
    """Warp the global voxel map by a pose-graph loop-closure correction."""

    config: ApplyClosureConfig

    global_map: In[DynamicCloud]
    loop_closure_event: In[GraphDelta3D]
    map_override: Out[DynamicCloud]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._latest_map: DynamicCloud | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.global_map.subscribe(self._on_global_map)))
        self.register_disposable(
            Disposable(self.loop_closure_event.subscribe(self._on_loop_closure))
        )
        logger.info("ApplyClosure started")

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_global_map(self, msg: DynamicCloud) -> None:
        with self._lock:
            self._latest_map = msg

    def _on_loop_closure(self, msg: GraphDelta3D) -> None:
        """Loop-closure trigger: apply correction to the latched map."""
        with self._lock:
            cloud = self._latest_map
        if cloud is None:
            return
        t0 = time.monotonic()
        corrected = apply_closure_to_cloud(cloud, msg)
        corrected.ts = time.time()
        self.map_override.publish(corrected)
        if self.config.log_each_apply:
            logger.info(
                "ApplyClosure applied",
                num_nodes=len(msg.nodes),
                num_points_in=len(cloud),
                num_points_out=len(corrected),
                elapsed_ms=round((time.monotonic() - t0) * 1000.0, 2),
            )
