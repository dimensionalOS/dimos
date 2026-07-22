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

"""Offline ``StreamTF`` + world-registration helpers for post-processing a recording."""

from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory2.tf import StreamTF
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.jnav.utils.recording_db import iterate_stream, store

if TYPE_CHECKING:
    from dimos.memory2.type.observation import Observation

# (odom stream, lidar-fallback candidates). First pair whose odom stream exists wins, so a
# recording never mixes rigs (e.g. fastlio + pointlio). Ordered mid360 rig -> go2 -> generic.
STREAM_PAIRS: list[tuple[str, list[str]]] = [
    ("pointlio_odometry", ["pointlio_lidar"]),
    ("fastlio_odometry", ["fastlio_lidar"]),
    ("go2_odom", ["go2_lidar", "l1_lidar", "lidar"]),
    ("odom", ["lidar"]),
]


def resolve_streams(
    available: set[str] | list[str], odom: str = "", lidar: str = ""
) -> tuple[str, str]:
    """``(odom_stream, lidar_stream)`` defaults from what a recording actually has."""
    if not odom:
        odom = next((name for name, _ in STREAM_PAIRS if name in available), "odom")
    if not lidar:
        candidates = next((ls for name, ls in STREAM_PAIRS if name == odom), ["lidar"])
        lidar = next((name for name in candidates if name in available), candidates[0])
    return odom, lidar


def default_odom_edge(store: Any, odom_stream: str) -> str:
    """``"parent:child"`` from the odom stream's own header, or ``""`` if it has no child frame
    (e.g. ``PoseStamped`` odometry)."""
    observation = next(iter(store.stream(odom_stream)), None)
    if observation is None:
        return ""
    child_frame = getattr(observation.data, "child_frame_id", "")
    if not child_frame:
        return ""
    return f"{observation.data.frame_id}:{child_frame}"


def payload_pose(payload: Any) -> Pose:
    """Pose from an odom payload — ``Odometry`` (has ``.pose``) or flat ``PoseStamped``."""
    if hasattr(payload, "pose"):
        return payload.pose  # type: ignore[no-any-return]
    return Pose(
        payload.x,
        payload.y,
        payload.z,
        payload.orientation.x,
        payload.orientation.y,
        payload.orientation.z,
        payload.orientation.w,
    )


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5 or 1.0
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        float,
    )


def world_register(
    observation: Observation[Any],
    store_tf: StreamTF | None,
    world_frame: str,
    fallback_frame: str,
    origin_lookup: Callable[[float], tuple[float, float, float]] | None = None,
) -> tuple[np.ndarray, tuple[float, float, float] | None]:
    """``(points_world_f32, sensor_origin_world)`` for one lidar observation.

    The scan's own ``frame_id`` decides everything: a scan already in
    ``world_frame`` is returned untouched; otherwise it is brought into the world
    via tf (``world_frame <- scan_frame``) and the tf translation is the ray
    origin. Falls back to ``fallback_frame`` when the scan carries no frame, then
    to the stored pose, then to assuming it is already world. ``origin_lookup``
    supplies the ray origin for already-world scans with no per-scan pose.
    Origin is ``None`` when none can be resolved (the caller skips that scan).
    """
    points = np.asarray(observation.data.points_f32())
    if not len(points):
        return points, None
    pose = observation.pose_tuple
    scan_frame = getattr(observation.data, "frame_id", "") or fallback_frame
    if scan_frame == world_frame:
        if pose is not None:
            origin = (float(pose[0]), float(pose[1]), float(pose[2]))
        elif origin_lookup is not None:
            origin = origin_lookup(float(observation.ts))
        else:
            origin = None
        return points.astype(np.float32), origin
    if store_tf is not None:
        transform = store_tf.get(world_frame, scan_frame, float(observation.ts), None)
        if transform is not None:
            rotation = np.asarray(transform.rotation.to_rotation_matrix(), float).reshape(3, 3)
            translation = np.array(
                [transform.translation.x, transform.translation.y, transform.translation.z], float
            )
            world = points @ rotation.T + translation
            return world.astype(np.float32), (
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
            )
    if pose is not None:
        rotation = _quat_to_matrix(pose[3], pose[4], pose[5], pose[6])
        world = points @ rotation.T + np.array(pose[:3], float)
        return world.astype(np.float32), (float(pose[0]), float(pose[1]), float(pose[2]))
    return points.astype(np.float32), None


def _format_tree(buffers: dict[tuple[str, str], Any]) -> str:
    children: dict[str, list[str]] = {}
    all_children = set()
    for parent, child in buffers:
        children.setdefault(parent, []).append(child)
        all_children.add(child)
    roots = sorted({parent for parent, _ in buffers} - all_children) or sorted(
        {parent for parent, _ in buffers}
    )
    lines: list[str] = []

    def walk(frame: str, depth: int, seen: frozenset[str]) -> None:
        lines.append("  " * depth + frame)
        if frame in seen:
            return
        for kid in sorted(children.get(frame, [])):
            walk(kid, depth + 1, seen | {frame})

    for root in roots:
        walk(root, 0, frozenset())
    return "\n".join(lines)


class RecordingTF(StreamTF):
    """``StreamTF`` that caches the whole recording's tf in one pass.

    ``StreamTF`` is built for live/windowed use: each lookup keeps only a window
    around the query time and evicts everything else. Recordings often publish
    near-static frames (sensor mounts, ``world->map``) exactly once at the start
    of the ``tf`` stream rather than into ``tf_static``, so a windowed lookup at
    any later time drops those edges and the transform chain breaks. Loading the
    full stream once keeps them buffered for the entire run; the only time-varying
    edge (``odom->base_link``) is densely sampled, so a nearest lookup
    (``time_tolerance=None``) reproduces the pose at each query time.

    An optional odom override (``odom_tf`` + ``odom_stream``) replaces one exact
    edge with a fed odometry stream so tf reflects the trajectory being scored.
    The override is applied lazily on first lookup, so recordings whose scans are
    already world-framed (no tf lookup needed) never pay for it.
    """

    _odom_edge: tuple[str, str] | None = None
    _odom_stream: Any = None

    @classmethod
    def from_store(
        cls,
        store: Any,
        stream: str = "tf",
        *,
        odom_tf: str | None = None,
        odom_stream: str | None = None,
    ) -> RecordingTF | None:
        if stream not in store.list_streams():
            return None
        recording_tf = cls(store.stream(stream, TFMessage))
        if odom_tf and odom_stream and odom_stream in store.list_streams():
            parent, _, child = odom_tf.partition(":")
            recording_tf._odom_edge = (parent, child)
            recording_tf._odom_stream = store.stream(odom_stream, Odometry)
        return recording_tf

    def _ensure(self, lo: float, hi: float) -> None:
        with self._cv:
            if self._covered is not None:
                return
            for observation in self.stream:
                self.receive_transform(*observation.data.transforms)
            if self._odom_edge is not None:
                self._apply_odom_override()
            self._covered = (-math.inf, math.inf)

    def _apply_odom_override(self) -> None:
        parent, child = self._odom_edge  # type: ignore[misc]
        self.buffers.pop((parent, child), None)
        self.receive_transform(
            *[
                Transform(
                    translation=observation.data.position,
                    rotation=observation.data.orientation,
                    frame_id=parent,
                    child_frame_id=child,
                    ts=float(observation.ts),
                )
                for observation in self._odom_stream
            ]
        )
        parents_by_child: dict[str, set[str]] = {}
        for edge_parent, edge_child in self.buffers:
            parents_by_child.setdefault(edge_child, set()).add(edge_parent)
        conflicts = {
            child: parents for child, parents in parents_by_child.items() if len(parents) > 1
        }
        if conflicts:
            detail = "\n".join(
                f"  {frame} now has parents: {', '.join(sorted(parents))}"
                for frame, parents in conflicts.items()
            )
            raise ValueError(
                f"--odom-tf {parent}:{child} breaks the tf tree (a frame gains a second parent).\n"
                f"replacement edge: {parent} -> {child}\n{detail}\n\n"
                f"current tree:\n{_format_tree(self.buffers)}"
            )


# tf may come online slightly after the odom stream starts (sensor/static warmup);
# odom samples before tf is available are skipped only within this opening window.
TF_STARTUP_TOLERANCE_S = 2.0


def tf_pose_samples(
    db_path: Path,
    odom_stream: str,
    *,
    world_frame: str,
    body_frame: str,
    startup_tolerance_s: float = TF_STARTUP_TOLERANCE_S,
) -> tuple[np.ndarray, np.ndarray]:
    """Robot trajectory composed from the recording's tf (``world_frame ->
    body_frame``) sampled at each odom timestamp.

    tf is REQUIRED — this raises (no odom-pose fallback) when the recording has no
    tf tree, or when tf never comes online within ``startup_tolerance_s`` of the
    odom start. Missing samples after tf is online (sporadic lookup gaps) are
    skipped."""
    db_store = store(db_path)
    tf = RecordingTF.from_store(db_store)
    if tf is None:
        raise SystemExit(
            f"{db_path}: no tf tree — eval requires tf (run add_tf first); odom fallback removed"
        )
    times: list[float] = []
    poses: list[list[float]] = []
    odom_t0: float | None = None
    tf_online = False
    for timestamp, _payload in iterate_stream(db_path, odom_stream):
        if odom_t0 is None:
            odom_t0 = timestamp
        # tolerance=None -> nearest recorded sample per edge; RecordingTF keeps the
        # whole recording buffered so one-shot static frames stay resolvable.
        transform = tf.get(world_frame, body_frame, timestamp, None)
        if transform is None:
            if tf_online:
                continue  # sporadic gap once tf is up — skip this sample
            if timestamp - odom_t0 <= startup_tolerance_s:
                continue  # opening warmup window — tf not online yet, tolerated
            raise SystemExit(
                f"{db_path}: tf did not come online within {startup_tolerance_s}s of the"
                f" odom start (world={world_frame!r} body={body_frame!r})"
            )
        tf_online = True
        times.append(timestamp)
        poses.append(
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ]
        )
    if not times:
        raise SystemExit(f"{db_path}: tf produced no {world_frame}->{body_frame} samples")
    return (
        np.asarray(times, dtype=np.float64),
        np.asarray(poses, dtype=np.float64).reshape(-1, 7),
    )
