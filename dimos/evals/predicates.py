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

"""Ground-truth predicates: physical outcomes from simulator truth, not agent memory.

Interactive cases that declare ``ground_truth=True`` get a second store in
their score callable — the GT recorder's db of world poses the sim publishes
for every free-joint scene body (MujocoSimModule ``publish_ground_truth``).
Predicates here turn a role's pose history into a 0.0/1.0 outcome that
composes with the usual scorers and aggregates::

    ROLES = {"cup": "cup", "apple": "apple"}

    def score(store: Store, gt: Store) -> float:
        picked = lifted(gt, ROLES, "cup", min_delta=0.05)
        collateral = displaced(gt, ROLES, "apple", threshold=0.10)
        return picked * (1.0 - collateral)

    InteractiveEval(..., score=score, ground_truth=True, aggregate=final)

``roles`` maps the case's semantic names to GT body names (the pose's
``frame_id``). All predicates raise LookupError while the GT stream has no
data for the role — the runner's sampler treats LookupError as "not yet,
keep waiting".
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

if TYPE_CHECKING:
    from dimos.memory.store.base import Store

GT_STREAM = "gt_object_poses"
"""Stream (and module port) the simulator's ground-truth poses flow on."""

# Body up-axis alignment below which a pose counts as toppled (cos 60°).
_UP_DOT_MIN = 0.5


def gt_poses(
    gt: Store, roles: Mapping[str, str], role: str, *, stream: str = GT_STREAM
) -> list[PoseStamped]:
    """Time-ordered world poses of the body *role* maps to.

    The GT stream multiplexes every scene body on one stream; ``frame_id``
    carries the body name.
    """
    body = roles[role]
    poses = [obs.data for obs in gt.streams[stream] if obs.data.frame_id == body]
    if not poses:
        raise LookupError(f"no GT poses for role {role!r} (body {body!r}) on stream {stream!r}")
    return poses


def lifted(gt: Store, roles: Mapping[str, str], role: str, *, min_delta: float) -> float:
    """1.0 once the body's z rises ``min_delta`` above its episode-start z."""
    poses = gt_poses(gt, roles, role)
    gain = max(p.position.z for p in poses) - poses[0].position.z
    return float(gain > min_delta)


def displaced(gt: Store, roles: Mapping[str, str], role: str, *, threshold: float) -> float:
    """1.0 once the body's xy distance from its episode-start xy exceeds
    ``threshold`` — the knock-down/knock-aside detector."""
    poses = gt_poses(gt, roles, role)
    x0, y0 = poses[0].position.x, poses[0].position.y
    farthest = max(((p.position.x - x0) ** 2 + (p.position.y - y0) ** 2) ** 0.5 for p in poses)
    return float(farthest > threshold)


def knocked_over(gt: Store, roles: Mapping[str, str], role: str) -> float:
    """1.0 once the body's up axis tips more than 60° off world-up."""
    poses = gt_poses(gt, roles, role)
    for p in poses:
        up_z = p.orientation.to_rotation_matrix()[2, 2]
        if up_z < _UP_DOT_MIN:
            return 1.0
    return 0.0


def near(gt: Store, roles: Mapping[str, str], role_a: str, role_b: str, *, dist: float) -> float:
    """1.0 when the two bodies' latest positions are within ``dist`` (3D)."""
    a, b = gt_poses(gt, roles, role_a)[-1], gt_poses(gt, roles, role_b)[-1]
    return float((a.position - b.position).length() <= dist)


def contained(
    gt: Store,
    roles: Mapping[str, str],
    role_a: str,
    role_b: str,
    *,
    xy_tol: float = 0.1,
) -> float:
    """1.0 when a sits inside b: within ``xy_tol`` of b's xy and above b's z.

    Pose-derived approximation — GT carries body poses, not shape extents,
    so b's "footprint" is the tolerance disc around its center and its "top"
    is its origin height. Size ``xy_tol`` to the container, not the content.
    """
    a, b = gt_poses(gt, roles, role_a)[-1], gt_poses(gt, roles, role_b)[-1]
    dx, dy = a.position.x - b.position.x, a.position.y - b.position.y
    return float((dx**2 + dy**2) ** 0.5 <= xy_tol and a.position.z > b.position.z)


def grasped(gt: Store, roles: Mapping[str, str], role: str) -> float:
    """Whether the gripper holds the object — placeholder.

    Not decidable from poses alone: a pose-only heuristic (object tracks the
    end effector) also fires on pushes and drags. Blocked on the contact /
    gripper-force channel from issue #3594 phase 2.
    """
    raise NotImplementedError(
        "grasped() needs the contact channel (issue #3594 phase 2); poses alone can't tell a grasp from a push"
    )
