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

"""Geometry interpolators for the T5 registry: lerp, slerp, pose/transform.

Leaf convenience module — imports msg types, so it sits OUTSIDE the engine
layers (``align.py`` never imports it). Call :func:`install` once to register
interpolators for ``Vector3``/``Quaternion``/``Pose``/``PoseStamped``/
``Transform``; ``interpolate()`` fields of those types then resolve without
per-module registration. T11's tf sampler reuses :func:`slerp` for its buffer
math.
"""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.pure.align import register_interpolator


def lerp(a: float, b: float, alpha: float) -> float:
    """Linear interpolation between two floats."""
    return a + (b - a) * alpha


def lerp_vec(a: Vector3, b: Vector3, alpha: float) -> Vector3:
    """Component-wise linear interpolation between two vectors."""
    return Vector3(lerp(a.x, b.x, alpha), lerp(a.y, b.y, alpha), lerp(a.z, b.z, alpha))


def slerp(a: Quaternion, b: Quaternion, alpha: float) -> Quaternion:
    """Shortest-path spherical interpolation between two unit quaternions."""
    an = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w)
    bn = math.sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w)
    if an == 0.0 or bn == 0.0:
        raise ValueError("slerp: zero-norm quaternion")
    ax, ay, az, aw = a.x / an, a.y / an, a.z / an, a.w / an
    bx, by, bz, bw = b.x / bn, b.y / bn, b.z / bn, b.w / bn

    dot = ax * bx + ay * by + az * bz + aw * bw
    if dot < 0.0:  # q and -q are the same rotation; flip to take the short arc
        bx, by, bz, bw, dot = -bx, -by, -bz, -bw, -dot
    if dot > 0.9995:  # nearly parallel: sin(theta) ~ 0, nlerp is exact enough
        x = lerp(ax, bx, alpha)
        y = lerp(ay, by, alpha)
        z = lerp(az, bz, alpha)
        w = lerp(aw, bw, alpha)
        n = math.sqrt(x * x + y * y + z * z + w * w)
        return Quaternion(x / n, y / n, z / n, w / n)
    theta = math.acos(min(1.0, dot))
    s = math.sin(theta)
    wa = math.sin((1.0 - alpha) * theta) / s
    wb = math.sin(alpha * theta) / s
    return Quaternion(ax * wa + bx * wb, ay * wa + by * wb, az * wa + bz * wb, aw * wa + bw * wb)


def interp_pose(a: Pose, b: Pose, alpha: float) -> Pose:
    """Interpolate two poses: position lerp + orientation slerp."""
    p = lerp_vec(a.position, b.position, alpha)
    q = slerp(a.orientation, b.orientation, alpha)
    return Pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def _require_same_frame(kind: str, name: str, fa: str, fb: str) -> None:
    if fa != fb:
        raise ValueError(
            f"cannot interpolate {kind} samples from different frames "
            f"({name}={fa!r} vs {fb!r}); one stream must carry one frame"
        )


def interp_pose_stamped(a: PoseStamped, b: PoseStamped, alpha: float) -> PoseStamped:
    """Interpolate two stamped poses; ts is lerped (= the tick time)."""
    _require_same_frame("PoseStamped", "frame_id", a.frame_id, b.frame_id)
    p = lerp_vec(a.position, b.position, alpha)
    q = slerp(a.orientation, b.orientation, alpha)
    out = PoseStamped(
        ts=lerp(a.ts, b.ts, alpha),
        frame_id=a.frame_id,
        position=[p.x, p.y, p.z],
        orientation=[q.x, q.y, q.z, q.w],
    )
    out.ts = lerp(a.ts, b.ts, alpha)  # ctor swaps ts=0.0 for wall clock; force it
    return out


def interp_transform(a: Transform, b: Transform, alpha: float) -> Transform:
    """Interpolate two transforms on the same edge; ts is lerped (= the tick time)."""
    _require_same_frame("Transform", "frame_id", a.frame_id, b.frame_id)
    _require_same_frame("Transform", "child_frame_id", a.child_frame_id, b.child_frame_id)
    out = Transform(
        translation=lerp_vec(a.translation, b.translation, alpha),
        rotation=slerp(a.rotation, b.rotation, alpha),
        frame_id=a.frame_id,
        child_frame_id=a.child_frame_id,
        ts=lerp(a.ts, b.ts, alpha),
    )
    out.ts = lerp(a.ts, b.ts, alpha)  # ctor swaps ts=0.0 for wall clock; force it
    return out


_installed = False


def install() -> None:
    """Register the geometry interpolators with the pure registry (idempotent)."""
    global _installed
    if _installed:
        return
    register_interpolator(Vector3, lerp_vec)
    register_interpolator(Quaternion, slerp)
    register_interpolator(Pose, interp_pose)
    register_interpolator(PoseStamped, interp_pose_stamped)
    register_interpolator(Transform, interp_transform)
    _installed = True
