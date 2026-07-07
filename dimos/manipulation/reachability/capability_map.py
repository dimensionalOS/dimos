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

"""Scored capability map for a humanoid arm, in the pelvis frame.

The map answers *"can the hand reach this pose from where the robot
stands?"* with a score, not a bit. It lives in the **gravity-aligned,
ground-level pelvis frame**: origin at the pelvis ground projection, z
along gravity, x along the pelvis heading; the pelvis is level at a fixed
height (the WBC's height command). θ is measured against gravity — pelvis
roll/pitch wobble is a disturbance, not a symmetry.

A 6D end-effector pose collapses to five indexed values:

```text
f(T) = (p_z, θ, x*, y*, gamma)
  p_z      TCP height above ground
  θ        angle between approach vector (TCP z-axis) and gravity z
  (x*,y*)  canonical planar offset (RM4D, arXiv:2410.06968 Eq. 2-4):
           translate the TCP ground-projection to the origin, rotate by
           -ψ (ψ = approach azimuth) so the approach vector lies in the
           x(+)z half-plane — the pelvis position in that frame
  gamma        in-plane rotation of the TCP x-axis about the approach vector
```

The (x*, y*) reduction is exact for this robot class because the WBC
gives a true SE(2) base (turn in place, fixed height, level pelvis) —
the quotiented symmetry is pelvis yaw. Consequently a **forward query is
heading-free**: "reachable from this pelvis position, possibly after
turning in place." "Reachable at the current heading right now" is one
mink IK solve, not a map query. gamma stays an explicit (coarse) dimension
because the G1 wrist_yaw is ±92.5°, far from the 360° that would justify
RM4D's 4D collapse; the 4D marginal is one ``max`` away when wanted.

Cells store saturating uint8 sample counts (a reachability *score*) plus
a per-(p_z,θ,x*,y*) bitmask of the construction-time approach azimuths
ψ — the heading hint a stance-selection executor needs to face the right
way on arrival (phase 5).

Known pole artifact (paper-faithful): at θ ≈ 0/π the azimuth ψ is
undefined and equivalent samples spray over a ring of (x*, y*) cells;
queries near the poles pick an arbitrary ring point. The evaluation
harness quantifies the cost.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any, TypeAlias

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

PoseInput: TypeAlias = "PoseStamped | NDArray[np.float64]"

_EPS = 1e-9
_UINT8_MAX = np.iinfo(np.uint8).max
_BODY_THETA_MASK_BITS = 64


def _base_link_pose_at_height(height: float) -> PoseStamped:
    return PoseStamped(
        frame_id="world",
        position=Vector3(0.0, 0.0, height),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


@dataclass(frozen=True)
class MapParams:
    """Discretization of the capability map.

    Defaults size the grid for a G1 arm: ~0.95 m reach from the pelvis
    axis, TCP heights up to ~1.8 m above ground, 5 cm cells, 5° approach
    bins, 30° in-plane bins.
    """

    r_xy: float = 1.0
    z_min: float = 0.0
    z_max: float = 1.8
    cell: float = 0.05
    n_theta: int = 36
    n_inplane: int = 12
    n_heading: int = 8  # ψ bins for the heading-hint bitmask (≤ 8 for uint8)
    base_link_pose: PoseStamped = field(default_factory=lambda: _base_link_pose_at_height(0.74))

    @property
    def n_z(self) -> int:
        return int(np.ceil((self.z_max - self.z_min) / self.cell))

    @property
    def n_xy(self) -> int:
        return int(np.ceil(2.0 * self.r_xy / self.cell))

    @property
    def pelvis_height(self) -> float:
        """Deprecated z-only alias for old call sites and map files."""
        return float(self.base_link_pose.position.z)

    @classmethod
    def at_base_height(cls, height: float, **kwargs: Any) -> MapParams:
        """Create map params with an identity-orientation base link at height z."""
        return cls(base_link_pose=_base_link_pose_at_height(height), **kwargs)

    @classmethod
    def from_json_dict(cls, values: dict[str, Any]) -> MapParams:
        """Load params from current or legacy serialized map metadata."""
        values = dict(values)
        legacy_height = values.pop("pelvis_height", None)
        if "base_link_pose" in values:
            values["base_link_pose"] = _pose_from_json(values["base_link_pose"])
        elif legacy_height is not None:
            values["base_link_pose"] = _base_link_pose_at_height(float(legacy_height))
        return cls(**values)

    def to_json_dict(self) -> dict[str, Any]:
        """Serialize params using JSON-native values."""
        return {
            "r_xy": self.r_xy,
            "z_min": self.z_min,
            "z_max": self.z_max,
            "cell": self.cell,
            "n_theta": self.n_theta,
            "n_inplane": self.n_inplane,
            "n_heading": self.n_heading,
            "base_link_pose": _pose_to_json(self.base_link_pose),
        }


def _pose_to_json(pose: PoseStamped) -> dict[str, Any]:
    return {
        "frame_id": pose.frame_id,
        "position": [pose.position.x, pose.position.y, pose.position.z],
        "orientation": [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    }


def _pose_from_json(values: dict[str, Any]) -> PoseStamped:
    return PoseStamped(
        frame_id=str(values.get("frame_id", "world")),
        position=Vector3(values.get("position", [0.0, 0.0, 0.0])),
        orientation=Quaternion(values.get("orientation", [0.0, 0.0, 0.0, 1.0])),
    )


def canonical_values(
    positions: NDArray[np.float64], rotations: NDArray[np.float64]
) -> tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    """Vectorized f(T): poses → (p_z, θ, x*, y*, gamma, ψ).

    Args:
        positions: (N, 3) TCP positions in the map frame.
        rotations: (N, 3, 3) TCP rotation matrices in the map frame.

    Returns:
        Six (N,) arrays. ψ is the approach azimuth (the quotiented gauge),
        returned for the heading hint.
    """
    p = np.atleast_2d(np.asarray(positions, dtype=np.float64))
    rot = np.asarray(rotations, dtype=np.float64).reshape(-1, 3, 3)

    r_z = rot[:, :, 2]
    p_z = p[:, 2]
    theta = np.arccos(np.clip(r_z[:, 2], -1.0, 1.0))
    psi = np.arctan2(r_z[:, 1], r_z[:, 0])

    c, s = np.cos(psi), np.sin(psi)
    x_star = c * (-p[:, 0]) + s * (-p[:, 1])
    y_star = -s * (-p[:, 0]) + c * (-p[:, 1])

    # In-plane angle gamma in the canonicalized frame (pose rotated by R_z(-ψ),
    # where the approach vector lies in the x(+)z half-plane). Reference
    # direction: the projection of gravity-z onto the plane ⊥ approach;
    # at the poles (approach ∥ z) fall back to the canonical x-axis.
    x_axis = rot[:, :, 0]
    # Rotate both vectors by R_z(-ψ).
    rzc = np.stack(
        [c * r_z[:, 0] + s * r_z[:, 1], -s * r_z[:, 0] + c * r_z[:, 1], r_z[:, 2]], axis=1
    )
    xc = np.stack(
        [c * x_axis[:, 0] + s * x_axis[:, 1], -s * x_axis[:, 0] + c * x_axis[:, 1], x_axis[:, 2]],
        axis=1,
    )
    z_hat = np.array([0.0, 0.0, 1.0])
    ref = z_hat - rzc * rzc[:, 2:3]
    norms = np.linalg.norm(ref, axis=1, keepdims=True)
    degenerate = norms[:, 0] < 1e-8
    if np.any(degenerate):
        x_hat = np.array([1.0, 0.0, 0.0])
        fallback = x_hat - rzc[degenerate] * rzc[degenerate, 0:1]
        fb_norm = np.linalg.norm(fallback, axis=1, keepdims=True)
        ref[degenerate] = fallback / np.maximum(fb_norm, _EPS)
        norms[degenerate] = 1.0
    ref = ref / np.maximum(norms, _EPS)
    e2 = np.cross(rzc, ref)
    t = xc - rzc * np.sum(xc * rzc, axis=1, keepdims=True)
    gamma = np.arctan2(np.sum(t * e2, axis=1), np.sum(t * ref, axis=1))

    return p_z, theta, x_star, y_star, gamma, psi


def pose_arrays(ee_poses: PoseInput) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return vectorized ``(positions, rotations)`` arrays for EE/TCP pose input.

    Args:
        ee_poses: A single :class:`PoseStamped`, one homogeneous ``(4, 4)``
            transform, or a batch of homogeneous ``(N, 4, 4)`` transforms.

    Returns:
        ``positions`` with shape ``(N, 3)`` and ``rotations`` with shape
        ``(N, 3, 3)``.
    """
    poses = _pose_matrix_batch(ee_poses)
    return poses[:, :3, 3], poses[:, :3, :3]


def canonical_values_from_poses(
    ee_poses: PoseInput,
) -> tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    """Vectorized f(T) for EE/TCP poses represented as homogeneous transforms."""
    return canonical_values(*pose_arrays(ee_poses))


def _pose_matrix_batch(ee_poses: PoseInput) -> NDArray[np.float64]:
    if isinstance(ee_poses, PoseStamped):
        return _pose_stamped_matrix(ee_poses)[None]
    poses = np.asarray(ee_poses, dtype=np.float64)
    if poses.shape == (4, 4):
        return poses[None]
    if poses.ndim == 3 and poses.shape[1:] == (4, 4):
        return poses
    raise ValueError(
        "EE pose input must be a PoseStamped, a homogeneous (4, 4) transform, "
        f"or a homogeneous (N, 4, 4) transform batch; got shape {poses.shape}"
    )


def _pose_stamped_matrix(pose: PoseStamped) -> NDArray[np.float64]:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _quat_xyzw_to_matrix(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    matrix[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return matrix


def _quat_xyzw_to_matrix(x: float, y: float, z: float, w: float) -> NDArray[np.float64]:
    quat = np.asarray((x, y, z, w), dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm < _EPS:
        raise ValueError("PoseStamped orientation quaternion has zero length")
    x, y, z, w = quat / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class CapabilityMap:
    """Scored per-arm capability map (forward queries; phase-5 inverse-ready)."""

    def __init__(
        self,
        params: MapParams,
        robot: str = "",
        model_id: str = "",
        counts: NDArray[np.uint8] | None = None,
        heading_hint: NDArray[np.uint8] | None = None,
        body_counts: NDArray[np.uint8] | None = None,
        body_theta_mask: NDArray[np.uint64] | None = None,
    ) -> None:
        self.params = params
        # ``robot`` is the registry key (e.g. "g1-left", "xarm7").
        self.robot = robot
        self.model_id = model_id
        shape5 = (params.n_z, params.n_theta, params.n_xy, params.n_xy, params.n_inplane)
        shape_body = (params.n_z, params.n_xy, params.n_xy)
        self.counts: NDArray[np.uint8] = (
            counts if counts is not None else np.zeros(shape5, dtype=np.uint8)
        )
        self.heading_hint: NDArray[np.uint8] = (
            heading_hint if heading_hint is not None else np.zeros(shape5[:4], dtype=np.uint8)
        )
        # Body-frame companions for visualization: where the TCP actually was
        # in pelvis coordinates (no heading quotient — the asymmetric blob a
        # human expects to see), and which approach angles were seen there.
        self.body_counts: NDArray[np.uint8] = (
            body_counts if body_counts is not None else np.zeros(shape_body, dtype=np.uint8)
        )
        self.body_theta_mask: NDArray[np.uint64] = (
            body_theta_mask
            if body_theta_mask is not None
            else np.zeros(shape_body, dtype=np.uint64)
        )
        if self.counts.shape != shape5:
            raise ValueError(f"counts shape {self.counts.shape} != params shape {shape5}")
        if params.n_theta > _BODY_THETA_MASK_BITS:
            raise ValueError(
                f"n_theta > {_BODY_THETA_MASK_BITS} does not fit the body theta bitmask"
            )

    # Indexing

    def indices(
        self,
        p_z: NDArray[np.float64],
        theta: NDArray[np.float64],
        x_star: NDArray[np.float64],
        y_star: NDArray[np.float64],
        gamma: NDArray[np.float64],
    ) -> tuple[
        NDArray[np.intp],
        NDArray[np.intp],
        NDArray[np.intp],
        NDArray[np.intp],
        NDArray[np.intp],
        NDArray[np.bool_],
    ]:
        """Vectorized 5D indices plus validity mask: (iz, it, ix, iy, ig, valid)."""
        params = self.params
        iz = np.floor((p_z - params.z_min) / params.cell).astype(np.intp)
        it = np.minimum((theta / np.pi * params.n_theta).astype(np.intp), params.n_theta - 1)
        ix = np.floor((x_star + params.r_xy) / params.cell).astype(np.intp)
        iy = np.floor((y_star + params.r_xy) / params.cell).astype(np.intp)
        ig = np.minimum(
            ((gamma + np.pi) / (2.0 * np.pi) * params.n_inplane).astype(np.intp),
            params.n_inplane - 1,
        )
        valid = (
            (iz >= 0)
            & (iz < params.n_z)
            & (ix >= 0)
            & (ix < params.n_xy)
            & (iy >= 0)
            & (iy < params.n_xy)
        )
        return iz, it, ix, iy, ig, valid

    def heading_bins(self, psi: NDArray[np.float64]) -> NDArray[np.uint8]:
        bins = np.minimum(
            ((psi + np.pi) / (2.0 * np.pi) * self.params.n_heading).astype(np.intp),
            self.params.n_heading - 1,
        )
        return (1 << bins.astype(np.uint8)).astype(np.uint8)

    # Construction

    def body_indices(
        self, positions: NDArray[np.float64]
    ) -> tuple[NDArray[np.intp], NDArray[np.intp], NDArray[np.intp], NDArray[np.bool_]]:
        """(iz, ix, iy, valid) for body-frame TCP positions."""
        params = self.params
        p = np.atleast_2d(np.asarray(positions, dtype=np.float64))
        iz = np.floor((p[:, 2] - params.z_min) / params.cell).astype(np.intp)
        ix = np.floor((p[:, 0] + params.r_xy) / params.cell).astype(np.intp)
        iy = np.floor((p[:, 1] + params.r_xy) / params.cell).astype(np.intp)
        valid = (
            (iz >= 0)
            & (iz < params.n_z)
            & (ix >= 0)
            & (ix < params.n_xy)
            & (iy >= 0)
            & (iy < params.n_xy)
        )
        return iz, ix, iy, valid

    def record_batch(self, positions: NDArray[np.float64], rotations: NDArray[np.float64]) -> int:
        """Mark a vectorized batch of reachable TCP pose components.

        This is the construction hot path: callers that already have arrays can
        avoid allocating pose objects. Use :meth:`record_poses` at API boundaries
        where EE/TCP poses are naturally represented as transforms.
        """
        p_z, theta, x_star, y_star, gamma, psi = canonical_values(positions, rotations)
        iz, it, ix, iy, ig, valid = self.indices(p_z, theta, x_star, y_star, gamma)
        iz, it_v, ix, iy, ig = (a[valid] for a in (iz, it, ix, iy, ig))

        # Saturating add (np.add.at on uint8 would wrap).
        flat = np.ravel_multi_index((iz, it_v, ix, iy, ig), self.counts.shape)
        unique, add = np.unique(flat, return_counts=True)
        current = self.counts.reshape(-1)[unique].astype(np.uint32)
        self.counts.reshape(-1)[unique] = np.minimum(current + add, _UINT8_MAX).astype(np.uint8)

        np.bitwise_or.at(self.heading_hint, (iz, it_v, ix, iy), self.heading_bins(psi[valid]))

        # Body-frame companions (construction frame == body frame).
        bz, bx, by, bvalid = self.body_indices(positions)
        bz, bx, by = bz[bvalid], bx[bvalid], by[bvalid]
        bflat = np.ravel_multi_index((bz, bx, by), self.body_counts.shape)
        bunique, badd = np.unique(bflat, return_counts=True)
        bcur = self.body_counts.reshape(-1)[bunique].astype(np.uint32)
        self.body_counts.reshape(-1)[bunique] = np.minimum(bcur + badd, _UINT8_MAX).astype(np.uint8)
        theta_bits = (np.uint64(1) << it[bvalid].astype(np.uint64)).astype(np.uint64)
        np.bitwise_or.at(self.body_theta_mask, (bz, bx, by), theta_bits)
        return int(valid.sum())

    def record_poses(self, ee_poses: PoseInput) -> int:
        """Mark reachable EE/TCP poses from homogeneous pose input.

        ``ee_poses`` may be a single :class:`PoseStamped`, one ``(4, 4)``
        transform, or a batch of ``(N, 4, 4)`` transforms.
        """
        return self.record_batch(*pose_arrays(ee_poses))

    def body_dexterity(self) -> NDArray[np.float64]:
        """Fraction of approach-angle bins observed per body-frame cell —
        Zacharias-style reachability index in [0, 1]."""
        return np.bitwise_count(self.body_theta_mask).astype(np.float64) / self.params.n_theta

    # Queries (heading-free: "reachable from here, possibly after turning")

    def scores(
        self, positions: NDArray[np.float64], rotations: NDArray[np.float64]
    ) -> NDArray[np.uint8]:
        """Scores for vectorized TCP pose components.

        This low-level form is kept for bulk construction/evaluation. Use
        :meth:`score_poses` or :meth:`score_pose` when the caller has EE/TCP
        poses as homogeneous transforms.
        """
        p_z, theta, x_star, y_star, gamma, _ = canonical_values(positions, rotations)
        iz, it, ix, iy, ig, valid = self.indices(p_z, theta, x_star, y_star, gamma)
        out = np.zeros(len(iz), dtype=np.uint8)
        v = valid
        out[v] = self.counts[iz[v], it[v], ix[v], iy[v], ig[v]]
        return out

    def score_poses(self, ee_poses: PoseInput) -> NDArray[np.uint8]:
        """Scores for EE/TCP poses (0 = unreachable/unknown)."""
        return self.scores(*pose_arrays(ee_poses))

    def score_pose(self, ee_pose: PoseInput) -> int:
        """Score one EE/TCP pose."""
        return int(self.score_poses(ee_pose)[0])

    def scores_4d(
        self, positions: NDArray[np.float64], rotations: NDArray[np.float64]
    ) -> NDArray[np.uint8]:
        """Scores from the gamma-marginal (RM4D-style 4D map: max over in-plane)."""
        p_z, theta, x_star, y_star, gamma, _ = canonical_values(positions, rotations)
        iz, it, ix, iy, _, valid = self.indices(p_z, theta, x_star, y_star, gamma)
        out = np.zeros(len(iz), dtype=np.uint8)
        v = valid
        out[v] = self.counts[iz[v], it[v], ix[v], iy[v], :].max(axis=-1)
        return out

    def score_poses_4d(self, ee_poses: PoseInput) -> NDArray[np.uint8]:
        """Gamma-marginal scores for EE/TCP poses."""
        return self.scores_4d(*pose_arrays(ee_poses))

    def reachable_pose(self, ee_pose: PoseInput, min_count: int = 1) -> bool:
        """Single EE/TCP pose → heading-free reachability."""
        return self.score_pose(ee_pose) >= min_count

    def reachable(self, pose: PoseInput, min_count: int = 1) -> bool:
        """Backward-compatible alias for :meth:`reachable_pose`."""
        return self.reachable_pose(pose, min_count=min_count)

    def position_scores(self) -> NDArray[np.uint8]:
        """Max-over-orientation score on an (n_z, n_r) radial grid.

        Heading-free reachability of a *position* depends only on
        (radius, height): sweeping the quotiented azimuth sweeps (x*, y*)
        around a circle. Bins cells by canonical radius; radial bin width
        equals the cell size.
        """
        params = self.params
        centers = (np.arange(params.n_xy) + 0.5) * params.cell - params.r_xy
        radius = np.hypot(centers[:, None], centers[None, :])
        r_bins = np.minimum((radius / params.cell).astype(np.intp), params.n_xy - 1)

        best = self.counts.max(axis=(1, 4))  # (n_z, n_xy, n_xy): max over θ, gamma
        out = np.zeros((params.n_z, params.n_xy), dtype=np.uint8)
        flat_bins = r_bins.reshape(-1)
        flat_best = best.reshape(params.n_z, -1)
        for r in range(params.n_xy):
            mask = flat_bins == r
            if np.any(mask):
                out[:, r] = flat_best[:, mask].max(axis=1)
        return out

    def theta_band_position_scores(self, theta_lo: float, theta_hi: float) -> NDArray[np.uint8]:
        """Like :meth:`position_scores`, restricted to approach angles in a band."""
        params = self.params
        t_lo = int(np.clip(theta_lo / np.pi * params.n_theta, 0, params.n_theta - 1))
        t_hi = int(np.clip(theta_hi / np.pi * params.n_theta, t_lo + 1, params.n_theta))
        centers = (np.arange(params.n_xy) + 0.5) * params.cell - params.r_xy
        radius = np.hypot(centers[:, None], centers[None, :])
        r_bins = np.minimum((radius / params.cell).astype(np.intp), params.n_xy - 1)
        best = self.counts[:, t_lo:t_hi].max(axis=(1, 4))
        out = np.zeros((params.n_z, params.n_xy), dtype=np.uint8)
        flat_bins = r_bins.reshape(-1)
        flat_best = best.reshape(params.n_z, -1)
        for r in range(params.n_xy):
            mask = flat_bins == r
            if np.any(mask):
                out[:, r] = flat_best[:, mask].max(axis=1)
        return out

    # Persistence

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            counts=self.counts,
            heading_hint=self.heading_hint,
            body_counts=self.body_counts,
            body_theta_mask=self.body_theta_mask,
            params=np.frombuffer(json.dumps(self.params.to_json_dict()).encode(), dtype=np.uint8),
            meta=np.frombuffer(
                json.dumps({"robot": self.robot, "model_id": self.model_id}).encode(),
                dtype=np.uint8,
            ),
        )
        logger.info(f"Capability map saved: {path} ({path.stat().st_size / 1e6:.1f} MB)")
        return path

    @classmethod
    def load(cls, path: str | Path) -> CapabilityMap:
        data = np.load(Path(path))
        params = MapParams.from_json_dict(json.loads(bytes(data["params"]).decode()))
        meta = json.loads(bytes(data["meta"]).decode())
        robot = meta.get("robot", "")
        if not robot and meta.get("side") in {"left", "right"}:
            robot = f"g1-{meta['side']}"
        elif not robot:
            robot = "g1-left"
        return cls(
            params=params,
            robot=robot,
            model_id=meta.get("model_id", ""),
            counts=data["counts"],
            heading_hint=data["heading_hint"],
            # Absent in maps built before the body-frame companions existed.
            body_counts=data["body_counts"] if "body_counts" in data else None,
            body_theta_mask=data["body_theta_mask"] if "body_theta_mask" in data else None,
        )

    @property
    def n_marked(self) -> int:
        return int(np.count_nonzero(self.counts))

    def summary(self) -> dict[str, float]:
        total = self.counts.size
        marked = self.n_marked
        return {
            "cells": total,
            "marked": marked,
            "fill_ratio": marked / total,
            "max_count": int(self.counts.max(initial=0)),
        }


def model_id_for(path: str | Path) -> str:
    """Content hash tying a map to the exact robot model it was built from."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:12]


__all__ = [
    "CapabilityMap",
    "MapParams",
    "canonical_values",
    "canonical_values_from_poses",
    "model_id_for",
    "pose_arrays",
]
