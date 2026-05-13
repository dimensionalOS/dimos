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

"""Runner abstraction for the RTAB-Map algorithm.

The module wrapper (:mod:`dimos.navigation.nav_stack.modules.rtab_map.rtab_map`)
sends each `(scan, odom)` pair into a :class:`RtabRunner` implementation.
Two implementations live here:

- :class:`LiteRtabRunner` — a pure-Python, deterministic implementation of the
  small subset of RTAB-Map behavior this module relies on: occupancy-grid voxel
  insertion with 3D ray-tracing clearing, ground/obstacle segmentation by
  surface-normal angle, keyframe-based loop closure detection on revisits, and
  per-keyframe drift correction. It is **not** a replacement for real
  RTAB-Map; it exists so the validation tests can exercise behavior without a
  C++ binary in the loop.

- :class:`SubprocessRtabRunner` — a skeleton that would shell out to a system
  `rtabmap` install. Left as a follow-up; raises if instantiated.

The seam is a :class:`typing.Protocol` so the module wrapper has no compile-time
dependency on either implementation, and tests can inject any duck-typed fake.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

import numpy as np

GROUND_LABEL = 1
OBSTACLE_LABEL = 2

# Maximum spatial gap, in meters, that the keyframe revisit detector considers
# "back near a prior keyframe." Anything closer counts as a candidate for loop
# closure. Larger means more candidates (more false positives); smaller means
# tighter trajectories required to close a loop.
_DEFAULT_REVISIT_RADIUS_M = 2.0

# A keyframe's normalized scan-match fitness must exceed this to count as a
# closed loop. Range [0, 1]; higher = stricter. Mirrors the spirit of
# rtabmap's Reg/MinFitness / Vis/MinInliers thresholds.
_DEFAULT_LOOP_FITNESS_THRESHOLD = 0.6


@dataclass
class RunnerStepResult:
    """Outputs produced by one runner step.

    Attributes:
        corrected_pose: 4x4 SE(3) pose in the `map` frame after applying the
            runner's accumulated correction to the incoming odom pose.
        tf_correction: 4x4 SE(3) transform expressing `map -> odom`. Equal to
            identity until the first loop closure shifts the map frame.
        octomap_voxels: (N, 3) float array of occupied voxel centroids in the
            map frame. Empty/cleared voxels are not represented.
        projected_2d_voxels: (M, 3) float array of the 2D projection of the
            occupied voxels (z held at `floor_z`). Equivalent to RTAB-Map's
            `octomap.createProjectionMap()` output.
        global_map_points: (K, 3) float array of accumulated scan points in
            the map frame; the equivalent of PGO's `global_map` output.
        loop_closure_event: True iff this step produced a new loop closure
            (i.e. matched a prior keyframe with sufficient fitness).
    """

    corrected_pose: np.ndarray
    tf_correction: np.ndarray
    octomap_voxels: np.ndarray
    projected_2d_voxels: np.ndarray
    global_map_points: np.ndarray
    loop_closure_event: bool


@runtime_checkable
class RtabRunner(Protocol):
    """Protocol satisfied by both the lite runner and any real RTAB-Map bridge.

    Stable across implementations so the module wrapper can swap them without
    code changes.
    """

    def process(
        self,
        scan_points_body: np.ndarray,
        odom_pose: np.ndarray,
        timestamp: float,
    ) -> RunnerStepResult:
        """Consume one (scan, odom) tuple, advance internal state, emit result.

        Args:
            scan_points_body: (N, 3) float array of scan points in the body
                frame.
            odom_pose: 4x4 SE(3) pose of body in the odom frame as reported
                by FastLIO2.
            timestamp: wall-clock seconds for this measurement.
        """
        ...


# ---------------------------------------------------------------------------
# Lite runner
# ---------------------------------------------------------------------------


@dataclass
class _Keyframe:
    pose_local: np.ndarray  # body -> odom (input)
    pose_global: np.ndarray  # body -> map (after correction)
    body_points: np.ndarray  # (N, 3)
    timestamp: float


@dataclass
class _Voxel:
    """Occupancy log-odds with hit/miss counters for diagnostic clarity."""

    log_odds: float = 0.0
    hits: int = 0
    misses: int = 0


@dataclass
class LiteRtabRunnerConfig:
    """Knobs for the lite runner. Defaults align with the user-spec defaults."""

    cell_size: float = 0.1
    ray_tracing: bool = True
    max_ground_angle_deg: float = 45.0
    ground_is_obstacle: bool = False
    log_odds_hit: float = 0.85
    log_odds_miss: float = -0.4
    occupied_threshold: float = 0.4
    keyframe_delta_trans: float = 0.5
    keyframe_delta_rad: float = np.deg2rad(10.0)
    revisit_radius_m: float = _DEFAULT_REVISIT_RADIUS_M
    loop_fitness_threshold: float = _DEFAULT_LOOP_FITNESS_THRESHOLD
    # Minimum elapsed time, in seconds, between the candidate keyframe and
    # the current one — guards against treating recently-passed keyframes as
    # revisits. Mirrors rtabmap's `RGBD/LoopClosureReextractFeatures` time
    # window logic.
    min_loop_detect_duration_s: float = 5.0
    floor_z: float = 0.0
    ground_neighbor_count: int = 8


class LiteRtabRunner:
    """Pure-Python stand-in for RTAB-Map.

    Implements just enough RTAB-Map-flavored behavior to validate the wrapper
    end-to-end without an external binary:

    * incremental 3D occupancy grid with hit log-odds + 3D DDA ray-tracing
      misses from the sensor origin to each occupied voxel (drives the
      "raycasting point clearing" behavior the user asked for as a default);
    * surface-normal-based ground vs. obstacle segmentation, controlled by
      ``max_ground_angle_deg`` (mirrors rtabmap's ``Grid/MaxGroundAngle``);
    * keyframe selection on translation/rotation thresholds and loop closure
      via geometric scan matching when the robot is back inside
      ``revisit_radius_m`` of a prior keyframe;
    * per-step ``map -> odom`` correction that is the identity until a loop
      closure rewires the global pose, after which it absorbs the accumulated
      odom drift.
    """

    def __init__(self, config: LiteRtabRunnerConfig | None = None) -> None:
        self._config = config or LiteRtabRunnerConfig()
        self._voxels: dict[tuple[int, int, int], _Voxel] = {}
        self._keyframes: list[_Keyframe] = []
        self._correction: np.ndarray = np.eye(4)  # map <- odom
        self._global_points: list[np.ndarray] = []
        self._loop_pairs: list[tuple[int, int]] = []

    # ------------- public API (RtabRunner Protocol) -------------

    def process(
        self,
        scan_points_body: np.ndarray,
        odom_pose: np.ndarray,
        timestamp: float,
    ) -> RunnerStepResult:
        self._validate_inputs(scan_points_body, odom_pose)

        corrected_pose = self._correction @ odom_pose
        scan_map = _transform_points(scan_points_body, corrected_pose)

        ground_mask = self.classify_ground(
            scan_points_body, max_ground_angle_deg=self._config.max_ground_angle_deg
        )

        labels = self._labels_from_ground_mask(ground_mask)
        self._update_octomap(corrected_pose, scan_map, labels)
        self._global_points.append(scan_map)

        loop_event = self._maybe_add_keyframe_and_close_loop(
            scan_points_body, odom_pose, corrected_pose, timestamp
        )

        # After a loop closure the correction may have changed; recompute
        # corrected pose so the emitted result reflects the new alignment.
        corrected_after = self._correction @ odom_pose

        return RunnerStepResult(
            corrected_pose=corrected_after,
            tf_correction=self._correction.copy(),
            octomap_voxels=self.occupied_voxel_centroids(),
            projected_2d_voxels=self.projected_2d_voxels(),
            global_map_points=self._stack_global_points(),
            loop_closure_event=loop_event,
        )

    # ------------- algorithmic primitives (also used by tests) -------------

    def classify_ground(
        self,
        points_body: np.ndarray,
        *,
        max_ground_angle_deg: float,
    ) -> np.ndarray:
        """Return a bool mask of length N: True where point is ground.

        Ground is a point whose locally-estimated surface normal is within
        ``max_ground_angle_deg`` of +Z. Normals are estimated by PCA over the
        nearest ``ground_neighbor_count`` neighbors. Mirrors RTAB-Map's
        ``Grid/MaxGroundAngle`` semantics.

        Points with too few neighbors for a stable PCA fall back to a coarse
        z-axis-aligned heuristic (treat as ground if locally near the lowest
        z in the cloud) — sufficient for synthetic tests.
        """
        if len(points_body) == 0:
            return np.zeros(0, dtype=bool)

        cos_threshold = float(np.cos(np.deg2rad(max_ground_angle_deg)))
        neighbor_count = min(self._config.ground_neighbor_count, len(points_body))

        if neighbor_count < 3:
            # Not enough points for a real PCA; treat anything near the
            # minimum z as ground.
            z = points_body[:, 2]
            return np.asarray(z <= np.min(z) + self._config.cell_size, dtype=bool)

        # Brute-force kNN — synthetic test clouds are tiny.
        diffs = points_body[:, None, :] - points_body[None, :, :]
        sq_dists = np.einsum("ijk,ijk->ij", diffs, diffs)
        order = np.argsort(sq_dists, axis=1)[:, :neighbor_count]

        mask = np.zeros(len(points_body), dtype=bool)
        for i, neighbors in enumerate(order):
            neighborhood = points_body[neighbors]
            centered = neighborhood - neighborhood.mean(axis=0)
            cov = centered.T @ centered
            _, eigvecs = np.linalg.eigh(cov)
            normal = eigvecs[:, 0]  # smallest eigenvalue's eigenvector
            normal = normal / (np.linalg.norm(normal) + 1e-12)
            # Normal may point either way; take absolute z-component.
            mask[i] = abs(float(normal[2])) >= cos_threshold

        return mask

    def occupied_voxel_centroids(self) -> np.ndarray:
        """(N, 3) centroids of currently-occupied voxels (above threshold)."""
        cells = [
            key
            for key, voxel in self._voxels.items()
            if voxel.log_odds >= self._config.occupied_threshold
        ]
        if not cells:
            return np.zeros((0, 3))
        arr = np.array(cells, dtype=np.float64)
        return (arr + 0.5) * self._config.cell_size

    def projected_2d_voxels(self) -> np.ndarray:
        """Projection of the 3D OctoMap to a 2D occupancy footprint.

        Mirrors RTAB-Map's ``octomap.createProjectionMap()``: every (x, y) with
        any occupied voxel above it shows up once, with z held at ``floor_z``.
        """
        occupied = self.occupied_voxel_centroids()
        if len(occupied) == 0:
            return np.zeros((0, 3))
        xy = np.unique(np.round(occupied[:, :2] / self._config.cell_size).astype(int), axis=0)
        xs = (xy[:, 0] + 0.5) * self._config.cell_size
        ys = (xy[:, 1] + 0.5) * self._config.cell_size
        zs = np.full_like(xs, fill_value=self._config.floor_z)
        return np.stack([xs, ys, zs], axis=1)

    @property
    def correction(self) -> np.ndarray:
        return self._correction.copy()

    @property
    def keyframe_count(self) -> int:
        return len(self._keyframes)

    @property
    def loop_closure_pairs(self) -> list[tuple[int, int]]:
        return list(self._loop_pairs)

    # ------------- internals -------------

    def _validate_inputs(self, scan: np.ndarray, odom_pose: np.ndarray) -> None:
        if scan.ndim != 2 or scan.shape[1] != 3:
            raise ValueError(f"scan_points_body must be (N, 3), got shape={scan.shape}")
        if odom_pose.shape != (4, 4):
            raise ValueError(f"odom_pose must be (4, 4), got shape={odom_pose.shape}")

    def _labels_from_ground_mask(self, ground_mask: np.ndarray) -> np.ndarray:
        labels = np.full(len(ground_mask), OBSTACLE_LABEL, dtype=np.int32)
        if not self._config.ground_is_obstacle:
            labels[ground_mask] = GROUND_LABEL
        return labels

    def _update_octomap(
        self,
        sensor_pose_map: np.ndarray,
        points_map: np.ndarray,
        labels: np.ndarray,
    ) -> None:
        origin = sensor_pose_map[:3, 3]
        origin_cell = self._cell_of(origin)

        for point, label in zip(points_map, labels, strict=True):
            target_cell = self._cell_of(point)

            if self._config.ray_tracing:
                for cell in _voxel_traversal(origin_cell, target_cell):
                    if cell == target_cell:
                        break
                    self._mark_free(cell)

            if label == OBSTACLE_LABEL:
                self._mark_occupied(target_cell)
            else:
                self._mark_free(target_cell)

    def _maybe_add_keyframe_and_close_loop(
        self,
        scan_body: np.ndarray,
        odom_pose: np.ndarray,
        corrected_pose: np.ndarray,
        timestamp: float,
    ) -> bool:
        if not self._is_new_keyframe(odom_pose):
            return False

        new_kf = _Keyframe(
            pose_local=odom_pose.copy(),
            pose_global=corrected_pose.copy(),
            body_points=scan_body.copy(),
            timestamp=timestamp,
        )
        self._keyframes.append(new_kf)
        new_idx = len(self._keyframes) - 1

        candidate_idx = self._find_revisit_candidate(new_kf)
        if candidate_idx is None:
            return False

        candidate = self._keyframes[candidate_idx]
        fitness, delta = self._scan_match(new_kf, candidate)
        if fitness < self._config.loop_fitness_threshold:
            return False

        # Apply correction: align the new keyframe's global pose with the
        # candidate keyframe's global pose modulo the recovered delta.
        target_global = candidate.pose_global @ delta
        delta_correction = target_global @ np.linalg.inv(corrected_pose)
        self._correction = delta_correction @ self._correction

        # Update the new keyframe's global pose to reflect the correction.
        new_kf.pose_global = self._correction @ new_kf.pose_local
        self._loop_pairs.append((candidate_idx, new_idx))
        return True

    def _is_new_keyframe(self, odom_pose: np.ndarray) -> bool:
        if not self._keyframes:
            return True
        last = self._keyframes[-1].pose_local
        delta_t = float(np.linalg.norm(odom_pose[:3, 3] - last[:3, 3]))
        rel_rot = last[:3, :3].T @ odom_pose[:3, :3]
        cos_angle = float(np.clip((np.trace(rel_rot) - 1.0) * 0.5, -1.0, 1.0))
        delta_r = float(np.arccos(cos_angle))
        return (
            delta_t >= self._config.keyframe_delta_trans
            or delta_r >= self._config.keyframe_delta_rad
        )

    def _find_revisit_candidate(self, new_kf: _Keyframe) -> int | None:
        if len(self._keyframes) < 2:
            return None
        new_xy = new_kf.pose_global[:2, 3]
        best_idx: int | None = None
        best_dist = float("inf")
        # Only consider keyframes that aren't the immediate prior AND are
        # older than ``min_loop_detect_duration_s``. The time gate guards
        # against the revisit detector firing on adjacent keyframes that
        # happen to be inside the spatial radius — that's a noise source,
        # not a real loop closure.
        for idx in range(len(self._keyframes) - 2):
            other = self._keyframes[idx]
            elapsed = new_kf.timestamp - other.timestamp
            if elapsed < self._config.min_loop_detect_duration_s:
                continue
            distance = float(np.linalg.norm(other.pose_global[:2, 3] - new_xy))
            if distance < best_dist:
                best_dist = distance
                best_idx = idx
        if best_idx is None or best_dist > self._config.revisit_radius_m:
            return None
        return best_idx

    def _scan_match(
        self,
        new_kf: _Keyframe,
        candidate: _Keyframe,
    ) -> tuple[float, np.ndarray]:
        """Approximate ICP between two keyframes. Returns (fitness, delta).

        ``delta`` is the SE(3) transform mapping the candidate's body frame to
        the new keyframe's body frame at the matched alignment.

        The lite runner uses a single-pass nearest-neighbor match in the
        body frame (no rotation search). It assumes scans of the same scene
        observed from nearby revisit poses look approximately identical in
        the body frame, with residual translation absorbed by a brute-force
        sweep across discrete offsets — enough for synthetic validation.
        Real ICP belongs in the production rtabmap binary.
        """
        new_pts = new_kf.body_points
        cand_pts = candidate.body_points
        if len(new_pts) == 0 or len(cand_pts) == 0:
            return 0.0, np.eye(4)

        cell = self._config.cell_size
        threshold = (2.0 * cell) ** 2  # within ~two voxels counts as a match

        best_fitness = 0.0
        best_offset = np.zeros(3)
        # Search a small grid of x/y offsets to absorb residual translation
        # between the two body frames. The grid stays inside one cell so the
        # candidate is locally consistent; rotation is ignored.
        offsets = [np.array([dx * cell, dy * cell, 0.0]) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
        for offset in offsets:
            moved = new_pts + offset
            diffs = moved[:, None, :] - cand_pts[None, :, :]
            sq_dists = np.einsum("ijk,ijk->ij", diffs, diffs)
            nearest_sq = sq_dists.min(axis=1)
            fraction = float((nearest_sq < threshold).mean())
            if fraction > best_fitness:
                best_fitness = fraction
                best_offset = offset

        delta = np.eye(4)
        delta[:3, 3] = best_offset
        return best_fitness, delta

    def _stack_global_points(self) -> np.ndarray:
        if not self._global_points:
            return np.zeros((0, 3))
        return np.concatenate(self._global_points, axis=0)

    def _cell_of(self, point: np.ndarray) -> tuple[int, int, int]:
        idx = np.floor(point / self._config.cell_size).astype(int)
        return int(idx[0]), int(idx[1]), int(idx[2])

    def _mark_occupied(self, cell: tuple[int, int, int]) -> None:
        voxel = self._voxels.setdefault(cell, _Voxel())
        voxel.log_odds += self._config.log_odds_hit
        voxel.hits += 1

    def _mark_free(self, cell: tuple[int, int, int]) -> None:
        voxel = self._voxels.setdefault(cell, _Voxel())
        voxel.log_odds += self._config.log_odds_miss
        voxel.misses += 1


class SubprocessRtabRunner:
    """Skeleton for shelling out to a system `rtabmap` install.

    Not implemented in this commit; tracked as follow-up. The wrapper module
    type-checks against the :class:`RtabRunner` protocol, so when this class
    is fleshed out the module needs no edits to consume it.
    """

    def __init__(self, **kwargs: Any) -> None:
        del kwargs
        raise NotImplementedError(
            "SubprocessRtabRunner is a placeholder; the system-rtabmap "
            "subprocess bridge is a follow-up. Use LiteRtabRunner for "
            "validation and integration testing until then."
        )

    def process(
        self,
        scan_points_body: np.ndarray,
        odom_pose: np.ndarray,
        timestamp: float,
    ) -> RunnerStepResult:
        del scan_points_body, odom_pose, timestamp
        raise NotImplementedError


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _transform_points(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return points
    rotated = points @ pose[:3, :3].T
    return np.asarray(rotated + pose[:3, 3])


def _voxel_traversal(
    start: tuple[int, int, int],
    end: tuple[int, int, int],
) -> list[tuple[int, int, int]]:
    """3D Bresenham/DDA-style voxel traversal from `start` to `end` inclusive.

    Returns the ordered sequence of integer voxel coordinates the line passes
    through. Used by the occupancy grid to mark intermediate voxels as free
    along each scan ray (the OctoMap raycasting clearing behavior).
    """
    x0, y0, z0 = start
    x1, y1, z1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    dz = abs(z1 - z0)
    sx = 1 if x1 > x0 else -1 if x1 < x0 else 0
    sy = 1 if y1 > y0 else -1 if y1 < y0 else 0
    sz = 1 if z1 > z0 else -1 if z1 < z0 else 0

    cells: list[tuple[int, int, int]] = [(x0, y0, z0)]
    x, y, z = x0, y0, z0

    if dx >= dy and dx >= dz:
        err_1 = 2 * dy - dx
        err_2 = 2 * dz - dx
        while x != x1:
            if err_1 > 0:
                y += sy
                err_1 -= 2 * dx
            if err_2 > 0:
                z += sz
                err_2 -= 2 * dx
            err_1 += 2 * dy
            err_2 += 2 * dz
            x += sx
            cells.append((x, y, z))
    elif dy >= dx and dy >= dz:
        err_1 = 2 * dx - dy
        err_2 = 2 * dz - dy
        while y != y1:
            if err_1 > 0:
                x += sx
                err_1 -= 2 * dy
            if err_2 > 0:
                z += sz
                err_2 -= 2 * dy
            err_1 += 2 * dx
            err_2 += 2 * dz
            y += sy
            cells.append((x, y, z))
    else:
        err_1 = 2 * dy - dz
        err_2 = 2 * dx - dz
        while z != z1:
            if err_1 > 0:
                y += sy
                err_1 -= 2 * dz
            if err_2 > 0:
                x += sx
                err_2 -= 2 * dz
            err_1 += 2 * dy
            err_2 += 2 * dx
            z += sz
            cells.append((x, y, z))

    return cells


__all__ = [
    "GROUND_LABEL",
    "OBSTACLE_LABEL",
    "LiteRtabRunner",
    "LiteRtabRunnerConfig",
    "RtabRunner",
    "RunnerStepResult",
    "SubprocessRtabRunner",
]
