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

"""Query-time scene inventory: prompt-free discovery plus geometric dedup.

``inventory()`` answers "what instances are on the table" for a time window,
computed at query time over the recording - no ingest pass, no persisted
instance table. Existence is decoupled from naming and the ordering is a
constraint, not a preference: propose (EdgeTAM automatic masks), lift
(masked depth to world supports), associate (hard constraints before any
score), and only then name (OWLv2, labels as metadata). Labels and
appearance never enter association; position and same-frame co-occurrence
decide everything, which is what keeps two identical objects two instances.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory2.tf import StreamTF
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.memory import gates
from dimos.perception.memory.gates import (
    MOTION_THRESHOLD,
    OPTICAL_FRAME,
    TF_TOLERANCE,
    WORLD_FRAME,
)
from dimos.perception.memory.support_plane import SupportPlane, fit_support_plane
from dimos.perception.memory.types import (
    Instance,
    InventoryPolicy,
    Support,
    SupportObservation,
    aabb_overlap,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
    from dimos.protocol.tf.tf import TFLookup

logger = setup_logger()

KEYFRAME_STRIDE = 2.5  # s - proposal keyframe grid
MAX_PROPOSALS_PER_FRAME = 40
NAME_FRAMES_PER_INSTANCE = 5
NAME_SCORE_FLOOR = 0.18
# An attachment must be the detector drawing a box around this member, not a
# box that merely crosses it.
NAME_ATTACH_IOU = 0.45
# Post-processing reports one label per box per request, so chunking is what
# lets a box carry more than one label. It changes no score.
NAME_PROMPT_CAP = 10
SUPPRESS_SCORE = 0.25
SUPPRESS_OVERLAP = 0.35
UNGROUNDED_TRACK_IOU = 0.40

# Naming vocabulary: a generic list of common tabletop and household objects,
# batched into one capped OWLv2 prompt. The vocabulary is a naming-pass
# concern only - existence is geometric and an instance that matches nothing
# here keeps its unknown-N name with score 0.
GENERIC_VOCABULARY = [
    "pen",
    "pencil",
    "marker",
    "highlighter",
    "eraser",
    "book",
    "notebook",
    "sticky notes",
    "sheet of paper",
    "roll of tape",
    "scissors",
    "stapler",
    "ruler",
    "laptop",
    "computer keyboard",
    "computer mouse",
    "mobile phone",
    "cup",
    "bottle",
    "drink can",
    "bowl",
    "cardboard box",
    "cable",
    "remote control",
    "glasses",
    "headphones",
    "wallet",
    "toy block",
]
SUPPRESS_QUERIES = ["person", "human hand", "human arm"]


@dataclass
class _Track:
    members: list[SupportObservation] = field(default_factory=list)
    frame_ts: set[float] = field(default_factory=set)
    labels: dict[str, float] = field(default_factory=dict)

    def add(self, obs: SupportObservation, frame_key: float) -> None:
        self.members.append(obs)
        self.frame_ts.add(frame_key)

    @property
    def centroid(self) -> np.ndarray:
        median: np.ndarray = np.median(np.stack([m.centroid for m in self.members]), axis=0)
        return median

    @property
    def aabb(self) -> tuple[np.ndarray, np.ndarray]:
        lo = np.median(np.stack([m.aabb_min for m in self.members]), axis=0)
        hi = np.median(np.stack([m.aabb_max for m in self.members]), axis=0)
        return lo, hi

    @property
    def latest(self) -> SupportObservation:
        return max(self.members, key=lambda m: m.ts)


@dataclass
class _Track2D:
    """Ungrounded track: RGB detections with no valid depth, 2D identity only."""

    members: list[Detection2DSeg] = field(default_factory=list)
    frame_ts: set[float] = field(default_factory=set)
    labels: dict[str, float] = field(default_factory=dict)


def _bbox_iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = ix * iy
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / union if union > 0 else 0.0


def _proposal_passes_2d(det: Detection2DSeg, image_area: float, policy: InventoryPolicy) -> bool:
    area = float((det.mask > 0).sum())
    if area < policy.min_mask_area_px:
        return False
    if area > policy.max_mask_area_fraction * image_area:
        return False
    return True


# A lifted cloud plainly spanning more than one object: wider than any single
# tabletop object here, or reaching table-to-well-above-hand height.
SPLIT_EXTENT_M = 0.30
SPLIT_HEIGHT_M = 0.10
SPLIT_EPS_M = 0.03


def _split_oversized(
    points: np.ndarray, plane: SupportPlane | None, policy: InventoryPolicy
) -> list[np.ndarray]:
    """Re-segment a mask-bled cloud by 3D connectivity.

    Automatic masks occasionally bleed across an object onto the table and
    its neighbors; the lifted cloud then violates single-object bounds. The
    repair is geometric: strip the support-surface points, then split by
    spatial connectivity - distinct objects on this rig are separated by
    more than the cluster gap, one object's surface is not.
    """
    extent = points.max(axis=0) - points.min(axis=0)
    if float(extent.max()) <= SPLIT_EXTENT_M and float(extent[2]) <= SPLIT_HEIGHT_M:
        return [points]
    if plane is None:
        return [points]
    heights = plane.height_above(points)
    if float((np.abs(heights) <= 0.003).mean()) < 0.15:
        # No appreciable support-surface content: this is one oversized body,
        # not a mask that bled across the table. Leave it to the extent cap.
        return [points]

    above = heights > 0.002
    body = points[above] if above.sum() >= policy.min_depth_points else points

    import open3d as o3d

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(body)
    labels = np.asarray(cloud.cluster_dbscan(eps=SPLIT_EPS_M, min_points=20))
    clusters = [
        body[labels == label]
        for label in range(labels.max() + 1)
        if (labels == label).sum() >= policy.min_depth_points
    ]
    return clusters if clusters else [body]


def _pixel_bbox(
    points: np.ndarray, camera_info: CameraInfo, transform: Any
) -> tuple[float, float, float, float]:
    """Project world points back into the frame for a sub-observation's bbox."""
    matrix = transform.to_matrix()
    optical = (matrix[:3, :3] @ points.T).T + matrix[:3, 3]
    z = np.maximum(optical[:, 2], 1e-6)
    fx, fy = camera_info.K[0], camera_info.K[4]
    cx, cy = camera_info.K[2], camera_info.K[5]
    u = fx * optical[:, 0] / z + cx
    v = fy * optical[:, 1] / z + cy
    return (float(u.min()), float(v.min()), float(u.max()), float(v.max()))


def _lift_frame(
    detections_2d: Any,
    store: Any,
    tf: TFLookup,
    camera_info: CameraInfo,
    obs_ts: float,
    camera_position: np.ndarray,
    policy: InventoryPolicy,
    optical_frame: str,
    world_frame: str,
    tf_tolerance: float,
    plane: SupportPlane | None = None,
) -> tuple[list[SupportObservation], list[Detection2DSeg]]:
    """Depth-lift accepted proposals of one frame; returns (grounded, ungrounded)."""
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    depth = gates.depth_at(store, obs_ts)
    transform = tf.get(optical_frame, world_frame, obs_ts, tf_tolerance)
    if depth is None or transform is None:
        return [], list(detections_2d)

    lifted = ImageDetections3DPC.from_depth(detections_2d, depth, camera_info, transform)

    grounded: list[SupportObservation] = []
    ungrounded: list[Detection2DSeg] = []
    lifted_by_track = {det3d.track_id: det3d for det3d in lifted}
    for det2d in detections_2d:
        det3d = lifted_by_track.get(det2d.track_id)
        if det3d is None or len(det3d.pointcloud) < policy.min_depth_points:
            ungrounded.append(det2d)
            continue
        points = np.asarray(det3d.pointcloud.pointcloud.points)
        mask_area = int((det2d.mask > 0).sum())
        for piece in _split_oversized(points, plane, policy):
            aabb_min, aabb_max = piece.min(axis=0), piece.max(axis=0)
            extent = aabb_max - aabb_min
            if float(extent.max()) > policy.max_object_extent_m:
                continue
            ranges = np.linalg.norm(piece - camera_position, axis=1)
            if float(np.median(ranges)) < policy.min_camera_range_m:
                continue
            whole = len(piece) == len(points)
            grounded.append(
                SupportObservation(
                    ts=obs_ts,
                    cloud=det3d.pointcloud
                    if whole
                    else PointCloud2.from_numpy(piece, frame_id="world", timestamp=obs_ts),
                    centroid=piece.mean(axis=0),
                    aabb_min=aabb_min,
                    aabb_max=aabb_max,
                    n_points=len(piece),
                    mask_area_px=mask_area if whole else int(mask_area * len(piece) / len(points)),
                    camera_position=camera_position,
                    bbox=det2d.bbox if whole else _pixel_bbox(piece, camera_info, transform),
                )
            )
    return grounded, ungrounded


def _in_scope(obs: SupportObservation, plane: SupportPlane | None, policy: InventoryPolicy) -> bool:
    """Support-plane scope: in a band above the plane, footprint on the workspace."""
    if policy.in_scope is not None:
        points = np.asarray(obs.cloud.pointcloud.points)
        return bool(policy.in_scope(points))
    if plane is None:
        return True
    points = np.asarray(obs.cloud.pointcloud.points)
    heights = plane.height_above(points)
    low, high = float(np.quantile(heights, 0.05)), float(np.quantile(heights, 0.95))
    band_lo, band_hi = policy.band_above_plane_m
    if low < band_lo or high > band_hi:
        return False
    if not policy.include_surfaces and high < policy.min_height_above_plane_m:
        # A patch of the surface itself: no volume above the plane.
        return False
    inside = plane.footprint_contains(points[:, :2])
    return bool(inside.mean() >= 0.3)


def _aabb_gap(a: SupportObservation, b: SupportObservation) -> float:
    """Largest per-axis separation between two world AABBs (0 when touching)."""
    gap = np.maximum(a.aabb_min - b.aabb_max, b.aabb_min - a.aabb_max)
    return float(gap.max())


def _cloud_gap(a: SupportObservation, b: SupportObservation) -> float:
    """Minimum point-to-point distance between two observation clouds.

    The AABB gap is a poor contact test for diagonal objects - an
    axis-aligned box overhangs its object's true footprint and "touches"
    neighbors that are centimeters of clear table away. Actual cloud
    distance is the physical claim. The AABB test remains as a cheap
    prefilter.
    """
    if _aabb_gap(a, b) > 0.06:
        return np.inf
    from scipy.spatial import cKDTree

    pa = np.asarray(a.cloud.pointcloud.points)
    pb = np.asarray(b.cloud.pointcloud.points)
    pa = pa[:: max(1, len(pa) // 800)]
    pb = pb[:: max(1, len(pb) // 800)]
    distances, _ = cKDTree(pa).query(pb, k=1)
    return float(distances.min())


def _absorb_into(target: SupportObservation, obs: SupportObservation) -> None:
    target.aabb_min = np.minimum(target.aabb_min, obs.aabb_min)
    target.aabb_max = np.maximum(target.aabb_max, obs.aabb_max)
    target.cloud = target.cloud + obs.cloud
    points = np.asarray(target.cloud.pointcloud.points)
    target.centroid = points.mean(axis=0)
    target.n_points = len(points)
    target.mask_area_px = max(target.mask_area_px, obs.mask_area_px)


def _merge_same_frame(
    observations: list[SupportObservation], policy: InventoryPolicy
) -> list[SupportObservation]:
    """Fuse same-frame proposals that are one physical support.

    The criterion is contact: two same-frame observations whose clouds
    touch (minimum cloud distance within the gap) are one rigid body -
    duplicates, whole-and-part pairs, and split halves of one object all
    satisfy it, while distinct objects on the workspace, identical twins
    included, sit farther apart than the gap. Runs to a fixed point so
    chains of touching pieces collapse into one support.
    """
    if policy.include_object_parts:
        return observations
    items = sorted(observations, key=lambda o: -o.n_points)
    changed = True
    while changed:
        changed = False
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                if _cloud_gap(items[i], items[j]) <= policy.same_frame_merge_gap_m:
                    _absorb_into(items[i], items[j])
                    items.pop(j)
                    changed = True
                    break
            if changed:
                break
    return items


def _associate(
    frames: list[tuple[float, list[SupportObservation]]], policy: InventoryPolicy
) -> list[_Track]:
    """Hard constraints first, geometric score second, Hungarian for the residual.

    Per frame, observations assign one-to-one to existing tracks - the
    same-frame constraint is structural, no score overrides it. A pair is
    forbidden outright (infinite cost) when the supports are farther apart
    than the search radius, their envelopes do not overlap enough, or their
    sizes are incompatible beyond measurement error.
    """
    from scipy.optimize import linear_sum_assignment

    forbidden = 1e6
    tracks: list[_Track] = []
    for frame_key, observations in frames:
        if not observations:
            continue
        if not tracks:
            for obs in observations:
                track = _Track()
                track.add(obs, frame_key)
                tracks.append(track)
            continue

        cost = np.full((len(observations), len(tracks)), forbidden)
        for i, obs in enumerate(observations):
            for j, track in enumerate(tracks):
                distance = float(np.linalg.norm(obs.centroid - track.centroid))
                if distance > policy.search_radius_m:
                    continue
                t_lo, t_hi = track.aabb
                size_gap = np.abs((t_hi - t_lo) - (obs.aabb_max - obs.aabb_min))
                if float(size_gap.max()) > 0.25:
                    continue
                overlap = aabb_overlap(
                    obs.aabb_min, obs.aabb_max, t_lo, t_hi, pad=policy.envelope_pad_m
                )
                if overlap < policy.overlap_accept:
                    continue
                cost[i, j] = 1.0 - overlap

        rows, cols = linear_sum_assignment(cost)
        assigned: dict[int, int] = {}
        for i, j in zip(rows, cols, strict=False):
            if cost[i, j] < forbidden:
                assigned[i] = j
        for i, obs in enumerate(observations):
            match = assigned.get(i)
            if match is not None:
                tracks[match].add(obs, frame_key)
            else:
                track = _Track()
                track.add(obs, frame_key)
                tracks.append(track)
    return tracks


def _tracks_are_fragments(a: _Track, b: _Track, policy: InventoryPolicy) -> bool:
    """Co-observed tracks that were touching whenever seen together.

    The same-frame veto keeps coexisting objects apart, but a mask split can
    put two pieces of one object into the same frame with a cloud gap just
    over the per-frame merge threshold, locking a permanent duplicate. Two
    rigid objects cannot occupy one volume: when the supports interpenetrate
    at containment level and every shared frame shows the pair in contact,
    they are pieces of one body. Identical twins never satisfy this - their
    supports do not overlap at all.
    """
    a_lo, a_hi = a.aabb
    b_lo, b_hi = b.aabb
    overlap = aabb_overlap(a_lo, a_hi, b_lo, b_hi, pad=policy.envelope_pad_m)
    if overlap < 0.5:
        return False
    shared = a.frame_ts & b.frame_ts
    for ts in shared:
        pairs_gap = min(
            _cloud_gap(ma, mb) for ma in a.members if ma.ts == ts for mb in b.members if mb.ts == ts
        )
        if pairs_gap > 1.5 * policy.same_frame_merge_gap_m:
            return False
    return True


def _merge_tracks(tracks: list[_Track], policy: InventoryPolicy) -> list[_Track]:
    """Collapse fragmented tracks of one support.

    Tracks merge when they never share a frame (the same-frame veto at
    instance level) and their supports overlap within the envelope - or when
    they do share frames but were demonstrably pieces of one body in every
    one of them. Runs to a fixed point.
    """
    changed = True
    while changed:
        changed = False
        for i in range(len(tracks)):
            for j in range(i + 1, len(tracks)):
                a, b = tracks[i], tracks[j]
                if float(np.linalg.norm(a.centroid - b.centroid)) > policy.search_radius_m:
                    continue
                if a.frame_ts & b.frame_ts:
                    if not _tracks_are_fragments(a, b, policy):
                        continue
                else:
                    a_lo, a_hi = a.aabb
                    b_lo, b_hi = b.aabb
                    overlap = aabb_overlap(a_lo, a_hi, b_lo, b_hi, pad=policy.envelope_pad_m)
                    if overlap < policy.overlap_accept:
                        continue
                for obs in b.members:
                    a.add(obs, obs.ts)
                tracks.pop(j)
                changed = True
                break
            if changed:
                break
    return tracks


def _track_ungrounded(
    frames: list[tuple[float, list[Detection2DSeg]]],
) -> list[_Track2D]:
    """Greedy 2D IoU association for detections that never produced depth."""
    tracks: list[_Track2D] = []
    for frame_key, detections in frames:
        for det in detections:
            best, best_iou = None, UNGROUNDED_TRACK_IOU
            for track in tracks:
                if frame_key in track.frame_ts:
                    continue
                iou = _bbox_iou(det.bbox, track.members[-1].bbox)
                if iou > best_iou:
                    best, best_iou = track, iou
            if best is None:
                best = _Track2D()
                tracks.append(best)
            best.members.append(det)
            best.frame_ts.add(frame_key)
    return [t for t in tracks if len(t.members) >= 2]


def _view_coverage(members: list[SupportObservation]) -> tuple[float, tuple[bool, bool, bool]]:
    """Azimuth-bin coverage of the viewpoints and which world axes were observed."""
    if not members:
        return 0.0, (False, False, False)
    centroid = np.median(np.stack([m.centroid for m in members]), axis=0)
    directions = []
    for m in members:
        v = m.camera_position - centroid
        norm = np.linalg.norm(v)
        if norm > 1e-6:
            directions.append(v / norm)
    if not directions:
        return 0.0, (False, False, False)
    dirs = np.stack(directions)
    azimuth = np.arctan2(dirs[:, 1], dirs[:, 0])
    bins = set(((azimuth + np.pi) / (2 * np.pi) * 8).astype(int) % 8)
    coverage = len(bins) / 8.0
    observed = tuple(bool((np.abs(dirs[:, i]) > 0.3).any()) for i in range(3))
    return coverage, observed  # type: ignore[return-value]


def _build_instance(index: int, track: _Track, grounded: bool = True) -> Instance:
    labels = tuple(sorted(track.labels.items(), key=lambda kv: -kv[1]))
    primary = labels[0][0] if labels else None
    latest = track.latest
    coverage, axes_observed = _view_coverage(track.members)
    lo, hi = track.aabb
    center = (lo + hi) / 2
    extent = np.maximum(hi - lo, 0.005)
    centroids = np.stack([m.centroid for m in track.members])
    sigma = centroids.std(axis=0) if len(track.members) > 1 else np.full(3, 0.01)
    support = Support(
        center_xyz=(float(center[0]), float(center[1]), float(center[2])),
        extent_xyz_m=(float(extent[0]), float(extent[1]), float(extent[2])),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        sigma_xyz_m=(float(sigma[0]), float(sigma[1]), float(sigma[2])),
        coverage=coverage,
        axes_observed=axes_observed,
        frame_id="world",
    )
    distinct_views = len({tuple(np.round(m.camera_position, 2)) for m in track.members})
    return Instance(
        instance_id=f"obj-{index:02d}",
        grounded=grounded,
        primary_label=primary,
        labels=labels,
        state="active",
        identity_confidence=min(1.0, distinct_views / 3.0),
        support=support,
        latest_position_xyz=(
            float(latest.centroid[0]),
            float(latest.centroid[1]),
            float(latest.centroid[2]),
        ),
        latest_seen_ts=latest.ts,
        members=track.members,
    )


def _naming_picks(track: _Track) -> list[SupportObservation]:
    """Members to name on: the largest view, then maximal viewpoint spread.

    Picking by mask area alone selects near-duplicate views when the sweep
    keeps returning to one vantage; the label then hinges on a single
    viewing angle. Greedy farthest-point selection over camera positions
    guarantees the close-up passes participate.
    """
    if len(track.members) <= NAME_FRAMES_PER_INSTANCE:
        return list(track.members)
    picks = [max(track.members, key=lambda m: m.mask_area_px)]
    remaining = [m for m in track.members if m is not picks[0]]
    while len(picks) < NAME_FRAMES_PER_INSTANCE and remaining:
        best = max(
            remaining,
            key=lambda m: min(
                float(np.linalg.norm(m.camera_position - p.camera_position)) for p in picks
            ),
        )
        picks.append(best)
        remaining.remove(best)
    return picks


def _name_and_suppress(
    tracks: list[_Track],
    tracks_2d: list[_Track2D],
    store: Any,
) -> None:
    """OWLv2 naming per instance on keyframes, person/hand suppressing observations.

    Runs after association by construction: association consumed unnamed
    supports, so per-view label instability cannot starve existence or split
    an instance. A naming failure degrades names, never counts.
    """
    from dimos.perception.detection.detectors.owlv2 import Owlv2Detector

    frame_members: dict[float, list[tuple[_Track, SupportObservation]]] = {}
    for track in tracks:
        for member in _naming_picks(track):
            frame_members.setdefault(member.ts, []).append((track, member))
    frame_members_2d: dict[float, list[tuple[_Track2D, Detection2DSeg]]] = {}
    for track2d in tracks_2d:
        for det in sorted(track2d.members, key=lambda d: -(d.mask > 0).sum())[:2]:
            frame_members_2d.setdefault(det.ts, []).append((track2d, det))

    if not frame_members and not frame_members_2d:
        return

    owl = Owlv2Detector()
    chunks = [
        GENERIC_VOCABULARY[i : i + NAME_PROMPT_CAP]
        for i in range(0, len(GENERIC_VOCABULARY), NAME_PROMPT_CAP)
    ]
    chunks.append(list(SUPPRESS_QUERIES))
    suppress_set = set(SUPPRESS_QUERIES)
    all_ts = sorted(set(frame_members) | set(frame_members_2d))
    logger.info(
        f"naming: OWLv2 over {len(all_ts)} keyframes, "
        f"{len(chunks)} prompts of <= {NAME_PROMPT_CAP} classes"
    )
    for ts in all_ts:
        try:
            image = store.streams.color_image.at(ts, 0.05).first().data
        except LookupError:
            continue
        detections = [
            det
            for chunk in chunks
            for det in owl.query_detections(image, chunk, threshold=NAME_SCORE_FLOOR)
        ]
        for det in detections:
            if det.name in suppress_set:
                if det.confidence < SUPPRESS_SCORE:
                    continue
                for track, member in frame_members.get(ts, []):
                    if member.bbox is None:
                        continue
                    inside = _mask_overlap_fraction_bbox(member.bbox, det.bbox)
                    if inside >= SUPPRESS_OVERLAP and member in track.members:
                        track.members.remove(member)
                continue
            best_target: Any = None
            best_iou = NAME_ATTACH_IOU
            for track, member in frame_members.get(ts, []):
                if member.bbox is None:
                    continue
                iou = _bbox_iou(member.bbox, det.bbox)
                if iou > best_iou:
                    best_target, best_iou = track, iou
            for track2d, det2d in frame_members_2d.get(ts, []):
                iou = _bbox_iou(det2d.bbox, det.bbox)
                if iou > best_iou:
                    best_target, best_iou = track2d, iou
            if best_target is not None:
                previous = best_target.labels.get(det.name, 0.0)
                best_target.labels[det.name] = max(previous, det.confidence)
    owl.stop()


def _mask_overlap_fraction_bbox(
    member_box: tuple[float, float, float, float], region: tuple[float, float, float, float]
) -> float:
    """Fraction of the member box inside the region box."""
    mx1, my1, mx2, my2 = member_box
    rx1, ry1, rx2, ry2 = region
    ix = max(0.0, min(mx2, rx2) - max(mx1, rx1))
    iy = max(0.0, min(my2, ry2) - max(my1, ry1))
    area = (mx2 - mx1) * (my2 - my1)
    return (ix * iy) / area if area > 0 else 0.0


def inventory(
    store: Any,
    *,
    after: float | None = None,
    before: float | None = None,
    include_ungrounded: bool = False,
    policy: InventoryPolicy | None = None,
    motion_threshold: float = MOTION_THRESHOLD,
    log_progress: bool = False,
    world_frame: str = WORLD_FRAME,
    optical_frame: str = OPTICAL_FRAME,
    tf_tolerance: float = TF_TOLERANCE,
) -> list[Instance]:
    """Deduplicated object instances for the window, computed at query time.

    Reports the scene as of the window's end: an instance's position and
    timestamp come from its latest member observation in this call. An
    object that moved between rest positions inside the window registers
    once per rest position; linking rest positions of one object is
    cross-time identity and out of scope here.

    ``log_progress`` enables per-keyframe discovery lines
    (``discovery: i/n  ts_offset=…  prop=…  scope=…  …s``). Off by default.
    """
    policy = policy or InventoryPolicy()
    tf = StreamTF.from_store(store)
    if tf is None:
        raise ValueError("recording has no tf stream")
    camera_info = store.streams.camera_info.first().data
    lo, hi = store.streams.color_image.get_time_range()
    t0 = after if after is not None else lo
    t1 = before if before is not None else hi
    logger.info(f"inventory window: {t0 - lo:.1f}s to {t1 - lo:.1f}s ({t1 - t0:.1f}s)")

    intervals = gates.still_intervals(tf, t0, t1, optical_frame, world_frame, tf_tolerance)
    gray: dict[float, Any] = {}

    keyframes = gates.keyframes(
        store,
        tf,
        t0,
        t1,
        KEYFRAME_STRIDE,
        intervals,
        gray,
        motion_threshold,
        optical_frame,
        world_frame,
        tf_tolerance,
    )
    logger.info(f"gates: {len(keyframes)} keyframes pass camera-still + scene-still")
    if not keyframes:
        return []

    plane = fit_support_plane(
        store, tf, camera_info, keyframes, optical_frame, world_frame, tf_tolerance
    )
    if plane is not None:
        logger.info(f"support plane: {plane.inlier_count} inliers")

    from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter

    segmenter = EdgeTAMImageSegmenter()
    frames_grounded: list[tuple[float, list[SupportObservation]]] = []
    frames_ungrounded: list[tuple[float, list[Detection2DSeg]]] = []
    image_area = float(camera_info.width * camera_info.height)

    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

    n_kf = len(keyframes)
    for i, obs in enumerate(keyframes, start=1):
        t_frame = perf_counter() if log_progress else 0.0
        proposals = segmenter.propose_all(obs.data)
        accepted = [det for det in proposals if _proposal_passes_2d(det, image_area, policy)]
        accepted = sorted(accepted, key=lambda d: -(d.mask > 0).sum())[:MAX_PROPOSALS_PER_FRAME]
        if not accepted:
            if log_progress:
                logger.info(
                    f"discovery: {i}/{n_kf}  ts_offset={obs.ts - lo:.1f}s  "
                    f"prop={len(proposals)}->0  scope=0  {perf_counter() - t_frame:.1f}s"
                )
            continue

        pose = gates.camera_pose(tf, obs.ts, optical_frame, world_frame, tf_tolerance)
        if pose is None:
            if log_progress:
                logger.info(
                    f"discovery: {i}/{n_kf}  ts_offset={obs.ts - lo:.1f}s  "
                    f"skip=no_pose  {perf_counter() - t_frame:.1f}s"
                )
            continue
        camera_position = np.array([pose.position.x, pose.position.y, pose.position.z])

        for j, det in enumerate(accepted):
            det.track_id = j
        grounded, ungrounded = _lift_frame(
            ImageDetections2D(obs.data, accepted),
            store,
            tf,
            camera_info,
            obs.ts,
            camera_position,
            policy,
            optical_frame,
            world_frame,
            tf_tolerance,
            plane,
        )
        grounded = [o for o in grounded if _in_scope(o, plane, policy)]
        grounded = _merge_same_frame(grounded, policy)
        frames_grounded.append((obs.ts, grounded))
        frames_ungrounded.append((obs.ts, ungrounded))
        if log_progress:
            logger.info(
                f"discovery: {i}/{n_kf}  ts_offset={obs.ts - lo:.1f}s  "
                f"prop={len(proposals)}->{len(accepted)}  scope={len(grounded)}  "
                f"{perf_counter() - t_frame:.1f}s"
            )

    del segmenter
    _free_accelerator()

    total = sum(len(g) for _, g in frames_grounded)
    logger.info(f"discovery: {total} in-scope supports across {len(frames_grounded)} keyframes")

    tracks = _associate(frames_grounded, policy)
    tracks = _merge_tracks(tracks, policy)
    tracks_2d = _track_ungrounded(frames_ungrounded) if include_ungrounded else []
    logger.info(f"association: {len(tracks)} grounded instances")

    _name_and_suppress(tracks, tracks_2d, store)
    tracks = [t for t in tracks if len(t.members) >= policy.min_member_observations]

    tracks.sort(key=lambda t: min(m.ts for m in t.members))
    instances: list[Instance] = []
    unknown = 0
    for index, track in enumerate(tracks):
        instance = _build_instance(index, track)
        if instance.primary_label is None:
            instance.primary_label = f"unknown-{unknown}"
            unknown += 1
        instances.append(instance)

    if include_ungrounded:
        for track2d in tracks_2d:
            labels = tuple(sorted(track2d.labels.items(), key=lambda kv: -kv[1]))
            primary = labels[0][0] if labels else None
            if primary is None:
                primary = f"unknown-{unknown}"
                unknown += 1
            latest = max(track2d.members, key=lambda d: d.ts)
            instances.append(
                Instance(
                    instance_id=f"obj-{len(instances):02d}",
                    grounded=False,
                    primary_label=primary,
                    labels=labels,
                    state="active",
                    identity_confidence=0.3,
                    support=None,
                    latest_position_xyz=None,
                    latest_seen_ts=latest.ts,
                    members=[],
                )
            )
    return instances


def _free_accelerator() -> None:
    import gc

    import torch

    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
    elif torch.backends.mps.is_available():
        torch.mps.empty_cache()
