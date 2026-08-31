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

"""Object registration types: supports, instances, localizations, policies.

The identity key throughout is the *support* - the object's occupied volume
in world coordinates. Labels and appearance are metadata attached to a
support, never a key.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal

import numpy as np

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class Support:
    """Occupied volume with an error envelope. The identity key."""

    center_xyz: tuple[float, float, float]
    extent_xyz_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    sigma_xyz_m: tuple[float, float, float]
    coverage: float
    axes_observed: tuple[bool, bool, bool]
    frame_id: str


@dataclass
class SupportObservation:
    """One accepted per-frame observation of a support: a masked depth lift."""

    ts: float
    cloud: PointCloud2
    centroid: np.ndarray  # (3,) world
    aabb_min: np.ndarray  # (3,) world
    aabb_max: np.ndarray  # (3,) world
    n_points: int
    mask_area_px: int
    camera_position: np.ndarray  # (3,) world
    score: float = 1.0
    bbox: tuple[float, float, float, float] | None = None


@dataclass
class Instance:
    """A deduplicated object instance computed for one inventory call."""

    instance_id: str
    grounded: bool
    primary_label: str | None
    labels: tuple[tuple[str, float], ...]
    state: Literal["active", "occluded", "stale", "retired"]
    identity_confidence: float
    support: Support | None
    latest_position_xyz: tuple[float, float, float] | None
    latest_seen_ts: float
    members: list[SupportObservation] = field(default_factory=list)


@dataclass
class Localization:
    """One verified instance of a queried object.

    ``point_cloud`` is the union of every viewpoint that saw the instance;
    position and timestamps follow the latest sighting.
    """

    instance_id: str
    semantic_score: float
    identity_score: float
    ambiguity_margin: float

    position_world_xyz: tuple[float, float, float] | None
    orientation_world_xyzw: tuple[float, float, float, float] | None
    frame_id: str

    support: Support | None
    pose_timestamp: float
    geometry_timestamp: float
    last_seen_timestamp: float

    point_cloud: PointCloud2 | None
    coverage: float
    n_views: int
    reason: str | None = None


@dataclass(frozen=True)
class LocalizePolicy:
    """Score and geometry thresholds for candidate formation, lift and acceptance.

    The funnel generalizes; these numbers do not. Each was fit to the score
    and height distributions of one measured scene, so a different rig,
    object scale or detector vocabulary needs its own instance rather than
    the defaults.

    `candidate_floor`: OWLv2 score at which a box is formed for a query.
        Boxes below this never lift.

    `accept_score`: A support group is returned when its highest member score meets this.
        The RGB-only path uses the same floor.

    `min_views`: Unique camera positions, rounded to 1 cm, required before a group is confirmed.
        A support seen from one pose only is dropped.

    `cluster_radius_m`: Two lifted detections are the same support when their cloud centers sit within this many meters.

    `peak_prominence`: Minimum SigLIP similarity rise for a semantic peak.
        A local maximum below this is not a peak.

    `peak_distance_s`: Minimum seconds between two semantic peaks.

    `peak_width_s`: Minimum peak width in seconds at half prominence.
        ``None`` disables the width gate.

    `verify_radius_m`: Index frames whose pose is within this many meters of a peak are gathered for OWLv2.

    `verify_window_s`: Keep the sharpest gathered frame per this many seconds.

    `settled_window_fraction`: Collapse index frames closer than this fraction of ``1/embed_hz`` to the sharper one, so one window does not count as two viewpoints.

    `tail_k`: After the last peak, take this many extra frames by query similarity so the window tail can be a latest sighting.

    `max_object_extent_m`: Drop a lift whose longest AABB edge exceeds this.

    `surface_patch_max_rise_m`: Drop a lift whose 95th-percentile height above the support is below this, when the drop test also holds.
        A cloud that hugs the support is a patch of the surface, not an object.

    `surface_patch_min_drop_m`: Drop a lift whose 5th-percentile height is above this, when the rise test also holds.

    `min_depth_points`: Minimum points on a depth lift.
        Projected-cloud rigs ignore this.

    `min_camera_range_m`: Drop a lift whose median point-to-camera range is below this.

    `fuse_voxel_m`: Voxel size of the union cloud at identity merge.
        ``0`` concatenates.

    `plane_cell_m`: XY cell size for the support-plane cache.

    `plane_keyframes`: Candidate frames sampled to fit the support plane.

    `refusal_margin`: If this instance's score minus the best coexisting rival is below this, ``reason`` is set.
        The instance is still returned.
    """

    candidate_floor: float = 0.25
    accept_score: float = 0.40
    min_views: int = 2
    cluster_radius_m: float = 0.08
    peak_prominence: float = 0.02
    peak_distance_s: float = 1.0
    peak_width_s: float | None = 0.5
    verify_radius_m: float = 1.6
    verify_window_s: float = 0.5
    settled_window_fraction: float = 0.5
    tail_k: int = 1
    max_object_extent_m: float = 0.60
    surface_patch_max_rise_m: float = 0.003
    surface_patch_min_drop_m: float = -0.02
    min_depth_points: int = 60
    min_camera_range_m: float = 0.28
    fuse_voxel_m: float = 0.01
    plane_cell_m: float = 2.0
    plane_keyframes: int = 5
    refusal_margin: float = 0.15


@dataclass(frozen=True)
class InventoryPolicy:
    """Thresholds for discovery, validity, scope, association and naming.

    Every geometric quantity is metric (meters, seconds, pixels, IoU) - a
    claim that can be checked against the recording. The two naming numbers
    are detector scores and decide whether a name is reported, never whether
    an instance exists. The candidate names are data and travel as a call
    argument, not as policy.
    """

    keyframe_stride_s: float = 2.5  # proposal keyframe grid
    min_mask_area_px: int = 400
    max_mask_area_fraction: float = 0.25
    min_depth_points: int = 60
    max_object_extent_m: float = 0.45
    min_height_above_plane_m: float = 0.003
    band_above_plane_m: tuple[float, float] = (-0.02, 0.30)
    min_camera_range_m: float = 0.28

    envelope_pad_m: float = 0.015
    search_radius_m: float = 0.15
    overlap_accept: float = 0.20
    # Same-object views may differ in bounding size by partiality alone; a
    # gap beyond this is two different bodies.
    size_gap_max_m: float = 0.25
    # The majority of a candidate's points must lie within the error envelope
    # of the track's accumulated support. Partial and newly revealed views of
    # one object satisfy this; a different object placed at a vacated rest
    # position does not, which is what AABB overlap cannot express.
    support_explained: float = 0.5
    # A lifted cloud plainly spanning more than one object: wider than any
    # single object at this rig's scale, or taller than one body. Repaired by
    # stripping support-surface points and splitting by 3D connectivity.
    split_extent_m: float = 0.30
    split_height_m: float = 0.10
    split_eps_m: float = 0.03
    # Same-frame observations whose clouds touch within this gap are one
    # body - rigid objects cannot interpenetrate, and distinct objects on a
    # workspace sit apart by more than sensor noise. This is what fuses
    # whole-and-part duplicate proposals while identical twins, centimeters
    # apart, stay two.
    same_frame_merge_gap_m: float = 0.02
    # A support observed in a single keyframe is unconfirmed - nothing saw it
    # from a second pose or moment, so it never becomes an instance.
    min_member_observations: int = 2

    # Naming abstention: a name is reported only when it clears the accept
    # floor and beats the runner-up canonical group by the margin, at the
    # frame and again over a track. Otherwise the instance stays unknown-N.
    name_accept_score: float = 0.18
    name_refusal_margin: float = 0.06
    # An attachment must be the detector drawing a box around this member.
    # Whole-object masks want a strict overlap; fragment masks of a large
    # object overlap their object's box only partially.
    name_attach_iou: float = 0.45

    include_object_parts: bool = False
    include_surfaces: bool = False
    include_containers: bool = True
    preserve_unknown_instances: bool = True

    in_scope: Callable[[np.ndarray], bool] | None = None


def aabb_overlap(
    a_min: np.ndarray,
    a_max: np.ndarray,
    b_min: np.ndarray,
    b_max: np.ndarray,
    pad: float = 0.0,
) -> float:
    """Intersection volume normalized by the smaller (padded) box volume.

    ``pad`` grows each box by the error envelope on every side, so the value
    is an overlap of envelopes, not of raw partial-view boxes. Degenerate
    axes are floored at 1 cm so thin objects (a pen, a sticky pad) do not
    produce zero volumes.
    """
    a_lo, a_hi = a_min - pad, a_max + pad
    b_lo, b_hi = b_min - pad, b_max + pad
    inter = np.minimum(a_hi, b_hi) - np.maximum(a_lo, b_lo)
    if (inter <= 0).any():
        return 0.0
    floor = 0.01
    vol_a = float(np.prod(np.maximum(a_hi - a_lo, floor)))
    vol_b = float(np.prod(np.maximum(b_hi - b_lo, floor)))
    vol_i = float(np.prod(np.maximum(inter, 0.0)))
    return vol_i / max(min(vol_a, vol_b), 1e-9)
