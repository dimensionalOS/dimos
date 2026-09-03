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

"""The localize answer types and the thresholds a caller tunes per call."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

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
class Localization:
    """One verified instance. Cloud is every viewpoint's union; pose is the latest."""

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
    """Per-call thresholds. The funnel generalizes; these numbers do not.

    Each was fit to one measured scene, so a different rig, object scale or
    detector vocabulary passes its own instance instead of the defaults.
    """

    candidate_floor: float = 0.25  # OWLv2 score at which a box is formed
    accept_score: float = 0.40  # group's best member score to be returned
    min_views: int = 2  # distinct camera positions (1 cm) to confirm a group
    cluster_radius_m: float = 0.08  # two lifts are one object within this
    peak_prominence: float = 0.02
    peak_distance_s: float = 1.0
    peak_width_s: float | None = 0.5  # None disables the width gate
    verify_radius_m: float = 1.6  # gather index frames this close to a peak
    verify_window_s: float = 0.5  # keep the sharpest gathered frame per window
    settled_window_fraction: float = 0.5  # of 1/embed_hz; collapses split windows
    tail_k: int = 1  # extra frames past the last peak, so the tail can be latest
    max_object_extent_m: float = 0.60
    surface_patch_max_rise_m: float = 0.003  # with the drop test, a lift hugging
    surface_patch_min_drop_m: float = -0.02  # the support is surface, not object
    min_points: int = 60  # points on a lift
    min_camera_range_m: float = 0.28
    fuse_voxel_m: float = 0.01  # union cloud voxel at merge; 0 concatenates
    plane_cell_m: float = 2.0  # XY cell of the support-plane cache
    plane_keyframes: int = 5
    refusal_margin: float = 0.15  # below this against a coexisting rival, flagged
