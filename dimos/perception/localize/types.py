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

"""The localize answer type and its thresholds; each rig's values are in ``localize_params``."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass
class Localization:
    """One verified instance. Cloud is every viewpoint's union; position is the latest."""

    instance_id: str
    semantic_score: float
    ambiguity_margin: float
    position_world_xyz: tuple[float, float, float]
    frame_id: str
    last_seen_timestamp: float
    point_cloud: PointCloud2
    n_views: int


@dataclass(frozen=True)
class LocalizeParams:
    """The thresholds of one localize call. The funnel generalizes; these numbers do not.

    Each rig's values are in ``localize_params``, fit on its recordings; the
    scores are on the scales of the models they were fit with.
    """

    candidate_floor: float  # detector score at which a box is formed
    accept_score: float  # group's best member score to be returned
    min_views: int  # distinct camera positions (1 cm) to confirm a group
    cluster_radius_m: float  # two lifts are one object within this
    peak_prominence: float
    peak_distance_s: float
    peak_width_s: float | None  # None disables the width gate
    verify_radius_m: float  # gather index frames this close to a peak
    verify_window_s: float  # keep the sharpest gathered frame per window
    settled_window_fraction: float  # of 1/embed_hz; collapses split windows
    tail_k: int  # extra frames past the last peak, so the tail can be latest
    max_object_extent_m: float
    surface_patch_max_rise_m: float  # with the drop test, a lift hugging
    surface_patch_min_drop_m: float  # the support is surface, not object
    min_points: int  # points on a lift
    min_camera_range_m: float
    fuse_voxel_m: float  # union cloud voxel at merge; 0 concatenates
    plane_keyframes: int
    plane_cell_m: float  # XY cell of the support-plane cache
