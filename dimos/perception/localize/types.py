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

"""The localize answer type and its per-call thresholds."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.messages.sensor_msgs.msg import PointCloud2


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

    Each rig passes its own instance on the blueprint; the defaults are desk
    scale, fit on the xArm recordings with SigLIP + OWLv2 + EdgeTAM.
    """

    candidate_floor: float = 0.25  # detector score at which a box is formed
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
    plane_keyframes: int = 5
    plane_cell_m: float = 2.0  # XY cell of the support-plane cache

    def read_key(self) -> LocalizeParams:
        """What a frame counts as read under: these params without the answer gates,
        which act only once the answer is built, so changing them reads no frame again."""
        return replace(self, accept_score=0.0, min_views=0)
