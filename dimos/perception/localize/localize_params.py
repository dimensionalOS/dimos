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

"""Each rig's ``LocalizeParams``, fit with SigLIP + OWLv2 + EdgeTAM on its recordings.

``PerceptionLocalize`` takes them at construction. A ``localize`` call, and
``tools/tool_localize --params``, change them with a string that
:func:`parse_params` reads: a preset name from ``PRESETS``, or a JSON object of
fields set on the current params, or on the preset its ``"base"`` key names.
"""

from __future__ import annotations

from dataclasses import replace
import json
from typing import Any

from dimos.perception.localize.types import LocalizeParams

# desk scale: a wrist or overhead camera over a tabletop
MANIPULATION_PARAMS = LocalizeParams(
    candidate_floor=0.25,
    accept_score=0.40,
    min_views=2,
    cluster_radius_m=0.08,
    peak_prominence=0.02,
    peak_distance_s=1.0,
    peak_width_s=0.5,
    verify_radius_m=1.6,
    verify_window_s=0.5,
    settled_window_fraction=0.5,
    tail_k=1,
    max_object_extent_m=0.60,
    surface_patch_max_rise_m=0.003,
    surface_patch_min_drop_m=-0.02,
    min_points=60,
    min_camera_range_m=0.28,
    fuse_voxel_m=0.01,
    plane_keyframes=5,
    plane_cell_m=2.0,
)

# room scale: a walking robot, furniture-sized objects, viewpoints meters apart,
# the lidar a 5 cm voxel map
NAVIGATION_PARAMS = replace(
    MANIPULATION_PARAMS,
    candidate_floor=0.18,
    accept_score=0.32,
    cluster_radius_m=0.30,
    verify_radius_m=5.0,
    max_object_extent_m=2.0,
    surface_patch_max_rise_m=0.08,
    surface_patch_min_drop_m=-0.06,
    min_points=13,
    min_camera_range_m=0.5,
    fuse_voxel_m=0.03,
)

# per-rig presets derived from the manipulation and navigation baselines
XARM_PARAMS = MANIPULATION_PARAMS
# the desk seen by dual OpenYAM wrist D405s, which see objects from 7 cm
DUAL_YAM_PARAMS = replace(MANIPULATION_PARAMS, min_camera_range_m=0.07)
GO2_PARAMS = NAVIGATION_PARAMS
# walking robot with dense depth instead of a 5 cm lidar map
G1_PARAMS = replace(NAVIGATION_PARAMS, min_points=30)

PRESETS: dict[str, LocalizeParams] = {
    "manipulation": MANIPULATION_PARAMS,
    "navigation": NAVIGATION_PARAMS,
    "xarm": XARM_PARAMS,
    "dual_yam": DUAL_YAM_PARAMS,
    "go2": GO2_PARAMS,
    "g1": G1_PARAMS,
}


def parse_params(spec: str, current: LocalizeParams) -> tuple[LocalizeParams, dict[str, Any]]:
    """The params ``spec`` starts from, and the fields it sets on them.

    ``""`` keeps ``current``, a preset name switches to that preset, and a JSON
    object sets its fields on ``current``, or on the preset its ``"base"`` key names.
    """
    spec = spec.strip()
    fields: dict[str, Any] = {}
    name: str | None = spec or None
    if spec.startswith("{"):
        fields = json.loads(spec)
        name = fields.pop("base", None)
    if name is None:
        return current, fields
    if name not in PRESETS:
        raise ValueError(f"unknown preset {name!r}: one of {', '.join(PRESETS)}")
    return PRESETS[name], fields
