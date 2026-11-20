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

"""Camera viewpoints for a wrist camera looking at a fixed tag."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.calibration.hand_eye import checked_transform
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform

HOME_SWEEP_SPACES = (
    "home_camera_offsets_world_m",
    "home_camera_offsets_world_m_v2",
    "home_camera_offsets_world_m_v3",
)


def read_orbit_mount(path: Path, mount_frame: str, camera_frame: str) -> Transform:
    """Read a validated camera mount for aiming the camera during a new calibration."""
    result = json.loads(path.read_text())
    if not isinstance(result, dict) or result.get("version") != 1:
        raise ValueError("Expected a version 1 hand_eye.json for orbit planning.")
    if result.get("validated") is not True or not all(
        isinstance(metrics := result.get(key), dict) and metrics.get("passed") is True
        for key in ("training_metrics", "validation_metrics")
    ):
        raise ValueError("Orbit planning needs a calibration that passed training and validation.")
    if (
        result.get("translation_unit") != "metre"
        or result.get("parent_frame") != mount_frame
        or result.get("child_frame") != camera_frame
    ):
        raise ValueError(
            f"Orbit calibration must map {camera_frame!r} into {mount_frame!r} in metres."
        )
    return Transform.from_matrix(
        checked_transform(np.asarray(result.get("camera_to_ee"))),
        frame_id=mount_frame,
        child_frame_id=camera_frame,
    )


def tag_views(count: int, validation: bool, *, flat_outer_arc: bool = False) -> list[list[float]]:
    """Rebuild the tag-plane arc stored by older sessions."""
    if validation:
        angles = np.linspace(60.0, -60.0, count)
        return [[float(angle), 0.09, 0.19] for angle in angles]
    angles = np.linspace(-50.0, 50.0, count)
    # Rise through the middle of the outer sweep to vary the viewing angle.
    rise = 0.0 if flat_outer_arc else 0.04
    return [[float(a), 0.17, float(0.18 + rise * (1.0 - (a / 50.0) ** 2))] for a in angles]


def home_sweep_views(
    count: int, validation: bool, *, space: str = HOME_SWEEP_SPACES[-1]
) -> list[list[float]]:
    """Optical-center offsets from home in world XYZ metres, with X forward and Z up."""
    if validation:
        lateral = np.linspace(1.0, -1.0, count)
        # Return closer to the tag, on a narrower and lower path.
        forward = 0.078 - 0.083 * lateral**2
        up = -0.050 - 0.047 * np.abs(lateral)
        offsets = np.column_stack((forward, 0.125 * lateral, up))
        if count == 4 and space != HOME_SWEEP_SPACES[0]:
            # Use the halfway pose that cleared the reflection on the printed tag.
            offsets[2] *= 0.5
    else:
        # Both endpoints and the raised center need a capture even for an even count.
        # Split any extra views evenly along each of the two straight segments.
        right = [0.012, -0.155, -0.110]
        raised_center = [-0.068, 0.0, 0.022]
        left = [0.012, 0.155, -0.110]
        first_half = (count - 1) // 2
        to_center = np.linspace(right, raised_center, first_half + 1)
        from_center = np.linspace(raised_center, left, count - first_half)
        offsets = np.concatenate((to_center, from_center[1:]))
        if count == 7 and space == HOME_SWEEP_SPACES[2]:
            # The first stop after center read the tag more reliably halfway toward home.
            offsets[4] *= 0.5
    views: list[list[float]] = offsets.tolist()
    return views


def tag_view_pose(
    home: PoseStamped,
    tip_from_camera: NDArray[np.float64],
    camera_from_tag: NDArray[np.float64],
    view: list[float],
    fraction: float,
    *,
    home_relative: bool = False,
) -> PoseStamped:
    """Move the optical center and aim its +Z axis at the tag with minimal wrist rotation."""
    home_camera = Transform.from_pose("tip", home).to_matrix() @ tip_from_camera
    world_from_tag = home_camera @ camera_from_tag
    if home_relative:
        offset = np.asarray(view)
        # Translation is relative to home, so a closer or tilted target can put
        # a requested view behind the printed face.
        normal = world_from_tag[:3, 2]
        home_height = (home_camera[:3, 3] - world_from_tag[:3, 3]) @ normal
        if home_height * (home_height + fraction * (offset @ normal)) <= 0:
            raise ValueError(
                "View crosses the tag plane. Choose a centered home farther from the tag."
            )
    else:
        # Old sessions recorded the camera center in cylindrical tag coordinates.
        home_in_tag = np.linalg.inv(camera_from_tag)[:3, 3]
        azimuth, radius, height = view
        angle = math.radians(azimuth)
        if np.linalg.norm(home_in_tag[:2]) > 1e-6:
            angle += math.atan2(home_in_tag[1], home_in_tag[0])
        center = np.array(
            [
                radius * math.cos(angle),
                radius * math.sin(angle),
                math.copysign(height, home_in_tag[2]),
            ]
        )
        offset = world_from_tag[:3, :3] @ (center - home_in_tag)
    camera = home_camera.copy()
    camera[:3, 3] += fraction * offset
    to_tag = world_from_tag[:3, 3] - camera[:3, 3]
    aim, _ = Rotation.align_vectors(to_tag / np.linalg.norm(to_tag), home_camera[:3, 2])
    camera[:3, :3] = aim.as_matrix() @ home_camera[:3, :3]
    goal = camera @ np.linalg.inv(tip_from_camera)
    return Transform.from_matrix(goal, frame_id=home.frame_id).to_pose()
