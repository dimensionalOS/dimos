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

"""Robot-mounted camera spec + frustum overlay helpers.

A ``CameraSpec`` is a fixed-mount RGB camera attached to a body in the
robot MJCF.  It carries everything a renderer (or a frustum overlay)
needs: the parent body name, a local mount transform in the body
frame, image-plane intrinsics, and an output resolution.

Conventions:
  * Mount quaternion is in **wxyz** order, expressing the camera's
    optical frame in the body frame: image-x = right, image-y = down,
    image-z = forward.  This matches OpenCV / viser camera conventions.
  * Robot body frame in the bundled G1 MJCF is the standard ROS one:
    body-x = forward, body-y = left, body-z = up.

The default ``g1_d435_default()`` mounts a RealSense D435i color
sensor on ``head_link`` looking forward — a sensible "robot's eye
view" for office walking.  Override the spec to pitch down for
manipulation, mount on torso, swap to a different sensor, etc.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class CameraSpec:
    """Fixed-mount RGB camera attached to a robot body."""

    body_name: str
    """MJCF body name to mount on (e.g. 'head_link', 'torso_link')."""

    mount_pos: tuple[float, float, float]
    """Camera optical-center position in the parent body's local frame, meters."""

    mount_wxyz: tuple[float, float, float, float]
    """Quaternion (w, x, y, z) mapping body frame to camera optical frame."""

    vfov_deg: float
    """Vertical field of view, degrees.  Horizontal FOV is derived from aspect."""

    width: int
    """Image width, pixels."""

    height: int
    """Image height, pixels."""

    frustum_scale: float = 0.15
    """Frustum wireframe size in meters when overlaid in viser."""

    frustum_color: tuple[int, int, int] = (50, 255, 100)
    """RGB 0..255 for the frustum overlay.  Lime by default."""

    @property
    def aspect(self) -> float:
        return self.width / self.height

    def focal_pixels(self) -> float:
        """Focal length in pixels, derived from VFOV.  Square pixels assumed."""
        return 0.5 * self.height / float(np.tan(np.radians(self.vfov_deg) * 0.5))

    def fx(self) -> float:
        return self.focal_pixels()

    def fy(self) -> float:
        return self.focal_pixels()

    def cx(self) -> float:
        return 0.5 * self.width

    def cy(self) -> float:
        return 0.5 * self.height


def _quat_from_matrix(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix -> (w, x, y, z) via mujoco.  Stable, no scipy dep."""
    flat = np.asarray(R, dtype=np.float64).flatten()
    out = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(out, flat)
    return (float(out[0]), float(out[1]), float(out[2]), float(out[3]))


def g1_d435_default() -> CameraSpec:
    """RealSense D435i color sensor mounted on G1 torso, pitched down.

    Mount: on ``torso_link``.  Offset
    ``(0.0576, 0.0325, 0.4299)`` matches the URDF d435_joint position
    (Intel-spec RGB sensor offset already baked in).  Optical axis
    pitched down by ~47.6° (0.831 rad) — same value Matrix uses on
    real G1 deployments — so the camera sees the workspace in front
    of the chest, not the horizon.  This is what manipulation /
    object registration consume; pick-and-place needs a downward
    view of the table.

    Intrinsics are Intel datasheet for D435i color
    (HFOV 69.4°, VFOV 42.5°), binned to 320x180.
    """
    # Body frame  : +x forward, +y left, +z up
    # Image frame : +x right,   +y down, +z forward
    # body_R_image = horizontal_forward @ R_pitch_down(47.6°)
    pitch_rad = 0.831
    c, s = np.cos(pitch_rad), np.sin(pitch_rad)
    horizontal = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=np.float64,
    )
    pitch_down = np.array(
        [[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]],
        dtype=np.float64,
    )
    body_R_image = horizontal @ pitch_down
    return CameraSpec(
        body_name="torso_link",
        mount_pos=(0.0576, 0.0325, 0.4299),
        mount_wxyz=_quat_from_matrix(body_R_image),
        vfov_deg=42.5,
        width=320,
        height=180,
    )


def g1_d435_forward() -> CameraSpec:
    """RealSense D435i mounted forward at G1 eye level (no pitch).

    Alternate to ``g1_d435_default`` for when the agent wants a
    "robot's eye view" of the room — looking at the horizon, walls,
    people — instead of the manipulation-oriented downward pitch.
    Useful for navigation / exploration / scene understanding tasks
    that don't care about the table workspace in front of the chest.

    Mount: on ``torso_link`` at ``(0.10, 0.0, 0.40)`` — 10 cm forward
    of the torso origin and 40 cm up, roughly at eye level just past
    the forehead mesh.  Optical axis horizontal.

    Intrinsics are Intel datasheet for D435i color
    (HFOV 69.4°, VFOV 42.5°), binned to 320x180.
    """
    body_R_image = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=np.float64,
    )
    return CameraSpec(
        body_name="torso_link",
        mount_pos=(0.10, 0.0, 0.40),
        mount_wxyz=_quat_from_matrix(body_R_image),
        vfov_deg=42.5,
        width=320,
        height=180,
    )


def world_pose(
    body_world_pos: np.ndarray,
    body_world_wxyz: np.ndarray,
    spec: CameraSpec,
) -> tuple[np.ndarray, np.ndarray]:
    """Compose the camera's world pose from its parent body's world pose.

    Returns ``(world_pos (3,), world_wxyz (4,))`` ready to feed straight
    into viser's ``add_camera_frustum`` (which uses the same
    +Z-forward / +Y-down convention the camera spec is in).
    """
    # World rotation: q_world_image = q_world_body * q_body_image
    out_quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mulQuat(
        out_quat,
        np.asarray(body_world_wxyz, dtype=np.float64),
        np.asarray(spec.mount_wxyz, dtype=np.float64),
    )

    # World translation: p_world_image = p_world_body + R_world_body @ mount_pos
    body_R_world = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(body_R_world, np.asarray(body_world_wxyz, dtype=np.float64))
    body_R_world = body_R_world.reshape(3, 3)
    out_pos = np.asarray(body_world_pos, dtype=np.float64) + body_R_world @ np.asarray(
        spec.mount_pos, dtype=np.float64
    )

    return out_pos, out_quat


__all__ = ["CameraSpec", "g1_d435_default", "g1_d435_forward", "world_pose"]
