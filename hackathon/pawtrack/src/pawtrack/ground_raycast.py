"""Pure pixel-to-ground raycast for absolute subject positioning (no DimOS).

Given the tracked subject's pixel in the camera image, the camera intrinsics,
and the camera's pose in a target frame, intersect the viewing ray with a
horizontal ground plane to recover the subject's position in that frame.

This is approach "B": it needs no depth sensor and no lidar return on the
subject -- only the camera pose and a known ground height -- so it stays robust
for a subject on a flat floor, where a sparse lidar might miss it (and so
``Detection3DPC.from_2d`` would return nothing). The result is *absolute* only
insofar as the supplied camera-to-target transform is into an absolute frame
(the ``map`` frame from relocalization); given a ``world``/odom transform the
result lands in that (drifting) frame instead.

Pure: numpy only, unit tested. The DimOS glue supplies the intrinsics (from
``CameraInfo.K``) and the 4x4 transform (``tf.get(target, camera_optical)
.to_matrix()``), then wraps the returned point in a ``PoseStamped``.
"""

from __future__ import annotations

import dataclasses

import numpy as np

# Rays whose ground-plane component is below this (in the target frame) are
# treated as parallel to the floor; the intersection would be at infinity.
_MIN_VERTICAL_COMPONENT = 1e-9


@dataclasses.dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics, in pixels.

    Attributes:
        fx: Focal length in x.
        fy: Focal length in y.
        cx: Principal point x.
        cy: Principal point y.
    """

    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("focal lengths fx, fy must be positive")

    @classmethod
    def from_k(cls, k: list[float]) -> CameraIntrinsics:
        """Build from a row-major 3x3 intrinsic matrix (e.g. ``CameraInfo.K``).

        Args:
            k: Nine elements ``[fx, 0, cx, 0, fy, cy, 0, 0, 1]`` (row-major).

        Returns:
            The intrinsics read from ``k``.

        Raises:
            ValueError: If ``k`` does not have nine elements.
        """
        if len(k) != 9:
            raise ValueError("K must have 9 elements (row-major 3x3)")
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


def pixel_to_ground_point(
    pixel: tuple[float, float],
    intrinsics: CameraIntrinsics,
    camera_to_target: np.ndarray,
    ground_z: float = 0.0,
) -> tuple[float, float, float] | None:
    """Intersect a pixel's viewing ray with a horizontal ground plane.

    The pixel is back-projected to a ray in the camera optical frame (``+x``
    right, ``+y`` down, ``+z`` forward), transformed into the target frame by
    ``camera_to_target``, and intersected with the plane ``z == ground_z``.

    Args:
        pixel: ``(u, v)`` pixel coordinates of the point (e.g. the subject's
            bbox bottom-center, where it meets the floor).
        intrinsics: Pinhole camera intrinsics.
        camera_to_target: 4x4 homogeneous transform mapping a camera-optical
            point to the target frame -- i.e. ``tf.get(target, camera_optical)
            .to_matrix()``.
        ground_z: Height of the ground plane in the target frame. Pass ``0``
            (the floor) for a pixel that sits on the floor; pass an offset only
            if the chosen pixel is above the floor.

    Returns:
        ``(x, y, z)`` of the ray/plane intersection in the target frame, or
        ``None`` if the ray is parallel to the plane or points away from it
        (the plane lies behind the camera along the ray).

    Raises:
        ValueError: If ``camera_to_target`` is not a 4x4 matrix.
    """
    matrix = np.asarray(camera_to_target, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("camera_to_target must be a 4x4 matrix")

    u, v = pixel
    ray_camera = np.array(
        [
            (u - intrinsics.cx) / intrinsics.fx,
            (v - intrinsics.cy) / intrinsics.fy,
            1.0,
        ]
    )
    origin = matrix[:3, 3]
    direction = matrix[:3, :3] @ ray_camera

    vertical = direction[2]
    if abs(vertical) < _MIN_VERTICAL_COMPONENT:
        return None  # ray parallel to the ground plane
    distance = (ground_z - origin[2]) / vertical
    if distance <= 0.0:
        return None  # intersection is behind the camera along the ray
    point = origin + distance * direction
    return (float(point[0]), float(point[1]), float(point[2]))
