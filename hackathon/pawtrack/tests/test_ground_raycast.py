"""Tests for the pure pixel-to-ground raycast."""

import numpy as np
import pytest

from pawtrack.ground_raycast import CameraIntrinsics, pixel_to_ground_point

# A camera 1 m above the floor looking straight down. Optical +z (view dir)
# maps to map -z (down); a 180-degree roll about x keeps the frame
# right-handed (optical +y -> map -y), so det(R) == +1.
_LOOK_DOWN = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
# fx == fy so a one-focal-length pixel offset is a 45-degree ray.
_INTR = CameraIntrinsics(fx=100.0, fy=100.0, cx=320.0, cy=240.0)


def _close(actual, expected, tol=1e-9):
    return all(abs(a - e) <= tol for a, e in zip(actual, expected))


def test_center_pixel_lands_directly_below_the_camera():
    point = pixel_to_ground_point((320.0, 240.0), _INTR, _LOOK_DOWN)
    assert _close(point, (0.0, 0.0, 0.0))


def test_offset_pixel_projects_with_correct_ground_offset():
    # One focal length right of center, camera 1 m up -> 45-degree ray -> 1 m.
    point = pixel_to_ground_point((420.0, 240.0), _INTR, _LOOK_DOWN)
    assert _close(point, (1.0, 0.0, 0.0))


def test_ground_z_offset_raises_the_intersection_plane():
    # A non-zero ground_z lifts the intersection plane off the floor.
    point = pixel_to_ground_point((320.0, 240.0), _INTR, _LOOK_DOWN, ground_z=0.11)
    assert _close(point, (0.0, 0.0, 0.11))


def test_camera_translation_offsets_the_ground_point():
    moved = _LOOK_DOWN.copy()
    moved[:3, 3] = [2.0, 3.0, 1.0]
    point = pixel_to_ground_point((320.0, 240.0), _INTR, moved)
    assert _close(point, (2.0, 3.0, 0.0))


def test_ray_pointing_up_returns_none():
    # Camera looking straight up: optical +z -> map +z, so the floor is behind.
    look_up = np.eye(4)
    look_up[2, 3] = 1.0  # 1 m above the floor
    assert pixel_to_ground_point((320.0, 240.0), _INTR, look_up) is None


def test_horizontal_ray_parallel_to_floor_returns_none():
    # Optical +z -> map +x (look along the floor): no vertical component.
    look_flat = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    assert pixel_to_ground_point((320.0, 240.0), _INTR, look_flat) is None


def test_from_k_reads_row_major_intrinsics():
    intr = CameraIntrinsics.from_k([100.0, 0.0, 320.0, 0.0, 110.0, 240.0, 0.0, 0.0, 1.0])
    assert (intr.fx, intr.fy, intr.cx, intr.cy) == (100.0, 110.0, 320.0, 240.0)


def test_from_k_rejects_wrong_length():
    with pytest.raises(ValueError, match="9 elements"):
        CameraIntrinsics.from_k([100.0, 0.0, 320.0])


def test_rejects_non_positive_focal_length():
    with pytest.raises(ValueError, match="focal lengths"):
        CameraIntrinsics(fx=0.0, fy=100.0, cx=1.0, cy=1.0)


def test_rejects_non_4x4_transform():
    with pytest.raises(ValueError, match="4x4"):
        pixel_to_ground_point((320.0, 240.0), _INTR, np.eye(3))
