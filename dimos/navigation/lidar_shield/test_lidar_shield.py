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

import math

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.lidar_shield.module import (
    LidarShieldConfig,
    _clamp_linear,
    _obstacle_columns,
    _ShieldCore,
    _validated_updates,
)

GROUND_Z = -0.4


def make_core(**overrides) -> _ShieldCore:
    core = _ShieldCore(LidarShieldConfig(**overrides))
    core.on_odom(0.0, 0.0, 0.0, now=0.0)
    return core


def ground_cloud(radius: float = 3.0, spacing: float = 0.1) -> np.ndarray:
    coords = np.arange(-radius, radius, spacing, dtype=np.float32)
    xx, yy = np.meshgrid(coords, coords)
    pts = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, GROUND_Z, np.float32)])
    return pts.astype(np.float32)


def column_at(x: float, y: float, top: float = 0.8) -> np.ndarray:
    z = np.arange(GROUND_Z, top, 0.05, dtype=np.float32)
    pts = np.column_stack([np.full(z.size, x, np.float32), np.full(z.size, y, np.float32), z])
    return pts


def scene(*obstacles: tuple[float, float]) -> np.ndarray:
    parts = [ground_cloud()] + [column_at(x, y) for x, y in obstacles]
    return np.vstack(parts)


def twist(vx: float, vy: float = 0.0, wz: float = 0.0) -> Twist:
    return Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))


def test_bubble_blocks_approach_but_not_retreat() -> None:
    core = make_core()
    changed = core.on_lidar(scene((0.42, 0.0)), now=0.1)
    assert changed and core.engaged
    assert core.nearest_m < 0.45

    out, _ = core.on_cmd(twist(0.5), now=0.2)  # toward: blocked
    assert out.linear.x == 0.0 and out.linear.y == 0.0

    out, _ = core.on_cmd(twist(-0.5), now=0.3)  # away: passes, clamped
    assert out.linear.x == -0.3

    out, _ = core.on_cmd(twist(0.0, 0.0, 1.0), now=0.4)  # pure rotation passes
    assert out.angular.z == 1.0


def test_escape_works_with_points_on_both_sides() -> None:
    core = make_core()
    core.on_lidar(scene((0.42, 0.0), (-2.0, 0.0)), now=0.1)
    assert core.engaged

    out, _ = core.on_cmd(twist(-0.5), now=0.2)  # rear column is far outside reach
    assert out.linear.x == -0.3


def test_strafe_past_wall_passes_at_full_speed() -> None:
    core = make_core()
    wall = [column_at(1.2, y) for y in np.arange(-1.0, 1.0, 0.1)]
    core.on_lidar(np.vstack([ground_cloud(), *wall]), now=0.1)

    out, changed = core.on_cmd(twist(1.5), now=0.2)  # straight at the wall: blocked
    assert changed and out.linear.x == 0.0
    assert out.angular.z == 0.0  # no contact, yaw untouched (input was 0)

    cmd = twist(0.0, 0.5)  # strafe parallel to the wall: free, unclamped
    out, _ = core.on_cmd(cmd, now=0.3)
    assert out is cmd


def test_clear_scene_passes_command_through() -> None:
    core = make_core()
    core.on_lidar(scene((2.5, 0.0)), now=0.1)
    assert not core.engaged

    cmd = twist(0.2)
    out, changed = core.on_cmd(cmd, now=0.2)
    assert not changed
    assert out is cmd


def test_capsule_stops_full_speed_before_wall() -> None:
    core = make_core()
    core.on_lidar(scene((1.2, 0.0)), now=0.1)
    assert not core.engaged

    # reach = 0.45 + 1.5 * 0.45 + 1.5^2 / (2 * 1.5) = 1.875 m > 1.2 m
    out, changed = core.on_cmd(twist(1.5), now=0.2)
    assert changed and core.engaged
    assert out.linear.x == 0.0

    # The same wall is out of reach at a crawl: 0.45 + 0.045 + 0.003 = 0.5 m
    core = make_core()
    core.on_lidar(scene((1.2, 0.0)), now=0.1)
    out, changed = core.on_cmd(twist(0.1), now=0.2)
    assert not changed and out.linear.x == 0.1


def test_capsule_respects_robot_yaw() -> None:
    core = make_core()
    core.on_odom(0.0, 0.0, math.pi / 2, now=0.0)
    # Wall on world +y, which is straight ahead once yaw is 90 degrees.
    core.on_lidar(scene((0.0, 1.2)), now=0.1)
    out, changed = core.on_cmd(twist(1.5), now=0.2)
    assert changed and out.linear.x == 0.0


def test_allow_escape_false_freezes_translation_while_engaged() -> None:
    core = make_core(allow_escape=False)
    core.on_lidar(scene((0.42, 0.0)), now=0.1)
    assert core.engaged
    out, _ = core.on_cmd(twist(-0.5, 0.0, 1.0), now=0.2)
    assert out.linear.x == 0.0 and out.angular.z == 0.0


def test_release_needs_consecutive_clear_frames() -> None:
    core = make_core()
    core.on_lidar(scene((0.42, 0.0)), now=0.1)
    assert core.engaged

    clear = scene()
    assert not core.on_lidar(clear, now=0.2)
    assert not core.on_lidar(clear, now=0.3)
    assert core.on_lidar(clear, now=0.4)  # third clear frame releases
    assert not core.engaged


def test_ground_points_never_trigger() -> None:
    core = make_core()
    core.on_lidar(ground_cloud(), now=0.1)
    assert core.points_in_band == 0
    out, changed = core.on_cmd(twist(1.5), now=0.2)
    assert not changed and out.linear.x == 1.5


def test_body_footprint_points_are_ignored() -> None:
    core = make_core()
    # Post-bump smear at the center and self-returns off a leg: both are
    # inside the body box and never obstacles, however tall their columns.
    core.on_lidar(scene((0.01, 0.0), (0.30, 0.10)), now=0.1)
    assert not core.engaged
    out, changed = core.on_cmd(twist(0.5), now=0.2)
    assert not changed and out.linear.x == 0.5


def test_floor_dilation_while_walking_never_triggers() -> None:
    # The moving Go2 stacks floor voxels 2-4 levels (0.10-0.20 m) high;
    # such speckles ahead must not read as obstacles on a flat surface.
    speckles = np.array(
        [
            [0.7, 0.10, GROUND_Z + 0.15],
            [0.8, 0.05, GROUND_Z + 0.16],
            [0.9, -0.05, GROUND_Z + 0.14],
            [1.0, 0.00, GROUND_Z + 0.18],
            [1.1, 0.08, GROUND_Z + 0.20],
        ],
        dtype=np.float32,
    )
    core = make_core()
    core.on_lidar(np.vstack([ground_cloud(), speckles]), now=0.1)
    assert core.points_in_band == 0
    out, changed = core.on_cmd(twist(1.5), now=0.2)
    assert not changed and out.linear.x == 1.5


def test_low_column_below_top_threshold_is_ignored() -> None:
    core = make_core()
    low_box = column_at(1.0, 0.0, top=GROUND_Z + 0.25)
    core.on_lidar(np.vstack([ground_cloud(), low_box]), now=0.1)
    out, changed = core.on_cmd(twist(1.5), now=0.2)
    assert not changed and out.linear.x == 1.5


def test_stale_state_is_motion_aware() -> None:
    core = make_core()
    core.on_lidar(scene(), now=0.0)
    for t in np.arange(0.0, 3.0, 0.1):
        core.on_odom(0.0, 0.0, 0.0, now=float(t))  # standing still
    assert core.stale_state(1.0) == "fresh"
    assert core.stale_state(3.0) == "paused"  # stationary idle: benign

    core = make_core()
    core.on_lidar(scene(), now=0.0)
    for i, t in enumerate(np.arange(2.2, 3.0, 0.1)):
        core.on_odom(0.5 * i * 0.1, 0.0, 0.0, now=float(t))  # walking at 0.5 m/s
    assert core.stale_state(3.0) == "dropout"  # moving blind: fail closed

    core = make_core()
    core.on_lidar(scene(), now=0.0)
    assert core.stale_state(3.0) == "dropout"  # odom dead too


def test_clamp_linear() -> None:
    out = _clamp_linear(twist(0.4, 0.3, 0.7), max_speed=0.25)
    assert math.hypot(out.linear.x, out.linear.y) == pytest.approx(0.25)
    assert out.angular.z == 0.7

    slow = twist(0.1, 0.0, 0.2)
    assert _clamp_linear(slow, max_speed=0.25) is slow


def test_injected_cloud_carries_intensity_tag() -> None:
    pts = np.zeros((5, 3), dtype=np.float32)
    pc = PointCloud2.from_numpy(pts, intensities=np.ones(5, dtype=np.float32))
    assert "intensities" in pc.pointcloud_tensor.point
    raw = PointCloud2.from_numpy(pts)
    assert "intensities" not in raw.pointcloud_tensor.point


def test_validated_updates_coerces_and_rejects_unknown() -> None:
    cfg = LidarShieldConfig()
    updates = _validated_updates(cfg, {"shield_radius_m": "0.5", "allow_escape": False})
    assert updates == {"shield_radius_m": 0.5, "allow_escape": False}
    assert cfg.shield_radius_m == 0.45  # untouched until the caller applies

    with pytest.raises(ValueError, match="typo_field"):
        _validated_updates(cfg, {"typo_field": 1.0})


def test_obstacle_columns_dedup_and_span() -> None:
    xy = np.array([[1.0, 1.0], [1.001, 1.001], [2.0, 2.0]], dtype=np.float32)
    cols = _obstacle_columns(xy, ground_z=GROUND_Z, voxel=0.05, height=0.6, max_columns=100)
    levels = round(0.6 / 0.05)
    assert cols.shape == (2 * levels, 3)
    assert cols[:, 2].min() > GROUND_Z
    assert cols[:, 2].max() <= GROUND_Z + 0.6 + 0.025
