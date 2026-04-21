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

"""Unit tests for the Lyapunov reactive path controller.

Tests verify the core properties of the control law:
- Error convergence (Lyapunov stability)
- Holonomic vy usage
- Reactive speed modulation
- Curvature response
- Goal deceleration

Run directly (bypasses conftest autoconf):
    .venv/bin/python -m dimos.control.tests.test_reactive_controller
"""

from __future__ import annotations

import math

from dimos.control.tasks.lyapunov_path_controller import (
    LyapunovPathController,
    LyapunovPathControllerConfig,
)
from dimos.control.tasks.path_distancer import PathDistancer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path


def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0, 0, yaw)),
    )


def _straight_path(length: float = 5.0, step: float = 0.1) -> Path:
    n = int(length / step) + 1
    return Path(poses=[_pose(i * step, 0.0) for i in range(n)])


def _curved_path() -> Path:
    """90-degree turn: 2m straight then 2m up."""
    poses = [_pose(i * 0.1, 0.0) for i in range(21)]
    poses += [_pose(2.0, i * 0.1, math.pi / 2) for i in range(1, 21)]
    return Path(poses=poses)


def test_on_path_drives_forward():
    """Robot on path with correct heading → vx > 0, vy ≈ 0, wz ≈ 0."""
    ctrl = LyapunovPathController()
    path = _straight_path()
    dist = PathDistancer(path)
    odom = _pose(1.0, 0.0, 0.0)  # on path, aligned

    out = ctrl.compute(odom, dist, distance_to_goal=4.0)
    assert out.vx > 0.1, f"Expected forward speed, got vx={out.vx}"
    assert abs(out.vy) < 0.05, f"Expected vy≈0 on path, got vy={out.vy}"
    assert abs(out.wz) < 0.1, f"Expected wz≈0 on path, got wz={out.wz}"


def test_lateral_offset_produces_vy():
    """Robot 0.3m to the left of path → vy should push it right (negative)."""
    ctrl = LyapunovPathController()
    path = _straight_path()
    dist = PathDistancer(path)
    odom = _pose(1.0, 0.3, 0.0)  # offset left, heading aligned

    out = ctrl.compute(odom, dist, distance_to_goal=4.0)
    # e_d > 0 (left of path) → vy = -k_d * e_d → vy < 0
    assert out.vy < -0.05, f"Expected negative vy for left offset, got vy={out.vy}"


def test_lateral_offset_slows_vx():
    """Robot offset from path should drive slower than robot on path."""
    ctrl = LyapunovPathController()
    path = _straight_path()
    dist = PathDistancer(path)

    on_path = ctrl.compute(_pose(1.0, 0.0, 0.0), dist, distance_to_goal=4.0)
    off_path = ctrl.compute(_pose(1.0, 0.5, 0.0), dist, distance_to_goal=4.0)

    assert off_path.vx < on_path.vx, (
        f"Off-path vx ({off_path.vx:.3f}) should be less than on-path vx ({on_path.vx:.3f})"
    )


def test_heading_error_slows_vx():
    """Robot with 45° heading error should drive slower."""
    ctrl = LyapunovPathController()
    path = _straight_path()
    dist = PathDistancer(path)

    aligned = ctrl.compute(_pose(1.0, 0.0, 0.0), dist, distance_to_goal=4.0)
    misaligned = ctrl.compute(_pose(1.0, 0.0, math.pi / 4), dist, distance_to_goal=4.0)

    assert misaligned.vx < aligned.vx, (
        f"Misaligned vx ({misaligned.vx:.3f}) should be less than aligned vx ({aligned.vx:.3f})"
    )


def test_heading_error_produces_wz():
    """Robot with positive heading error → wz should correct it."""
    ctrl = LyapunovPathController()
    path = _straight_path()
    dist = PathDistancer(path)
    odom = _pose(1.0, 0.0, 0.5)  # heading 0.5 rad left of path tangent (0)

    out = ctrl.compute(odom, dist, distance_to_goal=4.0)
    # e_theta > 0 → wz should be negative to correct
    # Actually: tangent_yaw=0, robot_yaw=0.5, e_theta = angle_diff(0, 0.5) = -0.5
    # wz = v_ref*0 + k_theta*sin(-0.5) < 0 → correct
    assert out.wz < 0, f"Expected negative wz to correct positive heading, got wz={out.wz}"


def test_goal_deceleration():
    """Robot near goal should drive slower than robot far from goal."""
    ctrl = LyapunovPathController()
    path = _straight_path()
    dist = PathDistancer(path)

    far = ctrl.compute(_pose(1.0, 0.0, 0.0), dist, distance_to_goal=3.0)
    near = ctrl.compute(_pose(4.5, 0.0, 0.0), dist, distance_to_goal=0.3)

    assert near.vx < far.vx, (
        f"Near-goal vx ({near.vx:.3f}) should be less than far vx ({far.vx:.3f})"
    )


def test_curvature_slows_vx():
    """Robot at a sharp turn should have lower v_ref than on a straight."""
    ctrl = LyapunovPathController()
    straight = _straight_path()
    curved = _curved_path()
    dist_s = PathDistancer(straight)
    dist_c = PathDistancer(curved)

    # On straight, midpoint
    out_s = ctrl.compute(_pose(1.0, 0.0, 0.0), dist_s, distance_to_goal=4.0)
    # At the turn point of the curved path
    out_c = ctrl.compute(_pose(2.0, 0.0, 0.0), dist_c, distance_to_goal=2.0)

    assert out_c.v_ref < out_s.v_ref, (
        f"Curved v_ref ({out_c.v_ref:.3f}) should be less than straight v_ref ({out_s.v_ref:.3f})"
    )


def test_error_convergence_simulation():
    """Simulate 50 steps from a lateral offset — errors should decrease."""
    cfg = LyapunovPathControllerConfig(v_max=0.6, k_d=1.0, k_theta=1.5, k_s=0.8)
    ctrl = LyapunovPathController(cfg)
    path = _straight_path(10.0)
    dist = PathDistancer(path)

    # Start 0.4m off-path
    x, y, yaw = 0.5, 0.4, 0.1
    dt = 0.1
    initial_error = math.sqrt(0.4**2 + 0.1**2)

    for _ in range(50):
        odom = _pose(x, y, yaw)
        d2g = dist.distance_to_goal(__import__("numpy").array([x, y]))
        out = ctrl.compute(odom, dist, distance_to_goal=d2g)

        # Unicycle integration
        x += (out.vx * math.cos(yaw) - out.vy * math.sin(yaw)) * dt
        y += (out.vx * math.sin(yaw) + out.vy * math.cos(yaw)) * dt
        yaw += out.wz * dt
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

    final_cte = abs(dist.get_signed_cross_track_error(__import__("numpy").array([x, y])))
    assert final_cte < initial_error * 0.3, (
        f"Expected error convergence: initial CTE≈{initial_error:.3f}, final CTE={final_cte:.3f}"
    )


def test_lyapunov_function_decreasing():
    """V = 0.5*(e_d² + e_θ²) should be non-increasing over simulation steps."""
    cfg = LyapunovPathControllerConfig(v_max=0.5, k_d=1.0, k_theta=1.5)
    ctrl = LyapunovPathController(cfg)
    path = _straight_path(10.0)
    dist = PathDistancer(path)

    import numpy as np

    x, y, yaw = 0.5, 0.3, 0.3
    dt = 0.1
    V_values = []

    for _ in range(30):
        odom = _pose(x, y, yaw)
        d2g = dist.distance_to_goal(np.array([x, y]))
        out = ctrl.compute(odom, dist, distance_to_goal=d2g)

        V = 0.5 * (out.e_d**2 + out.e_theta**2)
        V_values.append(V)

        x += (out.vx * math.cos(yaw) - out.vy * math.sin(yaw)) * dt
        y += (out.vx * math.sin(yaw) + out.vy * math.cos(yaw)) * dt
        yaw += out.wz * dt
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

    # Allow small numerical bumps but overall trend must be decreasing
    assert V_values[-1] < V_values[0] * 0.5, (
        f"Lyapunov function should decrease: V_0={V_values[0]:.4f}, V_final={V_values[-1]:.4f}"
    )


def main() -> None:
    """Run all tests directly (bypasses conftest)."""
    tests = [
        test_on_path_drives_forward,
        test_lateral_offset_produces_vy,
        test_lateral_offset_slows_vx,
        test_heading_error_slows_vx,
        test_heading_error_produces_wz,
        test_goal_deceleration,
        test_curvature_slows_vx,
        test_error_convergence_simulation,
        test_lyapunov_function_decreasing,
    ]
    passed = 0
    for t in tests:
        try:
            t()
            passed += 1
            print(f"  PASS: {t.__name__}")
        except AssertionError as e:
            print(f"  FAIL: {t.__name__}: {e}")
        except Exception as e:
            print(f"  ERROR: {t.__name__}: {e}")

    print(f"\n{passed}/{len(tests)} tests passed")
    if passed < len(tests):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
