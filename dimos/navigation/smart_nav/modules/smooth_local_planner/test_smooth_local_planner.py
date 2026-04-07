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

"""Unit tests for SmoothLocalPlanner pure functions + integration."""

from __future__ import annotations

import math

import numpy as np

from dimos.navigation.smart_nav.modules.smooth_local_planner.smooth_local_planner import (
    SmoothLocalPlanner,
    crop_obstacles,
    generate_arc,
    goal_in_vehicle_frame,
    score_candidate,
    select_curvature,
    update_ema,
)

# Shared defaults for score weights
_SCORE_KW = dict(
    robot_radius=0.50,
    clearance_cap=1.5,
    clearance_weight=2.0,
    alignment_weight=1.0,
    hysteresis_weight=0.4,
    hysteresis_sigma=0.4,
)


class _Cfg:
    """Minimal config holder for building a planner via __new__."""

    num_candidates = 21
    max_curvature = 1.5
    arc_length = 3.0
    arc_step = 0.15
    clearance_weight = 2.0
    alignment_weight = 1.0
    hysteresis_weight = 0.4
    hysteresis_sigma = 0.4
    clearance_cap = 1.5
    robot_radius = 0.50
    obstacle_range = 4.0
    obstacle_height_threshold = 0.15
    max_relative_z = 1.5
    min_relative_z = -1.5
    curvature_ema_alpha = 0.3
    blocked_speed_scale = 0.3
    publish_rate = 20.0
    publish_length = 3.0
    frame_id = "vehicle"


def _make_planner() -> SmoothLocalPlanner:
    p = SmoothLocalPlanner.__new__(SmoothLocalPlanner)
    p.config = _Cfg()  # type: ignore[assignment]
    p._build_arc_cache()
    p._prev_kappa_ema = 0.0
    return p


# ─── generate_arc ─────────────────────────────────────────────────────────


class TestGenerateArc:
    def test_straight_line_kappa_zero(self) -> None:
        arc = generate_arc(0.0, 3.0, 0.15)
        assert np.all(arc[:, 1] == 0.0)
        assert abs(arc[-1, 0] - 3.0) < 1e-6
        assert arc[0, 0] == 0.0

    def test_positive_kappa_curves_left(self) -> None:
        arc = generate_arc(0.5, 3.0, 0.15)
        # y monotonically increasing (curves +y)
        diffs = np.diff(arc[:, 1])
        assert np.all(diffs >= -1e-9)

    def test_endpoint_distance_along_arc(self) -> None:
        # Arc length from origin to last sample is ~= length
        length = 3.0
        arc = generate_arc(0.5, length, 0.15)
        seg = np.diff(arc, axis=0)
        total = float(np.sum(np.hypot(seg[:, 0], seg[:, 1])))
        assert abs(total - length) < 0.05

    def test_sample_count(self) -> None:
        arc = generate_arc(0.0, 3.0, 0.15)
        # length/step + 1 = 21, ± 1
        assert abs(arc.shape[0] - 21) <= 1

    def test_sample_spacing(self) -> None:
        arc = generate_arc(0.3, 3.0, 0.15)
        seg = np.diff(arc, axis=0)
        d = np.hypot(seg[:, 0], seg[:, 1])
        # chord ≈ step for small step
        assert np.all(np.abs(d - 0.15) < 0.02)


# ─── crop_obstacles ───────────────────────────────────────────────────────


class TestCropObstacles:
    def test_disk_crop(self) -> None:
        pts = np.array([[1.0, 0.0, 0.0], [5.0, 0.0, 0.0], [0.0, 2.0, 0.0]])
        out = crop_obstacles(pts, obstacle_range=3.0, min_relative_z=-5.0, max_relative_z=5.0)
        assert out.shape == (2, 2)

    def test_height_band_crop(self) -> None:
        pts = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 10.0], [1.0, 0.0, -10.0]])
        out = crop_obstacles(pts, obstacle_range=5.0, min_relative_z=-1.0, max_relative_z=1.0)
        assert out.shape == (1, 2)

    def test_drops_nan_inf(self) -> None:
        pts = np.array(
            [
                [1.0, 0.0, 0.0],
                [float("nan"), 0.0, 0.0],
                [1.0, float("inf"), 0.0],
            ]
        )
        out = crop_obstacles(pts, obstacle_range=5.0, min_relative_z=-1.0, max_relative_z=1.0)
        assert out.shape == (1, 2)

    def test_empty_input(self) -> None:
        out = crop_obstacles(
            np.zeros((0, 3)), obstacle_range=5.0, min_relative_z=-1.0, max_relative_z=1.0
        )
        assert out.shape == (0, 2)
        out2 = crop_obstacles(None, obstacle_range=5.0, min_relative_z=-1.0, max_relative_z=1.0)  # type: ignore[arg-type]
        assert out2.shape == (0, 2)

    def test_2d_input_ok(self) -> None:
        pts = np.array([[1.0, 0.0], [5.0, 0.0]])
        out = crop_obstacles(pts, obstacle_range=3.0, min_relative_z=-5.0, max_relative_z=5.0)
        assert out.shape == (1, 2)


# ─── score_candidate ──────────────────────────────────────────────────────


class TestScoreCandidate:
    def test_no_obstacles_not_blocked(self) -> None:
        arc = generate_arc(0.0, 3.0, 0.15)
        obs = np.zeros((0, 2))
        score, blocked, clr = score_candidate(
            arc, obs, goal_bearing=0.0, prev_kappa=0.0, kappa=0.0, **_SCORE_KW
        )
        assert not blocked
        assert clr == _SCORE_KW["clearance_cap"]
        # Score monotonic in alignment: straight arc + straight goal should
        # have a higher score than straight arc + 90° goal
        score_off, _, _ = score_candidate(
            arc, obs, goal_bearing=math.pi / 2, prev_kappa=0.0, kappa=0.0, **_SCORE_KW
        )
        assert score > score_off

    def test_obstacle_dead_ahead_blocks(self) -> None:
        arc = generate_arc(0.0, 3.0, 0.15)
        obs = np.array([[1.5, 0.0]])  # dead ahead, clearance 0
        _, blocked, _ = score_candidate(
            arc, obs, goal_bearing=0.0, prev_kappa=0.0, kappa=0.0, **_SCORE_KW
        )
        assert blocked

    def test_closer_obstacle_lower_clearance(self) -> None:
        arc = generate_arc(0.0, 3.0, 0.15)
        near = np.array([[1.5, 0.6]])
        far = np.array([[1.5, 2.0]])
        _, blocked_n, clr_n = score_candidate(
            arc, near, goal_bearing=0.0, prev_kappa=0.0, kappa=0.0, **_SCORE_KW
        )
        _, blocked_f, clr_f = score_candidate(
            arc, far, goal_bearing=0.0, prev_kappa=0.0, kappa=0.0, **_SCORE_KW
        )
        assert not blocked_n
        assert not blocked_f
        assert clr_n < clr_f

    def test_hysteresis_favors_prev(self) -> None:
        arc = generate_arc(0.3, 3.0, 0.15)
        obs = np.zeros((0, 2))
        sigma = _SCORE_KW["hysteresis_sigma"]
        score_near, _, _ = score_candidate(
            arc, obs, goal_bearing=0.0, prev_kappa=0.3, kappa=0.3, **_SCORE_KW
        )
        score_far, _, _ = score_candidate(
            arc, obs, goal_bearing=0.0, prev_kappa=0.3 + 3.0 * sigma, kappa=0.3, **_SCORE_KW
        )
        assert score_near > score_far


# ─── select_curvature ─────────────────────────────────────────────────────


class TestSelectCurvature:
    def _setup(self) -> tuple[np.ndarray, list[np.ndarray]]:
        kappas = np.linspace(-1.5, 1.5, 21)
        arcs = [generate_arc(float(k), 3.0, 0.15) for k in kappas]
        return kappas, arcs

    def test_straight_ahead_picks_zero(self) -> None:
        kappas, arcs = self._setup()
        raw, blocked, _ = select_curvature(
            np.zeros((0, 2)),
            goal_bearing=0.0,
            prev_kappa=0.0,
            kappa_candidates=kappas,
            arc_cache=arcs,
            **_SCORE_KW,
        )
        assert not blocked
        assert abs(raw) < 0.05

    def test_goal_left_picks_positive_kappa(self) -> None:
        kappas, arcs = self._setup()
        raw, blocked, _ = select_curvature(
            np.zeros((0, 2)),
            goal_bearing=math.pi / 4,  # 45° left
            prev_kappa=0.0,
            kappa_candidates=kappas,
            arc_cache=arcs,
            **_SCORE_KW,
        )
        assert not blocked
        assert raw > 0.0

    def test_wall_ahead_deflects(self) -> None:
        kappas, arcs = self._setup()
        # Wall segment dead-ahead 2m away, wide enough to block straight
        wall = np.array([[2.0, y] for y in np.linspace(-0.5, 0.5, 11)])
        raw, blocked, _ = select_curvature(
            wall,
            goal_bearing=0.0,
            prev_kappa=0.0,
            kappa_candidates=kappas,
            arc_cache=arcs,
            **_SCORE_KW,
        )
        assert not blocked
        assert abs(raw) > 0.1

    def test_all_blocked_holds_prev(self) -> None:
        kappas, arcs = self._setup()
        # Ring of obstacles within robot_radius of the origin
        angles = np.linspace(0.0, 2 * math.pi, 60, endpoint=False)
        ring = np.stack([0.2 * np.cos(angles), 0.2 * np.sin(angles)], axis=1)
        raw, blocked, _ = select_curvature(
            ring,
            goal_bearing=0.0,
            prev_kappa=0.7,
            kappa_candidates=kappas,
            arc_cache=arcs,
            **_SCORE_KW,
        )
        assert blocked
        assert raw == 0.7

    def test_hysteresis_breaks_tie(self) -> None:
        kappas, arcs = self._setup()
        # Goal dead ahead, no obstacles → score peaks at κ=0 but many
        # nearby candidates have near-equal score; hysteresis should
        # pull toward prev_kappa.
        raw_left, _, _ = select_curvature(
            np.zeros((0, 2)),
            goal_bearing=0.0,
            prev_kappa=-0.3,
            kappa_candidates=kappas,
            arc_cache=arcs,
            **_SCORE_KW,
        )
        raw_right, _, _ = select_curvature(
            np.zeros((0, 2)),
            goal_bearing=0.0,
            prev_kappa=0.3,
            kappa_candidates=kappas,
            arc_cache=arcs,
            **_SCORE_KW,
        )
        assert raw_left <= raw_right


# ─── update_ema ───────────────────────────────────────────────────────────


class TestUpdateEma:
    def test_alpha_one_returns_raw(self) -> None:
        assert update_ema(0.5, 0.9, 1.0) == 0.9

    def test_alpha_zero_returns_prev(self) -> None:
        assert update_ema(0.5, 0.9, 0.0) == 0.5

    def test_converges(self) -> None:
        v = 0.0
        for _ in range(50):
            v = update_ema(v, 0.7, 0.3)
        assert abs(v - 0.7) < 1e-3


# ─── goal_in_vehicle_frame ────────────────────────────────────────────────


class TestGoalInVehicleFrame:
    def test_identity_yaw(self) -> None:
        gx, gy = goal_in_vehicle_frame(3.0, 2.0, 0.0, 0.0, 0.0)
        assert abs(gx - 3.0) < 1e-9
        assert abs(gy - 2.0) < 1e-9

    def test_yaw_ninety(self) -> None:
        # World +x maps to vehicle -y when ryaw=+π/2 (robot faces +y)
        gx, gy = goal_in_vehicle_frame(1.0, 0.0, 0.0, 0.0, math.pi / 2)
        assert abs(gx) < 1e-9
        assert abs(gy - (-1.0)) < 1e-9

    def test_translation_subtracted(self) -> None:
        gx, gy = goal_in_vehicle_frame(5.0, 3.0, 2.0, 1.0, 0.0)
        assert abs(gx - 3.0) < 1e-9
        assert abs(gy - 2.0) < 1e-9


# ─── Integration (no LCM, no threads) ─────────────────────────────────────


class TestSmoothLocalPlannerIntegration:
    def test_ema_converges_to_constant_choice(self) -> None:
        p = _make_planner()
        # Goal hard left (90°), no obstacles → preferred kappa is
        # clearly positive (straight arc has alignment ≈ 0 at this bearing).
        for _ in range(25):
            raw, _, _ = select_curvature(
                np.zeros((0, 2)),
                goal_bearing=math.pi / 2,
                prev_kappa=p._prev_kappa_ema,
                kappa_candidates=p._kappa_candidates,
                arc_cache=p._arc_cache,
                **_SCORE_KW,
            )
            p._prev_kappa_ema = update_ema(p._prev_kappa_ema, raw, 0.3)
        # Converged toward the (positive) raw value
        assert p._prev_kappa_ema > 0.3

    def test_jitter_absorption(self) -> None:
        p = _make_planner()
        # Prime EMA at ~0.7 first.
        for _ in range(30):
            p._prev_kappa_ema = update_ema(p._prev_kappa_ema, 0.7, 0.3)
        # Now alternate a pair of nearby raw values; EMA deltas should stay small
        vals = [0.8, 0.6, 0.8, 0.6, 0.8, 0.6, 0.8, 0.6]
        last = p._prev_kappa_ema
        max_delta = 0.0
        for r in vals:
            new = update_ema(last, r, 0.3)
            max_delta = max(max_delta, abs(new - last))
            last = new
        assert max_delta < 0.07

    def test_all_blocked_shrinks_published_arc(self) -> None:
        p = _make_planner()
        p._prev_kappa_ema = 0.0
        # An all-blocked scenario: the arc length is shrunk
        length = p.config.publish_length * p.config.blocked_speed_scale
        arc = generate_arc(p._prev_kappa_ema, length, p.config.arc_step)
        # Last point must be within length * 1.05 of origin (straight)
        d = float(math.hypot(arc[-1, 0], arc[-1, 1]))
        assert d <= length * 1.05
