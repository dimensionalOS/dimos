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

"""Unit tests for ``dimos.manipulation.eval`` — no Drake required.

Covers the parts of the eval that can be exercised without spinning up a
DrakeWorld: data types, the ``reach_case`` constructor, the per-arm case
registry, and the pure reporting helpers (``print_scores``, ``to_json``).

The integration test that actually runs IK + RRT against a Drake world
lives in ``test_planner_eval.py``.
"""

from __future__ import annotations

import io
import json
import sys

import pytest

from dimos.manipulation.eval import (
    Case,
    Score,
    cases_for,
    default_cases,
    print_scores,
    reach_case,
    to_json,
)


# ── Case + Score data types ───────────────────────────────────────────────────


class TestCase:
    """Construction and defaults for the Case dataclass."""

    def test_required_only(self):
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        case = Case(name="probe", target=PoseStamped())
        assert case.name == "probe"
        assert case.obstacles == []
        assert case.start_joints is None
        assert case.position_tolerance_m == 0.01
        assert case.orientation_tolerance_rad == 0.1

    def test_with_start_joints(self):
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        case = Case(name="x", target=PoseStamped(), start_joints=[0.1, 0.2, 0.3])
        assert case.start_joints == [0.1, 0.2, 0.3]


class TestScore:
    """Construction and defaults for the Score dataclass."""

    def test_required_only(self):
        s = Score(name="x", passed=True)
        assert s.name == "x"
        assert s.passed is True
        assert s.reason == ""
        assert s.metrics == {}

    def test_with_metrics(self):
        s = Score(name="x", passed=True, metrics={"planning_time_s": 0.1})
        assert s.metrics["planning_time_s"] == 0.1

    def test_failure_carries_reason(self):
        s = Score(name="x", passed=False, reason="ik failed: NO_SOLUTION")
        assert s.passed is False
        assert "NO_SOLUTION" in s.reason


# ── reach_case helper ─────────────────────────────────────────────────────────


class TestReachCase:
    """The convenience constructor for single-reach cases."""

    def test_position(self):
        c = reach_case("p", 0.1, 0.2, 0.3)
        assert c.target.position.x == pytest.approx(0.1)
        assert c.target.position.y == pytest.approx(0.2)
        assert c.target.position.z == pytest.approx(0.3)

    def test_default_orientation_is_xarm_gripper_down(self):
        # xArm gripper-down quaternion is (1, 0, 0, 0) in (x, y, z, w) order
        c = reach_case("p", 0.0, 0.0, 0.0)
        assert c.target.orientation.x == pytest.approx(1.0)
        assert c.target.orientation.y == pytest.approx(0.0)
        assert c.target.orientation.z == pytest.approx(0.0)
        assert c.target.orientation.w == pytest.approx(0.0)

    def test_custom_orientation(self):
        c = reach_case("p", 0.0, 0.0, 0.0, orientation=(0.0, 0.0, 0.0, 1.0))
        assert c.target.orientation.w == pytest.approx(1.0)

    def test_default_no_obstacles(self):
        c = reach_case("p", 0.0, 0.0, 0.0)
        assert c.obstacles == []

    def test_with_obstacles(self):
        from dimos.manipulation.planning.spec.enums import ObstacleType
        from dimos.manipulation.planning.spec.models import Obstacle
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        obs = Obstacle(
            name="box1",
            obstacle_type=ObstacleType.BOX,
            pose=PoseStamped(),
            dimensions=(0.1, 0.1, 0.1),
        )
        c = reach_case("p", 0.0, 0.0, 0.0, obstacles=[obs])
        assert len(c.obstacles) == 1
        assert c.obstacles[0].name == "box1"

    def test_custom_tolerances(self):
        c = reach_case(
            "p",
            0.0,
            0.0,
            0.0,
            position_tolerance_m=0.005,
            orientation_tolerance_rad=0.05,
        )
        assert c.position_tolerance_m == 0.005
        assert c.orientation_tolerance_rad == 0.05

    def test_frame_id_is_world(self):
        c = reach_case("p", 0.0, 0.0, 0.0)
        assert c.target.frame_id == "world"


# ── case registry ─────────────────────────────────────────────────────────────


class TestCasesFor:
    """The per-arm case dispatch and its built-in registrations."""

    def test_xarm6_full_suite(self):
        cases = cases_for("xarm6")
        names = [c.name for c in cases]
        assert "reach_center" in names
        assert "reach_obstacle_box" in names
        assert len(cases) == 7

    def test_xarm7_drops_obstacle_case(self):
        # xArm7 is the xArm6 suite minus the obstacle case (extra DOF causes
        # COLLISION_AT_GOAL).
        names = {c.name for c in cases_for("xarm7")}
        assert "reach_obstacle_box" not in names
        assert "reach_center" in names

    def test_piper_uses_pitch90_orientation(self):
        # Piper's EE link points +X at zero config; pitch=90° quat puts the
        # gripper down. Quaternion (x, y, z, w) ≈ (0, 0.7071, 0, 0.7071).
        cases = cases_for("piper")
        assert len(cases) == 6
        first = cases[0]
        assert first.target.orientation.y == pytest.approx(0.7071, abs=1e-3)
        assert first.target.orientation.w == pytest.approx(0.7071, abs=1e-3)

    def test_unknown_arm_raises_value_error(self):
        with pytest.raises(ValueError, match="no registered cases"):
            cases_for("not_a_real_arm")

    def test_default_cases_is_xarm6(self):
        assert [c.name for c in default_cases()] == [c.name for c in cases_for("xarm6")]

    def test_obstacle_case_carries_one_obstacle(self):
        obstacle_case = next(c for c in cases_for("xarm6") if c.name == "reach_obstacle_box")
        assert len(obstacle_case.obstacles) == 1
        assert obstacle_case.obstacles[0].name == "blocker"


# ── reporting ─────────────────────────────────────────────────────────────────


def _capture_stdout(fn, *args, **kwargs) -> str:
    buf = io.StringIO()
    old = sys.stdout
    try:
        sys.stdout = buf
        fn(*args, **kwargs)
    finally:
        sys.stdout = old
    return buf.getvalue()


class TestPrintScores:
    """Terminal-table rendering of scores."""

    def test_empty_scores(self):
        out = _capture_stdout(print_scores, [])
        assert "0 cases" in out
        assert "0/0 passed" in out

    def test_passing_score(self):
        s = Score(
            name="reach_x",
            passed=True,
            metrics={"planning_time_s": 0.5, "position_error_m": 0.001},
        )
        out = _capture_stdout(print_scores, [s])
        assert "✓" in out
        assert "reach_x" in out
        assert "1/1 passed" in out
        assert "500ms" in out
        assert "err=1.0mm" in out

    def test_failing_score_shows_reason(self):
        s = Score(name="reach_x", passed=False, reason="ik failed: NO_SOLUTION")
        out = _capture_stdout(print_scores, [s])
        assert "✗" in out
        assert "ik failed: NO_SOLUTION" in out
        assert "0/1 passed" in out

    def test_mixed_results(self):
        scores = [
            Score(name="a", passed=True),
            Score(name="b", passed=False, reason="boom"),
            Score(name="c", passed=True),
        ]
        out = _capture_stdout(print_scores, scores)
        assert "2/3 passed" in out

    def test_velocity_ratio_rendered(self):
        s = Score(name="x", passed=True, metrics={"max_joint_velocity_ratio": 0.85})
        out = _capture_stdout(print_scores, [s])
        assert "v=85%" in out

    def test_no_velocity_ratio_when_absent(self):
        s = Score(name="x", passed=True, metrics={"planning_time_s": 0.1})
        out = _capture_stdout(print_scores, [s])
        assert "v=" not in out


class TestToJson:
    """JSON serialization of scores."""

    def test_round_trip(self, tmp_path):
        path = tmp_path / "out.json"
        scores = [
            Score(name="a", passed=True, metrics={"x": 1.5, "y": 2.5}),
            Score(name="b", passed=False, reason="rejected", metrics={"x": 0.0}),
        ]
        to_json(scores, str(path))
        loaded = json.loads(path.read_text())

        assert len(loaded) == 2
        assert loaded[0]["name"] == "a"
        assert loaded[0]["passed"] is True
        assert loaded[0]["reason"] == ""
        assert loaded[0]["metrics"]["x"] == 1.5
        assert loaded[1]["passed"] is False
        assert loaded[1]["reason"] == "rejected"

    def test_empty_scores(self, tmp_path):
        path = tmp_path / "empty.json"
        to_json([], str(path))
        assert json.loads(path.read_text()) == []

    def test_preserves_metric_keys(self, tmp_path):
        path = tmp_path / "out.json"
        scores = [
            Score(
                name="x",
                passed=True,
                metrics={
                    "planning_time_s": 0.5,
                    "position_error_m": 0.001,
                    "max_joint_velocity_ratio": 1.0,
                },
            )
        ]
        to_json(scores, str(path))
        loaded = json.loads(path.read_text())
        assert set(loaded[0]["metrics"].keys()) == {
            "planning_time_s",
            "position_error_m",
            "max_joint_velocity_ratio",
        }
