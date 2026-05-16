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
DrakeWorld: data types, the ``reach_scenario`` constructor, the per-arm scenario
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
    Scenario,
    Score,
    scenarios_for,
    default_scenarios,
    print_scores,
    reach_scenario,
    to_json,
)


# ── Scenario + Score data types ───────────────────────────────────────────────────


class TestCase:
    """Construction and defaults for the Scenario dataclass."""

    def test_required_only(self):
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        scenario = Scenario(name="probe", target=PoseStamped())
        assert scenario.name == "probe"
        assert scenario.obstacles == []
        assert scenario.start_joints is None
        assert scenario.position_tolerance_m == 0.01
        assert scenario.orientation_tolerance_rad == 0.1

    def test_with_start_joints(self):
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        scenario = Scenario(name="x", target=PoseStamped(), start_joints=[0.1, 0.2, 0.3])
        assert scenario.start_joints == [0.1, 0.2, 0.3]


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


# ── reach_scenario helper ─────────────────────────────────────────────────────────


class TestReachCase:
    """The convenience constructor for single-reach scenarios."""

    def test_position(self):
        c = reach_scenario("p", 0.1, 0.2, 0.3)
        assert c.target.position.x == pytest.approx(0.1)
        assert c.target.position.y == pytest.approx(0.2)
        assert c.target.position.z == pytest.approx(0.3)

    def test_default_orientation_is_xarm_gripper_down(self):
        # xArm gripper-down quaternion is (1, 0, 0, 0) in (x, y, z, w) order
        c = reach_scenario("p", 0.0, 0.0, 0.0)
        assert c.target.orientation.x == pytest.approx(1.0)
        assert c.target.orientation.y == pytest.approx(0.0)
        assert c.target.orientation.z == pytest.approx(0.0)
        assert c.target.orientation.w == pytest.approx(0.0)

    def test_custom_orientation(self):
        c = reach_scenario("p", 0.0, 0.0, 0.0, orientation=(0.0, 0.0, 0.0, 1.0))
        assert c.target.orientation.w == pytest.approx(1.0)

    def test_default_no_obstacles(self):
        c = reach_scenario("p", 0.0, 0.0, 0.0)
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
        c = reach_scenario("p", 0.0, 0.0, 0.0, obstacles=[obs])
        assert len(c.obstacles) == 1
        assert c.obstacles[0].name == "box1"

    def test_custom_tolerances(self):
        c = reach_scenario(
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
        c = reach_scenario("p", 0.0, 0.0, 0.0)
        assert c.target.frame_id == "world"


# ── scenario registry ─────────────────────────────────────────────────────────────


class TestCasesFor:
    """The per-arm scenario dispatch and its built-in registrations."""

    def test_xarm6_includes_all_sections(self):
        # xArm6 suite includes basic reaches, harder geometry, and a
        # pick-and-place sequence.
        names = [c.name for c in scenarios_for("xarm6")]
        assert "reach_center" in names
        assert "reach_obstacle_box" in names
        assert "reach_corner_far_right" in names
        assert "tight_clearance_passage" in names
        assert "pp_left_to_right_pick" in names

    def test_xarm7_drops_known_failing_cases(self):
        # xArm7 drops scenarios known to COLLISION_AT_GOAL with the 7-DOF arm.
        # Free-space reaches, corners, and the pick-and-place sequence
        # (which carries only a small work-surface obstacle) all still apply.
        names = {c.name for c in scenarios_for("xarm7")}
        assert "reach_obstacle_box" not in names
        assert "tight_clearance_passage" not in names
        assert "reach_center" in names
        assert "reach_corner_far_right" in names
        assert "pp_left_to_right_pick" in names

    def test_piper_uses_pitch90_orientation(self):
        # Piper's EE link points +X at zero config; pitch=90° quat puts the
        # gripper down. Quaternion (x, y, z, w) ≈ (0, 0.7071, 0, 0.7071).
        scenarios = scenarios_for("piper")
        assert len(scenarios) == 6
        first = scenarios[0]
        assert first.target.orientation.y == pytest.approx(0.7071, abs=1e-3)
        assert first.target.orientation.w == pytest.approx(0.7071, abs=1e-3)

    def test_unknown_arm_raises_value_error(self):
        with pytest.raises(ValueError, match="no registered scenarios"):
            scenarios_for("not_a_real_arm")

    def test_default_cases_is_xarm6(self):
        assert [c.name for c in default_scenarios()] == [c.name for c in scenarios_for("xarm6")]

    def test_obstacle_case_carries_one_obstacle(self):
        obstacle_case = next(c for c in scenarios_for("xarm6") if c.name == "reach_obstacle_box")
        assert len(obstacle_case.obstacles) == 1
        assert obstacle_case.obstacles[0].name == "blocker"

    def test_tight_clearance_carries_two_obstacles(self):
        scenario = next(c for c in scenarios_for("xarm6") if c.name == "tight_clearance_passage")
        assert len(scenario.obstacles) == 2
        names = {o.name for o in scenario.obstacles}
        assert names == {"corridor_left", "corridor_right"}


class TestPickPlaceCases:
    """The pick_place_scenarios() sequence builder."""

    def test_returns_six_cases(self):
        from dimos.manipulation.eval_scenarios import pick_place_scenarios

        scenarios = pick_place_scenarios("test", pick=(0.3, 0.1, 0.1), place=(0.3, -0.1, 0.1))
        assert len(scenarios) == 6

    def test_names_follow_pre_target_post_pattern(self):
        from dimos.manipulation.eval_scenarios import pick_place_scenarios

        scenarios = pick_place_scenarios("foo", pick=(0.3, 0.1, 0.1), place=(0.3, -0.1, 0.1))
        names = [c.name for c in scenarios]
        assert names == [
            "foo_pre_pick",
            "foo_pick",
            "foo_post_pick",
            "foo_pre_place",
            "foo_place",
            "foo_post_place",
        ]

    def test_pre_post_have_approach_height_offset(self):
        from dimos.manipulation.eval_scenarios import pick_place_scenarios

        scenarios = pick_place_scenarios(
            "x", pick=(0.3, 0.2, 0.10), place=(0.3, -0.2, 0.10), approach_height_m=0.07
        )
        # pre_pick / post_pick / pre_place / post_place are 7cm above their target
        assert scenarios[0].target.position.z == pytest.approx(0.17)  # pre_pick
        assert scenarios[1].target.position.z == pytest.approx(0.10)  # pick
        assert scenarios[2].target.position.z == pytest.approx(0.17)  # post_pick
        assert scenarios[3].target.position.z == pytest.approx(0.17)  # pre_place
        assert scenarios[4].target.position.z == pytest.approx(0.10)  # place
        assert scenarios[5].target.position.z == pytest.approx(0.17)  # post_place

    def test_pick_and_place_x_y_independent(self):
        from dimos.manipulation.eval_scenarios import pick_place_scenarios

        scenarios = pick_place_scenarios("x", pick=(0.4, 0.2, 0.1), place=(0.2, -0.3, 0.1))
        # First three scenarios use pick xy; last three use place xy
        for c in scenarios[:3]:
            assert c.target.position.x == pytest.approx(0.4)
            assert c.target.position.y == pytest.approx(0.2)
        for c in scenarios[3:]:
            assert c.target.position.x == pytest.approx(0.2)
            assert c.target.position.y == pytest.approx(-0.3)

    def test_default_orientation_is_xarm_gripper_down(self):
        from dimos.manipulation.eval_scenarios import pick_place_scenarios

        scenarios = pick_place_scenarios("x", pick=(0.3, 0, 0.1), place=(0.3, 0, 0.1))
        assert scenarios[0].target.orientation.x == pytest.approx(1.0)
        assert scenarios[0].target.orientation.w == pytest.approx(0.0)

    def test_custom_orientation_propagates(self):
        from dimos.manipulation.eval_scenarios import pick_place_scenarios

        q = (0.0, 0.7071, 0.0, 0.7071)
        scenarios = pick_place_scenarios("x", pick=(0.3, 0, 0.1), place=(0.3, 0, 0.1), orientation=q)
        for c in scenarios:
            assert c.target.orientation.y == pytest.approx(0.7071, abs=1e-3)
            assert c.target.orientation.w == pytest.approx(0.7071, abs=1e-3)

    def test_no_obstacles_by_default(self):
        from dimos.manipulation.eval_scenarios import pick_place_scenarios

        scenarios = pick_place_scenarios("x", pick=(0.3, 0, 0.1), place=(0.3, 0, 0.1))
        for c in scenarios:
            assert c.obstacles == []


class TestGraspHeuristicHelpers:
    """Helpers that pull in PickAndPlaceModule's production grasp heuristics."""

    def test_grasp_scenario_uses_object_top_for_z(self):
        from dimos.manipulation.eval import grasp_scenario

        s = grasp_scenario(
            "x", object_at=(0.30, 0.10, 0.06), object_size=(0.04, 0.04, 0.10)
        )
        # grasp z is the top of the object — center.z + size.z / 2
        assert s.target.position.z == pytest.approx(0.06 + 0.05)

    def test_grasp_scenario_shifts_toward_robot(self):
        from dimos.manipulation.eval import grasp_scenario

        # PickAndPlaceModule._occlusion_offset shifts grasp xy toward the
        # robot base. So for an object at +x, grasp x should be smaller.
        s = grasp_scenario(
            "x", object_at=(0.40, 0.0, 0.06), object_size=(0.05, 0.05, 0.10)
        )
        assert s.target.position.x < 0.40

    def test_pick_place_for_object_returns_six_scenarios(self):
        from dimos.manipulation.eval_scenarios import pick_place_for_object

        scenarios = pick_place_for_object(
            "x",
            object_at=(0.30, 0.10, 0.06),
            object_size=(0.04, 0.04, 0.10),
            place_at=(0.30, -0.10, 0.08),
        )
        assert len(scenarios) == 6
        assert scenarios[0].name == "x_pre_pick"
        assert scenarios[-1].name == "x_post_place"

    def test_pick_place_for_object_pick_uses_heuristic(self):
        from dimos.manipulation.eval_scenarios import pick_place_for_object

        scenarios = pick_place_for_object(
            "x",
            object_at=(0.40, 0.0, 0.06),
            object_size=(0.05, 0.05, 0.10),
            place_at=(0.30, -0.10, 0.08),
        )
        # Pick xy should be shifted from the object xy via _occlusion_offset
        assert scenarios[1].target.position.x < 0.40

    def test_pick_place_for_object_propagates_obstacles(self):
        from dimos.manipulation.eval_scenarios import _box, pick_place_for_object

        surface = _box("table", (0.35, 0.0, 0.02), (0.30, 0.40, 0.02))
        scenarios = pick_place_for_object(
            "x",
            object_at=(0.30, 0.10, 0.06),
            object_size=(0.04, 0.04, 0.10),
            place_at=(0.30, -0.10, 0.08),
            obstacles=[surface],
        )
        for s in scenarios:
            assert len(s.obstacles) == 1
            assert s.obstacles[0].name == "table"


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
        assert "0 scenarios" in out
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
