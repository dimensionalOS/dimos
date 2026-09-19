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

"""Offline tests for the TypeSafe policy: no API key, no simulator."""

from __future__ import annotations

import inspect
import json
import math
from pathlib import Path
from typing import Any

import pytest
from typesafe_sdk import Choice, ChoiceAnswer, Noul

from dimos.evals.agents.typesafe_policy import (
    STEP_CRITERIA,
    STEPS,
    TypeSafePolicy,
    build_questions,
    derive_goal_label,
    load_scene,
    scene_labels,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

SHIPPED_SCENE = Path(__file__).parents[1] / "suites" / "scenes" / "apartment_detections.json"


def _det(
    id_: str, label: str, cx: float, cy: float, cz: float, sx: float, sy: float, sz: float
) -> dict[str, Any]:
    """One entry in the DimSim detections schema (PR #4208)."""
    return {
        "id": id_,
        "label": label,
        "score": 1.0,
        "center_xyz": [cx, cy, cz],
        "size_xyz": [sx, sy, sz],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


SCENE = {
    "frame_id": "world",
    "timestamp": 0.0,
    "count": 6,
    "detections": [
        _det("couch", "Modern L-shaped sectional", 1.0, 4.0, 0.4, 2.0, 1.0, 0.8),  # goal
        _det("table", "Coffee table", 3.0, 0.0, 0.25, 1.0, 1.0, 0.5),
        _det("plate", "Dinner plate", 3.0, 0.0, 0.52, 0.2, 0.2, 0.02),  # on the table
        _det("header", "wall-south-header", 5.0, 0.0, 2.7, 0.2, 1.0, 0.9),  # overhead
        _det("wall-n", "wall-north", -5.0, 0.0, 1.5, 0.2, 12.0, 3.0),
        _det("wall-s", "wall-south", 5.0, 0.0, 1.5, 0.2, 12.0, 3.0),
    ],
}


@pytest.fixture
def scene_json(tmp_path: Path) -> Path:
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(SCENE))
    return path


def _pose(yaw_rad: float) -> PoseStamped:
    return PoseStamped(
        position=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2)),
        frame_id="world",
    )


def _choice(key: str, confidence: float = 0.9) -> ChoiceAnswer:
    return ChoiceAnswer(choice=key, confidence=confidence, probabilities={key: confidence})


def test_steps_and_criteria_agree() -> None:
    """Every option the model can pick maps to a vector, and vice versa."""
    assert set(STEPS) == set(STEP_CRITERIA)


def test_criteria_carry_no_compass_words() -> None:
    """Wall labels use a different compass; the step wording must not."""
    for text in STEP_CRITERIA.values():
        assert not any(word in text.lower() for word in ("north", "south", "east", "west"))


def test_questions_are_one_choice_and_one_noul() -> None:
    questions = build_questions()
    assert isinstance(questions["step"], Choice)
    assert isinstance(questions["reached"], Noul)


# --- scene loading ---------------------------------------------------------------


def test_scene_drops_overhead_and_contained_boxes(scene_json: Path) -> None:
    """The door header passes over the robot; the plate sits inside the table."""
    scene = load_scene(scene_json, "sectional")
    assert [o.label for o in scene.obstacles] == [
        "Modern L-shaped sectional",
        "Coffee table",
        "wall-north",
        "wall-south",
    ]


def test_scene_goal_box_and_room_bounds(scene_json: Path) -> None:
    scene = load_scene(scene_json, "sectional")
    assert scene.goal_label == "Modern L-shaped sectional"
    assert scene.goal_xy == (1.0, 4.0)
    assert scene.goal_box == (0.0, 3.5, 2.0, 4.5)
    assert scene.room_bounds == (-5.1, -6.0, 5.1, 6.0)  # from the walls, not the furniture


def test_scene_unknown_goal_raises(scene_json: Path) -> None:
    with pytest.raises(LookupError, match="hot tub"):
        load_scene(scene_json, "hot tub")


def test_shipped_apartment_scene_is_ground_truth() -> None:
    """Pins the copied PR #4208 snapshot and the filter: 15 walls + 33 objects.

    Of 107 detections, 43 are overhead (headers, wall cabinets, the TV) and 16
    are contained footprints (shelf and tabletop clutter, bedding, and one
    dining chair tucked fully under its table).
    """
    scene = load_scene(SHIPPED_SCENE, "sectional")
    labels = [o.label for o in scene.obstacles]
    assert sum(label.startswith(("wall", "yard")) for label in labels) == 15
    assert len(labels) == 48
    assert scene.goal_box == (0.156, 3.028, 1.956, 5.735)
    assert not any("header" in label for label in labels)


def test_observe_merges_static_scene_with_live_pose(scene_json: Path) -> None:
    policy = TypeSafePolicy(scene_json=scene_json)
    policy._pose = _pose(math.radians(90.0))
    _, state = policy.observe(load_scene(scene_json, "sectional"), tick=3)
    assert state.goal_box == (0.0, 3.5, 2.0, 4.5)
    assert state.robot_yaw_deg == pytest.approx(90.0)
    assert state.ticks_elapsed == 3
    assert "wall-north" in json.dumps(state.encode())


# --- controller ------------------------------------------------------------------
# The pick is the command. Nothing about heading is computed in code.


def test_forward_and_backward_scale_linear_x(scene_json: Path) -> None:
    policy = TypeSafePolicy(scene_json=scene_json)
    fwd, back = policy.twist(_choice("1,0")), policy.twist(_choice("-1,0"))
    assert (fwd.linear.x, fwd.angular.z) == pytest.approx((0.2, 0.0))
    assert (back.linear.x, back.angular.z) == pytest.approx((-0.2, 0.0))
    assert fwd.linear.y == 0.0  # DimSim ignores it anyway


def test_turns_scale_angular_z_with_the_ros_sign(scene_json: Path) -> None:
    """+angular.z is counter-clockwise (left) in ROS and in DimSim's physics."""
    policy = TypeSafePolicy(scene_json=scene_json)
    left, right = policy.twist(_choice("0,1")), policy.twist(_choice("0,-1"))
    assert (left.linear.x, left.angular.z) == pytest.approx((0.0, 0.5))
    assert (right.linear.x, right.angular.z) == pytest.approx((0.0, -0.5))


def test_stop_is_zero(scene_json: Path) -> None:
    twist = TypeSafePolicy(scene_json=scene_json).twist(_choice("0,0"))
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)


def test_twist_takes_no_pose(scene_json: Path) -> None:
    """Guards the contract: the command must not depend on the robot's heading."""
    assert list(inspect.signature(TypeSafePolicy.twist).parameters) == ["self", "step"]


def test_pick_is_applied_whatever_the_confidence(scene_json: Path) -> None:
    """No threshold: the argmax is the command. Only "0,0" stops the robot."""
    policy = TypeSafePolicy(scene_json=scene_json)
    twist = policy.twist(_choice("1,0", confidence=0.21))
    assert (twist.linear.x, twist.angular.z) == pytest.approx((0.2, 0.0))


# --- preflight -------------------------------------------------------------------


def test_preflight_rejects_modules(scene_json: Path) -> None:
    """The policy calls no tools, so it must never pull in a skill container."""
    policy = TypeSafePolicy(scene_json=scene_json, modules=("unitree-skill-container",))
    with pytest.raises(ValueError, match="no tools"):
        policy.preflight(None)  # type: ignore[arg-type]


def test_preflight_rejects_missing_scene(tmp_path: Path) -> None:
    policy = TypeSafePolicy(scene_json=tmp_path / "nope.json")
    with pytest.raises(FileNotFoundError):
        policy.preflight(None)  # type: ignore[arg-type]


def test_preflight_rejects_unknown_goal(scene_json: Path) -> None:
    policy = TypeSafePolicy(scene_json=scene_json, goal_label="jacuzzi")
    with pytest.raises(LookupError):
        policy.preflight(None)  # type: ignore[arg-type]


# --- goal from the instruction ---------------------------------------------------


def test_derive_goal_label_picks_the_word_that_names_an_object(scene_json: Path) -> None:
    labels = scene_labels(scene_json)
    assert derive_goal_label("navigate to the sectional couch", labels) == "sectional"
    assert derive_goal_label("go to the bathtub", scene_labels(SHIPPED_SCENE)) == "bathtub"


def test_derive_goal_label_ignores_stop_and_short_words(scene_json: Path) -> None:
    """'with' occurs inside labels ("with chrome") but never names a goal."""
    with pytest.raises(LookupError, match="names a scene object"):
        derive_goal_label("go with the flow", scene_labels(SHIPPED_SCENE))


def test_load_scene_rejects_an_empty_goal(scene_json: Path) -> None:
    with pytest.raises(ValueError, match="matches everything"):
        load_scene(scene_json, "")


def test_preflight_without_goal_label_only_checks_the_file(scene_json: Path) -> None:
    TypeSafePolicy(scene_json=scene_json).preflight(None)  # type: ignore[arg-type]


def test_wiring_follows_the_environment(scene_json: Path) -> None:
    from dimos.evals.agents.typesafe_policy import DIMSIM_WIRING, HABITAT_WIRING, Wiring
    from dimos.evals.environments.dimsim import DimSimEnvironment
    from dimos.evals.environments.habitat import HabitatEnvironment

    dimsim = DimSimEnvironment(blueprint=["unitree-go2", "mcp-server"])
    habitat = HabitatEnvironment(blueprint=["habitat-teleop", "mcp-server"])
    agent = TypeSafePolicy(scene_json=scene_json)
    agent.preflight(dimsim)
    assert agent._wiring == DIMSIM_WIRING
    agent.preflight(habitat)
    assert agent._wiring == HABITAT_WIRING

    explicit = TypeSafePolicy(scene_json=scene_json, cmd_topic="/twist", speed_scale=1.0)
    explicit.preflight(habitat)
    assert explicit._wiring == Wiring("/odometry", "Odometry", "/twist", 1.0)


def test_odometry_becomes_the_pose(scene_json: Path) -> None:
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.nav_msgs.Odometry import Odometry

    agent = TypeSafePolicy(scene_json=scene_json)
    odom = Odometry(ts=5.0, frame_id="world", pose=Pose(position=(1.0, 2.0, 0.0)))
    agent._on_odometry(odom)
    assert agent._pose_seen.is_set()
    assert agent._pose is not None
    assert tuple(agent._pose.position) == (1.0, 2.0, 0.0)
    assert (agent._pose.frame_id, agent._pose.ts) == ("world", 5.0)


def test_habitat_scales_commands_to_match_dimsim(scene_json: Path) -> None:
    from dimos.evals.agents.typesafe_policy import HABITAT_WIRING

    agent = TypeSafePolicy(scene_json=scene_json)
    agent._wiring = HABITAT_WIRING
    forward = agent.twist(_choice("1,0"))
    left = agent.twist(_choice("0,1"))
    assert forward.linear.x == pytest.approx(3.0 * agent.config.speed)
    assert left.angular.z == pytest.approx(3.0 * agent.config.turn_rate)
