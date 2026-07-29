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

import json
import math

import pytest

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.perceive_loop_skill import PerceiveLoopSkill
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer
from dimos.simulation.dimsim.agentic_blueprint import unitree_go2_agentic_dimsim
from dimos.simulation.dimsim.go2_connection import DimSimGO2Connection
from dimos.simulation.dimsim.mcp_client import DimSimMcpClient
from dimos.simulation.dimsim.navigation_skill_container import (
    DimSimNavigationSkillContainer,
)
from dimos.simulation.dimsim.perceive_loop_skill import DimSimPerceiveLoopSkill
from dimos.simulation.dimsim.spatial_memory import DimSimSpatialMemory
from dimos.simulation.dimsim.unitree_skill_container import DimSimUnitreeSkillContainer


def _pose(ts: float, x: float = 0, y: float = 0, yaw_deg: float = 0) -> PoseStamped:
    return PoseStamped(
        ts=ts,
        position=Vector3(x, y, 0.5),
        orientation=Quaternion.from_euler(Vector3(0, 0, math.radians(yaw_deg))),
    )


@pytest.fixture
def container() -> DimSimUnitreeSkillContainer:
    skill_container = DimSimUnitreeSkillContainer()
    try:
        yield skill_container
    finally:
        skill_container.stop()


def test_dimsim_blueprint_preserves_full_upstream_agent_with_eval_adapters() -> None:
    active_classes = {atom.module for atom in unitree_go2_agentic_dimsim.active_blueprints}

    assert UnitreeSkillContainer not in active_classes
    assert DimSimUnitreeSkillContainer in active_classes
    assert McpClient not in active_classes
    assert DimSimMcpClient in active_classes
    assert NavigationSkillContainer not in active_classes
    assert DimSimNavigationSkillContainer in active_classes
    assert PerceiveLoopSkill not in active_classes
    assert DimSimPerceiveLoopSkill in active_classes
    assert SpatialMemory not in active_classes
    assert DimSimSpatialMemory in active_classes
    assert GO2Connection not in active_classes
    assert DimSimGO2Connection in active_classes

    mcp_client = next(
        atom
        for atom in unitree_go2_agentic_dimsim.active_blueprints
        if atom.module is DimSimMcpClient
    )
    assert "system_prompt" not in mcp_client.kwargs
    assert "allowed_tools" not in mcp_client.kwargs


def test_relative_move_schema_explains_camera_left_right_sign(container) -> None:
    skill = next(skill for skill in container.get_skills() if skill.func_name == "relative_move")
    description = json.loads(skill.args_schema)["description"]

    assert "object on the left side" in description
    assert "positive `left`" in description
    assert "object on the right side" in description
    assert "negative `left`" in description


def test_start_disposes_odometry_subscription(mocker, container) -> None:
    mocker.patch.object(UnitreeSkillContainer, "start")
    unsubscribe = mocker.Mock()
    mocker.patch.object(container.odom, "subscribe", return_value=unsubscribe)

    container.start()
    container.stop()

    unsubscribe.assert_called_once_with()


def test_relative_move_reports_measured_forward_progress(mocker, container) -> None:
    container._on_odom(_pose(1))
    mocker.patch.object(
        container,
        "_next_odom",
        side_effect=[
            _pose(2, x=0.2),
            _pose(3, x=0.4),
            _pose(4, x=0.6),
        ],
    )
    publish = mocker.patch.object(container.tele_cmd_vel, "publish")

    result = container.relative_move(forward=0.6)

    assert result == (
        "Movement completed: travelled 0.60m of the requested 0.60m along the "
        "requested direction (lateral drift 0.00m). Observe to verify the camera view."
    )
    assert publish.call_args_list[-1].args == (Twist.zero(),)
    assert all(call.args[0].linear.x > 0 for call in publish.call_args_list[:-5])
    assert all(call.args == (Twist.zero(),) for call in publish.call_args_list[-5:])


def test_translation_rate_limits_commands_during_queued_odometry(
    mocker,
    container,
) -> None:
    start = _pose(1)
    mocker.patch.object(
        container,
        "_next_odom",
        side_effect=[
            _pose(2, x=0.2),
            _pose(3, x=0.4),
            _pose(4, x=0.6),
        ],
    )
    fake_time = mocker.Mock()
    fake_time.monotonic.side_effect = [0, 0, 0, 0.02, 0.02, 0.04, 0.04]
    mocker.patch(
        "dimos.simulation.dimsim.unitree_skill_container.time",
        fake_time,
    )
    publish = mocker.patch.object(container.tele_cmd_vel, "publish")

    result = container._run_translation(start, 0.6, 0, 0.6)

    assert result.status == "completed"
    assert result.along_m == pytest.approx(0.6)
    publish.assert_called_once()


def test_relative_move_reports_collision_sliding_as_blocked(mocker, container) -> None:
    container._on_odom(_pose(1))
    mocker.patch.object(
        container,
        "_next_odom",
        side_effect=[_pose(ts, y=(ts - 1) * 0.02) for ts in range(2, 27)],
    )
    publish = mocker.patch.object(container.tele_cmd_vel, "publish")

    result = container.relative_move(forward=1.0)

    assert result == (
        "Movement was blocked or only partially completed: travelled 0.00m of the "
        "requested 1.00m along the requested direction (lateral drift 0.50m). "
        "Rotate toward another open route, then observe."
    )
    assert publish.call_args_list[-1].args == (Twist.zero(),)


def test_relative_rotation_tracks_progress_across_yaw_wrap(mocker, container) -> None:
    container._on_odom(_pose(1, yaw_deg=170))
    mocker.patch.object(
        container,
        "_next_odom",
        side_effect=[
            _pose(2, yaw_deg=179),
            _pose(3, yaw_deg=-175),
            _pose(4, yaw_deg=-160),
        ],
    )
    mocker.patch.object(
        container,
        "_settled_odom",
        return_value=_pose(5, yaw_deg=-160),
    )
    publish = mocker.patch.object(container.tele_cmd_vel, "publish")

    result = container.relative_move(degrees=30)

    assert result == (
        "Movement completed: turned 30.0 degrees of the requested 30.0 degrees. "
        "Observe to verify the camera view."
    )
    assert publish.call_args_list[-1].args == (Twist.zero(),)
    assert all(call.args[0].angular.z > 0 for call in publish.call_args_list[:-10])
    assert all(call.args == (Twist.zero(),) for call in publish.call_args_list[-10:])


def test_precise_rotation_corrects_settled_overshoot(mocker, container) -> None:
    start = _pose(1, yaw_deg=0)
    coarse = mocker.patch.object(
        container,
        "_run_rotation",
        return_value=object(),
    )
    mocker.patch.object(container, "_clear_velocity")
    mocker.patch.object(
        container,
        "_settled_odom",
        side_effect=[
            _pose(2, yaw_deg=40),
            _pose(3, yaw_deg=30),
        ],
    )

    result = container._run_precise_rotation(start, math.radians(30))

    assert result.status == "completed"
    assert math.degrees(result.radians) == pytest.approx(30)
    assert coarse.call_count == 2
    first, correction = coarse.call_args_list
    assert first.args == (start, pytest.approx(math.radians(30)), 0.35)
    assert correction.args[0] == _pose(2, yaw_deg=40)
    assert correction.args[1] == pytest.approx(math.radians(-10))
    assert correction.args[2] == 0.08


def test_relative_move_stops_velocity_when_motion_raises(mocker, container) -> None:
    container._on_odom(_pose(1))
    mocker.patch.object(container, "_run_translation", side_effect=RuntimeError("failed"))
    publish = mocker.patch.object(container.tele_cmd_vel, "publish")
    mocker.patch("dimos.simulation.dimsim.unitree_skill_container.time.sleep")

    with pytest.raises(RuntimeError, match="failed"):
        container.relative_move(forward=0.5)

    assert len(publish.call_args_list) == 5
    assert all(call.args == (Twist.zero(),) for call in publish.call_args_list)


def test_relative_move_requires_fresh_odometry(mocker, container) -> None:
    mocker.patch.object(container, "_fresh_odom", return_value=None)
    publish = mocker.patch.object(container.tele_cmd_vel, "publish")

    result = container.relative_move(forward=0.5)

    assert result == "Movement not started: no fresh DimSim odometry is available."
    publish.assert_not_called()


@pytest.mark.parametrize(
    ("forward", "degrees", "expected"),
    [
        (
            1.01,
            0,
            "Movement rejected: request 1.00m or less per call, then observe.",
        ),
        (
            0,
            61,
            "Movement rejected: request at most 60 degrees per call, then observe.",
        ),
    ],
)
def test_relative_move_rejects_unbounded_commands(
    container,
    forward,
    degrees,
    expected,
) -> None:
    assert container.relative_move(forward=forward, degrees=degrees) == expected


def test_odom_cache_rejects_out_of_order_samples(container) -> None:
    newest = _pose(2, x=1)
    stale = _pose(1, x=9)

    container._on_odom(newest)
    container._on_odom(stale)

    assert container._latest_odom is newest


def test_relative_move_rejects_non_finite_values(container) -> None:
    with pytest.raises(ValueError, match="must be finite"):
        container.relative_move(forward=float("nan"))
