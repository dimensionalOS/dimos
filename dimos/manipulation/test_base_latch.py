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

"""Latching the planning base onto the robot's actual pose."""

from __future__ import annotations

from collections.abc import Iterator
import math
from pathlib import Path
from typing import Any, Protocol, cast
from unittest.mock import MagicMock

import pytest

from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
    ManipulationState,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def _robot_config(name: str = "g1", base_z: float = 0.74) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(0.0, 0.0, base_z), orientation=Quaternion()),
        joint_names=["joint1", "joint2"],
        base_link="pelvis",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=("joint1", "joint2"),
                base_link="pelvis",
                tip_link="left_hand",
            )
        ],
    )


def _pose(x: float, y: float, z: float = 0.74, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        frame_id="world",
        position=Vector3(x, y, z),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


class _StubWorldMonitor:
    """World monitor stand-in that re-places robots without a real scene.

    Like the real world, it replaces the stored config rather than mutating it,
    so the module has to re-read the placement after a latch.
    """

    def __init__(self, configs: dict[str, RobotModelConfig]) -> None:
        self._configs = dict(configs)
        self.calls: list[tuple[str, PoseStamped]] = []
        self.obstacle_poses: dict[str, PoseStamped] = {}
        self.rebuild_seconds = 1.5
        self.failure: Exception | None = None

    def set_robot_base_pose(self, robot_id: str, pose: PoseStamped) -> float:
        if self.failure is not None:
            raise self.failure
        self.calls.append((robot_id, pose))
        self._configs[robot_id] = self._configs[robot_id].model_copy(
            update={"base_pose": PoseStamped(pose)}
        )
        return self.rebuild_seconds

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        return self._configs[robot_id]

    def update_obstacle_pose(self, name: str, pose: PoseStamped) -> bool:
        self.obstacle_poses[name] = pose
        return True

    def stop_all_monitors(self) -> None:
        return None


class ModuleFactory(Protocol):
    def __call__(
        self, *configs: RobotModelConfig, **config_kwargs: Any
    ) -> tuple[ManipulationModule, _StubWorldMonitor]: ...


@pytest.fixture
def make_module() -> Iterator[ModuleFactory]:
    modules: list[ManipulationModule] = []

    def create(
        *configs: RobotModelConfig, **config_kwargs: Any
    ) -> tuple[ManipulationModule, _StubWorldMonitor]:
        robots = list(configs) or [_robot_config()]
        ids = {f"robot_{index + 1}": config for index, config in enumerate(robots)}
        monitor = _StubWorldMonitor(ids)
        module = ManipulationModule()
        modules.append(module)
        module.config = ManipulationModuleConfig(robots=robots, **config_kwargs)
        module._control_coordinator = MagicMock()
        module._world_monitor = cast("Any", monitor)
        module._robots = {config.name: (robot_id, config) for robot_id, config in ids.items()}
        module._state = ManipulationState.IDLE
        return module, monitor

    yield create

    for module in reversed(modules):
        module.stop()


@pytest.fixture
def latched(make_module: ModuleFactory) -> tuple[ManipulationModule, _StubWorldMonitor]:
    return make_module()


def test_latch_adopts_the_latest_odom_pose(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, monitor = latched
    module._on_odom(_pose(2.0, 1.0, 0.73, yaw=math.pi / 4))

    assert module.latch_base_pose() is True

    robot_id, pose = monitor.calls[0]
    assert robot_id == "robot_1"
    assert (pose.position.x, pose.position.y, pose.position.z) == (2.0, 1.0, 0.73)
    assert pose.orientation.to_euler().z == pytest.approx(math.pi / 4)


def test_latch_republishes_the_new_base_to_callers(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, _monitor = latched
    module.latch_base_pose(_pose(2.0, 1.0))

    base = module.get_base_pose()

    assert base is not None
    assert (base.position.x, base.position.y) == (2.0, 1.0)


def test_latch_prefers_an_explicit_pose_over_odom(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, monitor = latched
    module._on_odom(_pose(2.0, 1.0))

    assert module.latch_base_pose(_pose(5.0, 6.0)) is True

    assert monitor.calls[0][1].position.x == 5.0


def test_latch_without_any_pose_is_refused(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, monitor = latched

    assert module.latch_base_pose() is False
    assert monitor.calls == []
    assert "odom" in module.get_error()


def test_latch_refuses_a_non_world_frame(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, monitor = latched
    pose = _pose(1.0, 0.0)
    pose.frame_id = "pelvis"

    assert module.latch_base_pose(pose) is False
    assert monitor.calls == []


def test_latch_refuses_non_finite_values(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, monitor = latched

    assert module.latch_base_pose(_pose(float("nan"), 0.0)) is False
    assert monitor.calls == []


@pytest.mark.parametrize("state", [ManipulationState.PLANNING, ManipulationState.EXECUTING])
def test_latch_refuses_while_busy(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
    state: ManipulationState,
) -> None:
    module, monitor = latched
    module._state = state

    assert module.latch_base_pose(_pose(1.0, 0.0)) is False
    assert monitor.calls == []


def test_latch_discards_the_stored_plan(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, _monitor = latched
    module._last_plan = MagicMock()

    assert module.latch_base_pose(_pose(1.0, 0.0)) is True
    assert module._last_plan is None


def test_failed_latch_reports_instead_of_raising(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, monitor = latched
    monitor.failure = RuntimeError("scene build failed")

    assert module.latch_base_pose(_pose(1.0, 0.0)) is False
    assert "scene build failed" in module.get_error()


def test_drift_beyond_the_position_bound_warns_at_plan_time(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, _monitor = latched
    module._on_odom(_pose(2.0, 0.0))

    assert module._warn_if_base_drifted() is True
    assert "STALE" in module.describe_base_pose()


def test_rotation_alone_warns_at_plan_time(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, _monitor = latched
    module._on_odom(_pose(0.0, 0.0, yaw=math.radians(10.0)))

    drift = module._base_drift()

    assert drift is not None
    assert drift[0] == pytest.approx(0.0)
    assert math.degrees(drift[1]) == pytest.approx(10.0)
    assert module._warn_if_base_drifted() is True


def test_pose_within_both_bounds_stays_quiet(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, _monitor = latched
    module._on_odom(_pose(0.005, 0.0, 0.74, yaw=math.radians(1.0)))

    assert module._warn_if_base_drifted() is False
    assert "current" in module.describe_base_pose()


def test_latching_clears_the_drift_warning(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, _monitor = latched
    module._on_odom(_pose(2.0, 1.0, 0.73, yaw=math.pi / 4))
    assert module._warn_if_base_drifted() is True

    module.latch_base_pose()

    assert module._warn_if_base_drifted() is False


def test_drift_is_silent_without_odom(
    latched: tuple[ManipulationModule, _StubWorldMonitor],
) -> None:
    module, _monitor = latched

    assert module._base_drift() is None
    assert module._warn_if_base_drifted() is False


def test_floor_slab_follows_the_latched_base(make_module: ModuleFactory) -> None:
    module, monitor = make_module(floor_z=0.0)
    module._floor_base_offset = (0.7, 0.0)

    module.latch_base_pose(_pose(2.0, 1.0, 0.74, yaw=math.pi / 2))

    floor = monitor.obstacle_poses["floor"]
    assert floor.position.x == pytest.approx(2.0)
    assert floor.position.y == pytest.approx(1.7)
    assert floor.position.z == pytest.approx(-0.1)


def test_floor_slab_keeps_its_authored_place_for_a_table_robot(
    make_module: ModuleFactory,
) -> None:
    module, _monitor = make_module(_robot_config(name="arm", base_z=0.0), floor_z=0.0)
    module._floor_base_offset = (0.7, 0.0)

    floor = module._floor_pose()

    assert floor.position.x == pytest.approx(0.7)
    assert floor.position.y == pytest.approx(0.0)
    assert floor.position.z == pytest.approx(-0.1)


def test_multi_robot_latch_needs_an_explicit_robot(make_module: ModuleFactory) -> None:
    module, monitor = make_module(_robot_config(name="left"), _robot_config(name="right"))

    assert module.latch_base_pose(_pose(1.0, 0.0)) is False
    assert monitor.calls == []
    assert "odom_robot_name" in module.get_error()


def test_configured_odom_robot_resolves_the_latch(make_module: ModuleFactory) -> None:
    module, monitor = make_module(
        _robot_config(name="left"),
        _robot_config(name="right"),
        odom_robot_name="right",
    )

    assert module.latch_base_pose(_pose(1.0, 0.0)) is True
    assert monitor.calls[0][0] == "robot_2"
