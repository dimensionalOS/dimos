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

from pathlib import Path
import subprocess
import sys
from typing import Any, cast

import numpy as np
import pinocchio
import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.cartesian_ik_task.cartesian_ik_task import (
    CartesianIKTask,
    CartesianIKTaskConfig,
)
from dimos.control.tasks.cartesian_ik_task.pink_control_ik import (
    ControlIKResult,
    IKControlRuntimeError,
    PinkControlIKConfig,
)
from dimos.control.tasks.registry import control_task_registry
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def _robot(path: Path) -> RobotModelConfig:
    return RobotModelConfig(
        model_path=path,
        base_pose=PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]),
        joint_names=["joint1"],
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1",),
                base_link="base",
                tip_link="tool",
            )
        ],
        home_joints=[0.0],
    )


def _state(t_now: float, dt: float = 0.01, position: float = 0.0) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"joint1": position}), t_now=t_now, dt=dt
    )


class _FakeControlIK:
    nq = 1

    def __init__(self) -> None:
        self.target: Any | None = None
        self.dt: float | None = None
        self.increment = 0.0
        self.solve_seeds: list[np.ndarray] = []

    def solve(self, target: Any, measured: np.ndarray, dt: float) -> ControlIKResult:
        self.target = target
        self.dt = dt
        self.solve_seeds.append(measured.copy())
        positions = measured + self.increment
        return ControlIKResult(positions, positions - measured)


def test_cartesian_pipeline_passes_se3_target_and_bounded_dt(tmp_path: Path, mocker) -> None:
    backend = _FakeControlIK()
    mocker.patch(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.create_pink_control_ik",
        return_value=backend,
    )
    task = CartesianIKTask(
        "cartesian",
        CartesianIKTaskConfig(
            joint_names=["joint1"],
            control_ik=PinkControlIKConfig(robot_model=_robot(tmp_path / "unused.urdf")),
            min_dt=0.01,
            max_dt=0.05,
        ),
    )
    assert task.on_cartesian_command(
        PoseStamped(position=[0.2, -0.3, 0.4], orientation=[0, 0, 0, 2]), 1.0
    )

    assert task.compute(_state(1.01, dt=1.0)) is not None
    target = cast("pinocchio.SE3", backend.target)
    assert isinstance(target, pinocchio.SE3)
    assert np.allclose(target.translation, [0.2, -0.3, 0.4])
    assert np.allclose(target.rotation, np.eye(3))
    assert backend.dt == 0.05


def test_cartesian_pipeline_rejects_invalid_quaternion_with_hold(tmp_path: Path, mocker) -> None:
    backend = _FakeControlIK()
    mocker.patch(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.create_pink_control_ik",
        return_value=backend,
    )
    task = CartesianIKTask(
        "cartesian",
        CartesianIKTaskConfig(
            joint_names=["joint1"],
            control_ik=PinkControlIKConfig(robot_model=_robot(tmp_path / "unused.urdf")),
        ),
    )
    assert task.on_cartesian_command(PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 0]), 1.0)

    hold = task.compute(_state(1.01))
    assert hold is not None
    assert hold.positions == [0.0]
    assert backend.target is None


def test_factory_rejects_invalid_default_pink_configuration() -> None:
    config = TaskConfig(
        name="cartesian", type="cartesian_ik", joint_names=["j1"], priority=10, params={}
    )
    with pytest.raises(ValueError, match="control_ik"):
        control_task_registry.create("cartesian_ik", config, hardware={})


@pytest.mark.parametrize(
    "module_name",
    [
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task",
        "dimos.control.tasks.eef_twist_task.eef_twist_task",
    ],
)
def test_control_task_import_fails_actionably_without_pink(module_name: str) -> None:
    script = f"""
import sys

class BlockPink:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "pink":
            raise ModuleNotFoundError("No module named 'pink'", name="pink")
        return None

sys.meta_path.insert(0, BlockPink())
import {module_name}
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "Install it with `uv sync`" in result.stderr


def test_cartesian_runtime_error_is_a_measured_state_hold(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker
) -> None:
    backend = _FakeControlIK()

    def fail(target: Any, measured: np.ndarray, dt: float) -> ControlIKResult:
        raise IKControlRuntimeError("solver failed")

    monkeypatch.setattr(backend, "solve", fail)
    mocker.patch(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.create_pink_control_ik",
        return_value=backend,
    )
    task = CartesianIKTask(
        "cartesian",
        CartesianIKTaskConfig(
            joint_names=["joint1"],
            control_ik=PinkControlIKConfig(robot_model=_robot(tmp_path / "unused.urdf")),
        ),
    )
    assert task.on_cartesian_command(PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]), 1.0)

    hold = task.compute(_state(1.01))
    assert hold is not None
    assert hold.positions == [0.0]


def test_cartesian_pipeline_accumulates_from_accepted_commands_while_feedback_tracks(
    tmp_path: Path, mocker
) -> None:
    backend = _FakeControlIK()
    backend.increment = 0.01
    mocker.patch(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.create_pink_control_ik",
        return_value=backend,
    )
    task = CartesianIKTask(
        "cartesian",
        CartesianIKTaskConfig(
            joint_names=["joint1"],
            control_ik=PinkControlIKConfig(robot_model=_robot(tmp_path / "unused.urdf")),
            max_tracking_error_deg=10.0,
        ),
    )
    assert task.on_cartesian_command(PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]), 1.0)

    first = task.compute(_state(1.01))
    second = task.compute(_state(1.02))

    assert first is not None
    assert second is not None
    assert first.positions == pytest.approx([0.01])
    assert second.positions == pytest.approx([0.02])
    np.testing.assert_allclose(backend.solve_seeds, [[0.0], [0.01]])


def test_cartesian_pipeline_rebases_when_command_outpaces_feedback(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker
) -> None:
    backend = _FakeControlIK()
    backend.increment = 0.1
    mocker.patch(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.create_pink_control_ik",
        return_value=backend,
    )
    task = CartesianIKTask(
        "cartesian",
        CartesianIKTaskConfig(
            joint_names=["joint1"],
            control_ik=PinkControlIKConfig(robot_model=_robot(tmp_path / "unused.urdf")),
            max_tracking_error_deg=5.0,
        ),
    )
    assert task.on_cartesian_command(PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]), 1.0)

    first = task.compute(_state(1.01))
    second = task.compute(_state(1.02))

    assert first is not None
    assert second is not None
    assert first.positions == pytest.approx([0.1])
    assert second.positions == pytest.approx([0.1])
    np.testing.assert_allclose(backend.solve_seeds, [[0.0], [0.0]])

    def fail(target: Any, measured: np.ndarray, dt: float) -> ControlIKResult:
        raise IKControlRuntimeError("solver failed")

    monkeypatch.setattr(backend, "solve", fail)
    hold = task.compute(_state(1.03))

    assert hold is not None
    assert hold.positions == [0.0]
