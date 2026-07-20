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
from typing import cast

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
from dimos.control.tasks.eef_twist_task.eef_twist_task import create_task as _eef_create_task
from dimos.control.tasks.registry import control_task_registry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def _robot(path: Path) -> RobotModelConfig:
    return RobotModelConfig(
        name="tiny",
        model_path=path,
        base_pose=PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]),
        joint_names=["joint1"],
        end_effector_link="tool",
        home_joints=[0.0],
    )


def _state(t_now: float, dt: float = 0.01) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"joint1": 0.0}), t_now=t_now, dt=dt
    )


class _FakeControlIK:
    nq = 1

    def __init__(self) -> None:
        self.target: object | None = None
        self.dt: float | None = None

    def solve(self, target: object, measured: np.ndarray, dt: float) -> ControlIKResult:
        self.target = target
        self.dt = dt
        return ControlIKResult(measured.copy(), np.zeros(1))


def test_cartesian_pipeline_passes_se3_target_and_bounded_dt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    backend = _FakeControlIK()
    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.PinkControlIK",
        lambda *args, **kwargs: backend,
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


def test_cartesian_pipeline_rejects_invalid_quaternion_with_hold(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    backend = _FakeControlIK()
    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.PinkControlIK",
        lambda *args, **kwargs: backend,
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


def test_cartesian_and_eef_modules_import_without_pink() -> None:
    script = """
import sys

class BlockPink:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "pink":
            raise ModuleNotFoundError("No module named 'pink'", name="pink")
        return None

sys.meta_path.insert(0, BlockPink())
import dimos.control.tasks.cartesian_ik_task.cartesian_ik_task
import dimos.control.tasks.eef_twist_task.eef_twist_task
"""
    subprocess.run([sys.executable, "-c", script], check=True, capture_output=True, text=True)


def test_pink_factories_fail_actionably_when_pink_is_absent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import dimos.control.tasks.cartesian_ik_task.pink_control_ik as pink_control_ik

    robot = _robot(tmp_path / "unused.urdf")
    params = {"control_ik": {"robot_model": robot}}
    monkeypatch.setattr(pink_control_ik, "pink", None)

    for task_type in ("cartesian_ik", "eef_twist"):
        config = TaskConfig(name=task_type, type=task_type, joint_names=["joint1"], params=params)
        with pytest.raises(ModuleNotFoundError, match="uv sync --extra manipulation"):
            if task_type == "cartesian_ik":
                control_task_registry.create("cartesian_ik", config, hardware={})
            else:
                _eef_create_task(config, {})


def test_cartesian_runtime_error_is_a_measured_state_hold(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    backend = _FakeControlIK()

    def fail(target: object, measured: np.ndarray, dt: float) -> ControlIKResult:
        raise IKControlRuntimeError("solver failed")

    monkeypatch.setattr(backend, "solve", fail)
    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.PinkControlIK",
        lambda *args, **kwargs: backend,
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
