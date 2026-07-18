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
    PinkControlIK,
    PinkControlIKConfig,
    PinkControlRuntimeError,
    PinocchioIK,
)
from dimos.control.tasks.registry import control_task_registry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

_URDF = """\
<robot name="tiny">
  <link name="base"/>
  <link name="link1"/>
  <link name="tool"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/><origin xyz="0.2 0 0.2"/>
    <axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="tool"/><origin xyz="0.2 0 0.2"/>
    <axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
</robot>
"""

_CONTINUOUS_URDF = """\
<robot name="continuous">
  <link name="base"/><link name="tool"/>
  <joint name="joint1" type="continuous">
    <parent link="base"/><child link="tool"/><axis xyz="0 0 1"/>
    <limit effort="1" velocity="1"/>
  </joint>
</robot>
"""

_UNCONTROLLED_URDF = """\
<robot name="uncontrolled">
  <link name="base"/><link name="link1"/><link name="aux"/><link name="tool"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/><origin xyz="0.2 0 0.2"/>
    <axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
  <joint name="aux_joint" type="revolute">
    <parent link="link1"/><child link="aux"/><origin xyz="0.1 0 0"/>
    <axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="aux"/><child link="tool"/><origin xyz="0.2 0 0.2"/>
    <axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def _robot(
    path: Path,
    *,
    frame: str = "tool",
    joints: list[str] | None = None,
) -> RobotModelConfig:
    joint_names = joints or ["joint1", "joint2"]
    joint_count = len(joint_names)
    return RobotModelConfig(
        name="tiny",
        model_path=path,
        base_pose=PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]),
        joint_names=joint_names,
        end_effector_link=frame,
        home_joints=[0.4] * joint_count,
        joint_limits_lower=[-2.0] * joint_count,
        joint_limits_upper=[2.0] * joint_count,
        velocity_limits=[1.0] * joint_count,
    )


def _write_urdf(tmp_path: Path, name: str = "tiny.urdf", content: str = _URDF) -> Path:
    path = tmp_path / name
    path.write_text(content)
    return path


def test_pink_is_default_and_requires_robot_model() -> None:
    config = PinkControlIKConfig()

    assert config.backend == "pink"
    with pytest.raises(ValueError, match="RobotModelConfig"):
        config.validate_settings(2, None)


def test_pink_prepares_xacro_with_package_paths_and_arguments(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_path = _write_urdf(tmp_path)
    package_path = tmp_path / "description"
    package_path.mkdir()
    robot = _robot(model_path).model_copy(
        update={
            "model_path": tmp_path / "robot.xacro",
            "package_paths": {"description": package_path},
            "xacro_args": {"dof": "2"},
        }
    )
    prepared: dict[str, object] = {}

    def prepare(
        path: Path,
        package_paths: dict[str, Path],
        xacro_args: dict[str, str],
        convert_meshes: bool,
    ) -> str:
        prepared.update(
            path=path,
            package_paths=package_paths,
            xacro_args=xacro_args,
            convert_meshes=convert_meshes,
        )
        return str(model_path)

    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.pink_control_ik.prepare_urdf_for_drake",
        prepare,
    )

    PinkControlIK(
        tmp_path / "robot.xacro",
        None,
        ["joint1", "joint2"],
        PinkControlIKConfig(robot_model=robot),
    )

    assert prepared == {
        "path": tmp_path / "robot.xacro",
        "package_paths": {"description": package_path},
        "xacro_args": {"dof": "2"},
        "convert_meshes": False,
    }


def test_pink_validates_named_frame_and_exact_joint_mapping(tmp_path: Path) -> None:
    model_path = _write_urdf(tmp_path)

    with pytest.raises(ValueError, match="end-effector frame"):
        PinkControlIK(
            model_path,
            None,
            ["joint1", "joint2"],
            PinkControlIKConfig(robot_model=_robot(model_path, frame="missing")),
        )

    mismatched = _robot(model_path).model_copy(
        update={"joint_name_mapping": {"arm/joint1": "joint1", "arm/joint2": "joint2"}}
    )
    with pytest.raises(ValueError, match="exactly match"):
        PinkControlIK(
            model_path,
            None,
            ["joint1", "joint2"],
            PinkControlIKConfig(robot_model=mismatched),
        )


def test_pink_reanchors_measured_state_and_runs_one_frame_task_step(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_path = _write_urdf(tmp_path)
    backend = PinkControlIK(
        model_path,
        None,
        ["joint1", "joint2"],
        PinkControlIKConfig(robot_model=_robot(model_path)),
    )
    measured = np.array([0.3, 0.1])
    target = backend.forward_kinematics(measured)
    calls: list[tuple[object, list[object], float]] = []

    def solve(
        configuration: object, tasks: list[object], dt: float, **kwargs: object
    ) -> np.ndarray:
        calls.append((configuration, tasks, dt))
        return np.zeros(backend._model.nv)

    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.pink_control_ik.pink.solve_ik", solve
    )
    result = backend.solve(target, measured, 0.01)

    assert np.array_equal(result.positions, measured)
    assert len(calls) == 1
    assert len(calls[0][1]) == 1
    assert calls[0][2] == 0.01


def test_pink_backend_clamps_dt_from_backend_configuration(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_path = _write_urdf(tmp_path)
    backend = PinkControlIK(
        model_path,
        None,
        ["joint1", "joint2"],
        PinkControlIKConfig(robot_model=_robot(model_path), min_dt=0.01, max_dt=0.02),
    )
    calls: list[float] = []

    def solve(
        configuration: object, tasks: list[object], dt: float, **kwargs: object
    ) -> np.ndarray:
        calls.append(dt)
        return np.zeros(backend._model.nv)

    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.pink_control_ik.pink.solve_ik", solve
    )
    measured = np.array([0.3, 0.1])
    backend.solve(backend.forward_kinematics(measured), measured, 1.0)

    assert calls == [0.02]


def test_pink_rejects_uncontrolled_end_effector_chain_without_reference(
    tmp_path: Path,
) -> None:
    model_path = _write_urdf(tmp_path, "uncontrolled.urdf", _UNCONTROLLED_URDF)
    with pytest.raises(ValueError, match="reference_q.*uncontrolled joint"):
        PinkControlIK(
            model_path,
            None,
            ["joint1", "joint2"],
            PinkControlIKConfig(robot_model=_robot(model_path)),
        )


def test_continuous_joint_scalar_limits_fail_with_actionable_diagnostic(tmp_path: Path) -> None:
    model_path = _write_urdf(tmp_path, "continuous.urdf", _CONTINUOUS_URDF)
    robot = _robot(model_path, joints=["joint1"])

    with pytest.raises(ValueError, match="continuous joints.*tangent-space"):
        PinkControlIK(
            model_path,
            None,
            ["joint1"],
            PinkControlIKConfig(robot_model=robot),
        )

    roundtrip_robot = robot.model_copy(
        update={"joint_limits_lower": None, "joint_limits_upper": None}
    )
    backend = PinkControlIK(
        model_path,
        None,
        ["joint1"],
        PinkControlIKConfig(robot_model=roundtrip_robot),
    )
    angle = np.array([3.0])

    assert backend._q_widths == [2]
    assert np.allclose(backend._controlled_q(backend._full_q(angle), angle), angle)


def test_pink_applies_position_velocity_limits_and_finite_output(tmp_path: Path) -> None:
    model_path = _write_urdf(tmp_path)
    robot = _robot(model_path).model_copy(
        update={"joint_limits_lower": [-0.5, -0.25], "joint_limits_upper": [0.5, 0.25]}
    )
    backend = PinkControlIK(
        model_path,
        None,
        ["joint1", "joint2"],
        PinkControlIKConfig(robot_model=robot, max_velocity=0.2),
    )

    assert np.array_equal(backend._model.lowerPositionLimit[:2], np.array([-0.5, -0.25]))
    assert np.all(backend._model.velocityLimit[backend._v_indices] <= 0.2)
    result = backend.solve(
        backend.forward_kinematics(np.array([0.1, 0.1])), np.array([0.1, 0.1]), 0.01
    )
    assert result.positions.shape == (2,)
    assert np.all(np.isfinite(result.positions))


def test_pink_clamps_tiny_position_limit_overshoot(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_path = _write_urdf(tmp_path)
    robot = _robot(model_path).model_copy(
        update={"joint_limits_lower": [-1.22, -0.25], "joint_limits_upper": [1.22, 0.25]}
    )
    backend = PinkControlIK(
        model_path, None, ["joint1", "joint2"], PinkControlIKConfig(robot_model=robot)
    )
    measured = np.array([1.22, 0.1])

    def solve(
        configuration: object, tasks: list[object], dt: float, **kwargs: object
    ) -> np.ndarray:
        return np.array([0.00013784674535, -0.2])

    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.pink_control_ik.pink.solve_ik", solve
    )
    result = backend.solve(backend.forward_kinematics(measured), measured, 0.01)

    assert np.array_equal(result.positions, np.array([1.22, 0.098]))
    assert np.array_equal(result.velocity, np.array([0.00013784674535, -0.2]))


def test_pink_rejects_material_position_limit_violation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_path = _write_urdf(tmp_path)
    robot = _robot(model_path).model_copy(
        update={"joint_limits_lower": [-1.22, -0.25], "joint_limits_upper": [1.22, 0.25]}
    )
    backend = PinkControlIK(
        model_path, None, ["joint1", "joint2"], PinkControlIKConfig(robot_model=robot)
    )
    measured = np.array([1.22, 0.1])

    def solve(
        configuration: object, tasks: list[object], dt: float, **kwargs: object
    ) -> np.ndarray:
        return np.array([0.01, -0.2])

    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.pink_control_ik.pink.solve_ik", solve
    )
    with pytest.raises(PinkControlRuntimeError, match="out-of-bounds"):
        backend.solve(backend.forward_kinematics(measured), measured, 0.01)


def test_cartesian_pipeline_bounds_dt_and_holds_on_expected_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    backend = _FakeControlIK()
    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.PinkControlIK",
        lambda *args, **kwargs: backend,
    )
    task = CartesianIKTask(
        "cartesian",
        CartesianIKTaskConfig(
            joint_names=["j1", "j2"],
            model_path="unused.urdf",
            timeout=0.2,
        ),
    )
    pose = PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1])
    assert task.on_cartesian_command(pose, 1.0)
    assert task.compute(_cartesian_state(1.01, dt=1.0)) is not None
    assert backend.dt_calls == [task._config.control_ik.max_dt]

    invalid_dt_hold = task.compute(_cartesian_state(1.02, dt=0.0))
    assert invalid_dt_hold is not None
    assert invalid_dt_hold.positions == [0.0, 0.0]

    backend.raise_runtime = True
    assert task.on_cartesian_command(pose, 2.0)
    hold = task.compute(_cartesian_state(2.01))
    assert hold is not None
    assert hold.positions == [0.0, 0.0]
    assert hold.mode.value == "servo_position"


def test_factory_rejects_invalid_default_pink_configuration() -> None:
    config = TaskConfig(
        name="cartesian",
        type="cartesian_ik",
        joint_names=["j1", "j2"],
        priority=10,
        params={"model_path": "unused.urdf"},
    )

    with pytest.raises(ValueError, match="RobotModelConfig"):
        control_task_registry.create("cartesian_ik", config, hardware={})


def test_explicit_pinocchio_selection_does_not_fallback_from_pink(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_path = _write_urdf(tmp_path)
    legacy = _FakeLegacyIK(pinocchio.buildModelFromUrdf(str(model_path)))

    def load(path: Path, ee_joint_id: int) -> _FakeLegacyIK:
        legacy.calls.append((path, ee_joint_id))
        return legacy

    monkeypatch.setattr(PinocchioIK, "from_model_path", staticmethod(load))

    backend = PinkControlIK(
        model_path,
        2,
        ["joint1", "joint2"],
        PinkControlIKConfig(backend="pinocchio"),
    )

    assert backend._is_pinocchio
    assert legacy.calls == [(model_path, 2)]
    with pytest.raises(ValueError, match="RobotModelConfig"):
        PinkControlIK(model_path, None, ["joint1", "joint2"], PinkControlIKConfig())


class _FakeLegacyIK:
    nq = 2

    def __init__(self, model: pinocchio.Model) -> None:
        self.model = model
        self.calls: list[tuple[Path, int]] = []

    def forward_kinematics(self, q: np.ndarray) -> pinocchio.SE3:
        return pinocchio.SE3.Identity()


class _FakeControlIK:
    nq = 2

    def __init__(self) -> None:
        self.result = np.array([0.1, 0.2])
        self.raise_runtime = False
        self.dt_calls: list[float] = []

    def solve(self, target: object, measured: np.ndarray, dt: float) -> ControlIKResult:
        self.dt_calls.append(dt)
        if self.raise_runtime:
            raise RuntimeError("synthetic control failure")
        return ControlIKResult(self.result.copy(), self.result - measured)


def _cartesian_state(t_now: float, dt: float = 0.01) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"j1": 0.0, "j2": 0.0}),
        t_now=t_now,
        dt=dt,
    )
