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
import pytest

pytest.importorskip("pink")

from dimos.control.tasks.cartesian_ik_task.cartesian_ik_task import CartesianIKTaskConfig
from dimos.control.tasks.cartesian_ik_task.pink_control_ik import (
    IKControlRuntimeError,
    PinkControlIK,
    PinkControlIKConfig,
)
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


def test_pink_requires_robot_model() -> None:
    with pytest.raises(ValueError, match="robot_model"):
        PinkControlIKConfig()


def test_pink_settings_use_finite_declarative_validation(tmp_path: Path) -> None:
    robot = _robot(_write_urdf(tmp_path))

    with pytest.raises(ValueError, match="finite"):
        PinkControlIKConfig(robot_model=robot, max_velocity=np.inf)
    with pytest.raises(ValueError, match="finite"):
        PinkControlIKConfig(robot_model=robot, qpsolver_options={"eps": np.nan})
    with pytest.raises(ValueError, match="ordered"):
        CartesianIKTaskConfig(
            joint_names=["joint1", "joint2"],
            control_ik=PinkControlIKConfig(robot_model=robot),
            min_dt=0.1,
            max_dt=0.01,
        )


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
            PinkControlIKConfig(robot_model=_robot(model_path, frame="missing")),
        )

    mismatched = _robot(model_path).model_copy(
        update={"joint_name_mapping": {"joint1": "missing", "joint2": "joint2"}}
    )
    with pytest.raises(ValueError, match="unknown joint"):
        PinkControlIK(
            PinkControlIKConfig(robot_model=mismatched),
        )


def test_pink_reanchors_measured_state_and_runs_one_frame_task_step(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_path = _write_urdf(tmp_path)
    backend = PinkControlIK(
        PinkControlIKConfig(robot_model=_robot(model_path), qpsolver_options={"eps": 1e-6}),
    )
    measured = np.array([0.3, 0.1])
    target = backend.forward_kinematics(measured)
    calls: list[tuple[object, list[object], float, dict[str, object]]] = []

    def solve(
        configuration: object, tasks: list[object], dt: float, **kwargs: object
    ) -> np.ndarray:
        calls.append((configuration, tasks, dt, kwargs))
        return np.zeros(backend._model.nv)

    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.pink_control_ik.pink.solve_ik", solve
    )
    result = backend.solve(target, measured, 0.01)

    assert np.array_equal(result.positions, measured)
    assert len(calls) == 1
    assert len(calls[0][1]) == 2
    assert calls[0][1] == [backend._frame_task, backend._posture_task]
    assert backend._posture_task is not None
    assert np.array_equal(backend._posture_task.target_q, backend._full_q(measured))
    assert calls[0][2] == 0.01
    assert calls[0][3] == {
        "solver": "proxqp",
        "damping": 1e-4,
        "limits": backend._limits,
        "eps": 1e-6,
    }


def test_pink_solver_dependency_failure_is_translated_to_runtime_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    backend = PinkControlIK(PinkControlIKConfig(robot_model=_robot(_write_urdf(tmp_path))))

    def solve(
        configuration: object, tasks: list[object], dt: float, **kwargs: object
    ) -> np.ndarray:
        raise RuntimeError("solver dependency failed")

    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.pink_control_ik.pink.solve_ik", solve
    )
    measured = np.array([0.3, 0.1])
    with pytest.raises(IKControlRuntimeError, match="solver dependency failed"):
        backend.solve(backend.forward_kinematics(measured), measured, 0.01)


def test_pink_receives_pre_bounded_dt_unchanged(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_path = _write_urdf(tmp_path)
    backend = PinkControlIK(
        PinkControlIKConfig(robot_model=_robot(model_path)),
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
    backend.solve(backend.forward_kinematics(measured), measured, 0.05)

    assert calls == [0.05]


def test_pink_posture_task_can_be_disabled(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    model_path = _write_urdf(tmp_path)
    backend = PinkControlIK(PinkControlIKConfig(robot_model=_robot(model_path), posture_cost=0.0))
    calls: list[list[object]] = []

    def solve(
        configuration: object, tasks: list[object], dt: float, **kwargs: object
    ) -> np.ndarray:
        calls.append(tasks)
        return np.zeros(backend._model.nv)

    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.pink_control_ik.pink.solve_ik", solve
    )
    measured = np.array([0.3, 0.1])
    backend.solve(backend.forward_kinematics(measured), measured, 0.01)

    assert calls and len(calls[0]) == 1


def test_pink_rejects_uncontrolled_end_effector_chain_without_reference(
    tmp_path: Path,
) -> None:
    model_path = _write_urdf(tmp_path, "uncontrolled.urdf", _UNCONTROLLED_URDF)
    with pytest.raises(ValueError, match="reference_q.*uncontrolled joint"):
        PinkControlIK(
            PinkControlIKConfig(robot_model=_robot(model_path)),
        )


def test_continuous_joint_scalar_limits_fail_with_actionable_diagnostic(tmp_path: Path) -> None:
    model_path = _write_urdf(tmp_path, "continuous.urdf", _CONTINUOUS_URDF)
    robot = _robot(model_path, joints=["joint1"])

    with pytest.raises(ValueError, match="continuous joints.*tangent-space"):
        PinkControlIK(
            PinkControlIKConfig(robot_model=robot),
        )

    roundtrip_robot = robot.model_copy(
        update={"joint_limits_lower": None, "joint_limits_upper": None}
    )
    backend = PinkControlIK(
        PinkControlIKConfig(robot_model=roundtrip_robot),
    )
    angle = np.array([3.0])

    assert backend._q_widths == [2]
    assert np.allclose(backend._project_controlled_positions(backend._full_q(angle), angle), angle)


def test_pink_applies_position_velocity_limits_and_finite_output(tmp_path: Path) -> None:
    model_path = _write_urdf(tmp_path)
    robot = _robot(model_path).model_copy(
        update={"joint_limits_lower": [-0.5, -0.25], "joint_limits_upper": [0.5, 0.25]}
    )
    backend = PinkControlIK(
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
    backend = PinkControlIK(PinkControlIKConfig(robot_model=robot))
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
    backend = PinkControlIK(PinkControlIKConfig(robot_model=robot))
    measured = np.array([1.22, 0.1])

    def solve(
        configuration: object, tasks: list[object], dt: float, **kwargs: object
    ) -> np.ndarray:
        return np.array([0.01, -0.2])

    monkeypatch.setattr(
        "dimos.control.tasks.cartesian_ik_task.pink_control_ik.pink.solve_ik", solve
    )
    with pytest.raises(IKControlRuntimeError, match="out-of-bounds"):
        backend.solve(backend.forward_kinematics(measured), measured, 0.01)


@pytest.mark.parametrize("legacy_field", ["backend", "ee_joint_id", "self_collision_enabled"])
def test_pink_rejects_legacy_configuration_fields(tmp_path: Path, legacy_field: str) -> None:
    model_path = _write_urdf(tmp_path)
    with pytest.raises(ValueError, match=legacy_field):
        PinkControlIKConfig.model_validate(
            {"robot_model": _robot(model_path), legacy_field: "pinocchio"}
        )
