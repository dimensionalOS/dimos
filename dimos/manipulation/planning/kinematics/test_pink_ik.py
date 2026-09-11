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

"""Unit tests for the Pink IK planning backend."""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType, ModuleType, SimpleNamespace
from typing import Any, cast

import numpy as np
from pink.exceptions import NoSolutionFound
import pytest
from pytest_mock import MockerFixture

import dimos.control.tasks.pose_target_ik as pose_target_ik
from dimos.control.tasks.pose_target_ik import (
    PinkJointLimitError,
    PinkPoseTargetSolver,
    _PinkControlContext,
)
from dimos.manipulation.planning.factory import create_kinematics
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupDefinition
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
import dimos.manipulation.planning.kinematics.pink_ik as pink_planning
from dimos.manipulation.planning.kinematics.pink_ik import (
    PinkIK,
    PinkIKConfig,
    _finite_retry_limits,
)
import dimos.manipulation.planning.kinematics.pink_solver as pink_ik
from dimos.manipulation.planning.kinematics.pink_solver import (
    _CURRENT_POSTURE_TASK,
    _build_joint_mapping,
    _frame_task_key,
    _PinkRobotContext,
    _PinkSolverCore,
    _read_dimos_position,
    _seed_positions_for_mapping,
    _write_dimos_position,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.joint_space import (
    CoordinateTopology,
    JointCoordinate,
    JointSpace,
)
from dimos.manipulation.planning.spec.models import IKResult
from dimos.manipulation.planning.spec.validation import PreparedRobotModel, prepare_robot_model
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import LoadedRobotModel, PlanarBaseDefinition, RobotModel
from dimos.utils.transform_utils import matrix_to_pose

_TRACKING_ERROR_RAD = np.deg2rad(10.0)

_LOCKED_WAIST_CHAIN_URDF = """\
<robot name="locked_waist_chain">
  <link name="pelvis"/><link name="waist_yaw_link"/><link name="waist_roll_link"/>
  <link name="torso_link"/><link name="shoulder_pitch_link"/><link name="shoulder_roll_link"/>
  <link name="shoulder_yaw_link"/><link name="elbow_link"/><link name="wrist_roll_link"/>
  <link name="wrist_pitch_link"/><link name="wrist_yaw_link"/><link name="tool"/>
  <joint name="waist_yaw" type="revolute"><origin xyz="0 0 0"/><parent link="pelvis"/><child link="waist_yaw_link"/><axis xyz="0 0 1"/><limit lower="-2.618" upper="2.618" effort="1" velocity="1"/></joint>
  <joint name="waist_roll" type="revolute"><origin xyz="-0.0039635 0 0.044"/><parent link="waist_yaw_link"/><child link="waist_roll_link"/><axis xyz="1 0 0"/><limit lower="-0.52" upper="0.52" effort="1" velocity="1"/></joint>
  <joint name="waist_pitch" type="revolute"><origin xyz="0 0 0"/><parent link="waist_roll_link"/><child link="torso_link"/><axis xyz="0 1 0"/><limit lower="-0.52" upper="0.52" effort="1" velocity="1"/></joint>
  <joint name="shoulder_pitch" type="revolute"><origin xyz="0.0039563 0.10022 0.24778" rpy="0.27931 0.000054949 -0.00019159"/><parent link="torso_link"/><child link="shoulder_pitch_link"/><axis xyz="0 1 0"/><limit lower="-3.0892" upper="2.6704" effort="1" velocity="1"/></joint>
  <joint name="shoulder_roll" type="revolute"><origin xyz="0 0.038 -0.013831" rpy="-0.27925 0 0"/><parent link="shoulder_pitch_link"/><child link="shoulder_roll_link"/><axis xyz="1 0 0"/><limit lower="-1.5882" upper="2.2515" effort="1" velocity="1"/></joint>
  <joint name="shoulder_yaw" type="revolute"><origin xyz="0 0.00624 -0.1032"/><parent link="shoulder_roll_link"/><child link="shoulder_yaw_link"/><axis xyz="0 0 1"/><limit lower="-2.618" upper="2.618" effort="1" velocity="1"/></joint>
  <joint name="elbow" type="revolute"><origin xyz="0.015783 0 -0.080518"/><parent link="shoulder_yaw_link"/><child link="elbow_link"/><axis xyz="0 1 0"/><limit lower="-1.0472" upper="2.0944" effort="1" velocity="1"/></joint>
  <joint name="wrist_roll" type="revolute"><origin xyz="0.100 0.00188791 -0.010"/><parent link="elbow_link"/><child link="wrist_roll_link"/><axis xyz="1 0 0"/><limit lower="-1.9722" upper="1.9722" effort="1" velocity="1"/></joint>
  <joint name="wrist_pitch" type="revolute"><origin xyz="0.038 0 0"/><parent link="wrist_roll_link"/><child link="wrist_pitch_link"/><axis xyz="0 1 0"/><limit lower="-1.6144" upper="1.6144" effort="1" velocity="1"/></joint>
  <joint name="wrist_yaw" type="revolute"><origin xyz="0.046 0 0"/><parent link="wrist_pitch_link"/><child link="wrist_yaw_link"/><axis xyz="0 0 1"/><limit lower="-1.6144" upper="1.6144" effort="1" velocity="1"/></joint>
  <joint name="tool_fixed" type="fixed"><origin xyz="0.0415 0.003 0"/><parent link="wrist_yaw_link"/><child link="tool"/></joint>
</robot>
"""


class _StreamingTestPinkIK(PinkPoseTargetSolver):
    """Expose private control-side streaming primitives for unit tests."""

    def __init__(self, config: PinkIKConfig) -> None:
        _PinkSolverCore.__init__(self, config)
        self._control_contexts = {}
        self._robot_contexts = {}
        self.feedback_limit_tolerance = 1e-3
        self.command_limit_margin = 1e-4
        self._joint_increment_filter_weights = self._validated_increment_filter_weights()
        self._command_increment_history = deque(
            maxlen=len(self._joint_increment_filter_weights) - 1
        )

    def step_frame_targets(
        self,
        robot_model: RobotModelConfig,
        frame_targets: Mapping[str, PoseStamped],
        controlled_joints: list[str],
        command_state: JointState,
        measured_state: JointState,
        max_command_tracking_error_rad: float,
        dt: float | None = None,
        max_joint_velocity_rad_s: float = 5.0,
        joint_velocity_limits_rad_s: Mapping[str, float] | None = None,
        joint_command_filter_cutoff_hz: float | None = None,
    ) -> JointState:
        result = self._step_frame_targets(
            robot_model=robot_model,
            frame_targets=frame_targets,
            controlled_joints=controlled_joints,
            command_state=command_state,
            measured_state=measured_state,
            max_command_tracking_error_rad=max_command_tracking_error_rad,
            feedback_limit_tolerance=self.feedback_limit_tolerance,
            command_limit_margin=self.command_limit_margin,
            dt=dt,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
            joint_velocity_limits_rad_s=joint_velocity_limits_rad_s or {},
            joint_command_filter_cutoff_hz=joint_command_filter_cutoff_hz,
            command_increment_history=tuple(self._command_increment_history),
        )
        self._command_increment_history.append(result.bounded_increment)
        return result.command

    def validate_frame_targets(
        self,
        robot_model: RobotModelConfig,
        frame_names: list[str],
        controlled_joints: list[str],
    ) -> None:
        self._validate_frame_targets(
            robot_model,
            frame_names,
            controlled_joints,
            self.command_limit_margin,
        )


class _FakeJoint:
    def __init__(self, idx_q: int) -> None:
        self.idx_q = idx_q
        self.idx_v = idx_q
        self.nq = 1
        self.nv = 1


class _FakeFrame:
    def __init__(self, name: str, parent_joint: int = 0) -> None:
        self.name = name
        self.parentJoint = parent_joint


class _FakePlacement:
    def __init__(self, translation: np.ndarray) -> None:
        self.rotation = np.eye(3)
        self.translation = translation


class _FakeData:
    def __init__(self) -> None:
        self.q = np.zeros(3)
        self.oMf = [_FakePlacement(np.zeros(3)), _FakePlacement(np.zeros(3))]


class _FakeModel:
    nq = 3
    nv = 3

    def __init__(self) -> None:
        self.names = ["universe", "joint_b", "joint_a", "joint_c"]
        self.joints = [SimpleNamespace(idx_q=-1, nq=0), _FakeJoint(0), _FakeJoint(1), _FakeJoint(2)]
        self.frames = [_FakeFrame("base", 0), _FakeFrame("tool", 3)]
        self._joint_ids = {"joint_b": 1, "joint_a": 2, "joint_c": 3}
        self._frame_ids = {"base": 0, "tool": 1}
        self.lowerPositionLimit = np.full(3, -1.0)
        self.upperPositionLimit = np.full(3, 1.0)
        self.velocityLimit = np.array([2.0, 4.0, np.inf])
        self.configuration_limit = object()
        self.velocity_limit = object()

    def createData(self) -> _FakeData:
        return _FakeData()

    def existJointName(self, name: str) -> bool:
        return name in self._joint_ids

    def getJointId(self, name: str) -> int:
        return self._joint_ids.get(name, len(self.joints))

    def existFrame(self, name: str) -> bool:
        return name in self._frame_ids

    def getFrameId(self, name: str) -> int:
        return self._frame_ids.get(name, len(self.frames))


class _FakePlanarModel:
    nq = 4
    nv = 3

    def __init__(self) -> None:
        self.names = ["universe", "base/x", "base/y", "base/yaw"]
        self.joints = [
            SimpleNamespace(idx_q=-1, idx_v=-1, nq=0, nv=0),
            SimpleNamespace(idx_q=0, idx_v=0, nq=1, nv=1),
            SimpleNamespace(idx_q=1, idx_v=1, nq=1, nv=1),
            SimpleNamespace(idx_q=2, idx_v=2, nq=2, nv=1),
        ]
        self._joint_ids = {"base/x": 1, "base/y": 2, "base/yaw": 3}

    def existJointName(self, name: str) -> bool:
        return name in self._joint_ids

    def getJointId(self, name: str) -> int:
        return self._joint_ids.get(name, len(self.joints))


class _FakeSE3:
    def __init__(self, rotation: np.ndarray, translation: np.ndarray) -> None:
        self.rotation = rotation
        self.translation = translation


class _FakeConfiguration:
    def __init__(self, model: _FakeModel, data: _FakeData, q: np.ndarray) -> None:
        self.model = model
        self.data = data
        self.update(q)

    def integrate_inplace(self, velocity: np.ndarray, dt: float) -> None:
        self.update(self.q + velocity * dt)

    def update(self, q: np.ndarray) -> None:
        self.q = q.copy()
        self.q.setflags(write=False)


class _FakeFrameTask:
    def __init__(self, frame: str, **_: object) -> None:
        self.frame = frame
        self.target: _FakeSE3 | None = None

    def set_target(self, target: _FakeSE3) -> None:
        self.target = target


class _FakePostureTask:
    def __init__(self, cost: float) -> None:
        self.cost = cost

    def set_target_from_configuration(self, configuration: _FakeConfiguration) -> None:
        self.target = configuration.q.copy()


class _FakeLinearHolonomicTask:
    def __init__(self, A: np.ndarray, b: np.ndarray, q_0: np.ndarray) -> None:
        self.A = A
        self.b = b
        self.q_0 = q_0


class _AuxiliaryTask:
    def __init__(self, value: float) -> None:
        self.value = value


class _ComposablePinkIK(PinkIK):
    def _create_tasks(self, configuration: Any, target_frames: tuple[str, ...]) -> dict[str, Any]:
        tasks = super()._create_tasks(configuration, target_frames)
        tasks[_frame_task_key(target_frames[0])].customized = True
        tasks.pop(_CURRENT_POSTURE_TASK, None)
        tasks["posture/nominal"] = _AuxiliaryTask(1.0)
        tasks["auxiliary/damping"] = _AuxiliaryTask(2.0)
        return tasks


class _DerivedComposablePinkIK(_ComposablePinkIK):
    def _create_tasks(self, configuration: Any, target_frames: tuple[str, ...]) -> dict[str, Any]:
        tasks = super()._create_tasks(configuration, target_frames)
        tasks["auxiliary/damping"].value = 3.0
        return tasks


class _RecordingStreamingPinkIK(_StreamingTestPinkIK):
    def __init__(self, config: PinkIKConfig) -> None:
        super().__init__(config)
        self.before_tasks: list[Mapping[str, Any]] = []
        self.after_velocities: list[np.ndarray] = []

    def _before_solve(self, tasks: Mapping[str, Any], configuration: Any, dt: float) -> None:
        self.before_tasks.append(tasks)

    def _after_solve(self, tasks: Mapping[str, Any], velocity: np.ndarray, dt: float) -> None:
        self.after_velocities.append(velocity.copy())


class _RecordingPinkIK(PinkIK):
    def __init__(self, config: PinkIKConfig) -> None:
        super().__init__(config)
        self.before_tasks: list[Mapping[str, Any]] = []
        self.after_velocities: list[np.ndarray] = []

    def _before_solve(self, tasks: Mapping[str, Any], configuration: Any, dt: float) -> None:
        self.before_tasks.append(tasks)

    def _after_solve(self, tasks: Mapping[str, Any], velocity: np.ndarray, dt: float) -> None:
        self.after_velocities.append(velocity.copy())


class _MissingFramePinkIK(PinkIK):
    def _create_tasks(self, configuration: Any, target_frames: tuple[str, ...]) -> dict[str, Any]:
        tasks = super()._create_tasks(configuration, target_frames)
        tasks.pop(_frame_task_key(target_frames[0]))
        return tasks


class _MismatchedFramePinkIK(PinkIK):
    def _create_tasks(self, configuration: Any, target_frames: tuple[str, ...]) -> dict[str, Any]:
        tasks = super()._create_tasks(configuration, target_frames)
        tasks[_frame_task_key(target_frames[0])] = _FakeFrameTask("base")
        return tasks


@dataclass(frozen=True)
class _FakeModules:
    pink: ModuleType
    pinocchio: ModuleType


def _fake_modules(converge: bool = True) -> _FakeModules:
    pinocchio = ModuleType("pinocchio")
    pinocchio.SE3 = _FakeSE3  # type: ignore[attr-defined]
    pinocchio.neutral = lambda model: np.zeros(model.nq)  # type: ignore[attr-defined]

    def forward_kinematics(model: _FakeModel, data: _FakeData, q: np.ndarray) -> None:
        data.q = q.copy()

    def update_frame_placements(model: _FakeModel, data: _FakeData) -> None:
        data.oMf[1] = _FakePlacement(data.q.copy())

    pinocchio.forwardKinematics = forward_kinematics  # type: ignore[attr-defined]
    pinocchio.updateFramePlacements = update_frame_placements  # type: ignore[attr-defined]

    pink = ModuleType("pink")
    pink.Configuration = _FakeConfiguration  # type: ignore[attr-defined]
    pink.tasks = SimpleNamespace(
        FrameTask=_FakeFrameTask,
        LinearHolonomicTask=_FakeLinearHolonomicTask,
        PostureTask=_FakePostureTask,
    )

    def solve_ik(
        configuration: _FakeConfiguration,
        tasks: list[object],
        dt: float,
        **_: object,
    ) -> np.ndarray:
        if not converge:
            return np.zeros_like(configuration.q)
        frame_task = tasks[0]
        target = frame_task.target.translation  # type: ignore[attr-defined,union-attr]
        return (target - configuration.q) / dt

    pink.solve_ik = solve_ik  # type: ignore[attr-defined]

    return _FakeModules(pink=pink, pinocchio=pinocchio)


def _install_fake_modules(mocker: MockerFixture, converge: bool = True) -> _FakeModules:
    modules = _fake_modules(converge=converge)
    mocker.patch.object(pose_target_ik, "pink", modules.pink)
    mocker.patch.object(pink_planning, "pink", modules.pink)
    mocker.patch.object(pink_ik, "pink", modules.pink)
    mocker.patch.object(pink_ik, "pinocchio", modules.pinocchio)
    mocker.patch.object(pink_ik.qpsolvers, "available_solvers", ["proxqp"])
    return modules


def _robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/tmp/fake.urdf")),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        joint_names=["joint_a", "joint_b", "joint_c"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint_a", "joint_b", "joint_c"),
                base_link="base",
                tip_link="tool",
            )
        ],
    )


def _test_joint_space() -> JointSpace:
    return JointSpace(
        tuple(
            JointCoordinate(
                name=name,
                mechanism_type="revolute",
                topology=CoordinateTopology.INTERVAL,
                lower=-1.0,
                upper=1.0,
                max_velocity=1.0,
                max_acceleration=2.0,
            )
            for name in _robot_config().joint_names
        )
    )


def _prepared_test_model() -> PreparedRobotModel:
    config = _robot_config()
    return PreparedRobotModel(
        config=config,
        description=LoadedRobotModel("<robot/>", Path("/tmp/fake.urdf"), {}),
        joint_space=_test_joint_space(),
        planning_groups=(),
    )


def _streaming_ik(mocker: MockerFixture, converge: bool = True) -> _StreamingTestPinkIK:
    _install_fake_modules(mocker, converge=converge)
    return _StreamingTestPinkIK(PinkIKConfig(max_iterations=3))


def _pink_ik(mocker: MockerFixture, converge: bool = True) -> PinkIK:
    _install_fake_modules(mocker, converge=converge)
    return PinkIK(PinkIKConfig(max_iterations=3))


def _context() -> _PinkRobotContext:
    model = _FakeModel()
    joint_space = _test_joint_space()
    mapping = _build_joint_mapping(model, joint_space)
    return _PinkRobotContext(
        model=model,
        data=model.createData(),
        frame_id=1,
        frame_name="tool",
        mapping=mapping,
        joint_space=joint_space,
    )


def _controlled_context(frame_name: str, controlled_joints: list[str]) -> _PinkRobotContext:
    model = _FakeModel()
    return _PinkRobotContext(
        model=model,
        data=model.createData(),
        frame_id=model.getFrameId(frame_name),
        frame_name=frame_name,
        mapping=_build_joint_mapping(model, _test_joint_space(), controlled_joints),
        joint_space=_test_joint_space().select(tuple(controlled_joints)),
    )


def _combined_control_context(
    frame_names: tuple[str, ...], controlled_joints: list[str]
) -> _PinkControlContext:
    robot = _controlled_context(frame_names[0], controlled_joints)
    frames = {frame_names[0]: robot}
    for frame_name in frame_names[1:]:
        frames[frame_name] = _PinkRobotContext(
            model=robot.model,
            data=robot.data,
            frame_id=robot.model.getFrameId(frame_name),
            frame_name=frame_name,
            mapping=robot.mapping,
            joint_space=robot.joint_space,
        )
    return _PinkControlContext(robot=robot, frames=MappingProxyType(frames))


class _FakeWorld:
    is_finalized = True

    def __init__(self, collision_free: bool = True) -> None:
        self.config = _robot_config()
        self.prepared = _prepared_test_model()
        self.collision_free = collision_free
        self.joint_state_calls = 0
        self.groups = {
            "manipulator": PlanningGroup(
                id="manipulator",
                joint_names=("joint_a", "joint_b"),
                base_link="base",
                tip_link="tool",
            ),
            "no_tip": PlanningGroup(
                id="no_tip",
                joint_names=("joint_c",),
                base_link="base",
                tip_link=None,
            ),
            "wrist": PlanningGroup(
                id="wrist",
                joint_names=("joint_c",),
                base_link="base",
                tip_link="base",
            ),
        }

    def get_prepared_model(self) -> PreparedRobotModel:
        return self.prepared

    def scratch_context(self) -> nullcontext[None]:
        return nullcontext(None)

    def get_joint_state(self, ctx: object) -> JointState:
        self.joint_state_calls += 1
        return JointState({"name": ["joint_b", "joint_c", "joint_a"], "position": [0.0, 0.0, 0.0]})

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])

    def check_config_collision_free(self, joint_state: JointState) -> bool:
        return self.collision_free


def test_create_kinematics_pink_unavailable_solver_mentions_manipulation_extra(
    mocker: MockerFixture,
) -> None:
    mocker.patch.object(pink_ik.qpsolvers, "available_solvers", [])

    with pytest.raises(ImportError, match="--extra manipulation --inexact"):
        create_kinematics("pink")


def test_create_kinematics_pink_returns_backend(mocker: MockerFixture) -> None:
    _install_fake_modules(mocker)

    assert isinstance(create_kinematics("pink"), PinkIK)


def test_locked_joints_are_constrained_in_the_pink_qp(mocker: MockerFixture) -> None:
    modules = _install_fake_modules(mocker)
    ik = PinkIK(PinkIKConfig())
    context = _context()
    seed_q = np.array([0.1, 0.2, 0.3])

    constraints = ik._locked_joint_constraints(
        context,
        seed_q,
        {0: -0.4, 2: 0.6},
    )

    assert len(constraints) == 1
    constraint = cast("_FakeLinearHolonomicTask", constraints[0])
    assert constraint.A == pytest.approx(
        np.array(
            [
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
    )
    assert constraint.b == pytest.approx([0.0, 0.0])
    assert constraint.q_0 == pytest.approx([0.1, -0.4, 0.6])

    solve_ik = mocker.patch.object(modules.pink, "solve_ik", return_value=np.zeros(3))
    ik._step_configuration(
        configuration=_FakeConfiguration(context.model, context.data, seed_q),
        tasks={},
        dt=0.05,
        constraints=constraints,
    )

    assert solve_ik.call_args.kwargs["constraints"] == constraints


def test_create_kinematics_pink_config_passes_tuning(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)

    ik = create_kinematics(config=PinkKinematicsConfig(max_iterations=7, dt=0.02, posture_cost=0.0))

    assert isinstance(ik, PinkIK)
    assert ik.config.max_iterations == 7
    assert ik.config.dt == 0.02
    assert ik.config.posture_cost == 0.0


def test_pink_ik_config_overrides_are_applied(mocker: MockerFixture) -> None:
    _install_fake_modules(mocker)

    ik = PinkIK(PinkIKConfig(solver="proxqp", dt=0.1), max_iterations=7, posture_cost=0.0)

    assert ik.config == PinkIKConfig(
        solver="proxqp",
        dt=0.1,
        max_iterations=7,
        posture_cost=0.0,
    )


def test_joint_order_mapping_uses_names_not_positions() -> None:
    mapping = _build_joint_mapping(_FakeModel(), _test_joint_space())
    seed = JointState(name=["joint_b", "joint_c", "joint_a"], position=[20.0, 30.0, 10.0])

    assert mapping.idx_q == [1, 0, 2]
    assert mapping.idx_v == [1, 0, 2]
    assert _seed_positions_for_mapping(seed, mapping).tolist() == [10.0, 20.0, 30.0]


def test_planar_mapping_converts_continuous_yaw_to_public_scalar() -> None:
    planar = PlanarBaseDefinition(
        velocity_limits=(1.0, 1.0, 2.0),
        acceleration_limits=(2.0, 2.0, 4.0),
    )
    joint_space = JointSpace(
        (
            JointCoordinate(
                planar.joint_names[0], "prismatic", CoordinateTopology.LINE, None, None, 1.0, 2.0
            ),
            JointCoordinate(
                planar.joint_names[1], "prismatic", CoordinateTopology.LINE, None, None, 1.0, 2.0
            ),
            JointCoordinate(
                planar.joint_names[2], "continuous", CoordinateTopology.CIRCLE, None, None, 2.0, 4.0
            ),
        )
    )
    mapping = _build_joint_mapping(cast("Any", _FakePlanarModel()), joint_space)
    q = np.zeros(4)

    for index, value in enumerate((10.0, -4.0, np.pi + 0.2)):
        _write_dimos_position(q, mapping, index, value)

    assert mapping.periodic_indices == (2,)
    assert mapping.translation_indices == (0, 1)
    assert q == pytest.approx([10.0, -4.0, -np.cos(0.2), -np.sin(0.2)])
    assert [_read_dimos_position(q, mapping, index) for index in range(3)] == pytest.approx(
        [10.0, -4.0, -np.pi + 0.2]
    )


def test_planar_retry_sampling_uses_finite_expanding_translation_window() -> None:
    planar = PlanarBaseDefinition(
        velocity_limits=(1.0, 1.0, 2.0),
        acceleration_limits=(2.0, 2.0, 4.0),
    )
    joint_space = JointSpace(
        (
            JointCoordinate(
                planar.joint_names[0], "prismatic", CoordinateTopology.LINE, None, None, 1.0, 2.0
            ),
            JointCoordinate(
                planar.joint_names[1], "prismatic", CoordinateTopology.LINE, None, None, 1.0, 2.0
            ),
            JointCoordinate(
                planar.joint_names[2], "continuous", CoordinateTopology.CIRCLE, None, None, 2.0, 4.0
            ),
        )
    )
    lower, upper = _finite_retry_limits(
        joint_space,
        np.array([100.0, -50.0, 0.0]),
        np.full(3, -np.inf),
        np.full(3, np.inf),
        range(3),
        attempt=3,
    )

    assert lower == pytest.approx([96.0, -54.0, -np.pi])
    assert upper == pytest.approx([104.0, -46.0, np.pi])


def test_streaming_envelope_intersects_configured_and_urdf_velocity(
    mocker: MockerFixture,
) -> None:
    ik = _streaming_ik(mocker)
    context = _context()

    result = ik._apply_streaming_command_envelope(
        context=context,
        candidate=np.ones(3),
        previous=np.zeros(3),
        measured=np.zeros(3),
        dt=0.1,
        max_joint_velocity=3.0,
        joint_velocity_limits={"joint_a": 0.5, "joint_b": 1.0},
        max_tracking_error=1.0,
        command_limit_margin=1e-4,
    )

    assert result == pytest.approx([0.05, 0.1, 0.3])


def test_streaming_envelope_caps_command_at_measured_tracking_distance(
    mocker: MockerFixture,
) -> None:
    ik = _streaming_ik(mocker)
    context = _context()

    result = ik._apply_streaming_command_envelope(
        context=context,
        candidate=np.full(3, 0.5),
        previous=np.full(3, 0.1),
        measured=np.zeros(3),
        dt=0.1,
        max_joint_velocity=5.0,
        joint_velocity_limits={},
        max_tracking_error=0.15,
        command_limit_margin=1e-4,
    )

    assert result == pytest.approx([0.15, 0.15, 0.15])


def test_step_frame_targets_weighted_history_attenuates_alternating_increments(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    ik = _StreamingTestPinkIK(PinkIKConfig())
    context = _combined_control_context(("tool",), ["joint_a"])
    context.robot.model.velocityLimit[:] = 100.0
    context.robot.model.lowerPositionLimit[:] = -100.0
    context.robot.model.upperPositionLimit[:] = 100.0
    mocker.patch.object(ik, "_get_control_context", return_value=context)
    increments = iter((1.0, -1.0, 1.0, -1.0))

    def apply_increment(**kwargs: Any) -> None:
        configuration = kwargs["configuration"]
        q = configuration.q.copy()
        q[context.robot.mapping.idx_q[0]] += next(increments)
        configuration.update(q)

    mocker.patch.object(ik, "_step_configuration", side_effect=apply_increment)
    initial = JointState(name=["joint_a"], position=[0.0])
    alpha = 1.0 - np.exp(-2.0 * np.pi * 5.0 * 0.01)
    commands = [initial]

    for _ in range(4):
        commands.append(
            ik.step_frame_targets(
                robot_model=_robot_config(),
                frame_targets={"tool": PoseStamped()},
                controlled_joints=["joint_a"],
                command_state=commands[-1],
                measured_state=initial,
                max_command_tracking_error_rad=1.0,
                dt=0.01,
                max_joint_velocity_rad_s=100.0,
                joint_command_filter_cutoff_hz=5.0,
            )
        )

    accepted_increments = np.diff([command.position[0] for command in commands])
    assert accepted_increments == pytest.approx([alpha, -alpha / 3.0, 0.4 * alpha, -0.4 * alpha])


def test_step_frame_targets_weighted_history_preserves_steady_increment(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    ik = _StreamingTestPinkIK(PinkIKConfig())
    context = _combined_control_context(("tool",), ["joint_a"])
    context.robot.model.velocityLimit[:] = 100.0
    context.robot.model.lowerPositionLimit[:] = -100.0
    context.robot.model.upperPositionLimit[:] = 100.0
    mocker.patch.object(ik, "_get_control_context", return_value=context)

    def apply_increment(**kwargs: Any) -> None:
        configuration = kwargs["configuration"]
        q = configuration.q.copy()
        q[context.robot.mapping.idx_q[0]] += 1.0
        configuration.update(q)

    mocker.patch.object(ik, "_step_configuration", side_effect=apply_increment)
    initial = JointState(name=["joint_a"], position=[0.0])
    alpha = 1.0 - np.exp(-2.0 * np.pi * 5.0 * 0.01)
    commands = [initial]

    for _ in range(4):
        commands.append(
            ik.step_frame_targets(
                robot_model=_robot_config(),
                frame_targets={"tool": PoseStamped()},
                controlled_joints=["joint_a"],
                command_state=commands[-1],
                measured_state=initial,
                max_command_tracking_error_rad=2.0,
                dt=0.01,
                max_joint_velocity_rad_s=100.0,
                joint_command_filter_cutoff_hz=5.0,
            )
        )

    assert np.diff([command.position[0] for command in commands]) == pytest.approx(
        np.full(4, alpha)
    )


def test_step_frame_targets_rechecks_tracking_envelope_after_history_filter(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    ik = _StreamingTestPinkIK(PinkIKConfig())
    context = _combined_control_context(("tool",), ["joint_a"])
    context.robot.model.velocityLimit[:] = 100.0
    context.robot.model.lowerPositionLimit[:] = -100.0
    context.robot.model.upperPositionLimit[:] = 100.0
    mocker.patch.object(ik, "_get_control_context", return_value=context)

    def apply_increment(**kwargs: Any) -> None:
        configuration = kwargs["configuration"]
        q = configuration.q.copy()
        q[context.robot.mapping.idx_q[0]] += 1.0
        configuration.update(q)

    mocker.patch.object(ik, "_step_configuration", side_effect=apply_increment)
    initial = JointState(name=["joint_a"], position=[0.0])
    first = ik.step_frame_targets(
        robot_model=_robot_config(),
        frame_targets={"tool": PoseStamped()},
        controlled_joints=["joint_a"],
        command_state=initial,
        measured_state=initial,
        max_command_tracking_error_rad=0.2,
        dt=0.01,
        max_joint_velocity_rad_s=100.0,
        joint_command_filter_cutoff_hz=5.0,
    )

    second = ik.step_frame_targets(
        robot_model=_robot_config(),
        frame_targets={"tool": PoseStamped()},
        controlled_joints=["joint_a"],
        command_state=first,
        measured_state=JointState(name=["joint_a"], position=[-0.1]),
        max_command_tracking_error_rad=0.15,
        dt=0.01,
        max_joint_velocity_rad_s=100.0,
        joint_command_filter_cutoff_hz=5.0,
    )

    assert first.position == pytest.approx([0.2])
    assert second.position == pytest.approx([0.05])


def test_step_frame_targets_preserves_controlled_joint_order(
    mocker: MockerFixture,
) -> None:
    ik = _streaming_ik(mocker)
    mocker.patch.object(
        ik,
        "_get_control_context",
        return_value=_combined_control_context(("tool",), ["joint_c", "joint_a"]),
    )

    result = ik.step_frame_targets(
        robot_model=_robot_config(),
        frame_targets={
            "tool": PoseStamped(
                position=Vector3(0.1, 0.2, 0.3),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            )
        },
        controlled_joints=["joint_c", "joint_a"],
        command_state=JointState(name=["joint_a", "joint_c"], position=[0.0, 0.0]),
        measured_state=JointState(name=["joint_a", "joint_c"], position=[0.0, 0.0]),
        max_command_tracking_error_rad=_TRACKING_ERROR_RAD,
        dt=0.02,
    )

    assert result.name == ["joint_c", "joint_a"]
    assert result.position == pytest.approx([0.1, 0.08])


def test_step_frame_targets_builds_both_frame_tasks_with_tuning(
    mocker: MockerFixture,
) -> None:
    config = PinkIKConfig(
        position_cost=2.0,
        orientation_cost=3.0,
        lm_damping=4e-6,
        gain=0.25,
        posture_cost=0.0,
    )
    _install_fake_modules(mocker)
    ik = _StreamingTestPinkIK(config)
    mocker.patch.object(
        ik,
        "_get_control_context",
        return_value=_combined_control_context(("tool", "base"), ["joint_a", "joint_b", "joint_c"]),
    )
    frame_task = mocker.patch.object(pink_ik.pink.tasks, "FrameTask", wraps=_FakeFrameTask)
    solve_ik = mocker.spy(pink_ik.pink, "solve_ik")

    result = ik.step_frame_targets(
        robot_model=_robot_config(),
        frame_targets={
            "tool": PoseStamped(
                position=Vector3(0.1, 0.2, 0.3),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
            "base": PoseStamped(
                position=Vector3(),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        },
        controlled_joints=["joint_a", "joint_b", "joint_c"],
        command_state=JointState(name=["joint_a", "joint_b", "joint_c"], position=[0.0, 0.0, 0.0]),
        measured_state=JointState(name=["joint_a", "joint_b", "joint_c"], position=[0.0, 0.0, 0.0]),
        max_command_tracking_error_rad=_TRACKING_ERROR_RAD,
        dt=0.02,
    )

    assert [call.args[0] for call in frame_task.call_args_list] == ["tool", "base"]
    assert frame_task.call_args_list[0].kwargs == {
        "position_cost": 2.0,
        "orientation_cost": 3.0,
        "lm_damping": 4e-6,
        "gain": 0.25,
    }
    solve_ik.assert_called_once()
    assert solve_ik.call_args.args[2] == 0.02
    assert result.name == ["joint_a", "joint_b", "joint_c"]


def test_named_task_stack_supports_incremental_subclass_composition(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    context = _combined_control_context(("tool", "base"), ["joint_a", "joint_b", "joint_c"])
    configuration = _FakeConfiguration(context.robot.model, context.robot.data, np.zeros(3))
    ik = _DerivedComposablePinkIK(PinkIKConfig())

    tasks = ik._build_task_stack(configuration, ("tool", "base"))

    assert list(tasks) == [
        "frame/tool",
        "frame/base",
        "posture/nominal",
        "auxiliary/damping",
    ]
    assert tasks["frame/tool"].customized is True
    assert tasks["posture/nominal"].value == 1.0
    assert tasks["auxiliary/damping"].value == 3.0


@pytest.mark.parametrize(
    ("ik_type", "match"),
    [
        (_MissingFramePinkIK, "frame/tool.*frame-target task"),
        (_MismatchedFramePinkIK, "targets frame 'base'.*expected 'tool'"),
    ],
)
def test_named_task_stack_rejects_invalid_reserved_frame_tasks(
    mocker: MockerFixture,
    ik_type: type[PinkIK],
    match: str,
) -> None:
    _install_fake_modules(mocker)
    context = _combined_control_context(("tool",), ["joint_a", "joint_b", "joint_c"])
    configuration = _FakeConfiguration(context.robot.model, context.robot.data, np.zeros(3))

    with pytest.raises(ValueError, match=match):
        ik_type(PinkIKConfig())._build_task_stack(configuration, ("tool",))


def test_streaming_reuses_task_stack_across_steps(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    ik = _StreamingTestPinkIK(PinkIKConfig())
    context = _combined_control_context(("tool",), ["joint_a", "joint_b", "joint_c"])
    mocker.patch.object(ik, "_get_control_context", return_value=context)
    create_tasks = mocker.spy(ik, "_create_tasks")
    first_target = PoseStamped(
        position=Vector3(0.1, 0.2, 0.3),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )
    second_target = PoseStamped(
        position=Vector3(0.3, 0.2, 0.1),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    ik.step_frame_targets(
        _robot_config(),
        {"tool": first_target},
        ["joint_a", "joint_b", "joint_c"],
        JointState(name=["joint_a", "joint_b", "joint_c"], position=[0.0, 0.0, 0.0]),
        JointState(name=["joint_a", "joint_b", "joint_c"], position=[0.0, 0.0, 0.0]),
        _TRACKING_ERROR_RAD,
    )
    assert context.tasks is not None

    ik.step_frame_targets(
        _robot_config(),
        {"tool": second_target},
        ["joint_a", "joint_b", "joint_c"],
        JointState(name=["joint_a", "joint_b", "joint_c"], position=[0.1, 0.2, 0.3]),
        JointState(name=["joint_a", "joint_b", "joint_c"], position=[0.1, 0.2, 0.3]),
        _TRACKING_ERROR_RAD,
    )

    create_tasks.assert_called_once()
    assert context.tasks["frame/tool"].target.translation == pytest.approx([0.3, 0.2, 0.1])
    assert context.tasks[_CURRENT_POSTURE_TASK].target == pytest.approx([0.2, 0.1, 0.3])


def test_task_hooks_receive_read_only_stack_and_successful_velocity(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    ik = _RecordingStreamingPinkIK(PinkIKConfig(posture_cost=0.0))
    context = _combined_control_context(("tool",), ["joint_a", "joint_b", "joint_c"])
    mocker.patch.object(ik, "_get_control_context", return_value=context)

    ik.step_frame_targets(
        _robot_config(),
        {
            "tool": PoseStamped(
                position=Vector3(0.1, 0.2, 0.3),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            )
        },
        ["joint_a", "joint_b", "joint_c"],
        JointState(name=["joint_a", "joint_b", "joint_c"], position=[0.0, 0.0, 0.0]),
        JointState(name=["joint_a", "joint_b", "joint_c"], position=[0.0, 0.0, 0.0]),
        _TRACKING_ERROR_RAD,
        dt=0.05,
    )

    assert len(ik.before_tasks) == 1
    with pytest.raises(TypeError):
        cast("dict[str, Any]", ik.before_tasks[0])["auxiliary/new"] = _AuxiliaryTask(1.0)
    assert ik.after_velocities[0] == pytest.approx([2.0, 4.0, 6.0])


def test_after_solve_hook_is_not_called_when_solver_raises(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    ik = _RecordingStreamingPinkIK(PinkIKConfig(posture_cost=0.0))
    context = _combined_control_context(("tool",), ["joint_a", "joint_b", "joint_c"])
    mocker.patch.object(ik, "_get_control_context", return_value=context)
    mocker.patch.object(pink_ik.pink, "solve_ik", side_effect=RuntimeError("no solution"))

    with pytest.raises(RuntimeError, match="no solution"):
        ik.step_frame_targets(
            _robot_config(),
            {"tool": PoseStamped()},
            ["joint_a", "joint_b", "joint_c"],
            JointState(
                name=["joint_a", "joint_b", "joint_c"],
                position=[0.0, 0.0, 0.0],
            ),
            JointState(
                name=["joint_a", "joint_b", "joint_c"],
                position=[0.0, 0.0, 0.0],
            ),
            _TRACKING_ERROR_RAD,
        )

    assert len(ik.before_tasks) == 1
    assert ik.after_velocities == []


def test_separate_backends_do_not_share_task_instances(mocker: MockerFixture) -> None:
    _install_fake_modules(mocker)
    first = PinkIK(PinkIKConfig())
    second = PinkIK(PinkIKConfig())
    first_context = _combined_control_context(("tool",), ["joint_a", "joint_b", "joint_c"])
    second_context = _combined_control_context(("tool",), ["joint_a", "joint_b", "joint_c"])
    first_configuration = _FakeConfiguration(
        first_context.robot.model, first_context.robot.data, np.zeros(3)
    )
    second_configuration = _FakeConfiguration(
        second_context.robot.model, second_context.robot.data, np.zeros(3)
    )

    first_tasks = first._build_task_stack(first_configuration, ("tool",))
    second_tasks = second._build_task_stack(second_configuration, ("tool",))

    assert first_tasks["frame/tool"] is not second_tasks["frame/tool"]
    assert first_tasks[_CURRENT_POSTURE_TASK] is not second_tasks[_CURRENT_POSTURE_TASK]


def test_step_frame_targets_normalizes_feedback_and_saturates_commands(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    ik = _StreamingTestPinkIK(PinkIKConfig())
    context = _combined_control_context(("tool",), ["joint_a", "joint_b", "joint_c"])
    mocker.patch.object(ik, "_get_control_context", return_value=context)

    def step_outside_limits(**kwargs: Any) -> None:
        configuration = kwargs["configuration"]
        assert configuration.q == pytest.approx([0.0, -0.9999, 0.0])
        configuration.update(np.array([1.5, -1.5, 0.25]))

    mocker.patch.object(ik, "_step_configuration", side_effect=step_outside_limits)

    result = ik.step_frame_targets(
        robot_model=_robot_config(),
        frame_targets={"tool": PoseStamped()},
        controlled_joints=["joint_a", "joint_b", "joint_c"],
        command_state=JointState(
            name=["joint_a", "joint_b", "joint_c"],
            position=[-1.0005, 0.0, 0.0],
        ),
        measured_state=JointState(
            name=["joint_a", "joint_b", "joint_c"],
            position=[-1.0005, 0.0, 0.0],
        ),
        max_command_tracking_error_rad=_TRACKING_ERROR_RAD,
    )

    assert result.position == pytest.approx([-0.9999, 0.1, _TRACKING_ERROR_RAD])


def test_step_frame_targets_rejects_feedback_beyond_tolerance(
    mocker: MockerFixture,
) -> None:
    ik = _streaming_ik(mocker)
    mocker.patch.object(
        ik,
        "_get_control_context",
        return_value=_combined_control_context(("tool",), ["joint_a"]),
    )
    step = mocker.patch.object(ik, "_step_configuration")

    with pytest.raises(PinkJointLimitError, match="joint_a.*lower limit"):
        ik.step_frame_targets(
            robot_model=_robot_config(),
            frame_targets={"tool": PoseStamped()},
            controlled_joints=["joint_a"],
            command_state=JointState(name=["joint_a"], position=[0.0]),
            measured_state=JointState(name=["joint_a"], position=[-1.0011]),
            max_command_tracking_error_rad=_TRACKING_ERROR_RAD,
        )

    step.assert_not_called()


def test_step_frame_targets_velocity_limits_unbounded_position_joint(
    mocker: MockerFixture,
) -> None:
    ik = _streaming_ik(mocker)
    context = _combined_control_context(("tool",), ["joint_b"])
    context.robot.model.lowerPositionLimit[0] = -np.inf
    context.robot.model.upperPositionLimit[0] = np.inf
    mocker.patch.object(ik, "_get_control_context", return_value=context)

    def step_unbounded(**kwargs: Any) -> None:
        configuration = kwargs["configuration"]
        assert configuration.q[0] == 5.0
        configuration.update(np.array([6.0, 0.0, 0.0]))

    mocker.patch.object(ik, "_step_configuration", side_effect=step_unbounded)

    result = ik.step_frame_targets(
        robot_model=_robot_config(),
        frame_targets={"tool": PoseStamped()},
        controlled_joints=["joint_b"],
        command_state=JointState(name=["joint_b"], position=[5.0]),
        measured_state=JointState(name=["joint_b"], position=[5.0]),
        max_command_tracking_error_rad=_TRACKING_ERROR_RAD,
    )

    assert result.position == pytest.approx([5.1])


def test_validate_frame_targets_rejects_margin_wider_than_joint_range(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    ik = _StreamingTestPinkIK(PinkIKConfig())
    ik.command_limit_margin = 1.1
    mocker.patch.object(
        ik,
        "_get_control_context",
        return_value=_combined_control_context(("tool",), ["joint_a"]),
    )

    with pytest.raises(ValueError, match="command limit margin.*joint_a"):
        ik.validate_frame_targets(_robot_config(), ["tool"], ["joint_a"])


def test_step_frame_targets_rejects_unknown_frame(mocker: MockerFixture, tmp_path: Path) -> None:
    modules = _install_fake_modules(mocker)
    modules.pinocchio.buildModelFromXML = lambda xml: _FakeModel()  # type: ignore[attr-defined]
    config = _robot_config()
    model_path = tmp_path / "fake.urdf"
    model_path.write_text("<robot/>")
    config.model = RobotModel.from_file(model_path)
    mocker.patch.object(
        pose_target_ik,
        "prepare_robot_model",
        return_value=PreparedRobotModel(
            config=config,
            description=config.model.load(),
            joint_space=_test_joint_space(),
            planning_groups=(),
        ),
    )

    with pytest.raises(ValueError, match="missing_frame"):
        _StreamingTestPinkIK(PinkIKConfig()).step_frame_targets(
            robot_model=config,
            frame_targets={"missing_frame": PoseStamped()},
            controlled_joints=config.joint_names,
            command_state=JointState(name=config.joint_names, position=[0.0, 0.0, 0.0]),
            measured_state=JointState(name=config.joint_names, position=[0.0, 0.0, 0.0]),
            max_command_tracking_error_rad=_TRACKING_ERROR_RAD,
        )


def test_mapping_failure_for_missing_joint() -> None:
    config = _robot_config()
    config.joint_names = ["joint_a", "missing", "joint_c"]

    with pytest.raises(KeyError, match="missing"):
        _build_joint_mapping(_FakeModel(), _test_joint_space().select(tuple(config.joint_names)))


def test_solve_targets_returns_successful_ik_result(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    target = np.eye(4)
    target[:3, 3] = [0.1, 0.2, 0.3]

    result = ik._solve_targets(
        targets=[(_context(), target)],
        seed_q=np.zeros(3),
        lower_limits=np.array([-1.0, -1.0, -1.0]),
        upper_limits=np.array([1.0, 1.0, 1.0]),
        position_tolerance=0.001,
        orientation_tolerance=0.01,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["joint_a", "joint_b", "joint_c"]
    assert result.joint_state.position == pytest.approx([0.2, 0.1, 0.3])


def test_planning_uses_named_stack_and_task_lifecycle_hooks(
    mocker: MockerFixture,
) -> None:
    _install_fake_modules(mocker)
    ik = _RecordingPinkIK(PinkIKConfig(max_iterations=3))
    target = np.eye(4)
    target[:3, 3] = [0.1, 0.2, 0.3]

    result = ik._solve_targets(
        targets=[(_context(), target)],
        seed_q=np.zeros(3),
        lower_limits=np.array([-1.0, -1.0, -1.0]),
        upper_limits=np.array([1.0, 1.0, 1.0]),
        position_tolerance=0.001,
        orientation_tolerance=0.01,
    )

    assert result.status == IKStatus.SUCCESS
    assert len(ik.before_tasks) == 1
    assert list(ik.before_tasks[0]) == ["frame/tool", _CURRENT_POSTURE_TASK]
    assert ik.after_velocities[0] == pytest.approx([2.0, 4.0, 6.0])


def test_solve_targets_reports_non_convergence(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=False)
    target = np.eye(4)
    target[:3, 3] = [0.1, 0.0, 0.0]

    result = ik._solve_targets(
        targets=[(_context(), target)],
        seed_q=np.zeros(3),
        lower_limits=np.array([-1.0, -1.0, -1.0]),
        upper_limits=np.array([1.0, 1.0, 1.0]),
        position_tolerance=0.001,
        orientation_tolerance=0.01,
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert "did not converge" in result.message


def test_pose_target_solve_constrains_joints_outside_planning_group(tmp_path: Path) -> None:
    model_path = tmp_path / "locked_waist_chain.urdf"
    model_path.write_text(_LOCKED_WAIST_CHAIN_URDF)
    joint_names = [
        "waist_yaw",
        "waist_roll",
        "waist_pitch",
        "shoulder_pitch",
        "shoulder_roll",
        "shoulder_yaw",
        "elbow",
        "wrist_roll",
        "wrist_pitch",
        "wrist_yaw",
    ]
    arm_names = joint_names[3:]
    config = RobotModelConfig(
        model=RobotModel.from_file(model_path).with_default_joint_acceleration_limit(2.0),
        joint_names=joint_names,
        base_link="pelvis",
        planning_groups=[
            PlanningGroupDefinition(
                name="arm",
                joint_names=tuple(arm_names),
                base_link="pelvis",
                tip_link="tool",
            )
        ],
    )
    group = PlanningGroup(
        id="arm",
        joint_names=tuple(arm_names),
        base_link="pelvis",
        tip_link="tool",
    )
    seed_positions = np.array([0.0, 0.0, 0.0, -0.4, 0.2, 0.0, 1.2, 0.0, 0.0, 0.0])
    seed = JointState(name=joint_names, position=seed_positions.tolist())
    prepared = prepare_robot_model(config)

    class World:
        is_finalized = True

        def get_prepared_model(self) -> PreparedRobotModel:
            return prepared

        def scratch_context(self) -> nullcontext[None]:
            return nullcontext(None)

        def get_joint_state(self, ctx: object) -> JointState:
            return seed

        def check_config_collision_free(self, joint_state: JointState) -> bool:
            return True

        def set_joint_state(self, ctx: object, joint_state: JointState) -> None:
            pass

        def is_collision_free(self, ctx: object) -> bool:
            return True

    ik = PinkIK(PinkIKConfig(max_iterations=100))
    context = ik._build_robot_context(prepared, "tool")
    target_positions = seed_positions.copy()
    target_positions[3:] += np.array(
        [0.05003819, 0.15888552, 0.11027428, -0.10991712, -0.07993349, 0.14942138, -0.19789388]
    )
    target_q = ik._q_from_dimos_positions(context, target_positions)
    target_pose = matrix_to_pose(ik._current_frame_matrix(context, target_q))

    result = ik.solve_pose_targets(
        cast("Any", World()),
        {group: PoseStamped(position=target_pose.position, orientation=target_pose.orientation)},
        seed=seed,
        check_collision=False,
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == arm_names
    assert result.position_error <= 0.001
    assert result.orientation_error <= 0.01


def test_solve_rejects_collision_candidate(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    context = _context()
    ik._model_context = context

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=False)),
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=1,
    )

    assert result.status == IKStatus.COLLISION
    assert result.joint_state is None


def test_solve_retries_after_joint_limit_failure(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    context = _context()
    ik._model_context = context
    calls = 0

    def fake_solve_targets(**_: object) -> IKResult:
        nonlocal calls
        calls += 1
        if calls == 1:
            return IKResult(
                status=IKStatus.JOINT_LIMITS,
                joint_state=None,
                message="first attempt hit limits",
            )
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                name=["joint_a", "joint_b", "joint_c"],
                position=[0.1, 0.2, 0.3],
            ),
            position_error=0.0,
            orientation_error=0.0,
            iterations=1,
        )

    solve_targets = mocker.patch.object(ik, "_solve_targets", side_effect=fake_solve_targets)

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=2,
    )

    assert solve_targets.call_count == 2
    assert result.status == IKStatus.SUCCESS


def test_model_context_reuses_model_for_another_tip_frame(
    mocker: MockerFixture, tmp_path: Path
) -> None:
    modules = _install_fake_modules(mocker)
    modules.pinocchio.buildModelFromXML = lambda xml: _FakeModel()  # type: ignore[attr-defined]
    model_path = tmp_path / "fake.urdf"
    model_path.write_text("<robot/>")
    world = _FakeWorld()
    world.config.model = RobotModel.from_file(model_path)
    ik = PinkIK(PinkIKConfig(max_iterations=1))

    first = ik._get_model_context(cast("Any", world), "tool")
    second = ik._get_model_context(cast("Any", world), "base")

    assert first is not second
    assert first.model is second.model
    assert first.data is second.data
    assert first.mapping is second.mapping
    assert second.frame_name == "base"


def test_build_robot_context_rejects_base_link_not_model_root(
    mocker: MockerFixture, tmp_path: Path
) -> None:
    model = _FakeModel()
    model.frames[0] = _FakeFrame("base", parent_joint=1)
    modules = _install_fake_modules(mocker)
    modules.pinocchio.buildModelFromXML = lambda xml: model  # type: ignore[attr-defined]
    model_path = tmp_path / "fake.urdf"
    model_path.write_text("<robot/>")
    config = _robot_config()
    config.model = RobotModel.from_file(model_path)

    with pytest.raises(ValueError, match="base_link 'base'.*model root"):
        PinkIK(PinkIKConfig(max_iterations=1))._build_robot_context(
            PreparedRobotModel(
                config=config,
                description=config.model.load(),
                joint_space=_test_joint_space(),
                planning_groups=(),
            ),
            "tool",
        )


def test_solve_pose_targets_uses_group_tip_and_filters_group_joints(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    context = _context()
    get_context = mocker.patch.object(ik, "_get_model_context", return_value=context)
    mocker.patch.object(
        ik,
        "_solve_targets",
        return_value=IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}
            ),
        ),
    )
    world = _FakeWorld()

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.0, 0.0, 0.0]}),
        max_attempts=1,
    )

    get_context.assert_called_once_with(cast("Any", world), "tool")
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["joint_a", "joint_b"]
    assert result.joint_state.position == [0.1, 0.2]
    assert world.joint_state_calls == 0


def test_solve_pose_targets_rejects_group_without_tip(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker)
    world = _FakeWorld()

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["no_tip"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
    )

    assert result.status == IKStatus.UNSUPPORTED
    assert "no pose target frame" in result.message


def test_solve_pose_targets_partial_seed_reads_world_state(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    mocker.patch.object(
        ik,
        "_solve_targets",
        return_value=IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}
            ),
        ),
    )
    world = _FakeWorld()

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=JointState({"name": ["joint_a"], "position": [0.0]}),
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert world.joint_state_calls == 1


def test_solve_pose_targets_multi_target_uses_multi_frame_solve(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker)
    world = _FakeWorld()
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    solve_targets = mocker.patch.object(
        ik,
        "_solve_targets",
        return_value=IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}
            ),
            position_error=0.0,
            orientation_error=0.0,
        ),
    )

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
            world.groups["wrist"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
        },
        seed=JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.0, 0.0, 0.0]}),
        max_attempts=1,
    )

    solve_targets.assert_called_once()
    assert len(solve_targets.call_args.kwargs["targets"]) == 2
    assert result.joint_state is not None
    assert result.joint_state.name == ["joint_a", "joint_b", "joint_c"]
    assert result.joint_state.position == [0.1, 0.2, 0.3]


def test_solve_pose_targets_checks_multi_group_solution_together(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker)
    world = _FakeWorld(collision_free=False)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    solve_targets = mocker.patch.object(
        ik,
        "_solve_targets",
        return_value=IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}
            ),
        ),
    )

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
            world.groups["wrist"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
        },
        seed=JointState(
            {
                "name": ["joint_a", "joint_b", "joint_c"],
                "position": [0.0, 0.0, 0.0],
            }
        ),
        max_attempts=1,
    )

    solve_targets.assert_called_once()
    assert len(solve_targets.call_args.kwargs["targets"]) == 2
    assert result.status == IKStatus.COLLISION


def test_solve_pose_targets_auxiliary_only_retains_seed_selection_order(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker)
    world = _FakeWorld()

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={},
        auxiliary_groups=[world.groups["no_tip"], world.groups["manipulator"]],
        seed=JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}),
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["joint_c", "joint_a", "joint_b"]
    assert result.joint_state.position == [0.3, 0.1, 0.2]


def _solved_joint_state() -> JointState:
    return JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]})


def _qp_infeasible() -> NoSolutionFound:
    return NoSolutionFound(
        problem=cast("Any", SimpleNamespace()),
        results=cast("Any", SimpleNamespace()),
    )


def test_solve_retries_after_no_solution_found(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    outcomes: list[object] = [
        _qp_infeasible(),
        IKResult(
            status=IKStatus.SUCCESS,
            joint_state=_solved_joint_state(),
            position_error=0.0006,
            orientation_error=0.0,
            iterations=1,
        ),
    ]

    def fake_solve_targets(**_: object) -> IKResult:
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return cast("IKResult", outcome)

    solve_targets = mocker.patch.object(ik, "_solve_targets", side_effect=fake_solve_targets)

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=2,
    )

    assert solve_targets.call_count == 2
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None


def test_solve_reports_first_no_solution_found_after_all_attempts(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())

    def fake_solve_targets(**_: object) -> IKResult:
        raise _qp_infeasible()

    solve_targets = mocker.patch.object(ik, "_solve_targets", side_effect=fake_solve_targets)

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=3,
    )

    assert solve_targets.call_count == 3
    assert result.status == IKStatus.NO_SOLUTION
    assert "QP solver did not find a solution" in result.message


def test_solve_does_not_retry_unexpected_exception(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    solve_targets = mocker.patch.object(
        ik, "_solve_targets", side_effect=RuntimeError("unexpected solver failure")
    )

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=3,
    )

    assert solve_targets.call_count == 1
    assert result.status == IKStatus.NO_SOLUTION
    assert result.message == "Pink IK solver failed: unexpected solver failure"


def test_solve_mapping_value_error_fails_without_retrying(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    solve_targets = mocker.patch.object(
        ik, "_solve_targets", side_effect=ValueError("joint 'joint_z' is not in the model")
    )

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=5,
    )

    assert solve_targets.call_count == 1
    assert result.status == IKStatus.NO_SOLUTION
    assert "Pink IK mapping failed" in result.message


def _pose_targets_seed() -> JointState:
    return JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.0, 0.0, 0.0]})


def test_solve_pose_targets_retries_after_no_solution_found(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    world = _FakeWorld()
    outcomes: list[object] = [
        _qp_infeasible(),
        IKResult(
            status=IKStatus.SUCCESS,
            joint_state=_solved_joint_state(),
            position_error=0.0006,
            orientation_error=0.0,
        ),
    ]

    def fake_solve_targets(**_: object) -> IKResult:
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return cast("IKResult", outcome)

    solve_targets = mocker.patch.object(ik, "_solve_targets", side_effect=fake_solve_targets)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=_pose_targets_seed(),
        max_attempts=2,
    )

    assert solve_targets.call_count == 2
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None


def test_solve_pose_targets_reports_first_no_solution_found_after_all_attempts(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    world = _FakeWorld()

    def fake_solve_targets(**_: object) -> IKResult:
        raise _qp_infeasible()

    solve_targets = mocker.patch.object(ik, "_solve_targets", side_effect=fake_solve_targets)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=_pose_targets_seed(),
        max_attempts=3,
    )

    assert solve_targets.call_count == 3
    assert result.status == IKStatus.NO_SOLUTION
    assert "QP solver did not find a solution" in result.message


def test_solve_pose_targets_does_not_retry_unexpected_exception(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    world = _FakeWorld()
    solve_targets = mocker.patch.object(
        ik, "_solve_targets", side_effect=RuntimeError("unexpected solver failure")
    )

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=_pose_targets_seed(),
        max_attempts=3,
    )

    assert solve_targets.call_count == 1
    assert result.status == IKStatus.NO_SOLUTION
    assert result.message == "Pink IK solver failed: unexpected solver failure"


def test_solve_pose_targets_mapping_value_error_fails_without_retrying(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    world = _FakeWorld()
    solve_targets = mocker.patch.object(
        ik, "_solve_targets", side_effect=ValueError("joint 'joint_z' is not in the model")
    )

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=_pose_targets_seed(),
        max_attempts=5,
    )

    assert solve_targets.call_count == 1
    assert result.status == IKStatus.NO_SOLUTION
    assert "Pink IK mapping failed" in result.message
    assert world.joint_state_calls == 0
